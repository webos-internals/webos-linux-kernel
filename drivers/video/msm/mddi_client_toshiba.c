/* drivers/video/msm_fb/mddi_client_toshiba.c
 *
 * Support for Toshiba TC358720XBG mddi client devices which require no
 * special initialization code.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/gpio.h>
#include <asm/arch/msm_fb.h>

static DECLARE_WAIT_QUEUE_HEAD(toshiba_vsync_wait);
static volatile int toshiba_got_int;
static struct msmfb_callback *toshiba_callback;

static void dummy_function(struct mddi_panel_info *panel)
{
}

#define LCD_CONTROL_BLOCK_BASE 0x110000
#define	CMN         (LCD_CONTROL_BLOCK_BASE|0x10)
#define	INTFLG      (LCD_CONTROL_BLOCK_BASE|0x18)
#define HCYCLE      (LCD_CONTROL_BLOCK_BASE|0x34)
#define	HDE_START   (LCD_CONTROL_BLOCK_BASE|0x3C)
#define VPOS        (LCD_CONTROL_BLOCK_BASE|0xC0)
#define MPLFBUF     (LCD_CONTROL_BLOCK_BASE|0x20)
#define	WAKEUP      (LCD_CONTROL_BLOCK_BASE|0x54)
#define	WSYN_DLY    (LCD_CONTROL_BLOCK_BASE|0x58)
#define REGENB      (LCD_CONTROL_BLOCK_BASE|0x5C)

#define BASE5 0x150000
#define BASE6 0x160000
#define BASE7 0x170000

#define GPIOIEV     (BASE5 + 0x10)
#define GPIOIE      (BASE5 + 0x14)
#define GPIORIS     (BASE5 + 0x18)
#define GPIOMIS     (BASE5 + 0x1C)
#define GPIOIC      (BASE5 + 0x20)

#define INTMASK     (BASE6 + 0x0C)
#define INTMASK_VWAKEOUT (1U << 0)
#define INTMASK_VWAKEOUT_ACTIVE_LOW (1U << 8)
#define GPIOSEL     (BASE7 + 0x00)
#define GPIOSEL_VWAKEINT (1U << 0)

static void toshiba_request_vsync(struct mddi_panel_info *pi,
				  struct msmfb_callback *callback)
{
	toshiba_callback = callback;

	if (toshiba_got_int) {
		toshiba_got_int = 0;
		mddi_activate_link(pi->mddi); /* clears interrupt */
	}
}

static void toshiba_wait_vsync(struct mddi_panel_info *pi)
{
	if (toshiba_got_int) {
		toshiba_got_int = 0;
		mddi_activate_link(pi->mddi); /* clears interrupt */
	}
	if (wait_event_timeout(toshiba_vsync_wait, toshiba_got_int, HZ/2) == 0)
		printk(KERN_ERR "timeout waiting for VSYNC\n");
	toshiba_got_int = 0;
	/* interrupt clears when screen dma starts */
}


static struct mddi_panel_ops toshiba_panel_ops = {
	.enable = dummy_function,
	.disable = dummy_function,
	.wait_vsync = toshiba_wait_vsync,
	.request_vsync = toshiba_request_vsync
};

irqreturn_t toshiba_vsync_interrupt(int irq, void *data)
{
	toshiba_got_int = 1;
	if (toshiba_callback) {
		toshiba_callback->func(toshiba_callback);
		toshiba_callback = 0;
	}
	wake_up(&toshiba_vsync_wait);
	return IRQ_HANDLED;
}


static int mddi_toshiba_setup_vsync(struct mddi_info *mddi, int init)
{
	int ret;
	int gpio = 97;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	ret = gpio_direction_input(gpio);
	if (ret)
		goto err_gpio_direction_input_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;

	ret = request_irq(irq, toshiba_vsync_interrupt, IRQF_TRIGGER_RISING,
			  "vsync", NULL);
	if (ret)
		goto err_request_irq_failed;
	printk(KERN_INFO "vsync on gpio %d now %d\n",
	       gpio, gpio_get_value(gpio));
	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), mddi);
err_request_irq_failed:
err_get_irq_num_failed:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_toshiba_probe(struct platform_device *pdev)
{
	int ret;
	struct mddi_info *mddi;
	mddi = pdev->dev.platform_data;

	/* mddi_remote_write(mddi, 0, WAKEUP); */
	mddi_remote_write(mddi, GPIOSEL_VWAKEINT, GPIOSEL);
	mddi_remote_write(mddi, INTMASK_VWAKEOUT, INTMASK);

	ret = mddi_toshiba_setup_vsync(mddi, 1);
	if (ret) {
		dev_err(&pdev->dev, "mddi_toshiba_setup_vsync failed\n");
		return ret;
	}

	mddi_add_panel(pdev->dev.platform_data, &toshiba_panel_ops);
	return 0;
}

static int mddi_toshiba_remove(struct platform_device *pdev)
{
	struct mddi_info *mddi = pdev->dev.platform_data;
	mddi_toshiba_setup_vsync(mddi, 0);
	return 0;
}

static struct platform_driver mddi_client_d263_0000 = {
	.probe = mddi_toshiba_probe,
	.remove = mddi_toshiba_remove,
	.driver = { .name = "mddi_c_d263_0000" },
};

static int __init mddi_client_toshiba_init(void)
{
	platform_driver_register(&mddi_client_d263_0000);
	return 0;
}

module_init(mddi_client_toshiba_init);

