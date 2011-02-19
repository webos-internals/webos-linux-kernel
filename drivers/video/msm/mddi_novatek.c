/* drivers/video/msm/mddi_novatek.c
 *
 * Support for NT35451 mddi client devices
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Jacky Cheung <jacky.cheung@palm.com>
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

#include <linux/gpio.h>
#include <linux/pinmux.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <../board-shank-gpios.h>


#define NOVATEK_PRIM 1

/* 
 * This is to temporarily workaround the inability of modem
 * to provide the requested clk rate 
 * */
#define CLK_MULTIPLIER 3

#define RESET_GPIO 125

extern uint32 mddi_host_core_version;

extern mddi_gpio_info_type mddi_gpio;
extern boolean mddi_vsync_detect_enabled;
static uint32 mddi_novatek_rows_per_second = 416 * 60 ;
static uint32 mddi_novatek_rows_per_refresh = 416;

static struct msm_panel_common_pdata *mddi_novatek_pdata;

static int mddi_novatek_lcd_on(struct platform_device *pdev)
{

	struct msm_fb_data_type *mfd; 

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	printk("%s \n",__func__);

	//Remux the reset pin to output
	pinmux_config("LCD_RESET", PINMUX_CONFIG_ACTIVE);
	pinmux_config("LCD_TE", PINMUX_CONFIG_ACTIVE);
	

	//Reset (stay low for more than 3 ms)
	gpio_set_value(RESET_GPIO, 1);
	msleep(10);
	gpio_set_value(RESET_GPIO, 0);	
	msleep(10);
	gpio_set_value(RESET_GPIO, 1);
	msleep(20);	//20ms or more

	//Sleep out
	mddi_queue_register_write(0x1100, 0x00, FALSE, 0);

	msleep(100); //100ms or more
	
	//Turn on Vsync on 400th line
	mddi_queue_register_write(0x4400, 0x01, FALSE, 0);
	mddi_queue_register_write(0x4401, 0x90, FALSE, 0);
	mddi_queue_register_write(0x3500, 0x00, FALSE, 0);

	//Display on
	mddi_queue_register_write(0x2900, 0x00, FALSE, 0);

	return 0;
}

static int mddi_novatek_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	printk("%s \n",__func__);

	gpio_set_value(RESET_GPIO, 0);	

	mddi_queue_register_write(0x3400, 0x00, FALSE, 0);

	//Display off
	mddi_queue_register_write(0x2800, 0x00, FALSE, 0);

	//Sleep
	mddi_queue_register_write(0x1000, 0x00, FALSE, 0);

	//Deep sleep
	mddi_queue_register_write(0x4F00, 0x01, FALSE, 0);

	//Remux the reset pin to input
	pinmux_config("LCD_RESET", PINMUX_CONFIG_SLEEP);
	pinmux_config("LCD_TE", PINMUX_CONFIG_SLEEP);

	return 0;
}

static int __init mddi_novatek_probe(struct platform_device *pdev)
{
	uint32 id = 0;

	if (pdev->id == 0) {
		mddi_novatek_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	mddi_queue_register_read(0x1F80, &id, TRUE, 0);

	//Turn on Vsync on 400th line
	mddi_queue_register_write(0x4400, 0x01, FALSE, 0);
	mddi_queue_register_write(0x4401, 0x90, FALSE, 0);
	mddi_queue_register_write(0x3500, 0x00, FALSE, 0);


	printk(KERN_INFO "%s user id: 0x%x\n", __func__,id);


	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mddi_novatek_probe,
	.driver = {
		.name   = "mddi_novatek",
	},
};

static struct msm_fb_panel_data mddi_novatek_panel_data0 = {
	.on = mddi_novatek_lcd_on,
	.off = mddi_novatek_lcd_off,
	.set_backlight = NULL,
	.set_vsync_notifier = NULL, 
};

static struct platform_device this_device_0 = {
	.name   = "mddi_novatek",
	.id	= NOVATEK_PRIM,
	.dev	= {
		.platform_data = &mddi_novatek_panel_data0,
	}
};

static int __init mddi_novatek_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	ret = platform_driver_register(&this_driver);
	if (!ret) {
		pinfo = &mddi_novatek_panel_data0.panel_info;
		pinfo->xres = 320;
		pinfo->yres = 400;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = 18;
		pinfo->fb_num = 3;
		pinfo->clk_rate = 122880000 * CLK_MULTIPLIER;
		pinfo->clk_min = 122880000 * CLK_MULTIPLIER; 
		pinfo->clk_max = 122880000 * CLK_MULTIPLIER;
		pinfo->lcd.vsync_enable = TRUE;
      pinfo->lcd.refx100 =
             (mddi_novatek_rows_per_second * 100) /
              mddi_novatek_rows_per_refresh;
		pinfo->lcd.v_back_porch = 8;
		pinfo->lcd.v_front_porch = 8;
		pinfo->lcd.v_pulse_width = 0;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = (1 * HZ);
		pinfo->lcd.vsync_disable_count = 60;	//disable irq if 60 vsyncs (1 sec) are not used
		pinfo->bl_max = 7;
		pinfo->bl_min = 1;

		ret = platform_device_register(&this_device_0);
		if (ret)
		{
			printk(KERN_ERR "%s: failed to register device!\n", __func__);
			platform_driver_unregister(&this_driver);
		}

	}

	mddi_lcd.vsync_detected = 0;

	return ret;
}

module_init(mddi_novatek_init);
