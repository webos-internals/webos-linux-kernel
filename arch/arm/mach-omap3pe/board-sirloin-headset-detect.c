/*
 * linux/arch/arm/mach-omap3pe/board-headset-detect.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/twl4030-audio.h>
#include <asm/arch/twl4030-madc.h>

#include "headset-detect.h"

#define SIRLOIN_GPIO_HS_PLUG_IN     111
#define SIRLOIN_GPIO_HS_AMP_SD_N    86
#define SIRLOIN_GPIO_MIC_ANS_INT    87

#define SIRLOIN_GPIO_HS_AMP_MUX_NORMAL   "G25_3430_GPIO086"
#define SIRLOIN_GPIO_HS_AMP_MUX_OFFMODE  "G25_3430_GPIO086_OFFMODE"

#define HS_INSERT_DELAY            20
#define HS_REMOVE_DELAY            20
#define HS_BUTTON_DEBOUNCE_PERIOD  160
#define HS_BUTTON_PRESS_TIMEOUT    20
#define HS_BUTTON_PRESS_SLEEP      20
#define HS_TURNON_DELAY            40 /* time to wait for hardware to settle before powering amplifier */

#define INPUT_DEVICE_PHYS   "headset/input0"

#define DEBUG

#ifdef DEBUG
#define dprintk(x ...)    printk(x)
#else
#define dprintk(x ...)
#endif

enum {
	NONE,
	HEADSET,
	HEADSET_MIC,
} forced_device = -1;

struct headset_device {
	const char *name;
	struct platform_device *pdev;
	struct input_dev *idev;

	/* Well, this is completely sirloin specific anyway */
	unsigned gpio_hs_plug_in;
	unsigned gpio_hs_amp_sd_n;
	unsigned gpio_mic_ans_int;

	int audio_on;
	int amp_enabled;

	int polarity;    /* EVT0/1 active low, EVT2+ active high */
	int headset_inserted;
	int mic_present;
	work_func_t headset_detect_func;
	struct work_struct headset_detect_work;

	work_func_t headset_mic_detect_func;
	struct delayed_work headset_mic_detect_work;

	struct timer_list headset_detect_timer;
	struct timer_list button_press_timer;        /* To fake the button up event */
	struct timer_list button_dispatch_timer;  /* Button debounce */
	struct timer_list enable_headset_timer;   /* headset turn on delay */

	int button_pressed;
	int num_mic_check;
	int checking_mic;
#define MIC_CHECK_MAX   10
#define BUTTON_EVENT_QUEUE_SIZE    16
	enum {
		BUTTON_UP,
		BUTTON_DOWN,
	} button_events[BUTTON_EVENT_QUEUE_SIZE];
	int next_button_event;
	int last_polled_button;
};


static void headset_amp_enable(struct headset_device *dev, int enable)
{
	gpio_set_value(dev->gpio_hs_amp_sd_n, enable);
}

static void headset_force_amp_enable (struct headset_device *dev, int enable)
{
	if (enable != dev->amp_enabled) {
		dev->amp_enabled = enable;
		headset_amp_enable(dev, enable);

		if(enable)
			omap_cfg_reg( SIRLOIN_GPIO_HS_AMP_MUX_NORMAL );
		else
			omap_cfg_reg( SIRLOIN_GPIO_HS_AMP_MUX_OFFMODE );
	}
}

static void delayed_headset_enable_handler(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;
	headset_force_amp_enable(dev, dev->audio_on && dev->headset_inserted);
}

static void headset_codec_event(void *cookie, int event)
{
	struct headset_device *dev = cookie;

	dev->audio_on = event;
	if (event == CODEC_ENABLED) {
		mod_timer(&dev->enable_headset_timer,
			jiffies + msecs_to_jiffies(HS_TURNON_DELAY));
	} else {
		del_timer(&dev->enable_headset_timer);  // cancel any pending timer and immediately disable
		headset_force_amp_enable(dev, dev->audio_on && dev->headset_inserted);
	}
}

static uint16_t adcin_read(int adc)
{
	int r = 0;
	uint16_t v;

	r = madc_start_conversion(adc);
	if (r < 0) {
		printk(KERN_ERR "HS-DET: Failed to read ADCIN0\n");
		return 0;
	}
	v = (uint16_t)r;

	/* If there is no mic micbias will be pulled low */
	return v;
}

#define ADCIN0_READ()    adcin_read(0)
#define MIC_PRESENT()    (adcin_read(0) >= 0x100)

static ssize_t headset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct headset_device *d = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", d->headset_inserted);
}

static ssize_t hsmic_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct headset_device *d = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", d->mic_present);
	return sprintf(buf, "%#x\n", ADCIN0_READ());
}

static ssize_t adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%#x\n", ADCIN0_READ());
}

static ssize_t pol_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct headset_device *hdev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", hdev->polarity);
}

static ssize_t pol_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct headset_device *hdev = dev_get_drvdata(dev);
	if (count > 0) {
		hdev->polarity = !(buf[0] == '0');
	}
	return count;
}

DEVICE_ATTR(headset_detect, S_IRUGO, headset_show, NULL);
DEVICE_ATTR(hsmic_detect, S_IRUGO, hsmic_show, NULL);

DEVICE_ATTR(adc, S_IRUGO, adc_show, NULL);
DEVICE_ATTR(pol_ctl, S_IRUGO | S_IWUGO, pol_show, pol_store);

extern int twl4030_audio_mic_bias_enable(bool);

/* delayed work queue to test if mic is present in a 10 second period
 * stops testing if mic is found or the detection period is over,
 * whichever comes first.
 */
static void headset_mic_detect_handler(struct delayed_work *work)
{
	struct headset_device *dev;
	int mic;

	dev = container_of(work, struct headset_device, headset_mic_detect_work);

	if ((dev->num_mic_check++ < MIC_CHECK_MAX) && dev->headset_inserted && !dev->mic_present) {
		/* audiod overwrite registers when headset events occur, so keep setting hs mic bias*/
		twl4030_audio_mic_bias_enable(true);
		mic = MIC_PRESENT();

		if (mic) {
			dev->mic_present = mic;
			omap_cfg_reg("Y4_3430_MIC_ANS_ON"); /* MIC-ANS */
			twl4030_audio_mic_bias_enable(false);
			dev->checking_mic = false;
		}

		if (dev->mic_present) {
			input_report_switch(dev->idev, SW_HEADPHONE_MIC_INSERT, 1);
			mod_timer(&dev->enable_headset_timer,
				jiffies + msecs_to_jiffies(HS_TURNON_DELAY));
			/* Report headset removed if headset is inserted then mic is found
			   so that states are synced up properly in hidd */
			input_report_switch(dev->idev, SW_HEADPHONE_INSERT, 0);
		}
		else {
			/* keep checking every 500ms if mic isnt found */
			schedule_delayed_work(&dev->headset_mic_detect_work, msecs_to_jiffies(500));
		}
	}
	else {
		/* if number of checks exceeds the maximum amount of
		 * checks then just assume its a headset without mic
		 */
		twl4030_audio_mic_bias_enable(false);
		mod_timer(&dev->enable_headset_timer,
			jiffies + msecs_to_jiffies(HS_TURNON_DELAY));
		dev->checking_mic = false;
	}
}

static void headset_insert_handler(struct headset_device *dev)
{
	int mic;

	/* reset num_mic_check, so it detects mic for set time in seconds */
	dev->num_mic_check = 0;

	/* Check if there is a mic on the headset */
	dev->checking_mic = true;
	twl4030_audio_mic_bias_enable(true);
	mic = MIC_PRESENT();

	if (mic != dev->mic_present) {
		dev->mic_present = mic;
		if (mic) {
			omap_cfg_reg("Y4_3430_MIC_ANS_ON"); /* MIC-ANS */
		} else {
			omap_cfg_reg("Y4_3430_MIC_ANS_OFF"); /* MIC-ANS */
		}
	}

	mod_timer(&dev->enable_headset_timer,
		jiffies + msecs_to_jiffies(HS_TURNON_DELAY));

	if (dev->mic_present) {
		twl4030_audio_mic_bias_enable(false);
		input_report_switch(dev->idev, SW_HEADPHONE_MIC_INSERT, 1);
		dev->checking_mic = false;
	} else {
		input_report_switch(dev->idev, SW_HEADPHONE_INSERT, 1);
		schedule_delayed_work(&dev->headset_mic_detect_work, msecs_to_jiffies(500));
	}
}

static void headset_remove_handler(struct headset_device *dev)
{
	int mic_was_present;
	mic_was_present = dev->mic_present;

	/* Can't have a headset button or mic if nothing is inserted, 
	 * cancel the work related to it */
	if (dev->mic_present) {
                dev->button_pressed = 0;
                dev->next_button_event = 0;
		dev->mic_present = false;
		omap_cfg_reg("Y4_3430_MIC_ANS_OFF"); /* MIC-ANS */
		del_timer(&dev->button_press_timer);  // cancel any pending timer and immediately disable
	}

	/* FIXME only turn on the amp if audio is streaming */
	headset_force_amp_enable (dev, 0);

	/* turn off mic bias in the case that we were still checking for mic but headset was removed */
	if (dev->checking_mic) {
		twl4030_audio_mic_bias_enable(false);
		dev->checking_mic = false;
	}

	/* If the button is down, send a button up event */
	if (dev->button_pressed) {
		input_report_key(dev->idev, KEY_PLAYPAUSE, 0);
	}
	if (mic_was_present) {
		input_report_switch(dev->idev, SW_HEADPHONE_MIC_INSERT, 0);
	} else {
		input_report_switch(dev->idev, SW_HEADPHONE_INSERT, 0);
	}

	cancel_work_sync((struct work_struct *)&dev->headset_mic_detect_work);
}

static inline int headset_inserted(struct headset_device *dev)
{
	int ins = !(omap_get_gpio_level(dev->gpio_hs_plug_in));
	if (dev->polarity) {
		ins = !ins;
	}
	return ins;
}

static void headset_detect_handler(struct work_struct *work)
{
	struct headset_device *dev;
	int ins;

	dev = container_of(work, struct headset_device, headset_detect_work);

	/* Check HS-PLUG-IN. low = inserted, high = not inserted */
	ins = headset_inserted(dev);

	if (ins != dev->headset_inserted) {
		if (ins) {
                        printk(KERN_INFO "Headset inserted!\n");
			headset_insert_handler(dev);
		} else {
                        printk(KERN_INFO "Headset Removed!\n");
			headset_remove_handler(dev);
		}
		dev->headset_inserted = ins;
	}
}

static struct headset_device sirloin_headset_device = {
	.name = "headset-detect",
	.gpio_hs_plug_in = SIRLOIN_GPIO_HS_PLUG_IN,
	.gpio_hs_amp_sd_n = SIRLOIN_GPIO_HS_AMP_SD_N,
	.gpio_mic_ans_int = SIRLOIN_GPIO_MIC_ANS_INT,
	.headset_inserted = 0,
	.mic_present = 0,
	.button_pressed = 0,
	.num_mic_check = 0,
	.checking_mic = 0,
	.next_button_event = 0,
	.headset_detect_func = headset_detect_handler,
	.headset_mic_detect_func = (work_func_t) headset_mic_detect_handler,
};

static irqreturn_t mic_ans_int_irq(int irq, void *dev_id)
{
	struct headset_device *dev = (struct headset_device *)dev_id;

	mod_timer(&dev->button_press_timer,
			jiffies + msecs_to_jiffies(HS_BUTTON_PRESS_TIMEOUT));

	return IRQ_HANDLED;
}

static void button_dispatch(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;
	int i;

	if (!(dev->headset_inserted && dev->mic_present) || !headset_inserted(dev)) {
		/* No button events if the headset is not inserted */
		dev->next_button_event = 0;
		dev->button_pressed = 0;
		return;
	}

	for (i = 0; i < dev->next_button_event; i++) {
		if (dev->button_events[i] == BUTTON_UP) {
                        printk(KERN_INFO "HS Button released\n");
			input_report_key(dev->idev, KEY_PLAYPAUSE, 0);
		} else {
                        printk(KERN_INFO "HS Button pressed\n");
			input_report_key(dev->idev, KEY_PLAYPAUSE, 1);
		}
	}
	dev->next_button_event = 0;
}

static void button_press_timeout_handler(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;
	int state = gpio_get_value(dev->gpio_mic_ans_int);

	if (state) {
		if (dev->button_pressed) {
			dev->button_pressed = 0;

			dev->button_events[dev->next_button_event] = BUTTON_UP;
			dev->next_button_event += 1;
			mod_timer(&dev->button_dispatch_timer,
					jiffies + msecs_to_jiffies(HS_BUTTON_DEBOUNCE_PERIOD));
		}
	}
	else {
		if (!dev->button_pressed) {
			dev->button_pressed = 1;

			dev->button_events[dev->next_button_event] = BUTTON_DOWN;
			dev->next_button_event += 1;
			mod_timer(&dev->button_dispatch_timer,
					jiffies + msecs_to_jiffies(HS_BUTTON_DEBOUNCE_PERIOD));
		}
	}
}

static irqreturn_t hs_plug_in_irq(int irq, void *dev_id)
{
	struct headset_device *dev = (struct headset_device *)dev_id;

	if (gpio_get_value(dev->gpio_hs_plug_in)) {
		mod_timer(&dev->headset_detect_timer,
			jiffies + msecs_to_jiffies(HS_INSERT_DELAY));
	}
	else {
		mod_timer(&dev->headset_detect_timer,
			jiffies + msecs_to_jiffies(HS_REMOVE_DELAY));
	}

	return IRQ_HANDLED;
}

static void headset_detect_debounced(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;

	schedule_work(&dev->headset_detect_work);
}

static int headset_register_input_device(struct headset_device *dev, struct input_dev **input_device)
{
	int r;
	struct input_dev *idev = NULL;

	/* Publish as an input device */
	idev = input_allocate_device();
	if (!idev) {
		return -ENOMEM;
	}

	idev->name = "headset";

	idev->id.bustype = BUS_VIRTUAL;
	idev->id.vendor = 0x0;
	idev->id.product = 0x0;
	idev->id.version = 0x100;

	idev->dev.parent = &dev->pdev->dev;
	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SW);
	set_bit(KEY_PLAYPAUSE, idev->keybit);
	set_bit(SW_HEADPHONE_INSERT, idev->swbit);
	set_bit(SW_HEADPHONE_MIC_INSERT, idev->swbit);

	r = input_register_device(idev);
	if (r) {
		goto fail;
	}

	*input_device = idev;
	return 0;
fail:
	input_free_device(idev);
	return r;
}

static int headset_publish_sysfs(struct headset_device *dev)
{
	int r;

	/* Add entries in /sys to query state of headset */
	r = device_create_file(&dev->pdev->dev, &dev_attr_headset_detect);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to create headset_detect in /sys: %d\n", r);
	}
	r = device_create_file(&dev->pdev->dev, &dev_attr_hsmic_detect);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to create hsmic_detect in /sys: %d\n", r);
	}

	r = device_create_file(&dev->pdev->dev, &dev_attr_adc);
	if (r) {
		printk(KERN_ERR
				"HS-DET: Failed to create adc in /sys: %d\n", r);
	}
	r = device_create_file(&dev->pdev->dev, &dev_attr_pol_ctl);
	if (r) {
		printk(KERN_ERR
				"HS-DET: Failed to create pol_ctl in /sys: %d\n", r);
	}
	return 0;
}

static int __init headset_probe(struct platform_device *pdev)
{
	int r = 0;
	struct headset_device *dev = &sirloin_headset_device;
	struct sirloin_headset_detect_platform_data *pdata = 
		pdev->dev.platform_data;
	struct input_dev *idev = NULL;

	r = gpio_request(dev->gpio_hs_plug_in, "gpio");
	if (r) {
		printk(KERN_ERR "HS-DET: Couldn't request HS-PLUG-IN (gpio_%u)\n",
				dev->gpio_hs_plug_in);
		goto fail;
	}

	r = gpio_request(dev->gpio_hs_amp_sd_n, "gpio");
	if (r) {
		printk(KERN_ERR "HS-DET: Couldn't request HS-AMP-SD~ (gpio_%u)\n",
				dev->gpio_hs_amp_sd_n);
		goto fail;
	}

	r = gpio_request(dev->gpio_mic_ans_int, "gpio");
	if (r) {
		printk(KERN_ERR "HS-DET: Couldn't request MIC-ANS-INT (gpio_%u)\n",
				dev->gpio_mic_ans_int);
		goto fail;
	}

	/* Interrupt sources are inputs */
	gpio_direction_input(dev->gpio_hs_plug_in);

	/* Headset amp is off by default */
	gpio_direction_output(dev->gpio_hs_amp_sd_n, 0);

	/* Configure the pinmux for the GPIOs */
	omap_cfg_reg("B26_3430_GPIO111");    /* HS-PLUG-IN */
	omap_cfg_reg("G25_3430_GPIO086");    /* HS-AMP-SD~ */
	omap_cfg_reg("Y4_3430_MIC_ANS_OFF"); /* MIC-ANS */
	omap_cfg_reg("H27_3430_GPIO087");    /* MIC-ANS-INT */

	dev->polarity = pdata->polarity;

	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);

	/* Init the workqueues */
	INIT_WORK(&dev->headset_detect_work, dev->headset_detect_func);
	INIT_DELAYED_WORK(&dev->headset_mic_detect_work, dev->headset_mic_detect_func);

	init_timer(&dev->headset_detect_timer);
	dev->headset_detect_timer.function = headset_detect_debounced;
	dev->headset_detect_timer.data = (unsigned long)dev;

	init_timer(&dev->button_press_timer);
	dev->button_press_timer.function = button_press_timeout_handler;
	dev->button_press_timer.data = (unsigned long)dev;

	init_timer(&dev->button_dispatch_timer);
	dev->button_dispatch_timer.function = button_dispatch;
	dev->button_dispatch_timer.data = (unsigned long)dev;

	init_timer(&dev->enable_headset_timer);
	dev->enable_headset_timer.function = delayed_headset_enable_handler;
	dev->enable_headset_timer.data = (unsigned long)dev;

	/* Request HS-PLUG-IN IRQ on both edges to get both insert and
	 * remove interrupts */
	r = request_irq(OMAP_GPIO_IRQ(dev->gpio_hs_plug_in), hs_plug_in_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, dev->name, dev);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to request irq on hs_plug_in (gpio_%u)\n",
				dev->gpio_hs_plug_in);
		/* FIXME cleanup */
	}

	/* Request IRQ for MIC-ANS-INT to get interrupts for the
	 * headset button when the GPIO is pulled up */
	/* FIXME Don't know if we need both edges */
	r = request_irq(OMAP_GPIO_IRQ(dev->gpio_mic_ans_int), 
			mic_ans_int_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			dev->name, dev);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to request irq on MIC-ANS-INT (gpio_%u)\n",
				dev->gpio_mic_ans_int);
		/* FIXME cleanup */
	}

	r = headset_register_input_device(dev, &idev);
	if (r) {
		goto fail;
	}
	dev->idev = idev;

	headset_publish_sysfs(dev);

	/* mark this input device as wakeup capable */
	device_init_wakeup(&dev->idev->dev, 1);

	/* Check initial headset state */
	schedule_work(&dev->headset_detect_work);
	mod_timer(&dev->enable_headset_timer,
		jiffies + msecs_to_jiffies(HS_TURNON_DELAY));
	return r;

fail:
	if (dev->idev) {
		input_unregister_device(dev->idev);
		input_free_device(dev->idev);
	}

	free_irq(OMAP_GPIO_IRQ(dev->gpio_hs_plug_in), NULL);
	free_irq(OMAP_GPIO_IRQ(dev->gpio_mic_ans_int), NULL);

	gpio_free(dev->gpio_hs_plug_in);
	gpio_free(dev->gpio_hs_amp_sd_n);
	gpio_free(dev->gpio_mic_ans_int);

	return r;
}

static int __init headset_probe_forced(struct platform_device *pdev)
{
	int r = 0;
	struct headset_device *dev = &sirloin_headset_device;
	struct input_dev *idev = NULL;

	switch (forced_device) {
		case NONE:
			dev->headset_inserted = 0;
			dev->mic_present = 0;
			break;
		case HEADSET:
			dev->headset_inserted = 1;
			dev->mic_present = 0;
			break;
		case HEADSET_MIC:
			dev->headset_inserted = 1;
			dev->mic_present = 1;
			break;
		default:
			printk(KERN_ERR "headset: Cannot force device %d\n", forced_device);
			headset_probe(pdev);
	}

	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);

	r = headset_register_input_device(dev, &idev);
	if (r) {
		return r;
	}
	dev->idev = idev;

	headset_publish_sysfs(dev);

	return 0;
}

static int __init _headset_probe(struct platform_device *pdev)
{
	int r = 0;

	if (forced_device != -1) {
		r = headset_probe_forced(pdev);
	} else {
		r = headset_probe(pdev);
	}

	if (r == 0) {
		twl4030_register_codec_event_callback(headset_codec_event, platform_get_drvdata(pdev));
	}

	return r;
}

static int headset_remove(struct platform_device *pdev)
{
	struct headset_device *dev = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_headset_detect);
	device_remove_file(&pdev->dev, &dev_attr_hsmic_detect);
	device_remove_file(&pdev->dev, &dev_attr_adc);
	device_remove_file(&pdev->dev, &dev_attr_pol_ctl);

	free_irq(OMAP_GPIO_IRQ(dev->gpio_hs_plug_in), NULL);
	free_irq(OMAP_GPIO_IRQ(dev->gpio_mic_ans_int), NULL);

	cancel_work_sync(&dev->headset_detect_work);

	gpio_free(dev->gpio_hs_plug_in);
	gpio_free(dev->gpio_hs_amp_sd_n);
	gpio_free(dev->gpio_mic_ans_int);

	return 0;
}

static int headset_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct headset_device *dev = platform_get_drvdata(pdev);

	/* Do nothing if we're faking state */
	if (forced_device != -1) {
		return 0;
	}

	headset_force_amp_enable (dev, 0);

	if (dev->mic_present) {
		dev->next_button_event = 0;
		dev->button_pressed = 0;
	}

	if( device_may_wakeup(&dev->idev->dev)) {
		// disable irq, but enable irq wake
		disable_irq    (gpio_to_irq(dev->gpio_hs_plug_in));
		enable_irq_wake(gpio_to_irq(dev->gpio_hs_plug_in));

		disable_irq    (gpio_to_irq(dev->gpio_mic_ans_int));
		enable_irq_wake(gpio_to_irq(dev->gpio_mic_ans_int));
	}

	return 0;
}

static int headset_resume(struct platform_device *pdev)
{
	struct headset_device *dev = platform_get_drvdata(pdev);

	/* Do nothing if we're faking state */
	if (forced_device != -1) {
		return 0;
	}

	if (device_may_wakeup(&dev->idev->dev)) {
		// disable wake and reenable irq
		disable_irq_wake(gpio_to_irq(dev->gpio_hs_plug_in));
		enable_irq (gpio_to_irq(dev->gpio_hs_plug_in));

		disable_irq_wake(gpio_to_irq(dev->gpio_mic_ans_int));
		enable_irq (gpio_to_irq(dev->gpio_mic_ans_int));
	}

	/* If headset is resumed, waking the device, then check
	   to see if button was pressed if a mic is already inserted */
	if (dev->mic_present && dev->headset_inserted)
		mod_timer(&dev->button_press_timer,
			jiffies + msecs_to_jiffies(HS_BUTTON_PRESS_SLEEP));

	schedule_work(&dev->headset_detect_work);

	return 0;
}

static int __init force_headset_device(char *device)
{
	if (strcmp("headset", device) == 0) {
		forced_device = HEADSET;
	} else if (strcmp("headset-mic", device) == 0) {
		forced_device = HEADSET_MIC;
	} else if (strcmp("none", device) == 0) {
		forced_device = NONE;
	}
	return 0;
}

__setup("force_headset_device=", force_headset_device);


static struct platform_driver headset_driver = {
	.driver = {
		.name = "headset-detect",
	},
	.probe = _headset_probe,
	.remove = __devexit_p(headset_remove),
	.suspend = headset_suspend,
	.resume  = headset_resume,
};

static int __init headset_init(void)
{
	return platform_driver_register(&headset_driver);
}

static void __exit headset_exit(void)
{
	platform_driver_unregister(&headset_driver);
}

module_init(headset_init);
module_exit(headset_exit);

MODULE_DESCRIPTION("headset");
MODULE_LICENSE("GPL");
