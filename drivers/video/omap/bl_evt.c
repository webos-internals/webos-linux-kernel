/*
 * drivers/video/omap/bl_evt.c
 *
 * Backlight driver for LCD backlights that are controlled by PWM using TI's
 * evt chip
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Author: john chen (jchen1996@gmail.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/ctype.h>

#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/lcd.h>

#include "lcd.h"

#define MOD_NAME 			"LCD-BL: "
#define evt_MAX_BRIGHTNESS	255
#define evt_MIN_BRIGHTNESS	0

#undef MODDEBUG
//#define MODDEBUG	1

#ifdef  MODDEBUG
#define DPRINTK(format,...)\
	printk(KERN_INFO MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

struct evt_lcd_backlight {
	int timer_id;
	int cached;		/* current brightness setting */
	int min_brightness;	/* minimum brightness setting */
	int max_brightness;	/* maximum brightness setting */
	int on_value;		/* value to set brightness to when it turn on */
	int cur_state;

	int should_fade_in;
	int fade_in_msecs;
	int fade_in_step;

	int should_fade_out;
	int fade_out_msecs;
	int fade_out_step;

	struct omap_dm_timer *timer;
	struct mutex ops_lock;

	struct device *dev;
	struct device *parent;
};

/*
 * extern'ed here from the Maxim 8831 LED driver because the backlight
 * is connected to the Maxim 8831 chip, which is part of the LED subsystem
 */
extern void board_maxim8831_backlight_set_brightness(int brightness);

static void inline evt_lcd_backlight_on(struct evt_lcd_backlight *bl)
{
	DPRINTK("%s\n", __FUNCTION__);

	omap_dm_timer_enable(bl->timer);
	omap_dm_timer_set_source(bl->timer, OMAP_TIMER_SRC_32_KHZ);
	omap_dm_timer_set_load(bl->timer, 1, 0xffffff00);
}

static void inline evt_lcd_backlight_send_brightness(
					struct evt_lcd_backlight *bl,
					int brightness)
{
	DPRINTK("%s\n", __FUNCTION__);

	/* if the panel is currently not on, then we only set the turn on value,
	   but don't actually turn on the backlight */
	if (bl->cur_state == FB_BLANK_POWERDOWN) {
		bl->on_value = brightness;
		return;
	}

	mutex_lock(&bl->ops_lock);
	if (brightness <= bl->min_brightness) {
		brightness = bl->min_brightness;
		if (brightness == bl->cached) {
			goto unlock;
		} else {
			/* set brightness for Floyd/Emu/evt boards */
			DPRINTK("%s: Setting bl to min\n", __FUNCTION__);
			omap_dm_timer_set_pwm(bl->timer, 0, 1,
					OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
			omap_dm_timer_stop(bl->timer);
			omap_dm_timer_disable(bl->timer);

			/* set brightness for evtB boards */
			board_maxim8831_backlight_set_brightness(brightness);

			bl->cached = bl->min_brightness;
		}
	} else if (brightness >= bl->max_brightness) {
		brightness = bl->max_brightness;
		if (brightness == bl->cached) {
			goto unlock;
		} else {
			/* set brightness for Floyd/Emu/evt boards */
			DPRINTK("%s: Setting bl to max\n", __FUNCTION__);
			if (bl->min_brightness == bl->cached)
				evt_lcd_backlight_on(bl);
			omap_dm_timer_set_pwm(bl->timer, 1, 1,
					OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
			omap_dm_timer_stop(bl->timer);

			/* set brightness for evtB boards */
			board_maxim8831_backlight_set_brightness(evt_MAX_BRIGHTNESS);

			bl->cached = bl->max_brightness;
		}
	} else {
		if (brightness == bl->cached) {
			goto unlock;
		} else {
			int brightness_to_set;

			/* calculate what value to set the backlight to */
			brightness_to_set =
				(evt_MAX_BRIGHTNESS * brightness) /
				bl->max_brightness;

			/* set brightness for Floyd/Emu/evt boards */
			DPRINTK("%s: Setting brightness to %d\n", __FUNCTION__,
							brightness_to_set);

			if (bl->min_brightness == bl->cached)
				evt_lcd_backlight_on(bl);
			omap_dm_timer_set_pwm(bl->timer, 0, 1,
				OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
			omap_dm_timer_set_match(bl->timer, 1,
					(0xffffff00) | brightness_to_set);
			omap_dm_timer_start(bl->timer);

			/* set brightness for evtB boards */
			board_maxim8831_backlight_set_brightness(brightness_to_set);

			bl->cached = brightness;
		}
	}
unlock:
	mutex_unlock(&bl->ops_lock);
}

static void inline evt_lcd_backlight_fade_out(
					struct evt_lcd_backlight *bl)
{
	int i;
	int level;
	int num_of_steps;
	int time_to_wait;

	/* Avoid division by 0 */
	if ((0 == bl->cached) || (0 == bl->fade_out_step))
		return;

	level = bl->cached;
	num_of_steps = level / bl->fade_out_step;
	time_to_wait = bl->fade_out_msecs / num_of_steps;

	DPRINTK("%s: level=%d steps=%d wait=%d\n",
		__FUNCTION__, level, num_of_steps, time_to_wait);

	for (i=0; i<num_of_steps; i++) {
		level -= bl->fade_out_step;
		if (level < bl->min_brightness)
			break;

		evt_lcd_backlight_send_brightness(bl, level);
		mdelay(time_to_wait);
	}

	if (level != bl->min_brightness)
		evt_lcd_backlight_send_brightness(bl, bl->min_brightness);
}

static void inline evt_lcd_backlight_fade_in(
					struct evt_lcd_backlight *bl)
{
	int i;
	int level = 0;
	int num_of_steps;
	int time_to_wait;

	/* Avoid division by 0 */
	if ((0 == bl->on_value) || (0 == bl->fade_in_step))
		return;

	num_of_steps = bl->on_value / bl->fade_in_step;
	time_to_wait = bl->fade_in_msecs / num_of_steps;

	DPRINTK("%s: level=%d steps=%d wait=%d\n",
		__FUNCTION__, level, num_of_steps, time_to_wait);

	for (i=0; i<num_of_steps; i++) {
		level += bl->fade_in_step;
		evt_lcd_backlight_send_brightness(bl, level);
		mdelay(time_to_wait);
	}

	if (level != bl->on_value)
		evt_lcd_backlight_send_brightness(bl, bl->on_value);
}

static void evt_lcd_backlight_blank(struct evt_lcd_backlight *bl,
				       int mode)
{
	DPRINTK("%s\n", __FUNCTION__);

	if (mode == bl->cur_state)
		return;


	switch (mode) {
	case FB_BLANK_POWERDOWN:
		/* save the on value BEFORE we change brightness (before
		   bl->cached is changed */
		bl->on_value = bl->cached;
		if (bl->should_fade_out)
			evt_lcd_backlight_fade_out(bl);
		else
			evt_lcd_backlight_send_brightness(bl, 0);
		bl->cur_state = FB_BLANK_POWERDOWN;
		break;

	case FB_BLANK_UNBLANK:
		bl->cur_state = FB_BLANK_UNBLANK;
		if (bl->should_fade_in)
			evt_lcd_backlight_fade_in(bl);
		else
			evt_lcd_backlight_send_brightness(bl, bl->on_value);
		break;
	}
}

#ifdef CONFIG_PM
static int evt_lcd_backlight_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	/* don't suspend here - its done in lcd_panel.c
	 *
	 * struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	 * evt_lcd_backlight_blank(bl, FB_BLANK_POWERDOWN);
	 */
	return 0;
}

static int evt_lcd_backlight_resume(struct platform_device *pdev)
{
	/* don't suspend here - its done in lcd_panel.c
	 *
	 * struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	 * evt_lcd_backlight_blank(bl, FB_BLANK_UNBLANK);
	 */
	return 0;
}
#else
#define evt_lcd_backlight_suspend	NULL
#define evt_lcd_backlight_resume	NULL
#endif

/*
 * Backlight specific sysfs attributes
 */
static ssize_t backlight_show_brightness(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->cached);
}
static ssize_t backlight_store_brightness(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	int brightness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	evt_lcd_backlight_send_brightness(bl, brightness);

	bl->on_value = brightness;

	return count;
}
DEVICE_ATTR(brightness, S_IRUGO|S_IWUSR, backlight_show_brightness,
					backlight_store_brightness);

								       static ssize_t backlight_show_fadein_msecs(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->fade_in_msecs);
}
static ssize_t backlight_store_fadein_msecs(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	int fadein_msecs = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (fadein_msecs < 0)
		fadein_msecs = 0;

	bl->fade_in_msecs = fadein_msecs;

	return count;
}
DEVICE_ATTR(fadein_msecs, S_IRUGO|S_IWUSR, backlight_show_fadein_msecs,
					backlight_store_fadein_msecs);
static ssize_t backlight_show_fadeout_msecs(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->fade_out_msecs);
}
static ssize_t backlight_store_fadeout_msecs(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	int fadeout_msecs = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (fadeout_msecs < 0)
		fadeout_msecs = 0;

	bl->fade_out_msecs = fadeout_msecs;

	return count;
}
DEVICE_ATTR(fadeout_msecs, S_IRUGO|S_IWUSR, backlight_show_fadeout_msecs,
					backlight_store_fadeout_msecs);

static ssize_t backlight_show_fadeout_step(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->fade_out_step);
}
static ssize_t backlight_store_fadeout_step(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	int fadeout_step = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (fadeout_step <= 0)
		fadeout_step = 1;
	bl->fade_out_step = fadeout_step;

	return count;
}
DEVICE_ATTR(fadeout_step, S_IRUGO|S_IWUSR, backlight_show_fadeout_step,
					backlight_store_fadeout_step);

static ssize_t backlight_show_fadein_step(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->fade_in_step);
}
static ssize_t backlight_store_fadein_step(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	int fadein_step = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (fadein_step <= 0)
		fadein_step = 1;
	bl->fade_in_step = fadein_step;

	return count;
}
DEVICE_ATTR(fadein_step, S_IRUGO|S_IWUSR, backlight_show_fadein_step,
					backlight_store_fadein_step);

static ssize_t backlight_show_should_fadein(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->should_fade_in);
}
static ssize_t backlight_store_should_fadein(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	int should_fadein = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (should_fadein)
		bl->should_fade_in = 1;
	else
		bl->should_fade_in = 0;

	return count;
}
DEVICE_ATTR(should_fadein, S_IRUGO|S_IWUSR, backlight_show_should_fadein,
					backlight_store_should_fadein);

static ssize_t backlight_show_should_fadeout(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->should_fade_out);
}
static ssize_t backlight_store_should_fadeout(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);
	int should_fadeout = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (should_fadeout)
		bl->should_fade_out = 1;
	else
		bl->should_fade_out = 0;

	return count;
}
DEVICE_ATTR(should_fadeout, S_IRUGO|S_IWUSR, backlight_show_should_fadeout,
					backlight_store_should_fadeout);

static void evt_lcd_backlight_set_state(void *dev, unsigned int state)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);
	DPRINTK("%s dev at 0x%x, pdev at 0x%x, bl at 0x%x\n",
				__FUNCTION__, (int)dev, (int)pdev, (int)bl);

	if (state) {
		evt_lcd_backlight_blank(bl, FB_BLANK_UNBLANK);
	} else {
		evt_lcd_backlight_blank(bl, FB_BLANK_POWERDOWN);
	}
}
static void evt_lcd_backlight_set_brightness(void *dev, int brightness)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	evt_lcd_backlight_send_brightness(bl, brightness);

	DPRINTK("%s:  send brightness %d\n", __FUNCTION__, brightness);
}
static int evt_lcd_backlight_get_brightness(void *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s: got brightness %d\n", __FUNCTION__, bl->cached);

	return bl->cached;
}
static void evt_lcd_backlight_set_max_brightness(void *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	evt_lcd_backlight_send_brightness(bl, bl->max_brightness);
}

static struct lcd_backlight_ops evt_lcd_lcd_backlight_ops = {
	.bl_set_state = evt_lcd_backlight_set_state,
	.bl_set_brightness = evt_lcd_backlight_set_brightness,
	.bl_get_brightness = evt_lcd_backlight_get_brightness,
	.bl_set_max_brightness = evt_lcd_backlight_set_max_brightness,
};

static int evt_lcd_backlight_probe(struct platform_device *pdev)
{
	int result = 0;
	struct evt_lcd_backlight *bl;
	struct bl_platform_data *bl_plat;

	DPRINTK("%s\n", __FUNCTION__);

	bl = kzalloc(sizeof(struct evt_lcd_backlight), GFP_KERNEL);
	if (unlikely(!bl)) {
		result = -ENOMEM;
		goto error_memory;
	}

	platform_set_drvdata(pdev, bl);

	mutex_init(&bl->ops_lock);

	if (pdev->dev.platform_data != NULL) {
		bl_plat = (struct bl_platform_data*)pdev->dev.platform_data;
		bl->timer_id = bl_plat->timer_id;

		bl->parent = bl_plat->parent;

		bl->min_brightness = bl_plat->bl_params.min_brightness;
		bl->max_brightness = bl_plat->bl_params.max_brightness;

		bl->should_fade_in = bl_plat->bl_params.should_fade_in;
		bl->fade_in_msecs = bl_plat->bl_params.fade_in_msecs;
		bl->fade_in_step = bl_plat->bl_params.fade_in_step;

		bl->should_fade_out = bl_plat->bl_params.should_fade_out;
		bl->fade_out_msecs = bl_plat->bl_params.fade_out_msecs;
		bl->fade_out_step = bl_plat->bl_params.fade_out_step;

		DPRINTK("Backlight:  timer = %d, min=%d max = %d\n",
				bl->timer_id, bl->min_brightness,
				bl->max_brightness);
	} else {
		DPRINTK("Error:  No Timer ID available\n");
		goto error_register;
	}

	/* register the device and the ops with lcd_main */
	add_backlight_device(&pdev->dev, &evt_lcd_lcd_backlight_ops, bl->parent);


	/* get related dm timers */
	bl->timer = omap_dm_timer_request_specific(bl->timer_id);
	if (bl->timer == NULL) {
		result = -ENODEV;
		goto error_register;
	}
	omap_dm_timer_disable(bl->timer);

	bl->dev = &pdev->dev;

	/* turn the backlight on initially */
	evt_lcd_backlight_send_brightness(bl, bl->max_brightness);
	bl->cur_state = FB_BLANK_UNBLANK;
	bl->on_value = bl->max_brightness;

	/* sysfs entries */
	if (device_create_file(&pdev->dev, &dev_attr_brightness) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for brightness failed\n");
		goto error_sysfs;
	}
	if (device_create_file(&pdev->dev, &dev_attr_fadeout_msecs) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for fadeout_msecs failed\n");
		goto error_sysfs;
	}
	if (device_create_file(&pdev->dev, &dev_attr_fadein_msecs) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for fadein_msecs failed\n");
		goto error_sysfs;
	}
	if (device_create_file(&pdev->dev, &dev_attr_fadeout_step) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for fadeout_step failed\n");
		goto error_sysfs;
	}
	if (device_create_file(&pdev->dev, &dev_attr_fadein_step) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for fadein_step failed\n");
		goto error_sysfs;
	}
	if (device_create_file(&pdev->dev, &dev_attr_should_fadein) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for should_fadein failed\n");
		goto error_sysfs;
	}
	if (device_create_file(&pdev->dev, &dev_attr_should_fadeout) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for should_fadeout failed\n");
		goto error_sysfs;
	}

	printk(KERN_INFO "evt LCD backlight initialized\n");

	return 0;

error_sysfs:
	device_remove_file(&pdev->dev, &dev_attr_brightness);
	device_remove_file(&pdev->dev, &dev_attr_fadeout_msecs);
	device_remove_file(&pdev->dev, &dev_attr_fadein_msecs);
	device_remove_file(&pdev->dev, &dev_attr_fadeout_step);
	device_remove_file(&pdev->dev, &dev_attr_fadein_step);
	device_remove_file(&pdev->dev, &dev_attr_should_fadein);
	device_remove_file(&pdev->dev, &dev_attr_should_fadeout);

error_register:
	remove_backlight_device(&pdev->dev, bl->parent);
	kfree(bl);

error_memory:
	return result;

}

static int evt_lcd_backlight_remove(struct platform_device *pdev)
{
	struct evt_lcd_backlight *bl = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_brightness);
	device_remove_file(&pdev->dev, &dev_attr_fadeout_msecs);
	device_remove_file(&pdev->dev, &dev_attr_fadein_msecs);
	device_remove_file(&pdev->dev, &dev_attr_fadeout_step);
	device_remove_file(&pdev->dev, &dev_attr_fadein_step);
	device_remove_file(&pdev->dev, &dev_attr_should_fadein);
	device_remove_file(&pdev->dev, &dev_attr_should_fadeout);

	remove_backlight_device(&pdev->dev, bl->parent);
	kfree(bl);

	printk(KERN_INFO "evt LCD backlight removed\n");

	return 0;
}

static struct platform_driver evt_lcd_backlight_driver = {
	.probe = evt_lcd_backlight_probe,
	.remove = evt_lcd_backlight_remove,
	.suspend = evt_lcd_backlight_suspend,
	.resume = evt_lcd_backlight_resume,
	.driver = {
		   .name = "lcd-backlight",
	},
};

static int __init evt_lcd_backlight_init(void)
{
	return platform_driver_register(&evt_lcd_backlight_driver);
}

static void __exit evt_lcd_backlight_exit(void)
{
	platform_driver_unregister(&evt_lcd_backlight_driver);
}

module_init(evt_lcd_backlight_init);
module_exit(evt_lcd_backlight_exit);

MODULE_AUTHOR("John Chen <jchen1996@gmail.com>");
MODULE_DESCRIPTION("Palm evt LCD Backlight driver");
MODULE_LICENSE("GPL");
