/*
 * drivers/video/omap/bl_maxim8831.c
 *
 * Backlight driver for LCD's using Maxim 8831 chip to control the LCD backlight
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
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/ctype.h>

#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/mux.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/lcd.h>

#include "lcd.h"

#define MOD_NAME 		"LCD-BL: "

#undef MODDEBUG
// #define MODDEBUG		1

#ifdef  MODDEBUG
#define DPRINTK(format,...)\
	printk(KERN_INFO MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define FADE_MODE_FADEIN	1
#define FADE_MODE_FADEOUT	0

struct maxim8831_lcd_backlight {
	int i2c_id;
	int led_id;
	int powermode;
	int cached;
	int min_brightness;
	int max_brightness;
	int on_value;
	int fade_msecs;
	struct omap_dm_timer *timer;

	struct device *dev;
	struct device *parent;
};

/*
 * extern'ed here from the Maxim 8831 LED driver because the backlight
 * is connected to the Maxim 8831 chip, which is part of the LED subsystem
 */
extern void maxim8831_set_brightness_exported(	int i2c_id,
						int led_id,
						int brightness);
extern void maxim8831_set_ramp_up_time_exported(int i2c_id,
						int led_id,
						unsigned int ramptime);
extern void maxim8831_set_ramp_down_time_exported(int i2c_id,
						int led_id,
						unsigned int ramptime);

static void inline maxim8831_lcd_backlight_send_brightness(
					struct maxim8831_lcd_backlight *bl,
					int brightness)
{
	if (brightness == bl->cached)
		return;

	if (brightness < bl->min_brightness)
		brightness = bl->min_brightness;
	else if (brightness > bl->max_brightness)
		brightness = bl->max_brightness;

	maxim8831_set_brightness_exported(bl->i2c_id, bl->led_id, brightness);
	bl->cached = brightness;

}

static void maxim8831_lcd_backlight_blank(struct maxim8831_lcd_backlight *bl,
				       int mode)
{
	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		maxim8831_lcd_backlight_send_brightness(bl, 0);
		break;

	case FB_BLANK_UNBLANK:
		maxim8831_lcd_backlight_send_brightness(bl,
						     bl->on_value);
		break;
	}
}

#ifdef CONFIG_PM
static int maxim8831_lcd_backlight_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	maxim8831_lcd_backlight_blank(bl, FB_BLANK_POWERDOWN);
	return 0;
}

static int maxim8831_lcd_backlight_resume(struct platform_device *pdev)
{
	struct maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	maxim8831_lcd_backlight_blank(bl, bl->powermode);
	return 0;
}
#else
#define maxim8831_lcd_backlight_suspend	NULL
#define maxim8831_lcd_backlight_resume	NULL
#endif

/*
 * Backlight specific sysfs attributes
 */
static ssize_t backlight_show_brightness(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

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
	struct maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);
	int brightness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	maxim8831_lcd_backlight_send_brightness(bl, brightness);

	maxim8831_set_brightness_exported(3, 0, brightness);

	return count;
}
DEVICE_ATTR(brightness, S_IRUGO|S_IWUSR, backlight_show_brightness,
					backlight_store_brightness);
static ssize_t backlight_show_fade_msecs(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", bl->fade_msecs);
}
static ssize_t backlight_store_fade_msecs(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	struct platform_device *pdev = to_platform_device(dev);
	struct maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);
	int fade_msecs = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	bl->fade_msecs = fade_msecs;

	maxim8831_set_ramp_up_time_exported(bl->i2c_id, bl->led_id, fade_msecs);
	maxim8831_set_ramp_down_time_exported(bl->i2c_id, bl->led_id,
							fade_msecs);

	return count;
}
DEVICE_ATTR(fade_msecs, S_IRUGO|S_IWUSR, backlight_show_fade_msecs,
					backlight_store_fade_msecs);

/*
 * functions for the parent driver to use
 */
void maxim8831_lcd_backlight_set_state(void *dev, int state)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s: setting state %d\n", __FUNCTION__, state);

	if (state) {
		maxim8831_lcd_backlight_send_brightness(bl, bl->on_value);
	} else {
		bl->on_value = bl->cached;
		maxim8831_lcd_backlight_send_brightness(bl, bl->min_brightness);
	}
}
void maxim8831_lcd_backlight_set_brightness(void *dev, int brightness)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	maxim8831_lcd_backlight_send_brightness(bl, brightness);

	DPRINTK("%s:  send brightness %d\n", __FUNCTION__, brightness);
}

void maxim8831_lcd_backlight_set_max_brightness(void *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	maxim8831_lcd_backlight_send_brightness(bl, bl->max_brightness);
}

int maxim8831_lcd_backlight_get_brightness(void *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s: got brightness %d\n", __FUNCTION__, bl->cached);

	return bl->cached;
}

static struct lcd_backlight_ops maxim8831_lcd_lcd_backlight_ops = {
	.bl_set_state = maxim8831_lcd_backlight_set_state,
	.bl_set_brightness = maxim8831_lcd_backlight_set_brightness,
	.bl_get_brightness = maxim8831_lcd_backlight_get_brightness,
	.bl_set_max_brightness = maxim8831_lcd_backlight_set_max_brightness,
};

static int maxim8831_lcd_backlight_probe(struct platform_device *pdev)
{
	int result = 0;
	struct maxim8831_lcd_backlight *bl;
	struct bl_platform_data *bl_plat;

	DPRINTK("%s\n", __FUNCTION__);

	bl = kzalloc(sizeof(struct maxim8831_lcd_backlight), GFP_KERNEL);
	if (unlikely(!bl)) {
		result = -ENOMEM;
		goto error_memory;
	}

	platform_set_drvdata(pdev, bl);


	if (pdev->dev.platform_data != NULL) {
		bl_plat = (struct bl_platform_data*)pdev->dev.platform_data;
		bl->min_brightness = bl_plat->bl_params.min_brightness;
		bl->max_brightness = bl_plat->bl_params.max_brightness;
		bl->fade_msecs = bl_plat->fade_msecs;
		bl->i2c_id = bl_plat->i2c_id;
		bl->led_id = bl_plat->led_id;
		bl->parent = bl_plat->parent;

		DPRINTK("Backlight:  i2c = %d, led = %d, min=%d max = %d\n",
				bl->i2c_id, bl->led_id,
				bl->min_brightness, bl->max_brightness);
	} else {
		DPRINTK("Error:  platform data available\n");
		goto error_register;
	}

	/* register the device and the ops with lcd_main */
	panel_add_backlight(&pdev->dev, &maxim8831_lcd_lcd_backlight_ops, bl->parent);

	bl->powermode = FB_BLANK_POWERDOWN;

	bl->dev = &pdev->dev;

	/* turn the backlight on initially */
	// maxim8831_lcd_backlight_set_max_brightness(pdev);

	/* sysfs entries */
	if (device_create_file(&pdev->dev, &dev_attr_brightness) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for brightness failed\n");
		goto error_sysfs;
	}
	if (device_create_file(&pdev->dev, &dev_attr_fade_msecs) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for fade_msecs failed\n");
		goto error_sysfs;
	}

	printk(KERN_INFO "maxim8831 LCD backlight initialized\n");

	return 0;

error_sysfs:
	device_remove_file(&pdev->dev, &dev_attr_brightness);
	device_remove_file(&pdev->dev, &dev_attr_fade_msecs);

error_register:
	kfree(bl);

error_memory:
	return result;

}

static int maxim8831_lcd_backlight_remove(struct platform_device *pdev)
{
	struct maxim8831_lcd_backlight *bl = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_brightness);
	device_remove_file(&pdev->dev, &dev_attr_fade_msecs);

	panel_remove_backlight(&pdev->dev, bl->parent);
	kfree(bl);

	printk(KERN_INFO "maxim8831 LCD backlight removed\n");

	return 0;
}

static struct platform_driver maxim8831_lcd_backlight_driver = {
	.probe = maxim8831_lcd_backlight_probe,
	.remove = maxim8831_lcd_backlight_remove,
	.suspend = maxim8831_lcd_backlight_suspend,
	.resume = maxim8831_lcd_backlight_resume,
	.driver = {
		   .name = "lcd-backlight",
	},
};

static int __init maxim8831_lcd_backlight_init(void)
{
	return platform_driver_register(&maxim8831_lcd_backlight_driver);
}

static void __exit maxim8831_lcd_backlight_exit(void)
{
	platform_driver_unregister(&maxim8831_lcd_backlight_driver);
}

module_init(maxim8831_lcd_backlight_init);
module_exit(maxim8831_lcd_backlight_exit);

MODULE_AUTHOR("John Chen <jchen1996@gmail.com>");
MODULE_DESCRIPTION("maxim8831 LCD Backlight driver");
MODULE_LICENSE("GPL");
