/*
 * drivers/video/backlight/joplin_castle-backlight.c
 *
 * Backlight driver for OMAP based boards.
 *
 * Copyright (c) 2008 John Chen  <jchen1996@gmail.com>
 * Copyright (c) 2008 Damian Kowalewski  <damian.kowalewski@palm.com>
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

struct joplin_castle_lcd_backlight {
	int timer_id;
	int i2c_id;
	int led_id;
	int powermode;
	int cached;		/* current brightness setting */
	int min_brightness;	/* minimum brightness setting */
	int max_brightness;	/* maximum brightness setting */
	int on_value;		/* value to set brightness to when it turn on */
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

static void inline joplin_castle_lcd_backlight_on(
					struct joplin_castle_lcd_backlight *bl)
{
	DPRINTK("%s\n", __FUNCTION__);

	omap_dm_timer_enable(bl->timer);
	omap_dm_timer_set_source(bl->timer, OMAP_TIMER_SRC_32_KHZ);
	omap_dm_timer_set_load(bl->timer, 1, 0xffffff00);
}

static void inline joplin_castle_lcd_backlight_send_brightness(
					struct joplin_castle_lcd_backlight *bl,
					int brightness)
{
	DPRINTK("%s\n", __FUNCTION__);

	if (brightness == bl->cached)
		return;

	if (brightness < bl->min_brightness)
		brightness = bl->min_brightness;
	else if (brightness > bl->max_brightness)
		brightness = bl->max_brightness;

	if (brightness == bl->min_brightness) {
		omap_dm_timer_set_pwm(bl->timer, 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(bl->timer);
		omap_dm_timer_disable(bl->timer);

	} else if (brightness == bl->max_brightness) {
		if (0 == bl->cached)
			joplin_castle_lcd_backlight_on(bl);
		omap_dm_timer_set_pwm(bl->timer, 1, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(bl->timer);
	} else {
		if (0 == bl->cached)
			joplin_castle_lcd_backlight_on(bl);
		omap_dm_timer_set_pwm(bl->timer, 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_set_match(bl->timer, 1,
					(0xffffff00) | brightness);
		omap_dm_timer_start(bl->timer);
	}

	maxim8831_set_brightness_exported(bl->i2c_id, bl->led_id, brightness);

	bl->cached = brightness;
}

static void joplin_castle_lcd_backlight_blank(
				struct joplin_castle_lcd_backlight *bl, int mode)
{
	DPRINTK("%s\n", __FUNCTION__);

	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		joplin_castle_lcd_backlight_send_brightness(bl, 0);
		break;

	case FB_BLANK_UNBLANK:
		joplin_castle_lcd_backlight_send_brightness(bl, bl->on_value);
		break;
	}
}

#ifdef CONFIG_PM
static int joplin_castle_lcd_backlight_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

	joplin_castle_lcd_backlight_blank(bl, FB_BLANK_POWERDOWN);
	return 0;
}

static int joplin_castle_lcd_backlight_resume(struct platform_device *pdev)
{
	struct joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

	joplin_castle_lcd_backlight_blank(bl, bl->powermode);
	return 0;
}
#else
#define joplin_castle_lcd_backlight_suspend	NULL
#define joplin_castle_lcd_backlight_resume	NULL
#endif

/*
 * Backlight specific sysfs attributes
 */
static ssize_t backlight_show_brightness(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

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
	struct joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);
	int brightness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	joplin_castle_lcd_backlight_send_brightness(bl, brightness);
	bl->on_value = brightness;

	return count;
}
DEVICE_ATTR(brightness, S_IRUGO|S_IWUSR, backlight_show_brightness,
					backlight_store_brightness);

static ssize_t backlight_show_fade_msecs(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

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
	struct joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);
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

static void joplin_castle_lcd_backlight_set_state(void *dev, unsigned int state)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);
	DPRINTK("%s dev at 0x%x, pdev at 0x%x, bl at 0x%x\n",
				__FUNCTION__, (int)dev, (int)pdev, (int)bl);

	if (state) {
		joplin_castle_lcd_backlight_blank(bl, FB_BLANK_UNBLANK);
	} else {
		bl->on_value = bl->cached;
		joplin_castle_lcd_backlight_blank(bl, FB_BLANK_POWERDOWN);
	}
}
static void joplin_castle_lcd_backlight_set_brightness(void *dev, int brightness)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

	joplin_castle_lcd_backlight_send_brightness(bl, brightness);

	DPRINTK("%s:  send brightness %d\n", __FUNCTION__, brightness);
}
static int joplin_castle_lcd_backlight_get_brightness(void *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

	DPRINTK("%s: got brightness %d\n", __FUNCTION__, bl->cached);

	return bl->cached;
}
static void joplin_castle_lcd_backlight_set_max_brightness(void *dev)
{
	struct platform_device* pdev = to_platform_device(dev);
	struct  joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

	joplin_castle_lcd_backlight_send_brightness(bl, bl->max_brightness);
}

static struct lcd_lcd_backlight_ops joplin_castle_lcd_lcd_backlight_ops = {
	.bl_set_state = joplin_castle_lcd_backlight_set_state,
	.bl_set_brightness = joplin_castle_lcd_backlight_set_brightness,
	.bl_get_brightness = joplin_castle_lcd_backlight_get_brightness,
	.bl_set_max_brightness = joplin_castle_lcd_backlight_set_max_brightness,
};

static int joplin_castle_lcd_backlight_probe(struct platform_device *pdev)
{
	int result = 0;
	struct joplin_castle_lcd_backlight *bl;
	struct bl_platform_data *bl_plat;

	DPRINTK("%s\n", __FUNCTION__);

	bl = kzalloc(sizeof(struct joplin_castle_lcd_backlight), GFP_KERNEL);
	if (unlikely(!bl)) {
		result = -ENOMEM;
		goto error_memory;
	}

	platform_set_drvdata(pdev, bl);

	if (pdev->dev.platform_data != NULL) {
		bl_plat = (struct bl_platform_data*)pdev->dev.platform_data;
		bl->timer_id = bl_plat->timer_id;
		bl->min_brightness = bl_plat->min_brightness;
		bl->max_brightness = bl_plat->max_brightness;
		bl->on_value = bl->max_brightness;
		bl->parent = bl_plat->parent;
		bl->i2c_id = bl_plat->i2c_id;
		bl->led_id = bl_plat->led_id;
		bl->fade_msecs = bl_plat->fade_msecs;

		DPRINTK("Backlight:  timer = %d, min=%d max = %d\n",
				bl->timer_id, bl->min_brightness,
				bl->max_brightness);
	} else {
		DPRINTK("Error:  No Timer ID available\n");
		goto error_register;
	}

	/* register the device and the ops with lcd_main */
	lcd_add_backlight(&pdev->dev, &joplin_castle_lcd_lcd_backlight_ops, bl->parent);

	bl->powermode = FB_BLANK_POWERDOWN;

	/* get related dm timers */
	bl->timer = omap_dm_timer_request_specific(bl->timer_id);
	if (bl->timer == NULL) {
		result = -ENODEV;
		goto error_register;
	}
	omap_dm_timer_disable(bl->timer);

	bl->dev = &pdev->dev;

	/* turn the backlight on initially */
	joplin_castle_lcd_backlight_send_brightness(bl, bl->max_brightness);

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

	printk(KERN_INFO "Joplin_Castle LCD backlight initialized\n");

	return 0;

error_sysfs:
	device_remove_file(&pdev->dev, &dev_attr_brightness);
	device_remove_file(&pdev->dev, &dev_attr_fade_msecs);

error_register:
	kfree(bl);

error_memory:
	return result;

}

static int joplin_castle_lcd_backlight_remove(struct platform_device *pdev)
{
	struct joplin_castle_lcd_backlight *bl = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_brightness);
	device_remove_file(&pdev->dev, &dev_attr_fade_msecs);

	lcd_remove_backlight(&pdev->dev, bl->parent);
	kfree(bl);

	printk(KERN_INFO "Joplin LCD backlight removed\n");

	return 0;
}

static struct platform_driver joplin_castle_lcd_backlight_driver = {
	.probe = joplin_castle_lcd_backlight_probe,
	.remove = joplin_castle_lcd_backlight_remove,
	.suspend = joplin_castle_lcd_backlight_suspend,
	.resume = joplin_castle_lcd_backlight_resume,
	.driver = {
		   .name = "lcd-backlight",
	},
};

static int __init joplin_castle_lcd_backlight_init(void)
{
	return platform_driver_register(&joplin_castle_lcd_backlight_driver);
}

static void __exit joplin_castle_lcd_backlight_exit(void)
{
	platform_driver_unregister(&joplin_castle_lcd_backlight_driver);
}

module_init(joplin_castle_lcd_backlight_init);
module_exit(joplin_castle_lcd_backlight_exit);

MODULE_AUTHOR("John Chen <jchen1996@gmail.com>");
MODULE_DESCRIPTION("Joplin_Castle LCD Backlight driver");
MODULE_LICENSE("GPL");
