/*
 * drivers/video/backlight/joplin-backlight.c
 *
 * Backlight driver for Palm's OMAP based boards.
 *
 * Copyright (c) 2008 John Chen  <john.chen@palm.com>
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
#include <linux/backlight.h>

#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/mux.h>
#include <asm/arch/dmtimer.h>

#define MAX_INTENSITY		0xff
#define DM_TIMER_ID		9

struct castle_bl {
	int powermode;
	int current_brightness;
	int cached;
	struct omap_dm_timer *timer;

	struct device *dev;
	struct omap_backlight_config *pdata;
};

static void inline palm_omap_lcd_backlight_on(struct castle_bl *bl)
{
	omap_dm_timer_enable(bl->timer);
	omap_dm_timer_set_source(bl->timer, OMAP_TIMER_SRC_32_KHZ);
	omap_dm_timer_set_load(bl->timer, 1, 0xffffff00);
}

static void inline castle_bl_send_brightness(struct castle_bl *bl, 
							int brightness)
{
	if (brightness == bl->cached)
		return;

	switch (brightness) {
	case 0:
		omap_dm_timer_set_pwm(bl->timer, 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(bl->timer);
		omap_dm_timer_disable(bl->timer);
		break;
	case MAX_INTENSITY:
		if (0 == bl->cached)
			palm_omap_lcd_backlight_on(bl);
		omap_dm_timer_set_pwm(bl->timer, 1, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(bl->timer);
		break;
	default:
		if (0 == bl->cached)
			palm_omap_lcd_backlight_on(bl);
		omap_dm_timer_set_pwm(bl->timer, 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_set_match(bl->timer, 1,
					(0xffffff00) | brightness);
		omap_dm_timer_start(bl->timer);

		break;
	}
	bl->cached = brightness;
}

static void castle_bl_blank(struct castle_bl *bl, int mode)
{
	if (bl->pdata->set_power)
		bl->pdata->set_power(bl->dev, mode);

	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		castle_bl_send_brightness(bl, 0);
		break;

	case FB_BLANK_UNBLANK:
		castle_bl_send_brightness(bl, bl->current_brightness);
		break;
	}
}

#ifdef CONFIG_PM
static int castle_bl_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct backlight_device *bldev = platform_get_drvdata(pdev);
	struct castle_bl *bl = dev_get_drvdata(&bldev->dev);

	castle_bl_blank(bl, FB_BLANK_POWERDOWN);
	return 0;
}

static int castle_bl_resume(struct platform_device *pdev)
{
	struct backlight_device *bldev = platform_get_drvdata(pdev);
	struct castle_bl *bl = dev_get_drvdata(&bldev->dev);

	castle_bl_blank(bl, bl->powermode);
	return 0;
}
#else
#define castle_bl_suspend	NULL
#define castle_bl_resume	NULL
#endif

static int castle_bl_set_power(struct backlight_device *bldev,
					  int state)
{
	struct castle_bl *bl = dev_get_drvdata(&bldev->dev);

	castle_bl_blank(bl, state);
	bl->powermode = state;

	return 0;
}

static int castle_bl_update_status(struct backlight_device *bldev)
{
	struct castle_bl *bl = dev_get_drvdata(&bldev->dev);

	if (bl->current_brightness != bldev->props.brightness) {
		if (bldev->props.brightness < 0)
			return -EINVAL;	/* Leave current_brightness untouched */

		if (bl->powermode == FB_BLANK_UNBLANK)
			castle_bl_send_brightness(bl, bldev->props.brightness);
		bl->current_brightness = bldev->props.brightness;
	}

	if (bldev->props.fb_blank != bl->powermode)
		castle_bl_set_power(bldev, bldev->props.fb_blank);

	return 0;
}

static int castle_bl_get_intensity(struct backlight_device *bldev)
{
	struct castle_bl *bl = dev_get_drvdata(&bldev->dev);
	return bl->current_brightness;
}

static struct backlight_ops castle_bl_ops = {
	.get_brightness = castle_bl_get_intensity,
	.update_status = castle_bl_update_status,
};

static int castle_bl_probe(struct platform_device *pdev)
{
	int result = 0;
	struct backlight_device *dev;
	struct castle_bl *bl;
	struct omap_backlight_config *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -ENXIO;

	castle_bl_ops.check_fb = pdata->check_fb;

	bl = kzalloc(sizeof(struct castle_bl), GFP_KERNEL);
	if (unlikely(!bl)) {
		result = -ENOMEM;
		goto error_memory;
	}

	dev = backlight_device_register("lcd", &pdev->dev,
					bl, &castle_bl_ops);
	if (IS_ERR(dev)) {
		result = PTR_ERR(dev);
		goto error_register;
	}

	bl->powermode = FB_BLANK_POWERDOWN;
	bl->current_brightness = 0;

	/* get related dm timers */
	bl->timer = omap_dm_timer_request_specific(DM_TIMER_ID);
	if (bl->timer == NULL) {
		result = -ENODEV;
		goto error_timer;
	}
	omap_dm_timer_disable(bl->timer);

	bl->pdata = pdata;
	bl->dev = &pdev->dev;

	platform_set_drvdata(pdev, dev);

	dev->props.fb_blank = FB_BLANK_UNBLANK;
	dev->props.max_brightness = MAX_INTENSITY;
	dev->props.brightness = pdata->default_intensity;

	castle_bl_update_status(dev);

	printk(KERN_INFO "Palm OMAP LCD backlight initialized\n");

	return 0;

error_timer:
	backlight_device_unregister(dev);

error_register:
	kfree(bl);

error_memory:

	return result;

}

static int castle_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev = platform_get_drvdata(pdev);
	struct castle_bl *bl = dev_get_drvdata(&bldev->dev);

	backlight_device_unregister(bldev);

	omap_dm_timer_free(bl->timer);
	kfree(bl);

	return 0;
}

static struct platform_driver castle_bl_driver = {
	.probe   = castle_bl_probe,
	.remove  = castle_bl_remove,
	.suspend = castle_bl_suspend,
	.resume  = castle_bl_resume,
	.driver  = {
		   .name = "omap-bl",
	},
};

static int __init castle_bl_init(void)
{
	return platform_driver_register(&castle_bl_driver);
}

static void __exit castle_bl_exit(void)
{
	platform_driver_unregister(&castle_bl_driver);
}

module_init(castle_bl_init);
module_exit(castle_bl_exit);

MODULE_AUTHOR("Damian Kowalewski <damian.kowalewski@palm.com>");
MODULE_DESCRIPTION("Palm OMAP LCD Backlight driver");
MODULE_LICENSE("GPL");
