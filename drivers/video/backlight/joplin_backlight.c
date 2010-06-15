/*
 * drivers/video/backlight/joplin-backlight.c
 *
 * Backlight driver for OMAP based boards.
 *
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
#if defined(CONFIG_MACH_BRISKET)
#define DM_TIMER_ID		12
#endif
#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
#define DM_TIMER_ID		9
#endif

struct joplin_lcd_backlight {
	int powermode;
	int current_brightness;
	int cached;
	struct omap_dm_timer *timer;

	struct device *pdev;
	struct omap_backlight_config *pdata;
};

static void inline joplin_lcd_backlight_on(struct joplin_lcd_backlight *bl)
{
	omap_dm_timer_enable(bl->timer);
	omap_dm_timer_set_source(bl->timer, OMAP_TIMER_SRC_32_KHZ);
	omap_dm_timer_set_load(bl->timer, 1, 0xffffff00);
}

static void inline joplin_lcd_backlight_send_brightness(struct
							joplin_lcd_backlight
							*bl, int brightness)
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
			joplin_lcd_backlight_on(bl);
		omap_dm_timer_set_pwm(bl->timer, 1, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_stop(bl->timer);
		break;
	default:
		if (0 == bl->cached)
			joplin_lcd_backlight_on(bl);
		omap_dm_timer_set_pwm(bl->timer, 0, 1,
				      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
		omap_dm_timer_set_match(bl->timer, 1,
					(0xffffff00) | brightness);
		omap_dm_timer_start(bl->timer);

		break;
	}
	bl->cached = brightness;
}

static void joplin_lcd_backlight_blank(struct joplin_lcd_backlight *bl,
				       int mode)
{
	if (bl->pdata->set_power)
		bl->pdata->set_power(bl->pdev, mode);

	switch (mode) {
	case FB_BLANK_NORMAL:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_POWERDOWN:
		joplin_lcd_backlight_send_brightness(bl, 0);
		break;

	case FB_BLANK_UNBLANK:
		joplin_lcd_backlight_send_brightness(bl,
						     bl->current_brightness);
		break;
	}
}

#ifdef CONFIG_PM
static int joplin_lcd_backlight_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct backlight_device *bldev  = platform_get_drvdata(pdev);
	struct joplin_lcd_backlight *bl = dev_get_drvdata(&bldev->dev);

	joplin_lcd_backlight_blank(bl, FB_BLANK_POWERDOWN);
	return 0;
}

static int joplin_lcd_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bldev  = platform_get_drvdata(pdev);
	struct joplin_lcd_backlight *bl = dev_get_drvdata(&bldev->dev);

	joplin_lcd_backlight_blank(bl, bl->powermode);
	return 0;
}
#else
#define joplin_lcd_backlight_suspend	NULL
#define joplin_lcd_backlight_resume	NULL
#endif

static int joplin_lcd_backlight_set_power(struct backlight_device *bldev,
					  int state)
{
	struct joplin_lcd_backlight *bl = dev_get_drvdata(&bldev->dev);

	joplin_lcd_backlight_blank(bl, state);
	bl->powermode = state;

	return 0;
}

static int joplin_lcd_backlight_update_status(struct backlight_device *bldev)
{
	struct joplin_lcd_backlight *bl = dev_get_drvdata(&bldev->dev);

	if (bl->current_brightness != bldev->props.brightness) {
		if (bldev->props.brightness < 0)
			return -EINVAL;	/* Leave current_brightness untouched */

		if (bl->powermode == FB_BLANK_UNBLANK)
			joplin_lcd_backlight_send_brightness(bl,
							     bldev->props.
							     brightness);
		bl->current_brightness = bldev->props.brightness;
	}

	if (bldev->props.fb_blank != bl->powermode)
		joplin_lcd_backlight_set_power(bldev, bldev->props.fb_blank);

	return 0;
}

static int joplin_lcd_backlight_get_intensity(struct backlight_device *bldev)
{
	struct joplin_lcd_backlight *bl = dev_get_drvdata(&bldev->dev);
	return bl->current_brightness;
}

static struct backlight_ops joplin_lcd_backlight_ops = {
	.get_brightness = joplin_lcd_backlight_get_intensity,
	.update_status  = joplin_lcd_backlight_update_status,
};

static int joplin_lcd_backlight_probe(struct platform_device *pdev)
{
	int result = 0;
	struct backlight_device *bldev;
	struct joplin_lcd_backlight *bl;
	struct omap_backlight_config *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -ENXIO;

	joplin_lcd_backlight_ops.check_fb = pdata->check_fb;

	bl = kzalloc(sizeof(struct joplin_lcd_backlight), GFP_KERNEL);
	if (unlikely(!bl)) {
		result = -ENOMEM;
		goto error_memory;
	}

	bldev = backlight_device_register("lcd", &pdev->dev,
					   bl, &joplin_lcd_backlight_ops);
	if (IS_ERR(bldev)) {
		result = PTR_ERR(bldev);
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
	bl->pdev  = &pdev->dev;

	platform_set_drvdata(pdev, bldev);

	bldev->props.fb_blank = FB_BLANK_UNBLANK;
	bldev->props.max_brightness = MAX_INTENSITY;
	bldev->props.brightness = pdata->default_intensity;

	joplin_lcd_backlight_update_status(bldev);

	printk(KERN_INFO "Joplin LCD backlight initialized\n");

	return 0;

error_timer:
	backlight_device_unregister(bldev);

error_register:
	kfree(bl);

error_memory:
	return result;

}

static int joplin_lcd_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev  = platform_get_drvdata(pdev);
	struct joplin_lcd_backlight *bl = dev_get_drvdata(&bldev->dev);

	backlight_device_unregister(bldev);

	omap_dm_timer_free(bl->timer);
	kfree(bl);

	return 0;
}

static struct platform_driver joplin_lcd_backlight_driver = {
	.probe = joplin_lcd_backlight_probe,
	.remove = joplin_lcd_backlight_remove,
	.suspend = joplin_lcd_backlight_suspend,
	.resume = joplin_lcd_backlight_resume,
	.driver = {
		   .name = "joplin-backlight",
		   },
};

static int __init joplin_lcd_backlight_init(void)
{
	return platform_driver_register(&joplin_lcd_backlight_driver);
}

static void __exit joplin_lcd_backlight_exit(void)
{
	platform_driver_unregister(&joplin_lcd_backlight_driver);
}

module_init(joplin_lcd_backlight_init);
module_exit(joplin_lcd_backlight_exit);

MODULE_AUTHOR("Damian Kowalewski <damian.kowalewski@palm.com>");
MODULE_DESCRIPTION("Joplin LCD Backlight driver");
MODULE_LICENSE("GPL");
