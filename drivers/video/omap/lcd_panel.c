/*
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
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/console.h>

#include <asm/arch/hardware.h>
#include <asm/arch/board.h>

#include "lcd.h"

#define MOD_NAME 		"LCD: "

#undef MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define DPRINTK(format,...)\
	printk(KERN_INFO MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define DISPLAY_DEVICE_STATE_ON        1
#define DISPLAY_DEVICE_STATE_OFF       0
#define DISPLAY_PANEL_STATE_ON         1
#define DISPLAY_PANEL_STATE_OFF        0
#define DISPLAY_CONTROLLER_STATE_ON    1
#define DISPLAY_CONTROLLER_STATE_OFF   0
#define DISPLAY_BACKLIGHT_STATE_ON     1
#define DISPLAY_BACKLIGHT_STATE_OFF    0


struct lcd_params {
	struct display_device *disp_dev;
	struct platform_device *pdev;

	void *bl_dev;
	struct lcd_backlight_ops *bl_ops;

	void *pnl_dev;
	struct lcd_panel_ops *pnl_ops;

	void *ctrl_dev;
	struct lcd_controller_ops *ctrl_ops;

	int panel_state;

	struct mutex ops_lock;
};

#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
static struct lcd_params *dss_lcd_params;
static void panel_set_state(struct lcd_params *params, unsigned int state);

void omap2_dss_rgb_disable(void)
{
	DPRINTK("%s\n", __FUNCTION__);
	panel_set_state(dss_lcd_params, DISPLAY_DEVICE_STATE_OFF);
}

void omap2_dss_rgb_enable(void)
{
	DPRINTK("%s\n", __FUNCTION__);
	panel_set_state(dss_lcd_params, DISPLAY_DEVICE_STATE_ON);
}

EXPORT_SYMBOL(omap2_dss_rgb_disable);
EXPORT_SYMBOL(omap2_dss_rgb_enable);
#endif

/* local functions */
static void panel_set_state(struct lcd_params *params, unsigned int state)
{
	DPRINTK("%s, state = %d\n", __FUNCTION__, state);

	/* Get console lock to protect against framebuffer ioctl()s.
	 */
	acquire_console_sem();
	mutex_lock(&params->ops_lock);
	if (state) {
		if (params->panel_state == DISPLAY_DEVICE_STATE_ON) {
			DPRINTK(" %s:  Panel already on, returning...\n",
				__FUNCTION__);
			goto unlock;
		}

		/* Controller ON */
		if (params->ctrl_dev && params->ctrl_ops &&
		    params->ctrl_ops->ctrl_set_state) {
			params->ctrl_ops->ctrl_set_state(params->ctrl_dev,
						DISPLAY_CONTROLLER_STATE_ON);
		}

		/* Panel ON */
		if (params->pnl_dev && params->pnl_ops &&
		    params->pnl_ops->pnl_set_state) {
			params->pnl_ops->pnl_set_state(params->pnl_dev,
						DISPLAY_PANEL_STATE_ON);
		}

		/* Backlight ON */
		if (params->bl_dev && params->bl_ops &&
		    params->bl_ops->bl_set_state) {
			params->bl_ops->bl_set_state(params->bl_dev,
						DISPLAY_BACKLIGHT_STATE_ON);
		}
		params->panel_state = DISPLAY_DEVICE_STATE_ON;
	} else {
		if (params->panel_state == DISPLAY_DEVICE_STATE_OFF) {
			DPRINTK(" %s:  Panel already off, returning...\n",
				__FUNCTION__);
			goto unlock;
		}

		/* Backlight OFF */
		if (params->bl_dev && params->bl_ops &&
		    params->bl_ops->bl_set_state) {
			params->bl_ops->bl_set_state(params->bl_dev,
						DISPLAY_BACKLIGHT_STATE_OFF);
		}

		/* Panel OFF */
		if (params->pnl_dev && params->pnl_ops &&
		    params->pnl_ops->pnl_set_state) {
			params->pnl_ops->pnl_set_state(params->pnl_dev,
						DISPLAY_PANEL_STATE_OFF);
		}

		/* Controller OFF */
		if (params->ctrl_dev && params->ctrl_ops &&
		    params->ctrl_ops->ctrl_set_state) {
			params->ctrl_ops->ctrl_set_state(params->ctrl_dev,
						DISPLAY_CONTROLLER_STATE_OFF);
		}
		params->panel_state = DISPLAY_DEVICE_STATE_OFF;
	}

unlock:
	mutex_unlock(&params->ops_lock);
	release_console_sem();
}

static unsigned int panel_get_state(struct lcd_params *params)
{
	DPRINTK("%s\n", __FUNCTION__);

	return params->panel_state;
}

static void panel_set_brightness(struct lcd_params *params, int brightness)
{
	DPRINTK("%s: brightness = %d\n", __FUNCTION__, brightness);

	mutex_lock(&params->ops_lock);
	if (params->bl_dev && params->bl_ops &&
	    params->bl_ops->bl_set_brightness) {
		params->bl_ops->bl_set_brightness(params->bl_dev, brightness);
	}
	mutex_unlock(&params->ops_lock);
}

static int panel_get_brightness(struct lcd_params *params)
{
	int brightness = -1;

	DPRINTK("%s\n", __FUNCTION__);

	mutex_lock(&params->ops_lock);
	if (params->bl_dev && params->bl_ops &&
	    params->bl_ops->bl_get_brightness) {
		brightness =  params->bl_ops->bl_get_brightness(params->bl_dev);
	}
	mutex_unlock(&params->ops_lock);

	return brightness;
}

/*
 * sysfs entries for the display
 */
static ssize_t show_panel_brightness(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct display_device *disp_dev = platform_get_drvdata(pdev);
	struct lcd_params *params = class_get_devdata(&(disp_dev->class_dev));
	int brightness;

	DPRINTK("%s\n", __FUNCTION__);

	brightness = panel_get_brightness(params);

	return snprintf(buf, PAGE_SIZE, "%d\n", brightness);

}
static ssize_t store_panel_brightness(struct device *dev,
					struct device_attribute *dev_attr,
					const char* buf,
					size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct display_device *disp_dev = platform_get_drvdata(pdev);
	struct lcd_params *params = class_get_devdata(&(disp_dev->class_dev));
	char *endp;
	int panel_brightness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	panel_set_brightness(params, panel_brightness);

	return count;

}
DEVICE_ATTR(panel_brightness, S_IRUGO | S_IWUSR, show_panel_brightness,
						store_panel_brightness);

static ssize_t show_panel_state(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct display_device *disp_dev = platform_get_drvdata(pdev);
	struct lcd_params *params = class_get_devdata(&(disp_dev->class_dev));

	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%d\n", panel_get_state(params));

}
static ssize_t store_panel_state(struct device *dev,
					struct device_attribute *dev_attr,
					const char* buf,
					size_t count)
{
	struct platform_device *pdev    = to_platform_device(dev);
	struct display_device *disp_dev = platform_get_drvdata(pdev);
	struct lcd_params *params = class_get_devdata(&(disp_dev->class_dev));

	char *endp;
	int panel_state = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	panel_set_state(params, panel_state);

	return count;
}
DEVICE_ATTR(panel_state, S_IRUGO | S_IWUSR, show_panel_state, store_panel_state);

/*
 * registration functions for various components of the LCD
 */

/* backlight registrations */
void add_backlight_device(struct device *dev, struct lcd_backlight_ops *ops,
						struct device *parent)
{
	struct display_device *disp_dev = dev_get_drvdata(parent);
	struct lcd_params *params = class_get_devdata(
							&(disp_dev->class_dev));

	/* need to get the lcd_params ptr */
	DPRINTK("%s\n", __FUNCTION__);
	DPRINTK("%s: params at 0x%x\n", __FUNCTION__, (int)params);

	params->bl_dev = dev;
	params->bl_ops = ops;
}
EXPORT_SYMBOL(add_backlight_device);

void remove_backlight_device(struct device *dev, struct device *parent)
{
	struct display_device *disp_dev = dev_get_drvdata(parent);
	struct lcd_params *params = class_get_devdata(
							&(disp_dev->class_dev));

	params->bl_dev = NULL;
	params->bl_ops = NULL;
}
EXPORT_SYMBOL(remove_backlight_device);

/* panel registration */
void add_panel_device(struct device *dev, struct lcd_panel_ops *ops,
						struct device *parent)
{
	struct display_device *disp_dev = dev_get_drvdata(parent);
	struct lcd_params *params = class_get_devdata(
							&(disp_dev->class_dev));

	/* need to get the lcd_params ptr */
	DPRINTK("%s\n", __FUNCTION__);

	params->pnl_dev = dev;
	params->pnl_ops = ops;
}
EXPORT_SYMBOL(add_panel_device);

void remove_panel_device(struct device *dev, struct device *parent)
{
	struct display_device *disp_dev = dev_get_drvdata(parent);
	struct lcd_params *params = class_get_devdata(
							&(disp_dev->class_dev));

	params->pnl_dev = NULL;
	params->pnl_ops = NULL;
}
EXPORT_SYMBOL(remove_panel_device);

/* panel registration */
void add_controller_device(struct device *dev, struct lcd_controller_ops *ops,
						struct device *parent)
{
	struct display_device *disp_dev = dev_get_drvdata(parent);
	struct lcd_params *params = class_get_devdata(
							&(disp_dev->class_dev));

	/* need to get the lcd_params ptr */
	DPRINTK("%s\n", __FUNCTION__);

	params->ctrl_dev = dev;
	params->ctrl_ops = ops;
}
EXPORT_SYMBOL(add_controller_device);

void remove_controller_device(struct device *dev, struct device *parent)
{
	struct display_device *disp_dev = dev_get_drvdata(parent);
	struct lcd_params *params = class_get_devdata(
							&(disp_dev->class_dev));

	params->ctrl_dev = NULL;
	params->ctrl_ops = NULL;
}
EXPORT_SYMBOL(remove_controller_device);

/*
 * functions to support the lcd_class
 */
static void lcd_set_brightness(struct display_device *dev, int brightness)
{
	struct lcd_params *params = class_get_devdata(&(dev->class_dev));

	DPRINTK("%s\n", __FUNCTION__);

	panel_set_brightness(params, brightness);
}
static int lcd_get_brightness(struct display_device *dev)
{
	struct lcd_params *params = class_get_devdata(&(dev->class_dev));

	DPRINTK("%s\n", __FUNCTION__);

	return panel_get_brightness(params);
}

static void lcd_set_state(struct display_device *dev, unsigned int state)
{
	struct lcd_params *params = class_get_devdata(&(dev->class_dev));

	DPRINTK("%s\n", __FUNCTION__);

	panel_set_state(params, state);
}

static int lcd_get_state(struct display_device *dev)
{
	struct lcd_params *params = class_get_devdata(&(dev->class_dev));

	DPRINTK("%s\n", __FUNCTION__);

	return panel_get_state(params);
}

static struct display_device_ops lcd_panel_ops = {
	.display_device_set_state 	= lcd_set_state,
	.display_device_get_state	= lcd_get_state,
	.display_device_set_brightness	= lcd_set_brightness,
	.display_device_get_brightness	= lcd_get_brightness,
};

#ifdef CONFIG_PM
#  if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
	/* For the above machines suspend/resume is handled in:
	 *     omap2_dss_rgb_disable(void)
	 *     omap2_dss_rgb_enable(void)
	 * Do not implement suspend/resume here.
	 */
#    define lcd_resume	NULL

static int lcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct display_device *dev = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	/* Turn off the LCD at this point. It might already be off in which
	 * case the extra call won't hurt. However, if it's still on, this call
	 * will save the LCD context now. This will prevent any bad LCD state
	 * on wake-up.
	 */
	lcd_set_state(dev, 0);
	return 0;
}
#  else
static int lcd_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct display_device *dev = platform_get_drvdata(pdev);

	DPRINTK("%s - do nothing\n", __FUNCTION__);

//	lcd_set_state(dev, 0);
	return 0;
}

static int lcd_resume(struct platform_device *pdev)
{
	struct display_device *dev = platform_get_drvdata(pdev);

	DPRINTK("%s - do nothing\n", __FUNCTION__);

// 	lcd_set_state(dev, 1);
	return 0;
}
#  endif
#else
#define lcd_suspend	NULL
#define lcd_resume	NULL
#endif

static int lcd_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct lcd_params *params;
	char display_name[16];

	DPRINTK("%s\n", __FUNCTION__);

	/* register display */
	params = kzalloc(sizeof(struct lcd_params), GFP_KERNEL);
	if (unlikely(!params)) {
		ret = -ENOMEM;
		goto err0;
	}
	memset(params, 0, sizeof(struct lcd_params));
#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
	dss_lcd_params = params;
#endif

	DPRINTK("%s: params at 0x%x\n", __FUNCTION__, (int)params);
	DPRINTK("%s: pdev at 0x%x, dev at 0x%x\n",
				__FUNCTION__, (int)pdev, (int)(&pdev->dev));

	params->pdev = pdev;
	mutex_init(&params->ops_lock);

	snprintf(display_name, 16, "%s.%d", pdev->name, pdev->id);

	params->disp_dev = display_device_register(	display_name,
							&pdev->dev,
							&lcd_panel_ops,
							params);
	if (IS_ERR(params->disp_dev)) {
		ret = PTR_ERR(params->disp_dev);
		goto err1;
	}

	platform_set_drvdata(pdev, params->disp_dev);

	/* create display specific sysfs entries */
	if (device_create_file(&pdev->dev, &dev_attr_panel_state) < 0) {
		printk(KERN_ERR MOD_NAME
			"%s: Error adding sysfs-name\n", __FUNCTION__);
		goto err1;
	}
	if (device_create_file(&pdev->dev, &dev_attr_panel_brightness) < 0) {
		printk(KERN_ERR MOD_NAME
			"%s: Error adding sysfs-name\n", __FUNCTION__);
		goto err1;
	}

	/* initialize to ON because it was turned on in Bootie */
	// params->panel_state = DISPLAY_PANEL_STATE_ON;

	DPRINTK("%s: Display register ok\n", __FUNCTION__);

	return 0;

err1:
	device_remove_file(&pdev->dev, &dev_attr_panel_state);
	device_remove_file(&pdev->dev, &dev_attr_panel_brightness);

err0:
	kfree(params);
	printk(KERN_ERR MOD_NAME "%s: Error inializing display\n", __FUNCTION__);
	return ret;

}

static int lcd_remove(struct platform_device *pdev)
{
	struct display_device *dev = dev_get_drvdata(&(pdev->dev));
	struct lcd_params *params = class_get_devdata(&(dev->class_dev));

	DPRINTK("%s\n", __FUNCTION__);

	device_remove_file(&pdev->dev, &dev_attr_panel_state);
	device_remove_file(&pdev->dev, &dev_attr_panel_brightness);

	display_device_unregister(dev);

	kfree(params);

	return 0;
}

static void lcd_shutdown(struct platform_device *pdev)
{
	struct display_device *dev = dev_get_drvdata(&(pdev->dev));
	struct lcd_params *params = class_get_devdata(&(dev->class_dev));

	DPRINTK("%s\n", __FUNCTION__);

	panel_set_state(params, DISPLAY_DEVICE_STATE_OFF);
}

static struct platform_driver lcd_driver = {
	.probe = lcd_probe,
	.remove = lcd_remove,
	.shutdown = lcd_shutdown,
	.suspend = lcd_suspend,
	.resume = lcd_resume,
	.driver = {
		   .name = "lcd",
	},
};

static int __init lcd_init(void)
{
	DPRINTK("%s\n", __FUNCTION__);
	return platform_driver_register(&lcd_driver);
}

static void __exit lcd_exit(void)
{
	DPRINTK("%s\n", __FUNCTION__);
	platform_driver_unregister(&lcd_driver);
}

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("John Chen <jchne1996@gmail.com>");
MODULE_DESCRIPTION("LCD driver");
MODULE_LICENSE("GPL");
