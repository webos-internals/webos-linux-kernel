/*
 * drivers/video/omap/lcd_class.c
 *
 * display class implementation for Palm's LCD's
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
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/err.h>

#include "lcd.h"

#define MOD_NAME 			"LCD Class: "
#define DISP_NAME_SIZE			16

#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define DPRINTK(format,...)\
	printk(KERN_INFO MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define LCD_STATE_ON		 1
#define LCD_STATE_OFF		 0
#define LCD_STATE_ERROR		-1

/*
 * sysfs entries for the display class devices
 */
static ssize_t show_class_device_state(struct class_device *cdev,
				char *buf)
{
	struct display_device *disp = to_display_device(cdev);
	int state;

	DPRINTK("%s\n", __FUNCTION__);


	if (disp->ops && disp->ops->display_device_get_state) {
		state = disp->ops->display_device_get_state(disp);
	} else {
		state = LCD_STATE_ERROR;
		printk(KERN_ERR MOD_NAME "Error: No get_state ops\n");
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", state);
}
static ssize_t store_class_device_state(struct class_device *cdev,
				const char *buf,
				size_t count)
{
	char *endp;
	struct display_device *disp = to_display_device(cdev);
	int disp_state = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;
	int ret = count;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (disp->ops && disp->ops->display_device_set_state) {
		if (disp_state) {
			disp->ops->display_device_set_state(disp, LCD_STATE_ON);
		} else {
			disp->ops->display_device_set_state(disp, LCD_STATE_OFF);
		}
	} else {
		printk(KERN_ERR "Error: No set_state ops\n");
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t show_class_device_brightness(struct class_device *cdev,
				char *buf)
{
	struct display_device *disp = to_display_device(cdev);
	int brightness = -1;

	DPRINTK("%s\n", __FUNCTION__);


	if (disp->ops && disp->ops->display_device_get_state) {
		brightness = disp->ops->display_device_get_brightness(disp);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", brightness);
}
static ssize_t store_class_device_brightness(struct class_device *cdev,
				const char *buf,
				size_t count)
{
	char *endp;
	struct display_device *disp = to_display_device(cdev);
	int brightness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (disp->ops && disp->ops->display_device_set_brightness) {
		disp->ops->display_device_set_brightness(disp, brightness);
	}

	return count;
}

static struct class_device_attribute disp_class_device_attributes[] = {
	__ATTR(state, S_IRUGO|S_IWUSR, show_class_device_state,
					store_class_device_state),
	__ATTR(brightness, S_IRUGO|S_IWUSR, show_class_device_brightness,
					store_class_device_brightness),
};

static void display_class_release(struct class_device *cdev)
{
	DPRINTK("%s\n", __FUNCTION__);
	/* jsc - not sure what to put in here */
};

static struct class display_class = {
	.name = "display",
	.release = display_class_release,
};

/*
 * display_device_unregister - unregister a "device" with the class
 * @dev: the device to register
 *
 * display_device_unregister must be called when the class device is
 * no longer needed.
 *
 * returns the pointer to the new struct class device.
 */
void display_device_unregister(struct display_device *disp)
{
	int i;

	DPRINTK("%s\n", __FUNCTION__);

	if (!disp)
		return;

	for (i=0; i<ARRAY_SIZE(disp_class_device_attributes); i++) {
		class_device_remove_file(&disp->class_dev,
					&disp_class_device_attributes[i]);
	}

	disp->ops = NULL;

	/* un-register the class device */
	class_device_unregister(&disp->class_dev);
	kfree(disp);
}
EXPORT_SYMBOL(display_device_unregister);

/*
 * display_device_register - registers a device with the display class
 * @name: the name of the device to register
 * @dev:  pointer to the device
 * @display_device_ops:  pointer to the operations needed by the class
 * @devdata:  device-specific private data
 *
 * display_device_unregister must be called when the class device is
 * no longer needed.
 *
 * returns the pointer to the new struct class device.
 */
struct display_device *display_device_register(const char *name,
					struct device *dev,
					struct display_device_ops *ops,
					void *devdata)
{
	struct display_device *new_disp;
	int i, ret = -EINVAL;
	char display_name[DISPLAY_NAME_SIZE];

	DPRINTK("%s\n", __FUNCTION__);

	new_disp = kzalloc(sizeof(struct display_device), GFP_KERNEL);
	if (new_disp == NULL) {
		printk(KERN_ERR MOD_NAME "%s: No Memory\n", __FUNCTION__);
		ret = -ENOMEM;
		goto err0;
	}

	mutex_init(&new_disp->ops_lock);

	strlcpy(new_disp->name, name, DISP_NAME_SIZE);

	new_disp->ops = ops;
	new_disp->class_dev.class = &display_class;
	new_disp->class_dev.dev = dev;

	snprintf(display_name, DISP_NAME_SIZE, name);
	strlcpy(new_disp->class_dev.class_id, display_name, KOBJ_NAME_LEN);
	class_set_devdata(&new_disp->class_dev, devdata);

	/* register the device with the class */
	ret = class_device_register(&new_disp->class_dev);
	if (ret) {
		printk(KERN_ERR MOD_NAME "%s: Dev reg failed\n", __FUNCTION__);
		goto err1;
	}

	/* register the sysfs entries for the class device */
	for (i=0; i<ARRAY_SIZE(disp_class_device_attributes); i++) {
		ret = class_device_create_file(&new_disp->class_dev,
					&disp_class_device_attributes[i]);
		if (ret) {
			while(--i >= 0)
				class_device_remove_file(&new_disp->class_dev,
					&disp_class_device_attributes[i]);

			printk(KERN_ERR MOD_NAME
				"%s: Unable to register sysfs for class dev\n",
				__FUNCTION__);
			goto err2;
		}
	}

	DPRINTK("%s: display device %s registered \n", __FUNCTION__, name);

	return new_disp;
err2:
	display_device_unregister(new_disp);
err1:
	kfree(new_disp);
err0:

	printk(KERN_ERR MOD_NAME "Error:  can't register display\n");
	return ERR_PTR(ret);

}
EXPORT_SYMBOL(display_device_register);

#ifdef CONTROL_ALL_CLASS_DEVICES
/*
 * sysfs entries for the display class devices
 */
static ssize_t show_class_name(struct class *class, char *buf)
{
	DPRINTK("%s\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "%s\n", class->name);
}
CLASS_ATTR(name, S_IRUGO, show_class_name, NULL);

static ssize_t show_class_state(struct class *class, char *buf)
{
	struct list_head *item;
	struct class_device *cdev;
	struct platform_device *pdev;
	struct display_device *disp;
	int device_state = LCD_STATE_ERROR;

	DPRINTK("%s\n", __FUNCTION__);

	/* iterate through each device and call its show_state function */
	list_for_each(item, &class->children) {
		cdev = list_entry(item, struct class_device, node);
		pdev = to_platform_device(cdev->dev);
		disp = to_display_device(cdev);

		if (disp->ops && disp->ops->display_device_get_state) {
			device_state = disp->ops->display_device_get_state(disp);
			break;
		}
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", device_state);
}
static ssize_t store_class_state(struct class *class, const char *buf,
				size_t count)
{
	struct list_head *item;
	struct class_device *cdev;
	struct platform_device *pdev;
	struct display_device *disp;
	char *endp;
	int disp_state = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	/* iterate through each device and call its show_state function */
	list_for_each(item, &class->children) {
		cdev = list_entry(item, struct class_device, node);
		pdev = to_platform_device(cdev->dev);
		disp = to_display_device(cdev);

		if (disp->ops && disp->ops->display_device_set_state) {
			if (disp_state) {
				disp->ops->display_device_set_state(disp,
								LCD_STATE_ON);
			} else {
				disp->ops->display_device_set_state(disp,
								LCD_STATE_OFF);
			}
		}

		DPRINTK("%s: Device name: %s.%d\n",
					__FUNCTION__, pdev->name, pdev->id);
	}

	return count;
}
CLASS_ATTR(state, S_IRUGO | S_IWUSR, show_class_state, store_class_state);

static ssize_t show_class_brightness(struct class *class, char *buf)
{
	struct list_head *item;
	struct class_device *cdev;
	struct platform_device *pdev;
	struct display_device *disp;
	int brightness = -1;

	DPRINTK("%s\n", __FUNCTION__);

	/* iterate through each device and call its show_state function */
	list_for_each(item, &class->children) {
		cdev = list_entry(item, struct class_device, node);
		pdev = to_platform_device(cdev->dev);
		disp = to_display_device(cdev);

		if (disp->ops && disp->ops->display_device_get_brightness) {
			brightness = disp->ops->display_device_get_brightness(
									disp);
			break;	/* just need 1 of them */
		}
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", brightness);
}
static ssize_t store_class_brightness(struct class *class, const char *buf,
				size_t count)
{
	struct list_head *item;
	struct class_device *cdev;
	struct platform_device *pdev;
	struct display_device *disp;
	char *endp;
	int brightness = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	/* iterate through each device and call its show_state function */
	list_for_each(item, &class->children) {
		cdev = list_entry(item, struct class_device, node);
		pdev = to_platform_device(cdev->dev);
		disp = to_display_device(cdev);

		if (disp->ops && disp->ops->display_device_set_brightness) {
			disp->ops->display_device_set_brightness(disp,
								brightness);
		}
	}

	return count;
}
CLASS_ATTR(brightness, S_IRUGO | S_IWUSR, show_class_brightness,
						store_class_brightness);

#endif /* CONTROL_ALL_CLASS_DEVICES */

static int __init display_class_init(void)
{
	int rc;

	DPRINTK("%s\n", __FUNCTION__);

	DPRINTK("%s: Display class at 0x%x\n",
				__FUNCTION__, (int)&display_class);

	/* register the display class */
	rc = class_register(&display_class);
	if (rc) {
		printk(KERN_ERR MOD_NAME "Error: class registration failed\n");
	}

#ifdef CONTROL_ALL_CLASS_DEVICES
	rc = class_create_file(&display_class, &class_attr_name);
	if (rc) {
		printk(KERN_ERR MOD_NAME "Error:  class_create_file failed\n");
		goto err0;
	}
	rc = class_create_file(&display_class, &class_attr_state);
	if (rc) {
		printk(KERN_ERR MOD_NAME "Error:  class_create_file failed\n");
		goto err1;
	}
	rc = class_create_file(&display_class, &class_attr_brightness);
	if (rc) {
		printk(KERN_ERR MOD_NAME "Error:  class_create_file failed\n");
		goto err1;
	}
#endif /* CONTROL_ALL_CLASS_DEVICES */
	return rc;

#ifdef CONTROL_ALL_CLASS_DEVICES
err1:
	class_remove_file (&display_class, &class_attr_name);
	class_remove_file (&display_class, &class_attr_state);
	class_remove_file (&display_class, &class_attr_brightness);

err0:
	class_unregister(&display_class);
	return rc;
#endif /* CONTROL_ALL_CLASS_DEVICES */
}

static void __exit display_class_exit(void)
{
	DPRINTK("%s\n", __FUNCTION__);

#ifdef CONTROL_ALL_CLASS_DEVICES
	class_remove_file (&display_class, &class_attr_name);
	class_remove_file (&display_class, &class_attr_state);
	class_remove_file (&display_class, &class_attr_brightness);
#endif /* CONTROL_ALL_CLASS_DEVICES */

	class_unregister(&display_class);
}

/*
 * if this is compiled into the kernel, we need to ensure that the
 * class is registered before users of the class try to register lcd's
 */
postcore_initcall(display_class_init);
module_exit(display_class_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Chen <jchen1996@gmail.com>");
MODULE_DESCRIPTION("display high level abstraction");

