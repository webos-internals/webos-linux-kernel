/*
 * linux/drivers/input/lightsensor/lightsensor.c
 *
 * Ambient Light Sensor driver 
 *
 * Copyright (C) 2008 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/temt6200_lightsensor.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <asm/arch/gpio.h>
#include <asm/arch/menelaus.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/twl4030-madc.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

/* state */
#define STATE_OFF         0
#define STATE_SUSPENDED   1
#define STATE_ON          2

struct temt_ls_state {
	struct input_dev	*idev;
	struct temt6200_platform_data*	plat_data;
	struct mutex		lock;
	int			state;
	int			suspended;
	int			poll_interval;
	struct timer_list	poll_timer;
	struct work_struct	poll_work;
};

/*
 *   Returns const state string 
 */
static inline const char*
get_state_str(struct temt_ls_state* ls)
{
	if (ls->suspended || ls->state == STATE_SUSPENDED )
		return "suspended";

	if (ls->state == STATE_ON)
		return "on";

	if (ls->state == STATE_OFF)
		return "off";

	return "undefined";
}

/*
 *      Read one sample
 */
static int
light_sensor_read_sample ( struct temt_ls_state *ls )
{
	int ret;

	if( ls == NULL )
		return -ENODEV;
	
	mutex_lock(&ls->lock);

	/* enable light sensor */
	if (ls->plat_data->enable_lgt ) {
		(void) ls->plat_data->enable_lgt(1);
	}

	udelay(300);

	ret = madc_start_conversion ( ls->plat_data->channel );

	/* disable light sensor */
	if (ls->plat_data->enable_lgt) {
		(void) ls->plat_data->enable_lgt(0);
	}

	mutex_unlock(&ls->lock);
	
	return ret;
}

static void 
light_sensor_poll_timeout(unsigned long statePtr )
{
	struct temt_ls_state *ls = (struct temt_ls_state *)statePtr;

	/* Queue the work */
	schedule_work(&ls->poll_work); 
}

/*
 *     Read sample sysfs entry point
 */
static ssize_t
show_result( struct device *dev, struct device_attribute *dev_attr, 
             char *buf)
{
	int ret = light_sensor_read_sample ( dev_get_drvdata(dev));
	return  snprintf(buf, PAGE_SIZE, "%d\n", ret) + 1;
}

/*
 *   "average attibute"  
 */
static ssize_t
show_average( struct device *dev, struct device_attribute *dev_attr, 
              char   *buf)
{
	struct  temt_ls_state *ls = dev_get_drvdata(dev);
	
	return snprintf(buf, PAGE_SIZE, "%s\n", 
	                ls->plat_data->average ? "enabled" : "disabled" ) + 1;
}

static ssize_t
store_average( struct device *dev, struct device_attribute *dev_attr, 
               const char *buf, size_t count)
{
	char cmd[8];
	int i = 0, ret;
	struct temt_ls_state* state;

	if (!buf || !count) 
		return -EINVAL; 
	
	state = (struct temt_ls_state*)dev->driver_data;

	while (buf[i] != ' ' && buf[i] != '\n' && i < count) {
		cmd[i] = buf[i];
		i++;
	}
	cmd[i] = '\0';
	i++;

	/* Enable/disable average value option */
	if (!strcmp(cmd, "1") || !strcmp(cmd, "enabled")) {
		ret = madc_enable_averaging(state->plat_data->channel, 1);
		state->plat_data->average = 1;
	} 
	else if (!strcmp(cmd, "0") || !strcmp(cmd, "disabled")) {
		ret = madc_enable_averaging(state->plat_data->channel, 0);
		state->plat_data->average = 0;
	} 
	else {
		ret = -EINVAL;
	}

	if (ret < 0)
		return ret;
	else
		return count;
}

/*
 *   "mode" attribute
 */
static ssize_t
show_mode_attr ( struct device *dev,   struct device_attribute *dev_attr, 
                 char *buf)
{
	// this driver only supports poll mode
	return snprintf(buf, PAGE_SIZE, "poll\n")+1;
}

static ssize_t 
store_mode_attr ( struct device *dev, struct device_attribute *dev_attr, 
                  const char *buf, size_t count)
{
	if (!buf || !count) 
		return -EINVAL; 

	// this driver only supports poll mode
	return count;
}


/*
 *  "state" attribute (on, off, suspended)
 */
static ssize_t
show_state_attr( struct device *dev, struct device_attribute *dev_attr, 
                 char   *buf)
{
	struct temt_ls_state *ls = dev_get_drvdata(dev);
	return snprintf( buf, PAGE_SIZE, "%s\n", get_state_str(ls)) + 1;
}

static ssize_t 
store_state_attr( struct device *dev, struct device_attribute *dev_attr, 
                  const char *buf, size_t count)
{
	struct temt_ls_state *ls = dev_get_drvdata(dev);

	if (!buf || !count) 
		return -EINVAL; 

	if (count >= 3 && strncmp( buf, "off", 3) == 0 ) {
		ls->state = STATE_OFF;
		del_timer_sync(&ls->poll_timer);
		cancel_work_sync(&ls->poll_work);
	} 
	else if (count >= 9 && strncmp( buf,"suspended", 9) == 0) {
		ls->state = STATE_SUSPENDED;
		del_timer_sync(&ls->poll_timer);
		cancel_work_sync(&ls->poll_work);
	}
	else if (count >= 2 && strncmp( buf,"on", 2) == 0 ) {
		ls->state = STATE_ON;
		// Restart the timer
		if(!ls->suspended && ls->poll_interval > 0 ) { 
			mod_timer(&ls->poll_timer, 
			       jiffies + msecs_to_jiffies(ls->poll_interval));
		}
	} 
	
	return count;
}


static ssize_t 
show_poll_interval( struct device *dev,struct device_attribute *dev_attr, 
                    char *buf)
{
	struct temt_ls_state *state = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", state->poll_interval);
}

static ssize_t 
store_poll_interval(struct device *dev,struct device_attribute *dev_attr, 
                    const char *buf, size_t count)
{
	struct temt_ls_state *ls = dev_get_drvdata(dev);

	if (!buf || !count) 
		return -EINVAL; 

	ls->poll_interval =  simple_strtol(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR(result,         S_IRUGO,           show_result,         NULL);
static DEVICE_ATTR(average,        S_IRUGO | S_IWUGO, show_average,        store_average);
static DEVICE_ATTR(mode,           S_IRUGO | S_IWUSR, show_mode_attr,      store_mode_attr);
static DEVICE_ATTR(state,          S_IRUGO | S_IWUSR, show_state_attr,     store_state_attr);
static DEVICE_ATTR(poll_interval,  S_IRUGO | S_IWUSR, show_poll_interval,  store_poll_interval);

static void 
light_sensor_poll_result(struct work_struct *work) 
{
	int ret;
	struct temt_ls_state *ls;

	ls  = container_of(work, struct temt_ls_state, poll_work);
	ret = light_sensor_read_sample ( ls );
	if( ret >= 0 ) {
		input_report_abs(ls->idev, ABS_MISC, ret);
		input_sync(ls->idev);
	}
	if (ls->state == STATE_ON && !ls->suspended && ls->poll_interval > 0){
		mod_timer(&ls->poll_timer, 
		           jiffies + msecs_to_jiffies(ls->poll_interval));
	}
}

static int __devinit 
light_sensor_probe(struct platform_device *pdev)
{
	int ret;
	struct temt_ls_state* ls;
	struct input_dev   *idev;

	if( pdev->dev.platform_data == NULL )
		return (-ENODEV);

	ls = kzalloc(sizeof(struct temt_ls_state), GFP_KERNEL);
	if (!ls)
		return (-ENOMEM);

	ls->plat_data = pdev->dev.platform_data; 

	// Init lock 
	mutex_init(&ls->lock);

	/* Initially off */
	ls->state = STATE_OFF;

	/* Default value of poll interval is 1000ms */
	ls->poll_interval = 1000;

	// Init Workque interface  
	INIT_WORK(&ls->poll_work, light_sensor_poll_result ); 

	idev = input_allocate_device();
	if (idev == NULL) {
		ret = -ENOMEM;
		goto err1;
	}

	idev->name = pdev->name;
	idev->id.bustype = BUS_VIRTUAL;
	idev->id.vendor  = 0x0;
	idev->id.product = 0x0;
	idev->id.version = 0x100;
	idev->dev.parent = &pdev->dev;

	ret = input_register_device(idev);
	if (ret) {
		goto err2;
	}

	ls->idev = idev;

	set_bit(EV_ABS, ls->idev->evbit); 
	set_bit(ABS_MISC,  ls->idev->absbit); 

	// Set up a repeating timer
	setup_timer(&ls->poll_timer, 
	             light_sensor_poll_timeout, (unsigned long) ls);

	
	/* Create SYSFS attribute */
	if ((ret = device_create_file(&idev->dev, &dev_attr_result))) 
		goto err3;

	if ((ret = device_create_file(&idev->dev, &dev_attr_average))) 
		goto err4;

	if ((ret = device_create_file(&idev->dev, &dev_attr_poll_interval))) 
		goto err5;

	if ((ret = device_create_file(&idev->dev, &dev_attr_mode))) 
		goto err6;

	if ((ret = device_create_file(&idev->dev, &dev_attr_state)))
		goto err7;

	// attach 
	dev_set_drvdata(&idev->dev, ls);
	platform_set_drvdata(pdev,  ls);

	return 0;

err7:
	device_remove_file(&idev->dev, &dev_attr_mode);
err6:
	device_remove_file(&idev->dev, &dev_attr_poll_interval);
err5:
	device_remove_file(&idev->dev, &dev_attr_average);
err4:
	device_remove_file(&idev->dev, &dev_attr_result);
err3:
	input_unregister_device(idev);
err2:
	input_free_device(idev);
err1:
	kfree(ls);
	return ret;
}

static int __devexit 
light_sensor_remove(struct platform_device *pdev)
{
	struct temt_ls_state *ls = platform_get_drvdata(pdev);
	struct input_dev   *idev = ls->idev;

	del_timer(&ls->poll_timer);
	cancel_work_sync(&ls->poll_work);

	device_remove_file(&idev->dev, &dev_attr_result);
	device_remove_file(&idev->dev, &dev_attr_average);
	device_remove_file(&idev->dev, &dev_attr_poll_interval);
	device_remove_file(&idev->dev, &dev_attr_mode);
	device_remove_file(&idev->dev, &dev_attr_state);

	platform_set_drvdata (pdev, NULL);
	dev_set_drvdata(&idev->dev, NULL);

	input_free_device(idev);
	kfree(ls); 

	return 0;
}


#ifdef CONFIG_PM
static int 
light_sensor_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct temt_ls_state* ls = platform_get_drvdata(pdev);

	if (!ls->suspended++)
		return 0;

	del_timer(&ls->poll_timer);
	cancel_work_sync(&ls->poll_work);
	
	return 0;
}

static int 
light_sensor_resume(struct platform_device *pdev)
{
	struct temt_ls_state* ls = platform_get_drvdata(pdev);
	
	if (--ls->suspended)
		return 0;

	if (ls->state == STATE_ON && ls->poll_interval > 0) {
		mod_timer(&ls->poll_timer, 
		           jiffies + msecs_to_jiffies(ls->poll_interval));
	}
	return 0;
}
#else

#define  light_sensor_suspend  NULL
#define  light_sensor_resume   NULL

#endif

struct platform_driver temt6200_driver = {
	.probe		= light_sensor_probe,
	.remove		= __devexit_p(light_sensor_remove),
	.suspend	= light_sensor_suspend,
	.resume		= light_sensor_resume,
	.driver		= {
		.name	= TEMT6200_DRIVER,
	},
};

static int __init
light_senor_init(void)
{
	return platform_driver_register(&temt6200_driver);
}

static void __exit
light_sensor_exit(void)
{
	platform_driver_unregister(&temt6200_driver);
	return;
}

module_init(light_senor_init);
module_exit(light_sensor_exit);

MODULE_DESCRIPTION("TEMT6200 Light Sensor driver");
MODULE_LICENSE("GPL");
