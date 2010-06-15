/*
 * linux/drivers/misc/omap_hdq_battery.c
 *
 * Battery hdq driver for OMAP243x/343x
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * Author: Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <asm/arch/hdq.h>
#include <asm/arch/omap_bq2700_battery.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#define HIGH_BYTE(A)		((A)<< 8)
#define MASK_HB(A)		((A) & 0x03)
  
#define BQ_BATTERY_TEMP		0x06
#define BQ_BATTERY_TEMP_CK_OFFS	10926
#define BQ_BATTERY_TEMP_FACT_D	40
#define BQ_BATTERY_TEMP_FACT_M	10
#define BQ_BATTERY_VOLT		0x08
#define BQ_BATTERY_RSOC		0x0B /* Relative State-of-Charge */
#define BQ_BATTERY_CUR		0x14
#define BQ_BATTERY_AVR_CUR_FACT	357
#define BQ_BATTERY_RS		2000 /*Sensing resistor(ohm/100) */

static int omap_hdq_battery_probe(struct platform_device *dev);
static int omap_hdq_battery_remove(struct platform_device *dev);
#ifdef CONFIG_PM
static int omap_hdq_battery_suspend(struct platform_device *dev,
	pm_message_t state);
static int omap_hdq_battery_resume(struct platform_device *dev);
#endif

static struct platform_driver omap_bq2700_battery_driver = {
	.probe = omap_hdq_battery_probe,
	.remove = omap_hdq_battery_remove,
#ifdef CONFIG_PM
	.suspend = omap_hdq_battery_suspend,
	.resume = omap_hdq_battery_resume,
#endif
	.driver = {
		.name = "omap-bq2700-battery",
	},
};

#ifdef CONFIG_OMAP_BQ27000_BATTERY_EVENTS

/*
 * Battery event service rutines structure
 * 
 * This structure contains a set of function ponters
 * which points to the BQ battery event service 
 * routines. And a battery level mask register, which 
 * mask or unmask the software driven battery level
 * events.
 */
struct bqbattery_events bqbattery_event = {
	.battery_level = NULL,
	.battery_presence= NULL,	
	.battery_sw_level_event_cfg = 0,
	.battery_level_1 = 0,
	.battery_level_2 = 0,
	.battery_level_3 = 0,
	.battery_level_4 = 0,
	.temp_level = 0,
	.temp_presence = 0,
};

struct task_struct *bqbattery_level_thr= NULL, *bqbattery_presence_thr = NULL;
#endif

#ifdef CONFIG_OMAP_BATTERY_SYSFS
ssize_t show_bq_rsoc(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval, "BATTERY DRIVER: Main battery RSOC %d  \n ",
				bqbattery_rsoc());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

ssize_t show_bq_volt(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval,  "BATTERY DRIVER: Main battery voltage %d  \n ",
				bqbattery_voltage());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

ssize_t show_bq_curr(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval, "BATTERY DRIVER: Main battery current %d  \n ",
				bqbattery_current());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

ssize_t show_bq_temp(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval,"BATTERY DRIVER:Main battery temperature %d  \n ",
				bqbattery_temperature());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

static DEVICE_ATTR(bq_temperature, S_IRUGO, &show_bq_temp, NULL);
static DEVICE_ATTR(bq_voltage, S_IRUGO, &show_bq_volt, NULL);
static DEVICE_ATTR(bq_current, S_IRUGO, &show_bq_curr, NULL);
static DEVICE_ATTR(bq_power_level, S_IRUGO, &show_bq_rsoc, NULL);
#endif

static inline int read_bq_val (u8 reg, int *rt_value, int b_single);
/*
 * API function
 * 
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
int bqbattery_temperature(void)
{
	
	int ret, temp = 0;
		
	if((ret = read_bq_val (BQ_BATTERY_TEMP, &temp, 0)))
	{
		printk( KERN_ERR "BATTERY DRIVER:"
			"Error reading temperature from HDQ device");
		return ret;
	}
	
	return  ((temp * BQ_BATTERY_TEMP_FACT_M) - BQ_BATTERY_TEMP_CK_OFFS)
			/ BQ_BATTERY_TEMP_FACT_D;
	
}
EXPORT_SYMBOL(bqbattery_temperature);

/*
 * API function
 * 
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
int bqbattery_voltage(void)
{
	
	int ret, volt = 0;
		
	if((ret = read_bq_val (BQ_BATTERY_VOLT, &volt, 0)))
	{
		printk( KERN_ERR "BATTERY DRIVER:"
			"Error reading battery voltage from HDQ device");
		return ret;
	}
	
	return volt;
	
}
EXPORT_SYMBOL(bqbattery_voltage);

/*
 * API function
 * 
 * Return the battery average current
 * Or < 0 if something fails.
 */ 
int bqbattery_current(void)
{
	
	int ret, curr = 0;
		
	if((ret = read_bq_val (BQ_BATTERY_CUR, &curr, 0)))
	{
		printk( KERN_ERR "BATTERY DRIVER:"
			"Error reading battery charge current from HDQ device");
		return ret;
	}
		
	return curr;/*(curr * BQ_BATTERY_AVR_CUR_FACT)/ BQ_BATTERY_RS;*/
	
}
EXPORT_SYMBOL(bqbattery_current);

/*
 * API function:
 * 
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
int bqbattery_rsoc(void)
{
	
	int ret, rsoc = 0;
	
	if((ret = read_bq_val (BQ_BATTERY_RSOC, &rsoc, 1)))
	{
		printk( KERN_ERR "BATTERY DRIVER:"
			"Error reading battery Relative"
			"State-of-Charge from HDQ device");
		return ret;
	}
	
	return rsoc;
	
}
EXPORT_SYMBOL(bqbattery_rsoc);

/*
 * helper function
 * for read a 2 bytes register on BQjunior
 */
static inline int read_bq_val (u8 reg, int *rt_value, int b_single)
{
	
	u8 val;
	int ret;
	
	/* requesting HDQ module. */
	if ((ret = omap_hdq_get()))
	{				
		return ret;
	}
	
	/*  Send a break pulse to reset slave device (battery gas gauge) */  
	if ((ret= omap_hdq_break()))
	{	
		omap_hdq_reset();
		omap_hdq_put();
		return ret;
	}

	/*  Reading low byte. */ 
	if ((ret = omap_hdq_read(reg, &val)))
	{	
		omap_hdq_reset();	
		omap_hdq_put();
		return ret;
	}
	
	*rt_value = val;
	
	if (!b_single) 
	{
		/* Reading high byte. */
		if ((ret = omap_hdq_read(reg + 1, &val)))
		{	
			omap_hdq_reset();	
			omap_hdq_put();
			return ret;
		}
		
		*rt_value +=  HIGH_BYTE((int) val);
	}
		
	/*  releasing  HDQ module. */	
	if ((ret = omap_hdq_put()))
	{		
		return ret;
	}	
	return 0;	
}



static int omap_hdq_battery_probe(struct  platform_device *dev)
{

#ifdef CONFIG_OMAP_BATTERY_SYSFS
	if (device_create_file(&dev->dev, &dev_attr_bq_temperature) < 0)
		printk(KERN_ERR "Battery driver could not"
				"create bq temp sysfs entry\n");
	if (device_create_file(&dev->dev, &dev_attr_bq_voltage) < 0)
		printk(KERN_ERR "Battery driver could not"
				"create bq volt sysfs entry\n");
	if (device_create_file(&dev->dev, &dev_attr_bq_current) < 0)
		printk(KERN_ERR "Battery driver could not"
				"create bq curr sysfs entry\n");
	if (device_create_file(&dev->dev, &dev_attr_bq_power_level) < 0)
		printk(KERN_ERR "Battery driver could not"
				"create bq curr power level sysfs entry\n");
#endif

	return 0;
}

static int omap_hdq_battery_remove(struct  platform_device *dev)
{

#ifdef CONFIG_OMAP_BATTERY_SYSFS
	device_remove_file(&dev->dev, &dev_attr_bq_temperature);
	device_remove_file(&dev->dev, &dev_attr_bq_voltage);
	device_remove_file(&dev->dev, &dev_attr_bq_current);
	device_remove_file(&dev->dev, &dev_attr_bq_power_level);
#endif
	return 0;
}


#ifdef CONFIG_OMAP_BQ27000_BATTERY_EVENTS
static inline int bqbattery_presence_evt(void)
{
	int ret, event = 0;
	u8 val;
	
	/* In case of invalid service event function abandon */
	if (bqbattery_event.battery_presence == NULL)
		return -ENXIO;
		
	/* requesting HDQ module.*/ 
	if ((ret = omap_hdq_get()))
	{				
		return ret;
	}
	
	/*  Send a break pulse to reset slave device (battery gas gauge) */  
	if ((ret= omap_hdq_break()))
	{		
		omap_hdq_reset();
		omap_hdq_put();
	}
	
	/* trying to read any hdq register*/
	if ((ret = omap_hdq_read(0, &val)))
	{	
		/* Fail means battery disconnection*/
		omap_hdq_reset();
		event = EVT_BATTSTS_DISC;
	}
	else
	{	
		/* success battery presence */
		event = EVT_BATTSTS_CONN;
	}
	
	/*  releasing  HDQ module. */
	if ((ret = omap_hdq_put()))
	{
		return ret;
	}
	
	/*
	 * Run event notification if the state has changed or is
	 * an inconditional event triggering
	 */
	if ((bqbattery_event.temp_presence != event) ||
		(bqbattery_event.temp_presence == -1))
	{ 
		bqbattery_event.temp_presence = event;			
		bqbattery_event.battery_presence(event);
	}
		
	return 0;
}

static inline int bqbattery_level_evt(void)
{	
	int volt = 0, ret;
	
	/* In case of invalid service event function abandon */
	if (bqbattery_event.battery_level == NULL)
		return -ENXIO;
		
	/* Read battery voltage thru hdq interface */
	ret = read_bq_val (BQ_BATTERY_VOLT, &volt, 0);
	if(ret ==-ETIMEDOUT)
		return 0;
	
	if (ret)
		return ret;		
	/*
	 * in case voltage level hasn't change and is not needed a inconditional 
	 * event triggering
	 */ 
	if ((bqbattery_event.temp_level == volt) && 
		(bqbattery_event.temp_level != -1))
		return 0;
		
	bqbattery_event.temp_level = volt;
	
	if ((bqbattery_event.battery_sw_level_event_cfg & EVT_LVL_1_EN) &&
		(volt >= (bqbattery_event.battery_level_1 - VOLT_THR)) &&
		(volt <= (bqbattery_event.battery_level_1 + VOLT_THR)))
	{
		bqbattery_event.battery_level(EVT_LVL_1);
	}
		
	if ((bqbattery_event.battery_sw_level_event_cfg & EVT_LVL_2_EN) &&	
		(volt >= (bqbattery_event.battery_level_2 - VOLT_THR)) &&
		(volt <= (bqbattery_event.battery_level_2 + VOLT_THR)))
	{
		bqbattery_event.battery_level(EVT_LVL_2);
	}
	
	if ((bqbattery_event.battery_sw_level_event_cfg & EVT_LVL_3_EN) &&
		(volt >= (bqbattery_event.battery_level_3 - VOLT_THR)) &&
		(volt <= (bqbattery_event.battery_level_3 + VOLT_THR)))
	{
		bqbattery_event.battery_level(EVT_LVL_3);
	}
	
	if((bqbattery_event.battery_sw_level_event_cfg & EVT_LVL_4_EN) &&
		(volt >= (bqbattery_event.battery_level_4 - VOLT_THR)) &&
		(volt <= (bqbattery_event.battery_level_4 + VOLT_THR)))
	{
		bqbattery_event.battery_level(EVT_LVL_4);
	}
	
	return 0;
}

static int bqbattery_presence_thr_fcn (void *data)
{
	int ret;	
			
	while(1)
	{
		ret = bqbattery_presence_evt();	
		if(ret)
		{
			return 0;
		}
		else
		{
			msleep(BATT_PRES_DLY);
		}
	}	
	return 0;
}

static int bqbattery_level_thr_fcn (void *data)
{
	int ret;
	
	while(1)
	{
		ret = bqbattery_level_evt();			
		if(ret)
		{
			return 0;		
		}
		else
		{
			msleep(BATT_LVL_DLY);	
			
		}
	}
	return 0;	
}

/*
 * API function
 * 
 * Registers the event handler structure, all references pointing to NULL
 * will be ignored at the event triggering for either hardware enable or disable
 * event the corresponding API must be called.
 * 
 * Return 	0	on success
 *     		<0	on failure
 */
int bqbattery_event_register(struct bqbattery_events events)
{	
	bqbattery_event.battery_level = events.battery_level,
	bqbattery_event.battery_presence = events.battery_presence,	
	bqbattery_event.battery_sw_level_event_cfg = 
		(events.battery_sw_level_event_cfg &	(EVT_LVL_4_EN |
		EVT_LVL_3_EN | EVT_LVL_2_EN | EVT_LVL_1_EN));

	bqbattery_event.battery_level_1 = (events.battery_level_1 > 0)?
		events.battery_level_1: bqbattery_event.battery_level_1;
	bqbattery_event.battery_level_2 = (events.battery_level_2 > 0)?
		events.battery_level_2: bqbattery_event.battery_level_2;
	bqbattery_event.battery_level_3 = (events.battery_level_3 > 0)?
		events.battery_level_3: bqbattery_event.battery_level_3;
	bqbattery_event.battery_level_4 = (events.battery_level_4 > 0)?
		events.battery_level_4: bqbattery_event.battery_level_4;
		
	if(events.battery_presence != NULL)
	{		
		bqbattery_event.temp_presence = -1;
		bqbattery_presence_evt();
		bqbattery_sw_presence_en(EVT_ENABLE);		
	}
	else
	{
		bqbattery_sw_presence_en(EVT_DISABLE);
	}
	
	if(events.battery_level != NULL)
	{	
		bqbattery_event.temp_level = -1;
		bqbattery_level_evt();
		bqbattery_sw_level_en(events.battery_sw_level_event_cfg);
	}
	else
	{		
		bqbattery_sw_level_en(EVT_DISABLE);
	}	
		
	return 0;
}
EXPORT_SYMBOL(bqbattery_event_register);

/*
 * API function
 * 
 * Sets the voltage limit for any of the four battery voltage thresholds in
 * milivolts
 * 
 * 	 @level
 *  
 * 		EVT_LVL_4		voltage level 4
 *		EVT_LVL_3		voltage level 3
 *		EVT_LVL_2		voltage level 2
 *		EVT_LVL_1		voltage level 1
 * 
 * 	@volt
 * 
 * 		voltage value in milivolts from 0mV to 5000mV  
 * 	 
 */
int bqbattery_sw_level_set(int level, int volt)
{
	if ((volt< 0) || (volt > 5000))
		return -EINVAL;
		
	switch(level)
	{
		case EVT_LVL_4:
			bqbattery_event.battery_level_4 = volt;
			break;
			
		case EVT_LVL_3:
			bqbattery_event.battery_level_3 = volt;
			break;
			
		case EVT_LVL_2:
			bqbattery_event.battery_level_2 = volt;
			break;
			
		case EVT_LVL_1:
			bqbattery_event.battery_level_1 = volt;
			break;
			
		default:
			return -EINVAL;			
	}
	
	return 0;	 
}
EXPORT_SYMBOL(bqbattery_sw_level_set);

/*
 * API function
 * 
 * Gets the voltage setted limit for any of the four battery voltage 
 * thresholds in  milivolts
 * 
 * 	 @level
 *  
 * 		EVT_LVL_4		voltage level 4
 *		EVT_LVL_3		voltage level 3
 *		EVT_LVL_2		voltage level 2
 *		EVT_LVL_1		voltage level 1
 */
int bqbattery_sw_level_get(int level)
{
	switch(level)
	{
		case EVT_LVL_4:
			return bqbattery_event.battery_level_4;
			break;
			
		case EVT_LVL_3:
			return bqbattery_event.battery_level_3;
			break;
			
		case EVT_LVL_2:
			return bqbattery_event.battery_level_2;
			break;
			
		case EVT_LVL_1:
			return bqbattery_event.battery_level_1;
			break;
			
		default:
			return -EINVAL;			
	}	
}
EXPORT_SYMBOL(bqbattery_sw_level_get);

/*
 * API function
 * 
 * Disable/Enable battery level event notifications.
 * 
 * Note:
 * This Disable/Enable individual battery level notifications by software.
 * 
 * 	@enable_lvl
 * 
 * 		EVT_LVL_4_EN		Enable voltage level 4
 *		EVT_LVL_3_EN		Enable voltage level 3
 *		EVT_LVL_2_EN		Enable voltage level 2
 *		EVT_LVL_1_EN		Enable voltage level 1
 * 		EVT_DISABLE		Disable all
 * 
 * note:
 * The omission of the any of the battery level voltage enablers with
 * will disable the respective event. Passing a 0 will disable the hardware
 * interruption. 
 *   
 * Return 	0	on success
 *     		<0	on failure 
 */
int bqbattery_sw_level_en(int enable_lvl)
{
	bqbattery_event.battery_sw_level_event_cfg = (enable_lvl & 
	(EVT_LVL_4_EN | EVT_LVL_3_EN | EVT_LVL_2_EN | EVT_LVL_1_EN));
	
	if (bqbattery_event.battery_sw_level_event_cfg)
	{
		if(!(task_curr(bqbattery_level_thr)))
		{
			bqbattery_level_thr = kthread_run(bqbattery_level_thr_fcn,
				(void *) &bqbattery_event, "bqbattery_level_thr");
		}
	}
	else
	{
		if ((task_curr(bqbattery_level_thr)))
			kthread_stop(bqbattery_level_thr);		
	}
	
	return 0;
		
}
EXPORT_SYMBOL(bqbattery_sw_level_en);

/*
 * API function
 * 
 * Queries software battery level event notifications status.
 * 
 * Return:
 * 
 * 		EVT_LVL_4_EN		Enable voltage level 4
 *		EVT_LVL_3_EN		Enable voltage level 3
 *		EVT_LVL_2_EN		Enable voltage level 2
 *		EVT_LVL_1_EN		Enable voltage level 1
 * 		EVT_DISABLE		Disable all
 *
 *		<0 on failure 
 */
int bqbattery_sw_level_qu(void)
{	
	if((task_curr(bqbattery_level_thr)))
	{		
		return bqbattery_event.battery_sw_level_event_cfg & 
			(EVT_LVL_4_EN | EVT_LVL_3_EN | EVT_LVL_2_EN | 
			EVT_LVL_1_EN);
	}
		
	return 0;
}
EXPORT_SYMBOL(bqbattery_sw_level_qu);

/*
 * API function
 * 
 * Disable/Enable software battery presence event notifications.
 * 
 * 	@enable
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 
 * 	Return 	0	on success
 *     		<0	on failure
 */
int bqbattery_sw_presence_en(int enable)
{	
	if (enable)
	{
		if (!(task_curr(bqbattery_presence_thr)))
		{
			bqbattery_presence_thr = kthread_run(
				bqbattery_presence_thr_fcn,
				(void *) &bqbattery_event,
				"bqbattery_presence_thr");
		} 
	}
	else
	{
	 	if ((task_curr(bqbattery_presence_thr)))
			kthread_stop(bqbattery_presence_thr);
	}
		
	return 0;
		
}
EXPORT_SYMBOL(bqbattery_sw_presence_en);

/*
 * API function
 * 
 * Querries for battery voltage presence event notifications status.
 * 
 * Return 
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *
 * 		<0	On failure
 */
int bqbattery_sw_presence_qu(void)
{
	/* return timer_pending(&bqbattery_presence_tmr);*/
	return task_curr(bqbattery_presence_thr);
}
EXPORT_SYMBOL(bqbattery_sw_presence_qu);
#endif	

#ifdef CONFIG_PM
static int omap_hdq_battery_suspend(struct platform_device *dev,
	pm_message_t state)
{
#ifdef CONFIG_OMAP_BQ27000_BATTERY_EVENTS

	bqbattery_sw_presence_en(EVT_DISABLE);
	bqbattery_sw_level_en(EVT_DISABLE);

#endif  /* CONFIG_OMAP_BQ27000_BATTERY_EVENTS */
	return 0;
}

static int omap_hdq_battery_resume(struct platform_device *dev)
{
#ifdef CONFIG_OMAP_BQ27000_BATTERY_EVENTS

	if(bqbattery_event.battery_presence != NULL)
	{
		bqbattery_event.temp_presence = -1;
		bqbattery_presence_evt();
		bqbattery_sw_presence_en(EVT_ENABLE);
	}

	if(bqbattery_event.battery_level != NULL)
	{
		bqbattery_event.temp_level = -1;
		bqbattery_level_evt();
		bqbattery_sw_level_en(
			bqbattery_event.battery_sw_level_event_cfg);
	}

#endif  /* CONFIG_OMAP_BQ27000_BATTERY_EVENTS */
	return 0;
}
#endif

static int __init omap_hdq_battery_init(void)
{
	int ret;

	if((ret = platform_driver_register(&omap_bq2700_battery_driver)))
	{
		printk(KERN_ERR  "HDQ BATTERY DRIVER:"
			"driver register Failed ...!!!!\n");
	}

	return 0;

}

static void __exit omap_hdq_battery_exit(void)
{	
	platform_driver_unregister(&omap_bq2700_battery_driver);
}

module_init(omap_hdq_battery_init);
module_exit(omap_hdq_battery_exit);

MODULE_LICENSE("GPL");
