/*
 * linux/drivers/misc/omap_bci_battery.c
 * 
 * BCI battery driver for Linux
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Author: Texas Instruments, Inc. 
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
#include <asm/arch/twl4030.h>
#include <asm/arch/omap_bci_battery.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#define REG_BCICTL1 		0x023
#define REG_BCICTL2 		0x024
#define CGAIN			0x020
#define ITHEN			0x010
#define ITHSENS			0x007
#define REG_BCIMFTH1		0x016
#define REG_BCIMFTH2		0x017

/* Boot BCI flag bits */
#define BCIAUTOWEN		0x020
#define CONFIG_DONE		0x010
#define BCIAUTOUSB		0x002
#define BCIAUTOAC		0x001
#define BCIMSTAT_MASK		0x03F

/* Boot BCI register */
#define REG_BOOT_BCI		0x007
#define REG_CTRL1		0x00
#define MADC_ON			0x01
#define REG_SW1SELECT_MSB	0x07
#define SW1_CH9_SEL		0x02
#define REG_CTRL_SW1		0x012
#define SW1_TRIGGER		0x020
#define EOC_SW1			0x002
#define BUSY			0x001
#define REG_GPCH9		0x049 
#define REG_STS_HW_CONDITIONS	0x0F
#define STS_VBUS		0x080 
#define STS_CHG			0x02
#define REG_BCIMSTATEC		0x02
#define REG_BCIMFSTS4		0x010
#define REG_BCIMFSTS2		0x00E
#define REG_BCIMFSTS3 		0x00F
#define REG_BCIMFSTS1		0x001
#define USBFASTMCHG		0x004
#define BATSTSPCHG		0x004
#define BATSTSMCHG		0x040
#define VBATOV4			0x020
#define VBATOV3			0x010
#define VBATOV2			0x008
#define VBATOV1			0x004
#define MADC_LSB_MASK		0xC0
#define REG_BB_CFG		0x012
#define BBCHEN			0x010

/* Settings event enable/disable */
#define EVT_ENABLE		0x001
#define EVT_DISABLE		0x000

#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS
/* Power supply charge interrupt */
#define REG_PWR_ISR1		0x00
#define REG_PWR_IMR1		0x01
#define REG_PWR_EDR1		0x05
#define REG_PWR_SIH_CTRL	0x007

#define USB_PRES		0x004
#define CHG_PRES		0x002

#define USB_PRES_RISING		0x020
#define USB_PRES_FALLING	0x010
#define CHG_PRES_RISING		0x008
#define CHG_PRES_FALLING	0x004

#define COR			0x004

/* interrupt status registers */
#define REG_BCIISR1A 		0x0
#define REG_BCIISR2A 		0x01

/* Interrupt flags bits BCIISR1 */
#define BATSTS_ISR1		0x080
#define VBATLVL_ISR1		0x001

/* Interrupt mask registers for int1*/
#define REG_BCIIMR1A		0x002
#define REG_BCIIMR2A		0x003

 /* Interrupt masks for BCIIMR1 */
#define BATSTS_IMR1		0x080
#define VBATLVL_IMR1		0x001

/* Interrupt edge detection register */
#define REG_BCIEDR1		0x00A
#define REG_BCIEDR2		0x00B
#define REG_BCIEDR3		0x00C

/* BCIEDR2 */
#define	BATSTS_EDRRISIN		0x080
#define BATSTS_EDRFALLING	0x040

/* BCIEDR3 */
#define	VBATLVL_EDRRISIN	0x02
#endif

struct work_struct twl_event;
static void twl4030_bat_event_handler(struct work_struct *work);

static int omap_bci_battery_probe(struct platform_device *dev);
static int omap_bci_battery_remove(struct platform_device *dev);
#ifdef CONFIG_PM
static int omap_bci_battery_suspend(struct platform_device *dev,
	pm_message_t state);
static int omap_bci_battery_resume(struct platform_device *dev);
#endif

static struct platform_driver omap_bci_battery_driver = {
	.probe = omap_bci_battery_probe,
	.remove = omap_bci_battery_remove,
#ifdef CONFIG_PM
	.suspend = omap_bci_battery_suspend,
	.resume = omap_bci_battery_resume,
#endif
	.driver = {
		.name = "omap-bci-battery",
	},
};

#ifdef CONFIG_OMAP_BATTERY_SYSFS
ssize_t show_t2_bkbatt_curr(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval, "BATTERY DRIVER: Main battery RSOC %d  \n ",
			twl4030backupbatt_voltage());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

ssize_t show_t2_volt(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval,  "BATTERY DRIVER: Main battery voltage %d  \n ",
			twl4030battery_voltage());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

ssize_t show_t2_curr(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval, "BATTERY DRIVER: Main battery current %d  \n ",
			twl4030battery_current());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

ssize_t show_t2_temp(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	char *sval = buf;
	
	sval += sprintf (sval,"BATTERY DRIVER:Main battery temperature %d  \n ",
			twl4030battery_temperature());
	
	sval+= 1;
	*sval=0;
	
	return sval - buf + 1;
}

static DEVICE_ATTR(t2_temperature, S_IRUGO, &show_t2_temp, NULL);
static DEVICE_ATTR(t2_voltage, S_IRUGO, &show_t2_volt, NULL);
static DEVICE_ATTR(t2_current, S_IRUGO, &show_t2_curr, NULL);
static DEVICE_ATTR(t2_backup_voltage, S_IRUGO, &show_t2_bkbatt_curr, 
	NULL);

#endif 	/* CONFIG_OMAP_BATTERY_SYSFS */

static int twl4030madc_sw1_trigger (void);
static int read_bci_val (u8 reg_1);
static int clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg);

/*
 * Twl4030 battery temperature lookup table
 */
const int twl4030battery_temp_tbl [] =
{
/* 0 C*/
27100,
26000, 24900, 23900, 22900, 22000, 21100, 20300, 19400, 18700, 17900,
17200, 16500, 15900, 15300, 14700, 14100, 13600, 13100, 12600, 12100,
11600, 11200, 10800, 10400, 10000, 9630,   9280,   8950,   8620,   8310,
8020,   7730,   7460,   7200,   6950,   6710,   6470,   6250,   6040,   5830,
5640,   5450,   5260,   5090,   4920,   4760,   4600,   4450,   4310,   4170,
4040,   3910,   3790,   3670,   3550
};

#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS
/*
 * Timer information structure
 * 
 * This is structure stores the timer information
 * to monitor the twl4030 BCI module status. 
 * 
 */
struct timer_list twl4030bci_status_tmr = {{NULL}};

/*
 * Battery event service rutines structure
 * 
 * This structure contains a set of function ponters
 * which points to the twl4030 battery event service 
 * routines. And a battery level mask register, which 
 * mask or unmask the software driven battery level
 * events.
 * 
 */
struct twl4030battery_events twl4030battery_event = {
	.battery_level = NULL,
	.battery_presence= NULL,
	.charger_presence= NULL,
	.bci_status= NULL,
	.battery_sw_level_event_cfg = 0,	
	.temp_std = 0,
	};	
/*
 * helper function
 */
static int twl4030bci_status_evt(void)
{	
	int std;
	
	if (twl4030battery_event.bci_status == NULL)
		return -ENXIO;
	
	/* Reading the BIC status */
	std = twl4030bci_status();
	if (std < 0) 
		return std;
		
	/* Check if the BCI status has changed since the last time */
	if ((twl4030battery_event.temp_std != std) ||
		 (twl4030battery_event.temp_std == -1))
	{
		/* If so actualizes the previous status variable */
		twl4030battery_event.temp_std = std;
		
		/* calling the event service function */
		twl4030battery_event.bci_status(std);	 					
	}
	
	return 0;
}
/*
 * Timer Service Funtion
 * 
 * This function checks the BCI's status and is something 
 * has changed this call the event service function on the event
 * notification structure.
 * 
 */
static void twl4030bci_status_tmr_fcn (unsigned long data)
{
	schedule_work(&twl_event);
}

/*
 * Helper function
 * 
 * This fucntion drives charger power supply events
 */
static int twl4030charger_presence_evt (void)
{
	int ret, event = 0;
	u8 chg_sts, set = 0, clear = 0;

	/*
	 * if there is no event sevice routine there is no need to do 
	 * anything.
	 */
	if (twl4030battery_event.charger_presence == NULL)
		return -ENXIO;
	
	/* checking charger power supply status */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER,&chg_sts, 
		REG_STS_HW_CONDITIONS);
	if (ret)
		return IRQ_NONE;
				
	/* If the AC charger have been connected*/
	if (chg_sts & STS_CHG)
	{
		/* configuring falling edge detection for CHG_PRES */
		set = CHG_PRES_FALLING;
		clear = CHG_PRES_RISING;
		event = EVT_PWSPLY_AC_CONN;
	}
	/* If the AC charger have been disconnected*/
	else
	{
		/* configuring rising edge detection for CHG_PRES */
		set = CHG_PRES_RISING;
		clear = CHG_PRES_FALLING;
		event = EVT_PWSPLY_AC_DISC;
	}		
	 
	/* If the USB charger have been connected*/
	if (chg_sts & STS_VBUS)
	{
		/* configuring falling edge detection for USB_PRES */
		set |= USB_PRES_FALLING;
		clear |= USB_PRES_RISING;
		event |= EVT_PWSPLY_USB_CONN;
	}
	/* If the USB charger have been disconnected*/
	else
	{	/*
		 * configuring interrupt edge detection
		 * for USB_PRES and CHG_PRES.
	 	 */
		set |= USB_PRES_RISING;
		clear |= USB_PRES_FALLING;
		event |= EVT_PWSPLY_USB_DISC;
	}	
	
	/* Update the interrupt edge detection register */
	ret = clear_n_set(TWL4030_MODULE_INT, clear, set, REG_PWR_EDR1);
	if (ret)
		return 0;
			
	/* Calling  to the charger presence event service routine */
	twl4030battery_event.charger_presence(event);
	
	return 0;
}

/*
 * Interrupt service routine
 * 
 * Attends to TWL 4030 power module interruptions events, 
 * specifically USB_PRES (USB charger presence) CHG_PRES (AC charger presence) events
 * 
 */
static irqreturn_t  twl4030charger_interrupt(int irq, void *dev_id)
{
	u8 int_src, clear = 0;
	int ret;

	/* Reading interruption source*/
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &int_src, REG_PWR_ISR1);
	if (ret)
		return IRQ_NONE;
				
	/* Addition to enable shared interrupt */ 	 	
	if(!(int_src & (USB_PRES | CHG_PRES)))
		return IRQ_NONE;
	
	/* Cleaning interruption flags */
	clear = (int_src & USB_PRES)? (clear | USB_PRES): clear; 
	clear = (int_src & CHG_PRES)? (clear | CHG_PRES): clear;
		
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_INT, 
	clear, REG_PWR_ISR1);
	if (ret)
		return IRQ_NONE;

	/* fuction to handle charger presence events */
	ret = twl4030charger_presence_evt();
	if(ret == -ENXIO)
		twl4030charger_hw_presence_en(EVT_DISABLE);
		
	if(ret)
		return IRQ_NONE;
		
	return IRQ_HANDLED;
}

/*
 * Helper Function
 * 
 * This function handles the twl4030 battery presence interrupt
 * 
 */

static int twl4030battery_presence_evt(void)
{
	int ret;
	u8 batstsmchg, batstspchg;
	
	/* In case of invalid service event function abandon */
	if (twl4030battery_event.battery_presence == NULL)
		return -ENXIO;		
	
	/* checking for battery presence in main charge*/		
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, 
			&batstsmchg, REG_BCIMFSTS3)))
		return ret;
			
	/* checking for battery presence in precharge*/		
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_PRECHARGE,
			&batstspchg, REG_BCIMFSTS1)))
		return ret;
	
	/* In case of the battery insertion event */
	if ((batstspchg & BATSTSPCHG) || (batstsmchg & BATSTSMCHG))
	{			
		/* calling the battery presence event service function */
		twl4030battery_event.battery_presence(EVT_BATTSTS_CONN);
		
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, BATSTS_EDRRISIN,
			BATSTS_EDRFALLING, REG_BCIEDR2);
		if (ret)
			return ret;
	}
	
	/* In case of the battery removal event */
	else
	{	
		/* calling the battery presence event service function */
		twl4030battery_event.battery_presence(EVT_BATTSTS_DISC);
		
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, BATSTS_EDRFALLING,
			BATSTS_EDRRISIN, REG_BCIEDR2);
		if (ret)
			return ret;
	}	
	
	return 0;
}

/*
 * Helper Function
 * 
 * This function handles the twl4030 battery voltage level
 * 
 */

static int twl4030battery_level_evt(void)
{
	int ret;
	u8 mfst;
	
	/* In case of invalid service event function abandon */
	if(twl4030battery_event.battery_level == NULL)
		return -ENXIO;
			
	/* checking for threshold event */		
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE,
			&mfst, REG_BCIMFSTS2)))
		return ret;
		
	if((mfst & VBATOV4)  &&
		(twl4030battery_event.battery_sw_level_event_cfg &
		EVT_LVL_4_EN))
	{		
		twl4030battery_event.battery_level(EVT_LVL_4);
	}
	
	else if((mfst & VBATOV3) &&
		(twl4030battery_event.battery_sw_level_event_cfg &
		EVT_LVL_3_EN))
	{			
		twl4030battery_event.battery_level(EVT_LVL_3);
	}
	
	else if((mfst & VBATOV2) &&
		(twl4030battery_event.battery_sw_level_event_cfg &
		EVT_LVL_2_EN))
	{			
		twl4030battery_event.battery_level(EVT_LVL_2);
	}
			
	else if((mfst & VBATOV1)  &&
		(twl4030battery_event.battery_sw_level_event_cfg &
		EVT_LVL_1_EN)) 
	{	
		twl4030battery_event.battery_level(EVT_LVL_1);
	}
		
	return 0;
}

/*
 * Interrupt service routine
 * 
 * Attends to BCI interruptions events, 
 * specifically BATSTS (battery connection and removal)
 * VBATOV (main battery voltage threshold) events
 * 
 */
static irqreturn_t twl4030battery_interrupt(int irq, void *dev_id)
{
	int ret;
	u8 isr1a_val, isr2a_val, clear_2a, clear_1a;

	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &isr1a_val, 
				REG_BCIISR1A)))	
		return IRQ_NONE;
	
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &isr2a_val, 
				REG_BCIISR2A)))
		return IRQ_NONE;
	
	clear_2a = (isr2a_val & VBATLVL_ISR1)? (VBATLVL_ISR1): 0;
	clear_1a = (isr1a_val & BATSTS_ISR1)? (BATSTS_ISR1): 0;
	
	/* cleaning BCI interrupt status flags */
	if ((ret = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 
			clear_1a , REG_BCIISR1A)))
	return IRQ_NONE;
	
	if ((ret = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 
			clear_2a , REG_BCIISR2A)))
	return IRQ_NONE;
		
			
	/* battery connetion or removal event */
	if (isr1a_val & BATSTS_ISR1)
	{
		ret = twl4030battery_presence_evt();
		if (ret == -ENXIO)
		{
			twl4030battery_hw_presence_en(EVT_DISABLE);
			return IRQ_HANDLED;
		}
		if (ret)
			return IRQ_NONE;		
	}
		
	/* battery voltage threshold event*/
	else if (isr2a_val & VBATLVL_ISR1)
	{
		ret = twl4030battery_level_evt();
		if (ret == -ENXIO)
		{
			twl4030battery_hw_level_en(EVT_DISABLE);
			return IRQ_HANDLED;
		}	
		if (ret)
			return IRQ_NONE;
	}
	/* Only for debuging purpouses this branch never be taken */
	else
		return IRQ_NONE;
			
	return IRQ_HANDLED;		
}

/*
 * API function
 * 
 * Registers the event handler structure
 * 
 * Return 	0	on success
 *     		<0	on failure
 */
int twl4030battery_event_register(struct twl4030battery_events events)
{
	twl4030battery_event.battery_level = events.battery_level;
	twl4030battery_event.battery_presence = events.battery_presence;
	twl4030battery_event.charger_presence = events.charger_presence;
	twl4030battery_event.bci_status = events.bci_status;
	
	twl4030battery_event.battery_sw_level_event_cfg = (
		events.battery_sw_level_event_cfg & (EVT_LVL_4_EN |
		EVT_LVL_3_EN | EVT_LVL_2_EN | EVT_LVL_1_EN)) ;
	
	if(events.battery_level == NULL)
	{
		twl4030battery_hw_level_en(EVT_DISABLE);
	}
	else
	{
		twl4030battery_hw_level_en(EVT_ENABLE);
		twl4030battery_level_evt();	
	}
	
	if(events.battery_presence == NULL)
	{
		twl4030battery_hw_presence_en(EVT_DISABLE);
	}
	else
	{
		twl4030battery_hw_presence_en(EVT_ENABLE);
		twl4030battery_presence_evt();
	}
	
	if(events.charger_presence == NULL)
	{
		twl4030charger_hw_presence_en(EVT_DISABLE);
	}
	else
	{
		twl4030charger_hw_presence_en(EVT_ENABLE);
		twl4030charger_presence_evt();
	}
	
	if(events.bci_status == NULL)
	{
		twl4030bci_sw_status_en(EVT_DISABLE);
		flush_scheduled_work();
	}
	else
	{
		twl4030bci_status_tmr.expires = jiffies + CHG_STS_DLY;
		add_timer(&twl4030bci_status_tmr);
		twl4030battery_event.temp_std = -1;
		twl4030bci_status_evt();
	}
	
	return 0;
}
EXPORT_SYMBOL(twl4030battery_event_register);

/*
 * API function
 * 
 * 	Sets the voltage limit for any of the four battery voltage thresholds in
 * milivolts
 * 
 * 	 @level
 *  
 * 		EVT_LVL_4_EN		voltage level 4
 *		EVT_LVL_3_EN		voltage level 3
 *		EVT_LVL_2_EN		voltage level 2
 *		EVT_LVL_1_EN		voltage level 1
 * 
 * 	@volt
 * 
 * 		for EVT_LVL_1_EN: 
 * 			LSB =23.4 mV
 * 			Default [0x0B] =2.894V
 * 			Minimum [0x00] =2.636V
 * 			Maximum [0x0f] =2.988V
 * 		
 * 		for EVT_LVL_2_EN: 
 * 			LSB =46.8 mV
 * 			Default [0x09] =3.451V
 * 			Minimum [0x00] =3.029V
 * 			Maximum [0x0f] =3.732V
 * 
 * 		for EVT_LVL_3_EN: 
 * 			LSB =23.4 mV
 * 			Default [0x06] =3.902V
 * 			Minimum [0x00] =3.761V
 * 			Maximum [0x0f] =4.113V
 * 
 * 		for EVT_LVL_4_EN: 
 * 			LSB =23.4 mV
 * 			Default [0x08] =3.949V
 * 			Minimum [0x00] =3.761V
 * 			Maximum [0x0f] =4.113V
 * 	 
 */
int twl4030battery_sw_level_set(int level, int volt)
{
	u8 val, reg;
	int ret;
	
	switch(level)
	{
		case EVT_LVL_4_EN:
			val = ((u8)(volt & 0x0F)) << 4;
			reg = REG_BCIMFTH2;
			break;
			
		case EVT_LVL_3_EN:
			val = (u8)(volt & 0x0F);
			reg = REG_BCIMFTH2;
			break;
			
		case EVT_LVL_2_EN:
			val = ((u8)(volt & 0x0F)) << 4;
			reg = REG_BCIMFTH1;
			break;
			
		case EVT_LVL_1_EN:
			val = (u8)(volt & 0x0F);
			reg = REG_BCIMFTH1;
			break;
			
		default:
		return -EINVAL;
	}
	
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_MAIN_CHARGE, val, reg);
	if(ret)
		return ret;
		
	return 0;
}
EXPORT_SYMBOL(twl4030battery_sw_level_set);

/*
 * API function
 * 
 * Gets the voltage setted limit for any of the four battery voltage
 * thresholds in milivolts
 * 
 * 	 @level
 *  
 * 		EVT_LVL_4_EN		voltage level 4
 *		EVT_LVL_3_EN		voltage level 3
 *		EVT_LVL_2_EN		voltage level 2
 *		EVT_LVL_1_EN		voltage level 1
 */
int twl4030battery_sw_level_get(int level)
{
	u8 val, reg;
	int ret;	
	
	if((level == EVT_LVL_4_EN) || (level == EVT_LVL_3_EN))
	{
		reg = REG_BCIMFTH2;
	} 
	else if((level == EVT_LVL_2_EN) || (level == EVT_LVL_1_EN))
	{
		reg = REG_BCIMFTH1;
	}
	else
		return -EINVAL;
		
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val, reg);
	if(ret)
		return ret;
	
	if((level == EVT_LVL_4_EN) || (level == EVT_LVL_2_EN))
	{
		val = (val & 0xF0)>> 4;
	} 
	else if((level == EVT_LVL_3_EN) || (level == EVT_LVL_1_EN))
	{
		val = (val & 0x0F);
	}
		
	return (int)val;
}
EXPORT_SYMBOL(twl4030battery_sw_level_get);

/*
 * API function
 * 
 * Disable/Enable software battery level event notifications.
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
 *   
 *	Return 	0	on success
 *     		<0	on failure 
 */
int twl4030battery_sw_level_en (int enable_lvl)
{
	
	twl4030battery_event.battery_sw_level_event_cfg |= (enable_lvl & 
		(EVT_LVL_4_EN | EVT_LVL_3_EN | EVT_LVL_2_EN | EVT_LVL_1_EN)) ;
	
	return 0;
}
EXPORT_SYMBOL(twl4030battery_sw_level_en);

/*
 * API function
 * 
 * Queries software battery level event notifications status.
 *
 * 	Return:
 * 
 * 		EVT_LVL_4_EN		Enable voltage level 4
 *		EVT_LVL_3_EN		Enable voltage level 3
 *		EVT_LVL_2_EN		Enable voltage level 2
 *		EVT_LVL_1_EN		Enable voltage level 1
 *     	<0				on failure 
 */
int twl4030battery_sw_level_qu(void)
{	
	return twl4030battery_event.battery_sw_level_event_cfg & 
		(EVT_LVL_4_EN | EVT_LVL_3_EN | EVT_LVL_2_EN | EVT_LVL_1_EN);
}
EXPORT_SYMBOL(twl4030battery_sw_level_qu);

/*
 * API function
 * 
 * Disable/Enable hardware battery level event notifications.
 * 
 * Note:
 * This Disable/Enable all battery level notifications. 
 * 
 * 	@enable
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 
 * 	Return 	0	on success
 *     		<0	on failure
 */
int twl4030battery_hw_level_en (int enable)
{
	int ret;
	
	if(enable)
	{
		/* unmask VBATOV interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, VBATLVL_IMR1,
			0, REG_BCIIMR2A);
		if (ret)
			return ret;
			
		/* configuring interrupt edge detection for VBATOv */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			VBATLVL_EDRRISIN, REG_BCIEDR3);
		if(ret)
			return ret;
	}
	else
	{
		/* mask VBATOV interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			VBATLVL_IMR1, REG_BCIIMR2A);
		if (ret)
			return ret;
	}
			
	return 0;	
}
EXPORT_SYMBOL(twl4030battery_hw_level_en);

/*
 * API function
 * 
 * Queries hardware battery level event notifications status.
 * 
 * 	Return
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *     	<0	On failure
 */
int twl4030battery_hw_level_qu (void)
{
	int ret;
	u8 val;
	
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &val,
		REG_BCIIMR2A);
	
	return (ret)? ret: (val & VBATLVL_IMR1); 
}
EXPORT_SYMBOL(twl4030battery_hw_level_qu);

/*
 * API function
 * 
 * Disable/Enable hardware battery presence event notifications.
 * 
 * 	@enable
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 
 *	Return 	0	on success
 *     		<0	on failure
 */
int twl4030battery_hw_presence_en(int enable)
{	
	int ret;
	
	if(enable)
	{
		/* unmask BATSTS interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, BATSTS_IMR1,
			0, REG_BCIIMR1A);
		if (ret)
			return ret;
		
		/* configuring interrupt edge for BATSTS */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			BATSTS_EDRRISIN | BATSTS_EDRFALLING, REG_BCIEDR2);
		if (ret)
			return ret;
	}
	else
	{
		/* mask BATSTS interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			BATSTS_IMR1, REG_BCIIMR1A);
		if (ret)
			return ret;
	}
			
	return 0;	
}
EXPORT_SYMBOL(twl4030battery_hw_presence_en);

/*
 * API function
 * 
 * Queries hardware battery presence event notifications status.
 * 
 * 	Return
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *     	<0	On failure
 */
int twl4030battery_hw_presence_qu(void)
{
	int ret;
	u8 val;
	
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &val,
		REG_BCIIMR1A);
	
	return (ret)? ret: (val & BATSTS_IMR1);
}
EXPORT_SYMBOL(twl4030battery_hw_presence_qu);

/*
 * API function
 * 
 * Disable/Enable hardware charger presence event notifications.
 * 
 * 	@enable
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 
 * 	Return 	0	on success
 *     		<0	on failure
 */
int twl4030charger_hw_presence_en(int enable)
{	
	int ret;
	
	if(enable)
	{
		/* unmask USB_PRES and CHG_PRES interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INT, (USB_PRES | CHG_PRES), 0, 
			REG_PWR_IMR1);
		if (ret)
			return ret;
		/*	
		 * configuring interrupt edge detection
		 * for USB_PRES and CHG_PRES
		 */
		ret = clear_n_set(TWL4030_MODULE_INT, 0, (USB_PRES_RISING | 
			USB_PRES_FALLING | CHG_PRES_RISING | CHG_PRES_FALLING), 
			REG_PWR_EDR1);
		if (ret)
			return 0;
	}
	else
	{
		/* mask USB_PRES and CHG_PRES interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INT, 0, (USB_PRES | CHG_PRES),
			REG_PWR_IMR1);
		if (ret)
			return ret;
	}
	
	return 0;
}
EXPORT_SYMBOL(twl4030charger_hw_presence_en);

/*
 * API function
 * 
 * Queries hardware charger presence event notification status.
 * 
 * 	Return
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *     	<0	On failure
 */
int twl4030charger_hw_presence_qu(void)
{
	int ret;
	u8 val;
	
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &val,
		REG_PWR_IMR1);
	
	return (ret)? ret: (val & (USB_PRES | CHG_PRES));
}
EXPORT_SYMBOL(twl4030charger_hw_presence_qu);

/*
 * API function
 * 
 * Disable/Enable software battery charge status event notifications.
 * 
 * 	@enable
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 
 * 	Return 	0	on success
 *     		<0	on failure
 */
int twl4030bci_sw_status_en(int enable)
{

	if (enable)
	{
		if (!(timer_pending(&twl4030bci_status_tmr)))
		{
			mod_timer(&twl4030bci_status_tmr, jiffies +
				msecs_to_jiffies(CHG_STS_DLY));
			twl4030battery_event.temp_std = -1;
		}
	}
	else
	{
		if (timer_pending(&twl4030bci_status_tmr))
		{
			del_timer(&twl4030bci_status_tmr);
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030bci_sw_status_en);

/*
 * API function
 * 
 * Querries for battery charge status event notifications status.
 * 
 * 	Return 
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *     	<0	On failure
 */
int twl4030bci_sw_status_qu(void)
{
	return timer_pending(&twl4030bci_status_tmr);
}
EXPORT_SYMBOL(twl4030bci_sw_status_qu);

#endif  /* CONFIG_OMAP_TWL4030_BATTERY_EVENTS */

/*
 * API function
 * 
 * Disable/Enable AC Charge funtionality.
 * 
 * 	@enable
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 
 * 	Return 	0	on success
 *     		<0	on failure
 */
 int twl4030charger_ac_en(int enable)
 { 	
 	int ret;
 	
 	if (enable)
 	{
 		/* forcing the field BCIAUTOAC (BOOT_BCI[0) to 1 */
 		if ((ret = clear_n_set(TWL4030_MODULE_PM_MASTER, 0, 
 			(CONFIG_DONE | BCIAUTOWEN | BCIAUTOAC), 
 			REG_BOOT_BCI)))
 		{ 			
			return ret;
 		}
 	}
 	else
 	{
 		/* forcing the field BCIAUTOAC (BOOT_BCI[0) to 0*/
 		if ((ret = clear_n_set(TWL4030_MODULE_PM_MASTER, BCIAUTOAC, 
 			(CONFIG_DONE | BCIAUTOWEN), 
 			REG_BOOT_BCI)))
 		{ 			
			return ret;
 		}
 	}
 	return 0;
 }
EXPORT_SYMBOL(twl4030charger_ac_en);
 
/*
 * API function
 * 
 * Disable/Enable USB Charge funtionality.
 * 
 * 	@enable
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 
 * 	Return 	0	on success
 *     		<0	on failure
 */
 int twl4030charger_usb_en(int enable)
 {
 	u8 value;
	int ret;
	
	if(enable)
	{ 
	
		/* Checking for USB charger conneted */
		ret = twl4030charger_presence();
		if (ret < 0)
		{		
			return ret;
		}
		
		if (!(ret & USB_PW_CONN))
		{		
			return -ENXIO;		
		}
		
		/* forcing the field BCIAUTOUSB (BOOT_BCI[1]) to 1 */
		ret = clear_n_set(TWL4030_MODULE_PM_MASTER, 0, 
			(CONFIG_DONE | BCIAUTOWEN | BCIAUTOUSB),
			REG_BOOT_BCI);
		if(ret)
		{				
			return ret;
		}
		
		/* Enabling interfasing with usb thru OCP*/
		ret = clear_n_set(TWL4030_MODULE_USB, 0, PHY_DPLL_CLK,
			REG_PHY_CLK_CTRL);
		if(ret)
		{				
			return ret;
		}
			
		value = 0;	
		
		while (!(value & PHY_DPLL_CLK))
		{
			udelay(10);
			ret = twl4030_i2c_read_u8(TWL4030_MODULE_USB, &value,
				REG_PHY_CLK_CTRL_STS);
			if(ret)
			{					
				return ret;
			}	
		}	
			
		/* OTG_EN (POWER_CTRL[5]) to 1 */
		ret = clear_n_set(TWL4030_MODULE_USB, 0, OTG_EN,
			REG_POWER_CTRL);	
		if(ret)
		{				
			return ret;
		}
			
		mdelay(50);	
		
		/* forcing USBFASTMCHG(BCIMFSTS4[2]) to 1 */
		ret = clear_n_set(TWL4030_MODULE_MAIN_CHARGE, 0,
			USBFASTMCHG, REG_BCIMFSTS4);
		if(ret)
		{				
			return ret;
		}
	}
	else
	{
		ret = clear_n_set(TWL4030_MODULE_PM_MASTER, BCIAUTOUSB,
			(CONFIG_DONE | BCIAUTOWEN ), REG_BOOT_BCI);
		if(ret)
		{					
			return ret;
		}
	}
	
	return 0;
 }
EXPORT_SYMBOL(twl4030charger_usb_en);

/*
 * API function
 * 
 * Return battery temperature
 * Or < 0 if something fails. 
 */ 
int twl4030battery_temperature(void)
{	
	u8 val;
	int temp, curr, volt, res, ret;
	
	/* Getting and calculating the thermistor voltage */
	ret = read_bci_val(T2_BATTERY_TEMP);
	if (ret < 0)
		return ret;
		
	volt = (ret * 147) / 100;
		
	/* Getting and calculating the supply current in micro ampers */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		 REG_BCICTL2);
	if (ret)
		return 0;
		
	curr = ((val & ITHSENS)  +1) * 10; 
		
	/* Getting and calculating the thermistor resistance in ohms*/
	res = volt * 1000 / curr;
		
	/*calculating temperature*/
	for (temp = 55; temp >= 0; temp--)
	{	
		int actual = twl4030battery_temp_tbl [temp];
		if ((actual - res) >= 0)
			break;
	}
	 
	return temp + 1;		
}
EXPORT_SYMBOL(twl4030battery_temperature);

/*
 * API function
 * 
 * Return battery voltage
 * Or < 0 if something fails. 
 */  
int twl4030battery_voltage(void)
{
	int volt = read_bci_val(T2_BATTERY_VOLT); 
	
	return (volt * 588) / 100;	
}
EXPORT_SYMBOL(twl4030battery_voltage);

/*
 * API function
 * 
 * Return the AC power supply voltage
 * Or < 0 if something fails. 
 */  
int twl4030charger_ac_voltage(void)
{
	int volt = read_bci_val(T2_BATTERY_ACVOLT);
	return (volt * 735) / 100;	
}
EXPORT_SYMBOL(twl4030charger_ac_voltage);

/*
 * API function
 * 
 * Return the USB power supply voltage
 * Or < 0 if something fails. 
 */ 
int twl4030charger_usb_voltage(void)
{
	int volt = read_bci_val(T2_BATTERY_USBVOLT); 
	return (volt * 2058) / 300;
}
EXPORT_SYMBOL(twl4030charger_usb_voltage);

/*
 * API function
 * 
 * Return the battery current
 * Or < 0 if something fails. 
 */ 
int twl4030battery_current(void)
{	
	int ret, curr = read_bci_val(T2_BATTERY_CUR);
	u8 val;
	
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		REG_BCICTL1);
	if (ret)
		return ret;
	
	if (val & CGAIN) /* slope of 0.44 mV/mA */
	{
		return (curr * 147) / 44;
	}
	else /* slope of 0.88 mV/mA */
	{
		return (curr * 147) / 80;
	}
}
EXPORT_SYMBOL(twl4030battery_current);

/*
 * API function
 * 
 * Return the battery backup voltage
 * Or < 0 if something fails. 
 */
int twl4030backupbatt_voltage (void)
{
	int ret, temp;
	u8 volt;
	
	/* trigger MADC convertion */
	twl4030madc_sw1_trigger();
			
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MADC, &volt,
			REG_GPCH9 + 1)))
	{	
		return ret;
	}
	
	temp = ((int) volt) << 2;
	
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MADC, &volt,
		REG_GPCH9)))
	{		
		return ret;
	}
	
	temp = temp + ((int) ((volt & MADC_LSB_MASK) >> 6));

	return  (temp * 441) /100;
}
EXPORT_SYMBOL(twl4030backupbatt_voltage);

/*
 * API function:
 * 
 * Returns an integer value, that means,
 * NO_PW_CONN  no power supply is connected 
 * AC_PW_CONN  if the AC power supply is connected
 * USB_PW_CONN  if the USB power supply is connected
 * AC_PW_CONN + USB_PW_CONN if USB and AC power supplies are both connected
 *                                 
 * Or < 0 if something fails.
 */
int twl4030charger_presence(void)
{
	int ret;
	u8 hwsts;
		
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER,&hwsts,
		REG_STS_HW_CONDITIONS)))
	{
		printk(KERN_ERR "BATTERY DRIVER:"
			 "error reading STS_HW_CONDITIONS \n");
		return ret;
	}
		
	ret = (hwsts & STS_CHG)? AC_PW_CONN: NO_PW_CONN;
	
	ret += (hwsts & STS_VBUS)? USB_PW_CONN: NO_PW_CONN;
	
	return ret;
	
}

/*
 * API function:
 *   
 * Returns the main charge FSM status
 * Or < 0 if something fails. 
 */
int twl4030bci_status(void)
{
	int ret;
	u8 status;
			
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, 
		&status, REG_BCIMSTATEC)))
	{
		printk(KERN_ERR "BATTERY DRIVER: error reading BCIMSTATEC \n");
		return ret;
	}
			
	return (int) (status & BCIMSTAT_MASK);
}
EXPORT_SYMBOL(twl4030bci_status);

/*
 * helper function
 * for read a 2 bytes register on BCI module
 */
static int read_bci_val (u8 reg)
{
	int ret, temp;
	u8 val;
	
	/* reading MSB */
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		reg + 1)))
	{		
		return ret;
	}
	
	temp = ((int)(val & 0x03)) << 8;
	
	/* reading LSB */
	if ((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val, 
		reg)))
	{	
		return ret;
	}
	
	return temp + val;

}

/*
 * helper function
 * 
 * Triggers the sw1 request for the twl4030 module to measure the sw1 selected
 * channels 
 *
 * @retval 
 * 	0	On success 
 *	>0	On failure
 */
static int twl4030madc_sw1_trigger (void)
{
	u8 val;
	int ret;
		
	/* Triggering SW1 MADC convertion */
	if((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MADC, &val,
		REG_CTRL_SW1)))
	{		
		return ret;
	}	
		
	val |= SW1_TRIGGER;
	
	if((ret = twl4030_i2c_write_u8(TWL4030_MODULE_MADC, val,
		REG_CTRL_SW1)))
	{		
		return ret;
	}
	
	/* Waiting until the SW1 conversion ends*/
	val =  BUSY;
	
	while (!((val & EOC_SW1) && (!(val & BUSY))))
	{
		if((ret = twl4030_i2c_read_u8(TWL4030_MODULE_MADC, &val,
			REG_CTRL_SW1)))
		{		
			return ret;
		}
		mdelay(10);
	}	
	
	return 0;
}

/*
 * helper function
 * 
 * Settup the twl4030 MADC module to measure the backup
 * battery voltage.
 *
 * @retval 
 * 	0	On success 
 *	>0	On failure
 */
static int twl4030backupbatt_voltage_setup(void)
{	
	int ret;
	
	/* turning adc_on */
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_MADC, MADC_ON,
		REG_CTRL1);		
	if(ret)
		return ret;	
		
	/*setting MDC channel 9 to trigger by SW1*/		
	ret = clear_n_set(TWL4030_MODULE_MADC, 0, SW1_CH9_SEL,
		REG_SW1SELECT_MSB);
	if (ret)
		return ret;
	
	/* Starting backup batery charge */
	ret = clear_n_set(TWL4030_MODULE_PM_RECEIVER, 0, BBCHEN,
		REG_BB_CFG);
	if (ret)
		return ret;
				
	return 0;
}

/*
 * helper function
 * 
 * Settup the twl4030 BCI and MADC module to measure battery 
 * temperature
 *
 * @retval 
 * 	0	On success 
 *	>0	On failure
 */
static int twl4030battery_temp_setup(void)
{
	int ret;
	
	/* Enabling thermistor current */
	ret = clear_n_set(TWL4030_MODULE_MAIN_CHARGE, 0, ITHEN,
		REG_BCICTL1);
	if(ret)
		return ret;
	
	return 0;	
}

/*
 * helper function
 * 
 * Sets and clears bits on an given register on a given module
 * 
 * @ mod_no
 * 		module of the twl4030 chip were the given register is located
 * @ reg
 * 		Address offset of the given register
 *
 * @	clear
 * 		bits to clear (all in ones)
 *  
 * @ set
 * 		bits to set (all in ones)
 * 
 * @ retrieve
 * 	0	On success
 * 	<0	On failure
 * 
 */
static int clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg)
{
	int ret;
	u8 val = 0;
	
	/* Gets the initial register value */
	if((ret = twl4030_i2c_read_u8(mod_no, &val, reg)))
		return ret;
		
	/* Clearing all those bits to clear */
	val &= ~(clear);
	
	/* Setting all those bits to set */
	val |= set;	
		
	/* Update the register */
	if((ret = twl4030_i2c_write_u8(mod_no, val, reg)))
		return ret;	
		
	return 0;
}


static void twl4030_bat_event_handler(struct work_struct *work)
{
	int ret;

	ret = twl4030bci_status_evt();
	if (ret)
	{
		twl4030bci_sw_status_en(EVT_DISABLE);
	}
	else
	{
		twl4030bci_sw_status_en(EVT_ENABLE);
	}
}

static int omap_bci_battery_probe(struct  platform_device *dev)
{
	int ret;

	/* Enable AC and USB chagre */
	twl4030charger_ac_en(1);
	twl4030charger_usb_en(1);

	/* settings for temperature sensing */	
	ret = twl4030battery_temp_setup();
	if(ret)
		goto exit2;

	/* enabling GPCH09 for read back battery voltage */
	ret = twl4030backupbatt_voltage_setup();
	if(ret)		
		goto exit2;	
	
#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS
	
	/* setup for charge status monitoring */	
	init_timer(&twl4030bci_status_tmr);
	twl4030bci_status_tmr.function = twl4030bci_status_tmr_fcn;
	twl4030bci_status_tmr.data = (unsigned long) &twl4030bci_status_tmr;
	INIT_WORK(&twl_event, twl4030_bat_event_handler); 
	
	/* request BCI interruption */
	ret = request_irq(TWL4030_MODIRQ_BCI, twl4030battery_interrupt, 
		IRQF_DISABLED, dev->name, NULL);
	if(ret)
	{
		printk(KERN_ERR "BATTERY DRIVER: (BCI) IRQ%d is not free.\n",
			TWL4030_MODIRQ_PWR);
		goto exit2;				
	}
	
	/* Previous setup for power interrupt handling */
	ret = clear_n_set(TWL4030_MODULE_INT, COR, 0,
		REG_PWR_SIH_CTRL);
	if (ret)
	{
		printk(KERN_ERR "BATTERY DRIVER:"
			"power interrupt previous settup error.\n");
		goto exit3;
	}
	
	/* request Power interruption */
	ret = request_irq(TWL4030_MODIRQ_PWR, twl4030charger_interrupt,
		IRQF_DISABLED | IRQF_SHARED, dev->name, 
		twl4030charger_interrupt);
	if(ret)
	{
		printk(KERN_ERR "BATTERY DRIVER: (POWER) IRQ%d is not free.\n",
			TWL4030_MODIRQ_PWR);
		goto exit3;				
	}
		
#endif  /* CONFIG_OMAP_TWL4030_BATTERY_EVENTS */

#ifdef CONFIG_OMAP_BATTERY_SYSFS
	if (device_create_file(&dev->dev, &dev_attr_t2_temperature) < 0)
		printk(KERN_ERR "Battery driver could not"
				 "create t2 temp sysfs entry\n");
	if (device_create_file(&dev->dev, &dev_attr_t2_voltage) < 0)
		printk(KERN_ERR "Battery driver could not"
				"create t2 volt sysfs entry\n");
	if (device_create_file(&dev->dev, &dev_attr_t2_current) < 0)
		printk(KERN_ERR "Battery driver could not"
				"create t2 curr sysfs entry\n");
	if (device_create_file(&dev->dev, &dev_attr_t2_backup_voltage) < 0)
		printk(KERN_ERR "Battery driver could not"
				"create t2 backup volt sysfs entry\n");
#endif
			
	return 0;

#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS
	
	exit3:
		free_irq(TWL4030_MODIRQ_BCI, NULL);
	
#endif  /* CONFIG_OMAP_TWL4030_BATTERY_EVENTS */

	exit2:
		platform_driver_unregister(&omap_bci_battery_driver);

	return 0;
}

static int omap_bci_battery_remove(struct  platform_device *dev)
{

#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS

	free_irq(TWL4030_MODIRQ_BCI, NULL);
	free_irq(TWL4030_MODIRQ_PWR, twl4030charger_interrupt);

#endif  /* CONFIG_OMAP_TWL4030_BATTERY_EVENTS */

#ifdef CONFIG_OMAP_BATTERY_SYSFS
	device_remove_file(&dev->dev, &dev_attr_t2_temperature);
	device_remove_file(&dev->dev, &dev_attr_t2_voltage);
	device_remove_file(&dev->dev, &dev_attr_t2_current);
	device_remove_file(&dev->dev, &dev_attr_t2_backup_voltage);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int omap_bci_battery_suspend(struct platform_device *dev,
	pm_message_t state)
{
#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS
	/* Disable the sw monitering */
	twl4030bci_sw_status_en(EVT_DISABLE);
			
#endif /* CONFIG_OMAP_TWL4030_BATTERY_EVENTS*/
		
	return 0;
}

static int omap_bci_battery_resume(struct platform_device *dev)
{
#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS
	if(twl4030battery_event.bci_status != NULL)
	{
		/* Check if the timer needs to be added here again */
		twl4030bci_sw_status_en(EVT_ENABLE);
		twl4030battery_event.temp_std = -1;
		twl4030bci_status_evt();
	}       /* Resume the software monitering */
	
#endif /* CONFIG_OMAP_TWL4030_BATTERY_EVENTS*/
	return 0;
}
#endif /* CONFIG_PM */

/*
 * Battery driver module initializer function
 * registers battery driver structure
 */
static int __init omap_battery_init(void)
{
	int ret;

	if((ret = platform_driver_register(&omap_bci_battery_driver)))
	{
		printk(KERN_ERR  "BATTERY DRIVER:"
			"driver register Failed ...!!!!\n");
	}

	return 0;
}

/*
 * Battery driver module exit function
 * unregister battery driver structure
 */
static void __exit omap_battery_exit(void)
{	
	platform_driver_unregister(&omap_bci_battery_driver);
}

module_init(omap_battery_init);
module_exit(omap_battery_exit);

MODULE_LICENSE("GPL");
