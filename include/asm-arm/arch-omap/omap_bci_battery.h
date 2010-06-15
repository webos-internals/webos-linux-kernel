/*
 * include/asm-arm/arch-omap/omap_bci_battery.h
 * 
 * TWL4030 Battery interface for Linux
 * 
 *
 * Copyright (C) 2007 Texas Instruments, Inc. 
 *  
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
 
#ifndef __OMAP_BCI_BATTERY__
#define __OMAP_BCI_BATTERY__

/*  TWL4030 battery measuring parameters */
#define T2_BATTERY_VOLT		0x04 
#define T2_BATTERY_TEMP		0x06
#define T2_BATTERY_CUR		0x08
#define T2_BATTERY_ACVOLT	0x0A
#define T2_BATTERY_USBVOLT	0x0C
#define T2_BATTERY_BKVOLT

/* charger constants */
#define NO_PW_CONN		0
#define AC_PW_CONN		0x01
#define USB_PW_CONN		0x02

/* TWL4030_MODULE_USB */
#define REG_POWER_CTRL		0x0AC
#define REG_POWER_CTRL_SET 	0x0AD
#define REG_POWER_CTRL_CLR	0x0AE

#define OTG_EN			0x020

#define REG_PHY_CLK_CTRL	0x0FE
#define REG_PHY_CLK_CTRL_STS 	0x0FF

#define CLK32K_EN 		0x02
#define CLOCKGATING_EN		0x04

#define PHY_DPLL_CLK		0x01

#ifdef CONFIG_OMAP_TWL4030_BATTERY_EVENTS
/* charge status*/
#define CHG_STS_NONE		0x000
#define CHG_STS_OFF		0x001
#define CHG_STS_STBY		0x002
#define CHG_STS_OPN		0x003

#define CHG_STS_STD		0x008  /* Standar charge other wise quick charge */
#define CHG_STS_AC		0x020
#define CHG_STS_USB		0x010

#define CHG_STS_CONS		0x001
#define CHG_STS_OVVT		0x00E

#define CHG_STS_QCK1		0x002
#define CHG_STS_QCK2		0x003
#define CHG_STS_QCK3		0x004
#define CHG_STS_QCK4		0x005
#define CHG_STS_QCK5		0x006
#define CHG_STS_QCK6		0x007

#define CHG_STS_STP1		0x000
#define CHG_STS_STP2		0x001
#define CHG_STS_STP3		0x002 
#define CHG_STS_STD1		0x003
#define CHG_STS_STD2		0x004
#define CHG_STS_STD3		0x005
#define CHG_STS_STD4		0x006

#define CHG_STS_DLY		4000

#ifndef EVT_ENABLE
/* Battery voltage level event enable flags*/
#define EVT_LVL_4_EN		0x008
#define EVT_LVL_3_EN		0x004
#define EVT_LVL_2_EN		0x002
#define EVT_LVL_1_EN		0x001

/* Charger presence events */
#define EVT_PWSPLY_AC_CONN	0x001
#define EVT_PWSPLY_AC_DISC	0x002
#define EVT_PWSPLY_USB_CONN	0x004
#define EVT_PWSPLY_USB_DISC	0x008

/* Battery presence events*/
#define EVT_BATTSTS_CONN	0x001
#define EVT_BATTSTS_DISC	0x002

/* battery level events*/
#define EVT_LVL_1		0x001
#define EVT_LVL_2		0x002
#define EVT_LVL_3		0x004
#define EVT_LVL_4		0x008

#endif /*  EVT_ENABLE  */

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
struct twl4030battery_events
{	
	int (*battery_level)(int batt_level_event);	
	int (*battery_presence)(int batt_pres_event);
	int (*charger_presence)(int chgr_pres_event);
	int (*bci_status)(int bci_sts_event);
	int battery_sw_level_event_cfg;
	int temp_std;
};

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
int twl4030battery_event_register(struct twl4030battery_events events);

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
 * 		for EVT_LVL_1: 
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
 * 		for EVT_LVL_3: 
 * 			LSB =23.4 mV
 * 			Default [0x06] =3.902V
 * 			Minimum [0x00] =3.761V
 * 			Maximum [0x0f] =4.113V
 * 
 * 		for EVT_LVL_4: 
 * 			LSB =23.4 mV
 * 			Default [0x08] =3.949V
 * 			Minimum [0x00] =3.761V
 * 			Maximum [0x0f] =4.113V
 * 	 
 */
int twl4030battery_sw_level_set(int level, int volt);
/*
 * API function
 * 
 * Gets the voltage setted limit for any of the four battery voltage thresholds 
 * in milivolts
 * 
 * 	 @level
 *  
 * 		EVT_LVL_4		voltage level 4
 *		EVT_LVL_3		voltage level 3
 *		EVT_LVL_2		voltage level 2
 *		EVT_LVL_1		voltage level 1
 * 		
 */
int twl4030battery_sw_level_get(int level);

/*
 * API function
 * 
 * Disable/Enable by software individual battery level event notifications.
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
 * will disable the respective event. Passing a 0 will not disable the hardware
 * interruption. 
 *   
 *	Return 	0	on success
 *     		<0	on failure 
 */
int twl4030battery_sw_level_en (int enable_lvl);

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
 *  	   	<0			on failure 
 */
int twl4030battery_sw_level_qu (void);

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
int twl4030battery_hw_level_en (int enable);

/*
 * API function
 * 
 * Queries hardware battery level event notifications status.
 * 
 * 	Return
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *  	   	<0	On failure
 */
int twl4030battery_hw_level_qu (void);

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
int twl4030battery_hw_presence_en(int enable);

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
int twl4030battery_hw_presence_qu(void);

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
int twl4030charger_hw_presence_en(int enable);

/*
 * API function
 * 
 * Queries hardware charger presence event notification status.
 * 
 * 	Return
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 * 	    	<0	On failure
 */
int twl4030charger_hw_presence_qu(void);

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
int twl4030bci_sw_status_en(int enable);

/*
 * API function
 * 
 * Querries for battery charge status event notifications status.
 * 
 * 	Return 
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *  	   	<0	On failure
 */
int twl4030bci_sw_status_qu(void);

#endif /* CONFIG_OMAP_TWL4030_BATTERY_EVENTS */

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
 int twl4030charger_ac_en(int enable);
 
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
 int twl4030charger_usb_en(int enable);
/* 
 * API function
 * 
 * Return battery temperature
 * Or < 0 if something fails. 
 */
 int twl4030battery_temperature(void);

/*
 * API function
 * 
 * Return battery voltage
 * Or < 0 if something fails. 
 */ 
int twl4030battery_voltage(void);

/*
 * API function
 * 
 * Return the AC power supply voltage
 * Or < 0 if something fails. 
 */ 
int twl4030charger_ac_voltage(void);

/*
 * API function
 * 
 * Return the USB power supply voltage
 * Or < 0 if something fails. 
 */ 
int twl4030charger_usb_voltage(void);

/*
 * API function
 * 
 * Return the battery current
 * Or < 0 if something fails. 
 */
int twl4030battery_current(void);

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
int twl4030charger_presence(void);
/*
 * API function:
 *   
 * Returns the main charge FSM status
 * Or < 0 if something fails. 
 */
int twl4030bci_status(void);

/*
 * API function
 * 
 * Return the battery backup voltage
 * Or < 0 if something fails. 
 */
int twl4030backupbatt_voltage (void);
#endif  /* __OMAP_BCI_BATTERY__ */
