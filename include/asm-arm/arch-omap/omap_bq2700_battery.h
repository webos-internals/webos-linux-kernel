/*
 * include/asm-arm/arch-omap/omap_hdq_battery.h
 * 
 * BQ2700 Battery interface for Linux
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

#ifndef __OMAP_HDQ_BATTERY__
#define __OMAP_HDQ_BATTERY__ 

#ifdef CONFIG_OMAP_BQ27000_BATTERY_EVENTS
#define BATT_LVL_DLY		60000
#define BATT_PRES_DLY		60000

/* Settings event enable/disable */
#define EVT_ENABLE		0x001
#define EVT_DISABLE		0x000

/* Charger presence events */
#define EVT_PWSPLY_AC_CONN	0x001
#define EVT_PWSPLY_AC_DISC	0x002
#define EVT_PWSPLY_USB_CONN	0x004
#define EVT_PWSPLY_USB_DISC	0x008

/* Battery voltage level event enable flags*/
#define EVT_LVL_4_EN		0x008
#define EVT_LVL_3_EN		0x004
#define EVT_LVL_2_EN		0x002
#define EVT_LVL_1_EN		0x001

#define VOLT_THR		10

/* Battery presence events*/
#define EVT_BATTSTS_CONN	0x001
#define EVT_BATTSTS_DISC	0x002

/* battery level events*/
#define EVT_LVL_1		0x001
#define EVT_LVL_2		0x002
#define EVT_LVL_3		0x004
#define EVT_LVL_4		0x008

/*
 * Battery event service rutines structure
 * 
 * This structure contains a set of function ponters
 * which points to the BQ battery event service 
 * routines. And a battery level mask register, which 
 * mask or unmask the software driven battery level
 * events.
 * 
 */
struct bqbattery_events
{	
	int (*battery_level)(int batt_level_event);	
	int (*battery_presence)(int batt_pres_event);	
	int battery_sw_level_event_cfg;
	int battery_level_1;
	int battery_level_2;
	int battery_level_3;
	int battery_level_4;
	int temp_level;
	int temp_presence;
	
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
int bqbattery_event_register(struct bqbattery_events events);

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
int bqbattery_sw_level_set(int level, int volt);
/*
 * API function
 * 
 * Gets the voltage setted limit for any of the four battery voltage thresholds in
 * milivolts
 * 
 * 	 @level
 *  
 * 		EVT_LVL_4		voltage level 4
 *		EVT_LVL_3		voltage level 3
 *		EVT_LVL_2		voltage level 2
 *		EVT_LVL_1		voltage level 1
 */
int bqbattery_sw_level_get(int level);
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
 *Return 	0	on success
 *     		<0	on failure 
 */
int bqbattery_sw_level_en(int enable_lvl);
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
 *     	<0				on failure 
 */
int bqbattery_sw_level_qu(void);

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
int bqbattery_sw_presence_en(int enable);

/*
 * API function
 * 
 * Querries for battery voltage presence event notifications status.
 * 
 * Return 
 * 		EVT_ENABLE	Enable 
 * 		EVT_DISABLE	Disable
 *  	   	<0	On failure
 */
int bqbattery_sw_presencel_qu(void);

#endif /* CONFIG_OMAP_BQ27000_BATTERY_EVENTS */
/*
 * API function
 * 
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
int bqbattery_temperature(void);
/*
 * API function
 * 
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
int bqbattery_voltage(void);
/*
 * API function
 * 
 * Return the battery Average current
 * Or < 0 if something fails.
 */
int bqbattery_current(void);
/*
 * API function:
 * 
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
int bqbattery_rsoc(void);
#endif  /* __OMAP_HDQ_BATTERY__ */
