/*
 *  include/linux/i2c_smb339_charger.h
 *
 *  Copyright (C) 2009 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Chris Anderson (chris.anderson@palm.com)
 * 
 */

#ifndef _SMB339_THERMISTOR_H
#define _SMB339_THERMISTOR_H

#include <linux/ioctl.h>

#define SMB339_I2C_DEVICE       "SMB339"
#define SMB339_I2C_DRIVER       "SMB339"
#define SMB339_I2C_ADDR         0x2A

// Register 2
#define SMB339_FLOAT_VOLTAGE_420	0x4A

// Register 3
#define SMB339_AUTO_RCHRG_DISABLE   0x80
#define SMB339_CRNT_NO_END_CHRG	    0x40
#define SMB339_DISABLE_GLITCH_FILTER	0x04

// Register 4
#define SMB339_SAFETY_TIMER_DISABLE  0x08
#define SMB339_BATT_OV_END_CHRG   0x40

// Register 5
#define SMB339_PIN_CONTROL_ENABLE  0x01

// Register 31
#define SMB339_VOLATILE_WRITE   0x80
#define SMB339_DISABLE_CHARGING     0x10;
#define SMB339_AC_MODE          0x04

// Ioctls
#define SMB339_SET_MAX_CURRENT  _IOW(SMB339_I2C_ADDR, 0x01, int)

// Register enums
typedef enum {
    SMB339_REG_OUTPUT = 0,
    SMB339_REG_INPUT,
    SMB339_REG_FLOAT,
    SMB339_REG_CONTROL_A,
    SMB339_REG_CONTROL_B,
    SMB339_REG_PIN_CONTROL,
    SMB339_REG_OTG_CONTROL,

    SMB339_REG_CLEARIRQ = 0x30,
    SMB339_REG_COMMAND,

    SMB339_REG_PWRSOURCE = 0x34,
    SMB339_REG_BATTERY_STATUS_A,
    SMB339_REG_BATTERY_STATUS_B,
    SMB339_REG_BATTERY_STATUS_C,

    SMB339_REG_VERSION = 0x3B
} smb339_registers;

typedef enum {
    SMB339_CURRENT_550mA = 0,
    SMB339_CURRENT_650mA,
    SMB339_CURRENT_750mA,
    SMB339_CURRENT_850mA,
    SMB339_CURRENT_950mA,
    SMB339_CURRENT_1050mA,
    SMB339_CURRENT_1150mA,
    SMB339_CURRENT_1250mA,
    SMB339_CURRENT_DISABLE,
    SMB339_CURRENT_END,
} smb339_current_values;

#endif
