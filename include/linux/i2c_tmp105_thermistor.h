/*
 *  include/linux/i2c_tmp105_thermistor.h
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

#ifndef _TMP105_THERMISTOR_H
#define _TMP105_THERMISTOR_H

#define TMP105_I2C_DEVICE       "TMP105"
#define TMP105_I2C_DRIVER       "TMP105"
#define TMP105_I2C_ADDR         0x48

#define TMP105_REG_TEMP            0
#define TMP105_REG_CONFIG          1 
#define TMP105_REG_TLOW            2
#define TMP105_REG_THIGH           3
#define TMP105_CFG_SD              1
#define TMP105_CFG_OS              128

#endif
