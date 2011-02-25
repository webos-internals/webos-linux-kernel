/*
 * drivers/media/video/omap/d88.h
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * Copyright (C) 2007 Texas Instruments.
 *
 * Register defines for Auto Focus device
 *
 */
#ifndef CAMAF_DB88_H
#define CAMAF_DB88_H

/* Register Definitions */
#define CAMAF_D88_NXT_POSN		0x00
#define CAMAF_D88_CUR_POSN		0x01
#define CAMAF_D88_POR_STDBY		0x03
#define CAMAF_D88_THRSH			0x05
#define CAMAF_D88_FREQ_DIV		0x06
#define CAMAF_D88_REV			0x08
#define CAMAF_D88_ID			0x09

#define CAMAF_FREQUENCY_EQ1(mclk)     ((u16)(mclk/16000))

#define MID_FOCUS_POS	50

/*
 * Initializes D88 device for auto focus
 */ 
int d88_af_init(unsigned long mclk);

/*
 * Counterpart of initialization; for clean up
 */ 
int d88_af_deinit(void);

/*
 * Sets the specified focus value [0(far) - 100(near)]
 */ 
int d88_af_setfocus(u8 posn);

#endif /* End of of CAMAF_DB88_H */
