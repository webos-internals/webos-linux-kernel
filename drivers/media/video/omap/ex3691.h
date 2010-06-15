/*
 * drivers/media/video/omap/ex3691.h
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 * Register defines for TI EX3691 camera module (also known as Micron MT9D111)  
 *
 */

#ifndef EX3691_H
#define EX3691_H

#define EX3691_ID			0x1519
/* firmware drivers ID */
#define MON_DRV_ID			0
#define SEQ_DRV_ID			1
#define AE_DRV_ID			2
#define AWB_DRV_ID			3
#define FD_DRV_ID			4
#define AF_DRV_ID			5
#define AFM_DRV_ID			6
#define MODE_DRV_ID			7

/* Monitor driver variables */
#define MON_VER_OFFSET			12

/* Sequencer driver variables */
#define SEQ_MODE_OFFSET			2
#define SEQ_STATE_OFFSET		4

/* Sequencer Driver states */
#define SEQ_STATE_INIT			0
#define SEQ_STATE_TO_PREVIEW		1
#define SEQ_STATE_ENTER_PREVIEW		2
#define SEQ_STATE_PREVIEW		3
#define SEQ_STATE_LEAVE_PREVIEW		4
#define SEQ_STATE_TO_CAPTURE		5
#define SEQ_STATE_ENTER_CAPTURE		6
#define SEQ_STATE_CAPTURE		7
#define SEQ_STATE_LEAVE_CAPTURE		8
#define SEQ_STATE_STANDBY		9

/* pages */
#define PAGE_SENSOR_CORE		0
#define PAGE_IFP1			1
#define PAGE_IFP2			2

/* used registers */
#define REG_SENSOR_VER			0x00 
#define REG_RESET			0x0D
#define REG_PAGE			0xF0
#define REG_PAD_SLEW			0x0A 
#define REG_VAR_ADDR			0xC6 
#define REG_VAR_DATA			0xC8 

/* Bits for Microcontroller variable access using logic address */
#define VAR_LENGTH_16BIT		0
#define VAR_LENGTH_8BIT			1
#define VAR_LENGTH_SHIFT		15
#define VAR_ADDRESS_TYPE_LOGIC		1
#define VAR_ADDRESS_TYPE_SHIFT		13
#define VAR_DRV_ID_SHIFT		8
#define VAR_ADDRESS_SHIFT		0


#endif /* ifndef EX3691_H */

