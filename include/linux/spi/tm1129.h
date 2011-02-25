/*
 * Copyright (C) 2008-2009 Palm, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#ifndef __TM1129_H
#define __TM1129_H

#include <linux/spi/spi.h>

struct tm1129_platform_data {
    int penup_timeout;  // msec
    int flip_x;         // Flip the X coordinate?
    int flip_y;         // Flip the Y coordinate?
};

#define SPI_READ_BIT			0x80

/* Standard RMI control, command, and status registers */
#define DEVICE_CONTROL			0x0000
#define INTERRUPT_ENABLE		0x0001
#define ERROR_STATUS			0x0002
#define INTERRUPT_REQ_STATUS		0x0003
#define DEVICE_COMMAND			0x0004

#define DEVICE_COMMAND_RESET		(1 << 0)	
#define INTERRUPT_ENABLE_ABS_INT_EN	(1 << 0)
#define INTERRUPT_ENABLE_REL_INT_EN	(1 << 1)
#define INTERRUPT_ENABLE_BTN_INT_EN	(1 << 2)

#define INTERRUPT_REQ_STATUS_ABS_INT_RQ	(1 << 0)
#define INTERRUPT_REQ_STATUS_REL_INT_RQ	(1 << 1)
#define INTERRUPT_REQ_STATUS_BTN_INT_RQ	(1 << 2)

#define SAMPLE_RATE_80HZ		(2 << 6)
#define NORMAL_OPERATION		(1 << 0)
#define DEEP_SLEEP			(6 << 0)

/* Standard RIM query registers */
#define RMI_PROTOCOL_VERSION		0x0200
#define MANUFACTURER_ID			0x0201
#define PHY_INTERFACE_VERSION		0x0202
#define PROD_PROPERTY			0x0203
#define PROD_INFO_0			0x0204
#define PROD_INFO_1			0x0205
#define PROD_INFO_2			0x0206
#define PROD_INFO_3			0x0207

/* Data registers */
#define DATA_REG_0			0x0400
#define DATA_REG_1			0x0401
#define DATA_REG_2			0x0402
#define DATA_REG_3			0x0403
#define DATA_REG_4			0x0404
#define DATA_REG_5			0x0405
#define DATA_REG_6			0x0406
#define DATA_REG_7			0x0407
#define DATA_REG_8			0x0408
#define DATA_REG_9			0x0409
#define DATA_REG_10			0x040a
#define DATA_REG_11			0x040b
#define REL_HRZ_MOTION			0x040c
#define REL_VRT_MOTION			0x040d 
#define BUTTON_DATA			0x040e 
#define BUTTON_0_Z			0x040f
#define BUTTON_1_Z			0x0410
#define BUTTON_2_Z			0x0411
#define BUTTON_3_Z			0x0412
#define BUTTON_4_Z			0x0413

#define DATA_REG_0_SENSOR_STATUS	(7 << 0)
#define TRANSITIONAL_FINGER_COUNT	0x7

/* 2D sensor function registers */
#define FUNCTION_VERSION		0x1000
#define SENSOR_PROP			0x1003
#define SENSOR_X_MAX_HIGH		0x1004 
#define SENSOR_X_MAX_LOW		0x1005 
#define SENSOR_Y_MAX_HIGH		0x1006 
#define SENSOR_Y_MAX_LOW		0x1007 
#define SENSOR_RESOLUTION		0x1008 

#define SENSOR_PROP_2D_SCROLL		(1 << 5)
#define SENSOR_PROP_SCROLLER		(1 << 4)
#define SENSOR_PROP_MULTI_FINGER	(1 << 2)
#define SENSOR_PROP_PALM_DET		(1 << 0)

/* Function-specific registers */
#define GEN_2D_CONTROL			0x1041
#define SENSOR_SENSITIVITY		0x1044
#define SENSOR_MAX_POS_0		0x1046
#define SENSOR_MAX_POS_1		0x1047

#define GEN_BUTTON_CONTROL_0		0x1841
#define GEN_BUTTON_CONTROL_1		0x1842

#endif
