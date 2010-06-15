/*
 * include/asm-arm/arch-omap/gci/omap34xxcam_palkern.h
 * 
 * Top level public header file for PAL KERN module in
 * TI OMAP3430 Camera ISP
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

#ifndef _PALKERN_H
#define _PALKERN_H

#define PAL_READ	0
#define PAL_WRITE	1
/* Sensor board details redefined here */
#define VAUX_2_8_V		0x09
#define VAUX_DEV_GRP_P1		0x20
#define SENSOR_RESET_GPIO	98

#if 0
struct camisp_regtrans
{
	unsigned char *data;
	unsigned char ndata;
	unsigned char attr;
};
#endif


struct gpio_data
{
	int gpio_num;
	int is_input;
	int value;
};

typedef enum
{
	ISE_PAL_REG_TRANS_TYPE_WRITE = 1,
	ISE_PAL_REG_TRANS_TYPE_READ
} ISE_PAL_REG_TRANS_TYPE_T;

typedef enum
{
	ISE_FALSE = 0,
	ISE_TRUE = 1
} ISE_BOOLEAN_T;

typedef enum
{
	ISE_PAL_REG_TYPE_I2C = 1,
	ISE_PAL_REG_TYPE_CAM = 2
} ISE_PAL_REG_TYPE_T;

typedef enum{
	ISE_PAL_REG_STATUS_OK = 0 ,
	ISE_PAL_REG_STATUS_FAIL = 1,
	ISE_PAL_REG_STATUS_INVAL_PARAM,
	ISE_PAL_REG_STATUS_BLOCK
}ISE_PAL_REG_STATUS_T;

typedef struct ISE_PAL_REG_TRANS_S
{
	ISE_PAL_REG_STATUS_T status;
	ISE_PAL_REG_TRANS_TYPE_T trans_type;
	unsigned char *data;
	unsigned int size;
} ISE_PAL_REG_TRANS_T;

/* 
* Group of regtrans made in the GCI-PAL Mapper to send to the paluser 
* to direct to the pal kernel.
*/
struct pal_nregtrans
{
	ISE_PAL_REG_TRANS_T *alltrans;
	unsigned int nregtrans;
};

typedef struct ISE_PAL_REG_CONFIG_I2C_S
{
	unsigned int bus_speed_khz;
	/* The 8-bit write address of the imager. 
	 * This is the same thing as the 7-bit slave address 
	 * left shifted by 1.
	 */
	unsigned char write_addr;
	unsigned int drive_strength;
	/* Added this for specifying the I2C bus number */
	unsigned char i2c_num;
} ISE_PAL_REG_CONFIG_I2C_T;


typedef struct ISE_PAL_REG_CONFIG_S{
	ISE_BOOLEAN_T prevent_block;
	ISE_PAL_REG_TYPE_T reg_type;
	union
	{
	ISE_PAL_REG_CONFIG_I2C_T i2c_config;
	} type_config;
}ISE_PAL_REG_CONFIG_T;

/*
 *	I O C T L   C O D E S   F O R   ISE
 *
 */
#define GEN_CAM_PAL_REG_OPEN		_IOW  ('G', 0, ISE_PAL_REG_CONFIG_T)
#define GEN_CAM_PAL_REG_CLOSE		_IOW  ('G', 6, int)
#define GEN_CAM_PAL_REG_TRANSFER	_IOWR ('G', 1, struct pal_nregtrans)
#define GEN_CAM_PAL_REG_DETACH		_IOW  ('G', 2, int) 
#define GEN_CAM_PAL_GPIO_GET		_IOWR ('G',3, struct gpio_data)
#define GEN_CAM_PAL_GPIO_SET		_IOW ('G',4, struct gpio_data)
#define GEN_CAM_PAL_T2_VAUX_SET		_IOW ('G',7, unsigned int)
#define GEN_CAM_PAL_SET_MASTER_CLOCK	_IO   ('G',5)
#define GEN_CAM_PAL_UDELAY		_IOW ('G', 8, int)

#endif

