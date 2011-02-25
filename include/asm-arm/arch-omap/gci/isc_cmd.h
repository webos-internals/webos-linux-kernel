/*
 * include/linux/asm-arm/arch-omap/gci/isc_cmd.h
 *
 * Top level public header file for ISC module in
 * TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2007 Texas Instruments.
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
 

#ifndef __ISC_CMD_H__
#define __ISC_CMD_H__

#include <linux/types.h>
#include <asm/arch/cpu.h>

#define TRUE 1
#define FALSE 0

#define APP_ERROR	1 << 0
#define I2C_ERROR 	1 << 1
#define SENSOR_WARN	1 << 6
#define SENSOR_ERROR	1 << 7

#define SENSOR_WIDTH8	1
#define SENSOR_WIDTH16	2
#define SENSOR_WIDTH32	4

#define PRIMARY_SENSOR		0x0
#define SECONDARY_SENSOR	0x1

#define SENSOR_FORMAT_BAYER10 		0
#define SENSOR_FORMAT_BAYER9		1
#define SENSOR_FORMAT_BAYER8		2
#define SENSOR_FORMAT_RGB565		3
#define SENSOR_FORMAT_YUV422		4

/* major rev=0; moinor rev =10; sub rev=0; */
#define CAM_APP_VERSION		(0x0100) 


#define CAM_HW_VERSION (OMAP3430_REV_ES1_0)

//HW PLATFORM
typedef enum
{
	OMAP3430_ES10_SDP =0x0100, //3430 ES1.0 SDP platform
	OMAP3430_ES20_SDP =0x1001, //3430 ES2.0 SDP platform
}HWPLATFORM;

/* Structure used to indicate any changes in 
 * a section of a particular page
 */
typedef struct
{
	unsigned short cam_general_register 	: 1;
	unsigned short cam_sensor_control	: 1;
	unsigned short cam_prev_config		: 1;
	unsigned short cam_img_config		: 1;
	unsigned short cam_tuning_control	: 1;
	unsigned short cam_exp_config		: 1;
	unsigned short cam_wb_config		: 1;
	unsigned short cam_clock_control	: 1;
	unsigned short reserved 		: 8;
} CAMERA_REG_STATE;


#define PREVIEW_CONFIG_EVENT			(1 << 0)
#define PREVIEW_CONFIG_FMT_EVENT		(1 << 24)
#define PREVIEW_CONFIG_SZ_EVENT			(1 << 25)
#define PREVIEW_MODE_EVENT			(1 << 1)
#define PREVIEW_CONFIG_APPLIED_EVENT		(1 << 2)
/* the HOST wait for CONFIG_APPLIED message */
#define CONFIG_APPLIED_PEND_EVENT		(1 << 3)
#define IMAGE_CONFIG_EVENT			(1 << 4)
#define CAPTURE_MODE_EVENT			(1 << 5)


#define TUNING_CONFIG_EVENT			(1 << 6)
#define EXPOSURE_CONFIG_EVENT			(1 << 7)
#define WHITEBALANCE_CONFIG_EVENT		(1 << 8)
#define FOCUS_CONFIG_EVENT			(1 << 9)

#define CLOCK_CONTROL_EVENT			(1 << 10)
#define CAPTURE_STOP_EVENT			(1 << 11)
#define PREVIEW_STOP_EVENT			(1 << 12)

#define SENSOR_CONFIG_EVENT			(1 << 16)
#define SENSOR_MODE_EVENT			(1 << 17)
#define GENERAL_SETTINGS_EVENT			(1 << 18)

#define SYS_STATE_PREVIEW_ON_FLAG		(1 << 0)
#define SYS_STATE_STOPPED_FLAG			(1 << 1)

#define CAM_HW_PLATFORM (OMAP3430_ES10_SDP)

//Register address Base
#define REG_MAP				(0xBC000000)

#define PAGE0				(0x0)
#define PAGE1				(0x1)
#define PAGE2				(0x2)
#define PAGE3				(0x3)
#define PAGE4				(0x4)
#define PAGE5				(0x5)
#define PAGE6				(0x6)
#define PAGE7				(0x7)
#define PAGE8				(0x8)
#define PAGE9				(0x9)
#define PAGEA				(0xA)
#define PAGEB				(0xB)
#define PAGEC				(0xC)
#define PAGEE				(0xE)

#define PAGE0_SIZE			0x1A
#define PAGE1_SIZE			0x44
#define PAGE2_SIZE			0x19
#define PAGE3_SIZE			0x0F

/* Message with attribute  byte 0:offsetAddress  byte1: Attrib */
#define  ATTRIB_READ_ONLY			(1)
#define  ATTRIB_WRITE_ONLY			(1<< 1)
#define  ATTRIB_SHADOW				(1<< 2)
#define  ATTRIB_READ_WRITE			(ATTRIB_READ_ONLY|ATTRIB_WRITE_ONLY)

#define ATTRIB_READ_WRITE_SHADOW		(ATTRIB_READ_ONLY|ATTRIB_WRITE_ONLY|ATTRIB_SHADOW)
#define ADDRESS(attrib,page,addr)		(REG_MAP | (attrib<<12|page<<8|addr))

#define GET_PAGE(a)			((unsigned char)(((a)>>8)&0x0f))
#define GET_ADDRESS(a)			((unsigned char)((a)&0xff))

#define SET_PAGE(a)			((unsigned char)((a)&0x0f)<<8)

#define CMD_FF				ADDRESS(ATTRIB_READ_WRITE,PAGE1,0xFF)
#define CMD_FE				ADDRESS(ATTRIB_READ_WRITE,PAGE1,0xFE)
#define CMD_FD				ADDRESS(ATTRIB_READ_WRITE,PAGE1,0xFD)

/* <---------------------------- <PAGE 0 Regsiters> -------------------------------> */
//General Regsiters
#define CAM_POWER			ADDRESS(ATTRIB_READ_WRITE,PAGE0,0x00)
#define CAM_STATUS_L			ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x01)
#define CAM_STATUS_H			ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x02)
#define CAM_APP_VER_L			ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x03)
#define CAM_APP_VER_H			ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x04)
#define CAM_HW_PLATFORM_L		ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x05)
#define CAM_HW_PLATFORM_H		ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x06)
#define CAM_HW_VERSION_L		ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x07)
#define CAM_HW_VERSION_H		ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x08)
#define CAM_VERSION_L			ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x09)
#define CAM_VERSION_H			ADDRESS(ATTRIB_READ_ONLY,PAGE0,0x0A)
#define CAM_REG_STATE			ADDRESS(ATTRIB_READ_WRITE,PAGE0,0x0B)


//Sensor Control

#define CAM_SENSOR_SELECT		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE0,0x01)

#define CAM_IMG_CROP_HEIGHT_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE0,0x26)
#define CAM_IMG_CROP_HEIGHT_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE0,0x27)
#define CAM_IMG_CROP_WIDTH_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE0,0x28)
#define CAM_IMG_CROP_WIDTH_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE0,0x29)
#define CAM_PMA				ADDRESS(ATTRIB_READ_WRITE,PAGE0,0x12)
/* <---------------------------- <END OF PAGE 0 Regsiters> -------------------------------> */

/* <---------------------------- <PAGE 1 Regsiters> -------------------------------> */
#define CAM_SENSOR_WIDTH_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x00)
#define CAM_SENSOR_WIDTH_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x01)
#define CAM_SENSOR_HEIGHT_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x02)
#define CAM_SENSOR_HEIGHT_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x03)

#define CAM_VIEWAREA_WIDTH_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x04)
#define CAM_VIEWAREA_WIDTH_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x05)
#define CAM_VIEWAREA_HEIGHT_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x06)
#define CAM_VIEWAREA_HEIGHT_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x07)
#define CAM_SENSOR_FORMAT		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x08)
//Preview  resolution and Data Format
#define CAM_PRV_WIDTH_L			ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x10)
#define CAM_PRV_WIDTH_H			ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x11)
#define CAM_PRV_HEIGHT_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x12)
#define CAM_PRV_HEIGHT_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x13)
#define CAM_PRV_FORMAT_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x14)
#define CAM_PRV_FORMAT_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x15)

#define CAM_IMG_CAP_WIDTH_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x20)
#define CAM_IMG_CAP_WIDTH_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x21)
#define CAM_IMG_CAP_HEIGHT_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x22)
#define CAM_IMG_CAP_HEIGHT_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x23)
#define CAM_IMG_CAP_FORMAT_L		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x24)
#define CAM_IMG_CAP_FORMAT_H		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x25)
#define CAM_IMG_CAP_SHOTS		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x26)
#define CAM_IMG_CAP_DELAY_FRAME		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x27)
#define CAM_IMG_CAP_PERIOD		ADDRESS(ATTRIB_READ_WRITE_SHADOW,PAGE1,0x29)

#define CAM_IMG_CAP_TRIGGER		ADDRESS(ATTRIB_READ_WRITE,PAGE1,0x3F)

#define CURRENT_PAGE			ADDRESS(ATTRIB_WRITE_ONLY,PAGE0,0xFF)
#define PAGE_SWITCH			ADDRESS(ATTRIB_WRITE_ONLY,PAGE0,0xFF)

/* <---------------------------- <END OF PAGE 1 Regsiters> -------------------------------> */

/* <---------------------------- <PAGE 6 Regsiters> -------------------------------> */
//Native Pass through Access
#define CAM_NATIVE_ADDR			ADDRESS(ATTRIB_READ_WRITE,PAGEA,0x00)
#define CAM_NATIVE_DATA_WRITE		ADDRESS(ATTRIB_READ_WRITE,PAGEA,0x04)
#define CAM_NATIVE_DATA_READ		ADDRESS(ATTRIB_READ_WRITE,PAGEA,0x08)
#define CAM_NATIVE_AUTO_INCREMENT	ADDRESS(ATTRIB_READ_WRITE,PAGEA,0x0C)
//Sensor pass through Access
#define CAM_PASST_SLAVE_ADDR		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x10)
#define CAM_PASST_FORMAT		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x11)
#define CAM_PASST_REG_ADDR_L		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x12)
#define CAM_PASST_REG_ADDR_H		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x13)
#define CAM_PASST_REG_DATA_L		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x14)
#define CAM_PASST_REG_DATA_H		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x15)
#define CAM_PASST_STATUS		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x16)
#define CAM_PASST_AI			ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x17)
#define CAM_PASST_BUS_SPEED		ADDRESS(ATTRIB_READ_WRITE,PAGE6,0x18)
#define CAM_PASST_FORMAT_ADDR_WIDTH_SHIFT	0
#define CAM_PASST_FORMAT_DATA_WIDTH_SHIFT	4
#define CAM_PASST_FORMAT_MASK			0x3

/* <---------------------------- <END OF PAGE 6 Regsiters> -------------------------------> */


/* <---------------------------- <PAGE 0xB Regsiters> -------------------------------> */
//Sensor pass through Access
#define CAM_CCDC_SYNCMODE		ADDRESS(ATTRIB_READ_WRITE,PAGEC,0x80)
/* <---------------------------- <END OF PAGE 0xB Regsiters> -------------------------------> */

/* <---------------------------- <PAGE 0xC Regsiters> -------------------------------> */
//Sensor pass through Access
#define CAM_ISPIF_CTRL			ADDRESS(ATTRIB_READ_WRITE,PAGEE,0x80)

/* <---------------------------- <END OF PAGE 0xC Regsiters> -------------------------------> */


/* Page 1 Register field descriptions */
#define CAM_PRV_FORMAT_L_PRV_FORMAT		(1 << 1)
#define CAM_PRV_FORMAT_L_PRV_YUV_FORMAT_MASK	(0xC)
#define CAM_PRV_FORMAT_L_PRV_YUV_FORMAT_SHIFT	(2)

#define CAM_SENSOR_FORMAT_SENSOR_FORMAT_MASK	(0x70)
#define CAM_SENSOR_FORMAT_SENSOR_FORMAT_SHIFT	(4)

#define CAM_SENSOR_FORMAT_SENSOR_INTERFACE_MASK		(0xC)
#define CAM_SENSOR_FORMAT_SENSOR_INTERFACE_SHIFT	(2)

/* Page 0x0C Register Field Descriptions */
#define CAM_CCDC_SYNCMODE_MASK				0x3FFF
#define CAM_CCDC_SYNCMODE_VDHDOUT			(1 << 0)
#define CAM_CCDC_SYNCMODE_FLDOUT			(1 << 1)
#define CAM_CCDC_SYNCMODE_VDPOL				(1 << 2)
#define CAM_CCDC_SYNCMODE_HDPOL				(1 << 3)
#define CAM_CCDC_SYNCMODE_FLDPOL			(1 << 4)
#define CAM_CCDC_SYNCMODE_EXWEN				(1 << 5)
#define CAM_CCDC_SYNCMODE_DATAPOL			(1 << 6)
#define CAM_CCDC_SYNCMODE_FLDMODE			(1 << 7)
#define CAM_CCDC_SYNCMODE_DATSIZ_MASK			(0x7)
#define CAM_CCDC_SYNCMODE_DATSIZ_SHIFT			(0x8)
#define CAM_CCDC_SYNCMODE_PACK8				(1 << 11)
#define CAM_CCDC_SYNCMODE_INPMOD_MASK			(3)
#define CAM_CCDC_SYNCMODE_INPMOD_SHIFT			(12)

/* Page 0xE Register field descriptions */
#define CAM_ISPIF_CTRL_MASK				0xDF
#define CAM_ISPIF_CTRL_PAR_SER_CLK_SEL_MASK		(0x3)
#define CAM_ISPIF_CTRL_PAR_SER_CLK_SEL_SHIFT		(0x0)
#define CAM_ISPIF_CTRL_PAR_BRIDGE_MASK			(0x3)
#define CAM_ISPIF_CTRL_PAR_BRIDGE_SHIFT			(0x2)
#define CAM_ISPIF_CTRL_PAR_CLK_POL			(1 << 4)
#define CAM_ISPIF_CTRL_DATALANE_SHIFT_MASK		(0x3)
#define CAM_ISPIF_CTRL_DATALANE_SHIFT_SHIFT		(0x6)

int isc_open(void);
int isc_regtrans( unsigned char*data,unsigned char ndata, unsigned char attr);
int isc_close(void);

#endif	/*__ISC_CMD_H__*/

