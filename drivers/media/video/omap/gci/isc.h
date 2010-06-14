/*
 * drivers/media/video/omap/gci/isc.h
 *
 * Header file for ISC module in
 * TI's OMAP3430 Camera ISP
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
 *
 */

#ifndef ISC_H
#define ISC_H

/* Macros to be used in isc_sensor_info-interface */
/* TBD see if it can be in enum */
#define SENSOR_PARLL	0
#define SENSOR_CSI1	1
#define SENSOR_CSI2	2
/* Macros to be used in isc_sensor_info-dataformat */
/* TBD see if it can be in enum */
#define SENSOR_BAYER10 		0
#define SENSOR_BAYER9		1
#define SENSOR_BAYER8		2
#define SENSOR_RGB565		3
#define SENSOR_YUV422		4


/* Structure containing the sensor type information
 * to be shared with camera configuration module.
 */
struct isc_sensor_info{
	u8 interface; /* PARALLEL or CSI1 or CSI2 */
	u8 dataformat; /* Non Raw or Raw */
	u8 datasize; /* 10 or 11 or 12 bits */
};

/* Structure containing the input/output sizes 
 * of each module in the camera pipeline.  These sizes are
 * prepared by the camera configuration module
 * and is stored by ISC and used whenever required.
 */
struct isc_sizes{
	u32 ccdc_inp_w;
	u32 ccdc_inp_h;
	u32 ccdc_out_w;
	u32 ccdc_out_h;
	u32 prev_inp_w;
	u32 prev_inp_h;
	u32 prev_out_w;
	u32 prev_out_h;
	u32 resz_inp_w;
	u32 resz_inp_h;
	u32 resz_out_w;
	u32 resz_out_h;
};

struct captureparams{
	u8 cap_shots;
	u8 cap_delay_1stshot;
	u8 cap_delay_intershot;
};

#endif //#ifndef ISC_H

