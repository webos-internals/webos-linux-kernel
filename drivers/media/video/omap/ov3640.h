/*
 * drivers/media/video/omap/mt9p012.h
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
 * Copyright (C) 2008 Texas Instruments.
 *
 * ov3640 Sensor driver for OMAP camera sensor interface 
 *
 */

#ifndef OV3640_H
#define OV3640_H

#define OV3640_PIDL 0x300B
#define OV3640_PIDH 0x300A

#define OV3640_PIDH_MAGIC	0x36	/* high byte of product ID */
#define OV3640_PIDL_MAGIC	0x41	/* low byte of product ID should be 0x40 */



/* define a structure for ov3640 register initialization values */
struct ov3640_reg {
	unsigned int reg;
	unsigned char val;
};

enum image_size { XGA, QXGA };
enum pixel_format { YUV, RGB565, RGB555 };

#define NUM_IMAGE_SIZES 2
#define NUM_PIXEL_FORMATS 3

struct capture_size {
	unsigned long width;
	unsigned long height;
};

/* Array of image sizes supported by OV3640.  These must be ordered from 
 * smallest image size to largest.
 */
const static struct capture_size ov3640_sizes[] = {
	{ 1024, 768 },	/* XGA */
	{ 2048, 1536 },	/* QXGA */
};



#endif 
