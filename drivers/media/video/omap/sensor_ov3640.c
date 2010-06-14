/* 
 * drivers/media/video/omap/sensor_ov3640.c
 *
 * Copyright (C) 2008 Texas Instruments.
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
 * ov3640 Sensor driver for OMAP camera sensor interface
 *
 * January 2008 - First version.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <asm/arch/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/twl4030.h>


#include "sensor_if.h"
#include "ov3640.h"

#define CAMERA_ov3640
#ifdef CAMERA_ov3640
#define MOD_NAME "OV3640: "

#ifdef OV3640_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ERR MOD_NAME format "\n", ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define I2C_M_WR 0

#define ov3640_I2C_ADDR (0x78>>1)	//0x79 alternate address	

#define CAMERA_RESET_GPIO  	98	// for SDP ES2.0
#define CAMERA_POWER_DWN	55	

#define VAUX_2_8_V		0x09
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00


#define DEBUG_BASE		0x08000000
#define REG_SDP3430_FPGA_GPIO_2 (DEBUG_BASE + 0x0050)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)


#define OV3640_XCLK_HZ		12000000
#define OV3640_MIN_FPS		5
#define OV3640_DEF_FPS		15
#define OV3640_MAX_FPS		30



static struct camera_sensor camera_sensor_if;
static void __iomem *fpga_map_addr;

static void
enable_fpga_vio_1v8(u8 enable)
{
	u16 reg_val;

  	fpga_map_addr = ioremap(DEBUG_BASE, 4096);
	reg_val = readw(fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
	
	/* Ensure that the SPR_GPIO1_3v3 is 0 - powered off.. 1 is on */
	if (reg_val & FPGA_SPR_GPIO1_3v3) {
		reg_val &= ~FPGA_SPR_GPIO1_3v3;
		reg_val |= FPGA_GPIO6_DIR_CTRL; /* output mode */
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
		/* give a few milli sec to settle down
		 * Let the sensor also settle down.. if required.. 
		 */
		if (enable)
			mdelay(10);
	}
	if (enable) {
		reg_val |= FPGA_SPR_GPIO1_3v3 | FPGA_GPIO6_DIR_CTRL;
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
	}
	/* Vrise time for the voltage - should be less than 1 ms */
	mdelay(1);

}

static int
board_init(void)
{
#ifdef CONFIG_TWL4030_CORE
	/* turn on analog power */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX_DEV_GRP_P1,TWL4030_VAUX2_DEV_GRP);
	udelay(100);
#else
#error "no power companion board defined!"
#endif

	/* Request and configure gpio pins */
	if (omap_request_gpio(CAMERA_RESET_GPIO) != 0) {
		printk("Could not request GPIO %d",
			CAMERA_RESET_GPIO);
		return -EIO;
	}
		
	if (omap_request_gpio(CAMERA_POWER_DWN) != 0) {
		printk("Could not request GPIO %d",
			CAMERA_POWER_DWN);
		return -EIO;
	}

	/* set to output mode */
	omap_set_gpio_direction(CAMERA_RESET_GPIO, GPIO_DIR_OUTPUT);
	omap_set_gpio_direction(CAMERA_POWER_DWN, GPIO_DIR_OUTPUT);
	
	/* Turn ON Omnivision sensor */
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1);
	omap_set_gpio_dataout(CAMERA_POWER_DWN, 0);
	udelay(100);
	
	/* RESET Omnivision sensor */
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 0);
	udelay(100);
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1);
	
	/* Wait 10 ms before turn ON AD5820 */
	mdelay(10);
	
	enable_fpga_vio_1v8(1);
	udelay(100);
	
	return 0;
}

static void
board_cleanup(void)
{
#ifdef CONFIG_TWL4030_CORE
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX_DEV_GRP_NONE,TWL4030_VAUX2_DEV_GRP);
#else
#error "no power companion board defined!"
#endif
	enable_fpga_vio_1v8(0);
	omap_free_gpio(CAMERA_RESET_GPIO);
        iounmap(fpga_map_addr);
	omap_free_gpio(CAMERA_POWER_DWN);
}

struct ov3640_sensor {
	/* I2C parameters */
	struct i2c_client client;
	struct i2c_driver driver;
	int ver; /* ov3640 version */
	int isize; 
	unsigned long width;
	unsigned long height;
	unsigned long vsize;
	unsigned long hsize;
	struct v4l2_rect crop_rect;

};

static struct ov3640_sensor ov3640;

/* list of image formats supported by ov3640 sensor */
const static struct v4l2_fmtdesc ov3640_formats[] = {
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * We interpret RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  r4 r3 r2 r1 r0 g5 g4 g3
		 */
		.description	= "RGB565, le",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
	},{
		/* Note:  V4L2 defines RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	b4 b3 b2 b1 b0 g5 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	r4 r3 r2 r1 r0 g5 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB565, be",
		.pixelformat	= V4L2_PIX_FMT_RGB565X,
	},
	{
		.description	= "YUYV (YUV 4:2:2), packed",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},{
		.description	= "UYVY, packed",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	},
	{
		/* Note:  V4L2 defines RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  x  b4 b3 b2 b1 b0 g4 g3
		 *
		 * We interpret RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  x  r4 r3 r2 r1 r0 g4 g3
		 */
		.description	= "RGB555, le",
		.pixelformat	= V4L2_PIX_FMT_RGB555,
	},{
		/* Note:  V4L2 defines RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  b4 b3 b2 b1 b0 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  r4 r3 r2 r1 r0 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB555, be",
		.pixelformat	= V4L2_PIX_FMT_RGB555X,
	}
};

#define NUM_CAPTURE_FORMATS (sizeof(ov3640_formats)/sizeof(ov3640_formats[0]))


/* register initialization tables for ov3640 */
#define ov3640_REG_TERM 0xFFFF	/* terminating list entry for reg */
#define ov3640_VAL_TERM 0xFF	/* terminating list entry for val */


const static struct ov3640_reg ov3640_common[2][100] = {
	/* XGA_Default settings */
	{{ 0x304d, 0x45}, { 0x3087, 0x16}, { 0x30aa, 0x45},
	{ 0x30b0, 0xff}, { 0x30b1, 0xff}, { 0x30b2, 0x10}, 
	{ 0x300e, 0x32}, { 0x300f, 0x21}, { 0x3010, 0x20},   
	{ 0x3011, 0x00}, { 0x304c, 0x82}, { 0x3018, 0x38}, 
	{ 0x3019, 0x30}, { 0x301a, 0x61}, { 0x307d, 0x00}, 
	{ 0x3087, 0x02}, { 0x3082, 0x20}, { 0x3015, 0x12}, 
	{ 0x3014, 0x04}, { 0x3013, 0xf7}, { 0x303c, 0x08}, 
	{ 0x303d, 0x18}, { 0x303e, 0x06}, { 0x303f, 0x0c},
	{ 0x3030, 0x62}, { 0x3031, 0x26}, { 0x3032, 0xe6},
	{ 0x3033, 0x6e}, { 0x3034, 0xea}, { 0x3035, 0xae},
	{ 0x3036, 0xa6}, { 0x3037, 0x6a}, { 0x3104, 0x02}, 
	{ 0x3105, 0xfd}, { 0x3106, 0x00}, { 0x3107, 0xff}, 
	{ 0x3300, 0x13}, { 0x3301, 0xde}, { 0x3302, 0xef}, 
	{ 0x3316, 0xff}, { 0x3317, 0x00}, { 0x3308, 0xa5}, 
	{ 0x3312, 0x26}, { 0x3314, 0x42}, { 0x3313, 0x2b}, 
	{ 0x3315, 0x42}, { 0x3310, 0xd0}, { 0x3311, 0xbd}, 
	{ 0x330c, 0x18}, { 0x330d, 0x18}, { 0x330e, 0x56}, 
	{ 0x330f, 0x5c}, { 0x330b, 0x1c}, { 0x3306, 0x5c}, 
	{ 0x3307, 0x11}, { 0x336a, 0x52}, { 0x3370, 0x46}, 
	{ 0x3376, 0x38}, { 0x30b8, 0x20}, { 0x30b9, 0x17}, 
	{ 0x30ba, 0x04}, { 0x30bb, 0x08}, { 0x3507, 0x06}, 
	{ 0x350a, 0x4f}, { 0x3100, 0x02}, { 0x3301, 0xde}, 
	{ 0x3304, 0xfc}, { 0x3400, 0x00}, { 0x3404, 0x00}, 
	{ 0x3012, 0x10}, { 0x3023, 0x06}, { 0x3026, 0x03}, 
	{ 0x3027, 0x04}, { 0x302a, 0x03}, { 0x302b, 0x10}, 
	{ 0x3075, 0x24}, { 0x300d, 0x01}, { 0x30d7, 0x90}, 
	{ 0x3069, 0x04}, { 0x303e, 0x00}, { 0x303f, 0xc0}, 
	{ 0x335f, 0x34}, { 0x3360, 0x0c}, { 0x3361, 0x04}, 
	{ 0x3362, 0x34}, { 0x3363, 0x08}, { 0x3364, 0x04},
	{ 0x3403, 0x42}, { 0x3088, 0x04}, { 0x3089, 0x00}, 
	{ 0x308a, 0x03}, { 0x308b, 0x00}, { 0x308d, 0x04}, 
	{ 0x3086, 0x03}, { 0x3086, 0x00}, { 0x3600, 0xc4}, 
	{ 0x332b, 0x00}, { 0x332d, 0x60}, { 0x332f, 0x03}, 
	{ ov3640_REG_TERM, ov3640_VAL_TERM }},
	/* QXGA Default settings */
	{{ 0x304d, 0x45}, { 0x3087, 0x16}, { 0x30aa, 0x45},
	{ 0x30b0, 0xff}, { 0x30b1, 0xff}, { 0x30b2, 0x10}, 
	{ 0x300e, 0x39}, { 0x300f, 0x21}, { 0x3010, 0x20},   
	{ 0x3011, 0x00}, { 0x304c, 0x81}, { 0x3018, 0x38}, 
	{ 0x3019, 0x30}, { 0x301a, 0x61}, { 0x307d, 0x00}, 
	{ 0x3087, 0x02}, { 0x3082, 0x20}, { 0x3015, 0x12}, 
	{ 0x3014, 0x04}, { 0x3013, 0xf7}, { 0x303c, 0x08}, 
	{ 0x303d, 0x18}, { 0x303e, 0x06}, { 0x303f, 0x0c},
	{ 0x3030, 0x62}, { 0x3031, 0x26}, { 0x3032, 0xe6},
	{ 0x3033, 0x6e}, { 0x3034, 0xea}, { 0x3035, 0xae},
	{ 0x3036, 0xa6}, { 0x3037, 0x6a}, { 0x3104, 0x02}, 
	{ 0x3105, 0xfd}, { 0x3106, 0x00}, { 0x3107, 0xff}, 
	{ 0x3300, 0x13}, { 0x3301, 0xde}, { 0x3302, 0xef}, 
	{ 0x3316, 0xff}, { 0x3317, 0x00}, { 0x3308, 0xa5}, 
	{ 0x3312, 0x26}, { 0x3314, 0x42}, { 0x3313, 0x2b}, 
	{ 0x3315, 0x42}, { 0x3310, 0xd0}, { 0x3311, 0xbd}, 
	{ 0x330c, 0x18}, { 0x330d, 0x18}, { 0x330e, 0x56}, 
	{ 0x330f, 0x5c}, { 0x330b, 0x1c}, { 0x3306, 0x5c}, 
	{ 0x3307, 0x11}, { 0x336a, 0x52}, { 0x3370, 0x46}, 
	{ 0x3376, 0x38}, { 0x30b8, 0x20}, { 0x30b9, 0x17}, 
	{ 0x30ba, 0x04}, { 0x30bb, 0x08}, { 0x3507, 0x06}, 
	{ 0x350a, 0x4f}, { 0x3100, 0x02}, { 0x3301, 0xde}, 
	{ 0x3304, 0xfc}, { 0x3400, 0x00}, { 0x3404, 0x00}, 
	{ 0x30d7, 0x10}, { 0x3600, 0xc4}, { 0x3088, 0x08},
	{ 0x3089, 0x00}, { 0x308a, 0x06}, { 0x308b, 0x00},
	{ 0x308d, 0x04}, { 0x3086, 0x03}, { 0x3086, 0x03},
	{ 0x3086, 0x00}, { 0x332b, 0x00}, { 0x332d, 0x60},
	{ 0x332f, 0x03}, { ov3640_REG_TERM, ov3640_VAL_TERM }},
};

/* Brightness Settings - 7 levels */
const static struct ov3640_reg brightness[7][5] = {
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x3354, 0x09 },
	  { 0x335e, 0x30 },{ ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x3354, 0x09 },
	  { 0x335e, 0x20 },{ ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x3354, 0x09 },
	  { 0x335e, 0x10 },{ ov3640_REG_TERM, ov3640_VAL_TERM }},	
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x3354, 0x01 },
	  { 0x335e, 0x00 },{ ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x3354, 0x01 },
	  { 0x335e, 0x10 },{ ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x3354, 0x01 },
	  { 0x335e, 0x20 },{ ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x3354, 0x01 },
	  { 0x335e, 0x30 },{ ov3640_REG_TERM, ov3640_VAL_TERM }},
};

/* Contrast Settings - 7 levels */
const static struct ov3640_reg contrast[7][5] = {
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x335c, 0x14 }, 
	 { 0x335d, 0x14 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x335c, 0x18 }, 
	 { 0x335d, 0x18 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x335c, 0x1c }, 
	 { 0x335d, 0x1c }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x335c, 0x20 }, 
	 { 0x335d, 0x20 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x335c, 0x24 }, 
	 { 0x335d, 0x24 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x335c, 0x28 }, 
	 { 0x335d, 0x28 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x04 }, { 0x335c, 0x2c }, 
	 { 0x335d, 0x2c }, { ov3640_REG_TERM, ov3640_VAL_TERM }},	 
};

/* Saturation Settings - 5 levels */
const static struct ov3640_reg saturation[5][5] = {
	{{ 0x3302, 0xef }, { 0x3355, 0x06 }, { 0x3358, 0x20 }, 
	 { 0x3359, 0x20 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x06 }, { 0x3358, 0x30 }, 
	 { 0x3359, 0x30 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x06 }, { 0x3358, 0x40 }, 
	 { 0x3359, 0x40 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x06 }, { 0x3358, 0x50 }, 
	 { 0x3359, 0x50 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x06 }, { 0x3358, 0x70 }, 
	 { 0x3359, 0x70 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},	 
};

/* Color Settings - 3 colors */
const static struct ov3640_reg colors[3][5] = {
	{{ 0x3302, 0xef }, { 0x3355, 0x00 }, { 0x335a, 0x80 }, 
	 { 0x335b, 0x80 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x18 }, { 0x335a, 0x40 }, 
	 { 0x335b, 0xa6 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
	{{ 0x3302, 0xef }, { 0x3355, 0x18 }, { 0x335a, 0x80 }, 
	 { 0x335b, 0x80 }, { ov3640_REG_TERM, ov3640_VAL_TERM }},
};

/* ov3640 register configuration for all combinations of pixel format and 
 * image size
 */
	
const static struct ov3640_reg qxga_yuv[] = {
	{ 0x3100, 0x02 }, { 0x3301, 0xde }, { 0x3304, 0xfc },	
	{ 0x3400, 0x00 }, { 0x3404, 0x00 }, { 0x302a, 0x06 }, 
	{ 0x302b, 0x20 }, { 0x3507, 0x06 }, { 0x350a, 0x4f }, 
	{ 0x3600, 0xC4 }, { ov3640_REG_TERM, ov3640_VAL_TERM }
};

const static struct ov3640_reg qxga_565[] = {
	{ 0x3100, 0x02 }, { 0x3301, 0xde }, { 0x3304, 0xfc },	
	{ 0x3400, 0x01 }, { 0x3404, 0x11 }, { 0x302a, 0x06 }, 
	{ 0x302b, 0x20 }, { 0x3507, 0x06 }, { 0x350a, 0x4f }, 
	{ 0x3600, 0xC4 }, { ov3640_REG_TERM, ov3640_VAL_TERM }
};
	
const static struct ov3640_reg qxga_555[] = {
	{ 0x3100, 0x02 }, { 0x3301, 0xde }, { 0x3304, 0xfc },	
	{ 0x3400, 0x01 }, { 0x3404, 0x13 }, { 0x302a, 0x06 }, 
	{ 0x302b, 0x20 }, { 0x3507, 0x06 }, { 0x350a, 0x4f }, 
	{ 0x3600, 0xC4 }, { ov3640_REG_TERM, ov3640_VAL_TERM }
};
	
const static struct ov3640_reg xga_yuv[] = {
	{ 0x3100, 0x02 }, { 0x3301, 0xde }, { 0x3304, 0xfc },	
	{ 0x3400, 0x10 }, { 0x3404, 0x00 }, { 0x302a, 0x03 }, 
	{ 0x302b, 0x10 }, { 0x3507, 0x06 }, { 0x350a, 0x4f }, 
	{ 0x3600, 0xC4 }, { ov3640_REG_TERM, ov3640_VAL_TERM }
};
	
const static struct ov3640_reg xga_565[] = {
	{ 0x3100, 0x02 }, { 0x3301, 0xde }, { 0x3304, 0xfc },	
	{ 0x3400, 0x01 }, { 0x3404, 0x11 }, { 0x302a, 0x03 }, 
	{ 0x302b, 0x10 }, { 0x3507, 0x06 }, { 0x350a, 0x4f }, 
	{ 0x3600, 0xC4 }, { ov3640_REG_TERM, ov3640_VAL_TERM }
};
	
const static struct ov3640_reg xga_555[] = {
	{ 0x3100, 0x02 }, { 0x3301, 0xde }, { 0x3304, 0xfc },	
	{ 0x3400, 0x01 }, { 0x3404, 0x13 }, { 0x302a, 0x03 }, 
	{ 0x302b, 0x10 }, { 0x3507, 0x06 }, { 0x350a, 0x4f }, 
	{ 0x3600, 0xC4 }, { ov3640_REG_TERM, ov3640_VAL_TERM }
};

#define DEF_BRIGHTNESS    0
#define DEF_CONTRAST      0
#define DEF_SATURATION    0
#define PREV_DEFAULT_COLOR 0 
		 
/*  Video controls  */
static struct vcontrol {
        struct v4l2_queryctrl qc;
        int current_value;
        u8 reg;
        u8 mask;
        u8 start_bit;
} control[] = {
        { { V4L2_CID_BRIGHTNESS, V4L2_CTRL_TYPE_INTEGER, "Brightness", -3, 3, 1,
            DEF_BRIGHTNESS },
          0, ov3640_VAL_TERM, 0, 0 },
        { { V4L2_CID_CONTRAST, V4L2_CTRL_TYPE_INTEGER, "Contrast", -3, 3, 1,
            DEF_CONTRAST },
          0, ov3640_VAL_TERM, 0, 0 },
	{ { V4L2_CID_SATURATION, V4L2_CTRL_TYPE_INTEGER, "Saturation", -2, 2, 1,
            DEF_SATURATION  },
          0, ov3640_VAL_TERM, 0, 0 },
	{ { V4L2_CID_PRIVATE_BASE, V4L2_CTRL_TYPE_INTEGER, "Color Effects", 0, 2, 1,
            PREV_DEFAULT_COLOR  },
          0, ov3640_VAL_TERM, 0, 0 },
};

#define NUM_CONTROLS (sizeof(control)/sizeof(control[0]))

const static struct ov3640_reg *
	ov3640_reg_init[NUM_PIXEL_FORMATS][NUM_IMAGE_SIZES] =
{
 { xga_yuv, qxga_yuv },
 { xga_565, qxga_565 },
 { xga_555, qxga_555 }
};


/* 
 * Read a value from a register in an ov3640 sensor device.  The value is 
 * returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int 
ov3640_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;
	
	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);
	udelay(50);
	if (err >= 0) {
		msg->flags = I2C_M_RD;
		msg->len = 1;
		err = i2c_transfer(client->adapter, msg, 1);
		udelay(50);
	}
	if (err >= 0) {
		*val = *data;
		return 0;
	}
	return err;
}

/* Write a value to a register in an ov3640 sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int 
ov3640_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retries = 0;
	
	if (!client->adapter)
		return -ENODEV;

retry:	
	msg->addr = client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 3;
	msg->buf = data;
	
	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	data[2] = val;
	
	err = i2c_transfer(client->adapter, msg, 1);
	udelay(50);
	if (err >= 0)
		return 0;
	
	if (retries <= 5) {
		retries++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto retry;
	}
	return err;
	
}

static int 
ov3640_write_reg_mask(struct i2c_client *client, u16 reg, u8 *val, u8 mask)
{
	u8 oldval, newval;
	int rc;

	if (mask == 0xff) {
		newval = *val;
	} else {
		/* need to do read - modify - write */
		if ((rc = ov3640_read_reg(client, reg, &oldval)))
			return rc;
		oldval &= (~mask);              /* Clear the masked bits */
		*val &= mask;                  /* Enforce mask on value */
		newval = oldval | *val;        /* Set the desired bits */
	}

	/* write the new value to the register */
	if ((rc = ov3640_write_reg(client, reg, newval)))
		return rc;

	if ((rc = ov3640_read_reg(client, reg, &newval)))
		return rc;

	*val = newval & mask;
	return 0;
}

static int 
ov3640_read_reg_mask(struct i2c_client *client, u8 reg, u8 *val, u8 mask)
{
	int rc;

	if ((rc = ov3640_read_reg(client, reg, val)))
		return rc;
	(*val) &= mask;

	return 0;
}

/* Initialize a list of ov3640 registers.
 * The list of registers is terminated by the pair of values 
 * { ov3640_REG_TERM, ov3640_VAL_TERM }.
 * Returns zero if successful, or non-zero otherwise.
 */
static int 
ov3640_write_regs(struct i2c_client *client, const struct ov3640_reg reglist[])
{
	int err;
	const struct ov3640_reg *next = reglist;
	
	while (!((next->reg == ov3640_REG_TERM) 
		&& (next->val == ov3640_VAL_TERM)))
	{
		err = ov3640_write_reg(client, next->reg, next->val);
		udelay(100);
		if (err)
			return err;
		next++;
	}
	return 0;
}


/* Returns the index of the requested ID from the control structure array */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = NUM_CONTROLS - 1; i >= 0; i--)
		if (control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/* Set the camera brightness level. Return 0 on success. */
static int
ov3640_set_brightness(struct i2c_client *client, int level)
{
	/* the caller passes us a valid level */
	return ov3640_write_regs(client, brightness[level + 3]);

}

/* Set the camera contrast level. Return 0 on success. */
static int
ov3640_set_contrast(struct i2c_client *client, int level)
{
	/* the caller passes us a valid level */
	return ov3640_write_regs(client, contrast[level + 3]);
}

static int
ov3640_set_saturation(struct i2c_client *client, int level)
{
	/* the caller passes us a valid level */
	return ov3640_write_regs(client, saturation[level + 2]);

}

static int
ov3640_set_colors(struct i2c_client *client, int level)
{
	/* the caller passes us a valid level */
	return ov3640_write_regs(client, colors[level]);

}
/* Calculate the internal clock divisor (value of the CLKRC register) of the 
 * ov3640 given the image size, the frequency (in Hz) of its XCLK input and a 
 * desired frame period (in seconds).  The frame period 'fper' is expressed as 
 * a fraction.  The frame period is an input/output parameter.
 * Returns the value of the ov3640 CLKRC register that will yield the frame 
 * period returned in 'fper' at the specified xclk frequency.  The 
 * returned period will be as close to the requested period as possible.
 */
static unsigned char
ov3640_clkrc(enum image_size isize, unsigned long xclk, struct v4l2_fract *fper)
{
	const static unsigned long clks_per_frame[] = 
		{ 1862784, 7451136  };
	/*           XGA     QXGA  	*/
	unsigned long ov_fps, reg;
	/* Numbers from SW app notes */
	const int divisor_max = 64; 
	int div = 2000000;

	if (fper->numerator > 0)
		ov_fps = fper->denominator/fper->numerator;
	else
		ov_fps = OV3640_DEF_FPS;
	
	if (ov_fps > OV3640_MAX_FPS)
		ov_fps = OV3640_MAX_FPS;
	if (ov_fps < OV3640_MIN_FPS)
		ov_fps = OV3640_MIN_FPS;
	
	if ((ov_fps > OV3640_DEF_FPS)&& (isize == QXGA))
		ov_fps = OV3640_DEF_FPS;
		
	fper->numerator = 1;
	fper->denominator = ov_fps;
	
	/* We calculate the divisor based on SW app notes formula */
	div  *= (1 + (isize * 3));
	reg  = divisor_max - ov_fps * clks_per_frame[isize] / div;

	return (int) reg;
}

/* Find the best match for a requested image capture size.  The best match 
 * is chosen as the nearest match that has the same number or fewer pixels 
 * as the requested size, or the smallest image size if the requested size 
 * has fewer pixels than the smallest image.
 */
static enum image_size
ov3640_find_size(unsigned int width, unsigned int height)
{
	if ((width>ov3640_sizes[XGA].width)||(height>ov3640_sizes[XGA].height))
		return QXGA;
	return XGA;
}

/* Configure the ov3640 for a specified image size, pixel format, and frame 
 * period.  xclk is the frequency (in Hz) of the xclk input to the ov3640.  
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int
ov3640_configure(struct ov3640_sensor *sensor, 
	struct v4l2_pix_format *pix, 
	enum pixel_format pfmt,
	unsigned long xclk,
	struct v4l2_fract *fper)
{	
	enum image_size isize;
	int err;
	unsigned char clkrc;
	unsigned char hsize_l, hsize_h;
	unsigned char vsize_l, vsize_h;
	int vsize, hsize, height_l, height_h, width_l, width_h;
	int ratio;
	struct i2c_client *client = &sensor->client;

	
	/* Check if the image size anf fps can be achived (sizes greater
	 * than XGA have a max frame rate of 15 FPS)
	 */
	if ((fper->denominator/fper->numerator)<= OV3640_DEF_FPS) isize = QXGA;
	else if (ov3640_find_size(pix->width, pix->height) == XGA)
		isize = XGA;
	else return -EINVAL;
	
		
	/* Reset */
	ov3640_write_reg (client,0x3012, 0x80);
	mdelay(5);
	
	/* Common registers */	
	err = ov3640_write_regs(client, ov3640_common[isize]);
		
	/* Configure image size and pixel format */	
	err = ov3640_write_regs(client, ov3640_reg_init[pfmt][isize]);
	
	sensor->isize = isize;
	
	/* Scale image if needed*/
	if (((pix->width == ov3640_sizes[QXGA].width )&&
		(pix->height == ov3640_sizes[QXGA].height)&&(isize == QXGA)) ||
	   ((pix->width == ov3640_sizes[XGA].width)&&
		(pix->height == ov3640_sizes[XGA].height)&&(isize == XGA))){
	
	/* if the image size correspond to one of the base image sizes then
	   we don't have to scale */
	   
	
	} else {
		/* Default Ver and Hor sizes for QXGA and XGA*/
		if (isize == QXGA){
			vsize = 0x60c;
			hsize = 0x818;
		} else { 	
			vsize = 0x304;
			hsize = 0x40c;
		}
			
		/* Adjust V and H sizes for image sizes not derived form VGA*/
		ratio = (pix->width * 1000)/pix->height;
		
		if  (((vsize * ratio + 500 )/1000)>hsize) 
			vsize = (hsize * 1000) / ratio ;
				
		else
			hsize = (vsize * ratio + 500)/1000; 
			 
		/* We need even numbers */
		if (vsize & 1)
			vsize--;
		if (hsize & 1)
			hsize--;			 
		
		/* Adjusting numbers to set register correctly */
		hsize_l = (0xFF & hsize);
		hsize_h = (0xF00 & hsize)>> 8;
		vsize_l = (0xFF & vsize);
		vsize_h = (0x700 & vsize)>> 4;
	
		/* Per software app notes we have t add 0x08 and 0x04 in order
		 * to scale correctly 
		 */			 
		width_l = (0xFF & pix->width) + 0x08;
		width_h = (0xF00 & pix->width)>> 8;
		height_l = (0xFF & pix->height) + 0x04;
		height_h = (0x700 & pix->height)>> 4;


		/* Scaling */
		
		err = ov3640_write_reg (client, 0x3302, 0xEF);
		err = ov3640_write_reg (client, 0x335f, (vsize_h |hsize_h));
		err = ov3640_write_reg (client, 0x3360, hsize_l);
		err = ov3640_write_reg (client, 0x3361, vsize_l); 
		err = ov3640_write_reg (client, 0x3362, (height_h | width_h)); 
		err = ov3640_write_reg (client, 0x3363, width_l);
		err = ov3640_write_reg (client, 0x3364, height_l);
		err = ov3640_write_reg (client, 0x3403, 0x42);
		err = ov3640_write_reg (client, 0x3088, width_h);
		err = ov3640_write_reg (client, 0x3089, (width_l  - 0x08));
		err = ov3640_write_reg (client, 0x308a, (height_h >> 4));
		err = ov3640_write_reg (client, 0x308b, (height_l - 0x04));
		
	}
	
	/* Store image size */
	sensor->width = pix->width;
	sensor->height = pix->height;

	/* Calculate PLL divisor for desired framerate */
	clkrc = ov3640_clkrc(isize, xclk, fper);
	
	DPRINTK(" CLKRC: %d \n", clkrc);
	DPRINTK("Frames/second: %d \n", (fper->denominator/fper->numerator));
	
	/* Set frame rate */
	err = ov3640_write_reg(client, 0x300E, clkrc);
	
	mdelay(200);
	return err;
}


/* Detect if an ov3640 is present, returns a negative error number if no 
 * device is detected, or 0 if a device is detected.
 */
static int
ov3640_detect(struct i2c_client *client)
{
	u8 pidh, pidl;

	if (!client)
		return -ENODEV;
 
	if (ov3640_read_reg(client, OV3640_PIDH, &pidh))
		return -ENODEV;
	if (ov3640_read_reg(client, OV3640_PIDL, &pidl))
		return -ENODEV;

	
	if ((pidh != OV3640_PIDH_MAGIC) 
		|| (pidl != OV3640_PIDL_MAGIC))
	{

		return -ENODEV;
	} 
	return 0;

}

/* This function registers an I2C client via i2c_attach_client() for an ov3640 
 * sensor device.  If 'probe' is non-zero, then the I2C client is only 
 * registered if the device can be detected.  If 'probe' is zero, then no 
 * device detection is attempted and the I2C client is always registered.
 * Returns zero if an I2C client is successfully registered, or non-zero 
 * otherwise.
 */
static int 
ov3640_i2c_attach_client(struct i2c_adapter *adap, int addr, int probe)
{
	struct ov3640_sensor *sensor = &ov3640;
	struct i2c_client *client = &sensor->client;
	int err;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	client->addr = addr;
	i2c_set_clientdata(&sensor->client, &ov3640);
	client->driver = &sensor->driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	if (probe) {
		err = ov3640_detect(client);
		if (err < 0) {
			i2c_detach_client(client);
			client->adapter = NULL;
			return err;
		}
	}
	return 0;
}

/* This function is called by i2c_del_adapter() and i2c_del_driver() 
 * if the adapter or driver with which this I2C client is associated is 
 * removed.  This function unregisters the client via i2c_detach_client().
 * Returns zero if the client is successfully detached, or non-zero 
 * otherwise.
 */
static int 
ov3640_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;

	return err;
}

/* This function will be called for each registered I2C bus adapter when our 
 * I2C driver is registered via i2c_add_driver().  It will also be called 
 * whenever a new I2C adapter is registered after our I2C driver is registered.
 * This function probes the specified I2C bus adapter to determine if an 
 * ov3640 sensor device is present.  If a device is detected, an I2C client 
 * is registered for it via ov3640_i2c_attach_client().  Note that we can't use 
 * the standard i2c_probe() function to look for the sensor because the OMAP 
 * I2C controller doesn't support probing.
 * Returns zero if an ov3640 device is detected and an I2C client successfully 
 * registered for it, or non-zero otherwise.
 */
static int 
ov3640_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return ov3640_i2c_attach_client(adap, ov3640_I2C_ADDR, 1);
}

static int
ov3640sensor_power_on(void *priv)
{
	struct ov3640_sensor *sensor = (struct ov3640_sensor *) priv;
	struct i2c_client *client = &sensor->client;

	int err = 0;
	err = ov3640_write_reg ( client, 0x308d, 0x00 );
	err = ov3640_write_reg ( client, 0x30ad, 0x02 );
	err = ov3640_write_reg ( client, 0x308d, 0x00 );
	err = ov3640_write_reg ( client, 0x361e, 0x00 );
	return err;
}

static int
ov3640sensor_power_off(void *priv)
{
	struct ov3640_sensor *sensor = (struct ov3640_sensor *) priv;
	struct i2c_client *client = &sensor->client;

	int err = 0;
	err = ov3640_write_reg( client, 0x361e, 0x00 );
	err = ov3640_write_reg( client, 0x308d, 0x06 );
	err = ov3640_write_reg( client, 0x30ad, 0x82 );
	err = ov3640_write_reg( client, 0x308d, 0x0f );
	return err;
}

static int
ov3640sensor_query_control(struct v4l2_queryctrl *qc, void *priv)
{
	int i;

	i = find_vctrl (qc->id);
	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	if (i < 0)
		return -EINVAL;

	*qc = control[i].qc;
	return 0;
}

static int
ov3640sensor_get_control(struct v4l2_control *vc, void *priv)
{
	struct ov3640_sensor *sensor = (struct ov3640_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	int i, val;
	struct vcontrol * lvc;
	
	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];	

	/* treat synthetic controls differently */
	if (lvc->reg == ov3640_VAL_TERM) {
		switch (lvc->qc.id) {
			case V4L2_CID_BRIGHTNESS:
				vc->value = lvc->current_value;
				break;
			case V4L2_CID_CONTRAST:
				vc->value = lvc->current_value;
				break;
			case V4L2_CID_PRIVATE_BASE:
				vc->value = lvc->current_value;
				break;
			case V4L2_CID_SATURATION:
				vc->value = lvc->current_value;
				break;	
			default:
				return -EINVAL;
		}
	}
	else {
		if (ov3640_read_reg_mask(client, lvc->reg, (u8 *)&val,
					lvc->mask))
			return -EIO;
		vc->value = lvc->current_value = val >> lvc->start_bit;
	}

	return 0;
}

static int
ov3640sensor_set_control(struct v4l2_control *vc, void *priv)
{
	struct ov3640_sensor *sensor = (struct ov3640_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	struct vcontrol *lvc;
	int val = vc->value;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	if (val < lvc->qc.minimum || val > lvc->qc.maximum)
		return -ERANGE;

	/* treat synthetic controls differently */
	if (lvc->reg == ov3640_VAL_TERM) {
		switch (lvc->qc.id) {
			case V4L2_CID_BRIGHTNESS:
				if (ov3640_set_brightness(client, val))
					return -EIO;
				lvc->current_value = val;
				break;
			case V4L2_CID_CONTRAST:
				if (ov3640_set_contrast(client, val))
					return -EIO;
				lvc->current_value = val;
				break;
			case V4L2_CID_SATURATION:
				if (ov3640_set_saturation(client, val))
					return -EIO;
				lvc->current_value = val;
				break;
			case V4L2_CID_PRIVATE_BASE:
				if (ov3640_set_colors(client, val))
					return -EIO;
				lvc->current_value = val;
				break;
			default:
				return -EINVAL;
		}
	}
	else {
		val = val << lvc->start_bit;
		if (ov3640_write_reg_mask(client, lvc->reg, (u8 *)&val,
					 (u8)lvc->mask))
			return -EIO;
		vc->value = lvc->current_value = val>> lvc->start_bit;
	}

	return 0;
}

/* Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This 
 * ioctl is used to negotiate the image capture size and pixel format 
 * without actually making it take effect.
 */
static int
ov3640sensor_try_format(struct v4l2_pix_format *pix, void *priv)
{
	
	int ifmt;

	if (pix->width > ov3640_sizes[QXGA].width) 
		pix->width = ov3640_sizes[QXGA].width;
	if (pix->height > ov3640_sizes[QXGA].height) 
		pix->height = ov3640_sizes[QXGA].height;
	
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ov3640_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ov3640_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width*2;
	pix->sizeimage = pix->bytesperline*pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
		default:
			pix->colorspace = V4L2_COLORSPACE_JPEG;
			break;
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB565X:
		case V4L2_PIX_FMT_RGB555:
		case V4L2_PIX_FMT_RGB555X:
			pix->colorspace = V4L2_COLORSPACE_SRGB;
			break;
	}
	return 0;
}

/* Given the image capture format in pix, the nominal frame period in 
 * timeperframe, calculate the required xclk frequency 
 * The xclk input frequency of the OV3640 can be either 12MHz 
 * or 3MHz
 */
static unsigned long
ov3640sensor_calc_xclk(struct v4l2_pix_format *pix,
			struct v4l2_fract *timeperframe, void *priv)
{
	unsigned long ov_fps;
	int clk_div = OV3640_XCLK_HZ;

	DPRINTK("enter sensor_calc_xclk \n");	
	if ((timeperframe->numerator == 0) 
		|| (timeperframe->denominator == 0))
	{
		timeperframe->numerator = 1;
		timeperframe->denominator = OV3640_DEF_FPS;
	}
	//timeperframe->denominator /= timeperframe->denominator/timeperframe->numerator; 
	timeperframe->denominator /= timeperframe->numerator;
	ov_fps = timeperframe->denominator/timeperframe->numerator; 
	
	if (ov_fps < OV3640_MIN_FPS){
		timeperframe->denominator = OV3640_MIN_FPS;
	} else if (ov_fps > OV3640_MAX_FPS){
		timeperframe->denominator = OV3640_MAX_FPS;
	}
	
	timeperframe->numerator = 1;
	
	if ((timeperframe->denominator)< OV3640_DEF_FPS) 
		clk_div = OV3640_XCLK_HZ/4;
	else if (ov3640_find_size(pix->width, pix->height) == XGA)
		clk_div = OV3640_XCLK_HZ;

	return clk_div ;
}

/* Given a capture format in pix, the frame period in timeperframe, and
 * the xclk frequency, set the capture format of the OV3640 sensor.
 * The actual frame period will be returned in timeperframe.
 */
static int
ov3640sensor_configure(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
	struct ov3640_sensor *sensor = (struct ov3640_sensor *) priv;
  	enum pixel_format pfmt = YUV;

	DPRINTK("enter sensor_configure \n");
	
	switch (pix->pixelformat) {
		case V4L2_PIX_FMT_RGB565:
		case V4L2_PIX_FMT_RGB565X:
			pfmt = RGB565;
			break;
		case V4L2_PIX_FMT_RGB555:
		case V4L2_PIX_FMT_RGB555X:
			pfmt = RGB555;
			break;
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
		default:
			pfmt = YUV;
 	}
	return ov3640_configure(sensor,
				pix,
				pfmt, xclk, timeperframe);
}


/* Prepare for the driver to exit. Balances ov3640sensor_init().
 * This function must de-initialize the sensor and its associated data
 * structures.
 */
static int
ov3640sensor_cleanup(void *priv)
{
	struct ov3640_sensor *sensor = (struct ov3640_sensor *) priv;

	if (sensor) {
		i2c_del_driver(&sensor->driver);
		board_cleanup();
 	}
	return 0;
}

/* Initialize the ov3640 sensor.
 * This routine allocates and initializes the data structure for the sensor, 
 * powers up the sensor, registers the I2C driver, and sets a default image 
 * capture format in pix.  The capture format is not actually programmed 
 * into the ov3640 sensor by this routine.
 * This function must return a non-NULL value to indicate that 
 * initialization is successful.
 */
static void *
ov3640sensor_init(struct v4l2_pix_format *pix, struct v4l2_pix_format *pix2)
{
	struct ov3640_sensor *sensor = &ov3640;
	struct i2c_driver *driver = &sensor->driver;
 	int err;
	/* Power on sensor */
	if (board_init())
		return NULL;
	memset(sensor, 0, sizeof(*sensor));
 	
	driver->driver.name = "ov3640 I2C driver";
	driver->class = I2C_CLASS_HWMON,
	driver->attach_adapter = ov3640_i2c_probe_adapter;
	driver->detach_client = ov3640_i2c_detach_client;

	err = i2c_register_driver(THIS_MODULE, driver);
	if (err) {
		printk(KERN_ERR "Failed to register ov3640 I2C client.\n");
		board_cleanup();
		return NULL;
	}
	if (!sensor->client.adapter) {
		printk(KERN_WARNING 
			"Failed to detect ov3640 sensor chip.\n");
		ov3640sensor_cleanup((void *)sensor);
		return NULL;
	}
	else {
		printk(KERN_INFO 
			"ov3640 sensor chip version 0x%02x detected\n", 
			sensor->ver);
	}

	/* Make the default capture format CIF YUYV*/
	pix->width = 352;
	pix->height = 288;
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	ov3640sensor_try_format(pix, NULL);

	/* Set the default brightness and contrast */
	ov3640_set_brightness(&sensor->client, DEF_BRIGHTNESS);
	ov3640_set_contrast(&sensor->client, DEF_CONTRAST);
	ov3640_set_saturation(&sensor->client, DEF_SATURATION);
	return (void *)sensor;
}

/*
 * Unregister the sensor from the Camera Driver
 */
static void
ov3640sensor_unregister(void)
{
	omap_cam_unregister_sensor(&camera_sensor_if);
}

/*
 * Register the sensor with the Camera Driver
 */
static int __init
ov3640sensor_register(void)
{
	int err;
	
	err = omap_cam_register_sensor(&camera_sensor_if);
	if (err){
		 printk(KERN_WARNING " Failed to register sensor.\n");
		omap_cam_unregister_sensor(&camera_sensor_if);
	}
	return err;
}


static struct camera_sensor camera_sensor_if = {
	version: 	0x00,
	name:		"OV3640",
	sensor_type:	SENSOR_ISP,
	sensor_interface:SENSOR_PARALLEL,
	parallel_mode:	PAR_MODE_NOBT10,
	hs_polarity:	SYNC_ACTIVE_HIGH,
	vs_polarity:	SYNC_ACTIVE_HIGH,
	init: 		ov3640sensor_init,
	cleanup:	ov3640sensor_cleanup,
	power_on:	ov3640sensor_power_on,
	power_off:	ov3640sensor_power_off,
	try_format:	ov3640sensor_try_format,
	calc_xclk:	ov3640sensor_calc_xclk,
	configure:	ov3640sensor_configure,
	query_control:	ov3640sensor_query_control,
	get_control:	ov3640sensor_get_control,
	set_control:	ov3640sensor_set_control,
};


MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("OV3640 Sensor Driver");
MODULE_LICENSE("GPL");


late_initcall(ov3640sensor_register);
module_exit(ov3640sensor_unregister);

#endif	/* ifdef CAMERA_ov3640 */
