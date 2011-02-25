/*
 * drivers/media/video/omap/sensor_ex3691.c
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 * TI EX3691 (also known as Micron MT9D111) camera SOC driver. It implements
 * the sensor interface defined in sensor_if.h. 
 *
 * March 2006 - v2. Added crop support for digital zoom & view finder.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/sched.h>
#include <media/video-buf.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/twl4030.h>

#include "sensor_if.h"
#include "ex3691.h"

#if defined(CONFIG_VIDEO_OMAP24XX_CAMERA) || defined(CONFIG_VIDEO_OMAP34XX_CAMERA)
#undef EX3691_DEBUG
#define EX3691_REG_DUMP

#define MOD_NAME "EX3691:"

#ifdef EX3691_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ERR MOD_NAME format "\n", ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#ifdef EX3691_REG_DUMP
void dump_key_ex3691_regs(void *priv);
#endif

#define OMAP24XX_DIR_OUTPUT 0

/* Board specific code. Currently the only supported board is OMAP2430SDP.
 * This code can be moved to a board file.
 */
#ifdef CONFIG_MACH_OMAP_2430SDP
/* The EX3691 I2C camera chip has a fixed slave address of 0x48 or 0x5D
   depending on SADDR is LOW or HIGH. It is 0x48 on 2430SDP. */
#define EX3691_I2C_ADDR		0x48

#define CAMERA_STANDBY_GPIO 	32
#define CAMERA_RESET_GPIO  	128

#define VAUX3_2_8_V			0x03
#define VAUX3_DEV_GRP_P1		0x20
#define VAUX3_DEV_GRP_NONE	0x00

static int
board_init(void)
{
#ifdef CONFIG_TWL4030_CORE
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX3_2_8_V, TWL4030_VAUX3_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX3_DEV_GRP_P1,TWL4030_VAUX3_DEV_GRP);
#else
#error "no power companion board defined!" 
#endif

	/* Request and configure gpio pins */
	if (omap_request_gpio(CAMERA_STANDBY_GPIO) != 0)
		return -EIO;
	/* set to output mode */
	omap_set_gpio_direction(CAMERA_STANDBY_GPIO, OMAP24XX_DIR_OUTPUT);	
	if (omap_request_gpio(CAMERA_RESET_GPIO) != 0) {
		omap_free_gpio(CAMERA_STANDBY_GPIO);	   
		return -EIO;
	}
	/* set to output mode */
	omap_set_gpio_direction(CAMERA_RESET_GPIO, OMAP24XX_DIR_OUTPUT);

 	/* disable STANDBY */
 	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 0);

 	/* have to put sensor to reset to guarantee detection */ 
 	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 0); 
	udelay(100);

	/* nRESET is active LOW. set HIGH to release reset */ 
 	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1); 
	/* give sensor sometime to get out of the reset */
	udelay(500);

	return 0; 
}

static void
board_cleanup(void)
{
#ifdef CONFIG_TWL4030_CORE
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX3_DEV_GRP_NONE,TWL4030_VAUX3_DEV_GRP);
#else
#error "no power companion board defined!" 
#endif

	omap_free_gpio(CAMERA_STANDBY_GPIO);	   
	omap_free_gpio(CAMERA_RESET_GPIO);	   
}

static void
board_exit_standby(void)
{
 	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 0);
}
static void
board_enter_standby(void)
{	
	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 1);
}
#elif defined(CONFIG_MACH_OMAP_3430SDP)
/* The EX3691 I2C camera chip has a fixed slave address of 0x48 or 0x5D
   depending on SADDR is LOW or HIGH. It is 0x48 on 3430SDP. */
#define EX3691_I2C_ADDR		0x48

#define CAMERA_STANDBY_GPIO 	2
#define CAMERA_RESET_GPIO  	15

#define VAUX3_2_8_V			0x09
#define VAUX3_DEV_GRP_P1		0x20
#define VAUX3_DEV_GRP_NONE	0x00

static int
board_init(void)
{
#ifdef CONFIG_TWL4030_CORE
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX3_2_8_V, TWL4030_VAUX2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX3_DEV_GRP_P1,TWL4030_VAUX2_DEV_GRP);

#else
#error "no power companion board defined!" 
#endif

	/* Request and configure gpio pins */
	if (omap_request_gpio(CAMERA_STANDBY_GPIO) != 0)
		return -EIO;
	/* set to output mode */
	omap_set_gpio_direction(CAMERA_STANDBY_GPIO, OMAP24XX_DIR_OUTPUT);	
	if (omap_request_gpio(CAMERA_RESET_GPIO) != 0) {
		omap_free_gpio(CAMERA_STANDBY_GPIO);	   
		return -EIO;
	}
	/* set to output mode */
	omap_set_gpio_direction(CAMERA_RESET_GPIO, OMAP24XX_DIR_OUTPUT);

 	/* disable STANDBY */
 	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 0);

 	/* have to put sensor to reset to guarantee detection */ 
 	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 0); 
	udelay(100);

	/* nRESET is active LOW. set HIGH to release reset */ 
 	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1); 
	/* give sensor sometime to get out of the reset */
	udelay(500);

	return 0; 
}

static void
board_cleanup(void)
{
#ifdef CONFIG_TWL4030_CORE
//	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
//		VAUX3_DEV_GRP_NONE,TWL4030_VAUX2_DEV_GRP);
#else
#error "no power companion board defined!" 
#endif

	omap_free_gpio(CAMERA_STANDBY_GPIO);	   
	omap_free_gpio(CAMERA_RESET_GPIO);	   
}

static void
board_exit_standby(void)
{
 	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 0);
}
static void
board_enter_standby(void)
{	
	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 1);
}

#endif  /* CONFIG_MACH_OMAP_2430SDP */

#define EX3691_DEF_MCLK_MHZ	40
#define EX3691_MAX_MCLK_MHZ	80
#define EX3691_XCLK_HZ		24000000

#define EX3691_MIN_FPS		10
#define EX3691_DEF_FPS		15   /* At 40MHz mclk */
#define EX3691_MAX_FPS		30   /* At 80MHz mclk */

#define QCIF_WIDTH		176
#define QCIF_HEIGHT		144
#define QVGA_WIDTH		320
#define QVGA_HEIGHT		240
#define SVGA_WIDTH		800
#define SVGA_HEIGHT		600
#define UXGA_WIDTH		1600
#define UXGA_HEIGHT		1200

#define EX3691_I2C_DELAY	3  /* msec delay b/w accesses */	
 
static struct ex3691_sensor {
	/* I2C parameters */
	struct i2c_client client;
	struct i2c_driver driver;
	
	int ver; 	/* SOC version */
	int mclk; 	/* sensor master clock in MHz */ 

	unsigned long width_a;
	unsigned long height_a;
	int pfmt_a;
	int swapbyte_a;
	struct v4l2_rect crop_rect_a; 
	unsigned long width_b;
	unsigned long height_b;
	int pfmt_b;
	int swapbyte_b;
	struct v4l2_rect crop_rect_b; 
} ex3691;

/* list of image formats supported by the sensor driver */
const static struct v4l2_fmtdesc ex3691_formats[] = {
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
	},{
		.description	= "BAYER10",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	}
};
#define NUM_CAPTURE_FORMATS (sizeof(ex3691_formats)/sizeof(ex3691_formats[0]))

/* define a structure for EX3691 register initialization values */
struct ex3691_reg {
	u8	page;	/* page number */
	u8 	reg;    /* 8-bit offset */
	u16 	val;	/* 16-bit value */
};

enum pixel_format {RGB565, RGB565X, RGB555, RGB555X, YUYV, UYVY, BAYER10};
#define NUM_PIXEL_FORMATS 7

#define EX3691_TOK_TERM 	0xFF	/* terminating token for reg list */
#define EX3691_TOK_DELAY	100	/* delay token for reg list */

/* The following EX3691 commands are generated by DevWare. To keep the original
   format, we don't convert the C++ comment */ 
/* EX3691 SOC register initialization for all image sizes, pixel formats
 */
const static struct ex3691_reg initial_list[]= {
	{0, 0x33, 0x0343 	}, // RESERVED_CORE_33
       {1, 0xC6, 0xA115	}, //SEQ_LLMODE
	{1, 0xC8, 0x0020	}, //SEQ_LLMODE
	{0, 0x38, 0x0866 	}, // RESERVED_CORE_38
	{EX3691_TOK_DELAY, 0x00, 0x0064 }, // Delay =100ms
	{2, 0x80, 0x0168 	}, // LENS_CORRECTION_CONTROL
	{2, 0x81, 0x6432 	}, // ZONE_BOUNDS_X1_X2
	{2, 0x82, 0x3296 	}, // ZONE_BOUNDS_X0_X3
	{2, 0x83, 0x9664 	}, // ZONE_BOUNDS_X4_X5
	{2, 0x84, 0x5028 	}, // ZONE_BOUNDS_Y1_Y2
	{2, 0x85, 0x2878 	}, // ZONE_BOUNDS_Y0_Y3
	{2, 0x86, 0x7850 	}, // ZONE_BOUNDS_Y4_Y5
	{2, 0x87, 0x0000 	}, // CENTER_OFFSET
	{2, 0x88, 0x0152 	}, // FX_RED
	{2, 0x89, 0x015C 	}, // FX_GREEN
	{2, 0x8A, 0x00F4 	}, // FX_BLUE
	{2, 0x8B, 0x0108 	}, // FY_RED
	{2, 0x8C, 0x00FA 	}, // FY_GREEN
	{2, 0x8D, 0x00CF 	}, // FY_BLUE
	{2, 0x8E, 0x09AD 	}, // DF_DX_RED
	{2, 0x8F, 0x091E 	}, // DF_DX_GREEN
	{2, 0x90, 0x0B3F 	}, // DF_DX_BLUE
	{2, 0x91, 0x0C85 	}, // DF_DY_RED
	{2, 0x92, 0x0CFF 	}, // DF_DY_GREEN
	{2, 0x93, 0x0D86 	}, // DF_DY_BLUE
	{2, 0x94, 0x163A 	}, // SECOND_DERIV_ZONE_0_RED
	{2, 0x95, 0x0E47 	}, // SECOND_DERIV_ZONE_0_GREEN
	{2, 0x96, 0x103C 	}, // SECOND_DERIV_ZONE_0_BLUE
	{2, 0x97, 0x1D35 	}, // SECOND_DERIV_ZONE_1_RED
	{2, 0x98, 0x173E 	}, // SECOND_DERIV_ZONE_1_GREEN
	{2, 0x99, 0x1119 	}, // SECOND_DERIV_ZONE_1_BLUE
	{2, 0x9A, 0x1663 	}, // SECOND_DERIV_ZONE_2_RED
	{2, 0x9B, 0x1569 	}, // SECOND_DERIV_ZONE_2_GREEN
	{2, 0x9C, 0x104C 	}, // SECOND_DERIV_ZONE_2_BLUE
	{2, 0x9D, 0x1015 	}, // SECOND_DERIV_ZONE_3_RED
	{2, 0x9E, 0x1010 	}, // SECOND_DERIV_ZONE_3_GREEN
	{2, 0x9F, 0x0B0A 	}, // SECOND_DERIV_ZONE_3_BLUE
	{2, 0xA0, 0x0D53 	}, // SECOND_DERIV_ZONE_4_RED
	{2, 0xA1, 0x0D51 	}, // SECOND_DERIV_ZONE_4_GREEN
	{2, 0xA2, 0x0A44 	}, // SECOND_DERIV_ZONE_4_BLUE
	{2, 0xA3, 0x1545 	}, // SECOND_DERIV_ZONE_5_RED
	{2, 0xA4, 0x1643 	}, // SECOND_DERIV_ZONE_5_GREEN
	{2, 0xA5, 0x1231 	}, // SECOND_DERIV_ZONE_5_BLUE
	{2, 0xA6, 0x0047 	}, // SECOND_DERIV_ZONE_6_RED
	{2, 0xA7, 0x035C 	}, // SECOND_DERIV_ZONE_6_GREEN
	{2, 0xA8, 0xFE30 	}, // SECOND_DERIV_ZONE_6_BLUE
	{2, 0xA9, 0x4625 	}, // SECOND_DERIV_ZONE_7_RED
	{2, 0xAA, 0x47F3 	}, // SECOND_DERIV_ZONE_7_GREEN
	{2, 0xAB, 0x5859 	}, // SECOND_DERIV_ZONE_7_BLUE
	{2, 0xAC, 0x0000 	}, // X2_FACTORS
	{2, 0xAD, 0x0000 	}, // GLOBAL_OFFSET_FXY_FUNCTION
	{2, 0xAE, 0x0000 	}, // K_FACTOR_IN_K_FX_FY
	{1, 0x08, 0x01FC 	}, // COLOR_PIPELINE_CONTROL
	{EX3691_TOK_DELAY, 0x00, 0x0064 }, // Delay =100ms
	{1, 0xBE, 0x0004 	}, // YUV_YCBCR_CONTROL
	{0, 0x65, 0xA000 	}, // CLOCK_ENABLING
	{EX3691_TOK_DELAY, 0x00, 0x0064 }, // Delay =100ms
	{1, 0xC6, 0x104C 	}, // MCU_ADDRESS
	{1, 0xC8, 0x0000 	}, // MCU_DATA_0
	{1, 0xC6, 0x0400 	}, // MCU_ADDRESS
	{1, 0xC8, 0x3736 	}, // MCU_DATA_0
	{1, 0xC9, 0x3C3C 	}, // MCU_DATA_1
	{1, 0xCA, 0x3C3C 	}, // MCU_DATA_2
	{1, 0xCB, 0x30EC 	}, // MCU_DATA_3
	{1, 0xCC, 0x08BD 	}, // MCU_DATA_4
	{1, 0xCD, 0xA1B6 	}, // MCU_DATA_5
	{1, 0xCE, 0xD6A9 	}, // MCU_DATA_6
	{1, 0xCF, 0xF102 	}, // MCU_DATA_7
	{1, 0xC6, 0x0410 	}, // MCU_ADDRESS
	{1, 0xC8, 0xBD22 	}, // MCU_DATA_0
	{1, 0xC9, 0x6DF6 	}, // MCU_DATA_1
	{1, 0xCA, 0x02BE 	}, // MCU_DATA_2
	{1, 0xCB, 0x4F30 	}, // MCU_DATA_3
	{1, 0xCC, 0xED06 	}, // MCU_DATA_4
	{1, 0xCD, 0xF602 	}, // MCU_DATA_5
	{1, 0xCE, 0xC1ED 	}, // MCU_DATA_6
	{1, 0xCF, 0x04EC 	}, // MCU_DATA_7
	{1, 0xC6, 0x0420 	}, // MCU_ADDRESS
	{1, 0xC8, 0x06A3 	}, // MCU_DATA_0
	{1, 0xC9, 0x04ED 	}, // MCU_DATA_1
	{1, 0xCA, 0x02D6 	}, // MCU_DATA_2
	{1, 0xCB, 0xA94F 	}, // MCU_DATA_3
	{1, 0xCC, 0xED00 	}, // MCU_DATA_4
	{1, 0xCD, 0xF602 	}, // MCU_DATA_5
	{1, 0xCE, 0xBDBD 	}, // MCU_DATA_6
	{1, 0xCF, 0xD268 	}, // MCU_DATA_7
	{1, 0xC6, 0x0430 	}, // MCU_ADDRESS
	{1, 0xC8, 0xF602 	}, // MCU_DATA_0
	{1, 0xC9, 0xBE30 	}, // MCU_DATA_1
	{1, 0xCA, 0xE003 	}, // MCU_DATA_2
	{1, 0xCB, 0xF702 	}, // MCU_DATA_3
	{1, 0xCC, 0xC7F6 	}, // MCU_DATA_4
	{1, 0xCD, 0x02BF 	}, // MCU_DATA_5
	{1, 0xCE, 0x4FED 	}, // MCU_DATA_6
	{1, 0xCF, 0x06F6 	}, // MCU_DATA_7
	{1, 0xC6, 0x0440 	}, // MCU_ADDRESS
	{1, 0xC8, 0x02C2 	}, // MCU_DATA_0
	{1, 0xC9, 0xED04 	}, // MCU_DATA_1
	{1, 0xCA, 0xEC06 	}, // MCU_DATA_2
	{1, 0xCB, 0xA304 	}, // MCU_DATA_3
	{1, 0xCC, 0xED02 	}, // MCU_DATA_4
	{1, 0xCD, 0xD6A9 	}, // MCU_DATA_5
	{1, 0xCE, 0x4FED 	}, // MCU_DATA_6
	{1, 0xCF, 0x00F6 	}, // MCU_DATA_7
	{1, 0xC6, 0x0450 	}, // MCU_ADDRESS
	{1, 0xC8, 0x02BD 	}, // MCU_DATA_0
	{1, 0xC9, 0xBDD2 	}, // MCU_DATA_1
	{1, 0xCA, 0x68F6 	}, // MCU_DATA_2
	{1, 0xCB, 0x02BF 	}, // MCU_DATA_3
	{1, 0xCC, 0x30E0 	}, // MCU_DATA_4
	{1, 0xCD, 0x03F7 	}, // MCU_DATA_5
	{1, 0xCE, 0x02C8 	}, // MCU_DATA_6
	{1, 0xCF, 0xF602 	}, // MCU_DATA_7
	{1, 0xC6, 0x0460 	}, // MCU_ADDRESS
	{1, 0xC8, 0xC04F 	}, // MCU_DATA_0
	{1, 0xC9, 0xED06 	}, // MCU_DATA_1
	{1, 0xCA, 0xF602 	}, // MCU_DATA_2
	{1, 0xCB, 0xC3ED 	}, // MCU_DATA_3
	{1, 0xCC, 0x04EC 	}, // MCU_DATA_4
	{1, 0xCD, 0x06A3 	}, // MCU_DATA_5
	{1, 0xCE, 0x04ED 	}, // MCU_DATA_6
	{1, 0xCF, 0x02D6 	}, // MCU_DATA_7
	{1, 0xC6, 0x0470 	}, // MCU_ADDRESS
	{1, 0xC8, 0xA94F 	}, // MCU_DATA_0
	{1, 0xC9, 0xED00 	}, // MCU_DATA_1
	{1, 0xCA, 0xF602 	}, // MCU_DATA_2
	{1, 0xCB, 0xBDBD 	}, // MCU_DATA_3
	{1, 0xCC, 0xD268 	}, // MCU_DATA_4
	{1, 0xCD, 0xF602 	}, // MCU_DATA_5
	{1, 0xCE, 0xC07E 	}, // MCU_DATA_6
	{1, 0xCF, 0x050B 	}, // MCU_DATA_7
	{1, 0xC6, 0x0480 	}, // MCU_ADDRESS
	{1, 0xC8, 0xF602 	}, // MCU_DATA_0
	{1, 0xC9, 0xC44F 	}, // MCU_DATA_1
	{1, 0xCA, 0x30ED 	}, // MCU_DATA_2
	{1, 0xCB, 0x06F6 	}, // MCU_DATA_3
	{1, 0xCC, 0x02C1 	}, // MCU_DATA_4
	{1, 0xCD, 0xED04 	}, // MCU_DATA_5
	{1, 0xCE, 0xEC06 	}, // MCU_DATA_6
	{1, 0xCF, 0xA304 	}, // MCU_DATA_7
	{1, 0xC6, 0x0490 	}, // MCU_ADDRESS
	{1, 0xC8, 0xED02 	}, // MCU_DATA_0
	{1, 0xC9, 0xD6A8 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FED 	}, // MCU_DATA_2
	{1, 0xCB, 0x06D6 	}, // MCU_DATA_3
	{1, 0xCC, 0xA9ED 	}, // MCU_DATA_4
	{1, 0xCD, 0x04EC 	}, // MCU_DATA_5
	{1, 0xCE, 0x06A3 	}, // MCU_DATA_6
	{1, 0xCF, 0x04ED 	}, // MCU_DATA_7
	{1, 0xC6, 0x04A0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x00F6 	}, // MCU_DATA_0
	{1, 0xC9, 0x02BD 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FBD 	}, // MCU_DATA_2
	{1, 0xCB, 0xD268 	}, // MCU_DATA_3
	{1, 0xCC, 0xF602 	}, // MCU_DATA_4
	{1, 0xCD, 0xC430 	}, // MCU_DATA_5
	{1, 0xCE, 0xE003 	}, // MCU_DATA_6
	{1, 0xCF, 0xF702 	}, // MCU_DATA_7
	{1, 0xC6, 0x04B0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xC7F6 	}, // MCU_DATA_0
	{1, 0xC9, 0x02C5 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FED 	}, // MCU_DATA_2
	{1, 0xCB, 0x06F6 	}, // MCU_DATA_3
	{1, 0xCC, 0x02C2 	}, // MCU_DATA_4
	{1, 0xCD, 0xED04 	}, // MCU_DATA_5
	{1, 0xCE, 0xEC06 	}, // MCU_DATA_6
	{1, 0xCF, 0xA304 	}, // MCU_DATA_7
	{1, 0xC6, 0x04C0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xED02 	}, // MCU_DATA_0
	{1, 0xC9, 0xD6A8 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FED 	}, // MCU_DATA_2
	{1, 0xCB, 0x06D6 	}, // MCU_DATA_3
	{1, 0xCC, 0xA9ED 	}, // MCU_DATA_4
	{1, 0xCD, 0x04EC 	}, // MCU_DATA_5
	{1, 0xCE, 0x06A3 	}, // MCU_DATA_6
	{1, 0xCF, 0x04ED 	}, // MCU_DATA_7
	{1, 0xC6, 0x04D0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x00F6 	}, // MCU_DATA_0
	{1, 0xC9, 0x02BD 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FBD 	}, // MCU_DATA_2
	{1, 0xCB, 0xD268 	}, // MCU_DATA_3
	{1, 0xCC, 0xF602 	}, // MCU_DATA_4
	{1, 0xCD, 0xC530 	}, // MCU_DATA_5
	{1, 0xCE, 0xE003 	}, // MCU_DATA_6
	{1, 0xCF, 0xF702 	}, // MCU_DATA_7
	{1, 0xC6, 0x04E0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xC8F6 	}, // MCU_DATA_0
	{1, 0xC9, 0x02C6 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FED 	}, // MCU_DATA_2
	{1, 0xCB, 0x06F6 	}, // MCU_DATA_3
	{1, 0xCC, 0x02C3 	}, // MCU_DATA_4
	{1, 0xCD, 0xED04 	}, // MCU_DATA_5
	{1, 0xCE, 0xEC06 	}, // MCU_DATA_6
	{1, 0xCF, 0xA304 	}, // MCU_DATA_7
	{1, 0xC6, 0x04F0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xED02 	}, // MCU_DATA_0
	{1, 0xC9, 0xD6A8 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FED 	}, // MCU_DATA_2
	{1, 0xCB, 0x06D6 	}, // MCU_DATA_3
	{1, 0xCC, 0xA9ED 	}, // MCU_DATA_4
	{1, 0xCD, 0x04EC 	}, // MCU_DATA_5
	{1, 0xCE, 0x06A3 	}, // MCU_DATA_6
	{1, 0xCF, 0x04ED 	}, // MCU_DATA_7
	{1, 0xC6, 0x0500 	}, // MCU_ADDRESS
	{1, 0xC8, 0x00F6 	}, // MCU_DATA_0
	{1, 0xC9, 0x02BD 	}, // MCU_DATA_1
	{1, 0xCA, 0x4FBD 	}, // MCU_DATA_2
	{1, 0xCB, 0xD268 	}, // MCU_DATA_3
	{1, 0xCC, 0xF602 	}, // MCU_DATA_4
	{1, 0xCD, 0xC630 	}, // MCU_DATA_5
	{1, 0xCE, 0xE003 	}, // MCU_DATA_6
	{1, 0xCF, 0xF702 	}, // MCU_DATA_7
	{1, 0xC6, 0x0510 	}, // MCU_ADDRESS
	{1, 0xC8, 0xC9EE 	}, // MCU_DATA_0
	{1, 0xC9, 0x08EC 	}, // MCU_DATA_1
	{1, 0xCA, 0x0030 	}, // MCU_DATA_2
	{1, 0xCB, 0xED02 	}, // MCU_DATA_3
	{1, 0xCC, 0xF602 	}, // MCU_DATA_4
	{1, 0xCD, 0xC74F 	}, // MCU_DATA_5
	{1, 0xCE, 0xED00 	}, // MCU_DATA_6
	{1, 0xCF, 0xCC00 	}, // MCU_DATA_7
	{1, 0xC6, 0x0520 	}, // MCU_ADDRESS
	{1, 0xC8, 0x80BD 	}, // MCU_DATA_0
	{1, 0xC9, 0xD268 	}, // MCU_DATA_1
	{1, 0xCA, 0x301A 	}, // MCU_DATA_2
	{1, 0xCB, 0xEE08 	}, // MCU_DATA_3
	{1, 0xCC, 0xEC02 	}, // MCU_DATA_4
	{1, 0xCD, 0x18ED 	}, // MCU_DATA_5
	{1, 0xCE, 0x00EE 	}, // MCU_DATA_6
	{1, 0xCF, 0x08EC 	}, // MCU_DATA_7
	{1, 0xC6, 0x0530 	}, // MCU_ADDRESS
	{1, 0xC8, 0x0230 	}, // MCU_DATA_0
	{1, 0xC9, 0xED02 	}, // MCU_DATA_1
	{1, 0xCA, 0xF602 	}, // MCU_DATA_2
	{1, 0xCB, 0xC84F 	}, // MCU_DATA_3
	{1, 0xCC, 0xED00 	}, // MCU_DATA_4
	{1, 0xCD, 0xCC00 	}, // MCU_DATA_5
	{1, 0xCE, 0x80BD 	}, // MCU_DATA_6
	{1, 0xCF, 0xD268 	}, // MCU_DATA_7
	{1, 0xC6, 0x0540 	}, // MCU_ADDRESS
	{1, 0xC8, 0x301A 	}, // MCU_DATA_0
	{1, 0xC9, 0xEE08 	}, // MCU_DATA_1
	{1, 0xCA, 0xEC02 	}, // MCU_DATA_2
	{1, 0xCB, 0x18ED 	}, // MCU_DATA_3
	{1, 0xCC, 0x02EE 	}, // MCU_DATA_4
	{1, 0xCD, 0x08EC 	}, // MCU_DATA_5
	{1, 0xCE, 0x0430 	}, // MCU_DATA_6
	{1, 0xCF, 0xED02 	}, // MCU_DATA_7
	{1, 0xC6, 0x0550 	}, // MCU_ADDRESS
	{1, 0xC8, 0xF602 	}, // MCU_DATA_0
	{1, 0xC9, 0xC94F 	}, // MCU_DATA_1
	{1, 0xCA, 0xED00 	}, // MCU_DATA_2
	{1, 0xCB, 0xCC00 	}, // MCU_DATA_3
	{1, 0xCC, 0x80BD 	}, // MCU_DATA_4
	{1, 0xCD, 0xD268 	}, // MCU_DATA_5
	{1, 0xCE, 0x301A 	}, // MCU_DATA_6
	{1, 0xCF, 0xEE08 	}, // MCU_DATA_7
	{1, 0xC6, 0x0560 	}, // MCU_ADDRESS
	{1, 0xC8, 0xEC02 	}, // MCU_DATA_0
	{1, 0xC9, 0x18ED 	}, // MCU_DATA_1
	{1, 0xCA, 0x0430 	}, // MCU_DATA_2
	{1, 0xCB, 0xC60A 	}, // MCU_DATA_3
	{1, 0xCC, 0x3A35 	}, // MCU_DATA_4
	{1, 0xCD, 0x3930 	}, // MCU_DATA_5
	{1, 0xCE, 0x8FC3 	}, // MCU_DATA_6
	{1, 0xCF, 0xFFE5 	}, // MCU_DATA_7
	{1, 0xC6, 0x0570 	}, // MCU_ADDRESS
	{1, 0xC8, 0x8F35 	}, // MCU_DATA_0
	{1, 0xC9, 0x6F1A 	}, // MCU_DATA_1
	{1, 0xCA, 0xDC8C 	}, // MCU_DATA_2
	{1, 0xCB, 0x2C02 	}, // MCU_DATA_3
	{1, 0xCC, 0x6C1A 	}, // MCU_DATA_4
	{1, 0xCD, 0xDC8E 	}, // MCU_DATA_5
	{1, 0xCE, 0x2C06 	}, // MCU_DATA_6
	{1, 0xCF, 0xE61A 	}, // MCU_DATA_7
	{1, 0xC6, 0x0580 	}, // MCU_ADDRESS
	{1, 0xC8, 0xCB02 	}, // MCU_DATA_0
	{1, 0xC9, 0xE71A 	}, // MCU_DATA_1
	{1, 0xCA, 0xDC90 	}, // MCU_DATA_2
	{1, 0xCB, 0x2C06 	}, // MCU_DATA_3
	{1, 0xCC, 0xE61A 	}, // MCU_DATA_4
	{1, 0xCD, 0xCB04 	}, // MCU_DATA_5
	{1, 0xCE, 0xE71A 	}, // MCU_DATA_6
	{1, 0xCF, 0xDC94 	}, // MCU_DATA_7
	{1, 0xC6, 0x0590 	}, // MCU_ADDRESS
	{1, 0xC8, 0x2C06 	}, // MCU_DATA_0
	{1, 0xC9, 0xE61A 	}, // MCU_DATA_1
	{1, 0xCA, 0xCB08 	}, // MCU_DATA_2
	{1, 0xCB, 0xE71A 	}, // MCU_DATA_3
	{1, 0xCC, 0xDC96 	}, // MCU_DATA_4
	{1, 0xCD, 0x2C06 	}, // MCU_DATA_5
	{1, 0xCE, 0xE61A 	}, // MCU_DATA_6
	{1, 0xCF, 0xCB10 	}, // MCU_DATA_7
	{1, 0xC6, 0x05A0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xE71A 	}, // MCU_DATA_0
	{1, 0xC9, 0xDC98 	}, // MCU_DATA_1
	{1, 0xCA, 0x2C06 	}, // MCU_DATA_2
	{1, 0xCB, 0xE61A 	}, // MCU_DATA_3
	{1, 0xCC, 0xCB20 	}, // MCU_DATA_4
	{1, 0xCD, 0xE71A 	}, // MCU_DATA_5
	{1, 0xCE, 0x5F4F 	}, // MCU_DATA_6
	{1, 0xCF, 0xED0D 	}, // MCU_DATA_7
	{1, 0xC6, 0x05B0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xED0B 	}, // MCU_DATA_0
	{1, 0xC9, 0xC608 	}, // MCU_DATA_1
	{1, 0xCA, 0xE709 	}, // MCU_DATA_2
	{1, 0xCB, 0x4FE6 	}, // MCU_DATA_3
	{1, 0xCC, 0x092A 	}, // MCU_DATA_4
	{1, 0xCD, 0x0143 	}, // MCU_DATA_5
	{1, 0xCE, 0x058F 	}, // MCU_DATA_6
	{1, 0xCF, 0xEC8A 	}, // MCU_DATA_7
	{1, 0xC6, 0x05C0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x30ED 	}, // MCU_DATA_0
	{1, 0xC9, 0x0F2C 	}, // MCU_DATA_1
	{1, 0xCA, 0x065F 	}, // MCU_DATA_2
	{1, 0xCB, 0x4FA3 	}, // MCU_DATA_3
	{1, 0xCC, 0x0FED 	}, // MCU_DATA_4
	{1, 0xCD, 0x0FC6 	}, // MCU_DATA_5
	{1, 0xCE, 0x04E7 	}, // MCU_DATA_6
	{1, 0xCF, 0x0A20 	}, // MCU_DATA_7
	{1, 0xC6, 0x05D0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x07EC 	}, // MCU_DATA_0
	{1, 0xC9, 0x0F04 	}, // MCU_DATA_1
	{1, 0xCA, 0xED0F 	}, // MCU_DATA_2
	{1, 0xCB, 0x6A0A 	}, // MCU_DATA_3
	{1, 0xCC, 0x6D0F 	}, // MCU_DATA_4
	{1, 0xCD, 0x2704 	}, // MCU_DATA_5
	{1, 0xCE, 0x6D0A 	}, // MCU_DATA_6
	{1, 0xCF, 0x26F1 	}, // MCU_DATA_7
	{1, 0xC6, 0x05E0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x4FE6 	}, // MCU_DATA_0
	{1, 0xC9, 0x092A 	}, // MCU_DATA_1
	{1, 0xCA, 0x0143 	}, // MCU_DATA_2
	{1, 0xCB, 0xED06 	}, // MCU_DATA_3
	{1, 0xCC, 0x8FC3 	}, // MCU_DATA_4
	{1, 0xCD, 0x0011 	}, // MCU_DATA_5
	{1, 0xCE, 0x30E3 	}, // MCU_DATA_6
	{1, 0xCF, 0x0618 	}, // MCU_DATA_7
	{1, 0xC6, 0x05F0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x8FE6 	}, // MCU_DATA_0
	{1, 0xC9, 0x1018 	}, // MCU_DATA_1
	{1, 0xCA, 0xE700 	}, // MCU_DATA_2
	{1, 0xCB, 0xE609 	}, // MCU_DATA_3
	{1, 0xCC, 0xC104 	}, // MCU_DATA_4
	{1, 0xCD, 0x2F10 	}, // MCU_DATA_5
	{1, 0xCE, 0xEC0B 	}, // MCU_DATA_6
	{1, 0xCF, 0x0505 	}, // MCU_DATA_7
	{1, 0xC6, 0x0600 	}, // MCU_ADDRESS
	{1, 0xC8, 0x05ED 	}, // MCU_DATA_0
	{1, 0xC9, 0x0BE6 	}, // MCU_DATA_1
	{1, 0xCA, 0x0A4F 	}, // MCU_DATA_2
	{1, 0xCB, 0xE30B 	}, // MCU_DATA_3
	{1, 0xCC, 0xED0B 	}, // MCU_DATA_4
	{1, 0xCD, 0x200E 	}, // MCU_DATA_5
	{1, 0xCE, 0xEC0D 	}, // MCU_DATA_6
	{1, 0xCF, 0x0505 	}, // MCU_DATA_7
	{1, 0xC6, 0x0610 	}, // MCU_ADDRESS
	{1, 0xC8, 0x05ED 	}, // MCU_DATA_0
	{1, 0xC9, 0x0DE6 	}, // MCU_DATA_1
	{1, 0xCA, 0x0A4F 	}, // MCU_DATA_2
	{1, 0xCB, 0xE30D 	}, // MCU_DATA_3
	{1, 0xCC, 0xED0D 	}, // MCU_DATA_4
	{1, 0xCD, 0x6A09 	}, // MCU_DATA_5
	{1, 0xCE, 0x6D09 	}, // MCU_DATA_6
	{1, 0xCF, 0x2C96 	}, // MCU_DATA_7
	{1, 0xC6, 0x0620 	}, // MCU_ADDRESS
	{1, 0xC8, 0xCC01 	}, // MCU_DATA_0
	{1, 0xC9, 0x60ED 	}, // MCU_DATA_1
	{1, 0xCA, 0x00EC 	}, // MCU_DATA_2
	{1, 0xCB, 0x0DBD 	}, // MCU_DATA_3
	{1, 0xCC, 0xD350 	}, // MCU_DATA_4
	{1, 0xCD, 0xCC01 	}, // MCU_DATA_5
	{1, 0xCE, 0x6130 	}, // MCU_DATA_6
	{1, 0xCF, 0xED00 	}, // MCU_DATA_7
	{1, 0xC6, 0x0630 	}, // MCU_ADDRESS
	{1, 0xC8, 0xEC0B 	}, // MCU_DATA_0
	{1, 0xC9, 0xBDD3 	}, // MCU_DATA_1
	{1, 0xCA, 0x5030 	}, // MCU_DATA_2
	{1, 0xCB, 0xE612 	}, // MCU_DATA_3
	{1, 0xCC, 0x4F17 	}, // MCU_DATA_4
	{1, 0xCD, 0x5FED 	}, // MCU_DATA_5
	{1, 0xCE, 0x0FE6 	}, // MCU_DATA_6
	{1, 0xCF, 0x114F 	}, // MCU_DATA_7
	{1, 0xC6, 0x0640 	}, // MCU_ADDRESS
	{1, 0xC8, 0xE30F 	}, // MCU_DATA_0
	{1, 0xC9, 0xED0F 	}, // MCU_DATA_1
	{1, 0xCA, 0xCC01 	}, // MCU_DATA_2
	{1, 0xCB, 0x62ED 	}, // MCU_DATA_3
	{1, 0xCC, 0x00EC 	}, // MCU_DATA_4
	{1, 0xCD, 0x0FBD 	}, // MCU_DATA_5
	{1, 0xCE, 0xD350 	}, // MCU_DATA_6
	{1, 0xCF, 0x30E6 	}, // MCU_DATA_7
	{1, 0xC6, 0x0650 	}, // MCU_ADDRESS
	{1, 0xC8, 0x144F 	}, // MCU_DATA_0
	{1, 0xC9, 0x175F 	}, // MCU_DATA_1
	{1, 0xCA, 0xED0F 	}, // MCU_DATA_2
	{1, 0xCB, 0xE613 	}, // MCU_DATA_3
	{1, 0xCC, 0x4FE3 	}, // MCU_DATA_4
	{1, 0xCD, 0x0FED 	}, // MCU_DATA_5
	{1, 0xCE, 0x0FCC 	}, // MCU_DATA_6
	{1, 0xCF, 0x0163 	}, // MCU_DATA_7
	{1, 0xC6, 0x0660 	}, // MCU_ADDRESS
	{1, 0xC8, 0xED00 	}, // MCU_DATA_0
	{1, 0xC9, 0xEC0F 	}, // MCU_DATA_1
	{1, 0xCA, 0xBDD3 	}, // MCU_DATA_2
	{1, 0xCB, 0x5030 	}, // MCU_DATA_3
	{1, 0xCC, 0xE616 	}, // MCU_DATA_4
	{1, 0xCD, 0x4F17 	}, // MCU_DATA_5
	{1, 0xCE, 0x5FED 	}, // MCU_DATA_6
	{1, 0xCF, 0x0FE6 	}, // MCU_DATA_7
	{1, 0xC6, 0x0670 	}, // MCU_ADDRESS
	{1, 0xC8, 0x154F 	}, // MCU_DATA_0
	{1, 0xC9, 0xE30F 	}, // MCU_DATA_1
	{1, 0xCA, 0xED0F 	}, // MCU_DATA_2
	{1, 0xCB, 0xCC01 	}, // MCU_DATA_3
	{1, 0xCC, 0x64ED 	}, // MCU_DATA_4
	{1, 0xCD, 0x00EC 	}, // MCU_DATA_5
	{1, 0xCE, 0x0FBD 	}, // MCU_DATA_6
	{1, 0xCF, 0xD350 	}, // MCU_DATA_7
	{1, 0xC6, 0x0680 	}, // MCU_ADDRESS
	{1, 0xC8, 0x30E6 	}, // MCU_DATA_0
	{1, 0xC9, 0x184F 	}, // MCU_DATA_1
	{1, 0xCA, 0x175F 	}, // MCU_DATA_2
	{1, 0xCB, 0xED0F 	}, // MCU_DATA_3
	{1, 0xCC, 0xE617 	}, // MCU_DATA_4
	{1, 0xCD, 0x4FE3 	}, // MCU_DATA_5
	{1, 0xCE, 0x0FED 	}, // MCU_DATA_6
	{1, 0xCF, 0x0FCC 	}, // MCU_DATA_7
	{1, 0xC6, 0x0690 	}, // MCU_ADDRESS
	{1, 0xC8, 0x0165 	}, // MCU_DATA_0
	{1, 0xC9, 0xED00 	}, // MCU_DATA_1
	{1, 0xCA, 0xEC0F 	}, // MCU_DATA_2
	{1, 0xCB, 0xBDD3 	}, // MCU_DATA_3
	{1, 0xCC, 0x5030 	}, // MCU_DATA_4
	{1, 0xCD, 0xE61A 	}, // MCU_DATA_5
	{1, 0xCE, 0x4F17 	}, // MCU_DATA_6
	{1, 0xCF, 0x5FED 	}, // MCU_DATA_7
	{1, 0xC6, 0x06A0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x0FE6 	}, // MCU_DATA_0
	{1, 0xC9, 0x194F 	}, // MCU_DATA_1
	{1, 0xCA, 0xE30F 	}, // MCU_DATA_2
	{1, 0xCB, 0xED0F 	}, // MCU_DATA_3
	{1, 0xCC, 0xCC01 	}, // MCU_DATA_4
	{1, 0xCD, 0x66ED 	}, // MCU_DATA_5
	{1, 0xCE, 0x00EC 	}, // MCU_DATA_6
	{1, 0xCF, 0x0FBD 	}, // MCU_DATA_7
	{1, 0xC6, 0x06B0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xD350 	}, // MCU_DATA_0
	{1, 0xC9, 0x13AB 	}, // MCU_DATA_1
	{1, 0xCA, 0x0406 	}, // MCU_DATA_2
	{1, 0xCB, 0xCC7F 	}, // MCU_DATA_3
	{1, 0xCC, 0x8030 	}, // MCU_DATA_4
	{1, 0xCD, 0x207A 	}, // MCU_DATA_5
	{1, 0xCE, 0xDC9C 	}, // MCU_DATA_6
	{1, 0xCF, 0x30ED 	}, // MCU_DATA_7
	{1, 0xC6, 0x06C0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x02D6 	}, // MCU_DATA_0
	{1, 0xC9, 0xA44F 	}, // MCU_DATA_1
	{1, 0xCA, 0xED00 	}, // MCU_DATA_2
	{1, 0xCB, 0xCC00 	}, // MCU_DATA_3
	{1, 0xCC, 0x80BD 	}, // MCU_DATA_4
	{1, 0xCD, 0xD268 	}, // MCU_DATA_5
	{1, 0xCE, 0x30E6 	}, // MCU_DATA_6
	{1, 0xCF, 0x03E7 	}, // MCU_DATA_7
	{1, 0xC6, 0x06D0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x11DC 	}, // MCU_DATA_0
	{1, 0xC9, 0x9EED 	}, // MCU_DATA_1
	{1, 0xCA, 0x02D6 	}, // MCU_DATA_2
	{1, 0xCB, 0xA64F 	}, // MCU_DATA_3
	{1, 0xCC, 0xED00 	}, // MCU_DATA_4
	{1, 0xCD, 0xCC00 	}, // MCU_DATA_5
	{1, 0xCE, 0x80BD 	}, // MCU_DATA_6
	{1, 0xCF, 0xD268 	}, // MCU_DATA_7
	{1, 0xC6, 0x06E0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x30E6 	}, // MCU_DATA_0
	{1, 0xC9, 0x03E7 	}, // MCU_DATA_1
	{1, 0xCA, 0x12E6 	}, // MCU_DATA_2
	{1, 0xCB, 0x11A6 	}, // MCU_DATA_3
	{1, 0xCC, 0x123D 	}, // MCU_DATA_4
	{1, 0xCD, 0xED0F 	}, // MCU_DATA_5
	{1, 0xCE, 0x6F0A 	}, // MCU_DATA_6
	{1, 0xCF, 0x2007 	}, // MCU_DATA_7
	{1, 0xC6, 0x06F0 	}, // MCU_ADDRESS
	{1, 0xC8, 0xEC0F 	}, // MCU_DATA_0
	{1, 0xC9, 0x04ED 	}, // MCU_DATA_1
	{1, 0xCA, 0x0F6C 	}, // MCU_DATA_2
	{1, 0xCB, 0x0AEC 	}, // MCU_DATA_3
	{1, 0xCC, 0x0F1A 	}, // MCU_DATA_4
	{1, 0xCD, 0x8300 	}, // MCU_DATA_5
	{1, 0xCE, 0x2024 	}, // MCU_DATA_6
	{1, 0xCF, 0xF183 	}, // MCU_DATA_7
	{1, 0xC6, 0x0700 	}, // MCU_ADDRESS
	{1, 0xC8, 0x0010 	}, // MCU_DATA_0
	{1, 0xC9, 0xED0F 	}, // MCU_DATA_1
	{1, 0xCA, 0xC3E8 	}, // MCU_DATA_2
	{1, 0xCB, 0xFB8F 	}, // MCU_DATA_3
	{1, 0xCC, 0xE600 	}, // MCU_DATA_4
	{1, 0xCD, 0x30E7 	}, // MCU_DATA_5
	{1, 0xCE, 0x08E6 	}, // MCU_DATA_6
	{1, 0xCF, 0x0A86 	}, // MCU_DATA_7
	{1, 0xC6, 0x0710 	}, // MCU_ADDRESS
	{1, 0xC8, 0x203D 	}, // MCU_DATA_0
	{1, 0xC9, 0xEB08 	}, // MCU_DATA_1
	{1, 0xCA, 0xC040 	}, // MCU_DATA_2
	{1, 0xCB, 0xE708 	}, // MCU_DATA_3
	{1, 0xCC, 0xD6BA 	}, // MCU_DATA_4
	{1, 0xCD, 0x4FED 	}, // MCU_DATA_5
	{1, 0xCE, 0x06E6 	}, // MCU_DATA_6
	{1, 0xCF, 0x08ED 	}, // MCU_DATA_7
	{1, 0xC6, 0x0720 	}, // MCU_ADDRESS
	{1, 0xC8, 0x04E3 	}, // MCU_DATA_0
	{1, 0xC9, 0x06ED 	}, // MCU_DATA_1
	{1, 0xCA, 0x0F17 	}, // MCU_DATA_2
	{1, 0xCB, 0x5FED 	}, // MCU_DATA_3
	{1, 0xCC, 0x0FD6 	}, // MCU_DATA_4
	{1, 0xCD, 0xB94F 	}, // MCU_DATA_5
	{1, 0xCE, 0xED06 	}, // MCU_DATA_6
	{1, 0xCF, 0xE608 	}, // MCU_DATA_7
	{1, 0xC6, 0x0730 	}, // MCU_ADDRESS
	{1, 0xC8, 0xED04 	}, // MCU_DATA_0
	{1, 0xC9, 0xE306 	}, // MCU_DATA_1
	{1, 0xCA, 0xE30F 	}, // MCU_DATA_2
	{1, 0xCB, 0xED0F 	}, // MCU_DATA_3
	{1, 0xCC, 0xCC01 	}, // MCU_DATA_4
	{1, 0xCD, 0x52ED 	}, // MCU_DATA_5
	{1, 0xCE, 0x00EC 	}, // MCU_DATA_6
	{1, 0xCF, 0x0FBD 	}, // MCU_DATA_7
	{1, 0xC6, 0x0740 	}, // MCU_ADDRESS
	{1, 0xC8, 0xD350 	}, // MCU_DATA_0
	{1, 0xC9, 0x30C6 	}, // MCU_DATA_1
	{1, 0xCA, 0x1B3A 	}, // MCU_DATA_2
	{1, 0xCB, 0x3539 	}, // MCU_DATA_3
	{1, 0xCC, 0x3C3C 	}, // MCU_DATA_4
	{1, 0xCD, 0x34C6 	}, // MCU_DATA_5
	{1, 0xCE, 0x18F7 	}, // MCU_DATA_6
	{1, 0xCF, 0x02BD 	}, // MCU_DATA_7
	{1, 0xC6, 0x0750 	}, // MCU_ADDRESS
	{1, 0xC8, 0xC6B4 	}, // MCU_DATA_0
	{1, 0xC9, 0xF702 	}, // MCU_DATA_1
	{1, 0xCA, 0xBEC6 	}, // MCU_DATA_2
	{1, 0xCB, 0x96F7 	}, // MCU_DATA_3
	{1, 0xCC, 0x02BF 	}, // MCU_DATA_4
	{1, 0xCD, 0xC680 	}, // MCU_DATA_5
	{1, 0xCE, 0xF702 	}, // MCU_DATA_6
	{1, 0xCF, 0xC0F7 	}, // MCU_DATA_7
	{1, 0xC6, 0x0760 	}, // MCU_ADDRESS
	{1, 0xC8, 0x02C1 	}, // MCU_DATA_0
	{1, 0xC9, 0xF702 	}, // MCU_DATA_1
	{1, 0xCA, 0xC2F7 	}, // MCU_DATA_2
	{1, 0xCB, 0x02C3 	}, // MCU_DATA_3
	{1, 0xCC, 0xF702 	}, // MCU_DATA_4
	{1, 0xCD, 0xC4F7 	}, // MCU_DATA_5
	{1, 0xCE, 0x02C5 	}, // MCU_DATA_6
	{1, 0xCF, 0xF702 	}, // MCU_DATA_7
	{1, 0xC6, 0x0770 	}, // MCU_ADDRESS
	{1, 0xC8, 0xC6CC 	}, // MCU_DATA_0
	{1, 0xC9, 0x02AB 	}, // MCU_DATA_1
	{1, 0xCA, 0x30ED 	}, // MCU_DATA_2
	{1, 0xCB, 0x02FE 	}, // MCU_DATA_3
	{1, 0xCC, 0x1050 	}, // MCU_DATA_4
	{1, 0xCD, 0xEC06 	}, // MCU_DATA_5
	{1, 0xCE, 0xFD02 	}, // MCU_DATA_6
	{1, 0xCF, 0xA7FE 	}, // MCU_DATA_7
	{1, 0xC6, 0x0780 	}, // MCU_ADDRESS
	{1, 0xC8, 0x02A7 	}, // MCU_DATA_0
	{1, 0xC9, 0xEC00 	}, // MCU_DATA_1
	{1, 0xCA, 0xFD02 	}, // MCU_DATA_2
	{1, 0xCB, 0xA930 	}, // MCU_DATA_3
	{1, 0xCC, 0x6F04 	}, // MCU_DATA_4
	{1, 0xCD, 0xE604 	}, // MCU_DATA_5
	{1, 0xCE, 0x4F05 	}, // MCU_DATA_6
	{1, 0xCF, 0xF302 	}, // MCU_DATA_7
	{1, 0xC6, 0x0790 	}, // MCU_ADDRESS
	{1, 0xC8, 0xA98F 	}, // MCU_DATA_0
	{1, 0xC9, 0xEC00 	}, // MCU_DATA_1
	{1, 0xCA, 0x30ED 	}, // MCU_DATA_2
	{1, 0xCB, 0x00E6 	}, // MCU_DATA_3
	{1, 0xCC, 0x044F 	}, // MCU_DATA_4
	{1, 0xCD, 0x05E3 	}, // MCU_DATA_5
	{1, 0xCE, 0x0218 	}, // MCU_DATA_6
	{1, 0xCF, 0x8FEC 	}, // MCU_DATA_7
	{1, 0xC6, 0x07A0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x0018 	}, // MCU_DATA_0
	{1, 0xC9, 0xED00 	}, // MCU_DATA_1
	{1, 0xCA, 0x6C04 	}, // MCU_DATA_2
	{1, 0xCB, 0xE604 	}, // MCU_DATA_3
	{1, 0xCC, 0xC109 	}, // MCU_DATA_4
	{1, 0xCD, 0x25DE 	}, // MCU_DATA_5
	{1, 0xCE, 0xEE02 	}, // MCU_DATA_6
	{1, 0xCF, 0xCC05 	}, // MCU_DATA_7
	{1, 0xC6, 0x07B0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x6BED 	}, // MCU_DATA_0
	{1, 0xC9, 0x0830 	}, // MCU_DATA_1
	{1, 0xCA, 0xEE02 	}, // MCU_DATA_2
	{1, 0xCB, 0xCC04 	}, // MCU_DATA_3
	{1, 0xCC, 0x00ED 	}, // MCU_DATA_4
	{1, 0xCD, 0x0ECC 	}, // MCU_DATA_5
	{1, 0xCE, 0x02AB 	}, // MCU_DATA_6
	{1, 0xCF, 0xDD58 	}, // MCU_DATA_7
	{1, 0xC6, 0x07C0 	}, // MCU_ADDRESS
	{1, 0xC8, 0x3838 	}, // MCU_DATA_0
	{1, 0xC9, 0x3139 	}, // MCU_DATA_1
	{1, 0xC6, 0x2003	}, //MON_ARG1
	{1, 0xC8, 0x0748	}, //MON_ARG1
	{1, 0xC6, 0xA002	}, //MON_CMD
	{1, 0xC8, 0x0001	}, //MON_CMD
	{EX3691_TOK_DELAY, 0x00, 0x01F4 }, //Delay = 500ms
	{1, 0xC6, 0xA361	}, //AWB_TG_MIN0
	{1, 0xC8, 0x00E2	}, //AWB_TG_MIN0
	{1, 0x1F, 0x0018 	}, // RESERVED_SOC1_1F
	{1, 0x51, 0x7F40 	}, // RESERVED_SOC1_51
	{EX3691_TOK_DELAY, 0x00, 0x3E8  }, //Delay = 1000ms
	{0, 0x33, 0x0343 	}, // RESERVED_CORE_33
	{0, 0x38, 0x0868 	}, // RESERVED_CORE_38
	{1, 0xC6, 0xA10F	}, //SEQ_RESET_LEVEL_TH
	{1, 0xC8, 0x0042	}, //SEQ_RESET_LEVEL_TH
	{1, 0x1F, 0x0020 	}, // RESERVED_SOC1_1F
	{1, 0xC6, 0xAB04	}, //HG_MAX_DLEVEL
	{1, 0xC8, 0x0008	}, //HG_MAX_DLEVEL
	{1, 0xC6, 0xA120	}, //SEQ_CAP_MODE
	{1, 0xC8, 0x0002	}, //SEQ_CAP_MODE
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0001	}, //SEQ_CMD
	{EX3691_TOK_DELAY, 0x00, 0x03E8 }, // Delay =1000ms
	{1, 0xC6, 0xA102	}, //SEQ_MODE
	{1, 0xC8, 0x001F	}, //SEQ_MODE
	{1, 0x08, 0x01FC 	}, // COLOR_PIPELINE_CONTROL
	{1, 0x08, 0x01EC 	}, // COLOR_PIPELINE_CONTROL
	{1, 0x08, 0x01FC 	}, // COLOR_PIPELINE_CONTROL
	{1, 0x36, 0x0F08 	}, // APERTURE_PARAMETERS
	{1, 0xC6, 0x270B	}, //MODE_CONFIG
	{1, 0xC8, 0x0030	}, //MODE_CONFIG, JPEG disabled for A and B
	{1, 0xC6, 0xA121	}, //SEQ_CAP_MODE
	{1, 0xC8, 0x007f	}, //SEQ_CAP_MODE (127 frames before switching to Preview)
	{0, 0x05, 0x011E 	}, // HORZ_BLANK_B
	{0, 0x06, 0x006F 	}, // VERT_BLANK_B
	{0, 0x07, 0xFE 		}, // HORZ_BLANK_A
	{0, 0x08, 19 		}, // VERT_BLANK_A
	{0, 0x20, 0x0303 	}, // READ_MODE_B (Image flip settings)
	{0, 0x21, 0x8400 	}, // READ_MODE_A (1ADC)
	{1, 0xC6, 0x2717	}, //MODE_SENSOR_X_DELAY_A
	{1, 0xC8, 792		}, //MODE_SENSOR_X_DELAY_A
	{1, 0xC6, 0x270F	}, //MODE_SENSOR_ROW_START_A
	{1, 0xC8, 0x001C	}, //MODE_SENSOR_ROW_START_A
	{1, 0xC6, 0x2711	}, //MODE_SENSOR_COL_START_A
	{1, 0xC8, 0x003C	}, //MODE_SENSOR_COL_START_A
	{1, 0xC6, 0x2713	}, //MODE_SENSOR_ROW_HEIGHT_A
	{1, 0xC8, 0x04B0	}, //MODE_SENSOR_ROW_HEIGHT_A
	{1, 0xC6, 0x2715	}, //MODE_SENSOR_COL_WIDTH_A
	{1, 0xC8, 0x0640	}, //MODE_SENSOR_COL_WIDTH_A
	{1, 0xC6, 0x2719	}, //MODE_SENSOR_ROW_SPEED_A
	{1, 0xC8, 0x0011	}, //MODE_SENSOR_ROW_SPEED_A
	{1, 0xC6, 0x2707	}, //MODE_OUTPUT_WIDTH_B
	{1, 0xC8, 0x0640	}, //MODE_OUTPUT_WIDTH_B
	{1, 0xC6, 0x2709	}, //MODE_OUTPUT_HEIGHT_B
	{1, 0xC8, 0x04B0	}, //MODE_OUTPUT_HEIGHT_B
	{1, 0xC6, 0x271B	}, //MODE_SENSOR_ROW_START_B
	{1, 0xC8, 0x001C	}, //MODE_SENSOR_ROW_START_B
	{1, 0xC6, 0x271D	}, //MODE_SENSOR_COL_START_B
	{1, 0xC8, 0x003C	}, //MODE_SENSOR_COL_START_B
	{1, 0xC6, 0x271F	}, //MODE_SENSOR_ROW_HEIGHT_B
	{1, 0xC8, 0x04B0	}, //MODE_SENSOR_ROW_HEIGHT_B
	{1, 0xC6, 0x2721	}, //MODE_SENSOR_COL_WIDTH_B
	{1, 0xC8, 0x0640	}, //MODE_SENSOR_COL_WIDTH_B
	{1, 0xC6, 0x2723	}, //MODE_SENSOR_X_DELAY_B
	{1, 0xC8, 0x0716	}, //MODE_SENSOR_X_DELAY_B
	{1, 0xC6, 0x2725	}, //MODE_SENSOR_ROW_SPEED_B
	{1, 0xC8, 0x0011	}, //MODE_SENSOR_ROW_SPEED_B
	//Maximum Slew-Rate on IO-Pads (for Mode A)
	{1, 0xC6, 0x276B	}, //MODE_FIFO_CONF0_A
	{1, 0xC8, 0x0027	}, //MODE_FIFO_CONF0_A
	{1, 0xC6, 0x276D	}, //MODE_FIFO_CONF1_A
	{1, 0xC8, 0xE1E1	}, //MODE_FIFO_CONF1_A
	{1, 0xC6, 0xA76F	}, //MODE_FIFO_CONF2_A
	{1, 0xC8, 0x00E1	}, //MODE_FIFO_CONF2_A
	//Maximum Slew-Rate on IO-Pads (for Mode B)
	{1, 0xC6, 0x2772	}, //MODE_FIFO_CONF0_B
	{1, 0xC8, 0x0027	}, //MODE_FIFO_CONF0_B
	{1, 0xC6, 0x2774	}, //MODE_FIFO_CONF1_B
	{1, 0xC8, 0xE1E1	}, //MODE_FIFO_CONF1_B
	{1, 0xC6, 0xA776	}, //MODE_FIFO_CONF2_B
	{1, 0xC8, 0x00E1	}, //MODE_FIFO_CONF2_B
	//Set maximum integration time to get a minimum of 15 fps at 45MHz
	{1, 0xC6, 0xA20E	}, //AE_MAX_INDEX
	{1, 0xC8, 0x0004	}, //AE_MAX_INDEX
	//Set minimum integration time to get a maximum of 15 fps at 45MHz
	{1, 0xC6, 0xA20D	}, //AE_MAX_INDEX
	{1, 0xC8, 0x0004	}, //AE_MAX_INDEX
	// Configue all GPIO for output and set low to save power
	{1, 0xC6, 0x9078	}, 
	{1, 0xC8, 0x0000	},
	{1, 0xC6, 0x9079	}, 
	{1, 0xC8, 0x0000	},
	{1, 0xC6, 0x9070	}, 
	{1, 0xC8, 0x0000	},
	{1, 0xC6, 0x9071	}, 
	{1, 0xC8, 0x0000	},
	// gamma and contrast
	{1, 0xC6, 0xA743	}, // MODE_GAM_CONT_A
	{1, 0xC8, 0x0003	}, // MODE_GAM_CONT_A
	{1, 0xC6, 0xA744	}, // MODE_GAM_CONT_B
	{1, 0xC8, 0x0003	}, // MODE_GAM_CONT_B
	{EX3691_TOK_DELAY, 0x00, 0x01F4 }, // Delay =500m
	{EX3691_TOK_TERM, 0, 0 }
};
/* I/p freq = 24MHz, O/p freq = 40MHz, Slew rate = 7
 * This translates to a frame rate of 15fps. 
*/
#define INDEX_PLL_M		3
#define MASK_PLL_M		0x00FF
#define SHIFT_PLL_M		8
static struct ex3691_reg default_freq_list[]= {
	{0, 0x65, 0xA000 	},  // Disable PLL
	{0, 0x65, 0xE000 	},  // Power DOWN PLL
	{EX3691_TOK_DELAY, 0x00, 0x01F4 }, 	// Delay =500ms
	{0, 0x66, 0x500b	},  // Set value in PLL_REG for i/p 24MHz, and o/p 40MHz
	{0, 0x67, 0x0201 	},  // Set value in PLL2_REG 
	{0, 0x65, 0xA000 	},  // Power-up PLL
	{EX3691_TOK_DELAY, 0x00, 0x01F4 }, // Delay =500ms
	{0, 0x65, 0x2000  	}, //Enable PLL
	{EX3691_TOK_DELAY, 0x00, 0x01F4 }, // Delay =500ms
	{EX3691_TOK_TERM, 0, 0 }
};

#define INDEX_SIZE_WIDTH		1
#define INDEX_SIZE_HEIGHT		3
/* Default context A image size QCIF (for preview and streaming) */
static struct ex3691_reg default_size_a_list[]= 
{                                                  
	{1, 0xC6, 0x2703	}, //MODE_OUTPUT_WIDTH_A
	{1, 0xC8, 0x00B0	}, //MODE_OUTPUT_WIDTH_A
	{1, 0xC6, 0x2705	}, //MODE_OUTPUT_HEIGHT_A
	{1, 0xC8, 0x0090	}, //MODE_OUTPUT_HEIGHT_A
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD
	{EX3691_TOK_TERM, 0, 0 }
};
/* Default context B image size QVGA (for still image capturing */
static struct ex3691_reg default_size_b_list[]= 
{                                                  
	{1, 0xC6, 0x2707	}, //MODE_OUTPUT_WIDTH_B
	{1, 0xC8, 0x0140	}, //MODE_OUTPUT_WIDTH_B
	{1, 0xC6, 0x2709	}, //MODE_OUTPUT_HEIGHT_B
	{1, 0xC8, 0x00F0	}, //MODE_OUTPUT_HEIGHT_B
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD
	{EX3691_TOK_TERM, 0, 0 }
};

const static struct ex3691_reg context_a_list[]= {
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0001	}, //SEQ_CMD, Do Preview
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg context_b_list[]= {
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0002	}, //SEQ_CMD, Do capture
	{EX3691_TOK_TERM, 0, 0 }
};

const static struct ex3691_reg a_rgb565_list[]= {
	{1, 0x09, 0x2	}, //Disable factory bypass
	{1, 0xC6, 0xA77D	}, //MODE_OUTPUT_FORMAT_A
	{1, 0xC8, 0x0020	}, //MODE_OUTPUT_FORMAT_A; RGB565
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg a_yuyv_list[]= {
	{1, 0x09, 0x2	}, //Disable factory bypass
	{1, 0xC6, 0xA77D	}, //MODE_OUTPUT_FORMAT_A
	{1, 0xC8, 0x0002	}, //MODE_OUTPUT_FORMAT_A; Y-Cb-Y-Cr swap y & c
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg a_rgb555_list[]= {
	{1, 0x09, 0x2	}, //Disable factory bypass
	{1, 0xC6, 0xA77D	}, //MODE_OUTPUT_FORMAT_A
	{1, 0xC8, 0x0060	}, //MODE_OUTPUT_FORMAT_A
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg a_bayer10_list[]= {
	{1, 0x09, 0x0	}, //Factory bypass 10-bit sensor
	{EX3691_TOK_TERM, 0, 0 }
};

const static 
struct ex3691_reg *ex3691_setup_a_format[NUM_PIXEL_FORMATS] =
{
	a_rgb565_list,
	a_rgb565_list,
	a_rgb555_list,
	a_rgb555_list,
	a_yuyv_list,
	a_yuyv_list,
	a_bayer10_list
};
const static struct ex3691_reg b_rgb565_list[]= {
	{1, 0x09, 0x2	}, //Disable factory bypass
	{1, 0xC6, 0xA77E	}, //MODE_OUTPUT_FORMAT_B
	{1, 0xC8, 0x0020	}, //MODE_OUTPUT_FORMAT_B; RGB565
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg b_yuyv_list[]= {
	{1, 0x09, 0x2	}, //Disable factory bypass
	{1, 0xC6, 0xA77E	}, //MODE_OUTPUT_FORMAT_B
	{1, 0xC8, 0x0002	}, //MODE_OUTPUT_FORMAT_B; Y-Cb-Y-Cr
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg b_rgb555_list[]= {
	{1, 0x09, 0x2	}, //Disable factory bypass
	{1, 0xC6, 0xA77E	}, //MODE_OUTPUT_FORMAT_B
	{1, 0xC8, 0x0060	}, //MODE_OUTPUT_FORMAT_B
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};

const static struct ex3691_reg b_bayer10_list[]= {
	{1, 0x09, 0x0	}, //Factory bypass 10-bit sensor
	{EX3691_TOK_TERM, 0, 0 }
};

const static 
struct ex3691_reg *ex3691_setup_b_format[NUM_PIXEL_FORMATS] =
{
	b_rgb565_list,
	b_rgb565_list,
	b_rgb555_list,
	b_rgb555_list,
	b_yuyv_list,
	b_yuyv_list,
	b_bayer10_list
};

const static struct ex3691_reg enter_standby_list[]= {
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0003	}, //SEQ_CMD, standby
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg bypass_pll_list[]= {
	{0, 0x65, 0xA000 	},  // Disable PLL
	{EX3691_TOK_DELAY, 0x00, 0x01F4 }, // Delay =500ms
	{EX3691_TOK_TERM, 0, 0 }
};
const static struct ex3691_reg enable_pll_list[]= {
	{0, 0x65, 0x2000 	},  // Enable PLL
	{EX3691_TOK_DELAY, 0x00, 0x01F4 }, // Delay =500ms
	{EX3691_TOK_TERM, 0, 0 }
};

#define INDEX_SWAP_ENABLE	1
#define MASK_SWAP_ENABLE	0x7F
#define SHIFT_SWAP_ENABLE	7
static struct ex3691_reg a_default_swapbyte_list[]= {
	{1, 0xC6, 0x276B	}, //MODE_FIFO_CONF0_A
	{1, 0xC8, 0x0027	}, //MODE_FIFO_CONF0_A
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};
static struct ex3691_reg b_default_swapbyte_list[]= {
	{1, 0xC6, 0x2772	}, //MODE_FIFO_CONF0_B
	{1, 0xC8, 0x0027	}, //MODE_FIFO_CONF0_B
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};

#define MIN_BRIGHTNESS 		0
#define MAX_BRIGHTNESS 		15
#define DEF_BRIGHTNESS 		8
#define BRIGHTNESS_STEP 		1
#define INDEX_BRIGHTNESS		1
static struct ex3691_reg default_brightness_list[]= {
	{1, 0xC6, 0xA206	}, //AE_TARGET
	{1, 0xC8, DEF_BRIGHTNESS*4+10}, //AE_TARGET
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};

static struct ex3691_reg default_mode_list[]= {
	{1, 0xC6, 0xA137	}, //AE_MODE
	{1, 0xC8, 0x1}, //AE_FAST 
	{1, 0xC6, 0xA139	}, //AWB_MODE
	{1, 0xC8, 0x1}, //AWB_FAST
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};

#define GAM_TABLE_START_A		0xA745
#define GAM_TABLE_START_B		0xA758
#define GAM_TABLE_LENGTH		19
#define MIN_CONTRAST	 		0
#define MAX_CONTRAST	 		15
#define DEF_CONTRAST 			5
#define CONTRAST_STEP	 		1
#define CONTARST_LEVEL			(MAX_CONTRAST - MIN_CONTRAST + 1)
static struct ex3691_reg a_default_contrast_list[]= {
	{1, 0xC6, 0xA745	}, // MODE_GAM_TABLE_A_0
	{1, 0xC8, 0x0000	}, // MODE_GAM_TABLE_A_0
	{1, 0xC6, 0xA746	}, // MODE_GAM_TABLE_A_1
	{1, 0xC8, 0x001A	}, // MODE_GAM_TABLE_A_1
	{1, 0xC6, 0xA747	}, // MODE_GAM_TABLE_A_2
	{1, 0xC8, 0x0026	}, // MODE_GAM_TABLE_A_2
	{1, 0xC6, 0xA748	}, // MODE_GAM_TABLE_A_3
	{1, 0xC8, 0x0037	}, // MODE_GAM_TABLE_A_3
	{1, 0xC6, 0xA749	}, // MODE_GAM_TABLE_A_4
	{1, 0xC8, 0x0051	}, // MODE_GAM_TABLE_A_4
	{1, 0xC6, 0xA74A	}, // MODE_GAM_TABLE_A_5
	{1, 0xC8, 0x0066	}, // MODE_GAM_TABLE_A_5
	{1, 0xC6, 0xA74B	}, // MODE_GAM_TABLE_A_6
	{1, 0xC8, 0x0077	}, // MODE_GAM_TABLE_A_6
	{1, 0xC6, 0xA74C	}, // MODE_GAM_TABLE_A_7
	{1, 0xC8, 0x0086	}, // MODE_GAM_TABLE_A_7
	{1, 0xC6, 0xA74D	}, // MODE_GAM_TABLE_A_8
	{1, 0xC8, 0x0095	}, // MODE_GAM_TABLE_A_8
	{1, 0xC6, 0xA74E	}, // MODE_GAM_TABLE_A_9
	{1, 0xC8, 0x00A2	}, // MODE_GAM_TABLE_A_9
	{1, 0xC6, 0xA74F	}, // MODE_GAM_TABLE_A_10
	{1, 0xC8, 0x00AE	}, // MODE_GAM_TABLE_A_10
	{1, 0xC6, 0xA750	}, // MODE_GAM_TABLE_A_11
	{1, 0xC8, 0x00BA	}, // MODE_GAM_TABLE_A_11
	{1, 0xC6, 0xA751	}, // MODE_GAM_TABLE_A_12
	{1, 0xC8, 0x00C5	}, // MODE_GAM_TABLE_A_12
	{1, 0xC6, 0xA752	}, // MODE_GAM_TABLE_A_13
	{1, 0xC8, 0x00D0	}, // MODE_GAM_TABLE_A_13
	{1, 0xC6, 0xA753	}, // MODE_GAM_TABLE_A_14
	{1, 0xC8, 0x00DA	}, // MODE_GAM_TABLE_A_14
	{1, 0xC6, 0xA754	}, // MODE_GAM_TABLE_A_15
	{1, 0xC8, 0x00E3	}, // MODE_GAM_TABLE_A_15
	{1, 0xC6, 0xA755	}, // MODE_GAM_TABLE_A_16
	{1, 0xC8, 0x00ED	}, // MODE_GAM_TABLE_A_16
	{1, 0xC6, 0xA756	}, // MODE_GAM_TABLE_A_17
	{1, 0xC8, 0x00F6	}, // MODE_GAM_TABLE_A_17
	{1, 0xC6, 0xA757	}, // MODE_GAM_TABLE_A_18
	{1, 0xC8, 0x00FF	}, // MODE_GAM_TABLE_A_18
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};

/* pre-calcuated gamma correction table for each contrast levels 0-15 */ 
static const u16 gamma_table[CONTARST_LEVEL][GAM_TABLE_LENGTH] = {
	{0x0000, 0x0030, 0x0040, 0x0054, 0x006F, 0x0083, 0x0092,
	 0x00A0, 0x00AC, 0x00B7, 0x00C1, 0x00CB, 0x00D3, 0x00DC,
	 0x00E3, 0x00EB, 0x00F2, 0x00F9, 0x00FF}, /* 0.40 */
	{0x0000, 0x0027, 0x0036, 0x0049, 0x0064, 0x0078, 0x0089,
	 0x0097, 0x00A4, 0x00B0, 0x00BB, 0x00C5, 0x00CE, 0x00D7,
	 0x00E0, 0x00E8, 0x00F0, 0x00F8, 0x00FF}, /* 0.45 */
	{0x0000, 0x0020, 0x002D, 0x0040, 0x005A, 0x006E, 0x0080,
	 0x008F, 0x009C, 0x00A9, 0x00B4, 0x00BF, 0x00CA, 0x00D3,
	 0x00DD, 0x00E6, 0x00EF, 0x00F7, 0x00FF}, /* 0.50 */
	{0x0000, 0x001A, 0x0026, 0x0037, 0x0051, 0x0066, 0x0077,
	 0x0086, 0x0095, 0x00A2, 0x00AE, 0x00BA, 0x00C5, 0x00D0,
	 0x00DA, 0x00E3, 0x00ED, 0x00F6, 0x00FF}, /* 0.55 */
	{0x0000, 0x0015, 0x0020, 0x0030, 0x0049, 0x005D, 0x006F,
	 0x007F, 0x008E, 0x009B, 0x00A8, 0x00B5, 0x00C0, 0x00CC,
	 0x00D7, 0x00E1, 0x00EB, 0x00F5, 0x00FF}, /* 0.60 */
	{0x0000, 0x0011, 0x001B, 0x002A, 0x0042, 0x0056, 0x0068,
	 0x0078, 0x0087, 0x0095, 0x00A3, 0x00AF, 0x00BC, 0x00C8,
	 0x00D4, 0x00DF, 0x00EA, 0x00F5, 0x00FF}, /* 0.65 */
	{0x0000, 0x000E, 0x0017, 0x0025, 0x003B, 0x004F, 0x0061,
	 0x0071, 0x0080, 0x008F, 0x009D, 0x00AA, 0x00B8, 0x00C4,
	 0x00D0, 0x00DD, 0x00E8, 0x00F4, 0x00FF}, /* 0.70 */
	{0x0000, 0x000B, 0x0013, 0x0020, 0x0036, 0x0049, 0x005A,
	 0x006B, 0x007A, 0x0089, 0x0098, 0x00A6, 0x00B3, 0x00C1,
	 0x00CE, 0x00DA, 0x00E7, 0x00F3, 0x00FF}, /* 0.75 */
	{0x0000, 0x0009, 0x0010, 0x001C, 0x0030, 0x0043, 0x0054,
	 0x0065, 0x0074, 0x0084, 0x0092, 0x00A1, 0x00AF, 0x00BD,
	 0x00CB, 0x00D8, 0x00E5, 0x00F2, 0x00FF}, /* 0.80 */
	{0x0000, 0x0007, 0x000D, 0x0018, 0x002C, 0x003D, 0x004E,
	 0x005F, 0x006F, 0x007E, 0x008D, 0x009C, 0x00AB, 0x00B9,
	 0x00C8, 0x00D6, 0x00E4, 0x00F1, 0x00FF}, /* 0.85 */
	{0x0000, 0x0006, 0x000B, 0x0015, 0x0027, 0x0039, 0x0049,
	 0x005A, 0x0069, 0x0079, 0x0089, 0x0098, 0x00A7, 0x00B6,
	 0x00C5, 0x00D4, 0x00E2, 0x00F1, 0x00FF}, /* 0.90 */
	{0x0000, 0x0005, 0x0009, 0x0012, 0x0023, 0x0034, 0x0044,
	 0x0054, 0x0064, 0x0074, 0x0084, 0x0094, 0x00A3, 0x00B3,
	 0x00C2, 0x00D1, 0x00E1, 0x00F0, 0x00FF}, /* 0.95 */
	{0x0000, 0x0004, 0x0008, 0x0010, 0x0020, 0x0030, 0x0040,
	 0x0050, 0x0060, 0x0070, 0x0080, 0x008F, 0x009F, 0x00AF,
	 0x00BF, 0x00CF, 0x00DF, 0x00EF, 0x00FF}, /* 1.00 */
	{0x0000, 0x0003, 0x0007, 0x000E, 0x001D, 0x002C, 0x003B,
	 0x004B, 0x005B, 0x006B, 0x007B, 0x008B, 0x009C, 0x00AC,
	 0x00BD, 0x00CD, 0x00DE, 0x00EE, 0x00FF}, /* 1.05 */
	{0x0000, 0x0003, 0x0006, 0x000C, 0x001A, 0x0028, 0x0037,
	 0x0047, 0x0057, 0x0067, 0x0077, 0x0087, 0x0098, 0x00A9,
	 0x00BA, 0x00CB, 0x00DC, 0x00EE, 0x00FF}, /* 1.10 */
	{0x0000, 0x0002, 0x0005, 0x000B, 0x0017, 0x0025, 0x0034,
	 0x0043, 0x0053, 0x0063, 0x0073, 0x0084, 0x0095, 0x00A6,
	 0x00B7, 0x00C9, 0x00DB, 0x00ED, 0x00FF}, /* 1.15 */
}; 

#define INDEX_CROP_X0		1
#define INDEX_CROP_X1		3
#define INDEX_CROP_Y0		5
#define INDEX_CROP_Y1		7
static struct ex3691_reg a_crop_list[]= {
	{1, 0xC6, 0x2727	}, //MODE_CROP_X0_A
	{1, 0xC8, 0x0000	}, //MODE_CROP_X0_A
	{1, 0xC6, 0x2729	}, //MODE_CROP_X1_A
	{1, 0xC8, SVGA_WIDTH	}, //MODE_CROP_X1_A
	{1, 0xC6, 0x272B	}, //MODE_CROP_Y0_A
	{1, 0xC8, 0x0000	}, //MODE_CROP_Y0_A
	{1, 0xC6, 0x272D	}, //MODE_CROP_Y1_A
	{1, 0xC8, SVGA_HEIGHT	}, //MODE_CROP_Y1_A
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};
static struct ex3691_reg b_crop_list[]= {
	{1, 0xC6, 0x2735	}, //MODE_CROP_X0_B
	{1, 0xC8, 0x0000	}, //MODE_CROP_X0_B
	{1, 0xC6, 0x2737	}, //MODE_CROP_X1_B
	{1, 0xC8, UXGA_WIDTH	}, //MODE_CROP_X1_B
	{1, 0xC6, 0x2739	}, //MODE_CROP_Y0_B
	{1, 0xC8, 0x0000	}, //MODE_CROP_Y0_B
	{1, 0xC6, 0x273B	}, //MODE_CROP_Y1_B
	{1, 0xC8, UXGA_HEIGHT	}, //MODE_CROP_Y1_B
	{1, 0xC6, 0xA103	}, //SEQ_CMD
	{1, 0xC8, 0x0005	}, //SEQ_CMD, refresh
	{EX3691_TOK_TERM, 0, 0 }
};

static struct v4l2_queryctrl control[] = {
        { 
		.id = V4L2_CID_BRIGHTNESS, 
		.type = V4L2_CTRL_TYPE_INTEGER, 
		.name = "Brightness",
		.minimum = MIN_BRIGHTNESS,
		.maximum = MAX_BRIGHTNESS,
		.step = BRIGHTNESS_STEP,
		.default_value = DEF_BRIGHTNESS,
		.flags = 0,
		.reserved = {DEF_BRIGHTNESS, 0},
	},{
		.id = V4L2_CID_CONTRAST, 
		.type = V4L2_CTRL_TYPE_INTEGER, 
		.name = "Contrast",
		.minimum = MIN_CONTRAST, 
		.maximum = MAX_CONTRAST, 
		.step = CONTRAST_STEP, 
		.default_value = DEF_CONTRAST,
		.flags = 0,
		.reserved = {DEF_CONTRAST, 0},
	}
};

#define NUM_CONTROLS (sizeof(control)/sizeof(control[0]))

static int control_val[] = {DEF_BRIGHTNESS, DEF_CONTRAST};
/* 
 * Read a 16-bit value from a register.  The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int 
ex3691_read_reg(struct i2c_client *client, u8 reg, u16 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	*data = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		mdelay(EX3691_I2C_DELAY);
		msg->len = 2;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		/* high byte comes first so need to swap */
		*val = data[1] + (data[0] << 8);
		return 0;
	}
	DPRINTK("read from offset 0x%x error %d", reg, err);
	return err;
}

/* It is observed that the SOC fails to ACK the I2C bus when the master clock
 * is higher (e.g. max 80MHz). Retry seems to be working.
 */
#define RETRY_COUNT		5

/* Write a 16-bit value to a register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int 
ex3691_write_reg(struct i2c_client *client, u8 reg, u16 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[3];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;
	
again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 3;
	msg->buf = data;
	data[0] = reg;
	/* high byte goes out first */
	data[1] = (u8) (val >> 8);
	data[2] = (u8) (val & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	DPRINTK("wrote 0x%x to offset 0x%x error %d", val, reg, err);
	if (retry <= RETRY_COUNT) {
		DPRINTK("retry ... %d", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

static int
ex3691_write_mcu_data(struct i2c_client *client, u8* mcu_data, int num)
{
	int err;
	struct i2c_msg msg[1];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;
	
again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = num;
	msg->buf = mcu_data;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	DPRINTK("wrote mcu data error=%d, [0x%x, 0x%x, 0x%x]",
			err, mcu_data[0], mcu_data[1], mcu_data[2]);
	if (retry <= RETRY_COUNT) {
		DPRINTK("retry ... %d", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
} 				

/* 
 * Read a 8/16-bit value from a firmware driver given the driver ID and the
 * var offset. It assumes logic address. 
 * The value is returned if successful, or 0xFFFF otherwise.
 */
static u16
ex3691_read_firmware_var(struct i2c_client *client, int id, int offset, int byte)
{
	u16 val;
	
	/* always use logical address */
	val = VAR_ADDRESS_TYPE_LOGIC << VAR_ADDRESS_TYPE_SHIFT;
	val |= (byte << VAR_LENGTH_SHIFT);
	val |= (id << VAR_DRV_ID_SHIFT);
	val |= (offset << VAR_ADDRESS_SHIFT);

	/* setup page pointer */
	if (ex3691_write_reg(client, REG_PAGE, PAGE_IFP1))
		return 0xffff;
	/* setup var pointer */
	if (ex3691_write_reg(client, REG_VAR_ADDR, val))
		return 0xffff;		
	DPRINTK("sent 0x%x to read var at (%d, %d, %d):", val, id, offset, byte);
	/* read the var */
	if (ex3691_read_reg(client, REG_VAR_DATA, &val))
		return 0xffff;
	DPRINTK("Got 0x%x", val);

	if (byte == VAR_LENGTH_8BIT)
		val &= 0xff; 
	return val;
}

/* Initialize a list of EX3691 registers.
 * The list of registers is terminated by EX3691_TOK_TERM.
 * Returns zero if successful, or non-zero otherwise.
 */
#define OPTIMIZE_MCP_WRITE		1
static int 
ex3691_write_regs(struct i2c_client *client, const struct ex3691_reg reglist[])
{
	int err, i;
	u8 page = 0xFF;
	u8 mcu_data[19], *p;
	const struct ex3691_reg *next = reglist;
	
	for (; next->page != EX3691_TOK_TERM; next++) {
		if (next->page == EX3691_TOK_DELAY) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout((next->val/2)/10);
			continue;
		}

		if (page != next->page) {
			/* need to select a new page */
			err = ex3691_write_reg(client, REG_PAGE, next->page);
			if (err)
				return err;
			page = next->page;
		}

#ifdef OPTIMIZE_MCP_WRITE
		if ((page == PAGE_IFP1) && (next->reg == REG_VAR_DATA)) {
			p = mcu_data;
			*p++ = REG_VAR_DATA;
			*p++ = (u8) (next->val >> 8); 
			*p++ = (u8) (next->val & 0xff);
			i = 1;
			while ((next + i)->reg == (REG_VAR_DATA + i)) {
				*p++ = (u8) ((next+i)->val >> 8);
				*p++ = (u8) ((next+i)->val & 0xff);
				i++;
			}

			/* optimize multiple MCU data write */
			err = ex3691_write_mcu_data(client, mcu_data, 2*i+1); 				
			/* adjust pointer */
			next += (i-1);
		}
		else
#endif
		{
			err = ex3691_write_reg(client, next->reg, next->val);
		}
		if (err)
			return err;
		mdelay(EX3691_I2C_DELAY);
	}
	return 0;
}

static int
wait_for_seq_state(struct i2c_client *client, int state)
{
	int i, new_state;
	
	for (i = 0; i < 1000; i++) {
		new_state = ex3691_read_firmware_var(client, SEQ_DRV_ID,
				SEQ_STATE_OFFSET, 1);
		DPRINTK("seq state %d", new_state);
		if (new_state == state)
			return 0;
		mdelay(1);
	}
	DPRINTK("timeout waiting state %d", state);
	return -EIO;
}

/* Returns the index of the requested ID from the control structure array */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = NUM_CONTROLS - 1; i >= 0; i--)
		if (control[i].id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/* Set the camera brightness level. Return 0 on success. */
static int
ex3691_set_brightness(struct i2c_client *client, int level)
{
	/* the caller passes us a valid level */
	DPRINTK("new brightness level %d", level);
	/* brightness control is context-less */
	default_brightness_list[INDEX_BRIGHTNESS].val = level*4+10;
	if (ex3691_write_regs(client, default_brightness_list))
		return -EIO;	

	/* We force a 50ms brightness setting interval. */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(50));
	return 0;
}

/* Set the camera contrast level. Return 0 on success. */
static int
ex3691_set_contrast(struct i2c_client *client, int level)
{
	int i;
	
	/* the caller passes us a valid level */
	DPRINTK("new contrast level %d", level);

	/* The SOC fireware shadows the gamma table for both contexts, so
	 * contrast control is context-based. This driver doesn't allow control
	 * contrast for individual context. Any new contrast is programmed to
	 * both contexts.
	 */
	/* get the right gamma table values */
	for (i = 0; i < GAM_TABLE_LENGTH; i++)
		a_default_contrast_list[2*i+1].val = gamma_table[level][i]; 	
	
	/* program the valus to the context A */
	for (i = 0; i < GAM_TABLE_LENGTH; i++)
		a_default_contrast_list[2*i].val =  GAM_TABLE_START_A + i; 	
	if (ex3691_write_regs(client, a_default_contrast_list))
		return -EIO;	

	/* program the values to the context B */
	for (i = 0; i < GAM_TABLE_LENGTH; i++)
		a_default_contrast_list[2*i].val =  GAM_TABLE_START_B + i; 	
	if (ex3691_write_regs(client, a_default_contrast_list))
		return -EIO;	

	/* We force a 50ms contrast setting interval. */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(50));
	return 0;
}

/* Detect if an Ex3691 is present, and if so which revision. 
 * Returns a negative error number if no device is detected, or the 
 * non-negative value of the version ID register if a device is detected.
 */
static int
ex3691_detect(struct i2c_client *client)
{
	u16 ver;
	if (!client)
		return -ENODEV;
 
	if (ex3691_write_reg(client, REG_PAGE, PAGE_SENSOR_CORE))
		return -ENODEV;
	if (ex3691_read_reg(client, REG_SENSOR_VER, &ver))
		return -ENODEV;

	if (ver != EX3691_ID) {
		/* We didn't read the values we expected, so 
		 * this must not be an EX3691.
		 */
		return -ENODEV;
	}
	return ver;
}

/* This function registers an I2C client via i2c_attach_client() for the 
 * sensor device.  If 'probe' is non-zero, then the I2C client is only 
 * registered if the device can be detected.  If 'probe' is zero, then no 
 * device detection is attempted and the I2C client is always registered.
 * Returns zero if an I2C client is successfully registered, or non-zero 
 * otherwise.
 */
static int 
ex3691_i2c_attach_client(struct i2c_adapter *adap, int addr, int probe)
{
	struct ex3691_sensor *sensor = &ex3691;
	struct i2c_client *client = &sensor->client;
	int err;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	client->addr = addr;
	i2c_set_clientdata(&sensor->client, sensor);
	client->driver = &sensor->driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	if (probe) {
		err = ex3691_detect(client);
		if (err < 0) {
			i2c_detach_client(client);
			client->adapter = NULL;
			return err;
		}
		sensor->ver = err;
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
ex3691_i2c_detach_client(struct i2c_client *client)
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
 * EX3691 sensor device is present.  If a device is detected, an I2C client 
 * is registered for it via ex3691_i2c_attach_client().  Note that we can't use 
 * the standard i2c_probe() function to look for the sensor because the OMAP 
 * I2C controller doesn't support probing.
 * Returns zero if an EX3691 device is detected and an I2C client successfully 
 * registered for it, or non-zero otherwise.
 */
static int 
ex3691_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return ex3691_i2c_attach_client(adap, EX3691_I2C_ADDR, 1);
}

/* ---------------------------------------------------------------------
 * Following are sensor interface functions implemented by 
 * EX3691 sensor driver. They are named ex3691sensor_<name>
 */
static int
ex3691sensor_power_on(void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	int err;

	/* the caller already enabled our XCLK, wait to make sure it is stable */
	udelay(100);
	/* release STANDBY line */
	board_exit_standby();
	/* wait 1ms before enable PLL per Micron Tech. Note */
	mdelay(1);

	err = ex3691_write_regs(&sensor->client, enable_pll_list);
	if (err)
		return err;
	err = ex3691_write_regs(&sensor->client, context_a_list);
	if (err)
		return err;
	err = wait_for_seq_state(&sensor->client, SEQ_STATE_PREVIEW);
	return err;
}

/* we standby the camera in this call. Based on Micron Tech. Note, the hard
 *  standby we are using should only consume maximum 100uA.
 */
static int
ex3691sensor_power_off(void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	int err;

	/* have to standby in sequencer preview mode */
	if (ex3691_read_firmware_var(&sensor->client, SEQ_DRV_ID,
		SEQ_STATE_OFFSET, 1) != SEQ_STATE_PREVIEW) {
		printk(KERN_ERR MOD_NAME
			"calling power_off while not in preview state!\n");
		return -EIO;
	}
		
	/* Issue standby command to sequencer */
	err = ex3691_write_regs(&sensor->client, enter_standby_list);
 	if (err)
 		return err;	
	/* Poll the sequencer until it enters standby state */
	err = wait_for_seq_state(&sensor->client, SEQ_STATE_STANDBY);
	if (err)
		return err;
	/* bypass PLL */
	err = ex3691_write_regs(&sensor->client, bypass_pll_list);
	if (err)
		return err;	
	
	/* assert STANDBY line */
	board_enter_standby();
	/* the caller can cut off XCLK now */
	return 0;
}

static int
ex3691sensor_cropcap(struct v4l2_cropcap *cropcap, void *priv)
{
	/* we have a unified crop coordination for both buffer types */ 
	cropcap->bounds.left = cropcap->bounds.top = 0; 	
	cropcap->bounds.width = UXGA_WIDTH;
	cropcap->bounds.height = UXGA_HEIGHT;
	cropcap->defrect = cropcap->bounds;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;
	return 0;
}

static int
ex3691sensor_get_crop(struct  v4l2_crop *crop, void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	
	if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		crop->c = sensor->crop_rect_a; 
	else if (crop->type == V4L2_BUF_TYPE_STILL_CAPTURE)
		crop->c = sensor->crop_rect_b; 
	else
		return -EINVAL;
	return 0;
}

static int
ex3691sensor_set_crop(struct  v4l2_crop *crop, void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	struct v4l2_rect *cur_rect;
	unsigned long *cur_width, *cur_height;
	struct ex3691_reg *crop_list;
	int fps, err;
	
	switch (crop->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			cur_rect = &sensor->crop_rect_a;
			cur_width = &sensor->width_a;
			cur_height = &sensor->height_a;
			crop_list = a_crop_list;
			break;
		case V4L2_BUF_TYPE_STILL_CAPTURE:
			cur_rect = &sensor->crop_rect_b;
			cur_width = &sensor->width_b;
			cur_height = &sensor->height_b;
			crop_list = b_crop_list;
			break;
		default:
			/* invalid buffer type */
			return -EINVAL;
	}

	if ((crop->c.left == cur_rect->left) &&
	    (crop->c.width == cur_rect->width) &&
	    (crop->c.top == cur_rect->top) &&
	    (crop->c.height == cur_rect->height))
		return 0;
	
	/* out of range? then return the current crop rectangle */
	if ((crop->c.left + crop->c.width) > UXGA_WIDTH ||
	    (crop->c.top + crop->c.height) > UXGA_HEIGHT) {
		crop->c = *cur_rect;
		return 0;
	}

	/* The EX3691 SOC resizer can only scale down so we need to
	 * make sure that the scale factor is no bigger than 1.
	 */
	/* need /2 for context A */
	if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		if ((crop->c.width >> 1) < *cur_width)
			crop->c.width = (*cur_width) << 1; 
		if ((crop->c.height >> 1) < *cur_height)
			crop->c.height = (*cur_height) << 1; 
	}
	else {
		if (crop->c.width < *cur_width)
			crop->c.width = *cur_width; 
		if (crop->c.height < *cur_height)
			crop->c.height = *cur_height; 
	}

	/* adjust the parameters accordingly */
	crop_list[INDEX_CROP_X0].val = crop->c.left; 
	crop_list[INDEX_CROP_X1].val = crop->c.left + crop->c.width; 
	crop_list[INDEX_CROP_Y0].val = crop->c.top; 
	crop_list[INDEX_CROP_Y1].val = crop->c.top + crop->c.height;
	/* need /2 for context A */
	if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		crop_list[INDEX_CROP_X0].val >>= 1; 
		crop_list[INDEX_CROP_X1].val >>= 1; 
		crop_list[INDEX_CROP_Y0].val >>= 1; 
		crop_list[INDEX_CROP_Y1].val >>= 1;	
	}

	/* configure the SOC resizer */
	DPRINTK("Crop comamnd sequence for %s...",
	    (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)?"A":"B");
	DPRINTK("old (%d, %d) (%d, %d), new (%d, %d) (%d, %d)",
		cur_rect->left, cur_rect->top, cur_rect->width, cur_rect->height,
		crop->c.left, crop->c.top, crop->c.width, crop->c.height);
	err = ex3691_write_regs(&sensor->client, crop_list);
	if (err)
		return err;

	/* save back */
	*cur_rect = crop->c;

	/* Setting crop too fast can cause frame out-of-sync.
	 * We force a crop setting interval of a 1.5-Context_A frames.
	 */
	fps = (sensor->mclk * EX3691_MAX_FPS) / EX3691_MAX_MCLK_MHZ;
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(3000/(fps*2)));
	return 0;
}

static int
ex3691sensor_enter_still_capture(int frames, void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	int err;

	DPRINTK("Context B comamnd sequence ...");
	err = ex3691_write_regs(&sensor->client, context_b_list);
	if (err)
		return err;

	/* wait until the seq driver state change */
	err = wait_for_seq_state(&sensor->client, SEQ_STATE_CAPTURE);
	if (err)
		return err;

	/* need a few frames delay to stablize the output */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(200));
	return 0;
}
static int
ex3691sensor_exit_still_capture(void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	int err;

	DPRINTK("Context A comamnd sequence ...");
	err = ex3691_write_regs(&sensor->client, context_a_list);
	if (err)
		return err;
	/* wait until the seq driver state change */
	err = wait_for_seq_state(&sensor->client, SEQ_STATE_PREVIEW);
	return err;
}
 
static int
ex3691sensor_query_control(struct v4l2_queryctrl *qc, void *priv)
{
	int i;

	i = find_vctrl (qc->id);
	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	if (i < 0)
		return -EINVAL;

	*qc = control[i];
	return 0;
}

static int
ex3691sensor_get_control(struct v4l2_control *vc, void *priv)
{
	struct v4l2_queryctrl *lvc;
	int i;
	
	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];	
	switch (lvc->id) {
		case V4L2_CID_BRIGHTNESS:
		case V4L2_CID_CONTRAST:
			vc->value = control_val[i];
			break;			 
		default:
			return -EINVAL;
	}	
	return 0;
}

static int
ex3691sensor_set_control(struct v4l2_control *vc, void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	struct v4l2_queryctrl *lvc;
	int val = vc->value;
	int i, err;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	if (val < lvc->minimum || val > lvc->maximum)
		return -ERANGE;

	switch (lvc->id) {
		case V4L2_CID_BRIGHTNESS:
			err = ex3691_set_brightness(client, val);
			if (err)
				return err;
			break;			 
		case V4L2_CID_CONTRAST:
			err = ex3691_set_contrast(client, val);
			if (err)
				return err;
			break;			 
		default:
			return -EINVAL;
	}
	control_val[i] = val;
	return 0;
}

/* Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int
ex3691sensor_enum_pixformat(struct v4l2_fmtdesc *fmt, void *priv)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
		break;

		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
		break;

		default:
			return -EINVAL;
	}

	fmt->flags = ex3691_formats[index].flags;
	strncpy(fmt->description, ex3691_formats[index].description, sizeof(fmt->description));
	fmt->pixelformat = ex3691_formats[index].pixelformat;

	return 0;
}

static int
fill_format_info(struct v4l2_pix_format *pix)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ex3691_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ex3691_formats[ifmt].pixelformat;
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
/* Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This 
 * ioctl is used to negotiate the image capture size and pixel format 
 * without actually making it take effect.
 * We have to make sure that the agreed image isn't bigger than the crop.
 * This is due to the fact that SOC IFP can only scale down.
 */
static int
ex3691sensor_try_format(struct v4l2_pix_format *pix, void *priv)
{
	/* max resolution for video is SVGA */
	if (pix->width > SVGA_WIDTH)
		pix->width = SVGA_WIDTH;
	if (pix->height > SVGA_HEIGHT)
		pix->height = SVGA_HEIGHT;
	/* make sure the line is a mutiple of 32-bit word */
	pix->width &= ~1;

	return fill_format_info(pix);
}
static int
ex3691sensor_try_format_still_capture(struct v4l2_pix_format *pix, void *priv)
{
	/* max resolution for still image is UXGA */
	if (pix->width > UXGA_WIDTH)
		pix->width = UXGA_WIDTH;
	if (pix->height > UXGA_HEIGHT)
		pix->height = UXGA_HEIGHT;
	/* make sure the line is a mutiple of 32-bit word */
	pix->width &= ~1;

	return fill_format_info(pix);
}

/* Given the image capture format in pix, the nominal frame period in 
 * timeperframe, calculate the required xclk frequency 
 * This sensor driver is designed to utilize on-chip PLL so the input
 * clock from the host is fixed at 24MHZ.
 */
static unsigned long
ex3691sensor_calc_xclk(struct v4l2_pix_format *pix,
			struct v4l2_fract *timeperframe, void *priv)
{
	int fps;

	if ((timeperframe->numerator == 0) 
		|| (timeperframe->denominator == 0)) {
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = EX3691_DEF_FPS;
	}
	fps = timeperframe->denominator/timeperframe->numerator; 
	if (fps < EX3691_MIN_FPS)
		fps = EX3691_MIN_FPS;
	else if (fps > EX3691_MAX_FPS)
			fps = EX3691_MAX_FPS;
	timeperframe->numerator = 1;
	timeperframe->denominator = fps;
	
	return EX3691_XCLK_HZ;
}


static int
configure_master_clock(struct ex3691_sensor *sensor, int mclk)
{
	int err;

	DPRINTK("%d MHz master clock", mclk);
	default_freq_list[INDEX_PLL_M].val &= MASK_PLL_M;
	default_freq_list[INDEX_PLL_M].val |= ((2 * mclk) << SHIFT_PLL_M);
	err = ex3691_write_regs(&sensor->client, default_freq_list);
	if (!err)
		sensor->mclk = mclk;
	return err;
}	

static enum pixel_format
get_pixel_format(struct v4l2_pix_format *pix, int *swapbyte)
{
  	enum pixel_format pfmt;

	*swapbyte = 0;
	switch (pix->pixelformat) {
		case V4L2_PIX_FMT_RGB565X:
			pfmt = RGB565X;
			break;
		case V4L2_PIX_FMT_RGB565:
			pfmt = RGB565;
			break;
		case V4L2_PIX_FMT_RGB555X:
			pfmt = RGB555X;
			break;
		case V4L2_PIX_FMT_RGB555:
			pfmt = RGB555;
			break;
		case V4L2_PIX_FMT_UYVY:
			pfmt = UYVY;
			break;
		case V4L2_PIX_FMT_SGRBG10:
			pfmt = BAYER10;
			break;
		case V4L2_PIX_FMT_YUYV:
		default:
			pfmt = YUYV;
 	}
	return pfmt;
}
/* Given a capture format in pix, the frame period in timeperframe, and
 * the xclk frequency, configure the EX3691 context A for a specified
 * image size, pixel format, and frame period.   
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period will be returned in timeperframe.
 */
static int
ex3691sensor_configure(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
  	enum pixel_format pfmt;
	int swapbyte = 0, mclk;
	int err;

	/* configure frame rate if it changes. assumes the caller already 
	   normalized the frame rate by calling calc_xclk */
	mclk = ((timeperframe->denominator/timeperframe->numerator)
		* EX3691_MAX_MCLK_MHZ) / EX3691_MAX_FPS;
	if (mclk != sensor->mclk) {
		err = configure_master_clock(sensor, mclk);
		if (err)
			return err;
	}

	/* configure image pixel format only if it changes */
	pfmt = get_pixel_format(pix, &swapbyte);

	if (sensor->pfmt_a != pfmt) { 
		DPRINTK("Context A image fmt %d ...", pfmt);
		err = ex3691_write_regs(&sensor->client, ex3691_setup_a_format[pfmt]);
		if (err)
			return err;
		sensor->pfmt_a = pfmt;
	}
 
	/* configure image size if it changes */
	if ((sensor->width_a != pix->width) || (sensor->height_a != pix->height)) {
		/* set crop to default for the new image size */
		a_crop_list[INDEX_CROP_X0].val = 0; 
		a_crop_list[INDEX_CROP_X1].val = UXGA_WIDTH >> 1; 
		a_crop_list[INDEX_CROP_Y0].val = 0; 
		a_crop_list[INDEX_CROP_Y1].val = UXGA_HEIGHT >> 1;	
		DPRINTK("Context A default Crop...");
		err = ex3691_write_regs(&sensor->client, a_crop_list);
		if (err)
			return err;
		sensor->crop_rect_a.left = sensor->crop_rect_a.top = 0; 
		sensor->crop_rect_a.width = UXGA_WIDTH;  
		sensor->crop_rect_a.height = UXGA_HEIGHT;  
		DPRINTK("Context A image size (%d, %d)...", pix->width, pix->height);
		default_size_a_list[INDEX_SIZE_WIDTH].val = pix->width;
		default_size_a_list[INDEX_SIZE_HEIGHT].val = pix->height;
		err = ex3691_write_regs(&sensor->client, default_size_a_list);
		if (err)
			return err;
		sensor->width_a = pix->width;
		sensor->height_a = pix->height;
	}

	/* configure swapbyte if it changes */
	if (sensor->swapbyte_a != swapbyte) {
		a_default_swapbyte_list[INDEX_SWAP_ENABLE].val &= MASK_SWAP_ENABLE;
		if (swapbyte) 
			a_default_swapbyte_list[INDEX_SWAP_ENABLE].val |= (1 << SHIFT_SWAP_ENABLE);			
		DPRINTK("Context A byte swap %d...", swapbyte);
		err = ex3691_write_regs(&sensor->client, a_default_swapbyte_list);
		if (err)
			return err;
		sensor->swapbyte_a = swapbyte;
	}

	return 0;
}

/* Context B configuration */
static int
ex3691sensor_configure_still_capture(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
  	enum pixel_format pfmt;
	int swapbyte = 0;
	int err;

	/* configure image pixel format only if it changes */
	pfmt = get_pixel_format(pix, &swapbyte);
	if (sensor->pfmt_b != pfmt) { 
		DPRINTK("Context B image fmt %d ...", pfmt);
		err = ex3691_write_regs(&sensor->client, ex3691_setup_b_format[pfmt]);
		if (err)
			return err;
		sensor->pfmt_b = pfmt;
	}
 
	/* configure image size if it changes */
	if ((sensor->width_b != pix->width) || (sensor->height_b != pix->height)) { 
		/* set crop to default for the new image size */
		b_crop_list[INDEX_CROP_X0].val = 0; 
		b_crop_list[INDEX_CROP_X1].val = UXGA_WIDTH; 
		b_crop_list[INDEX_CROP_Y0].val = 0; 
		b_crop_list[INDEX_CROP_Y1].val = UXGA_HEIGHT;	
		DPRINTK("Context B default Crop...");
		err = ex3691_write_regs(&sensor->client, b_crop_list);
		if (err)
			return err;
		sensor->crop_rect_b.left = sensor->crop_rect_a.top = 0; 
		sensor->crop_rect_b.width = UXGA_WIDTH;  
		sensor->crop_rect_b.height = UXGA_HEIGHT;  
		DPRINTK("Context B image size (%d, %d)...", pix->width, pix->height);
		default_size_b_list[INDEX_SIZE_WIDTH].val = pix->width;
		default_size_b_list[INDEX_SIZE_HEIGHT].val = pix->height;
		err = ex3691_write_regs(&sensor->client, default_size_b_list);
		if (err)
			return err;
		sensor->width_b = pix->width;
		sensor->height_b = pix->height;
	}

	/* configure swapbyte if it changes */
	if (sensor->swapbyte_b != swapbyte) {
		b_default_swapbyte_list[INDEX_SWAP_ENABLE].val &= MASK_SWAP_ENABLE;
		if (swapbyte) 
			b_default_swapbyte_list[INDEX_SWAP_ENABLE].val |= (1 << SHIFT_SWAP_ENABLE);			
		err = ex3691_write_regs(&sensor->client, b_default_swapbyte_list);
		if (err)
			return err;
		sensor->swapbyte_b = swapbyte;
	}
	return 0;
}

/* Prepare for the driver to exit. Balances ex3691sensor_init().
 * This function must de-initialize the sensor and its associated data 
 * structures.
 */
static int
ex3691sensor_cleanup(void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;

	if (sensor) {
		i2c_del_driver(&sensor->driver);
		board_cleanup();
 	}
	return 0;
}


/* Initialize the EX3691 SOC.
 * This routine allocates and initializes the data structure for the sensor, 
 * powers up the sensor, registers the I2C driver, sends initilization sequence,
 * and sets a default image capture format in pix for both contexts. The capture
 * format is not actually programmed into the sensor by this routine. The camera
 * I/F driver configures the sensor for the needed format through _configure API  
 * 
 * This function must return a non-NULL value to indicate that 
 * initialization is successful.
 */
static void *
ex3691sensor_init(struct v4l2_pix_format *pix, struct v4l2_pix_format *pix2)
{
	struct ex3691_sensor *sensor = &ex3691;
	struct i2c_driver *driver = &sensor->driver;
 	int err;

	if (board_init())
		return NULL;
	memset(sensor, 0, sizeof(*sensor));
 
	driver->driver.name = "EX3691 I2C";
	driver->class = I2C_CLASS_HWMON;
	driver->attach_adapter = ex3691_i2c_probe_adapter;
	driver->detach_client = ex3691_i2c_detach_client;

	err = i2c_register_driver(THIS_MODULE, driver);
	if (err) {
		printk(KERN_ERR 
			MOD_NAME "Failed to register EX3691 I2C client.\n");
		ex3691sensor_cleanup((void *)sensor);
		return NULL;
	}
	if (!sensor->client.adapter) {
		printk(KERN_WARNING 
			MOD_NAME "Failed to detect EX3691 SOC.\n");
		ex3691sensor_cleanup((void *)sensor);
		return NULL;
	}
	else {
		printk(KERN_INFO 
			MOD_NAME "SOC detected 0x%02x (Monitor 0x%02x)\n", sensor->ver, 
			ex3691_read_firmware_var(&sensor->client, MON_DRV_ID, MON_VER_OFFSET, 1));
	}
	
	DPRINTK("Initial command sequence ...");
	if (ex3691_write_regs(&sensor->client, initial_list)) {
		ex3691sensor_cleanup((void *)sensor);
		return NULL;
	} 
	DPRINTK("Configure PLL ...");
	if (configure_master_clock(sensor, EX3691_DEF_MCLK_MHZ)) {
		ex3691sensor_cleanup((void *)sensor);
		return NULL;
	}

	/* Set the default brightness and contrast */
	ex3691_set_brightness(&sensor->client, DEF_BRIGHTNESS);
	ex3691_set_contrast(&sensor->client, DEF_CONTRAST);
	ex3691_write_regs(&sensor->client, default_mode_list);

	/* Set the default crop */
	ex3691_write_regs(&sensor->client, a_crop_list);
	ex3691_write_regs(&sensor->client, b_crop_list);
	sensor->crop_rect_a.left = sensor->crop_rect_a.top = 0; 
	sensor->crop_rect_a.width = UXGA_WIDTH;  
	sensor->crop_rect_a.height = UXGA_HEIGHT;  
	sensor->crop_rect_b.left = sensor->crop_rect_b.top = 0; 
	sensor->crop_rect_b.width = UXGA_WIDTH;  
	sensor->crop_rect_b.height = UXGA_HEIGHT;  

	/* little endian for both */
	sensor->swapbyte_a = 0;
	sensor->swapbyte_b = 0;

	/* Make the default format QCIF RGB565 for context A */
	pix->width = QCIF_WIDTH;
	pix->height = QCIF_HEIGHT;
	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	ex3691sensor_try_format(pix, (void *)sensor);
	/* Make the default format QVGA RGB565 for context B */
	pix2->width = QVGA_WIDTH;
	pix2->height = QVGA_HEIGHT;
	pix2->pixelformat = V4L2_PIX_FMT_RGB565;
	ex3691sensor_try_format_still_capture(pix2, (void *)sensor);
	/* default sizes and formats not configured yet */
	sensor->pfmt_a = sensor->pfmt_b = -1;

	return (void *)sensor;
}

 
struct camera_sensor camera_sensor_if = {
	version: 	0x02,
	name:		"EX3691",
	parallel_mode:	PAR_MODE_NOBT8,
	hs_polarity:	SYNC_ACTIVE_HIGH,
	vs_polarity:	SYNC_ACTIVE_HIGH,
	image_swap:	0,
	init: 		ex3691sensor_init,
	cleanup:	ex3691sensor_cleanup,
	power_on:	ex3691sensor_power_on,
	power_off:	ex3691sensor_power_off, 
	enum_pixformat:	ex3691sensor_enum_pixformat,
	try_format:	ex3691sensor_try_format,
	calc_xclk:	ex3691sensor_calc_xclk,
	configure:	ex3691sensor_configure,
	query_control:	ex3691sensor_query_control, 
	get_control:	ex3691sensor_get_control, 
	set_control:	ex3691sensor_set_control,
	/* we allow capturing the cropped images so we implement the following
	 * optional crop functions.
	 */
	cropcap:	ex3691sensor_cropcap,
	get_crop:	ex3691sensor_get_crop,
	set_crop:	ex3691sensor_set_crop,
	/* we support dual-context so we implement the following optional
	 * functions that are exclusively for the still image context.
	 */ 
	try_format_still_capture:	ex3691sensor_try_format_still_capture,
	configure_still_capture:	ex3691sensor_configure_still_capture,
	enter_still_capture: 		ex3691sensor_enter_still_capture,
	exit_still_capture:		ex3691sensor_exit_still_capture,
};

#ifdef CONFIG_MACH_OMAP_2430SDP
EXPORT_SYMBOL(camera_sensor_if);
#elif defined(CONFIG_MACH_OMAP_3430SDP)
static int __init
ex3691sensor_register(void)
{
	int err;
	
	err = omap_cam_register_sensor(&camera_sensor_if);
	if (err) {
		printk(KERN_WARNING MOD_NAME 
			" Failed to register sensor.\n");
		omap_cam_unregister_sensor(&camera_sensor_if);
	}

	return err;
}

static void
ex3691sensor_unregister(void)
{
	omap_cam_unregister_sensor(&camera_sensor_if);
}

late_initcall(ex3691sensor_register);
module_exit(ex3691sensor_unregister);
#endif

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("EX3691 Sensor Driver");
MODULE_LICENSE("GPL");

#ifdef EX3691_REG_DUMP
static struct ex3691_reg_read {
	u8 page;
	u8 reg;
} read_reg_list[] = {
	{0, 0x01},   // Row Start
	{0, 0x02},   // Col Start
	{0, 0x03},   // Row Width
	{0, 0x04},   // Col Width
	{0, 0x05},   // H Blanking B
	{0, 0x06},   // V Blanking B
	{0, 0x07},   // H Blanking A
	{0, 0x08},   // V Blanking A
	{0, 0x05},   // H Blanking B
	{0, 0x0A},   // Row Speed
	{0, 0x0B},   // Extra Delay
	{0, 0x0D},   // Reset
	{0, 0x20},   // Read Mode B
	{0, 0x21},   // Read Mode A
	{0, 0x65},   // Clock
	{0, 0x66},   // PLL Control 1
	{0, 0x67},   // PLL Control 2
	{0, 0xF2},   // Context Control
	{1, 0x08},   // Color Pipeline Control
	{1, 0x09},   // Factory Bypass
	{1, 0x11},   // Crop x0
	{1, 0x12},   // Crop x1
	{1, 0x13},   // Crop y0
	{1, 0x14},   // Crop y1
	{1, 0x15},   // Decimator Control
	{1, 0x16},   // Weight for H decimation
	{1, 0x17},   // Weight for V decimation
	{1, 0x97},   // Output Format
	{2, 0x00},   // JPEG Control
	{2, 0x0a},   // JPEG Bypass
	{2, 0x0d},   // output configure
	{2, 0x0e},   // pclk1 & pclk2 configure
	{2, 0x0f},   // pclk3 configure
	{EX3691_TOK_TERM, 0}
};
void dump_key_ex3691_regs(void *priv)
{
	struct ex3691_sensor *sensor = (struct ex3691_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	struct ex3691_reg_read *next = read_reg_list;
	u16 val;
	u8 page = 0xFF;
	
	for (; next->page != EX3691_TOK_TERM; next++) {
		if (page != next->page) {
			/* need to select a new page */
			if (ex3691_write_reg(client, REG_PAGE, next->page))
				return;
			page = next->page;
			mdelay(EX3691_I2C_DELAY);
		}
		if (ex3691_read_reg(client, next->reg, &val))
			return;
		printk("Page %d, Reg0x%x = 0x%x\n", next->page, next->reg, val);
		mdelay(EX3691_I2C_DELAY);
	}
}
#endif

#endif	/* ifdef CONFIG_VIDEO_OMAP24XX_CAMERA_EX3691 */






