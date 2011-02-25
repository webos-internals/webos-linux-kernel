/*
 * drivers/media/video/omap/sensor_mt9p012.c
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
 * Micron MT9P012 camera imager driver for OMAP3430SDP. It implements
 * the sensor interface defined in sensor_if.h.
 *
 * August 2007 - First version.
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
#include "mt9p012.h"

#ifdef CONFIG_VIDEO_OMAP_SENSOR_MT9P012
#define MOD_NAME "MT9P012: "

#undef MT9P012_DEBUG
#undef MT9P012_REG_DUMP

#ifdef MT9P012_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ERR MOD_NAME format "\n", ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#ifdef CONFIG_VIDEO_OMAP_AF_D88
#include "d88.h"
#endif

void dump_key_mt9p012_regs(void *priv);
static struct camera_sensor camera_sensor_if;

#define I2C_RETRY_COUNT		5

#define VIDEO_MODE		0
#define IMAGE_MODE		1
/* Test pattern */
#define TST_PAT			0x00	/* 0x02 for color bars */
/* we always ask for 12MHz xclk */
#define MT9P012_XCLK_HZ		12000000
#define MT9P012_24M_XCLZ_HZ	24000000
/* fps supported */
#define MT9P012_MIN_FPS		11 /* Limited by internal VCO, min 384 MHz*/
#define MT9P012_DEF_FPS		15
#define MT9P012_MAX_FPS		30
/* PLL values for still capture, 3 MP, @ 10 fps */
/* All are the same for 5 MP, except line_length */
#define LINE_LENGTH_3MP		5372
#define LINE_LENGTH_5MP		5372
#define PLL_DIVISOR		0x5
#define VT_PIX_CLK_DIVISOR	0x4
#define VT_SYS_CLK_DIVISOR	0x1
#define FINE_INT_TIME_DEFAULT	1200
/* 3 MP @ 10 fps for still capture */
#define MIN_EXP_TIME_CAPTURE_3MP 10
#define MAX_EXP_TIME_CAPTURE_3MP 69000
#define DEF_PIX_CLK_HZ_3MP 	 110400000
/* 5 MP @ 10 fps for still capture */
#define MIN_EXP_TIME_CAPTURE_5MP 10
#define MAX_EXP_TIME_CAPTURE_5MP 66000
#define DEF_PIX_CLK_HZ_5MP 	 110400000
/* Default coarse integration times to get a good exposure */
#define COARSE_INT_TIME_216	550
#define COARSE_INT_TIME_648	550
#define COARSE_INT_TIME_216_30FPS	1350
#define COARSE_INT_TIME_648_30FPS	1350
#define COARSE_INT_TIME_1296		1000
#define COARSE_INT_TIME_3MP		1700	/* max: frame length - 1 */
#define COARSE_INT_TIME_5MP		1700

#define COARSE_INT_TIME_INDEX	1

/* Sensor output sizes */
/* Still capture 5 MP */
#define IMAGE_WIDTH_MAX		2592
#define IMAGE_HEIGHT_MAX	1944
/* Still capture 3 MP and down to VGA, using ISP resizer */
#define IMAGE_WIDTH_MIN		2048
#define IMAGE_HEIGHT_MIN	1536
/* Video mode, for D1 NTSC, D1 PAL */
#define VIDEO_WIDTH_2X_BINN	1296
#define VIDEO_HEIGHT_2X_BINN	972
/* Video mode, for VGA, CIF, QVGA */
#define VIDEO_WIDTH_4X_BINN	648
#define VIDEO_HEIGHT_4X_BINN	486
/* (VIDEO_WIDTH_4X_BINN) x (0x10/SCALER) */
/* Video mode, for QCIF, SQCIF */
#define VIDEO_WIDTH_4X_BINN_SCALED	216
#define VIDEO_HEIGHT_4X_BINN_SCALED	162

#define QCIF_WIDTH	176
#define CIF_WIDTH	352
#define D1_WIDTH	768

/* Analog gain values */
#define MIN_GAIN	0x08
#define MAX_GAIN	0x7F

#define GAIN_INDEX	1

static unsigned long xclk_current = MT9P012_XCLK_HZ;

/* Board specific code. Currently the only supported board is OMAP3430SDP.
 * This code can be moved to a board file.
 */
#ifdef CONFIG_MACH_OMAP_3430SDP
/* The MT9P012 I2C camera chip has a fixed slave address. It changes
 * depending on SADDR is LOW or HIGH.
 * The sensor data sheet mentions the following:
 * Default i2c address is 0x6C(write) and 0x6D (read) (SADDR LOW)
 * Alternate i2c address is 0x6E(write) and 0x6F(read) (SADDR HIGH)
 * This is as per SMIA Spec.. as per i2c spec, we need to right shift 
 *   1 bit(we are indicating R/W bit separately)
 *  For MT9P012, it's address is  7 6 5 4 3 2 1 0
 *   				  0 1 1 0 1 1 0 R/W   0x6C >> 1 = 0x36
 * Default i2c Addr = 0x36
 * Alternate i2c Addr = 0x37
 */

#define MT9P012_TECHWIN_I2C_ADDR	0x10
#define MT9P012_GOLDRUSH_I2C_ADDR	0x36


#define CAMERA_RESET_GPIO  	98	/* for SDP ES2.0 */
#define CAMERA_STANDBY_GPIO	58

#define VAUX_2_8_V		0x09
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

#define DEBUG_BASE		0x08000000
#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

static void __iomem *fpga_map_addr;

static void
enable_fpga_vio_1v8(u8 enable)
{
	u16 reg_val;

  	fpga_map_addr = ioremap(DEBUG_BASE, 4096);
	reg_val = readw(fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
	DPRINTK("ioremap address is %p value is %x\n", fpga_map_addr, reg_val);

	/* Ensure that the SPR_GPIO1_3v3 is 0 - powered off.. 1 is on */
	if (reg_val & FPGA_SPR_GPIO1_3v3) {
		reg_val |= FPGA_SPR_GPIO1_3v3;
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
	/* Request and configure gpio pins */
	if (omap_request_gpio(CAMERA_STANDBY_GPIO) != 0) {
		printk("Could not request GPIO %d for AF D88\n",
			CAMERA_STANDBY_GPIO);
		return -EIO;
	}

	/* Request and configure gpio pins */
	if (omap_request_gpio(CAMERA_RESET_GPIO) != 0)
		return -EIO;

	/* STANDBY_GPIO is active HIGH for set LOW to release */
	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 1);
	
	/* nRESET is active LOW. set HIGH to release reset */
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1);

	/* set to output mode */
	omap_set_gpio_direction(CAMERA_STANDBY_GPIO, GPIO_DIR_OUTPUT);
	/* set to output mode */
	omap_set_gpio_direction(CAMERA_RESET_GPIO, GPIO_DIR_OUTPUT);

	/* turn on digital power */
  	enable_fpga_vio_1v8(1);
#ifdef CONFIG_TWL4030_CORE
	/* turn on analog power */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX_DEV_GRP_P1,TWL4030_VAUX2_DEV_GRP);
#else
#error "no power companion board defined!"
#endif

	omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, 0);

	udelay(1000);
	
	/* have to put sensor to reset to guarantee detection */
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 0);

	udelay(1500);

	/* nRESET is active LOW. set HIGH to release reset */
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1);
	/* give sensor sometime to get out of the reset. Datasheet says
	   2400 xclks. At 6 MHz, 400 usec are enough */
	udelay(300);

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
	omap_free_gpio(CAMERA_STANDBY_GPIO);
}
#endif  /* CONFIG_MACH_OMAP_3430SDP */


enum mt9p012_pll_type {
	PLL_5MP = 0,
	PLL_3MP,
	PLL_1296_15FPS,
	PLL_1296_30FPS,
	PLL_648_15FPS,
	PLL_648_30FPS,
	PLL_216_15FPS,
	PLL_216_30FPS
};

/* define a structure for sensor pll different values */
static struct mt9p012_pll_settings {
	u16	vt_pix_clk_div;
	u16	vt_sys_clk_div;
	u16	pre_pll_div;
	u16	pll_mult;
	u16	op_pix_clk_div;
	u16	op_sys_clk_div;

	u16	out_width;
	u16	out_height;
	u16	x_st;
	u16	y_st;
	u16	x_end;
	u16	y_end;

	u16	rd_md;
	u16	fine_int_tm;
	u16	frame_lines;
	u16	line_len;
	u16	coar_int_tm;
	
	u16	min_pll;
	u16	max_pll;
} all_pll_settings[] = {
	/* PLL_5MP */
	{4, 1, 5, 184,  8, 1,
	   IMAGE_WIDTH_MAX, IMAGE_HEIGHT_MAX,
	   8,   8, 2599, 1951, 0x0024,  882, 2056, 5372,
	   COARSE_INT_TIME_5MP, 160, 200},
	/* PLL_3MP */
	{4, 1, 5, 184,  8, 1,
	   IMAGE_WIDTH_MIN, IMAGE_HEIGHT_MIN,
	   8,   8, 2599, 1951, 0x0024,  882, 2056, 5372,
	   COARSE_INT_TIME_3MP, 160, 200},
	/* PLL_1296_15FPS */
	{5, 2, 3, 134, 10, 1,
	   VIDEO_WIDTH_2X_BINN, VIDEO_HEIGHT_2X_BINN,
	   8,   8, 2597, 1949, 0x046C, 1794, 1061, 3360,
	   COARSE_INT_TIME_1296, 96, 190},
	/* PLL_1296_30FPS */
	{5, 1, 3, 134, 10, 1,
	   VIDEO_WIDTH_2X_BINN, VIDEO_HEIGHT_2X_BINN,
	   8,   8, 2597, 1949, 0x046C, 1794, 1061, 3360,
	   COARSE_INT_TIME_1296, 96, 150},
	/* PLL_648_15FPS */
	{8, 2, 2, 126,  8, 2,
	   VIDEO_WIDTH_4X_BINN, VIDEO_HEIGHT_4X_BINN,
	   8,   8, 2593, 1945, 0x04FC, 1794,  574, 2712,
	   COARSE_INT_TIME_648, 92, 128},
	/* PLL_648_30FPS */
	{5, 2, 3, 192,  10, 2,
	   VIDEO_WIDTH_4X_BINN, VIDEO_HEIGHT_4X_BINN,
	   8,   8, 2593, 1945, 0x04FC, 1794,  1374, 3712,
	   COARSE_INT_TIME_648_30FPS, 96, 192},
	/* PLL_216_15FPS */
	{8, 2, 2, 126,  8, 2,
	   VIDEO_WIDTH_4X_BINN_SCALED, VIDEO_HEIGHT_4X_BINN_SCALED,
	   8,   8, 2593, 1945, 0x04FC, 1794,  574, 2712,
	   COARSE_INT_TIME_216, 92, 126},
	/* PLL_216_30FPS */
	{5, 2, 3, 192,  10, 2,
	   VIDEO_WIDTH_4X_BINN_SCALED, VIDEO_HEIGHT_4X_BINN_SCALED,
	   8,   8, 2593, 1945, 0x04FC, 1794,  1374, 3712,
	   COARSE_INT_TIME_216_30FPS, 96, 192}
};

static enum mt9p012_pll_type current_pll_video;
static enum mt9p012_pll_type current_pll_image;

#define MT9P012_I2C_DELAY	3  /* msec delay b/w accesses */

/* we keep 2 contexts, one for video mode and one for image mode */
struct mt9p012_sensor {
	struct i2c_client client;
	struct i2c_driver driver;

	/* model_id and mfr_id are used to detect the sensor */
	u16 model_id;
	u8  mfr_id;
	u8  revision;

	int mclk; 	/* sensor master clock in MHz */

	int scaler;
	unsigned long width_a;
	unsigned long height_a;
	struct v4l2_rect crop_rect_a;

	unsigned long width_b;
	unsigned long height_b;
	struct v4l2_rect crop_rect_b;
} mt9p012;

/* define a structure for sensor register initialization values */
struct mt9p012_reg {
	u16	length;
	u16 	reg;    /* 16-bit offset */
	u32 	val;	/* 8/16/32-bit value */
};

/* MT9P012 has 8/16/32 registers */
#define MT9P012_8BIT		1
#define MT9P012_16BIT		2
#define MT9P012_32BIT		4

#define MT9P012_TOK_TERM 	0xFF	/* terminating token for reg list */
#define MT9P012_TOK_DELAY	100	/* delay token for reg list */

const static struct mt9p012_reg initial_list[]= {
	{MT9P012_8BIT, REG_SOFTWARE_RESET, 0x01},
	{MT9P012_TOK_DELAY, 0x00, 5}, /* Delay = 5ms, min 2400 xcks */
	{MT9P012_16BIT, REG_RESET_REGISTER, 0x10C8},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_ANALOG_GAIN_GREENR, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_RED, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_BLUE, 0x0020},
	{MT9P012_16BIT, REG_ANALOG_GAIN_GREENB, 0x0020},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_GREENR, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_RED, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_BLUE, 0x0100},
	{MT9P012_16BIT, REG_DIGITAL_GAIN_GREENB, 0x0100},
	/* Recommended values for image quality, sensor Rev 1 */
	{MT9P012_16BIT, 0x3088, 0x6FFB},
	{MT9P012_16BIT, 0x308E, 0x2020},
	{MT9P012_16BIT, 0x309E, 0x4400},
	{MT9P012_16BIT, 0x30D4, 0x9080},
	{MT9P012_16BIT, 0x3126, 0x00FF},
	{MT9P012_16BIT, 0x3154, 0x1482},
	{MT9P012_16BIT, 0x3158, 0x97C7},
	{MT9P012_16BIT, 0x315A, 0x97C6},
	{MT9P012_16BIT, 0x3162, 0x074C},
	{MT9P012_16BIT, 0x3164, 0x0756},
	{MT9P012_16BIT, 0x3166, 0x0760},
	{MT9P012_16BIT, 0x316E, 0x8488},
	{MT9P012_16BIT, 0x3172, 0x0003},
	{MT9P012_16BIT, 0x30EA, 0x3F06},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update all at once */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Enters soft standby, all settings are maintained */
const static struct mt9p012_reg stream_off_list[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},
	/* Worst case: 10 fps, frame time = 100 ms. We need to wait until 
	   	       frame ends. */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_TOK_TERM, 0, 0}
};

/* Exits soft standby */
const static struct mt9p012_reg stream_on_list[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x01},
	/* Sensor datasheet says we need 1 ms to allow PLL lock */
	{MT9P012_TOK_DELAY, 0x00, 1},
	{MT9P012_TOK_TERM, 0, 0}
};

/* Video mode, 4x binning + scaling, range 8 - 15 fps */
const static struct mt9p012_reg enter_video_216_15fps[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 8},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 2},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 126},
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 8},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_4X_BINN_SCALED},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_4X_BINN_SCALED},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2593},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1945},
	{MT9P012_16BIT, REG_READ_MODE, 0x04FC},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 1794},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 574},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 2712},
	{MT9P012_16BIT, REG_SCALE_M, 0x0030}, /* 0x10/0x30 = 0.3333 */
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0002}, /* enable scaler */
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_216},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Video mode, 4x binning + scaling, range 16 - 30 fps */
const static struct mt9p012_reg enter_video_216_30fps[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 5},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 3},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 192},
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 10},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_4X_BINN},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_4X_BINN},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2593},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1945},
	{MT9P012_16BIT, REG_READ_MODE, 0x04FC},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 1794},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 1374},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 3712},
	{MT9P012_16BIT, REG_SCALE_M, 0x0030}, /* 0x10/0x30 = 0.3333 */
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0002}, /* enable scaler */
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_216_30FPS},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Video mode, 4x binning: 648 x 486, range 8 - 15 fps */
const static struct mt9p012_reg enter_video_648_15fps[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 8},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 2},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 126},
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 8},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_4X_BINN},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_4X_BINN},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2593},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1945},
	{MT9P012_16BIT, REG_READ_MODE, 0x04FC},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 1794},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 574},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 2712},
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0000},
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_648},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Video mode, 4x binning: 648 x 486, range 16 - 30 fps */
const static struct mt9p012_reg enter_video_648_30fps[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 5},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 3},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 192},
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 10},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_4X_BINN},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_4X_BINN},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2593},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1945},
	{MT9P012_16BIT, REG_READ_MODE, 0x04FC},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 1794},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 1374},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 3712},
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0000},
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_648_30FPS},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Video mode, scaler off: 1296 x 972, range  11 - 21 fps */
const static struct mt9p012_reg enter_video_1296_15fps[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 5},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 2},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 3},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 134},
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 10},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 1},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_2X_BINN},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_2X_BINN},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2597},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1949},
	{MT9P012_16BIT, REG_READ_MODE, 0x046C},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 1794},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 1061},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 3360},
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0000},
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_1296},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Video mode, scaler off: 1296 x 972, range  22 - 33 fps */
const static struct mt9p012_reg enter_video_1296_30fps[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 5},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 1},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 3},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 134},
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 10},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 1},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_2X_BINN},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_2X_BINN},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2597},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1949},
	{MT9P012_16BIT, REG_READ_MODE, 0x046C},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 1794},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 1061},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 3360},
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0000},
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_1296},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Image mode, 3 MP @ 10 fps */
const static struct mt9p012_reg enter_image_mode[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 4},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 1},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 5},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 184},  /* 10 fps */
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 8},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 1},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, IMAGE_WIDTH_MIN},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, IMAGE_HEIGHT_MIN},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2599},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1951},
	{MT9P012_16BIT, REG_READ_MODE, 0x0024},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 882},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 2056},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 5372},
	{MT9P012_16BIT, REG_SCALE_M, 0x0014}, /* 0x10/0x14 = 0.80 */
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0002}, /* enable scaler */
	{MT9P012_16BIT, REG_TEST_PATTERN, TST_PAT},
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_3MP},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Image mode, 5 MP @ 10 fps */
const static struct mt9p012_reg enter_image_mode_5MP[]= {
	{MT9P012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9P012_TOK_DELAY, 0x00, 100},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, 4},
	{MT9P012_16BIT, REG_VT_SYS_CLK_DIV, 1},
	{MT9P012_16BIT, REG_PRE_PLL_CLK_DIV, 5},
	{MT9P012_16BIT, REG_PLL_MULTIPLIER, 184},  /* 10 fps */
	{MT9P012_16BIT, REG_OP_PIX_CLK_DIV, 8},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, 1},
	{MT9P012_16BIT, REG_RESERVED_MFR_3064, 0x0805},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, IMAGE_WIDTH_MAX},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, IMAGE_HEIGHT_MAX},
	{MT9P012_16BIT, REG_X_ADDR_START, 8},
	{MT9P012_16BIT, REG_Y_ADDR_START, 8},
	{MT9P012_16BIT, REG_X_ADDR_END, 2599},
	{MT9P012_16BIT, REG_Y_ADDR_END, 1951},
	{MT9P012_16BIT, REG_READ_MODE, 0x0024},
	{MT9P012_16BIT, REG_FINE_INT_TIME, 882},
	{MT9P012_16BIT, REG_FRAME_LEN_LINES, 2056},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, 5372},
	{MT9P012_16BIT, REG_SCALE_M, 0x0000},
	{MT9P012_16BIT, REG_SCALING_MODE, 0x0000}, /* disable scaler */
	{MT9P012_16BIT, REG_COARSE_INT_TIME, COARSE_INT_TIME_5MP},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Structure which will set the exposure time */
static struct mt9p012_reg set_exposure_time[]= {
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01},
	{MT9P012_16BIT, REG_COARSE_INT_TIME, 500}, /* less than frame_lines-1 */
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* updating */
	{MT9P012_TOK_TERM, 0, 0}
};

/* Structure to set analog gain */
static struct mt9p012_reg set_analog_gain[]= {
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x01},
	{MT9P012_16BIT, REG_ANALOG_GAIN_GLOBAL, MIN_GAIN},
	{MT9P012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* updating */
	{MT9P012_TOK_TERM, 0, 0},
};

/* Structure to set frame rate */
static struct mt9p012_reg set_fps[2];

static u32 min_exposure_time;
static u32 max_exposure_time;
static u32 pix_clk_freq;
static u32 fps;

/*
 * @brief mt9p012_read_reg : Reads a 8/16/32-bit mt9p012 register.
 * 
 * @param client - i2c_client
 * @param data_length - MT9P012_8BIT, MT9P012_16BIT, MT9P012_32BIT
 * @param reg - register address
 * @param val - value read is returned in val
 *    
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012_read_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[4];
	
	if (!client->adapter)
		return -ENODEV;
	if (data_length != MT9P012_8BIT && data_length != MT9P012_16BIT
					&& data_length != MT9P012_32BIT)
		return -EINVAL;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		mdelay(MT9P012_I2C_DELAY);
		msg->len = data_length;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == MT9P012_8BIT)
			*val = data[0];
		else if (data_length == MT9P012_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	printk(KERN_ERR "read from offset 0x%x error %d", reg, err);
	return err;
}

/*
 * @brief mt9p012_write_reg : Writes a 8/16/32-bit mt9p012 register.
 * 
 * @param client - i2c_client
 * @param data_length - MT9P012_8BIT, MT9P012_16BIT, MT9P012_32BIT
 * @param reg - register address
 * @param val - value to be written
 *    
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[6];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;
	if (data_length != MT9P012_8BIT && data_length != MT9P012_16BIT
					&& data_length != MT9P012_32BIT)
		return -EINVAL;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2 + data_length;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);

	if (data_length == MT9P012_8BIT)
		data[2] = (u8) (val & 0xff);
	else if (data_length == MT9P012_16BIT){
		data[2] = (u8) (val >> 8);
		data[3] = (u8) (val & 0xff);
	} else {
		data[2] = (u8) (val >> 24);
		data[3] = (u8) (val >> 16);
		data[4] = (u8) (val >> 8);
		data[5] = (u8) (val & 0xff);
	}

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	DPRINTK("wrote 0x%x to offset 0x%x error %d", val, reg, err);
	if (retry <= I2C_RETRY_COUNT) {
		DPRINTK("retry ... %d", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

/*
 * @brief mt9p012_write_regs : Initializes a list of MT9P012 registers. The
 * 		list of registers is terminated by MT9P012_TOK_TERM.
 * 
 * @param client - i2c_client 
 * @param reglist - list of registers to be written
 *    
 * @returns - zero
 */
static int
mt9p012_write_regs(struct i2c_client *client,
		const struct mt9p012_reg reglist[])
{
	int err;
	const struct mt9p012_reg *next = reglist;

	for (; next->length != MT9P012_TOK_TERM; next++) {
		if (next->length == MT9P012_TOK_DELAY) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies(next->val));
			continue;
		}

		err = mt9p012_write_reg(client, next->length,
				next->reg, next->val);
		if (err)
			return err;
		mdelay(MT9P012_I2C_DELAY);
	}
	return 0;
}

/*
 * @brief mt9p012_switch_to_video : Change the list of MT9P012 registers to
 * 				    enter video mode. 
 *
 * @param client - i2c_client
 * @param scaler - 0, 1, 2
 *    
 * @returns - zero
 */
static int
mt9p012_switch_to_video(struct i2c_client *client, int scaler)
{
	if (scaler == 1) {
		if (current_pll_video == PLL_648_15FPS)
			mt9p012_write_regs(client, enter_video_648_15fps);
		else
			mt9p012_write_regs(client, enter_video_648_30fps);
	} else if (scaler == 2) {
		if (current_pll_video == PLL_216_15FPS)
			mt9p012_write_regs(client, enter_video_216_15fps);
		else
			mt9p012_write_regs(client, enter_video_216_30fps);	
	} else {
		if (current_pll_video == PLL_1296_15FPS)
			mt9p012_write_regs(client, enter_video_1296_15fps);
		else if (current_pll_video == PLL_1296_30FPS)
			mt9p012_write_regs(client, enter_video_1296_30fps);
		else {	/* For sizes greater than 1296 x  */
			mt9p012_write_regs(client, enter_image_mode_5MP);
			goto fps_fixed;
		}
	}
	mt9p012_write_regs(client, set_fps);
fps_fixed:
	mt9p012_write_regs(client, stream_on_list);
	return 0;
}

/*
 * @brief mt9p012_switch_to_image : Change the list of MT9P012 registers to
 * 				    enter still capture mode. 
 *
 * @param client - i2c_client
 * 
 * @returns - zero
 */
static int
mt9p012_switch_to_image(struct i2c_client *client)
{
	if (current_pll_image == PLL_3MP) {
		DPRINTK("Entering 3MP image mode...\n");
		mt9p012_write_regs(client, enter_image_mode);
	} else {
		DPRINTK("Entering 5MP image mode...\n");
		mt9p012_write_regs(client, enter_image_mode_5MP);
	}

	mt9p012_write_regs(client, stream_on_list);
	return 0;
}

/*
 * @brief mt9p012_detect : Detects if an MT9P012 is present and if so, 
 * 			   which revision.
 *
 * @param client - i2c_client
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012_detect(struct i2c_client *client)
{
	struct mt9p012_sensor *sensor = &mt9p012;
	u32 model_id, mfr_id, rev;

	DPRINTK("mt9p012_detect called\n");
	if (!client)
		return -ENODEV;

	if (mt9p012_read_reg(client, MT9P012_16BIT, REG_MODEL_ID, &model_id))
		return -ENODEV;
	if (mt9p012_read_reg(client, MT9P012_8BIT, REG_MANUFACTURER_ID,
				&mfr_id))
		return -ENODEV;
	if (mt9p012_read_reg(client, MT9P012_8BIT, REG_REVISION_NUMBER, &rev))
		return -ENODEV;

	if ((model_id != MT9P012_MOD_ID) || (mfr_id != MT9P012_MFR_ID)) {
		/* We didn't read the values we expected, so
		 * this must not be an MT9P012.
		 */
	  printk("model id mismatch 0x%x mfr 0x%x\n", model_id, mfr_id);
		return -ENODEV;
	}

	sensor->model_id = model_id;
	sensor->mfr_id = mfr_id;
	sensor->revision = rev;
	return 0;
}

/*
 * @brief mt9p012_i2c_attach_client : Registers an I2C client via
 * 			i2c_attach_client() for the sensor device. 
 * 			   which revision.
 *
 * @param adap - i2c_adapter
 * @param addr - i2c_address
 * @param probe - if non-zero, I2C client is only registered if the device can
 * 		  be detected. If zero, then no device detection is attempted
 * 		  and I2C client is alwas registered.  
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012_i2c_attach_client(struct i2c_adapter *adap, int addr, int probe)
{
	struct mt9p012_sensor *sensor = &mt9p012;
	struct i2c_client *client = &sensor->client;
	int err;

	DPRINTK("i2c attach client\n");
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
		err = mt9p012_detect(client);
		if (err < 0) {
			i2c_detach_client(client);
			client->adapter = NULL;
			return err;
		}
	}
	return 0;
}

/*
 * @brief mt9p012_i2c_detach_client : Unregisters the client via
 * 			i2c_detach_client(). It is called by i2c_del_adapter()
 * 			and i2c_del_driver().
 *
 * @param client - i2c_client
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;

	return err;
}

/*
 * @brief mt9p012_i2c_probe_adapter : This function will be called for each 
 * 		registered I2C bus adapter when our I2C driver is registered
 * 		via i2c_add_driver().  It will also be called whenever a new
 * 		I2C adapter is registered after our I2C driver is registered.
 * 		This function probes the specified I2C bus adapter to determine
 * 		if an MT9P012 sensor device is present.  If a device is
 * 		detected, an I2C client is registered for it via
 * 		mt9p012_i2c_attach_client().  Note that we can't use the 
 * 		standard i2c_probe() function to look for the sensor because
 * 		the OMAP I2C controller doesn't support probing.
 *
 * @param adap - i2c_adapter
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012_i2c_probe_adapter(struct i2c_adapter *adap)
{
	DPRINTK("i2c probe\n");
	/* Default to Techwin I2C address. If Techwin is not connected we look
	   for Goldrush */
	if(!mt9p012_i2c_attach_client(adap, MT9P012_TECHWIN_I2C_ADDR, 1))
		return 0;
	else
		return mt9p012_i2c_attach_client(adap, MT9P012_GOLDRUSH_I2C_ADDR,1);
	
}

/*
 * @brief mt9p012sensor_power_on : This function is called when resuming from
 * 				   suspend. It exits standby mode. 
 *
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012sensor_power_on(void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;
	int err;

	DPRINTK("Exit standby mode ...");
	err = mt9p012_write_regs(&sensor->client, stream_on_list);
	return err;
}

/*
 * @brief mt9p012sensor_power_off : This function is called when entering
 * 				    suspend. It enters soft standby by
 * 				    stopping streaming mode.
 *
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012sensor_power_off(void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;
	int err;

	DPRINTK("Entering standby mode ...");
	/* This is soft Standby */
	err = mt9p012_write_regs(&sensor->client, stream_off_list);
	return err;
}

/*
 * @brief mt9p012sensor_enter_still_capture : This function is called when
 * 			switching to still capture mode.
 *
 * @param frames - not used 
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012sensor_enter_still_capture(int frames, void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;
	int err;

	err = mt9p012_switch_to_image(&sensor->client);
	if (err)
		return err;

	dump_key_mt9p012_regs((void *)sensor);

	return 0;
}

/*
 * @brief mt9p012sensor_exit_still_capture : This function is called when
 * 			switching from still capture to video mode (default).
 *
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012sensor_exit_still_capture(void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;

	return mt9p012_switch_to_video(&sensor->client, sensor->scaler);
}

/*
 * @brief mt9p012sensor_try_format : Helps to implement the VIDIOC_TRY_FMT
 * 			ioctl for the CAPTURE buffer type. 
 *
 * @param pix - v4l2_pix_format structure
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero.
 */
static int
mt9p012sensor_try_format(struct v4l2_pix_format *pix, void *priv)
{
	/* we output one of three resolutions based on the expected size.
	 * camera driver should use ISP resizer to do further scaling
	 */
	if (pix->width < QCIF_WIDTH) {
		pix->width = VIDEO_WIDTH_4X_BINN_SCALED;
		pix->height = VIDEO_HEIGHT_4X_BINN_SCALED;
	} else if (pix->width <= CIF_WIDTH) {
		pix->width = VIDEO_WIDTH_4X_BINN;
		pix->height = VIDEO_HEIGHT_4X_BINN;
	} else if (pix->width <= D1_WIDTH) {
		pix->width = VIDEO_WIDTH_2X_BINN;
		pix->height = VIDEO_HEIGHT_2X_BINN;
	} else { /* For Burst Mode */
		pix->width =  IMAGE_WIDTH_MAX;
		pix->height = IMAGE_HEIGHT_MAX;	
	}

	return 0;
}

/*
 * @brief mt9p012sensor_try_format_still_capture : Helps to implement the 
 * 			VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type. 
 *
 * @param pix - v4l2_pix_format structure
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero.
 */
static int
mt9p012sensor_try_format_still_capture(struct v4l2_pix_format *pix, void *priv)
{
	/* we output one of two resolutions based on the expected size.
	 * camera driver should use ISP resizer to do further scaling
	 */
	if (pix->width > IMAGE_WIDTH_MIN) {
		pix->width = IMAGE_WIDTH_MAX;
		pix->height = IMAGE_HEIGHT_MAX;
	} else {
		pix->width =  IMAGE_WIDTH_MIN;
		pix->height = IMAGE_HEIGHT_MIN;
	}

	return 0;
}

/*
 * @brief mt9p012sensor_calc_xclk : This sensor driver is designed to utilize 
 * 				    on-chip PLL so the input clock from the
 * 				    host is fixed at 12MHZ.
 *
 * @param pix - v4l2_pix_format structure
 * @param timeperframe - parameters for fps calculation 
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - MT9P012_XCLK_HZ, fixed value @ 12 MHz.
 */
static unsigned long
mt9p012sensor_calc_xclk(struct v4l2_pix_format *pix,
			struct v4l2_fract *timeperframe, void *priv)
{
	if ((timeperframe->numerator == 0)
	|| (timeperframe->denominator == 0)){
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = MT9P012_DEF_FPS;
	}
	fps = timeperframe->denominator/timeperframe->numerator; 
	if (fps < MT9P012_MIN_FPS){
		fps = MT9P012_MIN_FPS;
	} else if (fps >MT9P012_MAX_FPS){
		fps = MT9P012_MAX_FPS;
	}
	timeperframe->numerator = 1;
	timeperframe->denominator = fps;

	if ((pix->width <= VIDEO_WIDTH_4X_BINN) && (fps > 15))
		xclk_current = MT9P012_24M_XCLZ_HZ;
	else
		xclk_current = MT9P012_XCLK_HZ;

	return xclk_current;
}

/*
 * @brief mt9p012sensor_configure : Configures sensor context A for a specified
 * 				    image size and frame rate. 
 *
 * @param pix - v4l2_pix_format structure
 * @param xclk - input clock, fixed @ 12 MHz 
 * @param timeperframe - parameters for fps calculation 
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012sensor_configure(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;
	int err = 0, row = 1;
	unsigned int vt_pix_clk;
	unsigned int pll_multiplier;
	unsigned int exposure_factor, pix_clk_scaled;
	
	fps = timeperframe->denominator / timeperframe->numerator;

	if (fps < MT9P012_MIN_FPS)
		fps = MT9P012_MAX_FPS;
	else if (fps > MT9P012_MAX_FPS)
		fps = MT9P012_MAX_FPS;
	DPRINTK("Frame rate = %d", fps);
	if (pix->width > VIDEO_WIDTH_2X_BINN) { /* Burst Mode */
		sensor->scaler = 0;
		fps = 10;
		current_pll_video = PLL_5MP;
		err = mt9p012_write_regs(&sensor->client, enter_image_mode_5MP);
		if (err) {
			printk("Error while setting video 5MP\n");
			return err;
		}
		goto skip_configure;
	} else if (pix->width > VIDEO_WIDTH_4X_BINN) {
		sensor->scaler = 0;
		if (fps > 21)
			current_pll_video = PLL_1296_30FPS;
		else
			current_pll_video = PLL_1296_15FPS;
	} else if (pix->width > VIDEO_WIDTH_4X_BINN_SCALED) {
		sensor->scaler = 1;
		if (fps > 15)
			current_pll_video = PLL_648_30FPS;
		else
			current_pll_video = PLL_648_15FPS;
	} else {
		sensor->scaler = 2;
		if (fps > 15)
			current_pll_video = PLL_216_30FPS;
		else
			current_pll_video = PLL_216_15FPS;
	}

	if (sensor->scaler && (fps < 16))
		row = 2; /* Adjustment when using 4x binning and 12 MHz clk */
	
	vt_pix_clk = fps * all_pll_settings[current_pll_video].frame_lines
			* all_pll_settings[current_pll_video].line_len;

	pll_multiplier = 
		(((vt_pix_clk
		  * all_pll_settings[current_pll_video].vt_pix_clk_div
		  * all_pll_settings[current_pll_video].vt_sys_clk_div
		  * row) / xclk_current)
		  * all_pll_settings[current_pll_video].pre_pll_div) + 1;

	if (pll_multiplier < all_pll_settings[current_pll_video].min_pll)
		pll_multiplier = all_pll_settings[current_pll_video].min_pll;
	else if (pll_multiplier > all_pll_settings[current_pll_video].max_pll)
		pll_multiplier = all_pll_settings[current_pll_video].max_pll;

	pix_clk_freq = (xclk_current /
			(all_pll_settings[current_pll_video].pre_pll_div
			 * all_pll_settings[current_pll_video].vt_pix_clk_div
			 * all_pll_settings[current_pll_video].vt_sys_clk_div
			 * row)) * pll_multiplier;
	min_exposure_time = (all_pll_settings[current_pll_video].fine_int_tm
			     * 1000000 / pix_clk_freq) + 1;
	DPRINTK("pre_pll_div = %d,\n vt_pix_clk_div = %d,\n vt_sys_clk_div = %d\n",
		all_pll_settings[current_pll_video].pre_pll_div,
		all_pll_settings[current_pll_video].vt_pix_clk_div,
		all_pll_settings[current_pll_video].vt_sys_clk_div);
	DPRINTK("pix_clk_freq %u, vt_pixclk %u, pll_mult = %u\n", pix_clk_freq, 
		vt_pix_clk, pll_multiplier);
	exposure_factor = (all_pll_settings[current_pll_video].frame_lines - 1)
				* all_pll_settings[current_pll_video].line_len;
	exposure_factor += all_pll_settings[current_pll_video].fine_int_tm;
	exposure_factor *= 100;
	pix_clk_scaled = pix_clk_freq / 100;
	max_exposure_time = (exposure_factor / pix_clk_scaled) * 100;
	DPRINTK("min_exp_time = %d;\nmax_exp_time = %d\n", 
		min_exposure_time, max_exposure_time);

	set_fps[0].length = MT9P012_16BIT;
	set_fps[0].reg = REG_PLL_MULTIPLIER;    
	set_fps[0].val = pll_multiplier;
	set_fps[1].length = MT9P012_TOK_TERM;
	set_fps[1].reg = 0;    
	set_fps[1].val = 0;

	if (pix->width <= VIDEO_WIDTH_4X_BINN_SCALED) {
		DPRINTK("Turning on scaler...");
		if (current_pll_video == PLL_216_15FPS)
			err = mt9p012_write_regs(&sensor->client,
						 enter_video_216_15fps);
		else
			err = mt9p012_write_regs(&sensor->client,
						 enter_video_216_30fps);
		if (err) {
			DPRINTK("Error turning on scaler...%d", err);
			return err;
		}
		sensor->scaler = 2;	
	} else if (pix->width <= VIDEO_WIDTH_4X_BINN) {
		DPRINTK("4x binning...");
		if (current_pll_video == PLL_648_15FPS)
			err = mt9p012_write_regs(&sensor->client,
						 enter_video_648_15fps);
		else
			err = mt9p012_write_regs(&sensor->client,
						 enter_video_648_30fps);
		if (err) {
			DPRINTK("Error setting 4x binning...%d", err);
			return err;
		}
		sensor->scaler = 1;
	} else {
		DPRINTK("2x binning...");
		if (current_pll_video == PLL_1296_15FPS)
			err = mt9p012_write_regs(&sensor->client,
						 enter_video_1296_15fps);
		else
			err = mt9p012_write_regs(&sensor->client,
						 enter_video_1296_30fps);
		if (err) {
			DPRINTK("Error setting 2x binning...%d", err);
			return err;
		}
		sensor->scaler = 0;
	}
	err = mt9p012_write_regs(&sensor->client, set_fps);
	if (err) {
		DPRINTK("Error setting frame rate...%d", err);
		return err;
	}
skip_configure:
	mdelay(20);
	err = mt9p012_write_regs(&sensor->client, stream_on_list);
	if (err) {
		DPRINTK("Error entering stream...%d", err);
		return err;
	}
	mdelay(20);
	
       	dump_key_mt9p012_regs((void *)sensor);
	return err;
}

/*
 * @brief mt9p012sensor_configure_still_capture : Configures sensor context B 
 * 				for a specified image size. 
 *
 * @param pix - v4l2_pix_format structure
 * @param xclk - input clock, fixed @ 12 MHz 
 * @param timeperframe - not used, fps fixed for still capture
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012sensor_configure_still_capture(struct v4l2_pix_format *pix,
		unsigned long xclk,struct v4l2_fract *timeperframe, void *priv)
{

	if (pix->width <= IMAGE_WIDTH_MIN)
		current_pll_image = PLL_3MP;
	else
		current_pll_image = PLL_5MP;

	return 0;
}

/*
 * @brief mt9p012sensor_set_exposure_time : API to set the sensor's exposure 
 * 					    time.
 *
 * @param mode - VIDEO_MODE or IMAGE_MODE
 * @param exp_time - exposure time from user, ~ 10 - 60,000 ns
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
int
mt9p012sensor_set_exposure_time(int mode, u32 exp_time)
{
	int err;
	struct mt9p012_sensor *sensor = &mt9p012;
	u32 coarse_int_time = 0;

	if (mode == VIDEO_MODE) {
		if((exp_time < min_exposure_time) ||
				(exp_time > max_exposure_time)) {
			printk("Exposure time not within the legal range.\n");
			printk("Exposure time must be between %d us",
					min_exposure_time);
			printk(" and %d us\n",
					max_exposure_time);
			return -EINVAL;
		}
		coarse_int_time = 
			((((exp_time / 10) * (pix_clk_freq / 1000)) / 1000)
			   - (all_pll_settings[current_pll_video].fine_int_tm
			      / 10))
			   / (all_pll_settings[current_pll_video].line_len
			      / 10);
	} else if (mode == IMAGE_MODE) {
		if (current_pll_image == PLL_3MP) {
			if((exp_time < MIN_EXP_TIME_CAPTURE_3MP) ||
					(exp_time > MAX_EXP_TIME_CAPTURE_3MP)) {
				printk("Exposure time not within "
					"the legal range.\n");
				printk("Exposure time must be between %d us",
					MIN_EXP_TIME_CAPTURE_3MP);
				printk(" and %d us\n",
					MAX_EXP_TIME_CAPTURE_3MP);
				return -EINVAL;
			}
			coarse_int_time =
				((((exp_time / 10)
				   * (DEF_PIX_CLK_HZ_3MP / 1000))
				 / 1000) - (FINE_INT_TIME_DEFAULT / 10))
				 / (LINE_LENGTH_3MP / 10);
		} else {
			if((exp_time < MIN_EXP_TIME_CAPTURE_5MP) ||
				(exp_time > MAX_EXP_TIME_CAPTURE_5MP)) {
				printk("Exposure time not within "
					"the legal range.\n");
				printk("Exposure time must be between %d us",
					MIN_EXP_TIME_CAPTURE_5MP);
				printk(" and %d us\n",
					MAX_EXP_TIME_CAPTURE_5MP);
				return -EINVAL;
			}
			coarse_int_time =
				((((exp_time / 10)
				   * (DEF_PIX_CLK_HZ_5MP / 1000))
				 / 1000) - (FINE_INT_TIME_DEFAULT / 10))
				 / (LINE_LENGTH_5MP / 10);
		}
	} else {
		DPRINTK("Err Invalid argument...");
		return -EINVAL;
	}
	DPRINTK("coarse_int_time calculated = %d\n", coarse_int_time);

	set_exposure_time[COARSE_INT_TIME_INDEX].val = coarse_int_time;
	err = mt9p012_write_regs(&sensor->client, set_exposure_time);
	if (err) {
		DPRINTK("Error setting exposure time...%d\n", err);
	}
	return err;
}

/*
 * @brief mt9p012sensor_set_exposure_time : API to set the sensor's analog gain.
 *
 * @param gain - 0x08 to 0x7F
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
int
mt9p012sensor_set_gain(u16 gain)
{
	int err;
	struct mt9p012_sensor *sensor = &mt9p012;
	
	if ((gain < MIN_GAIN) || (gain > MAX_GAIN))
	{
		DPRINTK("Gain not within the legal range...");
		return -EINVAL;
	}
	set_analog_gain[GAIN_INDEX].val = gain;
	err = mt9p012_write_regs(&sensor->client, set_analog_gain);
	if (err) {
		DPRINTK("Error setting gain...%d", err);
		return err;
	}
	return err;
}

/*
 * @brief mt9p012sensor_cleanup : Prepares for the driver to exit. Balances
 * 				   mt9p012sensor_init(). De-initialize the
 * 				   sensor and its associated data structs.  
 *
 * @param priv - mt9p012_sensor structure
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int
mt9p012sensor_cleanup(void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;
#ifdef CONFIG_VIDEO_OMAP_AF_D88
	int err = 0;

	err = d88_af_deinit();
	if (err)
		printk("Failed to deinitialize D88 AF\n");
#endif
	if (sensor) {
		i2c_del_driver(&sensor->driver);
		board_cleanup();
 	}
	return 0;
}

int mt9p012sensor_restore(void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;
	
	DPRINTK("reconfiguring the sensor\n");
	
	if (mt9p012_write_regs(&sensor->client, initial_list)) {
		mt9p012sensor_cleanup((void *)sensor);
		return -1;
	}
	mt9p012_switch_to_video(&sensor->client, sensor->scaler);
	DPRINTK("reconfiguring the sensor done\n");
	return 0;
}
EXPORT_SYMBOL(mt9p012sensor_restore);

/*
 * @brief mt9p012sensor_init : Initializes the sensor's data structures, powers
 * 			       it up, registers the I2C driver, sends init
 * 			       sequence and sets default image capture sizes in
 * 			       pix and pix2. The capture size is not actually
 * 			       programmed into the sensor by this routine.
 * 			       The camera I/F driver configures the sensor for
 * 			       the needed format through _configure API.
 *
 * @param pix - v4l2_pix_format structure for context A
 * @param pix2 - v4l2_pix_format structure for context B
 * 
 * @returns - a non-NULL value to indicate that the initialization was 
 * 	      successful.
 */
static void *
mt9p012sensor_init(struct v4l2_pix_format *pix, struct v4l2_pix_format *pix2)
{
	struct mt9p012_sensor *sensor = &mt9p012;
	struct i2c_driver *driver = &sensor->driver;
	int err;
	/* Power on sensor */
	if (board_init())
		return NULL;
	memset(sensor, 0, sizeof(*sensor));

	driver->driver.name = "MT9P012 I2C";
	driver->class = I2C_CLASS_HWMON;
	driver->attach_adapter = mt9p012_i2c_probe_adapter;
	driver->detach_client = mt9p012_i2c_detach_client;

	err = i2c_register_driver(THIS_MODULE, driver);
	if (err) {
		printk(KERN_ERR MOD_NAME
			"Failed to register MT9P012 I2C client. %d\n", err);
		board_cleanup();
		return NULL;
	}
	if (!sensor->client.adapter) {
		printk(KERN_WARNING MOD_NAME
			"Failed to detect MT9P012 sensor.\n");
		mt9p012sensor_cleanup((void *)sensor);
		return NULL;
	} else {
		printk(KERN_INFO
			MOD_NAME " detected (model_id 0x%04x, Rev 0x%02x, "
			"mfr 0x%02x)\n",sensor->model_id, sensor->revision,
			sensor->mfr_id);
	}
#ifdef CONFIG_VIDEO_OMAP_AF_D88
	err = d88_af_init(xclk_current);
	if (err)
		printk("Failed to initialize D88 AF\n");
	/* Just as a default position.
	   To do: Establish how we will use D88*/
	err = d88_af_setfocus(MID_FOCUS_POS);
	if (err)
		printk("Failed to set focus in D88\n");
#endif

	dump_key_mt9p012_regs((void *)sensor);
	DPRINTK("Initial command sequence ...");
	if (mt9p012_write_regs(&sensor->client, initial_list)) {
		mt9p012sensor_cleanup((void *)sensor);
		return NULL;
	}

	DPRINTK("Turning on the video, 4x binning + scaler by default ...");
	if (mt9p012_write_regs(&sensor->client, enter_video_216_15fps)) {
		mt9p012sensor_cleanup((void *)sensor);
		return NULL;
	}

	DPRINTK("Streaming on ...");
	if (mt9p012_write_regs(&sensor->client, stream_on_list)) {
		mt9p012sensor_cleanup((void *)sensor);
		return NULL;
	}

	sensor->scaler = 2;
	current_pll_video = PLL_216_15FPS;
	pix->width = VIDEO_WIDTH_4X_BINN_SCALED;
	pix->height = VIDEO_HEIGHT_4X_BINN_SCALED;
	pix->pixelformat = V4L2_PIX_FMT_SGRBG10;
	mt9p012sensor_try_format(pix, (void *)sensor);

	current_pll_image = PLL_3MP;
	pix2->width = IMAGE_WIDTH_MIN;
	pix2->height = IMAGE_HEIGHT_MIN;
	pix2->pixelformat = V4L2_PIX_FMT_SGRBG10;
	mt9p012sensor_try_format_still_capture(pix2, (void *)sensor);

       	dump_key_mt9p012_regs((void *)sensor);
	return (void *)sensor;
}

/*
 * @brief mt9p012sensor_unregister : Unregisters the sensor from the camera
 * 				     driver
 */
static void
mt9p012sensor_unregister(void)
{
	omap_cam_unregister_sensor(&camera_sensor_if);
}

/*
 * @brief mt9p012sensor_register : Registers the sensor with the camera
 * 				   driver.
 * 
 * @returns - zero if successful, or non-zero otherwise.
 */
static int __init
mt9p012sensor_register(void)
{
	int err;
	
	DPRINTK("Sensor register \n");
	err = omap_cam_register_sensor(&camera_sensor_if);
	if (err) {
		printk(KERN_WARNING MOD_NAME 
			" Failed to register sensor.\n");
		omap_cam_unregister_sensor(&camera_sensor_if);
	}
	return err;
}

static struct camera_sensor camera_sensor_if = {
	version: 	0x00,
	name:		"MT9P012",
	sensor_type:	SENSOR_RAW,
	sensor_interface:SENSOR_PARALLEL,
	parallel_mode:	PAR_MODE_NOBT8,
	hs_polarity:	SYNC_ACTIVE_HIGH,
	vs_polarity:	SYNC_ACTIVE_HIGH,
	init: 		mt9p012sensor_init,
	cleanup:	mt9p012sensor_cleanup,
	power_on:	mt9p012sensor_power_on,
	power_off:	mt9p012sensor_power_off,
	try_format:	mt9p012sensor_try_format,
	calc_xclk:	mt9p012sensor_calc_xclk,
	configure:	mt9p012sensor_configure,
	query_control:	NULL,
	get_control:	NULL,
	set_control:	NULL,
	cropcap:	NULL,
	get_crop:	NULL,
	set_crop:	NULL,
	/* we support dual-context so we implement the following optional
	 * functions that are exclusively for the still image context.
	 */
	try_format_still_capture:	mt9p012sensor_try_format_still_capture,
	configure_still_capture:	mt9p012sensor_configure_still_capture,
	enter_still_capture: 		mt9p012sensor_enter_still_capture,
	exit_still_capture:		mt9p012sensor_exit_still_capture,
	/* we allow setting of exposure time and gain by user */
	set_exposure_time:		mt9p012sensor_set_exposure_time,
	set_gain:			mt9p012sensor_set_gain,
	restore:			mt9p012sensor_restore,
};

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("MT9P012 Sensor Driver");
MODULE_LICENSE("GPL");

#ifdef MT9P012_REG_DUMP
static struct mt9p012_reg_read {
	u16 length;
	u16 reg;
	char *name;
} read_reg_list[] = {
	{MT9P012_16BIT, REG_MODEL_ID, "REG_MODEL_ID"},
	{MT9P012_8BIT, REG_MODE_SELECT, "REG_MODE_SELECT"},
	{MT9P012_8BIT, REG_IMAGE_ORIENTATION, "REG_IMAGE_ORIENTATION"},
	{MT9P012_16BIT, REG_FINE_INT_TIME, "REG_FINE_INT_TIME"},
	{MT9P012_16BIT, REG_COARSE_INT_TIME, "REG_COARSE_INT_TIME"},

	{MT9P012_16BIT, REG_VT_PIX_CLK_DIV, "REG_VT_PIX_CLK_DIV"},
	{MT9P012_16BIT,	REG_VT_SYS_CLK_DIV, "REG_VT_SYS_CLK_DIV"},
	{MT9P012_16BIT,	REG_PRE_PLL_CLK_DIV, "REG_PRE_PLL_CLK_DIV"},
	{MT9P012_16BIT,	REG_PLL_MULTIPLIER, "REG_PLL_MULTIPLIER"},
	{MT9P012_16BIT,	REG_OP_PIX_CLK_DIV, "REG_OP_PIX_CLK_DIV"},
	{MT9P012_16BIT, REG_OP_SYS_CLK_DIV, "REG_OP_SYS_CLK_DIV"},

	{MT9P012_16BIT, REG_FRAME_LEN_LINES, "REG_FRAME_LEN_LINES"},
	{MT9P012_16BIT, REG_LINE_LEN_PCK, "REG_LINE_LEN_PCK"},

	{MT9P012_16BIT, REG_X_ADDR_START, "REG_X_ADDR_START"},
	{MT9P012_16BIT, REG_Y_ADDR_START, "REG_Y_ADDR_START"},
	{MT9P012_16BIT, REG_X_ADDR_END, "REG_X_ADDR_END"},
	{MT9P012_16BIT, REG_Y_ADDR_END, "REG_Y_ADDR_END"},
	{MT9P012_16BIT, REG_X_OUTPUT_SIZE, "REG_X_OUTPUT_SIZE"},
	{MT9P012_16BIT, REG_Y_OUTPUT_SIZE, "REG_Y_OUTPUT_SIZE"},

	{MT9P012_16BIT, REG_SCALING_MODE, "REG_SCALING_MODE"},
	{MT9P012_16BIT, REG_SCALE_M, "REG_SCALE_M"},
	{MT9P012_16BIT, REG_SCALE_N, "REG_SCALE_N"},

	{MT9P012_16BIT, REG_ROW_SPEED, "REG_ROW_SPEED"},
	{MT9P012_16BIT, REG_RESET_REGISTER, "REG_RESET_REGISTER"},
	{MT9P012_8BIT, REG_PIXEL_ORDER, "REG_PIXEL_ORDER"},
	{MT9P012_16BIT, REG_READ_MODE, "REG_READ_MODE"},

	{MT9P012_16BIT, REG_DATAPATH_SELECT, "REG_DATAPATH_SELECT"},

	{MT9P012_TOK_TERM, 0, NULL}
};

void 
dump_key_mt9p012_regs(void *priv)
{
	struct mt9p012_sensor *sensor = (struct mt9p012_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	struct mt9p012_reg_read *next = read_reg_list;
	u32 val;

	printk("Begin of dumping:\n");
	for (; next->length != MT9P012_TOK_TERM; next++) {
		if (mt9p012_read_reg(client, next->length, next->reg, &val))
			break;
		printk("%s[0x%x][%s] = 0x%x\n", next->name, next->reg,
				(next->length == MT9P012_8BIT)?"8":
			(next->length == MT9P012_16BIT)?"16":"32", val);
		mdelay(MT9P012_I2C_DELAY);
	}
	printk("End of dumping.\n");
}
#else
void dump_key_mt9p012_regs(void *priv) {}
#endif	/* MT9P012_REG_DUMP */

late_initcall(mt9p012sensor_register);
module_exit(mt9p012sensor_unregister);

#endif	/* CONFIG_VIDEO_OMAP_SENSOR_MT9P012 */
