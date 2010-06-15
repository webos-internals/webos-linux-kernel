/*
 * Copyright (C) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 * Author: Haibo Jeff Zhong <hzhong@qualcomm.com>
 *
 */

#include <asm/arch/gpio.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <media/msm_camera_sensor.h>
#include <asm/arch/camera.h>
#include "mt9t013.h"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define MT9T013_REG_MODEL_ID 0x0000
#define MT9T013_MODEL_ID     0x2600
#define REG_GROUPED_PARAMETER_HOLD   0x0104
#define GROUPED_PARAMETER_HOLD       0x0100
#define GROUPED_PARAMETER_UPDATE     0x0000
#define REG_COARSE_INT_TIME          0x3012
#define REG_VT_PIX_CLK_DIV           0x0300
#define REG_VT_SYS_CLK_DIV           0x0302
#define REG_PRE_PLL_CLK_DIV          0x0304
#define REG_PLL_MULTIPLIER           0x0306
#define REG_OP_PIX_CLK_DIV           0x0308
#define REG_OP_SYS_CLK_DIV           0x030A
#define REG_SCALE_M                  0x0404
#define REG_FRAME_LENGTH_LINES       0x300A
#define REG_LINE_LENGTH_PCK          0x300C
#define REG_X_ADDR_START             0x3004
#define REG_Y_ADDR_START             0x3002
#define REG_X_ADDR_END               0x3008
#define REG_Y_ADDR_END               0x3006
#define REG_X_OUTPUT_SIZE            0x034C
#define REG_Y_OUTPUT_SIZE            0x034E
#define REG_FINE_INT_TIME            0x3014
#define REG_ROW_SPEED                0x3016
#define MT9T013_REG_RESET_REGISTER   0x301A
#define MT9T013_RESET_REGISTER_PWON  0x10CC
#define MT9T013_RESET_REGISTER_PWOFF 0x1008 /* 0x10C8 stop streaming*/
#define REG_READ_MODE                0x3040
#define REG_GLOBAL_GAIN              0x305E
#define REG_TEST_PATTERN_MODE        0x3070

struct reg_struct {
	uint16_t vt_pix_clk_div;          /*  0x0300 */
	uint16_t vt_sys_clk_div;          /*  0x0302 */
	uint16_t pre_pll_clk_div;         /*  0x0304 */
	uint16_t pll_multiplier;          /*  0x0306 */
	uint16_t op_pix_clk_div;          /*  0x0308 */
	uint16_t op_sys_clk_div;          /*  0x030A */
	uint16_t scale_m;                 /*  0x0404 */
	uint16_t row_speed;               /*  0x3016 */
	uint16_t x_addr_start;            /*  0x3004 */
	uint16_t x_addr_end;              /*  0x3008 */
	uint16_t y_addr_start;            /*  0x3002 */
	uint16_t y_addr_end;              /*  0x3006 */
	uint16_t read_mode;               /*  0x3040 */
	uint16_t x_output_size;           /*  0x034C */
	uint16_t y_output_size;           /*  0x034E */
	uint16_t line_length_pck;         /*  0x300C */
	uint16_t frame_length_lines;      /*  0x300A */
	uint16_t coarse_int_time; /*  0x3012 */
	uint16_t fine_int_time;   /*  0x3014 */
};

/*Micron settings from Applications for lower power consumption.*/
struct reg_struct mt9t013_reg_pat[2] = {
	{ /* Preview 2x2 binning 20fps, pclk MHz, MCLK 24MHz */
	/* vt_pix_clk_div:REG=0x0300 update get_snapshot_fps
	* if this change */
	10,

	/* vt_sys_clk_div: REG=0x0302  update get_snapshot_fps
	* if this change */
	1,

	/* pre_pll_clk_div REG=0x0304  update get_snapshot_fps
	* if this change */
	3,

	/* pll_multiplier  REG=0x0306 60 for 30fps preview, 40
	* for 20fps preview */
	80,

	/* op_pix_clk_div        REG=0x0308 */
	10,

	/* op_sys_clk_div        REG=0x030A */
	1,

	/* scale_m       REG=0x0404 */
	16,

	/* row_speed     REG=0x3016 */
	0x0111,

	/* x_addr_start  REG=0x3004 */
	8,

	/* x_addr_end    REG=0x3008 */
	2053,

	/* y_addr_start  REG=0x3002 */
	8,

	/* y_addr_end    REG=0x3006 */
	1541,

	/* read_mode     REG=0x3040 */
	0x046C,

	/* x_output_size REG=0x034C */
	1024,

	/* y_output_size REG=0x034E */
	768,

	/* line_length_pck    REG=0x300C */
	3540,

	/* frame_length_lines REG=0x300A */
	861,

	/* coarse_int_time REG=0x3012 */
	16,

	/* fine_int_time   REG=0x3014 */
	1461
	},
	{ /*Snapshot */
	/* vt_pix_clk_div  REG=0x0300 update get_snapshot_fps
	* if this change */
	10,

	/* vt_sys_clk_div  REG=0x0302 update get_snapshot_fps
	* if this change */
	1,

	/* pre_pll_clk_div REG=0x0304 update get_snapshot_fps
	 * if this change */
	3,

	/* pll_multiplier REG=0x0306 50 for 15fps snapshot,
	* 40 for 10fps snapshot */
	80,

	/* op_pix_clk_div        REG=0x0308 */
	10,

	/* op_sys_clk_div        REG=0x030A */
	1,

	/* scale_m       REG=0x0404 */
	16,

	/* row_speed     REG=0x3016 */
	0x0111,

	/* x_addr_start  REG=0x3004 */
	8,

	/* x_addr_end    REG=0x3008 */
	2073,

	/* y_addr_start  REG=0x3002 */
	8,

	/* y_addr_end    REG=0x3006 */
	1551,

	/* read_mode     REG=0x3040 */
	0x0024,

	/* x_output_size REG=0x034C */
	2066,

	/* y_output_size REG=0x034E */
	1544,

	/* line_length_pck REG=0x300C */
	4800,

	/* frame_length_lines    REG=0x300A */
	1629,

	/* coarse_int_time REG=0x3012 */
	16,

	/* fine_int_time REG=0x3014   */
	733
	}
};

enum mt9t013_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum mt9t013_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum mt9t013_reg_update_t {
  /* Sensor egisters that need to be updated during initialization */
  REG_INIT,
  /* Sensor egisters that needs periodic I2C writes */
  UPDATE_PERIODIC,
  /* All the sensor Registers will be updated */
  UPDATE_ALL,
  /* Not valid update */
  UPDATE_INVALID
};

enum mt9t013_setting_t {
	RES_PREVIEW,
	RES_CAPTURE
};





#define MT9T013_IMG_I2C_ADDR  0x36
/* actuator's Slave Address */
#define MT9T013_AF_I2C_ADDR   0x18

/*
* AF Total steps parameters
*/
#define MT9T013_TOTAL_STEPS_NEAR_TO_FAR    30 /* 28 */

/*
 * Time in milisecs for waiting for the sensor to reset.
 */
#define MT9T013_RESET_DELAY_MSECS   66

/* for 30 fps preview */
#define MT9T013_DEFAULT_CLOCK_RATE  24000000
#define MT9T013_DEFAULT_MAX_FPS     26


/* FIXME: Changes from here */
struct mt9t013_work_t {
	struct work_struct work;
};

struct mt9t013_ctrl_t {
	int8_t  opened;
	struct  msm_camera_device_platform_data *sensordata;
	struct  mt9t013_work_t *sensorw;
	struct  i2c_client *client;

	enum sensor_mode_t sensormode;
	uint32_t fps_divider; /* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum mt9t013_resolution_t prev_res;
	enum mt9t013_resolution_t pict_res;
	enum mt9t013_resolution_t curr_res;
  enum mt9t013_test_mode_t  set_test;
};

struct mt95013_i2c_reg_conf {
	unsigned short saddr;
	unsigned short waddr;
	unsigned short wdata;
};

static struct mt9t013_ctrl_t *mt9t013_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9t013_wait_queue);
DECLARE_MUTEX(mt9t013_sem);

static int mt9t013_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(mt9t013_ctrl->client->adapter, msgs, 2) < 0) {
		CDBG("mt9t013_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9t013_i2c_read_w(unsigned short saddr,
	unsigned short raddr, unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9t013_i2c_rxdata(saddr, buf, 2);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		CDBG("mt9t013_i2c_read failed!\n");

	return rc;
}

static int32_t mt9t013_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
	{
		.addr = saddr,
		.flags = 0,
		.len = length,
		.buf = txdata,
	},
	};

	if (i2c_transfer(mt9t013_ctrl->client->adapter, msg, 1) < 0) {
		CDBG("mt9t013_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9t013_i2c_write_b(unsigned short saddr,
	unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = wdata;
	rc = mt9t013_i2c_txdata(saddr, buf, 2);

	if (rc < 0)
		CDBG("i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t mt9t013_i2c_write_w(unsigned short saddr,
	unsigned short waddr, unsigned short wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00)>>8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9t013_i2c_txdata(saddr, buf, 4);

	if (rc < 0)
		CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t mt9t013_i2c_write_w_table(
	struct mt95013_i2c_reg_conf *reg_conf_tbl, int num_of_items_in_table)
{
	int i;
	int32_t rc = -EFAULT;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mt9t013_i2c_write_w(reg_conf_tbl->saddr,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
}

static uint32_t mt9t013_reset(struct msm_camera_device_platform_data *dev)
{
	int rc;

	rc = gpio_request(dev->sensor_reset, "mt9t013");
	if (!rc) {
		gpio_direction_output(dev->sensor_reset, 0);
		mdelay(2);
		gpio_direction_output(dev->sensor_reset, 1);
	}
	gpio_free(dev->sensor_reset);

	return rc;
}

static int32_t mt9t013_test(enum mt9t013_test_mode_t mo)
{
	int32_t rc = 0;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	if (mo == TEST_OFF)
		return 0;
	else {
		struct mt95013_i2c_reg_conf test_tbl[] = {
			{ MT9T013_IMG_I2C_ADDR, 0x3044, 0x0544 & 0xFBFF },
			{ MT9T013_IMG_I2C_ADDR, 0x30CA, 0x0004 | 0x0001 },
			{ MT9T013_IMG_I2C_ADDR, 0x30D4, 0x9020 & 0x7FFF },
			{ MT9T013_IMG_I2C_ADDR, 0x31E0, 0x0003 & 0xFFFE },
			{ MT9T013_IMG_I2C_ADDR, 0x3180, 0x91FF & 0x7FFF },
			{ MT9T013_IMG_I2C_ADDR, 0x301A, (0x10CC | 0x8000)
								& 0xFFF7 },
			{ MT9T013_IMG_I2C_ADDR, 0x301E, 0x0000 },
			{ MT9T013_IMG_I2C_ADDR, 0x3780, 0x0000 },

		};

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_TEST_PATTERN_MODE,
				(uint16_t)mo);
		if (rc < 0)
			return rc;

		rc = mt9t013_i2c_write_w_table(&test_tbl[0],
			sizeof(test_tbl)/sizeof(struct mt95013_i2c_reg_conf));
		if (rc < 0)
			return rc;
	}

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t013_set_lc(void)
{
	int32_t rc;

	/* [Lens shading 85 Percent TL84] */
	struct mt95013_i2c_reg_conf set_lc_tbl[] = {
		{ MT9T013_IMG_I2C_ADDR, 0x360A, 0x0290 }, /* P_RD_P0Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x360C, 0xC92D }, /* P_RD_P0Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x360E, 0x0771 }, /* P_RD_P0Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3610, 0xE38C }, /* P_RD_P0Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3612, 0xD74F }, /* P_RD_P0Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x364A, 0x168C }, /* P_RD_P1Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x364C, 0xCACB }, /* P_RD_P1Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x364E, 0x8C4C }, /* P_RD_P1Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3650, 0x0BEA }, /* P_RD_P1Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3652, 0xDC0F }, /* P_RD_P1Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x368A, 0x70B0 }, /* P_RD_P2Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x368C, 0x200B }, /* P_RD_P2Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x368E, 0x30B2 }, /* P_RD_P2Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3690, 0xD04F }, /* P_RD_P2Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3692, 0xACF5 }, /* P_RD_P2Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x36CA, 0xF7C9 }, /* P_RD_P3Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x36CC, 0x2AED }, /* P_RD_P3Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x36CE, 0xA652 }, /* P_RD_P3Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x36D0, 0x8192 }, /* P_RD_P3Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x36D2, 0x3A15 }, /* P_RD_P3Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x370A, 0xDA30 }, /* P_RD_P4Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x370C, 0x2E2F }, /* P_RD_P4Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x370E, 0xBB56 }, /* P_RD_P4Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3710, 0x8195 }, /* P_RD_P4Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3712, 0x02F9 }, /* P_RD_P4Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3600, 0x0230 }, /* P_GR_P0Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3602, 0x58AD }, /* P_GR_P0Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3604, 0x18D1 }, /* P_GR_P0Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3606, 0x260D }, /* P_GR_P0Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3608, 0xF530 }, /* P_GR_P0Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3640, 0x17EB }, /* P_GR_P1Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3642, 0x3CAB }, /* P_GR_P1Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3644, 0x87CE }, /* P_GR_P1Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3646, 0xC02E }, /* P_GR_P1Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3648, 0xF48F }, /* P_GR_P1Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3680, 0x5350 }, /* P_GR_P2Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3682, 0x7EAF }, /* P_GR_P2Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3684, 0x4312 }, /* P_GR_P2Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3686, 0xC652 }, /* P_GR_P2Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3688, 0xBC15 }, /* P_GR_P2Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x36C0, 0xB8AD }, /* P_GR_P3Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x36C2, 0xBDCD }, /* P_GR_P3Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x36C4, 0xE4B2 }, /* P_GR_P3Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x36C6, 0xB50F }, /* P_GR_P3Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x36C8, 0x5B95 }, /* P_GR_P3Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3700, 0xFC90 }, /* P_GR_P4Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3702, 0x8C51 }, /* P_GR_P4Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3704, 0xCED6 }, /* P_GR_P4Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3706, 0xB594 }, /* P_GR_P4Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3708, 0x0A39 }, /* P_GR_P4Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3614, 0x0230 }, /* P_BL_P0Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3616, 0x160D }, /* P_BL_P0Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3618, 0x08D1 }, /* P_BL_P0Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x361A, 0x98AB }, /* P_BL_P0Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x361C, 0xEA50 }, /* P_BL_P0Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3654, 0xB4EA }, /* P_BL_P1Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3656, 0xEA6C }, /* P_BL_P1Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3658, 0xFE08 }, /* P_BL_P1Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x365A, 0x2C6E }, /* P_BL_P1Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x365C, 0xEB0E }, /* P_BL_P1Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3694, 0x6DF0 }, /* P_BL_P2Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3696, 0x3ACF }, /* P_BL_P2Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3698, 0x3E0F }, /* P_BL_P2Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x369A, 0xB2B1 }, /* P_BL_P2Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x369C, 0xC374 }, /* P_BL_P2Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x36D4, 0xF2AA }, /* P_BL_P3Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x36D6, 0x8CCC }, /* P_BL_P3Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x36D8, 0xDEF2 }, /* P_BL_P3Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x36DA, 0xFA11 }, /* P_BL_P3Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x36DC, 0x42F5 }, /* P_BL_P3Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3714, 0xF4F1 }, /* P_BL_P4Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3716, 0xF6F0 }, /* P_BL_P4Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3718, 0x8FD6 }, /* P_BL_P4Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x371A, 0xEA14 }, /* P_BL_P4Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x371C, 0x6338 }, /* P_BL_P4Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x361E, 0x0350 }, /* P_GB_P0Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3620, 0x91AE }, /* P_GB_P0Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3622, 0x0571 }, /* P_GB_P0Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3624, 0x100D }, /* P_GB_P0Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3626, 0xCA70 }, /* P_GB_P0Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x365E, 0xE6CB }, /* P_GB_P1Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3660, 0x50ED }, /* P_GB_P1Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3662, 0x3DAE }, /* P_GB_P1Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3664, 0xAA4F }, /* P_GB_P1Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3666, 0xDC50 }, /* P_GB_P1Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x369E, 0x5470 }, /* P_GB_P2Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x36A0, 0x1F6E }, /* P_GB_P2Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x36A2, 0x6671 }, /* P_GB_P2Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x36A4, 0xC010 }, /* P_GB_P2Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x36A6, 0x8DF5 }, /* P_GB_P2Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x36DE, 0x0B0C }, /* P_GB_P3Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x36E0, 0x84CE }, /* P_GB_P3Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x36E2, 0x8493 }, /* P_GB_P3Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x36E4, 0xA610 }, /* P_GB_P3Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x36E6, 0x50B5 }, /* P_GB_P3Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x371E, 0x9651 }, /* P_GB_P4Q0 */
		{ MT9T013_IMG_I2C_ADDR, 0x3720, 0x1EAB }, /* P_GB_P4Q1 */
		{ MT9T013_IMG_I2C_ADDR, 0x3722, 0xAF76 }, /* P_GB_P4Q2 */
		{ MT9T013_IMG_I2C_ADDR, 0x3724, 0xE4F4 }, /* P_GB_P4Q3 */
		{ MT9T013_IMG_I2C_ADDR, 0x3726, 0x79F8 }, /* P_GB_P4Q4 */
		{ MT9T013_IMG_I2C_ADDR, 0x3782, 0x0410 }, /* POLY_ORIGIN_C */
		{ MT9T013_IMG_I2C_ADDR, 0x3784, 0x0320 }, /* POLY_ORIGIN_R  */
		{ MT9T013_IMG_I2C_ADDR, 0x3780, 0x8000 } /* POLY_SC_ENABLE */
	};

	rc = mt9t013_i2c_write_w_table(&set_lc_tbl[0],
		sizeof(set_lc_tbl)/sizeof(struct mt95013_i2c_reg_conf));
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t013_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	uint8_t code_val_msb, code_val_lsb;
	code_val_msb = 0x01;
	code_val_lsb = af_step;

	/* Write the digital code for current to the actuator */
	rc =
		mt9t013_i2c_write_b(MT9T013_AF_I2C_ADDR>>1,
			code_val_msb, code_val_lsb);

	mt9t013_ctrl->curr_lens_pos = 0;
	mt9t013_ctrl->init_curr_lens_pos = 0;
	return rc;
}

static void mt9t013_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;   /*Q10 */
	uint32_t pclk_mult; /*Q10 */

	if (mt9t013_ctrl->prev_res == QTR_SIZE) {
		divider =
			(uint32_t)(
		((mt9t013_reg_pat[RES_PREVIEW].frame_length_lines *
		mt9t013_reg_pat[RES_PREVIEW].line_length_pck) *
		0x00000400) /
		(mt9t013_reg_pat[RES_CAPTURE].frame_length_lines *
		mt9t013_reg_pat[RES_CAPTURE].line_length_pck));

		pclk_mult =
		(uint32_t) ((mt9t013_reg_pat[RES_CAPTURE].pll_multiplier *
		0x00000400) /
		(mt9t013_reg_pat[RES_PREVIEW].pll_multiplier));

	} else {
		/* full size resolution used for preview. */
		divider   = 0x00000400;  /*1.0 */
		pclk_mult = 0x00000400;  /*1.0 */
	}

	/* Verify PCLK settings and frame sizes. */
	*pfps =
		(uint16_t) (fps * divider * pclk_mult /
		0x00000400 / 0x00000400);
}

static uint16_t mt9t013_get_prev_lines_pf(void)
{
	if (mt9t013_ctrl->prev_res == QTR_SIZE)
		return mt9t013_reg_pat[RES_PREVIEW].frame_length_lines;
	else
		return mt9t013_reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9t013_get_prev_pixels_pl(void)
{
	if (mt9t013_ctrl->prev_res == QTR_SIZE)
		return mt9t013_reg_pat[RES_PREVIEW].line_length_pck;
	else
		return mt9t013_reg_pat[RES_CAPTURE].line_length_pck;
}

static uint16_t mt9t013_get_pict_lines_pf(void)
{
	return mt9t013_reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9t013_get_pict_pixels_pl(void)
{
	return mt9t013_reg_pat[RES_CAPTURE].line_length_pck;
}

static uint32_t mt9t013_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	if (mt9t013_ctrl->pict_res == QTR_SIZE) {
		snapshot_lines_per_frame =
		mt9t013_reg_pat[RES_PREVIEW].frame_length_lines - 1;
	} else  {
		snapshot_lines_per_frame =
		mt9t013_reg_pat[RES_CAPTURE].frame_length_lines - 1;
	}

	return snapshot_lines_per_frame * 24;
}

static int32_t mt9t013_set_fps(struct fps_cfg	*fps)
{
	/* input is new fps in Q8 format */
	int32_t rc = 0;

	mt9t013_ctrl->fps_divider = fps->fps_div;
	mt9t013_ctrl->pict_fps_divider = fps->pict_fps_div;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return -EBUSY;

	CDBG("mt9t013_set_fps: fps_div is %d, frame_rate is %d\n",
		fps->fps_div,
		(uint16_t) (
		mt9t013_reg_pat[RES_PREVIEW].frame_length_lines *
		fps->fps_div/0x00000400));

	CDBG("mt9t013_set_fps: fps_mult is %d, frame_rate is %d\n",
		fps->f_mult,
		(uint16_t) (
		mt9t013_reg_pat[RES_PREVIEW].line_length_pck *
		fps->f_mult / 0x00000400));

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_LINE_LENGTH_PCK,
			(uint16_t) (
			mt9t013_reg_pat[RES_PREVIEW].line_length_pck *
			fps->f_mult / 0x00000400));
	if (rc < 0)
		return rc;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9t013_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0x01FF;
	uint32_t line_length_ratio = 0x00000400;
	enum mt9t013_setting_t setting;
	int32_t rc = 0;

	if (mt9t013_ctrl->sensormode ==
			SENSOR_PREVIEW_MODE) {
		mt9t013_ctrl->my_reg_gain = gain;
		mt9t013_ctrl->my_reg_line_count = (uint16_t) line;
	}

	if (gain > 0x00000400)
		gain = max_legal_gain;

	/* Verify no overflow */
	if (mt9t013_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		line =
			(uint32_t) (line * mt9t013_ctrl->fps_divider /
			0x00000400);

		setting = RES_PREVIEW;

	} else {
		line =
			(uint32_t) (line * mt9t013_ctrl->pict_fps_divider /
			0x00000400);

		setting = RES_CAPTURE;
	}

	/*Set digital gain to 1 */
	gain |= 0x0200;

	if ((mt9t013_reg_pat[setting].frame_length_lines - 1) < line) {

		line_length_ratio =
		(uint32_t) (line * 0x00000400) /
		(mt9t013_reg_pat[setting].frame_length_lines - 1);
	} else
		line_length_ratio = 0x00000400;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GLOBAL_GAIN, gain);
	if (rc < 0)
		return rc;

/*	rc =
 *		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
 *			REG_LINE_LENGTH_PCK,
 *			(uint16_t)
 *			(mt9t013_reg_pat[setting].line_length_pck *
 *			line_length_ratio / 0x00000400));
 *	if (rc < 0)
 * 		return rc;
 */

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_COARSE_INT_TIME,
			(uint16_t)((uint32_t) line * 0x00000400 /
			line_length_ratio));
	if (rc < 0)
		return rc;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
	return rc;
}

static int32_t mt9t013_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	rc =
		mt9t013_write_exp_gain(gain, line);
	if (rc < 0)
		return rc;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			MT9T013_REG_RESET_REGISTER,
			0x10CC | 0x0002);

	mdelay(5);

	/* camera_timed_wait(snapshot_wait*exposure_ratio); */
	return rc;
}

static int32_t mt9t013_setting(enum mt9t013_reg_update_t rupdate,
	enum mt9t013_setting_t rt)
{
	int32_t rc = 0;

	switch (rupdate) {
	case UPDATE_PERIODIC: {

	if (rt == RES_PREVIEW ||
			rt == RES_CAPTURE) {

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				MT9T013_REG_RESET_REGISTER,
				MT9T013_RESET_REGISTER_PWOFF);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_VT_PIX_CLK_DIV,
				mt9t013_reg_pat[rt].vt_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_VT_SYS_CLK_DIV,
				mt9t013_reg_pat[rt].vt_sys_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_PRE_PLL_CLK_DIV,
				mt9t013_reg_pat[rt].pre_pll_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_PLL_MULTIPLIER,
				mt9t013_reg_pat[rt].pll_multiplier);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_OP_PIX_CLK_DIV,
				mt9t013_reg_pat[rt].op_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_OP_SYS_CLK_DIV,
				mt9t013_reg_pat[rt].op_sys_clk_div);
		if (rc < 0)
			return rc;

		mdelay(5);

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_ROW_SPEED,
				mt9t013_reg_pat[rt].row_speed);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_X_ADDR_START,
				mt9t013_reg_pat[rt].x_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_X_ADDR_END,
				mt9t013_reg_pat[rt].x_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_Y_ADDR_START,
				mt9t013_reg_pat[rt].y_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_Y_ADDR_END,
				mt9t013_reg_pat[rt].y_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_READ_MODE,
				mt9t013_reg_pat[rt].read_mode);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_SCALE_M,
				mt9t013_reg_pat[rt].scale_m);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_X_OUTPUT_SIZE,
				mt9t013_reg_pat[rt].x_output_size);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_Y_OUTPUT_SIZE,
				mt9t013_reg_pat[rt].y_output_size);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_LINE_LENGTH_PCK,
				mt9t013_reg_pat[rt].line_length_pck);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_FRAME_LENGTH_LINES,
			(mt9t013_reg_pat[rt].frame_length_lines *
			mt9t013_ctrl->fps_divider / 0x00000400));
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_COARSE_INT_TIME,
			mt9t013_reg_pat[rt].coarse_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_FINE_INT_TIME,
			mt9t013_reg_pat[rt].fine_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
		if (rc < 0)
			return rc;

		rc = mt9t013_test(mt9t013_ctrl->set_test);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			MT9T013_REG_RESET_REGISTER,
			MT9T013_RESET_REGISTER_PWON);
		if (rc < 0)
			return rc;

		mdelay(5);

		return rc;
	}
	}
		break;

	/*CAMSENSOR_REG_UPDATE_PERIODIC */
	case REG_INIT: {
	if (rt == RES_PREVIEW ||
			rt == RES_CAPTURE) {

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				MT9T013_REG_RESET_REGISTER,
				MT9T013_RESET_REGISTER_PWOFF);
		if (rc < 0)
			/* MODE_SELECT, stop streaming */
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_VT_PIX_CLK_DIV,
				mt9t013_reg_pat[rt].vt_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_VT_SYS_CLK_DIV,
				mt9t013_reg_pat[rt].vt_sys_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_PRE_PLL_CLK_DIV,
				mt9t013_reg_pat[rt].pre_pll_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_PLL_MULTIPLIER,
				mt9t013_reg_pat[rt].pll_multiplier);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_OP_PIX_CLK_DIV,
				mt9t013_reg_pat[rt].op_pix_clk_div);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_OP_SYS_CLK_DIV,
				mt9t013_reg_pat[rt].op_sys_clk_div);
		if (rc < 0)
			return rc;

		mdelay(5);

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		/* additional power saving mode ok around 38.2MHz */
		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				0x3084, 0x2409);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				0x3092, 0x0A49);
		if (rc < 0)
		return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				0x3094, 0x4949);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				0x3096, 0x4949);
		if (rc < 0)
			return rc;

		/* Set preview or snapshot mode */
		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_ROW_SPEED,
				mt9t013_reg_pat[rt].row_speed);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_X_ADDR_START,
				mt9t013_reg_pat[rt].x_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_X_ADDR_END,
				mt9t013_reg_pat[rt].x_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_Y_ADDR_START,
				mt9t013_reg_pat[rt].y_addr_start);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_Y_ADDR_END,
				mt9t013_reg_pat[rt].y_addr_end);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_READ_MODE,
				mt9t013_reg_pat[rt].read_mode);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_SCALE_M,
				mt9t013_reg_pat[rt].scale_m);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_X_OUTPUT_SIZE,
				mt9t013_reg_pat[rt].x_output_size);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_Y_OUTPUT_SIZE,
				mt9t013_reg_pat[rt].y_output_size);
		if (rc < 0)
			return 0;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_LINE_LENGTH_PCK,
				mt9t013_reg_pat[rt].line_length_pck);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_FRAME_LENGTH_LINES,
				mt9t013_reg_pat[rt].frame_length_lines);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_COARSE_INT_TIME,
				mt9t013_reg_pat[rt].coarse_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_FINE_INT_TIME,
				mt9t013_reg_pat[rt].fine_int_time);
		if (rc < 0)
			return rc;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_UPDATE);
			if (rc < 0)
				return rc;

		/* load lens shading */
		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		/* most likely needs to be written only once. */
		rc = mt9t013_set_lc();
		if (rc < 0)
			return -EBUSY;

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				REG_GROUPED_PARAMETER_HOLD,
				GROUPED_PARAMETER_UPDATE);
		if (rc < 0)
			return rc;

		rc = mt9t013_test(mt9t013_ctrl->set_test);
		if (rc < 0)
			return rc;

		mdelay(5);

		rc =
			mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
				MT9T013_REG_RESET_REGISTER,
				MT9T013_RESET_REGISTER_PWON);
		if (rc < 0)
			/* MODE_SELECT, stop streaming */
			return rc;

		CDBG("!!! mt9t013 !!! PowerOn is done!\n");
		mdelay(5);
		return rc;
		}
		} /* case CAMSENSOR_REG_INIT: */
		break;

	/*CAMSENSOR_REG_INIT */
	default:
		rc = -EFAULT;
		break;
	} /* switch (rupdate) */

	return rc;
}

static int32_t mt9t013_video_config(enum sensor_mode_t mode,
	enum sensor_resolution_t res)
{
	int32_t rc;

	switch (res) {
	case QTR_SIZE:
	 rc =
		 mt9t013_setting(UPDATE_PERIODIC, RES_PREVIEW);
	 if (rc < 0)
		 return rc;

		CDBG("sensor configuration done!\n");
		break;

	case FULL_SIZE:
		rc =
			mt9t013_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;
		break;

	default:
		return 0;
	} /* switch */

	mt9t013_ctrl->prev_res = res;
	mt9t013_ctrl->curr_res = res;
	mt9t013_ctrl->sensormode = mode;

	rc =
		mt9t013_write_exp_gain(mt9t013_ctrl->my_reg_gain,
			mt9t013_ctrl->my_reg_line_count);

	return rc;
}

static int32_t mt9t013_snapshot_config(enum sensor_mode_t mode)
{
	int32_t rc = 0;

	rc =
		mt9t013_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9t013_ctrl->curr_res = mt9t013_ctrl->pict_res;

	mt9t013_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9t013_raw_snapshot_config(enum sensor_mode_t mode)
{
	int32_t rc = 0;

	rc = mt9t013_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9t013_ctrl->curr_res = mt9t013_ctrl->pict_res;

	mt9t013_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9t013_power_down(void)
{
	int32_t rc = 0;

	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			MT9T013_REG_RESET_REGISTER,
			MT9T013_RESET_REGISTER_PWOFF);

	mdelay(5);

	return rc;
}

static int32_t mt9t013_move_focus(enum sensor_move_focus_t direction,
	int32_t num_steps)
{
	int16_t step_direction;
	int16_t actual_step;
	int16_t next_position;
	int16_t break_steps[4];
	uint8_t code_val_msb, code_val_lsb;
	int16_t i;

	if (num_steps > MT9T013_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = MT9T013_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0)
		return -EINVAL;

	if (direction == MOVE_NEAR)
		step_direction = 4;
	else if (direction == MOVE_FAR)
		step_direction = -4;
	else
		return -EINVAL;

	if (mt9t013_ctrl->curr_lens_pos < mt9t013_ctrl->init_curr_lens_pos)
		mt9t013_ctrl->curr_lens_pos = mt9t013_ctrl->init_curr_lens_pos;

	actual_step =
		(int16_t) (step_direction *
		(int16_t) num_steps);

	for (i = 0; i < 4; i++)
		break_steps[i] =
			actual_step / 4 * (i + 1) - actual_step / 4 * i;

	for (i = 0; i < 4; i++) {
		next_position =
		(int16_t)
		(mt9t013_ctrl->curr_lens_pos + break_steps[i]);

		if (next_position > 255)
			next_position = 255;
		else if (next_position < 0)
			next_position = 0;

		code_val_msb =
		((next_position >> 4) << 2) |
		((next_position << 4) >> 6);

		code_val_lsb =
		((next_position & 0x03) << 6);

		/* Writing the digital code for current to the actuator */
		if (mt9t013_i2c_write_b(MT9T013_AF_I2C_ADDR>>1,
				code_val_msb, code_val_lsb) < 0)
			return -EBUSY;

		/* Storing the current lens Position */
		mt9t013_ctrl->curr_lens_pos = next_position;

		if (i < 3)
			mdelay(1);
	} /* for */

	return 0;
}

static int32_t mt9t013_sensor_init(
	struct msm_camera_device_platform_data *camdev)
{
	int32_t  rc;
	uint16_t chipid;

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9T013_DEFAULT_CLOCK_RATE);

	mdelay(50);

	/* For initilization, it needs to do
	* 1. power up;
	* 2. reset the sensor.
	rc = mt9t013_reset(camdev);
	if (rc < 0)
		return rc;

	mdelay(50); */

	/* RESET the sensor image part via I2C command */
	rc = mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
		MT9T013_REG_RESET_REGISTER,
		0x1009);
	if (rc < 0)
		return rc;

	/* 3. Read sensor Model ID: */
	rc = mt9t013_i2c_read_w(MT9T013_IMG_I2C_ADDR,
		MT9T013_REG_MODEL_ID, &chipid);
	if (rc < 0)
		return rc;

	CDBG("mt9t013 model_id = 0x%x\n", chipid);

	/* 4. Compare sensor ID to MT9T012VC ID: */
	if (chipid != MT9T013_MODEL_ID)
		return -ENODEV;


	rc = mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
		0x3064, 0x0805);
	if (rc < 0)
		return rc;

	mdelay(MT9T013_RESET_DELAY_MSECS);

	if (mt9t013_ctrl->prev_res == QTR_SIZE)
		rc = mt9t013_setting(REG_INIT, RES_PREVIEW);
	else
		rc = mt9t013_setting(REG_INIT, RES_CAPTURE);

	if (rc < 0)
		return rc;

	/* sensor : output enable */
#if 0
	rc =
		mt9t013_i2c_write_w(MT9T013_IMG_I2C_ADDR,
			MT9T013_REG_RESET_REGISTER,
			MT9T013_RESET_REGISTER_PWON);

	/* if this fails, the sensor is not the MT9T013 */
	rc = mt9t013_set_default_focus(0);
#endif

	return rc;
}

static int mt9t013_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9t013_wait_queue);
	return 0;
}

static int32_t mt9t013_set_sensor_mode(enum sensor_mode_t mode,
	enum sensor_resolution_t res)
{
	int32_t rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9t013_video_config(mode, res);
		break;

	case SENSOR_SNAPSHOT_MODE:
		rc = mt9t013_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = mt9t013_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int mt9t013_open(struct inode *inode, struct file *fp)
{
	int32_t rc = 0;

	down(&mt9t013_sem);
	CDBG("mt9t013: open = %d\n", mt9t013_ctrl->opened);

	if (mt9t013_ctrl->opened) {
		rc = 0;
		goto open_done;
	}

	rc = mt9t013_sensor_init(mt9t013_ctrl->sensordata);

	CDBG("mt9t013_open: sensor init rc = %d\n", rc);

	if (rc >= 0)
		mt9t013_ctrl->opened = 1;
	else
		CDBG("mt9t013_open: sensor init failed!\n");

open_done:
	up(&mt9t013_sem);
	return rc;
}

static long mt9t013_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct sensor_cfg_data_t cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
				(void *)argp,
				sizeof(struct sensor_cfg_data_t)))
		return -EFAULT;

	down(&mt9t013_sem);

	switch (cmd) {
	case MSM_CAMSENSOR_IO_CFG: {
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
				mt9t013_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data_t)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			mt9t013_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data_t)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				mt9t013_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data_t)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				mt9t013_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data_t)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				mt9t013_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data_t)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				mt9t013_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data_t)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = mt9t013_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				mt9t013_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				mt9t013_set_pict_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = mt9t013_set_sensor_mode(cdata.mode,
						cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = mt9t013_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				mt9t013_move_focus(
					cdata.cfg.focus.dir,
					cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				mt9t013_set_default_focus(
					cdata.cfg.focus.steps);
			break;

		case CFG_SET_EFFECT:
			rc = mt9t013_set_default_focus(
						cdata.cfg.effect);
			break;

		default:
			rc = -EFAULT;
			break;
		}
	}
		break;

	default:
		rc = -EFAULT;
		break;
	}

	up(&mt9t013_sem);

	return rc;
}

static int mt9t013_release(struct inode *ip, struct file *fp)
{
	int rc = -EBADF;

	down(&mt9t013_sem);
	if (mt9t013_ctrl->opened)
		rc = mt9t013_ctrl->opened = 0;

	up(&mt9t013_sem);

	return rc;
}

static struct file_operations mt9t013_fops = {
	.owner 	= THIS_MODULE,
	.open 	= mt9t013_open,
	.release = mt9t013_release,
	.unlocked_ioctl = mt9t013_ioctl,
};

static struct miscdevice mt9t013_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "mt9t013",
	.fops 	= &mt9t013_fops,
};

static int mt9t013_probe(struct i2c_client *client)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	mt9t013_ctrl->sensorw =
		kzalloc(sizeof(struct mt9t013_work_t), GFP_KERNEL);

	if (!mt9t013_ctrl->sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9t013_ctrl->sensorw);
	mt9t013_init_client(client);
	mt9t013_ctrl->client = client;

	rc = gpio_request(mt9t013_ctrl->sensordata->sensor_reset, "mt9t013");
	if (!rc)
		gpio_direction_output(mt9t013_ctrl->sensordata->sensor_reset,
			1);

	gpio_free(mt9t013_ctrl->sensordata->sensor_reset);
	if (rc)
		goto probe_failure;

	mdelay(50);

	/* Register a misc device */
	rc = misc_register(&mt9t013_device);
	if (rc)
		goto probe_failure;

	return 0;

probe_failure:
	if (mt9t013_ctrl->sensorw != NULL) {
		kfree(mt9t013_ctrl->sensorw);
		mt9t013_ctrl->sensorw = NULL;
	}
	return rc;
}

static int mt9t013_remove(struct i2c_client *client)
{
	struct mt9t013_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	i2c_detach_client(client);
	mt9t013_ctrl->client = NULL;
	misc_deregister(&mt9t013_device);
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9t013_driver = {
	.probe  = mt9t013_probe,
	.remove = mt9t013_remove,
	.driver = {
		.name = "mt9t013",
	},
};

int32_t mt9t013_init(void *pdata)
{
	int32_t rc = 0;
	struct  msm_camera_device_platform_data *data =
		(struct  msm_camera_device_platform_data *)pdata;

	mt9t013_ctrl = kzalloc(sizeof(struct mt9t013_ctrl_t), GFP_KERNEL);
	if (!mt9t013_ctrl) {
		rc = -ENOMEM;
		goto init_failure;
	}

	mt9t013_ctrl->fps_divider = 1 * 0x00000400;
	mt9t013_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9t013_ctrl->set_test = TEST_OFF;
	mt9t013_ctrl->prev_res = QTR_SIZE;
	mt9t013_ctrl->pict_res = FULL_SIZE;

	if (data) {
		mt9t013_ctrl->sensordata = data;
		rc = i2c_add_driver(&mt9t013_driver);
	}

init_failure:
	return rc;
}
