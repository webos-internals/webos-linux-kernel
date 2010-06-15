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
#include "mt9d112.h"

/*  I2C Address of the Sensor  */
#define  MT9D112_IMG_I2C_ADDR 0x78>>1

/* Micron MT9D112 Registers and their values */
/* Sensor Core Registers */
#define  REG_MT9D112_MODEL_ID 0x3000
#define  MT9D112_MODEL_ID     0x1580

/*  SOC Registers Page 1  */
#define  REG_MT9D112_SENSOR_RESET     0x301A
#define  REG_MT9D112_STANDBY_CONTROL  0x3202
#define  REG_MT9D112_MCU_BOOT         0x3386

struct mt9d112_work_t {
	struct work_struct work;
};

struct mt9d112_ctrl_t {
	int8_t  opened;
	struct  msm_camera_device_platform_data *sensordata;
	struct  mt9d112_work_t *sensorw;
	struct  i2c_client *client;
};

enum mt9d112_width_t {
	WORD_LEN,
	BYTE_LEN
};

struct mt9d112_i2c_reg_conf {
	unsigned short saddr;
	unsigned short waddr;
	unsigned short wdata;
	enum mt9d112_width_t width;
	unsigned short mdelay_time;
};

static struct mt9d112_ctrl_t *mt9d112_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9d112_wait_queue);
DECLARE_MUTEX(mt9d112_sem);

static struct register_address_value_pair_t
preview_snapshot_mode_reg_settings_array[] = {
	{0x338C, 0x2703},
	{0x3390, 800},    /* Output Width (P) = 640 */
	{0x338C, 0x2705},
	{0x3390, 600},    /* Output Height (P) = 480 */
	{0x338C, 0x2707},
	{0x3390, 0x0640}, /* Output Width (S) = 1600 */
	{0x338C, 0x2709},
	{0x3390, 0x04B0}, /* Output Height (S) = 1200 */
	{0x338C, 0x270D},
	{0x3390, 0x0000}, /* Row Start (P) = 0 */
	{0x338C, 0x270F},
	{0x3390, 0x0000}, /* Column Start (P) = 0 */
	{0x338C, 0x2711},
	{0x3390, 0x04BD}, /* Row End (P) = 1213 */
	{0x338C, 0x2713},
	{0x3390, 0x064D}, /* Column End (P) = 1613 */
	{0x338C, 0x2715},
	{0x3390, 0x0000}, /* Extra Delay (P) = 0 */
	{0x338C, 0x2717},
	{0x3390, 0x2111}, /* Row Speed (P) = 8465 */
	{0x338C, 0x2719},
	{0x3390, 0x046C}, /* Read Mode (P) = 1132 */
	{0x338C, 0x271B},
	{0x3390, 0x024F}, /* Sensor_Sample_Time_pck(P) = 591 */
	{0x338C, 0x271D},
	{0x3390, 0x0102}, /* Sensor_Fine_Correction(P) = 258 */
	{0x338C, 0x271F},
	{0x3390, 0x0279}, /* Sensor_Fine_IT_min(P) = 633 */
	{0x338C, 0x2721},
	{0x3390, 0x0155}, /* Sensor_Fine_IT_max_margin(P) = 341 */
	{0x338C, 0x2723},
	{0x3390, 659},    /* Frame Lines (P) = 679 */
	{0x338C, 0x2725},
	{0x3390, 0x0824}, /* Line Length (P) = 2084 */
	{0x338C, 0x2727},
	{0x3390, 0x2020},
	{0x338C, 0x2729},
	{0x3390, 0x2020},
	{0x338C, 0x272B},
	{0x3390, 0x1020},
	{0x338C, 0x272D},
	{0x3390, 0x2007},
	{0x338C, 0x272F},
	{0x3390, 0x0004}, /* Row Start(S) = 4 */
	{0x338C, 0x2731},
	{0x3390, 0x0004}, /* Column Start(S) = 4 */
	{0x338C, 0x2733},
	{0x3390, 0x04BB}, /* Row End(S) = 1211 */
	{0x338C, 0x2735},
	{0x3390, 0x064B}, /* Column End(S) = 1611 */
	{0x338C, 0x2737},
	{0x3390, 0x04CE}, /* Extra Delay(S) = 1230 */
	{0x338C, 0x2739},
	{0x3390, 0x2111}, /* Row Speed(S) = 8465 */
	{0x338C, 0x273B},
	{0x3390, 0x0024}, /* Read Mode(S) = 36 */
	{0x338C, 0x273D},
	{0x3390, 0x0120}, /* Sensor sample time pck(S) = 288 */
	{0x338C, 0x2741},
	{0x3390, 0x0169}, /* Sensor_Fine_IT_min(P) = 361 */
	{0x338C, 0x2745},
	{0x3390, 0x04FF}, /* Frame Lines(S) = 1279 */
	{0x338C, 0x2747},
	{0x3390, 0x0824}, /* Line Length(S) = 2084 */
	{0x338C, 0x2751},
	{0x3390, 0x0000}, /* Crop_X0(P) = 0 */
	{0x338C, 0x2753},
	{0x3390, 0x0320}, /* Crop_X1(P) = 800 */
	{0x338C, 0x2755},
	{0x3390, 0x0000}, /* Crop_Y0(P) = 0 */
	{0x338C, 0x2757},
	{0x3390, 0x0258}, /* Crop_Y1(P) = 600 */
	{0x338C, 0x275F},
	{0x3390, 0x0000}, /* Crop_X0(S) = 0 */
	{0x338C, 0x2761},
	{0x3390, 0x0640}, /* Crop_X1(S) = 1600 */
	{0x338C, 0x2763},
	{0x3390, 0x0000}, /* Crop_Y0(S) = 0 */
	{0x338C, 0x2765},
	{0x3390, 0x04B0}, /* Crop_Y1(S) = 1200 */
	{0x338C, 0x222E},
	{0x3390, 0x00A0}, /* R9 Step = 160 */
	{0x338C, 0xA408},
	{0x3390, 0x001F},
	{0x338C, 0xA409},
	{0x3390, 0x0021},
	{0x338C, 0xA40A},
	{0x3390, 0x0025},
	{0x338C, 0xA40B},
	{0x3390, 0x0027},
	{0x338C, 0x2411},
	{0x3390, 0x00A0},
	{0x338C, 0x2413},
	{0x3390, 0x00C0},
	{0x338C, 0x2415},
	{0x3390, 0x00A0},
	{0x338C, 0x2417},
	{0x3390, 0x00C0},
	{0x338C, 0x2799},
	{0x3390, 0x6408}, /* MODE_SPEC_EFFECTS(P) */
	{0x338C, 0x279B},
	{0x3390, 0x6408}, /* MODE_SPEC_EFFECTS(S) */
};

static struct register_address_value_pair_t
noise_reduction_reg_settings_array[] = {
	{0x338C, 0xA76D},
	{0x3390, 0x0003},
	{0x338C, 0xA76E},
	{0x3390, 0x0003},
	{0x338C, 0xA76F},
	{0x3390, 0},
	{0x338C, 0xA770},
	{0x3390, 21},
	{0x338C, 0xA771},
	{0x3390, 37},
	{0x338C, 0xA772},
	{0x3390, 63},
	{0x338C, 0xA773},
	{0x3390, 100},
	{0x338C, 0xA774},
	{0x3390, 128},
	{0x338C, 0xA775},
	{0x3390, 151},
	{0x338C, 0xA776},
	{0x3390, 169},
	{0x338C, 0xA777},
	{0x3390, 186},
	{0x338C, 0xA778},
	{0x3390, 199},
	{0x338C, 0xA779},
	{0x3390, 210},
	{0x338C, 0xA77A},
	{0x3390, 220},
	{0x338C, 0xA77B},
	{0x3390, 228},
	{0x338C, 0xA77C},
	{0x3390, 234},
	{0x338C, 0xA77D},
	{0x3390, 240},
	{0x338C, 0xA77E},
	{0x3390, 244},
	{0x338C, 0xA77F},
	{0x3390, 248},
	{0x338C, 0xA780},
	{0x3390, 252},
	{0x338C, 0xA781},
	{0x3390, 255},
	{0x338C, 0xA782},
	{0x3390, 0},
	{0x338C, 0xA783},
	{0x3390, 21},
	{0x338C, 0xA784},
	{0x3390, 37},
	{0x338C, 0xA785},
	{0x3390, 63},
	{0x338C, 0xA786},
	{0x3390, 100},
	{0x338C, 0xA787},
	{0x3390, 128},
	{0x338C, 0xA788},
	{0x3390, 151},
	{0x338C, 0xA789},
	{0x3390, 169},
	{0x338C, 0xA78A},
	{0x3390, 186},
	{0x338C, 0xA78B},
	{0x3390, 199},
	{0x338C, 0xA78C},
	{0x3390, 210},
	{0x338C, 0xA78D},
	{0x3390, 220},
	{0x338C, 0xA78E},
	{0x3390, 228},
	{0x338C, 0xA78F},
	{0x3390, 234},
	{0x338C, 0xA790},
	{0x3390, 240},
	{0x338C, 0xA791},
	{0x3390, 244},
	{0x338C, 0xA793},
	{0x3390, 252},
	{0x338C, 0xA794},
	{0x3390, 255},
	{0x338C, 0xA103},
	{0x3390, 6},
};

static int mt9d112_reset(struct msm_camera_device_platform_data *dev)
{
	int rc = 0;

	rc = gpio_request(dev->sensor_reset, "mt9d112");

	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
		mdelay(20);
		rc = gpio_direction_output(dev->sensor_reset, 1);
	}

	gpio_free(dev->sensor_reset);
	return rc;
}

static int32_t mt9d112_i2c_txdata(unsigned short saddr,
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

	if (i2c_transfer(mt9d112_ctrl->client->adapter, msg, 1) < 0) {
		CDBG("mt9d112_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9d112_i2c_write(unsigned short saddr,
	unsigned short waddr, unsigned short wdata, enum mt9d112_width_t width)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	switch (width) {
	case WORD_LEN: {
		buf[0] = (waddr & 0xFF00)>>8;
		buf[1] = (waddr & 0x00FF);
		buf[2] = (wdata & 0xFF00)>>8;
		buf[3] = (wdata & 0x00FF);

		rc = mt9d112_i2c_txdata(saddr, buf, 4);
	}
		break;

	case BYTE_LEN: {
		buf[0] = waddr;
		buf[1] = wdata;
		rc = mt9d112_i2c_txdata(saddr, buf, 2);
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		CDBG(
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int32_t mt9d112_i2c_write_table(
	struct mt9d112_i2c_reg_conf const *reg_conf_tbl,
	int num_of_items_in_table)
{
	int i;
	int32_t rc = -EFAULT;

	for (i = 0; i < num_of_items_in_table; i++) {
		rc = mt9d112_i2c_write(reg_conf_tbl->saddr, reg_conf_tbl->waddr,
			reg_conf_tbl->wdata, reg_conf_tbl->width);
		if (rc < 0)
			break;
		if (reg_conf_tbl->mdelay_time != 0)
			mdelay(reg_conf_tbl->mdelay_time);
		reg_conf_tbl++;
	}

	return rc;
}

static int mt9d112_i2c_rxdata(unsigned short saddr,
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

	if (i2c_transfer(mt9d112_ctrl->client->adapter, msgs, 2) < 0) {
		CDBG("mt9d112_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t mt9d112_i2c_read(unsigned short   saddr,
	unsigned short raddr, unsigned short *rdata, enum mt9d112_width_t width)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	switch (width) {
	case WORD_LEN: {
		buf[0] = (raddr & 0xFF00)>>8;
		buf[1] = (raddr & 0x00FF);

		rc = mt9d112_i2c_rxdata(saddr, buf, 2);
		if (rc < 0)
			return rc;

		*rdata = buf[0] << 8 | buf[1];
	}
		break;

	default:
		break;
	}

	if (rc < 0)
		CDBG("mt9d112_i2c_read failed!\n");

	return rc;
}

static int32_t mt9d112_set_lens_roll_off(void)
{
	int32_t rc = 0;
	struct mt9d112_i2c_reg_conf const lens_roll_off_tbl[] = {
		{ MT9D112_IMG_I2C_ADDR, 0x34CE, 0x81A0, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34D0, 0x6331, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34D2, 0x3394, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34D4, 0x9966, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34D6, 0x4B25, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34D8, 0x2670, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34DA, 0x724C, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34DC, 0xFFFD, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34DE, 0x00CA, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34E6, 0x00AC, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34EE, 0x0EE1, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34F6, 0x0D87, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3500, 0xE1F7, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3508, 0x1CF4, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3510, 0x1D28, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3518, 0x1F26, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3520, 0x2220, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3528, 0x333D, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3530, 0x15D9, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3538, 0xCFB8, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x354C, 0x05FE, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3544, 0x05F8, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x355C, 0x0596, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3554, 0x0611, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34E0, 0x00F2, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34E8, 0x00A8, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34F0, 0x0F7B, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34F8, 0x0CD7, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3502, 0xFEDB, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x350A, 0x13E4, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3512, 0x1F2C, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x351A, 0x1D20, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3522, 0x2422, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x352A, 0x2925, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3532, 0x1D04, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x353A, 0xFBF2, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x354E, 0x0616, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3546, 0x0597, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x355E, 0x05CD, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3556, 0x0529, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34E4, 0x00B2, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34EC, 0x005E, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34F4, 0x0F43, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34FC, 0x0E2F, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3506, 0xF9FC, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x350E, 0x0CE4, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3516, 0x1E1E, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x351E, 0x1B19, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3526, 0x151B, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x352E, 0x1416, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3536, 0x10FC, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x353E, 0xC018, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3552, 0x06B4, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x354A, 0x0506, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3562, 0x06AB, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x355A, 0x063A, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34E2, 0x00E5, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34EA, 0x008B, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34F2, 0x0E4C, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x34FA, 0x0CA3, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3504, 0x0907, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x350C, 0x1DFD, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3514, 0x1E24, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x351C, 0x2529, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3524, 0x1D20, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x352C, 0x2332, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3534, 0x10E9, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x353C, 0x0BCB, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3550, 0x04EF, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3548, 0x0609, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3560, 0x0580, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3558, 0x05DD, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3540, 0x0000, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x3542, 0x0000, WORD_LEN, 0 }
	};

	rc = mt9d112_i2c_write_table(&lens_roll_off_tbl[0],
		sizeof(lens_roll_off_tbl)/sizeof(struct mt9d112_i2c_reg_conf));
	return rc;
}

static long mt9d112_reg_init(void)
{
	int32_t array_length;
	int32_t i;
	long rc;

	struct mt9d112_i2c_reg_conf const pll_setup_tbl[] = {
		{ MT9D112_IMG_I2C_ADDR, 0x341E, 0x8F09, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x341C, 0x0250, WORD_LEN, 0 },
		{ MT9D112_IMG_I2C_ADDR, 0x341E, 0x8F09, WORD_LEN, 5 },
		{ MT9D112_IMG_I2C_ADDR, 0x341E, 0x8F08, WORD_LEN, 0 }
	};

	/* Refresh Sequencer */
	struct mt9d112_i2c_reg_conf const sequencer_tbl[] = {
		{ MT9D112_IMG_I2C_ADDR, 0x338C, 0xA103, WORD_LEN, 0},
		{ MT9D112_IMG_I2C_ADDR, 0x3390, 0x0005, WORD_LEN, 5},
		{ MT9D112_IMG_I2C_ADDR, 0x338C, 0xA103, WORD_LEN, 0},
		{ MT9D112_IMG_I2C_ADDR, 0x3390, 0x0006, WORD_LEN, 0}
	};

	/* PLL Setup Start */
	rc = mt9d112_i2c_write_table(&pll_setup_tbl[0],
		sizeof(pll_setup_tbl)/sizeof(struct mt9d112_i2c_reg_conf));

	if (rc < 0)
		return rc;
	/* PLL Setup End   */

	array_length =
	sizeof(preview_snapshot_mode_reg_settings_array) /
	sizeof(preview_snapshot_mode_reg_settings_array[0]);

	/* Configure sensor for Preview mode and Snapshot mode */
	for (i = 0; i < array_length; i++) {

	rc =
	mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
	preview_snapshot_mode_reg_settings_array[i].register_address,
	preview_snapshot_mode_reg_settings_array[i].register_value,
	WORD_LEN);
	if (rc < 0)
		return rc;
	}

	/* Configure for Noise Reduction, Saturation and Aperture Correction */
	array_length =
	sizeof(noise_reduction_reg_settings_array) /
	sizeof(noise_reduction_reg_settings_array[0]);

	for (i = 0; i < array_length; i++) {

	rc =
	mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
	noise_reduction_reg_settings_array[i].register_address,
	noise_reduction_reg_settings_array[i].register_value,
	WORD_LEN);
	if (rc < 0)
		return rc;
	}

	/* Set Color Kill Saturation point to optimum value */
	rc =
	mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
	0x35A4,
	0x0593,
	WORD_LEN);
	if (rc < 0)
		return rc;

	rc = mt9d112_i2c_write_table(&sequencer_tbl[0],
		sizeof(sequencer_tbl)/sizeof(struct mt9d112_i2c_reg_conf));
	if (rc < 0)
		return rc;

	rc = mt9d112_set_lens_roll_off();
	if (rc < 0)
		return rc;

	return 0;
}

static long mt9d112_set_sensor_mode(enum sensor_mode_t mode)
{
	uint16_t clock;
	long rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x338C, 0xA20C, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x3390, 0x0004, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x338C, 0xA215, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x3390, 0x0004, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x338C, 0xA20B, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x3390, 0x0000, WORD_LEN);
		if (rc < 0)
			return rc;

		clock = 0x0250;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x341C, clock, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x338C, 0xA103, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x3390, 0x0001, WORD_LEN);
		if (rc < 0)
			return rc;

		mdelay(5);
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Switch to lower fps for Snapshot */
		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x341C, 0x0120, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x338C, 0xA120, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x3390, 0x0002, WORD_LEN);
		if (rc < 0)
			return rc;

		mdelay(5);

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x338C, 0xA103, WORD_LEN);
		if (rc < 0)
			return rc;

		rc =
			mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
				0x3390, 0x0002, WORD_LEN);
		if (rc < 0)
			return rc;
		break;

	default:
		return -EFAULT;
	}

	return 0;
}

static long mt9d112_set_effect(
	enum sensor_mode_t mode,
	int8_t effect
)
{
	uint16_t reg_addr;
	uint16_t reg_val;
	long rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		/* Context A Special Effects */
		reg_addr = 0x2799;
		break;

	case SENSOR_SNAPSHOT_MODE:
		/* Context B Special Effects */
		reg_addr = 0x279B;
		break;

	default :
		reg_addr = 0x2799;
		break;
	}

	switch ((enum camera_effect_t)effect) {
	case CAMERA_EFFECT_OFF: {
		reg_val = 0x6440;

		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x3390, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
	}
			break;

	case CAMERA_EFFECT_MONO: {
		reg_val = 0x6441;
		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x3390, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_NEGATIVE: {
		reg_val = 0x6443;
		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x3390, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_SOLARIZE: {
		reg_val = 0x6445;
		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x3390, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_SEPIA: {
		reg_val = 0x6442;
		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x3390, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;
	}
		break;

	case CAMERA_EFFECT_PASTEL:
	case CAMERA_EFFECT_MOSAIC:
	case CAMERA_EFFECT_RESIZE:
	default: {
		reg_val = 0x6440;
		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x338C, reg_addr, WORD_LEN);
		if (rc < 0)
			return rc;

		rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
			0x3390, reg_val, WORD_LEN);
		if (rc < 0)
			return rc;

		return -EFAULT;
	}
	}

  /* Refresh Sequencer */
	rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
		0x338C, 0xA103, WORD_LEN);
	if (rc < 0)
		return rc;

	rc = mt9d112_i2c_write(MT9D112_IMG_I2C_ADDR,
		0x3390, 0x0005, WORD_LEN);

	return rc;
}

int32_t mt9d112_sensor_init(void)
{
	uint16_t model_id = 0;
	uint32_t mt9d112_camclk_po_hz;
	int rc = 0;

	/* Input MCLK = 24MHz */
	mt9d112_camclk_po_hz = 24000000;
	msm_camio_clk_rate_set(mt9d112_camclk_po_hz);
	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	CDBG("init entry \n");
	rc = mt9d112_reset(mt9d112_ctrl->sensordata);
	if (rc < 0) {
		CDBG("reset failed!\n");
		return rc;
	}

	mdelay(5);

	/* Micron suggested Power up block Start:
	* Put MCU into Reset - Stop MCU */
	rc = mt9d112_i2c_write(
		MT9D112_IMG_I2C_ADDR,
		REG_MT9D112_MCU_BOOT,
		0x0501,
		WORD_LEN);
	if (rc < 0)
		return rc;

	/* Pull MCU from Reset - Start MCU */
	rc = mt9d112_i2c_write(
		MT9D112_IMG_I2C_ADDR,
		REG_MT9D112_MCU_BOOT,
		0x0500,
		WORD_LEN);
	if (rc < 0)
		return rc;

	mdelay(5);

	/* Micron Suggested - Power up block */
	rc = mt9d112_i2c_write(
		MT9D112_IMG_I2C_ADDR,
		REG_MT9D112_SENSOR_RESET,
		0x0ACC,
		WORD_LEN);
	if (rc < 0)
		return rc;

	rc = mt9d112_i2c_write(
		MT9D112_IMG_I2C_ADDR,
		REG_MT9D112_STANDBY_CONTROL,
		0x0008,
		WORD_LEN);
	if (rc < 0)
		return rc;

	/* FUSED_DEFECT_CORRECTION */
	rc = mt9d112_i2c_write(
		MT9D112_IMG_I2C_ADDR,
		0x33F4,
		0x031D,
		WORD_LEN);
	if (rc < 0)
		return rc;

	mdelay(5);

	/* Micron suggested Power up block End */
	/* Read the Model ID of the sensor */
	rc = mt9d112_i2c_read(MT9D112_IMG_I2C_ADDR,
		REG_MT9D112_MODEL_ID, &model_id, WORD_LEN);
	if (rc < 0)
		return rc;

	CDBG("mt9d112 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != MT9D112_MODEL_ID)
		return -EFAULT;

	/* The Sensor is indeed Micron MT9D112 */
	/* Initialize Sensor registers */

	rc = mt9d112_reg_init();
	if (rc < 0)
		return rc;

	return 0;
}

static int mt9d112_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9d112_wait_queue);
	return 0;
}

static int mt9d112_open(struct inode *inode, struct file *fp)
{
	int32_t rc = 0;

	/* down(&mt9d112_sem); */

	CDBG("mt9d112: open = %d\n",
		mt9d112_ctrl->opened);

	if (mt9d112_ctrl->opened) {
		rc = 0;
		goto open_done;
	}

	rc = mt9d112_sensor_init();

	if (rc >= 0)
		mt9d112_ctrl->opened = 1;
	else
		CDBG("sensor init failed!\n");

open_done:
	/* up(&mt9d112_sem); */
	return rc;
}

static long mt9d112_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct sensor_cfg_data_t cfg_data;
	long   rc = 0;

	if (copy_from_user(
				&cfg_data,
				(void *)argp,
				sizeof(struct sensor_cfg_data_t)))
		return -EFAULT;

	/* down(&mt9d112_sem); */

	CDBG("mt9d112_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	switch (cmd) {
	case MSM_CAMSENSOR_IO_CFG: {

		switch (cfg_data.cfgtype) {
		case CFG_SET_MODE:
			rc = mt9d112_set_sensor_mode(
						cfg_data.mode);
			break;

		case CFG_SET_EFFECT:
			rc = mt9d112_set_effect(
						cfg_data.mode,
						cfg_data.cfg.effect);
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

	/* up(&mt9d112_sem); */

	return rc;
}

static int mt9d112_release(struct inode *ip, struct file *fp)
{
	int rc = -EBADF;

	/* down(&mt9d112_sem); */
	if (mt9d112_ctrl->opened)
		rc = mt9d112_ctrl->opened = 0;

	/* up(&mt9d112_sem); */

	return rc;
}

static struct file_operations mt9d112_fops = {
	.owner 	= THIS_MODULE,
	.open 	= mt9d112_open,
	.release = mt9d112_release,
	.unlocked_ioctl = mt9d112_ioctl,
};

static struct miscdevice mt9d112_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "mt9d112",
	.fops 	= &mt9d112_fops,
};

static int mt9d112_probe(struct i2c_client *client)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	mt9d112_ctrl->sensorw =
		kzalloc(sizeof(struct mt9d112_work_t), GFP_KERNEL);

	if (!mt9d112_ctrl->sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9d112_ctrl->sensorw);
	mt9d112_init_client(client);
	mt9d112_ctrl->client = client;

	/* Register a misc device */
	rc = misc_register(&mt9d112_device);
	if (rc)
		goto probe_failure;

	return 0;

probe_failure:
	if (mt9d112_ctrl->sensorw != NULL) {
		kfree(mt9d112_ctrl->sensorw);
		mt9d112_ctrl->sensorw = NULL;
	}
	return rc;
}

static int mt9d112_remove(struct i2c_client *client)
{
	struct mt9d112_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	i2c_detach_client(client);
	mt9d112_ctrl->client = NULL;
	misc_deregister(&mt9d112_device);
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9d112_driver = {
	.probe  = mt9d112_probe,
	.remove = mt9d112_remove,
	.driver = {
		.name = "mt9d112",
	},
};

int32_t mt9d112_init(void *pdata)
{
	int32_t rc = 0;
	struct  msm_camera_device_platform_data *data =
		(struct  msm_camera_device_platform_data *)pdata;

	mt9d112_ctrl = kzalloc(sizeof(struct mt9d112_ctrl_t), GFP_KERNEL);
	if (!mt9d112_ctrl) {
		rc = -ENOMEM;
		goto init_failure;
	}

	if (data) {
		mt9d112_ctrl->sensordata = data;
		rc = i2c_add_driver(&mt9d112_driver);
	}

init_failure:
	return rc;
}
