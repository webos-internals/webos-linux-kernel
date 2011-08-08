/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <linux/mt9p013.h>
#include "mt9p013.h"

#define SUPPORT_MT9P015
/*=============================================================
    SENSOR REGISTER DEFINES
==============================================================*/
#define MT9P013_REG_MODEL_ID         0x0000
#define MT9P013_MODEL_ID             0x2803
#define REG_MODE_SELECT		     0x0100
#define MODE_SELECT_STREAMING_OFF    0x0000
#define MODE_SELECT_STREAMING_ON     0x0001
#define REG_SOFTWARE_RESET	     0x0103
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
#define REG_FINE_INTEGRATION_TIME    0x3014
#define REG_FINE_CORRECTION          0x3010
#define REG_ROW_SPEED                0x3016
#define REG_RESET_REGISTER   	     0x301A
#define RESET_REGISTER_PWON  	     0x001C
#define RESET_REGISTER_PWOFF         0x0018
#define REG_READ_MODE                0x3040
#define REG_FLASH                    0x3046
#define REG_FLASH_ON		     0x0100
#define REG_FLASH_OFF		     0x0000
#define REG_GLOBAL_GAIN              0x305E
#define REG_TEST_PATTERN_MODE        0x3070

#define MAX_SNAPSHOT_LUT_SIZE 	     10

#define MT9P013_REV_7
#define MT9P013_5131

#define MT9P013_AF_I2C_ADDR   0x8A
#ifndef DIFF
#define DIFF(a,b) ( ((a) > (b)) ? ((a) - (b)) : ((b) - (a)) )
#endif

enum mt9p013_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};

enum mt9p013_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum mt9p013_reg_update_t {
	REG_INIT,
	UPDATE_PERIODIC,
	UPDATE_ALL,
	UPDATE_INVALID
};

enum mt9p013_setting_t {
	RES_PREVIEW,
	RES_CAPTURE
};

/* AF Total steps parameters */
#define MT9P013_STEPS_NEAR_TO_CLOSEST_INF  30
#define MT9P013_TOTAL_STEPS_NEAR_TO_FAR    30

/* Min AF frame time */
#define MT9P013_MIN_AF_FRAME_TIME_MSEC 33

/* BAM steps to extend Infinity */
#define MT9P013_BAM_INF_EXT 22

/* BAM move time defines */
#define MT9P013_BAM_MOVE_BASE_MSEC 15
#define MT9P013_BAM_MOVE_RANGE_MSEC 150

#define MT9P013_MU5M0_PREVIEW_DUMMY_PIXELS 0
#define MT9P013_MU5M0_PREVIEW_DUMMY_LINES  0

/* Time in milisecs for waiting for the sensor to reset.*/
#define MT9P013_RESET_DELAY_MSECS   66

/* for 20 fps preview */
#define MT9P013_DEFAULT_CLOCK_RATE  24000000

struct mt9p013_work {
	struct work_struct work;
	int    (*init)(void);
	int    (*deinit)(void);
	int    (*power_shutdown)(void);
	int    (*power_resume)(void);
	
};
static struct mt9p013_work *mt9p013_sensorw;
static struct i2c_client *mt9p013_client;

struct mt9p013_ctrl {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;
	uint32_t pict_fps_divider;

	uint16_t curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum mt9p013_resolution_t prev_res;
	enum mt9p013_resolution_t pict_res;
	enum mt9p013_resolution_t curr_res;
	enum mt9p013_test_mode_t  set_test;
};

static uint16_t bam_macro, bam_infinite;
static uint16_t bam_step_lookup_table[MT9P013_TOTAL_STEPS_NEAR_TO_FAR + 1];
static struct mt9p013_ctrl *mt9p013_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9p013_wait_queue);
DEFINE_MUTEX(mt9p013_mut);

static u16 debug_regaddr = 0;
static u16 debug_focus_pos = 0;
static u16 is_flash_trigger = 0;

/*=============================================================*/

static int mt9p013_i2c_rxdata(unsigned short saddr, int slength,
			      unsigned char *rxdata, int rxlength)
{
	struct i2c_msg msgs[] = {
	{	.addr   = saddr,
		.flags = 0,
		.len   = slength,
		.buf   = rxdata,
	},
	{   .addr   = saddr,
		.flags = I2C_M_RD,
		.len   = rxlength,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(mt9p013_client->adapter, msgs, 2) < 0) {
		CDBG("%s failed!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int32_t mt9p013_i2c_af_read_b(unsigned short saddr, unsigned char raddr,
				  	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char data;

	if (!rdata)
		return -EIO;
	data = raddr;
	rc = mt9p013_i2c_rxdata(saddr, 1, &data, 1);

	if (rc < 0) {
		CDBG("%s failed\n", __func__);
		return rc;
	}	

	*rdata = data;

	CDBG("%s saddr = 0x%x addr = 0x%x, val = 0x%x\n", __func__, saddr, raddr, *rdata);
	return rc;
}

static int32_t mt9p013_i2c_sensor_read_w(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9p013_i2c_rxdata(saddr, 2, buf, 2);

	if (rc < 0) {
		CDBG("%s failed!\n", __func__);
		return rc;
	}

	*rdata = buf[0] << 8 | buf[1];

	CDBG("%s saddr = 0x%x addr = 0x%x, val = 0x%x\n", __func__, saddr, raddr, *rdata);
	return rc;
}

static int32_t mt9p013_i2c_txdata(unsigned short saddr,	unsigned char *txdata,
	int length)
{
	struct i2c_msg msg[] = {
		{
		.addr  = saddr,
		.flags = 0,
		.len = length,
		.buf = txdata,
		},
	};

	if (i2c_transfer(mt9p013_client->adapter, msg, 1) < 0) {
		CDBG("%s failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int32_t mt9p013_i2c_af_write_b(unsigned short saddr, unsigned short baddr,
	unsigned short bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	memset(buf, 0, sizeof(buf));
	buf[0] = baddr;
	buf[1] = bdata;
	rc = mt9p013_i2c_txdata(saddr, buf, 2);

	if (rc < 0) {
		CDBG("%s failed\n", __func__);
		return rc;
	}

	CDBG("%s saddr = 0x%x addr = 0x%x, val = 0x%x\n", __func__, saddr, baddr, bdata);
	return rc;
}

static int32_t mt9p013_i2c_sensor_write_b(unsigned short saddr, unsigned short waddr,
	unsigned short bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	rc = mt9p013_i2c_txdata(saddr, buf, 3);

	if (rc < 0) {
		CDBG("%s failed\n", __func__);
		return rc;
	}

	CDBG("%s saddr = 0x%x addr = 0x%x, val = 0x%x\n", __func__, saddr, waddr, bdata);
	return rc;
}

static int32_t mt9p013_i2c_sensor_write_w(unsigned short saddr, unsigned short waddr,
	unsigned short wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00)>>8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9p013_i2c_txdata(saddr, buf, 4);

	if (rc < 0) {
		CDBG("%s failed\n", __func__);
		return rc;
	}

	CDBG("%s saddr = 0x%x addr = 0x%x, val = 0x%x\n", __func__, saddr, waddr, wdata);
	return rc;
}

static ssize_t
mt9p013_attr_show_regdata(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	unsigned short val;
	int rc;
	
	CDBG("+++%s,%d: regaddr=0x%x\n",__func__,__LINE__, debug_regaddr);
	if ((rc = mt9p013_i2c_sensor_read_w(mt9p013_client->addr >> 1, debug_regaddr, &val)) < 0)
		goto exit;

	rc = sprintf(buf, "0x%04x\n", val);
	CDBG("---%s,%d: regddr=0x%x val=0x%x\n",__func__,__LINE__, debug_regaddr, val);
exit:
	return (rc);
}

static ssize_t
mt9p013_attr_store_regdata(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned int val;

	CDBG("+++%s,%d\n",__func__,__LINE__);

	if (!sscanf(buf,"%04x\n", &val)) {
		rc = -EINVAL;
		goto exit;
	}
	
	CDBG("***%s,%d: regaddr=0x%x regdata=0x%x\n",__func__,__LINE__, debug_regaddr, val);
	if ((rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1, debug_regaddr, val)) < 0)
		goto exit;

	rc = count;
	CDBG("---%s,%d\n",__func__,__LINE__);
exit:
	return (rc);
}

static ssize_t
mt9p013_attr_show_regaddr(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int rc;
	rc = sprintf(buf, "0x%04x\n", debug_regaddr);
	return (rc);
}

static ssize_t
mt9p013_attr_store_regaddr(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned int val;
	if (!sscanf(buf, "0x%04x", &val)) {
		rc = -EINVAL;
		goto exit;
	}

	debug_regaddr = val;
	rc = count;
exit:
	return (rc);
}

#define IC_CONTROL_1	0x34
#define IC_CONTROL_7	0x44

static uint16_t af_read_pos( uint16_t *pos)
{
	int rc = 0;
	unsigned short temp_lsb;
	unsigned short temp_msb;

	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x01, IC_CONTROL_1 | 0x03);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_read_b(MT9P013_AF_I2C_ADDR >> 1, 0x03, &temp_msb);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_read_b(MT9P013_AF_I2C_ADDR >> 1, 0x04, &temp_lsb);
	if (rc < 0) goto exit;

	*pos = ((temp_msb << 8) | temp_lsb);
exit:
	return rc;
}

static int af_move(uint16_t pos, uint16_t time)
{
	int rc = 0;

	if (pos > 1023)
		pos = 1023;

 	CDBG("af_move: pos = %d, time = %d\n", pos, time);

	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x01, IC_CONTROL_1 | 0x10);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x05, (pos >> 8) & 0x03);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x06, pos & 0xFF);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x0B, time);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x07, IC_CONTROL_7 | 0x01);
	if (rc < 0) goto exit;

	debug_focus_pos = pos;

	if (time > MT9P013_MIN_AF_FRAME_TIME_MSEC) {
		msleep(time);
	}
exit:
	return rc;
}

static int af_check_freq(uint16_t freq)
{
	int rc;
	uint16_t first_pos, second_pos;

	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x08, freq);
	if (rc < 0) return rc;
	rc = af_move(bam_macro, 40);
	if (rc < 0) return rc;
	rc = af_move(bam_infinite, 40);
	if (rc < 0) return rc;
	rc = af_move(bam_macro, 40);
	if (rc < 0) return rc;
	rc = af_read_pos(&first_pos);
	if (rc < 0) return rc;
	rc = af_move(bam_infinite, 40);
	if (rc < 0) return rc;
	rc = af_read_pos(&second_pos);
	if (rc < 0) return rc;
	
	if (second_pos > (first_pos + ((bam_macro + bam_infinite) >> 1)))
		return 1;

	return 0;
}

static int af_init_freq(void)
{
	uint16_t high_freq;
	uint16_t mid_freq;
	uint16_t low_freq;
	int rc = 0;

	rc = mt9p013_i2c_af_read_b(MT9P013_AF_I2C_ADDR >> 1, 0x08, &mid_freq);
	if (rc < 0) return rc;

	if ((mid_freq < 64) || (mid_freq > 127)) {
		CDBG("%s:%d mid_freq=%d\n", __func__, __LINE__, mid_freq);
		return 0;
	}

	low_freq = (mid_freq > 67) ? mid_freq - 3 : 64;
	if (af_check_freq(low_freq) == 1) {
		CDBG("%s:%d low_freq=%d\n", __func__, __LINE__, low_freq);
		return 0;
	}

	high_freq = (mid_freq < 124) ? mid_freq + 3 : 127;
	if (af_check_freq(high_freq) == 1) {
		CDBG("%s:%d high_freq=%d\n", __func__, __LINE__, high_freq);
		return 0;
	}

	CDBG("%s:%d mid_freq=%d\n", __func__, __LINE__, mid_freq);

	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x08, mid_freq);

	return rc;
}

static int af_init(void)
{
	int32_t  rc;
	uint8_t i;
	unsigned short temp_lsb;
	unsigned short temp_msb;
	uint16_t temp_q8;

	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x01, IC_CONTROL_1);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x07, IC_CONTROL_7);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x0A, 0xFF);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_write_b(MT9P013_AF_I2C_ADDR >> 1, 0x16, 0x15);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_read_b(MT9P013_AF_I2C_ADDR >> 1, 0x12, &temp_msb);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_read_b(MT9P013_AF_I2C_ADDR >> 1, 0x13, &temp_lsb);
	if (rc < 0) goto exit;
	bam_infinite = ( ((uint16_t) temp_msb) << 8) | ((uint16_t) temp_lsb);
	
	rc = mt9p013_i2c_af_read_b(MT9P013_AF_I2C_ADDR >> 1, 0x14, &temp_msb);
	if (rc < 0) goto exit;
	rc = mt9p013_i2c_af_read_b(MT9P013_AF_I2C_ADDR >> 1, 0x15, &temp_lsb);
	if (rc < 0) goto exit;
	bam_macro = ( ((uint16_t) temp_msb) << 8) | ((uint16_t) temp_lsb);

	CDBG("bam_infinite=%d bam_macro=%d\n", bam_infinite, bam_macro);

	temp_q8 = ((bam_infinite + MT9P013_BAM_INF_EXT - bam_macro) << 8) /
		MT9P013_TOTAL_STEPS_NEAR_TO_FAR;

	for (i = 0; i < MT9P013_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		bam_step_lookup_table[i] = bam_macro +
			(((temp_q8 * i) + (1 << 7)) >> 8);
	}

	bam_step_lookup_table[MT9P013_TOTAL_STEPS_NEAR_TO_FAR] = 
		bam_infinite + MT9P013_BAM_INF_EXT;

	for (i = 0; i <= MT9P013_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		CDBG("bam_step_lookup_table[%d]=%d\n"
			, i, bam_step_lookup_table[i]);
	}

exit:
	return rc;
}
static ssize_t
mt9p013_attr_show_focus_position(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int rc;
	rc = sprintf(buf, "%d\n", debug_focus_pos);
	
	return (rc);
}

static ssize_t
mt9p013_attr_store_focus_position(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc = 0;
	unsigned int pos;

	CDBG("+++%s,%d\n",__func__,__LINE__);

	if (!sscanf(buf,"%d\n", &pos)) {
		rc = -EINVAL;
		goto exit;
	}
	
	printk(KERN_INFO "af move to position=%d\n", pos);

	rc = af_move(pos, 140);

	if (rc != 0)
		goto exit;

	rc = count;
	CDBG("---%s,%d\n",__func__,__LINE__);
exit:
	return (rc);
}

static struct device_attribute mt9p013_attr_regdata =
	__ATTR(regdata, S_IRUGO|S_IWUGO, mt9p013_attr_show_regdata,
		mt9p013_attr_store_regdata);

static struct device_attribute mt9p013_attr_regaddr =
	__ATTR(regaddr, S_IRUGO|S_IWUGO, mt9p013_attr_show_regaddr,
		mt9p013_attr_store_regaddr);

static struct device_attribute mt9p013_attr_focus_position =
	__ATTR(focus_position, S_IRUGO|S_IWUGO, mt9p013_attr_show_focus_position,
		mt9p013_attr_store_focus_position);

static int32_t mt9p013_i2c_sensor_write_w_table(
	struct mt9p013_i2c_reg_conf const *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EFAULT;

	for (i = 0; i < num; i++) {
		rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
}

int32_t mt9p013_enable_flash(uint8_t enable)
{
	int rc = 0;

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
		REG_FLASH, 0x0600 | (enable ? REG_FLASH_ON : REG_FLASH_OFF));

	is_flash_trigger = 1;

	return rc;
}

static int32_t mt9p013_test(enum mt9p013_test_mode_t mo)
{
	int32_t rc = 0;
	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
		REG_GROUPED_PARAMETER_HOLD,
		GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;
	if (mo == TEST_OFF)
		return 0;
	else {
		rc = mt9p013_i2c_sensor_write_w_table(mt9p013_regs.ttbl,
					mt9p013_regs.ttbl_size);
		if (rc < 0)
			return rc;
		rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			REG_TEST_PATTERN_MODE, (uint16_t)mo);
		if (rc < 0)
			return rc;
	}
	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
		REG_GROUPED_PARAMETER_HOLD,
		GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;
	return rc;
}

static int32_t mt9p013_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;
	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
		REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;
	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1, 0x3780,
		((uint16_t) is_enable) << 15);
	if (rc < 0)
		return rc;
	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
		REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);

	return rc;
}

static int32_t mt9p013_set_lc(void)
{
	int32_t rc;

	rc = mt9p013_i2c_sensor_write_w_table(mt9p013_regs.lctbl,
			mt9p013_regs.lctbl_size);
	if (rc < 0)
		return rc;

	return rc;
}

static void mt9p013_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;	/*Q10 */
	//uint32_t pclk_mult;	/*Q10 */
	uint32_t d1;
	uint32_t d2;

	d1 =
		(uint32_t)(
		(mt9p013_regs.reg_pat[RES_PREVIEW].frame_length_lines *
		0x00000400) /
		mt9p013_regs.reg_pat[RES_CAPTURE].frame_length_lines);

	d2 =
		(uint32_t)(
		(mt9p013_regs.reg_pat[RES_PREVIEW].line_length_pck *
		0x00000400) /
		mt9p013_regs.reg_pat[RES_CAPTURE].line_length_pck);

	divider = (uint32_t) (d1 * d2) / 0x00000400;

	/* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t) (fps * divider / 0x00000400 );
}

static uint16_t mt9p013_get_prev_lines_pf(void)
{
	if (mt9p013_ctrl->prev_res == QTR_SIZE)
		return mt9p013_regs.reg_pat[RES_PREVIEW].frame_length_lines;
	else
		return mt9p013_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p013_get_prev_pixels_pl(void)
{
	if (mt9p013_ctrl->prev_res == QTR_SIZE)
		return mt9p013_regs.reg_pat[RES_PREVIEW].line_length_pck;
	else
		return mt9p013_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint16_t mt9p013_get_pict_lines_pf(void)
{
	return mt9p013_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p013_get_pict_pixels_pl(void)
{
	return mt9p013_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint32_t mt9p013_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	if (mt9p013_ctrl->pict_res == QTR_SIZE)
		snapshot_lines_per_frame =
		mt9p013_regs.reg_pat[RES_PREVIEW].frame_length_lines - 1;
	else
		snapshot_lines_per_frame =
		mt9p013_regs.reg_pat[RES_CAPTURE].frame_length_lines - 1;

	return snapshot_lines_per_frame * 24;
}

static int32_t mt9p013_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int32_t rc = 0;
	mt9p013_ctrl->fps_divider = fps->fps_div;
	mt9p013_ctrl->pict_fps_divider = fps->pict_fps_div;
	rc =
		mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return -EBUSY;
	rc =
		mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			REG_LINE_LENGTH_PCK,
			(mt9p013_regs.reg_pat[RES_PREVIEW].line_length_pck *
			fps->f_mult / 0x00000400));
	if (rc < 0)
	 	return rc;
	rc =
		mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);

	return rc;
}
static int32_t mt9p013_write_exp_gain(uint16_t gain, uint32_t line,int8_t is_outdoor) 
{
	uint16_t max_legal_gain = 0x01FF;
	int32_t rc = 0;

	if ( (mt9p013_ctrl->sensormode == SENSOR_PREVIEW_MODE) &&
	     (is_flash_trigger == 0)) {
		mt9p013_ctrl->my_reg_gain = gain;
		mt9p013_ctrl->my_reg_line_count = (uint16_t)line;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	gain |= 0x1000;

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1, 
		REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
	if (rc < 0) {
		CDBG("mt9p013_i2c_sensor_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

#ifdef USE_COLOR_SHIFT_FIX
	if (mt9p013_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		if(is_outdoor == 1) {
			CDBG("%s:%d, outdoor\n", __func__, __LINE__);
			rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			0x316c, 0xA4F0); 
			if (rc < 0) {
				CDBG("%s:failed... Line:%d \n",
								__func__, __LINE__);
				return rc;
			}
	        } else if (is_outdoor == 0) {
			CDBG("%s:%d, indoor\n", __func__, __LINE__);
			rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			0x316c, 0x4410); 
			if (rc < 0) {
				CDBG("%s:failed... Line:%d \n",
								__func__, __LINE__);
				return rc;
			}
		}
	}
#endif

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1, 
		REG_GLOBAL_GAIN, gain);
	if (rc < 0) {
		CDBG("mt9p013_i2c_sensor_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1, 
		REG_COARSE_INT_TIME, (uint16_t)(line ));
	if (rc < 0) {
		CDBG("mt9p013_i2c_sensor_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1, 
		REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		CDBG("mt9p013_i2c_sensor_write_w failed... Line:%d \n", __LINE__);

	return rc;
}

static int32_t mt9p013_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	CDBG("Line:%d mt9p013_set_pict_exp_gain \n", __LINE__);

	rc = mt9p013_write_exp_gain(gain, line, -1);
	if (rc < 0) {
		CDBG("Line:%d mt9p013_set_pict_exp_gain failed... \n",
			__LINE__);
		return rc;
	}

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
		REG_RESET_REGISTER,
		RESET_REGISTER_PWON | 0x0002);
	if (rc < 0) {
		CDBG("mt9p013_i2c_sensor_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	mdelay(5);

	/* camera_timed_wait(snapshot_wait*exposure_ratio); */
	return rc;
}

static int32_t mt9p013_setting(enum mt9p013_reg_update_t rupdate,
	enum mt9p013_setting_t rt)
{
	int32_t rc = 0;
	struct msm_camera_csi_params mt9p013_csi_params;
	CDBG("+++%s: rt = %d\n", __func__, rt);
	switch (rupdate) {
	case UPDATE_PERIODIC:
	  	if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
			struct mt9p013_i2c_reg_conf clk_tbl[] = {
			{REG_VT_PIX_CLK_DIV, mt9p013_regs.reg_pat[rt].vt_pix_clk_div},
			{REG_VT_SYS_CLK_DIV, mt9p013_regs.reg_pat[rt].vt_sys_clk_div},
			{REG_PRE_PLL_CLK_DIV, mt9p013_regs.reg_pat[rt].pre_pll_clk_div},
			{REG_PLL_MULTIPLIER, mt9p013_regs.reg_pat[rt].pll_multiplier},
			{REG_OP_PIX_CLK_DIV, mt9p013_regs.reg_pat[rt].op_pix_clk_div},
			{REG_OP_SYS_CLK_DIV, mt9p013_regs.reg_pat[rt].op_sys_clk_div},
			};

			struct mt9p013_i2c_reg_conf frame_tbl[] = {
			{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD},
			{REG_X_ADDR_START, mt9p013_regs.reg_pat[rt].x_addr_start},
			{REG_X_ADDR_END, mt9p013_regs.reg_pat[rt].x_addr_end},
			{REG_Y_ADDR_START, mt9p013_regs.reg_pat[rt].y_addr_start},
			{REG_Y_ADDR_END, mt9p013_regs.reg_pat[rt].y_addr_end},
			{REG_READ_MODE, mt9p013_regs.reg_pat[rt].read_mode},
			{REG_SCALE_M, mt9p013_regs.reg_pat[rt].scale_m},
			{REG_X_OUTPUT_SIZE, mt9p013_regs.reg_pat[rt].x_output_size},
			{REG_Y_OUTPUT_SIZE, mt9p013_regs.reg_pat[rt].y_output_size},
			{REG_LINE_LENGTH_PCK, mt9p013_regs.reg_pat[rt].line_length_pck},
			{REG_FRAME_LENGTH_LINES,
				(mt9p013_regs.reg_pat[rt].frame_length_lines *
				mt9p013_ctrl->fps_divider / 0x00000400)},
			{REG_COARSE_INT_TIME, mt9p013_regs.reg_pat[rt].coarse_int_time},
			{REG_FINE_INTEGRATION_TIME,
			  mt9p013_regs.reg_pat[rt].fine_int_time},
			{REG_FINE_CORRECTION,
			  mt9p013_regs.reg_pat[rt].fine_correction},
			{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE},
			};

			CDBG("update start\n");
			mt9p013_csi_params.data_format = CSI_10BIT;
			mt9p013_csi_params.lane_cnt = 2;
			mt9p013_csi_params.lane_assign = 0xe4;
			mt9p013_csi_params.dpcm_scheme = 0;
			mt9p013_csi_params.settle_cnt = 26;
		
			rc = msm_camio_csi_config(&mt9p013_csi_params);
			if (rc < 0)
				return rc;
			usleep(1000);
			rc = mt9p013_i2c_sensor_write_w_table(&clk_tbl[0],
				ARRAY_SIZE(clk_tbl));
			if (rc < 0)
				return rc;
			usleep(1000);
			rc = mt9p013_i2c_sensor_write_w_table(&frame_tbl[0],
				ARRAY_SIZE(frame_tbl));
			
			is_flash_trigger = 0;

			CDBG("update done\n");
			return rc;
	  	}
		break; /* UPDATE_PERIODIC */

	case REG_INIT: {
		struct mt9p013_i2c_reg_conf standby_tbl[] = {
		{REG_RESET_REGISTER, RESET_REGISTER_PWOFF},
		{0x3064, 0x0805},
		{0x31AE, 0x0202},
		{0x0112, 0x0A0A},
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD},
		{0x309E, 0x5D00},
		{0x3094, 0x5056},
		{0x31E0, 0x0081},
		{0x3088, 0x6FFF},
		{0x3086, 0x2468},
		{0x316c, 0xA4F0},
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE},
		};

		CDBG("init start\n");
		/* reset fps_divider */
		mt9p013_ctrl->fps_divider = 1 * 0x0400;

		msleep(10);

		rc = mt9p013_i2c_sensor_write_w_table(&standby_tbl[0],
			ARRAY_SIZE(standby_tbl));
		if (rc < 0)
			return rc;
		if (mt9p013_sensorw->init)
			rc = mt9p013_sensorw->init();
		CDBG("init done\n");
		}
		break; /* case REG_INIT: */

	default:
		rc = -EFAULT;
		break;
	} /* switch (rupdate) */
	CDBG("---%s: rc = %d\n", __func__, rc);
	return rc;
}

static int32_t mt9p013_video_config(int mode,
	int res)
{
	int32_t rc;

	switch (res) {
	case QTR_SIZE:
		rc = mt9p013_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0)
			return rc;

		CDBG("mt9p013 sensor configuration done!\n");
		break;

	case FULL_SIZE:
		rc =
		mt9p013_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;

		break;

	default:
		return 0;
	} /* switch */

	mt9p013_ctrl->prev_res = res;
	mt9p013_ctrl->curr_res = res;
	mt9p013_ctrl->sensormode = mode;

	rc = mt9p013_write_exp_gain(mt9p013_ctrl->my_reg_gain,
			mt9p013_ctrl->my_reg_line_count, -1);
	if (rc < 0)
		return rc;

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
			REG_RESET_REGISTER,
			RESET_REGISTER_PWON|0x0002);

	return rc;
}

static int32_t mt9p013_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = mt9p013_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p013_ctrl->curr_res = mt9p013_ctrl->pict_res;

	mt9p013_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p013_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = mt9p013_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p013_ctrl->curr_res = mt9p013_ctrl->pict_res;

	mt9p013_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p013_power_down(void)
{
	int32_t rc = 0;

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1,
		REG_RESET_REGISTER,
		RESET_REGISTER_PWOFF);
	if (rc < 0)
		return rc;
	if (mt9p013_sensorw->deinit)
		rc = mt9p013_sensorw->deinit();
	mdelay(5);
	return rc;
}

static int32_t mt9p013_move_focus(int direction, int32_t num_steps)
{
	int32_t rc;
	int16_t step_direction;
	int16_t actual_step;
	int16_t next_position;
	uint8_t code_val;
	uint8_t time_out;
	uint16_t actual_position_target;

	CDBG("+++%s:%d dir=%d, step=%d\n", __func__, __LINE__, direction, num_steps);
	if (num_steps > MT9P013_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = MT9P013_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0) {
		CDBG("%s failed at line %d ...\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (direction == MOVE_NEAR)
		step_direction = -1;
	else if (direction == MOVE_FAR)
		step_direction = 1;
	else {
		CDBG("%s failed at line %d ...\n", __func__, __LINE__);
		return -EINVAL;
	}

	actual_step = (int16_t) (step_direction * (int16_t) num_steps);
	next_position = (int16_t) (mt9p013_ctrl->curr_lens_pos + actual_step);

	if (next_position > MT9P013_TOTAL_STEPS_NEAR_TO_FAR)
		next_position = MT9P013_TOTAL_STEPS_NEAR_TO_FAR;
	else if (next_position < 0)
		next_position = 0;

	/* BAM move time */
	time_out = MT9P013_BAM_MOVE_BASE_MSEC +
		((MT9P013_BAM_MOVE_RANGE_MSEC * num_steps) /
		MT9P013_TOTAL_STEPS_NEAR_TO_FAR);

	code_val = next_position;
	actual_position_target = bam_step_lookup_table[code_val];
	CDBG("bam_step_lookup_table[%d]=%d\n", code_val, actual_position_target);

	rc = af_move(actual_position_target, time_out);

	/* Storing the current lens Position */
	mt9p013_ctrl->curr_lens_pos = next_position;

	CDBG("---%s:%d curr=%d\n", __func__, __LINE__, next_position);
	return rc;
}

static int32_t mt9p013_set_default_focus(void)
{
	int32_t rc = 0;

	af_move(bam_infinite, 140);

	mt9p013_ctrl->curr_lens_pos = MT9P013_TOTAL_STEPS_NEAR_TO_FAR;

	return rc;
}

static int mt9p013_probe_init_done( const struct msm_camera_sensor_info *data)
{
	return 0;
}


static int mt9p013_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t  rc;
	uint16_t chipid;
	uint16_t tempdata;
	CDBG("+++%s\n", __func__);

	/* enable mclk first */
	msm_camio_clk_rate_set(MT9P013_DEFAULT_CLOCK_RATE);

	msleep(20);

	/* 3. Read sensor Model ID: */
	rc = mt9p013_i2c_sensor_read_w(mt9p013_client->addr >> 1,
		MT9P013_REG_MODEL_ID, &chipid);
	if (rc < 0)
		goto init_probe_fail;
	/* 4. Compare sensor ID to MT9T013 ID: */
	if (chipid != MT9P013_MODEL_ID) {
		rc = -ENODEV;
		goto init_probe_fail;
	}

	// MT9P015 and MT9P013 have same model ID but can differentiate with below sequence
	rc = mt9p013_i2c_sensor_read_w(mt9p013_client->addr >> 1, 
		REG_RESET_REGISTER, &tempdata);
	if (rc < 0)
		goto init_probe_fail;

	tempdata |= 0x0020;

	rc = mt9p013_i2c_sensor_write_w(mt9p013_client->addr >> 1, 
		REG_RESET_REGISTER, tempdata);
	if (rc < 0)
		goto init_probe_fail;

	rc = mt9p013_i2c_sensor_read_w(mt9p013_client->addr >> 1,
		0x31FA, &tempdata);
	if (rc < 0)
		goto init_probe_fail;

	if (tempdata & 0x0E00) {
#ifdef SUPPORT_MT9P015
		printk(KERN_INFO "msm_camera: mt9p015 model id = 0x%x\n", chipid);
#else
		rc = -1;
		goto init_probe_fail;
#endif
	}
	else {
		printk(KERN_INFO "msm_camera: mt9p013 model id = 0x%x\n", chipid);
	}

	goto init_probe_done;

init_probe_fail:
	mt9p013_probe_init_done(data);
	CDBG("---%s failed rc = %d\n", __func__, rc);
init_probe_done:
	CDBG("---%s\n", __func__);
	return rc;
}

static int mt9p013_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t  rc;

	CDBG("+++%s\n", __func__);
	mt9p013_ctrl = kzalloc(sizeof(struct mt9p013_ctrl), GFP_KERNEL);
	if (!mt9p013_ctrl) {
		CDBG("mt9p013_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9p013_ctrl->fps_divider = 1 * 0x00000400;
	mt9p013_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9p013_ctrl->set_test = TEST_OFF;
	mt9p013_ctrl->prev_res = QTR_SIZE;
	mt9p013_ctrl->pict_res = FULL_SIZE;
    	mt9p013_ctrl->curr_lens_pos = 0;
	if (data)
		mt9p013_ctrl->sensordata = data;

	rc = mt9p013_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail1;

	if (mt9p013_ctrl->prev_res == QTR_SIZE)
		rc = mt9p013_setting(REG_INIT, RES_PREVIEW);
	else
		rc = mt9p013_setting(REG_INIT, RES_CAPTURE);

	if (rc < 0) {
		CDBG("%s failed. rc = %d\n", __func__, rc);
		goto init_fail1;
	}
	msleep(20);

	rc = af_init();

	if (rc < 0)
		goto init_fail1;

	rc = mt9p013_set_default_focus();

	if (rc >= 0)
		goto init_done;
init_fail1:
	mt9p013_probe_init_done(data);
	kfree(mt9p013_ctrl);
	CDBG("---%s failed rc = %d\n", __func__, rc);
init_done:
	CDBG("---%s\n", __func__);
	return rc;
}
static int mt9p013_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9p013_wait_queue);
	return 0;
}
static int32_t mt9p013_set_sensor_mode(int mode,
					int res)
{
	int32_t rc = 0;

	CDBG("%s:%d: mode=%d res=%d\n",__func__, __LINE__, mode, res);
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9p013_video_config(mode, res);
		break;

	case SENSOR_SNAPSHOT_MODE:
		rc = mt9p013_snapshot_config(mode);
		break;

	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = mt9p013_raw_snapshot_config(mode);
		break;

	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

int mt9p013_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	mutex_lock(&mt9p013_mut);

	CDBG("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		mt9p013_get_pict_fps(cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp, &cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = mt9p013_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = mt9p013_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = mt9p013_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl = mt9p013_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
			mt9p013_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = mt9p013_set_fps(&(cdata.cfg.fps));
		break;

	case CFG_SET_EXP_GAIN:
		rc = mt9p013_write_exp_gain(cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line,cdata.cfg.exp_gain.is_outdoor);
		break;

	case CFG_SET_PICT_EXP_GAIN:
		CDBG("Line:%d CFG_SET_PICT_EXP_GAIN \n", __LINE__);
		rc = mt9p013_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc = mt9p013_set_sensor_mode(
			cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = mt9p013_power_down();
		break;

	case CFG_MOVE_FOCUS:
		CDBG("mt9p013_ioctl: CFG_MOVE_FOCUS: dir=%d steps=%d\n",
			cdata.cfg.focus.dir, cdata.cfg.focus.steps);

		rc = mt9p013_move_focus(
			cdata.cfg.focus.dir,
			cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		CDBG("CFG_SET_DEFAULT_FOCUS\n");
		rc = mt9p013_set_default_focus();

		break;

	case CFG_SET_EFFECT:
//		rc = mt9p013_set_default_focus();
		break;

	case CFG_SET_LENS_SHADING:
//		CDBG("%s: CFG_SET_LENS_SHADING\n", __func__);
//		rc = mt9p013_lens_shading_enable(
//			cdata.cfg.lens_shading);
		break;

	case CFG_GET_AF_MAX_STEPS:
		cdata.max_steps = MT9P013_STEPS_NEAR_TO_CLOSEST_INF;
		CDBG("CFG_GET_AF_MAX_STEPS %d\n", cdata.max_steps);
		if (copy_to_user((void *)argp,
				 &cdata, sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&mt9p013_mut);
	return rc;
}

int mt9p013_sensor_release(void)
{
	int rc = -EBADF;

	mutex_lock(&mt9p013_mut);

	mt9p013_power_down();

	kfree(mt9p013_ctrl);
	mt9p013_ctrl = NULL;

	CDBG("%s completed\n", __func__);

	mutex_unlock(&mt9p013_mut);
	return rc;
}

static int mt9p013_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int rc = 0;
	struct mt9p013_platform_data *pdata = client->dev.platform_data;
	CDBG("+++%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9p013_sensorw = kzalloc(sizeof(struct mt9p013_work), GFP_KERNEL);
	if (!mt9p013_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}
	
	mt9p013_sensorw->init = pdata->init;
	mt9p013_sensorw->deinit = pdata->deinit;
	mt9p013_sensorw->power_shutdown = pdata->power_shutdown;
	mt9p013_sensorw->power_resume = pdata->power_resume;
	i2c_set_clientdata(client, mt9p013_sensorw);
	mt9p013_init_client(client);
	mt9p013_client = client;
	mdelay(50);

	if ((rc = device_create_file(&client->dev, &mt9p013_attr_regdata)))
		goto probe_failure;

	if ((rc = device_create_file(&client->dev, &mt9p013_attr_regaddr)))
		goto probe_failure;

	if ((rc = device_create_file(&client->dev, &mt9p013_attr_focus_position)))
		goto probe_failure;
	CDBG("---%s\n", __func__);
	return 0;

probe_failure:
	CDBG("---%s failed rc = %d\n", __func__, rc);
	return rc;
}

static const struct i2c_device_id mt9p013_i2c_id[] = {
	{"mt9p013", 0},
	{}
};


#ifdef CONFIG_PM
static int mt9p013_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc = 0;

	CDBG("%s\n", __func__);
	if (mt9p013_sensorw->power_shutdown)
		rc = mt9p013_sensorw->power_shutdown();

	return rc;
}

static int mt9p013_resume(struct i2c_client *client)
{
	int rc = 0;

	CDBG("%s\n", __func__);	
	if (mt9p013_sensorw->power_resume) {
		rc = mt9p013_sensorw->power_resume();
	}

	return rc;
}
#else
#define mt9p013_suspend NULL
#define mt9p013_resume  NULL
#endif /* CONFIG_PM */

static int __exit mt9p013_remove(struct i2c_client *client)
{
	struct mt9p013_work_t_t *sensorw = i2c_get_clientdata(client);
	device_remove_file(&client->dev, &mt9p013_attr_regdata);
	device_remove_file(&client->dev, &mt9p013_attr_regaddr);
	device_remove_file(&client->dev, &mt9p013_attr_focus_position);
	free_irq(client->irq, sensorw);
	mt9p013_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9p013_i2c_driver = {
	.id_table = mt9p013_i2c_id,
	.probe = mt9p013_i2c_probe,
	.remove = __exit_p(mt9p013_i2c_remove),
	.suspend = mt9p013_suspend,
	.resume = mt9p013_resume,
	.driver = {
		   .name = "mt9p013",
		   },
};

static int mt9p013_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc = i2c_add_driver(&mt9p013_i2c_driver);
	if (rc < 0 || mt9p013_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	rc = mt9p013_probe_init_sensor(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = mt9p013_sensor_open_init;
	s->s_release = mt9p013_sensor_release;
	s->s_config = mt9p013_sensor_config;
	mt9p013_probe_init_done(info);

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}


static int __mt9p013_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9p013_sensor_probe);
}
static struct platform_driver msm_camera_driver = {
	.probe = __mt9p013_probe,
	.driver = {
		   .name = "msm_camera_mt9p013",
		   .owner = THIS_MODULE,
		   },
};
static int __init mt9p013_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}
module_init(mt9p013_init);
