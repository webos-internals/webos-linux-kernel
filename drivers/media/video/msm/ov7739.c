/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "ov7739.h"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8    0x00000100

/* Omnivision8810 product ID register address */
#define REG_OV7739_MODEL_ID_MSB                       0x300A
#define REG_OV7739_MODEL_ID_LSB                       0x300B
#define REG_OV7739_MODEL_ID_MINOR                     0x501f

#define OV7739_MODEL_ID                       0x7739
#define OV7736_MODEL_ID                       0x7736
/* Omnivision8810 product ID */

/* Time in milisecs for waiting for the sensor to reset */
#define OV7739_RESET_DELAY_MSECS    66
#define OV7739_DEFAULT_CLOCK_RATE   24000000

/*============================================================================
							DATA DECLARATIONS
============================================================================*/
struct reg_addr_val_pair_struct ov7739_init_settings_array[] = {
	{0x3008, 0x82},
	{0x3008, 0x42},
	{0x3104, 0x03},
	{0x3017, 0x7f},
	{0x3018, 0xfc},
	{0x3602, 0x14},
	{0x3611, 0x44},
	{0x3631, 0x22},
	{0x3622, 0x00},
	{0x3633, 0x25},
	{0x370d, 0x07},
	{0x3620, 0x42},
	{0x3714, 0x19},
	{0x370b, 0x43},
	{0x3713, 0x1a},
	{0x401c, 0x00},
	{0x401e, 0x11},
	{0x4702, 0x01},
	{0x5000, 0x0e},
	{0x5001, 0x01},
	{0x3a00, 0x7a},
	{0x3a18, 0x00},
	{0x3a19, 0x3f},
	{0x300f, 0x88},
	{0x3011, 0x08},
	{0x5181, 0x04},
	{0x4303, 0xff},
	{0x4307, 0xff},
	{0x430b, 0xff},
	{0x4305, 0x00},
	{0x4309, 0x00},
	{0x430d, 0x00},
	{0x5000, 0x4f},
	{0x5001, 0x47},
	{0x4300, 0x30},
	{0x4301, 0x80},
	{0x501f, 0x01},
	{0x3017, 0x00},
	{0x3018, 0x00},
	{0x300e, 0x04},
	{0x4801, 0x0f},
	{0x4601, 0x02},
	{0x300f, 0x8a},
	{0x3011, 0x02}, //24MHz MCLK
	{0x3010, 0x00},
	{0x3818, 0x80},
	{0x4300, 0x3f},
	{0x3008, 0x02},
	{0x5180, 0x02},
	{0x5181, 0x02},
	{0x3a0f, 0x35},
	{0x3a10, 0x2c},
	{0x3a1b, 0x36},
	{0x3a1e, 0x2d},
	{0x3a11, 0x90},
	{0x3a1f, 0x10},
	{0x5000, 0xcf},
	{0x5481, 0x0a},
	{0x5482, 0x13},
	{0x5483, 0x23},
	{0x5484, 0x40},
	{0x5485, 0x4d},
	{0x5486, 0x58},
	{0x5487, 0x64},
	{0x5488, 0x6e},
	{0x5489, 0x78},
	{0x548a, 0x81},
	{0x548b, 0x92},
	{0x548c, 0xa1},
	{0x548d, 0xbb},
	{0x548e, 0xcf},
	{0x548f, 0xe3},
	{0x5490, 0x26},
	{0x5380, 0x42},
	{0x5381, 0x33},
	{0x5382, 0x0f},
	{0x5383, 0x0b},
	{0x5384, 0x42},
	{0x5385, 0x4d},
	{0x5392, 0x1e},
	{0x5801, 0x00},
	{0x5802, 0x06},
	{0x5803, 0x0a},
	{0x5804, 0x42},
	{0x5805, 0x2a},
	{0x5806, 0x25},
	{0x5001, 0xc7},
	{0x5580, 0x02},
	{0x5583, 0x40},
	{0x5584, 0x26},
	{0x5589, 0x10},
	{0x558a, 0x00},
	{0x558b, 0x3e},
	{0x5300, 0x0f},
	{0x5301, 0x30},
	{0x5302, 0x0d},
	{0x5303, 0x02},
	{0x5300, 0x0f},
	{0x5301, 0x30},
	{0x5302, 0x0d},
	{0x5303, 0x02},
	{0x5304, 0x0e},
	{0x5305, 0x30},
	{0x5306, 0x06},
	{0x5307, 0x40},
	{0x5680, 0x00},
	{0x5681, 0x50},
	{0x5682, 0x00},
	{0x5683, 0x3c},
	{0x5684, 0x11},
	{0x5685, 0xe0},
	{0x5686, 0x0d},
	{0x5687, 0x68},
	{0x5688, 0x03},
	{0x3008, 0x02},
};

/* 816x612, 24MHz MCLK 96MHz PCLK */
uint32_t OV7739_FULL_SIZE_WIDTH        = 640;
uint32_t OV7739_FULL_SIZE_HEIGHT       = 480;

uint32_t OV7739_QTR_SIZE_WIDTH         = 640;
uint32_t OV7739_QTR_SIZE_HEIGHT        = 480;

uint32_t OV7739_HRZ_FULL_BLK_PIXELS    = 16;
uint32_t OV7739_VER_FULL_BLK_LINES     = 12;
uint32_t OV7739_HRZ_QTR_BLK_PIXELS     = 16;
uint32_t OV7739_VER_QTR_BLK_LINES      = 12;

struct ov7739_work_t {
	struct work_struct work;
};
static struct  ov7739_work_t *ov7739_sensorw;
static struct  i2c_client *ov7739_client;
struct ov7739_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider;		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t fps;
	int32_t  curr_lens_pos;
	uint32_t curr_step_pos;
	uint32_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint32_t total_lines_per_frame;
	enum ov7739_resolution_t prev_res;
	enum ov7739_resolution_t pict_res;
	enum ov7739_resolution_t curr_res;
	enum ov7739_test_mode_t  set_test;
	unsigned short imgaddr;
};
static struct ov7739_ctrl_t *ov7739_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(ov7739_wait_queue);
DEFINE_MUTEX(ov7739_mut);
static uint16_t debug_regaddr = 0;

/*=============================================================*/

static int ov7739_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(ov7739_client->adapter, msgs, 2) < 0) {
		CDBG("ov7739_i2c_rxdata failed saddr=0x%x!\n", saddr);
		return -EIO;
	}
	return 0;
}
static int32_t ov7739_i2c_txdata(unsigned short saddr,
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
	if (i2c_transfer(ov7739_client->adapter, msg, 1) < 0) {
		CDBG("ov7739_i2c_txdata faild 0x%x\n", ov7739_client->addr);
		return -EIO;
	}

	return 0;
}

static int32_t ov7739_i2c_read(uint16_t raddr,
	uint8_t *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = ov7739_i2c_rxdata(ov7739_client->addr >> 1, buf, rlen);
	if (rc < 0) {
		CDBG("%s 0x%x failed!\n", __func__, raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("%s saddr=0x%x, raddr=0x%x rdata=0x%x\n", __func__, ov7739_client->addr, raddr, *rdata);
	return rc;
}
static int32_t ov7739_i2c_write_b(uint16_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	rc = ov7739_i2c_txdata(ov7739_client->addr >> 1, buf, 3);
	if (rc < 0)
		CDBG("%s failed, addr = 0x%x, val = 0x%x!\n", __func__,
			waddr, bdata);
	CDBG("%s addr = 0x%x, val = 0x%x\n", __func__, waddr, bdata);
	return rc;
}

static ssize_t
ov7739_attr_show_regdata(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	uint8_t val;
	int rc;
	
	CDBG("+++%s,%d: regaddr=0x%x\n",__func__,__LINE__, debug_regaddr);
	if ((rc = ov7739_i2c_read(debug_regaddr, &val,1)) < 0)
		goto exit;

	rc = sprintf(buf, "0x%02x\n", val);
	CDBG("---%s,%d: regddr=0x%x val=0x%x\n",__func__,__LINE__, debug_regaddr, val);
exit:
	return (rc);
}

static ssize_t
ov7739_attr_store_regdata(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned int val;

	CDBG("+++%s,%d\n",__func__,__LINE__);

	if (!sscanf(buf,"%02x\n", &val)) {
		rc = -EINVAL;
		goto exit;
	}
	
	CDBG("***%s,%d: regaddr=0x%x regdata=0x%x\n",__func__,__LINE__, debug_regaddr, val);
	if ((rc = ov7739_i2c_write_b(debug_regaddr, (uint8_t)val)) < 0)
		goto exit;

	rc = count;
	CDBG("---%s,%d\n",__func__,__LINE__);
exit:
	return (rc);
}

static ssize_t
ov7739_attr_show_regaddr(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int rc;
	rc = sprintf(buf, "0x%04x\n", debug_regaddr);
	return (rc);
}

static ssize_t
ov7739_attr_store_regaddr(struct device *dev, struct device_attribute *attr,
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

static struct device_attribute ov7739_attr_regdata =
	__ATTR(regdata, S_IRUGO|S_IWUGO, ov7739_attr_show_regdata,
		ov7739_attr_store_regdata);

static struct device_attribute ov7739_attr_regaddr =
	__ATTR(regaddr, S_IRUGO|S_IWUGO, ov7739_attr_show_regaddr,
		ov7739_attr_store_regaddr);


static int32_t ov7739_sensor_setting(int update_type, int rt)
{
	int32_t i, array_length;
	int32_t rc = 0;
	struct msm_camera_csi_params ov7739_csi_params;
	static int is_first = 1;

	switch (update_type) {
	case REG_INIT:
		CDBG("init start\n");
		rc = ov7739_i2c_write_b(0x3008, 0x82);
		if (rc < 0)
			return rc;
		is_first = 1;
		CDBG("init done\n");
		break;
	case UPDATE_PERIODIC:
		CDBG("update start rt=%d\n, rt");
		// for yuv sensor, only need to switch once as preview and snapshot has identical setting
		if ((rt == RES_PREVIEW) && (is_first == 1)) {
			ov7739_csi_params.lane_cnt = 1;
			ov7739_csi_params.data_format = CSI_8BIT;
			ov7739_csi_params.lane_assign = 0xe4;
			ov7739_csi_params.dpcm_scheme = 0;
			ov7739_csi_params.settle_cnt = 7;

			rc = msm_camio_csi_config(&ov7739_csi_params);
			if (rc < 0)
				return rc;
			msleep(10);
			array_length = sizeof(ov7739_init_settings_array) /
				sizeof(ov7739_init_settings_array[0]);
			for (i = 0; i < array_length; i++) {
				rc = ov7739_i2c_write_b(
					ov7739_init_settings_array[i].reg_addr,
					ov7739_init_settings_array[i].reg_val);
				if (rc < 0)
					return rc;
			}
			msleep(20);
			is_first = 0;
		}
		CDBG("update done\n");
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t ov7739_video_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	rt = RES_PREVIEW;

	if (ov7739_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	ov7739_ctrl->curr_res = ov7739_ctrl->prev_res;
	ov7739_ctrl->sensormode = mode;
	return rc;
}

static int32_t ov7739_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	rt = RES_CAPTURE;

	if (ov7739_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	ov7739_ctrl->curr_res = ov7739_ctrl->pict_res;
	ov7739_ctrl->sensormode = mode;
	return rc;
}
static int32_t ov7739_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	int rt;
	rt = RES_CAPTURE;

	if (ov7739_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	ov7739_ctrl->curr_res = ov7739_ctrl->prev_res;
	ov7739_ctrl->sensormode = mode;

	return rc;
}
static int32_t ov7739_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = ov7739_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
		rc = ov7739_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = ov7739_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}
static int32_t ov7739_power_down(void)
{
	return 0;
}
static int ov7739_probe_init_done(const struct msm_camera_sensor_info *data)
{
//	gpio_direction_output(data->sensor_reset, 0);
//	gpio_free(data->sensor_reset);
	return 0;
}

static int ov7739_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	uint8_t model_id_msb, model_id_lsb = 0, model_id_minor = 0;
	uint16_t model_id;
	int32_t rc = 0;

//	rc = gpio_request(data->sensor_reset, "ov7739");
//	if (!rc) {
//		CDBG("sensor_reset = %d\n", rc);
//		gpio_direction_output(data->sensor_reset, 0);
//		msleep(50);
//		gpio_direction_output(data->sensor_reset, 1);
//		msleep(50);
//	} else {
//		CDBG("gpio reset fail");
//		goto init_probe_done;
//	}

	/* According to spec, there needs to be a 20ms delay to ensure there is enough time 
  * between power-up and i2c use*/
	msleep(20);

	/* 3. Read sensor Model ID: */
	rc = ov7739_i2c_read(REG_OV7739_MODEL_ID_MSB, &model_id_msb, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s : %d : error reading model msb\n", __func__ , __LINE__ );
		goto init_probe_fail;
	}
	rc = ov7739_i2c_read(REG_OV7739_MODEL_ID_LSB, &model_id_lsb, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s : %d : error reading model lsb. msb:0x%X\n",
				__func__ , __LINE__ , model_id_msb);
                goto init_probe_fail;
	}

	model_id = (model_id_msb << 8) | ((model_id_lsb & 0x00FF)) ;

	/* Determine the difference between 7739 and 7736 sensor. This is for debugging
	 * due to some supplied devices reporting the wrong ID. Both are now supported. */
	rc = ov7739_i2c_read(REG_OV7739_MODEL_ID_MINOR, &model_id_minor, 1);
	if (rc < 0) {
		printk(KERN_ERR "%s : %d : error reading model ID minor. msb:0x%X lsb:0x%X\n",
				__func__ , __LINE__ , model_id_msb , model_id_lsb );
                goto init_probe_fail;
	}

	printk(KERN_INFO "msm_camera %s model_id = 0x%x, 0x%x, 0x%x, 0x%x\n",
                 model_id_minor ? "ov7736":"ov7739",
		 model_id, model_id_msb, model_id_lsb, model_id_minor);

	/* 4. Compare sensor ID to OV7739 ID: */
	if (model_id != OV7739_MODEL_ID && model_id != OV7736_MODEL_ID) {
		printk(KERN_ERR "%s : %d : Unknown model:0x%X\n",
				__func__ , __LINE__ , model_id);
		rc = -ENODEV;
		goto init_probe_fail;
	}
	msleep(10);
  goto init_probe_done;
init_probe_fail:
	printk(KERN_ERR " ov7739_probe_init_sensor fails; rc:%d\n" , rc );
//	gpio_direction_output(data->sensor_reset, 0);
//	gpio_free(data->sensor_reset);
init_probe_done:
	printk(KERN_INFO " ov7739_probe_init_sensor finishes\n");
	return rc;
}

int ov7739_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling ov7739_sensor_open_init\n");
	ov7739_ctrl = kzalloc(sizeof(struct ov7739_ctrl_t), GFP_KERNEL);
	if (!ov7739_ctrl) {
		CDBG("ov7739_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	ov7739_ctrl->fps_divider = 1 * 0x00000400;
	ov7739_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov7739_ctrl->fps = 30 * Q8;
	ov7739_ctrl->set_test = TEST_OFF;
	ov7739_ctrl->prev_res = QTR_SIZE;
	ov7739_ctrl->pict_res = FULL_SIZE;
	ov7739_ctrl->curr_res = INVALID_SIZE;

	if (data)
		ov7739_ctrl->sensordata = data;

	rc = ov7739_probe_init_sensor(data);
	if (rc < 0) {
		CDBG("Calling ov7739_sensor_open_init fail\n");
		goto init_fail;
	}

	rc = ov7739_sensor_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
init_fail:
	CDBG(" ov7739_sensor_open_init fail\n");
	ov7739_probe_init_done(data);
	kfree(ov7739_ctrl);
init_done:
	CDBG("ov7739_sensor_open_init done\n");
	return rc;
}

static int ov7739_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov7739_wait_queue);
	return 0;
}

static const struct i2c_device_id ov7739_i2c_id[] = {
	{"ov7739", 0},
	{ }
};

static int ov7739_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("ov7739_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	ov7739_sensorw = kzalloc(sizeof(struct ov7739_work_t), GFP_KERNEL);
	if (!ov7739_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov7739_sensorw);
	ov7739_init_client(client);
	ov7739_client = client;

	msleep(50);

	if ((rc = device_create_file(&client->dev, &ov7739_attr_regdata)))
		goto probe_failure;

	if ((rc = device_create_file(&client->dev, &ov7739_attr_regaddr)))
		goto probe_failure;

	CDBG("ov7739_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("ov7739_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit ov7739_remove(struct i2c_client *client)
{
	struct ov7739_work_t_t *sensorw = i2c_get_clientdata(client);
	device_remove_file(&client->dev, &ov7739_attr_regdata);
	device_remove_file(&client->dev, &ov7739_attr_regaddr);
	free_irq(client->irq, sensorw);
	ov7739_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov7739_i2c_driver = {
	.id_table = ov7739_i2c_id,
	.probe  = ov7739_i2c_probe,
	.remove = __exit_p(ov7739_i2c_remove),
	.driver = {
		.name = "ov7739",
	},
};

int ov7739_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&ov7739_mut);
	CDBG("ov7739_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_SET_MODE:
		rc = ov7739_set_sensor_mode(cdata.mode,
			cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = ov7739_power_down();
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&ov7739_mut);

	return rc;
}
static int ov7739_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&ov7739_mut);
	ov7739_power_down();
//	gpio_direction_output(ov7739_ctrl->sensordata->sensor_reset, 0);
//	gpio_free(ov7739_ctrl->sensordata->sensor_reset);
	kfree(ov7739_ctrl);
	ov7739_ctrl = NULL;
	CDBG("ov7739_release completed\n");
	mutex_unlock(&ov7739_mut);

	return rc;
}

static int ov7739_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	CDBG("%s: %d start\n", __func__, __LINE__);
	rc = i2c_add_driver(&ov7739_i2c_driver);
	if (rc < 0 || ov7739_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	msm_camio_clk_rate_set(OV7739_DEFAULT_CLOCK_RATE);
	rc = ov7739_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = ov7739_sensor_open_init;
	s->s_release = ov7739_sensor_release;
	s->s_config  = ov7739_sensor_config;
	ov7739_probe_init_done(info);

	CDBG("%s: %d end\n", __func__, __LINE__);
	return rc;

probe_fail:
	CDBG("ov7739_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&ov7739_i2c_driver);
	return rc;
}

static int __ov7739_probe(struct platform_device *pdev)
{

	return msm_camera_drv_start(pdev, ov7739_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov7739_probe,
	.driver = {
		.name = "msm_camera_ov7739",
		.owner = THIS_MODULE,
	},
};

static int __init ov7739_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov7739_init);

MODULE_DESCRIPTION("OMNI VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");

