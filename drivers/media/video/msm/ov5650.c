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
#include <linux/sysfs.h>
#include "ov5650.h"

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8    0x00000100

/* Omnivision8810 product ID register address */
#define OV5650_PIDH_REG 											0x300A
#define OV5650_PIDL_REG 											0x300B
/* Omnivision8810   product ID */
#define OV5650_PID  									0x56
/* Omnivision8810   version */
#define OV5650_VER  								 0x51
/* Time in milisecs for waiting for the sensor to   reset   */
#define OV5650_RESET_DELAY_MSECS		66
#define OV5650_DEFAULT_CLOCK_RATE   	12000000

/* Color bar pattern selection */
/* Color bar enabling control */
#define OV5650_TEST 	0x503d
/* Time in milisecs for waiting for the sensor to reset*/
#define OV5650_RESET_DELAY_MSECS	66
#define MAX_LINE_LENGTH_PCK 8190
/*============================================================================
		DATA DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */

enum ov5650_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3
};
enum ov5650_reg_update {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};
enum ov5650_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};
enum ov5650_setting {
	RES_PREVIEW,
	RES_CAPTURE
};


uint32_t OV5650_FULL_SIZE_DUMMY_PIXELS  =    0;
uint32_t OV5650_FULL_SIZE_DUMMY_LINES   =    0;
uint32_t OV5650_FULL_SIZE_WIDTH         = 2600;
uint32_t OV5650_FULL_SIZE_HEIGHT        =  1952;

uint32_t OV5650_QTR_SIZE_DUMMY_PIXELS   =    0;
uint32_t OV5650_QTR_SIZE_DUMMY_LINES    =    0;
uint32_t OV5650_QTR_SIZE_WIDTH      =  1300;
uint32_t OV5650_QTR_SIZE_HEIGHT     =  976;

uint32_t OV5650_HRZ_FULL_BLK_PIXELS =  652;
uint32_t OV5650_VER_FULL_BLK_LINES  =   16;
uint32_t OV5650_HRZ_QTR_BLK_PIXELS  =  1888;
uint32_t OV5650_VER_QTR_BLK_LINES   =   28;
static uint8_t ov5650_perview_offset = 4;
static uint8_t check = 0;
//static uint8_t ov5650_snapshot_offset = 3;

struct ov5650_work_t {
	struct work_struct work;
};
static struct  ov5650_work_t *ov5650_sensorw;
static struct  i2c_client *ov5650_client;
struct ov5650_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider;		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */
	uint16_t fps;
	int16_t  curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;
	enum ov5650_resolution_t prev_res;
	enum ov5650_resolution_t pict_res;
	enum ov5650_resolution_t curr_res;
	enum ov5650_test_mode_t  set_test;
	unsigned short imgaddr;
};
static struct ov5650_ctrl_t *ov5650_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(ov5650_wait_queue);
DECLARE_MUTEX(ov5650_sem);

static u16 debug_regaddr = 0;

/*=============================================================*/
static int ov5650_i2c_rxdata(unsigned short saddr,
							 unsigned char *rxdata, int length)
{
	/*Kalyani*/
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

	if (i2c_transfer(ov5650_client->adapter, msgs, 2) < 0) {
		CDBG("ov5650_i2c_rxdata failed 0x%x!\n", saddr);
		return -EIO;
	}

	return 0;
}
static int32_t ov5650_i2c_txdata(unsigned short saddr,
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

	if (i2c_transfer(ov5650_client->adapter, msg, 1) < 0) {
		CDBG("ov5650_i2c_txdata faild 0x%x\n", ov5650_client->addr);
		return -EIO;
	}

	return 0;
}


static int32_t ov5650_i2c_read(unsigned short raddr,
							   unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);

	rc = ov5650_i2c_rxdata(ov5650_client->addr>>1, buf, rlen);

	if (rc < 0) {
		CDBG("ov5650_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}

	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);

	return rc;

}
static int32_t ov5650_i2c_write_b(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;

	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = ov5650_i2c_txdata(ov5650_client->addr>>1, buf, 3);

	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			 waddr, bdata);
	}

	return rc;
}

static ssize_t
ov5650_attr_show_regdata(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	unsigned short val;
	int rc;
	
	CDBG("+++%s,%d: regaddr=0x%x\n",__func__,__LINE__, debug_regaddr);
	if ((rc = ov5650_i2c_read(debug_regaddr, &val,1)) < 0)
		goto exit;

	rc = sprintf(buf, "0x%02x\n", val);
	CDBG("---%s,%d: regddr=0x%x val=0x%x\n",__func__,__LINE__, debug_regaddr, val);
exit:
	return (rc);
}

static ssize_t
ov5650_attr_store_regdata(struct device *dev, struct device_attribute *attr,
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
	if ((rc = ov5650_i2c_write_b(debug_regaddr, (uint8_t)val)) < 0)
		goto exit;

	rc = count;
	CDBG("---%s,%d\n",__func__,__LINE__);
exit:
	return (rc);
}

static ssize_t
ov5650_attr_show_regaddr(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int rc;
	rc = sprintf(buf, "0x%04x\n", debug_regaddr);
	return (rc);
}

static ssize_t
ov5650_attr_store_regaddr(struct device *dev, struct device_attribute *attr,
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

static struct device_attribute ov5650_attr_regdata =
	__ATTR(regdata, S_IRUGO|S_IWUGO, ov5650_attr_show_regdata,
		ov5650_attr_store_regdata);

static struct device_attribute ov5650_attr_regaddr =
	__ATTR(regaddr, S_IRUGO|S_IWUGO, ov5650_attr_show_regaddr,
		ov5650_attr_store_regaddr);

static int32_t ov5650_move_focus( int direction,
								  int32_t num_steps)
{

	return 0;
}
static int32_t ov5650_set_default_focus(uint8_t af_step)
{

	return 0;
}
static void ov5650_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
#if 0
	uint16_t snapshot_height, preview_height, preview_width,snapshot_width;
	uint16_t snapshot_fps = fps;

	uint32_t divider;

	if (ov5650_ctrl->prev_res == QTR_SIZE) {
		preview_width = OV5650_QTR_SIZE_WIDTH  + OV5650_HRZ_QTR_BLK_PIXELS ;
		preview_height= OV5650_QTR_SIZE_HEIGHT + OV5650_VER_QTR_BLK_LINES ;
	} else {
		preview_width = OV5650_FULL_SIZE_WIDTH + OV5650_HRZ_FULL_BLK_PIXELS ;
		preview_height= OV5650_FULL_SIZE_HEIGHT + OV5650_VER_FULL_BLK_LINES ;
	}


	if (ov5650_ctrl->pict_res == QTR_SIZE) {
		snapshot_width  = OV5650_QTR_SIZE_WIDTH + OV5650_HRZ_QTR_BLK_PIXELS ;
		snapshot_height = OV5650_QTR_SIZE_HEIGHT + OV5650_VER_QTR_BLK_LINES ;
	} else {

		snapshot_width  = OV5650_FULL_SIZE_WIDTH + OV5650_HRZ_FULL_BLK_PIXELS;
		snapshot_height = OV5650_FULL_SIZE_HEIGHT + OV5650_VER_FULL_BLK_LINES;
	}

	divider = (snapshot_height*snapshot_width * 0x400)/(preview_height*preview_width);
	snapshot_fps = (uint16_t)((fps *0x400)/(divider));

	// This is a hack.
	*pfps =15;// snapshot_fps *2;
#endif

#if 1
	uint16_t preview_frame_length_lines, snapshot_frame_length_lines;
	uint16_t preview_line_length_pck, snapshot_line_length_pck;
	uint32_t divider, d1, d2;
	//uint16_t tempfps;
	/* Total frame_length_lines and line_length_pck for preview */
	preview_frame_length_lines = OV5650_QTR_SIZE_HEIGHT +
								 OV5650_VER_QTR_BLK_LINES;
	preview_line_length_pck = OV5650_QTR_SIZE_WIDTH +
							  OV5650_HRZ_QTR_BLK_PIXELS;
	/* Total frame_length_lines and line_length_pck for snapshot */
	snapshot_frame_length_lines = OV5650_FULL_SIZE_HEIGHT +
								  OV5650_VER_FULL_BLK_LINES;
	snapshot_line_length_pck = OV5650_FULL_SIZE_WIDTH +
							   OV5650_HRZ_FULL_BLK_PIXELS;
	d1 = preview_frame_length_lines * 0x00000400 /
		 snapshot_frame_length_lines;
	d2 = preview_line_length_pck * 0x00000400 /
		 snapshot_line_length_pck;
	divider = d1 * d2 / 0x400;

	*pfps = (uint16_t) (fps * divider / 0x400);

#endif
#if 0
	tempfps = (tempfps *188 * 0x400)/100;
	= tempfps/0x400;
#endif
#if 0
	uint32_t divider;	/*Q10 */
	//uint32_t d1;
	//uint32_t d2;
	uint16_t tempfps;
	uint16_t snapshot_height, preview_height, preview_width,snapshot_width;
	if (ov5650_ctrl->prev_res == QTR_SIZE) {
		preview_width = OV5650_QTR_SIZE_WIDTH  + OV5650_HRZ_QTR_BLK_PIXELS ;
		preview_height= OV5650_QTR_SIZE_HEIGHT + OV5650_VER_QTR_BLK_LINES ;
	} else {
		/* full size resolution used for preview. */
		preview_width = OV5650_FULL_SIZE_WIDTH + OV5650_HRZ_FULL_BLK_PIXELS ;
		preview_height= OV5650_FULL_SIZE_HEIGHT + OV5650_VER_FULL_BLK_LINES ;
	}
	if (ov5650_ctrl->pict_res == QTR_SIZE) {
		snapshot_width  = OV5650_QTR_SIZE_WIDTH + OV5650_HRZ_QTR_BLK_PIXELS ;
		snapshot_height = OV5650_QTR_SIZE_HEIGHT + OV5650_VER_QTR_BLK_LINES ;
	} else {
		snapshot_width  = OV5650_FULL_SIZE_WIDTH + OV5650_HRZ_FULL_BLK_PIXELS;
		snapshot_height = OV5650_FULL_SIZE_HEIGHT + OV5650_VER_FULL_BLK_LINES;
	}
	divider = (uint32_t)((snapshot_height*snapshot_width)/(preview_height*preview_width));
	tempfps = (uint16_t)((fps)/divider);
	tempfps = (tempfps *188 * 0x400)/100;
	*pfps = tempfps/0x400;
#endif



}/*endof ov5650_get_pict_fps*/

static uint16_t ov5650_get_prev_lines_pf(void)
{
	if (ov5650_ctrl->prev_res == QTR_SIZE) {
		return(OV5650_QTR_SIZE_HEIGHT + OV5650_VER_QTR_BLK_LINES);
	} else {
		return(OV5650_FULL_SIZE_HEIGHT + OV5650_VER_FULL_BLK_LINES);
	}
}

static uint16_t ov5650_get_prev_pixels_pl(void)
{
	if (ov5650_ctrl->prev_res == QTR_SIZE) {
		return(OV5650_QTR_SIZE_WIDTH + OV5650_HRZ_QTR_BLK_PIXELS);
	} else {
		return(OV5650_FULL_SIZE_WIDTH + OV5650_HRZ_FULL_BLK_PIXELS);
	}
}
static uint16_t ov5650_get_pict_lines_pf(void)
{
	if (ov5650_ctrl->pict_res == QTR_SIZE) {
		return(OV5650_QTR_SIZE_HEIGHT + OV5650_VER_QTR_BLK_LINES);
	} else {
		return(OV5650_FULL_SIZE_HEIGHT + OV5650_VER_FULL_BLK_LINES);
	}
}
static uint16_t ov5650_get_pict_pixels_pl(void)
{
	if (ov5650_ctrl->pict_res == QTR_SIZE) {
		return(OV5650_QTR_SIZE_WIDTH + OV5650_HRZ_QTR_BLK_PIXELS);
	} else {
		return(OV5650_FULL_SIZE_WIDTH + OV5650_HRZ_FULL_BLK_PIXELS);
	}
}

static uint32_t ov5650_get_pict_max_exp_lc(void)
{
	if (ov5650_ctrl->pict_res == QTR_SIZE) {
		return(OV5650_QTR_SIZE_HEIGHT + OV5650_VER_QTR_BLK_LINES)*24;
	} else {
		return 8644;
	}
}

static int32_t ov5650_set_fps(struct fps_cfg    *fps)
{
	int32_t rc = 0;
	ov5650_ctrl->fps_divider = fps->fps_div;
	ov5650_ctrl->pict_fps_divider = fps->pict_fps_div;
	ov5650_ctrl->fps = fps->f_mult;
	return rc;
}

static int32_t ov5650_write_exp_gain(uint16_t gain, uint32_t line)
{
	static uint16_t max_legal_gain  = 0x0070;

	uint16_t line_length_pck, frame_length_lines;
	uint8_t line_length_pck_hi = 0, line_length_pck_lo = 0;
	uint16_t line_length_ratio = 1 * Q8;
	int32_t rc = 0;
	if (ov5650_ctrl->sensormode != SENSOR_SNAPSHOT_MODE) {
		frame_length_lines = OV5650_QTR_SIZE_HEIGHT +
							 OV5650_VER_QTR_BLK_LINES;
		line_length_pck = OV5650_QTR_SIZE_WIDTH +
						  OV5650_HRZ_QTR_BLK_PIXELS;
		if (line > (frame_length_lines -
					ov5650_perview_offset)) {
			ov5650_ctrl->fps = (uint16_t) (30 * Q8 *
										   (frame_length_lines - ov5650_perview_offset)/
										   line);
		} else {
			ov5650_ctrl->fps = (uint16_t) (30 * Q8);
		}
	} else {
		frame_length_lines = OV5650_FULL_SIZE_HEIGHT +
							 OV5650_VER_FULL_BLK_LINES;
		line_length_pck = OV5650_FULL_SIZE_WIDTH +
						  OV5650_HRZ_FULL_BLK_PIXELS;
	}
	/* calculate line_length_ratio */
	if ((frame_length_lines - ov5650_perview_offset) < line) {
		line_length_ratio = (line*Q8) /
							(frame_length_lines - ov5650_perview_offset);
		line = frame_length_lines - ov5650_perview_offset;
	} else {
		line_length_ratio = 1*Q8;
	}
	if (ov5650_i2c_write_b(0x3212, 0x00) < 0) //enable group latch
		return rc;
line_length_pck = (line_length_pck >
			MAX_LINE_LENGTH_PCK) ?
			MAX_LINE_LENGTH_PCK : line_length_pck;
		line_length_pck = (uint16_t) (line_length_pck *
			line_length_ratio/Q8);
		CDBG("Line lenght pckl is %d\n",line_length_pck);
		line_length_pck_hi = (uint8_t) ((line_length_pck &
			0xFF00) >> 8);
		line_length_pck_lo = (uint8_t) (line_length_pck &
			0x00FF);
	// Update the total width
	if (ov5650_i2c_write_b(0x380c, line_length_pck_hi) < 0)
		return rc;
	if (ov5650_i2c_write_b(0x380d, line_length_pck_lo) < 0)
		return rc;
	if (gain > max_legal_gain)
		gain = max_legal_gain;
	// Write to gain register
	if (ov5650_i2c_write_b(0x350B, (uint8_t)gain) < 0)
		return rc;

	/* update line count registers */
	if (ov5650_i2c_write_b(0x3500, (uint8_t)(((uint16_t)line & 0xF000) >> 12)) < 0)
		return rc;

	if (ov5650_i2c_write_b(0x3501, (uint8_t)(((uint16_t)line & 0x0FF0) >> 4)) < 0)
		return rc;

	if (ov5650_i2c_write_b(0x3502, (uint8_t)(((uint16_t)line & 0x000F) << 4)) < 0)
		return rc;

	if (ov5650_i2c_write_b(0x3212, 0x10) < 0) //Group latch end
		return rc;
	if (ov5650_i2c_write_b(0x3212, 0xA0) < 0) //launch group
		return rc;
	return rc;
}
static int32_t ov5650_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = ov5650_write_exp_gain(gain, line);
	return rc;
}/* endof ov5650_set_pict_exp_gain*/

static int32_t ov5650_setting(int update_type, int rt)
{
	int32_t rc = 0;
	int i;
	struct msm_camera_csi_params ov5650_csi_params;

	switch (update_type) {
	case REG_INIT:
                ov5650_i2c_write_b( 0x3008, 0x82);
		ov5650_i2c_write_b( 0x3008, 0x42);
#if 0
		ov5650_csi_params.data_format = CSI_10BIT;
		ov5650_csi_params.lane_cnt = 2;
		ov5650_csi_params.lane_assign = 0xe4;
		ov5650_csi_params.dpcm_scheme = 0;
		ov5650_csi_params.settle_cnt = 20;


		rc = msm_camio_csi_config(&ov5650_csi_params);
		msleep(60);

		for (i=0; i<array_length; i++) {
			rc = ov5650_i2c_write_b( ov5650_init_settings_array[i].reg_addr, 
				ov5650_init_settings_array[i].reg_val);
			if ( rc < 0) {
				return rc;
			}
		}
#endif
		break;

	case UPDATE_PERIODIC:

		if (rt == RES_PREVIEW ) {
					if (check == 0) {
		
		CDBG("Kalyani configuring for preview\n");
		CDBG("Kalyani doing csi config\n");
		ov5650_csi_params.data_format = CSI_10BIT;
		ov5650_csi_params.lane_cnt = 2;
		ov5650_csi_params.lane_assign = 0xe4;
		ov5650_csi_params.dpcm_scheme = 0;
		ov5650_csi_params.settle_cnt = 20;
	
		CDBG("Kalyani here\n");
		rc = msm_camio_csi_config(&ov5650_csi_params);
			msleep(100);
			//check =1;
					}
			for (i=0; i<array_length; i++){
				rc = ov5650_i2c_write_b(ov5650_init_settings_array[i].reg_addr,
				ov5650_init_settings_array[i].reg_val);
				if ( rc < 0) 
					return rc;
	
			}
			ov5650_i2c_write_b( 0x3008, 0x02); 
			msleep(80);

		}
		else if (rt == RES_CAPTURE) {
			CDBG("Kalyani configuring for snapshot\n");

		ov5650_csi_params.data_format = CSI_10BIT;
		ov5650_csi_params.lane_cnt = 2;
		ov5650_csi_params.lane_assign = 0xe4;
		ov5650_csi_params.dpcm_scheme = 0;
		ov5650_csi_params.settle_cnt = 20;
				for (i=0; i<snapshot_array_length; i++){
				rc = ov5650_i2c_write_b(ov5650_snapshot_array[i].reg_addr,
				ov5650_snapshot_array[i].reg_val);
				if ( rc < 0) 
					return rc;
	
			}
				msleep(60);
		}
		
		CDBG("Kalyani done\n");
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int32_t ov5650_video_config(int mode, int rt )
{
	int32_t rc = 0;

	/* change sensor resolution if needed */
	if (ov5650_ctrl->prev_res == QTR_SIZE) {
		rt = RES_PREVIEW;

	} else {
		rt = RES_CAPTURE;

	}
	if (ov5650_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	ov5650_ctrl->curr_res = ov5650_ctrl->prev_res;
	ov5650_ctrl->sensormode = mode;
	return rc;

}/*end of ov354_video_config*/

static int32_t ov5650_snapshot_config(int mode,int rt)
{
	int32_t rc = 0;

	/*change sensor resolution if needed */
	if (ov5650_ctrl->curr_res != ov5650_ctrl->pict_res) {
		if (ov5650_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;

		} else {
			rt = RES_CAPTURE;

		}
		if (ov5650_setting(UPDATE_PERIODIC, rt) < 0)
			return rc;
	}

	ov5650_ctrl->curr_res = ov5650_ctrl->pict_res;
	ov5650_ctrl->sensormode = mode;
	return rc;
}/*end of ov5650_snapshot_config*/

static int32_t ov5650_raw_snapshot_config(int mode,int rt)
{
	int32_t rc = 0;
	/*change sensor resolution if needed */
	if (ov5650_ctrl->curr_res != ov5650_ctrl->pict_res) {
		if (ov5650_ctrl->pict_res == QTR_SIZE) {
			rt = RES_PREVIEW;

		} else {
			rt = RES_CAPTURE;

		}
		if (ov5650_setting(UPDATE_PERIODIC, rt) < 0)
			return rc;
	}

	ov5650_ctrl->curr_res = ov5650_ctrl->pict_res;
	ov5650_ctrl->sensormode = mode;
	return rc;
}/*end of ov5650_raw_snapshot_config*/
static int32_t ov5650_set_sensor_mode(int  mode,
									  int  res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = ov5650_video_config(mode,res);
		break;
	case SENSOR_SNAPSHOT_MODE:
		rc = ov5650_snapshot_config(mode,res);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		rc = ov5650_raw_snapshot_config(mode,res);
		break;

	default:
		rc =    -EINVAL;
		break;
	}
	return rc;
}
static int32_t ov5650_power_down(void)
{
	return 0;
}
static int ov5650_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t  chipidl, chipidh;
//	msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
//	msm_camio_clk_rate_set(OV5650_DEFAULT_CLOCK_RATE);
	mdelay(10);
//	rc = gpio_request(data->sensor_reset, "ov5650");
//	if (!rc)
//		gpio_direction_output(data->sensor_reset, 1);
//	else
//		goto init_probe_done;
	mdelay(20);
	CDBG("%s,%d reset io=%d\n",__func__,__LINE__, data->sensor_reset);
	/* 3. Read sensor Model ID: */
	if (ov5650_i2c_read(OV5650_PIDH_REG, &chipidh,1) < 0)
		goto init_probe_fail;
	if (ov5650_i2c_read(OV5650_PIDL_REG, &chipidl,1) < 0)
		goto init_probe_fail;
	printk(KERN_INFO "kov5650 model_id = 0x%x  0x%x\n", chipidh, chipidl);
	/* 4. Compare sensor ID to OV5650 ID: */
	if (chipidh != OV5650_PID) {
		rc = -ENODEV;
		printk(KERN_INFO "Kalyani probeinit fail\n");
		goto init_probe_fail;
	}
	mdelay(OV5650_RESET_DELAY_MSECS);
//	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	CDBG("%s,%d\n",__func__,__LINE__);
	goto init_probe_done;

	init_probe_fail:
	CDBG("%s,%d\n",__func__,__LINE__);
//	gpio_direction_output(data->sensor_reset, 0);
//	gpio_free(data->sensor_reset);
	init_probe_done:
	CDBG("%s,%d\n",__func__,__LINE__);
	printk(KERN_INFO " ov5650_probe_init_sensor finishes\n");
	return rc;
}
int ov5650_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t  rc;

	CDBG("Calling ov5650_sensor_open_init\n");
	ov5650_ctrl = kzalloc(sizeof(struct ov5650_ctrl_t), GFP_KERNEL);
	if (!ov5650_ctrl) {
		CDBG("ov5650_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	ov5650_ctrl->curr_lens_pos = -1;
	ov5650_ctrl->fps_divider = 1 * 0x00000400;
	ov5650_ctrl->pict_fps_divider = 1 * 0x00000400;
	ov5650_ctrl->set_test = TEST_OFF;
	ov5650_ctrl->prev_res = QTR_SIZE;
	ov5650_ctrl->pict_res = FULL_SIZE;
	ov5650_ctrl->curr_res = INVALID_SIZE;
	if (data)
		ov5650_ctrl->sensordata = data;
	/* enable mclk first */
	msm_camio_clk_rate_set(OV5650_DEFAULT_CLOCK_RATE);
	mdelay(20);
	rc = ov5650_probe_init_sensor(data);
	CDBG("rc = %d\n",rc);
	if (rc < 0)
		goto init_fail;
	/* Initialize Sensor registers */
	rc = ov5650_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0) {
		return rc;
	}

	ov5650_ctrl->fps = 30*Q8;
	/* generate test pattern */
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
	/* reset the driver state */
	init_fail:
	CDBG("init_fail \n");
	kfree(ov5650_ctrl);
	init_done:
	CDBG("init_done \n");
	return rc;
}/*endof ov5650_sensor_open_init*/
static int ov5650_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov5650_wait_queue);
	return 0;
}

static const struct i2c_device_id ov5650_i2c_id[] = {
	{ "ov5650", 0},
	{}
};

static int ov5650_i2c_probe(struct i2c_client *client,
							const struct i2c_device_id *id)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}
	ov5650_sensorw = kzalloc(sizeof(struct ov5650_work_t), GFP_KERNEL);
	if (!ov5650_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}
	i2c_set_clientdata(client, ov5650_sensorw);
	ov5650_init_client(client);
	ov5650_client = client;
	mdelay(50);

	if ((rc = device_create_file(&client->dev, &ov5650_attr_regdata)))
		goto probe_failure;

	if ((rc = device_create_file(&client->dev, &ov5650_attr_regaddr)))
		goto probe_failure;

	CDBG("ov5650_probe successed! rc = %d\n", rc);
	return 0;
	probe_failure:
	CDBG("ov5650_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit ov5650_remove(struct i2c_client *client)
{
	struct ov5650_work_t_t *sensorw = i2c_get_clientdata(client);
	device_remove_file(&client->dev, &ov5650_attr_regdata);
	device_remove_file(&client->dev, &ov5650_attr_regaddr);
	free_irq(client->irq, sensorw);
	ov5650_client = NULL;
	CDBG("Kalyani ov5650_remove\n");
	kfree(sensorw);
	return 0;
}

static struct i2c_driver ov5650_i2c_driver = {
	.id_table = ov5650_i2c_id,
	.probe  = ov5650_i2c_probe,
	.remove = __exit_p(ov5650_i2c_remove),
			  .driver = {
		.name = "ov5650",
	},
};

int ov5650_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
					   (void *)argp,
					   sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	down(&ov5650_sem);
	CDBG("ov5650_sensor_config: cfgtype = %d\n",
		 cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		ov5650_get_pict_fps(cdata.cfg.gfps.prevfps,
							&(cdata.cfg.gfps.pictfps));
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = ov5650_get_prev_lines_pf();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = ov5650_get_prev_pixels_pl();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = ov5650_get_pict_lines_pf();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl =
		ov5650_get_pict_pixels_pl();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc = ov5650_get_pict_max_exp_lc();
		if (copy_to_user((void *)argp,
						 &cdata,
						 sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = ov5650_set_fps(&(cdata.cfg.fps));
		break;
	case CFG_SET_EXP_GAIN:
		rc = ov5650_write_exp_gain(
								  cdata.cfg.exp_gain.gain,
								  cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_PICT_EXP_GAIN:
		rc = ov5650_set_pict_exp_gain(
									 cdata.cfg.exp_gain.gain,
									 cdata.cfg.exp_gain.line);
		break;
	case CFG_SET_MODE:
		rc = ov5650_set_sensor_mode(cdata.mode,
									cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = ov5650_power_down();
		break;
	case CFG_MOVE_FOCUS:
		rc = ov5650_move_focus(
							  cdata.cfg.focus.dir,
							  cdata.cfg.focus.steps);
		break;
	case CFG_SET_DEFAULT_FOCUS:
		rc = ov5650_set_default_focus(
									 cdata.cfg.focus.steps);
		break;
	case CFG_SET_EFFECT:
		rc = ov5650_set_default_focus(
									 cdata.cfg.effect);
		break;
	default:
		rc = -EFAULT;
		break;
	}
	up(&ov5650_sem);
	return rc;
}
static int ov5650_probe_init_done(const struct msm_camera_sensor_info *data)
{
//	gpio_direction_output(data->sensor_reset, 0);
//	gpio_free(data->sensor_reset);
	return 0;
}

static int ov5650_sensor_release(void)
{
	int rc = -EBADF;
	down(&ov5650_sem);
	ov5650_power_down();
//	gpio_direction_output(ov5650_ctrl->sensordata->sensor_reset,
//						  0);
//	gpio_free(ov5650_ctrl->sensordata->sensor_reset);
	kfree(ov5650_ctrl);
	ov5650_ctrl = NULL;
	up(&ov5650_sem);
	return rc;
}

static int ov5650_sensor_probe(const struct msm_camera_sensor_info *info,
							   struct msm_sensor_ctrl *s)
{
	int rc = 0;
	CDBG("%s: %d start\n", __func__, __LINE__);
	rc = i2c_add_driver(&ov5650_i2c_driver);
	if (rc < 0 || ov5650_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	msm_camio_clk_rate_set(12000000);
	mdelay(20);
	rc = ov5650_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;

	s->s_init = ov5650_sensor_open_init;
	s->s_release = ov5650_sensor_release;
	s->s_config  = ov5650_sensor_config;
	ov5650_probe_init_done(info);

	CDBG("%s: %d end\n", __func__, __LINE__);
	return rc;

	probe_fail:
	CDBG("SENSOR PROBE FAILS!\n");
	return rc;
}

static int __ov5650_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov5650_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov5650_probe,
	.driver = {
		.name = "msm_camera_ov5650",
		.owner = THIS_MODULE,
	},
};

static int __init ov5650_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(ov5650_init);
void ov5650_exit(void)
{
	i2c_del_driver(&ov5650_i2c_driver);
}

