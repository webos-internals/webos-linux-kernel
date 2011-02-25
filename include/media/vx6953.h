/*
 * Copyright (C) 2008-2009 Palm, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _MEDIA_VX6953_H
#define _MEDIA_VX6953_H

#include <linux/videodev2.h>

enum {
	VX6953_CTRL_EXT_CLK = 0,
	VX6953_CTRL_PIXEL_CLOCK,
	VX6953_CTRL_MODE_SELECT,
	VX6953_CTRL_IMAGE_ORIENTATION,
	VX6953_CTRL_GROUPED_PARAMETER_HOLD,
	VX6953_CTRL_CCP2_DATA_FORMAT,
	VX6953_CTRL_FINE_INTEGRATION_TIME,
	VX6953_CTRL_COARSE_INTEGRATION_TIME,
	VX6953_CTRL_ANALOGUE_GAIN_CODE_GLOBAL,
	VX6953_CTRL_DIGITAL_GAIN_GREENR,
	VX6953_CTRL_DIGITAL_GAIN_RED,
	VX6953_CTRL_DIGITAL_GAIN_BLUE,
	VX6953_CTRL_DIGITAL_GAIN_GREENB,
	VX6953_CTRL_VT_PIX_CLK_DIV,
	VX6953_CTRL_VT_SYS_CLK_DIV,
	VX6953_CTRL_PRE_PLL_CLK_DIV,
	VX6953_CTRL_PLL_MULTIPLIER,
	VX6953_CTRL_OP_SYS_CLK_DIV,
	VX6953_CTRL_OP_PIX_CLK_DIV,
	VX6953_CTRL_FRAME_LENGTH_LINES,
	VX6953_CTRL_CSI_SIGNALING_MODE,
	VX6953_CTRL_LINE_LENGTH_PCK,
	VX6953_CTRL_X_ADDR_START,
	VX6953_CTRL_Y_ADDR_START,
	VX6953_CTRL_X_ADDR_END,
	VX6953_CTRL_Y_ADDR_END,
	VX6953_CTRL_X_OUTPUT_SIZE,
	VX6953_CTRL_Y_OUTPUT_SIZE,
	VX6953_CTRL_X_EVEN_INC,
	VX6953_CTRL_X_ODD_INC,
	VX6953_CTRL_Y_EVEN_INC,
	VX6953_CTRL_Y_ODD_INC,
	VX6953_CTRL_MIN_FRAME_LENGTH_LINES,
	VX6953_CTRL_MAX_FRAME_LENGTH_LINES,
	VX6953_CTRL_MIN_FRAME_BLANKING_LINES,
	VX6953_CTRL_BINNING_MODE,
	VX6953_CTRL_BINNING_TYPE,
	VX6953_CTRL_BINNING_WEIGHTING,
	VX6953_CTRL_SHADING_CORRECTION,
	VX6953_CTRL_MAN_SPEC_DERATE_ENABLE,
	VX6953_CTRL_HOST_WB_STATS_GREEN_RED,
	VX6953_CTRL_HOST_WB_STATS_RED,
	VX6953_CTRL_HOST_WB_STATS_BLUE,
	VX6953_CTRL_HOST_WB_STATS_GREEN_BLUE,
	VX6953_CTRL_SET_TIME,
	VX6953_CTRL_COUNT
};

enum {
	VX6953_IMAGE_ORIENTATION_NORMAL = 0,
	VX6953_IMAGE_ORIENTATION_HORIZONTAL_MIRROR = 1,
	VX6953_IMAGE_ORIENTATION_VERTICAL_FLIP = 2,
	VX6953_IMAGE_ORIENTATION_MIRROR_AND_FLIP = 3,
};

/* TODO: revisit */
#define	V4L2_CID_VX6953_BASE		(V4L2_CID_PRIVATE_BASE | 0x520000)
#define V4L2_CID_VX6953(index)		(V4L2_CID_VX6953_BASE + (index))

#define V4L2_CID_VX6953_PIXEL_CLOCK \
	V4L2_CID_VX6953(VX6953_CTRL_PIXEL_CLOCK)

#define V4L2_CID_VX6953_IMAGE_ORIENTATION \
	V4L2_CID_VX6953(VX6953_CTRL_IMAGE_ORIENTATION)
	
#define V4L2_CID_VX6953_GROUPED_PARAMETER_HOLD \
	V4L2_CID_VX6953(VX6953_CTRL_GROUPED_PARAMETER_HOLD)

#define V4L2_CID_VX6953_FINE_INTEGRATION_TIME \
	V4L2_CID_VX6953(VX6953_CTRL_FINE_INTEGRATION_TIME)

#define V4L2_CID_VX6953_COARSE_INTEGRATION_TIME \
	V4L2_CID_VX6953(VX6953_CTRL_COARSE_INTEGRATION_TIME)

#define V4L2_CID_VX6953_ANALOGUE_GAIN_CODE_GLOBAL \
	V4L2_CID_VX6953(VX6953_CTRL_ANALOGUE_GAIN_CODE_GLOBAL)
	
#define V4L2_CID_VX6953_DIGITAL_GAIN_GREENR \
	V4L2_CID_VX6953(VX6953_CTRL_DIGITAL_GAIN_GREENR)
	
#define V4L2_CID_VX6953_DIGITAL_GAIN_RED \
	V4L2_CID_VX6953(VX6953_CTRL_DIGITAL_GAIN_RED)
	
#define V4L2_CID_VX6953_DIGITAL_GAIN_BLUE \
	V4L2_CID_VX6953(VX6953_CTRL_DIGITAL_GAIN_BLUE)
	
#define V4L2_CID_VX6953_DIGITAL_GAIN_GREENB \
	V4L2_CID_VX6953(VX6953_CTRL_DIGITAL_GAIN_GREENB)

#define V4L2_CID_VX6953_FRAME_LENGTH_LINES \
	V4L2_CID_VX6953(VX6953_CTRL_FRAME_LENGTH_LINES)
	
#define V4L2_CID_VX6953_LINE_LENGTH_PCK \
	V4L2_CID_VX6953(VX6953_CTRL_LINE_LENGTH_PCK)

#define V4L2_CID_VX6953_X_OUTPUT_SIZE \
	V4L2_CID_VX6953(VX6953_CTRL_X_OUTPUT_SIZE)
	
#define V4L2_CID_VX6953_Y_OUTPUT_SIZE \
	V4L2_CID_VX6953(VX6953_CTRL_Y_OUTPUT_SIZE)

#define V4L2_CID_VX6953_X_EVEN_INC \
	V4L2_CID_VX6953(VX6953_CTRL_X_EVEN_INC)

#define V4L2_CID_VX6953_X_ODD_INC \
	V4L2_CID_VX6953(VX6953_CTRL_X_ODD_INC)
	
#define V4L2_CID_VX6953_Y_EVEN_INC \
	V4L2_CID_VX6953(VX6953_CTRL_Y_EVEN_INC)

#define V4L2_CID_VX6953_Y_ODD_INC \
	V4L2_CID_VX6953(VX6953_CTRL_Y_ODD_INC)

#define V4L2_CID_VX6953_MIN_FRAME_LENGTH_LINES \
	V4L2_CID_VX6953(VX6953_CTRL_MIN_FRAME_LENGTH_LINES)

#define V4L2_CID_VX6953_BINNING_MODE \
	V4L2_CID_VX6953(VX6953_CTRL_BINNING_MODE)

#define V4L2_CID_VX6953_HOST_WB_STATS_GREEN_RED \
	V4L2_CID_VX6953(VX6953_CTRL_HOST_WB_STATS_GREEN_RED)

#define V4L2_CID_VX6953_HOST_WB_STATS_RED \
	V4L2_CID_VX6953(VX6953_CTRL_HOST_WB_STATS_RED)

#define V4L2_CID_VX6953_HOST_WB_STATS_BLUE \
	V4L2_CID_VX6953(VX6953_CTRL_HOST_WB_STATS_BLUE)

#define V4L2_CID_VX6953_HOST_WB_STATS_GREEN_BLUE \
	V4L2_CID_VX6953(VX6953_CTRL_HOST_WB_STATS_GREEN_BLUE)

#define V4L2_CID_VX6953_SET_TIME \
	V4L2_CID_VX6953(VX6953_CTRL_SET_TIME)

static inline __u32
vx6953_sub_sampling_factor(__u32 ei, __u32 oi)
{
	return ((ei + oi) / 2);
}

#ifdef __KERNEL__
#include <linux/i2c.h>
#include <linux/videodev2.h>

#define VX6953_I2C_DEVICE		"vx6953"
#define VX6953_I2C_DRIVER		"vx6953"
#define VX6953_I2C_ADDR			0x10

#define VX6953_CCP2_DATA_FORMAT_RAW10	0x0A0A
#define VX6953_CCP2_DATA_FORMAT_RAW10_8_DPCM	0x0A08

#define VX6953_RES_WIDTH 2608
#define VX6953_RES_HEIGHT 1960

struct vx6953_platform_data {
	u32			fourcc;
	u16			coarse_integration_time;
	u16			analogue_gain_code_global;
	u16			vt_pix_clk_div;
	u16			vt_sys_clk_div;
	u16			pre_pll_clk_div;
	u16			pll_multiplier;
	u16			op_sys_clk_div;
	u16			line_length_pck;
	u8			csi_signaling_mode;
	int			(*enable_vdig)(int enable);
	struct v4l2_fract	extclk;
};

extern int
vx6953_i2c_read_raw(struct i2c_client* i2c, u16 index, u8* buf, u16 len);

extern int
vx6953_i2c_write_raw(struct i2c_client* i2c, u8* buf, u16 len);

static inline int
vx6953_i2c_read_u8(struct i2c_client* i2c, u16 index, u8 *value)
{
	return (vx6953_i2c_read_raw(i2c, index, value, 1));
}

static inline int
vx6953_i2c_write_u8(struct i2c_client* i2c, u16 index, u8 value) 
{
	u8 buf[3];
	
	*((u16 *)&buf[0]) = cpu_to_be16(index);
	*((u8 *)&buf[2]) = value;
	
	return (vx6953_i2c_write_raw(i2c, buf, 3));
}

static inline int
vx6953_i2c_read_u16(struct i2c_client* i2c, u16 index, u16 *value)
{
	int rc;
	
	rc = vx6953_i2c_read_raw(i2c, index, (u8 *)value, 2);
	*value = be16_to_cpup(value);
	
	return (rc);
}

static inline int
vx6953_i2c_write_u16(struct i2c_client* i2c, u16 index, u16 value) 
{ 
	u8 buf[4];
	
	*((u16 *)&buf[0]) = cpu_to_be16(index);
	*((u16 *)&buf[2]) = cpu_to_be16(value);
	
	return (vx6953_i2c_write_raw(i2c, buf, 4));
}

static inline int
vx6953_i2c_read_u32(struct i2c_client* i2c, u16 index, u32* value)
{
	int rc;
	
	rc = vx6953_i2c_read_raw(i2c, index, (u8 *)value, 4);
	*value = be32_to_cpup(value);
	
	return (rc);
}

static inline int
vx6953_i2c_write_u32(struct i2c_client* i2c, u16 index, u32 value) 
{ 
	u8 buf[6];
	
	*((u16 *)&buf[0]) = cpu_to_be16(index);
	*((u32 *)&buf[2]) = cpu_to_be32(value);
	
	return (vx6953_i2c_write_raw(i2c, buf, 6));
}

#ifdef CONFIG_VIDEO_VX6953_DBG
extern int vx6953_dbg_register_i2c_client(struct i2c_client *);
extern void vx6953_dbg_unregister_i2c_client(struct i2c_client *);
#else // !CONFIG_VIDEO_VX6953_DBG
static inline int
vx6953_dbg_register_i2c_client(struct i2c_client *i2c)
{
	return (1);
}

static inline void
vx6953_dbg_unregister_i2c_client(struct i2c_client *i2c)
{
}
#endif // CONFIG_VIDEO_VX6953_DBG
#endif // __KERNEL__
#endif // _MEDIA_VX6953_H
