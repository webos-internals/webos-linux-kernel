/*
 * drivers/media/video/vx6852.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
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
 */

#include <asm/div64.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#ifdef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
#include <linux/hrtimer.h>
#endif /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */
#include <media/smia10.h>
#include <media/v4l2-int-device.h>
#include <media/vx6852.h>

#define USE_EDOF
#define USE_ANALOG_BINNING
#define WAIT_TIMEOUT				1000

#define MODEL_ID				0x0000
#define MODEL_ID_VX6852				852

#define REVISION_NUMBER				0x0002
#define SILICON_ID(rev)				(0xF & ((rev) >> 4))
#define NVM_VERSION(rev)			(0xF & (rev))

#define FRAME_COUNT				0x0005
#define MODE_SELECT				0x0100
#define MODE_SELECT_SOFTWARE_STANDBY		0x00
#define MODE_SELECT_STREAMING			0x01

#define FRAME_FORMAT_DESCRIPTOR_1_VX6852	8	/* Dummy pixels */
#define FRAME_FORMAT_DESCRIPTOR_2_VX6852	2	/* SOF lines */

#define IMAGE_ORIENTATION			0x0101
#define SOFTWARE_RESET				0x0103
#define SOFTWARE_RESET_SOFT_RESET		0x01

#define GROUPED_PARAMETER_HOLD			0x0104
#define CCP2_DATA_FORMAT			0x0112

#define MAX_FRAME_LENGTH_LINES			65535
#define MIN_FRAME_BLANKING_LINES		22

#define FINE_INTEGRATION_TIME			0x0200
#define MIN_FINE_INTEGRATION_TIME		543
#define MAX_FINE_INTEGRATION_TIME		598

#define COARSE_INTEGRATION_TIME			0x0202
#define MAX_COARSE_INTEGRATION_TIME(fll)	((fll) - 9)

#define ANALOGUE_GAIN_CODE_GLOBAL		0x0204
#define DIGITAL_GAIN_GREENR			0x020E
#define DIGITAL_GAIN_RED			0x0210
#define DIGITAL_GAIN_BLUE			0x0212
#define DIGITAL_GAIN_GREENB			0x0214
#define MIN_DIGITAL_GAIN			0x0100
#define MAX_DIGITAL_GAIN			0x01F8

#define VT_PIX_CLK_DIV				0x0300
#define VT_SYS_CLK_DIV				0x0302
#define PRE_PLL_CLK_DIV				0x0304
#define PLL_MULTIPLIER				0x0306
#define FRAME_LENGTH_LINES			0x0340
#define MIN_FRAME_LENGTH_LINES(yos)		((yos) + 24)

#define LINE_LENGTH_PCK				0x0342
#define X_ADDR_START				0x0344
#define Y_ADDR_START				0x0346
#define X_ADDR_END				0x0348
#define Y_ADDR_END				0x034A
#define X_OUTPUT_SIZE				0x034C
#define Y_OUTPUT_SIZE				0x034E
#define MAX_Y_OUTPUT_SIZE			1544
#define X_EVEN_INC				0x0380
#define X_ODD_INC				0x0382
#define Y_EVEN_INC				0x0384
#define Y_ODD_INC				0x0386
#define MAX_PLL_IP_FREQ_MHZ			0x1110
#define VTIMING_MAJOR				0x3015
#define IQ_COLUMN_CURRENT			0x3117
#define SENSOR_STATUS				0x3130
#define SENSOR_STATUS_STANDBY			1

#define BAYER_AVERAGE_ENABLE			0x3300

#define ILP_REVISION_ID				0x3800
#define ILP_REVISION_MAJOR(rev)			(0xF & ((rev) >> 4))
#define ILP_REVISION_MINOR(rev)			(0xF & (rev))

#define FIRMWARE_VER_MAJOR			0x3801
#define FIRMWARE_VER_MINOR			0x3802
#define PATCH_VER_MAJOR				0x3803
#define PATCH_VER_MINOR				0x3804
#define ILP_STATUS				0x380B
#define ILP_STATUS_STANDBY			1

#define DAF_SHARPNESS_LEVEL_MACRO		0x3960
#define DAF_SHARPNESS_LEVEL_PORTRAIT		0x3961
#define DAF_SHARPNESS_LEVEL_LANDSCAPE		0x3962
#define DAF_DENOISING_VALUE_OUTDOOR		0x3963
#define DAF_DENOISING_VALUE_INDOOR		0x3964
#define DAF_DENOISING_LEVEL_LOW_LIGHT		0x3965
#define DAF_NOISE_DETAIL_OUTDOOR		0x3966
#define DAF_NOISE_DETAIL_INDOOR			0x3967
#define DAF_NOISE_DETAIL_LOW_LIGHT		0x3968
#define DAF_BLUE_FRINGING_INTENSITY		0x3969
#define DAF_MODE				0x3978
#define DAF_MODE_CAPTURE			0
#define DAF_MODE_PREVIEW			1

#define DAF_FOCUS_STRATEGY			0x3979
#define DAF_FOCUS_CUSTOM_CENTER_WEIGHT		0x397B
#define DAF_FOCUS_CUSTOM_MIDDLE_WEIGHT		0x397C
#define DAF_FOCUS_CUSTOM_EXTERIOR_WEIGHT	0x397D
#define DAF_HOST_WB_STATS_GREEN_RED		0x399E
#define DAF_HOST_WB_STATS_RED			0x39A0
#define DAF_HOST_WB_STATS_BLUE			0x39A2
#define DAF_HOST_WB_STATS_GREEN_BLUE		0x39A4
#define MAX_DAF_HOST_WB_STATS			0x1000

#define DAF_HOST_WB_STATS_CONTROL_COIN		0x39A6
#define DAF_HOST_WB_STATS_ENABLE		0x39A7

#define DAF_ESTIMATED_FOCUS			0x39A8
#define __VT_SYS_CLK_DIV			0x3614
#define DAF_DATA_CUSTOM_CTRL_PEAK_DATA		0x39EE

#define CTRL_FLAG_8_BIT				0x0001
#define CTRL_FLAG_16_BIT			0x0002
#define CTRL_FLAG_32_BIT			0x0004
#define CTRL_FLAG_BIT_MASK			0x0007
/* NOTE: if needed, can OR with type */
#define CTRL_FLAG_8C				CTRL_FLAG_8_BIT
#define CTRL_FLAG_8UI				CTRL_FLAG_8_BIT
#define CTRL_FLAG_16UI				CTRL_FLAG_16_BIT
#define CTRL_FLAG_16UR				CTRL_FLAG_16_BIT
/* NOTE: rename? */
#define CTRL_FLAG_RO				0x0008
#define CTRL_FLAG_NO_CACHE			0x0010
#define CTRL_FLAG_CACHED			0x0020
#define CTRL_FLAG_MIN				0x0040
#define CTRL_FLAG_MAX				0x0080

#define CTRL_ID2INDEX(id)			((id) - V4L2_CID_VX6852_BASE)

#define INVARIANT(cond)						\
	do {							\
		if (!(cond)) {					\
			printk(KERN_DEBUG "!(" # cond ")\n");	\
			BUG();					\
		}						\
	} while (0)

#define SPEW(level, args...)					\
	do {							\
		if (vx6852_spew_level >= level)			\
			printk(KERN_DEBUG "VX6852:\t" args);	\
	} while (0)

union vx6852_value {
	u8	ui8;
	u16	ui16;
	u32	ui32;
};

struct vx6852_control {
	u16	reg;
	u16	flags;
	u32	value;
	u32	min;
	u32	max;
};

struct vx6852_config {
	int	hqm;	/* high quality mode? */
	u16	xas;	/* x address start */
	u16	xae;	/* x address end */
	u16	xos;	/* x output size */
	u16	yas;	/* y address start */
	u16	yae;	/* y address end */
	u16	yos;	/* y output size */
	u16	xoi;	/* x odd increment */
	u16	yoi;	/* y odd increment */
	u8	vtm;	/* vtiming major */
	u8	iqcc;	/* iq column current */
	u8	bae;	/* bayer average enable */

	u8	dafm;	/* digital auto focus mode */
	u16	fll;	/* frame length (lines) */
	u16	cit;	/* coarse integration time */
};

#ifdef CONFIG_VIDEO_VX6852_SETTING
struct vx6852_setting {
	char			name[FIRMWARE_NAME_MAX];
	const struct firmware	*fw;
};
#endif // CONFIG_VIDEO_VX6852_SETTING

struct vx6852_cci_data_format {
	const char *name;
	const char *format;
};

struct vx6852_client_data {
	/* TODO: remove me */
	struct mutex		lock;
	struct i2c_client	*i2c;
	u32			fourcc;
	u32			bpp;
	u8			ilp;	/* ilp revision id */
	u16			ccp2df;	/* ccp2 data format */
	u16			cit;	/* coarse integration time */
	u16			agcg;	/* analog gain code (global) */
	struct v4l2_fract	vtpc;	/* video timing pixel clock */
	u16			vtpcd;	/* video timing pixel clock divider */
	u16			vtscd;	/* video timing system clock divider */
	u16			ppllcd;	/* pre-pll clock divider */
	u16			pllm;	/* pll multiplier */
	u16			llpc;	/* line length (pixel clocks) */
	u16			cci_idx;
	const char		*cci_data_fmt;
	int			vdig;
	int			power;
	int			(*enable_vdig)(int);
#ifdef CONFIG_VIDEO_VX6852_SETTING
	struct vx6852_setting	once;
#endif // CONFIG_VIDEO_VX6852_SETTING
	struct vx6852_config	cfg;
	struct vx6852_control	ctrls[VX6852_CTRL_COUNT];
	struct v4l2_int_slave	slave;
	struct v4l2_int_device	dev;
};

/* TODO: move these to a flat structure for batching */
static struct vx6852_config vx6852_configs[] = {
	{ /* XQGA (preview) */
	.hqm = 0,
	.xas = 0,
	.xae = 2055,
	.xos = 2056,
	.yas = 0,
	.yae = 1543,
	.yos = 1544,
	.xoi = 1,
	.yoi = 1,
	.vtm = 0,
	.iqcc = 1,
	.bae = 0,
	.dafm = 1,
	},

#ifdef USE_EDOF
	{ /* XQGA (capture) */
	.hqm = 1,
	.xas = 0,
	.xae = 2055,
	.xos = 2056,
	.yas = 0,
	.yae = 1543,
	.yos = 1544,
	.xoi = 1,
	.yoi = 1,
	.vtm = 0,
	.iqcc = 1,
	.bae = 0,
	.dafm = 0,
	},
#endif // USE_EDOF

	{ /* XGA */
	.hqm = 0,
	.xas = 0,
	.xae = 2055,
	.xos = 1016,
	.yas = 0,
	.yae = 1543,
	.yos = 768,
	.xoi = 3,
	.yoi = 3,
#ifdef USE_ANALOG_BINNING
	.vtm = 1,
	.iqcc = 5,
	.bae = 1,
#else /* !USE_ANALOG_BINNING */
	.vtm = 0,
	.iqcc = 1,
	.bae = 0,
#endif /* USE_ANALOG_BINNING */
	.dafm = 1,
	},
};

static struct vx6852_control vx6852_controls[VX6852_CTRL_COUNT] = {
	[VX6852_CTRL_PIXEL_CLOCK] = {
		.reg = -1,
		.flags = CTRL_FLAG_RO,
	},
	[VX6852_CTRL_MODE_SELECT] = {
		.reg = MODE_SELECT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_IMAGE_ORIENTATION] = {
		.reg = IMAGE_ORIENTATION,
		.flags = CTRL_FLAG_8_BIT,
	},
	[VX6852_CTRL_GROUPED_PARAMETER_HOLD] = {
		.reg = GROUPED_PARAMETER_HOLD,
		.flags = CTRL_FLAG_8_BIT,
	},
	[VX6852_CTRL_CCP2_DATA_FORMAT] = {
		.reg = CCP2_DATA_FORMAT,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_FINE_INTEGRATION_TIME] = {
		.reg = FINE_INTEGRATION_TIME,
		.flags = CTRL_FLAG_16UI | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_FINE_INTEGRATION_TIME,
		.max = MAX_FINE_INTEGRATION_TIME,
	},
	[VX6852_CTRL_COARSE_INTEGRATION_TIME] = {
		.reg = COARSE_INTEGRATION_TIME,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_ANALOGUE_GAIN_CODE_GLOBAL] = {
		.reg = ANALOGUE_GAIN_CODE_GLOBAL,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_DIGITAL_GAIN_GREENR] = {
		.reg = DIGITAL_GAIN_GREENR,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6852_CTRL_DIGITAL_GAIN_RED] = {
		.reg = DIGITAL_GAIN_RED,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6852_CTRL_DIGITAL_GAIN_BLUE] = {
		.reg = DIGITAL_GAIN_BLUE,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6852_CTRL_DIGITAL_GAIN_GREENB] = {
		.reg = DIGITAL_GAIN_GREENB,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6852_CTRL_VT_PIX_CLK_DIV] = {
		.reg = VT_PIX_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_VT_SYS_CLK_DIV] = {
		.reg = VT_SYS_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_PRE_PLL_CLK_DIV] = {
		.reg = PRE_PLL_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_PLL_MULTIPLIER] = {
		.reg = PLL_MULTIPLIER,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_FRAME_LENGTH_LINES] = {
		.reg = FRAME_LENGTH_LINES,
		.flags = CTRL_FLAG_16UI | CTRL_FLAG_MAX,
		.max = MAX_FRAME_LENGTH_LINES,
	},
	[VX6852_CTRL_LINE_LENGTH_PCK] = {
		.reg = LINE_LENGTH_PCK,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_X_ADDR_START] = {
		.reg = X_ADDR_START,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_Y_ADDR_START] = {
		.reg = Y_ADDR_START,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_X_ADDR_END] = {
		.reg = X_ADDR_END,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_Y_ADDR_END] = {
		.reg = Y_ADDR_END,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_X_OUTPUT_SIZE] = {
		.reg = X_OUTPUT_SIZE,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_Y_OUTPUT_SIZE] = {
		.reg = Y_OUTPUT_SIZE,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_X_EVEN_INC] = {
		.reg = X_EVEN_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_X_ODD_INC] = {
		.reg = X_ODD_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_Y_EVEN_INC] = {
		.reg = Y_EVEN_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_Y_ODD_INC] = {
		.reg = Y_ODD_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_BAYER_AVERAGE_ENABLE] = {
		.reg = BAYER_AVERAGE_ENABLE,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_0x3614] = {
		.reg = __VT_SYS_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6852_CTRL_DAF_SHARPNESS_LEVEL_MACRO] = {
		.reg = DAF_SHARPNESS_LEVEL_MACRO,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_SHARPNESS_LEVEL_PORTRAIT] = {
		.reg = DAF_SHARPNESS_LEVEL_PORTRAIT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_SHARPNESS_LEVEL_LANDSCAPE] = {
		.reg = DAF_SHARPNESS_LEVEL_LANDSCAPE,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_DENOISING_VALUE_OUTDOOR] = {
		.reg = DAF_DENOISING_VALUE_OUTDOOR,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_DENOISING_VALUE_INDOOR] = {
		.reg = DAF_DENOISING_VALUE_INDOOR,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_DENOISING_LEVEL_LOW_LIGHT] = {
		.reg = DAF_DENOISING_LEVEL_LOW_LIGHT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_NOISE_DETAIL_OUTDOOR] = {
		.reg = DAF_NOISE_DETAIL_OUTDOOR,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_NOISE_DETAIL_INDOOR] = {
		.reg = DAF_NOISE_DETAIL_INDOOR,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_NOISE_DETAIL_LOW_LIGHT] = {
		.reg = DAF_NOISE_DETAIL_LOW_LIGHT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_BLUE_FRINGING_INTENSITY] = {
		.reg = DAF_BLUE_FRINGING_INTENSITY,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_MODE] = {
		.reg = DAF_MODE,
		.flags = CTRL_FLAG_8C,
	},
	[VX6852_CTRL_DAF_FOCUS_STRATEGY] = {
		.reg = DAF_FOCUS_STRATEGY,
		.flags = CTRL_FLAG_8C,
	},
	[VX6852_CTRL_DAF_FOCUS_CUSTOM_CENTER_WEIGHT] = {
		.reg = DAF_FOCUS_CUSTOM_CENTER_WEIGHT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_FOCUS_CUSTOM_MIDDLE_WEIGHT] = {
		.reg = DAF_FOCUS_CUSTOM_MIDDLE_WEIGHT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_FOCUS_CUSTOM_EXTERIOR_WEIGHT] = {
		.reg = DAF_FOCUS_CUSTOM_EXTERIOR_WEIGHT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6852_CTRL_DAF_HOST_WB_STATS_GREEN_RED] = {
		.reg = DAF_HOST_WB_STATS_GREEN_RED,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MAX,
		.max = MAX_DAF_HOST_WB_STATS,
	},
	[VX6852_CTRL_DAF_HOST_WB_STATS_RED] = {
		.reg = DAF_HOST_WB_STATS_RED,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MAX,
		.max = MAX_DAF_HOST_WB_STATS,
	},
	[VX6852_CTRL_DAF_HOST_WB_STATS_BLUE] = {
		.reg = DAF_HOST_WB_STATS_BLUE,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MAX,
		.max = MAX_DAF_HOST_WB_STATS,
	},
	[VX6852_CTRL_DAF_HOST_WB_STATS_GREEN_BLUE] = {
		.reg = DAF_HOST_WB_STATS_GREEN_BLUE,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MAX,
		.max = MAX_DAF_HOST_WB_STATS,
	},
	[VX6852_CTRL_DAF_HOST_WB_STATS_CONTROL_COIN] = {
		.reg = DAF_HOST_WB_STATS_CONTROL_COIN,
		.flags = CTRL_FLAG_8UI | CTRL_FLAG_MAX,
		.max = 1,
	},
	[VX6852_CTRL_DAF_HOST_WB_STATS_ENABLE] = {
		.reg = DAF_HOST_WB_STATS_ENABLE,
		.flags = CTRL_FLAG_8UI | CTRL_FLAG_MAX,
		.max = 1,
	},
	[VX6852_CTRL_DAF_ESTIMATED_FOCUS] = {
		.reg = DAF_ESTIMATED_FOCUS,
		.flags = CTRL_FLAG_8C | CTRL_FLAG_RO | CTRL_FLAG_NO_CACHE,
	},
};

static struct v4l2_ifparm vx6852_ifparm = {
	.if_type = V4L2_IF_TYPE_SMIA10,
	.u.smia10 = {
		.nr_cols = 2,
		.nr_rows = 2,
		.descs[0] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_VISIBLE_PIXEL_DATA,
			.nr = 2056,
		},
		.descs[1] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_DUMMY_PIXEL_DATA,
			.nr = 8,
		},
		.descs[2] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_EMBEDDED_DATA,
			.nr = 2,
		},
		.descs[3] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_VISIBLE_PIXEL_DATA,
			.nr = 1544,
		},
	},
};

static struct vx6852_cci_data_format vx6852_cci_data_formats[] = {
	{"octal", "%03o\n"},
	{"decimal", "%u\n"},
	{"hex", "%02x\n"},
};

static int vx6852_spew_level = 0;

#ifdef CONFIG_VIDEO_VX6852_DBG
static ssize_t
vx6852_show_spew_level(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t i;

	i = snprintf(buf, PAGE_SIZE, "%d\n", vx6852_spew_level);

	return (i + 1);
}

static ssize_t
vx6852_store_spew_level(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	char *endp;

	vx6852_spew_level = simple_strtoul(buf, &endp, 10);

	return (count);
}

static struct device_attribute vx6852_attr_spew_level =
	__ATTR(spew_level, S_IRUGO|S_IWUGO, vx6852_show_spew_level,
		vx6852_store_spew_level);

static inline int
vx6852_create_spew_level(struct device *dev)
{
	return (device_create_file(dev, &vx6852_attr_spew_level));
}

static inline void
vx6852_remove_spew_level(struct device *dev)
{
	device_remove_file(dev, &vx6852_attr_spew_level);
}
#else // !CONFIG_VIDEO_VX6852_DBG
static inline int
vx6852_create_spew_level(struct device *dev)
{
	return (0);
}

static inline void
vx6852_remove_spew_level(struct device *dev)
{
}
#endif // CONFIG_VIDEO_VX6852_DBG

static ssize_t
vx6852_attr_show_cci_data(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	u8 val;
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	if ((rc = vx6852_i2c_read_u8(i2c, data->cci_idx++, &val)))
		goto exit;

	rc = sprintf(buf, data->cci_data_fmt, val);
exit:
	return (rc);
}

static ssize_t
vx6852_attr_store_cci_data(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	unsigned int val;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!sscanf(buf, data->cci_data_fmt, &val)) {
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = vx6852_i2c_write_u8(i2c, data->cci_idx++, val)))
		goto exit;

	rc = count;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute vx6852_attr_cci_data =
	__ATTR(cci_data, S_IRUGO|S_IWUGO, vx6852_attr_show_cci_data,
		vx6852_attr_store_cci_data);

static ssize_t
vx6852_attr_show_cci_data_format(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i;
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);
	struct vx6852_cci_data_format *fmt;

	for (i = 0, rc = 0; i < ARRAY_SIZE(vx6852_cci_data_formats); ++i) {
		fmt = &vx6852_cci_data_formats[i];
		rc += snprintf(buf + rc, PAGE_SIZE - rc, "%s%s", fmt->name,
				!strcmp(fmt->format, data->cci_data_fmt)
					? "* " : " ");
	}

	rc += snprintf(buf + rc, PAGE_SIZE - rc, "\n");

	return (rc);
}

static ssize_t
vx6852_attr_store_cci_data_format(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int i;
	int rc = -EINVAL;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	for (i = 0; i < ARRAY_SIZE(vx6852_cci_data_formats); ++i) {
		if (!strncmp(buf, vx6852_cci_data_formats[i].name, count)) {
			data->cci_data_fmt = vx6852_cci_data_formats[i].format;
			rc = count;
			break;
		}
	}

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute vx6852_attr_cci_data_format =
	__ATTR(cci_data_format, S_IRUGO|S_IWUGO,
		vx6852_attr_show_cci_data_format,
		vx6852_attr_store_cci_data_format);

static ssize_t
vx6852_attr_show_cci_index(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	rc = sprintf(buf, "0x%04X\n", data->cci_idx);

	return (rc);
}

static ssize_t
vx6852_attr_store_cci_index(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	unsigned int idx;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!sscanf(buf, "0x%04X", &idx)) {
		rc = -EINVAL;
		goto exit;
	}

	data->cci_idx = idx;
	rc = count;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute vx6852_attr_cci_index =
	__ATTR(cci_index, S_IRUGO|S_IWUGO, vx6852_attr_show_cci_index,
		vx6852_attr_store_cci_index);

static ssize_t
vx6852_attr_show_vdig(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	rc = sprintf(buf, "%d\n", data->vdig);

	return (rc);
}

static int
vx6852_enable_vdig(struct vx6852_client_data *data, int enable)
{
	int rc = 0;

	if ((enable != data->vdig) && data->enable_vdig)
		rc = data->enable_vdig((data->vdig = enable));

	return (rc);
}

static ssize_t
vx6852_attr_store_vdig(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int rc;
	int vdig;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!sscanf(buf, "%d", &vdig)) {
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = vx6852_enable_vdig(data, !!vdig)))
		goto exit;

	rc = count;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute vx6852_attr_vdig =
	__ATTR(vdig, S_IRUGO|S_IWUGO, vx6852_attr_show_vdig,
		vx6852_attr_store_vdig);

int
vx6852_i2c_read_raw(struct i2c_client* i2c, u16 index, u8* buf, u16 len)
{
	int rc;
	struct i2c_msg msgs[2];

	msgs[0].addr = i2c->addr;
	msgs[0].len = 2;
	msgs[0].flags = 0;
	index = cpu_to_be16(index);
	msgs[0].buf = (u8 *)&index;

	msgs[1].addr = i2c->addr;
	msgs[1].len = len;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = buf;

	rc = i2c_transfer(i2c->adapter, msgs, 2);

	return (min(rc, 0));
}
EXPORT_SYMBOL(vx6852_i2c_read_raw);

int
vx6852_i2c_write_raw(struct i2c_client* i2c, u8* buf, u16 len)
{
	int rc;
	struct i2c_msg msg;

	msg.addr = i2c->addr;
	msg.len = len;
	msg.flags = 0;
	msg.buf = buf;

	rc = i2c_transfer(i2c->adapter, &msg, 1);

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	{
		int i;
		printk("vx6852-i2c,,");
		for (i = 0; i < len; i++) {
			printk("%02x,", buf[i]);
		}
		printk("\n");
	}
#endif
	return (min(rc, 0));
}
EXPORT_SYMBOL(vx6852_i2c_write_raw);

static int
vx6852_i2c_mwait_u8(struct i2c_client* i2c, u16 idx, u8 val, unsigned int ms)
{
	u8 buf;
	int rc;
	unsigned long expiry = jiffies + msecs_to_jiffies(ms);

	do {
		if ((rc = vx6852_i2c_read_raw(i2c, idx, &buf, 1))
			|| (buf == val))
			break;

		if (time_after(jiffies, expiry)) {
			rc = -ETIMEDOUT;
			break;
		}

		msleep(1);
	} while (1);

	return (rc);
}

#ifdef CONFIG_VIDEO_VX6852_SETTING
static int
vx6852_load_setting(struct device *dev, const char *name,
			struct vx6852_setting *setting)
{
	int rc;

	SPEW(1, "+++ %s: name=%s\n", __func__, name);

	if (setting->fw) {
		release_firmware(setting->fw);
		setting->fw = NULL;
	}

	strlcpy(setting->name, name, FIRMWARE_NAME_MAX);
	strstrip(setting->name);
	rc = request_firmware(&setting->fw, setting->name, dev);

	SPEW(1, "--- %s: rc=%d size=%u\n", __func__, rc,
		setting->fw ? setting->fw->size : 0);

	return (rc);
}

#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
static inline s64
timespec_sub_to_ns(struct timespec lhs, struct timespec rhs)
{
	s64 diff;

	diff = timespec_to_ns(&lhs) - timespec_to_ns(&rhs);

	return (diff);
}
#endif //CONFIG_VIDEO_VX6852_SETTING_PROFILE

#define SPEW_I2C_WRITE(lvl, buf, len)					\
	do {								\
		if (vx6852_spew_level >= (lvl))	{			\
			u16 i;						\
			u16 idx = (buf[0] << 8) | buf[1];		\
			for (i = 2; i != (len); i++, idx++)		\
				printk(KERN_DEBUG "VX6852:\t"		\
					"write: 0x%04X = 0x%02X\n",	\
					idx, (buf)[i]);			\
		}							\
	} while(0)

static int
vx6852_apply_setting(struct i2c_client *i2c, struct vx6852_setting *setting)
{
	u8 buf[16];
	int rc;
	u16 idx = 0xFFFF;
	u16 len = 0;
	struct smia10_parse_state state;
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
	s64 __t;
	unsigned int __i = 0;
	unsigned int __w = 0;
	unsigned int __r = 0;
	struct timespec __b;
	struct timespec __e;
	struct timespec __wt = {0, 0};
	struct timespec __rt = {0, 0};
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE

	SPEW(1, "+++ %s\n", __func__);

	if (!setting->fw) {
		rc = -EINVAL;
		goto exit;
	}

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	printk("vx6852-init\n");
#endif
	smia10_parse_init(&state, setting->fw->data, setting->fw->size);

	while (!(rc = smia10_parse_embedded_data(&state)) && state.size) {
		if (state.index != idx) {
			if (len) {
				SPEW_I2C_WRITE(4, buf, len);
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
				getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
				if ((rc = vx6852_i2c_write_raw(i2c, buf, len)))
					goto exit;
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
				getnstimeofday(&__e);
				__w++;
				__t = timespec_sub_to_ns(__e, __b);
				timespec_add_ns(&__wt, __t);
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
			}

			idx = state.index;
			len = 0;
			buf[len++] = idx >> 8;
			buf[len++] = idx;
		}

		buf[len++] = *state.data;
		idx++;
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
		__i++;
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE

		if ((len == sizeof(buf))
			|| (state.index == SOFTWARE_RESET)
			|| (state.index == DAF_DATA_CUSTOM_CTRL_PEAK_DATA)) {
			SPEW_I2C_WRITE(4, buf, len);
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
			getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
			if ((rc = vx6852_i2c_write_raw(i2c, buf, len)))
				goto exit;
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
			getnstimeofday(&__e);
			__w++;
			timespec_add_ns(&__wt, timespec_sub_to_ns(__e, __b));
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE

			idx = 0xFFFF;
			len = 0;
		}

		if (((state.index == SOFTWARE_RESET)
			|| (state.index == DAF_DATA_CUSTOM_CTRL_PEAK_DATA))) {
			SPEW(4, "wait: 0x%04X\n", state.index);
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
			getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
			if ((rc = vx6852_i2c_mwait_u8(i2c, state.index, 0,
							WAIT_TIMEOUT)))
				goto exit;
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
			getnstimeofday(&__e);
			__r++;
			timespec_add_ns(&__rt, timespec_sub_to_ns(__e, __b));
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
		}
	}

	if (len) {
		SPEW_I2C_WRITE(4, buf, len);
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
		getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
		if ((rc = vx6852_i2c_write_raw(i2c, buf, len)))
			goto exit;
#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
		getnstimeofday(&__e);
		__w++;
		timespec_add_ns(&__wt, timespec_sub_to_ns(__e, __b));
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
	}

#ifdef CONFIG_VIDEO_VX6852_SETTING_PROFILE
	SPEW(1, "regs:%u writes:%u in %ld.%09lds waits:%u in %ld.%09lds\n",
		__i,
		__w, __wt.tv_sec, __wt.tv_nsec,
		__r, __rt.tv_sec, __rt.tv_nsec);
#endif // CONFIG_VIDEO_VX6852_SETTING_PROFILE
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static ssize_t
vx6852_attr_show_setting(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_setting *setting = NULL;
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s\n", __func__);

	if (!strcmp(attr->attr.name, "register_reprogram_once"))
		setting = &data->once;

	mutex_lock(&data->lock);

	if (!setting->fw)
		rc = sprintf(buf, "not loaded\n");

	else
		rc = sprintf(buf, "loaded [%s]\n", setting->name);

	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static ssize_t
vx6852_attr_store_setting(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = -EINVAL;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6852_setting *setting = NULL;
	struct vx6852_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!strcmp(attr->attr.name, "register_reprogram_once"))
		setting = &data->once;

	mutex_lock(&data->lock);
	rc = vx6852_load_setting(dev, buf, setting);
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (!rc ? count : rc);
}

static struct device_attribute vx6852_attr_once =
	__ATTR(register_reprogram_once, S_IRUGO|S_IWUGO,
		vx6852_attr_show_setting,
		vx6852_attr_store_setting);

static inline int
vx6852_create_settings(struct device *dev)
{
	return (device_create_file(dev, &vx6852_attr_once));
}

static inline void
vx6852_remove_settings(struct device *dev)
{
	device_remove_file(dev, &vx6852_attr_once);
}
#else // !CONFIG_VIDEO_VX6852_SETTING
static inline int
vx6852_create_settings(struct device *dev)
{
	return (0);
}

static inline void
vx6852_remove_settings(struct device *dev)
{
}
#endif // CONFIG_VIDEO_VX6852_SETTING

static int
vx6852_get_control(struct vx6852_client_data *data, u16 index, u32 *_val)
{
	int rc;
	union vx6852_value val;
	struct vx6852_control *ctrl;

	ctrl = data->ctrls + index;

	SPEW(3, "+++ %s: index=%u\n", __func__, index);

	if (VX6852_CTRL_COUNT <= index) {
		rc = -EINVAL;
		goto exit;
	}
	if (!(CTRL_FLAG_NO_CACHE & ctrl->flags)
		&& (CTRL_FLAG_CACHED & ctrl->flags))
		goto cached;

	switch (CTRL_FLAG_BIT_MASK & ctrl->flags) {
	case CTRL_FLAG_8_BIT:
		rc = vx6852_i2c_read_u8(data->i2c, ctrl->reg, &val.ui8);
		ctrl->value = val.ui8;
		break;
	case CTRL_FLAG_16_BIT:
		rc = vx6852_i2c_read_u16(data->i2c, ctrl->reg, &val.ui16);
		ctrl->value = val.ui16;
		break;
	case CTRL_FLAG_32_BIT:
		rc = vx6852_i2c_read_u32(data->i2c, ctrl->reg, &val.ui32);
		ctrl->value = val.ui32;
		break;
	default:
		BUG();
		rc = -EINVAL;
	}
	if (rc) goto exit;

	ctrl->flags |= CTRL_FLAG_CACHED;
cached:
	*_val = ctrl->value;
	rc = 0;
exit:
	SPEW(3, "--- %s: rc=%d val=%u\n", __func__, rc, *_val);

	return (rc);
}

static int
vx6852_set_control(struct vx6852_client_data *data, u16 index, u32 _val)
{
	int rc;
	union vx6852_value val;
	struct vx6852_control *ctrl;

	ctrl = data->ctrls + index;

	SPEW(3, "+++ %s: index=%u val=%u\n", __func__, index, _val);

	if ((VX6852_CTRL_COUNT <= index)
		|| ((CTRL_FLAG_MIN & ctrl->flags) && (_val < ctrl->min))
		|| ((CTRL_FLAG_MAX & ctrl->flags) && (_val > ctrl->max))) {
		rc = -EINVAL;
		goto exit;
	}

	if (CTRL_FLAG_RO & ctrl->flags) {
		rc = -EACCES;
		goto exit;
	}

	ctrl->flags &= ~CTRL_FLAG_CACHED;

	switch (CTRL_FLAG_BIT_MASK & ctrl->flags) {
	case CTRL_FLAG_8_BIT:
		ctrl->value = val.ui8 = (u8)_val;
		rc = vx6852_i2c_write_u8(data->i2c, ctrl->reg, val.ui8);
		break;
	case CTRL_FLAG_16_BIT:
		ctrl->value = val.ui16 = (u16)_val;
		rc = vx6852_i2c_write_u16(data->i2c, ctrl->reg, val.ui16);
		break;
	case CTRL_FLAG_32_BIT:
		ctrl->value = val.ui32 = _val;
		rc = vx6852_i2c_write_u32(data->i2c, ctrl->reg, val.ui32);
		break;
	default:
		BUG();
		rc = -EINVAL;
	}

	if (rc) goto exit;

	ctrl->flags |= CTRL_FLAG_CACHED;
	rc = 0;
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static void vx6852_config_to_format(struct vx6852_client_data *dat,
					struct vx6852_config *cfg,
					struct v4l2_pix_format *fmt)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->width = cfg->xos;
	fmt->height = cfg->yos;
	fmt->pixelformat = dat->fourcc;
	fmt->bytesperline =
		((cfg->xos + FRAME_FORMAT_DESCRIPTOR_1_VX6852) * dat->bpp) / 8;
	fmt->sizeimage = fmt->bytesperline * cfg->yos;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

static int vx6852_ioctl_g_fmt(struct v4l2_int_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	struct vx6852_client_data *data;

	data = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);
	vx6852_config_to_format(data, &data->cfg, &fmt->fmt.pix);
	mutex_unlock(&data->lock);
	rc = 0;

	SPEW(1, "--- %s: rc=%d width=%u height=%u pixelformat=%c%c%c%c"
		" bytesperline=%u sizeimage=%u colorspace=%d\n", __func__,
		rc, fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24),
		fmt->fmt.pix.bytesperline, fmt->fmt.pix.sizeimage,
		fmt->fmt.pix.colorspace);

	return (rc);
}

static void vx6852_format_to_config(struct v4l2_pix_format *fmt,
					struct vx6852_config *cfg)
{
	int i;
	struct vx6852_config *best = &vx6852_configs[0];

	for (i = 1; i < ARRAY_SIZE(vx6852_configs); ++i) {
		if ((vx6852_configs[i].xos >= fmt->width)
			&& (vx6852_configs[i].yos >= fmt->height))
			best = &vx6852_configs[i];
	}

	cfg->hqm = best->hqm;
	cfg->xas = best->xas;
	cfg->xae = best->xae;
	cfg->xos = best->xos;
	cfg->yas = best->yas;
	cfg->yae = best->yae;
	cfg->yos = best->yos;
	cfg->xoi = best->xoi;
	cfg->yoi = best->yoi;
	cfg->vtm = best->vtm;
	cfg->iqcc = best->iqcc;
	cfg->bae = best->bae;
	cfg->fll = max((u16)MIN_FRAME_LENGTH_LINES(cfg->yos), cfg->fll);
	cfg->cit = min(cfg->cit, (u16)MAX_COARSE_INTEGRATION_TIME(cfg->fll));
	cfg->dafm = !(!cfg->dafm && cfg->hqm);
}

#ifdef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
static inline void vx6852_config_hacks(struct vx6852_config *cfg, u8 ilp)
{
	if (ilp < 0x21) {
		cfg->xoi = 1;
		cfg->vtm = 0;
	}
}
#endif /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */

static int vx6852_ioctl_try_fmt(struct v4l2_int_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	struct vx6852_config cfg;
	struct vx6852_client_data *dat;

	dat = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s: width=%u height=%u pixelformat=%c%c%c%c\n", __func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24));

	vx6852_format_to_config(&fmt->fmt.pix, &cfg);
	vx6852_config_to_format(dat, &cfg, &fmt->fmt.pix);
	rc = 0;

	SPEW(1, "--- %s: rc=%d width=%u height=%u pixelformat=%c%c%c%c"
		" bytesperline=%u sizeimage=%u colorspace=%d\n", __func__,
		rc, fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24),
		fmt->fmt.pix.bytesperline, fmt->fmt.pix.sizeimage,
		fmt->fmt.pix.colorspace);

	return (rc);
}

static int vx6852_ioctl_s_fmt(struct v4l2_int_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	u32 ms;
	struct vx6852_client_data *data;

	data = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s: width=%u height=%u pixelformat=%c%c%c%c\n", __func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24));

	mutex_lock(&data->lock);

	if ((rc = vx6852_get_control(data, VX6852_CTRL_MODE_SELECT, &ms)))
		goto unlock;

	if (MODE_SELECT_STREAMING == ms) {
		rc = -EBUSY;
		goto unlock;
	}

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	printk("vx6852-set-format\n");
#endif
	vx6852_format_to_config(&fmt->fmt.pix, &data->cfg);
#ifdef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
	vx6852_config_hacks(&data->cfg, data->ilp);
#endif /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */
	vx6852_config_to_format(data, &data->cfg, &fmt->fmt.pix);
	rc = 0;
unlock:
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d width=%u height=%u pixelformat=%c%c%c%c"
		" bytesperline=%u sizeimage=%u colorspace=%d\n", __func__,
		rc, fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24),
		fmt->fmt.pix.bytesperline, fmt->fmt.pix.sizeimage,
		fmt->fmt.pix.colorspace);

	return (rc);
}

static void vx6852_config_to_parm(struct vx6852_client_data *dat,
					struct vx6852_config *cfg,
					struct v4l2_captureparm *parm)
{
	u64 num;

	memset(parm, 0, sizeof(*parm));
	parm->capability = V4L2_CAP_TIMEPERFRAME;

	if (!cfg->dafm)
		parm->capturemode = V4L2_MODE_HIGHQUALITY;

	num = (u64)dat->llpc * cfg->fll * dat->vtpc.denominator * 10;
	do_div(num, dat->vtpc.numerator);
	parm->timeperframe.numerator = num;
	parm->timeperframe.denominator = 1;
}

static int vx6852_ioctl_g_parm(struct v4l2_int_device *dev,
				struct v4l2_streamparm *parm)
{
	int rc;
	struct vx6852_client_data *data;

	data = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);
	vx6852_config_to_parm(data, &data->cfg, &parm->parm.capture);
	mutex_unlock(&data->lock);
	rc = 0;

	SPEW(1, "--- %s: rc=%d highquality=%u timeperframe=%u/%u\n", __func__,
		rc, V4L2_MODE_HIGHQUALITY & parm->parm.capture.capturemode,
		parm->parm.capture.timeperframe.numerator,
		parm->parm.capture.timeperframe.denominator);

	return (rc);
}

static void vx6852_parm_to_config(struct vx6852_client_data *dat,
					struct v4l2_captureparm *parm,
					struct vx6852_config *cfg)
{
	u64 fll;
	u64 den;

	cfg->dafm = !((V4L2_MODE_HIGHQUALITY & parm->capturemode) && cfg->hqm);

	fll = (u64)parm->timeperframe.numerator * dat->vtpc.numerator;
	den = (u64)parm->timeperframe.denominator * 10 * dat->llpc
		* dat->vtpc.denominator;
	fll += den - 1;
	do_div(fll, den);
	cfg->fll = min(fll, (u64)MAX_FRAME_LENGTH_LINES);
	cfg->fll = max((u16)MIN_FRAME_LENGTH_LINES(cfg->yos), cfg->fll);
	cfg->cit = min(cfg->cit, (u16)MAX_COARSE_INTEGRATION_TIME(cfg->fll));
}

static int vx6852_ioctl_s_parm(struct v4l2_int_device *dev,
				struct v4l2_streamparm *parm)
{
	int rc;
	u32 val;
	struct vx6852_client_data *data;

	data = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s: highquality=%u timeperframe=%u/%u\n", __func__,
		V4L2_MODE_HIGHQUALITY & parm->parm.capture.capturemode,
		parm->parm.capture.timeperframe.numerator,
		parm->parm.capture.timeperframe.denominator);

	if (parm->parm.capture.timeperframe.numerator
		&& !parm->parm.capture.timeperframe.denominator) {
		rc = -EINVAL;
		goto exit;
	}

	mutex_lock(&data->lock);

	if ((rc = vx6852_get_control(data, VX6852_CTRL_MODE_SELECT, &val)))
		goto unlock;

	if (MODE_SELECT_STREAMING == val) {
		rc = -EBUSY;
		goto unlock;
	}

	vx6852_parm_to_config(data, &parm->parm.capture, &data->cfg);
	vx6852_config_to_parm(data, &data->cfg, &parm->parm.capture);
	rc = 0;
unlock:
	mutex_unlock(&data->lock);
exit:
	SPEW(1, "--- %s: rc=%d highquality=%u timeperframe=%u/%u\n", __func__,
		rc, V4L2_MODE_HIGHQUALITY & parm->parm.capture.capturemode,
		parm->parm.capture.timeperframe.numerator,
		parm->parm.capture.timeperframe.denominator);

	return (rc);
}

static int vx6852_ioctl_g_ctrl(struct v4l2_int_device *dev,
				struct v4l2_control *ctl)
{
	int rc;
	struct vx6852_client_data *dat;

	dat = container_of(dev, struct vx6852_client_data, dev);

	SPEW(3, "+++ %s: id=0x%08X(index=%u)\n", __func__, ctl->id,
		CTRL_ID2INDEX(ctl->id));

	if ((ctl->id < V4L2_CID_VX6852(0))
		|| (ctl->id > V4L2_CID_VX6852(VX6852_CTRL_COUNT - 1))) {
		rc = -EINVAL;
		goto exit;
	}

	mutex_lock(&dat->lock);

	switch (CTRL_ID2INDEX(ctl->id)) {
	case VX6852_CTRL_FRAME_LENGTH_LINES:
		ctl->value = dat->cfg.fll;
		rc = 0;
		break;
	default:
		rc = vx6852_get_control(dat, CTRL_ID2INDEX(ctl->id),
					&ctl->value);
		break;
	}

	mutex_unlock(&dat->lock);
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int vx6852_ioctl_s_ctrl(struct v4l2_int_device *dev,
				struct v4l2_control *ctl)
{
	int rc;
	struct vx6852_client_data *dat;

	dat = container_of(dev, struct vx6852_client_data, dev);

	SPEW(3, "+++ %s: id=0x%08X(index=%u) value=%u\n", __func__, ctl->id,
		CTRL_ID2INDEX(ctl->id), ctl->value);

	if ((ctl->id < V4L2_CID_VX6852(0))
		|| (ctl->id > V4L2_CID_VX6852(VX6852_CTRL_COUNT - 1))) {
		rc = -EINVAL;
		goto exit;
	}

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	if (V4L2_CID_VX6852_SET_TIME == ctl->id) {
		printk("vx6852-frame-cfg-end,%d\n", ctl->value);
		rc = 0;
		goto exit;
	}
#endif
	switch (CTRL_ID2INDEX(ctl->id)) {
	case VX6852_CTRL_COARSE_INTEGRATION_TIME:
		if (ctl->value > MAX_COARSE_INTEGRATION_TIME(dat->cfg.fll)) {
			rc = -EINVAL;
			goto exit;
		}
		break;
	case VX6852_CTRL_FRAME_LENGTH_LINES:
		if (ctl->value < MIN_FRAME_LENGTH_LINES(dat->cfg.yos)) {
			rc = -EINVAL;
			goto exit;
		}
		break;
	}

	mutex_lock(&dat->lock);

	if ((rc = vx6852_set_control(dat, CTRL_ID2INDEX(ctl->id), ctl->value)))
		goto unlock;

	switch (CTRL_ID2INDEX(ctl->id)) {
	case VX6852_CTRL_COARSE_INTEGRATION_TIME:
		dat->cfg.cit = ctl->value;
		break;
	case VX6852_CTRL_FRAME_LENGTH_LINES:
		dat->cfg.fll = ctl->value;
		break;
	}
unlock:
	mutex_unlock(&dat->lock);
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

#ifdef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
static int vx6852_detect_pll_lock_bug(struct vx6852_client_data *dat)
{
	u8 actual = 0;
	int rc;
	u32 frame;
	s64 elapsed = 0;
	u32 expected = 0;
	u32 exposure;
	struct timespec beg;
	struct timespec end;

	SPEW(1, "+++ %s\n", __func__);

	exposure = dat->vtpc.denominator * dat->cfg.cit * dat->llpc;
	exposure += (dat->vtpc.numerator * 1000) - 1;
	exposure /= dat->vtpc.numerator * 1000;

	frame = dat->vtpc.denominator * dat->cfg.fll * dat->llpc;
	frame += (dat->vtpc.numerator * 1000) - 1;
	frame /= dat->vtpc.numerator * 1000;

	if ((rc = vx6852_set_control(dat, VX6852_CTRL_MODE_SELECT,
					MODE_SELECT_STREAMING)))
		goto exit;

	/* fixed in 2.1 */
	if (dat->ilp >= 0x21)
		goto exit;

	ktime_get_ts(&beg);
	msleep(exposure + frame);
	ktime_get_ts(&end);
	elapsed = timespec_to_ns(&end) - timespec_to_ns(&beg);
	do_div(elapsed, NSEC_PER_MSEC);
	expected = elapsed - exposure;
	expected /= frame;

	if ((rc = vx6852_i2c_read_u8(dat->i2c, FRAME_COUNT, &actual)))
		goto exit;

	if (actual > expected) {
		vx6852_set_control(dat, VX6852_CTRL_MODE_SELECT,
					MODE_SELECT_SOFTWARE_STANDBY);
		rc = -EIO;
		printk(KERN_ERR VX6852_I2C_DEVICE ": PLL lock bug detected\n");
	}
exit:
	SPEW(1, "--- %s: rc=%d exposure=%ums frame=%ums expected=%u actual=%u"
		" nominal=%ums elapsed=%lldms\n", __func__, rc, exposure,
		frame, expected, actual, exposure + frame, elapsed);

	return (rc);
}
#endif /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */

static int vx6852_wait_standby(struct vx6852_client_data *data)
{
	int rc;
	unsigned int dt;

	dt = MAX_Y_OUTPUT_SIZE * data->llpc * data->vtpc.denominator;
	dt += (data->vtpc.numerator * 1000) - 1;
	dt /= data->vtpc.numerator * 1000;

#ifdef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
	if (data->ilp < 0x20) {
		rc = 0;
		goto exit;
	}
#endif /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */

	if ((rc = vx6852_i2c_mwait_u8(data->i2c, ILP_STATUS,
					ILP_STATUS_STANDBY, dt)))
		goto exit;

	if ((rc = vx6852_i2c_mwait_u8(data->i2c, SENSOR_STATUS,
					SENSOR_STATUS_STANDBY, dt)))
		goto exit;

exit:
	return (rc);
}

static int vx6852_ioctl_streamon(struct v4l2_int_device *dev,
					enum v4l2_buf_type type)
{
	u8 buf[18];
	int rc;
	int len;
	u32 ms;
	struct vx6852_client_data *data;

	data = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);

	if ((rc = vx6852_get_control(data, VX6852_CTRL_MODE_SELECT, &ms)))
		goto unlock;

	if (MODE_SELECT_STREAMING == ms) {
		rc = -EBUSY;
		goto unlock;
	}

	if ((rc = vx6852_wait_standby(data)))
		goto unlock;

	len = 0;
	*((u16 *)&buf[len]) = cpu_to_be16(FRAME_LENGTH_LINES);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.fll);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->llpc);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.xas);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.yas);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.xae);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.yae);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.xos);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.yos);
	len += 2;

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	printk("vx6852-streamon\n");
#endif
	if ((rc = vx6852_i2c_write_raw(data->i2c, buf, len)))
		goto unlock;

	len = 0;
	*((u16 *)&buf[len]) = cpu_to_be16(X_ODD_INC);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.xoi);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(1);	/* y_even_inc */
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.yoi);
	len += 2;

	if ((rc = vx6852_i2c_write_raw(data->i2c, buf, len)))
		goto unlock;

	if ((rc = vx6852_i2c_write_u16(data->i2c, COARSE_INTEGRATION_TIME,
					data->cfg.cit)))
		goto unlock;

	if ((rc = vx6852_i2c_write_u8(data->i2c, VTIMING_MAJOR,
					data->cfg.vtm)))
		goto unlock;

	if ((rc = vx6852_i2c_write_u8(data->i2c, IQ_COLUMN_CURRENT,
					data->cfg.iqcc)))
		goto unlock;

	if ((rc = vx6852_i2c_write_u8(data->i2c, BAYER_AVERAGE_ENABLE,
					data->cfg.bae)))
		goto unlock;

#ifdef USE_EDOF
	if ((rc = vx6852_i2c_write_u8(data->i2c, DAF_MODE, data->cfg.dafm)))
		goto unlock;
#endif // USE_EDOF

	SPEW(2, "cit=%u fll=%u xos=%u yos=%u xoi=%u yoi=%u vtm=%u iqcc=%u"
		" bae=%u dafm=%u\n", data->cfg.cit, data->cfg.fll,
		data->cfg.xos, data->cfg.yos, data->cfg.xoi, data->cfg.yoi,
		data->cfg.vtm, data->cfg.iqcc, data->cfg.bae, data->cfg.dafm);

#ifndef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
	rc = vx6852_set_control(data, VX6852_CTRL_MODE_SELECT,
				MODE_SELECT_STREAMING);
#else /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */
	rc = vx6852_detect_pll_lock_bug(data);
#endif /* !CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */
unlock:
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
vx6852_ioctl_streamoff(struct v4l2_int_device *dev, enum v4l2_buf_type type)
{
	int rc;
	struct vx6852_client_data *data;

	data = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	printk("vx6852-streamoff\n");
#endif
	rc = vx6852_set_control(data, VX6852_CTRL_MODE_SELECT,
				MODE_SELECT_SOFTWARE_STANDBY);
#ifdef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
	/* 1.x hangs when resolution changes shortly after software standby */
	if (data->ilp < 0x20)
		msleep(50);
#endif /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
vx6852_set_power_on(struct vx6852_client_data *data, int init)
{
	int rc;
#ifndef CONFIG_VIDEO_VX6852_SETTING
	u16 idx;
	union vx6852_value val;
#endif // !CONFIG_VIDEO_VX6852_SETTING

	SPEW(1, "+++ %s\n", __func__);

	if (data->enable_vdig
		&& (rc = data->enable_vdig(1)))
		goto exit;

	/* TODO: 2800 EXTCLK cycles */
	/* XSHUTDOWN --> 1st I2C transaction: 3.1 ms (852) */
	msleep(4);

	if (!init)
		goto powered_on;

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	printk("vx6852-poweron\n");
#endif
#ifdef CONFIG_VIDEO_VX6852_SETTING
	if ((rc = vx6852_apply_setting(data->i2c, &data->once)))
		goto disable_vdig;
#else // !CONFIG_VIDEO_VX6852_SETTING
	/* NOTE: software reset in case VDIG isn't cut */
	idx = SOFTWARE_RESET;
	val.ui8 = SOFTWARE_RESET_SOFT_RESET;
	rc = vx6852_i2c_write_u8(data->i2c, idx, val.ui8);
	if (rc) goto disable_vdig;

	idx = VX6852_CTRL_CCP2_DATA_FORMAT;
	rc = vx6852_set_control(data, idx, data->ccp2df);
	if (rc) goto disable_vdig;

	idx = VX6852_CTRL_COARSE_INTEGRATION_TIME;
	rc = vx6852_set_control(data, idx, data->cit);
	if (rc) goto disable_vdig;

	idx = VX6852_CTRL_VT_PIX_CLK_DIV;
	rc = vx6852_set_control(data, idx, data->vtpcd);
	if (rc) goto disable_vdig;

	idx = VX6852_CTRL_VT_SYS_CLK_DIV;
	rc = vx6852_set_control(data, idx, data->vtscd);
	if (rc) goto disable_vdig;

	/* TODO: should be revision dependent */
	idx = VX6852_CTRL_0x3614;
	rc = vx6852_set_control(data, idx, data->vtscd);
	if (rc) goto disable_vdig;

	idx = VX6852_CTRL_PRE_PLL_CLK_DIV;
	rc = vx6852_set_control(data, idx, data->ppllcd);
	if (rc) goto disable_vdig;

	idx = VX6852_CTRL_PLL_MULTIPLIER;
	rc = vx6852_set_control(data, idx, data->pllm);
	if (rc) goto disable_vdig;

	idx = VX6852_CTRL_LINE_LENGTH_PCK;
	rc = vx6852_set_control(data, idx, data->llpc);
	if (rc) goto disable_vdig;
#endif // CONFIG_VIDEO_VX6852_SETTING
powered_on:
	data->power = 1;
	rc = 0;
	goto exit;
disable_vdig:
	if (data->enable_vdig)
		(void)data->enable_vdig(0);
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static void
vx6852_set_power_off(struct vx6852_client_data *data)
{
	int i;
	u16 idx;

	SPEW(1, "+++ %s\n", __func__);

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	printk("vx6852-poweroff\n");
#endif
	idx = VX6852_CTRL_MODE_SELECT;
	(void)vx6852_set_control(data, idx, MODE_SELECT_SOFTWARE_STANDBY);

	if (data->enable_vdig)
		(void)data->enable_vdig(0);

	for (i = 0; VX6852_CTRL_COUNT > i; ++i) {
		if (VX6852_CTRL_PIXEL_CLOCK != i)
			data->ctrls[i].flags &= ~CTRL_FLAG_CACHED;
	}

	data->power = 0;

	SPEW(1, "--- %s\n", __func__);
}

static int
vx6852_ioctl_s_power(struct v4l2_int_device *dev, int power)
{
	int rc = 0;
	struct vx6852_client_data *data;

	data = container_of(dev, struct vx6852_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);

	if (power && !data->power)
		rc = vx6852_set_power_on(data, 1);

	else if (!power && data->power)
		vx6852_set_power_off(data);

	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
vx6852_ioctl_g_ifparm(struct v4l2_int_device *dev, struct v4l2_ifparm *parm)
{
	SPEW(1, "+++ %s\n", __func__);

	*parm = vx6852_ifparm;

	SPEW(1, "--- %s\n", __func__);

	return (0);
}

static struct v4l2_int_ioctl_desc vx6852_ioctls[] = {
	{
	.num = vidioc_int_g_fmt_cap_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_g_fmt,
	},
	{
	.num = vidioc_int_s_fmt_cap_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_s_fmt,
	},
	{
	.num = vidioc_int_try_fmt_cap_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_try_fmt,
	},
	{
	.num = vidioc_int_g_ctrl_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_g_ctrl,
	},
	{
	.num = vidioc_int_s_ctrl_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_s_ctrl,
	},
	{
	.num = vidioc_int_g_parm_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_g_parm,
	},
	{
	.num = vidioc_int_s_parm_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_s_parm,
	},
	{
	.num = vidioc_int_streamon_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_streamon,
	},
	{
	.num = vidioc_int_streamoff_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_streamoff,
	},
	{
	.num = vidioc_int_s_power_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_s_power,
	},
	{
	.num = vidioc_int_g_ifparm_num,
	.func = (v4l2_int_ioctl_func *)vx6852_ioctl_g_ifparm,
	},
};

static int
vx6852_i2c_probe(struct i2c_client *i2c)
{
	u8 rn;
	u8 fw_maj;
	u8 fw_min;
	u8 patch_maj;
	u8 patch_min;
	int rc;
	u16 id;
	struct v4l2_pix_format fmt;
	struct v4l2_captureparm parm;
	struct vx6852_client_data *data;
	struct vx6852_platform_data *pdata;

	SPEW(1, "%s\n", __func__);

	if (!(pdata = i2c->dev.platform_data)) {
		rc = -ENODEV;
		goto failed;
	}
	if (!(data = kzalloc(sizeof(struct vx6852_client_data), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto failed;
	}

	mutex_init(&data->lock);
	data->i2c = i2c;
	data->ccp2df = pdata->ccp2_data_format;
	data->cit = pdata->coarse_integration_time;
	data->agcg = pdata->analogue_gain_code_global;
	data->vtpc = pdata->vt_pix_clk_mhz;
	data->vtpcd = pdata->vt_pix_clk_div;
	data->vtscd = pdata->vt_sys_clk_div;
	data->ppllcd = pdata->pre_pll_clk_div;
	data->pllm = pdata->pll_multiplier;
	data->llpc = pdata->line_length_pck;
	data->fourcc = pdata->fourcc;
	data->bpp = pdata->bpp;
	data->vdig = !pdata->enable_vdig;
	data->enable_vdig = pdata->enable_vdig;
	data->cci_data_fmt = vx6852_cci_data_formats[0].format;

	/* Initialize format and parameters */
	memset(&fmt, 0, sizeof(fmt));
	fmt.width = -1;
	fmt.height = -1;
	memset(&parm, 0, sizeof(parm));
	parm.timeperframe.denominator = 1;
	vx6852_format_to_config(&fmt, &data->cfg);
	vx6852_parm_to_config(data, &parm, &data->cfg);

	memcpy(data->ctrls, vx6852_controls, sizeof(vx6852_controls));
	data->ctrls[VX6852_CTRL_PIXEL_CLOCK].value =
		(data->vtpc.numerator * 1000000) / data->vtpc.denominator;
	data->ctrls[VX6852_CTRL_PIXEL_CLOCK].flags |= CTRL_FLAG_CACHED;

	if ((rc = vx6852_set_power_on(data, 0)))
		goto vx6852_set_power_on_failed;

	if ((rc = vx6852_i2c_read_u16(i2c, MODEL_ID, &id))
		|| (rc = vx6852_i2c_read_u8(i2c, REVISION_NUMBER, &rn))
		|| (rc = vx6852_i2c_read_u8(i2c, ILP_REVISION_ID, &data->ilp))
		|| (rc = vx6852_i2c_read_u8(i2c, FIRMWARE_VER_MAJOR, &fw_maj))
		|| (rc = vx6852_i2c_read_u8(i2c, FIRMWARE_VER_MINOR, &fw_min))
		|| (rc = vx6852_i2c_read_u8(i2c, PATCH_VER_MAJOR, &patch_maj))
		|| (rc = vx6852_i2c_read_u8(i2c, PATCH_VER_MINOR, &patch_min))
		|| (MODEL_ID_VX6852 != id))
		goto unknown_model;

#ifdef CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS
	vx6852_config_hacks(&data->cfg, data->ilp);
#endif /* CONFIG_VIDEO_VX6852_SILICON_BUG_HACKS */
#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
	printk("vx6852-description,time,addr-high,addr-low,data1,[data2]\n");
#endif
	vx6852_set_power_off(data);
	strncpy(data->slave.attach_to, V4L2_INT_SMIA10, V4L2NAMESIZE);
	data->slave.num_ioctls = ARRAY_SIZE(vx6852_ioctls);
	data->slave.ioctls = vx6852_ioctls;
	strncpy(data->dev.name, VX6852_I2C_DEVICE, V4L2NAMESIZE);
	data->dev.type = v4l2_int_type_slave;
	data->dev.u.slave = &data->slave;

	if ((rc = v4l2_int_device_register(&data->dev)))
		goto v4l2_int_device_register_failed;

	i2c_set_clientdata(i2c, data);

	if ((rc = device_create_file(&i2c->dev, &vx6852_attr_vdig)))
		goto device_create_vdig_file_failed;

	if ((rc = device_create_file(&i2c->dev, &vx6852_attr_cci_data)))
		goto device_create_cci_data_file_failed;

	if ((rc = device_create_file(&i2c->dev, &vx6852_attr_cci_data_format)))
		goto device_create_cci_data_format_file_failed;

	if ((rc = device_create_file(&i2c->dev, &vx6852_attr_cci_index)))
		goto device_create_cci_index_file_failed;

	if ((rc = vx6852_create_settings(&i2c->dev)))
		goto vx6852_create_settings_failed;

	if ((rc = vx6852_create_spew_level(&i2c->dev)))
		goto vx6852_create_spew_level_failed;

	if ((rc = vx6852_dbg_register_i2c_client(i2c)))
		goto vx6852_dbg_register_i2c_client_failed;

	printk(KERN_INFO VX6852_I2C_DRIVER ": silicon id %u, nvm v%u"
		", ilp v%u.%u, firmware v%u.%u, patch v%u.%u\n",
		SILICON_ID(rn), NVM_VERSION(rn), ILP_REVISION_MAJOR(data->ilp),
		ILP_REVISION_MINOR(data->ilp), fw_maj, fw_min, patch_maj,
		patch_min);

	return (0);

vx6852_dbg_register_i2c_client_failed:
	vx6852_remove_spew_level(&i2c->dev);
vx6852_create_spew_level_failed:
	vx6852_remove_settings(&i2c->dev);
vx6852_create_settings_failed:
	device_remove_file(&i2c->dev, &vx6852_attr_cci_index);
device_create_cci_index_file_failed:
	device_remove_file(&i2c->dev, &vx6852_attr_cci_data_format);
device_create_cci_data_format_file_failed:
	device_remove_file(&i2c->dev, &vx6852_attr_cci_data);
device_create_cci_data_file_failed:
	device_remove_file(&i2c->dev, &vx6852_attr_vdig);
device_create_vdig_file_failed:
	v4l2_int_device_unregister(&data->dev);
v4l2_int_device_register_failed:
unknown_model:
	vx6852_set_power_off(data);
vx6852_set_power_on_failed:
	kfree(data);
failed:
	printk(KERN_ERR VX6852_I2C_DRIVER ": probe failed, rc=%d\n", rc);

	return (rc);
}

static int
vx6852_i2c_remove(struct i2c_client *i2c)
{
	struct vx6852_client_data *data;
	struct vx6852_platform_data *pdata;

	pdata = i2c->dev.platform_data;
	data = i2c_get_clientdata(i2c);

	vx6852_dbg_unregister_i2c_client(i2c);
	vx6852_remove_spew_level(&i2c->dev);
	vx6852_remove_settings(&i2c->dev);

#ifdef CONFIG_VIDEO_VX6852_SETTING
	if (data->once.fw)
		release_firmware(data->once.fw);
#endif // CONFIG_VIDEO_VX6852_SETTING
	device_remove_file(&i2c->dev, &vx6852_attr_cci_data);
	device_remove_file(&i2c->dev, &vx6852_attr_cci_data_format);
	device_remove_file(&i2c->dev, &vx6852_attr_cci_index);
	device_remove_file(&i2c->dev, &vx6852_attr_vdig);
	v4l2_int_device_unregister(&data->dev);
	(void)vx6852_ioctl_s_power(&data->dev, 0);
	kfree(data);

	return (0);
}

static struct i2c_driver vx6852_i2c_driver = {
	.driver = {
		.name = VX6852_I2C_DRIVER,
	},
	.probe = vx6852_i2c_probe,
	.remove = __devexit_p(vx6852_i2c_remove),
	/* NOTE: power is managed by v4l2 master */
};

static int __init
vx6852_module_init(void)
{
	return (i2c_add_driver(&vx6852_i2c_driver));
}

static void __exit
vx6852_module_exit(void)
{
	i2c_del_driver(&vx6852_i2c_driver);
}

module_init(vx6852_module_init);
module_exit(vx6852_module_exit);

MODULE_DESCRIPTION("ST Vx6852 sensor driver");
MODULE_LICENSE("GPL");
