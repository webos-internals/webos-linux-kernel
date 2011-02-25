/*
 * drivers/media/video/vx6953.c
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
#include <media/smia10.h>
#include <media/v4l2-int-device.h>
#include <media/vx6953.h>

//#define CONFIRM_LSC_WRITES
#define LSC_ERROR_IMPROVEMENT_HACK
#define USE_ANALOG_BINNING
#define WAIT_TIMEOUT				1000

#define MODEL_ID				0x0000
#define MODEL_ID_VX6953				953

#define REVISION_NUMBER				0x0002
#define SILICON_ID(rev)				(0xF & ((rev) >> 4))
#define NVM_VERSION(rev)			(0xF & (rev))
#define MODULE_REV_MINOR			0x0010

#define FRAME_COUNT				0x0005
#define MODE_SELECT				0x0100
#define MODE_SELECT_SOFTWARE_STANDBY		0x00
#define MODE_SELECT_STREAMING			0x01

#define DUMMY_PIX_NUM				0

#define IMAGE_ORIENTATION			0x0101
#define SOFTWARE_RESET				0x0103
#define SOFTWARE_RESET_SOFT_RESET		0x01

#define GROUPED_PARAMETER_HOLD			0x0104
#define CSI_SIGNALING_MODE			0x0111
#define CCP2_DATA_FORMAT			0x0112
#define EXT_CLK					0x0136

#define MAX_FRAME_LENGTH_LINES			65535
#define MIN_FRAME_BLANKING_LINES		15

#define FINE_INTEGRATION_TIME			0x0200
#define MIN_FINE_INTEGRATION_TIME		596
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
#define OP_PIX_CLK_DIV				0x0308
#define OP_SYS_CLK_DIV				0x030A
#define FRAME_LENGTH_LINES			0x0340

#define LINE_LENGTH_PCK				0x0342
#define X_ADDR_START				0x0344
#define Y_ADDR_START				0x0346
#define X_ADDR_END				0x0348
#define Y_ADDR_END				0x034A
#define X_OUTPUT_SIZE				0x034C
#define Y_OUTPUT_SIZE				0x034E
#define X_EVEN_INC				0x0380
#define X_ODD_INC				0x0382
#define Y_EVEN_INC				0x0384
#define Y_ODD_INC				0x0386
#define BINNING_MODE				0x0900
#define BINNING_TYPE				0x0901
#define BINNING_WEIGHTING			0x0902
#define MAX_PLL_IP_FREQ_MHZ			0x1110
#define SHADING_CORRECTION			0x0B00
#define MAN_SPEC_DERATE_ENABLE			0x3210

#define SENSOR_STATUS				0x3368
#define SENSOR_STATUS_STANDBY			1
#define STANDBY_TIMEOUT_MS			66

#define EDOF_MODE				0x0B80
#define EDOF_SHARPNESS				0x0B83
#define EDOF_DENOISING				0x0B84
#define EDOF_MODULE_SPECIFIC			0x0B85
#define EDOF_FOCUS_DISTANCE			0x0B88
#define EDOF_ESTIMATION_CONTROL			0X0B8A
#define HOST_WB_STATS_GREEN_RED			0x0B8E
#define HOST_WB_STATS_RED			0x0B90
#define HOST_WB_STATS_BLUE			0x0B92
#define HOST_WB_STATS_GREEN_BLUE		0x0B94

#define EDOF_MODE_CAPTURE			0x01
#define EDOF_MODE_PREVIEW			0x02

/* TODO: remove this define, only defined to allow successful compilation */
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

#define CTRL_ID2INDEX(id)			((id) - V4L2_CID_VX6953_BASE)

#define INVARIANT(cond)						\
	do {							\
		if (!(cond)) {					\
			printk(KERN_DEBUG "!(" # cond ")\n");	\
			BUG();					\
		}						\
	} while (0)

#define SPEW(level, args...)					\
	do {							\
		if (vx6953_spew_level >= level)			\
			printk("VX6953:\t" args);	\
	} while (0)

union vx6953_value {
	u8	ui8;
	u16	ui16;
	u32	ui32;
};

struct vx6953_control {
	u16	reg;
	u16	flags;
	u32	value;
	u32	min;
	u32	max;
};

/* sensor revision number */
enum {
	SREV_UNSUPORTED = -1,
	SREV_CUT2 = 0,
	SREV_CUT3,
	SREV_NUM,
};

static int g_sensor_rev = SREV_UNSUPORTED;

struct vx6953_config {
	int	hqm_supported;	/* high quality mode supported?*/
	u16	xas;	/* x address start */
	u16	xae;	/* x address end */
	u16	xos;	/* x output size */
	u16	yas;	/* y address start */
	u16	yae;	/* y address end */
	u16	yos;	/* y output size */
	u16	xoi;	/* x odd increment */
	u16	yoi;	/* y odd increment */
	u8	bin_mode;	/* binning mode */
	u8	bin_type;
	u8	bin_weighting;
	u8	hqm_used;	/* hqm used? */
	u16	fll;	/* frame length (lines) */
	u16	cit;	/* coarse integration time */

	// derating related paramters
	u8      derate_enable;
	u16     vtpcd;
	u16     llpc;
	u16     fll_margin;
 	u16	pll_op;	/* op pix divider */
 	u16	ccp2df;	/* ccp2 data format */
 	u32	bpp;	/* bytes per pixel */
};

#ifdef CONFIG_VIDEO_VX6953_SETTING
struct vx6953_setting {
	char			name[FIRMWARE_NAME_MAX];
	const struct firmware	*fw;
};
#endif // CONFIG_VIDEO_VX6953_SETTING

struct vx6953_cci_data_format {
	const char *name;
	const char *format;
};

struct vx6953_client_data {
	/* TODO: remove me */
	struct mutex		lock;
	struct i2c_client	*i2c;
	u32			fourcc;
	u8			ilp;	/* ilp revision id */
	u16			cit;	/* coarse integration time */
	u16			agcg;	/* analog gain code (global) */
	struct v4l2_fract	vtpc;	/* video timing pixel clock */
	u16			vtscd;	/* video timing system clock divider */
	u16			ppllcd;	/* pre-pll clock divider */
	u16			pllm;	/* pll multiplier */
	u16			op_sys_clk_div; 
	u8			csi_signaling_mode;
	u16			cci_idx;
	const char		*cci_data_fmt;
	int			vdig;
	int			power;
	int			(*enable_vdig)(int);
#ifdef CONFIG_VIDEO_VX6953_SETTING
	struct vx6953_setting	once;
#endif // CONFIG_VIDEO_VX6953_SETTING
	struct vx6953_config	cfg;
	struct vx6953_control	ctrls[VX6953_CTRL_COUNT];
	struct v4l2_int_slave	slave;
	struct v4l2_int_device	dev;
	struct v4l2_fract	extclk;
};

/* TODO: move these to a flat structure for batching */
static struct vx6953_config vx6953_configs[] = {
	{ /* 5MP */
	.hqm_supported = 1,
	.xas = 0,
	.xae = VX6953_RES_WIDTH - 1,
	.xos = VX6953_RES_WIDTH,
	.yas = 0,
	.yae = VX6953_RES_HEIGHT - 1,
	.yos = VX6953_RES_HEIGHT,
	.xoi = 1,
	.yoi = 1,
	.bin_mode = 0,

	.derate_enable = 0,
#ifdef USE_10BIT_STILL_CAPTURE
	.ccp2df = VX6953_CCP2_DATA_FORMAT_RAW10,
	.bpp 	= 10,
	.pll_op = 10,
	.vtpcd = 10,
#else
	.ccp2df = VX6953_CCP2_DATA_FORMAT_RAW10_8_DPCM,
	.bpp 	= 8,
	.pll_op = 8,
	.vtpcd = 8,
#endif
	.llpc = 2750,
	.fll_margin = 34,
	},

	{ /* 1296x980 */
	.hqm_supported = 0,
	.xas = 0,
	.xae = VX6953_RES_WIDTH - 1,
	.xos = 1296,
	.yas = 0,
	.yae = VX6953_RES_HEIGHT - 1,
	.yos = 980,
	.xoi = 3,
	.yoi = 3,
#ifdef USE_ANALOG_BINNING
	.bin_mode = 1,
	.bin_type = 0x22,
	.bin_weighting = 0x04,
#else
	/* decimation mode */
	.bin_mode = 0,
#endif
	.derate_enable = 1,
	.vtpcd = 7,
	.llpc = 2934,
	.fll_margin = 52,
	.ccp2df = VX6953_CCP2_DATA_FORMAT_RAW10_8_DPCM,
	.bpp 	= 8,
	.pll_op = 8,
	},
};

static struct vx6953_control vx6953_controls[VX6953_CTRL_COUNT] = {
	[VX6953_CTRL_PIXEL_CLOCK] = {
		.reg = -1,
		.flags = CTRL_FLAG_RO,
	},
	[VX6953_CTRL_MODE_SELECT] = {
		.reg = MODE_SELECT,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6953_CTRL_IMAGE_ORIENTATION] = {
		.reg = IMAGE_ORIENTATION,
		.flags = CTRL_FLAG_8_BIT,
	},
	[VX6953_CTRL_GROUPED_PARAMETER_HOLD] = {
		.reg = GROUPED_PARAMETER_HOLD,
		.flags = CTRL_FLAG_8_BIT,
	},
	[VX6953_CTRL_CCP2_DATA_FORMAT] = {
		.reg = CCP2_DATA_FORMAT,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_FINE_INTEGRATION_TIME] = {
		.reg = FINE_INTEGRATION_TIME,
		.flags = CTRL_FLAG_16UI | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_FINE_INTEGRATION_TIME,
		.max = MAX_FINE_INTEGRATION_TIME,
	},
	[VX6953_CTRL_COARSE_INTEGRATION_TIME] = {
		.reg = COARSE_INTEGRATION_TIME,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_ANALOGUE_GAIN_CODE_GLOBAL] = {
		.reg = ANALOGUE_GAIN_CODE_GLOBAL,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_DIGITAL_GAIN_GREENR] = {
		.reg = DIGITAL_GAIN_GREENR,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6953_CTRL_DIGITAL_GAIN_RED] = {
		.reg = DIGITAL_GAIN_RED,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6953_CTRL_DIGITAL_GAIN_BLUE] = {
		.reg = DIGITAL_GAIN_BLUE,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6953_CTRL_DIGITAL_GAIN_GREENB] = {
		.reg = DIGITAL_GAIN_GREENB,
		.flags = CTRL_FLAG_16UR | CTRL_FLAG_MIN | CTRL_FLAG_MAX,
		.min = MIN_DIGITAL_GAIN,
		.max = MAX_DIGITAL_GAIN,
	},
	[VX6953_CTRL_VT_PIX_CLK_DIV] = {
		.reg = VT_PIX_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_VT_SYS_CLK_DIV] = {
		.reg = VT_SYS_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_PRE_PLL_CLK_DIV] = {
		.reg = PRE_PLL_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_PLL_MULTIPLIER] = {
		.reg = PLL_MULTIPLIER,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_OP_SYS_CLK_DIV] = {
		.reg = OP_SYS_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_OP_PIX_CLK_DIV] = {
		.reg = OP_PIX_CLK_DIV,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_EXT_CLK] = {
		.reg = EXT_CLK,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_CSI_SIGNALING_MODE] = {
		.reg = CSI_SIGNALING_MODE,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6953_CTRL_FRAME_LENGTH_LINES] = {
		.reg = FRAME_LENGTH_LINES,
		.flags = CTRL_FLAG_16UI | CTRL_FLAG_MAX,
		.max = MAX_FRAME_LENGTH_LINES,
	},
	[VX6953_CTRL_LINE_LENGTH_PCK] = {
		.reg = LINE_LENGTH_PCK,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_SHADING_CORRECTION] = {
		.reg = SHADING_CORRECTION,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6953_CTRL_MAN_SPEC_DERATE_ENABLE] = {
		.reg = MAN_SPEC_DERATE_ENABLE,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6953_CTRL_X_ADDR_START] = {
		.reg = X_ADDR_START,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_Y_ADDR_START] = {
		.reg = Y_ADDR_START,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_X_ADDR_END] = {
		.reg = X_ADDR_END,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_Y_ADDR_END] = {
		.reg = Y_ADDR_END,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_X_OUTPUT_SIZE] = {
		.reg = X_OUTPUT_SIZE,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_Y_OUTPUT_SIZE] = {
		.reg = Y_OUTPUT_SIZE,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_X_EVEN_INC] = {
		.reg = X_EVEN_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_X_ODD_INC] = {
		.reg = X_ODD_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_Y_EVEN_INC] = {
		.reg = Y_EVEN_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_Y_ODD_INC] = {
		.reg = Y_ODD_INC,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_BINNING_MODE] = {
		.reg = BINNING_MODE,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6953_CTRL_BINNING_TYPE] = {
		.reg = BINNING_TYPE,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6953_CTRL_BINNING_WEIGHTING] = {
		.reg = BINNING_WEIGHTING,
		.flags = CTRL_FLAG_8UI,
	},
	[VX6953_CTRL_HOST_WB_STATS_GREEN_RED] = {
		.reg = HOST_WB_STATS_GREEN_RED,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_HOST_WB_STATS_RED] = {
		.reg = HOST_WB_STATS_RED,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_HOST_WB_STATS_BLUE] = {
		.reg = HOST_WB_STATS_BLUE,
		.flags = CTRL_FLAG_16UI,
	},
	[VX6953_CTRL_HOST_WB_STATS_GREEN_BLUE] = {
		.reg = HOST_WB_STATS_GREEN_BLUE,
		.flags = CTRL_FLAG_16UI,
	},
};

static struct v4l2_ifparm vx6953_ifparm = {
	.if_type = V4L2_IF_TYPE_SMIA10,
	.u.smia10 = {
		.nr_cols = 2,
		.nr_rows = 2,
		.descs[0] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_VISIBLE_PIXEL_DATA,
			.nr = VX6953_RES_WIDTH,
		},
		.descs[1] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_DUMMY_PIXEL_DATA,
			.nr = DUMMY_PIX_NUM,
		},
		.descs[2] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_EMBEDDED_DATA,
			.nr = 2,
		},
		.descs[3] = {
			.code = V4L2_IF_TYPE_SMIA10_CODE_VISIBLE_PIXEL_DATA,
			.nr = VX6953_RES_HEIGHT,
		},
	},
};

static struct vx6953_cci_data_format vx6953_cci_data_formats[] = {
	{"octal", "%03o\n"},
	{"decimal", "%u\n"},
	{"hex", "%02x\n"},
};

static int vx6953_spew_level = 0;

#ifdef LSC_ERROR_IMPROVEMENT_HACK
struct lsc_offset {
	u16 addr;
	int offset;
};

static u16 lsc_nvm_reg_addr[] = {
	0xFAB8,
	0XFABA,
	0XFABC,
	0XFABE,
	0XFAC0,
	0XFAC2,
	0XFAC4,
	0XFAC6,
	0XFAC8,
	0XFACA,
	0XFACC,
	0XFACE,
	0XFAD0,
	0XFAD2,
	0XFAD4,
	0XFAD6,
	0XFAD8,
	0XFADA,
	0XFADC,
	0XFADE,
	0XFAE0,
	0XFAE2,
	0XFAE4,
	0XFAE6,
	0XFAE8,
	0XFAEA,
	0XFAEC,
	0XFAEE,
	0XFAF0,
	0XFAF2,
	0XFAF4,
	0XFAF6,
	0XFAF8,
};

static struct lsc_offset new_lsc_reg_offset_d65[] = {
	{0xFAB8, 5},
	{0XFABA, 3},
	{0XFABC, -4},
	{0XFABE, -5},
	{0XFAC0, -1},
	{0XFAC2, 3},
	{0XFAC4, 4},
	{0XFAC6, 8},
	{0XFAC8, 12},
	{0XFACA, 3},
	{0XFACC, -22},
	{0XFACE, -21},
	{0XFAD0, 7},
	{0XFAD2, 17},
	{0XFAD4, 2},
	{0XFAD6, 35},
	{0XFAD8, 8},
	{0XFADA, 2},
	{0XFADC, -3},
	{0XFADE, -2},
	{0XFAE0, -9},
	{0XFAE2, -5},
	{0XFAE4, 10},
	{0XFAE6, 4},
	{0XFAE8, 9},
	{0XFAEA, 2},
	{0XFAEC, -4},
	{0XFAEE, -5},
	{0XFAF0, -2},
	{0XFAF2, 0},
	{0XFAF4, -6},
	{0XFAF6, 9},
	{0XFAF8, 0},

};
static struct lsc_offset new_lsc_reg_offset_cw[] = {
	{0xFAFA, -10},
	{0XFAFC, -15},
	{0XFAFE, -4},
	{0XFB00, -6},
	{0XFB02, -8},
	{0XFB04, -3},
	{0XFB06, -63},
	{0XFB08, 8},
	{0XFB0A, 25},
	{0XFB0C, -9},
	{0XFB0E, -39},
	{0XFB10, -51},
	{0XFB12, -9},
	{0XFB14, 0},
	{0XFB16, -21},
	{0XFB18, 112},
	{0XFB1A, -10},
	{0XFB1C, -23},
	{0XFB1E, -7},
	{0XFB20, -9},
	{0XFB22, 10},
	{0XFB24, 18},
	{0XFB26, -11},
	{0XFB28, 23},
	{0XFB2A, -10},
	{0XFB2C, -15},
	{0XFB2E, -4},
	{0XFB30, -6},
	{0XFB32, -10},
	{0XFB34, -3},
	{0XFB36, -52},
	{0XFB38, 7},
	{0XFB3A, 0},

};
static struct lsc_offset new_lsc_reg_offset_tl83[] = {
	{0xFB3C, -29},
	{0XFB3E, -18},
	{0XFB40, 6},
	{0XFB42, 1},
	{0XFB44, -17},
	{0XFB46, -35},
	{0XFB48, -77},
	{0XFB4A, 0},
	{0XFB4C, 5},
	{0XFB4E, -17},
	{0XFB50, -6},
	{0XFB52, -22},
	{0XFB54, -41},
	{0XFB56, -1},
	{0XFB58, -37},
	{0XFB5A, 83},
	{0XFB5C, -38},
	{0XFB5E, -32},
	{0XFB60, 1},
	{0XFB62, -2},
	{0XFB64, 15},
	{0XFB66, 25},
	{0XFB68, -67},
	{0XFB6A, 19},
	{0XFB6C, -28},
	{0XFB6E, -22},
	{0XFB70, 5},
	{0XFB72, 2},
	{0XFB74, -18},
	{0XFB76, 21},
	{0XFB78, -86},
	{0XFB7A, 0},
	{0XFB7C, 0},

};static struct lsc_offset new_lsc_reg_offset_horizon[] = {
	{0xFB7E, -30},
	{0XFB80, -20},
	{0XFB82, 8},
	{0XFB84, 11},
	{0XFB86, -16},
	{0XFB88, -26},
	{0XFB8A, -35},
	{0XFB8C, -53},
	{0XFB8E, -9},
	{0XFB90, -10},
	{0XFB92, 44},
	{0XFB94, 57},
	{0XFB96, -39},
	{0XFB98, -14},
	{0XFB9A, 50},
	{0XFB9C, -173},
	{0XFB9E, -38},
	{0XFBA0, -32},
	{0XFBA2, -1},
	{0XFBA4, 9},
	{0XFBA6, 39},
	{0XFBA8, 51},
	{0XFBAA, -33},
	{0XFBAC, -49},
	{0XFBAE, -28},
	{0XFBB0, -22},
	{0XFBB2, 7},
	{0XFBB4, 11},
	{0XFBB6, -21},
	{0XFBB8, 17},
	{0XFBBA, -62},
	{0XFBBC, -56},
	{0XFBBE, 0},

};

#endif

#ifdef CONFIG_VIDEO_VX6953_DBG
static ssize_t
vx6953_show_spew_level(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t i;

	i = snprintf(buf, PAGE_SIZE, "%d\n", vx6953_spew_level);

	return (i + 1);
}

static ssize_t
vx6953_store_spew_level(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	char *endp;

	vx6953_spew_level = simple_strtoul(buf, &endp, 10);

	return (count);
}

static struct device_attribute vx6953_attr_spew_level =
	__ATTR(spew_level, S_IRUGO|S_IWUGO, vx6953_show_spew_level,
		vx6953_store_spew_level);

static inline int
vx6953_create_spew_level(struct device *dev)
{
	return (device_create_file(dev, &vx6953_attr_spew_level));
}

static inline void
vx6953_remove_spew_level(struct device *dev)
{
	device_remove_file(dev, &vx6953_attr_spew_level);
}
#else // !CONFIG_VIDEO_VX6953_DBG
static inline int
vx6953_create_spew_level(struct device *dev)
{
	return (0);
}

static inline void
vx6953_remove_spew_level(struct device *dev)
{
}
#endif // CONFIG_VIDEO_VX6953_DBG

static ssize_t
vx6953_attr_show_cci_data(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	u8 val;
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

	if ((rc = vx6953_i2c_read_u8(i2c, data->cci_idx++, &val)))
		goto exit;

	rc = sprintf(buf, data->cci_data_fmt, val);
exit:
	return (rc);
}

static ssize_t
vx6953_attr_store_cci_data(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	unsigned int val;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!sscanf(buf, data->cci_data_fmt, &val)) {
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = vx6953_i2c_write_u8(i2c, data->cci_idx++, val)))
		goto exit;

	rc = count;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute vx6953_attr_cci_data =
	__ATTR(cci_data, S_IRUGO|S_IWUGO, vx6953_attr_show_cci_data,
		vx6953_attr_store_cci_data);

static ssize_t
vx6953_attr_show_cci_data_format(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i;
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);
	struct vx6953_cci_data_format *fmt;

	for (i = 0, rc = 0; i < ARRAY_SIZE(vx6953_cci_data_formats); ++i) {
		fmt = &vx6953_cci_data_formats[i];
		rc += snprintf(buf + rc, PAGE_SIZE - rc, "%s%s", fmt->name,
				!strcmp(fmt->format, data->cci_data_fmt)
					? "* " : " ");
	}

	rc += snprintf(buf + rc, PAGE_SIZE - rc, "\n");

	return (rc);
}

static ssize_t
vx6953_attr_store_cci_data_format(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int i;
	int rc = -EINVAL;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	for (i = 0; i < ARRAY_SIZE(vx6953_cci_data_formats); ++i) {
		if (!strncmp(buf, vx6953_cci_data_formats[i].name, count)) {
			data->cci_data_fmt = vx6953_cci_data_formats[i].format;
			rc = count;
			break;
		}
	}

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute vx6953_attr_cci_data_format =
	__ATTR(cci_data_format, S_IRUGO|S_IWUGO,
		vx6953_attr_show_cci_data_format,
		vx6953_attr_store_cci_data_format);

static ssize_t
vx6953_attr_show_cci_index(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

	rc = sprintf(buf, "0x%04X\n", data->cci_idx);

	return (rc);
}

static ssize_t
vx6953_attr_store_cci_index(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	unsigned int idx;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

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

static struct device_attribute vx6953_attr_cci_index =
	__ATTR(cci_index, S_IRUGO|S_IWUGO, vx6953_attr_show_cci_index,
		vx6953_attr_store_cci_index);

static ssize_t
vx6953_attr_show_vdig(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

	rc = sprintf(buf, "%d\n", data->vdig);

	return (rc);
}

static int
vx6953_enable_vdig(struct vx6953_client_data *data, int enable)
{
	int rc = 0;

	if ((enable != data->vdig) && data->enable_vdig)
		rc = data->enable_vdig((data->vdig = enable));

	return (rc);
}

static ssize_t
vx6953_attr_store_vdig(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int rc;
	int vdig;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!sscanf(buf, "%d", &vdig)) {
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = vx6953_enable_vdig(data, !!vdig)))
		goto exit;

	rc = count;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute vx6953_attr_vdig =
	__ATTR(vdig, S_IRUGO|S_IWUGO, vx6953_attr_show_vdig,
		vx6953_attr_store_vdig);

int
vx6953_i2c_read_raw(struct i2c_client* i2c, u16 index, u8* buf, u16 len)
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

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	{
		int i;
		printk("vx6953-i2c-r,,");
		for (i = 0; i < 2; i++) {
			printk("%02x,", msgs[0].buf[i]);
		}

		for (i = 0; i < len; i++) {
			printk("%02x,", buf[i]);
		}
		printk("\n");
	}
#endif
	return (min(rc, 0));
}
EXPORT_SYMBOL(vx6953_i2c_read_raw);

int
vx6953_i2c_write_raw(struct i2c_client* i2c, u8* buf, u16 len)
{
	int rc;
	struct i2c_msg msg;

	msg.addr = i2c->addr;
	msg.len = len;
	msg.flags = 0;
	msg.buf = buf;

	rc = i2c_transfer(i2c->adapter, &msg, 1);

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	{
		int i;
		printk("vx6953-i2c-w,,");
		for (i = 0; i < len; i++) {
			printk("%02x,", buf[i]);
		}
		printk("\n");
	}
#endif
	return (min(rc, 0));
}
EXPORT_SYMBOL(vx6953_i2c_write_raw);

static int
vx6953_i2c_mwait_u8(struct i2c_client* i2c, u16 idx, u8 val, unsigned int ms)
{
	u8 buf;
	int rc;
	unsigned long expiry = jiffies + msecs_to_jiffies(ms);

	do {
		if ((rc = vx6953_i2c_read_raw(i2c, idx, &buf, 1))
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

#ifdef CONFIG_VIDEO_VX6953_SETTING
static int
vx6953_load_setting(struct device *dev, const char *name,
			struct vx6953_setting *setting)
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

#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
static inline s64
timespec_sub_to_ns(struct timespec lhs, struct timespec rhs)
{
	s64 diff;

	diff = timespec_to_ns(&lhs) - timespec_to_ns(&rhs);

	return (diff);
}
#endif //CONFIG_VIDEO_VX6953_SETTING_PROFILE

#define SPEW_I2C_WRITE(lvl, buf, len)					\
	do {								\
		if (vx6953_spew_level >= (lvl))	{			\
			u16 i;						\
			u16 idx = (buf[0] << 8) | buf[1];		\
			for (i = 2; i != (len); i++, idx++)		\
				printk(KERN_DEBUG "VX6953:\t"		\
					"write: 0x%04X = 0x%02X\n",	\
					idx, (buf)[i]);			\
		}							\
	} while(0)

static int
vx6953_apply_setting(struct i2c_client *i2c, struct vx6953_setting *setting)
{
	u8 buf[16];
	int rc;
	u16 idx = 0xFFFF;
	u16 len = 0;
	struct smia10_parse_state state;
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
	s64 __t;
	unsigned int __i = 0;
	unsigned int __w = 0;
	unsigned int __r = 0;
	struct timespec __b;
	struct timespec __e;
	struct timespec __wt = {0, 0};
	struct timespec __rt = {0, 0};
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE

	SPEW(1, "+++ %s\n", __func__);

	if (!setting->fw) {
		rc = -EINVAL;
		goto exit;
	}

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	printk("vx6953-init\n");
#endif
	smia10_parse_init(&state, setting->fw->data, setting->fw->size);

	while (!(rc = smia10_parse_embedded_data(&state)) && state.size) {
		if (state.index != idx) {
			if (len) {
				SPEW_I2C_WRITE(4, buf, len);
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
				getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
				if ((rc = vx6953_i2c_write_raw(i2c, buf, len)))
					goto exit;
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
				getnstimeofday(&__e);
				__w++;
				__t = timespec_sub_to_ns(__e, __b);
				timespec_add_ns(&__wt, __t);
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
			}

			idx = state.index;
			len = 0;
			buf[len++] = idx >> 8;
			buf[len++] = idx;
		}

		buf[len++] = *state.data;
		idx++;
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
		__i++;
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE

		if ((len == sizeof(buf))
			|| (state.index == SOFTWARE_RESET)
			|| (state.index == DAF_DATA_CUSTOM_CTRL_PEAK_DATA)) {
			SPEW_I2C_WRITE(4, buf, len);
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
			getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
			if ((rc = vx6953_i2c_write_raw(i2c, buf, len)))
				goto exit;
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
			getnstimeofday(&__e);
			__w++;
			timespec_add_ns(&__wt, timespec_sub_to_ns(__e, __b));
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE

			idx = 0xFFFF;
			len = 0;
		}

		if (((state.index == SOFTWARE_RESET)
			|| (state.index == DAF_DATA_CUSTOM_CTRL_PEAK_DATA))) {
			SPEW(4, "wait: 0x%04X\n", state.index);
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
			getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
			if ((rc = vx6953_i2c_mwait_u8(i2c, state.index, 0,
							WAIT_TIMEOUT)))
				goto exit;
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
			getnstimeofday(&__e);
			__r++;
			timespec_add_ns(&__rt, timespec_sub_to_ns(__e, __b));
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
		}
	}

	if (len) {
		SPEW_I2C_WRITE(4, buf, len);
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
		getnstimeofday(&__b);
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
		if ((rc = vx6953_i2c_write_raw(i2c, buf, len)))
			goto exit;
#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
		getnstimeofday(&__e);
		__w++;
		timespec_add_ns(&__wt, timespec_sub_to_ns(__e, __b));
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
	}

#ifdef CONFIG_VIDEO_VX6953_SETTING_PROFILE
	SPEW(1, "regs:%u writes:%u in %ld.%09lds waits:%u in %ld.%09lds\n",
		__i,
		__w, __wt.tv_sec, __wt.tv_nsec,
		__r, __rt.tv_sec, __rt.tv_nsec);
#endif // CONFIG_VIDEO_VX6953_SETTING_PROFILE
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static ssize_t
vx6953_attr_show_setting(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_setting *setting = NULL;
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

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
vx6953_attr_store_setting(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc = -EINVAL;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct vx6953_setting *setting = NULL;
	struct vx6953_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!strcmp(attr->attr.name, "register_reprogram_once"))
		setting = &data->once;

	mutex_lock(&data->lock);
	rc = vx6953_load_setting(dev, buf, setting);
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (!rc ? count : rc);
}

static struct device_attribute vx6953_attr_once =
	__ATTR(register_reprogram_once, S_IRUGO|S_IWUGO,
		vx6953_attr_show_setting,
		vx6953_attr_store_setting);

static inline int
vx6953_create_settings(struct device *dev)
{
	return (device_create_file(dev, &vx6953_attr_once));
}

static inline void
vx6953_remove_settings(struct device *dev)
{
	device_remove_file(dev, &vx6953_attr_once);
}
#else // !CONFIG_VIDEO_VX6953_SETTING
static inline int
vx6953_create_settings(struct device *dev)
{
	return (0);
}

static inline void
vx6953_remove_settings(struct device *dev)
{
}
#endif // CONFIG_VIDEO_VX6953_SETTING

static int
vx6953_get_control(struct vx6953_client_data *data, u16 index, u32 *_val)
{
	int rc;
	union vx6953_value val;
	struct vx6953_control *ctrl;

	ctrl = data->ctrls + index;

	SPEW(3, "+++ %s: index=%u\n", __func__, index);

	if (VX6953_CTRL_COUNT <= index) {
		rc = -EINVAL;
		goto exit;
	}
	if (!(CTRL_FLAG_NO_CACHE & ctrl->flags)
		&& (CTRL_FLAG_CACHED & ctrl->flags))
		goto cached;

	switch (CTRL_FLAG_BIT_MASK & ctrl->flags) {
	case CTRL_FLAG_8_BIT:
		rc = vx6953_i2c_read_u8(data->i2c, ctrl->reg, &val.ui8);
		ctrl->value = val.ui8;
		break;
	case CTRL_FLAG_16_BIT:
		rc = vx6953_i2c_read_u16(data->i2c, ctrl->reg, &val.ui16);
		ctrl->value = val.ui16;
		break;
	case CTRL_FLAG_32_BIT:
		rc = vx6953_i2c_read_u32(data->i2c, ctrl->reg, &val.ui32);
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
vx6953_set_control(struct vx6953_client_data *data, u16 index, u32 _val)
{
	int rc;
	union vx6953_value val;
	struct vx6953_control *ctrl;

	ctrl = data->ctrls + index;

	SPEW(3, "+++ %s: index=%u val=%u\n", __func__, index, _val);

	if ((VX6953_CTRL_COUNT <= index)
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
		rc = vx6953_i2c_write_u8(data->i2c, ctrl->reg, val.ui8);
		break;
	case CTRL_FLAG_16_BIT:
		ctrl->value = val.ui16 = (u16)_val;
		rc = vx6953_i2c_write_u16(data->i2c, ctrl->reg, val.ui16);
		break;
	case CTRL_FLAG_32_BIT:
		ctrl->value = val.ui32 = _val;
		rc = vx6953_i2c_write_u32(data->i2c, ctrl->reg, val.ui32);
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

static void vx6953_config_to_format(struct vx6953_client_data *dat,
					struct vx6953_config *cfg,
					struct v4l2_pix_format *fmt)
{
	memset(fmt, 0, sizeof(*fmt));
	fmt->width = cfg->xos;
	fmt->height = cfg->yos;
	fmt->pixelformat = dat->fourcc;
	fmt->bytesperline = ((cfg->xos + DUMMY_PIX_NUM) * cfg->bpp) / 8;
	fmt->sizeimage = fmt->bytesperline * cfg->yos;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

static int vx6953_ioctl_g_fmt(struct v4l2_int_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	struct vx6953_client_data *data;

	data = container_of(dev, struct vx6953_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);
	vx6953_config_to_format(data, &data->cfg, &fmt->fmt.pix);
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

inline void vx6953_set_fll(struct vx6953_config *cfg)
{
	u16 max_fll = cfg->yos + cfg->fll_margin;

	/* clamp and update cit (exposure) accordingly */
	cfg->fll = min(cfg->fll, (u16)MAX_FRAME_LENGTH_LINES);
	cfg->fll = max(max_fll, cfg->fll);
	cfg->cit = min(cfg->cit, (u16)MAX_COARSE_INTEGRATION_TIME(cfg->fll));
}

static void update_pixel_clock(struct vx6953_client_data *d)
{
	/* make sure vtpc numerator does not overflow */
	d->vtpc.numerator = d->extclk.numerator * d->pllm;
	d->vtpc.denominator = 
		d->extclk.denominator * d->ppllcd * d->vtscd * d->cfg.vtpcd;

	d->ctrls[VX6953_CTRL_PIXEL_CLOCK].value =
		(d->vtpc.numerator * 1000000) / d->vtpc.denominator;
	d->ctrls[VX6953_CTRL_PIXEL_CLOCK].flags |= CTRL_FLAG_CACHED;
}

static void vx6953_format_to_config(struct v4l2_pix_format *fmt,
					struct vx6953_config *cfg)
{
	int i;
	struct vx6953_config *best = &vx6953_configs[0];

	for (i = 1; i < ARRAY_SIZE(vx6953_configs); ++i) {
		if ((vx6953_configs[i].xos >= fmt->width)
			&& (vx6953_configs[i].yos >= fmt->height))
			best = &vx6953_configs[i];
	}

	cfg->hqm_supported = best->hqm_supported;
	cfg->xas = best->xas;
	cfg->xae = best->xae;
	cfg->xos = best->xos;
	cfg->yas = best->yas;
	cfg->yae = best->yae;
	cfg->yos = best->yos;
	cfg->xoi = best->xoi;
	cfg->yoi = best->yoi;
	cfg->ccp2df = best->ccp2df;
	cfg->bpp    = best->bpp;
	cfg->pll_op = best->pll_op;
	cfg->bin_mode = best->bin_mode;
	cfg->bin_type = best->bin_type;
	cfg->bin_weighting = best->bin_weighting;
	cfg->derate_enable = best->derate_enable;
	cfg->vtpcd = best->vtpcd;
	cfg->llpc = best->llpc;
	cfg->fll_margin = best->fll_margin;
	vx6953_set_fll(cfg);
}

static int vx6953_ioctl_try_fmt(struct v4l2_int_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	struct vx6953_config cfg;
	struct vx6953_client_data *dat;

	dat = container_of(dev, struct vx6953_client_data, dev);

	SPEW(1, "+++ %s: width=%u height=%u pixelformat=%c%c%c%c\n", __func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24));

	vx6953_format_to_config(&fmt->fmt.pix, &cfg);
	vx6953_config_to_format(dat, &cfg, &fmt->fmt.pix);
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

static int vx6953_ioctl_s_fmt(struct v4l2_int_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	u32 ms;
	struct vx6953_client_data *data;

	data = container_of(dev, struct vx6953_client_data, dev);

	SPEW(1, "+++ %s: width=%u height=%u pixelformat=%c%c%c%c\n", __func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24));

	mutex_lock(&data->lock);

	if ((rc = vx6953_get_control(data, VX6953_CTRL_MODE_SELECT, &ms)))
		goto unlock;

	if (MODE_SELECT_STREAMING == ms) {
		rc = -EBUSY;
		goto unlock;
	}

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	printk("vx6953-set-format\n");
#endif
	vx6953_format_to_config(&fmt->fmt.pix, &data->cfg);
	update_pixel_clock(data);
	vx6953_config_to_format(data, &data->cfg, &fmt->fmt.pix);
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

static void vx6953_config_to_parm(struct vx6953_client_data *dat,
					struct vx6953_config *cfg,
					struct v4l2_captureparm *parm)
{
	u64 num;

	memset(parm, 0, sizeof(*parm));
	parm->capability = V4L2_CAP_TIMEPERFRAME;

	if (cfg->hqm_used)
		parm->capturemode = V4L2_MODE_HIGHQUALITY;

	num = (u64)cfg->llpc * cfg->fll * dat->vtpc.denominator * 10;
	do_div(num, dat->vtpc.numerator);
	parm->timeperframe.numerator = num;
	parm->timeperframe.denominator = 1;
}

static int vx6953_ioctl_g_parm(struct v4l2_int_device *dev,
				struct v4l2_streamparm *parm)
{
	int rc;
	struct vx6953_client_data *data;

	data = container_of(dev, struct vx6953_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);
	vx6953_config_to_parm(data, &data->cfg, &parm->parm.capture);
	mutex_unlock(&data->lock);
	rc = 0;

	SPEW(1, "--- %s: rc=%d highquality=%u timeperframe=%u/%u\n", __func__,
		rc, V4L2_MODE_HIGHQUALITY & parm->parm.capture.capturemode,
		parm->parm.capture.timeperframe.numerator,
		parm->parm.capture.timeperframe.denominator);

	return (rc);
}

static void vx6953_parm_to_config(struct vx6953_client_data *dat,
					struct v4l2_captureparm *parm,
					struct vx6953_config *cfg)
{
	u64 fll;
	u64 den;

	cfg->hqm_used = ((V4L2_MODE_HIGHQUALITY & parm->capturemode) && cfg->hqm_supported);

	fll = (u64)parm->timeperframe.numerator * dat->vtpc.numerator;
	den = (u64)parm->timeperframe.denominator * 10 * cfg->llpc
		* dat->vtpc.denominator;
	fll += den - 1;
	do_div(fll, den);
	vx6953_set_fll(cfg);
}

static int vx6953_ioctl_s_parm(struct v4l2_int_device *dev,
				struct v4l2_streamparm *parm)
{
	int rc;
	u32 val;
	struct vx6953_client_data *data;

	data = container_of(dev, struct vx6953_client_data, dev);

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

	if ((rc = vx6953_get_control(data, VX6953_CTRL_MODE_SELECT, &val)))
		goto unlock;

	if (MODE_SELECT_STREAMING == val) {
		rc = -EBUSY;
		goto unlock;
	}

	vx6953_parm_to_config(data, &parm->parm.capture, &data->cfg);
	vx6953_config_to_parm(data, &data->cfg, &parm->parm.capture);
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

static int vx6953_ioctl_g_ctrl(struct v4l2_int_device *dev,
				struct v4l2_control *ctl)
{
	int rc = 0;
	struct vx6953_client_data *dat;

	dat = container_of(dev, struct vx6953_client_data, dev);

	SPEW(3, "+++ %s: id=0x%08X(index=%u)\n", __func__, ctl->id,
		CTRL_ID2INDEX(ctl->id));

	if ((ctl->id < V4L2_CID_VX6953(0))
		|| (ctl->id > V4L2_CID_VX6953(VX6953_CTRL_COUNT - 1))) {
		rc = -EINVAL;
		goto exit;
	}

	mutex_lock(&dat->lock);

	switch (CTRL_ID2INDEX(ctl->id)) {
	case VX6953_CTRL_FRAME_LENGTH_LINES:
		ctl->value = dat->cfg.fll;
		break;
	case VX6953_CTRL_MIN_FRAME_LENGTH_LINES:
		ctl->value = dat->cfg.yos + dat->cfg.fll_margin;
		break;
	case VX6953_CTRL_LINE_LENGTH_PCK:
		ctl->value = dat->cfg.llpc;
		break;
	case VX6953_CTRL_BINNING_MODE:
		ctl->value = dat->cfg.bin_mode;
		break;
	default:
		rc = vx6953_get_control(dat, CTRL_ID2INDEX(ctl->id),
					&ctl->value);
		break;
	}

	mutex_unlock(&dat->lock);
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int vx6953_ioctl_s_ctrl(struct v4l2_int_device *dev,
				struct v4l2_control *ctl)
{
	int rc;
	struct vx6953_client_data *dat;

	dat = container_of(dev, struct vx6953_client_data, dev);

	SPEW(3, "+++ %s: id=0x%08X(index=%u) value=%u\n", __func__, ctl->id,
		CTRL_ID2INDEX(ctl->id), ctl->value);

	if ((ctl->id < V4L2_CID_VX6953(0))
		|| (ctl->id > V4L2_CID_VX6953(VX6953_CTRL_COUNT - 1))) {
		rc = -EINVAL;
		goto exit;
	}

	if (V4L2_CID_VX6953_SET_TIME == ctl->id) {
#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
		printk("vx6953-frame-cfg-end,%d\n", ctl->value);
#endif
		rc = 0;
		goto exit;
	}
	switch (CTRL_ID2INDEX(ctl->id)) {
	case VX6953_CTRL_COARSE_INTEGRATION_TIME:
		if (ctl->value > MAX_COARSE_INTEGRATION_TIME(dat->cfg.fll)) {
			rc = -EINVAL;
			goto exit;
		}
		break;
	case VX6953_CTRL_FRAME_LENGTH_LINES:
		if (ctl->value < (dat->cfg.yos + dat->cfg.fll_margin)) {
			rc = -EINVAL;
			goto exit;
		}
		break;
	}

	mutex_lock(&dat->lock);

	if ((rc = vx6953_set_control(dat, CTRL_ID2INDEX(ctl->id), ctl->value)))
		goto unlock;

	switch (CTRL_ID2INDEX(ctl->id)) {
	case VX6953_CTRL_COARSE_INTEGRATION_TIME:
		dat->cfg.cit = ctl->value;
		break;
	case VX6953_CTRL_FRAME_LENGTH_LINES:
		dat->cfg.fll = ctl->value;
		break;
	}
unlock:
	mutex_unlock(&dat->lock);
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int vx6953_wait_standby(struct vx6953_client_data *data)
{
	int rc;

	if ((rc = vx6953_i2c_mwait_u8(data->i2c, SENSOR_STATUS,
					SENSOR_STATUS_STANDBY, STANDBY_TIMEOUT_MS)))
		goto exit;

exit:
	return (rc);
}

static int vx6953_ioctl_streamon(struct v4l2_int_device *dev,
					enum v4l2_buf_type type)
{
	u8 buf[18];
	int rc;
	int len;
	u32 ms;
	struct vx6953_client_data *data;
	u16 idx;

	data = container_of(dev, struct vx6953_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);

	if ((rc = vx6953_get_control(data, VX6953_CTRL_MODE_SELECT, &ms)))
		goto unlock;

	if (MODE_SELECT_STREAMING == ms) {
		rc = -EBUSY;
		goto unlock;
	}

	idx = VX6953_CTRL_CCP2_DATA_FORMAT;
	rc = vx6953_set_control(data, idx, data->cfg.ccp2df);
	if (rc)
		goto unlock;

	idx = VX6953_CTRL_OP_PIX_CLK_DIV;
	rc = vx6953_set_control(data, idx, data->cfg.pll_op);
	if (rc)
		goto unlock;

	if ((rc = vx6953_wait_standby(data)))
		goto unlock;
	
	len = 0;
	*((u16 *)&buf[len]) = cpu_to_be16(FRAME_LENGTH_LINES);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.fll);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.llpc);
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

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	printk("vx6953-streamon\n");
#endif
	if ((rc = vx6953_i2c_write_raw(data->i2c, buf, len)))
		goto unlock;

	/* x_even_inc skipped? */
	len = 0;
	*((u16 *)&buf[len]) = cpu_to_be16(X_ODD_INC);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.xoi);
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(1);	/* y_even_inc */
	len += 2;
	*((u16 *)&buf[len]) = cpu_to_be16(data->cfg.yoi);
	len += 2;

	if ((rc = vx6953_i2c_write_raw(data->i2c, buf, len)))
		goto unlock;

	if ((rc = vx6953_i2c_write_u16(data->i2c, COARSE_INTEGRATION_TIME,
					data->cfg.cit)))
		goto unlock;

	if ((rc = vx6953_i2c_write_u8(data->i2c, BINNING_MODE, data->cfg.bin_mode)))
		goto unlock;
	
	if (data->cfg.bin_mode) {
		if ((rc = vx6953_i2c_write_u8(data->i2c, BINNING_TYPE,
						data->cfg.bin_type)))
			goto unlock;
		if ((rc = vx6953_i2c_write_u8(data->i2c, BINNING_WEIGHTING, 
						data->cfg.bin_weighting)))
			goto unlock;
	}
	if ((rc = vx6953_i2c_write_u8(data->i2c, MAN_SPEC_DERATE_ENABLE,
					data->cfg.derate_enable)))
		goto unlock;
	if ((rc = vx6953_i2c_write_u16(data->i2c, VT_PIX_CLK_DIV, data->cfg.vtpcd)))
		goto unlock;
	if ((rc = vx6953_i2c_write_u8(data->i2c, EDOF_MODE, data->cfg.hqm_used ? 
					EDOF_MODE_CAPTURE : EDOF_MODE_PREVIEW)))
		goto unlock;

#ifdef VERBOSE_FIXED
	SPEW(2, "cit=%u fll=%u xos=%u yos=%u xoi=%u yoi=%u vtm=%u iqcc=%u"
		" bin_mode=%u hqm_used=%u\n", data->cfg.cit, data->cfg.fll,
		data->cfg.xos, data->cfg.yos, data->cfg.xoi, data->cfg.yoi,
		data->cfg.vtm, data->cfg.iqcc, data->cfg.bin_mode, data->cfg.hqm_used);
#endif
	rc = vx6953_set_control(data, VX6953_CTRL_MODE_SELECT,
				MODE_SELECT_STREAMING);

	if ((rc = vx6953_i2c_write_u8(data->i2c, EDOF_MODE, data->cfg.hqm_used ? 
					EDOF_MODE_CAPTURE : EDOF_MODE_PREVIEW)))
		goto unlock;
	
unlock:
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
vx6953_ioctl_streamoff(struct v4l2_int_device *dev, enum v4l2_buf_type type)
{
	int rc;
	struct vx6953_client_data *data;

	data = container_of(dev, struct vx6953_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	printk("vx6953-streamoff\n");
#endif
	rc = vx6953_set_control(data, VX6953_CTRL_MODE_SELECT,
				MODE_SELECT_SOFTWARE_STANDBY);
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

#ifdef CONFIRM_LSC_WRITES
static void confirm_write(struct i2c_client *i2c, __u16 addr, __u16 v)
{
	__u16 value;

	if (vx6953_i2c_read_u16(i2c, addr, &value))
		printk(KERN_ERR "%s i2c read failed\n", __func__);

	if (value != v) {
		printk(KERN_ERR "%s: LSC addr %p not written, w=%04x r=%04x\n",
				__func__, addr, v, value);
	}
}
#endif

static int
vx6953_set_power_on(struct vx6953_client_data *data, int init)
{
	int rc;
	u16 idx;

	SPEW(1, "+++ %s\n", __func__);

	if (data->enable_vdig && (rc = data->enable_vdig(1)))
		goto exit;

	/* TODO: 2800 EXTCLK cycles */
	/* XSHUTDOWN --> 1st I2C transaction: 3.1 ms (852) */

	if (!init)
		goto powered_on;

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	printk("vx6953-poweron\n");
#endif
#ifdef ENABLE_SW_RESET_AFTER_POWER_UP
	{
		union vx6953_value val;
		/* NOTE: software reset in case VDIG isn't cut */
		idx = SOFTWARE_RESET;
		val.ui8 = SOFTWARE_RESET_SOFT_RESET;
		rc = vx6953_i2c_write_u8(data->i2c, idx, val.ui8);
		if (rc) goto disable_vdig;

		/* must delay after sw reset */
		msleep(5);
	}
#endif

	// common initializations
	{
		// init external clock (used in sensor for edof calc)
		__u32 n = data->extclk.numerator;
		__u32 d = data->extclk.denominator;

		__u8 extclk_hi = n / d;
		__u8 extclk_lo = (__u8)((n << 8) / d);
		__u16 extclk = extclk_lo | extclk_hi << 8;

		idx = VX6953_CTRL_EXT_CLK;
		rc = vx6953_set_control(data, idx, extclk);
		if (rc) goto disable_vdig;
	}

	idx = VX6953_CTRL_VT_PIX_CLK_DIV;
	rc = vx6953_set_control(data, idx, data->cfg.vtpcd);
	if (rc) goto disable_vdig;

	idx = VX6953_CTRL_VT_SYS_CLK_DIV;
	rc = vx6953_set_control(data, idx, data->vtscd);
	if (rc) goto disable_vdig;

	idx = VX6953_CTRL_PRE_PLL_CLK_DIV;
	rc = vx6953_set_control(data, idx, data->ppllcd);
	if (rc) goto disable_vdig;

	idx = VX6953_CTRL_PLL_MULTIPLIER;
	rc = vx6953_set_control(data, idx, data->pllm);
	if (rc) goto disable_vdig;

	idx = VX6953_CTRL_OP_SYS_CLK_DIV;
	rc = vx6953_set_control(data, idx, data->op_sys_clk_div);
	if (rc) goto disable_vdig;

#ifdef CONFIG_VIDEO_VX6953_SETTING
	if ((rc = vx6953_apply_setting(data->i2c, &data->once)))
		goto disable_vdig;
#endif

	idx = VX6953_CTRL_CSI_SIGNALING_MODE;
	rc = vx6953_set_control(data, idx, data->csi_signaling_mode);
	if (rc) goto disable_vdig;

#ifdef LSC_ERROR_IMPROVEMENT_HACK
#define NVM_REG_SPACE_ACCESS 0x3640
	{
		u8 lsc_ver = 0;
		rc = vx6953_i2c_read_u8(data->i2c, MODULE_REV_MINOR, &lsc_ver);
		if (rc) goto disable_vdig;

		if ( (g_sensor_rev == SREV_CUT3) && (lsc_ver == 0x01) )
		{
			int i;
			u16 value;

			printk("vx6953-lsc offseting enabled\n");

			// Enable access NVM register space
			rc = vx6953_i2c_write_u8(data->i2c, NVM_REG_SPACE_ACCESS, 0);
			if (rc) goto disable_vdig;
			
			// adjustments 
			for (i=0; i<sizeof(lsc_nvm_reg_addr)/sizeof(u16); i++)
			{
				__u16 new_value, addr;
				
				// read original nvm data
				rc = vx6953_i2c_read_u16(data->i2c, lsc_nvm_reg_addr[i], &value);
				if (rc)
					goto disable_vdig;

				// d65 adjustment
				new_value = value + new_lsc_reg_offset_d65[i].offset;
				addr = new_lsc_reg_offset_d65[i].addr;
				if ((rc = vx6953_i2c_write_u16(data->i2c, addr, new_value)))
					goto disable_vdig;

#ifdef CONFIRM_LSC_WRITES
				confirm_write(data->i2c, addr, new_value);
#endif
				// cw adjustment
				new_value = value + new_lsc_reg_offset_cw[i].offset;
				addr = new_lsc_reg_offset_cw[i].addr;
				if ((rc = vx6953_i2c_write_u16(data->i2c, addr, new_value)))
					goto disable_vdig;

#ifdef CONFIRM_LSC_WRITES
				confirm_write(data->i2c, addr, new_value);
#endif
				// tl83 adjustment
				new_value = value + new_lsc_reg_offset_tl83[i].offset;
				addr = new_lsc_reg_offset_tl83[i].addr;
				if ((rc = vx6953_i2c_write_u16(data->i2c, addr, new_value)))
					goto disable_vdig;

#ifdef CONFIRM_LSC_WRITES
				confirm_write(data->i2c, addr, new_value);
#endif
				// horizon adjustment
				new_value = value + new_lsc_reg_offset_horizon[i].offset;
				addr = new_lsc_reg_offset_horizon[i].addr;
				if ((rc = vx6953_i2c_write_u16(data->i2c, addr, new_value)))
					goto disable_vdig;

#ifdef CONFIRM_LSC_WRITES
				confirm_write(data->i2c, addr, new_value);
#endif
			}

			{
				/* recommended by an ST application note */
				struct reg_write {
					__u16 addr;
					__u16 data;
				} norm_red_gain_cast[] = {
					{ 0xFBF8, 0x005E },
					{ 0xFBFA, 0x0056 },
					{ 0xFBFC, 0x003A },
					{ 0xFBFE, 0x0038 },
				};
				for (i = 0; i < sizeof(norm_red_gain_cast) /
						sizeof(struct reg_write);
						i++) {
					rc = vx6953_i2c_write_u16(data->i2c,
							norm_red_gain_cast[i].addr,
							norm_red_gain_cast[i].data);
					if (rc)
						goto disable_vdig;
				}
			}

			// Disable access to NVM register space
			rc = vx6953_i2c_write_u8(data->i2c, NVM_REG_SPACE_ACCESS, 1);
			if (rc) goto disable_vdig;
		}
	}
#endif
	idx = VX6953_CTRL_SHADING_CORRECTION;
	rc = vx6953_set_control(data, idx, 0x01);
	if (rc) goto disable_vdig;

#ifndef CONFIG_VIDEO_VX6953_SETTING
	// TODO: make defines for those addr
	// Free running clock
	rc = vx6953_i2c_write_u8(data->i2c, 0x3001, 0x30);
	if (rc) goto disable_vdig;

	// IQ DAC setup
	rc = vx6953_i2c_write_u8(data->i2c, 0x3004, 0x34);
	if (rc) goto disable_vdig;

	// IQ white clamp
	rc = vx6953_i2c_write_u8(data->i2c, 0x3007, 0x0b);
	if (rc) goto disable_vdig;

	// IQ DAC BUFFER
	rc = vx6953_i2c_write_u8(data->i2c, 0x301D, 0x03);
	if (rc) goto disable_vdig;

	// IQ av_r2_shift
	rc = vx6953_i2c_write_u8(data->i2c, 0x317E, 0x11);
	if (rc) goto disable_vdig;

	// IQ av_slant_shift
	rc = vx6953_i2c_write_u8(data->i2c, 0x317F, 0x09);
	if (rc) goto disable_vdig;

	// IQ dither_dgain
	rc = vx6953_i2c_write_u8(data->i2c, 0x3400, 0x38); 
	if (rc) goto disable_vdig;

#endif // CONFIG_VIDEO_VX6953_SETTING
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
vx6953_set_power_off(struct vx6953_client_data *data)
{
	int i;
	u16 idx;

	SPEW(1, "+++ %s\n", __func__);

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	printk("vx6953-poweroff\n");
#endif
	idx = VX6953_CTRL_MODE_SELECT;
	(void)vx6953_set_control(data, idx, MODE_SELECT_SOFTWARE_STANDBY);

	if (data->enable_vdig)
		(void)data->enable_vdig(0);

	for (i = 0; VX6953_CTRL_COUNT > i; ++i) {
		if (VX6953_CTRL_PIXEL_CLOCK != i)
			data->ctrls[i].flags &= ~CTRL_FLAG_CACHED;
	}

	data->power = 0;

	SPEW(1, "--- %s\n", __func__);
}

static int
vx6953_ioctl_s_power(struct v4l2_int_device *dev, int power)
{
	int rc = 0;
	struct vx6953_client_data *data;

	data = container_of(dev, struct vx6953_client_data, dev);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);

	if (power && !data->power)
		rc = vx6953_set_power_on(data, 1);

	else if (!power && data->power)
		vx6953_set_power_off(data);

	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
vx6953_ioctl_g_ifparm(struct v4l2_int_device *dev, struct v4l2_ifparm *parm)
{
	SPEW(1, "+++ %s\n", __func__);

	*parm = vx6953_ifparm;

	SPEW(1, "--- %s\n", __func__);

	return (0);
}

static struct v4l2_int_ioctl_desc vx6953_ioctls[] = {
	{
	.num = vidioc_int_g_fmt_cap_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_g_fmt,
	},
	{
	.num = vidioc_int_s_fmt_cap_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_s_fmt,
	},
	{
	.num = vidioc_int_try_fmt_cap_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_try_fmt,
	},
	{
	.num = vidioc_int_g_ctrl_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_g_ctrl,
	},
	{
	.num = vidioc_int_s_ctrl_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_s_ctrl,
	},
	{
	.num = vidioc_int_g_parm_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_g_parm,
	},
	{
	.num = vidioc_int_s_parm_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_s_parm,
	},
	{
	.num = vidioc_int_streamon_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_streamon,
	},
	{
	.num = vidioc_int_streamoff_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_streamoff,
	},
	{
	.num = vidioc_int_s_power_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_s_power,
	},
	{
	.num = vidioc_int_g_ifparm_num,
	.func = (v4l2_int_ioctl_func *)vx6953_ioctl_g_ifparm,
	},
};

static int identify_rev(struct vx6953_client_data *d, int rn)
{
	int ret = 0;
	char *rev_str;

	switch (rn) {
		case 0x01:
		case 0x02:
			rev_str = "2.0";
			g_sensor_rev = SREV_CUT2;
			break;
		case 0x81:
			rev_str = "3.0";
			g_sensor_rev = SREV_CUT3;
			break;
		default:
			rev_str = "unsupported";
			ret = -1;
			break;
	}
	printk(KERN_INFO VX6953_I2C_DRIVER ": found silicon id %u nvm v%u (cut-%s)\n",
			SILICON_ID(rn), NVM_VERSION(rn), rev_str);
	return ret;
}

static int
vx6953_i2c_probe(struct i2c_client *i2c)
{
	u8 rn;
	int rc;
	u16 id;
	struct v4l2_pix_format fmt;
	struct v4l2_captureparm parm;
	struct vx6953_client_data *data;
	struct vx6953_platform_data *pdata;

	SPEW(1, "%s\n", __func__);

	if (!(pdata = i2c->dev.platform_data)) {
		rc = -ENODEV;
		goto failed;
	}
	if (!(data = kzalloc(sizeof(struct vx6953_client_data), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto failed;
	}

	mutex_init(&data->lock);
	data->i2c = i2c;
	data->cit = pdata->coarse_integration_time;
	data->agcg = pdata->analogue_gain_code_global;
	data->extclk = pdata->extclk;
	data->vtscd = pdata->vt_sys_clk_div;
	data->ppllcd = pdata->pre_pll_clk_div;
	data->pllm = pdata->pll_multiplier;
	data->op_sys_clk_div = pdata->op_sys_clk_div;
	data->csi_signaling_mode = pdata->csi_signaling_mode;
	data->fourcc = pdata->fourcc;
	data->vdig = !pdata->enable_vdig;
	data->enable_vdig = pdata->enable_vdig;
	data->cci_data_fmt = vx6953_cci_data_formats[0].format;

	/* Initialize format and parameters */
	memset(&fmt, 0, sizeof(fmt));
	fmt.width = -1;
	fmt.height = -1;
	memset(&parm, 0, sizeof(parm));
	parm.timeperframe.denominator = 1;
	vx6953_format_to_config(&fmt, &data->cfg);
	update_pixel_clock(data);
	vx6953_parm_to_config(data, &parm, &data->cfg);

	memcpy(data->ctrls, vx6953_controls, sizeof(vx6953_controls));
	if ((rc = vx6953_set_power_on(data, 0))) {
		rc = -ENODEV;
		printk(KERN_ERR "%s cannot power on device\n", __func__);
		goto vx6953_set_power_on_failed;
	}

	if ((rc = vx6953_i2c_read_u16(i2c, MODEL_ID, &id))
		|| (rc = vx6953_i2c_read_u8(i2c, REVISION_NUMBER, &rn))) {
		rc = -ENODEV;
		printk(KERN_ERR "%s comm fail\n", __func__);
		goto comm_fail;
	}

	if  (MODEL_ID_VX6953 != id) {
		rc = -ENODEV;
		printk(KERN_ERR "model-id does not match: expected %d, got %d\n", 
				MODEL_ID_VX6953, id);
		goto unknown_model;
	}

	if ((rc = identify_rev(data, rn)))
		goto unknown_rev;

#ifdef CONFIG_VIDEO_VX6953_I2C_LOG
	printk("vx6953-description,time,addr-high,addr-low,data1,[data2]\n");
#endif
	vx6953_set_power_off(data);
	strncpy(data->slave.attach_to, V4L2_INT_SMIA10, V4L2NAMESIZE);
	data->slave.num_ioctls = ARRAY_SIZE(vx6953_ioctls);
	data->slave.ioctls = vx6953_ioctls;
	strncpy(data->dev.name, VX6953_I2C_DEVICE, V4L2NAMESIZE);
	data->dev.type = v4l2_int_type_slave;
	data->dev.u.slave = &data->slave;

	if ((rc = v4l2_int_device_register(&data->dev)))
		goto v4l2_int_device_register_failed;

	i2c_set_clientdata(i2c, data);

	if ((rc = device_create_file(&i2c->dev, &vx6953_attr_vdig)))
		goto device_create_vdig_file_failed;

	if ((rc = device_create_file(&i2c->dev, &vx6953_attr_cci_data)))
		goto device_create_cci_data_file_failed;

	if ((rc = device_create_file(&i2c->dev, &vx6953_attr_cci_data_format)))
		goto device_create_cci_data_format_file_failed;

	if ((rc = device_create_file(&i2c->dev, &vx6953_attr_cci_index)))
		goto device_create_cci_index_file_failed;

	if ((rc = vx6953_create_settings(&i2c->dev)))
		goto vx6953_create_settings_failed;

	if ((rc = vx6953_create_spew_level(&i2c->dev)))
		goto vx6953_create_spew_level_failed;

	if ((rc = vx6953_dbg_register_i2c_client(i2c)))
		goto vx6953_dbg_register_i2c_client_failed;

	return (0);

vx6953_dbg_register_i2c_client_failed:
	vx6953_remove_spew_level(&i2c->dev);
vx6953_create_spew_level_failed:
	vx6953_remove_settings(&i2c->dev);
vx6953_create_settings_failed:
	device_remove_file(&i2c->dev, &vx6953_attr_cci_index);
device_create_cci_index_file_failed:
	device_remove_file(&i2c->dev, &vx6953_attr_cci_data_format);
device_create_cci_data_format_file_failed:
	device_remove_file(&i2c->dev, &vx6953_attr_cci_data);
device_create_cci_data_file_failed:
	device_remove_file(&i2c->dev, &vx6953_attr_vdig);
device_create_vdig_file_failed:
	v4l2_int_device_unregister(&data->dev);
v4l2_int_device_register_failed:
unknown_rev:
unknown_model:
comm_fail:
	vx6953_set_power_off(data);
vx6953_set_power_on_failed:
	kfree(data);
failed:
	printk(KERN_ERR VX6953_I2C_DRIVER ": probe failed, rc=%d\n", rc);

	return (rc);
}

static int
vx6953_i2c_remove(struct i2c_client *i2c)
{
	struct vx6953_client_data *data;
	struct vx6953_platform_data *pdata;

	pdata = i2c->dev.platform_data;
	data = i2c_get_clientdata(i2c);

	vx6953_dbg_unregister_i2c_client(i2c);
	vx6953_remove_spew_level(&i2c->dev);
	vx6953_remove_settings(&i2c->dev);

#ifdef CONFIG_VIDEO_VX6953_SETTING
	if (data->once.fw)
		release_firmware(data->once.fw);
#endif // CONFIG_VIDEO_VX6953_SETTING
	device_remove_file(&i2c->dev, &vx6953_attr_cci_data);
	device_remove_file(&i2c->dev, &vx6953_attr_cci_data_format);
	device_remove_file(&i2c->dev, &vx6953_attr_cci_index);
	device_remove_file(&i2c->dev, &vx6953_attr_vdig);
	v4l2_int_device_unregister(&data->dev);
	(void)vx6953_ioctl_s_power(&data->dev, 0);
	kfree(data);

	return (0);
}

static struct i2c_driver vx6953_i2c_driver = {
	.driver = {
		.name = VX6953_I2C_DRIVER,
	},
	.probe = vx6953_i2c_probe,
	.remove = __devexit_p(vx6953_i2c_remove),
	/* NOTE: power is managed by v4l2 master */
};

static int __init
vx6953_module_init(void)
{
	return (i2c_add_driver(&vx6953_i2c_driver));
}

static void __exit
vx6953_module_exit(void)
{
	i2c_del_driver(&vx6953_i2c_driver);
}

module_init(vx6953_module_init);
module_exit(vx6953_module_exit);

MODULE_DESCRIPTION("ST Vx6953 sensor driver");
MODULE_LICENSE("GPL");
