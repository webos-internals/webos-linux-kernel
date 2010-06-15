/*
 * drivers/media/video/omap/sensor_mt9d112.c
 *
 * Copyright (C) 2007 Wind River.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * TI MT9D112 camera SOC driver. It implements the sensor interface defined
 * in sensor_if.h.
 *
 * This file is based off of the sensor_ex3691.c file written by TI.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/sched.h>
#include <linux/byteorder/generic.h>
#include <media/videobuf-core.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <linux/firmware.h>
#include <asm/arch/twl4030.h>

#include "sensor_if.h"
//#include "mt9d112.h"
#include "camera_register_ops.h"

//#define MT9D112_DEBUG

#ifndef CONFIG_TWL4030_CORE
#error "sensor_mt9d112 requires TWL4030 power companion!"
#endif

#define MOD_NAME "MT9D112:"

#ifdef MT9D112_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ERR MOD_NAME format "\n", ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#ifdef MT9D112_REG_DUMP
void dump_key_mt9d112_regs(void *priv);
#endif

#define ENABLE      1
#define DISABLE     0

/*
 * Board Specific Code
 *
 * This section contains all of the code that is affected by how the camera
 * module is connected on the board. It deals with things like GPIO's and
 * power sources. Currently the only supported board is Joplin.
 *
 */

#ifdef CONFIG_MACH_BRISKET
/* The MT9D112 I2C camera chip has a fixed slave address of 0x78 or 0x7A
   depending on SADDR is LOW or HIGH. It is 0x78 on Joplin. */
#define MT9D112_I2C_ADDR		(0x78 >> 1)

#define CAMERA_STANDBY_GPIO 133
#define CAMERA_RESET_GPIO  	132

#define TWL4030_VAUX4_DEV_GRP   0x23
#define TWL4030_VAUX4_DEDICATED 0x26
#define VAUX4_OFF               0x40
#define VAUX4_2_8_V             0x09
#define VAUX4_DEV_GRP_P1        0x20
#define VAUX4_DEV_GRP_NONE      0x00

static int
board_init(void)
{
	/* Request and configure gpio pins */
	if (omap_request_gpio(CAMERA_STANDBY_GPIO) != 0)
		return -EIO;
	/* set to output mode */
	omap_set_gpio_direction(CAMERA_STANDBY_GPIO, 0);
	if (omap_request_gpio(CAMERA_RESET_GPIO) != 0) {
		omap_free_gpio(CAMERA_STANDBY_GPIO);
		return -EIO;
	}
	/* set to output mode */
	omap_set_gpio_direction(CAMERA_RESET_GPIO, 0);

	return 0;
}

static int
board_power_enable(int enable)
{
    if(enable) {
        twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
            VAUX4_2_8_V, TWL4030_VAUX4_DEDICATED);
        twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
            VAUX4_DEV_GRP_P1,TWL4030_VAUX4_DEV_GRP);
    	mdelay(100);
    } else {
        twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
            VAUX4_OFF, TWL4030_VAUX4_DEDICATED);
        twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
            VAUX4_DEV_GRP_NONE,TWL4030_VAUX4_DEV_GRP);
    }

    return 0;
}

static void
board_cleanup(void)
{
    board_power_enable(DISABLE);

	omap_free_gpio(CAMERA_STANDBY_GPIO);
	omap_free_gpio(CAMERA_RESET_GPIO);
}

static int
board_standby_enable(int enable)
{
    omap_set_gpio_dataout(CAMERA_STANDBY_GPIO, enable);

    return 0;
}

static int
board_reset(void)
{
 	/* have to put sensor to reset to guarantee detection */
 	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 0);
	mdelay(100);

	/* nRESET is active LOW. set HIGH to release reset */
 	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1);
	/* give sensor some time to get out of the reset */
	udelay(500);

    return 0;
}
#endif  /* CONFIG_MACH_BRISKET */


/*
 * Camera Sensor Driver
 */

#define MT9D112_DEF_MCLK_MHZ	54
#define MT9D112_MAX_MCLK_MHZ	80
#define MT9D112_XCLK_HZ		48000000

#define MT9D112_MIN_FPS		10
#define MT9D112_DEF_FPS		15   /* At 40MHz mclk */
#define MT9D112_MAX_FPS		30   /* At 80MHz mclk */

#define QCIF_WIDTH		160
#define QCIF_HEIGHT		120
#define QVGA_WIDTH		320
#define QVGA_HEIGHT		240
#define VGA_WIDTH		640
#define VGA_HEIGHT		480
#define SVGA_WIDTH		800
#define SVGA_HEIGHT		600
#define UXGA_WIDTH		1600
#define UXGA_HEIGHT		1200

#define REG_SENSOR_VER  0x3000
#define MT9D112_ID      0x1580
#define REG_MCUADDR     0x338c
#define REG_MCUDATA     0x3390
#define MCU_STATE       (MCUA_LOGADDR|MCUA_DEVID(1)|MCUA_REG(0x4))

#define MIN_BRIGHTNESS 		0
#define MAX_BRIGHTNESS 		15
#define DEF_BRIGHTNESS 		8
#define BRIGHTNESS_STEP 		1
#define MIN_CONTRAST	 		0
#define MAX_CONTRAST	 		15
#define DEF_CONTRAST 			5
#define CONTRAST_STEP	 		1

/*
 * Supported Color/Pixel Formats
 */
#define MT9D112_PIX_FORMATS()   \
    MT9D112_PIX_FORMAT(MT9D112_PFMT_RGB565, "RGB565, le", V4L2_PIX_FMT_RGB565),\
    MT9D112_PIX_FORMAT(MT9D112_PFMT_YUYV, "YUYV", V4L2_PIX_FMT_YUYV), \
    MT9D112_PIX_FORMAT(MT9D112_PFMT_UYVY, "UYVY", V4L2_PIX_FMT_UYVY)

/* To manage the color formats first define a table with all of the format
 * information.
 */
#define MT9D112_PIX_FORMAT(_tag, _description, _v4l2_format)    \
    {.description = _description, .pixelformat = _v4l2_format}

const static struct v4l2_fmtdesc mt9d112_pixel_formats[] =
{
    MT9D112_PIX_FORMATS()
};

/* Then define a matching enumeration for accessing the format table. */
#undef MT9D112_PIX_FORMAT
#define MT9D112_PIX_FORMAT(_tag, _description, _v4l2_format)    \
    _tag

enum mt9d112_pixel_format
{
    MT9D112_PFMT_INVALID = -1,
    MT9D112_PIX_FORMATS(),
    MT9D112_PFMT_COUNT
};

static struct vcontrol {
        struct v4l2_queryctrl qc;
        int current_value;
} control[] = {
        { {V4L2_CID_BRIGHTNESS, V4L2_CTRL_TYPE_INTEGER, "Brightness",
           MIN_BRIGHTNESS, MAX_BRIGHTNESS, BRIGHTNESS_STEP, DEF_BRIGHTNESS},
          DEF_BRIGHTNESS},
        { {V4L2_CID_CONTRAST, V4L2_CTRL_TYPE_INTEGER, "Contrast",
           MIN_CONTRAST, MAX_CONTRAST, CONTRAST_STEP, DEF_CONTRAST},
          DEF_CONTRAST},
};
#define NUM_CONTROLS (sizeof(control)/sizeof(control[0]))

/* Define THE structure for tracking the sensor and driver state. */
static struct mt9d112_sensor {
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

    REGISTER_OPERATION_ARRAY *regopsarray;
} mt9d112;


/*
 * Default sensor register settings.
 */
const static REGISTER_OPERATION _regs_init[] =
{
    {ROP_SET, 0x3202, 0x0000},  // No Standby
    {ROP_SET, 0x301A, 0x00CC},  // Enable parallell output and start streaming
    {ROP_SET, 0x3386, 0x0001},  // Put MCU into reset
    {ROP_SET, 0x3386, 0x0000},  // Bring MCU out of reset
    {ROP_DELAY, 0, 100},

    MCU_VAR(7, 13, 0x0000),     // Row Start (A) = 0
    MCU_VAR(7, 15, 0x0000),     // Column Start (A) = 0
    MCU_VAR(7, 17, 0x04BD),     // Row End (A) = 1213
    MCU_VAR(7, 19, 0x064D),     // Column End (A) = 1613
    MCU_VAR(7, 21, 0x0071),     // Extra Delay (A) = 113
    MCU_VAR(7, 23, 0x2111),     // Row Speed (A) = 8465
    MCU_VAR(7, 27, 0x024F),     // sensor_sample_time_pck (A) = 591
    MCU_VAR(7, 29, 0x0102),     // sensor_fine_correction (A) = 258
    MCU_VAR(7, 31, 0x0279),     // sensor_fine_IT_min (A) = 633
    MCU_VAR(7, 33, 0x0155),     // sensor_fine_IT_max_margin (A) = 341
    MCU_VAR(7, 35, 0x0293),     // Frame Lines (A) = 659
    MCU_VAR(7, 37, 0x060F),     // Line Length (A) = 1551
    MCU_VAR(7, 39, 0x2020),     // sensor_dac_id_4_5 (A) = 8224
    MCU_VAR(7, 41, 0x2020),     // sensor_dac_id_6_7 (A) = 8224
    MCU_VAR(7, 43, 0x1020),     // sensor_dac_id_8_9 (A) = 4128
    MCU_VAR(7, 45, 0x2007),     // sensor_dac_id_10_11 (A) = 8199
    MCU_VAR(7, 47, 0x0004),     // Row Start (B) = 4
    MCU_VAR(7, 49, 0x0004),     // Column Start (B) = 4
    MCU_VAR(7, 51, 0x04BB),     // Row End (B) = 1211
    MCU_VAR(7, 53, 0x064B),     // Column End (B) = 1611
    MCU_VAR(7, 55, 0x044E),     // Extra Delay (B) = 1102
    MCU_VAR(7, 57, 0x2111),     // Row Speed (B) = 8465
    MCU_VAR(7, 61, 0x0120),     // sensor_sample_time_pck (B) = 288
    MCU_VAR(7, 69, 0x05BF),     // Frame Lines (B) = 1471
    MCU_VAR(7, 71, 0x0824),     // Line Length (B) = 2084
    MCU_VAR(7, 81, 0x0000),     // Crop_X0 (A) = 0
    MCU_VAR(7, 83, 0x0320),     // Crop_X1 (A) = 800
    MCU_VAR(7, 85, 0x0000),     // Crop_Y0 (A) = 0
    MCU_VAR(7, 87, 0x0258),     // Crop_Y1 (A) = 600
    MCU_VAR(7, 95, 0x0000),     // Crop_X0 (B) = 0
    MCU_VAR(7, 97, 0x0640),     // Crop_X1 (B) = 1600
    MCU_VAR(7, 99, 0x0000),     // Crop_Y0 (B) = 0
    MCU_VAR(7, 101, 0x04B0),    // Crop_Y1 (B) = 1200
    MCU_VAR(2, 46, 0x00A5),     // R9 Step = 165
    MCU_VAR8(4, 8, 0x20),       // search_f1_50 = 32
    MCU_VAR8(4, 9, 0x22),       // search_f2_50 = 34
    MCU_VAR8(4, 10, 0x26),      // search_f1_60 = 38
    MCU_VAR8(4, 11, 0x28),      // search_f2_60 = 40
    MCU_VAR(4, 17, 0x00A5),     // R9_Step_60_A = 165
    MCU_VAR(4, 19, 0x00C6),     // R9_Step_50_A = 198
    MCU_VAR(4, 21, 0x007B),     // R9_Step_60_B = 123
    MCU_VAR(4, 23, 0x0093),     // R9_Step_50_B = 147
    {ROP_EOL, 0, 0}
};
const static REGISTER_OPERATION *regs_init = _regs_init;

/* PLL Lists */
const static REGISTER_OPERATION _regs_pll_init[]= {
    {ROP_SET, 0x341E, 0x8F0B},  // Bypas PLL - use 48 MHz masterklock instead
    {ROP_SET, 0x3214, 0x0707},  // Use max slev-rate
	{ROP_EOL, 0, 0}
};
const static REGISTER_OPERATION *regs_pll_init = _regs_pll_init;

static REGISTER_OPERATION _regs_pll_bypass[]= {
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_pll_bypass = _regs_pll_bypass;

/* Sensor Sequencer Programs */
static REGISTER_OPERATION _regs_program_snapshot_run[] = {
    MCU_VAR8(1, 0x0020, 0x00),     // Dont capture video
    MCU_VAR8(1, 0x0003, 0x02),     // CAPTURE command
//    {ROP_WAIT, REG_MCUDATA, 0xffff},
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_program_snapshot_run = _regs_program_snapshot_run;

static REGISTER_OPERATION _regs_program_preview_run[] = {
    MCU_VAR8(1, 0x0020, 0x00),     // Dont capture video
    MCU_VAR8(1, 0x0003, 0x01),     // PREVIEW command
    {ROP_WAIT, REG_MCUDATA, 0xffff},
    {ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_program_preview_run = _regs_program_preview_run;

static REGISTER_OPERATION _regs_sync_changes_hold[] = {
    {ROP_OR, 0x301A, 0x8000},
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_sync_changes_hold = _regs_sync_changes_hold;

static REGISTER_OPERATION _regs_sync_changes_release[] = {
    {ROP_AND, 0x301A, 0x7FFF},
    MCU_VAR8(1, 0x0003, 0x05),       // Refresh Sequencer = 5
    {ROP_WAIT, REG_MCUDATA, 0xffff},
    MCU_VAR8(1, 0x0003, 0x06),       // Refresh Sequencer Mode = 6
    {ROP_WAIT, REG_MCUDATA, 0xffff},
    {ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_sync_changes_release = _regs_sync_changes_release;

/* Context A Settings */
static REGISTER_OPERATION _regs_ctx_a_init[]= {
    {ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_a_init = _regs_ctx_a_init;

static REGISTER_OPERATION _regs_ctx_a_width[] = {
    MCU_VAR(7, 3, 320),
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_a_width = _regs_ctx_a_width;

static REGISTER_OPERATION _regs_ctx_a_height[] = {
    MCU_VAR(7, 5, 320),
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_a_height = _regs_ctx_a_height;

/* Define indexes into the crop REG_OP array below. */
#define INDEX_CROP_X0           (1)
#define INDEX_CROP_X1           (3)
#define INDEX_CROP_Y0           (5)
#define INDEX_CROP_Y1           (7)

static REGISTER_OPERATION _regs_ctx_a_crop[] = {
    MCU_VAR(7, 0x51, 0x0),          // Crop_X0_A
    MCU_VAR(7, 0x53, 0x320),        // Crop_X1_A
    MCU_VAR(7, 0x55, 0x0),          // Crop_Y0_A
    MCU_VAR(7, 0x57, 0x258),        // Crop_Y1_A
    MCU_VAR8(1, 0x0003, 0x05),      // Refresh Sequencer = 5
    {ROP_WAIT, REG_MCUDATA, 0xffff},
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_a_crop = _regs_ctx_a_crop;

static REGISTER_OPERATION _regs_ctx_a_rgb565[] = {
    MCU_VAR(7, 0x0095, 0x0020),     // Enable RGB
    MCU_VAR(7, 0x0019, 0x046C),     // No flip MODE_SENSOR_READ_MODE_A
	{ROP_EOL, 0, 0}
};

static REGISTER_OPERATION _regs_ctx_a_yuv422[] = {
    MCU_VAR(7, 0x0095, 0x0002),     // Enable YUV, switch Y and Cr/Cb
    MCU_VAR(7, 0x0019, 0x046C),     // No flip MODE_SENSOR_READ_MODE_A
	{ROP_EOL, 0, 0}
};

/* Indexed array of reg lists to set color formats. */
static REGISTER_OPERATION *regs_ctx_a_formats[MT9D112_PFMT_COUNT] = {
    _regs_ctx_a_rgb565,
    _regs_ctx_a_yuv422,
    _regs_ctx_a_yuv422  /* UYVY - Same as YUYV here because byteswapped at
                                  the camera controller level. */
};


/* Context B Settings */
static REGISTER_OPERATION _regs_ctx_b_init[]= {
    {ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_b_init = _regs_ctx_b_init;

static REGISTER_OPERATION _regs_ctx_b_width[] = {
    MCU_VAR(7, 0x0007, 320),
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_b_width = _regs_ctx_b_width;

static REGISTER_OPERATION _regs_ctx_b_height[] = {
    MCU_VAR(7, 0x0009, 320),
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_b_height = _regs_ctx_b_height;

static REGISTER_OPERATION _regs_ctx_b_crop[] = {
    MCU_VAR(7, 0x5f, 0x0),          // Crop_X0_B
    MCU_VAR(7, 0x61, 0x640),        // Crop_X1_B
    MCU_VAR(7, 0x63, 0x0),          // Crop_Y0_B
    MCU_VAR(7, 0x65, 0x4B0),        // Crop_Y1_B
    MCU_VAR8(1, 0x0003, 0x05),      // Refresh Sequencer = 5
    {ROP_WAIT, REG_MCUDATA, 0xffff},
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_b_crop = _regs_ctx_b_crop;

static REGISTER_OPERATION _regs_ctx_b_burst_count[] = {
    MCU_VAR8(1, 0x0021, 16),
	{ROP_EOL, 0, 0}
};
static REGISTER_OPERATION *regs_ctx_b_burst_count = _regs_ctx_b_burst_count;

static REGISTER_OPERATION _regs_ctx_b_rgb565[] = {
    MCU_VAR(7, 0x0097, 0x0020),     // Enable RGB
    MCU_VAR(7, 0x003B, 0x0024),     // No flip MODE_SENSOR_READ_MODE_B
	{ROP_EOL, 0, 0}
};

static REGISTER_OPERATION _regs_ctx_b_yuv422[] = {
    MCU_VAR(7, 0x0097, 0x0002),     // Enable YUV, switch Y and Cr/Cb
    MCU_VAR(7, 0x003B, 0x0024),     // No flip MODE_SENSOR_READ_MODE_B
	{ROP_EOL, 0, 0}
};

/* Indexed array of reg lists to set color formats. */
static REGISTER_OPERATION *regs_ctx_b_formats[MT9D112_PFMT_COUNT] = {
    _regs_ctx_b_rgb565,
    _regs_ctx_b_yuv422,
    _regs_ctx_b_yuv422  /* UYVY - Same as YUYV here because byteswapped at
                                  the camera controller level. */
};


/*
 * Register Access Functions
 */

/******************************************************************************
 *
 * mt9d112_read_reg()
 *
 * Read a 16-bit value from the specified sensor register.
 *
 * Parameters
 *   client         Structure for I2C communications.
 *   reg            Sensor register to read.
 *   val            16-bit value read from the sensor register.
 *
 * Returns
 *   Zero if successful, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112_read_reg(struct i2c_client *client, u16 reg, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	u16 data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

read_again:

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (u8*) &data[0];
	data[0] = htons(reg);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8*) &data[1];

	err = i2c_transfer(client->adapter, msg, 2);

	if (err >= 0) {
		*val = ntohs(data[1]);
//        DPRINTK("mt9d112_read_reg: read 0x%04x from offset 0x%04x error %d", *val, reg, err);
		return 0;
	}

/* It is observed that the SOC fails to ACK the I2C bus when the master clock
 * is higher (e.g. max 80MHz). Retry seems to be working.
 */
#define READ_RETRY_COUNT		5
	if (retry <= READ_RETRY_COUNT) {
		DPRINTK("READ retry ... %d\n", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto read_again;
	}
	DPRINTK("read from offset 0x%x error %d", reg, err);
	return err;
}

/******************************************************************************
 *
 * mt9d112_write_reg()
 *
 * Write a 16-bit value to a register.
 *
 * Parameters
 *   client         Structure for I2C communications.
 *   reg            Sensor register to write.
 *   val            16-bit value to write to the sensor register.
 *
 * Returns
 *   Zero if successful, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112_write_reg(struct i2c_client *client, u16 reg, u16 val)
{
	int err;
	struct i2c_msg msg[1];
	u16 data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

//	DPRINTK("mt9d112_write_reg: writing 0x%04x to offset 0x%04x", val, reg);

write_again:
	data[0] = htons(reg);
	data[1] = htons(val);
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = (u8*) data;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

/* It is observed that the SOC fails to ACK the I2C bus when the master clock
 * is higher (e.g. max 80MHz). Retry seems to be working.
 */
#define WRITE_RETRY_COUNT		5
	if (retry <= WRITE_RETRY_COUNT) {
		DPRINTK("WRITE retry ... %d\n", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto write_again;
	}

    if (err < 0) {
        printk(KERN_ERR MOD_NAME ":mt9d112_write_reg - Failed i2c_transfer() err=0x%x.\n", err);
    }

	return err;
}

/******************************************************************************
 *
 * mt9d112_write_regs()
 *
 * Execute a list of register operations. The list is terminated by
 * MT9D112_TOK_TERM.
 *
 * Parameters
 *   client         Structure for I2C communications.
 *   regs           Register operations to execute.
 *
 * Returns
 *   Zero if successful, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112_write_regs(struct i2c_client *client, const REGISTER_OPERATION regs[])
{
	int err = 0;
	const REGISTER_OPERATION *next = regs;
    const REGISTER_OPERATION *prev = NULL;
    u16 value = 0;
    int retries = 0;

	for (; next->opcode != ROP_EOL; prev = next++) {
		if (next->opcode == ROP_DELAY) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout((next->value/2)/10);
			continue;
		}

        if (next->opcode == ROP_WAIT) {
		for (retries = 0; retries < 10; ++retries) {
			err = mt9d112_read_reg(client, next->reg, &value);
			if (err || !value) break;
			if (msleep_interruptible(100)) {
				DPRINTK("Sleep interrupted");
				err = -EINTR;
				break;
			}
		}

//            printk("mt9d112: err = %d, value = 0x%04x, retries = %d\n", err, value, retries);

            if (retries == 10)
                err = -EIO;

            if (err) {
                printk(KERN_INFO MOD_NAME ":mt9d112_write_regs - Failed ROP_WAIT. err=0x%x. v 0x%x, retries %d/%d\n",
                       err, value, retries, next->value);
                break;
            }
            continue;
        }

        if (next->opcode == ROP_AND || next->opcode == ROP_OR) {
            err = mt9d112_read_reg(client, next->reg, &value);
            if (err) {
                printk(KERN_ERR MOD_NAME ":mt9d112_write_regs - Failed to "
                       "read current value. err=0x%x\n", err);
			    break;
            }
        }

        switch(next->opcode) {
        case ROP_AND:
            value &= next->value;
            break;
        case ROP_OR:
            value |= next->value;
            break;
        default:
            value = next->value;
            break;
        }

	err = mt9d112_write_reg(client, next->reg, value);
        if (err) {
            printk(KERN_ERR MOD_NAME ":mt9d112_write_regs - Failed to "
                   "write value. reg=0x%04x, val=0x%04x, err=0x%x\n",
                   next->reg, value, err);
			break;
        }
	}

    return err;
}

#ifdef MT9D112_DEBUG
/* Just for debugging. */
static void
mt9d112_regop_dump(const REGISTER_OPERATION* regop) {
    char *rops[] = {"ROP_EOL", "ROP_DELAY", "ROP_SET", "ROP_AND", "ROP_OR", "ROP_WAIT"};

    printk("mt9d112: REGOP(%s, 0x%04x, 0x%04x)\n", rops[regop->opcode], regop->reg, regop->value);
}

static int
mt9d112_confirm_regs(struct i2c_client *client, const REGISTER_OPERATION regs[])
{
	int err = 0;
	const REGISTER_OPERATION *next = regs;
    u16 value = 0;
    u32 mcuaddr = 0;

	for (; next->opcode != ROP_EOL; next++) {
//        mt9d112_regop_dump(next);
        if (next->opcode != ROP_SET) {
            mcuaddr = 0;
			continue;
        }

        /* If accessing MCU variable then go ahead and write the address. */
        if (next->reg == REG_MCUADDR) {
            err = mt9d112_write_reg(client, next->reg, next->value);
            if (err) {
                printk("mt9d112: Failed to write MCUADDR 0x%04x.\n", next->value);
                mcuaddr = 0;
                continue;
            }
            mcuaddr = next->value;
            continue;
        }

        err = mt9d112_read_reg(client, next->reg, &value);
        if (err) {
            printk("mt9d112: Failed to read reg 0x%04x.\n", next->reg);
            mcuaddr = 0;
            continue;
        }

        if (value != next->value) {
            printk("mt9d112: reg=0x%04x,mcuaddr=0x%04x expected 0x%04x got 0x%04x.\n", next->reg, mcuaddr, next->value, value);
            mcuaddr = 0;
        }
    }

    return 0;
}
#endif


/*
 * I2C Functions
 */

/******************************************************************************
 *
 * mt9d112_detect()
 *
 * Detect if an MT9D112 is present, and if so which revision.
 *
 * Parameters
 *   client         Structure for I2C communications.
 *
 * Returns
 *   Returns a negative error number if no device is detected, or the
 *   non-negative value of the version ID register if a device is detected.
 *
 *****************************************************************************/
static int
mt9d112_detect(struct i2c_client *client)
{
	u16 ver;
	if (!client)
		return -ENODEV;

	if (mt9d112_read_reg(client, REG_SENSOR_VER, &ver))
		return -ENODEV;

    printk("Probing I2C: ver=0x%04x.\n", ver);

    if (ver != MT9D112_ID) {
		/* We didn't read the values we expected, so
		 * this must not be an MT9D112.
		 */
		return -ENODEV;
	}

	return ver;
}

/******************************************************************************
 *
 * mt9d112_i2c_attach_client()
 *
 * This function registers an I2C client via i2c_attach_client() for the
 * sensor device.  If 'probe' is non-zero, then the I2C client is only
 * registered if the device can be detected.  If 'probe' is zero, then no
 * device detection is attempted and the I2C client is always registered.
 *
 * Parameters
 *   adap           Structure for I2C communications.
 *   addr           I2C address of client.
 *   probe          Set to 1 to request device be verified as present.
 *
 * Returns
 *   Returns zero if an I2C client is successfully registered, or non-zero
 *   otherwise.
 *
 *****************************************************************************/
static int
mt9d112_i2c_attach_client(struct i2c_adapter *adap, int addr, int probe)
{
	struct mt9d112_sensor *sensor = &mt9d112;
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
        printk(KERN_ERR MOD_NAME ":mt9d112_i2c_attach_client - Failed "
               "i2c_attach_client(). err=0x%x\n", err);
		client->adapter = NULL;
		return err;
	}

	if (probe) {
		err = mt9d112_detect(client);
		if (err < 0) {
			i2c_detach_client(client);
			client->adapter = NULL;
			return err;
		}
		sensor->ver = err;
	}
	return 0;
}

/******************************************************************************
 *
 * mt9d112_i2c_detach_client()
 *
 * This function is called by i2c_del_adapter() and i2c_del_driver()
 * if the adapter or driver with which this I2C client is associated is
 * removed.  This function unregisters the client via i2c_detach_client().
 *
 * Parameters
 *   client         Structure for I2C communications.
 *
 * Returns
 *   Returns zero if the client is successfully detached, or non-zero
 *   otherwise.
 *
 *****************************************************************************/
static int
mt9d112_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;

	return err;
}

/******************************************************************************
 *
 * mt9d112_i2c_probe_adapter()
 *
 * This function will be called for each registered I2C bus adapter when our
 * I2C driver is registered via i2c_add_driver().  It will also be called
 * whenever a new I2C adapter is registered after our I2C driver is registered.
 * This function probes the specified I2C bus adapter to determine if an
 * MT9D112 sensor device is present.  If a device is detected, an I2C client
 * is registered for it via mt9d112_i2c_attach_client().  Note that we can't use
 * the standard i2c_probe() function to look for the sensor because the OMAP
 * I2C controller doesn't support probing.
 *
 * Parameters
 *   adap           Structure for I2C communications.
 *
 * Returns
 *   Returns zero if an MT9D112 device is detected and an I2C client successfully
 *   registered for it, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return mt9d112_i2c_attach_client(adap, MT9D112_I2C_ADDR, 1);
}


/*
 * V4L2 API Helper Functions
 */

/******************************************************************************
 *
 * get_pixel_format()
 *
 * Convert V4L2 pixel format to MT9D112 pixel format.
 *
 * Parameters
 *   pix            V4L2 pixel format.
 *   swapbyte       Whether or not bytes are swapped.
 *
 * Returns
 *   Returns MT9D112 pixel format or MT9D112_PFMT_INVALID for unsupported
 *   formats.
 *
 *****************************************************************************/
static enum mt9d112_pixel_format
get_pixel_format(struct v4l2_pix_format *pix, int *swapbyte)
{
  	enum mt9d112_pixel_format pfmt = MT9D112_PFMT_INVALID;

    if(swapbyte != NULL)
	    *swapbyte = 0;

	switch (pix->pixelformat) {
		case V4L2_PIX_FMT_RGB565:
			pfmt = MT9D112_PFMT_RGB565;
			break;
		case V4L2_PIX_FMT_YUYV:
			pfmt = MT9D112_PFMT_YUYV;
            break;
		case V4L2_PIX_FMT_UYVY:
			pfmt = MT9D112_PFMT_UYVY;
            break;
		default:
            break;
 	}

	return pfmt;
}

/******************************************************************************
 *
 * get_bpp()
 *
 * Convert the MT9D112 pixel format into bits per pixel. (All formats are
 * currently 16 bits.)
 *
 * Parameters
 *   pfmt           MT9D112 pixel format.
 *
 * Returns
 *   Returns bits per pixel.
 *
 *****************************************************************************/
inline static u32
get_bpp(enum mt9d112_pixel_format pfmt)
{
    return 16;
}

/* Given a v4l2_pix_format with only the width, height, and pixelformat
 * specified fill in the remainder of the structure.
 */
static int
fill_format_info(struct v4l2_pix_format *pix)
{
  	enum mt9d112_pixel_format pfmt = get_pixel_format(pix, NULL);

    /* If could not support requested format then fill in preferred format. */
	if (pfmt == MT9D112_PFMT_INVALID)
        pfmt = MT9D112_PFMT_RGB565;

	pix->pixelformat = mt9d112_pixel_formats[pfmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * (get_bpp(pfmt) >> 3);
	pix->sizeimage = pix->bytesperline * pix->height;
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


/*
 * Driver Internal Functions
 */

/* Returns the index of the requested ID from the control structure array */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = NUM_CONTROLS - 1; i >= 0; i--)
		if (control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/* Set the camera brightness level. Return 0 on success. */
static int
mt9d112_set_brightness(struct i2c_client *client, int level)
{
#if 0
    /* the caller passes us a valid level */
	DPRINTK("new brightness level %d", level);
	/* brightness control is context-less */
	default_brightness_list[INDEX_BRIGHTNESS].val = level*4+10;
	if (mt9d112_write_regs(client, default_brightness_list))
		return -EIO;

	/* We force a 50ms brightness setting interval. */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(50));
#endif

	return 0;
}

/* Set the camera contrast level. Return 0 on success. */
static int
mt9d112_set_contrast(struct i2c_client *client, int level)
{
#if 0
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
	if (mt9d112_write_regs(client, a_default_contrast_list))
		return -EIO;

	/* program the values to the context B */
	for (i = 0; i < GAM_TABLE_LENGTH; i++)
		a_default_contrast_list[2*i].val =  GAM_TABLE_START_B + i;
	if (mt9d112_write_regs(client, a_default_contrast_list))
		return -EIO;

	/* We force a 50ms contrast setting interval. */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(50));
#endif

    return 0;
}

/******************************************************************************
 *
 * configure_master_clock()
 *
 * Configure the PLL, which is always running at a fixed rate in this
 * implementation.
 *
 * Parameters
 *   sensor         The sensor to operate on.
 *   mclk           The speed to set the master clock to in MHz.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
configure_master_clock(struct mt9d112_sensor *sensor, int mclk)
{
	int err;

	DPRINTK("%d MHz master clock", mclk);

    sensor->mclk = mclk;

    err = mt9d112_write_regs(&sensor->client, regs_pll_init);

	return err;
}

/******************************************************************************
 *
 * process_ini_data()
 *
 * Process the binary .ini file data sent to the driver from the caminiloader
 * user-space application. This routine copies the register operations arrays
 * and points into the array for each type of operation.
 *
 * Parameters
 *   sensor         The sensor to operate on.
 *   regopsarray    An array of REGISTER_OPERATION_ARRAYs.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
// process_ini_data() call has been commented out.
// So let's remove the function definition to appease compiler. 
#if 0
static int
process_ini_data(struct mt9d112_sensor *sensor,
                 REGISTER_OPERATION_ARRAY *regopsarray, size_t size)
{
    u32 count = 0;

    if(sensor == NULL || regopsarray == NULL || size == 0)
        return -EINVAL;

    sensor->regopsarray = kmalloc(size, GFP_KERNEL);
    if(sensor->regopsarray == NULL)
        return -ENOMEM;

    memcpy(sensor->regopsarray, regopsarray, size);

    /* Now start at the beginning of the local copy. */
    regopsarray = sensor->regopsarray;

    /* Loop through entire array of arrays pointing local operations into
    the list to override the default behavior. */
    while(size > 0)
    {
        count = regopsarray->count;

        DPRINTK("size = %d, regopsarray = %p\n", size, regopsarray);
        DPRINTK("type=%d, count=%d\n", regopsarray->type, regopsarray->count);

#ifdef MT9D112_DEBUG
#if 0
        /* Dumps the current array for debugging. */
        {
        u32 i;
        for(i = 0; i < regopsarray->count; i++)
        {
            mt9d112_regop_dump(&(regopsarray->rops[i]));
        }
        }
#endif
#endif

        /* Point to overridden ROPS. */
        switch(regopsarray->type)
        {
        case ROA_INIT:
            regs_init = regopsarray->rops;
            break;
        case ROA_PLL_INIT:
            regs_pll_init = regopsarray->rops;
            break;
        case ROA_PROGRAM_SNAPSHOT:
            regs_program_snapshot_run = regopsarray->rops;
            break;
        case ROA_PROGRAM_PREVIEW:
            regs_program_preview_run = regopsarray->rops;
            break;
        case ROA_SYNC_HOLD:
            regs_sync_changes_hold = regopsarray->rops;
            break;
        case ROA_SYNC_RELEASE:
            regs_sync_changes_release = regopsarray->rops;
            break;
        case ROA_CTXA_INIT:
            regs_ctx_a_init = regopsarray->rops;
            break;
        case ROA_CTXA_WIDTH:
            regs_ctx_a_width = regopsarray->rops;
            break;
        case ROA_CTXA_HEIGHT:
            regs_ctx_a_height = regopsarray->rops;
            break;
        case ROA_CTXA_RGB:
            regs_ctx_a_formats[0] = regopsarray->rops;
            break;
        case ROA_CTXA_YUV:
            regs_ctx_a_formats[1] = regopsarray->rops;
            break;
        case ROA_CTXB_INIT:
            regs_ctx_b_init = regopsarray->rops;
            break;
        case ROA_CTXB_WIDTH:
            regs_ctx_b_width = regopsarray->rops;
            break;
        case ROA_CTXB_HEIGHT:
            regs_ctx_b_height = regopsarray->rops;
            break;
        case ROA_CTXB_RGB:
            regs_ctx_b_formats[0] = regopsarray->rops;
            break;
        case ROA_CTXB_YUV:
            regs_ctx_b_formats[1] = regopsarray->rops;
            break;
        default:
            printk(KERN_WARNING MOD_NAME "process_ini_data: Unsupported "
                   "REGISTER OPERATION type.\n");
            break;
        }

        /* Check for end and advance to next array. */
        size -= (sizeof(REGISTER_OPERATION_ARRAY) +
            sizeof(REGISTER_OPERATION) * regopsarray->count);

        regopsarray = (REGISTER_OPERATION_ARRAY *) &(regopsarray->rops[count]);
    }

    return 0;
}
#endif

/*
 * Sensor Interface
 *
 * Following are sensor interface functions implemented by
 * MT9D112 sensor driver to allow it to work with the general camera
 * driver.
 *
 */

/******************************************************************************
 *
 * mt9d112sensor_power_on()
 *
 * This function is called as part of power management. It "resumes" camera
 * operations.
 *
 * Parameters
 *   priv           Not used.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_power_on(void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
	int err;
    static int initial_power_on = 1;

    DPRINTK("power_on()\n");

//#define SUSPEND_USE_STANDBY
#ifdef SUSPEND_USE_STANDBY
    if(initial_power_on) {
#else
    {
#endif
        err = board_power_enable(ENABLE);
        if(err) {
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_power_on - Failed "
                   "to enable power to camera.\n");
            return err;
        }
    }

    /* Release STANDBY line. */
	err = board_standby_enable(DISABLE);
    if(err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_power_on - Failed "
               "to release standby line.\n");
        return err;
    }
    mdelay(10);

#ifdef SUSPEND_USE_STANDBY
    if(initial_power_on) {
#else
    {
#endif
        /* Reset the camera. */
        err = board_reset();
        if(err) {
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_power_on - Failed "
                   "to reset camera.\n");
            return err;
        }
    }
    
	if (initial_power_on) {
		initial_power_on = 0;
		return (0);
	}

#ifndef SUSPEND_USE_STANDBY
    /*
     * Reconfigure camera with current settings.
     */
	DPRINTK("Initial command sequence ...");
	if ((err = mt9d112_write_regs(&sensor->client, regs_init))) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_init - "
               "Failed to write initial register settings.\n");
		return err;
	}

	DPRINTK("Configure PLL ...");
	if ((err = configure_master_clock(sensor, MT9D112_DEF_MCLK_MHZ))) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_init - "
               "Failed configure_master_clock().\n");
		return err;
	}

    /* Set up the preview mode settings. */
    DPRINTK("Setting preview mode settings.\n");
	err = mt9d112_write_regs(&sensor->client, regs_ctx_a_init);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_init - "
               "Failed to write initial ctx a settings.\n");
		return err;
    }

	/* Set the default brightness and contrast */
	mt9d112_set_brightness(&sensor->client, DEF_BRIGHTNESS);
	mt9d112_set_contrast(&sensor->client, DEF_CONTRAST);

    /* Put the kibosh on double-buffered register updates. */
    mt9d112_write_regs(&sensor->client, regs_sync_changes_hold);

	err = mt9d112_write_regs(&sensor->client, regs_ctx_a_formats[sensor->pfmt_a]);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set pfmt.\n");
		return err;
    }

	err = mt9d112_write_regs(&sensor->client, regs_ctx_a_width);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set width.\n");
		return err;
    }
	err = mt9d112_write_regs(&sensor->client, regs_ctx_a_height);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set height.\n");
		return err;
    }

	err = mt9d112_write_regs(&sensor->client, regs_ctx_b_formats[sensor->pfmt_b]);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set pfmt.\n");
		return err;
    }

	err = mt9d112_write_regs(&sensor->client, regs_ctx_b_width);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set width.\n");
		return err;
    }
	err = mt9d112_write_regs(&sensor->client, regs_ctx_b_height);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set height.\n");
		return err;
    }

    /* Release double-buffered register updates. */
    mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
#endif

	return err;
}

/******************************************************************************
 *
 * mt9d112sensor_power_off()
 *
 * This function is called as part of power management. It "suspends" camera
 * operations.
 *
 * Parameters
 *   priv           Not used.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_power_off(void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
	int err;

    DPRINTK("power_off().\n");

    /* Start preview mode (the default). */
	err = mt9d112_write_regs(&sensor->client, regs_program_preview_run);

    /* Bypass PLL. */
	err = mt9d112_write_regs(&sensor->client, regs_pll_bypass);
	if (err)
		return err;

	/* Assert STANDBY line. */
	board_standby_enable(ENABLE);
    mdelay(10);

#ifndef SUSPEND_USE_STANDBY
    /* Standby? We don't need no stinkin' standby! */
    board_power_enable(DISABLE);
#endif

	/* the caller can cut off XCLK now */
	return 0;
}


/******************************************************************************
 *
 * mt9d112sensor_cropcap()
 *
 * API to get the supported cropping sizes.
 *
 * Parameters
 *   cropcap        Structure to return values in.
 *   priv           Not used.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_cropcap(struct v4l2_cropcap *cropcap, void *priv)
{
	cropcap->bounds.left = cropcap->bounds.top = 0;
    if (cropcap->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
	    cropcap->bounds.width = SVGA_WIDTH;
	    cropcap->bounds.height = SVGA_HEIGHT;
    }
    else {
	    cropcap->bounds.width = UXGA_WIDTH;
	    cropcap->bounds.height = UXGA_HEIGHT;
    }
	cropcap->defrect = cropcap->bounds;
	cropcap->pixelaspect.numerator = 1;
	cropcap->pixelaspect.denominator = 1;

	return 0;
}

/******************************************************************************
 *
 * mt9d112sensor_get_crop()
 *
 * API to get the current crop area.
 *
 * Parameters
 *   crop           Structure to return values in.
 *   priv           Not used.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_get_crop(struct  v4l2_crop *crop, void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;

	if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		crop->c = sensor->crop_rect_a;
	else if (crop->type == V4L2_BUF_TYPE_STILL_CAPTURE)
		crop->c = sensor->crop_rect_b;
	else
		return -EINVAL;

	return 0;
}

/******************************************************************************
 *
 * mt9d112sensor_set_crop()
 *
 * API to set the current crop area.
 *
 * Note: The hardware has a limitation that the crop area cannot be set smaller
 *       than the output size (width/height). This routine will prevent this.
 *       It was decided to adjust the crop area to the current size as opposed
 *       to adjusting the current size to the crop area, because there may be
 *       user buffers involved. Thus, it is necessary to set the image size
 *       to be smaller than the desired crop area before setting the crop area.
 *
 * Parameters
 *   cropcap        Crop area to set.
 *   priv           Not used.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_set_crop(struct  v4l2_crop *crop, void *priv)
{
    struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
	struct v4l2_rect *cur_rect;
	unsigned long *cur_width, *cur_height;
	REGISTER_OPERATION *crop_list;
	int err;

	switch (crop->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
			cur_rect = &sensor->crop_rect_a;
			cur_width = &sensor->width_a;
			cur_height = &sensor->height_a;
			crop_list = regs_ctx_a_crop;
	        /* out of range? then return the current crop rectangle */
	        if ((crop->c.left + crop->c.width) > SVGA_WIDTH ||
	            (crop->c.top + crop->c.height) > SVGA_HEIGHT) {
		        crop->c = *cur_rect;
		        return 0;
	        }
			break;
		case V4L2_BUF_TYPE_STILL_CAPTURE:
			cur_rect = &sensor->crop_rect_b;
			cur_width = &sensor->width_b;
			cur_height = &sensor->height_b;
			crop_list = regs_ctx_b_crop;
	        /* out of range? then return the current crop rectangle */
	        if ((crop->c.left + crop->c.width) > UXGA_WIDTH ||
	            (crop->c.top + crop->c.height) > UXGA_HEIGHT) {
		        crop->c = *cur_rect;
		        return 0;
	        }
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

	/* The MT9D112 SOC resizer can only scale down so we need to
	 * make sure that the scale factor is no bigger than 1.
	 */
    if (crop->c.width < *cur_width) {
		printk(KERN_WARNING MOD_NAME "Cannot set crop smaller than current "
            "image width. Set smaller size first.\n");
		crop->c.width = *cur_width;
    }
    if (crop->c.height < *cur_height) {
		printk(KERN_WARNING MOD_NAME "Cannot set crop smaller than current "
            "image height. Set smaller size first.\n");
		crop->c.height = *cur_height;
    }

    /* adjust the parameters accordingly */
	crop_list[INDEX_CROP_X0].value = crop->c.left;
	crop_list[INDEX_CROP_X1].value = crop->c.left + crop->c.width;
	crop_list[INDEX_CROP_Y0].value = crop->c.top;
	crop_list[INDEX_CROP_Y1].value = crop->c.top + crop->c.height;

	/* configure the SOC resizer */
	DPRINTK("Crop command sequence for %s...",
	    (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)?"A":"B");
	DPRINTK("old (%d, %d) (%d, %d), new (%d, %d) (%d, %d)",
		cur_rect->left, cur_rect->top, cur_rect->width, cur_rect->height,
		crop->c.left, crop->c.top, crop->c.width, crop->c.height);

    /* Put the kibosh on double-buffered register updates. */
    err = mt9d112_write_regs(&sensor->client, regs_sync_changes_hold);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed regs_sync_changes_hold.\n");
        return err;
    }

    err = mt9d112_write_regs(&sensor->client, crop_list);
    /* Release double-buffered register updates. */
    mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
	if (err)
		return err;


	/* save back */
	*cur_rect = crop->c;

#if 0
    /* Setting crop too fast can cause frame out-of-sync.
	 * We force a crop setting interval of a 1.5-Context_A frames.
	 */
	fps = (sensor->mclk * MT9D112_MAX_FPS) / MT9D112_MAX_MCLK_MHZ;
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(3000/(fps*2)));
#endif

	return 0;
}


/******************************************************************************
 *
 * mt9d112sensor_enter_still_capture()
 *
 * This function is called to start the still capture process.
 *
 * Parameters
 *   frames         Number of burst mode frames to capture.
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_enter_still_capture(int frames, void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
	int err;
    u16 value = 0;
    int retries;

	DPRINTK("Context B initialization...");

    /* Put the kibosh on double-buffered register updates. */
    err = mt9d112_write_regs(&sensor->client, regs_sync_changes_hold);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed regs_sync_changes_hold.\n");
        return err;
    }

    /* Set up the core registers. (Must be done here because some registers
     * are common to both contexts. Only want to change these when requested
     * to do so.)
     */
	err = mt9d112_write_regs(&sensor->client, regs_ctx_b_init);
    if (err) {
        /* Release double-buffered register updates. */
        mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_enter_still_capture - Failed "
               "to write context b init.\n");
		return err;
    }

    /* Set the number of frames to capture. */
    regs_ctx_b_burst_count[1].value = frames;
	err = mt9d112_write_regs(&sensor->client, regs_ctx_b_burst_count);
    if (err) {
        /* Release double-buffered register updates. */
        mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_enter_still_capture - Failed "
               "to set number of burst frames.\n");
		return err;
    }

    /* Release double-buffered register updates. */
    mt9d112_write_regs(&sensor->client, regs_sync_changes_release);

    /* Start the still capture. */
	err = mt9d112_write_regs(&sensor->client, regs_program_snapshot_run);
    if (err) {
        DPRINTK(":mt9d112sensor_enter_still_capture - Failed "
               "to run snapshot program.\n");
		return err;
    }

    /* Make sure still capture is started. */
    err = mt9d112_write_reg(&sensor->client, REG_MCUADDR, MCU_STATE);

	for (retries = 0; retries < 20; ++retries) {
		err = mt9d112_read_reg(&sensor->client, REG_MCUDATA, &value);
		if (err || 4 < ntohs(value)) break;
		if (msleep_interruptible(100)) {
			err = -EINTR;
			break;
		}
	}
	
//    printk("enter_still_capture - retries = %d, state = 0x%x\n", retries, value);

    if(retries == 20) {
        printk(":mt9d112sensor_enter_still_capture - timed out with "
            "state = 0x%x\n", value);
        err = -EIO;
    }

	return err;
}

/******************************************************************************
 *
 * mt9d112sensor_exit_still_capture()
 *
 * This function is called to abort the still capture process and return to
 * video preview mode.
 *
 * Parameters
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_exit_still_capture(void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
	int err;

	DPRINTK("Context A command sequence ...");
	err = mt9d112_write_regs(&sensor->client, regs_ctx_a_init);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_exit_still_capture - Failed "
               "to write context a init.\n");
		return err;
    }

    /* Start the preview mode. */
    err = mt9d112_write_regs(&sensor->client, regs_program_preview_run);
	if (err)
            return err;

	return err;
}

static int
mt9d112sensor_query_control(struct v4l2_queryctrl *qc, void *priv)
{
	int i;

	i = find_vctrl (qc->id);
	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	if (i < 0)
		return -EINVAL;

	*qc = control[i].qc;
	return 0;
}

static int
mt9d112sensor_get_control(struct v4l2_control *vc, void *priv)
{
	struct vcontrol *lvc;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	switch (lvc->qc.id) {
		case V4L2_CID_BRIGHTNESS:
		case V4L2_CID_CONTRAST:
			vc->value = lvc->current_value;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int
mt9d112sensor_set_control(struct v4l2_control *vc, void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	struct vcontrol *lvc;
	int val = vc->value;
	int i, err;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	if (val < lvc->qc.minimum || val > lvc->qc.maximum)
		return -ERANGE;

	switch (lvc->qc.id) {
		case V4L2_CID_BRIGHTNESS:
			err = mt9d112_set_brightness(client, val);
			if (err)
				return err;
			lvc->current_value = val;
			break;
		case V4L2_CID_CONTRAST:
			err = mt9d112_set_contrast(client, val);
			if (err)
				return err;
			lvc->current_value = val;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

/******************************************************************************
 *
 * mt9d112sensor_enum_pixformat()
 *
 * This function implements VIDIOC_ENUM_FMT for the CAPTURE buffer type.
 *
 * Parameters
 *   fmt            The structure to fill with the next supported format.
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_enum_pixformat(struct v4l2_fmtdesc *fmt, void *priv)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
		case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= MT9D112_PFMT_COUNT)
			return -EINVAL;
		break;

		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		if (index >= MT9D112_PFMT_COUNT)
			return -EINVAL;
		break;

		default:
			return -EINVAL;
	}

	fmt->flags = mt9d112_pixel_formats[index].flags;
	strncpy(fmt->description, mt9d112_pixel_formats[index].description, sizeof(fmt->description));
	fmt->pixelformat = mt9d112_pixel_formats[index].pixelformat;

	return 0;
}

/******************************************************************************
 *
 * mt9d112sensor_try_format()
 *
 * Implements the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 * We have to make sure that the agreed image isn't bigger than the crop.
 * This is due to the fact that SOC IFP can only scale down.
 *
 *
 * Parameters
 *   pix            The prefilled structure to fill with the negotiated format.
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_try_format(struct v4l2_pix_format *pix, void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;

    /* Make sure that the agreed image isn't bigger than the crop. */
	if (pix->width > sensor->crop_rect_a.width)
		pix->width = sensor->crop_rect_a.width;
	if (pix->height > sensor->crop_rect_a.height)
		pix->height = sensor->crop_rect_a.height;

	/* make sure the line is a mutiple of 32-bit word */
	pix->width &= ~1;

	return fill_format_info(pix);
}
/* Same as above for the still capture context. */
static int
mt9d112sensor_try_format_still_capture(struct v4l2_pix_format *pix, void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;

    /* Make sure that the agreed image isn't bigger than the crop. */
	if (pix->width > sensor->crop_rect_b.width)
		pix->width = sensor->crop_rect_b.width;
	if (pix->height > sensor->crop_rect_b.height)
		pix->height = sensor->crop_rect_b.height;

	/* make sure the line is a mutiple of 32-bit word */
	pix->width &= ~1;

	return fill_format_info(pix);
}

/******************************************************************************
 *
 * mt9d112sensor_calc_xclk()
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency
 * This sensor driver is designed to utilize on-chip PLL so the input
 * clock from the host is fixed at 48MHZ.
 *
 * Parameters
 *   pix            Information about the desired pixel format.
 *   timeperframe   The structure to fill with timing information.
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns the xclk frequency in Hz.
 *
 *****************************************************************************/
static unsigned long
mt9d112sensor_calc_xclk(struct v4l2_pix_format *pix,
			struct v4l2_fract *timeperframe, void *priv)
{
	int fps;

	if ((timeperframe->numerator == 0)
		|| (timeperframe->denominator == 0)) {
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = MT9D112_DEF_FPS;
	}
	fps = timeperframe->denominator/timeperframe->numerator;
	if (fps < MT9D112_MIN_FPS)
		fps = MT9D112_MIN_FPS;
	else if (fps > MT9D112_MAX_FPS)
			fps = MT9D112_MAX_FPS;
	timeperframe->numerator = 1;
	timeperframe->denominator = fps;

	return MT9D112_XCLK_HZ;
}

/******************************************************************************
 *
 * mt9d112sensor_configure()
 *
 * Given a capture format in pix, the frame period in timeperframe, and
 * the xclk frequency, configure the MT9D112 context A for a specified
 * image size, pixel format, and frame period.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period will be returned in timeperframe.
 *
 * Parameters
 *   pix            Information about the desired pixel format.
 *   xclk           The input clock frequency in Hz.
 *   timeperframe   The structure to fill with timing information.
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_configure(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
    struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
  	enum mt9d112_pixel_format pfmt;
	int err;

//    printk("mt9d112: Confirming regs.\n");
//    mt9d112_confirm_regs(&sensor->client, regs_init);

	pfmt = get_pixel_format(pix, NULL);

    DPRINTK("configure() - (%d, %d, %s)\n", pix->width, pix->height,
        mt9d112_pixel_formats[pfmt].description);

    /* Put the kibosh on double-buffered register updates. */
    err = mt9d112_write_regs(&sensor->client, regs_sync_changes_hold);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed regs_sync_changes_hold.\n");
        return err;
    }

	if (sensor->pfmt_a != pfmt) {
		err = mt9d112_write_regs(&sensor->client, regs_ctx_a_formats[pfmt]);
        if (err) {
            /* Release double-buffered register updates. */
            mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set pfmt.\n");
			return err;
        }
		sensor->pfmt_a = pfmt;
	}

    if (sensor->width_a != pix->width) {
        regs_ctx_a_width[1].value = pix->width;
		err = mt9d112_write_regs(&sensor->client, regs_ctx_a_width);
        if (err) {
            /* Release double-buffered register updates. */
            mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set width.\n");
			return err;
        }
        sensor->width_a = pix->width;
    }

    if (sensor->height_a != pix->height) {
        regs_ctx_a_height[1].value = pix->height;
		err = mt9d112_write_regs(&sensor->client, regs_ctx_a_height);
        if (err) {
            /* Release double-buffered register updates. */
            mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to set height.\n");
			return err;
        }
        sensor->height_a = pix->height;
    }

//    printk("mt9d112: Confirming regs.\n");
//    mt9d112_confirm_regs(&sensor->client, regs_init);

    /* Release double-buffered register updates. */
    err = mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
    if (err) {
        /* Release double-buffered register updates. */
        mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed regs_sync_changes_release.\n");
        return err;
    }

    /* Start preview mode (the default). */
	err = mt9d112_write_regs(&sensor->client, regs_program_preview_run);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed to run preview program.\n");
		return err;
    }

//    printk("mt9d112: Confirming regs.\n");
//    mt9d112_confirm_regs(&sensor->client, regs_init);

	return 0;
}

/******************************************************************************
 *
 * mt9d112sensor_configure_still_capture()
 *
 * Given a capture format in pix, the frame period in timeperframe, and
 * the xclk frequency, configure the MT9D112 context B for a specified
 * image size, pixel format, and frame period.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period will be returned in timeperframe.
 *
 * Parameters
 *   pix            Information about the desired pixel format.
 *   xclk           The input clock frequency in Hz.
 *   timeperframe   The structure to fill with timing information.
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_configure_still_capture(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
    struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;
  	enum mt9d112_pixel_format pfmt;
	int err = 0;

	pfmt = get_pixel_format(pix, NULL);

    DPRINTK("configure_still_capture() - (%d, %d, %s)\n", pix->width, pix->height,
        mt9d112_pixel_formats[pfmt].description);

    /* Put the kibosh on double-buffered register updates. */
    err = mt9d112_write_regs(&sensor->client, regs_sync_changes_hold);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed regs_sync_changes_hold.\n");
        return err;
    }

	if (sensor->pfmt_b != pfmt) {
		err = mt9d112_write_regs(&sensor->client, regs_ctx_b_formats[pfmt]);
        if (err) {
            /* Release double-buffered register updates. */
            mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure_still_capture - "
                   "Failed to write context b pfmt.\n");
			return err;
        }
		sensor->pfmt_b = pfmt;
	}

    if (sensor->width_b != pix->width) {
        regs_ctx_b_width[1].value = pix->width;
		err = mt9d112_write_regs(&sensor->client, regs_ctx_b_width);
        if (err) {
            /* Release double-buffered register updates. */
            mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure_still_capture - "
                   "Failed to set width.\n");
			return err;
        }
        sensor->width_b = pix->width;
    }

    if (sensor->height_b != pix->height) {
        regs_ctx_b_height[1].value = pix->height;
		err = mt9d112_write_regs(&sensor->client, regs_ctx_b_height);
        if (err) {
            /* Release double-buffered register updates. */
            mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
            printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure_still_capture - "
                   "Failed to set height.\n");
			return err;
        }
        sensor->height_b = pix->height;
    }

    /* Release double-buffered register updates. */
    err = mt9d112_write_regs(&sensor->client, regs_sync_changes_release);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_configure - Failed regs_sync_changes_release.\n");
        return err;
    }

	return err;
}

/******************************************************************************
 *
 * mt9d112sensor_cleanup()
 *
 * Prepares the driver to exit. Balances mt9d112sensor_init(). This function
 * must de-initialize the sensor and its associated data structures.
 *
 * Parameters
 *   priv           The sensor to operate on.
 *
 * Returns
 *   Returns zero on success, or non-zero otherwise.
 *
 *****************************************************************************/
static int
mt9d112sensor_cleanup(void *priv)
{
	struct mt9d112_sensor *sensor = (struct mt9d112_sensor *) priv;

	if (sensor) {
		i2c_del_driver(&sensor->driver);
		board_cleanup();

        if(sensor->regopsarray != NULL)
            kfree(sensor->regopsarray);
 	}
	return 0;
}


/******************************************************************************
 *
 * mt9d112sensor_init()
 *
 * This routine allocates and initializes the data structure for the sensor,
 * powers up the sensor, registers the I2C driver, sends initilization sequence,
 * and sets a default image capture format in pix for both contexts. The capture
 * format is not actually programmed into the sensor by this routine. The camera
 * I/F driver configures the sensor for the needed format through _configure API
 *
 * Parameters
 *   pix            The desired pixel format for the video context (A).
 *   pix2           The desired pixel format for the still capture context (B).
 *
 * Returns
 *   This function returns a non-NULL value to indicate that initialization
 *   is successful.
 *
 *****************************************************************************/
static void *
mt9d112sensor_init(struct v4l2_pix_format *pix, struct v4l2_pix_format *pix2)
{
	struct mt9d112_sensor *sensor = &mt9d112;
	struct i2c_driver *driver = &sensor->driver;
 	int err;
	//const struct firmware *fw = NULL;

	if (board_init())
		return NULL;
	memset(sensor, 0, sizeof(*sensor));

    mt9d112sensor_power_on(sensor);

	driver->driver.name = "MT9D112 I2C driver";
	driver->id = I2C_DRIVERID_MISC;
	driver->class = I2C_CLASS_HWMON;
	driver->attach_adapter = mt9d112_i2c_probe_adapter;
	driver->detach_client = mt9d112_i2c_detach_client;

	err = i2c_add_driver(driver);
	if (err) {
		printk(KERN_ERR
			MOD_NAME "Failed to register MT9D112 I2C client.\n");
		mt9d112sensor_cleanup((void *)sensor);
		return NULL;
	}
	if (!sensor->client.adapter) {
		printk(KERN_WARNING MOD_NAME "Failed to detect MT9D112 SOC.\n");
		mt9d112sensor_cleanup((void *)sensor);
		return NULL;
	}
	else {
        printk(KERN_INFO MOD_NAME "MT9D112 SOC detected 0x%02x\n",
            sensor->ver);
	}

//    err = request_firmware(&fw, "JOPLIN_CAMERA:/mt9d112.ini",
//        &(sensor->client.dev));
//    if(err)
//        printk(KERN_WARNING MOD_NAME "Failed to get ini settings.\n");
//    else
//        process_ini_data(sensor, (REGISTER_OPERATION_ARRAY *)fw->data,
//            fw->size);

	DPRINTK("Initial command sequence ...");
	if (mt9d112_write_regs(&sensor->client, regs_init)) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_init - "
               "Failed to write initial register settings.\n");
		mt9d112sensor_cleanup((void *)sensor);
		return NULL;
	}
//    printk("mt9d112: Confirming regs.\n");
//    mt9d112_confirm_regs(&sensor->client, regs_init);

	DPRINTK("Configure PLL ...");
	if (configure_master_clock(sensor, MT9D112_DEF_MCLK_MHZ)) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_init - "
               "Failed configure_master_clock().\n");
		mt9d112sensor_cleanup((void *)sensor);
		return NULL;
	}

    /* Set up the preview mode settings. */
    DPRINTK("Setting preview mode settings.\n");
	err = mt9d112_write_regs(&sensor->client, regs_ctx_a_init);
    if (err) {
        printk(KERN_ERR MOD_NAME ":mt9d112sensor_init - "
               "Failed to write initial ctx a settings.\n");
		mt9d112sensor_cleanup((void *)sensor);
		return NULL;
    }

	/* Set the default brightness and contrast */
	mt9d112_set_brightness(&sensor->client, DEF_BRIGHTNESS);
	mt9d112_set_contrast(&sensor->client, DEF_CONTRAST);

	/* Set the default crop */
//	mt9d112_write_regs(&sensor->client, a_crop_list);
//	mt9d112_write_regs(&sensor->client, b_crop_list);
	sensor->crop_rect_a.left = sensor->crop_rect_a.top = 0;
	sensor->crop_rect_a.width = SVGA_WIDTH;
	sensor->crop_rect_a.height = SVGA_HEIGHT;
	sensor->crop_rect_b.left = sensor->crop_rect_b.top = 0;
	sensor->crop_rect_b.width = UXGA_WIDTH;
	sensor->crop_rect_b.height = UXGA_HEIGHT;

	/* little endian for both */
	sensor->swapbyte_a = 0;
	sensor->swapbyte_b = 0;

	/* Make the default format QCIF RGB565 for context A */
	pix->width = QCIF_WIDTH;
	pix->height = QCIF_HEIGHT;
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	mt9d112sensor_try_format(pix, (void *)sensor);
	/* Make the default format QVGA RGB565 for context B */
	pix2->width = QVGA_WIDTH;
	pix2->height = QVGA_HEIGHT;
	pix2->pixelformat = V4L2_PIX_FMT_RGB565;
	mt9d112sensor_try_format_still_capture(pix2, (void *)sensor);
	/* default sizes and formats not configured yet */
	sensor->pfmt_a = sensor->pfmt_b = -1;

//    printk("mt9d112: Confirming regs.\n");
//    mt9d112_confirm_regs(&sensor->client, regs_init);

	return (void *)sensor;
}

/* Sensor structure used to register with the OMAP24xx camera driver. */
struct camera_sensor camera_sensor_if = {
	version: 	0x01,
	name:		"MT9D112",
	parallel_mode:	PAR_MODE_NOBT8,
	hs_polarity:	SYNC_ACTIVE_HIGH,
	vs_polarity:	SYNC_ACTIVE_HIGH,
	image_swap:	0,
	init: 		mt9d112sensor_init,
	cleanup:	mt9d112sensor_cleanup,
	power_on:	mt9d112sensor_power_on,
	power_off:	mt9d112sensor_power_off,
	enum_pixformat:	mt9d112sensor_enum_pixformat,
	try_format:	mt9d112sensor_try_format,
	calc_xclk:	mt9d112sensor_calc_xclk,
	configure:	mt9d112sensor_configure,
	query_control:	mt9d112sensor_query_control,
	get_control:	mt9d112sensor_get_control,
	set_control:	mt9d112sensor_set_control,

    /* we allow capturing the cropped images so we implement the following
	 * optional crop functions.
	 */
	cropcap:	mt9d112sensor_cropcap,
	get_crop:	mt9d112sensor_get_crop,
	set_crop:	mt9d112sensor_set_crop,

    /* we support dual-context so we implement the following optional
	 * functions that are exclusively for the still image context.
	 */
	try_format_still_capture:	mt9d112sensor_try_format_still_capture,
	configure_still_capture:	mt9d112sensor_configure_still_capture,
	enter_still_capture: 		mt9d112sensor_enter_still_capture,
	exit_still_capture:		mt9d112sensor_exit_still_capture,
};

EXPORT_SYMBOL(camera_sensor_if);

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("MT9D112 Sensor Driver");
MODULE_LICENSE("GPL");
