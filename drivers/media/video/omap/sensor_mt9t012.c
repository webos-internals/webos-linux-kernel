/*
 * drivers/media/video/omap/sensor_mt9t012.c
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
 * Micron MT9T012 camera imager driver for OMAP3430SDP. It implements
 * the sensor interface defined in sensor_if.h.
 *
 * March 2007 - First version.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/mach-types.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/twl4030.h>

#include "sensor_if.h"
#include "mt9t012.h"

#ifdef CONFIG_VIDEO_OMAP_SENSOR_MT9T012
#undef MT9T012_DEBUG
#undef MT9T012_REG_DUMP

#define MOD_NAME "MT9T012:"

#ifdef MT9T012_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ERR MOD_NAME format "\n", ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

void dump_key_mt9t012_regs(void *priv);
static struct camera_sensor camera_sensor_if;

/* Board specific code. Currently the only supported board is OMAP3430SDP.
 * This code can be moved to a board file.
 */
#ifdef CONFIG_MACH_OMAP_3430SDP
/* The Mt9t012 I2C camera chip has a fixed slave address of 0x20 or 0x30
   depending on SADDR is LOW or HIGH. It is 0x20>>1 on 3430SDP. */
#define MT9T012_I2C_ADDR		0x10

#define CAMERA_STANDBY_GPIO 	2
#define CAMERA_RESET_GPIO  	15

#define VAUX_2_8_V			0x09
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

static int
board_init(void)
{
#ifdef CONFIG_TWL4030_CORE
	/* turn on analog power. Digital power supply is always on. */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
		VAUX_DEV_GRP_P1,TWL4030_VAUX2_DEV_GRP);
#else
#error "no power companion board defined!"
#endif

	/* Request and configure gpio pins, we only use reset pin */
	if (omap_request_gpio(CAMERA_RESET_GPIO) != 0)
		return -EIO;

	/* set to output mode */
	omap_set_gpio_direction(CAMERA_RESET_GPIO, GPIO_DIR_OUTPUT);

	/* have to put sensor to reset to guarantee detection */
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 0);
	udelay(100);

	/* nRESET is active LOW. set HIGH to release reset */
	omap_set_gpio_dataout(CAMERA_RESET_GPIO, 1);
	/* give sensor sometime to get out of the reset. Datasheet says
	   2400 xclks. 500us is enough for 6MHz xclk rate */
	udelay(500);

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

	omap_free_gpio(CAMERA_RESET_GPIO);
}

/* soft standby can be achieved by program or by STANDBY line */
static void
board_exit_standby(void)
{
}
static void
board_enter_standby(void)
{
}
#endif  /* CONFIG_MACH_OMAP_3430SDP */

#define VIDEO_MODE			0
#define IMAGE_MODE			1

/* we always ask for 12MHz xclk */
#define MT9T012_XCLK_HZ		12000000

#define MT9T012_MIN_FPS		12
#define MT9T012_DEF_FPS		15
#define MT9T012_MAX_FPS		30

#define FRAME_LENGTH	0x310
#define LINE_LENGTH	0x6AC
#define PLL_DIVISOR	3
#define VT_PIX_CLK_DIVISOR	10
#define COARSE_INT_TIME_15FPS	0x20F
#define COARSE_INT_TIME_30FPS	0x30F
/*
 * to ease development, we utilize sensor step by step:
 * Step1: no xy-inc no bining and no scaler 2000*1500 output - OK
 * step2: xy-inc 1000*750 ouput - OK
 * Step2: binning 1000*750 output - bad image. But bining should be superior to only xy-inc???
 * Step3: binning and scaler 500*375 output - enabling scaler doesn't generate valid syncs, no interrupt from ISP???
 */
#define USE_BINNING

//#define THREEMP_IMAGE_CAPTURE

#ifdef USE_BINNING

#define VIDEO_WIDTH_MAX_SCALER_ON	(512)
#define VIDEO_HEIGHT_MAX_SCALER_ON	(384)

#define VIDEO_WIDTH_MAX_SCALER_OFF	(1024)
#define VIDEO_HEIGHT_MAX_SCALER_OFF	(768)

#else

#define VIDEO_WIDTH_MAX		(2048)
#define VIDEO_HEIGHT_MAX	(1536)

#endif

#ifdef THREEMP_IMAGE_CAPTURE
#define IMAGE_WIDTH_MAX		(2048)
#define IMAGE_HEIGHT_MAX	(1536)
#else
#define IMAGE_WIDTH_MAX		(1024)
#define IMAGE_HEIGHT_MAX	(768)
#endif

#define MT9T012_I2C_DELAY	3  /* msec delay b/w accesses */

/* we keep 2 contexts, one for video mode and one for image mode */
struct mt9t012_sensor {
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
} mt9t012;
EXPORT_SYMBOL(mt9t012);


/* define a structure for sensor register initialization values */
struct mt9t012_reg {
	u16	length;
	u16 	reg;    /* 16-bit offset */
	u32 	val;	/* 8/16/32-bit value */
};


/* MT9T012 has 8/16/32 registers */
#define MT9T012_8BIT		1
#define MT9T012_16BIT		2
#define MT9T012_32BIT		4

#define MT9T012_TOK_TERM 	0xFF	/* terminating token for reg list */
#define MT9T012_TOK_DELAY	100	/* delay token for reg list */

const static struct mt9t012_reg initial_list[]= {
	{MT9T012_8BIT, REG_SOFTWARE_RESET, 0x01},
	{MT9T012_TOK_DELAY, 0x00, 0x05}, /* Delay =5ms, min 2400 xcks */
	{MT9T012_16BIT, REG_RESET_REGISTER, 0x18c8},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9T012_16BIT, REG_ANALOG_GAIN_GREENR, 0x0020},
	{MT9T012_16BIT, REG_ANALOG_GAIN_RED, 0x0020},
	{MT9T012_16BIT, REG_ANALOG_GAIN_BLUE, 0x0020},
	{MT9T012_16BIT, REG_ANALOG_GAIN_GREENB, 0x0020},
	{MT9T012_16BIT, REG_DIGITAL_GAIN_GREENR, 0x0100},
	{MT9T012_16BIT, REG_DIGITAL_GAIN_RED, 0x0100},
	{MT9T012_16BIT, REG_DIGITAL_GAIN_BLUE, 0x0100},
	{MT9T012_16BIT, REG_DIGITAL_GAIN_GREENB, 0x0100},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9T012_TOK_TERM, 0, 0}
};

const static struct mt9t012_reg pll_list[]= {
	{MT9T012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9T012_TOK_DELAY, 0x00, 0x05},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9T012_16BIT, REG_VT_PIX_CLK_DIV, 0x0005},
	{MT9T012_16BIT, REG_PRE_PLL_CLK_DIV, 0x0003},
	{MT9T012_16BIT, REG_PLL_MULTIPLIER, 0x0033},
	{MT9T012_16BIT, REG_OP_PIX_CLK_DIV, 0x000A},
	{MT9T012_16BIT, REG_OP_SYS_CLK_DIV, 0x0001},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9T012_TOK_TERM, 0, 0}
};

const static struct mt9t012_reg enable_pll_list[]= {
	{MT9T012_TOK_TERM, 0, 0}
};
const static struct mt9t012_reg bypass_pll_list[]= {
	{MT9T012_TOK_TERM, 0, 0}
};

const static struct mt9t012_reg stream_on_list[]= {
	{MT9T012_8BIT, REG_MODE_SELECT, 0x01},
	{MT9T012_TOK_DELAY, 0x00, 0x50},
	{MT9T012_TOK_TERM, 0, 0}
};
const static struct mt9t012_reg stream_off_list[]= {
	{MT9T012_8BIT, REG_MODE_SELECT, 0x00},
	{MT9T012_TOK_DELAY, 0x00, 0x50},
	{MT9T012_TOK_TERM, 0, 0}
};

/* configure SMIA parallel path, xy-binning, scaler */
const static struct mt9t012_reg enter_video_mode[]= {
	{MT9T012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9T012_TOK_DELAY, 0x00, 0x05},
#ifdef USE_BINNING  /* y_odd_inc=3, x_odd_inc=3, xy bin enable */
	/* TODO: figure out why bining messes up image? */
	{MT9T012_16BIT, REG_READ_MODE, (3<<2) | (3<<5) /*| (1<<10)*/},
#endif
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	{MT9T012_16BIT,REG_FRAME_LEN_LINES,0x310},
	{MT9T012_16BIT,REG_LINE_LEN_PCK,0x6AC},
	{MT9T012_16BIT,REG_COARSE_INT_TIME,0x30F},
	{MT9T012_16BIT,REG_FINE_INT_TIME, 0x204},
	{MT9T012_16BIT, REG_X_ADDR_START, 0x0004},
	{MT9T012_16BIT, REG_Y_ADDR_START, 0x0004},
	{MT9T012_16BIT, REG_X_ADDR_END, 0x0804},
	{MT9T012_16BIT, REG_Y_ADDR_END, 0x604},
	{MT9T012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_MAX_SCALER_ON},
	{MT9T012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_MAX_SCALER_ON},
	{MT9T012_16BIT, REG_SCALE_M, 0x0020}, /* (0x20/0x10=2) */
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9T012_TOK_TERM, 0, 0}
};

const static struct mt9t012_reg turn_on_scaler[]= {
	{MT9T012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9T012_TOK_DELAY, 0x00, 0x300},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	/* enable SMIA clock and disable seraliser */
	{MT9T012_16BIT, REG_RESET_REGISTER, 0x10c8},
	/*slew rate and datapath select */
	{MT9T012_16BIT, REG_DATAPATH_SELECT, (4<<10) | (4<<13) | 0x2},
	{MT9T012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_MAX_SCALER_ON},
	{MT9T012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_MAX_SCALER_ON},
	/* horizontal & vertical scaling */
	{MT9T012_16BIT, REG_SCALING_MODE, 0x0002},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* hold */
	{MT9T012_TOK_TERM, 0, 0}
};

const static struct mt9t012_reg turn_off_scaler[]= {
	{MT9T012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9T012_TOK_DELAY, 0x00, 0x300},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	/* disable SMIA clock and disable seraliser */
	{MT9T012_16BIT, REG_RESET_REGISTER, 0x18c8},
	/*slew rate and datapath select */
	{MT9T012_16BIT, REG_DATAPATH_SELECT, (4<<10) | (4<<13) | 0x0},
	{MT9T012_16BIT, REG_X_OUTPUT_SIZE, VIDEO_WIDTH_MAX_SCALER_OFF},
	{MT9T012_16BIT, REG_Y_OUTPUT_SIZE, VIDEO_HEIGHT_MAX_SCALER_OFF},
	{MT9T012_16BIT, REG_SCALING_MODE, 0x0000}, /* no scaling */
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* update */
	{MT9T012_TOK_TERM, 0, 0}
};

const static struct mt9t012_reg enter_image_mode[]= {
	{MT9T012_8BIT, REG_MODE_SELECT, 0x00},	/* stream off */
	{MT9T012_TOK_DELAY, 0x00, 0x300},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01}, /* hold */
	/*slew rate and datapath select */
	{MT9T012_16BIT, REG_DATAPATH_SELECT, (4<<10) | (4<<13) | 0x0},
	#ifdef THREEMP_IMAGE_CAPTURE
	{MT9T012_16BIT, REG_READ_MODE, (1<<2) | (1<<5)},
	{MT9T012_16BIT,REG_FRAME_LEN_LINES,0x610},
	{MT9T012_16BIT,REG_LINE_LEN_PCK,0xAAC},
	{MT9T012_16BIT,REG_COARSE_INT_TIME,0x60F},
	{MT9T012_16BIT,REG_FINE_INT_TIME, 0x204},
	{MT9T012_16BIT, REG_PLL_MULTIPLIER, 0x00A0},
	#else
	{MT9T012_16BIT, REG_READ_MODE, (3<<2) | (3<<5)},
	{MT9T012_16BIT, REG_PLL_MULTIPLIER, 0x0033},
	#endif
	{MT9T012_16BIT, REG_X_ADDR_START, 0x0004},
	{MT9T012_16BIT, REG_Y_ADDR_START, 0x0004},
	{MT9T012_16BIT, REG_X_ADDR_END, 0x0804},
	{MT9T012_16BIT, REG_Y_ADDR_END, 0x0604},
	{MT9T012_16BIT, REG_X_OUTPUT_SIZE, IMAGE_WIDTH_MAX},
	{MT9T012_16BIT, REG_Y_OUTPUT_SIZE, IMAGE_HEIGHT_MAX},
	{MT9T012_16BIT, REG_SCALING_MODE, 0x0000}, /* no scaling */
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* updating */
	{MT9T012_TOK_TERM, 0, 0}
};
/*
 * structure which will set the frame rate.will be populated
 * only when framerate is being calculated
 */
struct mt9t012_reg set_fps[2];

u32 min_exposure_time;
u32 max_exposure_time;
u32 pix_clk_freq;
#define COARSE_INT_TIME_INDEX	1
#define MIN_EXPOSURE_TIME_CAPTURE 26
#define MAX_EXPOSURE_TIME_CAPTURE 65000
#define DEF_PIX_CLK 20400000

/* structure which will set the exposure time*/
struct mt9t012_reg set_exposure_time[]= {
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01},
	{MT9T012_16BIT,REG_COARSE_INT_TIME,0x30F},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* updating */
	{MT9T012_TOK_TERM, 0, 0}
};

#define MIN_GAIN	0x08
#define MAX_GAIN	0x7F
#define GAIN_INDEX	1
struct mt9t012_reg set_analog_gain[]= {
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x01},
	{MT9T012_16BIT, REG_ANALOG_GAIN_GLOBAL, MIN_GAIN},
	{MT9T012_8BIT, REG_GROUPED_PAR_HOLD, 0x00}, /* updating */
	{MT9T012_TOK_TERM, 0, 0}
};
/*
 * Read a 8/16/32-bit mt9t012 register.  The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t012_read_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;
	if (data_length != MT9T012_8BIT && data_length != MT9T012_16BIT
					&& data_length != MT9T012_32BIT)
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
		mdelay(MT9T012_I2C_DELAY);
		msg->len = data_length;
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = 0;
		/* high byte comes first */
		if (data_length == MT9T012_8BIT)
			*val = data[0];
		else if (data_length == MT9T012_16BIT)
			*val = data[1] + (data[0] << 8);
		else
			*val = data[3] + (data[2] << 8) +
				(data[1] << 16) + (data[0] << 24);
		return 0;
	}
	DPRINTK("read from offset 0x%x error %d", reg, err);
	return err;
}

#define RETRY_COUNT		5

/* Write to a 8/16/32-bit mt9t012 register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t012_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[6];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;
	if (data_length != MT9T012_8BIT && data_length != MT9T012_16BIT
					&& data_length != MT9T012_32BIT)
		return -EINVAL;

again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2 + data_length;
	msg->buf = data;

	/* high byte goes out first */
	data[0] = (u8) (reg >> 8);;
	data[1] = (u8) (reg & 0xff);

	if (data_length == MT9T012_8BIT)
		data[2] = (u8) (val & 0xff);
	else if (data_length == MT9T012_16BIT){
		data[2] = (u8) (val >> 8);
		data[3] = (u8) (val & 0xff);
	}
	else {
		data[2] = (u8) (val >> 24);
		data[3] = (u8) (val >> 16);
		data[4] = (u8) (val >> 8);
		data[5] = (u8) (val & 0xff);
	}

	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;

	DPRINTK("wrote 0x%x to offset 0x%x error %d", val, reg, err);
	if (retry <= RETRY_COUNT) {
		DPRINTK("retry ... %d", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}


/* Initialize a list of MT9T012 registers.
 * The list of registers is terminated by MT9T012_TOK_TERM.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
mt9t012_write_regs(struct i2c_client *client,
		const struct mt9t012_reg reglist[])
{
	int err;
	const struct mt9t012_reg *next = reglist;

	for (; next->length != MT9T012_TOK_TERM; next++) {
		if (next->length == MT9T012_TOK_DELAY) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies(next->val));
			continue;
		}

		err = mt9t012_write_reg(client, next->length,
				next->reg, next->val);
		if (err)
			return err;
		mdelay(MT9T012_I2C_DELAY);
	}
	return 0;
}


static int
mt9t012_switch_to_video(struct i2c_client *client,int scaler)
{
	mt9t012_write_regs(client,enter_video_mode);
	if(scaler == 1){
		mt9t012_write_regs(client,turn_on_scaler);
	}
	else{
		mt9t012_write_regs(client,turn_off_scaler);
	}
	mt9t012_write_regs(client,set_fps);
	mt9t012_write_regs(client, stream_on_list);
	return 0;
}
static int
mt9t012_switch_to_image(struct i2c_client *client)
{
	mt9t012_write_regs(client,enter_image_mode);
	mt9t012_write_regs(client, stream_on_list);
	return 0;
}
/* Detect if an MT9T012 is present, and if so which revision.
 * Returns a negative error number if no device is detected.
 */
static int
mt9t012_detect(struct i2c_client *client)
{
	struct mt9t012_sensor *sensor = &mt9t012;
	u32 model_id, mfr_id, rev;

	if (!client)
		return -ENODEV;

	if (mt9t012_read_reg(client, MT9T012_16BIT, REG_MODEL_ID, &model_id))
		return -ENODEV;
	if (mt9t012_read_reg(client, MT9T012_8BIT, REG_MANUFACTURER_ID,
				&mfr_id))
		return -ENODEV;
	if (mt9t012_read_reg(client, MT9T012_8BIT, REG_REVISION_NUMBER, &rev))
		return -ENODEV;

	if ((model_id != MT9T012_MOD_ID) || (mfr_id != MT9T012_MFR_ID)) {
		/* We didn't read the values we expected, so
		 * this must not be an MT9T012.
		 */
		return -ENODEV;
	}

	sensor->model_id = model_id;
	sensor->mfr_id = mfr_id;
	sensor->revision = rev;
	return 0;
}

/* This function registers an I2C client via i2c_attach_client() for the
 * sensor device.  If 'probe' is non-zero, then the I2C client is only
 * registered if the device can be detected.  If 'probe' is zero, then no
 * device detection is attempted and the I2C client is always registered.
 * Returns zero if an I2C client is successfully registered, or non-zero
 * otherwise.
 */
static int
mt9t012_i2c_attach_client(struct i2c_adapter *adap, int addr, int probe)
{
	struct mt9t012_sensor *sensor = &mt9t012;
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
		client->adapter = NULL;
		return err;
	}

	if (probe) {
		err = mt9t012_detect(client);
		if (err < 0) {
			i2c_detach_client(client);
			client->adapter = NULL;
			return err;
		}
	}
	return 0;
}

/* This function is called by i2c_del_adapter() and i2c_del_driver()
 * if the adapter or driver with which this I2C client is associated is
 * removed.  This function unregisters the client via i2c_detach_client().
 * Returns zero if the client is successfully detached, or non-zero
 * otherwise.
 */
static int
mt9t012_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;

	return err;
}

/* This function will be called for each registered I2C bus adapter when our
 * I2C driver is registered via i2c_add_driver().  It will also be called
 * whenever a new I2C adapter is registered after our I2C driver is registered.
 * This function probes the specified I2C bus adapter to determine if an
 * MT9T012 sensor device is present.  If a device is detected, an I2C client
 * is registered for it via mt9t012_i2c_attach_client().  Note that we can't
 * use the standard i2c_probe() function to look for the sensor because the
 * OMAP I2C controller doesn't support probing.
 * Returns zero if an MT9T012 device is detected and an I2C client successfully
 * registered for it, or non-zero otherwise.
 */
static int
mt9t012_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return mt9t012_i2c_attach_client(adap, MT9T012_I2C_ADDR, 1);
}


/* ---------------------------------------------------------------------
 * Following are sensor interface functions implemented by
 * MT9T012 raw sensor driver. They are named mt9t012sensor_<name>
 */
static int
mt9t012sensor_power_on(void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;
	int err;

	/* the caller already enabled our XCLK, wait to make sure it is stable
	 */
	udelay(100);
	/* release STANDBY line */
	board_exit_standby();
	/* wait 1ms before enable PLL per Micron Tech. Note */
	mdelay(1);

	err = mt9t012_write_regs(&sensor->client, enable_pll_list);
	if (err)
		return err;
	err = mt9t012_switch_to_video(&sensor->client,sensor->scaler);
	return err;
}

/* we standby the camera in this call. Based on Micron Tech. Note, the hard
 *  standby we are using should only consume maximum 100uA.
 */
static int
mt9t012sensor_power_off(void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;
	int err;

	/* TODO: figure out what is needed here */
	DPRINTK("Entering standby mode ...");

	err = mt9t012_write_regs(&sensor->client, stream_off_list);
	mdelay(30);
	board_enter_standby();

	return 0;
}

/* TODO: need to decide where crop happens */
static int
mt9t012sensor_cropcap(struct v4l2_cropcap *cropcap, void *priv)
{
	return -EINVAL;
}

static int
mt9t012sensor_get_crop(struct  v4l2_crop *crop, void *priv)
{
	return -EINVAL;
}

static int
mt9t012sensor_set_crop(struct  v4l2_crop *crop, void *priv)
{
	return -EINVAL;
}

static int
mt9t012sensor_enter_still_capture(int frames, void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;
	int err;

	err = mt9t012_switch_to_image(&sensor->client);
	if (err)
		return err;

	/* need a few frames delay to stablize the output */
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(msecs_to_jiffies(200));
	return 0;
}
static int
mt9t012sensor_exit_still_capture(void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;

	return mt9t012_switch_to_video(&sensor->client,sensor->scaler);
}

static int
mt9t012sensor_query_control(struct v4l2_queryctrl *qc, void *priv)
{
	return -EINVAL;
}

static int
mt9t012sensor_get_control(struct v4l2_control *vc, void *priv)
{
	return -EINVAL;
}

static int
mt9t012sensor_set_control(struct v4l2_control *vc, void *priv)
{
	return -EINVAL;
}


/* Help to implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
 */
static int
mt9t012sensor_try_format(struct v4l2_pix_format *pix, void *priv)
{
	/* we output one of two resolutions based on the expected size.
	 * camera driver should use ISP resizer to do further scaling
	 */

	if (pix->width <= VIDEO_WIDTH_MAX_SCALER_ON) {
		pix->width = VIDEO_WIDTH_MAX_SCALER_ON;
		pix->height = VIDEO_HEIGHT_MAX_SCALER_ON;
	}
	else {
		pix->width = VIDEO_WIDTH_MAX_SCALER_OFF;
		pix->height = VIDEO_HEIGHT_MAX_SCALER_OFF;
	}

	return 0;
}
static int
mt9t012sensor_try_format_still_capture(struct v4l2_pix_format *pix, void *priv)
{
	/* max resolution for still image is QXGA + delta */
	/**
	 * we always output the max resolution. camera driver will use ISP
	 * resizer to do further scaling.Resizer cannot process input/output
	 * image width greater than 1280 in ES1.0 and so this check is required
	 * Supported sizes for image capture will be QXGA and any size where
	 * output width is less than 1280.
	 */
		pix->width =  IMAGE_WIDTH_MAX;
		pix->height = IMAGE_HEIGHT_MAX;

	return 0;
}

/* Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency
 * This sensor driver is designed to utilize on-chip PLL so the input
 * clock from the host is fixed at 24MHZ.
 */
static unsigned long
mt9t012sensor_calc_xclk(struct v4l2_pix_format *pix,
			struct v4l2_fract *timeperframe, void *priv)
{
	int fps;
	if ((timeperframe->numerator == 0)
	|| (timeperframe->denominator == 0)){
		/* supply a default nominal_timeperframe */
		timeperframe->numerator = 1;
		timeperframe->denominator = MT9T012_DEF_FPS;
	}
	fps = timeperframe->denominator/timeperframe->numerator; 
	if (fps < MT9T012_MIN_FPS){
		fps = MT9T012_MIN_FPS;
	}
	else if (fps >MT9T012_MAX_FPS){
		fps = MT9T012_MAX_FPS;
	}
	timeperframe->numerator = 1;
	timeperframe->denominator = fps;
	return MT9T012_XCLK_HZ;
}

/* Given a capture format in pix, the frame period in timeperframe, and
 * the xclk frequency, configure the MT9T012 context A for a specified
 * image size and frame period.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period will be returned in timeperframe.
 */
static int
mt9t012sensor_configure(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;
	int err = 0;

#if 0
	unsigned int mclk;

	/* configure frame rate if it changes. assumes the caller already
	   normalized the frame rate by calling calc_xclk */
	mclk = ((timeperframe->denominator/timeperframe->numerator)
		* MT9T012_MAX_MCLK_MHZ) / MT9T012_MAX_FPS;
	if (mclk != sensor->mclk) {
		err = configure_master_clock(sensor, mclk);
		if (err)
			return err;
	}
#endif


	unsigned int vt_pix_clk;
	unsigned int pll_multiplier;

	vt_pix_clk = (timeperframe->denominator/timeperframe->numerator)
				* FRAME_LENGTH * LINE_LENGTH;
	pll_multiplier = ((vt_pix_clk * VT_PIX_CLK_DIVISOR * PLL_DIVISOR)
				/MT9T012_XCLK_HZ) + 1;
	pix_clk_freq = (pll_multiplier * MT9T012_XCLK_HZ) /
				(PLL_DIVISOR * VT_PIX_CLK_DIVISOR);
	min_exposure_time = (0x204 * 1000000 / pix_clk_freq) + 1;
	max_exposure_time = ((0x30F * LINE_LENGTH) + 0x204) * 1000
						/ pix_clk_freq * 1000;
	set_fps[0].length = MT9T012_16BIT;
	set_fps[0].reg = REG_PLL_MULTIPLIER;    
	set_fps[0].val = pll_multiplier;
	set_fps[1].length = MT9T012_TOK_TERM;
	set_fps[1].reg = 0;    
	set_fps[1].val = 0;

	if (pix->width <= VIDEO_WIDTH_MAX_SCALER_ON) {
		DPRINTK("Turning on scaler...");
		err = mt9t012_write_regs(&sensor->client, turn_on_scaler);
		if (err) {
			DPRINTK("Error turning on scaler...%d", err);
			return err;
		}
		err = mt9t012_write_regs(&sensor->client, set_fps);
		if (err) {
			DPRINTK("Error setting frame rate...%d", err);
			return err;
		}
		mdelay(20);
		err = mt9t012_write_regs(&sensor->client, stream_on_list);
		if (err) {
			DPRINTK("Error entering stream...%d", err);
			return err;
		}
		sensor->scaler = 1;
		mdelay(20);
	}
	else
	if (pix->width > VIDEO_WIDTH_MAX_SCALER_ON ) {
		DPRINTK("Turning off scaler...");
		err = mt9t012_write_regs(&sensor->client, turn_off_scaler);
		if (err) {
			DPRINTK("Error turning off scaler...%d", err);
			return err;
		}
		err = mt9t012_write_regs(&sensor->client, set_fps);
		if (err) {
			DPRINTK("Error setting frame rate...%d", err);
			return err;
		}
		mdelay(20);
		err = mt9t012_write_regs(&sensor->client, stream_on_list);
		if (err) {
			DPRINTK("Error entering stream...%d", err);
			return err;
		}
		sensor->scaler = 0;
		mdelay(20);
	}
	
	dump_key_mt9t012_regs((void *)sensor);
	return err;
}

/* Context B configuration */
static int
mt9t012sensor_configure_still_capture(struct v4l2_pix_format *pix,
		unsigned long xclk,struct v4l2_fract *timeperframe, void *priv)
{

#if 0
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;
	int err;

	if ((pix->width != IMAGE_WIDTH_MAX) || (pix->height != IMAGE_HEIGHT_MAX)) {
		DPRINTK("Wrong Context B image size (%d, %d)...", pix->width, pix->height);
		return -EINVAL;
		//sensor->width_b = pix->width;
		//sensor->height_b = pix->height;
	}
#endif

	return 0;
}

/*
 * API to let user set sensor's exposure time
 */ 
int
mt9t012sensor_set_exposure_time(int mode, u32 exp_time)
{
	int err;
	struct mt9t012_sensor *sensor = &mt9t012;
	u32 coarse_int_time = 0;

	if (mode == VIDEO_MODE) {
		if((exp_time < min_exposure_time) ||
				(exp_time > max_exposure_time)) {
			DPRINTK("Exposure time not within the legal range...");
			return -EINVAL;
		}
		coarse_int_time = (((exp_time * (pix_clk_freq / 1000)) / 1000)
				- 0x204) / LINE_LENGTH ;
	} else if (mode == IMAGE_MODE) {
		if((exp_time < MIN_EXPOSURE_TIME_CAPTURE) ||
				(exp_time > MIN_EXPOSURE_TIME_CAPTURE)) {
			DPRINTK("Exposure time not within the legal range...");
			return -EINVAL;
		}
		coarse_int_time = (((exp_time * DEF_PIX_CLK) / 1000000)
					- 0x204) / LINE_LENGTH ;

	} else {
		DPRINTK("Err Invalid argument...");
		return -EINVAL;
	}

	set_exposure_time[COARSE_INT_TIME_INDEX].val = coarse_int_time;
	err = mt9t012_write_regs(&sensor->client, set_exposure_time);
	if (err) {
		DPRINTK("Error setting exposure time...%d\n", err);
	}
	return err;
}


/*
 * API to let user set sensor's analog gain
 */
int
mt9t012sensor_set_gain(u16 gain)
{
	int err;
	struct mt9t012_sensor *sensor = &mt9t012;
	
	if ((gain < MIN_GAIN) || (gain > MAX_GAIN))
	{
		DPRINTK("Gain not within the legal range...");
		return -EINVAL;
	}
	set_analog_gain[GAIN_INDEX].val = gain;
	err = mt9t012_write_regs(&sensor->client, set_analog_gain);
	if (err) {
		DPRINTK("Error setting gain...%d", err);
		return err;
	}
	return err;
}


/* Prepare for the driver to exit. Balances mt9t012sensor_init().
 * This function must de-initialize the sensor and its associated data
 * structures.
 */
static int
mt9t012sensor_cleanup(void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;

	if (sensor) {
		i2c_del_driver(&sensor->driver);
		board_cleanup();
 	}
	return 0;
}

int mt9t012sensor_restore(void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;
	
	DPRINTK("reconfiguring the sensor\n");
	mt9t012_write_regs(&sensor->client, initial_list);

	mt9t012_switch_to_video(&sensor->client, sensor->scaler);
	DPRINTK("reconfiguring the sensor done\n");
	return 0;
}

/* Initialize the MT9T012 raw imager.
 * This routine allocates and initializes the data structure for the sensor,
 * powers up the sensor, registers the I2C driver, sends initilization
 * sequence,and sets the default image capture sizes in pix and pix2. The
 * capture size is not actually programmed into the sensor by this routine.
 * The camera I/F driver configures the sensor for the needed format through
 * _configure API.
 * This function must return a non-NULL value to indicate that the
 * initialization is successful.
 */
static void *
mt9t012sensor_init(struct v4l2_pix_format *pix, struct v4l2_pix_format *pix2)
{
	struct mt9t012_sensor *sensor = &mt9t012;
	struct i2c_driver *driver = &sensor->driver;
	int err;

	if (board_init())
		return NULL;
	memset(sensor, 0, sizeof(*sensor));

	driver->driver.name = "MT9T012 I2C";
	driver->class = I2C_CLASS_HWMON;
	driver->attach_adapter = mt9t012_i2c_probe_adapter;
	driver->detach_client = mt9t012_i2c_detach_client;

	err = i2c_register_driver(THIS_MODULE, driver);
	if (err) {
		printk(KERN_ERR
			MOD_NAME "Failed to register MT9T012 I2C client. %d\n"
								, err);
		board_cleanup();
		return NULL;
	}
	if (!sensor->client.adapter) {
		printk(KERN_WARNING
			MOD_NAME "Failed to detect MT9T012 sensor.\n");
		mt9t012sensor_cleanup((void *)sensor);
		return NULL;
	}
	else {
		printk(KERN_INFO
			MOD_NAME " detected (model_id 0x%04x, Rev 0x%02x, "
			"mfr 0x%02x)\n",sensor->model_id, sensor->revision,
			sensor->mfr_id);
	}

	dump_key_mt9t012_regs((void *)sensor);
	DPRINTK("Initial command sequence ...");
	if (mt9t012_write_regs(&sensor->client, initial_list)) {
		mt9t012sensor_cleanup((void *)sensor);
		return NULL;
	}

	DPRINTK("Configure PLL ...");
	/* this sets up the default pll settings */
	if (mt9t012_write_regs(&sensor->client, pll_list)) {
		mt9t012sensor_cleanup((void *)sensor);
		return NULL;
	}

	DPRINTK("Entering video mode ...");
	if (mt9t012_write_regs(&sensor->client, enter_video_mode)) {
		mt9t012sensor_cleanup((void *)sensor);
		return NULL;
	}
	
	DPRINTK("Turning on the scaler ...");
	if (mt9t012_write_regs(&sensor->client, turn_on_scaler)) {
		mt9t012sensor_cleanup((void *)sensor);
		return NULL;
	}

	DPRINTK("Streaming on ...");
	if (mt9t012_write_regs(&sensor->client, stream_on_list)) {
		mt9t012sensor_cleanup((void *)sensor);
		return NULL;
	}

	sensor->scaler = 1;

	pix->width = VIDEO_WIDTH_MAX_SCALER_ON;
	pix->height = VIDEO_HEIGHT_MAX_SCALER_ON;
	pix->pixelformat = V4L2_PIX_FMT_SGRBG10;
	mt9t012sensor_try_format(pix, (void *)sensor);

	pix2->width = IMAGE_WIDTH_MAX;
	pix2->height = IMAGE_HEIGHT_MAX;
	pix2->pixelformat = V4L2_PIX_FMT_SGRBG10;
	mt9t012sensor_try_format_still_capture(pix2, (void *)sensor);

	dump_key_mt9t012_regs((void *)sensor);
	return (void *)sensor;
}

static void
mt9t012sensor_unregister(void)
{
	omap_cam_unregister_sensor(&camera_sensor_if);
}

static int __init
mt9t012sensor_register(void)
{
	int err;
	
	err = omap_cam_register_sensor(&camera_sensor_if);
	if (err) {
		printk(KERN_WARNING MOD_NAME 
			" Failed to register sensor.\n");
		omap_cam_unregister_sensor(&camera_sensor_if);
	}
	return err;
}

static struct camera_sensor camera_sensor_if = {
	version: 	0x03,
	name:		"MT9T012",
	sensor_type:	SENSOR_RAW,
	sensor_interface:SENSOR_PARALLEL,
	parallel_mode:	PAR_MODE_NOBT8,
	hs_polarity:	SYNC_ACTIVE_HIGH,
	vs_polarity:	SYNC_ACTIVE_HIGH,
	init: 		mt9t012sensor_init,
	cleanup:	mt9t012sensor_cleanup,
	power_on:	mt9t012sensor_power_on,
	power_off:	mt9t012sensor_power_off,
	try_format:	mt9t012sensor_try_format,
	calc_xclk:	mt9t012sensor_calc_xclk,
	configure:	mt9t012sensor_configure,
	query_control:	mt9t012sensor_query_control,
	get_control:	mt9t012sensor_get_control,
	set_control:	mt9t012sensor_set_control,
	/* we allow capturing the cropped images so we implement the following
	 * optional crop functions.
	 */
	cropcap:	mt9t012sensor_cropcap,
	get_crop:	mt9t012sensor_get_crop,
	set_crop:	mt9t012sensor_set_crop,
	/* we support dual-context so we implement the following optional
	 * functions that are exclusively for the still image context.
	 */
	try_format_still_capture:	mt9t012sensor_try_format_still_capture,
	configure_still_capture:	mt9t012sensor_configure_still_capture,
	enter_still_capture: 		mt9t012sensor_enter_still_capture,
	exit_still_capture:		mt9t012sensor_exit_still_capture,
	/*we allow setting of exposure time and gain by user*/
	set_exposure_time:		mt9t012sensor_set_exposure_time,
	set_gain:			mt9t012sensor_set_gain,
	restore:			mt9t012sensor_restore,
};

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("MT9T012 Sensor Driver");
MODULE_LICENSE("GPL");

#ifdef MT9T012_REG_DUMP
static struct mt9t012_reg_read {
	u16 length;
	u16 reg;
	char *name;
} read_reg_list[] = {
	{MT9T012_16BIT, REG_MODEL_ID, "REG_MODEL_ID"},
	{MT9T012_8BIT, REG_MODE_SELECT, "REG_MODE_SELECT"},
	{MT9T012_8BIT, REG_IMAGE_ORIENTATION, "REG_IMAGE_ORIENTATION"},
	{MT9T012_16BIT, REG_FINE_INT_TIME, "REG_FINE_INT_TIME"},
	{MT9T012_16BIT, REG_COARSE_INT_TIME, "REG_COARSE_INT_TIME"},

	{MT9T012_16BIT, REG_VT_PIX_CLK_DIV, "REG_VT_PIX_CLK_DIV"},
	{MT9T012_16BIT,	REG_VT_SYS_CLK_DIV, "REG_VT_SYS_CLK_DIV"},
	{MT9T012_16BIT,	REG_PRE_PLL_CLK_DIV, "REG_PRE_PLL_CLK_DIV"},
	{MT9T012_16BIT,	REG_PLL_MULTIPLIER, "REG_PLL_MULTIPLIER"},
	{MT9T012_16BIT,	REG_OP_PIX_CLK_DIV, "REG_OP_PIX_CLK_DIV"},
	{MT9T012_16BIT, REG_OP_SYS_CLK_DIV, "REG_OP_SYS_CLK_DIV"},

	{MT9T012_16BIT, REG_FRAME_LEN_LINES, "REG_FRAME_LEN_LINES"},
	{MT9T012_16BIT, REG_LINE_LEN_PCK, "REG_LINE_LEN_PCK"},

	{MT9T012_16BIT, REG_X_ADDR_START, "REG_X_ADDR_START"},
	{MT9T012_16BIT, REG_Y_ADDR_START, "REG_Y_ADDR_START"},
	{MT9T012_16BIT, REG_X_ADDR_END, "REG_X_ADDR_END"},
	{MT9T012_16BIT, REG_Y_ADDR_END, "REG_Y_ADDR_END"},
	{MT9T012_16BIT, REG_X_OUTPUT_SIZE, "REG_X_OUTPUT_SIZE"},
	{MT9T012_16BIT, REG_Y_OUTPUT_SIZE, "REG_Y_OUTPUT_SIZE"},

	{MT9T012_16BIT, REG_SCALING_MODE, "REG_SCALING_MODE"},
	{MT9T012_16BIT, REG_SCALE_M, "REG_SCALE_M"},
	{MT9T012_16BIT, REG_SCALE_N, "REG_SCALE_N"},

	{MT9T012_16BIT, REG_ROW_SPEED, "REG_ROW_SPEED"},
	{MT9T012_16BIT, REG_RESET_REGISTER, "REG_RESET_REGISTER"},
	{MT9T012_8BIT, REG_PIXEL_ORDER, "REG_PIXEL_ORDER"},
	{MT9T012_16BIT, REG_READ_MODE, "REG_READ_MODE"},

	{MT9T012_16BIT, REG_DATAPATH_SELECT, "REG_DATAPATH_SELECT"},

	{MT9T012_TOK_TERM, 0, NULL}
};
void dump_key_mt9t012_regs(void *priv)
{
	struct mt9t012_sensor *sensor = (struct mt9t012_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	struct mt9t012_reg_read *next = read_reg_list;
	u32 val;

	printk("Begin of dumping:\n");
	for (; next->length != MT9T012_TOK_TERM; next++) {
		if (mt9t012_read_reg(client, next->length, next->reg, &val))
			break;
		printk("%s[0x%x][%s] = 0x%x\n", next->name, next->reg,
				(next->length == MT9T012_8BIT)?"8":
			(next->length == MT9T012_16BIT)?"16":"32", val);
		mdelay(MT9T012_I2C_DELAY);
	}
	printk("End of dumping.\n");
}
#else
void dump_key_mt9t012_regs(void *priv) {}
#endif	/* MT9T012_REG_DUMP */

late_initcall(mt9t012sensor_register);
module_exit(mt9t012sensor_unregister);

#endif	/* CONFIG_VIDEO_OMAP_SENSOR_MT9T012 */


