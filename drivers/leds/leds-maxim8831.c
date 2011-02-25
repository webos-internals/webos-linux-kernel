/* drivers/leds/leds-maxim8831.c
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
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/i2c_maxim8831_led.h>

#undef MAXIM8831_DEBUG

#define MAXIM8831_NUM_VALID_RAMP_TIME		8
#define MAXIM8831_NUM_VALID_ON_OFF_TIME		4
static unsigned int valid_ramp_time[MAXIM8831_NUM_VALID_RAMP_TIME] = {
	64, 128, 256, 512, 1024, 2048, 4096, 8192
};

/* If the off_time or on_time is written with zero, it signals
 * that the user wants to disable automatic blink on/off feature.
 */
static unsigned int valid_off_time[MAXIM8831_NUM_VALID_ON_OFF_TIME] = {
	512, 1024, 2048, 4096
};
static unsigned int valid_on_time[MAXIM8831_NUM_VALID_ON_OFF_TIME] = {
	256, 512, 1024, 2048
};

struct maxim8831_device_state {
	struct i2c_client		*i2c_dev;
	struct maxim8831_platform_data	*pdata;
	struct mutex			lock;
	int				suspended;

	int (*maxim8831_board_probe)(void *);
	int (*maxim8831_board_remove)(void *);
	int (*maxim8831_board_suspend)(void *);
	int (*maxim8831_board_resume)(void *);
};

static int maxim8831_i2c_read_reg(struct i2c_client* client, u8 addr, u8* out)
{
	int ret;
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].len = 1;
	msg[0].flags = 0;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].len = 1;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = out;

	ret = i2c_transfer(client->adapter, msg, 2);

	return ret;
}

static int maxim8831_i2c_write_reg(struct i2c_client* client, u8 addr, u8 val)
{
	u8 buf[2] = {addr, val};
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);

	return ret;
}

#ifdef MAXIM8831_DEBUG
static void maxim8831_dump_regs(struct i2c_client *client)
{
	u8 regs[19];
	memset(regs, 0xff, 19);

        maxim8831_i2c_read_reg(client, ON_OFF_CNTL, &regs[0]);
        maxim8831_i2c_read_reg(client, LED1_RAMP_CNTL, &regs[1]);
        maxim8831_i2c_read_reg(client, LED2_RAMP_CNTL, &regs[2]);
        maxim8831_i2c_read_reg(client, LED3_RAMP_CNTL, &regs[3]);
        maxim8831_i2c_read_reg(client, LED4_RAMP_CNTL, &regs[4]);
        maxim8831_i2c_read_reg(client, LED5_RAMP_CNTL, &regs[5]);
        maxim8831_i2c_read_reg(client, ILED1_CNTL, &regs[6]);
        maxim8831_i2c_read_reg(client, ILED2_CNTL, &regs[7]);
        maxim8831_i2c_read_reg(client, ILED3_CNTL, &regs[8]);
        maxim8831_i2c_read_reg(client, ILED4_CNTL, &regs[9]);
        maxim8831_i2c_read_reg(client, ILED5_CNTL, &regs[10]);
        maxim8831_i2c_read_reg(client, LED3_BLINK_CNTL, &regs[11]);
        maxim8831_i2c_read_reg(client, LED4_BLINK_CNTL, &regs[12]);
        maxim8831_i2c_read_reg(client, LED5_BLINK_CNTL, &regs[13]);
        maxim8831_i2c_read_reg(client, BOOST_CNTL, &regs[14]);
        maxim8831_i2c_read_reg(client, STAT1, &regs[15]);
        maxim8831_i2c_read_reg(client, STAT2, &regs[16]);
        maxim8831_i2c_read_reg(client, CHIP_ID1, &regs[17]);
        maxim8831_i2c_read_reg(client, CHIP_ID2, &regs[18]);

	printk("ON_OFF_CNTL	: 0x%02X\n", regs[0]);
	printk("LED1_RAMP_CNTL	: 0x%02X\n", regs[1]);
	printk("LED2_RAMP_CNTL	: 0x%02X\n", regs[2]);
	printk("LED3_RAMP_CNTL	: 0x%02X\n", regs[3]);
	printk("LED4_RAMP_CNTL	: 0x%02X\n", regs[4]);
	printk("LED5_RAMP_CNTL	: 0x%02X\n", regs[5]);
	printk("ILED1_CNTL	: 0x%02X\n", regs[6]);
	printk("ILED2_CNTL	: 0x%02X\n", regs[7]);
	printk("ILED3_CNTL	: 0x%02X\n", regs[8]);
	printk("ILED4_CNTL	: 0x%02X\n", regs[9]);
	printk("ILED5_CNTL	: 0x%02X\n", regs[10]);
	printk("LED3_BLINK_CNTL	: 0x%02X\n", regs[11]);
	printk("LED4_BLINK_CNTL	: 0x%02X\n", regs[12]);
	printk("LED5_BLINK_CNTL	: 0x%02X\n", regs[13]);
	printk("BOOST_CNTL	: 0x%02X\n", regs[14]);
	printk("STAT1		: 0x%02X\n", regs[15]);
	printk("STAT2		: 0x%02X\n", regs[16]);
	printk("CHIP_ID1	: 0x%02X\n", regs[17]);
	printk("CHIP_ID2	: 0x%02X\n", regs[18]);

	return;
}

#endif

/* Check for status register for open or short LEDs. We don't want to
 * turn on LED current regulator if there is no LED or a faulty one.
 */
static void maxim8831_check_leds(struct maxim8831_device_state *devstate)
{
	u8 i, err, used, stat1, stat2 = 0;
	struct maxim8831_led_config *leds = devstate->pdata->leds;
	struct i2c_client *client = devstate->i2c_dev;

	used =  (leds[LED5].used << LED5) |
		(leds[LED4].used << LED4) |
		(leds[LED3].used << LED3) |
		(leds[LED2].used << LED2) |
		(leds[LED1].used << LED1);

	/* Check for open/short LEDs. */
	maxim8831_i2c_read_reg(client, STAT1, &stat1);
	err = used & stat1;

	/* Does the controller report open/short amongst used LEDs? */
	if (err) {
		for (i = 0; i < MAX_NUM_LEDS; i++) {
			printk(KERN_ERR "%s: LED%d: %s\n", MAXIM8831_I2C_DRIVER,
			       i+1, (((err & (1 << i)) ? "Open/Short" : "OK")));
		}
	}

	/* Check STAT2 register. */
	maxim8831_i2c_read_reg(client, STAT2, &stat2);
	if (stat2 & STAT2_OVP) {
		printk(KERN_ERR "%s: Output overvoltage detection\n",
		       MAXIM8831_I2C_DRIVER);
	}
	if (stat2 & STAT2_TSD) {
		printk(KERN_ERR "%s: Thermal shutdown detection\n",
		       MAXIM8831_I2C_DRIVER);
	}
	if (stat2 & STAT2_OSDD) {
		printk(KERN_ERR "%s: Open Schottky diode detection\n",
		       MAXIM8831_I2C_DRIVER);
	}
	return;
}


void maxim8831_mod_brightness(struct i2c_client *client, int id, int value)
{
	u8 stat1, on_off, regdata = 0;
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);

	mutex_lock(&devstate->lock);

	/* Write to the MAXIM8831 register and change brightness. */
	switch(id) {
		case LED1:
		case LED2:
			/* Translate the range 0 through 256
			 * to 0 through 127.
			 */
			regdata |= value/2;
			maxim8831_i2c_write_reg(client, ILED1_CNTL +\
						(id - LED1), regdata);
			break;

		case LED3:
		case LED4:
		case LED5:
			/* Translate the range 0 through 256
			 * to 0 through 31.
			 */
			regdata |= value/8;
			maxim8831_i2c_write_reg(client, ILED3_CNTL +\
						(id - LED3), regdata);
			break;

		default:
			printk(KERN_ERR "%s: Unsupported LED ID: %d\n",
			       MAXIM8831_I2C_DRIVER, id);
	}

	/* Read in the current ON_OFF_CNTL reg. */
	maxim8831_i2c_read_reg(client, ON_OFF_CNTL, &on_off);

	if (value) {
	/* Check for open/short LEDs. */
	maxim8831_i2c_read_reg(client, STAT1, &stat1);

	/* Turn on the given LED if there is no open/short. */
		on_off |= ((1 << id) & ~stat1);
	} else {
		/* Turn off the given LED. */
		on_off &= ~(1 << id);
	}

	/* Update the ON_OFF_CNTL reg. */
	maxim8831_i2c_write_reg(client, ON_OFF_CNTL, on_off);

	mutex_unlock(&devstate->lock);

	return;
}
EXPORT_SYMBOL(maxim8831_mod_brightness);

/* Utility function that translate user supplied timing attributes
 * in msec into register field values.
 */
static int msec_to_reg(unsigned int *msec, int num, unsigned int *valid)
{
	int i;

	/* If given msec is smaller than the minimal msec, set it to
	 * the minimum value and return the bit value to write.
	 * Even if msec is zero, it will be set to the first valid
	 * msec value.
	 */
	if (*msec < valid[0]) {
		*msec = valid[0];
		return 0;
	}

	/* If given msec is somewhere between the minimum and maximum,
	 * check it against valid msec values and round-up or down.
	 */
	for (i = 1; i < num; i++) {
		if (*msec <= valid[i]) {
			/* Round up */
			if ((valid[i] - *msec) < (*msec - valid[i - 1])) {
				*msec = valid[i];
				return i;
			} else {
				*msec = valid[i - 1];
				return (i - 1);
			}
		}
	}

	/* The given msec must have exceeded the max msec. set it to
	 * the maximum value and return the bit value to write.
	 */
	*msec = valid[num - 1];
	return (num - 1);
}

/* Program current control register, ramp registers, blink
 * registers, and then turn on/off the LED regulators.
 */
static void maxim8831_set_brightness( struct led_classdev *led_cdev,
				      enum led_brightness value)
{
	struct maxim8831_led_config *led_config;
	struct i2c_client *client;
	struct maxim8831_device_state *devstate = NULL;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. We can't be here without these set up correctly. */
	if ((!led_config) || (!led_config->used) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't set led brightness.\n",
		       MAXIM8831_I2C_DRIVER);
		return;
	}

	/* Get the handle to the I2C client. */
	client = led_config->client;

	/* Change the brightness */
	maxim8831_mod_brightness(client, led_config->id, (int)value);

#ifdef MAXIM8831_DEBUG
	maxim8831_dump_regs(client);
#endif

	/* Get the status of LEDs. */
	devstate = i2c_get_clientdata(client);
	maxim8831_check_leds(devstate);

	return;
}

/* "ramp_up_time" attribute show function. */
static ssize_t maxim8831_ramp_up_time_show( struct device *cdev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used)) {
		printk(KERN_ERR "%s: Invalid LED. Can't show led "
				"ramp_up_time.\n", MAXIM8831_I2C_DRIVER);
		return 0;
	}
	return sprintf(buf, "%u\n", led_config->ramp_up_time) + 1;
}

/* "ramp_up_time" attribute mod function. */
static void maxim8831_mod_ramp_up_time(struct maxim8831_led_config *led_config,
				       unsigned long val)
{
	struct i2c_client *client = led_config->client;
	int id = led_config->id;
	u8 regdata;
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);

	mutex_lock(&devstate->lock);

	/* Update the ramp_up_time. */
	maxim8831_i2c_read_reg(client, (LED1_RAMP_CNTL + id), &regdata);
	regdata &= ~LEDX_RAMP_UP;
	regdata |= msec_to_reg((unsigned int*)&val,
				MAXIM8831_NUM_VALID_RAMP_TIME,
				valid_ramp_time);
	maxim8831_i2c_write_reg(client, (LED1_RAMP_CNTL + id), regdata);

	led_config->ramp_up_time = val;

	mutex_unlock(&devstate->lock);

	return;
}

/* "ramp_up_time" attribute store function. */
static ssize_t maxim8831_ramp_up_time_store( struct device *cdev,
					     struct device_attribute *attr,
					     const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't store led "
				"ramp_up_time.\n", MAXIM8831_I2C_DRIVER);
		return ret;
	}

	/* Convert string to unsigned long. */
	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if (count == size) {
		maxim8831_mod_ramp_up_time(led_config, val);
                ret = count;
	}
	return ret;
}

/* "ramp_down_time" attribute show function. */
static ssize_t maxim8831_ramp_down_time_show( struct device *cdev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used)) {
		printk(KERN_ERR "%s: Invalid LED. Can't show led "
				"ramp_down_time.\n", MAXIM8831_I2C_DRIVER);
		return 0;
	}
	return sprintf(buf, "%u\n", led_config->ramp_down_time) + 1;
}

/* "ramp_down_time" attribute mod function. */
static void maxim8831_mod_ramp_down_time(struct maxim8831_led_config *led_config,
					 unsigned long val)
{
	struct i2c_client *client = led_config->client;
	int id = led_config->id;
	u8 regdata;
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);

	mutex_lock(&devstate->lock);

	/* Update the ramp_down_time. */
	maxim8831_i2c_read_reg(client, (LED1_RAMP_CNTL + id), &regdata);
	regdata &= ~LEDX_RAMP_DOWN;
	regdata |= (msec_to_reg((unsigned int*)&val,
				MAXIM8831_NUM_VALID_RAMP_TIME,
				valid_ramp_time) << 3);
	maxim8831_i2c_write_reg(client, (LED1_RAMP_CNTL + id), regdata);

	led_config->ramp_down_time = val;

	mutex_unlock(&devstate->lock);

	return;
}


/* "ramp_down_time" attribute store function. */
static ssize_t maxim8831_ramp_down_time_store(struct device *cdev,
					      struct device_attribute *attr,
					      const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't store led "
				"ramp_down_time.\n", MAXIM8831_I2C_DRIVER);
		return ret;
	}

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if (count == size) {
		maxim8831_mod_ramp_down_time(led_config, val);
                ret = count;
	}
	return ret;
}

/* "on_time" attribute show function. */
static ssize_t maxim8831_on_time_show(  struct device *cdev,
					struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used)) {
		printk(KERN_ERR "%s: Invalid LED. Can't show led "
				"on_time.\n", MAXIM8831_I2C_DRIVER);
		return 0;
	}
	return sprintf(buf, "%u\n", led_config->on_time) + 1;
}

/* "on_time" attribute mod function. */
static void maxim8831_mod_on_time(struct maxim8831_led_config *led_config,
				  unsigned long val)
{
	struct i2c_client *client = led_config->client;
	int id = led_config->id;
	u8 regdata;
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);

	mutex_lock(&devstate->lock);

	/* Update the on_time. */
	maxim8831_i2c_read_reg(client, (LED3_BLINK_CNTL + id - LED3), &regdata);
	regdata &= ~LEDX_BLINK_ON;
	regdata |= msec_to_reg((unsigned int*)&val,
				MAXIM8831_NUM_VALID_ON_OFF_TIME,
				valid_on_time);
	maxim8831_i2c_write_reg(client, (LED3_BLINK_CNTL + id - LED3), regdata);
	led_config->on_time = val;

	mutex_unlock(&devstate->lock);

	return;
}

/* "on_time" attribute store function. */
static ssize_t maxim8831_on_time_store( struct device *cdev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't store led "
				"on_time.\n", MAXIM8831_I2C_DRIVER);
		return ret;
	}

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if ((count == size) && (led_config->id >= LED3)) {
		maxim8831_mod_on_time(led_config, val);
                ret = count;
	}
	return ret;

}

/* "off_time" attribute show function. */
static ssize_t maxim8831_off_time_show( struct device *cdev,
					struct device_attribute *attr,
					char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used)) {
		printk(KERN_ERR "%s: Invalid LED. Can't show led "
				"off_time.\n", MAXIM8831_I2C_DRIVER);
		return 0;
	}
	return sprintf(buf, "%u\n", led_config->off_time) + 1;
}

/* "off_time" attribute mod function. */
static void maxim8831_mod_off_time(struct maxim8831_led_config *led_config,
				   unsigned long val)
{
	struct i2c_client *client = led_config->client;
	int id = led_config->id;
	u8 regdata;
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);

	mutex_lock(&devstate->lock);

	/* Update the off_time. */
	maxim8831_i2c_read_reg(client, (LED3_BLINK_CNTL + id - LED3), &regdata);
	regdata &= ~LEDX_BLINK_OFF;
	regdata |= (msec_to_reg((unsigned int*)&val,
				MAXIM8831_NUM_VALID_ON_OFF_TIME,
				valid_off_time) << 3);
	maxim8831_i2c_write_reg(client, (LED3_BLINK_CNTL + id - LED3), regdata);
	led_config->off_time = val;

	mutex_unlock(&devstate->lock);

	return;
}

/* "off_time" attribute store function. */
static ssize_t maxim8831_off_time_store( struct device *cdev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't store led "
				"off_time.\n", MAXIM8831_I2C_DRIVER);
		return ret;
	}

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if ((count == size) && (led_config->id >= LED3)) {
		maxim8831_mod_off_time(led_config, val);
                ret = count;
	}
	return ret;
}

/* "blink_enable" attribute show function. */
static ssize_t maxim8831_blink_enable_show( struct device *cdev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used)) {
		printk(KERN_ERR "%s: Invalid LED. Can't show led "
				"blink_enable.\n", MAXIM8831_I2C_DRIVER);
		return 0;
	}
	return sprintf(buf, "%u\n", led_config->blink_enable) + 1;
}

/* "blink_enable" attribute mod function. */
static int maxim8831_mod_blink_enable( struct maxim8831_led_config *led_config,
				       unsigned long val)
{
	struct i2c_client *client = led_config->client;
	int id = led_config->id;
	u8 regdata;
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);

	mutex_lock(&devstate->lock);

	/* Perform value check. */
	if (val > 0) {
		led_config->blink_enable = 1;
	} else if (!val) {
		led_config->blink_enable = 0;
	} else {
		/* Got negative number. Invalid */
		mutex_unlock(&devstate->lock);
		return (-EINVAL);
	}

	/* Write BLINK_EN bit. */
	maxim8831_i2c_read_reg(client, (LED3_BLINK_CNTL + id - LED3), &regdata);
	if (led_config->blink_enable) {
		regdata |= LEDX_BLINK_EN;
	} else {
		regdata &= ~LEDX_BLINK_EN;
	}
	maxim8831_i2c_write_reg(client, (LED3_BLINK_CNTL + id - LED3), regdata);

	mutex_unlock(&devstate->lock);

	return 0;
}
/* "blink_enable" attribute store function. */
static ssize_t maxim8831_blink_enable_store(struct device *cdev,
					    struct device_attribute *attr,
					    const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(cdev);
	struct maxim8831_led_config *led_config;
	int ret = -EINVAL;
	unsigned long val;
	char *after;
	size_t count;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct maxim8831_led_config, cdev);

	/* Sanity check. */
	if ((!led_config) || (!led_config->used) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't store led "
				"blink_enable.\n", MAXIM8831_I2C_DRIVER);
		return ret;
	}

	val = simple_strtoul(buf, &after, 10);
	count = after - buf;
	if (*after && isspace(*after))
		count++;

	if ((count == size) && (led_config->id >= LED3)) {
		if (maxim8831_mod_blink_enable(led_config, val) < 0) {
			return ret;
		}
                ret = count;
	}
	return ret;
}

static DEVICE_ATTR(ramp_up_time, 0644, maxim8831_ramp_up_time_show,
			 maxim8831_ramp_up_time_store);
static DEVICE_ATTR(on_time, 0644, maxim8831_on_time_show,
			 maxim8831_on_time_store);
static DEVICE_ATTR(ramp_down_time, 0644, maxim8831_ramp_down_time_show,
			 maxim8831_ramp_down_time_store);
static DEVICE_ATTR(off_time, 0644, maxim8831_off_time_show,
			 maxim8831_off_time_store);
static DEVICE_ATTR(blink_enable, 0644, maxim8831_blink_enable_show,
			 maxim8831_blink_enable_store);


/* register led device */
static int maxim8831_register_led (struct maxim8831_led_config *led,
                                   struct i2c_client *client )
{
	int  ret;

	if (!led->used )
		return 0;

	if (!led->cdev.brightness_set) {
		led->cdev.brightness_set = maxim8831_set_brightness;
	}

	/* Save the i2c client information for each led. */
	led->client = client;

	/* Register this LED instance to the LED class. */
	ret = led_classdev_register(&client->dev, &led->cdev );
	if( ret ) {
		printk(KERN_ERR "%s: Failed to register "
			        "class device for LED%d\n",
			        MAXIM8831_I2C_DRIVER, led->id );
		goto led_register_err;
	}

	if (led->default_state)
		maxim8831_mod_brightness(client, led->id, LED_FULL);

	/* Create attributes. */
	if (device_create_file( led->cdev.dev, &dev_attr_ramp_up_time)) {
		printk(KERN_ERR "%s: Failed to create "
				"ramp_up_time attribute for LED%d\n",
				MAXIM8831_I2C_DRIVER, led->id );
		goto ramp_up_time_err;
	}
	if (device_create_file( led->cdev.dev, &dev_attr_on_time)) {
		printk(KERN_ERR "%s: Failed to create "
				"on_time attribute for LED%d\n",
				MAXIM8831_I2C_DRIVER, led->id );
		goto on_time_err;
	}
	if (device_create_file( led->cdev.dev, &dev_attr_ramp_down_time)) {
		printk(KERN_ERR "%s: Failed to create "
				"ramp_down_time attribute for LED%d\n",
				MAXIM8831_I2C_DRIVER, led->id );
		goto ramp_down_time_err;
	}
	if (device_create_file( led->cdev.dev, &dev_attr_off_time)) {
		printk(KERN_ERR "%s: Failed to create "
				"off_time attribute for  LED%d\n",
				MAXIM8831_I2C_DRIVER, led->id );
		goto off_time_err;
	}
	if (device_create_file( led->cdev.dev, &dev_attr_blink_enable)) {
		printk(KERN_ERR "%s: Failed to create "
				"blink_enable attribute for LED%d\n",
				MAXIM8831_I2C_DRIVER, led->id );
		goto blink_enable_err;
	}

	return 0;

blink_enable_err:
	device_remove_file( led->cdev.dev, &dev_attr_off_time);
off_time_err:
	device_remove_file( led->cdev.dev, &dev_attr_ramp_down_time);
ramp_down_time_err:
	device_remove_file( led->cdev.dev, &dev_attr_on_time);
on_time_err:
	device_remove_file( led->cdev.dev, &dev_attr_ramp_up_time );
ramp_up_time_err:
	led_classdev_unregister( &led->cdev );
led_register_err:
	return -1;
}

/* unregister led */
static void maxim8831_unregister_led (struct maxim8831_led_config *led )
{
	if(!led->used )
		return;

	device_remove_file( led->cdev.dev, &dev_attr_ramp_up_time);
	device_remove_file( led->cdev.dev, &dev_attr_on_time  );
	device_remove_file( led->cdev.dev, &dev_attr_ramp_down_time);
	device_remove_file( led->cdev.dev, &dev_attr_off_time );
	device_remove_file( led->cdev.dev, &dev_attr_blink_enable );
	led_classdev_unregister( &led->cdev );
}

/* Main probe function */
static int maxim8831_i2c_probe(struct i2c_client *client)
{
	struct maxim8831_device_state *devstate = NULL;
	struct maxim8831_platform_data *pdata = client->dev.platform_data;
	struct maxim8831_led_config *leds = NULL;
	int i, ret = 0;

	/* Check the platform data. */
	if (pdata == NULL) {
		printk(KERN_ERR "%s: missing platform data.\n",
		       MAXIM8831_I2C_DRIVER);
		return (-ENODEV);
	}

	/* Create the device state */
	devstate = kzalloc(sizeof(struct maxim8831_device_state), GFP_KERNEL);
	if (!devstate) {
		return (-ENOMEM);
	}

	/* Attach i2c_dev */
	devstate->i2c_dev = client;

	/* Attach platform data */
	devstate->pdata = pdata;
	devstate->maxim8831_board_probe = pdata->board_probe;
	devstate->maxim8831_board_remove = pdata->board_remove;
	devstate->maxim8831_board_suspend = pdata->board_suspend;
	devstate->maxim8831_board_resume = pdata->board_resume;

	/* Attach driver data. */
	i2c_set_clientdata(client, devstate);

	leds = pdata->leds;

	/* Init spinlock */
	mutex_init(&devstate->lock);

	/* Check LEDs. */
	maxim8831_check_leds(devstate);

#ifdef MAXIM8831_DEBUG
	maxim8831_dump_regs(client);
#endif

	/* Register led devices */
	for (i = 0; i < MAX_NUM_LEDS; i++) {
		/* Register this LED instance to the LED class. */
		ret = maxim8831_register_led ( leds+i, client );
		if( ret )
			goto register_error;
	}

	/* export "client" pointer so that it can be used by other components */
	if (devstate->maxim8831_board_probe)
		devstate->maxim8831_board_probe(client);

	/* set the default state of each led. */
	for (i = 0; i < MAX_NUM_LEDS; i++) {
		if (leds[i].default_state)
			maxim8831_mod_brightness(client, i, LED_FULL);
	}


	return 0;

register_error:

	/* Unwind registered led devices */
	do {
		--i;
		maxim8831_unregister_led ( leds + i );
	} while ( i );

	return -ENODEV;
}

static int maxim8831_i2c_remove(struct i2c_client *client)
{
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);
	struct maxim8831_platform_data *pdata = client->dev.platform_data;
	struct maxim8831_led_config *leds = NULL;
	int i;

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < MAX_NUM_LEDS; i++) {
			maxim8831_unregister_led ( leds + i );
		}
	}

	if (devstate->maxim8831_board_remove)
		devstate->maxim8831_board_remove(client);

	i2c_set_clientdata(client, NULL);

	kfree(devstate);
	return 0;
}

#ifdef CONFIG_PM
static int maxim8831_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);
	struct maxim8831_platform_data *pdata = client->dev.platform_data;
	struct maxim8831_led_config *leds = NULL;
	int i;

	if (devstate->suspended) {
		return 0;
	}
	devstate->suspended = 1;

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < MAX_NUM_LEDS; i++) {
			if (leds[i].used) {
				led_classdev_suspend(&leds[i].cdev);
			}
		}
	}

	if (devstate->maxim8831_board_suspend)
		devstate->maxim8831_board_suspend(client);

	return 0;
}

static int maxim8831_i2c_resume(struct i2c_client *client)
{
	struct maxim8831_device_state *devstate = i2c_get_clientdata(client);
	struct maxim8831_platform_data *pdata = client->dev.platform_data;
	struct maxim8831_led_config *leds = NULL;
	int i;

	if (!devstate->suspended) {
		return 0;
	}
	devstate->suspended = 0;

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < MAX_NUM_LEDS; i++) {
			if (leds[i].used) {
				led_classdev_resume(&leds[i].cdev);
			}
		}
	}

	if (devstate->maxim8831_board_resume)
		devstate->maxim8831_board_resume(client);
	return 0;
}
#else
#define maxim8831_i2c_suspend	NULL
#define maxim8831_i2c_resume	NULL
#endif

static struct i2c_driver maxim8831_i2c_driver = {
	.probe		= maxim8831_i2c_probe,
	.remove		= maxim8831_i2c_remove,
	.suspend	= maxim8831_i2c_suspend,
	.resume		= maxim8831_i2c_resume,
	.driver		= {
		.name		= MAXIM8831_I2C_DRIVER,
		.owner		= THIS_MODULE,
	},
};

static int __init maxim8831_i2c_init(void)
{
	return i2c_add_driver(&maxim8831_i2c_driver);
}

static void __exit maxim8831_i2c_exit(void)
{
 	i2c_del_driver(&maxim8831_i2c_driver);
}

module_init(maxim8831_i2c_init);
module_exit(maxim8831_i2c_exit);

MODULE_DESCRIPTION(MAXIM8831_I2C_DRIVER" LED Controller Driver");
MODULE_LICENSE("GPL");
