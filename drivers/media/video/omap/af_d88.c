/*
 * drivers/media/video/omap/af_d88.c
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
 * DB88 auto focus driver for OMAP3430SDP.
 *
 * August 2007 - First version.
 */
 
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/arch/io.h>
#include "d88.h"

#define MOD_NAME "D88: "
/* For some reason, we see that the read operations to the D88 device
 * seems to put the device into an invalid state. this hence is being disabled
 */
#undef CAMDB88_AF_READ_VALIDATE

#define D88_AF_I2C_ADDR		0x1A /*(0x34>>1)*/
#define D88_I2C_RETRY_COUNT	5

#define MAX_FOCUS_VAL	1023
#define MAX_FOCUS_POS	100

/*==== Data Structure Definitions ==========================================*/
struct cam_af_dev
{
	struct i2c_client client;
	struct i2c_driver driver;
	u32 freq_mclk; /* in hz */
	u16 rev;
	u16 model_id;
}d88_device;

/*==== Helper Functions ====================================================*/
/**
 * @brief camaf_reg_write : D88 I2C write using i2c_transfer().
 *  
 * @param client - I2C client
 * @param offset - address offset
 * @param value - value to be written
 *  
 * @returns -  0 on success, other value otherwise
 */
static int
camaf_reg_write (struct i2c_client *client, u8 offset, u16 value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[3];
	int retry = 0;
	
	if (!client->adapter)
		return -ENODEV;
again:
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 3;
	msg->buf = data;
	
	data[0] = offset;
	data[1] = (u8) (value >> 8);
	data[2] = (u8) (value & 0xFF);
	
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;
		
	dev_dbg(&client->dev, "Wrote 0x%x to offset 0x%x error %d",
							value, offset, err);
	if (retry <= D88_I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retry ... %d", retry);
		retry++;
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(20));
		goto again;
	}
	return err;
}

#ifdef CAMDB88_AF_READ_VALIDATE
/**
 * @brief camaf_reg_read : D88 I2C read using i2c_transfer().
 *  
 * @param client - I2C client
 * @param offset - address offset
 * @param value - to store value read
 * 
 * @returns - 0 on success, other value otherwise  
 */
static int
camaf_reg_read (struct i2c_client *client, u8 offset, u16* value)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[3];
	
	if (!client->adapter)
		return -ENODEV;
	
	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 3;
	msg->buf = data;
	
	data[0] = offset;
	data[1] = 0;
	data[2] = 0;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		*value = data[2] + (data[1] << 8);		
		return 0;		
	}
	dev_dbg(&client->dev, "read from offset 0x%x error %d", offset, err);
	return err;
}
#endif

u8 i2c_allow_nak = 0;

/** 
 * @brief d88_detect : Detect the presence of D88 device
 * 
 * @param client - I2C client
 * 
 * @returns - 0 on success, other value otherwise
 */
static int
d88_detect(struct i2c_client *client)
{
	struct cam_af_dev *af_dev = &d88_device;
	u16 rev = 0;
	u16 id = 0;
	int err = 0;
#ifdef CAMDB88_AF_READ_VALIDATE	
	err = camaf_reg_read(client, CAMAF_D88_REV, &rev);
	if (err) {
		printk("Unable to read D88 revision number\n");
		return err;
	}
	err = camaf_reg_read(client, CAMAF_D88_ID, &id);
	if (err) {
		printk("Unable to read D88 ID number\n");
		return err;
	}
#endif
	af_dev->rev = rev;
	af_dev->model_id = id;
	return err;
}

/** 
 * @brief d88_i2c_attach_client : Registers I2C client through 
 * 				 i2c_attach_client().
 * 
 * @param adap - I2C adapter structure
 * @param addr - I2C client address
 * @param probe - If zero, no device detection attempted. If non-zero,
 * 		I2C client is only registered if device can be detected.
 * 
 * @returns - 0 on success, other value otherwise  
 */
static int
d88_i2c_attach_client(struct i2c_adapter *adap, int addr, int probe)
{
	struct cam_af_dev *af_dev = &d88_device;
	struct i2c_client *client = &af_dev->client;
	int err;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	client->addr = addr;
	i2c_set_clientdata(client, af_dev);
	client->driver = &af_dev->driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	if (probe) {
		err = d88_detect(client);
		if (err < 0) {
			i2c_detach_client(client);
			client->adapter = NULL;
			return err;
		}
	}
	return 0;
}

/**
 * @brief d88_i2c_detach_client : Unregisters I2C client via
 * 				i2c_detach_client().
 *  
 * @param client - I2C client
 * 
 * @returns - 0 on success  
 */ 
static int
d88_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;

	return err;
}

/**
 * @brief d88_i2c_probe_adapter : Probes the specified I2C bus to determine if
 * 				a D88 device is present.
 * 
 * @param adap - I2C adapter structure
 * 
 * @returns - 0 on success  
 */
static int
d88_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return d88_i2c_attach_client(adap, D88_AF_I2C_ADDR, 1);
}

/**
 * @brief d88_cleanup : Uses i2c_del_driver() to remove I2C driver.
 *  
 * @param priv - cam_af_dev structure
 * 
 * @returns - 0  
 */
static int
d88_cleanup(void *priv)
{
	struct cam_af_dev *af_dev = (struct cam_af_dev *) priv;

	if (af_dev) {
		i2c_del_driver(&af_dev->driver);
 	}
	return 0;
}


/*==== Exposed Function definitions ========================================*/
/**
 * @brief d88_af_setfocus : Sets the desired focus.
 *  
 * @param posn - Desired focus position, 0 (far) - 100 (close).
 * 
 * @returns - 0 on success, other value otherwise  
 */
int
d88_af_setfocus(u8 posn)
{
	struct cam_af_dev *af_dev = &d88_device;
	struct i2c_client *client = &af_dev->client;
	u16 focus_value;
	u32 calc_focus;
   	int ret = -EINVAL;

	if (posn > MAX_FOCUS_POS) {
		printk("Bad posn params\n");
		return ret;		
	}
	calc_focus = (posn * MAX_FOCUS_VAL) / MAX_FOCUS_POS;
	focus_value = (u16)calc_focus;

#ifdef CAMDB88_AF_READ_VALIDATE
	{
	u16 cur_focus_value;

	ret = camaf_reg_read(client, CAMAF_D88_CUR_POSN, &cur_focus_value);
	/* TBD: Something is failing in reads -> check the setup in i2c drv */
	if (ret) {
		drv_dbg("Read of current Lens position failed\n");
		return ret;
	}
	if (cur_focus_value == focus_value)
		dev_dbg(&client->dev, "Device already in requested focal point\n");
		return 0;
	}
#endif
	ret = camaf_reg_write(client, CAMAF_D88_NXT_POSN, focus_value);
	if (ret)
		dev_dbg(&client->dev, "Setfocus register write failed\n");
	return ret;
}

/**
 * @brief d88_af_deinit : Counterpart of d88_af_init().
 *  
 * @returns - 0 on success 
 */
int
d88_af_deinit(void)
{
	struct cam_af_dev *af_dev = &d88_device;
	struct i2c_client *client = &af_dev->client;
	int ret = 0;

	if (client->adapter != NULL)
		ret = d88_cleanup((void *)af_dev);
	return ret;
}

/**
 * @brief d88_af_init : Initializes D88, registering it via 
 * 			 i2c_register_driver().
 *   
 * @param mclk - input clock
 * 
 * @returns - 0 on success  
 */
int
d88_af_init(unsigned long mclk)
{
	struct cam_af_dev *af_dev = &d88_device;
	struct i2c_driver *driver = &af_dev->driver;	
	int err = -EINVAL;

	af_dev->freq_mclk= (u32) mclk;
	
	driver->driver.name = "D88 I2C";
	driver->class = I2C_CLASS_HWMON;
	driver->attach_adapter = d88_i2c_probe_adapter;
	driver->detach_client = d88_i2c_detach_client;

	err = i2c_register_driver(THIS_MODULE, driver);
	if (err) {
		printk(KERN_ERR MOD_NAME 
			"Failed to register D88 I2C client. %d\n", err);
		return err;
	}
	if (!af_dev->client.adapter) {
		printk(KERN_WARNING
			MOD_NAME "Failed to detect D88 AF Coil driver.\n");
		d88_cleanup((void *)af_dev);
		return err;
	}
	else {
		printk(KERN_INFO
			MOD_NAME " detected (model_id 0x%04x, Rev 0x%02x\n",
			af_dev->model_id, af_dev->rev);
	}

	err = camaf_reg_write (&af_dev->client, CAMAF_D88_FREQ_DIV,
				   CAMAF_FREQUENCY_EQ1(af_dev->freq_mclk));

	if(err)	{
		printk(KERN_INFO
			MOD_NAME "Unable to set the AF freq\n");
		d88_cleanup((void *)af_dev);
	}
	return err;
}

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("D88 Auto Focus Coil Driver");
MODULE_LICENSE("GPL");
