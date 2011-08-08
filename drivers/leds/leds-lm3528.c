/*
 *  linux/drivers/leds/leds-lm3528.c - NS LED driver for the LM3528
 *
 *  Copyright (C) 2010 HP Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/i2c_lm3528_led.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>

static int lm3528_level_table[] = {
	0x00,
	0x43, 0x46, 0x4A, 0x4D, 0x50,
	0x51, 0x52, 0x53, 0x54, 0x54,
	0x56, 0x57, 0x59, 0x5A, 0x5C,
	0x5C, 0x5D, 0x5D, 0x5E, 0x5E,
	0x5F, 0x60, 0x61, 0x62, 0x63,
	0x63, 0x63, 0x64, 0x64, 0x64,
	0x65, 0x66, 0x66, 0x67, 0x68,
	0x68, 0x68, 0x69, 0x69, 0x69,
	0x69, 0x6A, 0x6A, 0x6B, 0x6C,
	0x6C, 0x6C, 0x6C, 0x6D, 0x6D,
	0x6D, 0x6D, 0x6E, 0x6E, 0x6F,
	0x6F, 0x6F, 0x6F, 0x6F, 0x70,
	0x70, 0x70, 0x71, 0x71, 0x71,
	0x71, 0x72, 0x72, 0x72, 0x72,
	0x72, 0x73, 0x73, 0x73, 0x74,
	0x74, 0x74, 0x74, 0x74, 0x74,
	0x74, 0x75, 0x75, 0x75, 0x75,
	0x76, 0x76, 0x76, 0x76, 0x76,
	0x76, 0x76, 0x77, 0x77, 0x77,
	0x77, 0x77, 0x77, 0x78, 0x78,
};

struct lm3528_device_state {
	struct i2c_client	*i2c_dev;
	struct lm3528_platform_data pdata;
	struct mutex lock;
	struct file_operations fops;
	struct miscdevice mdev;
	int brightness;
	int suspended;
	bool boost_requested;
	int boost_min_pwm;
	int boost_wait_time;  //minimum time for boost to stablize
};

static struct lm3528_device_state *lm3528_state = NULL;

int lm3528_i2c_read_reg(struct i2c_client* client, u8 addr, u8* out)
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
	if (ret < 0) {
		printk(KERN_ERR "lm3528_i2c_read_reg: error reading addr=0x%X\n", addr);
	}

	return ret;
}

int lm3528_i2c_write_reg(struct i2c_client* client, u8 addr, u8 val)
{
	u8 buf[2] = {addr, val};
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, 1);

	if (ret < 0) {
		printk(KERN_ERR "lm3528_i2c_write_reg: error writing addr=0x%X val=0x%X\n", addr, val);
	}
	return ret;
}

static int convert_percent_to_pwm(int percent, int *pwm)
{
	int ret = 0;

	if(percent < 0 || percent > 100)
	{
		ret = -EINVAL;
		goto done;
	}

	*pwm = lm3528_level_table[percent];

done:
	return ret;
}


static int _lm3528_set_brightness(struct i2c_client *client, int pwm)
{
	return lm3528_i2c_write_reg(client, 0xA0, pwm);
}

static void lm3528_set_brightness(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	struct lm3528_device_state *devstate;
	struct lm3528_platform_data *pdata;
	int pwm = 0;

	struct i2c_client *client;

	/* Get the instance of LED configuration block. */
	pdata = container_of(led_cdev, struct lm3528_platform_data, cdev);

	/* Get the handle to the I2C client. */
	client = pdata->client;
	devstate = i2c_get_clientdata(client);

	/* Change the brightness */
	convert_percent_to_pwm(value,&pwm);

	mutex_lock(&devstate->lock);

	if(devstate->boost_requested && pwm <= devstate->boost_min_pwm){
		printk("%s: boost is requested. Cannot go to pwm 0x%x\n", __func__,pwm);
		goto exit;
	}

	_lm3528_set_brightness(client, pwm);

	//Wait for boost to stablize when it is implicitly enabled
	if( (devstate->brightness == 0 && value > 0) &&
		lm3528_state->boost_wait_time > 0){

		msleep(lm3528_state->boost_wait_time);
	}

	devstate->brightness = value;

exit:

	mutex_unlock(&devstate->lock);

	return;
}

static int lm3528_open(struct inode *ip, struct file *fp)
{
	struct lm3528_device_state * p_state;
	int ret = 0;

	p_state = container_of(fp->f_op, struct lm3528_device_state, fops);

	if(!p_state)
	{
		printk("ERROR: lm3528_open: p_state is bad\n");
		return -EINVAL;
	}

	return ret;
}


static const struct file_operations lm3528_fops = {
	.open = lm3528_open,
};

static int lm3528_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3528_device_state *state = NULL;
	struct lm3528_platform_data *pdata = client->dev.platform_data;

	int ret;
	int pwm;
	struct i2c_adapter *adapter;

	adapter = to_i2c_adapter(client->dev.parent);

	/* Check the platform data. */
	if (pdata == NULL) {
		printk(KERN_ERR "%s: missing platform data.\n",
		       LM3528_I2C_DRIVER);
		return (-ENODEV);
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
		return -EIO;
	}
	/* Create the device state */
	state = kzalloc(sizeof(struct lm3528_device_state), GFP_KERNEL);
	if (!state) {
		return (-ENOMEM);
	}

	lm3528_state = state;

	/* Attach i2c_dev */
	state->i2c_dev = client;

	/* Attach platform data */
	memcpy(&state->pdata, pdata, sizeof(struct lm3528_platform_data));

	/* store fops */
	memcpy(&state->fops, &lm3528_fops, sizeof(struct file_operations));

	state->mdev.name = "lm3528";
	state->mdev.minor = MISC_DYNAMIC_MINOR;
	state->mdev.fops = &state->fops;

	/* Create misc device */
	ret = misc_register(&state->mdev);

	if(ret) {
		printk(KERN_ERR "LM3528: Failed to create /dev/lm3528\n");
		goto err0;
	}


	/* Attach driver data. */
	i2c_set_clientdata(client, state);

	/* Init mutex */
	mutex_init(&state->lock);

	/* Enable LED chip. */
	lm3528_i2c_write_reg(client, 0x10, 0x07);

	/* Boost Settings */
	state->boost_requested = false;
	state->boost_min_pwm = 0;
	state->boost_wait_time = pdata->boost_wait_time;

	/* Set default current */
	state->brightness = pdata->default_brightness;
	convert_percent_to_pwm(state->brightness,&pwm);
	lm3528_i2c_write_reg(client, 0xA0, pwm);

	printk(KERN_INFO "LM3528: Turning %s on to default %d duty cycle\n",
		pdata->cdev.name, pdata->default_brightness);

	pdata->cdev.brightness_set = lm3528_set_brightness;
	pdata->cdev.brightness = pdata->default_brightness;

	/* Save the i2c client information for each led. */
	pdata->client = client;

	/* Register this LED instance to the LED class. */
	ret = led_classdev_register(&client->dev, &pdata->cdev);

	return 0;

err0:
	kfree(state);

	return (ret);
}

static int lm3528_i2c_remove(struct i2c_client *client)
{
	struct lm3528_platform_data *pdata = client->dev.platform_data;

	if (pdata) {
		led_classdev_unregister(&pdata->cdev);
	}

	i2c_set_clientdata(client, NULL);

	return 0;
}


/*
 * There is a dependency between the brightness setting and boost
 * enablement - main issue is that setting brightness to 0 would result in
 * implicitly turning the boost off.
 *
 * lm3528_boost_request and lm3528_boost_release functions have been
 * added to maintain the boost to be enabled regardless of the lcd class
 * brightness setting via sysfs.
 *
 * Please note that the state of the boost_request is not maintained
 * across Apps suspend/resume. On Apps resume, the lm3528_boost_request
 * must be made again if necessary.
 */
int lm3528_boost_release(void)
{
	if(!lm3528_state){
		printk("%s: cannot retreive state\n",__func__);
		return -1;
	}

	mutex_lock(&lm3528_state->lock);

	lm3528_state->boost_requested = false;
	lm3528_state->boost_min_pwm = 0;

	mutex_unlock(&lm3528_state->lock);

	return 0;
}

int lm3528_boost_request(int min_pwm)
{
	int current_pwm = 0;

	if(!lm3528_state){
		printk("%s: cannot retreive state\n",__func__);
		return -1;
	}

	if(lm3528_state->suspended) {
		return -1;
	}

	mutex_lock(&lm3528_state->lock);

	lm3528_state->boost_requested = true;
	lm3528_state->boost_min_pwm = min_pwm;

	//Already at the minimum level
	convert_percent_to_pwm(lm3528_state->brightness,&current_pwm);
	if(current_pwm >= min_pwm) {
		goto exit;
	}

	//Set the brightness level to minimum, implicitly enabling the boost
	_lm3528_set_brightness(lm3528_state->i2c_dev, min_pwm);

exit:

	//Wait for the boost to stablize
	if(lm3528_state->boost_wait_time > 0){
		msleep(lm3528_state->boost_wait_time);
	}

	mutex_unlock(&lm3528_state->lock);

	return 0;
}

#ifdef CONFIG_PM
static int lm3528_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	struct lm3528_device_state *devstate = i2c_get_clientdata(client);
	struct lm3528_platform_data *pdata = client->dev.platform_data;

	if (devstate->suspended) {
		return 0;
	}

	lm3528_i2c_write_reg(client, 0x10, 0);

	if (pdata) {
		led_classdev_suspend(&pdata->cdev);
	}

	lm3528_state->boost_requested = false;

	devstate->suspended = 1;

	return 0;
}

static int lm3528_i2c_resume(struct i2c_client *client)
{
	struct lm3528_device_state *state = i2c_get_clientdata(client);
	struct lm3528_platform_data *pdata = client->dev.platform_data;

	if (!state->suspended) {
		return 0;
	}

	if (pdata) {
		led_classdev_resume(&pdata->cdev);
	}

	lm3528_i2c_write_reg(client, 0x10, 0x7);

	state->suspended = 0;

	return 0;
}
#else
#define lm3528_i2c_suspend NULL
#define lm3528_i2c_resume  NULL
#endif

static void lm3528_i2c_shutdown(struct i2c_client *client)
{
	//Ensure backlight is off on shutdown
	lm3528_i2c_write_reg(client, 0xA0, 0);

	return;
}


static struct i2c_device_id lm3528_i2c_id[] = {
	{ LM3528_I2C_DEVICE, 0 },
	{ }

};

static struct i2c_driver lm3528_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name = LM3528_I2C_DRIVER,
	},
	.probe		= lm3528_i2c_probe,
	.remove 	= lm3528_i2c_remove,
	.suspend 	= lm3528_i2c_suspend,
	.resume 	= lm3528_i2c_resume,
	.shutdown	= lm3528_i2c_shutdown,
	.id_table 	= lm3528_i2c_id,
};

static int __init
lm3528_module_init(void)
{
	int ret;

	ret = i2c_add_driver(&lm3528_i2c_driver);
	return (ret);
}

static void __exit
lm3528_module_exit(void)
{
	i2c_del_driver(&lm3528_i2c_driver);
}

#if defined(CONFIG_DEBUG_FS)

static u8 reg = 0;

static int lm3528_debug_reg_val_write(void *data, u64 val)
{
	struct lm3528_device_state *state = (struct lm3528_device_state *)data;

	if(val > 0xFF) {
		printk("lm3528_debug: value too large\n");
		return -1;
	}

	lm3528_i2c_write_reg(state->i2c_dev, reg, (u8)val);

	return 0;
}

static int lm3528_debug_reg_val_read(void *data, u64 *val)
{
	struct lm3528_device_state *state = (struct lm3528_device_state *)data;
	u8 out = 0;

	lm3528_i2c_read_reg(state->i2c_dev, reg, &out);

	*val = out;

	return 0;
}

static int lm3528_debug_reg_set(void *data, u64 val)
{
	if(val > 0xFF) {
		printk("lm3528_debug: value too large\n");
		return -1;
	}

	reg = (u8)val;

	printk("lm3528_debug: set reg to 0x%x\n",reg);

	return 0;
}

static int lm3528_debug_reg_get(void *data, u64 *val)
{
	*val = (u64)reg;

	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(lm3528_reg_fops, lm3528_debug_reg_get, lm3528_debug_reg_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(lm3528_val_fops, lm3528_debug_reg_val_read, lm3528_debug_reg_val_write, "%llu\n");

static int __init lm3528_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("lm3528", 0);
	if (IS_ERR(dent))
		return 0;

	(void) debugfs_create_file("reg", 0644, dent, lm3528_state, &lm3528_reg_fops);
	(void) debugfs_create_file("val", 0644, dent, lm3528_state, &lm3528_val_fops);

	return 0;
}

device_initcall(lm3528_debug_init);
#endif



module_init(lm3528_module_init);
module_exit(lm3528_module_exit);


MODULE_DESCRIPTION("NS LM3528 LED driver");
MODULE_LICENSE("GPL");
