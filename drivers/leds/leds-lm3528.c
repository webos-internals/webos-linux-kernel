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



struct lm3528_device_state {
	struct i2c_client	*i2c_dev;
	struct lm3528_platform_data pdata;
	struct mutex lock;
	struct file_operations fops;
	struct miscdevice mdev;
	int brightness;
	int suspended;
};

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

	//LM3528 only takes 7-bit PWM
	*pwm = ((255 * percent) / 100) >> 1;

done:
	return ret;
}

static void lm3528_set_brightness(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	struct lm3528_device_state *devstate; 
	struct lm3528_platform_data *pdata; 
	int pwm;

	struct i2c_client *client;

	/* Get the instance of LED configuration block. */
	pdata = container_of(led_cdev, struct lm3528_platform_data, cdev);

	/* Get the handle to the I2C client. */
	client = pdata->client;
	devstate = i2c_get_clientdata(client);

	/* Change the brightness */
	devstate->brightness = value;
	convert_percent_to_pwm(devstate->brightness,&pwm);

	lm3528_i2c_write_reg(client, 0xA0, pwm);

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

	/* Set default current */	
        state->brightness = pdata->default_brightness;
        convert_percent_to_pwm(state->brightness,&pwm);
        lm3528_i2c_write_reg(client, 0xA0, pwm);

	printk(KERN_INFO "LM3528: Turning %s on to default %d duty cycle\n", 
		pdata->cdev.name, pdata->default_brightness);

	pdata->cdev.brightness_set = lm3528_set_brightness;

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

#ifdef CONFIG_PM
static int lm3528_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	struct lm3528_device_state *devstate = i2c_get_clientdata(client);
	struct lm3528_platform_data *pdata = client->dev.platform_data;

	if (devstate->suspended) {
		return 0;
	}

	if (pdata) {
		led_classdev_suspend(&pdata->cdev);
	}

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
	state->suspended = 0;

	return 0;
}
#else 
#define lm3528_i2c_suspend NULL
#define lm3528_i2c_resume  NULL
#endif


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

module_init(lm3528_module_init);
module_exit(lm3528_module_exit);


MODULE_DESCRIPTION("NS LM3528 LED driver");
MODULE_LICENSE("GPL");
