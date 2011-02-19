/*
 *  include/linux/i2c_lm3528_led.h
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

#ifndef _LM3528_LED_H
#define _LM3528_LED_H

#include <linux/i2c.h>
#include <linux/leds.h> 

#define LED_OFF		0
#define LED_ON		1


#define  LM3528_I2C_DEVICE   	"LM3528"
#define  LM3528_I2C_DRIVER   	"LM3528"
#define  LM3528_I2C_ADDR	0x36

/* lm3528 LED platform data structure */
struct lm3528_platform_data{ 
	struct led_classdev cdev;
	struct i2c_client *client;
	int default_brightness;
	u8 default_current;
	int default_state;
	char *dev_name;
};

extern int
lm3528_i2c_read_reg(struct i2c_client * client, u8 addr, u8 * out);

extern int
lm3528_i2c_write_reg(struct i2c_client * client, u8 addr, u8 val);

#endif // LM3528
