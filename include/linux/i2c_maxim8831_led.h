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

#ifndef _MAXIM8831_H
#define _MAXIM8831_H

#include <linux/i2c.h>
#include <linux/leds.h> 

#define MAXIM8831_I2C_DEVICE	"MAXIM8831"
#define MAXIM8831_I2C_DRIVER	"MAXIM8831"
		
#define MAXIM8831_I2C_ADDR	0x4D

/* MAXIM8831 part supports five different LED control points. */
enum {
	LED1 = 0,
	LED2,
	LED3,
	LED4,
	LED5,
	MAX_NUM_LEDS,
};

struct maxim8831_led_config {
	struct led_classdev cdev;
	struct i2c_client *client;
	int used;
	int id;
	int default_state;
	unsigned int ramp_up_time, ramp_down_time;
	unsigned int off_time, on_time, blink_enable;
};

struct maxim8831_platform_data {
	struct maxim8831_led_config leds[MAX_NUM_LEDS];
	int (*board_probe)     (void *);
	int (*board_remove)    (void *);
	int (*board_suspend)   (void *);
	int (*board_resume)    (void *);
};

/* Controller register & bit definition. */
#define ON_OFF_CNTL			0x00
#define LED1_RAMP_CNTL			0x03
#define LED2_RAMP_CNTL			0x04
#define LED3_RAMP_CNTL			0x05
#define LED4_RAMP_CNTL			0x06
#define LED5_RAMP_CNTL			0x07
#define ILED1_CNTL			0x0B
#define ILED2_CNTL			0x0C
#define ILED3_CNTL			0x0D
#define ILED4_CNTL			0x0E
#define ILED5_CNTL			0x0F
#define LED3_BLINK_CNTL			0x17
#define LED4_BLINK_CNTL			0x18
#define LED5_BLINK_CNTL			0x19
#define BOOST_CNTL			0x1D
#define STAT1				0x2D
#define STAT2				0x2E
#define CHIP_ID1			0x39
#define CHIP_ID2			0x3A

#define LEDX_BLINK_EN			(1 << 6)
#define LEDX_RAMP_UP			(0x7 << 0)
#define LEDX_RAMP_DOWN			(0x7 << 3)
#define LEDX_BLINK_ON			(0x3 << 0)
#define LEDX_BLINK_OFF			(0x3 << 3)

#define STAT2_OVP			(1 << 0)
#define STAT2_TSD			(1 << 1)
#define STAT2_OSDD			(1 << 2)

extern void maxim8831_mod_brightness(struct i2c_client *, int, int);


#endif // MAXIM8831_H 
