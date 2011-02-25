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

#ifndef _TPS6025X_H
#define _TPS6025X_H

#include <linux/i2c.h>
#include <linux/leds.h> 

#define TPS6025X_I2C_DEVICE   "TPS6025X"
#define TPS6025X_I2C_DRIVER   "TPS6025X"

#define TPS60250_I2C_ADDR	0x77
#define TPS60252_I2C_ADDR	0x76

/* TPS6025X part supports up to seven LEDs. Each LED belongs to
 * one of three LED groups. Each group can be controlled independently,
 * but the LEDs that belong to the same group cannot be controlled
 * separately.
 *
 * 1) Main Display LED group: DM1 through DM4.
 *    64 steps of brightness
 *    For 0 < current < 1.5mA, one step is 0.1mA
 *    For 1.5mA < current < 25.5mA, one step is 0.5mA
 *
 * 2) Sub Display LED group: DS1 and DS2
 *    64 steps of brightness
 *    For 0 < current < 1.5mA, one step is 0.1mA
 *    For 1.5mA < current < 25.5mA, one step is 0.5mA
 *
 * 3) Aux Display LED group: DM5.
 *    Aux display LED can be configured in Main Display Mode or
 *    Aux Display Mode.
 *    
 *    In Main Display Mode, 64 steps of brightness is supported
 *    with maximum current of 25mA. It will behave like two groups
 *    above.
 *
 *    In Aux Display Mode, only 4 steps (20%, 40%, 70%, and 100%)
 *    of brightness is supported, with maximum current of 80mA.
 */
enum {
	DM1 = 0,
	DM2,
	DM3,
	DM4,
	DM5,
	DS1,
	DS2,
	TPS_MAX_NUM_LEDS,
};

struct tps6025x_led_config {
	struct led_classdev cdev;
	struct i2c_client *client;
	int enable;
	int id;
};

struct tps6025x_platform_data {
	struct tps6025x_led_config leds[TPS_MAX_NUM_LEDS];
	int dm5_aux_display_mode;
};

/* Controller register & bit definition. */
#define ENABLE_CONTROL			0
#define ENOLD				(1 << 6)
#define ENMAIN				(1 << 5)
#define ENSUB2				(1 << 4)
#define ENSUB1				(1 << 3)
#define ENAUX				(1 << 2)
#define DM5_MODE			(3 << 0)

#define SUB_DISP_CURRENT_CONTROL	1
#define LSUB				(0x3f << 0)

#define MAIN_DISP_CURRENT_CONTROL	2
#define LMAIN				(0x3f << 0)

#define AUX_BRIGHTNESS_OP_CONTROL	3
#define LAUX				(0x3f << 2)
#define MODE				(3 << 0)

#define DM5_MAIN_MODE			1
#define DM5_AUX_MODE			2

#endif // TPS6025X_H 
