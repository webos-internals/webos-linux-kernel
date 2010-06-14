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

#ifndef _KXTE9_ACCELEROMETER_H
#define _KXTE9_ACCELEROMETER_H

#define KXTE9_I2C_DEVICE "kxte9_accelerometer"
#define KXTE9_I2C_DRIVER "kxte9_accelerometer"
#define KXTE9_I2C_DBG_DEVICE "kxte9_accelerometer_dbg"
#define KXTE9_I2C_DBG_DRIVER "kxte9_accelerometer_dbg"

#define KXTE9_I2C_ADDRESS 0x0F

#define ST_RESP		0x0C
#define B2S_THRESH		0x5B
#define WUF_THRESH		0x5A
#define WHO_AM_I	0x0F
#define TILT_POS_CUR	0x10
#define TILT_POS_PRE	0x11

#define TILT_POS_FU		(1<<0)
#define TILT_POS_FD		(1<<1)
#define TILT_POS_UP		(1<<2)
#define TILT_POS_DO		(1<<3)
#define TILT_POS_RI		(1<<4)
#define TILT_POS_LE		(1<<5)

#define WUF_THRESH_MAX_VALUE		20000
#define B2S_THRESH_MAX_VALUE		20000

#define XOUT		0x12
#define YOUT		0x13
#define ZOUT		0x14

#define INT_SRC_REG1	0x16
#define INT_SRC_REG2	0x17
#define STATUS_REG	0x18

#define INT_REL		0x1A
#define CTRL_REG1	0x1B
#define CTRL_REG2	0x1C
#define CTRL_REG3	0x1D
#define INT_CTRL_REG1	0x1E
#define INT_CTRL_REG2	0x1F

#define TILT_TIMER	0x28
#define WUF_TIMER	0x29
#define B2S_TIMER	0x2A

#define WUF_TIMER_MAX_VALUE		0xFF
#define B2S_TIMER_MAX_VALUE		0xFF

#define TILT_LOW_LIMIT_REG		0x5C
#define TILT_HIGH_LIMIT_REG		0x5D
#define TILT_HYSTERESIS_REG		0x5F

#define TILT_LOW_LIMIT_MAX		90


#define KXTE9_THRESH_LOCK_UNLOCK_REG	0x3E
#define KXTE9_THRESH_UNLOCK				0xCA
#define KXTE9_THRESH_LOCK				0xAA	// any value other than 0xCA is fine

#define OPERATING_MODE		0x80
#define STANDBY_MODE		0x1F

#define ENABLE_INTERRUPT	(0x10)
#define DISABLE_INTERRUPT	(0x0F)

#define XYZ_UNMASK		(0x0)
#define XYZ_MASK		(X_MASK | Y_MASK | Z_MASK)
#define X_MASK			(1<<7)
#define Y_MASK			(1<<6)
#define Z_MASK			(1<<5)

#define TILT_UNMASK		(0x0)
#define TILT_MASK		(LEFT_MASK | RIGHT_MASK | DOWN_MASK | UP_MASK | FACEDN_MASK | FACEUP_MASK)
#define LEFT_MASK		(1<<5)
#define RIGHT_MASK		(1<<4)
#define DOWN_MASK		(1<<3)
#define UP_MASK			(1<<2)
#define FACEDN_MASK		(1<<1)
#define FACEUP_MASK		(1<<0)

#define B2S_ENABLE		(1<<2)
#define WUF_ENABLE		(1<<1)
#define TPE_ENABLE		(1<<0)
#define ODR			(ODRA |ODRB)
#define	ODRA			(1<<4)
#define	ODRB			(1<<3)

#define OS2SA			(1<<3)
#define OS2SB			(1<<2)
#define OWUFA			(1<<1)
#define OWUFB			(1<<0)

#define B2S_INT			(1<<2)
#define WUF_INT			(1<<1)
#define TILT_INT		(1<<0)

#define ODR_1HZ			(0x0)
#define ODR_3HZ			(0x1)
#define ODR_10HZ		(0x2)
#define ODR_40HZ		(0x3)


#define TIMER_ENABLED		1
#define TIMER_DISABLED		0

#define POLL_INTERVAL		200

#define ACCEL_MIN_ABS_X		(-2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MAX_ABS_X		(2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MIN_ABS_Y		(-2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MAX_ABS_Y		(2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MIN_ABS_Z		(-2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MAX_ABS_Z		(2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_TILT_POS_MIN	(1)
#define ACCEL_TILT_POS_MAX	(6)

enum {
	MODE_OFF = 0,
	MODE_INT_ALL,
	MODE_INT_TILT,
	MODE_INT_AXIS,
};

struct kxte9_platform_data {
	char   *dev_name;
	int * xyz_translation_map;
	int	gpio;
	u16	wuf_thresh;
	u16	b2s_thresh;
	u32	wuf_timer;
	u32	b2s_timer;
	u8	tilt_timer;
	u8 tilt_thresh;
	u8 odr_main;
	u8 odr_b2s;
	u8 odr_wuf;
};

extern int
kxte9_i2c_read_u8(struct i2c_client * client, u8 index, u8 * out);

extern int
kxte9_i2c_write_u8(struct i2c_client * client, u8 index, u8 value);

#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXTE9
int  kxte9_dbg_register_i2c_client(struct i2c_client *i2c);
void kxte9_dbg_unregister_i2c_client(struct i2c_client *i2c);
#endif

#endif //KXTE9_ACCELEROMETER_H
