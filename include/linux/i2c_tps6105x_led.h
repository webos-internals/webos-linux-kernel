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

#ifndef _LINUX_TPS6105X_LED_H
#define _LINUX_TPS6105X_LED_H

#include <linux/i2c.h>

#define TPS6105X_I2C_DEVICE	"TPS6105X"
#define TPS6105X_I2C_DRIVER	"TPS6105X"
#define TPS6105X_I2C_ADDR	0x33

struct tps6105x_platform_data {
	int	(*enable_avin)(int enable);
};

extern int
tps6105x_i2c_read_u8(struct i2c_client* client, u8 index, u8* value);

extern int
tps6105x_i2c_write_u8(struct i2c_client* client, u8 index, u8 value);

#ifdef CONFIG_LEDS_TPS6105X_DBG
extern int tps6105x_dbg_register_i2c_client(struct i2c_client *);
extern void tps6105x_dbg_unregister_i2c_client(struct i2c_client *);
#else // !CONFIG_LEDS_TPS6105X_DBG
static inline int
tps6105x_dbg_register_i2c_client(struct i2c_client *)
{
	return (0);
}

static inline void
tps6105x_dbg_unregister_i2c_client(struct i2c_client *)
{
}
#endif // CONFIG_LEDS_TPS6105X_DBG
#endif // _LINUX_TPS6105X_LED_H
