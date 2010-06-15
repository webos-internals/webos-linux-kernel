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
