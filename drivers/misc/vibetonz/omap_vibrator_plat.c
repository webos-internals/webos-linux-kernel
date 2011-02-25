 /*
 * linux/drivers/misc/omap_vibrator_plat.c
 *
 * OMAP Vibrator platform-dependent file.
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
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/gpio.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/twl4030-audio.h>

#include "vibrator_plat.h"

#define LEDLED_OFFSET                (0x00)
#define CODEC_MODE_OFFSET            (0x01)
#define VIBRA_CTL_OFFSET             (0x45)
#define VIBRA_SET_OFFSET             (0x46)
#define APLL_CTL_OFFSET              (0x3a)

#define ENABLE_LEDA                  (0x01)
#define ENABLE_LEDB                  (0x01 << 1)
#define CODEC_POWER_ON               (0x01 << 1)
#define AUDIO_DATA_VIB_DRIVER        (0x01 << 4)
#define AUDIO_DATA_DEFINES_DIRECTION (0x01 << 5)
#define DIR_NEGATIVE_POLARITY        (0x01 << 1)
#define HBRIDGE_POWER_ON             (0x01)
#define APLL_ENABLED                 (0x01 << 4)
#define APLL_INP_FREQ_DEFAULT        (0x06) // 26MHz (reset value)

#define DPRINTK(args...)
//#define DPRINTK(args...) printk(args)

static inline int twl4030_clear_bits(u8 mod, u8 reg, u32 bits)
{
	int rc;
	u8  val;

	rc = twl4030_i2c_read_u8(mod, &val, reg);
	if (rc)
		return rc;

	val &= ~bits;
	return twl4030_i2c_write_u8(mod, val, reg);
}

static inline int twl4030_set_bits(u8 mod, u8 reg, u32 bits)
{
	int rc;
	u8  val;

	rc = twl4030_i2c_read_u8(mod, &val, reg);
	if (rc)
		return rc;

	val |= bits;
	return twl4030_i2c_write_u8(mod, val, reg);
}

int plat_vibrator_init(void)
{
	int rc;

	DPRINTK("In %s...\n", __func__);

	/* Disable LEDA and LEDB
	 */
	rc = twl4030_clear_bits(TWL4030_MODULE_LED, LEDLED_OFFSET,
				ENABLE_LEDA | ENABLE_LEDB);
	if (rc)
		goto err;

	/* Configure the vibrator:
	 *   AUDIO_DATA_VIB_DRIVER = 0b0:
	 *       Use local vibrator driver, not audio data.
	 *   AUDIO_DATA_DEFINES_DIRECTION = 0b00:
	 *       Direction is given by bit VIBRA_DIR.
	 */
	rc = twl4030_clear_bits(TWL4030_MODULE_AUDIO_VOICE, VIBRA_CTL_OFFSET,
				AUDIO_DATA_VIB_DRIVER | AUDIO_DATA_DEFINES_DIRECTION);
	if (rc)
		goto err;

#if 0
	/* enable the audio PLL */
	rc = twl4030_set_bits(TWL4030_MODULE_AUDIO_VOICE, APLL_CTL_OFFSET,
				APLL_INP_FREQ_DEFAULT | APLL_ENABLED);
	if (rc)
		goto err;
#endif

	return 0;

err:
	printk(KERN_ERR "ERROR: OMAP vibrator: Initialization failed.\n");
	return rc;

}

int plat_vibrator_deinit(void)
{
	int rc;

	DPRINTK("In %s...\n", __func__);

	/* turn H-Bridge off */
	rc = twl4030_clear_bits(TWL4030_MODULE_AUDIO_VOICE, VIBRA_CTL_OFFSET,
				HBRIDGE_POWER_ON);
	if (rc)
		goto err;

#if 0
	/* disable the audio PLL */
	rc = twl4030_clear_bits(TWL4030_MODULE_AUDIO_VOICE, APLL_CTL_OFFSET,
				APLL_ENABLED);
	if (rc)
		goto err;
#endif
	return 0;

err:
	printk(KERN_ERR "ERROR: OMAP vibrator: De-init failed.\n");
	return rc;

}

int plat_vibrator_set_direction(int dir_control)
{
	int rc;

	DPRINTK("In %s...\n", __func__);

	/* forward drive: h-bridge dir-control bit set to +ve polarity;
	   also turn h-bridge power control on */
	if (dir_control > 0) {
		DPRINTK("In %s: fwd drive...\n", __func__);
		rc = twl4030_clear_bits(TWL4030_MODULE_AUDIO_VOICE, VIBRA_CTL_OFFSET,
					DIR_NEGATIVE_POLARITY);
		if (rc)
			goto err;

		rc = twl4030_set_bits(TWL4030_MODULE_AUDIO_VOICE, VIBRA_CTL_OFFSET,
					HBRIDGE_POWER_ON);
		if (rc)
			goto err;
	}
        /* reverse drive: h-bridge dir-control bit set to -ve polarity
	   also turn h-bridge power control on */
	else if (dir_control < 0) {
		DPRINTK("In %s: rev drive...\n", __func__);
		rc = twl4030_set_bits(TWL4030_MODULE_AUDIO_VOICE, VIBRA_CTL_OFFSET,
					DIR_NEGATIVE_POLARITY | HBRIDGE_POWER_ON);
		if (rc)
			goto err;
	}
	else { /* disable vibrator: turn h-bridge power control off */
		DPRINTK("In %s: stop...\n", __func__);
		rc = twl4030_clear_bits(TWL4030_MODULE_AUDIO_VOICE, VIBRA_CTL_OFFSET,
					HBRIDGE_POWER_ON);
		if (rc)
			goto err;
	}

	return 0;

err:
	printk(KERN_ERR "ERROR: OMAP vibrator: Setting direction failed.\n");
	return rc;
}

int plat_vibrator_set_duty_cycle(unsigned int duty_cycle)
{
	int rc;
	u8  value;

	/* compute vib turn-on value which is the inverse of the duty-cycle value;
	   translate input value to range 0x0 - 0xff */
	value = (u8)(((100 - duty_cycle) * 0xff)/100);
        /* min turn-on value: 0x01; max value: 0xff */
	if (!value)
		value = 0x1;

	DPRINTK("In %s: input: %u, reg_val: %u\n", __func__, duty_cycle, value);

	/* write vib turn-on value */
	rc = twl4030_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE, value, VIBRA_SET_OFFSET);
	if (rc)
		goto err;

	return 0;

err:
	printk(KERN_ERR "ERROR: OMAP vibrator: Setting duty cycle failed.\n");
	return rc;
}

int plat_vibrator_enable(bool enable)
{
	return twl4030_audio_codec_enable(enable);
}

EXPORT_SYMBOL(plat_vibrator_set_direction);
EXPORT_SYMBOL(plat_vibrator_set_duty_cycle);
EXPORT_SYMBOL(plat_vibrator_enable);

