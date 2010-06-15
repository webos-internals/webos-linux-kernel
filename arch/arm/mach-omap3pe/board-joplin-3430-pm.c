/*
 * This file contains code specific to the Joplin board.
 *
 * Copyright (C) 2008 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/err.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/mux.h>
#include <asm/arch/power_companion.h>

#include "pm.h"

#ifdef CONFIG_PM

/* DEBUG settings
 */
// #define printd(args...) printk(args)
#ifndef printd
#  define printd(args...)
#endif

/******************************************************************************/

static void twl_master_write(u8 reg, u8 val)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, val, reg);
}

static void hack_twl_regs(void)
{
	/* Enable access to the group and level power registers */
	twl_master_write(TWL4030_PROTECT_KEY, 0xC0);
	twl_master_write(TWL4030_PROTECT_KEY, 0x0C);

	/* VDD1
	 *   VCC-1.3-CPU-MPU-S
	 */
	twl_write(TWL4030_VDD1_DEV_GRP,    TWL_DEV_GROUP_P1);
	twl_write(TWL4030_VDD1_REMAP,      TWL_RES_SLEEP);

	/* VDD2
	 *   VCC-1.2-CPU-CORE-S
	 */
	twl_write(TWL4030_VDD2_DEV_GRP,    TWL_DEV_GROUP_P1);
	twl_write(TWL4030_VDD2_REMAP,      TWL_RES_SLEEP);

	/* VIO
	 *   VCC-1.8
	 */
	twl_write(TWL4030_VIO_DEV_GRP,     TWL_DEV_GROUP_ALL);

	/* Internal LDOs
	 */
	twl_write(TWL4030_VINTANA1_DEV_GRP, TWL_DEV_GROUP_P1);

	twl_write(TWL4030_VINTANA2_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VINTANA2_REMAP,   TWL_RES_OFF);

	twl_write(TWL4030_VINTDIG_DEV_GRP,  TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VINTDIG_REMAP,    TWL_RES_SLEEP);

	/* USB
	 */
	twl_write(TWL4030_VUSB3V1_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VUSB1V5_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VUSB1V8_DEV_GRP, TWL_DEV_GROUP_P3);

	/* VAUX1
	 *   VCC-3.0
	 */
	twl_write(TWL4030_VAUX1_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX1_DEDICATED, TWL_VAUX1_3P00);
	twl_write(TWL4030_VAUX1_REMAP,     TWL_RES_OFF);

	/* VAUX2
	 *   VCC-2.8-TXCO
	 */
	twl_write(TWL4030_VAUX2_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX2_DEDICATED, TWL_VAUX2_2P80);
	twl_write(TWL4030_VAUX2_REMAP,     TWL_RES_OFF);

	/* VAUX3
	 *   VCC-1.8-WL
	 */
	twl_write(TWL4030_VAUX3_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX3_DEDICATED, TWL_VAUX3_1P80);
	twl_write(TWL4030_VAUX3_REMAP,     TWL_RES_OFF);

	/* VAUX4
	 *   VCC-2.8-CAM-A
	 */
	twl_write(TWL4030_VAUX4_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX4_DEDICATED, TWL_VAUX4_2P80);
	twl_write(TWL4030_VAUX4_REMAP,     TWL_RES_OFF);

	/* VMMC1
	 *   VCC-1.8-3.15-SD
	 */
	twl_write(TWL4030_VMMC1_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VMMC1_DEDICATED, TWL_VMMC1_3P00);
	twl_write(TWL4030_VMMC1_REMAP,     TWL_RES_OFF);

	/* VMMC2
	 *   VCC-3.15-WL
	 */
	twl_write(TWL4030_VMMC2_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VMMC2_DEDICATED, TWL_VMMC2_3P15);
	twl_write(TWL4030_VMMC2_REMAP,     TWL_RES_OFF);

	/* VDAC
	 *   VCC-1.8-DAC
	 */
	twl_write(TWL4030_VDAC_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VDAC_DEDICATED,  TWL_VDAC_1P80);
	twl_write(TWL4030_VDAC_REMAP,      TWL_RES_SLEEP);

	/* VPLL1
	 *   VCC-1.8-PLL1-S
	 */
	twl_write(TWL4030_VPLL1_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VPLL1_REMAP,     TWL_RES_OFF);

	/* VPLL2
	 *   VCC-1.8-PLL2-S
	 */
	twl_write(TWL4030_VPLL2_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VPLL2_REMAP,     TWL_RES_OFF);

	/* VSIM
	 *   TV-OUT-EN
	 */
	twl_write(TWL4030_VSIM_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VSIM_REMAP,      TWL_RES_OFF);
	
	/* Keep nRESPWRON active during suspend.
	 */
	twl_write(TWL4030_NRESPWRON_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_NRESPWRON_REMAP,   TWL_RES_SLEEP);

	/* The following resources are not needed during suspend. Turn them
	 * OFF.
	 */
	twl_write(TWL4030_REGEN_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_REGEN_REMAP,      TWL_RES_OFF);

	twl_write(TWL4030_SYSEN_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_SYSEN_REMAP,      TWL_RES_OFF);

	twl_write(TWL4030_CLKEN_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_CLKEN_REMAP,      TWL_RES_OFF);

	twl_write(TWL4030_HFCLKOUT_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_HFCLKOUT_REMAP,   TWL_RES_OFF);

	/* Disable access to the group and level power registers */
	twl_master_write(TWL4030_PROTECT_KEY, 0);
}

static void dump_twl_regs(void)
{
	int i;

#define REG_ENTRY(n) { n, #n }
	static struct {
	       u8 reg;
	       char *name;
	} reglist[] = {
		REG_ENTRY(TWL4030_VMMC1_DEV_GRP),
		REG_ENTRY(TWL4030_VMMC1_DEDICATED),
		REG_ENTRY(TWL4030_VMMC1_REMAP),
		REG_ENTRY(TWL4030_VMMC2_DEV_GRP),
		REG_ENTRY(TWL4030_VMMC2_DEDICATED),
		REG_ENTRY(TWL4030_VMMC2_REMAP),
		REG_ENTRY(TWL4030_VSIM_DEV_GRP),
		REG_ENTRY(TWL4030_VSIM_DEDICATED ),
		REG_ENTRY(TWL4030_VSIM_REMAP ),
		REG_ENTRY(TWL4030_VAUX1_DEV_GRP),
		REG_ENTRY(TWL4030_VAUX1_DEDICATED),
		REG_ENTRY(TWL4030_VAUX1_REMAP),
		REG_ENTRY(TWL4030_VAUX2_DEV_GRP),
		REG_ENTRY(TWL4030_VAUX2_DEDICATED),
		REG_ENTRY(TWL4030_VAUX2_REMAP),
		REG_ENTRY(TWL4030_VAUX3_DEV_GRP),
		REG_ENTRY(TWL4030_VAUX3_DEDICATED),
		REG_ENTRY(TWL4030_VAUX3_REMAP),
		REG_ENTRY(TWL4030_VAUX4_DEV_GRP),
		REG_ENTRY(TWL4030_VAUX4_DEDICATED),
		REG_ENTRY(TWL4030_VAUX4_REMAP),
		REG_ENTRY(TWL4030_VPLL2_DEV_GRP),
		REG_ENTRY(TWL4030_VPLL2_DEDICATED),
		REG_ENTRY(TWL4030_VPLL2_REMAP),
		REG_ENTRY(TWL4030_VIO_DEV_GRP),
		REG_ENTRY(TWL4030_VIO_CFG ),
		REG_ENTRY(TWL4030_VIO_REMAP),

		REG_ENTRY(TWL4030_VPLL1_DEV_GRP),
		REG_ENTRY(TWL4030_VPLL1_REMAP),

		REG_ENTRY(TWL4030_VPLL2_DEV_GRP),
		REG_ENTRY(TWL4030_VPLL2_REMAP),

		REG_ENTRY(TWL4030_VDAC_DEV_GRP),
		REG_ENTRY(TWL4030_VDAC_REMAP),
		REG_ENTRY(TWL4030_VINTANA1_DEV_GRP),
		REG_ENTRY(TWL4030_VINTANA2_DEV_GRP),
		REG_ENTRY(TWL4030_VINTDIG_DEV_GRP),
		REG_ENTRY(TWL4030_VDD1_DEV_GRP),
		REG_ENTRY(TWL4030_VDD1_REMAP),
		REG_ENTRY(TWL4030_VDD2_DEV_GRP),
		REG_ENTRY(TWL4030_VDD2_REMAP),
		REG_ENTRY(TWL4030_HFCLKOUT_DEV_GRP),
		REG_ENTRY(TWL4030_HFCLKOUT_REMAP),
		REG_ENTRY(TWL4030_REGEN_DEV_GRP),
		REG_ENTRY(TWL4030_REGEN_REMAP),
		REG_ENTRY(TWL4030_CLKEN_DEV_GRP),
		REG_ENTRY(TWL4030_CLKEN_REMAP),
		REG_ENTRY(TWL4030_SYSEN_DEV_GRP),
		REG_ENTRY(TWL4030_SYSEN_REMAP),
		REG_ENTRY(TWL4030_VUSB3V1_DEV_GRP),
		REG_ENTRY(TWL4030_VUSB1V5_DEV_GRP),
		REG_ENTRY(TWL4030_VUSB1V8_DEV_GRP),
	};

	printd("##### TRITON register dump\n");
	for (i = 0; i < ARRAY_SIZE(reglist); i++) {
		u8  val;
		int rc;

		rc = twl4030_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &val, reglist[i].reg);
		if (rc) {
			printd("##### TWL: Error reading register %02x\n",
					reglist[i].reg);
			continue;
		}
		printd("##### TWL REG %02x: %02x (%s)\n", reglist[i].reg, val, reglist[i].name);
	}
}

/******************************************************************************
 *
 * Target specific SUSPEND/RESUME hooks
 *
 ******************************************************************************/

static int omap3_pm_joplin_valid(suspend_state_t state)
{
	int ret;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		hack_twl_regs();
		dump_twl_regs();
		ret = 1;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap3_pm_joplin_prepare(void)
{
	/* Set up additional wake-up sources that are not set by drivers.
	 */
	enable_irq_wake(OMAP_GPIO_IRQ(MODEM_WAKE_APP_GPIO));
	enable_irq_wake(OMAP_GPIO_IRQ(MODEM_WAKE_APP_USB_GPIO));
	enable_irq_wake(OMAP_GPIO_IRQ(13 /* button */));

	return 0;
}

static int omap3_pm_joplin_enter(suspend_state_t state)
{
#ifdef CONFIG_CORE_OFF
	/* Mux the SPI chip selects to safe mode.
	 *
	 * Not muxing to safe mode can gate OFF.
	 */
	omap_cfg_reg("AC2_3430_SPI1_CS0_OFF");
	omap_cfg_reg("J26_3430_LCD_SPI3_CS0_OFF");
#endif

	return 0;
}

static void omap3_pm_joplin_finish(void)
{
#ifdef CONFIG_CORE_OFF
	/* Mux the SPI chip selects back to their working mode.
	 */
	omap_cfg_reg("AC2_3430_SPI1_CS0");
	omap_cfg_reg("J26_3430_LCD_SPI3_CS0");
#endif
}


/******************************************************************************
 *
 * INIT
 *
 ******************************************************************************/

static struct platform_suspend_ops joplin_pm_ops = {
	.valid		= omap3_pm_joplin_valid,
	.prepare	= omap3_pm_joplin_prepare,
	.enter		= omap3_pm_joplin_enter,
	.finish		= omap3_pm_joplin_finish,
};

static int __init omap3_joplin_pm_init(void)
{
	int rc;

	rc = omap3_pm_register_target_pm_ops(&joplin_pm_ops);
	if (rc) {
		printk(KERN_ERR "OMAP3 PM: FAILED to register Joplin target.\n");
		return -1;
	}

	printk(KERN_INFO "OMAP3: PM: Joplin target registered successfully.\n");
	return 0;
}

__initcall(omap3_joplin_pm_init);
#endif /* CONFIG_PM */
