/*
 * linux/arch/arm/mach-omap3pe/board-sirloin-3430-pm.c
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

#include <linux/wake_sources.h>
#ifdef CONFIG_FASTPATH
#include <linux/fastpath.h>
#include "board-sirloin-3430-wk.h"
#endif

#include "prcm-regs.h"
#include "pm.h"

#ifdef CONFIG_PM

/* Local compile switch flags
 */
//#define POWER_OFF_VAUX1_VAUX3_DURING_SUSPEND
//#define DEBUG_TRITON_REGISTERS

/* DEBUG settings
 */
//#define printd(args...) printk(args)
#ifndef printd
#  define printd(args...)
#endif

#ifdef CONFIG_FASTPATH
int omap3_wakeup_is_rtc_only(void);
void omap3_wakeup_sources_clear(void);
#endif

/******************************************************************************/

static void twl_master_write(u8 reg, u8 val)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, val, reg);
}

static void setup_twl4030_regs(void)
{
	/* Enable access to the group and level power registers */
	twl_master_write(TWL4030_PROTECT_KEY, 0xC0);
	twl_master_write(TWL4030_PROTECT_KEY, 0x0C);

	/* VDD1
	 *   VCC-1.3-CPU-MPU-S
	 */
	twl_write(TWL4030_VDD1_DEV_GRP,    TWL_DEV_GROUP_P1P3);
	twl_write(TWL4030_VDD1_REMAP,      TWL_RES_OFF);

	/* VDD2
	 *   VCC-1.2-CPU-CORE-S
	 */
	twl_write(TWL4030_VDD2_DEV_GRP,    TWL_DEV_GROUP_P1P3);
	twl_write(TWL4030_VDD2_REMAP,      TWL_RES_OFF);

	/* VIO
	 *   VCC-1.8
	 */
	twl_write(TWL4030_VIO_DEV_GRP,     TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VIO_REMAP,       TWL_RES_SLEEP);

	/* Internal LDOs
	 */
	twl_write(TWL4030_VINTANA1_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VINTANA1_REMAP,   TWL_RES_SLEEP);

	twl_write(TWL4030_VINTANA2_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VINTANA2_REMAP,   TWL_RES_OFF);

	twl_write(TWL4030_VINTDIG_DEV_GRP,  TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VINTDIG_REMAP,    TWL_RES_SLEEP);

	twl_write(TWL4030_32KCLKOUT_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_32KCLKOUT_REMAP,   TWL_RES_SLEEP);

	twl_write(TWL4030_NRESPWRON_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_NRESPWRON_REMAP,   TWL_RES_SLEEP);

	twl_write(TWL4030_TRITON_RESET_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_TRITON_RESET_REMAP,   TWL_RES_SLEEP);

	/* USB
	 */
	twl_write(TWL4030_VUSB3V1_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VUSB3V1_REMAP,   TWL_RES_OFF);
	twl_write(TWL4030_VUSB1V5_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VUSB1V5_REMAP,   TWL_RES_OFF);
	twl_write(TWL4030_VUSB1V8_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VUSB1V8_REMAP,   TWL_RES_OFF);

	twl_write(TWL4030_VUSBCP_DEV_GRP,  TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VUSBCP_REMAP,    TWL_RES_OFF);

	/* VAUX1
	 *   VCC-3.0-BTWL
	 */
	twl_write(TWL4030_VAUX1_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX1_DEDICATED, TWL_VAUX1_3P00);
#ifdef POWER_OFF_VAUX1_VAUX3_DURING_SUSPEND
	twl_write(TWL4030_VAUX1_REMAP,     TWL_RES_OFF);
#else
	twl_write(TWL4030_VAUX1_REMAP,     TWL_RES_ACTIVE);
#endif

	/* VAUX2
	 *   VCC-3.0-PROX
	 */
	twl_write(TWL4030_VAUX2_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX2_DEDICATED, TWL_VAUX2_2P80);
	twl_write(TWL4030_VAUX2_REMAP,     TWL_RES_OFF);

	/* VAUX3
	 *   VCC-1.8-WL
	 */
	twl_write(TWL4030_VAUX3_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX3_DEDICATED, TWL_VAUX3_1P80);
#ifdef POWER_OFF_VAUX1_VAUX3_DURING_SUSPEND
	twl_write(TWL4030_VAUX3_REMAP,     TWL_RES_OFF);
#else
	twl_write(TWL4030_VAUX3_REMAP,     TWL_RES_ACTIVE);
#endif

	/* VAUX4
	 *   VCC-3.0-AUD
	 */
	twl_write(TWL4030_VAUX4_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VAUX4_DEDICATED, TWL_VAUX4_2P80);
	twl_write(TWL4030_VAUX4_REMAP,     TWL_RES_OFF);

	/* VMMC1
	 *   VCC-3.0-IO
	 *
	 *   Need to keep on during suspend. Mostafa needs this power rail for
	 *   something mysterious. Keeping this on adds about 30-40uA sleep
	 *   current.
	 *
	 *   Update 080829: Keeping this OFF for the time being. Turning it ON
	 *   adds around ~900uA. Need to investigate more...
	 *
	 *   Update 081117: Turning this back ON for EVT2. We need VIO-3.0 for
	 *   the power button to work as the power button now sits on VIO-3.0
	 *   rather than VBAT.
	 */
	twl_write(TWL4030_VMMC1_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VMMC1_DEDICATED, TWL_VMMC1_3P00);
	twl_write(TWL4030_VMMC1_REMAP,     TWL_RES_SLEEP);

	/* VMMC2
	 *   Not connected: Turn OFF now.
	 */
	twl_write(TWL4030_VMMC2_DEV_GRP,   TWL_DEV_GROUP_NONE);
	twl_write(TWL4030_VMMC2_REMAP,     TWL_RES_OFF);

	/* VDAC
	 *   VCC-1.8-DAC
	 */
	twl_write(TWL4030_VDAC_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VDAC_DEDICATED,  TWL_VDAC_1P80);
	twl_write(TWL4030_VDAC_REMAP,      TWL_RES_OFF);

	/* VPLL1
	 *   VCC-1.8-PLL1-S
	 *   VCC-1.8-OSC
	 *
	 *   Need to keep on during suspend. 
	 *   Keeping VPLL1 enabled adds about 25uA sleep current.
	 */
	twl_write(TWL4030_VPLL1_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VPLL1_REMAP,     TWL_RES_SLEEP);

	/* VPLL2
	 *   VCC-1.8-PLL2 (VDDS_SDI)
	 */
	twl_write(TWL4030_VPLL2_DEV_GRP,   TWL_DEV_GROUP_P3);
	twl_write(TWL4030_VPLL2_REMAP,     TWL_RES_OFF);

	/* VSIM
	 *   TV-OUT-EN
	 */
	twl_write(TWL4030_VSIM_DEV_GRP,    TWL_DEV_GROUP_NONE);
	twl_write(TWL4030_VSIM_REMAP,      TWL_RES_OFF);

	/* The following resources are not needed during suspend. Turn them
	 * OFF.
	 */
	twl_write(TWL4030_REGEN_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_REGEN_REMAP,      TWL_RES_OFF);

	twl_write(TWL4030_SYSEN_DEV_GRP,    TWL_DEV_GROUP_P3);
	twl_write(TWL4030_SYSEN_REMAP,      TWL_RES_OFF);

	twl_write(TWL4030_CLKEN_DEV_GRP,    TWL_DEV_GROUP_P2P3);
	twl_write(TWL4030_CLKEN_REMAP,      TWL_RES_OFF);

	twl_write(TWL4030_HFCLKOUT_DEV_GRP, TWL_DEV_GROUP_P3);
	twl_write(TWL4030_HFCLKOUT_REMAP,   TWL_RES_OFF);

	/* Disable access to the group and level power registers */
	twl_master_write(TWL4030_PROTECT_KEY, 0);
}

static int __init init_twl_registers(void)
{
	printk(KERN_INFO "TWL4030: Setting up LDOs.\n");
	setup_twl4030_regs();
	return 0;
}
__initcall(init_twl_registers);

#ifdef DEBUG_TRITON_REGISTERS
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
		REG_ENTRY(TWL4030_VDAC_DEDICATED),
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
		REG_ENTRY(TWL4030_VUSB3V1_REMAP),
		REG_ENTRY(TWL4030_VUSB1V5_DEV_GRP),
		REG_ENTRY(TWL4030_VUSB1V5_REMAP),
		REG_ENTRY(TWL4030_VUSB1V8_DEV_GRP),
		REG_ENTRY(TWL4030_VUSB1V8_REMAP),

		REG_ENTRY(TWL4030_VUSBCP_DEV_GRP),
		REG_ENTRY(TWL4030_VUSBCP_REMAP),

	};

	printk("##### TRITON register dump\n");
	for (i = 0; i < ARRAY_SIZE(reglist); i++) {
		u8  val;
		int rc;

		rc = twl4030_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &val, reglist[i].reg);
		if (rc) {
			printk("##### TWL: Error reading register %02x\n", reglist[i].reg);
			continue;
		}
		printk("##### TWL REG %02x: %02x (%s)\n", reglist[i].reg, val, reglist[i].name);
	}
}
#else
#define dump_twl_regs()
#endif

/******************************************************************************
 *
 * Target specific SUSPEND/RESUME hooks
 *
 ******************************************************************************/

static int omap3_pm_sirloin_valid(suspend_state_t state)
{
	int valid = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		/* Triton registers are set up already at boot time. However,
		 * some drivers might have messed with the settings. Therefore,
		 * we are setting the registers again when we get here.
		 */
		setup_twl4030_regs();

		/* Clear the wake-up sources. This is necessary in case a
		 * driver aborts suspend and we unroll the suspend()ed drivers.
		 * We do not want to have any residual value in the wake-up
		 * sources mask when this happens.
		 */
#ifdef CONFIG_FASTPATH
		omap3_wakeup_sources_clear();
#endif
#ifdef CONFIG_WAKE_SOURCES
		clear_wakeup_events();
#endif

		valid = 1;
		break;

	default:
		valid = -EINVAL;
	}

	return valid;
}

static int omap3_pm_sirloin_prepare(void)
{
#ifdef CONFIG_FASTPATH
	omap3_wakeup_sources_clear();

	if (fastpath_prepare()) {
		printk(KERN_WARNING
			"%s: Could not prepare wakeup sources.\n", __FUNCTION__);
	}
#endif

	return 0;
}

static int omap3_pm_sirloin_enter(suspend_state_t state)
{
#ifdef CONFIG_CORE_OFF
	/* Configure the modem OFF mode muxing depending on the current GPIO
	 * setting.
	 */
	if (1 == omap_get_gpio_dataout(MODEM_POWER_ON_GPIO)) { /* GPIO153 */
		omap_cfg_reg("AD1_3430_GPIO153_PULLUP");
	} else {
		omap_cfg_reg("AD1_3430_GPIO153_PULLDOWN");
	}

	if (1 == omap_get_gpio_dataout(MODEM_WAKE_MODEM_GPIO)) { /* GPIO157 */
		omap_cfg_reg("AA21_3430_GPIO157_PULLUP");
	} else {
		omap_cfg_reg("AA21_3430_GPIO157_PULLDOWN");
	}

	/* Configure the BT OFF mode muxing depending on the current GPIO
	 * setting.
	 */
#ifdef CONFIG_MACH_SIRLOIN_3630
	if (1 == omap_get_gpio_dataout(BT_RESET_GPIO)) {
		omap_cfg_reg("AC3_BT_nRST_OFFMODE_HIGH");
	} else {
		omap_cfg_reg("AC3_BT_nRST_OFFMODE_LOW");
	}
#else
	if (1 == omap_get_gpio_dataout(BT_RESET_GPIO)) {
		omap_cfg_reg("AA3_BT_nRST_OFFMODE_HIGH");
	} else {
		omap_cfg_reg("AA3_BT_nRST_OFFMODE_LOW");
	}
#endif

	/* Configure the WL OFF mode muxing depending on the current GPIO
	 * setting.
	 */
#ifdef CONFIG_MACH_SIRLOIN_3630
	if (1 == omap_get_gpio_dataout(WIFI_RESET_GPIO)) {
		omap_cfg_reg("P26_WL_RST_nPWD_OFFMODE_HIGH");
	} else {
		omap_cfg_reg("P26_WL_RST_nPWD_OFFMODE_LOW");
	}
#else
	if (1 == omap_get_gpio_dataout(WIFI_RESET_GPIO)) {
		omap_cfg_reg("AE3_WL_RST_nPWD_OFFMODE_HIGH");
	} else {
		omap_cfg_reg("AE3_WL_RST_nPWD_OFFMODE_LOW");
	}
#endif	

	/* Configure the BT OFF mode muxing depending on the current GPIO
	 * setting.
	 */
	if (1 == omap_get_gpio_dataout(BT_WAKE_GPIO)) {
		omap_cfg_reg("AB2_BT_nWAKE_OFFMODE_HIGH");
	} else {
		omap_cfg_reg("AB2_BT_nWAKE_OFFMODE_LOW");
	}

#ifdef CONFIG_MACH_SIRLOIN_3630
	if (1 == omap_get_gpio_dataout(BT_RESET_GPIO)) {
		omap_cfg_reg("AC3_BT_nRST_OFFMODE_HIGH");
	} else {
		omap_cfg_reg("AC3_BT_nRST_OFFMODE_LOW");
	}
#else
	if (1 == omap_get_gpio_dataout(BT_RESET_GPIO)) {
		omap_cfg_reg("AA3_BT_nRST_OFFMODE_HIGH");
	} else {
		omap_cfg_reg("AA3_BT_nRST_OFFMODE_LOW");
	}
#endif

	/* Mux the SPI chip selects to safe mode.
	 *
	 * Not muxing to safe mode can gate OFF.
	 */
#ifdef CONFIG_MACH_SIRLOIN_3630
	omap_cfg_reg("Y4_3430_LCD_SPI2_CS0_OFF");
#else
	omap_cfg_reg("J26_3430_LCD_SPI3_CS0_OFF");
#endif
	omap_cfg_reg("AC2_3430_SPI1_CS0_OFF");


	/*
	 *  Mux backlight pwm to safe mode
	 */
#ifdef CONFIG_MACH_SIRLOIN_3630
	omap_cfg_reg("T8_LCD_LED_PWM_OFFMODE");
#else
#if defined(CONFIG_FB_OMAP_BACKLIGHT_EVT) \
	|| defined(CONFIG_FB_OMAP_BACKLIGHT_EVT_MODULE)
	/* LCD backlight PWM */
	omap_cfg_reg("Y2_LCD_LED_PWM_OFFMODE");
#endif /* CONFIG_FB_OMAP_BACKLIGHT_EVT[_MODULE] */
#endif /* CONFIG_MACH_SIRLOIN_3630 */



#endif /* CONFIG_CORE_OFF */

	return 0;
}

static void omap3_pm_sirloin_finish(void)
{
#ifdef CONFIG_CORE_OFF
	/* Mux the SPI chip selects back to their working mode.
	 */
	omap_cfg_reg("AC2_3430_SPI1_CS0");
#ifdef CONFIG_MACH_SIRLOIN_3630
	omap_cfg_reg("Y4_3430_LCD_SPI2_CS0");
#else
	omap_cfg_reg("J26_3430_LCD_SPI3_CS0");
#endif

	/* Restore the original muxing for modem */
	omap_cfg_reg("AD1_3430_GPIO153");
	omap_cfg_reg("AA21_3430_GPIO157");

	/* Restore the original muxing for WL */
#ifdef CONFIG_MACH_SIRLOIN_3630
	omap_cfg_reg("P26_WL_RST_nPWD");
#else
	omap_cfg_reg("AE3_WL_RST_nPWD");
#endif	

	/* Restore the original muxing for BT */
	omap_cfg_reg("AB2_BT_nWAKE");
#ifdef CONFIG_MACH_SIRLOIN_3630
	omap_cfg_reg("AC3_BT_nRST");
#else
	omap_cfg_reg("AA3_BT_nRST");
#endif


#ifdef CONFIG_MACH_SIRLOIN_3630
	omap_cfg_reg("T8_LCD_LED_PWM");
#else
#if defined(CONFIG_FB_OMAP_BACKLIGHT_EVT) \
	|| defined(CONFIG_FB_OMAP_BACKLIGHT_EVT_MODULE)
	/* LCD backlight PWM */
	omap_cfg_reg("Y2_LCD_LED_PWM");
#endif /* CONFIG_FB_OMAP_BACKLIGHT_EVT[_MODULE] */
#endif /* CONFIG_MACH_SIRLOIN_3630 */

#endif /* CONFIG_CORE_OFF */
}

static int omap3_pm_sirloin_fastsleep(void)
{
	int fastsleep = 0;
#ifdef CONFIG_FASTPATH
	int is_rtc_only;
	/* If RTC is only wakeup source, we may return to sleep. */
	is_rtc_only = omap3_wakeup_is_rtc_only();
	fastsleep = fastpath_fastsleep(is_rtc_only);
#endif
	return fastsleep;
}

/******************************************************************************
 *
 * INIT
 *
 ******************************************************************************/

static struct platform_suspend_ops sirloin_pm_ops = {
	.valid		= omap3_pm_sirloin_valid,
	.prepare	= omap3_pm_sirloin_prepare,
	.enter		= omap3_pm_sirloin_enter,
	.finish		= omap3_pm_sirloin_finish,
	.fastsleep  = omap3_pm_sirloin_fastsleep,
};

static int __init omap3_sirloin_pm_init(void)
{
	int rc;

	rc = omap3_pm_register_target_pm_ops(&sirloin_pm_ops);
	if (rc) {
		printk(KERN_ERR "OMAP3 PM: FAILED to register Sirloin target.\n");
		return -1;
	}

	printk(KERN_INFO "OMAP3: PM: Sirloin target registered successfully.\n");
	return 0;
}

__initcall(omap3_sirloin_pm_init);
#endif /* CONFIG_PM */
