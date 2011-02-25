/*
 * linux/arch/arm/mach-omap3pe/clock_tree.h
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Based on OMAP2 clock framework created by
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Based on OMAP2 clock framework created by
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Copyright (C) 2004 Nokia corporation
 * Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 * Based on clocks.h by Tony Lindgren, Gordon McNutt and RidgeRun, Inc
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
 *
 */

#ifndef _ARCH_ARM_MACH_OMAP3_CLOCK_TREE_H_
#define _ARCH_ARM_MACH_OMAP3_CLOCK_TREE_H_

#include <asm/arch/prcm.h>

#include "clock.h"


/******************************************************************************
 *
 * Prototypes for functions used below in clock tree table.
 *
 ******************************************************************************/

static void omap3_clk_recalc(struct clk *clk);
static void omap3_followparent_recalc(struct clk *clk);
static void omap3_propagate_rate(struct clk *clk);
static void omap3_table_recalc(struct clk *clk);
static long omap3_round_to_table_rate(struct clk *clk, unsigned long rate);
static int omap3_select_table_rate(struct clk *clk, unsigned long rate);

/*-------------------------------------------------------------------------
 * 34xx clock tree.
 *
 * NOTE:In many cases here we are assigning a 'default' parent. In many
 *  cases the parent is selectable. The get/set parent calls will also
 *  switch sources.
 *
 *  Several sources are given initial rates which may be wrong, this will
 *  be fixed up in the init func.
 *
 *  Things are broadly separated below by clock domains. It is
 *  noteworthy that most periferals have dependencies on multiple clock
 *  domains. Many get their interface clocks from the L4 domain, but get
 *  functional clocks from fixed sources or other core domain derived
 *  clocks.
 *-------------------------------------------------------------------------*/

#define SRC_SEL                 (1 << 9)        /* Source of the clock
						*can be changed */
#define VDD1_CONFIG_PARTICIPANT (1 << 10)       /* Fundamental clock */
#define VDD2_CONFIG_PARTICIPANT (1 << 11)       /* Fundamental clock */
#define F_CLK                   (1 << 12)       /* Functional clock */
#define I_CLK                   (1 << 13)       /* Interface clock */
#define DPLL_OUTPUT             (1 << 14)       /* DPLL output */
#define POWER_ON_REQUIRED       (1 << 28)       /* For devices which
						*need to be powered on */

struct vdd_prcm_config {
	unsigned long speed;
	unsigned long opp;
	unsigned char flags;
};

#ifdef CONFIG_MACH_SIRLOIN_3630
static struct vdd_prcm_config vdd1_rate_table[MAX_VDD1_OPP +1] = {
	{0, 0, 0},
	/*OPP1*/
	{S150M, PRCM_VDD1_OPP1, RATE_IN_343X},
	/*OPP2*/
	{S300M, PRCM_VDD1_OPP2, RATE_IN_343X},
	/*OPP3*/
	{S600M, PRCM_VDD1_OPP3, RATE_IN_343X},
	/*OPP4*/
	{S800M, PRCM_VDD1_OPP4, RATE_IN_343X},
	/*OPP5*/
	{S1000M, PRCM_VDD1_OPP5, RATE_IN_343X},
};

static struct vdd_prcm_config vdd2_rate_table[MAX_VDD2_OPP +1] = {
	{0, 0, 0},
	/*OPP1*/
	{S19M, PRCM_VDD2_OPP1, RATE_IN_343X},
	/*OPP2*/
	{S100M, PRCM_VDD2_OPP2, RATE_IN_343X},
	/*OPP3*/
	{S200M, PRCM_VDD2_OPP3, RATE_IN_343X},
};
#else
static struct vdd_prcm_config vdd1_rate_table[MAX_VDD1_OPP +1] = {
	{0, 0, 0},
	/*OPP1*/
	{S125M, PRCM_VDD1_OPP1, RATE_IN_343X},
	/*OPP2*/
	{S250M, PRCM_VDD1_OPP2, RATE_IN_343X},
	/*OPP3*/
	{S500M, PRCM_VDD1_OPP3, RATE_IN_343X},
	/*OPP4*/
	{S550M, PRCM_VDD1_OPP4, RATE_IN_343X},
	/*OPP5*/
	{S600M, PRCM_VDD1_OPP5, RATE_IN_343X},
};

static struct vdd_prcm_config vdd2_rate_table[MAX_VDD2_OPP +1] = {
	{0, 0, 0},
	/*OPP1*/
	{S19M, PRCM_VDD2_OPP1, RATE_IN_343X},
	/*OPP2*/
	{S83M, PRCM_VDD2_OPP2, RATE_IN_343X},
	/*OPP3*/
	{S166M, PRCM_VDD2_OPP3, RATE_IN_343X},
};
#endif

/* Base external input clocks */
/* 32K clock */
static struct clk omap_32k_fck = {
	.name = "omap_32k_fck",
	.prcmid = PRCM_SYS_32K_CLK,
	.rate = S_32K,
	.flags = CLOCK_IN_OMAP343X | RATE_FIXED |
	    ALWAYS_ENABLED | RATE_PROPAGATES,
	.recalc = &omap3_propagate_rate,
};

static struct clk osc_sys_ck = {	/* (*12, *13, 19.2, *26, 38.4)MHz */
	.name = "osc_sys_ck",
	.rate = S_OSC,		/* fixed up in clock init */
	.flags = CLOCK_IN_OMAP343X | RATE_FIXED |
	    ALWAYS_ENABLED | RATE_PROPAGATES,
	.recalc = &omap3_propagate_rate,
};

static struct clk sys_ck = {	/* (*12, *13, 19.2, 26, 38.4)MHz */
	.name = "sys_ck",	/* ~ ref_clk also */
	.parent = &osc_sys_ck,
	.rate = S_OSC,
	.prcmid = PRCM_SYS_CLK,
	.flags = CLOCK_IN_OMAP343X | RATE_FIXED |
	    ALWAYS_ENABLED | RATE_PROPAGATES,
	.recalc = &omap3_clk_recalc,
};

static struct clk sysaltck = {
	.name = "sysaltck",
	.rate = S_ALT,
	.prcmid = PRCM_SYS_ALT_CLK,
	.flags = CLOCK_IN_OMAP343X | RATE_FIXED |
	    ALWAYS_ENABLED | RATE_PROPAGATES,
	.recalc = &omap3_propagate_rate,
};

static struct clk usim_fck = {
	.name = "usim_fck",
	.parent = &sys_ck,
	.prcmid = PRCM_USIM,
	.flags = CLOCK_IN_OMAP343X | F_CLK | RATE_CKCTL | SRC_SEL,
	.recalc = &omap3_clk_recalc,
};

static struct clk usim_ick = {
	.name = "usim_ick",
	.parent = &sys_ck,
	.prcmid = PRCM_USIM,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk core_ck = {	/* Same as DPLL3_M2_CLK */
	.name = "core_ck",
	.parent = &sys_ck,
	.prcmid = PRCM_DPLL3_M2_CLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | DPLL_OUTPUT | VDD2_CONFIG_PARTICIPANT,
	.recalc = &omap3_clk_recalc,
};

static struct clk core_x2_ck = {	/* Same as DPLL3_M2X2_CLK */
	.name = "core_x2_ck",
	.parent = &sys_ck,
	.prcmid = PRCM_DPLL3_M2X2_CLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | DPLL_OUTPUT | VDD2_CONFIG_PARTICIPANT,
	.recalc = &omap3_clk_recalc,
};

static struct clk cpefuse_fck = {
	.name = "cpefuse_fck",
	.parent = &sys_ck,
	.prcmid = PRCM_CPEFUSE,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk ts_fck = {
	.name = "ts_fck",
	.parent = &omap_32k_fck,
	.prcmid = PRCM_TS,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk emul_core_alwon_ck = {	/* Same as DPLL3_M3X2_CLK */
	.name = "emul_core_alwon_ck",
	.parent = &sys_ck,
	.prcmid = PRCM_DPLL3_M3X2_CLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | DPLL_OUTPUT,
	.recalc = &omap3_clk_recalc,
};

static struct clk cm_96m_fck = {	/* same as DPLL4_M2X2_CLK */
	.name = "cm_96m_fck",
	.parent = &sys_ck,
	.prcmid = PRCM_DPLL4_M2X2_CLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | DPLL_OUTPUT,
	.recalc = &omap3_clk_recalc,

};

static struct clk func_96m_ck = {
	.name = "func_96m_ck",
	.parent = &sys_ck,
	.prcmid = PRCM_96M_CLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | SRC_SEL,
	.recalc = &omap3_followparent_recalc,

};

static struct clk dpll4_m3x2_ck = {
	.name = "dpll4_m3x2_ck",
	.parent = &sys_ck,
	.prcmid = PRCM_DPLL4_M3X2_CLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | DPLL_OUTPUT,
	.recalc = &omap3_clk_recalc,
};

static struct clk emul_per_alwon_ck = {	/* same as DPLL4_M6X2_CLK */
	.name = "emul_per_alwon_ck",
	.parent = &sys_ck,
	.prcmid = PRCM_DPLL4_M6X2_CLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | DPLL_OUTPUT,
	.recalc = &omap3_clk_recalc,
};

static struct clk func_48m_ck = {
	.name = "func_48m_ck",
	.parent = &func_96m_ck,	/*can be sysaltck too */
	.prcmid = PRCM_48M_FCLK,
	.flags = CLOCK_IN_OMAP343X | RATE_PROPAGATES |
	    ALWAYS_ENABLED | RATE_CKCTL | SRC_SEL,
	.recalc = &omap3_clk_recalc,
};

static struct clk func_12m_ck = {
	.name = "func_12m_ck",
	.parent = &func_48m_ck,
	.prcmid = PRCM_12M_FCLK,
	.flags = CLOCK_IN_OMAP343X | RATE_PROPAGATES |
	    ALWAYS_ENABLED | RATE_CKCTL,
	.recalc = &omap3_clk_recalc,
};

static struct clk dss_tv_fck = {
	/* This node controls dss_96m_fck too */
	.name = "dss_tv_fck",
	.parent = &dpll4_m3x2_ck,	/*can be sysaltck too */
	.prcmid = PRCM_TVOUT,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

/* External output clocks */
static struct clk sys_clkout1 = {
/*May need to revisit this - sys_clkout1 can be output clock too */
	.name = "sys_clkout1",
	.parent = &osc_sys_ck,
	.prcmid = PRCM_SYS_CLKOUT1,
	.flags = CLOCK_IN_OMAP343X,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sys_clkout2 = {
	.name = "sys_clkout2",
	.parent = &sys_ck,
/* Can be core_ck,dss_tv_fck or func_96m_ck too */
	.prcmid = PRCM_SYS_CLKOUT2,
	.flags = CLOCK_IN_OMAP343X | RATE_CKCTL | SRC_SEL,
	.recalc = &omap3_clk_recalc,
};

static struct clk l3_ck = {
	.name = "l3_ck",
	.parent = &core_ck,
	.prcmid = PRCM_L3_ICLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | RATE_CKCTL | VDD2_CONFIG_PARTICIPANT,
	.recalc = &omap3_clk_recalc,
};

static struct clk gpmc_fck = {
	.name = "gpmc_fck",
	.parent = &l3_ck,
	.prcmid = PRCM_GPMC,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED | VDD2_CONFIG_PARTICIPANT,
	.recalc = &omap3_followparent_recalc,
};

static struct clk l4_ck = {
	.name = "l4_ck",
	.parent = &l3_ck,
	.prcmid = PRCM_L4_ICLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | RATE_CKCTL | VDD2_CONFIG_PARTICIPANT,
	.recalc = &omap3_clk_recalc,
};

static struct clk rm_ick = {
	.name = "rm_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_RM_ICLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | RATE_CKCTL | VDD2_CONFIG_PARTICIPANT,
	.recalc = &omap3_clk_recalc,
};

static struct clk dpll1_fck = {
	.name = "dpll1_fck",
	.parent = &core_ck,
	.prcmid = PRCM_DPLL1_FCLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | RATE_CKCTL,
	.recalc = &omap3_clk_recalc,
};

static struct clk dpll2_fck = {
	.name = "dpll2_fck",
	.parent = &core_ck,
	.prcmid = PRCM_DPLL2_FCLK,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | RATE_CKCTL,
	.recalc = &omap3_clk_recalc,
};

/* Clocks in MPU Power Domain */
static struct clk mpu_ck = {	/* Control cpu */
	.name = "mpu_ck",
	.parent = &sys_ck,
	.prcmid = DOM_MPU,
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED |
	    RATE_PROPAGATES | VDD1_CONFIG_PARTICIPANT,
	.recalc = &omap3_clk_recalc,
};

/* Clocks in IVA2 Power Domain */
static struct clk iva2_ck = {
	.name = "iva2_ck",
	.parent = &sys_ck,
	.prcmid = PRCM_IVA2,
	.flags = CLOCK_IN_OMAP343X | F_CLK |
	    RATE_PROPAGATES | VDD1_CONFIG_PARTICIPANT | POWER_ON_REQUIRED,
	.recalc = &omap3_clk_recalc,
};

/* GFX domain clocks */
/* In the GFX domain, GFX_L3_FCLK and GFX_L3_ICLK are derived
 from l3_iclk and have one gating control bit (CM_ICLKEN_GFX.EN_GFX).
 So another clock (gfx_l3_ck) is represented in the clock tree with
 two child clocks - gfx_fclk and gfx_iclk
 When either of these child clocks are enabled, the EN_GFX bit will
 be set to 1, making sure that the clock is supplied */

static struct clk sgx_fck = {
	.name = "sgx_fck",
	.parent = &core_ck,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | RATE_CKCTL |
		 POWER_ON_REQUIRED,
	.prcmid = PRCM_SGX_FCLK,
	.recalc = &omap3_clk_recalc,
};

static struct clk sgx_ick = {
	.name = "sgx_ick",
	.parent = &l3_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.prcmid = PRCM_SGX_ICLK,
	.recalc = &omap3_followparent_recalc,
};


static struct clk hsotgusb_ick = {
	.name = "hsotgusb_ick",
	.parent = &l3_ck,
	.prcmid = PRCM_HSOTG,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sdrc_ick = {
	.name = "sdrc_ick",
	.parent = &l3_ck,
	.prcmid = PRCM_SDRC,
	.flags = CLOCK_IN_OMAP343X | I_CLK | ALWAYS_ENABLED | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk pka_ick = {
	.name = "pka_ick",
	.parent = &l3_ck,
	.prcmid = PRCM_PKA,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk aes2_ick = {
	.name = "aes2_ick",
	.parent = &l3_ck,
	.prcmid = PRCM_AES2,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sha12_ick = {
	.name = "sha12_ick",
	.parent = &l3_ck,
	.prcmid = PRCM_SHA12,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk des2_ick = {
	.name = "des2_ick",
	.parent = &l3_ck,
	.prcmid = PRCM_DES2,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk aes1_ick = {
	.name = "aes1_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_AES1,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sha11_ick = {
	.name = "sha11_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_SHA11,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk des1_ick = {
	.name = "des1_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_DES1,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk rng_ick = {
	.name = "rng_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_RNG,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

/* New MCBSP clock definitions based on platform device ID. */
static struct clk mcbsp1_fck = {
	.name = "mcbsp_fck",
	.id = 1,
	.parent = &func_96m_ck,	/* Can also be external clock */
	.prcmid = PRCM_MCBSP1,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp1_ick = {
	.name = "mcbsp_ick",
	.id = 1,
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP1,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp2_fck = {
	.name = "mcbsp_fck",
	.id = 2,
	.parent = &func_96m_ck,	/*Can be external clock too */
	.prcmid = PRCM_MCBSP2,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp2_ick = {
	.name = "mcbsp_ick",
	.id = 2,
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP2,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp3_fck = {
	.name = "mcbsp_fck",
	.id = 3,
	.parent = &func_96m_ck,	/* Can be external clock too */
	.prcmid = PRCM_MCBSP3,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp3_ick = {
	.name = "mcbsp_ick",
	.id = 3,
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP3,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp4_fck = {
	.name = "mcbsp_fck",
	.id = 4,
	.parent = &func_96m_ck,	/* can be external clock too */
	.prcmid = PRCM_MCBSP4,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp4_ick = {
	.name = "mcbsp_ick",
	.id = 4,
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP4,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp5_fck = {
	.name = "mcbsp_fck",
	.id = 5,
	.parent = &func_96m_ck,	/* Can be external clock too */
	.prcmid = PRCM_MCBSP5,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp5_ick = {
	.name = "mcbsp_ick",
	.id = 5,
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP5,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

/* Old MCBSP clock definitions based on clk ID names.
 * Note:
 *   These definitions do not interfere with the new platform_device ID based
 *   definitions.
 */
static struct clk mcbsp1_fck_old = {
	.name = "mcbsp1_fck",
	.parent = &func_96m_ck,	/* Can also be external clock */
	.prcmid = PRCM_MCBSP1,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp1_ick_old = {
	.name = "mcbsp1_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP1,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp2_fck_old = {
	.name = "mcbsp2_fck",
	.parent = &func_96m_ck,	/*Can be external clock too */
	.prcmid = PRCM_MCBSP2,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp2_ick_old = {
	.name = "mcbsp2_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP2,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp3_fck_old = {
	.name = "mcbsp3_fck",
	.parent = &func_96m_ck,	/* Can be external clock too */
	.prcmid = PRCM_MCBSP3,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp3_ick_old = {
	.name = "mcbsp3_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP3,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp4_fck_old = {
	.name = "mcbsp4_fck",
	.parent = &func_96m_ck,	/* can be external clock too */
	.prcmid = PRCM_MCBSP4,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp4_ick_old = {
	.name = "mcbsp4_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP4,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp5_fck_old = {
	.name = "mcbsp5_fck",
	.parent = &func_96m_ck,	/* Can be external clock too */
	.prcmid = PRCM_MCBSP5,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcbsp5_ick_old = {
	.name = "mcbsp5_ick",
	.parent = &l4_ck,
	.prcmid = PRCM_MCBSP5,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};


static struct clk mmchs1_fck = {
	.name = "mmchs_fck",
	.id = 1,
	.prcmid = PRCM_MMC1,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mmchs1_ick = {
	.name = "mmchs_ick",
	.id = 1,
	.prcmid = PRCM_MMC1,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mmchs2_fck = {
	.name = "mmchs_fck",
	.id = 2,
	.prcmid = PRCM_MMC2,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mmchs2_ick = {
	.name = "mmchs_ick",
	.id = 2,
	.prcmid = PRCM_MMC2,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mmchs3_fck = {
	.name = "mmchs_fck",
	.id = 3,
	.prcmid = PRCM_MMC3,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mmchs3_ick = {
	.name = "mmchs_ick",
	.id = 3,
	.prcmid = PRCM_MMC3,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mspro_ick = {
	.name = "mspro_ick",
	.prcmid = PRCM_MSPRO,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mspro_fck = {
	.name = "mspro_fck",
	.prcmid = PRCM_MSPRO,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk i2c1_fck = {
	.name = "i2c_fck",
	.id = 1,
	.prcmid = PRCM_I2C1,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk i2c1_ick = {
	.name = "i2c_ick",
	.id = 1,
	.prcmid = PRCM_I2C1,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk i2c2_fck = {
	.name = "i2c_fck",
	.id = 2,
	.prcmid = PRCM_I2C2,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk i2c2_ick = {
	.name = "i2c_ick",
	.id = 2,
	.prcmid = PRCM_I2C2,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk i2c3_fck = {
	.name = "i2c_fck",
	.id = 3,
	.prcmid = PRCM_I2C3,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk i2c3_ick = {
	.name = "i2c_ick",
	.id = 3,
	.prcmid = PRCM_I2C3,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};


static struct clk omap_120m_fck = {
	.name = "omap_120m_fck",
	.parent	= &sys_ck,
	.prcmid = PRCM_DPLL5_M2_CLK,
	.flags = CLOCK_IN_OMAP343X | RATE_PROPAGATES | DPLL_OUTPUT,
	.recalc = &omap3_clk_recalc,
};

static struct clk usbhost_ick = {
	.name = "usbhost_ick",
	.parent = &l3_ck,
	.prcmid = PRCM_USBHOST1,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk usbhost_48m_fck = {
	.name = "usbhost_48m_fck",
	.parent = &func_48m_ck,
	.prcmid = PRCM_USBHOST1,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk usbhost_120m_fck = {
	.name = "usbhost_120m_fck",
	.parent = &omap_120m_fck,
	.prcmid = PRCM_USBHOST2,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk usbtll_fck = {
	.name = "usbtll_fck",
	.prcmid = PRCM_USBTLL,
	.parent = &omap_120m_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};


static struct clk usbtll_ick = {
	.name = "usbtll_ick",
	.prcmid = PRCM_USBTLL,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk uart1_fck = {
	.name = "uart1_fck",
	.prcmid = PRCM_UART1,
	.parent = &func_48m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk uart1_ick = {
	.name = "uart1_ick",
	.prcmid = PRCM_UART1,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk uart2_fck = {
	.name = "uart2_fck",
	.prcmid = PRCM_UART2,
	.parent = &func_48m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk uart2_ick = {
	.name = "uart2_ick",
	.prcmid = PRCM_UART2,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi1_fck = {
	.name = "mcspi_fck",
	.id = 1,
	.prcmid = PRCM_MCSPI1,
	.parent = &func_48m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi1_ick = {
	.name = "mcspi_ick",
	.id = 1,
	.prcmid = PRCM_MCSPI1,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi2_fck = {
	.name = "mcspi_fck",
	.id = 2,
	.prcmid = PRCM_MCSPI2,
	.parent = &func_48m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi2_ick = {
	.name = "mcspi_ick",
	.id = 2,
	.prcmid = PRCM_MCSPI2,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi3_fck = {
	.name = "mcspi_fck",
	.id = 3,
	.prcmid = PRCM_MCSPI3,
	.parent = &func_48m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi3_ick = {
	.name = "mcspi_ick",
	.id = 3,
	.prcmid = PRCM_MCSPI3,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi4_fck = {
	.name = "mcspi_fck",
	.id = 4,
	.prcmid = PRCM_MCSPI4,
	.parent = &func_48m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mcspi4_ick = {
	.name = "mcspi_ick",
	.id = 4,
	.prcmid = PRCM_MCSPI4,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk hdq_fck = {
	.name = "hdq_fck",
	.prcmid = PRCM_HDQ,
	.parent = &func_12m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk hdq_ick = {
	.name = "hdq_ick",
	.prcmid = PRCM_HDQ,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt10_fck = {
	.name = "gpt10_fck",
	.prcmid = PRCM_GPT10,
	/* Can be omap_32k_fck too */
	.parent = &sys_ck,
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt10_ick = {
	.name = "gpt10_ick",
	.prcmid = PRCM_GPT10,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt11_fck = {
	.name = "gpt11_fck",
	.prcmid = PRCM_GPT11,
	/* Can be omap_32k_fck too */
	.parent = &sys_ck,
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt11_ick = {
	.name = "gpt11_ick",
	.prcmid = PRCM_GPT11,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk omapctrl_ick = {
	.name = "omapctrl_ick",
	.prcmid = PRCM_OMAP_CTRL,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk mailboxes_ick = {
	.name = "mailboxes_ick",
	.prcmid = PRCM_MBOXES,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk ssi_ick = {
	.name = "ssi_ick",
	.prcmid = PRCM_SSI,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk ssi_ssr_sst_fck = {
/* This clock node is used to represent both ssi_ssr_fclk
 * and ssi_sst_fclk since there is one enable bit for both */
	.name = "ssi_ssr_sst_fck",
	.prcmid = PRCM_SSI,
	.parent = &core_x2_ck,
	/*Rate of ssi_sst = ssi_ssr rate/2 */
	.flags = CLOCK_IN_OMAP343X | RATE_CKCTL | F_CLK,
	.recalc = &omap3_clk_recalc,
};

/* DSS domain clocks */
static struct clk dss1_alwon_fck = {
	.name = "dss1_alwon_fck",
	.prcmid = PRCM_DSS,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | DPLL_OUTPUT | POWER_ON_REQUIRED,
	.recalc = &omap3_clk_recalc,
};

static struct clk dss_96m_fck = {
	.name = "dss_96m_fck",
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | RATE_PROPAGATES,
	.recalc = &omap3_followparent_recalc,
};

static struct clk dss2_alwon_fck = {
	.name = "dss2_alwon_fck",
	.prcmid = PRCM_DSS2,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk dss_ick = {
	/* This node models both DSS_L3 and DSS_L4 clocks */
	.name = "dss_ick",
	.prcmid = PRCM_DSS,
	.parent = &l3_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk cam_ick = {
	/* Models both CAM_L3 and CAM_L4 clocks */
	.name = "cam_ick",
	.prcmid = PRCM_CAM,
	.parent = &l3_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk cam_mclk = {
	.name = "cam_mclk",
	.prcmid = PRCM_CAM,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | DPLL_OUTPUT | POWER_ON_REQUIRED,
	.recalc = &omap3_clk_recalc,
};

static struct clk csi2_fck = {
	.name = "csi2_fck",
	.prcmid = PRCM_CSI2,
	.parent = &func_96m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt1_ick = {
	.name = "gpt1_ick",
	.prcmid = PRCM_GPT1,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt1_fck = {
	.name = "gpt1_fck",
	.prcmid = PRCM_GPT1,
	/* Can be sys_ck too */
	.parent = &omap_32k_fck,
	.rate = S_32K,
	.flags = CLOCK_IN_OMAP343X | SRC_SEL | F_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sync_32k_ick = {
	.name = "omap_32ksync_ick",
	.prcmid = PRCM_32KSYNC,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sync_32k_fck = {
	.name = "sync_32k_fck",
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X,
	.recalc = &omap3_followparent_recalc,
};

static struct clk wdt2_ick = {
	.name = "wdt_ick",
	.id = 2,
	.prcmid = PRCM_WDT2,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk wdt2_fck = {
	.name = "wdt_fck",
	.id = 2,
	.prcmid = PRCM_WDT2,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio1_fck = {
	.name = "gpio1_fck",
	.prcmid = PRCM_GPIO1,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio1_ick = {
	.name = "gpio1_ick",
	.prcmid = PRCM_GPIO1,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt12_ick = {
	.name = "gpt12_ick",
	.prcmid = PRCM_GPT12,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt12_fck = {
	.name = "gpt12_fck",
	.parent = &omap_32k_fck,
	.rate = S_32K,
	/* No s/w control for this clock */
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk wdt1_ick = {
	.name = "wdt_ick",
	.id = 1,
	.prcmid = PRCM_WDT1,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk wdt1_fck = {
	.name = "wdt_fck",
	.id = 1,
	.parent = &omap_32k_fck,
	/* No s/w control for this clock */
	.flags = CLOCK_IN_OMAP343X | ALWAYS_ENABLED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk uart3_ick = {
	.name = "uart3_ick",
	.prcmid = PRCM_UART3,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk uart3_fck = {
	.name = "uart3_fck",
	.prcmid = PRCM_UART3,
	.parent = &func_48m_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk wdt3_fck = {
	.name = "wdt_fck",
	.id = 3,
	.prcmid = PRCM_WDT3,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk wdt3_ick = {
	.name = "wdt_ick",
	.id = 3,
	.prcmid = PRCM_WDT3,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio2_fck = {
	.name = "gpio2_fck",
	.prcmid = PRCM_GPIO2,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio2_ick = {
	.name = "gpio2_ick",
	.prcmid = PRCM_GPIO2,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio3_fck = {
	.name = "gpio3_fck",
	.prcmid = PRCM_GPIO3,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio3_ick = {
	.name = "gpio3_ick",
	.prcmid = PRCM_GPIO3,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio4_fck = {
	.name = "gpio4_fck",
	.prcmid = PRCM_GPIO4,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio4_ick = {
	.name = "gpio4_ick",
	.prcmid = PRCM_GPIO4,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio5_fck = {
	.name = "gpio5_fck",
	.prcmid = PRCM_GPIO5,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio5_ick = {
	.name = "gpio5_ick",
	.prcmid = PRCM_GPIO5,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio6_fck = {
	.name = "gpio6_fck",
	.prcmid = PRCM_GPIO6,
	.parent = &omap_32k_fck,
	.flags = CLOCK_IN_OMAP343X | F_CLK | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpio6_ick = {
	.name = "gpio6_ick",
	.prcmid = PRCM_GPIO6,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt2_ick = {
	.name = "gpt2_ick",
	.prcmid = PRCM_GPT2,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt2_fck = {
	.name = "gpt2_fck",
	.prcmid = PRCM_GPT2,
	.parent = &sys_ck,	/* can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt3_ick = {
	.name = "gpt3_ick",
	.prcmid = PRCM_GPT3,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt3_fck = {
	.name = "gpt3_fck",
	.prcmid = PRCM_GPT3,
	.parent = &sys_ck,	/* Can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt4_ick = {
	.name = "gpt4_ick",
	.prcmid = PRCM_GPT4,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt4_fck = {
	.name = "gpt4_fck",
	.prcmid = PRCM_GPT4,
	.parent = &sys_ck,	/* Can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt5_ick = {
	.name = "gpt5_ick",
	.prcmid = PRCM_GPT5,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt5_fck = {
	.name = "gpt5_fck",
	.prcmid = PRCM_GPT5,
	.parent = &sys_ck,	/* Can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt6_ick = {
	.name = "gpt6_ick",
	.prcmid = PRCM_GPT6,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt6_fck = {
	.name = "gpt6_fck",
	.prcmid = PRCM_GPT6,
	.parent = &sys_ck,	/* Can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt7_ick = {
	.name = "gpt7_ick",
	.prcmid = PRCM_GPT7,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt7_fck = {
	.name = "gpt7_fck",
	.prcmid = PRCM_GPT7,
	.parent = &sys_ck,	/* Can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt8_ick = {
	.name = "gpt8_ick",
	.prcmid = PRCM_GPT8,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt8_fck = {
	.name = "gpt8_fck",
	.prcmid = PRCM_GPT8,
	.parent = &sys_ck,	/* Can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt9_ick = {
	.name = "gpt9_ick",
	.prcmid = PRCM_GPT9,
	.parent = &l4_ck,
	.flags = CLOCK_IN_OMAP343X | I_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk gpt9_fck = {
	.name = "gpt9_fck",
	.prcmid = PRCM_GPT9,
	.parent = &sys_ck,	/* Can be omap_32k_fck too */
	.rate = S_OSC,
	.flags = CLOCK_IN_OMAP343X | F_CLK | SRC_SEL | POWER_ON_REQUIRED,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sr1_fck = {
	.name = "sr1_fck",
	.prcmid = PRCM_SR1,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK,
	.recalc = &omap3_followparent_recalc,
};

static struct clk sr2_fck = {
	.name = "sr2_fck",
	.prcmid = PRCM_SR2,
	.parent = &sys_ck,
	.flags = CLOCK_IN_OMAP343X | F_CLK,
	.recalc = &omap3_followparent_recalc,
};

/* This node is used to model the external mcbsp clock*/
/* Rate of the clock is supposed to be set using clk_set_rate API*/
static struct clk ext_mcbsp_ck = {
	.name = "ext_mcbsp_ck",
	.prcmid = PRCM_EXT_MCBSP_CLK,
	.rate = S96M,
	.flags = CLOCK_IN_OMAP343X | RATE_PROPAGATES,
	.recalc = &omap3_propagate_rate,
};

static struct clk virt_vdd1_prcm_set = {
	.name = "virt_vdd1_prcm_set",
	.flags = CLOCK_IN_OMAP343X | VIRTUAL_CLOCK | ALWAYS_ENABLED,
	.parent = &mpu_ck,	/* Indexed by mpu speed, no parent */
	.recalc = &omap3_table_recalc,	/* sets are keyed on mpu rate */
	.set_rate = &omap3_select_table_rate,
	.round_rate = &omap3_round_to_table_rate,
};

static struct clk virt_vdd2_prcm_set = {
	.name = "virt_vdd2_prcm_set",
	.flags = CLOCK_IN_OMAP343X | VIRTUAL_CLOCK | ALWAYS_ENABLED,
	.parent = &core_ck,	/* Indexed by core speed, no parent */
	.recalc = &omap3_table_recalc,	/* sets are keyed on mpu rate */
	.set_rate = &omap3_select_table_rate,
	.round_rate = &omap3_round_to_table_rate,
};

static struct clk *onchip_clks[] = {
	/* external root sources */
	&omap_32k_fck,
	&osc_sys_ck,
	&sys_ck,
	&sysaltck,
	/* internal sources */
	&core_ck,
	&core_x2_ck,
	&emul_core_alwon_ck,
	&cm_96m_fck,
	&func_96m_ck,
	&dpll4_m3x2_ck,
	&emul_per_alwon_ck,
	&func_48m_ck,
	&func_12m_ck,
	&dss_tv_fck,
	&sys_clkout1,
	&sys_clkout2,
	&l3_ck,
	&gpmc_fck,
	&l4_ck,
	&rm_ick,
	&dpll1_fck,
	&dpll2_fck,
	&mpu_ck,
	&iva2_ck,
	&usim_fck,
	&usim_ick,
	&cpefuse_fck,
	&ts_fck,
	&sgx_fck,
	&sgx_ick,
	&hsotgusb_ick,
	&sdrc_ick,
	&pka_ick,
	&aes2_ick,
	&sha12_ick,
	&des2_ick,
	&aes1_ick,
	&sha11_ick,
	&des1_ick,
	&rng_ick,
	&mcbsp1_fck,
	&mcbsp1_ick,
	&mcbsp2_fck,
	&mcbsp2_ick,
	&mcbsp3_fck,
	&mcbsp3_ick,
	&mcbsp4_fck,
	&mcbsp4_ick,
	&mcbsp5_fck,
	&mcbsp5_ick,
	&mcbsp1_fck_old,
	&mcbsp1_ick_old,
	&mcbsp2_fck_old,
	&mcbsp2_ick_old,
	&mcbsp3_fck_old,
	&mcbsp3_ick_old,
	&mcbsp4_fck_old,
	&mcbsp4_ick_old,
	&mcbsp5_fck_old,
	&mcbsp5_ick_old,
	&mmchs1_fck,
	&mmchs1_ick,
	&mmchs2_fck,
	&mmchs2_ick,
	&mmchs3_fck,
	&mmchs3_ick,
	&mspro_fck,
	&mspro_ick,
	&i2c1_fck,
	&i2c1_ick,
	&i2c2_fck,
	&i2c2_ick,
	&i2c3_fck,
	&i2c3_ick,
	&omap_120m_fck,
	&usbhost_ick,
	&usbhost_48m_fck,
	&usbhost_120m_fck,
	&usbtll_fck,
	&usbtll_ick,
	&uart1_fck,
	&uart1_ick,
	&uart2_fck,
	&uart2_ick,
	&mcspi1_fck,
	&mcspi1_ick,
	&mcspi2_fck,
	&mcspi2_ick,
	&mcspi3_fck,
	&mcspi3_ick,
	&mcspi4_fck,
	&mcspi4_ick,
	&hdq_fck,
	&hdq_ick,
	&gpt10_fck,
	&gpt10_ick,
	&gpt11_fck,
	&gpt11_ick,
	&omapctrl_ick,
	&mailboxes_ick,
	&ssi_ick,
	&ssi_ssr_sst_fck,
	&dss1_alwon_fck,
	&dss_96m_fck,
	&dss2_alwon_fck,
	&dss_ick,
	&cam_mclk,
	&cam_ick,
	&csi2_fck,
	&gpt1_fck,
	&gpt1_ick,
	&sync_32k_fck,
	&sync_32k_ick,
	&wdt2_fck,
	&wdt2_ick,
	&gpio1_fck,
	&gpio1_ick,
	&gpt12_fck,
	&gpt12_ick,
	&wdt1_fck,
	&wdt1_ick,
	&uart3_fck,
	&uart3_ick,
	&wdt3_fck,
	&wdt3_ick,
	&gpio2_fck,
	&gpio2_ick,
	&gpio3_fck,
	&gpio3_ick,
	&gpio4_fck,
	&gpio4_ick,
	&gpio5_fck,
	&gpio5_ick,
	&gpio6_fck,
	&gpio6_ick,
	&gpt2_fck,
	&gpt2_ick,
	&gpt3_fck,
	&gpt3_ick,
	&gpt4_fck,
	&gpt4_ick,
	&gpt5_fck,
	&gpt5_ick,
	&gpt6_fck,
	&gpt6_ick,
	&gpt7_fck,
	&gpt7_ick,
	&gpt8_fck,
	&gpt8_ick,
	&gpt9_fck,
	&gpt9_ick,
	&sr1_fck,
	&sr2_fck,
	/* External mcbsp clock */
	&ext_mcbsp_ck,
	/* virtual group clock */
	&virt_vdd1_prcm_set,
	&virt_vdd2_prcm_set,
};

#endif

