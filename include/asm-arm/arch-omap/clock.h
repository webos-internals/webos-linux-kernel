/*
 *  linux/include/asm-arm/arch-omap/clock.h
 *
 *  Copyright (C) 2004 - 2005 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *  Based on clocks.h by Tony Lindgren, Gordon McNutt and RidgeRun, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_OMAP_CLOCK_H
#define __ARCH_ARM_OMAP_CLOCK_H

#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#endif
#if defined(CONFIG_ARCH_OMAP34XX)
#include <asm/arch/resource.h>
#endif
#include <linux/clk.h>

struct clk {
#if defined(CONFIG_ARCH_OMAP34XX)
	struct res_handle *res;
#endif /* #if defined (CONFIG_ARCH_OMAP34XX) */
	struct list_head	node;
	struct module		*owner;
	const char		*name;
	int			id;
	struct clk		*parent;
	unsigned long		rate;
	__u32			flags;
	void __iomem		*enable_reg;
	__u8			enable_bit;
	__s8			usecount;
	void			(*recalc)(struct clk *);
	int			(*set_rate)(struct clk *, unsigned long);
	long			(*round_rate)(struct clk *, unsigned long);
	void			(*init)(struct clk *);
	int			(*enable)(struct clk *);
	void			(*disable)(struct clk *);
	__u32 prcmid;

#if 0
	u8			fixed_div;
	void __iomem		*clksel_reg;
	u32			clksel_mask;
	const struct clksel	*clksel;
	struct dpll_data	*dpll_data;
	const char              *clkdm_name;
	struct clockdomain      *clkdm;
#endif /* if 0 */
#if defined(CONFIG_ARCH_OMAP2)
	__u8			rate_offset;
	__u8			src_offset;
#endif
#if defined(CONFIG_PM_DEBUG) && defined(CONFIG_DEBUG_FS)
	struct dentry		*dent;	/* For visible tree hierarchy */
#endif
};

struct clk_functions {
	int		(*clk_enable)(struct clk *clk);
	void		(*clk_disable)(struct clk *clk);
	long		(*clk_round_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_rate)(struct clk *clk, unsigned long rate);
	int		(*clk_set_parent)(struct clk *clk, struct clk *parent);
	struct clk *	(*clk_get_parent)(struct clk *clk);
	void		(*clk_allow_idle)(struct clk *clk);
	void		(*clk_deny_idle)(struct clk *clk);
	void		(*clk_disable_unused)(struct clk *clk);
#ifdef CONFIG_CPU_FREQ
	void		(*clk_init_cpufreq_table)(struct cpufreq_frequency_table **);
#endif
};

extern unsigned int mpurate;

extern int clk_init(struct clk_functions *custom_clocks);
extern int clk_register(struct clk *clk);
extern void clk_unregister(struct clk *clk);
extern void propagate_rate(struct clk *clk);
extern void recalculate_root_clocks(void);
extern void followparent_recalc(struct clk *clk);
extern void clk_allow_idle(struct clk *clk);
extern void clk_deny_idle(struct clk *clk);
extern int clk_get_usecount(struct clk *clk);
#ifdef CONFIG_ARCH_OMAP34XX
extern void clk_enable_init_clocks(void);
#endif
#ifdef CONFIG_CPU_FREQ
extern void clk_init_cpufreq_table(struct cpufreq_frequency_table **table);
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
void context_save_done(struct clk *clk);
void save_scratchpad_contents(void);
#endif

void omap3_clk_prepare_for_reboot(void);

/* Clock flags */
#ifdef CONFIG_ARCH_OMAP3

#define RATE_CKCTL		(1 << 0)	/* Main fixed ratio clocks */
#define RATE_FIXED		(1 << 1)	/* Fixed clock rate */
#define RATE_PROPAGATES		(1 << 2)	/* Program children too */
#define VIRTUAL_CLOCK		(1 << 3)	/* Composite clock from table */
#define ALWAYS_ENABLED		(1 << 4)	/* Clock cannot be disabled */
#define ENABLE_REG_32BIT	(1 << 5)	/* Use 32-bit access */
#define VIRTUAL_IO_ADDRESS	(1 << 6)	/* Clock in virtual address */
#define CLOCK_IDLE_CONTROL	(1 << 7)
#define CLOCK_NO_IDLE_PARENT	(1 << 8)
#define DELAYED_APP		(1 << 9)	/* Delay application of clock */
#define CONFIG_PARTICIPANT	(1 << 10)	/* Fundamental clock */
#define ENABLE_ON_INIT		(1 << 11)	/* Enable upon framework init */
#define INVERT_ENABLE           (1 << 12)       /* 0 enables, 1 disables */
#define OFFSET_GR_MOD		(1 << 13)	/* 24xx GR_MOD reg as offset */

/* bits 13-20 are currently free */
#define CLOCK_IN_OMAP310	(1 << 21)
#define CLOCK_IN_OMAP730	(1 << 22)
#define CLOCK_IN_OMAP1510	(1 << 23)
#define CLOCK_IN_OMAP16XX	(1 << 24)
#define CLOCK_IN_OMAP242X	(1 << 25)
#define CLOCK_IN_OMAP243X	(1 << 26)
#define CLOCK_IN_OMAP343X	(1 << 27)	/* clocks common to all 343X */
#define PARENT_CONTROLS_CLOCK	(1 << 28)
#define CLOCK_IN_OMAP3430ES1	(1 << 29)	/* 3430ES1 clocks only */
#define CLOCK_IN_OMAP3430ES2	(1 << 30)	/* 3430ES2 clocks only */

#else

#define RATE_CKCTL		(1 << 0)	/* Rate of clock can be changed based on CLKSEL registers */
#define RATE_FIXED		(1 << 1)	/* Fixed clock rate */
#define RATE_PROPAGATES		(1 << 2)	/* Program children too */
#define VIRTUAL_CLOCK		(1 << 3)	/* Composite clock from table */
#define ALWAYS_ENABLED		(1 << 4)	/* Clock cannot be disabled */
#define ENABLE_REG_32BIT	(1 << 5)	/* Use 32-bit access */
#define VIRTUAL_IO_ADDRESS	(1 << 6)	/* Clock in virtual address */
#define CLOCK_IDLE_CONTROL	(1 << 7)
#define CLOCK_NO_IDLE_PARENT	(1 << 8)

#define DELAYED_APP		(1 << 9)	/* Delay application of clock */
#define CONFIG_PARTICIPANT	(1 << 10)	/* Fundamental clock */
#define CM_MPU_SEL1		(1 << 11)	/* Domain divider/source */
#define CM_DSP_SEL1		(1 << 12)
#define CM_GFX_SEL1		(1 << 13)
#define CM_MODEM_SEL1		(1 << 14)
#define CM_CORE_SEL1		(1 << 15)	/* Sets divider for many */
#define CM_CORE_SEL2		(1 << 16)	/* sets parent for GPT */
#define CM_WKUP_SEL1		(1 << 17)
#define CM_PLL_SEL1		(1 << 18)
#define CM_PLL_SEL2		(1 << 19)
#define CM_SYSCLKOUT_SEL1	(1 << 20)

#define CLOCK_IN_OMAP310	(1 << 21)
#define CLOCK_IN_OMAP730	(1 << 22)
#define CLOCK_IN_OMAP1510	(1 << 23)
#define CLOCK_IN_OMAP16XX	(1 << 24)
#define CLOCK_IN_OMAP242X	(1 << 25)
#define CLOCK_IN_OMAP243X	(1 << 26)
#define CLOCK_IN_OMAP343X	(1 << 27)

#define ENABLE_ON_INIT		(1 << 28)	/* Enable upon framework init */

#endif

/* Clksel_rate flags */
#define RATE_IN_242X	(1 << 0)
#define RATE_IN_243X	(1 << 1)
#define RATE_IN_343X	(1 << 2)

#endif
