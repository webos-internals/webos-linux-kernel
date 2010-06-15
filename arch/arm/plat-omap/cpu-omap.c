/*
 *  linux/arch/arm/plat-omap/cpu-omap.c
 *
 *  CPU frequency scaling for OMAP
 *
 *  Copyright (C) 2005 Nokia Corporation
 *  Written by Tony Lindgren <tony@atomide.com>
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/arch/clock.h>

#if defined(CONFIG_ARCH_OMAP3) && defined(CONFIG_OMAP3_PM)
#include <asm/arch/prcm.h>
#endif

#define VERY_HI_RATE	900000000

#ifdef CONFIG_CPU_FREQ_TABLE
static struct cpufreq_frequency_table *freq_table;
#endif

#ifdef CONFIG_ARCH_OMAP1
#define MPU_CLK		"mpu"
#elif defined(CONFIG_ARCH_OMAP3)
#define MPU_CLK		"virt_vdd1_prcm_set"
#else
#define MPU_CLK		"virt_prcm_set"
#endif

static struct clk *mpu_clk;

#if defined(CONFIG_ARCH_OMAP3) && defined(CONFIG_OMAP3_PM)
static struct constraint_handle *vdd1_handle;

static int cpufreq_pre_func(struct notifier_block *n, unsigned long event, void *ptr);
static int cpufreq_post_func(struct notifier_block *n, unsigned long event, void *ptr);
static struct notifier_block cpufreq_pre = {
	cpufreq_pre_func,
	NULL,
};
static struct notifier_block cpufreq_post = {
	cpufreq_post_func,
	NULL,
};
static struct constraint_id cnstr_id_vdd1 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};

#endif /* CONFIG_ARCH_OMAP3 && CONFIG_OMAP3_PM */

/* TODO: Add support for SDRAM timing changes */

static int omap_verify_speed(struct cpufreq_policy *policy)
{
#ifdef CONFIG_CPU_FREQ_TABLE
	if (freq_table)
		return cpufreq_frequency_table_verify(policy, freq_table);
#endif

	if (policy->cpu)
		return -EINVAL;

	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);

	policy->min = clk_round_rate(mpu_clk, policy->min * 1000) / 1000;
	policy->max = clk_round_rate(mpu_clk, policy->max * 1000) / 1000;
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int omap_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu)
		return 0;

	rate = clk_get_rate(mpu_clk) / 1000;
	return rate;
}

static int omap_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	struct cpufreq_freqs freqs;
	int ret = 0;

	/* Ensure desired rate is within allowed range.  Some govenors
	 * (ondemand) will just pass target_freq=0 to get the minimum. */
	if (target_freq < policy->cpuinfo.min_freq)
		target_freq = policy->cpuinfo.min_freq;
	if (target_freq > policy->cpuinfo.max_freq)
		target_freq = policy->cpuinfo.max_freq;

	freqs.old = omap_getspeed(0);
	freqs.new = clk_round_rate(mpu_clk, target_freq * 1000) / 1000;
	freqs.cpu = 0;

	if (freqs.old == freqs.new)
		return ret;

#ifdef CONFIG_ARCH_OMAP1
	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	ret = clk_set_rate(mpu_clk, freqs.new * 1000);
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
#elif defined(CONFIG_ARCH_OMAP3) && defined(CONFIG_OMAP3_PM)
	constraint_set(vdd1_handle,
			get_vdd1_arm_constraint_for_freq(freqs.new/1000));
#endif
	return ret;
}

#if defined(CONFIG_ARCH_OMAP3) && defined(CONFIG_OMAP3_PM)
static struct cpufreq_freqs freqs_notify;
static int cpufreq_pre_func(struct notifier_block *n, unsigned long event, void *ptr)
{
	freqs_notify.old = omap_getspeed(0);
	freqs_notify.new = get_arm_freq_for_opp(event);
	cpufreq_notify_transition(&freqs_notify, CPUFREQ_PRECHANGE);
	return 0;
}

static int cpufreq_post_func(struct notifier_block *n, unsigned long event, void *ptr)
{
	cpufreq_notify_transition(&freqs_notify, CPUFREQ_POSTCHANGE);
	return 0;
}
#endif

static int __init omap_cpu_init(struct cpufreq_policy *policy)
{
	printk("OMAP cpufreq driver initializing.\n");

	mpu_clk = clk_get(NULL, MPU_CLK);
	if (IS_ERR(mpu_clk)) {
		printk(KERN_ERR "OMAP cpufreq driver: Could not get clock resource "
				"for clock '%s'\n", MPU_CLK);
		return PTR_ERR(mpu_clk);
	}

	if (policy->cpu != 0)
		return -EINVAL;

	policy->cur = policy->min = policy->max = omap_getspeed(0);
	policy->governor = CPUFREQ_DEFAULT_GOVERNOR;

#ifdef CONFIG_CPU_FREQ_TABLE
	clk_init_cpufreq_table(&freq_table);
	if (freq_table) {
		int result;

		result = cpufreq_frequency_table_cpuinfo(policy, freq_table);
		if (!result)
			cpufreq_frequency_table_get_attr(freq_table,
							policy->cpu);
	} else
#endif
	{
		policy->cpuinfo.min_freq = clk_round_rate(mpu_clk, 0) / 1000;
		policy->cpuinfo.max_freq = clk_round_rate(mpu_clk,
							 VERY_HI_RATE) / 1000;
	}
	policy->cpuinfo.transition_latency = 100000;
#if defined(CONFIG_ARCH_OMAP3) && defined(CONFIG_OMAP3_PM)
	/* Request for VDD1 OPP3 by default */
	vdd1_handle = constraint_get("cpufreq-drv", &cnstr_id_vdd1);
	constraint_set(vdd1_handle, CO_VDD1_OPP3);
	constraint_register_pre_notification(vdd1_handle, &cpufreq_pre,
							max_vdd1_opp+1);
	constraint_register_post_notification(vdd1_handle, &cpufreq_post,
							max_vdd1_opp+1);
#endif
	return 0;
}

static int omap_cpu_exit(struct cpufreq_policy *policy)
{
	clk_put(mpu_clk);
	return 0;
}

#ifdef CONFIG_CPU_FREQ_TABLE
static struct freq_attr *omap_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};
#endif

static struct cpufreq_driver omap_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= omap_verify_speed,
	.target		= omap_target,
	.get		= omap_getspeed,
	.init		= omap_cpu_init,
	.exit		= omap_cpu_exit,
	.name		= "omap",
#ifdef CONFIG_CPU_FREQ_TABLE
	.attr		= omap_cpufreq_attr,
#endif
};

static int __init omap_cpufreq_init(void)
{
	return cpufreq_register_driver(&omap_driver);
}

late_initcall(omap_cpufreq_init);
