/*
 *  linux/arch/arm/mach-omap3pe/clock.c
 *
 *  OMAP34XX clock framework
 *
 *  based on linux/arch/arm/mach-omap2/clock.c
 *
 *  Copyright (C) 2007 Texas Instruments Inc.
 *  Karthik Dasu <karthik-dp@ti.com>
 *
 *  Based on omap2 clock.c Copyright (C) 2005 Texas Instruments Inc
 *  Richard Woodruff <r-woodruff2@ti.com>
 *  Cleaned up and modified to use omap shared clock framework by
 *  Tony Lindgren <tony@atomide.com>
 *
 *  Based on omap1 clock.c, Copyright (C) 2004 - 2005 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>

#include <asm/io.h>

#include <asm/arch/clock.h>
#include <asm/arch/sram.h>

#include "prcm-regs.h"
#include "memory.h"
#include "clock_tree.h"
#include "prcm_opp.h"


#include <linux/timer.h>

//#define DEBUG_CLK 1
#ifdef DEBUG_CLK
#  define DPRINTK(fmt, args...) \
		printk(KERN_INFO "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
static DEFINE_SPINLOCK(inatimer_lock);

static struct timer_list coredomain_timer;
static struct timer_list perdomain_timer;

struct inactivity_timer{
	u32 start_time, value;
	u32 module_timeout_value[32];
	struct timer_list *off_timer;
};

static struct inactivity_timer domain_inatimer[2];

void modify_timeout_value(struct clk *clk, u32 value)
{
	u32 prcm_id, domain_id, device_id;
	struct inactivity_timer *inatimer;

	prcm_id = clk->prcmid;
	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);

	if (domain_id == DOM_CORE1) {
		inatimer = &domain_inatimer[0];
	} else if (domain_id == DOM_PER) {
		inatimer = &domain_inatimer[1];
	} else {
		static int printed = 0;
		if (!printed) {
			printk(KERN_WARNING "Inactivity timer not supported\n");
			printed++;
		}
		return;
	}
	inatimer->module_timeout_value[device_id] = value;
}
EXPORT_SYMBOL(modify_timeout_value);

static void modify_inatimer(struct clk *clk)
{
	u32 prcm_id, domain_id, device_id;
	u32 new_value, remaining_time;
	struct inactivity_timer *inatimer;

	prcm_id = clk->prcmid;

	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);

	spin_lock(&inatimer_lock);
	if (domain_id == DOM_CORE1) {
		inatimer = &domain_inatimer[0];
		inatimer->off_timer = &coredomain_timer;
	} else if (domain_id == DOM_PER) {
		inatimer = &domain_inatimer[1];
		inatimer->off_timer = &perdomain_timer;
	} else {
		static int printed = 0;
		if (!printed) {
			printk(KERN_WARNING "Inactivity timer not supported\n");
			printed++;
		}
		spin_unlock(&inatimer_lock);
		return;
	}

	new_value = inatimer->module_timeout_value[device_id];

	if (new_value != 0) {
		/* If timer pending, check the remaining time and restart
		* the timer if required
		*/
		if (timer_pending(inatimer->off_timer)) {
			remaining_time = inatimer->value -
			(((jiffies - inatimer->start_time) * 1000) / HZ);

			if (new_value > remaining_time) {
				mod_timer(inatimer->off_timer, jiffies +
						(new_value * HZ / 1000));
				inatimer->start_time = jiffies;
				inatimer->value = new_value;
			}
		} else {

			mod_timer(inatimer->off_timer, jiffies +
						(new_value * HZ / 1000));
			inatimer->start_time = jiffies;
			inatimer->value = new_value;
		}
	}
	spin_unlock(&inatimer_lock);

}

int coredomain_timer_pending(void)
{
	return timer_pending(&coredomain_timer);
}
EXPORT_SYMBOL(coredomain_timer_pending);

static void coredomain_timer_func(unsigned long data)
{
	/* CORE domain timer function */
}

int perdomain_timer_pending(void)
{
	return timer_pending(&perdomain_timer);
}
EXPORT_SYMBOL(perdomain_timer_pending);

static void perdomain_timer_func(unsigned long data)
{
	/* PER domain timer function */
}
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
static int  enable_power_domain(struct clk *clk);
static void disable_power_domain(struct clk *clk);

#ifndef CONFIG_ARCH_OMAP3410
static char *id_to_name[] = {"iva2", "mpu", "core", "core", "sgx", NULL, "dss",
				"cam", "per", NULL, "neon", "core", "usb"};
#else
static char *id_to_name[] = {"iva2", "mpu", "core", "core", NULL, NULL, "dss",
				"cam", "per", NULL, NULL, "core", "usb"};
#endif

#endif /* CONFIG_AUTO_POWER_DOMAIN_CTRL */

static struct vdd_prcm_config *curr_vdd1_prcm_set;
static struct vdd_prcm_config *curr_vdd2_prcm_set;

#ifndef CONFIG_CPU_FREQ 
static unsigned long compute_lpj(unsigned long ref, u_int div, u_int mult)
{
	unsigned long new_jiffy_l, new_jiffy_h;

	/*
	 * Recalculate loops_per_jiffy.  We do it this way to
	 * avoid math overflow on 32-bit machines.  Maybe we
	 * should make this architecture dependent?  If you have
	 * a better way of doing this, please replace!
	 *
	 *    new = old * mult / div
	 */
	new_jiffy_h = ref / div;
	new_jiffy_l = (ref % div) / 100;
	new_jiffy_h *= mult;
	new_jiffy_l = new_jiffy_l * mult / div;

	return new_jiffy_h + new_jiffy_l * 100;
}
#endif
/*-------------------------------------------------------------------------
 * Omap3 specific clock functions
 *-------------------------------------------------------------------------*/
/* Make the rate same as that of the parent.
 If the rate propagates, call propagate_rate*/
static void omap3_followparent_recalc(struct clk *clk)
{
	followparent_recalc(clk);
	if (clk->flags & RATE_PROPAGATES)
		propagate_rate(clk);
}

static void omap3_propagate_rate(struct clk *clk)
{
	propagate_rate(clk);
}

/* Calls PRCM APIs to enable clock */
static int _omap3_clk_enable(struct clk *clk)
{
	int ret = 0;

	pr_debug("Clk: %s\n", clk->name);
	if (clk->flags & ALWAYS_ENABLED)
		return 0;

	if (clk == &sys_clkout1)
		ret = prcm_control_external_output_clock1(PRCM_ENABLE);

	if (clk == &sys_clkout2)
		ret = prcm_control_external_output_clock2(PRCM_ENABLE);

	if (clk->flags & F_CLK) {
#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
		ret = enable_power_domain(clk);
		if (ret)
			return ret;
#endif
		ret = prcm_clock_control(clk->prcmid, FCLK, PRCM_ENABLE,
				       PRCM_ACCESS_CHK);
	}
	if (clk->flags & I_CLK) {
#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
		ret = enable_power_domain(clk);
		if (ret)
			return ret;
#endif
		ret = prcm_clock_control(clk->prcmid, ICLK, PRCM_ENABLE,
				       PRCM_ACCESS_CHK);
	}

	pr_debug("Done Clk: %s\n", clk->name);
	if (ret == PRCM_FAIL)
		return -EINVAL;
	else
		return 0;
}

/* Calls PRCM APIs to disable clock */
static void _omap3_clk_disable(struct clk *clk)
{
	pr_debug("Clk: %s\n", clk->name);
	if (clk->flags & ALWAYS_ENABLED)
		return;

	if (clk == &sys_clkout1)
		prcm_control_external_output_clock1(PRCM_DISABLE);

	if (clk == &sys_clkout2)
		prcm_control_external_output_clock2(PRCM_DISABLE);

	if (clk->flags & F_CLK) {
		prcm_clock_control(clk->prcmid, FCLK, PRCM_DISABLE,
				   PRCM_NO_ACCESS_CHK);
#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
		disable_power_domain(clk);
#endif
	}

	if (clk->flags & I_CLK) {
		prcm_clock_control(clk->prcmid, ICLK, PRCM_DISABLE,
				   PRCM_NO_ACCESS_CHK);
#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
		disable_power_domain(clk);
#endif
	}

	pr_debug("Done Clk: %s\n", clk->name);
}

/* Update usecount and disable clock if usecount reaches 0 */
static void omap3_clk_disable(struct clk *clk)
{
	pr_debug("Name %s\n", clk->name);
	if (clk->usecount > 0 && !(--clk->usecount)) {
		_omap3_clk_disable(clk);
		if (likely((u32) clk->parent))
			omap3_clk_disable(clk->parent);
	}
	pr_debug("usecount: %d,%s\n", clk->usecount, clk->name);
}

/* Enable clock if it is not already enabled and update usecount */
static int omap3_clk_enable(struct clk *clk)
{
	int ret = 0;
	pr_debug("Name %s\n", clk->name);
	if (clk->usecount++ == 0) {
		if (likely((u32) clk->parent))
			ret = omap3_clk_enable(clk->parent);
		if (unlikely(ret != 0)) {
			clk->usecount--;
			return ret;
		}

		ret = _omap3_clk_enable(clk);

		if (unlikely(ret != 0) && clk->parent) {
			omap3_clk_disable(clk->parent);
			clk->usecount--;
		}
	}
	pr_debug("usecount: %d,%s\n", clk->usecount, clk->name);
	return ret;
}

/* Calls appropriate PRCM API to calculate rate of clock */
static void omap3_clk_recalc(struct clk *clk)
{
	u32 parent_rate, divider, ret, rate;
	parent_rate = clk->parent->rate;
	ret = PRCM_PASS;

	pr_debug("Clock name: %s\n", clk->name);

	if (clk == &sys_ck) {
		clk->rate = prcm_get_system_clock_speed() * 1000;
		ret = PRCM_PASS;
	}

	if (clk == &mpu_ck) {
		ret = prcm_get_processor_speed(clk->prcmid, &rate);
		clk->rate = rate * 1000;
	}

	if (clk == &iva2_ck) {
		ret = prcm_get_processor_speed(clk->prcmid, &rate);
		clk->rate = rate * 1000;
	}

	if (clk->flags & DPLL_OUTPUT) {
		ret = prcm_get_dpll_rate(clk->prcmid, &rate);
		if (ret == PRCM_PASS)
			clk->rate = rate * 1000;
	}

	if (clk->flags & RATE_CKCTL) {
		ret = prcm_clksel_get_divider(clk->prcmid, &divider);
		pr_debug("Divider: %d\n", divider);
		if (ret == PRCM_PASS)
			clk->rate = clk->parent->rate / divider;
	}

	if (ret != PRCM_PASS)
		printk(KERN_ERR "Error in clk_recalc: %d,%s\n", ret, clk->name);

	pr_debug("Rate: %lu\n", clk->rate);

	if (clk->flags & RATE_PROPAGATES)
		propagate_rate(clk);
}

/* Given a clock and a rate apply a clock specific rounding function */
/* This function should be called only for those clocks which have
 * dividers and which are not part of a clock configuration. In case
 * it is called for these clocks, it returns the current rate of the clocks */
static long omap3_clk_round_rate(struct clk *tclk, unsigned long rate)
{
	u32 new_div = 0, mnoutput;
	int ret;
	int valid_rate, parent_rate;

	pr_debug("Clock name: %s\n", tclk->name);

	if (tclk->flags & RATE_FIXED)
		return tclk->rate;

	/* For external mcbsp clock node, rate is supposed to be set by
	 * the corresponding device driver. So round rate will return
	 * the rate that is supposed to be set */
	if (tclk == &ext_mcbsp_ck)
		return rate;
	if ((tclk->flags & VDD1_CONFIG_PARTICIPANT)
	    || (tclk->flags & VDD2_CONFIG_PARTICIPANT))
	/* If the clock is part of a clock configuration, it cannot
	be changed on the fly */
		return tclk->rate;

	if (tclk->flags & DPLL_OUTPUT) {
		/* The only clocks for which DPLL OUTPUT can be changed on
		 * the fly (by changing only MX dividers) are:
		 * PRCM_DPLL3_M3X2_CLK, PRCM_DPLL4_M2X2_CLK,
		 *PRCM_DPLL4_M3X2_CLK, PRCM_DPLL4_M4X2_CLK,
		 *PRCM_DPLL4_M5X2_CLK, PRCM_DPLL4_M6X2_CLK */
		ret = prcm_get_dpll_mn_output(tclk->prcmid, &mnoutput);
		/*mnoutput has CLKOUT = (Freq*m)/n+1 in case dpll is locked
		 *or bypass clock if it is not locked */
		if (ret != PRCM_PASS)
			/* Rate cannot be changed. Return original rate*/
			return tclk->rate;
		ret =
		    prcm_clksel_round_rate(tclk->prcmid, (2 * mnoutput),
					   (rate / 1000), &new_div);
		if (ret == PRCM_PASS) {
			valid_rate = (2 * mnoutput * 1000) / new_div;
			pr_debug("Valid rate: %d\n", valid_rate);
			return valid_rate;
		} else
			/* No acceptable divider - return original rate */
			return tclk->rate;
	}

	if (tclk->flags & RATE_CKCTL) {
		parent_rate = tclk->parent->rate;
		ret =
		    prcm_clksel_round_rate(tclk->prcmid, parent_rate,
						   rate, &new_div);
		if (ret == PRCM_PASS)
			return parent_rate / new_div;
		else
			/* No acceptable divider - return original rate */
			return tclk->rate;
	}

	if (tclk->round_rate != 0)
		return tclk->round_rate(tclk, rate);

	return tclk->rate;
}

/* Set the clock rate for a clock source */
static int omap3_clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;
	u32 validrate, parent_rate, mnoutput;
	u32 new_div = 0;

	pr_debug("Clock name: %s\n", clk->name);

	/* For external mcbsp clock, the driver can set the rate using*/
	/* clk_set_rate API */
	if (clk == &ext_mcbsp_ck) {
		clk->rate = rate;
		clk->recalc(clk);
		return 0;
	}

	if (!(clk->flags & VDD1_CONFIG_PARTICIPANT)
	    && !(clk->flags & VDD2_CONFIG_PARTICIPANT)) {
		if (clk->flags & DPLL_OUTPUT) {
			ret = prcm_get_dpll_mn_output(clk->prcmid, &mnoutput);
			/*mnoutput has CLKOUT = (Freq*m)/n+1 in case dpll
			 *is locked or bypass clock if it is not locked */
			if (ret != PRCM_PASS)
				/* Signal error */
				return (-EINVAL);
			ret =
			    prcm_clksel_round_rate(clk->prcmid, (2 * mnoutput),
						   (rate / 1000), &new_div);
			if (ret == PRCM_PASS) {
				validrate = (2 * mnoutput * 1000) / new_div;
				pr_debug("Valid rate: %d\n", validrate);
				if (validrate != rate)
					return (-EINVAL);
			} else
			/* No acceptable divider - signal error */
				return (-EINVAL);
			ret = prcm_configure_dpll_divider(clk->prcmid, new_div);
		} else if (clk->flags & RATE_CKCTL) {
			parent_rate = clk->parent->rate;
			ret =
				prcm_clksel_round_rate(clk->prcmid,
							   parent_rate, rate,
							   &new_div);
			if (ret == PRCM_PASS) {
				validrate = parent_rate / new_div;
				pr_debug("Valid rate: %d\n", validrate);
				if (validrate != rate)
					return (ret);
			} else
				return (-EINVAL);
			ret =
				prcm_clksel_set_divider(clk->prcmid,
							    new_div);
			if (ret != PRCM_PASS)
				ret = -EINVAL;
		} else if (clk->set_rate != 0) {
		/*call clk specific rate func*/
			ret = clk->set_rate(clk, rate);
		}
	}

	if (ret == 0)
		clk->recalc(clk);

	return ret;
}

/* Change the parent of a clock */
static int omap3_clk_set_parent(struct clk *cclk, struct clk *new_parent)
{
	int ret = -EINVAL;

	pr_debug("Clock name: %s, Parent name: %s\n", cclk->name,
		new_parent->name);

	if ((cclk->flags & VDD1_CONFIG_PARTICIPANT)
	    || (cclk->flags & VDD2_CONFIG_PARTICIPANT))
		return ret;

	if (!(cclk->flags & SRC_SEL))
		return ret;

	if (cclk->usecount > 0)	/*if clock is currently active. */
		_omap3_clk_disable(cclk); /* shut down clock to stop glitch */

	ret = prcm_clk_set_source(cclk->prcmid, new_parent->prcmid);
	if (ret != PRCM_PASS)
		ret = -EINVAL;

	if (cclk->usecount > 0)
		_omap3_clk_enable(cclk);

	if (ret == 0) {
		cclk->parent = new_parent;
		cclk->recalc(cclk);
	}
	return ret;
}

/* Return the parent of a clock */
static struct clk *omap3_clk_get_parent(struct clk *clk)
{
	return clk->parent;
}

static void omap3_table_recalc(struct clk *clk)
{
	if ((clk != &virt_vdd1_prcm_set) && (clk != &virt_vdd2_prcm_set))
		return;

	if (clk == &virt_vdd1_prcm_set)
		clk->rate = curr_vdd1_prcm_set->speed;
	else
		clk->rate = curr_vdd2_prcm_set->speed;
	pr_debug("CLK RATE:%lu\n", clk->rate);
}

static long omap3_round_to_table_rate(struct clk *clk, unsigned long rate)
{
	struct vdd_prcm_config *ptr;
	long highest_rate;

	if ((clk != &virt_vdd1_prcm_set) && (clk != &virt_vdd2_prcm_set))
		return -EINVAL;

	highest_rate = -EINVAL;

	if (clk == &virt_vdd1_prcm_set)
		ptr = vdd1_rate_table + MAX_VDD1_OPP;
	else
		ptr = vdd2_rate_table + MAX_VDD2_OPP;

	for (; ptr->speed; ptr--) {
		highest_rate = ptr->speed;
		pr_debug("Highest speed : %lu, rate: %lu\n"
			, highest_rate, rate);
		if (ptr->speed <= rate)
			break;
	}
	return highest_rate;
}

static int omap3_select_table_rate(struct clk *clk, unsigned long rate)
{
	u8 cpu_mask = 0;
	u32 cur_vdd_rate, current_opp;
	struct vdd_prcm_config *prcm_vdd;
	unsigned long found_speed = 0;
	int ret = -EINVAL;
	int div = 0;

	if ((clk != &virt_vdd1_prcm_set) && (clk != &virt_vdd2_prcm_set))
		return ret;

	if (cpu_is_omap3430())
		cpu_mask = RATE_IN_343X;

	if (clk == &virt_vdd1_prcm_set)
		prcm_vdd = vdd1_rate_table + MAX_VDD1_OPP;
	else
		prcm_vdd = vdd2_rate_table + MAX_VDD2_OPP;

	for (; prcm_vdd->speed; prcm_vdd--) {
		if (!(prcm_vdd->flags & cpu_mask))
			continue;
		if (prcm_vdd->speed <= rate) {
			found_speed = prcm_vdd->speed;
			pr_debug("Found speed = %lu\n", found_speed);
			break;
		}
	}

	if (!found_speed) {
		printk(KERN_INFO "Could not set table rate to %luMHz\n",
		       rate / 1000000);
		return -EINVAL;
	}

	if (clk == &virt_vdd1_prcm_set) {
		ret = prcm_get_processor_speed(clk->parent->prcmid, &cur_vdd_rate);
		if (ret != PRCM_PASS)
			return -EINVAL;
		current_opp = curr_vdd1_prcm_set->opp;
	} else {
		ret = prcm_get_dpll_rate(clk->parent->prcmid, &cur_vdd_rate);
		if (ret != PRCM_PASS)
			return -EINVAL;
		/* Now cur_vdd_rate holds value of core_ck */
		/* The divider for l3_ck */
		ret = prcm_clksel_get_divider((&l3_ck)->prcmid, &div);
		if ((ret != PRCM_PASS) || (div == 0))
			return -EINVAL;
		cur_vdd_rate /= div;
		current_opp = curr_vdd2_prcm_set->opp;
	}

	cur_vdd_rate *= 1000;
	pr_debug("Current rate:%u\n", cur_vdd_rate);
	if (cur_vdd_rate != found_speed) {
		ret = prcm_do_frequency_scaling(prcm_vdd->opp, current_opp);
		if (ret != PRCM_PASS)
			return -EINVAL;
#ifdef CONFIG_CORE_OFF
		save_scratchpad_contents();
#endif
	}

	if (clk == &virt_vdd1_prcm_set) {
		curr_vdd1_prcm_set = prcm_vdd;
		prcm_set_current_vdd1_opp(prcm_vdd->opp);
		omap3_clk_recalc(&mpu_ck);
		omap3_propagate_rate(&mpu_ck);
		omap3_clk_recalc(&iva2_ck);
		omap3_propagate_rate(&iva2_ck);
#ifndef CONFIG_CPU_FREQ
		/*Update loops_per_jiffy if processor speed is being changed*/
		loops_per_jiffy = compute_lpj(loops_per_jiffy,
				(cur_vdd_rate / 1000),
				(found_speed / 1000));
#endif
	} else {
		curr_vdd2_prcm_set = prcm_vdd;
		prcm_set_current_vdd2_opp(prcm_vdd->opp);
		omap3_clk_recalc(&core_ck);
		omap3_propagate_rate(&core_ck);
		omap3_clk_recalc(&core_x2_ck);
		omap3_propagate_rate(&core_x2_ck);
		omap3_clk_recalc(&emul_core_alwon_ck);
		omap3_propagate_rate(&emul_core_alwon_ck);
	}
	return 0;
}

/*-------------------------------------------------------------------------*
 * Omap3 clock reset and init functions
 *-------------------------------------------------------------------------*/

/* If usecount of clock is 0, disable it */
static void omap3_disable_unused_clocks(struct clk *clk)
{
	pr_debug("Disabling unused clock \"%s\"...\n ", clk->name);
	_omap3_clk_disable(clk);
}

#ifdef CONFIG_CPU_FREQ
static struct cpufreq_frequency_table freq_table[ARRAY_SIZE(vdd1_rate_table)];

static void omap3_clk_init_cpufreq_table(struct cpufreq_frequency_table **table)
{
	struct vdd_prcm_config *prcm;
	int i = 0;

	prcm = vdd1_rate_table + ARRAY_SIZE(vdd1_rate_table) -1;
	for (; prcm->speed; prcm--) {
		freq_table[i].index = i;
		freq_table[i].frequency = prcm->speed / 1000;
		i++;
	}

	if (i == 0) {
		printk(KERN_WARNING "%s: failed to initialize frequency \
								table\n",
								__FUNCTION__);
		return;
	}

	freq_table[i].index = i;
	freq_table[i].frequency = CPUFREQ_TABLE_END;

	*table = &freq_table[0];
}
#endif

/* Get the rate of oscillator by calling PRCM API */
static void omap3_get_crystal_rate(struct clk *osc)
{
	osc->rate = prcm_get_crystal_rate() * 1000;
	pr_debug("Rate:%lu\n", osc->rate);
}

static struct clk_functions omap3_clk_functions = {
	.clk_enable = omap3_clk_enable,
	.clk_disable = omap3_clk_disable,
	.clk_round_rate = omap3_clk_round_rate,
	.clk_set_rate = omap3_clk_set_rate,
	.clk_set_parent = omap3_clk_set_parent,
	.clk_get_parent = omap3_clk_get_parent,
	.clk_disable_unused = omap3_disable_unused_clocks,
#ifdef CONFIG_CPU_FREQ
	.clk_init_cpufreq_table = omap3_clk_init_cpufreq_table,
#endif
};

/*
 * Set clocks for bypass mode for reboot to work.
 */
void omap3_clk_prepare_for_reboot(void)
{
	/* Will be implemented when frequency change code is in place */
	return;
}

/* Get the parent clocks by reading registers and populate
   the clock nodes with the correct parent information */
static int omap3_update_sources(void)
{
	struct clk **clkptr;
	struct clk *cp;
	u32 pid = 0, ret = -EINVAL;

	for (clkptr = onchip_clks;
	     clkptr < onchip_clks + ARRAY_SIZE(onchip_clks); clkptr++) {
		cp = *clkptr;
		if (cp->flags & SRC_SEL) {
			ret = prcm_clk_get_source(cp->prcmid, &pid);
			if (ret != PRCM_PASS) {
				pr_debug(KERN_ERR
				"prcm_clk_get_source returned error for %s\n",
					cp->name);
				return -EINVAL;
			}
			switch (cp->prcmid) {
			case PRCM_48M_FCLK:
				if (pid == PRCM_DPLL4_M2X2_CLK)
					cp->parent = &func_96m_ck;
				else
					cp->parent = &sysaltck;
				break;
#ifndef CONFIG_ARCH_OMAP3410
			case PRCM_TVOUT:
				if (pid == PRCM_DPLL4_M3X2_CLK)
					cp->parent = &dpll4_m3x2_ck;
				else
					cp->parent = &sysaltck;
				break;
#endif
			case PRCM_USIM:
				if (pid == PRCM_SYS_CLK)
					cp->parent = &sys_ck;
				else if (pid == PRCM_DPLL4_M2X2_CLK)
					cp->parent = &cm_96m_fck;
				else
					cp->parent = &ext_mcbsp_ck;
				break;
#ifndef CONFIG_ARCH_OMAP3410
			/* CORE_CK & 96MHz supported */
			case PRCM_SGX_FCLK:
				if (pid == PRCM_DPLL3_M2_CLK)
					cp->parent = &core_ck;
				else
					cp->parent = &ext_mcbsp_ck;
				break;
#endif
			case PRCM_96M_CLK:
				if (pid == PRCM_DPLL4_M2X2_CLK)
					cp->parent = &cm_96m_fck;
				else
					cp->parent = &sys_ck;
				break;
			case PRCM_SYS_CLKOUT2:
				if (pid == PRCM_DPLL3_M2_CLK)
					cp->parent = &core_ck;
				else if (pid == PRCM_SYS_CLK)
					cp->parent = &sys_ck;
				else if (pid == PRCM_DPLL4_M2X2_CLK)
					cp->parent = &func_96m_ck;
#ifndef CONFIG_ARCH_OMAP3410
				else
					cp->parent = &dss_tv_fck;
#endif
				break;
			case PRCM_MCBSP1:
			case PRCM_MCBSP2:
			case PRCM_MCBSP3:
			case PRCM_MCBSP4:
			case PRCM_MCBSP5:
				if (pid == PRCM_DPLL4_M2X2_CLK)
					cp->parent = &func_96m_ck;
				else
					cp->parent = &ext_mcbsp_ck;
				break;
			case PRCM_GPT1:
			case PRCM_GPT2:
			case PRCM_GPT3:
			case PRCM_GPT4:
			case PRCM_GPT5:
			case PRCM_GPT6:
			case PRCM_GPT7:
			case PRCM_GPT8:
			case PRCM_GPT9:
			case PRCM_GPT10:
			case PRCM_GPT11:
				if (pid == PRCM_SYS_32K_CLK)
					cp->parent = &omap_32k_fck;
				else
					cp->parent = &sys_ck;

			}
			pr_debug("Parent updated for clock: %s\n", cp->name);
			pr_debug("Parent name: %s\n", cp->parent->name);
		}
	}
	return 0;
}

/* Arch specific init */
static int __init omap3_clk_arch_init(void)
{
	struct vdd_prcm_config *vdd1_prcm;
	struct vdd_prcm_config *vdd2_prcm;
	u32 sys_clk_speed, mpu_speed, core_speed, l3_speed = 0;
	int div;

	printk("OMAP Clock subsystem initializing.\n");

	/* Lock DPLL5 */
	if (prcm_configure_dpll(DPLL5_PER2, -1, -1, -1))
		panic("FATAL ERROR: Unable to Configure DPLL5\n");
	if (prcm_enable_dpll(DPLL5_PER2))
		panic("FATAL ERROR: Unable to Lock DPLL5\n");
	omap3_get_crystal_rate(&osc_sys_ck);
	omap3_update_sources();

	sys_clk_speed = prcm_get_system_clock_speed() * 1000;
	prcm_get_processor_speed(DOM_MPU, &mpu_speed);
	mpu_speed = mpu_speed * 1000;
	prcm_get_dpll_rate((&core_ck)->prcmid, &core_speed);
	core_speed = core_speed * 1000;
	prcm_clksel_get_divider((&l3_ck)->prcmid, &div);
	if (div != 0)
		l3_speed = core_speed / div;
	else
		printk(KERN_ERR"Error: Divider for L3 returned 0 in omap3_clk"
					"_arch_init\n");

	printk(KERN_INFO"System clock speed: %u, mpu speed: %u, l3_speed: %u\n", 
			sys_clk_speed, mpu_speed, l3_speed);

	for (vdd1_prcm = vdd1_rate_table+MAX_VDD1_OPP; vdd1_prcm->speed;
		vdd1_prcm--) {
		pr_debug("%lu\n", vdd1_prcm->speed);
		if (vdd1_prcm->speed <= mpu_speed)
			break;
	}
	curr_vdd1_prcm_set = vdd1_prcm;
	prcm_set_current_vdd1_opp(vdd1_prcm->opp);

	for (vdd2_prcm = vdd2_rate_table+MAX_VDD2_OPP; vdd2_prcm->speed;
		vdd2_prcm--) {
		pr_debug("%lu\n", vdd2_prcm->speed);
		if (vdd2_prcm->speed <= l3_speed)
			break;
	}
	curr_vdd2_prcm_set = vdd2_prcm;
	prcm_set_current_vdd2_opp(vdd2_prcm->opp);

	propagate_rate(&osc_sys_ck);	/* update main root fast */
	propagate_rate(&omap_32k_fck);	/* update main root slow */
	propagate_rate(&sysaltck);	/* update alt ck tree */

	pr_debug("Rate propagation done for all clocks\n");
	return 0;
}

arch_initcall(omap3_clk_arch_init);

int __init omap3_clk_init(void)
{
	struct clk **clkp;

	clk_init(&omap3_clk_functions);

	for (clkp = onchip_clks; clkp < onchip_clks + ARRAY_SIZE(onchip_clks);
	     clkp++) {

		if ((*clkp)->flags & CLOCK_IN_OMAP343X) {
			clk_register(*clkp);
			continue;
		}
	}

#ifdef CONFIG_OMAP34XX_OFFMODE
	spin_lock_init(&inatimer_lock);
	init_timer_deferrable(&coredomain_timer);
	init_timer_deferrable(&perdomain_timer);

	coredomain_timer.function = coredomain_timer_func;
	perdomain_timer.function = perdomain_timer_func;
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

	return 0;
}

#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
/* This function will enable the power for the domain in the which the devices
 * falls. It is called from clk_enable function.  If the device is in PER
 * domain then the power state for PER becomes ON as soon as the first device
 * in PER calls clk_enable.
*/
static int enable_power_domain(struct clk *clk)
{
	char *resource_name;
	u32 domainid;
	u32 ret = 0;
	unsigned short level;

	domainid      = DOMAIN_ID(clk->prcmid);
	resource_name = id_to_name[domainid - 1];

	/* Request for logical resource. Only CORE domain is modelled as a
	 * logical resource.
	 */
	if (resource_name == NULL)
		return ret;

	pr_debug("%s: pwr_domain_name %s name %s\n", __FUNCTION__,
		resource_name, clk->name);

	if (clk->flags & POWER_ON_REQUIRED) {
		if (clk->res == NULL) {
			clk->res = resource_get(clk->name, resource_name);
			if (clk->res == NULL) {
				printk(KERN_ERR"Could not get resource handle"
						"for %s\n", resource_name);
				return -EINVAL;
			}
		}

		level = (!strcmp(resource_name, "core")) ?
					LOGICAL_USED : POWER_DOMAIN_ON;
		ret = resource_request(clk->res, level);
		if (ret) {
			printk(KERN_ERR "Could not request ON for resource %s\n",
							 resource_name);
			return ret;
		}
	}
	return ret;
}

/*
 * This function will disable the power for the domain in the which the
 * devices falls. It is called from clk_disable function.
 * If the device is in PER domain then the power state for PER
 * becomes OFF as soon as the last device in PER calls clk_disable.
*/
static void disable_power_domain(struct clk *clk)
{
	char *resource_name;
	u32 ret = 0;
	u32 domainid;

	domainid = DOMAIN_ID(clk->prcmid);
	resource_name = id_to_name[domainid - 1];

	if (clk->flags & POWER_ON_REQUIRED) {
		if (clk->res != NULL) {
#ifdef CONFIG_OMAP34XX_OFFMODE
			if ((domainid == DOM_CORE1) || (domainid == DOM_PER))
				modify_inatimer(clk);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

			ret = resource_release(clk->res);
			if (ret)
				pr_debug("Could not release resource handle for"
						"%s\n", resource_name);

			resource_put(clk->res);
		}
		clk->res = NULL;
	}
	return;
}
#endif /* CONFIG_AUTO_POWER_DOMAIN_CTRL */
