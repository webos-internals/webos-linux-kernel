/*
 *  linux/drivers/cpufreq/cpufreq_screenstate.c
 *
 *  Marco Benton marco@unixpsycho.com 
 *
 *   screenstate v3
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <asm/arch/prcm.h>
#include <linux/kernel_stat.h>

#define FACTOR_MIN 1
#define FACTOR_MAX 4

// charger poll is in secs converted to jiffies
#define CHPOLLMIN 1
#define CHPOLLMAX 300

// vDemand poll is in msecs converted to jiffies
#define VDPOLLMIN 10
#define VDPOLLMAX 1500

// default charger poll secs
#define CHARGER_POLL 3

// default vdemand poll msecs
#define VDEMAND_POLL 100

// default ondemand_enabled poll msecs
#define ONDEMAND_POLL 50

// default factor
#define VDEMAND_FACTOR 2

// screen on and off delay in secs
#define SON_DELAY 1
#define SOFF_DELAY 1

#ifdef CONFIG_MACH_SIRLOIN_3630
#define SCALEBACK_SPEED 800000
#else
#define SCALEBACK_SPEED 500000
#endif

struct ss_params {
	bool cpu_is_managed;
	bool lcd_state;
	bool vdemand_enabled;
	bool charging_state;
	bool ch_override;
	bool ondemand_enabled;
	unsigned int ch_poll;
	unsigned int vdemand_poll;
	unsigned int ondemand_poll;
	unsigned short sfactor;
} sscfg = {0, 1, 1, 0, 0, 0, 0, 0, 0, VDEMAND_FACTOR};

static struct ss_params	*ss_cfg = &sscfg;

static unsigned int
	opp, last_load;

static cputime64_t
	prev_cpu_wall = 0, prev_cpu_idle = 0;

int	gadget_event_state_current(void),
	ds2784_getcurrent(int *ret_current),
	set_voltage_level(u8 vdd, u8 vsel);

unsigned int
	prcm_get_current_vdd1_opp_no(void);

void	omap_pm_opp_get_vdd2_volts(u8 * vdd2_volt),
	omap_pm_opp_get_volts(u8 vdd1_volts[]);

unsigned short
	get_vdd1_arm_opp_for_freq(unsigned int freq);

static inline void
	check_charger(struct work_struct *work),
	check_load(struct work_struct *work),
	__cpufreq_gov_screenstate_lcdoff(struct work_struct *work),
	__cpufreq_gov_screenstate_lcdon(struct work_struct *work);

static DEFINE_MUTEX(screenstate_mutex);

static DECLARE_DELAYED_WORK(worker, check_charger);
static DECLARE_DELAYED_WORK(worker2, check_load);
static DECLARE_DELAYED_WORK(worker3, __cpufreq_gov_screenstate_lcdoff);
static DECLARE_DELAYED_WORK(worker4, __cpufreq_gov_screenstate_lcdon);

#define CPUFREQ_SCREENSTATE_ATTR(_name) \
static struct freq_attr _attr_##_name = {\
        .attr = {.name = __stringify(_name), .mode = 0644, }, \
        .show = show_##_name,\
        .store = store_##_name,\
};

static unsigned int jiffies_to_secs(unsigned long int jifs)
{
	return jifs / HZ;
}

static unsigned long int secs_to_jiffies(unsigned int secs)
{
	return secs * HZ;
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu)
{
	cputime64_t idle_time;
	cputime64_t cur_jiffies;
	cputime64_t busy_time;

	cur_jiffies = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
				  kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);

	idle_time = cputime64_sub(cur_jiffies, busy_time);
	return idle_time;
}

static void reset_voltage(void)
{
	u8 vdd2_volt, volt[PRCM_NO_VDD1_OPPS];

	omap_pm_opp_get_volts(volt);
	omap_pm_opp_get_vdd2_volts(&vdd2_volt);
	set_voltage_level(1, volt[opp - 1]);
	set_voltage_level(2, vdd2_volt);
}

static int screenstate_notifier(struct notifier_block *nb, unsigned long val,
				void *data)
{
	opp = prcm_get_current_vdd1_opp_no();

	return 0;
}

static struct notifier_block screenstate_notifier_block = {
	.notifier_call = screenstate_notifier
};

static void change_freq(void) { 
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	if (ss_cfg->charging_state)
		__cpufreq_driver_target(policy, SCALEBACK_SPEED,
					CPUFREQ_RELATION_H);
	else
		__cpufreq_driver_target(policy, (ss_cfg->lcd_state)
					? policy->max
					: policy->min, CPUFREQ_RELATION_H);
}

static inline void check_charger(struct work_struct *work)
{
	int cur = 0, current_mA = 0;

	ds2784_getcurrent(&cur);
	current_mA = gadget_event_state_current();
	if ((cur > 0) && ((current_mA != 100) && (current_mA < 500))) {
		// Assume Touchstone
		if (!ss_cfg->charging_state) {
			ss_cfg->charging_state = 1;
			change_freq();
			printk("screenstate: TS found!\n");
		}
	} else {
		if (current_mA == 1000) {
			if (!ss_cfg->charging_state) {
				ss_cfg->charging_state = 1;
				change_freq();
				printk("screenstate: 1000mA charger found!\n");
			}
		} else {
			if (ss_cfg->charging_state) {
				ss_cfg->charging_state = 0;
				change_freq();
				printk("screenstate: charger unplugged!\n");
			}
		}
	}
	schedule_delayed_work(&worker, ss_cfg->ch_poll);
	return;
}

static inline void check_load(struct work_struct *work)
{
	unsigned int tmp_idle_ticks, idle_ticks, total_ticks, load = 0;
	cputime64_t total_idle_ticks, cur_jiffies;
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	u8 vdd1_volt, vdd2_volt, volt[PRCM_NO_VDD1_OPPS];

	if (!opp)
		goto out;

	mutex_lock(&screenstate_mutex);

	idle_ticks = UINT_MAX;
	cur_jiffies = jiffies64_to_cputime64(get_jiffies_64());
	total_ticks = (unsigned int)cputime64_sub(cur_jiffies, prev_cpu_wall);
	prev_cpu_wall = get_jiffies_64();

	if (!total_ticks)
		goto out;

	total_idle_ticks = get_cpu_idle_time(0);
	tmp_idle_ticks = (unsigned int)cputime64_sub(total_idle_ticks,
						     prev_cpu_idle);
	prev_cpu_idle = total_idle_ticks;

	if (tmp_idle_ticks < idle_ticks)
		idle_ticks = tmp_idle_ticks;
	if (likely(total_ticks > idle_ticks))
		load = (100 * (total_ticks - idle_ticks)) / total_ticks;

	if (!last_load)
		goto out;

	if (ss_cfg->lcd_state && ss_cfg->ondemand_enabled)
		goto on_demand;

	if (!ss_cfg->vdemand_enabled)
		goto out;

	omap_pm_opp_get_volts(volt);
	omap_pm_opp_get_vdd2_volts(&vdd2_volt);
	vdd1_volt = volt[opp - 1];

	if ((load < 5) && (last_load < 5)) {
		set_voltage_level(1, vdd1_volt - (2 * ss_cfg->sfactor) - 2);
		set_voltage_level(2, vdd2_volt - (2 * ss_cfg->sfactor) - 2);
		goto out;
	}

	if ((load < 30) && (last_load > 29)) {
		set_voltage_level(1, vdd1_volt - (2 * ss_cfg->sfactor));
		set_voltage_level(2, vdd2_volt - (2 * ss_cfg->sfactor));
		goto out;
	}
	if (((load > 29) && (load < 70)) &&
	    ((last_load < 30) || (last_load > 69))) {
		set_voltage_level(1, vdd1_volt - ss_cfg->sfactor);
		set_voltage_level(2, vdd2_volt - ss_cfg->sfactor);
		goto out;
	}
	if ((load > 69) && (last_load < 70)) {
		set_voltage_level(1, vdd1_volt);
		set_voltage_level(2, vdd2_volt);
		goto out;
	}

	goto out;

on_demand:

	if (ss_cfg->charging_state) {
		if (policy->cur != SCALEBACK_SPEED) change_freq();
		goto out;
	}

	if (((load > 19) && (load < 70)) || ((load < 10) && (last_load > 49))) {
		__cpufreq_driver_target(policy, (load > 39)
			? get_arm_freq_for_opp(
				get_vdd1_arm_opp_for_freq(policy->max) - 1)
			: get_arm_freq_for_opp(
				get_vdd1_arm_opp_for_freq(policy->min) + 1),
			CPUFREQ_RELATION_L);
		goto out;
	}

	if ((load < 20) && (last_load < 41)) {
		if (policy->cur != policy->min)
			__cpufreq_driver_target(policy, policy->min,
						CPUFREQ_RELATION_L);
		goto out;
	}
	if ((load > 9)) {
		if (policy->cur != policy->max)
			__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
		goto out;
	}

 out:
	last_load = load;

	mutex_unlock(&screenstate_mutex);
	schedule_delayed_work(&worker2,
		(ss_cfg->lcd_state && ss_cfg->ondemand_enabled)
			? ss_cfg->ondemand_poll : ss_cfg->vdemand_poll);

	return;
}

static ssize_t show_vdemand_factor(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", ss_cfg->sfactor);
}

static ssize_t store_vdemand_factor(struct cpufreq_policy *policy,
				    const char *buf, size_t count)
{
	u8 i;

	if (sscanf(buf, "%hhu", &i)) {
		if ((i < FACTOR_MIN) || (i > FACTOR_MAX))
			printk("screenstate: invalid factor\n");
		else {
			ss_cfg->sfactor = i;
		}
	} else
		printk("screenstate: missing factor value\n");

	return count;
}

static ssize_t show_vdemand_enable(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", ss_cfg->vdemand_enabled);
}

static ssize_t store_vdemand_enable(struct cpufreq_policy *policy,
				    const char *buf, size_t count)
{
	u8 i;

	if (sscanf(buf, "%hhu", &i)) {
		if ((i != 0) && (i != 1))
			printk("screenstate: invalid vdemand bool\n");
		else {
			if ((!i) && (ss_cfg->vdemand_enabled)) {
				if (!ss_cfg->ondemand_enabled)
					cancel_delayed_work(&worker2);
				reset_voltage();
			}
			if ((i) && (!ss_cfg->vdemand_enabled))
				if (!ss_cfg->ondemand_enabled)
					schedule_delayed_work(&worker2,
						      ss_cfg->vdemand_poll);
			ss_cfg->vdemand_enabled = i;
		}
	} else
		printk("screenstate: missing value\n");

	return count;
}

static ssize_t show_charger_override(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", ss_cfg->ch_override);
}

static ssize_t store_charger_override(struct cpufreq_policy *policy,
				 const char *buf, size_t count)
{
	u8 i;

	if (sscanf(buf, "%hhu", &i)) {
		if ((i != 0) && (i != 1))
			printk("screenstate: invalid chrg bool\n");
		else {
			if ((i) && (!ss_cfg->ch_override)) {
				ss_cfg->charging_state = 0;
				cancel_delayed_work(&worker);
			}
			if ((!i) && (ss_cfg->ch_override))
				schedule_delayed_work(&worker, ss_cfg->ch_poll);
			ss_cfg->ch_override = i;
		}
	} else
		printk("screenstate: missing value\n");

	return count;
}

static ssize_t show_charger_poll_rate(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", jiffies_to_secs(ss_cfg->ch_poll));
}

static ssize_t store_charger_poll_rate(struct cpufreq_policy *policy,
				       const char *buf,
				       size_t count)
{
	unsigned int i;

	if (sscanf(buf, "%u", &i)) {
		if ((i < CHPOLLMIN) || (i > CHPOLLMAX))
			printk("screenstate: invalid chrg poll value\n");
		else
			ss_cfg->ch_poll = secs_to_jiffies(i);;
	} else
		printk("screenstate: missing value\n");

	return count;
}

static ssize_t show_vdemand_poll_rate(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", jiffies_to_msecs(ss_cfg->vdemand_poll));
}

static ssize_t store_vdemand_poll_rate(struct cpufreq_policy *policy,
				       const char *buf,
				       size_t count)
{
	unsigned int i;

	if (sscanf(buf, "%u", &i)) {
		if ((i < VDPOLLMIN) || (i > VDPOLLMAX))
			printk("screenstate: invalid poll time\n");
		else
			ss_cfg->vdemand_poll = msecs_to_jiffies(i);
	} else
		printk("screenstate: missing value\n");

	return count;
}

static ssize_t show_ondemand_enable(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", ss_cfg->ondemand_enabled);
}

static ssize_t store_ondemand_enable(struct cpufreq_policy *policy,
				     const char *buf,
				     size_t count)
{
	u8 i;

	if (sscanf(buf, "%hhu", &i)) {
		if ((i != 0) && (i != 1))
			printk("screenstate: invalid entry\n");
		else {
			if ((!i) && (ss_cfg->ondemand_enabled))
				if(!ss_cfg->vdemand_enabled)
					cancel_delayed_work(&worker2);
			if ((i) && (!ss_cfg->ondemand_enabled))
				if(!ss_cfg->vdemand_enabled)
					schedule_delayed_work(&worker2,
						ss_cfg->ondemand_poll);
			//if(i) reset_voltage();
			ss_cfg->ondemand_enabled = i;
			printk("screenstate: toggle ondemand\n");
		}
	} else
		printk("screenstate: missing value\n");

	return count;
}

CPUFREQ_SCREENSTATE_ATTR(vdemand_factor);
CPUFREQ_SCREENSTATE_ATTR(vdemand_enable);
CPUFREQ_SCREENSTATE_ATTR(charger_override);
CPUFREQ_SCREENSTATE_ATTR(vdemand_poll_rate);
CPUFREQ_SCREENSTATE_ATTR(charger_poll_rate);
CPUFREQ_SCREENSTATE_ATTR(ondemand_enable);

static struct attribute *default_attrs[] = {
	&_attr_vdemand_factor.attr,
	&_attr_vdemand_enable.attr,
	&_attr_charger_override.attr,
	&_attr_charger_poll_rate.attr,
	&_attr_vdemand_poll_rate.attr,
	&_attr_ondemand_enable.attr,
	NULL
};

static struct attribute_group screenstate_attr_group = {
	.attrs = default_attrs,
	.name = "screenstate-v3",
};

static int cpufreq_governor_screenstate(struct cpufreq_policy *policy,
					unsigned int event)
{
	int rc;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (ss_cfg->cpu_is_managed)
			break;

		mutex_lock(&screenstate_mutex);
		ss_cfg->cpu_is_managed = 1;
		ss_cfg->lcd_state = 1;
		ss_cfg->charging_state = 0;
		ss_cfg->vdemand_enabled = 1;
		ss_cfg->ch_override = 0;
		ss_cfg->ondemand_enabled = 0;
		ss_cfg->sfactor = VDEMAND_FACTOR;
		ss_cfg->vdemand_poll = msecs_to_jiffies(VDEMAND_POLL);
		ss_cfg->ondemand_poll = msecs_to_jiffies(ONDEMAND_POLL);
		ss_cfg->ch_poll = secs_to_jiffies(CHARGER_POLL);

		last_load = 0;
		prev_cpu_idle = get_cpu_idle_time(0);
		prev_cpu_wall = get_jiffies_64();
		opp = prcm_get_current_vdd1_opp_no();
		cpufreq_register_notifier(&screenstate_notifier_block,
					  CPUFREQ_TRANSITION_NOTIFIER);
		change_freq();

		schedule_delayed_work(&worker, ss_cfg->ch_poll);
		schedule_delayed_work(&worker2, ss_cfg->vdemand_poll);

		rc = sysfs_create_group(&policy->kobj, &screenstate_attr_group);
		if (rc) {
			mutex_unlock(&screenstate_mutex);
			return rc;
                }

		mutex_unlock(&screenstate_mutex);
		printk("screenstate: initialized\n");
		break;
	case CPUFREQ_GOV_STOP:
		mutex_lock(&screenstate_mutex);
		ss_cfg->cpu_is_managed = 0;
		ss_cfg->lcd_state = 0;
		cancel_delayed_work(&worker);
		cancel_delayed_work(&worker2);
		reset_voltage();
		cpufreq_unregister_notifier(&screenstate_notifier_block,
					    CPUFREQ_TRANSITION_NOTIFIER);
		sysfs_remove_group(&policy->kobj, &screenstate_attr_group);
		mutex_unlock(&screenstate_mutex);

		break;
	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&screenstate_mutex);
		printk("screenstate: policy change\n");
		change_freq();
		mutex_unlock(&screenstate_mutex);

		break;
	}
	return 0;
}

struct cpufreq_governor cpufreq_gov_screenstate = {
	.name = "screenstate-v3",
	.governor = cpufreq_governor_screenstate,
	.owner = THIS_MODULE,
};

static int __init cpufreq_gov_screenstate_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_screenstate);
}

static void __exit cpufreq_gov_screenstate_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_screenstate);
}

void cpufreq_gov_screenstate_lcdoff(void)
{
	if (ss_cfg->cpu_is_managed)
		schedule_delayed_work(&worker3, secs_to_jiffies(SON_DELAY));
}

EXPORT_SYMBOL(cpufreq_gov_screenstate_lcdoff);

static inline void __cpufreq_gov_screenstate_lcdoff(struct work_struct *work)
{
	printk("screenstate: lcd off\n");
	ss_cfg->lcd_state = 0;
	change_freq();
}

void cpufreq_gov_screenstate_lcdon(void)
{
	if (ss_cfg->cpu_is_managed)
		schedule_delayed_work(&worker4, secs_to_jiffies(SOFF_DELAY));
}

EXPORT_SYMBOL(cpufreq_gov_screenstate_lcdon);

static inline void __cpufreq_gov_screenstate_lcdon(struct work_struct *work)
{
	printk("screenstate: lcd on\n");
	ss_cfg->lcd_state = 1;
	change_freq();
}

unsigned short cpufreq_screenstate_lcd_state(void)
{
	if (ss_cfg->cpu_is_managed)
		return ss_cfg->lcd_state;
	else
		return 0;
}

EXPORT_SYMBOL(cpufreq_screenstate_lcd_state);

EXPORT_SYMBOL(cpufreq_gov_screenstate);
MODULE_AUTHOR("marco@unixpsycho.com");
MODULE_DESCRIPTION("CPUfreq policy governor 'screenstate-v3'");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCREENSTATE
fs_initcall(cpufreq_gov_screenstate_init);
#else
module_init(cpufreq_gov_screenstate_init);
#endif
module_exit(cpufreq_gov_screenstate_exit);
