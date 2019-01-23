/*
 *  drivers/cpufreq/cpufreq_override.c
 *
 *      Marco Benton <marco@unixpsycho.com>.
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
#include <linux/sysdev.h>
#include <linux/cpu.h>
#include <linux/sysfs.h>
#include <linux/cpufreq.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/workqueue.h>
#include <linux/string.h>
#include <linux/kernel_stat.h>
#include <linux/delay.h>

// Voltage min
#define VDD_MIN 800000

// Voltage max
#define VDD_MAX 1600000

// Max Freq count. Not the actual number of freqs
#define MAX_FREQS 35

// L2 Boost mode.  This requires mods in acpuclock-8x60.c
#define L2_BOOST 0

// CPU load ramp up percent
#define RAMPUP_PERCENT 20

// CPU load poll jiffies
#define POLL_FREQ 100

// Max charging freq default
#define CHARGING_MAX 1188000

/* ************* end of tunables ***************************************** */

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
void acpuclk_get_voltages(unsigned int acpu_freq_vlt_tbl[]);
void acpuclk_set_voltages(unsigned int acpu_freq_vlt_tbl[]);
#endif

unsigned int acpuclk_get_freqs(unsigned int acpu_freq_tbl[]);
static unsigned int freq_table[MAX_FREQS];
static unsigned int nr_freqs;

#ifdef CONFIG_CPU_FREQ_OVERRIDE_L2_HACK
void acpuclk_set_l2_hack(bool state);
static bool l2boost = L2_BOOST;
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE
static bool lcd_state = 1;
static bool chrg_state = 0;
static bool chrg_override = 1;
static unsigned int chrg_max = CHARGING_MAX, chrg_prevmax = 0;
void cpufreq_set_policy(struct cpufreq_policy *policy, unsigned int cpu);
#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE_ENABLE
static bool turbomode = 1;
#else
static bool turbomode = 0;
#endif
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
static cputime64_t prev_cpu_wall = 0, prev_cpu_idle = 0;
static unsigned int time_in_state = 0;
#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER_ENABLE
static bool power_save = 1;
#else
static bool power_save = 0;
#endif

static inline void check_load(struct work_struct *work);
static DEFINE_MUTEX(override_mutex);
static DECLARE_DELAYED_WORK(worker, check_load);
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE
static void cpu_state(bool state)
{
	struct sys_device *dev = get_cpu_sysdev(1);

	cpu_hotplug_driver_lock();

	if(!state) {
		if(!cpu_down(1)) kobject_uevent(&dev->kobj, KOBJ_OFFLINE);
	}
	else {
		if(!cpu_up(1)) kobject_uevent(&dev->kobj, KOBJ_ONLINE);
	}

	cpu_hotplug_driver_unlock();
}

bool cpufreq_override_get_state(void)
{
	unsigned int ret = 0;

#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
	if(lcd_state) ret = (power_save || turbomode);
#else
	if(lcd_state) ret = turbomode;
#endif

	return ret;
}
EXPORT_SYMBOL(cpufreq_override_get_state);

#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
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

static inline unsigned int cur_load(void)
{
        unsigned int tmp_idle_ticks, idle_ticks, total_ticks, load = 0, ret = 0;
        cputime64_t total_idle_ticks, cur_jiffies;

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

	ret = load;

out:
	return ret;
}

static inline void check_load(struct work_struct *work)
{
	mutex_lock(&override_mutex);

	BUG_ON(!power_save);

	if(!lcd_state) goto out;

	// only help out cpu0, not overall load
	if(cur_load() > RAMPUP_PERCENT) {
		if(!cpu_online(1))
			cpu_state(1);

		time_in_state = 0;
	}
	else {
		if(cpu_online(1) && (time_in_state > 9)) { 
			cpu_state(0);
			time_in_state = 0;
		}
		else
			if(cpu_online(1)) time_in_state++;
	}

out:
	mutex_unlock(&override_mutex);
	schedule_delayed_work_on(0,&worker, POLL_FREQ);
}
#endif

void cpufreq_override_set_lcd_state(bool state)
{
#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
	mutex_lock(&override_mutex);
#endif

	lcd_state = state;
	printk("override: lcd state=%u\n",state);

	// If screen is off, take cpu offline if in power save mode
#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
	if(power_save && !lcd_state) cpu_state(0);
#else
	if(!lcd_state) cpu_state(0);
#endif

	// force CPU online regardless
	if(lcd_state) cpu_state(1);

#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
	mutex_unlock(&override_mutex);
#endif
}
EXPORT_SYMBOL(cpufreq_override_set_lcd_state);

bool cpufreq_override_get_lcd_state(void)
{
	return lcd_state;
}
EXPORT_SYMBOL(cpufreq_override_get_lcd_state);
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE
void cpufreq_override_set_chrg(bool state)
{
	struct cpufreq_policy policy;
	int ttmp, i;

	if(chrg_override) goto out;

	mutex_lock(&override_mutex);

	if(state) {
		if(!chrg_state) {
			ttmp = turbomode;
			turbomode=1;
			cpu_state(1);
			for_each_online_cpu(i) {
				msleep(200);
				cpufreq_get_policy(&policy, i);
				chrg_prevmax = policy.max;
				policy.max = chrg_max;
				cpufreq_set_policy(&policy, i);
			}
			turbomode = ttmp;
		}
	}
	else {
		if(chrg_state) {
			ttmp = turbomode;
			turbomode=1;
			cpu_state(1);
			for_each_online_cpu(i) {
				msleep(200);
				cpufreq_get_policy(&policy, i);
				policy.max = (chrg_prevmax) ? chrg_prevmax :
							freq_table[nr_freqs-1];
				cpufreq_set_policy(&policy, i);
			}
			turbomode = ttmp;
		}
	}

	mutex_unlock(&override_mutex);
out:
	chrg_state = state;

	printk("override: charger %s!\n", (chrg_state) ? "plugged in" \
							: "unplugged");
}
EXPORT_SYMBOL(cpufreq_override_set_chrg);

static ssize_t show_override_charger(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
        return sprintf(buf,"%u\n", chrg_override);
}

static ssize_t store_override_charger(struct kobject *a, struct attribute *b,
                                const char *buf, size_t count)
{
	struct cpufreq_policy policy;
	unsigned int tmp, ttmp, i;

	if(sscanf(buf, "%u", &tmp) == 1) {
		tmp = (tmp != 0 && tmp != 1) ? chrg_override : tmp;
		if(tmp && (!chrg_override && chrg_state)) {
			ttmp = turbomode;
			turbomode=1;
			cpu_state(1);
			for_each_online_cpu(i) {
				msleep(200);
				cpufreq_get_policy(&policy, i);
				policy.max = (chrg_prevmax) ? chrg_prevmax :
							freq_table[nr_freqs-1];
				cpufreq_set_policy(&policy, i);
			}
			turbomode = ttmp;
		}
		if(!tmp && (chrg_override && chrg_state)) {
			ttmp = turbomode;
			turbomode=1;
			cpu_state(1);
			for_each_online_cpu(i) {
				msleep(200);
				cpufreq_get_policy(&policy, i);
				chrg_prevmax = policy.max;
				policy.max = chrg_max;
				cpufreq_set_policy(&policy, i);
			}
			turbomode = ttmp;
		}
		chrg_override = tmp;

		printk("override: set chrg_override: %u\n", chrg_override);
	}       
	else
		printk("override: invalid chrg_override mode\n");

	return count;
}
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
static ssize_t show_vdd_max(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return sprintf(buf, "%u\n", VDD_MAX);
}

static ssize_t show_vdd_min(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return sprintf(buf, "%u\n", VDD_MIN);
}

static ssize_t show_vdd(struct kobject *kobj, struct attribute *attr, char *buf)
{
	unsigned int i, acpu_freq_vlt_tbl[MAX_FREQS];
	char tmp[250];

	acpuclk_get_voltages(acpu_freq_vlt_tbl);

	strcpy(buf,"");

	for(i=0 ; i < nr_freqs ; ++i) {
		sprintf(tmp,"%u ",acpu_freq_vlt_tbl[i]);
		strcat(buf,tmp);
	}

	strcpy(tmp,buf);

	return sprintf(buf,"%s\n",tmp);
}

static ssize_t store_vdd(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int i = 0, acpu_freq_vlt_tbl[MAX_FREQS];
	unsigned int *c = acpu_freq_vlt_tbl;
	const char *wp = buf;

	for(i = 0; i < nr_freqs; i++) {
		wp=skip_spaces(wp);
		sscanf(wp,"%u",&c[i]);
		if(c[i] < VDD_MIN || c[i] > VDD_MAX) break;
		wp=strchr(wp,' ')+1;
  	}

	if(i != nr_freqs)
		printk("override: store_vdd invalid\n");
	else {
		acpuclk_set_voltages(acpu_freq_vlt_tbl);

		printk("override: set vdd %s\n",buf);
	}

	return count;
}
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
static ssize_t show_power_saver(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return sprintf(buf,"%u\n",power_save);
}

static ssize_t store_power_saver(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int tmp;

	if(sscanf(buf, "%u", &tmp) == 1) {
		tmp = (tmp != 0 && tmp != 1) ? 0 : tmp;
		if(tmp) {
			cpu_state(0);
#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE
			printk("override: disabling turbo mode\n");
			turbomode = 0;
#endif
			if(!power_save) schedule_delayed_work_on(0,&worker,1);

#ifdef CONFIG_CPU_FREQ_OVERRIDE_L2_HACK
			printk("override: disabling L2 boost...\n");
			l2boost = 0;
			acpuclk_set_l2_hack(0);
#endif

		}
		else {
			cpu_state(1);
			if(power_save) cancel_delayed_work_sync(&worker);
		}

		power_save = tmp;

		printk("override: set power_save: %u\n",power_save);
	}	
	else
		printk("override: invalid power save mode\n");

	return count;
}
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE
static ssize_t show_turbo_mode(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return sprintf(buf,"%u\n",turbomode);
}

static ssize_t store_turbo_mode(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int tmp;

	if(sscanf(buf, "%u", &tmp) == 1) {
		turbomode = (tmp != 0 && tmp != 1) ? turbomode : tmp;

		if(turbomode) {
			cpu_state(1);
#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
			if(power_save) {
				power_save = 0;
				printk("override: disabling power saver\n");
				cancel_delayed_work_sync(&worker);
			}
#endif
		}
		else cpu_state(0);

		printk("override: set turbo_mode: %u\n",turbomode);
	}
	else
		printk("override: invalid turbo_mode\n");

	return count;
}
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_L2_HACK
static ssize_t show_l2_boost(struct kobject *kobj, struct attribute *attr,
				char *buf)
{
	return sprintf(buf,"%u\n",l2boost);
}

static ssize_t store_l2_boost(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int tmp;

	if(sscanf(buf, "%u", &tmp) == 1) {
		l2boost = (tmp != 0 && tmp != 1) ? l2boost : tmp;
		if(l2boost) acpuclk_set_l2_hack(1);
		else acpuclk_set_l2_hack(0);

		printk("override: set l2 boost: %u\n",l2boost);
	}       
	else
		printk("override: invalid l2 boost mode\n");

	return count;
}
#endif

static ssize_t show_vdd_freqs(struct kobject *kobj, struct attribute *attr, char *buf)
{
	unsigned int i;
	char tmp[250];

	strcpy(buf,"");

	for(i=0 ; i < nr_freqs ; ++i) {
		sprintf(tmp,"%u ", freq_table[i]);
		strcat(buf,tmp);
	}

	strcpy(tmp,buf);

	return sprintf(buf,"%s\n",tmp);
}

#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
define_one_global_ro(vdd_min);
define_one_global_ro(vdd_max);
define_one_global_rw(vdd);
#endif
define_one_global_ro(vdd_freqs);
#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
define_one_global_rw(power_saver);
#endif
#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE
define_one_global_rw(turbo_mode);
define_one_global_rw(override_charger);
#endif
#ifdef CONFIG_CPU_FREQ_OVERRIDE_L2_HACK
define_one_global_rw(l2_boost);
#endif

static struct attribute *default_attrs[] = {
#ifdef CONFIG_CPU_FREQ_OVERRIDE_VOLT_CONFIG
	&vdd.attr,
	&vdd_min.attr,
	&vdd_max.attr,
#endif
	&vdd_freqs.attr,
#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
	&power_saver.attr,
#endif
#ifdef CONFIG_CPU_FREQ_OVERRIDE_TURBO_MODE
	&turbo_mode.attr,
	&override_charger.attr,
#endif
#ifdef CONFIG_CPU_FREQ_OVERRIDE_L2_HACK
	&l2_boost.attr,
#endif
	NULL
};

static struct attribute_group override_attr_group = {
	.attrs = default_attrs,
	.name = "override"
};

static int __init cpufreq_override_driver_init(void)
{
	int ret = 0;

	nr_freqs = acpuclk_get_freqs(freq_table);
	printk("override: freqs configured: %u\n",nr_freqs);

#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
	if(power_save) {
		schedule_delayed_work_on(0,&worker, 10);
		turbomode = 0;
	}
#endif

	if((ret = sysfs_create_group(cpufreq_global_kobject,&override_attr_group))) 
		 printk("override: failed!\n");
	else
		 printk("override: initialized!\n");

	return ret;
}

static void __exit cpufreq_override_driver_exit(void)
{
#ifdef CONFIG_CPU_FREQ_OVERRIDE_POWERSAVER
	cancel_delayed_work(&worker);
	flush_scheduled_work();
#endif

	sysfs_remove_group(cpufreq_global_kobject, &override_attr_group);
}

MODULE_AUTHOR("marco@unixpsycho.com");
MODULE_DESCRIPTION("'cpufreq_override' - A driver to do cool stuff ");
MODULE_LICENSE("GPL");

module_init(cpufreq_override_driver_init);
module_exit(cpufreq_override_driver_exit);

