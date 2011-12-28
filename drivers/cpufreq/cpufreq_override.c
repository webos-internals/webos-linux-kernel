/*
 *  drivers/cpufreq/cpufreq_override.c
 *
 *  	Marco Benton <marco@unixpsycho.com>.
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
#include <asm/arch/prcm.h>

// VDD1 Vsel max
#define VDD1_VSEL_MAX 112

// VDD1 Vsel min
#define VDD1_VSEL_MIN 25

// VDD2 Vsel max
#define VDD2_VSEL_MAX 55

// VDD2 Vsel min
#define VDD2_VSEL_MIN 33

// High temp alarm and cap
#ifdef CONFIG_MACH_SIRLOIN_3630
#define HIGHTEMP_SCALEBACK 48
#else
#define HIGHTEMP_SCALEBACK 50
#endif

//Reset temp from alarm
#ifdef CONFIG_MACH_SIRLOIN_3630
#define LOWTEMP_RESET 45
#else
#define LOWTEMP_RESET 47
#endif

// Polling frequency secs
#define BATTERY_POLLING 600

// Polling frequency secs
#ifdef CONFIG_MACH_SIRLOIN_3630
#define TEMP_POLLING 1
#else
#define TEMP_POLLING 3
#endif

// Battery scaleback percent
#define BATTERY_PERCENT 20

// Scaleback speed
#ifdef CONFIG_MACH_SIRLOIN_3630
#define SCALEBACK_SPEED 1000000
#else
#define SCALEBACK_SPEED 500000
#endif

void	omap_pm_opp_get_volts(u8 vdd1_volts[]),
	omap_pm_opp_set_volts(u8 vdd1_volts[]),
	omap_pm_opp_get_vdd2_volts(u8 * vdd2_volt),
	omap_pm_opp_set_vdd2_volts(u8 vdd2_volt),
	omap_pm_opp_get_vdd2_freq(u8 * vdd2_freq);

int	omap34xx_get_temp(void),
	cpufreq_set_policy(struct cpufreq_policy *policy),
	ds2784_getpercent(int *ret_percent);

static inline void
	check_temp(struct work_struct *work),
	check_battery(struct work_struct *work);

unsigned short
	get_vdd1_arm_opp_for_freq(unsigned int freq);

struct ovrd {
	bool overtemp_alarm;
	bool battery_alarm;
	u32 prev_maxspeed;
	u32 temp_scaleback_high;
	u32 temp_scaleback_low;
	u32 temp_scaleback_speed;
	u32 battery_scaleback_percent;
	u32 battery_scaleback_speed;
} ovrdcfg = {0, 0, 0, HIGHTEMP_SCALEBACK, LOWTEMP_RESET,
				SCALEBACK_SPEED, BATTERY_PERCENT,
				SCALEBACK_SPEED};

static struct ovrd *ovrd_policy = &ovrdcfg;

static DECLARE_DELAYED_WORK(worker, check_battery);
static DECLARE_DELAYED_WORK(worker2, check_temp);

#define CPUFREQ_OVERRIDE_ATTR(_name,_func) \
static struct freq_attr _attr_##_name = {\
        .attr = {.name = __stringify(_name), .mode = 0644, }, \
        .show = show_##_func,\
        .store = store_##_func,\
};

#define CPUFREQ_OVERRIDE_ATTR2(_name,_show) \
static struct freq_attr _attr_##_name = {\
	.attr = {.name = __stringify(_name), .mode = 0444, }, \
	.show = _show,\
};

/*static unsigned int jiffies_to_secs(unsigned long int jifs) {
	return jifs / HZ;
} */

static unsigned long int secs_to_jiffies(unsigned int secs)
{
	return secs * HZ;
}

static bool check_freq(unsigned int freq)
{
	if(get_vdd1_arm_opp_for_freq(freq)) return 1;
	else return 0;
}

static void change_freq_low(unsigned int freq)
{
	struct cpufreq_policy new_policy;
	cpufreq_get_policy(&new_policy, 0);
	ovrd_policy->prev_maxspeed = new_policy.max;
	new_policy.max = freq;
	cpufreq_set_policy(&new_policy);
}

static void change_freq_high(void)
{
	struct cpufreq_policy new_policy;
	cpufreq_get_policy(&new_policy, 0);
	new_policy.max = ovrd_policy->prev_maxspeed;
	cpufreq_set_policy(&new_policy);
	ovrd_policy->overtemp_alarm = 0;
}

static inline void check_temp(struct work_struct *work)
{
	u32 cputemp;

	if (ovrd_policy->battery_alarm)
		goto out;

	cputemp = omap34xx_get_temp();	// Get CPU temp

	// Check values in case driver hasnt polled
	cputemp = (cputemp < 100) ? cputemp : 0;

	if (cputemp > ovrd_policy->temp_scaleback_high) {
		if (!ovrd_policy->overtemp_alarm) {
			printk("override: CPU temp warning! %dC\n", cputemp);
			ovrd_policy->overtemp_alarm = 1;
			change_freq_low(ovrd_policy->temp_scaleback_speed);
		}
	} else {
		if ((ovrd_policy->overtemp_alarm) &&
		    (cputemp < ovrd_policy->temp_scaleback_low)) {
			printk("override: CPU temp back under control! %dC\n",
			       cputemp);
			ovrd_policy->overtemp_alarm = 0;
			change_freq_high();
		}
	}
 out:
	schedule_delayed_work(&worker2, secs_to_jiffies(TEMP_POLLING));
}

static inline void check_battery(struct work_struct *work)
{
	int battery_per;

	if (ovrd_policy->overtemp_alarm)
		goto out;

	ds2784_getpercent(&battery_per);	// Get battery percent left
	battery_per = (battery_per > 0) ? battery_per : 100;

	if (battery_per < ovrd_policy->battery_scaleback_percent) {
		if (!ovrd_policy->battery_alarm) {
			printk("override: battery low! < %d%%\n", battery_per);
			ovrd_policy->battery_alarm = 1;
			change_freq_low(ovrd_policy->battery_scaleback_speed);
		}
	} else {
		if (ovrd_policy->battery_alarm) {
			printk("override: battery OK\n");
			ovrd_policy->battery_alarm = 0;
			change_freq_high();
		}
	}

 out:
	schedule_delayed_work(&worker, secs_to_jiffies(BATTERY_POLLING));
}

static ssize_t show_vdd1_vsel_max(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", VDD1_VSEL_MAX);
}

static ssize_t show_vdd1_vsel_min(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", VDD1_VSEL_MIN);
}

static ssize_t show_vdd1_vsel(struct cpufreq_policy *policy, char *buf)
{
	u8 volt[PRCM_NO_VDD1_OPPS];

	omap_pm_opp_get_volts(volt);

#ifdef CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
#if PRCM_NO_VDD1_OPPS > 5
#ifdef CONFIG_MACH_SIRLOIN_3630
	return sprintf(buf, "%hu %hu %hu %hu %hu %hu\n", volt[6],
		       volt[5], volt[4], volt[3], volt[2], volt[1]);
#else
	return sprintf(buf, "%hu %hu %hu %hu %hu\n", volt[6],
		       volt[5], volt[4], volt[3], volt[2]);
#endif
#else				// PRCM_NO_VDD1_OPPS > 5
#ifdef CONFIG_MACH_SIRLOIN_3630
	return sprintf(buf, "%hu %hu %hu %hu\n", volt[4], volt[3],
		       volt[2], volt[1]);
#else
	return sprintf(buf, "%hu %hu %hu\n", volt[4], volt[3], volt[2]);
#endif

#endif
#else				// CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
#if PRCM_NO_VDD1_OPPS > 5
	return sprintf(buf, "%hu %hu %hu %hu %hu %hu %hu\n", volt[6],
		       volt[5], volt[4], volt[3], volt[2], volt[1], volt[0]);
#else
	return sprintf(buf, "%hu %hu %hu %hu %hu\n", volt[4],
		       volt[3], volt[2], volt[1], volt[0]);
#endif
#endif
	return sprintf(buf, "N/A\n");
}

static ssize_t show_vdd2_vsel_max(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", VDD2_VSEL_MAX);
}

static ssize_t show_vdd2_vsel_min(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%hu\n", VDD2_VSEL_MIN);
}

static ssize_t show_vdd2_vsel(struct cpufreq_policy *policy, char *buf)
{
	u8 volt;

	omap_pm_opp_get_vdd2_volts(&volt);
	return sprintf(buf, "%hu\n", volt);
}

static ssize_t show_vdd2_freqs(struct cpufreq_policy *policy, char *buf)
{
	u8 freq;

	omap_pm_opp_get_vdd2_freq(&freq);
	return sprintf(buf, "%hu\n", freq);
}

static ssize_t
store_vdd1_vsel(struct cpufreq_policy *policy, const char *buf, size_t count)
{
	u8 volt[PRCM_NO_VDD1_OPPS], i;

	omap_pm_opp_get_volts(volt);

#if PRCM_NO_VDD1_OPPS > 5
	i = (sscanf(buf, "%hhu %hhu %hhu %hhu %hhu %hhu %hhu",
		    &volt[6], &volt[5], &volt[4],
		    &volt[3], &volt[2], &volt[1], &volt[0]));
#else
	i = (sscanf(buf, "%hhu %hhu %hhu %hhu %hhu",
		    &volt[4], &volt[3], &volt[2], &volt[1], &volt[0]));
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
#ifdef CONFIG_MACH_SIRLOIN_3630
	if (i == (PRCM_NO_VDD1_OPPS - 1)) {
#else				// CONFIG_MACH_SIRLOIN_3630
	if (i == (PRCM_NO_VDD1_OPPS - 2)) {
#endif
#else				// CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
	if (i == PRCM_NO_VDD1_OPPS) {
#endif

#ifdef CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
#ifdef CONFIG_MACH_SIRLOIN_3630
		for (i = 1; i < PRCM_NO_VDD1_OPPS; i++) {
#else				// CONFIG_MACH_SIRLOIN_3630
		for (i = 2; i < PRCM_NO_VDD1_OPPS; i++) {
#endif
#else				// CONFIG_CPU_FREQ_OVERRIDE_STRIPOPP
		for (i = 0; i < PRCM_NO_VDD1_OPPS; i++) {
#endif
			if ((volt[i] < VDD1_VSEL_MIN)
			    || (volt[i] > VDD1_VSEL_MAX)) {
				printk("override: invalid vsel\n");
				break;
			}
		}
		if (i == PRCM_NO_VDD1_OPPS) {
			omap_pm_opp_set_volts(volt);
		}
	} else
		printk("override: missing vsel values\n");

	return count;
}

static ssize_t
store_vdd2_vsel(struct cpufreq_policy *policy, const char *buf, size_t count)
{
	u8 volt;

	if (sscanf(buf, "%hhu", &volt) == 1) {
		if ((volt < VDD2_VSEL_MIN) || (volt > VDD2_VSEL_MAX)) {
			printk("override: invalid vsel\n");
		} else
			omap_pm_opp_set_vdd2_volts(volt);
	} else
		printk("override: missing vsel values\n");

	return count;
}

static ssize_t
show_hightemp_scaleback(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", ovrd_policy->temp_scaleback_high);
}

static ssize_t
store_hightemp_scaleback(struct cpufreq_policy *policy,
			 const char *buf, size_t count)
{
	unsigned int maxtemp = 0;

	if (sscanf(buf, "%u", &maxtemp) == 1)
		ovrd_policy->temp_scaleback_high = (maxtemp < 60)
		    ? maxtemp : HIGHTEMP_SCALEBACK;
	else
		printk("override: invalid max temp\n");

	return count;
}

static ssize_t
show_battery_scaleback_per(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", ovrd_policy->battery_scaleback_percent);
}

static ssize_t
store_battery_scaleback_per(struct cpufreq_policy *policy,
			    const char *buf, size_t count)
{
	unsigned int bat = 0;

	if (sscanf(buf, "%u", &bat) == 1)
		ovrd_policy->battery_scaleback_percent = (bat < 101)
		    ? bat : BATTERY_PERCENT;
	else
		printk("override: invalid battery percentage\n");

	return count;
}

static ssize_t
show_battery_scaleback_speed(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", ovrd_policy->battery_scaleback_speed);
}

static ssize_t
store_battery_scaleback_speed(struct cpufreq_policy *policy,
			      const char *buf, size_t count)
{
	unsigned int bat = 0;

	if (sscanf(buf, "%u", &bat) == 1)
		ovrd_policy->battery_scaleback_speed = (check_freq(bat))
		    ? bat : SCALEBACK_SPEED;
	else
		printk("override: invalid battery scaleback speed\n");

	return count;
}

static ssize_t
show_lowtemp_reset(struct cpufreq_policy *policy, char *buf)
{

	return sprintf(buf, "%u\n", ovrd_policy->temp_scaleback_low);
}

static ssize_t
store_lowtemp_reset(struct cpufreq_policy *policy,
		    const char *buf, size_t count)
{
	unsigned int lowtemp = 0;

	if (sscanf(buf, "%u", &lowtemp) == 1)
		ovrd_policy->temp_scaleback_low =
		    (lowtemp < 60) ? lowtemp : LOWTEMP_RESET;
	else
		printk("override: invalid low temp\n");

	return count;
}

static ssize_t
show_temp_scaleback(struct cpufreq_policy *policy, char *buf)
{

	return sprintf(buf, "%u\n", ovrd_policy->temp_scaleback_speed);
}

static ssize_t
store_temp_scaleback(struct cpufreq_policy *policy,
		     const char *buf, size_t count)
{
	unsigned int tempscaleback = 0;

	if (sscanf(buf, "%u", &tempscaleback) == 1)
		ovrd_policy->temp_scaleback_speed = (check_freq(tempscaleback))
		    ? tempscaleback : SCALEBACK_SPEED;
	else
		printk("override: invalid scaleback speed\n");

	return count;
}

CPUFREQ_OVERRIDE_ATTR(vdd1_vsel, vdd1_vsel);
CPUFREQ_OVERRIDE_ATTR(vdd2_vsel, vdd2_vsel);
CPUFREQ_OVERRIDE_ATTR(battery_scaleback_percent, battery_scaleback_per);
CPUFREQ_OVERRIDE_ATTR(battery_scaleback_speed, battery_scaleback_speed);
CPUFREQ_OVERRIDE_ATTR2(vdd1_vsel_min, show_vdd1_vsel_min);
CPUFREQ_OVERRIDE_ATTR2(vdd1_vsel_max, show_vdd1_vsel_max);
CPUFREQ_OVERRIDE_ATTR2(vdd2_vsel_min, show_vdd2_vsel_min);
CPUFREQ_OVERRIDE_ATTR2(vdd2_vsel_max, show_vdd2_vsel_max);
CPUFREQ_OVERRIDE_ATTR2(vdd2_freq, show_vdd2_freqs);
CPUFREQ_OVERRIDE_ATTR(cpu_hightemp_alarm, hightemp_scaleback);
CPUFREQ_OVERRIDE_ATTR(cpu_hightemp_reset, lowtemp_reset);
CPUFREQ_OVERRIDE_ATTR(cpu_hightemp_scaleback_speed, temp_scaleback);

static struct attribute *default_attrs[] = {
	&_attr_vdd1_vsel.attr,
	&_attr_vdd1_vsel_min.attr,
	&_attr_vdd1_vsel_max.attr,
	&_attr_vdd2_vsel.attr,
	&_attr_vdd2_vsel_min.attr,
	&_attr_vdd2_vsel_max.attr,
	&_attr_vdd2_freq.attr,
	&_attr_cpu_hightemp_alarm.attr,
	&_attr_cpu_hightemp_reset.attr,
	&_attr_cpu_hightemp_scaleback_speed.attr,
	&_attr_battery_scaleback_percent.attr,
	&_attr_battery_scaleback_speed.attr,
	NULL
};

static struct attribute_group override_attr_group = {
	.attrs = default_attrs,
	.name = "override"
};

int cpufreq_override_driver_init(void)
{
	struct cpufreq_policy *data = cpufreq_cpu_get(0);
	schedule_delayed_work(&worker, secs_to_jiffies(BATTERY_POLLING));
	schedule_delayed_work(&worker2, secs_to_jiffies(TEMP_POLLING));
	printk("override: initialized!\n");
	return sysfs_create_group(&data->kobj, &override_attr_group);
}

EXPORT_SYMBOL(cpufreq_override_driver_init);

void cpufreq_override_driver_exit(void)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	cancel_delayed_work(&worker);
	cancel_delayed_work(&worker2);
	sysfs_remove_group(&policy->kobj, &override_attr_group);
}

EXPORT_SYMBOL(cpufreq_override_driver_exit);

MODULE_AUTHOR("marco@unixpsycho.com");
MODULE_DESCRIPTION("'cpufreq_override' - A driver to do cool stuff ");
MODULE_LICENSE("GPL");
