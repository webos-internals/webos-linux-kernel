/*
 *  drivers/cpufreq/cpufreq_ondemand_tickle.c
 *
 *  A version of cpufreq_ondemand supporing hinting, or tickling, into
 *  high performance levels based on platform defined events. This governor
 *  also supports setting a temporary frequency floor for maintaining
 *  minimum required performance levels while still conserving power, such
 *  as may be required in codecs, com stacks, etc. Ondemand_tickle should
 *  behave identically to ondemand when neither a tickel nor a floor is active.
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Palm Inc, Corey Tabaka <corey.tabaka@palm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq_tickle.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/list.h>

/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_FREQUENCY_DOWN_DIFFERENTIAL		(10)
#define DEF_FREQUENCY_UP_THRESHOLD		(80)
#define MICRO_FREQUENCY_DOWN_DIFFERENTIAL	(3)
#define MICRO_FREQUENCY_UP_THRESHOLD		(95)
#define MIN_FREQUENCY_UP_THRESHOLD		(11)
#define MAX_FREQUENCY_UP_THRESHOLD		(100)
#define MIN_FREQUENCY_DOWN_DIFFERENTIAL		(1)

/*
 * The max and default time in mS to keep the processor at max freq from a tickle.
 */
#define MAX_TICKLE_WINDOW				(10000)
#define DEF_TICKLE_WINDOW				(3000)

/*
 * The max and default time in mS to keep the processor at at least the floor freq.
 */
#define MAX_FLOOR_WINDOW				(10000)
#define DEF_FLOOR_WINDOW				(3000)

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate sampling
 * rate.
 * For CPUs with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work.
 * All times here are in uS.
 */
static unsigned int def_sampling_rate;
#define MIN_SAMPLING_RATE_RATIO			(2)
/* for correct statistics, we need at least 10 ticks between each measure */
#define MIN_STAT_SAMPLING_RATE 			\
	(MIN_SAMPLING_RATE_RATIO * jiffies_to_usecs(CONFIG_CPU_FREQ_MIN_TICKS))
#define MIN_SAMPLING_RATE			\
			(def_sampling_rate / MIN_SAMPLING_RATE_RATIO)
#define MAX_SAMPLING_RATE			(500 * def_sampling_rate)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)

static void do_dbs_timer(struct work_struct *work);

static void do_tickle_timer(unsigned long arg);
static void do_floor_timer(unsigned long arg);

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
 	struct delayed_work work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int freq_lo;
	unsigned int freq_lo_jiffies;
	unsigned int freq_hi_jiffies;

	int tickle_active;
	int floor_active;
	unsigned int freq_floor;
	unsigned int freq_save;
	unsigned int rel_save;
	int cur_load;
	unsigned int max_load_freq;
	
	int cpu;
	unsigned int enable:1,
	             sample_type:1;
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cpu_dbs_info);

static unsigned int dbs_enable;	/* number of CPUs using this policy */

static void adjust_for_load(struct cpu_dbs_info_s *this_dbs_info);

/*
 * DEADLOCK ALERT! There is a ordering requirement between cpu_hotplug
 * lock and dbs_mutex. cpu_hotplug lock should always be held before
 * dbs_mutex. If any function that can potentially take cpu_hotplug lock
 * (like __cpufreq_driver_target()) is being called with dbs_mutex taken, then
 * cpu_hotplug lock should be taken before that. Note that cpu_hotplug lock
 * is recursive for the same process. -Venki
 */
static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct	*kondemand_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int down_differential;
	unsigned int ignore_nice;
	unsigned int powersave_bias;
	unsigned int max_tickle_window;
	unsigned int max_floor_window;
} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
	.down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
	.ignore_nice = 0,
	.powersave_bias = 0,
	.max_tickle_window = DEF_TICKLE_WINDOW,
	.max_floor_window = DEF_FLOOR_WINDOW,
};

/*
 * tickle/floor state
 */
static struct {
	spinlock_t lock;

	int active;
	int active_tickle;
	int active_count;
	unsigned long tickle_jiffies;


	int floor_active;
	int floor_tickle;
	int floor_count;
	unsigned long floor_jiffies;
	unsigned int cur_freq_floor;

	struct work_struct tickle_work;
	struct work_struct floor_work;

	struct timer_list tickle_timer;
	struct timer_list floor_timer;
} tickle_state = {
	.lock = SPIN_LOCK_UNLOCKED,
	.active = 0,
	.active_tickle = 0,
	.active_count = 0,
	.floor_active = 0,
	.floor_tickle = 0,
	.floor_count = 0,
	.cur_freq_floor = 0,
};

/*
 * Stats for profiling response characteristics.
 */
#define NUM_SAMPLES 10000
struct sample_data {
	cputime64_t timestamp;
	cputime64_t user;
	cputime64_t system;
	cputime64_t irq;
	cputime64_t softirq;
	cputime64_t steal;
	cputime64_t nice;
	unsigned int cur_freq;
	unsigned int target_freq;
	int load;
};

struct sample_stats {
	struct sample_data *samples;
	unsigned int current_sample;
};

static DEFINE_PER_CPU(struct sample_stats, sample_stats);

static int sampling_enabled = 0;
module_param(sampling_enabled, bool, S_IRUGO | S_IWUSR);

static int tickling_enabled = 1;
module_param(tickling_enabled, bool, S_IRUGO | S_IWUSR);

static int clear_samples = 0;
module_param(clear_samples, bool, S_IRUGO | S_IWUSR);

static int print_tickles = 0;
module_param(print_tickles, bool, S_IRUGO | S_IWUSR);

/********************* procfs ********************/
struct stats_state {
	int cpu;
	int sample_index;
	int num_samples;
};

static struct seq_operations stats_op;

static void *stats_start(struct seq_file *, loff_t *);
static void *stats_next(struct seq_file *, void *v, loff_t *);
static void stats_stop(struct seq_file *, void *);
static int stats_show(struct seq_file *, void *);

static int stats_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &stats_op);
}

static struct file_operations proc_stats_operations = {
	.open		= stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static struct seq_operations stats_op = {
	.start		= stats_start,
	.next		= stats_next,
	.stop		= stats_stop,
	.show		= stats_show,
};

static int get_num_sample_records(void) {
	int i, count = 0;

	for (i = 0; i < NR_CPUS; i++) {
		struct sample_stats *stats = &per_cpu(sample_stats, i);

		if (stats->samples)
			count += NUM_SAMPLES;
	}

	return count;
}

static void *stats_start(struct seq_file *m, loff_t *pos)
{	
	struct stats_state *state = kmalloc(sizeof(struct stats_state),  GFP_KERNEL);

	printk(KERN_DEBUG "%s: %u\n", __FUNCTION__, (unsigned int) *pos);
	
	if (!state)
		return ERR_PTR(-ENOMEM);

	state->cpu = 0;
	state->sample_index = *pos;
	state->num_samples = get_num_sample_records();

	if (*pos >= state->num_samples)
		return NULL;
	
	while (state->sample_index > NUM_SAMPLES) {
		struct sample_stats *stats = &per_cpu(sample_stats, state->cpu);

		state->cpu++;
		if (stats->samples)
			state->sample_index -= NUM_SAMPLES;
	}

	return (void *) state;
}

static void *stats_next(struct seq_file *m, void *v, loff_t *pos)
{	
	struct stats_state *state = (struct stats_state *) v;
	
	printk(KERN_DEBUG "%s: %u\n" ,__FUNCTION__, (unsigned int) *pos);
	
	++*pos;
	
	if (*pos >= state->num_samples)
		return NULL;

	state->cpu = 0;
	state->sample_index = *pos;

	while (state->sample_index > NUM_SAMPLES) {
		struct sample_stats *stats = &per_cpu(sample_stats, state->cpu);

		state->cpu++;
		if (stats->samples)
			state->sample_index -= NUM_SAMPLES;
	}

	return (void *) state;
}

static void stats_stop(struct seq_file *m, void *v)
{
	kfree(v);
}

static int stats_show(struct seq_file *m, void *v)
{
	struct stats_state *state = v;
	struct sample_stats *stats = &per_cpu(sample_stats, state->cpu);
	int rc = 0;

	if (state->sample_index == 0) {
		rc = seq_printf(m, "cpufreq samples for cpu %u\n", state->cpu);
		if (rc)
			goto done;
	}
	
	if (stats->samples) {
		rc = seq_printf(m, "%u %u: %llu %llu %llu %llu %llu %llu %llu %d %u %u\n",
				state->cpu,
				state->sample_index,
				stats->samples[state->sample_index].timestamp,
				stats->samples[state->sample_index].user,
				stats->samples[state->sample_index].system,
				stats->samples[state->sample_index].irq,
				stats->samples[state->sample_index].softirq,
				stats->samples[state->sample_index].steal,
				stats->samples[state->sample_index].nice,
				stats->samples[state->sample_index].load,
				stats->samples[state->sample_index].cur_freq,
				stats->samples[state->sample_index].target_freq
		);
		if (rc)
			goto done;
	}

done:
	return rc;
}
/******************** end procfs ********************/

/********************** ioctl ***********************/

static struct class *tickle_class = NULL;
static struct cdev tickle_cdev;
static dev_t tickle_dev;

static LIST_HEAD(tickle_clients);

struct tickle_file_data {
	struct list_head list;

	int tickle_hold_flag;
	int floor_hold_flag;
	unsigned int floor_freq;
};

/* static tickle_file_data entry for use by anonymous floor tickles that don't have a file handle */
static struct tickle_file_data default_file_data;

static int get_max_client_floor(void);

static int tickle_open(struct inode *inode, struct file *filp);
static int tickle_release(struct inode *inode, struct file *filp);
static int tickle_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);

static struct file_operations tickle_fops = {
	.owner			= THIS_MODULE,
	.open			= tickle_open,
	.release		= tickle_release,
	.ioctl			= tickle_ioctl,
};

static int setup_tickle_device(void)
{
	int res = 0;
	struct device *dev = NULL;

	res = alloc_chrdev_region(&tickle_dev, 0, 1, "ondemandtcl");
	if (res < 0) {
		printk(KERN_ERR "%s: can't alloc major number (%d)\n", __FILE__, res);
		goto error;
	}
	
	tickle_class = class_create(THIS_MODULE, "ondemandtcl");
	if (IS_ERR(tickle_class)) {
		res = PTR_ERR(tickle_class);
		printk(KERN_ERR "%s: can't create class (%d)\n", __FILE__, res);
		goto error_unregister_region;
	}

	cdev_init(&tickle_cdev, &tickle_fops);
	tickle_cdev.owner = THIS_MODULE;
	res = cdev_add(&tickle_cdev, tickle_dev, 1);
	if (res < 0) {
		printk(KERN_ERR "%s: failed to add device (%d)\n", __FILE__, res);
		goto error_remove_class;
	}

	/* always use ondemandtcl0, since userspace is already depending on that name */
	dev = device_create(tickle_class, NULL, tickle_dev, NULL, "ondemandtcl%d", 0);
	if (IS_ERR(dev)) {
		res = PTR_ERR(dev);
		printk(KERN_ERR "%s: failed to create device (%d)\n", __FILE__, res);

		goto error_remove_cdev;
	}
	/* add the default file data for anonymous clients */
	default_file_data.floor_freq = 0;
	list_add(&default_file_data.list, &tickle_clients);

	return res;

error_remove_cdev:
	cdev_del(&tickle_cdev);

error_remove_class:
	class_destroy(tickle_class);
	tickle_class = NULL;

error_unregister_region:
	unregister_chrdev_region(tickle_dev, 1);
	
error:
	return res;
}

static void remove_tickle_device(void)
{
	if (tickle_class) {
		device_destroy(tickle_class, tickle_dev);
		class_destroy(tickle_class);
		cdev_del(&tickle_cdev);
		unregister_chrdev_region(tickle_dev, 1);

		/* for posterity */
		list_del(&default_file_data.list);
	}
}

static int tickle_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct tickle_file_data *data = kzalloc(sizeof(struct tickle_file_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	//printk("%s\n", __FUNCTION__);

	spin_lock_irqsave(&tickle_state.lock, flags);
	list_add(&data->list, &tickle_clients);
	spin_unlock_irqrestore(&tickle_state.lock, flags);

	filp->private_data = data;

	return 0;
}

static int tickle_release(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct tickle_file_data *data = filp->private_data;	

	//printk("%s\n", __FUNCTION__);

	if (data) {
		spin_lock_irqsave(&tickle_state.lock, flags);
		list_del(&data->list);
		spin_unlock_irqrestore(&tickle_state.lock, flags);

		CPUFREQ_UNHOLD_CHECK(&data->tickle_hold_flag);
		CPUFREQ_FLOOR_UNHOLD_CHECK(&data->floor_hold_flag);

		kfree(data);
	}

	return 0;
}

static int tickle_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	struct tickle_file_data *data = filp->private_data;

	//printk("%s: cmd = %d, arg = %ld\n", __FUNCTION__, cmd, arg);
	
	if (_IOC_TYPE(cmd) != TICKLE_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > TICKLE_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
		case TICKLE_IOCT_TICKLE:
			CPUFREQ_TICKLE_MILLIS((unsigned int) arg);
			break;

		case TICKLE_IOCT_FLOOR:
			CPUFREQ_FLOOR((unsigned int) arg);
			break;

		case TICKLE_IOC_TICKLE_HOLD:
			CPUFREQ_HOLD_CHECK(&data->floor_hold_flag);
			break;

		case TICKLE_IOC_TICKLE_UNHOLD:
			CPUFREQ_UNHOLD_CHECK(&data->floor_hold_flag);
			break;

		case TICKLE_IOCT_FLOOR_HOLD:
			spin_lock_irqsave(&tickle_state.lock, flags);
			data->floor_freq = (unsigned int) arg;
			spin_unlock_irqrestore(&tickle_state.lock, flags);

			CPUFREQ_FLOOR_HOLD_CHECK((unsigned int) arg, &data->floor_hold_flag);
			break;

		case TICKLE_IOC_FLOOR_UNHOLD:
			spin_lock_irqsave(&tickle_state.lock, flags);
			data->floor_freq = 0;
			spin_unlock_irqrestore(&tickle_state.lock, flags);

			CPUFREQ_FLOOR_UNHOLD_CHECK(&data->floor_hold_flag);
			break;

		case TICKLE_IOC_TICKLE_HOLD_SYNC:
			CPUFREQ_HOLD_SYNC();
			break;

		default:
			return -ENOTTY;
	}

	return 0;
}

/* This needs to be called with spinlock tickle_state.lock held */
static int get_max_client_floor(void)
{
	int floor_freq = 0;
	struct tickle_file_data *file_data;

	list_for_each_entry(file_data, &tickle_clients, list) {
		if (file_data->floor_freq > floor_freq)
			floor_freq = file_data->floor_freq;
	}

	return floor_freq;
}

/******************** end ioctl *********************/

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
							cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
			kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = cur_wall_time;

	return idle_time;
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

/*
 * Find right freq to be set now with powersave_bias on.
 * Returns the freq_hi to be used right now and will set freq_hi_jiffies,
 * freq_lo, and freq_lo_jiffies in percpu area for averaging freqs.
 */
static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
					  unsigned int freq_next,
					  unsigned int relation)
{
	unsigned int freq_req, freq_reduc, freq_avg;
	unsigned int freq_hi, freq_lo;
	unsigned int index = 0;
	unsigned int jiffies_total, jiffies_hi, jiffies_lo;
	struct cpu_dbs_info_s *dbs_info = &per_cpu(cpu_dbs_info, policy->cpu);

	if (!dbs_info->freq_table) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_next;
	}

	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
			relation, &index);
	freq_req = dbs_info->freq_table[index].frequency;
	freq_reduc = freq_req * dbs_tuners_ins.powersave_bias / 1000;
	freq_avg = freq_req - freq_reduc;

	/* Find freq bounds for freq_avg in freq_table */
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_H, &index);
	freq_lo = dbs_info->freq_table[index].frequency;
	index = 0;
	cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
			CPUFREQ_RELATION_L, &index);
	freq_hi = dbs_info->freq_table[index].frequency;

	/* Find out how long we have to be in hi and lo freqs */
	if (freq_hi == freq_lo) {
		dbs_info->freq_lo = 0;
		dbs_info->freq_lo_jiffies = 0;
		return freq_lo;
	}
	jiffies_total = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
	jiffies_hi += ((freq_hi - freq_lo) / 2);
	jiffies_hi /= (freq_hi - freq_lo);
	jiffies_lo = jiffies_total - jiffies_hi;
	dbs_info->freq_lo = freq_lo;
	dbs_info->freq_lo_jiffies = jiffies_lo;
	dbs_info->freq_hi_jiffies = jiffies_hi;
	return freq_hi;
}

static void ondemand_powersave_bias_init(void)
{
	int i;
	for_each_online_cpu(i) {
		struct cpu_dbs_info_s *dbs_info = &per_cpu(cpu_dbs_info, i);
		dbs_info->freq_table = cpufreq_frequency_get_table(i);
		dbs_info->freq_lo = 0;
	}
}

/************************** sysfs interface ************************/
static ssize_t show_sampling_rate_max(struct cpufreq_policy *policy, char *buf)
{
	return sprintf (buf, "%u\n", MAX_SAMPLING_RATE);
}

static ssize_t show_sampling_rate_min(struct cpufreq_policy *policy, char *buf)
{
	return sprintf (buf, "%u\n", MIN_SAMPLING_RATE);
}

#define define_one_ro(_name)		\
static struct freq_attr _name =		\
__ATTR(_name, 0444, show_##_name, NULL)

define_one_ro(sampling_rate_max);
define_one_ro(sampling_rate_min);

/* cpufreq_ondemand Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct cpufreq_policy *unused, char *buf)				\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(up_threshold, up_threshold);
show_one(down_differential, down_differential);
show_one(ignore_nice_load, ignore_nice);
show_one(powersave_bias, powersave_bias);
show_one(max_tickle_window, max_tickle_window);
show_one(max_floor_window, max_floor_window);

static ssize_t store_sampling_rate(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	mutex_lock(&dbs_mutex);
	if (ret != 1 || input > MAX_SAMPLING_RATE
		     || input < MIN_SAMPLING_RATE) {
		mutex_unlock(&dbs_mutex);
		return -EINVAL;
	}

	dbs_tuners_ins.sampling_rate = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_up_threshold(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	mutex_lock(&dbs_mutex);
	if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
			input < MIN_FREQUENCY_UP_THRESHOLD) {
		mutex_unlock(&dbs_mutex);
		return -EINVAL;
	}

	dbs_tuners_ins.up_threshold = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_down_differential(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	mutex_lock(&dbs_mutex);
	if (ret != 1 || input >= dbs_tuners_ins.up_threshold ||
			input < MIN_FREQUENCY_DOWN_DIFFERENTIAL) {
		mutex_unlock(&dbs_mutex);
		return -EINVAL;
	}

	dbs_tuners_ins.down_differential = input;
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_ignore_nice_load(struct cpufreq_policy *policy,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if ( ret != 1 )
		return -EINVAL;

	if ( input > 1 )
		input = 1;

	mutex_lock(&dbs_mutex);
	if ( input == dbs_tuners_ins.ignore_nice ) { /* nothing to do */
		mutex_unlock(&dbs_mutex);
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;

	}
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_powersave_bias(struct cpufreq_policy *unused,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 1000)
		input = 1000;

	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.powersave_bias = input;
	ondemand_powersave_bias_init();
	mutex_unlock(&dbs_mutex);

	return count;
}

static ssize_t store_max_tickle_window(struct cpufreq_policy *unuesd,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	
	if (ret != 1)
		return -EINVAL;
	
	if (input > MAX_TICKLE_WINDOW)
		input = MAX_TICKLE_WINDOW;
	
	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.max_tickle_window = input;
	mutex_unlock(&dbs_mutex);
	
	return count;
}

static ssize_t store_max_floor_window(struct cpufreq_policy *unuesd,
		const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	
	if (ret != 1)
		return -EINVAL;
	
	if (input > MAX_FLOOR_WINDOW)
		input = MAX_FLOOR_WINDOW;
	
	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.max_floor_window = input;
	mutex_unlock(&dbs_mutex);
	
	return count;
}

#define define_one_rw(_name) \
static struct freq_attr _name = \
__ATTR(_name, 0644, show_##_name, store_##_name)

define_one_rw(sampling_rate);
define_one_rw(up_threshold);
define_one_rw(down_differential);
define_one_rw(ignore_nice_load);
define_one_rw(powersave_bias);
define_one_rw(max_tickle_window);
define_one_rw(max_floor_window);

static struct attribute * dbs_attributes[] = {
	&sampling_rate_max.attr,
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
	&down_differential.attr,
	&ignore_nice_load.attr,
	&powersave_bias.attr,
	&max_tickle_window.attr,
	&max_floor_window.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "ondemandtcl",
};

/************************** sysfs end ************************/

static void record_sample(unsigned int cur_freq, unsigned int target_freq,
		int load, int cpu)
{
	struct sample_stats *stats;
	cputime64_t cur_jiffies;
	int i;
	
	/* abuse a module parameter to trigger clearing the sample buffer */
	if (clear_samples) {
		for_each_online_cpu(i) {
			stats = &per_cpu(sample_stats, i);
			if (stats->samples)
				memset(stats->samples, 0, sizeof(struct sample_data) * NUM_SAMPLES);
			stats->current_sample = 0;
		}
		clear_samples = 0;
	}
	
	if (!sampling_enabled)
		return;
	
	cur_jiffies = jiffies64_to_cputime64(get_jiffies_64());
	stats = &per_cpu(sample_stats, cpu);
	
	if (!stats->samples)
		return;

	//if (target_freq != cur_freq) {
		i = stats->current_sample;
		stats->samples[i].timestamp = cur_jiffies;
		stats->samples[i].user = kstat_cpu(cpu).cpustat.user;
		stats->samples[i].system = kstat_cpu(cpu).cpustat.system;
		stats->samples[i].irq = kstat_cpu(cpu).cpustat.irq;
		stats->samples[i].softirq = kstat_cpu(cpu).cpustat.softirq;
		stats->samples[i].steal = kstat_cpu(cpu).cpustat.steal;
		stats->samples[i].nice = kstat_cpu(cpu).cpustat.nice;
		stats->samples[i].cur_freq = cur_freq;
		stats->samples[i].target_freq = target_freq;
		stats->samples[i].load = load;
		stats->current_sample = (i + 1) % NUM_SAMPLES;
	//}
}

static void do_tickle_state_change(struct work_struct *work)
{
	int cpu;
	unsigned long flags;
	int active, active_tickle, active_count;
	unsigned long tickle_jiffies;

	spin_lock_irqsave(&tickle_state.lock, flags);
	
	active = tickle_state.active;
	active_count = tickle_state.active_count;
	active_tickle = tickle_state.active_tickle;
	tickle_jiffies = tickle_state.tickle_jiffies;

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	if (print_tickles)
		printk("%s: active=%u, active_count=%u, tickle_jiffies=%lu\n",
				__FUNCTION__, active, active_count, tickle_jiffies);

	if (active_count) {
		if (!active) {
			for_each_online_cpu(cpu) {
				struct cpu_dbs_info_s *dbs_info;
				struct cpufreq_policy *policy;

				if (lock_policy_rwsem_write(cpu) < 0)
					continue;

				mutex_lock(&dbs_mutex);

				dbs_info = &per_cpu(cpu_dbs_info, cpu);
				policy = dbs_info->cur_policy;

				/* if we don't have a policy then we probably got tickled before setup completed */
				if (!policy || !dbs_info->enable) {
					mutex_unlock(&dbs_mutex);
					unlock_policy_rwsem_write(cpu);
					continue;
				}

				/* save the current operating frequency */	
				dbs_info->freq_save = policy->cur;

				/* ramp up to the policy max */
				if (policy->cur < policy->max) {
					record_sample(policy->cur, policy->max, -2, policy->cpu);
					__cpufreq_driver_target(policy, policy->max, CPUFREQ_RELATION_H);
				}

				dbs_info->tickle_active = 1;

				mutex_unlock(&dbs_mutex);
				unlock_policy_rwsem_write(cpu);
			}
		}
		
		if (active_tickle)
			mod_timer(&tickle_state.tickle_timer, tickle_jiffies);

		active = 1;
	} else if (active) {

		for_each_online_cpu(cpu) {
			struct cpu_dbs_info_s *dbs_info;
			struct cpufreq_policy *policy;

			if (lock_policy_rwsem_write(cpu) < 0)
				continue;

			mutex_lock(&dbs_mutex);

			dbs_info = &per_cpu(cpu_dbs_info, cpu);
			policy = dbs_info->cur_policy;

			if (!dbs_info->enable) {
				mutex_unlock(&dbs_mutex);
				unlock_policy_rwsem_write(cpu);
				continue;
			}

			dbs_info->tickle_active = 0;
			adjust_for_load(dbs_info);

			record_sample(policy->cur, policy->min, -5, policy->cpu);
			mutex_unlock(&dbs_mutex);
			unlock_policy_rwsem_write(cpu);
		}

		active = 0;
	}
	
	/* update tickle_state */
	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.active = active;
	spin_unlock_irqrestore(&tickle_state.lock, flags);
}

void cpufreq_ondemand_tickle_millis(unsigned int millis)
{
	int queue = 0;
	unsigned long flags, expire;

	if (!tickling_enabled)
		return;

	if (millis > dbs_tuners_ins.max_tickle_window)
		millis = dbs_tuners_ins.max_tickle_window;
	
	expire = jiffies + msecs_to_jiffies(millis);

	spin_lock_irqsave(&tickle_state.lock, flags);
	
	if (time_after(expire, tickle_state.tickle_jiffies)) {
		tickle_state.tickle_jiffies = expire;

		if (!tickle_state.active_tickle)
			tickle_state.active_count += 1;
		
		tickle_state.active_tickle = 1;
		queue = 1;
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);
	
	if (queue)
		queue_work(kondemand_wq, &tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_tickle_millis);

void cpufreq_ondemand_tickle(void)
{
	cpufreq_ondemand_tickle_millis(dbs_tuners_ins.max_tickle_window);
}
EXPORT_SYMBOL(cpufreq_ondemand_tickle);

void cpufreq_ondemand_hold(void)
{
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.active_count += 1;
	spin_unlock_irqrestore(&tickle_state.lock, flags);

	queue_work(kondemand_wq, &tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_hold);

void cpufreq_ondemand_unhold(void)
{
	int queue = 0;
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);

	if (tickle_state.active_count) {
		tickle_state.active_count -= 1;
		queue = 1;
	} else {
		printk(KERN_WARNING "%s: attempt to decrement when active_count == 0!\n",
				__FUNCTION__);
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	if (queue)
		queue_work(kondemand_wq, &tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_unhold);

void cpufreq_ondemand_hold_check(int *flag)
{
	if (!*flag) {
		cpufreq_ondemand_hold();
		*flag = 1;
	}
}
EXPORT_SYMBOL(cpufreq_ondemand_hold_check);

/* this tickle option waits until the CPU is up to the max freq before it returns */
void cpufreq_ondemand_hold_sync(void)
{
	cpufreq_ondemand_hold();
	flush_work(&tickle_state.tickle_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_hold_sync);

void cpufreq_ondemand_unhold_check(int *flag)
{
	if (*flag) {
		cpufreq_ondemand_unhold();
		*flag = 0;
	}
}
EXPORT_SYMBOL(cpufreq_ondemand_unhold_check);

static void do_floor_state_change(struct work_struct *work)
{
	int cpu, floor_active, floor_tickle, floor_count, cur_freq_floor, pending_freq_floor;
	unsigned long flags, floor_jiffies;

	spin_lock_irqsave(&tickle_state.lock, flags);

	floor_active = tickle_state.floor_active;
	floor_tickle = tickle_state.floor_tickle;
	floor_count = tickle_state.floor_count;
	cur_freq_floor = tickle_state.cur_freq_floor;
	floor_jiffies = tickle_state.floor_jiffies;

	pending_freq_floor = get_max_client_floor();

	spin_unlock_irqrestore(&tickle_state.lock, flags);


	if (print_tickles)
		printk("%s: floor_active=%u, floor_tickle=%u, floor_count=%u, floor_jiffies=%lu, "
				"cur_freq_floor=%u, pending_freq_floor=%u\n",
				__FUNCTION__, floor_active, floor_tickle, floor_count, floor_jiffies,
				cur_freq_floor, pending_freq_floor);

	if (floor_count) {
		if (!floor_active || pending_freq_floor != cur_freq_floor) {

			for_each_online_cpu(cpu) {
				unsigned int f = pending_freq_floor;
				struct cpu_dbs_info_s *dbs_info;
				struct cpufreq_policy *policy;

				if (lock_policy_rwsem_write(cpu) < 0)
					continue;

				mutex_lock(&dbs_mutex);

				dbs_info = &per_cpu(cpu_dbs_info, cpu);
				policy = dbs_info->cur_policy;

				if (!policy || !dbs_info->enable) {
					mutex_unlock(&dbs_mutex);
					unlock_policy_rwsem_write(cpu);
					continue;
				}

				if (policy->min >= f) {
					mutex_unlock(&dbs_mutex);
					unlock_policy_rwsem_write(cpu);
					continue;
				}
				
				if (f > policy->max)
					f = policy->max;

				dbs_info->freq_floor = f;
				dbs_info->freq_save = policy->cur;
				dbs_info->floor_active = 1;

				if (policy->cur < f) {
					record_sample(policy->cur, f, -3, policy->cpu);
					__cpufreq_driver_target(policy, f, CPUFREQ_RELATION_H);
				}

				mutex_unlock(&dbs_mutex);
				unlock_policy_rwsem_write(cpu);
			}

		}
		
		if (floor_tickle)
			mod_timer(&tickle_state.floor_timer, floor_jiffies);

		floor_active = 1;
		cur_freq_floor = pending_freq_floor;
	} else if (floor_active) {

		for_each_online_cpu(cpu) {
			struct cpu_dbs_info_s *dbs_info;
			struct cpufreq_policy *policy;

			if (lock_policy_rwsem_write(cpu) < 0)
				continue;

			mutex_lock(&dbs_mutex);

			dbs_info = &per_cpu(cpu_dbs_info, cpu);
			policy = dbs_info->cur_policy;

			if (!dbs_info->enable) {
				mutex_unlock(&dbs_mutex);
				unlock_policy_rwsem_write(cpu);
				continue;
			}

			dbs_info->floor_active = 0;
			__cpufreq_driver_target(policy, dbs_info->freq_save, dbs_info->rel_save);
			record_sample(dbs_info->freq_save, policy->min, -5, policy->cpu);

			mutex_unlock(&dbs_mutex);
			unlock_policy_rwsem_write(cpu);
		}


		floor_active = 0;
		cur_freq_floor = 0;
	}

	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.floor_active = floor_active;
	tickle_state.cur_freq_floor = cur_freq_floor;
	spin_unlock_irqrestore(&tickle_state.lock, flags);
}	

void cpufreq_ondemand_floor_millis(unsigned int freq, unsigned int millis)
{
	int queue = 0;
	unsigned long flags, expire;

	if (!tickling_enabled)
		return;

	if (millis > dbs_tuners_ins.max_floor_window)
		millis = dbs_tuners_ins.max_floor_window;
	
	expire = jiffies + msecs_to_jiffies(millis);

	spin_lock_irqsave(&tickle_state.lock, flags);
	
	if (time_after(expire, tickle_state.floor_jiffies) ||
			(freq >  get_max_client_floor() && freq > tickle_state.cur_freq_floor)) {
		if (time_after(expire, tickle_state.floor_jiffies))
			tickle_state.floor_jiffies = expire;
		
		if (!tickle_state.floor_tickle)
			tickle_state.floor_count += 1;
		
		tickle_state.floor_tickle = 1;
		
		default_file_data.floor_freq = max(default_file_data.floor_freq, freq);

		queue = 1;
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);
	
	if (queue)
		queue_work(kondemand_wq, &tickle_state.floor_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_floor_millis);

void cpufreq_ondemand_floor(unsigned int freq)
{
	cpufreq_ondemand_floor_millis(freq, dbs_tuners_ins.max_floor_window);
}
EXPORT_SYMBOL(cpufreq_ondemand_floor);

void cpufreq_ondemand_floor_hold(unsigned int freq)
{
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);
	tickle_state.floor_count += 1;
	
	default_file_data.floor_freq = freq;

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	queue_work(kondemand_wq, &tickle_state.floor_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_floor_hold);

void cpufreq_ondemand_floor_unhold(void)
{
	int queue = 0;
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);

	if (tickle_state.floor_count) {
		tickle_state.floor_count -= 1;
		queue = 1;
	} else {
		printk(KERN_WARNING "%s: attempt to decrement when floor_count == 0!\n",
				__FUNCTION__);
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	if (queue)
		queue_work(kondemand_wq, &tickle_state.floor_work);
}
EXPORT_SYMBOL(cpufreq_ondemand_floor_unhold);

void cpufreq_ondemand_floor_hold_check(unsigned int freq, int *flag)
{
	if (!*flag) {
		cpufreq_ondemand_floor_hold(freq);
		*flag = 1;
	}
}
EXPORT_SYMBOL(cpufreq_ondemand_floor_hold_check);

void cpufreq_ondemand_floor_unhold_check(int *flag)
{
	if (*flag) {
		cpufreq_ondemand_floor_unhold();
		*flag = 0;
	}
}
EXPORT_SYMBOL(cpufreq_ondemand_floor_unhold_check);


/** 
* @brief 
*
* Should be called with the lock_policy_rwsem_write already held.
*
* @param  this_dbs_info 
*/
static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int max_load_freq;

	struct cpufreq_policy *policy;
	unsigned int j;

	this_dbs_info->cur_load = 0;
   
	if (!this_dbs_info->enable)
		return;

	this_dbs_info->freq_lo = 0;
	policy = this_dbs_info->cur_policy;

	/*
	 * Every sampling_rate, we check, if current idle time is less
	 * than 20% (default), then we try to increase frequency
	 * Every sampling_rate, we look for a the lowest
	 * frequency which can sustain the load while keeping idle time over
	 * 30%. If such a frequency exist, we try to decrease to this frequency.
	 *
	 * Any frequency increase takes it to the maximum frequency.
	 * Frequency reduction happens at minimum steps of
	 * 5% (default) of current frequency
	 */

	/* Get Absolute Load - in terms of freq */
	max_load_freq = 0;

	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;
		unsigned int load, load_freq;
		int freq_avg;

		j_dbs_info = &per_cpu(cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);

		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
				j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
				j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		if (dbs_tuners_ins.ignore_nice) {
			cputime64_t cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = cputime64_sub(kstat_cpu(j).cpustat.nice,
					 j_dbs_info->prev_cpu_nice);
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kstat_cpu(j).cpustat.nice;
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;
		this_dbs_info->cur_load = load;

		freq_avg = __cpufreq_driver_getavg(policy, j);
		if (freq_avg <= 0) {
			freq_avg = policy->cur;
		}

		load_freq = load * freq_avg;
		if (load_freq > max_load_freq) {
			max_load_freq = load_freq;
		}
	}
	this_dbs_info->max_load_freq = max_load_freq;
}

/** 
* @brief 
*
* Should be called with the lock_policy_rwsem_write already held.
*
* @param  this_dbs_info 
*/
static void adjust_for_load(struct cpu_dbs_info_s *this_dbs_info) 
{
	unsigned int 		max_load_freq;
	unsigned int		load;
	struct cpufreq_policy*	policy;

	if ((!this_dbs_info->enable) || (this_dbs_info->tickle_active)) {
		return;
	}

	max_load_freq 	= this_dbs_info->max_load_freq;
	load		= this_dbs_info->cur_load;
	policy 		= this_dbs_info->cur_policy;
	/* Check for frequency increase */
	if (max_load_freq > dbs_tuners_ins.up_threshold * policy->cur) {
		/* if we are already at full speed then break out early */
		if (!dbs_tuners_ins.powersave_bias) {
			record_sample(policy->cur, policy->max, load, policy->cpu);
			if (policy->cur == policy->max)
				return;

			if (this_dbs_info->floor_active) {
				this_dbs_info->freq_save = policy->max;
				this_dbs_info->rel_save = CPUFREQ_RELATION_H;
			}

			__cpufreq_driver_target(policy, policy->max,
				CPUFREQ_RELATION_H);
		} else {
			int freq = powersave_bias_target(policy, policy->max,
					CPUFREQ_RELATION_H);

			if (this_dbs_info->floor_active) {
				this_dbs_info->freq_save = freq;
				this_dbs_info->rel_save = CPUFREQ_RELATION_L;

				if (freq <= this_dbs_info->freq_floor) {
					freq = this_dbs_info->freq_floor;
				}
			}
			record_sample(policy->cur, freq, load, policy->cpu);

			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
		return;
	}

	/* Check for frequency decrease */
	/* if we cannot reduce the frequency anymore, break out early */
	if (policy->cur == policy->min)
		return;

	/*
	 * The optimal frequency is the frequency that is the lowest that
	 * can support the current CPU usage without triggering the up
	 * policy. To be safe, we focus 10 points under the threshold.
	 */
	if (max_load_freq <
	    (dbs_tuners_ins.up_threshold - dbs_tuners_ins.down_differential) *
	     policy->cur) {
		unsigned int freq_next;
		freq_next = max_load_freq /
				(dbs_tuners_ins.up_threshold -
				 dbs_tuners_ins.down_differential);

		if (!dbs_tuners_ins.powersave_bias) {
			if (this_dbs_info->floor_active) {
				this_dbs_info->freq_save = freq_next;
				this_dbs_info->rel_save = CPUFREQ_RELATION_L;

				if (freq_next <= this_dbs_info->freq_floor) {
					freq_next = this_dbs_info->freq_floor;
				}
			}
			record_sample(policy->cur, freq_next, load, policy->cpu);
			__cpufreq_driver_target(policy, freq_next,
					CPUFREQ_RELATION_L);
		} else {
			int freq = powersave_bias_target(policy, freq_next,
					CPUFREQ_RELATION_L);
			if (this_dbs_info->floor_active) {
				this_dbs_info->freq_save = freq;
				this_dbs_info->rel_save = CPUFREQ_RELATION_L;

				if (freq <= this_dbs_info->freq_floor) {
					freq = this_dbs_info->freq_floor;
				}
			}
			record_sample(policy->cur, freq, load, policy->cpu);
			__cpufreq_driver_target(policy, freq,
				CPUFREQ_RELATION_L);
		}
	}
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;
	int sample_type = dbs_info->sample_type;

	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	delay -= jiffies % delay;

	if (lock_policy_rwsem_write(cpu) < 0)
		return;

	if (!dbs_info->enable) {
		unlock_policy_rwsem_write(cpu);
		return;
	}

	/* Common NORMAL_SAMPLE setup */
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	if (!dbs_tuners_ins.powersave_bias ||
	    sample_type == DBS_NORMAL_SAMPLE) {
		dbs_check_cpu(dbs_info);
		adjust_for_load(dbs_info);

		if (dbs_info->freq_lo) {
			/* Setup timer for SUB_SAMPLE */
			dbs_info->sample_type = DBS_SUB_SAMPLE;
			delay = dbs_info->freq_hi_jiffies;
		}
	} else {
		record_sample(dbs_info->cur_policy->cur, dbs_info->freq_lo, -1, cpu);
		__cpufreq_driver_target(dbs_info->cur_policy,
	                        	dbs_info->freq_lo,
	                        	CPUFREQ_RELATION_H);
	}
	queue_delayed_work_on(cpu, kondemand_wq, &dbs_info->work, delay);
	unlock_policy_rwsem_write(cpu);
}

static void do_tickle_timer(unsigned long arg)
{
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);
	
	if (tickle_state.active_count) {
		if (tickle_state.active_tickle) {
			tickle_state.active_count -= 1;
			tickle_state.active_tickle = 0;
		}
	} else {
		printk(KERN_WARNING "%s: attempt to decrement when active_count == 0!\n",
				__FUNCTION__);
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	queue_work(kondemand_wq, &tickle_state.tickle_work);
}

static void do_floor_timer(unsigned long arg)
{
	unsigned long flags;

	spin_lock_irqsave(&tickle_state.lock, flags);

	if (tickle_state.floor_count) {
		if (tickle_state.floor_tickle) {
			tickle_state.floor_count -= 1;
			tickle_state.floor_tickle = 0;
		}
	} else {
		printk(KERN_WARNING "%s: attempt to decrement when floor_count == 0!\n",
				__FUNCTION__);
	}

	spin_unlock_irqrestore(&tickle_state.lock, flags);

	queue_work(kondemand_wq, &tickle_state.floor_work);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	ondemand_powersave_bias_init();
	dbs_info->sample_type = DBS_NORMAL_SAMPLE;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, kondemand_wq, &dbs_info->work,
	                      delay);

	dbs_info->tickle_active = 0;
	dbs_info->floor_active = 0;
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work(&dbs_info->work);
}

static void dbs_refresh_callback(struct work_struct *unused)
{
	struct cpufreq_policy *policy;
	struct cpu_dbs_info_s *this_dbs_info;

	if (lock_policy_rwsem_write(0) < 0)
		return;

	this_dbs_info = &per_cpu(cpu_dbs_info, 0);
	policy = this_dbs_info->cur_policy;

	if (policy->cur < policy->max) {
		policy->cur = policy->max;

		__cpufreq_driver_target(policy, policy->max,
					CPUFREQ_RELATION_L);
		this_dbs_info->prev_cpu_idle = get_cpu_idle_time(0,
				&this_dbs_info->prev_cpu_wall);
	}
	unlock_policy_rwsem_write(0);
}

static DECLARE_WORK(dbs_refresh_work, dbs_refresh_callback);

static void dbs_input_event(struct input_handle *handle, unsigned int type,
		unsigned int code, int value)
{
	schedule_work(&dbs_refresh_work);
}

static int dbs_input_connect(struct input_handler *handler,
		struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "cpufreq";

	error = input_register_handle(handle);
	if (error)
		goto err2;

	error = input_open_device(handle);
	if (error)
		goto err1;

	return 0;
err1:
	input_unregister_handle(handle);
err2:
	kfree(handle);
	return error;
}

static void dbs_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id dbs_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler dbs_input_handler = {
	.event		= dbs_input_event,
	.connect	= dbs_input_connect,
	.disconnect	= dbs_input_disconnect,
	.name		= "cpufreq_ond_tcl",
	.id_table	= dbs_ids,
};

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;
	struct proc_dir_entry *entry;

	this_dbs_info = &per_cpu(cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		if (this_dbs_info->enable) /* Already enabled */
			break;

		mutex_lock(&dbs_mutex);
		dbs_enable++;

		rc = sysfs_create_group(&policy->kobj, &dbs_attr_group);
		if (rc) {
			dbs_enable--;
			mutex_unlock(&dbs_mutex);
			return rc;
		}

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice) {
				j_dbs_info->prev_cpu_nice =
						kstat_cpu(j).cpustat.nice;
			}
		}
		this_dbs_info->cpu = cpu;
		
		/* sampling must be enabled prior to becoming the active governor */
		if (sampling_enabled) {
			for_each_cpu_mask(j, policy->cpus) {
				struct sample_stats *stats = &per_cpu(sample_stats, j);
			
				// if allocation fails, sampling is disabled on this cpu
				if (!stats->samples)
					stats->samples = vmalloc(sizeof(struct sample_data) * NUM_SAMPLES);
				
				if (!stats->samples)
					continue;

				// touch each page to force allocation of physical pages
				memset(stats->samples, 0, sizeof(struct sample_data) * NUM_SAMPLES);	
			}

			entry = create_proc_entry("ondemandtcl_samples", 0444, NULL);
			if (entry)
				entry->proc_fops = &proc_stats_operations;
		}

		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			unsigned int latency;
			/* policy latency is in nS. Convert it to uS first */
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;

			def_sampling_rate = latency *
				CONFIG_CPU_FREQ_SAMPLING_LATENCY_MULTIPLIER;

			if (def_sampling_rate < MIN_STAT_SAMPLING_RATE)
				def_sampling_rate = MIN_STAT_SAMPLING_RATE;

			dbs_tuners_ins.sampling_rate = def_sampling_rate;
		}
		dbs_timer_init(this_dbs_info);
		rc = input_register_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);
		break;

	case CPUFREQ_GOV_STOP:
		mutex_lock(&dbs_mutex);
		dbs_timer_exit(this_dbs_info);
		sysfs_remove_group(&policy->kobj, &dbs_attr_group);
		dbs_enable--;

		for_each_cpu_mask(j, policy->cpus) {
			struct sample_stats *stats = &per_cpu(sample_stats, j);
			
			if (stats->samples)
				vfree(stats->samples);

			stats->samples = NULL;
		}

		remove_proc_entry("ondemand_samples", NULL);

		input_unregister_handler(&dbs_input_handler);
		mutex_unlock(&dbs_mutex);
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&dbs_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur) {
			record_sample(policy->cur, policy->max, -1, policy->cpu);
			__cpufreq_driver_target(this_dbs_info->cur_policy,
			                        policy->max,
			                        CPUFREQ_RELATION_H);
		} else if (policy->min > this_dbs_info->cur_policy->cur) {
			record_sample(policy->cur, policy->min, -1, policy->cpu);
			__cpufreq_driver_target(this_dbs_info->cur_policy,
			                        policy->min,
			                        CPUFREQ_RELATION_L);
		}
		mutex_unlock(&dbs_mutex);
		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND_TICKLE
static
#endif
struct cpufreq_governor cpufreq_gov_ondemand_tickle = {
	.name			= "ondemandtcl",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency = TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	int err;
	cputime64_t wall;
	u64 idle_time;
	int cpu = get_cpu();

	idle_time = get_cpu_idle_time_us(cpu, &wall);
	put_cpu();
	if (idle_time != -1ULL) {
		/* Idle micro accounting is supported. Use finer thresholds */
		dbs_tuners_ins.up_threshold = MICRO_FREQUENCY_UP_THRESHOLD;
		dbs_tuners_ins.down_differential =
					MICRO_FREQUENCY_DOWN_DIFFERENTIAL;
	}

	kondemand_wq = create_workqueue("kondemandtcl");
	if (!kondemand_wq) {
		printk(KERN_ERR "Creation of kondemandtcl failed\n");
		return -EFAULT;
	}

	init_timer(&tickle_state.tickle_timer);
	init_timer(&tickle_state.floor_timer);

	tickle_state.tickle_timer.function = do_tickle_timer;
	tickle_state.floor_timer.function = do_floor_timer;

	INIT_WORK(&tickle_state.tickle_work, do_tickle_state_change);
	INIT_WORK(&tickle_state.floor_work, do_floor_state_change);

	/* init these to current jiffies or short circuiting doesn't work until jiffies wraps */
	tickle_state.tickle_jiffies = tickle_state.floor_jiffies = jiffies;

	err = setup_tickle_device();
	if (err < 0) {
		return err;
	}

	err = cpufreq_register_governor(&cpufreq_gov_ondemand_tickle);
	if (err)
		destroy_workqueue(kondemand_wq);

	return err;
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	remove_tickle_device();
	cpufreq_unregister_governor(&cpufreq_gov_ondemand_tickle);

	/* cancel any pending timers before destroying the work queue */
	del_timer(&tickle_state.tickle_timer);
	del_timer(&tickle_state.floor_timer);

	destroy_workqueue(kondemand_wq);
}


MODULE_AUTHOR("Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>");
MODULE_AUTHOR("Alexey Starikovskiy <alexey.y.starikovskiy@intel.com>");
MODULE_AUTHOR("Corey Tabaka <corey.tabaka@palm.com>");
MODULE_DESCRIPTION("'cpufreq_ondemand_tickle' - A dynamic cpufreq governor for "
                   "Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ONDEMAND_TICKLE
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
