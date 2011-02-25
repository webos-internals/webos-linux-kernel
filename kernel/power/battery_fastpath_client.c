/*
 * battery_fastpath_client.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/rtc.h>
#include <linux/battery_simple.h>

#include "power.h"
#include <linux/gen_timer.h>
#include <linux/fastpath_client.h>

/**
 * Debug
 */

/* FIXME: Should move away from DEBUG as it clobbers existing DEBUG macro */
#undef DEBUG

#define DEBUG(...)                           \
do {                                                         \
        if (batterycheck_debug_level)                \
                printk(__VA_ARGS__);                 \
} while (0)

#define INFO(...)                            \
do {                                         \
        printk(KERN_INFO __VA_ARGS__);           \
} while (0)

#define ERROR(...)                           \
do {                                         \
        printk(KERN_ERR __VA_ARGS__);            \
} while (0)



/**
 * Module parameters
 */

static uint32_t fastpath_enabled = 1;
module_param_named(enabled, fastpath_enabled, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "enable/disable fastpath.");

/** 
* @brief Turn on debug messages.
*/
static uint32_t batterycheck_debug_level = 0;
module_param_named(debug_level, batterycheck_debug_level, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "battery check debug level.");

struct battery {
	int enabled;
	int percent;
	int voltage_uV;
	int temperature_C;
	unsigned long last_read_sec;
};

struct battery battery;
struct battery fake_battery;

/** 
* @brief Turn fake battery on.
*/
module_param_named(fake_battery, fake_battery.enabled, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(fake_battery, "Use fake_battery parameters.");

/** 
* @brief Fake battery parameters.
*/
module_param_named(fake_percent, fake_battery.percent, int, S_IRUGO | S_IWUSR);
module_param_named(fake_voltage_uV, fake_battery.voltage_uV, int, S_IRUGO | S_IWUSR);
module_param_named(fake_temperature_C, fake_battery.temperature_C, int, S_IRUGO | S_IWUSR);

/** 
* @brief Battery reporting thresholds
*/
static int battery_threshold_array[] = {
	100,
	20,
	10,
	5,
};
static int battery_threshold_size = ARRAY_SIZE(battery_threshold_array);

module_param_array_named(battery_thresholds, battery_threshold_array,
		 int, &battery_threshold_size, S_IRUGO | S_IWUSR);

/** 
* @brief Last reported threshold (index into battery_threshold_array)
*/
static int last_battery_threshold;
module_param_named(last_battery_threshold, last_battery_threshold, int, S_IRUGO | S_IWUSR);

/** 
* @brief If from batterycheck, the reason why we woke up.
*/
enum {
	BATTERYCHECK_NONE = 0,
	BATTERYCHECK_THRESHOLD_REACHED,
	BATTERYCHECK_CRITICAL_LOW_BATTERY,
	BATTERYCHECK_CRITICAL_TEMPERATURE,
	BATTERYCHECK_END,
};
static int batterycheck_wakeup = BATTERYCHECK_NONE;

static const char *batterycheck_wakeup_string[BATTERYCHECK_END] = {
	"none",
	"threshold",
	"criticalbatt",
	"criticaltemp",
};

/**
 * Battery parameters... this should eventually be factored out into the arch
 * specific portion.
 */
#define BATTERY_MAX_TEMPERATURE_C  60
#define BATTERY_HIGH_TEMPERATURE_C 50     /* TODO check this experimentally */
#define BATTERY_MIN_VOLTAGE_UV    3400000

#define BATTERY_PERCENT_SAMPLES 5

#define BATTERY_VOLTAGE_SAMPLES 5
#define BATTERY_VOLTAGE_MAJORITY 3

#define SECS_PER_MIN 60
#define POLL_FREQ_SEC_3800          (10*SECS_PER_MIN)
#define POLL_FREQ_SEC_3600_3800     (5*SECS_PER_MIN)
#define POLL_FREQ_SEC_3500_3600     (1*SECS_PER_MIN)
#define POLL_FREQ_SEC_3400_3500     (10)
#define POLL_FREQ_SEC_LT_3400       (5)

#define POLL_FREQ_HIGH_TEMP         (5) 
#define POLL_FREQ_DEFAULT           (5)

struct battery_poll_freq {
	int voltage_uV;
	int poll_interval_secs;
};

/**
 * How frequently we poll for batttery updates when asleep.
 * TODO: make this a module parameter.
 */
static struct battery_poll_freq battery_poll_freq[] = {
  { 3800000, POLL_FREQ_SEC_3800      },
  { 3600000, POLL_FREQ_SEC_3600_3800 },
  { 3500000, POLL_FREQ_SEC_3500_3600 },
  { 3400000, POLL_FREQ_SEC_3400_3500 },
  { 0,		 POLL_FREQ_SEC_LT_3400   },
};

static int percent_sample[BATTERY_PERCENT_SAMPLES];
static int percent_sample_index = 0;

static int voltage_sample_uV[BATTERY_VOLTAGE_SAMPLES];
static int voltage_sample_index = 0;

extern gen_timer_stats_t gtimerstats;

/**
 * Methods.
 */

static void percent_samples_record(int percent)
{
	percent_sample_index =
		( percent_sample_index + 1 ) % BATTERY_PERCENT_SAMPLES;

	percent_sample[percent_sample_index] = percent;
}

/** 
* @brief Returns the last percent sample.
* 
* @retval
*/
static int percent_get_last(void)
{
	return percent_sample[percent_sample_index];
}

static int battery_threshold_changed(void)
{
	int i;
	int percent;
	int threshold = 0;

	percent = percent_get_last();

	for (i = 0; i < ARRAY_SIZE(battery_threshold_array); i++) {
		if (percent <= battery_threshold_array[i]) {
			threshold = i;
		}
	}

	DEBUG("%s: Percent %d%%, threshold %d\n",
			__FUNCTION__, percent, threshold);
	
	if (threshold != last_battery_threshold) {
		last_battery_threshold = threshold;
		return 1;
	}
	return 0;
}

static void voltage_samples_record(int voltage)
{
	voltage_sample_index =
		( voltage_sample_index + 1 ) % BATTERY_VOLTAGE_SAMPLES;

	voltage_sample_uV[voltage_sample_index] = voltage;
}

static int voltage_samples_less_than(int min_voltage_uV)
{
	int i;
	int count = 0;
	int voltage;

	for (i = 0; i < BATTERY_VOLTAGE_SAMPLES; i++) {
		voltage = voltage_sample_uV[i];
		if (voltage != 0 && voltage <= min_voltage_uV)
			count++;
	}

	return count >= BATTERY_VOLTAGE_MAJORITY;
}

/** 
* @brief Returns the last voltage sample.
* 
* @retval
*/
static int voltage_get_last(void)
{
	return voltage_sample_uV[voltage_sample_index];
}

/** 
* @brief Returns the next wakeup time interval.
* 
* @retval
*/
static int battery_poll_next_seconds(void)
{
	int i;
	int poll_interval = POLL_FREQ_DEFAULT;
	int last_voltage;
	struct battery_poll_freq *poll_freq;
	
	last_voltage = voltage_get_last();

	for (i = 0; i < ARRAY_SIZE(battery_poll_freq); i++) {

		poll_freq = &battery_poll_freq[i];

		if (last_voltage >= poll_freq->voltage_uV) {
			poll_interval = poll_freq->poll_interval_secs;
			break;
		}
	}

	if (battery.temperature_C >= BATTERY_HIGH_TEMPERATURE_C)
	{
		poll_interval = min(poll_interval, POLL_FREQ_HIGH_TEMP);
	}

	return poll_interval;
}

static int battery_read(unsigned long now)
{
	int retval;
	int percent;
	int voltage_uV;
	int temperature_C;

	if (fake_battery.enabled) {
		/* Debugging overrides */
		percent = fake_battery.percent;
		voltage_uV = fake_battery.voltage_uV;
		temperature_C = fake_battery.temperature_C;
	}
	else {
		retval = battery_get_percent(&percent);
		if (retval < 0) {
			ERROR("%s: Could not get battery percent.\n", __FUNCTION__);
			goto error;
		}
		retval = battery_get_voltage(&voltage_uV);
		if (retval < 0) {
			ERROR("%s: Could not get battery voltage.\n", __FUNCTION__);
			goto error;
		}
		retval = battery_get_temperature(&temperature_C);
		if (retval < 0) {
			ERROR("%s: Could not get battery temperature.\n", __FUNCTION__);
			goto error;
		}
	}

	percent_samples_record(percent);
	voltage_samples_record(voltage_uV);

	battery.percent = percent;
	battery.voltage_uV = voltage_uV;
	battery.temperature_C = temperature_C;

	if (now) {
		battery.last_read_sec = now;
	}

	DEBUG("%s: Battery P: %d%%, V: %duV, T: %dC\n",
		__FUNCTION__, percent, voltage_uV, temperature_C);
	return 0;
error:
	return -1;
}

/** 
* @brief Returns true if battery level is critical, and false otherwise.
* 
* @retval
*/
static int battery_is_critical(void)
{
	return voltage_samples_less_than(BATTERY_MIN_VOLTAGE_UV);
}

static int battery_is_too_hot(void)
{
	return (battery.temperature_C >= BATTERY_MAX_TEMPERATURE_C);
}

static unsigned long rtc_now(struct rtc_device *rtc)
{
	int retval;
	unsigned long now;
	struct rtc_wkalrm alm;

	retval = rtc_read_time(rtc, &alm.time);
	if (retval < 0) {
		ERROR("%s: unable to read the rtc (hardware) clock\n", __FUNCTION__);
		goto cleanup;
	}

	DEBUG("%s: Now %02d:%02d:%02d\n", __FUNCTION__,
			alm.time.tm_hour, alm.time.tm_min, alm.time.tm_sec);

	rtc_tm_to_time(&alm.time, &now);

	return now;
cleanup:
	return 0;
}



static const char *batterycheck_wakeup_to_string(void)
{
	return batterycheck_wakeup_string[batterycheck_wakeup];
}

/**
 * Init
 */
int batterycheck_prepare_handler(void);
int batterycheck_fastwake_handler(void);

static fastpath_client_t batterycheck_client = {
	.name = "Battery Check Fastpath Client",
	.prepare = batterycheck_prepare_handler,
	.fastwake = batterycheck_fastwake_handler,
	.next_alarm = 0,
};

int batterycheck_prepare_handler(void) {
	unsigned long now=0,next_alarm;
	struct rtc_device *rtc;

	DEBUG("======%s\n", __FUNCTION__);
		

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (NULL == rtc) {
		ERROR("%s: unable to open rtc device (%s)\n",
				__FUNCTION__, CONFIG_RTC_HCTOSYS_DEVICE);
	}
	else {
		now = rtc_now(rtc);
	}

        /* Read battery if last reading was a while ago... */
        if (now > battery.last_read_sec + 10 * SECS_PER_MIN) {
                int retval;
                retval = battery_read(now);
                if (retval < 0) {
                        DEBUG("%s: Error reading battery.\n", __FUNCTION__);
                }
        }
	
	if(batterycheck_client.next_alarm > now) {
		DEBUG("%s: Pending alarm for battery check\n",__FUNCTION__);
		next_alarm = batterycheck_client.next_alarm - now;
	}
	else
		next_alarm=battery_poll_next_seconds();

	if (rtc) {
		rtc_class_close(rtc);
	}
	DEBUG("======%s  Returning alarm timeout : %lu\n", __FUNCTION__,next_alarm);
	return next_alarm;
}

int batterycheck_fastwake_handler(void) {
	int retval;
	unsigned long now=0;
	int fastsleep_required=0;

	struct rtc_device *rtc;
	DEBUG("======%s\n", __FUNCTION__);
	
	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (NULL == rtc) {
		ERROR("%s: unable to open rtc device (%s)\n",
				__FUNCTION__, CONFIG_RTC_HCTOSYS_DEVICE);
	}
	else {
		now = rtc_now(rtc);
	}
	
	batterycheck_wakeup = BATTERYCHECK_NONE;
	
	retval = battery_read(now);
	if (retval < 0) {
		DEBUG("%s: Error reading battery.\n", __FUNCTION__);
	}
	else if (battery_is_critical()) {
		batterycheck_wakeup = BATTERYCHECK_CRITICAL_LOW_BATTERY;
	}
	else if (battery_is_too_hot()) {
		batterycheck_wakeup = BATTERYCHECK_CRITICAL_TEMPERATURE;
	}
	else if (battery_threshold_changed()) {
		batterycheck_wakeup = BATTERYCHECK_THRESHOLD_REACHED;
	}
	if (batterycheck_wakeup != BATTERYCHECK_NONE) {
		DEBUG("%s: Battery %s... waking up.\n",
				__FUNCTION__, batterycheck_wakeup_to_string());
	}
	else {
		DEBUG("%s: Fast sleep!\n", __FUNCTION__);
		fastsleep_required = 1;
	}

	gtimerstats.battery_timeouts++;
	if (rtc) {
		rtc_class_close(rtc);
	}
	return fastsleep_required;	
}

static ssize_t batterycheck_wakeup_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%s\n", batterycheck_wakeup_to_string());
}

static int __init batterycheck_init(void)
{
	batterycheck_wakeup = BATTERYCHECK_NONE;

	memset(percent_sample, 0, sizeof(percent_sample));
	memset(voltage_sample_uV, 0, sizeof(voltage_sample_uV));

	register_fastpath_client(&batterycheck_client);
	
	return 0;
}



static struct subsys_attribute batterycheck_wakeup_sysfs = {
	.attr = {
		.name = __stringify(batterycheck_wakeup),
		.mode = 0644,
	},
	.show  = batterycheck_wakeup_show,
	.store = NULL,
};

static int __init batterycheck_sysfs_init(void)
{
        int retval;

        retval = subsys_create_file(&power_subsys, &batterycheck_wakeup_sysfs);
        if (retval)
                goto error;

error:
        return retval;
}


static int __init batterycheck_module_init(void)
{
	int rc;

	rc = batterycheck_init();
	if (rc) {
		ERROR("%s could not init batterycheck: %d\n",
				__FUNCTION__, rc);
		return -1;
	}

	rc = batterycheck_sysfs_init();
	if (rc) {
		ERROR("%s could not init fastpath_sysfs: %d\n",
				__FUNCTION__, rc);
		return -1;
	}

	return 0;
}

static void __exit batterycheck_module_exit(void)
{
	unregister_fastpath_client(&batterycheck_client);
}

module_init(batterycheck_module_init);
module_exit(batterycheck_module_exit);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("Provides the ability to periodically wake device to check for critical battery.");
MODULE_LICENSE("GPL");
