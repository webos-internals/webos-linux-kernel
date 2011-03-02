/*
 * linux/arch/arm/mach-omap3pe/fastpath.c
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

#define	EPOCH	1900	/* RTC epoch */

/*
 * Debug
 */

/* FIXME: Should move away from DEBUG as it clobbers existing DEBUG macro */
#undef DEBUG

#define DEBUG(...)                           \
do {                                                         \
        if (fastpath_debug_level)                \
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
static int32_t fastpath_debug_level = 0;
module_param_named(debug_level, fastpath_debug_level, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "fastpath debug level.");

static unsigned long alarm_user;

/** 
* @brief State nodes.
*/
enum {
	FASTPATH_STATE_WAKE = 0,
	FASTPATH_STATE_RTC_SYSTEM,
	FASTPATH_STATE_RTC_USER,
};
static int fastpath_state;



extern gen_timer_stats_t gtimerstats;
static struct list_head fastpath_client_list_head;
static unsigned long sleep_offset_old,time_old;
unsigned long sleep_offset = 0,current_sleep_length = 0;
 
/**
 * Methods.
 */

static int __init fastpath_init(void)
{
	fastpath_state = FASTPATH_STATE_WAKE;
	return 0;
}
	
/** 
* @brief Return the current rtc alarm
* 
* @retval
*/
static unsigned long rtc_alarm(struct rtc_device *rtc)
{
	int retval;
	unsigned long alarm_secs;
	struct rtc_wkalrm alm;

	retval = rtc_read_alarm(rtc, &alm);
	if (retval < 0) {
		ERROR("%s: unable to read the rtc alarm\n", __FUNCTION__);
		goto error;
	}

	if (alm.enabled) {
		INFO("%s: Read current rtc alarm of %04d/%02d/%02d %02d:%02d:%02d\n",
		     __FUNCTION__,
		     EPOCH + alm.time.tm_year, alm.time.tm_mon, alm.time.tm_mday,
		     alm.time.tm_hour, alm.time.tm_min, alm.time.tm_sec);

		rtc_tm_to_time(&alm.time, &alarm_secs);
	}
	else {
		alarm_secs = 0;
	}
	return alarm_secs;

error:
	return 0;
}

static int rtc_alarm_write(struct rtc_device *rtc, unsigned long next_alarm)
{
	int retval;
	struct rtc_wkalrm alm;

	alm.enabled = 1;
	rtc_time_to_tm(next_alarm, &alm.time);

	retval = rtc_set_alarm(rtc, &alm);
	if (retval < 0) {
		ERROR("%s: Error %d could not set the rtc alarm\n",
				__FUNCTION__, retval);
		goto error;
	}

	DEBUG("%s: Setting rtc alarm to %04d/%02d/%02d %02d:%02d:%02d\n",
		__FUNCTION__,
	      EPOCH + alm.time.tm_year, alm.time.tm_mon, alm.time.tm_mday,
	      alm.time.tm_hour, alm.time.tm_min, alm.time.tm_sec);

error:
	return retval;
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

	DEBUG("%s: Now %04d/%02d/%02d %02d:%02d:%02d\n", __FUNCTION__,
	      EPOCH + alm.time.tm_year, alm.time.tm_mon, alm.time.tm_mday,
	      alm.time.tm_hour, alm.time.tm_min, alm.time.tm_sec);

	rtc_tm_to_time(&alm.time, &now);

	return now;
cleanup:
	return 0;
}



/** 
* @brief Set up RTC to wake us up periodically so that we can
*        check the battery.
* 
* @retval
*/
int fastpath_prepare(void)
{
	unsigned long now, next_alarm=0;
	struct rtc_device *rtc;
	struct list_head *ptr;
	struct fastpath_client *client, *next_alarm_client = NULL;

	DEBUG("================================%s\n", __FUNCTION__);

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (NULL == rtc) {
		ERROR("%s: unable to open rtc device (%s)\n",
			__FUNCTION__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -ENODEV;
	}

	if (!fastpath_enabled) {

		if (fastpath_debug_level) {
			rtc_now(rtc);
			rtc_alarm(rtc);
		}
		rtc_class_close(rtc);
		return 0;
	}

	now=rtc_now(rtc);

	DEBUG("Executing list_for_each\n");
	list_for_each(ptr,&fastpath_client_list_head){
		client=list_entry(ptr,struct fastpath_client,entry);
		client->next_alarm = now + client->prepare();
		if((client->next_alarm > now) && (!next_alarm || next_alarm > client->next_alarm)) {
			next_alarm = client->next_alarm;
			next_alarm_client = client;
		}
		DEBUG("Next alarm : %lu\n",client->next_alarm);
	}
	

	sleep_offset_old=sleep_offset;
	time_old=now;
	DEBUG("Min Next alarm : %lu\n",next_alarm);

	/* On first entry, save the user rtc alarm */
	if (FASTPATH_STATE_WAKE == fastpath_state) {
		alarm_user = rtc_alarm(rtc);
	}

	if (fastpath_debug_level || FASTPATH_STATE_WAKE == fastpath_state) {
		struct rtc_time alm_tm;
		struct rtc_time now_tm;
		struct rtc_time next_alrm_tm;

		rtc_time_to_tm(now, &now_tm);
		rtc_time_to_tm(alarm_user, &alm_tm);
		rtc_time_to_tm(next_alarm, &next_alrm_tm);
		//rtc_time_to_tm(battery.last_read_sec, &last_batt_tm);

		INFO("%s: RTC = %04d/%02d/%02d %02d:%02d:%02d, "
				"UserAlarm = %04d/%02d/%02d %02d:%02d:%02d, "
				"NextFastWake = %04d/%02d/%02d %02d:%02d:%02d\n",
			__FUNCTION__,
		     EPOCH + now_tm.tm_year, now_tm.tm_mon, now_tm.tm_mday,
		     now_tm.tm_hour, now_tm.tm_min, now_tm.tm_sec,
		     EPOCH + alm_tm.tm_year, alm_tm.tm_mon, alm_tm.tm_mday,
		     alm_tm.tm_hour, alm_tm.tm_min, alm_tm.tm_sec,
		     EPOCH + next_alrm_tm.tm_year, next_alrm_tm.tm_mon, next_alrm_tm.tm_mday,
		     next_alrm_tm.tm_hour, next_alrm_tm.tm_min, next_alrm_tm.tm_sec);
		if(next_alarm_client)
			INFO("%s: Alarm set after %lu seconds by fastpath client : %s\n",
				__FUNCTION__,next_alarm-now,next_alarm_client->name);
			
	}

	/* Pick whether next alarm should be user rtc alarm or system rtc alarm.
	 *
	 * alarm_user >= now - 2 is to catch those cases where the alarm might
	 * fire on the way down in the suspend path.  The suspend logic should
	 * treat these cases as fall-throughs, but it is not currently.
	 */ 
	if (alarm_user && alarm_user >= now - 2 &&
		alarm_user <= next_alarm) {

		next_alarm = alarm_user;
		fastpath_state = FASTPATH_STATE_RTC_USER;
	}
	else {
		fastpath_state = FASTPATH_STATE_RTC_SYSTEM;
	}

	/* Set the new alarm. */
	rtc_alarm_write(rtc, max(next_alarm, now + 2));

	rtc_class_close(rtc);
	return 0;
}

/** 
* @brief Called to check if we should go back to sleep.
* 
* @retval 1 if we should go back to sleep.
*/
int fastpath_fastsleep(int wakeup_is_rtc)
{
	int fastsleep_required = 0;
	struct rtc_device *rtc;
	unsigned long now = 0;
	struct list_head *ptr;
	struct fastpath_client *client;
	int ret=0;
	
	DEBUG("%s=======wakeup_is_rtc : %d=========== \n", __FUNCTION__,wakeup_is_rtc);


	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (NULL == rtc) {
		ERROR("%s: unable to open rtc device (%s)\n",
				__FUNCTION__, CONFIG_RTC_HCTOSYS_DEVICE);
	}
	else {
		now = rtc_now(rtc);
	}


	if (!fastpath_enabled) {

		if (fastpath_debug_level) {
			rtc_now(rtc);
			rtc_alarm(rtc);
		}
		goto end;
	}

	current_sleep_length = now - time_old;
	sleep_offset = sleep_offset_old + current_sleep_length;
	
	/* Skip battery check and don't re-enter sleep
	 * if not real RTC wakeup. */
	if (wakeup_is_rtc) {
		unsigned long now;
		unsigned long alarm;
		int diff;

		now = rtc_now(rtc);
		alarm = rtc_alarm(rtc);

		diff = alarm - now;
		if (diff > 0) {
			DEBUG("%s: not real RTC wakeup.\n", __FUNCTION__);
			wakeup_is_rtc = 0;
		}
	}
	
	DEBUG("state : %d wakeup_is_rtc : %d sleep offset : %lu rtc jiffies : %lu \n",
						fastpath_state,wakeup_is_rtc,sleep_offset,
						jiffies+(sleep_offset*HZ));

	if(wakeup_is_rtc) {
		if (FASTPATH_STATE_RTC_SYSTEM == fastpath_state && wakeup_is_rtc) {
			fastsleep_required=1;
			list_for_each(ptr,&fastpath_client_list_head) {
				client=list_entry(ptr,struct fastpath_client,entry);
				if((now - client->next_alarm) <=2 ) {
					INFO("%s: RTC Wakeup for Fastpath client : %s\n",__FUNCTION__,client->name);
					ret = client->fastwake();	
					if(!ret)
						fastsleep_required=0;
					client->next_alarm=0;
				}
			}
			if(fastsleep_required)
				goto end;

		}
		else if(fastpath_state == FASTPATH_STATE_RTC_USER) {
			INFO("%s: RTC Wakeup for User Alarm\n",__FUNCTION__);
			gtimerstats.user_timeouts++;
		}
	}
	else { 
		INFO("%s: Non RTC Wakeup\n",__FUNCTION__);
		gtimerstats.non_rtc_wakeup++;
	}

		
	/* Wake up path. */

	/* Restore the original user set rtc alarm. */	
	if (alarm_user && rtc) {
		rtc_alarm_write(rtc, alarm_user);
	}

	fastpath_state = FASTPATH_STATE_WAKE;
end:
	if (rtc) {
		rtc_class_close(rtc);
	}

	DEBUG("================================%s (sleep req %d)\n", __FUNCTION__, 
			fastsleep_required);
	return fastsleep_required;
}

int register_fastpath_client(fastpath_client_t *client) {
	DEBUG("======%s\n", __FUNCTION__);
        list_add_tail(&client->entry,&fastpath_client_list_head);
	return 0;
}

void unregister_fastpath_client(fastpath_client_t *client) {
	DEBUG("======%s\n", __FUNCTION__);
        list_del(&client->entry);
}

static int __init fastpath_module_init(void)
{
	int rc;


	rc = fastpath_init();
	if (rc) {
		ERROR("%s could not init fastpath: %d\n",
				__FUNCTION__, rc);
		return -1;
	}

	//timerchk_sysfs_init();
        
	INIT_LIST_HEAD(&fastpath_client_list_head);

	return 0;
}

static void __exit fastpath_module_exit(void)
{
}

module_init(fastpath_module_init);
module_exit(fastpath_module_exit);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("Provides the ability to periodically wake device to check for critical battery.");
MODULE_LICENSE("GPL");
