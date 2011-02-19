/*
 *
 * shank charge control
 *
 * Copyright (C) 2010 Palm, Inc.
 * Author: Edward Falk <edward.falk@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/spinlock.h>

#include <mach/gpio.h>

#include "proc_comm.h"


/**
 * Proc-comm API to control USB charging, v2.
 *
 * @param command  charging subcommand.  See arch/arm/mach-msm/proc_comm.h
 * @param value    Depends on command.
 *
 * @return  The return value from msm_proc_comm()
 */
int proc_comm_chrg_control_v2(palm_chrg_cmd_v2_t command, int value)
{
	int ret_val = 0;
	unsigned data1, data2;

	data1 = PCOMM_PALM_DIAG_CONTROL_CHRG_V2;
	data2 = ((command & 0xff)) | (value & 0xffff) << 0x8;
	ret_val = msm_proc_comm(PCOM_PALM_DIAG_CMDS, &data1, &data2);

	return ret_val;
}


static enum charge_type {NONE, USB, WALL, PUCK, HOLSTER} charge_type = NONE;
static int trickle = 0;		/* Trickle charging selected */
static int maxMA = 0;		/* Charging current limit */
static int vmax = 0;
static int trickle_imax = 0;
static int battfet = 0;

	/* Utility functions -- wrappers around proc_comm_chrg_control_v2 */

static int pmic_chrg_battfet_close(int closed)
{
	int rc = proc_comm_chrg_control_v2(PCOMM_CHGR_BATT_FET_CTRL, closed);
	if (rc == 0) battfet = closed;
	return rc;
}

static int pmic_chrg_imax_sel(int current_mA)
{
	int rc;
	if (current_mA < 50 || current_mA > 1600) {
		printk(KERN_WARNING "%s current_mA %d is out of range",
			__FUNCTION__, current_mA);
		return -EINVAL;
	}
	rc = proc_comm_chrg_control_v2(PCOMM_CHGR_IMAX_SEL_CTRL, current_mA);
	if (rc == 0) maxMA = current_mA;
	return rc;
}

static int pmic_chrg_vmax_sel(int voltage_mV)
{
	int rc;
	if (voltage_mV < 2400 || voltage_mV > 5575) {
		printk(KERN_WARNING "%s voltage_mV %d is out of range",
			__FUNCTION__, voltage_mV);
		return -EINVAL;
	}
	rc = proc_comm_chrg_control_v2(PCOMM_CHGR_VMAX_SEL_CTRL, voltage_mV);
	if (rc == 0) vmax = voltage_mV;
	return rc;
}

#if 0
static int pmic_chrg_auto_charge_enable(int enabled)
{
	return proc_comm_chrg_control_v2(PCOMM_CHGR_AUTO_CHG_CTRL, enabled);
}
#endif

static int pmic_chrg_trickle_charge_enable(int enabled)
{
	int rc = proc_comm_chrg_control_v2(PCOMM_CHGR_TRCKL_CHG_CTRL, enabled);
	if (rc == 0) trickle = enabled;
	return rc;
}

static int pmic_chrg_trickle_charge_imax_sel(int mA)
{
	int rc;
	if (mA < 10 || mA > 160) {
		printk(KERN_WARNING "%s mA %d is out of range",
			__FUNCTION__, mA);
		return -EINVAL;
	}
	rc = proc_comm_chrg_control_v2(PCOMM_CHGR_TRCKL_IMAX_CTRL, mA);
	if (rc == 0) trickle_imax = mA;
	return rc;
}



static int stop_charging(void)
{
	/* Stop charging: open battfet, turn off trickle */
	int rc;
	if ((rc = pmic_chrg_imax_sel(maxMA)) != 0 ||
	    (rc = pmic_chrg_battfet_close(false)) != 0 ||
	    (rc = pmic_chrg_trickle_charge_enable(false)) != 0)
		return rc;
	charge_type = NONE;
	return 0;
}

static int usb_charging(void)
{
	/* Stop charging: open battfet, turn off trickle */
	/* TODO: does the PMIC allow us to differentiate charge sources? */
	int rc;
	if (maxMA >= 100) {
		trickle = 0;
		if ((rc = pmic_chrg_imax_sel(maxMA)) != 0 ||
		    (rc = pmic_chrg_vmax_sel(4200)) != 0 ||
		    (rc = pmic_chrg_battfet_close(true)) != 0)
			return rc;

	} else {
		trickle = 1;
		if ((rc = pmic_chrg_imax_sel(500)) != 0 ||
		    (rc = pmic_chrg_vmax_sel(4200)) != 0 ||
		    (rc = pmic_chrg_battfet_close(false)) != 0 ||
		    (rc = pmic_chrg_trickle_charge_imax_sel(80)) != 0 ||
		    (rc = pmic_chrg_trickle_charge_enable(true)) != 0)
			return rc;
	}
	charge_type = USB;
	return 0;
}

static int wall_charging(void)
{
	/* TODO: differentiate charger sources.  For now, they're all the
	 * same.
	 */
	int rc;
	if ((rc = usb_charging()) == 0)
		charge_type = WALL;
	return rc;
}

static int puck_charging(void)
{
	/* TODO: differentiate charger sources.  For now, they're all the
	 * same.
	 */
	int rc;
	if ((rc = usb_charging()) == 0)
		charge_type = PUCK;
	return rc;
}

static int holster_charging(void)
{
	/* TODO: differentiate charger sources.  For now, they're all the
	 * same.
	 */
	int rc;
	if ((rc = usb_charging()) == 0)
		charge_type = HOLSTER;
	return rc;
}




	/* Interface to /sysfs/power/charging */

int target_charging_set_imax(int imax_mA)
{
	return pmic_chrg_imax_sel(imax_mA);
}

int target_charging_get_imax(void)
{
	return maxMA;
}

int target_charging_set_vmax(int vmax_mV)
{
	return pmic_chrg_vmax_sel(vmax_mV);
}

int target_charging_get_vmax(void)
{
	return vmax;
}

int target_charging_set_trickle_imax(int trickle_imax_mA)
{
	return pmic_chrg_trickle_charge_imax_sel(trickle_imax_mA);
}

int target_charging_get_trickle_imax(void)
{
	return trickle_imax;
}

int target_charging_set_battfet(int closed)
{
	return pmic_chrg_battfet_close(closed);
}

int target_charging_get_battfet(void)
{
	return battfet;
}

int target_charging_set_trickle(int enable)
{
	return pmic_chrg_trickle_charge_enable(enable);
}

int target_charging_get_trickle(void)
{
	return trickle;
}





	/* TODO: define a standard internal API for controlling charging,
	 * and move the stuff below to its own architecture-independent
	 * file.
	 */

struct kobject *charging_kobj;


/**
 * Show the current charging type and current limit.
 */
static ssize_t charging_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	switch (charge_type) {
		default:
		case NONE: return sprintf(buf, "none|%d\n", maxMA);
		case USB: return sprintf(buf, "usb|%d\n", maxMA);
		case WALL: return sprintf(buf, "wall|%d\n", maxMA);
		case PUCK: return sprintf(buf, "puck|%d\n", maxMA);
		case HOLSTER: return sprintf(buf, "holster|%d\n", maxMA);
	}
}

/*
 * Utility for charging_store()
 */
static int getMA(const char *buf, int dflt)
{
	int val;
	if (*buf != '|') return dflt;
	if (sscanf(buf+1, "%d", &val) != 1) return dflt;
	return val;
}
#define CHARGE_CURRENT_LIMIT 1000

/**
 * Parse the input for a charging command and optional current limit.
 */
static ssize_t charging_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	if (strncmp(buf, "off", 3) == 0 || strncmp(buf, "none", 4) == 0) {
		stop_charging();
	}
	else if (strncmp(buf, "usb", 3) == 0) {
		maxMA = getMA(buf+3, CHARGE_CURRENT_LIMIT);
		//USB cable will limit the maximal current drawn from USB bus
		//if (maxMA > 500) maxMA = 500;	/* USB spec sets this limit */
		usb_charging();
	}
	else if (strncmp(buf, "wall", 4) == 0) {
		maxMA = getMA(buf+4, CHARGE_CURRENT_LIMIT);
		wall_charging();
	}
	else if (strncmp(buf, "puck", 4) == 0) {
		maxMA = getMA(buf+4, CHARGE_CURRENT_LIMIT);
		puck_charging();
	}
	else if (strncmp(buf, "holster", 7) == 0) {
		maxMA = getMA(buf+7, CHARGE_CURRENT_LIMIT);
		holster_charging();
	}
	else {
		printk(KERN_INFO "Unrecognized charging command: %s\n", buf);
	}
	return n;
}

static ssize_t imax_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", target_charging_get_imax());
}

static ssize_t imax_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int v = simple_strtol(buf, NULL, 10);
	return target_charging_set_imax(v) ? -EINVAL : len;
}

static ssize_t vmax_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", target_charging_get_vmax());
}

static ssize_t vmax_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int v = simple_strtol(buf, NULL, 10);
	return target_charging_set_vmax(v) ? -EINVAL : len;
}

static ssize_t trickle_imax_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", target_charging_get_trickle_imax());
}

static ssize_t trickle_imax_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t len)
{
	int v = simple_strtol(buf, NULL, 10);
	return target_charging_set_trickle_imax(v) ? -EINVAL : len;
}

static ssize_t battfet_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", target_charging_get_battfet());
}

static ssize_t battfet_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int v = simple_strtol(buf, NULL, 10);
	return target_charging_set_battfet(v!=0) ? -EINVAL : len;
}

static ssize_t trickle_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", target_charging_get_trickle());
}

static ssize_t trickle_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int v = simple_strtol(buf, NULL, 10);
	return target_charging_set_trickle(v!=0) ? -EINVAL : len;
}



static struct kobj_attribute charging_attr = {
	.attr = { .name = "charging", .mode = 0644, },
	.show = charging_show,
	.store = charging_store,
};

static struct kobj_attribute imax_attr = {
	.attr = { .name = "imax", .mode = 0644, },
	.show = imax_show,
	.store = imax_store,
};

static struct kobj_attribute vmax_attr = {
	.attr = { .name = "vmax", .mode = 0644, },
	.show = vmax_show,
	.store = vmax_store,
};

static struct kobj_attribute trickle_imax_attr = {
	.attr = { .name = "trickle_imax", .mode = 0644, },
	.show = trickle_imax_show,
	.store = trickle_imax_store,
};

static struct kobj_attribute battfet_attr = {
	.attr = { .name = "battfet", .mode = 0644, },
	.show = battfet_show,
	.store = battfet_store,
};

static struct kobj_attribute trickle_attr = {
	.attr = { .name = "trickle", .mode = 0644, },
	.show = trickle_show,
	.store = trickle_store,
};

static struct attribute * g[] = {
	&charging_attr.attr,
	&imax_attr.attr,
	&vmax_attr.attr,
	&trickle_imax_attr.attr,
	&battfet_attr.attr,
	&trickle_attr.attr,
	NULL,
};

static struct attribute_group charging_group = {
	.attrs = g,
};


static int __init shank_charging_init(void)
{
	if (power_kobj == NULL)
		return -ENOENT;
	charging_kobj = kobject_create_and_add("charging", power_kobj);
	if (charging_kobj == NULL)
		return -ENOMEM;
	return sysfs_create_group(charging_kobj, &charging_group);
}
__initcall(shank_charging_init);
