/* arch/arm/mach-msm/vreg.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/string.h>
#include <asm/arch/vreg.h>

#include "proc_comm.h"

#if defined(CONFIG_MSM_VREG_SWITCH_INVERTED)
#define VREG_SWITCH_ENABLE 0
#define VREG_SWITCH_DISABLE 1
#else
#define VREG_SWITCH_ENABLE 1
#define VREG_SWITCH_DISABLE 0
#endif

struct vreg {
	const char *name;
	unsigned id;
	int status;
	unsigned refcnt;
};

#define VREG(_name, _id, _status, _refcnt) \
	{ .name = _name, .id = _id, .status = _status, .refcnt = _refcnt }

static struct vreg vregs[] = {
	VREG("msma",	0, 0, 0),
	VREG("msmp",	1, 0, 0),
	VREG("msme1",	2, 0, 0),
	VREG("msmc1",	3, 0, 0),
	VREG("msmc2",	4, 0, 0),
	VREG("gp3",	5, 0, 0),
	VREG("msme2",	6, 0, 0),
	VREG("gp4",	7, 0, 0),
	VREG("gp1",	8, 0, 0),
	VREG("tcxo",	9, 0, 0),
	VREG("pa",	10, 0, 0),
	VREG("rftx",	11, 0, 0),
	VREG("rfrx1",	12, 0, 0),
	VREG("rfrx2",	13, 0, 0),
	VREG("synt",	14, 0, 0),
	VREG("wlan",	15, 0, 0),
	VREG("usb",	16, 0, 0),
	VREG("boost",	17, 0, 0),
	VREG("mmc",	18, 0, 0),
	VREG("ruim",	19, 0, 0),
	VREG("msmc0",	20, 0, 0),
	VREG("gp2",	21, 0, 0),
	VREG("gp5",	22, 0, 0),
	VREG("gp6",	23, 0, 0),
	VREG("rf",	24, 0, 0),
	VREG("rf_vco",	26, 0, 0),
	VREG("mpll",	27, 0, 0),
	VREG("s2",	28, 0, 0),
	VREG("s3",	29, 0, 0),
	VREG("rfubm",	30, 0, 0),
	VREG("ncp",	31, 0, 0),
};

struct vreg *vreg_get(struct device *dev, const char *id)
{
	int n;
	for (n = 0; n < ARRAY_SIZE(vregs); n++) {
		if (!strcmp(vregs[n].name, id))
			return vregs + n;
	}
	return ERR_PTR(-ENOENT);
}
EXPORT_SYMBOL(vreg_get);

void vreg_put(struct vreg *vreg)
{
}

int vreg_enable(struct vreg *vreg)
{
	unsigned id = vreg->id;
	int enable = VREG_SWITCH_ENABLE;

	if (vreg->refcnt == 0)
		vreg->status = msm_proc_comm(PCOM_VREG_SWITCH, &id, &enable);

	if ((vreg->refcnt < UINT_MAX) && (!vreg->status))
		vreg->refcnt++;

	return vreg->status;
}
EXPORT_SYMBOL(vreg_enable);

int vreg_disable(struct vreg *vreg)
{
	unsigned id = vreg->id;
	int disable = VREG_SWITCH_DISABLE;

	if (!vreg->refcnt)
		return 0;

	if (vreg->refcnt == 1)
		vreg->status = msm_proc_comm(PCOM_VREG_SWITCH, &id, &disable);

	if (!vreg->status)
		vreg->refcnt--;

	return vreg->status;
}
EXPORT_SYMBOL(vreg_disable);

int vreg_refcount(struct vreg *vreg)
{
	return vreg->refcnt;
}
EXPORT_SYMBOL(vreg_refcount);	

int vreg_set_level(struct vreg *vreg, unsigned mv)
{
	unsigned id = vreg->id;

	vreg->status = msm_proc_comm(PCOM_VREG_SET_LEVEL, &id, &mv);
	return vreg->status;
}
EXPORT_SYMBOL(vreg_set_level);

#if defined(CONFIG_DEBUG_FS)

static void vreg_debug_set(void *data, u64 val)
{
	struct vreg *vreg = data;
	switch (val) {
	case 0:
		vreg_disable(vreg);
		break;
	case 1:
		vreg_enable(vreg);
		break;
	default:
		vreg_set_level(vreg, val);
		break;
	}
	return;
}

static u64 vreg_debug_get(void *data)
{
	struct vreg *vreg = data;
	unsigned id_status;
	uint32_t cmd_status = PCOMM_PALM_DIAG_VREG_GET_STATUS;

	if (!vreg)
		return 0;

	id_status = vreg->id;
	if (msm_proc_comm(PCOM_PALM_DIAG_CMDS, &cmd_status, &id_status)) {
		printk(KERN_ERR "VREG_GET_STATUS \"%s\" failed\n", vreg->name);
		return 1;
	}
	return id_status ? 1 : 0;
}

DEFINE_SIMPLE_ATTRIBUTE(vreg_fops, vreg_debug_get, vreg_debug_set, "%llu\n");

static int __init vreg_debug_init(void)
{
	struct dentry *dent;
	int n;

	dent = debugfs_create_dir("vreg", 0);
	if (IS_ERR(dent))
		return 0;

	for (n = 0; n < ARRAY_SIZE(vregs); n++)
		(void) debugfs_create_file(vregs[n].name, 0644,
					   dent, vregs + n, &vreg_fops);

	return 0;
}

device_initcall(vreg_debug_init);
#endif
