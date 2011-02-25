/*
 * linux/arch/arm/mach-omap3pe/constraint.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on resource framework by
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * History:
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/notifier.h>
#include <linux/clk.h>
#if 0 /* 2.6.26 */
#include <linux/pm_qos_params.h>
#endif
#include <linux/latency.h>

#include <asm/arch/clock.h>
#include <asm/arch/prcm.h>
#include <asm/arch/resource.h>

#include "prcm_opp.h"


/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

//#define DEBUG_PRCM 1

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

/******************************************************************************
 *
 * Tables for operation point settings.
 *
 ******************************************************************************/

static struct res_handle *vdd1_res;

static unsigned int vdd1_users;
static unsigned int vdd1_arm_prenotifications;
static unsigned int vdd1_arm_postnotifications;
static unsigned int vdd1_dsp_prenotifications;
static unsigned int vdd1_dsp_postnotifications;
static struct atomic_notifier_head freq_arm_pre_notifier_list;
static struct atomic_notifier_head freq_arm_post_notifier_list;
static struct atomic_notifier_head freq_dsp_pre_notifier_list;
static struct atomic_notifier_head freq_dsp_post_notifier_list;

/******************************************************************************
 *
 * Notifier functions
 *
 ******************************************************************************/

static char *id_to_lname[] = {"lat_iva2", "lat_mpu", "lat_core1", "lat_core1",
			     "lat_3d", NULL, "lat_dss", "lat_cam", "lat_per",
				NULL, "lat_neon",
				NULL, NULL,
			      "lat_core1", "lat_usb",
};

static int nb_arm_freq_prenotify_func(struct notifier_block *n, unsigned long event,
			       void *ptr)
{
	unsigned int freq = get_arm_freq_for_opp(event);
	atomic_notifier_call_chain(&freq_arm_pre_notifier_list, freq, NULL);
	return 0;
}

static int nb_arm_freq_postnotify_func(struct notifier_block *n, unsigned long event,
				void *ptr)
{
	unsigned int freq = get_arm_freq_for_opp(event);
	atomic_notifier_call_chain(&freq_arm_post_notifier_list, freq, NULL);
	return 0;
}

static int nb_dsp_freq_prenotify_func(struct notifier_block *n, unsigned long event,
			       void *ptr)
{
	unsigned int freq = get_dsp_freq_for_opp(event);
	atomic_notifier_call_chain(&freq_dsp_pre_notifier_list, freq, NULL);
	return 0;
}

static int nb_dsp_freq_postnotify_func(struct notifier_block *n, unsigned long event,
				void *ptr)
{
	unsigned int freq = get_dsp_freq_for_opp(event);
	atomic_notifier_call_chain(&freq_dsp_post_notifier_list, freq, NULL);
	return 0;
}

static struct notifier_block nb_arm_freq_prenotify = {
	nb_arm_freq_prenotify_func,
	NULL,
};

static struct notifier_block nb_arm_freq_postnotify = {
	nb_arm_freq_postnotify_func,
	NULL,
};

static struct notifier_block nb_dsp_freq_prenotify = {
	nb_dsp_freq_prenotify_func,
	NULL,
};

static struct notifier_block nb_dsp_freq_postnotify = {
	nb_dsp_freq_postnotify_func,
	NULL,
};

#if 0 /* 2.6.26 */
static int activate_constraint(struct shared_resource *resp,
			unsigned short current_value,
			unsigned short target_value)
{
	static u8 pm_qos_req_added = 0;

	unsigned long type = resp->prcm_id;

	DPRINTK("CUR_VAL = %d, TAR_VAL = %d\n", current_value, target_value);
	if (type == RES_LATENCY_CO) {
		/* TODO Need to use pm_qos here */
		/* Remove previous latencies set by omap_lt_co */
		if (pm_qos_req_added)
			pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY,
							"omap_lt_co");
		switch (target_value) {
		case CO_LATENCY_WFI:
		/* Allows only wfi + tick suppression. State C1*/
		pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
						"omap_lt_co", 99);
		break;
		case CO_LATENCY_MPURET_COREON:
		/* Allows upto MPU RET. State C2*/
		pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
						"omap_lt_co", 3299);
		break;
		case CO_LATENCY_MPUOFF_COREON:
		/* Allows upto MPU OFF. State C3*/
		pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
						"omap_lt_co", 9999);
		break;
		case CO_LATENCY_MPUOFF_CORERET:
		/* Allows upto CORE RET. State C4*/
		pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
						"omap_lt_co", 39999);
		break;
		case CO_UNUSED:
		break;
		default:
		pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
						"omap_lt_co", target_value);
		break;
		}
	} else if ((type == PRCM_ARMFREQ_CONSTRAINT)) {
		unsigned short co;

		co = get_vdd1_arm_constraint_for_freq(target_value);
		resource_request(vdd1_res, co);
	} else if ((type == PRCM_DSPFREQ_CONSTRAINT)) {
		unsigned short co;

		co = get_vdd1_dsp_constraint_for_freq(target_value);
		resource_request(vdd1_res, co);
	} else if (type == PRCM_VDD1_CONSTRAINT) {
		if (target_value > CO_VDD1_OPP2) {
			if (vdd2_res == NULL) {
				vdd2_res = resource_get("co_fwk", "vdd2_opp");
				resource_request(vdd2_res, CO_VDD2_OPP3);
			}
		}
		vdd1_opp_setting(target_value);
		if (target_value <= CO_VDD1_OPP2) {
			if (vdd2_res != NULL) {
				resource_release(vdd2_res);
				resource_put(vdd2_res);
				vdd2_res = NULL;
			}
		}
	} else if (type == PRCM_VDD2_CONSTRAINT) {
		vdd2_opp_setting(target_value);
	}
	return 0;
}
#endif

int activate_constraint(struct shared_resource *resp,
			unsigned short current_value,
			unsigned short target_value)
{
	static struct res_handle *vdd2_res = NULL;

	unsigned long type = resp->prcm_id;

	DPRINTK( "%s: %s: CUR_VAL = %d, TAR_VAL = %d\n",
		__func__, resp->name, current_value, target_value);
	if (type == RES_LATENCY_CO) {
		switch (target_value) {
		case CO_LATENCY_WFI:
			/* Allows only wfi + tick suppression. State C1 */
			set_acceptable_latency("omap_lt_co", 99);
			break;
		case CO_LATENCY_MPURET_COREON:
			/* Allows upto MPU RET. State C2 */
			set_acceptable_latency("omap_lt_co", 3299);
			break;
		case CO_LATENCY_MPUOFF_COREON:
			/* Allows upto MPU OFF. State C3 */
			set_acceptable_latency("omap_lt_co", 4999);
			break;
		case CO_LATENCY_MPUOFF_CORERET:
			/* Allows upto CORE RET. State C4 */
			set_acceptable_latency("omap_lt_co", 39999);
			break;
		case CO_UNUSED:
			/* Removing the constraints */
			remove_acceptable_latency("omap_lt_co");
			break;
		default:
			set_acceptable_latency("omap_lt_co", target_value);
			break;
		}
	} else if ((type == PRCM_ARMFREQ_CONSTRAINT)) {
		unsigned short co;
		co = get_vdd1_arm_constraint_for_freq(target_value);
		resource_request(vdd1_res, co);

	} else if ((type == PRCM_DSPFREQ_CONSTRAINT)) {
		unsigned short co;
		co = get_vdd1_dsp_constraint_for_freq(target_value);
		resource_request(vdd1_res, co);

	} else if (type == PRCM_VDD1_CONSTRAINT) {
		vdd1_opp_setting(target_value);
		if (target_value > CO_VDD1_OPP2) {
			if (vdd2_res == NULL) {
				vdd2_res = resource_get("co_fwk", "vdd2_opp");
				resource_request(vdd2_res, CO_VDD2_OPP3);
			}
		}
		if (target_value <= CO_VDD1_OPP2) {
			if (vdd2_res != NULL) {
				resource_release(vdd2_res);
				resource_put(vdd2_res);
				vdd2_res = NULL;
			}
		}
	} else if (type == PRCM_VDD2_CONSTRAINT) {
		vdd2_opp_setting(target_value);
	}
	return 0;
}

int activate_pd_constraint(struct shared_resource *resp,
			   unsigned short current_value,
			   unsigned short target_value)
{
	struct res_handle_node *node;

	DPRINTK( "%s: %s: CUR_VAL = %d, TAR_VAL = %d\n",
		__func__, resp->name, current_value, target_value);

	if (list_empty(&resp->linked_res_handles))
		return 0;

	node = list_first_entry(&resp->linked_res_handles,
				struct res_handle_node,
				node);

	switch (target_value) {
	case CO_LATENCY_ON:
		/* Allows only ON power doamin state */
		return resource_request(node->handle, POWER_DOMAIN_ON);
		break;
	case CO_LATENCY_RET:
		/* Allows ON and RET power domain states */
		return resource_request(node->handle,
					POWER_DOMAIN_RET);
		break;
	case CO_UNUSED:
		/* Removing the constraints */
		return resource_release(node->handle);
		break;
	default:
		/* do nothing - allows all power domain states */
		break;
	}
	return 0;
}

int validate_constraint(struct shared_resource *res,
				unsigned short current_value,
				unsigned short target_value)
{
	u32 min_freq;
	u32 max_freq;

	if (target_value == DEFAULT_LEVEL)
		return 0;

	if (res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		min_freq = get_arm_freq_for_opp(MIN_VDD1_OPP);
		max_freq = get_arm_freq_for_opp(MAX_VDD1_OPP);
		if ((target_value < min_freq) || (target_value > max_freq)) {
			printk(KERN_ERR "Invalid ARM frequency requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		min_freq = get_dsp_freq_for_opp(MIN_VDD1_OPP);
		max_freq = get_dsp_freq_for_opp(MAX_VDD1_OPP);
		if ((target_value < min_freq) || (target_value > min_freq)) {
			printk(KERN_ERR "Invalid DSP frequency requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_VDD1_CONSTRAINT) {
		if ((target_value < MIN_VDD1_OPP) ||
				(target_value > MAX_VDD1_OPP)) {
			printk(KERN_ERR "Invalid VDD1 OPP requested\n");
			return -1;
		}
	} else if (res->prcm_id == PRCM_VDD2_CONSTRAINT) {
		if ((target_value < MIN_VDD2_OPP) ||
				(target_value > MAX_VDD2_OPP)) {
			printk(KERN_ERR "Invalid VDD2 OPP requested\n");
			return -1;
		}
	}
	return 0;
}

/* Wrappers to handle Constraints */
struct constraint_handle *constraint_get(const char *name,
					 struct constraint_id *cnstr_id)
{
	char *id;
	struct clk *clk;
	switch (cnstr_id->type) {
	case RES_CLOCK_RAMPUP_CO:
		clk = (struct clk *)cnstr_id->data;
		if ((clk == NULL) || IS_ERR(clk))
			return ERR_PTR(-ENOENT);

		id = id_to_lname[DOMAIN_ID(clk->prcmid) - 1];
		break;

	case RES_FREQ_CO:
		DPRINTK("Freq constraint %s requested\n",
			(char *)cnstr_id->data);
		if (vdd1_users == 0)
			vdd1_res = resource_get("co_fwk", "vdd1_opp");
		vdd1_users++;


			id = (char *)cnstr_id->data;
		break;

	default:
		id = (char *)cnstr_id->data;
		break;
	}
	/* Just calls the shared resource api */
	return (struct constraint_handle *) resource_get(name, id);
}
EXPORT_SYMBOL(constraint_get);

int constraint_set(struct constraint_handle *constraint,
					unsigned short constraint_level)
{
	return resource_request((struct res_handle *)constraint,
							constraint_level);
}
EXPORT_SYMBOL(constraint_set);

int constraint_remove(struct constraint_handle *constraint)
{
	return resource_release((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_remove);

void constraint_put(struct constraint_handle *constraint)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if ((strcmp("arm_freq", res_h->res->name) == 0)
		|| (strcmp("dsp_freq", res_h->res->name) == 0)) {
		vdd1_users--;
		if (vdd1_users == 0) {
			resource_release(vdd1_res);
			resource_put(vdd1_res);
			vdd1_res = NULL;
		}
	}
	resource_put((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_put);

void constraint_register_pre_notification(struct constraint_handle *constraint,
			 struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if (!vdd1_arm_prenotifications)
			resource_register_pre_notification(vdd1_res,
				&nb_arm_freq_prenotify, CO_VDD1_MAXLEVEL);
		atomic_notifier_chain_register(&freq_arm_pre_notifier_list, nb);
		vdd1_arm_prenotifications++;
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if (!vdd1_dsp_prenotifications)
			resource_register_pre_notification(vdd1_res,
				&nb_dsp_freq_prenotify, CO_VDD1_MAXLEVEL);

		atomic_notifier_chain_register(&freq_dsp_pre_notifier_list, nb);
		vdd1_dsp_prenotifications++;
	} else {
		resource_register_pre_notification(
			(struct res_handle *)constraint, nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_register_pre_notification);

void constraint_register_post_notification(struct constraint_handle *constraint,
		struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		if (!vdd1_arm_postnotifications)
			resource_register_post_notification(vdd1_res,
				&nb_arm_freq_postnotify, CO_VDD1_MAXLEVEL);


		atomic_notifier_chain_register(&freq_arm_post_notifier_list,
									nb);
		vdd1_arm_postnotifications++;
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		if (!vdd1_dsp_postnotifications)
			resource_register_post_notification(vdd1_res,
				&nb_dsp_freq_postnotify, CO_VDD1_MAXLEVEL);

		atomic_notifier_chain_register(&freq_dsp_post_notifier_list,
									nb);
		vdd1_dsp_postnotifications++;
	} else {
		resource_register_post_notification(
				(struct res_handle *)constraint,
				nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_register_post_notification);

void constraint_unregister_pre_notification(
		struct constraint_handle *constraint,
		struct notifier_block *nb,
		 unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		vdd1_arm_prenotifications--;
		atomic_notifier_chain_unregister(&freq_arm_pre_notifier_list,
									nb);
		if (!vdd1_arm_prenotifications)
			resource_unregister_pre_notification(vdd1_res,
			&nb_arm_freq_prenotify, CO_VDD1_MAXLEVEL);
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		vdd1_dsp_prenotifications--;
		atomic_notifier_chain_unregister(&freq_dsp_pre_notifier_list,
									nb);
		if (!vdd1_dsp_prenotifications)
			resource_unregister_pre_notification(vdd1_res,
				&nb_dsp_freq_prenotify, CO_VDD1_MAXLEVEL);
	} else {
		resource_unregister_pre_notification(
				(struct res_handle *)constraint,
				nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_unregister_pre_notification);

void constraint_unregister_post_notification(
		struct constraint_handle *constraint,
		struct notifier_block *nb, unsigned short target_level)
{
	struct  res_handle *res_h;
	res_h = (struct res_handle *)constraint;
	if (res_h->res->prcm_id == PRCM_ARMFREQ_CONSTRAINT) {
		vdd1_arm_postnotifications--;
		atomic_notifier_chain_unregister(&freq_arm_post_notifier_list,
									nb);
		if (!vdd1_arm_postnotifications)
			resource_unregister_post_notification(vdd1_res,
				&nb_arm_freq_postnotify, CO_VDD1_MAXLEVEL);
	} else if (res_h->res->prcm_id == PRCM_DSPFREQ_CONSTRAINT) {
		vdd1_dsp_postnotifications--;
		atomic_notifier_chain_unregister(&freq_dsp_post_notifier_list,
									nb);
		if (!vdd1_dsp_postnotifications)
			resource_unregister_post_notification(vdd1_res,
				&nb_dsp_freq_postnotify, CO_VDD1_MAXLEVEL);
		} else {
		resource_unregister_post_notification(
			(struct res_handle *)constraint, nb, target_level);
	}
}
EXPORT_SYMBOL(constraint_unregister_post_notification);

int constraint_get_level(struct constraint_handle *constraint)
{
	return resource_get_level((struct res_handle *)constraint);
}
EXPORT_SYMBOL(constraint_get_level);
