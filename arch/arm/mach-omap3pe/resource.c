/*
 * linux/arch/arm/mach-omap3/resource.c
 *
 * Copyright (C) 2008 Palm, Inc.
 *
 * Based on OMAP34XX Shared Resource Framework
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
#include <asm/arch/resource.h>
#include <asm/arch/clock.h>
#include <asm/arch/prcm.h>

#include <asm/arch/power_companion.h>
#include "prcm_pwr.h"
#include "constraint.h"
#include "prcm-regs.h"

//#define DEBUG_PRCM 1

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

/* global variables which can be used in idle_thread */
short	core_active = LOGICAL_UNUSED;

/* Activation function to change target level for a Power Domain resource
 * Transtitions from RET to OFF and OFF to RET are not allowed. */
static int activate_power_res(struct shared_resource *resp,
			       unsigned short current_level,
			       unsigned short target_level)
{
	int ret = 0;
	unsigned long prcm_id = resp->prcm_id;

	/* Common code for transitioning to RET/OFF. */
	switch (target_level) {
	case POWER_DOMAIN_RET:
	case POWER_DOMAIN_OFF:
#ifdef CONFIG_HW_SUP_TRANS
		ret = prcm_set_clock_domain_state(prcm_id, PRCM_NO_AUTO,
								PRCM_FALSE);
		if (ret != PRCM_PASS) {
			printk(KERN_ERR"FAILED in clock domain to NO_AUTO for"
						" domain %lu \n", prcm_id);
			return -1;
		}

		switch (prcm_id) {
		case DOM_DSS:
		case DOM_CAM:
		case DOM_PER:
		case DOM_USBHOST:
		case DOM_SGX:
			prcm_clear_sleep_dependency(prcm_id, PRCM_SLEEPDEP_EN_MPU);
			prcm_clear_wkup_dependency (prcm_id, PRCM_WKDEP_EN_MPU);
			break;
		}
#endif /* CONFIG_HW_SUP_TRANS */
		if (current_level != POWER_DOMAIN_ON) {
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_FORCE);
			if (ret != PRCM_PASS)
				return ret;
		}
		break;
	}

	switch (target_level) {
	case POWER_DOMAIN_ON:
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_FORCE);
#ifdef CONFIG_HW_SUP_TRANS
		switch (prcm_id) {
		case DOM_DSS:
		case DOM_CAM:
		case DOM_PER:
		case DOM_USBHOST:
		case DOM_SGX:
			if (prcm_set_wkup_dependency(prcm_id,
					PRCM_WKDEP_EN_MPU) != PRCM_PASS) {
				printk("Domain %lu : wakeup dependency could"
						 " not be set\n", prcm_id);
				return -1;
			}
			if (prcm_set_sleep_dependency(prcm_id,
					PRCM_SLEEPDEP_EN_MPU) != PRCM_PASS) {
				printk("Domain %lu : sleep dependency could"
					" not be set\n", prcm_id);
				return -1;
			}
			/* NO BREAK */
		case DOM_NEON:
		case DOM_IVA2:
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_ON, PRCM_AUTO);
			break;
		}
#endif /* #ifdef CONFIG_HW_SUP_TRANS */
		break;

	case POWER_DOMAIN_RET:
	case POWER_DOMAIN_OFF:
		if (prcm_id == DOM_PER)
			ret = prcm_set_power_domain_state(prcm_id,
						POWER_DOMAIN_RET, PRCM_FORCE);
		else
#ifndef CONFIG_CORE_OFF
			/* HACK:
			 *   We only put DOM_USBHOST in RET instead of OFF,
			 *   otherwise the USB root hub will not work after
			 *   suspend/resume.  There is ongoing work in 2.6.25
			 *   that should solve this problem at some point,
			 *   until then we keep this hack around.
			 *   -wgr-
			 */
			if (DOM_USBHOST == prcm_id) 
				ret = prcm_set_power_domain_state(prcm_id,
							  POWER_DOMAIN_RET,
							  PRCM_FORCE);
			else
#endif
				ret = prcm_set_power_domain_state(prcm_id,
							  POWER_DOMAIN_OFF,
							  PRCM_FORCE);
		break;
	default:
		ret = PRCM_FAIL;
	}
	return ret;
}

/* Activation function to change target level for a Logical resource */
static int activate_logical_res(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level)
{
	unsigned long prcm_id = resp->prcm_id;

	switch (prcm_id) {
	case DOM_CORE1:
		core_active = (target_level)?LOGICAL_USED:LOGICAL_UNUSED;
		break;
	default:
		return -1;
	}
	return 0;
}

/* Function to change target level for Memory or Logic resource */
static int prcm_set_memory_resource_on_state(unsigned short state)
{
	u32 value;
	value = PM_PWSTCTRL_CORE;
	switch (state) {
	case MEMORY_ON:
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_PWRSTATEBIT2;
		value |= PRCM_CORE_MEM1ONBITS | PRCM_CORE_MEM2ONBITS;
		break;
	case MEMORY_OFF:
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_PWRSTATEBIT2;
		value &= ~(PRCM_CORE_MEM1ONBITS | PRCM_CORE_MEM2ONBITS);
		break;
	default:
		pr_debug("Resource State Not Supported");
		return -1;
	}
	PM_PWSTCTRL_CORE = value;
	return 0;
}

static int activate_memory_logic(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level)
{
	unsigned long domain_id;
	unsigned long prcm_id = resp->prcm_id;
	domain_id = DOMAIN_ID(prcm_id);
	switch (prcm_id) {
	case PRCM_CORE_MEM1ON:
		prcm_set_memory_resource_on_state(target_level);
		break;
	case PRCM_CORE_MEM2ON:
		prcm_set_memory_resource_on_state(target_level);
		break;
	case PRCM_CORE_MEM1RET:
		break;
	case PRCM_CORE_MEM2RET:
		break;
	case PRCM_CORE_LOGICRET:
		break;
	default :
		DPRINTK("Unsupported resourse\n");
		return -1;
	}
	return 0;
}


/* Functio to validate the memory or logic resource transition */
int validate_memory_logic(struct shared_resource *resp,
			 unsigned short current_level,
			 unsigned short target_level)
{
	if (target_level >= resp->max_levels)
		return -1;
	return 0;
}

/******************************************************************************
 *
 * Validate if the power domain transition from current to target level is
 * valid.
 *
 ******************************************************************************/

int activate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level)
{
	unsigned long prcm_id = resp->prcm_id;
	int ret;
	ret = prcm_dpll_clock_auto_control(prcm_id, target_level);
	if (ret == PRCM_FAIL) {
		DPRINTK("Invalid DPLL Autoidle resource state\n");
		return -1;
	}
	return 0;
}

int validate_autoidle_resource(struct shared_resource *resp,
			unsigned short current_level,
			unsigned short target_level)
{
	int invalidlevelmin = 2, invalidlevelmax = 4;
	if (target_level >= resp->max_levels) {
		printk(KERN_ERR"Invalid Target Level\n");
		return -1;
	}
	if (strncmp(resp->name, "core_autoidle_res", 17) == 0)
	if (target_level >= invalidlevelmin &&
			target_level <= invalidlevelmax) {
		printk(KERN_ERR"Invalid Target Level\n");
		return -1;
	}
	return 0;
}

int activate_triton_power_res(struct shared_resource *resp,
	unsigned short current_level,
	unsigned short target_level)
{
	int result;
#ifdef CONFIG_CORE_OFF
	save_scratchpad_contents();
#endif

	result = twl4030_ldo_set_voltage(resp->prcm_id, target_level);
	return result;
}

int validate_triton_power_res(struct shared_resource *resp,
	unsigned short current_level,
	unsigned short target_level)
{
	if (target_level >= resp->max_levels)
		return -1;
	return 0;
}


/* Validate if the power domain transition from current to target level is*/
/* valid							     	*/
static int validate_power_res(struct shared_resource *res,
			unsigned short current_level,
			unsigned short target_level)
{
	DPRINTK("Current_level = %d, Target_level = %d\n",
			current_level, target_level);

	if ((target_level >= res->max_levels)) {
		DPRINTK("Invalid target_level requested\n");
		return -1;
	}

	/* RET to OFF and OFF to RET transitions not allowed for IVA*/
	if (res->prcm_id == DOM_IVA2) {
		if ((current_level < POWER_DOMAIN_ON) &&
				(target_level != POWER_DOMAIN_ON)) {
			DPRINTK("%d to %d transitions for"
					" Power Domain IVA are not allowed\n",
					current_level, target_level);
			return -1;
		}
	}

	return 0;
}

/* Validate if the logical resource transition is valid */
static int validate_logical_res(struct shared_resource *res,
				unsigned short current_level,
				unsigned short target_level)
{
	if (target_level >= res->max_levels)
		return -1;
	return 0;
}

/******************************************************************************
 *
 * Data structures for resource registration
 *
 ******************************************************************************/

static struct shared_resource dss = {
	.name = "dss",
	.prcm_id = DOM_DSS,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource cam = {
	.name = "cam",
	.prcm_id = DOM_CAM,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource iva2 = {
	.name = "iva2",
	.prcm_id = DOM_IVA2,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource sgx = {
	.name = "sgx",
	.prcm_id = DOM_SGX,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource usbhost = {
	.name = "usb",
	.prcm_id = DOM_USBHOST,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource per = {
	.name = "per",
	.prcm_id = DOM_PER,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource neon  = {
	.name = "neon",
	.prcm_id = DOM_NEON,
	.res_type = RES_POWER_DOMAIN,
	.no_of_users = 0,
	.curr_level = POWER_DOMAIN_OFF,
	.max_levels = POWER_DOMAIN_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_power_res,
	.validate = validate_power_res,
};

static struct shared_resource core = {
	.name = "core",
	.prcm_id = DOM_CORE1,
	.res_type = RES_LOGICAL,
	.no_of_users = 0,
	.curr_level = LOGICAL_UNUSED,
	.max_levels = LOGICAL_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_logical_res,
	.validate = validate_logical_res,
};

/* Constraint resources */
static struct shared_resource latency = {
	.name = "latency",
	.prcm_id = RES_LATENCY_CO,
	.res_type = RES_LATENCY_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource arm_freq = {
	.name = "arm_freq",
	.prcm_id = PRCM_ARMFREQ_CONSTRAINT,
	.res_type = RES_FREQ_CO,
	.no_of_users = 0,
	.curr_level = 0, /* The initial state will be determined later */
	.max_levels = CO_VDD1_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource dsp_freq = {
	.name = "dsp_freq",
	.prcm_id = PRCM_DSPFREQ_CONSTRAINT,
	.res_type = RES_FREQ_CO,
	.no_of_users = 0,
	.curr_level = 0, /* The initial state will be determined later */
	.max_levels = CO_VDD1_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource vdd1_opp = {
	.name = "vdd1_opp",
	.prcm_id = PRCM_VDD1_CONSTRAINT,
	.res_type = RES_OPP_CO,
	.no_of_users = 0,
	.curr_level = 0, /* The initial state will be determined later */
	.max_levels = CO_VDD1_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static struct shared_resource vdd2_opp = {
	.name = "vdd2_opp",
	.prcm_id = PRCM_VDD2_CONSTRAINT,
	.res_type = RES_OPP_CO,
	.no_of_users = 0,
	.curr_level = 0, /* The initial state will be determined later */
	.max_levels = CO_VDD2_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_constraint,
	.validate = validate_constraint,
};

static char *lat_dss_linked_res[] = {"dss",};

static struct shared_resource lat_dss = {
	.name = "lat_dss",
	.prcm_id = PRCM_DSS_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_dss_linked_res) /
			  sizeof(lat_dss_linked_res[0]),
	.linked_res_names = lat_dss_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_cam_linked_res[] = {"cam",};

static struct shared_resource lat_cam = {
	.name = "lat_cam",
	.prcm_id = PRCM_CAM_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_cam_linked_res) /
			  sizeof(lat_cam_linked_res[0]),
	.linked_res_names = lat_cam_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_iva2_linked_res[] = {"iva2",};

static struct shared_resource lat_iva2 = {
	.name = "lat_iva2",
	.prcm_id = PRCM_IVA2_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_iva2_linked_res) /
			  sizeof(lat_iva2_linked_res[0]),
	.linked_res_names = lat_iva2_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_3d_linked_res[] = {"sgx",};

static struct shared_resource lat_3d = {
	.name = "lat_3d",
	.prcm_id = PRCM_3D_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_3d_linked_res) /
			  sizeof(lat_3d_linked_res[0]),
	.linked_res_names = lat_3d_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_usbhost_linked_res[] = {"usb",};

static struct shared_resource lat_usbhost = {
	.name = "lat_usbhost",
	.prcm_id = PRCM_USBHOST_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_usbhost_linked_res) /
			  sizeof(lat_usbhost_linked_res[0]),
	.linked_res_names = lat_usbhost_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_per_linked_res[] = {"per",};

static struct shared_resource lat_per = {
	.name = "lat_per",
	.prcm_id = PRCM_PER_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_per_linked_res) /
			  sizeof(lat_per_linked_res[0]),
	.linked_res_names = lat_per_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_neon_linked_res[] = {"neon",};

static struct shared_resource lat_neon = {
	.name = "lat_neon",
	.prcm_id = PRCM_NEON_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_neon_linked_res) /
			  sizeof(lat_neon_linked_res[0]),
	.linked_res_names = lat_neon_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

static char *lat_core1_linked_res[] = {"core",};

static struct shared_resource lat_core1 = {
	.name = "lat_core1",
	.prcm_id = PRCM_CORE1_CONSTRAINT,
	.res_type = RES_CLOCK_RAMPUP_CO,
	.no_of_users = 0,
	.curr_level = CO_UNUSED,
	.max_levels = CO_MAXLEVEL,
	.linked_res_num = sizeof(lat_core1_linked_res) /
			  sizeof(lat_core1_linked_res[0]),
	.linked_res_names = lat_core1_linked_res,
	.action = activate_pd_constraint,
	.validate = validate_constraint,
};

/* Domain logic and memory resource */
static struct shared_resource mpu_l2cacheon = {
	.name = "mpu_l2cacheon",
	.prcm_id = PRCM_MPU_L2CACHEON,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_OFF,
	.max_levels = MEMORY_MAXLEVEL_DOMAINON,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource mpu_l2cacheret = {
	.name = "mpu_l2cacheret",
	.prcm_id = PRCM_MPU_L2CACHERET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_OFF,
	.max_levels = MEMORY_MAXLEVEL_DOMAINRET,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource mpu_logicl1cacheret = {
	.name = "mpu_logicl1cacheret",
	.prcm_id = PRCM_MPU_LOGICL1CACHERET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = LOGIC_OFF,
	.max_levels = LOGIC_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem2on = {
	.name = "core_mem2on",
	.prcm_id = PRCM_CORE_MEM2ON,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_ON,
	.max_levels = MEMORY_MAXLEVEL_DOMAINON,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem1on = {
	.name = "core_mem1on",
	.prcm_id = PRCM_CORE_MEM1ON,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_ON,
	.max_levels = MEMORY_MAXLEVEL_DOMAINON,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem2ret = {
	.name = "core_mem2ret",
	.prcm_id = PRCM_CORE_MEM2RET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_RET,
	.max_levels = MEMORY_MAXLEVEL_DOMAINRET,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_mem1ret = {
	.name = "core_mem1ret",
	.prcm_id = PRCM_CORE_MEM1RET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = MEMORY_RET,
	.max_levels = MEMORY_MAXLEVEL_DOMAINRET,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource core_logicret = {
	.name = "core_logicret",
	.prcm_id = PRCM_CORE_LOGICRET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = LOGIC_RET,
	.max_levels = LOGIC_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource per_logicret = {
	.name = "per_logicret",
	.prcm_id = PRCM_PER_LOGICRET,
	.res_type = RES_MEMORY_LOGIC,
	.no_of_users = 0,
	.curr_level = LOGIC_OFF,
	.max_levels = LOGIC_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_memory_logic,
	.validate = validate_memory_logic,
};

static struct shared_resource per_autoidle_res = {
	.name = "per_autoidle_res",
	.prcm_id = DPLL4_PER ,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};


static struct shared_resource mpu_autoidle_res = {
	.name = "mpu_autoidle_res",
	.prcm_id = DPLL1_MPU,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL ,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource core_autoidle_res = {
	.name = "core_autoidle_res",
	.prcm_id = DPLL3_CORE,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_CORE_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource iva2_autoidle_res = {
	.name = "iva2_autoidle_res",
	.prcm_id = DPLL2_IVA2,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource per2_autoidle_res = {
	.name = "per2_autoidle_res",
	.prcm_id = DPLL5_PER2,
	.res_type = RES_DPLL,
	.no_of_users = 0,
	.curr_level = DPLL_AUTOIDLE,
	.max_levels = DPLL_RES_MAXLEVEL,
	.linked_res_num = 0,
	.action = activate_autoidle_resource,
	.validate = validate_autoidle_resource,
};

static struct shared_resource t2_vaux1 = {
	.name = "t2_vaux1",
	.prcm_id = TWL4030_VAUX1_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX1_OFF,
	.max_levels = T2_VAUX1_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vaux2 = {
	.name = "t2_vaux2",
	.prcm_id = TWL4030_VAUX2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX2_OFF,
	.max_levels = T2_VAUX2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vaux3 = {
	.name = "t2_vaux3",
	.prcm_id = TWL4030_VAUX3_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX3_OFF,
	.max_levels = T2_VAUX3_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vaux4 = {
	.name = "t2_vaux4",
	.prcm_id = TWL4030_VAUX4_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VAUX4_OFF,
	.max_levels = T2_VAUX4_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vmmc1 = {
	.name = "t2_vmmc1",
	.prcm_id = TWL4030_VMMC1_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VMMC1_OFF,
	.max_levels = T2_VMMC1_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vmmc2 = {
	.name = "t2_vmmc2",
	.prcm_id = TWL4030_VMMC2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VMMC2_OFF,
	.max_levels = T2_VMMC2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vpll2 = {
	.name = "t2_vpll2",
	.prcm_id = TWL4030_VPLL2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VPLL2_OFF,
	.max_levels = T2_VPLL2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vsim = {
	.name = "t2_vsim",
	.prcm_id = TWL4030_VSIM_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VSIM_OFF,
	.max_levels = T2_VSIM_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vdac = {
	.name = "t2_vdac",
	.prcm_id = TWL4030_VDAC_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VDAC_OFF,
	.max_levels = T2_VDAC_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vintana2 = {
	.name = "t2_vintana2",
	.prcm_id = TWL4030_VINTANA2_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VINTANA2_OFF,
	.max_levels = T2_VINTANA2_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vio = {
	.name = "t2_vio",
	.prcm_id = TWL4030_VIO_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VIO_OFF,
	.max_levels = T2_VIO_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vusb1v5 = {
	.name = "t2_vusb1v5",
	.prcm_id = TWL4030_VUSB1V5_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VUSB1V5_OFF,
	.max_levels = T2_VUSB1V5_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vusb1v8 = {
	.name = "t2_vusb1v8",
	.prcm_id = TWL4030_VUSB1V8_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VUSB1V8_OFF,
	.max_levels = T2_VUSB1V8_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource t2_vusb3v1 = {
	.name = "t2_vusb3v1",
	.prcm_id = TWL4030_VUSB3V1_ID,
	.res_type = RES_T2_POWER,
	.no_of_users = 0,
	.curr_level = T2_VUSB3V1_OFF,
	.max_levels = T2_VUSB3V1_MAX,
	.linked_res_num = 0,
	.action = activate_triton_power_res,
	.validate = validate_triton_power_res,
};

static struct shared_resource *res_list[] = {
	/* Power domain resources */
	&dss,
	&cam,
	&iva2,
	&sgx,
	&usbhost,
	&per,
	&neon,
	/* Logical resources */
	&core,
	/* Constraints */
	&latency,
	&arm_freq,
	&dsp_freq,
	&vdd1_opp,
	&vdd2_opp,
	&lat_dss,
	&lat_cam,
	&lat_iva2,
	&lat_3d,
	&lat_usbhost,
	&lat_per,
	&lat_neon,
	&lat_core1,
	/*memory and logic resource */
	&mpu_l2cacheon,
	&mpu_l2cacheret,
	&mpu_logicl1cacheret,
	&core_mem2on,
	&core_mem1on,
	&core_mem2ret,
	&core_mem1ret,
	&core_logicret,
	&per_logicret,
	&per_autoidle_res,
	&mpu_autoidle_res,
	&core_autoidle_res,
	&iva2_autoidle_res,
	&per2_autoidle_res,
	&t2_vaux1,
	&t2_vaux2,
	&t2_vaux3,
	&t2_vaux4,
	&t2_vmmc1,
	&t2_vmmc2,
	&t2_vpll2,
	&t2_vsim,
	&t2_vdac,
	&t2_vintana2,
	&t2_vio,
	&t2_vusb1v5,
	&t2_vusb1v8,
	&t2_vusb3v1,
	NULL
};

int __init omap3_resource_init(void)
{
	return resource_init(res_list);
}

/******************************************************************************
 *
 * PROCFS interface implementation
 *
 ******************************************************************************/

#ifdef CONFIG_PROC_FS

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void *omap_shared_res_start(struct seq_file *m, loff_t * pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *omap_shared_res_next(struct seq_file *m, void *v, loff_t * pos)
{
	++*pos;
	return NULL;
}

static void omap_shared_res_stop(struct seq_file *m, void *v)
{
}

int omap_shared_res_show(struct seq_file *m, void *v)
{
	int  i;
	struct shared_resource *resp;
	struct users_list *user;

	for (i = 0; i < ARRAY_SIZE(res_list); i++) {

		resp = res_list[i];
		if (resp == NULL)
			continue;
			
		seq_printf(m, "Number of users for domain %s: %lu\n",
			   resp->name, resp->no_of_users);

		list_for_each_entry(user, &(resp->users_list), node) {
			seq_printf(m, "User: %s  Level: %lu\n",
				   user->usr_name, user->level);
		}
	}
	return 0;
}

static struct seq_operations omap_shared_res_op = {
	.start = omap_shared_res_start,
	.next  = omap_shared_res_next,
	.stop  = omap_shared_res_stop,
	.show  = omap_shared_res_show
};

static int omap_shared_res_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &omap_shared_res_op);
}

static struct file_operations proc_shared_res_ops = {
	.open		= omap_shared_res_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

/* This API creates a proc entry for shared resources. */
int proc_entry_create(void)
{
	struct proc_dir_entry *entry;

	/* Create a proc entry for shared resources */
	entry = create_proc_entry("shared_resources", 0, NULL);
	if (entry) {
		entry->proc_fops = &proc_shared_res_ops;
	}
	else {
		printk(KERN_ERR "create_proc_entry 'shared_resources' failed\n");
	}

	return 0;
}

__initcall(proc_entry_create);
#endif

#if 0
/* What are we trying to accomplish here??
 *   -wgr-
 */
/* Turn off unused power domains during bootup
 * If OFFMODE macro is enabled, all power domains are turned off during bootup
 * If OFFMODE macro is not enabled, all power domains except IVA and GFX
 * are put to retention and IVA,GFX are put to off
 * NEON is not put to off because it is required by VFP
 */
int turn_power_domains_off(void)
{
	struct shared_resource **resp;
	u32 state;
	u32 prcmid;
	for (resp = res_list; *resp; resp++) {
		if (list_empty(&((*resp)->users_list))) {
			if ((*resp)->res_type == RES_POWER_DOMAIN) {
				prcmid = (*resp)->prcm_id;
				prcm_get_power_domain_state(prcmid, &state);
				if (state == PRCM_ON) {
#ifdef CONFIG_OMAP34XX_OFFMODE
					if (prcmid == DOM_PER)
						state = PRCM_ON;
					else
						state = PRCM_OFF;
#else
					if ((prcmid == DOM_IVA2) ||
							(prcmid == DOM_SGX))
						state = PRCM_OFF;
					else if (prcmid == DOM_NEON)
						state = PRCM_ON;
					else
						state = PRCM_RET;
#endif
					prcm_force_power_domain_state
						((*resp)->prcm_id, state);
				}
			}
		}
	}
	return 0;
}

#ifdef CONFIG_AUTO_POWER_DOMAIN_CTRL
late_initcall(turn_power_domains_off);
#endif

#endif /* #if 0 */
