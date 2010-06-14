/*
 * linux/arch/arm/mach-omap3pe/prcm_slpwk.c
 *
 * Copyright (C) 2008 Palm, Inc.
 *
 * Based on OMAP 34xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <asm/arch/prcm.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/arch/power_companion.h>

#include "prcm-regs.h"

/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

/*======================== DEVICE IDLE STATE =================================*/
/*============================================================================*/
/*= This function returns the specified module's idle status. It reads the   =*/
/*= CM_IDLEST_<DOMAIN> register to determine whether the device is accessible=*/
/*= or not. The result returned is PRCM_TRUE if the device is accessible     =*/
/*= else PRCM_FALSE. The function returns PRCM_FAIL if the parameters are    =*/
/*= not valid.                                                               =*/
/*============================================================================*/

int prcm_is_device_accessible(u32 deviceid, u8 *result)
{
	u32 domain;
	u32 omap;
	u32 valid;
	u32 enbit;
	volatile u32 *addr;

	omap   = OMAP(deviceid);
	enbit  = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430 | AT_3430_ES2)))
		return PRCM_FAIL;

	addr  = get_addr(domain, REG_IDLEST);
	valid = get_val_bits(domain, REG_IDLEST);

	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;

	if (!(*addr & (1 << enbit))) {
		*result = PRCM_TRUE;
	} else {
		*result = PRCM_FALSE;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== WAKEUP EVENT CONTROL ==============================*/
/*============================================================================*/
/*= Some modules have the option to wakeup the domain which is labeled a     =*/
/*= wakeup event.  This function will enable or disable the wake up event    =*/
/*= for the specified module.  If enabled, it allows the module to generate  =*/
/*= a wake up event.  If disabled, it masks the wake up event from the       =*/
/*= specified module.  This register will modify the PM_WKEN_<DOMAIN>        =*/
/*= register.  This function will return PRCM_FAIL if the parameters passed  =*/
/*= are invalid, otherwise it will return PRCM_PASS.  Valid parameters for   =*/
/*= control are PRCM_ENABLE and PRCM_DISABLE.                                =*/
/*============================================================================*/

//### Might consider using this function at some point...
//### or just remove it...
int prcm_wakeup_event_control(u32 deviceid, u8 control)
{
	u32 domain, omap, valid, enbit;
	volatile u32 *addr;

	omap = OMAP(deviceid);
	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430 | AT_3430_ES2)))
		return PRCM_FAIL;

	addr = get_addr(domain, REG_WKEN);
	valid = get_val_bits(domain, REG_WKEN);

	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;

	if (control == PRCM_ENABLE)
		*addr |= (1 << enbit);
	else if (control == PRCM_DISABLE)
		*addr &= ~(1 << enbit);

	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET WAKE UP DEPENDENCY  ===========================*/
/*============================================================================*/
/*= This function sets the wake up dependency for a specified power          =*/
/*= domain by accessing the wake up dependendcy register.                    =*/
/*============================================================================*/

int prcm_set_wkup_dependency(u32 domainid, u32 wkup_dep)
{
	switch (domainid) {
	case DOM_IVA2:
		PM_WKDEP_IVA2 |= wkup_dep;
		break;
	case DOM_MPU:
		PM_WKDEP_MPU |= wkup_dep;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case DOM_SGX:
		PM_WKDEP_SGX |= wkup_dep;
		break;
#endif
	case DOM_USBHOST:
		PM_WKDEP_USBHOST |= wkup_dep;
		break;
	case DOM_DSS:
		PM_WKDEP_DSS |= wkup_dep;
		break;
	case DOM_CAM:
		PM_WKDEP_CAM |= wkup_dep;
		break;
	case DOM_PER:
		PM_WKDEP_PER |= wkup_dep;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case DOM_NEON:
		PM_WKDEP_NEON |= wkup_dep;
		break;
#endif
	default:
		printk(KERN_INFO "Wake up dependency does not exist for "
		       "the domain %d\n", domainid);
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CLEAR WAKE UP DEPENDENCY  =========================*/
/*============================================================================*/
/*= This function clears the wake up dependency for a specified power        =*/
/*= domain by accessing the wake up dependendcy register.                    =*/
/*============================================================================*/

int prcm_clear_wkup_dependency(u32 domainid, u32 wkup_dep)
{
	switch (domainid) {
	case DOM_IVA2:
		PM_WKDEP_IVA2 &= ~wkup_dep;
		break;
	case DOM_MPU:
		PM_WKDEP_MPU &= ~wkup_dep;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case DOM_SGX:
		PM_WKDEP_SGX &= ~wkup_dep;
		break;
#endif
	case DOM_USBHOST:
		PM_WKDEP_USBHOST &= ~wkup_dep;
		break;
	case DOM_DSS:
		PM_WKDEP_DSS &= ~wkup_dep;
		break;
	case DOM_CAM:
		PM_WKDEP_CAM &= ~wkup_dep;
		break;
	case DOM_PER:
		PM_WKDEP_PER &= ~wkup_dep;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case DOM_NEON:
		PM_WKDEP_NEON &= ~wkup_dep;
		break;
#endif
	default:
		printk(KERN_INFO "Cannot clear Wake up dependency,"
		       "does not exist for the domain %d\n", domainid);
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET SLEEP DEPENDENCY  =============================*/
/*============================================================================*/
/*= This function sets the sleep dependency for a specified power            =*/
/*= domain by accessing the sleep dependency register.                       =*/
/*============================================================================*/

int prcm_set_sleep_dependency(u32 domainid, u32 sleep_dep)
{
	switch (domainid) {
#ifndef CONFIG_ARCH_OMAP3410
	case DOM_SGX:
		CM_SLEEPDEP_SGX |= sleep_dep;
		break;
#endif
	case DOM_USBHOST:
		CM_SLEEPDEP_USBHOST |= sleep_dep;
		break;
	case DOM_DSS:
		CM_SLEEPDEP_DSS |= sleep_dep;
		break;
	case DOM_CAM:
		CM_SLEEPDEP_CAM |= sleep_dep;
		break;
	case DOM_PER:
		CM_SLEEPDEP_PER |= sleep_dep;
		break;
	default:
		printk(KERN_INFO "Sleep dependency does not exist for"
		       "the domain %d\n", domainid);
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CLEAR SLEEP DEPENDENCY  ===========================*/
/*============================================================================*/
/*= This function clears the sleep dependency for a specified power          =*/
/*= domain by accessing the sleep dependency register.                       =*/
/*============================================================================*/

int prcm_clear_sleep_dependency(u32 domainid, u32 sleep_dep)
{
	switch (domainid) {
#ifndef CONFIG_ARCH_OMAP3410
	case DOM_SGX:
		CM_SLEEPDEP_SGX &= ~sleep_dep;
		break;
#endif
	case DOM_USBHOST:
		CM_SLEEPDEP_USBHOST &= ~sleep_dep;
		break;
	case DOM_DSS:
		CM_SLEEPDEP_DSS &= ~sleep_dep;
		break;
	case DOM_CAM:
		CM_SLEEPDEP_CAM &= ~sleep_dep;
		break;
	case DOM_PER:
		CM_SLEEPDEP_PER &= ~sleep_dep;
		break;
	default:
		printk(KERN_INFO "Cannot clear Sleep dependency, does"
		       " not exist for this domain %d\n", domainid);
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}
