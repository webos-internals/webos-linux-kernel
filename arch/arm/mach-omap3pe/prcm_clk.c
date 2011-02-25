/*
 * linux/arch/arm/mach-omap3pe/prcm_clk.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP 34xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu/Rajendra Nayak/Pavan Chinnabhandar
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
#include "prcm_pwr.h"
#include "prcm_util.h"

/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

//#define DEBUG_PRCM

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

/******************************************************************************
 *
 * Local helper functions
 *
 ******************************************************************************/

/*============================================================================*/
/*======================== CHECK DEVICE STATUS ===============================*/
/*============================================================================*/
/*= This command checks if the specified module is in the desired state.The  =*/
/*= valid parameters for control are PRCM_ENABLE and PRCM_DISABLE.With       =*/
/*= PRCM_ENABLE the function returns PRCM TRUE if the device is accessible   =*/
/*= else PRCM_FAIL. With PRCM_DISABLE the function returns PRCM_TRUE if the  =*/
/*= device is not accessible, else PRCM_FAIL                                  */
/*============================================================================*/

static int check_device_status(u32 deviceid, u8 control)
{
	u8  curr_state;
	int ret;
	int retries;

	if ((control != PRCM_ENABLE) && (control != PRCM_DISABLE))
		return PRCM_FAIL;

	retries = 100;
	do {
		ret = prcm_is_device_accessible(deviceid, &curr_state);
		if (ret != PRCM_PASS)
			return ret;

		if (curr_state == control)
			return PRCM_PASS;

		udelay(5);
	} while (retries--);

	printk(KERN_INFO "Timeout in check_device_status(), deviceid %d, "
			"desired state %d, current state %d\n",
			deviceid, control, curr_state);

	return PRCM_FAIL;
}

/*============================================================================*/
/*======================== CHECK ACCESSIBILITY ===============================*/
/*============================================================================*/
/*= The function checks whether a module is accessible, by checking for      =*/
/*= its FCLK and ICLK being enabled                                          =*/
/*============================================================================*/
/*============================================================================*/

static int check_accessibility(u32 deviceid, u8 clk_type)
{
	u32 valid = 0;
	u32 enbit;
	u32 type;
	u32 domain;
	volatile u32 *addr = 0;

	enbit  = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);
	type   = DEV_TYPE(deviceid);

	if (type == TARGET) {
		/* Skip devices without CM_IDLEST register bit support */
		addr  = get_addr(domain, REG_IDLEST);
		valid = get_val_bits(domain, REG_IDLEST);
		if (!(addr) || !(valid & (1 << enbit)))
			return PRCM_PASS;
		/* Check if FCLK/ICLK is absent or ICLK is ON if present. */
		if (clk_type == ICLK) {
			addr  = get_addr(domain, REG_FCLKEN);
			valid = get_val_bits(domain, REG_FCLKEN);
		} else if (clk_type == FCLK) {
			addr  = get_addr(domain, REG_ICLKEN);
			valid = get_val_bits(domain, REG_ICLKEN);
		}
		if (!(addr) || !(valid & (1 << enbit))
		    || (*addr & (1 << enbit)))
			/* FCLK/ICLK present and is ON */
			return check_device_status(deviceid, PRCM_ENABLE);
	} else {		/* type = INITIATOR or INITIATOR+TARGET
				 * IDLEST bit cannot be polled,
				 * it only specifies the
				 * standby status
				 * Check if the ICLK is present and ON */
		if (clk_type == FCLK) {
			addr  = get_addr(domain, REG_ICLKEN);
			valid = get_val_bits(domain, REG_ICLKEN);
		} else if (clk_type == ICLK) {
			addr  = get_addr(domain, REG_FCLKEN);
			valid = get_val_bits(domain, REG_FCLKEN);
		}
		if (!(addr) || !(valid & (1 << enbit))
		    || (*addr & (1 << enbit))) {
			/* FCLK/ICLK present and is ON
			 * Wait for sometime for the clocks to stabilize*/
			DPRINTK("Adding delay\n");
			udelay(100);
			return PRCM_PASS;
		}
	}
	return PRCM_PASS;
}

/******************************************************************************
 *
 * 
 * PUBLIC API
 *
 *
 ******************************************************************************/

/*============================================================================*/
/*======================== CLOCK CONTROL  ====================================*/
/*============================================================================*/
/*= This function will either enable or disable the fclk/iclk clock for the  =*/
/*= specified module.  This command will modify the CM_F/ICLKEN_<DOMAIN>/    =*/
/*= setting.  The valid parameters for control are PRCM_ENABLE and           =*/
/*= PRCM_DISABLE. This command will return PRCM_FAIL if the passed parameters=*/
/*= are not valid,otherwise it will return PRCM_PASS.If the parameter        =*/
/*= checkaccessibility is PRCM_TRUE, the function waits till the device is   =*/
/*= accessible before returning (provided both fclk/iclk clock are           =*/
/*= enabled).                                                                =*/
/*============================================================================*/

int prcm_clock_control(u32 deviceid, u8 clk_type, u8 control,
		       u8 checkaccessibility)
{
	u32 domain;
	u32 omap;
	u32 valid;
	u32 enbit;
	volatile u32 *addr;

	DPRINTK("Clock:%d Control:%d\n", clk_type, control);

	omap = OMAP(deviceid);
	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	switch (clk_type) {
	case ICLK:
		addr  = get_addr(domain, REG_ICLKEN);
		valid = get_val_bits(domain, REG_ICLKEN);
		break;
	case FCLK:
		addr  = get_addr(domain, REG_FCLKEN);
		valid = get_val_bits(domain, REG_FCLKEN);
		break;
	default:
		return PRCM_FAIL;
	}

	DPRINTK("Address before: %x\n", *addr);

	/* No functional/Interface Clk control for the device */
	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;

	if (control == PRCM_ENABLE)
		*addr |= (1 << enbit);
	else if (control == PRCM_DISABLE)
		*addr &= ~(1 << enbit);

	DPRINTK("Address after: %x\n", *addr);

	if (checkaccessibility)
		return check_accessibility(deviceid, clk_type);

	return PRCM_PASS;
}

/*============================================================================*/
/*======================== WAIT FOR CLOCK  ===================================*/
/*============================================================================*/
/*= Provide simple dead wait for access on clocks.                           =*/
/*============================================================================*/
void prcm_wait_for_clock(u32 deviceid)
{
	u8 ok;
	do {
		prcm_is_device_accessible(deviceid, &ok);
	} while (ok != PRCM_TRUE);
}

/*============================================================================*/
/*======================== INTERFACE CLOCK AUTOIDLE  =========================*/
/*============================================================================*/
/*= This command will either enable or disable the interface clock AutoIdle  =*/
/*= feature for the specified module.  This command will modify the          =*/
/*= CM_AUTOIDLE_<DOMAIN> setting.  The valid parameters for control are      =*/
/*= PRCM_ENABLE and PRCM_DISABLE.  This command will return PRCM_FAIL if the =*/
/*= passed parameters are not valid, otherwise it will return PRCM_PASS.     =*/
/*============================================================================*/

int prcm_interface_clock_autoidle(u32 deviceid, u8 control)
//### Is this function used?
{
	u32 domain;
	u32 omap;
	u32 valid;
	u32 enbit;
	volatile u32 *addr;

	omap = OMAP(deviceid);
	enbit = DEV_BIT_POS(deviceid);
	domain = DOMAIN_ID(deviceid);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	addr  = get_addr(domain, REG_AUTOIDLE);
	valid = get_val_bits(domain, REG_AUTOIDLE);

	if (!(addr) || !(valid & (1 << enbit)))
		return PRCM_FAIL;

	if (control == PRCM_ENABLE)
		*addr |= (1 << enbit);
	else if (control == PRCM_DISABLE)
		*addr &= ~(1 << enbit);

	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CLOCK DOMAIN STATE  ===============================*/
/*============================================================================*/
/*= This function returns the state of the clock domain. It reads the        =*/
/*= CM_CLKSTST_<DOMAIN> register and returns the result as PRCM_ENABLE if    =*/
/*= the clock domain is active, else returns PRCM_DISABLE  		     =*/
/*= a wake up event.  If disabled, it masks the wake up event from the       =*/
/*= specified module.  This register will modify the PM_WKEN_<DOMAIN>        =*/
/*= register.  This function will return PRCM_FAIL if the parameters passed  =*/
/*= are invalid, otherwise it will return PRCM_PASS.  Valid parameters for   =*/
/*= control are PRCM_ENABLE and PRCM_DISABLE.                                =*/
/*============================================================================*/

static int prcm_is_clock_domain_active(u32 domainid, u8 *result)
{
	u32 valid;
	volatile u32 *addr;

	/* Core domain check is not supported */
	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk(KERN_INFO "%s() for DOM_CORE1/DOM_CORE2 "
				"is not supported\n", __FUNCTION__);
		return PRCM_FAIL;
	}

	addr  = get_addr(domainid, REG_CLKSTST);
	if (!addr)
		return PRCM_FAIL;

	valid = get_val_bits(domainid, REG_CLKSTST);

	*result = (*addr & valid) ? PRCM_ENABLE : PRCM_DISABLE;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CLOCK DOMAIN STATUS ===============================*/
/*============================================================================*/
/* This function waits for the clock domain to transition to the desired state*/
/* It polls on the clock domain state and times out after a wait of ~500 micro*/
/* secs. It returns PRCM_PASS id the clock domain transitions to the desired  */
/* state within the timeout period, else return PRCM_FAIL		      */
/*============================================================================*/

static int prcm_check_clock_domain_status(u32 domainid, u8 desired_state)
{
	u8  curr_state;
	int ret;
	int retries;

	/* Core domain check is not supported */
	if ((domainid == DOM_CORE1) ||
	    (domainid == DOM_CORE2) ||
	    (domainid == DOM_MPU)) {
		printk(KERN_INFO "%s() is not supported for "
			"DOM_CORE1/DOM_CORE2/DOM_MPU\n", __FUNCTION__);
		return PRCM_FAIL;
	}

	FAIL_ON_INVALID_DOMAINID(domainid);

	retries = 500;
	do {
		ret = prcm_is_clock_domain_active(domainid, &curr_state);
		if (ret != PRCM_PASS)
			return ret;

		if (curr_state == desired_state)
			return PRCM_PASS;

		udelay(1);
	} while (retries--);

#if 0
	printk(KERN_INFO "Timeout in %s(), "
			"domainid %d, desired state %d, current state %d\n",
			__FUNCTION__, domainid, desired_state, curr_state);
#endif			

	return PRCM_FAIL;
}

/*============================================================================*/
/*======================== SET CLOCK DOMAIN STATE ============================*/
/*============================================================================*/
/* This function sets the clock domain state to the 'new_state' specified. In */
/* software supervised mode it checks if all the pre-conditions for clock     */
/* domain transition are met. If check_accessibility is set to PRCM_TRUE the  */
/* function also waits fo the clock domain transition to complete             */
/*============================================================================*/

int prcm_set_clock_domain_state(u32 domainid, u8 new_state, u8 check_state)
{
	volatile u32 *addr;
	u32 valid;

	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk(KERN_INFO "Currently prcm_set_clock_domain_state "
			"for the following domains "
			"(DOM_CORE1/DOM_CORE2)is not supported\n");
		return PRCM_FAIL;
	}

	addr  = get_addr(domainid, REG_CLKSTCTRL);
	valid = get_val_bits(domainid, REG_CLKSTCTRL);
	if (!addr) {
		printk(KERN_INFO "No ClkStCtrl for domain %d\n", domainid);
		return PRCM_FAIL;
	}

	/* It is not appropriate to pass check_state = TRUE for states
	 * PRCM_NO_AUTO and PRCM_HWSUP_AUTO.
	 * In case of PRCM_NO_AUTO, hardware control of clock domain is disabled
	 * In case of PRCM_HWSUP_AUTO, the clock domain will transition
	 * automatically when conditions are satisfied
	 */
	if (check_state == PRCM_TRUE) {
		if ((new_state == PRCM_NO_AUTO) ||
		    (new_state == PRCM_HWSUP_AUTO)) {
			printk(KERN_INFO "Cannot wait till clock domain goes"
			"to specified state when target state is: %d\n",
								new_state);
			return PRCM_FAIL;
		}
	}

	/* Check preconditions for SWSUP sleep if check_state = TRUE */
	if (new_state == PRCM_SWSUP_SLEEP) {
		if (domainid == DOM_MPU)
			/* There is no SWSUPERVISED sleep for MPU*/
			return PRCM_FAIL;

		if (check_state == PRCM_TRUE) {
			/* Check if the pre-conditions for the clock domain
			 * transition are met. Check for fclk/iclk to be
			 * disabled. */
			u32 fclk_mask;
			u32 iclk_mask;

			prcm_get_domain_functional_clocks(domainid, &fclk_mask);
			prcm_get_domain_interface_clocks(domainid, &iclk_mask);
			if (fclk_mask || iclk_mask) {
				printk(KERN_INFO "Pre condition for clock "
					"domain transition not met:\n"
					"  Clocks enabled in domain %d:\n"
					"  Fclk_mask: 0x%08x\n"
					"  Iclk_mask: 0x%08x\n",
					domainid, fclk_mask, iclk_mask);
				return PRCM_FAIL;
			}
#ifndef CONFIG_ARCH_OMAP3410
			if (domainid != DOM_NEON)
#endif
			{
				/* This check not needed for NEON Check if all
				 * Initiators are in standby, and all devices
				 * idle. */
				u32 init_mask;
				u32 dev_mask;

				prcm_get_initiators_not_standby(domainid,
								&init_mask);
				prcm_get_devices_not_idle(domainid, &dev_mask);
				if (init_mask || dev_mask) {
					printk(KERN_INFO "Pre condition for "
						"clock domain transition not"
						" met:\n"
						"  init_mask: 0x%08x\n"
						"  dev_mask:  0x%08x\n"
						"  Domain: %d\n",
						init_mask, dev_mask, domainid);
					return PRCM_FAIL;
				}
			}
		}
	}

	/* Program Clkstctrl register */
	*addr = (*addr & ~valid) | new_state;

	/* NEON has no CLKSTST register */
#ifndef CONFIG_ARCH_OMAP3410
	if ((check_state == PRCM_TRUE) && (domainid != DOM_NEON))
#else
	if (check_state == PRCM_TRUE)
#endif
	{
		/* Wait for the Clock domain to transition to the new state */
		if (new_state == PRCM_SWSUP_WKUP)
			return prcm_check_clock_domain_status(domainid,
							      PRCM_ENABLE);

		if (new_state == PRCM_SWSUP_SLEEP)
			return prcm_check_clock_domain_status(domainid,
							      PRCM_DISABLE);
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== GET DOMAIN INTERFACE CLKS  ========================*/
/*============================================================================*/
/*= This function returns the CM_ICLKEN_<domain> register value in result for=*/
/*= the specified domain. The function returns PRCM_FAIL if the ICLKEN       =*/
/*= register is not available for the specified domain, else returns         =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/

int prcm_get_domain_interface_clocks(u32 domainid, u32 *result)
{
	u32 valid;
	volatile u32 *addr;

	/* We need to pre-initialize result here as we might exit if the
	 * requested domain does not have an gate-able interface clock. This is
	 * not an error, it needs to be reported as "no interface clock
	 * enabled".
	 */
	*result = 0;

	FAIL_ON_INVALID_DOMAINID(domainid);

	addr  = get_addr(domainid, REG_ICLKEN);
	valid = get_val_bits(domainid, REG_ICLKEN);

	if (!addr)
		return PRCM_FAIL;

	*result = *addr & valid;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== GET DOMAIN FUNCTIONAL CLKS  =======================*/
/*============================================================================*/
/*= This function returns the CM_FCLKEN_<domain> register in the result for  =*/
/*= the specified domain. The function returns PRCM_FAIL if the FCLKEN       =*/
/* register is not available for the specified domain, else returns          =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/

int prcm_get_domain_functional_clocks(u32 domainid, u32 *result)
{
	u32 valid;
	volatile u32 *addr;

	/* We need to pre-initialize result here as we might exit if the
	 * requested domain does not have an gate-able interface clock. This is
	 * not an error, it needs to be reported as "no interface clock
	 * enabled".
	 */
	*result = 0;

	FAIL_ON_INVALID_DOMAINID(domainid);

	addr  = get_addr(domainid, REG_FCLKEN);
	valid = get_val_bits(domainid, REG_FCLKEN);

	if (!addr)
		return PRCM_FAIL;

	*result = *addr & valid;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET DOMAIN INTERFACE CLKS  ========================*/
/*============================================================================*/
/*= This function sets CM_ICLKEN_<domain> register for the specified domain  =*/
/*= with the mask specified in setmask. The function returns PRCM_FAIL if the=*/
/*= ICLKEN register is not available for the specified domain, else returns  =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/

int prcm_set_domain_interface_clocks(u32 domainid, u32 setmask)
{
	u32 valid;
	volatile u32 *addr;

	FAIL_ON_INVALID_DOMAINID(domainid);

	addr  = get_addr(domainid, REG_ICLKEN);
	valid = get_val_bits(domainid, REG_ICLKEN);

	if (!addr)
		return PRCM_FAIL;

	*addr = setmask & valid;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET DOMAIN FUNCTIONAL CLKS  =======================*/
/*============================================================================*/
/*= This function sets CM_FCLKEN_<domain> register for the specified domain  =*/
/*= with the mask specified in setmask. The function returns PRCM_FAIL if the=*/
/*=  FCLKEN register is not available for the specified domain, else returns =*/
/*= PRCM_PASS.                                                               =*/
/*============================================================================*/

int prcm_set_domain_functional_clocks(u32 domainid, u32 setmask)
{
	u32 valid;
	volatile u32 *addr;

	FAIL_ON_INVALID_DOMAINID(domainid);

	addr  = get_addr(domainid, REG_FCLKEN);
	valid = get_val_bits(domainid, REG_FCLKEN);

	if (!addr)
		return PRCM_FAIL;

	*addr = setmask & valid;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== GET OSC RATE  =====================================*/
/*============================================================================*/
/*= This function returns the Current System clock speed in KHz. Fetches     =*/
/*= values from PRM_CLKSEL[2:0]. Ranges from 12,13,19.2,26,38.4 MHz          =*/
/*============================================================================*/

int prcm_get_crystal_rate(void)
{
	u32 osc_clkspeed;

	osc_clkspeed = PRM_CLKSEL & 0x7;

	switch (osc_clkspeed) {
	case 0: return 12000;	/*12MHz*/
	case 1: return 13000;	/*13MHz*/
	case 2: return 19200;	/*19.2MHz*/
	case 3: return 26000;	/*26MHz*/
	case 4: return 38400;	/*38.4MHz*/
	case 5: return 16800;	/*16.8MHz*/
	}

	return 0;
}

/*============================================================================*/
/*======================== READ SYSTEM CLOCK   ===============================*/
/*============================================================================*/
/*= This function returns the Current System clock speed in KHz. Fetches     =*/
/*= values from PRM_CLKSEL[2:0]. Ranges from 12,13,19.2,26,38.4 MHz          =*/
/*============================================================================*/

int prcm_get_system_clock_speed(void)
{
	u32 osc_clkspeed;
	u32 sys_clkdiv;

	osc_clkspeed = prcm_get_crystal_rate();
	if (osc_clkspeed == 0)
		return PRCM_FAIL;

	sys_clkdiv = (PRM_CLKSRC_CTRL >> 6) & 0x3;
	return osc_clkspeed / sys_clkdiv;
}

// -wgr- /*============================================================================*/
// -wgr- /*======================== SELECT SYS CLK DIVIDER  ===========================*/
// -wgr- /*============================================================================*/
// -wgr- /*= This function sets the divider value for the system clock as specified   =*/
// -wgr- /*= in setting.                                                              =*/
// -wgr- /*============================================================================*/
// -wgr- 
// -wgr- int prcm_select_system_clock_divider(u32 setting)
// -wgr- {
// -wgr- 	u32 new_value = 0x00000000;
// -wgr- 
// -wgr- 	new_value = (PRM_CLKSRC_CTRL & SYSCLK_DIV_MASK);
// -wgr- 	new_value = new_value | (setting << 6);
// -wgr- 	PRM_CLKSRC_CTRL = new_value;
// -wgr- 
// -wgr- 	return PRCM_PASS;
// -wgr- }

/*============================================================================*/
/*======================== CTRL EXT OUTPUT CLK1 ==============================*/
/*= This command will either enable or disable the device output clock       =*/
/*= sys_clkout1.sys_clkout1 belongs to the WKUP power domain so is available =*/
/*= even in OFF mode, provided that the internal or the external oscillator  =*/
/*= is active. The register modified is the PRM_CLKOUT_CTRL register. This   =*/
/*= function will always return PRCM_PASS. Valid parameters  for control are =*/
/*= PRCM_ENABLE and PRCM_DISABLE.                                            =*/
/*============================================================================*/

int prcm_control_external_output_clock1(u32 control)
{
	u32 new_value = 0x00000000;

	new_value = (PRM_CLKOUT_CTRL & EXTCLK_OUTCTRL_MASK);
	new_value = new_value | (control << 7);
	PRM_CLKOUT_CTRL = new_value;

	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CTRL EXT OUTPUT CLK2 ==============================*/
/*= This command will either enable or disable the device output clock 	     =*/
/*= sys_clkout2. The register modified is the CM_CLKOUT_CTRL register. This  =*/
/*= function will always return PRCM_PASS.Valid parameters for control are   =*/
/*= PRCM_ENABLE and PRCM_DISABLE.                                            =*/
/*============================================================================*/

int prcm_control_external_output_clock2(u32 control)
{
	u32 new_value = 0x00000000;

	new_value = (CM_CLKOUT_CTRL & EXTCLK_OUTCTRL_MASK);
	new_value = new_value | (control << 7);
	CM_CLKOUT_CTRL = new_value;

	return PRCM_PASS;
}

/*============================================================================*/
/*======================== GET PROCESSOR SPEED ===============================*/
/*= This function will return the processor speed for the specified domain.  =*/
/*= The domain is identified using the domainid parameter.                   =*/
/*= The function calls prcm_get_dpll_rate with the correct DPLL id for the   =*/
/*= domain.                                                                  =*/
/*= The processor speed returned in processor_speed parameter.s in Khz       =*/
/*= The function returns PRCM_FAIL if a failure is returned from             =*/
/*= prcm_get_dpll_rate or an invalid domain id is passed.                    =*/
/*= Returns PRCM_PASS on success.                                            =*/
/*============================================================================*/

int prcm_get_processor_speed(u32 domainid, u32 *processor_speed)
{
	int ret = PRCM_PASS;
	switch (domainid) {
	case DOM_MPU:
		ret = prcm_get_dpll_rate(PRCM_DPLL1_M2X2_CLK, processor_speed);
		if (ret != PRCM_PASS)
			break;
		else
			/*There is a divider in mpu which makes the actual
			   processor speed half the dpll output */
			*processor_speed = *processor_speed / 2;
		break;
	case PRCM_IVA2:
		ret = prcm_get_dpll_rate(PRCM_DPLL2_M2X2_CLK, processor_speed);
		if (ret != PRCM_PASS)
			break;
		else
			/*There is a divider in iva which makes the actual
			   processor speed half the dpll output */
			*processor_speed = *processor_speed / 2;
		break;
	default:
		ret = PRCM_FAIL;
		break;
	}
	return ret;
}

