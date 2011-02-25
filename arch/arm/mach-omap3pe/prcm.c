/*
 * linux/arch/arm/mach-omap3pe/prcm.c
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

/* #define DEBUG_PRCM 1 */
#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...) \
		printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

#ifndef CONFIG_PM
#define omap_sram_idle() \
	{	\
		__asm__ __volatile__ ("wfi");	\
	}
#endif

/******************************************************************************/

extern void omap3_clk_prepare_for_reboot(void);

/******************************************************************************
 *
 * PRCM init
 *
 * This function is called by pm_init at boot time.
 *
 ******************************************************************************/

int __init prcm_init(void)
{
	int rc;
#ifdef CONFIG_ARCH_OMAP3410
	volatile u32 *addr;
	u32 new_val, valid;
#endif

#ifdef CONFIG_MACH_SIRLOIN_3630
	/*  Errata 1.102. RTA feature is not supported */
	CONTROL_MEM_RTA_CTRL = 0;
#endif

	/* Disable Modem WKUP */
//	PM_WKEN1_CORE       &= ~(1<<31);
	
	/* Disable D2D  */
	CM_ICLKEN1_CORE     &= ~(1<<3);
	CM_CLKSTCTRL_CORE   |=  (3<<4);

	/* Configure polarity:
	 *   SYS_OFF_MODE   - active low
	 *   SYS_CLKREQ     - active high
	 */
	PRM_POLCTRL &= ~PRM_POLCTRL_OFFMODE_POL;
	PRM_POLCTRL |= PRM_POLCTRL_CLKREQ_POL;


	/* Set up pad according to David Derreck's suggestion. */
	CONTROL_PADCONF_SAD2D_IDLEACK = (CONTROL_PADCONF_SAD2D_IDLEACK & 0xFFFF0000) |
						IO_PAD_INPUTENABLE    |
						IO_PAD_PULLUDENABLE   |
						IO_PAD_PULLTYPESELECT |
						IO_PAD_MUXMODE0;

	CONTROL_PADCONF_SAD2D_MSTDBY = (CONTROL_PADCONF_SAD2D_MSTDBY & 0xFFFF0000) |
						IO_PAD_INPUTENABLE    |
						IO_PAD_PULLUDENABLE   |
						IO_PAD_PULLTYPESELECT |
						IO_PAD_MUXMODE0;

	CONTROL_PADCONF_CHASSIS_SWAK = (CONTROL_PADCONF_CHASSIS_SWAK & 0xFFFF0000) |
						IO_PAD_INPUTENABLE    |
						IO_PAD_PULLUDENABLE   |
						0 | /* pulldown */
						IO_PAD_MUXMODE0;

	udelay(100);

	PM_WKST1_CORE = PM_WKST1_CORE;

	/* Set up SMARTIDLE and AUTOIDLE for control module. */
	CONTROL_SYSCONFIG = 0x11;

	/* Initiate save of padconf registers */
	CONTROL_PADCONF_OFF |= 0x02;
	rc = WAIT_UNTIL(CONTROL_GENERAL_PURPOSE_STATUS & 0x01, 1000);
	if (rc < 0) {
		printk(KERN_WARNING "Timeout saving pad configuration in %s\n",
				__FUNCTION__);
	}

	/* Clear sleep and wakeup dependencies for all domains */
	CM_SLEEPDEP_DSS = 0;
	PM_WKDEP_IVA2   = 0;
	PM_WKDEP_MPU    = 0;
	PM_WKDEP_DSS    = 0;
	PM_WKDEP_NEON   = 0;
	CM_SLEEPDEP_CAM = 0;
	PM_WKDEP_CAM    = 0;

	/* Clear groupselect registers. Required bits will be set as needed.
	 */
	PM_MPUGRPSEL_WKUP	= 0x0;
	PM_MPUGRPSEL1_CORE	= 0x0;
	PM_MPUGRPSEL3_CORE	= 0x0;
	PM_MPUGRPSEL_PER	= 0x0;
	PM_MPUGRPSEL_USBHOST	= 0x0;

	PM_IVA2GRPSEL_WKUP	= 0x0;
	PM_IVA2GRPSEL1_CORE	= 0x0;
	PM_IVA2GRPSEL3_CORE	= 0x0;
	PM_IVA2GRPSEL_PER	= 0x0;
	PM_IVA2GRPSEL_USBHOST	= 0x0;
	

#ifdef CONFIG_HW_SUP_TRANS
	if (prcm_set_wkup_dependency(DOM_PER,
			PRCM_WKDEP_EN_MPU) != PRCM_PASS) {
		printk(KERN_INFO "Domain %d : wakeup dependency"
					" could not be set\n", DOM_PER);
		return -1;
	}
	if (prcm_set_sleep_dependency(DOM_PER,
			PRCM_SLEEPDEP_EN_MPU) != PRCM_PASS) {
		printk(KERN_INFO "Domain %d : sleep dependency"
					" could not be set\n", DOM_PER);
		return -1;
	}
#else
	CM_SLEEPDEP_PER     = 0;
	PM_WKDEP_PER        = 0;
#endif
	CM_SLEEPDEP_SGX     = 0;
	CM_SLEEPDEP_USBHOST = 0;
	PM_WKDEP_SGX        = 0;
	PM_WKDEP_USBHOST    = 0;

	/* Enable interface clock autoidle for all modules, except:
	 *  - Disable Autoidle for HDQ, as it is not OCP compliant.
	 *  - Disable Autoidle for GPT1 - errata 1.4.
	 */
	CM_AUTOIDLE1_CORE   = get_val_bits(DOM_CORE1,   REG_AUTOIDLE) & ~(1<<22);
	CM_AUTOIDLE2_CORE   = get_val_bits(DOM_CORE2,   REG_AUTOIDLE);
	CM_AUTOIDLE_WKUP    = get_val_bits(DOM_WKUP,    REG_AUTOIDLE) & ~(0x1);
	CM_AUTOIDLE_DSS     = get_val_bits(DOM_DSS,     REG_AUTOIDLE);
	CM_AUTOIDLE_CAM     = get_val_bits(DOM_CAM,     REG_AUTOIDLE);
	CM_AUTOIDLE_PER     = get_val_bits(DOM_PER,     REG_AUTOIDLE);
	CM_AUTOIDLE3_CORE   = get_val_bits(DOM_CORE3,   REG_AUTOIDLE);
	CM_AUTOIDLE_USBHOST = get_val_bits(DOM_USBHOST, REG_AUTOIDLE);

	CM_AUTOIDLE2_PLL     = 0x1; /* DPLL5 autoidle */
	CM_AUTOIDLE_PLL      = 0x9; /* DPLL3, DPLL4 autoidle */
	CM_AUTOIDLE_PLL_MPU  = LOW_POWER_STOP;
	CM_AUTOIDLE_PLL_IVA2 = LOW_POWER_STOP;

#ifndef CONFIG_DISABLE_EMUDOMAIN_CONTROL
	CM_CLKSTCTRL_EMU = 0x3;
#endif

	/* Enable AutoIdle for modules in the following domains.
	 */
	prcm_set_domain_power_configuration(DOM_DSS, PRCM_SIDLEMODE_DONTCARE,
					PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);
	prcm_set_domain_power_configuration(DOM_PER, PRCM_SIDLEMODE_DONTCARE,
					PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);

	/* Enable SmartStandby, SmartIdle and AutoIdle for modules in the
	 * following domains.
	 */
	prcm_set_domain_power_configuration(DOM_CORE1, PRCM_SMART_IDLE,
					PRCM_SMART_STANDBY, PRCM_TRUE);
	prcm_set_domain_power_configuration(DOM_CAM, PRCM_SMART_IDLE,
					PRCM_SMART_STANDBY, PRCM_TRUE);
	prcm_set_domain_power_configuration(DOM_WKUP, PRCM_SMART_IDLE,
					PRCM_SMART_STANDBY, PRCM_TRUE);


	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0) ||
	    is_sil_rev_equal_to(OMAP3430_REV_ES2_0) ||
	    is_sil_rev_equal_to(OMAP3430_REV_ES2_1)) {
		/* Due to Errata 1.27, IVA2 will not go to ret/off during
		 * bootup. We need to manually boot it to idle mode. This is
		 * only required during bootup and not for subsequent
		 * transitions.
		 * Configure IVA to boot in idle mode.
		 */
		CONTROL_IVA2_BOOTMOD = 0x1;
		/* Clear reset of IVA2*/
		RM_RSTCTRL_IVA2 = 0x0;
		/* Clear reset status */
		RM_RSTST_IVA2 = 0xFFFFFFFF;
	}

	/* L3, L4 and D2D clock autogating */
	CM_CLKSTCTRL_CORE = (CLK_D2D_HW_SUP_ENABLE |
			     CLK_L4_HW_SUP_ENABLE |
			     CLK_L3_HW_SUP_ENABLE);

	/* Change DPLL2 divider to 4.
	 * When DPLL2 is going into bypass it is using CORE clk as input. We
	 * need to divide by 4 to avoid the DPLL running at CORE rate.
	 */
	CM_CLKSEL1_PLL_IVA2 = (CM_CLKSEL1_PLL_IVA2 & ~(0x7 << 19)) | (0x4 << 19);

	clear_domain_reset_status();

	if( PM_WKST1_CORE ) {
		panic("Unable to clear PM_WKST1_CORE=0x%08x\n", PM_WKST1_CORE );
	}
	
	return PRCM_PASS;
}

/* Clear out any status which may gate sleep */
void clear_domain_reset_status(void)
{
#ifndef CONFIG_ARCH_OMAP3410
	RM_RSTST_SGX     = RM_RSTST_SGX;
	RM_RSTST_NEON    = RM_RSTST_NEON;
#endif
	RM_RSTST_IVA2    = RM_RSTST_IVA2;
	RM_RSTST_MPU     = RM_RSTST_MPU;
	RM_RSTST_CORE    = RM_RSTST_CORE;
	RM_RSTST_DSS     = RM_RSTST_DSS;
	RM_RSTST_CAM     = RM_RSTST_CAM;
	RM_RSTST_PER     = RM_RSTST_PER;
	RM_RSTST_EMU     = RM_RSTST_EMU;
	RM_RSTST_USBHOST = RM_RSTST_USBHOST;

	PM_WKST1_CORE   = PM_WKST1_CORE;
	PM_WKST3_CORE   = PM_WKST3_CORE;
	PM_WKST_PER     = PM_WKST_PER;
	PM_WKST_WKUP    = PM_WKST_WKUP;
	PM_WKST_USBHOST = PM_WKST_USBHOST;

	PRM_RSTST          = PRM_RSTST;
	PRM_IRQSTATUS_IVA2 = PRM_IRQSTATUS_IVA2;
	PRM_IRQSTATUS_MPU  = PRM_IRQSTATUS_MPU;
}
