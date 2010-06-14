/*
 * linux/arch/arm/mach-omap2/prcm_regs.c
 *
 * Copyright (C) 2008 Palm, Inc.
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

#include <linux/kernel.h>
#include <asm/arch/prcm.h>

#include "prcm-regs.h"

/******************************************************************************
 *
 * Domain register info table
 *
 ******************************************************************************/

struct domain_registers prcm_domain_register[PRCM_NUM_DOMAINS] = {
	{ { /* IVA2 domain */
		{0x1,		&CM_FCLKEN_IVA2},
		{0,		0},
		{0x1,		&CM_IDLEST_IVA2},
		{0,		0},
		{0,		0},
		{0,		0},
		{0x3,		&CM_CLKSTCTRL_IVA2},
		{0x1,		&CM_CLKSTST_IVA2},
		{0xFF0F0F,	&PM_PWSTCTRL_IVA2},
		{0x100FF7,	&PM_PWSTST_IVA2},
		{0xFF7, 	&PM_PREPWSTST_IVA2},
		{0,		0},
		{0x3F0F,	&RM_RSTST_IVA2},
	} },
	{ { /* MPU domain */
		{0,		0},
		{0,		0},
		{0x1,		&CM_IDLEST_MPU},
		{0,		0},
		{0,		0},
		{0,		0},
		{0x3,		&CM_CLKSTCTRL_MPU},
		{0x1,		&CM_CLKSTST_MPU},
		{0x3010F,	&PM_PWSTCTRL_MPU},
		{0x1000C7,	&PM_PWSTST_MPU},
		{0xC7,		&PM_PREPWSTST_MPU},
		{0,		0},
		{0x80F,		&RM_RSTST_MPU},
	} },
	{ { /* CORE1 */
		{0x43FFFE01,	&CM_FCLKEN1_CORE},
		{0x7FFFFED3, 	&CM_ICLKEN1_CORE},
		{0x7FFFFFF7, 	&CM_IDLEST1_CORE},
		{0x7FFFFED1,	&CM_AUTOIDLE1_CORE},
		{0x433FFE10,	&PM_WKEN1_CORE},
		{0x433FFE10,	&PM_WKST1_CORE},
		{0x0F,		&CM_CLKSTCTRL_CORE},
		{0x3,		&CM_CLKSTST_CORE},
		{0xF031F,	&PM_PWSTCTRL_CORE},
		{0x1000F7,	&PM_PWSTST_CORE},
		{0xF7,		&PM_PREPWSTST_CORE},
		{0xC0,		&CM_CLKSEL_CORE},
		{0x7,		&RM_RSTST_CORE},
	} },
	{ { /* CORE2 */
		{0,		0},
		{0x1F,		&CM_ICLKEN2_CORE},
		{0x1F,		&CM_IDLEST2_CORE},
		{0x1F,		&CM_AUTOIDLE2_CORE},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
	} },
	{ { /* SGX */
		{0x2,		&CM_FCLKEN_SGX},
		{0x1,		&CM_ICLKEN_SGX},
		{0x1,		&CM_IDLEST_SGX},
		{0,		0},
		{0,		0},
		{0,		0},
		{0x3,		&CM_CLKSTCTRL_SGX},
		{0x1,		&CM_CLKSTST_SGX},
		{0x30107,	&PM_PWSTCTRL_SGX},
		{0x100003,	&PM_PWSTST_SGX},
		{0x3,		&PM_PREPWSTST_SGX},
		{0,		0},
		{0xF,		&RM_RSTST_SGX},
	} },
	{ { /* WKUP */
		{0x2E9,		&CM_FCLKEN_WKUP},
		{0x23F,		&CM_ICLKEN_WKUP},
		{0x2FF,		&CM_IDLEST_WKUP},
		{0x23F,		&CM_AUTOIDLE_WKUP},
		{0x3CB,		&PM_WKEN_WKUP},
		{0x3CB,		&PM_WKST_WKUP},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0x7F,		&CM_CLKSEL_WKUP},
		{0,		0},
	} },
	{ { /* DSS */
		{0x7,		&CM_FCLKEN_DSS},
		{0x1,		&CM_ICLKEN_DSS},
		{0x3,		&CM_IDLEST_DSS},
		{0x1,		&CM_AUTOIDLE_DSS},
		{0x1,		&PM_WKEN_DSS},
		{0,		0},
		{0x3,		&CM_CLKSTCTRL_DSS},
		{0x1,		&CM_CLKSTST_DSS},
		{0x30107,	&PM_PWSTCTRL_DSS},
		{0x100003,	&PM_PWSTST_DSS},
		{0x3,		&PM_PREPWSTST_DSS},
		{0,		0},
		{0xF,		&RM_RSTST_DSS},
	} },
	{ { /* CAM */
		{0x3,		&CM_FCLKEN_CAM},
		{0x1,		&CM_ICLKEN_CAM},
		{0x1,		&CM_IDLEST_CAM},
		{0x1,		&CM_AUTOIDLE_CAM},
		{0,		0},
		{0,		0},
		{0x3,		&CM_CLKSTCTRL_CAM},
		{0x1,		&CM_CLKSTST_CAM},
		{0x30107,	&PM_PWSTCTRL_CAM},
		{0x100003,	&PM_PWSTST_CAM},
		{0x3,		&PM_PREPWSTST_CAM},
		{0,		0},
		{0xF,		&RM_RSTST_CAM},
	} },
	{ { /* PER */
		{0x3FFFF,	&CM_FCLKEN_PER},
		{0x3FFFF,	&CM_ICLKEN_PER},
		{0x3FFFF,	&CM_IDLEST_PER},
		{0x3FFFF,	&CM_AUTOIDLE_PER},
		{0x3EFFF,	&PM_WKEN_PER},
		{0x3EFFF,	&PM_WKST_PER},
		{0x3,		&CM_CLKSTCTRL_PER},
		{0x1,		&CM_CLKSTST_PER},
		{0x30107,	&PM_PWSTCTRL_PER},
		{0x100003,	&PM_PWSTST_PER},
		{0x3,		&PM_PREPWSTST_PER},
		{0xFF,		&CM_CLKSEL_PER},
		{0xF,		&RM_RSTST_PER},
	} },
	{ { /* EMU */
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0x3,		&CM_CLKSTCTRL_EMU},
		{0x1,		&CM_CLKSTST_EMU},
		{0,		0},
		{0x100003,	&PM_PWSTST_EMU},
		{0,		0},
		{0,		0},
		{0x7,		&RM_RSTST_EMU},
	} },
	{ { /* NEON */
		{0,		0},
		{0,		0},
		{0x1,		&CM_IDLEST_NEON},
		{0,		0},
		{0,		0},
		{0,		0},
		{0x3,		&CM_CLKSTCTRL_NEON},
		{0,		0},
		{0x7,		&PM_PWSTCTRL_NEON},
		{0x100003,	&PM_PWSTST_NEON},
		{0x3,		&PM_PREPWSTST_NEON},
		{0,		0},
		{0xF,		&RM_RSTST_NEON},
	} },
	{ { /* CORE3 */
		{0x7,		&CM_FCLKEN3_CORE},
		{0x4,		&CM_ICLKEN3_CORE},
		{0x5,		&CM_IDLEST3_CORE},
		{0x4,		&CM_AUTOIDLE3_CORE},
		{0x4,		&PM_WKEN3_CORE},
		{0x4,		&PM_WKST3_CORE},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
	} },
	{ { /* USBHOST */
		{0x3,		&CM_FCLKEN_USBHOST},
		{0x1,		&CM_ICLKEN_USBHOST},
		{0x3,		&CM_IDLEST_USBHOST},
		{0x1,		&CM_AUTOIDLE_USBHOST},
		{0x1,		&PM_WKEN_USBHOST},
		{0x1,		&PM_WKST_USBHOST},
		{0x3,		&CM_CLKSTCTRL_USBHOST},
		{0x1,		&CM_CLKSTST_USBHOST},
		{0x30117,	&PM_PWSTCTRL_USBHOST},
		{0x100003, 	&PM_PWSTST_USBHOST},
		{0x3,		&PM_PREPWSTST_USBHOST},
		{0,		0},
		{0xF,		&RM_RSTST_USBHOST},
	} },
};


/******************************************************************************
 *
 * DUMP PRCM registers (used for sleep/wake debugging)
 *
 ******************************************************************************/

#ifdef DEBUG_SUSPEND
#define preg(reg) \
	printk("%25s\t(0x%p):\t0x%08x\n", #reg, &reg, reg);

void prcm_debug(char *desc)
{
	printk("%s:\n", desc);

	preg(CM_FCLKEN_IVA2);
	preg(CM_CLKEN_PLL_IVA2);
	preg(CM_IDLEST_IVA2);
	preg(CM_IDLEST_PLL_IVA2);
	preg(CM_AUTOIDLE_PLL_IVA2);
	preg(CM_CLKSEL1_PLL_IVA2);
	preg(CM_CLKSEL2_PLL_IVA2);
	preg(CM_CLKSTCTRL_IVA2);
	preg(CM_CLKSTST_IVA2);

	preg(CM_CLKEN_PLL_MPU);
	preg(CM_IDLEST_MPU);
	preg(CM_IDLEST_PLL_MPU);
	preg(CM_AUTOIDLE_PLL_MPU);
	preg(CM_CLKSEL1_PLL_MPU);
	preg(CM_CLKSEL2_PLL_MPU);
	preg(CM_CLKSTCTRL_MPU);
	preg(CM_CLKSTST_MPU);

	preg(CM_FCLKEN1_CORE);
	preg(CM_ICLKEN1_CORE);
	preg(CM_ICLKEN2_CORE);
	preg(CM_IDLEST1_CORE);
	preg(CM_IDLEST2_CORE);
	preg(CM_AUTOIDLE1_CORE);
	preg(CM_AUTOIDLE2_CORE);
	preg(CM_CLKSEL_CORE);
	preg(CM_CLKSTCTRL_CORE);
	preg(CM_CLKSTST_CORE);

#ifndef CONFIG_ARCH_OMAP3410
	preg(CM_FCLKEN_SGX);
	preg(CM_ICLKEN_SGX);
	preg(CM_IDLEST_SGX);
	preg(CM_CLKSEL_SGX);
	preg(CM_SLEEPDEP_SGX);
	preg(CM_CLKSTCTRL_SGX);
	preg(CM_CLKSTST_SGX);
#endif

	preg(CM_FCLKEN_WKUP);
	preg(CM_ICLKEN_WKUP);
	preg(CM_IDLEST_WKUP);
	preg(CM_AUTOIDLE_WKUP);
	preg(CM_CLKSEL_WKUP);

	preg(CM_CLKEN_PLL);
	preg(CM_IDLEST_CKGEN);
	preg(CM_AUTOIDLE_PLL);
	preg(CM_CLKSEL1_PLL);
	preg(CM_CLKSEL2_PLL);
	preg(CM_CLKSEL3_PLL);
	preg(CM_CLKOUT_CTRL);

	preg(CM_FCLKEN_DSS);
	preg(CM_ICLKEN_DSS);
	preg(CM_IDLEST_DSS);
	preg(CM_AUTOIDLE_DSS);
	preg(CM_CLKSEL_DSS);
	preg(CM_SLEEPDEP_DSS);
	preg(CM_CLKSTCTRL_DSS);
	preg(CM_CLKSTST_DSS);

	preg(CM_FCLKEN_CAM);
	preg(CM_ICLKEN_CAM);
	preg(CM_IDLEST_CAM);
	preg(CM_AUTOIDLE_CAM);
	preg(CM_CLKSEL_CAM);
	preg(CM_SLEEPDEP_CAM);
	preg(CM_CLKSTCTRL_CAM);
	preg(CM_CLKSTST_CAM);

	preg(CM_FCLKEN_PER);
	preg(CM_ICLKEN_PER);
	preg(CM_IDLEST_PER);
	preg(CM_AUTOIDLE_PER);
	preg(CM_CLKSEL_PER);
	preg(CM_SLEEPDEP_PER);
	preg(CM_CLKSTCTRL_PER);
	preg(CM_CLKSTST_PER);

	preg(CM_POLCTRL);

#ifndef CONFIG_ARCH_OMAP3410
	preg(CM_IDLEST_NEON);
	preg(CM_CLKSTCTRL_NEON);
#endif
	preg(RM_RSTCTRL_IVA2);
	preg(RM_RSTST_IVA2);
	preg(PM_WKDEP_IVA2);
	preg(PM_PWSTCTRL_IVA2);
	preg(PM_PWSTST_IVA2);
	preg(PM_PREPWSTST_IVA2);
	preg(PRM_IRQSTATUS_IVA2);
	preg(PRM_IRQENABLE_IVA2);

	preg(PRM_REVISION);
	preg(PRM_SYSCONFIG);
	preg(PRM_IRQSTATUS_MPU);
	preg(PRM_IRQENABLE_MPU);

	preg(RM_RSTST_MPU);
	preg(PM_WKDEP_MPU);
	preg(PM_PWSTCTRL_MPU);
	preg(PM_PWSTST_MPU);
	preg(PM_PREPWSTST_MPU);

	preg(RM_RSTCTRL_CORE);
	preg(RM_RSTST_CORE);
	preg(PM_WKEN1_CORE);
	preg(PM_MPUGRPSEL1_CORE);
	preg(PM_IVA2GRPSEL1_CORE);
	preg(PM_WKST1_CORE);
	preg(PM_PWSTCTRL_CORE);
	preg(PM_PWSTST_CORE);
	preg(PM_PREPWSTST_CORE);

#ifndef CONFIG_ARCH_OMAP3410
	preg(RM_RSTST_SGX);
	preg(PM_WKDEP_SGX);
	preg(PM_PWSTCTRL_SGX);
	preg(PM_PWSTST_SGX);
	preg(PM_PREPWSTST_SGX);
#endif

	preg(PM_WKEN_WKUP);
	preg(PM_MPUGRPSEL_WKUP);
	preg(PM_IVA2GRPSEL_WKUP);
	preg(PM_WKST_WKUP);

	preg(PRM_CLKSEL);
	preg(PRM_CLKOUT_CTRL);

	preg(RM_RSTST_DSS);
	preg(PM_WKEN_DSS);
	preg(PM_WKDEP_DSS);
	preg(PM_PWSTCTRL_DSS);
	preg(PM_PWSTST_DSS);
	preg(PM_PREPWSTST_DSS);

	preg(RM_RSTST_CAM);
	preg(PM_WKDEP_CAM);
	preg(PM_PWSTCTRL_CAM);
	preg(PM_PWSTST_CAM);
	preg(PM_PREPWSTST_CAM);

	preg(RM_RSTST_PER);
	preg(PM_WKEN_PER);
	preg(PM_MPUGRPSEL_PER);
	preg(PM_IVA2GRPSEL_PER);
	preg(PM_WKST_PER);
	preg(PM_WKDEP_PER);
	preg(PM_PWSTCTRL_PER);
	preg(PM_PWSTST_PER);
	preg(PM_PREPWSTST_PER);

	preg(PRM_RSTCTRL);
	preg(PRM_RSTTIME);
	preg(PRM_RSTST);
	preg(PRM_CLKSETUP);
}
#endif /* DEBUG_SUSPEND */
