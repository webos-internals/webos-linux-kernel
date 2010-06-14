/*
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

#include <linux/kernel.h>
#include <asm/arch/prcm.h>

#include "prcm-regs.h"

/******************************************************************************
 *
 * Change state of MPU power domain
 *
 ******************************************************************************/

int prcm_set_mpu_domain_state(u32 mpu_dom_state)
{
	u32 id_type, value, ret;
	id_type = get_other_id_type(mpu_dom_state);
	if (!(id_type & ID_MPU_DOM_STATE))
		return PRCM_FAIL;

	if (mpu_dom_state > PRCM_MPU_OFF)
		return PRCM_FAIL;

	value = PM_PWSTCTRL_MPU & ~PWSTST_PWST_MASK;

	switch (mpu_dom_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
		value |= 0x3;
		RM_RSTST_MPU |= DOM_WKUP_RST;
		break;
	case PRCM_MPU_CSWR_L2RET:
	case PRCM_MPU_CSWR_L2OFF:
	case PRCM_MPU_OSWR_L2RET:
	case PRCM_MPU_OSWR_L2OFF:
		value |= 0x1;
		break;
	case PRCM_MPU_OFF:
		value |= 0x0;
		break;
	default:
		return PRCM_FAIL;
	}
	PM_PWSTCTRL_MPU = value;

	if ((mpu_dom_state == PRCM_MPU_OSWR_L2RET) ||
	    (mpu_dom_state == PRCM_MPU_OSWR_L2OFF))
		PM_PWSTCTRL_MPU &= ~(0x4);

	if ((mpu_dom_state == PRCM_MPU_CSWR_L2OFF) ||
	    (mpu_dom_state == PRCM_MPU_OSWR_L2OFF))
		PM_PWSTCTRL_MPU &= ~(0x100);

	if (mpu_dom_state != PRCM_MPU_ACTIVE)
		ret = prcm_set_clock_domain_state(
				DOM_MPU, PRCM_HWSUP_AUTO, PRCM_FALSE);
	else
		ret = prcm_set_clock_domain_state(
				DOM_MPU, PRCM_NO_AUTO, PRCM_FALSE);
	return ret;
}

/******************************************************************************
 *
 * Change state of CORE power domain
 *
 ******************************************************************************/

int prcm_set_core_domain_state(u32 core_dom_state)
{
	u32 id_type;
	u32 value;

	id_type = get_other_id_type(core_dom_state);
	if (!(id_type & ID_CORE_DOM_STATE))
		return PRCM_FAIL;

	value = PM_PWSTCTRL_CORE & ~PWSTST_PWST_MASK;

	switch (core_dom_state) {
	case PRCM_CORE_ACTIVE:
	case PRCM_CORE_INACTIVE:
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_PWRSTATEBIT2;
		RM_RSTST_CORE |= DOM_WKUP_RST;
		RM_RSTST_IVA2 |= COREDOM_WKUP_RST;
		RM_RSTST_MPU  |= COREDOM_WKUP_RST;
		RM_RSTST_SGX  |= COREDOM_WKUP_RST;
		RM_RSTST_DSS  |= COREDOM_WKUP_RST;
		RM_RSTST_CAM  |= COREDOM_WKUP_RST;
		RM_RSTST_PER  |= COREDOM_WKUP_RST;
		break;

	case PRCM_CORE_CSWR_MEMRET:
		value |= (PRCM_CORE_MEMBIT | PRCM_CORE_LOGICBIT | 
					     PRCM_CORE_PWRSTATEBIT1);
		value &= ~PRCM_CORE_PWRSTATEBIT2;
		PRM_VOLTCTRL |= PRM_VOLTCTRL_AUTO_RET;
#ifdef CONFIG_SYSOFFMODE
		PRM_VOLTCTRL &= ~PRM_VOLTCTRL_SEL_OFF;
#endif
		break;

	case PRCM_CORE_CSWR_MEM1OFF:
		value &= ~(PRCM_CORE_MEMBIT1 | PRCM_CORE_PWRSTATEBIT2);
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_LOGICBIT;
		break;

	case PRCM_CORE_CSWR_MEM2OFF:
		value &= ~(PRCM_CORE_MEMBIT2 | PRCM_CORE_PWRSTATEBIT2);
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_LOGICBIT;
		break;

	case PRCM_CORE_CSWR_MEMOFF:
		value &= ~(PRCM_CORE_MEMBIT | PRCM_CORE_PWRSTATEBIT2);
		value |= PRCM_CORE_PWRSTATEBIT1 | PRCM_CORE_LOGICBIT;
		break;

	case PRCM_CORE_OSWR_MEM1OFF:
		value |= PRCM_CORE_PWRSTATEBIT1;
		value &= ~(PRCM_CORE_MEMBIT1 | PRCM_CORE_LOGICBIT |
						PRCM_CORE_PWRSTATEBIT2);
		break;

	case PRCM_CORE_OSWR_MEMRET:
		value |= (PRCM_CORE_MEMBIT | PRCM_CORE_PWRSTATEBIT1);
		value &= ~(PRCM_CORE_LOGICBIT | PRCM_CORE_PWRSTATEBIT2);
		break;

	case PRCM_CORE_OSWR_MEM2OFF:
		value |= PRCM_CORE_PWRSTATEBIT1;
		value &= ~(PRCM_CORE_MEMBIT2 | PRCM_CORE_LOGICBIT |
					PRCM_CORE_PWRSTATEBIT2);
		break;

	case PRCM_CORE_OSWR_MEMOFF:
		value |= PRCM_CORE_PWRSTATEBIT1;
		value &= ~(PRCM_CORE_MEMBIT | PRCM_CORE_LOGICBIT  |
					PRCM_CORE_PWRSTATEBIT2);
		break;

	case PRCM_CORE_OFF:
		value |= PRCM_CORE_PWRSTATEOFF;
		PRM_VOLTCTRL |= PRM_VOLTCTRL_AUTO_OFF;
#ifdef CONFIG_SYSOFFMODE
		PRM_VOLTCTRL |= PRM_VOLTCTRL_SEL_OFF;
#endif
		prcm_save_core_context(PRCM_CORE_OFF);
		break;

	default:
		return PRCM_FAIL;
	}

	PM_PWSTCTRL_CORE = value;

#if 1 /* was #ifdef CONFIG_HW_SUP_TRANS */
	/* L3, L4 and D2D clock autogating */
	CM_CLKSTCTRL_CORE = (CLK_D2D_HW_SUP_ENABLE |
			     CLK_L4_HW_SUP_ENABLE |
			     CLK_L3_HW_SUP_ENABLE);
#else
	if (core_dom_state != PRCM_CORE_ACTIVE) {
		/* L3, L4 and D2D clock autogating */
		CM_CLKSTCTRL_CORE = (CLK_D2D_HW_SUP_ENABLE |
				     CLK_L4_HW_SUP_ENABLE |
				     CLK_L3_HW_SUP_ENABLE);
	} else {
		CM_CLKSTCTRL_CORE = 0x0;
	}
#endif

	return PRCM_PASS;
}


