/*
 * linux/arch/arm/mach-omap3pe/sram_idle.c
 *
 * Copyright (C) 2008 Palm, Inc.
 *
 * Based on OMAP3 Power Management Routines
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <asm/arch/pm.h>
#include <asm/arch/sram.h>

#include "scratchpad.h"
#include "prcm-regs.h"

static void (*_omap_sram_idle) (u32 * addr, int save_state) = NULL;

void omap_sram_idle_setup(void)
{
	_omap_sram_idle = omap_sram_push(omap34xx_suspend, omap34xx_suspend_sz);
}

/******************************************************************************
 *
 * IDLE function.
 *
 ******************************************************************************/

void omap_sram_idle(u32 mpu_target_state)
{
	/* Variable to tell what needs to be saved and restored
	 * in omap_sram_idle*/
	/* save_state = 0 => Nothing to save and restored */
	/* save_state = 1 => Only L1 and logic lost */
	/* save_state = 2 => Only L2 lost */
	/* save_state = 3 => L1, L2 and logic lost */

	int save_state = 0;

	if (!_omap_sram_idle)
		return;

	switch (mpu_target_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
	case PRCM_MPU_CSWR_L2RET:
		/* No need to save context */
		save_state = 0;
		break;
	case PRCM_MPU_OSWR_L2RET:
		/* L1 and Logic lost */
		save_state = 1;
		break;
	case PRCM_MPU_CSWR_L2OFF:
		/* Only L2 lost */
		save_state = 2;
		break;
	case PRCM_MPU_OSWR_L2OFF:
	case PRCM_MPU_OFF:
		/* L1, L2 and logic lost */
		save_state = 3;
		break;
	default:
		/* Invalid state */
		printk(KERN_ERR "Invalid mpu state in sram_idle\n");
		return;
	}

	_omap_sram_idle(get_context_mem_addr(), save_state);

//### Enabling clocks here. Wonder if there is a better place for this...
	CM_ICLKEN1_CORE |= 0x40;
	CM_ICLKEN1_CORE |= 0x02;


#ifdef CONFIG_MPU_OFF
	/* Restore table entry modified during MMU restoration */
	if ((PM_PREPWSTST_MPU & 0x3) == 0x0) {
		restore_table_entry();
	}
#endif
}

