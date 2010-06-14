/*
 * Copyright (C) 2008-2009 Palm, Inc.
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

#include <asm/io.h>
#include <asm/arch/pm.h>

#include "scratchpad.h"
#include "prcm-regs.h"

/* Macro for debugging */
//#define RESTORE_TABLE_ENTRY 1

#define SCRATCHPAD_ROM_OFFSET		0x158
#define TABLE_ADDRESS_OFFSET		0x31
#define TABLE_VALUE_OFFSET		0x30
#define CONTROL_REG_VALUE_OFFSET	0x32

static u32 context_mem[256];

u32 *get_context_mem_addr(void)
{
	return context_mem;
}

#ifndef DONT_DO_THE_SCRATCHPAD_DANCE
static u32 *scratchpad_restore_addr = NULL;
static u32 restore_pointer_address  = 0;

void scratchpad_set_restore_addr(void)
{
	if (scratchpad_restore_addr)
		*scratchpad_restore_addr = restore_pointer_address;
	else
		printd("##### scratchpad_set_restore_addr() called with "
				"scratchpad_restore_addr == NULL\n");
}

void scratchpad_clr_restore_addr(void)
{
	if (scratchpad_restore_addr)
		*scratchpad_restore_addr = 0;
	else
		printd("##### scratchpad_clr_restore_addr() called with "
				"scratchpad_restore_addr == NULL\n");
}
#endif

/******************************************************************************
 *
 * Populate the scratchpad structure with restore structure
 *
 ******************************************************************************/

void save_scratchpad_contents(void)
{
	u32 *scratchpad_address;
	u32 *restore_address;
	u32 *sdram_context_address;

	/* Get virtual address of the scratchpad base */
	scratchpad_address = (u32 *) &SCRATCHPAD_BASE;

	/* Get Restore pointer to jump to while waking up from OFF */
	if (is_sil_rev_greater_than(OMAP3430_REV_ES2_1))
		restore_address = get_es3_restore_pointer();
	else
		restore_address = get_restore_pointer();

	/* Convert it to physical address */
	restore_address = (u32 *) virt_to_phys(restore_address);

	/* Get address where registers are saved in SDRAM */
	sdram_context_address = (u32 *) virt_to_phys(context_mem);

	/* Booting configuration pointer*/
	*(scratchpad_address++) = 0x0;

#ifdef DONT_DO_THE_SCRATCHPAD_DANCE
	*(scratchpad_address++) = (u32) restore_address;
#else
	/* Public restore pointer */
	/* On ES 2.0, if scratchpad is populated with valid pointer, warm reset
	 * does not work So populate scratchpad restore address only in cpuidle
	 * and suspend calls.
	 *
	 * Remember the address of where the restore_address is stored and also
	 * the value of the restore address itself so we can "enable" and
	 * "disable" the pointer later on using scratchpad_set_restore_addr()
	 * and scratchpad_clr_restore_addr().
	 */
	scratchpad_restore_addr = scratchpad_address;
	restore_pointer_address = (u32) restore_address;

	/* Initialize restore addr with 0x0 for now. */
	*(scratchpad_address++) = 0;
#endif

	/* Secure ram restore pointer */
	*(scratchpad_address++) = 0x0;
	/* SDRC Module semaphore */
	*(scratchpad_address++) = 0x0;
	/* PRCM Block Offset */
	*(scratchpad_address++) = 0x2C;
	/* SDRC Block Offset */
	*(scratchpad_address++) = 0x64;
	/* Empty */
	/* Offset 0x8*/
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	/* Offset 0xC*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x10*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x14*/
	*(scratchpad_address++) = 0x0;
	/* Offset 0x18*/
	/* PRCM Block */
	*(scratchpad_address++) = PRM_CLKSRC_CTRL;
	*(scratchpad_address++) = PRM_CLKSEL;
	*(scratchpad_address++) = CM_CLKSEL_CORE;
	*(scratchpad_address++) = CM_CLKSEL_WKUP;
	*(scratchpad_address++) = CM_CLKEN_PLL;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL;
	*(scratchpad_address++) = CM_CLKSEL1_PLL;
	*(scratchpad_address++) = CM_CLKSEL2_PLL;
	*(scratchpad_address++) = CM_CLKSEL3_PLL;
	*(scratchpad_address++) = CM_CLKEN_PLL_MPU;
	*(scratchpad_address++) = CM_AUTOIDLE_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL1_PLL_MPU;
	*(scratchpad_address++) = CM_CLKSEL2_PLL_MPU;
	*(scratchpad_address++) = 0x0;
	/* SDRC Block */
	*(scratchpad_address++) = ((SDRC_CS_CFG & 0xFFFF) << 16) |
					(SDRC_SYSCONFIG & 0xFFFF);
	*(scratchpad_address++) = ((SDRC_ERR_TYPE & 0xFFFF) << 16) |
					(SDRC_SHARING & 0xFFFF);
	*(scratchpad_address++) = SDRC_DLLA_CTRL;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_POWER_REG;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_MCFG_0;
	*(scratchpad_address++) = SDRC_MR_0 & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRLA_0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRLB_0;
	*(scratchpad_address++) = SDRC_RFR_CTRL_0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_MCFG_1;
	*(scratchpad_address++) = SDRC_MR_1 & 0xFFFF;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = SDRC_ACTIM_CTRLA_1;
	*(scratchpad_address++) = SDRC_ACTIM_CTRLB_1;
	*(scratchpad_address++) = SDRC_RFR_CTRL_1;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = 0x0;
	*(scratchpad_address++) = (u32) sdram_context_address;
}


/******************************************************************************
 *
 * Clears the scratchpad contents in case of cold boot- called during bootup
 *
 ******************************************************************************/

#ifdef CONFIG_PM
void clear_scratchpad_contents(void)
{
	/* Check if it is a cold reboot */
	if (PRM_RSTST & 0x1) {
		u32 offset;
		u32 v_addr = (u32) &SCRATCHPAD_ROM_BASE;

		for (offset = 0; offset <= SCRATCHPAD_ROM_OFFSET; offset += 0x4)
			__raw_writel(0x0, (v_addr + offset));
		PRM_RSTST |= 0x1;
	}
}
#endif


/******************************************************************************
 *
 * Function to restore the table entry that was modified for enabling MMU
 *
 ******************************************************************************/

#ifdef CONFIG_MPU_OFF
static void restore_control_register(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c1, c0, 0" : : "r" (val));
}

void restore_table_entry(void)
{
	u32 *scratchpad_address;
	u32 previous_value, control_reg_value;
	u32 *address;
	/* Get virtual address of scratchpad base */
	scratchpad_address = (u32 *) &SCRATCHPAD_BASE;
	/* Get address of entry that was modified */
	address = (u32 *) *(scratchpad_address + TABLE_ADDRESS_OFFSET);
	/* Get the previous value which needs to be restored */
	previous_value = *(scratchpad_address + TABLE_VALUE_OFFSET);
#ifdef RESTORE_TABLE_ENTRY
	/* Convert address to virtual address */
	address = __va(address);
	/* Restore table entry */
	*address = previous_value;
	/* Flush TLB */
	flush_tlb_all();
#endif
	control_reg_value = *(scratchpad_address + CONTROL_REG_VALUE_OFFSET);
	/* Restore control register*/
	/* This will enable caches and prediction */
	restore_control_register(control_reg_value);
}
#endif

