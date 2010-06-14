/*
 * linux/arch/arm/mach-omap3pe/sram.S
 *
 * Omap3 specific functions that need to be run in internal SRAM
 *
 * (C) Copyright 2007
 * Texas Instruments Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * (C) Copyright 2004
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; version 2 of
 * that License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/io.h>
#include <asm/hardware.h>

#include "prcm-regs.h"

#define CM_CLKSEL1_PLL_V	IO_ADDRESS(CM_BASE + 0xD40)
#define	CM_ICLKEN1_CORE_V	IO_ADDRESS(CM_BASE + 0xA10)
#define CM_IDLEST1_CORE_V	IO_ADDRESS(CM_BASE + 0xA20)

#define	SDRC_POWER_V		IO_ADDRESS(SDRC_BASE + 0x070)
#define SDRC_RFR_CTRL_V		IO_ADDRESS(SDRC_BASE + 0x0A4)
#define SDRC_ACTIM_CTRL_A_V	IO_ADDRESS(SDRC_BASE + 0x09C)
#define	SDRC_ACTIM_CTRL_B_V	IO_ADDRESS(SDRC_BASE + 0x0A0)
#define	SDRC_DLLA_STATUS_V	IO_ADDRESS(SDRC_BASE + 0x064)
#define	SDRC_DLLA_CTRL_V	IO_ADDRESS(SDRC_BASE + 0x060)

	.text

ENTRY(sram_ddr_init)
	stmfd	sp!, {r0 - r12, lr}	@ save registers on stack
	ldmfd	sp!, {r0 - r12, pc}	@ restore regs and return
ENTRY(sram_ddr_init_sz)
	.word	. - sram_ddr_init

ENTRY(sram_reprogram_sdrc)
	stmfd	sp!, {r0 - r10, lr}	@ save registers on stack
	ldmfd	sp!, {r0 - r10, pc}	@ restore regs and return
ENTRY(sram_reprogram_sdrc_sz)
	.word	. - sram_reprogram_sdrc

/*
 * Set dividers and pll. Also recalculate DLL value for DDR and unlock mode.
 */
ENTRY(sram_set_prcm)
	stmfd	sp!, {r0-r12, lr}	@ regs to stack
ENTRY(sram_set_prcm_sz)
	.word	. - sram_set_prcm

/*
 * Change frequency of core dpll
 * r0 = sdrc_rfr_ctrl r1 = sdrc_actim_ctrla  r2 = sdrc_actim_ctrlb r3 = M2
 */
ENTRY(sram_configure_core_dpll)
	stmfd	sp!, {r1-r12, lr}	@ store regs to stack
	cmp	r3, #0x2
	it	ne
	blne	configure_sdrc
	cmp	r3, #0x2
	it	ne
	blne	lock_dll
	cmp	r3, #0x1
	it	ne
	blne	unlock_dll
	bl	sdram_in_selfrefresh	@ put the SDRAM in self refresh
	bl 	configure_core_dpll
	bl	enable_sdrc
	cmp	r3, #0x1
	it	ne
	blne	wait_dll_unlock
	cmp	r3, #0x2
	it	ne
	blne	wait_dll_lock
	cmp	r3, #0x1
	it	ne
	blne	configure_sdrc
	mov 	r0, #0 			@ return value 
	ldmfd	sp!, {r1-r12, pc}	@ restore regs and return
unlock_dll:
	ldr     r4, sdrc_dlla_ctrl
	ldr     r5, [r4]
	orr	r5, r5, #0x4
	str	r5, [r4]
	bx	lr
lock_dll:
	ldr     r4, sdrc_dlla_ctrl
	ldr     r5, [r4]
	bic	r5, r5, #0x4
	str     r5, [r4]
	bx      lr
sdram_in_selfrefresh:
	mov	r5, #0x0		@ Move 0 to R5
	mcr	p15, 0, r5, c7, c10, 5  @ memory barrier
	ldr 	r4, sdrc_power		@ read the SDRC_POWER register
	ldr 	r5, [r4]		@ read the contents of SDRC_POWER
	orr 	r5, r5, #0x40		@ enable self refresh on idle req
	str 	r5, [r4]		@ write back to SDRC_POWER register
	ldr	r4, cm_iclken1_core	@ read the CM_ICLKEN1_CORE reg
	ldr	r5, [r4]
	bic	r5, r5, #0x2		@ disable iclk bit for SRDC
	str 	r5, [r4]
wait_sdrc_idle:
	ldr 	r4, cm_idlest1_core
	ldr 	r5, [r4]
	and 	r5, r5, #0x2		@ check for SDRC idle 
	cmp 	r5, #2
	bne 	wait_sdrc_idle
	bx 	lr
configure_core_dpll:
	ldr 	r4, cm_clksel1_pll
	ldr	r5, [r4]
	ldr	r6, core_m2_mask_val	@ modify m2 for core dpll
	and	r5, r5, r6
	orr	r5, r5, r3, lsl #0x1B	@ r3 contains the M2 val
	str	r5, [r4]
	ldr 	r5, clk_stabilize_delay	@ wait for the clock to stabilise
	cmp	r3, #2
	bne	wait_clk_stable
	bx	lr
wait_clk_stable:
	subs 	r5, r5, #1
	bne	wait_clk_stable
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bx	lr
enable_sdrc:
	ldr 	r4, cm_iclken1_core
	ldr	r5, [r4]
	orr 	r5, r5, #0x2		@ enable iclk bit for SDRC
	str 	r5, [r4]
wait_sdrc_idle1:
	ldr 	r4, cm_idlest1_core
	ldr	r5, [r4]
	and 	r5, r5, #0x2
	cmp	r5, #0
	bne	wait_sdrc_idle1
	ldr	r4, sdrc_power
	ldr	r5, [r4]
	bic 	r5, r5, #0x40
	str 	r5, [r4]
	bx	lr
wait_dll_lock:
	ldr	r4, sdrc_dlla_status
	ldr	r5, [r4]
	and 	r5, r5, #0x4
	cmp	r5, #0x4
	bne	wait_dll_lock
	bx	lr
wait_dll_unlock:
	ldr     r4, sdrc_dlla_status
	ldr     r5, [r4]
	and     r5, r5, #0x4
	cmp     r5, #0x0
	bne     wait_dll_unlock
	bx      lr
configure_sdrc:
	ldr     r4, sdrc_rfr_ctrl
	str     r0, [r4]
	ldr 	r4, sdrc_actim_ctrla
	str     r1, [r4]
	ldr     r4, sdrc_actim_ctrlb
	str     r2, [r4]
	bx      lr
sdrc_power:
	.word SDRC_POWER_V
cm_clksel1_pll:
	.word CM_CLKSEL1_PLL_V
cm_idlest1_core:
	.word CM_IDLEST1_CORE_V
cm_iclken1_core:
	.word CM_ICLKEN1_CORE_V
sdrc_rfr_ctrl:
        .word SDRC_RFR_CTRL_V
sdrc_actim_ctrla:
	.word SDRC_ACTIM_CTRL_A_V
sdrc_actim_ctrlb:
	.word SDRC_ACTIM_CTRL_B_V
sdrc_dlla_status:
	.word SDRC_DLLA_STATUS_V
sdrc_dlla_ctrl:
	.word SDRC_DLLA_CTRL_V
core_m2_mask_val:
	.word 0xE7FFFFFF
clk_stabilize_delay:
	.word 0x000000C8
	
ENTRY(sram_configure_core_dpll_sz)
	.word	. - sram_configure_core_dpll

/*
 * Reprogram GPMC
 */
ENTRY(sram_reprogram_gpmc)
	stmfd	sp!, {r0-r12, lr}	@ regs to stack
	ldmfd	sp!, {r0-r12, pc}	@ restore regs and return

ENTRY(sram_reprogram_gpmc_sz)
	.word	. - sram_reprogram_gpmc
