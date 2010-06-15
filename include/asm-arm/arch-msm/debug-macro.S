/* include/asm-arm/arch-msm7200/debug-macro.S
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/hardware.h>
#include <asm/arch/msm_iomap.h>

	.macro	addruart,rx
#if defined(MSM_DEBUG_UART_PHYS)
	@ see if the MMU is enabled and select appropriate base address
	mrc	p15, 0, \rx, c1, c0
	tst	\rx, #1
	ldreq	\rx, =MSM_DEBUG_UART_PHYS
	ldrne	\rx, =MSM_DEBUG_UART_BASE
#endif
	.endm

	.macro	senduart,rd,rx
#if defined(MSM_DEBUG_UART_PHYS)
	str	\rd, [\rx, #0x0C]
#endif
	.endm

	.macro	waituart,rd,rx
#if defined(MSM_DEBUG_UART_PHYS)
	@ wait for TX_READY
1:	ldr	\rd, [\rx, #0x08]
	tst	\rd, #0x04
	beq	1b
#endif
	.endm

	.macro	busyuart,rd,rx
	.endm
