/*
 * arch/arm/kernel/kgdb-jmp.S
 *
 * Trivial setjmp and longjmp procedures to support bus error recovery
 * which may occur during kgdb memory read/write operations.
 *
 * Author: MontaVista Software, Inc. <source@mvista.com>
 *         source@mvista.com
 *
 * 2002-2005 (c) MontaVista Software, Inc.  This file is licensed under the
 * terms of the GNU General Public License version 2. This program as licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */
#include <linux/linkage.h>

ENTRY (kgdb_fault_setjmp)
	/* Save registers */
	stmia	r0, {r0-r14}
	str	lr,[r0, #60]
	mrs	r1,cpsr
	str	r1,[r0,#64]
	ldr	r1,[r0,#4]
	mov	r0, #0
	mov	pc,lr

ENTRY (kgdb_fault_longjmp)
	/* Restore registers */
	mov	r1,#1
	str	r1,[r0]
	ldr     r1,[r0, #64]
	msr     spsr,r1
	ldmia	r0,{r0-pc}^
