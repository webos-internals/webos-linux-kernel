#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/io.h>
#include <asm/hardware.h>

#include "omap_wdt_fiq.h"

	.text

	.global wdt_fiq_end

ENTRY(wdt_fiq_start)
	mov	r8, #0
	ldr	r9, [r8]			@ data abort!

	subs	pc, lr, #4			@ we don't reach here

wdt_fiq_end:

/*
 * Check the size of the FIQ,
 * it cannot go beyond 0xffff0200, and is copied to 0xffff001c
 */
.if (wdt_fiq_end - wdt_fiq_start) > (0x200 - 0x1c)
	.err
.endif
