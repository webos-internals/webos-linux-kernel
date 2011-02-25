/*
 * linux/arch/arm/mach-omap3pe/context.h
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on linux/arch/arm/mach-omap2 by
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#ifndef __ARCH_ARM_MACH_OMAP3_CONTEXT_H
#define __ARCH_ARM_MACH_OMAP3_CONTEXT_H

#include <linux/types.h>
#include <asm/arch/clock.h>

#ifndef CONFIG_ARCH_OMAP3410
void omap3_save_neon_context(void);
void omap3_restore_neon_context(void);
#endif

void omap3_save_per_context(void);
void omap3_restore_per_context(void);

#endif

