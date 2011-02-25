/*
 * linux/arch/arm/mach-omap3pe/resource.h
 *
 * Copyright (C) 2008 Palm, Inc.
 *
 * Based on OMAP34XX Shared Resource Framework
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
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

#ifndef __ARCH_ARM_MACH_OMAP3_RESOURCE_H
#define __ARCH_ARM_MACH_OMAP3_RESOURCE_H

#include <linux/kernel.h>

int __init omap3_resource_init(void);

/* Flags to denote Pool usage */
#define RES_UNUSED			0x0
#define RES_USED			0x1

#endif /* __ARCH_ARM_MACH_OMAP3_RESOURCE_H */
