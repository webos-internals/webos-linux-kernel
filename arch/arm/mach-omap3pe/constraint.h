/*
 * linux/arch/arm/mach-omap3/constraint.h
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

#ifndef __ARCH_ARM_MACH_OMAP3_CONSTRAINT_H
#define __ARCH_ARM_MACH_OMAP3_CONSTRAINT_H

/* Constraints levels */
#define CO_UNUSED		0x0

/* Latency constraint levels */
#define CO_LATENCY_WFI			0x1
#define CO_LATENCY_MPURET_COREON	0x2
#define CO_LATENCY_MPUOFF_COREON	0x3
#define CO_LATENCY_MPUOFF_CORERET	0x4

/* Power domain latency constraint levels */
#define CO_LATENCY_ON			0x1
#define CO_LATENCY_RET			0x2

int activate_constraint(struct shared_resource *resp,
			unsigned short current_value,
			unsigned short target_value);

int activate_pd_constraint(struct shared_resource *resp,
			   unsigned short current_value,
			   unsigned short target_value);

int validate_constraint(struct shared_resource *res,
				unsigned short current_value,
				unsigned short target_value);
#endif /* __ARCH_ARM_MACH_OMAP3_CONSTRAINT_H */
