/*
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP2/3 CPU idle implmentation.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
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

#ifndef _PM_34XX_CPUIDLE_H_
#define _PM_34XX_CPUIDLE_H_

#include <linux/seq_file.h>

/* -wgr- HACK.
 * This was introduced by TI to make cpuidle work. This function is implemented
 * in arch/arm/plat-omap/timer-gp.c
 */
extern void omap2_gp_timer_program_next_event(unsigned long cycles);

int pm_cpuidle_prepwst_show(struct seq_file *m, void *v);

int __init pm_cpuidle_init(void);

#endif /* _PM_34XX_CPUIDLE_H_ */
