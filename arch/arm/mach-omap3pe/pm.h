/*
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on
 *
 * OMAP3 Power Management Routines
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

#ifndef _PM_H_
#define _PM_H_

#include <linux/suspend.h>

#include "prcm_pwr.h"

void memory_logic_res_setting(struct system_power_state *trg_st);
#ifdef CONFIG_CORE_OFF
void omap3_restore_core_settings(void);
#endif
int  omap3_pm_register_target_pm_ops(struct platform_suspend_ops *ops);

#endif /* _PM_34XX_CPUIDLE_H_ */


