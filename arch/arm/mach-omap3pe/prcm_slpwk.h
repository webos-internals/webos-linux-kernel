/*
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

#ifndef _PRCM_SLPWK_H_
#define _PRCM_SLPWK_H_

#include <linux/types.h>

void prcm_wait_for_clock(u32 deviceid);
int prcm_is_device_accessible(u32 deviceid, u8 *result);
void prcm_setup_default_wakeup_sources(void);

#endif /* _PM_34XX_CPUIDLE_H_ */



