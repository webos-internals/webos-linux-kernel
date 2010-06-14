/*
 * Copyright (C) 2008 Palm, Inc.
 *
 * Based on OMAP 34xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef _PRCM_SLPWK_H_
#define _PRCM_SLPWK_H_

#include <linux/types.h>

void prcm_wait_for_clock(u32 deviceid);
int prcm_is_device_accessible(u32 deviceid, u8 *result);
void prcm_setup_default_wakeup_sources(void);

#endif /* _PM_34XX_CPUIDLE_H_ */



