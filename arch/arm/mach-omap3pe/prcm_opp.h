/*
 * Copyright (C) 2008-2009 Palm, Inc.
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
 *
 */

#ifndef _PRCM_OPP_H_
#define _PRCM_OPP_H_

#include <linux/init.h>
#include <linux/types.h>

/******************************************************************************/

struct dpll_param {
	u32 dpll_m;
	u32 dpll_n;
	u32 dpll_freqsel;
	u32 dpll_m2;
};

/******************************************************************************/

int prcm_lock_iva_dpll(u32 target_opp_id);
void prcm_scale_finish(void);

void vdd1_opp_setting(u32 target_opp_no);
void vdd2_opp_setting(u32 target_opp_no);

u32 prcm_get_current_vdd1_opp(void);
u32 prcm_get_current_vdd2_opp(void);
u32 omap3_max_vdd1_opp(void);

int __init prcm_vdd_clk_init(void);

#ifdef CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND
void prcm_scale_prepare(void);
#endif

#endif
