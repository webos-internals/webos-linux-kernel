/* linux/include/asm-arm/arch-msm/clock.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_CLOCK_H
#define __ASM_ARCH_MSM_CLOCK_H

#include <linux/types.h>

struct msm_acpu_clock_platform_data
{
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
	unsigned int max_axi_khz;
};

struct clk;

void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);
void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks);
void __init halibut_clock_init(void);
void __init blenny_clock_init(void);
void __init clock_init(uint32_t, uint32_t, uint32_t, unsigned long,
			unsigned long);
int clk_register(struct clk *);

#endif
