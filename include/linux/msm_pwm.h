/* include/linux/msm_pwm.h
 *
 * MSM7K pwm support header
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Kevin McCray <kevin.mccray@palm.com>
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

#ifndef _LINUX_MSM_PWM_H
#define _LINUX_MSM_PWM_H

#include <asm/arch/msm_iomap.h>

// Qualcomm's Software Interface Manual (80-VF815-2 Rev C) doesn't
// have these registers bit values defined but according to
// Qualcomm it should be the following:
// GP_MN_CLK_MDIV bits[8:0] = M value
// GP_MN_CLK_NDIV bits[12:0] = 1's complement of N value
// GP_MN_CLK_DUTY bits [12:0] = D value which must be between M and (M-N)
#define GP_MN_CLK_MDIV  (MSM_WEB_TCXO4_BASE + 0x4C)
#define GP_MN_CLK_NDIV	(MSM_WEB_TCXO4_BASE + 0x50)
#define GP_MN_CLK_DUTY	(MSM_WEB_TCXO4_BASE + 0x54)
#define WEB_TCXO4_TEST	(MSM_WEB_TCXO4_BASE + 0x58)


#define PRPH_WEB_NS_REG 	(MSM_CLK_CTL_BASE + 0x0080)
#define PRPH_WEB_ROOT_ENA 	(1 << 11)
#define PRPH_CLK_BRANCH_ENA 	(1 << 9)
#define PRPH_CLK_INV		(1 << 10)	

#define GP_M_VALUE_20KHZ	2
#define GP_N_VALUE_20KHZ	0x1DE
#define GP_N_1COMP_VALUE_20KHZ	0x1E21


// Exported PWM functions
int msm_pwm_configure(u16 duty_percent);
int msm_pwm_enable(void);
int msm_pwm_disable(void);

#endif

