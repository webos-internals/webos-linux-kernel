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

#include <mach/msm_iomap.h>

// Exported PWM functions
int msm_pwm_configure(u16 duty_percent);
int msm_pwm_enable(void);
int msm_pwm_disable(void);

#endif
