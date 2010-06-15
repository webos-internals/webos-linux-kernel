/* arch/arm/mach-msm/pm.h
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

#ifndef __ARCH_ARM_MACH_MSM_PM_H
#define __ARCH_ARM_MACH_MSM_PM_H

#include <asm/arch/msm_iomap.h>

enum {
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE,
	MSM_PM_SLEEP_MODE_APPS_SLEEP,
	MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT,
	MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT,
	MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN,
	MSM_PM_SLEEP_MODE_NR
};

struct msm_pm_platform_data {
	u8 supported;
	u8 suspend_enabled;  /* enabled for suspend */
	u8 idle_enabled;     /* enabled for idle low power */
	u32 latency;         /* interrupt latency in microseconds when entering
				and exiting the low power mode */
	u32 residency;       /* time threshold in microseconds beyond which
				staying in the low power mode saves power */
};

#define A11S_CLK_SLEEP_EN_ADDR MSM_CSR_BASE + 0x11c

#define CLK_SLEEP_EN_ARM11_CORE	0x01
#define CLK_SLEEP_EN_ARM11_AHB	0x02
#define CLK_SLEEP_EN_ID_BRIDGE	0x04
#define CLK_SLEEP_EN_DMA_BRIDGE	0x08
#define CLK_SLEEP_EN_PBUS	0x10
#define CLK_SLEEP_EN_DEBUG_TIME	0x20
#define CLK_SLEEP_EN_GP_TIMER	0x40

void msm_pm_set_platform_data(struct msm_pm_platform_data *data);


/******************************************************************************
 * Shared Memory Data
 *****************************************************************************/

#define DEM_WAKEUP_REASON_NONE       0x00000000
#define DEM_WAKEUP_REASON_SMD        0x00000001
#define DEM_WAKEUP_REASON_INT        0x00000002
#define DEM_WAKEUP_REASON_GPIO       0x00000004
#define DEM_WAKEUP_REASON_TIMER      0x00000008
#define DEM_WAKEUP_REASON_ALARM      0x00000010
#define DEM_WAKEUP_REASON_RESET      0x00000020
#define DEM_WAKEUP_REASON_OTHER      0x00000040
#define DEM_WAKEUP_REASON_REMOTE     0x00000080

#define DEM_MAX_PORT_NAME_LEN (20)

struct msm_pm_smem_t {
	uint32_t sleep_time;
	uint32_t irq_mask;
	uint32_t resources_used;
	uint32_t power_collapse_delay;
	uint32_t wakeup_reason;
	uint32_t pending_irqs;
	uint32_t rpc_prog;
	uint32_t rpc_proc;
	char     smd_port_name[DEM_MAX_PORT_NAME_LEN];
	uint32_t gpio;
};

#endif
