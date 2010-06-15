/* include/linux/msm_perf.h
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

#ifndef __INCLUDE_LINUX_MSM_PERF_H
#define __INCLUDE_LINUX_MSM_PERF_H

#include <linux/types.h>
#include <linux/ioctl.h>

struct msm_perf_snapshot
{
	uint32_t	ccnt;    /* Cycle Count Register */
	int		ccnt_ov; /* ccnt overflow */
	uint32_t	pmn0;    /* Count Register 0 */
	int		pmn0_ov; /* pmn0 overflow */
	uint32_t	pmn1;    /* Count Register 1 */
	int		pmn1_ov; /* pmn1 overflow */
};

struct msm_perf_set_enable_args
{
	int enable; /* 1 == enable all perf counters */
};

struct msm_perf_set_ccnt_args
{
	int period;	/* 0 = count every cycle, 1 = count every 64th cycle */
};

struct msm_perf_event_args
{
	int pmn0_event;
	int pmn1_event;

#define PMN_EVENT_ICACHE_MISS			0x00
#define PMN_EVENT_I_STALL			0x01
#define PMN_EVENT_D_STALL			0x02
#define PMN_EVENT_I_MICROTLB_MISS		0x03
#define PMN_EVENT_D_MICROTLB_MISS		0x04
#define PMN_EVENT_BRANCH_EXECUTED		0x05
#define PMN_EVENT_BRANCH_MISPREDICT		0x06
#define PMN_EVENT_INSTRUCTION_EXECUTED		0x07
#define PMN_EVENT_DCACHE_ACCESS_CACHEABLE_OP	0x09
#define PMN_EVENT_DCACHE_ACCESS			0x0a
#define PMN_EVENT_DCACHE_MISS_NON_CACHEABLE_OP	0x0b
#define PMN_EVENT_DCACHE_WRITEBACK		0x0c
#define PMN_EVENT_SOFTWARE_CHANGED_PC		0x0d
#define PMN_EVENT_MAIN_TLB_MISS			0x0f
#define PMN_EVENT_EXPLICIT_EXT_DATA_ACCESS	0x10
#define PMN_EVENT_LS_STALL			0x11
#define PMN_EVENT_WRITE_BUFFER_DRAIN		0x12
#define PMN_EVENT_ETMEXTOUT_0			0x20
#define PMN_EVENT_ETMEXTOUT_1			0x21
#define PMN_EVENT_ETMEXTOUT_0_1			0x22
#define PMN_EVENT_CYCLE				0xff
};

#define MSM_PERF_IOCTL_MAGIC (0xC9)

#define MSM_PERF_IOCTL_SET_ENABLE \
   _IOWR(MSM_PERF_IOCTL_MAGIC, 1, unsigned int)

#define MSM_PERF_IOCTL_RESET \
   _IOWR(MSM_PERF_IOCTL_MAGIC, 2, unsigned int)

#define MSM_PERF_IOCTL_SET_CCNT_PERIOD \
   _IOWR(MSM_PERF_IOCTL_MAGIC, 3, unsigned int)

#define MSM_PERF_IOCTL_SET_EVENTS \
   _IOWR(MSM_PERF_IOCTL_MAGIC, 4, unsigned int)

#define MSM_PERF_IOCTL_GET_EVENTS \
   _IOR(MSM_PERF_IOCTL_MAGIC, 5, unsigned int)


#endif
