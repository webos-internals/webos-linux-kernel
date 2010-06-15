/* arch/arm/mach-msm/msm_adm.h
 *
 * MSM7x25 Application Data Mover Interface
 *
 * Copyright (C) 2009 Palm, Inc.
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

#ifndef _ARCH_ARM_MACH_MSM_ADM_H
#define _ARCH_ARM_MACH_MSM_ADM_H

#include <asm/arch/dma.h>
#include <linux/workqueue.h>

#define DMA_READY	0
#define DMA_BUSY	1
	
struct pmem_adm_nc_dmadata {
	dmov_box	cmd[1];
	uint32_t	cmdptr;
};

struct adm_device_state {
	struct miscdevice mdev;
	struct file_operations fops;

	uint32_t	fb_start;
	uint32_t	fb_size;

	wait_queue_head_t wq;
	int dma_state;
};

struct adm_dma_data {
	struct pmem_adm_nc_dmadata	*nc;
	dma_addr_t			nc_busaddr;
	dma_addr_t			cmd_busaddr;
	dma_addr_t			cmdptr_busaddr;

	struct msm_dmov_cmd		hdr;
	enum dma_data_direction		dir;

	int				channel;

	struct work_struct 		workitem;

	struct adm_device_state		*state;
};


#endif //_ARCH_ARM_MACH_MSM_ADM_H

