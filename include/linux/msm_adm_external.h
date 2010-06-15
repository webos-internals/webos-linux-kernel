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

#ifndef _ARCH_ARM_MACH_MSM_ADM_EXTERNAL_H
#define _ARCH_ARM_MACH_MSM_ADM_EXTERNAL_H


#define MSMADM_IOCTL_MAGIC 'K'
#define MSMADM_PMEM_BOX_COPY          _IOW(MSMADM_IOCTL_MAGIC, 1, unsigned int)

struct adm_box_img
{
	uint32_t rows;
	uint32_t columns;
	uint32_t row_size;
	uint32_t offset;
	int fd;
};

struct pmem_adm_box_transfer
{
	struct adm_box_img src;
	struct adm_box_img dst;
};


#endif //_ARCH_ARM_MACH_MSM_ADM_EXTERNAL_H

