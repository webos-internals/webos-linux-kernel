/* arch/arm/mach-msm/mux.h
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

#ifndef __ARCH_ARM_MACH_MSM_MUX_H
#define __ARCH_ARM_MACH_MSM_MUX_H

struct pin_config {
	char *name;
	unsigned int active_mode;
	unsigned int sleep_mode;
};

#define MUX_CFG_MSM7x27(desc, active, sleep) {		\
	.name		= desc,				\
	.active_mode	= active,			\
	.sleep_mode	= sleep				\
},

#define CONFIG_ACTIVE	0
#define CONFIG_SLEEP	1


void msm_set_mux(char *name, int cfg);
void msm_dump_pin_table(void);

#endif
