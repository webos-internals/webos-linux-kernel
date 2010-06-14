/*
 * linux/arch/arm/mach-omap3pe/clock.h
 *
 * Based on linux/arch/arm/mach-omap2/clock.h
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Based on OMAP2 clock framework created by
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Copyright (C) 2004 Nokia corporation
 * Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 * Based on clocks.h by Tony Lindgren, Gordon McNutt and RidgeRun, Inc
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
 *
 */

#ifndef __ARCH_ARM_MACH_OMAP3_CLOCK_H
#define __ARCH_ARM_MACH_OMAP3_CLOCK_H

/* initial hardcode defines */
#define S_OSC	13000000
#define S_32K	32768
#define S_ALT	54000000

#define S12M	12000000
#define S13M	13000000
#define S19M	19200000
#define S26M	26000000
#define S38M	38400000
#define S96M	96000000
#define S83M	83000000
#define S166M	166000000

#define S120M	120000000
#define S125M	125000000
#define S250M	250000000
#define S500M	500000000
#define S550M	550000000
#define S625M	625000000
#define S600M	600000000

/* Macro to enable clock control via clock framework */
#define ENABLE_CLOCKCONTROL 1

#endif
