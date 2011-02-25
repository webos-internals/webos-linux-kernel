/*
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP2 serial support.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _SERIAL_H_
#define _SERIAL_H_

#define UART_TIME_OUT 6000	/* ms before cutting clock */

void omap_uart_save_ctx(u32 unum);
void omap_uart_restore_ctx(u32 unum);

#endif /* _PM_34XX_CPUIDLE_H_ */

