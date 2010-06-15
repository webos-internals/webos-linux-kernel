/*
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#ifndef _SERIAL_H_
#define _SERIAL_H_

#define UART_TIME_OUT 6000	/* ms before cutting clock */

void omap_uart_save_ctx(u32 unum);
void omap_uart_restore_ctx(u32 unum);

#endif /* _PM_34XX_CPUIDLE_H_ */

