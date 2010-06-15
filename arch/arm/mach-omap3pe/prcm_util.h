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

#ifndef _PRCM_UTIL_H_
#define _PRCM_UTIL_H_

#include <linux/delay.h>

/* Macros to wait while/until a condition is true. Timeout specified in usecs.
 * Use for very short waits only.
 *
 * Return value:
 *  >= 0   - no timeout.
 *  <  0   - timeout occured.
 */
#define WAIT_WHILE(_cond, _usecs) \
	({ \
		int retries = _usecs; \
		while (retries-- && (_cond)) { udelay(1); } \
		retries; \
	})

#define WAIT_UNTIL(_cond, _usecs)	WAIT_WHILE(!(_cond), _usecs)

void prcm_save_registers(void);
void prcm_restore_registers(void);

#endif /* _PM_34XX_CPUIDLE_H_ */
