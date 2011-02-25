/*
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP3 Power Management Routines
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SCRATCHPAD_H_
#define _SCRATCHPAD_H_

#include <linux/types.h>

/* The scratchpad dance is only required for ES2.0 and earlier.
 */
#define DONT_DO_THE_SCRATCHPAD_DANCE


u32 *get_context_mem_addr(void);
#ifdef DONT_DO_THE_SCRATCHPAD_DANCE
#  define scratchpad_set_restore_addr()
#  define scratchpad_clr_restore_addr()
#else
void scratchpad_set_restore_addr(void);
void scratchpad_clr_restore_addr(void);
#endif
void restore_table_entry(void);

#endif
