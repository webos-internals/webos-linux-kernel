/*
 * sound/arm/omap/omap2_dasf_mailbox.h
 *
 * OMAP alsa-dasf driver Software Mailbox interface
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 */

#ifndef __OMAP_DASF_MAILBOX_H
#define __OMAP_DASF_MAILBOX_H

#include "omap2_dasf_host.h"
#include "omap2_dasf_std.h"

typedef struct {
	struct semaphore put_cmd_mutex;
	struct semaphore get_response_mutex;
	struct semaphore get_cmd_mutex;
	struct semaphore put_response_mutex;
	dasf_trapped_cmd_t cmd_area;
	dasf_trapped_response_t response_area;
	boolean_t cmd_empty;
	boolean_t response_empty;
} dasf_mailbox_t;

/* mailbox API */

ALSA_STATUS dasf_init_mailbox(dasf_mailbox_t * dasf_mailbox);

ALSA_STATUS dasf_put_mailbox_cmd(dasf_mailbox_t * dasf_mailbox,
				 dasf_trapped_cmd_t * cmd,
				 dasf_trapped_response_t * response);

ALSA_STATUS dasf_get_mailbox_cmd(dasf_mailbox_t * dasf_mailbox,
				 dasf_trapped_cmd_t * cmd);

ALSA_STATUS dasf_put_mailbox_response(dasf_mailbox_t * dasf_mailbox,
				      dasf_trapped_response_t * response);

#endif				/* End of #ifndef __OMAP_DASF_MAILBOX_H */
