/*
 * sound/arm/omap/omap2-dasf-mailbox.c
 * 
 * SW mailbox implementation for OMAP ALSA DSP driver  
 *
 * 
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
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

#include "omap2_dasf_host.h"
#include "omap2_dasf_mailbox.h"
#include "omap2_dasf_std.h"

// #undef DEBUG
#define DEBUG

#ifdef DEBUG
#define DPRINT  printk
#define FN_IN printk("[omap2_dasf_mailbox.c:[%s] start\n", __FUNCTION__)
#define FN_OUT(n) printk("[omap2_dasf_mailbox.c:[%s] end(%d)\n", __FUNCTION__ , n)
#else
#define DPRINT( x... )
#define FN_IN
#define FN_OUT(x)
#endif

/* Mailbox time out callback function */
static void timeout_callback(unsigned long ptr)
{
	dasf_mailbox_t *mailbox_ptr = (dasf_mailbox_t *) ptr;

	FN_IN;
	if (test_and_set(&mailbox_ptr->response_empty, alsa_false) == alsa_true) {
		DPRINT("timeout_callback: up the response mutex \n");
		up(&mailbox_ptr->get_response_mutex);
	}
}

ALSA_STATUS dasf_init_mailbox(dasf_mailbox_t * dasf_mailbox_ptr)
{
	FN_IN;
	/* Initialize the semaphores */
	init_MUTEX(&dasf_mailbox_ptr->put_cmd_mutex);
	init_MUTEX_LOCKED(&dasf_mailbox_ptr->get_cmd_mutex);
	init_MUTEX_LOCKED(&dasf_mailbox_ptr->get_response_mutex);
	init_MUTEX(&dasf_mailbox_ptr->put_response_mutex);
	dasf_mailbox_ptr->cmd_empty = alsa_true;
	dasf_mailbox_ptr->response_empty = alsa_true;

	return ALSA_SOK;
}

ALSA_STATUS dasf_put_mailbox_cmd(dasf_mailbox_t * dasf_mailbox_ptr,
				 dasf_trapped_cmd_t * dasf_cmd_ptr,
				 dasf_trapped_response_t * dasf_res_ptr)
{
	ALSA_STATUS status = ALSA_SOK;
	struct timer_list timeout;

	FN_IN;
	/* Later, we may need to add timeout here */
	if (down_interruptible(&(dasf_mailbox_ptr->put_cmd_mutex))) {
		DPRINT("Interrupted waiting in mailbox command \n");
		status = ALSA_EINTERRUPTED;
		return status;
	}

	/* Flush any stale response messages */
	memset(&(dasf_mailbox_ptr->response_area), 0x00, sizeof(dasf_trapped_response_t));
	
	/* copy the command */
	dasf_mailbox_ptr->cmd_area = *(dasf_cmd_ptr);
	dasf_mailbox_ptr->response_empty = alsa_true;

	/* Release the DASF get command semaphore */
	up(&(dasf_mailbox_ptr->get_cmd_mutex));

	/* Wait for the response from the DASF side */
	/* start a timer -- */
	init_timer(&timeout);
	timeout.function = timeout_callback;
	timeout.data = (unsigned long)dasf_mailbox_ptr;
	timeout.expires = jiffies + DASF_TIMEOUT * HZ / 1000;
	add_timer(&timeout);

	if (down_interruptible(&(dasf_mailbox_ptr->get_response_mutex))) {
		DPRINT("Interrupted waiting in mailbox Response \n");
		up(&(dasf_mailbox_ptr->put_cmd_mutex));
		status = ALSA_EINTERRUPTED;
	}

	if (in_interrupt()) {
		if (!del_timer(&timeout)) {
			DPRINT("dasf_put_mailbox_cmd: Timer expired \n");
		}
	} else {
		if (!del_timer_sync(&timeout)) {
			DPRINT("dasf_put_mailbox_cmd: Timer expired \n");
		}
	}

	if (dasf_mailbox_ptr->response_empty == alsa_true) {
		DPRINT("mailbox.response_empty = TRUE !!!!!!!\n");
		status = ALSA_ETIMEOUT;
	} else {
		*(dasf_res_ptr) = dasf_mailbox_ptr->response_area;
		status = ALSA_SOK;
	}

	/* give away the command semaphore and return */
	up(&(dasf_mailbox_ptr->put_cmd_mutex));

	return status;
}

ALSA_STATUS dasf_get_mailbox_cmd(dasf_mailbox_t * dasf_mailbox_ptr,
				 dasf_trapped_cmd_t * dasf_cmd_ptr)
{
	ALSA_STATUS status = ALSA_SOK;

	FN_IN;
	/* Wait for the semaphore */
	if (down_interruptible(&(dasf_mailbox_ptr->get_cmd_mutex))) {
		DPRINT("Interrupted waiting in mailbox Response \n");
		status = ALSA_EINTERRUPTED;
		return status;
	}

	*(dasf_cmd_ptr) = dasf_mailbox_ptr->cmd_area;

	return status;
}

ALSA_STATUS dasf_put_mailbox_response(dasf_mailbox_t * dasf_mailbox_ptr,
				      dasf_trapped_response_t * res_ptr)
{
	ALSA_STATUS status = ALSA_SOK;

	FN_IN;
	dasf_mailbox_ptr->response_area = *(res_ptr);

	dasf_mailbox_ptr->response_empty = alsa_false;
	/* Giveaway the semaphore */
	up(&(dasf_mailbox_ptr->get_response_mutex));

	FN_OUT(status);
	return status;
}

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Software Mailbox frame work for ALSA-DSP driver");
MODULE_LICENSE(" GPL ");
