/*  
 * linux/drivers/sound/omap-audio-buf-intfc.h
 *
 * Common audio DMA handling for the OMAP processors
 *
 * Copyright (C) 2004-2006 Texas Instruments, Inc.
 *
 * Copyright (C) 2000, 2001 Nicolas Pitre <nico@cam.org>
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
 * 2004/08/12  Nishanth Menon - Modified to integrate Audio requirements on 
 *                              1610,1710 platforms
 */

#ifndef __OMAP_AUDIO_BUF_INTFC_H
#define __OMAP_AUDIO_BUF_INTFC_H

/************************** INCLUDES ******************************************/

/* Requires omap-audio.h */
#include "omap2-audio.h"

/************************** GLOBAL FUNCTIONS **********************************/

int audio_setup_buf(audio_state_t * state, audio_stream_t * s);
void audio_discard_buf(audio_state_t * state, audio_stream_t * s);
int audio_process_buf(audio_state_t * state, audio_stream_t * s);
void audio_prime_rx(audio_state_t * state);
int audio_set_fragments(audio_state_t * state, audio_stream_t * s, int val);
int audio_sync(audio_state_t * state, audio_stream_t * s);
u32 audio_get_buf_pos(audio_state_t * state, audio_stream_t * s);
void audio_reset(audio_state_t * state, audio_stream_t * s);
void audio_buf_irq_handler(audio_state_t * state, audio_stream_t * s);
int audio_timed_get_sem(struct semaphore *sem);

#endif				/* #ifndef __OMAP_AUDIO_BUF_INTFC_H */
