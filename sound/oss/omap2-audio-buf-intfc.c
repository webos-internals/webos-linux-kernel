/*
 * linux/drivers/sound/omap-audio-buf-intfc.c
 *
 * Common audio Buffer handling for the OMAP processors
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
 * 2004-06-07	Sriram Kannan	- Created new file from omap_audio_buf_intfc.c.
 *                                This file will contain only the DMA interface
 *                                and buffer handling of OMAP audio driver.
 * 2004-06-22	Sriram Kannan	- removed legacy code (auto-init). 
 *                                Self-linking of DMA logical channel.
 * 2004-08-12   Nishanth Menon  - Modified to integrate Audio requirements 
 *                                on 1610,1710 platforms
 * 2004-11-01   Nishanth Menon  - 16xx platform code base modified to 
 *                                support multi channel chaining.
 * 2004-12-15   Nishanth Menon  - Improved 16xx platform channel logic 
 *                                introduced - tasklets, queue handling updated
 * 2005-03-31   Nishanth Menon  - Integrated 24xx section to work done in 
 *                                opensource.
 * 2005-12-27   Nishanth Menon  - Removed dependency of buf from code-
 *                                this code will now be present in codec and lower layer
 *                                this will handle buffer handling alone
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/sysrq.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>

#include "omap2-audio-buf-intfc.h"

/* Change to define if required */
#undef DEBUG

#ifdef DEBUG
#define DPRINTK(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#define FN_IN printk(KERN_INFO "[%s]: start\n", __FUNCTION__)
#define FN_OUT(n) printk(KERN_INFO "[%s]: end(%u)\n",__FUNCTION__, n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(x)
#endif
#define ERR(ARGS...) printk(KERN_ERR "{%s}-ERROR: ", __FUNCTION__);printk(ARGS);

/* wait for 8 Sec
 * sample rate of 8000, (1/8000) * (64000/2) sec
 */
#define MAX_WAIT_TIMEOUT 90
/* Wait for 1 buffer */
#define TIMEOUT_TIME 10

/************************ DATA STRUCTURES *************************************/

struct audio_dsr_work_item {
	audio_stream_t *s;
	audio_state_t *state;
};

static u8 work_item_running = 0;
static struct audio_dsr_work_item work1, work2;

/*************************** MODULE SPECIFIC FUNCTIONS PROTOTYPES *************/

static void audio_dsr_handler(unsigned long);

DECLARE_TASKLET(audio_isr_work1, audio_dsr_handler, (unsigned long)&work1);
DECLARE_TASKLET(audio_isr_work2, audio_dsr_handler, (unsigned long)&work2);

/************************** GLOBAL FUNCTIONS DEFINTIONS ***********************/

/** 
 * @brief audio_timed_get_sem - this function guarentees a timeout duration 
 *  before which a semaphore is expected to be got.
 *  
 * @param sem the semaphore to wait on
 * 
 * @return -ETIMEDOUT if timedout, 0 on success
 */
int audio_timed_get_sem(struct semaphore *sem)
{
	int counter = 0;
	FN_IN;
	DPRINTK("aCurrent Jiffies=%d\n", jiffies);
	while (counter < MAX_WAIT_TIMEOUT) {
		if (!down_trylock(sem)) {
			DPRINTK("GNow Jiffies=%d\n", jiffies);
			DPRINTK("Got Sem\n");
			return 0;
		}
		counter++;
		if (signal_pending(current)) {
			/* will not wait at all if a signal is already pending!! */
			set_current_state(TASK_UNINTERRUPTIBLE);
		} else {
			set_current_state(TASK_INTERRUPTIBLE);
		}
		schedule_timeout(TIMEOUT_TIME);
		set_current_state(TASK_RUNNING);
	}
	DPRINTK("aNow Jiffies=%d\n", jiffies);
	DPRINTK("TIMEDOUT\n");
	FN_OUT(-ETIMEDOUT);
	return -ETIMEDOUT;
}

/*******************************************************************************
 *
 * Buffer creation/destruction
 *
 ******************************************************************************/

/** 
 * @brief audio_setup_buf - create a buffer for a stream
 * 
 * @param state - the codec state structure
 * @param s - the stream
 * 
 * @return 0 if success, else return error code
 */
int audio_setup_buf(audio_state_t * state, audio_stream_t * s)
{
	int frag;
	int bufsize = 0;
	char *bufbuf = NULL;
	dma_addr_t bufphys = 0;

	FN_IN;
	if (s->buffers) {
		FN_OUT(1);
		return -EBUSY;
	}
	s->buffers = kmalloc(sizeof(audio_buf_t) * s->nbfrags, GFP_KERNEL);
	if (!s->buffers)
		goto err;
	memset(s->buffers, 0, sizeof(audio_buf_t) * s->nbfrags);
	for (frag = 0; frag < s->nbfrags; frag++) {
		audio_buf_t *b = &s->buffers[frag];

		/*
		 * Let's allocate non-cached memory for DMA buffers.
		 * We try to allocate all memory at once.
		 * If this fails (a common reason is memory fragmentation),
		 * then we allocate more smaller buffers.
		 */
		if (!bufsize) {
			bufsize = (s->nbfrags - frag) * s->fragsize;
			do {
				bufbuf = dma_alloc_coherent(NULL,
					bufsize,
					&bufphys,
					GFP_KERNEL | GFP_DMA);

				if (!bufbuf)
					bufsize -= s->fragsize;
			}
			while (!bufbuf && bufsize);
			if (!bufbuf)
				goto err;
			b->master = bufsize;
			memzero(bufbuf, bufsize);
		}
		b->data = bufbuf;
		b->buf_addr = bufphys;
		bufbuf += s->fragsize;
		bufphys += s->fragsize;
		bufsize -= s->fragsize;
	}
	s->bytecount = 0;
	s->fragcount = 0;
	sema_init(&s->sem, s->nbfrags);
	FN_OUT(0);
	return 0;
      err:
	audio_discard_buf(state, s);
	FN_OUT(-ENOMEM);
	return -ENOMEM;
}

/** 
 * @brief audio_discard_buf - destroy the buffers for a stream
 * 
 * @param state - the codec state structure
 * @param s - the stream
 */
void audio_discard_buf(audio_state_t * state, audio_stream_t * s)
{
	FN_IN;
	/* ensure DMA isn't using those buffers */
	audio_reset(state, s);
	if (s->buffers) {
		int frag;
		for (frag = 0; frag < s->nbfrags; frag++) {
			if (!s->buffers[frag].master)
				continue;
			dma_free_coherent(NULL,
				s->buffers[frag].master,
				s->buffers[frag].data,
				s->buffers[frag].buf_addr);
		}
		kfree(s->buffers);
		s->buffers = NULL;
	}
	FN_OUT(0);
}

/** 
 * @brief audio_process_buf - This will end up starting the transfer. Proper 
 *                        fragments of Transfers will be initiated.
 * 
 * @param state - the codec state structure
 * @param s - the stream
 * 
 * @return 0 on success else the error code
 */
int audio_process_buf(audio_state_t * state, audio_stream_t * s)
{
	int ret = 0;
	unsigned long flags;

	FN_IN;
	if (unlikely((!state) || (!s))) {
		ERR("Null Params.. Codec=%p s=%p\n", state, s);
		return -EPERM;
	}

	/* Dont let the ISR over ride touching the in_use flag */
	local_irq_save(flags);
	if (1 == s->in_use) {
		local_irq_restore(flags);
		/* Warn alone, we can handle this!! */
		DPRINTK("Called again while In Use \n");
		return 0;
	}
	s->in_use = 1;
	local_irq_restore(flags);

	if (s->stopped) {
		/* Warn alone, we can handle this!! */
		DPRINTK("Transfer is already stopped.\n");
		s->in_use = 0;
		return 0;
	}

	while (s->pending_frags) {
		audio_buf_t *b = &s->buffers[s->buf_head];
		u32 buf_size = s->fragsize - b->offset;
		DPRINTK("buf_size=%d, fragsize=%d, offset=%d\n", buf_size,
			s->fragsize, b->offset);
#ifndef AUDIO_DRAIN_ON_SUSPEND
		/* if we are suspended - dont process any more buffers */
#ifdef CONFIG_PM
		if (state->suspended) {
			goto process_out;
		}
#endif
#endif
		if (buf_size) {
			ret =
			    state->hw_transfer(state, s,
					       (void *)(b->buf_addr +
							b->offset), buf_size);
		}
		/* Do not continue and move the frags forward..
		 * the completion of the next transfer will put it thru..
		 */
		if (ret == -EBUSY) {
			ret = 0;
			DPRINTK("TIMEDOUT!!");
			goto process_out;
		}
		if (ret) {
			printk(KERN_ERR "Transfer Failed.[%d][%d]\n", ret,
			       buf_size);
			goto process_out;
		}
		b->buf_ref++;
		b->offset += buf_size;
		if (b->offset >= s->fragsize) {
			s->pending_frags--;
			if (++s->buf_head >= s->nbfrags)
				s->buf_head = 0;
		}
	}

      process_out:
	s->in_use = 0;

	FN_OUT(ret);
	return ret;
}

/** 
 * @brief audio_prime_rx - Prime Rx - 
 *            Since the recieve buffer has no time limit as to when it would 
 *            arrive, we need to prime it 
 * @param state - the codec state
 */
void audio_prime_rx(audio_state_t * state)
{
	audio_stream_t *is = state->input_stream;

	FN_IN;
	if (state->need_tx_for_rx) {
		/*
		 * With some codecs like the Philips UDA1341 we must ensure
		 * there is an output stream at any time while recording since
		 * this is how the UDA1341 gets its clock from the SA1100.
		 * So while there is no playback data to send, the output DMA
		 * would have already been stopped.
		 */
		audio_process_buf(state, state->output_stream);
	}
	is->pending_frags = is->nbfrags;
	sema_init(&is->sem, 0);
	is->active = 1;
	audio_process_buf(state, is);

	FN_OUT(0);
	return;
}

/** 
 * @brief audio_set_fragments - set the fragment size
 * 
 * @param state - the state structure of the codec
 * @param s - the stream
 * @param val - set it to?  [from OSS specification]
 *              The argument to this call is an integer encoded as 0xMMMMSSSS (in hex).
 *              The 16 least significant bits determine the fragment size. 
 *              The size is 2^SSSS. For example SSSS=0008 gives fragment size 
 *              of 256 bytes (2^8). The minimum is 16 bytes (SSSS=4) and the maximum 
 *              is total_buffer_size/2.  Some devices or processor architectures may 
 *              require larger fragments - in this case the requested fragment size 
 *              is automatically increased.
 * @return set value in the same format, err value if failed
 */
int audio_set_fragments(audio_state_t * state, audio_stream_t * s, int val)
{
	int old_frag_size = s->fragsize;
	int old_num_frags = s->nbfrags;
	FN_IN;
	/* cant change buffer sizes when stream is active */
	if (s->active)
		return -EBUSY;
	if (s->buffers)
		audio_discard_buf(state, s);
	/* higher 16 bits contain number of frags */
	s->nbfrags = (val >> 16) & 0x7FFF;
	/* lower 16 bits contain size of each frag */
	val &= 0xFFFF;
	/* We now need to re-adjust the val for the ideal values */
	if (val < 4)
		val = 4;
	if (val > 15)
		val = 15;
	/* Fragsize is 2^val size */
	s->fragsize = 1 << val;
	/* Check min */
	if (s->nbfrags < 2)
		s->nbfrags = 2;	/* min 2 byte chunks */
	/* Check Max - we dont want more than 128Kb kernel space memory allocated!! */
	if (s->nbfrags * s->fragsize > 128 * 1024)
		s->nbfrags = 128 * 1024 / s->fragsize;
	FN_OUT(0);
	if (audio_setup_buf(state, s)) {
		/* reset the old params back */
		s->nbfrags = old_num_frags;
		s->fragsize = old_frag_size;
		return -ENOMEM;
	}
	/* return back the values we decided on.. */
	return val | (s->nbfrags << 16);

}

/** 
 * @brief audio_sync - Sync up the buffers before we shutdown, 
 *                     else under-run errors will happen
 * @param state - the codec state
 * @param s - stream
 * 
 * @return - 0 on success, error on failure
 */
int audio_sync(audio_state_t * state, audio_stream_t * s)
{
#ifdef SHIFT_HACK
	audio_buf_t *b;
	u32 shiftval = 0;
	unsigned long flags;
#endif
	int counter = 0;

	DECLARE_WAITQUEUE(wait, current);

	FN_IN;

	if (!s->buffers || s->mapped) {
		FN_OUT(1);
		return 0;
	}

	/* Let's wait for all buffers to complete */
	set_current_state(TASK_INTERRUPTIBLE);
	add_wait_queue(&s->wq, &wait);
	/* Some one to give me buffer please?? */
	while ((s->pending_frags || (atomic_read(&s->sem.count) < s->nbfrags))
	       && !signal_pending(current)) {
		schedule_timeout(TIMEOUT_TIME);
		if (counter++ > MAX_WAIT_TIMEOUT) {
			break;
		}
		set_current_state(TASK_INTERRUPTIBLE);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&s->wq, &wait);
	if (counter >= MAX_WAIT_TIMEOUT) {
		audio_reset(state, s);
		return -ETIMEDOUT;
	}
#ifdef SHIFT_HACK
	/* undo the pointer hack above */
	if (shiftval) {
		local_irq_save(flags);
		b->buf_addr += shiftval;
		b->data += shiftval;
		/* ensure sane DMA code behavior if not yet processed */
		if (b->offset != 0)
			b->offset = s->fragsize;
		local_irq_restore(flags);
	}
#endif
	FN_OUT(0);
	return 0;
}

/** 
 * @brief audio_get_buf_pos - Get the buf posn
 * 
 * @param state - the codec state structure
 * @param s - stream
 * 
 * @return offset
 */
u32 audio_get_buf_pos(audio_state_t * state, audio_stream_t * s)
{
	audio_buf_t *b = &s->buffers[s->buf_tail];
	u32 offset;

	FN_IN;
	if (b->buf_ref) {
		offset = state->hw_transfer_posn(s) - b->buf_addr;
		if (offset >= s->fragsize)
			offset = s->fragsize - 4;
	} else if (s->pending_frags) {
		offset = b->offset;
	} else {
		offset = 0;
	}
	FN_OUT(offset);
	return offset;
}

/** 
 * @brief audio_reset - Reset the audio buffers
 * 
 * @param state - the codec state structure
 * @param s - the stream
 */
void audio_reset(audio_state_t * state, audio_stream_t * s)
{
	int frag;
	FN_IN;
	if (s->buffers) {
		state->hw_transfer_stop(s);
		if (s->input_or_output == FMODE_READ) {
			sema_init(&s->sem, 0);
			s->pending_frags = s->nbfrags;
		} else {
			sema_init(&s->sem, s->nbfrags);
			s->pending_frags = 0;
		}
		for (frag = 0; frag < s->nbfrags; frag++) {
			audio_buf_t *b = &s->buffers[frag];
			b->offset = 0;
			b->buf_ref = 0;
		}
	}
	s->active = 0;
	s->stopped = 0;
	s->fragcount = 0;
	s->bytecount = 0;
	s->buf_tail = 0;
	s->buf_head = 0;
	s->usr_head = 0;
	FN_OUT(0);
	return;
}

/************************** MODULE FUNCTIONS DEFINTIONS ***********************/

/*******************************************************************************
 *
 * ISR related functions
 *
 ******************************************************************************/
/** 
 * @brief audio_dsr_handler work item handler
 * 
 * @param data - param
 */
static void audio_dsr_handler(unsigned long data)
{
	struct audio_dsr_work_item *work = (struct audio_dsr_work_item *)data;
	audio_stream_t *s = (work->s);
	audio_state_t *state = (work->state);
	audio_buf_t *b = &s->buffers[s->buf_tail];

	FN_IN;
	/* Try to fill again */
	if (!s->buffers) {
		printk(KERN_CRIT "omap-audio:"
		       "received DMA IRQ for non existent buffers!\n");
		return;
	} else if (b->buf_ref && --b->buf_ref == 0 && b->offset >= s->fragsize) {
		/* This fragment is done */
		b->offset = 0;
		s->bytecount += s->fragsize;
		s->fragcount++;

		if (++s->buf_tail >= s->nbfrags)
			s->buf_tail = 0;

		if (!s->mapped)
			up(&s->sem);
		else
			s->pending_frags++;

		wake_up(&s->wq);
	}

	/* Empty the rest of the buffers */
#ifdef AUDIO_DRAIN_ON_SUSPEND
	audio_process_buf(state, s);
#else
	/* if suspended - dont process the next set of buffers */
#ifdef CONFIG_PM
	if (!state->suspended)
#endif
	{
		audio_process_buf(state, s);
	}
#endif

	FN_OUT(0);
}

/** 
 * @brief audio_buf_irq_handler - ISRs have to be short and smart.. 
 *             So we transfer every heavy duty stuff to the work item
 *             IMPNOTE: we handle only 2 callbacks at a time. no more 
 *             is to be generated by transfer drivers
 * 
 * @param state - the codec state structure
 * @param s - the stream
 */
void audio_buf_irq_handler(audio_state_t * state, audio_stream_t * s)
{
	FN_IN;
	DPRINTK("state=%p stream=%p\n", state, s);

	/* Start the High priority tasklets - we ping pong the tasklets */
	if (!work_item_running) {
		work1.s = s;
		work1.state = state;
		/* schedule tasklet 1 */
		tasklet_hi_schedule(&audio_isr_work1);
		work_item_running = 1;
	} else {
		work2.s = s;
		work2.state = state;
		/* schedule tasklet 2 */
		tasklet_hi_schedule(&audio_isr_work2);
		work_item_running = 0;
	}

	FN_OUT(0);
	return;
}

EXPORT_SYMBOL(audio_setup_buf);
EXPORT_SYMBOL(audio_process_buf);
EXPORT_SYMBOL(audio_prime_rx);
EXPORT_SYMBOL(audio_set_fragments);
EXPORT_SYMBOL(audio_sync);
EXPORT_SYMBOL(audio_get_buf_pos);
EXPORT_SYMBOL(audio_reset);
EXPORT_SYMBOL(audio_discard_buf);
EXPORT_SYMBOL(audio_buf_irq_handler);
EXPORT_SYMBOL(audio_timed_get_sem);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Common Buffer Logic for Audio driver on OMAP processors");
MODULE_LICENSE("GPL");
