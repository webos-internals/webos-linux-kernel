/*
 * linux/drivers/sound/omap-audio2.h
 *
 * Common audio handling for the OMAP processors
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
 *  History
 *  -------
 *  2004/08/12 Nishanth Menon - Modified to integrate Audio requirements on 
 *                              1610,1710 platforms
 *  2004/04/04 Nishanth menon - Added hooks for power management
 */

#ifndef __OMAP_AUDIO_H
#define __OMAP_AUDIO_H

/* Requires dma.h -dma-able buffers...*/
#include <asm/arch/dma.h>


#ifdef  CONFIG_PM
#define audio_suspend_lockout(s,f) \
	if ((s)->suspended) {\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		DPRINTK("%s[%d]: Waiting for suspend completion\n",__FUNCTION__,__LINE__);\
		wait_event((s)->suspend_wq,\
		(s)->suspended == 0);\
		DPRINTK("%s[%d]: wait done (susp=%d)\n",__FUNCTION__,__LINE__,s->suspended);\
	}
#else
#define audio_suspend_lockout(s, f) do {} while(0)
#endif

/*
 * Buffer Management
 */
typedef struct {
	int offset;		/* current offset */
	char *data;		/* points to actual buffer */
	dma_addr_t buf_addr;	/* physical buffer address */
	int buf_ref;		/* DMA refcount - we do not know how many buffers can s/w take. */
	int master;		/* owner for buffer allocation, 
				 * contain size when true */
} audio_buf_t;

/*
 * Structure describing the data stream related information
 */
typedef struct {
	char *id;		/* identification string */
	audio_buf_t *buffers;	/* pointer to audio buffer structures */
	u32 usr_head;		/* user fragment index */
	u32 buf_head;		/* BUF fragment index to go */
	u32 buf_tail;		/* BUF fragment index to complete */
	u32 fragsize;		/* fragment i.e. buffer size */
	u32 nbfrags;		/* nbr of fragments i.e. buffers */
	u32 pending_frags;	/* Fragments sent to BUF */
	u8 in_use;		/*  Is this is use? */
	int *lch;		/*  Chain of channels this stream is linked to */
	int input_or_output;	/* Direction of this data stream */
	int bytecount;		/* nbr of processed bytes */
	int fragcount;		/* nbr of fragment transitions */
	struct semaphore sem;	/* account for fragment usage */
	wait_queue_head_t wq;	/* for poll */
	int mapped:1;		/* mmap()'ed buffers */
	int active:1;		/* actually in progress */
	int stopped:1;		/* might be active but stopped */
} audio_stream_t;

/* Forward declaration */
struct audio_state_s;

/* On completion of a transfer, the codec driver should call this - 
 * matching the state and stream given during hw_transfer */
typedef void (*buf_irq_handler) (struct audio_state_s * state,
				 audio_stream_t * s);

/*
 * State structure for one instance
 */
typedef struct audio_state_s {
	struct module *owner;	/* Codec module ID */
	audio_stream_t *output_stream;
	audio_stream_t *input_stream;
	int need_tx_for_rx:1;	/* if data must be sent while receiving */
	void *data;
	int fmode;              /* File Open Mode */
	struct pm_dev *pm_dev;
	struct semaphore sem;	/* to protect against races in attach() */
	u8 use_ref; /* Is this codec already under use?? */
	u8 user_sync; /* user controlled sync? */

	/* Power management suspend lockout stuff */
#ifdef CONFIG_PM
	int suspended;
	wait_queue_head_t suspend_wq;
#endif
	/* Data transfer APIs 
	 * Codec Drivers Should *not Modify* any of the provided pointers 
	 * Codecs need to figure out based on stream directions and handle the 
	 * directions accordingly 
	 */
	int (*hw_transfer) (struct audio_state_s * state, audio_stream_t * s,
			    void *buffer_phy, u32 size);
	int (*hw_transfer_stop) (audio_stream_t * s);
	int (*hw_transfer_posn) (audio_stream_t * s);
	int (*hw_transfer_init) (struct audio_state_s * state, audio_stream_t * s, buf_irq_handler callback);
	/* Codec functionality APIs */
	int (*hw_init) (struct audio_state_s * state, void *);
	void (*hw_shutdown) (void *);
/* Borrowed from http://www.dtek.chalmers.se/groups/dvd/dist/oss_audio.c */
/* AFMT_AC3 is really IEC61937 / IEC60958, mpeg/ac3/dts over spdif */
#ifndef AFMT_AC3
#define AFMT_AC3        0x00000400      /* Dolby Digital AC3 */
#endif
#ifndef AFMT_S32_LE
#define AFMT_S32_LE     0x00001000  /* 32/24-bits, in 24bit use the msbs */ 
#endif
#ifndef AFMT_S32_BE
#define AFMT_S32_BE     0x00002000  /* 32/24-bits, in 24bit use the msbs */ 
#endif
	int (*client_ioctl) (struct inode *, struct file *, uint, ulong);
	int (*hw_probe) (void);
	void (*hw_remove) (void);
	void (*hw_cleanup) (void);
	int (*hw_suspend) (struct audio_state_s * state);
	int (*hw_resume) (struct audio_state_s * state);
	int (*hw_sidle) (struct audio_state_s * state, u32 idle_state);
	/* frequency scaling stuff */
	int (*hw_prescale) (void);
	int (*hw_postscale) (void);


} audio_state_t;

extern int audio_register_codec(audio_state_t * codec_state);
extern int audio_unregister_codec(audio_state_t * codec_state);

/* Drain Buffers on Suspend/exit ?
 * Implies that the driver, instead of dumping the last filled buffers
 * will attempt to wait till they are played before proceed.
 * By default - we are not interested in this.change to define if required
 */
#undef AUDIO_DRAIN_ON_SUSPEND

#endif				/* End of #ifndef __OMAP_AUDIO_H */
