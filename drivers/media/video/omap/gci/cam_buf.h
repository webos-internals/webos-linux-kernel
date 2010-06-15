/*
 * drivers/media/video/omap/gci/cam_buf.h
 *
 * Header file for Camera buffer managment
 * for TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP34XXCAM_H
#define OMAP34XXCAM_H

#define DMA_THRESHOLD 32

#define NUM_SG_DMA (VIDEO_MAX_FRAME+2)

typedef void (*callback_t) (unsigned long status, void *arg1, void *arg2);

/* Defining the same callback structure as Camera configuration module */
struct sgdma_state{
	dma_addr_t isp_addr;    /* ISP mmu space addr */ 
	unsigned long status;	/* DMA return code */
	callback_t callback;
	void *arg1;
	void *arg2;
};

/* per-device data structure */
struct omap34xxcam_buf {

	struct device dev;
	struct video_device *vfd;

	/* mapped ISP mmu addrs */
	dma_addr_t isp_addr_capture[VIDEO_MAX_FRAME];

	spinlock_t vbq_lock;		/* spinlock for videobuf queues */
	struct videobuf_queue_ops vbq_ops;	/* videobuf queue operations */
	unsigned long field_count;	/* field counter for videobuf_buffer */

	/* scatter-gather DMA management */
	spinlock_t sg_lock;
	int free_sgdma;	/* number of free sg dma slots */
	int next_sgdma;	/* index of next sg dma slot to use */
	struct sgdma_state sgdma[NUM_SG_DMA];

	/* A flag to indicate a DMA transfer has been started.*/
	int dma_notify;

	
	/* We allow streaming from at most one filehandle at a time.  
	 * non-NULL means streaming is in progress.
	 */
	struct omap34xxcam_fh *streaming;
	/* These parameters are set/queried by the 
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with a CAPTURE buffer type.
	 */
	struct v4l2_pix_format pix, pix_raw;

	/* Power management suspend lockout */
	int suspended;
	wait_queue_head_t suspend_wq;

};

/* per-filehandle data structure */
struct omap34xxcam_fh {
	struct omap34xxcam_buf *cam;
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
};

#endif	/* ifndef OMAP34XXCAM_H */
