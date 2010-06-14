/*
 * drivers/media/video/omap/omap34xxcam.h
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
 *
 * Video-for-Linux (Version 2) camera capture driver for OMAP34xx ISP.
 * Leverage omap24xx camera driver
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2006 Texas Instruments.
 *
 */

#ifndef OMAP34XXCAM_H
#define OMAP34XXCAM_H

#define DMA_THRESHOLD 32

#define NUM_SG_DMA (VIDEO_MAX_FRAME+2)

#define OMAP_ISP_CCDC    	(1 << 0)
#define OMAP_ISP_PREVIEW	(1 << 1)
#define OMAP_ISP_RESIZER	(1 << 2)
#define OMAP_ISP_AEWB   	(1 << 3)
#define OMAP_ISP_AF     	(1 << 4)
#define OMAP_ISP_HIST   	(1 << 5)

struct sgdma_state {
	dma_addr_t isp_addr;    /* ISP space addr */ 
	unsigned long status;	/* DMA return code */
	isp_callback_t callback;
	void *arg;
};

/* per-device data structure */
struct omap34xxcam_device {
	enum isp_interface_type if_type;
	
	unsigned long overlay_base;
	unsigned long overlay_base_phys;
	unsigned long overlay_size;
	unsigned long overlay_base_dma;
	int overlay_rotation;

	/* mapped ISP mmu addrs */
	dma_addr_t isp_addr_overlay;
	dma_addr_t isp_addr_read;
	dma_addr_t isp_addr_capture[VIDEO_MAX_FRAME];

	unsigned int isp_pipeline;   /* bit masks of pipeline usage */

	struct device dev;
	struct video_device *vfd;

	spinlock_t overlay_lock;	/* spinlock for overlay DMA counter */
	int overlay_cnt;		/* count of queued overlay DMA xfers */
	struct scatterlist overlay_sglist;

	spinlock_t vbq_lock;		/* spinlock for videobuf queues */
	struct videobuf_queue_ops vbq_ops;	/* videobuf queue operations */
	unsigned long field_count;	/* field counter for videobuf_buffer */

	/* scatter-gather DMA management */
	spinlock_t sg_lock;
	int free_sgdma;	/* number of free sg dma slots */
	int next_sgdma;	/* index of next sg dma slot to use */
	struct sgdma_state sgdma[NUM_SG_DMA];

	/* dma_notify is a flag to indicate a DMA transfer has been started.
	 */
	int dma_notify;

	/* The img_lock is used to serialize access to the image parameters for 
	 * overlay and capture.
	 */
	spinlock_t img_lock;

 	/* Access to everything below here is locked by img_lock */
 	
 	/* We allow streaming from at most one filehandle at a time.  
 	 * non-NULL means streaming is in progress.
 	 */
	struct omap34xxcam_fh *streaming;
	/* We allow previewing from at most one filehandle at a time.  
	 * non-NULL means previewing is in progress.
	 */
	struct omap34xxcam_fh *previewing;

	/* capture parameters (frame rate, number of buffers) */
	struct v4l2_captureparm cparm;
	struct v4l2_captureparm cparm2;

	/* This is the frame period actually requested by the user. */
	struct v4l2_fract nominal_timeperframe;

	/* frequency (in Hz) of camera interface xclk output */
	unsigned long xclk;

	/* pointer to camera sensor interface interface */
	struct camera_sensor *cam_sensor;
	/* blind pointer to private data structure for sensor */
	void *sensor;
	
	/* tried ISP output sizes for video mode */
	unsigned long ccdc_input_width;
	unsigned long ccdc_input_height;
	unsigned long ccdc_width;
	unsigned long ccdc_height;
	unsigned long preview_width;
	unsigned long preview_height;
	unsigned long resizer_width;
	unsigned long resizer_height;
	/* tried ISP output sizes for image mode */
	unsigned long ccdc_input_width2;
	unsigned long ccdc_input_height2;
	unsigned long ccdc_width2;
	unsigned long ccdc_height2;
	unsigned long preview_width2;
	unsigned long preview_height2;
	unsigned long resizer_width2;
	unsigned long resizer_height2;

	/* pix defines the size and pixel format of the image captured by the 
	 * sensor.  This also defines the size of the framebuffers.  The 
	 * same pool of framebuffers is used for video capture and video 
	 * overlay.  These parameters are set/queried by the 
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with a CAPTURE buffer type.
	 */
	struct v4l2_pix_format pix, pix_raw;
	struct v4l2_pix_format pix2, pix2_raw;
	int still_capture;

	/* preview_crop defines the size and offset of the video overlay source window 
	 * within the framebuffer.  These parameters are set/queried by the 
	 * VIDIOC_S_CROP/VIDIOC_G_CROP ioctls with an OVERLAY buffer type.  
	 * The cropping rectangle allows a subset of the captured image to be 
	 * previewed.  It only affects the portion of the image previewed, not 
	 * captured; the cropping of the captured image is done seperately.
	 */
	struct v4l2_rect preview_crop;

	/* win defines the size and offset of the video overlay target window 
	 * within the video display.  These parameters are set/queried by the 
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with an OVERLAY buffer type.
	 */
	struct v4l2_window win;

	/* fbuf reflects the size of the video display.  It is queried with the 
	 * VIDIOC_G_FBUF ioctl.  The size of the video display cannot be 
	 * changed with the VIDIOC_S_FBUF ioctl.
	 */
	struct v4l2_framebuffer fbuf;

	/* the display controller video layer for camera preview
	 */
	int vid_preview;

	/* Power management suspend lockout */
	int suspended;
	wait_queue_head_t suspend_wq;

};

/* per-filehandle data structure */
struct omap34xxcam_fh {
	struct omap34xxcam_device *cam;
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
};

#endif	/* ifndef OMAP34XXCAM_H */
