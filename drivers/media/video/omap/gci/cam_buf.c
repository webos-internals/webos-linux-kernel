/*
 * drivers/media/video/omap/gci/cam_buf.c
 *
 * Camera buffer data managment
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/videodev.h>
#include <media/video-buf.h>
#include <media/v4l2-dev.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <asm/arch/display.h>
#include <asm/arch/clock.h>
#include <linux/platform_device.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/scatterlist.h>
#include <asm/irq.h>
#include <asm/semaphore.h>

#undef CONFIG_PM

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#include "../omap24xxcam_user.h"


#include "cam_buf.h"
#include "cam_config.h"

#define QVGA_SIZE 	320 * 240 * 2
#define QXGA_SIZE 	2048 * 1536 * 2	/* memory needed for QXGA image */
#define QCIF_WIDTH 	176
#define QCIF_HEIGHT 	144


#define ISP_BUF_INIT		0
#define ISP_FREE_RUNNING	1
#define ISP_BUF_TRAN		2


#if 0
#define DPRINTK_CAMBUF(format,...)\
	printk(KERN_INFO "CAMBUF: " format, ## __VA_ARGS__)
#else
#define DPRINTK_CAMBUF(format,...)
#endif

#define CAM_NAME "omap34xxcam"


/* global variables */
static struct omap34xxcam_buf *saved_cam;

/* module parameters */
static int video_nr = 0;	/* video device minor (-1 ==> auto assign) */

/* Maximum amount of memory to use for capture buffers.
 * Default is 4800KB, enough to double-buffer QXGA. */
static int capture_mem = QXGA_SIZE * 4;

static int isp_state = 0;

/* -------------------------------------------------------------------------- */
#ifdef CONFIG_PM
#define omap34xxcam_suspend_lockout(s,f) \
	if ((s)->suspended) {\
		int err;\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		err = wait_event_interruptible((s)->suspend_wq,\
					(s)->suspended == 0);\
		if (err < 0)\
			return err;\
	}
#else
#define omap34xxcam_suspend_lockout(s,f) s=s
#endif



/*
 * Process the scatter-gather DMA queue by starting queued transfers.
 * This can be called from either a process context or an IRQ context.
 * 	case 1: from a process,.
 * 	case 2: from a process,.
 * 	case 3: from IRQ,.
 * We make sure updating buffer pointer happens only once on the
 * shadow register,
 */
static void
omap34xxcam_sg_dma_process(struct omap34xxcam_buf *cam, int irq)
{
	struct sgdma_state *sgdma;
	unsigned long irqflags;
	DPRINTK_CAMBUF("+ sg_dma_process\n");
	spin_lock_irqsave(&cam->sg_lock, irqflags);
	/* any pending sgdma ?
	 * we can at most start or queue one sgdma */
	if ((NUM_SG_DMA - cam->free_sgdma) > 0) {
		/* get the next sgdma */
		sgdma = cam->sgdma +
			(cam->next_sgdma + cam->free_sgdma) % NUM_SG_DMA;
		if (!irq) {
			if (cam->dma_notify) {
				/* case 1: queue & start. */
				camcfg_set_callback((void*)sgdma);
				camcfg_set_curr_buf(sgdma->isp_addr);
				isp_state = camcfg_useispbuf(ISP_BUF_TRAN);
				cam->dma_notify = 0;
			}
			else {
				/* case 2:
				 * defer queuing until the current frame done
				 * assuming we don't miss the vsync of the
				 * next frame. */
				if(isp_state == ISP_FREE_RUNNING){
					camcfg_set_callback((void*)sgdma); 
					camcfg_set_curr_buf(sgdma->isp_addr);
					isp_state = camcfg_useispbuf
						(ISP_BUF_TRAN);
				}
			}
		}
		else {
			/* case 3:
			 * only need to queue (update buf ptr). */
			camcfg_set_callback((void*)sgdma);
			camcfg_set_curr_buf(sgdma->isp_addr);
			isp_state = camcfg_useispbuf(ISP_BUF_INIT);
			if (cam->dma_notify) {
				cam->dma_notify = 0;
			}
		}
	}
	else {
		isp_state = camcfg_useispbuf(ISP_FREE_RUNNING);
		//svcam->dma_notify = 1;
	}

	spin_unlock_irqrestore(&cam->sg_lock, irqflags);

	DPRINTK_CAMBUF("- sg_dma_process\n");
}

/* Queue a scatter-gather DMA transfer from the camera to memory.
 * Returns zero if the transfer was successfully queued, or
 * non-zero if all of the scatter-gather slots are already in use.
 */
static int
omap34xxcam_sg_dma_queue(struct omap34xxcam_buf *cam, dma_addr_t isp_addr,
			 isp_callback_t callback, void *arg, int irq)
{
	unsigned long irqflags;
	struct sgdma_state *sgdma;
	DPRINTK_CAMBUF("+ omap34xxcam_sg_dma_queue\n");
	spin_lock_irqsave(&cam->sg_lock, irqflags);

	if (!cam->free_sgdma) {
		spin_unlock_irqrestore(&cam->sg_lock, irqflags);
		return -EBUSY;
	}

	sgdma = cam->sgdma + cam->next_sgdma;

	sgdma->isp_addr = isp_addr;
	sgdma->status = 0;
	sgdma->callback = callback;
	sgdma->arg1 = cam;
	sgdma->arg2 = arg;

	cam->next_sgdma = (cam->next_sgdma + 1) % NUM_SG_DMA;
	cam->free_sgdma--;
	
	spin_unlock_irqrestore(&cam->sg_lock, irqflags);

	omap34xxcam_sg_dma_process(cam, irq);
	DPRINTK_CAMBUF("- omap34xxcam_sg_dma_queue\n");
	return 0;
}

/* Initialises the sgdma structure with NULL
 *  TBD See if sgdma could be removed or make NUM_SG_DMA 1
 */
static void
omap34xxcam_sgdma_init(struct omap34xxcam_buf *cam)
{
	int sg;

	cam->free_sgdma = NUM_SG_DMA;
	cam->next_sgdma = 0;
	for (sg = 0; sg < NUM_SG_DMA; sg++) {
		cam->sgdma[sg].status = 0;
		cam->sgdma[sg].callback = NULL;
		cam->sgdma[sg].arg1 = NULL;
		cam->sgdma[sg].arg2 = NULL;
	}
	spin_lock_init(&cam->sg_lock);
}


/* This routine is called from interrupt context when a scatter-gather DMA
 * transfer of a videobuf_buffer completes.
 */
static void
omap34xxcam_vbq_complete(unsigned long status, void *arg1, void *arg2)
{
	struct omap34xxcam_buf *cam = (struct omap34xxcam_buf *) arg1;
	struct videobuf_buffer *vb = (struct videobuf_buffer *) arg2;
	cam->free_sgdma++;
	if(cam->free_sgdma > NUM_SG_DMA)
		cam->free_sgdma = NUM_SG_DMA;

	spin_lock(&cam->vbq_lock);
	do_gettimeofday(&vb->ts);
	vb->field_count = cam->field_count;
	cam->field_count += 2;
	if (status){
		vb->state = STATE_ERROR;
		DPRINTK_CAMBUF("\tRECD ERROR!!!!!\n");
		//omap34xxcam_error_handler(cam);
	} else {
		vb->state = STATE_DONE;
		if (cam->streaming)
			omap34xxcam_sg_dma_process(cam, 1);
	}
	wake_up(&vb->done);
	spin_unlock(&cam->vbq_lock);
}

/* This routine called when stream off or when camera release is done
 * Unmaps buffer as camcfg_unmap and videobuf_unmap
 */
static void
omap34xxcam_vbq_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_buf *cam = fh->cam;
	DPRINTK_CAMBUF("+CAMBUF :omap34xxcam_vbq_release \n");
	/* TBD not efficient to call unset for every buf->release */
	camcfg_unset_callback();
	/* we already drain the queue so videobuf_waiton is no longer needed */
	videobuf_dma_unmap(q, &vb->dma);
	videobuf_dma_free(&vb->dma);

	/* video-buf calls us multiple times. need to make sure that we only
	 * unmap once */
	if (cam->isp_addr_capture[vb->i]) {
		//TBD Currently vbq_release is unmapping the buffers
		//when ISP is using it.Need to put this in proper place 
		//or need to do synchronise unset_callback with the isr
			camcfg_unmap_va_buf(cam->isp_addr_capture[vb->i]);
			cam->isp_addr_capture[vb->i] = 0;
		}

	vb->state = STATE_NEEDS_INIT;
	DPRINTK_CAMBUF("-CAMBUF :omap34xxcam_vbq_release \n");
}

/* Limit the number of available kernel image capture buffers based on the
 * number requested, the currently selected image size, and the maximum
 * amount of memory permitted for kernel capture buffers.
 */
static int
omap34xxcam_vbq_setup(struct videobuf_queue *q, unsigned int *cnt,
		unsigned int *size)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_buf *cam = fh->cam;

	if (*cnt <= 0)
	/* supply a default number of buffers */
		*cnt = VIDEO_MAX_FRAME;	
	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	*size = cam->pix.sizeimage;

	while (*size * *cnt > capture_mem)
		(*cnt)--;

	return 0;
}

/* Checks if the buffer size is enough for the image size to proceed.
 * Considers the scattergather list for image size and not for buffer size
 * Prepares the ispmmu address for the buffer by mapping with 
 * ispmmu and consistent_sync(dma)
 * Inits the vb members with size, state for videobuf to queue
 */
static int
omap34xxcam_vbq_prepare(struct videobuf_queue *q, struct videobuf_buffer *vb,
	enum v4l2_field field)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_buf *cam = fh->cam;
	unsigned int size, isp_addr;
	int err = 0;

	size = cam->pix.sizeimage;
	if (vb->baddr) {
		/* This is a userspace buffer. */
		if (size > vb->bsize) {
			/* The buffer isn't big enough. */
			err = -EINVAL;
		} else {
			vb->size = size;
			/* video-buf uses bsize (buffer size) instead of
			 * size (image size) to generate sg slist. Work
			 * around this bug by making bsize the same as
			 * size.
			 */
			vb->bsize = size;
		}
	} else {
		if (vb->state != STATE_NEEDS_INIT) {
			/* We have a kernel bounce buffer that has already been
			 * allocated.
			 */
			if (size > vb->size) {
				/* The image size has been changed to a larger
				 * size since this buffer was allocated, so we
				 * need to free and reallocate it.
				 */
				omap34xxcam_vbq_release(q, vb);
				vb->size = size;
			}
		} else {
			/* We need to allocate a new kernel bounce buffer. */
			vb->size = size;
		}
	}

	vb->width = cam->pix.width;
	vb->height = cam->pix.height;

	vb->field = field;

	if (err)
		return err;

	if (vb->state == STATE_NEEDS_INIT) {
		err = videobuf_iolock(q, vb, NULL);
		DPRINTK_CAMBUF("omap34xxcam_vbq_prepare: ret from\
				videobuf_ioclok %d\n", err);
		if (!err) {
			isp_addr = camcfg_map_va_buf(vb->dma.sglist,
					vb->dma.sglen);
			if (!isp_addr)
				err = -EIO;
			else {
				cam->isp_addr_capture[vb->i]= isp_addr;
			}
		}
	}

	if (!err) {
		vb->state = STATE_PREPARED;
		if (vb->baddr) {
			/* sync a userspace buffer */
			flush_cache_user_range(NULL, vb->baddr,
					(vb->baddr + vb->bsize));
		} else {
			/* sync a kernel buffer */
			dmac_flush_range(vb->dma.vmalloc,
					(vb->dma.vmalloc +
					 (vb->dma.nr_pages << PAGE_SHIFT)));
			outer_flush_range(__pa(vb->dma.vmalloc),
					__pa(vb->dma.vmalloc +
					(vb->dma.nr_pages << PAGE_SHIFT)));

		}
	} else
		omap34xxcam_vbq_release(q, vb);

	return err;
}

/* Calls sg_dma_queue to set the callback and buffer for getting the 
 * image frame. Sets the vb state to be queued.
 */
static void
omap34xxcam_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_buf *cam = fh->cam;
	enum videobuf_state state = vb->state;
	int err;
	vb->state = STATE_QUEUED;
	err = omap34xxcam_sg_dma_queue(cam,
			cam->isp_addr_capture[vb->i],
			omap34xxcam_vbq_complete, vb, 0);
	if (err) {
		/* Oops.  We're not supposed to get any errors here.  The only
		 * way we could get an error is if we ran out of scatter-gather
		 * DMA slots, but we are supposed to have at least as many
		 * scatter-gather DMA slots as video buffers so that can't
		 * happen.
		 */
		DPRINTK_CAMBUF("ERROR:\
			: Failed to queue a video buffer for DMA!\n");
		vb->state = state;
	}
}

/* Handles ioctls such as Get/Set format, Req Buf, 
 * Q/DQ bufs, Stream ON/OFF
 */
static int
omap34xxcam_do_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     void *arg)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_buf *cam = fh->cam;
	int err = 0;

	switch (cmd) {
	case VIDIOC_G_FMT:
		{
		struct v4l2_format *f = (struct v4l2_format *) arg;
		struct v4l2_pix_format *pix = &f->fmt.pix;
		memset(pix, 0, sizeof (*pix));
		*pix = cam->pix;
		/* TBD No need of knowing the pixel format */
		return err;
		}
	case VIDIOC_S_FMT:
		{
		struct v4l2_format *f = (struct v4l2_format *) arg;
		cam->pix.width = f->fmt.pix.width;
		cam->pix.height = f->fmt.pix.height;
		cam->pix.sizeimage = cam->pix.width * cam->pix.height *2;
		/* TBD No need of knowing the pixel format */
		return err;
		}
	case VIDIOC_REQBUFS:
		return videobuf_reqbufs(&fh->vbq, arg);

	case VIDIOC_QUERYBUF:
		return videobuf_querybuf(&fh->vbq, arg);

	case VIDIOC_QBUF:
		return videobuf_qbuf(&fh->vbq, arg);

	case VIDIOC_DQBUF:
		return videobuf_dqbuf(&fh->vbq, arg,
				file->f_flags & O_NONBLOCK);
	case VIDIOC_STREAMON:
		{
		if (cam->streaming) {
			spin_unlock(&cam->vbq_lock);
			return -EBUSY;
		} else
			cam->streaming = fh;

		cam->dma_notify = 1;
		return videobuf_streamon(&fh->vbq);
		}

	case VIDIOC_STREAMOFF:
		{
		struct videobuf_queue *q = &fh->vbq;
		int i;
		/* video-buf lib has trouble to turn off streaming while
		 * any buffer is still in QUEUED state. Let's wait until
		 * all queued buffers are filled.
		 */
		

		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			if (q->bufs[i]->state == STATE_QUEUED){
			err = videobuf_waiton(q->bufs[i], 0, 0);
			if (err)
				return err;
			}
		}
		isp_state = camcfg_useispbuf(ISP_BUF_INIT);
		camcfg_unset_callback();
		if (cam->streaming == fh)
			cam->streaming = NULL;

		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			//Currently vbq_
			//release is unmapping the buffers
			//when ISP is using it. Need to put this in
			//proper place or need to do synchronise 
			//unset_callback with the isr
			if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
				omap34xxcam_vbq_release(q,
							q->bufs[i]);
		}
		cam->dma_notify = 0;
		//TBD Currently vbq_release is unmapping the buffers
		//when ISP is using it.Need to put this in proper place 
		//or need to do synchronise unset_callback with the isr
		/* Unmap ISPMMU addr for mmap memory is done via calling 
		 * vbq_release from video stream off method
		 */
		videobuf_streamoff(q);
		DPRINTK_CAMBUF("Stream off done\n");
		return err;
		}
	default:
		return -EINVAL;
	}
	return err;
}

static int
omap34xxcam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_buf *cam = fh->cam;

	omap34xxcam_suspend_lockout(cam, file);
	return videobuf_mmap_mapper(&fh->vbq, vma);
}

static int
omap34xxcam_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		  unsigned long arg)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_buf *cam = fh->cam;
	DPRINTK_CAMBUF("+CAM_BUF omap34xxcam_ioctl\n");
	omap34xxcam_suspend_lockout(cam, file);
	return video_usercopy(inode, file, cmd, arg, omap34xxcam_do_ioctl);
}

static int
omap34xxcam_release(struct inode *inode, struct file *file)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_buf *cam = fh->cam;
	struct videobuf_queue *q = &fh->vbq;
	int i;
	DPRINTK_CAMBUF("+ omap34xxcam_release");
	omap34xxcam_suspend_lockout(cam, file);


	/* stop streaming capture */
	if (cam->streaming == fh) {
		/* Call unset before releasing the buffer */
		/* Moving this inside for G/S_FMT calls only where
		 * camcfg_init() is not done */
		isp_state = camcfg_useispbuf(ISP_BUF_INIT);
		camcfg_unset_callback();
		cam->streaming = NULL;
		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
				omap34xxcam_vbq_release(q, q->bufs[i]);
		}
		videobuf_streamoff(q);
	}	
	cam->free_sgdma = NUM_SG_DMA;

	/* release read_buf videobuf_buffer struct */
	if (fh->vbq.read_buf) {
		omap34xxcam_vbq_release(q, fh->vbq.read_buf);
		kfree(fh->vbq.read_buf);
	}

	/* free video_buffer objects */
	videobuf_mmap_free(q);

	file->private_data = NULL;
	kfree(fh);
	DPRINTK_CAMBUF("- omap34xxcam_release");
	return 0;
}

static int
omap34xxcam_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct omap34xxcam_buf *cam = saved_cam;
	struct omap34xxcam_fh *fh;

	DPRINTK_CAMBUF("+ omap34xxcam_open\n");

	if (!cam || !cam->vfd || (cam->vfd->minor != minor))
		return -ENODEV;

	omap34xxcam_suspend_lockout(cam, file);

	/* allocate per-filehandle data */
	fh = kmalloc(sizeof (*fh), GFP_KERNEL);
	if (NULL == fh) {
		return -ENOMEM;
	}
	file->private_data = fh;
	fh->cam = cam;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	videobuf_queue_init(&fh->vbq, &cam->vbq_ops, NULL, &cam->vbq_lock,
			    fh->type, V4L2_FIELD_NONE,
			    sizeof (struct videobuf_buffer), fh);
	DPRINTK_CAMBUF("- omap34xxcam_open\n");
	return 0;
}

static struct file_operations omap34xxcam_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.ioctl = omap34xxcam_ioctl,
	.mmap = omap34xxcam_mmap,
	.open = omap34xxcam_open,
	.release = omap34xxcam_release,
};

/* -------------------------------------------------------------------------- */
/* TBD See what part is to be done here
 * and what part in palkern driver
 */
#ifdef CONFIG_PM
static int
omap34xxcam_suspend(struct platform_device *dev, u32 state)
{
	struct omap34xxcam_buf *cam = omap_get_drvdata(dev);
	if (cam->suspended)
		return 0;

	/* lock-out applications during suspend */
	cam->suspended = 1;

	/* ???what else is needed to suspend isp??? */
	if(cam->streaming){
		camcfg_unset_callback();
		/*omap34xxcam_stop_isp(cam);*/
	}
	return 0;
}
static int
omap34xxcam_resume(struct platform_device *dev)
{
	struct omap34xxcam_buf *cam = omap_get_drvdata(dev);
	if (!cam->suspended)
		return 0;

	if(cam->streaming ){
		/* what else is needed to resume isp??? */
		if (cam->free_sgdma > NUM_SG_DMA)
			cam->free_sgdma = NUM_SG_DMA;

		if (cam->streaming) {
			/* capture was in progress, so we need to register
			 * our routine to restart the camera interface 
			 * the next time a DMA transfer is queued.
			 */
			cam->dma_notify = 1;
			omap34xxcam_sg_dma_process(cam,0);
		}
	}
	/* wake up applications waiting on suspend queue */
	cam->suspended = 0;
	wake_up(&cam->suspend_wq);
	return 0;
}
#endif				/* PM */


static int
omap34xxcam_probe(struct platform_device *dev)
{
	return 0;
}

static void
omap34xxcam_dev_release(struct device *dev)
{
}

static struct platform_device omap34xxcam_dev = {
	.name = CAM_NAME,
	.id   = 100,
	.dev = {
		.release = omap34xxcam_dev_release,
		},
};

static struct platform_driver omap34xxcam_driver = {
	.driver = {
		.name = CAM_NAME,
		},
	.probe = omap34xxcam_probe,
#ifdef CONFIG_PM
	.suspend = omap34xxcam_suspend,
	.resume = omap34xxcam_resume,
#endif
};

static void
omap34xxcam_cleanup(void)
{
	struct omap34xxcam_buf *cam = saved_cam;
	struct video_device *vfd;
	
	if (!cam)
		return;

	vfd = cam->vfd;

	if (vfd) {
		if (vfd->minor == -1) {
			/* The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vfd);
		} else {
			/* The unregister function will release the 
			 * video_device struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
			video_device_release(vfd);
		}
		cam->vfd = NULL;
	}
	platform_device_unregister(&omap34xxcam_dev);
	platform_driver_unregister(&omap34xxcam_driver);

	kfree(cam);
	saved_cam = NULL;
}


static int __init
omap34xxcam_init(void)
{
	struct omap34xxcam_buf *cam;
	struct video_device *vfd;
	int ret;
	int i = 0;
	
	DPRINTK_CAMBUF("+ omap34xxcam_init\n");
	ret = platform_driver_register(&omap34xxcam_driver);
	if (ret != 0)
		return ret;
	ret = platform_device_register(&omap34xxcam_dev);
	if (ret != 0) {
		platform_driver_unregister(&omap34xxcam_driver);
		return ret;
	}
	cam = kmalloc(sizeof (struct omap34xxcam_buf), GFP_KERNEL);
	if (!cam) {
		DPRINTK_CAMBUF("ERROR: could not allocate memory\n");
		platform_device_register(&omap34xxcam_dev);
		platform_driver_unregister(&omap34xxcam_driver);
		return -ENOMEM;
	}
	memset(cam, 0, sizeof (struct omap34xxcam_buf));
	saved_cam = cam;

	/* set driver specific data to use in power mgmt functions */
	platform_set_drvdata(&omap34xxcam_dev, cam);


	/* initialize the video_device struct */
	vfd = cam->vfd = video_device_alloc();
	if (!vfd) {
		printk(KERN_ERR CAM_NAME
			": could not allocate video device struct\n");
		goto init_error;
	}
	vfd->release = video_device_release;

	strncpy(vfd->name, CAM_NAME, sizeof (vfd->name));
	/* TBD Refer with MM what types are required */
	vfd->type = VID_TYPE_CAPTURE | VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;
	/* Register for a VID_HARDWARE_* ID in videodev.h */
	vfd->hardware = 0;
	vfd->fops = &omap34xxcam_fops;
	video_set_drvdata(vfd, cam);
	vfd->minor = -1;

	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		printk(KERN_ERR CAM_NAME
		       ": could not register Video for Linux device\n");
		isp_put();
		goto init_error;
	}
	else
		vfd->minor = video_nr;


	for(i = 0; i< VIDEO_MAX_FRAME; i++)
		cam->isp_addr_capture[i] = 0;

	/* initialize the videobuf queue ops */
	cam->vbq_ops.buf_setup = omap34xxcam_vbq_setup;
	cam->vbq_ops.buf_prepare = omap34xxcam_vbq_prepare;
	cam->vbq_ops.buf_queue = omap34xxcam_vbq_queue;
	cam->vbq_ops.buf_release = omap34xxcam_vbq_release;
	spin_lock_init(&cam->vbq_lock);

	cam->field_count = 0;

	/* Impose a lower limit on the amount of memory allocated for capture.
	 * We require at least enough memory to double-buffer QVGA (300KB).
	 */
	if (capture_mem < QVGA_SIZE * 2)
		capture_mem = QVGA_SIZE * 2;

	cam->streaming = NULL;
	
	/* TBD initialize the preview parameters based on the default capture
	 * format
	 */
	cam->pix.width = QCIF_WIDTH;
	cam->pix.height = QCIF_HEIGHT;
	cam->pix.sizeimage = cam->pix.width * cam->pix.height * 2;
	/* TBD Remove this no need of knowing the pixel format */
	cam->pix.pixelformat = V4L2_PIX_FMT_UYVY;

//	spin_lock_init(&isc_temp_buf_lock);

	/* initialize the SGDMA data */
	omap34xxcam_sgdma_init(cam);

	cam->suspended = 0;
	init_waitqueue_head(&cam->suspend_wq);

	printk(KERN_INFO CAM_NAME
		": registered device video%d [v4l2]\n", vfd->minor);
	DPRINTK_CAMBUF("- omap34xxcam_init");
	return 0;
	
init_error:
	omap34xxcam_cleanup();
	return -ENODEV;
}


MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP34xx Video for Linux camera buffer Management driver");
MODULE_LICENSE("GPL");

module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr,
		"Minor number for video device (-1 ==> auto assign)");
module_param(capture_mem, int, 0);
MODULE_PARM_DESC(capture_mem,
	"Maximum amount of memory for capture buffers (default 4 QXGA buffers)");

module_init(omap34xxcam_init);
module_exit(omap34xxcam_cleanup);



