/*
 * drivers/media/video/omap/omap34xxcam.c
 *
 * Video-for-Linux (Version 2) camera capture driver for OMAP34xx ISP.
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
 * Leverage omap24xx camera driver
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2006 Texas Instruments.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
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

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#include <asm/arch/resource.h>

#include "isp/isp.h"
#include "isp/ispreg.h"
#include "isp/ispccdc.h"
#include "isp/isppreview.h"
#include "isp/ispresizer.h"
#include "isp/ispmmu.h"
#include "isp/isph3a.h"
#include "isp/isphist.h"
#include "sensor_if.h"
#include "omap24xxlib.h"
#include "omap24xxcam_user.h"

#include "omap34xxcam.h"


#undef OMAP_CAM_DEBUG

#ifdef OMAP_CAM_DEBUG
#define DPRINTK_CAM(format,...)\
	printk("CAM: " format, ## __VA_ARGS__)
#else
#define DPRINTK_CAM(format,...)
#endif

#ifndef CONFIG_ARCH_OMAP3410
#define USE_ISP_PREVIEW
#define USE_ISP_RESZ
#endif

/*3MP image capture cannot have the resizer in ES1.0 on as the maximum
 * input/output width to resizer should be less than 1280
 */ 
//#define THREEMP_IMAGE_CAPTURE
/* TODO list:
   - 24xx driver doesn't stop capture interface. only do so in suspend
   - support sensor reg/unreg
   - rmove sg info from _sg_dma_queue API. they are no longer needed
   - cam->isp_pipeline
   - still need 2 overlay sgdma?
   - how to support preview rotation: reserve a 4MB MMU space
   - CSI support
   - only 8 and 10 bit can be used for isp pipeline. other than ccdc, no processing for 11/12.

*/

/* OMAP3430 has 12 rotation Contexts(settings).
 * Camera driver uses VRFB Context 1 if video 1 pipeline is chosen for Preview
 * or Context 2 if video 2 pipeline is chosen.
 * sms_rot_phy is recalculated each time before preview starts.
 */
static unsigned long sms_rot_phy[4];
#define VRFB_BASE		0x70000000
#define VRFB_CONTEXT_SIZE	0x04000000
#define VRFB_ANGLE_SIZE	0x01000000

#define DEFAULT_SENSOR_XCLK  12000000	/* 12MHz */

#define QVGA_SIZE 	320*240*2	/* memory needed for QVGA image */
#define VGA_SIZE 	QVGA_SIZE*4	/* memory needed for VGA image */
#define SXGA_SIZE 	1280*960*2	/* memory needed for SXGA image */
#define D1_SIZE         768*572*2	/* memory needed for D1 image */

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QXGA_WIDTH	2048
#define QXGA_HEIGHT	1536
#define QCIF_WIDTH	176
#define QCIF_HEIGHT	144
#define MAX_RESIZER_INPUTSIZE	1280
#define CAM_NAME "omap34xxcam"

#ifndef CONFIG_ARCH_OMAP3410
extern void isph3a_notify (int notify);
#endif

int sensor_init = 0;
/* Need these for streaming case temporary buffer*/
static unsigned long isp_temp_virt_addr;
static unsigned long isp_temp_phy_addr;
static unsigned long isp_temp_ispmmu_addr;
static int applyCrop = 0;

/* Internal states to maintain buffer synchronization on startup
ISP_BUF_INIT     - ISP is running and new buffers have been queued but not written to. 
ISP_FREE_RUNNING - ISP is running but no buffers have been queued
ISP_BUF_TRAN     - CCDC bit is written to but not latched on till the next VSYNC. Data
		   will be written to the buffer only when state transitions to ISP_BUF_INIT
*/

#define ISP_BUF_INIT     0
#define ISP_FREE_RUNNING 1
#define ISP_BUF_TRAN     2

spinlock_t isp_temp_buf_lock;

static int isp_temp_state = ISP_BUF_INIT;

/* this is the sensor ops implemented by the associated sesnor driver */
extern struct camera_sensor camera_sensor_if;

/* global variables */
static struct omap34xxcam_device *saved_cam;

struct constraint_handle *co_opp_camera_vdd1;
struct constraint_handle *co_opp_camera_vdd2;
struct constraint_handle *co_opp_camera_latency;

/* module parameters */
static int video_nr = 0;	/* video device minor (-1 ==> auto assign) */

/* Maximum amount of memory to use for capture buffers.
 * Default is 4800KB, enough to double-buffer SXGA. */
static int capture_mem = 2048*1536*2; //SXGA_SIZE * 2;

/* Size of video overlay framebuffer.  This determines the maximum image size
 * that can be previewed.  Default is 600KB, enough for VGA. */
/* TODO: very big now enough for ISP Preview output. Will make it VGA */
static int overlay_mem = 2048*1536*2/4;

/* Crop capabilities */
static struct v4l2_rect ispcroprect_a;
static struct v4l2_rect ispcroprect_b;
static struct v4l2_rect cur_rect;

/* Brightness and Contrast capabilities */
static struct v4l2_queryctrl ispbccontrol[] = {
        { 
		.id = V4L2_CID_BRIGHTNESS, 
		.type = V4L2_CTRL_TYPE_INTEGER, 
		.name = "Brightness",
		.minimum = ISPPRV_BRIGHT_LOW,
		.maximum = ISPPRV_BRIGHT_HIGH,
		.step = ISPPRV_BRIGHT_STEP,
		.default_value = ISPPRV_BRIGHT_DEF
	},{
		.id = V4L2_CID_CONTRAST, 
		.type = V4L2_CTRL_TYPE_INTEGER, 
		.name = "Contrast",
		.minimum = ISPPRV_CONTRAST_LOW, 
		.maximum = ISPPRV_CONTRAST_HIGH, 
		.step = ISPPRV_CONTRAST_STEP, 
		.default_value = ISPPRV_CONTRAST_DEF
	},{
		.id = V4L2_CID_PRIVATE_BASE, 
		.type = V4L2_CTRL_TYPE_INTEGER, 
		.name = "Color Effects",
		.minimum = PREV_DEFAULT_COLOR, 
		.maximum = PREV_SEPIA_COLOR, 
		.step = 1, 
		.default_value = PREV_DEFAULT_COLOR }
};

static struct constraint_id cnstr_id_vdd1 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};

static struct constraint_id cnstr_id_vdd2 = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd2_opp",
};

static struct constraint_id cnstr_id_latency = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};

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


/* -------------------------------------------------------------------------- */
static void
omap34xxcam_set_isp_buf(struct omap34xxcam_device *cam, struct sgdma_state *sgdma)
{
#ifdef USE_ISP_RESZ
	/* This has to occur before the vysnc of the intended frame comes */
	if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
		ispresizer_set_outaddr(sgdma->isp_addr);
	}
	else
#endif
#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
		isppreview_set_outaddr(sgdma->isp_addr);
	}
	else
#endif
	if (cam->isp_pipeline & OMAP_ISP_CCDC) {
		ispccdc_set_outaddr(sgdma->isp_addr);		
	}
}

static void
omap34xxcam_set_isp_callback(struct omap34xxcam_device *cam, struct sgdma_state *sgdma)
{
       omap_writel(omap_readl(ISP_IRQ0STATUS)| ISP_INT_CLR, ISP_IRQ0STATUS);        
#ifdef USE_ISP_RESZ
	if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
	isp_set_callback(CBK_RESZ_DONE, sgdma->callback, cam, sgdma->arg);
	}
#endif

#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
	isp_set_callback(CBK_PREV_DONE, sgdma->callback, cam, sgdma->arg);
	}
#endif
	if (cam->isp_pipeline & OMAP_ISP_CCDC) {
	isp_set_callback(CBK_CCDC_VD0, sgdma->callback, cam, sgdma->arg);
	}
	isp_set_callback(CBK_HS_VS, sgdma->callback, cam, sgdma->arg);
}

static void
omap34xxcam_unset_callback(struct omap34xxcam_device *cam)
{
	isp_unset_callback(CBK_HS_VS);
#ifdef USE_ISP_RESZ
	/* This has to occur before the vysnc of the intended frame comes */
	if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
		isp_unset_callback(CBK_RESZ_DONE);
	}
#endif
#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
		isp_unset_callback(CBK_PREV_DONE);
	}
#endif
	if (cam->isp_pipeline & OMAP_ISP_CCDC) {
		isp_unset_callback(CBK_CCDC_VD0);
	}
	omap_writel(omap_readl(ISP_IRQ0STATUS)| ISP_INT_CLR, ISP_IRQ0STATUS);
}
static int
omap34xxcam_start_isp(struct omap34xxcam_device *cam)
{
	/* start the needed isp components assuming these components
	 * are configured correctly.
	 */
#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW)
		isppreview_enable(1);
#endif
	return 0;
}

static inline int
omap34xxcam_stop_isp(struct omap34xxcam_device *cam)
{
	omap34xxcam_unset_callback(cam);

#ifdef USE_ISP_RESZ
	if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
		ispresizer_enable(0);
		while (ispresizer_busy()) {
			mdelay(10);
		}
	}
#endif
#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
		isppreview_enable(0);
		while (isppreview_busy()) {
			mdelay(10);
		}
	}
#endif
	if (cam->isp_pipeline & OMAP_ISP_CCDC) {
		ispccdc_enable(0);
		while (ispccdc_busy()) {
			mdelay(10);
		}
	}

	return 0;
}

/*
 * Process the scatter-gather DMA queue by starting queued transfers.
 * This can be called from either a process context or an IRQ context.
 * 	case 1: from a process, ISP not started.
 * 	case 2: from a process, ISP started.
 * 	case 3: from IRQ, ISP started.
 * We make sure updating buffer pointer happens only once on the
 * shadow register,
 */
static void
omap34xxcam_sg_dma_process(struct omap34xxcam_device *cam, int irq)
{
	struct sgdma_state *sgdma;
	unsigned long irqflags;
	spin_lock_irqsave(&cam->sg_lock, irqflags);
	/* any pending sgdma ?
	 * we can at most start or queue one sgdma */
	if ((NUM_SG_DMA - cam->free_sgdma) > 0) {
		/* get the next sgdma */
	sgdma = cam->sgdma + (cam->next_sgdma + cam->free_sgdma) % NUM_SG_DMA;
	  if (!irq) {
	    if (cam->dma_notify) {
	      /* case 1: queue & start. */
	      omap34xxcam_set_isp_callback(cam,sgdma);
	      omap34xxcam_set_isp_buf(cam, sgdma);
		  ispccdc_enable(1);
	      omap34xxcam_start_isp(cam);
	      cam->dma_notify = 0;
          spin_lock(&isp_temp_buf_lock);		  
   		  isp_temp_state = ISP_BUF_TRAN;
          spin_unlock(&isp_temp_buf_lock);

	    } else {
	    /* case 3:only need to queue (update buf ptr). */
	    spin_lock(&isp_temp_buf_lock);
		if(isp_temp_state == ISP_FREE_RUNNING)
			{
	          omap34xxcam_set_isp_callback(cam,sgdma);			 
		      omap34xxcam_set_isp_buf(cam, sgdma);
			  /* Non startup case */	
    	  	  ispccdc_enable(1);
 		  	  isp_temp_state = ISP_BUF_TRAN;		  
			}
	      spin_unlock(&isp_temp_buf_lock);
	    }
	  } else {
	    /* case 3:only need to queue (update buf ptr). */
	    spin_lock(&isp_temp_buf_lock);
	    omap34xxcam_set_isp_callback(cam,sgdma);			 
	    omap34xxcam_set_isp_buf(cam, sgdma);
		/* Non startup case */		
		ispccdc_enable(1);
		isp_temp_state = ISP_BUF_INIT;		
		spin_unlock(&isp_temp_buf_lock);		
	    /* TODO: clear irq. old interrupt can come first. OK for preview.*/
	    
	    if (cam->dma_notify) {
	      omap34xxcam_start_isp(cam);
	      cam->dma_notify = 0;
	    }
	  }
	}
       	else {
	  spin_lock(&isp_temp_buf_lock);
	  /* Disable VD0 and CCDC here before next VSYNC */
      omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_CCDC_VD0_IRQ),ISP_IRQ0ENABLE);
      ispccdc_enable(0);
  	  isp_temp_state = ISP_FREE_RUNNING;
	  spin_unlock(&isp_temp_buf_lock);
	}


	spin_unlock_irqrestore(&cam->sg_lock, irqflags);
}

/* Queue a scatter-gather DMA transfer from the camera to memory.
 * Returns zero if the transfer was successfully queued, or
 * non-zero if all of the scatter-gather slots are already in use.
 */
static int
omap34xxcam_sg_dma_queue(struct omap34xxcam_device *cam, dma_addr_t isp_addr,
			 isp_callback_t callback, void *arg, int irq)
{
	unsigned long irqflags;
	struct sgdma_state *sgdma;

	spin_lock_irqsave(&cam->sg_lock, irqflags);

	if (!cam->free_sgdma) {
		spin_unlock_irqrestore(&cam->sg_lock, irqflags);
		return -EBUSY;
	}

	sgdma = cam->sgdma + cam->next_sgdma;

	sgdma->isp_addr = isp_addr;
	sgdma->status = 0;
	sgdma->callback = callback;
	sgdma->arg = arg;

	cam->next_sgdma = (cam->next_sgdma + 1) % NUM_SG_DMA;
	cam->free_sgdma--;

	spin_unlock_irqrestore(&cam->sg_lock, irqflags);

	omap34xxcam_sg_dma_process(cam, irq);

	return 0;
}

static void
omap34xxcam_sgdma_init(struct omap34xxcam_device *cam)
{
	int sg;

	cam->free_sgdma = NUM_SG_DMA;
	cam->next_sgdma = 0;
	for (sg = 0; sg < NUM_SG_DMA; sg++) {
		cam->sgdma[sg].status = 0;
		cam->sgdma[sg].callback = NULL;
		cam->sgdma[sg].arg = NULL;
	}
}

/* -------------------------------------------------------------------------- */

/* Callback routine for overlay DMA completion.  We just start another DMA
 * transfer unless overlay has been turned off.
 */
static void
omap34xxcam_overlay_callback(unsigned long status, void *arg1, void *arg2)
{
	struct omap34xxcam_device *cam = (struct omap34xxcam_device *) arg1;
	unsigned long irqflags;
	int err;

	spin_lock(&cam->sg_lock);
	cam->free_sgdma++;
 	if(cam->free_sgdma > NUM_SG_DMA)
	  cam->free_sgdma = NUM_SG_DMA;
	spin_unlock(&cam->sg_lock);
	
	spin_lock_irqsave(&cam->overlay_lock, irqflags);

	switch (status) {
			case CCDC_VD0:
				/* Program shadow registers for CCDC */
				ispccdc_config_shadow_registers();
			      spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
				  return;

#ifdef USE_ISP_PREVIEW
			case PREV_DONE:
			  if (cam->isp_pipeline & OMAP_ISP_RESIZER){			   

			    if(!applyCrop)
			      ispresizer_enable(1);

			    if(applyCrop && !ispresizer_busy())
			      {
				ispresizer_enable(0);
				ispresizer_applycrop();
				applyCrop = 0;
			      }

			  }

				/* Program shadow registers for PREVIEW */
				isppreview_config_shadow_registers();
				isph3a_update_wb();
				/* Check if PREVIEW is the last module in the pipeline */
				if (cam->isp_pipeline & OMAP_ISP_RESIZER){
					spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
					return;
				}
				else {
					break;
				}
#endif
#ifdef USE_ISP_RESZ
			case RESZ_DONE:
				/* Program shadow registers for RESIZER */	
				ispresizer_config_shadow_registers();
				break;
#endif
			case HS_VS:
	  			break;
			default:
				break;
	}
	
	if (cam->overlay_cnt > 0)
		--cam->overlay_cnt;

	if (!cam->previewing) {
		omap34xxcam_stop_isp(cam);
		spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
		return;
	}

	
	while (cam->overlay_cnt < 2) {
		err = omap34xxcam_sg_dma_queue(cam,
					       cam->isp_addr_overlay,
					       omap34xxcam_overlay_callback,
					       NULL, 1);
		if (err)
			break;			
		++cam->overlay_cnt;
	}

	spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
}

/* Begin DMA'ing into the camera overlay framebuffer.  We queue up two DMA
 * transfers so that all frames will be captured.
 */
static void
omap34xxcam_start_overlay_dma(struct omap34xxcam_device *cam)
{
	int err;
	unsigned long irqflags;

	if (!cam->previewing)
		return;

	if (cam->pix.sizeimage > cam->overlay_size)
		return;

	spin_lock_irqsave(&cam->overlay_lock, irqflags);
	while (cam->overlay_cnt < 2) {
		err = omap34xxcam_sg_dma_queue(cam,
					       cam->isp_addr_overlay,
					       omap34xxcam_overlay_callback,
					       NULL, 0);
		if (err)
			break;
		++cam->overlay_cnt;
	}

	spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
}


/* -------------------------------------------------------------------------- */

/* This routine is called from interrupt context when a scatter-gather DMA
 * transfer of a videobuf_buffer completes.
 */
static void
omap34xxcam_vbq_complete(unsigned long status, void *arg1, void *arg2)
{
	struct omap34xxcam_device *cam = (struct omap34xxcam_device *) arg1;
	struct videobuf_buffer *vb = (struct videobuf_buffer *) arg2;


	spin_lock(&cam->vbq_lock);


	switch (status) {
	case CCDC_VD0:
	  /* Program shadow registers for CCDC */
	  ispccdc_config_shadow_registers();
	  /* Check if CCDC is the last module in the pipeline */	
	    spin_unlock(&cam->vbq_lock);
	 if ((cam->isp_pipeline & OMAP_ISP_RESIZER) || (cam->isp_pipeline & OMAP_ISP_PREVIEW))
	    return;
	  else
	  	{
	  	spin_lock(&isp_temp_buf_lock);
		/* Either in ISP_BUF_TRAN or ISP_FREE_RUNNING then return */
	  	if(isp_temp_state != ISP_BUF_INIT)
	  		{
			spin_unlock(&isp_temp_buf_lock);
			return;
			}
		else
			{
			spin_unlock(&isp_temp_buf_lock);
			break;
			}
	  	}
#ifdef USE_ISP_PREVIEW
	case PREV_DONE:
	  if (cam->isp_pipeline & OMAP_ISP_RESIZER){			   
	    if(!applyCrop && (isp_temp_state == ISP_BUF_INIT))
	      ispresizer_enable(1);
	    
	    if(applyCrop && !ispresizer_busy())
	      {
		ispresizer_enable(0);
		ispresizer_applycrop();
		applyCrop = 0;
	      }	    	    
	  }

	  /* Program shadow registers for PREVIEW */
	  isppreview_config_shadow_registers();
	  isph3a_update_wb();
	  /* Check if PREVIEW is the last module in the pipeline */
	  if (cam->isp_pipeline & OMAP_ISP_RESIZER){
	    spin_unlock(&cam->vbq_lock);
	    return;
	    break;
	  }
	  else {
	    break;
	  }
#endif
#ifdef USE_ISP_RESZ
	case RESZ_DONE:
	  /* Program shadow registers for RESIZER */
	  ispresizer_config_shadow_registers();
	  spin_lock(&isp_temp_buf_lock);
	  if(isp_temp_state != ISP_BUF_INIT) {
	    if (cam->still_capture || cam->streaming == NULL) {
	      omap34xxcam_stop_isp(cam);
	    } 
	    spin_unlock(&isp_temp_buf_lock);
		spin_unlock(&cam->vbq_lock);
	    return;
	  }
	  spin_unlock(&isp_temp_buf_lock);
	  break;
#endif
	 case HS_VS:
	  spin_lock(&isp_temp_buf_lock);
	  if(isp_temp_state == ISP_BUF_TRAN)
	  	{
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_CCDC_VD0_IRQ,
				ISP_IRQ0ENABLE);
		isp_temp_state = ISP_BUF_INIT;
	  	}	  	
	  spin_unlock(&isp_temp_buf_lock);
	  spin_unlock(&cam->vbq_lock);
	  return;	
	  
	default:
	  spin_unlock(&cam->vbq_lock);
	  return;
	}

	spin_lock(&cam->sg_lock);
	cam->free_sgdma++;
        if(cam->free_sgdma > NUM_SG_DMA)
	  cam->free_sgdma = NUM_SG_DMA;
	spin_unlock(&cam->sg_lock);

	spin_lock(&cam->vbq_lock);

	do_gettimeofday(&vb->ts);
	vb->field_count = cam->field_count;
	cam->field_count += 2;
	if ((status & MMU_ERR) == MMU_ERR){
		vb->state = STATE_ERROR;
		printk("\tRECD ERROR!!!!!\n");
		//omap34xxcam_error_handler(cam);
	} else {
		vb->state = STATE_DONE;
		if (cam->streaming)
			{
			omap34xxcam_sg_dma_process(cam, 1);	
			}
	}

	if (cam->still_capture || cam->streaming == NULL) {
		omap34xxcam_stop_isp(cam);
	}
	wake_up(&vb->done);
	spin_unlock(&cam->vbq_lock);
}

static void
omap34xxcam_vbq_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_device *cam = fh->cam;

	omap34xxcam_stop_isp(cam);
	/* we already drain the queue so videobuf_waiton is no longer needed */
	videobuf_dma_unmap(q, &vb->dma);
	videobuf_dma_free(&vb->dma);

	/* video-buf calls us multiple times. need to make sure that we only
	 * unmap once */
	if (cam->still_capture) {
		if (cam->isp_addr_read) {
			ispmmu_unmap(cam->isp_addr_read);
			cam->isp_addr_read = 0;
		}
	}
	else {
		if (cam->isp_addr_capture[vb->i]) {
			ispmmu_unmap(cam->isp_addr_capture[vb->i]);
			cam->isp_addr_capture[vb->i] = 0;
		}
	}

	vb->state = STATE_NEEDS_INIT;
}

/* Limit the number of available kernel image capture buffers based on the
 * number requested, the currently selected image size, and the maximum
 * amount of memory permitted for kernel capture buffers.
 */
static int
omap34xxcam_vbq_setup(struct videobuf_queue *q, unsigned int *cnt, unsigned int *size)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_device *cam = fh->cam;

	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;	/* supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	spin_lock(&cam->img_lock);
	if (cam->still_capture && cam->cam_sensor->try_format_still_capture)
		*size = cam->pix2.sizeimage;
	else
		*size = cam->pix.sizeimage;
	spin_unlock(&cam->img_lock);

#if 0 // - Need to check for mmaped case
	while (*size * *cnt > capture_mem)
		(*cnt)--;
#endif

	return 0;
}

static int
omap34xxcam_vbq_prepare(struct videobuf_queue *q, struct videobuf_buffer *vb,
	enum v4l2_field field)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_device *cam = fh->cam;
	unsigned int size, isp_addr;


	int err = 0;

	spin_lock(&cam->img_lock);
	if (cam->still_capture && cam->cam_sensor->try_format_still_capture)
		size = cam->pix2.sizeimage;
	else
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
				spin_unlock(&cam->img_lock);
				omap34xxcam_vbq_release(q, vb);
				spin_lock(&cam->img_lock);
				vb->size = size;
			}
		} else {
			/* We need to allocate a new kernel bounce buffer. */
			vb->size = size;
		}
	}

	if (cam->still_capture && cam->cam_sensor->try_format_still_capture) {
		vb->width = cam->pix2.width;
		vb->height = cam->pix2.height;
	}
	else {
		vb->width = cam->pix.width;
		vb->height = cam->pix.height;
	}
	vb->field = field;

	spin_unlock(&cam->img_lock);

	if (err)
		return err;

	if (vb->state == STATE_NEEDS_INIT) {
		err = videobuf_iolock(q, vb, NULL);
		if (!err) {
			isp_addr = ispmmu_map_sg(vb->dma.sglist, vb->dma.sglen);
			if (!isp_addr)
				err = -EIO;
			else {
			    	if (cam->still_capture)
			    		cam->isp_addr_read = isp_addr;
				else
					cam->isp_addr_capture[vb->i]= isp_addr;
			}
		}
	}

	if (!err) {
		vb->state = STATE_PREPARED;
		if (vb->baddr) {
		    flush_cache_user_range(NULL, vb->baddr, (vb->baddr + vb->bsize));
		} else {
			/* sync a kernel buffer */
			dmac_flush_range(vb->dma.vmalloc, (vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));
			outer_flush_range(__pa(vb->dma.vmalloc), __pa(vb->dma.vmalloc + (vb->dma.nr_pages << PAGE_SHIFT)));
		}
	} else
		omap34xxcam_vbq_release(q, vb);

	return err;
}

static void
omap34xxcam_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = q->priv_data;
	struct omap34xxcam_device *cam = fh->cam;
	enum videobuf_state state = vb->state;
	int err;

	vb->state = STATE_QUEUED;
	err = omap34xxcam_sg_dma_queue(cam,
				       cam->still_capture?cam->isp_addr_read:
				       cam->isp_addr_capture[vb->i],
				       omap34xxcam_vbq_complete, vb, 0);
	if (err) {
		/* Oops.  We're not supposed to get any errors here.  The only
		 * way we could get an error is if we ran out of scatter-gather
		 * DMA slots, but we are supposed to have at least as many
		 * scatter-gather DMA slots as video buffers so that can't
		 * happen.
		 */
		printk(KERN_DEBUG CAM_NAME
		       ": Failed to queue a video buffer for DMA!\n");
		vb->state = state;
	}
}

/* -------------------------------------------------------------------------- */
/*
 * This function is used to set up a default preview window for the current
 * image. It also sets up a proper preview crop.
 * It is called when a new image size is set or a new rotation is set.
 */
void static
omap34xxcam_preview(struct omap34xxcam_device *cam)
{
	/* Get the panel parameters.
	 * They can change from LCD to TV
	 * or TV to LCD
	 */
	omap2_disp_get_dss();
	if (cam->overlay_rotation == PREVIEW_ROTATION_90 ||
	    cam->overlay_rotation == PREVIEW_ROTATION_270)
		omap2_disp_get_panel_size(
		 omap2_disp_get_output_dev(cam->vid_preview),
			  	 &(cam->fbuf.fmt.height),
			  	 &(cam->fbuf.fmt.width));
	else
		omap2_disp_get_panel_size(
		 omap2_disp_get_output_dev(cam->vid_preview),
			  	 &(cam->fbuf.fmt.width),
			  	 &(cam->fbuf.fmt.height));
		
	/* intialize the preview parameters */
	omap24xxvout_new_format(&cam->pix, &cam->fbuf,
				&cam->preview_crop, &cam->win);
	omap2_disp_put_dss();
}

/* list of image formats supported via OMAP ISP */
const static struct v4l2_fmtdesc isp_formats[] = {
	{
		.description	= "UYVY, packed",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	},
	{
		.description	= "YUYV (YUV 4:2:2), packed",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},
	{
		.description	= "Bayer10 (GrR/BGb)",
		.pixelformat	= V4L2_PIX_FMT_SGRBG10,
	},
};
#define NUM_CAPTURE_FORMATS (sizeof(isp_formats)/sizeof(isp_formats[0]))
#define NUM_OVERLAY_FORMATS 2

/*
 * Try format for raw sensors. We need to connect sensor output to ISP for
 * necessary processing.
 * context: 0 none
 *          1 video
 *          2 image
 */
static int
omap34xxcamisp_try_format(struct omap34xxcam_device *cam,
	struct v4l2_pix_format *pix, int context)
{
	struct v4l2_pix_format pixfmt = *pix;
	unsigned int resizer_w, resizer_h;	
	unsigned int width = pix->width;
	unsigned int height = pix->height;
	int ifmt;

	/* first negotiate the size with sensor driver */
	/* TODO: revisit is needed for still capture. try_format vs, try_format_still_capture */
	if (context == 1){
		cam->cam_sensor->try_format(&pixfmt, cam->sensor);
		cam->pix_raw = pixfmt;
		cam->ccdc_input_width  = pixfmt.width;
		cam->ccdc_input_height = pixfmt.height;	
	}
	else if (context == 2){
		cam->cam_sensor->try_format_still_capture
			(&pixfmt, cam->sensor);
		cam->pix2_raw = pixfmt;
		cam->ccdc_input_width2  = pixfmt.width;
		cam->ccdc_input_height2 = pixfmt.height;
	}

	if (pixfmt.pixelformat == V4L2_PIX_FMT_SGRBG10)
		ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_MEM);
	else
		ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);
	/* then negotiate the size with isp ccdc driver */
	ispccdc_try_size(pixfmt.width, pixfmt.height, &pix->width, &pix->height);
	if (context == 1) {
		cam->ccdc_width = pix->width;
		cam->ccdc_height = pix->height;
	}
	else if (context == 2) {
		cam->ccdc_width2 = pix->width;
		cam->ccdc_height2 = pix->height;
	}

#ifdef USE_ISP_PREVIEW
	/* for YUV formats, we need to negotiate with isp preview/resizer */
	if (pixfmt.pixelformat != V4L2_PIX_FMT_SGRBG10) {
		pixfmt.width = width;
		pixfmt.height = height;
		isppreview_try_size(pix->width, pix->height,
				&pixfmt.width, &pixfmt.height);
		if (context == 1) {
			cam->preview_width = pixfmt.width;
			cam->preview_height = pixfmt.height;
		}
		else if (context == 2) {
			cam->preview_width2 = pixfmt.width;
			cam->preview_height2 = pixfmt.height;
		}
#ifndef USE_ISP_RESZ
		pix->width = pixfmt.width;
		pix->height = pixfmt.height;
#else
		pix->width = width;
		pix->height = height;
		resizer_w = pixfmt.width;
		resizer_h = pixfmt.height;
		ispresizer_try_size(&resizer_w, &resizer_h,
				&pix->width, &pix->height);
		if (context == 1) {
			cam->resizer_width = pix->width;
			cam->resizer_height = pix->height;
		}
		else if (context == 2) {
			/* If sensor outputs 3 MP raw image, resizer has to
			 * be turned off because in ES1.0 resizer does not
			 * support input/output width to be greater than 1280
			 */	
		#ifdef THREEMP_IMAGE_CAPTURE
			pix->width = pixfmt.width;
			pix->height = pixfmt.height;
		#else
			cam->resizer_width2 = pix->width;
			cam->resizer_height2 = pix->height;
		#endif
		}
#endif
	}

#endif

	/* done with size negotiation, now fill other info */
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == isp_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 1;
	pix->pixelformat = isp_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width*2;
	pix->sizeimage = pix->bytesperline*pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_UYVY:
			pix->colorspace = V4L2_COLORSPACE_JPEG;
			break;
		default:
			pix->colorspace = V4L2_COLORSPACE_SRGB;
			break;
	}
	return 0;
}


#ifdef USE_ISP_RESZ
static void
omap34xxcamisp_config_resizercrop(struct omap34xxcam_device *cam, int context)
{
	u8 crop_scaling_w;
	u8 crop_scaling_h;
	struct v4l2_pix_format *pix = &cam->pix;
	struct v4l2_pix_format *pix2 = &cam->pix2;

	if (context == 1) {
	        crop_scaling_w = (cam->preview_width * 10)/pix->width;
		crop_scaling_h = (cam->preview_height * 10)/pix->height;

		cur_rect.left   = (ispcroprect_a.left * crop_scaling_w)/10;
		cur_rect.top    = (ispcroprect_a.top * crop_scaling_h)/10;
		cur_rect.width  = (ispcroprect_a.width * crop_scaling_w)/10;
		cur_rect.height = (ispcroprect_a.height * crop_scaling_h)/10;

		ispresizer_trycrop(cur_rect.left,
				   cur_rect.top,
				   cur_rect.width,
				   cur_rect.height, 
				   cam->resizer_width, 
				   cam->resizer_height);

	}
	else if (context == 2) {
		crop_scaling_w = (cam->preview_width * 10)/pix2->width;
		crop_scaling_h = (cam->preview_height * 10)/pix2->height;

		cur_rect.left   = (ispcroprect_b.left * crop_scaling_w)/10;
		cur_rect.top    = (ispcroprect_b.top * crop_scaling_h)/10;
		cur_rect.width  = (ispcroprect_b.width * crop_scaling_w)/10;
		cur_rect.height = (ispcroprect_b.height * crop_scaling_h)/10;	

		ispresizer_trycrop(cur_rect.left,
				   cur_rect.top,
				   cur_rect.width,
				   cur_rect.height, 
				   cam->resizer_width2, 
				   cam->resizer_height2);

	}
}

#endif

/*
 * Setup bit mask indicating how ISP pipeline is used.
 * acquire img_lock?
 */
static void
omap34xxcamisp_calc_pipeline(struct omap34xxcam_device *cam)
{
	struct v4l2_pix_format *pixfmt = &cam->pix;

	if (cam->still_capture && cam->cam_sensor->try_format_still_capture)
		pixfmt = &cam->pix2;

	/* always need CCDC for parallel capturing */
	if (cam->cam_sensor->sensor_interface == SENSOR_PARALLEL)
		cam->isp_pipeline = OMAP_ISP_CCDC;

	if(pixfmt->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_MEM);
		return;
	}
#ifdef USE_ISP_PREVIEW
	if (cam->cam_sensor->sensor_type == SENSOR_RAW &&
	    (pixfmt->pixelformat == V4L2_PIX_FMT_UYVY ||
	     pixfmt->pixelformat == V4L2_PIX_FMT_YUYV)) {
		/* isp preview has to be used.
		 * isp resizer always used.
		 */
		cam->isp_pipeline |= OMAP_ISP_PREVIEW;
		/* If sensor outputs 3 MP raw image, resizer has to
		* be turned off because in ES1.0 resizer does not
		* support input/output width to be greater than 1280
		*/
		if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
			if (cam->still_capture && 
			   cam->pix2_raw.width >= MAX_RESIZER_INPUTSIZE) {
				isppreview_config_datapath(PRV_RAW_CCDC,
							   PREVIEW_MEM);
				cam->isp_pipeline &= ~OMAP_ISP_RESIZER;
			} else {
#ifdef USE_ISP_RESZ
				cam->isp_pipeline |= OMAP_ISP_RESIZER;
				isppreview_config_datapath(PRV_RAW_CCDC,
							   PREVIEW_RSZ);
#else
				isppreview_config_datapath(PRV_RAW_CCDC,
							   PREVIEW_MEM);
#endif
			}
		} else {
#ifdef USE_ISP_RESZ
			cam->isp_pipeline |= OMAP_ISP_RESIZER;
			isppreview_config_datapath(PRV_RAW_CCDC, PREVIEW_RSZ);
#else
			isppreview_config_datapath(PRV_RAW_CCDC, PREVIEW_MEM);
#endif
		}
	}
#endif
}

/* 
 * Config ISP pipeline for ISP sensor. Only CCDC is needed
 * context: 1 video, 2 image
 */
static void
omap34xxcam_config_pipeline(struct omap34xxcam_device *cam, int context)
{
	u32 w, h;


	if (context == 1) {
		w = cam->pix.width;
		h = cam->pix.height;
		if(cam->pix.pixelformat == V4L2_PIX_FMT_SGRBG10)
			ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_MEM);
		else
			ispccdc_config_datapath(CCDC_YUV_SYNC, CCDC_OTHERS_MEM);
		ispccdc_try_size(w, h, &cam->pix.width, &cam->pix.height);
		ispccdc_config_size(w, h, cam->pix.width, cam->pix.height);
	} else {
		w = cam->pix2.width;
		h = cam->pix2.height;
		if(cam->pix2.pixelformat == V4L2_PIX_FMT_SGRBG10)
			ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_MEM);
		else
			ispccdc_config_datapath(CCDC_YUV_SYNC, CCDC_OTHERS_MEM);

		ispccdc_try_size(w, h, &cam->pix2.width, &cam->pix2.height);
		ispccdc_config_size(w, h, cam->pix2.width, cam->pix2.height);
	}
}

/* 
 * Config ISP pipeline for raw sensor
 * context: 1 video, 2 image
 */
static void
omap34xxcamisp_config_pipeline(struct omap34xxcam_device *cam, int context)
{
	u8 crop_scaling_w;
	u8 crop_scaling_h;
	struct v4l2_pix_format *pix = &cam->pix;
	struct v4l2_pix_format *pix2 = &cam->pix2;

	if (context == 1) {

		crop_scaling_w = (cam->ccdc_input_width * 10)/pix->width;
		crop_scaling_h = (cam->ccdc_input_height * 10)/pix->height;
		/* We dont want to loose resolution if crop size is same as output size */
		if(pix->width == ispcroprect_a.width)
			crop_scaling_w = 0;

		/* We dont want to loose resolution if crop size is same as output size */
		if(pix->height == ispcroprect_a.height)
			crop_scaling_h = 0;
		cur_rect.left   = (ispcroprect_a.left * crop_scaling_w)/10;
		cur_rect.top    = (ispcroprect_a.top * crop_scaling_h)/10;
		cur_rect.width  = (ispcroprect_a.width * crop_scaling_w)/10;
		cur_rect.height = (ispcroprect_a.height * crop_scaling_h)/10;

		omap34xxcamisp_try_format(cam, &cam->pix, 1);
		ispccdc_config_size(cam->pix_raw.width,
				cam->pix_raw.height,
	 			cam->ccdc_width,
	 		    	cam->ccdc_height);
#ifdef USE_ISP_PREVIEW
		if (cam->pix.pixelformat != V4L2_PIX_FMT_SGRBG10) {
			isppreview_config_size(cam->ccdc_width,
				    cam->ccdc_height,
		 		    cam->preview_width,
		 		    cam->preview_height);
#ifdef USE_ISP_RESZ
			ispresizer_config_size(cam->preview_width,
				    cam->preview_height,
		 		    cam->resizer_width,
		 		    cam->resizer_height);
#endif
			/* Reapply resizer settings in case a crop is set. */
			if (crop_scaling_h || crop_scaling_w) {
				omap34xxcamisp_config_resizercrop(cam,1);
			}
			
			if (cam->pix.pixelformat == V4L2_PIX_FMT_UYVY) {
				isppreview_config_ycpos(YCPOS_YCrYCb);
#ifdef USE_ISP_RESZ
				ispresizer_config_ycpos(0);
#endif
			}
			else {
				isppreview_config_ycpos(YCPOS_CrYCbY);
#ifdef USE_ISP_RESZ
				ispresizer_config_ycpos(1);
#endif
			}				 
		}
#endif
	}
	else if (context == 2) {
		crop_scaling_w = (cam->ccdc_input_width2 * 10)/pix2->width;
		crop_scaling_h = (cam->ccdc_input_height2 * 10)/pix2->height;
		/* We dont want to loose resolution if crop size is same as output size */
		if (pix2->width == ispcroprect_b.width)
			crop_scaling_w = 0;

		/* We dont want to loose resolution if crop size is same as output size */
		if (pix2->height == ispcroprect_b.height)
			crop_scaling_h = 0;

		cur_rect.left   = (ispcroprect_b.left * crop_scaling_w)/10;
		cur_rect.top    = (ispcroprect_b.top * crop_scaling_h)/10;
		cur_rect.width  = (ispcroprect_b.width * crop_scaling_w)/10;
		cur_rect.height = (ispcroprect_b.height * crop_scaling_h)/10;	

		ispccdc_config_crop(cur_rect.left, 
				cur_rect.top, 
				cur_rect.height, 
				cur_rect.width);
		
		omap34xxcamisp_try_format(cam, &cam->pix2, 2);
		ispccdc_config_size(cam->pix2_raw.width,
				cam->pix2_raw.height,
				cam->ccdc_width2,
				cam->ccdc_height2);
#ifdef USE_ISP_PREVIEW
		if (cam->pix2.pixelformat != V4L2_PIX_FMT_SGRBG10) {
			isppreview_config_size(cam->ccdc_width2,
				    cam->ccdc_height2,
		 		    cam->preview_width2,
		 		    cam->preview_height2);
#ifdef USE_ISP_RESZ
		#ifndef THREEMP_IMAGE_CAPTURE
			ispresizer_config_size(cam->preview_width2,
				    cam->preview_height2,
		 		    cam->resizer_width2,
		 		    cam->resizer_height2);
		#endif
#endif
			if (cam->pix2.pixelformat == V4L2_PIX_FMT_UYVY) {
				isppreview_config_ycpos(YCPOS_YCrYCb);
#ifdef USE_ISP_RESZ
				ispresizer_config_ycpos(0);
#endif
			}
			else {
				isppreview_config_ycpos(YCPOS_CrYCbY);
#ifdef USE_ISP_RESZ
				ispresizer_config_ycpos(1);
#endif
			}
		}
#endif
	}
}

static void
omap34xxcamisp_configure_interface(struct omap34xxcam_device *cam, int context)
{
	struct isp_interface_config config;
	config.ccdc_par_ser = cam->cam_sensor->sensor_interface;
	if((cam->cam_sensor->sensor_type == SENSOR_RAW)){
		config.dataline_shift = 1;
		config.hsvs_syncdetect = 3/*ISPCTRL_SYNC_DETECT_VSRISE*/;
		config.para_clk_pol = 0;
		config.par_bridge = 0;
	}
	if(cam->cam_sensor->sensor_type == SENSOR_ISP){
		config.dataline_shift = 2;
		config.hsvs_syncdetect = 3/*ISPCTRL_SYNC_DETECT_VSRISE*/;
		config.para_clk_pol = 0;
		config.par_bridge = 3;

		/* UYVY RGB565 packed in 8 bits should have bytes swapped */
		if(context == 1) {
			if ((cam->pix.pixelformat == V4L2_PIX_FMT_UYVY) ||
				(cam->pix.pixelformat == V4L2_PIX_FMT_RGB565))
				config.par_bridge = 2;
			else if(cam->pix.pixelformat == V4L2_PIX_FMT_SGRBG10) {
				config.dataline_shift = 1;
				config.par_bridge = 0;
			}
		} else {
			if((cam->pix2.pixelformat == V4L2_PIX_FMT_UYVY) ||
				(cam->pix2.pixelformat == V4L2_PIX_FMT_RGB565))
		    		config.par_bridge = 2;
			else if(cam->pix2.pixelformat == V4L2_PIX_FMT_SGRBG10) {
				config.dataline_shift = 1;
				config.par_bridge = 0;
			}
		}
	}
	config.shutter = 0;
	config.prestrobe = 0;
	config.strobe = 0;
	config.vdint0_timing = 0;
	config.vdint1_timing = 0;
	isp_configure_interface(&config);
}


static int
omap34xxcam_do_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     void *arg)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;
	int err;

	switch (cmd) {
		/* for time being below IOCTL cmd is here */

	case VIDIOC_ENUMINPUT:
		{
			/* default handler assumes 1 video input (the camera) */
			struct v4l2_input *input = (struct v4l2_input *) arg;
			int index = input->index;

			memset(input, 0, sizeof (*input));
			input->index = index;

			if (index > 0)
				return -EINVAL;

			strncpy(input->name, "camera", sizeof (input->name));
			input->type = V4L2_INPUT_TYPE_CAMERA;

			return 0;
		}

	case VIDIOC_G_INPUT:
		{
			unsigned int *input = arg;
			*input = 0;

			return 0;
		}

	case VIDIOC_S_INPUT:
		{
			unsigned int *input = arg;

			if (*input > 0)
				return -EINVAL;

			return 0;
		}

	case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap =
			    (struct v4l2_capability *) arg;

			memset(cap, 0, sizeof (*cap));
			strncpy(cap->driver, CAM_NAME, sizeof (cap->driver));
			strncpy(cap->card, cam->vfd->name, sizeof (cap->card));
			cap->bus_info[0] = '\0';
			cap->capabilities =
			    V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_VIDEO_OVERLAY |
			    V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
			return 0;
		}

	case VIDIOC_ENUM_FMT:
		{
			struct v4l2_fmtdesc *fmt = arg;

			/* we need to make up formats if this is a raw sensor */
			if (cam->cam_sensor->sensor_type == SENSOR_RAW) {
				int index = fmt->index;
				enum v4l2_buf_type type = fmt->type;

				memset(fmt, 0, sizeof(*fmt));
				fmt->index = index;
				fmt->type = type;

				switch (fmt->type) {
				case V4L2_BUF_TYPE_STILL_CAPTURE:
				case V4L2_BUF_TYPE_VIDEO_CAPTURE:
					if (index >= NUM_CAPTURE_FORMATS)
						return -EINVAL;
					break;
				case V4L2_BUF_TYPE_VIDEO_OVERLAY:
					if (index >= NUM_OVERLAY_FORMATS)
						return -EINVAL;
					break;
				default:
					return -EINVAL;
				}
				fmt->flags = isp_formats[index].flags;
				strncpy(fmt->description,
					isp_formats[index].description,
					sizeof(fmt->description));
				fmt->pixelformat = isp_formats[index].pixelformat;
				return 0;
			}
			else
				return cam->cam_sensor->enum_pixformat(fmt,
							       cam->sensor);
		}

	case VIDIOC_G_FMT:
		{
			struct v4l2_format *f = (struct v4l2_format *) arg;
			switch (f->type) {
			case V4L2_BUF_TYPE_STILL_CAPTURE:
				{
					if (cam->cam_sensor->try_format_still_capture){
					struct v4l2_pix_format *pix =
					    &f->fmt.pix;
					memset(pix, 0, sizeof (*pix));
					spin_lock(&cam->img_lock);
					*pix = cam->pix2;
					spin_unlock(&cam->img_lock);
					return 0;
					}
					/* else fall through */
				}
			case V4L2_BUF_TYPE_VIDEO_CAPTURE:
				{
					struct v4l2_pix_format *pix =
					    &f->fmt.pix;
					memset(pix, 0, sizeof (*pix));
					spin_lock(&cam->img_lock);
					*pix = cam->pix;
					spin_unlock(&cam->img_lock);
					return 0;
				}

			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				{
					struct v4l2_window *win = &f->fmt.win;
					memset(win, 0, sizeof (*win));
					/* The API has a bit of a problem here.
					 * We're returning a v4l2_window
					 * structure, but that structure
					 * contains pointers to variable-sized
					 * objects for clipping rectangles and
					 * clipping bitmaps.  We will just
					 * return NULLs for those pointers.
					 */
					spin_lock(&cam->img_lock);
					win->w = cam->win.w;
					win->field = cam->win.field;
					win->chromakey = cam->win.chromakey;
					spin_unlock(&cam->img_lock);
					return 0;
				}

			default:
				{
					return -EINVAL;
				}
			}
		}

	case VIDIOC_TRY_FMT:
		{
			struct v4l2_format *f = (struct v4l2_format *) arg;

			switch (f->type) {
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				{
					struct v4l2_window *win = &f->fmt.win;

					spin_lock(&cam->img_lock);
					err =
					    omap24xxvout_try_window(&cam->fbuf,
								    win);
					spin_unlock(&cam->img_lock);
					return err;
				}

			case V4L2_BUF_TYPE_STILL_CAPTURE:
				{
					if (cam->cam_sensor->try_format_still_capture){
						if (cam->cam_sensor->sensor_type == SENSOR_RAW)
							return omap34xxcamisp_try_format(cam,
									&f->fmt.pix, 0);
						else
							return cam->cam_sensor->
							try_format_still_capture(&f->
									   fmt.
									   pix,
									   cam->
									   sensor);
					}
					/* else fall through */
				}
			case V4L2_BUF_TYPE_VIDEO_CAPTURE:
				{
					if (cam->cam_sensor->sensor_type == SENSOR_RAW)
						return omap34xxcamisp_try_format(cam,
									&f->fmt.pix, 0);
					else
						return cam->cam_sensor->try_format(&f->
									   fmt.
									   pix,
									   cam->
									   sensor);
				}

			default:
				{
					return -EINVAL;
				}
			}
		}

	case VIDIOC_S_FMT:
		{
			struct v4l2_format *f = (struct v4l2_format *) arg;
			struct v4l2_pix_format *pixfmt;

			switch (f->type) {
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				{
					struct v4l2_window *win = &f->fmt.win;

					spin_lock(&cam->img_lock);
					if (cam->previewing || cam->streaming) {
						spin_unlock(&cam->img_lock);
						return -EBUSY;
					}
					/*
					 * Reset Crop values since image size might have changed
					 */
					ispcroprect_a.left   = 0;
					ispcroprect_a.top    = 0;
					ispcroprect_a.width  = f->fmt.pix.width;						
					ispcroprect_a.height = f->fmt.pix.height;					
					
					/* Get the panel parameters.
					 * They can change from LCD to TV
					 * or TV to LCD
					 */
					omap2_disp_get_dss();
					if (cam->overlay_rotation == PREVIEW_ROTATION_90 ||
					    cam->overlay_rotation == PREVIEW_ROTATION_270)
						omap2_disp_get_panel_size(
					 		omap2_disp_get_output_dev(cam->vid_preview),
				  	 			&(cam->fbuf.fmt.height),
				  	 			&(cam->fbuf.fmt.width));
					else
						omap2_disp_get_panel_size(
					 		omap2_disp_get_output_dev(cam->vid_preview),
				  	 			&(cam->fbuf.fmt.width),
				  	 			&(cam->fbuf.fmt.height));

					err =
					    omap24xxvout_new_window(&cam->preview_crop,
								    &cam->win,
								    &cam->fbuf,
								    win);
					omap2_disp_put_dss();
					spin_unlock(&cam->img_lock);
					return err;
				}

			case V4L2_BUF_TYPE_STILL_CAPTURE:
				{					
					if (cam->cam_sensor->try_format_still_capture &&
					    cam->cam_sensor->configure_still_capture) {
					spin_lock(&cam->img_lock);
					if (cam->cam_sensor->sensor_type == SENSOR_RAW) {

					/*
					 * Reset Crop values since image size might have changed
					 */					
						ispcroprect_b.left   = 0;
						ispcroprect_b.top    = 0;
						ispcroprect_b.width  = f->fmt.pix.width;						
						ispcroprect_b.height = f->fmt.pix.height;					
						omap34xxcamisp_try_format(cam, &f->fmt.pix, 2);
						pixfmt = &cam->pix2_raw;
					}
					else {
						cam->cam_sensor->try_format_still_capture(&f->fmt.pix,
								    cam->
								    sensor);
						pixfmt = &f->fmt.pix;
					}

					/* set the new user capture format */
					cam->pix2 = f->fmt.pix;
					spin_unlock(&cam->img_lock);

					err =
					    cam->cam_sensor->configure_still_capture
					    			      (pixfmt,
								       cam->
								       xclk,
								       &cam->
								       cparm.
								       timeperframe,
								       cam->
								       sensor);
					if (cam->cam_sensor->sensor_type == SENSOR_RAW) 
						omap34xxcamisp_config_pipeline(cam, 2);
					else /* ISP sensor */
						omap34xxcam_config_pipeline(cam, 2);
					
					return err;
					}
					/* else fall through */
				}
			case V4L2_BUF_TYPE_VIDEO_CAPTURE:
				{
					spin_lock(&cam->img_lock);
					if (cam->streaming || cam->previewing) {
						spin_unlock(&cam->img_lock);
						return -EBUSY;
					}


					
					if (cam->cam_sensor->sensor_type == SENSOR_RAW) {

					/*
					 * Reset Crop values since image size might have changed
					 */
						ispcroprect_a.left   = 0;
						ispcroprect_a.top    = 0;
						ispcroprect_a.width  = f->fmt.pix.width;						
						ispcroprect_a.height = f->fmt.pix.height;					
						omap34xxcamisp_try_format(cam, &f->fmt.pix, 1);
						pixfmt = &cam->pix_raw;
					}
					else {
						cam->cam_sensor->try_format(&f->fmt.pix,
								    cam->
								    sensor);
						pixfmt = &f->fmt.pix;
					}

					/* set the new user capture format */
					cam->pix = f->fmt.pix;

					/* adjust the capture frame rate */
					cam->xclk =
					    cam->cam_sensor->calc_xclk(pixfmt,
					     			       &cam->
								       nominal_timeperframe,
								       cam->
								       sensor);
					cam->cparm.timeperframe =
					    cam->nominal_timeperframe;

					/* set a default display window and preview crop */
					omap34xxcam_preview(cam);
					spin_unlock(&cam->img_lock);

					/* negotiate xclk with isp */
					cam->xclk = isp_negotiate_xclka(cam->xclk);
					/* program the agreed new xclk frequency */
					cam->xclk = isp_set_xclka(cam->xclk);

					/* program the sensor */
					err =
					    cam->cam_sensor->configure(pixfmt,
								       cam->
								       xclk,
								       &cam->
								       cparm.
								       timeperframe,
								       cam->
								       sensor);
					if (cam->cam_sensor->sensor_type == SENSOR_RAW)
						omap34xxcamisp_config_pipeline(cam, 1);
					else /* ISP sensor */
						omap34xxcam_config_pipeline(cam, 1);
					return err;
				}

			default:
				{
					return -EINVAL;
				}
			}
		}

	case VIDIOC_G_FBUF:
		{
			struct v4l2_framebuffer *fbuf =
			    (struct v4l2_framebuffer *) arg;

			spin_lock(&cam->img_lock);
			*fbuf = cam->fbuf;
			spin_unlock(&cam->img_lock);
			return 0;
		}

	case VIDIOC_S_FBUF:
		{
			struct v4l2_framebuffer *fbuf =
			    (struct v4l2_framebuffer *) arg;
			unsigned int flags = fbuf->flags;

			/* The only field the user is allowed to change is
			 * fbuf->flags.
			 */
			spin_lock(&cam->img_lock);
			if (cam->previewing) {
				spin_unlock(&cam->img_lock);
				return -EBUSY;
			}
			if (flags & V4L2_FBUF_FLAG_CHROMAKEY)
				cam->fbuf.flags |= V4L2_FBUF_FLAG_CHROMAKEY;
			else
				cam->fbuf.flags &= ~V4L2_FBUF_FLAG_CHROMAKEY;
			spin_unlock(&cam->img_lock);
			return 0;
		}

	case VIDIOC_CROPCAP:
		{
			struct v4l2_cropcap *cropcap =
			    (struct v4l2_cropcap *) arg;
			enum v4l2_buf_type type = cropcap->type;

			memset(cropcap, 0, sizeof (*cropcap));
			cropcap->type = type;
			switch (type) {
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				{
					struct v4l2_pix_format *pix = &cam->pix;

					spin_lock(&cam->img_lock);
					cropcap->bounds.width = pix->width & ~1;
					cropcap->bounds.height =
					    pix->height & ~1;
					omap24xxvout_default_crop(&cam->pix,
								  &cam->fbuf,
								  &cropcap->
								  defrect);
					spin_unlock(&cam->img_lock);
					cropcap->pixelaspect.numerator = 1;
					cropcap->pixelaspect.denominator = 1;
					return 0;
				}

			case V4L2_BUF_TYPE_STILL_CAPTURE:
			case V4L2_BUF_TYPE_VIDEO_CAPTURE:
				{
					struct v4l2_pix_format *pix = &cam->pix;
					struct v4l2_pix_format *pix2 = &cam->pix2;

					if (cam->cam_sensor->sensor_type == SENSOR_RAW) {
						cropcap->bounds.left = cropcap->bounds.top = 0;							
						if (type == V4L2_BUF_TYPE_STILL_CAPTURE) {
							cropcap->bounds.width  = pix2->width;
							cropcap->bounds.height = pix2->height;
						} else {
							cropcap->bounds.width  = pix->width;
							cropcap->bounds.height = pix->height;					
						}
						cropcap->defrect = cropcap->bounds;
						cropcap->pixelaspect.numerator = 1;
						cropcap->pixelaspect.denominator = 1;
						return 0;
					} else {
						if (cam->cam_sensor->cropcap) {
							return cam->cam_sensor->cropcap(cropcap,
									cam->sensor);
						}
					}
					spin_lock(&cam->img_lock);
					cropcap->bounds.width = cam->pix.width;
					cropcap->bounds.height =
						cam->pix.height;
					spin_unlock(&cam->img_lock);
					cropcap->defrect.width =
						cropcap->bounds.width;
					cropcap->defrect.height =
						cropcap->bounds.height;
					cropcap->pixelaspect.numerator = 1;
					cropcap->pixelaspect.denominator = 1;
					return 0;
				}

			default:
				{
					return -EINVAL;
				}
			}
		}

	case VIDIOC_G_CROP:
		{
			struct v4l2_crop *crop = (struct v4l2_crop *) arg;

			switch (crop->type) {
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				{
					spin_lock(&cam->img_lock);
					crop->c = cam->preview_crop;
					spin_unlock(&cam->img_lock);
					return 0;
				}

			case V4L2_BUF_TYPE_STILL_CAPTURE:
			case V4L2_BUF_TYPE_VIDEO_CAPTURE:
				{
					if (cam->cam_sensor->sensor_type == SENSOR_RAW) {
					/*
					 * Use ISP capabilities to crop 
					 */
					 	if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
							crop->c = ispcroprect_a;
					 	else
							crop->c = ispcroprect_b;
					 	return 0;
					} else {
						if (cam->cam_sensor->get_crop) {
							return cam->cam_sensor->get_crop(crop,
								  cam->sensor);
						}
					}
					/* The sensor doesn't support cropping.
					 * We don't support it either.
					 */
					return -EINVAL;
				}

			default:
				{
					return -EINVAL;
				}
			}
		}

	case VIDIOC_S_CROP:
		{
			struct v4l2_crop *crop = (struct v4l2_crop *) arg;
			struct v4l2_pix_format *pix = &cam->pix;
			struct v4l2_pix_format *pix2 = &cam->pix2;
			switch (crop->type) {
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				{
					spin_lock(&cam->img_lock);
					if (cam->previewing) {
						spin_unlock(&cam->img_lock);
						return -EBUSY;
					}
					err = omap24xxvout_new_crop(&cam->pix,
								    &cam->preview_crop,
								    &cam->win,
								    &cam->fbuf,
								    &crop->c);
					spin_unlock(&cam->img_lock);
					return err;
				}

			case V4L2_BUF_TYPE_STILL_CAPTURE:
			case V4L2_BUF_TYPE_VIDEO_CAPTURE:
				{
					if (cam->cam_sensor->sensor_type == SENSOR_RAW){
#ifdef USE_ISP_RESZ
						if (crop->type == V4L2_BUF_TYPE_VIDEO_CAPTURE){
							if ((crop->c.left + crop->c.width) > pix->width)
								return -EINVAL;

							if ((crop->c.top + crop->c.height) > pix->height)
								return -EINVAL;

							ispcroprect_a.left = crop->c.left;
							ispcroprect_a.top = crop->c.top;

							ispcroprect_a.width = crop->c.width;
							ispcroprect_a.height = crop->c.height;

							omap34xxcamisp_config_resizercrop(cam,1);
							applyCrop = 1;
						} else {
							if ((crop->c.left + crop->c.width) > pix2->width)
								return -EINVAL;

							if ((crop->c.top + crop->c.height) > pix2->height)
								return -EINVAL;

							ispcroprect_b.left = crop->c.left;
							ispcroprect_b.top = crop->c.top;
							ispcroprect_b.width = crop->c.width;						
							ispcroprect_b.height = crop->c.height;
					    omap34xxcamisp_config_pipeline(cam, 2);
					  }
						return 0;
#else
					return -EINVAL;
#endif
				    	} else {				
						if (cam->cam_sensor->set_crop) {
							err = cam->cam_sensor->
							set_crop(crop, cam->sensor);
							return err;
						}
					}
					/* The sensor doesn't support cropping.
					 * We don't support it either.
					 */
					return -EINVAL;
				}

			default:
				{
					return -EINVAL;
				}
			}
		}

	case VIDIOC_G_PARM:
		{
			struct v4l2_streamparm *parm =
			    (struct v4l2_streamparm *) arg;
			enum v4l2_buf_type type = parm->type;

			memset(parm, 0, sizeof (*parm));
			parm->type = type;
			if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
				return -EINVAL;
			spin_lock(&cam->img_lock);
			parm->parm.capture = cam->cparm;
			spin_unlock(&cam->img_lock);
			return 0;
		}

	case VIDIOC_S_PARM:
		{
			struct v4l2_streamparm *parm =
			    (struct v4l2_streamparm *) arg;
			struct v4l2_captureparm *cparm = &parm->parm.capture;

			if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
				return -EINVAL;
			spin_lock(&cam->img_lock);
			if (cam->streaming || cam->previewing) {
				spin_unlock(&cam->img_lock);
				return -EBUSY;
			}
			cam->cparm.capturemode = cparm->capturemode;
			if (cparm->timeperframe.numerator
			    && cparm->timeperframe.denominator) {
				cam->nominal_timeperframe = cparm->timeperframe;
				/* adjust the capture frame rate */
				cam->xclk =
				    cam->cam_sensor->calc_xclk(
				    	(cam->cam_sensor->sensor_type == SENSOR_RAW)?
				    	&cam->pix_raw : &cam->pix,
					&cam->nominal_timeperframe,
					cam->sensor);
				cam->xclk = isp_negotiate_xclka(cam->xclk);
				cam->cparm.timeperframe =
				    cam->nominal_timeperframe;
				spin_unlock(&cam->img_lock);

				/* program xclk */
				cam->xclk = isp_set_xclka(cam->xclk);

				/* program the sensor */
				err =
				    cam->cam_sensor->configure(
				    	(cam->cam_sensor->sensor_type == SENSOR_RAW)?
				    	&cam->pix_raw : &cam->pix,
					cam->xclk,
					&cam->cparm.timeperframe,
					cam->sensor);
				cparm->timeperframe = cam->cparm.timeperframe;
			} else {
				spin_unlock(&cam->img_lock);
			}
			return 0;
		}

	case VIDIOC_G_OVERLAY_ROT:
		{
			int *rotation = arg;
			
			spin_lock(&cam->img_lock);
			*rotation = cam->overlay_rotation;
			spin_unlock(&cam->img_lock);
			return 0;
		}
	case VIDIOC_S_OVERLAY_ROT:
		{
			int *rotation = arg;
			
			if (*rotation < PREVIEW_ROTATION_NO ||
			    *rotation > PREVIEW_ROTATION_270)
			    return -EINVAL;

			spin_lock(&cam->img_lock);
			if (*rotation == cam->overlay_rotation) {
				spin_unlock(&cam->img_lock);
				return 0;
			}
			if (cam->streaming || cam->previewing) {
				spin_unlock(&cam->img_lock);
				return -EBUSY;
			}

			/* set the new rottaion mode */
			cam->overlay_rotation = *rotation;

			/* we have to adjust the preview crop and window */
			/* set a default display window and preview crop */
			omap34xxcam_preview(cam);

			spin_unlock(&cam->img_lock);
			return 0;
		}
	case VIDIOC_OVERLAY:
		{
			int *on = arg;
			int vrfb_pixelsize = 2;
			int rotation, dss_dma_start;
			int outputoffset;

			if(*on){
				if(isp_temp_ispmmu_addr){
					ispmmu_unmap(isp_temp_ispmmu_addr);
					isp_temp_ispmmu_addr = 0;
				}
			} else {
				if(!isp_temp_ispmmu_addr)
					isp_temp_ispmmu_addr = ispmmu_map(isp_temp_phy_addr,D1_SIZE);
			}
			spin_lock(&cam->img_lock);
			/*
			 * We do not allow previewing any format that is not
			 * supported by OMAP DSS.
			 */
			if (cam->pix.pixelformat != V4L2_PIX_FMT_YUYV &&
			    cam->pix.pixelformat != V4L2_PIX_FMT_UYVY &&
			    cam->pix.pixelformat != V4L2_PIX_FMT_RGB565 &&
			    cam->pix.pixelformat != V4L2_PIX_FMT_RGB565X){
				spin_unlock(&cam->img_lock);
				return -EINVAL;
			}
			/*
			 * We design the driver in such a way that video preview
			 * and video capture are mutually exclusive.
			 */
			if (!(cam->previewing || cam->previewing) && *on) {
				if (cam->pix.sizeimage <= cam->overlay_size) {
					/* YUV images require twice as much memory as RGB
					 * per VRFB requirement. So the max preview YUV
					 * image is smaller than RGB image.
					 */
					if ((V4L2_PIX_FMT_YUYV == cam->pix.pixelformat ||
					    V4L2_PIX_FMT_UYVY == cam->pix.pixelformat) &&
					    cam->overlay_rotation) {
					    if (cam->pix.sizeimage*2 > cam->overlay_size) {
					    	spin_unlock(&cam->img_lock);
						return -EINVAL;
					    }
					    vrfb_pixelsize <<= 1;
					}
					/* V1 is the default for preview */
					cam->vid_preview = (*on == 2) ?
					    OMAP2_VIDEO2 : OMAP2_VIDEO1;
					if (!omap2_disp_request_layer
					    (cam->vid_preview)) {
						spin_unlock(&cam->img_lock);
						return -EBUSY;
					}
					omap2_disp_get_dss();
					
					DPRINTK_CAM("Setting constraint for VDD2\n");
					constraint_set(co_opp_camera_vdd2,
						CO_VDD2_OPP3);
					constraint_set(co_opp_camera_latency,
						CO_LATENCY_MPURET_COREON);
					
					
					if (cam->overlay_rotation) {
						/* re-calculate sms_rot_phy[] */
						int i, base = VRFB_BASE +
							cam->vid_preview*VRFB_CONTEXT_SIZE;
						for (i = 0; i < 4; i++)
							sms_rot_phy[i] = base +
								i*VRFB_ANGLE_SIZE;

						rotation = (cam->overlay_rotation-1)*90;
						if (rotation == 90 || rotation == 270) {
							if (rotation == 90)
								rotation = 270;
							else
								rotation = 90;

							omap2_disp_set_vrfb(cam->vid_preview,
							cam->overlay_base_phys, cam->pix.height,
							cam->pix.width,vrfb_pixelsize);

						}
						else
							omap2_disp_set_vrfb(cam->vid_preview,
							cam->overlay_base_phys, cam->pix.width,
							cam->pix.height,vrfb_pixelsize);

						cam->overlay_base_dma =
							sms_rot_phy[rotation/90];
						
						dss_dma_start = sms_rot_phy[0];
						
					}
					else {
						cam->overlay_base_dma = cam->overlay_base_phys;
						dss_dma_start = cam->overlay_base_phys;
						rotation = -1;
					}

					omap34xxcamisp_configure_interface(cam, 1);
					omap34xxcamisp_calc_pipeline(cam);
					#ifndef CONFIG_ARCH_OMAP3410
					isph3a_notify(0);
					#endif          
					if (cam->cam_sensor->sensor_type == SENSOR_RAW)
						omap34xxcamisp_config_pipeline(cam, 1);
					else
						omap34xxcam_config_pipeline(cam, 1);

					/* does mapping for overlay buffer */
					sg_dma_address(&cam->overlay_sglist) = cam->overlay_base_dma;

					/* Configure the ISP MMU to map enough 
					memory for VRFB memory space for rotation*/					

					if(cam->overlay_rotation)
						sg_dma_len(&cam->overlay_sglist) = 2048*2048*vrfb_pixelsize;
					else
						sg_dma_len(&cam->overlay_sglist) = cam->overlay_size;						

					cam->isp_addr_overlay = ispmmu_map(
						sg_dma_address(&cam->overlay_sglist), 
						sg_dma_len(&cam->overlay_sglist));


					/* Set output offset to maximum number of bytes per line 
					in VRFB inorder to allow 2D addressing */

					outputoffset = 2048*vrfb_pixelsize;
					
					/* Lets configure offset here */
					
					if(cam->overlay_rotation){
#ifdef USE_ISP_RESZ						
						if (cam->isp_pipeline & OMAP_ISP_RESIZER)
							ispresizer_config_outlineoffset(outputoffset);
						else
#endif
#ifdef USE_ISP_PREVIEW
						if (cam->isp_pipeline & OMAP_ISP_PREVIEW)
							isppreview_config_outlineoffset(outputoffset);
						else
#endif
						if (cam->isp_pipeline & OMAP_ISP_CCDC)
							ispccdc_config_outlineoffset(outputoffset, 0, 0);
						}


					/* Turn on the overlay window */
					omap2_disp_config_vlayer(cam->vid_preview,
								 &cam->pix,
								 &cam->preview_crop,
								 &cam->win,
								 rotation,
								 0);
					omap2_disp_start_vlayer(cam->vid_preview,
								&cam->pix,
								&cam->preview_crop,
								&cam->win,
								dss_dma_start,
								rotation,
								0);
					cam->previewing = fh;
					spin_unlock(&cam->img_lock);

#if 0	//TODO: revisit
					ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);
#if 1
					isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_MEM);
#else //To be enabled
					isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_RSZ);
		      			ispreszier_config_datapath(RSZ_OTFLY_YUV);
#endif
#endif //0

					/* start the camera interface */
					cam->dma_notify = 1;
					omap34xxcam_start_overlay_dma(cam);
				} else {
					/* Image size is bigger than overlay
					 * buffer.
					 */
					spin_unlock(&cam->img_lock);
					return -EINVAL;
				}
			} else if (cam->previewing && !*on) {
				/* turn overlay off */
				omap2_disp_disable_layer(cam->vid_preview);
				omap2_disp_release_layer(cam->vid_preview);
				omap2_disp_put_dss();
				cam->overlay_cnt = 0;
	      			cam->dma_notify = 0;
				cam->previewing = NULL;
				omap34xxcam_stop_isp(cam);
				DPRINTK_CAM("Removing constraint for VDD2\n");
				constraint_remove(co_opp_camera_vdd2);
				constraint_remove(co_opp_camera_latency);
				spin_unlock(&cam->img_lock);
				ispmmu_unmap(cam->isp_addr_overlay);
			} else
				spin_unlock(&cam->img_lock);

			return 0;
		}

	case VIDIOC_REQBUFS:
		return videobuf_reqbufs(&fh->vbq, arg);

	case VIDIOC_QUERYBUF:
		return videobuf_querybuf(&fh->vbq, arg);

	case VIDIOC_QBUF:
		return videobuf_qbuf(&fh->vbq, arg);

	case VIDIOC_DQBUF:
		return videobuf_dqbuf(&fh->vbq, arg, file->f_flags & O_NONBLOCK);

	case VIDIOC_STREAMON:
		{
			spin_lock(&cam->img_lock);
			if (cam->streaming || cam->previewing) {
				spin_unlock(&cam->img_lock);
				return -EBUSY;
			} else
				cam->streaming = fh;
			spin_unlock(&cam->img_lock);
		#ifndef CONFIG_ARCH_OMAP3410
      			isph3a_notify(0);
		#endif      		
      			
			if (cam->pix.width >= 640 && cam->pix.height >= 480 ) {
				DPRINTK_CAM("Setting constraint for VDD1\n");
			 	constraint_set(co_opp_camera_vdd1, CO_VDD1_OPP3);
				
			}
			DPRINTK_CAM("Setting constraint for VDD2\n");
			constraint_set(co_opp_camera_vdd2, CO_VDD2_OPP3);
			constraint_set(co_opp_camera_latency, CO_LATENCY_MPURET_COREON);
		
			
			omap34xxcamisp_configure_interface(cam, 1);
			omap34xxcamisp_calc_pipeline(cam);
			
			if (cam->cam_sensor->sensor_type == SENSOR_RAW)
				omap34xxcamisp_config_pipeline(cam, 1);
			else
				omap34xxcam_config_pipeline(cam, 1);
			cam->dma_notify = 1;
			return videobuf_streamon(&fh->vbq);
		}

	case VIDIOC_STREAMOFF:
		{
			struct videobuf_queue *q = &fh->vbq;
			int i, err;

			/* video-buf lib has trouble to turn off streaming while
			   any buffer is still in QUEUED state. Let's wait until
			   all queued buffers are filled.
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
			omap34xxcam_stop_isp(cam);

		#ifndef CONFIG_ARCH_OMAP3410      
      			isph3a_notify(1);
		#endif
			isp_temp_state = 0;
			spin_lock(&cam->img_lock);
			if (cam->streaming == fh)
				cam->streaming = NULL;
			spin_unlock(&cam->img_lock);

			for (i = 0; i < VIDEO_MAX_FRAME; i++) {
				if (NULL == q->bufs[i])
					continue;
				if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
					omap34xxcam_vbq_release(q,
								q->bufs[i]);
			}
			cam->dma_notify = 0;
			videobuf_streamoff(q);
			if (cam->pix.width >= 640 && cam->pix.height >= 480 ) {
				DPRINTK_CAM("Removing constraint for VDD1 \n");
				constraint_remove(co_opp_camera_vdd1);
			}
			DPRINTK_CAM("Removing constraint for VDD2 \n");
			constraint_remove(co_opp_camera_vdd2);
			constraint_remove(co_opp_camera_latency);
      
			return 0;
		}

	case VIDIOC_G_CTRL:
	  {
		struct v4l2_control *vc = arg;
		u8 current_value;

		if(cam->cam_sensor->sensor_type == SENSOR_RAW) {
#ifdef USE_ISP_PREVIEW
			if(vc->id == V4L2_CID_BRIGHTNESS) {
				isppreview_query_brightness(&current_value);
				vc->value = current_value/ISPPRV_BRIGHT_UNITS;
				return 0;
			} else if(vc->id == V4L2_CID_CONTRAST) {
				isppreview_query_contrast(&current_value);
				vc->value = current_value/ISPPRV_CONTRAST_UNITS;
				return 0;
			} else if(vc->id == V4L2_CID_PRIVATE_BASE) {
				isppreview_get_color(&current_value);
				vc->value = current_value;
				return 0;					
			} else
#endif
				return -EINVAL;
		}
		return cam->cam_sensor->get_control(vc, cam->sensor);
	  }
	case VIDIOC_S_CTRL:
	  {
		struct v4l2_control *vc = arg;
		u8 new_value;

		if(cam->cam_sensor->sensor_type == SENSOR_RAW) {
#ifdef USE_ISP_PREVIEW
			if(vc->id == V4L2_CID_BRIGHTNESS) {
				new_value = vc->value;
				if(new_value > ISPPRV_BRIGHT_HIGH)
					return -EINVAL;
				isppreview_update_brightness(&new_value);
				return 0;
			} else if(vc->id == V4L2_CID_CONTRAST) {
				new_value = vc->value;
				if(new_value > ISPPRV_CONTRAST_HIGH)
					return -EINVAL;
				isppreview_update_contrast(&new_value);
				return 0;
			} else if(vc->id == V4L2_CID_PRIVATE_BASE) {
				new_value = vc->value;
				if(new_value > PREV_SEPIA_COLOR)
					return -EINVAL;
				isppreview_set_color(&new_value);
				return 0;
			} else
#endif
				return -EINVAL;
		}
		return cam->cam_sensor->set_control(vc, cam->sensor);
	  }
	case VIDIOC_QUERYCTRL:
	  {
		struct v4l2_queryctrl *qc = arg;
		if(cam->cam_sensor->sensor_type == SENSOR_RAW) {
			if(qc->id == V4L2_CID_BRIGHTNESS) {
				*qc = ispbccontrol[0];
				return 0;
			} else if(qc->id == V4L2_CID_CONTRAST) {
				*qc = ispbccontrol[1];
				return 0;
			} else if(qc->id == V4L2_CID_PRIVATE_BASE) {
				*qc = ispbccontrol[2];
				return 0;
			} else
			  return -EINVAL;
		}
		return cam->cam_sensor->query_control(qc, cam->sensor);
	  }
	case VIDIOC_QUERYMENU:
		{
			return -EINVAL;
		}

	case VIDIOC_ENUMSTD:
	case VIDIOC_G_STD:
	case VIDIOC_S_STD:
	case VIDIOC_QUERYSTD:
		{
			/* Digital cameras don't have an analog video standard,
			 * so we don't need to implement these ioctls.
			 */
			return -EINVAL;
		}

	case VIDIOC_G_AUDIO:
	case VIDIOC_S_AUDIO:
	case VIDIOC_G_AUDOUT:
	case VIDIOC_S_AUDOUT:
		{
			/* we don't have any audio inputs or outputs */
			return -EINVAL;
		}

	case VIDIOC_G_JPEGCOMP:
	case VIDIOC_S_JPEGCOMP:
		{
			/* JPEG compression is not supported */
			return -EINVAL;
		}

	case VIDIOC_G_TUNER:
	case VIDIOC_S_TUNER:
	case VIDIOC_G_MODULATOR:
	case VIDIOC_S_MODULATOR:
	case VIDIOC_G_FREQUENCY:
	case VIDIOC_S_FREQUENCY:
		{
			/* we don't have a tuner or modulator */
			return -EINVAL;
		}

	case VIDIOC_ENUMOUTPUT:
	case VIDIOC_G_OUTPUT:
	case VIDIOC_S_OUTPUT:
		{
			/* we don't have any video outputs */
			return -EINVAL;
		}

	case VIDIOC_ISP_2ACFG:
		{
#ifndef CONFIG_ARCH_OMAP3410
			return isph3a_aewb_configure(arg);
#else
			return -EINVAL;
#endif		

		}

	case VIDIOC_ISP_HIST_CFG:
		{ 
#ifndef CONFIG_ARCH_OMAP3410
			return isp_hist_configure(arg);
#else
			return -EINVAL;
#endif		
		}

	case VIDIOC_ISP_2AREQ:
		{
#ifndef CONFIG_ARCH_OMAP3410
			return isph3a_aewb_request_statistics(arg);
#else
			return -EINVAL;
#endif		
		}
    		
	case VIDIOC_ISP_HISTREQ:
		{
#ifndef CONFIG_ARCH_OMAP3410
	    		return isp_hist_request_statistics(arg);
#else
			return -EINVAL;
#endif		
		}
	case VIDIOC_ISP_STANDBY:
		{	
			int *on = arg;
			if ( *on == 0) {
				isp_get();
	    			return 0;
			} else if ( *on == 1) {
				isp_put();
				return 0;
			} else 
				return -EINVAL;
		}

	default:
		{
			/* unrecognized ioctl */
			return -ENOIOCTLCMD;
		}
	}
	
	return 0;
}

/* -------------------------------------------------------------------------- */

	/*
	 *  file operations
	 */

static unsigned int
omap34xxcam_poll(struct file *file, struct poll_table_struct *wait)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;
	struct videobuf_buffer *vb;
	enum v4l2_field field;

	omap34xxcam_suspend_lockout(cam, file);

	spin_lock(&cam->img_lock);

	if (cam->streaming == fh) {
		spin_unlock(&cam->img_lock);
		/* streaming capture */
		if (list_empty(&fh->vbq.stream))
			return POLLERR;
		vb = list_entry(fh->vbq.stream.next, struct videobuf_buffer,
				stream);
	} else if (cam->streaming) {
		/* streaming I/O is in progress on another file descriptor */
		spin_unlock(&cam->img_lock);
		return POLLERR;
	} else {
		/* read() capture */
		spin_unlock(&cam->img_lock);
		mutex_lock(&fh->vbq.lock);
		cam->still_capture = 1;
		if (fh->vbq.read_buf == NULL) {
			/* need to capture a new image */
			fh->vbq.read_buf = videobuf_alloc(fh->vbq.msize);
			if (fh->vbq.read_buf == NULL) {
				mutex_unlock(&fh->vbq.lock);
				return POLLERR;
			}
			fh->vbq.read_buf->memory = V4L2_MEMORY_USERPTR;
			field = videobuf_next_field(&fh->vbq);
			if (fh->vbq.ops->buf_prepare(&fh->vbq, fh->vbq.read_buf,
						     field) != 0) {
				mutex_unlock(&fh->vbq.lock);
				return POLLERR;
			}

			omap34xxcamisp_calc_pipeline(cam);
			cam->dma_notify = 1;
			fh->vbq.ops->buf_queue(&fh->vbq, fh->vbq.read_buf);
			fh->vbq.read_off = 0;
		}
		mutex_unlock(&fh->vbq.lock);
		vb = (struct videobuf_buffer *) fh->vbq.read_buf;
	}

	poll_wait(file, &vb->done, wait);
	if (vb->state == STATE_DONE || vb->state == STATE_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

static ssize_t
omap34xxcam_read(struct file *file, char *data, size_t count, loff_t * ppos)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;
	struct omap34xxcam_fh *preview_fh;
	unsigned long irqflags;
	int free_sgdma, err;

	omap34xxcam_suspend_lockout(cam, file);

	/* user buffer has to be word aligned */
	if (((unsigned int) data & 0x3) != 0)
		return -EIO;

	spin_lock(&cam->img_lock);
	if (cam->streaming) {
		spin_unlock(&cam->img_lock);
		return -EBUSY;
	}

      	omap34xxcamisp_configure_interface(cam, 2);
	preview_fh = NULL;
	if (cam->previewing) {
		preview_fh = cam->previewing;
		/* stop preview */
		cam->previewing = NULL;
		spin_unlock(&cam->img_lock);
		/* We need wait until sgdmas used by preview are freed.
		 * To minimize the shot-to-shot delay, we don't want to
		 * yield. Just want to start one-shot capture as soon as
		 * possible. An alternative is to stop the dma but the
		 * sync error would require a reset of the camera system.
		 */
		do {
			/* prevent the race with dma handler */
			spin_lock_irqsave(&cam->sg_lock, irqflags);
			free_sgdma = cam->free_sgdma;
			spin_unlock_irqrestore(&cam->sg_lock, irqflags);
		} while (NUM_SG_DMA != free_sgdma);

		spin_lock(&cam->img_lock);
	}
	cam->still_capture = 1;
	spin_unlock(&cam->img_lock);

	DPRINTK_CAM("Setting constraint for VDD2\n");
	constraint_set(co_opp_camera_vdd2, CO_VDD2_OPP3);
	constraint_set(co_opp_camera_latency, CO_LATENCY_MPURET_COREON);


	if (cam->cam_sensor->enter_still_capture)
		cam->cam_sensor->enter_still_capture(16, cam->sensor);

	omap34xxcamisp_calc_pipeline(cam);
	if (cam->cam_sensor->sensor_type == SENSOR_RAW)
		omap34xxcamisp_config_pipeline(cam, 2);
	else /* ISP sensor */
		omap34xxcam_config_pipeline(cam, 2);

	cam->dma_notify = 1;
	if (((unsigned int) data & (32 - 1)) == 0) {
		/* use zero-copy if user buffer is aligned at 32-byte */
		err = videobuf_read_one(&fh->vbq, data, count, ppos, file->f_flags & O_NONBLOCK);
	} else {
		/* if user buffer is not aligned at 32-byte, we will use kernel
		   bounce buffer to capture the image pretending that we want to
		   read one pixel less than the actual image size.
		 */
		err = videobuf_read_one(&fh->vbq, data, count - 2, ppos, file->f_flags & O_NONBLOCK);
	}
	spin_lock(&cam->img_lock);
	cam->still_capture = 0;
	spin_unlock(&cam->img_lock);

	if (cam->cam_sensor->exit_still_capture)
		cam->cam_sensor->exit_still_capture(cam->sensor);

	/* if previwing was on, re-start it after the read */
	if (preview_fh) { /* was previewing */
		spin_lock(&cam->img_lock);
		cam->previewing = preview_fh;
		spin_unlock(&cam->img_lock);
      		omap34xxcamisp_configure_interface(cam, 1);
		omap34xxcamisp_calc_pipeline(cam);
		if (cam->cam_sensor->sensor_type == SENSOR_RAW)
			omap34xxcamisp_config_pipeline(cam, 1);
		else /* ISP sensor */
			omap34xxcam_config_pipeline(cam, 1);
		
		cam->dma_notify = 1;
		omap34xxcam_start_overlay_dma(cam);
	}
	DPRINTK_CAM("Removing constraint for VDD2\n");
	constraint_remove(co_opp_camera_vdd2);
	constraint_remove(co_opp_camera_latency);
	return err;
}

static int
omap34xxcam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;

	omap34xxcam_suspend_lockout(cam, file);
	return videobuf_mmap_mapper(&fh->vbq, vma);
}

static int
omap34xxcam_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		  unsigned long arg)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;

	omap34xxcam_suspend_lockout(cam, file);
	return video_usercopy(inode, file, cmd, arg, omap34xxcam_do_ioctl);
}

static int
omap34xxcam_release(struct inode *inode, struct file *file)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_device *cam = fh->cam;
	struct videobuf_queue *q = &fh->vbq;
	int i;

	omap34xxcam_suspend_lockout(cam, file);

	spin_lock(&cam->img_lock);
	/* turn off overlay */
	if (cam->previewing == fh) {
		cam->previewing = NULL;
		omap34xxcam_stop_isp(cam);
		cam->overlay_cnt = 0;
		spin_unlock(&cam->img_lock);
		omap2_disp_disable_layer(cam->vid_preview);
		omap2_disp_release_layer(cam->vid_preview);
		omap2_disp_put_dss();
		spin_lock(&cam->img_lock);
		ispmmu_unmap(cam->isp_addr_overlay);
	}

	/* stop streaming capture */
	if (cam->streaming == fh) {
		cam->streaming = NULL;
		spin_unlock(&cam->img_lock);
		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
				omap34xxcam_vbq_release(q, q->bufs[i]);
		}
		videobuf_streamoff(q);
		spin_lock(&cam->img_lock);
	}
	cam->free_sgdma = NUM_SG_DMA;
	spin_unlock(&cam->img_lock);

	/* release read_buf videobuf_buffer struct */
	if (fh->vbq.read_buf) {
		omap34xxcam_vbq_release(q, fh->vbq.read_buf);
		kfree(fh->vbq.read_buf);
	}

	/* free video_buffer objects */
	videobuf_mmap_free(q);

	file->private_data = NULL;
	kfree(fh);
#ifndef CONFIG_ARCH_OMAP3410
        isph3a_notify(1);
#endif
	ispccdc_free();
	if (cam->cam_sensor->sensor_type == SENSOR_RAW) {
#ifdef USE_ISP_PREVIEW
		isppreview_free();
#endif
#ifdef USE_ISP_RESZ
		ispresizer_free();
#endif
	}

	isp_put();
	isp_temp_state = ISP_BUF_INIT;
	return 0;
}

void omap34xxcam_sensor_restore(void)
{
	struct omap34xxcam_device *cam = saved_cam;
	if (sensor_init && cam->cam_sensor->restore)
		cam->cam_sensor->restore(cam->sensor);
}
EXPORT_SYMBOL(omap34xxcam_sensor_restore);

static int
omap34xxcam_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct omap34xxcam_device *cam = saved_cam;
	struct omap34xxcam_fh *fh;

	if (!cam || !cam->vfd || (cam->vfd->minor != minor))
		return -ENODEV;

	isp_temp_state = ISP_BUF_INIT;
	omap34xxcam_suspend_lockout(cam, file);

	isp_get();

	/* we request the needed ISP resources in one place */
	if (ispccdc_request()) {
		isp_put();
		return -EBUSY;
	}

	if (cam->cam_sensor->sensor_type == SENSOR_RAW) {
#ifdef USE_ISP_PREVIEW
		if (isppreview_request()) {
			ispccdc_free();
			isp_put();
			return -EBUSY;
		}
#endif
#ifdef USE_ISP_RESZ
		if (ispresizer_request()) {
			ispccdc_free();
			isppreview_free();
			isp_put();
			return -EBUSY;
		}
#endif

	}

	
	/* allocate per-filehandle data */
	fh = kmalloc(sizeof (*fh), GFP_KERNEL);
	if (NULL == fh) {
		ispccdc_free();
		if (cam->cam_sensor->sensor_type == SENSOR_RAW) {
#ifdef USE_ISP_PREVIEW
			isppreview_free();
#endif
#ifdef USE_ISP_RESZ
			ispresizer_free();
#endif
		}
		isp_put();
		return -ENOMEM;
	}
	file->private_data = fh;
	fh->cam = cam;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	videobuf_queue_init(&fh->vbq, &cam->vbq_ops, NULL, &cam->vbq_lock,
			    fh->type, V4L2_FIELD_NONE,
			    sizeof (struct videobuf_buffer), fh);

	if(!isp_temp_ispmmu_addr)
		isp_temp_ispmmu_addr = ispmmu_map(isp_temp_phy_addr,D1_SIZE);

	return 0;
}

static struct file_operations omap34xxcam_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = omap34xxcam_read,
	.poll = omap34xxcam_poll,
	.ioctl = omap34xxcam_ioctl,
	.mmap = omap34xxcam_mmap,
	.open = omap34xxcam_open,
	.release = omap34xxcam_release,
};

/* -------------------------------------------------------------------------- */
#ifdef CONFIG_PM
static int
omap34xxcam_suspend(struct platform_device *dev, pm_message_t state)
{
	struct omap34xxcam_device *cam = platform_get_drvdata(dev);
	if (cam->suspended)
		return 0;
	/*So as to turn off the xclka if the camera is not running.*/
	if(!(cam->previewing || cam->streaming || cam->still_capture))
		isp_get();
	/* lock-out applications during suspend */
	cam->suspended = 1;
	/* stall previewing */
	spin_lock(&cam->img_lock);
	if (cam->previewing) {
		omap2_disp_disable_layer(cam->vid_preview);
		/* we still hold the video layer */
		omap2_disp_put_dss();
	}
	spin_unlock(&cam->img_lock);
	/* ???what else is needed to suspend isp??? */
	if(cam->streaming || cam->previewing || cam->still_capture){
		omap34xxcam_stop_isp(cam);
	}

	/* power down the sensor */
	if (cam->sensor)
		cam->cam_sensor->power_off(cam->sensor);
	
	/* stop XCLK */
	isp_set_xclka(0);

	isp_put();
	isp_temp_state = ISP_BUF_INIT;
#ifndef CONFIG_ARCH_OMAP3410
        isph3a_notify(1);
#endif     
	return 0;
}
static int
omap34xxcam_resume(struct platform_device *dev)
{
	struct omap34xxcam_device *cam = platform_get_drvdata(dev);
	if (!cam->suspended)
		return 0;

	isp_get();

	/* Recalculate the last xclk used */
	cam->xclk =
	  cam->cam_sensor->calc_xclk(&cam->pix,
		  		     &cam->
				     nominal_timeperframe,
				     cam->
				     sensor);

	/* set XCLK */
	isp_set_xclka(cam->xclk);

	/* power up the sensor */
	if (cam->sensor)
	cam->cam_sensor->power_on(cam->sensor);

	if(cam->streaming || cam->previewing || cam->still_capture){
		/* ???what else is needed to resume isp??? */
		if (cam->free_sgdma > NUM_SG_DMA)
			cam->free_sgdma = NUM_SG_DMA;

		if (cam->streaming || cam->previewing) {
			/* capture or previewing was in progress, so we need to register
			 * our routine to restart the camera interface the next time a
			 * DMA transfer is queued.
			 */
			omap34xxcamisp_calc_pipeline(cam);
		        if (cam->cam_sensor->sensor_type == SENSOR_RAW)
			  omap34xxcamisp_config_pipeline(cam, 1);
			else /* ISP sensor */
			  omap34xxcam_config_pipeline(cam, 1);

			cam->dma_notify = 1;
		}
		if (cam->previewing) {
			if (cam->overlay_cnt > 0)
				--cam->overlay_cnt;
			omap2_disp_get_dss();
			omap2_disp_enable_layer(cam->vid_preview);
			omap34xxcam_start_overlay_dma(cam);
		}
		if (cam->streaming) {
		  omap34xxcam_sg_dma_process(cam,0);
		}
	}
	else{
		isp_put();
	}
	/* camera interface will be enabled through dma_notify function
	 ** automatically when new dma starts
	 */

	/* wake up applications waiting on suspend queue */
	cam->suspended = 0;
	wake_up(&cam->suspend_wq);
#ifndef CONFIG_ARCH_OMAP3410
        isph3a_notify(0);
#endif
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
omap34xxcamisp_config_ispsensor(struct omap34xxcam_device *cam)
{
	u32 w = cam->pix.width;
	u32 h = cam->pix.height;

	ispccdc_config_datapath(CCDC_YUV_SYNC, CCDC_OTHERS_MEM);
	ispccdc_try_size(w, h, &cam->pix.width, &cam->pix.height);
	printk("The agreed initial preview size (ISP sensor):\n");
	printk("\tsensor ouput (%u, %u)\n", w, h);
	printk("\tCCDC output (%u, %u)\n", cam->pix.width, cam->pix.height);
	/* set a new default display window and preview crop */
	omap34xxcam_preview(cam);
	ispccdc_config_size(w, h, cam->pix.width, cam->pix.height);
}
/*
 * Helper function to have needed ISP configuration for raw sensor
 */
static void
omap34xxcamisp_config_rawsensor(struct omap34xxcam_device *cam)
{
#ifdef USE_ISP_PREVIEW
	isppreview_request();
	ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);
#ifdef USE_ISP_RESZ
	ispresizer_request();
	isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_RSZ);
	ispresizer_config_datapath(RSZ_OTFLY_YUV);
#else
	isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_MEM);
#endif
#else
	ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_MEM);
#endif
	/* 
	 * Default we preview QCIF in UYVY:
	 * YCrYCb Y1(31:24)Cr0(23;16)Y0(15:8)Cb0(7:0) 
	 */
#ifndef CONFIG_ARCH_OMAP3410
	cam->pix.pixelformat = V4L2_PIX_FMT_UYVY;
#else
	cam->pix.pixelformat =  V4L2_PIX_FMT_SGRBG10;
#endif
	cam->pix.width = QCIF_WIDTH;
	cam->pix.height = QCIF_HEIGHT;
	omap34xxcamisp_try_format(cam, &cam->pix, 1);

	printk("The agreed initial preview size (Raw sensor):\n");
	printk("\tsensor ouput (%u, %u)\n",
		cam->pix_raw.width, cam->pix_raw.height);
	printk("\tCCDC output (%lu, %lu)\n",
		cam->ccdc_width, cam->ccdc_height);
	printk("\tPreview output (%lu, %lu)\n",
		cam->preview_width, cam->preview_height);
	printk("\tResizer output (%lu, %lu)\n",
		cam->resizer_width, cam->resizer_height);

	cam->pix2.pixelformat = V4L2_PIX_FMT_UYVY;
#ifdef THREEMP_IMAGE_CAPTURE
	cam->pix2.width = QXGA_WIDTH;
	cam->pix2.height = QXGA_HEIGHT;
#else	
	cam->pix2.width = VGA_WIDTH;
	cam->pix2.height = VGA_HEIGHT;
#endif
	ispcroprect_a.left = 0;
	ispcroprect_a.top = 0;
	ispcroprect_a.width = cam->pix.width;						
	ispcroprect_a.height = cam->pix.height;	
	

	/* set a new default display window and preview crop */
	omap34xxcam_preview(cam);

	ispccdc_config_size(cam->pix_raw.width, cam->pix_raw.height,
				cam->ccdc_width, cam->ccdc_height);
#ifdef USE_ISP_PREVIEW
	isppreview_config_size(cam->ccdc_width, cam->ccdc_height,
				cam->preview_width, cam->preview_height);
#ifdef USE_ISP_RESZ
	ispresizer_config_size(cam->preview_width, cam->preview_height,
				cam->resizer_width,cam->resizer_height);
#endif
#endif

	ispcroprect_b.left = 0;
	ispcroprect_b.top = 0;
	ispcroprect_b.width = cam->pix2.width;						
	ispcroprect_b.height = cam->pix2.height;			

	omap34xxcamisp_try_format(cam, &cam->pix2, 2);
	printk("The agreed initial image capture size (Raw sensor):\n");
	printk("\tsensor ouput (%u, %u)\n",
		cam->pix2_raw.width, cam->pix2_raw.height);
	printk("\tCCDC output (%lu, %lu)\n",
		cam->ccdc_width2, cam->ccdc_height2);
	printk("\tPreview output (%lu, %lu)\n",
		cam->preview_width2, cam->preview_height2);
	printk("\tResizer output (%lu, %lu)\n",
		cam->resizer_width2, cam->resizer_height2);
	sensor_init = 1;

#ifdef USE_ISP_PREVIEW
	isppreview_free();
#ifdef USE_ISP_RESZ
	ispresizer_free();
#endif
#endif
}

/*
 * API to be used by sensor drivers to register with camera driver
 */ 
int omap_cam_register_sensor(struct camera_sensor *sensor)
{
	struct omap34xxcam_device *cam = saved_cam;

	isp_get();

	cam->cam_sensor = sensor;
	cam->if_type = ISP_PARLL;
	if (cam->cam_sensor->sensor_interface == SENSOR_SERIAL1)
		cam->if_type = ISP_CSIA;
	else if (cam->cam_sensor->sensor_interface == SENSOR_SERIAL2)
		cam->if_type = ISP_CSIB;
	if (isp_request_interface(cam->if_type)) {
		printk(KERN_ERR CAM_NAME ": cannot get isp interface\n");
		isp_put();
		return -EINVAL;
	}
	/* Enable the xclk output.  The sensor may (and does, in the case of
	 * the OV9640) require an xclk input in order for its initialization
	 * routine to work.
	 */
	/* choose an arbitrary xclk frequency */
	cam->xclk = isp_set_xclka(DEFAULT_SENSOR_XCLK);

	/* initialize the sensor and define a default capture format cam->pix */
	cam->sensor = cam->cam_sensor->init(&cam->pix, &cam->pix2);
	if (!cam->sensor) {
		printk(KERN_ERR CAM_NAME ": cannot initialize sensor\n");
		isp_put();
		return -EINVAL;
	}
	printk(KERN_INFO "Sensor is %s\n", cam->cam_sensor->name);

	/* select an arbitrary default capture frame rate of 15fps */
	cam->nominal_timeperframe.numerator = 1;
	cam->nominal_timeperframe.denominator = 15;
	/* calculate xclk based on the default capture format and default
	 * frame rate
	 */
	cam->xclk = cam->cam_sensor->calc_xclk(&cam->pix,
					       &cam->nominal_timeperframe,
					       cam->sensor);
	cam->cparm.timeperframe = cam->nominal_timeperframe;

	/* negotiate xclk with isp */
	cam->xclk = isp_negotiate_xclka(cam->xclk);
	/* program the agreed new xclk frequency */
	cam->xclk = isp_set_xclka(cam->xclk);

	/* initialize the image preview parameters based on the default capture
	 * format
	 */
	omap24xxvout_new_format(&cam->pix, &cam->fbuf,
			&cam->preview_crop, &cam->win);

	/* program the sensor for the default capture format and rate */
	cam->cam_sensor->configure(&cam->pix, cam->xclk,
				   &cam->cparm.timeperframe, cam->sensor);
	if (cam->cam_sensor->configure_still_capture)
		cam->cam_sensor->configure_still_capture(&cam->pix2, cam->xclk,
				   &cam->cparm.timeperframe, cam->sensor);
	/* configue ISP for the default image */
	ispccdc_request();
	/* for raw sensor, the returned size is not user visile sizes */
	if (cam->cam_sensor->sensor_type == SENSOR_RAW)
		omap34xxcamisp_config_rawsensor(cam);
	else  /* ISP sensor */
		omap34xxcamisp_config_ispsensor(cam);

	ispccdc_free();
	isp_put();
	return 0;
}

/*
 * API to be used by h3a driver to set sensor exposure time
 */ 
int
omap34xxcam_set_exposure_time(int mode, u32 exp_time)
{
	struct omap34xxcam_device *cam = saved_cam;
  return cam->cam_sensor->set_exposure_time(mode, exp_time);
}
EXPORT_SYMBOL(omap34xxcam_set_exposure_time);

/*
 * API to be used by h3a driver to set gain
 */ 
int
omap34xxcam_set_gain(u16 gain)
{
	struct omap34xxcam_device *cam = saved_cam;
  return cam->cam_sensor->set_gain(gain);
}
EXPORT_SYMBOL(omap34xxcam_set_gain);

/*
 * API to unregister sensor from camera driver
 */ 
int omap_cam_unregister_sensor(struct camera_sensor *sensor)
{
	struct omap34xxcam_device *cam = saved_cam;

	if (!sensor)
		return 0;
	isp_free_interface(cam->if_type);

	/* TO DO: unregister just specified sensor */
	if (cam->sensor)
		cam->cam_sensor->cleanup(cam->sensor);
	/* sensor allocated private data is gone */
	cam->sensor = NULL;
	return 0;
}

static void
omap34xxcam_cleanup(void)
{
	struct omap34xxcam_device *cam = saved_cam;
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
			/* The unregister function will release the video_device
			 * struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
			video_device_release(vfd);
		}
		cam->vfd = NULL;
	}

	if (cam->isp_addr_overlay) {
		ispmmu_unmap(cam->isp_addr_overlay);
		cam->isp_addr_overlay = 0;
	}

	if (cam->overlay_base) {
		dma_free_coherent(NULL, cam->overlay_size,
			(void *)cam->overlay_base, cam->overlay_base_phys);
		cam->overlay_base = 0;
	}
	cam->overlay_base = cam->overlay_base_phys = 0;

	if(isp_temp_ispmmu_addr){
	  ispmmu_unmap(isp_temp_ispmmu_addr);
	  isp_temp_ispmmu_addr = 0;
	  isp_temp_phy_addr = 0;
	}
	
	DPRINTK_CAM("Releasing constraint for VDD1 and VDD2\n");
	constraint_put(co_opp_camera_vdd1);
	constraint_put(co_opp_camera_vdd2);
	constraint_put(co_opp_camera_latency);

	platform_device_unregister(&omap34xxcam_dev);
	platform_driver_unregister(&omap34xxcam_driver);

	kfree(cam);
	saved_cam = NULL;
}

/*TODO: to support 2 sensors, move device handing code in init & cleanup to the new
omap_register/unregister_sensor functions */
static int __init
omap34xxcam_init(void)
{
	struct omap34xxcam_device *cam;
	struct video_device *vfd;
	struct isp_sysc isp_sysconfig;
	int ret;

	ret = platform_driver_register(&omap34xxcam_driver);
	if (ret != 0)
		return ret;
	ret = platform_device_register(&omap34xxcam_dev);
	if (ret != 0) {
		platform_driver_unregister(&omap34xxcam_driver);
		return ret;
	}

	cam = kmalloc(sizeof (struct omap34xxcam_device), GFP_KERNEL);
	if (!cam) {
		printk(KERN_ERR CAM_NAME ": could not allocate memory\n");
		platform_device_unregister(&omap34xxcam_dev);
		platform_driver_unregister(&omap34xxcam_driver);
		return -ENOMEM;
	}
	memset(cam, 0, sizeof (struct omap34xxcam_device));
	saved_cam = cam;

	/* set driver specific data to use in power mgmt functions */
	//	omap_set_drvdata(&omap34xxcam_dev, cam);
	platform_set_drvdata(&omap34xxcam_dev, cam);

	cam->suspended = 0;
	init_waitqueue_head(&cam->suspend_wq);

	/* initialize the video_device struct */
	vfd = cam->vfd = video_device_alloc();
	if (!vfd) {
		printk(KERN_ERR CAM_NAME
		       ": could not allocate video device struct\n");
		goto init_error;
	}
	vfd->release = video_device_release;

	strncpy(vfd->name, CAM_NAME, sizeof (vfd->name));
	vfd->type = VID_TYPE_CAPTURE | VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;
	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->hardware = 0;
	vfd->fops = &omap34xxcam_fops;
	video_set_drvdata(vfd, cam);
	vfd->minor = -1;

	/* initialize the videobuf queue ops */
	cam->vbq_ops.buf_setup = omap34xxcam_vbq_setup;
	cam->vbq_ops.buf_prepare = omap34xxcam_vbq_prepare;
	cam->vbq_ops.buf_queue = omap34xxcam_vbq_queue;
	cam->vbq_ops.buf_release = omap34xxcam_vbq_release;
	spin_lock_init(&cam->vbq_lock);

	/* Impose a lower limit on the amount of memory allocated for capture.
	 * We require at least enough memory to double-buffer QVGA (300KB).
	 */
	if (capture_mem < QVGA_SIZE * 2)
		capture_mem = QVGA_SIZE * 2;

	/* allocate coherent memory for the overlay framebuffer */
	cam->overlay_size = overlay_mem;
	if (cam->overlay_size > 0) {
		cam->overlay_base = (unsigned long) dma_alloc_coherent(NULL,
			cam->overlay_size,
			(dma_addr_t *) &cam->overlay_base_phys,
			GFP_KERNEL | GFP_DMA);
		if (!cam->overlay_base) {
			printk(KERN_ERR CAM_NAME
			       "\n\n\n\n\n\n: cannot allocate overlay framebuffer\n");
			goto init_error;
		}
	}
	memset((void *) cam->overlay_base, 0, cam->overlay_size);

	/* this is a VRFB address in the case of rotation */
	cam->overlay_base_dma = cam->overlay_base_phys;
	printk(KERN_INFO "overlay buffer at 0x%x\n", (unsigned int)cam->overlay_base_dma);

	isp_get();
	isp_sysconfig.reset = 0;
	isp_sysconfig.idle_mode = 1;
	isp_power_settings(isp_sysconfig);

	/* initialize the overlay spinlock  */
	spin_lock_init(&cam->overlay_lock);

	/* initialize the spinlock used to serialize access to the image
	 * parameters
	 */

	spin_lock_init(&cam->img_lock);
	spin_lock_init(&isp_temp_buf_lock);

	isp_temp_virt_addr = cam->overlay_base;
	isp_temp_phy_addr = cam->overlay_base_phys;
       	isp_temp_ispmmu_addr = ispmmu_map(isp_temp_phy_addr,D1_SIZE);

  	/* initialize the streaming capture parameters */
	cam->cparm.readbuffers = 1;
	cam->cparm.capability = V4L2_CAP_TIMEPERFRAME;

	/* get the framebuffer parameters in case the sensor init routine
	 * needs them
	 */
	/* the display controller video layer used for camera preview */
	cam->vid_preview = OMAP2_VIDEO1;
	omap2_disp_get_dss();
	omap2_disp_get_panel_size(omap2_disp_get_output_dev(cam->vid_preview),
				  &(cam->fbuf.fmt.width),
				  &(cam->fbuf.fmt.height));
	omap2_disp_put_dss();

	/* initialize the SGDMA data */
	omap34xxcam_sgdma_init(cam);

	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		printk(KERN_ERR CAM_NAME
		       ": could not register Video for Linux device\n");
		isp_put();
		goto init_error;
	}
	else
		vfd->minor = video_nr;

	printk(KERN_INFO CAM_NAME
	       ": registered device video%d [v4l2]\n", vfd->minor);
	isp_put();
	DPRINTK_CAM("Getting constraint for VDD1 and VDD2\n");
	co_opp_camera_latency = constraint_get("omap34xxcam", &cnstr_id_latency);
	co_opp_camera_vdd1 = constraint_get("omap34xxcam", &cnstr_id_vdd1);
	co_opp_camera_vdd2 = constraint_get("omap34xxcam", &cnstr_id_vdd2);

#undef CAMERA_TEST
#ifdef CAMERA_TEST
	if (cam->pix.sizeimage <= cam->overlay_size) {
		printk(KERN_INFO CAM_NAME ": Camera test--enabling overlay\n");
		isp_get();
		/* does one-time mapping for overlay buffer */
		sg_dma_address(&cam->overlay_sglist) = cam->overlay_base_dma;
		sg_dma_len(&cam->overlay_sglist) = cam->overlay_size;
		cam->isp_addr_overlay =
			ispmmu_map(sg_dma_address(&cam->overlay_sglist), 
				sg_dma_len(&cam->overlay_sglist));

		ispccdc_request();
		if (cam->cam_sensor->sensor_type == SENSOR_RAW) {
			isppreview_request();
			ispresizer_request();
		}
		omap34xxcamisp_configure_interface(cam, 1);
		omap34xxcamisp_config_pipeline(cam, 1);

		omap2_disp_get_dss();
		/* turn on the video overlay */
		omap2_disp_config_vlayer(cam->vid_preview, &cam->pix,
					 &cam->preview_crop, &cam->win, -1, 0);
		omap2_disp_start_vlayer(cam->vid_preview, &cam->pix, 
					&cam->preview_crop, &cam->win,
					cam->overlay_base_dma, -1, 0);
		cam->previewing = (struct omap34xxcam_fh *) 1;
		omap34xxcamisp_calc_pipeline(cam);
		
		/* start the camera interface */
		cam->dma_notify = 1;
		omap34xxcam_start_overlay_dma(cam);

		/* don't free resource since preview runs forever */
	} else {
		printk(KERN_INFO CAM_NAME
		       ": Can't start camera test--overlay buffer too"
		       " small\n");
	}
#endif
	return 0;

init_error:
	omap34xxcam_cleanup();
	return -ENODEV;
}


MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP34xx Video for Linux camera driver");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL_GPL(omap_cam_register_sensor);
EXPORT_SYMBOL_GPL(omap_cam_unregister_sensor);

module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr,
		"Minor number for video device (-1 ==> auto assign)");
module_param(capture_mem, int, 0);
MODULE_PARM_DESC(capture_mem,
	"Maximum amount of memory for capture buffers (default 4800KB)");
module_param(overlay_mem, int, 0);
MODULE_PARM_DESC(overlay_mem,
	"Preview overlay framebuffer size (default 600KB)");

module_init(omap34xxcam_init);
module_exit(omap34xxcam_cleanup);

