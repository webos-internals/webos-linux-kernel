/*
 * drivers/media/video/omap/omap24xxcam.c
 *
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * January 2006 - Modified for new sensor interface.
 * Febuary 2006 - added crop support for digital zoom & view finder.
 * March 2006 - added preview rotation support.
 * Copyright (C) 2006 Texas Instruments, Inc.
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
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-sg.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/arch/display.h>
#include <asm/arch/clock.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/scatterlist.h>
#include <asm/irq.h>
#include <asm/semaphore.h>
#include <asm/cacheflush.h>

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif

#include "omap24xxcam.h"
#include "omap24xxcam_user.h"
#include "sensor_if.h"
#include "omap24xxlib.h"

/* OMAP2420 has 4 VRFB contexts. OMAP2430 has 12.
 * Camera driver uses VRFB Context 1 if video 1 pipeline is chosen for Preview
 * or Context 2 if video 2 pipeline is chosen.
 * sms_rot_phy is recalculated each time before preview starts.
 */ 
static unsigned long sms_rot_phy[4];
#define VRFB_BASE		0x70000000
#define VRFB_CONTEXT_SIZE	0x04000000
#define VRFB_ANGLE_SIZE	0x01000000

#define DEFAULT_CAM_FUNC_CLK 96000000	/* 96MHz */
#define DEFAULT_SENSOR_XCLK  12000000	/* 12MHz */

#define QVGA_SIZE 	320*240*2	/* memory needed for QVGA image */
#define VGA_SIZE 	QVGA_SIZE*4	/* memory needed for VGA image */
#define SXGA_SIZE 	1280*960*2	/* memory needed for SXGA image */

#define CAM_NAME "omap24xxcam"

/* this is the sensor ops implemented by the associated sesnor driver */
extern struct camera_sensor camera_sensor_if;

static void omap24xxcam_cleanup(void);

/* global variables */
static struct omap24xxcam_device *saved_cam;

/* module parameters */
static int video_nr = 0;	/* video device minor (-1 ==> auto assign) */

/* Maximum amount of memory to use for capture buffers.
 * Default is 4800KB, enough to double-buffer SXGA. */
static int capture_mem = SXGA_SIZE * 2;

/* Size of video overlay framebuffer.  This determines the maximum image size
 * that can be previewed.  Default is 600KB, enough for VGA. */
static int overlay_mem = VGA_SIZE;

/* -------------------------------------------------------------------------- */
#ifdef CONFIG_PM
#define omap24xxcam_suspend_lockout(s,f) \
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
#define omap24xxcam_suspend_lockout(s,f) s=s
#endif
/* Set the value of the CC_CTRL register in cam->cc_ctrl that is required to
 * support the currently selected capture format in cam->pix.  The CC_CTRL bits
 * which must be configured are:  NOBT_SYNCHRO, BT_CORRECT, PAR_ORDERCAM,
 * PAR_CLK_POL, NOBT_HS_POL, NOBT_VS_POL, PAR_MODE, and CCP_MODE.  The CC_RST,
 * CC_FRAME_TRIG, and CC_EN bits are actively managed by the driver and should
 * be set to zero by this routine.
 */
static void
omap24xxcam_sensor_cc_ctrl(struct omap24xxcam_device *cam)
{
	struct v4l2_pix_format *pix = cam->still_capture?&cam->pix2:&cam->pix;
	struct camera_sensor *sensor = cam->cam_sensor;
	 
	/* basic mode */
	cam->cc_ctrl = sensor->parallel_mode << CC_CTRL_PAR_MODE_SHIFT;
	/* sync polarity */
	cam->cc_ctrl |= (sensor->hs_polarity << CC_CTRL_NOBT_HS_POL_SHIFT);
	cam->cc_ctrl |= (sensor->vs_polarity << CC_CTRL_NOBT_VS_POL_SHIFT);
	/* based on BT or NOBT */
	if ((sensor->parallel_mode == PAR_MODE_BT8) ||
	  (sensor->parallel_mode == PAR_MODE_BT10))
		/* BT correction enable */
		cam->cc_ctrl |= (sensor->bt_correction ? CC_CTRL_BT_CORRECT : 0);
	else
		/* always use the rising edger to trigger the acquisition
		   in NOBT modes. This is recommended */
		cam->cc_ctrl |= CC_CTRL_NOBT_SYNCHRO;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555:
	default:
		/* These formats need a 16-bit byte swap */
		cam->cc_ctrl |= CC_CTRL_PAR_ORDERCAM;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555X:
		/* These formats don't need a 16-bit byte swap */
		break;
	}
}

/*
 * camera core register I/O routines
 */

static __inline__ u32
cc_reg_in(const struct omap24xxcam_device *cam, u32 offset)
{
	return readl(cam->cam_mmio_base + CC_REG_OFFSET + offset);
}

static __inline__ u32
cc_reg_out(const struct omap24xxcam_device *cam, u32 offset, u32 val)
{
	writel(val, cam->cam_mmio_base + CC_REG_OFFSET + offset);
	return val;
}

static __inline__ u32
cc_reg_merge(const struct omap24xxcam_device *cam, u32 offset,
	     u32 val, u32 mask)
{
	u32 addr = cam->cam_mmio_base + CC_REG_OFFSET + offset;
	u32 new_val = (readl(addr) & ~mask) | (val & mask);

	writel(new_val, addr);
	return new_val;
}

static void
cc_init(const struct omap24xxcam_device *cam, int reset)
{
	/* 2420 has a silicon bug that CC reset doesn't deasset DMA. We have
 	 * to reset cam subsystem to recover from any CC error. For 2420, we
 	 * don't directly reset CC.
 	 */  
	if (cpu_is_omap2420()) {
		cc_reg_out(cam, CC_SYSCONFIG, CC_SYSCONFIG_AUTOIDLE);
		return;
	}

	if (reset) {
		unsigned long timeout;
		/* CC reset works for 2430! */
		cc_reg_out(cam, CC_SYSCONFIG, CC_SYSCONFIG_SOFTRESET);
		/* wait for reset to complete */
		timeout = jiffies + msecs_to_jiffies(10);
		while (!(cc_reg_in(cam, CC_SYSSTATUS) & CC_SYSSTATUS_RESETDONE)
	       		&& time_before(jiffies, timeout)) {
			udelay(10);
		}
		if (unlikely
	    	(!(cc_reg_in(cam, CC_SYSSTATUS) & CC_SYSSTATUS_RESETDONE))) {
			printk(KERN_WARNING CAM_NAME
		       		": timeout waiting for cameracore reset\n");
		}
	}
	cc_reg_merge(cam, CC_SYSCONFIG, CC_SYSCONFIG_AUTOIDLE,
					CC_SYSCONFIG_AUTOIDLE);
}

/*
 * camera DMA register I/O routines
 */

static __inline__ u32
camdma_reg_in(const struct omap24xxcam_device *cam, u32 offset)
{
	return readl(cam->cam_mmio_base + CAMDMA_REG_OFFSET + offset);
}

static __inline__ u32
camdma_reg_out(const struct omap24xxcam_device *cam, u32 offset, u32 val)
{
	writel(val, cam->cam_mmio_base + CAMDMA_REG_OFFSET + offset);
	return val;
}

static __inline__ u32
camdma_reg_merge(const struct omap24xxcam_device *cam, u32 offset,
		 u32 val, u32 mask)
{
	u32 addr = cam->cam_mmio_base + CAMDMA_REG_OFFSET + offset;
	u32 new_val = (readl(addr) & ~mask) | (val & mask);

	writel(new_val, addr);
	return new_val;
}

static void
camdma_init(const struct omap24xxcam_device *cam)
{
	camdma_reg_out(cam, CAMDMA_OCP_SYSCONFIG,
		       CAMDMA_OCP_SYSCONFIG_MIDLEMODE_FSTANDBY
		       | CAMDMA_OCP_SYSCONFIG_SIDLEMODE_FIDLE
		       | CAMDMA_OCP_SYSCONFIG_AUTOIDLE);

	camdma_reg_merge(cam, CAMDMA_GCR, 0x10,
			 CAMDMA_GCR_MAX_CHANNEL_FIFO_DEPTH);
}

/*
 * camera MMU register I/O routines
 */

static __inline__ u32
cammmu_reg_in(const struct omap24xxcam_device *cam, u32 offset)
{
	return readl(cam->cam_mmio_base + CAMMMU_REG_OFFSET + offset);
}

static __inline__ u32
cammmu_reg_out(const struct omap24xxcam_device *cam, u32 offset, u32 val)
{
	writel(val, cam->cam_mmio_base + CAMMMU_REG_OFFSET + offset);
	return val;
}

static __inline__ u32
cammmu_reg_merge(const struct omap24xxcam_device *cam, u32 offset,
		 u32 val, u32 mask)
{
	u32 addr = cam->cam_mmio_base + CAMMMU_REG_OFFSET + offset;
	u32 new_val = (readl(addr) & ~mask) | (val & mask);

	writel(new_val, addr);
	return new_val;
}

static void
cammmu_init(const struct omap24xxcam_device *cam)
{
	/* set the camera MMU autoidle bit */
	cammmu_reg_out(cam, CAMMMU_SYSCONFIG, CAMMMU_SYSCONFIG_AUTOIDLE);
}

/*
 * camera subsystem register I/O routines
 */

static __inline__ u32
cam_reg_in(const struct omap24xxcam_device *cam, u32 offset)
{
	return readl(cam->cam_mmio_base + offset);
}

static __inline__ u32
cam_reg_out(const struct omap24xxcam_device *cam, u32 offset, u32 val)
{
	writel(val, cam->cam_mmio_base + offset);
	return val;
}

static __inline__ u32
cam_reg_merge(const struct omap24xxcam_device *cam, u32 offset,
	      u32 val, u32 mask)
{
	u32 addr = cam->cam_mmio_base + offset;
	u32 new_val = (readl(addr) & ~mask) | (val & mask);

	writel(new_val, addr);
	return new_val;
}

/* Reset the camera subsystem (camera core, camera DMA, and camera MMU) */
static void
cam_reset(const struct omap24xxcam_device *cam, unsigned long timeout_ticks)
{
	unsigned long timeout;

	cam_reg_out(cam, CAM_SYSCONFIG, CAM_SYSCONFIG_SOFTRESET);
	/* wait for reset to complete */
	timeout = jiffies + timeout_ticks;
	while (!(cam_reg_in(cam, CAM_SYSSTATUS) & CAM_SYSSTATUS_RESETDONE)
	       && time_before(jiffies, timeout)) {
		if (!in_atomic()) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		} else
			udelay(10);
	}
	if (unlikely
	    (!(cam_reg_in(cam, CAM_SYSSTATUS) & CAM_SYSSTATUS_RESETDONE))) {
		printk(KERN_WARNING CAM_NAME
		       ": timeout waiting for camera subsystem reset\n");
	}

	return;
}

/* Initialize the camera subsystem (camera core, camera DMA, and camera MMU) */
static void
cam_init(const struct omap24xxcam_device *cam)
{
	/* reset the camera subsystem with a timeout of 200ms */
	cam_reset(cam, msecs_to_jiffies(200));

	/* set the camera subsystem autoidle bit */
	cam_reg_out(cam, CAM_SYSCONFIG, CAM_SYSCONFIG_AUTOIDLE);

	/* initialize the camera MMU */
	cammmu_init(cam);

	/* initialize the camera DMA controller */
	camdma_init(cam);

	/* initialize the camera core module */
	cc_init(cam, 0);
}

/* Program the camera interface xclk for the frequency cam->xclk based on
 * the functional clock frequency cam->mclk.  If the specifed cam->xclk
 * frequency is not possible based on the value of cam->mclk, then the
 * closest xclk frequency lower than the specified xclk will be selected.
 * The actual xclk frequency is returned in cam->xclk.  If cam->xclk is zero,
 * then xclk is turned off (stable low value).
 */
static void
omap24xxcam_set_xclk(struct omap24xxcam_device *cam)
{
	unsigned long divisor;

	if (cam->mclk == 0)
		cam->mclk = DEFAULT_CAM_FUNC_CLK;	/* supply a default mclk */

	if (cam->xclk == 0) {
		cc_reg_out(cam, CC_CTRL_XCLK, CC_CTRL_XCLK_DIV_STABLE_LOW);
		return;
	}

	if (cam->xclk > cam->mclk)
		cam->xclk = cam->mclk;

	divisor = cam->mclk / cam->xclk;
	if (cam->xclk * divisor < cam->mclk)
		divisor += 1;
	if (divisor > (CC_CTRL_XCLK_DIV >> CC_CTRL_XCLK_DIV_SHIFT))
		divisor = CC_CTRL_XCLK_DIV >> CC_CTRL_XCLK_DIV_SHIFT;
	cam->xclk = cam->mclk / divisor;
	if (divisor == 1)
		cc_reg_out(cam, CC_CTRL_XCLK, CC_CTRL_XCLK_DIV_BYPASS);
	else
		cc_reg_out(cam, CC_CTRL_XCLK, divisor);
}

/* -------------------------------------------------------------------------- */

/* Start a DMA transfer from the camera to memory.
 * Returns zero if the transfer was successfully started, or non-zero if all
 * DMA channels are already in use or starting is currently inhibited.
 */
static void
omap24xxcam_overlay_callback(struct omap24xxcam_device *cam, unsigned long csr,
			     void *arg);
static int
omap24xxcam_dma_start(struct omap24xxcam_device *cam, dma_addr_t start,
		      unsigned long len, dma_callback_t callback, void *arg)
{
	unsigned long irqflags;
	int dmach, vrfb_pixelsize = 2;
	void (*dma_notify) (struct omap24xxcam_device * cam);

	spin_lock_irqsave(&cam->dma_lock, irqflags);

	if (!cam->free_dmach || cam->dma_stop) {
		spin_unlock_irqrestore(&cam->dma_lock, irqflags);
		return -EBUSY;
	}

	dmach = cam->next_dmach;

	cam->camdma[dmach].callback = callback;
	cam->camdma[dmach].arg = arg;

	if ((cam->sgdma[(int)arg].callback == omap24xxcam_overlay_callback)
		&& cam->overlay_rotation) {
		if (V4L2_PIX_FMT_YUYV == cam->pix.pixelformat ||
		    V4L2_PIX_FMT_UYVY == cam->pix.pixelformat)
				vrfb_pixelsize <<= 1;
		camdma_reg_out(cam, CAMDMA_CCR(dmach),
		       CAMDMA_CCR_SEL_SRC_DST_SYNC
		       | CAMDMA_CCR_BS
		       | CAMDMA_CCR_DST_AMODE_DBL_IDX
		       | CAMDMA_CCR_SRC_AMODE_POST_INC
		       | CAMDMA_CCR_FS | CAMDMA_CCR_SYNCHRO_CAMERA);
		camdma_reg_out(cam, CAMDMA_CEN(dmach), cam->pix.bytesperline);
		camdma_reg_out(cam, CAMDMA_CFN(dmach), cam->pix.height);
		camdma_reg_out(cam, CAMDMA_CDEI(dmach), 1);
		camdma_reg_out(cam, CAMDMA_CDFI(dmach), 
			(2048*vrfb_pixelsize-cam->pix.bytesperline)+1);
	}
	else {
		camdma_reg_out(cam, CAMDMA_CCR(dmach),
		       CAMDMA_CCR_SEL_SRC_DST_SYNC
		       | CAMDMA_CCR_BS
		       | CAMDMA_CCR_DST_AMODE_POST_INC
		       | CAMDMA_CCR_SRC_AMODE_POST_INC
		       | CAMDMA_CCR_FS | CAMDMA_CCR_SYNCHRO_CAMERA);
		camdma_reg_out(cam, CAMDMA_CEN(dmach), len);
		camdma_reg_out(cam, CAMDMA_CFN(dmach), 1);
		camdma_reg_out(cam, CAMDMA_CDEI(dmach), 0);
		camdma_reg_out(cam, CAMDMA_CDFI(dmach), 0);
	}
	camdma_reg_out(cam, CAMDMA_CLNK_CTRL(dmach), 0);
	camdma_reg_out(cam, CAMDMA_CSDP(dmach),
		       CAMDMA_CSDP_WRITE_MODE_POSTED
		       | CAMDMA_CSDP_DST_BURST_EN_16
		       | CAMDMA_CSDP_DST_PACKED
		       | CAMDMA_CSDP_SRC_BURST_EN_16
		       | CAMDMA_CSDP_SRC_PACKED
		       | CAMDMA_CSDP_DATA_TYPE_8BITS);
	camdma_reg_out(cam, CAMDMA_CSSA(dmach), 0);
	camdma_reg_out(cam, CAMDMA_CDSA(dmach), start);
	camdma_reg_out(cam, CAMDMA_CSEI(dmach), 0);
	camdma_reg_out(cam, CAMDMA_CSFI(dmach), DMA_THRESHOLD);
	camdma_reg_out(cam, CAMDMA_CSR(dmach),
		       CAMDMA_CSR_MISALIGNED_ERR
		       | CAMDMA_CSR_SUPERVISOR_ERR
		       | CAMDMA_CSR_SECURE_ERR
		       | CAMDMA_CSR_TRANS_ERR
		       | CAMDMA_CSR_BLOCK | CAMDMA_CSR_DROP);
	camdma_reg_out(cam, CAMDMA_CICR(dmach),
		       CAMDMA_CICR_MISALIGNED_ERR_IE
		       | CAMDMA_CICR_SUPERVISOR_ERR_IE
		       | CAMDMA_CICR_SECURE_ERR_IE
		       | CAMDMA_CICR_TRANS_ERR_IE
		       | CAMDMA_CICR_BLOCK_IE | CAMDMA_CICR_DROP_IE);

	/* We're ready to start the DMA transfer. */
	if (cam->free_dmach < NUM_CAMDMA_CHANNELS) {
		/* A transfer is already in progress, so try to chain to it. */
		int prev_dmach, ch;

		if (dmach == 0)
			prev_dmach = NUM_CAMDMA_CHANNELS - 1;
		else
			prev_dmach = dmach - 1;
		camdma_reg_out(cam, CAMDMA_CLNK_CTRL(prev_dmach),
			       CAMDMA_CLNK_CTRL_ENABLE_LNK | dmach);
		/* Did we chain the DMA transfer before the previous one
		 * finished? Scan the in-use channel list. ch is the first one.
		 */
		ch = (dmach + cam->free_dmach) % NUM_CAMDMA_CHANNELS;
		/* dmach is guranteed to start as long as we find one channel
		   is still enabled */
		while (!(camdma_reg_in(cam, CAMDMA_CCR(ch))
			 & CAMDMA_CCR_ENABLE)) {
			if (ch == dmach) {
				/* The previous transfers have ended and this one
				 * hasn't started, so we must not have chained
				 * to the previous one in time.  We'll have to
				 * start it now.
				 */
				if ((cam->sgdma[(int)arg].callback == omap24xxcam_overlay_callback)
					&& cam->overlay_rotation)
				camdma_reg_out(cam, CAMDMA_CCR(dmach),
		       			CAMDMA_CCR_SEL_SRC_DST_SYNC
		       			| CAMDMA_CCR_BS
		       			| CAMDMA_CCR_DST_AMODE_DBL_IDX
		       			| CAMDMA_CCR_SRC_AMODE_POST_INC
		       			| CAMDMA_CCR_ENABLE
		       			| CAMDMA_CCR_FS | CAMDMA_CCR_SYNCHRO_CAMERA);
 				else
				camdma_reg_out(cam, CAMDMA_CCR(dmach),
					       CAMDMA_CCR_SEL_SRC_DST_SYNC
					       | CAMDMA_CCR_BS
					       | CAMDMA_CCR_DST_AMODE_POST_INC
					       | CAMDMA_CCR_SRC_AMODE_POST_INC
					       | CAMDMA_CCR_ENABLE
					       | CAMDMA_CCR_FS
					       | CAMDMA_CCR_SYNCHRO_CAMERA);
				break;
			} else
				ch = (ch + 1) % NUM_CAMDMA_CHANNELS;
		}
	} else {
		/* No transfer is in progress, so we'll just start this one
		 * now.
		 */
		if ((cam->sgdma[(int)arg].callback == omap24xxcam_overlay_callback)
			&& cam->overlay_rotation)
		camdma_reg_out(cam, CAMDMA_CCR(dmach),
		       CAMDMA_CCR_SEL_SRC_DST_SYNC
		       | CAMDMA_CCR_BS
		       | CAMDMA_CCR_DST_AMODE_DBL_IDX
		       | CAMDMA_CCR_SRC_AMODE_POST_INC
		       | CAMDMA_CCR_ENABLE
		       | CAMDMA_CCR_FS | CAMDMA_CCR_SYNCHRO_CAMERA);
 		else
		camdma_reg_out(cam, CAMDMA_CCR(dmach),
			       CAMDMA_CCR_SEL_SRC_DST_SYNC
			       | CAMDMA_CCR_BS
			       | CAMDMA_CCR_DST_AMODE_POST_INC
			       | CAMDMA_CCR_SRC_AMODE_POST_INC
			       | CAMDMA_CCR_ENABLE
			       | CAMDMA_CCR_FS | CAMDMA_CCR_SYNCHRO_CAMERA);
	}

	cam->next_dmach = (cam->next_dmach + 1) % NUM_CAMDMA_CHANNELS;
	cam->free_dmach--;

	dma_notify = cam->dma_notify;
	cam->dma_notify = NULL;

	spin_unlock_irqrestore(&cam->dma_lock, irqflags);

	if (dma_notify)
		(*dma_notify) (cam);

	return 0;
}

/* DMA completion routine for the scatter-gather DMA fragments. */
static void
omap24xxcam_sg_dma_callback(struct omap24xxcam_device *cam, unsigned long csr,
			    void *arg)
{
	int sgslot = (int) arg;
	struct sgdma_state *sgdma;
	const unsigned long csr_error = CAMDMA_CSR_MISALIGNED_ERR
	    | CAMDMA_CSR_SUPERVISOR_ERR | CAMDMA_CSR_SECURE_ERR
	    | CAMDMA_CSR_TRANS_ERR | CAMDMA_CSR_DROP;

	spin_lock(&cam->sg_lock);

	sgdma = cam->sgdma + sgslot;
	if (!sgdma->queued_sglist) {
		spin_unlock(&cam->sg_lock);
		printk(KERN_DEBUG CAM_NAME
		       ": sgdma completed when none queued!\n");
		return;
	}

	sgdma->csr |= csr;
	if (!--sgdma->queued_sglist) {
		/* Queue for this sglist is empty, so check to see if we're
		 * done.
		 */
		if ((sgdma->next_sglist == sgdma->sglen)
		    || (sgdma->csr & csr_error)) {
			dma_callback_t callback = sgdma->callback;
			void *arg = sgdma->arg;
			unsigned long sg_csr = sgdma->csr;
			/* All done with this sglist */
			cam->free_sgdma++;
			if (callback) {
				spin_unlock(&cam->sg_lock);
				(*callback) (cam, sg_csr, arg);
				return;
			}
		}
	}

	spin_unlock(&cam->sg_lock);
}

/* Process the scatter-gather DMA queue by starting queued transfers. */
static void
omap24xxcam_sg_dma_process(struct omap24xxcam_device *cam)
{
	unsigned long irqflags;
	int queued_sgdma, sgslot;
	struct sgdma_state *sgdma;
	const unsigned long csr_error = CAMDMA_CSR_MISALIGNED_ERR
	    | CAMDMA_CSR_SUPERVISOR_ERR | CAMDMA_CSR_SECURE_ERR
	    | CAMDMA_CSR_TRANS_ERR | CAMDMA_CSR_DROP;

	spin_lock_irqsave(&cam->sg_lock, irqflags);

	queued_sgdma = NUM_SG_DMA - cam->free_sgdma;
	sgslot = (cam->next_sgdma + cam->free_sgdma) % NUM_SG_DMA;
	while (queued_sgdma > 0) {
		sgdma = cam->sgdma + sgslot;
		while ((sgdma->next_sglist < sgdma->sglen) &&
		       !(sgdma->csr & csr_error)) {
			const struct scatterlist *sglist;

			sglist = sgdma->sglist + sgdma->next_sglist;
			/* try to start the next DMA transfer */
			if (omap24xxcam_dma_start(cam, sg_dma_address(sglist),
						  sg_dma_len(sglist),
						  omap24xxcam_sg_dma_callback,
						  (void *) sgslot)) {
				/* DMA start failed */
				spin_unlock_irqrestore(&cam->sg_lock, irqflags);
				return;
			} else {
				/* DMA start was successful */
				sgdma->next_sglist++;
				sgdma->queued_sglist++;
			}
		}
		queued_sgdma--;
		sgslot = (sgslot + 1) % NUM_SG_DMA;
	}

	spin_unlock_irqrestore(&cam->sg_lock, irqflags);
}

/* Queue a scatter-gather DMA transfer from the camera to memory.
 * Returns zero if the transfer was successfully queued, or
 * non-zero if all of the scatter-gather slots are already in use.
 */
static int
omap24xxcam_sg_dma_queue(struct omap24xxcam_device *cam,
			 const struct scatterlist *sglist, int sglen,
			 dma_callback_t callback, void *arg)
{
	unsigned long irqflags;
	struct sgdma_state *sgdma;

	if ((sglen < 0) || ((sglen > 0) & !sglist))
		return -EINVAL;

	spin_lock_irqsave(&cam->sg_lock, irqflags);

	if (!cam->free_sgdma) {
		spin_unlock_irqrestore(&cam->sg_lock, irqflags);
		return -EBUSY;
	}

	sgdma = cam->sgdma + cam->next_sgdma;

	sgdma->sglist = sglist;
	sgdma->sglen = sglen;
	sgdma->next_sglist = 0;
	sgdma->queued_sglist = 0;
	sgdma->csr = 0;
	sgdma->callback = callback;
	sgdma->arg = arg;

	cam->next_sgdma = (cam->next_sgdma + 1) % NUM_SG_DMA;
	cam->free_sgdma--;

	spin_unlock_irqrestore(&cam->sg_lock, irqflags);

	omap24xxcam_sg_dma_process(cam);

	return 0;
}

/* Abort all chained DMA transfers.  After all transfers have been aborted and
 * the DMA controller is idle, the completion routines for any aborted transfers
 * will be called in sequence.  The DMA controller may not be idle after this
 * routine completes, because the completion routines might start new transfers.
 */
static void
omap24xxcam_dma_abort(struct omap24xxcam_device *cam, unsigned long csr)
{
	unsigned long irqflags;
	int dmach, i, free_dmach;
	dma_callback_t callback;
	void *arg;

	spin_lock_irqsave(&cam->dma_lock, irqflags);

	/* stop any DMA transfers in progress */
	dmach = (cam->next_dmach + cam->free_dmach) % NUM_CAMDMA_CHANNELS;
	for (i = 0; i < NUM_CAMDMA_CHANNELS; i++) {
		/* mask all interrupts from this channel */
		camdma_reg_out(cam, CAMDMA_CICR(dmach), 0);
		/* unlink this channel */
		camdma_reg_merge(cam, CAMDMA_CLNK_CTRL(dmach), 0,
				 CAMDMA_CLNK_CTRL_ENABLE_LNK);
		/* disable this channel */
		camdma_reg_merge(cam, CAMDMA_CCR(dmach), 0, CAMDMA_CCR_ENABLE);
		dmach = (dmach + 1) % NUM_CAMDMA_CHANNELS;
	}

	/* We have to be careful here because the callback routine might start
	 * a new DMA transfer, and we only want to abort transfers that were
	 * started before this routine was called.
	 */
	free_dmach = cam->free_dmach;
	while ((cam->free_dmach < NUM_CAMDMA_CHANNELS) &&
	       (free_dmach < NUM_CAMDMA_CHANNELS)) {
		dmach = (cam->next_dmach + cam->free_dmach)
		    % NUM_CAMDMA_CHANNELS;
		callback = cam->camdma[dmach].callback;
		arg = cam->camdma[dmach].arg;
		cam->free_dmach++;
		free_dmach++;
		if (callback) {
			/* leave interrupts disabled during callback */
			spin_unlock(&cam->dma_lock);
			(*callback) (cam, csr, arg);
			spin_lock(&cam->dma_lock);
		}
	}

	spin_unlock_irqrestore(&cam->dma_lock, irqflags);
}

/* Abort all chained DMA transfers.  After all transfers have been aborted and
 * the DMA controller is idle, the completion routines for any aborted transfers
 * will be called in sequence.  If the completion routines attempt to start a
 * new DMA transfer it will fail, so the DMA controller will be idle after this
 * routine completes.
 */
static void
omap24xxcam_dma_stop(struct omap24xxcam_device *cam, unsigned long csr)
{
	unsigned long irqflags;

	spin_lock_irqsave(&cam->dma_lock, irqflags);
	cam->dma_stop++;
	spin_unlock_irqrestore(&cam->dma_lock, irqflags);
	omap24xxcam_dma_abort(cam, csr);
	spin_lock_irqsave(&cam->dma_lock, irqflags);
	cam->dma_stop--;
	spin_unlock_irqrestore(&cam->dma_lock, irqflags);
}

/* Sync scatter-gather DMA by aborting any DMA transfers currently in progress.
 * Any queued scatter-gather DMA transactions that have not yet been started
 * will remain queued.  The DMA controller will be idle after this routine
 * completes.  When the scatter-gather queue is restarted, the next
 * scatter-gather DMA transfer will begin at the start of a new transaction.
 */
static void
omap24xxcam_sg_dma_sync(struct omap24xxcam_device *cam, unsigned long csr)
{
	unsigned long irqflags;
	int sgslot;
	struct sgdma_state *sgdma;

	/* stop any DMA transfers in progress */
	omap24xxcam_dma_stop(cam, csr);

	spin_lock_irqsave(&cam->sg_lock, irqflags);

	if (cam->free_sgdma < NUM_SG_DMA) {
		sgslot = (cam->next_sgdma + cam->free_sgdma) % NUM_SG_DMA;
		sgdma = cam->sgdma + sgslot;
		if (sgdma->next_sglist != 0) {
			/* This DMA transfer was in progress, so abort it. */
			dma_callback_t callback = sgdma->callback;
			void *arg = sgdma->arg;
			cam->free_sgdma++;
			if (callback) {
				/* leave interrupts masked */
				spin_unlock(&cam->sg_lock);
				(*callback) (cam, csr, arg);
				spin_lock(&cam->sg_lock);
			}
		}
	}

	spin_unlock_irqrestore(&cam->sg_lock, irqflags);
}

/* Register a routine to be called once immediately after a DMA transfer is
 * started.  The purpose of this is to allow the camera interface to be
 * started only after a DMA transaction has been queued in order to avoid
 * DMA overruns.  The registered callback routine will only be called one
 * time and then discarded.  Only one callback routine may be registered at a
 * time.
 * Returns zero if successful, or a non-zero error code if a different callback
 * routine has already been registered.
 */
static int
omap24xxcam_dma_notify(struct omap24xxcam_device *cam,
		       void (*callback) (struct omap24xxcam_device * cam))
{
	unsigned long irqflags;

	spin_lock_irqsave(&cam->dma_lock, irqflags);

	if (cam->dma_notify && (cam->dma_notify != callback)) {
		spin_unlock_irqrestore(&cam->dma_lock, irqflags);
		return -EBUSY;
	}

	cam->dma_notify = callback;

	spin_unlock_irqrestore(&cam->dma_lock, irqflags);

	return 0;
}

/* Camera DMA interrupt service routine. */
static void
omap24xxcam_dma_isr(struct omap24xxcam_device *cam)
{
	int dmach, i;
	dma_callback_t callback;
	void *arg;
	unsigned long csr;
	const unsigned long csr_error = CAMDMA_CSR_MISALIGNED_ERR
	    | CAMDMA_CSR_SUPERVISOR_ERR | CAMDMA_CSR_SECURE_ERR
	    | CAMDMA_CSR_TRANS_ERR | CAMDMA_CSR_DROP;

	spin_lock(&cam->dma_lock);
	if (cam->free_dmach == NUM_CAMDMA_CHANNELS) {
		/* A camera DMA interrupt occurred while all channels are idle,
		 * so we'll acknowledge the interrupt in the IRQSTATUS register
		 * and exit.
		 */
		for (i = 0; i < NUM_CAMDMA_CHANNELS; ++i) {
			csr = camdma_reg_in(cam, CAMDMA_CSR(i));
			/* ack interrupt in CSR */
			camdma_reg_out(cam, CAMDMA_CSR(i), csr);
		}
		camdma_reg_out(cam, CAMDMA_IRQSTATUS_L0, 0xffffffff);
		spin_unlock(&cam->dma_lock);
		return;
	}

	while (cam->free_dmach < NUM_CAMDMA_CHANNELS) {
		dmach = (cam->next_dmach + cam->free_dmach)
		    % NUM_CAMDMA_CHANNELS;
		if (camdma_reg_in(cam, CAMDMA_CCR(dmach)) & CAMDMA_CCR_ENABLE) {
			/* This buffer hasn't finished yet, so we're done. */
			break;
		}
		csr = camdma_reg_in(cam, CAMDMA_CSR(dmach));
		/* ack interrupt in CSR */
		camdma_reg_out(cam, CAMDMA_CSR(dmach), csr);
		/* ack interrupt in IRQSTATUS */
		camdma_reg_out(cam, CAMDMA_IRQSTATUS_L0, (1 << dmach));
		if (csr & csr_error) {
			/* A DMA error occurred, so stop all DMA transfers in
			 * progress.
			 */
			spin_unlock(&cam->dma_lock);
			omap24xxcam_dma_stop(cam, csr);
			return;
		} else {
			callback = cam->camdma[dmach].callback;
			arg = cam->camdma[dmach].arg;
			cam->free_dmach++;
			if (callback) {
				spin_unlock(&cam->dma_lock);
				(*callback) (cam, csr, arg);
				spin_lock(&cam->dma_lock);
			}
		}
	}

	spin_unlock(&cam->dma_lock);

	omap24xxcam_sg_dma_process(cam);
}

/* Shutdown the camera DMA driver and controller. */
static void
omap24xxcam_dma_exit(struct omap24xxcam_device *cam)
{
	int ch;

	/* Mask all DMA interrupts */
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L0, 0);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L1, 0);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L2, 0);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L3, 0);

	/* disable all DMA channels */
	for (ch = 0; ch < NUM_CAMDMA_CHANNELS; ch++)
		camdma_reg_out(cam, CAMDMA_CCR(ch), 0);
}

/* Initialize the camera DMA driver. */
static void
omap24xxcam_dma_init(struct omap24xxcam_device *cam)
{
	int ch, sg;

	/* group all channels on DMA IRQ0 and unmask irq */
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L0, 0xffffffff);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L1, 0);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L2, 0);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L3, 0);

	spin_lock_init(&cam->dma_lock);
	cam->free_dmach = NUM_CAMDMA_CHANNELS;
	cam->next_dmach = 0;
	for (ch = 0; ch < NUM_CAMDMA_CHANNELS; ch++) {
		cam->camdma[ch].callback = NULL;
		cam->camdma[ch].arg = NULL;
	}

	spin_lock_init(&cam->sg_lock);
	cam->free_sgdma = NUM_SG_DMA;
	cam->next_sgdma = 0;
	for (sg = 0; sg < NUM_SG_DMA; sg++) {
		cam->sgdma[sg].sglen = 0;
		cam->sgdma[sg].next_sglist = 0;
		cam->sgdma[sg].queued_sglist = 0;
		cam->sgdma[sg].csr = 0;
		cam->sgdma[sg].callback = NULL;
		cam->sgdma[sg].arg = NULL;
	}
}

/* -------------------------------------------------------------------------- */

/* Callback routine for overlay DMA completion.  We just start another DMA
 * transfer unless overlay has been turned off.
 */
static void
omap24xxcam_overlay_callback(struct omap24xxcam_device *cam, unsigned long csr,
			     void *arg)
{
	int err;
	unsigned long irqflags;
	const unsigned long csr_error = CAMDMA_CSR_MISALIGNED_ERR
	    | CAMDMA_CSR_SUPERVISOR_ERR | CAMDMA_CSR_SECURE_ERR
	    | CAMDMA_CSR_TRANS_ERR | CAMDMA_CSR_DROP;

	spin_lock_irqsave(&cam->overlay_lock, irqflags);

	if (cam->overlay_cnt > 0)
		--cam->overlay_cnt;

	if (!cam->previewing || (csr & csr_error)) {
		spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
		return;
	}

	sg_dma_address(&cam->overlay_sglist) = cam->overlay_base_dma;
	sg_dma_len(&cam->overlay_sglist) = cam->pix.sizeimage;

	while (cam->overlay_cnt < 2) {
		err = omap24xxcam_sg_dma_queue(cam, &cam->overlay_sglist, 1,
					       omap24xxcam_overlay_callback,
					       NULL);
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
omap24xxcam_start_overlay_dma(struct omap24xxcam_device *cam)
{
	int err;
	unsigned long irqflags;

	if (!cam->previewing)
		return;

	if (cam->pix.sizeimage > cam->overlay_size)
		return;

	spin_lock_irqsave(&cam->overlay_lock, irqflags);

	sg_dma_address(&cam->overlay_sglist) = cam->overlay_base_dma;
	sg_dma_len(&cam->overlay_sglist) = cam->pix.sizeimage;

	while (cam->overlay_cnt < 2) {
		err = omap24xxcam_sg_dma_queue(cam, &cam->overlay_sglist, 1,
					       omap24xxcam_overlay_callback,
					       NULL);
		if (err)
			break;
		++cam->overlay_cnt;
	}

	spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
}

/* Enable the camera core interface. */
static void
omap24xxcam_cc_enable(struct omap24xxcam_device *cam)
{
	/* Get the CC_CTRL register value for the current capture format. */
	omap24xxcam_sensor_cc_ctrl(cam);

	/* program the camera interface DMA packet size */
	cc_reg_out(cam, CC_CTRL_DMA, CC_CTRL_DMA_EN | (DMA_THRESHOLD / 4 - 1));

	/* enable camera core error interrupts */
	cc_reg_out(cam, CC_IRQENABLE, CC_IRQENABLE_FW_ERR_IRQ
		   | CC_IRQENABLE_FSC_ERR_IRQ | CC_IRQENABLE_SSC_ERR_IRQ
		   | CC_IRQENABLE_FIFO_OF_IRQ);

	/* enable the camera interface */
	cc_reg_out(cam, CC_CTRL, cam->cc_ctrl | CC_CTRL_CC_EN);
}

/* Error recovery for DMA errors and camera interface errors. */
static void
omap24xxcam_error_handler(struct omap24xxcam_device *cam)
{
	/* Get the CC_CTRL register value for the current capture format. */
	omap24xxcam_sensor_cc_ctrl(cam);

	/* Disable and reset the camera interface. */
	cc_reg_out(cam, CC_CTRL,
		   cam->cc_ctrl & ~(CC_CTRL_CC_EN | CC_CTRL_CC_FRAME_TRIG));
	cc_reg_out(cam, CC_CTRL, (cam->cc_ctrl | CC_CTRL_CC_RST)
		   & ~(CC_CTRL_CC_EN | CC_CTRL_CC_FRAME_TRIG));

	/* Stop the DMA controller and frame sync scatter-gather DMA. */
	omap24xxcam_sg_dma_sync(cam, CAMDMA_CSR_TRANS_ERR);

	/* Reset and re-initialize the entire camera subsystem.
	 * Resetting the camera FIFO via the CC_RST bit in the CC_CTRL
	 * register is supposed to be sufficient to recover from a
	 * camera interface error, but it doesn't seem to be enough.  If
	 * we only do that then subsequent image captures are out of sync
	 * by either one or two times DMA_THRESHOLD bytes.  Resetting and
	 * re-initializing the entire camera subsystem prevents the problem
	 * with frame synchronization.  Calling cam_init() from an ISR is
	 * undesirable since it waits an indeterminate amount of time for the
	 * camera subsystem reset to complete.  If we ever figure out exactly
	 * what needs to be reset to recover from a camera interface error,
	 * maybe we can replace this global reset with something less drastic.
	 */
	 /* This is a 2420 silicon bug. It is fixed in all 2430 revs */
	if (cpu_is_omap2420())
		cam_init(cam);
	else
		cc_init(cam, 1);

	omap24xxcam_set_xclk(cam);
	/* group all channels on DMA IRQ0 and unmask irq */
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L0, 0xffffffff);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L1, 0);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L2, 0);
	camdma_reg_out(cam, CAMDMA_IRQENABLE_L3, 0);

	if (cam->previewing || cam->streaming) {
		/* Preview or capture is in progress, so we need to register
		 * our routine to restart the camera interface the next time a
		 * DMA transfer is queued.
		 */
		omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
	}

	/* Restart overlay DMA if preview is enabled. */
	if (cam->previewing)
		omap24xxcam_start_overlay_dma(cam);

	/* Restart the scatter-gather DMA queue. */
	omap24xxcam_sg_dma_process(cam);
}

/* Interrupt service routine for camera core interrupts. */
static void
omap24xxcam_cc_isr(struct omap24xxcam_device *cam)
{
	unsigned long cc_irqstatus;
	const unsigned long cc_irqstatus_err = CC_IRQSTATUS_FW_ERR_IRQ
	    | CC_IRQSTATUS_FSC_ERR_IRQ | CC_IRQSTATUS_SSC_ERR_IRQ
	    | CC_IRQSTATUS_FIFO_OF_IRQ;

	cc_irqstatus = cc_reg_in(cam, CC_IRQSTATUS);
	cc_reg_out(cam, CC_IRQSTATUS, cc_irqstatus);

	if (cc_irqstatus & cc_irqstatus_err)
		omap24xxcam_error_handler(cam);
}

static irqreturn_t
omap24xxcam_isr(int irq, void *arg)
{
	struct omap24xxcam_device *cam = (struct omap24xxcam_device *) arg;
	unsigned long irqstatus;
	unsigned int irqhandled = 0;

	irqstatus = cam_reg_in(cam, CAM_IRQSTATUS);
	if (irqstatus &
	    (CAM_IRQSTATUS_DMA_IRQ2 | CAM_IRQSTATUS_DMA_IRQ1
	     | CAM_IRQSTATUS_DMA_IRQ0)) {
		omap24xxcam_dma_isr(cam);
		irqhandled = 1;
	}
	if (irqstatus & CAM_IRQSTATUS_CC_IRQ) {
		omap24xxcam_cc_isr(cam);
		irqhandled = 1;
	}
	if (irqstatus & CAM_IRQSTATUS_MMU_IRQ)
		printk(KERN_ERR CAM_NAME ": Unhandled camera MMU interrupt!\n");

	return IRQ_RETVAL(irqhandled);
}

static void
omap24xxcam_adjust_xclk(struct omap24xxcam_device *cam)
{
	unsigned long divisor;

	if (cam->xclk > cam->mclk)
		cam->xclk = cam->mclk;
	divisor = cam->mclk / cam->xclk;
	if (cam->xclk * divisor < cam->mclk)
		divisor += 1;
	if (divisor > 30)
		divisor = 30;

	cam->xclk = cam->mclk / divisor;
}

/* -------------------------------------------------------------------------- */

/* This routine is called from interrupt context when a scatter-gather DMA
 * transfer of a videobuf_buffer completes.
 */
static void
omap24xxcam_vbq_complete(struct omap24xxcam_device *cam, unsigned long csr,
			 void *arg)
{
	struct videobuf_buffer *vb = (struct videobuf_buffer *) arg;
	const unsigned long csr_error = CAMDMA_CSR_MISALIGNED_ERR
	    | CAMDMA_CSR_SUPERVISOR_ERR | CAMDMA_CSR_SECURE_ERR
	    | CAMDMA_CSR_TRANS_ERR | CAMDMA_CSR_DROP;

	spin_lock(&cam->vbq_lock);

	do_gettimeofday(&vb->ts);
	vb->field_count = cam->field_count;
	cam->field_count += 2;
	if (csr & csr_error) {
		vb->state = STATE_ERROR;
		omap24xxcam_error_handler(cam);
	} else
		vb->state = STATE_DONE;
	wake_up(&vb->done);
	spin_unlock(&cam->vbq_lock);
}

static void
omap24xxcam_vbq_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	videobuf_waiton(vb, 0, 0);
	videobuf_dma_unmap(q, videobuf_to_dma(vb));
	videobuf_dma_free(videobuf_to_dma(vb));

	vb->state = STATE_NEEDS_INIT;
}

/* Limit the number of available kernel image capture buffers based on the
 * number requested, the currently selected image size, and the maximum
 * amount of memory permitted for kernel capture buffers.
 */
static int
omap24xxcam_vbq_setup(struct videobuf_queue *q, unsigned int *cnt, unsigned int *size)
{
	struct omap24xxcam_fh *fh = q->priv_data;
	struct omap24xxcam_device *cam = fh->cam;

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

	while (*size * *cnt > capture_mem)
		(*cnt)--;

	return 0;
}

static int
omap24xxcam_vbq_prepare(struct videobuf_queue *q, struct videobuf_buffer *vb,
	enum v4l2_field field)
{
	struct omap24xxcam_fh *fh = q->priv_data;
	struct omap24xxcam_device *cam = fh->cam;
	unsigned int size;
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
				omap24xxcam_vbq_release(q, vb);
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

	if (vb->state == STATE_NEEDS_INIT)
		err = videobuf_iolock(q, vb, NULL);

	if (!err) {
		vb->state = STATE_PREPARED;
		if (vb->baddr) {
			flush_cache_user_range(NULL, vb->baddr, (vb->baddr + vb->bsize));
		} else {
			/* sync a kernel buffer */
			dmac_flush_range(videobuf_to_dma(vb)->vmalloc,
						(videobuf_to_dma(vb)->vmalloc
							+ (videobuf_to_dma(vb)->nr_pages << PAGE_SHIFT)));
			outer_flush_range(__pa(videobuf_to_dma(vb)->vmalloc),
						__pa(videobuf_to_dma(vb)->vmalloc
							+ (videobuf_to_dma(vb)->nr_pages << PAGE_SHIFT)));
		}
	} else
		omap24xxcam_vbq_release(q, vb);

	return err;
}

static void
omap24xxcam_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct omap24xxcam_fh *fh = q->priv_data;
	struct omap24xxcam_device *cam = fh->cam;
	enum videobuf_state state = vb->state;
	int err;

	/* Videobuf buffers are aligned to pagesize.
 	 * So, we need to correct the last buffer size to avoid
 	 * unnecessary reading
	 */
	if ((vb->size + videobuf_to_dma(vb)->sglist[0].offset) % PAGE_SIZE) {
		videobuf_to_dma(vb)->sglist[videobuf_to_dma(vb)->sglen-1].length = 
			(vb->size + videobuf_to_dma(vb)->sglist[0].offset) % PAGE_SIZE;
	}   

	vb->state = STATE_QUEUED;
	err = omap24xxcam_sg_dma_queue(cam, videobuf_to_dma(vb)->sglist,
					videobuf_to_dma(vb)->sglen,
					omap24xxcam_vbq_complete, vb);
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
omap24xxcam_preview(struct omap24xxcam_device *cam)
{
	/* Get the panel parameters.
	 * They can change from LCD to TV
	 * or TV to LCD
	 */
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
}

static int
omap24xxcam_do_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		     void *arg)
{
	struct omap24xxcam_fh *fh = file->private_data;
	struct omap24xxcam_device *cam = fh->cam;
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

			switch (f->type) {
			case V4L2_BUF_TYPE_VIDEO_OVERLAY:
				{
					struct v4l2_window *win = &f->fmt.win;

					spin_lock(&cam->img_lock);
					if (cam->previewing || cam->streaming) {
						spin_unlock(&cam->img_lock);
						return -EBUSY;
					}
					/* Get the panel parameters.
					 * They can change from LCD to TV
					 * or TV to LCD
					 */
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
					spin_unlock(&cam->img_lock);
					return err;
				}

			case V4L2_BUF_TYPE_STILL_CAPTURE:
				{
					if (cam->cam_sensor->try_format_still_capture &&
					    cam->cam_sensor->configure_still_capture) {
					spin_lock(&cam->img_lock);
					cam->cam_sensor->try_format_still_capture(&f->fmt.pix,
								    cam->
								    sensor);
					/* set the new capture format */
					cam->pix2 = f->fmt.pix;
					spin_unlock(&cam->img_lock);
					err =
					    cam->cam_sensor->configure_still_capture
					    			      (&cam->
								       pix2,
								       cam->
								       xclk,
								       &cam->
								       cparm.
								       timeperframe,
								       cam->
								       sensor);
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
					cam->cam_sensor->try_format(&f->fmt.pix,
								    cam->
								    sensor);
					/* set the new capture format */
					cam->pix = f->fmt.pix;

					/* adjust the capture frame rate */
					cam->xclk =
					    cam->cam_sensor->calc_xclk(&cam->
								       pix,
								       &cam->
								       nominal_timeperframe,
								       cam->
								       sensor);
					omap24xxcam_adjust_xclk(cam);
					cam->cparm.timeperframe =
					    cam->nominal_timeperframe;
					
					/* set a default display window and preview crop */
					omap2_disp_get_dss();
					omap24xxcam_preview(cam);
					omap2_disp_put_dss();
					spin_unlock(&cam->img_lock);

					/* program xclk */
					omap24xxcam_set_xclk(cam);
					/* program the sensor */
					err =
					    cam->cam_sensor->configure(&cam->
								       pix,
								       cam->
								       xclk,
								       &cam->
								       cparm.
								       timeperframe,
								       cam->
								       sensor);
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
					if (cam->cam_sensor->cropcap) {
					return cam->cam_sensor->cropcap(cropcap, 
								cam->sensor); 
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
					if (cam->cam_sensor->get_crop) {
					return cam->cam_sensor->
						get_crop(crop, cam->sensor); 
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
					if (cam->cam_sensor->set_crop) {
						err = cam->cam_sensor->
							set_crop(crop, cam->sensor);
						return err; 
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
				    cam->cam_sensor->calc_xclk(&cam->pix,
							       &cam->
							       nominal_timeperframe,
							       cam->sensor);
				omap24xxcam_adjust_xclk(cam);
				cam->cparm.timeperframe =
				    cam->nominal_timeperframe;
				spin_unlock(&cam->img_lock);
				/* program xclk */
				omap24xxcam_set_xclk(cam);
				/* program the sensor */
				err =
				    cam->cam_sensor->configure(&cam->pix,
							       cam->xclk,
							       &cam->cparm.
							       timeperframe,
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
			omap24xxcam_preview(cam);

			spin_unlock(&cam->img_lock);
			return 0;
		}
	case VIDIOC_OVERLAY:
		{
			int *on = arg;
			int vrfb_pixelsize = 2;
			int rotation, dss_dma_start;

			spin_lock(&cam->img_lock);
			if (!cam->previewing && *on) {
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
					/* start the camera interface */
					omap24xxcam_dma_notify(cam,
							       omap24xxcam_cc_enable);
					omap24xxcam_start_overlay_dma(cam);
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
				cam->previewing = NULL;
				spin_unlock(&cam->img_lock);
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
			if (cam->streaming) {
				spin_unlock(&cam->img_lock);
				return -EBUSY;
			} else
				cam->streaming = fh;
			spin_unlock(&cam->img_lock);

			omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
			return videobuf_streamon(&fh->vbq);
		}

	case VIDIOC_STREAMOFF:
		{
			struct videobuf_queue *q = &fh->vbq;
			int i;

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

			err = videobuf_streamoff(q);
			if (err < 0)
				return err;

			for (i = 0; i < VIDEO_MAX_FRAME; i++) {
				if (NULL == q->bufs[i])
					continue;
				if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
					omap24xxcam_vbq_release(q,
								q->bufs[i]);
			}

			spin_lock(&cam->img_lock);
			if (cam->streaming == fh)
				cam->streaming = NULL;
			spin_unlock(&cam->img_lock);
			return 0;
		}

	case VIDIOC_G_CTRL:
		{
			struct v4l2_control *vc = arg;
			return cam->cam_sensor->get_control(vc, cam->sensor);
		}
	case VIDIOC_S_CTRL:
		{
			struct v4l2_control *vc = arg;
			return cam->cam_sensor->set_control(vc, cam->sensor);
		}
	case VIDIOC_QUERYCTRL:
		{
			struct v4l2_queryctrl *qc = arg;
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

	case VIDIOC_ENUM_FMT:
		{
			struct v4l2_fmtdesc *fmt = arg;
			return cam->cam_sensor->enum_pixformat(fmt,
							       cam->sensor);
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
omap24xxcam_poll(struct file *file, struct poll_table_struct *wait)
{
	struct omap24xxcam_fh *fh = file->private_data;
	struct omap24xxcam_device *cam = fh->cam;
	struct videobuf_buffer *vb;
	enum v4l2_field field;

	omap24xxcam_suspend_lockout(cam, file);

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
		if (fh->vbq.read_buf == NULL) {
			/* need to capture a new image */
			fh->vbq.read_buf = videobuf_alloc(&fh->vbq);
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

			omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
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
omap24xxcam_read(struct file *file, char *data, size_t count, loff_t * ppos)
{
	struct omap24xxcam_fh *fh = file->private_data;
	struct omap24xxcam_device *cam = fh->cam;
	struct omap24xxcam_fh *preview_fh;
	unsigned long irqflags;
	int free_sgdma, err;

	omap24xxcam_suspend_lockout(cam, file);

	/* user buffer has to be word aligned */
	if (((unsigned int) data & 0x3) != 0)
		return -EIO;

	spin_lock(&cam->img_lock);
	if (cam->streaming) {
		spin_unlock(&cam->img_lock);
		return -EBUSY;
	}

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

	if (cam->cam_sensor->enter_still_capture)
		cam->cam_sensor->enter_still_capture(16, cam->sensor);
	omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
	if (((unsigned int) data & (DMA_THRESHOLD - 1)) == 0) {
		/* use zero-copy if user buffer is aligned with DMA_THRESHOLD */
		err = videobuf_read_one(&fh->vbq, data, count, ppos, file->f_flags & O_NONBLOCK);
	} else {
		/* if user buffer is not aligned with DMA_THRESHOLD, the last DMA
		   transfer to the first page frame is less than DMA_THRESHOLD.
		   Camera DMA physical channel 1 doesn't seem to handle this
		   partial transfer correctly. We end up with sync problem with
		   an offset of DMA_THRESHOLD. For this case, We will use kernel
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
		omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
		omap24xxcam_start_overlay_dma(cam);
	}

	return err;
}

static int
omap24xxcam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap24xxcam_fh *fh = file->private_data;
	struct omap24xxcam_device *cam = fh->cam;

	omap24xxcam_suspend_lockout(cam, file);
	return videobuf_mmap_mapper(&fh->vbq, vma);
}

static int
omap24xxcam_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
		  unsigned long arg)
{
	struct omap24xxcam_fh *fh = file->private_data;
	struct omap24xxcam_device *cam = fh->cam;

	omap24xxcam_suspend_lockout(cam, file);
	return video_usercopy(inode, file, cmd, arg, omap24xxcam_do_ioctl);
}

static int
omap24xxcam_release(struct inode *inode, struct file *file)
{
	struct omap24xxcam_fh *fh = file->private_data;
	struct omap24xxcam_device *cam = fh->cam;
	struct videobuf_queue *q = &fh->vbq;
	int i;

	omap24xxcam_suspend_lockout(cam, file);

	spin_lock(&cam->img_lock);
	/* turn off overlay */
	if (cam->previewing == fh) {
		cam->previewing = NULL;
		spin_unlock(&cam->img_lock);
		omap2_disp_disable_layer(cam->vid_preview);
		omap2_disp_release_layer(cam->vid_preview);
		omap2_disp_put_dss();
		spin_lock(&cam->img_lock);
	}

	/* stop streaming capture */
	if (cam->streaming == fh) {
		cam->streaming = NULL;
		spin_unlock(&cam->img_lock);
		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (NULL == q->bufs[i])
				continue;
			if (q->bufs[i]->memory == V4L2_MEMORY_USERPTR)
				omap24xxcam_vbq_release(q, q->bufs[i]);
		}
		videobuf_streamoff(q);
		spin_lock(&cam->img_lock);
	}
	spin_unlock(&cam->img_lock);

	/* release read_buf videobuf_buffer struct */
	if (fh->vbq.read_buf) {
		omap24xxcam_vbq_release(q, fh->vbq.read_buf);
		kfree(fh->vbq.read_buf);
	}

	/* free video_buffer objects */
	videobuf_mmap_free(q);

	file->private_data = NULL;
	kfree(fh);

	return 0;
}

static int
omap24xxcam_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct omap24xxcam_device *cam = saved_cam;
	struct omap24xxcam_fh *fh;

	if (!cam || !cam->vfd || (cam->vfd->minor != minor))
		return -ENODEV;

	omap24xxcam_suspend_lockout(cam, file);

	/* allocate per-filehandle data */
	fh = kmalloc(sizeof (*fh), GFP_KERNEL);
	if (NULL == fh)
		return -ENOMEM;
	file->private_data = fh;
	fh->cam = cam;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	videobuf_queue_pci_init(&fh->vbq, &cam->vbq_ops, NULL, &cam->vbq_lock,
				fh->type, V4L2_FIELD_NONE,
				sizeof (struct videobuf_buffer), fh);

	return 0;
}

static struct file_operations omap24xxcam_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = omap24xxcam_read,
	.poll = omap24xxcam_poll,
	.ioctl = omap24xxcam_ioctl,
	.mmap = omap24xxcam_mmap,
	.open = omap24xxcam_open,
	.release = omap24xxcam_release,
};

/* -------------------------------------------------------------------------- */
#ifdef CONFIG_PM
static int
omap24xxcam_suspend(struct platform_device *dev, pm_message_t state)
{
	struct omap24xxcam_device *cam = platform_get_drvdata(dev);

	if (cam->suspended)
		return 0;
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

	/* Get the CC_CTRL register value for the current capture format. */
	omap24xxcam_sensor_cc_ctrl(cam);

	/* Disable the camera interface. */
	cc_reg_out(cam, CC_CTRL,
		   cam->cc_ctrl & ~(CC_CTRL_CC_EN | CC_CTRL_CC_FRAME_TRIG));

	/* Stop the DMA controller and frame sync scatter-gather DMA. */
	omap24xxcam_sg_dma_sync(cam, CAMDMA_CSR_TRANS_ERR);

	/* power down the sensor */
	cam->cam_sensor->power_off(cam->sensor);

	/* stop XCLK */
	cc_reg_out(cam, CC_CTRL_XCLK, CC_CTRL_XCLK_DIV_STABLE_LOW);

	clk_disable(cam->cami);
	clk_disable(cam->camf);

	return 0;
}
static int
omap24xxcam_resume(struct platform_device *dev)
{
	struct omap24xxcam_device *cam = platform_get_drvdata(dev);

	if (!cam->suspended)
		return 0;

	clk_enable(cam->cami);
	clk_enable(cam->camf);

	if (cpu_is_omap2420())
		cam_init(cam);
	else
		cc_init(cam, 1);

	/* set XCLK */
	omap24xxcam_set_xclk(cam);

	/* power up the sensor */
	cam->cam_sensor->power_on(cam->sensor);

	if (cam->streaming || cam->previewing) {
		/* capture or previewing was in progress, so we need to register
		 * our routine to restart the camera interface the next time a
		 * DMA transfer is queued.
		 */
		omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
	}
	if (cam->previewing) {
		omap2_disp_get_dss();
		omap2_disp_enable_layer(cam->vid_preview);
		omap24xxcam_start_overlay_dma(cam);
	}
	omap24xxcam_sg_dma_process(cam);

	/* camera interface will be enabled through dma_notify function
	 ** automatically when new dma starts
	 */

	/* wake up applications waiting on suspend queue */
	cam->suspended = 0;
	wake_up(&cam->suspend_wq);
	return 0;
}
#endif				/* PM */

#ifdef CONFIG_DPM
static struct constraints omap24xxcam_constraints = {
	.count = 2,
	.param = {
		  {DPM_MD_V, OMAP24XX_V_MIN, OMAP24XX_V_MAX},
		  {DPM_MD_SLEEP_MODE, PM_SUSPEND_STANDBY, PM_SUSPEND_MEM},
		  },
};

static int
omap24xxcam_pre_scale_handler(struct notifier_block *op, 
			unsigned long level, void *ptr)
{
	struct omap24xxcam_device *cam = saved_cam;

	switch (level) {
	case SCALE_PRECHANGE:
		omap24xxcam_sensor_cc_ctrl(cam);
		cc_reg_out(cam, CC_CTRL,
			   cam->
			   cc_ctrl & ~(CC_CTRL_CC_EN | CC_CTRL_CC_FRAME_TRIG));
		omap24xxcam_sg_dma_sync(cam, CAMDMA_CSR_TRANS_ERR);
		cc_reg_out(cam, CC_CTRL_XCLK, CC_CTRL_XCLK_DIV_STABLE_LOW);
		break;

	case SCALE_POSTCHANGE:
		break;
	}
	return 0;
}
static int
omap24xxcam_post_scale_handler(struct notifier_block *op, 
			unsigned long level, void *ptr)
{
	struct omap24xxcam_device *cam = saved_cam;

	switch (level) {
	case SCALE_PRECHANGE:
		break;

	case SCALE_POSTCHANGE:
		if (cpu_is_omap2420())
			cam_init(cam);
		else
			cc_init(cam, 1);
		omap24xxcam_set_xclk(cam);
		if (cam->streaming || cam->previewing) {
			omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
		}
		if (cam->previewing)
			omap24xxcam_start_overlay_dma(cam);
		omap24xxcam_sg_dma_process(cam);
		break;

	}
	return 0;
}
static struct notifier_block omap24xxcam_pre_scale = {
	.notifier_call = omap24xxcam_pre_scale_handler,
};

static struct notifier_block omap24xxcam_post_scale = {
	.notifier_call = omap24xxcam_post_scale_handler,
};

#endif 			/* DPM */

static int
omap24xxcam_probe(struct platform_device *dev)
{
	return 0;
}
static void
omap24xxcam_dev_release(struct device *dev)
{
}

static struct platform_device omap24xxcam_dev = {
	.name = CAM_NAME,
	.id = -1,
	.dev = {
		.release = omap24xxcam_dev_release,
#ifdef CONFIG_DPM
		.constraints = &omap24xxcam_constraints,
#endif
	},
};

static struct platform_driver omap24xxcam_driver = {
	.driver = {
		.name = CAM_NAME,
	},
	.probe = omap24xxcam_probe,
#ifdef CONFIG_PM
	.suspend = omap24xxcam_suspend,
	.resume = omap24xxcam_resume,
#endif
};

static int __init
omap24xxcam_init(void)
{
	struct omap24xxcam_device *cam;
	struct video_device *vfd;
	int ret;

	ret = platform_driver_register(&omap24xxcam_driver);
	if (ret != 0)
		return ret;
	ret = platform_device_register(&omap24xxcam_dev);
	if (ret != 0) {
		platform_driver_unregister(&omap24xxcam_driver);
		return ret;
	}
#ifdef CONFIG_DPM
	/* Scaling is enabled only when DPM is enabled */
	dpm_register_scale(&omap24xxcam_pre_scale,SCALE_PRECHANGE);
	dpm_register_scale(&omap24xxcam_post_scale,SCALE_POSTCHANGE);
#endif

	cam = kmalloc(sizeof (struct omap24xxcam_device), GFP_KERNEL);
	if (!cam) {
		printk(KERN_ERR CAM_NAME ": could not allocate memory\n");
		goto init_error;
	}
	memset(cam, 0, sizeof (struct omap24xxcam_device));
	saved_cam = cam;

	/* set driver specific data to use in power mgmt functions */
	platform_set_drvdata(&omap24xxcam_dev, cam);

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
	vfd->fops = &omap24xxcam_fops;
	video_set_drvdata(vfd, cam);
	vfd->minor = -1;

	/* initialize the videobuf queue ops */
	cam->vbq_ops.buf_setup = omap24xxcam_vbq_setup;
	cam->vbq_ops.buf_prepare = omap24xxcam_vbq_prepare;
	cam->vbq_ops.buf_queue = omap24xxcam_vbq_queue;
	cam->vbq_ops.buf_release = omap24xxcam_vbq_release;
	spin_lock_init(&cam->vbq_lock);

	/* Impose a lower limit on the amount of memory allocated for capture.
	 * We require at least enough memory to double-buffer QVGA (300KB).
	 */
	if (capture_mem < QVGA_SIZE * 2)
		capture_mem = QVGA_SIZE * 2;

	/* request the mem region for the camera registers */
	if (!request_mem_region(CAM_REG_BASE, CAM_REG_SIZE, vfd->name)) {
		printk(KERN_ERR CAM_NAME
		       ": cannot reserve camera register I/O region\n");
		goto init_error;
	}
	cam->cam_mmio_base_phys = CAM_REG_BASE;
	cam->cam_mmio_size = CAM_REG_SIZE;

	/* map the region */
	cam->cam_mmio_base = (unsigned long)
	    ioremap(cam->cam_mmio_base_phys, cam->cam_mmio_size);
	if (!cam->cam_mmio_base) {
		printk(KERN_ERR CAM_NAME
		       ": cannot map camera register I/O region\n");
		goto init_error;
	}

	/* allocate coherent memory for the overlay framebuffer */
	cam->overlay_size = overlay_mem;
	if (cam->overlay_size > 0) {
		cam->overlay_base = (unsigned long) dma_alloc_coherent(NULL,
			cam->overlay_size,
			(dma_addr_t *) &cam->overlay_base_phys,
			GFP_KERNEL | GFP_DMA);
		if (!cam->overlay_base) {
			printk(KERN_ERR CAM_NAME
			       ": cannot allocate overlay framebuffer\n");
			goto init_error;
		}
	}
	memset((void *) cam->overlay_base, 0, cam->overlay_size);
	
	/* this is a VRFB address in the case of rotation */
	cam->overlay_base_dma = cam->overlay_base_phys;

	/* initialize the overlay spinlock  */
	spin_lock_init(&cam->overlay_lock);

	/* Enable interface & functional clocks */
	cam->cami = clk_get(NULL,"cam_ick");
	cam->camf = clk_get(NULL,"cam_fck");
	if (!cam->cami || !cam->camf) {
		printk(KERN_ERR CAM_NAME ": cannot get clocks\n");
		goto init_error;
	}
	clk_enable(cam->cami);
	clk_enable(cam->camf);

	/* initialize the camera interface */
	cam_init(cam);

	/* initialize the spinlock used to serialize access to the image
	 * parameters
	 */
	spin_lock_init(&cam->img_lock);
	cam->cam_sensor = &camera_sensor_if;

	/* initialize the camera interface functional clock frequency */
	cam->mclk = DEFAULT_CAM_FUNC_CLK;

	/* initialize the streaming capture parameters */
	cam->cparm.readbuffers = 1;
	cam->cparm.capability = V4L2_CAP_TIMEPERFRAME;

	/* Enable the xclk output.  The sensor may (and does, in the case of
	 * the OV9640) require an xclk input in order for its initialization
	 * routine to work.
	 */
	cam->xclk = DEFAULT_SENSOR_XCLK;	/* choose an arbitrary xclk frequency */
	omap24xxcam_set_xclk(cam);

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

	/* select an arbitrary default capture frame rate of 15fps */
	cam->nominal_timeperframe.numerator = 1;
	cam->nominal_timeperframe.denominator = 15;

	/* initialize the sensor and define a default capture format cam->pix */
	cam->sensor = cam->cam_sensor->init(&cam->pix, &cam->pix2);
	if (!cam->sensor) {
		printk(KERN_ERR CAM_NAME ": cannot initialize sensor\n");
		goto init_error;
	}
	printk(KERN_INFO "Sensor is %s\n", cam->cam_sensor->name);

	/* calculate xclk based on the default capture format and default
	 * frame rate
	 */
	cam->xclk = cam->cam_sensor->calc_xclk(&cam->pix,
					       &cam->nominal_timeperframe,
					       cam->sensor);
	omap24xxcam_adjust_xclk(cam);
	cam->cparm.timeperframe = cam->nominal_timeperframe;

	/* program the camera interface to generate the new xclk frequency */
	omap24xxcam_set_xclk(cam);

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

	/* initialize the DMA driver */
	omap24xxcam_dma_init(cam);

	/* install the interrupt service routine */
	if (request_irq(INT_24XX_CAM_IRQ, omap24xxcam_isr, 0, CAM_NAME, cam)) {
		printk(KERN_ERR CAM_NAME
		       ": could not install interrupt service routine\n");
		goto init_error;
	}
	cam->irq = INT_24XX_CAM_IRQ;

	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		printk(KERN_ERR CAM_NAME
		       ": could not register Video for Linux device\n");
		goto init_error;
	}
	else
		vfd->minor = video_nr;

	printk(KERN_INFO CAM_NAME
	       ": registered device video%d [v4l2]\n", vfd->minor);

#undef CAMERA_TEST
#ifdef CAMERA_TEST
	if (cam->pix.sizeimage <= cam->overlay_size) {
		printk(KERN_INFO CAM_NAME ": Camera test--enabling overlay\n");
		omap2_disp_get_dss();
		/* turn on the video overlay */
		omap2_disp_config_vlayer(cam->vid_preview, &cam->pix,
					 &cam->preview_crop, &cam->win, -1, 0);
		omap2_disp_start_vlayer(cam->vid_preview, &cam->pix, &cam->preview_crop,
					cam->overlay_base_dma, -1, 0);
		cam->previewing = (struct omap24xxcam_fh *) 1;
		/* start the camera interface */
		omap24xxcam_dma_notify(cam, omap24xxcam_cc_enable);
		omap24xxcam_start_overlay_dma(cam);
	} else {
		printk(KERN_INFO CAM_NAME
		       ": Can't start camera test--overlay buffer too"
		       " small\n");
	}
#endif

	return 0;

      init_error:
	omap24xxcam_cleanup();
	return -ENODEV;
}

static void
omap24xxcam_cleanup(void)
{
	struct omap24xxcam_device *cam = saved_cam;
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
		}
		cam->vfd = NULL;
	}

	if (cam->irq) {
		free_irq(cam->irq, cam);
		cam->irq = 0;
	}

	omap24xxcam_dma_exit(cam);
	if (cam->sensor)
		cam->cam_sensor->cleanup(cam->sensor);
	/* sensor allocated private data is gone */
	cam->sensor = NULL;

	if (cam->overlay_base) {
		dma_free_coherent(NULL, cam->overlay_size,
			(void *)cam->overlay_base, cam->overlay_base_phys);
		cam->overlay_base = 0;
	}
	cam->overlay_base = cam->overlay_base_phys = 0;

	if (cam->cam_mmio_base) {
		iounmap((void *) cam->cam_mmio_base);
		cam->cam_mmio_base = 0;
	}
	if (cam->cam_mmio_base_phys) {
		release_mem_region(cam->cam_mmio_base_phys, cam->cam_mmio_size);
		cam->cam_mmio_base_phys = 0;
	}

#ifdef CONFIG_DPM
	dpm_unregister_scale(&omap24xxcam_pre_scale,SCALE_PRECHANGE);
	dpm_unregister_scale(&omap24xxcam_post_scale,SCALE_POSTCHANGE);
#endif
	platform_device_unregister(&omap24xxcam_dev);
	platform_driver_unregister(&omap24xxcam_driver);

	if (cam->cami) {
		clk_disable(cam->cami);
		clk_put(cam->cami);
	}
	if (cam->camf) {
		clk_disable(cam->camf);
		clk_put(cam->camf);
	}

	kfree(cam);
	saved_cam = NULL;
}

MODULE_AUTHOR("MontaVista Software, Inc.");
MODULE_DESCRIPTION("OMAP24xx Video for Linux camera driver");
MODULE_LICENSE("GPL");
module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr,
		"Minor number for video device (-1 ==> auto assign)");
module_param(capture_mem, int, 0);
MODULE_PARM_DESC(capture_mem,
	"Maximum amount of memory for capture buffers (default 4800KB)");
module_param(overlay_mem, int, 0);
MODULE_PARM_DESC(overlay_mem,
	"Preview overlay framebuffer size (default 600KB)");

module_init(omap24xxcam_init);
module_exit(omap24xxcam_cleanup);
