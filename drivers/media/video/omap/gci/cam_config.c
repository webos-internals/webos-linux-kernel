/*
 * drivers/media/video/omap/gci/cam_config.c
 *
 * Camera input port configurations
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

#include <asm/types.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <asm/semaphore.h>
#include <asm/arch/io.h>


#include "../isp/ispreg.h"
#include "../isp/isp.h"
#include "../isp/ispccdc.h"
#include "../isp/isppreview.h"
#include "../isp/ispresizer.h"
#include "../isp/ispmmu.h"
#include "../isp/isph3a.h"

#include "isc.h"
#include "cam_config.h"

#ifndef CONFIG_ARCH_OMAP3410
#define USE_ISP_PREVIEW
#define USE_ISP_RESZ
#endif


#define DEFAULT_SENSOR_XCLK  12000000	/* 12MHz */
#define VGA_SIZE 	QVGA_SIZE*4	/* memory needed for VGA image */

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QCIF_WIDTH	176
#define QCIF_HEIGHT	144
#define MAX_RESIZER_INPUTSIZE	1280

#define ISPBUF_SIZE 2048*1536*2/4

int camcfg_suspended = 1;
static struct camcfg_device *cam;
static volatile u8 ispbuf_use;
static volatile u8 ispstop = 0;
static volatile isp_actual_tempbuf;
static volatile u8 cam_started;
static void camcfg_configure_inp_interface(void);

static spinlock_t isc_temp_buf_lock;
static int isc_temp_state = 0;
static struct captureparams cap_parms;

#if 0
#define DPRINTK_CAMCFG(format,...)\
	printk("CAMCFG: " format, ## __VA_ARGS__)
#else
#define DPRINTK_CAMCFG(format,...)
#endif

static void
camcfg_callback(unsigned long status, void *arg1, void *arg2);
/* ------------------------------------------------------------------------- */

/*
 * This method Updates the buffer address for the camera hardware
 * to fillup the data. The address is palced at the module which
 * is at the end of the pipeline. This is called with the value
 * returned by the camcfg_map_va_buf();
 * buf_addr	: ISPMMU mapped address
 */
void
camcfg_set_curr_buf(u32 buf_addr)
{
	/* If buff address other than the ispbuf is passed
	 * use the set one */
	if((buf_addr != cam->ispbuf_mmuaddr) ||
		(buf_addr != cam->ispbuf_base_phys))
		ispbuf_use = 0;
	DPRINTK_CAMCFG("curr_buf_addr = 0x%x\n",buf_addr);
	if (!camcfg_suspended) {
#ifdef USE_ISP_RESZ
		/* This has to occur before the vysnc of the
		 * intended frame comes
		 */
		if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
			ispresizer_set_outaddr(buf_addr);
		}
		else
#endif
#ifdef USE_ISP_PREVIEW
		if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
			isppreview_set_outaddr(buf_addr);
		}
		else
#endif
		if (cam->isp_pipeline & OMAP_ISP_CCDC) {
			ispccdc_set_outaddr(buf_addr);
		}
	}
}

/*
 * Sets the callback to notify that the buffer given to the
 * camera hardware is filled up and ready to consume.
 * cbarg	: Callback method and its arguments.
 */
void
camcfg_set_callback(struct callback_arg *cbkarg)
{
	cam->cbk.callback = cbkarg->callback;
	cam->cbk.arg1 = cbkarg->arg1;
	cam->cbk.arg2 = cbkarg->arg2;
	/* If callback is set back again, then
 	 * it means that it is called from qbuf
 	 * but cannot start using the buffer if
 	 * set_curr_buf() is not called. So do the ispbuf = 0
 	 * in set_curr_buf()
 	 */
}

static void
camcfg_callback(unsigned long status, void *arg1, void *arg2)
{
	struct camcfg_device *cam = (struct camcfg_device *) arg1;
	unsigned long irqflags;
	spin_lock_irqsave(&cam->lock, irqflags);
	DPRINTK_CAMCFG("reallcalback %x\n",status);
	switch (status) {
			case MMU_ERR:
				printk(KERN_ERR "\nMMU mapping error\n");
				spin_unlock_irqrestore(&cam->lock, irqflags);
				return;
			case CCDC_VD0:
				/* Program shadow registers for CCDC */
				ispccdc_config_shadow_registers();
				/* Check if CCDC is the last
				 * module in the pipeline
				 */
				if ((cam->isp_pipeline & OMAP_ISP_RESIZER) ||
					(cam->isp_pipeline & OMAP_ISP_PREVIEW)) {
					/* Only Program the Shadow
					 * registers if needed
					 */
					spin_unlock_irqrestore(&cam->lock,
							irqflags);
					return;
				}
				else {
					spin_lock(isc_temp_buf_lock);
					if(isc_temp_state != ISP_INIT){
						spin_unlock(isc_temp_buf_lock);
						spin_unlock_irqrestore(&cam->lock,
								irqflags);
						return;
					}
					else
						break;
					spin_unlock(isc_temp_buf_lock);
					break;
				}
			case CCDC_VD1:
				spin_lock(isc_temp_buf_lock);
				if(isc_temp_state != ISP_INIT){
					spin_unlock(isc_temp_buf_lock);
					spin_unlock_irqrestore(&cam->lock,
							irqflags);
					return;
				}
				ispccdc_enable(0);
				spin_unlock(isc_temp_buf_lock);
				spin_unlock_irqrestore(&cam->lock, irqflags);
				return;
#if defined (USE_ISP_PREVIEW) || defined (USE_ISP_RESZ)
			case PREV_DONE:
				/* Program shadow registers for PREVIEW */
				isppreview_config_shadow_registers();
				/* Check if PREVIEW is the last module in the pipeline */
				if (cam->isp_pipeline & OMAP_ISP_RESIZER){
					spin_unlock_irqrestore(&cam->lock,
							irqflags);
					return;
				}
				else {

				break;

				}
			case RESZ_DONE:
				/* Program shadow registers for RESIZER */
				ispresizer_config_shadow_registers();
				spin_lock(isc_temp_buf_lock);
				if(isc_temp_state != ISP_INIT) {
					/* TBD for raw seensor */
				/*	if (ispstop) {
						printk("from resizer\n");
					camcfg_stop();
					}*/
					spin_unlock(isc_temp_buf_lock);
					spin_unlock_irqrestore(&cam->lock,
							irqflags);
					return;
				}
				spin_unlock(isc_temp_buf_lock);
				break;
#endif
			case HS_VS:
				spin_lock(isc_temp_buf_lock);
				if(isc_temp_state == ISP_TRAN)
				{
					isp_CCDC_VD01_enable();
					isc_temp_state = ISP_INIT;
				}
				if(isc_temp_state == ISP_FREE_RUNNING) {
					printk("vsync %d \n",ispccdc_busy());
				}
				spin_unlock(isc_temp_buf_lock);
				spin_unlock_irqrestore(&cam->lock,
						irqflags);
				return;

			default:
				spin_unlock_irqrestore(&cam->lock,
						irqflags);
				return;
	}

	cam->frame_ctr++;

	/* ISP RSZ only operates in one-shot mode. Need to re-enable it */

	status = 0; /* No error when it reaches here */
	if(cam->cbk.callback){
		if(cap_parms.cap_shots){
			/* Check if the number of frames elapsed for
			 * the first shot*/
			if(cam->frame_ctr == cap_parms.cap_delay_1stshot+1){
				cam->signal_buf_filled = 1;
			}
			else if((cam->frame_ctr >
				cap_parms.cap_delay_1stshot + 1) &&
				(!((cam->frame_ctr -
				(cap_parms.cap_delay_1stshot+1))
				% (cap_parms.cap_delay_intershot+1)))){
				cam->signal_buf_filled = 1;
			}
			else if(cam->frame_ctr == (cap_parms.cap_delay_1stshot
					+ 1 + (cap_parms.cap_delay_intershot *
					 cap_parms.cap_shots))){
				cam->signal_buf_filled = 1;
				cam->frame_ctr = 0;
				}
		}
		else
			cam->signal_buf_filled = 1;
		if(cam->signal_buf_filled)
			cam->cbk.callback(status,cam->cbk.arg1, cam->cbk.arg2);
		else{
		spin_lock(isc_temp_buf_lock);
			isc_temp_state = ISP_INIT;
		spin_unlock(isc_temp_buf_lock);
		}
		cam->signal_buf_filled = 0;
	}
	if (cam->isp_pipeline == OMAP_ISP_CCDC)
		ispccdc_enable(1);

#ifdef USE_ISP_RESZ
	if (cam->isp_pipeline & OMAP_ISP_RESIZER){
		ispresizer_enable(1);
	}
#endif
	spin_unlock_irqrestore(&cam->lock, irqflags);
}

/*
 * The callback is made NULL, so that the Camera config callback does not
 * call this callback.  Also indicates the camera config to use its
 * own isp buffer since no user to consume the camera buffer
 * Return	:Error if the camera config is not able to switch to
 * 		its own isp buffer.
 */
int
camcfg_unset_callback(void)
{
	DPRINTK_CAMCFG("+CAMCFG unset_callback\n");
	spin_lock(isc_temp_buf_lock);
	if (cam_started) {
		isc_temp_state = ISP_FREE_RUNNING;
		ispstop = 1;
	}

	if(isc_temp_state == ISP_FREE_RUNNING){
		camcfg_stop();
	}
	spin_unlock(isc_temp_buf_lock);
	cam->cbk.callback = NULL;
	cam->cbk.arg1 = NULL;
	cam->cbk.arg2 = NULL;
	DPRINTK_CAMCFG("-CAMCFG unset_callback\n");
	return 0;
}

int
camcfg_useispbuf(int state)
{
	spin_lock(isc_temp_buf_lock);
		isc_temp_state = state;
	if((isc_temp_state == ISP_TRAN) &&
			(cam->isp_pipeline == OMAP_ISP_CCDC) &&
			!camcfg_suspended)
		ispccdc_enable(1);
	else if((isc_temp_state == ISP_INIT) &&
			(cam->isp_pipeline == OMAP_ISP_CCDC) &&
			!camcfg_suspended)
		ispccdc_enable(1);
	else if((isc_temp_state == ISP_FREE_RUNNING) && !camcfg_suspended)
		isp_CCDC_VD01_disable();
	spin_unlock(isc_temp_buf_lock);
		/* TBD Check if needed to wait till ispbuf_use =1 */
	return state;
}
/*
 * Start the Camera ie each modules in the camera pipeline to give frames
 * Also sets the camcfg callback to the ISP ctrl ISR.
 */
void
camcfg_start(void)
{
	DPRINTK_CAMCFG("+CAM_CFG start isp\n");
	cam->frame_ctr = 0;
	cam->signal_buf_filled = 0;

#ifdef USE_ISP_RESZ
	if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
	isp_set_callback(CBK_RESZ_DONE, camcfg_callback, (void*)cam, 0);
	}
#endif

#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
	isp_set_callback(CBK_PREV_DONE, camcfg_callback, (void*)cam, 0);
	}
#endif
	if (cam->isp_pipeline & OMAP_ISP_CCDC) {
	isp_set_callback(CBK_CCDC_VD0, camcfg_callback, (void*)cam, 0);
	}
	isp_set_callback(CBK_HS_VS, camcfg_callback, (void*)cam, 0);

	if(cam->isp_pipeline == OMAP_ISP_CCDC)
		isp_set_callback(CBK_CCDC_VD1, camcfg_callback, (void*)cam, 0);
	/* start the needed isp components assuming these components
	 * are configured correctly.
	 */
	camcfg_start_isp();
	cam_started = 1;
	DPRINTK_CAMCFG("-CAM_CFG start isp\n");
	return;
}

void
camcfg_start_isp()
{
#ifdef USE_ISP_RESZ
	if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
		ispresizer_enable(1);
	}
#endif
#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
		isppreview_enable(1);
	}
#endif
	if ((cam->isp_pipeline & OMAP_ISP_CCDC) &&
			(cam->isp_pipeline != OMAP_ISP_CCDC)) {
		ispccdc_enable(1);
	}
}

/*
 * Stop giving frames by stopping all the modules in the pipeline.
 * Unsets the camcfg callback from ISP ctrl isr.
 */
int
camcfg_stop(void)
{
	DPRINTK_CAMCFG("+CAM_CFG stop isp\n");

	if(cam_started && (cam->isp_pipeline == OMAP_ISP_CCDC)) {
		ispstop = 1;
		while(ispccdc_busy()){
			ispccdc_enable(0);
			udelay(5);
			}
		ispstop =0;
	}
	if (cam_started) {
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
	isp_unset_callback(CBK_HS_VS);
	if(cam->isp_pipeline == OMAP_ISP_CCDC)
		isp_unset_callback(CBK_CCDC_VD1);

	camcfg_stop_isp();
	
	cam_started = 0;
	isc_temp_state = 0;
	ispstop = 0;
	DPRINTK_CAMCFG("-CAM_CFG stop isp\n");
	DPRINTK_CAMCFG("-CAM_CFG CCDC_busy = 0x%x\n",ispccdc_busy());
#ifdef USE_ISP_PREVIEW
	DPRINTK_CAMCFG("-CAM_CFG Preview_busy = 0x%x\n",isppreview_busy());
#endif
#ifdef USE_ISP_RESZ
	DPRINTK_CAMCFG("-CAM_CFG Resizer_busy = 0x%x\n",ispresizer_busy());
#endif
	/* TBD return error ie timeout if cam could not
	 * be stopped.
	 */
	}
	return 0;
}

int
camcfg_stop_isp()
{
	int timeout;
#ifdef USE_ISP_RESZ
	if (cam->isp_pipeline & OMAP_ISP_RESIZER) {
		ispresizer_enable(0);
		timeout = 0;
		while (ispresizer_busy() && (timeout < 20)) {
			timeout++;
			mdelay(10);
		}
	}
#endif
#ifdef USE_ISP_PREVIEW
	if (cam->isp_pipeline & OMAP_ISP_PREVIEW) {
		isppreview_enable(0);
		timeout = 0;
		while (isppreview_busy() && (timeout < 20)) {
			timeout++;
			mdelay(10);
		}
	}
#endif
	if (cam->isp_pipeline & OMAP_ISP_CCDC) {
		ispccdc_enable(0);
		timeout = 0;
		while (ispccdc_busy() && (timeout < 20)) {
			timeout++;
			mdelay(10);
		}
	}

	if (ispccdc_busy()) {
		isp_save_ctx();
		omap_writel(omap_readl(ISP_SYSCONFIG) | ISP_SYSCONFIG_SOFTRESET,
				ISP_SYSCONFIG);
		while(!(omap_readl(ISP_SYSSTATUS) & 0x1)) {
			mdelay(1);
	 	}
	isp_restore_ctx();
	}
}
/* Prepares the input/output sizes of the isp modules for the given
 * input size(output size of the sensor) and output size (output size
 * of the Camera driver). It chooses the isp modules to be considered
 * based on the pipeline. The prepared sizes are placed in the void*ptr
 * after typecasting to struct isc_sizes
 * pix		: Output frame size requested from Camera
 * inpsize	: Input frame size (ie sensor output) tothe Camera
 * Return	: ptr - Filled with struct isc_sizes which is used to
 * 		 store the prepared input/output sizes of each of
 * 		 the isp module.
 */
void
camcfg_prepare_size(struct frame_size *pix, struct frame_size *inpsize,
			void* ptr)
{
	struct frame_size pixfmt = *pix;
	struct isc_sizes *isc_sizeptr = ptr;

	unsigned int resizer_w, resizer_h;
	unsigned int width = pix->width;
	unsigned int height = pix->height;


	DPRINTK_CAMCFG("+ CAM_CFG camcfg_prepare_size %d %d %d %d\n",
		inpsize->width, inpsize->height, pix->width, pix->height);
	/* TBD to be removed later */
	/*Sensor try format returns SCALAR ON for now */
	pixfmt.width = inpsize->width;
	pixfmt.height = inpsize->height;
	cam->pix_raw = pixfmt;
	cam->ccdc_input_width  = pixfmt.width;
	cam->ccdc_input_height = pixfmt.height;

	switch(cam->isp_pipeline){
		case CCDC_ALONE:
			ispccdc_try_size(pixfmt.width, pixfmt.height,
					&pix->width, &pix->height);
			cam->ccdc_width = pix->width;
			cam->ccdc_height = pix->height;
			break;
#if defined (USE_ISP_PREVIEW) || defined (USE_ISP_RESZ)
		case CCDC_PREV_RESZ:
			/* then negotiate the size with isp ccdc driver */
			ispccdc_try_size(pixfmt.width, pixfmt.height,
					&pix->width, &pix->height);
			cam->ccdc_width = pix->width;
			cam->ccdc_height = pix->height;
			pixfmt.width = width;
			pixfmt.height = height;
			isppreview_try_size(pix->width, pix->height,
					&pixfmt.width, &pixfmt.height);
			cam->preview_width = pixfmt.width;
			cam->preview_height = pixfmt.height;
			pix->width = width;
			pix->height = height;
			resizer_w = pixfmt.width;
			resizer_h = pixfmt.height;
			ispresizer_try_size(&resizer_w, &resizer_h,
					&pix->width, &pix->height);
			cam->resizer_width = pix->width;
			cam->resizer_height = pix->height;
			DPRINTK_CAMCFG("prepare size %x %x %x %x %d %d \n",
					cam->ccdc_width,cam->ccdc_height,
					cam->preview_width,cam->preview_height,
					width,height);
		break;
		case CCDC_RESZ:
			ispccdc_try_size(pixfmt.width, pixfmt.height,
					&pix->width, &pix->height);
				cam->ccdc_width = pix->width;
				cam->ccdc_height = pix->height;
			pixfmt.width = width;
			pixfmt.height = height;
			pix->width = width;
			pix->height = height;
			resizer_w = pixfmt.width;
			resizer_h = pixfmt.height;
			ispresizer_try_size(&resizer_w, &resizer_h,
					&pix->width, &pix->height);
			cam->resizer_width = pix->width;
			cam->resizer_height = pix->height;
		break;
		case CCDC_PREV:
			ispccdc_try_size(pixfmt.width, pixfmt.height,
					&pix->width, &pix->height);
				cam->ccdc_width = pix->width;
				cam->ccdc_height = pix->height;
			pixfmt.width = width;
			pixfmt.height = height;
			isppreview_try_size(pix->width, pix->height,
					&pixfmt.width, &pixfmt.height);
			cam->preview_width = pixfmt.width;
			cam->preview_height = pixfmt.height;
			DPRINTK_CAMCFG("prepare size %x %x %x %x %d %d \n",
					cam->ccdc_width,cam->ccdc_height,
					cam->preview_width,cam->preview_height,
					width,height);
		break;
#endif
	}
	/*Fill up the isc_sizes with width/height for each module */
	isc_sizeptr->ccdc_inp_w = cam->pix_raw.width;
	isc_sizeptr->ccdc_inp_h = cam->pix_raw.height;
	isc_sizeptr->ccdc_out_w = cam->ccdc_width;
	isc_sizeptr->ccdc_out_h = cam->ccdc_height;
	isc_sizeptr->prev_out_w = cam->preview_width;
	isc_sizeptr->prev_out_h = cam->preview_height;
	isc_sizeptr->resz_out_w = cam->resizer_width;
	isc_sizeptr->resz_out_h = cam->resizer_height;
	DPRINTK_CAMCFG("isc_sizeptr: ccdc_inp (%d, %d) ccdc_out (%d, %d)\
		 prev_out (%d, %d) resz_out (%d, %d)\n",
		isc_sizeptr->ccdc_inp_w,
		isc_sizeptr->ccdc_inp_h,
		isc_sizeptr->ccdc_out_w,
		isc_sizeptr->ccdc_out_h,
		isc_sizeptr->prev_out_w,
		isc_sizeptr->prev_out_h,
		isc_sizeptr->resz_out_w,
		isc_sizeptr->resz_out_h);
	DPRINTK_CAMCFG("- CAM_CFG camcfg_prepare_size\n");
	return;
}

/* The isp modules are configured for their data output format for each e
 * engine in the pipeline.
 * Along with the input/output data format of the modules, default
 * configuration of each of the modules are done
 * except for input/output sizes.
 */
void
camcfg_configure_pixformat(void)
{
	/* TBD Commented as of now since isp_ctrl reg
	 * is going mad was called in camdriver before config pipeline
	 */
	camcfg_configure_inp_interface();
	switch(cam->isp_pipeline){
		case CCDC_ALONE:
			DPRINTK_CAMCFG("here 0x%x",CCDC_ALONE);
			if(cam->pix_inpfmt != PIX_FMT_BAYER10_GrRBGb)
			ispccdc_config_datapath(CCDC_YUV_SYNC, CCDC_OTHERS_MEM);
			else
			ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_MEM);
			break;
#if defined (USE_ISP_PREVIEW) || defined (USE_ISP_RESZ)
		case CCDC_PREV_RESZ:
			DPRINTK_CAMCFG("here 0x%x",CCDC_PREV_RESZ);
			ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);
			isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_RSZ);
			ispresizer_config_datapath(RSZ_OTFLY_YUV);
			break;
		case CCDC_RESZ:
			DPRINTK_CAMCFG("here 0x%x",CCDC_RESZ);
			ispccdc_config_datapath(CCDC_RAW, CCDC_RESZ);
			ispresizer_config_datapath(RSZ_OTFLY_YUV);
			break;
		case CCDC_PREV:
			DPRINTK_CAMCFG("here 0x%x",CCDC_RESZ);
			ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);
			isppreview_config_datapath(PRV_RAW_CCDC,PREVIEW_MEM);
			break;
#endif
	};
}


/*
 * Based on the input pixelformat (output format of the sensor) and
 * the output pixelformat (output format of the camera driver),
 * the camera pipeline is set.
 * pix_outfmt	: The data format that need to come out of camera.
 * pix_inpfmt	: The data format coming into the camera from Sensor.
 */
void
camcfg_prepare_pixformat(enum cam_fmt pix_outfmt, void * pix_inpfmt)
{
	struct isc_sensor_info *inpfmt_info;

	DPRINTK_CAMCFG("+ CAM_CFG camcfg_prepare_pixformat \n");


	cam->pix_outfmt = pix_outfmt;

	inpfmt_info= (struct isc_sensor_info*)pix_inpfmt;

	DPRINTK_CAMCFG("+ CAM_CFG pix_outfmt = %d, pix_inpfmt = %d\n",
				pix_outfmt, inpfmt_info->dataformat);

	if(inpfmt_info->dataformat == SENSOR_BAYER10){
		cam->sensor->sensor_type = SENSOR_RAW;
		cam->pix_inpfmt = PIX_FMT_BAYER10_GrRBGb;
	}

	else if(inpfmt_info->dataformat == SENSOR_RGB565){
		cam->sensor->sensor_type = SENSOR_ISP;
		cam->pix_inpfmt = PIX_FMT_RGB565;
	}
	else if(inpfmt_info->dataformat == SENSOR_YUV422){
		cam->sensor->sensor_type = SENSOR_ISP;
		cam->pix_inpfmt = PIX_FMT_UYVY;
	}
	if(inpfmt_info->interface == SENSOR_PARLL) {
		cam->if_type = ISP_PARLL;
		cam->sensor->sensor_interface = SENSOR_PARALLEL;
	}
	else if (inpfmt_info->interface == SENSOR_SERIAL1) {
		cam->if_type = ISP_CSIA;
		cam->sensor->sensor_interface = SENSOR_SERIAL1;
	}
	else if (inpfmt_info->interface == SENSOR_SERIAL2) {
		cam->if_type = ISP_CSIB;
		cam->sensor->sensor_interface = SENSOR_SERIAL2;
	}
	/* TBD How to avoid multiple calls to this when pipeline changes*/
	if (isp_request_interface(cam->if_type)) {
		DPRINTK_CAMCFG("ERROR/WARN : cannot get isp interface\n");
	}

	DPRINTK_CAMCFG("- CAM_CFG camcfg_prepare_pixformat 0x%x\n",
			cam->isp_pipeline);
}

void
camcfg_set_pipeline()
{
	cam->isp_pipeline = 0;
	if(cam->sensor->sensor_interface == SENSOR_PARALLEL) {
		if (ispccdc_request()) {
			return -EBUSY;
		}
		cam->isp_pipeline |= OMAP_ISP_CCDC;
	}
	if (cam->pix_inpfmt == PIX_FMT_BAYER10_GrRBGb &&
		(cam->pix_outfmt == PIX_FMT_UYVY ||
		cam->pix_outfmt == PIX_FMT_YUYV)) {
		/* isp preview has to be used.
		 * isp resizer always used atleast for 1:1
		 * TBD revisit the resizer for Crop/zoom
		 */
#ifdef USE_ISP_PREVIEW
		if (isppreview_request()) {
			ispccdc_free();
			return -EBUSY;
		}
		cam->isp_pipeline |= OMAP_ISP_PREVIEW;
#endif
#ifdef USE_ISP_RESZ
		if (ispresizer_request()) {
			ispccdc_free();
#ifdef USE_ISP_PREVIEW
			isppreview_free();
#endif
			return -EBUSY;
		}
		cam->isp_pipeline |= OMAP_ISP_RESIZER;
#endif
	}
	/* TBD for Smart */
//	cam->isp_pipeline = OMAP_ISP_CCDC;	
}

/*
 * Configures the isp registers with the input output sizes
 * for each module in the pipeline.
 * cfgsize	: struct isc_size returned by camcfg_prepare_size()
 */
void
camcfg_configure_size(struct isc_sizes *cfgsize)
{
	DPRINTK_CAMCFG("+ CAM_CFG camcfg_configure_size\n");
	/* TBD This method success depend on whether
	 * prepare_size() had been called with the same inp/out
	 * sizes. To make it independent change the isp_config_size()
	 */
	if(cfgsize)
	{
	cam->pix_raw.width = cfgsize->ccdc_inp_w;
	cam->pix_raw.height = cfgsize->ccdc_inp_h;
	cam->ccdc_width = cfgsize->ccdc_out_w;
	cam->ccdc_height = cfgsize->ccdc_out_h;
	cam->preview_width = cfgsize->prev_out_w;
	cam->preview_height = cfgsize->prev_out_h;
	cam->resizer_width = cfgsize->resz_out_w;
	cam->resizer_height = cfgsize->resz_out_h;
	}

	DPRINTK_CAMCFG("cfgsize: ccdc_inp (%d, %d) ccdc_out (%d, %d)\
		 prev_out (%d, %d) resz_out (%d, %d)\n",
		cfgsize->ccdc_inp_w,
		cfgsize->ccdc_inp_h,
		cfgsize->ccdc_out_w,
		cfgsize->ccdc_out_h,
		cfgsize->prev_out_w,
		cfgsize->prev_out_h,
		cfgsize->resz_out_w,
		cfgsize->resz_out_h);

	switch(cam->isp_pipeline){
		case CCDC_ALONE:
			ispccdc_config_size(cam->pix_raw.width,
				cam->pix_raw.height,
				cam->ccdc_width,
				cam->ccdc_height);
		break;

#if defined (USE_ISP_PREVIEW) || defined (USE_ISP_RESZ)
		case CCDC_PREV_RESZ:
			ispccdc_config_size(cam->pix_raw.width,
				cam->pix_raw.height,
				cam->ccdc_width,
				cam->ccdc_height);
			isppreview_config_size(cam->ccdc_width,
				cam->ccdc_height,
				cam->preview_width,
				cam->preview_height);
			ispresizer_config_size(cam->preview_width,
				cam->preview_height,
				cam->resizer_width,
				cam->resizer_height);
			cam->pix.width = cam->resizer_width;
			cam->pix.height = cam->resizer_height;
		break;
		case CCDC_RESZ:
			ispccdc_config_size(cam->pix_raw.width,
				cam->pix_raw.height,
				cam->ccdc_width,
				cam->ccdc_height);
			ispresizer_config_size(cam->ccdc_width,
				cam->ccdc_height,
				cam->resizer_width,
				cam->resizer_height);
			cam->pix.width = cam->resizer_width;
			cam->pix.height = cam->resizer_height;
		break;
		case CCDC_PREV:
			ispccdc_config_size(cam->pix_raw.width,
				cam->pix_raw.height,
				cam->ccdc_width,
				cam->ccdc_height);
			isppreview_config_size(cam->ccdc_width,
				cam->ccdc_height,
				cam->preview_width,
				cam->preview_height);
			cam->pix.width = cam->preview_width;
			cam->pix.height = cam->preview_height;
		break;
#endif
	};
	DPRINTK_CAMCFG("- CAM_CFG camcfg_configure_size\n");
}

static void
camcfg_configure_inp_interface(void)
{
	struct isp_interface_config config;
	DPRINTK_CAMCFG("+ CAM_CFG camcfg_configure_interface\n");
	config.ccdc_par_ser = cam->sensor->sensor_interface;
	config.para_clk_pol = 0;
	if((cam->sensor->sensor_type == SENSOR_RAW)){
		config.dataline_shift = 0;
		config.par_bridge = 0;
	}
	if(cam->sensor->sensor_type == SENSOR_ISP){
		config.dataline_shift = 2;
		config.par_bridge = 3;

		/* UYVY RGB565 packed in 8 bits should have bytes swapped */
		if ((cam->pix_inpfmt == PIX_FMT_UYVY) ||
			(cam->pix_inpfmt == PIX_FMT_RGB565))
		;
	/* TBD see if this is needed for Smart RGB data
		config.par_bridge = 2; */
	}
	/* TBD Smart sensor has falling edge, raw sensor has rising edge */
	config.hsvs_syncdetect = 3/*ISPCTRL_SYNC_DETECT_VSRISE*/;
	config.shutter = 0;
	config.prestrobe = 0;
	config.strobe = 0;
	config.vdint0_timing = 0;
	config.vdint1_timing = 0;
	isp_configure_interface(&config);
	DPRINTK_CAMCFG("datalineshift = %d, par_bridge = %d,ccdc_par_ser = %d",
			config.dataline_shift,
			config.par_bridge,
			config.ccdc_par_ser);
	if (cam->pix_outfmt != PIX_FMT_BAYER10_GrRBGb) {
		if (cam->pix_outfmt == PIX_FMT_UYVY) {
#ifdef USE_ISP_PREVIEW
			isppreview_config_ycpos(YCPOS_YCrYCb);
#endif
#ifdef USE_ISP_RESZ
			ispresizer_config_ycpos(0);
#endif
		}
		else {
#ifdef USE_ISP_PREVIEW
			isppreview_config_ycpos(YCPOS_CrYCbY);
#endif
#ifdef USE_ISP_RESZ
			ispresizer_config_ycpos(1);
#endif
		}
	}
	DPRINTK_CAMCFG("- CAM_CFG camcfg_configure_interface\n");
}

/*
 * Used when buffer to be given to the camera hardware is not
 * physically contiguous. This method takes in the linked list
 * of physical address that are discontiguous (sglist -
 * see ispmmu section in isp document for more details) and
 * returns the virtually contiguous address as seen by the ISPMMU
 * The return value of this method is then used in the
 * camcfg_set_curr_buf(buf_addr);
 * sglist	: Linked list of physical addresses that are discontigous.
 * sglen	: Length of the linked list.
 * Return 	: Virtually contiguous address as seen by the camera MMU
 */
u32
camcfg_map_va_buf(const struct scatterlist *sglist, int sglen)
{
	DPRINTK_CAMCFG("CAMCFG sglist[0] = 0x%x\n",sg_dma_address(sglist + 0));
	return (ispmmu_map_sg(sglist,sglen));

}

/*
 * Unmaps the virtual to physical mapping maintained in the Camera MMU.
 * To be called before the buffer is freed.
 * buf_addr	: Camera MMU address to be un mapped
 * Return 	: Error value returned by Camera MMU unmap
 */
/* Check return 0 for success */
u32
camcfg_unmap_va_buf(u32 buf_addr)
{
	return (ispmmu_unmap(buf_addr));
}

void
camcfg_update_capture_params(struct captureparams *parms)
{
	if(cap_parms.cap_shots)
		cam->frame_ctr = 0;
	cap_parms.cap_delay_1stshot = parms->cap_delay_1stshot;
	cap_parms.cap_delay_intershot = parms->cap_delay_intershot;
	cap_parms.cap_shots = parms->cap_shots;
}
void
camcfg_cleanup(void)
{
	DPRINTK_CAMCFG("+ CAM_CFG camcfg_cleanup\n");
	/* Resetting some of the variables for a good start */
	camcfg_out_of_standby();
	ispbuf_use = 0 ;
	isp_actual_tempbuf = 0;
	ispmmu_unmap(cam->ispbuf_mmuaddr);
	isp_free_interface(cam->if_type);
	ispccdc_free();
#ifdef USE_ISP_PREVIEW
	isppreview_free();
#endif

#ifdef USE_ISP_RESZ
	ispresizer_free();
#endif

	camcfg_in_standby();
	if (cam->ispbuf_base) {
		dma_free_coherent(NULL,ISPBUF_SIZE,
			(void *)cam->ispbuf_base, cam->ispbuf_base_phys);
		cam->ispbuf_base = 0;
	}
	cam->ispbuf_base = cam->ispbuf_base_phys = 0;
	DPRINTK_CAMCFG("- CAM_CFG camcfg_cleanup\n");
}

/*
 * Open method for this module.  Does the memory allocs if any is needed
 * Does the inits for the Camcfg structure memebers.  Assumes that the
 * dependent modules ie isp, ispmmu are already available for use.
 */
int
camcfg_init(void)
{
	struct isp_sysc isp_sysconfig;
	DPRINTK_CAMCFG("+ CAM_CFG camcfg_init\n");
	cam = (struct camcfg_device *)kmalloc(
			sizeof(struct camcfg_device),GFP_KERNEL);
	cam->sensor = (struct camera_imager *)kmalloc(
			sizeof(struct camera_imager),GFP_KERNEL);

	camcfg_out_of_standby();
	isp_sysconfig.reset = 0;
	isp_sysconfig.idle_mode = 1;
/*	isp_set_xclka(DEFAULT_SENSOR_XCLK);*/
	isp_power_settings(isp_sysconfig);

	/* we request the needed ISP resources in one place
	 * TBD Need to save power by putting off the clocks
	 */
/*	if (ispccdc_request()) {
		isp_put();
		return -EBUSY;
	}
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
#endif*/

	/* allocate coherent memory for the isp_buf framebuffer */
	cam->ispbuf_base = (unsigned long) dma_alloc_coherent(NULL,
		ISPBUF_SIZE,
		(dma_addr_t *) &cam->ispbuf_base_phys,
		GFP_KERNEL | GFP_DMA);
	if (!cam->ispbuf_base) {
		DPRINTK_CAMCFG("ERROR:\n\n: cannot allocate isp buffer\n");
		goto init_error;
	}
	memset((void *) cam->ispbuf_base, 0, ISPBUF_SIZE);

	DPRINTK_CAMCFG("ispbuf phy addr at 0x%x\n",
			(unsigned int)cam->ispbuf_base_phys);
	/* TBD see where the mapping sits properly */
	//Check for non-zero return value for success
	cam->ispbuf_mmuaddr = ispmmu_map(cam->ispbuf_base_phys, ISPBUF_SIZE);
	DPRINTK_CAMCFG("ispbuf mmuaddr = 0x%x",cam->ispbuf_mmuaddr);
	camcfg_set_curr_buf(cam->ispbuf_mmuaddr);

	spin_lock_init(&cam->lock);
	spin_lock_init(&isc_temp_buf_lock);
	DPRINTK_CAMCFG("- CAM_CFG camcfg_init\n");
	return 0;

init_error:
	camcfg_cleanup();
	return -ENOMEM;
}

/* TBD Fill up this with default camcfg settings needed by
 * H3A, or smart sensor.
 * Does the default camera isp settings if any
 */
int
camcfg_settings(void)
{
	u32 old_xclka;
	int err = 0;
	DPRINTK_CAMCFG("+ CAM_CFG camcfg_settings\n");

	/* TBD Remove all these assumptions when giving
	 * Smart sensor and CSI2 support
	 */

	/* xclk is enabled by the PAL kern.  Check here if
	 * the settings are fine else report a general error
	 */
	old_xclka = isp_get_xclka();
	/* negotiate xclk with isp */
	cam->xclk = isp_negotiate_xclka(old_xclka);
	/* Flag error if not matching with the agreed
	 * new xclk frequency
	 */
	if(cam->xclk != old_xclka){
		err = -1;
		DPRINTK_CAMCFG(" ERR: Clka negotiate error \n");
	}
#if 0
	/* configue ISP for the default image */
	cam->pix_inpfmt = PIX_FMT_BAYER10_GrRBGb;
	cam->pix_outfmt = PIX_FMT_UYVY;
	cam->pix.width = QCIF_WIDTH;
	cam->pix.height = QCIF_HEIGHT;
	cam->pix_raw.width = 512;
	cam->pix_raw.height = 384;
#endif
	DPRINTK_CAMCFG("- CAM_CFG camcfg_settings\n");
init_error:
	return err;
}

void
camcfg_in_standby()
{
	if (cam_started) {
		camcfg_stop_isp();
	}
	if (camcfg_suspended == 0) {
		printk("putting in standby");
		isp_put();
		camcfg_suspended = 1;
	}	
}

void
camcfg_out_of_standby()
{
	if (camcfg_suspended == 1) {
		printk("getting out of stand by\n");
		isp_get();
		camcfg_suspended = 0;		
	}
	if (cam_started) {
		camcfg_start_isp();
	}
}
omap34xxcam_set_exposure_time(){}
omap34xxcam_set_gain(){}
omap34xxcam_sensor_restore() {}
/* TBD Make CAMCFG a module with module_init(),module_exit()
 *
 * module_init(camcfg_init);
 * module_exit(camcfg_exit);
 */

