/*
 * drivers/media/video/omap/gci/cam_config.h
 *
 * Header file for Camera input port 
 * configurations for TI's OMAP3430 Camera ISP
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

#ifndef CAM_CONFIG_H
#define CAM_CONFIG_H

#include <asm/scatterlist.h>
#include "../isp/isp.h"
#include "../gci/isc.h"

#define OMAP_ISP_CCDC		(1 << 0)
#define OMAP_ISP_PREVIEW	(1 << 1)
#define OMAP_ISP_RESIZER	(1 << 2)

#define ISP_INIT 0
#define ISP_FREE_RUNNING 1
#define ISP_TRAN 2

/* Enumeration for camera input and output data format */
enum cam_fmt{
	PIX_FMT_UYVY,
	PIX_FMT_YVYU,
	PIX_FMT_VYUY,
	PIX_FMT_YUYV,
	PIX_FMT_BAYER10_GrRBGb,
	PIX_FMT_RGB565
};

/* Enumeration for Modules involved in the pipeline */
enum pipeline{
	CCDC_PREV_RESZ = OMAP_ISP_CCDC |OMAP_ISP_PREVIEW |
			OMAP_ISP_RESIZER,
	CCDC_PREV = OMAP_ISP_CCDC |OMAP_ISP_PREVIEW,
	CCDC_RESZ = OMAP_ISP_CCDC |OMAP_ISP_RESIZER,
	CCDC_ALONE = OMAP_ISP_CCDC
};

/* Sturcture for frame width and height */
struct frame_size {
	u32 width;
	u32 height;
};

/* Structure for specifying the crop parameters */
/* TBD not used right now, see if it is useful in CCDC crop */
struct rect{
	u8 left;
	u8 top;
	u32 width;
	u32 height;
};

/* Defining the same callback structure as Camera buffer management module */
struct callback_arg{
	dma_addr_t isp_addr;	/* ISP space addr */ 
	unsigned long status;	/* DMA return code */
	isp_callback_t callback;
	void *arg1;
	void *arg2;
};

/* Determing the sensor type is raw or smart.
 * TBD In future try not to have this.
 */
/* TBD remove Senthil currently here to avoid compilation */
enum imager_type{
	SENSOR_RAW,
	SENSOR_ISP
};

/* Determing the input interface for Camera ISP 
 * At present only PAR is supported
 */
enum imager_if{
	SENSOR_PARALLEL,
	SENSOR_SERIAL1,
	SENSOR_SERIAL2
};

/* Structure to get the sensor interface info */
struct camera_imager{
	enum imager_type sensor_type;
	enum imager_if sensor_interface;
};

/* per-device data structure */
struct camcfg_device {
	/* Address for ispbuffer owned by camera config */
	unsigned long ispbuf_base; /* Virtual addr */
	unsigned long ispbuf_base_phys; /* Physical addr */
	unsigned long ispbuf_mmuaddr; /* ISPMMU mapped addr */
	
	enum isp_interface_type if_type;
	
	/* bit masks of pipeline usage */
	u8 isp_pipeline;
	
	enum cam_fmt pix_outfmt;
	enum cam_fmt pix_inpfmt;
	
	/* TBD Remove 
	 * Not used as of now since pix, pix_raw takes care 
	 */
	u32 cam_inpwidth;
	u32 cam_inpheight;
	u32 cam_outwidth;
	u32 cam_outheight;

	/* pointer to camera sensor interface interface */
	struct camera_imager *sensor;
	
	/* Prepared ISP output sizes for video mode */
	unsigned long ccdc_input_width;
	unsigned long ccdc_input_height;
	unsigned long ccdc_width;
	unsigned long ccdc_height;
	unsigned long preview_width;
	unsigned long preview_height;
	unsigned long resizer_width;
	unsigned long resizer_height;
	
	/* Callback to be called from the ISP CTRL isr */
	struct callback_arg cbk;
	
	struct frame_size pix, pix_raw;

	u8 signal_buf_filled;
	u32 frame_ctr;

	/* Used for irq save restore */
	spinlock_t lock;
	/* TBD Use this to see if the xclk set by isc_regtrans
	 * is okay or not 
	 */
	unsigned int xclk;
};

/*****************************************************************/
/* APIs to be called from Camera buffer managment */
/*****************************************************************/

/*
 *Used when buffer to be given to the camera hardware is not 
 * physically contiguous. This method takes in the linked list 
 * of physical address that are discontiguous (sglist - 
 * see ispmmu section in isp document for more details) and 
 * returns the virtually contiguous address as seen by the ISPMMU
 * The return value of this method is then used in the
 * camcfg_set_curr_buf(buf_addr);
 */
u32 camcfg_map_va_buf(const struct scatterlist *sglist, int sglen);

/*
 * Unmaps the virtual to physical mapping maintained in the Camera MMU. 
 * To be called before the buffer is freed.
 */
u32 camcfg_unmap_va_buf(u32 buf_addr);

/*
 * This method Updates the buffer address for the camera hardware
 * to fillup the data. This is called with the value returned by
 * the camcfg_map_va_buf();
 */
void camcfg_set_curr_buf(u32 buf_addr);

/*
 * Sets the callback to notify that the buffer given to the 
 * camera hardware is filled up and ready to consume.
 */
void camcfg_set_callback(struct callback_arg *cbkarg);

/*
 * The callback is made NULL, so that the Camera config callback does not 
 * call this callback.  Also indicates the camera config to use its
 * own isp buffer since no user to consume the camera buffer
 */
int camcfg_unset_callback(void);

/*
 * The method indicates to the camcfg when to use its own buffer
 * and when to transition from its own buffer to the queued buffer
 */
int camcfg_useispbuf(int state);

/*****************************************************************/
/*APIs to be called from ISP state controller */
/*****************************************************************/

/* Prepares the input/output sizes of the isp modules for the given 
 * input size(output size of the sensor) and output size (output size 
 * of the Camera driver). It chooses the isp modules to be considered 
 * based on the pipeline. The prepared sizes are placed in the void*ptr 
 * after typecasting to struct isc_sizes
 */
void camcfg_prepare_size(struct frame_size *outsize, 
			struct frame_size *inpsize,void* ptr);

/*
 * Configures the isp registers with the input output sizes
 * for each module in the pipeline.
 */
void camcfg_configure_size(struct isc_sizes *cfgsize);

/* TBD This method should take care of isc_regtrans(native programming) */
void camcfg_configure_interface(void);

/* The isp modules are configured for their data output format for each e
 * engine in the pipeline.
 * Along with the input/output data format of the modules, default 
 * configuration of each of the modules are done 
 * except for input/output sizes.
 */
void camcfg_configure_pixformat(void);

/*
 * Prepares the input pixelformat (output format of the sensor) and 
 * the output pixelformat (output format of the camera driver)
 */
void camcfg_prepare_pixformat(enum cam_fmt pix_outfmt, void* pix_inpfmt);

/*
 * Based on the input pixelformat (output format of the sensor) and 
 * the output pixelformat (output format of the camera driver) obtained
 * from camcfg_configure_pixformat the camera pipeline is set.
 */
void camcfg_set_pipeline();

/*
 * Start the Camera ie each modules in the camera pipeline to give frames
 * Also sets the camcfg callback to the ISP ctrl ISR.
 */
void camcfg_start(void);

/* 
 * Stop giving frames by stopping all the modules in the pipeline.
 * Unsets the camcfg callback from ISP ctrl isr.
 */
int camcfg_stop(void);

/*
 * Updates the capture parameters that dictates when to call the 
 * callback to indicated that the queued buffer is filled up
 */
void camcfg_update_capture_params(struct captureparams *cap_parms);


int camcfg_settings(void);

/* TBD once CAMCFG is a module then remove this */
int camcfg_init(void);
void camcfg_cleanup(void);


#endif	/* ifndef CAM_CONFIG_H */
