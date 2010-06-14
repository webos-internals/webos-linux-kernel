/*
 * drivers/media/video/ispresizer.c
 *
 * Driver Library for Resizer module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C)2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Resizer module for ISP driver on OMAP3430. It implements
 * the Resizer module APIs defined in ispresizer.h.
 */

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/module.h>
#include <asm/semaphore.h>

#include "isp.h"
#include "ispreg.h"
#include "ispresizer.h"

/*
 * Resizer Constants
 */
#define MAX_IN_WIDTH_MEMORY_MODE 4095

#define MAX_IN_WIDTH_ONTHEFLY_MODE 1280
#define MAX_IN_WIDTH_ONTHEFLY_MODE_ES2 4095
#define MAX_IN_HEIGHT 4095
#define MINIMUM_RESIZE_VALUE 64
#define MAXIMUM_RESIZE_VALUE 1024
#define MID_RESIZE_VALUE 512

#define MAX_7TAP_HRSZ_OUTWIDTH 1280
#define MAX_7TAP_VRSZ_OUTWIDTH 640

#define MAX_7TAP_HRSZ_OUTWIDTH_ES2 3300
#define MAX_7TAP_VRSZ_OUTWIDTH_ES2 1650

#define DEFAULTSTPIXEL 0
#define DEFAULTSTPHASE 1
#define DEFAULTHSTPIXEL4TAPMODE 3
#define FOURPHASE 4
#define EIGHTPHASE 8
#define RESIZECONSTANT 256
#define SHIFTER4TAPMODE 0
#define SHIFTER7TAPMODE 1
#define DEFAULTOFFSET 7
#define OFFSETVERT4TAPMODE 4
#define OPWDALIGNCONSTANT 0xFFFFFFF0

/* Default configuration of resizer,filter coefficients,yenh for camera isp*/
static struct isprsz_yenh ispreszdefaultyenh = {0, 0, 0, 0};
static	struct isprsz_coef ispreszdefcoef = {
{
			0x0000, 0x0100, 0x0000, 0x0000,
			0x03FA, 0x00F6, 0x0010, 0x0000,
			0x03F9, 0x00DB, 0x002C, 0x0000,
			0x03FB, 0x00B3, 0x0053, 0x03FF,
			0x03FD, 0x0082, 0x0084, 0x03FD,
			0x03FF, 0x0053, 0x00B3, 0x03FB,
			0x0000, 0x002C, 0x00DB, 0x03F9,
			0x0000, 0x0010, 0x00F6, 0x03FA
			},
			{
			0x0000, 0x0100, 0x0000, 0x0000,
			0x03FA, 0x00F6, 0x0010, 0x0000,
			0x03F9, 0x00DB, 0x002C, 0x0000,
			0x03FB, 0x00B3, 0x0053, 0x03FF,
			0x03FD, 0x0082, 0x0084, 0x03FD,
			0x03FF, 0x0053, 0x00B3, 0x03FB,
			0x0000, 0x002C, 0x00DB, 0x03F9,
			0x0000, 0x0010, 0x00F6, 0x03FA
			},
			{  
			0x0004, 0x0023, 0x005A, 0x0058,
			0x0023, 0x0004, 0x0000, 0x0002,
			0x0018, 0x004d, 0x0060, 0x0031,
			0x0008, 0x0000, 0x0001, 0x000f,
			0x003f, 0x0062, 0x003f, 0x000f,
			0x0001, 0x0000, 0x0008, 0x0031,
			0x0060, 0x004d, 0x0018, 0x0002
			},
			{
			0x0004, 0x0023, 0x005A, 0x0058,
			0x0023, 0x0004, 0x0000, 0x0002,
			0x0018, 0x004d, 0x0060, 0x0031,
			0x0008, 0x0000, 0x0001, 0x000f,
			0x003f, 0x0062, 0x003f, 0x000f,
			0x0001, 0x0000, 0x0008, 0x0031,
			0x0060, 0x004d, 0x0018, 0x0002
			}
	};

/*
 * Structure for the resizer module to store its own information.
 */
static struct isp_res {
	u8 res_inuse;
	u8 h_startphase;
	u8 v_startphase;
	u16 h_resz;
	u16 v_resz;
	u32 outputwidth;
	u32 outputheight;
	u32 inputwidth;
	u32 inputheight;
	u8 algo;
	u32 ipht_crop;
	u32 ipwd_crop;
        u32 cropwidth;
        u32 cropheight;
	enum ispresizer_input resinput;
	struct isprsz_coef coeflist;
	struct semaphore semlock;
}ispres_obj;

/* Structure for saving/restoring resizer module registers*/
static struct isp_reg isprsz_reg_list[]= {
	{ISPRSZ_CNT, 0x0000},
	{ISPRSZ_OUT_SIZE, 0x0000},
	{ISPRSZ_IN_START, 0x0000},
	{ISPRSZ_IN_SIZE, 0x0000},
	{ISPRSZ_SDR_INADD, 0x0000},
	{ISPRSZ_SDR_INOFF, 0x0000},
	{ISPRSZ_SDR_OUTADD, 0x0000},
	{ISPRSZ_SDR_OUTOFF, 0x0000},
	{ISPRSZ_HFILT10, 0x0000},
	{ISPRSZ_HFILT32, 0x0000},
	{ISPRSZ_HFILT54, 0x0000},
	{ISPRSZ_HFILT76, 0x0000},
	{ISPRSZ_HFILT98, 0x0000},
	{ISPRSZ_HFILT1110, 0x0000},
	{ISPRSZ_HFILT1312, 0x0000},
	{ISPRSZ_HFILT1514, 0x0000},
	{ISPRSZ_HFILT1716, 0x0000},
	{ISPRSZ_HFILT1918, 0x0000},
	{ISPRSZ_HFILT2120, 0x0000},
	{ISPRSZ_HFILT2322, 0x0000},
	{ISPRSZ_HFILT2524, 0x0000},
	{ISPRSZ_HFILT2726, 0x0000},
	{ISPRSZ_HFILT2928, 0x0000},
	{ISPRSZ_HFILT3130, 0x0000},
	{ISPRSZ_VFILT10, 0x0000},
	{ISPRSZ_VFILT32, 0x0000},
	{ISPRSZ_VFILT54, 0x0000},
	{ISPRSZ_VFILT76, 0x0000},
	{ISPRSZ_VFILT98, 0x0000},
	{ISPRSZ_VFILT1110, 0x0000},
	{ISPRSZ_VFILT1312, 0x0000},
	{ISPRSZ_VFILT1514, 0x0000},
	{ISPRSZ_VFILT1716, 0x0000},
	{ISPRSZ_VFILT1918, 0x0000},
	{ISPRSZ_VFILT2120, 0x0000},
	{ISPRSZ_VFILT2322, 0x0000},
	{ISPRSZ_VFILT2524, 0x0000},
	{ISPRSZ_VFILT2726, 0x0000},
	{ISPRSZ_VFILT2928, 0x0000},
	{ISPRSZ_VFILT3130, 0x0000},
	{ISPRSZ_YENH, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};



void
ispresizer_config_shadow_registers()
{
	// printk(" RESIZER DONE\n");
		return;
}

void
ispresizer_trycrop(u32 left, u32 top, u32 width, u32 height, u32 ow, u32 oh)
{
  ispres_obj.cropwidth  = width + 6;
  ispres_obj.cropheight = height + 6;
  ispresizer_try_size(&ispres_obj.cropwidth, &ispres_obj.cropheight, &ow, &oh);
  ispres_obj.ipht_crop = top;
  ispres_obj.ipwd_crop = left;
}

void
ispresizer_applycrop()
{
  ispresizer_config_size(ispres_obj.cropwidth, ispres_obj.cropheight, ispres_obj.outputwidth, ispres_obj.outputheight);
		return;
}


/*
 * Reserve the Resizer module.
 * Only one user at a time.
 */
int
ispresizer_request()
{
	down(&(ispres_obj.semlock));
	if (!(ispres_obj.res_inuse)){
		ispres_obj.res_inuse = 1;
		up(&(ispres_obj.semlock));
		/* Turn on Resizer module Clocks.*/
		omap_writel(omap_readl(ISP_CTRL) | ISPCTRL_SBL_WR0_RAM_EN |
				ISPCTRL_RSZ_CLK_EN, ISP_CTRL);
 		return 0;
	}
	else{
		up(&(ispres_obj.semlock));
		printk(KERN_ERR "ISP_ERR : Resizer Module Busy\n");
		return -EBUSY;
	}
}

/*
 * Makes Resizer module free.
 */
int
ispresizer_free()
{
	down(&(ispres_obj.semlock));
	if (ispres_obj.res_inuse){
		ispres_obj.res_inuse = 0;
		up(&(ispres_obj.semlock));
		omap_writel(omap_readl(ISP_CTRL)& ~(ISPCTRL_RSZ_CLK_EN |
				ISPCTRL_SBL_WR0_RAM_EN), ISP_CTRL);
		return 0;
	}
	else{
		up(&(ispres_obj.semlock));
		DPRINTK_ISPRESZ("ISP_ERR : Resizer Module already freed\n");
		return -EINVAL;
	}
}

/*
 * Sets up the default resizer configuration according to the arguments.
 * input		: Indicates the module that gives the image to resizer
 */
int
ispresizer_config_datapath(enum ispresizer_input input)
{
	u32 cnt = 0;
	DPRINTK_ISPRESZ("ispresizer_config_datapath()+\n");
	ispres_obj.resinput = input;
	switch (input){
	case RSZ_OTFLY_YUV:
		cnt &= ~ISPRSZ_CNT_INPTYP;
		cnt &= ~ISPRSZ_CNT_INPSRC;
		/* according to TRM, inline address and inline offset must be
		 * set to 0 for OTF input mode
		 */
		ispresizer_set_inaddr(0);
		ispresizer_config_inlineoffset(0);
		break;
	case RSZ_MEM_YUV:
		cnt |= ISPRSZ_CNT_INPSRC;
		cnt &= ~ISPRSZ_CNT_INPTYP;
		break;
	case RSZ_MEM_COL8:
		cnt |= ISPRSZ_CNT_INPSRC;
		cnt |= ISPRSZ_CNT_INPTYP;
		break;
	default:
		printk(KERN_ERR "ISP_ERR : Wrong Input\n");
		return -EINVAL;
	}
	omap_writel(omap_readl(ISPRSZ_CNT) | cnt, ISPRSZ_CNT);
	/*Set up  default parameters
	 */
	ispresizer_config_ycpos(0);
	ispresizer_config_filter_coef(&ispreszdefcoef);
	ispresizer_enable_cbilin(0);
	ispresizer_config_luma_enhance(&ispreszdefaultyenh);
	DPRINTK_ISPRESZ("ispresizer_config_datapath()-\n");
	return 0;
}

#if 0 
/* 
 * Maintained here inorder to reference earlier implementation.
 */
static void
trysize_calculation(u32 inputsize, u32 outputsize,u8 horzdir)
{
	u32 org_input,input,output,rsz;
	u8 shifter = SHIFTER4TAPMODE;
	u8 offset = DEFAULTOFFSET;
	u8 stPh, numPhases, endPhase;
	int stPix,phase;
	long fip, cip;
	DPRINTK_ISPRESZ("trysize_calculation()+\n");
	/*Calculate crop,start phase,resize ratio and input size */
	org_input = inputsize;
	input = inputsize;
	output = outputsize;
	/*
	 * Adjust the input to resizer.Also assign default values for vertical start
	 * pixel and vertical crop according to the resize ratio.
	 */
	input -= 6;
	rsz = input * RESIZECONSTANT / output;
	/*
	 * if resize ratio is less than 64,the requested scale up is
	 * not supported by the resizer.So calculate the maximum
	 * output width/height possible with resize ratio 64 and pass it
	 * back to the requester
	 */
	if (rsz < MINIMUM_RESIZE_VALUE){
		rsz = MINIMUM_RESIZE_VALUE;
		output = input * RESIZECONSTANT / rsz;
		if (horzdir){
			output &= OPWDALIGNCONSTANT;
			DPRINTK_ISPRESZ("Required output width is not\
				achievable.The achievable width is %d",
				output);
		}
		else
			DPRINTK_ISPRESZ("Required output height is not\
				achievable.The achievable height is %d",
				output);
	}
	/*
	 * if resize ratio is greater than 1024,the requested scale
	 * down is not supported by the resizer.So calculate the
	 * maximum output width/height possible with resize ratio 1024 and
	 * pass it back to the requester.
	 */
	if (rsz > MAXIMUM_RESIZE_VALUE){
		rsz = MAXIMUM_RESIZE_VALUE;
		output = input * RESIZECONSTANT / rsz;
		if (horzdir){
			output &= OPWDALIGNCONSTANT;
			DPRINTK_ISPRESZ("Required output width is not\
				achievable.The achievable width is %d",
				output);
		}
		else
			DPRINTK_ISPRESZ("Required output height is not\
				achievable.The achievable height is %d",
				output);
	}
	if(rsz <= MID_RESIZE_VALUE){
		if(!horzdir){
			input +=2;
			stPix = DEFAULTSTPIXEL;
			stPh = DEFAULTSTPHASE;
		}
		else{
			input -=4;
			stPix = DEFAULTHSTPIXEL4TAPMODE;
			stPh = DEFAULTSTPHASE;
		}
		rsz = input * RESIZECONSTANT / output;
		if(rsz > MID_RESIZE_VALUE){
			input -=2;
			rsz = MID_RESIZE_VALUE;
		}
	}
	else{
		stPix = DEFAULTSTPIXEL;
		stPh = DEFAULTSTPHASE;
	}	
	/* Adjust the phase and start pixel*/
	numPhases = (rsz>MID_RESIZE_VALUE) ? FOURPHASE : EIGHTPHASE;
	phase = stPh;
	stPix += phase / numPhases;
	phase %= numPhases;
	/*
	 * If rsz > 512 use the equation for 4-phase 7-tap mode.For that set
	 * shifter to be 1.else use equation for 8-phase 4-tap mode.In case
	 * of calculating vetical input for 8-phase 4-tap mode set the offset
	 * to be 4.In all other cases it is 7.
	 */
	if(rsz > MID_RESIZE_VALUE)
		shifter = SHIFTER7TAPMODE;
	else if(!horzdir)
		offset = OFFSETVERT4TAPMODE;
	/*
	 * Calculate the input and the endphase.The calculations finally boil
	 * down to the 4-tap and 7-tap equations stated in the beginning of
	 * function ispresizer_try_size.The two different equation are
	 * automatically chosen depending on the value of shifter and
	 * numphases.Endphase is used later on to calculate the start phase.
	 */
	{
		fip = (phase<<(5+shifter)) + (16<<shifter) + ((output-1)*rsz);
		cip = fip >> (5+shifter);
		input = cip/numPhases + offset;
		endPhase = cip % numPhases;
	}
	phase += ((org_input - input)*numPhases - endPhase) >> 1;
	if(phase < 0){
		stPix += (phase+1) / numPhases - 1;
		phase = (phase%numPhases) + numPhases;
	}
	else{
		stPix += phase / numPhases;
		phase %= numPhases;
	}
	/* Recalculating input size since the phase may have changed.*/
	input = (((phase<<(5+shifter)) + (16<<shifter) + ((output-1)*rsz))>>8)
		+ offset;
	if(horzdir){
		ispres_obj.outputwidth = output;
		ispres_obj.h_resz = rsz;
		ispres_obj.inputwidth = input;
		ispres_obj.ipwd_crop = stPix;
		ispres_obj.h_startphase = phase;
	}
	else{
		ispres_obj.outputheight = output;
		ispres_obj.inputheight = input;
		ispres_obj.ipht_crop = stPix;
		ispres_obj.v_resz = rsz;
		ispres_obj.v_startphase = phase;
	}
	DPRINTK_ISPRESZ("trysize_calculation()-\n");
}
#endif

/*
 * Calculates the horizontal and vertical resize ratio,number of pixels to
 * be cropped in the resizer module and checks the validity of various
 * parameters.This function internally calls trysize_calculation,which does
 * the actual calculations and populates required members of isp_res struct
 * Formula used for calculation is:-
 * 8-phase 4-tap mode :-
 * inputwidth = (32*sph + (ow - 1)*hrsz + 16) >> 8 + 7
 * inputheight = (32*spv + (oh - 1)*vrsz + 16) >> 8 + 4
 * endpahse for width = ( ( 32*sph + (ow - 1)*hrsz +16 ) >> 5 )% 8
 * endphase for height = ( ( 32*sph + (oh - 1)*hrsz +16 ) >> 5 )% 8
 * 4-phase 7-tap mode :-
 * inputwidth = (64*sph + (ow - 1)*hrsz + 32) >> 8 + 7
 * inputheight = (64*spv + (oh - 1)*vrsz + 32) >> 8 + 7
 * endpahse for width = ( ( 64*sph + (ow - 1)*hrsz +32 ) >> 6 )% 4
 * endphase for height = ( ( 64*sph + (oh - 1)*hrsz +32 ) >> 6 )% 4
 * where
 * sph = Start phase horizontal
 * spv = Start phase vertical
 * ow = Output width
 * oh = Output height
 * hrsz = Horizontal resize value
 * vrsz = Vertical resize value
 * Fills up the output/input widht/height,horizontal/vertical resize ratio,
 * horizontal/vertical crop variables in the isp_res structure .
 * input_w	: input width for the resizer in number of pixels per line
 * input_h	: input height for the resizer in number of lines
 * output_w	: output width from the resizer in number of pixels per line
 *                resizer when writing to memory needs this to be multiple of 16
 * output_h	: output height for the resizer in number of lines, must be even
*/
int
ispresizer_try_size(u32 *input_width, u32 *input_height, u32 *output_w, u32 *output_h)
{
	u32 rsz,rsz_7, rsz_4;
	u32 sph;
	u32 input_w, input_h;
	u32 output;
	int max_in_otf, max_out_7tap;
	input_w = *input_width;
	input_h = *input_height;

	/*
	 * This has to be done inorder to make sure that the try size does not 
	 * end up with input height/width greater than what the preview will output.
	 */
	input_w = input_w - 6;
	input_h = input_h - 6;
	
	if (input_h > MAX_IN_HEIGHT)
		return -EINVAL;

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
		max_in_otf = MAX_IN_WIDTH_ONTHEFLY_MODE;
		max_out_7tap = MAX_7TAP_VRSZ_OUTWIDTH;
	} else {
		max_in_otf = MAX_IN_WIDTH_ONTHEFLY_MODE_ES2;
		max_out_7tap = MAX_7TAP_VRSZ_OUTWIDTH_ES2;
	}

	if (ispres_obj.resinput == RSZ_OTFLY_YUV) {
		if (input_w > max_in_otf)
			return -EINVAL;
	} else {
		if (input_w > MAX_IN_WIDTH_MEMORY_MODE)
			return -EINVAL;
	}


	*(output_h) = *(output_h) & 0xFFFFFFFE;
	output = *(output_h);
	sph = DEFAULTSTPHASE;

	/* For height */
	rsz_7 = ((input_h - 7) * 256) / (output - 1);
	rsz_4 = ((input_h - 4) * 256) / (output - 1);

	rsz = (input_h * 256) / output;

	if (rsz <= MID_RESIZE_VALUE) {
		rsz = rsz_4;
		if (rsz < MINIMUM_RESIZE_VALUE) {
			rsz = MINIMUM_RESIZE_VALUE;
			output = (((input_h - 4) * 256) / rsz) + 1;
			printk("\t ISP_ERR: rsz was less than min - new op_h is %d\n", output);
		}
	} else {
		rsz = rsz_7;
		if (*(output_w) > max_out_7tap)
			*(output_w) =  max_out_7tap;
		if (rsz > MAXIMUM_RESIZE_VALUE) {
	    		rsz = MAXIMUM_RESIZE_VALUE;	
			output = (((input_h - 7) * 256) / rsz) + 1;
			printk("\t ISP_ERR: rsz was more than max - new op_h is %d\n", output);
		}
	}

	/* Recalculate input */
	if (rsz > MID_RESIZE_VALUE) {
		input_h = (((64 * sph) + ((output - 1)* rsz) + 32) / 256) + 7;
	}else {
		input_h = (((32 * sph) + ((output - 1)* rsz) + 16) / 256) + 4;
	}

	ispres_obj.outputheight = output;
	ispres_obj.v_resz = rsz;
	ispres_obj.inputheight = input_h;
	ispres_obj.ipht_crop = DEFAULTSTPIXEL;
	ispres_obj.v_startphase = sph;


	*(output_w) = *(output_w) & 0xFFFFFFF0;
	output = *(output_w);
	sph = DEFAULTSTPHASE;

	/* For Width */
	rsz_7 = ((input_w - 7) * 256) / (output - 1);
	rsz_4 = ((input_w - 4) * 256) / (output - 1);

	rsz = (input_w * 256) / output;
	if (rsz > MID_RESIZE_VALUE){
		rsz = rsz_7;
		if (rsz > MAXIMUM_RESIZE_VALUE) {
			rsz = MAXIMUM_RESIZE_VALUE;	
			output = (((input_w - 7) * 256) / rsz) + 1;
			printk("\t ISP_ERR: rsz was greater than max - new op_w is %d\n", output);
		}
	} else {
		rsz = rsz_4;
		if (rsz < MINIMUM_RESIZE_VALUE) {
			rsz = MINIMUM_RESIZE_VALUE;
			output = (((input_w - 4) * 256) / rsz) + 1;
			printk("\t ISP_ERR: rsz was less than min - new op_w is %d\n", output);
		}
	}
	
	/* Recalculate input based on TRM equations */
	if (rsz > MID_RESIZE_VALUE)
		{
		input_w = (((64 * sph) + ((output - 1) * rsz) + 32) / 256) + 7;
		}
	else
		{
		input_w = (((32 * sph) + ((output - 1) * rsz) + 16) / 256) + 7;
		}

	ispres_obj.outputwidth = output;
 	ispres_obj.h_resz = rsz;
	ispres_obj.inputwidth = input_w;
	ispres_obj.ipwd_crop = DEFAULTSTPIXEL;
	ispres_obj.h_startphase = sph;

	*input_height = input_h;
	*input_width  = input_w;
	return 0;
}

/*
 * Configures the appropriate values stored in the isp_res structure in
 * the resizer registers
 * input_w	: input width for the resizer in number of pixels per line
 * input_h	: input height for the resizer in number of lines
 * output_w	: output width from the resizer in number of pixels per line
 * output_h	: output height for the resizer in number of lines
 */
int
ispresizer_config_size(u32 input_w, u32 input_h, u32 output_w, u32 output_h)
{
	int i,j;
	u32 res;
	DPRINTK_ISPRESZ("ispresizer_config_size()+, input_w=%d,input_h=%d,\
			output_w=%d, output_h=%d,hresz=%d,vresz=%d,\
			hcrop=%d,vcrop=%d,hstph=%d,vstph=%d\n",
			ispres_obj.inputwidth, ispres_obj.inputheight, 
			ispres_obj.outputwidth, ispres_obj.outputheight,
			ispres_obj.h_resz,ispres_obj.v_resz,
			ispres_obj.ipwd_crop,ispres_obj.ipht_crop,
			ispres_obj.h_startphase,ispres_obj.v_startphase);
	if ((output_w != ispres_obj.outputwidth)
			|| (output_h != ispres_obj.outputheight)){
		printk(KERN_ERR "Output parameters passed do not match\
				the values calculated by the trysize passed w %d, h %d\n",output_w ,output_h);
		return -EINVAL;
		}
	/* Set horizontal and vertical starting phase */
	res =omap_readl(ISPRSZ_CNT) & (~ (ISPRSZ_CNT_HSTPH_MASK |
					ISPRSZ_CNT_VSTPH_MASK));
	omap_writel(res | (ispres_obj.h_startphase << ISPRSZ_CNT_HSTPH_SHIFT)
			| (ispres_obj.v_startphase << ISPRSZ_CNT_VSTPH_SHIFT)
			,ISPRSZ_CNT);
	/* Set horizontal and vertical start pixel */
	omap_writel(((ispres_obj.ipwd_crop * 2) << ISPRSZ_IN_START_HORZ_ST_SHIFT) |
		(ispres_obj.ipht_crop << ISPRSZ_IN_START_VERT_ST_SHIFT),
		ISPRSZ_IN_START);


	/*Set input width and height*/
	omap_writel((ispres_obj.inputwidth << ISPRSZ_IN_SIZE_HORZ_SHIFT) |
		(ispres_obj.inputheight << ISPRSZ_IN_SIZE_VERT_SHIFT), ISPRSZ_IN_SIZE);
	/*Set output width and height*/
	if (! ispres_obj.algo)
		omap_writel((output_w << ISPRSZ_OUT_SIZE_HORZ_SHIFT) |
			(output_h << ISPRSZ_OUT_SIZE_VERT_SHIFT),
			ISPRSZ_OUT_SIZE);
	else
		omap_writel(((output_w - 4 ) << ISPRSZ_OUT_SIZE_HORZ_SHIFT) |
			(output_h << ISPRSZ_OUT_SIZE_VERT_SHIFT),
			ISPRSZ_OUT_SIZE);


	/*Set horizontal and vertical resize ratios*/
	res = omap_readl(ISPRSZ_CNT) & (~ (ISPRSZ_CNT_HRSZ_MASK |
					ISPRSZ_CNT_VRSZ_MASK));
	omap_writel(res | ((ispres_obj.h_resz - 1)  << ISPRSZ_CNT_HRSZ_SHIFT)
			| ((ispres_obj.v_resz - 1) << ISPRSZ_CNT_VRSZ_SHIFT)
			, ISPRSZ_CNT);
	/*Set the horizontal/vertical filter coefficients depending on the
	 * resize values
	 */
	if (ispres_obj.h_resz <= MID_RESIZE_VALUE){
		j = 0;
		for (i=0;i<16;i++){
			omap_writel((ispres_obj.coeflist.
			h_filter_coef_4tap[j] <<
			ISPRSZ_HFILT10_COEF0_SHIFT) |
			(ispres_obj.coeflist.h_filter_coef_4tap[j+1]
			<< ISPRSZ_HFILT10_COEF1_SHIFT),
			ISPRSZ_HFILT10 + ( i * 0x04 ));

		}
	}
	else {
		j = 0;
		for (i=0;i<16;i++){
			if ((i + 1) % 4 == 0){
				omap_writel((ispres_obj.coeflist.
				h_filter_coef_7tap[j] <<
				ISPRSZ_HFILT10_COEF0_SHIFT) ,
				ISPRSZ_HFILT10 + ( i * 0x04 ));
				j+=1;
			}
			else{
				omap_writel((ispres_obj.coeflist.
				h_filter_coef_7tap[j] <<
				ISPRSZ_HFILT10_COEF0_SHIFT) |
				(ispres_obj.coeflist.
				h_filter_coef_7tap[j+1] <<
				ISPRSZ_HFILT10_COEF1_SHIFT),
				ISPRSZ_HFILT10 + ( i * 0x04 ));
				j+=2;
			}
		}
	}
	if (ispres_obj.v_resz <= MID_RESIZE_VALUE){
		j = 0;
		for (i=0;i<16;i++){
			omap_writel((ispres_obj.coeflist.
			v_filter_coef_4tap[j] <<
			ISPRSZ_VFILT10_COEF0_SHIFT) |
			(ispres_obj.coeflist.v_filter_coef_4tap[j+1]
			<< ISPRSZ_VFILT10_COEF1_SHIFT),
			ISPRSZ_VFILT10 + ( i * 0x04 ));
			j+=2;
		}
	}
	else {
		j = 0;
		for (i=0;i<16;i++){
			if ((i + 1) % 4 == 0){
				omap_writel((ispres_obj.coeflist.
				v_filter_coef_7tap[j] <<
				ISPRSZ_VFILT10_COEF0_SHIFT) ,
				ISPRSZ_VFILT10 + ( i * 0x04 ));
				j+=1;
			}
			else{
				omap_writel((ispres_obj.coeflist.
				v_filter_coef_7tap[j] <<
				ISPRSZ_VFILT10_COEF0_SHIFT) |
				(ispres_obj.coeflist.
				v_filter_coef_7tap[j+1] <<
				ISPRSZ_VFILT10_COEF1_SHIFT),
				ISPRSZ_VFILT10 + ( i * 0x04 ));
				j+=2;
			}
		}
	}	
		
	/* Configure the outline offset to e outputwidth*2*/
	ispresizer_config_outlineoffset(output_w*2);
	DPRINTK_ISPRESZ("ispresizer_config_size()-\n");
	return 0;
}

/*
 * Enables the Resizer module.
 * Client should configure all the sub modules in Resizer before this.
 * enable	: 1- Enables the resizer module.
 */
void
ispresizer_enable(u8 enable)
{
	DPRINTK_ISPRESZ("+ispresizer_enable()+\n");
	if(enable)
		omap_writel((omap_readl(ISPRSZ_PCR)) |
				ISPRSZ_PCR_ENABLE, ISPRSZ_PCR);
	else {
		omap_writel((omap_readl(ISPRSZ_PCR)) &
				~ISPRSZ_PCR_ENABLE, ISPRSZ_PCR);
	}
	DPRINTK_ISPRESZ("+ispresizer_enable()-\n");
}

int
ispresizer_busy(void)
{
	return (omap_readl(ISPRSZ_PCR) & ISPPRV_PCR_BUSY);
}

/*
 * Sets the horizontal and vertical start phase.
 * This API just updates the isp_res struct.Actual register write happens in
 * ispresizer_config_size.
 * hstartphase	: horizontal start phase(0-7)
 * vstartphase	: vertical startphase(0-7)
 */
void
ispresizer_config_startphase(u8 hstartphase, u8 vstartphase)
{
	DPRINTK_ISPRESZ("ispresizer_config_startphase()+\n");
	ispres_obj.h_startphase = hstartphase;
	ispres_obj.v_startphase = vstartphase;
	DPRINTK_ISPRESZ("ispresizer_config_startphase()-\n");
}

/*
 * Sets whether the output should be in YC or CY format.
 * yc	:0 - YC format
 * 	 1 - CY format
 */
void
ispresizer_config_ycpos(u8 yc)
{
	DPRINTK_ISPRESZ("ispresizer_config_ycpos()+\n");
	if (yc)
		omap_writel((omap_readl(ISPRSZ_CNT)) |
			(ISPRSZ_CNT_YCPOS), ISPRSZ_CNT);
	else
		omap_writel((omap_readl(ISPRSZ_CNT)) &
			(~ISPRSZ_CNT_YCPOS), ISPRSZ_CNT);
	DPRINTK_ISPRESZ("ispresizer_config_ycpos()-\n");
}

/*
 * Sets the chrominance algorithm
 * cbilin	:0 - chrominance uses same processing as luminance
 * 		 1 - bilinear interpolation processing
 */
void
ispresizer_enable_cbilin(u8 enable)
{
	DPRINTK_ISPRESZ("ispresizer_enable_cbilin()+\n");
	if (enable)
		omap_writel((omap_readl(ISPRSZ_CNT)) |
			(ISPRSZ_CNT_CBILIN), ISPRSZ_CNT);
	else
		omap_writel((omap_readl(ISPRSZ_CNT)) &
			(~ISPRSZ_CNT_CBILIN) , ISPRSZ_CNT);
	DPRINTK_ISPRESZ("ispresizer_enable_cbilin()-\n");
}

/*
 * Configures luminance enhancer parameters.
 * yenh		:structure containing desired values for core,slope,gain and 
 * 		 algo parameters
 */
void
ispresizer_config_luma_enhance(struct isprsz_yenh *yenh)
{
	DPRINTK_ISPRESZ("ispresizer_config_luma_enhance()+\n");
	ispres_obj.algo = yenh->algo;
	omap_writel((yenh->algo << ISPRSZ_YENH_ALGO_SHIFT) |
			(yenh->gain << ISPRSZ_YENH_GAIN_SHIFT) |
			(yenh->slope << ISPRSZ_YENH_SLOP_SHIFT) |
			(yenh->coreoffset << ISPRSZ_YENH_CORE_SHIFT),
			ISPRSZ_YENH);
	DPRINTK_ISPRESZ("ispresizer_config_luma_enhance()-\n");
}

/*
 * Sets the filter coefficients for both 4-tap and 7-tap mode.
 * This API just updates the isp_res struct.Actual register write happens in
 * ispresizer_config_size.
 * coef		:structure containing horizontal and vertical filter
 * 		 coefficients for both 4-tap and 7-tap mode
 */
void
ispresizer_config_filter_coef(struct isprsz_coef *coef)
{
	int i;
	DPRINTK_ISPRESZ("ispresizer_config_filter_coef()+\n");
	for (i=0;i<32;i++){
		ispres_obj.coeflist.h_filter_coef_4tap[i] =
			coef->h_filter_coef_4tap[i];
		ispres_obj.coeflist.v_filter_coef_4tap[i] =
			coef->v_filter_coef_4tap[i];
	}
	for (i=0;i<28;i++){
		ispres_obj.coeflist.h_filter_coef_7tap[i] =
			coef->h_filter_coef_7tap[i];
		ispres_obj.coeflist.v_filter_coef_7tap[i] =
			coef->v_filter_coef_7tap[i];
	}
	DPRINTK_ISPRESZ("ispresizer_config_filter_coef()-\n");
}

/*
 * Configures the Read address line offset.
 * offset		: Line Offset for the input image.
 */
int
ispresizer_config_inlineoffset(u32 offset)
{
	DPRINTK_ISPRESZ("ispresizer_config_inlineoffset()+\n");
	if (offset%32)
		return -EINVAL;
	omap_writel(offset << ISPRSZ_SDR_INOFF_OFFSET_SHIFT, ISPRSZ_SDR_INOFF);
	DPRINTK_ISPRESZ("ispresizer_config_inlineoffset()-\n");
	return 0;
}

/*
 * Configures the memory address from which the input frame is to be read.
 * addr		: 32bit memory address aligned on 32byte boundary.
 */
int
ispresizer_set_inaddr(u32 addr)
{
	DPRINTK_ISPRESZ("ispresizer_set_inaddr()+\n");
	if (addr%32)
		return -EINVAL;
	omap_writel(addr << ISPRSZ_SDR_INADD_ADDR_SHIFT, ISPRSZ_SDR_INADD);
	DPRINTK_ISPRESZ("ispresizer_set_inaddr()-\n");
	return 0;
}

/*
 * Configures the Write address line offset.
 * offset	: Line Offset for the preview output.
 */
int
ispresizer_config_outlineoffset(u32 offset)
{
	DPRINTK_ISPRESZ("ispresizer_config_outlineoffset()+\n");
	if (offset%32)
		return -EINVAL;
	omap_writel(offset << ISPRSZ_SDR_OUTOFF_OFFSET_SHIFT,
			ISPRSZ_SDR_OUTOFF);
	DPRINTK_ISPRESZ("ispresizer_config_outlineoffset()-\n");
	return 0;
}

/*
 * Configures the memory address to which the output frame is written.
 * addr		: 32bit memory address aligned on 32byte boundary.
 */
int
ispresizer_set_outaddr(u32 addr)
{
	DPRINTK_ISPRESZ("ispresizer_set_outaddr()+\n");
	if (addr%32)
		return -EINVAL;
	omap_writel(addr << ISPRSZ_SDR_OUTADD_ADDR_SHIFT, ISPRSZ_SDR_OUTADD);

	DPRINTK_ISPRESZ("ispresizer_set_outaddr()-\n");
	return 0;
}

/*
 * Saves the values of the resizer module registers.
 */
void 
ispresizer_save_context(void)
{
	DPRINTK_ISPRESZ (" Saving context\n");
	isp_save_context(isprsz_reg_list);
}

/*
 * Restores the values of the resizer module registers.
 */
void 
ispresizer_restore_context(void)
{
	DPRINTK_ISPRESZ (" Restoring context\n");
	isp_restore_context(isprsz_reg_list);
}

/*
 * Prints the values of the Resizer Module registers
 */
void
ispresizer_print_status()
{
#ifdef	OMAP_ISPRESZ_DEBUG
	DPRINTK_ISPRESZ("###ISP_CTRL inresizer =0x%x\n", omap_readl(ISP_CTRL));

	DPRINTK_ISPRESZ("###ISP_IRQ0ENABLE in resizer =0x%x\n",
			omap_readl(ISP_IRQ0ENABLE));
	DPRINTK_ISPRESZ("###ISP_IRQ0STATUS in resizer =0x%x\n",
			omap_readl(ISP_IRQ0STATUS));
	DPRINTK_ISPRESZ("###RSZ PCR =0x%x\n", omap_readl(ISPRSZ_PCR));
	DPRINTK_ISPRESZ("###RSZ CNT =0x%x\n", omap_readl(ISPRSZ_CNT));
	DPRINTK_ISPRESZ("###RSZ OUT SIZE =0x%x\n",
			omap_readl(ISPRSZ_OUT_SIZE));
	DPRINTK_ISPRESZ("###RSZ IN START =0x%x\n",
			omap_readl(ISPRSZ_IN_START));
	DPRINTK_ISPRESZ("###RSZ IN SIZE =0x%x\n", omap_readl(ISPRSZ_IN_SIZE));
	DPRINTK_ISPRESZ("###RSZ SDR INADD =0x%x\n",
			omap_readl(ISPRSZ_SDR_INADD));
	DPRINTK_ISPRESZ("###RSZ SDR INOFF =0x%x\n",
			omap_readl(ISPRSZ_SDR_INOFF));
	DPRINTK_ISPRESZ("###RSZ SDR OUTADD =0x%x\n",
			omap_readl(ISPRSZ_SDR_OUTADD));
	DPRINTK_ISPRESZ("###RSZ SDR OTOFF =0x%x\n",
			omap_readl(ISPRSZ_SDR_OUTOFF));
	DPRINTK_ISPRESZ("###RSZ YENH =0x%x\n", omap_readl(ISPRSZ_YENH));
#endif
}

/*
 * Module Initialisation.
 */
static int __init
isp_resizer_init(void)
{
	/*Nothing to do other than mutex init*/
	init_MUTEX(&(ispres_obj.semlock));
	return 0;
}

static void
isp_resizer_cleanup(void)
{
 /*Nothing to do*/
}

module_init(isp_resizer_init);
module_exit(isp_resizer_cleanup);
EXPORT_SYMBOL(ispresizer_config_shadow_registers);
EXPORT_SYMBOL(ispresizer_request);
EXPORT_SYMBOL(ispresizer_free);
EXPORT_SYMBOL(ispresizer_config_datapath);
EXPORT_SYMBOL(ispresizer_try_size);
EXPORT_SYMBOL(ispresizer_config_size);
EXPORT_SYMBOL(ispresizer_enable);
EXPORT_SYMBOL(ispresizer_config_startphase);
EXPORT_SYMBOL(ispresizer_config_ycpos);
EXPORT_SYMBOL(ispresizer_enable_cbilin);
EXPORT_SYMBOL(ispresizer_config_luma_enhance);
EXPORT_SYMBOL(ispresizer_config_filter_coef);
EXPORT_SYMBOL(ispresizer_config_inlineoffset);
EXPORT_SYMBOL(ispresizer_set_inaddr);
EXPORT_SYMBOL(ispresizer_config_outlineoffset);
EXPORT_SYMBOL(ispresizer_set_outaddr);
EXPORT_SYMBOL(ispresizer_save_context);
EXPORT_SYMBOL(ispresizer_restore_context);
EXPORT_SYMBOL(ispresizer_print_status);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP Resizer Library");
MODULE_LICENSE("GPL");
