/*
 * drivers/media/video/ispresizer.h
 *
 * Driver include file for Resizer module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_RESIZER_H
#define OMAP_ISP_RESIZER_H

/************************************************************************
The client is supposed to call resizer API in the following sequence:
	- request()
	- config_datatpath()
	- optionally config/enable sub modules
	- try/config size
	- setup callback
	- setup in/out memory offsets and ptrs
	- enable()
	...
	- disable()
	- free()
*************************************************************************/

void ispresizer_config_shadow_registers(void);

/*
 * Reserve the resizer module and turns on the clocks
 * Only one user at a time.
 */
int ispresizer_request(void);

/*
 * Marks Resizer module free and turns off the clocks.
 */
int ispresizer_free(void);

/*
 *Enumeration Constants for input format
 */
enum ispresizer_input {
	RSZ_OTFLY_YUV,
	RSZ_MEM_YUV,
	RSZ_MEM_COL8
};

/*
 * Sets up the default resizer configuration according to the arguments.
 */
int ispresizer_config_datapath(enum ispresizer_input input);

/*
 * Sets the chrominance algorithm
 */
void ispresizer_enable_cbilin(u8 enable);

/*
 * Sets whether the output should be in YC or CY format.
 */
void ispresizer_config_ycpos(u8 yc);

/*
 * Sets the horizontal and vertical start phase.
 */
void ispresizer_config_startphase(u8 hstartphase, u8 vstartphase);

/*
 * Structure for resizer filter coeffcients.
 */
struct isprsz_coef{
	/* 8-phase/4-tap mode(.5x-4x) */
	u16 h_filter_coef_4tap[32];
	u16 v_filter_coef_4tap[32];
	/* 4-phase/7-tap mode(.25x-.5x) */
	u16 h_filter_coef_7tap[28];
	u16 v_filter_coef_7tap[28];
};

/*
 * Sets the filter coefficients for both 4-tap and 7-tap mode.
 * Note this API doesn't program to hardware at all. It only make a local
 * copy of filter arrays. The actual programming happnes when _config_size
 * is called.
 */
void ispresizer_config_filter_coef(struct isprsz_coef *coef);

/*
 * Structure for resizer luminance enhancer parameters
 */
struct isprsz_yenh{
	u8 algo;
	u8 gain;
	u8 slope;
	u8 coreoffset;
	};

/*
 * Configures luminance enhancer parameters.
 */
void ispresizer_config_luma_enhance(struct isprsz_yenh *yenh);
/*
 * Calculates the horizontal and vertical resize ratio,number of pixels to
 * be cropped in the resizer module and checks the validity of various
 * parameters.We don't expose API to change RSZ_IN_START (cropping). HORZ_ST
 * and VERT_ST are implictly set based on the expected output size and the
 * need of small cropping on the input image.
 * User should already config yenh/stphase before attempting any size API.
 */
int ispresizer_try_size(u32 *input_w, u32 *input_h, u32 *output_w,
	       	u32 *output_h);


/*
 * Applies Crop values to hardware
 */
void ispresizer_applycrop(void);

/*
 * Try size for applying crop. Updates global resizer structure. Does not update h/w
 */
void ispresizer_trycrop(u32 left, u32 top, u32 width, u32 height, u32 ow, u32 oh);

/*
 * APT that programs I/O sizes, ratios, and the right filter coefficients
 * to resizer hardware.
 */
int ispresizer_config_size(u32 input_w, u32 input_h, u32 output_w, 
		u32 output_h);

/*
 * Configures the Read address line offset.
 */
int ispresizer_config_inlineoffset(u32 offset);

/*
 * Configures the memory address from which the input frame is to be read.
 */
int ispresizer_set_inaddr(u32 addr);

/*
 * Configures the Write address line offset.
 */
int ispresizer_config_outlineoffset(u32 offset);

/*
 * Configures the memory address to which the output frame is written.
 */
int ispresizer_set_outaddr(u32 addr);

/*
 * Enables the Resizer module.
 * ES1 only works on one-shot. ES2 allows On-The-Fly.
 * A client should config everything else before enabling the resizer.
 */
void ispresizer_enable(u8 enable);
int ispresizer_busy(void);

/*
 * Saves resizer context
 */
void ispresizer_save_context(void);

/*
 * Restores resizer context
 */
void ispresizer_restore_context(void);

/*
 * Prints the values of the Resizer Module registers
 */
void ispresizer_print_status(void);

#endif   /* OMAP_ISP_RESIZER_H */
