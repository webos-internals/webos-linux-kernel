/*
 * drivers/media/video/omap/isp/isppreview.h
 *
 * Driver include file for Preview module in TI's OMAP3430 Camera ISP
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

#ifndef OMAP_ISP_PREVIEW_H
#define OMAP_ISP_PREVIEW_H

/* Isp query control structure */

#define ISPPRV_BRIGHT_STEP		0x1
#define ISPPRV_BRIGHT_DEF		0x1
#define ISPPRV_BRIGHT_LOW		0x0
#define ISPPRV_BRIGHT_HIGH		0xF
#define ISPPRV_BRIGHT_UNITS      	0x7

#define ISPPRV_CONTRAST_STEP	0x1
#define ISPPRV_CONTRAST_DEF	0x2
#define ISPPRV_CONTRAST_LOW	0x0
#define ISPPRV_CONTRAST_HIGH	0xF
#define ISPPRV_CONTRAST_UNITS	0x5

#define NO_AVE   	0x0
#define AVE_2_PIX	0x1
#define AVE_4_PIX	0x2
#define AVE_8_PIX	0x3
#define AVE_ODD_PIXEL_DIST  	(1 << 4) /* For Bayer Sensors */
#define AVE_EVEN_PIXEL_DIST 	(1 << 2)

#define WB_GAIN_MAX	4
#define RGB_MAX         3

/* Features list */
#define PREV_AVERAGER			(1 << 0)
#define PREV_INVERSE_ALAW 		(1 << 1)
#define PREV_HORZ_MEDIAN_FILTER		(1 << 2)
#define PREV_NOISE_FILTER 		(1 << 3)
#define PREV_CFA         		(1 << 4)
#define PREV_GAMMA_BYPASS      		(1 << 5)
#define PREV_LUMA_ENHANCE     		(1 << 6)
#define PREV_CHROMA_SUPPRESS  		(1 << 7)
#define PREV_DARK_FRAME_SUBTRACT	(1 << 8)
#define PREV_LENS_SHADING    		(1 << 9)
#define PREV_DARK_FRAME_CAPTURE  	(1 << 10)
/*
 *Enumeration Constants for input and output format
 */
enum preview_input {
	PRV_RAW_CCDC,
	PRV_RAW_MEM,
	PRV_RGBBAYERCFA,
	PRV_COMPCFA,
	PRV_CCDC_DRKF,
	PRV_OTHERS
};
enum preview_output {
	PREVIEW_RSZ,
	PREVIEW_MEM
};
/*
 * Configure byte layout of YUV image
 */
enum preview_ycpos_mode {
	YCPOS_YCrYCb = 0,
	YCPOS_YCbYCr = 1,
	YCPOS_CbYCrY = 2,
	YCPOS_CrYCbY = 3
};

enum preview_color_effect {
	PREV_DEFAULT_COLOR = 0,
	PREV_BW_COLOR = 1,
	PREV_SEPIA_COLOR = 2
};
/*
 *Structure for Horizontal Median Filter
 */
struct ispprev_hmed {
	/* Distance between consecutive pixels of same color in the odd line*/
	u8 odddist;
	/*Distance between consecutive pixels of same color in the even line*/
	u8 evendist;
	/* Horizontal median filter threshold */
	u8 thres;
};
/*
 * Structure for Noise Filter
 */
struct ispprev_nf {
	/* Flag to enable or disable the Defect Correction in NF*/
	u8 defect_corr_en;
	/* Strength to be used in Noise Filter*/
	u8 strgth;
	/*Pointer to the Noise Filter table */
	u32 *table;
};
/*
 * Enumeration for CFA Formats supported by preview
 */
enum cfa_fmt {
	CFAFMT_BAYER, CFAFMT_SONYVGA, CFAFMT_RGBFOVEON,
	CFAFMT_DNSPL, CFAFMT_HONEYCOMB, CFAFMT_RRGGBBFOVEON
};
/*
 * Structure for CFA Inpterpolation
 */
struct ispprev_cfa {
	/* CFA Format Enum value supported by preview.*/
	enum cfa_fmt cfafmt;
	/* CFA Gradient Threshold - Vertical */
	u8 cfa_gradthrs_vert;
	/* CFA Gradient Threshold - Horizontal */
	u8 cfa_gradthrs_horz;
	/* Pointer to the CFA table */
	u32 *cfa_table;
};
/*
 * Structure for Gamma Correction
 */
struct ispprev_gtable {
	/* Pointer to the red gamma table */
	u32 *redtable;
	/* Pointer to the green gamma table */
	u32 *greentable;
	/* Pointer to the blue gamma table */
	u32 *bluetable;
};
/*
 * Structure for Chrominance Suppression
 */
struct ispprev_csup {
	/* Gain */
	u8 gain;
	/* Threshold */
	u8 thres;
	/* Flag to enable/disable the High Pass Filter */
	u8 hypf_en;
};
/*
 * Structure for White Balance
 */
struct ispprev_wbal {
	/*Digital gain (U10Q8) */
	u16 dgain;
	/*White balance gain - COEF 3 (U8Q5) */
	u8 coef3;
	/*White balance gain - COEF 2 (U8Q5) */
	u8 coef2;
	/*White balance gain - COEF 1 (U8Q5) */
	u8 coef1;
	/*White balance gain - COEF 0 (U8Q5) */
	u8 coef0;
};

struct prev_white_balance {
	u16 wb_dgain;	/* white balance common gain */
	u8 wb_gain[WB_GAIN_MAX];  /* individual color gains */
	u8 wb_coefmatrix[WB_GAIN_MAX][WB_GAIN_MAX];
};
/*
 * Structure for Black Adjustment
 */
struct ispprev_blkadj {
	/*Black level offset adjustment for Red in 2's complement format */
	u8 red;
	/*Black level offset adjustment for Green in 2's complement format */
	u8 green;
	/* Black level offset adjustment for Blue in 2's complement format */
	u8 blue;
};
/*
 * Structure for RGB to RGB Blending
 */
struct ispprev_rgbtorgb {
	/*
	 * Blending values(S12Q8 format)
	 *	[RR] [GR] [BR]
	 *	[RG] [GG] [BG]
	 *	[RB] [GB] [BB]
	 */
	u16 matrix[3][3];
	/*Blending offset value for R,G,B in 2's complement integer format*/
	u16 offset[3];
};
/*
 * Structure for Color Space Conversion from RGB-YCbYCr
 */
struct ispprev_csc {
	/*
	 *Color space conversion coefficients(S10Q8)
	 *	[CSCRY]    [CSCGY]   [CSCBY]
	 *	[CSCRCB] [CSCGCB] [CSCBCB]
	 *	[CSCRCR] [CSCGCR] [CSCBCR]
	 */
	u16 matrix[RGB_MAX][RGB_MAX];
	/*
	 *CSC offset values for Y offset, CB offset and CR offset respectively
	 */
	s16 offset[RGB_MAX];
};
/*
 * Structure for Y, C Value Limit
 */
struct ispprev_yclimit{
	u8 minC;
	u8 maxC;
	u8 minY;
	u8 maxY;
};

/*
 * Structure for size parameters
 */ 
struct prev_size_params {
	unsigned int hstart;	/* Starting pixel */
	unsigned int vstart;	/* Starting line */
	unsigned int hsize;	/* width of input image */
	unsigned int vsize;	/* height of input image */
	unsigned char pixsize;	/* pixel size of the image in 
				   terms of bits */
	unsigned short in_pitch;	/* line offset of input image */
	unsigned short out_pitch;	/* line offset of output image */
};

/*
 * Structure RGB2YCbCr parameters
 */ 
struct prev_rgb2ycbcr_coeffs {
	short coeff[RGB_MAX][RGB_MAX];	/* color conversion gains in 
					   3x3 matrix */
	short offset[RGB_MAX];	/* color conversion offsets */
};

/*
 * Structure for Dark frame suppression
 */ 
struct prev_darkfrm_params {
	u32 addr;	/* memory start address */
	u32 offset;	/* line offset */
};


/*
 * Structure for all configuration
 */
struct prev_params {
	u16 features;	/* Set of features enabled */

	enum preview_ycpos_mode pix_fmt; /* output pixel format */

	struct ispprev_cfa cfa; /* CFA coefficients */

	struct ispprev_csup csup;  /* chroma suppression coefficients */

	u32 *ytable;	/* luma enhancement coeffs */

	struct ispprev_nf nf; /* noise filter coefficients */

	struct ispprev_gtable gtable;	/* gamma coefficients */

	struct ispprev_wbal wbal;
	/*
	struct prev_white_balance prev_wbal;
	*/
	struct ispprev_blkadj blk_adj;	/* black adjustment parameters */

	struct ispprev_rgbtorgb rgb2rgb;  /* rgb blending parameters */

	struct ispprev_csc rgb2ycbcr;  /* rgb to ycbcr parameters */

	struct ispprev_hmed hmf_params;	/* horizontal median filter */
	
	struct prev_size_params size_params;	/* size parameters */
	struct prev_darkfrm_params drkf_params;
	u8 lens_shading_shift;
	u8 average;	/* down sampling rate for averager */

	u8 contrast;		/* contrast */
	u8 brightness;		/* brightness */
};

/*
 * Allows user to program shadow registers associated with preview module.
 */
void isppreview_config_shadow_registers(void);

/*
 * Reserve the preview module and turns on the clocks
 * Only one user at a time.
 */
int isppreview_request(void);

/*
 * Marks Preview module free and turns off the clocks.
 */
int isppreview_free(void);

/*
 * Sets up the default preview configuration according to the arguments.
 */
int isppreview_config_datapath(enum preview_input input,
					enum preview_output output);

/*
 * Configures Output Format in Preview
 */
void isppreview_config_ycpos(enum preview_ycpos_mode mode);

/*
 * Configures Averager in Preview
 */ 
void isppreview_config_averager(u8 average);

/*
 * Enable/Disable the Inverse A-Law module in Preview
 */
void isppreview_enable_invalaw(u8 enable);

/*
 * Enable/Disable of the darkframe subtract for each captured frame.
 */
void isppreview_enable_drkframe(u8 enable);

/*
 * If dark frame subtract not to be used, then enable this shading compensation
 */
void isppreview_enable_shadcomp(u8 enable);

/*
 * Configure the shift value to be used in shading compensation.
 */
void isppreview_config_drkf_shadcomp( u8 scomp_shtval );

/*
 * Enable/Disable the GammaByPass
 */
void isppreview_enable_gammabypass(u8 enable);

/*
 *Enable/Disable of the Horizontal Median Filter
 */
void isppreview_enable_hmed(u8 enable);

/*
 *Configures the Horizontal Median Filter
 */
void isppreview_config_hmed(struct ispprev_hmed);

/*
 * Enable/Disable the Noise Filter
 */
void isppreview_enable_noisefilter(u8 enable);

/*
 * Configures the Noise Filter
 */
void isppreview_config_noisefilter(struct ispprev_nf prev_nf);

/*
 * Configures the CFA Interpolation parameters
 */
void isppreview_config_cfa(struct ispprev_cfa);

/*
 * Configures the Gamma Correction table values
 */
void isppreview_config_gammacorrn(struct ispprev_gtable);

/*
 * Configures the Chroma Suppression
 */
void isppreview_config_chroma_suppression( struct ispprev_csup csup );

/*
 * Enable/Disable the CFA Interpolation
 */
void isppreview_enable_cfa(u8 enable);

/*
 * Configures the Luminance Enhancement table values
 */
void isppreview_config_luma_enhancement(u32 * ytable );

/*
 * Enable/Disable the Luminance Enhancement
 */
void isppreview_enable_luma_enhancement(u8 enable);

/*
 * Enable/Disable the Chrominance Suppression
 */
void isppreview_enable_chroma_suppression(u8 enable);

/*
 * Configures the White Balance parameters
 */
void isppreview_config_whitebalance(struct ispprev_wbal);

/*
 * Configures the Black Adjustment parameters
 */
void isppreview_config_blkadj(struct ispprev_blkadj);

/*
 * Configures the RGB-RGB Blending matrix
 */
void isppreview_config_rgb_blending(struct ispprev_rgbtorgb);

/*
 * Configures the RGB-YCbYCr conversion matrix
 */
void isppreview_config_rgb_to_ycbcr(struct ispprev_csc);

/*
 * Updates the contrast setting in the preview module.
 */
void isppreview_update_contrast(u8 *contrast);

/*
 * Query the current contrast setting in hardware
 */
void isppreview_query_contrast(u8 *contrast);

/*
 * Configures the Contrast.
 */
void isppreview_config_contrast(u8 contrast);

/*
 * Gets the range contrast value
 */
void isppreview_get_contrast_range( u8 *min_contrast, u8 *max_contrast );

/* 
* Update the current brightness setting in the preview module 
*/
void isppreview_update_brightness(u8 *brightness);

/*
 * Configures the brightness.
 */
void isppreview_config_brightness(u8 brightness);

/*
 * Gets the range brightness value
 */
void isppreview_get_brightness_range( u8 *min_brightness,
						u8 *max_brightness );

void isppreview_set_color(u8 *mode);
void isppreview_get_color(u8 *mode);

/*
 * Query the current brightness setting in hardware
 */
void isppreview_query_brightness( u8 *brightness);

/*
 * Configures the max and minim Y and C values.
 */
void isppreview_config_yc_range( struct ispprev_yclimit yclimit );

/*
 * Calculates the number of pixels cropped in the submodules that are enabled,
 * Fills up the output widht height variables in the isp_prev structure .
*/
void isppreview_try_size(u32 input_w, u32 input_h, u32 * output_w,
				u32 * output_h);

/*
 * Configures the appropriate values stored in the isp_prev structure to
 * HORZ/VERT_INFO.
 * Configures PRV_AVE if needed for downampling as calculated in trysize.
 */
int isppreview_config_size(u32 input_w, u32 input_h, u32 output_w,
			u32 output_h);

/*
 * Configures the Read address line offset.
 */
int isppreview_config_inlineoffset(u32 offset);

/*
 * Configures the memory address from which the input frame is to be read.
 */
int isppreview_set_inaddr(u32 addr);

/*
 * Configures the Write address line offset.
 */
int isppreview_config_outlineoffset(u32 offset);

/*
 * Configures the memory address to which the output frame is written.
 */
int isppreview_set_outaddr(u32 addr);

/*
 * Configures the Dark frame address line offset.
 */
int isppreview_config_darklineoffset(u32 offset);

/*
 * Configures the memory address where the Dark frame should be stored.
 */
int isppreview_set_darkaddr(u32 addr);

/*
 * Enables the Preview module.
 */
void isppreview_enable(u8 enable);

/*
 * Checks Busy Status of Preview Engine
 */ 
int isppreview_busy(void);

/*
 * Gets pointer to Preview Parameters Structure used by ISP Preview
 */ 
struct prev_params * isppreview_get_config(void);

/*
 * Prints the values of the Preview Module registers
 * Also prints other debug information stored in the preview moduel
 */
void isppreview_print_status(void);

/*
 * Saves preview context
 */
void isppreview_save_context(void);

/*
 * Restores preview context
 */
void isppreview_restore_context(void);

#endif/* OMAP_ISP_PREVIEW_H */
