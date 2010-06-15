/*
 * drivers/media/video/omap/isp/isppreview.c
 *
 * Driver Library for Preview module in TI's OMAP3430 Camera ISP
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

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <asm/io.h>
#include <asm/semaphore.h>

#include "isp.h"
#include "ispreg.h"
#include "isppreview.h"

/* Structure for saving/restoring preview module registers*/
static struct isp_reg ispprev_reg_list[]= {
	{ISPPRV_HORZ_INFO, 0x0000},
	{ISPPRV_VERT_INFO, 0x0000},
	{ISPPRV_RSDR_ADDR, 0x0000},
	{ISPPRV_RADR_OFFSET, 0x0000},
	{ISPPRV_DSDR_ADDR, 0x0000},
	{ISPPRV_DRKF_OFFSET, 0x0000},
	{ISPPRV_WSDR_ADDR, 0x0000},
	{ISPPRV_WADD_OFFSET, 0x0000},
	{ISPPRV_AVE, 0x0000},
	{ISPPRV_HMED, 0x0000},
	{ISPPRV_NF, 0x0000},
	{ISPPRV_WB_DGAIN, 0x0000},
	{ISPPRV_WBGAIN, 0x0000},
	{ISPPRV_WBSEL, 0x0000},
	{ISPPRV_CFA, 0x0000},
	{ISPPRV_BLKADJOFF, 0x0000},
 	{ISPPRV_RGB_MAT1, 0x0000},
	{ISPPRV_RGB_MAT2, 0x0000},
	{ISPPRV_RGB_MAT3, 0x0000},
	{ISPPRV_RGB_MAT4, 0x0000},
	{ISPPRV_RGB_MAT5, 0x0000},
	{ISPPRV_RGB_OFF1, 0x0000},
	{ISPPRV_RGB_OFF2, 0x0000},
	{ISPPRV_CSC0, 0x0000},
	{ISPPRV_CSC1, 0x0000},
	{ISPPRV_CSC2, 0x0000},
	{ISPPRV_CSC_OFFSET, 0x0000},
	{ISPPRV_CNT_BRT, 0x0000},
	{ISPPRV_CSUP, 0x0000},
	{ISPPRV_SETUP_YC, 0x0000},
	{ISPPRV_SET_TBL_ADDR, 0x0000},
	{ISPPRV_SET_TBL_DATA, 0x0000},
	{ISPPRV_CDC_THR0, 0x0000},
	{ISPPRV_CDC_THR1, 0x0000},
	{ISPPRV_CDC_THR2, 0x0000},
	{ISPPRV_CDC_THR3, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};


/* Default values in Office Flourescent Light for RGBtoRGB Blending */
static struct ispprev_rgbtorgb flr_rgb2rgb = {
	{	/* RGB-RGB Matrix */
		{ 0x01E2, 0x0F30, 0x0FEE },
		{ 0x0F9B, 0x01AC, 0x0FB9 },
		{ 0x0FE0, 0x0EC0, 0x0260 }
	},	/* RGB Offset */
		{0x0000, 0x0000, 0x0000}
};

/* Default values in Office Flourescent Light for RGB to YUV Conversion*/
static struct ispprev_csc flr_prev_csc[] = {
	{
		{	/* CSC Coef Matrix */
			{ 66, 129, 25},
			{ -38, -75, 112},
			{ 112,-94 ,-18}
		},	/* CSC Offset */
			{0x0,0x0,0x0}
	},
	{
		{	/* CSC Coef Matrix Sepia*/
			{ 19, 38, 7},
			{ 0, 0, 0},
			{ 0, 0 ,0}
		},	/* CSC Offset */
			{0x0,0xE7,0x14}
	},
	{
		{	/* CSC Coef Matrix BW*/
			{ 66, 129, 25},
			{ 0, 0, 0},
			{ 0, 0 ,0}
		},	/* CSC Offset */
			{0x0,0x0,0x0}
	}
};


/* Default values in Office Flourescent Light for CFA Gradient*/
static u8 flr_cfa_gradthrs_horz = 0x28;
static u8 flr_cfa_gradthrs_vert = 0x28;

/* Default values in Office Flourescent Light for Chroma Suppression*/
static u8 flr_csup_gain = 0x0D;
static u8 flr_csup_thres = 0xEB;

/* Default values in Office Flourescent Light for Noise Filter*/
static u8 flr_nf_strgth = 0x0d ;

/* Default values in Office Flourescent Light for White Balance*/
static u16 flr_wbal_dgain = 0x100;
static u8 flr_wbal_coef0 = 0x68;
static u8 flr_wbal_coef1 = 0x5c;
static u8 flr_wbal_coef2 = 0x5c;
static u8 flr_wbal_coef3 = 0x94;

/* Default values in Office Flourescent Light for Black Adjustment*/
static u8 flr_blkadj_blue = 0x0;
static u8 flr_blkadj_green = 0x0;
static u8 flr_blkadj_red = 0x0;

static int update_color_matrix;

/*
 * Structure for the preview module to store its own information.
 */
static struct isp_prev {
	u8 prev_inuse;
	u32 prevout_w;
	u32 prevout_h;
	u32 previn_w;
	u32 previn_h;
	enum preview_input prev_inpfmt;
	enum preview_output prev_outfmt;
	u8 hmed_en;
	u8 nf_en;
	u8 cfa_en;
	u8 csup_en;
	u8 yenh_en;
	u8 fmtavg;
	u8 brightness;
	u8 contrast;
	enum preview_color_effect color;
	enum cfa_fmt cfafmt;
	struct semaphore semlock;
} ispprev_obj;

/* Saved parameters */
struct prev_params *prev_config_params = NULL;

/*
 * Coeficient Tables for the submodules in Preview.
 * Array is initialised with the values from.the tables text file.
 */

/*
 * CFA Filter Coefficient Table
 *
 */
static u32 cfa_coef_table[] = {
#include "cfa_coef_table.h"
};

/*
 * Gamma Correction Table - Red
 */
static u32 redgamma_table[] = {
#include "redgamma_table.h"
};

/*
 * Gamma Correction Table - Green
 */
static u32 greengamma_table[] = {
#include "greengamma_table.h"
};

/*
 * Gamma Correction Table - Blue
 */
static u32 bluegamma_table[] = {
#include "bluegamma_table.h"
};

/*
 * Noise Filter Threshold table
 */
static u32 noise_filter_table[] = {
#include "noise_filter_table.h"
};

/*
 * Luminance Enhancement Table
 */
static u32 luma_enhance_table[] = {
#include "luma_enhance_table.h"
};

/*
 * Allows user to program shadow registers associated with preview module.
 */
void
isppreview_config_shadow_registers()
{
	u8 current_brightness_contrast;
	
	/* Program Brightness if needed */
	isppreview_query_brightness(&current_brightness_contrast);	
	if(current_brightness_contrast != ((ispprev_obj.brightness)*ISPPRV_BRIGHT_UNITS))
		{
		DPRINTK_ISPPREV(" Changing Brightness level to %d\n", ispprev_obj.brightness);
		isppreview_config_brightness((ispprev_obj.brightness)*ISPPRV_BRIGHT_UNITS);
		}
	
	/* Program Contrast if needed */
	isppreview_query_contrast(&current_brightness_contrast);	
	if(current_brightness_contrast != ((ispprev_obj.contrast)*ISPPRV_CONTRAST_UNITS))
		{
		DPRINTK_ISPPREV(" Changing Contrast level to %d\n", ispprev_obj.contrast);
		isppreview_config_contrast((ispprev_obj.contrast)*ISPPRV_CONTRAST_UNITS);
		}
	if(update_color_matrix) {
		isppreview_config_rgb_to_ycbcr(
			flr_prev_csc[ispprev_obj.color]);
		update_color_matrix = 0;
	}
}

/*
 * Reserve the preview module.
 * Only one user at a time.
 */
int
isppreview_request()
{
	down(&(ispprev_obj.semlock));
	if (!(ispprev_obj.prev_inuse)) {
		ispprev_obj.prev_inuse = 1;
		up(&(ispprev_obj.semlock));
		/* Turn on Preview module Clocks. */
		omap_writel((omap_readl(ISP_CTRL)) | ISPCTRL_PREV_RAM_EN |
			ISPCTRL_PREV_CLK_EN | ISPCTRL_SBL_WR1_RAM_EN
			,ISP_CTRL);
		return 0;
	} else{
		up(&(ispprev_obj.semlock));
		printk(KERN_ERR "ISP_ERR : Preview Module Busy\n");
		return -EBUSY;
	}
}

/*
 * Marks Preview module free.
 */
int
isppreview_free()
{
	down(&(ispprev_obj.semlock));
	if (ispprev_obj.prev_inuse){
		ispprev_obj.prev_inuse = 0;
		up(&(ispprev_obj.semlock));
		omap_writel(omap_readl(ISP_CTRL)& ~(ISPCTRL_PREV_CLK_EN |
				ISPCTRL_PREV_RAM_EN
				|ISPCTRL_SBL_WR1_RAM_EN), ISP_CTRL);
		return 0;
	}
	else{
		up(&(ispprev_obj.semlock));
		DPRINTK_ISPPREV("ISP_ERR : Preview Module already freed\n");
		return -EINVAL;
	}

}

/* Sets up the default preview configuration according to the arguments.
 * input		: Indicates the module that gives the image to preview
 * output		: Indicates the module to which the preview outputs to.
 */
int
isppreview_config_datapath(enum preview_input input,
					enum preview_output output)
{
	u32 pcr = 0;
	u8 enable = 0;
	struct prev_params *params = prev_config_params;
	struct ispprev_yclimit yclimit;

	pcr = omap_readl(ISPPRV_PCR);

	switch (input) {
	case PRV_RAW_CCDC:
		pcr &= ~(ISPPRV_PCR_SOURCE);
		pcr &= ~(ISPPRV_PCR_ONESHOT);
		ispprev_obj.prev_inpfmt = PRV_RAW_CCDC;
		break;
	case PRV_RAW_MEM:
		pcr |= ISPPRV_PCR_SOURCE;
		pcr |= ISPPRV_PCR_ONESHOT;
		ispprev_obj.prev_inpfmt = PRV_RAW_MEM;
		break;
	case PRV_CCDC_DRKF:
		pcr |= ISPPRV_PCR_DRKFCAP;
		pcr |= ISPPRV_PCR_ONESHOT;
		ispprev_obj.prev_inpfmt = PRV_CCDC_DRKF;
		break;
	/* Just check for input path validity.  No PCR update required
	* for the current HW setup.
	*/
	case PRV_COMPCFA:
		ispprev_obj.prev_inpfmt = PRV_COMPCFA;
		break;
	case PRV_OTHERS:
		ispprev_obj.prev_inpfmt = PRV_OTHERS;
		break;
	case PRV_RGBBAYERCFA:
		ispprev_obj.prev_inpfmt = PRV_RGBBAYERCFA;
		break;
	default:
		printk(KERN_ERR "ISP_ERR : Wrong Input\n");
		return -EINVAL;
	};

	if (output == PREVIEW_RSZ){
		pcr |= ISPPRV_PCR_RSZPORT;
		pcr &= (~ISPPRV_PCR_SDRPORT);
		ispprev_obj.prev_outfmt = PREVIEW_RSZ;
	}
	else if (output == PREVIEW_MEM){
		pcr &= (~ISPPRV_PCR_RSZPORT);
		pcr |= ISPPRV_PCR_SDRPORT;
		ispprev_obj.prev_outfmt = PREVIEW_MEM;
	}
	else{
		printk(KERN_ERR "ISP_ERR : Wrong Output\n");
		return -EINVAL;
	}
	omap_writel(pcr, ISPPRV_PCR);

	/* Default Output format configured is YCrYCb (UYVY) */
	isppreview_config_ycpos(params->pix_fmt);

	/* CFA */
	if (params->cfa.cfa_table != NULL)
		isppreview_config_cfa(params->cfa);
	/* Chroma Suppression */
	if (params->csup.hypf_en == 1)
		isppreview_config_chroma_suppression(params->csup);
	/* Luma */
	if (params->ytable != NULL)
		isppreview_config_luma_enhancement(params->ytable);
	/* Noise Filter */
	if (params->nf.defect_corr_en == 1)
		isppreview_config_noisefilter(params->nf);
	/* Gamma Correction */
	if (params->gtable.redtable != NULL)
		isppreview_config_gammacorrn(params->gtable);

	/* Enabling specific features */
	enable =((params->features & PREV_CFA) == PREV_CFA) ? 1 : 0;
	isppreview_enable_cfa(enable);

	enable = ((params->features & PREV_CHROMA_SUPPRESS)
			== PREV_CHROMA_SUPPRESS) ? 1 : 0;
	isppreview_enable_chroma_suppression(enable);

	enable = ((params->features & PREV_LUMA_ENHANCE)
			== PREV_LUMA_ENHANCE) ? 1 : 0;
	isppreview_enable_luma_enhancement(enable);
	
	enable = ((params->features & PREV_NOISE_FILTER)
			== PREV_NOISE_FILTER) ? 1 : 0;
	isppreview_enable_noisefilter(enable);

	enable = ((params->features & PREV_GAMMA_BYPASS)
			== PREV_GAMMA_BYPASS) ? 1 : 0;
	isppreview_enable_gammabypass(enable);

	isppreview_config_whitebalance(params->wbal);
	isppreview_config_blkadj(params->blk_adj);
	isppreview_config_rgb_blending(params->rgb2rgb);
	isppreview_config_rgb_to_ycbcr(params->rgb2ycbcr);

	isppreview_config_contrast(params->contrast * ISPPRV_CONTRAST_UNITS);
	isppreview_config_brightness(params->brightness * ISPPRV_BRIGHT_UNITS);

	yclimit.minC = ISPPRV_YC_MIN;
	yclimit.maxC = ISPPRV_YC_MAX;
	yclimit.minY = ISPPRV_YC_MIN;
	yclimit.maxY = ISPPRV_YC_MAX;
	isppreview_config_yc_range(yclimit);

	return 0;
}

/*
 * Configure byte layout of YUV image
 */
void isppreview_config_ycpos(enum preview_ycpos_mode mode)
{
	u32 pcr = omap_readl(ISPPRV_PCR);
	pcr &= (~ISPPRV_PCR_YCPOS_CrYCbY);
	pcr |= (mode << ISPPRV_PCR_YCPOS_SHIFT);
	omap_writel(pcr, ISPPRV_PCR);
}

/*
 * Enable/disable/configure averager
 */
void
isppreview_config_averager(u8 average)
{
	int reg = 0;

	reg = AVE_ODD_PIXEL_DIST | AVE_EVEN_PIXEL_DIST | average;
	omap_writel(reg, ISPPRV_AVE);
}

/*
 * Enable/Disable the Inverse A-Law module in Preview
 * enable		: 1- Reverse the ALaw done in CCDC.
 */
void
isppreview_enable_invalaw(u8 enable)
{
	u32 pcr_val = 0;
	pcr_val = omap_readl(ISPPRV_PCR);

	if(enable)
		omap_writel(pcr_val | ISPPRV_PCR_WIDTH | ISPPRV_PCR_INVALAW,
			ISPPRV_PCR);
	else
		omap_writel(pcr_val & ~( ISPPRV_PCR_WIDTH | ISPPRV_PCR_INVALAW),
			ISPPRV_PCR);
}



/* Enable/Disable of the darkframe subtract for each captured frame.
 * enable		: 1- Acquires memory bandwidth since the pixels in 
 *  			each frame is subtracted with the pixels in the 
 *  			current frame.
 */
void
isppreview_enable_drkframe(u8 enable)
{
	if(enable)
		omap_writel(omap_readl(ISPPRV_PCR) | ISPPRV_PCR_DRKFEN,
					ISPPRV_PCR);
	else
		omap_writel((omap_readl(ISPPRV_PCR)) & ~ISPPRV_PCR_DRKFEN,
					ISPPRV_PCR);
}

/* If dark frame subtract not to be used, then enable this shading compensation.
 * enable		: 1- Enables the shading compensation.
 */
void
isppreview_enable_shadcomp(u8 enable)
{

	if(enable){
		omap_writel((omap_readl(ISPPRV_PCR))| ISPPRV_PCR_SCOMP_EN,
				ISPPRV_PCR);
		isppreview_enable_drkframe(1);
	}
	else
		omap_writel((omap_readl(ISPPRV_PCR))& ~ISPPRV_PCR_SCOMP_EN,
				ISPPRV_PCR);
}

/* Configure the shift value to be used in shading compensation.
 * scomp_shtval	: 3bit value of shift used in shading compensation.
 */
void
isppreview_config_drkf_shadcomp( u8 scomp_shtval )
{
	u32 pcr_val = omap_readl(ISPPRV_PCR);

	pcr_val &=ISPPRV_PCR_SCOMP_SFT_MASK;
	omap_writel(pcr_val |(scomp_shtval <<ISPPRV_PCR_SCOMP_SFT_SHIFT),
				ISPPRV_PCR);
}

/*
 *Enable/Disable of the Horizontal Median Filter
 * enable		: 1- Enables Horizontal Median Filter
 */
void
isppreview_enable_hmed(u8 enable)
{
	if (enable) {
		omap_writel((omap_readl(ISPPRV_PCR)) | ISPPRV_PCR_HMEDEN,
			ISPPRV_PCR);
		ispprev_obj.hmed_en = 1;
	} else {
		omap_writel((omap_readl(ISPPRV_PCR)) & (~ISPPRV_PCR_HMEDEN),
			ISPPRV_PCR);
		ispprev_obj.hmed_en = 0;
	}
}

/*
 *Configures the Horizontal Median Filter
 * prev_hmed	: Structure containing the odd and even distance between the
 * 		pixels in the image along with the filter threshold.
 */
void
isppreview_config_hmed(struct ispprev_hmed prev_hmed)
{

	u32 odddist =0;
	u32 evendist =0;

	if (prev_hmed.odddist == 1)
		odddist = ~ISPPRV_HMED_ODDDIST;
	else /* else the odd distance is 2 */
		odddist = ISPPRV_HMED_ODDDIST;

	if (prev_hmed.evendist == 1)
		evendist = ~ISPPRV_HMED_EVENDIST;
	else /* else the even distance is 2 */
		evendist = ISPPRV_HMED_EVENDIST;

	omap_writel(odddist | evendist
			| (prev_hmed.thres<<ISPPRV_HMED_THRESHOLD_SHIFT),
			ISPPRV_HMED);

}

/*
 * Configures the Noise Filter
 * prev_nf	: Structure containing the noisefilter table , strength to be used
 *		for the noise filter and the defect correction enable flag.
 */
void
isppreview_config_noisefilter(struct ispprev_nf prev_nf)
{
	int i =0;
	omap_writel(prev_nf.strgth, ISPPRV_NF);

	if (prev_nf.defect_corr_en)
		omap_writel((omap_readl(ISPPRV_PCR)) | ISPPRV_PCR_DCOREN,
			ISPPRV_PCR);
	else
		omap_writel((omap_readl(ISPPRV_PCR)) &(~ ISPPRV_PCR_DCOREN),
			ISPPRV_PCR);

	omap_writel(ISPPRV_NF_TABLE_ADDR, ISPPRV_SET_TBL_ADDR);

	/* Array of 256 */
	for (i = 0; i < 256; i++)
		omap_writel(prev_nf.table[i], ISPPRV_SET_TBL_DATA);

}

/*
 * Configures the CFA Interpolation parameters
 * prev_cfa	: Structure containing the CFA interpolation table, CFA format
 *		in the image, vertical and horizontal gradient threshold.
 */
void
isppreview_config_cfa(struct ispprev_cfa prev_cfa)
{
	int i =0;
	ispprev_obj.cfafmt = prev_cfa.cfafmt;

	omap_writel((omap_readl(ISPPRV_PCR))
		| (prev_cfa.cfafmt << ISPPRV_PCR_CFAFMT_SHIFT), ISPPRV_PCR);

	omap_writel((prev_cfa.cfa_gradthrs_vert << ISPPRV_CFA_GRADTH_VER_SHIFT)
		| (prev_cfa.cfa_gradthrs_horz << ISPPRV_CFA_GRADTH_HOR_SHIFT),
		ISPPRV_CFA);

	omap_writel(ISPPRV_CFA_TABLE_ADDR, ISPPRV_SET_TBL_ADDR);

	/* Array of 576 */
	for (i = 0; i < 576; i++)
		omap_writel(prev_cfa.cfa_table[i], ISPPRV_SET_TBL_DATA);

}

/*
 * Configures the Gamma Correction table values
  * gtable	: Structure containing the table for red,blue,green gamma table.
 */
void
isppreview_config_gammacorrn(struct ispprev_gtable gtable)
{
	int i = 0;

	omap_writel(ISPPRV_REDGAMMA_TABLE_ADDR, ISPPRV_SET_TBL_ADDR);
	/* Array of 1024 */
	for (i = 0; i < 1024; i++)
		omap_writel(gtable.redtable[i], ISPPRV_SET_TBL_DATA);

	omap_writel(ISPPRV_GREENGAMMA_TABLE_ADDR, ISPPRV_SET_TBL_ADDR);
	/* Array of 1024 */
	for (i = 0; i < 1024; i++)
		omap_writel(gtable.greentable[i], ISPPRV_SET_TBL_DATA);

	omap_writel(ISPPRV_BLUEGAMMA_TABLE_ADDR, ISPPRV_SET_TBL_ADDR);
	/* Array of 1024 */
	for (i = 0; i < 1024; i++)
		omap_writel(gtable.bluetable[i], ISPPRV_SET_TBL_DATA);
}

/*
 * Configures the Luminance Enhancement table values
  * ytable	: Structure containing the table for Luminance Enhancement table.
 */
void
isppreview_config_luma_enhancement(u32 * ytable)
{
	int i = 0;
	omap_writel(ISPPRV_YENH_TABLE_ADDR, ISPPRV_SET_TBL_ADDR);
	/* Array of 128 */
	for (i = 0; i < 128; i++)
		omap_writel(ytable[i], ISPPRV_SET_TBL_DATA);
}

/*
 * Configures the Chroma Suppression
 * csup		: Structure containing the threshold value for suppression
 *		and the hypass filter enable flag.
 */
void
isppreview_config_chroma_suppression(struct ispprev_csup csup)
{
	omap_writel(csup.gain | (csup.thres << ISPPRV_CSUP_THRES_SHIFT)
			| (csup.hypf_en << ISPPRV_CSUP_HPYF_SHIFT)
			, ISPPRV_CSUP);
}

/*
 * Enable/Disable the Noise Filter
 * enable		: 1 - Enables the Noise Filter.
 */
void
isppreview_enable_noisefilter(u8 enable)
{
	if (enable) {
		omap_writel((omap_readl(ISPPRV_PCR)) | ISPPRV_PCR_NFEN,
			ISPPRV_PCR);
		ispprev_obj.nf_en = 1;
	} else {
		omap_writel((omap_readl(ISPPRV_PCR)) & (~ISPPRV_PCR_NFEN),
			ISPPRV_PCR);
		ispprev_obj.nf_en = 0;
	}
}

/*
 * Enable/Disable the CFA Interpolation
 * enable		: 1 - Enables the CFA.
 */
void
isppreview_enable_cfa(u8 enable)
{
	if (enable) {
		omap_writel((omap_readl(ISPPRV_PCR)) | ISPPRV_PCR_CFAEN,
			ISPPRV_PCR);
		ispprev_obj.cfa_en = 1;
	} else {
		omap_writel((omap_readl(ISPPRV_PCR)) & (~ISPPRV_PCR_CFAEN),
			ISPPRV_PCR);
		ispprev_obj.cfa_en = 0;
	}

}

/*
 * Enable/Disable the GammaByPass
 * enable		: 1 - Bypasses Gamma - 10bit input is cropped to 8MSB.
 		0 - Goes through Gamma Correction. input and output is 10bit.
  */
void
isppreview_enable_gammabypass(u8 enable)
{
	if (enable)
		omap_writel((omap_readl(ISPPRV_PCR)) | ISPPRV_PCR_GAMMA_BYPASS,
			ISPPRV_PCR);
	else
		omap_writel((omap_readl(ISPPRV_PCR)) & (~ISPPRV_PCR_GAMMA_BYPASS),
			ISPPRV_PCR);
}

/*
 * Enable/Disable the Luminance Enhancement
 * enable		: 1 - Enable the Luminance Enhancement.
 */
void
isppreview_enable_luma_enhancement(u8 enable)
{
	if (enable){
		omap_writel((omap_readl(ISPPRV_PCR)) | ISPPRV_PCR_YNENHEN,
			ISPPRV_PCR);
		ispprev_obj.yenh_en = 1;
		}
	else{
		omap_writel((omap_readl(ISPPRV_PCR)) & (~ISPPRV_PCR_YNENHEN),
			ISPPRV_PCR);
		ispprev_obj.yenh_en =0;
	}
}

/*
 * Enable/Disable the Chrominance Suppression
 * enable		: 1 - Enable the Chrominance Suppression.
  */
void
isppreview_enable_chroma_suppression(u8 enable)
{
	if (enable){
		omap_writel((omap_readl(ISPPRV_PCR)) | ISPPRV_PCR_SUPEN,
			ISPPRV_PCR);
		ispprev_obj.csup_en =1;
	}
	else{
		omap_writel((omap_readl(ISPPRV_PCR)) & (~ISPPRV_PCR_SUPEN),
			ISPPRV_PCR);
		ispprev_obj.csup_en =0;
	}
}

/*
 * Configures the White Balance parameters. Coefficient matrix always with
 * 		default values. 
 * prev_wbal	: Structure containing the digital gain and white balance
 *		coefficient.
 */
void
isppreview_config_whitebalance(struct ispprev_wbal prev_wbal)
{

	omap_writel(prev_wbal.dgain, ISPPRV_WB_DGAIN);
	omap_writel(prev_wbal.coef0 |
		prev_wbal.coef1 << ISPPRV_WBGAIN_COEF1_SHIFT |
		prev_wbal.coef2 << ISPPRV_WBGAIN_COEF2_SHIFT |
		prev_wbal.coef3 << ISPPRV_WBGAIN_COEF3_SHIFT, ISPPRV_WBGAIN);

	/* Keeping the HW default value as such */
	omap_writel(ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N0_0_SHIFT
		| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N0_1_SHIFT
		| ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N0_2_SHIFT
		| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N0_3_SHIFT
		| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N1_0_SHIFT
		| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N1_1_SHIFT
		| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N1_2_SHIFT
		| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N1_3_SHIFT
		| ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N2_0_SHIFT
		| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N2_1_SHIFT
		| ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N2_2_SHIFT
		| ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N2_3_SHIFT
		| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N3_0_SHIFT
		| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N3_1_SHIFT
		| ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N3_2_SHIFT
		| ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N3_3_SHIFT,
		ISPPRV_WBSEL);

}

/*
 * Configures the White Balance parameters. Coefficient matrix can be changed.
 * prev_wbal	: Structure containing the digital gain and white balance
 *		coefficient.
 */
void
isppreview_config_whitebalance2(struct prev_white_balance prev_wbal)
{
	omap_writel(prev_wbal.wb_dgain, ISPPRV_WB_DGAIN);
	omap_writel(prev_wbal.wb_gain[0]
		| prev_wbal.wb_gain[1] << ISPPRV_WBGAIN_COEF1_SHIFT
		| prev_wbal.wb_gain[2] << ISPPRV_WBGAIN_COEF2_SHIFT
		| prev_wbal.wb_gain[3] << ISPPRV_WBGAIN_COEF3_SHIFT,
		ISPPRV_WBGAIN);

	/* Changing the HW default value */
	omap_writel(prev_wbal.wb_coefmatrix[0][0] << ISPPRV_WBSEL_N0_0_SHIFT
		| prev_wbal.wb_coefmatrix[0][1] << ISPPRV_WBSEL_N0_1_SHIFT
		| prev_wbal.wb_coefmatrix[0][2] << ISPPRV_WBSEL_N0_2_SHIFT
		| prev_wbal.wb_coefmatrix[0][3] << ISPPRV_WBSEL_N0_3_SHIFT
		| prev_wbal.wb_coefmatrix[1][0] << ISPPRV_WBSEL_N1_0_SHIFT
		| prev_wbal.wb_coefmatrix[1][1] << ISPPRV_WBSEL_N1_1_SHIFT
		| prev_wbal.wb_coefmatrix[1][2] << ISPPRV_WBSEL_N1_2_SHIFT
		| prev_wbal.wb_coefmatrix[1][3] << ISPPRV_WBSEL_N1_3_SHIFT
		| prev_wbal.wb_coefmatrix[2][0] << ISPPRV_WBSEL_N2_0_SHIFT
		| prev_wbal.wb_coefmatrix[2][1] << ISPPRV_WBSEL_N2_1_SHIFT
		| prev_wbal.wb_coefmatrix[2][2] << ISPPRV_WBSEL_N2_2_SHIFT
		| prev_wbal.wb_coefmatrix[2][3] << ISPPRV_WBSEL_N2_3_SHIFT
		| prev_wbal.wb_coefmatrix[3][0] << ISPPRV_WBSEL_N3_0_SHIFT
		| prev_wbal.wb_coefmatrix[3][1] << ISPPRV_WBSEL_N3_1_SHIFT
		| prev_wbal.wb_coefmatrix[3][2] << ISPPRV_WBSEL_N3_2_SHIFT
		| prev_wbal.wb_coefmatrix[3][3] << ISPPRV_WBSEL_N3_3_SHIFT,
		ISPPRV_WBSEL);
}


/*
 * Configures the Black Adjustment parameters
 * prev_blkadj	: Structure containing the black adjustment towards red,
 *		green, blue.
 */
void
isppreview_config_blkadj(struct ispprev_blkadj prev_blkadj)
{
	omap_writel(prev_blkadj.blue
		| (prev_blkadj.green << ISPPRV_BLKADJOFF_G_SHIFT)
		| (prev_blkadj.red << ISPPRV_BLKADJOFF_R_SHIFT)
		, ISPPRV_BLKADJOFF);
}

/*
 * Configures the RGB-RGB Blending matrix
 * rgb2rgb	: Structure containing the rgb to rgb blending matrix and the
 *		rgb offset.
 */
void
isppreview_config_rgb_blending(struct ispprev_rgbtorgb rgb2rgb)
{
	omap_writel((rgb2rgb.matrix[0][0] << ISPPRV_RGB_MAT1_MTX_RR_SHIFT)
		| (rgb2rgb.matrix[0][1] << ISPPRV_RGB_MAT1_MTX_GR_SHIFT),
		ISPPRV_RGB_MAT1);

	omap_writel((rgb2rgb.matrix[0][2] << ISPPRV_RGB_MAT2_MTX_BR_SHIFT)
		| (rgb2rgb.matrix[1][0] << ISPPRV_RGB_MAT2_MTX_RG_SHIFT),
		ISPPRV_RGB_MAT2);

	omap_writel((rgb2rgb.matrix[1][1] << ISPPRV_RGB_MAT3_MTX_GG_SHIFT)
		| (rgb2rgb.matrix[1][2] << ISPPRV_RGB_MAT3_MTX_BG_SHIFT),
		ISPPRV_RGB_MAT3);

	omap_writel((rgb2rgb.matrix[2][0] << ISPPRV_RGB_MAT4_MTX_RB_SHIFT)
		| (rgb2rgb.matrix[2][1] << ISPPRV_RGB_MAT4_MTX_GB_SHIFT),
		ISPPRV_RGB_MAT4);

	omap_writel((rgb2rgb.matrix[2][2] << ISPPRV_RGB_MAT5_MTX_BB_SHIFT),
		ISPPRV_RGB_MAT5);

	omap_writel((rgb2rgb.offset[0] << ISPPRV_RGB_OFF1_MTX_OFFG_SHIFT)
		| (rgb2rgb.offset[1] << ISPPRV_RGB_OFF1_MTX_OFFR_SHIFT),
		ISPPRV_RGB_OFF1);

	omap_writel(rgb2rgb.offset[2] << ISPPRV_RGB_OFF2_MTX_OFFB_SHIFT,
		ISPPRV_RGB_OFF2);

}

/*
 * Configures the RGB-YCbYCr conversion matrix
 * prev_csc	: Structure containing the RGB to YCbYCr matrix and the
 *		YCbCr offset.
 */
void
isppreview_config_rgb_to_ycbcr(struct ispprev_csc prev_csc)
{
	omap_writel(prev_csc.matrix[0][0] << ISPPRV_CSC0_RY_SHIFT
		| prev_csc.matrix[0][1] << ISPPRV_CSC0_GY_SHIFT
		| prev_csc.matrix[0][2] << ISPPRV_CSC0_BY_SHIFT,
		ISPPRV_CSC0);

	omap_writel(prev_csc.matrix[1][0] << ISPPRV_CSC1_RCB_SHIFT
		| prev_csc.matrix[1][1] << ISPPRV_CSC1_GCB_SHIFT
		| prev_csc.matrix[1][2] << ISPPRV_CSC1_BCB_SHIFT,
		ISPPRV_CSC1);

	omap_writel(prev_csc.matrix[2][0] << ISPPRV_CSC2_RCR_SHIFT
		| prev_csc.matrix[2][1] << ISPPRV_CSC2_GCR_SHIFT
		| prev_csc.matrix[2][2] << ISPPRV_CSC2_BCR_SHIFT,
		ISPPRV_CSC2);

	omap_writel(prev_csc.offset[0] << ISPPRV_CSC_OFFSET_CR_SHIFT
		| prev_csc.offset[1] << ISPPRV_CSC_OFFSET_CB_SHIFT
		| prev_csc.offset[2] << ISPPRV_CSC_OFFSET_Y_SHIFT,
		ISPPRV_CSC_OFFSET);
}


/*
 * Query the contrast.
 * contrast	: Pointer to hold the current programmed contrast value.
 */
void
isppreview_query_contrast(u8 *contrast)
{
	u32 brt_cnt_val = 0;
	brt_cnt_val = omap_readl(ISPPRV_CNT_BRT);
	*contrast = (brt_cnt_val >> ISPPRV_CNT_BRT_CNT_SHIFT) & 0xFF;
	DPRINTK_ISPPREV(" Current brt cnt value in hw is %x\n", brt_cnt_val);
}

/*
 * Updates the contrast.
 * 		Value should be programmed before enabling the module.
 */
void
isppreview_update_contrast(u8 *contrast)
{
	ispprev_obj.contrast = *contrast;
}

/*
 * Configures the Contrast.
 * contrast	: 8bitvalue in U8Q4 format.
 * 		Value should be programmed before enabling the module.
 */
void
isppreview_config_contrast(u8 contrast)
{
	u32 brt_cnt_val = 0;

	brt_cnt_val = omap_readl(ISPPRV_CNT_BRT);
	brt_cnt_val &= ~(0xFF << ISPPRV_CNT_BRT_CNT_SHIFT);
	contrast &=0xFF;
	omap_writel((brt_cnt_val)|(contrast << ISPPRV_CNT_BRT_CNT_SHIFT)
			, ISPPRV_CNT_BRT);
}

/*
 * Gets the range contrast value
 * min_contrast	: Pointer to hold the minimum Contrast value
 * max_contrast	: Pointer to hold the maximum Contrast value
 */
void
isppreview_get_contrast_range( u8 *min_contrast, u8 *max_contrast )
{
	*min_contrast = ISPPRV_CONTRAST_MIN;
	*max_contrast = ISPPRV_CONTRAST_MAX;
}


/*
 * Updates the brightness in the preview module. 
 */
void
isppreview_update_brightness(u8 *brightness)
{
	ispprev_obj.brightness = *brightness;
}

/*
 * Configures the brightness.
 * contrast	: 8bitvalue in U8Q0 format.
 */
void
isppreview_config_brightness(u8 brightness)
{
	u32 brt_cnt_val = 0;
	DPRINTK_ISPPREV("\tConfiguring brightness in ISP: %d\n", brightness);
	brt_cnt_val = omap_readl(ISPPRV_CNT_BRT);
	brt_cnt_val &= ~(0xFF << ISPPRV_CNT_BRT_BRT_SHIFT);
	brightness &= 0xFF;
	omap_writel((brt_cnt_val)|(brightness << ISPPRV_CNT_BRT_BRT_SHIFT)
			, ISPPRV_CNT_BRT);
	
}


/*
 * Query the brightness.
 * brightness	: Pointer to hold the current programmed brightness value.
 */
void
isppreview_query_brightness(u8 *brightness)
{

	*brightness = omap_readl(ISPPRV_CNT_BRT);
}

/*
 * Gets the range brightness value
 * min_brightness	: Pointer to hold the minimum brightness value
 * max_brightness	: Pointer to hold the maximum brightness value
 */
void
isppreview_get_brightness_range( u8 *min_brightness, u8 *max_brightness )
{
	*min_brightness = ISPPRV_BRIGHT_MIN;
	*max_brightness = ISPPRV_BRIGHT_MAX;
}


/**
 * @brief isppreview_set_color -- sets the color effect.
 * @param mode -- indicates the required color effect.
 */
void isppreview_set_color(u8 *mode)
{
	ispprev_obj.color = *mode;
	update_color_matrix = 1;
}

/**
 * @brief isppreview_get_color -- gets the current color effect.
 * @param mode -- indicates the current color effect.
 */
void isppreview_get_color(u8 *mode)
{
	*mode = ispprev_obj.color;
}

/*
 * Configures the max and minim Y and C values.
 * yclimit		: Structure containing the min,max Y,C values.
 */
void
isppreview_config_yc_range( struct ispprev_yclimit yclimit )
{
	omap_writel(((yclimit.maxC << ISPPRV_SETUP_YC_MAXC_SHIFT)
			| (yclimit.maxY << ISPPRV_SETUP_YC_MAXY_SHIFT)
			| (yclimit.minC << ISPPRV_SETUP_YC_MINC_SHIFT)
			| (yclimit.minY << ISPPRV_SETUP_YC_MINY_SHIFT))
			,ISPPRV_SETUP_YC);
}

/*
 * Calculates the number of pixels cropped in the submodules that are enabled,
 * Fills up the output widht height variables in the isp_prev structure .
 * input_w	: input width for the preview in number of pixels per line
 * input_h	: input height for the preview in number of lines
 * output_w	: output width from the preview in number of pixels per line
 * output_h	: output height for the preview in number of lines
*/
void
isppreview_try_size(u32 input_w, u32 input_h, u32 * output_w,
			u32 * output_h)
{
	u32 prevout_w = input_w;
	u32 prevout_h = input_h;
	u32 div =0;
	int max_out;

	ispprev_obj.previn_w = input_w;
	ispprev_obj.previn_h = input_h;

	/*Checks if input size is more than the preview output width limit,
	*else suggests for downsampling in the averager.
	*/
	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0))
		max_out = ISPPRV_MAXOUTPUT_WIDTH;
	else
		max_out = ISPPRV_MAXOUTPUT_WIDTH_ES2;
		
	ispprev_obj.fmtavg = 0;

	if(input_w > max_out){
		div = (input_w/max_out);
		if(div>=2 && div < 4) {
			ispprev_obj.fmtavg = 1;
			prevout_w/=2;
		}else if(div>=4 && div <8){
			ispprev_obj.fmtavg = 2;
			prevout_w/=4;
		}else if(div>=8 ){
			ispprev_obj.fmtavg = 3;
			prevout_w/=8;
		}
	}

	if (ispprev_obj.hmed_en)
		prevout_w -= 4;
	if (ispprev_obj.nf_en){
		prevout_w -= 4;
		prevout_h -= 4;
	}
	if (ispprev_obj.cfa_en) {
		switch (ispprev_obj.cfafmt) {
		case CFAFMT_BAYER:
		case CFAFMT_SONYVGA:
			prevout_w -= 4;
			prevout_h -= 4;
			break;
		case CFAFMT_RGBFOVEON:
		case CFAFMT_RRGGBBFOVEON:
		case CFAFMT_DNSPL:
		case CFAFMT_HONEYCOMB:
			prevout_h -= 2;
			break;
		};
	}
	if ((ispprev_obj.yenh_en) || (ispprev_obj.csup_en))
		prevout_w -= 2;

	/* FMTSPH is always set to be 4 */
	 prevout_w -= 4;
	/* Reserving for now, another 2 extra pixels from Preview to Resizer 
	prevout_w -=2;*/

   /*
    * Make sure that preview always outputs even number of pixels
    */
	if(prevout_w % 2)
		prevout_w -=1;
	
	if(ispprev_obj.prev_outfmt == PREVIEW_MEM){
		if(((prevout_w*2)&ISP_32B_BOUNDARY_OFFSET) != (prevout_w*2))
			prevout_w = ((prevout_w*2)&ISP_32B_BOUNDARY_OFFSET)/2;
	}
	ispprev_obj.prevout_w = *output_w =prevout_w;
	ispprev_obj.prevout_h = *output_h = prevout_h;
}


/*
 * Configures the appropriate values stored in the isp_prev structure to
 * HORZ/VERT_INFO.
 * Configures PRV_AVE if needed for downsampling as calculated in trysize.
 * input_w	: input width for the preview in number of pixels per line
 * input_h	: input height for the preview in number of lines
 * output_w	: output width from the preview in number of pixels per line
 * output_h	: output height for the preview in number of lines
 */
int
isppreview_config_size(u32 input_w, u32 input_h, u32 output_w,
			u32 output_h)
{
	u32 prevsdroff;

	/* Checks if the parameters match the values calculated in the
	* isppreview_try_size(). If not return error.
	*/
	if ((output_w != ispprev_obj.prevout_w)
		|| (output_h != ispprev_obj.prevout_h)){
			printk(KERN_ERR "ISP_ERR : isppreview_try_size should "
				"be called before config size\n");
			return -EINVAL;
	}

	omap_writel((4 << ISPPRV_HORZ_INFO_SPH_SHIFT)
		    | (ispprev_obj.previn_w - 1), ISPPRV_HORZ_INFO);
	omap_writel((0 << ISPPRV_VERT_INFO_SLV_SHIFT)
		    | (ispprev_obj.previn_h - 1), ISPPRV_VERT_INFO);

	if(ispprev_obj.cfafmt == CFAFMT_BAYER)
		omap_writel(ISPPRV_AVE_EVENDIST_2<< ISPPRV_AVE_EVENDIST_SHIFT
			| ISPPRV_AVE_ODDDIST_2<< ISPPRV_AVE_ODDDIST_SHIFT
			| ispprev_obj.fmtavg,
			ISPPRV_AVE);

	/* When written to memory output should be of 32byte boundary */
	if(ispprev_obj.prev_outfmt == PREVIEW_MEM){
		prevsdroff =ispprev_obj.prevout_w*2;
		if((prevsdroff & ISP_32B_BOUNDARY_OFFSET) != prevsdroff){
			DPRINTK_ISPPREV("ISP_WARN : Preview output buffer line "
				"size is truncated to 32byte boundary\n");
			prevsdroff &= ISP_32B_BOUNDARY_BUF ;
		}
		isppreview_config_outlineoffset(prevsdroff);
	}
	return 0;
}

/*
 * Configures the Read address line offset.
 * offset		: Line Offset for the input image.
 */
int
isppreview_config_inlineoffset(u32 offset)
{
	if((offset & ISP_32B_BOUNDARY_OFFSET )== offset)
		omap_writel(offset&0xFFFF, ISPPRV_RADR_OFFSET);
	else{
		printk(KERN_ERR "ISP_ERR : Offset should be in 32 byte "
			"boundary\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * Configures the memory address from which the input frame is to be read.
 * addr		: 32bit memory address aligned on 32byte boundary.
 */
int
isppreview_set_inaddr(u32 addr)
{
	if((addr & ISP_32B_BOUNDARY_BUF )== addr)
		omap_writel(addr, ISPPRV_RSDR_ADDR);
	else{
		printk(KERN_ERR "ISP_ERR : Address should be in 32 byte "
			"boundary\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * Configures the Write address line offset.
 * offset		: Line Offset for the preview output.
 */
int
isppreview_config_outlineoffset(u32 offset)
{
	if((offset & ISP_32B_BOUNDARY_OFFSET )== offset){
		omap_writel(offset&0xFFFF, ISPPRV_WADD_OFFSET);
		}
	else{
		printk(KERN_ERR "ISP_ERR : Offset should be in 32 byte "
			"boundary\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * Configures the memory address to which the output frame is written.
 * addr		: 32bit memory address aligned on 32byte boundary.
 */
int
isppreview_set_outaddr(u32 addr)
{
	if((addr & ISP_32B_BOUNDARY_BUF) == addr){
		omap_writel(addr, ISPPRV_WSDR_ADDR);
	}
	else{
		printk(KERN_ERR "ISP_ERR : Address should be in 32 byte "
			"boundary\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * Configures the Dark frame address line offset.
 * offset		: Line Offset for the Darkframe.
 */
int
isppreview_config_darklineoffset(u32 offset)
{
	if((offset & ISP_32B_BOUNDARY_OFFSET )== offset)
		omap_writel(offset&0xFFFF, ISPPRV_DRKF_OFFSET);
	else{
		printk(KERN_ERR "ISP_ERR : Offset should be in 32 byte "
			"boundary\n");
		return -EINVAL;
	}
	return 0;
}


/*
 * Configures the memory address where the Dark frame should be stored.
 * addr		: 32bit memory address aligned on 32 bit boundary.
 */
int
isppreview_set_darkaddr(u32 addr)
{
	if((addr & ISP_32B_BOUNDARY_BUF )== addr)
		omap_writel(addr, ISPPRV_DSDR_ADDR);
	else{
		printk(KERN_ERR "ISP_ERR : Address should be in 32 byte "
			"boundary\n");
		return -EINVAL;
	}
	return 0;
}


/*
 *
 * Enables the Preview module.
 * Client should configure all the sub modules in Preview before this.
 * enable		: 1- Enables the preview module.
 */
void
isppreview_enable(u8 enable)
{

	if(enable)
		omap_writel((omap_readl(ISPPRV_PCR))
				| ISPPRV_PCR_EN, ISPPRV_PCR);
	else
		omap_writel((omap_readl(ISPPRV_PCR))
				& ~ISPPRV_PCR_EN, ISPPRV_PCR);
}

int
isppreview_busy(void)
{
	return (omap_readl(ISPPRV_PCR) & ISPPRV_PCR_BUSY);
}

struct prev_params *
isppreview_get_config(void)
{
	return prev_config_params;
}

/*
 * Saves the values of the preview module registers.
 */
void 
isppreview_save_context(void)
{
	DPRINTK_ISPPREV (" Saving context\n");
	isp_save_context(ispprev_reg_list);	
}

/*
 * Restores the values of the preview module registers.
 */
void 
isppreview_restore_context(void)
{
	DPRINTK_ISPPREV (" Restoring context\n");
	isp_restore_context(ispprev_reg_list);	
}


/*
 * Prints the values of the Preview Module registers
 * Also prints other debug information stored in the preview moduel
 */
void
isppreview_print_status(void)
{
#ifdef OMAP_ISPPREV_DEBUG
	printk("Module in use =%d\n",ispprev_obj.prev_inuse );
	DPRINTK_ISPPREV("Preview Input format =%d, Output Format =%d\n",
					ispprev_obj.prev_inpfmt, ispprev_obj.prev_outfmt);
	DPRINTK_ISPPREV("Accepted Preview Input (width = %d,Height = %d)\n"
					,ispprev_obj.previn_w,ispprev_obj.previn_h);
	DPRINTK_ISPPREV("Accepted Preview Output (width = %d,Height = %d)\n"
					,ispprev_obj.prevout_w,ispprev_obj.prevout_h);
	DPRINTK_ISPPREV("###ISP_CTRL in preview =0x%x\n", omap_readl(ISP_CTRL));
	DPRINTK_ISPPREV("###ISP_IRQ0ENABLE in preview =0x%x\n", omap_readl(ISP_IRQ0ENABLE));
	DPRINTK_ISPPREV("###ISP_IRQ0STATUS in preview =0x%x\n", omap_readl(ISP_IRQ0STATUS));
	DPRINTK_ISPPREV("###PRV PCR =0x%x\n", omap_readl(ISPPRV_PCR));
	DPRINTK_ISPPREV("###PRV HORZ_INFO =0x%x\n", omap_readl(ISPPRV_HORZ_INFO));
	DPRINTK_ISPPREV("###PRV VERT_INFO =0x%x\n", omap_readl(ISPPRV_VERT_INFO));
	DPRINTK_ISPPREV("###PRV WSDR_ADDR =0x%x\n", omap_readl(ISPPRV_WSDR_ADDR));
	DPRINTK_ISPPREV("###PRV WADD_OFFSET =0x%x\n", omap_readl(ISPPRV_WADD_OFFSET));
	DPRINTK_ISPPREV("###PRV AVE =0x%x\n", omap_readl(ISPPRV_AVE));
	DPRINTK_ISPPREV("###PRV HMED =0x%x\n", omap_readl(ISPPRV_HMED));
	DPRINTK_ISPPREV("###PRV NF =0x%x\n", omap_readl(ISPPRV_NF));
	DPRINTK_ISPPREV("###PRV WB_DGAIN =0x%x\n", omap_readl(ISPPRV_WB_DGAIN));
	DPRINTK_ISPPREV("###PRV WBGAIN =0x%x\n", omap_readl(ISPPRV_WBGAIN));
	DPRINTK_ISPPREV("###PRV WBSEL =0x%x\n", omap_readl(ISPPRV_WBSEL));
	DPRINTK_ISPPREV("###PRV CFA =0x%x\n", omap_readl(ISPPRV_CFA));
	DPRINTK_ISPPREV("###PRV BLKADJOFF =0x%x\n", omap_readl(ISPPRV_BLKADJOFF));
	DPRINTK_ISPPREV("###PRV RGB_MAT1 =0x%x\n", omap_readl(ISPPRV_RGB_MAT1));
	DPRINTK_ISPPREV("###PRV RGB_MAT2 =0x%x\n", omap_readl(ISPPRV_RGB_MAT2));
	DPRINTK_ISPPREV("###PRV RGB_MAT3 =0x%x\n", omap_readl(ISPPRV_RGB_MAT3));
	DPRINTK_ISPPREV("###PRV RGB_MAT4 =0x%x\n", omap_readl(ISPPRV_RGB_MAT4));
	DPRINTK_ISPPREV("###PRV RGB_MAT5 =0x%x\n", omap_readl(ISPPRV_RGB_MAT5));
	DPRINTK_ISPPREV("###PRV RGB_OFF1 =0x%x\n", omap_readl(ISPPRV_RGB_OFF1));
	DPRINTK_ISPPREV("###PRV RGB_OFF2 =0x%x\n", omap_readl(ISPPRV_RGB_OFF2));
	DPRINTK_ISPPREV("###PRV CSC0 =0x%x\n", omap_readl(ISPPRV_CSC0));
	DPRINTK_ISPPREV("###PRV CSC1 =0x%x\n", omap_readl(ISPPRV_CSC1));
	DPRINTK_ISPPREV("###PRV CSC2 =0x%x\n", omap_readl(ISPPRV_CSC2));
	DPRINTK_ISPPREV("###PRV CSC_OFFSET =0x%x\n", omap_readl(ISPPRV_CSC_OFFSET));
	DPRINTK_ISPPREV("###PRV CNT_BRT =0x%x\n", omap_readl(ISPPRV_CNT_BRT));
	DPRINTK_ISPPREV("###PRV CSUP =0x%x\n", omap_readl(ISPPRV_CSUP));
	DPRINTK_ISPPREV("###PRV SETUP_YC =0x%x\n", omap_readl(ISPPRV_SETUP_YC));
#endif
}

/*
 * Module Initialisation.
 */
static int __init
isp_preview_init(void)
{
	struct prev_params *params;

	prev_config_params = kmalloc(sizeof(*prev_config_params), GFP_KERNEL);
	if (prev_config_params == NULL) {
		printk("Can't get memory for isp_preview params!\n");
		return -ENOMEM;
	}
	params = prev_config_params;

	ispprev_obj.prev_inuse = 0;
	init_MUTEX(&(ispprev_obj.semlock));

	if (is_sil_rev_equal_to(OMAP3430_REV_ES2_0)){
		flr_wbal_coef0 = 0x23;
		flr_wbal_coef1 = 0x20;
		flr_wbal_coef2 = 0x20;
		flr_wbal_coef3 = 0x39;
	}

	/* Init values */
	ispprev_obj.color = PREV_DEFAULT_COLOR;
	params->contrast = ispprev_obj.contrast = ISPPRV_CONTRAST_DEF;
	params->brightness = ispprev_obj.brightness = ISPPRV_BRIGHT_DEF;
	params->average = NO_AVE;
	params->lens_shading_shift = 0;
	params->pix_fmt = YCPOS_YCrYCb;
	/* Color Filter Array */
	params->cfa.cfafmt = CFAFMT_BAYER;
	params->cfa.cfa_table = cfa_coef_table;
	params->cfa.cfa_gradthrs_horz = flr_cfa_gradthrs_horz;
	params->cfa.cfa_gradthrs_vert = flr_cfa_gradthrs_vert;
	/* Chroma Suppression */
	params->csup.gain = flr_csup_gain;
	params->csup.thres = flr_csup_thres;
	params->csup.hypf_en = 0;
	/* Lumma Enhancement Table */
	params->ytable = luma_enhance_table;
	/* Noise Filter */
	params->nf.defect_corr_en = 0;
	params->nf.strgth = flr_nf_strgth;
	params->nf.table = noise_filter_table;
	/* Gamma Correction */
	params->gtable.bluetable = bluegamma_table;
	params->gtable.greentable = greengamma_table;
	params->gtable.redtable = redgamma_table;
	/* White Balance */
	params->wbal.dgain = flr_wbal_dgain;
	params->wbal.coef0 = flr_wbal_coef0;
	params->wbal.coef1 = flr_wbal_coef1;
	params->wbal.coef2 = flr_wbal_coef2;
	params->wbal.coef3 = flr_wbal_coef3;
	/* Black Adjustment */	
	params->blk_adj.red = flr_blkadj_red;
	params->blk_adj.green = flr_blkadj_green;
	params->blk_adj.blue = flr_blkadj_blue;
	/* RGB to RGB Blending */
	params->rgb2rgb = flr_rgb2rgb;	
	/* RGB to YCbCr Blending */
	params->rgb2ycbcr = flr_prev_csc[ispprev_obj.color];

	/* Features enabled by default */
	params->features = PREV_CFA | PREV_CHROMA_SUPPRESS | PREV_LUMA_ENHANCE;
	params->features &= ~(PREV_AVERAGER | PREV_INVERSE_ALAW
			      | PREV_HORZ_MEDIAN_FILTER | PREV_NOISE_FILTER
			      | PREV_GAMMA_BYPASS | PREV_DARK_FRAME_SUBTRACT
			      | PREV_LENS_SHADING | PREV_DARK_FRAME_CAPTURE);
	return 0;
}

static void
isp_preview_cleanup(void)
{
	kfree(prev_config_params);
	prev_config_params = NULL;
}

module_init(isp_preview_init);
module_exit(isp_preview_cleanup);

EXPORT_SYMBOL(isppreview_config_shadow_registers);
EXPORT_SYMBOL(isppreview_request);
EXPORT_SYMBOL(isppreview_free);
EXPORT_SYMBOL(isppreview_busy);
EXPORT_SYMBOL(isppreview_get_config);
EXPORT_SYMBOL(isppreview_config_datapath);
EXPORT_SYMBOL(isppreview_config_ycpos);
EXPORT_SYMBOL(isppreview_config_averager);
EXPORT_SYMBOL(isppreview_enable_invalaw);
EXPORT_SYMBOL(isppreview_enable_drkframe);
EXPORT_SYMBOL(isppreview_enable_shadcomp);
EXPORT_SYMBOL(isppreview_config_drkf_shadcomp);
EXPORT_SYMBOL(isppreview_enable_hmed);
EXPORT_SYMBOL(isppreview_config_hmed);
EXPORT_SYMBOL(isppreview_config_noisefilter);
EXPORT_SYMBOL(isppreview_config_cfa);
EXPORT_SYMBOL(isppreview_config_gammacorrn);
EXPORT_SYMBOL(isppreview_config_luma_enhancement);
EXPORT_SYMBOL(isppreview_config_chroma_suppression);
EXPORT_SYMBOL(isppreview_enable_noisefilter);
EXPORT_SYMBOL(isppreview_enable_cfa);
EXPORT_SYMBOL(isppreview_enable_gammabypass);
EXPORT_SYMBOL(isppreview_enable_luma_enhancement);
EXPORT_SYMBOL(isppreview_enable_chroma_suppression);
EXPORT_SYMBOL(isppreview_config_whitebalance);
EXPORT_SYMBOL(isppreview_config_whitebalance2);
EXPORT_SYMBOL(isppreview_config_blkadj);
EXPORT_SYMBOL(isppreview_config_rgb_blending);
EXPORT_SYMBOL(isppreview_config_rgb_to_ycbcr);
EXPORT_SYMBOL(isppreview_config_contrast);
EXPORT_SYMBOL(isppreview_get_contrast_range);
EXPORT_SYMBOL(isppreview_config_brightness);
EXPORT_SYMBOL(isppreview_get_brightness_range);
EXPORT_SYMBOL(isppreview_set_color);
EXPORT_SYMBOL(isppreview_get_color);
EXPORT_SYMBOL(isppreview_config_yc_range);
EXPORT_SYMBOL(isppreview_try_size);
EXPORT_SYMBOL(isppreview_config_size);
EXPORT_SYMBOL(isppreview_config_inlineoffset);
EXPORT_SYMBOL(isppreview_set_inaddr);
EXPORT_SYMBOL(isppreview_config_outlineoffset);
EXPORT_SYMBOL(isppreview_set_outaddr);
EXPORT_SYMBOL(isppreview_config_darklineoffset);
EXPORT_SYMBOL(isppreview_set_darkaddr);
EXPORT_SYMBOL(isppreview_enable);
EXPORT_SYMBOL(isppreview_save_context);
EXPORT_SYMBOL(isppreview_restore_context);
EXPORT_SYMBOL(isppreview_print_status);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP Preview Library");
MODULE_LICENSE("GPL");
