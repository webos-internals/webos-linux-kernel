/*
 * drivers/media/video/omap/isp/ispccdc.c
 *
 * Driver Library for CCDC module in TI's OMAP3430 Camera ISP
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
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <asm/mach-types.h>
#include <asm/arch/clock.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/scatterlist.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"
#ifndef CONFIG_ARCH_OMAP3410
	#include "isppreview.h"
#endif
#include "ispmmu.h"

/*
 * Structure for the CCDC module to store its own information.
 */
static struct isp_ccdc {
	u8 ccdc_inuse;
	u32 ccdcout_w;
	u32 ccdcout_h;
	u32 ccdcin_w;
	u32 ccdcin_h;
	u32 ccdcin_woffset;
	u32 ccdcin_hoffset;
	u32 crop_w;
	u32 crop_h;
	u8 ccdc_inpfmt;
	u8 ccdc_outfmt;
	u8 vpout_en;
	u8 wen;
	u8 exwen;
	u8 refmt_en;
	u8 ccdcslave;
	u8 syncif_ipmod;
	u8 obclamp_en;
	struct semaphore semlock;
} ispccdc_obj;

/* Structure for saving/restoring CCDC module registers*/
static struct isp_reg ispccdc_reg_list[]= {
	{ISPCCDC_SYN_MODE, 0x0000},
	{ISPCCDC_HD_VD_WID, 0x0000},
	{ISPCCDC_PIX_LINES, 0x0000},
	{ISPCCDC_HORZ_INFO, 0x0000},
	{ISPCCDC_VERT_START, 0x0000},
	{ISPCCDC_VERT_LINES, 0x0000},
	{ISPCCDC_CULLING, 0x0000}, 
	{ISPCCDC_HSIZE_OFF, 0x0000},
	{ISPCCDC_SDOFST, 0x0000},
	{ISPCCDC_SDR_ADDR, 0x0000},
	{ISPCCDC_CLAMP, 0x0000},
	{ISPCCDC_DCSUB, 0x0000},
	{ISPCCDC_COLPTN, 0x0000},
	{ISPCCDC_BLKCMP, 0x0000},		
	{ISPCCDC_FPC, 0x0000},
	{ISPCCDC_FPC_ADDR, 0x0000},
	{ISPCCDC_VDINT, 0x0000},
	{ISPCCDC_ALAW, 0x0000},
	{ISPCCDC_REC656IF, 0x0000},
	{ISPCCDC_CFG, 0x0000},
	{ISPCCDC_FMTCFG, 0x0000},
	{ISPCCDC_FMT_HORZ, 0x0000},
	{ISPCCDC_FMT_VERT, 0x0000},
	{ISPCCDC_FMT_ADDR0, 0x0000},
	{ISPCCDC_FMT_ADDR1, 0x0000},
	{ISPCCDC_FMT_ADDR2, 0x0000},
	{ISPCCDC_FMT_ADDR3, 0x0000},
	{ISPCCDC_FMT_ADDR4, 0x0000},
	{ISPCCDC_FMT_ADDR5, 0x0000},
	{ISPCCDC_FMT_ADDR6, 0x0000},
	{ISPCCDC_FMT_ADDR7, 0x0000},
	{ISPCCDC_PRGEVEN0, 0x0000},
	{ISPCCDC_PRGEVEN1, 0x0000},
	{ISPCCDC_PRGODD0, 0x0000},
	{ISPCCDC_PRGODD1, 0x0000},
	{ISPCCDC_VP_OUT, 0x0000},
	{ISPCCDC_LSC_CONFIG, 0x0000},
	{ISPCCDC_LSC_INITIAL, 0x0000},
	{ISPCCDC_LSC_TABLE_BASE, 0x0000},
	{ISPCCDC_LSC_TABLE_OFFSET, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};

/*
 * LSC gain table
 */
const u8 ispccdc_lsc_tbl[] = {
	#include "ispccd_lsc.dat"
};
  

/*
 * Reserve the CCDC module.
 * Only one user at a time.
 */
int
ispccdc_request(void)
{
	down(&(ispccdc_obj.semlock));
	if (!(ispccdc_obj.ccdc_inuse)) {
		ispccdc_obj.ccdc_inuse = 1;
		up(&(ispccdc_obj.semlock));
		/* Turn on CCDC module Clocks. */
		omap_writel((omap_readl(ISP_CTRL)) | ISPCTRL_CCDC_RAM_EN
			|ISPCTRL_CCDC_CLK_EN|ISPCTRL_SBL_WR1_RAM_EN, ISP_CTRL);
		/* VDLC = 1 is a must if CCDC to be used */
		omap_writel((omap_readl(ISPCCDC_CFG)) | ISPCCDC_CFG_VDLC
					,ISPCCDC_CFG);
		return 0;
	} else{
		up(&(ispccdc_obj.semlock));
		DPRINTK_ISPCCDC("ISP_ERR : CCDC Module Busy");
		return -EBUSY;
	}
}

/*
 * Marks CCDC module free.
 */
int
ispccdc_free(void)
{
	down(&(ispccdc_obj.semlock));
	if (ispccdc_obj.ccdc_inuse){
		ispccdc_obj.ccdc_inuse = 0;
		up(&(ispccdc_obj.semlock));
		/* Turn off CCDC module Clocks. */
		omap_writel((omap_readl(ISP_CTRL))& ~(ISPCTRL_CCDC_CLK_EN
				|ISPCTRL_CCDC_RAM_EN
				| ISPCTRL_SBL_WR1_RAM_EN), ISP_CTRL);
		return 0;
	}
	else{
		up(&(ispccdc_obj.semlock));
		DPRINTK_ISPCCDC("ISP_ERR : CCDC Module already freed");
		return -EINVAL;
	}
}

void 
ispccdc_config_crop(u32 left, u32 top, u32 height, u32 width)
{

/* The following restrictions are applied for the crop settings. If incoming
 *  values do not follow these restrictions then we map the settings to the closest 
 *  acceptable crop value.
 * 1) Left offset is always odd. This can be avoided if we enable byte swap
 *    option for incoming data into CCDC.
 * 2) Height offset is always even.
 * 3) Crop width is always a multiple of 16 pixels
 * 4) Crop height is always even.
 */

	ispccdc_obj.ccdcin_woffset = left + ((left+1)%2);
	ispccdc_obj.ccdcin_hoffset = top + (top % 2);


	ispccdc_obj.crop_w = width - (width % 16);
	ispccdc_obj.crop_h = height + (height % 2);

	DPRINTK_ISPCCDC("\n\tOffsets L %d T %d W %d H %d\n", 
		ispccdc_obj.ccdcin_woffset, ispccdc_obj.ccdcin_hoffset,ispccdc_obj.crop_w, ispccdc_obj.crop_h);
	
}
/* Sets up the default CCDC configuration according to the arguments.
 * input		: Indicates the module that gives the image to CCDC
 * output		: Indicates the module to which the CCDC outputs to.
 */
int
ispccdc_config_datapath(enum ccdc_input input,
				enum ccdc_output output)
{
	u32 syn_mode = 0;
	struct ispccdc_vp vpcfg;
	struct ispccdc_syncif syncif;
	struct ispccdc_bclamp blkcfg;
	/* Color pattern is
	Gr   R   Gr   R   Gr   R ...
	B   Gb . B   Gb   B   Gb.....
	Gr   R   Gr   R   Gr   R ...
	B   Gb . B   Gb   B   Gb.....
	*/
	u32 colptn = ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC0_SHIFT
		| ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP0PLC1_SHIFT
		| ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC2_SHIFT
		|ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP0PLC3_SHIFT
		| ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP1PLC0_SHIFT
		| ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP1PLC1_SHIFT
		| ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP1PLC2_SHIFT
		| ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP1PLC3_SHIFT
		| ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC0_SHIFT
		| ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP2PLC1_SHIFT
		| ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC2_SHIFT
		| ISPCCDC_COLPTN_R_Ye << ISPCCDC_COLPTN_CP2PLC3_SHIFT
		| ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP3PLC0_SHIFT
		| ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP3PLC1_SHIFT
		| ISPCCDC_COLPTN_B_Mg << ISPCCDC_COLPTN_CP3PLC2_SHIFT
		| ISPCCDC_COLPTN_Gb_G << ISPCCDC_COLPTN_CP3PLC3_SHIFT;

	/* CCDC does not convert the image format */
	if (((input == CCDC_RAW) || (input == CCDC_OTHERS))
		&& (output == CCDC_YUV_RSZ)){
		DPRINTK_ISPCCDC("ISP_ERR : Wrong CCDC i/p,o/p Combination");
		return -EINVAL;
	}

	syn_mode = omap_readl(ISPCCDC_SYN_MODE);

	switch (output) {
	case CCDC_YUV_RSZ:
		syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode &= ~ISPCCDC_SYN_MODE_WEN;
		break;

	case CCDC_YUV_MEM_RSZ:
		syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
		ispccdc_obj.wen = 1;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		break;

	case CCDC_OTHERS_VP:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode &= ~ISPCCDC_SYN_MODE_WEN;
		/* Video Port Configuration */
		vpcfg.bitshift_sel = BIT9_0;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(vpcfg);
		ispccdc_enable_vp(1);
		break;

	case CCDC_OTHERS_MEM:
		syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		/*Generally cam_wen is used with cam_hs, vs signals */
		syn_mode |= ISPCCDC_SYN_MODE_EXWEN;
		omap_writel((omap_readl(ISPCCDC_CFG))
				| ISPCCDC_CFG_WENLOG, ISPCCDC_CFG);
		break;

	case CCDC_OTHERS_VP_MEM:
		syn_mode |= ISPCCDC_SYN_MODE_VP2SDR;
		syn_mode |= ISPCCDC_SYN_MODE_WEN;
		//Generally cam_wen is used with cam_hs, vs signals */
		syn_mode |= ISPCCDC_SYN_MODE_EXWEN;
		omap_writel((omap_readl(ISPCCDC_CFG))
				| ISPCCDC_CFG_WENLOG, ISPCCDC_CFG);
		/* Video Port Configuration */
		vpcfg.bitshift_sel = BIT9_0;
		vpcfg.freq_sel = PIXCLKBY2;
		ispccdc_config_vp(vpcfg);
		ispccdc_enable_vp(1);
		break;
	default:
		DPRINTK_ISPCCDC("ISP_ERR : Wrong CCDC Input");
		return -EINVAL;
	};

	omap_writel(syn_mode, ISPCCDC_SYN_MODE);

	switch (input) {
	case CCDC_RAW:
		/* Slave mode */
		syncif.ccdc_mastermode = 0;
		/* Normal */
		syncif.datapol = 0;
		syncif.datsz = DAT10;
		/* Progressive Mode */
		syncif.fldmode = 0;
		/* Input */
		syncif.fldout = 0;
		/* Positive */
		syncif.fldpol = 0;
		/* Odd Field */
		syncif.fldstat = 0;
		/*Positive */
		syncif.hdpol = 0;
		syncif.ipmod = RAW;
		/* Positive */
		syncif.vdpol = 0;
		ispccdc_config_sync_if(syncif);
		ispccdc_config_imgattr(colptn);
		blkcfg.dcsubval = 42;
		ispccdc_config_black_clamp(blkcfg);
		break;
	case CCDC_YUV_SYNC:
		/* Slave mode */
		syncif.ccdc_mastermode = 0;
		/* Normal */
		syncif.datapol = 0;
		syncif.datsz = DAT8;
		/* Progressive Mode */
		syncif.fldmode = 0;
		/* Input */
		syncif.fldout = 0;
		/* Positive */
		syncif.fldpol = 0;
		/* Odd Field */
		syncif.fldstat = 0;
		/*Positive */
		syncif.hdpol = 0;
		syncif.ipmod = YUV16;
		/*Positive */
		syncif.vdpol = 0;
		ispccdc_config_imgattr(0);
		ispccdc_config_sync_if(syncif);
		blkcfg.dcsubval = 0;
		ispccdc_config_black_clamp(blkcfg);

		break;
	case CCDC_YUV_BT:
		break;
	case CCDC_OTHERS:
		break;
	default:
		DPRINTK_ISPCCDC("ISP_ERR : Wrong CCDC Input");
		return -EINVAL;
	}

	ispccdc_obj.ccdc_inpfmt = input;
	ispccdc_obj.ccdc_outfmt = output;
	return 0;
}

/*
 * Configures the sync interface parameters between the sensor and the CCDC.
 * syncif		: Structure containing the sync parameters like field state,
 		CCDC in master/slave mode, raw/yuv data, polarity of data,
 		field, hs, vs signals.
 */
void
ispccdc_config_sync_if(struct ispccdc_syncif syncif)
{
	u32 syn_mode = omap_readl(ISPCCDC_SYN_MODE);

	syn_mode |= ISPCCDC_SYN_MODE_VDHDEN;

	if (syncif.fldstat)
		syn_mode |= ISPCCDC_SYN_MODE_FLDSTAT;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDSTAT;

	syn_mode &= ISPCCDC_SYN_MODE_INPMOD_MASK;
	ispccdc_obj.syncif_ipmod = syncif.ipmod;

	switch (syncif.ipmod) {
	case RAW:
		break;
	case YUV16:
		syn_mode |= ISPCCDC_SYN_MODE_INPMOD_YCBCR16;
		break;
	case YUV8:
		syn_mode |= ISPCCDC_SYN_MODE_INPMOD_YCBCR8;
		break;
	};

	syn_mode &= ISPCCDC_SYN_MODE_DATSIZ_MASK;
	switch (syncif.datsz) {
	case DAT8:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_8;
		break;
	case DAT10:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_10;
		break;
	case DAT11:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_11;
		break;
	case DAT12:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_12;
		break;
	};

	if (syncif.fldmode)
		/*Interlaced mode*/
		syn_mode |= ISPCCDC_SYN_MODE_FLDMODE;
	else
		/*Progressive mode */
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDMODE;

	if (syncif.datapol)
		/*One's complement */
		syn_mode |= ISPCCDC_SYN_MODE_DATAPOL;
	else
		/*Normal */
		syn_mode &= ~ISPCCDC_SYN_MODE_DATAPOL;

	if (syncif.fldpol)
		/*Negative */
		syn_mode |= ISPCCDC_SYN_MODE_FLDPOL;
	else
		/*Positive */
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDPOL;

	if (syncif.hdpol)
		/*Negative */
		syn_mode |= ISPCCDC_SYN_MODE_HDPOL;
	else
		/*Positive */
		syn_mode &= ~ISPCCDC_SYN_MODE_HDPOL;

	if (syncif.vdpol)
		/*Negative */
		syn_mode |= ISPCCDC_SYN_MODE_VDPOL;
	else
		/*Positive */
		syn_mode &= ~ISPCCDC_SYN_MODE_VDPOL;

	if (syncif.ccdc_mastermode) {
		/*fld, hd, vd are output signals in master mode */
		syn_mode |= ISPCCDC_SYN_MODE_FLDOUT
			| ISPCCDC_SYN_MODE_VDHDOUT;
		omap_writel(syncif.hs_width << ISPCCDC_HD_VD_WID_HDW_SHIFT
			| syncif.vs_width << ISPCCDC_HD_VD_WID_VDW_SHIFT,
			ISPCCDC_HD_VD_WID);

		/*Pixel per line, half line per frame are used
		* along with HS/VS as output
		*/
		omap_writel(syncif.ppln << ISPCCDC_PIX_LINES_PPLN_SHIFT
			| syncif.hlprf << ISPCCDC_PIX_LINES_HLPRF_SHIFT,
			ISPCCDC_PIX_LINES);
	} else
		/*fld, hd,vd input signals in slave mode */
		syn_mode &= ~(ISPCCDC_SYN_MODE_FLDOUT
			| ISPCCDC_SYN_MODE_VDHDOUT);

	omap_writel(syn_mode, ISPCCDC_SYN_MODE);

	if(!(syncif.bt_r656_en))
		omap_writel((omap_readl(ISPCCDC_REC656IF))
			& (~ISPCCDC_REC656IF_R656ON),ISPCCDC_REC656IF);
}

/*
 * Configures the optical/digital black clamp parameters in CCDC.
 * bclamp	: Structure containing the optical black average gain,
 *		optical black sample length, sample lines, and the start pixel
 *		position of the samples w.r.t the HS pulse .
 */
int
ispccdc_config_black_clamp(struct ispccdc_bclamp bclamp)
{
	u32 bclamp_val =0;
	if(ispccdc_obj.obclamp_en){
		bclamp_val |= bclamp.obgain<<ISPCCDC_CLAMP_OBGAIN_SHIFT;
		bclamp_val |= bclamp.oblen <<ISPCCDC_CLAMP_OBSLEN_SHIFT;
		bclamp_val |= bclamp.oblines<<ISPCCDC_CLAMP_OBSLN_SHIFT;
		bclamp_val |= bclamp.obstpixel<<ISPCCDC_CLAMP_OBST_SHIFT;
		omap_writel(bclamp_val, ISPCCDC_CLAMP);
	} else {
		/*
		* HW Errata 1.39. Camera ISP: DC substract not supported for
		* YUV 8bit and ITU656 
		*/
		if(is_sil_rev_less_than(OMAP3430_REV_ES2_0))
			if((ispccdc_obj.syncif_ipmod == YUV16) ||
			(ispccdc_obj.syncif_ipmod == YUV8) ||
			((omap_readl(ISPCCDC_REC656IF) & ISPCCDC_REC656IF_R656ON)
			== ISPCCDC_REC656IF_R656ON))
				bclamp.dcsubval = 0;
		omap_writel(bclamp.dcsubval, ISPCCDC_DCSUB);
	}
	return 0;
}

/*
 * Enables  the optical or Digital black clamp.
 * enable	:	: 1- Enables Optical Black clamp
 *		0 - Enables Digital Black clamp.
 */
void
ispccdc_enable_black_clamp(u8 enable)
{
	if (enable){
		omap_writel((omap_readl(ISPCCDC_CLAMP))|ISPCCDC_CLAMP_CLAMPEN
				, ISPCCDC_CLAMP);
		ispccdc_obj.obclamp_en = 1;
	}
	else{
		omap_writel((omap_readl(ISPCCDC_CLAMP))
			& (~ISPCCDC_CLAMP_CLAMPEN),ISPCCDC_CLAMP);
		ispccdc_obj.obclamp_en = 0;
	}
}

/*
 * Configures the Faulty Pixel Correction parameters.
 * fpc		: Structure containing the number of faulty pixels corrected
 *		in the frame, address of the FPC table.
 */
int
ispccdc_config_fpc( struct ispccdc_fpc fpc)
{
	u32 fpc_val = 0;

	fpc_val = omap_readl(ISPCCDC_FPC);

	if((fpc.fpcaddr & 0xFFFFFFC0) == fpc.fpcaddr){
		/*Make sure that FPC is disabled*/
		omap_writel(fpc_val&(~ISPCCDC_FPC_FPCEN), ISPCCDC_FPC);
		omap_writel(fpc.fpcaddr, ISPCCDC_FPC_ADDR);
	}
	else{
		DPRINTK_ISPCCDC("FPC Address should be on 64byte boundary\n");
		return -EINVAL;
	}
	/*Retain the FPC Enable bit along with the configuration*/
	omap_writel(fpc_val|(fpc.fpnum<<ISPCCDC_FPC_FPNUM_SHIFT)
			, ISPCCDC_FPC);
	return 0;
}

/*
 * Enables  the Faulty Pixel Correction.
 * enable	:	: 1- Enables FPC
 */
void
ispccdc_enable_fpc(u8 enable)
{
	if(enable)
		omap_writel((omap_readl(ISPCCDC_FPC))|ISPCCDC_FPC_FPCEN
			, ISPCCDC_FPC);
	else
		omap_writel((omap_readl(ISPCCDC_FPC))
			&(~ISPCCDC_FPC_FPCEN), ISPCCDC_FPC);
}

/*
 * Configures the Black Level Compensation parameters.
 * blcomp	: Structure containing the black level compensation value
 *		for RGrGbB pixels. in 2's complement.
 */
void
ispccdc_config_black_comp( struct ispccdc_blcomp blcomp )
{
	u32 blcomp_val = 0;
	blcomp_val |= blcomp.b_mg << ISPCCDC_BLKCMP_B_MG_SHIFT;
	blcomp_val |= blcomp.gb_g << ISPCCDC_BLKCMP_GB_G_SHIFT;
	blcomp_val |= blcomp.gr_cy << ISPCCDC_BLKCMP_GR_CY_SHIFT;
	blcomp_val |= blcomp.r_ye << ISPCCDC_BLKCMP_R_YE_SHIFT;

	omap_writel(blcomp_val, ISPCCDC_BLKCMP);
}

/*
 * Configures the Video Port Configuration parameters.
 * vpcfg		: Structure containing the Video Port input frequency,
 *		and the 10 bit format.
 */
void
ispccdc_config_vp(struct ispccdc_vp vpcfg)
{
	u32 fmtcfg_vp = omap_readl(ISPCCDC_FMTCFG);

	/*Clear the existing values */
	fmtcfg_vp &= ISPCCDC_FMTCFG_VPIN_MASK &
		ISPCCDC_FMTCF_VPIF_FRQ_MASK;

	switch (vpcfg.bitshift_sel) {
	case BIT9_0:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_9_0;
		break;
	case BIT10_1:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_10_1;
		break;
	case BIT11_2:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_11_2;
		break;
	case BIT12_3:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_12_3;
		break;
	};
	switch (vpcfg.freq_sel) {
	case PIXCLKBY2:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY2;
		break;
	case PIXCLKBY3_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY3;
		break;
	case PIXCLKBY4_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY4;
		break;
	case PIXCLKBY5_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY5;
		break;
	case PIXCLKBY6_5:
		fmtcfg_vp |= ISPCCDC_FMTCF_VPIF_FRQ_BY6;
		break;
	};
	omap_writel(fmtcfg_vp, ISPCCDC_FMTCFG);
}

/*
 * Enables  the Video Port.
 * enable	:	: 1- Enables VP
 */
void
ispccdc_enable_vp(u8 enable)
{
	if (enable)
		omap_writel((omap_readl(ISPCCDC_FMTCFG))
			| ISPCCDC_FMTCFG_VPEN,ISPCCDC_FMTCFG);
	else
		omap_writel((omap_readl(ISPCCDC_FMTCFG))
			& (~ISPCCDC_FMTCFG_VPEN),ISPCCDC_FMTCFG);
}

/*
 * Configures the Reformatter register values if line alternating is disabled.
 * else just enabling the line alternating is enough.
 * refmt	:	: Structure containing the memory address to format and
 *		the bit fields for the reformatter registers.
 */
void
ispccdc_config_reformatter( struct ispccdc_refmt refmt )
{
	u32 fmtcfg_val = 0;

	fmtcfg_val = omap_readl(ISPCCDC_FMTCFG);

	if(refmt.lnalt)
		fmtcfg_val |=  ISPCCDC_FMTCFG_LNALT;
	else{
		fmtcfg_val &= ~ISPCCDC_FMTCFG_LNALT;
		/*Clear fields of lnum plen_even/odd*/
		fmtcfg_val &= 0xFFFFF003;
		fmtcfg_val |= refmt.lnum << ISPCCDC_FMTCFG_LNUM_SHIFT;
		fmtcfg_val |= refmt.plen_even << ISPCCDC_FMTCFG_PLEN_EVEN_SHIFT;
		fmtcfg_val |= refmt.plen_odd << ISPCCDC_FMTCFG_PLEN_ODD_SHIFT;

		/*The arguments have the proper caluclated addresses
		* and bit fields for the reformatter configuration*/
		omap_writel(refmt.prgeven0, ISPCCDC_PRGEVEN0);
		omap_writel(refmt.prgeven1, ISPCCDC_PRGEVEN1);
		omap_writel(refmt.prgodd0, ISPCCDC_PRGODD0);
		omap_writel(refmt.prgodd1, ISPCCDC_PRGODD1);
		omap_writel(refmt.fmtaddr0, ISPCCDC_FMT_ADDR0);
		omap_writel(refmt.fmtaddr1, ISPCCDC_FMT_ADDR1);
		omap_writel(refmt.fmtaddr2, ISPCCDC_FMT_ADDR2);
		omap_writel(refmt.fmtaddr3, ISPCCDC_FMT_ADDR3);
		omap_writel(refmt.fmtaddr4, ISPCCDC_FMT_ADDR4);
		omap_writel(refmt.fmtaddr5, ISPCCDC_FMT_ADDR5);
		omap_writel(refmt.fmtaddr6, ISPCCDC_FMT_ADDR6);
		omap_writel(refmt.fmtaddr7, ISPCCDC_FMT_ADDR7);
	}
	omap_writel(fmtcfg_val, ISPCCDC_FMTCFG);
}

/*
 * Enables  the Reformatter
 * enable	:	: 1- Enables Data Reformatter
 */
void
ispccdc_enable_reformatter(u8 enable)
{
	if(enable){
		omap_writel((omap_readl(ISPCCDC_FMTCFG))
			|ISPCCDC_FMTCFG_FMTEN, ISPCCDC_FMTCFG);
		ispccdc_obj.refmt_en = 1;
	}
	else{
		omap_writel((omap_readl(ISPCCDC_FMTCFG))
			& ~ISPCCDC_FMTCFG_FMTEN, ISPCCDC_FMTCFG);
		ispccdc_obj.refmt_en = 0;
	}
}

/*
 * Configures the Culling parameters.
 * cull	:	: Structure containing the vertical culling pattern,
 * 		and horizontal culling pattern for odd and even lines.
 */
void
ispccdc_config_culling(struct ispccdc_culling cull)
{
	u32 culling_val = 0;

	culling_val |= cull.v_pattern<<ISPCCDC_CULLING_CULV_SHIFT;
	culling_val |= cull.h_even << ISPCCDC_CULLING_CULHEVN_SHIFT;
	culling_val |= cull.h_odd << ISPCCDC_CULLING_CULHODD_SHIFT;

	omap_writel(culling_val, ISPCCDC_CULLING);
}

/*
 * Enables  the Low pass Filter
 * enable	:	: 1- Enables LPF
 */
void
ispccdc_enable_lpf(u8 enable)
{
	if(enable)
		omap_writel((omap_readl(ISPCCDC_SYN_MODE))
			|ISPCCDC_SYN_MODE_LPF,ISPCCDC_SYN_MODE);
	else
		omap_writel((omap_readl(ISPCCDC_SYN_MODE))
			&(~ISPCCDC_SYN_MODE_LPF),ISPCCDC_SYN_MODE);
}

/*
 * Configures the input width for A-law.
 * ipwidth	: Input width for ALaw
 */
void
ispccdc_config_alaw(enum alaw_ipwidth ipwidth)
{
	omap_writel(ipwidth << ISPCCDC_ALAW_GWDI_SHIFT, ISPCCDC_ALAW);
}

/*
 * Enables  the A-law compression
 * enable	:	: 1- Enables A-Law
 */
void
ispccdc_enable_alaw(u8 enable)
{
	if(enable)
		omap_writel((omap_readl(ISPCCDC_ALAW))
			|ISPCCDC_ALAW_CCDTBL,ISPCCDC_ALAW);
	else
		omap_writel((omap_readl(ISPCCDC_ALAW))
			&~ISPCCDC_ALAW_CCDTBL,ISPCCDC_ALAW);
}

/*
 * Load lens shading table
 */
int  
ispccdc_load_lsc (void)
{
	unsigned long ispmmu_addr;
	u8 *gain_table;
	
	/* Disable LSC module*/
	omap_writel(omap_readl(ISPCCDC_LSC_CONFIG) & 0xFE, ISPCCDC_LSC_CONFIG);
	
	gain_table = (u8 *)kmalloc(sizeof(ispccdc_lsc_tbl), GFP_KERNEL|GFP_DMA);
	if(!gain_table){
		return -ENOMEM;
	}
	
	memcpy (gain_table, ispccdc_lsc_tbl, sizeof(ispccdc_lsc_tbl));
	ispmmu_addr = ispmmu_map(virt_to_phys(gain_table), 
					sizeof(ispccdc_lsc_tbl));
	
	omap_writel(ispmmu_addr , ISPCCDC_LSC_TABLE_BASE);
	
	return 0;
}

/*
 * Configures the lens shading compensation module
 * lsc_cfg	: LSC configuration structure
 */
void 
ispccdc_config_lsc (struct ispccdc_lsc_config * lsc_cfg)
{
	int reg;
	
	/* Disable LSC module*/
	omap_writel(omap_readl(ISPCCDC_LSC_CONFIG) & 0xFE, ISPCCDC_LSC_CONFIG);
		
	omap_writel(lsc_cfg->offset, ISPCCDC_LSC_TABLE_OFFSET);

	reg = omap_readl(ISPCCDC_LSC_CONFIG); 
	reg &= ~ISPCCDC_LSC_GAIN_MODE_N_MASK; 
	reg |= (lsc_cfg->gain_mode_n << ISPCCDC_LSC_GAIN_MODE_N_SHIFT);
    	reg &= ~ISPCCDC_LSC_GAIN_MODE_M_MASK;
	reg |= (lsc_cfg->gain_mode_m << ISPCCDC_LSC_GAIN_MODE_M_SHIFT);
    	reg &= ~ISPCCDC_LSC_GAIN_FORMAT_MASK;
	reg |= (lsc_cfg->gain_format << ISPCCDC_LSC_GAIN_FORMAT_SHIFT);
    	reg &= ~ISPCCDC_LSC_AFTER_REFORMATTER_MASK;
	omap_writel(reg , ISPCCDC_LSC_CONFIG);
		
    	reg = omap_readl(ISPCCDC_FMT_HORZ);   
	reg &= ~ISPCCDC_FMT_HORZ_FMTSPH_MASK;
	reg |= (lsc_cfg->fmtsph << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT);
    	reg &= ~ISPCCDC_FMT_HORZ_FMTLNH_MASK;
	reg |= (lsc_cfg->fmtlnh << ISPCCDC_FMT_HORZ_FMTLNH_SHIFT);
	omap_writel(reg , ISPCCDC_FMT_HORZ);
		
	reg = omap_readl(ISPCCDC_FMT_VERT);    
	reg &= ~ISPCCDC_FMT_VERT_FMTSLV_MASK; 
	reg |= (lsc_cfg->fmtslv << ISPCCDC_FMT_VERT_FMTSLV_SHIFT);
    	reg &= ~ISPCCDC_FMT_VERT_FMTLNV_MASK; 
    	reg |= (lsc_cfg->fmtlnv << ISPCCDC_FMT_VERT_FMTLNV_SHIFT);
	omap_writel(reg , ISPCCDC_FMT_VERT);
	  
	reg = omap_readl(ISPCCDC_LSC_INITIAL);    
	reg &= ~ISPCCDC_LSC_INITIAL_X_MASK;
	reg |= (lsc_cfg->initial_x << ISPCCDC_LSC_INITIAL_X_SHIFT);
    	reg &= ~ISPCCDC_LSC_INITIAL_Y_MASK;
	reg |= (lsc_cfg->initial_y << ISPCCDC_LSC_INITIAL_Y_SHIFT);
    	omap_writel(reg , ISPCCDC_LSC_INITIAL);
    	    	
}
    
#ifndef CONFIG_ARCH_OMAP3410

/*
 * Enables  lens shading compensation module
 * enable	:0 - Disable LSC	: 1- Enables LSC
 */
void 
ispccdc_enable_lsc(u8 enable)
{
	int reg;
	if (enable)
	{
		reg = omap_readl(ISP_CTRL);
		omap_writel(omap_readl(ISP_CTRL) | ISPCTRL_SBL_SHARED_RPORTB, ISP_CTRL);
		omap_writel(omap_readl(ISPCCDC_LSC_CONFIG) | 0x01, ISPCCDC_LSC_CONFIG);
		isppreview_enable_shadcomp(1);
	} else {
		isppreview_enable_shadcomp(0);
		omap_writel(omap_readl(ISPCCDC_LSC_CONFIG) & 0xFE, ISPCCDC_LSC_CONFIG);
		omap_writel(omap_readl(ISP_CTRL) & ~ISPCTRL_SBL_SHARED_RPORTB, ISP_CTRL);
	}
}
#else
void 
ispccdc_enable_lsc(u8 enable) {}
#endif
/*
 * Configures the sensor image specific attribute.
 * colptn		: Color pattern of the sensor.
 */
void
ispccdc_config_imgattr(u32 colptn)
{
	omap_writel(colptn, ISPCCDC_COLPTN);
}

/*
 * Programs the shadow registers associated with CCDC.
 */
void
ispccdc_config_shadow_registers()
{
	return;
}

/*
 * Calculates the number of pixels cropped if the reformater is disabled,
 * Fills up the output widht height variables in the isp_ccdc structure .
 * input_w	: input width for the CCDC in number of pixels per line
 * input_h	: input height for the CCDC in number of lines
 * output_w	: output width from the CCDC in number of pixels per line
 * output_h	: output height for the CCDC in number of lines
*/
int
ispccdc_try_size(u32 input_w, u32 input_h, u32 * output_w,
			u32 * output_h)
{
/*
 * CCDC cannot handle less than 2 pixels for input.
 */
	if(input_w < 2) {
		DPRINTK_ISPCCDC("ISP_ERR: CCDC cannot handle input width less than 2 pixels\n");
		return -EINVAL;
	}

/*
 * If crop settings are issued then output size from CCDC
 * will be equal to the crop window specified.
 */

	if(ispccdc_obj.crop_w)
		*output_w = ispccdc_obj.crop_w;
	else
		*output_w = input_w;

	if(ispccdc_obj.crop_h)
		*output_h = ispccdc_obj.crop_h;
	else
		*output_h = input_h;

	if((!ispccdc_obj.refmt_en) && (ispccdc_obj.ccdc_outfmt !=CCDC_OTHERS_MEM))
		*output_h -= 1;

	ispccdc_obj.ccdcout_w = *output_w;
	ispccdc_obj.ccdcout_h = *output_h;
	ispccdc_obj.ccdcin_w = input_w;
	ispccdc_obj.ccdcin_h = input_h;

	DPRINTK_ISPCCDC("try size: ccdcin_w=%u,ccdcin_h=%u,ccdcout_w=%u, ccdcout_h=%u\n",
		ispccdc_obj.ccdcin_w, ispccdc_obj.ccdcin_h,
		ispccdc_obj.ccdcout_w, ispccdc_obj.ccdcout_h);

	return 0;
}

/*
 * Configures the appropriate values stored in the isp_ccdc structure to
 * HORZ/VERT_INFO registers and the VP_OUT depending on whether the image
 * is stored in memory or given to the another module in the ISP pipeline.
 * input_w	: input width for the CCDC  in number of pixels per line
 * input_h	: input height for the CCDC in number of lines
 * output_w	: output width from the CCDC in number of pixels per line
 * output_h	: output height for the CCDC in number of lines
 */
int
ispccdc_config_size(u32 input_w, u32 input_h, u32 output_w,
			u32 output_h)
{

	DPRINTK_ISPCCDC("config size: input_w=%u,input_h=%u,output_w=%u,output_h=%u\n",
		input_w, input_h, output_w, output_h);
	if ((output_w != ispccdc_obj.ccdcout_w)
		|| (output_h != ispccdc_obj.ccdcout_h)){
			DPRINTK_ISPCCDC("ISP_ERR : ispccdc_try_size should \
				be called before config size\n");
			return -EINVAL;
	}

	if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_VP) {
		/* Start with 1 pixel apart */
		omap_writel((ispccdc_obj.ccdcin_woffset << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT)
			| (ispccdc_obj.ccdcin_w
			<< ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
			ISPCCDC_FMT_HORZ);

		omap_writel((ispccdc_obj.ccdcin_hoffset << ISPCCDC_FMT_VERT_FMTSLV_SHIFT)
			| ((ispccdc_obj.ccdcin_h + 1)
			<< ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
			ISPCCDC_FMT_VERT);

		omap_writel((ispccdc_obj.ccdcout_w
			<< ISPCCDC_VP_OUT_HORZ_NUM_SHIFT)
			| (ispccdc_obj.ccdcout_h
			<< ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
			ISPCCDC_VP_OUT);
		omap_writel((((ispccdc_obj.ccdcout_h - 1) &  ISPCCDC_VDINT_0_MASK)
			     << ISPCCDC_VDINT_0_SHIFT)
			    | (((ispccdc_obj.ccdcout_h - 1) &  ISPCCDC_VDINT_1_MASK)						     << ISPCCDC_VDINT_1_SHIFT), ISPCCDC_VDINT);

	}
	else if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_MEM) {
		omap_writel(0 << ISPCCDC_HORZ_INFO_SPH_SHIFT
			    | ((ispccdc_obj.ccdcout_w - 1)
			<< ISPCCDC_HORZ_INFO_NPH_SHIFT),
			ISPCCDC_HORZ_INFO);
		omap_writel(0 << ISPCCDC_VERT_START_SLV0_SHIFT,
			ISPCCDC_VERT_START);
		omap_writel((ispccdc_obj.ccdcout_h - 1)
			<< ISPCCDC_VERT_LINES_NLV_SHIFT,
			ISPCCDC_VERT_LINES);
		/*Configure the HSIZE_OFF with output buffer width */
		ispccdc_config_outlineoffset(ispccdc_obj.ccdcout_w*2, 0, 0);
		omap_writel((((ispccdc_obj.ccdcout_h - 1) &  ISPCCDC_VDINT_0_MASK)
			     << ISPCCDC_VDINT_0_SHIFT)
			    | (((50 ) &  ISPCCDC_VDINT_1_MASK)  
			     << ISPCCDC_VDINT_1_SHIFT), ISPCCDC_VDINT);
	}
	else if (ispccdc_obj.ccdc_outfmt == CCDC_OTHERS_VP_MEM){
		/* Start with 1 pixel apart */
		omap_writel((1 << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT)
			| (ispccdc_obj.ccdcin_w
			<< ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
			ISPCCDC_FMT_HORZ);

		omap_writel((0 << ISPCCDC_FMT_VERT_FMTSLV_SHIFT)
			| ((ispccdc_obj.ccdcin_h + 1)
			<< ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
			ISPCCDC_FMT_VERT);

		omap_writel((ispccdc_obj.ccdcout_w
			<< ISPCCDC_VP_OUT_HORZ_NUM_SHIFT)
			| (ispccdc_obj.ccdcout_h
			<< ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
			ISPCCDC_VP_OUT);
		omap_writel(0 << ISPCCDC_HORZ_INFO_SPH_SHIFT
			    | ((ispccdc_obj.ccdcout_w - 1)
			<< ISPCCDC_HORZ_INFO_NPH_SHIFT),
			ISPCCDC_HORZ_INFO);
		omap_writel(0 << ISPCCDC_VERT_START_SLV0_SHIFT,
			ISPCCDC_VERT_START);
		omap_writel((ispccdc_obj.ccdcout_h - 1)
			<< ISPCCDC_VERT_LINES_NLV_SHIFT,
			ISPCCDC_VERT_LINES);
		/*Configure the HSIZE_OFF with output buffer width*/
		ispccdc_config_outlineoffset(ispccdc_obj.ccdcout_w*2, 0, 0);
		omap_writel((((ispccdc_obj.ccdcout_h - 1) &  ISPCCDC_VDINT_0_MASK)
			     << ISPCCDC_VDINT_0_SHIFT)
			    | (((ispccdc_obj.ccdcout_h - 1) &  ISPCCDC_VDINT_1_MASK)  
			     << ISPCCDC_VDINT_1_SHIFT), ISPCCDC_VDINT);

	}
	return 0;
}

/*
 * Configures the output line offset when stored in memory.
 * Configures the num of even and odd line fields in case of rearranging
 * the lines
 * offset 		: twice the Output width and aligned on 32byte boundary.
 * oddeven	: odd/even line pattern to be chosen to store the output
 * numlines	: Configure the value 0-3 for +1-4lines, 4-7 for -1-4lines
 */
int
ispccdc_config_outlineoffset(u32 offset, u8 oddeven, u8 numlines)
{


	/*Make sure offset is multiple of 32bytes. ie last 5bits should be zero*/
	if((offset & ISP_32B_BOUNDARY_OFFSET )== offset)
		omap_writel((offset&0xFFFF), ISPCCDC_HSIZE_OFF);
	else{
		DPRINTK_ISPCCDC("ISP_ERR : Offset should be in 32 byte \
			boundary");
		return -EINVAL;
	}

	/*0 - By default Donot inverse the field identification */
	omap_writel((omap_readl(ISPCCDC_SDOFST) & (~ISPCCDC_SDOFST_FINV)),
			ISPCCDC_SDOFST);

	/*0 - By default one line offset*/
	omap_writel(omap_readl(ISPCCDC_SDOFST) & ISPCCDC_SDOFST_FOFST_1L,
			ISPCCDC_SDOFST);

	switch (oddeven) {
	case EVENEVEN:		/*even lines even fields*/
		omap_writel((omap_readl(ISPCCDC_SDOFST) )|
			((numlines & 0x7) << ISPCCDC_SDOFST_LOFST0_SHIFT)
			, ISPCCDC_SDOFST);
		break;
	case ODDEVEN:		/*odd lines even fields*/
		omap_writel((omap_readl(ISPCCDC_SDOFST) )|
			((numlines & 0x7) << ISPCCDC_SDOFST_LOFST1_SHIFT)
			, ISPCCDC_SDOFST);
		break;
	case EVENODD:		/*even lines odd fields*/
		omap_writel((omap_readl(ISPCCDC_SDOFST)) |
			((numlines & 0x7) << ISPCCDC_SDOFST_LOFST2_SHIFT)
			, ISPCCDC_SDOFST);
		break;
	case ODDODD:		/*odd lines odd fields*/
		omap_writel((omap_readl(ISPCCDC_SDOFST)) |
			((numlines & 0x7) << ISPCCDC_SDOFST_LOFST3_SHIFT)
			, ISPCCDC_SDOFST);
		break;
	default:
		break;
	}
	return 0;
}

/*
 * Configures the memory address where the output should be stored.
 * addr		: 32bit memory address aligned on 32 bit boundary.
 */
int
ispccdc_set_outaddr(u32 addr)
{
	if((addr & ISP_32B_BOUNDARY_BUF )== addr){
		omap_writel(addr, ISPCCDC_SDR_ADDR);
		return 0;
	}
	else{
		DPRINTK_ISPCCDC("ISP_ERR : Address should be in 32 byte \
			boundary");
		return -EINVAL;
	}

}

/*
 *
 * Enables the CCDC module.
 * Client should configure all the sub modules in CCDC before this.
 * enable		: 1- Enables the preview module.
 */
void
ispccdc_enable(u8 enable)
{
	/* Before enabling ccdc we need to clear IRQ status */
  //  omap_writel(omap_readl(ISP_IRQ0STATUS)| ISP_INT_CLR, ISP_IRQ0STATUS);


	if (enable) {
		omap_writel(omap_readl(ISPCCDC_PCR) | (ISPCCDC_PCR_EN),
			ISPCCDC_PCR);
  }
	else
		omap_writel(omap_readl(ISPCCDC_PCR) & ~(ISPCCDC_PCR_EN),
			ISPCCDC_PCR);


}

int ispccdc_busy(void)
{
	return (omap_readl(ISPCCDC_PCR) & ISPCCDC_PCR_BUSY);
}

/*
 * Saves the values of the CCDC module registers.
 */
void 
ispccdc_save_context(void)
{
	DPRINTK_ISPCCDC (" Saving context \n");
	isp_save_context(ispccdc_reg_list);	

}

/*
 * Restores the values of the CCDC module registers.
 */
void 
ispccdc_restore_context(void)
{
	DPRINTK_ISPCCDC (" Restoring context\n");
	isp_restore_context(ispccdc_reg_list);
}

/*
 * Prints the values of the CCDC Module registers
 * Also prints other debug information stored in the CCDC module
 */
void
ispccdc_print_status(void)
{
#ifdef	OMAP_ISPCCDC_DEBUG
	DPRINTK_ISPCCDC("Module in use =%d\n",ispccdc_obj.ccdc_inuse );;
	DPRINTK_ISPCCDC("Accepted CCDC Input (width = %d,Height = %d)\n"
				,ispccdc_obj.ccdcin_w,ispccdc_obj.ccdcin_h);
	DPRINTK_ISPCCDC("Accepted CCDC Output (width = %d,Height = %d)\n"
				,ispccdc_obj.ccdcout_w,ispccdc_obj.ccdcout_h);

	DPRINTK_ISPCCDC("###CCDC PCR=0x%x\n", omap_readl(ISPCCDC_PCR));
	DPRINTK_ISPCCDC("ISP_CTRL =0x%x\n",omap_readl(ISP_CTRL));
	switch (ispccdc_obj.ccdc_inpfmt) {
	case CCDC_RAW:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_RAW\n");
		break;
	case CCDC_YUV_SYNC:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_YUV_SYNC\n");
		break;
	case CCDC_YUV_BT:
		DPRINTK_ISPCCDC("ccdc input format is CCDC_YUV_BT\n");
		break;

	}
	switch (ispccdc_obj.ccdc_outfmt) {
	case CCDC_OTHERS_VP:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_OTHERS_VP\n");
		break;
	case CCDC_OTHERS_MEM:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_OTHERS_MEM\n");
		break;
	case CCDC_YUV_RSZ:
		DPRINTK_ISPCCDC("ccdc output format is CCDC_YUV_RSZ\n");
		break;
	}
	DPRINTK_ISPCCDC("###ISP_CTRL in ccdc =0x%x\n", omap_readl(ISP_CTRL));
	DPRINTK_ISPCCDC("###ISP_IRQ0ENABLE in ccdc =0x%x\n", omap_readl(ISP_IRQ0ENABLE));
	DPRINTK_ISPCCDC("###ISP_IRQ0STATUS in ccdc =0x%x\n", omap_readl(ISP_IRQ0STATUS));
	DPRINTK_ISPCCDC("###CCDC SYN_MODE=0x%x\n", omap_readl(ISPCCDC_SYN_MODE));
	DPRINTK_ISPCCDC("###CCDC HORZ_INFO=0x%x\n", omap_readl(ISPCCDC_HORZ_INFO));
	DPRINTK_ISPCCDC("###CCDC VERT_START=0x%x\n", omap_readl(ISPCCDC_VERT_START));
	DPRINTK_ISPCCDC("###CCDC VERT_LINES=0x%x\n", omap_readl(ISPCCDC_VERT_LINES));
	DPRINTK_ISPCCDC("###CCDC CULLING=0x%x\n", omap_readl(ISPCCDC_CULLING));
	DPRINTK_ISPCCDC("###CCDC HSIZE_OFF=0x%x\n", omap_readl(ISPCCDC_HSIZE_OFF));
	DPRINTK_ISPCCDC("###CCDC SDOFST=0x%x\n", omap_readl(ISPCCDC_SDOFST));
	DPRINTK_ISPCCDC("###CCDC SDR_ADDR=0x%x\n", omap_readl(ISPCCDC_SDR_ADDR));
	DPRINTK_ISPCCDC("###CCDC CLAMP=0x%x\n", omap_readl(ISPCCDC_CLAMP));
	DPRINTK_ISPCCDC("###CCDC COLPTN=0x%x\n", omap_readl(ISPCCDC_COLPTN));
	DPRINTK_ISPCCDC("###CCDC CFG=0x%x\n", omap_readl(ISPCCDC_CFG));
	DPRINTK_ISPCCDC("###CCDC VP_OUT=0x%x\n", omap_readl(ISPCCDC_VP_OUT));
	DPRINTK_ISPCCDC("###CCDC_SDR_ADDR= 0x%x\n", omap_readl(ISPCCDC_SDR_ADDR));
	DPRINTK_ISPCCDC("###CCDC FMTCFG=0x%x\n", omap_readl(ISPCCDC_FMTCFG));
	DPRINTK_ISPCCDC("###CCDC FMT_HORZ=0x%x\n", omap_readl(ISPCCDC_FMT_HORZ));
	DPRINTK_ISPCCDC("###CCDC FMT_VERT=0x%x\n", omap_readl(ISPCCDC_FMT_VERT));
#endif
}

/*
 * Module Initialisation.
 */
static int __init
isp_ccdc_init(void)
{
	ispccdc_obj.ccdc_inuse = 0;
	ispccdc_config_crop(0,0,0,0);
	init_MUTEX(&(ispccdc_obj.semlock));
	return 0;
}

static void
isp_ccdc_cleanup(void)
{
/* Nothing to do */
}

module_init(isp_ccdc_init);
module_exit(isp_ccdc_cleanup);

EXPORT_SYMBOL(ispccdc_config_shadow_registers);
EXPORT_SYMBOL(ispccdc_request);
EXPORT_SYMBOL(ispccdc_free);
EXPORT_SYMBOL(ispccdc_config_datapath);
EXPORT_SYMBOL(ispccdc_config_sync_if);
EXPORT_SYMBOL(ispccdc_config_black_clamp);
EXPORT_SYMBOL(ispccdc_enable_black_clamp);
EXPORT_SYMBOL(ispccdc_config_fpc);
EXPORT_SYMBOL(ispccdc_enable_fpc);
EXPORT_SYMBOL(ispccdc_config_black_comp);
EXPORT_SYMBOL(ispccdc_config_vp);
EXPORT_SYMBOL(ispccdc_enable_vp);
EXPORT_SYMBOL(ispccdc_config_reformatter);
EXPORT_SYMBOL(ispccdc_enable_reformatter);
EXPORT_SYMBOL(ispccdc_config_culling);
EXPORT_SYMBOL(ispccdc_enable_lpf);
EXPORT_SYMBOL(ispccdc_config_alaw);
EXPORT_SYMBOL(ispccdc_enable_alaw);
EXPORT_SYMBOL(ispccdc_load_lsc);
EXPORT_SYMBOL(ispccdc_config_lsc);
EXPORT_SYMBOL(ispccdc_enable_lsc);
EXPORT_SYMBOL(ispccdc_config_imgattr);
EXPORT_SYMBOL(ispccdc_try_size);
EXPORT_SYMBOL(ispccdc_config_size);
EXPORT_SYMBOL(ispccdc_config_outlineoffset);
EXPORT_SYMBOL(ispccdc_set_outaddr);
EXPORT_SYMBOL(ispccdc_enable);
EXPORT_SYMBOL(ispccdc_print_status);
EXPORT_SYMBOL(ispccdc_save_context);
EXPORT_SYMBOL(ispccdc_restore_context);



MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP CCDC Library");
MODULE_LICENSE("GPL");
