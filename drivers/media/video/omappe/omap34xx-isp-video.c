/*
 * drivers/media/video/omap/omap34xx-isp-video.c
 *
 * Copyright (C) Palm, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/delay.h>
#include <linux/version.h>
#include <asm/arch/isp.h>
#include "omap34xx.h"
#include "omap34xx-isp.h"

#define ISP_BASE		0x480BC000
#define ISP_REG(off)		(ISP_BASE + (off))

#define ISP_CTRL		ISP_REG(0x40)
#define	PREV_RAM_EN_MASK	(0x1 << 17)
#define	PREV_RAM_EN(val)	((0x1 & (val)) << 17)
#define	CCDC_RAM_EN_MASK	(0x1 << 16)
#define	CCDC_RAM_EN(val)	((0x1 & (val)) << 16)
#define	RSZ_CLK_EN_MASK		(0x1 << 13)
#define	RSZ_CLK_EN(val)		((0x1 & (val)) << 13)
#define	PRV_CLK_EN_MASK		(0x1 << 12)
#define	PRV_CLK_EN(val)		((0x1 & (val)) << 12)
#define	CCDC_CLK_EN_MASK	(0x1 << 8)
#define	CCDC_CLK_EN(val)	((0x1 & (val)) << 8)

#define ISP_IRQ0ENABLE		ISP_REG(0x0C)
#define ISP_IRQ0STATUS		ISP_REG(0x10)
#define	HS_VS_IRQ_MASK		(0x1 << 31)
#define	RSZ_DONE_IRQ_MASK	(0x1 << 24)
#define	PRV_DONE_IRQ_MASK	(0x1 << 20)
#define	CCDC_VD0_IRQ_MASK	(0x1 << 8)

#define CCDC_BASE		0x480BC600
#define CCDC_REG(off)		(CCDC_BASE + (off))

#define CCDC_PCR		CCDC_REG(0x04)
#define BUSY_MASK		(0x1 << 1)
#define ENABLE_MASK		(0x1 << 0)

#define CCDC_SYN_MODE		CCDC_REG(0x08)
#define WEN_MASK		(0x1 << 17)
#define	WEN(val)		((0x1 & (val)) << 17)

#define CCDC_HORZ_INFO		CCDC_REG(0x14)
#define NPH_MASK		(0x7FFF << 0)
#define	NPH(val)		((0x7FFF & (val)) << 0)

#define CCDC_VERT_LINES		CCDC_REG(0x1C)
#define NLV_MASK		(0x7FFF << 0)
#define	NLV(val)		((0x7FFF & (val)) << 0)

#define CCDC_HSIZE_OFF		CCDC_REG(0x24)
#define LNOFST_MASK		(0xFFFF << 0)
#define	LNOFST(val)		((0xFFFF & (val)) << 0)

#define CCDC_SDR_ADDR		CCDC_REG(0x2C)
#define ADDR_MASK		0xFFFFFFD0
#define ADDR(val)		(0xFFFFFFD0 & (val))
#define ADDR_ALIGN		256

#define CCDC_CLAMP		CCDC_REG(0x30)
#define CLAMPEN_MASK		(0x1 << 31)
#define CLAMPEN(val)		((0x1 & (val)) << 31)
#define OBSLEN_MASK		(0x7 << 28)
#define OBSLEN(val)		((0x7 & (val)) << 28)
#define OBSLN_MASK		(0x7 << 25)
#define OBSLN(val)		((0x7 & (val)) << 25)
#define OBST_MASK		(0x7FFF << 10)
#define OBST(val)		((0x7FFF & (val)) << 10)
#define OBGAIN_MASK		(0x1F << 0)
#define OBGAIN(val)		((0x1F & (val)) << 0)

#define CCDC_DCSUB		CCDC_REG(0x34)
#define DCSUB_MASK		(0x3FFF << 0)
#define DCSUB(val)		((0x3FFF & (val)) << 0)

#define CCDC_COLPTN		CCDC_REG(0x38)
#define	CPLx(l, val)		((0xFF & (val)) << (8 * (l)))
#define	CPPx(p, val)		((0x3 & (val)) << (2 * (p)))
#define CPLxPx_RED		0x0
#define CPLxPx_GREENR		0x1
#define CPLxPx_GREENB		0x2
#define CPLxPx_BLUE		0x3
#define CPLx_GR			(CPPx(0, CPLxPx_GREENR) \
					| CPPx(1, CPLxPx_RED) \
					| CPPx(2, CPLxPx_GREENR) \
					| CPPx(3, CPLxPx_RED))
#define CPLx_BG			(CPPx(0, CPLxPx_BLUE) \
					| CPPx(1, CPLxPx_GREENB) \
					| CPPx(2, CPLxPx_BLUE) \
					| CPPx(3, CPLxPx_GREENB))
#define COLPTN_GRBG		(CPLx(0, CPLx_GR) \
					| CPLx(1, CPLx_BG) \
					| CPLx(2, CPLx_GR) \
					| CPLx(3, CPLx_BG))

#define CCDC_BLKCMP		CCDC_REG(0x3C)
#define R_YE_MASK		(0xFF << 24)
#define R_YE(val)		((0xFF & (val)) << 24)
#define GR_CY_MASK		(0xFF << 16)
#define GR_CY(val)		((0xFF & (val)) << 16)
#define GB_G_MASK		(0xFF << 8)
#define GB_G(val)		((0xFF & (val)) << 8)
#define B_MG_MASK		(0xFF << 0)
#define B_MG(val)		((0xFF & (val)) << 0)

#define CCDC_VDINT		CCDC_REG(0x48)
#define VDINT0_MASK		(0x7FFF << 16)
#define	VDINT0(val)		((0x7FFF & (val)) << 16)

#define CCDC_FMTCFG		CCDC_REG(0x58)
#define VPEN_MASK		(0x1 << 15)
#define	VPEN(val)		((0x1 & (val)) << 15)

#define CCDC_FMT_HORZ		CCDC_REG(0x5C)
#define FMTSPH_MASK		(0x1FFF << 16)
#define	FMTSPH(val)		((0x1FFF & (val)) << 16)
#define FMTLNH_MASK		(0x1FFF << 0)
#define	FMTLNH(val)		((0x1FFF & (val)) << 0)

#define CCDC_FMT_VERT		CCDC_REG(0x60)
#define FMTLNV_MASK		(0x1FFF << 0)
#define	FMTLNV(val)		((0x1FFF & (val)) << 0)

#define CCDC_VP_OUT		CCDC_REG(0x94)
#define VERT_NUM_MASK		(0x3FFF << 17)
#define	VERT_NUM(val)		((0x3FFF & (val)) << 17)
#define HORZ_NUM_MASK		(0x1FFF << 4)
#define	HORZ_NUM(val)		((0x1FFF & (val)) << 4)

#define H3A_BASE		0x480BCC00
#define H3A_REG(off)		(H3A_BASE + (off))

#define H3A_PCR			H3A_REG(0x04)
#define	RGBPOS_MASK		(0x7 << 11)
#define	RGBPOS(val)		((0x7 & (val)) << 11)
#define RGBPOS_GRBG		0x2

#define PRV_BASE		0x480BCE00
#define PRV_REG(off)		(PRV_BASE + (off))

#define PRV_PCR			PRV_REG(0x04)
#define DCOR_METHOD_MASK	(0x1 << 28)
#define	DCOR_METHOD(val)	((0x1 & (val)) << 28)
#define DCOREN_MASK		(0x1 << 27)
#define	DCOREN(val)		((0x1 & (val)) << 27)
#define GAMMA_BYPASS_MASK	(0x1 << 26)
#define	GAMMA_BYPASS(val)	((0x1 & (val)) << 26)
#define SDRPORT_MASK		(0x1 << 20)
#define	SDRPORT(val)		((0x1 & (val)) << 20)
#define RSZPORT_MASK		(0x1 << 19)
#define	RSZPORT(val)		((0x1 & (val)) << 19)
#define PCR_YCPOS_MASK		(0x3 << 17)
#define	PCR_YCPOS(val)		((0x3 & (val)) << 17)
#define PCR_YCPOS_YCRYCB	0x0
#define PCR_YCPOS_CRYCBY	0x3
#define SUPEN_MASK		(0x1 << 16)
#define	SUPEN(val)		((0x1 & (val)) << 16)
#define YNENHEN_MASK		(0x1 << 15)
#define	YNENHEN(val)		((0x1 & (val)) << 15)
#define CFAEN_MASK		(0x1 << 10)
#define	CFAEN(val)		((0x1 & (val)) << 10)
#define NFEN_MASK		(0x1 << 9)
#define	NFEN(val)		((0x1 & (val)) << 9)
#define PRV_PCR_ONESHOT_MASK	(0x1 << 3)

#define PRV_HORZ_INFO		PRV_REG(0x08)
#define SPH_MASK		(0x3FFF << 16)
#define	SPH(val)		((0x3FFF & (val)) << 16)
#define EPH_MASK		(0x3FFF << 0)
#define	EPH(val)		((0x3FFF & (val)) << 0)

#define PRV_VERT_INFO		PRV_REG(0x0C)
#define ELV_MASK		(0x3FFF << 0)
#define	ELV(val)		((0x3FFF & (val)) << 0)

#define PRV_WSDR_ADDR		PRV_REG(0x20)

#define PRV_WADD_OFFSET		PRV_REG(0x24)
#define OFFSET_MASK		(0xFFFF << 0)
#define OFFSET(val)		((0xFFFF & (val)) << 0)

#define PRV_AVE			PRV_REG(0x28)
#define ODDDIST_MASK		(0x3 << 4)
#define ODDDIST(val)		((0x3 & (val)) << 4)
#define EVENDIST_MASK		(0x3 << 2)
#define EVENDIST(val)		((0x3 & (val)) << 2)

#define PRV_NF			PRV_REG(0x30)
#define SPR_MASK		(0x3 << 0)
#define	SPR(val)		((0x3 & (val)) << 0)

#define PRV_WB_DGAIN		PRV_REG(0x34)
#define DGAIN_MASK		(0x3FF << 0)
#define	DGAIN(val)		((0x3FF & (val)) << 0)

#define PRV_WBGAIN		PRV_REG(0x38)
#define WBGAIN_COEFx_MASK(x)	(0xFF << (8 * (x)))
#define	WBGAIN_COEFx(x, val)	((0xFF & (val)) << (8 * (x)))

#define PRV_WBSEL		PRV_REG(0x3C)
#define	NPx(p, val)		((0x3 & (val)) << (2 * (p)))
#define	NLx(l, val)		((0xFF & (val)) << (8 * (l)))
#define NLxPx_GREENR		0x0
#define NLxPx_RED		0x1
#define NLxPx_BLUE		0x2
#define NLxPx_GREENB		0x3
#define NLx_GR			(NPx(0, NLxPx_GREENR) \
					| NPx(1, NLxPx_RED) \
					| NPx(2, NLxPx_GREENR) \
					| NPx(3, NLxPx_RED))
#define NLx_BG			(NPx(0, NLxPx_BLUE) \
					| NPx(1, NLxPx_GREENB) \
					| NPx(2, NLxPx_BLUE) \
					| NPx(3, NLxPx_GREENB))
#define WBSEL_GRBG		(NLx(0, NLx_GR) \
					| NLx(1, NLx_BG) \
					| NLx(2, NLx_GR) \
					| NLx(3, NLx_BG))

#define PRV_CFA			PRV_REG(0x40)
#define GRADTH_VER_MASK		(0xFF << 8)
#define	GRADTH_VER(val)		((0xFF & (val)) << 8)
#define GRADTH_HOR_MASK		(0xFF << 0)
#define	GRADTH_HOR(val)		((0xFF & (val)) << 0)

#define PRV_BLKADJOFF		PRV_REG(0x44)
#define R_MASK			(0xFF << 16)
#define	R(val)			((0xFF & (val)) << 16)
#define G_MASK			(0xFF << 8)
#define	G(val)			((0xFF & (val)) << 8)
#define B_MASK			(0xFF << 0)
#define	B(val)			((0xFF & (val)) << 0)

#define PRV_RGB_MAT1		PRV_REG(0x48)
#define MTX_GR_MASK		(0xFFF << 16)
#define	MTX_GR(val)		((0xFFF & (val)) << 16)
#define MTX_RR_MASK		(0xFFF << 0)
#define	MTX_RR(val)		((0xFFF & (val)) << 0)

#define PRV_RGB_MAT2		PRV_REG(0x4C)
#define MTX_RG_MASK		(0xFFF << 16)
#define	MTX_RG(val)		((0xFFF & (val)) << 16)
#define MTX_BR_MASK		(0xFFF << 0)
#define	MTX_BR(val)		((0xFFF & (val)) << 0)

#define PRV_RGB_MAT3		PRV_REG(0x50)
#define MTX_BG_MASK		(0xFFF << 16)
#define	MTX_BG(val)		((0xFFF & (val)) << 16)
#define MTX_GG_MASK		(0xFFF << 0)
#define	MTX_GG(val)		((0xFFF & (val)) << 0)

#define PRV_RGB_MAT4		PRV_REG(0x54)
#define MTX_GB_MASK		(0xFFF << 16)
#define	MTX_GB(val)		((0xFFF & (val)) << 16)
#define MTX_RB_MASK		(0xFFF << 0)
#define	MTX_RB(val)		((0xFFF & (val)) << 0)

#define PRV_RGB_MAT5		PRV_REG(0x58)
#define MTX_BB_MASK		(0xFFF << 0)
#define	MTX_BB(val)		((0xFFF & (val)) << 0)

#define PRV_CSC0		PRV_REG(0x64)
#define CSCBY_MASK		(0x3FF << 20)
#define	CSCBY(val)		((0x3FF & (val)) << 20)
#define CSCGY_MASK		(0x3FF << 10)
#define	CSCGY(val)		((0x3FF & (val)) << 10)
#define CSCRY_MASK		(0x3FF << 0)
#define	CSCRY(val)		((0x3FF & (val)) << 0)

#define PRV_CSC1		PRV_REG(0x68)
#define CSCBCB_MASK		(0x3FF << 20)
#define	CSCBCB(val)		((0x3FF & (val)) << 20)
#define CSCGCB_MASK		(0x3FF << 10)
#define	CSCGCB(val)		((0x3FF & (val)) << 10)
#define CSCRCB_MASK		(0x3FF << 0)
#define	CSCRCB(val)		((0x3FF & (val)) << 0)

#define PRV_CSC2		PRV_REG(0x6C)
#define CSCBCR_MASK		(0x3FF << 20)
#define	CSCBCR(val)		((0x3FF & (val)) << 20)
#define CSCGCR_MASK		(0x3FF << 10)
#define	CSCGCR(val)		((0x3FF & (val)) << 10)
#define CSCRCR_MASK		(0x3FF << 0)
#define	CSCRCR(val)		((0x3FF & (val)) << 0)

#define PRV_CSC_OFFSET		PRV_REG(0x70)
#define CSC_YOFST_MASK		(0xFF << 16)
#define	CSC_YOFST(val)		((0xFF & (val)) << 16)
#define CSC_OFSTCB_MASK		(0xFF << 8)
#define	CSC_OFSTCB(val)		((0xFF & (val)) << 8)
#define CSC_OFSTCR_MASK		(0xFF << 0)
#define	CSC_OFSTCR(val)		((0xFF & (val)) << 0)

#define PRV_CNT_BRT		PRV_REG(0x74)
#define CNT_MASK		(0xFF << 8)
#define	CNT(val)		((0xFF & (val)) << 8)
#define BRT_MASK		(0xFF << 0)
#define	BRT(val)		((0xFF & (val)) << 0)

#define PRV_SET_TBL_ADDR	PRV_REG(0x80)
#define	TBL_ADDR(val)		((0x1FFF & (val)) << 0)
#define RGAMMA_TBL_START	TBL_ADDR(0x0000)
#define GGAMMA_TBL_START	TBL_ADDR(0x0400)
#define BGAMMA_TBL_START	TBL_ADDR(0x0800)
#define NF_TBL_START		TBL_ADDR(0x0C00)
#define YNENH_TBL_START		TBL_ADDR(0x1000)
#define CFA_TBL_START		TBL_ADDR(0x1400)

#define PRV_SET_TBL_DATA	PRV_REG(0x84)
#define	TBL_DATA(val)		((0xFFFFF & (val)) << 0)

#define PRV_CDC_THRx(x)		PRV_REG(0x90 + (4 * (x)))
#define	CORRECT_MASK		(0x3FF << 16)
#define	CORRECT(val)		((0x3FF & (val)) << 16)
#define	DETECT_MASK		(0x3FF << 0)
#define	DETECT(val)		((0x3FF & (val)) << 0)

#define RSZ_BASE		0x480BD000
#define RSZ_REG(off)		(RSZ_BASE + (off))

#define RSZ_PCR			RSZ_REG(0x04)
#define RSZ_PCR_ONESHOT_MASK	(0x1 << 2)

#define RSZ_CNT			RSZ_REG(0x08)
#define CNT_YCPOS_MASK		(0x1 << 26)
#define	CNT_YCPOS(val)		((0x1 & (val)) << 26)
#define CNT_YCPOS_YC		0x0
#define CNT_YCPOS_CY		0x1
#define VSTPH_MASK		(0x7 << 23)
#define	VSTPH(val)		((0x7 & (val)) << 23)
#define HSTPH_MASK		(0x7 << 20)
#define	HSTPH(val)		((0x7 & (val)) << 20)
#define VRSZ_MASK		(0x3FF << 10)
#define	VRSZ(val)		((0x3FF & (val)) << 10)
#define HRSZ_MASK		(0x3FF << 0)
#define	HRSZ(val)		((0x3FF & (val)) << 0)

#define RSZ_OUT_SIZE		RSZ_REG(0x0C)
#define VERT_MASK		(0x1FFF << 16)
#define	VERT(val)		((0x1FFF & (val)) << 16)
#define HORZ_MASK		(0x1FFF << 0)
#define	HORZ(val)		((0x1FFF & (val)) << 0)

#define RSZ_IN_START		RSZ_REG(0x10)
#define VERT_ST_MASK		(0x1FFF << 16)
#define	VERT_ST(val)		((0x1FFF & (val)) << 16)
#define HORZ_ST_MASK		(0x1FFF << 0)
#define	HORZ_ST(val)		((0x1FFF & (val)) << 0)

#define RSZ_IN_SIZE		RSZ_REG(0x14)
#define RSZ_SDR_OUTADDR		RSZ_REG(0x20)
#define RSZ_SDR_OUTOFF		RSZ_REG(0x24)

#define RSZ_HFILTx(x)		RSZ_REG(0x28 + (4 * ((x) / 2)))
#define RSZ_VFILTx(x)		RSZ_REG(0x68 + (4 * ((x) / 2)))
#define FILT_COEFx_MASK(x)	(0x3FF << (16 * (0x1 & (x))))
#define	FILT_COEFx(x, val)	((0x3FF & (val)) << (16 * (0x1 & (x))))

#define RSZ_COEFS		32
#define RSZ_SPM_4_TAP		32
#define RSZ_SPM_7_TAP		64
#define RSZ_C1_4_TAP		16
#define RSZ_C1_7_TAP		32
#define RSZ_HC2_4_TAP		7
#define RSZ_VC2_4_TAP		4
#define RSZ_C2_7_TAP		7
#define RSZ_MIN_4_TAP		64
#define RSZ_MAX_4_TAP		512
#define RSZ_MIN_7_TAP		513
#define RSZ_MAX_7_TAP		1024
#define RSZ_ALIGNW		16
#define RSZ_ALIGNH		1

#define PIX_FORMAT_PRIV_VALID	0x00000001

#define CONFIG_PRV_NF		0x00000001
#define CONFIG_PRV_WB		0x00000002
#define CONFIG_PRV_CFA		0x00000004
#define CONFIG_PRV_RGB		0x00000008
#define CONFIG_PRV_YUV		0x00000010
#define CONFIG_PRV_DCOR		0x00000020
#define CONFIG_PRV_GAMMA	0x00000040
#define CONFIG_PRV_BLKADJ	0x00000080
#define CONFIG_PRV_QUIESCED	\
	(CONFIG_PRV_NF | CONFIG_PRV_CFA | CONFIG_PRV_GAMMA)
#define CONFIG_CCDC_CLAMP	0x00000100
#define CONFIG_CCDC_BLKCOMP	0x00000200
#define CONFIG_ALL		0x000003FF

#define UwQxI(v, x)		((v) >> (x))
#define UwQxF(v, x)		(((1 << (x)) - 1) & (v))
#define SwQxI(v, x)		UwQxI(~(v) + 1, x)
#define SwQxF(v, x)		UwQxF(~(v) + 1, x)
#define U5Q4I(v)		(u8)UwQxI(v, 4)
#define U5Q4F(v)		(u8)UwQxF(v, 4)
#define U8Q4I(v)		(u8)UwQxI(v, 4)
#define U8Q4F(v)		(u8)UwQxF(v, 4)
#define U8Q5I(v)		(u8)UwQxI(v, 5)
#define U8Q5F(v)		(u8)UwQxF(v, 5)
#define S8Q6I(v)		(u8)((v) < 0 ? SwQxI(v, 6) : UwQxI(v, 6))
#define S8Q6F(v)		(u8)((v) < 0 ? SwQxF(v, 6) : UwQxF(v, 6))
#define U10Q8I(v)		(u16)UwQxI(v, 8)
#define U10Q8F(v)		(u16)UwQxF(v, 8)
#define S10Q8I(v)		(u16)((v) < 0 ? SwQxI(v, 8) : UwQxI(v, 8))
#define S10Q8F(v)		(u16)((v) < 0 ? SwQxF(v, 8) : UwQxF(v, 8))
#define S12Q8I(v)		(u16)((v) < 0 ? SwQxI(v, 8) : UwQxI(v, 8))
#define S12Q8F(v)		(u16)((v) < 0 ? SwQxF(v, 8) : UwQxF(v, 8))

#define INVARIANT(cond)						\
	do {							\
		if (!(cond)) {					\
			printk(KERN_DEBUG "!(" # cond ")\n");	\
			BUG();					\
		}						\
	} while (0)

#define SPEW(level, args...)					\
	do {							\
		if (omap34xx_isp_video_spew_level >= level)	\
			printk(KERN_DEBUG "VIDEO:\t" args);	\
	} while (0)

struct omap34xx_isp_video_config {
	u8	ccdc_syn_mode_wen;
	u16	ccdc_horz_info_nph;
	u16	ccdc_vert_lines_nlv;
	u16	ccdc_hsize_off_lnofst;
	u32	ccdc_colptn;
	u8	ccdc_fmtcfg_vpen;
	u16	ccdc_fmt_horz_fmtsph;
	u16	ccdc_fmt_horz_fmtlnh;
	u16	ccdc_fmt_vert_fmtlnv;
	u16	ccdc_vp_out_vert_num;
	u16	ccdc_vp_out_horz_num;

	u8	h3a_pcr_rgbpos;

	u8	prv_pcr_sdrport;
	u8	prv_pcr_rszport;
	u8	prv_pcr_ycpos;
	u16	prv_horz_info_sph;
	u16	prv_horz_info_eph;
	u16	prv_vert_info_elv;
	u16	prv_wadd_offset;
	u32	prv_wbsel;

	u8	rsz_cnt_ycpos;
	u16	rsz_cnt_vrsz;
	u16	rsz_cnt_hrsz;
	u16	rsz_out_size_vert;
	u16	rsz_out_size_horz;
	u16	rsz_in_start_vert_st;
	u16	rsz_in_start_horz_st;
	u16	rsz_in_size_vert;
	u16	rsz_in_size_horz;
	u16	rsz_sdr_outoff_offset;
	u16	rsz_coefs[RSZ_COEFS];
};

struct omap34xx_isp_video_format {
	struct v4l2_pix_format			pix;
	struct v4l2_pix_format			slv;
	struct omap34xx_isp_video_config	cfg;
};

struct omap34xx_isp_video_buffer {
	struct omap34xx_isp_buffer	buf;
	int				active;
};

struct omap34xx_isp_video_data {
	u8	rsz_cnt_vstph;
	u8	rsz_cnt_hstph;

	unsigned int				flags;
	unsigned long				frames;
	struct v4l2_format			slv;
	struct omap34xx_isp_prv_nf_cfg		nf;
	struct omap34xx_isp_prv_wb_cfg		wb;
	struct omap34xx_isp_prv_cfa_cfg		cfa;
	struct omap34xx_isp_prv_rgb_cfg		rgb;
	struct omap34xx_isp_prv_yuv_cfg		yuv;
	struct omap34xx_isp_prv_dcor_cfg	dcor;
	struct omap34xx_isp_video_buffer	*dma;
	struct omap34xx_isp_video_format	*vfmt;
	struct omap34xx_isp_prv_gamma_cfg	gamma;
	struct omap34xx_isp_ccdc_clamp_cfg	clamp;
	struct omap34xx_isp_prv_blkadj_cfg	blkadj;
	struct omap34xx_isp_ccdc_blkcomp_cfg	blkcomp;
};

/* TODO: my rightful location is the board file */
static struct omap34xx_isp_video_format omap34xx_isp_video_formats[] = {
	[0] = { /* QXGA */
		.pix = {
			.width = 2048,
			.height = 1536,
			.pixelformat =  V4L2_PIX_FMT_UYVY,
			.bytesperline = 4096,
			.sizeimage = 6291456,
			.colorspace = V4L2_COLORSPACE_SRGB,
		},
		.slv = {
			.width = 2056,
			.height = 1544,
			.pixelformat = V4L2_PIX_FMT_SGRBG10,
		},
		.cfg = {
			.ccdc_syn_mode_wen = 0,
			.ccdc_colptn = COLPTN_GRBG,
			.ccdc_fmtcfg_vpen = 1,
			.ccdc_fmt_horz_fmtsph = 0,
			.ccdc_fmt_horz_fmtlnh = 2056,
			.ccdc_fmt_vert_fmtlnv = 1544,
			.ccdc_vp_out_vert_num = 1543,
			.ccdc_vp_out_horz_num = 2056,
			.h3a_pcr_rgbpos = RGBPOS_GRBG,
			.prv_pcr_sdrport = 1,
			.prv_pcr_rszport = 0,
			.prv_pcr_ycpos = PCR_YCPOS_YCRYCB,
			.prv_horz_info_sph = 2,
			.prv_horz_info_eph = 2053,
			.prv_vert_info_elv = 1542,
			.prv_wadd_offset = 4096,
			.prv_wbsel = WBSEL_GRBG,
		},
	},
	[1] = { /* VGA */
		.pix = {
			.width = 640,
			.height = 480,
			.pixelformat =  V4L2_PIX_FMT_UYVY,
			.bytesperline = 1280,
			.sizeimage = 614400,
			.colorspace = V4L2_COLORSPACE_JPEG,
		},

		.slv = {
			.width = 1016,
			.height = 768,
			.pixelformat = V4L2_PIX_FMT_SGRBG10,
		},

		.cfg = {
			.ccdc_syn_mode_wen = 0,
			.ccdc_colptn = COLPTN_GRBG,
			.ccdc_fmtcfg_vpen = 1,
			.ccdc_fmt_horz_fmtsph = 0,
			.ccdc_fmt_horz_fmtlnh = 1016,
			.ccdc_fmt_vert_fmtlnv = 768,
			.ccdc_vp_out_vert_num = 767,
			.ccdc_vp_out_horz_num = 1016,
			.h3a_pcr_rgbpos = RGBPOS_GRBG,
			.prv_pcr_sdrport = 0,
			.prv_pcr_rszport = 1,
			.prv_pcr_ycpos = PCR_YCPOS_YCRYCB,
			.prv_horz_info_sph = 2,
			.prv_horz_info_eph = 1013,
			.prv_vert_info_elv = 766,
			.prv_wbsel = WBSEL_GRBG,
			.rsz_cnt_ycpos = CNT_YCPOS_YC,
			.rsz_cnt_vrsz = 383,
			.rsz_cnt_hrsz = 383,
			.rsz_out_size_vert = 480,
			.rsz_out_size_horz = 640,
			.rsz_in_start_vert_st = 18,
			.rsz_in_start_horz_st = 38,
			.rsz_in_size_vert = 722,
			.rsz_in_size_horz = 965,
			.rsz_sdr_outoff_offset = 1280,
			.rsz_coefs = {
				S10Q8(0, 28),
				S10Q8(0, 200),
				S10Q8(0, 28),
				S10Q8(0, 0),
				S10Q8(0, 15),
				S10Q8(0, 194),
				S10Q8(0, 47),
				S10Q8(0, 0),
				S10Q8(0, 7),
				S10Q8(0, 178),
				S10Q8(0, 71),
				S10Q8(0, 0),
				S10Q8(0, 2),
				S10Q8(0, 156),
				S10Q8(0, 98),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 128),
				S10Q8(0, 128),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 98),
				S10Q8(0, 156),
				S10Q8(0, 2),
				S10Q8(0, 0),
				S10Q8(0, 71),
				S10Q8(0, 178),
				S10Q8(0, 7),
				S10Q8(0, 0),
				S10Q8(0, 47),
				S10Q8(0, 194),
				S10Q8(0, 15),
			},
		},
	},
	[2] = { /* HVGA */
		.pix = {
			.width = 480,
			.height = 320,
			.pixelformat =  V4L2_PIX_FMT_UYVY,
			.bytesperline = 960,
			.sizeimage = 307200,
			.colorspace = V4L2_COLORSPACE_SRGB,
		},
		.slv = {
			.width = 1016,
			.height = 768,
			.pixelformat = V4L2_PIX_FMT_SGRBG10,
		},
		.cfg = {
			.ccdc_syn_mode_wen = 0,
			.ccdc_colptn = COLPTN_GRBG,
			.ccdc_fmtcfg_vpen = 1,
			.ccdc_fmt_horz_fmtsph = 0,
			.ccdc_fmt_horz_fmtlnh = 1016,
			.ccdc_fmt_vert_fmtlnv = 768,
			.ccdc_vp_out_vert_num = 767,
			.ccdc_vp_out_horz_num = 1016,
			.h3a_pcr_rgbpos = RGBPOS_GRBG,
			.prv_pcr_sdrport = 0,
			.prv_pcr_rszport = 1,
			.prv_pcr_ycpos = PCR_YCPOS_YCRYCB,
			.prv_horz_info_sph = 2,
			.prv_horz_info_eph = 1013,
			.prv_vert_info_elv = 766,
			.prv_wbsel = WBSEL_GRBG,
			.rsz_cnt_ycpos = CNT_YCPOS_YC,
			.rsz_cnt_vrsz = 530,
			.rsz_cnt_hrsz = 531,
			.rsz_out_size_vert = 320,
			.rsz_out_size_horz = 480,
			.rsz_in_start_vert_st = 45,
			.rsz_in_start_horz_st = 2,
			.rsz_in_size_vert = 668,
			.rsz_in_size_horz = 1002,
			.rsz_sdr_outoff_offset = 960,
			.rsz_coefs = {
				-S10Q8(0, 1),
				S10Q8(0, 20),
				S10Q8(0, 108),
				S10Q8(0, 110),
				S10Q8(0, 20),
				-S10Q8(0, 1),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 8),
				S10Q8(0, 87),
				S10Q8(0, 123),
				S10Q8(0, 39),
				-S10Q8(0, 1),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 1),
				S10Q8(0, 62),
				S10Q8(0, 130),
				S10Q8(0, 62),
				S10Q8(0, 1),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 0),
				-S10Q8(0, 1),
				S10Q8(0, 39),
				S10Q8(0, 123),
				S10Q8(0, 87),
				S10Q8(0, 8),
				S10Q8(0, 0),
				S10Q8(0, 0),
			},
		},
	},
	[3] = { /* QVGA */
		.pix = {
			.width = 320,
			.height = 240,
			.pixelformat =  V4L2_PIX_FMT_UYVY,
			.bytesperline = 640,
			.sizeimage = 153600,
			.colorspace = V4L2_COLORSPACE_SRGB,
		},
		.slv = {
			.width = 1016,
			.height = 768,
			.pixelformat = V4L2_PIX_FMT_SGRBG10,
		},
		.cfg = {
			.ccdc_syn_mode_wen = 0,
			.ccdc_colptn = COLPTN_GRBG,
			.ccdc_fmtcfg_vpen = 1,
			.ccdc_fmt_horz_fmtsph = 0,
			.ccdc_fmt_horz_fmtlnh = 1016,
			.ccdc_fmt_vert_fmtlnv = 768,
			.ccdc_vp_out_vert_num = 767,
			.ccdc_vp_out_horz_num = 1016,
			.h3a_pcr_rgbpos = RGBPOS_GRBG,
			.prv_pcr_sdrport = 0,
			.prv_pcr_rszport = 1,
			.prv_pcr_ycpos = PCR_YCPOS_YCRYCB,
			.prv_horz_info_sph = 2,
			.prv_horz_info_eph = 1013,
			.prv_vert_info_elv = 766,
			.prv_wbsel = WBSEL_GRBG,
			.rsz_cnt_ycpos = CNT_YCPOS_YC,
			.rsz_cnt_vrsz = 797,
			.rsz_cnt_hrsz = 799,
			.rsz_out_size_vert = 240,
			.rsz_out_size_horz = 320,
			.rsz_in_start_vert_st = 3,
			.rsz_in_start_horz_st = 0,
			.rsz_in_size_vert = 752,
			.rsz_in_size_horz = 1004,
			.rsz_sdr_outoff_offset = 640,
			.rsz_coefs = {
				S10Q8(0, 2),
				S10Q8(0, 32),
				S10Q8(0, 94),
				S10Q8(0, 94),
				S10Q8(0, 32),
				S10Q8(0, 2),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 1),
				S10Q8(0, 20),
				S10Q8(0, 80),
				S10Q8(0, 103),
				S10Q8(0, 47),
				S10Q8(0, 5),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 11),
				S10Q8(0, 64),
				S10Q8(0, 106),
				S10Q8(0, 64),
				S10Q8(0, 11),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 0),
				S10Q8(0, 5),
				S10Q8(0, 47),
				S10Q8(0, 103),
				S10Q8(0, 80),
				S10Q8(0, 20),
				S10Q8(0, 1),
				S10Q8(0, 0),
			},
		},
	},
};

static u8 omap34xx_isp_prv_cfa_tbl[OMAP34XX_ISP_PRV_CFA_TBL_LEN] = {
	#include "omap34xx-isp-prv-cfa-tbl.h"
};

static int omap34xx_isp_video_spew_level = 0;

#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
static ssize_t
omap34xx_isp_video_show_spew_level(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t i = 0;

	i = snprintf(buf, PAGE_SIZE, "%d\n", omap34xx_isp_video_spew_level);

	return (i + 1);
}

static ssize_t
omap34xx_isp_video_store_spew_level(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char *endp;

	omap34xx_isp_video_spew_level = simple_strtoul(buf, &endp, 10);

	return (count);
}

static struct device_attribute omap34xx_isp_video_attr_spew_level =
	__ATTR(video_spew_level, S_IRUGO|S_IWUGO,
		omap34xx_isp_video_show_spew_level,
		omap34xx_isp_video_store_spew_level);

static inline int
omap34xx_isp_video_create_spew_level(struct device *dev)
{
	return (device_create_file(dev, &omap34xx_isp_video_attr_spew_level));
}

static inline void
omap34xx_isp_video_remove_spew_level(struct device *dev)
{
	device_remove_file(dev, &omap34xx_isp_video_attr_spew_level);
}
#else // !CONFIG_VIDEO_OMAP34XX_ISP_DBG
static inline int
omap34xx_isp_video_create_spew_level(struct device *dev)
{
	return (0);
}

static inline void
omap34xx_isp_video_remove_spew_level(struct device *dev)
{
}
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG

static inline void
omap34xx_isp_video_buffer_done(struct omap34xx_v4l2_device *dev)
{
	struct omap34xx_isp_video_data *data = dev->priv;

	/* TODO: not SMP friendly... */
	data->dma = NULL;
	wmb();
	omap34xx_v4l2_device_videobuf_done(dev, STATE_DONE);
	++data->frames;
}

static int
omap34xx_isp_video_isr(struct omap34xx_isp_v4l2_device *dev)
{
	unsigned int irq0;
	struct omap34xx_isp_video_data *data;

	data = (struct omap34xx_isp_video_data *)dev->dev.priv;

	irq0 = omap_readl(ISP_IRQ0STATUS);
	irq0 &= HS_VS_IRQ_MASK | RSZ_DONE_IRQ_MASK | PRV_DONE_IRQ_MASK
		| CCDC_VD0_IRQ_MASK;
	omap_writel(irq0, ISP_IRQ0STATUS);

	if (!irq0)
		goto exit;

	SPEW(3, "ISP_IRQSTATUS=0x%08X(%s,%s,%s,%s)\n", irq0,
		HS_VS_IRQ_MASK & irq0 ? __stringify(HS_VS) : "",
		RSZ_DONE_IRQ_MASK & irq0 ? __stringify(RSZ_DONE) : "",
		PRV_DONE_IRQ_MASK & irq0 ? __stringify(PRV_DONE) : "",
		CCDC_VD0_IRQ_MASK & irq0 ? __stringify(CCDC_VD0) : "");

	if ((CCDC_VD0_IRQ_MASK & irq0)
		&& data->vfmt->cfg.ccdc_syn_mode_wen
		&& data->dma
		&& !data->dma->active
		&& omap_testl(CCDC_PCR, BUSY_MASK))
		data->dma->active = 1;

	if ((HS_VS_IRQ_MASK & irq0)
		&& data->vfmt->cfg.ccdc_syn_mode_wen
		&& data->dma
		&& data->dma->active) {
		omap_clearl(CCDC_PCR, ENABLE_MASK);

		if (!omap_testl(CCDC_PCR, BUSY_MASK))
			omap34xx_isp_video_buffer_done(&dev->dev);
	}
	if ((PRV_DONE_IRQ_MASK & irq0)
		&& data->vfmt->cfg.prv_pcr_sdrport
		&& data->dma)
		omap34xx_isp_video_buffer_done(&dev->dev);

	if ((RSZ_DONE_IRQ_MASK & irq0)
		&& data->vfmt->cfg.prv_pcr_rszport
		&& data->dma)
		omap34xx_isp_video_buffer_done(&dev->dev);
exit:
	return (!!irq0);
}

static void
__omap34xx_isp_video_config(struct omap34xx_isp_video_data *data)
{
	int i;
	unsigned int val;
	unsigned int bits;

	if (CONFIG_CCDC_CLAMP & data->flags) {
		val = CLAMPEN(data->clamp.clamp_clampen);
		bits = CLAMPEN_MASK;
		val |= OBSLEN(data->clamp.clamp_obslen);
		bits |= OBSLEN_MASK;
		val |= OBSLN(data->clamp.clamp_obsln);
		bits |= OBSLN_MASK;
		val |= OBST(data->clamp.clamp_obst);
		bits |= OBST_MASK;
		val |= OBGAIN(data->clamp.clamp_obgain);
		bits |= OBGAIN_MASK;
		omap_masked_writel(CCDC_CLAMP, val, bits);
		SPEW(2, "CCDC_CLAMP: CLAMPEN=%u OBSLEN=%u OBSLN=%u OBST=%u"
			"OBGAIN=%u.%u/16\n",
			data->clamp.clamp_clampen, data->clamp.clamp_obslen,
			data->clamp.clamp_obsln, data->clamp.clamp_obst,
			U5Q4I(data->clamp.clamp_obgain),
			U5Q4F(data->clamp.clamp_obgain));

		val = DCSUB(data->clamp.dcsub);
		omap_masked_writel(CCDC_DCSUB, val, DCSUB_MASK);
		SPEW(2, "CCDC_DCSUB: DCSUB=%u\n", data->clamp.dcsub);
	}

	if (CONFIG_CCDC_BLKCOMP & data->flags) {
		val = R_YE(data->blkcomp.blkcmp_r_ye);
		bits = R_YE_MASK;
		val |= GR_CY(data->blkcomp.blkcmp_gr_cy);
		bits |= GR_CY_MASK;
		val |= GB_G(data->blkcomp.blkcmp_gb_g);
		bits |= GB_G_MASK;
		val |= B_MG(data->blkcomp.blkcmp_b_mg);
		bits |= B_MG_MASK;
		omap_masked_writel(CCDC_BLKCMP, val, bits);
		SPEW(2, "CCDC_BLKCMP: R_YE=%d GR_CY=%d GB_G=%d B_MG=%d\n",
			data->blkcomp.blkcmp_r_ye, data->blkcomp.blkcmp_gr_cy,
			data->blkcomp.blkcmp_gb_g, data->blkcomp.blkcmp_b_mg);
	}

	/*
	 * bad things happen when certain features of the preview module are
	 * configured before it quiesces. note: delay is used as this routine is
	 * typically called in interrupt context.	
	 */
	if (CONFIG_PRV_QUIESCED & data->flags) {
		while (omap_testl(PRV_PCR, BUSY_MASK))
			mdelay(1);
	}

	if (CONFIG_PRV_NF & data->flags) {
		val = NFEN(data->nf.pcr_nfen);
		omap_masked_writel(PRV_PCR, val, NFEN_MASK);
		SPEW(2, "PRV_PCR: NFEN=%u\n", data->nf.pcr_nfen);

		omap_masked_writel(PRV_NF, SPR(data->nf.nf_spr), SPR_MASK);
		SPEW(2, "PRV_NF: SPR=%u\n", data->nf.nf_spr);

		omap_writel(NF_TBL_START, PRV_SET_TBL_ADDR);

		for (i = 0; OMAP34XX_ISP_PRV_NF_TBL_LEN > i; ++i) {
			val = TBL_DATA(data->nf.tbl[i]);
			omap_writel(val, PRV_SET_TBL_DATA);
		}
	}
	if (CONFIG_PRV_WB & data->flags) {
		val = DGAIN(data->wb.wb_dgain);
		omap_masked_writel(PRV_WB_DGAIN, val, DGAIN_MASK);
		SPEW(2, "PRV_WB_DGAIN: DGAIN=%u.%u/256\n",
			U10Q8I(data->wb.wb_dgain),
			U10Q8F(data->wb.wb_dgain));

		val = WBGAIN_COEFx(3, data->wb.wbgain_coef3);
		val |= WBGAIN_COEFx(2, data->wb.wbgain_coef2);
		val |= WBGAIN_COEFx(1, data->wb.wbgain_coef1);
		val |= WBGAIN_COEFx(0, data->wb.wbgain_coef0);
		bits = WBGAIN_COEFx_MASK(3);
		bits |= WBGAIN_COEFx_MASK(2);
		bits |= WBGAIN_COEFx_MASK(1);
		bits |= WBGAIN_COEFx_MASK(0);
		omap_masked_writel(PRV_WBGAIN, val, bits);
		SPEW(2, "PRV_WBGAIN: COEF3=%u.%u/32 COEF2=%u.%u/32"
			" COEF1=%u.%u/32 COEF0=%u.%u/32\n",
			U8Q5I(data->wb.wbgain_coef3),
			U8Q5F(data->wb.wbgain_coef3),
			U8Q5I(data->wb.wbgain_coef2),
			U8Q5F(data->wb.wbgain_coef2),
			U8Q5I(data->wb.wbgain_coef1),
			U8Q5F(data->wb.wbgain_coef1),
			U8Q5I(data->wb.wbgain_coef0),
			U8Q5F(data->wb.wbgain_coef0));
	}
	if (CONFIG_PRV_CFA & data->flags) {
		val = CFAEN(data->cfa.pcr_cfaen);
		omap_masked_writel(PRV_PCR, val, CFAEN_MASK);
		SPEW(2, "PRV_PCR: CFAEN=%u\n", data->cfa.pcr_cfaen);

		val = GRADTH_VER(data->cfa.cfa_gradth_ver);
		val |= GRADTH_HOR(data->cfa.cfa_gradth_hor);
		bits = GRADTH_VER_MASK | GRADTH_HOR_MASK;
		omap_masked_writel(PRV_CFA, val, bits);
		SPEW(2, "PRV_CFA: GRADTH_VER=%u GRADTH_HOR=%u\n",
			data->cfa.cfa_gradth_ver, data->cfa.cfa_gradth_hor);

		omap_writel(CFA_TBL_START, PRV_SET_TBL_ADDR);

		for (i = 0; OMAP34XX_ISP_PRV_CFA_TBL_LEN > i; ++i) {
			val = TBL_DATA(data->cfa.tbl[i]);
			omap_writel(val, PRV_SET_TBL_DATA);
		}
	}
	if (CONFIG_PRV_RGB & data->flags) {
		val = MTX_GR(data->rgb.rgb_mat1_mtx_gr);
		val |= MTX_RR(data->rgb.rgb_mat1_mtx_rr);
		bits = MTX_GR_MASK | MTX_RR_MASK;
		omap_masked_writel(PRV_RGB_MAT1, val, bits);
		SPEW(2, "PRV_RGB_MAT1: MTX_GR=%s%u.%u/256"
			" MTX_RR=%s%u.%u/256\n",
			data->rgb.rgb_mat1_mtx_gr < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat1_mtx_gr),
			S12Q8F(data->rgb.rgb_mat1_mtx_gr),
			data->rgb.rgb_mat1_mtx_rr < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat1_mtx_rr),
			S12Q8F(data->rgb.rgb_mat1_mtx_rr));

		val = MTX_RG(data->rgb.rgb_mat2_mtx_rg);
		val |= MTX_BR(data->rgb.rgb_mat2_mtx_br);
		bits = MTX_RG_MASK | MTX_BR_MASK;
		omap_masked_writel(PRV_RGB_MAT2, val, bits);
		SPEW(2, "PRV_RGB_MAT2: MTX_RG=%s%u.%u/256"
			" MTX_BR=%s%u.%u/256\n",
			data->rgb.rgb_mat2_mtx_rg < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat2_mtx_rg),
			S12Q8F(data->rgb.rgb_mat2_mtx_rg),
			data->rgb.rgb_mat2_mtx_br < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat2_mtx_br),
			S12Q8F(data->rgb.rgb_mat2_mtx_br));

		val = MTX_BG(data->rgb.rgb_mat3_mtx_bg);
		val |= MTX_GG(data->rgb.rgb_mat3_mtx_gg);
		bits = MTX_BG_MASK | MTX_GG_MASK;
		omap_masked_writel(PRV_RGB_MAT3, val, bits);
		SPEW(2, "PRV_RGB_MAT3: MTX_BG=%s%u.%u/256"
			" MTX_GG=%s%u.%u/256\n",
			data->rgb.rgb_mat3_mtx_bg < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat3_mtx_bg),
			S12Q8F(data->rgb.rgb_mat3_mtx_bg),
			data->rgb.rgb_mat3_mtx_gg < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat3_mtx_gg),
			S12Q8F(data->rgb.rgb_mat3_mtx_gg));

		val = MTX_GB(data->rgb.rgb_mat4_mtx_gb);
		val |= MTX_RB(data->rgb.rgb_mat4_mtx_rb);
		bits = MTX_GB_MASK | MTX_RB_MASK;
		omap_masked_writel(PRV_RGB_MAT4, val, bits);
		SPEW(2, "PRV_RGB_MAT4: MTX_GB=%s%u.%u/256"
			" MTX_RB=%s%u.%u/256\n",
			data->rgb.rgb_mat4_mtx_gb < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat4_mtx_gb),
			S12Q8F(data->rgb.rgb_mat4_mtx_gb),
			data->rgb.rgb_mat4_mtx_rb < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat4_mtx_rb),
			S12Q8F(data->rgb.rgb_mat4_mtx_rb));

		val = MTX_BB(data->rgb.rgb_mat5_mtx_bb);
		omap_masked_writel(PRV_RGB_MAT5, val, MTX_BB_MASK);
		SPEW(2, "PRV_RGB_MAT5: MTX_BB=%s%u.%u/256\n",
			data->rgb.rgb_mat5_mtx_bb < 0 ? "-" : "",
			S12Q8I(data->rgb.rgb_mat5_mtx_bb),
			S12Q8F(data->rgb.rgb_mat5_mtx_bb));
	}
	if (CONFIG_PRV_YUV & data->flags) {
		val = CSCBY(data->yuv.csc0_cscby);
		val |= CSCGY(data->yuv.csc0_cscgy);
		val |= CSCRY(data->yuv.csc0_cscry);
		bits = CSCBY_MASK | CSCGY_MASK | CSCRY_MASK;
		omap_masked_writel(PRV_CSC0, val, bits);
		SPEW(2, "PRV_CSC0: CSCBY=%s%u.%u/256 CSCGY=%s%u.%u/256"
			" CSCRY=%s%u.%u/256\n",
			data->yuv.csc0_cscby < 0 ? "-" : "",
			S10Q8I(data->yuv.csc0_cscby),
			S10Q8F(data->yuv.csc0_cscby),
			data->yuv.csc0_cscgy < 0 ? "-" : "",
			S10Q8I(data->yuv.csc0_cscgy),
			S10Q8F(data->yuv.csc0_cscgy),
			data->yuv.csc0_cscry < 0 ? "-" : "",
			S10Q8I(data->yuv.csc0_cscry),
			S10Q8F(data->yuv.csc0_cscry));

		val = CSCBCB(data->yuv.csc1_cscbcb);
		val |= CSCGCB(data->yuv.csc1_cscgcb);
		val |= CSCRCB(data->yuv.csc1_cscrcb);
		bits = CSCBCB_MASK | CSCGCB_MASK | CSCRCB_MASK;
		omap_masked_writel(PRV_CSC1, val, bits);
		SPEW(2, "PRV_CSC1: CSCBCB=%s%u.%u/256 CSCGCB=%s%u.%u/256"
			" CSCRCB=%s%u.%u/256\n",
			data->yuv.csc1_cscbcb < 0 ? "-" : "",
			S10Q8I(data->yuv.csc1_cscbcb),
			S10Q8F(data->yuv.csc1_cscbcb),
			data->yuv.csc1_cscgcb < 0 ? "-" : "",
			S10Q8I(data->yuv.csc1_cscgcb),
			S10Q8F(data->yuv.csc1_cscgcb),
			data->yuv.csc1_cscrcb < 0 ? "-" : "",
			S10Q8I(data->yuv.csc1_cscrcb),
			S10Q8F(data->yuv.csc1_cscrcb));

		val = CSCBCR(data->yuv.csc2_cscbcr);
		val |= CSCGCR(data->yuv.csc2_cscgcr);
		val |= CSCRCR(data->yuv.csc2_cscrcr);
		bits = CSCBCR_MASK | CSCGCR_MASK | CSCRCR_MASK;
		omap_masked_writel(PRV_CSC2, val, bits);
		SPEW(2, "PRV_CSC2: CSCBCR=%s%u.%u/256 CSCGCR=%s%u.%u/256"
			" CSCRCR=%s%u.%u/256\n",
			data->yuv.csc2_cscbcr < 0 ? "-" : "",
			S10Q8I(data->yuv.csc2_cscbcr),
			S10Q8F(data->yuv.csc2_cscbcr),
			data->yuv.csc2_cscgcr < 0 ? "-" : "",
			S10Q8I(data->yuv.csc2_cscgcr),
			S10Q8F(data->yuv.csc2_cscgcr),
			data->yuv.csc2_cscrcr < 0 ? "-" : "",
			S10Q8I(data->yuv.csc2_cscrcr),
			S10Q8F(data->yuv.csc2_cscrcr));

		val = CSC_YOFST(data->yuv.csc_offset_yofst);
		val |= CSCGCR(data->yuv.csc_offset_ofstcb);
		val |= CSCRCR(data->yuv.csc_offset_ofstcr);
		bits = CSC_YOFST_MASK | CSC_OFSTCB_MASK | CSC_OFSTCR_MASK;
		omap_masked_writel(PRV_CSC_OFFSET, val, bits);
		SPEW(2, "PRV_CSC_OFFSET: CSC_YOFST=%s%u/256 CSC_OFSTCB=%s%u/256"
			" CSC_OFSTCR=%s%u/256\n",
			data->yuv.csc_offset_yofst < 0 ? "-" : "",
			data->yuv.csc_offset_yofst,
			data->yuv.csc_offset_ofstcb < 0 ? "-" : "",
			data->yuv.csc_offset_ofstcb,
			data->yuv.csc_offset_ofstcr < 0 ? "-" : "",
			data->yuv.csc_offset_ofstcr);
	}
	if (CONFIG_PRV_DCOR & data->flags) {
		val = DCOR_METHOD(data->dcor.pcr_dcor_method);
		val |= DCOREN(data->dcor.pcr_dcoren);
		bits = DCOR_METHOD_MASK | DCOREN_MASK;
		omap_masked_writel(PRV_PCR, val, bits);
		SPEW(2, "PRV_PCR: DCOR_METHOD=%u DCOREN=%u\n",
			data->dcor.pcr_dcor_method, data->dcor.pcr_dcoren);

		val = CORRECT(data->dcor.cdc_thr0_correct);
		val |= DETECT(data->dcor.cdc_thr0_detect);
		bits = CORRECT_MASK | DETECT_MASK;
		omap_masked_writel(PRV_CDC_THRx(0), val, bits);
		SPEW(2, "PRV_CDC_THR0: CORRECT=%u DETECT=%u\n",
			data->dcor.cdc_thr0_correct,
			data->dcor.cdc_thr0_detect);

		val = CORRECT(data->dcor.cdc_thr1_correct);
		val |= DETECT(data->dcor.cdc_thr1_detect);
		bits = CORRECT_MASK | DETECT_MASK;
		omap_masked_writel(PRV_CDC_THRx(1), val, bits);
		SPEW(2, "PRV_CDC_THR1: CORRECT=%u DETECT=%u\n",
			data->dcor.cdc_thr1_correct,
			data->dcor.cdc_thr1_detect);

		val = CORRECT(data->dcor.cdc_thr2_correct);
		val |= DETECT(data->dcor.cdc_thr2_detect);
		bits = CORRECT_MASK | DETECT_MASK;
		omap_masked_writel(PRV_CDC_THRx(2), val, bits);
		SPEW(2, "PRV_CDC_THR2: CORRECT=%u DETECT=%u\n",
			data->dcor.cdc_thr2_correct,
			data->dcor.cdc_thr2_detect);

		val = CORRECT(data->dcor.cdc_thr3_correct);
		val |= DETECT(data->dcor.cdc_thr3_detect);
		bits = CORRECT_MASK | DETECT_MASK;
		omap_masked_writel(PRV_CDC_THRx(3), val, bits);
		SPEW(2, "PRV_CDC_THR3: CORRECT=%u DETECT=%u\n",
			data->dcor.cdc_thr3_correct,
			data->dcor.cdc_thr3_detect);
	}
	if (CONFIG_PRV_GAMMA & data->flags) {
		val = GAMMA_BYPASS(data->gamma.pcr_gamma_bypass);
		omap_masked_writel(PRV_PCR, val, GAMMA_BYPASS_MASK);
		SPEW(2, "PRV_PCR: GAMMA_BYPASS=%u\n",
			data->gamma.pcr_gamma_bypass);

		omap_writel(RGAMMA_TBL_START, PRV_SET_TBL_ADDR);

		for (i = 0; OMAP34XX_ISP_PRV_GAMMA_TBL_LEN > i; ++i) {
			val = TBL_DATA(data->gamma.red_tbl[i]);
			omap_writel(val, PRV_SET_TBL_DATA);
		}

		omap_writel(GGAMMA_TBL_START, PRV_SET_TBL_ADDR);

		for (i = 0; OMAP34XX_ISP_PRV_GAMMA_TBL_LEN > i; ++i) {
			val = TBL_DATA(data->gamma.green_tbl[i]);
			omap_writel(val, PRV_SET_TBL_DATA);
		}

		omap_writel(BGAMMA_TBL_START, PRV_SET_TBL_ADDR);

		for (i = 0; OMAP34XX_ISP_PRV_GAMMA_TBL_LEN > i; ++i) {
			val = TBL_DATA(data->gamma.blue_tbl[i]);
			omap_writel(val, PRV_SET_TBL_DATA);
		}
	}
	if (CONFIG_PRV_BLKADJ & data->flags) {
		val = R(data->blkadj.blkadjoff_r);
		val |= G(data->blkadj.blkadjoff_g);
		val |= B(data->blkadj.blkadjoff_b);
		bits = R_MASK | G_MASK | B_MASK;
		omap_masked_writel(PRV_BLKADJOFF, val, bits);
		SPEW(2, "PRV_BLKADJOFF: R=%u G=%u B=%u\n",
			data->blkadj.blkadjoff_r, data->blkadj.blkadjoff_g,
			data->blkadj.blkadjoff_b);
	}

	data->flags &= ~CONFIG_ALL;
}

static void omap34xx_isp_video_configure(struct omap34xx_isp_v4l2_device *dev)
{
	unsigned int val;
	unsigned int bits;
	struct omap34xx_isp_video_data *data = dev->dev.priv;

	SPEW(1, "+++ %s\n", __func__);

	bits = HS_VS_IRQ_MASK | RSZ_DONE_IRQ_MASK | PRV_DONE_IRQ_MASK
		| CCDC_VD0_IRQ_MASK;
	omap_setl(ISP_IRQ0ENABLE, bits);

	val = VSTPH(data->rsz_cnt_vstph);
	val |= HSTPH(data->rsz_cnt_hstph);
	omap_masked_writel(RSZ_CNT, val, VSTPH_MASK | HSTPH_MASK);
	SPEW(2, "RSZ_CNT: VSTPH=%u HSTPH=%u\n", data->rsz_cnt_vstph,
		data->rsz_cnt_hstph);

	/* force hardware configuration */
	data->flags |= CONFIG_ALL;
	__omap34xx_isp_video_config(data);

	SPEW(1, "--- %s\n", __func__);
}

static unsigned int
omap34xx_isp_video_videobuf_size(struct omap34xx_v4l2_device *dev)
{
	unsigned int size;
	struct omap34xx_isp_video_data *data = dev->priv;

	SPEW(3, "+++ %s\n", __func__);

	/* TODO: assumes format has previously been set */
	size = data->vfmt->pix.sizeimage;

	SPEW(3, "--- %s: size=%u\n", __func__, size);

	return (size);
}

static size_t
omap34xx_isp_video_videobuf_align(struct omap34xx_v4l2_device *dev)
{
	size_t align;

	SPEW(3, "+++ %s\n", __func__);

	align = ADDR_ALIGN;

	SPEW(3, "--- %s: align=%u\n", __func__, align);

	return (align);
}

static void
omap34xx_isp_video_videobuf_start(struct omap34xx_v4l2_device *dev,
					struct videobuf_buffer *qbuf)
{
	struct omap34xx_isp_video_data *data = dev->priv;
	struct omap34xx_isp_video_buffer *buf;

	buf = container_of(qbuf, struct omap34xx_isp_video_buffer, buf.qbuf);

	SPEW(3, "+++ %s: addr=0x%08X\n", __func__, buf->buf.addr);

	INVARIANT(!data->dma);

	if (CONFIG_ALL & data->flags)
		__omap34xx_isp_video_config(data);

	buf->active = 0;
	wmb();
	data->dma = buf;

	if (data->vfmt->cfg.ccdc_syn_mode_wen) {
		omap_writel(ADDR(buf->buf.addr), CCDC_SDR_ADDR);
		omap_setl(CCDC_PCR, ENABLE_MASK);
	}
	else if (data->vfmt->cfg.prv_pcr_sdrport) {
		omap_writel(ADDR(buf->buf.addr), PRV_WSDR_ADDR);
		omap_setl(PRV_PCR, PRV_PCR_ONESHOT_MASK | ENABLE_MASK);
	}
	else if (data->vfmt->cfg.prv_pcr_rszport) {
		omap_writel(ADDR(buf->buf.addr), RSZ_SDR_OUTADDR);
		omap_setl(RSZ_PCR, RSZ_PCR_ONESHOT_MASK | ENABLE_MASK);
		omap_setl(PRV_PCR, PRV_PCR_ONESHOT_MASK | ENABLE_MASK);
	}

	SPEW(3, "--- %s\n", __func__);
}

static int
omap34xx_isp_video_vidioc_querycap(struct omap34xx_v4l2_device *dev,
					struct v4l2_capability *cap)
{
	SPEW(1, "+++ %s\n", __func__);

	memset(cap, 0, sizeof (*cap));
	strncpy(cap->driver, OMAP34XX_ISP_DRIVER, sizeof(cap->driver));
	/* TODO: revisit */
	strncpy(cap->card, "OMAP34xx ISP", sizeof(cap->card));
	cap->version = LINUX_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE
				| V4L2_CAP_READWRITE
				| V4L2_CAP_STREAMING;

	SPEW(1, "--- %s\n", __func__);

	return (0);
}

static int omap34xx_isp_video_vidioc_enum_fmt(struct omap34xx_v4l2_device *dev,
						struct v4l2_fmtdesc *desc)
{
	int rc;

	SPEW(1, "+++ %s: index=%u\n", __func__, desc->index);

	if (ARRAY_SIZE(omap34xx_isp_video_formats) <= desc->index) {
		rc = -EINVAL;
		goto exit;
	}

	desc->pixelformat =
		omap34xx_isp_video_formats[desc->index].pix.pixelformat;
	rc = 0;
exit:
	SPEW(1, "--- %s: rc=%d pixelformat=%c%c%c%c\n", __func__, rc,
		0xFF & (desc->pixelformat >> 0),
		0xFF & (desc->pixelformat >> 8),
		0xFF & (desc->pixelformat >> 16),
		0xFF & (desc->pixelformat >> 24));

	return (rc);
}

static int omap34xx_isp_video_try_format(
		struct v4l2_pix_format *pix,
		struct omap34xx_isp_video_format **vfmt)
{
	int i;
	int rc = -EINVAL;
	struct omap34xx_isp_video_format *cur;
	struct omap34xx_isp_video_format *sel = NULL;

	for (i = 0, cur = &omap34xx_isp_video_formats[0];
		ARRAY_SIZE(omap34xx_isp_video_formats) > i;
		i++, cur++) {
		if (pix->pixelformat != cur->pix.pixelformat)
			continue;

		if (!sel || ((pix->width <= cur->pix.width)
				&& (pix->height <= cur->pix.height)))
			sel = cur;
	}

	if (sel) {
		*pix = sel->pix;
		rc = 0;
	}

	if (vfmt)
		*vfmt = sel;

	return (rc);
}

static int omap34xx_isp_video_vidioc_g_fmt(struct omap34xx_v4l2_device *dev,
						struct v4l2_format *fmt)
{
	int rc;
	struct omap34xx_isp_video_data *data = dev->priv;

	SPEW(1, "+++ %s\n", __func__);

	fmt->fmt.pix = data->vfmt->pix;
	rc = 0;

	SPEW(1, "--- %s: rc=%d width=%u height=%u pixelformat=%c%c%c%c"
		" bytesperline=%u sizeimage=%u colorspace=%d\n", __func__,
		rc, fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24),
		fmt->fmt.pix.bytesperline, fmt->fmt.pix.sizeimage,
		fmt->fmt.pix.colorspace);

	return (rc);
}

static int omap34xx_isp_video_set_format(struct omap34xx_v4l2_device *dev,
						struct v4l2_pix_format *pix)
{
	int rc;
	struct omap34xx_isp_input *in;
	struct omap34xx_isp_video_data *dat = dev->priv;
	struct omap34xx_isp_video_format *vfmt;

	if ((rc = omap34xx_isp_video_try_format(pix, &vfmt)))
		goto exit;

	dat->slv.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dat->slv.fmt.pix = vfmt->slv;
	in = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	if ((rc = omap34xx_v4l2_int_master_ioctl_s_fmt(&in->mst, &dat->slv)))
		goto exit;

	if ((dat->slv.fmt.pix.pixelformat != vfmt->slv.pixelformat)
		|| (dat->slv.fmt.pix.width != vfmt->slv.width)
		|| (dat->slv.fmt.pix.height != vfmt->slv.height)) {
		rc = -EINVAL;
		goto exit;
	}

	dat->vfmt = vfmt;
exit:
	return (rc);
}

static int omap34xx_isp_video_vidioc_s_fmt(struct omap34xx_v4l2_device *dev,
						struct v4l2_format *fmt)
{
	int rc;

	SPEW(1, "+++ %s: width=%u height=%u pixelformat=%c%c%c%c\n", __func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24));

	rc = omap34xx_isp_video_set_format(dev, &fmt->fmt.pix);

	SPEW(1, "--- %s: rc=%d width=%u height=%u pixelformat=%c%c%c%c"
		" bytesperline=%u sizeimage=%u colorspace=%d\n", __func__,
		rc, fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24),
		fmt->fmt.pix.bytesperline, fmt->fmt.pix.sizeimage,
		fmt->fmt.pix.colorspace);

	return (rc);
}

static int omap34xx_isp_video_vidioc_try_fmt(struct omap34xx_v4l2_device *dev,
						struct v4l2_format *fmt)
{
	int rc;

	SPEW(1, "+++ %s: width=%u height=%u pixelformat=%c%c%c%c\n", __func__,
		fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24));

	rc = omap34xx_isp_video_try_format(&fmt->fmt.pix, NULL);

	SPEW(1, "--- %s: rc=%d width=%u height=%u pixelformat=%c%c%c%c"
		" bytesperline=%u sizeimage=%u colorspace=%d\n", __func__,
		rc, fmt->fmt.pix.width, fmt->fmt.pix.height,
		0xFF & (fmt->fmt.pix.pixelformat >> 0),
		0xFF & (fmt->fmt.pix.pixelformat >> 8),
		0xFF & (fmt->fmt.pix.pixelformat >> 16),
		0xFF & (fmt->fmt.pix.pixelformat >> 24),
		fmt->fmt.pix.bytesperline, fmt->fmt.pix.sizeimage,
		fmt->fmt.pix.colorspace);

	return (rc);
}

static int omap34xx_isp_video_vidioc_streamon(struct omap34xx_v4l2_device *dev)
{
	int i;
	int rc;
	unsigned int val;
	unsigned int bits;
	struct omap34xx_isp_input *in;
	struct omap34xx_isp_video_data *dat = dev->priv;

	SPEW(1, "+++ %s\n", __func__);

	in = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	/* NOTE: make sure a format is set */
	if ((rc = omap34xx_isp_video_set_format(dev, &dat->vfmt->pix)))
		goto exit;

	if ((rc = omap34xx_v4l2_int_master_ioctl_streamon(&in->mst)))
		goto exit;

	val = WEN(dat->vfmt->cfg.ccdc_syn_mode_wen);
	omap_masked_writel(CCDC_SYN_MODE, val, WEN_MASK);
	SPEW(2, "CCDC_SYN_MODE: WEN=%u\n", dat->vfmt->cfg.ccdc_syn_mode_wen);

	omap_writel(dat->vfmt->cfg.ccdc_colptn, CCDC_COLPTN);
	SPEW(2, "CCDC_COLPTN: 0x%08X\n", dat->vfmt->cfg.ccdc_colptn);

	val = VPEN(dat->vfmt->cfg.ccdc_fmtcfg_vpen);
	omap_masked_writel(CCDC_FMTCFG, val, VPEN_MASK);
	SPEW(2, "CCDC_FMTCFG: VPEN=%u\n", dat->vfmt->cfg.ccdc_fmtcfg_vpen);

	val = FMTSPH(dat->vfmt->cfg.ccdc_fmt_horz_fmtsph);
	val |= FMTLNH(dat->vfmt->cfg.ccdc_fmt_horz_fmtlnh);
	omap_masked_writel(CCDC_FMT_HORZ, val, FMTSPH_MASK | FMTLNH_MASK);
	SPEW(2, "CCDC_FMT_HORZ: FMTSPH=%u FMTLNH=%u\n",
		dat->vfmt->cfg.ccdc_fmt_horz_fmtsph,
		dat->vfmt->cfg.ccdc_fmt_horz_fmtlnh);

	val = FMTLNV(dat->vfmt->cfg.ccdc_fmt_vert_fmtlnv);
	omap_masked_writel(CCDC_FMT_VERT, val, FMTLNV_MASK);
	SPEW(2, "CCDC_FMT_VERT: FMTLNV=%u\n", dat->vfmt->cfg.ccdc_fmt_vert_fmtlnv);

	val = VERT_NUM(dat->vfmt->cfg.ccdc_vp_out_vert_num);
	val |= HORZ_NUM(dat->vfmt->cfg.ccdc_vp_out_horz_num);
	bits = VERT_NUM_MASK | HORZ_NUM_MASK;
	omap_masked_writel(CCDC_VP_OUT, val, bits);
	SPEW(2, "CCDC_VP_OUT: VERT_NUM=%u HORZ_NUM=%u\n",
		dat->vfmt->cfg.ccdc_vp_out_vert_num,
		dat->vfmt->cfg.ccdc_vp_out_horz_num);

	val = RGBPOS(dat->vfmt->cfg.h3a_pcr_rgbpos);
	omap_masked_writel(H3A_PCR, val, RGBPOS_MASK);
	SPEW(2, "H3A_PCR: RGBPOS=%u\n", dat->vfmt->cfg.h3a_pcr_rgbpos);

	if (dat->vfmt->cfg.ccdc_syn_mode_wen) {
		val = NPH(dat->vfmt->cfg.ccdc_horz_info_nph);
		omap_masked_writel(CCDC_HORZ_INFO, val, NPH_MASK);
		SPEW(2, "CCDC_HORZ_INFO: NPH=%u\n",
			dat->vfmt->cfg.ccdc_horz_info_nph);

		val = NLV(dat->vfmt->cfg.ccdc_vert_lines_nlv);
		omap_masked_writel(CCDC_VERT_LINES, val, NLV_MASK);
		SPEW(2, "CCDC_VERT_LINES: NLV=%u\n",
			dat->vfmt->cfg.ccdc_vert_lines_nlv);

		val = LNOFST(dat->vfmt->cfg.ccdc_hsize_off_lnofst);
		omap_masked_writel(CCDC_HSIZE_OFF, val, LNOFST_MASK);
		SPEW(2, "CCDC_HSIZE_OFF: LNOFST=%u\n",
			dat->vfmt->cfg.ccdc_hsize_off_lnofst);

		goto configured;
	}

	val = SPH(dat->vfmt->cfg.prv_horz_info_sph);
	val |= EPH(dat->vfmt->cfg.prv_horz_info_eph);
	bits = SPH_MASK | EPH_MASK;
	omap_masked_writel(PRV_HORZ_INFO, val, bits);
	SPEW(2, "PRV_HORZ_INFO: SPH=%u EPH=%u\n",
		dat->vfmt->cfg.prv_horz_info_sph,
		dat->vfmt->cfg.prv_horz_info_eph);

	val = ELV(dat->vfmt->cfg.prv_vert_info_elv);
	omap_masked_writel(PRV_VERT_INFO, val, ELV_MASK);
	SPEW(2, "PRV_VERT_INFO: ELV=%u\n", dat->vfmt->cfg.prv_vert_info_elv);

	val = SDRPORT(dat->vfmt->cfg.prv_pcr_sdrport);
	val |= RSZPORT(dat->vfmt->cfg.prv_pcr_rszport);
	val |= PCR_YCPOS(dat->vfmt->cfg.prv_pcr_ycpos);
	bits = SDRPORT_MASK | RSZPORT_MASK | PCR_YCPOS_MASK;
	omap_masked_writel(PRV_PCR, val, bits);
	SPEW(2, "PRV_PCR: SDRPORT=%u RSZPORT=%u PCR_YCPOS=%u\n",
		dat->vfmt->cfg.prv_pcr_sdrport, dat->vfmt->cfg.prv_pcr_rszport,
		dat->vfmt->cfg.prv_pcr_ycpos);

	val = OFFSET(dat->vfmt->cfg.prv_wadd_offset);
	omap_masked_writel(PRV_WADD_OFFSET, val, OFFSET_MASK);
	SPEW(2, "PRV_WADD_OFFSET: OFFSET=%u\n", dat->vfmt->cfg.prv_wadd_offset);

	omap_writel(dat->vfmt->cfg.prv_wbsel, PRV_WBSEL);
	SPEW(2, "PRV_WBSEL: 0x%08X\n", dat->vfmt->cfg.prv_wbsel);

	if (dat->vfmt->cfg.prv_pcr_sdrport)
		goto configured;

	val = CNT_YCPOS(dat->vfmt->cfg.rsz_cnt_ycpos);
	val |= VRSZ(dat->vfmt->cfg.rsz_cnt_vrsz);
	val |= HRSZ(dat->vfmt->cfg.rsz_cnt_hrsz);
	bits = CNT_YCPOS_MASK | VRSZ_MASK | HRSZ_MASK;
	omap_masked_writel(RSZ_CNT, val, bits);
	SPEW(2, "RSZ_CNT: YCPOS=%u VRSZ=%u HRSZ=%u\n",
		dat->vfmt->cfg.rsz_cnt_ycpos, dat->vfmt->cfg.rsz_cnt_vrsz,
		dat->vfmt->cfg.rsz_cnt_hrsz);

	val = VERT(dat->vfmt->cfg.rsz_in_size_vert);
	val |= HORZ(dat->vfmt->cfg.rsz_in_size_horz);
	omap_masked_writel(RSZ_IN_SIZE, val, VERT_MASK | HORZ_MASK);
	SPEW(2, "RSZ_IN_SIZE: VERT=%u HORZ=%u\n",
		dat->vfmt->cfg.rsz_in_size_vert,
		dat->vfmt->cfg.rsz_in_size_horz);

	val = VERT_ST(dat->vfmt->cfg.rsz_in_start_vert_st);
	val |= HORZ_ST(dat->vfmt->cfg.rsz_in_start_horz_st);
	omap_masked_writel(RSZ_IN_START, val, VERT_ST_MASK | HORZ_ST_MASK);
	SPEW(2, "RSZ_IN_START: VERT_ST=%u HORZ_ST=%u\n",
		dat->vfmt->cfg.rsz_in_start_vert_st,
		dat->vfmt->cfg.rsz_in_start_horz_st);

	val = VERT(dat->vfmt->cfg.rsz_out_size_vert);
	val |= HORZ(dat->vfmt->cfg.rsz_out_size_horz);
	omap_masked_writel(RSZ_OUT_SIZE, val, VERT_MASK | HORZ_MASK);
	SPEW(2, "RSZ_OUT_SIZE: VERT=%u HORZ=%u\n",
		dat->vfmt->cfg.rsz_out_size_vert,
		dat->vfmt->cfg.rsz_out_size_horz);

	for (i = 0; RSZ_COEFS > i; ++i) {
		val = FILT_COEFx(i, dat->vfmt->cfg.rsz_coefs[i]);
		omap_masked_writel(RSZ_HFILTx(i), val, FILT_COEFx_MASK(i));
		omap_masked_writel(RSZ_VFILTx(i), val, FILT_COEFx_MASK(i));
	}

	val = OFFSET(dat->vfmt->cfg.rsz_sdr_outoff_offset);
	omap_masked_writel(RSZ_SDR_OUTOFF, val, OFFSET_MASK);
	SPEW(2, "RSZ_SDR_OUTOFF: OFFSET=%u\n",
		dat->vfmt->cfg.rsz_sdr_outoff_offset);

configured:
	bits = PREV_RAM_EN(dat->vfmt->cfg.prv_pcr_sdrport);
	bits |= CCDC_RAM_EN_MASK;
	bits |= RSZ_CLK_EN(dat->vfmt->cfg.prv_pcr_rszport);
	bits |= PRV_CLK_EN(!dat->vfmt->cfg.ccdc_syn_mode_wen);
	bits |= CCDC_CLK_EN_MASK;
	omap_setl(ISP_CTRL, bits);

	if (!dat->vfmt->cfg.ccdc_syn_mode_wen)
		omap_setl(CCDC_PCR, ENABLE_MASK);

	dat->frames = 0;
	rc = 0;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_isp_video_vidioc_streamoff(struct omap34xx_v4l2_device *dev)
{
	struct omap34xx_isp_input *in;
	struct omap34xx_isp_video_data *dat = dev->priv;

	SPEW(1, "+++ %s\n", __func__);

	in = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;
	(void)omap34xx_v4l2_int_master_ioctl_streamoff(&in->mst);

	/*
	 * HACK!
	 * The resizer sets its busy bit immediately after it's enabled. To
	 * reset the bit we've no choice but to employ the nuclear option...
	 */
	(void)omap34xx_isp_reset(dev->parent, msecs_to_jiffies(1));

	/* TODO: not SMP friendly... */
	if (dat->dma) {
		dat->dma = NULL;
		omap34xx_v4l2_device_videobuf_done(dev, STATE_ERROR);
	}

	SPEW(1, "--- %s: frames=%lu\n", __func__, dat->frames);

	return (0);
}

static int
omap34xx_isp_video_get_config(struct omap34xx_v4l2_device *dev,
				struct v4l2_control *ctrl, void **cfg,
				unsigned long *size, unsigned int *flag)
{
	int rc;
	struct omap34xx_isp_video_data *data = dev->priv;

	SPEW(3, "+++ %s: id=0x%08X\n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_OMAP34XX_ISP_CCDC_CLAMP_CFG:
		*cfg = &data->clamp;
		*size = sizeof(data->clamp);
		*flag = CONFIG_CCDC_CLAMP;
		break;
	case V4L2_CID_OMAP34XX_ISP_CCDC_BLKCOMP_CFG:
		*cfg = &data->blkcomp;
		*size = sizeof(data->blkcomp);
		*flag = CONFIG_CCDC_BLKCOMP;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_DCOR_CFG:
		*cfg = &data->dcor;
		*size = sizeof(data->dcor);
		*flag = CONFIG_PRV_DCOR;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_NF_CFG:
		*cfg = &data->nf;
		*size = sizeof(data->nf);
		*flag = CONFIG_PRV_NF;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_WB_CFG:
		*cfg = &data->wb;
		*size = sizeof(data->wb);
		*flag = CONFIG_PRV_WB;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_CFA_CFG:
		*cfg = &data->cfa;
		*size = sizeof(data->cfa);
		*flag = CONFIG_PRV_CFA;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_BLKADJ_CFG:
		*cfg = &data->blkadj;
		*size = sizeof(data->blkadj);
		*flag = CONFIG_PRV_BLKADJ;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_RGB_CFG:
		*cfg = &data->rgb;
		*size = sizeof(data->rgb);
		*flag = CONFIG_PRV_RGB;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_GAMMA_CFG:
		*cfg = &data->gamma;
		*size = sizeof(data->gamma);
		*flag = CONFIG_PRV_GAMMA;
		break;
	case V4L2_CID_OMAP34XX_ISP_PRV_YUV_CFG:
		*cfg = &data->yuv;
		*size = sizeof(data->yuv);
		*flag = CONFIG_PRV_YUV;
		break;
	default:
		rc = -EINVAL;
		goto exit;
	}

	rc = 0;
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_isp_video_vidioc_g_ctrl(struct omap34xx_v4l2_device *dev,
						struct v4l2_control *ctl)
{
	int rc;
	void *cfg;
	unsigned int flag;
	unsigned long n;
	struct omap34xx_isp_input *in;
	struct omap34xx_isp_video_data *dat = dev->priv;

	SPEW(3, "+++ %s\n", __func__);

	in = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	if (V4L2_CID_OMAP34XX_ISP_BASE != V4L2_CTRL_ID2CLASS(ctl->id)) {
		rc = omap34xx_v4l2_int_master_ioctl_g_ctrl(&in->mst, ctl);
		goto exit;
	}

	switch (ctl->id) {
	case V4L2_CID_OMAP34XX_ISP_PRV_OUT_HORZ:
		ctl->value = dat->vfmt->cfg.ccdc_vp_out_horz_num;
		rc = 0;
		goto exit;
	case V4L2_CID_OMAP34XX_ISP_PRV_OUT_VERT:
		ctl->value = dat->vfmt->cfg.ccdc_vp_out_vert_num;
		rc = 0;
		goto exit;
	}

	if ((rc = omap34xx_isp_video_get_config(dev, ctl, &cfg, &n, &flag)))
		goto exit;

	if (copy_to_user((void *)ctl->value, cfg, n))
		rc = -EINVAL;
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
omap34xx_isp_video_vidioc_s_ctrl(struct omap34xx_v4l2_device *dev,
					struct v4l2_control *ctrl)
{
	int rc;
	void *cfg;
	unsigned int flag;
	unsigned long n;
	unsigned long flags;
	struct omap34xx_isp_input *in;
	struct omap34xx_isp_video_data *data = dev->priv;

	in = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	SPEW(3, "+++ %s\n", __func__);

	if (V4L2_CID_OMAP34XX_ISP_BASE != V4L2_CTRL_ID2CLASS(ctrl->id)) {
		rc = omap34xx_v4l2_int_master_ioctl_s_ctrl(&in->mst, ctrl);
		goto exit;
	}
	if ((rc = omap34xx_isp_video_get_config(dev, ctrl, &cfg, &n, &flag)))
		goto exit;

	spin_lock_irqsave(&dev->qlock, flags);

	if (!copy_from_user(cfg, (void *)ctrl->value, n))
		data->flags |= flag;

	else
		rc = -EINVAL;

	spin_unlock_irqrestore(&dev->qlock, flags);
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_isp_video_vidioc_g_parm(struct omap34xx_v4l2_device *dev,
						struct v4l2_streamparm *parm)
{
	struct omap34xx_isp_input *in;

	in = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	return (omap34xx_v4l2_int_master_ioctl_g_parm(&in->mst, parm));
}

static int omap34xx_isp_video_vidioc_s_parm(struct omap34xx_v4l2_device *dev,
						struct v4l2_streamparm *parm)
{
	struct omap34xx_isp_input *in;

	in = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	return (omap34xx_v4l2_int_master_ioctl_s_parm(&in->mst, parm));
}

int omap34xx_isp_video_device_probe(struct omap34xx_isp_v4l2_device *dev)
{
	int i;
	int rc;
	struct platform_device *pdev = dev->dev.parent;
	struct omap34xx_isp_video_data *data;
	struct omap34xx_isp_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		rc = -ENODEV;
		goto exit;
	}

	if (!(data = kzalloc(sizeof(*data), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	data->gamma.pcr_gamma_bypass = pdata->prv_pcr_gamma_bypass;
	data->cfa.pcr_cfaen = pdata->prv_pcr_cfaen;
	data->wb.wb_dgain = pdata->prv_wb_dgain;
	data->wb.wbgain_coef0 = pdata->prv_wbgain_coef0;
	data->wb.wbgain_coef1 = pdata->prv_wbgain_coef1;
	data->wb.wbgain_coef2 = pdata->prv_wbgain_coef2;
	data->wb.wbgain_coef3 = pdata->prv_wbgain_coef3;
	data->rgb.rgb_mat1_mtx_rr = pdata->prv_rgb_mat1_mtx_rr;
	data->rgb.rgb_mat3_mtx_gg = pdata->prv_rgb_mat3_mtx_gg;
	data->rgb.rgb_mat5_mtx_bb = pdata->prv_rgb_mat5_mtx_bb;
	data->yuv.csc0_cscby = pdata->prv_csc0_cscby;
	data->yuv.csc0_cscgy = pdata->prv_csc0_cscgy;
	data->yuv.csc0_cscry = pdata->prv_csc0_cscry;
	data->yuv.csc1_cscbcb = pdata->prv_csc1_cscbcb;
	data->yuv.csc1_cscgcb = pdata->prv_csc1_cscgcb;
	data->yuv.csc1_cscrcb = pdata->prv_csc1_cscrcb;
	data->yuv.csc2_cscbcr = pdata->prv_csc2_cscbcr;
	data->yuv.csc2_cscgcr = pdata->prv_csc2_cscgcr;
	data->yuv.csc2_cscrcr = pdata->prv_csc2_cscrcr;

	for (i = 0; OMAP34XX_ISP_PRV_CFA_TBL_LEN > i; ++i)
		data->cfa.tbl[i] = omap34xx_isp_prv_cfa_tbl[i];

	data->rsz_cnt_vstph = pdata->rsz_cnt_vstph;
	data->rsz_cnt_hstph = pdata->rsz_cnt_hstph;

	data->vfmt = &omap34xx_isp_video_formats[0];

	dev->isr = omap34xx_isp_video_isr;
	dev->configure = omap34xx_isp_video_configure;
	dev->dev.priv = data;
	dev->dev.qtype = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dev->dev.qsize = sizeof(struct omap34xx_isp_video_buffer);
	dev->dev.file_open = omap34xx_isp_file_open;
	dev->dev.file_release = omap34xx_isp_file_release;
	dev->dev.videobuf_size = omap34xx_isp_video_videobuf_size;
	dev->dev.videobuf_align = omap34xx_isp_video_videobuf_align;
	dev->dev.videobuf_init = omap34xx_isp_videobuf_init;
	dev->dev.videobuf_start = omap34xx_isp_video_videobuf_start;
	dev->dev.videobuf_release = omap34xx_isp_videobuf_release;
	dev->dev.vidioc_querycap = omap34xx_isp_video_vidioc_querycap;
	dev->dev.vidioc_g_fmt = omap34xx_isp_video_vidioc_g_fmt;
	dev->dev.vidioc_s_fmt = omap34xx_isp_video_vidioc_s_fmt;
	dev->dev.vidioc_try_fmt = omap34xx_isp_video_vidioc_try_fmt;
	dev->dev.vidioc_enum_input = omap34xx_isp_vidioc_enum_input;
	dev->dev.vidioc_g_input = omap34xx_isp_vidioc_g_input;
	dev->dev.vidioc_s_input = omap34xx_isp_vidioc_s_input;
	dev->dev.vidioc_enum_fmt = omap34xx_isp_video_vidioc_enum_fmt;
	dev->dev.vidioc_g_ctrl = omap34xx_isp_video_vidioc_g_ctrl;
	dev->dev.vidioc_s_ctrl = omap34xx_isp_video_vidioc_s_ctrl;
	dev->dev.vidioc_g_parm = omap34xx_isp_video_vidioc_g_parm;
	dev->dev.vidioc_s_parm = omap34xx_isp_video_vidioc_s_parm;
	dev->dev.vidioc_streamoff = omap34xx_isp_video_vidioc_streamoff;
	dev->dev.vidioc_streamon = omap34xx_isp_video_vidioc_streamon;

	if ((rc = omap34xx_v4l2_device_register(&dev->dev)))
		goto omap34xx_v4l2_device_register_failed;

	if ((rc = omap34xx_isp_video_create_spew_level(&dev->dev.parent->dev)))
		goto omap34xx_isp_video_create_spew_level_failed;

	goto exit;

omap34xx_isp_video_create_spew_level_failed:
	video_unregister_device(&dev->dev.dev);
omap34xx_v4l2_device_register_failed:
	kfree(data);
exit:
	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_video_device_probe);

int omap34xx_isp_video_device_remove(struct omap34xx_isp_v4l2_device *dev)
{
	struct omap34xx_isp_video_data *data = dev->dev.priv;

	omap34xx_isp_video_remove_spew_level(&dev->dev.parent->dev);
	video_unregister_device(&dev->dev.dev);
	kfree(data);

	return (0);
}
EXPORT_SYMBOL(omap34xx_isp_video_device_remove);
