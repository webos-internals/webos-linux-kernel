/*
 * include/asm-arm/arch-omap24xx/display.h
 *
 * Copyright (C) 2004-2005 Texas Instruments.
 * Copyright (C) 2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.

 * Leveraged from original Linux 2.6 framebuffer driver for OMAP24xx
 * Author: Andy Lowe (source@mvista.com)
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 */

#ifndef	__ASM_ARCH_OMAP2_DISP_H
#define	__ASM_ARCH_OMAP2_DISP_H

#include <linux/videodev.h>

/*physical memory map definitions */
	/* display subsystem */
#define DSS_REG_BASE			0x48050000
#define DSS_REG_SIZE			0x00001000
	/* DSS */
#define DSS_REG_OFFSET			0x00000000
	/* display controller */
#define DISPC_REG_OFFSET		0x00000400
	/* remote framebuffer interface */
#define RFBI_REG_OFFSET			0x00000800
	/* video encoder */
#define VENC_REG_OFFSET			0x00000C00

/* display subsystem register offsets */
#define DSS_REVISION			0x000
#define DSS_SYSCONFIG			0x010
#define DSS_SYSSTATUS			0x014
#define DSS_IRQSTATUS			0x018
#define DSS_CONTROL			0x040
#ifdef CONFIG_ARCH_OMAP3430
#define DSS_SDI_CONTROL			0x044 /* omap3430 specific */
#define DSS_PLL_CONTROL			0x048 /* omap3430 specific */
#define DSS_SDI_STATUS			0x05C
#endif /* CONFIG_ARCH_OMAP3430 */
#define DSS_PSA_LCD_REG_1		0x050
#define DSS_PSA_LCD_REG_2		0x054
#define DSS_PSA_VIDEO_REG		0x058
#define DSS_STATUS			0x05C

/* display controller register offsets */
#define DISPC_REVISION			0x000
#define DISPC_SYSCONFIG			0x010
#define DISPC_SYSSTATUS			0x014
#define DISPC_IRQSTATUS			0x018
#define DISPC_IRQENABLE			0x01C
#define DISPC_CONTROL			0x040
#define DISPC_CONFIG			0x044
#define DISPC_CAPABLE			0x048
#define DISPC_DEFAULT_COLOR0		0x04C
#define DISPC_DEFAULT_COLOR1		0x050
#define DISPC_TRANS_COLOR0		0x054
#define DISPC_TRANS_COLOR1		0x058
#define DISPC_LINE_STATUS		0x05C
#define DISPC_LINE_NUMBER		0x060
#define DISPC_TIMING_H			0x064
#define DISPC_TIMING_V			0x068
#define DISPC_POL_FREQ			0x06C
#define DISPC_DIVISOR			0x070
#define DISPC_GLOBAL_ALPHA		0x074
#define DISPC_SIZE_DIG			0x078
#define DISPC_SIZE_LCD			0x07C
#define DISPC_GFX_BA0			0x080
#define DISPC_GFX_BA1			0x084
#define DISPC_GFX_POSITION		0x088
#define DISPC_GFX_SIZE			0x08C
#define DISPC_GFX_ATTRIBUTES		0x0A0
#define DISPC_GFX_FIFO_THRESHOLD	0x0A4
#define DISPC_GFX_FIFO_SIZE		0x0A8
#define DISPC_GFX_ROW_INC		0x0AC
#define DISPC_GFX_PIXEL_INC		0x0B0
#define DISPC_GFX_WINDOW_SKIP		0x0B4
#define DISPC_GFX_TABLE_BA		0x0B8

#define RFBI_SYSCONFIG			0x010

/* The registers for the video pipelines are parameterized by the video pipeline
 * index: n=0 for VID1 and n=1 for VID2.
 */
#define DISPC_VID_BA0(n)		(0x0BC + (n)*0x90)
#define DISPC_VID_BA1(n)		(0x0C0 + (n)*0x90)
#define DISPC_VID_POSITION(n)		(0x0C4 + (n)*0x90)
#define DISPC_VID_SIZE(n)		(0x0C8 + (n)*0x90)
#define DISPC_VID_ATTRIBUTES(n)		(0x0CC + (n)*0x90)
#define DISPC_VID_FIFO_THRESHOLD(n)	(0x0D0 + (n)*0x90)
#define DISPC_VID_FIFO_SIZE(n)		(0x0D4 + (n)*0x90)
#define DISPC_VID_ROW_INC(n)		(0x0D8 + (n)*0x90)
#define DISPC_VID_PIXEL_INC(n)		(0x0DC + (n)*0x90)
#define DISPC_VID_FIR(n)		(0x0E0 + (n)*0x90)
#define DISPC_VID_PICTURE_SIZE(n)	(0x0E4 + (n)*0x90)
#define DISPC_VID_ACCU0(n)		(0x0E8 + (n)*0x90)
#define DISPC_VID_ACCU1(n)		(0x0EC + (n)*0x90)

/* The FIR coefficients are parameterized by the video pipeline index n = {0, 1}
 * and the coefficient index i = {0, 1, 2, 3, 4, 5, 6, 7}.
 */
#define DISPC_VID_FIR_COEF_H(n, i)	(0x0F0 + (i)*0x8 + (n)*0x90)
#define DISPC_VID_FIR_COEF_HV(n, i)	(0x0F4 + (i)*0x8 + (n)*0x90)
#define DISPC_VID_FIR_COEF_V(n, i)  	(0x1E0 + (i)*0x4 + (n)*0x20)
#define DISPC_VID_CONV_COEF0(n)		(0x130 + (n)*0x90)
#define DISPC_VID_CONV_COEF1(n)		(0x134 + (n)*0x90)
#define DISPC_VID_CONV_COEF2(n)		(0x138 + (n)*0x90)
#define DISPC_VID_CONV_COEF3(n)		(0x13C + (n)*0x90)
#define DISPC_VID_CONV_COEF4(n)		(0x140 + (n)*0x90)

#define DISPC_DATA_CYCLE1		0x1D4
#define DISPC_DATA_CYCLE2		0x1D8
#define DISPC_DATA_CYCLE3		0x1DC

#define DISPC_CPR_R			0x220
#define DISPC_CPR_G			0x224
#define DISPC_CPR_B			0x228

/* bit fields within selected registers */
#define DSS_CONTROL_VENC_OUT				(1 << 6)
#define DSS_CONTROL_TV_REF				(1 << 5)
#define DSS_CONTROL_DAC_DEMEN				(1 << 4)
#define DSS_CONTROL_VENC_CLOCK_4X_ENABLE		(1 << 3)
#define DSS_CONTROL_VENC_CLOCK_MODE			(1 << 2)
#define DSS_CONTROL_CLK					(1 << 0)
#define DSS_CONTROL_APLL_CLK				1
#define DSS_CONTROL_DPLL_CLK				0
#define DSS_SYSCONFIG_SOFTRESET				(1 <<  1)
#define DSS_SYSSTATUS_RESETDONE				(1 <<  0)
#define DSS_SYSCONFIG_SIDLEMODE				(3 <<  3)
#define DSS_SYSCONFIG_SIDLEMODE_FIDLE			(0 <<  3)
#define DSS_SYSCONFIG_SIDLEMODE_NIDLE			(1 <<  3)
#define DSS_SYSCONFIG_SIDLEMODE_SIDLE			(2 <<  3)
#define DSS_SYSCONFIG_SOFTRESET				(1 <<  1)
#define DSS_SYSCONFIG_AUTOIDLE				(1 <<  0)
#ifdef CONFIG_ARCH_OMAP3430
#define DSS_SDI_CONTROL_PDIV				(31 << 15)
#define DSS_SDI_CONTROL_PDIV_SHIFT			15
#define DSS_SDI_CONTROL_PRSEL				(3 << 2)
#define DSS_SDI_CONTROL_PRSEL_2PAIR			(1 << 2)
#define DSS_SDI_CONTROL_BWSEL				(3 << 0)
#define DSS_SDI_CONTROL_BWSEL_24B			(2 << 0)
#define DSS_PLL_CONTROL_GOBIT				(1 << 28)
#define DSS_PLL_CONTROL_LOCKSEL				(3 << 26)
#define DSS_PLL_CONTROL_LOCKSEL_PHASE			(0 << 26)
#define DSS_PLL_CONTROL_LOCKSEL_FINE_PHASE		(1 << 26)
#define DSS_PLL_CONTROL_LOCKSEL_FREQUENCY		(2 << 26)
#define DSS_PLL_CONTROL_PLLLPMODE			(1 << 21)
#define DSS_PLL_CONTROL_FREQSEL				(15 << 22)
#define DSS_PLL_CONTROL_FREQSEL_SHIFT			22
#define DSS_PLL_CONTROL_SYSRESET			(1 << 18)
#define DSS_PLL_CONTROL_STOPMODE			(1 << 17)
#define DSS_PLL_CONTROL_REGN				(63 << 11)
#define DSS_PLL_CONTROL_REGN_SHIFT			11
#define DSS_PLL_CONTROL_REGM				(1023 << 1)
#define DSS_PLL_CONTROL_REGM_SHIFT			1
#define DSS_SDI_STATUS_PLL_BUSYFLAG			(1 << 6)
#define DSS_SDI_STATUS_PLL_LOCK				(1 << 5)
#define DSS_SDI_STATUS_RESET_DONE			(1 << 2)
#endif // CONFIG_ARCH_OMAP3430

#define DISPC_REVISION_MAJOR				(15 << 4)
#define DISPC_REVISION_MAJOR_SHIFT			4
#define DISPC_REVISION_MINOR				(15 << 0)
#define DISPC_REVISION_MINOR_SHIFT			0

#define DISPC_SYSCONFIG_MIDLEMODE			(3 << 12)
#define DISPC_SYSCONFIG_MIDLEMODE_FSTANDBY		(0 << 12)
#define DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY		(1 << 12)
#define DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY		(2 << 12)
#define DISPC_SYSCONFIG_SIDLEMODE			(3 <<  3)
#define DISPC_SYSCONFIG_SIDLEMODE_FIDLE			(0 <<  3)
#define DISPC_SYSCONFIG_SIDLEMODE_NIDLE			(1 <<  3)
#define DISPC_SYSCONFIG_SIDLEMODE_SIDLE			(2 <<  3)
#define DISPC_SYSCONFIG_SOFTRESET			(1 <<  1)
#define DISPC_SYSCONFIG_AUTOIDLE			(1 <<  0)
#define DISPC_SYSCONFIG_CLKACTIVITY			(2 <<  8)
#define DISPC_SYSCONFIG_ENABLE_WKUP			(1 <<  2)

#define DISPC_SYSSTATUS_RESETDONE			(1 << 0)

#define DISPC_IRQSTATUS_SYNCLOSTDIGITAL			(1 << 15)
#define DISPC_IRQSTATUS_SYNCLOST			(1 << 14)
#define DISPC_IRQSTATUS_VID2ENDWINDOW			(1 << 13)
#define DISPC_IRQSTATUS_VID2FIFOUNDERFLOW		(1 << 12)
#define DISPC_IRQSTATUS_VID1ENDWINDOW			(1 << 11)
#define DISPC_IRQSTATUS_VID1FIFOUNDERFLOW		(1 << 10)
#define DISPC_IRQSTATUS_OCPERROR			(1 <<  9)
#define DISPC_IRQSTATUS_PALETTEGAMMALOADING		(1 <<  8)
#define DISPC_IRQSTATUS_GFXENDWINDOW			(1 <<  7)
#define DISPC_IRQSTATUS_GFXFIFOUNDERFLOW		(1 <<  6)
#define DISPC_IRQSTATUS_PROGRAMMEDLINENUMBER		(1 <<  5)
#define DISPC_IRQSTATUS_ACBIASCOUNTSTATUS		(1 <<  4)
#define DISPC_IRQSTATUS_EVSYNC_ODD			(1 <<  3)
#define DISPC_IRQSTATUS_EVSYNC_EVEN			(1 <<  2)
#define DISPC_IRQSTATUS_VSYNC				(1 <<  1)
#define DISPC_IRQSTATUS_FRAMEDONE			(1 <<  0)

#define DISPC_IRQENABLE_SYNCLOSTDIGITAL			(1 << 15)
#define DISPC_IRQENABLE_SYNCLOST			(1 << 14)
#define DISPC_IRQENABLE_VID2ENDWINDOW			(1 << 13)
#define DISPC_IRQENABLE_VID2FIFOUNDERFLOW		(1 << 12)
#define DISPC_IRQENABLE_VID1ENDWINDOW			(1 << 11)
#define DISPC_IRQENABLE_VID1FIFOUNDERFLOW		(1 << 10)
#define DISPC_IRQENABLE_OCPERROR			(1 <<  9)
#define DISPC_IRQENABLE_PALETTEGAMMALOADING		(1 <<  8)
#define DISPC_IRQENABLE_GFXENDWINDOW			(1 <<  7)
#define DISPC_IRQENABLE_GFXFIFOUNDERFLOW		(1 <<  6)
#define DISPC_IRQENABLE_PROGRAMMEDLINENUMBER		(1 <<  5)
#define DISPC_IRQENABLE_ACBIASCOUNTSTATUS		(1 <<  4)
#define DISPC_IRQENABLE_EVSYNC_ODD			(1 <<  3)
#define DISPC_IRQENABLE_EVSYNC_EVEN			(1 <<  2)
#define DISPC_IRQENABLE_VSYNC				(1 <<  1)
#define DISPC_IRQENABLE_FRAMEDONE			(1 <<  0)

#define DISPC_CONTROL_LCDENABLEPOL_ACTIVELOW		(0 << 29)
#define DISPC_CONTROL_LCDENABLEPOL_ACTIVEHIGH		(1 << 29)
#define DISPC_CONTROL_LCDENABLESIGNAL			(1 << 28)
#define DISPC_CONTROL_PCKFREEENABLE			(1 << 27)
#define DISPC_CONTROL_TDMUNUSEDBITS			(3 << 25)
#define DISPC_CONTROL_TDMUNUSEDBITS_LOWLEVEL		(0 << 25)
#define DISPC_CONTROL_TDMUNUSEDBITS_HIGHLEVEL		(1 << 25)
#define DISPC_CONTROL_TDMUNUSEDBITS_UNCHANGED		(2 << 25)
#define DISPC_CONTROL_TDMCYCLEFORMAT			(3 << 23)
#define DISPC_CONTROL_TDMCYCLEFORMAT_1CYCPERPIX		(0 << 23)
#define DISPC_CONTROL_TDMCYCLEFORMAT_2CYCPERPIX		(1 << 23)
#define DISPC_CONTROL_TDMCYCLEFORMAT_3CYCPERPIX		(2 << 23)
#define DISPC_CONTROL_TDMCYCLEFORMAT_3CYCPER2PIX	(3 << 23)
#define DISPC_CONTROL_TDMPARALLELMODE			(3 << 21)
#define DISPC_CONTROL_TDMPARALLELMODE_8BPARAINT		(0 << 21)
#define DISPC_CONTROL_TDMPARALLELMODE_9BPARAINT		(1 << 21)
#define DISPC_CONTROL_TDMPARALLELMODE_12BPARAINT	(2 << 21)
#define DISPC_CONTROL_TDMPARALLELMODE_16BPARAINT	(3 << 21)
#define DISPC_CONTROL_TDMENABLE				(1 << 20)
#define DISPC_CONTROL_HT				(7 << 17)
#define DISPC_CONTROL_HT_SHIFT				17
#define DISPC_CONTROL_GPOUT1				(1 << 16)
#define DISPC_CONTROL_GPOUT0				(1 << 15)
#define DISPC_CONTROL_GPIN1				(1 << 14)
#define DISPC_CONTROL_GPIN0				(1 << 13)
#define DISPC_CONTROL_OVERLAYOPTIMIZATION		(1 << 12)
#define DISPC_CONTROL_RFBIMODE				(1 << 11)
#define DISPC_CONTROL_SECURE				(1 << 10)
#define DISPC_CONTROL_TFTDATALINES			(3 <<  8)
#define DISPC_CONTROL_TFTDATALINES_OALSB12B		(0 <<  8)
#define DISPC_CONTROL_TFTDATALINES_OALSB16B		(1 <<  8)
#define DISPC_CONTROL_TFTDATALINES_OALSB18B		(2 <<  8)
#define DISPC_CONTROL_TFTDATALINES_OALSB24B		(3 <<  8)
#define DISPC_CONTROL_TFTDITHERENABLE			(1 <<  7)
#define DISPC_CONTROL_GODIGITAL				(1 <<  6)
#define DISPC_CONTROL_GOLCD				(1 <<  5)
#define DISPC_CONTROL_M8B				(1 <<  4)
#define DISPC_CONTROL_STNTFT				(1 <<  3)
#define DISPC_CONTROL_MONOCOLOR				(1 <<  2)
#define DISPC_CONTROL_DIGITALENABLE			(1 <<  1)
#define DISPC_CONTROL_LCDENABLE				(1 <<  0)

#define DISPC_CONFIG_TVALPHAENABLE			(1 << 19)
#define DISPC_CONFIG_LCDALPHAENABLE			(1 << 18)
#ifdef CONFIG_ARCH_OMAP3430
#define DISPC_CONFIG_FIFOMERGE				(1 << 14)
#endif
#define DISPC_CONFIG_TCKDIGSELECTION			(1 << 13)
#define DISPC_CONFIG_TCKDIGENABLE			(1 << 12)
#define DISPC_CONFIG_TCKLCDSELECTION			(1 << 11)
#define DISPC_CONFIG_TCKLCDENABLE			(1 << 10)
#define DISPC_CONFIG_FUNCGATED				(1 <<  9)
#define DISPC_CONFIG_ACBIASGATED			(1 <<  8)
#define DISPC_CONFIG_VSYNCGATED				(1 <<  7)
#define DISPC_CONFIG_HSYNCGATED				(1 <<  6)
#define DISPC_CONFIG_PIXELCLOCKGATED			(1 <<  5)
#define DISPC_CONFIG_PIXELDATAGATED			(1 <<  4)
#define DISPC_CONFIG_PALETTEGAMMATABLE			(1 <<  3)
#define DISPC_CONFIG_LOADMODE_FRDATLFFR			(3 <<  1)
#define DISPC_CONFIG_LOADMODE_FRDATLEFR			(1 <<  2)
#define DISPC_CONFIG_LOADMODE_PGTABUSETB		(1 <<  1)
#define DISPC_CONFIG_LOADMODE_MASK			(3 <<  1)
#define DISPC_CONFIG_PIXELGATED				(1 <<  0)

#define DISPC_CAPABLE_GFXGAMMATABLECAPABLE		(1 <<  9)
#define DISPC_CAPABLE_GFXLAYERCAPABLE			(1 <<  8)
#define DISPC_CAPABLE_GFXTRANSDSTCAPABLE		(1 <<  7)
#define DISPC_CAPABLE_STNDITHERINGCAPABLE		(1 <<  6)
#define DISPC_CAPABLE_TFTDITHERINGCAPABLE		(1 <<  5)
#define DISPC_CAPABLE_VIDTRANSSRCCAPABLE		(1 <<  4)
#define DISPC_CAPABLE_VIDLAYERCAPABLE			(1 <<  3)
#define DISPC_CAPABLE_VIDVERTFIRCAPABLE			(1 <<  2)
#define DISPC_CAPABLE_VIDHORFIRCAPABLE			(1 <<  1)
#define DISPC_CAPABLE_VIDCAPABLE			(1 <<  0)

#define DISPC_POL_FREQ_ONOFF_SHIFT			17
#define DISPC_POL_FREQ_ONOFF				(1 << 17)
#define DISPC_POL_FREQ_RF				(1 << 16)
#define DISPC_POL_FREQ_IEO				(1 << 15)
#define DISPC_POL_FREQ_IPC_SHIFT			14
#define DISPC_POL_FREQ_IPC				(1 << 14)
#define DISPC_POL_FREQ_IHS				(1 << 13)
#define DISPC_POL_FREQ_IVS				(1 << 12)
#define DISPC_POL_FREQ_ACBI				(15 << 8)
#define DISPC_POL_FREQ_ACBI_SHIFT			8
#define DISPC_POL_FREQ_ACB				0xFF
#define DISPC_POL_FREQ_ACB_SHIFT			0

#define DISPC_TIMING_H_HBP				(0xFF << 20)
#define DISPC_TIMING_H_HBP_SHIFT			20
#define DISPC_TIMING_H_HFP				(0xFF << 8)
#define DISPC_TIMING_H_HFP_SHIFT			8
#define DISPC_TIMING_H_HSW				(0x3F << 0)
#define DISPC_TIMING_H_HSW_SHIFT			0

#define DISPC_TIMING_V_VBP				(0xFF << 20)
#define DISPC_TIMING_V_VBP_SHIFT			20
#define DISPC_TIMING_V_VFP				(0xFF << 8)
#define DISPC_TIMING_V_VFP_SHIFT			8
#define DISPC_TIMING_V_VSW				(0x3F << 0)
#define DISPC_TIMING_V_VSW_SHIFT			0

#define DISPC_DIVISOR_LCD				(0xFF << 16)
#define DISPC_DIVISOR_LCD_SHIFT				16
#define DISPC_DIVISOR_PCD				0xFF
#define DISPC_DIVISOR_PCD_SHIFT				0

#define DISPC_GLOBAL_ALPHA_VID2_GALPHA			(0xFF << 16)
#define DISPC_GLOBAL_ALPHA_VID2_GALPHA_SHIFT		16
#define DISPC_GLOBAL_ALPHA_GFX_GALPHA			0xFF
#define DISPC_GLOBAL_ALPHA_GFX_GALPHA_SHIFT		0

#define DISPC_SIZE_LCD_LPP				(0x7FF << 16)
#define DISPC_SIZE_LCD_LPP_SHIFT			16
#define DISPC_SIZE_LCD_PPL				0x7FF
#define DISPC_SIZE_LCD_PPL_SHIFT			0


#define DISPC_SIZE_DIG_LPP				(0x7FF << 16)
#define DISPC_SIZE_DIG_LPP_SHIFT			16
#define DISPC_SIZE_DIG_PPL				0x7FF
#define DISPC_SIZE_DIG_PPL_SHIFT			0

#define DISPC_GFX_POSITION_GFXPOSY			(0x7FF << 16)
#define DISPC_GFX_POSITION_GFXPOSY_SHIFT		16
#define DISPC_GFX_POSITION_GFXPOSX			0x7FF
#define DISPC_GFX_POSITION_GFXPOSX_SHIFT		0

#define DISPC_GFX_SIZE_GFXSIZEY				(0x7FF << 16)
#define DISPC_GFX_SIZE_GFXSIZEY_SHIFT			16
#define DISPC_GFX_SIZE_GFXSIZEX				0x7FF
#define DISPC_GFX_SIZE_GFXSIZEX_SHIFT			0

#define DISPC_GFX_ATTRIBUTES_GFXENDIANNESS		(1 << 10)
#define DISPC_GFX_ATTRIBUTES_GFXNIBBLEMODE		(1 <<  9)
#define DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT		(1 <<  8)
#define DISPC_GFX_ATTRIBUTES_GFXBURSTSIZE		(3 <<  6)
#define DISPC_GFX_ATTRIBUTES_GFXBURSTSIZE_BURST4X32	(0 <<  6)
#define DISPC_GFX_ATTRIBUTES_GFXBURSTSIZE_BURST8X32	(1 <<  6)
#define DISPC_GFX_ATTRIBUTES_GFXBURSTSIZE_BURST16X32	(2 <<  6)
#define DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE	(1 <<  5)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT			(15 << 1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP1		(0 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP2		(1 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP4		(2 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP8		(3 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB12		(4 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB16		(6 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB24		(8 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_ARGB32		(12 <<  1)
#define DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGBA32		(13 <<  1)
#define DISPC_GFX_ATTRIBUTES_ENABLE			(1 <<  0)
#define DISPC_GFX_ATTRIBUTES_GFXREPEN			5

#ifdef CONFIG_ARCH_OMAP3430
#define DISPC_GFX_FIFO_THRESHOLD_HIGH			(0xFFF << 16)
#define DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT		16
#define DISPC_GFX_FIFO_THRESHOLD_LOW			0xFFF
#define DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT		0
#else
#define DISPC_GFX_FIFO_THRESHOLD_HIGH			(0x1FF << 16)
#define DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT		16
#define DISPC_GFX_FIFO_THRESHOLD_LOW			0x1FF
#define DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT		0
#endif /* CONFIG_ARCH_OMAP3430 */

#define DISPC_VID_POSITION_VIDPOSY			(0x7FF << 16)
#define DISPC_VID_POSITION_VIDPOSY_SHIFT		16
#define DISPC_VID_POSITION_VIDPOSX			0x7FF
#define DISPC_VID_POSITION_VIDPOSX_SHIFT		0

#define DISPC_VID_SIZE_VIDSIZEY				(0x7FF << 16)
#define DISPC_VID_SIZE_VIDSIZEY_SHIFT			16
#define DISPC_VID_SIZE_VIDSIZEX				0x7FF
#define DISPC_VID_SIZE_VIDSIZEX_SHIFT			0

#define DISPC_VID_ATTRIBUTES_VIDROWREPEATENABLE		(1 << 18)
#define DISPC_VID_ATTRIBUTES_VIDENDIANNESS		(1 << 17)
#define DISPC_VID_ATTRIBUTES_VIDCHANNELOUT		(1 << 16)
#define DISPC_VID_ATTRIBUTES_VIDBURSTSIZE		(3 << 14)
#define DISPC_VID_ATTRIBUTES_VIDBURSTSIZE_BURST4X32	(0 << 14)
#define DISPC_VID_ATTRIBUTES_VIDBURSTSIZE_BURST8X32	(1 << 14)
#define DISPC_VID_ATTRIBUTES_VIDBURSTSIZE_BURST16X32	(2 << 14)
#define DISPC_VID_ATTRIBUTES_VIDROTATION(n)		((n) << 12)
#define DISPC_VID_ATTRIBUTES_VIDFULLRANGE		(1 << 11)
#define DISPC_VID_ATTRIBUTES_VIDREPLICATIONENABLE	(1 << 10)
#define DISPC_VID_ATTRIBUTES_VIDCOLORCONVENABLE		(1 <<  9)
#define DISPC_VID_ATTRIBUTES_VIDVRESIZECONF		(1 <<  8)
#define DISPC_VID_ATTRIBUTES_VIDHRESIZECONF		(1 <<  7)
#define DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_VRESIZE	(1 <<  6)
#define DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_HRESIZE	(1 <<  5)
#define DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_HVRESIZE (0x3 << 5)
#define DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS (1 << 21)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT			(15 << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB12		(4  << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_ARGB12		(5  << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB16		(6  << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24            (8  << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24P           (9  << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_YUV2		(10 << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_UYVY		(11 << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_ARGB32		(12 << 1)
#define DISPC_VID_ATTRIBUTES_VIDFORMAT_RGBA32		(13 << 1)
#define DISPC_VID_ATTRIBUTES_ENABLE			(1 <<  0)

#define DISPC_VID_PICTURE_SIZE_VIDORGSIZEY		(0x7FF << 16)
#define DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT	16
#define DISPC_VID_PICTURE_SIZE_VIDORGSIZEX		0x7FF
#define DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT	0

#define DISPC_VID_ATTRIBUTES_VIDROT			12
#define DISPC_VID_ATTRIBUTES_VIDROWREPEAT		18

/*RFBI Sysconfig values */
#define RFBI_SYSCONFIG_SIDLEMODE_SIDLE			(2 << 3)

/* VENC register offsets */
#define VENC_F_CONTROL				0x0008
#define VENC_VIDOUT_CTRL			0x0010
#define VENC_SYNC_CONTROL			0x0014
#define VENC_LLEN				0x001C
#define VENC_FLENS				0x0020
#define VENC_HFLTR_CTRL				0x0024
#define VENC_CC_CARR_WSS_CARR			0x0028
#define VENC_C_PHASE				0x002C
#define VENC_GAIN_U				0x0030
#define VENC_GAIN_V				0x0034
#define VENC_GAIN_Y				0x0038
#define VENC_BLACK_LEVEL			0x003C
#define VENC_BLANK_LEVEL			0x0040
#define VENC_X_COLOR				0x0044
#define VENC_M_CONTROL				0x0048
#define VENC_BSTAMP_WSS_DATA			0x004C
#define VENC_S_CARR				0x0050
#define VENC_LINE21				0x0054
#define VENC_LN_SEL				0x0058
#define VENC_L21_WC_CTL				0x005C
#define VENC_HTRIGGER_VTRIGGER			0x0060
#define VENC_SAVID_EAVID			0x0064
#define VENC_FLEN_FAL				0x0068
#define VENC_LAL_PHASE_RESET			0x006C
#define VENC_HS_INT_START_STOP_X		0x0070
#define VENC_HS_EXT_START_STOP_X		0x0074
#define VENC_VS_INT_START_X			0x0078
#define VENC_VS_INT_STOP_X_VS_INT_START_Y	0x007C
#define VENC_VS_INT_STOP_Y_VS_EXT_START_X	0x0080
#define VENC_VS_EXT_STOP_X_VS_EXT_START_Y	0x0084
#define VENC_VS_EXT_STOP_Y			0x0088
#define VENC_AVID_START_STOP_X			0x0090
#define VENC_AVID_START_STOP_Y			0x0094
#define VENC_FID_INT_START_X_FID_INT_START_Y	0x00A0
#define VENC_FID_INT_OFFSET_Y_FID_EXT_START_X	0x00A4
#define VENC_FID_EXT_START_Y_FID_EXT_OFFSET_Y	0x00A8
#define VENC_TVDETGP_INT_START_STOP_X		0x00B0
#define VENC_TVDETGP_INT_START_STOP_Y		0x00B4
#define VENC_GEN_CTRL				0x00B8
#define VENC_DAC_TST				0x00C4
#define VENC_DAC				0x00C8

/* VENC bit fields */
#define VENC_FCONTROL_RESET			(1<<8)

#define VENC_L21_WC_CTL_INV			(1<<15)
#define VENC_L21_WC_CTL_INV_SHIFT	15
#define VENC_L21_WC_CTL_EVEN_ODD_EN		(3<<13)
#define VENC_L21_WC_CTL_EVEN_ODD_EN_SHIFT	13
#define VENC_L21_WC_CTL_LINE		(0x1F<<8)
#define VENC_L21_WC_CTL_LINE_SHIFT	8
#define VENC_L21_WC_CTL_L21EN		(0x3)
#define VENC_L21_WC_CTL_L21EN_SHIFT	 0


#define VENC_BSTAMP_WSS_DATA_WSS_DATA		(0xFFFFF<<8)
#define VENC_BSTAMP_WSS_DATA_WSS_DATA_SHIFT 8
#define VENC_BSTAMP_WSS_DATA_WSS_SQP		(1<<7)
#define VENC_BSTAMP_WSS_DATA_WSS_SQP_SHIFT	7
#define VENC_BSTAMP_WSS_DATA_WSS_BSTAP		(0x7F<<0)
#define VENC_BSTAMP_WSS_DATA_WSS_BSTAP_SHIFT 0

#define VENC_CC_CARR_WSS_CARR_FWSS		(0xFFFF<<16)
#define VENC_CC_CARR_WSS_CARR_FWSS_SHIFT	16
#define VENC_CC_CARR_WSS_CARR_FCC		(0xFFFF<<0)
#define VENC_CC_CARR_WSS_CARR_FCC_SHIFT		0


/* Rotation using VRFB */
#define SMS_ROT_VIRT_BASE(context, degree)	0x70000000 		\
						| 0x4000000 * (context)	\
						| 0x1000000 * (degree/90)
#define SMS_IMAGEHEIGHT_OFFSET			16
#define SMS_IMAGEWIDTH_OFFSET			0
#define SMS_PH_OFFSET				8
#define SMS_PW_OFFSET				4
#define SMS_PS_OFFSET				0

#ifdef CONFIG_ARCH_OMAP2420
#define OMAP_SMS_BASE	(L3_24XX_BASE + 0x8000)
#endif
#ifdef CONFIG_ARCH_OMAP2430
#define OMAP_SMS_BASE	OMAP243X_SMS_PHYS
#endif
#ifdef CONFIG_ARCH_OMAP3430
#define OMAP_SMS_BASE	SMS_PHYS
#endif

#define SMS_ROT0_PHYSICAL_BA(context)	__REG32(OMAP_SMS_BASE + 0x188 \
						+ 0x10 * context)
#define SMS_ROT_CONTROL(context)	__REG32(OMAP_SMS_BASE + 0x180 \
						+ 0x10 * context)
#define SMS_ROT0_SIZE(context)		__REG32(OMAP_SMS_BASE + 0x184 \
						+ 0x10 * context)

/* Structure to store and restore the DSS registers */
struct omap24xx_dispc_regs {
	u32 revision;			/* 0x000 */
	u32 res1[3];
	u32 sysconfig;		/* 0x010 */
	u32 sysstatus;		/* 0x014 */
	u32 irqstatus;		/* 0x018 */
	u32 irqenable;		/* 0x01C */
	u32 res2[8];
	u32 control;			/* 0x040 */
	u32 config;			/* 0x044 */
	u32 capable;			/* 0x048 */
	u32 default_color0;		/* 0x04C */
	u32 default_color1;		/* 0x050 */
	u32 trans_color0;		/* 0x054 */
	u32 trans_color1;		/* 0x058 */
	u32 line_status;		/* 0x05C */
	u32 line_number;		/* 0x060 */
	u32 timing_h;			/* 0x064 */
	u32 timing_v;			/* 0x068 */
	u32 pol_freq;			/* 0x06C */
	u32 divisor;			/* 0x070 */
	u32 res3[1];
	u32 size_dig;			/* 0x078 */
	u32 size_lcd;			/* 0x07C */
	u32 gfx_ba0;			/* 0x080 */
	u32 gfx_ba1;			/* 0x084 */
	u32 gfx_position;		/* 0x088 */
	u32 gfx_size;			/* 0x08C */
	u32 res4[4];
	u32 gfx_attributes;		/* 0x0A0 */
	u32 gfx_fifo_threshold;	/* 0x0A4 */
	u32 gfx_fifo_size;		/* 0x0A8 */
	u32 gfx_row_inc;		/* 0x0AC */
	u32 gfx_pixel_inc;		/* 0x0B0 */
	u32 gfx_window_skip;		/* 0x0B4 */
	u32 gfx_table_ba;		/* 0x0B8 */
	u32 vid1_ba0;			/* 0x0BC */
	u32 vid1_ba1;			/* 0x0C0 */
	u32 vid1_position;		/* 0x0C4 */
	u32 vid1_size;		/* 0x0C8 */
	u32 vid1_attributes;		/* 0x0CC */
	u32 vid1_fifo_threshold;	/* 0x0D0 */
	u32 vid1_fifo_size;		/* 0x0D4 */
	u32 vid1_row_inc;		/* 0x0D8 */
	u32 vid1_pixel_inc;		/* 0x0DC */
	u32 vid1_fir;			/* 0x0E0 */
	u32 vid1_picture_size;	/* 0x0E4 */
	u32 vid1_accu0;		/* 0x0E8 */
	u32 vid1_accu1;		/* 0x0EC */
	u32 vid1_fir_coef_h0;		/* 0x0F0 */
	u32 vid1_fir_coef_hv0;	/* 0x0F4 */
	u32 vid1_fir_coef_h1;		/* 0x0F8 */
	u32 vid1_fir_coef_hv1;	/* 0x0FC */
	u32 vid1_fir_coef_h2;		/* 0x100 */
	u32 vid1_fir_coef_hv2;	/* 0x104 */
	u32 vid1_fir_coef_h3;		/* 0x108 */
	u32 vid1_fir_coef_hv3;	/* 0x10C */
	u32 vid1_fir_coef_h4;		/* 0x110 */
	u32 vid1_fir_coef_hv4;	/* 0x114 */
	u32 vid1_fir_coef_h5;		/* 0x118 */
	u32 vid1_fir_coef_hv5;	/* 0x11C */
	u32 vid1_fir_coef_h6;		/* 0x120 */
	u32 vid1_fir_coef_hv6;	/* 0x124 */
	u32 vid1_fir_coef_h7;		/* 0x128 */
	u32 vid1_fir_coef_hv7;	/* 0x12C */
	u32 vid1_conv_coef0;		/* 0x130 */
	u32 vid1_conv_coef1;		/* 0x134 */
	u32 vid1_conv_coef2;		/* 0x138 */
	u32 vid1_conv_coef3;		/* 0x13C */
	u32 vid1_conv_coef4;		/* 0x140 */
	u32 res5[2];
	u32 vid2_ba0;			/* 0x14C */
	u32 vid2_ba1;			/* 0x150 */
	u32 vid2_position;		/* 0x154 */
	u32 vid2_size;		/* 0x158 */
	u32 vid2_attributes;		/* 0x15C */
	u32 vid2_fifo_threshold;	/* 0x160 */
	u32 vid2_fifo_size;		/* 0x164 */
	u32 vid2_row_inc;		/* 0x168 */
	u32 vid2_pixel_inc;		/* 0x16C */
	u32 vid2_fir;			/* 0x170 */
	u32 vid2_picture_size;	/* 0x174 */
	u32 vid2_accu0;		/* 0x178 */
	u32 vid2_accu1;		/* 0x17C */
	u32 vid2_fir_coef_h0;		/* 0x180 */
	u32 vid2_fir_coef_hv0;	/* 0x184 */
	u32 vid2_fir_coef_h1;		/* 0x188 */
	u32 vid2_fir_coef_hv1;	/* 0x18C */
	u32 vid2_fir_coef_h2;		/* 0x190 */
	u32 vid2_fir_coef_hv2;	/* 0x194 */
	u32 vid2_fir_coef_h3;		/* 0x198 */
	u32 vid2_fir_coef_hv3;	/* 0x19C */
	u32 vid2_fir_coef_h4;		/* 0x1A0 */
	u32 vid2_fir_coef_hv4;	/* 0x1A4 */
	u32 vid2_fir_coef_h5;		/* 0x1A8 */
	u32 vid2_fir_coef_hv5;	/* 0x1AC */
	u32 vid2_fir_coef_h6;		/* 0x1B0 */
	u32 vid2_fir_coef_hv6;	/* 0x1B4 */
	u32 vid2_fir_coef_h7;		/* 0x1B8 */
	u32 vid2_fir_coef_hv7;	/* 0x1BC */
	u32 vid2_conv_coef0;		/* 0x1C0 */
	u32 vid2_conv_coef1;		/* 0x1C4 */
	u32 vid2_conv_coef2;		/* 0x1C8 */
	u32 vid2_conv_coef3;		/* 0x1CC */
	u32 vid2_conv_coef4;		/* 0x1D0 */
	u32 data_cycle1;		/* 0x1D4 */
	u32 data_cycle2;		/* 0x1D8 */
	u32 data_cycle3;		/* 0x1DC */
#ifdef CONFIG_ARCH_OMAP3430
	/* omap3430 specific registers */
	u32 vid1_fir_coef_v0;		/* 0x1E0 */
	u32 vid1_fir_coef_v1;		/* 0x1E4 */
	u32 vid1_fir_coef_v2;		/* 0x1E8 */
	u32 vid1_fir_coef_v3;		/* 0x1EC */
	u32 vid1_fir_coef_v4;		/* 0x1F0 */
	u32 vid1_fir_coef_v5;		/* 0x1F4 */
	u32 vid1_fir_coef_v6;		/* 0x1F8 */
	u32 vid1_fir_coef_v7;		/* 0x1FC */
	u32 vid2_fir_coef_v0;		/* 0x200 */
	u32 vid2_fir_coef_v1;		/* 0x204 */
	u32 vid2_fir_coef_v2;		/* 0x208 */
	u32 vid2_fir_coef_v3;		/* 0x20C */
	u32 vid2_fir_coef_v4;		/* 0x210 */
	u32 vid2_fir_coef_v5;		/* 0x214 */
	u32 vid2_fir_coef_v6;		/* 0x218 */
	u32 vid2_fir_coef_v7;		/* 0x21C */
	u32 cpr_coef_r;			/* 0x220 */
	u32 cpr_coef_g;			/* 0x224 */
	u32 cpr_coef_b;			/* 0x228 */
	u32 gfx_preload;		/* 0x22C */
	u32 vid1_preload;		/* 0x230 */
	u32 vid2_preload;		/* 0x234 */
#endif /* CONFIG_ARCH_OMAP3430 */
};

/* WARN: read-only registers omitted! */
struct omap_dss_regs {
	u32 sysconfig;
	u32 control;
#ifdef CONFIG_ARCH_OMAP3430
	u32 sdi_control;
	u32 pll_control;
#endif
	struct omap24xx_dispc_regs dispc;
};

struct tvlcd_status_t {
	int ltype;
	int output_dev;
	int status;
}; 

/* color space conversion matrices */
const static s16 cc_bt601[3][3] = {
	{298, 409, 0}, {298, -208, -100}, {298, 0, 517}
};
const static s16 cc_bt709[3][3] = {
	{298, 459, 0}, {298, -137, -55}, {298, 0, 541}
};
const static s16 cc_bt601_full[3][3] = {
	{256, 351, 0}, {256, -179, -86}, {256, 0, 443}
};

/*----------- following are exposed values and APIs -------------------------*/

#define OMAP2_GRAPHICS		0
#define OMAP2_VIDEO1		1
#define OMAP2_VIDEO2		2
#define OMAP_DSS_GENERIC	3
#define OMAP_DSS_DISPC_GENERIC	4
#define DSS_CTX_NUMBER		(OMAP_DSS_DISPC_GENERIC + 1)

#define OMAP2_OUTPUT_LCD	4
#define OMAP2_OUTPUT_TV		5

/* Dithering enable/disable */
#define DITHERING_ON		28
#define DITHERING_OFF		29

/* TVOUT Definitions */
enum omap2_tvstandard {	
	PAL_BDGHI = 0,
	PAL_NC,
	PAL_N,
	PAL_M,
	PAL_60,
	NTSC_M,
	NTSC_J,
	NTSC_443,
};

#define WSS_OFF		0x0
#define WSS_ODD_ON		0x1
#define WSS_EVEN_ON		0x2
#define WSS_EVEN_ODD_ON		0x3


/* TV ref ON/OFF */
#define TVREF_ON		30
#define TVREF_OFF		31

/* LCD data lines configuration */
#define LCD_DATA_LINE_12BIT     32
#define LCD_DATA_LINE_16BIT     33
#define LCD_DATA_LINE_18BIT     34
#define LCD_DATA_LINE_24BIT     35

/* transparent color key types */
#define OMAP2_GFX_DESTINATION 	100
#define OMAP2_VIDEO_SOURCE	101

/* SDRAM page size parameters used for VRFB settings */
#define PAGE_WIDTH_EXP		5	/* page width = 1 << PAGE_WIDTH_EXP */
#define PAGE_HEIGHT_EXP		5	/* page height = 1 << PAGE_HEIGHT_EXP */

/* 2048 x 2048 is max res supported by OMAP24xx display controller */
#define MAX_PIXELS_PER_LINE	2048
#define MAX_LINES		2048

#define TV_OFF			0
#define TV_ON			1
#define LCD_OFF			0
#define LCD_ON			1

/* States needed for TV-LCD on the fly */
#define TVLCD_STOP		1
#define TVLCD_CONTINUE		2
#define TVLCD_OK		3

extern s16 current_colorconv_values[2][3][3];

/* input layer APIs */
extern int omap2_disp_request_layer (int ltype);
extern void omap2_disp_release_layer (int ltype);
extern void omap2_disp_disable_layer (int ltype);
extern void omap2_disp_enable_layer (int ltype);
extern void omap2_disp_config_vlayer (int ltype, struct v4l2_pix_format *pix,
					struct v4l2_rect *crop,
					struct v4l2_window *win,
					int rotation_deg, int mirroring);
extern int omap2_disp_reg_sync_bit(int output_dev);
extern void omap2_disp_config_gfxlayer (u32 size_x, u32 size_y,
					int color_depth);
extern void omap2_disp_start_vlayer (int ltype, struct v4l2_pix_format *pix,
					struct v4l2_rect *crop, struct v4l2_window *win,
					unsigned long fb_base_phys,
					int rotation_deg, int mirroring);
extern void omap2_disp_start_gfxlayer (void);

/* output device APIs */
extern void omap2_disp_get_panel_size (int output_dev, int *witdth,
					int *height);
extern void omap2_disp_set_panel_size (int output_dev, int witdth,
					int height);
extern void omap2_disp_disable_output_dev (int output_dev);
extern void omap2_disp_enable_output_dev (int output_dev);
extern void omap2_disp_config_lcd(u32 pixdiv, u32 hbp, u32 hfp, u32 hsw,
				u32 vbp, u32 vfp, u32 vsw, u32 bpp);
extern void omap2_disp_lcdcfg_polfreq(u32 hsync_high, u32 vsync_high,
		u32 acb,u32 ipc, u32 onoff);
extern void omap2_disp_set_pcd (u32 pcd);
extern void omap2_disp_set_dssfclk (void);
extern void omap2_disp_set_tvstandard (int tvstandard);
extern int omap2_disp_get_tvstandard (void);
extern void omap2_disp_get_tvlcd(struct tvlcd_status_t *status);
extern void omap2_disp_set_tvlcd(int status);
extern void omap2_disp_set_dithering (int dither_state);
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
extern void omap2_disp_set_tvref (int tvref_state);
#endif
extern int omap2_disp_get_dithering (void);

extern void omap2_disp_set_lcddatalines (int no_of_lines);
extern int omap2_disp_get_lcddatalines (void);

/* connection of input layers to output devices */
extern int omap2_disp_get_output_dev (int ltype);
extern void omap2_disp_set_output_dev (int ltype, int output_dev);
extern void omap2_disp_set_dma_params (int ltype, int output_dev,
					u32 ba0, u32 ba1, u32 row_inc,
					u32 pix_inc);

void omap2_disp_get_all_clks(void);
void omap2_disp_put_all_clks(void);

/* DSS power management */
extern void omap2_disp_get_dss (void);
extern void omap2_disp_put_dss (void);


/* Color conversion */
void omap2_disp_set_default_colorconv (int ltype,
					struct v4l2_pix_format *pix);
void omap2_disp_set_colorconv (int ltype, struct v4l2_pix_format *pix);

/* background color */
extern void omap2_disp_set_bg_color (int output_dev, int color);
extern void omap2_disp_get_bg_color (int output_dev, int *color);

/* transparent color key */
extern void omap2_disp_set_colorkey (int output_dev, int key_type,
					int key_val);
extern void omap2_disp_get_colorkey (int output_dev, int *key_type,
					int *key_val);
extern void omap2_disp_enable_colorkey (int output_dev);
extern void omap2_disp_disable_colorkey (int output_dev);

/* alpha blending */
int omap2_disp_get_alphablend(int output_dev);
void omap2_disp_set_alphablend(int output_dev,int value);
unsigned char omap2_disp_get_global_alphablend_value(int ltype);
void omap2_disp_set_global_alphablend_value(int ltype,int value);

/* other helpers */
extern void omap2_disp_set_gfx_palette (u32 palette_ba);
extern void omap2_disp_pixels_per_clock (unsigned int *nom,
					 unsigned int *den);

/* rotation APIs */
extern int omap2_disp_set_vrfb (int context, u32 phy_addr,
				u32 width, u32 height, u32 bytes_per_pixel);

/* display controller register synchronization */
void omap2_disp_reg_sync (int output_dev);
extern int omap2_disp_reg_sync_done (int output_dev);

/* disable LCD and TV outputs and sync with next frame */
extern void omap2_disp_disable (unsigned long timeout_ticks);

/* interrupt handling */
typedef void (*omap2_disp_isr_t) (void *arg, struct pt_regs * regs);
extern int omap2_disp_register_isr (omap2_disp_isr_t isr, void *arg,
					unsigned int mask);
extern int omap2_disp_unregister_isr (omap2_disp_isr_t isr);
extern int omap2_disp_irqenable(omap2_disp_isr_t isr,unsigned int mask);
extern int omap2_disp_irqdisable(omap2_disp_isr_t isr,unsigned int mask);
extern void omap2_disp_register_synclost_isr(void);
extern void omap2_disp_save_initstate (int layer);
extern void omap2_disp_restore_initstate (int layer);

/* LPR */
struct omap_dss_config {
	u32 fifo_hi_thrs; /* FIFO high threshold */
	u32 fifo_lo_thrs; /* FIFO low  threshold */
	u16 logdiv;       /* Logic clock divider (LCD) */
	u16 pixdiv;       /* Pixel clock divider (PCD) */
};

#define DISPLAY_LPR_STATE_OFF     0
#define DISPLAY_LPR_STATE_ON      1
int omap2_disp_lpr_state(void);
int omap2_disp_lpr_set_state(int state);

#define DISPLAY_LPR_MODE_MANUAL   0
#define DISPLAY_LPR_MODE_AUTO     1
int omap2_disp_lpr_mode(void);
int omap2_disp_lpr_set_mode(int mode);

int omap2_disp_lpr_is_enabled(void);
void omap2_disp_lpr_set_cfg(const struct omap_dss_config *lpr_cfg_on,
			    const struct omap_dss_config *lpr_cfg_off);
int omap2_disp_get_gfx_fifo_low_threshold(void);
void omap2_disp_set_gfx_fifo_low_threshold(int thrs);
int omap2_disp_get_gfx_fifo_high_threshold(void);
void omap2_disp_set_gfx_fifo_high_threshold(int thrs); 






void omap2_tv_clock_enable(void);
void omap2_tv_clock_disable(void);

/*------------------ end of exposed values and APIs -------------------------*/

#endif /* __ASM_ARCH_OMAP2_DISP_H */
