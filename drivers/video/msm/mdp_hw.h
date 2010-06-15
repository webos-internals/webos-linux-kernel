/* drivers/video/msm_fb/mdp_hw.h
 *
 * Copyright (C) 2007 QUALCOMM Incorporated
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _MDP_HW_H_
#define _MDP_HW_H_

#include <asm/arch/msm_iomap.h>
#include <linux/types.h>
#include <asm/io.h>

/* Pulled from "mdp.h" in qualcomm's display driver */
#define inpw(port)             readw(port)
#define outpw(port, val)       writew(val, port)
#define inpdw(port)            readl(port)
#define outpdw(port, val)      writel(val, port)

#define outp32(addr, val) writel(val, addr)
#define outp16(addr, val) writew(val, addr)
#define outp8(addr, val) writeb(val, addr)
#define outp(addr, val) outp32(addr,val)

#define MDP_OUTP(addr, data) outpdw((addr), (data))


#define MDP_CSC_PFMVn(n)	(MSM_MDP_BASE + 0x40400 + 4 * (n))
#define MDP_CSC_PRMVn(n)	(MSM_MDP_BASE + 0x40440 + 4 * (n))
#define MDP_CSC_PBVn(n)		(MSM_MDP_BASE + 0x40500 + 4 * (n))

#ifdef CONFIG_FB_MSM_MDP31
#define MDP_CSC_PLVn(n)		(MSM_MDP_BASE + 0x40600 + 4 * (n))
#define MDP_CSC_PLV_POSTn(n)		(MSM_MDP_BASE + 0x40680 + 4 * (n))
#define MDP_PPP_SCALE_COEFF_LSBn(n)		(MSM_MDP_BASE + 0x50400 + 8 * (n))
#define MDP_PPP_SCALE_COEFF_MSBn(n)		(MSM_MDP_BASE + 0x50404 + 8 * (n))

#define SCALE_D0_SET  0
#define SCALE_D1_SET  BIT(0)
#define SCALE_D2_SET  BIT(1)
#define SCALE_U1_SET  (BIT(0)|BIT(1))

#else
#define MDP_CSC_PLVn(n)		(MSM_MDP_BASE + 0x40580 + 4 * (n))
#endif

/* -----------------------------------------------------------------------
** {3x3} + {3} ccs matrix
** ----------------------------------------------------------------------- */
typedef struct mdp_ccs_type_ {
	u16 ccs1;
	u16 ccs2;
	u16 ccs3;
	u16 ccs4;
	u16 ccs5;
	u16 ccs6;
	u16 ccs7;
	u16 ccs8;
	u16 ccs9;
	u16 ccs10;
	u16 ccs11;
	u16 ccs12;
} MDP_CCS_TYPE;


#define MDP_CMD_DEBUG_ACCESS_BASE   (MSM_MDP_BASE+0x10000)

/* END port from "mdp.h" */

#if defined(CONFIG_ARCH_MSM7X25) // MSM7x25

#define MDP_INTR_ENABLE                  (MSM_MDP_BASE + 0x00020)
#define MDP_INTR_STATUS                  (MSM_MDP_BASE + 0x00024)
#define MDP_INTR_CLEAR                   (MSM_MDP_BASE + 0x00028)
#define MDP_DMA_START                    (MSM_MDP_BASE + 0x00044)

#define MDP_EBI2_PORTMAP_MODE            (MSM_MDP_BASE + 0x0005c)
#define MDP_MODE                         (MSM_MDP_BASE + 0x00060)

/* MDP_PPP_SRC_SIZE */
#define MDP_FULL_BYPASS_WORD2			(MSM_MDP_BASE + 0x10108)
/* MDP_PPP_SRCP0_ADDR */
#define MDP_FULL_BYPASS_WORD3			(MSM_MDP_BASE + 0x1010C)
/* MDP_PPP_SRCP1_ADDR */
#define MDP_FULL_BYPASS_WORD4			(MSM_MDP_BASE + 0x10110)
/* MDP_PPP_SRC_YSTRIDE1 */
#define MDP_FULL_BYPASS_WORD7			(MSM_MDP_BASE + 0x1011C)
/* MDP_PPP_SRC_FORMAT */
#define MDP_FULL_BYPASS_WORD9			(MSM_MDP_BASE + 0x10124)
/* MDP_PPP_SRC_UNPACK_PATTERN1 */
#define MDP_FULL_BYPASS_WORD10			(MSM_MDP_BASE + 0x10128)
/* MDP_PPP_OP_MODE */
#define MDP_FULL_BYPASS_WORD14			(MSM_MDP_BASE + 0x10138)
/* MDP_PPP_SCALE_PHASEX_INIT */
#define MDP_FULL_BYPASS_WORD15			(MSM_MDP_BASE + 0x1013C)
/* MDP_PPP_SCALE_PHASEY_INIT */
#define MDP_FULL_BYPASS_WORD16			(MSM_MDP_BASE + 0x10140)
/* MDP_PPP_SCALE_PHASEX_STEP */
#define MDP_FULL_BYPASS_WORD17			(MSM_MDP_BASE + 0x10144)
/* MDP_PPP_SCALE_PHASEY_STEP */
#define MDP_FULL_BYPASS_WORD18			(MSM_MDP_BASE + 0x10148)
/* MDP_PPP_BLEND_PARAM */
#define MDP_FULL_BYPASS_WORD19			(MSM_MDP_BASE + 0x1014C)
/* MDP_PPP_OUT_FORMAT */
/*
 * TODO: bit definition is different betweeen 7201 and 7x25, need to fix 
 * the references to this register.
 */
#define MDP_FULL_BYPASS_WORD20			(MSM_MDP_BASE + 0x10150)

/* MDP_PPP_OUT_PACK_PATTERN1 */
#define MDP_FULL_BYPASS_WORD21			(MSM_MDP_BASE + 0x10154)
/* MDP_PPP_OUT_SIZE */
#define MDP_FULL_BYPASS_WORD25			(MSM_MDP_BASE + 0x10164)
/* MDP_PPP_OUTP0_ADDR */
#define MDP_FULL_BYPASS_WORD26			(MSM_MDP_BASE + 0x10168)
/* MDP_PPP_OUTP1_ADDR */
#define MDP_FULL_BYPASS_WORD27			(MSM_MDP_BASE + 0x1016C)
/* MDP_PPP_OUT_YSTRIDE1 */
#define MDP_FULL_BYPASS_WORD30			(MSM_MDP_BASE + 0x10178)
/* MDP_PPP_SRC_EDGE_REP */
#define MDP_FULL_BYPASS_WORD46			(MSM_MDP_BASE + 0x101B8)
/* MDP_PPP_BGP0_ADDR */
#define MDP_FULL_BYPASS_WORD48			(MSM_MDP_BASE + 0x101C0)
/* MDP_PPP_BGP1_ADDR */
#define MDP_FULL_BYPASS_WORD49			(MSM_MDP_BASE + 0x101C4)
/* MDP_PPP_BG_YSTRIDE1 */
#define MDP_FULL_BYPASS_WORD51			(MSM_MDP_BASE + 0x101CC)
/* MDP_PPP_BG_FORMAT */
#define MDP_FULL_BYPASS_WORD53			(MSM_MDP_BASE + 0x101D4)
/* MDP_PPP_BG_UNPACK_PATTERN1 */
#define MDP_FULL_BYPASS_WORD54			(MSM_MDP_BASE + 0x101D8)
/* MDP_DISPLAY0_START */
#define MDP_DISPLAY0_START			(MSM_MDP_BASE + 0x0030)


/*
drivers/video/msm/mdp_ppp.c:790: error: 'MDP_FULL_BYPASS_WORD3' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:812: error: 'MDP_FULL_BYPASS_WORD30' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:814: error: 'MDP_FULL_BYPASS_WORD46' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:816: error: 'MDP_FULL_BYPASS_WORD48' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:817: error: 'MDP_FULL_BYPASS_WORD49' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:818: error: 'MDP_FULL_BYPASS_WORD51' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:819: error: 'MDP_FULL_BYPASS_WORD53' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:820: error: 'MDP_FULL_BYPASS_WORD54' undeclared (first use in this function)
		drivers/video/msm/mdp_ppp.c:823: error: 'MDP_DISPLAY0_START' undeclared (first use in this function)
*/
#define MDP_DMA_CONFIG		                (MSM_MDP_BASE + 0x90000)
#define MDP_FULL_BYPASS_WORD33                  (MSM_MDP_BASE + 0x90004)
#define MDP_FULL_BYPASS_WORD34                  (MSM_MDP_BASE + 0x90008)
#define MDP_FULL_BYPASS_WORD35                  (MSM_MDP_BASE + 0x9000C)
#define MDP_FULL_BYPASS_WORD37                  (MSM_MDP_BASE + 0x90010)
#define MDP_FULL_BYPASS_WORD40                  (MSM_MDP_BASE + 0x00090)
#define MDP_FULL_BYPASS_WORD41                  (MSM_MDP_BASE + 0x00094)
#define MDP_FULL_BYPASS_WORD43           	(MSM_MDP_BASE + 0x0009C)
/* MDP_INTR_ENABLE */
#define DL0_ROI_DONE           (1<<0)
#define DL1_ROI_DONE           (1<<1)
#define S_DMA2_DONE                       (1<<2)
#define E_DMA2_DONE                       (1<<3)
#define DL0_PPP_TERM_DONE      (1<<4)
#define DL1_PPP_TERM_DONE      (1<<5)
#define TV_OUT_DMA3_DONE       (1<<6)
#define TV_ENC_UNDERRUN        (1<<7)
#define DL0_FETCH_DONE         (1<<11)
#define DL1_FETCH_DONE         (1<<12)
#define P_DMA2_DONE                       (1<<14)

#define DL0_DMA2_TERM_DONE     P_DMA2_DONE
#define DL1_DMA2_TERM_DONE     S_DMA2_DONE

#else // MSM7200
#define MDP_SYNC_CONFIG_0                (MSM_MDP_BASE + 0x00000)
#define MDP_SYNC_CONFIG_1                (MSM_MDP_BASE + 0x00004)
#define MDP_SYNC_CONFIG_2                (MSM_MDP_BASE + 0x00008)
#define MDP_SYNC_STATUS_0                (MSM_MDP_BASE + 0x0000c)
#define MDP_SYNC_STATUS_1                (MSM_MDP_BASE + 0x00010)
#define MDP_SYNC_STATUS_2                (MSM_MDP_BASE + 0x00014)
#define MDP_SYNC_THRESH_0                (MSM_MDP_BASE + 0x00018)
#define MDP_SYNC_THRESH_1                (MSM_MDP_BASE + 0x0001c)
#define MDP_INTR_ENABLE                  (MSM_MDP_BASE + 0x00020)
#define MDP_INTR_STATUS                  (MSM_MDP_BASE + 0x00024)
#define MDP_INTR_CLEAR                   (MSM_MDP_BASE + 0x00028)
#define MDP_DISPLAY0_START               (MSM_MDP_BASE + 0x00030)
#define MDP_DISPLAY1_START               (MSM_MDP_BASE + 0x00034)
#define MDP_DISPLAY_STATUS               (MSM_MDP_BASE + 0x00038)
#define MDP_EBI2_LCD0                    (MSM_MDP_BASE + 0x0003c)
#define MDP_EBI2_LCD1                    (MSM_MDP_BASE + 0x00040)
#define MDP_DISPLAY0_ADDR                (MSM_MDP_BASE + 0x00054)
#define MDP_DISPLAY1_ADDR                (MSM_MDP_BASE + 0x00058)
#define MDP_EBI2_PORTMAP_MODE            (MSM_MDP_BASE + 0x0005c)
#define MDP_MODE                         (MSM_MDP_BASE + 0x00060)
#define MDP_TV_OUT_STATUS                (MSM_MDP_BASE + 0x00064)
#define MDP_HW_VERSION                   (MSM_MDP_BASE + 0x00070)
#define MDP_SW_RESET                     (MSM_MDP_BASE + 0x00074)
#define MDP_AXI_ERROR_MASTER_STOP        (MSM_MDP_BASE + 0x00078)
#define MDP_SEL_CLK_OR_HCLK_TEST_BUS     (MSM_MDP_BASE + 0x0007c)
#define MDP_PRIMARY_VSYNC_OUT_CTRL       (MSM_MDP_BASE + 0x00080)
#define MDP_SECONDARY_VSYNC_OUT_CTRL     (MSM_MDP_BASE + 0x00084)
#define MDP_EXTERNAL_VSYNC_OUT_CTRL      (MSM_MDP_BASE + 0x00088)
#define MDP_VSYNC_CTRL                   (MSM_MDP_BASE + 0x0008c)
#define MDP_CGC_EN                       (MSM_MDP_BASE + 0x00100)
#define MDP_CMD_STATUS                   (MSM_MDP_BASE + 0x10008)
#define MDP_PROFILE_EN                   (MSM_MDP_BASE + 0x10010)
#define MDP_PROFILE_COUNT                (MSM_MDP_BASE + 0x10014)
#define MDP_DMA_START                    (MSM_MDP_BASE + 0x10044)
#define MDP_FULL_BYPASS_WORD0            (MSM_MDP_BASE + 0x10100)
#define MDP_FULL_BYPASS_WORD1            (MSM_MDP_BASE + 0x10104)
#define MDP_COMMAND_CONFIG               (MSM_MDP_BASE + 0x10104)
#define MDP_FULL_BYPASS_WORD2            (MSM_MDP_BASE + 0x10108)
#define MDP_FULL_BYPASS_WORD3            (MSM_MDP_BASE + 0x1010c)
#define MDP_FULL_BYPASS_WORD4            (MSM_MDP_BASE + 0x10110)
#define MDP_FULL_BYPASS_WORD6            (MSM_MDP_BASE + 0x10118)
#define MDP_FULL_BYPASS_WORD7            (MSM_MDP_BASE + 0x1011c)
#define MDP_FULL_BYPASS_WORD8            (MSM_MDP_BASE + 0x10120)
#define MDP_FULL_BYPASS_WORD9            (MSM_MDP_BASE + 0x10124)
#define MDP_PPP_SOURCE_CONFIG            (MSM_MDP_BASE + 0x10124)
#define MDP_FULL_BYPASS_WORD10           (MSM_MDP_BASE + 0x10128)
#define MDP_FULL_BYPASS_WORD11           (MSM_MDP_BASE + 0x1012c)
#define MDP_FULL_BYPASS_WORD12           (MSM_MDP_BASE + 0x10130)
#define MDP_FULL_BYPASS_WORD13           (MSM_MDP_BASE + 0x10134)
#define MDP_FULL_BYPASS_WORD14           (MSM_MDP_BASE + 0x10138)
#define MDP_PPP_OPERATION_CONFIG         (MSM_MDP_BASE + 0x10138)
#define MDP_FULL_BYPASS_WORD15           (MSM_MDP_BASE + 0x1013c)
#define MDP_FULL_BYPASS_WORD16           (MSM_MDP_BASE + 0x10140)
#define MDP_FULL_BYPASS_WORD17           (MSM_MDP_BASE + 0x10144)
#define MDP_FULL_BYPASS_WORD18           (MSM_MDP_BASE + 0x10148)
#define MDP_FULL_BYPASS_WORD19           (MSM_MDP_BASE + 0x1014c)
#define MDP_FULL_BYPASS_WORD20           (MSM_MDP_BASE + 0x10150)
#define MDP_PPP_DESTINATION_CONFIG       (MSM_MDP_BASE + 0x10150)
#define MDP_FULL_BYPASS_WORD21           (MSM_MDP_BASE + 0x10154)
#define MDP_FULL_BYPASS_WORD22           (MSM_MDP_BASE + 0x10158)
#define MDP_FULL_BYPASS_WORD23           (MSM_MDP_BASE + 0x1015c)
#define MDP_FULL_BYPASS_WORD24           (MSM_MDP_BASE + 0x10160)
#define MDP_FULL_BYPASS_WORD25           (MSM_MDP_BASE + 0x10164)
#define MDP_FULL_BYPASS_WORD26           (MSM_MDP_BASE + 0x10168)
#define MDP_FULL_BYPASS_WORD27           (MSM_MDP_BASE + 0x1016c)
#define MDP_FULL_BYPASS_WORD29           (MSM_MDP_BASE + 0x10174)
#define MDP_FULL_BYPASS_WORD30           (MSM_MDP_BASE + 0x10178)
#define MDP_FULL_BYPASS_WORD31           (MSM_MDP_BASE + 0x1017c)
#define MDP_FULL_BYPASS_WORD32           (MSM_MDP_BASE + 0x10180)
#define MDP_DMA_CONFIG                   (MSM_MDP_BASE + 0x10180)
#define MDP_FULL_BYPASS_WORD33           (MSM_MDP_BASE + 0x10184)
#define MDP_FULL_BYPASS_WORD34           (MSM_MDP_BASE + 0x10188)
#define MDP_FULL_BYPASS_WORD35           (MSM_MDP_BASE + 0x1018c)
#define MDP_FULL_BYPASS_WORD37           (MSM_MDP_BASE + 0x10194)
#define MDP_FULL_BYPASS_WORD39           (MSM_MDP_BASE + 0x1019c)
#define MDP_FULL_BYPASS_WORD40           (MSM_MDP_BASE + 0x101a0)
#define MDP_FULL_BYPASS_WORD41           (MSM_MDP_BASE + 0x101a4)
#define MDP_FULL_BYPASS_WORD43           (MSM_MDP_BASE + 0x101ac)
#define MDP_FULL_BYPASS_WORD46           (MSM_MDP_BASE + 0x101b8)
#define MDP_FULL_BYPASS_WORD47           (MSM_MDP_BASE + 0x101bc)
#define MDP_FULL_BYPASS_WORD48           (MSM_MDP_BASE + 0x101c0)
#define MDP_FULL_BYPASS_WORD49           (MSM_MDP_BASE + 0x101c4)
#define MDP_FULL_BYPASS_WORD50           (MSM_MDP_BASE + 0x101c8)
#define MDP_FULL_BYPASS_WORD51           (MSM_MDP_BASE + 0x101cc)
#define MDP_FULL_BYPASS_WORD52           (MSM_MDP_BASE + 0x101d0)
#define MDP_FULL_BYPASS_WORD53           (MSM_MDP_BASE + 0x101d4)
#define MDP_FULL_BYPASS_WORD54           (MSM_MDP_BASE + 0x101d8)
#define MDP_FULL_BYPASS_WORD55           (MSM_MDP_BASE + 0x101dc)
#define MDP_FULL_BYPASS_WORD56           (MSM_MDP_BASE + 0x101e0)
#define MDP_FULL_BYPASS_WORD57           (MSM_MDP_BASE + 0x101e4)
#define MDP_FULL_BYPASS_WORD58           (MSM_MDP_BASE + 0x101e8)
#define MDP_FULL_BYPASS_WORD59           (MSM_MDP_BASE + 0x101ec)
#define MDP_FULL_BYPASS_WORD60           (MSM_MDP_BASE + 0x101f0)
#define MDP_VSYNC_THRESHOLD              (MSM_MDP_BASE + 0x101f0)
#define MDP_FULL_BYPASS_WORD61           (MSM_MDP_BASE + 0x101f4)
#define MDP_FULL_BYPASS_WORD62           (MSM_MDP_BASE + 0x101f8)
#define MDP_FULL_BYPASS_WORD63           (MSM_MDP_BASE + 0x101fc)
#define MDP_TFETCH_TEST_MODE             (MSM_MDP_BASE + 0x20004)
#define MDP_TFETCH_STATUS                (MSM_MDP_BASE + 0x20008)
#define MDP_TFETCH_TILE_COUNT            (MSM_MDP_BASE + 0x20010)
#define MDP_TFETCH_FETCH_COUNT           (MSM_MDP_BASE + 0x20014)
#define MDP_TFETCH_CONSTANT_COLOR        (MSM_MDP_BASE + 0x20040)
#define MDP_CSC_BYPASS                   (MSM_MDP_BASE + 0x40004)
#define MDP_SCALE_COEFF_LSB              (MSM_MDP_BASE + 0x5fffc)
#define MDP_TV_OUT_CTL                   (MSM_MDP_BASE + 0xc0000)
#define MDP_TV_OUT_FIR_COEFF             (MSM_MDP_BASE + 0xc0004)
#define MDP_TV_OUT_BUF_ADDR              (MSM_MDP_BASE + 0xc0008)
#define MDP_TV_OUT_CC_DATA               (MSM_MDP_BASE + 0xc000c)
#define MDP_TV_OUT_SOBEL                 (MSM_MDP_BASE + 0xc0010)
#define MDP_TV_OUT_Y_CLAMP               (MSM_MDP_BASE + 0xc0018)
#define MDP_TV_OUT_CB_CLAMP              (MSM_MDP_BASE + 0xc001c)
#define MDP_TV_OUT_CR_CLAMP              (MSM_MDP_BASE + 0xc0020)
#define MDP_TEST_MODE_CLK                (MSM_MDP_BASE + 0xd0000)
#define MDP_TEST_MISR_RESET_CLK          (MSM_MDP_BASE + 0xd0004)
#define MDP_TEST_EXPORT_MISR_CLK         (MSM_MDP_BASE + 0xd0008)
#define MDP_TEST_MISR_CURR_VAL_CLK       (MSM_MDP_BASE + 0xd000c)
#define MDP_TEST_MODE_HCLK               (MSM_MDP_BASE + 0xd0100)
#define MDP_TEST_MISR_RESET_HCLK         (MSM_MDP_BASE + 0xd0104)
#define MDP_TEST_EXPORT_MISR_HCLK        (MSM_MDP_BASE + 0xd0108)
#define MDP_TEST_MISR_CURR_VAL_HCLK      (MSM_MDP_BASE + 0xd010c)
#define MDP_TEST_MODE_DCLK               (MSM_MDP_BASE + 0xd0200)
#define MDP_TEST_MISR_RESET_DCLK         (MSM_MDP_BASE + 0xd0204)
#define MDP_TEST_EXPORT_MISR_DCLK        (MSM_MDP_BASE + 0xd0208)
#define MDP_TEST_MISR_CURR_VAL_DCLK      (MSM_MDP_BASE + 0xd020c)
#define MDP_TEST_CAPTURED_DCLK           (MSM_MDP_BASE + 0xd0210)
#define MDP_TEST_MISR_CAPT_VAL_DCLK      (MSM_MDP_BASE + 0xd0214)
#define MDP_LCDC_CTL                     (MSM_MDP_BASE + 0xe0000)
#define MDP_LCDC_HSYNC_CTL               (MSM_MDP_BASE + 0xe0004)
#define MDP_LCDC_VSYNC_CTL               (MSM_MDP_BASE + 0xe0008)
#define MDP_LCDC_ACTIVE_HCTL             (MSM_MDP_BASE + 0xe000c)
#define MDP_LCDC_ACTIVE_VCTL             (MSM_MDP_BASE + 0xe0010)
#define MDP_LCDC_BORDER_CLR              (MSM_MDP_BASE + 0xe0014)
#define MDP_LCDC_H_BLANK                 (MSM_MDP_BASE + 0xe0018)
#define MDP_LCDC_V_BLANK                 (MSM_MDP_BASE + 0xe001c)
#define MDP_LCDC_UNDERFLOW_CLR           (MSM_MDP_BASE + 0xe0020)
#define MDP_LCDC_HSYNC_SKEW              (MSM_MDP_BASE + 0xe0024)
#define MDP_LCDC_TEST_CTL                (MSM_MDP_BASE + 0xe0028)
#define MDP_LCDC_LINE_IRQ                (MSM_MDP_BASE + 0xe002c)
#define MDP_LCDC_CTL_POLARITY            (MSM_MDP_BASE + 0xe0030)
#define MDP_LCDC_DMA_CONFIG              (MSM_MDP_BASE + 0xe1000)
#define MDP_LCDC_DMA_SIZE                (MSM_MDP_BASE + 0xe1004)
#define MDP_LCDC_DMA_IBUF_ADDR           (MSM_MDP_BASE + 0xe1008)
#define MDP_LCDC_DMA_IBUF_Y_STRIDE       (MSM_MDP_BASE + 0xe100c)


/* MDP_INTR_ENABLE */
#define DL0_ROI_DONE           (1<<0)
#define DL1_ROI_DONE           (1<<1)
#define DL0_DMA2_TERM_DONE     (1<<2)
#define DL1_DMA2_TERM_DONE     (1<<3)
#define DL0_PPP_TERM_DONE      (1<<4)
#define DL1_PPP_TERM_DONE      (1<<5)
#define TV_OUT_DMA3_DONE       (1<<6)
#define TV_ENC_UNDERRUN        (1<<7)
#define DL0_FETCH_DONE         (1<<11)
#define DL1_FETCH_DONE         (1<<12)
#endif

#define MDP_DMA2_TERM 0x1
#define MDP_DMA3_TERM 0x2
#define MDP_PPP_TERM 0x3


#define MDP_PPP_BUSY_STATUS (DL0_ROI_DONE| \
			   DL1_ROI_DONE| \
			   DL0_PPP_TERM_DONE| \
			   DL1_PPP_TERM_DONE)

#define MDP_ANY_INTR_MASK (DL0_ROI_DONE| \
			   DL1_ROI_DONE| \
			   DL0_DMA2_TERM_DONE| \
			   DL1_DMA2_TERM_DONE| \
			   DL0_PPP_TERM_DONE| \
			   DL1_PPP_TERM_DONE| \
			   DL0_FETCH_DONE| \
			   DL1_FETCH_DONE| \
			   TV_ENC_UNDERRUN)

#define MDP_TOP_LUMA       16
#define MDP_TOP_CHROMA     0
#define MDP_BOTTOM_LUMA    19
#define MDP_BOTTOM_CHROMA  3
#define MDP_LEFT_LUMA      22
#define MDP_LEFT_CHROMA    6
#define MDP_RIGHT_LUMA     25
#define MDP_RIGHT_CHROMA   9

#define CLR_G 0x0
#define CLR_B 0x1
#define CLR_R 0x2
#define CLR_ALPHA 0x3

#define CLR_Y  CLR_G
#define CLR_CB CLR_B
#define CLR_CR CLR_R

/* from lsb to msb */
#define MDP_GET_PACK_PATTERN(a, x, y, z, bit) \
	(((a)<<(bit*3))|((x)<<(bit*2))|((y)<<bit)|(z))

/* MDP_SYNC_CONFIG_0/1/2 */
#define MDP_SYNCFG_HGT_LOC 22
#define MDP_SYNCFG_VSYNC_EXT_EN (1<<21)
#define MDP_SYNCFG_VSYNC_INT_EN (1<<20)

/* MDP_SYNC_THRESH_0 */
#define MDP_PRIM_BELOW_LOC 0
#define MDP_PRIM_ABOVE_LOC 8

/* MDP_{PRIMARY,SECONDARY,EXTERNAL}_VSYNC_OUT_CRL */
#define VSYNC_PULSE_EN (1<<31)
#define VSYNC_PULSE_INV (1<<30)

/* MDP_VSYNC_CTRL */
#define DISP0_VSYNC_MAP_VSYNC0 0
#define DISP0_VSYNC_MAP_VSYNC1 (1<<0)
#define DISP0_VSYNC_MAP_VSYNC2 (1<<0)|(1<<1)

#define DISP1_VSYNC_MAP_VSYNC0 0
#define DISP1_VSYNC_MAP_VSYNC1 (1<<2)
#define DISP1_VSYNC_MAP_VSYNC2 (1<<2)|(1<<3)

#define PRIMARY_LCD_SYNC_EN (1<<4)
#define PRIMARY_LCD_SYNC_DISABLE 0

#define SECONDARY_LCD_SYNC_EN (1<<5)
#define SECONDARY_LCD_SYNC_DISABLE 0

#define EXTERNAL_LCD_SYNC_EN (1<<6)
#define EXTERNAL_LCD_SYNC_DISABLE 0

/* MDP_VSYNC_THRESHOLD / MDP_FULL_BYPASS_WORD60 */
#define VSYNC_THRESHOLD_ABOVE_LOC 0
#define VSYNC_THRESHOLD_BELOW_LOC 16
#define VSYNC_ANTI_TEAR_EN (1<<31)

/* MDP_COMMAND_CONFIG / MDP_FULL_BYPASS_WORD1 */
#define MDP_CMD_DBGBUS_EN (1<<0)

/* MDP_PPP_SOURCE_CONFIG / MDP_FULL_BYPASS_WORD9&53 */
#define PPP_SRC_C0G_8BIT ((1<<1)|(1<<0))
#define PPP_SRC_C1B_8BIT ((1<<3)|(1<<2))
#define PPP_SRC_C2R_8BIT ((1<<5)|(1<<4))
#define PPP_SRC_C3A_8BIT ((1<<7)|(1<<6))

#define PPP_SRC_C0G_6BIT (1<<1)
#define PPP_SRC_C1B_6BIT (1<<3)
#define PPP_SRC_C2R_6BIT (1<<5)

#define PPP_SRC_C0G_5BIT (1<<0)
#define PPP_SRC_C1B_5BIT (1<<2)
#define PPP_SRC_C2R_5BIT (1<<4)

#define PPP_SRC_C3ALPHA_EN (1<<8)

#define PPP_SRC_BPP_1BYTES 0
#define PPP_SRC_BPP_2BYTES (1<<9)
#define PPP_SRC_BPP_3BYTES (1<<10)
#define PPP_SRC_BPP_4BYTES ((1<<10)|(1<<9))

#define PPP_SRC_BPP_ROI_ODD_X (1<<11)
#define PPP_SRC_BPP_ROI_ODD_Y (1<<12)
#define PPP_SRC_INTERLVD_2COMPONENTS (1<<13)
#define PPP_SRC_INTERLVD_3COMPONENTS (1<<14)
#define PPP_SRC_INTERLVD_4COMPONENTS ((1<<14)|(1<<13))


/* RGB666 unpack format
** TIGHT means R6+G6+B6 together
** LOOSE means R6+2 +G6+2+ B6+2 (with MSB)
**          or 2+R6 +2+G6 +2+B6 (with LSB)
*/
#define PPP_SRC_PACK_TIGHT (1<<17)
#define PPP_SRC_PACK_LOOSE 0
#define PPP_SRC_PACK_ALIGN_LSB 0
#define PPP_SRC_PACK_ALIGN_MSB (1<<18)

#define PPP_SRC_PLANE_INTERLVD 0
#define PPP_SRC_PLANE_PSEUDOPLNR (1<<20)

#define PPP_SRC_WMV9_MODE (1<<21)

/* MDP_PPP_OPERATION_CONFIG / MDP_FULL_BYPASS_WORD14 */
#define PPP_OP_SCALE_X_ON (1<<0)
#define PPP_OP_SCALE_Y_ON (1<<1)

#define PPP_OP_CONVERT_RGB2YCBCR 0
#define PPP_OP_CONVERT_YCBCR2RGB (1<<2)
#define PPP_OP_CONVERT_ON (1<<3)

#define PPP_OP_CONVERT_MATRIX_PRIMARY 0
#define PPP_OP_CONVERT_MATRIX_SECONDARY (1<<4)

#define PPP_OP_LUT_C0_ON (1<<5)
#define PPP_OP_LUT_C1_ON (1<<6)
#define PPP_OP_LUT_C2_ON (1<<7)

/* rotate or blend enable */
#define PPP_OP_ROT_ON (1<<8)

#define PPP_OP_ROT_90 (1<<9)
#define PPP_OP_FLIP_LR (1<<10)
#define PPP_OP_FLIP_UD (1<<11)

#define PPP_OP_BLEND_ON (1<<12)

#define PPP_OP_BLEND_SRCPIXEL_ALPHA 0
#define PPP_OP_BLEND_DSTPIXEL_ALPHA (1<<13)
#define PPP_OP_BLEND_CONSTANT_ALPHA (1<<14)
#define PPP_OP_BLEND_SRCPIXEL_TRANSP ((1<<13)|(1<<14))

#define PPP_OP_BLEND_ALPHA_BLEND_NORMAL 0
#define PPP_OP_BLEND_ALPHA_BLEND_REVERSE (1<<15)

#define PPP_OP_DITHER_EN (1<<16)

#define PPP_OP_COLOR_SPACE_RGB 0
#define PPP_OP_COLOR_SPACE_YCBCR (1<<17)

#define PPP_OP_SRC_CHROMA_RGB 0
#define PPP_OP_SRC_CHROMA_H2V1 (1<<18)
#define PPP_OP_SRC_CHROMA_H1V2 (1<<19)
#define PPP_OP_SRC_CHROMA_420 ((1<<18)|(1<<19))
#define PPP_OP_SRC_CHROMA_COSITE 0
#define PPP_OP_SRC_CHROMA_OFFSITE (1<<20)

#define PPP_OP_DST_CHROMA_RGB 0
#define PPP_OP_DST_CHROMA_H2V1 (1<<21)
#define PPP_OP_DST_CHROMA_H1V2 (1<<22)
#define PPP_OP_DST_CHROMA_420 ((1<<21)|(1<<22))
#define PPP_OP_DST_CHROMA_COSITE 0
#define PPP_OP_DST_CHROMA_OFFSITE (1<<23)

#define PPP_BLEND_ALPHA_TRANSP (1<<24)

#define PPP_OP_BG_CHROMA_RGB 0
#define PPP_OP_BG_CHROMA_H2V1 (1<<25)
#define PPP_OP_BG_CHROMA_H1V2 (1<<26)
#define PPP_OP_BG_CHROMA_420 (1<<25)|(1<<26)
#define PPP_OP_BG_CHROMA_SITE_COSITE 0
#define PPP_OP_BG_CHROMA_SITE_OFFSITE (1<<27)

/* MDP_PPP_DESTINATION_CONFIG / MDP_FULL_BYPASS_WORD20 */
#define PPP_DST_C0G_8BIT ((1<<0)|(1<<1))
#define PPP_DST_C1B_8BIT ((1<<3)|(1<<2))
#define PPP_DST_C2R_8BIT ((1<<5)|(1<<4))
#define PPP_DST_C3A_8BIT ((1<<7)|(1<<6))

#define PPP_DST_C0G_6BIT (1<<1)
#define PPP_DST_C1B_6BIT (1<<3)
#define PPP_DST_C2R_6BIT (1<<5)

#define PPP_DST_C0G_5BIT (1<<0)
#define PPP_DST_C1B_5BIT (1<<2)
#define PPP_DST_C2R_5BIT (1<<4)

#define PPP_DST_C3A_8BIT ((1<<7)|(1<<6))
#define PPP_DST_C3ALPHA_EN (1<<8)

#define PPP_DST_INTERLVD_2COMPONENTS (1<<9)
#define PPP_DST_INTERLVD_3COMPONENTS (1<<10)
#define PPP_DST_INTERLVD_4COMPONENTS ((1<<10)|(1<<9))
#define PPP_DST_INTERLVD_6COMPONENTS ((1<<11)|(1<<9))

#define PPP_DST_PACK_LOOSE 0
#define PPP_DST_PACK_TIGHT (1<<13)
#define PPP_DST_PACK_ALIGN_LSB 0
#define PPP_DST_PACK_ALIGN_MSB (1<<14)

#define PPP_DST_OUT_SEL_AXI 0
#define PPP_DST_OUT_SEL_MDDI (1<<15)

#define PPP_DST_BPP_2BYTES (1<<16)
#define PPP_DST_BPP_3BYTES (1<<17)
#define PPP_DST_BPP_4BYTES ((1<<17)|(1<<16))

#define PPP_DST_PLANE_INTERLVD 0
#define PPP_DST_PLANE_PLANAR (1<<18)
#define PPP_DST_PLANE_PSEUDOPLNR (1<<19)

#define PPP_DST_TO_TV (1<<20)

#define PPP_DST_MDDI_PRIMARY 0
#define PPP_DST_MDDI_SECONDARY (1<<21)
#define PPP_DST_MDDI_EXTERNAL (1<<22)


/* PPP Background config */
#define PPP_BG_C0G_8BIT ((1<<0)|(1<<1))
#define PPP_BG_C1B_8BIT ((1<<3)|(1<<2))
#define PPP_BG_C2R_8BIT ((1<<5)|(1<<4))
#define PPP_BG_C3A_8BIT ((1<<7)|(1<<6))

#define PPP_BG_C0G_6BIT (1<<1)
#define PPP_BG_C1B_6BIT (1<<3)
#define PPP_BG_C2R_6BIT (1<<5)

#define PPP_BG_C0G_5BIT (1<<0)
#define PPP_BG_C1B_5BIT (1<<2)
#define PPP_BG_C2R_5BIT (1<<4)

#define PPP_BG_C3A_8BIT ((1<<7)|(1<<6))
#define PPP_BG_C3ALPHA_EN (1<<8)

#define PPP_BG_BPP_2BYTES (1<<9)
#define PPP_BG_BPP_3BYTES (1<<10)
#define PPP_BG_BPP_4BYTES ((1<<9)|(1<<10))

#define PPP_BG_INTERLVD_2COMPONENTS (1<<13)
#define PPP_BG_INTERLVD_3COMPONENTS (1<<14)
#define PPP_BG_INTERLVD_4COMPONENTS ((1<<14)|(1<<13))
#define PPP_BG_INTERLVD_6COMPONENTS ((1<<15)|(1<<13))

#define PPP_BG_PACK_LOOSE 0
#define PPP_BG_PACK_TIGHT (1<<17)
#define PPP_BG_PACK_ALIGN_LSB 0
#define PPP_BG_PACK_ALIGN_MSB (1<<18)

#define PPP_BG_PLANE_INTERLVD 0
#define PPP_BG_PLANE_PLANAR (1<<19)
#define PPP_BG_PLANE_PSEUDOPLNR (1<<20)

#define PPP_BG_WMV9_MODE_ON (1<<21)

#define PPP_BG_DONOTROTATE (1<<22)


/* image configurations by image type */
#define PPP_CFG_MDP_RGB_565(dir)	PPP_##dir##_C2R_5BIT | \
					PPP_##dir##_C0G_6BIT | \
					PPP_##dir##_C1B_5BIT | \
					PPP_##dir##_BPP_2BYTES | \
					PPP_##dir##_INTERLVD_3COMPONENTS | \
					PPP_##dir##_PACK_TIGHT | \
					PPP_##dir##_PACK_ALIGN_LSB | \
					PPP_##dir##_PLANE_INTERLVD

#define PPP_CFG_MDP_RGB_888(dir)	PPP_##dir##_C2R_8BIT | \
					PPP_##dir##_C0G_8BIT | \
					PPP_##dir##_C1B_8BIT | \
					PPP_##dir##_BPP_3BYTES | \
					PPP_##dir##_INTERLVD_3COMPONENTS | \
					PPP_##dir##_PACK_TIGHT | \
					PPP_##dir##_PACK_ALIGN_LSB | \
					PPP_##dir##_PLANE_INTERLVD

#define PPP_CFG_MDP_ARGB_8888(dir)	PPP_##dir##_C2R_8BIT | \
					PPP_##dir##_C0G_8BIT | \
					PPP_##dir##_C1B_8BIT | \
					PPP_##dir##_C3A_8BIT | \
					PPP_##dir##_C3ALPHA_EN | \
					PPP_##dir##_BPP_4BYTES | \
					PPP_##dir##_INTERLVD_4COMPONENTS | \
					PPP_##dir##_PACK_TIGHT | \
					PPP_##dir##_PACK_ALIGN_LSB | \
					PPP_##dir##_PLANE_INTERLVD

#define PPP_CFG_MDP_XRGB_8888(dir) PPP_CFG_MDP_ARGB_8888(dir)
#define PPP_CFG_MDP_RGBA_8888(dir) PPP_CFG_MDP_ARGB_8888(dir)

#define PPP_CFG_MDP_Y_CBCR_H2V2(dir)	PPP_##dir##_C2R_8BIT | \
					PPP_##dir##_C0G_8BIT | \
					PPP_##dir##_C1B_8BIT | \
					PPP_##dir##_C3A_8BIT | \
					PPP_##dir##_BPP_2BYTES | \
					PPP_##dir##_INTERLVD_2COMPONENTS | \
					PPP_##dir##_PACK_TIGHT | \
					PPP_##dir##_PACK_ALIGN_LSB | \
					PPP_##dir##_PLANE_PSEUDOPLNR

#define PPP_CFG_MDP_Y_CRCB_H2V2(dir)	PPP_CFG_MDP_Y_CBCR_H2V2(dir)

#define PPP_CFG_MDP_YCRYCB_H2V1(dir)	PPP_##dir##_C2R_8BIT | \
					PPP_##dir##_C0G_8BIT | \
					PPP_##dir##_C1B_8BIT | \
					PPP_##dir##_C3A_8BIT | \
					PPP_##dir##_BPP_2BYTES | \
					PPP_##dir##_INTERLVD_4COMPONENTS | \
					PPP_##dir##_PACK_TIGHT | \
					PPP_##dir##_PACK_ALIGN_LSB |\
					PPP_##dir##_PLANE_INTERLVD

#define PPP_CFG_MDP_Y_CBCR_H2V1(dir)	PPP_##dir##_C2R_8BIT | \
					PPP_##dir##_C0G_8BIT | \
					PPP_##dir##_C1B_8BIT | \
					PPP_##dir##_C3A_8BIT | \
					PPP_##dir##_BPP_2BYTES |   \
					PPP_##dir##_INTERLVD_2COMPONENTS |  \
					PPP_##dir##_PACK_TIGHT | \
					PPP_##dir##_PACK_ALIGN_LSB | \
					PPP_##dir##_PLANE_PSEUDOPLNR

#define PPP_CFG_MDP_Y_CRCB_H2V1(dir)	PPP_CFG_MDP_Y_CBCR_H2V1(dir)

#define PPP_PACK_PATTERN_MDP_RGB_565 \
	MDP_GET_PACK_PATTERN(0, CLR_R, CLR_G, CLR_B, 8)
#define PPP_PACK_PATTERN_MDP_RGB_888 PPP_PACK_PATTERN_MDP_RGB_565
#define PPP_PACK_PATTERN_MDP_XRGB_8888 \
	MDP_GET_PACK_PATTERN(CLR_ALPHA, CLR_R, CLR_G, CLR_B, 8)
#define PPP_PACK_PATTERN_MDP_ARGB_8888 PPP_PACK_PATTERN_MDP_XRGB_8888
#define PPP_PACK_PATTERN_MDP_RGBA_8888 \
	MDP_GET_PACK_PATTERN(CLR_ALPHA, CLR_B, CLR_G, CLR_R, 8)
#define PPP_PACK_PATTERN_MDP_Y_CBCR_H2V1 \
	MDP_GET_PACK_PATTERN(0, 0, CLR_CB, CLR_CR, 8)
#define PPP_PACK_PATTERN_MDP_Y_CBCR_H2V2 PPP_PACK_PATTERN_MDP_Y_CBCR_H2V1
#define PPP_PACK_PATTERN_MDP_Y_CRCB_H2V1 \
	MDP_GET_PACK_PATTERN(0, 0, CLR_CR, CLR_CB, 8)
#define PPP_PACK_PATTERN_MDP_Y_CRCB_H2V2 PPP_PACK_PATTERN_MDP_Y_CRCB_H2V1
#define PPP_PACK_PATTERN_MDP_YCRYCB_H2V1 \
	MDP_GET_PACK_PATTERN(CLR_Y, CLR_R, CLR_Y, CLR_B, 8)

#define PPP_CHROMA_SAMP_MDP_RGB_565(dir) PPP_OP_##dir##_CHROMA_RGB
#define PPP_CHROMA_SAMP_MDP_RGB_888(dir) PPP_OP_##dir##_CHROMA_RGB
#define PPP_CHROMA_SAMP_MDP_XRGB_8888(dir) PPP_OP_##dir##_CHROMA_RGB
#define PPP_CHROMA_SAMP_MDP_ARGB_8888(dir) PPP_OP_##dir##_CHROMA_RGB
#define PPP_CHROMA_SAMP_MDP_RGBA_8888(dir) PPP_OP_##dir##_CHROMA_RGB
#define PPP_CHROMA_SAMP_MDP_Y_CBCR_H2V1(dir) PPP_OP_##dir##_CHROMA_H2V1
#define PPP_CHROMA_SAMP_MDP_Y_CBCR_H2V2(dir) PPP_OP_##dir##_CHROMA_420
#define PPP_CHROMA_SAMP_MDP_Y_CRCB_H2V1(dir) PPP_OP_##dir##_CHROMA_H2V1
#define PPP_CHROMA_SAMP_MDP_Y_CRCB_H2V2(dir) PPP_OP_##dir##_CHROMA_420
#define PPP_CHROMA_SAMP_MDP_YCRYCB_H2V1(dir) PPP_OP_##dir##_CHROMA_H2V1

/* Helpful array generation macros */
#define PPP_ARRAY0(name) \
	[MDP_RGB_565] = PPP_##name##_MDP_RGB_565,\
	[MDP_RGB_888] = PPP_##name##_MDP_RGB_888,\
	[MDP_XRGB_8888] = PPP_##name##_MDP_XRGB_8888,\
	[MDP_ARGB_8888] = PPP_##name##_MDP_ARGB_8888,\
	[MDP_RGBA_8888] = PPP_##name##_MDP_RGBA_8888,\
	[MDP_Y_CBCR_H2V1] = PPP_##name##_MDP_Y_CBCR_H2V1,\
	[MDP_Y_CBCR_H2V2] = PPP_##name##_MDP_Y_CBCR_H2V2,\
	[MDP_Y_CRCB_H2V1] = PPP_##name##_MDP_Y_CRCB_H2V1,\
	[MDP_Y_CRCB_H2V2] = PPP_##name##_MDP_Y_CRCB_H2V2,\
	[MDP_YCRYCB_H2V1] = PPP_##name##_MDP_YCRYCB_H2V1

#define PPP_ARRAY1(name, dir) \
	[MDP_RGB_565] = PPP_##name##_MDP_RGB_565(dir),\
	[MDP_RGB_888] = PPP_##name##_MDP_RGB_888(dir),\
	[MDP_XRGB_8888] = PPP_##name##_MDP_XRGB_8888(dir),\
	[MDP_ARGB_8888] = PPP_##name##_MDP_ARGB_8888(dir),\
	[MDP_RGBA_8888] = PPP_##name##_MDP_RGBA_8888(dir),\
	[MDP_Y_CBCR_H2V1] = PPP_##name##_MDP_Y_CBCR_H2V1(dir),\
	[MDP_Y_CBCR_H2V2] = PPP_##name##_MDP_Y_CBCR_H2V2(dir),\
	[MDP_Y_CRCB_H2V1] = PPP_##name##_MDP_Y_CRCB_H2V1(dir),\
	[MDP_Y_CRCB_H2V2] = PPP_##name##_MDP_Y_CRCB_H2V2(dir),\
	[MDP_YCRYCB_H2V1] = PPP_##name##_MDP_YCRYCB_H2V1(dir)

#define IS_YCRCB(img) ((img == MDP_Y_CRCB_H2V2) | (img == MDP_Y_CBCR_H2V2) | \
		       (img == MDP_Y_CRCB_H2V1) | (img == MDP_Y_CBCR_H2V1) | \
		       (img == MDP_YCRYCB_H2V1))
#define IS_RGB(img) ((img == MDP_RGB_565) | (img == MDP_RGB_888) | \
		     (img == MDP_ARGB_8888) | (img == MDP_RGBA_8888) | \
		     (img == MDP_XRGB_8888))

#define IS_PSEUDOPLNR(img) ((img == MDP_Y_CRCB_H2V2) | \
			    (img == MDP_Y_CBCR_H2V2) | \
			    (img == MDP_Y_CRCB_H2V1) | \
			    (img == MDP_Y_CBCR_H2V1))

/* Mappings from addr to purpose */
#define PPP_ADDR_SRC_ROI		MDP_FULL_BYPASS_WORD2
#define PPP_ADDR_SRC0			MDP_FULL_BYPASS_WORD3
#define PPP_ADDR_SRC1			MDP_FULL_BYPASS_WORD4
#define PPP_ADDR_SRC_YSTRIDE		MDP_FULL_BYPASS_WORD7
#define PPP_ADDR_SRC_CFG		MDP_FULL_BYPASS_WORD9
#define PPP_ADDR_SRC_PACK_PATTERN	MDP_FULL_BYPASS_WORD10
#define PPP_ADDR_OPERATION		MDP_FULL_BYPASS_WORD14
#define PPP_ADDR_PHASEX_INIT		MDP_FULL_BYPASS_WORD15
#define PPP_ADDR_PHASEY_INIT		MDP_FULL_BYPASS_WORD16
#define PPP_ADDR_PHASEX_STEP		MDP_FULL_BYPASS_WORD17
#define PPP_ADDR_PHASEY_STEP		MDP_FULL_BYPASS_WORD18
#define PPP_ADDR_ALPHA_TRANSP		MDP_FULL_BYPASS_WORD19
#define PPP_ADDR_DST_CFG		MDP_FULL_BYPASS_WORD20
#define PPP_ADDR_DST_PACK_PATTERN	MDP_FULL_BYPASS_WORD21
#define PPP_ADDR_DST_ROI		MDP_FULL_BYPASS_WORD25
#define PPP_ADDR_DST0			MDP_FULL_BYPASS_WORD26
#define PPP_ADDR_DST1			MDP_FULL_BYPASS_WORD27
#define PPP_ADDR_DST_YSTRIDE		MDP_FULL_BYPASS_WORD30
#define PPP_ADDR_EDGE			MDP_FULL_BYPASS_WORD46
#define PPP_ADDR_BG0			MDP_FULL_BYPASS_WORD48
#define PPP_ADDR_BG1			MDP_FULL_BYPASS_WORD49
#define PPP_ADDR_BG_YSTRIDE		MDP_FULL_BYPASS_WORD51
#define PPP_ADDR_BG_CFG			MDP_FULL_BYPASS_WORD53
#define PPP_ADDR_BG_PACK_PATTERN	MDP_FULL_BYPASS_WORD54

/* MDP_DMA_CONFIG / MDP_FULL_BYPASS_WORD32 */
#define DMA_DSTC0G_6BITS (1<<1)
#define DMA_DSTC1B_6BITS (1<<3)
#define DMA_DSTC2R_6BITS (1<<5)
#define DMA_DSTC0G_5BITS (1<<0)
#define DMA_DSTC1B_5BITS (1<<2)
#define DMA_DSTC2R_5BITS (1<<4)

#define DMA_PACK_TIGHT (1<<6)
#define DMA_PACK_LOOSE 0
#define DMA_PACK_ALIGN_LSB 0
#define DMA_PACK_ALIGN_MSB (1<<7)
#define DMA_PACK_PATTERN_RGB \
	(MDP_GET_PACK_PATTERN(0, CLR_R, CLR_G, CLR_B, 2)<<8)

#define DMA_OUT_SEL_AHB  0
#if defined(CONFIG_ARCH_MSM7X25)
#define DMA_OUT_SEL_MDDI (1<<19)
#else
#define DMA_OUT_SEL_MDDI (1<<14)
#endif
#define DMA_AHBM_LCD_SEL_PRIMARY 0
#define DMA_AHBM_LCD_SEL_SECONDARY (1<<15)
#define DMA_IBUF_C3ALPHA_EN (1<<16)
#if defined(CONFIG_ARCH_MSM7X25)
#define DMA_DITHER_EN (1<<24)
#else
#define DMA_DITHER_EN (1<<17)
#endif

#define DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY 0
#define DMA_MDDI_DMAOUT_LCD_SEL_SECONDARY (1<<18)
#define DMA_MDDI_DMAOUT_LCD_SEL_EXTERNAL (1<<19)

#if defined(CONFIG_ARCH_MSM7X25)
#define DMA_IBUF_FORMAT_RGB565 (1<<25)
#define DMA_IBUF_FORMAT_ARGB8888 (2<<25)
#define DMA_IBUF_FORMAT_RGB888 0
#else
#define DMA_IBUF_FORMAT_RGB565 (1<<20)
#endif
#define DMA_IBUF_FORMAT_RGB888_OR_ARGB8888 0

#define DMA_IBUF_NONCONTIGUOUS (1<<21)

/* MDDI REGISTER ? */
#define MDDI_VDO_PACKET_DESC  0x5666
#define MDDI_VDO_PACKET_PRIM  0xC3
#define MDDI_VDO_PACKET_SECD  0xC0

#endif
