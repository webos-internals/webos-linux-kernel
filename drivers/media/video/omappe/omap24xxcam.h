/*
 * drivers/media/video/omap/omap24xxcam.h
 *
 * Video-for-Linux (Version 2) camera capture driver for 
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 * August 2005 - Modified for new display code.
 * January 2006 - Enhanced to support new sensor interface and preview rotation.
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 */

#ifndef OMAP24XXCAM_H
#define OMAP24XXCAM_H

/* physical memory map definitions */
	/* camera subsystem */
#define CAM_REG_BASE			0x48052000
#define CAM_REG_SIZE			0x00001000
	/* camera core */
#define CC_REG_OFFSET			0x00000400
	/* camera DMA */
#define CAMDMA_REG_OFFSET		0x00000800
	/* camera MMU */
#define CAMMMU_REG_OFFSET		0x00000C00

/* define camera subsystem register offsets */
#define CAM_REVISION			0x000
#define CAM_SYSCONFIG			0x010
#define CAM_SYSSTATUS			0x014
#define CAM_IRQSTATUS			0x018
#define CAM_GPO			0x040
#define CAM_GPI			0x050

/* define camera core register offsets */
#define CC_REVISION			0x000
#define CC_SYSCONFIG			0x010
#define CC_SYSSTATUS			0x014
#define CC_IRQSTATUS			0x018
#define CC_IRQENABLE			0x01C
#define CC_CTRL			0x040
#define CC_CTRL_DMA			0x044
#define CC_CTRL_XCLK			0x048
#define CC_FIFODATA			0x04C
#define CC_TEST			0x050
#define CC_GENPAR			0x054
#define CC_CCPFSCR			0x058
#define CC_CCPFECR			0x05C
#define CC_CCPLSCR			0x060
#define CC_CCPLECR			0x064
#define CC_CCPDFR			0x068

/* define camera dma register offsets */
#define CAMDMA_REVISION		0x000
#define CAMDMA_IRQSTATUS_L0		0x008
#define CAMDMA_IRQSTATUS_L1		0x00C
#define CAMDMA_IRQSTATUS_L2		0x010
#define CAMDMA_IRQSTATUS_L3		0x014
#define CAMDMA_IRQENABLE_L0		0x018
#define CAMDMA_IRQENABLE_L1		0x01C
#define CAMDMA_IRQENABLE_L2		0x020
#define CAMDMA_IRQENABLE_L3		0x024
#define CAMDMA_SYSSTATUS		0x028
#define CAMDMA_OCP_SYSCONFIG		0x02C
#define CAMDMA_CAPS_0			0x064
#define CAMDMA_CAPS_2			0x06C
#define CAMDMA_CAPS_3			0x070
#define CAMDMA_CAPS_4			0x074
#define CAMDMA_GCR			0x078
#define CAMDMA_CCR(n)			(0x080 + (n)*0x60)
#define CAMDMA_CLNK_CTRL(n)		(0x084 + (n)*0x60)
#define CAMDMA_CICR(n)			(0x088 + (n)*0x60)
#define CAMDMA_CSR(n)			(0x08C + (n)*0x60)
#define CAMDMA_CSDP(n)			(0x090 + (n)*0x60)
#define CAMDMA_CEN(n)			(0x094 + (n)*0x60)
#define CAMDMA_CFN(n)			(0x098 + (n)*0x60)
#define CAMDMA_CSSA(n)			(0x09C + (n)*0x60)
#define CAMDMA_CDSA(n)			(0x0A0 + (n)*0x60)
#define CAMDMA_CSEI(n)			(0x0A4 + (n)*0x60)
#define CAMDMA_CSFI(n)			(0x0A8 + (n)*0x60)
#define CAMDMA_CDEI(n)			(0x0AC + (n)*0x60)
#define CAMDMA_CDFI(n)			(0x0B0 + (n)*0x60)
#define CAMDMA_CSAC(n)			(0x0B4 + (n)*0x60)
#define CAMDMA_CDAC(n)			(0x0B8 + (n)*0x60)
#define CAMDMA_CCEN(n)			(0x0BC + (n)*0x60)
#define CAMDMA_CCFN(n)			(0x0C0 + (n)*0x60)
#define CAMDMA_COLOR(n)		(0x0C4 + (n)*0x60)

/* define camera mmu register offsets */
#define CAMMMU_REVISION		0x000
#define CAMMMU_SYSCONFIG		0x010
#define CAMMMU_SYSSTATUS		0x014
#define CAMMMU_IRQSTATUS		0x018
#define CAMMMU_IRQENABLE		0x01C
#define CAMMMU_WALKING_ST		0x040
#define CAMMMU_CNTL			0x044
#define CAMMMU_FAULT_AD		0x048
#define CAMMMU_TTB			0x04C
#define CAMMMU_LOCK			0x050
#define CAMMMU_LD_TLB			0x054
#define CAMMMU_CAM			0x058
#define CAMMMU_RAM			0x05C
#define CAMMMU_GFLUSH			0x060
#define CAMMMU_FLUSH_ENTRY		0x064
#define CAMMMU_READ_CAM		0x068
#define CAMMMU_READ_RAM		0x06C
#define CAMMMU_EMU_FAULT_AD		0x070

/* Define bit fields within selected registers */
#define CAM_REVISION_MAJOR		(15 << 4)
#define CAM_REVISION_MAJOR_SHIFT	4
#define CAM_REVISION_MINOR		(15 << 0)
#define CAM_REVISION_MINOR_SHIFT	0

#define CAM_SYSCONFIG_SOFTRESET	(1 <<  1)
#define CAM_SYSCONFIG_AUTOIDLE		(1 <<  0)

#define CAM_SYSSTATUS_RESETDONE	(1 <<  0)

#define CAM_IRQSTATUS_CC_IRQ		(1 <<  4)
#define CAM_IRQSTATUS_MMU_IRQ		(1 <<  3)
#define CAM_IRQSTATUS_DMA_IRQ2		(1 <<  2)
#define CAM_IRQSTATUS_DMA_IRQ1		(1 <<  1)
#define CAM_IRQSTATUS_DMA_IRQ0		(1 <<  0)

#define CAM_GPO_CAM_S_P_EN		(1 <<  1)
#define CAM_GPO_CAM_CCP_MODE		(1 <<  0)

#define CAM_GPI_CC_DMA_REQ1		(1 << 24)
#define CAP_GPI_CC_DMA_REQ0		(1 << 23)
#define CAP_GPI_CAM_MSTANDBY		(1 << 21)
#define CAP_GPI_CAM_WAIT		(1 << 20)
#define CAP_GPI_CAM_S_DATA		(1 << 17)
#define CAP_GPI_CAM_S_CLK		(1 << 16)
#define CAP_GPI_CAM_P_DATA		(0xFFF << 3)
#define CAP_GPI_CAM_P_DATA_SHIFT	3
#define CAP_GPI_CAM_P_VS		(1 <<  2)
#define CAP_GPI_CAM_P_HS		(1 <<  1)
#define CAP_GPI_CAM_P_CLK		(1 <<  0)

#define CC_REVISION_MAJOR		(15 << 4)
#define CC_REVISION_MAJOR_SHIFT	4
#define CC_REVISION_MINOR		(15 << 0)
#define CC_REVISION_MINOR_SHIFT	0

#define CC_SYSCONFIG_SIDLEMODE		(3 <<  3)
#define CC_SYSCONFIG_SIDLEMODE_FIDLE	(0 <<  3)
#define CC_SYSCONFIG_SIDLEMODE_NIDLE	(1 <<  3)
#define CC_SYSCONFIG_SOFTRESET		(1 <<  1)
#define CC_SYSCONFIG_AUTOIDLE		(1 <<  0)

#define CC_SYSSTATUS_RESETDONE		(1 <<  0)

#define CC_IRQSTATUS_FS_IRQ		(1 << 19)
#define CC_IRQSTATUS_LE_IRQ		(1 << 18)
#define CC_IRQSTATUS_LS_IRQ		(1 << 17)
#define CC_IRQSTATUS_FE_IRQ		(1 << 16)
#define CC_IRQSTATUS_FW_ERR_IRQ	(1 << 10)
#define CC_IRQSTATUS_FSC_ERR_IRQ	(1 <<  9)
#define CC_IRQSTATUS_SSC_ERR_IRQ	(1 <<  8)
#define CC_IRQSTATUS_FIFO_NOEMPTY_IRQ	(1 <<  4)
#define CC_IRQSTATUS_FIFO_FULL_IRQ	(1 <<  3)
#define CC_IRQSTATUS_FIFO_THR_IRQ	(1 <<  2)
#define CC_IRQSTATUS_FIFO_OF_IRQ	(1 <<  1)
#define CC_IRQSTATUS_FIFO_UF_IRQ	(1 <<  0)

#define CC_IRQENABLE_FS_IRQ		(1 << 19)
#define CC_IRQENABLE_LE_IRQ		(1 << 18)
#define CC_IRQENABLE_LS_IRQ		(1 << 17)
#define CC_IRQENABLE_FE_IRQ		(1 << 16)
#define CC_IRQENABLE_FW_ERR_IRQ	(1 << 10)
#define CC_IRQENABLE_FSC_ERR_IRQ	(1 <<  9)
#define CC_IRQENABLE_SSC_ERR_IRQ	(1 <<  8)
#define CC_IRQENABLE_FIFO_NOEMPTY_IRQ	(1 <<  4)
#define CC_IRQENABLE_FIFO_FULL_IRQ	(1 <<  3)
#define CC_IRQENABLE_FIFO_THR_IRQ	(1 <<  2)
#define CC_IRQENABLE_FIFO_OF_IRQ	(1 <<  1)
#define CC_IRQENABLE_FIFO_UF_IRQ	(1 <<  0)

#define CC_CTRL_CC_RST			(1 << 18)
#define CC_CTRL_CC_FRAME_TRIG		(1 << 17)
#define CC_CTRL_CC_EN			(1 << 16)
#define CC_CTRL_NOBT_SYNCHRO		(1 << 13)
#define CC_CTRL_BT_CORRECT		(1 << 12)
#define CC_CTRL_PAR_ORDERCAM		(1 << 11)
#define CC_CTRL_PAR_CLK_POL		(1 << 10)
#define CC_CTRL_NOBT_HS_POL_SHIFT	9
#define CC_CTRL_NOBT_HS_POL		(1 <<  9)
#define CC_CTRL_NOBT_VS_POL_SHIFT	8
#define CC_CTRL_NOBT_VS_POL		(1 <<  8)
#define CC_CTRL_PAR_MODE		(7 <<  1)
#define CC_CTRL_PAR_MODE_SHIFT		1
#define CC_CTRL_PAR_MODE_NOBT8		(0 <<  1)
#define CC_CTRL_PAR_MODE_NOBT10	(1 <<  1)
#define CC_CTRL_PAR_MODE_NOBT12	(2 <<  1)
#define CC_CTRL_PAR_MODE_BT8		(4 <<  1)
#define CC_CTRL_PAR_MODE_BT10		(5 <<  1)
#define CC_CTRL_PAR_MODE_FIFOTEST	(7 <<  1)
#define CC_CTRL_CCP_MODE		(1 <<  0)

#define CC_CTRL_DMA_EN			(1 <<  8)
#define CC_CTRL_DMA_FIFO_THRESHOLD	(0x7F << 0)
#define CC_CTRL_DMA_FIFO_THRESHOLD_SHIFT	0

#define CC_CTRL_XCLK_DIV		 	(0x1F << 0)
#define CC_CTRL_XCLK_DIV_SHIFT		 	0
#define CC_CTRL_XCLK_DIV_STABLE_LOW	 	(0 <<  0)
#define CC_CTRL_XCLK_DIV_STABLE_HIGH	 	(1 <<  0)
#define CC_CTRL_XCLK_DIV_BYPASS	 	(31 << 0)

#define CC_TEST_FIFO_RD_POINTER	 	(0xFF << 24)
#define CC_TEST_FIFO_RD_POINTER_SHIFT	 	24
#define CC_TEST_FIFO_WR_POINTER	 	(0xFF << 16)
#define CC_TEST_FIFO_WR_POINTER_SHIFT	 	16
#define CC_TEST_FIFO_LEVEL		 	(0xFF <<  8)
#define CC_TEST_FIFO_LEVEL_SHIFT		8
#define CC_TEST_FIFO_LEVEL_PEAK		(0xFF <<  0)
#define CC_TEST_FIFO_LEVEL_PEAK_SHIFT		0

#define CC_GENPAR_FIFO_DEPTH			(7 <<  0)
#define CC_GENPAR_FIFO_DEPTH_SHIFT		0

#define CC_CCPDFR_ALPHA			(0xFF <<  8)
#define CC_CCPDFR_ALPHA_SHIFT			8
#define CC_CCPDFR_DATAFORMAT			(15 <<  0)
#define CC_CCPDFR_DATAFORMAT_SHIFT		0
#define CC_CCPDFR_DATAFORMAT_YUV422BE		( 0 <<  0)
#define CC_CCPDFR_DATAFORMAT_YUV422		( 1 <<  0)
#define CC_CCPDFR_DATAFORMAT_YUV420		( 2 <<  0)
#define CC_CCPDFR_DATAFORMAT_RGB444		( 4 <<  0)
#define CC_CCPDFR_DATAFORMAT_RGB565		( 5 <<  0)
#define CC_CCPDFR_DATAFORMAT_RGB888NDE		( 6 <<  0)
#define CC_CCPDFR_DATAFORMAT_RGB888		( 7 <<  0)
#define CC_CCPDFR_DATAFORMAT_RAW8NDE		( 8 <<  0)
#define CC_CCPDFR_DATAFORMAT_RAW8		( 9 <<  0)
#define CC_CCPDFR_DATAFORMAT_RAW10NDE		(10 <<  0)
#define CC_CCPDFR_DATAFORMAT_RAW10		(11 <<  0)
#define CC_CCPDFR_DATAFORMAT_RAW12NDE		(12 <<  0)
#define CC_CCPDFR_DATAFORMAT_RAW12		(13 <<  0)
#define CC_CCPDFR_DATAFORMAT_JPEG8		(15 <<  0)

#define CAMDMA_REVISION_MAJOR			(15 << 4)
#define CAMDMA_REVISION_MAJOR_SHIFT		4
#define CAMDMA_REVISION_MINOR			(15 << 0)
#define CAMDMA_REVISION_MINOR_SHIFT		0

#define CAMDMA_OCP_SYSCONFIG_MIDLEMODE			(3 << 12)
#define CAMDMA_OCP_SYSCONFIG_MIDLEMODE_FSTANDBY	(0 << 12)
#define CAMDMA_OCP_SYSCONFIG_MIDLEMODE_NSTANDBY	(1 << 12)
#define CAMDMA_OCP_SYSCONFIG_MIDLEMODE_SSTANDBY	(2 << 12)
#define CAMDMA_OCP_SYSCONFIG_FUNC_CLOCK		(1 <<  9)
#define CAMDMA_OCP_SYSCONFIG_OCP_CLOCK			(1 <<  8)
#define CAMDMA_OCP_SYSCONFIG_EMUFREE			(1 <<  5)
#define CAMDMA_OCP_SYSCONFIG_SIDLEMODE			(3 <<  3)
#define CAMDMA_OCP_SYSCONFIG_SIDLEMODE_FIDLE		(0 <<  3)
#define CAMDMA_OCP_SYSCONFIG_SIDLEMODE_NIDLE		(1 <<  3)
#define CAMDMA_OCP_SYSCONFIG_SIDLEMODE_SIDLE		(2 <<  3)
#define CAMDMA_OCP_SYSCONFIG_SOFTRESET			(1 <<  1)
#define CAMDMA_OCP_SYSCONFIG_AUTOIDLE			(1 <<  0)

#define CAMDMA_SYSSTATUS_RESETDONE			(1 <<  0)

#define CAMDMA_GCR_ARBITRATION_RATE			(0xFF << 16)
#define CAMDMA_GCR_ARBITRATION_RATE_SHIFT		16
#define CAMDMA_GCR_MAX_CHANNEL_FIFO_DEPTH		(0xFF << 0)
#define CAMDMA_GCR_MAX_CHANNEL_FIFO_DEPTH_SHIFT	0

#define CAMDMA_CCR_SEL_SRC_DST_SYNC			(1 << 24)
#define CAMDMA_CCR_PREFETCH				(1 << 23)
#define CAMDMA_CCR_SUPERVISOR				(1 << 22)
#define CAMDMA_CCR_SECURE				(1 << 21)
#define CAMDMA_CCR_BS					(1 << 18)
#define CAMDMA_CCR_TRANSPARENT_COPY_ENABLE		(1 << 17)
#define CAMDMA_CCR_CONSTANT_FILL_ENABLE		(1 << 16)
#define CAMDMA_CCR_DST_AMODE				(3 << 14)
#define CAMDMA_CCR_DST_AMODE_CONST_ADDR		(0 << 14)
#define CAMDMA_CCR_DST_AMODE_POST_INC			(1 << 14)
#define CAMDMA_CCR_DST_AMODE_SGL_IDX			(2 << 14)
#define CAMDMA_CCR_DST_AMODE_DBL_IDX			(3 << 14)
#define CAMDMA_CCR_SRC_AMODE				(3 << 12)
#define CAMDMA_CCR_SRC_AMODE_CONST_ADDR		(0 << 12)
#define CAMDMA_CCR_SRC_AMODE_POST_INC			(1 << 12)
#define CAMDMA_CCR_SRC_AMODE_SGL_IDX			(2 << 12)
#define CAMDMA_CCR_SRC_AMODE_DBL_IDX			(3 << 12)
#define CAMDMA_CCR_WR_ACTIVE				(1 << 10)
#define CAMDMA_CCR_RD_ACTIVE				(1 <<  9)
#define CAMDMA_CCR_SUSPEND_SENSITIVE			(1 <<  8)
#define CAMDMA_CCR_ENABLE				(1 <<  7)
#define CAMDMA_CCR_PRIO				(1 <<  6)
#define CAMDMA_CCR_FS					(1 <<  5)
#define CAMDMA_CCR_SYNCHRO				((3 << 19) | (31 << 0))
#define CAMDMA_CCR_SYNCHRO_CAMERA			0x01

#define CAMDMA_CLNK_CTRL_ENABLE_LNK			(1 << 15)
#define CAMDMA_CLNK_CTRL_NEXTLCH_ID			(0x1F << 0)
#define CAMDMA_CLNK_CTRL_NEXTLCH_ID_SHIFT		0

#define CAMDMA_CICR_MISALIGNED_ERR_IE			(1 << 11)
#define CAMDMA_CICR_SUPERVISOR_ERR_IE			(1 << 10)
#define CAMDMA_CICR_SECURE_ERR_IE			(1 <<  9)
#define CAMDMA_CICR_TRANS_ERR_IE			(1 <<  8)
#define CAMDMA_CICR_PACKET_IE				(1 <<  7)
#define CAMDMA_CICR_BLOCK_IE				(1 <<  5)
#define CAMDMA_CICR_LAST_IE				(1 <<  4)
#define CAMDMA_CICR_FRAME_IE				(1 <<  3)
#define CAMDMA_CICR_HALF_IE				(1 <<  2)
#define CAMDMA_CICR_DROP_IE				(1 <<  1)

#define CAMDMA_CSR_MISALIGNED_ERR			(1 << 11)
#define CAMDMA_CSR_SUPERVISOR_ERR			(1 << 10)
#define CAMDMA_CSR_SECURE_ERR				(1 <<  9)
#define CAMDMA_CSR_TRANS_ERR				(1 <<  8)
#define CAMDMA_CSR_PACKET				(1 <<  7)
#define CAMDMA_CSR_SYNC				(1 <<  6)
#define CAMDMA_CSR_BLOCK				(1 <<  5)
#define CAMDMA_CSR_LAST				(1 <<  4)
#define CAMDMA_CSR_FRAME				(1 <<  3)
#define CAMDMA_CSR_HALF				(1 <<  2)
#define CAMDMA_CSR_DROP				(1 <<  1)

#define CAMDMA_CSDP_SRC_ENDIANNESS			(1 << 21)
#define CAMDMA_CSDP_SRC_ENDIANNESS_LOCK		(1 << 20)
#define CAMDMA_CSDP_DST_ENDIANNESS			(1 << 19)
#define CAMDMA_CSDP_DST_ENDIANNESS_LOCK		(1 << 18)
#define CAMDMA_CSDP_WRITE_MODE				(3 << 16)
#define CAMDMA_CSDP_WRITE_MODE_WRNP			(0 << 16)
#define CAMDMA_CSDP_WRITE_MODE_POSTED			(1 << 16)
#define CAMDMA_CSDP_WRITE_MODE_POSTED_LAST_WRNP	(2 << 16)
#define CAMDMA_CSDP_DST_BURST_EN			(3 << 14)
#define CAMDMA_CSDP_DST_BURST_EN_1			(0 << 14)
#define CAMDMA_CSDP_DST_BURST_EN_16			(1 << 14)
#define CAMDMA_CSDP_DST_BURST_EN_32			(2 << 14)
#define CAMDMA_CSDP_DST_BURST_EN_64			(3 << 14)
#define CAMDMA_CSDP_DST_PACKED				(1 << 13)
#define CAMDMA_CSDP_WR_ADD_TRSLT			(15 << 9)
#define CAMDMA_CSDP_WR_ADD_TRSLT_ENABLE_MREQADD	(3 <<  9)
#define CAMDMA_CSDP_SRC_BURST_EN			(3 <<  7)
#define CAMDMA_CSDP_SRC_BURST_EN_1			(0 <<  7)
#define CAMDMA_CSDP_SRC_BURST_EN_16			(1 <<  7)
#define CAMDMA_CSDP_SRC_BURST_EN_32			(2 <<  7)
#define CAMDMA_CSDP_SRC_BURST_EN_64			(3 <<  7)
#define CAMDMA_CSDP_SRC_PACKED				(1 <<  6)
#define CAMDMA_CSDP_RD_ADD_TRSLT			(15 << 2)
#define CAMDMA_CSDP_RD_ADD_TRSLT_ENABLE_MREQADD	(3 <<  2)
#define CAMDMA_CSDP_DATA_TYPE				(3 <<  0)
#define CAMDMA_CSDP_DATA_TYPE_8BITS			(0 <<  0)
#define CAMDMA_CSDP_DATA_TYPE_16BITS			(1 <<  0)
#define CAMDMA_CSDP_DATA_TYPE_32BITS			(2 <<  0)

#define CAMMMU_SYSCONFIG_AUTOIDLE			(1 <<  0)

struct omap24xx_cc_regs {
	u32 revision;			/* 0x000 */
	u32 res1[3];
	u32 sysconfig;			/* 0x010 */
	u32 sysstatus;			/* 0x014 */
	u32 irqstatus;			/* 0x018 */
	u32 irqenable;			/* 0x01C */
	u32 res2[8];
	u32 ctrl;			/* 0x040 */
	u32 ctrl_dma;			/* 0x044 */
	u32 ctrl_xclk;			/* 0x048 */
	u32 fifodata;			/* 0x04C */
	u32 test;			/* 0x050 */
	u32 genpar;			/* 0x054 */
	u32 ccpfscr;			/* 0x058 */
	u32 ccpfecr;			/* 0x05C */
	u32 ccplscr;			/* 0x060 */
	u32 ccplecr;			/* 0x064 */
	u32 ccpdfr;			/* 0x068 */
};

/* forward declarations */
struct omap24xxcam_fh;
struct omap24xxcam_device;


/* camera DMA definitions */
#define DMA_THRESHOLD 32 /* number of bytes transferred per DMA request */
/* NUM_CAMDMA_CHANNELS is the number of logical channels provided by the camera 
 * DMA controller.
 */
#define NUM_CAMDMA_CHANNELS 4
/* NUM_SG_DMA is the number of scatter-gather DMA transfers that can be queued.
 * We need it to be 2 greater than the maximum number of video frames so that 
 * we can use 2 slots for overlay while still having VIDEO_MAX_FRAME slots left 
 * for streaming.
 */
#define NUM_SG_DMA (VIDEO_MAX_FRAME+2)

typedef void (*dma_callback_t)(struct omap24xxcam_device *cam, 
	unsigned long status, void *arg);

struct camdma_state {
	dma_callback_t callback;
	void *arg;
};
struct sgdma_state {
	const struct scatterlist *sglist;
	int sglen;		/* number of sglist entries */
	int next_sglist;	/* index of next sglist entry to process */
	int queued_sglist;	/* number of sglist entries queued for DMA */
	unsigned long csr;	/* DMA return code */
	dma_callback_t callback;
	void *arg;
};

/* per-device data structure */
struct omap24xxcam_device {
	struct clk *cami;
	struct clk *camf;
	unsigned int irq;
	
	unsigned long cam_mmio_base;
	unsigned long cam_mmio_base_phys;
	unsigned long cam_mmio_size;
	
	unsigned long overlay_base;
	unsigned long overlay_base_phys;
	unsigned long overlay_size;
	unsigned long overlay_base_dma;
	int overlay_rotation;

	/* camera DMA management */
	spinlock_t dma_lock;
	/* While dma_stop!=0, an attempt to start a new DMA transfer will 
	 * fail.
	 */
	int dma_stop;
	int free_dmach;	/* number of dma channels free */
	int next_dmach; /* index of next dma channel to use */
	struct camdma_state camdma[NUM_CAMDMA_CHANNELS];
	/* dma_notify is a pointer to a callback routine for notification when 
	 * a DMA transfer has been started.
	 */
	void (*dma_notify)(struct omap24xxcam_device *cam);

	/* frequncy (in Hz) of camera interface functional clock (MCLK) */
	unsigned long mclk;

	struct device dev;
	struct video_device *vfd;

	spinlock_t overlay_lock;	/* spinlock for overlay DMA counter */
	int overlay_cnt;		/* count of queued overlay DMA xfers */
	struct scatterlist overlay_sglist;

	spinlock_t vbq_lock;		/* spinlock for videobuf queues */
	struct videobuf_queue_ops vbq_ops;	/* videobuf queue operations */
	unsigned long field_count;	/* field counter for videobuf_buffer */

	/* scatter-gather DMA management */
	spinlock_t sg_lock;
	int free_sgdma;	/* number of free sg dma slots */
	int next_sgdma;	/* index of next sg dma slot to use */
	struct sgdma_state sgdma[NUM_SG_DMA];

	/* The img_lock is used to serialize access to the image parameters for 
	 * overlay and capture.
	 */
	spinlock_t img_lock;

 	/* Access to everything below here is locked by img_lock */
 	
 	/* We allow streaming from at most one filehandle at a time.  
 	 * non-NULL means streaming is in progress.
 	 */
	struct omap24xxcam_fh *streaming;
	/* We allow previewing from at most one filehandle at a time.  
	 * non-NULL means previewing is in progress.
	 */
	struct omap24xxcam_fh *previewing;

	/* capture parameters (frame rate, number of buffers) */
	struct v4l2_captureparm cparm;
	struct v4l2_captureparm cparm2;

	/* This is the frame period actually requested by the user. */
	struct v4l2_fract nominal_timeperframe;

	/* frequency (in Hz) of camera interface xclk output */
	unsigned long xclk;

	/* pointer to camera sensor interface interface */
	struct camera_sensor *cam_sensor;
	/* blind pointer to private data structure for sensor */
	void *sensor;
	
	/* pix defines the size and pixel format of the image captured by the 
	 * sensor.  This also defines the size of the framebuffers.  The 
	 * same pool of framebuffers is used for video capture and video 
	 * overlay.  These parameters are set/queried by the 
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with a CAPTURE buffer type.
	 */
	struct v4l2_pix_format pix;
	struct v4l2_pix_format pix2;
	int still_capture;

	/* preview_crop defines the size and offset of the video overlay source window 
	 * within the framebuffer.  These parameters are set/queried by the 
	 * VIDIOC_S_CROP/VIDIOC_G_CROP ioctls with an OVERLAY buffer type.  
	 * The cropping rectangle allows a subset of the captured image to be 
	 * previewed.  It only affects the portion of the image previewed, not 
	 * captured; the cropping of the captured image is done seperately.
	 */
	struct v4l2_rect preview_crop;

	/* win defines the size and offset of the video overlay target window 
	 * within the video display.  These parameters are set/queried by the 
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with an OVERLAY buffer type.
	 */
	struct v4l2_window win;

	/* fbuf reflects the size of the video display.  It is queried with the 
	 * VIDIOC_G_FBUF ioctl.  The size of the video display cannot be 
	 * changed with the VIDIOC_S_FBUF ioctl.
	 */
	struct v4l2_framebuffer fbuf;


	/* value of CC_CTRL register required to support current capture 
	 * format
	 */
	unsigned long cc_ctrl;

	/* the display controller video layer for camera preview
	 */
	int vid_preview;

	/* Power management suspend lockout */
	int suspended;
	wait_queue_head_t suspend_wq;

};

/* per-filehandle data structure */
struct omap24xxcam_fh {
	struct omap24xxcam_device *cam;
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
};

#endif	/* ifndef OMAP24XXCAM_H */
