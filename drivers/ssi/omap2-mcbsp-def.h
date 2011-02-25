/*
 * drivers/ssi/omap2-mcbsp-def.h
 *
 * Header file for OMAP-2 McBSP driver
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Nov 2006 - added 3430 support
 */

#ifndef __ASM_ARCH_OMAP2_McBSPDEF_H
#define __ASM_ARCH_OMAP2_McBSPDEF_H

#include <asm/arch/hardware.h>
#include <asm/arch/irqs.h>
#include <asm/arch/dma.h>
#include <asm/arch/mux.h>
#include <asm/arch/omap2_mcbsp.h>
#ifdef CONFIG_TRACK_RESOURCES
#include <linux/device.h>
#endif

#define MCBSP_POWER	1
#define OMAP24XX_LOGICAL_DMA_CH_COUNT   32
#define MCBSP_FIFO_SIZE 64
#define MCBSP2_FIFO_SIZE 1024
#define MCBSP2_SYSCONFIG_LVL1 1
#define MCBSP2_SYSCONFIG_LVL2 2

/* Base Address for each MCBSP interface */
#define OMAP24XX_VA_SYSTEM_CONTROL_BASE        IO_ADDRESS(OMAP24XX_CTRL_BASE)

/* OMAP2/3 Register definitions */
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430) 

#ifdef CONFIG_ARCH_OMAP2430
#define OMAP2_MCBSP1_BASE_PHY		(L4_24XX_BASE + 0x74000)
#define OMAP2_MCBSP2_BASE_PHY		(L4_24XX_BASE + 0x76000)
#define OMAP2_MCBSP3_BASE_PHY		(L4_24XX_BASE + 0x8C000)
#define OMAP2_MCBSP4_BASE_PHY		(L4_24XX_BASE + 0x8E000)
#define OMAP2_MCBSP5_BASE_PHY		(L4_24XX_BASE + 0x96000)
#else
#define OMAP2_MCBSP1_BASE_PHY		(L4_34XX_BASE + 0x74000)
#define OMAP2_MCBSP2_BASE_PHY		(L4_PER_34XX_BASE + 0x22000)
#define OMAP2_MCBSP3_BASE_PHY		(L4_PER_34XX_BASE + 0x24000)
#define OMAP2_MCBSP4_BASE_PHY		(L4_PER_34XX_BASE + 0x26000)
#define OMAP2_MCBSP5_BASE_PHY		(L4_34XX_BASE + 0x96000)
#define OMAP2_MCBSP2_ST_BASE_PHY	(L4_PER_BASE + 0x28000)
#define OMAP2_MCBSP3_ST_BASE_PHY	(L4_PER_BASE + 0x2A000)
#endif

#define OMAP2_MCBSP1_BASE		IO_ADDRESS(OMAP2_MCBSP1_BASE_PHY)
#define OMAP2_MCBSP2_BASE		IO_ADDRESS(OMAP2_MCBSP2_BASE_PHY)
#define OMAP2_MCBSP3_BASE		IO_ADDRESS(OMAP2_MCBSP3_BASE_PHY)
#define OMAP2_MCBSP4_BASE		IO_ADDRESS(OMAP2_MCBSP4_BASE_PHY)
#define OMAP2_MCBSP5_BASE		IO_ADDRESS(OMAP2_MCBSP5_BASE_PHY)
#define OMAP2_MCBSP2_ST_BASE		IO_ADDRESS(OMAP2_MCBSP2_ST_BASE_PHY)
#define OMAP2_MCBSP3_ST_BASE		IO_ADDRESS(OMAP2_MCBSP3_ST_BASE_PHY)
#define OMAP2_MCBSP_REG_DRR		0x00
#define OMAP2_MCBSP_REG_DXR		0x08

#else

#define OMAP2_MCBSP1_BASE_PHY		0x48074000
#define OMAP2_MCBSP2_BASE_PHY		0x48076000

#define OMAP2_MCBSP1_BASE		IO_ADDRESS(OMAP2_MCBSP1_BASE_PHY)
#define OMAP2_MCBSP2_BASE		IO_ADDRESS(OMAP2_MCBSP2_BASE_PHY)

#define OMAP2_MCBSP_REG_DRR2		0x00
#define OMAP2_MCBSP_REG_DRR1		0x04
#define OMAP2_MCBSP_REG_DXR2		0x08
#define OMAP2_MCBSP_REG_DXR1		0x0C

#endif

#define OMAP2_MCBSP_REG_SPCR2		0x10
#define OMAP2_MCBSP_REG_SPCR1		0x14
#define OMAP2_MCBSP_REG_RCR2		0x18
#define OMAP2_MCBSP_REG_RCR1		0x1C
#define OMAP2_MCBSP_REG_XCR2		0x20
#define OMAP2_MCBSP_REG_XCR1		0x24
#define OMAP2_MCBSP_REG_SRGR2		0x28
#define OMAP2_MCBSP_REG_SRGR1		0x2C
#define OMAP2_MCBSP_REG_MCR2		0x30
#define OMAP2_MCBSP_REG_MCR1		0x34
#define OMAP2_MCBSP_REG_RCERA		0x38
#define OMAP2_MCBSP_REG_RCERB		0x3C
#define OMAP2_MCBSP_REG_XCERA		0x40
#define OMAP2_MCBSP_REG_XCERB		0x44
#define OMAP2_MCBSP_REG_PCR		0x48
#define OMAP2_MCBSP_REG_RCERC		0x4C
#define OMAP2_MCBSP_REG_RCERD		0x50
#define OMAP2_MCBSP_REG_XCERC		0x54
#define OMAP2_MCBSP_REG_XCERD		0x58
#define OMAP2_MCBSP_REG_RCERE		0x5C
#define OMAP2_MCBSP_REG_RCERF		0x60
#define OMAP2_MCBSP_REG_XCERE		0x64
#define OMAP2_MCBSP_REG_XCERF		0x68
#define OMAP2_MCBSP_REG_RCERG 		0x6C
#define OMAP2_MCBSP_REG_RCERH 		0x70
#define OMAP2_MCBSP_REG_XCERG 		0x74
#define OMAP2_MCBSP_REG_XCERH 		0x78
#define OMAP2_MCBSP_REG_REV 		0x7C
#define OMAP2_MCBSP_REG_RINTCLR 	0x80
#define OMAP2_MCBSP_REG_XINTCLR 	0x84
#define OMAP2_MCBSP_REG_ROVFLCLR 	0x88
#define OMAP2_MCBSP_REG_SYSCONFIG 	0x8C
#define OMAP2_MCBSP_REG_THRSH2 		0x90
#define OMAP2_MCBSP_REG_THRSH1 		0x94
#define OMAP2_MCBSP_REG_IRQSTATUS 	0xA0
#define OMAP2_MCBSP_REG_IRQENABLE	0xA4
#define OMAP2_MCBSP_REG_WAKEUPEN	0xA8
#define OMAP2_MCBSP_REG_XCCR		0xAC
#define OMAP2_MCBSP_REG_RCCR		0xB0
#define OMAP2_MCBSP_REG_XBUFFSTAT	0xB4
#define OMAP2_MCBSP_REG_RBUFFSTAT	0xB8

#define OMAP2_MCBSP_REG_ST_SYSCONFIG	0x10
/* Field definitions */
#define OMAP2_MCBSP_ST_AUTO_IDLE	0x1

#define OMAP2_MCBSP_BIT(ARG)		((0x01)<<(ARG))

/* Field masks for Register SPCR1 */
#define ALB				OMAP2_MCBSP_BIT(15) 
#define RJUST(ARG)			(((ARG) & 0x03) << 13)
#define CLKSTP(ARG)			(((ARG) & 0x03) << 11)
#define DXENA				OMAP2_MCBSP_BIT(7)
#define RINTM(ARG)			(((ARG) & 0x03) << 4)
#define RSYNCERR			OMAP2_MCBSP_BIT(3)
#define RFULL				OMAP2_MCBSP_BIT(2)
#define RRDY				OMAP2_MCBSP_BIT(1)
#define RRST				(0x01)

/* Field masks for Register SPCR2 */
#define FREE				OMAP2_MCBSP_BIT(9)
#define SOFT				OMAP2_MCBSP_BIT(8)
#define FRST				OMAP2_MCBSP_BIT(7)
#define GRST				OMAP2_MCBSP_BIT(6)
#define XINTM(ARG)			(((ARG) & 0x03) << 4)
#define XSYNCERR			OMAP2_MCBSP_BIT(3)
#define XEMPTY				OMAP2_MCBSP_BIT(2)
#define XRDY				OMAP2_MCBSP_BIT(1)
#define XRST				(0x01)

/* Field masks for Register RCR1 */
#define RFRLEN1(ARG)			(((ARG) & 0x7F) << 8)
#define RWDLEN1(ARG)			(((ARG) & 0x07) << 5)

/* Field masks for Register RCR2 */
#define RPHASE				OMAP2_MCBSP_BIT(15)
#define RFRLEN2(ARG)			(((ARG) & 0x7F) << 8)
#define RWDLEN2(ARG)			(((ARG) & 0x07) << 5)
#define RREVERSE(ARG)			(((ARG) & 0x03) << 3)
#define RFIG                            OMAP2_MCBSP_BIT(2)
#define RDATDLY(ARG)			((ARG) & 0x03)

/* Field masks for Register XCR1 */
#define XFRLEN1(ARG)			(((ARG) & 0x7F) << 8)
#define XWDLEN1(ARG)			(((ARG) & 0x07) << 5)

/* Field masks for Register XCR2 */
#define XPHASE				OMAP2_MCBSP_BIT(15)
#define XFRLEN2(ARG)			(((ARG) & 0x7F) << 8)
#define XWDLEN2(ARG)			(((ARG) & 0x07) << 5)
#define XREVERSE(ARG)			(((ARG) & 0x03) << 3)
#define XFIG                            OMAP2_MCBSP_BIT(2)
#define XDATDLY(ARG)			((ARG) & 0x03)

/* Field masks for Register SRGR1 */
#define FWID(ARG)			(((ARG) & 0xFF) << 8)
#define CLKGDV(ARG)			((ARG) & 0xFF)

/* Field masks for Register SRGR2 */
#define GSYNC				OMAP2_MCBSP_BIT(15)
#define CLKSP				OMAP2_MCBSP_BIT(14)
#define CLKSM				OMAP2_MCBSP_BIT(13)
#define FSGM				OMAP2_MCBSP_BIT(12)
#define FPER(ARG)			((ARG) & 0xFFF)

/* Field masks for Register MCR1 */
#define RMCME				OMAP2_MCBSP_BIT(9)
#define RPBBLK(ARG)			(((ARG) & 0x03) << 7)
#define RPABLK(ARG)			(((ARG) & 0x03) << 5)
#define RMCM				(0x01)

/* Field masks for Register MCR2 */
#define XMCME				OMAP2_MCBSP_BIT(9)
#define XPBBLK(ARG)			(((ARG) & 0x03) << 7)
#define XPABLK(ARG)			(((ARG) & 0x03) << 5)
#define XMCM(ARG)			((ARG) & 0x03)

/* Field masks for Register PCR */
#define IDLEEN				OMAP2_MCBSP_BIT(14)
#define XIOEN				OMAP2_MCBSP_BIT(13)
#define RIOEN				OMAP2_MCBSP_BIT(12)
#define FSXM				OMAP2_MCBSP_BIT(11)
#define FSRM				OMAP2_MCBSP_BIT(10)
#define CLKXM				OMAP2_MCBSP_BIT(9)
#define CLKRM				OMAP2_MCBSP_BIT(8)
#define SCLKME				OMAP2_MCBSP_BIT(7)
#define CLKSSTAT			OMAP2_MCBSP_BIT(6)
#define DXSTAT				OMAP2_MCBSP_BIT(5)
#define DRSTAT				OMAP2_MCBSP_BIT(4)
#define FSXP				OMAP2_MCBSP_BIT(3)
#define FSRP				OMAP2_MCBSP_BIT(2)
#define CLKXP				OMAP2_MCBSP_BIT(1)
#define CLKRP				(0x01)

/* Field Masks for Register SYSCONFIG */
#define FORCE_IDLE			0x0
#define NO_IDLE				0x1
#define SMART_IDLE			0x2
#define MCBSP_SYSC_IOFF_FOFF		0x0
#define MCBSP_SYSC_IOFF_FON		0x2 /* Err in TRM ES2.0 ?? */
#define CLOCKACTIVITY(ARG)		(((ARG) & 0x03) << 8)
#define SIDLEMODE(ARG)			(((ARG) & 0x03) << 3)
#define ENAWAKEUP			OMAP2_MCBSP_BIT(2)
#define SOFTRST    			OMAP2_MCBSP_BIT(1)

/* Field Masks for Register THRSH2 */
#define XTHRESHOLD(ARG)			(((ARG) & 0xFFF) << 0)

/* Field Masks for Register THRSH1 */
#define RTHRESHOLD(ARG)			(((ARG) & 0xFFF) << 0)

/* Field Masks for Register XCCR */
#define PPCONNECT			OMAP2_MCBSP_BIT(14)
#define DXENDLY(ARG)			(((ARG) & 0x03) << 12)
#define DLB				OMAP2_MCBSP_BIT(5)
#define XDMAEN				OMAP2_MCBSP_BIT(3)
#define XDISABLE			OMAP2_MCBSP_BIT(0)

/* Field Masks for Register RCCR */
#define RDMAEN				OMAP2_MCBSP_BIT(3)
#define RDISABLE			OMAP2_MCBSP_BIT(0)

/* McBSP read and write */

#if defined CONFIG_ARCH_OMAP2430 || defined(CONFIG_ARCH_OMAP3430)
#define OMAP2_MCBSP_READ(base, reg)              (__raw_readl((base) + OMAP2_MCBSP_REG_##reg))
#define OMAP2_MCBSP_WRITE(base, reg, val)        (__raw_writel(((u32)(val)), ((base) + OMAP2_MCBSP_REG_##reg)))
#else
#define OMAP2_MCBSP_READ(base, reg)              (__raw_readw((base) + OMAP2_MCBSP_REG_##reg))
#define OMAP2_MCBSP_WRITE(base, reg, val)        (__raw_writew((val), ((base) + OMAP2_MCBSP_REG_##reg)))
#endif

/* Name for each interface */
#define OMAP2_MCBSP_STR			"OMAP2_MCBSP"

/* Interface specific global structure */
struct omap2_mcbsp{
	u32 virt_base;          /* virtual base */
	u32 phy_base;           /* physical base */
	u32 mode;		/* Master or Slave */
	u8 free;                /* free or not */
	u8 auto_reset;          /* Auto Reset */
	u8 txskip_alt;          /* Tx skip flags */
	u8 rxskip_alt;          /* Rx skip flags */
	struct clk *fck;        /* func clock */
	struct clk *ick;        /* if clock */
	u32	fclksrc;	/* fclk source */
	spinlock_t lock;	/* spin lock */
	char name[sizeof(OMAP2_MCBSP_STR)+1];	/* Name for each interface*/
	u8 suspended;     			/* for power management state */ 
	u8 master_mode;
	
	/* IRQ related fields */
	omap2_mcbsp_isr_t isr;			/* User provided ISR */
	void *irqargs;				/* User data (IRQ args) */
	u32 irqmask;				/* IRQ mask */
	
	/* Contains pointers to user data (will be used with rx/tx complete callback)*/
	int *rxcb_userdata;	
	int *txcb_userdata;
	
	omap2_mcbsp_dma_cb rx_dma_cb;
	omap2_mcbsp_dma_cb tx_dma_cb;
	
	int dma_chain_id_rx;
	int dma_chain_id_tx;	
	
	int rx_data_type;
	int tx_data_type;	

	int rxdmachain_state;
	int txdmachain_state;

	int interface_mode;
	struct semaphore semlock;
	struct semaphore tx_semlock;
	struct omap_dma_channel_params rx_param;
	/* for receive only */
	int rx_config_done;
};


/* Device name needed for resource tracking*/
#ifdef CONFIG_TRACK_RESOURCES
static struct device_driver mcbsp_driver = {
	.name = "mcbsp",
};

static struct device mcbsp_device = {
	.driver = &mcbsp_driver,
};
#endif

#ifdef CONFIG_ARCH_OMAP3430
/* IRQ numbers for McBSP interfacei for 34XX */
u32 omap2_mcbsp_irq [OMAP2_MAX_MCBSP_COUNT] =	
{	INT_34XX_MCBSP1_IRQ,
	INT_34XX_MCBSP2_IRQ,
	INT_34XX_MCBSP3_IRQ,
	INT_34XX_MCBSP4_IRQ,
	INT_34XX_MCBSP5_IRQ,
};
#endif

#ifdef CONFIG_ARCH_OMAP2430
/* IRQ numbers for McBSP interface for 243X */
u32 omap2_mcbsp_irq [OMAP2_MAX_MCBSP_COUNT] =
{       INT_243X_MCBSP1_IRQ,
        INT_243X_MCBSP2_IRQ,
        INT_243X_MCBSP3_IRQ,
        INT_243X_MCBSP4_IRQ,
        INT_243X_MCBSP5_IRQ,
};
#endif


#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
/* McBSP DMA requests*/
u32 omap2_mcbsp_dma_tx[OMAP2_MAX_MCBSP_COUNT]=
{	OMAP24XX_DMA_MCBSP1_TX,
	OMAP24XX_DMA_MCBSP2_TX,
	OMAP24XX_DMA_MCBSP3_TX,
	OMAP24XX_DMA_MCBSP4_TX,
	OMAP24XX_DMA_MCBSP5_TX,
};
	
u32 omap2_mcbsp_dma_rx[OMAP2_MAX_MCBSP_COUNT] =	
{	OMAP24XX_DMA_MCBSP1_RX,
	OMAP24XX_DMA_MCBSP2_RX,
	OMAP24XX_DMA_MCBSP3_RX,
	OMAP24XX_DMA_MCBSP4_RX,
	OMAP24XX_DMA_MCBSP5_RX,
};

static int omap2_mcbsp_drr[OMAP2_MAX_MCBSP_COUNT]= 
{
	OMAP2_MCBSP1_BASE_PHY + OMAP2_MCBSP_REG_DRR,
	OMAP2_MCBSP2_BASE_PHY + OMAP2_MCBSP_REG_DRR,
	OMAP2_MCBSP3_BASE_PHY + OMAP2_MCBSP_REG_DRR,
	OMAP2_MCBSP4_BASE_PHY + OMAP2_MCBSP_REG_DRR,
	OMAP2_MCBSP5_BASE_PHY + OMAP2_MCBSP_REG_DRR,
};

static int omap2_mcbsp_dxr[OMAP2_MAX_MCBSP_COUNT]= 
{
	OMAP2_MCBSP1_BASE_PHY + OMAP2_MCBSP_REG_DXR,
	OMAP2_MCBSP2_BASE_PHY + OMAP2_MCBSP_REG_DXR,
	OMAP2_MCBSP3_BASE_PHY + OMAP2_MCBSP_REG_DXR,
	OMAP2_MCBSP4_BASE_PHY + OMAP2_MCBSP_REG_DXR,
	OMAP2_MCBSP5_BASE_PHY + OMAP2_MCBSP_REG_DXR,
};

static struct omap2_mcbsp omap2_mcbsp_info[OMAP2_MAX_MCBSP_COUNT] = {
        [0] = {.virt_base 	= OMAP2_MCBSP1_BASE,
               .phy_base  	= OMAP2_MCBSP1_BASE_PHY,
               .name		= OMAP2_MCBSP_STR "1",
			   },
        [1] = {.virt_base 	= OMAP2_MCBSP2_BASE,
               .phy_base  	= OMAP2_MCBSP2_BASE_PHY,
               .name		= OMAP2_MCBSP_STR "2",
               },
        [2] = {.virt_base 	= OMAP2_MCBSP3_BASE,
               .phy_base  	= OMAP2_MCBSP3_BASE_PHY,
               .name		= OMAP2_MCBSP_STR "3",
               },
        [3] = {.virt_base 	= OMAP2_MCBSP4_BASE,
               .phy_base  	= OMAP2_MCBSP4_BASE_PHY,
               .name		= OMAP2_MCBSP_STR "4",
               },
        [4] = {.virt_base 	= OMAP2_MCBSP5_BASE,
               .phy_base  	= OMAP2_MCBSP5_BASE_PHY,
               .name		= OMAP2_MCBSP_STR "5",
               },
};
static char omap2_mcbsp_fck[OMAP2_MAX_MCBSP_COUNT][15]={	"mcbsp1_fck\0",
								"mcbsp2_fck\0",
								"mcbsp3_fck\0",
								"mcbsp4_fck\0",
								"mcbsp5_fck\0"
							};
										
static char omap2_mcbsp_ick[OMAP2_MAX_MCBSP_COUNT][15]={	"mcbsp1_ick\0",
								"mcbsp2_ick\0",
								"mcbsp3_ick\0",
								"mcbsp4_ick\0",
								"mcbsp5_ick\0"
							};
					
static int omap2_mcbsp_max_dmachs_rx[OMAP2_MAX_MCBSP_COUNT]={2,2,2,2,2};
static int omap2_mcbsp_max_dmachs_tx[OMAP2_MAX_MCBSP_COUNT]={2,2,2,2,2};

#else /* OMAP242x McBSP definitions */

/* Enable the following for pcrm and muxing logic to be enabled for mcbsp2 24xx */
/* in 2420, we do a bit of pin muxing, in 243x.. uboot does it for us */
#define MCBSP_PIN_MUXING

#ifdef MCBSP_PIN_MUXING

#define CONTROL_REG_BASE_ADDR		OMAP24XX_VA_SYSTEM_CONTROL_BASE

#define CONTROL_PADCONF_eac_ac_sclk 	0x0124
#define CONTROL_PADCONF_eac_ac_fs   	0x0125
#define CONTROL_PADCONF_eac_ac_din  	0x0126
#define CONTROL_PADCONF_eac_ac_dout 	0x0127
#define CONTROL_PADCONF_eac_ac_mclk 	0x0128
#define CONTROL_PADCONF_sys_clkout  	0x0137

/* Mode register values */
#define config_eac_ac_sclk 		0x0009
#define config_eac_ac_fs   		0x0009
#define config_eac_ac_din  		0x0009
#define config_eac_ac_dout 		0x0009
#define config_eac_ac_mclk 		0x000B

static __inline__ u8 omap_mcbsp_pin_out(u32 offset, u8 val)
{
	return writeb(val, CONTROL_REG_BASE_ADDR + offset);
}

#endif  /* End of pin mux reg read/write */

/* IRQ numbers for McBSP interface */
u32 omap2_mcbsp_irq [OMAP2_MAX_MCBSP_COUNT] =	
{
	INT_MCBSP1_IRQ_OV,
	INT_MCBSP2_IRQ_OV,
};

/* McBSP DMA requests*/
u32 omap2_mcbsp_dma_tx[OMAP2_MAX_MCBSP_COUNT]=
{	
	OMAP_DMA_MCBSP1_TX,
	OMAP_DMA_MCBSP2_TX,
};
	
u32 omap2_mcbsp_dma_rx[OMAP2_MAX_MCBSP_COUNT] =	
{
	OMAP_DMA_MCBSP1_RX,
	OMAP_DMA_MCBSP2_RX,
};

static int omap2_mcbsp_drr[OMAP2_MAX_MCBSP_COUNT]= 
{
	OMAP2_MCBSP1_BASE_PHY + OMAP2_MCBSP_REG_DRR1,
	OMAP2_MCBSP2_BASE_PHY + OMAP2_MCBSP_REG_DRR1,
};

static int omap2_mcbsp_dxr[OMAP2_MAX_MCBSP_COUNT]= 
{
	OMAP2_MCBSP1_BASE_PHY + OMAP2_MCBSP_REG_DXR1,
	OMAP2_MCBSP2_BASE_PHY + OMAP2_MCBSP_REG_DXR1,
};

static struct omap2_mcbsp omap2_mcbsp_info[OMAP2_MAX_MCBSP_COUNT] = {
        [0] = {.virt_base 	= OMAP2_MCBSP1_BASE,
               .phy_base  	= OMAP2_MCBSP1_BASE_PHY,
               .name		= OMAP2_MCBSP_STR "1",
			   },
        [1] = {.virt_base 	= OMAP2_MCBSP2_BASE,
               .phy_base  	= OMAP2_MCBSP2_BASE_PHY,
               .name		= OMAP2_MCBSP_STR "2",
               },
};

static char omap2_mcbsp_fck[OMAP2_MAX_MCBSP_COUNT][15]={
	"mcbsp1_fck\0",
	"mcbsp2_fck\0",
};
										
static char omap2_mcbsp_ick[OMAP2_MAX_MCBSP_COUNT][15]={
	"mcbsp1_ick\0",
	"mcbsp2_ick\0",
};
					
static int omap2_mcbsp_max_dmachs_rx[OMAP2_MAX_MCBSP_COUNT]={2,2};
static int omap2_mcbsp_max_dmachs_tx[OMAP2_MAX_MCBSP_COUNT]={2,2};

#endif

#define	OMAP2_MCBSP_MAX_MULTI_CHS	127

u32 omap_get_mcbspid[OMAP24XX_LOGICAL_DMA_CH_COUNT] = {0};

/* Check if the McBPP Id passed to the interface is proper */
/* Keep this Macro here as the array is initialized above this */
#define OMAP2_McBSP_CHECKID(mcbsp_id) \
        (mcbsp_id > (OMAP2_MAX_MCBSP_COUNT-1)) || (mcbsp_id < OMAP2_MCBSP_INTERFACE1) || \
        (omap2_mcbsp_info[mcbsp_id].free == 1)

/* Check if the device is already suspended */
#define	OMAP2_McBSP_SUSPENDED(mcbsp_id)	(omap2_mcbsp_info[mcbsp_id].suspended == 1)
#endif /* __ASM_ARCH_OMAP2_McBSPDEF_H */
