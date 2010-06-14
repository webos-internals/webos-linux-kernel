/*
 * drivers/mmc/omap_hsmmc_pe.c
 *
 * SD/MMC/SDIO driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2007-2008 Palm, Inc
 *
 * Based on omap_hsmmc.c by Texas Instruments, Inc
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#undef CONFIG_HRES_COUNTER      // MAR: Disable it for now.    
#include <linux/hres_counter.h> // MAR: remove me

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/hardware.h>

#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <asm/arch/control.h>

/* SDIO */
#define SD_IO_WRITE			(1<<31)
#define SD_IO_BLKMODE			(1<<27)
#define SD_IO_RW_DIRECT_MASK		0xF000FF00
#define SD_IO_RW_DIRECT_IO_ABORT	0x80000C00
#define SD_IO_RW_DIRECT_FUNC_SEL	0x80001A00
#define SD_IO_RW_DIRECT_BUS_SUSPEND	0x80001800

/* Some declaration that probably should be moved  */
#define MMC1_ACTIVE_OVERWRITE (1<<31)
#define OMAP_MMC1_DEVID 1
#define OMAP_MMC2_DEVID 2

/* HSMMC Host Controller Registers */
#define HSMMC_SYSCONFIG		0x0010
#define HSMMC_SYSSTATUS		0x0014
#define HSMMC_CSRE		0x0024
#define HSMMC_SYSTEST		0x0028
#define HSMMC_CON		0x002C
#define HSMMC_BLK		0x0104
#define HSMMC_ARG		0x0108
#define HSMMC_CMD		0x010C
#define HSMMC_RSP10		0x0110
#define HSMMC_RSP32		0x0114
#define HSMMC_RSP54		0x0118
#define HSMMC_RSP76		0x011C
#define HSMMC_DATA		0x0120
#define HSMMC_PSTATE		0x0124
#define HSMMC_HCTL		0x0128
#define HSMMC_SYSCTL		0x012C
#define HSMMC_STAT		0x0130
#define HSMMC_IE		0x0134
#define HSMMC_ISE		0x0138
#define HSMMC_AC12		0x013C
#define HSMMC_CAPA		0x0140
#define HSMMC_CUR_CAPA		0x0148
#define HSMMC_REV		0x01FC

/* HSMMC controller bit definitions */
#define HSMMC_CAPA_VS18		(1<<26)
#define HSMMC_CAPA_VS30		(1<<25)
#define HSMMC_CAPA_VS33		(1<<24)

#define HSMMC_HCTL_SDVS18 	(0x5<<9)
#define HSMMC_HCTL_SDVS30 	(0x6<<9)
#define HSMMC_HCTL_SDVSCLR	0xFFFFF1FF
#define HSMMC_HCTL_SDVSDET	0x00000400
#define HSMMC_HCTL_SDBP   	(1<<8)

#define HSMMC_SYSCTL_DTO_VAL	(0xE)
#define HSMMC_SYSCTL_DTO_BIT	(16)

#define HSMMC_SYSCTL_CLKD_MASK	0x0000FFC0
#define HSMMC_SYSCTL_CLKD_BIT	(6)

#define HSMMC_SYSCTL_ICE	(1 << 0)
#define HSMMC_SYSCTL_ICS	(1 << 1)
#define HSMMC_SYSCTL_CEN	(1 << 2)

#define HSMMC_SYSCTL_SRDATA     (1 <<26)  /* soft reset for dat lines */
#define HSMMC_SYSCTL_SRCMD      (1 <<25)  /* soft reset for cmd lines */
#define HSMMC_SYSCTL_SRALL      (1 <<24)  /* soft reset for cmd lines */

#define HSMMC_CON_INIT		(1<<1)
#define HSMMC_CON_OD		(1<<0)     /* open drain mode */
#define HSMMC_CON_CLKEXTFREE	(1<<16)    /* free running card clock */
#define HSMMC_CON_EIGHT_BIT	(1<<5)     /* 8-bit mode */

 
#define HSMMC_STAT_CC		(1 << 0)   /* Command complete  */
#define HSMMC_STAT_TC		(1 << 1)   /* Trnasfer complete */
#define HSMMC_STAT_CARD_INT	(1 << 8)   /* SDIO Card INT */
#define HSMMC_STAT_ERR		(1 << 15)  /* Any error */
#define HSMMC_STAT_CMD_TIMEOUT	(1 << 16)  /* Command response time-out */
#define HSMMC_STAT_CMD_CRC	(1 << 17)  /* Command CRC error */
#define HSMMC_STAT_CMD_CEB	(1 << 18)  /* Command end bit error */
#define HSMMC_STAT_CMD_CIE	(1 << 19)  /* Command index error */
#define HSMMC_STAT_DATA_TIMEOUT	(1 << 20)  /* Data response time-out */
#define HSMMC_STAT_DATA_CRC	(1 << 21)  /* Data CRC error */
#define HSMMC_STAT_DATA_DEB     (1 << 22)  /* Data end bit error */
#define HSMMC_STAT_CARD_ERR	(1 << 28)  /* Card ERR */


#define HSMMC_PSTATE_BWE	(1<<10)
#define HSMMC_PSTATE_BRE	(1<<11)
#define HSMMC_PSTATE_BRW	(1<<10)
#define HSMMC_PSTATE_BRR	(1<<11)

#define HSMMC_CMD_DMA_EN	(1<<0)     /* DMA enable  */
#define HSMMC_CMD_BCE		(1<<1)     /* Block count enable */
#define HSMMC_CMD_DDIR		(1<<4)     /* Data direction */
#define HSMMC_CMD_MSBS		(1<<5)     /* Multi-block select */
#define HSMMC_CMD_DP_SELECT	(1<<21)    /* Data present */


#define HSMMC_INT_EN_MASK	0x307F0033
#define HSMMC_SDIO_CARD_INT	(1<<8)

#define HSMMC_HCTL_FOUR_BIT	(1<<1)

/* commands */
#define HSMMC_CMD_INIT_CMD	0x00000000

 /* SYSCONFIG bit values */
#define HSMMC_SYSCONFIG_CLKACT_IOFF		(0x0 << 8)
#define HSMMC_SYSCONFIG_CLKACT_ION		(0x1 << 8)
#define HSMMC_SYSCONFIG_CLKACT_FOFF		(0x0 << 9)
#define HSMMC_SYSCONFIG_CLKACT_FON		(0x1 << 9)

#define HSMMC_SYSCONFIG_SIDLE_FORCEIDLE		(0x0 << 3)
#define HSMMC_SYSCONFIG_SIDLE_NOIDLE		(0x1 << 3)
#define HSMMC_SYSCONFIG_SIDLE_SMARTIDLE		(0x2 << 3)

#define HSMMC_SYSCONFIG_ENAWAKEUP		(0x1 << 2)
#define HSMMC_SYSCONFIG_SOFTRESET		(0x1 << 1)
#define HSMMC_SYSCONFIG_AUTOIDLE		(0x1)

/* selectors for sysconfig */
#define HSMMC_SYSCONFIG_MODE_SMART_IDLE		(0x1)
#define HSMMC_SYSCONFIG_MODE_FORCE_IDLE		(0x2)

/* SYSSTATUS */
#define HSMMC_SYSSTATUS_RESETDONE		(0x1)

#define HSMMC_MASTER_CLOCK	96000000
#define HSMMC_TIMEOUT_MS	20


#ifdef CONFIG_OMAP34XX_OFFMODE
extern int context_restore_required(struct clk *clk);
extern void modify_timeout_value(struct clk *clk, u32 value);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

#ifdef CONFIG_OMAP34XX_OFFMODE
struct omap_hsmmc_regs {
	u32 hctl;
	u32 capa;
	u32 sysconfig;
	u32 ise;
	u32 ie;
	u32 con;
	u32 sysctl;
};
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

struct mmc_omap_host {
	int             power_mode;
	int             initstream;        // init state  
	int             suspended;         // suspend state
	int             retry_cnt;         // for workaround for <ES3.0 bugs
	int             retry_max;         // max number of retries 
	int             cmd52_poll;        // do CMD52 in poll mode
	/**/
	struct          mmc_host    *mmc;  // mmc_host context
	struct          mmc_request *mrq;  // current request if any
	struct          mmc_command *cmd;  // current command if any
	struct          mmc_data    *data; // current data if any 

	/* Resources */
	struct          resource *mem_res;
	void            __iomem *base;
	void            *mapbase;
	unsigned int    id;
	int             irq;

	/* clocks */
	struct clk       *fclk; 
	struct clk       *iclk; 
	struct clk       *dbclk;
	struct clk       *gptfck; /* Required for a 3430 ES1.0 Sil errata fix */
	int               clk_on;      // clock status
	struct timer_list clk_timer;   // clock inactivity timer 
	typeof(jiffies)   clk_timeout; // clock inactivity timeout

	/*  */
	spinlock_t      splock;

	// DMA section
	int    use_dma;      // set if we should use dma
	int    dma_ch;       // allocated dma channel
	int    dma_ch_num;   // number on channels
	int    dma_dir;      // dma direction 
	int    dma_len;      // total number segments in request
	int    dma_sg_cnt;   // number of segments left
	int    dma_rx_trig;  // rx dma request (instance specific)
	int    dma_tx_trig;  // tx dma request (instance specific)

#ifdef CONFIG_OMAP34XX_OFFMODE
	struct omap_hsmmc_regs hsmmc_ctx;
#endif

	// Board specific callbacks
	int (*board_power_mode) (void *, int mode   );
	int (*board_probe)      (void * );
	int (*board_remove)     (void * );
	int (*board_shutdown)   (void * );
	int (*board_suspend)    (void * );
	int (*board_resume)     (void * );
};

static void omap_mmc_dma_cb(int lch, u16 ch_status, void *data);

static void omap_mmc_request      (struct mmc_host *mmc, 
                                   struct mmc_request *req);
static void omap_mmc_request_done ( struct mmc_omap_host *host, 
                                    struct mmc_request *mrq );
static void omap_mmc_cmd_done     ( struct mmc_omap_host *host, 
                                    struct mmc_command *cmd);

static spinlock_t mmc_gpt_lock;
static int     gptfclk_counter = 0;

static const char *fclk24xx [] = {   
	"mmchs1_fck",  
	"mmchs2_fck"    
};

static const char *iclk24xx [] = {   
	"mmchs1_ick",   
	"mmchs2_ick"   
};

static const char *dbclk24xx[] = {   
	"mmchsdb1_fck", 
	"mmchsdb2_fck" 
};

static int mmc_dma_rx_trig  [] = {   
	OMAP24XX_DMA_MMC1_RX, 
	OMAP24XX_DMA_MMC2_RX,
};

static int mmc_dma_tx_trig  [] = {   
	OMAP24XX_DMA_MMC1_TX, 
	OMAP24XX_DMA_MMC2_TX,
};


/* MMC Host controller registers read/write API's */
static inline u32
omap_hsmmc_read(struct mmc_omap_host *host, u32 reg)
{
	return __raw_readl(host->base + reg);
}

static inline void
omap_hsmmc_write(struct mmc_omap_host *host, u32 reg, u32 val)
{
	__raw_writel( val, host->base + reg);
}

static inline u32
omap_hsmmc_set_mask(struct mmc_omap_host *host, u32 reg, u32 mask )
{
	u32 val = __raw_readl(host->base + reg) | mask;
	__raw_writel( val, host->base + reg);
	return val;
}

static inline u32
omap_hsmmc_clr_mask(struct mmc_omap_host *host, u32 reg, u32 mask )
{
	u32 val = __raw_readl(host->base + reg) & ~mask;
	__raw_writel( val, host->base + reg);
	return val;
}

static int 
omap_mmc_sysconfig (struct mmc_omap_host *host, int level)
{
	u32 sysconfig_val;

	switch (level) {
	case HSMMC_SYSCONFIG_MODE_SMART_IDLE:
		/*
		 * All features of SYSCONFIG enabled
		 * Note: MMC has Wakeup capability enabled at reset.
		 * If this capability is required then care needs to be taken
		 * for other wakeup related register such as HCTL(IWE bit) and
		 * interrupts in IE and ISE registers
		 *
		 * Clock activity has only FCLK ON
		 *
		 * Enabling SmartIdle in ES1.0, pervents CORE from going to
		 * retention. Hence even Wakeup capability is disabled.
		 */
		sysconfig_val = 0 |
			        HSMMC_SYSCONFIG_CLKACT_IOFF     |
			        HSMMC_SYSCONFIG_CLKACT_FOFF     |
			        HSMMC_SYSCONFIG_SIDLE_SMARTIDLE |
			        HSMMC_SYSCONFIG_ENAWAKEUP       |
//			        HSMMC_SYSCONFIG_CLKACT_ION      | 
//			        HSMMC_SYSCONFIG_CLKACT_FON      | 
//			        HSMMC_SYSCONFIG_SIDLE_NOIDLE |
			        HSMMC_SYSCONFIG_AUTOIDLE;

		omap_hsmmc_write(host, HSMMC_SYSCONFIG, sysconfig_val);
		break;

	case HSMMC_SYSCONFIG_MODE_FORCE_IDLE:
		/*
		 * Clock activity has ICLK and FCLK OFF
		 */
		sysconfig_val = HSMMC_SYSCONFIG_CLKACT_IOFF     |
			        HSMMC_SYSCONFIG_CLKACT_FOFF     |
			        HSMMC_SYSCONFIG_SIDLE_FORCEIDLE |
			        HSMMC_SYSCONFIG_AUTOIDLE;

		omap_hsmmc_write(host, HSMMC_SYSCONFIG, sysconfig_val);
		break;
	}
	return 0;
}

#ifdef CONFIG_OMAP34XX_OFFMODE
static void 
omap2_hsmmc_save_ctx(struct mmc_omap_host *host)
{
	struct omap_hsmmc_regs *regs = &host->hsmmc_ctx;

	/* MMC : context save */
	regs->hctl = omap_hsmmc_read(host, HSMMC_HCTL);
	regs->capa = omap_hsmmc_read(host, HSMMC_CAPA);
	regs->sysconfig = omap_hsmmc_read(host, HSMMC_SYSCONFIG);
	regs->ise  = omap_hsmmc_read(host, HSMMC_ISE);
	regs->ie   = omap_hsmmc_read(host, HSMMC_IE);
	regs->con  = omap_hsmmc_read(host, HSMMC_CON);
	regs->sysctl = omap_hsmmc_read(host, HSMMC_SYSCTL);
}

static void 
omap2_hsmmc_restore_ctx(struct mmc_omap_host *host)
{
	struct omap_hsmmc_regs *regs = &host->hsmmc_ctx;

	/* MMC : context restore */
	omap_hsmmc_write(host, HSMMC_HCTL, regs->hctl);
	omap_hsmmc_write(host, HSMMC_CAPA, regs->capa);
	omap_hsmmc_write(host, HSMMC_CON,  regs->con);
	omap_hsmmc_write(host, HSMMC_SYSCONFIG, regs->sysconfig);
	omap_hsmmc_write(host, HSMMC_ISE,  regs->ise);
	omap_hsmmc_write(host, HSMMC_IE,   regs->ie);
	omap_hsmmc_write(host, HSMMC_SYSCTL, regs->sysctl);
}
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

static void
hres_dump_regs ( struct mmc_omap_host *host )
{
	hres_event ( "HTLC",      0, omap_hsmmc_read(host, HSMMC_HCTL));
	hres_event ( "CAPA",      0, omap_hsmmc_read(host, HSMMC_CAPA));
	hres_event ( "SYSCONFIG", 0, omap_hsmmc_read(host, HSMMC_SYSCONFIG));
	hres_event ( "SYSSTATUS", 0, omap_hsmmc_read(host, HSMMC_SYSSTATUS));
	hres_event ( "SYSCTL",    0, omap_hsmmc_read(host, HSMMC_SYSCTL ));
	hres_event ( "ISE",       0, omap_hsmmc_read(host, HSMMC_ISE ));
	hres_event ( "IE",        0, omap_hsmmc_read(host, HSMMC_IE  ));
	hres_event ( "CON",       0, omap_hsmmc_read(host, HSMMC_CON ));
	hres_event ( "STAT",      0, omap_hsmmc_read(host, HSMMC_STAT   ));
	hres_event ( "PSTATE",    0, omap_hsmmc_read(host, HSMMC_PSTATE ));
	hres_event ( "CMD",       0, omap_hsmmc_read(host, HSMMC_CMD ));
	hres_event ( "ARG",       0, omap_hsmmc_read(host, HSMMC_ARG ));
	hres_event ( "BLK",       0, omap_hsmmc_read(host, HSMMC_BLK ));
	hres_event ( "CSRE",      0, omap_hsmmc_read(host, HSMMC_CSRE ));
#ifdef CONFIG_ARCH_OMAP34XX
	hres_event ( "PBIAS1",    0, CONTROL_PBIAS_1 );
	hres_event ( "DEVCONF1",  0, CONTROL_DEVCONF1);
#else	
	hres_event ( "PBIAS1",    0, omap_readl(OMAP2_CONTROL_PBIAS_1)  );
	hres_event ( "DEVCONF0",  0, omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0) );
	hres_event ( "DEVCONF1",  0, omap_ctrl_readl(OMAP2_CONTROL_DEVCONF1) );
#endif	
}

/*
 *  Get omap2 clocks
 */
static int 
omap2_get_mmc_clock(struct platform_device *pdev, 
                    struct mmc_omap_host   *host)
{
	int i = host->id - 1; // it is 1 based

	host->fclk = clk_get(&pdev->dev, fclk24xx[i] );
	if (IS_ERR(host->fclk)) {
		host->fclk = NULL;
		return PTR_ERR(host->fclk);
	}
	host->iclk = clk_get(&pdev->dev, iclk24xx[i] );
	if (IS_ERR(host->iclk)) {
		host->iclk = NULL;
		clk_put(host->fclk);
		return PTR_ERR(host->iclk);
	}
	host->dbclk = clk_get(&pdev->dev, dbclk24xx[i] );
	/*
	 * Only through a error message, MMC can still work
	 * without debounce clock.
	 */
	if (IS_ERR(host->dbclk)) {
		dev_err( mmc_dev(host->mmc), "failed to get debounce clock\n");
	}

	return  0;
}

/**
 *   Get omap3 clocks
 */
static int 
omap3_get_mmc_clock( struct platform_device *pdev, 
                     struct mmc_omap_host   *host)
{
	/* 3430-ES1.0  Sil errata fix */
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		host->gptfck = clk_get(&pdev->dev, "gpt10_fck");
		if (IS_ERR(host->gptfck)) {
			host->gptfck = NULL;
			return PTR_ERR(host->gptfck);
		}
	}
	host->fclk = clk_get(&pdev->dev, "mmchs_fck");
	if (IS_ERR(host->fclk)) {
		host->fclk = NULL;
		return PTR_ERR(host->fclk);
	}

	host->iclk = clk_get(&pdev->dev, "mmchs_ick");
	if (IS_ERR(host->iclk)) {
		clk_put(host->fclk);
		host->iclk = NULL;
		return PTR_ERR(host->iclk);
	}
	return 0;
}

/**
 *   Enable omap mmc clocks
 */
static int 
omap_mmc_clk_enable(struct mmc_omap_host *host, int set_timer)
{
	unsigned long flags;

	spin_lock_irqsave(&host->splock, flags);
	del_timer_sync(&host->clk_timer);
	if( likely(host->clk_on)) {
		spin_unlock_irqrestore(&host->splock, flags);
		return 0;
	}

	/* 3430-ES1.0  Sil errata fix */
	if( is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		if( gptfclk_counter == 0 ) {
			if( clk_enable(host->gptfck) != 0 ) {
				spin_unlock(&mmc_gpt_lock);
				dev_err( mmc_dev(host->mmc),
					"unable to enable gptfck clock\n");
				goto clk_en_err;
			}
		}
		gptfclk_counter ++;
		spin_unlock(&mmc_gpt_lock);
	}

	if (clk_enable(host->iclk) != 0)
		goto clk_en_err1;

	if (clk_enable(host->fclk) != 0)
		goto clk_en_err2;

	if (cpu_is_omap2430()) {
		if (clk_enable(host->dbclk) != 0) {
			dev_warn( mmc_dev(host->mmc),
				"Failed to enable debounce clock\n" );
		}
	}
	
	host->clk_on = 1;
	if( set_timer ) {
		mod_timer(&host->clk_timer, jiffies + host->clk_timeout);
	}
	spin_unlock_irqrestore(&host->splock, flags);
	return 1;

clk_en_err2:
	/* On fclk failure */
	clk_disable(host->iclk);

clk_en_err1:
	/* On iclk failure */
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		gptfclk_counter --;
		if( gptfclk_counter == 0)
			clk_disable(host->gptfck);
		spin_unlock(&mmc_gpt_lock);
	}
clk_en_err:
	dev_err(mmc_dev(host->mmc),
		"Unable to enable MMC clocks \n");

	spin_unlock_irqrestore(&host->splock, flags);
	return -1;
}

/**
 *   Disable omap mmc clocks
 */
static int 
omap_mmc_clk_disable (struct mmc_omap_host *host)
{
	u32 val, cnt;
	unsigned long flags;

	spin_lock_irqsave(&host->splock, flags);
	if(!host->clk_on) {
		spin_unlock_irqrestore(&host->splock, flags);
		return 0;
	}

	// reset state machines
	omap_hsmmc_set_mask ( host, HSMMC_SYSCTL, 
	                      HSMMC_SYSCTL_SRCMD | HSMMC_SYSCTL_SRDATA );
	cnt = 20;
	while ( cnt-- ) {
		val = omap_hsmmc_read(host, HSMMC_SYSCTL);
		if((val & (HSMMC_SYSCTL_SRCMD | HSMMC_SYSCTL_SRDATA)) == 0) {
			break;
		}
		udelay(1);
	}

//	hres_event_cnt(cnt);
	BUG_ON( cnt == 0 );

	clk_disable(host->fclk);
	clk_disable(host->iclk);

	/* 3430-ES1.0  Sil errata fix */
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		gptfclk_counter --;
		if (!gptfclk_counter)
			clk_disable(host->gptfck);
		spin_unlock(&mmc_gpt_lock);
	}

	if (cpu_is_omap2430()) {
		clk_disable(host->dbclk);
	}

	host->clk_on = 0;
	spin_unlock_irqrestore(&host->splock, flags);
	return 0;
}

/*
 * Stop clock to the card
 */
static void 
omap_mmc_stop_card_clock(struct mmc_omap_host *host)
{
	/* Stop clock to the card */
	omap_hsmmc_clr_mask(host, HSMMC_SYSCTL, HSMMC_SYSCTL_CEN );
	if ((omap_hsmmc_read(host, HSMMC_SYSCTL) & HSMMC_SYSCTL_CEN) != 0x0){
		dev_err( mmc_dev(host->mmc), 
		    "MMC clock not stoped, clock freq can not be altered\n");
	}
}

/*
 *
 */
static void
omap_mmc_set_high_speed_mode(struct mmc_omap_host *host, int high_speed )
{
#ifdef CONFIG_ARCH_OMAP34XX
	// set 34xx specific bits
	if (host->id == OMAP_MMC1_DEVID) {
		if( high_speed ) {
			//  PBIASSPEEDCTRL0: support 52MHz
			CONTROL_PBIAS_1  |= (1 <<  2); 

			//  PBIASSPEEDCTRL1: support 52MHz
			CONTROL_PBIAS_1  |= (1 << 10); 
		} else {
			//  PBIASSPEEDCTRL0: support 52MHz
			CONTROL_PBIAS_1  &= ~(1 <<  2); 

			//  PBIASSPEEDCTRL1: support 52MHz
			CONTROL_PBIAS_1  &= ~(1 << 10); 
		}
		//  MMCSDIO1ADPCLKISEL bit
		CONTROL_DEVCONF0 |=  (1 << 24);
		
	}
	if (host->id == OMAP_MMC2_DEVID) {
		// MMCSDIO2ADPCLKISEL bit
		CONTROL_DEVCONF1 |= (1 << 6);
	}
#else	
	if( host->id == OMAP_MMC1_DEVID) {
		u32 val;
		val  = omap_readl ( OMAP2_CONTROL_PBIAS_1 );
		if( high_speed )
			val |=  (1 << 2); 
		else
			val &= ~(1 << 2); 
		omap_writel( val, OMAP2_CONTROL_PBIAS_1 );
	}
#endif
}

/*
 *  Set and start clock to the card
 */
static void
omap_mmc_start_card_clock(struct mmc_omap_host *host, int clock )
{
	u32 val, dsor;
	typeof(jiffies) timeout;

	/* configure high speed mode */
	omap_mmc_set_high_speed_mode ( host, clock >= 26000000);

	/* Enable MMC_SD_CLK */
	// MAR: DOLATER make master clock dynamic
	dsor = HSMMC_MASTER_CLOCK / clock;
	if (dsor < 1)
		dsor = 1;

	if (HSMMC_MASTER_CLOCK / dsor > clock )
		dsor++;

	if (dsor > 250)
		dsor = 250;

	val = omap_hsmmc_read(host, HSMMC_SYSCTL);
	val = val & ~(HSMMC_SYSCTL_CLKD_MASK);
	val = val | (dsor << HSMMC_SYSCTL_CLKD_BIT);
	val = val | (HSMMC_SYSCTL_DTO_VAL << HSMMC_SYSCTL_DTO_BIT);
	omap_hsmmc_write(host, HSMMC_SYSCTL, val);

	omap_hsmmc_set_mask ( host, HSMMC_SYSCTL, HSMMC_SYSCTL_ICE);

	/* wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(HSMMC_TIMEOUT_MS);
	while ((omap_hsmmc_read(host, HSMMC_SYSCTL) & HSMMC_SYSCTL_ICS) != 0x2
		&& time_before(jiffies, timeout)) ;
		

	/* Enable clock to the card */
	omap_hsmmc_set_mask ( host, HSMMC_SYSCTL, HSMMC_SYSCTL_CEN);
}


/*
 *  reset cmd fsm and poll for complete
 */
static int
omap_hsmmc_reset_cmd_fsm (struct mmc_omap_host *host )
{
	u32 val;
	typeof(jiffies) timeout = jiffies + msecs_to_jiffies(HSMMC_TIMEOUT_MS);

	omap_hsmmc_set_mask ( host, HSMMC_SYSCTL, HSMMC_SYSCTL_SRCMD );
	do {
		val = omap_hsmmc_read(host, HSMMC_SYSCTL);
		if((val & HSMMC_SYSCTL_SRCMD) == 0) {
			return 0;
		}
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

/*
 *  reset data fsm and poll for complete
 */
static int
omap_hsmmc_reset_data_fsm (struct mmc_omap_host *host )
{
	u32 val;
	typeof(jiffies) timeout = jiffies + msecs_to_jiffies(HSMMC_TIMEOUT_MS);

	omap_hsmmc_set_mask ( host, HSMMC_SYSCTL, HSMMC_SYSCTL_SRDATA );
	do {
		val = omap_hsmmc_read(host, HSMMC_SYSCTL);
		if((val & HSMMC_SYSCTL_SRDATA) == 0) {
			return 0;
		}
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

/*
 *  handle command error
 */
static int 
omap_mmc_handle_cmd_error ( struct mmc_omap_host *host, u32 stat )
{
	int rc;

	if((stat & HSMMC_STAT_ERR)==0)
		return 0;

	if( stat & HSMMC_STAT_CMD_CRC ) {
		rc = -EILSEQ;
		dev_err(mmc_dev(host->mmc),
		        "CMD%d: command CRC error\n", host->mrq->cmd->opcode );
		goto done;
	}

	if( stat & HSMMC_STAT_CMD_TIMEOUT ) {
		rc = -ETIMEDOUT;
		dev_dbg(mmc_dev(host->mmc),
		        "CMD%d: command timeout\n", host->mrq->cmd->opcode);
		goto done;
	}

	if((stat & HSMMC_STAT_CMD_CIE) || (stat & HSMMC_STAT_CMD_CEB)) {
		rc = -EINVAL;
		dev_err(mmc_dev(host->mmc),
		        "CMD%d: CIE/CEB error\n", host->mrq->cmd->opcode );
		goto done;
	}
	
	return 0;
done:
	omap_hsmmc_reset_cmd_fsm ( host );
	return rc;
}

/*
 *  handle xfer error
 */
static int 
omap_mmc_handle_xfer_error ( struct mmc_omap_host *host, u32 stat )
{
	int rc;

	if((stat & HSMMC_STAT_ERR)==0)
		return 0;

	if( stat & HSMMC_STAT_DATA_CRC ) {
		rc = -EILSEQ;
		dev_err(mmc_dev(host->mmc),
		        "CMD%d: data CRC error\n", host->mrq->cmd->opcode );
		goto done;
	}

	if( stat & HSMMC_STAT_DATA_TIMEOUT ) {
		rc = -ETIMEDOUT;
		dev_dbg(mmc_dev(host->mmc),
		        "CMD%d: data timeout\n", host->mrq->cmd->opcode);
		goto done;
	}

	if(stat & HSMMC_STAT_DATA_DEB) {
		rc = -EINVAL;
		dev_err(mmc_dev(host->mmc),
		        "CMD%d: DEB error\n", host->mrq->cmd->opcode );
		goto done;
	}
	return 0;
done:
	omap_hsmmc_reset_cmd_fsm  ( host );
	omap_hsmmc_reset_data_fsm ( host );
	return rc;
}


/*
 *  poll for command complete. Should be used only for non-data commands
 */
static int
omap_hsmmc_wait_for_cc (struct mmc_omap_host *host, int msec)
{
	u32 status;
	int count = msec * 1000;

	do {
		udelay(1);

		status = omap_hsmmc_read(host, HSMMC_STAT);

		if (status & HSMMC_STAT_ERR)
			return omap_mmc_handle_cmd_error(host, status);
		
		if (status & HSMMC_STAT_CC)
			return 0; // command complete

	} while (--count > 0);

	if( host->cmd &&  host->cmd->opcode == SD_IO_RW_DIRECT ) {
		dev_err(mmc_dev(host->mmc),"CMD%d: CC poll timeout: last try\n",
		        host->cmd->opcode );
	}

	// read again one more time after the while loop ends
	status = omap_hsmmc_read(host, HSMMC_STAT);
	if (status & HSMMC_STAT_ERR)
                return omap_mmc_handle_cmd_error(host, status);

	if (status & HSMMC_STAT_CC)
		return 0; // command complete

	if( host->cmd && host->cmd->opcode == SD_IO_RW_DIRECT ) {

		dev_err (mmc_dev(host->mmc),"CMD%d: CC poll timeout\n", 
		         host->cmd->opcode );

		dev_err (mmc_dev(host->mmc),"HSMMC_STAT=0x%x\n",
			 omap_hsmmc_read(host, HSMMC_STAT));

		dev_err (mmc_dev(host->mmc),"HSMMC_PSSTATE=0x%x\n",
			 omap_hsmmc_read(host, HSMMC_PSTATE));
			
		dev_err (mmc_dev(host->mmc),"HSMMC_ISE=0x%x\n",
			 omap_hsmmc_read(host, HSMMC_ISE));
			 
		dev_err (mmc_dev(host->mmc),"HSMMC_IE=0x%x\n",
			 omap_hsmmc_read(host, HSMMC_IE));
			
		dev_err (mmc_dev(host->mmc),"HSMMC_CON=0x%x\n",
			 omap_hsmmc_read(host, HSMMC_CON));

	}

	return -ETIMEDOUT;
}

/*
 * send init stream sequence to the card before sending IDLE command
 */
static int
omap_mmc_send_init_stream(struct mmc_omap_host *host)
{
	int rc;

	if (host->initstream)
		return 0; // already initialized

	disable_irq(host->irq);

	omap_hsmmc_write(host, HSMMC_ISE, 0);
	omap_hsmmc_write(host, HSMMC_IE,  0);
	omap_hsmmc_set_mask (host, HSMMC_CON, HSMMC_CON_INIT);

	omap_hsmmc_write(host, HSMMC_CMD, HSMMC_CMD_INIT_CMD);
	rc = omap_hsmmc_wait_for_cc(host, HSMMC_TIMEOUT_MS );
	omap_hsmmc_clr_mask  (host, HSMMC_STAT, 0 );

	omap_hsmmc_clr_mask  (host, HSMMC_CON,  HSMMC_CON_INIT);
	omap_hsmmc_clr_mask  (host, HSMMC_STAT, 0 );

	enable_irq(host->irq);

	if (rc == 0) 
		host->initstream = 1;
	return 0;
}


/*
 * Returns response type for given command
 * 
 * RSP_TYPE CICE CCCE
 *	00 	0 	0 No Response
 *	01 	0 	1 R2
 *	10 	0 	0 R3 (R4 for SD cards)
 *	10 	1 	1 R1, R6, R5, R7
 *	11 	1 	1 R1b, R5b
 */
static int
omap_mmc_get_resp_type (struct mmc_command *cmd)
{
	int resptype;
	
	switch ( mmc_resp_type(cmd)) {
	case MMC_RSP_R2:
		resptype = 1;
		break;
		
	case MMC_RSP_R1:
//	case MMC_RSP_R5:  the same value as MMC_RSP_R1 
//	case MMC_RSP_R6:  the same value as MMC_RSP_R1  
//	case MMC_RSP_R7:  the same value as MMC_RSP_R1 
	case MMC_RSP_R3:
//	case MMC_RSP_R4:  the same value as MMC_RSP_R3 
		resptype = 2;
		break;
		
	case MMC_RSP_R1B:
		resptype = 3;
		break;
		
	default:
		resptype = 0;
		break;
	}

	return resptype;
}


/*
 *     Return command type for given command
 *
 *     CMD_TYPE Command type.
 *         0x0: Others Commands
 *         0x1: Upon CMD52 "Bus Suspend" operation
 *         0x2: Upon CMD52 "Function Select" operation
 *         0x3: Upon CMD12 or CMD52 "I/O Abort" command 
 */
static int 
omap_mmc_get_cmd_type (struct mmc_command *cmd)
{
	if( cmd->opcode == MMC_STOP_TRANSMISSION )
		return 0x3;

	if( cmd->opcode == SD_IO_RW_DIRECT ) {
		int arg = cmd->arg & SD_IO_RW_DIRECT_MASK;

		if (arg == SD_IO_RW_DIRECT_FUNC_SEL)
			return 0x2; // function select

		if (arg == SD_IO_RW_DIRECT_IO_ABORT)
			return 0x3; // IO abort

		if (arg == SD_IO_RW_DIRECT_BUS_SUSPEND)
			return 0x1; // IO abort
	}

	return 0;
}

/*
 *  Issue command and poll for completion
 */
static void 
omap_mmc_start_cmd_poll( struct mmc_omap_host *host,
                         struct mmc_command   *cmd, u32 cmdreg )
{
	// DOLATER: add check for CMDI bit in PSTAT 

	omap_hsmmc_write(host, HSMMC_STAT, 0xFFFFFFFF);
	omap_hsmmc_write(host, HSMMC_ISE,  0);
	omap_hsmmc_write(host, HSMMC_IE,   HSMMC_INT_EN_MASK);

	omap_hsmmc_write(host, HSMMC_ARG, cmd->arg);
	omap_hsmmc_write(host, HSMMC_CMD, cmdreg);

	cmd->error = omap_hsmmc_wait_for_cc(host, HSMMC_TIMEOUT_MS);

	omap_hsmmc_write(host, HSMMC_STAT, 0xFFFFFFFF);
	(void)omap_hsmmc_read (host, HSMMC_STAT);

	omap_mmc_cmd_done(host, cmd);
}

/*
 *  Issue command in irq mode
 */
static void 
omap_mmc_start_cmd_irq (struct mmc_omap_host *host,
                        struct mmc_command   *cmd, u32 cmdreg )
{
	// DOLATER: add check for CMDI bit in PSTAT 

	/* Clear status bits and enable interrupts */
	omap_hsmmc_write(host, HSMMC_STAT, 0xFFFFFFFF);
	omap_hsmmc_write(host, HSMMC_ISE, HSMMC_INT_EN_MASK);
	omap_hsmmc_write(host, HSMMC_IE,  HSMMC_INT_EN_MASK);

	/* issue command */
	omap_hsmmc_write(host, HSMMC_ARG, cmd->arg);
	omap_hsmmc_write(host, HSMMC_CMD, cmdreg);
}

/*
 * Configure the resptype, cmdtype and send the given command to the card
 */
static void
omap_mmc_start_command(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	int cmdreg, poll = 0;

	dev_dbg( mmc_dev(host->mmc), 
	         "CMD%d, argument 0x%08x\n", cmd->opcode, cmd->arg );

	hres_event ( "cmd_start", cmd->opcode, cmd->arg );
 
	host->cmd = cmd;

	cmdreg = (cmd->opcode << 24) 
	       | (omap_mmc_get_resp_type(cmd) << 16) 
	       | (omap_mmc_get_cmd_type (cmd) << 22);

	if( cmd->data ) {
		cmdreg |= HSMMC_CMD_DP_SELECT;

		if( cmd->data->flags & MMC_DATA_READ )
			cmdreg |= HSMMC_CMD_DDIR;

		if( cmd->data->blocks > 1 ) 
			cmdreg |= (HSMMC_CMD_MSBS | HSMMC_CMD_BCE);
			
		if (host->use_dma)
			cmdreg |= HSMMC_CMD_DMA_EN;
	}

	if (cmd->opcode == SD_IO_RW_DIRECT) {
		if ((cmd->arg & SD_IO_WRITE) == 0)
			cmdreg |= HSMMC_CMD_DDIR;
		if (host->cmd52_poll)
			poll = 1;
	}

	if (poll)
		omap_mmc_start_cmd_poll ( host, cmd, cmdreg );
	else
		omap_mmc_start_cmd_irq  ( host, cmd, cmdreg );
}

/*
 *  Notify the xfer done on MMC/SD cards to the core
 */
static void
omap_mmc_xfer_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	unsigned long flags;

	if (!data)
		return;

	hres_event ( "xfer_done",  host->dma_sg_cnt, data->error );

	spin_lock_irqsave ( &host->splock, flags );
	// we "might" have a bit of race between dma and mmc xfer end interrupts.
	// lets make sure both are done before ending request
	if (data->error)
		host->dma_sg_cnt = 0;

	if (--host->dma_sg_cnt >= 0) { 
		// we still have some data or waiting for xfer interrupt.
		spin_unlock_irqrestore ( &host->splock, flags );
		return;
	}
	omap_stop_dma_chain_transfers ( host->dma_ch  ); // stop DMA
	spin_unlock_irqrestore ( &host->splock, flags );

	// at this point xfer either done or aborted
	host->data = NULL;
		
	if (!data->error) {
		data->bytes_xfered += data->blocks * (data->blksz);
	}
	// if there is a stop command, start it
	if (data->stop) {
		omap_mmc_start_command(host, data->stop);
		return;
	}
	// we are done
	omap_mmc_request_done(host, data->mrq);
}

/*
 * Notify the core about command completion
 */
static void
omap_mmc_cmd_done(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	host->cmd = NULL;

	hres_event ( "cmd_done", cmd->opcode, cmd->arg );

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = omap_hsmmc_read(host, HSMMC_RSP10);
			cmd->resp[2] = omap_hsmmc_read(host, HSMMC_RSP32);
			cmd->resp[1] = omap_hsmmc_read(host, HSMMC_RSP54);
			cmd->resp[0] = omap_hsmmc_read(host, HSMMC_RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = omap_hsmmc_read(host, HSMMC_RSP10);

			if( cmd->opcode == MMC_WRITE_BLOCK ||
				cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK ||
				cmd->opcode == MMC_READ_SINGLE_BLOCK ||
				cmd->opcode == MMC_READ_MULTIPLE_BLOCK ||
				cmd->opcode == MMC_STOP_TRANSMISSION ||
				cmd->opcode == MMC_SEND_STATUS ) {
				if( cmd->resp[0] & 0x80000000 ) {
					printk( "MMC_CMD%d failed 0x%08x\n",
					        cmd->opcode, cmd->resp[0] );
				}
				if( cmd->resp[0] & 0x7fff0000 ) {
					panic( "MMC_CMD%d failed 0x%08x\n",
					        cmd->opcode, cmd->resp[0] );
				}
			} else {
				if( cmd->opcode != MMC_SEND_OP_COND ) {
					if( cmd->resp[0] & 0xFFFF0000 ) {
						printk( "MMC_CMD%d RESP 0x%08x\n",
							cmd->opcode, cmd->resp[0] );
					}
				}
			}
			
		}
	}

	if (host->data == NULL || cmd->error ) {
		dev_dbg( mmc_dev(host->mmc), 
		         "end request, err %x\n", cmd->error );
		omap_mmc_request_done ( host, cmd->mrq );
	}
}


/*
 *  Setup common dma params.
 */
static void
omap_mmc_dma_setup_params ( struct mmc_omap_host *host,
                            struct omap_dma_channel_params *p)
{
	memset ( p, 0, sizeof(struct omap_dma_channel_params));

	p->data_type   = OMAP_DMA_DATA_TYPE_S32;
	p->elem_count  = 1;   
	p->frame_count = 1;   
	p->sync_mode   = OMAP_DMA_SYNC_FRAME;

	if( host->dma_dir == DMA_FROM_DEVICE ) {
		p->src_amode   = OMAP_DMA_AMODE_CONSTANT;
		p->src_start   = 0;
		p->dst_amode   = OMAP_DMA_AMODE_POST_INC;
		p->trigger     = host->dma_rx_trig;
		p->src_or_dst_synch = OMAP_DMA_SRC_SYNC;
	}
	else {
		p->dst_amode   = OMAP_DMA_AMODE_CONSTANT;
		p->dst_start   = 0;
		p->src_amode   = OMAP_DMA_AMODE_POST_INC;
		p->trigger     = host->dma_tx_trig;
		p->src_or_dst_synch = OMAP_DMA_DST_SYNC;
	}
}


/*
 * Routine to configure and start DMA for the MMC card
 */
static int
omap_mmc_dma_start_data_xfer(struct mmc_omap_host *host )
{
	int rc, i, nob;
	dma_addr_t   src, dest;
	struct scatterlist *sg;
	struct mmc_data *data = host->data;
	struct omap_dma_channel_params params;

	BUG_ON( host->dma_ch == -1);  // no DMA chain
	BUG_ON( data->sg_len ==  0);  // nothing to transfer
	BUG_ON( data->sg_len > host->dma_ch_num ); // no enough channels
	BUG_ON( data->blksz  & 0x3);  // invalid block size.

	hres_event("start_data_xfer", data->sg_len, 0);

	host->dma_len = dma_map_sg( mmc_dev(host->mmc), 
	                            data->sg, data->sg_len, host->dma_dir);

	omap_mmc_dma_setup_params   ( host, &params );
	omap_modify_dma_chain_params( host->dma_ch, params );

        sg  = host->data->sg;
	host->dma_sg_cnt = data->sg_len;
	for( i = 0; i < host->dma_sg_cnt; i++, sg++ ) {

		nob = sg_dma_len ( sg ) / data->blksz;

		hres_event ( "queue_xfer", i, nob );

		if( host->dma_dir == DMA_TO_DEVICE ) {
			dest = (dma_addr_t) (host->mapbase + HSMMC_DATA);
			src  = sg_dma_address ( sg );
		} else {
			src  = (dma_addr_t) (host->mapbase + HSMMC_DATA);
			dest = sg_dma_address ( sg );
		}

		rc = omap_dma_chain_a_transfer( host->dma_ch, src, dest, 
	                                        data->blksz / 4, nob, host );
		if( rc ) {
			dev_err(mmc_dev(host->mmc),
				"dma_queue_xfer() failed with %d\n", rc );
			return rc;
		}
	}

	omap_start_dma_chain_transfers(host->dma_ch);

	return 0;
}

/*
 *   
 */
static void 
omap_mmc_request_done ( struct mmc_omap_host *host, struct mmc_request *mrq )
{
	hres_event ( "req_done", 0, 0 );

	host->mrq  = NULL;
	host->data = NULL;
	host->cmd  = NULL;

	// unmap sg
	if (host->dma_len) {
		dma_unmap_sg( mmc_dev(host->mmc), 
		              mrq->data->sg, host->dma_len, host->dma_dir );
		host->dma_len = 0; 
	}

	if (mrq->data && mrq->data->error) {
		// DOLATER: remove me
		hres_dump_regs (host);
		hres_evlog_disable ();
//		hres_evlog_print ();

		host->retry_cnt++;
		if( host->retry_cnt < host->retry_max ) {
			dev_err( mmc_dev(host->mmc), 
			        "data err (%d), retry (%d)...\n", 
			            mrq->data->error, host->retry_cnt );
			mrq->cmd->error  = 0;
			mrq->data->error = 0;
			omap_mmc_request(host->mmc, mrq);
			return;
		}
	}
	hres_evlog_disable ();
	mod_timer(&host->clk_timer, jiffies + host->clk_timeout);

	host->retry_cnt = 0;
	mmc_request_done(host->mmc, mrq);
}

/*
 * The MMC controller IRQ handler
 */
static irqreturn_t 
omap_mmc_irq(int irq, void *dev_id)
{
	int clk;
	u32 status;
	struct mmc_omap_host *host = (struct mmc_omap_host *)dev_id;

	clk = omap_mmc_clk_enable (host, 1);

	status = omap_hsmmc_read(host, HSMMC_STAT);
	omap_hsmmc_write(host, HSMMC_STAT, status);
	(void)omap_hsmmc_read (host, HSMMC_STAT);

	if( clk > 0 ) {
		dev_info( mmc_dev(host->mmc), "status in IRQ %x\n", status );
	}

//	dev_info( mmc_dev(host->mmc), "status in IRQ %x\n", status );
	hres_event( "mmc:status", host->id, status );
	
	if (status & HSMMC_STAT_ERR)
		goto err;
		
	if (host->cmd &&  (status & HSMMC_STAT_CC))
		omap_mmc_cmd_done(host, host->cmd);

	if (host->data && (status & HSMMC_STAT_TC))
		omap_mmc_xfer_done(host, host->data);

	return IRQ_HANDLED;

err:
	if (host->cmd) {
		host->cmd->error  = omap_mmc_handle_cmd_error(host, status);
		omap_mmc_cmd_done(host, host->cmd);
	}
	if (host->data) {
		host->data->error = omap_mmc_handle_xfer_error(host, status);
		omap_mmc_xfer_done(host, host->data);
	}
	return IRQ_HANDLED;
}


/*
 * DMA call back function
 */
static void 
omap_mmc_dma_cb(int lch, u16 ch_status, void *data)
{
	struct mmc_omap_host *host = data;
	omap_mmc_xfer_done ( host, host->data );
}

/*
 *  Inactivity timer callback
 */
static void
omap_mmc_timer_cb(unsigned long data)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *) data;
	omap_mmc_clk_disable(host);
}

/*
 * Routine to configure block leangth for MMC/SD cards
 * and intiate the transfer.
 */
static void
omap_mmc_prepare_data(struct mmc_omap_host *host, struct mmc_request *req)
{
	host->data = req->data;

	if (req->data == NULL) {
		host->dma_dir = DMA_NONE;
		omap_hsmmc_write(host, HSMMC_BLK, 0 );
		return;
	}

	host->use_dma = 1;  // use dma

	if( req->data->flags & MMC_DATA_WRITE ) {
		host->dma_dir = DMA_TO_DEVICE;
		hres_event ( "write", req->data->blocks, req->data->blksz );
	} else {
		host->dma_dir = DMA_FROM_DEVICE;
		hres_event ( "read" , req->data->blocks, req->data->blksz );
	}

	BUG_ON(req->data->blksz  > 0x400);
	BUG_ON(req->data->blocks > 65535);
	omap_hsmmc_write(host, HSMMC_BLK,(req->data->blksz) | 
	                                 (req->data->blocks << 16));
	return;
}


/*
 *
 */
static int 
omap_hsmmc_handle_go_idle ( struct mmc_omap_host *host )
{
	int rc;

	/* send init stream if needed */
	rc = omap_mmc_send_init_stream (host);
	if (unlikely(rc))
		return rc;

	/* reset data fsm */
	rc = omap_hsmmc_reset_data_fsm (host);
	if (rc) {
		return rc;
	}

	/* reset command fsm */
	rc = omap_hsmmc_reset_cmd_fsm  (host);
	if (rc) {
		return rc;
	}

	return 0;
}


/*
 * Request function. Exposed API to core for read/write operation
 */
static void 
omap_mmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	int rc;
	struct mmc_omap_host *host = mmc_priv(mmc);

	WARN_ON(host->mrq != NULL);
	WARN_ON(req->cmd  == NULL);
	
	host->mrq = req;

	hres_evlog_reset  ();
//	hres_evlog_enable ();

	omap_mmc_clk_enable (host, 0);

	/* Reset MMC Controller's Data FSM */
	if (req->cmd->opcode == MMC_GO_IDLE_STATE) {
		rc = omap_hsmmc_handle_go_idle (host);
		if (rc) {
			req->cmd->error = rc;
			omap_mmc_request_done ( host, req );
			return;
		}
	}
	
	host->use_dma = 0; // start with non-dma mode
	
	if( req->data ) {
		omap_mmc_prepare_data  ( host, req );
	
		if( host->use_dma ) {
			rc = omap_mmc_dma_start_data_xfer ( host );
			if (rc) {
				dev_err(mmc_dev(host->mmc),
				"Failed to initiate dma data xfer\n" );
			}
		}
	}

	omap_mmc_start_command(host, req->cmd);
}

/*
 * Change the socket power: 
 * 	power_mode = 0 OFF
 * 	power_mode = 1 switches to 1.8V
 * 	power_mode = 2 switches to 3V
 * 	DOLATER: make a macro
 */
static void 
omap_mmc_set_power_mode ( struct mmc_omap_host *host, int mode )
{
	int rc;

	if( mode && host->power_mode == mode )
		return;

	rc = host->board_power_mode ( host->mmc, mode );
	if( rc ) {
		dev_err( mmc_dev(host->mmc), 
			"Failed (%d) to set power mode to %d\n", rc, mode );
	}
	host->power_mode = mode;
	return;
}

/*
 * Routine to configure clock values. Exposed API to core
 */
static void 
omap_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	int power_mode;
	u32 val;
	struct mmc_omap_host *host = mmc_priv(mmc);

	dev_dbg( mmc_dev(host->mmc), "set_ios: clock %dHz busmode %d "
		"powermode %d vdd %d bus width %d\n",
		 ios->clock, ios->bus_mode,
		 ios->power_mode, ios->vdd, ios->bus_width );

	omap_mmc_clk_enable (host, 0);

	// select power mode
	power_mode = 0;
	if ((1 << ios->vdd) & (MMC_VDD_165_195 | MMC_VDD_20_21)) { 
		// treat MMC_VDD_20_21 as a power mode 1 (1.8V)
		power_mode = 1;
	}
	if ((1 << ios->vdd) & (MMC_VDD_29_30 | MMC_VDD_30_31)) { // 3.0V
		power_mode = 2;
	}

	omap_mmc_stop_card_clock ( host );

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		power_mode = 0;
		host->initstream = 0;
		omap_mmc_set_power_mode ( host, power_mode );
		break;
	case MMC_POWER_UP:
		omap_mmc_set_power_mode ( host, power_mode );
		break;

	case MMC_POWER_ON:
		omap_mmc_set_power_mode ( host, power_mode );

		val = omap_hsmmc_read(host, HSMMC_HCTL);

		val &= ~HSMMC_HCTL_SDBP;
		val &=  HSMMC_HCTL_SDVSCLR;
		if (power_mode == 2 )
			val |= HSMMC_HCTL_SDVS30 | HSMMC_HCTL_SDBP;
		if (power_mode == 1 )
			val |= HSMMC_HCTL_SDVS18 | HSMMC_HCTL_SDBP;

		omap_hsmmc_write(host, HSMMC_HCTL, val);
		break;
	}

	switch (mmc->ios.bus_width) {
	case MMC_BUS_WIDTH_8:
		omap_hsmmc_clr_mask ( host, HSMMC_HCTL, HSMMC_HCTL_FOUR_BIT);
		omap_hsmmc_set_mask ( host, HSMMC_CON,  HSMMC_CON_EIGHT_BIT);
		break;
	case MMC_BUS_WIDTH_4:
		omap_hsmmc_clr_mask ( host, HSMMC_CON,  HSMMC_CON_EIGHT_BIT);
		omap_hsmmc_set_mask ( host, HSMMC_HCTL, HSMMC_HCTL_FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		omap_hsmmc_clr_mask ( host, HSMMC_CON,  HSMMC_CON_EIGHT_BIT);
		omap_hsmmc_clr_mask ( host, HSMMC_HCTL, HSMMC_HCTL_FOUR_BIT);
		break;
	}

	switch (ios->bus_mode) {
	case MMC_BUSMODE_OPENDRAIN:
		omap_hsmmc_set_mask ( host, HSMMC_CON,  HSMMC_CON_OD);
		break;
		
	case MMC_BUSMODE_PUSHPULL:
		omap_hsmmc_clr_mask ( host, HSMMC_CON,  HSMMC_CON_OD);
		break;
	}

	if (ios->clock) {
		omap_mmc_start_card_clock ( host, ios->clock );
	}

	// start inactivity timer
	mod_timer(&host->clk_timer, jiffies + host->clk_timeout);
}


/*
 *  Allocate dma chain
 */
static int 
omap_mmc_dma_alloc ( struct mmc_omap_host *host, int lch_num )
{
	int rc; 
	struct omap_dma_channel_params params;

	// params will be changed
	omap_mmc_dma_setup_params ( host, &params );

	rc = omap_request_dma_chain ( params.trigger, mmc_hostname(host->mmc),
		omap_mmc_dma_cb, &host->dma_ch, lch_num, 
		OMAP_DMA_STATIC_CHAIN, params );
	        
	return rc;
}

/*
 *  Free dma chain
 */
static void
omap_mmc_dma_free ( struct mmc_omap_host *host ) 
{
	if (host->dma_ch) {
		omap_free_dma_chain ( host->dma_ch );
		host->dma_ch = -1;
	}
}


/*
 *  cmd52 poll control
 */
static ssize_t 
cmd52_poll_attr_show( struct device *dev, 
                      struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct mmc_omap_host   *host = platform_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", host->cmd52_poll);
}

static ssize_t 
cmd52_poll_attr_store( struct device *dev, 
                       struct device_attribute *attr, 
                       const char *buf, size_t count)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct mmc_omap_host   *host = platform_get_drvdata(pdev);
	
	if(!count)
		return count;
		
	host->cmd52_poll = simple_strtol (buf, NULL, 0);
	
	return count;
}

DEVICE_ATTR(cmd52_poll, S_IRUGO | S_IWUSR, 
            cmd52_poll_attr_show, cmd52_poll_attr_store);


static struct mmc_host_ops omap_mmc_ops = {
	.request = omap_mmc_request,
	.set_ios = omap_mmc_set_ios,
};


/*
 * Routine implementing the driver probe method
 */
static int __init omap_mmc_probe(struct platform_device *pdev)
{
	int ret = 0, irq;
	u32 capa_val = 0;
	struct resource *res;
	struct mmc_host *mmc;
	struct omap_mmc_conf *minfo = pdev->dev.platform_data;
	struct mmc_omap_host *host  = NULL;

	if (minfo == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENODEV;

	res = request_mem_region( res->start, res->end - res->start + 1,
	                          pdev->name);
	if (res == NULL)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct mmc_omap_host), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;

	host->power_mode = 0;
	host->initstream = 0;
	host->suspended  = 0;
	host->retry_cnt  = 0;
	host->retry_max  = minfo->host_retry_max;
	
	host->mem_res    = res;
	host->irq        = irq;
	host->id         = pdev->id;
	host->mapbase    = (void *)host->mem_res->start;
	host->base       = (void __iomem *)IO_ADDRESS(host->mapbase);
	mmc->ops         = &omap_mmc_ops;

	/* DMA */
	host->use_dma    =  0;
	host->dma_ch     = -1;
        host->dma_rx_trig = mmc_dma_rx_trig [host->id-1];
	host->dma_tx_trig = mmc_dma_tx_trig [host->id-1];
	host->dma_ch_num  = minfo->host_dma_ch;
	if( host->dma_ch_num == 0)
	    host->dma_ch_num = 2;
	host->cmd52_poll = minfo->host_cmd52_poll;

	/* Block device parameters */
//	mmc->max_phys_segs = 32;  // there is no real limit
//	mmc->max_hw_segs   = 32;  // there is no real limit
	mmc->max_phys_segs = host->dma_ch_num;  
	mmc->max_hw_segs   = host->dma_ch_num;  

	mmc->max_blk_size  = 512;
	mmc->max_blk_count = 65535;
	mmc->max_req_size  = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_seg_size  = mmc->max_req_size;

	/* Host capabilities */
	mmc->f_min     = 400000;
	mmc->f_max     = minfo->host_fmax;
	mmc->caps      = minfo->host_caps; // start with board specific caps
	mmc->ocr_avail = minfo->host_ocr ; // and board specific ocr

	/* */

	/* Setup board specific host callbacks */
	host->board_power_mode = minfo->board_power_mode;
	host->board_probe      = minfo->board_probe;
	host->board_remove     = minfo->board_remove;
	host->board_shutdown   = minfo->board_shutdown;
	host->board_suspend    = minfo->board_suspend;
	host->board_resume     = minfo->board_resume;

	/* Init spin lock */
	spin_lock_init ( &host->splock );

	/* Init inactivity timer */
	host->clk_timeout = msecs_to_jiffies(100);
	setup_timer ( &host->clk_timer, omap_mmc_timer_cb,(unsigned long)host); 

	/* grab dma channels */
	ret = omap_mmc_dma_alloc ( host, host->dma_ch_num );
	if (ret) {
		dev_err(mmc_dev(host->mmc),
			"omap_mmc_dma_alloc() failed with %d\n", ret );
		goto dma_alloc_err;
	}

	/* Check host provided capabilities */
	
	if( is_sil_rev_less_than(OMAP3430_REV_ES3_0) && 
	    host->id == OMAP_MMC1_DEVID) {
		// limit max clock if running pre 3.0 silicons
		if( mmc->f_max > 16000000 ) {
			mmc->f_max = 16000000;
			dev_info(mmc_dev(host->mmc), 
			         "Limit bus clock to %d\n", mmc->f_max );
		}
	}

	/* Bus width */
	if (cpu_is_omap2430()) {
		/* 2430 does not support 8 bit */
		mmc->caps &= ~MMC_CAP_8_BIT_DATA;

		if (is_sil_rev_equal_to(OMAP2430_REV_ES1_0)) {
			/* OMAP2430 ES1.0 does not support 4-bit */
			mmc->caps &= ~MMC_CAP_4_BIT_DATA;
		}
	}
	if (cpu_is_omap3410()) {
		/* 3410 does not support 8-bit */
		mmc->caps &= ~MMC_CAP_8_BIT_DATA;
	}

	/* voltage 3.0V & 1.8V */
	mmc->ocr_avail &= MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_165_195;
	if (host->id == OMAP_MMC1_DEVID) {
		capa_val  = HSMMC_CAPA_VS30 | HSMMC_CAPA_VS18;
		if (is_sil_rev_equal_to(OMAP2430_REV_ES2_0)) {
			/* OMAP2430 MMC1 ES2.0 - 1.8V only */
			capa_val = HSMMC_CAPA_VS18;
			mmc->ocr_avail &= MMC_VDD_165_195;
		}
	} 
	if (host->id != OMAP_MMC1_DEVID) {
		// MMC2 only supports 1.8V
		capa_val = HSMMC_CAPA_VS18;
		mmc->ocr_avail &= MMC_VDD_165_195;
	}
	if( mmc->ocr_avail == 0 ) {
		dev_warn ( mmc_dev(host->mmc), 
		           "No usible ocr point is available\n");
	}
	/* Set non-board specific capabilities */
	mmc->caps |=   MMC_CAP_MULTIWRITE 
	             | MMC_CAP_BYTEBLOCK 
	             | MMC_CAP_MMC_HIGHSPEED
	             | MMC_CAP_SD_HIGHSPEED;

	/* Additional chip specific settings */
	if (is_sil_rev_greater_than(OMAP2430_REV_ES2_0)) {
		if (host->id == OMAP_MMC1_DEVID) {
			/* enable MMC1_ACTIVE_OVERWRITE bit */
#ifdef CONFIG_ARCH_OMAP34XX
			CONTROL_DEVCONF1 |= MMC1_ACTIVE_OVERWRITE;
#else
			omap_ctrl_writel( omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1) 
			      | MMC1_ACTIVE_OVERWRITE, OMAP243X_CONTROL_DEVCONF1 );
#endif
		}
	}

	if (is_sil_rev_less_than(OMAP3430_REV_ES3_0)) {
		mmc->caps |=   MMC_CAP_OPEN_ENDED_ONLY;
	}

	/* get clocks */
	if( cpu_is_omap2430()) {
		ret = omap2_get_mmc_clock( pdev, host );
		if( ret ) {
			dev_err(mmc_dev(host->mmc), "Error getting clock\n");
			goto clk_get_err;
		}
	}

	if( cpu_is_omap34xx()) {
		ret = omap3_get_mmc_clock( pdev, host );
		if( ret ) {
			dev_err(mmc_dev(host->mmc), "Error getting clock\n");
			goto clk_get_err;
		}
	}

#ifdef CONFIG_OMAP34XX_OFFMODE
	modify_timeout_value(host->fclk, 500);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

	/* enable clocks */
	omap_mmc_clk_enable(host, 0);

	/* DOLATER insert controller reset and init here */

	/* setup sysconfig */
	omap_mmc_sysconfig (host, HSMMC_SYSCONFIG_MODE_SMART_IDLE);

	/* set host capabilities */
	omap_hsmmc_write(host, HSMMC_CAPA, capa_val );

	/* DOLATER, add HCTL and CON initialization here */


	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, omap_mmc_irq, 0, pdev->name, host );
	if (ret) {
		dev_err ( mmc_dev(host->mmc), "unable to grab mmc irq\n" );
		goto mmc_irq_err;
	}

	if( host->board_probe ) {
		if( host->board_probe ( host->mmc )) {
			dev_err ( mmc_dev(host->mmc), "board probe failed\n" );
			goto board_probe_err;
		}
	}

	platform_set_drvdata(pdev, host);

	mmc_add_host(mmc);

	/* create device sysfs attributes */
	ret = device_create_file(&pdev->dev, &dev_attr_cmd52_poll);
	if( ret ) 
		goto board_probe_err;


	dev_info ( mmc_dev(host->mmc), "initialized\n" );

	mod_timer(&host->clk_timer, jiffies + host->clk_timeout);

	return 0;

board_probe_err:
	free_irq(host->irq, host);

mmc_irq_err:
	omap_mmc_clk_disable(host);

	clk_put(host->fclk);
	clk_put(host->iclk);

	if (cpu_is_omap2430())
		clk_put(host->dbclk);

clk_get_err:
	omap_mmc_dma_free (host);

dma_alloc_err:
	mmc_free_host(mmc);
	
	return ret;
}

/*
 * Routine implementing the driver shutdown method
 */
static void 
omap_mmc_shutdown(struct platform_device *pdev)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	if( host && host->board_shutdown ) {
		host->board_shutdown ( host->mmc );
	}
}

/*
 * Routine implementing the driver remove method
 */
static int
omap_mmc_remove(struct platform_device *pdev)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if( host == NULL ) 
		return 0;
	
	/* Notify the core to remove the host */
	mmc_remove_host ( host->mmc );

	device_remove_file(&pdev->dev, &dev_attr_cmd52_poll);

	omap_mmc_dma_free (host);

	if( host->board_remove ) {
		host->board_remove ( host->mmc );
	}

	/* switch for force idle mode */
	omap_mmc_sysconfig (host, HSMMC_SYSCONFIG_MODE_FORCE_IDLE );

	/* disable clocks */
	omap_mmc_clk_disable ( host );

	/* put clocks */
	clk_put(host->fclk);
	clk_put(host->iclk);

	if (cpu_is_omap2430()) {
		clk_put(host->dbclk);
	}

	free_irq ( host->irq, host );

	mmc_free_host ( host->mmc );

	return 0;
}

#ifdef CONFIG_PM
/*
 * Routine to suspend the MMC device
 */
static int 
omap_mmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	u32 status;
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	if(!host || host->suspended) 
		return 0;

	omap_mmc_clk_enable (host, 0);

	/* Disable card detect interrupt */
	if( host->board_suspend ) {
		host->board_suspend ( host );
	}

	/* Notify the core to suspend the host */
	ret = mmc_suspend_host ( host->mmc, state );
	if (ret) 
		return ret;

#ifdef CONFIG_OMAP34XX_OFFMODE
	omap2_hsmmc_save_ctx(host);
#endif

	/* Disable Interrupts */
	omap_hsmmc_write(host, HSMMC_ISE, 0 );
	omap_hsmmc_write(host, HSMMC_IE,  0 );

	/* Clearing the STAT register*/
	status = omap_hsmmc_read(host, HSMMC_STAT );
        omap_hsmmc_write(host, HSMMC_STAT, status );
	(void)omap_hsmmc_read (host, HSMMC_STAT);

	/* setup sysconfig */
	omap_mmc_sysconfig (host, HSMMC_SYSCONFIG_MODE_FORCE_IDLE);

	/* disable clks for MMC */
	omap_mmc_clk_disable ( host );

	if (is_sil_rev_greater_than(OMAP2430_REV_ES2_0)) {
		if (host->id == OMAP_MMC1_DEVID) {
#ifdef CONFIG_ARCH_OMAP34XX
			CONTROL_DEVCONF1 &= ~MMC1_ACTIVE_OVERWRITE;
#else
			omap_ctrl_writel(omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1)
				& ~MMC1_ACTIVE_OVERWRITE,
				OMAP243X_CONTROL_DEVCONF1);
#endif
		}
	}

	host->initstream = 0;
	host->suspended  = 1;

	return ret;
}

/*
 * Routine to resume the MMC device
 */
static int omap_mmc_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	if(!host || !host->suspended)
		return 0;

	if (is_sil_rev_greater_than(OMAP2430_REV_ES2_0)) {
		// This needs to be done before functional clock is enabled
		if (host->id == OMAP_MMC1_DEVID) {
#ifdef CONFIG_ARCH_OMAP34XX
			CONTROL_DEVCONF1 |= MMC1_ACTIVE_OVERWRITE;
#else
			omap_ctrl_writel(omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1)
				| MMC1_ACTIVE_OVERWRITE,
				OMAP243X_CONTROL_DEVCONF1);
#endif
		}
	}

	// enable omap clocks
	omap_mmc_clk_enable(host, 0);

	// reset module
	omap_hsmmc_set_mask ( host, HSMMC_SYSCONFIG, HSMMC_SYSCONFIG_SOFTRESET);
	{
		u32 val, cnt = 20;
		while ( cnt-- ) {
			val = omap_hsmmc_read(host, HSMMC_SYSSTATUS);
			if( val & HSMMC_SYSSTATUS_RESETDONE )
				break;
			udelay(1);
		}
		BUG_ON(cnt == 0);
	}

#ifdef CONFIG_OMAP34XX_OFFMODE
	omap2_hsmmc_restore_ctx(host);
#endif

	/* setup sysconfig */
	omap_mmc_sysconfig( host, HSMMC_SYSCONFIG_MODE_SMART_IDLE );

	/* Notify the core to resume the host */
	ret = mmc_resume_host(host->mmc);
	if (ret == 0) {
		host->suspended = 0;
	}

	/* Reenable card detect interrupt */
	if( host->board_resume ) {
		host->board_resume ( host );
	}

	return ret;
}

#else
#define omap_mmc_suspend	NULL
#define omap_mmc_resume 	NULL
#endif


static struct platform_driver omap_mmc_driver = {
	.probe   = omap_mmc_probe,
	.remove  = omap_mmc_remove,
	.shutdown = omap_mmc_shutdown,
	.suspend = omap_mmc_suspend,
	.resume  = omap_mmc_resume,
	.driver  = {
		   .name = "hsmmc-omap",
	},
};

/*
 * Driver init method
 */
static int __init omap_mmc_init(void)
{
	/* Register the MMC driver */
	if (platform_driver_register(&omap_mmc_driver)) {
		printk(KERN_ERR ":failed to register MMC driver\n");
		return -ENODEV;
	}
	return 0;
}

/*
 * Driver exit method
 */
static void __exit omap_mmc_cleanup(void)
{
	/* Unregister MMC driver */
	platform_driver_unregister(&omap_mmc_driver);
}

module_init(omap_mmc_init);
module_exit(omap_mmc_cleanup);

MODULE_DESCRIPTION("OMAP 2430/3430 Multimedia Card driver");
MODULE_LICENSE("GPL");

