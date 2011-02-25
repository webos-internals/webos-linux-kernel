/*
 * drivers/mmc/omap_hsmmc.c
 *
 * Driver for OMAP2430/3430 MMC controller.
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/i2c.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/host.h>
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/mach-types.h>
#include <asm/arch/twl4030.h>
#include <asm/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/cpu.h>
#include <asm/arch/clock.h>
#include <asm/semaphore.h>
#include "omap_hsmmc.h"

#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif

/* 
 * PALM 
 *
 * This "aggressive" power control for MMC adds nearly 1ms of overhead to every command!!!
 * Turn off this brain dead "feature".
 */
/* TAG for Aggressive Power changes in MMC */
#undef AGGR_PM_CAP 


#ifdef AGGR_PM_CAP
#define mmc_clk_enable_aggressive(host)		mmc_clk_enable(host)
#define mmc_clk_disable_aggressive(host)	mmc_clk_disable(host)
#else
#define mmc_clk_enable_aggressive(host)		/* NULL */
#define mmc_clk_disable_aggressive(host)	/* NULL */
#endif


#ifdef AGGR_PM_CAP
/* SYSCONFIG bit values */
#define OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FOFF	0x0
#define OMAP_MMC_SYSCONFIG_CLKACT_ION_FOFF	0x1
#define OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FON	0x2
#define OMAP_MMC_SYSCONFIG_CLKACT_ION_FON	0x3

#define OMAP_MMC_SYSCONFIG_SIDLE_FORCEIDLE	0x0
#define OMAP_MMC_SYSCONFIG_SIDLE_NOIDLE		0x1
#define OMAP_MMC_SYSCONFIG_SIDLE_SMARTIDLE	0x2

#define OMAP_MMC_SYSCONFIG_ENAWAKEUP	0x1

#define OMAP_MMC_SYSCONFIG_AUTOIDLE	0x1

/* SYSCONFIG bit Masks */
#define OMAP_MMC_SYSCONFIG_CLKACT_SHIFT		0x8
#define OMAP_MMC_SYSCONFIG_SIDLE_SHIFT		0x3
#define OMAP_MMC_SYSCONFIG_ENAWAKEUP_SHIFT	0x2

#define OMAP_MMC_SYSCONFIG_LVL1	0x1
#define OMAP_MMC_SYSCONFIG_LVL2	0x2
#endif /* #ifdef AGGR_PM_CAP */

#if defined(CONFIG_MACH_OMAP_2430SDP) \
 || defined(CONFIG_MACH_OMAP_3430SDP) \
 || defined(CONFIG_MACH_BRISKET) \
 || defined(CONFIG_MACH_FLANK)   \
 || defined(CONFIG_MACH_SIRLOIN)
extern int enable_mmc_power(int slot);
extern int disable_mmc_power(int slot);
extern int mask_carddetect_int(int slot);
extern int unmask_carddetect_int(int slot);
extern int setup_mmc_carddetect_irq(int irq);
extern int switch_power_mode(int power_mode);

extern ssize_t mmc_omap_show_cover_switch(struct device *dev, struct
					device_attribute *attr, char *buf);
extern ssize_t set_mmc_carddetect(struct device *dev, struct device_attribute
				*attr, const char *buf, size_t count);
DEVICE_ATTR(mmc_cover_switch, S_IRUGO, mmc_omap_show_cover_switch, NULL);
DEVICE_ATTR(mmc_card_detect, S_IWUSR, NULL, set_mmc_carddetect);
#endif

struct mmc_omap_host *saved_host1, *saved_host2;

struct mmc_omap_host {
	int		suspended;
	struct		mmc_host *mmc;
	struct		mmc_request *mrq;
	struct		mmc_command *cmd;
	struct		mmc_data *data;
#ifdef CONFIG_OMAP_SDIO
	struct		mmc_data *sdiodata;
	int		sdio_card_intr;
#endif
	struct		timer_list detect_timer;
	struct		resource *mem_res;
	void		__iomem *base;
	void		*mapbase;
	struct		clk *fclk, *iclk, *dbclk;
	/* Required for a 3430 ES1.0 Sil errata fix */
	struct		clk *gptfck;
	unsigned int	id;
	int		irq;
	int		card_detect_irq;
	unsigned char	bus_mode;
	struct		semaphore sem;
	unsigned char	datadir;
	u32		*buffer;
	u32		bytesleft;
	int		use_dma, dma_ch;
	unsigned int	dma_len;
	unsigned int	dma_dir;
	struct		work_struct mmc_carddetect_work;
	int		initstream;
};

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
static struct omap_hsmmc_regs hsmmc_ctx[2];
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

#ifdef CONFIG_OMAP_SDIO
static int blkmode_bytecount[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static int polling_mode = 0;

static void 
sdio_omap_polling_extended( struct mmc_omap_host *host,
                            struct mmc_command *cmd, u32 cmdreg );
static void 
sdio_omap_polling_direct  ( struct mmc_omap_host *host,
                            struct mmc_command *cmd, u32 cmdreg );

#endif /* ifdef CONFIG_OMAP_SDIO */

#ifdef CONFIG_MMC_OMAP3430
static spinlock_t mmc_gpt_lock;
static int gptfclk_counter;
#endif /* #ifdef CONFIG_MMC_OMAP3430 */

static int mmc_clk_counter [NO_OF_MMC_HOSTS];

#ifdef AGGR_PM_CAP
static int omap_mmc_sysconfig (struct mmc_omap_host *host, int level)
{
	u32 sysconfig_val;

	switch (level) {
	case OMAP_MMC_SYSCONFIG_LVL1:
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
		sysconfig_val = (
					(OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FON <<
					OMAP_MMC_SYSCONFIG_CLKACT_SHIFT) |
					(OMAP_MMC_SYSCONFIG_SIDLE_SMARTIDLE <<
					OMAP_MMC_SYSCONFIG_SIDLE_SHIFT) |
					OMAP_MMC_SYSCONFIG_AUTOIDLE
				);

		OMAP_HSMMC_WRITE(host->base, SYSCONFIG, sysconfig_val);
		break;

	case OMAP_MMC_SYSCONFIG_LVL2:
		/*
		 * Clock activity has ICLK and FCLK OFF
		 */
		sysconfig_val = (
					(OMAP_MMC_SYSCONFIG_CLKACT_IOFF_FOFF <<
					OMAP_MMC_SYSCONFIG_CLKACT_SHIFT) |
					OMAP_MMC_SYSCONFIG_AUTOIDLE
				);

		OMAP_HSMMC_WRITE(host->base, SYSCONFIG, sysconfig_val);
		break;
	}
	return 0;
}
#endif /* #ifdef AGGR_PM_CAP */

#ifdef CONFIG_OMAP34XX_OFFMODE
static void omap2_hsmmc_save_ctx(struct mmc_omap_host *host)
{
	/* MMC : context save */
	hsmmc_ctx[host->id - 1].hctl = OMAP_HSMMC_READ(host->base, HCTL);
	hsmmc_ctx[host->id - 1].capa = OMAP_HSMMC_READ(host->base, CAPA);
	hsmmc_ctx[host->id - 1].sysconfig = OMAP_HSMMC_READ(host->base,
								SYSCONFIG);
	hsmmc_ctx[host->id - 1].ise = OMAP_HSMMC_READ(host->base, ISE);
	hsmmc_ctx[host->id - 1].ie = OMAP_HSMMC_READ(host->base, IE);
	hsmmc_ctx[host->id - 1].con = OMAP_HSMMC_READ(host->base, CON);
	hsmmc_ctx[host->id - 1].sysctl = OMAP_HSMMC_READ(host->base, SYSCTL);
}

static void omap2_hsmmc_restore_ctx(struct mmc_omap_host *host)
{
	/* MMC : context restore */
	OMAP_HSMMC_WRITE(host->base, HCTL, hsmmc_ctx[host->id - 1].hctl);
	OMAP_HSMMC_WRITE(host->base, CAPA, hsmmc_ctx[host->id - 1].capa);
	OMAP_HSMMC_WRITE(host->base, CON, hsmmc_ctx[host->id - 1].con);
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG,
					hsmmc_ctx[host->id - 1].sysconfig);
	OMAP_HSMMC_WRITE(host->base, ISE, hsmmc_ctx[host->id - 1].ise);
	OMAP_HSMMC_WRITE(host->base, IE, hsmmc_ctx[host->id - 1].ie);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, hsmmc_ctx[host->id - 1].sysctl);
	OMAP_HSMMC_WRITE(host->base, HCTL, OMAP_HSMMC_READ(host->base,
								HCTL) | SDBP);
}
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

static int mmc_clk_enable (struct mmc_omap_host *host)
{
	int hostid = host->id - 1;

#ifdef CONFIG_MMC_OMAP3430
	/* 3430-ES1.0  Sil errata fix */
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		if (!gptfclk_counter) {
			if (clk_enable(host->gptfck) != 0) {
				dev_dbg(mmc_dev(host->mmc),
					"Unable to enable gptfck clock \n");
				goto clk_en_err;
			}
		}
		gptfclk_counter ++;
		spin_unlock(&mmc_gpt_lock);
	}
#endif /* #ifdef CONFIG_MMC_OMAP3430 */

	if (!mmc_clk_counter[hostid]) {
		if (clk_enable(host->iclk) != 0)
			goto clk_en_err1;

		if (clk_enable(host->fclk) != 0)
			goto clk_en_err2;

#ifdef AGGR_PM_CAP
		omap_mmc_sysconfig (host, OMAP_MMC_SYSCONFIG_LVL1);
#endif /* #ifdef AGGR_PM_CAP */
#ifdef CONFIG_OMAP34XX_OFFMODE
		if (context_restore_required(host->fclk))
			omap2_hsmmc_restore_ctx(host);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
	}
	mmc_clk_counter[hostid] ++;
	return 0;

clk_en_err2:
	/* On fclk failure */
	clk_disable(host->iclk);

clk_en_err1:
	/* On iclk failure */
#ifdef CONFIG_MMC_OMAP3430
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		gptfclk_counter --;
		if (!gptfclk_counter)
			clk_disable(host->gptfck);
		spin_unlock(&mmc_gpt_lock);
	}
#endif /* #ifdef CONFIG_MMC_OMAP3430 */

#ifdef CONFIG_MMC_OMAP3430
clk_en_err:
	dev_dbg(mmc_dev(host->mmc),
		"Unable to enable MMC clocks \n");
	spin_unlock(&mmc_gpt_lock);
#endif /* #ifdef CONFIG_MMC_OMAP3430 */
	return -1;
}

static int mmc_clk_disable (struct mmc_omap_host *host)
{
	int hostid = host->id - 1;

	mmc_clk_counter[hostid] --;
	if (!mmc_clk_counter[hostid]) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		omap2_hsmmc_save_ctx(host);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifdef AGGR_PM_CAP
		omap_mmc_sysconfig (host, OMAP_MMC_SYSCONFIG_LVL2);
#endif /* #ifdef AGGR_PM_CAP */
		clk_disable(host->fclk);
		clk_disable(host->iclk);
	}

#ifdef CONFIG_MMC_OMAP3430
	/* 3430-ES1.0  Sil errata fix */
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		spin_lock(&mmc_gpt_lock);
		gptfclk_counter --;
		if (!gptfclk_counter)
			clk_disable(host->gptfck);
		spin_unlock(&mmc_gpt_lock);
	}
#endif /* #ifdef CONFIG_MMC_OMAP3430 */
	return 0;
}

/*
 * Stop clock to the card
 */
static void omap_mmc_stop_clock(struct mmc_omap_host *host)
{
	/* Stop clock to the card */
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			OMAP_HSMMC_READ(host->base, SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(host->base, SYSCTL) & CEN) != 0x0)
		dev_dbg(mmc_dev(host->mmc), "MMC clock not stoped,"
					"clock freq can not be altered\n");
}

/*
 * Send init stream sequence to the card before sending IDLE command
 */
static void send_init_stream(struct mmc_omap_host *host)
{
	int reg = 0, status;
	typeof(jiffies) timeout;

	disable_irq(host->irq);
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, IE, INT_CLEAR);

	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) | INIT_STREAM);
	OMAP_HSMMC_WRITE(host->base, CMD, INIT_STREAM_CMD);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((reg != CC) && time_before(jiffies, timeout)) {
		reg = OMAP_HSMMC_READ(host->base, STAT) & CC;
	}

	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base, CON) & ~INIT_STREAM);

	status = OMAP_HSMMC_READ(host->base, STAT);
	OMAP_HSMMC_WRITE(host->base, STAT, status);

	enable_irq(host->irq);
}

/*
 * Configure the resptype, cmdtype and send the given command to the card
 */
static void
mmc_omap_start_command(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	int  cmdreg = 0, resptype = 0, cmdtype = 0;
#ifdef CONFIG_OMAP_SDIO
	int func = 0, new_size = 0;
#endif	/* ifdef CONFIG_OMAP_SDIO */

	dev_dbg(mmc_dev(host->mmc), "%s: CMD%d, argument 0x%08x\n",
			mmc_hostname(host->mmc), cmd->opcode, cmd->arg);

	host->cmd = cmd;

#ifdef CONFIG_OMAP_SDIO
	if (cmd->opcode == IO_RW_DIRECT) {
		if ((cmd->arg & IO_RW_DIRECT_MASK) == IO_RW_DIRECT_ARG_MASK)
			cmdtype = 0x2;
	}
#endif	/* ifdef CONFIG_OMAP_SDIO */

	mmc_clk_enable_aggressive(host);

	/* Clear status bits and enable interrupts */
	OMAP_HSMMC_WRITE(host->base, STAT, OMAP_HSMMC_STAT_CLEAR);

	switch (RSP_TYPE(mmc_resp_type(cmd))) {
	case RSP_TYPE(MMC_RSP_R1):
	case RSP_TYPE(MMC_RSP_R3):
		/* resp 1, resp 1b */
		resptype = 2;
		break;
	case RSP_TYPE(MMC_RSP_R2):
		resptype = 1;
		break;
	default:
		break;
	}

	cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

	if (cmd->opcode == MMC_READ_SINGLE_BLOCK
		|| cmd->opcode == MMC_READ_MULTIPLE_BLOCK
		|| cmd->opcode == SD_APP_SEND_SCR
		|| (cmd->opcode == SD_SWITCH && cmd->arg == 0xfffff1)
		|| (cmd->opcode == SD_SWITCH && cmd->arg == 0x80fffff1)
		|| (cmd->opcode == MMC_SEND_EXT_CSD && cmd->arg == 0)) {

		if (host->use_dma)
			cmdreg |= DP_SELECT | DDIR | MSBS | BCE | DMA_EN;
		else
			cmdreg |= DP_SELECT | DDIR | MSBS | BCE;

	} else if (cmd->opcode == MMC_WRITE_BLOCK
		|| cmd->opcode == MMC_WRITE_MULTIPLE_BLOCK) {

		if (host->use_dma)
			cmdreg |= DP_SELECT | MSBS | BCE | DMA_EN;
		else
			cmdreg |= DP_SELECT | MSBS | BCE;

		cmdreg &= ~(DDIR);
	}

#ifdef CONFIG_OMAP_SDIO
	/* Handle I/O related command settings */
	if (cmd->opcode == IO_RW_EXTENDED) {
		if (polling_mode == 1) {
			sdio_omap_polling_extended ( host, cmd, cmdreg );
			return;
		}

		if (cmd->arg & OMAP_SDIO_WRITE) {
#ifdef  CONFIG_OMAP_SDIO_NON_DMA_MODE
			cmdreg |= DP_SELECT;
#else
			cmdreg |= DP_SELECT | DMA_EN | BCE | MSBS;
#endif
			cmdreg &= ~(DDIR);
		} else {
#ifdef  CONFIG_OMAP_SDIO_NON_DMA_MODE
			cmdreg |= DP_SELECT | DDIR;
#else
			cmdreg |= DP_SELECT | DDIR | DMA_EN | BCE | MSBS;
#endif
		}
	}

	if (cmd->opcode == IO_RW_DIRECT) {
		if (cmd->arg & OMAP_SDIO_WRITE) {
			cmdreg &= ~(DDIR);
			if ((cmd->arg & sdio_blkmode_mask) ==
					sdio_blkmode_regaddr1) {
				func = ((cmd->arg & sdio_rw_function_mask)
					>> 17);
				new_size = (cmd->arg & 0xFF);
				blkmode_bytecount[func] =
					blkmode_bytecount[func] & 0xFF00;
				blkmode_bytecount[func] =
					blkmode_bytecount[func] | new_size;
			} else if ((cmd->arg & sdio_blkmode_mask) ==
					sdio_blkmode_regaddr2) {
				func = ((cmd->arg & sdio_rw_function_mask)
					>> 17);
				new_size = ((cmd->arg & 0xFF) << 8);
				blkmode_bytecount[func] =
					blkmode_bytecount[func] & 0x00FF;
				blkmode_bytecount[func] =
					blkmode_bytecount[func] | new_size;
			} else {
				cmdreg |= DDIR;
			}
		}
		sdio_omap_polling_direct ( host, cmd, cmdreg );
		return;
	}

#endif	/* ifdef CONFIG_OMAP_SDIO */
	if (cmd->opcode == MMC_GO_IDLE_STATE 
		|| cmd->opcode == MMC_SEND_OP_COND
		|| cmd->opcode == MMC_ALL_SEND_CID)
		OMAP_HSMMC_WRITE(host->base, CON,
				OMAP_HSMMC_READ(host->base, CON) | OD);

	if (cmd->opcode == MMC_GO_IDLE_STATE) {
		if (host->initstream == 0) {
			send_init_stream(host);
			host->initstream = 1;
		}
	}

#ifdef CONFIG_OMAP_SDIO
	if (host->sdio_card_intr) {
		OMAP_HSMMC_WRITE(host->base, ISE, SDIO_CARD_INT_EN_MASK);
		OMAP_HSMMC_WRITE(host->base, IE, SDIO_CARD_INT_EN_MASK);
	} else {
#endif
		OMAP_HSMMC_WRITE(host->base, ISE, INT_EN_MASK);
		OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK);
#ifdef CONFIG_OMAP_SDIO
	}
#endif

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);
}

#ifdef CONFIG_OMAP_SDIO
/*
 * Notify the xfer done on SDIO card to the core
 */
static void
sdio_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *sdiodata)
{
	if (!sdiodata)
		return;
#ifndef  CONFIG_OMAP_SDIO_NON_DMA_MODE
	if (polling_mode == 0)
		dma_unmap_sg(mmc_dev(host->mmc), host->mrq->data->sg,
			host->dma_len, host->dma_dir);
#endif
	host->data = NULL;
	host->sdiodata = NULL;
	host->datadir = OMAP_MMC_DATADIR_NONE;

	mmc_clk_disable_aggressive(host);

	if (!host->cmd) {
		host->mrq = NULL;
		mmc_request_done(host->mmc, sdiodata->mrq);
	}
	return;
}
#endif	/* ifdef CONFIG_OMAP_SDIO */

/*
 * Notify the xfer done on MMC/SD cards to the core
 */
static void
mmc_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *data)
{
	host->data = NULL;

	if (host->use_dma && host->dma_ch != -1) {
		dma_unmap_sg(mmc_dev(host->mmc), data->sg, host->dma_len,
			host->dma_dir);
	}

	host->datadir = OMAP_MMC_DATADIR_NONE;

	if (data->error == MMC_ERR_NONE)
		data->bytes_xfered += data->blocks * (data->blksz);

	mmc_clk_disable_aggressive(host);

	if (!data->stop) {
		host->mrq = NULL;
		mmc_request_done(host->mmc, data->mrq);
		return;
	}

	mmc_omap_start_command(host, data->stop);
}

/*
 * Notify the core about command completion
 */
static void
mmc_omap_cmd_done(struct mmc_omap_host *host, struct mmc_command *cmd)
{
	host->cmd = NULL;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			/* response type 2 */
			cmd->resp[3] = OMAP_HSMMC_READ(host->base, RSP10);
			cmd->resp[2] = OMAP_HSMMC_READ(host->base, RSP32);
			cmd->resp[1] = OMAP_HSMMC_READ(host->base, RSP54);
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP76);
		} else {
			/* response types 1, 1b, 3, 4, 5, 6 */
			cmd->resp[0] = OMAP_HSMMC_READ(host->base, RSP10);
		}
	}
#ifdef CONFIG_OMAP_SDIO
	if (host->mmc->mode == MMC_MODE_SDIO) {
		if (host->sdiodata == NULL || cmd->error != MMC_ERR_NONE) {
			dev_dbg(mmc_dev(host->mmc), "%s: End request, err %x\n",
					mmc_hostname(host->mmc), cmd->error);
			host->mrq = NULL;

			mmc_clk_disable_aggressive(host);

			mmc_request_done(host->mmc, cmd->mrq);
		}
	} else {
#endif
		if (host->data == NULL || cmd->error != MMC_ERR_NONE) {
			dev_dbg(mmc_dev(host->mmc), "%s: End request, err %x\n",
					mmc_hostname(host->mmc), cmd->error);
			host->mrq = NULL;

			mmc_clk_disable_aggressive(host);

			mmc_request_done(host->mmc, cmd->mrq);
		}

#ifdef CONFIG_OMAP_SDIO
	}
#endif
}

/*
 * Dma cleaning in case of command errors
 */
static void mmc_dma_cleanup(struct mmc_omap_host *host)
{
	int dma_ch;

	host->data->error |= MMC_ERR_TIMEOUT;

	if (host->use_dma && host->dma_ch != -1) {
		dma_unmap_sg(mmc_dev(host->mmc), host->data->sg, host->dma_len,
			host->dma_dir);
		dma_ch = host->dma_ch;
		host->dma_ch = -1;
		omap_free_dma(dma_ch);
		up(&host->sem);
	}
}

#if defined(CONFIG_OMAP_SDIO) && defined(CONFIG_OMAP_SDIO_NON_DMA_MODE)
/*
 * Sdio non dma mode data transfer function
 */
static void sdio_non_dma_xfer(struct mmc_omap_host *host)
{
	int i, readCnt, bytec;
	typeof(jiffies) timeout;

	if (host->cmd) {
		timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);

		if (host->cmd->opcode == IO_RW_EXTENDED) {
			bytec = (host->cmd->arg & 0x1FF);

			if (bytec == 0)
				bytec = 0x200;

			readCnt = (bytec / 4);

			if (bytec % 4)
				readCnt++;

			if (host->cmd->arg & OMAP_SDIO_WRITE) {
				while (((OMAP_HSMMC_READ(host->base, PSTATE)
					& BWE) != BRW)
					&& time_before(jiffies, timeout));

				for (i = 0; i < readCnt; i++)
					OMAP_HSMMC_WRITE(host->base, DATA,
							*host->buffer++);
			} else {
				while (((OMAP_HSMMC_READ(host->base, PSTATE)
					& BRE) != BRR)
					&& time_before(jiffies, timeout));

				for (i = 0; i < readCnt; i++)
					*host->buffer++ =
					OMAP_HSMMC_READ(host->base, DATA);
			}
		}
	}
}
#endif

/*
 * The MMC controller IRQ handler
 */
static irqreturn_t mmc_omap_irq(int irq, void *dev_id)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *)dev_id;
	int end_command, end_transfer, status;
	typeof(jiffies) timeout;

	if (host->cmd == NULL && host->data == NULL) {
		status = OMAP_HSMMC_READ(host->base, STAT);
		if (status != 0) {
			OMAP_HSMMC_WRITE(host->base, STAT, status);
		}

		mmc_clk_disable_aggressive(host);

		return IRQ_HANDLED;
	}

	end_command = 0;
	end_transfer = 0;
	if (host->cmd) {
		if (host->cmd->opcode == MMC_SELECT_CARD
			|| host->cmd->opcode == MMC_SWITCH) {
			timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
			while (time_before(jiffies, timeout)) {
				if ((OMAP_HSMMC_READ(host->base, STAT)
					& CC) == CC)
					break;
			}
		}
	}
	status = OMAP_HSMMC_READ(host->base, STAT);
	dev_dbg(mmc_dev(host->mmc), "Status in IRQ %x\n", status);

	if (status & (OMAP_HSMMC_ERR)) {
		if (status & (OMAP_HSMMC_CMD_TIMEOUT) ||
			status & (OMAP_HSMMC_CMD_CRC)) {
			if (host->cmd) {
				if (status & (OMAP_HSMMC_CMD_TIMEOUT)) {
					/*
					 * Timeouts are normal in case of
					 * MMC_SEND_STATUS
					 */
					if (host->cmd->opcode !=
						MMC_ALL_SEND_CID)
						dev_dbg(mmc_dev(host->mmc),
							"CMD Timeout CMD%d\n",
							host->cmd->opcode);
					host->cmd->error |= MMC_ERR_TIMEOUT;
				} else {
					dev_dbg(mmc_dev(host->mmc),
						"%s: Command CRC error CMD%d\n",
						mmc_hostname(host->mmc),
						host->cmd->opcode);
					host->cmd->error |= MMC_ERR_BADCRC;
				}
				end_command = 1;
			}

			if (host->data)
				mmc_dma_cleanup(host);
		}

		if (status & (OMAP_HSMMC_DATA_TIMEOUT) ||
			status & (OMAP_HSMMC_DATA_CRC)) {
			if (host->data) {
				if (status & (OMAP_HSMMC_DATA_TIMEOUT)) {
					dev_dbg(mmc_dev(host->mmc),
						"%s:Data timeout\n",
						mmc_hostname(host->mmc));
					mmc_dma_cleanup(host);
				} else {
					host->data->error |= MMC_ERR_BADCRC;
					dev_dbg(mmc_dev(host->mmc),
							"%s: Data CRC error,"
							" bytes left %d\n",
							mmc_hostname(host->mmc),
							host->bytesleft);
				}
				end_transfer = 1;
			}
		}

		if (status & OMAP_HSMMC_CARD_ERR) {
			dev_dbg(mmc_dev(host->mmc),
				"MMC%d: Card status error (CMD%d)\n",
				host->id, host->cmd->opcode);
			if (host->cmd) {
				host->cmd->error |= MMC_ERR_FAILED;
				end_command = 1;
			}
			if (host->data) {
				host->data->error |= MMC_ERR_FAILED;
				end_transfer = 1;
			}
		}
	}

#ifdef CONFIG_OMAP_SDIO
	if (host->mmc->mode == MMC_MODE_SDIO && host->sdio_card_intr) {
		if (status & OMAP_HSMMC_CARD_INT) {
			dev_dbg(mmc_dev(host->mmc),
				"MMC%d: SDIO CARD interrupt status %x\n",
				host->id, status);
			OMAP_HSMMC_WRITE(host->base, IE, INT_EN_MASK & 
				(~OMAP_HSMMC_CARD_INT));
			host->sdio_card_intr = 0;
			/*	
			 * SDIO Card interrupt notifier code should go here,
			 * it should clear the source of interrupt and then
			 * call again the interrupt enable API.
			 */
			return IRQ_HANDLED;	
		}
	}
#endif

#if defined(CONFIG_OMAP_SDIO) && defined(CONFIG_OMAP_SDIO_NON_DMA_MODE)
	sdio_non_dma_xfer(host);
#endif /* ifdef CONFIG_OMAP_SDIO */

	OMAP_HSMMC_WRITE(host->base, STAT, status);

	if (end_command || (status & CC)) {
		mmc_omap_cmd_done(host, host->cmd);
	}

	if (end_transfer || (status & TC)) {
		if (host->mmc->mode == MMC_MODE_MMC
			|| host->mmc->mode == MMC_MODE_SD)
			mmc_omap_xfer_done(host, host->data);
#ifdef CONFIG_OMAP_SDIO
		else if (host->mmc->mode == MMC_MODE_SDIO)
			sdio_omap_xfer_done(host, host->sdiodata);
#endif	/* ifdef CONFIG_OMAP_SDIO */
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_OMAP_SDIO

/*
 * Function for polling mode read write for SDIO cards
 */
static void 
sdio_omap_polling_extended( struct mmc_omap_host *host,
                            struct mmc_command *cmd, u32 cmdreg )
{
	int i, readCnt, bytec, status = 0;
	typeof(jiffies) timeout;

	if (cmd->arg & OMAP_SDIO_WRITE) {
		cmdreg |= DP_SELECT;
		cmdreg &= ~(DDIR);
	} else {
		cmdreg |= DP_SELECT | DDIR;
	}

	OMAP_HSMMC_WRITE(host->base, STAT, OMAP_HSMMC_STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR  );
	OMAP_HSMMC_WRITE(host->base, IE,  INT_EN_MASK);

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		status = OMAP_HSMMC_READ(host->base, STAT);
		if ((status & CC))
			break;
	}
	if (!(status & CC)) {
		dev_dbg(mmc_dev(host->mmc),
		"SDIO Command error CMD IO_RW_extd\n");
		host->cmd->error |= MMC_ERR_TIMEOUT;
		mmc_omap_cmd_done(host, host->cmd);
		return;
	}

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	bytec = (host->cmd->arg & 0x1FF);
	readCnt = (bytec / 4);

	if (bytec % 4)
		readCnt++;

	if (host->cmd->arg & OMAP_SDIO_WRITE) {
		while (((OMAP_HSMMC_READ(host->base, PSTATE)
			& BWE) != BRW)
			&& time_before(jiffies, timeout)) ;

		for (i = 0; i < readCnt; i++)
			OMAP_HSMMC_WRITE(host->base, DATA, *host->buffer++);
	} else {
		while (((OMAP_HSMMC_READ(host->base, PSTATE) & BRE) != BRR)
			&& time_before(jiffies, timeout)) ;

		for (i = 0; i < readCnt; i++)
			*host->buffer++ = OMAP_HSMMC_READ(host->base, DATA);
	}

	status = 0;
	mmc_omap_cmd_done(host, host->cmd);
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);

	while (time_before(jiffies, timeout)) {
		status = OMAP_HSMMC_READ(host->base, STAT);
		if (status & TC)
			break;
	}

	if (!(status & TC)) {
		dev_dbg(mmc_dev(host->mmc), "SDIO data sending error \n");
		host->data->error = MMC_ERR_TIMEOUT;
		return;
	}

	sdio_omap_xfer_done(host, host->sdiodata);
	return;
}


/*
 * Function for polling mode read write for SDIO cards
 */
static void 
sdio_omap_polling_direct( struct mmc_omap_host *host,
                          struct mmc_command *cmd, u32 cmdreg )
{
	int status = 0;
	u32 ise, ie;
	typeof(jiffies) timeout;

	// DOLATER: add check for CMDI bit in PSTAT 

	ise = OMAP_HSMMC_READ(host->base, ISE);
	ie  = OMAP_HSMMC_READ(host->base, IE);

	OMAP_HSMMC_WRITE(host->base, STAT, OMAP_HSMMC_STAT_CLEAR);
	OMAP_HSMMC_WRITE(host->base, ISE,  INT_CLEAR   );
	OMAP_HSMMC_WRITE(host->base, IE,   INT_EN_MASK );

	OMAP_HSMMC_WRITE(host->base, ARG, cmd->arg);
	OMAP_HSMMC_WRITE(host->base, CMD, cmdreg);

	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while (time_before(jiffies, timeout)) {
		status = OMAP_HSMMC_READ(host->base, STAT);
		if ((status & CC))
			break;
	}
	if (!(status & CC)) {
		dev_dbg(mmc_dev(host->mmc),
		"SDIO Command error CMD IO_RW_extd\n");
		host->cmd->error |= MMC_ERR_TIMEOUT;
		// DOLATER: add better status handling 
	}



//	if( status )
//		printk ( "sdio status = %x\n", status );

	OMAP_HSMMC_WRITE(host->base, STAT, OMAP_HSMMC_STAT_CLEAR );
	OMAP_HSMMC_WRITE(host->base, ISE,  ise );
	OMAP_HSMMC_WRITE(host->base, IE,   ie  );

	mmc_omap_cmd_done ( host, host->cmd );
	return;
}

#endif

/*
 * Turn the socket power ON/OFF
 */
static int mmc_omap_power(struct mmc_omap_host *host, int on)
{
	int ret = 0;

	if (machine_is_omap_2430sdp() ||
	    machine_is_omap_3430sdp() ||
	    machine_is_sirloin() ||
	    machine_is_flank() ||
	    machine_is_brisket()) {
		if (on) {
			ret = enable_mmc_power(host->id);
		} else {
			ret = disable_mmc_power(host->id);
		}
	}

	return ret;
}
/*
 * power switching module for mmc slot 1
 * power_mode=0 switches to 1.8V
 * power_mode=1 switches to 3V
 * Caller makes sure that it calls on  slot 1 with correct cpu revision
 */
static int omap_mmc_switch_opcond(struct mmc_omap_host *host, int power_mode)
{
	int ret = 0;

	mmc_clk_disable(host);

	if (cpu_is_omap2430())
		clk_disable(host->dbclk);

	ret = mmc_omap_power(host,0);
	if (ret != 0)
		dev_dbg(mmc_dev(host->mmc),"Unable to disable power to MMC1\n");

	if (machine_is_omap_2430sdp() ||
	    machine_is_omap_3430sdp() ||
	    machine_is_flank() ||
	    machine_is_brisket()) {
		if (switch_power_mode(power_mode))
			dev_dbg(mmc_dev(host->mmc), "Unable to switch operating"
				"voltage to the card\n");
	}

	mmc_clk_enable(host);

	if (cpu_is_omap2430()) {
		if (clk_enable(host->dbclk) != 0)
			dev_dbg(mmc_dev(host->mmc),
				"Unable to enable MMC1 debounce clock"
				"while switching power\n");
	}

	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) & SDVSCLR);
	if (power_mode == 0) {
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDVS18);
	} else {
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDVS30);
		host->initstream = 0;
	}
	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	return 0;
}

/*
 * Work Item to notify the core about card insertion/removal
 */
static void mmc_omap_detect(struct work_struct *work)
{
	struct mmc_omap_host *host = container_of(work, struct mmc_omap_host,
		mmc_carddetect_work);

	mmc_clk_enable_aggressive(host);

	if (cpu_is_omap34xx() || (cpu_is_omap2430()
		&& omap2_cpu_rev() == 2)) {
		if (host->id == OMAP_MMC1_DEVID) {
			if (!(OMAP_HSMMC_READ(host->base, HCTL)
				& SDVSDET)) {
				if (omap_mmc_switch_opcond(host, 1) != 0)
					dev_dbg(mmc_dev(host->mmc),
					"mmc_omap_detect:switch"
					"command operation failed\n");
				host->mmc->ios.vdd =
					fls(host->mmc->ocr_avail) - 1;
			}
		}
	}
	mmc_detect_change(host->mmc, (HZ * 200) / 1000);

	mmc_clk_disable_aggressive(host);
}

/*
 * Interrupt service routine for handling card insertion and removal
 */
static irqreturn_t mmc_omap_irq_cd(int irq, void *dev_id)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *)dev_id;

	if (machine_is_omap_2430sdp() ||
	    machine_is_omap_3430sdp() ||
	    machine_is_flank() ||
	    machine_is_brisket())
		schedule_work(&host->mmc_carddetect_work);
	else
		dev_dbg(mmc_dev(host->mmc), "Place to implement MMC hotplug"
			"implementation based on what the other"
			"board can support\n");

	return IRQ_HANDLED;
}

/*
 * DMA call back function
 */
static void mmc_omap_dma_cb(int lch, u16 ch_status, void *data)
{
	struct mmc_omap_host *host = (struct mmc_omap_host *)data;
	int dma_ch;

	/*
	 * Only report the error for the time being, until the error handling
	 * for these type of errors is supported from the core
	 */
	if (ch_status & (1 << 11))
		dev_dbg(mmc_dev(host->mmc), " %s :MISALIGNED_ADRS_ERR\n",
			mmc_hostname(host->mmc));

	if (host->dma_ch < 0) {
		dev_dbg(mmc_dev(host->mmc), "%s:"
			"DMA callback while DMA not enabled?\n",
			mmc_hostname(host->mmc));
		return;
	}

	dma_ch = host->dma_ch;
	host->dma_ch = -1;
	omap_free_dma(dma_ch);
	up(&host->sem);
}
/*
 * Configure dma src and destination parameters
 */
static int mmc_omap_config_dma_param(int sync_dir, struct mmc_omap_host *host,
				     struct mmc_data *data)
{
	if (sync_dir == OMAP_DMA_DST_SYNC) {
		omap_set_dma_dest_params(host->dma_ch,
			0, // dest_port required only for OMAP1
			OMAP_DMA_AMODE_CONSTANT,
			(dma_addr_t) (host-> mapbase + OMAP_HSMMC_DATA),0, 0);
		omap_set_dma_src_params(host->dma_ch,
			0, // src_port required only for OMAP1
			OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(&data-> sg[0]), 0, 0);
	} else {
		omap_set_dma_src_params(host->dma_ch,
			0, // src_port required only for OMAP1
			OMAP_DMA_AMODE_CONSTANT,
			(dma_addr_t) (host->mapbase +OMAP_HSMMC_DATA),0, 0);
		omap_set_dma_dest_params(host->dma_ch,
			0, // dest_port required only for OMAP1
			OMAP_DMA_AMODE_POST_INC,
			sg_dma_address(&data->sg[0]), 0, 0);
	}
	return 0;
}

#ifdef CONFIG_OMAP_SDIO
#ifndef CONFIG_OMAP_SDIO_NON_DMA_MODE
/*
 * Routine to configure and start dma for SDIO card
 */
static int
sdio_omap_start_dma_transfer(struct mmc_omap_host *host,
			     struct mmc_request *req)
{
	int sync_dev, sync_dir, dma_ch, ret, readCnt, bytecount;
	int nob = 1, func = 0;
	struct mmc_data *data = req->data;
	/*
	 * If for some reason the DMA transfer is still active,
	 * we wait for timeout period and free the dma
	 */
	if (host->dma_ch != -1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(100);
		if (down_trylock(&host->sem)) {
			dma_ch = host->dma_ch;
			host->dma_ch = -1;
			omap_free_dma(dma_ch);
			up(&host->sem);
			return 1;
		}
	} else {
		if (down_trylock(&host->sem)) {
			dev_dbg(mmc_dev(host->mmc),
				"Semaphore was not initialized \n");
			BUG();
		}
	}

	if (req->cmd->opcode == IO_RW_EXTENDED) {
		if (req->cmd->arg & OMAP_SDIO_WRITE) {
			if (host->id == OMAP_MMC1_DEVID)
				sync_dev = OMAP24XX_DMA_MMC1_TX;
			else
				sync_dev = OMAP24XX_DMA_MMC2_TX;
		} else {
			if (host->id == OMAP_MMC1_DEVID)
				sync_dev = OMAP24XX_DMA_MMC1_RX;
			else
				sync_dev = OMAP24XX_DMA_MMC2_RX;
		}

		ret = omap_request_dma(sync_dev, "SDIO", mmc_omap_dma_cb,
						host,&dma_ch);
		if (ret != 0) {
			dev_dbg(mmc_dev(host->mmc),
				"%s: omap_request_dma() failed with %d\n",
				mmc_hostname(host->mmc), ret);
			return ret;
		}

		host->dma_len =  dma_map_sg(mmc_dev(host->mmc), data->sg,
						data->sg_len, host->dma_dir);
		host->dma_ch = dma_ch;

		if (req->cmd->arg & OMAP_SDIO_WRITE) {
			sync_dir = OMAP_DMA_DST_SYNC;
			mmc_omap_config_dma_param(sync_dir, host, data);
		} else {
			sync_dir = OMAP_DMA_SRC_SYNC;
			mmc_omap_config_dma_param(sync_dir, host, data);
		}

		if (req->cmd->arg & SDIO_BLKMODE) {
			nob = req->cmd->arg & 0x1FF;

			if (nob == 0)
				nob = 0x200;

			func = ((req->cmd->arg & sdio_function_mask) >> 28);
			bytecount = blkmode_bytecount[func];
			readCnt = (bytecount / 4);

			if (bytecount % 4)
				readCnt++;

		} else {
			bytecount = req->cmd->arg & 0x1FF;

			if (bytecount == 0)
				bytecount = 0x200;

			readCnt = (bytecount / 4);

			if (bytecount % 4)
				readCnt++;
		}

		omap_set_dma_transfer_params(dma_ch,
				OMAP_DMA_DATA_TYPE_S32, readCnt,
				nob,OMAP_DMA_SYNC_FRAME, sync_dev,sync_dir);

		omap_start_dma(dma_ch);
	}
	return 0;
}
#endif
#endif	/* ifdef CONFIG_OMAP_SDIO */

/*
 * Rotine to configure and start DMA for the MMC card
 */
static int
mmc_omap_start_dma_transfer(struct mmc_omap_host *host, struct mmc_request *req)
{
	int sync_dev, sync_dir, dma_ch, ret;
	struct mmc_data *data = req->data;

	/*
	 * If for some reason the DMA transfer is still active,
	 * we wait for timeout period and free the dma
	 */
	if (host->dma_ch != -1) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(100);
		if (down_trylock(&host->sem)) {
			dma_ch = host->dma_ch;
			host->dma_ch = -1;
			omap_free_dma(dma_ch);
			up(&host->sem);
			return 1;
		}
	} else {
		if (down_trylock(&host->sem)) {
			dev_dbg(mmc_dev(host->mmc),
				"Semaphore was not initialized \n");
			BUG();
		}
	}

	if (!(data->flags & MMC_DATA_WRITE)) {
		host->dma_dir = DMA_FROM_DEVICE;
		if (host->id == OMAP_MMC1_DEVID)
			sync_dev = OMAP24XX_DMA_MMC1_RX;
		else
			sync_dev = OMAP24XX_DMA_MMC2_RX;
	} else {
		host->dma_dir = DMA_TO_DEVICE;
		if (host->id == OMAP_MMC1_DEVID)
			sync_dev = OMAP24XX_DMA_MMC1_TX;
		else
			sync_dev = OMAP24XX_DMA_MMC2_TX;
	}

	ret = omap_request_dma(sync_dev, "MMC/SD", mmc_omap_dma_cb, host,
						&dma_ch);
	if (ret != 0) {
		dev_dbg(mmc_dev(host->mmc),
			"%s: omap_request_dma() failed with %d\n",
			mmc_hostname(host->mmc), ret);
		return ret;
	}

	host->dma_len = dma_map_sg(mmc_dev(host->mmc), data->sg, data->sg_len,
								host->dma_dir);
	host->dma_ch = dma_ch;

	if (!(data->flags & MMC_DATA_WRITE)) {
		sync_dir = OMAP_DMA_SRC_SYNC;
		mmc_omap_config_dma_param(sync_dir, host, data);
	} else {
		sync_dir = OMAP_DMA_DST_SYNC;
		mmc_omap_config_dma_param(sync_dir, host, data);
	}

	omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S32,
					(data->blksz / 4), data->blocks,
					OMAP_DMA_SYNC_FRAME, sync_dev,
					sync_dir);

	omap_start_dma(dma_ch);
	return 0;
}

/*
 * Routine to configure block leangth for MMC/SD/SDIO cards
 * and intiate the transfer.
 */
static int
mmc_omap_prepare_data(struct mmc_omap_host *host, struct mmc_request *req)
{
	int ret = 0;
#ifdef CONFIG_OMAP_SDIO
	int byte_count = 0, func = 0;
#endif

	host->data = req->data;

#ifdef CONFIG_OMAP_SDIO
	host->sdiodata = req->data;
	if (req->cmd->opcode == IO_RW_EXTENDED) {
		if (req->cmd->arg & OMAP_SDIO_WRITE)
			host->datadir = OMAP_MMC_DATADIR_WRITE;
		else
			host->datadir = OMAP_MMC_DATADIR_READ;

		if ((req->cmd->arg & 0x1FF) == 0)
			byte_count = 0x200;
		else
			byte_count = req->cmd->arg & 0x1FF;

		func = ((req->cmd->arg & sdio_function_mask) >> 28);

		if (req->cmd->arg & SDIO_BLKMODE) {
			OMAP_HSMMC_WRITE(host->base, BLK,
					blkmode_bytecount[func]);
			OMAP_HSMMC_WRITE(host->base, BLK,
					OMAP_HSMMC_READ(host->base,
					BLK) | (byte_count << 16));
		} else {
			OMAP_HSMMC_WRITE(host->base, BLK, byte_count);
			OMAP_HSMMC_WRITE(host->base, BLK,
					OMAP_HSMMC_READ(host->base,BLK)
					| (1 << 16));
		}

#ifdef  CONFIG_OMAP_SDIO_NON_DMA_MODE
		host->buffer = (u32 *) req->data->sdio_buffer_virt;
#else
		if (polling_mode == 0) {
			ret = sdio_omap_start_dma_transfer(host, req);
			if (ret != 0) {
				dev_dbg(mmc_dev(host->mmc),
					"Sdio start dma failure\n");
				return ret;
			} else {
				host->buffer = NULL;
				host->bytesleft = 0;
			}
		} else {
			host->buffer = (u32 *) req->data->sdio_buffer_virt;
		}
#endif
		return 0;
	}
#endif	/* ifdef CONFIG_OMAP_SDIO */
	if (req->data == NULL) {
		host->datadir = OMAP_MMC_DATADIR_NONE;
		OMAP_HSMMC_WRITE(host->base, BLK, BLK_CLEAR);
		return 0;
	}

	OMAP_HSMMC_WRITE(host->base, BLK, (req->data->blksz));
	OMAP_HSMMC_WRITE(host->base, BLK,
					OMAP_HSMMC_READ(host->base,
					BLK) | (req->data->blocks << 16));
	host->datadir = (req->data-> flags & MMC_DATA_WRITE) ?
			OMAP_MMC_DATADIR_WRITE : OMAP_MMC_DATADIR_READ;

	if (host->use_dma) {
		ret = mmc_omap_start_dma_transfer(host, req);
		if (ret != 0) {
			dev_dbg(mmc_dev(host->mmc), "MMC start dma failure\n");
			return ret;
		} else {
			host->buffer = NULL;
			host->bytesleft = 0;
		}
	} else {
		/* Revert to CPU copy */
		host->buffer =
			(u32 *) (page_address(req->data->sg->page) +
				req->data->sg->offset);
		host->bytesleft = req->data->blocks * (req->data->blksz);
		host->dma_ch = -1;
	}

	return 0;
}

/*
 * Request function. Exposed API to core for read/write operation
 */
static void omap_mmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_omap_host *host = mmc_priv(mmc);

	WARN_ON(host->mrq != NULL);
	host->mrq = req;

	mmc_clk_enable_aggressive(host);

	/* Reset MMC Controller's Data FSM */
	if (req->cmd->opcode == MMC_GO_IDLE_STATE) {
		OMAP_HSMMC_WRITE(host->base, SYSCTL,
				OMAP_HSMMC_READ(host->base, SYSCTL) | 1 << 26);
		while (OMAP_HSMMC_READ(host->base, SYSCTL) & (1 << 26)) ;
	}

	if (req->cmd->opcode == SD_APP_SEND_SCR
		|| req->cmd->opcode == MMC_SEND_EXT_CSD)
		mmc->ios.bus_width = MMC_BUS_WIDTH_1;

	if (mmc_omap_prepare_data(host, req))
		dev_dbg(mmc_dev(host->mmc),
			"MMC host %s failed to initiate data transfer\n",
			mmc_hostname(host->mmc));

	mmc_clk_disable_aggressive(host);

	mmc_omap_start_command(host, req->cmd);
}


/*
 * Routine to configure clock values. Exposed API to core
 */
static void omap_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct mmc_omap_host *host = mmc_priv(mmc);
	u16 dsor = 0;
	unsigned long regVal;
	typeof(jiffies) timeout;
#ifdef CONFIG_MMC_OMAP3430
	int *addr;
#endif

	dev_dbg(mmc_dev(host->mmc), "%s: set_ios: clock %dHz busmode %d"
			"powermode %d Vdd %x Bus Width %d\n",
			mmc_hostname(host->mmc), ios->clock, ios->bus_mode,
			ios->power_mode, ios->vdd, ios->bus_width);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		host->initstream = 0;
#ifdef CONFIG_MMC_OMAP3430
		if (host->id == OMAP_MMC1_DEVID) {
			addr = (int *)&CONTROL_PBIAS_1;
			*addr &= ~(1 << 1);
			if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
				*addr &= ~(1 << 9);
		}
#endif
		if (mmc_omap_power(host, 0))
			dev_dbg(mmc_dev(host->mmc),
				"Could not disable power to MMC%d\n",host->id);
		break;
	case MMC_POWER_UP:
		if (mmc_omap_power(host, 1))
			dev_dbg(mmc_dev(host->mmc),
				"Could not enable power to MMC%d\n",host->id);
#ifdef CONFIG_MMC_OMAP3430
		if (host->id == OMAP_MMC1_DEVID) {
			addr = (int *)&CONTROL_PBIAS_1;
			*addr |= (1 << 1);
			if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
				*addr |= (1 << 9);
		}
#endif
		break;
	}

	mmc_clk_enable_aggressive(host);

	switch (mmc->ios.bus_width) {
	case MMC_BUS_WIDTH_8:
               	OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base,CON)
			| EIGHT_BIT);
		break;
	case MMC_BUS_WIDTH_4:
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base,HCTL)
			| FOUR_BIT);
		break;
	case MMC_BUS_WIDTH_1:
		OMAP_HSMMC_WRITE(host->base, CON,
			OMAP_HSMMC_READ(host->base,CON) & ~EIGHT_BIT);
		OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base,HCTL) & ~FOUR_BIT);
		break;
	}

	if (host->id == OMAP_MMC1_DEVID) {
		if ((cpu_is_omap34xx() && is_sil_rev_less_than(OMAP3430_REV_ES2_0))
			|| (cpu_is_omap2430() && omap2_cpu_rev() == 2)) {
			if ((OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET) &&
		    		((ios->vdd == 7) || (ios->vdd == 8))) {
				if (omap_mmc_switch_opcond(host, 0) != 0)
					dev_dbg(mmc_dev(host->mmc),
				       		"omap_mmc_set_ios:"
						"switch operation failed\n");
				host->initstream = 0;
			}
		}
	}

	if (ios->clock) {
		/* Enable MMC_SD_CLK */
		dsor = OMAP_MMC_MASTER_CLOCK / ios->clock;
		if (dsor < 1)
			dsor = 1;

		if (OMAP_MMC_MASTER_CLOCK / dsor > ios->clock)
			dsor++;

		if (dsor > 250)
			dsor = 250;
	}

	omap_mmc_stop_clock(host);
	regVal = OMAP_HSMMC_READ(host->base, SYSCTL);
	regVal = regVal & ~(CLKD_MASK);
	regVal = regVal | (dsor << 6);
	regVal = regVal | (DTO << 16);
	OMAP_HSMMC_WRITE(host->base, SYSCTL, regVal);
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			OMAP_HSMMC_READ(host->base, SYSCTL) | ICE);

	/* wait till the ICS bit is set */
	timeout = jiffies + msecs_to_jiffies(MMC_TIMEOUT_MS);
	while ((OMAP_HSMMC_READ(host->base, SYSCTL) & ICS) != 0x2
		&& time_before(jiffies, timeout)) ;
	/* Enable clock to the card */
	OMAP_HSMMC_WRITE(host->base, SYSCTL,
			OMAP_HSMMC_READ(host->base, SYSCTL) | CEN);

	mmc_clk_disable_aggressive(host);
}

static struct mmc_host_ops mmc_omap_ops = {
	.request = omap_mmc_request,
	.set_ios = omap_mmc_set_ios,
};

#ifdef CONFIG_OMAP_SDIO
/*
 * Routine implementing SDIO polling mode sysfs entry
 */
static ssize_t sdio_polling_switch(struct device *dev,
				struct device_attribute *attr,
			   	const char *buf, size_t count)
{
	char cmd[25];
	int i = 0;

	if (count < 6) {
		dev_dbg(dev, "Invalid string\n");
		return count;
	}

	while (buf[i] != ' ' && buf[i] != '\n' && i < count) {
		cmd[i] = buf[i];
		i++;
	}

	cmd[i] = '\0';
	i++;

	if (!strcmp(cmd, "Enable")) {
		polling_mode = 1;
	} else if (!strcmp(cmd, "Disable")) {
		polling_mode = 0;
	} else {
		dev_dbg(dev, "Unrecognized string\n");
		dev_dbg(dev, "Usage:\n");
		dev_dbg(dev, "echo Enable >"
			"/sys/devices/platform/hsmmc-omap/sdio_polling_switch\n");
		dev_dbg(dev, "echo Disable >"
			"/sys/devices/platform/hsmmc-omap/sdio_polling_switch\n");
	}
	return count;
}

static DEVICE_ATTR(sdio_polling_switch, S_IWUSR, NULL, sdio_polling_switch);
#endif

/*
 * Routine implementing the driver probe method
 */
static int __init omap_mmc_probe(struct platform_device *pdev)
{
	struct omap_mmc_conf *minfo = pdev->dev.platform_data;
	struct mmc_host *mmc;
	struct mmc_omap_host *host = NULL;
	struct resource *res;
	int ret = 0, irq;
#ifdef CONFIG_MMC_OMAP3430
	int *addr;
#endif	

	if (minfo == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (res == NULL || irq < 0)
		return -ENXIO;

	res = request_mem_region(res->start, res->end - res->start + 1,
							pdev->name);
	if (res == NULL)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct mmc_omap_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto mmc_alloc_err;
	}

	host = mmc_priv(mmc);
	host->mmc = mmc;

	sema_init(&host->sem, 1);

	host->use_dma = OMAP_USE_DMA;
	host->dma_ch = -1;
	host->initstream = 0;
	host->mem_res = res;
	host->irq = irq;
	host->id = pdev->id;
	host->mapbase = (void *)host->mem_res->start;
	host->base = (void __iomem *)IO_ADDRESS(host->mapbase);
	mmc->ops = &mmc_omap_ops;
	mmc->f_min = 400000;
	mmc->f_max = 52000000;
#ifdef CONFIG_OMAP_SDIO
	host->sdio_card_intr = 0;
#endif

	if (cpu_is_omap2430()) {
		if (host->id == OMAP_MMC1_DEVID) {
			host->fclk = clk_get(&pdev->dev, "mmchs1_fck");
			if (IS_ERR(host->fclk)) {
				ret = PTR_ERR(host->fclk);
				host->fclk = NULL;
				goto clk_get_err;
			}
			host->iclk = clk_get(&pdev->dev, "mmchs1_ick");
			if (IS_ERR(host->iclk)) {
				ret = PTR_ERR(host->iclk);
				host->iclk = NULL;
				clk_put(host->fclk);
				goto clk_get_err;
			}
			host->dbclk = clk_get(&pdev->dev, "mmchsdb1_fck");
			/*
			 * Only through a error message, MMC can still work
			 * without debounce clock.
			 */
			if (IS_ERR(host->dbclk))
				dev_dbg(mmc_dev(host->mmc),
					"Failed to get debounce"
					"clock for MMC1\n");
		} else {
			host->fclk = clk_get(&pdev->dev, "mmchs2_fck");
			if (IS_ERR(host->fclk)) {
				ret = PTR_ERR(host->fclk);
				host->fclk = NULL;
				goto clk_get_err;
			}
			host->iclk = clk_get(&pdev->dev, "mmchs2_ick");
			if (IS_ERR(host->iclk)) {
				ret = PTR_ERR(host->iclk);
				host->iclk = NULL;
				clk_put(host->fclk);
				goto clk_get_err;
			}
			host->dbclk = clk_get(&pdev->dev, "mmchsdb2_fck");
			/*
			 * Only through a error message, MMC can still work
			 * without debounce clock.
			 */
			if (IS_ERR(host->dbclk))
				dev_dbg(mmc_dev(host->mmc),
			       		"Failed to get"
					"debounce clock for MMC2\n");
		}
	}

	if (cpu_is_omap34xx()) {
		/* 3430-ES1.0  Sil errata fix */
		if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
			host->gptfck = clk_get(&pdev->dev, "gpt10_fck");
			if (IS_ERR(host->gptfck)) {
				ret = PTR_ERR(host->gptfck);
				host->gptfck = NULL;
				goto clk_get_err;
			}
		}

		host->fclk = clk_get(&pdev->dev, "mmc_fck");
		if (IS_ERR(host->fclk)) {
			ret = PTR_ERR(host->fclk);
			host->fclk = NULL;
			goto clk_get_err;
		}

		host->iclk = clk_get(&pdev->dev, "mmc_ick");
		if (IS_ERR(host->iclk)) {
			ret = PTR_ERR(host->iclk);
			clk_put(host->fclk);
			host->iclk = NULL;
			goto clk_get_err;
		}
	}

#ifdef CONFIG_OMAP34XX_OFFMODE
	modify_timeout_value(host->fclk, 500);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */

	mmc_clk_enable(host);

	if (cpu_is_omap2430()) {
		if (clk_enable(host->dbclk) != 0)
			dev_dbg(mmc_dev(host->mmc),
				"Failed to enable debounce clock for MMC%d\n",
				host->id);
	}

	if (cpu_is_omap2430()) {
		omap_ctrl_writel(omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1) | MMC1_ACTIVE_OVERWRITE,
							OMAP243X_CONTROL_DEVCONF1);
		if (minfo->wire4)
			/* OMAP2430 ES2.0 and onwards can support 4-bit */
			if (omap2_cpu_rev() >= 1)
				mmc->caps = MMC_CAP_4_BIT_DATA;
	}

	if (host->id == OMAP_MMC1_DEVID) {
		if (cpu_is_omap34xx()) {
#ifdef CONFIG_MMC_OMAP3430
			addr = (int *)&CONTROL_PBIAS_1;
			*addr |= (1 << 2);

			addr = (int *)&CONTROL_DEVCONF0;
			*addr |= (1 << 24);
#endif
			/* There is no 8-bit field in the structure yet */
			if (minfo->wire4) {
				if (cpu_is_omap3410()) {
					mmc->caps = MMC_CAP_4_BIT_DATA;
				}
				else
					mmc->caps = MMC_CAP_8_BIT_DATA;
			}

			OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base,
					HCTL) | SDVS30);
		} else if (cpu_is_omap2430()) {
			/* OMAP2430 MMC1 on ES1.0 and ES2.1 can support 3V */
			if (omap2_cpu_rev() == 0 || omap2_cpu_rev() > 1) {
				OMAP_HSMMC_WRITE(host->base, HCTL,
						OMAP_HSMMC_READ(host->base,
								HCTL) | SDVS30);
			} else {
				/* OMAP2430 MMC1 ES2.0 - 1.8V only */
				OMAP_HSMMC_WRITE(host->base, HCTL,
						OMAP_HSMMC_READ(host->base,
								HCTL) | SDVS18);
			}
		}
	} else if (host->id == OMAP_MMC2_DEVID) {
		if (cpu_is_omap34xx()) {
#ifdef CONFIG_MMC_OMAP3430
			addr = (int *)&CONTROL_DEVCONF1;
			*addr |= (1 << 6);
#endif

			if (minfo->wire4)
				mmc->caps = MMC_CAP_4_BIT_DATA;
		}
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base,
						HCTL) | SDVS18);
	}

	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;

	if (machine_is_brisket() || 
	    machine_is_sirloin() ||    
	    machine_is_flank()) {
		// expose real voltage
		mmc->ocr_avail = MMC_VDD_29_30 | MMC_VDD_30_31;
		mmc->caps     &=~MMC_CAP_8_BIT_DATA;
	}

	OMAP_HSMMC_WRITE(host->base, CAPA,OMAP_HSMMC_READ(host->base,
							CAPA) | VS30 | VS18);

	mmc->caps |= MMC_CAP_MULTIWRITE | MMC_CAP_BYTEBLOCK | MMC_CAP_MMC_HIGHSPEED |
	 	         MMC_CAP_SD_HIGHSPEED;

	/* Set the controller to AUTO IDLE mode */
	OMAP_HSMMC_WRITE(host->base, SYSCONFIG,
			OMAP_HSMMC_READ(host->base, SYSCONFIG) | AUTOIDLE);
	/* Set SD bus power bit */
	OMAP_HSMMC_WRITE(host->base, HCTL,
			OMAP_HSMMC_READ(host->base, HCTL) | SDBP);

	if (machine_is_omap_2430sdp() ||
	    machine_is_omap_3430sdp() ||
	    machine_is_flank() ||
	    machine_is_brisket()) {
		/*
		 * Create sysfs entries for enabling/disabling hotplug
		 * support for MMC cards
		 */
		if (device_create_file(&pdev->dev,
				&dev_attr_mmc_cover_switch) < 0) {
			dev_dbg(mmc_dev(host->mmc),
		       		"Unable to create sysfs"
				"attribute for MMC1 cover switch\n");
		}
		if (device_create_file(&pdev->dev,
			&dev_attr_mmc_card_detect) < 0) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to create sysfs"
				"attribute for MMC1 card detect\n");
		}
	}

#ifdef CONFIG_OMAP_SDIO
	if (device_create_file(&pdev->dev, &dev_attr_sdio_polling_switch) < 0) {
		dev_dbg(mmc_dev(host->mmc),
			"Unable to create sysfs"
			"attribute for SDIO 1 polling switch\n");
	}
#endif

	/* Request IRQ for MMC operations */
	ret = request_irq(host->irq, mmc_omap_irq, 0, pdev->name,
				host);
	if (ret) {
		dev_dbg(mmc_dev(host->mmc), "Unable to grab HSMMC IRQ");
		goto irq_err;
	}

	host->card_detect_irq = minfo->switch_pin;
	if (minfo->switch_pin >= 0) {
		if (machine_is_omap_2430sdp() || machine_is_omap_3430sdp()) {
			host->card_detect_irq =
				TWL4030_GPIO_IRQ_NO(minfo->switch_pin);
			INIT_WORK(&host->mmc_carddetect_work, mmc_omap_detect);

			if (setup_mmc_carddetect_irq(minfo->switch_pin)) {
				free_irq(host->irq, host);
				goto irq_err;
			}
		}
		if (machine_is_brisket() ||
		    machine_is_flank()) {
			// we are using external gpio pin
			INIT_WORK(&host->mmc_carddetect_work, mmc_omap_detect);

			if (setup_mmc_carddetect_irq(minfo->switch_pin)) {
				free_irq(host->irq, host);
				goto irq_err;
			}
		}
	}

	if (minfo->switch_pin >= 0) {
		ret = request_irq(host->card_detect_irq,
			mmc_omap_irq_cd, IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, 
			pdev->name,  host );
		if (ret < 0) {
			dev_dbg(mmc_dev(host->mmc),
				"Unable to grab CD IRQ");
			free_irq(host->irq, host);
			goto irq_err;
		}
	}

	if (host->id == OMAP_MMC1_DEVID)
		saved_host1 = host;
	else
		saved_host2 = host;

	platform_set_drvdata(pdev, host);

	mmc_clk_disable_aggressive(host);

	mmc_add_host(mmc);
	return 0;

clk_get_err:
	dev_dbg(mmc_dev(host->mmc),
		"Error getting clock for MMC\n");
	if (host) {
		mmc_free_host(mmc);
	}
	return ret;

mmc_alloc_err:
	if (host)
		mmc_free_host(mmc);
	return ret;

irq_err:
	mmc_clk_disable(host);

	if (cpu_is_omap2430())
		clk_disable(host->dbclk);

	clk_put(host->fclk);
	clk_put(host->iclk);

	if (cpu_is_omap2430())
		clk_put(host->dbclk);

	if (host)
		mmc_free_host(mmc);
	return ret;
}

/*
 * Routine implementing the driver remove method
 */
static int omap_mmc_remove(struct platform_device *pdev)
{
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);

	if (host) {
		free_irq(host->irq, host);
		if (host->card_detect_irq >= 0)
			free_irq(host->card_detect_irq, host);
		flush_scheduled_work();

		/* Free the clks */
#ifndef AGGR_PM_CAP
		mmc_clk_disable(host);
#endif /* #ifndef AGGR_PM_CAP */

		if (cpu_is_omap2430())
			clk_disable(host->dbclk);
		clk_put(host->fclk);
		clk_put(host->iclk);
		if (cpu_is_omap2430())
			clk_put(host->dbclk);

#ifdef CONFIG_OMAP_SDIO
		device_remove_file(&pdev->dev,
					&dev_attr_sdio_polling_switch);
#endif
		mmc_free_host(host->mmc);
	}
	return 0;
}

#ifdef CONFIG_PM
/*
 * Routine to suspend the MMC device
 */
static int omap_mmc_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret = 0;
	int status;
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	if (host && host->suspended)
		return 0;

	if (host) {
		/* Notify the core to suspend the host */
		ret = mmc_suspend_host(host->mmc, state);
		if (ret == 0) {
			host->suspended = 1;

		/* Temporarily enabling the clocks for configuration */
		mmc_clk_enable_aggressive(host);

		if (machine_is_omap_2430sdp() ||
		    machine_is_omap_3430sdp() ||
		    machine_is_flank() ||
		    machine_is_brisket()) {
			disable_irq(host->card_detect_irq);
			ret = mask_carddetect_int(host->id);
			if (ret)
				dev_dbg(mmc_dev(host->mmc),
					"Unable to mask the card detect"
					"interrupt in suspend\n");
		}

		if (cpu_is_omap34xx() || (cpu_is_omap2430()
				&& omap2_cpu_rev() == 2)) {
			if (!(OMAP_HSMMC_READ(host->base, HCTL) & SDVSDET)) {
				OMAP_HSMMC_WRITE(host->base,HCTL,
						OMAP_HSMMC_READ(host->base,HCTL)
						& SDVSCLR);
				OMAP_HSMMC_WRITE(host->base,HCTL,
						OMAP_HSMMC_READ(host->base,HCTL)
						|SDVS30);
				OMAP_HSMMC_WRITE(host->base,HCTL,
						OMAP_HSMMC_READ(host->base,HCTL)
						| SDBP);
			}
		}

		/* Disable Interrupts */
		OMAP_HSMMC_WRITE(host->base, ISE, INT_CLEAR);
		OMAP_HSMMC_WRITE(host->base, IE, INT_CLEAR);

		/* Clearing the STAT register*/
		status = OMAP_HSMMC_READ(host->base, STAT);
	        OMAP_HSMMC_WRITE(host->base, STAT, status);
		/* disable clks for MMC1 */
		mmc_clk_disable(host);

		if (cpu_is_omap2430())
			clk_disable(host->dbclk);

		if (cpu_is_omap2430()) {
			if (host->id == OMAP_MMC1_DEVID) {
				if (omap2_cpu_rev() == 2) {
					omap_ctrl_writel(omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1)
						& ~MMC1_ACTIVE_OVERWRITE,
						OMAP243X_CONTROL_DEVCONF1);
				}
			}
		}

		ret = mmc_omap_power(host,0);
		if (ret != 0)
			dev_dbg(mmc_dev(host->mmc),
				"Unable to disable power to MMC1\n");
		host->initstream = 0;
		}
	}
	return ret;
}

/*
 * Routine to resume the MMC device
 */
static int omap_mmc_resume(struct platform_device *pdev)
{
	int ret = 0;
	struct mmc_omap_host *host = platform_get_drvdata(pdev);

	if (host && !host->suspended)
		return 0;

	if (host) {
		if (cpu_is_omap2430()) {
			if (host->id == OMAP_MMC1_DEVID) {
				if (omap2_cpu_rev() == 2)
					omap_ctrl_writel(omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1)
						| MMC1_ACTIVE_OVERWRITE,
						OMAP243X_CONTROL_DEVCONF1);
			}
		}
		ret = mmc_omap_power(host,1);
		if (ret != 0) {
			dev_dbg(mmc_dev(host->mmc),
			       "Unable to enable power to MMC1\n");
			return ret;
		}

#ifndef AGGR_PM_CAP
		mmc_clk_enable(host);
#endif /* #ifndef AGGR_PM_CAP */

		if (cpu_is_omap2430()) {
			if (clk_enable(host->dbclk) != 0)
				dev_dbg(mmc_dev(host->mmc),
		       			"Unable to enable debounce"
					"clock for MMC1\n");
		}

		if (machine_is_omap_2430sdp() ||
		    machine_is_omap_3430sdp() ||
		    machine_is_flank() ||
		    machine_is_brisket()) {
			enable_irq(host->card_detect_irq);
			ret = unmask_carddetect_int(host->id);
			if (ret)
				dev_dbg(mmc_dev(host->mmc),
					"Unable to unmask the card"
					"detect interrupt\n");
		}

		/* Notify the core to resume the host */
		ret = mmc_resume_host(host->mmc);
		if (ret == 0)
			host->suspended = 0;
	}
	return ret;
}

#else
#define omap_mmc_suspend	NULL
#define omap_mmc_resume		NULL
#endif

#ifdef CONFIG_DPM
static int
omap_mmc_pre_scale(int slot, struct notifier_block *op, unsigned long level,
		   void *ptr)
{
	int i = 0, timeout = 20;
	struct mmc_omap_host *host = (slot == MMC1) ? saved_host1 : saved_host2;

	switch (level) {
	case SCALE_PRECHANGE:
		/* If DMA is active then enable the stop at block gap event */
		if (host->dma_ch) {
			OMAP_HSMMC_WRITE(host->base, HCTL,
					OMAP_HSMMC_READ(host->base,HCTL) | SBGR);
			while (((OMAP_HSMMC_READ(host->base, STAT) & TC) != 0x2)
				|| (i < timeout)) {
				i++;
				/*
				 * Wait for 5 Micro seconds before reading the
				 * Block gap status
				 */
				udelay(5);
			}
			host->dma_ch = -1;
		}
		break;
	}
	return 0;
}

static int
omap_mmc_post_scale(int slot, struct notifier_block *op, unsigned long level,
		    void *ptr)
{
	struct mmc_omap_host *host = (slot == MMC1) ? saved_host1 : saved_host2;

	switch (level) {
	case SCALE_POSTCHANGE:
		if (host->dma_ch == -1) {
		/*
		 * Reset the stop at block gap event before re-starting the
		 * transmission
		 */
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base,HCTL) & ~(SBGR));
		/* Restart the transmision from the previously left block */
		OMAP_HSMMC_WRITE(host->base, HCTL,
				OMAP_HSMMC_READ(host->base,HCTL) | CT);
		/* 1ms delay reuired after re-starting transfer */
		mdelay(1);
		}
		break;
	}
	return 0;
}

#if defined(CONFIG_OMAP2430_MMC1) || defined(CONFIG_OMAP3430_MMC1)
/*
 * Prescale function for MMC1 controller
 */
static int
omap_mmc1_scale_prechange(struct notifier_block *op, unsigned long level,
			  void *ptr)
{
	return omap_mmc_pre_scale(MMC1, op, level, ptr);
}

/*
 * Post scale  function for MMC1 controller
 */
static int
omap_mmc1_scale_postchange(struct notifier_block *op, unsigned long level,
			   void *ptr)
{
	return omap_mmc_post_scale(MMC1, op, level, ptr);
}

static struct notifier_block omap_mmc1_pre_scale = {
	.notifier_call = omap_mmc1_scale_prechange,
};

static struct notifier_block omap_mmc1_post_scale = {
	.notifier_call = omap_mmc1_scale_postchange,
};
#endif
#if defined(CONFIG_OMAP2430_MMC2) || defined(CONFIG_OMAP3430_MMC2)
/*
 * Prescale function for MMC2 controller
 */
static int
omap_mmc2_scale_prechange(struct notifier_block *op, unsigned long level,
			  void *ptr)
{
	return omap_mmc_pre_scale(MMC2, op, level, ptr);
}

/*
 * Post scale  function for MMC2 controller
 */
static int
omap_mmc2_scale_postchange(struct notifier_block *op, unsigned long level,
			   void *ptr)
{
	return omap_mmc_post_scale(MMC2, op, level, ptr);
}

static struct notifier_block omap_mmc2_pre_scale = {
	.notifier_call = omap_mmc2_scale_prechange,
};

static struct notifier_block omap_mmc2_post_scale = {
	.notifier_call = omap_mmc2_scale_postchange,
};
#endif
#endif

static struct platform_driver omap_mmc_driver = {
	.probe = omap_mmc_probe,
	.remove = omap_mmc_remove,
	.suspend = omap_mmc_suspend,
	.resume = omap_mmc_resume,
	.driver = {
		   .name = "hsmmc-omap",
		   },
};

#ifdef CONFIG_OMAP_SDIO
/* API for Enable/Disable SDIO card interrupt */
int sdio_card_int_enable(int enable, int slot)
{
	int intr_mask, intr_ena;
	struct mmc_omap_host *host = (slot == MMC1) ? saved_host1 : saved_host2;

	intr_ena = OMAP_HSMMC_READ(host->base, ISE);
	intr_mask = OMAP_HSMMC_READ(host->base, IE);
	host->sdio_card_intr = enable;

	if (enable == SDIO_CARD_INT_DISABLE) {
		intr_ena = intr_ena & ~(OMAP_HSMMC_CARD_INT);
		intr_mask = intr_mask & ~(OMAP_HSMMC_CARD_INT);
	} else {
		intr_ena = intr_ena | OMAP_HSMMC_CARD_INT;
		intr_mask = intr_mask | OMAP_HSMMC_CARD_INT;
	}
	OMAP_HSMMC_WRITE(host->base, ISE, intr_ena);
	OMAP_HSMMC_WRITE(host->base, IE, intr_mask);
	return 0;
}
EXPORT_SYMBOL(sdio_card_int_enable);
#endif

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
#ifdef CONFIG_DPM
#if defined(CONFIG_OMAP2430_MMC1) || defined(CONFIG_OMAP3430_MMC1)
	/* DPM scale registration for MMC1 controller */
	dpm_register_scale(&omap_mmc1_pre_scale, SCALE_PRECHANGE);
	dpm_register_scale(&omap_mmc1_post_scale, SCALE_POSTCHANGE);
#endif
#if defined(CONFIG_OMAP2430_MMC2) || defined(CONFIG_OMAP3430_MMC2)
	/* DPM scale registration for MMC2 controller */
	dpm_register_scale(&omap_mmc2_pre_scale, SCALE_PRECHANGE);
	dpm_register_scale(&omap_mmc2_post_scale, SCALE_POSTCHANGE);
#endif
#endif
	return 0;
}

/*
 * Driver exit method
 */
static void __exit omap_mmc_cleanup(void)
{
	/* Unregister MMC driver */
	platform_driver_unregister(&omap_mmc_driver);

#ifdef CONFIG_DPM
#if defined(CONFIG_OMAP2430_MMC1) || defined(CONFIG_OMAP3430_MMC1)
	/* Unregister DPM scale functions for MMC1 controller */
	dpm_unregister_scale(&omap_mmc1_pre_scale, SCALE_PRECHANGE);
	dpm_unregister_scale(&omap_mmc1_post_scale, SCALE_POSTCHANGE);
#endif
#if defined(CONFIG_OMAP2430_MMC2) || defined(CONFIG_OMAP3430_MMC2)
	/* Unregister DPM scale functions for MMC2 controller */
	dpm_unregister_scale(&omap_mmc2_pre_scale, SCALE_PRECHANGE);
	dpm_unregister_scale(&omap_mmc2_post_scale, SCALE_POSTCHANGE);
#endif
#endif
}

module_init(omap_mmc_init);
module_exit(omap_mmc_cleanup);

MODULE_DESCRIPTION("OMAP 2430/3430 Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
