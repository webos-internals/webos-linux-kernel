/*
 *  linux/drivers/mmc/host/msm_sdcc.c - Qualcomm MSM 7X00A SDCC Driver
 *
 *  Copyright (C) 2007 Google Inc,
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Based on mmci.c
 *
 * Author: San Mehat (san@android.com)
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/log2.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/clk.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>

#include <asm/cacheflush.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/memory.h>
#include <asm/dma-mapping.h>

#include <asm/mach/mmc.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/dma.h>
#include <asm/arch/msm_mmc.h>
#include <linux/jiffies.h>

#ifdef CONFIG_HIGH_RES_TIMERS
#include <linux/hrtimer.h>
#endif

#include "msm_sdcc.h"

#define DRIVER_NAME "msm-sdcc"

#define DBG(host, fmt, args...)	\
	pr_debug("%s: %s: " fmt, mmc_hostname(host->mmc), __func__ , args)

#define MSMSDCC_POLLING_DEBUG	0

#define MSMSDCC_DEBUG_TIMING      1 

#define MSMSDCC_DEBUG_TIMING_PORT 3

#if (MSMSDCC_DEBUG_TIMING && LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS)

#include <linux/hres_counter.h>

static char* dbg_strings[] = { "msmsdcc suspend",
                               "msmsdcc resume",
                               "msmsdcc probe",
                               "msmsdcc request",
                               "msmsdcc set ios",
                               "msmsdcc polling request",
                               "msmsdcc start cmd",
                               "msmsdcc start data",
                               "msmsdcc stop data",
                               "msmsdcc request end",
                               "msmsdcc status notify",
                               "msmsdcc dma complete",
                               "msmsdcc data end isr",
                               "msmsdcc data wq",
                               "msmsdcc irq",
                               "msmsdcc polling rx",
                               "msmsdcc polling tx",
                               "msmsdcc data error",
			     };

enum {
	MSMSDCC_SUSPEND_EVENT =  0,	
 	MSMSDCC_RESUME_EVENT,	
 	MSMSDCC_PROBE_EVENT,	
 	MSMSDCC_REQUEST_EVENT,	
 	MSMSDCC_SET_IOS_EVENT,	
 	MSMSDCC_POLLING_REQUEST_EVENT,	
	MSMSDCC_START_CMD_EVENT,	
	MSMSDCC_START_DATA_EVENT,	
	MSMSDCC_STOP_DATA_EVENT,	
 	MSMSDCC_REQUEST_END_EVENT,	
 	MSMSDCC_STATUS_NOTIFY_EVENT,	
 	MSMSDCC_DMA_COMPLETE_EVENT,	
 	MSMSDCC_DATA_END_ISR_EVENT,	
 	MSMSDCC_DATA_END_WQ_EVENT,	
 	MSMSDCC_IRQ_EVENT,	
 	MSMSDCC_POLLING_RX_EVENT,	
 	MSMSDCC_POLLING_TX_EVENT,	
 	MSMSDCC_DATA_ERR_EVENT,	
};

#define MSMSDCC_LOG(p_host, eventid, arg1, arg2 )                        \
         if ( p_host->pdev_id == MSMSDCC_DEBUG_TIMING_PORT ) {           \
	 	hres_event(dbg_strings[eventid], (u32)arg1, (u32)arg2 ); \
         }                                                               \

#else
	#define MSMSDCC_LOG(args...)
#endif


#if defined(CONFIG_DEBUG_FS)
static void msmsdcc_dbg_createhost(struct msmsdcc_host *);
static struct dentry *debugfs_dir;
#endif

static unsigned int msmsdcc_fmin = 144000;
static unsigned int msmsdcc_fmax = 50000000;
static unsigned int msmsdcc_4bit = 1;
static unsigned int msmsdcc_pwrsave = 0;

static char *msmsdcc_clks[] = { NULL, "sdc1_clk", "sdc2_clk", "sdc3_clk",
				"sdc4_clk" };
static char *msmsdcc_pclks[] = { NULL, "sdc1_pclk", "sdc2_pclk", "sdc3_pclk",
				 "sdc4_pclk" };

#define VERBOSE_COMMAND_TIMEOUTS	0

#define MAX_DATA_END_WAIT_TIME_USEC     3000000
#define MAX_ISR_DATA_END_WAIT_TIME_USEC 200
#define ISR_POLLING_INTERVAL_USEC       5
#define MSMSDCC_POLLING_TIME_USEC       1000000

static void
msmsdcc_start_command(struct msmsdcc_host *host,
		      struct mmc_command *cmd, u32 c);

static void
msmsdcc_dump_fifodata(struct msmsdcc_host *host)
{
	uint32_t reg_datacnt = readl(host->base + MMCIDATACNT);
	uint32_t reg_fifocnt = readl(host->base + MMCIFIFOCNT);
	uint32_t reg_status = readl(host->base + MMCISTATUS);

	printk(KERN_DEBUG "%s: DATACNT = %d, FIFOCNT = %d, STATUS = 0x%.8x\n",
	       mmc_hostname(host->mmc), reg_datacnt, reg_fifocnt, reg_status);
}

static void
msmsdcc_request_end(struct msmsdcc_host *host, struct mmc_request *mrq)
{
	writel(0, host->base + MMCICOMMAND);

	BUG_ON(host->data);

	MSMSDCC_LOG(host, MSMSDCC_REQUEST_END_EVENT, mrq->cmd->opcode, mrq->cmd->error);

	host->mrq = NULL;
	host->cmd = NULL;
#if 0
	if (mrq->data && mrq->data->error)
		mrq->cmd->error = mrq->data->error;
#endif

	if (mrq->data)
		mrq->data->bytes_xfered = host->data_xfered;
	if (mrq->cmd->error == -ETIMEDOUT) {
		mdelay(5);
	}

	/*
	 * Need to drop the host lock here; mmc_request_done may call
	 * back into the driver...
	 */
	spin_unlock(&host->lock);
	mmc_request_done(host->mmc, mrq);
	spin_lock(&host->lock);
}

static void
msmsdcc_stop_data(struct msmsdcc_host *host)
{
	writel(0, host->base + MMCIDATACTRL);
	host->data = NULL;
}

uint32_t msmsdcc_fifo_addr(struct msmsdcc_host *host)
{
	if (host->pdev_id == 1)
		return MSM_SDC1_PHYS + MMCIFIFO;
	else if (host->pdev_id == 2)
		return MSM_SDC2_PHYS + MMCIFIFO;
	else if (host->pdev_id == 3)
		return MSM_SDC3_PHYS + MMCIFIFO;
	else if (host->pdev_id == 4)
		return MSM_SDC4_PHYS + MMCIFIFO;
	else
		BUG();
}

static int
msmsdcc_wait_for_dataend(struct msmsdcc_host *host, unsigned int wait_usec, int polling_interval_ms)
{
	uint32_t reg_status;
	unsigned long now_jiffies, timeout_jiffies;
	unsigned long wait_jiffies = usecs_to_jiffies(wait_usec);
#ifdef CONFIG_HIGH_RES_TIMERS
	struct timespec ts;
	ts.tv_sec  = 0;
	ts.tv_nsec = polling_interval_ms * 1000000;
#endif

	now_jiffies     = jiffies;
	timeout_jiffies = now_jiffies + wait_jiffies;

	while( time_before_eq(now_jiffies,timeout_jiffies)  ) {
		reg_status = readl(host->base + MMCISTATUS);
		if ((reg_status & (MCI_DATAEND | MCI_DATABLOCKEND | MCI_TXACTIVE | MCI_RXACTIVE)) ==
				  (MCI_DATAEND | MCI_DATABLOCKEND))
			break;

		if ( polling_interval_ms ) {
#ifdef CONFIG_HIGH_RES_TIMERS
			hrtimer_nanosleep( &ts, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
#else
			msleep(polling_interval_ms);
#endif
		}

		now_jiffies = jiffies;
	}

	if ( time_after(now_jiffies,timeout_jiffies) )
		return -ETIMEDOUT;
	
	return 0;
}

static int
msmsdcc_wait_for_dataend_isr(struct msmsdcc_host *host, unsigned int wait_usec, unsigned int polling_interval_usec)
{
	uint32_t reg_status;
	int i;


	for ( i = 0 ; i < ((wait_usec / polling_interval_usec) + 1); i++ ) {
		reg_status = readl(host->base + MMCISTATUS);
		if ((reg_status & (MCI_DATAEND | MCI_DATABLOCKEND | MCI_TXACTIVE | MCI_RXACTIVE)) ==
				  (MCI_DATAEND | MCI_DATABLOCKEND)) {
			return 0;
		}
		udelay(polling_interval_usec);
	}
	
	return -ETIMEDOUT;
}

static void
__msmsdcc_dma_complete_data_transaction(struct msmsdcc_host   *host)
{
	struct mmc_request    *mrq = host->mrq;
	uint32_t	      reg_datacnt, reg_status;

	reg_status = readl(host->base + MMCISTATUS);
	BUG_ON(!mrq);

	writel(MCI_DATAEND | MCI_DATABLOCKEND | MCI_TXACTIVE | MCI_RXACTIVE, host->base + MMCICLEAR);

	msmsdcc_stop_data(host);

	/*
	 * According to QCT it is only ok to read datacnt
	 * after the transfer is complete (or otherwise stopped)
	 */
	barrier();
	reg_datacnt = readl(host->base + MMCIDATACNT);


	dma_unmap_sg(mmc_dev(host->mmc), host->dma.sg, host->dma.num_ents,
		     host->dma.dir);

	if (host->dma.user_pages) {
		struct scatterlist *sg = host->dma.sg;
		int i;

		for (i = 0; i < host->dma.num_ents; i++, sg++)
			flush_dcache_page(sg_page(sg));
	}

	if (!mrq->data->error) {
		host->data_xfered = host->xfer_size - reg_datacnt;

		if (host->data_xfered != host->xfer_size) {
			printk(KERN_WARNING
			       "%s: Short xfer (%d != %d), dc %d, flags 0x%x, stat0x%x, reg_status0x%x\n",
			       mmc_hostname(host->mmc), host->data_xfered,
			       host->xfer_size, reg_datacnt, mrq->data->flags, readl(host->base + MMCISTATUS), reg_status);
		}
	} else {
		
		printk(KERN_ERR "%s: DC %d, BS %d, B %d, OP %d ERROR %d\n",
			mmc_hostname(host->mmc), reg_datacnt, mrq->data->blksz,
			mrq->data->blocks, mrq->cmd->opcode, mrq->data->error);
	}

	host->dma.sg = NULL;

	/* Don't send STOP if we're a read but got a command error */
	if (!mrq->data->stop || mrq->cmd->error) {
		writel(0, host->base + MMCICOMMAND);

		host->mrq = NULL;
		host->cmd = NULL;
#if 0
		if (mrq->data && mrq->data->error)
			mrq->cmd->error = mrq->data->error;
#endif
		mrq->data->bytes_xfered = host->data_xfered;
#if 0
		if (mrq->cmd->error == -ETIMEDOUT)
			mdelay(5);
#endif

		mmc_request_done(host->mmc, mrq);
	} else {
		msmsdcc_start_command(host, mrq->data->stop, 0);
	}

	return;
}

static void 
__msmsdcc_data_end_worker(struct work_struct* io_p_work)
{
	struct msmsdcc_host   *host;
	struct msmsdcc_worker *p_data_end_wq;
	int		      rc = 0;
	unsigned long	      flags;
	struct mmc_request    *mrq;

	p_data_end_wq = container_of(io_p_work, struct msmsdcc_worker, worker);
	host          = container_of(p_data_end_wq, struct msmsdcc_host, data_end_wq);

	BUG_ON(!host);
	mrq = host->mrq;
	BUG_ON(!mrq);

	rc = msmsdcc_wait_for_dataend( host, MAX_DATA_END_WAIT_TIME_USEC, 5);

	if (rc) {
		printk(KERN_ERR
			"%s: Timed out in WQ waiting for DATAEND\n", 
			mmc_hostname(host->mmc));
		mrq->data->error = -ETIMEDOUT;
	} 

	spin_lock_irqsave(&host->lock, flags);

	__msmsdcc_dma_complete_data_transaction(host);

	spin_unlock_irqrestore(&host->lock, flags);
	return;

}

static void
msmsdcc_dma_complete_func(struct msm_dmov_cmd *cmd,
			  unsigned int result,
			  struct msm_dmov_errdata *err)
{
	struct msmsdcc_dma_data	*dma_data = container_of(cmd, struct msmsdcc_dma_data, hdr);
	struct msmsdcc_host	*host = dma_data->host;
	struct mmc_request      *mrq = host->mrq;
	unsigned long		flags;
	int			rc = 0;
	uint32_t	        reg_status;	

	spin_lock_irqsave(&host->lock, flags);

	BUG_ON(!host);
	BUG_ON(!mrq);

	if (!(result & DMOV_RSLT_VALID))
		printk(KERN_ERR "%s: DM result not valid\n",
		       mmc_hostname(host->mmc));
	else if (!(result & DMOV_RSLT_DONE)) {
		/*
		 * Either an error or a flush occurred
		 */
		if (result & DMOV_RSLT_ERROR)
			printk(KERN_ERR "%s: DMA error (0x%.8x)\n",
			       mmc_hostname(host->mmc), result);
		if (result & DMOV_RSLT_FLUSH)
			printk(KERN_ERR "%s: DMA channel flushed (0x%.8x)\n",
			       mmc_hostname(host->mmc), result);
		msmsdcc_dump_fifodata(host);
		if (err)
			printk(KERN_ERR
			       "Flush data: %.8x %.8x %.8x %.8x %.8x %.8x\n",
			       err->flush[0], err->flush[1], err->flush[2],
			       err->flush[3], err->flush[4], err->flush[5]);
		if (!mrq->data->error)
			mrq->data->error = -EIO;
	}

	reg_status = readl(host->base + MMCISTATUS);
	if ((result & DMOV_RSLT_DONE)
	  && (reg_status & (MCI_DATAEND | MCI_DATABLOCKEND | MCI_TXACTIVE | MCI_RXACTIVE))
			 !=(MCI_DATAEND | MCI_DATABLOCKEND)) {
#if 0
		printk(KERN_ERR
		       "%s: DMA result 0x%.8x but still waiting for DATAEND (0x%.8x), flags0x%x\n",
		       mmc_hostname(host->mmc), result, reg_status, mrq->data->flags);
#endif

		if (result & DMOV_RSLT_VALID) {
			rc = msmsdcc_wait_for_dataend_isr( host,
						           MAX_ISR_DATA_END_WAIT_TIME_USEC,
                                                           ISR_POLLING_INTERVAL_USEC );
			if (rc) {
				
#if 0
				printk(KERN_ERR
				       "%s: Timed out in ISR waiting for DATAEND\n", 
				       mmc_hostname(host->mmc));
#endif
				//Continue waiting in workqueue thread
				queue_work(host->data_end_wq.p_worker, &(host->data_end_wq.worker));			
				goto out;
			} 
		}
	}

	__msmsdcc_dma_complete_data_transaction(host);

out:
	spin_unlock_irqrestore(&host->lock, flags);
	return;
}

static int validate_dma(struct msmsdcc_host *host, struct mmc_data *data)
{
	if (host->dma.channel == -1)
		return -ENOENT;
	if ((data->blksz * data->blocks) < MCI_FIFOSIZE)
		return -EINVAL;
	if ((data->blksz * data->blocks) % MCI_FIFOSIZE)
		return -EINVAL;
	return 0;
}

static int msmsdcc_config_dma(struct msmsdcc_host *host, struct mmc_data *data)
{
	struct msmsdcc_nc_dmadata *nc;
	dmov_box *box;
	uint32_t rows;
	uint32_t crci;
	unsigned int n;
	int i, rc;
	struct scatterlist *sg = data->sg;
	
	rc = validate_dma(host, data);
	if (rc)
		return rc;

	host->dma.sg = data->sg;
	host->dma.num_ents = data->sg_len;
	nc = host->dma.nc;

	if (host->pdev_id == 1)
		crci = DMOV_SDC1_CRCI;
	else if (host->pdev_id == 2)
		crci = DMOV_SDC2_CRCI;
	else if (host->pdev_id == 3)
		crci = DMOV_SDC3_CRCI;
	else if (host->pdev_id == 4)
		crci = DMOV_SDC4_CRCI;
	else {
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOENT;
	}

	if (data->flags & MMC_DATA_READ)
		host->dma.dir = DMA_FROM_DEVICE;
	else
		host->dma.dir = DMA_TO_DEVICE;

	host->dma.user_pages = (data->flags & MMC_DATA_USERPAGE);

	n = dma_map_sg(mmc_dev(host->mmc), host->dma.sg,
			host->dma.num_ents, host->dma.dir);

	if (n != host->dma.num_ents) {
		printk(KERN_ERR "%s: Unable to map in all sg elements\n",
		       mmc_hostname(host->mmc));
		host->dma.sg = NULL;
		host->dma.num_ents = 0;
		return -ENOMEM;
	}

	box = &nc->cmd[0];
	for (i = 0; i < host->dma.num_ents; i++) {
		box->cmd = CMD_MODE_BOX;

		if (i == (host->dma.num_ents - 1))
			box->cmd |= CMD_LC;
		rows = (sg_dma_len(sg) % MCI_FIFOSIZE) ?
			(sg_dma_len(sg) / MCI_FIFOSIZE) + 1:
			(sg_dma_len(sg) / MCI_FIFOSIZE) ;

		if (data->flags & MMC_DATA_READ) {
			box->src_row_addr = msmsdcc_fifo_addr(host);
			box->dst_row_addr = sg_dma_address(sg);

			box->src_dst_len = (MCI_FIFOSIZE << 16) | (MCI_FIFOSIZE);
			box->row_offset = MCI_FIFOSIZE;

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_SRC_CRCI(crci);
		} else {
			box->src_row_addr = sg_dma_address(sg);
			box->dst_row_addr = msmsdcc_fifo_addr(host);

			box->src_dst_len = (MCI_FIFOSIZE << 16) | (MCI_FIFOSIZE);
			box->row_offset = (MCI_FIFOSIZE << 16);

			box->num_rows = rows * ((1 << 16) + 1);
			box->cmd |= CMD_DST_CRCI(crci);
		}
		box++;
		sg++;
	}

	/* location of command block must be 64 bit aligned */
	BUG_ON(host->dma.cmd_busaddr & 0x07);

	nc->cmdptr = (host->dma.cmd_busaddr >> 3) | CMD_PTR_LP;
	host->dma.hdr.cmdptr = DMOV_CMD_PTR_LIST |
			       DMOV_CMD_ADDR(host->dma.cmdptr_busaddr);
	host->dma.hdr.complete_func = msmsdcc_dma_complete_func;

	return 0;
}

static void
msmsdcc_start_data(struct msmsdcc_host *host, struct mmc_data *data)
{
	unsigned int datactrl, timeout;
	unsigned long long clks;
	void __iomem *base = host->base;

	host->data = data;
	host->xfer_size = data->blksz * data->blocks;
	host->xfer_remain = host->xfer_size;
	host->data_xfered = 0;

	MSMSDCC_LOG(host, MSMSDCC_START_DATA_EVENT, data->mrq->cmd->opcode, host->xfer_size );

	clks = (unsigned long long)data->timeout_ns * host->clk_rate;
	do_div(clks, 1000000000UL);
	timeout = data->timeout_clks + (unsigned int)clks;

	//Set the minimal timeout for write operations
	if (!(data->flags & MMC_DATA_READ)) {
		unsigned int timeout_min;
		unsigned long long clks_min;

		clks_min = (unsigned long long) MAX_DATA_END_WAIT_TIME_USEC * 1000 * host->clk_rate;
		do_div(clks_min, 1000000000UL);
		timeout_min = (unsigned int) clks_min;

		if ( timeout_min > timeout ) {
			timeout = timeout_min;
		}
	}

#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
	host->data_timeout = timeout;
#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */

	writel(timeout, base + MMCIDATATIMER);

	writel(host->xfer_size, base + MMCIDATALENGTH);

	datactrl = MCI_DPSM_ENABLE | (data->blksz << 4);

	if (!msmsdcc_config_dma(host, data))
		datactrl |= MCI_DPSM_DMAENABLE;

	if (data->flags & MMC_DATA_READ)
		datactrl |= MCI_DPSM_DIRECTION;

	writel(datactrl, base + MMCIDATACTRL);
#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
	host->hres_data_start = hres_get_counter();
#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */

	if (datactrl & MCI_DPSM_DMAENABLE)
		msm_dmov_enqueue_cmd(host->dma.channel, &host->dma.hdr);
}

static void
msmsdcc_start_command(struct msmsdcc_host *host, struct mmc_command *cmd, u32 c)
{
	void __iomem *base = host->base;

	MSMSDCC_LOG(host, MSMSDCC_START_CMD_EVENT, cmd->opcode, cmd->arg );

	DBG(host, "op %02x arg %08x flags %08x\n",
	    cmd->opcode, cmd->arg, cmd->flags);

	if (readl(base + MMCICOMMAND) & MCI_CPSM_ENABLE) {
		writel(0, base + MMCICOMMAND);
		udelay(2 + ((5 * 1000000) / host->clk_rate));
	}

	c |= cmd->opcode | MCI_CPSM_ENABLE;

	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136)
			c |= MCI_CPSM_LONGRSP;
		c |= MCI_CPSM_RESPONSE;
	}

	if (/*interrupt*/0)
		c |= MCI_CPSM_INTERRUPT;

	if ((((cmd->opcode == 17) || (cmd->opcode == 18))  ||
	     ((cmd->opcode == 24) || (cmd->opcode == 25))) ||
	      (cmd->opcode == 53))
		c |= MCI_CSPM_DATCMD;

	if (cmd == cmd->mrq->stop)
		c |= MCI_CSPM_MCIABORT;

	host->cmd = cmd;

#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
	host->hres_cmd_start = hres_get_counter();
#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */

	writel(cmd->arg, base + MMCIARGUMENT);
	writel(c, base + MMCICOMMAND);
}

static void
msmsdcc_data_err(struct msmsdcc_host *host, struct mmc_data *data,
		 unsigned int status)
{
	if (status & MCI_DATACRCFAIL) {
		printk(KERN_ERR "%s: Data CRC error\n", mmc_hostname(host->mmc));
		printk(KERN_ERR "%s: opcode 0x%.8x\n", __func__, data->mrq->cmd->opcode);
		printk(KERN_ERR "%s: blksz %d, blocks %d\n", __func__, data->blksz, data->blocks);
		data->error = -EILSEQ;
	} else if (status & MCI_DATATIMEOUT) {
		unsigned int clk_rate, pclk_rate;
#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
		u32 hres_now = hres_get_counter();
		u32 hres_delta_cmd  = hres_now - host->hres_cmd_start;
		u32 hres_delta_data = hres_now - host->hres_data_start;
#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */
		clk_rate = clk_get_rate(host->clk);
		pclk_rate = clk_get_rate(host->pclk);

		printk(KERN_ERR "%s: Data timeout\n", mmc_hostname(host->mmc));
		printk(KERN_ERR "%s: opcode 0x%.8x\n", __func__, data->mrq->cmd->opcode);
		printk(KERN_ERR "%s: blksz %d, blocks %d\n", __func__, data->blksz, data->blocks);
		printk(KERN_ERR "%s: clk=%u, pclk=%u\n", mmc_hostname(host->mmc), clk_rate, pclk_rate);
#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
		printk(KERN_ERR"HRES cmd %d (%d usec) data %d (%d usec)\n", 
                       hres_delta_cmd , hres_get_delta_usec( host->hres_cmd_start ,hres_now),
                       hres_delta_data , hres_get_delta_usec( host->hres_data_start ,hres_now));
		printk(KERN_ERR"Data timeout clocks %d ns %d msmsdcc %d\n", data->timeout_clks, data->timeout_ns , host->data_timeout );
#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */

		data->error = -ETIMEDOUT;

		MSMSDCC_LOG(host, MSMSDCC_DATA_ERR_EVENT, data->mrq->cmd->opcode, data->blocks );

		panic("MMC timeout error.");
	} else if (status & MCI_RXOVERRUN) {
		printk(KERN_ERR "%s: RX overrun\n", mmc_hostname(host->mmc));
		data->error = -EIO;
	} else if (status & MCI_TXUNDERRUN) {
		printk(KERN_ERR "%s: TX underrun\n", mmc_hostname(host->mmc));
		data->error = -EIO;
	} else {
		printk(KERN_ERR "%s: Unknown error (0x%.8x)\n",
		      mmc_hostname(host->mmc), status);
		data->error = -EIO;
	}
}

/*
 * IRQ Handler for processing DMA request related errors and
 * command completion.
 */
static irqreturn_t
msmsdcc_irq(int irq, void *dev_id)
{
	struct msmsdcc_host	*host = dev_id;
	void __iomem		*base = host->base;
	u32			status;
	int			ret = 0;

	spin_lock(&host->lock);

	do {
		struct mmc_command *cmd;
		struct mmc_data *data;
		status = readl(host->base + MMCISTATUS);

		DBG(host, "irq0 %08x\n", status);

		status &= readl(host->base + MMCIMASK0);
		writel(status, host->base + MMCICLEAR);

		/*
		 * Check for data errors
		 */
		data = host->data;
		if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|
			      MCI_RXOVERRUN) && data) {
			msmsdcc_data_err(host, data, status);
			msm_dmov_stop_cmd(host->dma.channel, &host->dma.hdr, 0);
		}


		/*
		 * Check for proper command response
		 */
		cmd = host->cmd;
		if (status & (MCI_CMDSENT | MCI_CMDRESPEND | MCI_CMDCRCFAIL |
			      MCI_CMDTIMEOUT) && cmd) {
			host->cmd = NULL;
			cmd->resp[0] = readl(base + MMCIRESPONSE0);
			cmd->resp[1] = readl(base + MMCIRESPONSE1);
			cmd->resp[2] = readl(base + MMCIRESPONSE2);
			cmd->resp[3] = readl(base + MMCIRESPONSE3);

			del_timer(&host->command_timer);
			if (status & MCI_CMDTIMEOUT) {
#if VERBOSE_COMMAND_TIMEOUTS

			#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
				u32 hres_now = hres_get_counter();
				u32 hres_delta_cmd  = hres_now - host->hres_cmd_start;
			#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */

				printk(KERN_ERR "%s: IRQ Command timeout opcode %d\n",
				       mmc_hostname(host->mmc), cmd->opcode);

			#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
				printk(KERN_ERR"IRQ HRES cmd %d (%d usec)\n", 
					hres_delta_cmd , hres_get_delta_usec( host->hres_cmd_start ,hres_now));
			#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */
#endif /* VERBOSE_COMMAND_TIMEOUTS */
				MSMSDCC_LOG(host, MSMSDCC_IRQ_EVENT, 1, cmd->opcode );

				cmd->error = -ETIMEDOUT;
			} else if (status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC) {
				printk(KERN_ERR "%s: IRQ Command CRC error opcode %d\n",
				       mmc_hostname(host->mmc), cmd->opcode);
				cmd->error = -EILSEQ;
				MSMSDCC_LOG(host, MSMSDCC_IRQ_EVENT, 2, cmd->opcode );
			}

			if (status & (MCI_CMDSENT | MCI_CMDRESPEND)) {
				MSMSDCC_LOG(host, MSMSDCC_IRQ_EVENT, 3, cmd->opcode );
			}

			if (!cmd->data || cmd->error) {
				if (host->data && host->dma.sg)
					msm_dmov_stop_cmd(host->dma.channel,
							  &host->dma.hdr, 0);
				else if (host->data) { /* Non DMA transfer */
					msmsdcc_stop_data(host);
					msmsdcc_request_end(host, cmd->mrq);
				} else /* host->data == NULL */
					msmsdcc_request_end(host, cmd->mrq);
			} else if (!(cmd->data->flags & MMC_DATA_READ))
				msmsdcc_start_data(host, cmd->data);
		}

		ret = 1;
	} while (status);

	spin_unlock(&host->lock);

	return IRQ_RETVAL(ret);
}


static int
msmsdcc_waitfor_cmd(struct msmsdcc_host *host, struct mmc_command *cmd,
		    uint32_t *status)
{
	int i;

	for ( i = 0; i < (( MSMSDCC_POLLING_TIME_USEC / ISR_POLLING_INTERVAL_USEC ) + 1); i++) {
		*status = readl(host->base + MMCISTATUS);

		if (*status & MCI_CMDCRCFAIL && cmd->flags & MMC_RSP_CRC)
			return -EILSEQ;
		if (*status & MCI_CMDTIMEOUT)
			return -ETIMEDOUT;
		if (*status & (MCI_CMDSENT | MCI_CMDRESPEND))
			return 0;

		udelay( ISR_POLLING_INTERVAL_USEC );
	}

#if MSMSDCC_POLLING_DEBUG
	printk("%s: Timed out waiting for command status\n", __func__);
#endif
	return -ETIMEDOUT;
}

static int
msmsdcc_polling_rx(struct msmsdcc_host *host, struct mmc_data *data)
{
	void __iomem		*base = host->base;
	uint32_t		brtr = data->blksz * data->blocks;
	struct scatterlist	*sg = data->sg;
	unsigned int		sg_len = data->sg_len;
	unsigned int		sg_off = 0;
	unsigned int		count = 0;
	uint32_t		status;
	int			rc;
	int                     iteration_cnt, timeout_cnt;

	iteration_cnt	= 0;
	timeout_cnt = MSMSDCC_POLLING_TIME_USEC / ISR_POLLING_INTERVAL_USEC + 1;

	writel(0x18007ff, host->base + MMCICLEAR);
	while(brtr) {
		unsigned int	sg_remain;
		unsigned long	flags;
		char		*buffer;
		uint32_t	*ptr;

		local_irq_save(flags);
		buffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
		ptr = (uint32_t *) buffer;

		sg_remain = sg->length - sg_off;

		while(sg_remain) {

			/* Wait for FIFO data */
			status = readl(base + MMCISTATUS);

			if (status & (MCI_DATACRCFAIL | MCI_DATATIMEOUT |
				      MCI_TXUNDERRUN)) {
				if (status & MCI_DATACRCFAIL)
					data->error = -EILSEQ;
				else if (status & MCI_DATATIMEOUT)
					data->error = -ETIMEDOUT;
				else
					data->error = -EIO;
				printk(KERN_ERR "%s: Data error (%d)\n",
				       mmc_hostname(host->mmc), data->error);

				kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
				local_irq_restore(flags);
				goto out;
			}
		
			if (status & MCI_RXDATAAVLBL) {
				*ptr = readl(base + MMCIFIFO);
				ptr++;
				count += sizeof(uint32_t);
				sg_off += sizeof(uint32_t);
				sg_remain -= sizeof(uint32_t);
				brtr -= sizeof(uint32_t);
				data->bytes_xfered += sizeof(uint32_t);

				//Reset the timeout 
				iteration_cnt =  0;

				writel(0x18007ff, host->base + MMCICLEAR);
				continue;
			}

			udelay(ISR_POLLING_INTERVAL_USEC);
			iteration_cnt++;

			if ( iteration_cnt >= timeout_cnt ) {
				uint32_t datacnt, fifocnt;
				uint32_t dbg[2];

				datacnt = readl(base + MMCIDATACNT);
				fifocnt = readl(base + MMCIFIFOCNT);
				status = readl(base + MMCISTATUS);
				
				printk(KERN_ERR "%s: Timed out waiting for RXDATAAVLBL (st = 0x%.8x)\n", 
				       mmc_hostname(host->mmc), status);
				printk(KERN_ERR "%s: 0x%.8x 0x%.8x 0x%.8x\n",
				       mmc_hostname(host->mmc), datacnt, 
				       fifocnt, status);
				dbg[0] = readl(base + MMCIFIFO);
				dbg[1] = readl(base + MMCIFIFO + 4);
				printk(KERN_ERR "%s: Data = 0x%.8x 0x%.8x\n",
				       mmc_hostname(host->mmc), dbg[0], dbg[1]);
				printk(KERN_ERR "%s: opcode 0x%.8x\n", __func__, data->mrq->cmd->opcode);

				data->error = -ETIMEDOUT;
				kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
				local_irq_restore(flags);
				goto out;
			}
		}

		/* Done with this SG element */
		kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
		local_irq_restore(flags);

		if (--sg_len) {
			/* Advance to next SG element */
			sg++;
			sg_off = 0;
		}
	}

        rc = msmsdcc_wait_for_dataend_isr( host,
   				           MAX_DATA_END_WAIT_TIME_USEC, 
                                           ISR_POLLING_INTERVAL_USEC);


	if (rc) {
		printk(KERN_ERR
		       "%s: Timed out waiting for DATAEND %d\n", 
			mmc_hostname(host->mmc), __LINE__);
	}


#if MSMSDCC_POLLING_DEBUG
	printk(KERN_DEBUG "%s: Rx complete (%d bytes xfered)\n",
	       mmc_hostname(host->mmc), data->bytes_xfered);
#endif

out:
	writel(0x18007ff, host->base + MMCICLEAR);
	return data->error;
}

static int
msmsdcc_polling_tx(struct msmsdcc_host *host, struct mmc_data *data)
{
	void __iomem		*base = host->base;
	uint32_t		brtw = data->blksz * data->blocks;
	struct scatterlist	*sg = data->sg;
	unsigned int		sg_len = data->sg_len;
	unsigned int		sg_off = 0;
	uint32_t		status;
	int			rc;
	int                     iteration_cnt, timeout_cnt;

#if MSMSDCC_POLLING_DEBUG
	printk(KERN_DEBUG "%s: TX blksz %d, blocks %d\n",
	       mmc_hostname(host->mmc), data->blksz, data->blocks);
#endif

	iteration_cnt	= 0;
	timeout_cnt = MSMSDCC_POLLING_TIME_USEC / ISR_POLLING_INTERVAL_USEC + 1;

	writel(0x18007ff, host->base + MMCICLEAR);
	
	while(brtw) {
		unsigned int	sg_remain;
		unsigned long	flags;
		char		*buffer, *ptr;
		u32		data32;

		local_irq_save(flags);
		buffer = kmap_atomic(sg_page(sg), KM_BIO_SRC_IRQ) + sg->offset;
		ptr = buffer;

		sg_remain = sg->length - sg_off;

#if MSMSDCC_POLLING_DEBUG
		printk(KERN_DEBUG "%s: SG buffer @ %p (remain = %d)\n",
		       mmc_hostname(host->mmc), buffer, sg_remain);
#endif
		while(sg_remain) {
			/* Wait for the FIFO to be ready */
			status = readl(base + MMCISTATUS);

			if (status & (MCI_DATACRCFAIL | MCI_DATATIMEOUT |
				      MCI_TXUNDERRUN)) {
				if (status & MCI_DATACRCFAIL)
					data->error = -EILSEQ;
				else if (status & MCI_DATATIMEOUT)
					data->error = -ETIMEDOUT;
				else
					data->error = -EIO;
				printk(KERN_ERR "%s: Data error (%d)\n",
				       mmc_hostname(host->mmc), data->error);
				writel(0x18007ff, host->base + MMCICLEAR);

				kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
				local_irq_restore(flags);
				goto out;
			}
		

			/* Write (up to) a full fifo length */
			if (!(status & MCI_TXFIFOFULL)) {
				data32 = *(u32*)(ptr);

				writel(data32, base + MMCIFIFO);
				ptr		+= 4;
				sg_off		+= 4;
				sg_remain	-= 4;
				brtw		-= 4;
				//Reset the timeout 
				iteration_cnt =  0;

				continue;
			}

			udelay(ISR_POLLING_INTERVAL_USEC);
			iteration_cnt++;

			if ( iteration_cnt >= timeout_cnt ) {
				printk(KERN_ERR
				       "%s: Timed out waiting for TXFIFOEMPTY (0x%.8x)\n", 
				       mmc_hostname(host->mmc), status);
				data->error = -ETIMEDOUT;
				kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
				local_irq_restore(flags);
				goto out;
			}
		}

		/* Done with this SG element */
		kunmap_atomic(buffer, KM_BIO_SRC_IRQ);
		local_irq_restore(flags);

		if (--sg_len) {
			/* Advance to next SG element */
			sg++;
			sg_off = 0;
		}
	}

	/*
	 * Wait for data xmit to complete
	 */
	rc = msmsdcc_wait_for_dataend_isr( host,
				           MAX_DATA_END_WAIT_TIME_USEC, 
                                           ISR_POLLING_INTERVAL_USEC);

	if (rc) {
		printk(KERN_ERR
		       "%s: Timed out waiting for DATAEND\n", 
		       mmc_hostname(host->mmc));
		data->error = -ETIMEDOUT;
		goto out;
	} 

	data->bytes_xfered = data->blksz * data->blocks;
#if MSMSDCC_POLLING_DEBUG
	printk(KERN_DEBUG "%s: Tx complete (%d bytes xfered)\n",
	       mmc_hostname(host->mmc), data->bytes_xfered);
#endif

out:
	writel(0x18007ff, host->base + MMCICLEAR);
	return data->error;
}

static void
msmsdcc_do_polling_request(struct msmsdcc_host *host, struct mmc_request *mrq)
{
	struct mmc_command	*cmd = mrq->cmd;
	struct mmc_data		*data = mrq->data;
	uint32_t		status;
	int			rc;

	writel(0x018007FF, host->base + MMCICLEAR);

	msmsdcc_start_command(host, cmd, 0);
	rc = msmsdcc_waitfor_cmd(host, cmd, &status);
#if MSMSDCC_POLLING_DEBUG
	printk(KERN_DEBUG
	       "%s: Polling waitforcmd rc = %d (status 0x%.8x)\n",
	       mmc_hostname(host->mmc), rc, status);
#endif
	host->cmd = NULL;
	cmd->resp[0] = readl(host->base + MMCIRESPONSE0);
	cmd->resp[1] = readl(host->base + MMCIRESPONSE1);
	cmd->resp[2] = readl(host->base + MMCIRESPONSE2);
	cmd->resp[3] = readl(host->base + MMCIRESPONSE3);

	if (rc) {
		printk(KERN_ERR "%s: Command error (%d)\n",
		       mmc_hostname(host->mmc), rc);
		cmd->error = rc;
		goto done;
	}

	if (data) {
		if (!(mrq->data->flags & MMC_DATA_READ)) {
			msmsdcc_start_data(host, data);
			msmsdcc_polling_tx(host, data);
		} else 
			msmsdcc_polling_rx(host, data);
		msmsdcc_stop_data(host);
	}

done:
#if MSMSDCC_POLLING_DEBUG
	printk(KERN_DEBUG
	       "%s: Done request (cmd_err = %d, dat_err = %d, stop_err = %d)\n",
	       mmc_hostname(host->mmc), cmd->error, 
	       (mrq->data != NULL ? mrq->data->error : -1),
	       (mrq->stop != NULL ? mrq->stop->error : -1));
#endif
	   
	writel(0x18007ff, host->base + MMCICLEAR);
	spin_unlock_irq(&host->lock);
	mmc_request_done(host->mmc, host->mrq);
	return;
}

static void
msmsdcc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	int poll = 0;

	WARN_ON(host->mrq != NULL);

	spin_lock_irq(&host->lock);

	MSMSDCC_LOG(host, MSMSDCC_REQUEST_EVENT, mrq->cmd->opcode, mrq->cmd->arg);

	if (host->eject) {
		if (mrq->data && !(mrq->data->flags & MMC_DATA_READ)) {
			mrq->cmd->error = 0;
			mrq->data->bytes_xfered = mrq->data->blksz * mrq->data->blocks;
		} else
			mrq->cmd->error = -ENOMEDIUM;

		spin_unlock_irq(&host->lock);
		mmc_request_done(mmc, mrq);
		return;
	}

	host->mrq = mrq;

	/*
	 * According to QCT PIO interrupts may be broken,
	 * so if we're not going to use DMA, fall back to
	 * polling
	 */
	if (mrq->data && validate_dma(host, mrq->data)) {

		writel(0, host->base + MMCIMASK0);
		poll = 1;
		
	} else {
		writel(MCI_IRQENABLE, host->base + MMCIMASK0);
	}

	//According to Qcomm guidelines delay is needed here:
        //This register must be updated to the MCLK domain so subsequent writes to this //register will be ignored until 3 MCLK and 3 HCLK have passed
	udelay(1);

	if (mrq->data && mrq->data->flags & MMC_DATA_READ)
		msmsdcc_start_data(host, mrq->data);

	if (poll) {
		msmsdcc_do_polling_request(host, mrq);
		host->mrq = NULL;
	} else {
		msmsdcc_start_command(host, mrq->cmd, 0);
		mod_timer(&host->command_timer, jiffies + (HZ / 2));
		spin_unlock_irq(&host->lock);
	}

}

static void
msmsdcc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct msmsdcc_host *host = mmc_priv(mmc);
	u32 clk = 0, pwr = 0;
	int rc;

	MSMSDCC_LOG(host, MSMSDCC_SET_IOS_EVENT, 0, ios->clock );
	MSMSDCC_LOG(host, MSMSDCC_SET_IOS_EVENT, 1, ios->vdd );
	MSMSDCC_LOG(host, MSMSDCC_SET_IOS_EVENT, 2, ios->bus_mode );
	MSMSDCC_LOG(host, MSMSDCC_SET_IOS_EVENT, 3, ios->power_mode );
	MSMSDCC_LOG(host, MSMSDCC_SET_IOS_EVENT, 4, ios->bus_width );


	if (ios->clock) {

		if (!host->clks_on) {
			clk_enable(host->pclk);
			clk_enable(host->clk);
			host->clks_on = 1;
		}
		if (ios->clock != host->clk_rate) {
			rc = clk_set_rate(host->clk, ios->clock);
			if (rc < 0)
				printk(KERN_ERR
				       "Error setting clock rate (%d)\n", rc);
			else
				host->clk_rate = ios->clock;
		}
		clk |= MCI_CLK_ENABLE;
	} 

	if (ios->bus_width == MMC_BUS_WIDTH_4)
		clk |= (2 << 10); /* Set WIDEBUS */

	if(host->plat->get_powersave_mode)
		msmsdcc_pwrsave = host->plat->get_powersave_mode(mmc_dev(mmc));

	if (ios->clock > 400000 && msmsdcc_pwrsave)
		clk |= (1 << 9); /* PWRSAVE */

	clk |= (1 << 12); /* FLOW_ENA */
	clk |= (1 << 15); /* feedback clock */

	if (host->plat->translate_vdd)
		pwr |= host->plat->translate_vdd(mmc_dev(mmc), ios->vdd);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		break;
	case MMC_POWER_UP:
		pwr |= MCI_PWR_UP;
		break;
	case MMC_POWER_ON:
		pwr |= MCI_PWR_ON;
		break;
	}

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN)
		pwr |= MCI_OD;

	writel(clk, host->base + MMCICLOCK);

	if (host->pwr != pwr) {
		host->pwr = pwr;
		writel(pwr, host->base + MMCIPOWER);
	}

	if (!(clk & MCI_CLK_ENABLE) && host->clks_on) {
		clk_disable(host->clk);
		clk_disable(host->pclk);
		host->clks_on = 0;
	}
}

static const struct mmc_host_ops msmsdcc_ops = {
	.request	= msmsdcc_request,
	.set_ios	= msmsdcc_set_ios,
};

static void
msmsdcc_check_status(unsigned long data)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *)data;
	unsigned int status;

	if (!host->plat->status) {
		mmc_detect_change(host->mmc, 0);
		goto out;
	}

	status = host->plat->status(mmc_dev(host->mmc));
	host->eject = !status;
	if (status ^ host->oldstat) {
		printk(KERN_INFO
		       "%s: Slot status change detected (%d -> %d)\n",
		       mmc_hostname(host->mmc), host->oldstat, status);
		mmc_detect_change(host->mmc, 0);
	}

	host->oldstat = status;

out:
	if (host->timer.function)
		mod_timer(&host->timer, jiffies + HZ);
}

static irqreturn_t
msmsdcc_platform_status_irq(int irq, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;

	printk(KERN_DEBUG "%s: %d\n", __func__, irq);
	msmsdcc_check_status((unsigned long) host);
	return IRQ_HANDLED;
}

static void
msmsdcc_status_notify_cb(int card_present, void *dev_id)
{
	struct msmsdcc_host *host = dev_id;

	printk(KERN_DEBUG "%s: card_present %d\n", mmc_hostname(host->mmc),
	       card_present);
	msmsdcc_check_status((unsigned long) host);
}

/*
 * called when a command expires.
 * Dump some debugging, and then error
 * out the transaction.
 */
static void
msmsdcc_command_expired(unsigned long _data)
{
	struct msmsdcc_host	*host = (struct msmsdcc_host *) _data;
	struct mmc_request	*mrq;
	unsigned long		flags;
#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
	u32 hres_now = hres_get_counter();
	u32 hres_delta_cmd  = hres_now - host->hres_cmd_start;
#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */
	spin_lock_irqsave(&host->lock, flags);
	mrq = host->mrq;

	if (!mrq) {
		printk(KERN_INFO "%s: Command expiry misfire\n",
		       mmc_hostname(host->mmc));
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	printk(KERN_ERR "%s: Command timeout (%p %p %p %p) opcode %d\n",
	       mmc_hostname(host->mmc), mrq, mrq->cmd,
	       mrq->data, host->dma.sg, mrq->cmd->opcode);

#if LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS
	printk(KERN_ERR"HRES cmd %d (%d usec)\n", 
		hres_delta_cmd , hres_get_delta_usec( host->hres_cmd_start ,hres_now));
#endif /* LOG_MMC_TIMEOUT_TIMING_MEASUREMENTS */

	mrq->cmd->error = -ETIMEDOUT;
	msmsdcc_stop_data(host);

	writel(0, host->base + MMCICOMMAND);

	host->mrq = NULL;
	host->cmd = NULL;

	spin_unlock_irqrestore(&host->lock, flags);
	mmc_request_done(host->mmc, mrq);
}

static int
msmsdcc_init_dma(struct msmsdcc_host *host)
{
	memset(&host->dma, 0, sizeof(struct msmsdcc_dma_data));
	host->dma.host = host;
	host->dma.channel = -1;

	if (!host->dmares)
		return -ENODEV;

	host->dma.nc = dma_alloc_coherent(NULL,
					  sizeof(struct msmsdcc_nc_dmadata),
					  &host->dma.nc_busaddr,
					  GFP_KERNEL);
	if (host->dma.nc == NULL) {
		printk(KERN_ERR "Unable to allocate DMA buffer\n");
		return -ENOMEM;
	}
	memset(host->dma.nc, 0x00, sizeof(struct msmsdcc_nc_dmadata));
	host->dma.cmd_busaddr = host->dma.nc_busaddr;
	host->dma.cmdptr_busaddr = host->dma.nc_busaddr +
				offsetof(struct msmsdcc_nc_dmadata, cmdptr);
	host->dma.channel = host->dmares->start;

	return 0;
}

static int
msmsdcc_probe(struct platform_device *pdev)
{
	struct msm_mmc_platform_data *plat = pdev->dev.platform_data;
	struct msmsdcc_host *host;
	struct mmc_host *mmc;
	struct resource *irqres = NULL;
	struct resource *memres = NULL;
	struct resource *dmares = NULL;
	int ret;
	int i;

	/* must have platform data */
	if (!plat) {
		printk(KERN_ERR "%s: Platform data not available\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	if (pdev->id < 1 || pdev->id > 4)
		return -EINVAL;

	if (pdev->resource == NULL || pdev->num_resources < 2) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	for (i = 0; i < pdev->num_resources; i++) {
		if (pdev->resource[i].flags & IORESOURCE_MEM)
			memres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_IRQ)
			irqres = &pdev->resource[i];
		if (pdev->resource[i].flags & IORESOURCE_DMA)
			dmares = &pdev->resource[i];
	}
	if (!irqres || !memres) {
		printk(KERN_ERR "%s: Invalid resource\n", __func__);
		return -ENXIO;
	}

	/*
	 * Setup our host structure
	 */

	mmc = mmc_alloc_host(sizeof(struct msmsdcc_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	host = mmc_priv(mmc);
	host->pdev_id = pdev->id;
	host->plat = plat;
	host->mmc = mmc;
	host->base = (void __iomem *)memres->start;
	host->irqres = irqres;
	host->memres = memres;
	host->dmares = dmares;
	spin_lock_init(&host->lock);

#ifdef CONFIG_MMC_EMBEDDED_SDIO
	if (plat->embedded_sdio)
		mmc_set_embedded_sdio_data(mmc,
					   &plat->embedded_sdio->cis,
					   &plat->embedded_sdio->cccr,
					   plat->embedded_sdio->funcs,
					   plat->embedded_sdio->num_funcs);
#endif

	/*
	 * Setup DMA
	 */
	msmsdcc_init_dma(host);

	/*
	 * Setup main peripheral bus clock
	 */
	host->pclk = clk_get(&pdev->dev, msmsdcc_pclks[pdev->id]);
	if (IS_ERR(host->pclk)) {
		ret = PTR_ERR(host->pclk);
		goto host_free;
	}

	ret = clk_enable(host->pclk);
	if (ret)
		goto pclk_put;

	host->pclk_rate = clk_get_rate(host->pclk);

	/*
	 * Setup SDC MMC clock
	 */
	host->clk = clk_get(&pdev->dev, msmsdcc_clks[pdev->id]);
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		goto pclk_disable;
	}

	ret = clk_enable(host->clk);
	if (ret)
		goto clk_put;

	ret = clk_set_rate(host->clk, msmsdcc_fmin);
	if (ret) {
		printk(KERN_ERR "%s: Clock rate set failed (%d)\n",
		       __func__, ret);
		goto clk_disable;
	}

	host->clk_rate = clk_get_rate(host->clk);

	host->clks_on = 1;

	/*
	 * Setup MMC host structure
	 */
	mmc->ops = &msmsdcc_ops;
	mmc->f_min = msmsdcc_fmin;
	mmc->f_max = msmsdcc_fmax;
	mmc->ocr_avail = plat->ocr_mask;
	mmc->caps = MMC_CAP_MULTIWRITE | MMC_CAP_MMC_HIGHSPEED;

	if (msmsdcc_4bit)
		mmc->caps |= MMC_CAP_4_BIT_DATA;

	if ( host->plat->get_detection_on_start_mode && 
             host->plat->get_detection_on_start_mode(mmc_dev(mmc))) {
		mmc->caps |= MMC_CAP_NO_DETECTION_ON_START;
	}

	mmc->max_phys_segs = NR_SG;
	mmc->max_hw_segs = NR_SG;
	mmc->max_blk_size = 4096;	/* MCI_DATA_CTL BLOCKSIZE up to 4096 */
	mmc->max_blk_count = 65536;

	mmc->max_req_size = 33554432;	/* MCI_DATA_LENGTH is 25 bits */
	mmc->max_seg_size = mmc->max_req_size;
		
	writel(0, host->base + MMCIMASK0);
	writel(0x5c007ff, host->base + MMCICLEAR);

	writel(MCI_IRQENABLE, host->base + MMCIMASK0);

	/*
	 * Setup card detect change
	 */

	memset(&host->timer, 0, sizeof(host->timer));

	if (plat->status_irq) {
		ret = request_irq(plat->status_irq,
				  msmsdcc_platform_status_irq,
				  IRQF_SHARED,
				  DRIVER_NAME " (slot)",
				  host);
		if (ret) {
			printk(KERN_ERR "Unable to get slot IRQ %d (%d)\n",
			       plat->status_irq, ret);
			goto clk_disable;
		}
	} else if (plat->register_status_notify) {
		plat->register_status_notify(msmsdcc_status_notify_cb, host);
	} else if (!plat->status)
		printk(KERN_ERR "%s: No card detect facilities available\n",
		       mmc_hostname(mmc));
	else {
		init_timer(&host->timer);
		host->timer.data = (unsigned long)host;
		host->timer.function = msmsdcc_check_status;
		host->timer.expires = jiffies + HZ;
		add_timer(&host->timer);
	}

	if (plat->status) {
		host->oldstat = host->plat->status(mmc_dev(host->mmc));
		host->eject = !host->oldstat;
	}

	/*
	 * Setup data end wq
	 */
	snprintf(host->data_end_wq.name, 
					sizeof(host->data_end_wq.name),
					"msmsdcc_data_end_wq%d", 
					pdev->id);
	host->data_end_wq.p_worker = 
				create_singlethread_workqueue(host->data_end_wq.name);

	INIT_WORK(&(host->data_end_wq.worker), __msmsdcc_data_end_worker );

	/*
	 * Setup a command timer. We currently need this due to
	 * some 'strange' timeout / error handling situations.
	 */
	init_timer(&host->command_timer);
	host->command_timer.data = (unsigned long) host;
	host->command_timer.function = msmsdcc_command_expired;

	ret = request_irq(irqres->start, msmsdcc_irq, IRQF_SHARED,
			  DRIVER_NAME " (cmd)", host);
	if (ret)
		goto platform_irq_free;

	mmc_set_drvdata(pdev, mmc);
	mmc_add_host(mmc);

	printk(KERN_INFO
	       "%s: Qualcomm MSM SDCC at 0x%016llx irq %d,%d dma %d\n",
	       mmc_hostname(mmc), (unsigned long long)memres->start,
	       (unsigned int) irqres->start,
	       (unsigned int) plat->status_irq, host->dma.channel);
	printk(KERN_INFO "%s: 4 bit data mode %s\n", mmc_hostname(mmc),
	       (mmc->caps & MMC_CAP_4_BIT_DATA ? "enabled" : "disabled"));
	printk(KERN_INFO "%s: MMC clock %u -> %u Hz, PCLK %u Hz\n",
	       mmc_hostname(mmc), msmsdcc_fmin, msmsdcc_fmax, host->pclk_rate);
	printk(KERN_INFO "%s: Slot eject status = %d\n", mmc_hostname(mmc),
	       host->eject);
	printk(KERN_INFO "%s: Power save feature enable = %d\n",
	       mmc_hostname(mmc), msmsdcc_pwrsave);

	if (host->dma.channel != -1) {
		printk(KERN_INFO
		       "%s: DM non-cached buffer at %p, dma_addr 0x%.8x\n",
		       mmc_hostname(mmc), host->dma.nc, host->dma.nc_busaddr);
		printk(KERN_INFO
		       "%s: DM cmd busaddr %u, cmdptr busaddr %u\n",
		       mmc_hostname(mmc), host->dma.cmd_busaddr,
		       host->dma.cmdptr_busaddr);
	} else
		printk(KERN_INFO
		       "%s: PIO transfer enabled\n", mmc_hostname(mmc));
	if (host->timer.function)
		printk(KERN_INFO "%s: Polling status mode enabled\n",
		       mmc_hostname(mmc));

	if(plat->board_probe)
	{
		if(plat->board_probe(host->mmc)) 
		{
			printk("msmsdcc_probe: board_probe failed!\n");
			goto platform_irq_free;
		}
	}

#if defined(CONFIG_DEBUG_FS)
	msmsdcc_dbg_createhost(host);
#endif
	return 0;

 platform_irq_free:
	if (plat->status_irq)
		free_irq(plat->status_irq, host);
 clk_disable:
	clk_disable(host->clk);
 clk_put:
	clk_put(host->clk);
 pclk_disable:
	clk_disable(host->pclk);
 pclk_put:
	clk_put(host->pclk);
 host_free:
	mmc_free_host(mmc);
 out:
	return ret;
}

static int
msmsdcc_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = mmc_get_drvdata(dev);
	int rc = 0;

	if (mmc) {
		struct msmsdcc_host *host = mmc_priv(mmc);

		MSMSDCC_LOG(host, MSMSDCC_SUSPEND_EVENT, 0, 0 );

		if (host->plat->status_irq)
			disable_irq(host->plat->status_irq);
		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO)
			rc = mmc_suspend_host(mmc, state);

		MSMSDCC_LOG(host, MSMSDCC_SUSPEND_EVENT, 1, rc );

		if (!rc) {
			writel(0, host->base + MMCIMASK0);

			if (host->clks_on) {
				clk_disable(host->clk);
				clk_disable(host->pclk);
				host->clks_on = 0;
			}
		}
	}
	return rc;
}

static int
msmsdcc_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = mmc_get_drvdata(dev);
	int rc = 0;

	if (mmc) {
		struct msmsdcc_host *host = mmc_priv(mmc);

		MSMSDCC_LOG(host, MSMSDCC_RESUME_EVENT, 0, 0 );

		if (!host->clks_on) {
			clk_enable(host->pclk);
			clk_enable(host->clk);
			host->clks_on = 1;
		}

		writel(MCI_IRQENABLE, host->base + MMCIMASK0);
		if (mmc->card && mmc->card->type != MMC_TYPE_SDIO)
			rc = mmc_resume_host(mmc);

		if (host->plat->status_irq)
			enable_irq(host->plat->status_irq);

		MSMSDCC_LOG(host, MSMSDCC_RESUME_EVENT, 1, rc );
	}
	return rc;
}

static struct platform_driver msmsdcc_driver = {
	.probe		= msmsdcc_probe,
	.suspend	= msmsdcc_suspend,
	.resume		= msmsdcc_resume,
	.driver		= {
		.name	= "msm_sdcc",
	},
};

static int __init msmsdcc_init(void)
{
	return platform_driver_register(&msmsdcc_driver);
}

static void __exit msmsdcc_exit(void)
{
	platform_driver_unregister(&msmsdcc_driver);
}

static int __init msmsdcc_pwrsave_setup(char *__unused)
{
	msmsdcc_pwrsave = 1;
	return 1;
}

static int __init msmsdcc_nopwrsave_setup(char *__unused)
{
	msmsdcc_pwrsave = 0;
	return 1;
}

static int __init msmsdcc_4bit_setup(char *__unused)
{
	msmsdcc_4bit = 1;
	return 1;
}

static int __init msmsdcc_1bit_setup(char *__unused)
{
	msmsdcc_4bit = 0;
	return 1;
}

static int __init msmsdcc_fmin_setup(char *str)
{
	unsigned int n;

	if (!get_option(&str, &n))
		return 0;
	msmsdcc_fmin = n;
	return 1;
}

static int __init msmsdcc_fmax_setup(char *str)
{
	unsigned int n;

	if (!get_option(&str, &n))
		return 0;
	msmsdcc_fmax = n;
	return 1;
}

__setup("msmsdcc_4bit", msmsdcc_4bit_setup);
__setup("msmsdcc_1bit", msmsdcc_1bit_setup);
__setup("msmsdcc_pwrsave", msmsdcc_pwrsave_setup);
__setup("msmsdcc_nopwrsave", msmsdcc_nopwrsave_setup);
__setup("msmsdcc_fmin=", msmsdcc_fmin_setup);
__setup("msmsdcc_fmax=", msmsdcc_fmax_setup);

module_init(msmsdcc_init);
module_exit(msmsdcc_exit);
module_param(msmsdcc_fmin, uint, 0444);
module_param(msmsdcc_fmax, uint, 0444);
module_param(msmsdcc_4bit, uint, 0444);

MODULE_DESCRIPTION("Qualcomm MSM 7X00A Multimedia Card Interface driver");
MODULE_LICENSE("GPL");

#if defined(CONFIG_DEBUG_FS)

static int 
msmsdcc_dbg_state_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t
msmsdcc_dbg_state_read(struct file *file, char __user *ubuf,
		       size_t count, loff_t *ppos)
{
	struct msmsdcc_host *host = (struct msmsdcc_host *) file->private_data;
	char buf[1024];
	int max, i;

	i = 0;
	max = sizeof(buf) -1;

	i += scnprintf(buf + i, max - i, "STAT: %p %p %p\n", host->mrq, host->cmd,
		       host->data);
	if (host->cmd) {
		struct mmc_command *cmd = host->cmd;

		i+= scnprintf(buf + i, max - i, "CMD : %.8x %.8x %.8x\n",
			      cmd->opcode, cmd->arg, cmd->flags);
	}
	if (host->data) {
		struct mmc_data *data = host->data;
		i+= scnprintf(buf + i, max - i, "DAT0: %.8x %.8x %.8x %.8x %.8x %.8x\n",
			      data->timeout_ns, data->timeout_clks,
			      data->blksz, data->blocks, data->error,
			      data->flags);
		i+= scnprintf(buf + i, max - i, "DAT1: %.8x %.8x %.8x %p\n",
			      host->xfer_size, host->xfer_remain,
			      host->data_xfered, host->dma.sg);
	}

	return simple_read_from_buffer(ubuf, count, ppos, buf, i);
}

static const struct file_operations msmsdcc_dbg_state_ops = {
	.read	= msmsdcc_dbg_state_read,
	.open	= msmsdcc_dbg_state_open,
};

static void msmsdcc_dbg_createhost(struct msmsdcc_host *host)
{
	if (debugfs_dir) {
		debugfs_create_file(mmc_hostname(host->mmc), 0644, debugfs_dir,
				    host, &msmsdcc_dbg_state_ops);
	}
}

static int __init msmsdcc_dbg_init(void)
{
	int err;

	debugfs_dir = debugfs_create_dir("msmsdcc", 0);
	if (IS_ERR(debugfs_dir)) {
		err = PTR_ERR(debugfs_dir);
		debugfs_dir = NULL;
		return err;
	}

	return 0;
}

device_initcall(msmsdcc_dbg_init);
#endif
