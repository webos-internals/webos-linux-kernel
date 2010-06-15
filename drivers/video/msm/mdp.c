/* drivers/video/msm_fb/mdp.c
 *
 * MSM MDP Interface (used by framebuffer core)
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

#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/msm_fb.h>
#include <../arch/arm/mach-msm/clock.h>

#include "mdp_hw.h"

extern void mdp_hw_init(void);

/* setup color conversion coefficients */
void mdp_set_ccs(uint16_t *ccs)
{
	int n;
	for (n = 0; n < 9; n++)
		writel(ccs[n], MSM_MDP_BASE + 0x40440 + 4 * n);
	writel(ccs[9], MSM_MDP_BASE + 0x40500 + 4 * 0);
	writel(ccs[10], MSM_MDP_BASE + 0x40500 + 4 * 0);
	writel(ccs[11], MSM_MDP_BASE + 0x40500 + 4 * 0);
}

static DECLARE_WAIT_QUEUE_HEAD(mdp_dma2_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(mdp_ppp_waitqueue);
static struct msmfb_callback *dma_callback;
static struct clk *clk;
static unsigned int mdp_irq_mask;
static DEFINE_SPINLOCK(mdp_lock);

int enable_mdp_irq(uint32_t mask)
{
	unsigned long irq_flags;
	int ret = 0;

	BUG_ON(!mask);

	spin_lock_irqsave(&mdp_lock, irq_flags);
	/* if the mask bits are already set return an error, this interrupt
	 * is already enabled */
	if (mdp_irq_mask & mask) {
		//printk(KERN_ERR "mdp irq already on already on %x %x\n",
		//       mdp_irq_mask, mask);
		ret = -1;
	}
	/* if the mdp irq is not already enabled enable it */
	if (!mdp_irq_mask) {
		if (clk)
			clk_enable(clk);
		enable_irq(INT_MDP);
	}

	/* update the irq mask to reflect the fact that the interrupt is
	 * enabled */
	mdp_irq_mask |= mask;
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return ret;
}

static int locked_disable_mdp_irq(uint32_t mask)
{
	/* this interrupt is already disabled! */
	if (!(mdp_irq_mask & mask)) {
		printk(KERN_ERR "mdp irq already off %x %x\n",
		       mdp_irq_mask, mask);
		return -1;
	}
	/* update the irq mask to reflect the fact that the interrupt is
	 * disabled */
	mdp_irq_mask &= ~(mask);
	/* if no one is waiting on the interrupt, disable it */
	if (!mdp_irq_mask) {
		disable_irq(INT_MDP);
		if (clk)
			clk_disable(clk);
	}
	return 0;
}

int disable_mdp_irq(uint32_t mask)
{
	unsigned long irq_flags;
	int ret;

	spin_lock_irqsave(&mdp_lock, irq_flags);
	ret = locked_disable_mdp_irq(mask);
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return ret;
}

static uint32_t last_isr_status;
static ktime_t last_isr_time;
void mdp_print_isr_status(void)
{
	uint64_t tmp;

	tmp = ktime_to_ns(last_isr_time);
	do_div(tmp, 1000000);
	printk(KERN_ERR"%s: last_isr_time: %llu ms\n", __func__,tmp);
	tmp = ktime_to_ns(ktime_get());
	do_div(tmp, 1000000);
	printk(KERN_ERR"%s: now: %llu ms\n", __func__,tmp);
	printk(KERN_ERR"%s: mdp_irq_mask: %d\n", __func__, mdp_irq_mask);
	printk(KERN_ERR"%s: last_isr_status: %d\n", __func__,last_isr_status);
	if(clk)
		printk(KERN_ERR"%s: clk: %d\n", __func__, clk->count);
	if(dma_callback)
		printk(KERN_ERR"%s: dma_callback exists\n", __func__ );
}


static irqreturn_t mdp_isr(int irq, void *data)
{
	uint32_t status;
	unsigned long irq_flags;

	last_isr_time = ktime_get();

	spin_lock_irqsave(&mdp_lock, irq_flags);

	status = readl(MDP_INTR_STATUS);
	writel(status, MDP_INTR_CLEAR);

	status &= mdp_irq_mask;
	if (status & DL0_DMA2_TERM_DONE) {
		if (dma_callback && dma_callback->func) {
			dma_callback->func(dma_callback);
			dma_callback = NULL;
		}
		wake_up(&mdp_dma2_waitqueue);
	}

	if (status & DL0_ROI_DONE)
		wake_up(&mdp_ppp_waitqueue);

	if (status)
		locked_disable_mdp_irq(status);

	last_isr_status = status;

	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return IRQ_HANDLED;
}

uint32_t mdp_check_mask(uint32_t mask)
{
	uint32_t ret;
	unsigned long irq_flags;

	spin_lock_irqsave(&mdp_lock, irq_flags);
	ret = mdp_irq_mask & mask;
	spin_unlock_irqrestore(&mdp_lock, irq_flags);
	return ret;
}

static int mdp_wait(uint32_t mask, wait_queue_head_t *wq)
{
	int ret = 0;
	unsigned int irq_flags;

	wait_event_timeout(*wq, !mdp_check_mask(mask), HZ);

	spin_lock_irqsave(&mdp_lock, irq_flags);
	if (mdp_irq_mask & mask) {
		locked_disable_mdp_irq(mask);
		printk(KERN_WARNING "timeout waiting for mdp to complete %x\n",
		       mask);
		ret = -ETIMEDOUT;
	}
	spin_unlock_irqrestore(&mdp_lock, irq_flags);

	return ret;
}

void mdp_dma_wait(void)
{
	mdp_wait(DL0_DMA2_TERM_DONE, &mdp_dma2_waitqueue);
}

int mdp_ppp_wait(void)
{
	return mdp_wait(DL0_ROI_DONE, &mdp_ppp_waitqueue);
}


void mdp_dma_to_mddi(uint32_t addr, uint32_t stride, uint32_t width,
		     uint32_t height, uint32_t x, uint32_t y,
		     struct msmfb_callback *callback, uint32_t dma2_ibufformat, bool blocking)

{

	uint32_t dma2_cfg;
	uint16_t ld_param = 0; /* 0=PRIM, 1=SECD, 2=EXT */

	if (enable_mdp_irq(DL0_DMA2_TERM_DONE)) {
		//printk(KERN_ERR "mdp_dma_to_mddi: busy\n");
		return;
	}

	dma_callback = callback;

	dma2_cfg = DMA_PACK_TIGHT |
		DMA_PACK_ALIGN_LSB |
		DMA_PACK_PATTERN_RGB |
		DMA_OUT_SEL_AHB |
		DMA_IBUF_NONCONTIGUOUS;

	dma2_cfg |= dma2_ibufformat;
			
	dma2_cfg |= DMA_OUT_SEL_MDDI;

	dma2_cfg |= DMA_MDDI_DMAOUT_LCD_SEL_PRIMARY;

	dma2_cfg |= DMA_DITHER_EN;

	/* setup size, address, and stride */
        writel((height << 16) | (width), MDP_FULL_BYPASS_WORD33);
        writel(addr, MDP_FULL_BYPASS_WORD34);
        writel(stride, MDP_FULL_BYPASS_WORD35);

	/* 666 18BPP */
	dma2_cfg |= DMA_DSTC0G_6BITS | DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;

	/* set y & x offset and MDDI transaction parameters */
        writel((y << 16) | (x), MDP_FULL_BYPASS_WORD37);
        writel(ld_param, MDP_FULL_BYPASS_WORD40);
	writel((MDDI_VDO_PACKET_DESC << 16) | MDDI_VDO_PACKET_PRIM,
		   MDP_FULL_BYPASS_WORD41);

	writel(dma2_cfg, MDP_DMA_CONFIG);

	/* start DMA2 */
	writel(0, MDP_DMA_START);

	if(blocking) {
		mdp_dma_wait();
	}
}

void mdp_set_grp_disp(unsigned disp_id)
{
	disp_id &= 0xf;
	writel(disp_id, MDP_FULL_BYPASS_WORD43);
}

#include "mdp_scale_tables.h"

int mdp_init(struct fb_info *info)
{
	int ret;
	int n;
#if !defined(CONFIG_MSM7X00A_6056_COMPAT)

	clk = clk_get(0, "mdp_clk");
#if 0
	if (clk)
		clk_enable(clk);
#endif
#endif

	ret = request_irq(INT_MDP, mdp_isr, IRQF_DISABLED, "msm_mdp", NULL);
	disable_irq(INT_MDP);
	mdp_irq_mask = 0;

	mdp_hw_init();

	for (n = 0; n < ARRAY_SIZE(mdp_upscale_table); n++)
		writel(mdp_upscale_table[n].val, mdp_upscale_table[n].reg);

	return 0;
}


