/*
 * drivers/media/video/omap/isp/isph3a.c
 *
 * H3A module for TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>

#include "isp.h"
#include "ispreg.h"
#include "isph3a.h"
#include "ispmmu.h"
#include "isppreview.h"


struct isph3a_aewb_buffer {
	unsigned long virt_addr;
	unsigned long phy_addr;
	unsigned long addr_align;
	unsigned long ispmmu_addr;
	unsigned long mmap_addr;        /* For userspace */
	
	u8 locked;
	u16 frame_num;
	struct isph3a_aewb_buffer *next;
};

static struct isph3a_aewb_status {
	u8 initialized;
	u8 update;
	u8 stats_req;
	u8 stats_done;
	u16 frame_req;
	
	struct isph3a_aewb_buffer h3a_buff[H3A_MAX_BUFF];
	unsigned int stats_buf_size;
	unsigned int min_buf_size;

	u16 win_count;
	u32 frame_count;
	wait_queue_head_t stats_wait;
	spinlock_t buffer_lock;
} aewbstat;

static struct isph3a_aewb_regs {		
	u32 reg_pcr;
	u32 reg_win1;
	u32 reg_start;
	u32 reg_blk;
	u32 reg_subwin;
} aewb_regs;

static struct isph3a_aewb_config aewb_config_local = {
	.saturation_limit = 	0x3FF,
	.win_height = 		0,	/* Range: 2 - 256 even values only */
	.win_width = 		0,	/* Range: 6 - 256 even values only */
	.ver_win_count =	0,	/* Range: 1 - 128 */
	.hor_win_count = 	0,	/* Range: 1 - 36 */
	.ver_win_start = 	0,	/* Range: 0 - 4095 */
	.hor_win_start = 	0,	/* Range: 0 - 4095 */
	.blk_ver_win_start = 	0,	/* Range: 0 - 4095 */
	.blk_win_height = 	0,	/* Range: 2 - 256 even values only */
	.subsample_ver_inc = 	0,	/* Range: 2 - 32 even values only */
	.subsample_hor_inc = 	0,	/* Range: 2 - 32 even values only */
	.alaw_enable = 		0,	/* AEW ALAW EN flag */
	.aewb_enable = 		0,	/* AE AWB stats generation EN flag */ 
}; /* With reset values */


/* Structure for saving/restoring h3a module registers*/
static struct isp_reg isph3a_reg_list[]= {
	{ISPH3A_AEWWIN1, 0x0000},
	{ISPH3A_AEWINSTART, 0x0000},
	{ISPH3A_AEWINBLK, 0x0000},
	{ISPH3A_AEWSUBWIN, 0x0000},
	{ISPH3A_AEWBUFST, 0x0000},
	{ISPH3A_AFPAX1, 0x0000},
	{ISPH3A_AFPAX2, 0x0000}, 
	{ISPH3A_AFPAXSTART, 0x0000},
	{ISPH3A_AFIIRSH, 0x0000},
	{ISPH3A_AFBUFST, 0x0000},
	{ISPH3A_AFCOEF010, 0x0000},
	{ISPH3A_AFCOEF032, 0x0000},
	{ISPH3A_AFCOEF054, 0x0000},
	{ISPH3A_AFCOEF076, 0x0000},		
	{ISPH3A_AFCOEF098, 0x0000},
	{ISPH3A_AFCOEF0010, 0x0000},
	{ISPH3A_AFCOEF110, 0x0000},
	{ISPH3A_AFCOEF132, 0x0000},
	{ISPH3A_AFCOEF154, 0x0000},
	{ISPH3A_AFCOEF176, 0x0000},
	{ISPH3A_AFCOEF198, 0x0000},
	{ISPH3A_AFCOEF1010, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};
  

extern int omap34xxcam_set_exposure_time(int mode, u32 exp_time);
extern int omap34xxcam_set_gain(u16 gain);
static struct ispprev_wbal h3awb_update; /* Keep changes in AEWB gains */
static struct isph3a_aewb_buffer *active_buff = NULL;
static int camnotify;
static int wb_update = 0;
static void isph3a_print_status(void);

/*
 * Enables AEW engine in the H3A module.
 * Client should configure all the AE & AWB registers in H3A before this.
 * enable		: 1- Enables the AE & AWB engine.
 */
static void
isph3a_aewb_enable(u8 enable)
{
	/* Before enabling AEWB we need to clear H3A bit in IRQ0 status reg */
	omap_writel(IRQ0STATUS_H3A_AWB_DONE_IRQ, ISP_IRQ0STATUS);

	if (enable) {
		aewb_regs.reg_pcr |= ISPH3A_PCR_AEW_EN;
		omap_writel(omap_readl(ISPH3A_PCR) | (ISPH3A_PCR_AEW_EN),
			ISPH3A_PCR);
		DPRINTK_ISPH3A("    h3a enabled \n");
	} else {
		aewb_regs.reg_pcr &= ~ISPH3A_PCR_AEW_EN;
		omap_writel(omap_readl(ISPH3A_PCR) & ~(ISPH3A_PCR_AEW_EN),
			ISPH3A_PCR);
		DPRINTK_ISPH3A("    h3a disabled \n");
	}
	aewb_config_local.aewb_enable = enable;
}

/*
 * Updates WB parameters. Needs to be called when no ISP Preview processing is
 * taking place. 
 */  
void
isph3a_update_wb(void)
{
	if (wb_update) {
		isppreview_config_whitebalance(h3awb_update);
		wb_update = 0;
	}
	return;
}

/*
 * Helper function to update h3a registers
 */
static void
isph3a_aewb_update_regs(void)
{
	omap_writel(aewb_regs.reg_pcr, ISPH3A_PCR);
	omap_writel(aewb_regs.reg_win1, ISPH3A_AEWWIN1);
	omap_writel(aewb_regs.reg_start, ISPH3A_AEWINSTART);
	omap_writel(aewb_regs.reg_blk, ISPH3A_AEWINBLK);
	omap_writel(aewb_regs.reg_subwin, ISPH3A_AEWSUBWIN);

	aewbstat.update = 0;
	aewbstat.frame_count = 0;
}

/*
 * Helper function to update buffer cache pages
 */ 
static void
isph3a_aewb_update_req_buffer(struct isph3a_aewb_buffer *buffer)
{
	int size = aewbstat.stats_buf_size;

	size = PAGE_ALIGN(size);
	/* Update the kernel pages of the requested buffer */
	dmac_inv_range((void *)buffer->addr_align, (void *)buffer->addr_align + size);
}

/*
 * Helper function to check for stats available of specified frame
 * Returns 0 if stats available for frame requested; -1 otherwise.
 */
static int
isph3a_aewb_stats_available(struct isph3a_aewb_data *aewbdata)
{
	int i;
	unsigned long irqflags;
	
	spin_lock_irqsave(&aewbstat.buffer_lock, irqflags);
	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if ((aewbdata->frame_number == aewbstat.h3a_buff[i].frame_num)
			&& (aewbstat.h3a_buff[i].frame_num !=
				active_buff->frame_num)) {
			aewbstat.h3a_buff[i].locked = 1;
			spin_unlock_irqrestore(&aewbstat.buffer_lock, irqflags);
			isph3a_aewb_update_req_buffer(&aewbstat.h3a_buff[i]);
			aewbstat.h3a_buff[i].frame_num = 0;
			aewbdata->h3a_aewb_statistics_buf = (void *)
				aewbstat.h3a_buff[i].mmap_addr;
			return 0;
		}
	}
	spin_unlock_irqrestore(&aewbstat.buffer_lock, irqflags);
	/* Stats unavailable */
  
	aewbdata->h3a_aewb_statistics_buf = NULL;
	return -1;
}

/*
 * Helper function to link allocated buffers
 */
static void
isph3a_aewb_link_buffers(void)
{
	int i;

	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if ((i + 1) < H3A_MAX_BUFF)
			aewbstat.h3a_buff[i].next = &aewbstat.h3a_buff[i + 1];
		else
			aewbstat.h3a_buff[i].next = &aewbstat.h3a_buff[0];
	}
}

/*
 * Helper function to unlock all buffers
 */
static void
isph3a_aewb_unlock_buffers(void)
{
	int i;
	unsigned long irqflags;

	spin_lock_irqsave(&aewbstat.buffer_lock, irqflags);
	for (i = 0; i < H3A_MAX_BUFF; i++) {
		aewbstat.h3a_buff[i].locked = 0;
	}
	spin_unlock_irqrestore(&aewbstat.buffer_lock, irqflags);
}

/*
 * Callback from ISP driver for H3A AEW interrupt
 * status 	: IRQ0STATUS in case of MMU error, 0 for h3a interrupt
 * arg1		: Not used as of now.
 * arg2		: Not used as of now.
 */
static void
isph3a_aewb_isr(unsigned long status, void *arg1, void *arg2)
{
	u16 frame_align;

	if ((H3A_AWB_DONE & status) != H3A_AWB_DONE)
		return;

	/* Exchange buffers */
	active_buff = active_buff->next;
	if (active_buff->locked == 1)
		active_buff = active_buff->next;
	omap_writel(active_buff->ispmmu_addr, ISPH3A_AEWBUFST);

	/* Update frame counter */
	aewbstat.frame_count++;
	frame_align = aewbstat.frame_count;
	if (aewbstat.frame_count > MAX_FRAME_COUNT) {
		aewbstat.frame_count = 1;
		frame_align++;
	}
	active_buff->frame_num = aewbstat.frame_count;

	/* Future Stats requested? */
	if (aewbstat.stats_req) {
		/* Is the frame we want already done? */
		DPRINTK_ISPH3A("waiting for frame %d\n", aewbstat.frame_req);
		if (frame_align >= (aewbstat.frame_req + 1)){
			aewbstat.stats_req = 0;
			aewbstat.stats_done = 1;
			wake_up_interruptible(&aewbstat.stats_wait);
		}
	}

	if (aewbstat.update) {
		isph3a_aewb_update_regs();
	}
	DPRINTK_ISPH3A(".");
}

/*
 * Helper function to check and store user given params.
 * As most of them are busy-lock registers, need to wait
 * until AEW_BUSY = 0 --> to program them during ISR.
 */
static int
isph3a_aewb_set_params(struct isph3a_aewb_config *user_cfg)
{
	/* Saturation limit */
	if(unlikely(user_cfg->saturation_limit > MAX_SATURATION_LIM)) {
		printk("Invalid Saturation_limit: %d\n",
			user_cfg->saturation_limit);
		return -EINVAL;
	} else if (aewb_config_local.saturation_limit !=
						user_cfg->saturation_limit) {
		WRITE_SAT_LIM(aewb_regs.reg_pcr, user_cfg->saturation_limit);
		aewb_config_local.saturation_limit =
						user_cfg->saturation_limit;		
		aewbstat.update = 1;
	}
	/* A-Law */
	if (aewb_config_local.alaw_enable != user_cfg->alaw_enable) {
		WRITE_ALAW(aewb_regs.reg_pcr, user_cfg->alaw_enable);
		aewb_config_local.alaw_enable = user_cfg->alaw_enable;
		aewbstat.update = 1;
	}
	/* Window height */
	if (unlikely((user_cfg->win_height < MIN_WIN_H)
			|| (user_cfg->win_height > MAX_WIN_H)
			|| (user_cfg->win_height & 0x01))){
		printk("Invalid window height: %d\n", user_cfg->win_height);
		return -EINVAL;
	} else if (aewb_config_local.win_height != user_cfg->win_height) {
		WRITE_WIN_H(aewb_regs.reg_win1, user_cfg->win_height);
		aewb_config_local.win_height = user_cfg->win_height;
		aewbstat.update = 1;
	}
	/* Window width */
	if (unlikely((user_cfg->win_width < MIN_WIN_W)
			|| (user_cfg->win_width > MAX_WIN_W)
			|| (user_cfg->win_width & 0x01))){
		printk("Invalid window width: %d\n", user_cfg->win_width);
		return -EINVAL;
	} else if (aewb_config_local.win_width != user_cfg->win_width) {
		WRITE_WIN_W(aewb_regs.reg_win1, user_cfg->win_width);
		aewb_config_local.win_width = user_cfg->win_width;
		aewbstat.update = 1;
	}
	/* Vertical window count */
	if (unlikely((user_cfg->ver_win_count < 1)
			|| (user_cfg->ver_win_count > MAX_WINVC))) {
		printk("Invalid vertical window count: %d\n",
			user_cfg->ver_win_count);
		return -EINVAL;		
	} else if (aewb_config_local.ver_win_count
				!= user_cfg->ver_win_count){
		WRITE_VER_C(aewb_regs.reg_win1,
					user_cfg->ver_win_count);
		aewb_config_local.ver_win_count	=
					user_cfg->ver_win_count;
		aewbstat.update = 1;
	}
	/* Horizontal window count */
	if (unlikely((user_cfg->hor_win_count < 1)
			|| (user_cfg->hor_win_count > MAX_WINHC))) {
		printk("Invalid horizontal window count: %d\n",
			user_cfg->hor_win_count);
		return -EINVAL;		
	} else if (aewb_config_local.hor_win_count
				!= user_cfg->hor_win_count){
		WRITE_HOR_C(aewb_regs.reg_win1,
					user_cfg->hor_win_count);
		aewb_config_local.hor_win_count	=
					user_cfg->hor_win_count;
		aewbstat.update = 1;
	}
	/* Windows vertical start position */
	if (unlikely(user_cfg->ver_win_start > MAX_WINSTART)) {
		printk("Invalid vertical window start: %d\n",
			user_cfg->ver_win_start);
		return -EINVAL;			
	} else if (aewb_config_local.ver_win_start
				!= user_cfg->ver_win_start){
		WRITE_VER_WIN_ST(aewb_regs.reg_start, 
					user_cfg->ver_win_start);
		aewb_config_local.ver_win_start	=
					user_cfg->ver_win_start;
		aewbstat.update = 1;			
	}
	/* Windows horizontal start position */
	if (unlikely(user_cfg->hor_win_start > MAX_WINSTART)) {
		printk("Invalid horizontal window start: %d\n",
			user_cfg->hor_win_start);
		return -EINVAL;			
	} else if (aewb_config_local.hor_win_start
				!= user_cfg->hor_win_start){
		WRITE_HOR_WIN_ST(aewb_regs.reg_start, 
					 user_cfg->hor_win_start);
		aewb_config_local.hor_win_start	=
					user_cfg->hor_win_start;
		aewbstat.update = 1;
	}
	/* Black Line vertical start position */
	if (unlikely(user_cfg->blk_ver_win_start > MAX_WINSTART)) {
		printk("Invalid black vertical window start: %d\n",
			user_cfg->blk_ver_win_start);
		return -EINVAL;			
	} else if (aewb_config_local.blk_ver_win_start
				!= user_cfg->blk_ver_win_start){
		WRITE_BLK_VER_WIN_ST(aewb_regs.reg_blk, 
					user_cfg->blk_ver_win_start);
		aewb_config_local.blk_ver_win_start =
					user_cfg->blk_ver_win_start;
		aewbstat.update = 1;
	}
	/* Black line height */
	if (unlikely((user_cfg->blk_win_height < MIN_WIN_H)
			|| (user_cfg->blk_win_height > MAX_WIN_H)
			|| (user_cfg->blk_win_height & 0x01))){
		printk("Invalid black window height: %d\n",
			user_cfg->blk_win_height);
		return -EINVAL;
	} else if (aewb_config_local.blk_win_height
				!= user_cfg->blk_win_height) {
		WRITE_BLK_WIN_H(aewb_regs.reg_blk,
				user_cfg->blk_win_height);
		aewb_config_local.blk_win_height
				= user_cfg->blk_win_height;
		aewbstat.update = 1;
	}
	/* Vertical sampling point increments */
	if (unlikely((user_cfg->subsample_ver_inc < MIN_SUB_INC)
			|| (user_cfg->subsample_ver_inc > MAX_SUB_INC)
			|| (user_cfg->subsample_ver_inc & 0x01))) {
		printk("Invalid vertical subsample increment: %d\n",
			user_cfg->subsample_ver_inc);
		return -EINVAL;	
	} else if (aewb_config_local.subsample_ver_inc
				!= user_cfg->subsample_ver_inc)  {
		WRITE_SUB_VER_INC(aewb_regs.reg_subwin,
				  user_cfg->subsample_ver_inc);
		aewb_config_local.subsample_ver_inc
					= user_cfg->subsample_ver_inc;
		aewbstat.update = 1;
	}
	/* Horizontal sampling point increments */	
	if (unlikely((user_cfg->subsample_hor_inc < MIN_SUB_INC)
			|| (user_cfg->subsample_hor_inc > MAX_SUB_INC)
			|| (user_cfg->subsample_hor_inc & 0x01))) {
		printk("Invalid horizontal subsample increment: %d\n",
			user_cfg->subsample_hor_inc);
		return -EINVAL;	
	} else if (aewb_config_local.subsample_hor_inc
				!= user_cfg->subsample_hor_inc)  {
		WRITE_SUB_HOR_INC(aewb_regs.reg_subwin,
				  user_cfg->subsample_hor_inc);
		aewb_config_local.subsample_hor_inc
					= user_cfg->subsample_hor_inc;
		aewbstat.update = 1;
	}

	if ((!aewbstat.initialized) || (0 == aewb_config_local.aewb_enable)) {
		isph3a_aewb_update_regs();
		aewbstat.initialized = 1;
	}
	return 0;
}

/*
 * Helper function to munmap kernel buffers from user space.
 */ 
static int
isph3a_aewb_munmap(struct isph3a_aewb_buffer *buffer)
{
	/* TO DO: munmap succesfully the kernel buffers, so they can be 
	   remmaped again */
	buffer->mmap_addr = 0;
	return 0;
}

/*
 * Helper function to mmap buffers to user space. 
 * buffer passed need to already have a valid physical address: buffer->phy_addr
 * It returns user pointer as unsigned long in buffer->mmap_addr
 */
static int
isph3a_aewb_mmap_buffers(struct isph3a_aewb_buffer *buffer)
{
	struct vm_area_struct vma;
	struct mm_struct *mm = current->mm;
	int size = aewbstat.stats_buf_size;
	unsigned long addr = 0;
	unsigned long pgoff = 0, flags = MAP_SHARED | MAP_ANONYMOUS;
	unsigned long prot = PROT_READ | PROT_WRITE;
	void * pos = (void *) buffer->addr_align;

	size = PAGE_ALIGN(size);

	addr = get_unmapped_area(NULL, addr, size, pgoff, flags);
	vma.vm_mm = mm;
	vma.vm_start = addr;
	vma.vm_end = addr + size;
	vma.vm_flags = calc_vm_prot_bits(prot) | calc_vm_flag_bits(flags);
	vma.vm_pgoff = pgoff;
	vma.vm_file = NULL;
	vma.vm_page_prot = protection_map[vma.vm_flags];

	while (size > 0) {
		if (vm_insert_page(&vma, addr, vmalloc_to_page(pos)))
			return -EAGAIN;
		addr += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	buffer->mmap_addr = vma.vm_start;
	return 0;
}

/*
 * API to configure AEW registers and enable/disable H3A engine
 */
int
isph3a_aewb_configure(struct isph3a_aewb_config *aewbcfg)
{
	int ret = 0;
	int i;
	int win_count = 0;

	if (NULL == aewbcfg) {
		printk("Null argument in configuration. \n");
		return -EINVAL;
	}

	if (!aewbstat.initialized) {
		DPRINTK_ISPH3A("Setting callback for H3A\n");
		ret = isp_set_callback(CBK_H3A_AWB_DONE, isph3a_aewb_isr, 
					(void*)NULL, (void*)NULL);
		if (ret) {
			printk("No callback for H3A\n");
			return ret;
		}
	}

	ret = isph3a_aewb_set_params(aewbcfg);
	if (ret) {
		printk("Invalid parameters! \n");
		return ret;
	}

	win_count = (aewbcfg->ver_win_count * aewbcfg->hor_win_count);
	win_count += aewbcfg->hor_win_count; /* Blk windows row*/
	ret = (win_count / 8);
	win_count += (win_count % 8)? 1: 0;
	win_count += ret;
	
	aewbstat.win_count = win_count;

	if (aewbstat.stats_buf_size && ((win_count * AEWB_PACKET_SIZE)
					> aewbstat.stats_buf_size)) {
		DPRINTK_ISPH3A("There was a previous buffer... \n");
		isph3a_aewb_enable(0);
		for (i = 0; i < H3A_MAX_BUFF; i++) {
			isph3a_aewb_munmap(&aewbstat.h3a_buff[i]);
			ispmmu_unmap(aewbstat.h3a_buff[i].ispmmu_addr);
			dma_free_coherent(NULL,
				  aewbstat.min_buf_size + 64,
				  (void *)aewbstat.h3a_buff[i].virt_addr,
				  (dma_addr_t)aewbstat.h3a_buff[i].phy_addr);
	  		aewbstat.h3a_buff[i].virt_addr = 0;	  		
	  	}
		aewbstat.stats_buf_size = 0;
	}

	if (!aewbstat.h3a_buff[0].virt_addr) {
		aewbstat.stats_buf_size = win_count * AEWB_PACKET_SIZE;
		aewbstat.min_buf_size = PAGE_ALIGN(aewbstat.stats_buf_size);

		for (i = 0; i < H3A_MAX_BUFF; i++) {
			aewbstat.h3a_buff[i].virt_addr =
				(unsigned long)dma_alloc_coherent(NULL,
						aewbstat.min_buf_size,
						(dma_addr_t *)
						 &aewbstat.h3a_buff[i].phy_addr,
						GFP_KERNEL | GFP_DMA);
			if (aewbstat.h3a_buff[i].virt_addr == 0) {
				printk("Can't acquire memory for "
					"buffer[%d]\n", i);
				return -ENOMEM;
			}
			aewbstat.h3a_buff[i].addr_align =
					aewbstat.h3a_buff[i].virt_addr;
			while((aewbstat.h3a_buff[i].addr_align & 0xFFFFFFC0) !=
				       aewbstat.h3a_buff[i].addr_align)
				aewbstat.h3a_buff[i].addr_align++;
			aewbstat.h3a_buff[i].ispmmu_addr =
				ispmmu_map(aewbstat.h3a_buff[i].phy_addr,
					   aewbstat.min_buf_size);
		}
		isph3a_aewb_unlock_buffers();
		isph3a_aewb_link_buffers();

		/* First active buffer */
		if (active_buff == NULL)
			active_buff = &aewbstat.h3a_buff[0];
		omap_writel(active_buff->ispmmu_addr, ISPH3A_AEWBUFST);
	}
	/* Always remap when calling Configure */
	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if (aewbstat.h3a_buff[i].mmap_addr) {
			isph3a_aewb_munmap(&aewbstat.h3a_buff[i]);
			DPRINTK_ISPH3A("We have munmaped buffer 0x%lX\n",
				aewbstat.h3a_buff[i].virt_addr);
		}
		isph3a_aewb_mmap_buffers(&aewbstat.h3a_buff[i]);
		DPRINTK_ISPH3A("buff[%d] addr is:\n    virt    0x%lX\n"
				       "    aligned 0x%lX\n"
				       "    phys    0x%lX\n"
				       "    ispmmu  0x%08lX\n"
				       "    mmapped 0x%lX\n", i,
					aewbstat.h3a_buff[i].virt_addr,
					aewbstat.h3a_buff[i].addr_align,
					aewbstat.h3a_buff[i].phy_addr,
					aewbstat.h3a_buff[i].ispmmu_addr,
					aewbstat.h3a_buff[i].mmap_addr);		
	}
	/* Enable/disable engine */
	isph3a_aewb_enable(aewbcfg->aewb_enable);
	isph3a_print_status();

	return 0;
}


/*
 * This API allows the user to update White Balance gains, as well as
 * exposure time and analog gain. It is also used to request frame
 * statistics. 
 */ 
int
isph3a_aewb_request_statistics(struct isph3a_aewb_data *aewbdata)
{
	int ret = 0;
	u16 frame_diff = 0;
	u16 frame_cnt = aewbstat.frame_count;
	wait_queue_t wqt;
	u32 exp_time = aewbdata->shutter;
	u16 gain = aewbdata->gain;
	
	
	if (!aewb_config_local.aewb_enable) {
		printk("H3A engine not enabled\n");
		return -EINVAL;
	}
	aewbdata->h3a_aewb_statistics_buf = NULL;
  
	DPRINTK_ISPH3A("User data received: \n");
	DPRINTK_ISPH3A("Digital gain = 0x%04x\n", aewbdata->dgain);
	DPRINTK_ISPH3A("WB gain b *=   0x%04x\n", aewbdata->wb_gain_b);
	DPRINTK_ISPH3A("WB gain r *=   0x%04x\n", aewbdata->wb_gain_r);
	DPRINTK_ISPH3A("WB gain gb =   0x%04x\n", aewbdata->wb_gain_gb);
	DPRINTK_ISPH3A("WB gain gr =   0x%04x\n", aewbdata->wb_gain_gr);
	DPRINTK_ISPH3A("ISP AEWB request status wait for interrupt\n");

	if (aewbdata->update != 0) {
		if (aewbdata->update & SET_EXPOSURE) {
		ret = omap34xxcam_set_exposure_time(0, exp_time);
			if (ret) {
				printk("Error while setting exposure time\n");
				return ret;
			}
		}
		if (aewbdata->update & SET_ANALOG_GAIN) {
		ret = omap34xxcam_set_gain(gain);
			if (ret) {
				printk("Error while setting gain value\n");
				return ret;
			}
		}

		if (aewbdata->update & SET_DIGITAL_GAIN)
			h3awb_update.dgain = (u16)aewbdata->dgain;
		if (aewbdata->update & SET_COLOR_GAINS) {
			h3awb_update.coef3 = (u8)aewbdata->wb_gain_b;
			h3awb_update.coef2 = (u8)aewbdata->wb_gain_gr;
			h3awb_update.coef1 = (u8)aewbdata->wb_gain_gb;
			h3awb_update.coef0 = (u8)aewbdata->wb_gain_r;
		}
		if (aewbdata->update & (SET_COLOR_GAINS | SET_DIGITAL_GAIN))
			wb_update = 1;
	
		if (aewbdata->update & REQUEST_STATISTICS) {
			isph3a_aewb_unlock_buffers();

			/* Stats available? */
			DPRINTK_ISPH3A("Stats available?\n");
			ret = isph3a_aewb_stats_available(aewbdata);
			if (!ret)
				goto out;

			DPRINTK_ISPH3A("Stats in near future?\n");
			/* Stats in near future? */
			if (aewbdata->frame_number > frame_cnt) {
				frame_diff = aewbdata->frame_number - frame_cnt;
			} else if (aewbdata->frame_number < frame_cnt) {
				if ((frame_cnt >
					(MAX_FRAME_COUNT - MAX_FUTURE_FRAMES))
					&& (aewbdata->frame_number
						< MAX_FRAME_COUNT))
					frame_diff = aewbdata->frame_number
						    + MAX_FRAME_COUNT
						    - frame_cnt;
				else {
					/* Frame unavailable */
					frame_diff = MAX_FUTURE_FRAMES + 1;
					aewbdata->h3a_aewb_statistics_buf = NULL;
				}
			}

			if (frame_diff > MAX_FUTURE_FRAMES) {
				printk("Invalid frame requested\n");
				
			} else if(!camnotify){
				/* Block until frame in near future completes */
				aewbstat.frame_req = aewbdata->frame_number;
				aewbstat.stats_req = 1;
				aewbstat.stats_done = 0;
				init_waitqueue_entry(&wqt, current);
				ret =
				   wait_event_interruptible(aewbstat.stats_wait,
						aewbstat.stats_done == 1);
				if (ret < 0)
					return ret;
		
				DPRINTK_ISPH3A("ISP AEWB request status"
						" interrupt raised\n");
				/* Stats now available */
				ret = isph3a_aewb_stats_available(aewbdata);
				if (ret) {
					DPRINTK_ISPH3A
						("After waiting for stats,"
						" stats not available!!\n");
				}
			}
		}
	}
  out:	
	aewbdata->curr_frame = aewbstat.frame_count;
	
	return 0;
}

/*
 * Module Initialisation.
 */
static int __init
isph3a_aewb_init(void)
{
	memset(&aewbstat, 0, sizeof(aewbstat));
	memset(&aewb_regs, 0, sizeof(aewb_regs));

	init_waitqueue_head(&aewbstat.stats_wait);
	spin_lock_init(&aewbstat.buffer_lock);
	return 0;
}

/*
 * Module exit.
 */ 
static void
isph3a_aewb_cleanup(void)
{
	int i;
	isph3a_aewb_enable(0);
	isp_unset_callback(CBK_H3A_AWB_DONE);

	if (aewbstat.h3a_buff) {
		/* Free buffers */
		for (i = 0; i < H3A_MAX_BUFF; i++) {
			ispmmu_unmap(aewbstat.h3a_buff[i].ispmmu_addr);
			dma_free_coherent(NULL,
				aewbstat.min_buf_size + 64,
				(void *)aewbstat.h3a_buff[i].virt_addr,
				(dma_addr_t)aewbstat.h3a_buff[i].phy_addr);
		}
	}
	memset(&aewbstat, 0, sizeof(aewbstat));
	memset(&aewb_regs, 0, sizeof(aewb_regs));
}

/*
 * Debug print
 */ 
static void
isph3a_print_status(void)
{
	DPRINTK_ISPH3A("ISPH3A_PCR = 0x%08x\n",
			omap_readl(ISPH3A_PCR));
	DPRINTK_ISPH3A("ISPH3A_AEWWIN1 = 0x%08x\n",
			omap_readl(ISPH3A_AEWWIN1));
	DPRINTK_ISPH3A("ISPH3A_AEWINSTART = 0x%08x\n",
			omap_readl(ISPH3A_AEWINSTART));
	DPRINTK_ISPH3A("ISPH3A_AEWINBLK = 0x%08x\n",
			omap_readl(ISPH3A_AEWINBLK));
	DPRINTK_ISPH3A("ISPH3A_AEWSUBWIN = 0x%08x\n",
			omap_readl(ISPH3A_AEWSUBWIN));
	DPRINTK_ISPH3A("ISPH3A_AEWBUFST = 0x%08x\n",
			omap_readl(ISPH3A_AEWBUFST));
	DPRINTK_ISPH3A("stats windows = %d\n",
			aewbstat.win_count);
	DPRINTK_ISPH3A("stats buff size = %d\n",
			aewbstat.stats_buf_size);
}
void
isph3a_notify (int notify)
{
  camnotify = notify;
  if(camnotify && aewbstat.initialized){
	printk("Warning Camera Off \n");
	aewbstat.stats_req = 0;
	aewbstat.stats_done = 1;
	wake_up_interruptible(&aewbstat.stats_wait);
	}
}
/*
 * Saves the values of the h3a module registers.
 */
void 
isph3a_save_context(void)
{
	DPRINTK_ISPH3A (" Saving context\n");
	isp_save_context(isph3a_reg_list);
}

/*
 * Restores the values of the h3a module registers.
 */
void 
isph3a_restore_context(void)
{
	DPRINTK_ISPH3A (" Restoring context\n");
	isp_restore_context(isph3a_reg_list);
}

module_init(isph3a_aewb_init);
module_exit(isph3a_aewb_cleanup);

EXPORT_SYMBOL(isph3a_aewb_configure);
EXPORT_SYMBOL(isph3a_aewb_request_statistics);
EXPORT_SYMBOL(isph3a_save_context);
EXPORT_SYMBOL(isph3a_restore_context);
EXPORT_SYMBOL(isph3a_notify);
EXPORT_SYMBOL(isph3a_update_wb);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("H3A ISP Module");
MODULE_LICENSE("GPL");

