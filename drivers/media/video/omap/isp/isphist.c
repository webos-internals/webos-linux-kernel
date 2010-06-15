/*
 * drivers/media/video/omap/isp/isphist.c
 *
 * HISTOGRAM module for TI's OMAP3430 Camera ISP
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
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>


#include "isp.h"
#include "ispreg.h"
#include "isphist.h"
#include "ispmmu.h"
#include "isppreview.h"

struct isp_hist_status {

	u8 hist_enable;		/* enables the histogram module */
	u8 initialized;		/* hist module correctly initialiazed */
	u8 frame_cnt;
	u8 frame_req;
	u8 completed;

} histstat;

struct isp_hist_buffer {
	unsigned long virt_addr;
	unsigned long phy_addr;
	unsigned long addr_align;
	unsigned long ispmmu_addr;
	unsigned long mmap_addr;	/* For userspace */

} hist_buff;

static struct isp_hist_regs {
	u32 reg_pcr;
	u32 reg_cnt;
	u32 reg_wb_gain;
	u32 reg_r0_h;
	u32 reg_r0_v;
	u32 reg_r1_h;
	u32 reg_r1_v;
	u32 reg_r2_h;
	u32 reg_r2_v;
	u32 reg_r3_h;
	u32 reg_r3_v;
	u32 reg_hist_addr;
	u32 reg_hist_data;
	u32 reg_hist_radd;
	u32 reg_hist_radd_off;
	u32 reg_h_v_info;
} hist_regs;
 

/* Structure for saving/restoring histogram module registers*/
struct isp_reg isphist_reg_list[]= {
	{ISPHIST_CNT, 0x0000},
	{ISPHIST_WB_GAIN, 0x0000},
	{ISPHIST_R0_HORZ, 0x0000},
	{ISPHIST_R0_VERT, 0x0000},
	{ISPHIST_R1_HORZ, 0x0000},
	{ISPHIST_R1_VERT, 0x0000},
	{ISPHIST_R2_HORZ, 0x0000}, 
	{ISPHIST_R2_VERT, 0x0000},
	{ISPHIST_R3_HORZ, 0x0000},
	{ISPHIST_R3_VERT, 0x0000},
	{ISPHIST_ADDR, 0x0000},
	{ISPHIST_RADD, 0x0000},
	{ISPHIST_RADD_OFF, 0x0000},		
	{ISPHIST_H_V_INFO, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};
  

static void isp_hist_print_status(void);

/**********************************************************************
* Function    : isp_hist_enable
* Parameters  : enable
* Returns     : none
* Description : Client should configure all the Histogram registers 
*               before calling this function.
**********************************************************************/
static void isp_hist_enable(u8 enable)
{
	if (enable) {
		omap_writel(omap_readl(ISPHIST_PCR) | (ISPHIST_PCR_EN),
			    ISPHIST_PCR);
		DPRINTK_ISPHIST("   histogram enabled \n");

	} else {
		omap_writel(omap_readl(ISPHIST_PCR) & ~(ISPHIST_PCR_EN),
			    ISPHIST_PCR);
		DPRINTK_ISPHIST("   histogram disabled \n");

	}

	histstat.hist_enable = enable;

}

/**********************************************************************
* Function    : isp_hist_update_regs
* Parameters  : none
* Returns     : none
* Description : Helper function to update Histogram registers with  
*               values configured by the user.
**********************************************************************/
static void isp_hist_update_regs(void)
{
	omap_writel(hist_regs.reg_pcr, ISPHIST_PCR);
	omap_writel(hist_regs.reg_cnt, ISPHIST_CNT);
	omap_writel(hist_regs.reg_wb_gain, ISPHIST_WB_GAIN);
	omap_writel(hist_regs.reg_r0_h, ISPHIST_R0_HORZ);
	omap_writel(hist_regs.reg_r0_v, ISPHIST_R0_VERT);
	omap_writel(hist_regs.reg_r1_h, ISPHIST_R1_HORZ);
	omap_writel(hist_regs.reg_r1_v, ISPHIST_R1_VERT);
	omap_writel(hist_regs.reg_r2_h, ISPHIST_R2_HORZ);
	omap_writel(hist_regs.reg_r2_v, ISPHIST_R2_VERT);
	omap_writel(hist_regs.reg_r3_h, ISPHIST_R3_HORZ);
	omap_writel(hist_regs.reg_r3_v, ISPHIST_R3_VERT);
	omap_writel(hist_regs.reg_hist_addr, ISPHIST_ADDR);
	omap_writel(hist_regs.reg_hist_data, ISPHIST_DATA);
	omap_writel(hist_regs.reg_hist_radd, ISPHIST_RADD);
	omap_writel(hist_regs.reg_hist_radd_off, ISPHIST_RADD_OFF);
	omap_writel(hist_regs.reg_h_v_info, ISPHIST_H_V_INFO);

}

/**********************************************************************
* Function    : isp_hist_isr
* Parameters  : IRQ0STATUS in case of MMU error, 0 for hist interrupt.
*		arg1 and arg2 Not used as of now.
* Returns     : none
* Description : Callback from ISP driver for HIST interrupt.
*               
**********************************************************************/
static void isp_hist_isr(unsigned long status, void *arg1, void *arg2)
{
	isp_hist_enable(0);

	if ((HIST_DONE & status) != HIST_DONE)
		return;

	if (!histstat.completed) {
		if (histstat.frame_req == histstat.frame_cnt) {
			histstat.frame_cnt = 0;
			histstat.frame_req = 0;
			histstat.completed = 1;
		} else {
			isp_hist_enable(1);
			histstat.frame_cnt++;
		}
	}
}

/**********************************************************************
* Function    : isp_hist_reset_mem
* Parameters  : none
* Returns     : Returns 0 after histogram memory was cleared.
* Description : Helper function to clear Histogram memory before start
*               statistics engine.
**********************************************************************/
static int isp_hist_reset_mem(void)
{
	int i;

	omap_writel((omap_readl(ISPHIST_CNT)) | ISPHIST_CNT_CLR_EN,
		    ISPHIST_CNT);

	for (i = 0; i < HIST_MEM_SIZE; i++) {
		omap_readl(ISPHIST_DATA);
	}

	omap_writel((omap_readl(ISPHIST_CNT)) & ~ISPHIST_CNT_CLR_EN,
		    ISPHIST_CNT);

	return 0;
}

/**********************************************************************
* Function    : isp_hist_set_params
* Parameters  : Pointer to user configuration structure.
* Returns     : Returns 0 on success configuration.
* Description : Helper function to check and store user given params.
*               
**********************************************************************/
static int isp_hist_set_params(struct isp_hist_config *user_cfg)
{

	int reg_num = 0;
	int bit_shift = 0;


	if (omap_readl(ISPHIST_PCR) & ISPHIST_PCR_BUSY_MASK)
		return -EINVAL;
	
	/* DATA SIZE *///     0x0: The pixels are coded on more than 8 bits.
		      //      0x1: The pixels are coded on 8 bits.
	if (user_cfg->input_bit_width > MIN_BIT_WIDTH){
		WRITE_DATA_SIZE(hist_regs.reg_cnt, 0);
	} else {
		WRITE_DATA_SIZE(hist_regs.reg_cnt, 1);
	}
	WRITE_SOURCE(hist_regs.reg_cnt, user_cfg->hist_source);

	if (user_cfg->hist_source) {

		/* frame-input width and height if source is memory */
		WRITE_HV_INFO(hist_regs.reg_h_v_info, user_cfg->hist_h_v_info);

		if ((user_cfg->hist_radd & ISP_32B_BOUNDARY_BUF) ==
		    user_cfg->hist_radd) {
			WRITE_RADD(hist_regs.reg_hist_radd,
				   user_cfg->hist_radd);
		} else {
			printk(KERN_ERR "Address should be in 32 byte boundary\n");
			return -EINVAL;
		}

		/* line-offset for frame-input */
		if ((user_cfg->hist_radd_off & ISP_32B_BOUNDARY_OFFSET) ==
		    user_cfg->hist_radd_off) {
			WRITE_RADD_OFF(hist_regs.reg_hist_radd_off,
				       user_cfg->hist_radd_off);
		} else {
			printk(KERN_ERR "Offset should be in 32 byte boundary\n");
			return -EINVAL;
		}

	}

	isp_hist_reset_mem();
	DPRINTK_ISPHIST("ISPHIST: Memory Cleared\n");
	histstat.frame_req = user_cfg->hist_frames;

	/* White Balance Field-to-Pattern Assignments */
	if (unlikely((user_cfg->wb_gain_R > MAX_WB_GAIN)
		     || (user_cfg->wb_gain_RG > MAX_WB_GAIN)
		     || (user_cfg->wb_gain_B > MAX_WB_GAIN)
		     || (user_cfg->wb_gain_BG > MAX_WB_GAIN))) {
		printk(KERN_ERR "Invalid WB gain\n");
		return -EINVAL;
	} else {
		WRITE_WB_R(hist_regs.reg_wb_gain, user_cfg->wb_gain_R);
		WRITE_WB_RG(hist_regs.reg_wb_gain, user_cfg->wb_gain_RG);
		WRITE_WB_B(hist_regs.reg_wb_gain, user_cfg->wb_gain_B);
		WRITE_WB_BG(hist_regs.reg_wb_gain, user_cfg->wb_gain_BG);
	}

	/* Regions size and position */

	if (user_cfg->num_regions > MAX_REGIONS)
		return -EINVAL;

	if (likely((user_cfg->reg0_hor & ISPHIST_REGHORIZ_HEND_MASK)
		     - ((user_cfg->reg0_hor & ISPHIST_REGHORIZ_HSTART_MASK)
			>> ISPHIST_REGHORIZ_HSTART_SHIFT))) {
		WRITE_REG_HORIZ(hist_regs.reg_r0_h, user_cfg->reg0_hor);
		reg_num++;
	} else {
		printk(KERN_ERR "Invalid Region parameters\n");
		return -EINVAL;
	}

	if (likely((user_cfg->reg0_ver & ISPHIST_REGVERT_VEND_MASK)
		     - ((user_cfg->reg0_ver & ISPHIST_REGVERT_VSTART_MASK)
			>> ISPHIST_REGVERT_VSTART_SHIFT))) {
		WRITE_REG_VERT(hist_regs.reg_r0_v, user_cfg->reg0_ver);
	} else {
		printk(KERN_ERR "Invalid Region parameters\n");
		return -EINVAL;
	}

	if (user_cfg->num_regions >= 1) {
		if (likely((user_cfg->reg1_hor & ISPHIST_REGHORIZ_HEND_MASK)
			     -
			     ((user_cfg->
			       reg1_hor & ISPHIST_REGHORIZ_HSTART_MASK)
			      >> ISPHIST_REGHORIZ_HSTART_SHIFT))) {
			WRITE_REG_HORIZ(hist_regs.reg_r1_h, user_cfg->reg1_hor);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}

		if (likely((user_cfg->reg1_ver & ISPHIST_REGVERT_VEND_MASK)
			     -
			     ((user_cfg->reg1_ver & ISPHIST_REGVERT_VSTART_MASK)
			      >> ISPHIST_REGVERT_VSTART_SHIFT))) {
			WRITE_REG_VERT(hist_regs.reg_r1_v, user_cfg->reg1_ver);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}
	}

	if (user_cfg->num_regions >= 2) {
		if (likely((user_cfg->reg2_hor & ISPHIST_REGHORIZ_HEND_MASK)
			     -
			     ((user_cfg->
			       reg2_hor & ISPHIST_REGHORIZ_HSTART_MASK)
			      >> ISPHIST_REGHORIZ_HSTART_SHIFT))) {
			WRITE_REG_HORIZ(hist_regs.reg_r2_h, user_cfg->reg2_hor);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}

		if (likely((user_cfg->reg2_ver & ISPHIST_REGVERT_VEND_MASK)
			     -
			     ((user_cfg->reg2_ver & ISPHIST_REGVERT_VSTART_MASK)
			      >> ISPHIST_REGVERT_VSTART_SHIFT))) {
			WRITE_REG_VERT(hist_regs.reg_r2_v, user_cfg->reg2_ver);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}
	}

	if (user_cfg->num_regions >= 3) {
		if (likely((user_cfg->reg3_hor & ISPHIST_REGHORIZ_HEND_MASK)
			     -
			     ((user_cfg->
			       reg3_hor & ISPHIST_REGHORIZ_HSTART_MASK)
			      >> ISPHIST_REGHORIZ_HSTART_SHIFT))) {
			WRITE_REG_HORIZ(hist_regs.reg_r3_h, user_cfg->reg3_hor);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}

		if (likely((user_cfg->reg3_ver & ISPHIST_REGVERT_VEND_MASK)
			     -
			     ((user_cfg->reg3_ver & ISPHIST_REGVERT_VSTART_MASK)
			      >> ISPHIST_REGVERT_VSTART_SHIFT))) {
			WRITE_REG_VERT(hist_regs.reg_r3_v, user_cfg->reg3_ver);
		} else {
			printk(KERN_ERR "Invalid Region parameters\n");
			return -EINVAL;
		}
	}
	/* NUMBER OF BINS */
	reg_num = user_cfg->num_regions;
	if (unlikely(((user_cfg->hist_bins > BINS_256)
		      && (user_cfg->hist_bins != BINS_32))
		     || ((user_cfg->hist_bins == BINS_256) && reg_num != 0)
		     || ((user_cfg->hist_bins == BINS_128) && reg_num >= 2))) {
		printk(KERN_ERR "Invalid Bins Number: %d\n", user_cfg->hist_bins);
		return -EINVAL;
	} else {
		WRITE_NUM_BINS(hist_regs.reg_cnt, user_cfg->hist_bins);
	}

	/* PIXEL SHIFT VALUE */

	if ((user_cfg->input_bit_width > MAX_BIT_WIDTH) ||
	    (user_cfg->input_bit_width < MIN_BIT_WIDTH)) {
		printk(KERN_ERR "Invalid Bit Width: %d\n", user_cfg->input_bit_width);
		return -EINVAL;
	} else {

		switch (user_cfg->hist_bins) {
		case BINS_256:
			bit_shift = user_cfg->input_bit_width - 8;
			break;
		case BINS_128:
			bit_shift = user_cfg->input_bit_width - 7;
			break;
		case BINS_64:
			bit_shift = user_cfg->input_bit_width - 6;
			break;
		case BINS_32:
			bit_shift = user_cfg->input_bit_width - 5;
			break;
		default:
			{
				return -EINVAL;
			}
		}
		WRITE_BIT_SHIFT(hist_regs.reg_cnt, bit_shift);
	}

	isp_hist_update_regs();
	histstat.initialized = 1;

	return 0;
}

/**********************************************************************
* Function    : isp_hist_mmap_buffer 
* Parameters  : Pointer to buffer structure.
* Returns     : Returns 0 on success buffer mapped.
* Description : Helper function to mmap buffers to user space.
*		buffer passed need to already have a valid physical 
*		address: buffer->phy_addr. It returns user pointer as 
*		unsigned long in buffer->mmap_addr
*
**********************************************************************/
static int isp_hist_mmap_buffer(struct isp_hist_buffer *buffer)
{
	struct vm_area_struct vma;
	struct mm_struct *mm = current->mm;
	int size = PAGE_SIZE;
	unsigned long addr = 0;
	unsigned long pgoff = 0, flags = MAP_SHARED | MAP_ANONYMOUS;
	unsigned long prot = PROT_READ | PROT_WRITE;
	void *pos = (void *)buffer->virt_addr;

	size = PAGE_ALIGN(size);

	addr = get_unmapped_area(NULL, addr, size, pgoff, flags);
	vma.vm_mm = mm;
	vma.vm_start = addr;
	vma.vm_end = addr + size;
	vma.vm_flags = calc_vm_prot_bits(prot) | calc_vm_flag_bits(flags);
	vma.vm_pgoff = pgoff;
	vma.vm_file = NULL;
	vma.vm_page_prot = protection_map[vma.vm_flags];

	if (vm_insert_page(&vma, addr, vmalloc_to_page(pos)))
		return -EAGAIN;

	buffer->mmap_addr = vma.vm_start;
	return 0;
}

/**********************************************************************
* Function    : isp_hist_configure
* Parameters  : Pointer to user configuration structure.
* Returns     : Returns 0 on success configuration.
* Description : API to configure HIST registers
*
**********************************************************************/
int isp_hist_configure(struct isp_hist_config *histcfg)
{

	int ret = 0;

	if (NULL == histcfg) {
		printk(KERN_ERR "Null argument in configuration. \n");
		return -EINVAL;
	}

	if (!histstat.initialized) {
		DPRINTK_ISPHIST("Setting callback for HISTOGRAM\n");
		ret = isp_set_callback(CBK_HIST_DONE, isp_hist_isr,
				       (void *)NULL, (void *)NULL);
		if (ret) {
			printk(KERN_ERR "No callback for HIST\n");
			return ret;
		}
	}

	ret = isp_hist_set_params(histcfg);
	if (ret) {
		printk(KERN_ERR "Invalid parameters! \n");
		return ret;
	}
  
	if (hist_buff.virt_addr != 0){
		hist_buff.mmap_addr = 0;
		ispmmu_unmap(hist_buff.ispmmu_addr);
		dma_free_coherent(NULL, PAGE_SIZE, (void *)hist_buff.virt_addr,
		(dma_addr_t)hist_buff.phy_addr);
	}
          						
	hist_buff.virt_addr =
	    (unsigned long)dma_alloc_coherent(NULL, PAGE_SIZE, (dma_addr_t *)
					      &hist_buff.phy_addr, GFP_KERNEL | GFP_DMA);
	if (hist_buff.virt_addr == 0) {
		printk(KERN_ERR "Can't acquire memory for ");
		return -ENOMEM;
	}

	hist_buff.ispmmu_addr = ispmmu_map(hist_buff.phy_addr, PAGE_SIZE);

	if (hist_buff.mmap_addr) {
		hist_buff.mmap_addr = 0;
		DPRINTK_ISPHIST("We have munmaped buffer 0x%lX\n",
				hist_buff.virt_addr);
	}

	isp_hist_mmap_buffer(&hist_buff);

	histstat.frame_cnt = 0;	/*reset number of frames to be processed */
	histstat.completed = 0;
	isp_hist_enable(1);
	isp_hist_print_status();

	return 0;
}

/**********************************************************************
* Function    : isp_hist_request_statistics
* Parameters  : Pointer to data structure.
* Returns     : Returns 0 on successful request.
* Description : This API allows the user to request 
*		for histogram statistics
*
**********************************************************************/
int isp_hist_request_statistics(struct isp_hist_data *histdata)
{
	int i;

	if (omap_readl(ISPHIST_PCR) & ISPHIST_PCR_BUSY_MASK)
		return -EBUSY;
	
	if (!histstat.completed && histstat.initialized)
		return -EINVAL;

	omap_writel((omap_readl(ISPHIST_CNT)) | ISPHIST_CNT_CLR_EN,
		    ISPHIST_CNT);
	histdata->hist_statistics_buf = (u32 *) hist_buff.mmap_addr;

	for (i = 0; i < HIST_MEM_SIZE; i++) {
		*(histdata->hist_statistics_buf + i) = omap_readl(ISPHIST_DATA);
	}
	omap_writel((omap_readl(ISPHIST_CNT)) & ~ISPHIST_CNT_CLR_EN,
		    ISPHIST_CNT);
	histstat.completed = 0;
	return 0;
}

/**********************************************************************
* Function    : isp_hist_init
* Parameters  : none.
* Returns     : Returns 0 on success initialization.
* Description : Module Initialisation.               
*
**********************************************************************/
static int __init isp_hist_init(void)
{
	memset(&histstat, 0, sizeof(histstat));
	memset(&hist_regs, 0, sizeof(hist_regs));

	return 0;
}

/**********************************************************************
* Function    : isp_hist_cleanup
* Parameters  : none
* Returns     : none
* Description : Module exit.
*
**********************************************************************/
static void isp_hist_cleanup(void)
{

	isp_hist_enable(0);
	
	mdelay(100);
	
	isp_unset_callback(CBK_HIST_DONE);

	if (hist_buff.ispmmu_addr) {
		/* Free buffers */
		ispmmu_unmap(hist_buff.ispmmu_addr);
		dma_free_coherent(NULL,  PAGE_SIZE,
				  (void *)hist_buff.virt_addr,
				  (dma_addr_t) hist_buff.phy_addr);
	}

	memset(&histstat, 0, sizeof(histstat));
	memset(&hist_regs, 0, sizeof(hist_regs));

}

/**********************************************************************
* Function    : isphist_save_context
* Parameters  : none
* Returns     : none
* Description : Saves the values of the histogram module registers.
*
**********************************************************************/
void 
isphist_save_context(void)
{
	DPRINTK_ISPHIST (" Saving context\n");
	isp_save_context(isphist_reg_list);
}

/**********************************************************************
* Function    : isphist_restore_context
* Parameters  : none
* Returns     : none
* Description : Restores the values of the histogram module registers.
*
**********************************************************************/
void 
isphist_restore_context(void)
{
	DPRINTK_ISPHIST (" Restoring context\n");
	isp_restore_context(isphist_reg_list);
}

/**********************************************************************
* Function    : isp_hist_print_status
* Parameters  : none
* Returns     : none
* Description : Debug print
*
**********************************************************************/
static void isp_hist_print_status(void)
{
	DPRINTK_ISPHIST("ISPHIST_PCR = 0x%08x\n", 
			omap_readl(ISPHIST_PCR));
	DPRINTK_ISPHIST("ISPHIST_CNT = 0x%08x\n", 
			omap_readl(ISPHIST_CNT));
	DPRINTK_ISPHIST("ISPHIST_WB_GAIN = 0x%08x\n",
			omap_readl(ISPHIST_WB_GAIN));
	DPRINTK_ISPHIST("ISPHIST_R0_HORZ = 0x%08x\n",
			omap_readl(ISPHIST_R0_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R0_VERT = 0x%08x\n",
			omap_readl(ISPHIST_R0_VERT));
	DPRINTK_ISPHIST("ISPHIST_R1_HORZ = 0x%08x\n",
			omap_readl(ISPHIST_R1_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R1_VERT = 0x%08x\n",
			omap_readl(ISPHIST_R1_VERT));
	DPRINTK_ISPHIST("ISPHIST_R2_HORZ = 0x%08x\n",
			omap_readl(ISPHIST_R2_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R2_VERT = 0x%08x\n",
			omap_readl(ISPHIST_R2_VERT));
	DPRINTK_ISPHIST("ISPHIST_R3_HORZ = 0x%08x\n",
			omap_readl(ISPHIST_R3_HORZ));
	DPRINTK_ISPHIST("ISPHIST_R3_VERT = 0x%08x\n",
			omap_readl(ISPHIST_R3_VERT));
	DPRINTK_ISPHIST("ISPHIST_ADDR = 0x%08x\n", 
			omap_readl(ISPHIST_ADDR));
	DPRINTK_ISPHIST("ISPHIST_RADD = 0x%08x\n", 
			omap_readl(ISPHIST_RADD));
	DPRINTK_ISPHIST("ISPHIST_RADD_OFF = 0x%08x\n",
			omap_readl(ISPHIST_RADD_OFF));
	DPRINTK_ISPHIST("ISPHIST_H_V_INFO = 0x%08x\n",
			omap_readl(ISPHIST_H_V_INFO));
}



module_init(isp_hist_init);
module_exit(isp_hist_cleanup);
module_exit(isphist_save_context);
module_exit(isphist_restore_context);

EXPORT_SYMBOL(isp_hist_configure);
EXPORT_SYMBOL(isp_hist_request_statistics);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("HISTOGRAM ISP Module");
MODULE_LICENSE("GPL");
