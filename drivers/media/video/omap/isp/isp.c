 /*
  * drivers/media/video/omap/isp/isp.c
 *
 * Driver Library for ISP Control module in TI's OMAP3430 Camera ISP
 * ISP interface and IRQ related APIs are defined here.
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

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/mach-types.h>
#include <asm/arch/clock.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"
#include "isppreview.h"
#include "ispresizer.h"
#include "ispmmu.h"
#include "isph3a.h"
#include "isphist.h"

#define ISP_XCLKA_DEFAULT 	0x12

extern void omap34xxcam_sensor_restore(void);

/*Structure for IRQ related info */
static struct ispirq {
	isp_callback_t isp_callbk[8];
	void *isp_callbk_arg1[8];
	void *isp_callbk_arg2[8];
} ispirq_obj;

/* Structure for storing ISP Control module information*/
static struct isp {
	spinlock_t lock;	/* spinlock to sync b/w isr and processes */
	struct semaphore semlock;
	u8 if_status;
	u8 interfacetype;
	int ref_count;
	struct clk * cam_ick;
	struct clk * cam_fck;
} isp_obj;

/* Structure for saving/restoring ISP module registers*/

static struct isp_reg isp_reg_list[]= {
	{ISP_SYSCONFIG, 0x0000},
	{ISP_IRQ0ENABLE, 0x0000},
	{ISP_IRQ1ENABLE, 0x0000},
	{ISP_TCTRL_GRESET_LENGTH, 0x0000},
	{ISP_TCTRL_PSTRB_REPLAY, 0x0000},
	{ISP_CTRL, 0x0000},
	{ISP_TCTRL_CTRL, 0x0000}, 
	{ISP_TCTRL_FRAME, 0x0000},
	{ISP_TCTRL_PSTRB_DELAY, 0x0000},
	{ISP_TCTRL_STRB_DELAY, 0x0000},
	{ISP_TCTRL_SHUT_DELAY, 0x0000},
	{ISP_TCTRL_PSTRB_LENGTH, 0x0000},
	{ISP_TCTRL_STRB_LENGTH, 0x0000},
	{ISP_TCTRL_SHUT_LENGTH, 0x0000},		
	{ISP_CBUFF_SYSCONFIG, 0x0000},
	{ISP_CBUFF_IRQENABLE, 0x0000},
	{ISP_CBUFF0_CTRL, 0x0000},
	{ISP_CBUFF1_CTRL, 0x0000},
	{ISP_CBUFF0_START, 0x0000},
	{ISP_CBUFF1_START, 0x0000},
	{ISP_CBUFF0_END, 0x0000},
	{ISP_CBUFF1_END, 0x0000},
	{ISP_CBUFF0_WINDOWSIZE, 0x0000},
	{ISP_CBUFF1_WINDOWSIZE, 0x0000},
	{ISP_CBUFF0_THRESHOLD, 0x0000},
	{ISP_CBUFF1_THRESHOLD, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};

//  flag to check first time of isp_get  
static int off_mode=0;

/*
 *Sets the callback for the ISP module done events.
 * type		: Type of the event for which callback is requested.
 * callback	: Method to be called as callback in the ISR context.
 * arg1		: Argument to be passed when callback is called in ISR.
 * arg2		: Argument to be passed when callback is called in ISR.
 */
int
isp_set_callback(enum isp_callback_type type, isp_callback_t callback,
		 void *arg1, void *arg2)
{
	unsigned long irqflags = 0;

	if (callback == NULL){
		DPRINTK_ISPCTRL("ISP_ERR : Null Callback\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	ispirq_obj.isp_callbk[type] = callback;
	ispirq_obj.isp_callbk_arg1[type] = arg1;
	ispirq_obj.isp_callbk_arg2[type] = arg2;
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);
	
	switch (type) {
	case CBK_HS_VS:	
		omap_writel(IRQ0ENABLE_HS_VS_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_HS_VS_IRQ,
				ISP_IRQ0ENABLE);
		break;
#if 0
	case CBK_CCDC_VD1:
		omap_writel(IRQ0ENABLE_CCDC_VD1_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_CCDC_VD1_IRQ,
				ISP_IRQ0ENABLE);
		break;
#endif
	case CBK_PREV_DONE:
		omap_writel(IRQ0ENABLE_PRV_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_PRV_DONE_IRQ,
				ISP_IRQ0ENABLE);
		break;
	case CBK_RESZ_DONE:
		omap_writel(IRQ0ENABLE_RSZ_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_RSZ_DONE_IRQ,
				ISP_IRQ0ENABLE);
		break;
	case CBK_MMU_ERR:
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_MMU_ERR_IRQ,
				ISP_IRQ0ENABLE);
		omap_writel(omap_readl(ISPMMU_IRQENABLE)|IRQENABLE_MULTIHITFAULT 
				| IRQENABLE_TWFAULT | IRQENABLE_EMUMISS 
				| IRQENABLE_TRANSLNFAULT | IRQENABLE_TLBMISS,
				ISPMMU_IRQENABLE);
		break;
	case CBK_H3A_AWB_DONE:
		omap_writel(IRQ0ENABLE_H3A_AWB_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|
				IRQ0ENABLE_H3A_AWB_DONE_IRQ,
				ISP_IRQ0ENABLE);
		break;
	case CBK_HIST_DONE:
		omap_writel(IRQ0ENABLE_HIST_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|
				IRQ0ENABLE_HIST_DONE_IRQ,
				ISP_IRQ0ENABLE);
		break;	
	default:
		break;
	};

	return 0;
}

/*
 *Clears  the callback for the ISP module done events.
 * type		: Type of the event for which callback to be cleared.
 */
int
isp_unset_callback(enum isp_callback_type type)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	ispirq_obj.isp_callbk[type] = NULL;
	ispirq_obj.isp_callbk_arg1[type] = NULL;
	ispirq_obj.isp_callbk_arg2[type] = NULL;
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);

	switch (type) {
	case CBK_CCDC_VD0:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_CCDC_VD0_IRQ),ISP_IRQ0ENABLE);
		break;
	case CBK_CCDC_VD1:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_CCDC_VD1_IRQ),ISP_IRQ0ENABLE);
		break;
	case CBK_PREV_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_PRV_DONE_IRQ),ISP_IRQ0ENABLE);
		break;
	case CBK_RESZ_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_RSZ_DONE_IRQ),ISP_IRQ0ENABLE);
		break;
	case CBK_MMU_ERR:
		omap_writel(omap_readl(ISPMMU_IRQENABLE)&
				~(IRQENABLE_MULTIHITFAULT 
				| IRQENABLE_TWFAULT | IRQENABLE_EMUMISS 
				| IRQENABLE_TRANSLNFAULT | IRQENABLE_TLBMISS),
				ISPMMU_IRQENABLE);
		break;
	case CBK_H3A_AWB_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_H3A_AWB_DONE_IRQ),ISP_IRQ0ENABLE);
		break;
	case CBK_HIST_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_HIST_DONE_IRQ),ISP_IRQ0ENABLE);
		break;
	case CBK_HS_VS:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_HS_VS_IRQ),ISP_IRQ0ENABLE);
		break;
		
	default:
		break;
	};
	return 0;
}

/*
 * Reserves the parallel or serial interface requested for.
 * At most 2 interfaces shall be enabled at a time.
 * if_t		: Type of interface requested parallel or serial.
 */
int
isp_request_interface(enum isp_interface_type if_t)
{
	if (isp_obj.if_status & if_t){
		DPRINTK_ISPCTRL("ISP_ERR : Requested Interface already \
			allocated\n");
		return -EBUSY;	/*Requested interface is already allocated*/
	}
	if ((isp_obj.if_status == (ISP_PARLL | ISP_CSIA))
		|| isp_obj.if_status == (ISP_CSIA | ISP_CSIB)){
		DPRINTK_ISPCTRL("ISP_ERR : No Free interface now\n");
		return -EBUSY;	/*No free interface */
	}
	if (isp_obj.if_status == 0) {	/* this is the first request. always OK. */
		isp_obj.if_status = if_t;
		return 0;
	}

	/* Previously we had only one interface alloacted,
	* try to allocate the second for if_t
	*/
	if (((isp_obj.if_status == ISP_PARLL) && (if_t == ISP_CSIA)) ||
		((isp_obj.if_status == ISP_CSIA) && (if_t == ISP_PARLL)) ||
		((isp_obj.if_status == ISP_CSIA) && (if_t == ISP_CSIB)) ||
		((isp_obj.if_status == ISP_CSIB) && (if_t == ISP_CSIA))) {
		isp_obj.if_status |= if_t;
		return 0;
	}
	else{	/* all other combs invalid */
		DPRINTK_ISPCTRL("ISP_ERR : Invalid Combination Serial- \
			Parallel interface\n");
		return -EINVAL;
	}
}

/*
 * Frees the parallel or serial interface that is passed.
 * if_t		: Type of interface tobe freed parallel or serial.
 */
int
isp_free_interface(enum isp_interface_type if_t)
{
	isp_obj.if_status &= ~(if_t);
	return 0;
}

/*
 * Verifies the clock range passed with the max/min values for MCLK divisor.
 * xclk		: Divisor for MCLK to produce xclk.
 */
u32
isp_negotiate_xclka(u32 xclk)
{
	/* Actual negotiation happens for Frame rate control if needed*/
	/* Now checking only the boundary conditions */
	if((xclk ==0) || (xclk==1)){
		DPRINTK_ISPCTRL("ISP_ERR : Divisor is getting disabled\n");
		return -EINVAL;
	}
	if(xclk == ISPTCTRL_CTRL_DIVA_Bypass)
		DPRINTK_ISPCTRL("Divisor is bypassed cam_xclka=cam_xclk\n");

	return xclk;
}

/*
 * Verifies the clock range passed with the max/min values for MCLK divisor.
 * xclk		: Divisor for MCLK to produce xclk.
 */
u32
isp_negotiate_xclkb(u32 xclk)
{
	/* Actual negotiation happens for Frame rate control if needed*/
	/* Now checking only the boundary conditions */
	if((xclk ==0) || (xclk==1)){
		DPRINTK_ISPCTRL("ISP_ERR : Divisor is getting disabled\n");
		return -EINVAL;
	}
	if(xclk == ISPTCTRL_CTRL_DIVB_Bypass)
		DPRINTK_ISPCTRL("Divisor is bypassed cam_xclkb=cam_xclk\n");
	return xclk;
}

/*
 * Configures the clock with the passed MCLK divisor A.
 * xclk		: Desired clock frequency
 * xclka  	: Divisor for MCLK to produce xclk.
 */
u32
isp_set_xclka(u32 xclk)
{
	unsigned long xclka;

	if(xclk)
	  xclka = CM_CAM_MCLK_HZ / xclk;
	else
	  xclka = xclk;

	omap_writel(((omap_readl(ISP_TCTRL_CTRL)) & ~ISPTCTRL_CTRL_DIVA_Bypass)
		    | (xclka << ISPTCTRL_CTRL_DIVA_SHIFT),
		    ISP_TCTRL_CTRL);
	DPRINTK_ISPCTRL("isp_set_xclka() = %lu, divisor = %lu\n",
			(unsigned long)xclk, xclka);
	return xclka;
}

/*
 * Configures the clock with the passed MCLK divisor B.
 * xclk		: Divisor for MCLK to produce xclk.
 */
u32
isp_set_xclkb(u32 xclk)
{
	/* Now use fixed clock untill frame rate control*/
	omap_writel((omap_readl(ISP_TCTRL_CTRL))
		|(xclk<<ISPTCTRL_CTRL_DIVB_SHIFT)
		, ISP_TCTRL_CTRL);
	return xclk;
}

/*
 * Returns the XCLKA in Hz.
 */
u32
isp_get_xclka(void)
{
	u32 xclka_val = 0;
	u32 xclka = 0;

	xclka_val = omap_readl(ISP_TCTRL_CTRL) & ISPTCTRL_CTRL_DIVA_Bypass;

	/*Power Management should provide an API for configured CAM_MCLK freq*/
	/* MCLK is 216 MHz*/
	xclka =CM_CAM_MCLK_HZ/xclka_val;

	return xclka;
}

/*
 * Returns the XCLKB in Hz.
 */
u32
isp_get_xclkb(void)
{
	u32 xclkb_val = 0;
	u32 xclkb = 0;

	xclkb_val = omap_readl(ISP_TCTRL_CTRL) & ISPTCTRL_CTRL_DIVB_Bypass;

	/*Power Management should provide an API for configured CAM_MCLK freq*/
	/* MCLK is 216 MHz*/
	xclkb =CM_CAM_MCLK_HZ/xclkb_val;

	return xclkb;
}
/*
 Sysconfig settings, For Power Management
*/
void 
isp_power_settings(struct isp_sysc isp_sysconfig)
{

	if(isp_sysconfig.idle_mode){
		omap_writel(ISP_SYSCONFIG_AUTOIDLE |
			(ISP_SYSCONFIG_MIdleMode_SmartStandBy <<
			ISP_SYSCONFIG_MIdleMode_SHIFT), ISP_SYSCONFIG);

		omap_writel(ISPMMU_AUTOIDLE | 
			(ISPMMU_SIdlemode_Smartidle << ISPMMU_SIdlemode_Shift)
						   ,ISPMMU_SYSCONFIG);
		if(is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
			omap_writel(ISPCSI1_AUTOIDLE |
				(ISPCSI1_MIdleMode_SmartStandBy <<
				ISPCSI1_MIdleMode_Shift), ISP_CSIA_SYSCONFIG);
		
			omap_writel(ISPCSI1_AUTOIDLE |
				(ISPCSI1_MIdleMode_SmartStandBy <<
				ISPCSI1_MIdleMode_Shift), ISP_CSIB_SYSCONFIG);
		}
		omap_writel(ISPCTRL_SBL_AutoIdle,ISP_CTRL);

	} else {
		omap_writel(ISP_SYSCONFIG_AUTOIDLE |
			(ISP_SYSCONFIG_MIdleMode_ForceStandBy <<
			ISP_SYSCONFIG_MIdleMode_SHIFT), ISP_SYSCONFIG);

		omap_writel(ISPMMU_AUTOIDLE |
			(ISPMMU_SIdlemode_Noidle << ISPMMU_SIdlemode_Shift)
						,ISPMMU_SYSCONFIG);
		if(is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
			omap_writel(ISPCSI1_AUTOIDLE |
				(ISPCSI1_MIdleMode_ForceStandBy <<
				ISPCSI1_MIdleMode_Shift), ISP_CSIA_SYSCONFIG);

			omap_writel(ISPCSI1_AUTOIDLE |
				(ISPCSI1_MIdleMode_ForceStandBy <<
				ISPCSI1_MIdleMode_Shift), ISP_CSIB_SYSCONFIG);
		}

		omap_writel(ISPCTRL_SBL_AutoIdle,ISP_CTRL);

		}	


}

/*
 * Configures the ISP Control interace related parameters.
 * config		: Structure containing the interface info like parallel
 *		or serial, data line fro 10bits among the 12 lines,
 *		sync detection on VS/HS rising/falling edges.
 */
int
isp_configure_interface(struct isp_interface_config *config)
{
	u32 ispctrl_val = omap_readl(ISP_CTRL);
	u32 ispccdc_vdint_val;

	ispctrl_val &= (ISPCTRL_PAR_SER_CLK_SEL_MASK);
	ispctrl_val |= config->ccdc_par_ser;
	ispctrl_val &= ISPCTRL_SHIFT_MASK;
	ispctrl_val |= (config->dataline_shift<<ISPCTRL_SHIFT_SHIFT);
	ispctrl_val &= ~ISPCTRL_PAR_CLK_POL_INV;
	ispctrl_val |= (config->para_clk_pol<<ISPCTRL_PAR_CLK_POL_SHIFT);
	ispctrl_val &= ~(ISPCTRL_PAR_BRIDGE_BENDIAN);
	ispctrl_val |= (config->par_bridge<<ISPCTRL_PAR_BRIDGE_SHIFT);
	ispctrl_val &= ~(ISPCTRL_SYNC_DETECT_VSRISE);
	ispctrl_val |= (config->hsvs_syncdetect<<ISPCTRL_SYNC_DETECT_SHIFT);

	omap_writel(ispctrl_val, ISP_CTRL);

	/*Configuring the number of horizontal lines from the VS pulse
	for the interrupt.*/
	ispccdc_vdint_val = omap_readl(ISPCCDC_VDINT);
	ispccdc_vdint_val &= ~(ISPCCDC_VDINT_0_MASK << ISPCCDC_VDINT_0_SHIFT);
	ispccdc_vdint_val &= ~(ISPCCDC_VDINT_1_MASK << ISPCCDC_VDINT_1_SHIFT);
	omap_writel((config->vdint0_timing<< ISPCCDC_VDINT_0_SHIFT)
		| (config->vdint1_timing << ISPCCDC_VDINT_1_SHIFT)
		, ISPCCDC_VDINT);
	return 0;
}

void
isp_CCDC_VD01_enable()
{
	omap_writel(IRQ0ENABLE_CCDC_VD0_IRQ | IRQ0ENABLE_CCDC_VD1_IRQ,
				ISP_IRQ0STATUS);
	omap_writel(omap_readl(ISP_IRQ0ENABLE) | IRQ0ENABLE_CCDC_VD0_IRQ
			| IRQ0ENABLE_CCDC_VD1_IRQ,ISP_IRQ0ENABLE);
}

void
isp_CCDC_VD01_disable()
{
	omap_writel((omap_readl(ISP_IRQ0ENABLE))&
			(~IRQ0ENABLE_CCDC_VD0_IRQ),ISP_IRQ0ENABLE);
	omap_writel((omap_readl(ISP_IRQ0ENABLE))&
			(~IRQ0ENABLE_CCDC_VD1_IRQ),ISP_IRQ0ENABLE);
}

/*
 * ISR for Camera ISP module. Handles the corresponding callback if plugged in.
 * irq		: Not used currently.
 * ispirq_disp	: The object that is passed while request_irq is called.
 *		This is the ispirq_obj object containing info on the callback.
 */
static irqreturn_t
omap34xx_isp_isr(int irq, void *ispirq_disp)
{
	struct ispirq *irqdis = (struct ispirq *) ispirq_disp;
	u32 irqstatus = 0;
	unsigned long irqflags = 0;
	irqstatus = omap_readl(ISP_IRQ0STATUS);
	/* acknowledge the interrupt */
	omap_writel(irqstatus, ISP_IRQ0STATUS);       
	spin_lock_irqsave(&isp_obj.lock, irqflags);
	/*
	* If MMU error then flag this first via the set callback since
	* no recovery mechanism for MMU error
	*/
	if ((irqstatus & MMU_ERR) == MMU_ERR){
		if (irqdis->isp_callbk[4])
			irqdis->isp_callbk[4](irqstatus,
					irqdis->isp_callbk_arg1[4],
					irqdis->isp_callbk_arg2[4]);
		/* Inform the error for all the set callbacks */
		if (irqdis->isp_callbk[0])
			irqdis->isp_callbk[0](irqstatus,
					irqdis->isp_callbk_arg1[0],
					irqdis->isp_callbk_arg2[0]);
		if (irqdis->isp_callbk[1])
			irqdis->isp_callbk[1](irqstatus,
					irqdis->isp_callbk_arg1[1],
					irqdis->isp_callbk_arg2[1]);
		if (irqdis->isp_callbk[2])
			irqdis->isp_callbk[2](irqstatus,
					irqdis->isp_callbk_arg1[2],
					irqdis->isp_callbk_arg2[2]);
		if (irqdis->isp_callbk[3])
				irqdis->isp_callbk[3](irqstatus,
					irqdis->isp_callbk_arg1[3],
					irqdis->isp_callbk_arg2[3]);
		if (irqdis->isp_callbk[5])
				irqdis->isp_callbk[5](irqstatus,
					irqdis->isp_callbk_arg1[5],
					irqdis->isp_callbk_arg2[5]);
		if (irqdis->isp_callbk[6])
				irqdis->isp_callbk[6](irqstatus,
					irqdis->isp_callbk_arg1[6],
					irqdis->isp_callbk_arg2[6]);
		if (irqdis->isp_callbk[7])
				irqdis->isp_callbk[7](irqstatus,
					irqdis->isp_callbk_arg1[7],
					irqdis->isp_callbk_arg2[7]);
		
	} else {
	/* if an ISR is registered for this interface, execute the same */
		if ((irqstatus & CCDC_VD1) == CCDC_VD1)
			if (irqdis->isp_callbk[1])
				irqdis->isp_callbk[1](CCDC_VD1,
						irqdis->isp_callbk_arg1[1],
						irqdis->isp_callbk_arg2[1]);

		if ((irqstatus & CCDC_VD0) == CCDC_VD0)
			if (irqdis->isp_callbk[0])
				irqdis->isp_callbk[0](CCDC_VD0,
						irqdis->isp_callbk_arg1[0],
						irqdis->isp_callbk_arg2[0]);

		if ((irqstatus & PREV_DONE) == PREV_DONE)
			if (irqdis->isp_callbk[2])
				irqdis->isp_callbk[2](PREV_DONE,
						irqdis->isp_callbk_arg1[2],
						irqdis->isp_callbk_arg2[2]);

		if ((irqstatus & RESZ_DONE) == RESZ_DONE)
			if (irqdis->isp_callbk[3])
				irqdis->isp_callbk[3](RESZ_DONE,
						irqdis->isp_callbk_arg1[3],
						irqdis->isp_callbk_arg2[3]);

		if ((irqstatus & H3A_AWB_DONE) == H3A_AWB_DONE)
			if (irqdis->isp_callbk[5])
				irqdis->isp_callbk[5](H3A_AWB_DONE,
						irqdis->isp_callbk_arg1[5],
						irqdis->isp_callbk_arg2[5]);
						
		if ((irqstatus & HIST_DONE) == HIST_DONE)
			if (irqdis->isp_callbk[6])
			  irqdis->isp_callbk[6](HIST_DONE,
						irqdis->isp_callbk_arg1[6],
						irqdis->isp_callbk_arg2[6]);

		if ((irqstatus & HS_VS) == HS_VS)
			if (irqdis->isp_callbk[7])
			  irqdis->isp_callbk[7](HS_VS,
						irqdis->isp_callbk_arg1[7],
						irqdis->isp_callbk_arg2[7]);
	}
	
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);

	return IRQ_HANDLED;
}

void
isp_save_ctx(void)
{
			isp_save_context(isp_reg_list);
			ispccdc_save_context();
#ifndef CONFIG_ARCH_OMAP3410
			isphist_save_context();
			isph3a_save_context();
			isppreview_save_context();
			ispresizer_save_context();
#endif
			ispmmu_save_context();
}

void
isp_restore_ctx(void)
{
			isp_restore_context(isp_reg_list);
			omap34xxcam_sensor_restore();
 			ispccdc_restore_context();
#ifndef CONFIG_ARCH_OMAP3410
			isphist_restore_context();
			isph3a_restore_context();
			isppreview_restore_context();
			ispresizer_restore_context();
#endif
			ispmmu_restore_context();
}

/*
 * Acquires the ISP resource. Initialises the clocks for the first aquire.
 */
int
isp_get(void)
{
	int ret_err = 0;
	DPRINTK_ISPCTRL("isp_get: old %d\n", isp_obj.ref_count);
	down(&(isp_obj.semlock));
	if (isp_obj.ref_count == 0) {
		isp_obj.cam_ick = clk_get(NULL, "cam_ick");
		if(IS_ERR(isp_obj.cam_ick)){
			up(&(isp_obj.semlock));
			DPRINTK_ISPCTRL("ISP_ERR: clk_get for ick failed\n");
			return PTR_ERR(isp_obj.cam_ick);
		}
		isp_obj.cam_fck = clk_get(NULL, "cam_fck");
		if(IS_ERR(isp_obj.cam_fck)){
			up(&(isp_obj.semlock));
			DPRINTK_ISPCTRL("ISP_ERR: clk_get for fck failed\n");
			return PTR_ERR(isp_obj.cam_fck);
		}
		/* Cam IF Clk */
		ret_err = clk_enable(isp_obj.cam_ick);
		if(ret_err){
			up(&(isp_obj.semlock));
			clk_put(isp_obj.cam_ick);
			clk_put(isp_obj.cam_fck);
			DPRINTK_ISPCTRL("ISP_ERR: clk_en for ick failed\n");
			return ret_err;
		}
		/* Cam Func Clk */
		ret_err = clk_enable(isp_obj.cam_fck);
		if(ret_err){
			up(&(isp_obj.semlock));
			clk_put(isp_obj.cam_ick);
			clk_put(isp_obj.cam_fck);
			DPRINTK_ISPCTRL("ISP_ERR: clk_en for fck failed\n");
			return ret_err;
		}
		/* Context restore */
		if ( off_mode == 1 ) {
			isp_restore_ctx();
		}
	}
	isp_obj.ref_count++;
	up(&(isp_obj.semlock));
	
	
	DPRINTK_ISPCTRL("isp_get: new %d\n", isp_obj.ref_count);
	return isp_obj.ref_count;
}

/*
 * Releases the ISP resource. Releases the clocks also for the last release.
 */
int
isp_put(void)
{
	DPRINTK_ISPCTRL("isp_put: old %d\n", isp_obj.ref_count);
	down(&(isp_obj.semlock));
	if (isp_obj.ref_count)
		if (--isp_obj.ref_count == 0) {
			isp_save_ctx();
			off_mode = 1;
			
			/* Disable all interrupts */
			/* shut down ISP clocks */
			clk_disable(isp_obj.cam_ick);
			clk_disable(isp_obj.cam_fck);
			clk_put(isp_obj.cam_ick);
			clk_put(isp_obj.cam_fck);
		}
	up(&(isp_obj.semlock));
	DPRINTK_ISPCTRL("isp_put: new %d\n", isp_obj.ref_count);
	return isp_obj.ref_count;
}

/*
 * Saves the values of the ISP module registers.
 */
void 
isp_save_context(struct isp_reg* reg_list)
{	
	struct isp_reg *next = reg_list;
		
	for (; next->reg != ISP_TOK_TERM; next++) {
		next->val = omap_readl(next->reg);
	}
}

/*
 * Restores the values of the ISP module registers.
 */
void 
isp_restore_context(struct isp_reg* reg_list)
{
	struct isp_reg *next = reg_list;
		
	for (; next->reg != ISP_TOK_TERM; next++) {
		omap_writel(next->val, next->reg);
	}
}

/*
 * Module Initialisation.
 */
static int __init
isp_init(void)
{
	DPRINTK_ISPCTRL("+isp_init for Omap 3430 Camera ISP\n");
	isp_obj.ref_count = 0;

	init_MUTEX(&(isp_obj.semlock));
	if (request_irq(INT_34XX_CAM_IRQ, omap34xx_isp_isr, IRQF_SHARED,
		"Omap 34xx Camera ISP", &ispirq_obj)) {
		DPRINTK_ISPCTRL(" Could not install ISR\n");
		return -EINVAL;
	}
	else{
		spin_lock_init(&isp_obj.lock);
		DPRINTK_ISPCTRL("-isp_init for Omap 3430 Camera ISP\n");
		return 0;
	}
}

/*
 * Module Cleanup.
 */
static void __exit
isp_cleanup(void)
{
	free_irq(INT_34XX_CAM_IRQ,&ispirq_obj);
}

/*
 * Prints the values of the ISP Control Module registers
 * Also prints other debug information stored in the ISP module structure
 */
void
isp_print_status(void)
{
#ifdef	OMAP_ISPCTRL_DEBUG
	DPRINTK_ISPCTRL("###CM_FCLKEN_CAM=0x%x\n",
			omap_readl(CM_FCLKEN_CAM));
	DPRINTK_ISPCTRL("###CM_ICLKEN_CAM=0x%x\n",
			omap_readl(CM_ICLKEN_CAM));
	DPRINTK_ISPCTRL("###CM_CLKSEL_CAM=0x%x\n",
			omap_readl(CM_CLKSEL_CAM));
	DPRINTK_ISPCTRL("###CM_AUTOIDLE_CAM=0x%x\n",
			omap_readl(CM_AUTOIDLE_CAM));
	DPRINTK_ISPCTRL("###CM_CLKEN_PLL[18:16] \
			should be 0x7, =0x%x\n",
			omap_readl(CM_CLKEN_PLL));
	DPRINTK_ISPCTRL("###CM_CLKSEL2_PLL[18:8] should be 0x2D,\
			[6:0] should be 1=0x%x\n",
			omap_readl(CM_CLKSEL2_PLL));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_HS=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_HS));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_XCLKA=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_XCLKA));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D1=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_D1));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D3=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_D3));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D5=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_D5));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D7=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_D7));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D9=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_D9));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D11=0x%x\n",
			omap_readl(CTRL_PADCONF_CAM_D11));
#endif
}

module_init(isp_init);
module_exit(isp_cleanup);

EXPORT_SYMBOL(isp_set_callback);
EXPORT_SYMBOL(isp_unset_callback);
EXPORT_SYMBOL(isp_request_interface);
EXPORT_SYMBOL(isp_free_interface);
EXPORT_SYMBOL(isp_negotiate_xclka);
EXPORT_SYMBOL(isp_negotiate_xclkb);
EXPORT_SYMBOL(isp_set_xclka);
EXPORT_SYMBOL(isp_set_xclkb);
EXPORT_SYMBOL(isp_get_xclka);
EXPORT_SYMBOL(isp_get_xclkb);
EXPORT_SYMBOL(isp_configure_interface);
EXPORT_SYMBOL(isp_get);
EXPORT_SYMBOL(isp_put);
EXPORT_SYMBOL(isp_save_context);
EXPORT_SYMBOL(isp_restore_context);
EXPORT_SYMBOL(isp_power_settings);
EXPORT_SYMBOL(isp_print_status);


MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP Control Module Library");
MODULE_LICENSE("GPL");
