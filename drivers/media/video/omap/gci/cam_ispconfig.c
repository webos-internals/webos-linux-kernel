/*
 * drivers/media/video/omap/gci/cam_ispconfig.c
 *
 * Camera configurations for TI's OMAP3430 Camera ISP
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

#include <asm/types.h>
#include <asm/mach-types.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/io.h>

#include <asm/arch/gci/isc_cmd.h>

#include "../isp/ispreg.h"
#include "../isp/isp.h"
#include "../isp/ispccdc.h"
#include "../isp/isppreview.h"
#include "../isp/ispresizer.h"
#include "../isp/ispmmu.h"
#include "../isp/isph3a.h"

#define CM_CAM_MCLK_HZ		216000000


/* Sets the divisor for xclka, xclkb */
int 
camispcfg_set_xclk(unsigned int xclka_b,unsigned int div)
{
	unsigned int xclk_div;
	isp_get();
	if(xclka_b == 1){

		xclk_div = isp_negotiate_xclka(div);
		isp_set_xclka(CM_CAM_MCLK_HZ/xclk_div);
	}
	else if(xclka_b == 2){
		xclk_div = isp_negotiate_xclkb(div);
		isp_set_xclkb(xclk_div);
	}
	isp_put();
}

/* Sets the sensor related settings in ISPCTRL registers */
int 
camispcfg_set_ispif(unsigned char addr, unsigned int val)
{
	unsigned int old_val;
	isp_get();
	switch(addr)
	{
	case GET_ADDRESS(CAM_ISPIF_CTRL):
	{
		old_val = omap_readl(ISP_CTRL);
		old_val &= ~CAM_ISPIF_CTRL_MASK;
		val &= CAM_ISPIF_CTRL_MASK;
		omap_writel(old_val | val,ISP_CTRL);
		printk("oldval = 0x%x, val = 0x%x", old_val, val);
		printk(" ispctrl val = 0x%x", omap_readl(ISP_CTRL));
	}
	break;
	default:
	break;
	};
	isp_put();
	return 0;
}

/* Sets the sensor related settings in CCDC registers */
int 
camispcfg_set_ccdc(unsigned char addr, unsigned int val)
{
	unsigned int old_val;
	isp_get();
	switch(addr)
	{
	case GET_ADDRESS(CAM_CCDC_SYNCMODE):
	{
		old_val = omap_readl(ISPCCDC_SYN_MODE);
		old_val &= ~CAM_CCDC_SYNCMODE_MASK;
		val &= CAM_CCDC_SYNCMODE_MASK;
		omap_writel(old_val | val,ISPCCDC_SYN_MODE);
		printk("oldval = 0x%x, val = 0x%x", old_val, val);
		printk(" ccdc sync mode val = 0x%x", omap_readl(ISPCCDC_SYN_MODE));
	}
	break;
	default:
	break;
	};
	isp_put();
	return 0;
}


