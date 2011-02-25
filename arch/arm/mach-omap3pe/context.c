/*
 * linux/arch/arm/mach-omap3pe/context.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on linux/arch/arm/mach-omap2 by
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * History:
 *
 */

#include <asm/arch/pm.h>
#include <asm/arch/gpio.h>
#include <asm/arch/prcm.h>

#include "prcm_pwr.h"
#include "context.h"
#include "serial.h"
#include "prcm-regs.h"
#include "prcm_util.h"
#include "prcm_slpwk.h"

#ifdef CONFIG_OMAP34XX_OFFMODE
static DEFINE_SPINLOCK(svres_reg_lock);

/* An array of struct which contains the context attributes for each domain.
 * NOTE:
 *   Domain ID indices are 1-based.
 */
struct domain_ctxsvres_status
{
	u32 context_saved;
	u32 context_restore;
};
static struct domain_ctxsvres_status domain_ctxsvres[PRCM_NUM_DOMAINS];

/*  This function is called by the DeviceDriver when it has done context save.
 */
void context_save_done(struct clk *clk)
{
	u32 prcm_id;
	u32 domain_id;
	u32 device_id;

	prcm_id = clk->prcmid;

	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);

	domain_ctxsvres[domain_id-1].context_saved |= (1 << device_id);
}
EXPORT_SYMBOL(context_save_done);


/*  This function is called by the DeviceDriver to check whether context restore
 *  is required.
 */
int context_restore_required(struct clk *clk)
{
	struct domain_ctxsvres_status *ctxsvres;
	u32 prcm_id, domain_id, device_id, device_bitpos;
	int ret;

	ret = 0;
	prcm_id = clk->prcmid;

	domain_id = DOMAIN_ID(prcm_id);
	device_id = DEV_BIT_POS(prcm_id);
	device_bitpos = 1 << device_id;

	spin_lock(&svres_reg_lock);
	ctxsvres = &domain_ctxsvres[domain_id-1];

	if (ctxsvres->context_restore & device_bitpos) {
		ret = 1;
		ctxsvres->context_saved &= ~device_bitpos;
		ctxsvres->context_restore &= ~device_bitpos;
	}
	spin_unlock(&svres_reg_lock);

	return ret;
}
EXPORT_SYMBOL(context_restore_required);

/*  This function is called to update the context attributes when power domain
 *  is put to OFF.
 */
void context_restore_update(u32 domain_id)
{
	/* NOTE: domain_id index is 1-based. */
	if ((0 == domain_id) || (domain_id > PRCM_NUM_DOMAINS)) {
		printk(KERN_WARNING "WARNING: %s called with illegal DOMAIN ID %d\n",
				__FUNCTION__, domain_id);
		return;
	}

	spin_lock(&svres_reg_lock);
	domain_ctxsvres[domain_id-1].context_restore =
			domain_ctxsvres[domain_id-1].context_saved;

	if (domain_id == DOM_CORE1) {
		domain_ctxsvres[DOM_CORE2-1].context_restore =
				domain_ctxsvres[DOM_CORE2-1].context_saved;
		domain_ctxsvres[DOM_CORE3-1].context_restore =
				domain_ctxsvres[DOM_CORE3-1].context_saved;
	}
	spin_unlock(&svres_reg_lock);
}
EXPORT_SYMBOL(context_restore_update);

#endif

/******************************************************************************
 *
 * Context save and restore functions
 *
 ******************************************************************************/

static struct gpmc_context		gpmc_ctx;
static struct control_module_context	control_ctx;
static struct int_controller_context	intc_context;
static struct neon_context		neon_ctx;
static struct usbtll_context		usbtll_ctx;

/******************************************************************************
 *
 * SAVE/RESTORE GPMC context.
 *
 ******************************************************************************/

static void gpmc_save_context(void)
{
	gpmc_ctx.sysconfig		= GPMC_SYS_CONFIG;
	gpmc_ctx.irqenable		= GPMC_IRQ_ENABLE;
	gpmc_ctx.timeout_ctrl		= GPMC_TIMEOUT_CONTROL;
	gpmc_ctx.config			= GPMC_CFG;
	gpmc_ctx.prefetch_config1	= GPMC_PREFETCH_CONFIG_1;
	gpmc_ctx.prefetch_config2	= GPMC_PREFETCH_CONFIG_2;
	gpmc_ctx.prefetch_control	= GPMC_PREFETCH_CTRL;
	gpmc_ctx.cs0_context.cs_valid	= GPMC_CONFIG7_0 & (1 << 6);

	if (gpmc_ctx.cs0_context.cs_valid) {
		gpmc_ctx.cs0_context.config1 = GPMC_CONFIG1_0;
		gpmc_ctx.cs0_context.config2 = GPMC_CONFIG2_0;
		gpmc_ctx.cs0_context.config3 = GPMC_CONFIG3_0;
		gpmc_ctx.cs0_context.config4 = GPMC_CONFIG4_0;
		gpmc_ctx.cs0_context.config5 = GPMC_CONFIG5_0;
		gpmc_ctx.cs0_context.config6 = GPMC_CONFIG6_0;
		gpmc_ctx.cs0_context.config7 = GPMC_CONFIG7_0;
	}
	gpmc_ctx.cs1_context.cs_valid = GPMC_CONFIG7_1 & (1 << 6);
	if (gpmc_ctx.cs1_context.cs_valid) {
		gpmc_ctx.cs1_context.config1 = GPMC_CONFIG1_1;
		gpmc_ctx.cs1_context.config2 = GPMC_CONFIG2_1;
		gpmc_ctx.cs1_context.config3 = GPMC_CONFIG3_1;
		gpmc_ctx.cs1_context.config4 = GPMC_CONFIG4_1;
		gpmc_ctx.cs1_context.config5 = GPMC_CONFIG5_1;
		gpmc_ctx.cs1_context.config6 = GPMC_CONFIG6_1;
		gpmc_ctx.cs1_context.config7 = GPMC_CONFIG7_1;
	}
	gpmc_ctx.cs2_context.cs_valid = GPMC_CONFIG7_2 & (1 << 6);
	if (gpmc_ctx.cs2_context.cs_valid) {
		gpmc_ctx.cs2_context.config1 = GPMC_CONFIG1_2;
		gpmc_ctx.cs2_context.config2 = GPMC_CONFIG2_2;
		gpmc_ctx.cs2_context.config3 = GPMC_CONFIG3_2;
		gpmc_ctx.cs2_context.config4 = GPMC_CONFIG4_2;
		gpmc_ctx.cs2_context.config5 = GPMC_CONFIG5_2;
		gpmc_ctx.cs2_context.config6 = GPMC_CONFIG6_2;
		gpmc_ctx.cs2_context.config7 = GPMC_CONFIG7_2;
	}
	gpmc_ctx.cs3_context.cs_valid = GPMC_CONFIG7_3 & (1 << 6);
	if (gpmc_ctx.cs3_context.cs_valid) {
		gpmc_ctx.cs3_context.config1 = GPMC_CONFIG1_3;
		gpmc_ctx.cs3_context.config2 = GPMC_CONFIG2_3;
		gpmc_ctx.cs3_context.config3 = GPMC_CONFIG3_3;
		gpmc_ctx.cs3_context.config4 = GPMC_CONFIG4_3;
		gpmc_ctx.cs3_context.config5 = GPMC_CONFIG5_3;
		gpmc_ctx.cs3_context.config6 = GPMC_CONFIG6_3;
		gpmc_ctx.cs3_context.config7 = GPMC_CONFIG7_3;
	}
	gpmc_ctx.cs4_context.cs_valid = GPMC_CONFIG7_4 & (1 << 6);
	if (gpmc_ctx.cs4_context.cs_valid) {
		gpmc_ctx.cs4_context.config1 = GPMC_CONFIG1_4;
		gpmc_ctx.cs4_context.config2 = GPMC_CONFIG2_4;
		gpmc_ctx.cs4_context.config3 = GPMC_CONFIG3_4;
		gpmc_ctx.cs4_context.config4 = GPMC_CONFIG4_4;
		gpmc_ctx.cs4_context.config5 = GPMC_CONFIG5_4;
		gpmc_ctx.cs4_context.config6 = GPMC_CONFIG6_4;
		gpmc_ctx.cs4_context.config7 = GPMC_CONFIG7_4;
	}
	gpmc_ctx.cs5_context.cs_valid = GPMC_CONFIG7_5 & (1 << 6);
	if (gpmc_ctx.cs5_context.cs_valid) {
		gpmc_ctx.cs5_context.config1 = GPMC_CONFIG1_5;
		gpmc_ctx.cs5_context.config2 = GPMC_CONFIG2_5;
		gpmc_ctx.cs5_context.config3 = GPMC_CONFIG3_5;
		gpmc_ctx.cs5_context.config4 = GPMC_CONFIG4_5;
		gpmc_ctx.cs5_context.config5 = GPMC_CONFIG5_5;
		gpmc_ctx.cs5_context.config6 = GPMC_CONFIG6_5;
		gpmc_ctx.cs5_context.config7 = GPMC_CONFIG7_5;
	}
	gpmc_ctx.cs6_context.cs_valid = GPMC_CONFIG7_6 & (1 << 6);
	if (gpmc_ctx.cs6_context.cs_valid) {
		gpmc_ctx.cs6_context.config1 = GPMC_CONFIG1_6;
		gpmc_ctx.cs6_context.config2 = GPMC_CONFIG2_6;
		gpmc_ctx.cs6_context.config3 = GPMC_CONFIG3_6;
		gpmc_ctx.cs6_context.config4 = GPMC_CONFIG4_6;
		gpmc_ctx.cs6_context.config5 = GPMC_CONFIG5_6;
		gpmc_ctx.cs6_context.config6 = GPMC_CONFIG6_6;
		gpmc_ctx.cs6_context.config7 = GPMC_CONFIG7_6;
	}
	gpmc_ctx.cs7_context.cs_valid = GPMC_CONFIG7_7 & (1 << 6);
	if (gpmc_ctx.cs7_context.cs_valid) {
		gpmc_ctx.cs7_context.config1 = GPMC_CONFIG1_7;
		gpmc_ctx.cs7_context.config2 = GPMC_CONFIG2_7;
		gpmc_ctx.cs7_context.config3 = GPMC_CONFIG3_7;
		gpmc_ctx.cs7_context.config4 = GPMC_CONFIG4_7;
		gpmc_ctx.cs7_context.config5 = GPMC_CONFIG5_7;
		gpmc_ctx.cs7_context.config6 = GPMC_CONFIG6_7;
		gpmc_ctx.cs7_context.config7 = GPMC_CONFIG7_7;
	}
}

static void gpmc_restore_context(void)
{
	GPMC_SYS_CONFIG		= gpmc_ctx.sysconfig;
	GPMC_IRQ_ENABLE		= gpmc_ctx.irqenable;
	GPMC_TIMEOUT_CONTROL	= gpmc_ctx.timeout_ctrl;
	GPMC_CFG		= gpmc_ctx.config;
	GPMC_PREFETCH_CONFIG_1	= gpmc_ctx.prefetch_config1;
	GPMC_PREFETCH_CONFIG_2	= gpmc_ctx.prefetch_config2;
	GPMC_PREFETCH_CTRL	= gpmc_ctx.prefetch_control;

	if (gpmc_ctx.cs0_context.cs_valid) {
		GPMC_CONFIG1_0 = gpmc_ctx.cs0_context.config1;
		GPMC_CONFIG2_0 = gpmc_ctx.cs0_context.config2;
		GPMC_CONFIG3_0 = gpmc_ctx.cs0_context.config3;
		GPMC_CONFIG4_0 = gpmc_ctx.cs0_context.config4;
		GPMC_CONFIG5_0 = gpmc_ctx.cs0_context.config5;
		GPMC_CONFIG6_0 = gpmc_ctx.cs0_context.config6;
		GPMC_CONFIG7_0 = gpmc_ctx.cs0_context.config7;
	}
	if (gpmc_ctx.cs1_context.cs_valid) {
		GPMC_CONFIG1_1 = gpmc_ctx.cs1_context.config1;
		GPMC_CONFIG2_1 = gpmc_ctx.cs1_context.config2;
		GPMC_CONFIG3_1 = gpmc_ctx.cs1_context.config3;
		GPMC_CONFIG4_1 = gpmc_ctx.cs1_context.config4;
		GPMC_CONFIG5_1 = gpmc_ctx.cs1_context.config5;
		GPMC_CONFIG6_1 = gpmc_ctx.cs1_context.config6;
		GPMC_CONFIG7_1 = gpmc_ctx.cs1_context.config7;
	}
	if (gpmc_ctx.cs2_context.cs_valid) {
		GPMC_CONFIG1_2 = gpmc_ctx.cs2_context.config1;
		GPMC_CONFIG2_2 = gpmc_ctx.cs2_context.config2;
		GPMC_CONFIG3_2 = gpmc_ctx.cs2_context.config3;
		GPMC_CONFIG4_2 = gpmc_ctx.cs2_context.config4;
		GPMC_CONFIG5_2 = gpmc_ctx.cs2_context.config5;
		GPMC_CONFIG6_2 = gpmc_ctx.cs2_context.config6;
		GPMC_CONFIG7_2 = gpmc_ctx.cs2_context.config7;
	}
	if (gpmc_ctx.cs3_context.cs_valid) {
		GPMC_CONFIG1_3 = gpmc_ctx.cs3_context.config1;
		GPMC_CONFIG2_3 = gpmc_ctx.cs3_context.config2;
		GPMC_CONFIG3_3 = gpmc_ctx.cs3_context.config3;
		GPMC_CONFIG4_3 = gpmc_ctx.cs3_context.config4;
		GPMC_CONFIG5_3 = gpmc_ctx.cs3_context.config5;
		GPMC_CONFIG6_3 = gpmc_ctx.cs3_context.config6;
		GPMC_CONFIG7_3 = gpmc_ctx.cs3_context.config7;
	}
	if (gpmc_ctx.cs4_context.cs_valid) {
		GPMC_CONFIG1_4 = gpmc_ctx.cs4_context.config1;
		GPMC_CONFIG2_4 = gpmc_ctx.cs4_context.config2;
		GPMC_CONFIG3_4 = gpmc_ctx.cs4_context.config3;
		GPMC_CONFIG4_4 = gpmc_ctx.cs4_context.config4;
		GPMC_CONFIG5_4 = gpmc_ctx.cs4_context.config5;
		GPMC_CONFIG6_4 = gpmc_ctx.cs4_context.config6;
		GPMC_CONFIG7_4 = gpmc_ctx.cs4_context.config7;
	}
	if (gpmc_ctx.cs5_context.cs_valid) {
		GPMC_CONFIG1_5 = gpmc_ctx.cs5_context.config1;
		GPMC_CONFIG2_5 = gpmc_ctx.cs5_context.config2;
		GPMC_CONFIG3_5 = gpmc_ctx.cs5_context.config3;
		GPMC_CONFIG4_5 = gpmc_ctx.cs5_context.config4;
		GPMC_CONFIG5_5 = gpmc_ctx.cs5_context.config5;
		GPMC_CONFIG6_5 = gpmc_ctx.cs5_context.config6;
		GPMC_CONFIG7_5 = gpmc_ctx.cs5_context.config7;
	}
	if (gpmc_ctx.cs6_context.cs_valid) {
		GPMC_CONFIG1_6 = gpmc_ctx.cs6_context.config1;
		GPMC_CONFIG2_6 = gpmc_ctx.cs6_context.config2;
		GPMC_CONFIG3_6 = gpmc_ctx.cs6_context.config3;
		GPMC_CONFIG4_6 = gpmc_ctx.cs6_context.config4;
		GPMC_CONFIG5_6 = gpmc_ctx.cs6_context.config5;
		GPMC_CONFIG6_6 = gpmc_ctx.cs6_context.config6;
		GPMC_CONFIG7_6 = gpmc_ctx.cs6_context.config7;
	}
	if (gpmc_ctx.cs7_context.cs_valid) {
		GPMC_CONFIG1_7 = gpmc_ctx.cs7_context.config1;
		GPMC_CONFIG2_7 = gpmc_ctx.cs7_context.config2;
		GPMC_CONFIG3_7 = gpmc_ctx.cs7_context.config3;
		GPMC_CONFIG4_7 = gpmc_ctx.cs7_context.config4;
		GPMC_CONFIG5_7 = gpmc_ctx.cs7_context.config5;
		GPMC_CONFIG6_7 = gpmc_ctx.cs7_context.config6;
		GPMC_CONFIG7_7 = gpmc_ctx.cs7_context.config7;
	}
}

/******************************************************************************
 *
 * SAVE/RESTORE Control Module Context
 *
 ******************************************************************************/

static void control_save_context(void)
{
	control_ctx.sysconfig		= CONTROL_SYSCONFIG;
	control_ctx.devconf0		= CONTROL_DEVCONF0;
	control_ctx.mem_dftrw0		= CONTROL_MEM_DFTRW0;
	control_ctx.mem_dftrw1		= CONTROL_MEM_DFTRW1;
	control_ctx.msuspendmux_0	= CONTROL_MSUSPENDMUX_0;
	control_ctx.msuspendmux_1	= CONTROL_MSUSPENDMUX_1;
	control_ctx.msuspendmux_2	= CONTROL_MSUSPENDMUX_2;
	control_ctx.msuspendmux_3	= CONTROL_MSUSPENDMUX_3;
	control_ctx.msuspendmux_4	= CONTROL_MSUSPENDMUX_4;
	control_ctx.msuspendmux_5	= CONTROL_MSUSPENDMUX_5;
	control_ctx.sec_ctrl		= CONTROL_SEC_CTRL;
	control_ctx.devconf1		= CONTROL_DEVCONF1;
	control_ctx.csirxfe		= CONTROL_CSIRXFE;
	control_ctx.iva2_bootaddr	= CONTROL_IVA2_BOOTADDR;
	control_ctx.iva2_bootmod	= CONTROL_IVA2_BOOTMOD;
	control_ctx.debobs_0		= CONTROL_DEBOBS_0;
	control_ctx.debobs_1		= CONTROL_DEBOBS_1;
	control_ctx.debobs_2		= CONTROL_DEBOBS_2;
	control_ctx.debobs_3		= CONTROL_DEBOBS_3;
	control_ctx.debobs_4		= CONTROL_DEBOBS_4;
	control_ctx.debobs_5		= CONTROL_DEBOBS_5;
	control_ctx.debobs_6		= CONTROL_DEBOBS_6;
	control_ctx.debobs_7		= CONTROL_DEBOBS_7;
	control_ctx.debobs_8		= CONTROL_DEBOBS_8;
	control_ctx.prog_io0		= CONTROL_PROG_IO0;
	control_ctx.prog_io1		= CONTROL_PROG_IO1;
	control_ctx.dss_dpll_spreading	= CONTROL_DSS_DPLL_SPREADING;
	control_ctx.core_dpll_spreading	= CONTROL_CORE_DPLL_SPREADING;
	control_ctx.per_dpll_spreading	= CONTROL_PER_DPLL_SPREADING;
	control_ctx.usbhost_dpll_spreading = CONTROL_USBHOST_DPLL_SPREADING;
	control_ctx.pbias_lite		= CONTROL_PBIAS_1;
	control_ctx.temp_sensor		= CONTROL_TEMP_SENSOR;
	control_ctx.sramldo4		= CONTROL_SRAMLDO4;
	control_ctx.sramldo5		= CONTROL_SRAMLDO5;
	control_ctx.csi			= CONTROL_CSI;
}

static void control_restore_context(void)
{
	CONTROL_SYSCONFIG		= control_ctx.sysconfig;
	CONTROL_DEVCONF0		= control_ctx.devconf0;
	CONTROL_MEM_DFTRW0		= control_ctx.mem_dftrw0;
	CONTROL_MEM_DFTRW1		= control_ctx.mem_dftrw1;
	CONTROL_MSUSPENDMUX_0		= control_ctx.msuspendmux_0;
	CONTROL_MSUSPENDMUX_1		= control_ctx.msuspendmux_1;
	CONTROL_MSUSPENDMUX_2		= control_ctx.msuspendmux_2;
	CONTROL_MSUSPENDMUX_3		= control_ctx.msuspendmux_3;
	CONTROL_MSUSPENDMUX_4		= control_ctx.msuspendmux_4;
	CONTROL_MSUSPENDMUX_5		= control_ctx.msuspendmux_5;
	CONTROL_SEC_CTRL		= control_ctx.sec_ctrl;
	CONTROL_DEVCONF1		= control_ctx.devconf1;
	CONTROL_CSIRXFE			= control_ctx.csirxfe;
	CONTROL_IVA2_BOOTADDR		= control_ctx.iva2_bootaddr;
	CONTROL_IVA2_BOOTMOD		= control_ctx.iva2_bootmod;
	CONTROL_DEBOBS_0		= control_ctx.debobs_0;
	CONTROL_DEBOBS_1		= control_ctx.debobs_1;
	CONTROL_DEBOBS_2		= control_ctx.debobs_2;
	CONTROL_DEBOBS_3		= control_ctx.debobs_3;
	CONTROL_DEBOBS_4		= control_ctx.debobs_4;
	CONTROL_DEBOBS_5		= control_ctx.debobs_5;
	CONTROL_DEBOBS_6		= control_ctx.debobs_6;
	CONTROL_DEBOBS_7		= control_ctx.debobs_7;
	CONTROL_DEBOBS_8		= control_ctx.debobs_8;
	CONTROL_PROG_IO0		= control_ctx.prog_io0;
	CONTROL_PROG_IO1		= control_ctx.prog_io1;
	CONTROL_DSS_DPLL_SPREADING	= control_ctx.dss_dpll_spreading;
	CONTROL_CORE_DPLL_SPREADING	= control_ctx.core_dpll_spreading;
	CONTROL_PER_DPLL_SPREADING	= control_ctx.per_dpll_spreading;
	CONTROL_USBHOST_DPLL_SPREADING	= control_ctx.usbhost_dpll_spreading;
	CONTROL_PBIAS_1			= control_ctx.pbias_lite;
	CONTROL_TEMP_SENSOR		= control_ctx.temp_sensor;
	CONTROL_SRAMLDO4		= control_ctx.sramldo4;
	CONTROL_SRAMLDO5		= control_ctx.sramldo5;
	CONTROL_CSI			= control_ctx.csi;
}

/******************************************************************************
 *
 * SAVE/RESTORE INTC Module Context
 *
 ******************************************************************************/

static void save_intc_context(void)
{
	int i = 0;
	intc_context.sysconfig	= INTCPS_SYSCONFIG;
	intc_context.protection	= INTCPS_PROTECTION;
	intc_context.idle	= INTCPS_IDLE;
	intc_context.threshold	= INTCPS_THRESHOLD;
	for (i = 0; i < 96; i++)
		intc_context.ilr[i] = IC_REG32_34XX(0x100 + 0x4*i);
	/* MIRs are saved and restore with other PRCM registers */
}

static void restore_intc_context(void)
{
	int i = 0;
	INTCPS_SYSCONFIG	= intc_context.sysconfig;
	INTCPS_PROTECTION	= intc_context.protection;
	INTCPS_IDLE		= intc_context.idle;
	INTCPS_THRESHOLD	= intc_context.threshold;
	for (i = 0; i < 96; i++)
		IC_REG32_34XX(0x100 + 0x4*i) = intc_context.ilr[i];
	/* MIRs are saved and restore with other PRCM registers */
}

/******************************************************************************
 *
 * SAVE/RESTORE USBTLL Module Context
 *
 ******************************************************************************/

#define USBTLL_MAX_CHANNELS	3

static void save_usbtll_context(void)
{
	int i;
	int old_iclk;
	int old_fclk;
	int ret;

	old_fclk = CM_FCLKEN3_CORE & 0x4;
	old_iclk = CM_ICLKEN3_CORE & 0x4;

	if (!old_fclk)
		CM_FCLKEN3_CORE |= 0x4;
	if (!old_iclk)
		CM_ICLKEN3_CORE |= 0x4;

	ret = WAIT_WHILE(CM_IDLEST3_CORE & 0x4, 1000);
	if (ret < 0) {
		printk(KERN_ERR "%s: can't enable clocks\n", __func__);
		return;
	}

	usbtll_ctx.usbtll_sysconfig = USBTLL_SYSCONFIG;
	usbtll_ctx.usbtll_irqenable = USBTLL_IRQENABLE;
	usbtll_ctx.tll_shared_conf = USBTLL_SHARED_CONF;

	for (i = 0; i < USBTLL_MAX_CHANNELS; i++) {
		usbtll_ctx.tll_channel_conf[i] =
			USBTLL_CHANNEL_CONF(i);
		usbtll_ctx.ulpi_function_ctrl[i] =
			USBTLL_ULPI_FUNCTION_CTRL(i);
		usbtll_ctx.ulpi_interface_ctrl[i] =
			USBTLL_ULPI_INTERFACE_CTRL(i);
		usbtll_ctx.ulpi_otg_ctrl[i] =
			USBTLL_ULPI_OTG_CTRL(i);
		usbtll_ctx.ulpi_usb_int_en_rise[i] =
			USBTLL_ULPI_INT_EN_RISE(i);
		usbtll_ctx.ulpi_usb_int_en_fall[i] =
			USBTLL_ULPI_INT_EN_FALL(i);
		usbtll_ctx.ulpi_usb_int_status[i] =
			USBTLL_ULPI_INT_STATUS(i);
	}

	if (!old_iclk)
		CM_ICLKEN3_CORE &= ~0x4;
	if (!old_fclk)
		CM_FCLKEN3_CORE &= ~0x4;

	ret = WAIT_UNTIL(CM_IDLEST3_CORE & 0x4, 1000);
	if (ret < 0) {
		printk(KERN_ERR "%s: can't disable clocks\n", __func__);
		return;
	}
}

static void restore_usbtll_context(void)
{
	int i;
	int old_iclk;
	int old_fclk;
	int ret;

	old_fclk = CM_FCLKEN3_CORE & 0x4;
	old_iclk = CM_ICLKEN3_CORE & 0x4;

	if (!old_fclk)
		CM_FCLKEN3_CORE |= 0x4;
	if (!old_iclk)
		CM_ICLKEN3_CORE |= 0x4;

	ret = WAIT_WHILE(CM_IDLEST3_CORE & 0x4, 1000);
	if (ret < 0) {
		printk(KERN_ERR "%s: can't enable clocks\n", __func__);
		return;
	}

//	USBTLL_SYSCONFIG = 0x2; /* soft reset */
//	WAIT_UNTIL(USBTLL_SYSSTATUS & 0x1, 1000); /* wait until reset done */

	USBTLL_SYSCONFIG = usbtll_ctx.usbtll_sysconfig;
	USBTLL_SHARED_CONF = usbtll_ctx.tll_shared_conf;

	for (i = 0; i < USBTLL_MAX_CHANNELS; i++) {
		USBTLL_CHANNEL_CONF(i) =
			usbtll_ctx.tll_channel_conf[i];
		USBTLL_ULPI_INTERFACE_CTRL(i) =
			usbtll_ctx.ulpi_interface_ctrl[i];
		USBTLL_ULPI_FUNCTION_CTRL(i) =
			usbtll_ctx.ulpi_function_ctrl[i];
		USBTLL_ULPI_OTG_CTRL(i) =
			usbtll_ctx.ulpi_otg_ctrl[i];
		USBTLL_ULPI_INT_EN_RISE(i) =
			usbtll_ctx.ulpi_usb_int_en_rise[i];
		USBTLL_ULPI_INT_EN_FALL(i) =
			usbtll_ctx.ulpi_usb_int_en_fall[i];
		USBTLL_ULPI_INT_STATUS(i) =
			usbtll_ctx.ulpi_usb_int_status[i];
	}
	USBTLL_IRQENABLE = usbtll_ctx.usbtll_irqenable;

	if (!old_iclk)
		CM_ICLKEN3_CORE &= ~0x4;
	if (!old_fclk)
		CM_FCLKEN3_CORE &= ~0x4;

	ret = WAIT_UNTIL(CM_IDLEST3_CORE & 0x4, 1000);
	if (ret < 0) {
		printk(KERN_ERR "%s: can't disable clocks\n", __func__);
		return;
	}
}

/******************************************************************************
 *
 * SAVE/RESTORE Core Context
 *
 ******************************************************************************/

void prcm_save_core_context(u32 target_core_state)
{
	if (target_core_state == PRCM_CORE_OFF) {
		/* Save interrupt controller context */
		save_intc_context();
		if (is_sil_rev_less_than(OMAP3430_REV_ES3_0)) {
			/* Save usbtll context */
			save_usbtll_context();
		}
		/* Save gpmc context */
		gpmc_save_context();
		/* Save control module context */
		control_save_context();
	}
	else if (target_core_state >= PRCM_CORE_OSWR_MEMRET) {
		/* Save interrupt controller context */
		save_intc_context();
		if (is_sil_rev_less_than(OMAP3430_REV_ES3_0)) {
			/* Save usbtll context */
			save_usbtll_context();
		}
		/* Save gpmc context */
		gpmc_save_context();
	}
}

void prcm_restore_core_context(u32 target_core_state)
{
	u8 state;
	if (target_core_state == PRCM_CORE_OFF) {
		prcm_get_pre_power_domain_state(DOM_CORE1, &state);
		if (state == PRCM_OFF) {
			restore_intc_context();
			gpmc_restore_context();
			control_restore_context();

			prcm_set_domain_power_configuration(DOM_CORE1,
				PRCM_SIDLEMODE_DONTCARE,
				PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);

			/* Since PER is also handled along with CORE
			* enable autoidle for PER also
			*/
			prcm_set_domain_power_configuration(DOM_PER,
				PRCM_SIDLEMODE_DONTCARE,
				PRCM_MIDLEMODE_DONTCARE, PRCM_TRUE);
			/* Lock DPLL5 */
			prcm_configure_dpll(DPLL5_PER2, -1, -1, -1);
			prcm_enable_dpll(DPLL5_PER2);
			if (is_sil_rev_less_than(OMAP3430_REV_ES3_0)) {
				/* Needs to be after DPLL5 is locked */
				restore_usbtll_context();
			}
		}
	}

	if ((target_core_state >= PRCM_CORE_OSWR_MEMRET) &&
	    (target_core_state != PRCM_CORE_OFF)) {
		restore_intc_context();
		if (is_sil_rev_less_than(OMAP3430_REV_ES3_0)) {
			restore_usbtll_context();
		}
		gpmc_restore_context();
	}
}

/******************************************************************************
 *
 * SAVE/RESTORE NEON Module Context
 *
 ******************************************************************************/
#define read_fpscr() ({ \
	unsigned int __fpscr; \
	asm volatile("mrc p10, 7, %0, cr1, cr0, 0" : "=r" (__fpscr) : : "cc"); \
	__fpscr; \
	})

#define read_fpexc() ({ \
	unsigned int __fpexc; \
	asm volatile("mrc p10, 7, %0, cr8, cr0, 0" : "=r" (__fpexc) : : "cc"); \
	__fpexc; \
	})

#define write_fpscr(val) asm volatile ("mcr p10, 7, %0, cr1, cr0, 0" :: "r" (val) : "cc")
#define write_fpexc(val) asm volatile ("mcr p10, 7, %0, cr8, cr0, 0" :: "r" (val) : "cc")

#define save_neon_regs(addr) asm volatile ("stc p11, cr0, [%0], #32*4; stcl p11, cr0, [%0], #32*4;" :: "r" (addr));
#define load_neon_regs(addr) asm volatile ("ldc p11, cr0, [%0], #32*4; ldcl p11, cr0, [%0], #32*4;" :: "r" (addr));

void omap3_save_neon_context(void)
{
#ifdef CONFIG_NEON
	neon_ctx.fpexc = read_fpexc();

	/* make sure vfp/neon is enabled so we can access the registers */
	write_fpexc(neon_ctx.fpexc | (1<<30));

	neon_ctx.fpscr = read_fpscr();

	/* save the main state */
	save_neon_regs(neon_ctx.fpregs);
#endif

	return;
}

void omap3_restore_neon_context(void)
{
#ifdef CONFIG_VFP
	vfp_enable();
#endif

#ifdef CONFIG_NEON
	/* enable vfp */
	write_fpexc(read_fpexc() | (1<<30));

	/* load the main register state */
	load_neon_regs(neon_ctx.fpregs);

	/* restore the vfp/neon control registers */
	write_fpscr(neon_ctx.fpscr);
	write_fpexc(neon_ctx.fpexc);
#endif
}

/******************************************************************************
 *
 * SAVE/RESTORE PER Modules Context
 *
 ******************************************************************************/

void omap3_save_per_context(void)
{
	omap_gpio_save();
	omap_uart_save_ctx(2);
}

void omap3_restore_per_context(void)
{
	omap_gpio_restore();
	prcm_wait_for_clock(PRCM_UART3);
	omap_uart_restore_ctx(2);
}



