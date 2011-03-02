/*
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP3 Power Management Routines
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/cpuidle.h>
#include <linux/suspend.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <linux/mm.h>
#include <asm/mmu.h>
#include <asm/tlbflush.h>

#include <asm/arch/irqs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sram.h>
#include <asm/arch/pm.h>
#include <linux/tick.h>

#ifdef CONFIG_CPU_IDLE
#include "pm_cpuidle.h"
#endif

#include "prcm-regs.h"
#include "prcm_opp.h"
#include "prcm_pwr.h"
#include "prcm_util.h"
#include "sram_idle.h"
#include "serial.h"
#include "context.h"
#include "scratchpad.h"

//#define DEBUG_HW_SUP 1
#define PRCM_MPU_IRQ		11	/* MPU IRQ 11 */

/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

/* #define DEBUG_PM 1 */
#ifdef DEBUG_PM
# define DPRINTK(fmt, args...) \
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

#define printd(args...) printk(args)
#ifndef printd
#  define printd(args...)
#endif

/******************************************************************************
 *
 * Globals
 *
 ******************************************************************************/

static struct res_handle *memret1;
static struct res_handle *memret2;
static struct res_handle *logret1;

/******************************************************************************
 *
 * Misc
 *
 ******************************************************************************/

void memory_logic_res_setting(struct system_power_state *trg_st)
{
	int res1_level = resource_get_level(memret1);
	int res2_level = resource_get_level(memret2);
	int res3_level = resource_get_level(logret1);

	if (res3_level == LOGIC_RET) {
		if ((res1_level == MEMORY_RET) && (res2_level == MEMORY_RET))
			trg_st->core_state = PRCM_CORE_CSWR_MEMRET;
		else if (res1_level == MEMORY_OFF && res2_level == MEMORY_RET)
			trg_st->core_state = PRCM_CORE_CSWR_MEM1OFF;
		else if (res1_level == MEMORY_RET && res2_level == MEMORY_OFF)
			trg_st->core_state = PRCM_CORE_CSWR_MEM2OFF;
		else
			trg_st->core_state = PRCM_CORE_CSWR_MEMOFF;
	} else if (res3_level == LOGIC_OFF) {
		if ((res1_level && res2_level) == MEMORY_RET)
			trg_st->core_state = PRCM_CORE_OSWR_MEMRET;
		else if (res1_level == MEMORY_OFF && res2_level == MEMORY_RET)
			trg_st->core_state = PRCM_CORE_OSWR_MEM1OFF;
		else if (res1_level == MEMORY_RET && res2_level == MEMORY_OFF)
			trg_st->core_state = PRCM_CORE_OSWR_MEM2OFF;
		else
			trg_st->core_state = PRCM_CORE_OSWR_MEMOFF;
	} else
		DPRINTK("Current State not supported in Retention");
}

/******************************************************************************
 *
 * SUSPEND entry point
 *
 ******************************************************************************/

#ifdef CONFIG_CORE_OFF
void omap3_restore_core_settings(void)
{
	prcm_lock_iva_dpll(prcm_get_current_vdd1_opp());
	restore_sram_functions();
	omap_sram_idle_setup();
	save_scratchpad_contents();
}
#endif

#ifdef CONFIG_PM

/******************************************************************************
 *
 * PM operations implemented per target. Will be called in valid, prepare,
 * enter and finish functions.
 *
 ******************************************************************************/

static struct platform_suspend_ops *omap3_pm_target_ops = 0;

int omap3_pm_register_target_pm_ops(struct platform_suspend_ops *ops)
{
	if (!ops)
		return -1;

	omap3_pm_target_ops = ops;
	return 0;
}

/******************************************************************************
 *
 * System SUSPEND
 *
 ******************************************************************************/

static int omap3_pm_suspend(void)
{
	struct system_power_state trg_st;
	u32 suspend_state;
	int ret;

	disable_smartreflex(SR1_ID);
	disable_smartreflex(SR2_ID);

#ifdef CONFIG_OMAP34XX_OFFMODE
	context_restore_update(DOM_PER);
	context_restore_update(DOM_CORE1);

	trg_st.mpu_state	= PRCM_MPU_OFF;
	suspend_state		= PRCM_OFF;
#else
	trg_st.mpu_state	= PRCM_MPU_CSWR_L2RET;
	suspend_state		= PRCM_RET;
#endif

#ifndef CONFIG_ARCH_OMAP3410
	trg_st.neon_state	= suspend_state;
	trg_st.gfx_state	= suspend_state;
#endif
	trg_st.iva2_state	= suspend_state;
	trg_st.dss_state	= suspend_state;
	trg_st.cam_state	= suspend_state;
	trg_st.per_state	= suspend_state;
#ifndef CONFIG_CORE_OFF
	/* If we keep CORE in RET instead of OFF, we also keep PER in RET. */
	trg_st.per_state	= PRCM_RET;
#endif
	trg_st.usbhost_state	= suspend_state;


#ifndef CONFIG_ARCH_OMAP3410
	if (trg_st.neon_state == PRCM_OFF)
		omap3_save_neon_context();
#endif
	if (trg_st.per_state == PRCM_OFF)
		omap3_save_per_context();

#ifdef CONFIG_CORE_OFF
	trg_st.core_state	= PRCM_CORE_OFF;
#else
	trg_st.core_state	= PRCM_CORE_CSWR_MEMRET;
#endif

	if (trg_st.core_state == PRCM_CORE_CSWR_MEMRET) {
		memory_logic_res_setting(&trg_st);
	}

	if (trg_st.core_state >= PRCM_CORE_OSWR_MEMRET) {
		prcm_save_core_context(trg_st.core_state);
	}

#ifdef CONFIG_OMAP_32K_TIMER
	omap_disable_system_timer();
#endif

	/* prcm_set_chip_power_mode() will suspend the system. Function will
	 * return upon system wake-up.
	 */
	ret = prcm_set_chip_power_mode(&trg_st);
	if (ret != 0)
		printk(KERN_ERR "Could not enter target state in pm_suspend\n");
	else {
//		printk(KERN_INFO
//			"Successfully put chip to target state in suspend\n");
	}


#ifndef CONFIG_ARCH_OMAP3410
	if (trg_st.neon_state == PRCM_OFF)
		omap3_restore_neon_context();
#endif
	if (trg_st.per_state == PRCM_OFF)
		omap3_restore_per_context();

#ifdef CONFIG_CORE_OFF
	omap3_restore_core_settings();
#else
	if (trg_st.core_state >= PRCM_CORE_OSWR_MEMRET) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		context_restore_update(DOM_CORE1);
#endif
		prcm_restore_core_context(trg_st.core_state);
	}

	if (trg_st.core_state > PRCM_CORE_CSWR_MEMRET) {
		restore_sram_functions();	/*restore sram data */
		omap_sram_idle_setup();
	}
#endif
	enable_smartreflex(SR1_ID);
	enable_smartreflex(SR2_ID);

#ifdef CONFIG_OMAP_32K_TIMER
	omap_enable_system_timer();
#endif
	return 0;
}

static void (*saved_idle)(void) = NULL;

static int omap3_pm_valid(suspend_state_t state)
{
	int valid = 0;

	/* Do target specific suspend validation.
	 */
	if (omap3_pm_target_ops && omap3_pm_target_ops->valid) {
		valid = omap3_pm_target_ops->valid(state);
	}

	return valid;
}

static int omap3_pm_prepare(void)
{
	int ret = 0;

	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;

	/* Do target specific suspend preparation.
	 */
	if (omap3_pm_target_ops && omap3_pm_target_ops->prepare) {
		ret = omap3_pm_target_ops->prepare();
	}
	if (ret)
		return -EINVAL;


#ifdef CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND
	prcm_scale_prepare();
#endif
	return 0;
}

static int omap3_pm_enter(suspend_state_t state)
{
	int ret = 0;

	/* Do target specific suspend finzalizication.
	 */
	if (omap3_pm_target_ops && omap3_pm_target_ops->enter) {
		ret = omap3_pm_target_ops->enter(state);
	}
	if (ret)
		return -EINVAL;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap3_pm_suspend();
		break;
		/*case PM_SUSPEND_DISK:
		   ret = -ENOTSUPP;
		   break; */
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap3_pm_finish(void)
{
#ifdef CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND
	prcm_scale_finish();
#endif

	/* Do target specific suspend finalizafication.
	 */
	if (omap3_pm_target_ops && omap3_pm_target_ops->finish) {
		omap3_pm_target_ops->finish();
	}

	pm_idle = saved_idle;
}

static int omap3_pm_fastsleep(void)
{
	int fastsleep = 0;

	/* Do target specific fastsleep check.
	 */
	if (omap3_pm_target_ops && omap3_pm_target_ops->fastsleep) {
		fastsleep = omap3_pm_target_ops->fastsleep();
	}

	return fastsleep;
}

static struct platform_suspend_ops omap_pm_ops = {
	.valid		= omap3_pm_valid,
	.prepare	= omap3_pm_prepare,
	.enter		= omap3_pm_enter,
	.finish		= omap3_pm_finish,
	.fastsleep  = omap3_pm_fastsleep,
};
#endif

/******************************************************************************
 *
 * Interrupt handler
 *
 ******************************************************************************/

#ifdef CONFIG_PM

#define PRCM_IRQ_TIMEOUT 500000

/* PRM_IRQSTATUS_IVA2, PRM_IRQSTATUS_MPU shared bits */
#define OMAP3430_WKUP_ST	(1 << 0)
#define OMAP3430_IO_ST		(1 << 9)

static void handle_wkup_wkup ( void )
{
	u32 fclk, iclk;
	u32 cnt,  wkst;

	wkst = PM_WKST_WKUP;
	if (!wkst)
		return;

	cnt  = 0;
	iclk = CM_ICLKEN_WKUP;
	fclk = CM_FCLKEN_WKUP;
	for (;;) {
		CM_ICLKEN_WKUP |= wkst;
		CM_FCLKEN_WKUP |= wkst;
		PM_WKST_WKUP   = wkst;
		wkst = PM_WKST_WKUP;
		if (!wkst)
			break;
		if( cnt++ > PRCM_IRQ_TIMEOUT ) {
			panic("Timeout in %s: PM_WKST_WKUP=0x%08x\n",
			       __func__, PM_WKST_WKUP);
		}
		udelay(1);
	}
	CM_ICLKEN_WKUP = iclk;
	CM_FCLKEN_WKUP = fclk;
	return;
}


static void handle_core1_wkup ( void )
{
	u32 fclk, iclk;
	u32 cnt,  wkst;

	wkst = PM_WKST1_CORE;
	if (!wkst)
		return;

	cnt  = 0;
	iclk = CM_ICLKEN1_CORE;
	fclk = CM_FCLKEN1_CORE;
	for (;;){
		CM_ICLKEN1_CORE |= wkst;
		CM_FCLKEN1_CORE |= wkst;
		PM_WKST1_CORE = wkst;
		wkst = PM_WKST1_CORE;
		if (!wkst)
			break;
		if( cnt++ > PRCM_IRQ_TIMEOUT ) {
			panic("Timeout in %s: PM_WKST1_CORE=0x%08x\n",
			       __func__, PM_WKST1_CORE);
		}
		udelay(1);
	}
	CM_ICLKEN1_CORE = iclk;
	CM_FCLKEN1_CORE = fclk;
}

static void handle_core3_wkup ( void )
{
	u32 fclk, iclk;
	u32 cnt,  wkst;

	wkst = PM_WKST3_CORE;
	if(!wkst)
		return;

	cnt  = 0;
	iclk = CM_ICLKEN3_CORE;
	fclk = CM_FCLKEN3_CORE;
	for (;;){
		CM_ICLKEN3_CORE |= wkst;
		CM_FCLKEN3_CORE |= wkst;
		PM_WKST3_CORE = wkst;
		wkst = PM_WKST3_CORE;
		if (!wkst)
			break;
		if( cnt++ > PRCM_IRQ_TIMEOUT ) {
			panic("Timeout in %s: PM_WKST3_CORE=0x%08x\n",
			       __func__, PM_WKST3_CORE);
		}
		udelay(1);
	}
	CM_ICLKEN3_CORE = iclk;
	CM_FCLKEN3_CORE = fclk;
}


static void handle_usbhost_wkup (void)
{
	u32 fclk, iclk;
	u32 cnt,  wkst;

	wkst= PM_WKST_USBHOST;
	if (!wkst)
		return;

	cnt  = 0;
	iclk = CM_ICLKEN_USBHOST;
	fclk = CM_FCLKEN_USBHOST;
	CM_ICLKEN_USBHOST = 0x1;
	CM_FCLKEN_USBHOST = 0x3; /* both bits */
	for (;;){
		PM_WKST_USBHOST = wkst;
		wkst = PM_WKST_USBHOST;
		if (!wkst)
			break;
		if( cnt++ > PRCM_IRQ_TIMEOUT ) {
			panic( "Timeout in %s: PM_WKST_USBHOST=0x%08x\n",
				__func__, PM_WKST_USBHOST);
		}
		udelay(1);
	}
	CM_ICLKEN_USBHOST = iclk;
	CM_FCLKEN_USBHOST = fclk;
}


static void handle_per_wkup(void)
{
	u32 fclk, iclk;
	u32 cnt,  wkst;

	wkst = PM_WKST_PER;
	if (!wkst)
		return;

	cnt  = 0;
	iclk = CM_ICLKEN_PER;
	fclk = CM_FCLKEN_PER;
	for (;;){
		CM_ICLKEN_PER |= wkst;
		CM_FCLKEN_PER |= wkst;
		PM_WKST_PER = wkst;
		wkst = PM_WKST_PER;
		if (!wkst)
			break;
		if( cnt++ > PRCM_IRQ_TIMEOUT ) {
			panic("Timeout in %s: PM_WKST_PER=0x%08x\n",
			       __func__, PM_WKST_PER);
		}
		udelay(1);
	}
	CM_ICLKEN_PER = iclk;
	CM_FCLKEN_PER = fclk;
}


static irqreturn_t prcm_interrupt_handler(int irq, void *dev_id)
{
	u32 cnt, irq_st_mpu, irq_en_mpu;

	cnt = 0;
	irq_st_mpu = PRM_IRQSTATUS_MPU;
	irq_en_mpu = PRM_IRQENABLE_MPU;
	for (;;){
		irq_st_mpu = PRM_IRQSTATUS_MPU & irq_en_mpu;
		if (irq_st_mpu & (OMAP3430_WKUP_ST | OMAP3430_IO_ST)) {
			handle_wkup_wkup();
			handle_core1_wkup();
			handle_per_wkup();
			handle_core3_wkup();
			handle_usbhost_wkup();
		}
		PRM_IRQSTATUS_MPU = irq_st_mpu;
		irq_st_mpu = PRM_IRQSTATUS_MPU & irq_en_mpu;
		if (!irq_st_mpu)
			break;

		if (cnt++ > PRCM_IRQ_TIMEOUT) {
			panic("Timeout in %s: PRM_IRQSTATUS_MPU=0x%08x\n",
			       __func__, PRM_IRQSTATUS_MPU);
		}
		udelay(1);
	}
	return IRQ_HANDLED;
}
#endif /* CONFIG_PM */

/******************************************************************************
 *
 * PROCFS support
 *
 ******************************************************************************/

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static void *omap_pm_prepwst_start(struct seq_file *m, loff_t * pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *omap_pm_prepwst_next(struct seq_file *m, void *v, loff_t * pos)
{
	++*pos;
	return NULL;
}

static void omap_pm_prepwst_stop(struct seq_file *m, void *v)
{
}

int omap_pm_prepwst_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Previous power state for MPU +CORE\n");

#ifdef DEBUG_HW_SUP
	seq_printf(m, "PM_PWSTST_CORE %x\n", PM_PWSTST_CORE);
	seq_printf(m, "PM_PREPWSTST_CORE %x\n", PM_PREPWSTST_CORE);
	seq_printf(m, "CM_CLKSTCTRL_CORE %x\n", CM_CLKSTCTRL_CORE);
	seq_printf(m, "CM_CLKSTST_CORE %x\n\n", CM_CLKSTST_CORE);

	seq_printf(m, "PM_PWSTST_IVA2 %x\n", PM_PWSTST_IVA2);
	seq_printf(m, "PM_PREPWSTST_IVA2 %x\n", PM_PREPWSTST_IVA2);
	seq_printf(m, "CM_CLKSTCTRL_IVA2 %x\n", CM_CLKSTCTRL_IVA2);
	seq_printf(m, "CM_CLKSTST_IVA2 %x\n\n", CM_CLKSTST_IVA2);

#ifndef CONFIG_ARCH_OMAP3410
	seq_printf(m, "PM_PWSTST_NEON %x\n", PM_PWSTST_NEON);
	seq_printf(m, "PM_PREPWSTST_NEON %x\n", PM_PREPWSTST_NEON);
	seq_printf(m, "CM_CLKSTCTRL_NEON %x\n\n", CM_CLKSTCTRL_NEON);
#endif
	seq_printf(m, "PM_PWSTST_PER %x\n", PM_PWSTST_PER);
	seq_printf(m, "PM_PREPWSTST_PER %x\n", PM_PREPWSTST_PER);
	seq_printf(m, "CM_CLKSTCTRL_PER %x\n", CM_CLKSTCTRL_PER);
	seq_printf(m, "CM_CLKSTST_PER %x\n\n", CM_CLKSTST_PER);

	seq_printf(m, "PM_PWSTST_DSS %x\n", PM_PWSTST_DSS);
	seq_printf(m, "PM_PREPWSTST_DSS %x\n", PM_PREPWSTST_DSS);
	seq_printf(m, "CM_CLKSTCTRL_DSS %x\n", CM_CLKSTCTRL_DSS);
	seq_printf(m, "CM_CLKSTST_DSS %x\n\n", CM_CLKSTST_DSS);

	seq_printf(m, "PM_PWSTST_USBHOST %x\n", PM_PWSTST_USBHOST);
	seq_printf(m, "PM_PREPWSTST_USBHOST %x\n", PM_PREPWSTST_USBHOST);
	seq_printf(m, "CM_CLKSTCTRL_USBHOST %x\n", CM_CLKSTCTRL_USBHOST);
	seq_printf(m, "CM_CLKSTST_USBHOST %x\n\n", CM_CLKSTST_USBHOST);

#ifndef CONFIG_ARCH_OMAP3410
	seq_printf(m, "PM_PWSTST_SGX %x\n", PM_PWSTST_SGX);
	seq_printf(m, "PM_PREPWSTST_SGX %x\n", PM_PREPWSTST_SGX);
	seq_printf(m, "CM_CLKSTCTRL_SGX %x\n", CM_CLKSTCTRL_SGX);
	seq_printf(m, "CM_CLKSTST_SGX %x\n\n", CM_CLKSTST_SGX);
#endif

	seq_printf(m, "PM_PWSTST_CAM %x\n", PM_PWSTST_CAM);
	seq_printf(m, "PM_PREPWSTST_CAM %x\n", PM_PREPWSTST_CAM);
	seq_printf(m, "CM_CLKSTCTRL_CAM %x\n", CM_CLKSTCTRL_CAM);
	seq_printf(m, "CM_CLKSTST_CAM %x\n\n", CM_CLKSTST_CAM);
#endif				/* #ifdef DEBUG_HW_SUP */

#ifdef CONFIG_CPU_IDLE
	pm_cpuidle_prepwst_show(m, v);
#endif

	return 0;
}

static struct seq_operations omap_pm_prepwst_op = {
	.start = omap_pm_prepwst_start,
	.next = omap_pm_prepwst_next,
	.stop = omap_pm_prepwst_stop,
	.show = omap_pm_prepwst_show
};

static int omap_pm_prepwst_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &omap_pm_prepwst_op);
}

static struct file_operations proc_pm_prepwst_ops = {
	.open = omap_pm_prepwst_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

int create_pmproc_entry(void)
{
	struct proc_dir_entry *entry;

	/* Create a proc entry for shared resources */
	entry = create_proc_entry("pm_prepwst", 0, NULL);
	if (entry)
		entry->proc_fops = &proc_pm_prepwst_ops;
	else
		printk(KERN_ERR "create_proc_entry 'pm_prepwst' failed\n");

	return 0;
}
#endif				/* #ifdef CONFIG_PROC_FS */

/******************************************************************************
 *
 * INIT
 *
 ******************************************************************************/

#ifdef CONFIG_PM
int __init omap3_pm_init(void)
{
	int ret;

	printk(KERN_ERR "OMAP 3430 Power Management subsystem initializing.\n");

	omap_sram_idle_setup();
	suspend_set_ops(&omap_pm_ops);

	/* In case of cold boot, clear scratchpad */
	if (RM_RSTST_CORE & 0x1)
		clear_scratchpad_contents();
#ifdef CONFIG_MPU_OFF
	save_scratchpad_contents();
#endif

	/*
	 * Initialize the PRCM subsystem
	 */
	prcm_init();

	memret1 = (struct res_handle *)resource_get("resouret1", "core_mem1ret");
	memret2 = (struct res_handle *)resource_get("resouret12", "core_mem2ret");
	logret1 = (struct res_handle *)resource_get("logic retntion", "core_logicret");

	/* Initialize settings for voltage scaling.
	 */
	if (0 != prcm_vdd_clk_init()) {
		printk(KERN_ERR "could not initialize clocks.\n");
		return -1;
	}

	PRM_IRQSTATUS_MPU  = 0x3FFFFFD; // It takes into account reserved bits
	PRM_IRQENABLE_MPU  = 0x201;     // IOPAD + WKUP event

	/* Registering PRCM Interrupts to MPU, for Wakeup events */
	ret = request_irq(PRCM_MPU_IRQ, prcm_interrupt_handler,
			  IRQF_DISABLED, "prcm", NULL);
	if (ret) {
		printk(KERN_ERR "request_irq failed to register for 0x%x\n",
		       PRCM_MPU_IRQ);
		return -1;
	}

#ifdef CONFIG_PROC_FS
	create_pmproc_entry();
#endif
	return 0;
}

__initcall(omap3_pm_init);
#endif /* CONFIG_PM */
