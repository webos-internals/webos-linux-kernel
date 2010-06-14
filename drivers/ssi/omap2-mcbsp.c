/*
 * drivers/ssi/omap2-mcbsp.c
 *
 * Driver Library for TI's Multichannled Buffered Serial Port(McBSP) Interface
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/clock.h>
#include "omap2-mcbsp-def.h"

/* ----- debug defines ----------------------------------------------- */
#define OMAP2_MCBSP_POLL_READ_WRITE	0
#define OMAP2_MCBSP_DEBUG_REGS	0

/* Debug - Five macros:
 * FN_IN, FN_OUT(value),D1,D2,D3 enabled based on log level
 */

/* Log level standard used here:
 * Log level 3 all messages
 * Log level 2 all entry-exit points
 * Log level 1 major messages
 * Log level 0 no messages
 */
#define MCBSP_LOG_LEVEL 0
/* detail - 0 - no detail
 *          1 - function name
 *          2 - function name, line number
 * prefix is added to every log message
 */
#define MCBSP_DETAIL    2

/* kernel log level*/
//#define MCBSP_K_LOG_LEVEL KERN_DEBUG
#define MCBSP_K_LOG_LEVEL

#if ( MCBSP_DETAIL > 0 )
#define DL1 "%s "
#define DR1 ,__FUNCTION__
#else
#define DL1
#define DR1
#endif
#if ( MCBSP_DETAIL > 1 )
#define DL2 "[%d] "
#define DR2 ,__LINE__
#else
#define DL2
#define DR2
#endif

/* Wanted to reduce printks... at the same time ease development too..
 * cant do with the format,args version.. does not work with null args :(
 */
#define D(format,...)\
	printk( MCBSP_K_LOG_LEVEL DL1 DL2 format "\n" DR1 DR2, ## __VA_ARGS__)

#if (MCBSP_LOG_LEVEL >= 1)
#define D1(ARGS...) D(ARGS)
#else
#define D1(ARGS...)
#endif
#if (MCBSP_LOG_LEVEL >= 2)
#define D2(ARGS...) D(ARGS)
#else
#define D2(ARGS...)
#endif
#if (MCBSP_LOG_LEVEL >= 3)
#define D3(ARGS...) D(ARGS)
#else
#define D3(ARGS...)
#endif

#if (MCBSP_LOG_LEVEL >= 2)
#define FN_IN printk(MCBSP_K_LOG_LEVEL "%s: Entry\n",__FUNCTION__);
#define FN_OUT(ARG) printk(MCBSP_K_LOG_LEVEL "%s[%d]:Exit(%d)\n",__FUNCTION__,__LINE__,ARG);
#else
#define FN_IN
#define FN_OUT(ARG)
#endif

/*****************************************************************************************/
/*
 * This function configures external interfaces
 * to get mcbsp working on 2420 based H4 board
 */
#ifdef CONFIG_ARCH_OMAP2420
static void omap_mcbsp_ext_config(void)
{
#ifdef MCBSP_PIN_MUXING
	/* Pin Muxing
	 * These pins are used exclusively b/w mcbsp2 and eac. since eac is no longer
	 * present, we can have this in mux.h. currently hacking in here.
	 */

	/* Enable Mode1 for McBSP2  */
	omap_mcbsp_pin_out(CONTROL_PADCONF_eac_ac_sclk, config_eac_ac_sclk);
	omap_mcbsp_pin_out(CONTROL_PADCONF_eac_ac_fs, config_eac_ac_fs);
	omap_mcbsp_pin_out(CONTROL_PADCONF_eac_ac_din, config_eac_ac_din);
	omap_mcbsp_pin_out(CONTROL_PADCONF_eac_ac_dout, config_eac_ac_dout);

	/* Enable Mode3 for GPIO117  */
	omap_mcbsp_pin_out(CONTROL_PADCONF_eac_ac_mclk, config_eac_ac_mclk);

#endif				/* MCBSP_PIN_MUXING */
}
#endif				/* End of #ifdef CONFIG_ARCH_OMAP24XX */

#ifdef	OMAP2_MCBSP_DEBUG_REGS

/*
 * Dump the McBSP register values
 * mcbsp_id	: Interface number
 */
int omap_mcbsp_dump_reg(u32 mcbsp_id)
{
	u32 virt_base;
	FN_IN;
	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	virt_base = omap2_mcbsp_info[mcbsp_id].virt_base;

	printk("\n***************************\n");
	printk("\n    MCBSP ID  = %d\n", mcbsp_id);
	printk("\n***************************\n");

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)

	printk("OMAP2_MCBSP_REG_SYSCONFIG [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_SYSCONFIG,
	       OMAP2_MCBSP_READ(virt_base, SYSCONFIG));
	printk("OMAP2_MCBSP_REG_THRSH2 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_THRSH2, OMAP2_MCBSP_READ(virt_base,
								    THRSH2));
	printk("OMAP2_MCBSP_REG_THRSH1 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_THRSH1, OMAP2_MCBSP_READ(virt_base,
								    THRSH1));
	printk("OMAP2_MCBSP_REG_XCCR  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_XCCR, OMAP2_MCBSP_READ(virt_base,
								  XCCR));
	printk("OMAP2_MCBSP_REG_RCCR  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_RCCR, OMAP2_MCBSP_READ(virt_base,
								  RCCR));
#endif
	printk("OMAP2_MCBSP_REG_RCR2  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_RCR2, OMAP2_MCBSP_READ(virt_base,
								  RCR2));

	printk("OMAP2_MCBSP_REG_RCR1  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_RCR1, OMAP2_MCBSP_READ(virt_base,
								  RCR1));

	printk("OMAP2_MCBSP_REG_XCR2  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_XCR2, OMAP2_MCBSP_READ(virt_base,
								  XCR2));

	printk("OMAP2_MCBSP_REG_XCR1  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_XCR1, OMAP2_MCBSP_READ(virt_base,
								  XCR1));

	printk("OMAP2_MCBSP_REG_SRGR2 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_SRGR2, OMAP2_MCBSP_READ(virt_base,
								   SRGR2));

	printk("OMAP2_MCBSP_REG_SRGR1 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_SRGR1, OMAP2_MCBSP_READ(virt_base,
								   SRGR1));

	printk("OMAP2_MCBSP_REG_PCR   [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_PCR, OMAP2_MCBSP_READ(virt_base,
								 PCR));

	printk("OMAP2_MCBSP_REG_SPCR2 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_SPCR2, OMAP2_MCBSP_READ(virt_base,
								   SPCR2));

	printk("OMAP2_MCBSP_REG_SPCR1 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_SPCR1, OMAP2_MCBSP_READ(virt_base,
								   SPCR1));

	printk("OMAP2_MCBSP_REG_REV   [0x%08x] = 0x%08x\n",
	       virt_base + OMAP2_MCBSP_REG_REV, OMAP2_MCBSP_READ(virt_base,
								 REV));

	FN_OUT(0);
	return 0;
}
#endif
#ifdef MCBSP_POWER
/*
 * mcbsp power settings
 * mcbsp_id     : McBSP interface number
 * level        : power settings level
 */
static void  mcbsp_power_settings(u32 mcbsp_id, int level)
{
	u32 base;
	base = omap2_mcbsp_info[mcbsp_id].virt_base;

	if(level == MCBSP2_SYSCONFIG_LVL1)
		OMAP2_MCBSP_WRITE(base, SYSCONFIG,
		CLOCKACTIVITY(MCBSP_SYSC_IOFF_FON) | SIDLEMODE(SMART_IDLE) |
		ENAWAKEUP);

	if(level == MCBSP2_SYSCONFIG_LVL2)
		OMAP2_MCBSP_WRITE(base, SYSCONFIG,
		CLOCKACTIVITY(MCBSP_SYSC_IOFF_FOFF) | SIDLEMODE(FORCE_IDLE));
}
#endif

/*
 * Basic Reset Reciever
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
static int mcbsp_rrst_set(u32 base, u8 state)
{
	if (state == OMAP2_MCBSP_RRST_DISABLE) {
		/* Reset the Receiver */
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) & (~RRST));
		state = 0;
	} else if (state == OMAP2_MCBSP_RRST_ENABLE) {
		/* Receiver is pulled out of reset */
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) | (RRST));
		state = 1;
	} else {

		D2(KERN_ERR "OMAP2_McBSP: Invalid RRST state \n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	return 0;
}

/*
 * Basic Reset Transmitter
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
static int mcbsp_xrst_set(u32 base, u8 state)
{
	if (state == OMAP2_MCBSP_XRST_DISABLE) {
		/* Reset the Transmitter */
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) & (~XRST));
	} else if (state == OMAP2_MCBSP_XRST_ENABLE) {
		/* Transmitter is pulled out of reset */
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) | (XRST));
	} else {

		D2(KERN_ERR "OMAP2_McBSP: Invalid XRST state \n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	return 0;
}

/* mcbsp_lock	: To avoid muliple entries in critical sections, uses Semaphore locks */
static inline int mcbsp_lock(struct semaphore *sem)
{
	FN_IN;
	/*
	 * If called from interrupt context and if the lock is already
	 * active (aquired) then return EBUSY
	 */
	if (in_interrupt()) {
		D3("OMAP2_McBSP: In mcbsp_lock interrupt context \n");
		if (down_trylock(sem)) {
			D3("\n OMAP2_McBSP: Unable to get lock returning with ebusy\n");
			FN_OUT(-EBUSY);
			return -EBUSY;
		} else {
			D3("\n OMAP2_McBSP: Got the semaphore lock\n");
			FN_OUT(0);
			return 0;
		}
	} else {
		D3("\n OMAP2_McBSP: Not in interrupt getting locked \n");
		FN_OUT(1);
		return down_interruptible(sem);
	}
}

#ifdef	OMAP2_MCBSP_POLL_READ_WRITE

/*
 * Transmit a word in polling mode
 * mcbsp_id	: McBSP interface number
 * buf		: data to be transmitted
 */
int omap_mcbsp_pollwrite(u32 mcbsp_id, u32 data)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;

	D3("OMAP2_McBSP: Data Requested=[0x%x] Data Written=[0x%x]\n", data,
	   data & (0xFFFF));
	data &= 0xFFFF;		/* only 16 bits at a time please */
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
	/*
	 * poll write accepts only 16 bits at a time,
	 * 2430 has 32 bit registers so writew is used explicitly
	 */
	writew(data, base + OMAP2_MCBSP_REG_DXR);
#else
	OMAP2_MCBSP_WRITE(base, DXR1, data);
#endif
	/* if frame sync error - clear the error */
	if (OMAP2_MCBSP_READ(base, SPCR2) & XSYNCERR) {
		/* clear error */
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) & (~XSYNCERR));
		/* resend */
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EPERM);
		return -EPERM;
	} else {
		/* wait for transmit confirmation */
		int attemps = 0;
		while (!(OMAP2_MCBSP_READ(base, SPCR2) & XRDY)) {
			if (attemps++ > 1000) {
				OMAP2_MCBSP_WRITE(base, SPCR2,
						  OMAP2_MCBSP_READ(base,
								   SPCR2) &
						  (~XRST));
				udelay(10);
				OMAP2_MCBSP_WRITE(base, SPCR2,
						  OMAP2_MCBSP_READ(base,
								   SPCR2) |
						  (XRST));
				udelay(10);
				D2(KERN_ERR
				   " Could not write to McBSP Register\n");
				up(&omap2_mcbsp_info[mcbsp_id].semlock);
				FN_OUT(-EAGAIN);
				return -EAGAIN;
			}
			/*
			 * schedule time out is not used as this function is
			 * used only for debugging purposes
			 */
			udelay(2000);
		}
	}
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Receive a word in polling mode
 * mcbsp_id	: McBSP interface number
 *
 * returns received value
 */
int omap_mcbsp_pollread(u32 mcbsp_id)
{
	u32 base;
	u32 data;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EPERM);
		return -EPERM;
	}

	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;

	/* if frame sync error - clear the error */
	if (OMAP2_MCBSP_READ(base, SPCR1) & RSYNCERR) {
		printk(KERN_ERR "RSYNCERR 0x%x\n",
		       OMAP2_MCBSP_READ(base, SPCR1));
		/* clear error */
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) & (~RSYNCERR));
		/* resend */
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EPERM);
		return -EPERM;
	} else {
		/* wait for recieve confirmation */
		int attemps = 0;
		while (!(OMAP2_MCBSP_READ(base, SPCR1) & RRDY)) {
			if (attemps++ > 100000) {
				continue;
				D2(KERN_ERR
				   " Could not read from McBSP Register\n");
				omap_mcbsp_dump_reg(mcbsp_id);
				OMAP2_MCBSP_WRITE(base, SPCR1,
						  OMAP2_MCBSP_READ(base,
								   SPCR1) &
						  (~RRST));
				udelay(10);
				OMAP2_MCBSP_WRITE(base, SPCR1,
						  OMAP2_MCBSP_READ(base,
								   SPCR1) |
						  (RRST));
				udelay(10);
				up(&omap2_mcbsp_info[mcbsp_id].semlock);
				FN_OUT(-EAGAIN);
				return -EAGAIN;
			}
			/*
			 * schedule time out is not used as this function is
			 * used only for debugging purposes
			 */
			//udelay(1);
		}
	}

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
	/*
	 * poll read returns only 16 bits at a time,
	 * 2430 has 32 bit registers so readw is used explicitly
	 */
	data = OMAP2_MCBSP_READ(base, DRR);
#else
	data = OMAP2_MCBSP_READ(base, DRR1);
#endif
	D2("Data Read=[0x%x] Data Passed Down=[0x%x]\n", data, data & (0xFFFF));
	data &= 0xFFFF;
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(data);
	return data;
}
#endif

/*
 * Reset and Enable/Disable Sample Rate Generator
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0) [RESET] / Enable (1) the SRG
 */
int omap2_mcbsp_set_srg(u32 mcbsp_id, u8 state)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (state == OMAP2_MCBSP_SRG_DISABLE) {
		/* Reset the Sample Rate Generator */
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) & (~GRST));
	} else if (state == OMAP2_MCBSP_SRG_ENABLE) {
		/* Sample Rate Generator is pulled out of reset */
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) | (GRST));
	} else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid SRG state\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);
}

/*
 * Reset and Enable/Disable Frame Sync Generator
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0)/ Enable (1) the FSG
 */
int omap2_mcbsp_set_fsg(u32 mcbsp_id, u8 state)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (state == OMAP2_MCBSP_FSG_DISABLE) {
		/* Frame Sync Generator is reset */
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) & (~FRST));
	} else if (state == OMAP2_MCBSP_FSG_ENABLE) {
		/* Frame Sync Generator is pulled out of reset */
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) | (FRST));
	} else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid FSG state \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);
}

/*
 * Basic enable/disable Reciever
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
int omap2_mcbsp_set_ren(u32 mcbsp_id, u8 state)
{
	int ret = 0;
	u32 base = 0;
	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}
	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (state == OMAP2_MCBSP_RX_DISABLE) {
		/* Disable the Reciever */
		OMAP2_MCBSP_WRITE(base, RCCR,
				  OMAP2_MCBSP_READ(base, RCCR) & ~(RDISABLE));
	} else if (state == OMAP2_MCBSP_RX_ENABLE) {
		/* Reciever is pulled out of reset */
		OMAP2_MCBSP_WRITE(base, RCCR,
				  OMAP2_MCBSP_READ(base, RCCR) | (RDISABLE));
	} else {

		D2(KERN_ERR "OMAP2_McBSP: Invalid RRST state \n");
		FN_OUT(-EINVAL);
		ret = -EINVAL;
	}
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(ret);
	return (ret);
}

/*
 * Basic enable/disable Transmitter
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
int omap2_mcbsp_set_xen(u32 mcbsp_id, u8 state)
{
	int ret = 0;
	u32 base = 0;
	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}
	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (state == OMAP2_MCBSP_TX_DISABLE) {
		/* Reset the Transmitter */
		OMAP2_MCBSP_WRITE(base, XCCR,
				  OMAP2_MCBSP_READ(base, XCCR) & (~XDISABLE));
	} else if (state == OMAP2_MCBSP_TX_ENABLE) {
		/* Transmitter is pulled out of reset */
		OMAP2_MCBSP_WRITE(base, RCCR,
				  OMAP2_MCBSP_READ(base, XCCR) | (XDISABLE));
	} else {

		D2(KERN_ERR "OMAP2_McBSP: Invalid XEN state \n");
		ret = -EINVAL;
	}
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(ret);
	return (ret);
}

/*
 * Reset Transmitter
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
int omap2_mcbsp_set_xrst(u32 mcbsp_id, u8 state)
{
	u32 base;
	int ret = 0;
	int counter = 100;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	ret = mcbsp_xrst_set(base, state);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: reset failed %d\n", ret);
		goto out_xrst;
	}
	if (state == OMAP2_MCBSP_XRST_DISABLE) {
		state = 0;
	} else if (state == OMAP2_MCBSP_XRST_ENABLE) {
		state = 1;
	}
	/* Wait for Transmitter reset to be complete */
	while (counter > 1) {
		u32 xrdy = (OMAP2_MCBSP_READ(base, SPCR2) & XRDY);
		if ((state && xrdy) || (!state && !xrdy))
			break;
		counter--;
		udelay(10);
	}
	if (counter <= 1) {
		D2(KERN_WARNING
		   "xRDY never Got %s even after setting RRST!!n",
		   (state) ? "Enabled" : "Disabled");
		FN_OUT(-ETIMEDOUT);
		ret = -ETIMEDOUT;
	}

      out_xrst:
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(ret);
	return (ret);
}

/*
 * Reset Receiver
 * mcbsp_id	: McBSP interface number
 * state	: Disable (0)/ Enable (1) the receiver
 */
int omap2_mcbsp_set_rrst(u32 mcbsp_id, u8 state)
{
	u32 base;
	int ret;

	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}
	ret = mcbsp_rrst_set(base, state);
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(ret);
	return (ret);
}

/*
 * Interface Reset
 * mcbsp_id	: McBSP interface ID
 * Resets the McBSP interface
 */
int omap2_mcbsp_interface_reset(u32 mcbsp_id)
{
	u32 base;
	int counter = 0;
	int wait_for_reset = 10000;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
	/* Write 1 to sysconfig bit[1] to trigger interface reset */
	OMAP2_MCBSP_WRITE(base, SYSCONFIG,
			  OMAP2_MCBSP_READ(base, SYSCONFIG) | (SOFTRST));
	while (OMAP2_MCBSP_READ(base, SYSCONFIG) & SOFTRST) {
		if (!in_interrupt()) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(10);
		}
		if (counter++ > wait_for_reset) {
			D2(KERN_ERR "mcbsp[%d] Reset timeout\n", mcbsp_id);
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-ETIMEDOUT);
			return -ETIMEDOUT;
		}
	}
#ifdef MCBSP_POWER
	mcbsp_power_settings(mcbsp_id, MCBSP2_SYSCONFIG_LVL1);
#endif
	/*
	 * Enabling all wakeup events. Enabled here to set it once after every
	 * McBSP soft reset
	 */
	OMAP2_MCBSP_WRITE(base, WAKEUPEN, 0xFFFF);
#else

	/*
	 * No SYSCONFIG Module Reset available on 2420
	 * Reset Regs from SPCR2 to SRGR1
	 */
	for (counter = 0x10; counter < 0x30;) {
		__raw_writew(0, base + counter);
		counter += 4;
	}
	/* Reset PCR */
	__raw_writew(0, base + 0x48);

#endif

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Auto idle enable
 * mcbsp_id	: McBSP interface ID
 */
int omap2_mcbsp_autoidle_enable(u32 mcbsp_id)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	/* Enabling the auto idle bit in PCR register is shown as a legacy mode
	   in 2430 TRM */
	OMAP2_MCBSP_WRITE(base, PCR, OMAP2_MCBSP_READ(base, PCR) | (IDLEEN));
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Auto idle disable
 * mcbsp_id	: McBSP interface ID
 */
int omap2_mcbsp_autoidle_disable(u32 mcbsp_id)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	/* Enabling the auto idle bit in PCR register is shown as a legacy mode
	   in 2430 TRM */
	OMAP2_MCBSP_WRITE(base, PCR, OMAP2_MCBSP_READ(base, PCR) & (~IDLEEN));
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Power management hooks
 */

/*
 * Suspend, support for Client driver's power management
 * mcbsp_id	: McBSP interface ID
 * Stops the clocks (interface and functional),
 * Interrupts are disabled
 *
 * Note: Data transmission can be stopped using the stop API's
 *
 * Assumptions: Client driver will stop the ongoing DMA (if any).
 */
int omap2_mcbsp_suspend(u32 mcbsp_id)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	omap2_mcbsp_info[mcbsp_id].suspended = 1;

	/* Disable interrupts */
	OMAP2_MCBSP_WRITE(base, IRQENABLE, 0x00000000);

	/* Stop FCLK */
	clk_disable(omap2_mcbsp_info[mcbsp_id].fck);

	/* Stop ICLK */
	clk_disable(omap2_mcbsp_info[mcbsp_id].ick);

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Resume ,support for Client driver's power managment
 * mcbsp_id	: McBSP interface ID
 *
 * Starts the clocks (interface and functional), Interrupts are enabled
 */
int omap2_mcbsp_resume(u32 mcbsp_id)
{
	u32 base;
	int ret,ret_err;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is not suspended */
	if (!(OMAP2_McBSP_SUSPENDED(mcbsp_id))) {
		D2(KERN_ERR "OMAP2_McBSP: Interface not suspended to resume\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	omap2_mcbsp_info[mcbsp_id].suspended = 0;

	/* Start ICLK */
	ret_err=clk_enable(omap2_mcbsp_info[mcbsp_id].ick);
	if(ret_err){
                return ret_err;
        }


	/* Start FCLK */
	ret_err=clk_enable(omap2_mcbsp_info[mcbsp_id].fck);
	if(ret_err){
		clk_disable(omap2_mcbsp_info[mcbsp_id].ick);
                return ret_err;
        }

	/* Enable the interrupts */
	OMAP2_MCBSP_WRITE(base, IRQENABLE, omap2_mcbsp_info[mcbsp_id].irqmask);

	up(&omap2_mcbsp_info[mcbsp_id].semlock);

	/*
	 * Dont start DMA, we don't know which one to start with,
	 * wait for send or receive request
	 */
	FN_OUT(0);
	return (0);
}

/*
 * Enable/Disable the Global data rx/tx behaviour
 * mcbsp_id	: McBSP interface No.
 * mode		: Data mode (Loop back or synchronous rx/tx)
 */
int omap2_mcbsp_datatxrx_mode(u32 mcbsp_id, u32 mode)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	switch (mode) {
		/* Analog Loop back mode  */
	case OMAP2_MCBSP_ALB:
		/* Disable the Digital loop back mode */
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
		OMAP2_MCBSP_WRITE(base, XCCR,
				  OMAP2_MCBSP_READ(base, XCCR) & ~(DLB));
#endif
		/* Enable the Analog loop back mode */
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) | (ALB));
		break;

		/* Digital Loop back mode */
	case OMAP2_MCBSP_DLB:
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
		/* Disable the Analog loop back mode */
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) & ~(ALB));
		/* Enable the Digital loop back mode */
		OMAP2_MCBSP_WRITE(base, XCCR,
				  OMAP2_MCBSP_READ(base, XCCR) | (DLB));
#else
		/* Disable the Analog loop back mode */
		/* There is no ALB in 242x, but its the same bit field */
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) | (ALB));
#endif
		break;

		/* Synchronous Tx Rx mode */
	case OMAP2_MCBSP_SYNCTXRX:
		/* ALB=0; DLB = 0; sets the mode to synchronous tx rx */
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) & ~(ALB));
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
		OMAP2_MCBSP_WRITE(base, XCCR,
				  OMAP2_MCBSP_READ(base, XCCR) & ~(DLB));
#endif
		break;

	default:
		D2(KERN_ERR "OMAP2_McBSP: Invalid datatxrx state \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	D3("OMAP2_McBSP: txrxm SPCR1 [0x%08x] = 0x%08x\n",
	   base + OMAP2_MCBSP_REG_SPCR1, OMAP2_MCBSP_READ(base, SPCR1));
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);
}

/*
 * Receive multichannel selection
 * mcbsp_id	: McBSP Interface ID.
 * state	: Enable/Disable multichannel for reception
 */
int omap2_mcbsp_rxmultich_enable(u32 mcbsp_id, u8 state)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (state == OMAP2_MCBSP_RXMUTICH_ENABLE) {
		/* All the 128 channels are enabled */
		OMAP2_MCBSP_WRITE(base, MCR1,
				  OMAP2_MCBSP_READ(base, MCR1) & ~(RMCM));
	} else if (state == OMAP2_MCBSP_RXMUTICH_DISABLE) {
		/*
		 * All channels are disabled by default.
		 * Required channels are selected by enabling
		 * RP(A/B)BLK and RCER(A/B) appropriately.
		 */
		OMAP2_MCBSP_WRITE(base, MCR1,
				  OMAP2_MCBSP_READ(base, MCR1) | (RMCM));
	} else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid rxmultichannel state \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);

}

/*
 * Transmit multichannel selection
 * mcbsp_id	: McBSP interface ID
 * state	: Enable/Disable multichannel mode for transmission
 */
int omap2_mcbsp_txmultich_enable(u32 mcbsp_id, u32 state)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (state == OMAP2_MCBSP_TXMUTICH_ENABLE) {
		/* All the 128 channels are enabled */
		OMAP2_MCBSP_WRITE(base, MCR2,
				  OMAP2_MCBSP_READ(base, MCR2) | (XMCM(0)));
	} else if (state == OMAP2_MCBSP_TXMUTICH_DISABLE) {
		/*
		 * All channels are disabled by default.
		 * Required channels are selected by enabling
		 * RP(A/B)BLK and RCER(A/B) appropriately.
		 */
		OMAP2_MCBSP_WRITE(base, MCR2,
				  OMAP2_MCBSP_READ(base, MCR2) | (XMCM(1)));
	} else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid rxmultichannel state \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);
}

/*
 * Transmit multichannel configuration
 * mcbsp_id	: McBSP interface No.
 *
 */
int omap2_mcbsp_txmultich_cfg(u32 mcbsp_id, u8 part_mode,
			       u8 parta_enable, u8 partb_enable, u32 ch_enable)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	/* check for valid partition mode */
	if ((part_mode != OMAP2_MCBSP_TWOPARTITION_MODE
	     && part_mode != OMAP2_MCBSP_EIGHTPARTITION_MODE))
		goto tx_mch_err;

	OMAP2_MCBSP_WRITE(base, MCR2,
			  (OMAP2_MCBSP_READ(base, MCR2) | part_mode |
			   parta_enable | partb_enable));
	OMAP2_MCBSP_WRITE(base, XCERA,
			 (OMAP2_MCBSP_READ(base, XCERA) | ch_enable));
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);

      tx_mch_err:
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(-EINVAL);
	return (-EINVAL);

}

/*
 * Receive multichannel configuration
 * mcbsp_id	: McBSP interface No.
 *
 */
int omap2_mcbsp_rxmultich_cfg(u32 mcbsp_id, u8 part_mode,
			       u8 parta_enable, u8 partb_enable, u32 ch_enable)
{
	u32 base;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	/* check for valid partition mode */
	if ((part_mode != OMAP2_MCBSP_TWOPARTITION_MODE
	     && part_mode != OMAP2_MCBSP_EIGHTPARTITION_MODE))
		goto rx_mch_err;


	OMAP2_MCBSP_WRITE(base, MCR1,
			  (OMAP2_MCBSP_READ(base, MCR1) | part_mode |
			   parta_enable | partb_enable));
	 OMAP2_MCBSP_WRITE(base, RCERA,
			(OMAP2_MCBSP_READ(base, RCERA) | ch_enable));

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);

      rx_mch_err:
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(-EINVAL);
	return (-EINVAL);
}

/*
 * Configure the Frame Sync Generator
 * mcbsp_id	: McBSP Interface ID
 * tx_fsync_src	: FSX source (External or Internal)
 * rx_fsync_src	: FSR source (External or Internal)
 * tx_polarity	: Polarity of the FSX source clock (Active high / Active low)
 * rx_polarity	: Polarity of the FSR source clock (Active high / Active low)
 * period		: when the next frame-sync signal becomes active
 * pulse_width	: Width of the frame-sync pulse (FSG)
 */
int omap2_mcbsp_fsync_cfg(u32 mcbsp_id, u8 tx_fsync_src,
			  u8 rx_fsync_src, u8 tx_polarity,
			  u8 rx_polarity, u32 period, u32 pulse_width, u8 fsgm)
{
	u32 base;
	u32 w_pcr = 0;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	w_pcr = OMAP2_MCBSP_READ(base, PCR);

	/* Rx Polarity */
	if (rx_polarity == OMAP2_MCBSP_FS_ACTIVE_LOW)
		w_pcr = w_pcr | FSRP;
	else if (rx_polarity == OMAP2_MCBSP_FS_ACTIVE_HIGH)
		w_pcr = w_pcr & ~(FSRP);
	else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid Rx polarity \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Tx Polarity */
	if (tx_polarity == OMAP2_MCBSP_FS_ACTIVE_LOW)
		w_pcr = w_pcr | FSXP;
	else if (tx_polarity == OMAP2_MCBSP_FS_ACTIVE_HIGH)
		w_pcr = w_pcr & ~(FSXP);
	else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid tx polarity \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	/* Tx Frame sync source selection */
	if (tx_fsync_src == OMAP2_MCBSP_TXFSYNC_EXTERNAL)
		w_pcr = w_pcr & ~(FSXM);
	else if (tx_fsync_src == OMAP2_MCBSP_TXFSYNC_INTERNAL)
		w_pcr = w_pcr | (FSXM);
	else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid tx fsync src\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	/* Rx Frame sync source selection */
	if (rx_fsync_src == OMAP2_MCBSP_RXFSYNC_EXTERNAL)
		w_pcr = w_pcr & ~(FSRM);
	else if (rx_fsync_src == OMAP2_MCBSP_RXFSYNC_INTERNAL)
		w_pcr = w_pcr | (FSRM);
	else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid rx fsync src\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	OMAP2_MCBSP_WRITE(base, PCR, w_pcr);

	/* Clear the fields */
	OMAP2_MCBSP_WRITE(base, SRGR2,
			  (OMAP2_MCBSP_READ(base, SRGR2) & ~(FPER(0xFFF)|FSGM)));
	OMAP2_MCBSP_WRITE(base, SRGR1,
			  (OMAP2_MCBSP_READ(base, SRGR1) & ~(FWID(0xFF))));
	/*
	 * if the sync source is internal only then its valid to
	 * write pulse width and period
	 */
	if (rx_fsync_src == OMAP2_MCBSP_RXFSYNC_INTERNAL ||
	    tx_fsync_src == OMAP2_MCBSP_TXFSYNC_INTERNAL) {

		/* Update the registers */
		OMAP2_MCBSP_WRITE(base, SRGR2,
				  OMAP2_MCBSP_READ(base, SRGR2) | FPER(period)|(fsgm? FSGM : 0));
		OMAP2_MCBSP_WRITE(base, SRGR1,
				  OMAP2_MCBSP_READ(base,
						   SRGR1) | FWID(pulse_width));
	}
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);
}

/*
 * Configure the receiver clock
 * mcbsp_id	: McBSP interface ID
 * clk_mode	: Input clock source for receiver (External or Internal)
 * polarity	: Polarity of the input clock (Rising or Falling)
 */
int omap2_mcbsp_rxclk_cfg(u32 mcbsp_id, u8 clk_mode, u8 polarity)
{
	u32 base;
	u32 w_pcr = 0;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	w_pcr = OMAP2_MCBSP_READ(base, PCR);
	/* Select the rxclk input source */
	if (clk_mode == OMAP2_MCBSP_CLKRXSRC_EXTERNAL)
		w_pcr &= ~CLKRM;
	else if (clk_mode == OMAP2_MCBSP_CLKRXSRC_INTERNAL)
		w_pcr |= CLKRM;
	else {

		D2(KERN_ERR "OMAP2_McBSP: Invalid Rx Clock Source \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	/* Set the clock polarity */
	if (polarity == OMAP2_MCBSP_CLKR_POLARITY_FALLING)
		w_pcr &= ~CLKRP;
	else if (polarity == OMAP2_MCBSP_CLKR_POLARITY_RISING) {
		w_pcr |= CLKRP;
	} else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid CLKR Polarity \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	OMAP2_MCBSP_WRITE(base, PCR, w_pcr);
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Configure the transmitter clock
 * mcbsp_id	: McBSP interface ID
 * clk_mode	: Input clock source for transmitter module (External or Internal)
 * polarity	: polarity of the input clock (Rising or Falling)
 */
int omap2_mcbsp_txclk_cfg(u32 mcbsp_id, u8 clk_mode, u8 polarity)
{
	u32 base;
	int ret;
	u32 w_pcr = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	w_pcr = OMAP2_MCBSP_READ(base, PCR);

	/* Select the txclk input source */
	if (clk_mode == OMAP2_MCBSP_CLKTXSRC_EXTERNAL)
		w_pcr &= ~CLKXM;
	else if (clk_mode == OMAP2_MCBSP_CLKTXSRC_INTERNAL)
		w_pcr |= CLKXM;
	else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid CLK Tx Source\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	/* Set the clock polarity */
	if (polarity == OMAP2_MCBSP_CLKX_POLARITY_RISING)
		w_pcr &= ~CLKXP;
	else if (polarity == OMAP2_MCBSP_CLKX_POLARITY_FALLING) {
		w_pcr |= CLKXP;
	} else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid CLKX polarity\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	OMAP2_MCBSP_WRITE(base, PCR, w_pcr);
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;

}

/*
 * Configure the Sample rate and data clock CLKG
 * sample_rate		: Required sampling rate in Hz
 * bits_per_sample	: Number of bits per sample
 * clk_rate		: The SRC Clock frequency in hertz, required for clks,
 * 				clkr, clkx and fclk (non prcm).
 * srg_src		: Source of SRG input clock (can be clks, clkr, clkx, fclk
 * sync_mode		: free running or running
 * polarity		: Polarity of input clock
 */
int omap2_mcbsp_srg_cfg(u32 mcbsp_id, u32 sample_rate,
			u32 bits_per_sample, u32 srg_src, u32 clk_rate,
			u8 sync_mode, u8 polarity)
{
	u32 base;
	u32 clkgdv = 0;
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		printk(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		printk(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		printk(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (sample_rate == 0 || bits_per_sample == 0) {
		printk(KERN_ERR
		       "OMAP2_McBSP: Invalid sample rate or bits per sample\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Select the input clock source for Sample Rate Generator */
	switch (srg_src) {
		/* Signal on the mcbsp_clks pin */
	case OMAP2_MCBSP_SRGCLKSRC_CLKS:
		/* PCR[SCLKME] = 0 SRGR2[CLKSM] =0 */
		OMAP2_MCBSP_WRITE(base, PCR,
				  OMAP2_MCBSP_READ(base, PCR) & ~(SCLKME));
		OMAP2_MCBSP_WRITE(base, SRGR2,
				  OMAP2_MCBSP_READ(base, SRGR2) & ~(CLKSM));

		/* Set the clock polarity */
		if (polarity == OMAP2_MCBSP_CLKS_POLARITY_RISING)
			OMAP2_MCBSP_WRITE(base, SRGR2,
					  OMAP2_MCBSP_READ(base,
							   SRGR2) & ~(CLKSP));
		else if (polarity == OMAP2_MCBSP_CLKS_POLARITY_FALLING) {

		/*
		 * ERRATA 1.49: McBSP master operation at low voltage is only possible if CLKSP=0
		 * In Master mode, if client driver tries to configure input
		 * clock polarity as falling edge, we force it to Rising edge
		 */
			if(omap2_mcbsp_info[mcbsp_id].interface_mode == OMAP2_MCBSP_MASTER){
		 		printk(KERN_WARNING "OMAP2_McBSP: Due to ERRATA 1.49 CLKSP is always set to RISING edge");
				OMAP2_MCBSP_WRITE(base, SRGR2,
					OMAP2_MCBSP_READ(base, SRGR2) & ~(CLKSP));
			}
			else{
				OMAP2_MCBSP_WRITE(base, SRGR2,
					OMAP2_MCBSP_READ(base, SRGR2) |(CLKSP));
			}
		} else {
			printk(KERN_ERR "OMAP2_McBSP: Invalid CLKS polarity\n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EINVAL);
			return -EINVAL;
		}
		break;

		/* McBSPn_FCLK clock */
	case OMAP2_MCBSP_SRGCLKSRC_FCLK:
		/* SRG SRC is selected to be FCLK,
		   in turn FCLK src can be clks/clkr/clkx/prcm96Mhz */
		/* PCR[SCLKME] = 0 SRGR2[CLKSM] =1 */
		OMAP2_MCBSP_WRITE(base, PCR,
				  OMAP2_MCBSP_READ(base, PCR) & ~(SCLKME));
		OMAP2_MCBSP_WRITE(base, SRGR2,
				  OMAP2_MCBSP_READ(base, SRGR2) | (CLKSM));

		/* if its prcm96Mhz then we know the clk_rate */
		if (omap2_mcbsp_info[mcbsp_id].fclksrc ==
		    OMAP2_MCBSP_FCLKSRC_PRCM) {
			clk_rate = clk_get_rate(omap2_mcbsp_info[mcbsp_id].fck);
		}
		/* No polarity for FCLK */

		break;

		/* Signal on the mcbspn_clkr pin */
	case OMAP2_MCBSP_SRGCLKSRC_CLKR:
		/* PCR[SCLKME] = 1 SRGR2[CLKSM] =0 */
		OMAP2_MCBSP_WRITE(base, PCR,
				  OMAP2_MCBSP_READ(base, PCR) | (SCLKME));
		OMAP2_MCBSP_WRITE(base, SRGR2,
				  OMAP2_MCBSP_READ(base, SRGR2) & ~(CLKSM));

		/* Set the clock polarity */
		if (polarity == OMAP2_MCBSP_CLKR_POLARITY_FALLING)
			OMAP2_MCBSP_WRITE(base, PCR,
					  OMAP2_MCBSP_READ(base,
							   PCR) & ~(CLKRP));
		else if (polarity == OMAP2_MCBSP_CLKR_POLARITY_RISING) {
			OMAP2_MCBSP_WRITE(base, PCR,
					  OMAP2_MCBSP_READ(base,
							   PCR) | (CLKRP));
		} else {
			printk(KERN_ERR "OMAP2_McBSP: Invalid CLKR polarity\n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EINVAL);
			return -EINVAL;
		}
		break;

		/* Signal on the mcbspn_clkx pin        */
	case OMAP2_MCBSP_SRGCLKSRC_CLKX:
		/* PCR[SCLKME] = 1 SRGR2[CLKSM] =1 */
		OMAP2_MCBSP_WRITE(base, PCR,
				  OMAP2_MCBSP_READ(base, PCR) | (SCLKME));
		OMAP2_MCBSP_WRITE(base, SRGR2,
				  OMAP2_MCBSP_READ(base, SRGR2) | (CLKSM));

		/* Set the clock polarity */
		if (polarity == OMAP2_MCBSP_CLKX_POLARITY_RISING)
			OMAP2_MCBSP_WRITE(base, PCR,
					  OMAP2_MCBSP_READ(base,
							   PCR) & ~(CLKXP));
		else if (polarity == OMAP2_MCBSP_CLKX_POLARITY_FALLING) {
			OMAP2_MCBSP_WRITE(base, SRGR2,
					  OMAP2_MCBSP_READ(base,
							   PCR) | (CLKXP));
		} else {
			printk(KERN_ERR "OMAP2_McBSP: Invalid CLKX polarity\n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EINVAL);
			return -EINVAL;
		}
		break;

	default:
		printk(KERN_ERR "OMAP2_McBSP: Invalid SRG source\n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	OMAP2_MCBSP_WRITE(base, SRGR1,
			  (OMAP2_MCBSP_READ(base, SRGR1) & ~(CLKGDV(0xFF))));
	if (omap2_mcbsp_info[mcbsp_id].interface_mode == OMAP2_MCBSP_MASTER) {
		if (clk_rate == 0) {
			printk(KERN_ERR "OMAP2_McBSP: Invalid Clock Rate \n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EINVAL);
			return -EINVAL;
		}

		/* Write the Clock Divisor value */

		clkgdv = clk_rate / (sample_rate * (bits_per_sample - 1));
		if (clkgdv) {
			OMAP2_MCBSP_WRITE(base, SRGR1,
					  OMAP2_MCBSP_READ(base,
							   SRGR1) |
					  CLKGDV(clkgdv));
		} else {
			printk(KERN_ERR "OMAP2_McBSP: Invalid clock divisor\n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EINVAL);
			return -EINVAL;
		}
	}

	/* Select the SRG Clock Synchronization mode */
	if (sync_mode == OMAP2_MCBSP_SRG_FREERUNNING) {
		OMAP2_MCBSP_WRITE(base, SRGR2,
				  OMAP2_MCBSP_READ(base, SRGR2) & ~(GSYNC));
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) | (FREE));
	} else if (sync_mode == OMAP2_MCBSP_SRG_RUNNING) {
		OMAP2_MCBSP_WRITE(base, SRGR2,
				  OMAP2_MCBSP_READ(base, SRGR2) | (GSYNC));
		OMAP2_MCBSP_WRITE(base, SPCR2,
				  OMAP2_MCBSP_READ(base, SPCR2) & ~(FREE));
	} else {
		printk(KERN_ERR "OMAP2_McBSP: Invalid SRG sync mode \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return (0);
}

/*
 * Callback routine for Receive DMA
 */
static
void omap2_mcbsp_rxdmacb(int chainid, u16 ch_status, void *data)
{
	u32 mcbsp_id;
	int ret = 0;
	FN_IN;
	mcbsp_id = omap_get_mcbspid[chainid];
	/* If we are at the last transfer, Shut down the reciever */
	if ((omap2_mcbsp_info[mcbsp_id].auto_reset & OMAP2_MCBSP_AUTO_RRST)
	    && (omap_dma_chain_status(chainid) == OMAP_DMA_CHAIN_INACTIVE)) {
		ret =
		    mcbsp_rrst_set(omap2_mcbsp_info[mcbsp_id].virt_base,
				   OMAP2_MCBSP_RRST_DISABLE);
		if (unlikely(ret < 0)) {
			printk(KERN_ERR
			       "RESET DISABLE FAILED IN CALLBACK ignoring %d\n",
			       ret);
		}
	}

	if (omap2_mcbsp_info[mcbsp_id].rx_dma_cb != NULL) {
		omap2_mcbsp_info[mcbsp_id].rx_dma_cb(ch_status, data);
	}
}

/*
 * Callback routine for Transmit DMA
 */
static
void omap2_mcbsp_txdmacb(int chainid, u16 ch_status, void *data)
{
	u32 mcbsp_id;
	int ret = 0;
	mcbsp_id = omap_get_mcbspid[chainid];

	/* If we are at the last transfer, Shut down the Transmitter */
	if ((omap2_mcbsp_info[mcbsp_id].auto_reset & OMAP2_MCBSP_AUTO_XRST)
	    && (omap_dma_chain_status(chainid) == OMAP_DMA_CHAIN_INACTIVE)) {
		ret =
		    mcbsp_xrst_set(omap2_mcbsp_info[mcbsp_id].virt_base,
				   OMAP2_MCBSP_XRST_DISABLE);
		if (unlikely(ret < 0)) {
			printk(KERN_ERR
			       "RESET DISABLE FAILED IN CALLBACK ignoring %d\n",
			       ret);
		}
	}
	if (omap2_mcbsp_info[mcbsp_id].tx_dma_cb != NULL) {
		omap2_mcbsp_info[mcbsp_id].tx_dma_cb(ch_status, data);
	}
}

/*
 * Configure the receiver parameters
 * mcbsp_id	: McBSP Interface ID
 * rp		: Receive parameters
 */
int omap2_mcbsp_set_recv_params(u32 mcbsp_id, omap2_mcbsp_transfer_params * rp)
{
	u32 base;
	int err, chain_id = -1;
	u32 w_rcr1 = 0, w_rcr2 = 0, dt = 0;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 mcbsp_fifo_size;
#endif
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}
#ifdef CONFIG_USE_MCBSP_FIFO
	if ( mcbsp_id == OMAP2_MCBSP_INTERFACE2)
		mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
	else
		mcbsp_fifo_size = MCBSP_FIFO_SIZE;
#endif

	/* Configure the DMA parameters */

	/* Override default values?? */
	if (rp->data_type != OMAP2_MCBSP_WORDLEN_NONE) {
		dt = rp->data_type;
	} else {
		dt = rp->word_length1;
	}

	/* Need to check for other parameters too */

	if (dt == OMAP2_MCBSP_WORDLEN_8)
		omap2_mcbsp_info[mcbsp_id].rx_param.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt <= OMAP2_MCBSP_WORDLEN_16 && dt > OMAP2_MCBSP_WORDLEN_8)
		omap2_mcbsp_info[mcbsp_id].rx_param.data_type = OMAP_DMA_DATA_TYPE_S16;
	else
		omap2_mcbsp_info[mcbsp_id].rx_param.data_type = OMAP_DMA_DATA_TYPE_S32;

	omap2_mcbsp_info[mcbsp_id].rx_param.read_prio = DMA_CH_PRIO_HIGH;
	omap2_mcbsp_info[mcbsp_id].rx_param.write_prio = DMA_CH_PRIO_HIGH;

	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, DMA_DEFAULT_FIFO_DEPTH,0);

	omap2_mcbsp_info[mcbsp_id].rx_param.elem_count = 1;
	omap2_mcbsp_info[mcbsp_id].rx_param.frame_count = 1;
/* If McBSP FIFO is used, do a packet sync DMA */
#ifdef CONFIG_USE_MCBSP_FIFO
	omap2_mcbsp_info[mcbsp_id].rx_param.sync_mode = OMAP_DMA_SYNC_PACKET;
	omap2_mcbsp_info[mcbsp_id].rx_param.src_fi = mcbsp_fifo_size;
#else
	omap2_mcbsp_info[mcbsp_id].rx_param.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	omap2_mcbsp_info[mcbsp_id].rx_param.src_fi = 0;
#endif
	omap2_mcbsp_info[mcbsp_id].rx_param.trigger = omap2_mcbsp_dma_rx[mcbsp_id];
	omap2_mcbsp_info[mcbsp_id].rx_param.src_or_dst_synch = 0x01;
	omap2_mcbsp_info[mcbsp_id].rx_param.src_amode = OMAP_DMA_AMODE_CONSTANT;
	omap2_mcbsp_info[mcbsp_id].rx_param.src_start = omap2_mcbsp_drr[mcbsp_id];
	omap2_mcbsp_info[mcbsp_id].rx_param.src_ei = 0x0;
	omap2_mcbsp_info[mcbsp_id].rx_param.dst_start = 0x0;	/* buffer start address will be provided in receive */
	/* Indexing is always in bytes - so multiply with dt */
	dt = (omap2_mcbsp_info[mcbsp_id].rx_param.data_type == OMAP_DMA_DATA_TYPE_S8) ? 1 :
	    (omap2_mcbsp_info[mcbsp_id].rx_param.data_type == OMAP_DMA_DATA_TYPE_S16) ? 2 : 4;
	if (rp->skip_alt == OMAP2_MCBSP_SKIP_SECOND) {
		printk(" receive second\n");
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_ei = (1);
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_fi = (1) + ((-1) * dt);
	} else if (rp->skip_alt == OMAP2_MCBSP_SKIP_FIRST) {
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_ei = 1 + (-2) * dt;
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_fi = 1 + (2) * dt;
	} else if (rp->skip_alt == OMAP2_MCBSP_SKIP_ADD) {
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_ei = 1;
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_fi = 1 + (1) * dt;
	} else {
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_amode = OMAP_DMA_AMODE_POST_INC;
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_ei = 0;
		omap2_mcbsp_info[mcbsp_id].rx_param.dst_fi = 0;
	}
	D2(KERN_INFO "rx DI ei=%d, fi=%d, dt=%d,amode=0x%x\n", omap2_mcbsp_info[mcbsp_id].rx_param.src_ei,
	   omap2_mcbsp_info[mcbsp_id].rx_param.src_fi, dt, omap2_mcbsp_info[mcbsp_id].rx_param.src_amode);

	omap2_mcbsp_info[mcbsp_id].rxskip_alt &=
	    ~(OMAP2_MCBSP_SKIP_FIRST | OMAP2_MCBSP_SKIP_SECOND|OMAP2_MCBSP_SKIP_ADD);
	omap2_mcbsp_info[mcbsp_id].rxskip_alt |=
	    (rp->skip_alt & (OMAP2_MCBSP_SKIP_FIRST | OMAP2_MCBSP_SKIP_SECOND|OMAP2_MCBSP_SKIP_ADD));
	omap2_mcbsp_info[mcbsp_id].auto_reset &= ~OMAP2_MCBSP_AUTO_RRST;
	omap2_mcbsp_info[mcbsp_id].auto_reset |=
	    (rp->auto_reset & OMAP2_MCBSP_AUTO_RRST);

	omap2_mcbsp_info[mcbsp_id].rx_data_type = omap2_mcbsp_info[mcbsp_id].rx_param.data_type << 0x1;
	if (omap2_mcbsp_info[mcbsp_id].rx_param.data_type == 0)
		omap2_mcbsp_info[mcbsp_id].rx_data_type = 1;

	omap2_mcbsp_info[mcbsp_id].rx_dma_cb = rp->callback;
	omap2_mcbsp_info[mcbsp_id].rx_config_done = 0;

	/* Based on RJUST field the DMA parameters can be configured to operate in Double index mode */

	/* request for a chain of dma channels for data reception through this interface */
	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx == -1) {
		err =
		    omap_request_dma_chain(mcbsp_id,
					   omap2_mcbsp_info[mcbsp_id].
					   name, omap2_mcbsp_rxdmacb,
					   &chain_id,
					   omap2_mcbsp_max_dmachs_rx
					   [mcbsp_id],
					   OMAP_DMA_DYNAMIC_CHAIN, omap2_mcbsp_info[mcbsp_id].rx_param);
		if (err < 0) {
			D2("Receive path configuration failed \n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EPERM);
			return -EPERM;
		}
		omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx = chain_id;
		omap_get_mcbspid[chain_id] = mcbsp_id;
		omap2_mcbsp_info[mcbsp_id].rxdmachain_state = 0;
	} else {
		/* DMA params already set, modify the same!! */
		err =
		    omap_modify_dma_chain_params(omap2_mcbsp_info[mcbsp_id].
						 dma_chain_id_rx, omap2_mcbsp_info[mcbsp_id].rx_param);
		if (err < 0) {
			D2("DMA reconfiguration failed\n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EPERM);
			return -EPERM;
		}

	}

	/* Write phase */
	if (rp->phase == OMAP2_MCBSP_FRAME_SINGLEPHASE) {
		w_rcr2 = w_rcr2 & ~(RPHASE);
	} else {
		w_rcr2 = w_rcr2 | (RPHASE);

		/* if dual phase then we have to take frlen2 and wordlength2 also into account */
		w_rcr2 =
		    w_rcr2 | RWDLEN2(rp->word_length2) | RFRLEN2(rp->
								 frame_length2);
	}

	/* Write data delay and reverse params */
	if (rp->fig == 0)
		w_rcr2 = w_rcr2 & ~(RFIG);
	else
		w_rcr2 = w_rcr2 | (RFIG);

	w_rcr2 =
	    w_rcr2 | RREVERSE(rp->reverse_compand) | RDATDLY(rp->data_delay);
	w_rcr1 =
	    w_rcr1 | RWDLEN1(rp->word_length1) | RFRLEN1(rp->frame_length1);

	OMAP2_MCBSP_WRITE(base, RCR1, w_rcr1);
	OMAP2_MCBSP_WRITE(base, RCR2, w_rcr2);

	OMAP2_MCBSP_WRITE(base, SPCR1,
			  (OMAP2_MCBSP_READ(base, SPCR1) & ~RJUST(0x3)));

	OMAP2_MCBSP_WRITE(base, SPCR1, OMAP2_MCBSP_READ(base, SPCR1) |
			  RJUST(rp->justification));

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Configure the transmitter parameters
 * mcbsp_id	: McBSP Interface ID
 * tp		: Transfer parameters
 */

int omap2_mcbsp_set_trans_params(u32 mcbsp_id, omap2_mcbsp_transfer_params * tp)
{
	u32 base;
	struct omap_dma_channel_params params;
	int err = 0, chain_id = -1;
	u32 w_xcr1 = 0, w_xcr2 = 0;
	int ret;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 mcbsp_fifo_size;
#endif
	u32 dt = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (tp->word_length1 < OMAP2_MCBSP_WORDLEN_8 &&
	    tp->word_length1 > OMAP2_MCBSP_WORDLEN_32) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid tx word_length \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
#ifdef CONFIG_USE_MCBSP_FIFO
	if ( mcbsp_id == OMAP2_MCBSP_INTERFACE2)
		mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
	else
		mcbsp_fifo_size = MCBSP_FIFO_SIZE;
#endif

	/* Override default values?? */
	if (tp->data_type != OMAP2_MCBSP_WORDLEN_NONE) {
		dt = tp->data_type;
	} else {
		dt = tp->word_length1;
	}
	if (dt == OMAP2_MCBSP_WORDLEN_8)
		params.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt <= OMAP2_MCBSP_WORDLEN_16 && dt > OMAP2_MCBSP_WORDLEN_8)
		params.data_type = OMAP_DMA_DATA_TYPE_S16;
	else
		params.data_type = OMAP_DMA_DATA_TYPE_S32;

	params.read_prio = DMA_CH_PRIO_HIGH;
	params.write_prio = DMA_CH_PRIO_HIGH;

	omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, DMA_DEFAULT_FIFO_DEPTH,0);

	/* we will modify the following when we do the actual data transfer */
	params.elem_count = 2;
	params.frame_count = 2;
/* IF McBSP FIFO is used, use packet sync DMA*/
#ifdef CONFIG_USE_MCBSP_FIFO
	params.sync_mode = OMAP_DMA_SYNC_PACKET;
	params.dst_fi = mcbsp_fifo_size;
#else
	params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	params.dst_fi = 0;
#endif
	params.trigger = omap2_mcbsp_dma_tx[mcbsp_id];
	params.src_or_dst_synch = 0;
	params.src_start = 0;	/* buffer pointer to be provided in send */
	/* Indexing is always in bytes - so multiply with dt */
	omap2_mcbsp_info[mcbsp_id].tx_data_type = params.data_type << 0x1;
	if (params.data_type == 0)
		omap2_mcbsp_info[mcbsp_id].tx_data_type = 1;
	dt = omap2_mcbsp_info[mcbsp_id].tx_data_type;
	if (tp->skip_alt == OMAP2_MCBSP_SKIP_SECOND) {
		params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		params.src_ei = (1);
		params.src_fi = (1) + ((-1) * dt);
	} else if (tp->skip_alt == OMAP2_MCBSP_SKIP_FIRST) {
		params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		params.src_ei = 1 + (-2) * dt;
		params.src_fi = 1 + (2) * dt;
	} else {
		params.src_amode = OMAP_DMA_AMODE_POST_INC;
		params.src_ei = 0;
		params.src_fi = 0;
	}
	D2(KERN_INFO "tx DI ei=%d, fi=%d, dt=%d,amode=0x%x\n", params.src_ei,
	   params.src_fi, dt, params.src_amode);
	params.dst_amode = OMAP_DMA_AMODE_CONSTANT;
	params.dst_start = omap2_mcbsp_dxr[mcbsp_id];
	params.dst_ei = 0;

	omap2_mcbsp_info[mcbsp_id].txskip_alt &=
	    ~(OMAP2_MCBSP_SKIP_FIRST | OMAP2_MCBSP_SKIP_SECOND);
	omap2_mcbsp_info[mcbsp_id].txskip_alt |=
	    (tp->skip_alt & (OMAP2_MCBSP_SKIP_FIRST | OMAP2_MCBSP_SKIP_SECOND));
	omap2_mcbsp_info[mcbsp_id].auto_reset &= ~OMAP2_MCBSP_AUTO_XRST;
	omap2_mcbsp_info[mcbsp_id].auto_reset |=
	    (tp->auto_reset & OMAP2_MCBSP_AUTO_XRST);

	omap2_mcbsp_info[mcbsp_id].tx_dma_cb = tp->callback;

	/* Based on Rjust we can do a double indexing DMA params configuration here */

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx == -1) {
		err =
		    omap_request_dma_chain(mcbsp_id,
					   omap2_mcbsp_info[mcbsp_id].
					   name, omap2_mcbsp_txdmacb,
					   &chain_id,
					   omap2_mcbsp_max_dmachs_tx
					   [mcbsp_id],
					   OMAP_DMA_DYNAMIC_CHAIN, params);
		if (err < 0) {
			D2(KERN_ERR
			   "Transmit path configuration failed \n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			return -EPERM;
		}
		omap2_mcbsp_info[mcbsp_id].txdmachain_state = 0;
		omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx = chain_id;
		omap_get_mcbspid[chain_id] = mcbsp_id;
	} else {
		/* DMA params already set, modify the same!! */
		err =
		    omap_modify_dma_chain_params(omap2_mcbsp_info[mcbsp_id].
						 dma_chain_id_tx, params);
		if (err < 0) {
			D2(KERN_ERR "DMA reconfiguration failed\n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			return -EPERM;
		}

	}
#ifdef CONFIG_USE_MCBSP_FIFO
	OMAP2_MCBSP_WRITE(base,THRSH2,(mcbsp_fifo_size - 1));
#endif

	/* Write phase */
	if (tp->phase == OMAP2_MCBSP_FRAME_SINGLEPHASE) {
		w_xcr2 = w_xcr2 & ~(XPHASE);
	} else {
		w_xcr2 = w_xcr2 | (XPHASE);

		/* if dual phase then we have to take frlen2 and wordlength2 also into account */
		w_xcr2 =
		    w_xcr2 | XWDLEN2(tp->word_length2) | XFRLEN2(tp->
								 frame_length2);
	}

	/* Write data delay and reverse params */

	if (tp->fig == 0)
		w_xcr2 = w_xcr2 & ~(XFIG);
	else
		w_xcr2 = w_xcr2 | (XFIG);

	w_xcr2 =
	    w_xcr2 | XREVERSE(tp->reverse_compand) | XDATDLY(tp->data_delay);
	w_xcr1 =
	    w_xcr1 | XWDLEN1(tp->word_length1) | XFRLEN1(tp->frame_length1);

	OMAP2_MCBSP_WRITE(base, XCR1, w_xcr1);
	OMAP2_MCBSP_WRITE(base, XCR2, w_xcr2);

	OMAP2_MCBSP_WRITE(base, SPCR1,
			  (OMAP2_MCBSP_READ(base, SPCR1) & ~RJUST(0x3)));
	OMAP2_MCBSP_WRITE(base, SPCR1,
			  OMAP2_MCBSP_READ(base,
					   SPCR1) | RJUST(tp->justification));

	/* DXENA and DXENDLY configurations will be added heree */
	if (tp->dxena == OMAP2_MCBSP_DXEN_DISABLE) {
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) & ~(DXENA));
	} else if (tp->dxena == OMAP2_MCBSP_DXEN_DISABLE) {
		OMAP2_MCBSP_WRITE(base, SPCR1,
				  OMAP2_MCBSP_READ(base, SPCR1) | (DXENA));
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
		OMAP2_MCBSP_WRITE(base, XCCR,
				  (OMAP2_MCBSP_READ(base, XCCR) &
				   ~DXENDLY(0x3)));
		OMAP2_MCBSP_WRITE(base, XCCR, DXENDLY(tp->dxendly));
#endif

	}

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(err);
	return err;
}

/*
 * Start receving data on a McBSP interface
 * mcbsp_id		: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The destination address [NOTE: This should be a physical address]
 * buf_size		: Buffer size
 */

int omap2_mcbsp_receive_data(u32 mcbsp_id, void *cbdata,
			     dma_addr_t buf_start_addr, u32 buf_size)
{
	u32 dma_chain_status = 0;
	int ret;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 base;
	u32 thrsh1=256;/* lowest value for McBSP threshold*/
	u32 mcbsp_fifo_size;
	int err;
#endif
	u8 enable = 0;
	int e_count = 0;
	int f_count = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}
#ifdef CONFIG_USE_MCBSP_FIFO
	base = omap2_mcbsp_info[mcbsp_id].virt_base;
#endif
	/* Auto RRST handling logic - disable the Reciever before 1st dma */
	if ((omap2_mcbsp_info[mcbsp_id].auto_reset & OMAP2_MCBSP_AUTO_RRST)
	    &&
	    (omap_dma_chain_status(omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx)
	     == OMAP_DMA_CHAIN_INACTIVE)) {
		ret =
		    mcbsp_rrst_set(omap2_mcbsp_info[mcbsp_id].virt_base,
				   OMAP2_MCBSP_RRST_DISABLE);
		if (unlikely(ret)) {
			D2(KERN_ERR "OMAP2_MCBSP: RRSTSet failed %d\n", ret);
			return (ret);
		}
		enable = 1;
	}


	/* Little bit of smart coding to avoid multiple divisions.
	 * for skip_first and second, we need to set e_count =2, and f_count = number of frames =
	 * number of elements/e_count (number of elements is already computed in e_count, so reuse it..
	 */
	e_count = (buf_size / omap2_mcbsp_info[mcbsp_id].rx_data_type);
	/* IF McBSP FIFO is used, change receive side configuration */
#ifdef CONFIG_USE_MCBSP_FIFO
	if(omap2_mcbsp_info[mcbsp_id].rx_config_done ==0) {
						omap2_mcbsp_info[mcbsp_id].rx_config_done = 1;
						if ( mcbsp_id == OMAP2_MCBSP_INTERFACE2)
							mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
						else
							mcbsp_fifo_size = MCBSP_FIFO_SIZE;

						if ( e_count < mcbsp_fifo_size ) {
								thrsh1=e_count;
						} else {
						/* Find the optimum threshold value for MCBSP
								 to transfer complete data*/
						if((e_count % mcbsp_fifo_size) == 0)
							thrsh1=mcbsp_fifo_size;
						else if((e_count % ((mcbsp_fifo_size * 3)/4)) == 0)
							thrsh1 = (mcbsp_fifo_size * 3)/4;
						else if((e_count % ((mcbsp_fifo_size * 1)/2)) == 0)
							thrsh1 = (mcbsp_fifo_size * 1)/2;
						else if((e_count % ((mcbsp_fifo_size * 1)/4)) == 0)
							thrsh1 = (mcbsp_fifo_size * 1)/4;
						else
							thrsh1=1;
						}
						OMAP2_MCBSP_WRITE(base,THRSH1,(thrsh1-1));

						if (thrsh1 != mcbsp_fifo_size) {
								omap2_mcbsp_info[mcbsp_id].rx_param.src_fi = thrsh1;
								/* if threshold =1, use element sync DMA */
								if(thrsh1 ==1 ) {
									omap2_mcbsp_info[mcbsp_id].rx_param.sync_mode = OMAP_DMA_SYNC_ELEMENT;
									omap2_mcbsp_info[mcbsp_id].rx_param.src_fi = 0;
								}
								err = omap_modify_dma_chain_params(
								omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx,
												omap2_mcbsp_info[mcbsp_id].rx_param);
								if (err < 0) {
								D2("DMA reconfiguration failed\n");
								up(&omap2_mcbsp_info[mcbsp_id].semlock);
								FN_OUT(-EPERM);
								return -EPERM;
								}
					}
		}

#endif
	if (omap2_mcbsp_info[mcbsp_id].rxskip_alt == OMAP2_MCBSP_SKIP_ADD) {
		f_count = e_count;
		e_count = 1;
	} else if (omap2_mcbsp_info[mcbsp_id].rxskip_alt != OMAP2_MCBSP_SKIP_NONE) {
		/* OMAP2_MCBSP_SKIP_FIRST/SECOND */
		/* since the number of frames = total number of elements/element count,
		 * However, with double indexing for data transfers, double the number of elements
		 * need to be transmitted for the complete transfer to happen
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}
	/* If the DMA is to be configured to skip the first byte, we need to jump backwards,
	 * so we need to move one chunk forward and ask dma if we dont want the client driver
	 * knowing abt this.
	 */
	if (omap2_mcbsp_info[mcbsp_id].rxskip_alt == OMAP2_MCBSP_SKIP_FIRST) {
		buf_start_addr += omap2_mcbsp_info[mcbsp_id].rx_data_type;
	}
	dma_chain_status =
	    omap_dma_chain_a_transfer(omap2_mcbsp_info[mcbsp_id].
				      dma_chain_id_rx,
				      omap2_mcbsp_drr[mcbsp_id],
				      buf_start_addr, e_count, f_count, cbdata);

	if (omap2_mcbsp_info[mcbsp_id].rxdmachain_state == 0) {
		dma_chain_status =
		    omap_start_dma_chain_transfers(omap2_mcbsp_info
						   [mcbsp_id].dma_chain_id_rx);
		omap2_mcbsp_info[mcbsp_id].rxdmachain_state = 1;
	}
	/* Auto RRST handling logic - Enable the Reciever after 1st dma */
	if (enable &&
	    (omap_dma_chain_status(omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx)
	     == OMAP_DMA_CHAIN_ACTIVE)) {
		ret =
		    mcbsp_rrst_set(omap2_mcbsp_info[mcbsp_id].virt_base,
				   OMAP2_MCBSP_RRST_ENABLE);
		if (unlikely(ret)) {
			D2(KERN_ERR "OMAP2_MCBSP: RRSTSet failed %d\n", ret);
			return (ret);
		}
	}

	if (dma_chain_status == -EBUSY) {
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EBUSY);
		return -EBUSY;
	} else if (dma_chain_status == -EINVAL) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP data transmission error \n");
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EPERM);
		return -EPERM;
	}
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Start transmitting data through a McBSP interface
 * mcbsp_id		: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The source address [NOTE: This should be a physical address]
 * buf_size		: Buffer size
 */
int omap2_mcbsp_send_data(u32 mcbsp_id, void *cbdata,
			  dma_addr_t buf_start_addr, u32 buf_size)
{
	u32 dma_chain_state = 0;
	int ret;
	u8 enable = 0;
	int e_count = 0;
	int f_count = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}
	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].tx_semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	/* Auto RRST handling logic - disable the Reciever before 1st dma */
	if ((omap2_mcbsp_info[mcbsp_id].auto_reset & OMAP2_MCBSP_AUTO_XRST)
	    &&
	    (omap_dma_chain_status(omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx)
	     == OMAP_DMA_CHAIN_INACTIVE)) {
		ret =
		    mcbsp_xrst_set(omap2_mcbsp_info[mcbsp_id].virt_base,
				   OMAP2_MCBSP_XRST_DISABLE);
		if (unlikely(ret)) {
			D2(KERN_ERR "OMAP2_MCBSP: XRSTSet failed %d\n", ret);
			return (ret);
		}
		enable = 1;
	}

	/* Little bit of smart coding to avoid multiple divisions.
	 * for skip_first and second, we need to set e_count =2, and f_count = number of frames =
	 * number of elements/e_count (number of elements is already computed in e_count, so reuse it..
	 */
	e_count = (buf_size / omap2_mcbsp_info[mcbsp_id].tx_data_type);
	if (omap2_mcbsp_info[mcbsp_id].txskip_alt != OMAP2_MCBSP_SKIP_NONE) {
		/* OMAP2_MCBSP_SKIP_FIRST/SECOND */
		/* since the number of frames = total number of elements/element count,
		 * However, with double indexing for data transfers, double the number of elements
		 * need to be transmitted for the complete transfer to happen
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}
	D2(KERN_INFO "ec=%d fc=%d dt=0x%x bufsz=%d\n", e_count, f_count,
	   omap2_mcbsp_info[mcbsp_id].tx_data_type, buf_size);
	/* If the DMA is to be configured to skip the first byte, we need to jump backwards,
	 * so we need to move one chunk forward and ask dma if we dont want the client driver
	 * knowing abt this.
	 */
	if (omap2_mcbsp_info[mcbsp_id].txskip_alt == OMAP2_MCBSP_SKIP_FIRST) {
		buf_start_addr += omap2_mcbsp_info[mcbsp_id].tx_data_type;
	}
	dma_chain_state =
	    omap_dma_chain_a_transfer(omap2_mcbsp_info[mcbsp_id].
				      dma_chain_id_tx, buf_start_addr,
				      omap2_mcbsp_dxr[mcbsp_id],
				      e_count, f_count, cbdata);

	if (omap2_mcbsp_info[mcbsp_id].txdmachain_state == 0) {
		dma_chain_state =
		    omap_start_dma_chain_transfers(omap2_mcbsp_info
						   [mcbsp_id].dma_chain_id_tx);
		omap2_mcbsp_info[mcbsp_id].txdmachain_state = 1;
	}

	if (dma_chain_state == -EBUSY) {
		up(&omap2_mcbsp_info[mcbsp_id].tx_semlock);
		FN_OUT(-EBUSY);
		return -EBUSY;
	} else if (dma_chain_state == -EINVAL) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP data transmission error \n");
		up(&omap2_mcbsp_info[mcbsp_id].tx_semlock);
		FN_OUT(-EPERM);
		return -EPERM;
	}
	/* Auto XRST handling logic - Enable the Reciever after 1st dma */
	if (enable &&
	    (omap_dma_chain_status(omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx)
	     == OMAP_DMA_CHAIN_ACTIVE)) {
		ret =
		    mcbsp_xrst_set(omap2_mcbsp_info[mcbsp_id].virt_base,
				   OMAP2_MCBSP_XRST_ENABLE);
		if (unlikely(ret)) {
			D2(KERN_ERR "OMAP2_MCBSP: XRSTSet failed %d\n", ret);
			return (ret);
		}
	}
	up(&omap2_mcbsp_info[mcbsp_id].tx_semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Stop transmitting data on a McBSP interface
 * mcbsp_id	: McBSP interface ID
 */
int omap2_mcbsp_stop_datatx(u32 mcbsp_id)
{
	int ret;
	FN_IN;
	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx != -1) {
		if (omap_stop_dma_chain_transfers
		    (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx) != 0) {
			D2(KERN_ERR
			   "OMAP McBSP internal error unable to stop tx dma chain \n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EPERM);
			return -EPERM;
		}
	} else {
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EPERM);
		return -EPERM;
	}
	/* we might want to restart the transfer */
	omap2_mcbsp_info[mcbsp_id].txdmachain_state = 0;
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Stop receving data on a McBSP interface
 * mcbsp_id	: McBSP interface ID
 */
int omap2_mcbsp_stop_datarx(u32 mcbsp_id)
{
	int ret;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx != -1) {
		if (omap_stop_dma_chain_transfers
		    (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx) != 0) {
			D2(KERN_ERR
			   "OMAP McBSP internal error unable to stop rx dma chain \n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EPERM);
			return -EPERM;
		}
	} else {
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EPERM);
		return -EPERM;
	}
	/* we might want to restart the transfer */
	omap2_mcbsp_info[mcbsp_id].rxdmachain_state = 0;
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(0);
	return 0;
}

/*
 * Get the element index and frame index of transmitter
 * mcbsp_id	: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap2_mcbsp_transmitter_index(int mcbsp_id, int *ei, int *fi)
{
	int eix = 0, fix = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		goto txinx_err;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}
	if ((!ei) || (!fi)) {
		D2(KERN_ERR
		   "OMAP2_McBSP: Invalid ei and fi params in trasmitter_index\n");
		goto txinx_err;
	}

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx == -1) {
		D2(KERN_ERR "OMAP2_McBSP: Transmitter not started\n");
		goto txinx_err;
	}

	if (omap_get_dma_chain_index
	    (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx, &eix, &fix) != 0) {
		D2(KERN_ERR "OMAP2_McBSP: Getting chain index failed\n");
		goto txinx_err;
	}

	*ei = eix;
	*fi = fix;

	FN_OUT(0);
	return 0;

      txinx_err:
	FN_OUT(-EINVAL);
	return -EINVAL;
}

/* Get the transmitter destination position */
int omap2_mcbsp_transmitter_destpos(int mcbsp_id)
{
	FN_IN;
	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx == -1) {
		D2(KERN_ERR "OMAP2_McBSP: Transmitter not started\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	FN_OUT(0);
	return (omap_get_dma_chain_dst_pos
		(omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx));
}

/* Get the transmitter source position */
int omap2_mcbsp_transmitter_srcpos(int mcbsp_id)
{
	FN_IN;
	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx == -1) {
		D2(KERN_ERR "OMAP2_McBSP: Transmitter not started\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	FN_OUT(0);
	return (omap_get_dma_chain_src_pos
		(omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx));
}

/*
 * Get the element index and frame index of receiver
 * mcbsp_id	: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap2_mcbsp_receiver_index(int mcbsp_id, int *ei, int *fi)
{
	int eix = 0, fix = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		goto rxinx_err;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	/* Check if ei and fi are valid */
	if ((!ei) || (!fi)) {
		D2(KERN_ERR
		   "OMAP2_McBSP: Invalid ei and fi params in receiver_index\n");
		goto rxinx_err;
	}

	/* Check if chain exists */
	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx == -1) {
		D2(KERN_ERR "OMAP2_McBSP: Receiver not started\n");
		goto rxinx_err;
	}

	/* Get dma_chain_index */
	if (omap_get_dma_chain_index
	    (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx, &eix, &fix) != 0) {
		D2(KERN_ERR "OMAP2_McBSP: Getting chain index failed\n");
		goto rxinx_err;
	}

	*ei = eix;
	*fi = fix;

	FN_OUT(0);
	return 0;

      rxinx_err:
	FN_OUT(-EINVAL);
	return -EINVAL;
}

/* Get the receiver destination position */
int omap2_mcbsp_receiver_destpos(int mcbsp_id)
{
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx == -1) {
		D2(KERN_ERR "OMAP2_McBSP: Receiver not started\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	FN_OUT(0);
	return (omap_get_dma_chain_dst_pos
		(omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx));
}

/* Get the receiver source position */
int omap2_mcbsp_receiver_srcpos(int mcbsp_id)
{
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx == -1) {
		D2(KERN_ERR "OMAP2_McBSP: Receiver not started\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	FN_OUT(0);
	return (omap_get_dma_chain_src_pos
		(omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx));
}

/*
 * Master ISR, centralized interrupt handler for all the mcbsp interfaces
 */
static irqreturn_t
omap2_mcbsp_master_isr(int irq, void *interface)
{
	u32 mcbsp_id = (u32) interface;
	u32 irqstatus = 0;
	u32 base;
	unsigned long flags = 0;
	FN_IN;

	/* A unlikely interrupt */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return(-EINVAL);
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EINVAL);
		return(-EINVAL);
	}

	spin_lock_irqsave(&omap2_mcbsp_info[mcbsp_id].lock, flags);

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	/* if an ISR is registered for this interface, execute the same */
	if (omap2_mcbsp_info[mcbsp_id].isr != NULL) {
		irqstatus = OMAP2_MCBSP_READ(base, IRQSTATUS);
		OMAP2_MCBSP_WRITE(base, IRQSTATUS, irqstatus);
		omap2_mcbsp_info[mcbsp_id].
		    isr(omap2_mcbsp_info[mcbsp_id].irqargs, irqstatus);
	}

	/* acknowledge the interrupt */
	OMAP2_MCBSP_WRITE(base, IRQSTATUS, irqstatus);
	spin_unlock_irqrestore(&omap2_mcbsp_info[mcbsp_id].lock, flags);
	//FN_OUT(IRQ_HANDLED);
	return IRQ_HANDLED;
}

/*
 * Register a ISR for McBSP interrupts.
 * mcbsp_id	:	McBSP interface ID
 * isr		: 	isr to be invoked for an interrupt
 * arg		:	user data to be passed as a parameter to ISR
 * mask		: 	Interrupts to be enabled
 */

int omap2_mcbsp_register_isr(u32 mcbsp_id, omap2_mcbsp_isr_t isr,
			     void *arg, unsigned int mask)
{
	u32 base;
	unsigned long flags = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	if (isr == NULL) {
		D2(KERN_ERR
		   "OMAP2_McBSP: ISR is not registered for %d McBSP interface\n",
		   mcbsp_id);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/*
	 * We can have the master irq registered multiple times with irq core by
	 * using SA_SHIRQ flag. Better to force a unregister before registering.
	 */
	if (omap2_mcbsp_info[mcbsp_id].isr != NULL) {
		D2(KERN_ERR
		   "OMAP2_McBSP: ISR is already registered for %d McBSP interface\n",
		   mcbsp_id);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	base = omap2_mcbsp_info[mcbsp_id].virt_base;

	/* request and register a IRQ handler for this interface */
	if (request_irq
	    (omap2_mcbsp_irq[mcbsp_id], omap2_mcbsp_master_isr,
	     IRQF_SHARED, "omap2 mcbsp irq", (void *)mcbsp_id)) {
		D2(KERN_WARNING
		   "omap2 mcbsp IRQ %d registration for interface %d failed\n",
		   omap2_mcbsp_irq[mcbsp_id], mcbsp_id);
		FN_OUT(-EPERM);
		return -EPERM;
	}

	spin_lock_irqsave(&omap2_mcbsp_info[mcbsp_id].lock, flags);

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
	/* Clear all the previous interrupts */
	OMAP2_MCBSP_WRITE(base, IRQSTATUS, 0xFFFFFFFF);
#endif

	/* register isr and args */
	omap2_mcbsp_info[mcbsp_id].isr = isr;
	omap2_mcbsp_info[mcbsp_id].irqargs = arg;
	omap2_mcbsp_info[mcbsp_id].irqmask = mask;

	/* Enable the interrupts */
	OMAP2_MCBSP_WRITE(base, IRQENABLE, mask);

	spin_unlock_irqrestore(&omap2_mcbsp_info[mcbsp_id].lock, flags);
	FN_OUT(0);
	return 0;
}

/*
 * Unregister an ISR routine
 * mcbsp_id	: McBSP interface ID
 *
 * McBSPn IRQ will be disabled
 */
int omap2_mcbsp_unregister_isr(u32 mcbsp_id)
{
	u32 base;
	unsigned long flags = 0;
	FN_IN;

	/* Check if mcbsp interface is valid and is reserved */
	if (OMAP2_McBSP_CHECKID(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	if (omap2_mcbsp_info[mcbsp_id].isr == NULL) {
		D2(KERN_ERR "OMAP2_McBSP: ISR not found\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	base = omap2_mcbsp_info[mcbsp_id].virt_base;
	spin_lock_irqsave(&omap2_mcbsp_info[mcbsp_id].lock, flags);

	free_irq(omap2_mcbsp_irq[mcbsp_id], (void *)mcbsp_id);

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430)
	/* Clear all the previous interrupts */
	OMAP2_MCBSP_WRITE(base, IRQSTATUS, 0xFFFFFFFF);
#endif

	/* register isr and args */
	omap2_mcbsp_info[mcbsp_id].isr = NULL;
	omap2_mcbsp_info[mcbsp_id].irqargs = NULL;
	omap2_mcbsp_info[mcbsp_id].irqmask = 0x00000000;

	/* Disable the interrupts */
	OMAP2_MCBSP_WRITE(base, IRQENABLE, 0x00000000);

	spin_unlock_irqrestore(&omap2_mcbsp_info[mcbsp_id].lock, flags);

	FN_OUT(0);
	return 0;
}

/*
 * Request / reserve a MCBSP interface
 * mcbsp_id		: McBSP interface ID
 * interface mode	: Master / Slave
 * fclk_source		: External or Internal (PRCM) clock
 *
 * Interface reset is done after reserving.
 */
int omap2_mcbsp_request_interface(u32 mcbsp_id, u8 interface_mode,
				  u8 fclk_source)
{
	int ret,ret_err;
	FN_IN;
	/* Check if mcbsp interface is valid */
	if ((mcbsp_id > (OMAP2_MAX_MCBSP_COUNT - 1))
	    || (mcbsp_id < OMAP2_MCBSP_INTERFACE1)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface ID\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* if already reserved */
	if (omap2_mcbsp_info[mcbsp_id].free == 0) {
		D2(KERN_ERR
		   "OMAP2_McBSP: McBSP interface %d already reserved \n",
		   mcbsp_id);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/*
	 * if the interface operating mode doesnt belong to Master or Slave return error
	 */
	if ((interface_mode != OMAP2_MCBSP_MASTER) &&
	    (interface_mode != OMAP2_MCBSP_SLAVE)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid interface mode \n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}
	if (interface_mode == OMAP2_MCBSP_MASTER)
		omap2_mcbsp_info[mcbsp_id].master_mode = 1;
	else
		omap2_mcbsp_info[mcbsp_id].master_mode = 0;

	/* Get the interface clock for the MCBSP interface */
	omap2_mcbsp_info[mcbsp_id].ick =
#ifdef CONFIG_TRACK_RESOURCES
	    clk_get(&mcbsp_device, omap2_mcbsp_ick[mcbsp_id]);
#else
	    clk_get(NULL, omap2_mcbsp_ick[mcbsp_id]);
#endif
	if (IS_ERR(omap2_mcbsp_info[mcbsp_id].ick)) {
		omap2_mcbsp_info[mcbsp_id].ick = NULL;
	}
	if (omap2_mcbsp_info[mcbsp_id].ick == NULL) {
		D2(KERN_ERR
		   "OMAP2_McBSP: Unable to get clk for mcbsp%d - %s\n",
		   mcbsp_id, omap2_mcbsp_ick[mcbsp_id]);
		goto _ret_err;
	};

	/*
	 * If McBSP fclk source is selected to be PRCM, then enable the same
	 */
	if (fclk_source == OMAP2_MCBSP_FCLKSRC_PRCM) {
		/* Configure the fclk module src for McBSP interface */
		omap2_mcbsp_info[mcbsp_id].fck =
#ifdef CONFIG_TRACK_RESOURCES
		    clk_get(&mcbsp_device, omap2_mcbsp_fck[mcbsp_id]);
#else
		    clk_get(NULL, omap2_mcbsp_fck[mcbsp_id]);
#endif
			if (IS_ERR(omap2_mcbsp_info[mcbsp_id].ick)) {
                        omap2_mcbsp_info[mcbsp_id].ick =NULL;
                }
	if (omap2_mcbsp_info[mcbsp_id].fck == NULL) {
			D2(KERN_ERR
			   "OMAP2_McBSP: Unable to get clk for mcbsp%d - %s\n",
			   mcbsp_id, omap2_mcbsp_fck[mcbsp_id]);
			goto _ret_err;
		};
		ret_err=clk_enable(omap2_mcbsp_info[mcbsp_id].fck);
		if(ret_err){
			clk_put(omap2_mcbsp_info[mcbsp_id].fck);
			clk_put(omap2_mcbsp_info[mcbsp_id].ick);
			goto _ret_err;
		}
	}

	/*
	 * The else condition can be updated for a specific board,
	 * where external clock is choosed as FCLK source, for now we will be returing
	 * invalid parameter
	 */
	else {
		D2(KERN_ERR "OMAP2_McBSP: Invalid FCLK source");
		goto _ret_err;
	}

	/* Turn ON the iclk */
	ret_err=clk_enable(omap2_mcbsp_info[mcbsp_id].ick);
	if(ret_err){
		clk_disable(omap2_mcbsp_info[mcbsp_id].fck);
		clk_put(omap2_mcbsp_info[mcbsp_id].fck);
		clk_put(omap2_mcbsp_info[mcbsp_id].ick);
		goto _ret_err;
	}

	/* store the data mode and interface mode for future use */
	omap2_mcbsp_info[mcbsp_id].interface_mode = interface_mode;

	/* Store the fclksrc for future references */
	omap2_mcbsp_info[mcbsp_id].fclksrc = fclk_source;

	omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx = -1;
	omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx = -1;
	omap2_mcbsp_info[mcbsp_id].suspended = 0;
	omap2_mcbsp_info[mcbsp_id].isr = NULL;

	/* reserve the interface */
	omap2_mcbsp_info[mcbsp_id].free = 0;

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	/*
	 * Reset the interface
	 * Why? when ever reboot (soft) is done there won't be any system reset,
	 * there may be some default values that can cause some issues
	 */
	omap2_mcbsp_interface_reset(mcbsp_id);
	FN_OUT(0);
	return (0);

_ret_err:
	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	FN_OUT(-EINVAL);
	return -EINVAL;
}

/*
 * Release a reserved McBSP interface
 * mcbsp_id	: McBSP interface Number
 */
int omap2_mcbsp_release_interface(u32 mcbsp_id)
{
	int ret;
	FN_IN;
	/* Check if mcbsp interface is valid */
	if ((mcbsp_id > (OMAP2_MAX_MCBSP_COUNT - 1))
	    || (mcbsp_id < OMAP2_MCBSP_INTERFACE1)) {
		D2(KERN_ERR "OMAP2_McBSP: Invalid McBSP interface\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Check if mcbsp interface is suspended */
	if (OMAP2_McBSP_SUSPENDED(mcbsp_id)) {
		D2(KERN_ERR "OMAP2_McBSP: Interface suspended\n");
		FN_OUT(-EPERM);
		return -EPERM;
	}

	ret = mcbsp_lock(&omap2_mcbsp_info[mcbsp_id].semlock);
	if (ret < 0) {
		D2(KERN_ERR "OMAP2_McBSP: McBSP Lock failed %d\n", ret);
		FN_OUT(ret);
		return (ret);
	}

	if (omap2_mcbsp_info[mcbsp_id].free == 1) {
		D2(KERN_ERR
		   "OMAP2_McBSP: McBSP%d was not reserved\n", mcbsp_id + 1);
		up(&omap2_mcbsp_info[mcbsp_id].semlock);
		FN_OUT(-EINVAL);
		return -EINVAL;
	}

	/* Stop and Free the rx dma chain */
	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx != -1) {
		if (omap_stop_dma_chain_transfers
		    (omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx) != 0) {
			D2(KERN_ERR
			   "OMAP_McBSP internal error unable to stop tx dma chain \n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EPERM);
			return -EPERM;
		}
		omap_free_dma_chain(omap2_mcbsp_info[mcbsp_id].dma_chain_id_rx);
		omap_get_mcbspid[omap2_mcbsp_info[mcbsp_id].
				 dma_chain_id_rx] = -1;
	}

	/* Stop and Free the tx dma chain */
	if (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx != -1) {
		if (omap_stop_dma_chain_transfers
		    (omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx) != 0) {
			D2(KERN_ERR
			   "OMAP2_McBSP internal error unable to stop tx dma chain \n");
			up(&omap2_mcbsp_info[mcbsp_id].semlock);
			FN_OUT(-EPERM);
			return -EPERM;
		}
		omap_free_dma_chain(omap2_mcbsp_info[mcbsp_id].dma_chain_id_tx);
		omap_get_mcbspid[omap2_mcbsp_info[mcbsp_id].
				 dma_chain_id_tx] = -1;
	}
#ifdef MCBSP_POWER
	mcbsp_power_settings(mcbsp_id,MCBSP2_SYSCONFIG_LVL2);
#endif


	/*
	 * If the McBSP FCLK source is PRCM internal module, then free the same
	 */

	if (omap2_mcbsp_info[mcbsp_id].fclksrc == OMAP2_MCBSP_FCLKSRC_PRCM) {
		if (omap2_mcbsp_info[mcbsp_id].fck) {
			clk_disable(omap2_mcbsp_info[mcbsp_id].fck);
			omap2_mcbsp_info[mcbsp_id].fck = NULL;
		}
	}

	/*
	 * An else condition can be added for a external FCLK source
	 * for a specific board
	 */

	if (omap2_mcbsp_info[mcbsp_id].ick) {
		clk_disable(omap2_mcbsp_info[mcbsp_id].ick);
		omap2_mcbsp_info[mcbsp_id].ick = NULL;
	}

	if (omap2_mcbsp_info[mcbsp_id].isr != NULL) {
		free_irq(omap2_mcbsp_irq[mcbsp_id], (void *)mcbsp_id);
	}

	up(&omap2_mcbsp_info[mcbsp_id].semlock);
	/* un reserve the interface */
	omap2_mcbsp_info[mcbsp_id].free = 1;
	FN_OUT(0);
	return 0;
}

/*
 * McBSP driver initialization module
 */
static int __init omap2_mcbsp_init(void)
{
	int interface;

	/* Initialize mcbsp global structure parameters */
	for (interface = 0; interface < OMAP2_MAX_MCBSP_COUNT; interface++) {
		omap2_mcbsp_info[interface].suspended = 0;

		/* Un-reserve the interfaces */
		omap2_mcbsp_info[interface].free = 1;

		/* Initialize a spin lock for each interface */
		spin_lock_init(&omap2_mcbsp_info[interface].lock);

		/* Initialize a sem lock */
		sema_init(&omap2_mcbsp_info[interface].semlock, 1);

		/*
		 * semlock alone will not suffice in full duplex
		 * mode,so initialize another semaphore for tx
		 */
		sema_init(&omap2_mcbsp_info[interface].tx_semlock, 1);

		/* By default set the interface in slave Mode */
		omap2_mcbsp_info[interface].mode = OMAP2_MCBSP_SLAVE;

	}

#ifdef CONFIG_ARCH_OMAP2420
	omap_mcbsp_ext_config();
#endif
	return (0);
}

/*
 * McBSP driver exit module
 */
static void __exit omap2_mcbsp_cleanup(void)
{
	/* Nothing to be done for now */
	/* Reserved for future enhancements if any */

}

module_init(omap2_mcbsp_init);
module_exit(omap2_mcbsp_cleanup);

EXPORT_SYMBOL(omap2_mcbsp_release_interface);
EXPORT_SYMBOL(omap2_mcbsp_request_interface);
EXPORT_SYMBOL(omap2_mcbsp_unregister_isr);
EXPORT_SYMBOL(omap2_mcbsp_register_isr);
EXPORT_SYMBOL(omap2_mcbsp_stop_datarx);
EXPORT_SYMBOL(omap2_mcbsp_stop_datatx);
EXPORT_SYMBOL(omap2_mcbsp_send_data);
EXPORT_SYMBOL(omap2_mcbsp_receive_data);
EXPORT_SYMBOL(omap2_mcbsp_transmitter_index);
EXPORT_SYMBOL(omap2_mcbsp_transmitter_destpos);
EXPORT_SYMBOL(omap2_mcbsp_transmitter_srcpos);
EXPORT_SYMBOL(omap2_mcbsp_receiver_index);
EXPORT_SYMBOL(omap2_mcbsp_receiver_destpos);
EXPORT_SYMBOL(omap2_mcbsp_receiver_srcpos);
EXPORT_SYMBOL(omap2_mcbsp_set_trans_params);
EXPORT_SYMBOL(omap2_mcbsp_set_recv_params);
EXPORT_SYMBOL(omap2_mcbsp_srg_cfg);
EXPORT_SYMBOL(omap2_mcbsp_txclk_cfg);
EXPORT_SYMBOL(omap2_mcbsp_rxclk_cfg);
EXPORT_SYMBOL(omap2_mcbsp_fsync_cfg);
EXPORT_SYMBOL(omap2_mcbsp_rxmultich_cfg);
EXPORT_SYMBOL(omap2_mcbsp_txmultich_cfg);
EXPORT_SYMBOL(omap2_mcbsp_txmultich_enable);
EXPORT_SYMBOL(omap2_mcbsp_rxmultich_enable);
EXPORT_SYMBOL(omap2_mcbsp_datatxrx_mode);
EXPORT_SYMBOL(omap2_mcbsp_resume);
EXPORT_SYMBOL(omap2_mcbsp_suspend);
EXPORT_SYMBOL(omap2_mcbsp_autoidle_disable);
EXPORT_SYMBOL(omap2_mcbsp_autoidle_enable);
EXPORT_SYMBOL(omap2_mcbsp_interface_reset);
EXPORT_SYMBOL(omap2_mcbsp_set_rrst);
EXPORT_SYMBOL(omap2_mcbsp_set_xrst);
EXPORT_SYMBOL(omap2_mcbsp_set_xen);
EXPORT_SYMBOL(omap2_mcbsp_set_ren);
EXPORT_SYMBOL(omap2_mcbsp_set_fsg);
EXPORT_SYMBOL(omap2_mcbsp_set_srg);
#ifdef	OMAP2_MCBSP_POLL_READ_WRITE
EXPORT_SYMBOL(omap_mcbsp_pollread);
EXPORT_SYMBOL(omap_mcbsp_pollwrite);
#endif
#ifdef	OMAP2_MCBSP_DEBUG_REGS
EXPORT_SYMBOL(omap_mcbsp_dump_reg);
#endif

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("McBSP driver Library");
MODULE_LICENSE("GPL");
