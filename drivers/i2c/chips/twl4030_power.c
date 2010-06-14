/*
 * arch/arm/plat-omap/power_companion.c
 *
 * This file contains code to control TWL4030 power compaion chip operation.
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/arch/power_companion.h>
#include <asm/arch/clock.h>
#include <asm/arch/mux.h>
#include <asm/arch/prcm.h>
#include <asm/semaphore.h>
#include <asm/arch/twl4030.h>

struct twl4030_power_registers {
	u8 dev_grp_address;
	u8 vsel_address;
};

struct twl4030_power_registers twl4030_ldo_info[] = {
 {TWL4030_VAUX1_DEV_GRP, TWL4030_VAUX1_DEDICATED},	/* VAUX1_ID*/
 {TWL4030_VAUX2_DEV_GRP, TWL4030_VAUX2_DEDICATED},	/* VAUX2_ID*/
 {TWL4030_VAUX3_DEV_GRP, TWL4030_VAUX3_DEDICATED},	/* VAUX3_ID*/
 {TWL4030_VAUX4_DEV_GRP, TWL4030_VAUX4_DEDICATED},	/* VAUX4_ID*/
 {TWL4030_VMMC1_DEV_GRP, TWL4030_VMMC1_DEDICATED},	/* VMMC1_ID*/
 {TWL4030_VMMC2_DEV_GRP, TWL4030_VMMC2_DEDICATED},	/* VMMC2_ID*/
 {TWL4030_VPLL1_DEV_GRP, TWL4030_VPLL1_DEDICATED},	/* VPLL1_ID*/
 {TWL4030_VPLL2_DEV_GRP, TWL4030_VPLL2_DEDICATED},	/* VPLL2_ID*/
 {TWL4030_VSIM_DEV_GRP, TWL4030_VSIM_DEDICATED},	/* VSIM_ID*/
 {TWL4030_VDAC_DEV_GRP, TWL4030_VDAC_DEDICATED},	/* VDAC_ID*/
 {TWL4030_VINTANA1_DEV_GRP, TWL4030_VINTANA1_DEDICATED}, /* VINTANA1_ID*/
 {TWL4030_VINTANA2_DEV_GRP, TWL4030_VINTANA2_DEDICATED}, /* VINTANA2_ID*/
 {TWL4030_VINTDIG_DEV_GRP, TWL4030_VINTDIG_DEDICATED},	/* VINTDIG_ID*/
 {TWL4030_VIO_DEV_GRP, TWL4030_VIO_VSEL},		/* VIO_ID*/
 {TWL4030_VUSB1V5_DEV_GRP, 0},				/* VUSB1V5_ID*/
 {TWL4030_VUSB1V8_DEV_GRP, 0},				/* VUSB1V8_ID*/
 {TWL4030_VUSB3V1_DEV_GRP, 0},				/* VUSB3V1_ID*/
};

/* Note: H4 uses the Menelaus companion chip.  Currently it is assumed u-boot
 * has set up the chip.  For Triton2, some part of this is done by the loader.
 * However, it turns out that a polled mode u-boot I2C setup can take a long
 * time.  So major init sequences are better done here.  T2 needs to setup
 * sequences before it can effectively shut down its HF clock and handle more
 * complex power needs.
 */

int power_companion_initialized = 0;

struct semaphore pwr_reg_lock;

static int vaux1_ldo_use_count;

int twl4030_vaux1_ldo_use(void)
{
	vaux1_ldo_use_count++;

	/* If first module to use the VAUX2 voltage, enable the voltage */
	if (vaux1_ldo_use_count == 1) {
		if (0 != t2_out(PM_RECEIVER, ENABLE_VAUX1_DEDICATED,
					  TWL4030_VAUX1_DEDICATED)) {
			vaux1_ldo_use_count--;
			return -EIO;
		}
		if (0 != t2_out(PM_RECEIVER, ENABLE_VAUX1_DEV_GRP,
					  TWL4030_VAUX1_DEV_GRP)) {
			vaux1_ldo_use_count--;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux1_ldo_use);

int twl4030_vaux1_ldo_unuse(void)
{
	if (vaux1_ldo_use_count == 0)
		return 0;

	/* If VAUX2 voltage is not used by any modules, disable the voltage */
	vaux1_ldo_use_count--;
	if (vaux1_ldo_use_count == 0) {
		/*
		 * The VAUX2 voltage is not used by any module
		 * Disable the voltage
		 */
		if (0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX1_DEDICATED)) {
			vaux1_ldo_use_count++;
			return -EIO;
		}
		if (0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX1_DEV_GRP)) {
			vaux1_ldo_use_count++;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux1_ldo_unuse);

static int vaux2_ldo_use_count = 0;

int twl4030_vaux2_ldo_use(void)
{
	vaux2_ldo_use_count++;

	/* If first module to use the VAUX2 voltage, enable the voltage */
	if (vaux2_ldo_use_count == 1) {
		if (0 != t2_out(PM_RECEIVER, ENABLE_VAUX2_DEDICATED,
					  TWL4030_VAUX2_DEDICATED)) {
			vaux2_ldo_use_count--;
			return -EIO;
		}
		if (0 != t2_out(PM_RECEIVER, ENABLE_VAUX2_DEV_GRP,
					  TWL4030_VAUX2_DEV_GRP)) {
			vaux2_ldo_use_count--;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux2_ldo_use);

int twl4030_vaux2_ldo_unuse(void)
{
	if (vaux2_ldo_use_count == 0)
		return 0;

	/* If VAUX2 voltage is not used by any modules, disable the voltage */
	vaux2_ldo_use_count--;
	if (vaux2_ldo_use_count == 0) {
		/*
		 * The VAUX2 voltage is not used by any module
		 * Disable the voltage
		 */
		if (0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX2_DEDICATED)) {
			vaux2_ldo_use_count++;
			return -EIO;
		}
		if (0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX2_DEV_GRP)) {
			vaux2_ldo_use_count++;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux2_ldo_unuse);

static int vaux3_ldo_use_count = 0;
int twl4030_vaux3_ldo_use(void)
{
	vaux3_ldo_use_count++;

	/* If first module to use the VAUX2 voltage, enable the voltage */
	if (vaux3_ldo_use_count == 1) {
		if (0 != t2_out(PM_RECEIVER, ENABLE_VAUX3_DEDICATED,
					  TWL4030_VAUX3_DEDICATED)) {
			vaux3_ldo_use_count--;
			return -EIO;
		}
		if (0 != t2_out(PM_RECEIVER, ENABLE_VAUX3_DEV_GRP,
					  TWL4030_VAUX3_DEV_GRP)) {
			vaux3_ldo_use_count--;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux3_ldo_use);

int twl4030_vaux3_ldo_unuse(void)
{
	if (vaux3_ldo_use_count == 0)
		return 0;

	/* If VAUX3 voltage is not used by any modules, disable the voltage */
	vaux3_ldo_use_count--;
	if (vaux3_ldo_use_count == 0) {
		/*
		 * The VAUX3 voltage is not used by any module
		 * Disable the voltage
		 */
		if (0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX3_DEDICATED)) {
			vaux3_ldo_use_count++;
			return -EIO;
		}

		if (0 != t2_out(PM_RECEIVER, 0x00, TWL4030_VAUX3_DEV_GRP)) {
			vaux3_ldo_use_count++;
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(twl4030_vaux3_ldo_unuse);

int enable_vmode_companion_voltage_scaling(int vdd, int floor, int roof,
								int mode)
{
	int e = 0;

	/* set floor, roof voltage, enable jump mode, enable scaling */
	if (vdd == PRCM_VDD1) {
		e |= t2_out(PM_RECEIVER, floor, R_VDD1_VFLOOR);
		e |= t2_out(PM_RECEIVER, roof, R_VDD1_VROOF);
		e |= t2_out(PM_RECEIVER, mode, R_VDD1_STEP);
		e |= t2_out(PM_RECEIVER, EN_SCALE, R_VDD1_VMODE_CFG);
	} else if (vdd == PRCM_VDD2) {
		e |= t2_out(PM_RECEIVER, floor, R_VDD2_VFLOOR);
		e |= t2_out(PM_RECEIVER, roof, R_VDD2_VROOF);
		e |= t2_out(PM_RECEIVER, mode, R_VDD2_STEP);
		e |= t2_out(PM_RECEIVER, EN_SCALE, R_VDD2_VMODE_CFG);
	}
	if (e) {
		printk(KERN_ERR "Error enabling voltage scaling\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(enable_vmode_companion_voltage_scaling);

int disable_vmode_companion_voltage_scaling(void)
{
	int e = 0;
	e = t2_out(PM_RECEIVER, DIS_SCALE, R_VDD1_VMODE_CFG);
	if (e)
		e = -EIO;
	return e;
}
EXPORT_SYMBOL(disable_vmode_companion_voltage_scaling);

int write_val_companion_reg(u8 mod_no, u8 value, u8 reg)
{
	int e = 0;

	down(&pwr_reg_lock);
	e |= t2_out(mod_no, value, reg);
	up(&pwr_reg_lock);
	if (e) {
		printk(KERN_ERR "TWL4030 Power Companion Register\
							Access Error\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(write_val_companion_reg);

int set_bits_companion_reg(u8 mod_no, u8 value, u8 reg)
{
	int e = 0;
	u8 RdReg;

	down(&pwr_reg_lock);
	e |= t2_in(mod_no, &RdReg, reg);
	RdReg |= value;
	e |= t2_out(mod_no, RdReg, reg);
	up(&pwr_reg_lock);
	if (e) {
		printk(KERN_ERR "TWL4030 Power Companion Register\
							Access Error\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(set_bits_companion_reg);

int clear_bits_companion_reg(u8 mod_no, u8 value, u8 reg)
{
	int e = 0;
	u8 RdReg;

	down(&pwr_reg_lock);
	e |= t2_in(mod_no, &RdReg, reg);
	RdReg &= ~value;
	e |= t2_out(mod_no, RdReg, reg);
	up(&pwr_reg_lock);
	if (e) {
		printk(KERN_ERR "TWL4030 Power Companion Register\
							Access Error\n");
		e = -EIO;
	}
	return e;
}
EXPORT_SYMBOL(clear_bits_companion_reg);

static int protect_pm_master(void)
{
	int e = 0;
	e = t2_out(PM_MASTER, KEY_LOCK, R_PROTECT_KEY);
	return e;
}

static int unprotect_pm_master(void)
{
	int e = 0;
	e |= t2_out(PM_MASTER, KEY_UNLOCK1, R_PROTECT_KEY);
	e |= t2_out(PM_MASTER, KEY_UNLOCK2, R_PROTECT_KEY);
	return e;
}

static int init_pwr_intc(void)
{
	int e;

	/* Set interrupt to clear when we write it. Enable interrupt
	 * exclusively between INT1 and INT2
	 */
	e = t2_out(PM_INT, SIH_EXCLEN, R_PWR_SIH_CTRL);

	/* mask all PWR INT1/2 interrupts */
	e |= t2_out(PM_INT, 0xFF, R_PWR_IMR1);
	e |= t2_out(PM_INT, 0xFF, R_PWR_IMR2);

	/* disable falling and rising edge of all PWR sources */
	e |= t2_out(PM_INT, 0x00, R_PWR_EDR1);
	e |= t2_out(PM_INT, 0x00, R_PWR_EDR2);

	/* clear all PWR INT1/2 interrupts */
	e |= t2_out(PM_INT, 0xFF, R_PWR_ISR1);
	e |= t2_out(PM_INT, 0xFF, R_PWR_ISR2);

	return e;
}


#ifdef CONFIG_OMAP_VOLT_VSEL
/*
 * This function sets the VSEL values in the Triton Power IC. This is done in
 * the software configurable mode.
 */
int set_voltage_level(u8 vdd, u8 vsel)
{
	int e = 0;

	/* Disabling the SmartReflex */
	e = clear_bits_companion_reg(PM_RECEIVER,
					DCDC_GLOBAL_CFG_ENABLE_SRFLX,
					R_DCDC_GLOBAL_CFG);
	if (e) {
		printk(KERN_INFO "Unable to disable SR\n");
		return e;
	}

	if (vdd == PRCM_VDD1) {
		/* Enable in software configuration mode, by disabling the
		 * hardware mode.
		 */
		e = clear_bits_companion_reg(PM_RECEIVER,
						 VDD1_VMODE_CFG_ENABLE_VMODE,
						 R_VDD1_VMODE_CFG);
		if (e) {
			printk(KERN_INFO "Unable to set Power IC in software \
				controlled mode\n");
			return e;
		}

		/* Set the step mode to single step */
		e = set_bits_companion_reg(PM_RECEIVER,
						VDD1_STEP_STEP_REG_0,
						R_VDD1_STEP);
		if (e) {
			printk(KERN_INFO "Unable to set the Voltage Step to \
				zero\n");
			return e;
		}

		/* Set the VSEL value */
		e = write_val_companion_reg(PM_RECEIVER,
						 vsel,
						 R_VDD1_VSEL);
		if (e) {
			printk(KERN_INFO "Unable to set the Voltage level to \
				vsel\n");
			return e;
		}
	} else { /* VDD2 */
		/* Presently only 2 OPP levels (i.e. OPP1 and OPP2)
		 * are supported.
		 * Enable in software configuration mode, by disabling the
		 * hardware mode.
		 */
		e = clear_bits_companion_reg(PM_RECEIVER,
						 VDD2_VMODE_CFG_ENABLE_VMODE,
						 R_VDD2_VMODE_CFG);
		if (e) {
			printk(KERN_INFO "Unable to set Power IC in software \
				controlled mode\n");
			return e;
		}

		/* Set the step mode to single step */
		e = set_bits_companion_reg(PM_RECEIVER,
						VDD2_STEP_STEP_REG_0,
						R_VDD2_STEP);
		if (e) {
			printk(KERN_INFO "Unable to set the Voltage Step to \
				zero\n");
			return e;
		}

		/* Set the VSEL value */
		e = write_val_companion_reg(PM_RECEIVER,
						 vsel,
						 R_VDD2_VSEL);
		if (e) {
			printk(KERN_INFO "Unable to set the Voltage level to \
				vsel\n");
			return e;
		}
	}
	/* 10 microsecond delay for the single step voltage to stabilize */
	udelay(10);

	return e;
}
EXPORT_SYMBOL(set_voltage_level);
#endif /* CONFIG_OMAP_VOLT_VSEL */

/* Set up an action to shut down T2's clocks and wake them up
 * following P1.  Depending on usage this sequence might be
 * reduced to just a unicast to the HFCLKOUT resource.
 */

union triton_pmb_message{
	int msg_word;
	char msg_byte[4];
};

static int config_sleep_wake_sequence(void)
{
#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
	int i;
	int e;
	u8  adr;

	/* Base address for script in Triton memory. */
	#define SLEEP_ON_BASE	0x2B
	u8 sleep_on_seq[] = {
		0x14, 0x78, 0x04, SLEEP_ON_BASE + 1, /* SLEEP brcst RC */
		0x1E, 0x78, 0x02, 0x3F,              /* SLEEP brcst PP/RC */
	};

	#define SLEEP_OFF_BASE	(SLEEP_ON_BASE + (ARRAY_SIZE(sleep_on_seq) / 4))
	u8 sleep_off_seq[] = {
		0x01, 0x7E, 0x30, SLEEP_OFF_BASE + 1, /* Enable RC HFCLK_EN */
		0x1A, 0x7E, 0x37, SLEEP_OFF_BASE + 2, /* ACTIVE brcst PP/PR */
		0x1E, 0x7E, 0x02, 0x3F,               /* ACTIVE brcst RC */
	};

	printk(KERN_INFO "OMAP: Programming TWL sleep/wake scripts...\n");

	/* Program SLEEP ON sequence. */
	e  = t2_out(PM_MASTER, SLEEP_ON_BASE, R_SEQ_ADD_A2S);

	/* R_SEQ_ADD_xxx addresses are based on 32 bit wide memory, however
	 * R_MEMORY_ADDRESS is based on 8 bit wide memory.
	 *  => R_MEMORY_ADDRESS = R_SEQ_ADD_xxx << 2
	 */
	adr = SLEEP_ON_BASE << 2;
	for (i = 0; i < ARRAY_SIZE(sleep_on_seq); i++) {
		e |= t2_out(PM_MASTER, adr, R_MEMORY_ADDRESS);
		e |= t2_out(PM_MASTER, sleep_on_seq[i], R_MEMORY_DATA);
		adr++;
	}

	/* Program SLEEP OFF sequence. */
	e |= t2_out(PM_MASTER, SLEEP_OFF_BASE, R_SEQ_ADD_S2A12);
	e |= t2_out(PM_MASTER, SLEEP_OFF_BASE, R_SEQ_ADD_S2A3);

	adr = SLEEP_OFF_BASE << 2;
	for (i = 0; i < ARRAY_SIZE(sleep_off_seq); i++) {
		e |= t2_out(PM_MASTER, adr, R_MEMORY_ADDRESS);
		e |= t2_out(PM_MASTER, sleep_off_seq[i], R_MEMORY_DATA);
		adr++;
	}

	/* On some joplinemu2 boards setting the registers below causes the
	 * board to hang at boot. Needs more investigation. For now, do not set
	 * for joplinemu2 boards (aka MACH_FLANK).
	 */
#ifndef CONFIG_MACH_FLANK
	/* P1/P2/P3 LVL_WAKEUP should be on LEVEL */
	e |= t2_out(PM_MASTER, 0x00 , R_CFG_P1_TRANSITION);

	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P1_SW_EVENTS);
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P2_SW_EVENTS);
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P3_SW_EVENTS);
#endif

	if (e)
		printk(KERN_ERR
			"OMAP: ERROR: Programming TWL sleep/wake scripts failed.\n");
	return e;

#else
#ifndef CONFIG_DISABLE_HFCLK
	#define A2S_I 2
#else
	#define A2S_I 4
#endif /* #ifndef CONFIG_DISABLE_HFCLK */

	#define AS2_J 4
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP_LDP)
#ifndef CONFIG_DISABLE_HFCLK
	#define S2A_I 2
#else
	#define S2A_I 4
#endif /* #ifndef CONFIG_DISABLE_HFCLK */

#else
	#define S2A_I 3
#endif
	#define S2A_J 4
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP_LDP)
/*
*	Power Bus Message Format
*	Broadcast Message (16 Bits)
*	DEV_GRP[31:29] MT[28]  RES_GRP[27:25]  RES_TYPE2[24:23] RES_TYPE[22:20]
*	RES_STATE[19:16]
*
*	Singular Message (16 Bits)
*	DEV_GRP[31:29] MT[28]  RES_ID[27:20]  RES_STATE[19:16]
*	DELAY[15:8]  NEXT_ADDR[7:0]
*
*/

	union triton_pmb_message sleep_on_seq[A2S_I] = {
#ifndef CONFIG_DISABLE_HFCLK
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x0F << RES_ID_SHIFT |
		RES_STATE_OFF | 0x04 << DELAY_SHIFT | 0x2C},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x10 << RES_ID_SHIFT |
		RES_STATE_OFF | 0x02 << DELAY_SHIFT | 0x3F}
	};
#else
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x0F << RES_ID_SHIFT |
		RES_STATE_OFF | 0x04 << DELAY_SHIFT | 0x2C},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x10 << RES_ID_SHIFT |
		RES_STATE_OFF | 0x02 << DELAY_SHIFT | 0x2D},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x07 << RES_ID_SHIFT |
		RES_STATE_OFF | 0x03 << DELAY_SHIFT | 0x2E},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x19 << RES_ID_SHIFT |
		RES_STATE_OFF | 0x03 << DELAY_SHIFT | 0x3F}
	};
#endif /* #ifndef CONFIG_DISABLE_HFCLK */

	union triton_pmb_message sleep_off_seq[S2A_I] = {
#ifndef CONFIG_DISABLE_HFCLK
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x0F << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x04 << DELAY_SHIFT | 0x30},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x10 << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x02 << DELAY_SHIFT | 0x3F}
	};
#else
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x07 << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x30 << DELAY_SHIFT | 0x30},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x0F << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x30 << DELAY_SHIFT | 0x31},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x10 << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x37 << DELAY_SHIFT | 0x32},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x19 << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x3 << DELAY_SHIFT | 0x3F}
	};
#endif /* #ifndef CONFIG_DISABLE_HFCLK */

#else
	union triton_pmb_message sleep_on_seq[A2S_I] = {
		{DEV_GRP_NULL | MSG_TYPE_BROADCAST | RES_GRP_RC | RES_TYPE2_R0 |
		RES_TYPE_R7 | RES_STATE_SLEEP | 0x04 << DELAY_SHIFT | 0x2C},
		{DEV_GRP_NULL | MSG_TYPE_BROADCAST | RES_GRP_ALL | RES_TYPE2_R0 |
		RES_TYPE_R7 | RES_STATE_SLEEP | 0x02 << DELAY_SHIFT | 0x3F}
	};

	union triton_pmb_message sleep_off_seq[S2A_I] = {
		{DEV_GRP_NULL | MSG_TYPE_SINGULAR | 0x17 << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x30 << DELAY_SHIFT | 0x2F},
		{DEV_GRP_NULL | MSG_TYPE_BROADCAST | RES_GRP_PP_PR | RES_TYPE2_R0 |
		RES_TYPE_R7 | RES_STATE_ACTIVE | 0x37 << DELAY_SHIFT | 0x30},
		{DEV_GRP_NULL | MSG_TYPE_BROADCAST | RES_GRP_ALL | RES_TYPE2_R0 |
		RES_TYPE_R7 | RES_STATE_ACTIVE | 0x02 << DELAY_SHIFT | 0x3F}
	};
#endif
	int e = 0, i = 0, j = 0;
	u8 data;
	/* CLKREQ is pulled high on the 2430SDP, therefore, we need to take
	 * it out of the HFCLKOUT DEV_GRP for P1 else HFCLKOUT can't be stopped.
	 */
	e |= t2_out(PM_RECEIVER, 0x20, R_HFCLKOUT_DEV_GRP);

	/* Set ACTIVE to SLEEP SEQ address in T2 memory*/
	e |= t2_out(PM_MASTER, 0x2B, R_SEQ_ADD_A2S);
	/* Set SLEEP to ACTIVE SEQ address for P1 and P2 */
	e |= t2_out(PM_MASTER, 0x2F, R_SEQ_ADD_S2A12);
	/* Set SLEEP to ACTIVE SEQ address for P3 */
	e |= t2_out(PM_MASTER, 0x2F, R_SEQ_ADD_S2A3);
	/* Install Active->Sleep (A2S) sequence */
	while (i < A2S_I) {
		while (j < AS2_J) {
			/*set starting t2-addr*/
			data = (((0x2B + i) << 2) + j);
			/*select t2-addr*/
			e |= t2_out(PM_MASTER, data, R_MEMORY_ADDRESS);
			data = sleep_on_seq[i].msg_byte[3 - j]; /* get data */
			/*data into t2-addr*/
			e |= t2_out(PM_MASTER, data, R_MEMORY_DATA);
			j++;
		}
		j = 0;
		i++;
	}

	i = j = 0;

	/* Install Sleep->Active (S2A) sequence */
	while (i < S2A_I) {
		while (j < S2A_J) {
			/* set starting t2-addr */
			data = (((0x2F + i) << 2) + j);
			/* sel t2-addr */
			e |= t2_out(PM_MASTER, data, R_MEMORY_ADDRESS);
			data = sleep_off_seq[i].msg_byte[3 - j]; /* get data */
			/* put into t2-addr */
			e |= t2_out(PM_MASTER, data, R_MEMORY_DATA);
			j++;
		}
		j = 0;
		i++;
	}
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP_LDP)
	/* Disabling AC charger effect on sleep-active transitions */
	e |= t2_in(PM_MASTER, &data, R_CFG_P1_TRANSITION);
	data &= 0x0;
	e |= t2_out(PM_MASTER, data , R_CFG_P1_TRANSITION);
#endif
	/* P1/P2/P3 LVL_WAKEUP should be on LEVEL */
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P1_SW_EVENTS);
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P2_SW_EVENTS);
	e |= t2_out(PM_MASTER, LVL_WAKEUP, R_P3_SW_EVENTS);

	if (e)
		printk(KERN_ERR "TWL4030 Power Companion seq config error\n");

	return e;
#endif	
}

/* Programming the WARMRESET Sequence on TRITON */
#if defined(CONFIG_MACH_OMAP_3430SDP) || defined(CONFIG_MACH_OMAP_LDP)
static int config_warmreset_sequence(void)
{
	#define WRST_I 6
	#define WRST_J 4


	union triton_pmb_message t2_wrst_seq[WRST_I] = {
		{DEV_GRP_NULL | MSG_TYPE_SINGULAR | 0x1B << RES_ID_SHIFT |
		RES_STATE_OFF | 0x02 << DELAY_SHIFT | 0x39},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x0F << RES_ID_SHIFT |
		RES_STATE_WRST | 0x0E << DELAY_SHIFT | 0x3A},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x10 << RES_ID_SHIFT |
		RES_STATE_WRST | 0x0E << DELAY_SHIFT | 0x3B},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x07 << RES_ID_SHIFT |
		RES_STATE_WRST | 0x60 << DELAY_SHIFT | 0x3C},
		{DEV_GRP_P1 | MSG_TYPE_SINGULAR | 0x19 << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x02 << DELAY_SHIFT | 0x3D},
		{DEV_GRP_NULL | MSG_TYPE_SINGULAR | 0x1B << RES_ID_SHIFT |
		RES_STATE_ACTIVE | 0x02 << DELAY_SHIFT | 0x3F}
	};

	int e = 0, i = 0, j = 0;
	u8 data, rd_data;

	/* Set WARM RESET SEQ address for P1 */
	e |= t2_out(PM_MASTER, 0x38, R_SEQ_ADD_WARM);

	/* Install Warm Reset sequence */
	while (i < WRST_I) {
		while (j < WRST_J) {
			/* set starting t2-addr */
			data = (((0x38 + i) << 2) + j);
			/* sel t2-addr */
			e |= t2_out(PM_MASTER, data, R_MEMORY_ADDRESS);
			/* get data */
			data = t2_wrst_seq[i].msg_byte[3 - j];
			/* put into t2-addr */
			e |= t2_out(PM_MASTER, data, R_MEMORY_DATA);
			j++;
		}
		j = 0;
		i++;
	}

	/* P1/P2/P3 enable WARMRESET */
	e |= t2_in(PM_MASTER, &rd_data, R_P1_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	e |= t2_out(PM_MASTER, rd_data, R_P1_SW_EVENTS);

	e |= t2_in(PM_MASTER, &rd_data, R_P2_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	e |= t2_out(PM_MASTER, rd_data, R_P2_SW_EVENTS);

	e |= t2_in(PM_MASTER, &rd_data, R_P3_SW_EVENTS);
	rd_data |= ENABLE_WARMRESET;
	e |= t2_out(PM_MASTER, rd_data, R_P3_SW_EVENTS);

	if (e)
		printk(KERN_ERR "TWL4030 Power Companion Warmreset\
						 seq config error\n");
	return e;
}
#else
#define config_warmreset_sequence() 0
#endif


int  twl4030_ldo_set_voltage(u8 ldo_index, u8 level)
{
	int result = 0;
	u8 dev_grp_register, dev_grp_value;
	u8 vsel_register, vsel_value;

	if (ldo_index >= TWL4030_LDO_MAX_ID)
		return -1;

	dev_grp_register = twl4030_ldo_info[ldo_index].dev_grp_address;
	vsel_register = twl4030_ldo_info[ldo_index].vsel_address;

	if (level == 0) {
		dev_grp_value = TWL4030_DEV_GRP_NONE;
		vsel_value = TWL4030_DEDICATED_NONE;
	} else {
		dev_grp_value = TWL4030_DEV_GRP_P1;
		vsel_value = level - 1;
	}

	if (dev_grp_register != 0)
		result = twl4030_i2c_write_u8(PM_RECEIVER,
			dev_grp_value, dev_grp_register);

	if (vsel_register != 0)
		result = twl4030_i2c_write_u8(PM_RECEIVER,
			vsel_value, vsel_register);

	return result;
}
EXPORT_SYMBOL(twl4030_ldo_set_voltage);

int power_companion_init(void)
{
	struct clk *osc;
	u32 rate, ctrl = HFCLK_FREQ_26_MHZ;
	int e = 0;

	if (power_companion_initialized != 0)
		return 0;

#if defined(CONFIG_ARCH_OMAP2)
	osc = clk_get(NULL,"osc_ck");
#elif defined(CONFIG_ARCH_OMAP3)
	osc = clk_get(NULL,"osc_sys_ck");
#endif
	rate = clk_get_rate(osc);
	clk_put(osc);

	switch (rate) {
	case 19200000 : ctrl = HFCLK_FREQ_19p2_MHZ; break;
	case 26000000 : ctrl = HFCLK_FREQ_26_MHZ; break;
	case 38400000 : ctrl = HFCLK_FREQ_38p4_MHZ; break;
	}
	ctrl |= HIGH_PERF_SQ;

	/* Semaphore lock to prevent concurrent access */
	init_MUTEX(&pwr_reg_lock);

	e |= unprotect_pm_master();
	e |= t2_out(PM_MASTER, ctrl, R_CFG_BOOT); /* effect->MADC+USB ck en */
	e |= init_pwr_intc();
	e |= config_sleep_wake_sequence(); /* Set action for P1-SLEEP1 */
	e |= config_warmreset_sequence(); /* Set action for WARMRESET */
	e |= protect_pm_master();

#ifdef CONFIG_OMAP_VOLT_VMODE
	if (cpu_is_omap24xx())
		e |= enable_vmode_companion_voltage_scaling(PRCM_VDD1, V_1p05,
				V_1p3, EN_JMP);
	else if (cpu_is_omap3430()) {
		e |= enable_vmode_companion_voltage_scaling(PRCM_VDD1, V_1p05,
				V_1p2, EN_JMP);
		e |= enable_vmode_companion_voltage_scaling(PRCM_VDD2, V_1p15,
				V_1p15, EN_JMP);
	}
#elif defined(CONFIG_OMAP_VOLT_SR) || defined(CONFIG_OMAP_VOLT_SR_BYPASS)
	/* Enabling the SmartReflex */
	e = set_bits_companion_reg(PM_RECEIVER, DCDC_GLOBAL_CFG_ENABLE_SRFLX,
							R_DCDC_GLOBAL_CFG);
	if (e)
		printk(KERN_INFO "Unable to enable SR\n");
#endif

	if (e) {
		printk(KERN_ERR "TWL4030 Power Companion Init Error\n");
		e = -EIO;
	} else {
		power_companion_initialized = 1;
		printk(KERN_INFO "TWL4030 Power Companion Active\n");
	}
	return e;
}
EXPORT_SYMBOL(power_companion_init);
