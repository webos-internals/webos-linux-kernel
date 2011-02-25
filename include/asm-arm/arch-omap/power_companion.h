/*
 * linux/include/asm-arm/arch-omap2/power_companion.h
 *
 * Power Companion Chip defines.
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __ASM_ARM_ARCH_OMAP2_POWER_COMPANION_H
#define __ASM_ARM_ARCH_OMAP2_POWER_COMPANION_H

#include <asm/arch/twl4030.h>

#define PM_MASTER	TWL4030_MODULE_PM_MASTER
#define PM_RECEIVER	TWL4030_MODULE_PM_RECEIVER
#define PM_INT		TWL4030_MODULE_INT

#define t2_out(c, v, r)	twl4030_i2c_write_u8(c, v, r)
#define t2_in(c, v, r)	twl4030_i2c_read_u8(c, v, r)


int power_companion_init(void);
int enable_vmode_companion_voltage_scaling(int vdd, int floor, int roof,
								int mode);
int disable_companion_voltage_scaling(void);
int write_bits_companion_reg(u8 mod_no, u8 value, u8 reg);
int set_bits_companion_reg(u8 mod_no, u8 value, u8 reg);
int clear_bits_companion_reg(u8 mod_no, u8 value, u8 reg);
int set_voltage_level(u8 vdd, u8 vsel);
int twl4030_ldo_set_voltage(u8 ldo_index, u8 level);

#define PHY_TO_OFF_PM_MASTER(p)		(p - 0x36)
#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)
#define PHY_TO_OFF_PM_INT(p)		(p - 0x2e)

/* resource - vaux1 */
#define ENABLE_VAUX1_DEDICATED	0x03
#define ENABLE_VAUX1_DEV_GRP	0x20
int twl4030_vaux1_ldo_use(void);
int twl4030_vaux1_ldo_unuse(void);

/* resource - vaux2 */
#define ENABLE_VAUX2_DEDICATED	0x09
#define ENABLE_VAUX2_DEV_GRP	0x20
int twl4030_vaux2_ldo_use(void);
int twl4030_vaux2_ldo_unuse(void);

/* resource - vaux3 */
#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20
int twl4030_vaux3_ldo_use(void);
int twl4030_vaux3_ldo_unuse(void);

/* resource - hfclk */
#define R_HFCLKOUT_DEV_GRP PHY_TO_OFF_PM_RECIEVER(0xe6)

/* boot config */
#define R_CFG_BOOT		PHY_TO_OFF_PM_MASTER(0x3b)
#define HFCLK_FREQ_19p2_MHZ	(1 << 0)
#define HFCLK_FREQ_26_MHZ	(2 << 0)
#define HFCLK_FREQ_38p4_MHZ	(3 << 0)
#define HIGH_PERF_SQ		(1 << 3)

/* access control */
#define R_PROTECT_KEY		PHY_TO_OFF_PM_MASTER(0x44)
#define KEY_UNLOCK1		0xce
#define KEY_UNLOCK2		0xec
#define KEY_LOCK		0x00

/* Vmode control */
#define R_DCDC_GLOBAL_CFG	PHY_TO_OFF_PM_RECIEVER(0x61)

#define R_VDD1_VSEL		PHY_TO_OFF_PM_RECIEVER(0xb9)
#define R_VDD1_VMODE_CFG	PHY_TO_OFF_PM_RECIEVER(0xba)
#define R_VDD1_VFLOOR		PHY_TO_OFF_PM_RECIEVER(0xbb)
#define R_VDD1_VROOF		PHY_TO_OFF_PM_RECIEVER(0xbc)
#define R_VDD1_STEP		PHY_TO_OFF_PM_RECIEVER(0xbd)

#define R_VDD2_VSEL		PHY_TO_OFF_PM_RECIEVER(0xc7)
#define R_VDD2_VMODE_CFG	PHY_TO_OFF_PM_RECIEVER(0xc8)
#define R_VDD2_VFLOOR		PHY_TO_OFF_PM_RECIEVER(0xc9)
#define R_VDD2_VROOF		PHY_TO_OFF_PM_RECIEVER(0xca)
#define R_VDD2_STEP		PHY_TO_OFF_PM_RECIEVER(0xcb)

/* R_DCDC_GLOBAL_CFG register, SMARTREFLEX_ENABLE valuws */
#define DCDC_GLOBAL_CFG_ENABLE_SRFLX	0x08

/* R_VDD1_VMODE_CFG register, ENABLE_VMODE values */
#define VDD1_VMODE_CFG_ENABLE_VMODE	0x01

/* R_VDD1_STEP register, STEP_REG values */
#define VDD1_STEP_STEP_REG_0	0x00

/* R_VDD2_VMODE_CFG register, ENABLE_VMODE values */
#define VDD2_VMODE_CFG_ENABLE_VMODE	0x01

/* R_VDD2_STEP register, STEP_REG values */
#define VDD2_STEP_STEP_REG_0	0x00

#define EN_JMP			0x0
#define DIS_SCALE		0x0
#define EN_SCALE		0x1

#define V_1p3			0x38
#define V_1p2			0x30
#define V_1p15			0x2C
#define V_1p05			0x24
#define V_1p0			0x20

/* sequence script */
#define R_SEQ_ADD_A2S	PHY_TO_OFF_PM_MASTER(0x55)
#define R_SEQ_ADD_S2A12	PHY_TO_OFF_PM_MASTER(0x56)
#define	R_SEQ_ADD_S2A3	PHY_TO_OFF_PM_MASTER(0x57)
#define	R_SEQ_ADD_WARM	PHY_TO_OFF_PM_MASTER(0x58)
#define R_MEMORY_ADDRESS	PHY_TO_OFF_PM_MASTER(0x59)
#define R_MEMORY_DATA	PHY_TO_OFF_PM_MASTER(0x5a)

#define DEV_GRP_NULL		0x0 << 29
#define DEV_GRP_P1		0x1 << 29
#define DEV_GRP_P2		0x2 << 29
#define DEV_GRP_P3		0x4 << 29

#define MSG_TYPE_SINGULAR	0x0 << 28
#define MSG_TYPE_BROADCAST	0x1 << 28

#define RES_GRP_RES		0x0 << 25
#define RES_GRP_PP		0x1 << 25
#define RES_GRP_RC		0x2 << 25
#define RES_GRP_PP_RC		0x3 << 25
#define RES_GRP_PR		0x4 << 25
#define RES_GRP_PP_PR		0x5 << 25
#define RES_GRP_RC_PR		0x6 << 25
#define RES_GRP_ALL		0x7 << 25

#define RES_TYPE2_R0		0x0 << 23

#define RES_TYPE_R7		0x7 << 20

#define RES_ID_SHIFT		20

#define RES_STATE_WRST		0xF << 16
#define RES_STATE_ACTIVE	0xE << 16
#define RES_STATE_SLEEP		0x8 << 16
#define RES_STATE_OFF		0x0 << 16

#define DELAY_SHIFT		8

/* PM events */
#define R_P1_SW_EVENTS	PHY_TO_OFF_PM_MASTER(0x46)
#define R_P2_SW_EVENTS	PHY_TO_OFF_PM_MASTER(0x47)
#define R_P3_SW_EVENTS	PHY_TO_OFF_PM_MASTER(0x48)
#define R_CFG_P1_TRANSITION	PHY_TO_OFF_PM_MASTER(0x36)
#define R_CFG_P2_TRANSITION	PHY_TO_OFF_PM_MASTER(0x37)
#define R_CFG_P3_TRANSITION	PHY_TO_OFF_PM_MASTER(0x38)
#define LVL_WAKEUP	0x08
#define STARTON_CHG	0x02
#define ENABLE_WARMRESET	0x10

#define R_PWR_ISR1	PHY_TO_OFF_PM_INT(0x2e)
#define R_PWR_IMR1	PHY_TO_OFF_PM_INT(0x2f)
#define R_PWR_ISR2	PHY_TO_OFF_PM_INT(0x30)
#define R_PWR_IMR2	PHY_TO_OFF_PM_INT(0x31)
#define R_PWR_EDR1	PHY_TO_OFF_PM_INT(0x33)
#define R_PWR_EDR2	PHY_TO_OFF_PM_INT(0x34)
#define R_PWR_SIH_CTRL	PHY_TO_OFF_PM_INT(0x35)
#define SIH_COR		0x04
#define SIH_EXCLEN	0x01

/* R_PWR_ISR1,2 R_PWR_IMR1,2 bits */
#define RTC_IT		0x08

/* R_PWR_EDR1 bits */
#define RTC_IT_RISING	0x80

#endif /* POWER_COMPANION */

