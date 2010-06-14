/*
 * twl4030.h - header for TWL4030 PM and audio CODEC device
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef __TWL4030_H_
#define __TWL4030_H_

#define TWL_STATE_DEVOFF	(1<<0)
#define TWL_STATE_DEVSLP	(1<<1)
#define TWL_STATE_DEVACT	(1<<2)

#define twl_write(r,v) \
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,v,r)
#define twl_target_message(dev_grp, res_id, res_state)  \
                           (u16)((dev_grp << 13) | (res_id << 4) | res_state)

/* USB ID */
#define TWL4030_MODULE_USB		0x00
/* AUD ID */
#define TWL4030_MODULE_AUDIO_VOICE	0x01
#define TWL4030_MODULE_GPIO		0x02
#define TWL4030_MODULE_INTBR		0x03
#define TWL4030_MODULE_PIH		0x04
#define TWL4030_MODULE_TEST		0x05
/* AUX ID */
#define TWL4030_MODULE_KEYPAD		0x06
#define TWL4030_MODULE_MADC		0x07
#define TWL4030_MODULE_INTERRUPTS	0x08
#define TWL4030_MODULE_LED		0x09
#define TWL4030_MODULE_MAIN_CHARGE	0x0A
#define TWL4030_MODULE_PRECHARGE	0x0B
#define TWL4030_MODULE_PWM0		0x0C
#define TWL4030_MODULE_PWM1		0x0D
#define TWL4030_MODULE_PWMA		0x0E
#define TWL4030_MODULE_PWMB		0x0F
/* POWER ID */
#define TWL4030_MODULE_BACKUP		0x10
#define TWL4030_MODULE_INT		0x11
#define TWL4030_MODULE_PM_MASTER	0x12
#define TWL4030_MODULE_PM_RECEIVER	0x13
#define TWL4030_MODULE_RTC		0x14
#define TWL4030_MODULE_SECURED_REG	0x15

/* IRQ information-need base */
#include <asm/arch/irqs.h>
/* TWL4030 interrupts */

#define TWL4030_MODIRQ_GPIO		(IH_TWL4030_BASE + 0)
#define TWL4030_MODIRQ_KEYPAD		(IH_TWL4030_BASE + 1)
#define TWL4030_MODIRQ_BCI		(IH_TWL4030_BASE + 2)
#define TWL4030_MODIRQ_MADC		(IH_TWL4030_BASE + 3)
#define TWL4030_MODIRQ_USB		(IH_TWL4030_BASE + 4)
#define TWL4030_MODIRQ_PWR		(IH_TWL4030_BASE + 5)
/* Rest are unsued currently*/

/* PM_MASTER Registers */
#define TWL4030_PROTECT_KEY		0x0E

#define TWL4030_CFG_PWRANA2		0x09

#define TWL4030_P1_SW_EVENTS		0x10
#define TWL4030_P2_SW_EVENTS		0x11
#define TWL4030_P3_SW_EVENTS		0x12

#define TWL4030_PB_CFG			0x14
#define TWL4030_PB_WORD_MSB		0x15
#define TWL4030_PB_WORD_LSB		0x16

/* PM_RECEIVER Registers */
#define TWL4030_DCDC_GLOBAL_CFG		0x06

#define TWL4030_VAUX1_DEV_GRP		0x17
#define TWL4030_VAUX1_REMAP		0x19
#define TWL4030_VAUX1_DEDICATED		0x1A
#define TWL4030_VAUX2_DEV_GRP		0x1B
#define TWL4030_VAUX2_REMAP		0x1D
#define TWL4030_VAUX2_DEDICATED		0x1E
#define TWL4030_VAUX3_DEV_GRP		0x1F
#define TWL4030_VAUX3_REMAP		0x21
#define TWL4030_VAUX3_DEDICATED		0x22
#define TWL4030_VAUX4_DEV_GRP		0x23
#define TWL4030_VAUX4_REMAP		0x25
#define TWL4030_VAUX4_DEDICATED		0x26
#define TWL4030_VMMC1_DEV_GRP		0x27
#define TWL4030_VMMC1_REMAP		0x29
#define TWL4030_VMMC1_DEDICATED		0x2A
#define TWL4030_VMMC2_DEV_GRP		0x2B
#define TWL4030_VMMC2_REMAP		0x2D
#define TWL4030_VMMC2_DEDICATED		0x2E
#define TWL4030_VPLL1_DEV_GRP		0x2F
#define TWL4030_VPLL1_REMAP		0x31
#define TWL4030_VPLL1_DEDICATED		0x32
#define TWL4030_VPLL2_DEV_GRP		0x33
#define TWL4030_VPLL2_REMAP		0x35
#define TWL4030_VPLL2_DEDICATED		0x36
#define TWL4030_VSIM_DEV_GRP		0x37
#define TWL4030_VSIM_REMAP		0x39
#define TWL4030_VSIM_DEDICATED		0x3A
#define TWL4030_VDAC_DEV_GRP		0x3B
#define TWL4030_VDAC_REMAP		0x3D
#define TWL4030_VDAC_DEDICATED		0x3E
#define TWL4030_VINTANA1_DEV_GRP	0x3F
#define TWL4030_VINTANA1_REMAP		0x41
#define TWL4030_VINTANA1_DEDICATED 	0x42
#define TWL4030_VINTANA2_DEV_GRP	0x43
#define TWL4030_VINTANA2_REMAP		0x45
#define TWL4030_VINTANA2_DEDICATED 	0x46
#define TWL4030_VINTDIG_DEV_GRP		0x47
#define TWL4030_VINTDIG_REMAP		0x49
#define TWL4030_VINTDIG_DEDICATED  	0x4A
#define TWL4030_VIO_DEV_GRP		0x4B
#define TWL4030_VIO_VSEL		0x54
#define TWL4030_VIO_REMAP		0x4D
#define TWL4030_VIO_CFG			0x4E
#define TWL4030_VDD1_DEV_GRP		0x55
#define TWL4030_VDD1_REMAP		0x57
#define TWL4030_VDD2_DEV_GRP		0x63
#define TWL4030_VDD2_REMAP		0x65
#define TWL4030_VUSB1V5_DEV_GRP		0x71
#define TWL4030_VUSB1V5_REMAP		0x73
#define TWL4030_VUSB1V8_DEV_GRP		0x74
#define TWL4030_VUSB1V8_REMAP		0x76
#define TWL4030_VUSB3V1_DEV_GRP		0x77
#define TWL4030_VUSB3V1_REMAP		0x79
#define TWL4030_VUSBCP_DEV_GRP		0x7A
#define TWL4030_VUSBCP_REMAP		0x7C
#define TWL4030_VUSB_DEDICATED1		0x7D
#define TWL4030_VUSB_DEDICATED2		0x7E
#define TWL4030_REGEN_DEV_GRP		0x7F
#define TWL4030_REGEN_REMAP		0x81
#define TWL4030_NRESPWRON_DEV_GRP	0x82
#define TWL4030_NRESPWRON_REMAP		0x84
#define TWL4030_CLKEN_DEV_GRP		0x85
#define TWL4030_CLKEN_REMAP		0x87
#define TWL4030_SYSEN_DEV_GRP		0x88
#define TWL4030_SYSEN_REMAP		0x8A
#define TWL4030_HFCLKOUT_DEV_GRP	0x8B
#define TWL4030_HFCLKOUT_REMAP		0x8D
#define TWL4030_32KCLKOUT_DEV_GRP	0x8E
#define TWL4030_32KCLKOUT_REMAP		0x90
#define TWL4030_TRITON_RESET_DEV_GRP	0x91
#define TWL4030_TRITON_RESET_REMAP	0x93
#define TWL4030_MAINREF_DEV_GRP		0x94
#define TWL4030_MAINREF_REMAP		0x96

/* TWL_DEV_GROUP_x */
#define TWL_DEV_GROUP_NONE		(0 << 5)
#define TWL_DEV_GROUP_P1		(1 << 5)
#define TWL_DEV_GROUP_P2		(2 << 5)
#define TWL_DEV_GROUP_P1P2		(3 << 5)
#define TWL_DEV_GROUP_P3		(4 << 5)
#define TWL_DEV_GROUP_P1P3		(5 << 5)
#define TWL_DEV_GROUP_P2P3		(6 << 5)
#define TWL_DEV_GROUP_ALL		(7 << 5)

/* Processor Groups */
#define TWL_PROCESSOR_GRP1		0x01
#define TWL_PROCESSOR_GRP2		0x02
#define TWL_PROCESSOR_GRP3		0x04

/* Power Source IDs */
#define TWL_VAUX1_RES_ID		1
#define TWL_VAUX2_RES_ID		2
#define TWL_VAUX3_RES_ID		3
#define TWL_VAUX4_RES_ID		4
#define TWL_VMMC1_RES_ID		5
#define TWL_VMMC2_RES_ID		6
#define TWL_VPLL1_RES_ID		7
#define TWL_VPLL2_RES_ID		8
#define TWL_VSIM_RES_ID			9
#define TWL_VDAC_RES_ID			10
#define TWL_VIO_RES_ID			14
#define TWL_VDD1_RES_ID			15
#define TWL_VDD2_RES_ID			16
#define TWL_VUSB_1V5_RES_ID		17
#define TWL_VUSB_1V8_RES_ID		18
#define TWL_VUSB_3V1_RES_ID		19
#define TWL_REGEN_RES_ID		21
#define TWL_CLKEN_RES_ID		23
#define TWL_SYSEN_RES_ID		24
#define TWL_HFCLKOUT_RES_ID		25

#define TWL_RES_OFF			0x00	// Resource in OFF
#define TWL_RES_SLEEP			0x08	// Resource in SLEEP
#define TWL_RES_ACTIVE			0x0D	// Resource in ACTIVE

/* TWL4030 LDO IDs */
#define TWL4030_VAUX1_ID		0x0
#define TWL4030_VAUX2_ID		0x1
#define TWL4030_VAUX3_ID		0x2
#define TWL4030_VAUX4_ID		0x3
#define TWL4030_VMMC1_ID		0x4
#define TWL4030_VMMC2_ID		0x5
#define TWL4030_VPLL1_ID		0x6
#define TWL4030_VPLL2_ID		0x7
#define TWL4030_VSIM_ID			0x8
#define TWL4030_VDAC_ID			0x9
#define TWL4030_VINTANA1_ID		0xA
#define TWL4030_VINTANA2_ID		0xB
#define TWL4030_VINTDIG_ID		0xC
#define TWL4030_VIO_ID			0xD
#define TWL4030_VUSB1V5_ID		0xE
#define TWL4030_VUSB1V8_ID		0xF
#define TWL4030_VUSB3V1_ID		0x10
#define TWL4030_LDO_MAX_ID		0x11

/* TWL4030 Power register values */
#define TWL4030_DEV_GRP_P1		0x20
#define TWL4030_DEV_GRP_NONE		0x0
#define TWL4030_DEDICATED_NONE		0x0

/* TWL4030 GPIO interrupt definitions */

#define TWL4030_GPIO_MIN		0
#define TWL4030_GPIO_MAX		18
#define TWL4030_GPIO_MAX_CD		2
#define TWL4030_GPIO_IRQ_NO(n)		(IH_TWL4030_GPIO_BASE+n)
#define TWL4030_GPIO_IS_INPUT		1
#define TWL4030_GPIO_IS_OUTPUT		0
#define TWL4030_GPIO_IS_ENABLE		1
#define TWL4030_GPIO_IS_DISABLE		0
#define TWL4030_GPIO_PULL_UP		0
#define TWL4030_GPIO_PULL_DOWN		1
#define TWL4030_GPIO_PULL_NONE		2
#define TWL4030_GPIO_EDGE_NONE		0
#define TWL4030_GPIO_EDGE_RISING	1
#define TWL4030_GPIO_EDGE_FALLING	2

/* VAUX1 possible voltage values */
#define TWL_VAUX1_1P50			0x00
#define TWL_VAUX1_1P80			0x01
#define TWL_VAUX1_2P50			0x02
#define TWL_VAUX1_2P80			0x03
#define TWL_VAUX1_3P00			0x04

/* VAUX2 possible voltage values */
#define TWL_VAUX2_1P00			0x00
#define TWL_VAUX2_1P20			0x02
#define TWL_VAUX2_1P30			0x03
#define TWL_VAUX2_1P50			0x04
#define TWL_VAUX2_1P80			0x05
#define TWL_VAUX2_1P85			0x06
#define TWL_VAUX2_2P50			0x07
#define TWL_VAUX2_2P60			0x08
#define TWL_VAUX2_2P80			0x09

/* VAUX3 possible voltage values */
#define TWL_VAUX3_1P50			0x00
#define TWL_VAUX3_1P80			0x01
#define TWL_VAUX3_2P50			0x02
#define TWL_VAUX3_2P80			0x03

/* VAUX4 possible voltage values */
#define TWL_VAUX4_0P70			0x00
#define TWL_VAUX4_1P00			0x01
#define TWL_VAUX4_1P20			0x02
#define TWL_VAUX4_1P30			0x03
#define TWL_VAUX4_1P50			0x04
#define TWL_VAUX4_1P80			0x05
#define TWL_VAUX4_1P85			0x06
#define TWL_VAUX4_2P50			0x07
#define TWL_VAUX4_2P60			0x08
#define TWL_VAUX4_2P80			0x09

/* VMMC1 possible voltage values */
#define TWL_VMMC1_1P85			0x00
#define TWL_VMMC1_2P85			0x01
#define TWL_VMMC1_3P00			0x02
#define TWL_VMMC1_3P15			0x03

/* VMMC2 possible voltage values */
#define TWL_VMMC2_1P00			0x00
#define TWL_VMMC2_1P20			0x02
#define TWL_VMMC2_1P30			0x03
#define TWL_VMMC2_1P50			0x04
#define TWL_VMMC2_1P80			0x05
#define TWL_VMMC2_1P85			0x06
#define TWL_VMMC2_2P50			0x07
#define TWL_VMMC2_2P60			0x08
#define TWL_VMMC2_2P80			0x09
#define TWL_VMMC2_2P85			0x0A
#define TWL_VMMC2_3P00			0x0B
#define TWL_VMMC2_3P15			0x0C	// or 0x0D or 0x0E or 0x0F

/* VDAC possible voltage values */
#define TWL_VDAC_1P20			0x00
#define TWL_VDAC_1P30			0x01
#define TWL_VDAC_1P80			0x02

/* VPLL2 possible voltage values */
#define TWL_VPLL2_0P70			0x00
#define TWL_VPLL2_1P00			0x01
#define TWL_VPLL2_1P20			0x02
#define TWL_VPLL2_1P30			0x03
#define TWL_VPLL2_1P50			0x04
#define TWL_VPLL2_1P80			0x05
#define TWL_VPLL2_1P85			0x06
#define TWL_VPLL2_2P50			0x07
#define TWL_VPLL2_2P60			0x08
#define TWL_VPLL2_2P80			0x09
#define TWL_VPLL2_2P85			0x0A
#define TWL_VPLL2_3P00			0x0B
#define TWL_VPLL2_3P15			0x0C

/* Functions to read and write from TWL4030 */

/*
 * IMP NOTE:
 * The base address of the module will be added by the triton driver
 * It is the caller's responsibility to ensure sane values
 */
int twl4030_i2c_write_u8(u8 mod_no, u8 val, u8 reg);
int twl4030_i2c_read_u8(u8 mod_no, u8 *val, u8 reg);

 /*
  * i2c_write: IMPORTANT - Allocate value num_bytes+1 and valid data starts at
  *		Offset 1.
  */
int twl4030_i2c_write(u8 mod_no, u8 *value, u8 reg, u8 num_bytes);
int twl4030_i2c_read(u8 mod_no, u8 *value, u8 reg, u8 num_bytes);

#endif /* End of __TWL4030_H */
