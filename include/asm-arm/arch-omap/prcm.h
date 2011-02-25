/*
 * linux/include/asm-arm/arch-omap/prcm.h
 *
 * Access definitions for use in OMAP34XX clock and power management
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 */

#include <linux/types.h>

#ifndef __ASM_ARM_ARCH_PRCM_H
#define __ASM_ARM_ARCH_PRCM_H

/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

#define CHECK_DOMAIN_IDX

#ifdef CHECK_DOMAIN_IDX
#define FAIL_ON_INVALID_DOMAINID(_id) \
        do { if ((_id) > PRCM_NUM_DOMAINS) return PRCM_FAIL; } while (0)
#else
#define FAIL_ON_INVALID_DOMAINID(_id)
#endif

/*
 * Takes a u32 and returns a string that shows bit representation of value.
 */
#define binstr(val) \
	({ \
		char buf[40]; \
		char *p = buf; \
		u32  n  = val; \
		int  i  = 32; \
		while (i--) { \
			*p++ = n & 0x80000000 ? '1' : '0'; \
			n <<= 1; \
	 		if (i && (0 == (i & 0x7))) *p++ = '.'; \
		} \
	 	*p = '\0'; \
		buf; \
	})

/******************************************************************************/

/* Return Values of PRCM Lib API's */
#define PRCM_PASS	0
#define PRCM_FAIL	1

/* Definitions for enable/ disable */
#define PRCM_ENABLE	1
#define PRCM_DISABLE	0

#define PRCM_TRUE	1
#define PRCM_FALSE	0

/* Check accessibility or dont check accessibility*/
#define PRCM_ACCESS_CHK		1
#define PRCM_NO_ACCESS_CHK	0

/* IDLE METHOD */
#define PRCM_AUTO	1
#define PRCM_FORCE	2
#define PRCM_MANUAL	3
#define PRCM_IDLE_METHOD_STR(m) \
	((m) == PRCM_AUTO   ? "PRCM_AUTO"   : \
	 (m) == PRCM_FORCE  ? "PRCM_FORCE"  : \
	 (m) == PRCM_MANUAL ? "PRCM_MANUAL" : "INVALID")

/* POWER DOMAIN STATES */
#define PRCM_OFF	0x0
#define PRCM_RET	0x1
#define PRCM_INACTIVE	0x2
#define PRCM_ON		0x3

/* CLOCK STATE TRANSITIONS */
#define PRCM_NO_AUTO		0x0
#define PRCM_SWSUP_SLEEP	0x1
#define PRCM_SWSUP_WKUP		0x2
#define PRCM_HWSUP_AUTO		0x3

#define PRCM_FORCE_IDLE		0x0
#define PRCM_NO_IDLE		0x1
#define PRCM_SMART_IDLE		0x2
#define PRCM_SIDLEMODE_DONTCARE	0x3

#define PRCM_FORCE_STANDBY	0x0
#define PRCM_NO_STANDBY		0x1
#define PRCM_SMART_STANDBY	0x2
#define PRCM_MIDLEMODE_DONTCARE	0x3

/* Masks for standby, idle and auto idle modes */
#define PRCM_AUTO_IDLE_MASK	0x1
#define PRCM_IDLE_MASK		0x18
#define PRCM_STANDBY_MASK	0x3000

/* Offsets for standby, idle and auto idle modes */
#define PRCM_AUTO_IDLE_OFF	0x0
#define PRCM_IDLE_OFF		0x3
#define PRCM_STANDBY_OFF	0xC

/* Mask for setting wakeup dependency */
#define PRCM_WKDEP_EN_CORE	0x1
#define PRCM_WKDEP_EN_MPU	0x2
#define PRCM_WKDEP_EN_IVA2	0x4
#define PRCM_WKDEP_EN_WKUP	0x10
#define PRCM_WKDEP_EN_DSS	0x20
#define PRCM_WKDEP_EN_PER 	0x80

/* Mask for setting sleep dependency */
#define PRCM_SLEEPDEP_EN_CORE	0x1
#define PRCM_SLEEPDEP_EN_MPU	0x2
#define PRCM_SLEEPDEP_EN_IVA2	0x4

/* Mask for H/W supervised transitions L3, L4 and D2D CLKS */
#define CLK_D2D_HW_SUP_ENABLE	0x30 
#define CLK_L4_HW_SUP_ENABLE	0xC
#define CLK_L3_HW_SUP_ENABLE	0x3

/* VDDs*/
#define PRCM_VDD1	1
#define PRCM_VDD2	2
#define PRCM_MAX_SYSC_REGS 30


/* OMAP Revisions */
#define AT_3430		1	/*3430 ES 1.0 */
#define AT_3430_ES2	2	/*3430 ES 2.0 */

/* Domains */
#define DOM_IVA2	1
#define DOM_MPU		2
#define DOM_CORE1	3
#define DOM_CORE2	4
#define DOM_SGX		5
#define DOM_WKUP	6
#define DOM_DSS		7
#define DOM_CAM		8
#define DOM_PER		9 
#define DOM_EMU		10
#define DOM_NEON	11
#define DOM_CORE3	12
#define	DOM_USBHOST	13
#define PRCM_NUM_DOMAINS	13

/* DPLL's */
#define DPLL1_MPU	1
#define DPLL2_IVA2	2
#define DPLL3_CORE	3
#define DPLL4_PER	4
#define DPLL5_PER2	5
#define NO_OF_DPLL	5

/* PUT_DPLL_IN_BYPASS PARAMETERS */
#define LOW_POWER_STOP      1
#define LOW_POWER_BYPASS    5
#define FAST_RELOCK_BYPASS  6

/* DPLL dividers */
#define DPLL_M2 		0x0
#define DPLL_M2X2 		0x1
#define DPLL_M3X2		0x2
#define DPLL_M4X2		0x3
#define DPLL_M5X2		0x4
#define DPLL_M6X2		0x5
#define NO_DPLL_DIV		0x6

/* Device Type */
#define INITIATOR	1
#define TARGET		2
#define INIT_TAR	3

/* Clock Type */
#define FCLK	1
#define ICLK	2

/* Initiator masks for all domains */
#define IVA2_IMASK		0x1
#define MPU_IMASK		0x1
#define CORE2_IMASK		0x0
#define CORE1_IMASK		0x15
#define	CORE3_IMASK		0x0
#ifndef CONFIG_ARCH_OMAP3410
#define SGX_IMASK		0x1
#endif
#define	USBHOST_IMASK		0x1
#define WKUP_IMASK		0x0
#define DSS_IMASK		0x1
#define CAM_IMASK		0x1
#define PER_IMASK		0x0
#define NEON_IMASK		0x1

/* Type of device IDs - Note that a particular device ID 
 can be of multiple types*/
#define ID_DEV			0x0	/*Leaf node eg:uart */
#define ID_DPLL_OP		0x1	/*Dpll output */
#define ID_CLK_DIV		0x2	/*Clock has a divider */
#define ID_CLK_SRC		0x4	/*Source of the clock can be selected */
		
#define ID_CLK_PARENT		0xE1	/*Parent of some other clock */
#define ID_OPP			0xE2 	/*OPP*/
#define ID_MPU_DOM_STATE	0xE4	/*Mpu power domain state*/
#define ID_CORE_DOM_STATE	0xE8	/*Core power domain state*/
#define ID_SYSCONF		0xF0	

#define ID_CONSTRAINT_CLK	0xF1	/* a clock rampup constraint */
#define ID_CONSTRAINT_OPP	0xF2	/* OPP constraint */
#define ID_CONSTRAINT_FREQ	0xF3	/* Frequnecy constraint */
#define ID_MEMORY_RES		0xF4	/* Memory resource */
#define ID_LOGIC_RES		0xF5	/* Logic resource */

/* DEVICE ID: bits 0-4 for Device bit position */
#define DEV_BIT_POS_SHIFT	0
#define DEV_BIT_POS_MASK	0x1F
/* DEVICE ID: bits 5-8 for domainid */
#define DOMAIN_ID_SHIFT		5
#define DOMAIN_ID_MASK		0xF
/* DEVICE ID: bits 9-10 for device type */
#define DEV_TYPE_SHIFT		9
#define DEV_TYPE_MASK		0x3
/* DEVICE ID: bits 11-13 for clk src*/
#define CLK_SRC_BIT_POS_SHIFT	11
#define CLK_SRC_BIT_POS_MASK	0x7
/* CLK ID: bits 14-18 for the clk number */
#define CLK_NO_POS		14
#define CLK_NO_MASK		0x1F
/* DPLL ID: bits 19-21  for the DPLL number */
#define DPLL_NO_POS 		19
#define DPLL_NO_MASK		0x7
/* DPLL ID: bits 22-24 for Divider */
#define DPLL_DIV_POS		22
#define DPLL_DIV_MASK		0x7
/* DEVICE ID/DPLL ID/CLOCK ID: bits 25-27 for ID type */
#define ID_TYPE_SHIFT		25
#define ID_TYPE_MASK		0x7
/* DEVICE ID/DPLL ID/CLOCK ID: bits 28-31 for OMAP type */
#define OMAP_TYPE_SHIFT		28
#define OMAP_TYPE_MASK		0xF

/* Other IDs: bits 20-27 for ID type */
/* These IDs have bits 25,26,27 as 1 */
#define OTHER_ID_TYPE_SHIFT		20
#define OTHER_ID_TYPE_MASK		0xFF

/* OPP ID: bits: 0-4 for OPP number */
#define OPP_NO_POS		0
#define OPP_NO_MASK		0x1F
/* OPP ID: bits: 5-6 for VDD */
#define VDD_NO_POS		5
#define VDD_NO_MASK		0x3
/* COMMMON bits in DEVICE ID/DPLL ID/CLOCK ID */
/* Macros */
#define ID_DEV_BIT_POS(X)		((X & DEV_BIT_POS_MASK)<< DEV_BIT_POS_SHIFT)
#define ID_CLK_SRC_BIT_POS(X) 		((X & CLK_SRC_BIT_POS_MASK)<< CLK_SRC_BIT_POS_SHIFT)
#define ID_DOMAIN(X)			((X & DOMAIN_ID_MASK) << DOMAIN_ID_SHIFT)
#define ID_DEV_TYPE(X)			((X & DEV_TYPE_MASK) << DEV_TYPE_SHIFT)
#define ID_DPLL_NO(X)			((X & DPLL_NO_MASK) << DPLL_NO_POS)
#define ID_DPLL_DIV(X)			((X & DPLL_DIV_MASK) << DPLL_DIV_POS)
#define ID_CLK_NO(X)			((X & CLK_NO_MASK) << CLK_NO_POS)
#define ID_TYPE(X)			((X & ID_TYPE_MASK) << ID_TYPE_SHIFT)
#define OTHER_ID_TYPE(X)		((X & OTHER_ID_TYPE_MASK) << OTHER_ID_TYPE_SHIFT)
#define ID_ES(X)			((X & ES_TYPE_MASK) << ES_TYPE_SHIFT)
#define ID_OMAP(X)			((X & OMAP_TYPE_MASK) << OMAP_TYPE_SHIFT)
#define ID_OPP_NO(X)			((X & OPP_NO_MASK) << OPP_NO_POS)
#define ID_VDD(X)			((X & VDD_NO_MASK) << VDD_NO_POS)
#define DEV_BIT_POS(X)			((X >> DEV_BIT_POS_SHIFT) & DEV_BIT_POS_MASK)
#define CLK_SRC_BIT_POS(X)      	((X >> CLK_SRC_BIT_POS_SHIFT) & CLK_SRC_BIT_POS_MASK)
#define DOMAIN_ID(X)			((X >> DOMAIN_ID_SHIFT) & DOMAIN_ID_MASK)
#define DEV_TYPE(X)			((X >> DEV_TYPE_SHIFT) & DEV_TYPE_MASK)
#define OMAP(X)				((X >> OMAP_TYPE_SHIFT) & OMAP_TYPE_MASK)
#define get_id_type(X)			((X >> ID_TYPE_SHIFT) & ID_TYPE_MASK)
#define get_other_id_type(X)		((X >> OTHER_ID_TYPE_SHIFT) & OTHER_ID_TYPE_MASK)
#define get_opp_no(X)			((X >> OPP_NO_POS) & OPP_NO_MASK)
#define get_vdd(X)			((X >> VDD_NO_POS) & VDD_NO_MASK)
#define get_addr(domainId,regId)	(prcm_domain_register[domainId-1].regdef[regId].reg_addr)
#define get_val_bits(domainId,regId)	(prcm_domain_register[domainId-1].regdef[regId].valid_bits)
#define get_addr_pll(dpll,regId)	(dpll_reg[dpll-1].regdef[regId].reg_addr)
#define get_val_bits_pll(dpll,regId)	(dpll_reg[dpll-1].regdef[regId].valid_bits)
#define get_idlest_lock_bit(dpll_id)	((dpll_id==DPLL4_PER)?0x2:0x1)
#define get_dpll_enbitmask(dpll_id)	((dpll_id==DPLL4_PER)?DPLL4_ENBIT_MASK:DPLL_ENBIT_MASK)
/* DEVICE ID's */
/* Devices in Core1 registers */
#define PRCM_SSI	(ID_OMAP(AT_3430) | ID_TYPE( (ID_DEV | ID_CLK_DIV)) | ID_DEV_TYPE(INITIATOR)\
				| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x0) | ID_CLK_NO(0x7))
#define PRCM_SDRC	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x1))
#define PRCM_SDMA	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(INITIATOR)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x2))
#define PRCM_D2D	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(INITIATOR)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x3))
#define PRCM_HSOTG	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(INITIATOR)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x4))

#define PRCM_OMAP_CTRL	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x6))
#define PRCM_MBOXES	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x7))
#define PRCM_FAC	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x8))
#define PRCM_MCBSP1	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
						| ID_DEV_TYPE(TARGET) | ID_DOMAIN(DOM_CORE1) | \
						ID_CLK_SRC_BIT_POS(0x2) | ID_DEV_BIT_POS(0x9))
#define PRCM_MCBSP5	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
						| ID_DEV_TYPE(TARGET) | ID_DOMAIN(DOM_CORE1) | \
						ID_CLK_SRC_BIT_POS(0x4) | ID_DEV_BIT_POS(0xA))
#define PRCM_GPT10	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
						| ID_DEV_TYPE(TARGET) | ID_DOMAIN(DOM_CORE1) | \
						ID_CLK_SRC_BIT_POS(0x6) | ID_DEV_BIT_POS(0xB))
#define PRCM_GPT11	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
						| ID_DEV_TYPE(TARGET) | ID_DOMAIN(DOM_CORE1) | \
						ID_CLK_SRC_BIT_POS(0x7) | ID_DEV_BIT_POS(0xC))
#define PRCM_UART1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0xD))
#define PRCM_UART2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0xE))
#define PRCM_I2C1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0xF))
#define PRCM_I2C2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x10))
#define PRCM_I2C3	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x11))
#define PRCM_MCSPI1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x12))
#define PRCM_MCSPI2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x13))
#define PRCM_MCSPI3	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x14))
#define PRCM_MCSPI4	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x15))
#define PRCM_HDQ	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x16))
#define PRCM_MSPRO	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x17))
#define PRCM_MMC1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x18))
#define PRCM_MMC2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x19))
#define PRCM_DES2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x1A))
#define PRCM_SHA12	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x1B))
#define PRCM_AES2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x1C))
#define PRCM_ICR	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x1D))
#define PRCM_MMC3	(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x1E))
#define PRCM_SMS	(ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(TARGET) | \
						ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x18))
#define PRCM_GPMC	(ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(TARGET) | \
						ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x19))
#define PRCM_MPU_INTC   (ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(TARGET) | \
                                                ID_DOMAIN(DOM_CORE1) | ID_DEV_BIT_POS(0x1A))
#define PRCM_CORE1_CONSTRAINT	(ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_CORE1))
/* Devices in Core2 registers */
#define PRCM_DES1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE2) | ID_DEV_BIT_POS(0x0))
#define PRCM_SHA11	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE2) | ID_DEV_BIT_POS(0x1))
#define PRCM_RNG	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE2) | ID_DEV_BIT_POS(0x2))
#define PRCM_AES1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE2) | ID_DEV_BIT_POS(0x3))
#define PRCM_PKA	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE2) | ID_DEV_BIT_POS(0x4))

/* Devices in Core3 registers */
#define PRCM_CPEFUSE	(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE3) | ID_DEV_BIT_POS(0x0))
#define PRCM_TS		(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE3) | ID_DEV_BIT_POS(0x1))
#define PRCM_USBTLL	(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
						| ID_DOMAIN(DOM_CORE3) | ID_DEV_BIT_POS(0x2))
/* Devices in SGX registers */
#define PRCM_SGX_ICLK	(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_DEV) |\
			ID_DEV_TYPE(INITIATOR) | ID_DOMAIN(DOM_SGX)\
			| ID_DEV_BIT_POS(0x0))
#define PRCM_SGX_FCLK	(ID_OMAP(AT_3430_ES2) |\
			ID_TYPE((ID_DEV | ID_CLK_DIV | ID_CLK_SRC))\
			| ID_DOMAIN(DOM_SGX) | ID_DEV_TYPE(TARGET) |\
			ID_CLK_NO(0x6) | ID_DEV_BIT_POS(0x1))
#define PRCM_3D_CONSTRAINT	(ID_OMAP(AT_3430) |\
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_SGX))

/* Devices in WKUP registers */
#define PRCM_GPT1	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
						| ID_DEV_TYPE(TARGET) | ID_DOMAIN(DOM_WKUP) | \
				ID_CLK_SRC_BIT_POS(0x0) | ID_DEV_BIT_POS(0x0))
#define PRCM_GPT12	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				| ID_DOMAIN(DOM_WKUP) | ID_DEV_BIT_POS(0x1))
#define PRCM_32KSYNC	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				| ID_DOMAIN(DOM_WKUP) | ID_DEV_BIT_POS(0x2))
#define PRCM_GPIO1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				| ID_DOMAIN(DOM_WKUP) | ID_DEV_BIT_POS(0x3))
#define PRCM_WDT1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				| ID_DOMAIN(DOM_WKUP) | ID_DEV_BIT_POS(0x4))
#define PRCM_WDT2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				| ID_DOMAIN(DOM_WKUP) | ID_DEV_BIT_POS(0x5))
#define PRCM_SR1	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				| ID_DOMAIN(DOM_WKUP) | ID_DEV_BIT_POS(0x6))
#define PRCM_SR2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				| ID_DOMAIN(DOM_WKUP) | ID_DEV_BIT_POS(0x7))
#define PRCM_USIM	(ID_OMAP(AT_3430_ES2) | ID_TYPE((ID_DEV | ID_CLK_DIV |\
				ID_CLK_SRC)) | ID_DEV_TYPE(TARGET) |\
				ID_DOMAIN(DOM_WKUP)\
				| ID_DEV_BIT_POS(0x9) | ID_CLK_NO(0xA))

/* Devices in IVA registers */
#define PRCM_IVA2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(INITIATOR)\
					| ID_DOMAIN(DOM_IVA2) | ID_DEV_BIT_POS(0x0))
#define PRCM_IVA2_CONSTRAINT	(ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_IVA2))
/* Devices in DSS registers */
#define PRCM_DSS	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_DPLL_OP)) | ID_DEV_TYPE(INITIATOR)\
				| ID_DOMAIN(DOM_DSS) | ID_DEV_BIT_POS(0x0) | \
				ID_DPLL_DIV(DPLL_M4X2) | ID_DPLL_NO(DPLL4_PER) )
#define PRCM_DSS2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(INITIATOR)\
				| ID_DOMAIN(DOM_DSS) | ID_DEV_BIT_POS(0x1))
#ifndef CONFIG_ARCH_OMAP3410
#define PRCM_TVOUT	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC)) | \
				ID_DEV_TYPE(TARGET) | ID_DOMAIN(DOM_DSS) | ID_DEV_BIT_POS(0x2))
#endif
#define PRCM_DISPC	(ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(INIT_TAR) \
				| ID_DOMAIN(DOM_DSS) | ID_DEV_BIT_POS(0x1))
#define PRCM_RFBI	(ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(INITIATOR) \
				| ID_DOMAIN(DOM_DSS) | ID_DEV_BIT_POS(0x2))

#define PRCM_DSS_CONSTRAINT	(ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_DSS))

/* Devices in CAM Domain */
#define PRCM_CAM	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_DPLL_OP)) | ID_DEV_TYPE(INITIATOR)\
			 	| ID_DOMAIN(DOM_CAM) | ID_DEV_BIT_POS(0x0) |\
				 ID_DPLL_DIV(DPLL_M5X2) | ID_DPLL_NO(DPLL4_PER))

#define PRCM_CSI2	(ID_OMAP(AT_3430_ES2) | ID_TYPE((ID_DEV | ID_DPLL_OP)) | ID_DEV_TYPE(INITIATOR)\
				| ID_DOMAIN(DOM_CAM) | ID_DEV_BIT_POS(0x1))

#define PRCM_CSIA	(ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(INIT_TAR)\
				 | ID_DOMAIN(DOM_CAM) | ID_DEV_BIT_POS(0x1))
#ifndef CONFIG_ARCH_OMAP3410
#define PRCM_CSIB	(ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(INIT_TAR)\
				 | ID_DOMAIN(DOM_CAM) | ID_DEV_BIT_POS(0x2))
#endif
#define PRCM_MMU        (ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(INIT_TAR)\
                                 | ID_DOMAIN(DOM_CAM) | ID_DEV_BIT_POS(0x3))
#define PRCM_ISP_CTRL   (ID_OMAP(AT_3430) | OTHER_ID_TYPE(ID_SYSCONF) | ID_DEV_TYPE(INIT_TAR)\
                                 | ID_DOMAIN(DOM_CAM) | ID_DEV_BIT_POS(0x4))
#define PRCM_CAM_CONSTRAINT	(ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_CAM))

/* Devices in PER Domain */
#define PRCM_MCBSP2	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x6) | ID_DEV_BIT_POS(0x0))
#define PRCM_MCBSP3	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x0) | ID_DEV_BIT_POS(0x1))
#define PRCM_MCBSP4	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x2) | ID_DEV_BIT_POS(0x2))
#define PRCM_GPT2	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x0) | ID_DEV_BIT_POS(0x3))
#define PRCM_GPT3	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x1) | ID_DEV_BIT_POS(0x4))
#define PRCM_GPT4	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x2) | ID_DEV_BIT_POS(0x5))
#define PRCM_GPT5	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x3) | ID_DEV_BIT_POS(0x6))
#define PRCM_GPT6	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x4) | ID_DEV_BIT_POS(0x7))
#define PRCM_GPT7	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x5) | ID_DEV_BIT_POS(0x8))
#define PRCM_GPT8	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x6) | ID_DEV_BIT_POS(0x9))
#define PRCM_GPT9	(ID_OMAP(AT_3430) | ID_TYPE((ID_DEV | ID_CLK_SRC))\
				| ID_DEV_TYPE(TARGET) |ID_DOMAIN(DOM_PER) | \
				ID_CLK_SRC_BIT_POS(0x7) | ID_DEV_BIT_POS(0xA))
#define PRCM_UART3	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				|ID_DOMAIN(DOM_PER) | ID_DEV_BIT_POS(0xB))
#define PRCM_WDT3	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				|ID_DOMAIN(DOM_PER) | ID_DEV_BIT_POS(0xC))
#define PRCM_GPIO2	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				|ID_DOMAIN(DOM_PER) | ID_DEV_BIT_POS(0xD))
#define PRCM_GPIO3	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				|ID_DOMAIN(DOM_PER) | ID_DEV_BIT_POS(0xE))
#define PRCM_GPIO4	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				|ID_DOMAIN(DOM_PER) | ID_DEV_BIT_POS(0xF))
#define PRCM_GPIO5	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				|ID_DOMAIN(DOM_PER) | ID_DEV_BIT_POS(0x10))
#define PRCM_GPIO6	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(TARGET)\
				|ID_DOMAIN(DOM_PER) | ID_DEV_BIT_POS(0x11))
#define PRCM_PER_CONSTRAINT	(ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_PER))

/* Devices in NEON Domain */
#define PRCM_NEON	(ID_OMAP(AT_3430) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(INITIATOR)\
				| ID_DOMAIN(DOM_NEON) | ID_DEV_BIT_POS(0x0))
#define PRCM_NEON_CONSTRAINT	(ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_NEON))

/* Devices in USBHOST Domain */
#define PRCM_USBHOST1	(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_DEV) | ID_DEV_TYPE(INITIATOR)\
				| ID_DOMAIN(DOM_USBHOST) | ID_DEV_BIT_POS(0x0))
#define PRCM_USBHOST2	(ID_OMAP(AT_3430_ES2) | ID_TYPE((ID_DEV | ID_DPLL_OP))\
				| ID_DEV_TYPE(INITIATOR) | ID_DOMAIN(DOM_USBHOST)\
				| ID_DEV_BIT_POS(0x1)| ID_DPLL_DIV(DPLL_M2)\
				| ID_DPLL_NO(DPLL5_PER2))
#define PRCM_USBHOST_CONSTRAINT	(ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_CLK) | \
				 ID_DOMAIN(DOM_USBHOST))

/* DPLL ID's */
#define PRCM_DPLL1_M2X2_CLK	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M2X2) | ID_DPLL_NO(DPLL1_MPU))
#define PRCM_DPLL2_M2X2_CLK	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M2X2) | ID_DPLL_NO(DPLL2_IVA2))
#define PRCM_DPLL3_M2_CLK 	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M2) | ID_DPLL_NO(DPLL3_CORE))
#define PRCM_DPLL3_M2X2_CLK	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M2X2) | ID_DPLL_NO(DPLL3_CORE))
#define PRCM_DPLL3_M3X2_CLK	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M3X2) | ID_DPLL_NO(DPLL3_CORE))
#define PRCM_DPLL4_M2X2_CLK	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M2X2) | ID_DPLL_NO(DPLL4_PER))
#define PRCM_DPLL4_M3X2_CLK 	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M3X2) | ID_DPLL_NO(DPLL4_PER))
#define PRCM_DPLL4_M6X2_CLK	(ID_OMAP(AT_3430) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M6X2) | ID_DPLL_NO(DPLL4_PER))
#define PRCM_DPLL5_M2_CLK	(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_DPLL_OP)\
				| ID_DPLL_DIV(DPLL_M2) | ID_DPLL_NO(DPLL5_PER2))

/* CLK ID's */
#define PRCM_L3_ICLK 		(ID_OMAP(AT_3430) | ID_TYPE(ID_CLK_DIV) |\
							ID_CLK_NO(0x1))
#define PRCM_L4_ICLK 		(ID_OMAP(AT_3430) | ID_TYPE(ID_CLK_DIV) |\
							ID_CLK_NO(0x2))
#define PRCM_RM_ICLK 		(ID_OMAP(AT_3430) | ID_TYPE(ID_CLK_DIV) |\
							ID_CLK_NO(0x3))
#define PRCM_SYS_CLKOUT2 	(ID_OMAP(AT_3430) | ID_TYPE((ID_CLK_DIV  | ID_CLK_SRC))\
							| ID_CLK_NO(0x5))
#define PRCM_DPLL1_FCLK		(ID_OMAP(AT_3430) | ID_TYPE(ID_CLK_DIV)\
							| ID_CLK_NO(0x8))
#define PRCM_DPLL2_FCLK		(ID_OMAP(AT_3430) | ID_TYPE(ID_CLK_DIV)\
							| ID_CLK_NO(0x9))
#define PRCM_96M_CLK		(ID_OMAP(AT_3430_ES2) | ID_TYPE(ID_CLK_SRC)\
							| ID_CLK_SRC_BIT_POS(0x6))
#define PRCM_NO_OF_CLKS		0xA
/* func_48m_fclk and func_12m_fclk have dividers */
/*but they are fixed dividers.*/
/*So these are not included in the array */
#define PRCM_48M_FCLK 		(ID_OMAP(AT_3430) | ID_TYPE((ID_CLK_DIV\
							| ID_CLK_SRC)) | ID_CLK_NO(0xB))
#define PRCM_12M_FCLK 		(ID_OMAP(AT_3430) | ID_TYPE(ID_CLK_DIV)\
							| ID_CLK_NO(0xE))
/* Clock IDs for parent clocks */
#define PRCM_SYS_ALT_CLK	(OMAP(AT_3430)| OTHER_ID_TYPE(ID_CLK_PARENT)\
							| ID_CLK_NO(0xF))
#define PRCM_SYS_CLK		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_CLK_PARENT)\
							| ID_CLK_NO(0x10))
#define PRCM_SYS_32K_CLK	(OMAP(AT_3430)| OTHER_ID_TYPE(ID_CLK_PARENT)\
							| ID_CLK_NO(0x11))
#define PRCM_EXT_MCBSP_CLK	(OMAP(AT_3430)| OTHER_ID_TYPE(ID_CLK_PARENT)\
							| ID_CLK_NO(0x12))
#define PRCM_SYS_CLKOUT1	(OMAP(AT_3430)| OTHER_ID_TYPE(ID_CLK_PARENT)\
							| ID_CLK_NO(0x13))
/* VDD1 OPPs */
#define PRCM_VDD1_OPP1		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD1)\
							| ID_OPP_NO(0x1))
#define PRCM_VDD1_OPP2		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD1)\
							| ID_OPP_NO(0x2))
#define PRCM_VDD1_OPP3		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD1)\
							| ID_OPP_NO(0x3))
#define PRCM_VDD1_OPP4		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD1)\
							| ID_OPP_NO(0x4))
#define PRCM_VDD1_OPP5		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD1)\
							| ID_OPP_NO(0x5))
#define PRCM_NO_VDD1_OPPS	5

/* VDD2 OPPs */
#define PRCM_VDD2_OPP1		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD2)\
							| ID_OPP_NO(0x1))
#define PRCM_VDD2_OPP2		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD2)\
							| ID_OPP_NO(0x2))
#define PRCM_VDD2_OPP3		(OMAP(AT_3430)| OTHER_ID_TYPE(ID_OPP) | ID_VDD(PRCM_VDD2)\
							| ID_OPP_NO(0x3))
#define PRCM_NO_VDD2_OPPS	3

/* OPP and Frequency Constraints */
#define PRCM_VDD1_CONSTRAINT 	(ID_OMAP(AT_3430) | \
				OTHER_ID_TYPE(ID_CONSTRAINT_OPP) | 0x1)
#define PRCM_VDD2_CONSTRAINT 	(ID_OMAP(AT_3430) | \
				OTHER_ID_TYPE(ID_CONSTRAINT_OPP) | 0x2)
#define PRCM_ARMFREQ_CONSTRAINT (ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_FREQ) | 0x3)
#define PRCM_DSPFREQ_CONSTRAINT (ID_OMAP(AT_3430) | \
				 OTHER_ID_TYPE(ID_CONSTRAINT_FREQ) | 0x4)

/* Mpu power domain states */
#define PRCM_MPU_ACTIVE		(OTHER_ID_TYPE(ID_MPU_DOM_STATE) | 0x0)
#define PRCM_MPU_INACTIVE	(OTHER_ID_TYPE(ID_MPU_DOM_STATE) | 0x1)
#define PRCM_MPU_CSWR_L2RET	(OTHER_ID_TYPE(ID_MPU_DOM_STATE) | 0x3)
#define PRCM_MPU_CSWR_L2OFF	(OTHER_ID_TYPE(ID_MPU_DOM_STATE) | 0x7)
#define PRCM_MPU_OSWR_L2RET	(OTHER_ID_TYPE(ID_MPU_DOM_STATE) | 0xF)
#define PRCM_MPU_OSWR_L2OFF	(OTHER_ID_TYPE(ID_MPU_DOM_STATE) | 0x1F)
#define PRCM_MPU_OFF		(OTHER_ID_TYPE(ID_MPU_DOM_STATE) | 0x3F)

/* Core power domain states */
#define PRCM_CORE_ACTIVE	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x0)
#define PRCM_CORE_INACTIVE	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x1)
#define PRCM_CORE_CSWR_MEMRET	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x3)
#define PRCM_CORE_CSWR_MEM1OFF	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x7)
#define PRCM_CORE_CSWR_MEM2OFF	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0xF)
#define PRCM_CORE_CSWR_MEMOFF	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x1F)
#define PRCM_CORE_OSWR_MEMRET	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x3F)
#define PRCM_CORE_OSWR_MEM1OFF	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x7F)
#define PRCM_CORE_OSWR_MEM2OFF	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0xFF)
#define PRCM_CORE_OSWR_MEMOFF	(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x1FF)
#define PRCM_CORE_OFF		(OTHER_ID_TYPE(ID_CORE_DOM_STATE) | 0x3FF)

/* Memory and Logic resources */
#define PRCM_MPU_L2CACHEON	(OTHER_ID_TYPE(ID_MEMORY_RES) | \
						ID_DOMAIN(DOM_MPU) | 0x1)
#define PRCM_MPU_L2CACHERET	(OTHER_ID_TYPE(ID_MEMORY_RES) | \
						ID_DOMAIN(DOM_MPU) | 0x2)
#define PRCM_MPU_LOGICL1CACHERET	(OTHER_ID_TYPE(ID_LOGIC_RES) | \
						ID_DOMAIN(DOM_MPU) | 0x3)

#define PRCM_CORE_MEM2ON	(OTHER_ID_TYPE(ID_MEMORY_RES) | \
						ID_DOMAIN(DOM_CORE1) | 0x1)
#define PRCM_CORE_MEM1ON	(OTHER_ID_TYPE(ID_MEMORY_RES) | \
						ID_DOMAIN(DOM_CORE1) | 0x2)
#define PRCM_CORE_MEM2RET	(OTHER_ID_TYPE(ID_MEMORY_RES) | \
						ID_DOMAIN(DOM_CORE1) | 0x3)
#define PRCM_CORE_MEM1RET	(OTHER_ID_TYPE(ID_MEMORY_RES) | \
						ID_DOMAIN(DOM_CORE1) | 0x4)
#define PRCM_CORE_LOGICRET	(OTHER_ID_TYPE(ID_LOGIC_RES) | \
						ID_DOMAIN(DOM_CORE1) | 0x5)
#define  PRCM_CORE_MEMORYCHANGE (OTHER_ID_TYPE(ID_LOGIC_RES) | \
						ID_DOMAIN(DOM_CORE1) | 0x6)
#define  PRCM_CORE_POWERSTATE   (OTHER_ID_TYPE(ID_LOGIC_RES) | \
						ID_DOMAIN(DOM_CORE1) | 0x7)

#define PRCM_PER_LOGICRET	(OTHER_ID_TYPE(ID_LOGIC_RES) | \
						ID_DOMAIN(DOM_PER) | 0x1)

/* VDD1 and VDD2 sleep states */
#define PRCM_VDD_ACTIVE		1
#define PRCM_VDD_RET		2
#define PRCM_VDD_OFF		3

#define PRCM_WAKEUP_T2_KEYPAD	0x1
#define PRCM_WAKEUP_TOUCHSCREEN	0x2
#define PRCM_WAKEUP_UART1	0x4

/* PRM_VC_SMPS_SA */
#define PRM_VC_SMPS_SA1_SHIFT	16
#define PRM_VC_SMPS_SA0_SHIFT	0
#define PRM_VC_SMPS_SA1_MASK	(0x7F << PRM_VC_SMPS_SA1_SHIFT)
#define PRM_VC_SMPS_SA0_MASK	(0x7F << PRM_VC_SMPS_SA0_SHIFT)

/* PRM_VC_SMPS_VOL_RA */
#define PRM_VC_SMPS_VOLRA1_SHIFT	16
#define PRM_VC_SMPS_VOLRA0_SHIFT	0
#define PRM_VC_SMPS_VOLRA1_MASK		(0xFF << PRM_VC_SMPS_VOLRA1_SHIFT)
#define PRM_VC_SMPS_VOLRA0_MASK		(0xFF << PRM_VC_SMPS_VOLRA0_SHIFT)

/* PRM_VC_SMPS_CMD_RA */
#define PRM_VC_SMPS_CMDRA1_SHIFT	16
#define PRM_VC_SMPS_CMDRA0_SHIFT	0
#define PRM_VC_SMPS_CMDRA1_MASK		(0xFF << PRM_VC_SMPS_CMDRA1_SHIFT)
#define PRM_VC_SMPS_CMDRA0_MASK		(0xFF << PRM_VC_SMPS_CMDRA0_SHIFT)

/* PRM_VC_CMD_VAL_0 specific bits */
#if 0 /* Original TI drop values. */
#define PRM_VC_CMD_VAL0_ON		0x30
#define PRM_VC_CMD_VAL0_ONLP		0x18
#define PRM_VC_CMD_VAL0_RET		0x18
#define PRM_VC_CMD_VAL0_OFF		0x18

#else /* David Derrick's values. */
#define PRM_VC_CMD_VAL0_ON		0x30
#define PRM_VC_CMD_VAL0_ONLP		0x20
#define PRM_VC_CMD_VAL0_RET		0x18
#define PRM_VC_CMD_VAL0_OFF		0x00
#endif

/* PRM_VC_CMD_VAL_1 specific bits */
#if 0 /* Original TI drop values. */
#define PRM_VC_CMD_VAL1_ON		0x2C
#define PRM_VC_CMD_VAL1_ONLP		0x18
#define PRM_VC_CMD_VAL1_RET		0x18
#define PRM_VC_CMD_VAL1_OFF		0x18

#else /* David Derrick's values. */
#define PRM_VC_CMD_VAL1_ON		0x30
#define PRM_VC_CMD_VAL1_ONLP		0x20
#define PRM_VC_CMD_VAL1_RET		0x18
#define PRM_VC_CMD_VAL1_OFF		0x00
#endif

#define PRM_VC_CMD_ON_SHIFT		24
#define PRM_VC_CMD_ONLP_SHIFT		16
#define PRM_VC_CMD_RET_SHIFT		8
#define PRM_VC_CMD_OFF_SHIFT		0
#define PRM_VC_CMD_ON_MASK		(0xFF << PRM_VC_CMD_ON_SHIFT)
#define PRM_VC_CMD_ONLP_MASK		(0xFF << PRM_VC_CMD_ONLP_SHIFT)
#define PRM_VC_CMD_RET_MASK		(0xFF << PRM_VC_CMD_RET_SHIFT)
#define PRM_VC_CMD_OFF_MASK		(0xFF << PRM_VC_CMD_OFF_SHIFT)

/* PRM_VC_BYPASS_VAL */
#define PRM_VC_BYPASS_VALID		(0x1 << 24)
#define PRM_VC_BYPASS_DATA_SHIFT	16
#define PRM_VC_BYPASS_REGADDR_SHIFT	8
#define PRM_VC_BYPASS_SLAVEADDR_SHIFT	0
#define PRM_VC_BYPASS_DATA_MASK		(0xFF << PRM_VC_BYPASS_DATA_SHIFT)
#define PRM_VC_BYPASS_REGADDR_MASK	(0xFF << PRM_VC_BYPASS_REGADDR_SHIFT)
#define PRM_VC_BYPASS_SLAVEADDR_MASK	(0x7F << PRM_VC_BYPASS_SLAVEADDR_SHIFT)

/* PRM_VC_CH_CONF */
#define PRM_VC_CH_CONF_CMD1		(0x1 << 20)
#define PRM_VC_CH_CONF_RAV1		(0x1 << 17)

/* PRM_VC_I2C_CFG */
#define PRM_VC_I2C_CFG_MCODE		0x0
#define PRM_VC_I2C_CFG_HSEN		(0x1 << 3)
#define PRM_VC_I2C_CFG_SREN		(0x1 << 4)
#define PRM_VC_I2C_CFG_HSMASTER		(0x1 << 5)

/* PRM_VOLTCTRL */
#define PRM_VOLTCTRL_AUTO_SLEEP		0x1
#define PRM_VOLTCTRL_AUTO_RET		0x2
#define PRM_VOLTCTRL_AUTO_OFF		0x4
#define PRM_VOLTCTRL_SEL_OFF		0x8
#define PRM_VOLTCTRL_SEL_VMODE		0x10

/* Constants to define setup durations */
#define PRM_CLKSETUP_DURATION		0xFF
#define PRM_VOLTSETUP_TIME2		0xFFF
#define PRM_VOLTSETUP_TIME1		0xFFF
#define PRM_VOLTOFFSET_DURATION		0xFF
#define PRM_VOLTSETUP2_DURATION		0xFF

/* PRM_VOLTSETUP1 */
#define PRM_VOLTSETUP_TIME2_OFFSET	16
#define PRM_VOLTSETUP_TIME1_OFFSET	0

/* PRM_POLCTRL */
#define PRM_POLCTRL_EXTVOL_POL		(1<<0)
#define PRM_POLCTRL_CLKREQ_POL		(1<<1)
#define PRM_POLCTRL_CLKOUT_POL		(1<<2)
#define PRM_POLCTRL_OFFMODE_POL		(1<<3)

/* PRM_IRQENABLE_MPU */
#define PRM_VC_TIMEOUTERR_EN		(0x1 << 24)
#define PRM_VC_RAERR_EN			(0x1 << 23)
#define PRM_VC_SAERR_EN			(0x1 << 22)

/* PRM_IRQSTATUS_MPU */
#define PRM_VC_TIMEOUTERR_ST		(0x1 << 24)
#define PRM_VC_RAERR_ST			(0x1 << 23)
#define PRM_VC_SAERR_ST			(0x1 << 22)

/* T2 SMART REFLEX */
#define R_SRI2C_SLAVE_ADDR		0x12
#define R_VDD1_SR_CONTROL		0x00
#define R_VDD2_SR_CONTROL		0x01
#define T2_SMPS_UPDATE_DELAY		600	/* In uSec */

/* GPTimer wait delay */
#define GPTIMER_WAIT_DELAY		50	/* In usec */

#define SDRC_PWR_AUTOCOUNT_MASK		(0xFFFF << 8)
#define SDRC_PWR_CLKCTRL_MASK		(0x3 << 4)


/* PRM_LDO_ABB_SETUP */
#define OMAP3630_SR2_IN_TRANSITION			(1 << 6)
#define OMAP3630_SR2_STATUS_SHIFT			3
#define OMAP3630_SR2_STATUS_MASK			(0x3 << 3)
#define OMAP3630_OPP_CHANGE				(1 << 2)
#define OMAP3630_OPP_SEL_SHIFT				0
#define OMAP3630_OPP_SEL_MASK				(0x3 << 0)

/* PRM_LDO_ABB_CTRL */
#define OMAP3630_SR2_WTCNT_VALUE_SHIFT			8
#define OMAP3630_SR2_WTCNT_VALUE_MASK			(0xff << 8)
#define OMAP3630_SLEEP_RBB_SEL				(1 << 3)
#define OMAP3630_ACTIVE_FBB_SEL				(1 << 2)
#define OMAP3630_ACTIVE_RBB_SEL				(1 << 1)
#define OMAP3630_SR2EN					(1 << 0)

/* PRM_IRQENABLE_MPU specific bits */
#define OMAP3630_VC_BYPASS_ACK_EN			(1 << 28)
#define OMAP3630_VC_VP1_ACK_EN				(1 << 27)
#define OMAP3630_ABB_LDO_TRANXDONE_ST			(1 << 26)
#define OMAP3630_VP1_TRANXDONE_ST			(1 << 15)
#define OMAP3630_VP2_TRANXDONE_ST			(1 << 21)

#define OMAP3630_VP_FORCEUPDATE				(1 << 1)
#define OMAP3630_VP_INITVDD				(1 << 2)
#define OMAP3630_VP_INITVOLTAGE_SHIFT			8
#define OMAP3630_VP_INITVOLTAGE				(0xFF << OMAP3630_VP_INITVOLTAGE_SHIFT)


void omap_sram_idle(u32 mpu_target_state);
extern u32 omap3_configure_core_dpll(
			u32 m2, u32 unlock_dll, u32 f, u32 inc,
			u32 sdrc_rfr_ctrl_0, u32 sdrc_actim_ctrl_a_0,
			u32 sdrc_actim_ctrl_b_0, u32 sdrc_mr_0,
			u32 sdrc_rfr_ctrl_1, u32 sdrc_actim_ctrl_a_1,
			u32 sdrc_actim_ctrl_b_1, u32 sdrc_mr_1);



/* Structure to save interrupt controller context */
struct int_controller_context {
	u32 sysconfig;
	u32 protection;
	u32 idle;
	u32 threshold;
	u32 ilr[96];
	u32 mir_0;
	u32 mir_1;
	u32 mir_2;
};

/* Structure to save control module context */
struct control_module_context {
	u32 sysconfig;
	u32 devconf0;
	u32 mem_dftrw0;
	u32 mem_dftrw1;
	u32 msuspendmux_0;
	u32 msuspendmux_1;
	u32 msuspendmux_2;
	u32 msuspendmux_3;
	u32 msuspendmux_4;
	u32 msuspendmux_5;
	u32 sec_ctrl;
	u32 devconf1;
	u32 csirxfe;
	u32 iva2_bootaddr;
	u32 iva2_bootmod;
	u32 debobs_0;
	u32 debobs_1;
	u32 debobs_2;
	u32 debobs_3;
	u32 debobs_4;
	u32 debobs_5;
	u32 debobs_6;
	u32 debobs_7;
	u32 debobs_8;
	u32 prog_io0;
	u32 prog_io1;
	u32 dss_dpll_spreading;
	u32 core_dpll_spreading;
	u32 per_dpll_spreading;
	u32 usbhost_dpll_spreading;
	u32 pbias_lite;
	u32 temp_sensor;
	u32 sramldo4;
	u32 sramldo5;
	u32 csi;
};

/* Structure to save gpmc cs context */
struct gpmc_cs_context {
	int cs_valid;
	u32 config1;
	u32 config2;
	u32 config3;
	u32 config4;
	u32 config5;
	u32 config6;
	u32 config7;
};

/* Structure to save gpmc context */
struct gpmc_context {
	u32 sysconfig;
	u32 irqenable;
	u32 timeout_ctrl;
	u32 config;
	u32 prefetch_config1;
	u32 prefetch_config2;
	u32 prefetch_control;
	struct gpmc_cs_context cs0_context;
	struct gpmc_cs_context cs1_context;
	struct gpmc_cs_context cs2_context;
	struct gpmc_cs_context cs3_context;
	struct gpmc_cs_context cs4_context;
	struct gpmc_cs_context cs5_context;
	struct gpmc_cs_context cs6_context;
	struct gpmc_cs_context cs7_context;
};

/* Structure to save neon context */
struct neon_context {
	u32 fpscr;
	u32 fpexc;
	u32 fpregs[64];
};

/* Structure to save usbtll context */

struct usbtll_context {
	u32 usbtll_sysconfig;
	u32 usbtll_irqenable;
	u32 tll_shared_conf;
	u32 tll_channel_conf[3];
	u32 ulpi_function_ctrl[3];
	u32 ulpi_interface_ctrl[3];
	u32 ulpi_otg_ctrl[3];
	u32 ulpi_usb_int_en_rise[3];
	u32 ulpi_usb_int_en_fall[3];
	u32 ulpi_usb_int_status[3];
	u32 ulpi_vendor_int_en[3];
	u32 ulpi_vendor_int_status[3];
};

/* Register Types */
enum regtype {
	REG_FCLKEN,
	REG_ICLKEN,
	REG_IDLEST,
	REG_AUTOIDLE,
	REG_WKEN,
	REG_WKST,
	REG_CLKSTCTRL,
	REG_CLKSTST,
	REG_PWSTCTRL,
	REG_PWSTST,
	REG_PREPWSTST,
	REG_CLK_SRC,
	REG_RSTST,
	PRCM_REG_TYPES,
};

enum dpll_regtype {
	REG_CLKEN_PLL,
	REG_AUTOIDLE_PLL,
	REG_CLKSEL1_PLL,
	REG_IDLEST_PLL,
	REG_M2_DIV_PLL,
	REG_M2X2_DIV_PLL,
	REG_M3_DIV_PLL,
	REG_M4_DIV_PLL,
	REG_M5_DIV_PLL,
	REG_M6_DIV_PLL,
	PRCM_DPLL_REG_TYPES,
};

/* Offset of MX registers in the pll_register array*/
#define MX_ARRAY_OFFSET REG_M2_DIV_PLL

/* Structure to store the attributes of register */
struct reg_def {
	u32 valid_bits;
	volatile u32 *reg_addr;
};

/* Structure to store attributes of registers in a domain */
struct domain_registers {
	struct reg_def regdef[PRCM_REG_TYPES];
};

extern struct domain_registers prcm_domain_register[PRCM_NUM_DOMAINS];

/* Structure to store DPLL information */
struct dpll_registers {
	struct reg_def regdef[PRCM_DPLL_REG_TYPES];
};

/* Structure for mpu, core and VDD states */
/* This structure will be used in OS Idle code */
struct mpu_core_vdd_state {
	u32 mpu_state;
	u32 core_state;
	u32 vdd_state;
};

/* SDRC/GPMC params for DVFS */

struct sdrc_config {
	u32 sdrc_rfr_ctrl;
	u32 sdrc_actim_ctrla;
	u32 sdrc_actim_ctrlb;
};

struct gpmc_config {
	u32 gpmc_config2;
	u32 gpmc_config3;
	u32 gpmc_config4;
	u32 gpmc_config5;
	u32 gpmc_config6;
};

#define no_sdrc_cs 2
#define no_gpmc_cs 4

struct dvfs_config {
	struct sdrc_config sdrc_cfg[no_sdrc_cs];
	struct gpmc_config gpmc_cfg[no_gpmc_cs];
};

int prcm_clock_control(u32 deviceid, u8 clk_type, u8 control,
			      u8 checkaccessibility);
int prcm_is_device_accessible(u32 deviceid, u8 *result);
int prcm_interface_clock_autoidle(u32 deviceid, u8 control);
int prcm_wakeup_event_control(u32 deviceid, u8 control);

/* OPP Accessor functions */
unsigned short get_vdd1_arm_constraint_for_freq(unsigned int freq);
unsigned short get_vdd1_dsp_constraint_for_freq(unsigned int freq);
unsigned int get_arm_freq_for_opp(unsigned int opp);
unsigned int get_dsp_freq_for_opp(unsigned int opp);
unsigned int get_opp_for_index(unsigned int idx);

/******************************************************************************
 *
 * DPLL control
 *
 ******************************************************************************/

int prcm_enable_dpll(u32 dpll);
int prcm_configure_dpll(u32 dpll, u32 mult, u8 div, u8 freq_sel);
int prcm_put_dpll_in_bypass(u32 dpll, u32 bypass_mode);
int prcm_dpll_clock_auto_control(u32 dpll, u32 setting);
int prcm_get_dpll_mn_output(u32 dpll, u32 *mn_output);
int prcm_get_dpll_rate(u32 dpll, u32 *rate);
int prcm_configure_dpll_divider(u32 dpll, u32 setting);

/******************************************************************************
 *
 * Other
 *
 ******************************************************************************/

int prcm_set_clock_domain_state(u32 domainid, u8 new_state, 
						u8 check_state);
int prcm_get_domain_interface_clocks(u32 domainid, u32 *result);
int prcm_get_domain_functional_clocks(u32 domainid, u32 *result);
int prcm_set_domain_interface_clocks(u32 domainid, u32 setmask);
int prcm_set_domain_functional_clocks(u32 domainid, u32 setmask);
int prcm_get_crystal_rate(void);
int prcm_get_system_clock_speed(void);
int prcm_select_system_clock_divider(u32 setting);
int prcm_control_external_output_clock1(u32 control);
int prcm_control_external_output_clock2(u32 control);
int prcm_select_external_output_clock2_divider(u32 setting);
int prcm_select_external_output_clock2_source(u32 setting);
int prcm_clksel_get_divider(u32 clk, u32 *div);
int prcm_clksel_set_divider(u32 clk, u32 div);
int prcm_get_processor_speed(u32 domainid, u32 *processor_speed);
int prcm_clksel_round_rate(u32 clk, u32 parent_rate, u32 target_rate,
				  u32 *newdiv);
int prcm_clk_set_source(u32 clk_id, u32 parent_id);
int prcm_clk_get_source(u32 clk_id, u32 *parent_id);
int prcm_set_power_configuration(u32 domainid, u8 idlemode, 
					u8 standbymode, u8 autoidleenable);
int prcm_set_wkup_dependency(u32 domainid, u32 wkup_dep);
int prcm_set_sleep_dependency(u32 domainid, u32 sleep_dep);
int prcm_clear_wkup_dependency(u32 domainid, u32 wkup_dep);
int prcm_clear_sleep_dependency(u32 domainid, u32 sleep_dep);
int prcm_set_mpu_domain_state(u32 mpu_dom_state);
int prcm_set_core_domain_state(u32 core_dom_state);

void prcm_save_core_context(u32 target_core_state);

int prcm_init(void);
int prcm_set_device_power_configuration(u32 deviceid, u8 idlemode, 
					u8 standbymode, u8 autoidleenable);
/******************************************************************************
 *
 * Level 2 PRCM API
 *
 ******************************************************************************/

int prcm_do_frequency_scaling(u32 target_opp, u32 current_opp);
void omap_dma_global_context_restore(void);
void omap_dma_global_context_save(void);
int prcm_lock_iva_dpll(u32 target_opp);
void clear_domain_reset_status(void);
u32 omap_prcm_get_reset_sources(void);

int sr_voltagescale(u32 target_opp, u32 cur_opp, u32 vsel_ext);

/******************************************************************************
 *
 * Section for prcm_vdd.c
 *
 ******************************************************************************/

#endif
