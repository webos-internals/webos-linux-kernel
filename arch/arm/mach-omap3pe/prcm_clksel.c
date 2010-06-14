/*
 * linux/arch/arm/mach-omap3pe/prcm_clksel.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP 34xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu/Rajendra Nayak/Pavan Chinnabhandar
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
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <asm/arch/prcm.h>
#include <linux/delay.h>
#include <asm/io.h>

#include "prcm-regs.h"
#include <asm/arch/power_companion.h>


/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif


/******************************************************************************
 *
 * Table to store clksel registers for various clocks
 *
 ******************************************************************************/

static struct reg_def clksel_reg[PRCM_NO_OF_CLKS] = {
	{0x3, &CM_CLKSEL_CORE},		/* L3_ICLK */
	{0xC, &CM_CLKSEL_CORE},		/* L4_ICLK */
	{0x6, &CM_CLKSEL_WKUP},		/* RM_ICLK */
	{0x00, NULL},
	{0x38, &CM_CLKOUT_CTRL},	/* SYS_CLOCKOUT2 */
#ifndef CONFIG_ARCH_OMAP3410
	{0x7, &CM_CLKSEL_SGX},		/* SGX_L3_FCLK */
#else
	{0x00, NULL},
#endif
	{0xF00, &CM_CLKSEL_CORE},	/* SSI */
	{0x180000, &CM_CLKSEL1_PLL_MPU},/* DPLL1_FCLK */
	{0x180000, &CM_CLKSEL1_PLL_IVA2},/* DPLL2_FCLK */
	{0x78, &CM_CLKSEL_WKUP}, 	/* USIM_FCLK */

};

/******************************************************************************
 *
 * Offset values in registers for clocks which have dividers
 *
 ******************************************************************************/

static u32 clk_div_offset[PRCM_NO_OF_CLKS] = {
	0x0,	/* L3_ICLK */
	0x2,	/* L4_ICLK */
	0x1,	/* RM_ICLK */
	0x0,	/* USB_L4_ICLK */
	0x3,	/* SYS_CLKOUT2 */
	0x0,	/* SGX_L3_FCLK */
	0x8,	/* SSI_CLK */
	0x13,	/* DPLL1_FCLK */
	0x13,	/* DPLL2_FCLK */
	0x3,	/* SYS_USIM_CLK */
};

/******************************************************************************
 *
 * Possible values for DPLL dividers
 *
 ******************************************************************************/

static u32 div_dpll[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };

static u32 div_dpll3[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
	           17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };

/******************************************************************************
 *
 * Array containing possible divider values for all clocks which have dividers
 * 
 ******************************************************************************/

static u32 div_arr[] = {
	1, 2, 0, 0, 0, 0,	/* L3 divs */
	1, 2, 0, 0, 0, 0,	/* L4 divs */
	1, 2, 0, 0, 0, 0,	/* RM divs */
	0, 0, 0, 0, 0, 0,	/* USB L4 divs */
	1, 2, 4, 8, 16, 0,	/* sys clkout2 divs */
#ifndef CONFIG_ARCH_OMAP3410
	3, 4, 6, 0, 0, 0,	/* sgx divs */
#else
	0, 0, 0, 0, 0, 0,	/* sgx divs */
#endif
	1, 2, 3, 4, 5, 6,	/* SSI divs */
	1, 2, 4, 0, 0, 0,	/* dpll1 fclk divs */
	1, 2, 4, 0, 0, 0,	/* dpll2 fclk divs */
	2, 4, 8, 10, 16, 20,	/* cm usim divs */
};

/*============================================================================*/
/*======================== GET CLOCK DIVIDER =================================*/
/*= This function will return the divider for the clocks which are specified =*/
/*= by the clk parameter.                                                    =*/
/*= For 48M and 12M functional clocks, the dividers 2 and 4 are returned     =*/
/*= respectively since they do not have a divider. For all the other clocks, =*/
/*= the divider is returned after reading the correct register.              =*/
/*============================================================================*/

int prcm_clksel_get_divider(u32 clk, u32 *div)
{
	u32 valid, offset, clk_id, omap, id_type;
	volatile u32 *addr;

	id_type = get_id_type(clk);
	if (!(id_type & ID_CLK_DIV))
		return PRCM_FAIL;	/*No divider */

	omap = OMAP(clk);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	/* In case of 48m_fclk and 12m_fclk, the dividers are fixed */
	/* So this information is not stored in an array */
	if (clk == PRCM_48M_FCLK) {
		(*div) = 2;
		return PRCM_PASS;
	} else if (clk == PRCM_12M_FCLK) {
		(*div) = 4;
		return PRCM_PASS;
	}
	clk_id = (clk >> CLK_NO_POS) & CLK_NO_MASK;
	DPRINTK(" clkid: %d\n", clk_id);
	offset = clk_div_offset[clk_id-1];

	DPRINTK(" offset: %d\n", offset);
	if (clk_id > PRCM_NO_OF_CLKS)
		return PRCM_FAIL;

	addr = clksel_reg[clk_id - 1].reg_addr;
	if (!addr)
		return PRCM_FAIL;
	valid = clksel_reg[clk_id - 1].valid_bits;

	DPRINTK(" valid: %x\n", valid);

	*div = (*addr & valid) >> offset;
	/* For sys_clkout2, the divider is not same as value in register */
	/* Need to do bit shifting to get actual divider value */
	if (clk == PRCM_SYS_CLKOUT2)
		*div = (1 << (*div));
	if (clk == PRCM_USIM) {
		switch (*div) {
		case 0x3:
			*div = 2;
			break;
		case 0x5:
			*div = 8;
			break;
		case 0x6:
			*div = 10;
			break;
		case 0x7:
			*div = 4;
			break;
		case 0x9:
			*div = 16;
			break;
		case 0xA:
			*div = 20;
			break;
		default:
			break;
		}
	}
#ifndef CONFIG_ARCH_OMAP3410
	if (clk == PRCM_SGX_FCLK) {
		switch (*div) {
		case 0x0:
			*div = 3;
			break;
		case 0x1:
			*div = 4;
			break;
		case 0x2:
			*div = 6;
			break;
		case 0x3:
			*div = 1;
			break;
		default:
			break;
		}
	}
#endif
	DPRINTK(" output: %d\n", *div);
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET CLOCK DIVIDER =================================*/
/*= This function will set the divider for the clocks which are specified    =*/
/*= by the clk parameter.                                                    =*/
/*= For 48M and 12M functional clocks, the register to set the divider does  =*/
/*= not exist and hence PRCM_FAIL is returned. For all the other clocks,     =*/
/*= the divider is set in the correct register. In this case the function    =*/
/*= will return PRCM_PASS.                                                   =*/
/*============================================================================*/

int prcm_clksel_set_divider(u32 clk, u32 div)
{
	u32 valid, offset, clk_id, new_val, omap, id_type;
	volatile u32 *addr;
	u32 divider = 0;
	u8 reenable_clk = PRCM_FALSE;
	u32 parent_id;

	id_type = get_id_type(clk);
	if (!(id_type & ID_CLK_DIV))
		return PRCM_FAIL;	/*No divider */

	omap = OMAP(clk);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	if (clk == PRCM_48M_FCLK)
		return PRCM_FAIL;
	else if (clk == PRCM_12M_FCLK)
		return PRCM_FAIL;
	if (clk == PRCM_USIM) {
		prcm_clk_get_source(PRCM_USIM, &parent_id);
		switch (div) {
		case 2:
			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				div = 0x3;
			break;
		case 4:
			if (parent_id == PRCM_DPLL5_M2_CLK)
				div = 0x7;
				break;
		case 8:
			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				div = 0x5;
			break;
		case 10:
			div = 0x6;
			break;
		case 16:
			div = 0x9;
			break;
		case 20:
			div = 0xA;
			break;
		default:
			break;
		}
	}
#ifndef CONFIG_ARCH_OMAP3410
	if (clk == PRCM_SGX_FCLK) {
		switch (div) {
		case 3:
			div = 0x0;
			break;
		case 4:
			div = 0x1;
			break;
		case 6:
			div = 0x2;
			break;
		case 1:
			div = 0x3;
			break;
		default:
			break;
		}
	}
#endif
	clk_id = (clk >> CLK_NO_POS) & CLK_NO_MASK;
	offset = clk_div_offset[clk_id-1];

	if (clk_id > PRCM_NO_OF_CLKS)
		return PRCM_FAIL;

	addr = clksel_reg[clk_id - 1].reg_addr;
	if (!addr)
		return PRCM_FAIL;
	valid = clksel_reg[clk_id - 1].valid_bits;

	/* In case of sysclkout2, the value to be programmed in the register
	   is 0 for divider 1, 1 for divider 2, 2 for divider 4,
	   3 for divider 8 and 4 for divider 16 */
	if (clk == PRCM_SYS_CLKOUT2) {
		if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
			/* WORKAROUND FOR SILICON ERRATA 1.37 */
			/* Disabling sysclkout2, bit 7 of CM_CLKOUT_CTRL */
			if (*addr&(1 << 7)) {
				*addr &= (~(1 << 7));
				reenable_clk = PRCM_TRUE;
			}
		}

		for (divider = 0; divider <= 4; divider++) {
			if (div & 0x1) {
				div = divider;
				break;
			}
			div >>= 1;
		}
	}
	DPRINTK("Divider:%d\n", div);

	new_val = (*addr & ~valid) | (div << offset);
	*addr = new_val;

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0) && (clk == PRCM_SYS_CLKOUT2)
							 && (reenable_clk)) {
			/* WORKAROUND FOR SILCON ERRATA 1.37 */
			/* Enabling sysclkout2, bit 7 of CM_CLKOUT_CTRL */
			*addr |= (1 << 7);
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SELECT ROUND RATE =================================*/
/*= This function will return the round rate for the specified clock.        =*/
/*= Two arrays of valid dividers is maintained and the new divider is        =*/
/*= calculated using the elements of the array.                              =*/
/*= The function returns PRCM_FAIL if the clock id is invalid.               =*/
/*= In all the other cases it returns PRCM_PASS.                             =*/
/*============================================================================*/

int prcm_clksel_round_rate(u32 clk, u32 parent_rate, u32 target_rate,
			   u32 *newdiv)
{
	u32 omap, max_div = 6, div;
	u32 index, clk_id, test_rate, dpll_id;
	u32 id_type;
	u32 div_96M[] = {2, 4, 8, 10, 0, 0};
	u32 div_120M[] = {4, 8, 16, 20, 0, 0};
	u32 div_sys[] = {1, 2, 0, 0, 0, 0};
#ifndef CONFIG_ARCH_OMAP3410
	u32 div_core[] = {3, 4, 6, 0, 0, 0};
#endif
	u32 *div_array = NULL;


	*newdiv = 0;
	omap = OMAP(clk);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	id_type = get_id_type(clk);

	if (is_sil_rev_equal_to(OMAP3430_REV_ES1_0)) {
		/* Workaround for limiation 2.5 */
		/* On ES 1.0, DPLL4 dividers cannot be changed */
		if (id_type & ID_DPLL_OP) {
			dpll_id = (clk >> DPLL_NO_POS) & DPLL_NO_MASK;
			if (dpll_id == DPLL4_PER)
				return PRCM_FAIL;
			/* For DPLL3, divider can only be changed */
						/*through OPP change */
			if (clk == PRCM_DPLL3_M3X2_CLK)
				return PRCM_FAIL;
		}
	}

	if (id_type & ID_DPLL_OP) {
		switch (clk) {
		case PRCM_DPLL3_M3X2_CLK:
			for (index = 0; index < 31; index++) {
				if (!div_dpll3[index])
					return PRCM_FAIL;
				test_rate = parent_rate / div_dpll3[index];
				if (test_rate <= target_rate) {
					*newdiv = div_dpll3[index];
					return PRCM_PASS;
				}
			}
			break;
		case PRCM_DPLL5_M2_CLK:
		case PRCM_DPLL4_M2X2_CLK:
		case PRCM_DPLL4_M3X2_CLK:
		case PRCM_DSS:
		case PRCM_CAM:
		case PRCM_DPLL4_M6X2_CLK:
			for (index = 0; index < 16; index++) {
				if (!div_dpll[index]) {
					/* We have hit a NULL element
					 * which means dividers are exhausted */
					return PRCM_FAIL;
				}
				test_rate = parent_rate / div_dpll[index];
				if (test_rate <= target_rate) {
					*newdiv = div_dpll[index];
					return PRCM_PASS;
				}
			}
			break;
		default:
			/* No acceptable divider or divider
			cannot be changed on the fly */
			return PRCM_FAIL;
		}
	}

	if (!(id_type & ID_CLK_DIV))
		return PRCM_FAIL;	/* Clock rate cannot be changed */

	/* For 48M Fclk and 12M Fclk, the dividers are fixed.
	 * So we return the fixed dividers */
	if (clk == PRCM_48M_FCLK) {
		*newdiv = 2;
		return PRCM_PASS;
	}
	if (clk == PRCM_12M_FCLK) {
		*newdiv = 4;
		return PRCM_PASS;
	}
	if (clk == PRCM_USIM) {
		if (parent_rate == 96000000)
			div_array = div_96M;
		else if (parent_rate == 120000000)
			div_array = div_120M;
		else
			div_array = div_sys;
		for (index = 0; index < max_div; index++) {
			if (!(*div_array))
				return PRCM_FAIL;
			test_rate = parent_rate / *div_array;
			if (test_rate <= target_rate) {
				*newdiv = *div_array;
				return PRCM_PASS;
			}
			++div_array;
		}
		return PRCM_FAIL;
	}
#ifndef CONFIG_ARCH_OMAP3410
	if (clk == PRCM_SGX_FCLK) {
		if (parent_rate == 96000000) {
			*newdiv = 1;
			return PRCM_PASS;
			}
		else {
			div_array = div_core;
			for (index = 0; index < max_div; index++) {
				if (!(*div_array))
					return PRCM_FAIL;
				test_rate = parent_rate / *div_array;
				if (test_rate <= target_rate) {
					*newdiv = *div_array;
					return PRCM_PASS;
				}
				++div_array;
			}
			return PRCM_FAIL;
		}
	}
#endif
	clk_id = clk & CLK_NO_MASK;
	if (clk_id > PRCM_NO_OF_CLKS)
		return PRCM_FAIL;

	for (index = 0; index < max_div; index++) {
		if (!div_arr[index]) {
			/* We have hit a NULL element which means
			 * dividers are exhausted */
			return PRCM_FAIL;
		}
		div = div_arr[(clk_id * max_div) + index];
		DPRINTK("Testing divider: %u\n", div);
		test_rate = parent_rate / div;
		if (test_rate <= target_rate) {
			*newdiv = div;
			DPRINTK("New div: %d\n", *newdiv);
			return PRCM_PASS;
		}
	}
	return PRCM_FAIL;	/*No acceptable divider */
}

/*============================================================================*/
/*============================== GET SOURCE ==================================*/
/*= This function will get the source or parent for a specified clock.       =*/
/*= The child clock is specified in the clk_id and the parent clock is       =*/
/*= specified in the parent_id parameter (which is passed by reference)      =*/
/*= Based on the clk_id parameter, and the register settings, the parent     =*/
/*= id is updated with the appropriate device ID                             =*/
/*============================================================================*/

int prcm_clk_get_source(u32 clk_id, u32 *parent_id)
{
	u32 valid, valid_val, bit_pos, domain, omap, id_type;
	u8 ret = PRCM_PASS, result = -1;
	volatile u32 *addr;

	id_type = get_id_type(clk_id);
	if (!(id_type & ID_CLK_SRC))
		return PRCM_FAIL;	/*Rate cannot be selected */

	omap = OMAP(clk_id);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	switch (clk_id) {
	case PRCM_48M_FCLK:
		addr = &CM_CLKSEL1_PLL;
		bit_pos = 0x3;
		if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
			*parent_id = PRCM_DPLL4_M2X2_CLK;
		else
			*parent_id = PRCM_SYS_ALT_CLK;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case PRCM_TVOUT:
		addr = &CM_CLKSEL1_PLL;
		bit_pos = 0x5;
		if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
			*parent_id = PRCM_DPLL4_M3X2_CLK;
		else
			*parent_id = PRCM_SYS_ALT_CLK;
		break;
#endif
	case PRCM_USIM:
		addr = &CM_CLKSEL_WKUP;
		bit_pos = 0x3;
		valid = 0x78;
		valid_val = (*addr & valid) >> bit_pos;
		if ((valid_val == 0x1) || (valid_val == 0x2))
			*parent_id = PRCM_SYS_CLK;
		else if ((valid_val >= 0x3) && (valid_val <= 0x6))
			*parent_id = PRCM_DPLL4_M2X2_CLK;
		else if ((valid_val >= 0x7) && (valid_val <= 0xA))
			*parent_id = PRCM_DPLL5_M2_CLK;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case PRCM_SGX_FCLK:
		addr = &CM_CLKSEL_SGX;
		bit_pos = 0x0;
		valid = 0x7;
		valid_val = (*addr & valid) >> bit_pos;
		if ((valid_val >= 0x0) && (valid_val <= 0x2))
			*parent_id = PRCM_DPLL3_M2_CLK;
		else
			*parent_id = PRCM_EXT_MCBSP_CLK;
		break;
#endif
	case PRCM_96M_CLK:
		addr = &CM_CLKSEL1_PLL;
		bit_pos = 0x6;
		valid_val = *addr & (1<<bit_pos);
		if (valid_val == 0)
			*parent_id = PRCM_DPLL4_M2X2_CLK;
		else
			*parent_id = PRCM_SYS_CLK;
		break;
	case PRCM_SYS_CLKOUT2:
		addr = &CM_CLKOUT_CTRL;
		bit_pos = 0x0;
		valid = 0x3;
		valid_val = *addr & (valid << bit_pos);
		if (valid_val == 0)
			*parent_id = PRCM_DPLL3_M2_CLK;
		else if (valid_val == 1)
			*parent_id = PRCM_SYS_CLK;
		else if (valid_val == 2)
			*parent_id = PRCM_DPLL4_M2X2_CLK;
#ifndef CONFIG_ARCH_OMAP3410
		else
			*parent_id = PRCM_TVOUT;
#endif
		break;
	case PRCM_MCBSP1:
	case PRCM_MCBSP2:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						   PRCM_ENABLE, PRCM_TRUE);
			}

			else {
				addr = &CONTROL_DEVCONF0;
				bit_pos = CLK_SRC_BIT_POS(clk_id);

				if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
					*parent_id = PRCM_DPLL4_M2X2_CLK;
				else
					*parent_id = PRCM_EXT_MCBSP_CLK;
			}
		} else
			return PRCM_FAIL;
		break;
	case PRCM_MCBSP3:
	case PRCM_MCBSP4:
	case PRCM_MCBSP5:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						PRCM_ENABLE, PRCM_TRUE);
			}

			else {
				addr = &CONTROL_DEVCONF1;
				bit_pos = CLK_SRC_BIT_POS(clk_id);

				if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
					*parent_id = PRCM_DPLL4_M2X2_CLK;
				else
					*parent_id = PRCM_EXT_MCBSP_CLK;
			}
		} else
			return PRCM_FAIL;
		break;
	case PRCM_GPT1:
	case PRCM_GPT2:
	case PRCM_GPT3:
	case PRCM_GPT4:
	case PRCM_GPT5:
	case PRCM_GPT6:
	case PRCM_GPT7:
	case PRCM_GPT8:
	case PRCM_GPT9:
	case PRCM_GPT10:
	case PRCM_GPT11:

		bit_pos = CLK_SRC_BIT_POS(clk_id);
		domain = DOMAIN_ID(clk_id);
		addr = get_addr(domain, REG_CLK_SRC);
		if (!addr)
			return PRCM_FAIL;

		if (((*addr & (1 << bit_pos)) >> bit_pos) == 0)
			*parent_id = PRCM_SYS_32K_CLK;
		else
			*parent_id = PRCM_SYS_CLK;

		break;

	default:
		printk(KERN_INFO "Invalid clock ID in prcm_clk_get_source\n");
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*============================== SET SOURCE ==================================*/
/*= This function will set the source or parent for a specified clock.       =*/
/*= The child clock is specified in the clk_id and the parent clock is       =*/
/*= specified in the parent_id parameter.                                    =*/
/*= Based on the clk_id parameter, the correct register is updated with the  =*/
/*= specified clock source.                                                  =*/
/*= The function always returns PRCM_PASS.                                   =*/
/*============================================================================*/

int prcm_clk_set_source(u32 clk_id, u32 parent_id)
{
	u32 valid, bit_pos, domain, bit_val, src_bit = 0, omap, id_type;
	u8 ret = PRCM_PASS, result = -1;

	volatile u32 *addr;

	id_type = get_id_type(clk_id);
	if (!(id_type & ID_CLK_SRC))
		return PRCM_FAIL;	/*Rate cannot be selected */

	omap = OMAP(clk_id);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	switch (clk_id) {
	case PRCM_48M_FCLK:
		/* Check the parent for this clock */
		if (parent_id == PRCM_DPLL4_M2X2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_ALT_CLK)
			src_bit = 1;
		else
			return PRCM_FAIL;
		addr = &CM_CLKSEL1_PLL;
		bit_pos = 0x3;
		break;
	case PRCM_USIM:
		addr = &CM_CLKSEL_WKUP;
		valid = *addr & 0xFFFFFF87;
		if (parent_id == PRCM_SYS_CLK) {
			valid |= (0x1 << 3);
			*addr = valid;
			}
		else if (parent_id == PRCM_DPLL4_M2X2_CLK) {
			valid |= (0x3 << 3);
			*addr = valid;
			}
		else if (parent_id == PRCM_DPLL5_M2_CLK) {
			valid |= (0x7 << 3);
			*addr = valid;
		}
		return PRCM_PASS;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case PRCM_SGX_FCLK:
		if (parent_id == PRCM_EXT_MCBSP_CLK)
			CM_CLKSEL_SGX = 0x3;
		else if (parent_id == PRCM_DPLL3_M2_CLK)
			CM_CLKSEL_SGX = 0x0; /*Default divider is 3 */
		return PRCM_PASS;
		break;
#endif
	case PRCM_96M_CLK:
		if (parent_id == PRCM_DPLL4_M2X2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_CLK)
			src_bit = 1;
		addr = &CM_CLKSEL1_PLL;
		bit_pos = 0x6;
		break;
#ifndef CONFIG_ARCH_OMAP3410
	case PRCM_TVOUT:
		/* Check the parent for this clock */
		if (parent_id == PRCM_DPLL4_M3X2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_ALT_CLK)
			src_bit = 1;
		else
			return PRCM_FAIL;
		addr = &CM_CLKSEL1_PLL;
		bit_pos = 0x5;
		break;
#endif
	case PRCM_SYS_CLKOUT2:
		/* Check the parent for this clock */
		if (parent_id == PRCM_DPLL3_M2_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_CLK)
			src_bit = 1;
		else if (parent_id == PRCM_DPLL4_M2X2_CLK)
			src_bit = 2;
#ifndef CONFIG_ARCH_OMAP3410
		else if (parent_id == PRCM_TVOUT)
			src_bit = 3;
#endif
		else
			return PRCM_FAIL;
		addr = &CM_CLKOUT_CTRL;
		bit_pos = 0x0;
		valid = 0x3;
		*addr = (*addr & ~valid) | src_bit;
		/* Returning from here because we have already
		 *updated the register */
		return PRCM_PASS;
		break;
	case PRCM_MCBSP1:
	case PRCM_MCBSP2:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						   PRCM_ENABLE, PRCM_TRUE);
			}
			bit_pos = CLK_SRC_BIT_POS(clk_id);
			addr = &CONTROL_DEVCONF0;

			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				src_bit = 0;
			else if (parent_id == PRCM_EXT_MCBSP_CLK)
				src_bit = 1;
			else
				return PRCM_FAIL;
		} else
			return ret;
		break;
	case PRCM_MCBSP3:
	case PRCM_MCBSP4:
	case PRCM_MCBSP5:
		ret = prcm_is_device_accessible(PRCM_OMAP_CTRL, &result);
		if (ret == PRCM_PASS) {
			if (result == PRCM_FALSE) {
				/*
				 * The device is not accessible.
				 * So enable the interface clock.
				 */
				prcm_clock_control(PRCM_OMAP_CTRL, ICLK,
						   PRCM_ENABLE, PRCM_TRUE);
			}
			bit_pos = CLK_SRC_BIT_POS(clk_id);
			addr = &CONTROL_DEVCONF1;

			if (parent_id == PRCM_DPLL4_M2X2_CLK)
				src_bit = 0;
			else if (parent_id == PRCM_EXT_MCBSP_CLK)
				src_bit = 1;
			else
				return PRCM_FAIL;
		} else
			return PRCM_FAIL;
		break;
	case PRCM_GPT1:
	case PRCM_GPT2:
	case PRCM_GPT3:
	case PRCM_GPT4:
	case PRCM_GPT5:
	case PRCM_GPT6:
	case PRCM_GPT7:
	case PRCM_GPT8:
	case PRCM_GPT9:
	case PRCM_GPT10:
	case PRCM_GPT11:
		/* Setting clock source for GP timers. */
		if (parent_id == PRCM_SYS_32K_CLK)
			src_bit = 0;
		else if (parent_id == PRCM_SYS_CLK)
			src_bit = 1;
		else
			return PRCM_FAIL;
		bit_pos = CLK_SRC_BIT_POS(clk_id);
		domain = DOMAIN_ID(clk_id);
		addr = get_addr(domain, REG_CLK_SRC);
		if (!addr)
			return PRCM_FAIL;
		break;
	default:
		printk(KERN_INFO "Invalid clock ID in prcm_clk_set_source\n");
		return PRCM_FAIL;
	}
	DPRINTK("Src bit: %d, bit_pos: %d\n", src_bit, bit_pos);
	bit_val = ((1 << bit_pos) & *addr) >> bit_pos;

	if (bit_val && !(src_bit))
		*addr &= (~(1 << bit_pos));
	else if (!(bit_val) && src_bit)
		*addr |= (1 << bit_pos);

	return PRCM_PASS;
}


