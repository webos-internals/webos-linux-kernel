/*
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

#include <linux/types.h>
#include <linux/kernel.h>
#ifdef CONFIG_CPU_FREQ
#include <linux/cpufreq.h>
#endif

#include <asm/system.h>
#include <asm/arch/prcm.h>
#include <asm/arch/clock.h>
#include <asm/arch/power_companion.h>
#include <asm/arch/pm.h>

#include "prcm-regs.h"
#include "prcm_opp.h"
#include "prcm_util.h"
#include "clock.h"

/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

//#define DEBUG_PRCM

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

static u8 mpu_iva2_vdd1_volts [PRCM_NO_VDD1_OPPS] = {
	/* Vsel corresponding to 0.975V (OPP1), 1.050V (OPP2),
				1.20V (OPP3), 1.27V (OPP4), 1.35 (OPP5) */
	0x1e, 0x24, 0x30, 0x36, 0x3C
};

static u8 core_l3_vdd2_volts [PRCM_NO_VDD2_OPPS] = { /* only 3 OPPs */
	/* Vsel corresponding to 0.975V (OPP1), 1.05V (OPP2), 1.15 (OPP3) */
	0x1e, 0x24, 0x2c
};


static struct dvfs_config omap3_vdd2_config[2] = {
	{
	/* SDRC CS0/CS1 values 83MHZ*/
	{{0x00025801, 0x629db4c6, 0x00012214},
	 {0x00025801, 0x629db4c6, 0x00012214} },
	/* GPMC CS0/1/2/3 values 83 MHZ*/
	/* CS0 values are for NOR */
	/* CS1 values are for NAND */
	/* CS2 values are for ONE-NAND */
	/* CS3 values are for FPGA */
	{{0x00101000, 0x00040401, 0x0E050E05, 0x010A1010, 0x100802C1},
	 {0x000A0A00, 0x000A0A00, 0x08010801, 0x01060A0A, 0x10080580},
	 {0x00080801, 0x00020201, 0x08020802, 0x01080808, 0x10030000},
	 {0x00101001, 0x00040402, 0x0E050E05, 0x020F1010, 0x0F0502C2} }
	},

	/* SDRC CS0/CS1 values 166MHZ*/
	{
	{{0x0004e201, 0x629db4c6, 0x00012214},
	 {0x0004e201, 0x629db4c6, 0x00012214} },
	/* GPMC CS0/1/2/3 values 166 MHZ*/
	{{0x001f1f00, 0x00080802, 0x1C091C09, 0x01131F1F, 0x1F0F03C2},
	 {0x00141400, 0x00141400, 0x0F010F01, 0x010C1414, 0x1F0F0A80},
	 {0x000F0F01, 0x00030301, 0x0F040F04, 0x010F1010, 0x1F060000},
	 {0x001F1F01, 0x00080803, 0x1D091D09, 0x041D1F1F, 0x1D0904C4} }
	},
};

static void omap3_configure_gpmc(struct dvfs_config *cfg)
{
	/* Set the GPMC paramaters */
	GPMC_CONFIG2_0 = cfg->gpmc_cfg[0].gpmc_config2;
	GPMC_CONFIG3_0 = cfg->gpmc_cfg[0].gpmc_config3;
	GPMC_CONFIG4_0 = cfg->gpmc_cfg[0].gpmc_config4;
	GPMC_CONFIG5_0 = cfg->gpmc_cfg[0].gpmc_config5;
	GPMC_CONFIG6_0 = cfg->gpmc_cfg[0].gpmc_config6;
	GPMC_CONFIG2_1 = cfg->gpmc_cfg[1].gpmc_config2;
	GPMC_CONFIG3_1 = cfg->gpmc_cfg[1].gpmc_config3;
	GPMC_CONFIG4_1 = cfg->gpmc_cfg[1].gpmc_config4;
	GPMC_CONFIG5_1 = cfg->gpmc_cfg[1].gpmc_config5;
	GPMC_CONFIG6_1 = cfg->gpmc_cfg[1].gpmc_config6;
	GPMC_CONFIG2_2 = cfg->gpmc_cfg[2].gpmc_config2;
	GPMC_CONFIG3_2 = cfg->gpmc_cfg[2].gpmc_config3;
	GPMC_CONFIG4_2 = cfg->gpmc_cfg[2].gpmc_config4;
	GPMC_CONFIG5_2 = cfg->gpmc_cfg[2].gpmc_config5;
	GPMC_CONFIG6_2 = cfg->gpmc_cfg[2].gpmc_config6;
	GPMC_CONFIG2_3 = cfg->gpmc_cfg[3].gpmc_config2;
	GPMC_CONFIG3_3 = cfg->gpmc_cfg[3].gpmc_config3;
	GPMC_CONFIG4_3 = cfg->gpmc_cfg[3].gpmc_config4;
	GPMC_CONFIG5_3 = cfg->gpmc_cfg[3].gpmc_config5;
	GPMC_CONFIG6_3 = cfg->gpmc_cfg[3].gpmc_config6;
	return;
}

static u32 current_vdd1_opp = 0;
static u32 current_vdd2_opp = 0;

static struct clk *p_vdd1_clk = NULL;
static struct clk *p_vdd2_clk = NULL;

u32 prcm_get_current_vdd1_opp(void)
{
	return current_vdd1_opp;
}

u32 prcm_get_current_vdd2_opp(void)
{
	return current_vdd2_opp;
}

u32 omap3_max_vdd1_opp(void)
{
	return 5;

	/* This function call is used in the bridgedriver.
	 *
	 * The code below is from a newer TI kernel drop and returns the max
	 * OPP based on silicon revision and part binning. We are not using
	 * this code right now but are keeping it here for future reference.
	 *
	 * For our kernel we are always returning 5 as max OPP.
	 */
#if 0
	u32 sku_id;

	if (is_sil_rev_less_than(OMAP3430_REV_ES3_1) ||
		is_sil_rev_equal_to(OMAP3430_REV_ES3_1))
		return 5;

	sku_id = omap_readl(CONTROL_PRODUCTION_ID_SKU_ID) & 0xf;
	if (sku_id == 0)
		return 5;
	else if (sku_id == 1)
		return 6;
	else if (sku_id >= 2)
		return 7;
	return 5;
#endif
}
EXPORT_SYMBOL(omap3_max_vdd1_opp);


/******************************************************************************
 *
 * Table of DPLL register definitions.
 *
 ******************************************************************************/

/* Tables having M,N,M2 and FreqSel values for different sys_clk speeds & OPPs*/
/* The tables are organized as follows: */
/* Rows : 1 - 12M, 2 - 13M, 3 - 19.2M, 4 - 26M, 5 - 38.4M */
/* Columns : 1 - OPP1, 2 - OPP2, 3 - OPP3, 4 - OPP4  5 - OPP5 */

/* MPU parameters */
static struct dpll_param mpu_dpll_param[5][PRCM_NO_VDD1_OPPS] = {
	/* 12M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x0FA, 0x05, 0x07, 0x04}, {0x0FA, 0x05, 0x07, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x0FA, 0x05, 0x07, 0x01}, {0x113, 0x05, 0x07, 0x01},
	/* OPP5 (600 Mhz) */
	{0x12C, 0x05, 0x07, 0x01} },
	/* 13M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x1F4, 0x0C, 0x03, 0x04}, {0x1F4, 0x0C, 0x03, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x1F4, 0x0C, 0x03, 0x01}, {0x226, 0x0C, 0x03, 0x01},
	/* OPP5 (600 Mhz) */
	{0x258, 0x0C, 0x03, 0x01} },
	/* 19.2M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x271, 0x17, 0x03, 0x04}, {0x271, 0x17, 0x03, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x271, 0x17, 0x03, 0x01}, {0x191, 0x0D, 0x05, 0x01},
	/* OPP5 (600 Mhz) */
	{0x177, 0x0B, 0x06, 0x01} },
	/* 26M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x0FA, 0x0C, 0x07, 0x04}, {0x0FA, 0x0C, 0x07, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x0FA, 0x0C, 0x07, 0x01}, {0x113, 0x0C, 0x07, 0x01},
	/* OPP5 (600 Mhz) */
	{0x12C, 0x0C, 0x07, 0x01} },
	/* 38.4M values */
	/* OPP1(125 Mhz) and OPP2(250 Mhz)*/
	{{0x271, 0x2F, 0x03, 0x04}, {0x271, 0x2F, 0x03, 0x02},
	/* OPP3(500 Mhz) and OPP4(550 Mhz)*/
	{0x271, 0x2F, 0x03, 0x01}, {0x1BC, 0x1E, 0x04, 0x01},
	/* OPP5 (600 Mhz) */
	{0x177, 0x17, 0x06, 0x01} },
};

/* IVA parameters */
static struct dpll_param iva_dpll_param[5][PRCM_NO_VDD1_OPPS] = {
	/* 12M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0B4, 0x05, 0x07, 0x04}, {0x0B4, 0x05, 0x07, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0B4, 0x05, 0x07, 0x01}, {0x0C6, 0x05, 0x07, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x0D7, 0x05, 0x07, 0x01} },
	/* 13M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x168, 0x0C, 0x03, 0x04}, {0x168, 0x0C, 0x03, 0x02},
	/* OPP3(360 Mhz) and OPP4(400 Mhz)*/
	 {0x168, 0x0C, 0x03, 0x01}, {0x190, 0x0C, 0x03, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x1AE, 0x0C, 0x03, 0x01} },
	/* 19.2M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0E1, 0x0B, 0x06, 0x04}, {0x0E1, 0x0B, 0x06, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0E1, 0x0B, 0x06, 0x01}, {0x14A, 0x0F, 0x04, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x203, 0x16, 0x03, 0x01} },
	/* 26M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0B4, 0x0C, 0x07, 0x04}, {0x0B4, 0x0C, 0x07, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0B4, 0x0C, 0x07, 0x01}, {0x0C6, 0x0C, 0x07, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x0D7, 0x0C, 0x07, 0x01} },
	/* 38.4M values */
	/* OPP1(90 Mhz) and OPP2(180 Mhz)*/
	{{0x0E1, 0x17, 0x06, 0x04}, {0x0E1, 0x17, 0x06, 0x02},
	/* OPP3(360 Mhz) and OPP4(396 Mhz)*/
	 {0x0E1, 0x17, 0x06, 0x01}, {0x14A, 0x1F, 0x04, 0x01},
	/* OPP5 (430 Mhz) */
	 {0x23B, 0x32, 0x01, 0x01} },
};

/* CORE parameters */
static struct dpll_param core_dpll_param[5][PRCM_NO_VDD2_OPPS] = {
	/* 12M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x0A6, 0x05, 0x07, 0x02}, {0x0A6, 0x05, 0x07, 0x01} },
	/* 13M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x14C, 0x0C, 0x03, 0x02}, {0x14C, 0x0C, 0x03, 0x01} },
	/* 19.2M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x19F, 0x17, 0x03, 0x02}, {0x19F, 0x17, 0x03, 0x01} },
	/* 26M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x0A6, 0x0C, 0x07, 0x02}, {0x0A6, 0x0C, 0x07, 0x01} },
	/* 38.4M values */
	/* OPP1(83 Mhz) and OPP2(166 Mhz) */
	{{0, 0, 0, 0}, {0x19F, 0x2F, 0x03, 0x02}, {0x19F, 0x2F, 0x03, 0x01} },
};

static struct dpll_registers dpll_reg[NO_OF_DPLL] = {
	{ { /* DPLL1_MPU */
		{0xFFFFFF0F,	&CM_CLKEN_PLL_MPU},
		{0x7,		&CM_AUTOIDLE_PLL_MPU},
		{0xFFF800FF,	&CM_CLKSEL1_PLL_MPU},
		{0,		&CM_IDLEST_PLL_MPU},
		{0xFFFFFFE0,	&CM_CLKSEL2_PLL_MPU},
		{0xFFFFFFE0,	&CM_CLKSEL2_PLL_MPU},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
	} },
	{ { /* DPLL2_IVA2 */
		{0xFFFFFF0F,	&CM_CLKEN_PLL_IVA2},
		{0x7,		&CM_AUTOIDLE_PLL_IVA2},
		{0xFFF800FF,	&CM_CLKSEL1_PLL_IVA2},
		{0,		&CM_IDLEST_PLL_IVA2},
		{0xFFFFFFE0,	&CM_CLKSEL2_PLL_IVA2},
		{0xFFFFFFE0,	&CM_CLKSEL2_PLL_IVA2},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
	} },
	{ { /* DPLL3_CORE */
		{0xFFFFFF0F,	&CM_CLKEN_PLL},
		{0x7,		&CM_AUTOIDLE_PLL},
		{0xF800FFFF,	&CM_CLKSEL1_PLL},
		{0,		&CM_IDLEST_CKGEN},
		{0x07FFFFFF,	&CM_CLKSEL1_PLL},
		{0x07FFFFFF,	&CM_CLKSEL1_PLL},
		{0xFFE0FFFF,	&CM_CLKSEL1_EMU},
		{0,		0},
		{0,		0},
		{0,		0},
	} },
	{ { /* DPLL4_PER */
		{0xFF0FFFFF,	&CM_CLKEN_PLL},
		{0x38,		&CM_AUTOIDLE_PLL},
		{0xFFF800FF,	&CM_CLKSEL2_PLL},
		{0,		&CM_IDLEST_CKGEN},
		{0xFFFFFFE0,	&CM_CLKSEL3_PLL},
		{0xFFFFFFE0,	&CM_CLKSEL3_PLL},
		{0xFFFFE0FF,	&CM_CLKSEL_DSS},
		{0xFFFFFFE0,	&CM_CLKSEL_DSS},
		{0xFFFFFFE0,	&CM_CLKSEL_CAM},
		{0xE0FFFFFF,	&CM_CLKSEL1_EMU},
	} },
	{ { /* DPLL5_PER2 */
		{0xFFFFFF0F,	&CM_CLKEN2_PLL},
		{0x7,		&CM_AUTOIDLE2_PLL},
		{0xFFF800FF,	&CM_CLKSEL4_PLL},
		{0,		&CM_IDLEST2_CKGEN},
		{0xFFFFFFE0,	&CM_CLKSEL5_PLL},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
		{0,		0},
	} }
};

static u32 dpll_mx_shift[NO_OF_DPLL][NO_DPLL_DIV] = {
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
	{0x1B, 0x1B, 0x10, 0x0, 0x0, 0x0},
	{0x0, 0x0, 0x8, 0x0, 0x0, 0x18},
	{0x0, 0x0, 0x0, 0x0, 0x0, 0x0},
};

static struct dpll_param usb_dpll_param[5] = {
	/* 12M values */
	{0x3C, 0x05, 0x07, 0x01},
	/* 13M values */
	{0x78, 0x0C, 0x03, 0x01},
	/* 19.2M values  */
	{0x4B, 0x0B, 0x06, 0x01},
	/* 26M values */
	{0x3C, 0x0C, 0x07, 0x01},
	/* 38.4M values */
	{0x4B, 0x17, 0x06, 0x01},
};

/******************************************************************************
 *
 * Globals.
 *
 ******************************************************************************/

static struct constraint_id cnstr_id1 = {
	.type = RES_FREQ_CO,
	.data = (void *)"vdd1_opp",
};

static struct constraint_id cnstr_id2 = {
	.type = RES_FREQ_CO,
	.data = (void *)"vdd2_opp",
};

/******************************************************************************
 *
 * PRCM API for voltage scaling.
 * 
 ******************************************************************************/

static int prcm_do_voltage_scaling(u32 target_opp_id, u32 current_opp_id)
{
	u32 id_type, vdd;
	int curr_opp_no, target_opp_no;
	u8 vsel = 0;

	if (target_opp_id == current_opp_id)
		return PRCM_PASS;

	vdd = get_vdd(target_opp_id);

	/* Only VDD1 and VDD2 are allowed to be scaled by software */
	/* Presently only VDD1 code is written */
	if ((vdd != PRCM_VDD1) && (vdd != PRCM_VDD2)) {
		DPRINTK("Not VDD1 or VDD2\n");
		return PRCM_FAIL;
	}

	if (vdd != get_vdd(current_opp_id)) {
		DPRINTK("VDD value for target and current not matching\n");
		return PRCM_FAIL;
	}

	id_type = get_other_id_type(target_opp_id);
	if (!(id_type & ID_OPP)) {
		DPRINTK("ID type of target is not a OPP ID type\n");
		return PRCM_FAIL;
	}

	id_type = get_other_id_type(current_opp_id);
	if (!(id_type & ID_OPP)) {
		DPRINTK("ID type of current is not a OPP ID type\n");
		return PRCM_FAIL;
	}

	curr_opp_no   = current_opp_id & OPP_NO_MASK;
	target_opp_no = target_opp_id  & OPP_NO_MASK;

	if (curr_opp_no == target_opp_no) {
		DPRINTK("Nothing to change, both current and target"
						"OPPs are same\n");
		return PRCM_PASS;
	}

	if (vdd == PRCM_VDD1)
		vsel = mpu_iva2_vdd1_volts[target_opp_no - 1];

	else if (vdd == PRCM_VDD2)
		vsel = core_l3_vdd2_volts[target_opp_no - 1];

#if defined(CONFIG_OMAP_VOLT_VSEL)
	if (set_voltage_level(vdd, vsel)) {
		printk(KERN_ERR "Unable to set the voltage level\n");
		return PRCM_FAIL;
	}

#elif defined(CONFIG_OMAP_VOLT_SR_BYPASS)
	if (sr_voltagescale_vcbypass(target_opp_id, vsel)) {
		printk(KERN_ERR "Unable to set the voltage level\n");
		return PRCM_FAIL;
	}
#endif /* #ifdef  CONFIG_OMAP_VOLT_VSEL */

	return PRCM_PASS;
}


#ifdef CONFIG_ENABLE_VOLTSCALE_IN_SUSPEND
void prcm_scale_prepare(void)
{
	u32 valid_rate;


	/* Scale to lowest OPP for VDD1 before executing suspend */
	if (current_vdd1_opp != PRCM_VDD1_OPP1) {
		valid_rate = clk_round_rate(p_vdd1_clk, S125M);
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
		prcm_do_voltage_scaling(PRCM_VDD1_OPP1, current_vdd1_opp);
	}
	if (current_vdd2_opp != PRCM_VDD2_OPP2) {
		valid_rate = clk_round_rate(p_vdd2_clk, S83M);
		p_vdd1_clk->set_rate(p_vdd2_clk, valid_rate);
		prcm_do_voltage_scaling(PRCM_VDD2_OPP2, current_vdd2_opp);
	}
}


void prcm_scale_finish(void)
{
	u32 valid_rate = 0;

	/* Scale back to previous OPP for VDD2*/
	if (current_vdd2_opp != PRCM_VDD2_OPP2) {
		prcm_do_voltage_scaling(PRCM_VDD2_OPP3, PRCM_VDD2_OPP2);
		valid_rate = clk_round_rate(p_vdd2_clk, S166M);
		p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
	}

	prcm_do_voltage_scaling(current_vdd1_opp, PRCM_VDD1_OPP1);
	if (current_vdd1_opp != PRCM_VDD1_OPP1) {
		switch (current_vdd1_opp) {
		case PRCM_VDD1_OPP2:
			valid_rate = clk_round_rate(p_vdd1_clk, S250M);
			break;
		case PRCM_VDD1_OPP3:
			valid_rate = clk_round_rate(p_vdd1_clk, S500M);
			break;
		case PRCM_VDD1_OPP4:
			valid_rate = clk_round_rate(p_vdd1_clk, S550M);
			break;
		case PRCM_VDD1_OPP5:
			valid_rate = clk_round_rate(p_vdd1_clk, S600M);
			break;
		}
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
	}
}
#endif

static struct vdd1_arm_dsp_freq_d {
	unsigned int freq_mpu;
	unsigned int freq_dsp;
	unsigned int constraint;
	unsigned int opp;
} vdd1_arm_dsp_freq[MAX_VDD1_OPP] = {
	{125,  90, CO_VDD1_OPP1, PRCM_VDD1_OPP1},
	{250, 180, CO_VDD1_OPP2, PRCM_VDD1_OPP2},
	{500, 360, CO_VDD1_OPP3, PRCM_VDD1_OPP3},
	{550, 396, CO_VDD1_OPP4, PRCM_VDD1_OPP4},
	{600, 430, CO_VDD1_OPP5, PRCM_VDD1_OPP5},
};
static struct vdd2_core_freq_d {
	unsigned int freq;
	unsigned int constraint;
	unsigned int opp;
} vdd2_core_freq[MAX_VDD2_OPP] = {
	{0, CO_VDD2_OPP1, PRCM_VDD2_OPP1},
	{83, CO_VDD2_OPP2, PRCM_VDD2_OPP2},
	{166, CO_VDD2_OPP3, PRCM_VDD2_OPP3},
};

static unsigned int rnd_rate_vdd1[5] = {
	S125M, S250M, S500M, S550M, S600M
};
static unsigned int rnd_rate_vdd2[3] = {
	0, S83M, S166M
};

/* To set the opp for vdd1 */
void vdd1_opp_setting(u32 target_opp_no)
{
	u32 valid_rate;
	unsigned int cur_opp_no, target_vdd1_opp;

	target_vdd1_opp = vdd1_arm_dsp_freq[target_opp_no-1].opp;
	cur_opp_no = get_opp_no(current_vdd1_opp);

	if (p_vdd1_clk == NULL)
		p_vdd1_clk = clk_get(NULL, "virt_vdd1_prcm_set");

	if (!p_vdd1_clk) {
		printk(KERN_ERR "OMAP: Could not get resource for clock "
				"virt_vdd1_prcm_set\n");
		return;
	}

	if (cur_opp_no < target_opp_no) {
		prcm_do_voltage_scaling(target_vdd1_opp, current_vdd1_opp);
		valid_rate = clk_round_rate(p_vdd1_clk,
					    rnd_rate_vdd1[target_opp_no-1]);
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
	} else {
		valid_rate = clk_round_rate(p_vdd1_clk,
					    rnd_rate_vdd1[target_opp_no-1]);
		p_vdd1_clk->set_rate(p_vdd1_clk, valid_rate);
		prcm_do_voltage_scaling(target_vdd1_opp, current_vdd1_opp);
	}

	current_vdd1_opp = target_vdd1_opp;
}

/* To set the opp value for vdd2 */
void vdd2_opp_setting(u32 target_opp_no)
{
	u32 valid_rate;
	unsigned int cur_opp_no, target_vdd2_opp;

	target_vdd2_opp = vdd2_core_freq[target_opp_no-1].opp;
	cur_opp_no = get_opp_no(current_vdd2_opp);

	if (p_vdd2_clk == NULL)
		p_vdd2_clk = clk_get(NULL, "virt_vdd2_prcm_set");

	if (cur_opp_no < target_opp_no) {
		prcm_do_voltage_scaling(target_vdd2_opp, current_vdd2_opp);
		valid_rate = clk_round_rate(p_vdd2_clk,
			rnd_rate_vdd2[target_opp_no-1]);
		p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
	} else {
		valid_rate = clk_round_rate(p_vdd2_clk,
			rnd_rate_vdd2[target_opp_no-1]);
		p_vdd2_clk->set_rate(p_vdd2_clk, valid_rate);
		prcm_do_voltage_scaling(target_vdd2_opp, current_vdd2_opp);
	}

	current_vdd2_opp = target_vdd2_opp;
}

/******************************************************************************
 *
 * Accessor functions for OPP frequency table.
 *
 ******************************************************************************/

unsigned short get_vdd1_arm_constraint_for_freq(unsigned int freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vdd1_arm_dsp_freq); i++) {
		if (vdd1_arm_dsp_freq[i].freq_mpu >= freq) {
			return (unsigned short) vdd1_arm_dsp_freq[i].constraint;
		}
	}

	/* no match */
	return 0;
}

unsigned short get_vdd1_dsp_constraint_for_freq(unsigned int freq)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vdd1_arm_dsp_freq); i++) {
		if (vdd1_arm_dsp_freq[i].freq_dsp >= freq) {
			return (unsigned short) vdd1_arm_dsp_freq[i].constraint;
		}
	}

	/* no match */
	return 0;
}

unsigned int get_arm_freq_for_opp(unsigned int opp)
{
	/* NOTE: opp index is 1 based */
	if (!opp || opp > ARRAY_SIZE(vdd1_arm_dsp_freq)) {
		printk(KERN_ERR
			"MPU frequencey request for illegal OPP %d\n", opp);
		return 0;
	}
	return vdd1_arm_dsp_freq[opp -1].freq_mpu * 1000;
}

unsigned int get_dsp_freq_for_opp(unsigned int opp)
{
	/* NOTE: opp index is 1 based */
	if (!opp || opp > ARRAY_SIZE(vdd1_arm_dsp_freq)) {
		printk(KERN_ERR
			"ARM frequencey request for illegal OPP %d\n", opp);
		return 0;
	}
	return vdd1_arm_dsp_freq[opp -1].freq_dsp * 1000;
}

unsigned int get_opp_for_index(unsigned int idx)
{
	if (idx >= ARRAY_SIZE(vdd1_arm_dsp_freq)) {
		printk(KERN_ERR
			"OPP request for illegal index %d\n", idx);
		return 0;
	}
	return vdd1_arm_dsp_freq[idx].opp;
}

#if defined(CONFIG_PM) && defined(CONFIG_SYSFS)
/******************************************************************************
 *
 * SYSFS interface for VDD1 and VDD2
 *
 ******************************************************************************/

static struct constraint_handle *vdd1_opp_co = NULL;
static struct constraint_handle *vdd2_opp_co = NULL;

/*
 * VDD 1
 */
static ssize_t omap_pm_vdd1_opp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%x\n",
			(unsigned int) get_opp_no(current_vdd1_opp));
}

static ssize_t omap_pm_vdd1_opp_store(struct kset *subsys,
						const char *buf, size_t n)
{
	u32 target_opp_no;

	sscanf(buf, "%u", &target_opp_no);

	/* Check target OPP for valid range. */
	if ((target_opp_no < 1) || (target_opp_no > 5)) {
		printk(KERN_ERR "opp_store for vdd1 ES 2.0:Invalid value\n");
		return -EINVAL;
	}

	/* If current and target OPP are the same, there is nothing to do. */
	if (target_opp_no == get_opp_no(current_vdd1_opp)) {
		DPRINTK("Target and current opp values are same\n");
		return n;
	}

	if (target_opp_no == 1) {
		/* If target OPP is == 1 then we remove the constraint and
		 * release the constraint handle.
		 */
		if (vdd1_opp_co != NULL) {
			constraint_remove(vdd1_opp_co);
			constraint_put(vdd1_opp_co);
			vdd1_opp_co = NULL;
		}
	} else {
		/* If an OPP != 1 has been requested, we get the constraint
		 * handle and set the constraint.
		 */
		if (vdd1_opp_co == NULL)
			vdd1_opp_co = constraint_get("pm_fwk", &cnstr_id1);
		constraint_set(vdd1_opp_co, target_opp_no);
	}

	return n;
}

static struct subsys_attribute vdd1_opp = {
	.attr = {
		.name = __stringify(vdd1_opp_value),
		.mode = 0644,
	},
	.show = omap_pm_vdd1_opp_show,
	.store = omap_pm_vdd1_opp_store,
};

/*
 * VDD 2
 */
static ssize_t omap_pm_vdd2_opp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%x\n",
			(unsigned int) get_opp_no(current_vdd2_opp));
}

static ssize_t omap_pm_vdd2_opp_store(struct kset *subsys,
				      const char *buf, size_t n)
{
	u32 target_opp_no;

	sscanf(buf, "%u", &target_opp_no);
	if ((target_opp_no < 1) || (target_opp_no > 3)) {
		printk(KERN_ERR "opp_store for vdd2 ES 2.0:Invalid value\n");
		return -EINVAL;
	}
	if (target_opp_no == get_opp_no(current_vdd2_opp)) {
		DPRINTK("Target and current opp values are same\n");
	} else {
		if (target_opp_no != 1) {
			if (vdd2_opp_co == NULL)
				vdd2_opp_co = constraint_get("pm_fwk",
								&cnstr_id2);
			constraint_set(vdd2_opp_co, target_opp_no);
		} else {
			if (vdd2_opp_co != NULL) {
				constraint_remove(vdd2_opp_co);
				constraint_put(vdd2_opp_co);
				vdd2_opp_co = NULL;
			}
		}
	}

	return n;
}

static struct subsys_attribute vdd2_opp = {
	.attr = {
		.name = __stringify(vdd2_opp_value),
		.mode = 0644,
	},
	.show = omap_pm_vdd2_opp_show,
	.store = omap_pm_vdd2_opp_store,
};
#endif	/* CONFIG_PM && CONFIG_SYSFS */

/*============================================================================*/
/*======================== get dpll m and n vals =============================*/
/*============================================================================*/
/*= this function returns the mult and div values for the specified dpll     =*/
/*============================================================================*/

static int get_dpll_m_n(u32 dpll_id, u32 *mult, u32 *div)
{
	u32 valid;
	volatile u32 *addr;

	addr = get_addr_pll(dpll_id, REG_CLKSEL1_PLL);
	if (!addr)
		return PRCM_FAIL;

	valid = get_val_bits_pll(dpll_id, REG_CLKSEL1_PLL);
	if (dpll_id == DPLL3_CORE) {
		*mult = (*addr & ~valid) >> 16;
		*div = (*addr & ~DPLL3_N_MASK) >> 8;
	} else {
		*mult = (*addr & ~valid) >> 8;
		*div = (*addr & ~DPLL_N_MASK);
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== GET DPLL MX DIV VAL ===============================*/
/*============================================================================*/
/*= This function returns the MX divider value(e.g M2,M3,M4,M5,M6            =*/
/*= for the specified dpll.                                                  =*/
/*============================================================================*/

static int get_dpll_mx(u32 dpll_id, u32 dpll_div, u32 dpll_mxsft)
{
	u32 valid;
	volatile u32 *addr;

	addr = get_addr_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);
	if (!addr)
		DPRINTK(KERN_INFO "Address not valid\n");

	valid = get_val_bits_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);
	return (*addr & ~valid) >> dpll_mxsft;
}

/*============================================================================*/
/*======================== CHECK IF DPLL IS LOCKED============================*/
/*============================================================================*/
/*= The function checks whether a particular DPLL is locked or not           =*/
/*= It returns PRCM_TRUE if it is locked and returns PRCM_FALSE if it is not =*/
/*============================================================================*/

static int is_dpll_locked(u32 dpll_id, int *result)
{
	volatile u32 *addr;
	u32 dpll_enbit_mask, dpll_idlest_lock_bit;

	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);
	if (!addr)
		return PRCM_FAIL;

	dpll_enbit_mask = get_dpll_enbitmask(dpll_id);
	dpll_idlest_lock_bit = get_idlest_lock_bit(dpll_id);

	if ((*addr & (~dpll_enbit_mask)) == (~dpll_enbit_mask))
		*result = PRCM_TRUE;
	else
		*result = PRCM_FALSE;
	return PRCM_PASS;
}

/******************************************************************************
 *
 * PUBLIC API
 *
 ******************************************************************************/

/*============================================================================*/
/*======================== ENABLE DPLL =======================================*/
/*============================================================================*/
/*= The function enables the specified DPLL and then wait for the DPLL to    =*/
/*= lock before returning.The function modifies the CM_CLKEN_PLL_<DPLL>      =*/
/*= setting. The function returns PRCM_FAIL if the dpllid passed is invalid  =*/
/*============================================================================*/

int prcm_enable_dpll(u32 dpll_id)
{
	u32 dpll_idlest_lock_bit, dpll_enbit_mask;
	volatile u32 *addr, *addr_auto;
	u32 dpll_autoidle;
	int ret, enabled;
	int loop_rem;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	/* Currently, this API does not allow locking of core DPLL */
	/* Locking of core DPLL needs to be done without access to SDRAM */
	/* This can be done safely if execution is done from SRAM */
	if (dpll_id == DPLL3_CORE)
		return PRCM_FAIL;

	/* Store the DPLL autoidle */
	addr_auto = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	dpll_autoidle = *addr_auto;
	*addr_auto = 0x0;

	ret = is_dpll_locked(dpll_id, &enabled);
	if (ret != PRCM_PASS) {
		*addr_auto = dpll_autoidle;
		return ret;
	}
	if (enabled == PRCM_TRUE) {
		DPRINTK("DPLL%d already enabled\n", dpll_id);
		*addr_auto = dpll_autoidle;
		return PRCM_PASS;
	}

	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);

	if (!addr) {
		*addr_auto = dpll_autoidle;
		return PRCM_FAIL;
	}

	dpll_enbit_mask = get_dpll_enbitmask(dpll_id);
	dpll_idlest_lock_bit = get_idlest_lock_bit(dpll_id);

	*addr |= ~dpll_enbit_mask;	/* enable DPLL in lock mode */

	/* wait for DPLL to lock */
	loop_rem = WAIT_UNTIL((*get_addr_pll(dpll_id, REG_IDLEST_PLL) &
						dpll_idlest_lock_bit), 5000);

	/* Restore the autoidle for the DPLL back */
	*addr_auto = dpll_autoidle;

	if (loop_rem < 0) {
		printk(KERN_INFO "Timeout waiting for dpll to lock, dpll_id %d\n",
				dpll_id);
		return PRCM_FAIL;
	}

	return PRCM_PASS;
}

/*============================================================================*/
/*======================== PUT DPLL IN BYPASS ================================*/
/*============================================================================*/
/*= This function will put the specified DPLL in various Bypass modes as     =*/
/*= specified by bypass_mode. The function waits for the DPLL to go into     =*/
/*= the given bypass_mode before returning                                   =*/
/*============================================================================*/

int prcm_put_dpll_in_bypass(u32 dpll_id, u32 bypass_mode)
{
	u32 new_val;
	volatile u32 *addr, *addr_auto;
	u32 dpll_autoidle;
	int ret = PRCM_FAIL;

	if (dpll_id > NO_OF_DPLL)
		return ret;

	/* Currently, the API does not allow putting CORE dpll in bypass mode
	 * To safely put dpll in bypass mode, it is better to execute code
	 * from sram so that there is no access to sdram */
	if (dpll_id == DPLL3_CORE)
		return ret;

	DPRINTK("dpllid:%d\n", dpll_id);

	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);
	if (!addr)
		return ret;

	/* This is needed if the condition in while loop returns true the */
							/*very first time*/
	ret = PRCM_PASS;

	/* Store the DPLL autoidle */
	addr_auto = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	dpll_autoidle = *addr_auto;
	*addr_auto = 0x0;

	if (dpll_id == DPLL1_MPU) {
		new_val = (*addr & DPLL_ENBIT_MASK) | LOW_POWER_BYPASS;
		*addr = new_val;
		ret = WAIT_WHILE((*get_addr_pll(dpll_id, REG_IDLEST_PLL) & 0x1),
				  1000);

	} else if (dpll_id == DPLL4_PER) {
		new_val = (*addr & DPLL4_ENBIT_MASK) | (LOW_POWER_STOP << 16);
		*addr = new_val;
		ret = WAIT_WHILE((*get_addr_pll(dpll_id, REG_IDLEST_PLL) & 0x2),
				1000);
	} else {
		if ((dpll_id == DPLL5_PER2) && (bypass_mode != LOW_POWER_STOP))
			return ret;
		new_val = (*addr & DPLL_ENBIT_MASK) | bypass_mode;
		*addr = new_val;
		ret = WAIT_WHILE((*get_addr_pll(dpll_id, REG_IDLEST_PLL) & 0x1),
				1000);
	}
	if (ret < 0)
		printk(KERN_INFO "Timeout in prcm_put_dpll_in_bypass, "
			"dpll_id %d, desired mode %d\n", dpll_id, bypass_mode);

	/* Restore the autoidle for the DPLL back */
	*addr_auto = dpll_autoidle;
	return (ret < 0) ? PRCM_FAIL : PRCM_PASS;
}

/*============================================================================*/
/*======================== DPLL AUTO CONTROL =================================*/
/*============================================================================*/
/*= This function will enable/disable the auto control feature of the        =*/
/*= specified dpll.                                                          =*/
/*= The parameter passed as input should be PRCM_ENABLE or PRCM_DISABLE      =*/
/*============================================================================*/

int prcm_dpll_clock_auto_control(u32 dpll_id, u32 control)
{
	u32 valid;
	volatile u32 *addr;
	u32 setting;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	addr = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	if (!addr)
		return PRCM_FAIL;

	valid = get_val_bits_pll(dpll_id, REG_AUTOIDLE_PLL);

	if (dpll_id == DPLL4_PER)
		setting = 1 << 3;
	else
		setting = 1;

	switch (control) {
	case PRCM_ENABLE:
		*addr |= setting;
		break;
	case PRCM_DISABLE:
		*addr &= (~setting);
		break;
	default:
		return PRCM_FAIL;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/* This function returns the mn output of a dpll                              */
/* (without m2,m3,m4,m5,m6 dividers                                           */
/* In other words, it returns CLKOUT = (Fref * m)/(n+1)	in case dpll is locked*/
/* And bypass clock if it is in bypass mode.                                  */
/* The frequency value returned in mn_output is in Khz                        */
/*============================================================================*/

int prcm_get_dpll_mn_output(u32 dpll, u32 *mn_output)
{
	u32 dpll_idlest_lock_bit, clksel1_pll, dpll_id;
	u32 sys_clkspeed, core_clkspeed;
	int bypassclk_divider;
	volatile u32 *addr;

	dpll_id = (dpll >> DPLL_NO_POS) & DPLL_NO_MASK;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	dpll_idlest_lock_bit = get_idlest_lock_bit(dpll_id);

	DPRINTK("dpll: %d\n", dpll_id);

	addr = get_addr_pll(dpll_id, REG_IDLEST_PLL);
	/* Get the sys clock speed */
	sys_clkspeed = prcm_get_system_clock_speed();
	DPRINTK("SYSCLKSPEED = %d\n", sys_clkspeed);

	if (*addr & dpll_idlest_lock_bit) {
		u32 mult = 0;
		u32 div  = 0;

		/* dpll locked */
		get_dpll_m_n(dpll_id, &mult, &div);
		DPRINTK("DPLL not in bypass,M=%d, N=%d\n", mult, div);
		*mn_output = ((sys_clkspeed * mult) / (div + 1));
	} else {
		/* dpll is in bypass mode */
		DPRINTK("DPLL in bypass\n");
		if ((dpll_id == DPLL3_CORE) ||
		    (dpll_id == DPLL5_PER2) ||
		    (dpll_id == DPLL4_PER))
			*mn_output = sys_clkspeed;
		else {		/* DPLL1 and DPLL2
				 * Check if DPLL3 is in bypass */
			prcm_get_dpll_rate(PRCM_DPLL3_M2X2_CLK, &core_clkspeed);
			DPRINTK("Core clk speed: %d\n", core_clkspeed);
			if (dpll_id == DPLL1_MPU)
				clksel1_pll = CM_CLKSEL1_PLL_MPU;
			else
				clksel1_pll = CM_CLKSEL1_PLL_IVA2;
			bypassclk_divider = (clksel1_pll >> 19) & 0x3;
			*mn_output = core_clkspeed / bypassclk_divider;
		}
	}
	DPRINTK("DPLL_MN_OUTPUT: %d\n", *mn_output);
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== GET DPLL RATE =====================================*/
/*============================================================================*/
/*= This function returns the rate for the specified dpll in KHz. It checks  =*/
/*= if the DPLL is in locked mode or unlocked mode and accordingly calculates=*/
/*= the output.                                                              =*/
/*============================================================================*/

int prcm_get_dpll_rate(u32 dpll, u32 *output)
{
	u32 dpll_id, dpll_div, dpll_mxsft, id_type;
	u32 mx, omap;
	u32 mn_output;

	id_type = get_id_type(dpll);
	if (!(id_type & ID_DPLL_OP))
		return PRCM_FAIL;	/*Not dpll op */

	omap = OMAP(dpll);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	dpll_id = (dpll >> DPLL_NO_POS) & DPLL_NO_MASK;
	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	dpll_div = (dpll >> DPLL_DIV_POS) & DPLL_DIV_MASK;
	dpll_mxsft = dpll_mx_shift[dpll_id-1][dpll_div];
	DPRINTK("dpllmxsft:%d\n", dpll_mxsft);

	DPRINTK("Output required:%d\n", dpll_div);
	mx = get_dpll_mx(dpll_id, dpll_div, dpll_mxsft);
	DPRINTK("MX: %d\n", mx);

	prcm_get_dpll_mn_output(dpll, &mn_output);

	/* Except for DPLL_M2 all clocks are (mn_output*2)/mx */
	if (dpll_div == DPLL_M2)
		*output = (mn_output) / mx;
	else
		*output = (2 * mn_output) / mx;
	DPRINTK("Output: %d\n", *output);
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CONFIGURE DPLL DIVIDER ============================*/
/*============================================================================*/
/*= Each DPLL can have upto six independent output clocks based on various   =*/
/*= divider values. This function configures the different divider values for=*/
/*= the specified dpll.                                                      =*/
/*============================================================================*/

int prcm_configure_dpll_divider(u32 dpll, u32 setting)
{
	u32 dpll_id, valid, dpll_mxsft, omap, id_type;
	u32 dpll_div, new_val = 0x00000000;
	volatile u32 *addr;

	id_type = get_id_type(dpll);
	if (!(id_type & ID_DPLL_OP))
		return PRCM_FAIL;	/*Not DPLL OP */

	omap = OMAP(dpll);

	if (cpu_is_omap3430() && !(omap & (AT_3430|AT_3430_ES2)))
		return PRCM_FAIL;

	dpll_id = (dpll >> DPLL_NO_POS) & DPLL_NO_MASK;
	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	dpll_div = (dpll >> DPLL_DIV_POS) & DPLL_DIV_MASK;
	dpll_mxsft = dpll_mx_shift[dpll_id-1][dpll_div];
	addr = get_addr_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);
	valid = get_val_bits_pll(dpll_id, dpll_div + MX_ARRAY_OFFSET);

	new_val = (*addr & valid) | (setting << dpll_mxsft);

	*addr = new_val;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== CONFIGURE DPLL ====================================*/
/*============================================================================*/
/*= This function will set up the multiply and divide fields for the         =*/
/*= specified DPLL with the passed mult and div values. It also sets the     =*/
/*= frequency select field for the specified DPLL. The function returns      =*/
/*= PRCM_FAIL if the parameters passed are not valid.                        =*/
/*= Note that this function only does the configuration. Locking is done     =*/
/*= by prcm_enable_dpll() function                                           =*/
/*============================================================================*/

int prcm_configure_dpll(u32 dpll_id, u32 mult, u8 div, u8 freq_sel)
{
	u32 valid;
	volatile u32 *addr;
	volatile u32 *addr_auto;
	u32 new_reg_val = 0x0;
	int ret;
	int enabled;
	int index;
	u32 sys_clkspeed;
	u32 dpll_autoidle;

	if (dpll_id > NO_OF_DPLL)
		return PRCM_FAIL;

	/* Store the DPLL autoidle */
	addr_auto = get_addr_pll(dpll_id, REG_AUTOIDLE_PLL);
	dpll_autoidle = *addr_auto;
	*addr_auto = 0x0;

	/* DPLL M,N,FreqSel values should be changed only if the DPLL
	 * is in bypass mode. If it is not in bypass mode, return error */
	ret = is_dpll_locked(dpll_id, &enabled);

	if (enabled == PRCM_TRUE) {
		printk(KERN_INFO
			"DPLL %d enabled - m,n values cannot be changed\n",
			dpll_id);
		*addr_auto = dpll_autoidle;
		return PRCM_FAIL;
	}

	/* Configure M and N values */
	addr = get_addr_pll(dpll_id, REG_CLKSEL1_PLL);
	if (!addr)
		return PRCM_FAIL;
	if (dpll_id == DPLL5_PER2) {
		/* get the M/N/freqsel values */
		sys_clkspeed = prcm_get_system_clock_speed();
		switch (sys_clkspeed) {
		case (int)(12000):
			index = 0;
			break;
		case (int)(13000):
			index = 1;
			break;
		case (int)(19200):
			index = 2;
			break;
		case (int)(26000):
			index = 3;
			break;
		case (int)(38400):
			index = 4;
			break;
		default:
			return PRCM_FAIL;
		}
		mult = usb_dpll_param[index].dpll_m;
		div = usb_dpll_param[index].dpll_n;
		freq_sel = usb_dpll_param[index].dpll_freqsel;
	}
	valid = get_val_bits_pll(dpll_id, REG_CLKSEL1_PLL);
	if (dpll_id == DPLL3_CORE) {
		new_reg_val = *addr & valid & DPLL3_N_MASK;
		new_reg_val |= (mult << 16) | (div << 8);
		*addr = new_reg_val;
	} else {
		new_reg_val = *addr & valid & DPLL_N_MASK;
		new_reg_val |= (mult << 8) | div;
		*addr = new_reg_val;
	}

	/* Configure FreqSel values */
	addr = get_addr_pll(dpll_id, REG_CLKEN_PLL);
	if (!addr)
		return PRCM_FAIL;
	valid = get_val_bits_pll(dpll_id, REG_CLKEN_PLL);
	if (dpll_id == DPLL4_PER) {
		new_reg_val = *addr & valid;
		new_reg_val |= (freq_sel << 20);
		*addr = new_reg_val;
	} else {
		new_reg_val = *addr & valid;
		new_reg_val |= (freq_sel << 4);
		*addr = new_reg_val;
	}
	*addr_auto = dpll_autoidle;
	return PRCM_PASS;
}

/* API called to lock iva dpll after wakeup from core off */
int prcm_lock_iva_dpll(u32 target_opp_id)
{
	u32 tar_m, tar_n, tar_freqsel, sys_clkspeed;
	int index, target_opp_no;
	target_opp_no = target_opp_id & OPP_NO_MASK;
	sys_clkspeed = prcm_get_system_clock_speed();

	switch (sys_clkspeed) {
	case 12000: index = 0; break;
	case 13000: index = 1; break;
	case 19200: index = 2; break;
	case 26000: index = 3; break;
	case 38400: index = 4; break;
	default:
		return PRCM_FAIL;
	}

	tar_m = iva_dpll_param[index][target_opp_no - 1].dpll_m;
	tar_n = iva_dpll_param[index][target_opp_no - 1].dpll_n;
	/* M/N need to be changed - so put DPLL in bypass */
	prcm_put_dpll_in_bypass(DPLL2_IVA2, LOW_POWER_BYPASS);
	/* Reset M2 divider */
	prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, 0x1);
	/* Set M,N,Freqsel values */
	tar_freqsel = iva_dpll_param[index][target_opp_no - 1].dpll_freqsel;
	prcm_configure_dpll(DPLL2_IVA2, tar_m, tar_n, tar_freqsel);
	/* Lock the DPLL */
	prcm_enable_dpll(DPLL2_IVA2);
	return PRCM_PASS;
}

/******************************************************************************
 *
 * Level 2 API for frequency scaling.
 *
 ******************************************************************************/

int prcm_do_frequency_scaling(u32 target_opp_id, u32 current_opp_id)
{
	u32 id_type, vdd;
	u32 curr_m, curr_n, tar_m, tar_n, tar_freqsel, tar_m2;
	int curr_opp_no, target_opp_no, index;
	u32 sys_clkspeed;
	unsigned long flags;
	u32 rfr_ctrl = 0, actim_ctrla = 0, actim_ctrlb = 0;

	if (target_opp_id == current_opp_id)
		return PRCM_PASS;

	id_type = get_other_id_type(target_opp_id);
	if (!(id_type & ID_OPP))
		return PRCM_FAIL;

	id_type = get_other_id_type(current_opp_id);
	if (!(id_type & ID_OPP))
		return PRCM_FAIL;

	vdd = get_vdd(target_opp_id);
	if (vdd != get_vdd(current_opp_id))
		return PRCM_FAIL;

	sys_clkspeed = prcm_get_system_clock_speed();

	switch (sys_clkspeed) {
	case 12000: index = 0; break;
	case 13000: index = 1; break;
	case 19200: index = 2; break;
	case 26000: index = 3; break;
	case 38400: index = 4; break;
	default:
		return PRCM_FAIL;
	}

	if (vdd == PRCM_VDD1) {
		curr_opp_no = current_opp_id & OPP_NO_MASK;
		target_opp_no = target_opp_id & OPP_NO_MASK;
		curr_m = (mpu_dpll_param[index][curr_opp_no - 1].dpll_m);
		curr_n = (mpu_dpll_param[index][curr_opp_no - 1].dpll_n);
		tar_m = (mpu_dpll_param[index][target_opp_no - 1].dpll_m);
		tar_n = (mpu_dpll_param[index][target_opp_no - 1].dpll_n);

		if ((curr_m != tar_m) || (curr_n != tar_n)) {
			/* M/N need to be changed - so put DPLL in bypass */
			prcm_put_dpll_in_bypass(DPLL1_MPU, LOW_POWER_BYPASS);
			/* Reset M2 divider */
			prcm_configure_dpll_divider(PRCM_DPLL1_M2X2_CLK, 0x1);
			/* Set M,N,Freqsel values */
			tar_freqsel =
				mpu_dpll_param[index][target_opp_no - 1].dpll_freqsel;
			prcm_configure_dpll(DPLL1_MPU, tar_m, tar_n, tar_freqsel);
			/* Lock the DPLL */
			prcm_enable_dpll(DPLL1_MPU);
		}
		/* Configure M2 */
		tar_m2 = mpu_dpll_param[index][target_opp_no - 1].dpll_m2;
		prcm_configure_dpll_divider(PRCM_DPLL1_M2X2_CLK, tar_m2);

		curr_m = iva_dpll_param[index][curr_opp_no - 1].dpll_m;
		curr_n = iva_dpll_param[index][curr_opp_no - 1].dpll_n;
		tar_m =  iva_dpll_param[index][target_opp_no - 1].dpll_m;
		tar_n =  iva_dpll_param[index][target_opp_no - 1].dpll_n;

		if ((curr_m != tar_m) || (curr_n != tar_n)) {
			/* M/N need to be changed - so put DPLL in bypass */
			prcm_put_dpll_in_bypass(DPLL2_IVA2, LOW_POWER_BYPASS);
			/* Reset M2 divider */
			prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, 0x1);
			/* Set M,N,Freqsel values */
			tar_freqsel =
				iva_dpll_param[index][target_opp_no - 1].dpll_freqsel;
			prcm_configure_dpll(DPLL2_IVA2, tar_m, tar_n, tar_freqsel);
			/* Lock the DPLL */
			prcm_enable_dpll(DPLL2_IVA2);
		}
		/* Configure M2 */
		tar_m2 = iva_dpll_param[index][target_opp_no - 1].dpll_m2;
		prcm_configure_dpll_divider(PRCM_DPLL2_M2X2_CLK, tar_m2);

		/* Todo: Find out if some recalibation needs to be done
		 * in the OS since MPU freq has been changed */
		}
	else if (vdd == PRCM_VDD2) {
		target_opp_no = target_opp_id & OPP_NO_MASK;
		tar_m = (core_dpll_param[index][target_opp_no - 1].dpll_m);
		tar_n = (core_dpll_param[index][target_opp_no - 1].dpll_n);
		tar_freqsel = (core_dpll_param[index]
					[target_opp_no - 1].dpll_freqsel);
		tar_m2 = (core_dpll_param[index][target_opp_no - 1].dpll_m2);
		/* This function is executed from SRAM */
		local_irq_save(flags);
		if (target_opp_id == PRCM_VDD2_OPP3) {
			omap3_configure_gpmc(&omap3_vdd2_config[1]);
			rfr_ctrl    = omap3_vdd2_config[1].sdrc_cfg[0].sdrc_rfr_ctrl;
			actim_ctrla = omap3_vdd2_config[1].sdrc_cfg[0].sdrc_actim_ctrla;
			actim_ctrlb = omap3_vdd2_config[1].sdrc_cfg[0].sdrc_actim_ctrlb;
		} else if (target_opp_id == PRCM_VDD2_OPP2) {
			rfr_ctrl    = omap3_vdd2_config[0].sdrc_cfg[0].sdrc_rfr_ctrl;
			actim_ctrla = omap3_vdd2_config[0].sdrc_cfg[0].sdrc_actim_ctrla;
			actim_ctrlb = omap3_vdd2_config[0].sdrc_cfg[0].sdrc_actim_ctrlb;
		}
		omap3_configure_core_dpll(rfr_ctrl, actim_ctrla,
					  actim_ctrlb, tar_m2);
		if (target_opp_id == PRCM_VDD2_OPP2)
			/* TODO: We still configure the 166 GPMC values even at
			 * 83
			 */
			omap3_configure_gpmc(&omap3_vdd2_config[1]);

		local_irq_restore(flags);
	}
	return PRCM_PASS;
}


/******************************************************************************
 *
 * INIT
 *
 ******************************************************************************/

int __init prcm_vdd_clk_init(void)
{
	u32 opp_vdd1, mpu_khz, target_vdd1_opp, target_mpu_khz;

	printk(KERN_INFO "OMAP: Setting up clocks.\n");

	if (p_vdd1_clk == NULL)
		p_vdd1_clk = clk_get(NULL, "virt_vdd1_prcm_set");
	if (p_vdd1_clk == NULL) {
		printk(KERN_ERR "Unable to get the VDD1 clk\n");
		goto err_out;
	}

	if (p_vdd2_clk == NULL) 
		p_vdd2_clk = clk_get(NULL, "virt_vdd2_prcm_set");
	if (p_vdd2_clk == NULL) {
		printk(KERN_ERR "Unable to get the VDD2 clk\n");
		goto err_out;
	}

	/* Configure OPP for VDD1 based on compile time setting */
#if defined(CONFIG_OMAP3ES2_VDD1_OPP1)
	target_vdd1_opp = PRCM_VDD1_OPP1;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP2)
	target_vdd1_opp = PRCM_VDD1_OPP2;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP3)
	target_vdd1_opp = PRCM_VDD1_OPP3;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP4)
	target_vdd1_opp = PRCM_VDD1_OPP4;
#elif defined(CONFIG_OMAP3ES2_VDD1_OPP5)
	target_vdd1_opp = PRCM_VDD1_OPP5;
#endif
        target_mpu_khz  = get_arm_freq_for_opp(get_opp_no(target_vdd1_opp));

	/* get current OPP */
	prcm_get_processor_speed(DOM_MPU, &mpu_khz); 
	
	opp_vdd1 = get_vdd1_arm_constraint_for_freq(mpu_khz/1000);
	if( opp_vdd1 != get_opp_no(target_vdd1_opp) ) {
		printk(KERN_WARNING "OMAP: OPP mismatch: current/target=%d:%d\n", 
		       opp_vdd1, get_opp_no(target_vdd1_opp) );
		current_vdd1_opp = get_opp_for_index(opp_vdd1-1);
	}
	else if( mpu_khz != target_mpu_khz ){
		// opp is the same but clock does not match
		printk(KERN_WARNING "OMAP: MPU speed mismatch: current/target=%d:%d\n", 
		       mpu_khz, target_mpu_khz );
		if( mpu_khz < target_mpu_khz )
			opp_vdd1 = opp_vdd1 - 1; 
		else
			opp_vdd1 = opp_vdd1 + 1;
		current_vdd1_opp = get_opp_for_index(opp_vdd1-1);
	}
	else {
		// the same
		current_vdd1_opp = target_vdd1_opp;
	}

	if( current_vdd1_opp != target_vdd1_opp ) {
		printk(KERN_INFO "OMAP: Change MPU speed: %d => %d MHz\n",
		       mpu_khz/1000, target_mpu_khz/1000);
		vdd1_opp_setting(get_opp_no(target_vdd1_opp));
        }

	printk(KERN_INFO "OMAP: ARM/DSP clock at %d/%d MHz.\n",
	       get_arm_freq_for_opp(get_opp_no(target_vdd1_opp))/1000,
	       get_dsp_freq_for_opp(get_opp_no(target_vdd1_opp))/1000);

#ifdef  CONFIG_CPU_FREQ
	/* Get actual cpu speed */
	prcm_get_processor_speed(DOM_MPU, &target_mpu_khz); 

	loops_per_jiffy = cpufreq_scale( loops_per_jiffy, mpu_khz, target_mpu_khz);
	printk(KERN_INFO "BogoMIPS: %lu.%02lu @%d KHz\n",
	    loops_per_jiffy / (500000/HZ),
	   (loops_per_jiffy / (5000/HZ)) % 100, target_mpu_khz);
#endif

#ifdef CONFIG_OMAP3ES2_VDD2_OPP2
	current_vdd2_opp = PRCM_VDD2_OPP2;
#elif defined(CONFIG_OMAP3ES2_VDD2_OPP3)
	current_vdd2_opp = PRCM_VDD2_OPP3;
	prcm_do_voltage_scaling(PRCM_VDD2_OPP3, PRCM_VDD2_OPP2);
#endif

#if defined(CONFIG_PM) && defined(CONFIG_SYSFS)
	/* Sysfs entry for setting opp for vdd1 & vdd2 */
	if (0 != subsys_create_file(&power_subsys, &vdd1_opp)) {
		printk(KERN_ERR "Could not create sysfs entry for VDD1.\n");
	}
	if (0 != subsys_create_file(&power_subsys, &vdd2_opp)) {
		printk(KERN_ERR "Could not create sysfs entry for VDD2.\n");
	}
#endif

	return 0;

err_out:
	if (p_vdd1_clk)
		clk_put(p_vdd1_clk);
	if (p_vdd2_clk) 
		clk_put(p_vdd2_clk);

	return -1;
}


