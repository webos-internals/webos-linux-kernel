/*
 * linux/arch/arm/mach-omap3pe/smartreflex.c
 *
 * OMAP34XX SmartReflex Voltage Control
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>

#include <asm/arch/prcm.h>
#include <asm/arch/power_companion.h>
#include <asm/arch/resource.h>
#include <asm/io.h>

#include "smartreflex.h"
#include "prcm-regs.h"
#include "prcm_opp.h"
#include "prcm_util.h"

/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

/* #define DEBUG_SR 1 */

#ifdef DEBUG_SR
#  define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __func__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#define ABB_TRANXDONE_TIMEOUT	30

static u32 sr_nvalues = 0x0;


/*
 *  SR v1.5 support
 */
#ifdef CONFIG_OMAP_SMARTREFLEX_v15 

#define SR15_CONVERGANCE_TIMEOUT   (5)

typedef struct sr15_t {
	int             target_valid;    /* set if target_vsel is valid */  
	int             target_vsel;     /* target voltage */  
	typeof(jiffies) expires;
} sr15_t;


static u32    sr15_reeval_period = (24 * 60 * 60); /* seconds */
static struct sr15_t vdd1_sr15[PRCM_NO_VDD1_OPPS];
static struct sr15_t vdd2_sr15[PRCM_NO_VDD2_OPPS];

#endif /* CONFIG_OMAP_SMARTREFLEX_v15 */


struct omap_sr {
	int srid;
	int enabled;
	int sr_mode;
	struct clk *fck;
	u32 req_opp_no;
#ifdef CONFIG_MACH_SIRLOIN_3630
	u32 opp2_nvalue, opp3_nvalue, opp4_nvalue, opp5_nvalue;
#else
	u32 opp1_nvalue, opp2_nvalue, opp3_nvalue, opp4_nvalue, opp5_nvalue;
#endif
	u32 senp_mod, senn_mod;
	u32 srbase_addr;
	u32 vpbase_addr;
#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
	int    sr15_active;     
	int    sr15_opp_num;
	struct sr15_t *sr15_opp;
#endif
};

static struct omap_sr sr1 = {
	.srid = SR1,
	.enabled = 0,
	.sr_mode = SR_MODE_0,
	.srbase_addr = OMAP34XX_SR1_BASE,
#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
	.sr15_active  = 0,
	.sr15_opp_num = ARRAY_SIZE(vdd1_sr15),
	.sr15_opp     = vdd1_sr15,
	.sr_mode      = SR_MODE_15,
#else	
	.sr_mode     = SR_MODE_0,
#endif	
};

static struct omap_sr sr2 = {
	.srid = SR2,
	.enabled = 0,
	.sr_mode = SR_MODE_0,
	.srbase_addr = OMAP34XX_SR2_BASE,
#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
	.sr15_active  = 0,
	.sr15_opp_num = ARRAY_SIZE(vdd2_sr15),
	.sr15_opp     = vdd2_sr15,
	.sr_mode      = SR_MODE_15,
#else	
	.sr_mode      = SR_MODE_0,
#endif	
};


static inline void sr_write_reg(struct omap_sr *sr, int offset, u32 value)
{
	omap_writel(value, sr->srbase_addr + offset);
}

static inline void sr_modify_reg(struct omap_sr *sr, 
				 int offset, u32 mask, u32 value)
{
	u32 reg_val;

	reg_val = omap_readl(sr->srbase_addr + offset);
	reg_val &= ~mask;
	reg_val |= value;

	omap_writel(reg_val, sr->srbase_addr + offset);
}

static inline u32 sr_read_reg(struct omap_sr *sr, int offset)
{
	return omap_readl(sr->srbase_addr + offset);
}

#if 0
void sr_dump_sr_regs(struct omap_sr *sr)
{
	printk("####### SR%d\n", sr->srid);
	printk("####### SRCONFIG        = 0x%08x\n",	sr_read_reg(sr, SRCONFIG));
	printk("####### SRSTATUS        = 0x%08x\n",	sr_read_reg(sr, SRSTATUS));
	printk("####### SENVAL          = 0x%08x\n",	sr_read_reg(sr, SENVAL));
	printk("####### SENMIN          = 0x%08x\n",	sr_read_reg(sr, SENMIN));
	printk("####### SENMAX          = 0x%08x\n",	sr_read_reg(sr, SENMAX));
	printk("####### SENAVG          = 0x%08x\n",	sr_read_reg(sr, SENAVG));
	printk("####### AVGWEIGHT       = 0x%08x\n",	sr_read_reg(sr, AVGWEIGHT));
	printk("####### NVALUERECIPROCAL= 0x%08x\n",	sr_read_reg(sr, NVALUERECIPROCAL));
#ifdef CONFIG_MACH_SIRLOIN_3630
	printk("####### IRQSTATUS       = 0x%08x\n",	sr_read_reg(sr, IRQSTATUS));
	printk("####### IRQSTATUS_RAW   = 0x%08x\n",	sr_read_reg(sr, IRQSTATUS_RAW));
	printk("####### IRQENABLE_SET   = 0x%08x\n",	sr_read_reg(sr, IRQENABLE_SET));
	printk("####### IRQENABLE_CLR   = 0x%08x\n",	sr_read_reg(sr, IRQENABLE_CLR));
	printk("####### SENERROR        = 0x%08x\n",	sr_read_reg(sr, SENERROR));
	printk("####### ERRCONFIG       = 0x%08x\n",	sr_read_reg(sr, ERRCONFIG));
#else
	printk("####### SENERROR        = 0x%08x\n" ,	sr_read_reg(sr, SENERROR));
	printk("####### ERRCONFIG       = 0x%08x\n",	sr_read_reg(sr, ERRCONFIG));
#endif
	printk("####### SR%d: done\n", sr->srid);
}


void sr_dump_vp_regs(void)
{
	printk("####### PRM_VC_SMPS_SA     = 0x%08x\n",	PRM_VC_SMPS_SA);
	printk("####### PRM_VC_SMPS_VOL_RA = 0x%08x\n",	PRM_VC_SMPS_VOL_RA);
	printk("####### PRM_VC_SMPS_CMD_RA = 0x%08x\n",	PRM_VC_SMPS_CMD_RA);
	printk("####### PRM_VC_CMD_VAL_0   = 0x%08x\n",	PRM_VC_CMD_VAL_0);
	printk("####### PRM_VC_CMD_VAL_1   = 0x%08x\n",	PRM_VC_CMD_VAL_1);
	printk("####### PRM_VC_CH_CONF     = 0x%08x\n",	PRM_VC_CH_CONF);
	printk("####### PRM_VC_I2C_CFG     = 0x%08x\n",	PRM_VC_I2C_CFG);
	printk("####### PRM_VC_BYPASS_VAL  = 0x%08x\n",	PRM_VC_BYPASS_VAL);

	printk("####### PRM_VOLTCTRL       = 0x%08x\n",	PRM_VOLTCTRL);
	printk("####### PRM_VOLTOFFSET     = 0x%08x\n",	PRM_VOLTOFFSET);
	printk("####### PRM_VOLTSETUP1     = 0x%08x\n",	PRM_VOLTSETUP1);
	printk("####### PRM_VOLTSETUP2     = 0x%08x\n",	PRM_VOLTSETUP2);

	printk("####### PRM_VP1_CONFIG     = 0x%08x\n",	PRM_VP1_CONFIG);
	printk("####### PRM_VP1_VSTEPMIN   = 0x%08x\n",	PRM_VP1_VSTEPMIN);
	printk("####### PRM_VP1_VSTEPMAX   = 0x%08x\n",	PRM_VP1_VSTEPMAX);
	printk("####### PRM_VP1_VLIMITTO   = 0x%08x\n",	PRM_VP1_VLIMITTO);
	printk("####### PRM_VP1_VOLTAGE    = 0x%08x\n",	PRM_VP1_VOLTAGE);
	printk("####### PRM_VP1_STATUS     = 0x%08x\n",	PRM_VP1_STATUS);

	printk("####### PRM_VP2_CONFIG     = 0x%08x\n",	PRM_VP2_CONFIG);
	printk("####### PRM_VP2_VSTEPMIN   = 0x%08x\n",	PRM_VP2_VSTEPMIN);
	printk("####### PRM_VP2_VSTEPMAX   = 0x%08x\n",	PRM_VP2_VSTEPMAX);
	printk("####### PRM_VP2_VLIMITTO   = 0x%08x\n",	PRM_VP2_VLIMITTO);
	printk("####### PRM_VP2_VOLTAGE    = 0x%08x\n",	PRM_VP2_VOLTAGE);
	printk("####### PRM_VP2_STATUS     = 0x%08x\n",	PRM_VP2_STATUS);
	
	printk("####### PRM_LDO_ABB_SETUP  = 0x%08x\n",	PRM_LDO_ABB_SETUP);
	printk("####### PRM_LDO_ABB_CTRL   = 0x%08x\n",	PRM_LDO_ABB_CTRL);
}
#endif

static int sr_clk_enable(struct omap_sr *sr)
{
	if (clk_enable(sr->fck) != 0) {
		printk(KERN_ERR "Could not enable sr%d_fck\n", sr->srid);
		goto clk_enable_err;
	}

#ifndef CONFIG_MACH_SIRLOIN_3630
	/* set fclk- active , iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, 
			SR_CLKACTIVITY_MASK, SR_CLKACTIVITY_IOFF_FON);
#endif

	return 0;

clk_enable_err:
	return -1;
}

static int sr_clk_disable(struct omap_sr *sr)
{
#ifndef CONFIG_MACH_SIRLOIN_3630
	/* set fclk, iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, 
			SR_CLKACTIVITY_MASK, SR_CLKACTIVITY_IOFF_FOFF);
#endif

	clk_disable(sr->fck);

	return 0;
}

#ifndef CONFIG_MACH_SIRLOIN_3630

static void cal_reciprocal(u32 sensor, u32 *sengain, u32 *rnsen)
{
	u32 gn, rn, mul;

	for (gn = 0; gn < GAIN_MAXLIMIT; gn++) {
		mul = 1 << (gn + 8);
		rn = mul / sensor;
		if (rn < R_MAXLIMIT) {
			*sengain = gn;
			*rnsen = rn;
		}
	}
}

static u32 cal_test_nvalue(u32 sennval, u32 senpval)
{
	u32 senpgain, senngain;
	u32 rnsenp, rnsenn;

	/* Calculating the gain and reciprocal of
	* the SenN and SenP values */
	cal_reciprocal(senpval, &senpgain, &rnsenp);
	cal_reciprocal(sennval, &senngain, &rnsenn);

	return( (senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		(rnsenp   << NVALUERECIPROCAL_RNSENP_SHIFT) |
		(rnsenn   << NVALUERECIPROCAL_RNSENN_SHIFT));
}
#endif


#ifdef CONFIG_MACH_SIRLOIN_3630
static int sr_set_efuse_nvalues(void) 
{ 
	sr1.opp2_nvalue = omap_readl(CONTROL_FUSE_OPP50_VDD1 ) & 0xFFFFFF;
	sr1.opp3_nvalue = omap_readl(CONTROL_FUSE_OPP100_VDD1) & 0xFFFFFF;
	sr1.opp4_nvalue = omap_readl(CONTROL_FUSE_OPP130_VDD1) & 0xFFFFFF;
	sr1.opp5_nvalue = omap_readl(CONTROL_FUSE_OPP1G_VDD1 ) & 0xFFFFFF;
	
	if( sr1.opp2_nvalue == 0 || sr1.opp3_nvalue == 0 || 
	    sr1.opp4_nvalue == 0 || sr1.opp5_nvalue == 0 )
		return 0;
		

	sr2.opp2_nvalue = omap_readl(CONTROL_FUSE_OPP50_VDD2  ) & 0xFFFFFF;
	sr2.opp3_nvalue = omap_readl(CONTROL_FUSE_OPP100_VDD2 ) & 0xFFFFFF;

	if( sr2.opp2_nvalue == 0 || sr2.opp3_nvalue == 0 ) {
		return 0;
	}

	printk(KERN_INFO "SmartReflex Driver: Using EFUSED nvalues\n");

#if 0
	printk("FUSE_OPP50_VDD1 =0x%08x @ 0x%08x\n", omap_readl(CONTROL_FUSE_OPP50_VDD1),  CONTROL_FUSE_OPP50_VDD1);
	printk("FUSE_OPP100_VDD1=0x%08x @ 0x%08x\n", omap_readl(CONTROL_FUSE_OPP100_VDD1), CONTROL_FUSE_OPP100_VDD1);
	printk("FUSE_OPP130_VDD1=0x%08x @ 0x%08x\n", omap_readl(CONTROL_FUSE_OPP130_VDD1), CONTROL_FUSE_OPP130_VDD1);
	printk("FUSE_OPP1G_VDD1 =0x%08x @ 0x%08x\n", omap_readl(CONTROL_FUSE_OPP1G_VDD1) , CONTROL_FUSE_OPP1G_VDD1);

	printk("FUSE_OPP50_VDD2 =0x%08x @ 0x%08x\n", omap_readl(CONTROL_FUSE_OPP50_VDD2 ), CONTROL_FUSE_OPP50_VDD2 );
	printk("FUSE_OPP100_VDD2=0x%08x @ 0x%08x\n", omap_readl(CONTROL_FUSE_OPP100_VDD2), CONTROL_FUSE_OPP100_VDD2);
#endif

	return USE_EFUSE_NVALUES; 
}
#else
static int sr_set_efuse_nvalues(void)
{
	u32 n;
	u32 fuse_sr;
	int ret;

	fuse_sr = omap_readl(CONTROL_FUSE_SR);

	n = (fuse_sr & 0x0F);
	sr1.senp_mod = (n & 0x03);
	sr1.senn_mod = (n & 0x0C) >> 2;
	/* Read values for VDD1 from EFUSE */
	sr1.opp1_nvalue = omap_readl(CONTROL_FUSE_OPP1_VDD1) & 0xFFFFFF;
	sr1.opp2_nvalue = omap_readl(CONTROL_FUSE_OPP2_VDD1) & 0xFFFFFF;
	sr1.opp3_nvalue = omap_readl(CONTROL_FUSE_OPP3_VDD1) & 0xFFFFFF;

	sr1.opp4_nvalue = omap_readl(CONTROL_FUSE_OPP4_VDD1) & 0xFFFFFF;
	sr1.opp5_nvalue = omap_readl(CONTROL_FUSE_OPP5_VDD1) & 0xFFFFFF;

	n = (fuse_sr & 0x0F00) >> 8;
	sr2.senp_mod = (n & 0x03);
	sr2.senn_mod = (n & 0x0C) >> 2;
	/* Read values for VDD2 from EFUSE */
	sr2.opp1_nvalue = omap_readl(CONTROL_FUSE_OPP1_VDD2) & 0xFFFFFF;
	sr2.opp2_nvalue = omap_readl(CONTROL_FUSE_OPP2_VDD2) & 0xFFFFFF;
	sr2.opp3_nvalue = omap_readl(CONTROL_FUSE_OPP3_VDD2) & 0xFFFFFF;

	if (fuse_sr  == 0x0)
		ret = 0;
	else
		ret = USE_EFUSE_NVALUES;

	return(ret);
}
#endif

static int sr_set_test_nvalues(void)
{

#ifdef CONFIG_MACH_SIRLOIN_3630
	/* calculate nvalues for VDD1 OPP */
	/* 6 % system margin */
	sr1.opp2_nvalue = 0x00898DEB;
	sr1.opp3_nvalue = 0x00999B83;
	sr1.opp4_nvalue = 0x00AAC5A8;
	sr1.opp5_nvalue = 0x00AAB197;

	/* calculate nvalues for VDD2 OPP */
	sr2.opp2_nvalue = 0x00898BEB;
	sr2.opp3_nvalue = 0x009A8CEE;
# if 0
	/* 10% system margin values */
	sr1.opp2_nvalue = 0x008985E1;
	sr1.opp3_nvalue = 0x009A90F4;
	sr1.opp4_nvalue = 0x00AADABA;
	sr1.opp5_nvalue = 0x00AAA58E;

	/* calculate nvalues for VDD2 OPP */
	sr2.opp2_nvalue = 0x0099EEC8;
	sr2.opp3_nvalue = 0x00AAF3CF;
#endif

#else
	sr1.senp_mod = 0x03;    /* SenN-M5 enabled */
	sr1.senn_mod = 0x03;
	/* calculate nvalues for VDD1 OPP */
	sr1.opp1_nvalue = cal_test_nvalue(0x373 + 0x100, 0x28c + 0x100);
	sr1.opp2_nvalue = cal_test_nvalue(0x506 + 0x1a0, 0x3be + 0x1a0);
	sr1.opp3_nvalue = cal_test_nvalue(0x85b + 0x200, 0x655 + 0x200);
	sr1.opp4_nvalue = cal_test_nvalue(0x964 + 0x2a0, 0x727 + 0x2a0);
	sr1.opp5_nvalue = cal_test_nvalue(0xacd + 0x330, 0x848 + 0x330);

	sr2.senp_mod = 0x03;
	sr2.senn_mod = 0x03;
	/* calculate nvalues for VDD2 OPP */
	sr2.opp1_nvalue = cal_test_nvalue(0x359, 0x25d);
	sr2.opp2_nvalue = cal_test_nvalue(0x4f5 + 0x1c0, 0x390 + 0x1c0);
	sr2.opp3_nvalue = cal_test_nvalue(0x76f + 0x200, 0x579 + 0x200);
#endif

	printk(KERN_INFO "SmartReflex: Using TEST nvalues\n");

	return USE_TEST_NVALUES;
}

static void sr_configure_vp(int srid)
{
	u32 vpconfig;

	if (srid == SR1) {
#ifdef CONFIG_MACH_SIRLOIN_3630
		vpconfig = PRM_VP1_CONFIG_ERROROFFSET | PRM_VP1_CONFIG_TIMEOUTEN;
#else
		vpconfig = PRM_VP1_CONFIG_ERROROFFSET 
			 | PRM_VP1_CONFIG_ERRORGAIN
			 | PRM_VP1_CONFIG_INITVOLTAGE 
			 | PRM_VP1_CONFIG_TIMEOUTEN;
#endif

		PRM_VP1_CONFIG = vpconfig;
		PRM_VP1_VSTEPMIN = PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN
				 | PRM_VP1_VSTEPMIN_VSTEPMIN;

		PRM_VP1_VSTEPMAX = PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX
				 | PRM_VP1_VSTEPMAX_VSTEPMAX;

		PRM_VP1_VLIMITTO = PRM_VP1_VLIMITTO_VDDMAX
				 | PRM_VP1_VLIMITTO_VDDMIN
				 | PRM_VP1_VLIMITTO_TIMEOUT;

		PRM_VP1_CONFIG |=  PRM_VP1_CONFIG_INITVDD;
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_INITVDD;

	} else if (srid == SR2) {
#ifdef CONFIG_MACH_SIRLOIN_3630
		vpconfig = PRM_VP1_CONFIG_ERROROFFSET 
			 | PRM_VP1_CONFIG_TIMEOUTEN;
#else
		vpconfig = PRM_VP2_CONFIG_ERROROFFSET 
			 | PRM_VP2_CONFIG_ERRORGAIN
			 | PRM_VP2_CONFIG_INITVOLTAGE 
			 | PRM_VP2_CONFIG_TIMEOUTEN;
#endif

		PRM_VP2_CONFIG = vpconfig;
		PRM_VP2_VSTEPMIN = PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN
				 | PRM_VP2_VSTEPMIN_VSTEPMIN;

		PRM_VP2_VSTEPMAX = PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX
				 | PRM_VP2_VSTEPMAX_VSTEPMAX;

		PRM_VP2_VLIMITTO = PRM_VP2_VLIMITTO_VDDMAX 
				 | PRM_VP2_VLIMITTO_VDDMIN 
				 | PRM_VP2_VLIMITTO_TIMEOUT;

		PRM_VP2_CONFIG |=  PRM_VP2_CONFIG_INITVDD;
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_INITVDD;

	}
}

#define is_twl5030(n)	   	0
#define is_twl_rev_equal_to(v)	0

static void sr_configure_vc(void)
{
// PRM_VC_CMD_VALx_ON should be updated each time you change voltage with DVFS
	PRM_VC_SMPS_SA =
		(R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA1_SHIFT) |
		(R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA0_SHIFT);

	PRM_VC_SMPS_VOL_RA = (R_VDD2_SR_CONTROL << PRM_VC_SMPS_VOLRA1_SHIFT) |
			     (R_VDD1_SR_CONTROL << PRM_VC_SMPS_VOLRA0_SHIFT);

	PRM_VC_CMD_VAL_0 = (PRM_VC_CMD_VAL0_ON << PRM_VC_CMD_ON_SHIFT)
			 | (PRM_VC_CMD_VAL0_ONLP << PRM_VC_CMD_ONLP_SHIFT)
			 | (PRM_VC_CMD_VAL0_RET << PRM_VC_CMD_RET_SHIFT)
			 | (PRM_VC_CMD_VAL0_OFF << PRM_VC_CMD_OFF_SHIFT);

#ifdef CONFIG_TWL4030_CORE
	if (is_twl5030() && is_twl_rev_equal_to(TWL5030_REV_ES1_0)) {
		PRM_VC_CMD_VAL_1 = 
			(PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_ON_SHIFT)   |
			(PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_ONLP_SHIFT) |
			(PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_RET_SHIFT)  |
			(PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_OFF_SHIFT);
	} else {
		PRM_VC_CMD_VAL_1 = 
			(PRM_VC_CMD_VAL1_ON   << PRM_VC_CMD_ON_SHIFT)   |
			(PRM_VC_CMD_VAL1_ONLP << PRM_VC_CMD_ONLP_SHIFT) |
			(PRM_VC_CMD_VAL1_RET  << PRM_VC_CMD_RET_SHIFT)  |
			(PRM_VC_CMD_VAL1_OFF  << PRM_VC_CMD_OFF_SHIFT);
	}
#else
/* Add appropriate power IC configuration */
#endif

	PRM_VC_CH_CONF = PRM_VC_CH_CONF_CMD1 | PRM_VC_CH_CONF_RAV1;

	PRM_VC_I2C_CFG =  PRM_VC_I2C_CFG_MCODE 
			| PRM_VC_I2C_CFG_HSEN
			
	/* Errata ID: i531 (3630)
	 * 2.10. 1 Extra Power consumed when Repeated Start operation
	 * mode is enabled on I2C interface dedicated for Smart Reflex (I2C4)
	 */
#if 0	 
			| PRM_VC_I2C_CFG_SREN
#endif
			;

	/* Setup voltctrl and other setup times */
#ifdef CONFIG_SYSOFFMODE
	PRM_VOLTCTRL = PRM_VOLTCTRL_AUTO_OFF | PRM_VOLTCTRL_AUTO_RET;
	PRM_CLKSETUP = PRM_CLKSETUP_DURATION;
	PRM_VOLTSETUP1 = (PRM_VOLTSETUP_TIME2 << PRM_VOLTSETUP_TIME2_OFFSET) |
			 (PRM_VOLTSETUP_TIME1 << PRM_VOLTSETUP_TIME1_OFFSET);
	PRM_VOLTOFFSET = PRM_VOLTOFFSET_DURATION;
	PRM_VOLTSETUP2 = PRM_VOLTSETUP2_DURATION;
#else
	PRM_VOLTCTRL |= PRM_VOLTCTRL_AUTO_RET;
#endif

}


static void sr_configure(struct omap_sr *sr)
{
	u32 sys_clk, sr_clk_length = 0;
	u32 sr_config;
	u32 senp_en , senn_en;

#ifdef CONFIG_MACH_SIRLOIN_3630
	senp_en = 1;
	senn_en = 1;
#else
	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;
#endif

	sys_clk = prcm_get_system_clock_speed();

	switch (sys_clk) {
	case 12000:
		sr_clk_length = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000:
		sr_clk_length = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200:
		sr_clk_length = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000:
		sr_clk_length = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400:
		sr_clk_length = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default :
		printk(KERN_ERR "Invalid sysclk value\n");
		break;
	}

	if (sr->srid == SR1) {
#ifdef CONFIG_MACH_SIRLOIN_3630
		sr_config = (sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT);
#else
		sr_config = SR1_SRCONFIG_ACCUMDATA |
			(sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;
#endif

		sr_write_reg(sr, SRCONFIG, sr_config);

#ifdef CONFIG_MACH_SIRLOIN_3630
		sr_modify_reg(sr, ERRCONFIG, 
			(SR_ERRWEIGHT_MASK | SR_ERRMAXLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT));
#else
		sr_modify_reg(sr, ERRCONFIG, 
			(SR_ERRWEIGHT_MASK | SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT | SR1_ERRMINLIMIT));
#endif


	} else if (sr->srid == SR2) {
#ifdef CONFIG_MACH_SIRLOIN_3630
		sr_config = (sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT);
#else
		sr_config = SR2_SRCONFIG_ACCUMDATA |
			(sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;
#endif

		sr_write_reg(sr, SRCONFIG, sr_config);

#ifdef CONFIG_MACH_SIRLOIN_3630
		sr_modify_reg(sr, ERRCONFIG, 
			(SR_ERRWEIGHT_MASK | SR_ERRMAXLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT));
#else
		sr_modify_reg(sr, ERRCONFIG, 
			(SR_ERRWEIGHT_MASK | SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT | SR2_ERRMINLIMIT));
#endif

	}
}

static int sr_reset_voltage(int srid)
{
	u32 vsel = 0, target_opp_no;
	u32 reg_addr = 0;
	u32 vc_bypass_value;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;

	if (srid == SR1) {
		target_opp_no = get_opp_no(prcm_get_current_vdd1_opp());
		vsel = get_mpu_iva2_vdd1_volts(target_opp_no);
		reg_addr = R_VDD1_SR_CONTROL;
		t2_smps_steps = abs(vsel - PRM_VP1_VOLTAGE);

	} else if (srid == SR2) {
		target_opp_no = get_opp_no(prcm_get_current_vdd2_opp());
		vsel = get_core_l3_vdd2_volts(target_opp_no);
		reg_addr = R_VDD2_SR_CONTROL;
		t2_smps_steps = abs(vsel - PRM_VP2_VOLTAGE);
	}

	vc_bypass_value = (vsel << PRM_VC_BYPASS_DATA_SHIFT) |
			  (reg_addr << PRM_VC_BYPASS_REGADDR_SHIFT) |
			  (R_SRI2C_SLAVE_ADDR << PRM_VC_BYPASS_SLAVEADDR_SHIFT);

	PRM_VC_BYPASS_VAL = vc_bypass_value;

	PRM_VC_BYPASS_VAL |= PRM_VC_BYPASS_VALID;

	DPRINTK("PRM_VC_BYPASS_VAL %X\n", PRM_VC_BYPASS_VAL);
	DPRINTK("PRM_IRQST_MPU     %X\n", PRM_IRQSTATUS_MPU);

	while ((PRM_VC_BYPASS_VAL & PRM_VC_BYPASS_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			panic("Loop count exceeded in check SR I2C write\n");
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
	}
	t2_smps_delay = 2 + ((t2_smps_steps * 125) / 40);
	udelay(t2_smps_delay);

	return SR_PASS;
}

/******************************************************************************/

static u8 sr1errminlimit[5] = {
//	0x00, /* OPP1, not used */
	SR1_ERRMINLIMIT_OPP2,
	SR1_ERRMINLIMIT_OPP2,
	SR1_ERRMINLIMIT_OPP3,
	SR1_ERRMINLIMIT_OPP4,
	SR1_ERRMINLIMIT_OPP5
};

static u32 vp1errgain[5] = {
//	0x00, /* OPP1, not used */
	PRM_VP1_CONFIG_ERRORGAIN_OPP2,
	PRM_VP1_CONFIG_ERRORGAIN_OPP2,
	PRM_VP1_CONFIG_ERRORGAIN_OPP3,
	PRM_VP1_CONFIG_ERRORGAIN_OPP4,
	PRM_VP1_CONFIG_ERRORGAIN_OPP5
};

static u8 sr2errminlimit[3] = {
	0x00, /* OPP1, not used */
	SR2_ERRMINLIMIT_OPP2,
	SR2_ERRMINLIMIT_OPP3
};

static u32 vp2errgain[5] = {
	0x00, /* OPP1, not used */
	PRM_VP2_CONFIG_ERRORGAIN_OPP2,
	PRM_VP2_CONFIG_ERRORGAIN_OPP3
};

static void sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 nvalue_reciprocal, get_current_nvalue, vc, errorgain, errorminlimit;

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRDISABLE);

	sr->req_opp_no = target_opp_no;

#ifdef CONFIG_MACH_SIRLOIN_3630
	if (sr->srid == SR1) {
		switch (target_opp_no) {
		case 5:
			nvalue_reciprocal = sr->opp5_nvalue;
			break;
		case 4:
			nvalue_reciprocal = sr->opp4_nvalue;
			break;
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
		case 1: /* OPP 1 defaults to same as OPP 2 */
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		default:
			panic("Invalid VDD1 OPP (%d) in %s(). Bailing.\n",
				target_opp_no, __FUNCTION__);
			return;
		}
		errorgain     = vp1errgain[target_opp_no -1];
		errorminlimit = sr1errminlimit[target_opp_no -1];

	} else {
		switch (target_opp_no)
		{
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
		case 1:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		default:
			panic("Invalid VDD2 OPP (%d) in %s(). Bailing.\n",
				target_opp_no, __FUNCTION__);
			return;
		}
		errorgain     = vp2errgain[target_opp_no -1];
		errorminlimit = sr2errminlimit[target_opp_no -1];
	}
#else
	if (sr->srid == SR1) {
		errorgain = PRM_VP1_CONFIG_ERRORGAIN;
		errorminlimit = SR1_ERRMINLIMIT;
		switch (target_opp_no) {
		case 5:
			nvalue_reciprocal = sr->opp5_nvalue;
			break;
		case 4:
			nvalue_reciprocal = sr->opp4_nvalue;
			break;
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	} else {
		errorgain = PRM_VP2_CONFIG_ERRORGAIN;
		errorminlimit = SR2_ERRMINLIMIT;
		switch (target_opp_no) {
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	}
#endif
	get_current_nvalue = sr_read_reg(sr, NVALUERECIPROCAL);

	if (nvalue_reciprocal == 0) {
		printk(KERN_ERR "SmartReflex nvalues not configured.\n");
		return;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

	/* Enable the interrupt */
	sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST),
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST));

	sr_modify_reg(sr, ERRCONFIG,
			(SR1_ERRORMINLIMIT_MASK), (errorminlimit));

	if (sr->srid == SR1) {
		/* set/latch init voltage */
		vc = (PRM_VP1_CONFIG & ~(PRM_VPX_CONFIG_INITVOLTAGE_MASK |
					 PRM_VP1_CONFIG_INITVDD |
					 PRM_VP1_CONFIG_ERRORGAIN_MASK)) |
		     ((get_mpu_iva2_vdd1_volts(target_opp_no) << 8) | errorgain);
		PRM_VP1_CONFIG = vc; /* set initvoltage & clear InitVdd */
		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_INITVDD; /* write1 to latch */
		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_VPENABLE;/* write2 enable */
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_INITVDD;/* write3 clear */
	} else if (sr->srid == SR2) {
		/* set/latch init voltage */
		vc = (PRM_VP2_CONFIG & ~(PRM_VPX_CONFIG_INITVOLTAGE_MASK |
					 PRM_VP2_CONFIG_INITVDD |
					 PRM_VP2_CONFIG_ERRORGAIN_MASK)) |
		     ((get_core_l3_vdd2_volts(target_opp_no) << 8) | errorgain);
		PRM_VP2_CONFIG = vc; /* set initvoltage & clear InitVdd */
		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_INITVDD; /* write1 to latch */
		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_VPENABLE;/* write2 enable */
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_INITVDD;/* write3 clear */
	}

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);

	/* mark it as enabled */
	sr->enabled = 1;
}

static void sr_disable(struct omap_sr *sr)
{
	/* mark it as disabled */
	sr->enabled = 0;

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRDISABLE);

	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, 0x0);

	if (sr->srid == SR1) {
		/* Disable VP1 */
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_VPENABLE;
	} else if (sr->srid == SR2) {
		/* Disable VP2 */
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_VPENABLE;
	}
}

static void 
sr_start_vddautocomp(struct omap_sr *sr, u32 target_opp_no)
{
	if (!sr->enabled) {
		sr_clk_enable(sr);
		sr_configure(sr);
		sr_enable(sr, target_opp_no);
	}
}

static int  
sr_stop_vddautocomp(struct omap_sr *sr)
{
	if (sr->enabled) {
		sr_disable(sr);
		sr_clk_disable(sr);
		return SR_TRUE;
	}
	return SR_FALSE;
}

void enable_smartreflex(int srid)
{
	if (srid == SR1) {
		sr_start_vddautocomp ( &sr1, 
			get_opp_no(prcm_get_current_vdd1_opp()));
		return;

	}
	if (srid == SR2) {
		sr_start_vddautocomp ( &sr2, 
			get_opp_no(prcm_get_current_vdd2_opp()));
		return;
	}
	return;
}

int disable_smartreflex(int srid)
{
	int    rc;
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	rc = sr_stop_vddautocomp ( sr );
	if( rc ) {
		sr_reset_voltage(srid);
	}
	return rc;
}


/******************************************************************************
 *
 * ABB
 *
 ******************************************************************************/
/**
 * omap3630_abb_change_active_opp - handle OPP changes with Adaptive Body-Bias
 * @target_opp_no: OPP we're transitioning to
 *
 * Adaptive Body-Bias is a 3630-specific technique to boost voltage in high
 * OPPs for silicon with weak characteristics as well as lower voltage in low
 * OPPs for silicon with strong characteristics.
 *
 * Only Foward Body-Bias for operating at high OPPs is implemented below.
 * Reverse Body-Bias for saving power in active cases and sleep cases is not
 * yet implemented.
 */

#ifdef CONFIG_MACH_SIRLOIN_3630

#define  FBB_OPP   CO_VDD1_OPP5
 
static void omap3630_abb_change_active_opp(u32 target_opp_no)
{
	u32 val;
	int timeout;

	DPRINTK("Set FBB for OPP%d\n", target_opp_no );

	/* FIXME: force ACTIVE_FBB_SEL until EFUSE values are burnt in */
	PRM_LDO_ABB_CTRL |= OMAP3630_ACTIVE_FBB_SEL;

	/* program settling time of 30us for ABB ldo transition */
	val = PRM_LDO_ABB_CTRL & ~OMAP3630_SR2_WTCNT_VALUE_MASK;
	PRM_LDO_ABB_CTRL = val | (0x62 << OMAP3630_SR2_WTCNT_VALUE_SHIFT);

	/* set SR2EN */
	PRM_LDO_ABB_CTRL |= OMAP3630_SR2EN;

	/* clear ABB ldo interrupt status */
	PRM_IRQSTATUS_MPU = OMAP3630_ABB_LDO_TRANXDONE_ST;

	/* select OPP */
	val = PRM_LDO_ABB_SETUP & ~OMAP3630_OPP_SEL_MASK;
	val |= ((target_opp_no == FBB_OPP ? 1 : 2) << OMAP3630_OPP_SEL_SHIFT);
	PRM_LDO_ABB_SETUP = val;

	/* enable ABB LDO OPP change */
	PRM_LDO_ABB_SETUP |= OMAP3630_OPP_CHANGE;

	timeout = 0;
	/* wait until OPP change completes */
	while ((timeout < ABB_TRANXDONE_TIMEOUT) &&
		!(PRM_IRQSTATUS_MPU & OMAP3630_ABB_LDO_TRANXDONE_ST)) {
		udelay(1);
		timeout++;
	}
	if (timeout == ABB_TRANXDONE_TIMEOUT) {
		panic("ABB: TRANXDONE timed out waiting for OPP change\n");
	}

	/* clear ABB ldo interrupt status */
	PRM_IRQSTATUS_MPU = OMAP3630_ABB_LDO_TRANXDONE_ST;
}

#endif /* CONFIG_MACH_SIRLOIN_3630 */


static u32 
sr_get_curr_vsel(struct omap_sr *sr, u32 opp_no)
{
	if (sr->srid == SR1) {
		if( sr->enabled || opp_no == 0) {
			return (PRM_VP1_VOLTAGE & 0xFF);
		} else {
			return (u32) get_mpu_iva2_vdd1_volts(opp_no);
		}
	}
	if (sr->srid == SR2) {
		if( sr->enabled || opp_no == 0) {
			return (PRM_VP2_VOLTAGE & 0xFF);
		} else {
			return (u32) get_core_l3_vdd2_volts(opp_no);
		}
	}
	return 0;
}

#ifdef CONFIG_OMAP_SMARTREFLEX_v15 

static u32
sr_get_nominal_voltage(int vdd, int opp_no)
{
	return (vdd == PRCM_VDD1) ?
		(u32) get_mpu_iva2_vdd1_volts(opp_no) :
		(u32) get_core_l3_vdd2_volts (opp_no);
}

static int
sr15_update_target_opp(int vdd, int opp_no, int msec )
{
	struct omap_sr *sr = NULL;
	int need_to_reset_vsel = 0;

	sr = (vdd == PRCM_VDD1) ? &sr1 : &sr2;

	/* update current opp */
	if (sr->sr15_active) {
		u32 old_vsel;
		u32 new_vsel;
		u32 nominal_vsel;

		if (msec) mdelay(msec);

		old_vsel = sr->sr15_opp[opp_no-1].target_vsel;
		new_vsel = sr_get_curr_vsel( sr, opp_no );

		if (vdd == PRCM_VDD1 && opp_no == CO_VDD1_OPP5) {
			/* add 3 steps to make a margin */
			printk( KERN_NOTICE
				"SR%d: OPP%d: calibrated to %d. add margin.\n",
				sr->srid, opp_no, new_vsel );

			new_vsel += 3;
			nominal_vsel = sr_get_nominal_voltage(vdd, opp_no);
			if (new_vsel > nominal_vsel)
				new_vsel = nominal_vsel;

			need_to_reset_vsel = 1;
		}
		sr->sr15_active = 0;
		sr->sr15_opp[opp_no-1].target_vsel  = new_vsel;
		sr->sr15_opp[opp_no-1].target_valid = 1;
		sr->sr15_opp[opp_no-1].expires = jiffies + 
				       sr15_reeval_period * HZ;
		printk( KERN_NOTICE "SR%d: OPP%d: update vsel %d => %d\n",
		        sr->srid, opp_no, old_vsel, new_vsel );

		sr_stop_vddautocomp   ( sr );
	}
	return need_to_reset_vsel;
}


static void
sr15_invalidate_opps(struct omap_sr *sr)
{
	int  i;
	for( i = 0; i < sr->sr15_opp_num; i++ ) {
		sr->sr15_opp[i].target_valid = 0;
		sr->sr15_opp[i].target_vsel  = 0;
	}
}

static  u32 
sr_get_target_voltage (int vdd, int target_no, int curr_no )
{
	u32 target_vsel;
	u32 nominal_vsel;
	struct omap_sr *sr = NULL;

	nominal_vsel = sr_get_nominal_voltage(vdd, target_no);
	sr = (vdd == PRCM_VDD1) ? &sr1 : &sr2;

	if (sr->sr_mode != SR_MODE_15)
		return nominal_vsel;

	/* for v1.5 mode */
	if (!sr->sr15_opp[target_no-1].target_valid) {
		/* if target opp is not valid,
		 * go to nominal voltage and recalibrate.
		 */
		target_vsel = nominal_vsel;
		sr->sr15_active = 1;
	} else if (time_after(jiffies, sr->sr15_opp[target_no-1].expires)) {
		/* if target opp is valid but expired,
		 * go to (current + 4 steps) voltage and recalibrate.
		 */
		sr->sr15_opp[target_no-1].target_valid = 0;

		target_vsel = sr->sr15_opp[target_no-1].target_vsel + 4;
		if (target_vsel > nominal_vsel)
			target_vsel = nominal_vsel;
		sr->sr15_active = 1;
	} else {
		/* target opp is valid */
		target_vsel = sr->sr15_opp[target_no-1].target_vsel;
	}

	if (sr->sr15_active)
		printk(KERN_NOTICE
		       "SR%d: OPP%d: initiate calibration (initial vsel %d)\n",
		       sr->srid, target_no, target_vsel);

	return target_vsel;
}

#else  /* CONFIG_OMAP_SMARTREFLEX_v15 */

static  u32 
sr_get_target_voltage (int vdd, int target_no, int curr_no )
{
	u32 target_vsel = 0;

	if (vdd == PRCM_VDD1) {
		target_vsel = (u32) get_mpu_iva2_vdd1_volts(target_no);
	} else if (vdd == PRCM_VDD2) {
		target_vsel = (u32) get_core_l3_vdd2_volts (target_no);
	} else {
		panic("invalid (%d) vdd param\n", vdd );
	}
	return target_vsel;
}

#endif /* CONFIG_OMAP_SMARTREFLEX_v15 */


/* Voltage Scaling using SR VCBYPASS */
int sr_voltagescale(u32 target_opp, u32 cur_opp, u32 vsel )
{
	int sr_status = 0;
	u32 vdd;
	u32 target_opp_no, cur_opp_no;
	u32 reg_addr = 0;
	int loop_cnt;
#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	u32 vc_bypass_value;
	u32 retries_cnt;
#endif
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;

	vdd  = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);
	cur_opp_no    = get_opp_no(cur_opp);

#ifdef CONFIG_OMAP_SMARTREFLEX_v15
repeat:
#endif
	if(!vsel)
		vsel  = sr_get_target_voltage (vdd, target_opp_no, cur_opp_no);

	if (vdd == PRCM_VDD1) {
		t2_smps_steps = abs(vsel - sr_get_curr_vsel( &sr1, cur_opp_no));
		sr_status = sr_stop_vddautocomp (&sr1);

#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
		/* override sr_status if we use sr15 */
		if (sr1.sr_mode == SR_MODE_15) {
			if( sr1.sr15_active )
				sr_status = SR_TRUE;
			else
				sr_status = SR_FALSE;
		}
#endif
		

		PRM_VC_CMD_VAL_0 = (PRM_VC_CMD_VAL_0 & ~PRM_VC_CMD_ON_MASK) |
				    (vsel << PRM_VC_CMD_ON_SHIFT);
		reg_addr = R_VDD1_SR_CONTROL;

	} else if (vdd == PRCM_VDD2) {
		t2_smps_steps = abs(vsel - sr_get_curr_vsel( &sr2, cur_opp_no));
		sr_status = sr_stop_vddautocomp (&sr2);

#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
		/* override sr_status if we use sr15 */
		if (sr1.sr_mode == SR_MODE_15) {
			if( sr2.sr15_active )
				sr_status = SR_TRUE;
			else
				sr_status = SR_FALSE;
		}
#endif
 
		PRM_VC_CMD_VAL_1 = (PRM_VC_CMD_VAL_1 & ~PRM_VC_CMD_ON_MASK) |
				    (vsel << PRM_VC_CMD_ON_SHIFT);
		reg_addr = R_VDD2_SR_CONTROL;
	}

	if (vdd == PRCM_VDD1) {
		/* We only need to do this here for 5->non-5 transitions */
		if( cur_opp_no == FBB_OPP && target_opp_no != FBB_OPP ) {
			omap3630_abb_change_active_opp(target_opp_no);
		}
	}

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
	vc_bypass_value = (vsel << PRM_VC_BYPASS_DATA_SHIFT) |
			  (reg_addr << PRM_VC_BYPASS_REGADDR_SHIFT) |
			  (R_SRI2C_SLAVE_ADDR << PRM_VC_BYPASS_SLAVEADDR_SHIFT);

	PRM_VC_BYPASS_VAL = vc_bypass_value;

	PRM_VC_BYPASS_VAL |= PRM_VC_BYPASS_VALID;

	DPRINTK("PRM_VC_BYPASS_VAL %X\n", PRM_VC_BYPASS_VAL);
	DPRINTK("PRM_IRQST_MPU %X\n",     PRM_IRQSTATUS_MPU);

	loop_cnt = 0;
	retries_cnt = 0;
	while ((PRM_VC_BYPASS_VAL & PRM_VC_BYPASS_VALID) != 0x0) {
		loop_cnt++;
		if (retries_cnt > 10) {
			panic("Loop count exceeded in check SR I2C write\n");
		}
		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			udelay(10);
		}
	}
#elif defined (CONFIG_OMAP_VOLT_SR_FORCEUPDATE) /* Force Update */
	/* Clear interrupt and wait until i2c command is finished. */
	loop_cnt = 255;
	while (loop_cnt-- > 0) {
		PRM_IRQSTATUS_MPU = OMAP3630_VP1_TRANXDONE_ST | OMAP3630_VP2_TRANXDONE_ST;

		if (!(PRM_IRQSTATUS_MPU & (OMAP3630_VP1_TRANXDONE_ST | OMAP3630_VP2_TRANXDONE_ST)))
				break;
		udelay(1);
	}
	if (loop_cnt <= 0) {
		panic( "MPU IRQSTATUS did not clear in ForceUpdate (%08x\n)\n",
			PRM_IRQSTATUS_MPU);
	}

	if (vdd == PRCM_VDD1) {
		PRM_VP1_CONFIG &= ~(OMAP3630_VP_FORCEUPDATE |
				    OMAP3630_VP_INITVDD     |
				    OMAP3630_VP_INITVOLTAGE);
		PRM_VP1_CONFIG |= (vsel << OMAP3630_VP_INITVOLTAGE_SHIFT);

		PRM_VP1_CONFIG |= OMAP3630_VP_INITVDD;

		PRM_VP1_CONFIG |= OMAP3630_VP_FORCEUPDATE;

		loop_cnt = 200;
		while (loop_cnt-- > 0 &&
			!(PRM_IRQSTATUS_MPU & OMAP3630_VP1_TRANXDONE_ST))
			udelay(1);

		if (loop_cnt <= 0) {
			panic("%s: Voltage ForceUpdate timed out.\n", "VDD1");
		}
		/* Clear interrupt status */
		PRM_IRQSTATUS_MPU = OMAP3630_VP1_TRANXDONE_ST;
	} else if (vdd == PRCM_VDD2) {
		PRM_VP2_CONFIG &= ~(OMAP3630_VP_FORCEUPDATE |
				    OMAP3630_VP_INITVDD     |
				    OMAP3630_VP_INITVOLTAGE);
		PRM_VP2_CONFIG |= (vsel << OMAP3630_VP_INITVOLTAGE_SHIFT);

		PRM_VP2_CONFIG |= OMAP3630_VP_INITVDD;

		PRM_VP2_CONFIG |= OMAP3630_VP_FORCEUPDATE;

		loop_cnt = 200;
		while (loop_cnt-- > 0 &&
			!(PRM_IRQSTATUS_MPU & OMAP3630_VP2_TRANXDONE_ST))
			udelay(1);

		if (loop_cnt <= 0) {
			panic("%s: Voltage ForceUpdate timed out.\n", "VDD2");
		}
		/* Clear interrupt status */
		PRM_IRQSTATUS_MPU = OMAP3630_VP2_TRANXDONE_ST; 
	}
#endif

	/* T2 SMPS slew rate (min) 4mV/uS */
	t2_smps_delay = 2 + ((t2_smps_steps * 125) / 40);
	udelay(t2_smps_delay);

#ifdef CONFIG_OMAP_VOLT_SR_FORCEUPDATE
	/* clear forceupdate and initvdd bits */
	if (vdd == PRCM_VDD1) {
		PRM_VP1_CONFIG &=
			~(OMAP3630_VP_FORCEUPDATE | OMAP3630_VP_INITVDD);
	} else if (vdd == PRCM_VDD2) {
		PRM_VP2_CONFIG &=
			~(OMAP3630_VP_FORCEUPDATE | OMAP3630_VP_INITVDD);
	}
#endif

#ifdef CONFIG_MACH_SIRLOIN_3630
	if (vdd == PRCM_VDD1) {
		/* We only need to do this here for non-5 ->5. transitions */
		if( target_opp_no == FBB_OPP && cur_opp_no != FBB_OPP )
			omap3630_abb_change_active_opp(target_opp_no);
	}
#endif

	if (sr_status) {
		if (vdd == PRCM_VDD1) {
			sr_start_vddautocomp  ( &sr1, target_opp_no);
		}
		else if (vdd == PRCM_VDD2) {
			sr_start_vddautocomp  ( &sr2, target_opp_no);
		}
#ifdef CONFIG_OMAP_SMARTREFLEX_v15
		if (sr15_update_target_opp( vdd, target_opp_no,
					    SR15_CONVERGANCE_TIMEOUT)) {
			/*
			 * target vsel has been adjusted on the table.
			 * need to repeat for it.
			 */
			vsel = 0;
			goto repeat;
		}
#endif
	}

	return SR_PASS;
}

/* Sysfs interface to select SR VDD1 auto compensation */
static ssize_t 
omap_sr_vdd1_autocomp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%d\n", sr1.sr_mode );
}

static ssize_t 
omap_sr_vdd1_autocomp_store(struct kset *subsys, const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 ) {
		printk(KERN_ERR "sr_vdd1_autocomp: Invalid value\n");
		return -EINVAL;
	}

	if (sr_nvalues == 0) {
		printk(KERN_ERR "\nVDD1 nvalues not set, use test nvalues\n");
		return -EINVAL;
	}

	if (value == 0 ) {
		sr1.sr_mode = SR_MODE_0;
		disable_smartreflex (SR1);
	} 
	else if (value == 1 || value == 30) {
		sr1.sr_mode = SR_MODE_30;
		enable_smartreflex  (SR1);
	}
#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
	else if (value == 15) {
		sr1.sr_mode = SR_MODE_15;
		disable_smartreflex (SR1);
		sr15_invalidate_opps(&sr1);
	}
#endif	

	return n;
}

static struct subsys_attribute sr_vdd1_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd1_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd1_autocomp_show,
	.store = omap_sr_vdd1_autocomp_store,
};

/* Sysfs interface to select SR VDD2 auto compensation */
static ssize_t omap_sr_vdd2_autocomp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%d\n", sr2.sr_mode );
}

static ssize_t omap_sr_vdd2_autocomp_store(struct kset *subsys,
				const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1) {
		printk(KERN_ERR "sr_vdd2_autocomp: Invalid value\n");
		return -EINVAL;
	}

	if (sr_nvalues == 0) {
		printk(KERN_ERR "\nVDD2 nvalues not set, use test nvalues\n");
		return -EINVAL;
	}

	if (value == 0 ) {
		sr2.sr_mode = SR_MODE_0;
		disable_smartreflex (SR2);
	} 
	else if (value == 1 || value == 30) {
		sr2.sr_mode = SR_MODE_30;
		enable_smartreflex  (SR2);
	}
#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
	else if (value == 15 ) {
		sr2.sr_mode = SR_MODE_15;
		disable_smartreflex (SR2);
		sr15_invalidate_opps(&sr2);
	}
#endif

	return n;
}

static struct subsys_attribute sr_vdd2_autocomp = {
	.attr = {
	.name = __stringify(sr_vdd2_autocomp),
	.mode = 0644,
	},
	.show = omap_sr_vdd2_autocomp_show,
	.store = omap_sr_vdd2_autocomp_store,
};

/* Sysfs interface to set TEST NVALUES */
static ssize_t omap_sr_setnvalues_test_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%d\n", (sr_nvalues - 1));
}

static ssize_t omap_sr_setnvalues_test_store(struct kset *subsys,
				const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_setnvalues: Invalid value\n");
		return -EINVAL;
	}

	if ((sr1.enabled == 1) || (sr2.enabled == 1)) {
		printk(KERN_ERR 
			"\nDisable SR autocomp before changing nvalues\n");
		return -EINVAL;
	}

	value = 1; // ensure to use test values for now as eFuse is empty

	/* Disable SmartReflex before changing the nvalues */
	if (value == 0) {
		sr_nvalues = sr_set_efuse_nvalues();
	} else {
		sr_nvalues = sr_set_test_nvalues();
	}

	return n;
}

static struct subsys_attribute sr_setnvalues_test = {
	.attr = {
	.name = __stringify(sr_setnvalues_test),
	.mode = 0644,
	},
	.show = omap_sr_setnvalues_test_show,
	.store = omap_sr_setnvalues_test_store,
};

/******************************************************************************/

/* Sysfs interface to set VP1 error gain values */
static ssize_t omap_sr_vp1errgain_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "1 %02x\n2 %02x\n3 %02x\n4 %02x\n5 %02x\n",
			vp1errgain[0] >> 16,
			vp1errgain[1] >> 16,
			vp1errgain[2] >> 16,
			vp1errgain[3] >> 16,
			vp1errgain[4] >> 16);
}

static ssize_t omap_sr_vp1errgain_store(struct kset *subsys,
				const char *buf, size_t n)
{
	u32 idx;
	u32 value;

	if (sscanf(buf, "%d %02x", &idx, &value) != 2 ||
			(idx < 1) || (idx > 5)) {
		printk(KERN_ERR "sr_vp1errgain: Invalid value\n");
		return -EINVAL;
	}

	vp1errgain[idx -1] = value << 16;
	return n;
}

static struct subsys_attribute sr_vp1errgain = {
	.attr = {
	.name = __stringify(sr_vp1errgain),
	.mode = 0644,
	},
	.show  = omap_sr_vp1errgain_show,
	.store = omap_sr_vp1errgain_store,
};

/* Sysfs interface to set VP2 error gain values */
static ssize_t omap_sr_vp2errgain_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "1 %02x\n2 %02x\n3 %02x\n",
			vp2errgain[0] >> 16,
			vp2errgain[1] >> 16,
			vp2errgain[2] >> 16);
}

static ssize_t omap_sr_vp2errgain_store(struct kset *subsys,
				const char *buf, size_t n)
{
	u32 idx;
	u32 value;

	if (sscanf(buf, "%d %02x", &idx, &value) != 2 ||
			(idx < 1) || (idx > 3)) {
		printk(KERN_ERR "sr_vp2errgain: Invalid value\n");
		return -EINVAL;
	}

	vp2errgain[idx -1] = value << 16;
	return n;
}

static struct subsys_attribute sr_vp2errgain = {
	.attr = {
	.name = __stringify(sr_vp2errgain),
	.mode = 0644,
	},
	.show  = omap_sr_vp2errgain_show,
	.store = omap_sr_vp2errgain_store,
};


/* Sysfs interface to set SR1 error min values */
static ssize_t omap_sr_sr1errminlimit_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "1 %02x\n2 %02x\n3 %02x\n4 %02x\n5 %02x\n",
			sr1errminlimit[0],
			sr1errminlimit[1],
			sr1errminlimit[2],
			sr1errminlimit[3],
			sr1errminlimit[4]);
}

static ssize_t omap_sr_sr1errminlimit_store(struct kset *subsys,
				const char *buf, size_t n)
{
	u32 idx;
	u32 value;

	if (sscanf(buf, "%d %02x", &idx, &value) != 2 ||
			(idx < 1) || (idx > 5)) {
		printk(KERN_ERR "sr_sr1errminlimit Invalid value\n");
		return -EINVAL;
	}

	sr1errminlimit[idx -1] = value;
	return n;
}

static struct subsys_attribute sr_sr1errminlimit= {
	.attr = {
	.name = __stringify(sr_sr1errminlimit),
	.mode = 0644,
	},
	.show  = omap_sr_sr1errminlimit_show,
	.store = omap_sr_sr1errminlimit_store,
};

/* Sysfs interface to set SR2 error min values */
static ssize_t omap_sr_sr2errminlimit_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "1 %02x\n2 %02x\n3 %02x\n",
			sr2errminlimit[0],
			sr2errminlimit[1],
			sr2errminlimit[2]);
}

static ssize_t omap_sr_sr2errminlimit_store(struct kset *subsys,
				const char *buf, size_t n)
{
	u32 idx;
	u32 value;

	if (sscanf(buf, "%d %02x", &idx, &value) != 2 ||
			(idx < 1) || (idx > 5)) {
		printk(KERN_ERR "sr_sr2errminlimit Invalid value\n");
		return -EINVAL;
	}

	sr2errminlimit[idx -1] = value;
	return n;
}

static struct subsys_attribute sr_sr2errminlimit= {
	.attr = {
	.name = __stringify(sr_sr2errminlimit),
	.mode = 0644,
	},
	.show  = omap_sr_sr2errminlimit_show,
	.store = omap_sr_sr2errminlimit_store,
};

static ssize_t 
vdd1_vsel_show(struct kset *subsys, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", PRM_VP1_VOLTAGE );
}


static struct subsys_attribute vdd1_vsel = {
	.attr = {
	.name = __stringify(vdd1_vsel),
	.mode = 0444,
	},
	.show  = vdd1_vsel_show,
};


static ssize_t 
vdd2_vsel_show(struct kset *subsys, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", PRM_VP2_VOLTAGE );
}


static struct subsys_attribute vdd2_vsel = {
	.attr = {
	.name = __stringify(vdd2_vsel),
	.mode = 0444,
	},
	.show  = vdd2_vsel_show,
};


#ifdef CONFIG_OMAP_SMARTREFLEX_v15 

static ssize_t 
sr15_period_show(struct kset *subsys, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", sr15_reeval_period );
}

static ssize_t 
sr15_period_store(struct kset *subsys, const char *buf, size_t n)
{
	u32 value;

	value = (u32) simple_strtoul ( buf, NULL, 0 );
	if( value > 5 ) {
		/* update period */
		sr15_reeval_period = value;

		/* invalidate sr15 opp */
		if (sr1.sr_mode == SR_MODE_15)
			sr15_invalidate_opps(&sr1);
		if (sr2.sr_mode == SR_MODE_15)
			sr15_invalidate_opps(&sr2);
	} else {
		printk (KERN_ERR "SR1.5: reevaluation period is too short\n");
	}

	return n;
}


static struct subsys_attribute sr15_period = {
	.attr = {
	.name = __stringify(sr15_period),
	.mode = 0644,
	},
	.show  = sr15_period_show,
	.store = sr15_period_store,
};

static ssize_t 
sr15_vdd1_opp_state_show(struct kset *subsys, char *buf)
{
	int i, len, vsel;

	len = 0;
	for ( i = 0; i < sr1.sr15_opp_num; i++ ) {
		vsel = get_mpu_iva2_vdd1_volts(i+1); 
		len += snprintf(buf + len, PAGE_SIZE, "%3d ", vsel);
	}
	len += snprintf(buf + len, PAGE_SIZE, "\n");

	for ( i = 0; i < sr1.sr15_opp_num; i++ ) {
		if (sr1.sr15_opp[i].target_valid)
			vsel = sr1.sr15_opp[i].target_vsel;
		else 
			vsel = 0;
		
		len += snprintf(buf + len, PAGE_SIZE, "%3d ", vsel);
	}

	len += snprintf(buf + len, PAGE_SIZE, "\n");

	return len;
}


static struct subsys_attribute sr15_vdd1_opp_state = {
	.attr = {
	.name = __stringify(sr15_vdd1_opp_state),
	.mode = 0444,
	},
	.show  = sr15_vdd1_opp_state_show,
};


static ssize_t 
sr15_vdd2_opp_state_show(struct kset *subsys, char *buf)
{
	int i, len, vsel;

	len = 0;
	for ( i = 0; i < sr2.sr15_opp_num; i++ ) {
		vsel = get_core_l3_vdd2_volts(i+1);
		len += snprintf(buf + len, PAGE_SIZE, "%3d ", vsel);
	}

	len += snprintf(buf + len, PAGE_SIZE, "\n");

	for ( i = 0; i < sr2.sr15_opp_num; i++ ) {
		if (sr2.sr15_opp[i].target_valid)
			vsel = sr2.sr15_opp[i].target_vsel;
		else 
			vsel = 0;
		
		len += snprintf(buf + len, PAGE_SIZE, "%3d ", vsel);
	}

	len += snprintf(buf + len, PAGE_SIZE, "\n");

	return len;
}


static struct subsys_attribute sr15_vdd2_opp_state = {
	.attr = {
	.name = __stringify(sr15_vdd2_opp_state),
	.mode = 0444,
	},
	.show  = sr15_vdd2_opp_state_show,
};

#endif

/******************************************************************************/

static int __init omap3_sr_init(void)
{
	int ret = 0;
#ifdef CONFIG_TWL4030_CORE
	u8 RdReg;
#else
#endif /* #ifdef  CONFIG_TWL4030_CORE */

	printk(KERN_INFO "SmartReflex Driver: Initializing\n");

#ifdef CONFIG_ARCH_OMAP34XX
	sr1.fck = clk_get(NULL, "sr1_fck");
	if (IS_ERR(sr1.fck)) {
		panic("Could not get sr1_fck\n");
	}

	sr2.fck = clk_get(NULL, "sr2_fck");
	if (IS_ERR(sr2.fck)) {
		panic("Could not get sr2_fck\n");
	}
#endif /* #ifdef CONFIG_ARCH_OMAP34XX */

	/* set nvalues */
	sr_nvalues = sr_set_efuse_nvalues();
	if (sr_nvalues == 0x0) {
		sr_nvalues = sr_set_test_nvalues();
	}

	/* Call the VPConfig, VCConfig */
	sr_configure_vp(SR1);
	sr_configure_vp(SR2);

	sr_configure_vc();

#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
	sr15_invalidate_opps(&sr1);
	sr15_invalidate_opps(&sr2);
#endif

#ifdef CONFIG_TWL4030_CORE
	/* Enable SR on T2 */
	ret = t2_in(PM_RECEIVER, &RdReg, R_DCDC_GLOBAL_CFG);
	RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
	ret |= t2_out(PM_RECEIVER, RdReg, R_DCDC_GLOBAL_CFG);
	if (ret) {
		panic("Error: Triton-SR mode not set : %d\n", ret);
	}
#else
/* Add appropriate power IC function call to Enable SR on T2 */
#endif /* CONFIG_TWL4030_CORE */

	ret = subsys_create_file(&power_subsys, &sr_vdd1_autocomp);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_vdd2_autocomp);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_setnvalues_test);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_vp1errgain);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_vp2errgain);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_sr1errminlimit);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_sr2errminlimit);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &vdd1_vsel);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &vdd2_vsel);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

#ifdef CONFIG_OMAP_SMARTREFLEX_v15 
	ret = subsys_create_file(&power_subsys, &sr15_period);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr15_vdd1_opp_state);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr15_vdd2_opp_state);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);
#endif

	printk(KERN_INFO "SmartReflex Driver: Initialized\n");

	return 0;
}

device_initcall(omap3_sr_init);
