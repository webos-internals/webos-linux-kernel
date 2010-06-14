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
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

struct omap_sr{
	int srid;
	int is_sr_reset;
	int is_autocomp_active;
	struct clk *fck;
	u32 req_opp_no;
	u32 opp1_nvalue, opp2_nvalue, opp3_nvalue, opp4_nvalue, opp5_nvalue;
	u32 senp_mod, senn_mod;
	u32 srbase_addr;
	u32 vpbase_addr;
};

static struct omap_sr sr1 = {
	.srid = SR1,
	.is_sr_reset = 1,
	.is_autocomp_active = 0,
	.srbase_addr = OMAP34XX_SR1_BASE,
};

static struct omap_sr sr2 = {
	.srid = SR2,
	.is_sr_reset = 1,
	.is_autocomp_active = 0,
	.srbase_addr = OMAP34XX_SR2_BASE,
};

static inline void sr_write_reg(struct omap_sr *sr, int offset, u32 value)
{
	omap_writel(value, sr->srbase_addr + offset);
}

static inline void sr_modify_reg(struct omap_sr *sr, int offset, u32 mask,
								u32 value)
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


static int sr_clk_enable(struct omap_sr *sr)
{
	if (clk_enable(sr->fck) != 0) {
		printk(KERN_ERR "Could not enable sr%d_fck\n", sr->srid);
		goto clk_enable_err;
	}

	/* set fclk- active , iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
						SR_CLKACTIVITY_IOFF_FON);

	return 0;

clk_enable_err:
	return -1;
}

static int sr_clk_disable(struct omap_sr *sr)
{
	/* set fclk, iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
						SR_CLKACTIVITY_IOFF_FOFF);

	clk_disable(sr->fck);
	sr->is_sr_reset = 1;

	return 0;
}

static void sr_configure(struct omap_sr *sr)
{
	u32 sys_clk, sr_clk_length = 0;
	u32 sr_config;
	u32 senp_en , senn_en;

	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;

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

	DPRINTK(KERN_DEBUG "SR : sys clk %lu\n", sys_clk);
	if (sr->srid == SR1) {
		sr_config = SR1_SRCONFIG_ACCUMDATA |
			(sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);

		sr_write_reg(sr, AVGWEIGHT, SR1_AVGWEIGHT_SENPAVGWEIGHT |
					SR1_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT | SR1_ERRMINLIMIT));

	} else if (sr->srid == SR2) {
		sr_config = SR2_SRCONFIG_ACCUMDATA |
			(sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);

		sr_write_reg(sr, AVGWEIGHT, SR2_AVGWEIGHT_SENPAVGWEIGHT |
					SR2_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT | SR2_ERRMINLIMIT));

	}
	sr->is_sr_reset = 0;
}

static void sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 nvalue_reciprocal, current_nvalue;

	sr->req_opp_no = target_opp_no;

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

	current_nvalue = sr_read_reg(sr, NVALUERECIPROCAL);

	if (current_nvalue == nvalue_reciprocal) {
		DPRINTK("System is already at the desired voltage level\n");
		return;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

	/* Enable the interrupt */
	sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST),
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST));

	if (sr->srid == SR1) {
		/* Enable VP1 */
		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_VPENABLE;
	} else if (sr->srid == SR2) {
		/* Enable VP2 */
		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_VPENABLE;
	}

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);

}

static void sr_disable(struct omap_sr *sr)
{
	sr->is_sr_reset = 1;

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, ~SRCONFIG_SRENABLE);

	if (sr->srid == SR1) {
		/* Enable VP1 */
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_VPENABLE;
	} else if (sr->srid == SR2) {
		/* Enable VP2 */
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_VPENABLE;
	}
}

void sr_start_vddautocomap(int srid, u32 target_opp_no)
{
	struct omap_sr *sr;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	} else {
		return; /* invalid srid. */
	}

	if (sr->is_sr_reset == 1) {
		sr_clk_enable(sr);
		sr_configure(sr);
	}

	if (sr->is_autocomp_active == 1)
		DPRINTK(KERN_WARNING "SR%d: VDD autocomp is already active\n",
									srid);

	sr->is_autocomp_active = 1;
	sr_enable(sr, target_opp_no);
}
EXPORT_SYMBOL(sr_start_vddautocomap);

int sr_stop_vddautocomap(int srid)
{
	struct omap_sr *sr;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	} else {
		return -1; /* invalid srid. */
	}

	if (sr->is_autocomp_active == 1) {
		sr_disable(sr);
		sr_clk_disable(sr);
		sr->is_autocomp_active = 0;
		return SR_TRUE;
	} else {
		DPRINTK(KERN_WARNING "SR%d: VDD autocomp is not active\n", srid);
		return SR_FALSE;
	}

}
EXPORT_SYMBOL(sr_stop_vddautocomap);

void enable_smartreflex(int srid)
{
	u32 target_opp_no = 0;
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 1) {
			if (srid == SR1) {
				/* Enable SR clks */
				CM_FCLKEN_WKUP |= SR1_CLK_ENABLE;
				target_opp_no = get_opp_no(prcm_get_current_vdd1_opp());

			} else if (srid == SR2) {
				/* Enable SR clks */
				CM_FCLKEN_WKUP |= SR2_CLK_ENABLE;
				target_opp_no = get_opp_no(prcm_get_current_vdd2_opp());
			}

			sr_configure(sr);

			sr_enable(sr, target_opp_no);
		}
	}
}

void disable_smartreflex(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}
	else {
		return; /* invalid srid. */
	}

	if (sr->is_autocomp_active == 1) {
		if (srid == SR1) {
			/* Enable SR clk */
			CM_FCLKEN_WKUP |= SR1_CLK_ENABLE;

		} else if (srid == SR2) {
			/* Enable SR clk */
			CM_FCLKEN_WKUP |= SR2_CLK_ENABLE;
		}

		if (sr->is_sr_reset == 0) {

			sr->is_sr_reset = 1;
			/* SRCONFIG - disable SR */
			sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
							~SRCONFIG_SRENABLE);

			if (sr->srid == SR1) {
				/* Disable SR clk */
				CM_FCLKEN_WKUP &= ~SR1_CLK_ENABLE;
				/* Enable VP1 */
				PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_VPENABLE;

			} else if (sr->srid == SR2) {
				/* Disable SR clk */
				CM_FCLKEN_WKUP &= ~SR2_CLK_ENABLE;
				/* Enable VP2 */
				PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_VPENABLE;
			}
		}
	}
}

#ifdef CONFIG_OMAP_VOLT_SR_BYPASS
/* Voltage Scaling using SR VCBYPASS */
int sr_voltagescale_vcbypass(u32 target_opp, u8 vsel)
{
	int rc;
	u32 vdd;
	u32 target_opp_no;
	u32 vc_bypass_value;
	u32 reg_addr = 0;
	int sr_status = 0;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);

	if (vdd == PRCM_VDD1) {
		sr_status = sr_stop_vddautocomap(SR1);

		PRM_VC_CMD_VAL_0 = (PRM_VC_CMD_VAL_0 & ~PRM_VC_CMD_ON_MASK) |
						(vsel << PRM_VC_CMD_ON_SHIFT);
		reg_addr = R_VDD1_SR_CONTROL;

	} else if (vdd == PRCM_VDD2) {
		sr_status = sr_stop_vddautocomap(SR2);

		PRM_VC_CMD_VAL_1 = (PRM_VC_CMD_VAL_1 & ~PRM_VC_CMD_ON_MASK) |
						(vsel << PRM_VC_CMD_ON_SHIFT);
		reg_addr = R_VDD2_SR_CONTROL;
	} else {
		printk(KERN_WARNING
			"Invalid VDD (%d) in sr_stop_vddautocomap().\n", vdd);
		return PRCM_FAIL;
	}

	vc_bypass_value = (vsel << PRM_VC_BYPASS_DATA_SHIFT) |
			  (reg_addr << PRM_VC_BYPASS_REGADDR_SHIFT) |
			  (R_SRI2C_SLAVE_ADDR << PRM_VC_BYPASS_SLAVEADDR_SHIFT);

	PRM_VC_BYPASS_VAL = vc_bypass_value;

	PRM_VC_BYPASS_VAL |= PRM_VC_BYPASS_VALID;

	DPRINTK("%s : PRM_VC_BYPASS_VAL %X\n", __FUNCTION__, PRM_VC_BYPASS_VAL);
	DPRINTK("PRM_IRQST_MPU %X\n", PRM_IRQSTATUS_MPU);


	rc = WAIT_WHILE((PRM_VC_BYPASS_VAL & PRM_VC_BYPASS_VALID) != 0x0, 300);
	if (rc < 0) {
		printk(KERN_ERR "Timeout in check SR I2C write, "
				"target_opp %d\n", target_opp);
		return PRCM_FAIL;
	}

	udelay(T2_SMPS_UPDATE_DELAY);

	if (sr_status) {
		if (vdd == PRCM_VDD1)
			sr_start_vddautocomap(SR1, target_opp_no);
		else if (vdd == PRCM_VDD2)
			sr_start_vddautocomap(SR2, target_opp_no);
	}

	return SR_PASS;
}
#endif

/******************************************************************************
 *
 * Functions needed only if Power Management is selected
 *
 ******************************************************************************/
#ifdef CONFIG_PM

#ifndef USE_EFUSE_VALUES
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
#endif

static void sr_set_nvalues(struct omap_sr *sr)
{
#ifdef USE_EFUSE_VALUES
	u32 n1, n2;
#else
	u32 senpval, sennval;
	u32 senpgain, senngain;
	u32 rnsenp, rnsenn;
#endif

	if (sr->srid == SR1) {
#ifdef USE_EFUSE_VALUES
		/* Read values for VDD1 from EFUSE */
#else
		/* since E-Fuse Values are not available, calculating the
		 * reciprocal of the SenN and SenP values for SR1
		 */
		sr->senp_mod = 0x03;        /* SenN-M5 enabled */
		sr->senn_mod = 0x03;

		/* for OPP5 */
		senpval = 0x848 + 0x330;
		sennval = 0xacd + 0x330;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp5_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP4 */
		senpval = 0x727 + 0x2a0;
		sennval = 0x964 + 0x2a0;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp4_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP3 */
		senpval = 0x655 + 0x200;
		sennval = 0x85b + 0x200;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp3_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP2 */
		senpval = 0x3be + 0x1a0;
		sennval = 0x506 + 0x1a0;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp2_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP1 */
		senpval = 0x28c + 0x100;
		sennval = 0x373 + 0x100;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp1_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		sr_clk_enable(sr);
		sr_write_reg(sr, NVALUERECIPROCAL, sr->opp3_nvalue);
		sr_clk_disable(sr);

#endif
	} else if (sr->srid == SR2) {
#ifdef USE_EFUSE_VALUES
		/* Read values for VDD2 from EFUSE */
#else
		/* since E-Fuse Values are not available, calculating the
		 * reciprocal of the SenN and SenP values for SR2
		 */
		sr->senp_mod = 0x03;
		sr->senn_mod = 0x03;

		/* for OPP3 */
		senpval = 0x579 + 0x200;
		sennval = 0x76f + 0x200;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp3_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP2 */
		senpval = 0x390 + 0x1c0;
		sennval = 0x4f5 + 0x1c0;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp2_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

		/* for OPP1 */
		senpval = 0x25d;
		sennval = 0x359;

		cal_reciprocal(senpval, &senpgain, &rnsenp);
		cal_reciprocal(sennval, &senngain, &rnsenn);

		sr->opp1_nvalue =
				((senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
				(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
				(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
				(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT));

#endif
	}

}

static void sr_configure_vp(int srid)
{
	u32 vpconfig;

	if (srid == SR1) {
		vpconfig = PRM_VP1_CONFIG_ERROROFFSET | PRM_VP1_CONFIG_ERRORGAIN
			| PRM_VP1_CONFIG_INITVOLTAGE | PRM_VP1_CONFIG_TIMEOUTEN;

		PRM_VP1_CONFIG = vpconfig;
		PRM_VP1_VSTEPMIN = PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN |
						PRM_VP1_VSTEPMIN_VSTEPMIN;

		PRM_VP1_VSTEPMAX = PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX |
						PRM_VP1_VSTEPMAX_VSTEPMAX;

		PRM_VP1_VLIMITTO = PRM_VP1_VLIMITTO_VDDMAX |
			PRM_VP1_VLIMITTO_VDDMIN | PRM_VP1_VLIMITTO_TIMEOUT;

		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_INITVDD;
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_INITVDD;

	} else if (srid == SR2) {
		vpconfig = PRM_VP2_CONFIG_ERROROFFSET | PRM_VP2_CONFIG_ERRORGAIN
			| PRM_VP2_CONFIG_INITVOLTAGE | PRM_VP2_CONFIG_TIMEOUTEN;

		PRM_VP2_CONFIG = vpconfig;
		PRM_VP2_VSTEPMIN = PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN |
						PRM_VP2_VSTEPMIN_VSTEPMIN;

		PRM_VP2_VSTEPMAX = PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX |
						PRM_VP2_VSTEPMAX_VSTEPMAX;

		PRM_VP2_VLIMITTO = PRM_VP2_VLIMITTO_VDDMAX |
			PRM_VP2_VLIMITTO_VDDMIN | PRM_VP2_VLIMITTO_TIMEOUT;

		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_INITVDD;
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_INITVDD;

	}
}

static void sr_configure_vc(void)
{
	PRM_VC_SMPS_SA = (R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA1_SHIFT) |
			 (R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA0_SHIFT);

	PRM_VC_SMPS_VOL_RA = (R_VDD2_SR_CONTROL << PRM_VC_SMPS_VOLRA1_SHIFT) |
			     (R_VDD1_SR_CONTROL << PRM_VC_SMPS_VOLRA0_SHIFT);

	PRM_VC_CMD_VAL_0 = (PRM_VC_CMD_VAL0_ON   << PRM_VC_CMD_ON_SHIFT)   |
			   (PRM_VC_CMD_VAL0_ONLP << PRM_VC_CMD_ONLP_SHIFT) |
			   (PRM_VC_CMD_VAL0_RET  << PRM_VC_CMD_RET_SHIFT)  |
			   (PRM_VC_CMD_VAL0_OFF  << PRM_VC_CMD_OFF_SHIFT);

	PRM_VC_CMD_VAL_1 = (PRM_VC_CMD_VAL1_ON   << PRM_VC_CMD_ON_SHIFT)   |
			   (PRM_VC_CMD_VAL1_ONLP << PRM_VC_CMD_ONLP_SHIFT) |
			   (PRM_VC_CMD_VAL1_RET  << PRM_VC_CMD_RET_SHIFT)  |
			   (PRM_VC_CMD_VAL1_OFF  << PRM_VC_CMD_OFF_SHIFT);

	PRM_VC_CH_CONF = PRM_VC_CH_CONF_CMD1 | PRM_VC_CH_CONF_RAV1;

#if 0 /* Original TI drop value. */
	PRM_VC_I2C_CFG = PRM_VC_I2C_CFG_MCODE |
			 PRM_VC_I2C_CFG_HSEN |
			 PRM_VC_I2C_CFG_SREN;

#else /* David Derreck's value. */
	PRM_VC_I2C_CFG = PRM_VC_I2C_CFG_MCODE;
#endif

	/* Setup voltctrl and other setup times */
#ifdef CONFIG_SYSOFFMODE
	PRM_VOLTCTRL   = PRM_VOLTCTRL_AUTO_OFF | PRM_VOLTCTRL_AUTO_RET;
	PRM_CLKSETUP   = PRM_CLKSETUP_DURATION;
	PRM_VOLTSETUP1 = (PRM_VOLTSETUP_TIME2 << PRM_VOLTSETUP_TIME2_OFFSET) |
			 (PRM_VOLTSETUP_TIME1 << PRM_VOLTSETUP_TIME1_OFFSET);
	PRM_VOLTOFFSET = PRM_VOLTOFFSET_DURATION;
	PRM_VOLTSETUP2 = PRM_VOLTSETUP2_DURATION;
#else
	PRM_VOLTCTRL  |= PRM_VOLTCTRL_AUTO_RET;
#endif
}

/* Sysfs interface to select SR VDD1 auto compensation */
static ssize_t omap_sr_vdd1_autocomp_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%d\n", sr1.is_autocomp_active);
}

static ssize_t omap_sr_vdd1_autocomp_store(struct kset *subsys,
				const char *buf, size_t n)
{
	u32 current_vdd1opp_no;
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_vdd1_autocomp: Invalid value\n");
		return -EINVAL;
	}

	current_vdd1opp_no = get_opp_no(prcm_get_current_vdd1_opp());

	if (value == 0) {
		sr_stop_vddautocomap(SR1);
	} else {
		sr_start_vddautocomap(SR1, current_vdd1opp_no);
	}

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
	return sprintf(buf, "%d\n", sr2.is_autocomp_active);
}

static ssize_t omap_sr_vdd2_autocomp_store(struct kset *subsys,
				const char *buf, size_t n)
{
	u32 current_vdd2opp_no;
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_vdd2_autocomp: Invalid value\n");
		return -EINVAL;
	}

	current_vdd2opp_no = get_opp_no(prcm_get_current_vdd2_opp());

	if (value == 0) {
		sr_stop_vddautocomap(SR2);
	} else {
		sr_start_vddautocomap(SR2, current_vdd2opp_no);
	}

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

static int __init omap3_sr_init(void)
{
	int ret = 0;
	u8 RdReg;

	printk("OMAP: Initializing SmartReflex subsystem...\n");

#ifdef CONFIG_ARCH_OMAP34XX
	sr1.fck = clk_get(NULL, "sr1_fck");
	if (IS_ERR(sr1.fck))
		printk(KERN_ERR "Could not get sr1_fck\n");

	sr2.fck = clk_get(NULL, "sr2_fck");
	if (IS_ERR(sr2.fck))
		printk(KERN_ERR "Could not get sr2_fck\n");
#endif

	/* Call the VPConfig, VCConfig, set N Values. */
	sr_set_nvalues(&sr1);
	sr_configure_vp(SR1);

	sr_set_nvalues(&sr2);
	sr_configure_vp(SR2);

	sr_configure_vc();

	/* Enable SR on T2 */
	ret = t2_in(PM_RECEIVER, &RdReg, R_DCDC_GLOBAL_CFG);
	RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
	ret |= t2_out(PM_RECEIVER, RdReg, R_DCDC_GLOBAL_CFG);
	if (ret) {
		printk(KERN_ERR
			"OMAP: ERROR initializing SmartReflex subsystem.\n");
		return -1;
	}

	printk(KERN_INFO "OMAP: SmartReflex subsystem initialized.\n");

	ret = subsys_create_file(&power_subsys, &sr_vdd1_autocomp);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	ret = subsys_create_file(&power_subsys, &sr_vdd2_autocomp);
	if (ret)
		printk(KERN_ERR "subsys_create_file failed: %d\n", ret);

	return 0;
}

device_initcall(omap3_sr_init);
#endif /* CONFIG_PM */
