/*
 * linux/drivers/sound/omap-audio-twl4030.c
 *
 * Codec driver for TWL4030 for OMAP processors
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
 *
 * History:
 *  -------
 *  2006-01-18 Nishanth Menon - Created
 */

/***************************** INCLUDES ************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>

#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/arch/dma.h>
#include <asm/io.h>
#include <asm/hardware.h>

#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/mach-types.h>

#include "omap2-audio.h"
#if defined(CONFIG_ARCH_OMAP243X) || defined(CONFIG_ARCH_OMAP3430)
#include <asm/arch/omap2_mcbsp.h>
#include <asm/arch/clock.h>
#else
#error "Unsupported configuration"
#endif
#include <asm/arch/twl4030.h>
#include "omap2-audio-twl4030.h"

/*********** Debug Macros ********/
/* Change to define if required */
/* To Generate a rather shrill tone -test the entire path */
#undef TONE_GEN
/* To dump the twl registers for debug */
#undef TWL_DUMP_REGISTERS
#undef TWL_DUMP_REGISTERS_MCBSP
#undef DEBUG

#undef DPRINTK
#ifdef DEBUG
#define DPRINTK(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#define FN_IN printk(KERN_INFO "[%s]: start\n", __FUNCTION__)
#define FN_OUT(n) printk(KERN_INFO "[%s]: end(%u)\n",__FUNCTION__, n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(n)

#endif				/* DEBUG */

#ifdef TWL_DUMP_REGISTERS
static void twl4030_dumpRegisters(void);
#endif
#ifdef TONE_GEN
void toneGen(void);
#endif

/****************                                          **************
 * 
 ***************************** COMM APIS ********************************
 *
 */

/** 
 * @brief audio_twl4030_write
 * 
 * @param address 
 * @param data 
 * 
 * @return  0 if successful
 */
static inline int audio_twl4030_write(u8 address, u8 data)
{
	int ret = 0;
	/* DEBUG */
	DPRINTK("add[0x%02x]<=0x%02x\n", address, data);
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE, data, address);
	if (ret >= 0) {
#if 0
		/* PARANOIA Check - what if we wrote and it did not Go there?? */
		u8 data1;
		ret = twl4030_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &data1, address);
		if (ret < 0) {
			printk(KERN_ERR "re-read failed\n");
		} else {
			if (data1 != data) {
				printk
				    ("Write mismatch - wrote %x read %x"
				     " from reg %x\n", data, data1, address);
			}
		}
#endif
		ret = 0;
	} else {
		DPRINTK("TWL4030:Audio:Write[0x%x]"
			" Error %d\n", address, ret);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief audio_twl4030_read
 * 
 * @param address 
 * 
 * @return  data if successful else return negative error value
 */
static inline int audio_twl4030_read(u8 address)
{
	u8 data;
	int ret = 0;
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, &data, address);
	if (ret >= 0) {
		ret = data;
	} else {
		DPRINTK("TWL4030:Audio:Read[0x%x] Error %d\n", address,
			ret);
	}
	return ret;
}

/****************                                          **************
 * 
 ***************************** CODEC UTIL APIS **************************
 *
 */

/** 
 * @brief twl4030_ext_mut_conf - configure GPIO for data out
 * 
 * @return error in case of failure else 0
 */
static inline int twl4030_ext_mut_conf(void)
{
	int ret;
	u8 data;
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data , GPIO_DATA_DIR); 
	if (ret) 
		return ret;
	data |= 0x1<< T2_AUD_EXT_MUT_GPIO;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data , GPIO_DATA_DIR);
	return ret;
}

/** 
 * @brief twl4030_ext_mut_off - disable mute also handle time of wait
 * 
 * @return error in case of failure else 0
 */
static inline int twl4030_ext_mut_off(void)
{
	int ret;
	u8 data;
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data , GPIO_CLR); 
	if (ret) 
		return ret;
	/* Wait for ramp duration.. settling time for signal */
	udelay(1);
	data |= 0x1<< T2_AUD_EXT_MUT_GPIO;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, GPIO_CLR); //Clear mute
	return ret;
}

/** 
 * @brief twl4030_ext_mut_on - enable mute
 * 
 * @return error in case of failure else 0
 */
static inline int twl4030_ext_mut_on(void)
{
	int ret;
	u8 data;
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data , GPIO_SET); 
	if (ret) 
		return ret;
	data |= 0x1<< T2_AUD_EXT_MUT_GPIO;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, GPIO_SET); //Set mute
	return ret;
}
/** 
 * @brief twl4030_codec_on
 * 
 * @return 0 if success, else error value
 */
static inline int twl4030_codec_on(void)
{
	int data = audio_twl4030_read(REG_CODEC_MODE);
	FN_IN;
	if (unlikely(data < 0)) {
		printk(KERN_ERR "Reg read failed\n");
		return data;
	}
	if (unlikely(data & BIT_CODEC_MODE_CODECPDZ_M)) {
		DPRINTK("Codec already powered on??\n");
		FN_OUT(0);
		return 0;
	}
	data |= BIT_CODEC_MODE_CODECPDZ_M;
	FN_OUT(0);
	return audio_twl4030_write(REG_CODEC_MODE, (u8) data);
}

/** 
 * @brief twl4030_codec_off - Switch off the codec
 * 
 * @return 0 if success, else error value
 */
static inline int twl4030_codec_off(void)
{
	int data = audio_twl4030_read(REG_CODEC_MODE);
	FN_IN;
	if (unlikely(data < 0)) {
		printk(KERN_ERR "Reg read failed\n");
		return data;
	}
	/* Expected to be already off at bootup - but
	 * we do not know the status of the device 
	 * hence would like to force a shut down
	 */
	if (unlikely(!(data & BIT_CODEC_MODE_CODECPDZ_M))) {
		DPRINTK("Codec already powered off??\n");
		FN_OUT(0);
		return 0;
	}
	data &= ~BIT_CODEC_MODE_CODECPDZ_M;
	FN_OUT(0);
	return audio_twl4030_write(REG_CODEC_MODE, (u8) data);
}

/** 
 * @brief twl4030_codec_tog_on - set the power to on after toggle to off 
 *                               and then on
 * 
 * @return 0 if success, else error value
 */
static int twl4030_codec_tog_on(void)
{
	int ret = 0;
	int data = audio_twl4030_read(REG_CODEC_MODE);
	FN_IN;
	if (unlikely(data < 0)) {
		printk(KERN_ERR "Reg read failed\n");
		return data;
	}
	data &= ~BIT_CODEC_MODE_CODECPDZ_M;
	ret =  audio_twl4030_write(REG_CODEC_MODE, (u8) data);
	if (ret) {
		printk(KERN_ERR "CODEC WRITE failed ! %d\n",ret);
		FN_OUT(ret);
		return ret;
	}
	udelay(10); /* 10 ms delay for power settling */
	data |= BIT_CODEC_MODE_CODECPDZ_M;
	ret =  audio_twl4030_write(REG_CODEC_MODE, (u8) data);
	udelay(10); /* 10 ms delay for power settling */
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl_mcbsp_irq - the Interrupt handler for mcbsp events
 * 
 * @param arg  - ignored
 * @param irq_status -mcbsp interrupt status
 * @param regs -ignored
 */
static void twl_mcbsp_irq(void *arg, u32 irq_status )
{
#ifdef DEBUG
	printk(KERN_ERR "ERROR IN TRANSFER 0x%08x ", irq_status);
	if (irq_status & OMAP2_MCBSP_IRQSTAT_XOVFL) {
		printk(KERN_INFO "XOVFLSTAT ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_XUNDFL) {
		printk(KERN_INFO "XUNDFLFSTAT ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_XRDY) {
		printk(KERN_INFO "XRDY ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_XEOF) {
		printk(KERN_INFO "XEOF ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_XFSX) {
		printk(KERN_INFO "XFSX ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_XSYNCERR) {
		printk(KERN_INFO "XSYNCERR ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_ROVFL) {
		printk(KERN_INFO "ROVFLSTAT ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_RUNDFL) {
		printk(KERN_INFO "RUNDFLFSTAT ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_RRDY) {
		printk(KERN_INFO "RRDY ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_REOF) {
		printk(KERN_INFO "REOF ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_RFSX) {
		printk(KERN_INFO "RFSR ");
	}
	if (irq_status & OMAP2_MCBSP_IRQSTAT_RSYNCERR) {
		printk(KERN_INFO "RSYNCERR ");
	}
	printk(KERN_INFO "\n");
#endif
	/* In case of errors - we need to reset the reciever to ensure
	 * recovery 
	 */
	if ((omap_twl_state.fmode & FMODE_WRITE)
	    && (irq_status &
		(OMAP2_MCBSP_IRQSTAT_XSYNCERR | OMAP2_MCBSP_IRQSTAT_XUNDFL |
		 OMAP2_MCBSP_IRQSTAT_XOVFL))) {
		/* We will handle this when we attempt a transfer */
		tx_err = 1;
	}
	if ((omap_twl_state.fmode & FMODE_READ)
	    && (irq_status &
		(OMAP2_MCBSP_IRQSTAT_RSYNCERR | OMAP2_MCBSP_IRQSTAT_RUNDFL |
		 OMAP2_MCBSP_IRQSTAT_ROVFL))) {
		/* We will handle this when we attempt a transfer */
		rx_err = 1;
	}
}

/** 
 * @brief twl4030_enable_output - enable the output path
 *  * NOTE * Codec power must be shut down during before this call
 *  * NOTE * This does not take care of gain settings
 *           for the specified output
 * @return 0 if successful
 */
static int twl4030_enable_output(void)
{
	u8 ear_ctl = 0;
	u8 hs_ctl = 0;
	u8 hs_pop = 0;
	u8 hf_ctll = 0;
	u8 hf_ctlr = 0;
	u8 dac_ctl = 0;
	u8 opt = 0;
	u32 line = 0;
	int ret = 0;
	FN_IN;
	opt =
	    audio_twl4030_read(REG_OPTION) & ~(BIT_OPTION_ARXR2_EN_M |
					       BIT_OPTION_ARXL2_EN_M);

	/* AR2 and AL2 are active for I2S */
	if ((current_output & OUTPUT_STEREO_HEADSET) == OUTPUT_STEREO_HEADSET) {
		hs_ctl |= BIT_HS_SEL_HSOR_AR2_EN_M | BIT_HS_SEL_HSOL_AL2_EN_M;
		/* POP control - VMID? */
		hs_pop = BIT_HS_POPN_SET_VMID_EN_M;
		dac_ctl = BIT_AVDAC_CTL_ADACL2_EN_M | BIT_AVDAC_CTL_ADACR2_EN_M;
		opt |= BIT_OPTION_ARXR2_EN_M | BIT_OPTION_ARXL2_EN_M;
	}
	if ((current_output & OUTPUT_HANDS_FREE_CLASSD) ==
	    OUTPUT_HANDS_FREE_CLASSD) {
		hf_ctll |=
		    HANDS_FREEL_AL2 << BIT_HFL_CTL_HFL_INPUT_SEL |
		    BIT_HFL_CTL_HFL_REF_EN_M;
		hf_ctlr |=
		    HANDS_FREER_AR2 << BIT_HFR_CTL_HFR_INPUT_SEL |
		    BIT_HFR_CTL_HFR_REF_EN_M;
		dac_ctl = BIT_AVDAC_CTL_ADACL2_EN_M | BIT_AVDAC_CTL_ADACR2_EN_M;
		opt |= BIT_OPTION_ARXR2_EN_M | BIT_OPTION_ARXL2_EN_M;
	}
	if ((current_output & OUTPUT_MONO_EARPIECE) == OUTPUT_MONO_EARPIECE) {
		/* only AL2 comes in case of i2s */
		ear_ctl |= BIT_EAR_CTL_EAR_AL2_EN_M;
		dac_ctl = BIT_AVDAC_CTL_ADACL2_EN_M;
		opt |= BIT_OPTION_ARXL2_EN_M;
	}
	if (opt) {
		ret = audio_twl4030_write(REG_OPTION, opt);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (ear_ctl) {
		ret = audio_twl4030_write(REG_EAR_CTL, ear_ctl);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (hs_ctl) {
		ret = audio_twl4030_write(REG_HS_SEL, hs_ctl);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (hs_pop) {
		/* IMPORTANT: The following sequence is *required*
		 * for starting the headset- esp for 
		 * ensuring the existance of the negative phase of
		 * analog signal
		 */
		ret = audio_twl4030_write(REG_HS_POPN_SET, hs_pop);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hs_pop |= BIT_HS_POPN_SET_RAMP_EN_M;
		udelay(1); /* require a short delay before enabling ramp */
		ret = audio_twl4030_write(REG_HS_POPN_SET, hs_pop);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	/* IMPORTANT: The following sequence is *required*
	 * for starting the speakers!
	 */
	if (hf_ctll) {
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctll |= BIT_HFL_CTL_HFL_RAMP_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctll |= BIT_HFL_CTL_HFL_LOOP_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctll |= BIT_HFL_CTL_HFL_HB_EN_M;
		ret = audio_twl4030_write(REG_HFL_CTL, hf_ctll);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (hf_ctlr) {
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctlr |= BIT_HFR_CTL_HFR_RAMP_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctlr |= BIT_HFR_CTL_HFR_LOOP_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
		hf_ctlr |= BIT_HFR_CTL_HFR_HB_EN_M;
		ret = audio_twl4030_write(REG_HFR_CTL, hf_ctlr);
		if (ret) {
			line = __LINE__;
			goto enable_op_exit;
		}
	}
	if (dac_ctl) {
		/* I2S should go thru DACR2, DACL2- unless we are on mono */
		if (current_stereomode == MONO_MODE) {
			dac_ctl &= ~BIT_AVDAC_CTL_ADACR2_EN_M;
		}
		ret = audio_twl4030_write(REG_AVDAC_CTL, dac_ctl);
	}
      enable_op_exit:
	if (ret)
		printk(KERN_ERR "Error in Enable output[%d] in Line %d\n", ret,
		       line);
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_disable_output - remove the output path
 * *NOTE* shut down the codec before attempting this
 * 
 * @return 0 if successful
 */
static int twl4030_disable_output(void)
{
	int ret = 0;
	int line = 0;
	FN_IN;
	ret = audio_twl4030_write(REG_PRECKR_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_PRECKL_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_PREDR_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_PREDL_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_EAR_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_HS_SEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_HFL_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_op_exit;
	}
	ret = audio_twl4030_write(REG_HFR_CTL, 0x0);
      disable_op_exit:
	if (ret)
		printk(KERN_ERR "Disable Output Error [%d] in Line %d\n", ret,
		       line);
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_enable_input - enable the input to the correct device
 *  Enable all the ADCs and path settings
 *  *NOTE* This will not set the gain
 *  Reason being that the gain setting can be done with CODEC power down
 *  *NOTE* This should be called with codec power down
 *
 * @return 0 if success else error value<0
 */
static int twl4030_enable_input(void)
{
	u8 micbias_ctl = 0;
	u8 mic_en1 = 0;
	u8 mic_en2 = 0;
	u8 adc = 0;
	int ret = 0;
	u8 opt_ip = 0;
	int line = 0;
	FN_IN;
	ret = audio_twl4030_read(REG_OPTION);
	if (ret < 0)
		goto enable_ip_exit;

	/* Dont use path1/path2 left and right.. we will enable it later.. */
	opt_ip =
	    (ret &
	     ~(BIT_OPTION_ATXL1_EN_M | BIT_OPTION_ATXR1_EN_M |
	       BIT_OPTION_ATXL2_VTXL_EN_M | BIT_OPTION_ATXR2_VTXR_EN_M));

	DPRINTK("Enable Input 0x%x HS=0x%x MM=0x%x SM=0x%x\n", current_input,
		INPUT_HEADSET_MIC, INPUT_MAIN_MIC, INPUT_SUB_MIC);
	/* HS MIC */
	if ((current_input & INPUT_HEADSET_MIC) == INPUT_HEADSET_MIC) {
		/* mono Path */
		micbias_ctl |= BIT_MICBIAS_CTL_HSMICBIAS_EN_M;
		mic_en1 |= BIT_ANAMICL_HSMIC_EN_M | BIT_ANAMICL_MICAMPL_EN_M;
		adc |= BIT_AVADC_CTL_ADCL_EN_M;
		opt_ip |= BIT_OPTION_ATXL1_EN_M;
	}
	/* Main Mic */
	if ((current_input & INPUT_MAIN_MIC) == INPUT_MAIN_MIC) {
		micbias_ctl |= BIT_MICBIAS_CTL_MICBIAS1_EN_M;
		mic_en1 |= BIT_ANAMICL_MAINMIC_EN_M | BIT_ANAMICL_MICAMPL_EN_M;
		adc |= BIT_AVADC_CTL_ADCL_EN_M;
		opt_ip |= BIT_OPTION_ATXL1_EN_M;
	}
	/* Sub Mic */
	if ((current_input & INPUT_SUB_MIC) == INPUT_SUB_MIC) {
		micbias_ctl |= BIT_MICBIAS_CTL_MICBIAS2_EN_M;
		mic_en2 |= BIT_ANAMICR_SUBMIC_EN_M | BIT_ANAMICR_MICAMPR_EN_M;
		adc |= BIT_AVADC_CTL_ADCR_EN_M;
		opt_ip |= BIT_OPTION_ATXR1_EN_M;
	}
	DPRINTK("mic_biasctl=0x%x en1=%x en2=%x adc=%x\n", micbias_ctl, mic_en1,
		mic_en2, adc);
	ret = audio_twl4030_write(REG_OPTION, opt_ip);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_MICBIAS_CTL, micbias_ctl);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICL, mic_en1);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICR, mic_en2);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_AVADC_CTL, adc);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	/* use ADC TX2 routed to digi mic - we dont use tx2 path
	 * -route it to digi mic1 
	 */
	ret = audio_twl4030_write(REG_ADCMICSEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto enable_ip_exit;
	}
	ret = audio_twl4030_write(REG_DIGMIXING, 0x0);	/* No Karaoke pls */
      enable_ip_exit:
	if (ret)
		printk(KERN_ERR "Error In Enable input[%d] in Line %d\n", ret,
		       line);
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_disable_input - reset all the inputs
 *  *NOTE* This should be called with codec power down
 * 
 * @return  0 if successful
 */
static int twl4030_disable_input(void)
{
	int ret = 0;
	int line = 0;
	FN_IN;
	/* lets shut up all devices */
	ret = audio_twl4030_write(REG_MICBIAS_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ANAMICR, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_AVADC_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_ADCMICSEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto disable_ip_exit;
	}
	ret = audio_twl4030_write(REG_DIGMIXING, 0x0);
      disable_ip_exit:
	if (ret)
		printk(KERN_ERR "Error in disable of input [%d] in Line %d\n",
		       ret, line);
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_select_source - set up proper source
 * Call me with the Codec powered down
 * @param flag 
 * @param val 
 * 
 * @return  0 if successful
 */
static int twl4030_select_source(int flag, int val)
{
	int ret = 0;
	int temp = 0;
	FN_IN;
	switch (flag) {
	case DIR_OUT:
		/*
		 * If more than one play device selected,
		 * disable the device that is currently in use.
		 */
		if (hweight32(val) > 1)
			val &= ~twl4030_local.outsrc;
		/* can select multiple o/ps */
		if ((val & SOUND_MASK_LINE1) == SOUND_MASK_LINE1) {
			temp |= OUTPUT_STEREO_HEADSET;
		}
		if ((val & SOUND_MASK_SPEAKER) == SOUND_MASK_SPEAKER) {
			temp |= OUTPUT_HANDS_FREE_CLASSD;
		}
		if ((val & SOUND_MASK_PHONEOUT) == SOUND_MASK_PHONEOUT) {
			temp |= OUTPUT_MONO_EARPIECE;
		}
		current_output = temp;
		/* toggle the source */
		if (!(ret = twl4030_disable_output())) {
			ret = twl4030_enable_output();
		}
		if (!ret) {
			twl4030_local.outsrc = val;
		}
		break;
	case DIR_IN:
		/* if more than one device requested, reject the request */
		if (hweight32(val) > 1)
			return -EINVAL;
		/* select multiple i/ps? */
		if ((val & SOUND_MASK_LINE) == SOUND_MASK_LINE) {
			temp |= INPUT_HEADSET_MIC;
		}
		if ((val & SOUND_MASK_MIC) == SOUND_MASK_MIC) {
			temp |= INPUT_MAIN_MIC | INPUT_SUB_MIC;
		}
		current_input = temp;
		/* Toggle the source */
		if (!(ret = twl4030_disable_input())) {
			ret = twl4030_enable_input();
		}

		if (!ret) {
			twl4030_local.recsrc = val;
		}

		break;
	default:
		printk(KERN_WARNING PLATFORM_NAME "-" CODEC_NAME
		       ": Wrong twl4030_selectsource flag specified\n");
		ret = -EPERM;
		break;

	}
	if (!ret) {
		twl4030_local.mod_cnt++;
	} else {
		printk(KERN_ERR "Error selsrc Flag=%d,err=%d\n", flag, ret);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_setvolume - set the gain of the requested device
 * 
 * @param flag device for which the gain is to be set
 * @param gain_l 
 * @param gain_r 
 * 
 * @return error if this was not done, else returns 0
 */
static int twl4030_setvolume(int flag, u8 gain_l, u8 gain_r)
{
	int ret = 0;
	FN_IN;
	if ((gain_l > AUDIO_MAX_OUTPUT_VOLUME)
	    || (gain_r > AUDIO_MAX_OUTPUT_VOLUME)) {
		printk(KERN_ERR "Invalid gain value %d %d\n", gain_l, gain_r);
		return -EPERM;
	}
	DPRINTK
	    ("FLAG=0x%02x GAIN_L=%d[0x%02x] GAIN_R=%d[0x%2x] [R=%x]"
	     " [W=%x] [IP=0x%x] [OP=0x%x]\n",
	     flag, gain_l, gain_l, gain_r, gain_r,
	     (omap_twl_state.fmode & FMODE_READ) ? 1 : 0,
	     (omap_twl_state.fmode & FMODE_WRITE) ? 1 : 0, current_input,
	     current_output);

	switch (flag) {
	case OUTPUT_VOLUME:
		{
			/* normal volume control */
			u8 fine_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    AUDIO_OUTPUT_INCREMENT);
			u8 fine_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    AUDIO_OUTPUT_INCREMENT);
			/* Inverted power control big value is small volume */
			u8 ana_val_r =
			    (unsigned
			     char)(((AUDIO_MAX_OUTPUT_VOLUME -
				     gain_r) * COMPUTE_PRECISION) /
				   ARX_APGA_INCR);
			u8 ana_val_l =
			    (unsigned
			     char)(((AUDIO_MAX_OUTPUT_VOLUME -
				     gain_l) * COMPUTE_PRECISION) /
				   ARX_APGA_INCR);
			/* default value at this time... make it ioctl ?? */
			u8 coarse_val = AUDIO_DEF_COARSE_VOLUME_LEVEL;

			DPRINTK("outputVol gain_l requested %d, set %d\n",
				gain_l, fine_val_l);
			DPRINTK("outputVol gain_r requested %d, set %d\n",
				gain_r, fine_val_r);
			/* I2S - SDRL2 and SDRR2 */
			/* Digital boost */
			ret =
			    audio_twl4030_write(REG_ARXL2PGA,
						coarse_val <<
						BIT_ARXL2PGA_ARXL2PGA_CGAIN |
						fine_val_l <<
						BIT_ARXL2PGA_ARXL2PGA_FGAIN);
			if (!ret) {
				ret =
				    audio_twl4030_write(REG_ARXR2PGA,
							coarse_val <<
							BIT_ARXL2PGA_ARXL2PGA_CGAIN
							| fine_val_r <<
							BIT_ARXR2PGA_ARXR2PGA_FGAIN);
			}
			/* Analog boost */
			if (!ret) {
				ret =
				    audio_twl4030_write(REG_ARXL2_APGA_CTL,
							BIT_ARXL2_APGA_CTL_ARXL2_PDZ_M
							|
							BIT_ARXL2_APGA_CTL_ARXL2_DA_EN_M
							| ana_val_l <<
							BIT_ARXL2_APGA_CTL_ARXL2_GAIN_SET);
			}
			if (!ret) {
				ret =
				    audio_twl4030_write(REG_ARXR2_APGA_CTL,
							BIT_ARXR2_APGA_CTL_ARXR2_PDZ_M
							|
							BIT_ARXR2_APGA_CTL_ARXR2_DA_EN_M
							| ana_val_r <<
							BIT_ARXR2_APGA_CTL_ARXR2_GAIN_SET);
			}
			if (!ret) {
				twl4030_local.play_volume =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_r);
				DPRINTK("RESULT=play_vol=0x%x\n",
					twl4030_local.play_volume);
			}
		}
		break;
	case OUTPUT_STEREO_HEADSET:
		/* only if current output device is stereo headset */
		if ((current_output & OUTPUT_STEREO_HEADSET) ==
		    OUTPUT_STEREO_HEADSET) {
			/* normal volume control */
			u8 fine_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 fine_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 value_set[NON_LIN_GAIN_MAX] = NON_LIN_VALS;
			/*HS_GAIN_SET */
			DPRINTK("outputHS gain_l requested %d, set %d [0x%x]\n",
				gain_l, fine_val_l, value_set[fine_val_l]);
			DPRINTK("outputHS gain_r requested %d, set %d [0x%x]\n",
				gain_r, fine_val_r, value_set[fine_val_r]);
			/* Handle Mute request */
			fine_val_l = (gain_l == 0) ? 0 : value_set[fine_val_l];
			fine_val_r = (gain_r == 0) ? 0 : value_set[fine_val_r];
			ret = audio_twl4030_write(REG_HS_GAIN_SET,
						  (fine_val_l <<
						   BIT_HS_GAIN_SET_HSL_GAIN) |
						  (fine_val_r <<
						   BIT_HS_GAIN_SET_HSR_GAIN));
			if (!ret) {
				twl4030_local.hset =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_r);
				DPRINTK("RESULT=hset=0x%x\n",
					twl4030_local.hset);
			}
		}
		break;
	case OUTPUT_HANDS_FREE_CLASSD:
		if ((current_output & OUTPUT_HANDS_FREE_CLASSD) ==
		    OUTPUT_HANDS_FREE_CLASSD) {
			/* NOTE: CLASSD no special gain */
			twl4030_local.classd =
			    WRITE_LEFT_VOLUME(gain_l) |
			    WRITE_RIGHT_VOLUME(gain_r);
			DPRINTK("RESULT=classd=0x%x\n", twl4030_local.classd);
		}
		break;

	case OUTPUT_MONO_EARPIECE:
		if ((current_output & OUTPUT_MONO_EARPIECE) ==
		    OUTPUT_MONO_EARPIECE) {
			/* normal volume control */
			u8 fine_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    NON_LIN_INCREMENT);
			u8 value_set[NON_LIN_GAIN_MAX] = NON_LIN_VALS;
			DPRINTK("outputHS gain_l requested %d, set %d [0x%x]\n",
				gain_l, fine_val_l, value_set[fine_val_l]);
			fine_val_l = (gain_l == 0) ? 0 : value_set[fine_val_l];
			/*EAR_CTL */
			ret = audio_twl4030_write(REG_EAR_CTL,
						  fine_val_l <<
						  BIT_EAR_CTL_EAR_GAIN);
			if (!ret) {
				twl4030_local.ear = WRITE_LEFT_VOLUME(gain_l);
				DPRINTK("RESULT=ear=0x%x\n", twl4030_local.ear);
			}
		}
		break;
		/* Set input volume */
	case INPUT_VOLUME:
		{
			u8 set_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    AUDIO_INPUT_INCREMENT);
			u8 set_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    AUDIO_INPUT_INCREMENT);
			/* NOTE: ANAMIC gain settings is handled by a 
			 * default value 
			 */
			DPRINTK("input gain_l requested %d, set %d\n", gain_l,
				set_val_l);
			DPRINTK("input gain_r requested %d, set %d\n", gain_r,
				set_val_r);
			/* I2S - TXL1 and TXR1 only */
			ret =
			    audio_twl4030_write(REG_ATXL1PGA,
						set_val_l <<
						BIT_ATXL1PGA_ATXL1PGA_GAIN);
			if (!ret) {
				ret =
				    audio_twl4030_write(REG_ATXR1PGA,
							set_val_r <<
							BIT_ATXR1PGA_ATXR1PGA_GAIN);
			}
			if (!ret) {
				twl4030_local.rec_volume =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_r);
				DPRINTK("RESULT=rec_vol=0x%x\n",
					twl4030_local.rec_volume);
			}
		}
		break;
	case INPUT_HEADSET_MIC:
		if ((current_input & INPUT_HEADSET_MIC) == INPUT_HEADSET_MIC) {
			u8 set_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			DPRINTK("anamic gain_l requested %d, set %d\n", gain_l,
				set_val_l);
			/* ANAMIC_GAIN */
			ret =
			    audio_twl4030_write(REG_ANAMIC_GAIN,
						set_val_l <<
						BIT_ANAMIC_GAIN_MICAMPL_GAIN);
			if (!ret) {
				twl4030_local.line =
				    WRITE_LEFT_VOLUME(gain_l) |
				    WRITE_RIGHT_VOLUME(gain_l);
				DPRINTK("RESULT=line=0x%x\n",
					twl4030_local.line);
			}
		}
		break;
	case INPUT_MAIN_MIC:
		/* We do not use ALC Use ANAMIC_GAIN */
		/* left volume for main mic */
		if ((current_input & INPUT_MAIN_MIC) == INPUT_MAIN_MIC) {
			u8 set_val_l =
			    (unsigned char)((gain_l * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			int read_val = audio_twl4030_read(REG_ANAMIC_GAIN);
			if (read_val >= 0) {
				/* clear the left vol entry */
				read_val &= ~(BIT_ANAMIC_GAIN_MICAMPL_GAIN_M);
				read_val |=
				    set_val_l << BIT_ANAMIC_GAIN_MICAMPL_GAIN;
				ret =
				    audio_twl4030_write(REG_ANAMIC_GAIN,
							(u8) read_val);
				if (!ret) {
					twl4030_local.mic =
					    WRITE_LEFT_VOLUME(gain_l) |
					    WRITE_RIGHT_VOLUME(twl4030_local.
							       mic);
					DPRINTK("RESULT=mic (main)=0x%x\n",
						twl4030_local.mic);
				}
			}
		}
		break;
	case INPUT_SUB_MIC:
		/* We do not use ALC */
		/* right volume for submic */
		if ((current_input & INPUT_SUB_MIC) == INPUT_SUB_MIC) {
			u8 set_val_r =
			    (unsigned char)((gain_r * COMPUTE_PRECISION) /
					    MIC_AMP_INCR);
			int read_val = audio_twl4030_read(REG_ANAMIC_GAIN);
			if (read_val >= 0) {
				/* clear the right vol entry */
				read_val &= ~(BIT_ANAMIC_GAIN_MICAMPR_GAIN_M);
				read_val |=
				    set_val_r << BIT_ANAMIC_GAIN_MICAMPR_GAIN;
				ret =
				    audio_twl4030_write(REG_ANAMIC_GAIN,
							(u8) read_val);
				if (!ret) {
					twl4030_local.mic =
					    WRITE_LEFT_VOLUME(twl4030_local.
							      mic) |
					    WRITE_RIGHT_VOLUME(gain_r);
					DPRINTK("RESULT=mic (sub)=0x%x\n",
						twl4030_local.mic);
				}
			}
		}
		break;

	default:
		printk(KERN_WARNING PLATFORM_NAME "-" CODEC_NAME
		       ": Wrong twl4030_setvolume flag specified\n");
		ret = -EPERM;
		break;
	}
	if (!ret) {
		twl4030_local.mod_cnt++;
	}

	FN_OUT(0);
	return ret;
}

/** 
 * @brief twl4030_codec_conf_data_path - Configure the codec's data path
 * 
 * @return  0 if successful
 */
static int twl4030_codec_conf_data_path(void)
{
	u8 codec_data_width = 0;
	u8 codec_mode = 0;

	/* Check sample width */
	if (current_bitspersample == AUDIO_SAMPLE_DATA_WIDTH_16) {
		codec_data_width = AUDIO_DATA_WIDTH_16SAMPLE_16DATA;

	} else if (current_bitspersample == AUDIO_SAMPLE_DATA_WIDTH_24) {
		codec_data_width = AUDIO_DATA_WIDTH_32SAMPLE_24DATA;
	} else {
		printk(KERN_ERR "Unknown sample width %d\n",
		       current_bitspersample);
		return -EPERM;
	}

	/* No Need to set BIT_AUDIO_IF_CLK256FS_EN_M -not using it as CLKS!! */
	/* configure the audio IF of codec- Application Mode */
	codec_mode =
#ifndef TWL_MASTER
	    BIT_AUDIO_IF_AIF_SLAVE_EN_M |
#endif
	    (codec_data_width << BIT_AUDIO_IF_DATA_WIDTH) |
	    (AUDIO_DATA_FORMAT_I2S << BIT_AUDIO_IF_AIF_FORMAT) |
	    BIT_AUDIO_IF_AIF_EN_M;

	return audio_twl4030_write(REG_AUDIO_IF, codec_mode);
}

/** 
 * @brief twl4030_conf_data_interface
 * *NOTE* Call only from dsp device
 * 
 * @return  0 if successful
 */
static int twl4030_conf_data_interface(void)
{
	int ret = 0;
	int line = 0;
	int frame_length1 = OMAP2_MCBSP_FRAMELEN_2;
	int word_length1 = OMAP2_MCBSP_WORDLEN_32;
	int frame_polarity = OMAP2_MCBSP_FS_ACTIVE_LOW;
	int skip_alt = OMAP2_MCBSP_SKIP_NONE;

	FN_IN;
	/* Check sample width */
	if (current_bitspersample == AUDIO_SAMPLE_DATA_WIDTH_16) {
		if (current_stereomode == STEREO_MODE) {
			frame_polarity = OMAP2_MCBSP_FS_ACTIVE_HIGH;
		} else {
			/* mono Mode */
			/* use 16 bits dma even though 32 bit width */
			word_length1 = OMAP2_MCBSP_WORDLEN_16;
		}
		/* 1 word */
		frame_length1 = OMAP2_MCBSP_FRAMELEN_1;

	} else if (current_bitspersample == AUDIO_SAMPLE_DATA_WIDTH_24) {
		if (current_stereomode == MONO_MODE) {
			/* mono Mode */
			/* use 32 bits dma and do doubleindex */
			skip_alt = OMAP2_MCBSP_SKIP_SECOND;
		}
		/* 2 words */
		frame_length1 = OMAP2_MCBSP_FRAMELEN_2;
	} else {
		printk(KERN_ERR "Unknown sample width %d\n",
		       current_bitspersample);
		return -EPERM;
	}

	/* reset the McBSP registers so that we can 
	 * configure it 
	 */
	if (unlikely(ret = omap2_mcbsp_interface_reset(AUDIO_MCBSP))) {
		printk(KERN_ERR "conf_data Reset for MCBSP Failed[%d]\n", ret);
		/* Dont care abt result */
		return ret;
	}
	/* setup the new params */
	plat_mcbsp_config.tx_params.frame_length1 = frame_length1;
	plat_mcbsp_config.tx_params.word_length1 = word_length1;
	plat_mcbsp_config.tx_params.word_length2 = 0;
	plat_mcbsp_config.tx_params.frame_length2 = 0;
	plat_mcbsp_config.tx_params.skip_alt = skip_alt;
	plat_mcbsp_config.rx_params.frame_length1 = frame_length1;
	plat_mcbsp_config.rx_params.word_length1 = word_length1;
	plat_mcbsp_config.rx_params.word_length2 = 0;
	plat_mcbsp_config.rx_params.frame_length2 = 0;
	plat_mcbsp_config.rx_params.skip_alt = skip_alt;
	plat_mcbsp_config.fs_clk_pol = frame_polarity;
	plat_mcbsp_config.tx_polarity = frame_polarity;
	plat_mcbsp_config.rx_polarity = frame_polarity;

	ret = omap2_mcbsp_set_srg(AUDIO_MCBSP, OMAP2_MCBSP_SRG_DISABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret = omap2_mcbsp_set_fsg(AUDIO_MCBSP, OMAP2_MCBSP_FSG_DISABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	DPRINTK("mcbsp_id=%d samplerate=%d,"
		"bits=%d clk_src=%d clk=%d, sync=%d pol=%d\n",
		AUDIO_MCBSP,
		audio_samplerate,
		current_bitspersample,
		plat_mcbsp_config.srg_clk_src,
		1,
		plat_mcbsp_config.srg_clk_sync, plat_mcbsp_config.srg_clk_pol);

	/* Set the sample rate in mcbsp */
	/* PRCM Clock used - dont care abt clock - mcbsp, find it out */
	ret = omap2_mcbsp_srg_cfg(AUDIO_MCBSP,
				  audio_samplerate,
				  current_bitspersample,
				  plat_mcbsp_config.srg_clk_src,
				  1,
				  plat_mcbsp_config.srg_clk_sync,
				  plat_mcbsp_config.srg_clk_pol);
	if (unlikely(ret < 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}

	/* Setup the framesync clocks */
	DPRINTK("fsync cfg rxsrc=%d, txsrc=%d tx_pol=%d rx_pol=%d "
		"period=%d pulsewidth=%d\n",
		plat_mcbsp_config.tx_clk_src,
		plat_mcbsp_config.rx_clk_src,
		plat_mcbsp_config.tx_polarity,
		plat_mcbsp_config.rx_polarity,
		current_bitspersample * 2 - 1, current_bitspersample - 1);
#ifdef TWL_MASTER
	ret =
	    omap2_mcbsp_fsync_cfg(AUDIO_MCBSP,
				  plat_mcbsp_config.tx_clk_src,
				  plat_mcbsp_config.rx_clk_src,
				  plat_mcbsp_config.tx_polarity,
				  plat_mcbsp_config.rx_polarity, 0, 0, 0);
#else
	ret =
	    omap2_mcbsp_fsync_cfg(AUDIO_MCBSP,
				  plat_mcbsp_config.tx_clk_src,
				  plat_mcbsp_config.rx_clk_src,
				  plat_mcbsp_config.tx_polarity,
				  plat_mcbsp_config.rx_polarity,
				  current_bitspersample * 2 - 1,
				  current_bitspersample - 1, 1);
#endif
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	DPRINTK("IIItx_pol=%d, %d\n", plat_mcbsp_config.tx_clk_pol,
		plat_mcbsp_config.rx_clk_pol);
	ret =
	    omap2_mcbsp_txclk_cfg(AUDIO_MCBSP,
				  plat_mcbsp_config.tx_ip_clk,
				  plat_mcbsp_config.tx_clk_pol);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret =
	    omap2_mcbsp_rxclk_cfg(AUDIO_MCBSP,
				  plat_mcbsp_config.rx_ip_clk,
				  plat_mcbsp_config.rx_clk_pol);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
#ifndef TWL_MASTER
	ret = omap2_mcbsp_set_srg(AUDIO_MCBSP, OMAP2_MCBSP_SRG_ENABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret = omap2_mcbsp_set_fsg(AUDIO_MCBSP, OMAP2_MCBSP_FSG_ENABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
	}
#endif
      mcbsp_config_exit:
	if (unlikely(ret != 0)) {
		printk(KERN_ERR
		       "Unable to configure Mcbsp ret=%d @ line %d.", ret,
		       line);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_set_samplerate - set the sample rate of the codec
 *  and communication media (mcbsp)
 *  * NOTE* Shut down the codec to change sample rate
 *          Cannot reprogram the Codec APLL while codec is powered
 * 
 * @param sample_rate  - rate we wish to set
 * 
 * @return  0 if successful
 */
static int twl4030_set_samplerate(long sample_rate)
{
	int ret = 0;
	int count = 0;
	u8 codec_mode = 0;

	FN_IN;
	/* validate if rate is proper */
	for (; count < NUMBER_OF_RATES_SUPPORTED; count++) {
		if (valid_sample_rates[count].rate == sample_rate) {
			break;
		}
	}

	if (count >= NUMBER_OF_RATES_SUPPORTED) {
		printk(KERN_ERR "[%d] Unsupported sample rate!!\n",
		       (u32) sample_rate);
		return -EPERM;
	}

	ret =
	    audio_twl4030_write(REG_APLL_CTL,
				AUDIO_APLL_DEFAULT << BIT_APLL_CTL_APLL_INFREQ);
	if (ret < 0) {
		printk(KERN_ERR "unable to Set the INFREQ %d\n", ret);
		return ret;
	}
	/* Configure the codec -rate */
	ret = audio_twl4030_read(REG_CODEC_MODE);
	if (ret < 0) {
		printk(KERN_ERR "unable to read codec_mode %d\n", ret);
		return ret;
	}
	codec_mode = (u8) ret;

	/* clear unnecessary bits */
	codec_mode &= ~(BIT_CODEC_MODE_APLL_RATE_M);

	codec_mode |=
	    (valid_sample_rates[count].apll << BIT_CODEC_MODE_APLL_RATE);
	ret = audio_twl4030_write(REG_CODEC_MODE, codec_mode);

	/* program the apll */
	if (!ret) {
		ret =
		    audio_twl4030_write(REG_APLL_CTL,
					AUDIO_APLL_DEFAULT <<
					BIT_APLL_CTL_APLL_INFREQ |
					BIT_APLL_CTL_APLL_EN_M);
	}
	/* change the sample rate if we are successful */
	if (!ret) {
		audio_samplerate = sample_rate;
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_wait_inactive_stream - wait for streams to be inactive
 * call me only if streams are present
 * @return 0 or timedout
 */
static int twl4030_wait_inactive_stream(void)
{
	int count = TIMEOUT_WAIT_FOR_ACTIVE;
	while(count) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(1);
		count--;
		DPRINTK("Waiting %d\n", count);
		if ((omap_twl_state.output_stream->active)
		    || (omap_twl_state.input_stream->active)) {
			break;
		}
	}
	if (!count) {
		return -ETIMEDOUT;
	}
	return 0;
}
/** 
 * @brief twl4030_bit_set - setup the bitsize (16 or 24bit)
 * 
 * @param bit - bitsize
 * @param dsp -in dsp context?
 * 
 * @return 0 if all goes well, else error value
 */
static int twl4030_bit_set(int bit, int dsp)
{
	int ret = 0;
	int line = 0;
	FN_IN;
	if (bit == current_bitspersample) {
		/* we dont need to do anything at all! */
		DPRINTK ("nothing to set in Bitsize settings\n");
		return 0;
	}
	/* If the data streams are active.. it is a very very bad idea to change
	 * data transfer modes 
	 */
	if ((omap_twl_state.output_stream
	     && omap_twl_state.output_stream->active)
	    || (omap_twl_state.input_stream
		&& omap_twl_state.input_stream->active)) {
		if (twl4030_wait_inactive_stream()) {
			printk(KERN_ERR "Timedout Attempt to switch Bit mode"
				       "while active streams!!\n");
			return -EPERM;
		}
	}
	if (unlikely(ret = twl4030_ext_mut_on())) {
		printk(KERN_ERR "twl4030_ext_mut_on failed [%d]\n", ret);
		return ret;
	}
	DPRINTK("Bits per sample: OLD=0x%x, new=0x%x\n", current_bitspersample,
		bit);
	current_bitspersample = bit;
	/* toggle power of codec */
	ret = twl4030_codec_tog_on();
	if (ret) {
		line = __LINE__;
		goto set_bit_exit;
	}
	ret = twl4030_codec_conf_data_path();
	if (ret) {
		line = __LINE__;
		goto set_bit_exit;
	}
	/* we put the codec off again while configuring mcbsp */
	ret = twl4030_codec_off();
	if (ret) {
		line = __LINE__;
		goto set_bit_exit;
	}
	if (dsp) {
		ret = twl4030_conf_data_interface();
		if (ret) {
			line = __LINE__;
			goto set_bit_exit;
		}
		if ((omap_twl_state.fmode & FMODE_READ) == FMODE_READ) {
			ret =
			    omap2_mcbsp_set_recv_params(AUDIO_MCBSP,
							&
							(plat_mcbsp_config.
							 rx_params));
			if (ret < 0) {
				line = __LINE__;
				goto set_bit_exit;
			}
		}
		if ((omap_twl_state.fmode & FMODE_WRITE) == FMODE_WRITE) {
			ret =
			    omap2_mcbsp_set_trans_params(AUDIO_MCBSP,
							 & (plat_mcbsp_config.
							  tx_params));
			if (ret < 0) {
				line = __LINE__;
				goto set_bit_exit;
			}
		}
	}
	ret = twl4030_codec_on();
	if (unlikely(!ret && (ret = twl4030_ext_mut_off()))) {
		printk(KERN_ERR "twl4030_ext_mut_off failed [%d]\n", ret);
	}
      set_bit_exit:
	if (ret) {
		printk(KERN_ERR "Error in setting bit rate [0x%d]@%d\n", ret,
		       line);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_stereomode_set - setup the stereo mode
 * in mono mode only the LEFT data on I2S will be valid. 
 * This is same for RX and TX. mode is common for both rx and tx
 * 
 * @param mode  - stereo or mono
 *
 * @param dsp  - called from dsp context?
 * 
 * @return 0 if success/return error code
 */
static int twl4030_stereomode_set(int mode, int dsp)
{
	int ret = 0;
	u8 dac_ctl = 0;
	FN_IN;
	if (current_stereomode == mode) {
		/* nothing to do at all */
		DPRINTK ("nothing to do in stereomode setting\n");
		return 0;
	}
	/* If the data streams are active.. it is a very very bad idea to change
	 * data transfer modes 
	 */
	if ((omap_twl_state.output_stream
	     && omap_twl_state.output_stream->active)
	    || (omap_twl_state.input_stream
		&& omap_twl_state.input_stream->active)) {
		if (twl4030_wait_inactive_stream()) {
			printk(KERN_ERR "Timedout Attempt to switch mono/stereo mode"
				       "while active streams!!\n");
			return -EPERM;
		}
	}
	if (unlikely(ret = twl4030_ext_mut_on())) {
		printk(KERN_ERR "twl4030_ext_mut_on failed [%d]\n", ret);
		return ret;
	}
	/* toggle power of codec */
	ret = twl4030_codec_tog_on();
	if (ret < 0) {
		printk(KERN_ERR "MONO/STEREO Codec set failed\n");
		goto set_stereo_mode_exit;
	}
	ret = audio_twl4030_read(REG_AVDAC_CTL);
	if (ret < 0) {
		printk(KERN_ERR "did not get dac ctrl reg\n");
		goto set_stereo_mode_exit;
	}
	dac_ctl = ret;
	current_stereomode = mode;
	if (current_stereomode == MONO_MODE) {
		dac_ctl &= ~BIT_AVDAC_CTL_ADACR2_EN_M;
	} else {
		dac_ctl |= BIT_AVDAC_CTL_ADACR2_EN_M;
	}
	ret = audio_twl4030_write(REG_AVDAC_CTL, dac_ctl);
	if (ret < 0) {
		printk(KERN_ERR "did not set dac ctrl reg\n");
		goto set_stereo_mode_exit;
	}

	/* Power off codec */
	ret = twl4030_codec_off();
	if (ret) {
		printk(KERN_ERR "Unable to switch off the codec \n");
		goto set_stereo_mode_exit;
	}
	/* Let the Mcbsp work its magic */
	if (dsp) {
		ret = twl4030_conf_data_interface();
		if (ret) {
			printk(KERN_ERR "Configure data interface failed\n");
			goto set_stereo_mode_exit;
		}
		if ((omap_twl_state.fmode & FMODE_READ) == FMODE_READ) {
			ret =
			    omap2_mcbsp_set_recv_params(AUDIO_MCBSP,
							&
							(plat_mcbsp_config.
							 rx_params));
			if (ret < 0) {
				printk(KERN_ERR "MONO/STEREO RX params failed");
				goto set_stereo_mode_exit;
			}
		}
		if ((omap_twl_state.fmode & FMODE_WRITE) == FMODE_WRITE) {
			ret =
			    omap2_mcbsp_set_trans_params(AUDIO_MCBSP,
							 &
							 (plat_mcbsp_config.
							  tx_params));
			if (ret < 0) {
				printk(KERN_ERR "MONO/STEREO TX params failed");
				goto set_stereo_mode_exit;
			}
		}
	}
	/* Set the Mixing bit off if stereo, else set it to on */
#ifdef MONO_MODE_SOUNDS_STEREO
	ret =
	    audio_twl4030_write(REG_RX_PATH_SEL,
				((mode ==
				  STEREO_MODE) ? 0x00 :
				 BIT_RX_PATH_SEL_RXL2_SEL_M |
				 BIT_RX_PATH_SEL_RXR2_SEL_M));
#else
	ret = audio_twl4030_write(REG_RX_PATH_SEL, 0x00);
#endif
	ret = twl4030_codec_on();
	if (unlikely(!ret && (ret = twl4030_ext_mut_off()))) {
		printk(KERN_ERR "twl4030_ext_mut_off failed [%d]\n", ret);
	}
      set_stereo_mode_exit:
	if (ret) {
		printk(KERN_ERR "Setting Stereo mode failed[0x%x]\n", mode);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_handle_common_ioctl handle common dsp ioctls common for
 * /dev/dsp and /dev/mixer devices
 * 
 * @param cmd ioctl
 * @param arg ioctl
 * @param *ret result of the operation (if returned with 1) 
 * @param dsp dsp/mixer device?- we dont do mcbsp ops in mixer device
 * 
 * @return 0 if handled, else returns 1
 */
static int twl4030_handle_common_ioctl(uint cmd, ulong arg, int *ret, u8 dsp)
{
	/* res=1 implies - not handled
	 * res=0 implied handled
	 */
	int res = 1;
	long val = 0;
	FN_IN;
	if (unlikely(!ret)) {
		printk(KERN_ERR "Ret is null???!\n");
		return res;
	}
	switch (cmd) {
	case SNDCTL_DSP_STEREO:
		res = 0;
		*ret = get_user(val, (int *)arg);
		if (*ret)
			break;
		/* only 0-mono or 1-stereo */
		if (val & ~0x1) {
			*ret = -EINVAL;
			break;
		}
		*ret =
		    twl4030_stereomode_set((val ==
					    0) ? MONO_MODE : STEREO_MODE, dsp);
		if (*ret)
			break;
		*ret =
		    put_user((current_stereomode == MONO_MODE) ? 0 : 1,
			     (int *)arg);
		break;
	case SNDCTL_DSP_CHANNELS:
		res = 0;
		*ret = get_user(val, (int *)arg);
		if (*ret)
			break;
		/* only 1-mono or 2-stereo */
		if (!val || (val & ~0x3)) {
			*ret = -EINVAL;
			break;
		}
		*ret =
		    twl4030_stereomode_set((val ==
					    1) ? MONO_MODE : STEREO_MODE, dsp);
		if (*ret)
			break;
		/* Fall through */
	case SOUND_PCM_READ_CHANNELS:
		res = 0;
		FN_OUT(2);
		*ret =
		    put_user((current_stereomode == MONO_MODE) ? 1 : 2,
			     (int *)arg);
		break;
	case SNDCTL_DSP_SPEED:
		res = 0;
		*ret = get_user(val, (long *)arg);
		if (*ret) {
			printk(KERN_ERR "Unable to get from user space\n");
			break;
		}
		if (unlikely(*ret = twl4030_ext_mut_on())) {
			printk(KERN_ERR "twl4030_ext_mut_on failed [%d]\n", *ret);
			break;
		}
		*ret = twl4030_codec_tog_on();
		if (*ret) {
			printk(KERN_ERR "Unable Toggle the codec\n");
			break;
		}
		*ret = twl4030_set_samplerate(val);
		if (*ret) {
			printk(KERN_ERR "Unable to setup sample rate \n");
			break;
		}
		*ret = twl4030_codec_off();
		if (*ret) {
			printk(KERN_ERR "Unable switch off codec\n");
			break;
		}
#ifndef TWL_MASTER
		/* We need to reset the sample rate on mcbsp only 
		 * if twl is not master
		 */
		if (dsp) {
			*ret = twl4030_conf_data_interface();
			if (*ret) {
				printk(KERN_ERR
				       "Unable to setup sample rate \n");
				break;
			}
		}
#endif
		*ret = twl4030_codec_on();
		if (*ret) {
			printk(KERN_ERR "Unable to switch on Codec\n");
		}
		if (unlikely(*ret = twl4030_ext_mut_off())) {
			printk(KERN_ERR "twl4030_ext_mut_off failed [%d]\n", *ret);
			break;
		}
		/* Fall through */
	case SOUND_PCM_READ_RATE:
		res = 0;
		FN_OUT(3);
		*ret = put_user(audio_samplerate, (long *)arg);
		break;
	case SNDCTL_DSP_SETFMT:
		res = 0;
		*ret = get_user(val, (long *)arg);
		if (*ret) {
			printk(KERN_ERR "Unable to get from user space\n");
			break;
		}
		/* If this is a query - fall through to the read */
		if (val != AFMT_QUERY) {
			switch (val) {
			case AFMT_S16_LE:
				*ret =
				    twl4030_bit_set(AUDIO_SAMPLE_DATA_WIDTH_16,
						    dsp);
				break;
			case AFMT_S32_LE:
				*ret =
				    twl4030_bit_set(AUDIO_SAMPLE_DATA_WIDTH_24,
						    dsp);
				break;
			default:
				printk(KERN_ERR "Invalid Set Fmt request[%x]\n",
				       (int)val);
				*ret = -EINVAL;
				break;
			}
			if (*ret) {
				printk(KERN_ERR "Setting [0x%x] failed\n",
				       (int)val);
				break;
			}
		}
		/* Fall thru */
	case SOUND_PCM_READ_BITS:
		res = 0;
		switch (current_bitspersample) {
		case AUDIO_SAMPLE_DATA_WIDTH_16:
			val = AFMT_S16_LE;
			break;
		case AUDIO_SAMPLE_DATA_WIDTH_24:
			val = AFMT_S32_LE;
			break;
		default:
			printk(KERN_ERR "Invalid fmt value[0x%x-0x%x]\n",
			       (int)val, (current_bitspersample));
			*ret = -EINVAL;
			break;
		}
		/* supporting 16 and 32 bit */
		*ret = put_user(val, (long *)arg);
		break;

	case SNDCTL_DSP_GETFMTS:
		res = 0;
		FN_OUT(4);
		/* supporting 16 and 32 bit */
		*ret = put_user(AFMT_S16_LE | AFMT_S32_LE, (long *)arg);
		break;
	}
	FN_OUT(res);
	return res;
}

/** 
 * @brief twl4030_unconfigure
 */
static void twl4030_unconfigure(void)
{
	FN_IN;
	if (1 == twl4030_configured) {
		twl4030_codec_off();
		twl4030_disable_output();
		twl4030_disable_input();
		omap_twl_state.fmode = 0;
	}
	if (twl4030_configured > 0)
		twl4030_configured--;

	FN_OUT(0);
}

/** 
 * @brief twl4030_cleanup - clean up the register settings 
 * 
 * @return  0 if successful
 */
static int twl4030_cleanup(void)
{
	int ret = 0;
	int line = 0;
	ret = audio_twl4030_write(REG_VRXPGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_VDL_APGA_CTL, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_BTPGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_FREQSEL, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_TONOFF, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_PGA_CTL2, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_PGA_CTL1, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_DTMF_WANONOFF, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_HS_POPN_SET, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	/* Shut down the voice and other paths completely */
	ret = audio_twl4030_write(REG_ARXR1PGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL1PGA, 0x0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL1_APGA_CTL, 0x00);	/*path1 */
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR1_APGA_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL2_APGA_CTL, 0x00);	/*path1 */
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR2_APGA_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ALC_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXL2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ARXR2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ATXL1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_ATXR1PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_AVTXL2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_AVTXR2PGA, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_AVDAC_CTL, 0x00);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
	ret = audio_twl4030_write(REG_OPTION, 0);
	if (ret) {
		line = __LINE__;
		goto cleanup_exit;
	}
      cleanup_exit:
	if (ret) {
		printk(KERN_ERR "Cleanup Error[%d] Error in line %d\n",
		       ret, line);
	}
	FN_OUT(ret);
	return (ret);
}

/** 
 * @brief twl4030_configure
 * *NOTE* should be called with codec off
 * 
 * @return  0 if successful
 */
static int twl4030_configure(void)
{
	int ret = 0;
	int line = 0;
	FN_IN;

	if (0 == twl4030_configured) {
		int data = 0;
		ret = twl4030_ext_mut_conf();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_ext_mut_on();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}

		ret = twl4030_cleanup();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}

		/* set up the codec_mode - use option 1
		 * - assume no voice path 
		 */
		data = audio_twl4030_read(REG_CODEC_MODE);
		if (data < 0) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = audio_twl4030_write(REG_CODEC_MODE,
					  ((u8) data) | CODEC_OPTION_1 <<
					  BIT_CODEC_MODE_OPT_MODE);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		ret = twl4030_codec_conf_data_path();
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		/* Select Rx path
		 * SDRL1->RxL1 
		 * SDRR1->RxR1
		 * SDRL2->RxL2 (mono SDRM2)
		 * SDRL2->RxL2 (mono SDRM2)
		 */
		ret =
		    audio_twl4030_write(REG_RX_PATH_SEL,
					((current_stereomode ==
					  STEREO_MODE) ? 0x00 :
					 BIT_RX_PATH_SEL_RXL2_SEL_M |
					 BIT_RX_PATH_SEL_RXR2_SEL_M));
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		/* set the gains - we do not know the defaults
		 * attempt to set the volume of all the devices
		 * only those enabled get set.
		 */
		if (omap_twl_state.fmode & FMODE_WRITE) {
			ret = twl4030_setvolume(OUTPUT_VOLUME,
						READ_LEFT_VOLUME
						(twl4030_local.
						 play_volume),
						READ_RIGHT_VOLUME
						(twl4030_local.play_volume));
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
			ret = twl4030_setvolume(OUTPUT_STEREO_HEADSET,
						READ_LEFT_VOLUME
						(twl4030_local.hset),
						READ_RIGHT_VOLUME
						(twl4030_local.hset));
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
			ret =
			    twl4030_setvolume(OUTPUT_HANDS_FREE_CLASSD,
					      READ_LEFT_VOLUME
					      (twl4030_local.classd),
					      READ_RIGHT_VOLUME
					      (twl4030_local.classd));
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
			ret = twl4030_setvolume(OUTPUT_MONO_EARPIECE,
						READ_LEFT_VOLUME
						(twl4030_local.ear), 0);
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
		}
		if (omap_twl_state.fmode & FMODE_READ) {
			ret = twl4030_setvolume(INPUT_VOLUME,
						READ_LEFT_VOLUME
						(twl4030_local.
						 rec_volume),
						READ_RIGHT_VOLUME
						(twl4030_local.rec_volume));
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
			ret = twl4030_setvolume(INPUT_HEADSET_MIC,
						READ_LEFT_VOLUME
						(twl4030_local.line),
						READ_RIGHT_VOLUME
						(twl4030_local.line));
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
			ret = twl4030_setvolume(INPUT_MAIN_MIC,
						READ_LEFT_VOLUME
						(twl4030_local.mic), 0);
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
			ret = twl4030_setvolume(INPUT_SUB_MIC, 0,
						READ_RIGHT_VOLUME
						(twl4030_local.mic));
			if (ret) {
				line = __LINE__;
				goto configure_exit;
			}
		}
		/* Do not bypass the high pass filters */
		ret = audio_twl4030_write(REG_MISC_SET_2, 0x0);
		if (ret) {
			line = __LINE__;
			goto configure_exit;
		}
		/* switch on the input required */
		if (!(twl4030_disable_output())) {
			if (omap_twl_state.fmode & FMODE_WRITE) {
				ret = twl4030_enable_output();
				if (ret) {
					line = __LINE__;
					goto configure_exit;
				}
			}
		}
		if (!(twl4030_disable_input())) {
			if (omap_twl_state.fmode & FMODE_READ) {
				ret = twl4030_enable_input();
				if (ret) {
					line = __LINE__;
					goto configure_exit;
				}
			}
		}
	}
	twl4030_configured++;
      configure_exit:
	if (ret) {
		printk(KERN_ERR
		       "Configuration Error[%d] Error in line %d\n", ret, line);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief twl4030_mcbsp_dma_cb - the mcbsp call back
 * 
 * @param ch_status - dma channel status value
 * @param arg - the stream_callback structure
 */
static void twl4030_mcbsp_dma_cb(u16 ch_status, void *arg)
{
	struct stream_callback_struct *stream =
	    (struct stream_callback_struct *)arg;
	FN_IN;
	if (unlikely(!stream)) {
		printk(KERN_ERR "No Stream information!!\n");
		return;
	}
	if (unlikely(!stream->callback)) {
		printk(KERN_ERR "No Callback information!!\n");
		return;
	}
	if (ch_status) {
		printk(KERN_ERR "Error happend[%d 0x%x]!!\n", ch_status,
		       ch_status);
		return;

	}
	stream->callback(stream->state, stream->s);
	FN_OUT(0);
}

/****************                                          **************
 * 
 ***************************** MIXER  APIS ******************************
 *
 */

/** 
 * @brief mixer_open
 * 
 * @param inode 
 * @param file 
 * 
 * @return  0 if successful
 */
static int mixer_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	/* Any mixer specific initialization */

	/* Shut down the twl4030 only if the device is already dead */
	if (0 == twl4030_configured) {
		if (unlikely(ret = twl4030_ext_mut_conf())) {
			printk(KERN_ERR "twl4030_ext_mut_conf failed [%d]\n", ret);
			return ret;
		}
		if (unlikely(ret = twl4030_ext_mut_on())) {
			printk(KERN_ERR "twl4030_ext_mut_on failed [%d]\n", ret);
			return ret;
		}
		/* Setup such that it is full duplex -allows dsp side
		 * operations to be smooth
		 */
		omap_twl_state.fmode = file->f_mode;
		/* No one is using the device, we might be looking at
		 * rx,tx from dsp side. Enable the rx and tx paths
		 */
		if (unlikely(ret = twl4030_codec_tog_on())) {
			printk(KERN_ERR
			       " twl4030_codec_tog failed [%d]\n", ret);
			/* Dont care abt result */
			return ret;
		}
		if (unlikely(ret = twl4030_set_samplerate(audio_samplerate))) {
			printk(KERN_ERR " sample rate set failed [%d]\n", ret);
			return ret;
		}
	}
	/* configure me */
	if (unlikely(ret = twl4030_configure())) {
		omap_twl_state.fmode = 0;
		printk(KERN_ERR " twl4030_configure_device failed [%d]\n", ret);
		/* Dont care abt result */
		return ret;
	}
	/* Start codec if codec is just configured */
	if (1 == twl4030_configured) {
		/* setup the default sample rate etc.. */
		/* Toggle the codec enable to set the new values */
		if (unlikely(ret = twl4030_codec_tog_on())) {
			printk(KERN_ERR
			       " twl4030_codec_tog failed [%d]\n", ret);
			goto mixer_fail;
		}
		if (unlikely(ret = twl4030_ext_mut_off())) {
			printk(KERN_ERR "twl4030_ext_mut_off failed [%d]\n", ret);
			goto mixer_fail;
		}
	}

	return 0;
      mixer_fail:
	/* Dont care abt result */
	twl4030_unconfigure();
	return ret;
}

/** 
 * @brief mixer_release
 * 
 * @param inode 
 * @param file 
 * 
 * @return  0 
 */
static int mixer_release(struct inode *inode, struct file *file)
{
	/* if only I am around, quit the device */
	twl4030_unconfigure();
	/* Any mixer specific Un-initialization */

	return 0;
}

/** 
 * @brief mixer_ioctl
 * 
 * @param inode 
 * @param file 
 * @param cmd 
 * @param arg 
 * 
 * @return  0 if successful
 */
static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	int val;
	int ret = 0;
	int nr = _IOC_NR(cmd);

	FN_IN;

	/* Some exceptional cases -
	 * WARNING: This is against the OSS specification.
	 * We handle some of the DSP commands in mixer if given directly
	 * over mixer -
	 * reasons being that we want to be able to configure codec for the 
	 * dsp side playback.
	 */
	if (!twl4030_handle_common_ioctl(cmd, arg, &ret, MIXER_DEVICE)) {
		return ret;
	}

	/*
	 * We only accept mixer (type 'M') ioctls.
	 */
	if (_IOC_TYPE(cmd) != 'M')
		return -EINVAL;

	DPRINTK(" 0x%08x\n", cmd);

	if (cmd == SOUND_MIXER_INFO) {
		struct mixer_info mi;

		strncpy(mi.id, "TWL4030", sizeof(mi.id));
		strncpy(mi.name, "TI TWL4030", sizeof(mi.name));
		mi.modify_counter = twl4030_local.mod_cnt;
		FN_OUT(1);
		return copy_to_user((void *)arg, &mi, sizeof(mi));
	}

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		int gain_l, gain_r;
		ret = get_user(val, (int *)arg);
		if (ret)
			goto out;
		/* Basic Check for range */
		if (val >
		    ((AUDIO_MAX_OUTPUT_VOLUME << 8) | AUDIO_MAX_OUTPUT_VOLUME))
		{
			printk(KERN_ERR "Over range gain value 0x%x\n", val);
			return -EINVAL;
		}
		/* From OSS spec:
		 * The least significant
		 * byte gives volume for the left channel and the next 8 bits
		 * for the right channel.
		 */
		gain_l = READ_LEFT_VOLUME(val);
		gain_r = READ_RIGHT_VOLUME(val);

		switch (nr) {
		/*** Change global Volume */
		case SOUND_MIXER_VOLUME:
			ret = twl4030_setvolume(OUTPUT_VOLUME, gain_l, gain_r);
			break;

		case SOUND_MIXER_RECLEV:
			ret = twl4030_setvolume(INPUT_VOLUME, gain_l, gain_r);
			break;

		/*** Change specific device volume */

			/* OUTPUT DEVICES */
		case SOUND_MIXER_LINE1:
			ret =
			    twl4030_setvolume(OUTPUT_STEREO_HEADSET,
					      gain_l, gain_r);
			break;
		case SOUND_MIXER_SPEAKER:
			ret =
			    twl4030_setvolume(OUTPUT_HANDS_FREE_CLASSD,
					      gain_l, gain_r);
			break;
		case SOUND_MIXER_PHONEOUT:
			ret =
			    twl4030_setvolume(OUTPUT_MONO_EARPIECE,
					      gain_l, gain_r);
			break;

			/* INPUT DEVICES */
		case SOUND_MIXER_LINE:
			ret =
			    twl4030_setvolume(INPUT_HEADSET_MIC, gain_l,
					      gain_r);
			break;
		case SOUND_MIXER_MIC:
			/* only the left gain */
			ret = twl4030_setvolume(INPUT_MAIN_MIC, gain_l, 0);
			if (!ret) {
				/* only the right volume */
				ret =
				    twl4030_setvolume(INPUT_SUB_MIC, 0, gain_r);
			}
			break;

		/*** Change the record or play source */
		case SOUND_MIXER_OUTSRC:
			if ((val & OUT_SRC_MASK)) {
				if (twl4030_local.outsrc != val) {
					ret = twl4030_codec_tog_on();
					if (ret) {
						printk(KERN_ERR
						       "Codec power tog "
						       "failed!\n");
						break;
					}
					/* setup regs */
					ret =
					    twl4030_select_source(DIR_OUT, val);
					if (ret) {
						printk(KERN_ERR
						       "Source selection "
						       "failed!\n");
						break;
					}
					ret = twl4030_codec_tog_on();
					if (ret) {
						printk(KERN_ERR
						       "Codec power tog "
						       "failed!\n");
						break;
					}
				}
			} else {
				ret = -EINVAL;
			}
			break;

		case SOUND_MIXER_RECSRC:
			if ((val & REC_SRC_MASK)) {
				if (hweight32(val) > 1) {
					ret = -EINVAL;
					break;
				}
				if (twl4030_local.recsrc != val) {
					ret = twl4030_codec_tog_on();
					if (ret) {
						printk(KERN_ERR
						       "Codec power tog "
						       "failed!\n");
						break;
					}
					ret =
					    twl4030_select_source(DIR_IN, val);
					if (ret) {
						printk(KERN_ERR
						       "Source selection "
						       "failed!\n");
						break;
					}
					ret = twl4030_codec_tog_on();
					if (ret) {
						printk(KERN_ERR
						       "Codec power tog "
						       "failed!\n");
						break;
					}
				}
			} else {
				ret = -EINVAL;
			}
			break;

		default:
			ret = -EINVAL;
		}
	}

	if (ret == 0 && _IOC_DIR(cmd) & _IOC_READ) {
		ret = 0;

		switch (nr) {
			/* gimme volume */
		case SOUND_MIXER_VOLUME:
			val = twl4030_local.play_volume;
			break;
		case SOUND_MIXER_LINE1:
			val = twl4030_local.hset;
			break;
		case SOUND_MIXER_SPEAKER:
			val = twl4030_local.classd;
			break;
		case SOUND_MIXER_PHONEOUT:
			val =
			    WRITE_LEFT_VOLUME(twl4030_local.
					      ear) |
			    WRITE_RIGHT_VOLUME(twl4030_local.ear);
			break;
		case SOUND_MIXER_RECLEV:
			val = twl4030_local.rec_volume;
			break;
		case SOUND_MIXER_LINE:
			val = twl4030_local.line;
			break;
		case SOUND_MIXER_MIC:
			val = twl4030_local.mic;
			break;

			/* current sources */
		case SOUND_MIXER_OUTSRC:
			val = twl4030_local.outsrc;
			break;
		case SOUND_MIXER_RECSRC:
			val = twl4030_local.recsrc;
			break;
		case SOUND_MIXER_RECMASK:
			val = REC_MASK;
			break;
		case SOUND_MIXER_DEVMASK:
			val = DEV_MASK;
			break;
		case SOUND_MIXER_CAPS:
			val = SOUND_CAP_EXCL_INPUT;
			break;
		case SOUND_MIXER_STEREODEVS:
			val = DEV_STEREO_DEV;
			break;
		default:
			val = 0;
			printk(KERN_WARNING " unknown mixer "
			       "read ioctl flag specified\n");
			ret = -EINVAL;
			break;
		}

		if (ret == 0)
			ret = put_user(val, (int *)arg);
	}
      out:
	FN_OUT(0);
	return ret;

}

/****************                                          **************
 * 
 ***************************** CODEC  APIS ******************************
 *
 */

/** 
 * @brief omap_twl4030_transfer [hw_transfer]
 *  - Do the transfer to a said stream
 * @param s - stream for which the transfer occurs
 * @param buffer_phy - the physical address of the buffer
 * @param size - num bytes to transfer
 * 
 * @return 0 if success else return value -EBUSY implies retry later..
 */
static int omap_twl4030_transfer(struct audio_state_s *state,
				 audio_stream_t * s, void *buffer_phy, u32 size)
{
	int ret = 0;
	int restart = 0;
	FN_IN;
	if (unlikely(!s || !state || !buffer_phy)) {
		printk(KERN_ERR "Stream/State/phy_buf NULL!!\n");
		return -EPERM;
	}
	/* DEBUG */
#ifdef TWL_DUMP_REGISTERS_MCBSP
	printk(KERN_INFO "TRANSFER");
	omap_mcbsp_dump_reg(AUDIO_MCBSP);
#endif
	/* Error Recovery Strategy -
	 * 1. stop codec
	 * 2. stop mcbsp tx/rx
	 * 3. stop dma tx/rx
	 * ---
	 * Restart:
	 * 1. send/recievedata (starts mcbsp,dma)
	 * 2. start codec
	 */
	if (s->input_or_output == FMODE_READ) {
		DPRINTK("RX-%d", size);
		if (rx_err) {
			/* Detected failure in mcbsp - try to live */
			DPRINTK("DETECTED MCBSP RX FAILURE.. \n");
			if (unlikely(ret = twl4030_codec_off())) {
				printk(KERN_ERR
				       " Codec off failed [%d]\n", ret);
				goto transfer_exit;
			}
			if (unlikely
			    (ret =
			     omap2_mcbsp_set_rrst(AUDIO_MCBSP,
						  OMAP2_MCBSP_RRST_DISABLE))) {
				printk(KERN_ERR
				       " mcbsp rrst disable failed [%d]\n",
				       ret);
				goto transfer_exit;
			}
			if (unlikely
			    (ret = omap2_mcbsp_stop_datarx(AUDIO_MCBSP))) {
				printk(KERN_ERR
				       " Stop Data Rx for MCBSP Failed[%d]\n",
				       ret);
				goto transfer_exit;
			}
			restart = 1;
			rx_err = 0;
		}

		ret =
		    omap2_mcbsp_receive_data(AUDIO_MCBSP,
					     &in_stream_data,
					     (dma_addr_t) buffer_phy, size);
		DPRINTK("rx-%d\n", ret);
	} else {
		DPRINTK("TX-%d", size);
		if (tx_err) {
			/* Detected failure in mcbsp - try to live */
			DPRINTK("DETECTED MCBSP TX FAILURE.. \n");
			if (unlikely(ret = twl4030_ext_mut_on())) {
				printk(KERN_ERR "twl4030_ext_mut_on failed [%d]\n", ret);
				goto transfer_exit;
			}
			if (unlikely(ret = twl4030_codec_off())) {
				printk(KERN_ERR
				       " Codec off failed [%d]\n", ret);
				goto transfer_exit;
			}
			if (unlikely
			    (ret =
			     omap2_mcbsp_set_xrst(AUDIO_MCBSP,
						  OMAP2_MCBSP_XRST_DISABLE))) {
				printk(KERN_ERR
				       " mcbsp xrst disable failed [%d]\n",
				       ret);
				goto transfer_exit;
			}
			if (unlikely
			    (ret = omap2_mcbsp_stop_datatx(AUDIO_MCBSP))) {
				printk(KERN_ERR
				       " Stop Data Tx for MCBSP Failed[%d]\n",
				       ret);
				goto transfer_exit;
			}
			restart = 1;
			tx_err = 0;
		}
		ret =
		    omap2_mcbsp_send_data(AUDIO_MCBSP,
					  &out_stream_data,
					  (dma_addr_t) buffer_phy, size);
	}
	if (restart) {
#ifdef TWL_DUMP_REGISTERS_MCBSP
		printk(KERN_INFO "restart TRANSFER");
		omap_mcbsp_dump_reg(AUDIO_MCBSP);
#endif
		twl4030_codec_on();
		twl4030_ext_mut_off();
	}

      transfer_exit:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_transfer_stop [hw_transfer_stop]
 *  - Stop Transfers on the current stream
 * @param s - stream for which the transfer occurs
 * 
 * @return 0 if success else return value
 */
static int omap_twl4030_transfer_stop(audio_stream_t * s)
{
	int ret = 0;
	FN_IN;
	if (unlikely(!s)) {
		printk(KERN_ERR "Stream IS NULL!!\n");
		return -EPERM;
	}
	if (s->input_or_output == FMODE_READ) {
		ret = omap2_mcbsp_stop_datarx(AUDIO_MCBSP);
		(void)omap2_mcbsp_set_rrst(AUDIO_MCBSP,
					   OMAP2_MCBSP_RRST_DISABLE);
	} else {
		ret = omap2_mcbsp_stop_datatx(AUDIO_MCBSP);
		(void)omap2_mcbsp_set_xrst(AUDIO_MCBSP,
					   OMAP2_MCBSP_XRST_DISABLE);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_transfer_posn [hw_transfer_posn]
 *  - Where am i?
 * @param s - stream for which the transfer occurs
 * 
 * @return posn of the transfer
 */
static int omap_twl4030_transfer_posn(audio_stream_t * s)
{
	int ret = 0;
	int fi, ei;
	FN_IN;
	if (unlikely(!s)) {
		printk(KERN_ERR "Stream IS NULL!!\n");
		return -EPERM;
	}
	/* we always ask only one frame to transmit/recieve,
	 * variant is the element num 
	 */
	if (s->input_or_output == FMODE_READ) {
		ret = omap2_mcbsp_receiver_index(AUDIO_MCBSP, &ei, &fi);
		ret = (ei * (plat_mcbsp_config.rx_params.word_length1));
	} else {
		ret = omap2_mcbsp_transmitter_index(AUDIO_MCBSP, &ei, &fi);
		ret = (ei * (plat_mcbsp_config.tx_params.word_length1));
	}
	if (ret < 0) {
		printk(KERN_ERR
		       "twl4030_transfer_posn: Unable to find index of "
		       "transfer\n");
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_transfer_init [hw_transfer_init]
 *  - initialize the current stream
 * @param state -codec state for which this transfer is to be done
 * @param s - stream for which the transfer occurs
 * @param callback - call me back when I am done with the transfer
 * 
 * @return 0 if success else return value
 */
static int omap_twl4030_transfer_init(audio_state_t * state,
				      audio_stream_t * s,
				      buf_irq_handler callback)
{
	int ret = 0;
	struct stream_callback_struct *stream;
	FN_IN;
	if (unlikely(!s || !callback)) {
		printk(KERN_ERR "Stream/CALLBACK IS NULL!!\n");
		return -EPERM;
	}
	if (s->input_or_output == FMODE_READ) {
		stream = &in_stream_data;
	} else {
		stream = &out_stream_data;
	}
	memset(stream, 0, sizeof(struct stream_callback_struct));
	stream->s = s;
	stream->state = state;
	stream->callback = callback;
	if (s->input_or_output == FMODE_READ) {
		ret =
		    omap2_mcbsp_set_recv_params(AUDIO_MCBSP,
						&(plat_mcbsp_config.rx_params));
		if (ret < 0) {
			printk(KERN_ERR "RECV params failed");
			goto transfer_exit;
		}
	} else {
		ret =
		    omap2_mcbsp_set_trans_params(AUDIO_MCBSP,
						 &(plat_mcbsp_config.
						   tx_params));
		if (ret < 0) {
			printk(KERN_ERR "TX params failed");
			goto transfer_exit;
		}
	}
      transfer_exit:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_initialize
 * 
 * @param dummy 
 * 
 * @return  0 if successful
 */
static int omap_twl4030_initialize(struct audio_state_s *state, void *dummy)
{

	int ret = 0;

	FN_IN;
#ifdef TWL_MASTER
	if (unlikely
	    (ret =
	     omap2_mcbsp_request_interface(AUDIO_MCBSP,
					   OMAP2_MCBSP_SLAVE,
					   OMAP2_MCBSP_FCLKSRC_PRCM))) {
#else
	if (unlikely
	    (ret =
	     omap2_mcbsp_request_interface(AUDIO_MCBSP,
					   OMAP2_MCBSP_MASTER,
					   OMAP2_MCBSP_FCLKSRC_PRCM))) {
#endif
		printk(KERN_ERR " Request for MCBSP Failed[%d]\n", ret);
		goto initialize_exit_path1;
	}

	omap_twl_state.fmode = state->fmode;
#ifdef TWL_DUMP_REGISTERS
	printk(KERN_INFO "pre\n");
	twl4030_dumpRegisters();
#endif
	ret = twl4030_ext_mut_conf();
	if (ret) {
		printk(KERN_ERR "a twl4030_ext_mut_conf failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	ret = twl4030_ext_mut_on();
	if (ret) {
		printk(KERN_ERR "a twl4030_ext_mut_on failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* Toggle the codec power mode */
	if (unlikely(ret = twl4030_codec_tog_on())) {
		printk(KERN_ERR "a1 twl4030_codec_tog failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* Sample rate configuration - set APLLs to setup regs */
	ret = twl4030_set_samplerate(audio_samplerate);
	if (ret) {
		printk(KERN_ERR "Sample rate setting failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	if (unlikely(ret = twl4030_configure())) {
		printk(KERN_ERR " twl4030_configure_device failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* switch off the codec so that when mcbsp starts.. we are waiting */
	if (unlikely(twl4030_codec_off())) {
		printk(KERN_ERR "a2 twl4030_codec_off failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	ret = twl4030_conf_data_interface();
	if (ret) {
		printk(KERN_ERR "Codec Data init failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* register ISR */
	ret =
	    omap2_mcbsp_register_isr(AUDIO_MCBSP, twl_mcbsp_irq, NULL,
				     OMAP2_MCBSP_IRQEN_XOVFL |
				     OMAP2_MCBSP_IRQEN_XUNDFL |
				     OMAP2_MCBSP_IRQEN_ROVFL |
				     OMAP2_MCBSP_IRQEN_RUNDFL);
	if (ret) {
		printk(KERN_ERR "register of ISR failed [%d]\n", ret);
		goto initialize_exit_path3;
	}
	if (unlikely(ret = twl4030_codec_on())) {
		printk(KERN_ERR "a2 twl4030_codec_on failed [%d]\n", ret);
		goto initialize_exit_path3;
	}
	if (unlikely(ret = twl4030_ext_mut_off())) {
		printk(KERN_ERR "twl4030_ext_mut_off failed [%d]\n", ret);
		goto initialize_exit_path3;
	}
	/* Codec is operational */
#ifdef TWL_DUMP_REGISTERS
	printk(KERN_INFO "post\n");
	twl4030_dumpRegisters();
#endif
#ifdef TWL_DUMP_REGISTERS_MCBSP
	printk(KERN_INFO "CONFIG");
	omap_mcbsp_dump_reg(AUDIO_MCBSP);
#endif
#ifdef DEBUG

#ifdef TONE_GEN
	toneGen();
#endif
#endif				/* DEBUG */
	FN_OUT(0);
	return 0;
      initialize_exit_path3:
	twl4030_unconfigure();
	(void)omap2_mcbsp_interface_reset(AUDIO_MCBSP);
      initialize_exit_path2:
	/* Dont care abt result */
	(void)omap2_mcbsp_release_interface(AUDIO_MCBSP);
      initialize_exit_path1:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_shutdown
 * 
 * @param dummy 
 */
static void omap_twl4030_shutdown(void *dummy)
{

	FN_IN;
	omap2_mcbsp_unregister_isr(AUDIO_MCBSP);
	(void)omap2_mcbsp_interface_reset(AUDIO_MCBSP);
	(void)omap2_mcbsp_release_interface(AUDIO_MCBSP);
	twl4030_unconfigure();
	FN_OUT(0);
}

/** 
 * @brief omap_twl4030_ioctl
 * All generic ioctl's are handled by audio_ioctl() [File: omap-audio.c]. This
 * routine handles some platform specific ioctl's
 * 
 * @param inode 
 * @param file 
 * @param cmd 
 * @param arg 
 * 
 * @return  0 if successful
 */
static int
omap_twl4030_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	int ret = 0;

	DPRINTK(" 0x%08x\n", cmd);

	/*
	 * These are platform dependent ioctls which are not handled by the
	 * generic omap-audio module.
	 */
	if (twl4030_handle_common_ioctl(cmd, arg, &ret, DSP_DEVICE)) {
		/* not handled by common ioctls */
		/* Maybe this is meant for the mixer (As per OSS Docs) */
		return mixer_ioctl(inode, file, cmd, arg);
	}
	return ret;

}

/** 
 * @brief omap_twl4030_probe - check if the device is in real present or not
 *  If present then register the mixer device. else return failure
 * 
 * @return  0 if chip is present and intialized properly
 */
static int omap_twl4030_probe(void)
{
	int ret = 0;
	FN_IN;

	/* check if T2 device is actually present - Read IDCODE reg */
	printk(KERN_INFO PLATFORM_NAME " " CODEC_NAME " Audio Support: ");
	if (twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, (u8 *) & ret, 0x00)) {
		ret = -ENODEV;
		printk(KERN_INFO "Chip Not detected\n");
		goto twl4030_probe_out;
	}
	/* register mixer devices */
	mixer_dev_id = register_sound_mixer(&omap_mixer_fops, -1);
	if (mixer_dev_id >= 0) {
		/* Announcement Time */
		printk("Chip Rev[0x%02x] Initialized\n", ret);
		ret = 0;
	} else {
		printk(KERN_INFO "Mixer Not Initialized\n");
		ret = mixer_dev_id;
	}

      twl4030_probe_out:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_remove
 */
static void omap_twl4030_remove(void)
{
	FN_IN;
	unregister_sound_mixer(mixer_dev_id);

	FN_OUT(0);

}

/** 
 * @brief omap_twl4030_suspend
 * we do not know how long we'd be without using the codec, so 
 * we can shut it down.. not temporary phenomenon like scaling
 * 
 * @return  0 
 */
static int omap_twl4030_suspend(struct audio_state_s *state)
{

	FN_IN;
	/* Now, no such guarentees...
	 * Shut down the codec - it is expected that all the data transfers 
	 * would have Stopped by now 
	 */
	omap_twl4030_shutdown(NULL);
	FN_OUT(0);
	return 0;
}

/** 
 * @brief omap_twl4030_resume
 * 
 * @return  0 if successful
 */
static int omap_twl4030_resume(struct audio_state_s *state)
{

	int ret = 0;
	FN_IN;

	/* Just re-initialize the codec No transfers are pending till we are 
	 * done 
	 */
	ret = omap_twl4030_initialize(state, NULL);

	FN_OUT(ret);
	return ret;
}
/* TODO -- fix for PM */
#if 0
/** 
 * @brief omap_twl4030_sidle
 * 
 * @return  0 if successful
 */
static int omap_twl4030_sidle(struct audio_state_s *state, u32 idle_state)
{

	int ret = 0;
	FN_IN;
	/* Translate DPM level to McBSP supported level and pass it along */
	switch (idle_state) {
		case 0:
			/* Level D0 is ideally 'no-idle' mode with low latencies.
			 * So, disable local auto idle.
			 */ 
			ret = omap2_mcbsp_set_sidle_level(AUDIO_MCBSP, ICLK_AUTOIDLE_DIS);
			break;
		case 1:
			/* Level D1 Auto Idle, smart Idle and smart Standby, In
			 * 2430 McBSP context, it is setting the 'ILCK auto
			 * idle at the PRCM */
			ret = omap2_mcbsp_set_sidle_level(AUDIO_MCBSP, ICLK_AUTOIDLE_EN);
			break;

		case 2:
			/* Level D2 Auto Idle, smart Idle and smart Standby,
			 * with Aysc wakeup capability..
			 * On 2430 McBSP does not have wakeup capability. so,
			 * for now this state is similar to D1.. In future, we
			 * will use GPtimer as a wakeup event */
			ret = omap2_mcbsp_set_sidle_level(AUDIO_MCBSP, ICLK_AUTOIDLE_EN);
			break;
		default:
			break;
	}	
	FN_OUT(ret);
	return ret;
}
#endif
/** 
 * @brief omap_twl4030_prescale
 * 
 * @return  0 if successful
 */
static int omap_twl4030_prescale(void)
{
	int ret = 0;
	FN_IN;
	/* 
	 * This is a very effective,efficient and a pretty dirty way of 
	 * doing things. Mux out the mcbsp2 signals out, no signals, no
	 * dmareq and no transfers
	 */
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430) 
	ret = omap_cfg_reg("AC10_2430_MCBSP2_FSX_OFF");
	ret = omap_cfg_reg("AD16_2430_MCBSP2_CLX_OFF");
	ret = omap_cfg_reg("AE13_2430_MCBSP2_DX_OFF");
	ret = omap_cfg_reg("AD13_2430_MCBSP2_DR_OFF");
	/* Give time for these to settle down empirical vals */
	udelay(TWL4030_MCBSP2_2430SDP_PRESCALE_TIME);
#else
	/* Find other ways to shut off signals to mcbsp */
#endif
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_postscale
 * 
 * @return  0 if successful
 */
static int omap_twl4030_postscale(void)
{
	int ret = 0;
	FN_IN;
	/* 
	 * This is a very effective,efficient and a pretty dirty way 
	 * of doing things. Mux back the mcbsp2 signals out
	 */
#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430) 
	ret = omap_cfg_reg("AC10_2430_MCBSP2_FSX");
	ret = omap_cfg_reg("AD16_2430_MCBSP2_CLX");
	ret = omap_cfg_reg("AE13_2430_MCBSP2_DX");
	ret = omap_cfg_reg("AD13_2430_MCBSP2_DR");
	/* Give time for these to settle down empirical vals */
	udelay(TWL4030_MCBSP2_2430SDP_PRESCALE_TIME);
#else
	/* Find other ways to shut off signals to mcbsp */
#endif
	FN_OUT(ret);
	return ret;
}

/****************                                          **************
 * 
 ***************************** MODULE APIS ******************************
 *
 */

/** 
 * @brief twl4030_init
 * 
 * @return  0 if successful
 */
static int __init twl4030_init(void)
{

	int err = 0;
	FN_IN;
	/* register the codec with the audio driver */
	if ((err = audio_register_codec(&omap_twl_state))) {
		printk(KERN_ERR
		       "Failed to register TWL driver with Audio OSS Driver\n");
	}
	FN_OUT(err);
	return err;
}

/** 
 * @brief twl4030_exit
 * 
 */
static void __exit twl4030_exit(void)
{

	FN_IN;
	(void)audio_unregister_codec(&omap_twl_state);
	FN_OUT(0);
	return;
}

/****************                                          **************
 * 
 ***************************** DEBUG APIS *******************************
 *
 */

/*******************************************************************************
 * TONEGEN:
 * This is a test to generate a rather unpleasant sound..
 * verifies if the mcbsp is active 
 *
 ******************************************************************************/
#ifdef TONE_GEN
/* Generates a shrill tone */
u16 tone[] = {
	0x0ce4, 0x0ce4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000,
	0x0CE4, 0x0CE4, 0x1985, 0x1985, 0x25A1, 0x25A1, 0x30FD, 0x30FE,
	0x3B56, 0x3B55, 0x447A, 0x447A, 0x4C3B, 0x4C3C, 0x526D, 0x526C,
	0x56F1, 0x56F1, 0x59B1, 0x59B1, 0x5A9E, 0x5A9D, 0x59B1, 0x59B2,
	0x56F3, 0x56F2, 0x526D, 0x526D, 0x4C3B, 0x4C3B, 0x447C, 0x447C,
	0x3B5A, 0x3B59, 0x30FE, 0x30FE, 0x25A5, 0x25A6, 0x1989, 0x198A,
	0x0CE5, 0x0CE3, 0x0000, 0x0000, 0xF31C, 0xF31C, 0xE677, 0xE676,
	0xDA5B, 0xDA5B, 0xCF03, 0xCF03, 0xC4AA, 0xC4AA, 0xBB83, 0xBB83,
	0xB3C5, 0xB3C5, 0xAD94, 0xAD94, 0xA90D, 0xA90E, 0xA64F, 0xA64E,
	0xA562, 0xA563, 0xA64F, 0xA64F, 0xA910, 0xA90F, 0xAD93, 0xAD94,
	0xB3C4, 0xB3C4, 0xBB87, 0xBB86, 0xC4AB, 0xC4AB, 0xCF03, 0xCF03,
	0xDA5B, 0xDA5A, 0xE67B, 0xE67B, 0xF31B, 0xF3AC, 0x0000, 0x0000
};

void toneGen(void)
{
	int count = 0;
	int ret = 0;
	printk(KERN_INFO "TONE GEN TEST :");

	for (count = 0; count < 5000; count++) {
		int bytes;
		for (bytes = 0; bytes < sizeof(tone) / 2; bytes++) {
			ret = omap_mcbsp_pollwrite(AUDIO_MCBSP, tone[bytes]);
			if (ret == -1) {
				/* retry */
				bytes--;
			} else if (ret == -2) {
				printk(KERN_INFO "ERROR:bytes=%d\n", bytes);
				return;
			}
		}
	}
	printk(KERN_INFO "SUCCESS\n");
}

#endif				/* End of TONE_GEN */

/*******************************************************************************
 *
 * TWL_DUMP_REGISTERS:
 * This will dump the entire register set of Page 2 twl4030. 
 * Useful for major goof ups
 *
 ******************************************************************************/
#ifdef TWL_DUMP_REGISTERS
/** 
 * @brief twl4030_dumpRegisters
 */
static void twl4030_dumpRegisters(void)
{
	int i = 0;
	u16 data = 0;
	printk(KERN_INFO "TWL 4030 Register dump for Audio Module\n");
	for (i = REG_CODEC_MODE; i <= REG_MISC_SET_2; i++) {
		data = audio_twl4030_read(i);
		printk(KERN_INFO "Register[0x%02x]=0x%04x\n", i, data);

	}
}
#endif				/* End of #ifdef TWL_DUMP_REGISTERS */

module_init(twl4030_init);
module_exit(twl4030_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Codec audio driver for the TI TWL4030 codec.");
MODULE_LICENSE("GPL");
