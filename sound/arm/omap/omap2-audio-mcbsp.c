/*
 * sound/arm/omap/omap-audio-mcbsp.c
 *
 * Codec driver for TWL4030 for OMAP processors
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
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
 *  2006-09-15 Jian Zhang - Ported to ALSA 
 *  2007-04-16 Leonides Martinez - Added ALSA controls
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
#include <linux/device.h>

#include <asm/semaphore.h>
#include <asm/uaccess.h>
#include <asm/hardware.h>
#include <asm/arch/dma.h>
#include <asm/io.h>

#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/mach-types.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/control.h>

#include "omap2-audio_if.h"
#if defined(CONFIG_ARCH_OMAP243X) || defined(CONFIG_ARCH_OMAP3430)
#include <asm/arch/omap2_mcbsp.h>
#include <asm/arch/clock.h>
#else
#error "Unsupported configuration"
#endif
#include "omap2-audio-mcbsp.h"

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

#ifdef CONFIG_SND_OMAP_3430SDP
/** 
 * @brief twl_mcbsp_irq - the Interrupt handler for mcbsp events
 * 
 * @param arg  - ignored
 * @param irq_status -mcbsp interrupt status
 * @param regs -ignored
 */
static void twl_mcbsp_irq(void *arg, u32 irq_status)
{
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)arg;
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
	if ((irq_status &
		(OMAP2_MCBSP_IRQSTAT_XSYNCERR | OMAP2_MCBSP_IRQSTAT_XUNDFL |
		 OMAP2_MCBSP_IRQSTAT_XOVFL))) {
		/* We will handle this when we attempt a transfer */
		codec->tx_err = 1;
	}
	if ((irq_status &
		(OMAP2_MCBSP_IRQSTAT_RSYNCERR | OMAP2_MCBSP_IRQSTAT_RUNDFL |
		 OMAP2_MCBSP_IRQSTAT_ROVFL))) {
		/* We will handle this when we attempt a transfer */
		codec->rx_err = 1;
	}
}
#endif

/** 
 * @brief twl4030_conf_data_interface
 * *NOTE* Call only from dsp device
 * 
 * @return  0 if successful
 */
static int mcbsp_conf_data_interface(struct omap_alsa_codec *codec)
{
	int ret = 0;
	int line = 0;

	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;
	struct mcbsp_config *mcbsp_config = &pdata->mcbsp_config;
	uint8_t current_bitspersample = AUDIO_SAMPLE_DATA_WIDTH_16;

	FN_IN;

	/* reset the McBSP registers so that we can 
	 * configure it 
	 */
	if (unlikely(ret = omap2_mcbsp_interface_reset(mcbsp_id))) {
		printk(KERN_ERR "conf_data Reset for MCBSP Failed[%d]\n", ret);
		/* Dont care abt result */
		return ret;
	}

	ret = omap2_mcbsp_set_srg(mcbsp_id, OMAP2_MCBSP_SRG_DISABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret = omap2_mcbsp_set_fsg(mcbsp_id, OMAP2_MCBSP_FSG_DISABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	DPRINTK("mcbsp_id=%d samplerate=%d "
		"bits=%d clk_src=%d clk=%d, sync=%d pol=%d\n",
		mcbsp_id, codec->samplerate,
		current_bitspersample,
		mcbsp_config->srg_clk_src,
		1,
		mcbsp_config->srg_clk_sync, mcbsp_config->srg_clk_pol);

	/* Set the sample rate in mcbsp */
	/* PRCM Clock used - dont care abt clock - mcbsp, find it out */
	ret = omap2_mcbsp_srg_cfg(mcbsp_id,
				  codec->samplerate,
				  current_bitspersample,
				  mcbsp_config->srg_clk_src,
				  1,
				  mcbsp_config->srg_clk_sync,
				  mcbsp_config->srg_clk_pol);
	if (unlikely(ret < 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}

	/* Setup the framesync clocks */
	DPRINTK("fsync cfg rxsrc=%d, txsrc=%d tx_pol=%d rx_pol=%d "
		"period=%d pulsewidth=%d\n",
		mcbsp_config->fsx_src,
		mcbsp_config->fsr_src,
		mcbsp_config->fsx_pol,
		mcbsp_config->fsr_pol,
		current_bitspersample * 2 - 1, current_bitspersample - 1);
#ifdef TWL_MASTER
	ret =
	    omap2_mcbsp_fsync_cfg(mcbsp_id,
				  mcbsp_config->fsx_src,
				  mcbsp_config->fsr_src,
				  mcbsp_config->fsx_pol,
				  mcbsp_config->fsr_pol, 0, 0, 0);
#else
	ret =
	    omap2_mcbsp_fsync_cfg(mcbsp_id,
				  mcbsp_config->fsx_src,
				  mcbsp_config->fsr_src,
				  mcbsp_config->fsx_pol,
				  mcbsp_config->fsr_pol,
				  current_bitspersample * 2 - 1,
				  current_bitspersample - 1, 1);
#endif
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	DPRINTK("IIItx_pol=%d, %d\n", mcbsp_config->clkx_pol,
		mcbsp_config->clkr_pol);
	ret =
	    omap2_mcbsp_txclk_cfg(mcbsp_id,
				  mcbsp_config->clkx_src,
				  mcbsp_config->clkx_pol);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret =
	    omap2_mcbsp_rxclk_cfg(mcbsp_id,
				  mcbsp_config->clkr_src,
				  mcbsp_config->clkr_pol);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
#ifndef TWL_MASTER
	ret = omap2_mcbsp_set_srg(mcbsp_id, OMAP2_MCBSP_SRG_ENABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret = omap2_mcbsp_set_fsg(mcbsp_id, OMAP2_MCBSP_FSG_ENABLE);
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

static int audio_mcbsp_set_samplerate(long sample_rate, void *id)
{
	int ret = 0;
	int count = 0;
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;

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

	codec->samplerate = sample_rate;

	FN_OUT(ret);
	return ret;
}

static int audio_mcbsp_stereomode_set(int mode, int dsp, void *id)
{
	int ret = 0;
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;
	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;
	struct mcbsp_config *mcbsp_config = &pdata->mcbsp_config;
	
	if (codec->stereomode == mode) {
		DPRINTK ("nothing to do in stereomode setting\n");
		return 0;
	}

	codec->stereomode = mode;

	if (dsp) {
		ret = mcbsp_conf_data_interface(codec);
		if (ret) {
			printk(KERN_ERR "Configure data interface failed\n");
			goto set_stereo_mode_exit;
		}
		ret = omap2_mcbsp_set_recv_params(mcbsp_id, &(mcbsp_config->rx_params));
		if (ret < 0) {
			printk(KERN_ERR "MONO/STEREO RX params failed");
			goto set_stereo_mode_exit;
		}
		ret = omap2_mcbsp_set_trans_params(mcbsp_id, &(mcbsp_config->tx_params));
		if (ret < 0) {
			printk(KERN_ERR "MONO/STEREO TX params failed");
			goto set_stereo_mode_exit;
		}
	}

set_stereo_mode_exit:
	if (ret) {
		printk(KERN_ERR "Setting Stereo mode failed[0x%x]\n", mode);
	}

	return ret;
}

/** 
 * @brief twl4030_mcbsp_dma_cb - the mcbsp call back
 * 
 * @param ch_status - dma channel status value
 * @param arg - the stream_callback structure
 */
static void mcbsp_dma_cb(u16 ch_status, void *arg)
{
	if (ch_status) {
		printk(KERN_ERR "Error happend[%d 0x%x]!!\n", ch_status,
		       ch_status);
		return;

	}
#ifdef CONFIG_SND_OMAP_3430SDP
	audio_period_handler(arg);
#endif
	FN_OUT(0);
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
static int audio_mcbsp_transfer(int mode, void *buffer_phy, u32 size, void *arg, void *id)
{
	int ret = 0;
	int restart = 0;
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;
	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;
	int tx_err = codec->tx_err;
	int rx_err = codec->rx_err;
	FN_IN;

	//printk("audio_mcbsp_transfer: mcbsp%d\n", mcbsp_id);
	//omap_mcbsp_dump_reg(mcbsp_id);

#ifdef TWL_DUMP_REGISTERS_MCBSP
	printk(KERN_INFO "TRANSFER");
	omap_mcbsp_dump_reg(mcbsp_id);
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
	if (pdata->acquired) {
		if (mode == SNDRV_PCM_STREAM_CAPTURE) {
			DPRINTK("RX-%d", size);
			/* Capture Path to be implemented */
			if (rx_err) {
#if 0
				/* Detected failure in mcbsp - try to live */
				DPRINTK("DETECTED MCBSP RX FAILURE.. \n");
				if (unlikely(ret = twl4030_codec_off())) {
					printk(KERN_ERR " Codec off failed [%d]\n", ret);
					goto transfer_exit;
				}
				if (unlikely
				    (ret = omap2_mcbsp_set_rrst(AUDIO_MCBSP, OMAP2_MCBSP_RRST_DISABLE))) {
					printk(KERN_ERR " mcbsp rrst disable failed [%d]\n", ret);
					goto transfer_exit;
				}
				if (unlikely
				    (ret = omap2_mcbsp_stop_datarx(AUDIO_MCBSP))) {
					printk(KERN_ERR " Stop Data Rx for MCBSP Failed[%d]\n", ret);
					goto transfer_exit;
				}
				restart = 1;
#endif
				rx_err = 0;
			}
			ret = omap2_mcbsp_receive_data(mcbsp_id, arg, (dma_addr_t) buffer_phy, size);
			DPRINTK("rx-%d\n", ret);
		} else {
			DPRINTK("TX-%d", size);
			if (tx_err) {
#if 0
				/* Detected failure in mcbsp - try to live */
				DPRINTK("DETECTED MCBSP TX FAILURE.. \n");
				if (unlikely(ret = twl4030_ext_mut_on())) {
					printk(KERN_ERR "twl4030_ext_mut_on failed [%d]\n", ret);
					goto transfer_exit;
				}
				if (unlikely(ret = twl4030_codec_off())) {
					printk(KERN_ERR " Codec off failed [%d]\n", ret);
					goto transfer_exit;
				}
				if (unlikely
				    (ret = omap2_mcbsp_set_xrst(AUDIO_MCBSP, OMAP2_MCBSP_XRST_DISABLE))) {
					printk(KERN_ERR " mcbsp xrst disable failed [%d]\n", ret);
					goto transfer_exit;
				}
				if (unlikely
				    (ret = omap2_mcbsp_stop_datatx(AUDIO_MCBSP))) {
					printk(KERN_ERR " Stop Data Tx for MCBSP Failed[%d]\n", ret);
					goto transfer_exit;
				}
				restart = 1;
#endif
				tx_err = 0;
			}
			ret = omap2_mcbsp_send_data(mcbsp_id, arg, (dma_addr_t) buffer_phy, size);
		}
		if (restart) {
#ifdef TWL_DUMP_REGISTERS_MCBSP
			printk(KERN_INFO "restart TRANSFER");
			omap_mcbsp_dump_reg(mcbsp_id);
#endif
		}
	}
#if 0
      transfer_exit:
#endif
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
static int audio_mcbsp_transfer_stop(int mode, void *id)
{
	int ret = 0;
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;
	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;
	FN_IN;

	if (pdata->acquired) { 
		if (mode == SNDRV_PCM_STREAM_CAPTURE) {
			ret = omap2_mcbsp_stop_datarx(mcbsp_id);
			(void)omap2_mcbsp_set_rrst(mcbsp_id,
						   OMAP2_MCBSP_RRST_DISABLE);
		} else {
			ret = omap2_mcbsp_stop_datatx(mcbsp_id);
			(void)omap2_mcbsp_set_xrst(mcbsp_id,
						   OMAP2_MCBSP_XRST_DISABLE);
		}
	}
	FN_OUT(ret);
	return ret;
}

static inline int element_size(int mcbsp_wordlen)
{ 
	if (mcbsp_wordlen == OMAP2_MCBSP_WORDLEN_32)
		return 4;
	if (mcbsp_wordlen == OMAP2_MCBSP_WORDLEN_16)
		return 2;
	/* we don't allow any other word lengths */
	return -1;
}
/** 
 * @brief omap_twl4030_transfer_posn [hw_transfer_posn]
 *  - Where am i?
 * @param s - stream for which the transfer occurs
 * 
 * @return posn of the transfer
 */
static int audio_mcbsp_transfer_posn(int mode, void *id)
{
	int ret = 0;
	int fi, ei;
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;
	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;
	struct mcbsp_config *mcbsp_config = &pdata->mcbsp_config;
	FN_IN;

	/* we always ask only one frame to transmit/recieve,
	 * variant is the element num 
	 */
	if (pdata->acquired) {
		if (mode == SNDRV_PCM_STREAM_CAPTURE) {
			ret = omap2_mcbsp_receiver_index(mcbsp_id, &ei, &fi);
			ret = ei * element_size(mcbsp_config->rx_params.word_length1);
		} else {
			ret = omap2_mcbsp_transmitter_index(mcbsp_id, &ei, &fi);
			ret = ei * element_size(mcbsp_config->tx_params.word_length1);
		}
		if (ret < 0) {
			printk(KERN_ERR
			       "twl4030_transfer_posn: Unable to find index of "
			       "transfer\n");
		}
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
static int audio_mcbsp_transfer_init(int mode, void *id)
{
	int ret = 0;
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;
	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;
	struct mcbsp_config *mcbsp_config = &pdata->mcbsp_config;

	FN_IN;

	if (pdata->acquired) {
		if (mode == SNDRV_PCM_STREAM_CAPTURE) {
			ret =
			    omap2_mcbsp_set_recv_params(mcbsp_id, &(mcbsp_config->rx_params));
			if (ret < 0) {
				printk(KERN_ERR "RECV params failed");
				goto transfer_exit;
			}
		} else {
			ret =
			    omap2_mcbsp_set_trans_params(mcbsp_id, &(mcbsp_config->tx_params));
			if (ret < 0) {
				printk(KERN_ERR "TX params failed");
				goto transfer_exit;
			}
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
static int audio_mcbsp_initialize(void *id)
{
	int ret = 0;
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;
	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;

	FN_IN;
	if (pdata->acquired) {
		ret = 0;
		goto initialize_exit_path1;
	}
	pdata->acquired = 1;

#ifdef TWL_MASTER
	if (unlikely
	    (ret = omap2_mcbsp_request_interface(mcbsp_id,
						   OMAP2_MCBSP_SLAVE,
						   OMAP2_MCBSP_FCLKSRC_PRCM))) {
#else
	if (unlikely
	    (ret = omap2_mcbsp_request_interface(mcbsp_id,
						   OMAP2_MCBSP_MASTER,
						   OMAP2_MCBSP_FCLKSRC_PRCM))) {
#endif
		printk(KERN_ERR " Request for MCBSP Failed[%d]\n", ret);
		goto initialize_exit_path1;
	}

	ret = mcbsp_conf_data_interface(codec);
	if (ret) {
		printk(KERN_ERR "Codec Data init failed [%d]\n", ret);
		goto initialize_exit_path2;
	}
	/* register ISR */
	ret =
	    omap2_mcbsp_register_isr(mcbsp_id, twl_mcbsp_irq, codec, 
				     OMAP2_MCBSP_IRQEN_XOVFL |
				     OMAP2_MCBSP_IRQEN_XUNDFL |
				     OMAP2_MCBSP_IRQEN_ROVFL |
				     OMAP2_MCBSP_IRQEN_RUNDFL);
	if (ret) {
		printk(KERN_ERR "register of ISR failed [%d]\n", ret);
		goto initialize_exit_path3;
	}
	/* Codec is operational */
#ifdef TWL_DUMP_REGISTERS_MCBSP
	printk(KERN_INFO "CONFIG");
	omap_mcbsp_dump_reg(mcbsp_id);
#endif
	FN_OUT(0);
	return 0;
      initialize_exit_path3:
	(void)omap2_mcbsp_interface_reset(mcbsp_id);
      initialize_exit_path2:
	/* Dont care abt result */
	(void)omap2_mcbsp_release_interface(mcbsp_id);
      initialize_exit_path1:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_shutdown
 * 
 * @param dummy 
 */
static void audio_mcbsp_shutdown(void *id)
{
	struct omap_alsa_codec *codec = (struct omap_alsa_codec *)id;
	struct mcbsp_codec_data *pdata = (struct mcbsp_codec_data *)codec->private_data;
	unsigned int mcbsp_id = pdata->mcbsp_id;

	FN_IN;
#ifdef CONFIG_SND_OMAP_3430SDP
	pdata->acquired = 0;
	omap2_mcbsp_unregister_isr(mcbsp_id);
	(void)omap2_mcbsp_interface_reset(mcbsp_id);
	omap2_mcbsp_release_interface(mcbsp_id);
#endif
	FN_OUT(0);
}

/** 
 * @brief omap_twl4030_sidle
 * 
 * @return  0 if successful
 */
static int audio_mcbsp_sidle(u32 idle_state)
{

	int ret = 0;
	FN_IN;
	/* Translate DPM level to McBSP supported level and pass it along */
#if 0
	switch (idle_state) {
		case DEV_SUSPEND_ON:
			/* Level DEV_SUSPEND_ON is ideally 'no-idle' mode with 
			 * low latencies. So, disable local auto idle.
			 */ 
			ret = omap2_mcbsp_set_sidle_level(AUDIO_MCBSP,
							  ICLK_AUTOIDLE_DIS);
			break;
		case DEV_SUSPEND_IDLE_1:
			/* Level DEV_SUSPEND_IDLE_1 Auto Idle, smart Idle 
                         * and smart Standby, In 2430 McBSP context, it is 
                         * setting the 'ILCK auto idle at the PRCM */
			ret = omap2_mcbsp_set_sidle_level(AUDIO_MCBSP,
							  ICLK_AUTOIDLE_EN);
			break;
		case DEV_SUSPEND_IDLE_2:
			/* Level DEV_SUSPEND_IDLE_2 Auto Idle, smart Idle 
			 * and smart Standby, with Aysc wakeup capability..
			 * On 2430 McBSP does not have wakeup capability. so,
			 * for now this state is similar to DEV_SUSPEND_IDLE_1.. 
			 * In future, we will use GPtimer as a wakeup event */
			ret = omap2_mcbsp_set_sidle_level(AUDIO_MCBSP,
							  ICLK_AUTOIDLE_EN);  
			break;
		default:
			break;
	}       
	FN_OUT(ret);
#endif
	return ret;
}

/** 
 * @brief omap_twl4030_probe - check if the device is in real present or not
 *  If present then register the mixer device. else return failure
 * 
 * @return  0 if chip is present and intialized properly
 */
static int audio_mcbsp_probe()
{
	return 0;
}

static int audio_mcbsp_mixer_init(struct snd_card *card) 
{
	return 0;
}

static int audio_mcbsp_mixer_shutdown(struct snd_card *card) 
{
	return 0;
}

#if 0 /* To be enabled later */
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
#if CONFIG_ARCH_OMAP3430
	ret = omap2_cfg_reg(AC10_2430_MCBSP2_FSX_OFF);
	ret = omap2_cfg_reg(AD16_2430_MCBSP2_CLX_OFF);
	ret = omap2_cfg_reg(AE13_2430_MCBSP2_DX_OFF);
	ret = omap2_cfg_reg(AD13_2430_MCBSP2_DR_OFF);
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
#if CONFIG_ARCH_OMAP243X
	ret = omap2_cfg_reg(AC10_2430_MCBSP2_FSX);
	ret = omap2_cfg_reg(AD16_2430_MCBSP2_CLX);
	ret = omap2_cfg_reg(AE13_2430_MCBSP2_DX);
	ret = omap2_cfg_reg(AD13_2430_MCBSP2_DR);
	/* Give time for these to settle down empirical vals */
	udelay(TWL4030_MCBSP2_2430SDP_PRESCALE_TIME);
#else
	/* Find other ways to shut off signals to mcbsp */
#endif
	FN_OUT(ret);
	return ret;
}
#endif /* Freq. Scaling */


/****************                                          **************
 * 
 ***************************** MODULE APIS ******************************
 *
 */

/** 
 * @brief audio_mcbsp_init
 * 
 * @return  0 if successful
 */
static int __init audio_mcbsp_init(void)
{
	int err = 0;
	struct omap_alsa_codec *codec;
	struct mcbsp_codec_data *pdata;
	FN_IN;

	pdata = mcbsp_codec_data;
	for (codec = audio_mcbsp_codecs; codec->name; codec++) {
		codec->private_data = (void *)pdata++;
		/* register the codec with the audio driver */
		if ((err = audio_register_codec(codec))) {
			printk(KERN_ERR "Failed to register mcbsp driver"
				   " with Audio ALSA Driver\n");
		}
	}

	FN_OUT(err);
	return err;
}

/** 
 * @brief audio_mcbsp_exit
 * 
 */
static void __exit audio_mcbsp_exit(void)
{
	struct omap_alsa_codec *codec;
	FN_IN;
	for (codec = audio_mcbsp_codecs; codec; codec++) {
		(void)audio_unregister_codec(codec);
	}
	FN_OUT(0);
	return;
}

module_init(audio_mcbsp_init);
module_exit(audio_mcbsp_exit);

MODULE_DESCRIPTION("audio-mcbsp");
MODULE_LICENSE("GPL");
