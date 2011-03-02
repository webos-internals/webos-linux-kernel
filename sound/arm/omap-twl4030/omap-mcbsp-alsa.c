#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>

#include <linux/platform_device.h>

#include <asm/arch/clock.h>
#include <asm/arch/omap2_mcbsp.h>

#include <asm/arch/twl4030-audio.h>

#include "omap-mcbsp-alsa.h"
#include "audio-debug.h"
#include "script.h"

struct omap_alsa_stream {
	int stream_id;                 /* numeric identification */
	bool active;                   /* using this stream for transfer now */
	spinlock_t dma_lock;           /* locking in DMA operations */
#if 0
	int period;                    /* current transfer period */
	int periods;                   /* current count of periods registered in DMA engine */
	spinlock_t dma_lock;           /* locking in DMA operations */
	int offset;                    /* start position of the last period in alsa buffer */
#endif
	int periods_queued;             /* current period queued */
	int periods_completed;          /* current period completed */
	int periods;                   /* number of periods in the buffer */
	struct snd_pcm_substream *ss;  /* the pcm stream */

	void *dma_cpu_addr;
	dma_addr_t dma_handle;
	size_t dma_bytes;
#define DEFAULT_DMA_BUFSIZE    (64 * 1024)    /* 64k buffer max */
};

struct omap_mcbsp_alsa_chip {
	struct snd_card *card;
	struct omap_alsa_stream as[2];

	struct snd_pcm_hardware *hw_playback;
	struct snd_pcm_hardware *hw_capture;
	struct snd_pcm_hw_constraint_list *rate_constraint;

	unsigned int sample_rate;
	unsigned int data_width;
	unsigned int channels;

	int usage_cnt;
	bool configured;

	struct mcbsp_audio_config *mcbsp;
	struct omap_alsa_codec_ops *c_ops;
};

/* Only one callback */
//static twl4030_codec_event_callback callback;
//static void *callback_cookie;

static void omap_mcbsp_alsa_mcbsp_irq(void *arg, u32 irq_status)
{
#if 0
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
}

static void omap_mcbsp_alsa_process_dma(struct omap_alsa_stream *as)
{
	int r = 0;
	struct omap_mcbsp_alsa_chip *chip = snd_pcm_substream_chip(as->ss);
	struct snd_pcm_runtime *runtime;
	unsigned int dma_size;
	unsigned int offset;

	LTRACE_ENTRY;

	/* FIXME This is only DMA'ing one period's worth of data at a time---
	 * if we do bigger DMAs do we get a performance gain? */

	runtime = as->ss->runtime;
	if (as->active) {
		dma_size = frames_to_bytes(runtime, runtime->period_size);
		//offset = dma_size * as->period;
		offset = dma_size * as->periods_queued;

		TRACEF(40, OMAP_ALSA_TRACE_DRIVER, 
				"mcbsp %u period %u periods %u period_size %u dma_size %u "
				"dma_addr %p offset %p\n",
				//chip->mcbsp->mcbsp_id, as->period, as->periods, 
				chip->mcbsp->mcbsp_id, as->periods_queued, as->periods_completed,
				(unsigned int)runtime->period_size, dma_size, 
				runtime->dma_addr, offset);

		if (chip->mcbsp->acquired) {
			if (as->stream_id == SNDRV_PCM_STREAM_CAPTURE) {
				r = omap2_mcbsp_receive_data(chip->mcbsp->mcbsp_id, as, 
						(dma_addr_t)((void *)runtime->dma_addr + offset),
						dma_size);
				if (unlikely(r)) {
					printk(KERN_ERR
							"OMAP-AUDIO: failed to rx on mcbsp%d\n",
							chip->mcbsp->mcbsp_id);
				}
			} else {
				r = omap2_mcbsp_send_data(chip->mcbsp->mcbsp_id, as,
						(dma_addr_t)((void *)runtime->dma_addr + offset),
						dma_size);
				if (unlikely(r)) {
					printk(KERN_ERR
							"OMAP-AUDIO: failed to tx on mcbsp%d\n",
							chip->mcbsp->mcbsp_id);
				}
			}
		} else {
			TRACEF(4, OMAP_ALSA_TRACE_DRIVER,
					"mcbsp%d not acquired\b", chip->mcbsp->mcbsp_id);
		}

#if 0
		as->period += 1;
		as->period %= runtime->periods;
		as->periods += 1;
		as->offset = offset;
#endif
		as->periods = runtime->periods;
		as->periods_queued += 1;
		as->periods_queued %= as->periods;
	}

	LTRACE_EXIT;
}

static void omap_mcbsp_alsa_dma_complete(u16 ch_status, void *arg)
{
	struct omap_alsa_stream *as = (struct omap_alsa_stream *)arg;

	LTRACE_ENTRY;

	if (ch_status) {
		printk(KERN_ERR "OMAP-AUDIO: DMA error %#x\n", ch_status);
		return;
	}

	spin_lock(&as->dma_lock);
#if 0
	if (as->periods > 0) {
		as->periods -= 1;
	}
#endif
	as->periods_completed += 1;
	if( as->periods ) {
		as->periods_completed %= as->periods;
	}
	/* Queue the next DMA */
	omap_mcbsp_alsa_process_dma(as);

	spin_unlock(&as->dma_lock);

	if (as->active) {
		snd_pcm_period_elapsed(as->ss);
	}


	LTRACE_EXIT;
}

/* FIXME */
static inline int element_size(int mcbsp_wordlen)
{
	if (mcbsp_wordlen == OMAP2_MCBSP_WORDLEN_32) {
		return 4;
	}
	if (mcbsp_wordlen == OMAP2_MCBSP_WORDLEN_16) {
		return 2;
	}
	return -1;
}

static void *dma_buffer_allocate(struct omap_alsa_stream *as)
{
	snd_assert(as->dma_cpu_addr == NULL);
	snd_assert(as->dma_handle == 0);
	snd_assert(as->dma_bytes == 0);

	as->dma_cpu_addr = dma_alloc_coherent(NULL, DEFAULT_DMA_BUFSIZE, 
			&as->dma_handle, GFP_KERNEL | GFP_DMA);

	return as->dma_cpu_addr;
}

static void dma_buffer_free(struct omap_alsa_stream *as)
{
	snd_assert(as->dma_cpu_addr && as->dma_handle);

	dma_free_coherent(NULL, DEFAULT_DMA_BUFSIZE, as->dma_cpu_addr, 
			as->dma_handle);
}

/* mcbsp need to be reconfigured where there is a sample rate change */
static int omap_mcbsp_alsa_pcm_mcbsp_configure(
		struct omap_mcbsp_alsa_chip *chip)
{
	int r = 0;
	struct mcbsp_audio_config *mcbsp = chip->mcbsp;

	int line;    /* for error reporting */

	/* Some settings are different when in stereo */
	if (chip->channels > 1) {
		mcbsp->rx_params.phase = OMAP2_MCBSP_FRAME_DUALPHASE;
		mcbsp->rx_params.word_length1 = OMAP2_MCBSP_WORDLEN_16;
		mcbsp->rx_params.word_length2 = OMAP2_MCBSP_WORDLEN_16;
		mcbsp->tx_params.phase = OMAP2_MCBSP_FRAME_DUALPHASE;
		mcbsp->tx_params.word_length1 = OMAP2_MCBSP_WORDLEN_16;
		mcbsp->tx_params.word_length2 = OMAP2_MCBSP_WORDLEN_16;
	} else {
		mcbsp->rx_params.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE;
		mcbsp->rx_params.word_length1 = OMAP2_MCBSP_WORDLEN_16;
		mcbsp->rx_params.word_length2 = 0;
		mcbsp->tx_params.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE;
		mcbsp->tx_params.word_length1 = OMAP2_MCBSP_WORDLEN_16;
		mcbsp->tx_params.word_length2 = 0;
	}

	/* Reset the mcbsp before configuring it */
	r = omap2_mcbsp_interface_reset(mcbsp->mcbsp_id);
	if (unlikely(r)) {
		printk(KERN_ERR
			"OMAP-AUDIO: failed to reset mcbsp%d\n", 
			mcbsp->mcbsp_id);
		return r;
	}

	/* Disable sample rate/frame sync generator */
	r = omap2_mcbsp_set_srg(mcbsp->mcbsp_id, OMAP2_MCBSP_SRG_DISABLE);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}
	r = omap2_mcbsp_set_fsg(mcbsp->mcbsp_id, OMAP2_MCBSP_FSG_DISABLE);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}

	/* Configure clocks */
	r = omap2_mcbsp_srg_cfg(mcbsp->mcbsp_id, chip->sample_rate,
			chip->data_width, mcbsp->srg_clk_src, 1,
			mcbsp->srg_clk_sync, mcbsp->srg_clk_pol);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}
	r = omap2_mcbsp_fsync_cfg(mcbsp->mcbsp_id, mcbsp->fsx_src,
			mcbsp->fsr_src, mcbsp->fsx_pol, mcbsp->fsr_pol, 0, 0, 0);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}
	r = omap2_mcbsp_txclk_cfg(mcbsp->mcbsp_id, 
			mcbsp->clkx_src, mcbsp->clkx_pol);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}
	r = omap2_mcbsp_rxclk_cfg(mcbsp->mcbsp_id,
			mcbsp->clkr_src, mcbsp->clkr_pol);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}

	/* Configure tx/rx params */
	r = omap2_mcbsp_set_recv_params(mcbsp->mcbsp_id, &mcbsp->rx_params);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}
	r = omap2_mcbsp_set_trans_params(mcbsp->mcbsp_id, &mcbsp->tx_params);
	if (unlikely(r)) {
		line = __LINE__;
		goto err_p;
	}

	return r;

err_p:
	printk(KERN_ERR
		"OMAP-AUDIO: failed to configure mcbsp%d (line %d)\n",
		mcbsp->mcbsp_id, line);
	return r;
}

static int omap_mcbsp_alsa_pcm_open(struct snd_pcm_substream *ss)
{
	int r = 0;
	struct omap_mcbsp_alsa_chip *chip = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	int stream_id = ss->pstr->stream;

	int line;    /* for error reporting */

	LTRACE_ENTRY;

	/* Enable codec */
	if (chip->c_ops && chip->usage_cnt == 0 && chip->c_ops->enable) {
		chip->c_ops->enable(true);
	}

	chip->as[stream_id].ss = ss;
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = *(chip->hw_playback);
	} else {
		runtime->hw = *(chip->hw_capture);
		if (chip->c_ops && chip->c_ops->capture_enable) {
			chip->c_ops->capture_enable();
		}
	}

	/* Apply the HW constraints */
	if (chip->rate_constraint) {
		r = snd_pcm_hw_constraint_list(runtime, 0, 
				SNDRV_PCM_HW_PARAM_RATE, chip->rate_constraint);
		if (r < 0) {
			line = __LINE__;
			goto err_p;
		}
	}
	r = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (r < 0) {
		line = __LINE__;
		goto err_p;
	}
	r = snd_pcm_hw_constraint_step(runtime, 0, 
			SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 1024);
	if (r < 0) {
		line = __LINE__;
		goto err_p;
	}
	r = snd_pcm_hw_constraint_step(runtime, 0,
			SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 1024);
	if (r < 0) {
		line = __LINE__;
		goto err_p;
	}

	chip->usage_cnt += 1;

	LTRACE_EXIT;

	return r;

err_p:
	printk(KERN_ERR "OMAP-AUDIO: failed to open pcm (line %d)\n", line);
	return r;
}

static int omap_mcbsp_alsa_pcm_close(struct snd_pcm_substream *ss)
{
	int r = 0;
	struct omap_mcbsp_alsa_chip *chip = snd_pcm_substream_chip(ss);

	LTRACE_ENTRY;

	chip->usage_cnt -= 1;
	if (chip->usage_cnt == 0 && chip->configured) {
		omap2_mcbsp_unregister_isr(chip->mcbsp->mcbsp_id);
		omap2_mcbsp_interface_reset(chip->mcbsp->mcbsp_id);
		omap2_mcbsp_release_interface(chip->mcbsp->mcbsp_id);
		chip->mcbsp->acquired = false;

		if (chip->c_ops && chip->c_ops->enable) {
			chip->c_ops->enable(false);
		}

		chip->configured = false;
	}

	LTRACE_EXIT;

	return r;
}

static int omap_mcbsp_alsa_pcm_hw_params(struct snd_pcm_substream *ss, 
		struct snd_pcm_hw_params *hp)
{
	int r = 0;
	struct omap_mcbsp_alsa_chip *chip = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct mcbsp_audio_config *mcbsp = chip->mcbsp;
	struct omap_alsa_stream *as;
	size_t req_bufsize;

	LTRACE_ENTRY;

	/* Prepare the DMA buffer */
	req_bufsize = params_buffer_bytes(hp);
	as = &chip->as[ss->pstr->stream];

	snd_assert(req_bufsize <= DEFAULT_DMA_BUFSIZE);
	snd_assert(as->dma_cpu_addr);

	runtime->dma_area = as->dma_cpu_addr;
	runtime->dma_addr = as->dma_handle;
	runtime->dma_bytes = req_bufsize;
	memset((void *)runtime->dma_area, 0, runtime->dma_bytes);

	/* Set DMA callbacks if unset */
	if (!mcbsp->tx_params.callback) {
		mcbsp->tx_params.callback = omap_mcbsp_alsa_dma_complete;
	}
	if (!mcbsp->rx_params.callback) {
		mcbsp->rx_params.callback = omap_mcbsp_alsa_dma_complete;
	}

	/* Acquire the mcbsp interface */
	if (mcbsp->acquired) {
		TRACEF(20, OMAP_ALSA_TRACE_DRIVER, 
				"mcbsp%d already acquired\n", mcbsp->mcbsp_id);
	} else {
		r = omap2_mcbsp_request_interface(mcbsp->mcbsp_id,
				OMAP2_MCBSP_SLAVE, mcbsp->mcbsp_clk_src);
		if (unlikely(r)) {
			printk(KERN_ERR
				"OMAP-AUDIO: failed to acquire mcbsp%d\n", 
				mcbsp->mcbsp_id);
			return r;
		}

		/* Register ISR */
		r = omap2_mcbsp_register_isr(mcbsp->mcbsp_id, 
				omap_mcbsp_alsa_mcbsp_irq, chip, 
				OMAP2_MCBSP_IRQEN_XOVFL | OMAP2_MCBSP_IRQEN_XUNDFL |
				OMAP2_MCBSP_IRQEN_ROVFL | OMAP2_MCBSP_IRQEN_RUNDFL);
		if (unlikely(r)) {
			printk(KERN_ERR 
				"OMAP-AUDIO: failed to register isr for mcbsp%d\n", 
				mcbsp->mcbsp_id);
			goto err;
		}

		mcbsp->acquired = true;
	}

	return r;

err:
	omap2_mcbsp_interface_reset(mcbsp->mcbsp_id);
	omap2_mcbsp_release_interface(mcbsp->mcbsp_id);
	mcbsp->acquired = false;

	return r;
}

static int omap_mcbsp_alsa_pcm_hw_free(struct snd_pcm_substream *ss)
{
	int r = 0;
	struct snd_pcm_runtime *runtime = ss->runtime;

	LTRACE_ENTRY;

	/* Do not actually free the buffer---same buffer is reused across opens */
	runtime->dma_area = NULL;
	runtime->dma_bytes = 0;
	runtime->dma_addr = (dma_addr_t)NULL;

	return r;
}

static int omap_mcbsp_alsa_pcm_prepare(struct snd_pcm_substream *ss)
{
	int r = 0;
	struct omap_mcbsp_alsa_chip *chip = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *runtime = ss->runtime;
	struct omap_alsa_stream *as = &chip->as[ss->pstr->stream];

	bool changed = false;

	LTRACE_ENTRY;

	/* Check if the sample rate, channels, format changed 
	 * FIXME handle this */
	if (runtime->rate != chip->sample_rate) {
		TRACEF(20, OMAP_ALSA_TRACE_DRIVER, "Sample rate changed (%u -> %u)\n", 
				chip->sample_rate, runtime->rate);
		changed = true;
	}
	if (runtime->channels != chip->channels) {
		TRACEF(20, OMAP_ALSA_TRACE_DRIVER, "Channels changed (%u -> %u)\n", 
				chip->channels, runtime->channels);
		changed = true;
	}
	if (changed && chip->configured) {
		TRACEF(4, OMAP_ALSA_TRACE_DRIVER, 
				"Both stream must use the same settings "
				"(sample rate %u %u, channels %u %u)\n", 
				chip->sample_rate, runtime->rate, 
				chip->channels, runtime->channels);
		return -EINVAL;
	}

	chip->sample_rate = runtime->rate;
	chip->channels = runtime->channels;
	chip->data_width = 16;    /* format checked by hw constraints */

	TRACEF(20, OMAP_ALSA_TRACE_DRIVER, 
			"sample_rate %u, data_width %u channels %u\n",
			chip->sample_rate, chip->data_width, chip->channels);

	if (!chip->configured) {
		/* Configure codec */
		if (chip->c_ops && chip->c_ops->configure) {
			chip->c_ops->configure(chip->sample_rate, chip->data_width,
					chip->channels);
		}

		/* Configure mcbsp */
		r = omap_mcbsp_alsa_pcm_mcbsp_configure(chip);
		if (unlikely(r)) {
			return r;
		}

		chip->configured = true;
	}

	/* Reset DMA position */
#if 0
	as->period = 0;
	as->periods = 0;
#endif
	spin_lock(&as->dma_lock);
	as->periods = 0;
	as->periods_queued = 0;
	as->periods_completed = 0;
	spin_unlock(&as->dma_lock);

	/* At this point the mcbsp and codec are configured with the given 
	 * sample rate, channels, format, and ready for transfer */

	return r;
}

static int omap_mcbsp_alsa_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	/* atomic */
	int r = 0;
	struct omap_mcbsp_alsa_chip *chip = snd_pcm_substream_chip(ss);
	struct omap_alsa_stream *as = &chip->as[ss->pstr->stream];
	unsigned long flags;

	LTRACE_ENTRY;

	spin_lock(&as->dma_lock);
	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		{
			as->active = true;
			omap_mcbsp_alsa_process_dma(as);
			/* Queue one more DMA to get rid of popping noise */
			omap_mcbsp_alsa_process_dma(as);
			if (chip->c_ops && chip->c_ops->mute && as->stream_id == SNDRV_PCM_STREAM_PLAYBACK) {/* currently only defined in twl4030-i2s */
				chip->c_ops->mute(false);
			}
			break;
		}

		case SNDRV_PCM_TRIGGER_STOP:
		{
			spin_lock_irqsave(&as->dma_lock, flags);
			as->active = false;
#if 0
			as->period = 0;
			as->periods = 0;
#endif
			as->periods = 0;
			as->periods_queued = 0;
			as->periods_completed = 0;

			if (chip->mcbsp->acquired) {
				/* stop the mcbsp */
				if (as->stream_id == SNDRV_PCM_STREAM_CAPTURE) {
					r = omap2_mcbsp_stop_datarx(chip->mcbsp->mcbsp_id);
					if (unlikely(r)) {
						printk(KERN_ERR 
							"OMAP-AUDIO: failed to stop mcbsp%d rx\n",
							chip->mcbsp->mcbsp_id);
					}
					omap2_mcbsp_set_rrst(chip->mcbsp->mcbsp_id,
							OMAP2_MCBSP_RRST_DISABLE);
				} else {
					r = omap2_mcbsp_stop_datatx(chip->mcbsp->mcbsp_id);
					if (unlikely(r)) {
						printk(KERN_ERR 
							"OMAP-AUDIO: failed to stop mcbsp%d tx\n",
							chip->mcbsp->mcbsp_id);
					}
					omap2_mcbsp_set_xrst(chip->mcbsp->mcbsp_id,
							OMAP2_MCBSP_XRST_DISABLE);
				}
			} else {
				TRACEF(4, OMAP_ALSA_TRACE_DRIVER,
					"mcbsp%d not acquired\b", chip->mcbsp->mcbsp_id);
			}

			spin_unlock_irqrestore(&as->dma_lock, flags);
			break;
		}

		default:
		{
			r = -EINVAL;
			break;
		}
	}
	spin_unlock(&as->dma_lock);

	return r;
}

static snd_pcm_uframes_t omap_mcbsp_alsa_pcm_pointer(
		struct snd_pcm_substream *ss)
{
	/* atomic */
	snd_pcm_uframes_t f = 0;
	struct omap_mcbsp_alsa_chip *chip = snd_pcm_substream_chip(ss);
	struct omap_alsa_stream *as = &chip->as[ss->pstr->stream];

	//LTRACE_ENTRY;

	spin_lock(&as->dma_lock);
	f = as->periods_completed;
	spin_unlock(&as->dma_lock);

	f = f * ss->runtime->period_size;
	TRACEF(40, OMAP_ALSA_TRACE_DRIVER, 
			"[%u] queued %d completed %d f %u\n", ss->pstr->stream,
			as->periods_queued, as->periods_completed, f);

	return f;
}

static int omap_mcbsp_alsa_pcm_mmap(struct snd_pcm_substream *ss,
		struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = ss->runtime;
	
	LTRACE_ENTRY;

	return dma_mmap_coherent(ss->pcm->card->dev, vma,
			runtime->dma_area, runtime->dma_addr, runtime->dma_bytes);
}

static struct snd_pcm_ops omap_mcbsp_alsa_playback_ops = {
	.open = omap_mcbsp_alsa_pcm_open,
	.close = omap_mcbsp_alsa_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = omap_mcbsp_alsa_pcm_hw_params,
	.hw_free = omap_mcbsp_alsa_pcm_hw_free,
	.prepare = omap_mcbsp_alsa_pcm_prepare,
	.trigger = omap_mcbsp_alsa_pcm_trigger,
	.pointer = omap_mcbsp_alsa_pcm_pointer,
	.mmap = omap_mcbsp_alsa_pcm_mmap,
};

static struct snd_pcm_ops omap_mcbsp_alsa_capture_ops = {
	.open = omap_mcbsp_alsa_pcm_open,
	.close = omap_mcbsp_alsa_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = omap_mcbsp_alsa_pcm_hw_params,
	.hw_free = omap_mcbsp_alsa_pcm_hw_free,
	.prepare = omap_mcbsp_alsa_pcm_prepare,
	.trigger = omap_mcbsp_alsa_pcm_trigger,
	.pointer = omap_mcbsp_alsa_pcm_pointer,
	.mmap = omap_mcbsp_alsa_pcm_mmap,
};

static void omap_mcbsp_alsa_drv_private_free(struct snd_card *card)
{
	LTRACE_ENTRY;

	schedule_timeout_interruptible(2);
}

static int omap_mcbsp_alsa_drv_probe(struct platform_device *pdev)
{
	int r = 0;
	struct omap_alsa_dev_private *priv;
	struct omap_alsa_codec_ops *c_ops;
	struct omap_mcbsp_alsa_chip *chip = NULL;
	struct snd_card *card = NULL;
	struct snd_pcm *pcm;

	LTRACE_ENTRY;

	priv = (struct omap_alsa_dev_private *)pdev->dev.platform_data;
	if (priv->get_codec_ops) {
		c_ops = priv->get_codec_ops();

		/* Probe the codec */
		r = c_ops->probe();
		if (r) {
			return r;
		}
	} else {
		c_ops = NULL;
	}

	card = snd_card_new(-1, NULL, THIS_MODULE, 
			sizeof(struct omap_mcbsp_alsa_chip));
	if (!card) {
		r = -EINVAL;
		return r;
	}

	chip = (struct omap_mcbsp_alsa_chip *)
		kzalloc(sizeof(struct omap_mcbsp_alsa_chip), GFP_KERNEL);
	card->private_data = (void *)chip;
	card->private_free = omap_mcbsp_alsa_drv_private_free;

	chip->mcbsp = priv->mcbsp;
	chip->c_ops = c_ops;

	snd_assert(priv->get_hw_playback);
	snd_assert(priv->get_hw_capture);
	chip->hw_playback = priv->get_hw_playback();
	chip->hw_capture = priv->get_hw_capture();

	if (priv->get_rate_constraint) {
		chip->rate_constraint = priv->get_rate_constraint();
	}

	r = (int)dma_buffer_allocate(&chip->as[SNDRV_PCM_STREAM_PLAYBACK]);
	if (r == 0) {
		printk(KERN_ERR "omap-audio: failed to allocate playback buffer\n");
		r = -ENOMEM;
		goto probe_err;
	}
	r = (int)dma_buffer_allocate(&chip->as[SNDRV_PCM_STREAM_CAPTURE]);
	if (r == 0) {
		printk(KERN_ERR "omap-audio: failed to allocate playback buffer\n");
		r = -ENOMEM;
		goto probe_err;
	}

	spin_lock_init(&chip->as[SNDRV_PCM_STREAM_PLAYBACK].dma_lock);
	spin_lock_init(&chip->as[SNDRV_PCM_STREAM_CAPTURE].dma_lock);

	r = snd_pcm_new(card, "omap-mcbsp-pcm", 0, 1, 1, &pcm);
	if (r < 0) {
		snd_card_free(card);
		return r;
	}

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, 
			&omap_mcbsp_alsa_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&omap_mcbsp_alsa_capture_ops);
	pcm->private_data = (void *)chip;
	pcm->info_flags = 0;
	strcpy(pcm->name, "omap-mcbsp-pcm");
	
	chip->as[SNDRV_PCM_STREAM_PLAYBACK].stream_id = SNDRV_PCM_STREAM_PLAYBACK;
	chip->as[SNDRV_PCM_STREAM_CAPTURE].stream_id = SNDRV_PCM_STREAM_CAPTURE;

	strcpy(card->driver, "omap-mcbsp-alsa");
	strcpy(card->shortname, priv->name);
	sprintf(card->longname, priv->name);

	snd_card_set_dev(card, &pdev->dev);

	r = snd_card_register(card);
	if (r == 0) {
		printk(KERN_INFO "audio support initialized (%s)\n", priv->name);
		platform_set_drvdata(pdev, card);

		/* init mixer */
		if (c_ops && c_ops->mixer_init) {
			c_ops->mixer_init(card);
		}
	}

	return r;

probe_err:
	if (card) {
		snd_card_free(card);
	}
	if (chip) {
		if (chip->as[SNDRV_PCM_STREAM_PLAYBACK].dma_cpu_addr) {
			dma_buffer_free(&chip->as[SNDRV_PCM_STREAM_PLAYBACK]);
		}
		if (chip->as[SNDRV_PCM_STREAM_CAPTURE].dma_cpu_addr) {
			dma_buffer_free(&chip->as[SNDRV_PCM_STREAM_CAPTURE]);
		}

		kfree(chip);
	}

	return r;
}

static int omap_mcbsp_alsa_drv_remove(struct platform_device *pdev)
{
	struct snd_card *card;

	LTRACE_ENTRY;

	card = platform_get_drvdata(pdev);

	kfree(card->private_data);
	card->private_data = NULL;
	snd_card_free(card);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int 
omap_mcbsp_alsa_drv_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct snd_card             *card;
	struct omap_mcbsp_alsa_chip *chip;

	card = platform_get_drvdata(pdev);
	if (!card)
		return -1;

	chip = (struct omap_mcbsp_alsa_chip *) (card->private_data);
	if (!chip)
		return -1;

	/*
	 * if the chip is configured, it hasnt been closed yet,
	 * so return error and it will retry when the stream is
	 * actually closed.  So do not suspend, will hang.
	 */
	if (chip->configured)
		return -1;

	if (chip->mcbsp) {
		/* TODO: Do we need to do more here? Stop TX/RX, enable
		 * autoidle?
		 */
		omap2_mcbsp_suspend(chip->mcbsp->mcbsp_id);
	}

	/* Don't think we need this call, because when
	 * streams close they should suspend the codec*/
//	if (chip->c_ops && chip->c_ops->suspend)
//		chip->c_ops->suspend();

	return 0;
}

static int 
omap_mcbsp_alsa_drv_resume(struct platform_device *pdev)
{
	struct snd_card             *card;
	struct omap_mcbsp_alsa_chip *chip;

	card = platform_get_drvdata(pdev);
	if (!card)
		return -1;

	chip = (struct omap_mcbsp_alsa_chip *) (card->private_data);
	if (!chip)
		return -1;

	if (chip->configured)
		return -1;

	/* Don't think we need this call, because
	 * when streams are opened they resume the codec */
//	if (chip->c_ops && chip->c_ops->resume)
//		chip->c_ops->resume();

	if (chip->mcbsp) {
		/* TODO: Do we need to do more here? Enable TX/RX, disable
		 * autoidle?
		 */
		omap2_mcbsp_resume(chip->mcbsp->mcbsp_id);
	}

	return 0;
}
#else
#define omap_mcbsp_alsa_drv_suspend  NULL
#define omap_mcbsp_alsa_drv_resume   NULL
#endif

static struct platform_driver omap_mcbsp_alsa_driver = {
	.probe   = omap_mcbsp_alsa_drv_probe,
	.remove  = omap_mcbsp_alsa_drv_remove,
	.suspend = omap_mcbsp_alsa_drv_suspend,
	.resume  = omap_mcbsp_alsa_drv_resume,
	.driver  = {
		.name = OMAP_MCBSP_AUDIO_NAME,
	},
};

static int __init omap_mcbsp_alsa_init(void)
{
	return platform_driver_register(&omap_mcbsp_alsa_driver);
}

static void __exit omap_mcbsp_alsa_exit(void)
{
	platform_driver_unregister(&omap_mcbsp_alsa_driver);
}

module_init(omap_mcbsp_alsa_init);
module_exit(omap_mcbsp_alsa_exit);
