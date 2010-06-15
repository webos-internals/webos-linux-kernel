/*
 * sound/arm/omap/omap2-audio.c
 *
 * Common ALSA audio handling for the OMAP processors
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 * Based on omap linux open source community alsa driver 
 * sound/arm/omap/omap-alsa.c
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil 
 * Based on sa11xx-uda1341.c,
 * Copyright (C) 2002 Tomas Kasparek <tomas.kasparek@seznam.cz> 
 *
 */

// #include <linux/config.h>
#include <linux/autoconf.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>
// #include <asm/arch/bus.h>
#include <asm/arch/clock.h>
#include <asm/arch/omap2_mcbsp.h>

#include "omap2-audio_if.h"

/*
 * Buffer management for alsa and dma
 */
struct omap_alsa_stream {
	int stream_id;		/* numeric identification */
	int active:1;		/* we are using this stream for transfer now */
	int period;		/* current transfer period */
	int periods;		/* current count of periods registerd in the DMA engine */
	spinlock_t dma_lock;	/* for locking in DMA operations */
	int offset;		/* store start position of the last period in the alsa buffer */
	struct snd_pcm_substream *stream;	/* the pcm stream */
};

struct omap_alsa_state {
	struct snd_card *card;
	struct snd_pcm *pcm;
	long samplerate;
	struct omap_alsa_stream s[2];	/* playback & capture */
	struct omap_alsa_codec *codec;
};

/***************************** MACROS ************************************/
/* Change to define if need be */
#undef DEBUG

#ifdef DEBUG
#define DPRINTK  printk
#define FN_IN printk("[omap_audio.c:[%s] start\n", __FUNCTION__)
#define FN_OUT(n) printk("[omap_audio.c:[%s] end(%d)\n", __FUNCTION__ , n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(x)
#endif

#define OMAP_AUDIO_NAME		"omap-audio"
#define DMA_NB_FRAGS	16
#define DMA_FRAG_SIZE	(1024 * 8)

static int audio_probe(struct platform_device *dev);
static int audio_remove(struct platform_device *dev);
static void audio_release(struct device *dev);

#ifdef CONFIG_PM
static int audio_suspend(struct platform_device *dev, pm_message_t state);
static int audio_resume(struct platform_device *dev);
#endif

#if 0
static struct omap_driver omap_audio_driver = {
	.drv = {
		.name = OMAP_AUDIO_NAME,
		},
	.devid = OMAP24xx_AUDIO_DEVID,
	.busid = OMAP_BUS_L3,
	.clocks = 0,
	.probe = audio_probe,
#ifdef CONFIG_PM        
        .suspend = audio_suspend,
        .resume = audio_resume,
#endif  
	.remove = audio_remove,
};

static struct omap_dev omap_audio_device = {
	.name = OMAP_AUDIO_NAME,
	.devid = OMAP24xx_AUDIO_DEVID,
	.busid = OMAP_BUS_L3,
	.dev = {
		/* We might add additional things in future.. */
		.release = audio_release,
		},
};
#else
static struct platform_driver omap_audio_driver = {
	.probe = audio_probe,
	.remove = audio_remove,
#ifdef CONFIG_PM
	.suspend = audio_suspend,
	.resume = audio_resume,
#endif
	.driver = {
		.name = OMAP_AUDIO_NAME,
	},
};

static struct platform_device omap_audio_devices[] = {
	{
		/* twl4030 */
		.name = OMAP_AUDIO_NAME,
		.id = 7,
		.dev = {
			.release = audio_release,
		}
	},
#if defined(CONFIG_MACH_BRISKET) | defined(CONFIG_MACH_FLANK)
	{
		.name = OMAP_AUDIO_NAME,
		.id = 8,
		.dev = {
			.release = audio_release,
		}
	},
	{
		.name = OMAP_AUDIO_NAME,
		.id = 9,
		.dev = {
			.release = audio_release,
		}
	},
#endif
#if defined(CONFIG_MACH_BRISKET)
	{
		/* modem */
		.name = OMAP_AUDIO_NAME,
		.id = 10,
		.dev = {
			.release = audio_release,
		}
	},
#endif
	{
		NULL
	}
};
static struct platform_device *omap_audio_device = omap_audio_devices;

#endif

#ifdef CONFIG_PM

// static int dpm_state;
pm_message_t dpm_state;

/** 
 * @brief audio_suspend - Function to handle suspend operations
 * 
 * @param dev 
 * @param state 
 * 
 * @return 
 */
static int audio_suspend(struct platform_device *dev, pm_message_t state)
{
	int ret = 0;
	struct omap_alsa_state *chip;
	struct snd_card *card = platform_get_drvdata(dev);
	struct omap_alsa_codec *codec;
	struct omap_alsa_state *omap_audio_state = (struct omap_alsa_state *)dev->dev.platform_data;

	/* XXX I think chip is the same as omap_audio_state ... */

	dpm_state = state;
	chip = card->private_data;
	codec = chip->codec;

	if (card->power_state != SNDRV_CTL_POWER_D3) {
		if (chip->card->power_state != SNDRV_CTL_POWER_D3) {
			snd_power_change_state(chip->card,
					SNDRV_CTL_POWER_D3);
			snd_pcm_suspend_all(chip->pcm);
			if (omap_audio_state->s[SNDRV_PCM_STREAM_PLAYBACK].active) {
				ret = codec->codec_transfer_stop(SNDRV_PCM_STREAM_PLAYBACK, codec);
				codec->codec_shutdown(codec);
			}
			if (omap_audio_state->s[SNDRV_PCM_STREAM_CAPTURE].active) {
				ret = codec->codec_transfer_stop(SNDRV_PCM_STREAM_CAPTURE, codec);
				codec->codec_shutdown(codec);
			}
		}
	}

	return ret;
}

/** 
 * @brief audio_resume - Function to handle resume operations
 * 
 * @param dev 
 * 
 * @return 
 */

static int audio_resume(struct platform_device *dev)
{
	int ret = 0;
	struct omap_alsa_state *chip;
	struct snd_card *card = platform_get_drvdata(dev);
	struct omap_alsa_codec *codec; 
	struct omap_alsa_state *omap_audio_state = (struct omap_alsa_state *)dev->dev.platform_data;

	chip = card->private_data;
	codec = chip->codec;

	if (card->power_state != SNDRV_CTL_POWER_D0) {                          
		if (chip->card->power_state != SNDRV_CTL_POWER_D0) {
			if (omap_audio_state->s[SNDRV_PCM_STREAM_PLAYBACK].active) {
				codec->codec_init(codec);
				ret = codec->codec_transfer_init(SNDRV_PCM_STREAM_PLAYBACK, codec);
			}
			if (omap_audio_state->s[SNDRV_PCM_STREAM_CAPTURE].active) {
				codec->codec_init(codec);
				ret = codec->codec_transfer_init(SNDRV_PCM_STREAM_CAPTURE, codec);
			}
			snd_power_change_state(chip->card, SNDRV_CTL_POWER_D0);
		}
	}
#if 0
	/* TODO */
	dpm_state = (pm_message_t)0;

#endif
	return ret;        
}
#else
#define audio_suspend NULL
#define audio_resume NULL
#endif  

// static u_int audio_get_dma_pos(struct omap_alsa_stream *s)
static snd_pcm_uframes_t audio_get_dma_pos(struct omap_alsa_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct omap_alsa_state *state =
	    snd_pcm_substream_chip(substream);
	struct omap_alsa_codec *codec = state->codec;	
	// unsigned int offset;
	snd_pcm_uframes_t offset;
	unsigned long flags;
	int ret = 0;

	FN_IN;
	if (unlikely(!s)) {
		printk(KERN_ERR "Stream IS NULL!!\n");
		return -EPERM;
	}
	/* we always ask only one frame to transmit/recieve,
	 * variant is the element num 
	 */
	/* this must be called w/ interrupts locked as requested in dma.c */
	spin_lock_irqsave(&s->dma_lock, flags);
	ret = codec->codec_transfer_posn(s->stream_id, codec);
 	spin_unlock_irqrestore(&s->dma_lock, flags);

	/* Now, the position related to the end of that period */
#if 1
	offset = bytes_to_frames(runtime, s->offset) - bytes_to_frames(runtime, ret);
	printk("o %x r %d of %x p %d\n", offset, ret, s->offset, s->periods);
#else
	offset = bytes_to_frames(runtime, (unsigned int)ret - (unsigned int)runtime->dma_addr);
	printk("offset %u csaci %p dma_addr %p\n", offset, ret, runtime->dma_addr);
#endif

	if (offset >= runtime->buffer_size)
		offset = 0;

#if 0
	if (ret < 0) {
		printk(KERN_ERR
		       "codec_transfer_posn: Unable to find index of "
		       "transfer\n");
	}
#endif
	return offset;
}

#include <linux/time.h>

/*
 *  Main dma routine, requests dma according where you are in main alsa buffer
 */
static int audio_process_dma(struct omap_alsa_stream *s)
{
	struct snd_pcm_substream *substream = s->stream;
	struct snd_pcm_runtime *runtime;
	struct omap_alsa_state *state =
	    snd_pcm_substream_chip(substream);
	struct omap_alsa_codec *codec = state->codec;	
	unsigned int dma_size;
	unsigned int offset;
	int ret;
	
	FN_IN;
	runtime = substream->runtime;
	if (s->active) {
#if 1
		dma_size = frames_to_bytes(runtime, runtime->period_size);
		offset = dma_size * s->period;
		snd_assert(dma_size <= DMA_FRAG_SIZE,);
		ret = codec->codec_transfer_start(s->stream_id,
					(void *) (runtime->dma_addr) +
					 offset, dma_size, s, codec);
		struct timeval tv;
		do_gettimeofday(&tv);
		//printk("r %u.%u %p %x %u\n", tv.tv_sec, tv.tv_usec, (void *)runtime->dma_addr + offset, dma_size, s->period);
		if (ret) {
#if 0
			printk(KERN_ERR
			       "audio_process_dma: cannot queue DMA buffer (%i)\n",
			       ret);
#endif
			return ret;
		}

		s->period++;
		s->period %= runtime->periods;
		s->periods++;
		s->offset = offset;
#else
		dma_size = runtime->dma_bytes / 2;
		offset = dma_size * s->period;
		ret = codec->codec_transfer_start(s->stream_id,
				(void *)(runtime->dma_addr) + offset, dma_size, s, codec);
		struct timeval tv;
		do_gettimeofday(&tv);
		printk("r %u.%u %p %x %u\n", tv.tv_sec, tv.tv_usec, (void *)runtime->dma_addr + offset, dma_size, s->period);
		if (ret) {
			printk(KERN_ERR "audio_process_dma: can't queue dma %i\n", ret);
			return ret;
		}
		s->period++;
		s->period %= 2;
		s->periods++;
		s->offset = offset;
#endif
	}

	return 0;
}


/* 
 *  This is called when dma IRQ occurs at the end of each transmited block
 */
void audio_period_handler(void *arg)
{
	struct omap_alsa_stream *s = arg;
	
	FN_IN;
	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	spin_lock(&s->dma_lock);
	if (s->periods > 0) 
		s->periods--;
	
	//while (audio_process_dma(s) == 0) { }
	audio_process_dma(s);
	spin_unlock(&s->dma_lock);
}

/* 
 * Alsa section
 * PCM settings and callbacks
 */
static int snd_omap_alsa_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int stream_id = substream->pstr->stream;
	struct omap_alsa_state *state =
	    snd_pcm_substream_chip(substream);
	struct omap_alsa_codec *codec = state->codec;	
	struct omap_alsa_stream *s = &state->s[stream_id];
	int err = 0;
	unsigned long flags;
	
	FN_IN;
	/* note local interrupts are already disabled in the midlevel code */
	spin_lock(&s->dma_lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/* requested stream startup */
		s->active = 1;
		audio_process_dma(s);
		/* hnagalla -- 04/02/07 -- queue one more to get rid of poping noise */
		audio_process_dma(s);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		/* requested stream shutdown */
		spin_lock_irqsave(&s->dma_lock, flags);
		s->active = 0;
		s->period = 0;
		s->periods = 0;
		/* TODO: need return ret code? */
		codec->codec_transfer_stop(s->stream_id, codec);
		spin_unlock_irqrestore(&s->dma_lock, flags);
		break;
	default:
		err = -EINVAL;
		break;
	}
	spin_unlock(&s->dma_lock);
	
	return err;
}

static int snd_omap_alsa_prepare(struct snd_pcm_substream * substream)
{
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_codec *codec = state->codec; 
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct omap_alsa_stream *s = &state->s[substream->pstr->stream];
	
	FN_IN;
	/* set requested samplerate */
	codec->codec_set_samplerate(runtime->rate, codec);
	state->samplerate = runtime->rate;
	if (runtime->channels == 1) {
		/* Set Mono_mode, with DSP option */
		codec->codec_set_stereomode(0x02, 0x01, codec); 
	}
	else {
		/* Set Stereo_mode, with DSP option */
		codec->codec_set_stereomode(0x01, 0x01, codec);
	} 
	s->period = 0;
	s->periods = 0;

	return 0;
}

static  snd_pcm_uframes_t snd_omap_alsa_pointer(struct snd_pcm_substream *substream)
{
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);

	FN_IN;	
	return audio_get_dma_pos(&state->s[substream->pstr->stream]);
}

static int snd_card_omap_alsa_open(struct snd_pcm_substream * substream)
{
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_codec *codec = state->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	int err;
	
	FN_IN;

	codec->codec_init(codec);

	state->s[stream_id].stream = substream;
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) { 
		runtime->hw = *(codec->pcm_hardware_playback);
	}
	else { 
		runtime->hw = *(codec->pcm_hardware_capture);
	}
		
	if ((err = snd_pcm_hw_constraint_integer(runtime,
					   SNDRV_PCM_HW_PARAM_PERIODS)) < 0) 
		return err;
	
	if ((err = snd_pcm_hw_constraint_list(runtime,
					0,
					SNDRV_PCM_HW_PARAM_RATE,
					codec->pcm_hw_constraint_list)) < 0) 
		return err;

	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 128);
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_SIZE, 128);
	
	if ((err = codec->codec_transfer_init(stream_id, codec))) {
		printk(KERN_ERR " IS init failed!! [%d]\n", err);
		return err;
	}

	return 0;
}

static int snd_card_omap_alsa_close(struct snd_pcm_substream * substream)
{
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_codec *codec = state->codec; 
	
	FN_IN;
	codec->codec_shutdown(codec);
	state->s[substream->pstr->stream].stream = NULL;
	
	return 0;
}

/* HW params & free */
static int snd_omap_alsa_hw_params(struct snd_pcm_substream * substream,
				    struct snd_pcm_hw_params * hw_params)
{
	struct snd_pcm_runtime  *runtime = substream->runtime;
	size_t size = 0;


	FN_IN;

	size = params_buffer_bytes(hw_params);
	if (size && runtime->dma_area) {
		/* already allocated */
		if (runtime->dma_bytes >= size)
			return 0; /* already large enough */
		dma_free_coherent(NULL,runtime->dma_bytes,runtime->dma_area,runtime->dma_addr);
	}
	runtime->dma_area = dma_alloc_coherent(NULL, size,&runtime->dma_addr,GFP_KERNEL | GFP_DMA);
	if (runtime->dma_area)
	{
		memset((void *)runtime->dma_area,0,size);
		runtime->dma_bytes = size;
	}

	printk("%s: buffer %p addr %p size %x\n", __func__, runtime->dma_area, runtime->dma_addr, size);
	return 0;
#if 0
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
#endif
}

static int snd_omap_alsa_hw_free(struct snd_pcm_substream * substream)
{

	struct snd_pcm_runtime  *runtime = substream->runtime;

	FN_IN;

	if (runtime->dma_area && runtime->dma_bytes && runtime->dma_addr)
	{
		dma_free_coherent(NULL,runtime->dma_bytes,runtime->dma_area,runtime->dma_addr);
		runtime->dma_area = NULL;
		runtime->dma_bytes = 0;
		runtime->dma_area = NULL;
       }
       return 0;
#if 0
	return snd_pcm_lib_free_pages(substream);
#endif
}

/* pcm operations */
static struct snd_pcm_ops snd_card_omap_alsa_playback_ops = {
	.open =		snd_card_omap_alsa_open,
	.close =	snd_card_omap_alsa_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_omap_alsa_hw_params,
	.hw_free =	snd_omap_alsa_hw_free,
	.prepare =	snd_omap_alsa_prepare,
	.trigger =	snd_omap_alsa_trigger,
	.pointer =	snd_omap_alsa_pointer,
};

static struct snd_pcm_ops snd_card_omap_alsa_capture_ops = {
	.open =		snd_card_omap_alsa_open,
	.close =	snd_card_omap_alsa_close,
	.ioctl =	snd_pcm_lib_ioctl,
	.hw_params =	snd_omap_alsa_hw_params,
	.hw_free =	snd_omap_alsa_hw_free,
	.prepare =	snd_omap_alsa_prepare,
	.trigger =	snd_omap_alsa_trigger,
	.pointer =	snd_omap_alsa_pointer,
};

/*
 *  Alsa init and exit section
 *  
 *  Inits pcm alsa structures, allocate the alsa buffer, suspend, resume
 */
static int snd_card_omap_alsa_pcm(struct omap_alsa_state *state, int device)
{
	struct snd_pcm *pcm;
	int err;
	
	FN_IN;
	if ((err = snd_pcm_new(state->card, "OMAP PCM", device, 1, 1, &pcm)) < 0)
		return err;

#if 0 
	/* sets up initial buffer with continuous allocation */
	snd_pcm_lib_preallocate_pages_for_all(pcm,
					      SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL),
					      DMA_FRAG_SIZE * DMA_NB_FRAGS,
					      DMA_FRAG_SIZE * DMA_NB_FRAGS);

#endif
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_card_omap_alsa_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_card_omap_alsa_capture_ops);
	pcm->private_data = state;
	pcm->info_flags = 0;
	strcpy(pcm->name, "omap alsa pcm");

	state->s[SNDRV_PCM_STREAM_PLAYBACK].stream_id = SNDRV_PCM_STREAM_PLAYBACK;
	state->s[SNDRV_PCM_STREAM_CAPTURE].stream_id = SNDRV_PCM_STREAM_CAPTURE;

	state->pcm = pcm;

	return 0;
}


void snd_omap_alsa_free(struct snd_card * card)
{
	FN_IN;
	
	/*
	 * Turn off codec after it is done.
	 * Can't do it immediately, since it may still have
	 * buffered data.
	 */
	schedule_timeout_interruptible(2);
}

/** 
 * @brief audio_probe - The Audio driver probe function
 * WARNING!!!!  : It is expected that the codec would have registered with us 
 *                by now
 * 
 * @param dev 
 * 
 * @return 
 */
static int audio_probe(struct platform_device *dev)
{
	int err = 0;
	struct snd_card *card;
	struct omap_alsa_state *state = dev->dev.platform_data;
	struct omap_alsa_codec *codec = state->codec; 
	
	FN_IN;

	if (codec->codec_probe())
		return -ENODEV;
 
	/* register the soundcard */
	card = snd_card_new(-1, NULL, THIS_MODULE, sizeof(state));
	if (card == NULL)
		goto nodev1;

	card->private_data = (void *)state;
	card->private_free = snd_omap_alsa_free;

	state->card = card;
	state->samplerate = codec->codec_default_samplerate();

	spin_lock_init(&state->s[SNDRV_PCM_STREAM_PLAYBACK].dma_lock);
	spin_lock_init(&state->s[SNDRV_PCM_STREAM_CAPTURE].dma_lock);

	/* PCM */
	if ((err = snd_card_omap_alsa_pcm(state, 0)) < 0)
		goto nodev3;

	strcpy(card->driver, "OMAP_ALSA");
	strcpy(card->shortname, codec->name);
	sprintf(card->longname, codec->name);

	// snd_card_set_dev(card, (struct device *)dev);
	snd_card_set_dev(card, &dev->dev);
	
	if ((err = snd_card_register(card)) == 0) {
		printk(KERN_INFO "audio support initialized\n");
		platform_set_drvdata(dev, card);
		codec->mixer_init(card);
		return 0;
	}
	
nodev3:
	snd_card_free(card);
nodev1:
	FN_OUT(err);
	return err;
}

/** 
 * @brief audio_remove - Function to handle removal operations
 * 
 * @param dev 
 * 
 * @return 
 */
static int audio_remove(struct platform_device *dev)
{
	struct snd_card *card = platform_get_drvdata(dev);
	
	FN_IN;
	card->private_data = NULL;
	snd_card_free(card);
	platform_set_drvdata(dev, NULL);
	
	FN_OUT(0);
	return 0;
}

static void audio_release(struct device *dev)
{
	/* Nothing to Release! */
}

/** 
 * @brief audio_register_codec - Register a Codec fn points using this function
 * WARNING!!!!!          : Codecs should ensure that they do so! no sanity 
 *                         checks during runtime is done due to obvious 
 *                         performance penalties.
 */
int audio_register_codec(struct omap_alsa_codec *codec)
{
	int ret;
	struct omap_alsa_state *omap_audio_state;

	FN_IN;

#if 0
	/* XXX only do this once */
	ret = platform_driver_register(&omap_audio_driver);
	if (ret) {
		printk(KERN_ERR "OMAP Audio Driver register failed =%d\n", ret);
		/* continue */
	}
#endif

#if 0
	/* We really want one driver to support multiple codecs. But omap bus
	 * match code won't allow us to do it
	 */
	if (omap_audio_state) {
		printk(KERN_ERR "OMAP Audio only supports one codec now\n");
		return -EINVAL;
	}
#endif

	if (!codec)
		return -EINVAL;

	/* TODO: more sanity check on the codec passed to us */

	if (omap_audio_device->name) {
		omap_audio_state = kmalloc(sizeof(*omap_audio_state), GFP_KERNEL);
		if (!omap_audio_state)
			return -ENOMEM;
		memset(omap_audio_state, 0, sizeof(*omap_audio_state));
		
		/* setup pointers */
		omap_audio_state->codec = codec;
		omap_audio_device->dev.platform_data = omap_audio_state; 
		
		ret = platform_device_register(omap_audio_device);
		if (ret) {
			printk(KERN_ERR "OMAP Audio Device Register failed =%d\n", ret);
			kfree(omap_audio_state);
			omap_audio_state = NULL;
			return ret;
		}

		omap_audio_device += 1;
	}

	return 0;
}

/** 
 * @brief audio_unregister_codec - Un-Register a Codec using this function
 * 
 * @param codec_state 
 * 
 * @return 
 */
int audio_unregister_codec(struct omap_alsa_codec *codec)
{
	/* XXX rewrite this */
#if 0
	if (!omap_audio_state || omap_audio_state->codec != codec) {
		printk(KERN_ERR "Bad codec unregister call\n");
		return -EINVAL;
	}

	platform_device_unregister(&omap_audio_device);
	kfree(omap_audio_state);
	omap_audio_state = NULL;

	/* clean up any codec device that isn't removed by its driver */
	if (omap_audio_state) {
		platform_device_unregister(&omap_audio_device);
		kfree(omap_audio_state);
	}
	platform_driver_unregister(&omap_audio_driver);

#endif
	return 0;
}

static int __init omap2_audio_init(void)
{
	return platform_driver_register(&omap_audio_driver);
}

static void __exit omap2_audio_exit(void)
{
	platform_driver_unregister(&omap_audio_driver);
}

module_init(omap2_audio_init);
module_exit(omap2_audio_exit);

EXPORT_SYMBOL(audio_register_codec);
EXPORT_SYMBOL(audio_unregister_codec);
EXPORT_SYMBOL(audio_period_handler);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ALSA audio handling for OMAP processors");
MODULE_LICENSE("GPL");
