#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/arch/twl4030.h>
#include <asm/arch/twl4030-audio.h>

#include "twl4030-audio.h"
#include "twl4030-audio-hw.h"
#include "audio-debug.h"
#include "script.h"

inline int twl4030_audio_write(u8 addr, u8 data)
{
	return twl4030_i2c_write_u8(TWL4030_MODULE_AUDIO_VOICE, data, addr);
}

static inline int twl4030_audio_read(u8 addr, u8 *datap)
{
	return twl4030_i2c_read_u8(TWL4030_MODULE_AUDIO_VOICE, datap, addr);
}

inline int twl4030_audio_bit_set(u8 addr, u8 set)
{
	int rc;
	u8 d;

	rc = twl4030_audio_read(addr, &d);
	if (unlikely(rc < 0)) {
		printk(KERN_INFO 
				"OMAP-AUDIO: failed to read AUDIO_VOICE addr %#x\n", addr);
		return rc;
	}

	d |= set;

	rc = twl4030_audio_write(addr, d);
	if (unlikely(rc < 0)) {
		printk(KERN_INFO 
				"OMAP-AUDIO: failed to write AUDIO_VOICE addr %#x data %#x\n", 
				addr, d);
	}

	return rc;
}

inline int twl4030_audio_bit_clr(u8 addr, u8 clr)
{
	int rc;
	u8 d;

	rc = twl4030_audio_read(addr, &d);
	if (unlikely(rc < 0)) {
		printk(KERN_INFO 
				"OMAP-AUDIO: failed to read AUDIO_VOICE addr %#x\n", addr);
		return rc;
	}

	d &= ~clr;

	rc = twl4030_audio_write(addr, d);
	if (unlikely(rc < 0)) {
		printk(KERN_INFO 
				"OMAP-AUDIO: failed to write AUDIO_VOICE addr %#x data %#x\n", 
				addr, d);
	}

	return rc;
}

static inline int twl4030_audio_bit_clr_set(u8 addr, u8 clr, u8 set)
{
	int rc;
	u8 d;

	rc = twl4030_audio_read(addr, &d);
	if (unlikely(rc < 0)) {
		printk(KERN_INFO 
				"OMAP-AUDIO: failed to read AUDIO_VOICE addr %#x\n", addr);
		return rc;
	}

	d &= ~clr;
	d |= set;

	rc = twl4030_audio_write(addr, d);
	if (unlikely(rc < 0)) {
		printk(KERN_INFO 
				"OMAP-AUDIO: failed to write AUDIO_VOICE addr %#x data %#x\n", 
				addr, d);
	}

	return rc;
}

static int twl4030_audio_pdrv_probe(struct platform_device *pdev)
{
	script_init(&pdev->dev, (struct snd_card *)pdev->dev.platform_data);
	return 0;
}

/* FIXME organization */
static struct platform_driver twl4030_audio_pdrv = {
	.probe = twl4030_audio_pdrv_probe,
	.driver = {
		.name = "twl4030_audio",
	},
};

static struct platform_device twl4030_audio_pdev = {
	.name = "twl4030_audio",
	.id = -1,
};

static int twl4030_audio_probe(void)
{
	int rc = 0;
	u8 val;

	LTRACE_ENTRY;

	rc = twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, &val, 0x0);
	if (rc) {
		rc = -ENODEV;
		printk(KERN_INFO "OMAP-AUDIO: twl4030 not detected\n");
	}

	return rc;
}

static int twl4030_audio_audio_configure(unsigned int sample_rate, 
				unsigned int data_width, unsigned int channels)
{
	int rc = 0;
	
	u8 val;

	LTRACE_ENTRY;

	switch (sample_rate) {
		case 8000:
			val = AUDIO_MODE_RATE_08_000;
			break;
		case 11025:
			val = AUDIO_MODE_RATE_11_025;
			break;
		case 12000:
			val = AUDIO_MODE_RATE_12_000;
			break;
		case 16000:
			val = AUDIO_MODE_RATE_16_000;
			break;
		case 22050:
			val = AUDIO_MODE_RATE_22_050;
			break;
		case 24000:
			val = AUDIO_MODE_RATE_24_000;
			break;
		case 32000:
			val = AUDIO_MODE_RATE_32_000;
			break;
		case 44100:
			val = AUDIO_MODE_RATE_44_100;
			break;
		case 48000:
			val = AUDIO_MODE_RATE_48_000;
			break;
		default:
			printk(KERN_ERR "OMAP-AUDIO: Invalid sample rate %u\n", 
					sample_rate);
			val = AUDIO_MODE_RATE_44_100;
			rc = -EINVAL;
			break;
	};
	twl4030_audio_bit_clr_set(REG_CODEC_MODE, BIT_CODEC_MODE_APLL_RATE_M, 
			val << BIT_CODEC_MODE_APLL_RATE);

	switch (data_width) {
		case 16:
			twl4030_audio_bit_clr_set(REG_AUDIO_IF, BIT_AUDIO_IF_DATA_WIDTH_M, 
					AUDIO_DATA_WIDTH_16SAMPLE_16DATA);
			break;
		case 24:
			twl4030_audio_bit_clr_set(REG_AUDIO_IF, BIT_AUDIO_IF_DATA_WIDTH_M,
					AUDIO_DATA_WIDTH_32SAMPLE_24DATA);
			break;
		default:
			printk(KERN_ERR "OMAP-AUDIO: Invalid data width %u\n", data_width);
			rc = -EINVAL;
			break;
	}

	return rc;
}

/* Only one callback */
static twl4030_codec_event_callback callback;
static void *callback_cookie;

int twl4030_register_codec_event_callback(twl4030_codec_event_callback cb, void *cookie)
{
	if (callback != NULL) {
		return -1;
	}
	callback = cb;
	callback_cookie = cookie;
	return 0;
}


int twl4030_audio_codec_enable(int enable)
{
	static int ref = 0;
	u8 curr_on;

	twl4030_audio_read(REG_CODEC_MODE, &curr_on);
	curr_on = (curr_on & BIT_CODEC_MODE_CODECPDZ_M) ? 1 : 0;

	if (enable) {
		ref++;
		if (curr_on)
			return 0;

		if (callback != NULL) {
			callback(callback_cookie, CODEC_ENABLED);
		}

		twl4030_audio_write(REG_APLL_CTL, BIT_APLL_CTL_APLL_EN_M | 
				AUDIO_APLL_DEFAULT << BIT_APLL_CTL_APLL_INFREQ);
		twl4030_audio_bit_set(REG_CODEC_MODE, BIT_CODEC_MODE_CODECPDZ_M);
	} else {
		if (!ref) {
			printk(KERN_WARNING
				"Trying to disable CODEC with ref count 0\n");
			return -1;
		}
		if (--ref)
			return 0;

		if (callback != NULL) {
			callback(callback_cookie, CODEC_DISABLED);
		}

		twl4030_audio_bit_clr(REG_CODEC_MODE, BIT_CODEC_MODE_CODECPDZ_M);
		twl4030_audio_write(REG_APLL_CTL, 0);
	}

	return 0;
}

static int twl4030_audio_audio_enable(bool enable)
{

	int rc = 0;

	LTRACE_ENTRY;

	if (enable) {
		/* FIXME turn on the codec here. However there's a pop every time we turn
		 * on/off the codec so maybe we should turn it on once on init, and only
		 * toggle power on suspend/resume */
		if (script_run_init() < 0) {
			/* Some random defaults. Option 2, voice and audio paths
			 * enabled, digmixing VRX to ARX2 */
			twl4030_audio_bit_clr(REG_CODEC_MODE, BIT_CODEC_MODE_OPT_MODE_M);
			twl4030_audio_write(REG_DIGMIXING, BIT_DIGMIXING_ARX2_MIXING_M);
		}

		twl4030_audio_write(REG_AUDIO_IF, BIT_AUDIO_IF_AIF_EN_M);
		twl4030_audio_codec_enable(1);

	} else {
		twl4030_audio_bit_clr(REG_AUDIO_IF, BIT_AUDIO_IF_AIF_EN_M);
		twl4030_audio_codec_enable(0);
	}

	return rc;
}

int twl4030_audio_mic_bias_enable(bool enable)
{
	static u8 mic_bias = 0;
	int rc = 0;

	LTRACE_ENTRY;

	if (enable) {
		twl4030_audio_read(REG_MICBIAS_CTL, &mic_bias);

		/* Codec needs to be on before enabling mic bias */
		twl4030_audio_codec_enable(1);
		twl4030_audio_bit_set(REG_MICBIAS_CTL, BIT_MICBIAS_CTL_HSMICBIAS_EN_M);

	} else {
		twl4030_audio_write(REG_MICBIAS_CTL, mic_bias);
		twl4030_audio_codec_enable(0);
	}

	return rc;
}

/* FIXME Big hack for experimentation */
static uint8_t codec_mode;
static uint8_t hs_gain_set;
static uint8_t hs_popn_set;

static int twl4030_audio_audio_suspend(void)
{
	u8 val;

	/* Save context */
	twl4030_audio_read(REG_CODEC_MODE,  &codec_mode);
	twl4030_audio_read(REG_HS_POPN_SET, &hs_popn_set);
	twl4030_audio_read(REG_HS_GAIN_SET, &hs_gain_set);

	/* Disable ramp */
	twl4030_audio_read(REG_HS_POPN_SET, &val);
	val &= ~BIT_HS_POPN_SET_RAMP_EN_M;
	twl4030_audio_write(REG_HS_POPN_SET, val);

	/* Disable headset output stage and gain setting */
	twl4030_audio_read(REG_HS_GAIN_SET, &val);
	val &= ~BIT_HS_GAIN_SET_HSR_GAIN_M; /* right channel */
	val &= ~BIT_HS_GAIN_SET_HSL_GAIN_M; /* left channel */
	twl4030_audio_write(REG_HS_GAIN_SET, val);

	/* Disable the bias out */
	twl4030_audio_read(REG_HS_POPN_SET, &val);
	val &= ~BIT_HS_POPN_SET_VMID_EN_M;
	twl4030_audio_write(REG_HS_POPN_SET, val);

	/* Power down the audio submodule */
	twl4030_audio_read(REG_CODEC_MODE, &val);
	val &= ~BIT_CODEC_MODE_CODECPDZ_M;
	twl4030_audio_write(REG_CODEC_MODE, val);

	return 0;
}

static int twl4030_audio_audio_resume(void)
{
	twl4030_audio_write(REG_CODEC_MODE,  codec_mode);
	twl4030_audio_write(REG_HS_GAIN_SET, hs_gain_set);
	twl4030_audio_write(REG_HS_POPN_SET, hs_popn_set);
	
	return 0;
}

static int twl4030_audio_mixer_init(struct snd_card *card);

static struct omap_alsa_codec_ops twl4030_audio_ops = {
	.probe      = twl4030_audio_probe,
	.mixer_init = twl4030_audio_mixer_init,
	.configure  = twl4030_audio_audio_configure,
	.enable     = twl4030_audio_audio_enable,
	.suspend    = twl4030_audio_audio_suspend,
	.resume     = twl4030_audio_audio_resume,
};

inline struct omap_alsa_codec_ops *twl4030_audio_get_codec_ops(void)
{
	return &twl4030_audio_ops;
}

/* HW constraints go here because it is dependent on the codec */

static struct snd_pcm_hardware twl4030_hw_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = (SNDRV_PCM_RATE_8000 | 
			SNDRV_PCM_RATE_11025 |
			SNDRV_PCM_RATE_16000 | 
			SNDRV_PCM_RATE_22050 |
			SNDRV_PCM_RATE_32000 | 
			SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000 | 
			SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_min = 128,
	.period_bytes_max = 8 * 1024,
	.periods_min = 4,
	.periods_max = 16,
	.fifo_size = 0,
};

static struct snd_pcm_hardware twl4030_hw_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = (SNDRV_PCM_RATE_8000 | 
			SNDRV_PCM_RATE_11025 |
			SNDRV_PCM_RATE_16000 | 
			SNDRV_PCM_RATE_22050 |
			SNDRV_PCM_RATE_32000 | 
			SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000 | 
			SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_min = 128,
	.period_bytes_max = 8 * 1024,
	.periods_min = 4,
	.periods_max = 16,
	.fifo_size = 0,
};

static unsigned int twl4030_rates[] = {
	8000,
	11025,
	12000,
	16000,
	22050,
	24000,
	32000,
	44100,
	48000,
};

static struct snd_pcm_hw_constraint_list twl4030_rate_constraint_list = {
	.count = ARRAY_SIZE(twl4030_rates),
	.list = twl4030_rates,
	.mask = 0,
};

inline struct snd_pcm_hardware *twl4030_audio_get_hw_playback(void)
{
	return &twl4030_hw_playback;
}

inline struct snd_pcm_hardware *twl4030_audio_get_hw_capture(void)
{
	return &twl4030_hw_capture;
}

inline 
struct snd_pcm_hw_constraint_list *twl4030_audio_get_rate_constraint(void)
{
	return &twl4030_rate_constraint_list;
}

static int twl4030_audio_mixer_init(struct snd_card *card)
{
	/* Register the twl4030 device---for scripting */
	twl4030_audio_pdev.dev.platform_data = card;
	if (platform_device_register(&twl4030_audio_pdev) < 0) {
		printk(KERN_ERR "Unable to register twl4030_audio device\n");
	}

	return 0;
}

static int __init twl4030_audio_init(void)
{
	return platform_driver_register(&twl4030_audio_pdrv);
}
subsys_initcall(twl4030_audio_init);
