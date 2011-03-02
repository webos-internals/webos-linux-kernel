#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>

#include <asm/arch/twl4030.h>
#include <asm/arch/twl4030-audio.h>

#include "twl4030-audio.h"
#include "twl4030-audio-hw.h"
#include "audio-debug.h"
#include "script.h"


static int8_t savedHFR_CTL = -1;

static int phonecallEnabled = 0; /* phone call is in progress */
static DEFINE_MUTEX(codec_lock);
static DEFINE_MUTEX(mic_lock);

static int twl4030_audio_mixer_init(struct snd_card *card);
static int twl4030_audio_audio_mute(bool mute);

inline int twl4030_audio_write(u8 addr, u8 data)
{

	if (unlikely(addr == REG_HFR_CTL)) { 
		/* save the state of the REG_HFR_CTL register 
		to stop clicks and pops on suspend/resume */
		savedHFR_CTL = data;
	}
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
		printk(KERN_ERR 
			"OMAP-AUDIO: failed to read AUDIO_VOICE addr %#x\n",
			addr);
		return rc;
	}

	d |= set;

	rc = twl4030_audio_write(addr, d);
	if (unlikely(rc < 0)) {
		printk(KERN_ERR
		"OMAP-AUDIO: failed to write AUDIO_VOICE addr %#x data %#x\n", 
		 addr, d);
	}
	if (unlikely(addr == REG_HFR_CTL)) { 
		/* save the state of the REG_HFR_CTL register 
		to stop clicks and pops on suspend/resume */
		savedHFR_CTL = d;
	}

	return rc;
}

inline int twl4030_audio_bit_clr(u8 addr, u8 clr)
{
	int rc;
	u8 d;

	rc = twl4030_audio_read(addr, &d);
	if (unlikely(rc < 0)) {
		printk(KERN_ERR
			"OMAP-AUDIO: failed to read AUDIO_VOICE addr %#x\n", 
			addr);
		return rc;
	}

	d &= ~clr;

	rc = twl4030_audio_write(addr, d);
	if (unlikely(rc < 0)) {
		printk(KERN_ERR
			"OMAP-AUDIO: failed to write AUDIO_VOICE addr %#x data %#x\n", 
			addr, d);
	}
	if (unlikely(addr == REG_HFR_CTL)){ 
		/* save the state of the REG_HFR_CTL register to stop 
		clicks and pops on suspend/resume */
		savedHFR_CTL = d;
	}
	return rc;
}

static inline int twl4030_audio_bit_clr_set(u8 addr, u8 clr, u8 set)
{
	int rc;
	u8 d;

	rc = twl4030_audio_read(addr, &d);
	if (unlikely(rc < 0)) {
		printk(KERN_ERR
			"OMAP-AUDIO: failed to read AUDIO_VOICE addr %#x\n", 
			addr);
		return rc;
	}

	d &= ~clr;
	d |= set;

	rc = twl4030_audio_write(addr, d);
	if (unlikely(rc < 0)) {
		printk(KERN_ERR
			"OMAP-AUDIO: failed to write AUDIO_VOICE addr %#x data %#x\n", 
			addr, d);
	}

	if (unlikely(addr == REG_HFR_CTL)) { 
		/* save the state of the REG_HFR_CTL register 
		to stop clicks and pops on suspend/resume */
		 savedHFR_CTL = d;
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
		printk(KERN_ERR "OMAP-AUDIO: twl4030 not detected\n");
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
static bool mic_event = false;

int 
twl4030_register_codec_event_callback(twl4030_codec_event_callback cb, 
                                      void *cookie)
{
	if (callback != NULL) {
		return -1;
	}
	callback = cb;
	callback_cookie = cookie;
	return 0;
}
/*
	Called from script when phone call begins and ends
	used to decide whether to mute back speaker or not.  
	Can be removed when correct mute/unmute scripts exist for audiod
*/
int twl4030_audio_codec_phonecall_enable(int enable)
{
	phonecallEnabled = enable;
	return twl4030_audio_codec_enable( enable );
}

static int twl4030_audio_audio_capture_enable(void)
{
	u8 cos;
	int time = 1;

	/* Anti-pop for microphone */
	twl4030_audio_bit_set(REG_ANAMICL, BIT_ANAMICL_OFFSET_CNCL_SEL_M);
	twl4030_audio_bit_set(REG_ANAMICL, BIT_ANAMICL_CNCL_OFFSET_START_M);

	while (time <= 256) {
		twl4030_audio_read(REG_ANAMICL, &cos);
		if ((cos & BIT_ANAMICL_CNCL_OFFSET_START_M) == 0)
		    break;
		msleep(time);
		time <<= 1;
	}

	return 0;
}

static int twl4030_audio_audio_suspend(void)
{
	printk(KERN_DEBUG "%s: suspending codec\n", __FUNCTION__);

	/* Mute back speaker if enabled */
	twl4030_audio_audio_mute(true);

	/* Disable ramp */
	twl4030_audio_bit_clr(REG_HS_POPN_SET, BIT_HS_POPN_SET_RAMP_EN_M);

	/* Disable headset output stage and gain setting */
	twl4030_audio_bit_clr(REG_HS_GAIN_SET, BIT_HS_GAIN_SET_HSR_GAIN_M);
	twl4030_audio_bit_clr(REG_HS_GAIN_SET, BIT_HS_GAIN_SET_HSL_GAIN_M);

	/* Disable the bias out */
	twl4030_audio_bit_clr(REG_HS_POPN_SET, BIT_HS_POPN_SET_VMID_EN_M);

	/* Power down the audio submodule */
	twl4030_audio_bit_clr(REG_CODEC_MODE, BIT_CODEC_MODE_CODECPDZ_M);

	twl4030_audio_write(REG_APLL_CTL, 0);

	return 0;
}

static int twl4030_audio_audio_resume(void)
{
	printk(KERN_DEBUG "%s: resuming codec\n", __FUNCTION__);

	/* Power up the audio submodule */
	twl4030_audio_bit_set(REG_CODEC_MODE, BIT_CODEC_MODE_CODECPDZ_M);

	/* Enable the bias out */
	twl4030_audio_bit_set(REG_HS_POPN_SET, BIT_HS_POPN_SET_VMID_EN_M);

	/* Enable headset output stage and gain setting */
	twl4030_audio_bit_set(REG_HS_GAIN_SET, BIT_HS_GAIN_SET_HSR_GAIN_M);
	twl4030_audio_bit_set(REG_HS_GAIN_SET, BIT_HS_GAIN_SET_HSL_GAIN_M);

	/* Enable ramp */
	twl4030_audio_bit_set(REG_HS_POPN_SET, BIT_HS_POPN_SET_RAMP_EN_M);

	twl4030_audio_write(REG_APLL_CTL, BIT_APLL_CTL_APLL_EN_M |
		AUDIO_APLL_DEFAULT << BIT_APLL_CTL_APLL_INFREQ);

	return 0;
}

int twl4030_audio_codec_enable(int enable)
{
	static int ref = 0;
	int ret = 0;

	mutex_lock(&codec_lock);

	if (enable) {
		if (callback != NULL && !mic_event)
			callback(callback_cookie, CODEC_ENABLED);

		if (ref++)
			goto unlock;

		twl4030_audio_audio_resume();
	} else {
		if (!ref) {
			printk(KERN_WARNING
				"Trying to disable CODEC with ref count 0\n");
			ret = -1;
			goto unlock;
		}

		if (--ref)
			goto unlock;

		if (callback != NULL && !mic_event)
			callback(callback_cookie, CODEC_DISABLED);

		twl4030_audio_audio_suspend();
	}

unlock:
	mutex_unlock(&codec_lock);

	return ret;
}

static int twl4030_audio_audio_enable(bool enable)
{
	int rc = 0;

	LTRACE_ENTRY;

	mutex_lock(&mic_lock);
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
		mic_event = false;
		twl4030_audio_write(REG_AUDIO_IF, BIT_AUDIO_IF_AIF_EN_M);
		twl4030_audio_codec_enable(1);
	} else {
		mic_event = false;
		twl4030_audio_bit_clr(REG_AUDIO_IF, BIT_AUDIO_IF_AIF_EN_M);
		twl4030_audio_codec_enable(0);
	}
	mutex_unlock(&mic_lock);

	return rc;
}

int twl4030_audio_mic_bias_enable(bool enable)
{
	int rc = 0;
	static u8 mic_bias = 0;
	static bool mic_on = false;

	LTRACE_ENTRY;

	mutex_lock(&mic_lock);
	if (enable) {
		twl4030_audio_read(REG_MICBIAS_CTL, &mic_bias);
		mic_event = true; /* prevents from disabling the codec when 
		            headset is inserted while codec is already on */

		/* Codec needs to be on before enabling mic bias, 
		   so only turn on when it is not already turned on */
		if (!mic_on)
			twl4030_audio_codec_enable(1);
		twl4030_audio_bit_set(REG_MICBIAS_CTL, BIT_MICBIAS_CTL_HSMICBIAS_EN_M);
		mic_on = true;
	} else {
		mic_event = true;
		twl4030_audio_write(REG_MICBIAS_CTL, mic_bias);
		if (mic_on)
			twl4030_audio_codec_enable(0);
		mic_on = false;
	}
	mic_event = false;
	mutex_unlock(&mic_lock);

	return rc;
}

static struct omap_alsa_codec_ops twl4030_audio_ops = {
	.probe      	= twl4030_audio_probe,
	.mixer_init 	= twl4030_audio_mixer_init,
	.configure  	= twl4030_audio_audio_configure,
	.enable     	= twl4030_audio_audio_enable,
	.capture_enable = twl4030_audio_audio_capture_enable,
	.suspend    	= twl4030_audio_audio_suspend,
	.resume     	= twl4030_audio_audio_resume,
	.mute       	= twl4030_audio_audio_mute,
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
/* twl4030_audio_audio_mute  
   added sequencing for rear speaker to stop clicks and pops - 
   if speaker is active shown by: BIT_HFR_CTL_HFR_HB_EN_M
*/
static int twl4030_audio_audio_mute(bool mute)
{
	int rc = 0;
	u8 hfr;
	twl4030_audio_read(REG_HFR_CTL, &hfr); 
	LTRACE_ENTRY;

	if (mute && ( 0 == phonecallEnabled ) ) { 
		/* only mute back speaker if we are not on a call */
		if ( 0 != (hfr & BIT_HFR_CTL_HFR_HB_EN_M ) ) {
			twl4030_audio_bit_clr(REG_HFR_CTL, BIT_HFR_CTL_HFR_LOOP_EN_M  | BIT_HFR_CTL_HFR_REF_EN_M );
			twl4030_audio_bit_clr(REG_HFR_CTL, BIT_HFR_CTL_HFR_RAMP_EN_M );
			twl4030_audio_bit_clr(REG_HFR_CTL, BIT_HFR_CTL_HFR_REF_EN_M  );
			savedHFR_CTL = hfr;
		}
	}
	else {  /* unmute sequence for rear speaker */
		if ( 0 != (savedHFR_CTL & BIT_HFR_CTL_HFR_HB_EN_M ) ) {
			twl4030_audio_bit_set(REG_HFR_CTL, BIT_HFR_CTL_HFR_REF_EN_M);
			twl4030_audio_bit_set(REG_HFR_CTL, BIT_HFR_CTL_HFR_RAMP_EN_M);
			twl4030_audio_bit_set(REG_HFR_CTL, BIT_HFR_CTL_HFR_LOOP_EN_M);
			twl4030_audio_bit_set(REG_HFR_CTL, BIT_HFR_CTL_HFR_HB_EN_M);
		}
	}
	return rc;
}

