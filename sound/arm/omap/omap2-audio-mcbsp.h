#ifndef __OMAP_AUDIO_MCBSP_H__
#define __OMAP_AUDIO_MCBSP_H__

#include "omap2-audio-mcbsp_if.h"

#define AUDIO_SAMPLE_DATA_WIDTH_16               16
#define AUDIO_SAMPLE_DATA_WIDTH_24               24

/* Modem/BT provides the clock */
#define TWL_MASTER

static int mcbsp_conf_data_interface(struct omap_alsa_codec *codec);
static void mcbsp_dma_cb(u16 ch_status, void *arg);
static int audio_mcbsp_transfer(int mode, void *buffer_phy,u32 size, void *arg, void *id);
static int audio_mcbsp_transfer_stop(int mode, void *id);
static int audio_mcbsp_transfer_posn(int mode, void *id);
static int audio_mcbsp_transfer_init(int mode, void *id);
static int audio_mcbsp_initialize(void *id);
static void audio_mcbsp_shutdown(void *id);
static int audio_mcbsp_sidle(u32 level);
static int audio_mcbsp_probe(void);
static int audio_mcbsp_set_samplerate(long, void *id);
static int audio_mcbsp_stereomode_set(int mode, int dsp, void *);

static int audio_mcbsp_mixer_init(struct snd_card *card); 
static int audio_mcbsp_mixer_shutdown(struct snd_card *card); 


/* Should go in board specific data */

static unsigned int mcbsp_rates[] = {
	8000, 11025, 12000,
	16000, 22050, 24000,
	32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list mcbsp_pcm_hw_constraint_list = {
	.count = ARRAY_SIZE(mcbsp_rates),
	.list = mcbsp_rates,
	.mask = 0,
};

static struct snd_pcm_hardware mcbsp_pcm_hardware_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER),
 	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000),
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
#ifdef CONFIG_SND_OMAP_3430DASF
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 128,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 125,
#else
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 2,
	.periods_max = 16,
#endif
	.fifo_size = 0,
};

static struct snd_pcm_hardware mcbsp_pcm_hardware_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000),
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 2,
	.periods_max = 16,
	.fifo_size = 0,
};


#define AUDIO_RATE_DEFAULT                       8000

static int audio_mcbsp_default_samplerate(void)
{
	return AUDIO_RATE_DEFAULT;
}

#define STEREO_MODE                              (0x1)
#define MONO_MODE                                (0x2)

/* To store characteristic info regarding the codec for the audio driver */
static struct omap_alsa_codec audio_mcbsp_codecs[] = {
	{
		/* bluetooth */
		.name = "mcbsp-bluetooth",

		.stereomode = STEREO_MODE,
		.samplerate = AUDIO_RATE_DEFAULT,

		.pcm_hw_constraint_list = &mcbsp_pcm_hw_constraint_list,
		.pcm_hardware_playback = &mcbsp_pcm_hardware_playback,
		.pcm_hardware_capture = &mcbsp_pcm_hardware_capture,

		.codec_probe = audio_mcbsp_probe,
		.codec_init = audio_mcbsp_initialize,
		.codec_shutdown = audio_mcbsp_shutdown,
		.codec_sidle = audio_mcbsp_sidle,      

		.codec_set_samplerate = audio_mcbsp_set_samplerate, 
#ifdef CONFIG_SND_OMAP_3430SDP
		.codec_set_stereomode = audio_mcbsp_stereomode_set, 
#endif	
		.codec_default_samplerate = audio_mcbsp_default_samplerate,

		.codec_transfer_init = audio_mcbsp_transfer_init,
		.codec_transfer_start = audio_mcbsp_transfer,
		.codec_transfer_stop = audio_mcbsp_transfer_stop,
		.codec_transfer_posn = audio_mcbsp_transfer_posn,

		.mixer_init = audio_mcbsp_mixer_init,
		.mixer_shutdown = audio_mcbsp_mixer_shutdown,
	},
#if defined(CONFIG_MACH_BRISKET) | defined(CONFIG_MACH_FLANK)
	{
		/* modem */
		.name = "mcbsp-modem",

		.stereomode = STEREO_MODE,
		.samplerate = AUDIO_RATE_DEFAULT,

		.pcm_hw_constraint_list = &mcbsp_pcm_hw_constraint_list,
		.pcm_hardware_playback = &mcbsp_pcm_hardware_playback,
		.pcm_hardware_capture = &mcbsp_pcm_hardware_capture,

		.codec_probe = audio_mcbsp_probe,
		.codec_init = audio_mcbsp_initialize,
		.codec_shutdown = audio_mcbsp_shutdown,
		.codec_sidle = audio_mcbsp_sidle,      

		.codec_set_samplerate = audio_mcbsp_set_samplerate, 
#ifdef CONFIG_SND_OMAP_3430SDP
		.codec_set_stereomode = audio_mcbsp_stereomode_set, 
#endif	
		.codec_default_samplerate = audio_mcbsp_default_samplerate,

		.codec_transfer_init = audio_mcbsp_transfer_init,
		.codec_transfer_start = audio_mcbsp_transfer,
		.codec_transfer_stop = audio_mcbsp_transfer_stop,
		.codec_transfer_posn = audio_mcbsp_transfer_posn,

		.mixer_init = audio_mcbsp_mixer_init,
		.mixer_shutdown = audio_mcbsp_mixer_shutdown,
	},
#endif
#if defined(CONFIG_MACH_BRISKET)
	{
		/* twl4030 pcm */
		.name = "mcbsp-twl4030-pcm",

		.stereomode = STEREO_MODE,
		.samplerate = AUDIO_RATE_DEFAULT,

		.pcm_hw_constraint_list = &mcbsp_pcm_hw_constraint_list,
		.pcm_hardware_playback = &mcbsp_pcm_hardware_playback,
		.pcm_hardware_capture = &mcbsp_pcm_hardware_capture,

		.codec_probe = audio_mcbsp_probe,
		.codec_init = audio_mcbsp_initialize,
		.codec_shutdown = audio_mcbsp_shutdown,
		.codec_sidle = audio_mcbsp_sidle,      

		.codec_set_samplerate = audio_mcbsp_set_samplerate, 
#ifdef CONFIG_SND_OMAP_3430SDP
		.codec_set_stereomode = audio_mcbsp_stereomode_set, 
#endif	
		.codec_default_samplerate = audio_mcbsp_default_samplerate,

		.codec_transfer_init = audio_mcbsp_transfer_init,
		.codec_transfer_start = audio_mcbsp_transfer,
		.codec_transfer_stop = audio_mcbsp_transfer_stop,
		.codec_transfer_posn = audio_mcbsp_transfer_posn,

		.mixer_init = audio_mcbsp_mixer_init,
		.mixer_shutdown = audio_mcbsp_mixer_shutdown,
	},
#endif
	{ NULL },
};

static struct mcbsp_codec_data mcbsp_codec_data[] = {
	{
		/* bluetooth */
		.mcbsp_id = OMAP2_MCBSP_INTERFACE4,
		.mcbsp_config = {
			.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
#ifdef TWL_MASTER
			.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
			.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
			.fsx_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
			.fsr_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
			.clkr_src = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
			.clkx_src = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,
#else
			.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
			.srg_clk_sync = OMAP2_MCBSP_SRG_RUNNING,
			.fsx_src = OMAP2_MCBSP_TXFSYNC_INTERNAL,
			.fsr_src = OMAP2_MCBSP_RXFSYNC_INTERNAL,
			.clkr_src = OMAP2_MCBSP_CLKRXSRC_INTERNAL,	
			.clkx_src = OMAP2_MCBSP_CLKTXSRC_INTERNAL,
#endif
			.fsx_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
			.fsr_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
			.srg_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
			.clkx_pol = OMAP2_MCBSP_CLKX_POLARITY_RISING,
			.clkr_pol = OMAP2_MCBSP_CLKR_POLARITY_RISING,
			.tx_params = {
				.data_type = OMAP2_MCBSP_WORDLEN_NONE,
				.skip_alt = OMAP2_MCBSP_SKIP_NONE,
				.auto_reset = OMAP2_MCBSP_AUTO_XRST,
				.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
				.data_delay = OMAP2_MCBSP_DATADELAY0,
				.reverse_compand = OMAP2_MCBSP_MSBFIRST,
				.word_length1 = OMAP2_MCBSP_WORDLEN_16,
				.word_length2 = 0,
				.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
				.frame_length2 = 0,
				.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
				.dxena = 0,
				.dxendly = 0,
				.callback = mcbsp_dma_cb,
			},
			.rx_params = {
				.data_type = OMAP2_MCBSP_WORDLEN_NONE,
				.skip_alt = OMAP2_MCBSP_SKIP_NONE,
				.auto_reset = OMAP2_MCBSP_AUTO_RRST,
				.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
				.data_delay = OMAP2_MCBSP_DATADELAY0,
				.reverse_compand = OMAP2_MCBSP_MSBFIRST,
				.word_length1 = OMAP2_MCBSP_WORDLEN_16,
				.word_length2 = 0,
				.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
				.frame_length2 = 0,
				.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
				.dxena = 0,
				.dxendly = 0,
				.callback = mcbsp_dma_cb,
				},
		},
	},
#if defined(CONFIG_MACH_BRISKET) | defined(CONFIG_MACH_FLANK)
	{
		/* modem */
		.mcbsp_id = OMAP2_MCBSP_INTERFACE5,
		.mcbsp_config = {
			.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
#ifdef TWL_MASTER
			.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
			.fsx_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
			.fsr_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
			.clkr_src = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
			.clkx_src = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,

			.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
#else
			.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
			.fsx_src = OMAP2_MCBSP_TXFSYNC_INTERNAL,
			.fsr_src = OMAP2_MCBSP_RXFSYNC_INTERNAL,
			.clkr_src = OMAP2_MCBSP_CLKRXSRC_INTERNAL,	
			.clkx_src = OMAP2_MCBSP_CLKTXSRC_INTERNAL,

			.srg_clk_sync = OMAP2_MCBSP_SRG_RUNNING,
#endif
			.fsx_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
			.fsr_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
			.srg_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
			.clkx_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
			.clkr_pol = OMAP2_MCBSP_CLKR_POLARITY_FALLING,
			.tx_params = {
				.data_type = OMAP2_MCBSP_WORDLEN_NONE,
				.skip_alt = OMAP2_MCBSP_SKIP_NONE,
				.auto_reset = OMAP2_MCBSP_AUTO_XRST,
				.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
				.data_delay = OMAP2_MCBSP_DATADELAY1,
				.reverse_compand = OMAP2_MCBSP_MSBFIRST,
				.word_length1 = OMAP2_MCBSP_WORDLEN_16,
				.word_length2 = 0,
				.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
				.frame_length2 = 0,
				.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
				.dxena = 0,
				.dxendly = 0,
				.callback = mcbsp_dma_cb,
			},
			.rx_params = {
				.data_type = OMAP2_MCBSP_WORDLEN_NONE,
				.skip_alt = OMAP2_MCBSP_SKIP_NONE,
				.auto_reset = OMAP2_MCBSP_AUTO_RRST,
				.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
				.data_delay = OMAP2_MCBSP_DATADELAY1,
				.reverse_compand = OMAP2_MCBSP_MSBFIRST,
				.word_length1 = OMAP2_MCBSP_WORDLEN_16,
				.word_length2 = 0,
				.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
				.frame_length2 = 0,
				.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
				.dxena = 0,
				.dxendly = 0,
				.callback = mcbsp_dma_cb,
				},
		},
	},
#endif
#if defined(CONFIG_MACH_BRISKET)
	{
		/* twl4030 pcm */
		.mcbsp_id = OMAP2_MCBSP_INTERFACE3,
		.mcbsp_config = {
			.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
#ifdef TWL_MASTER
			.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
			.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
			.fsx_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
			.fsr_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
			.clkr_src = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
			.clkx_src = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,
#else
			.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
			.srg_clk_sync = OMAP2_MCBSP_SRG_RUNNING,
			.fsx_src = OMAP2_MCBSP_TXFSYNC_INTERNAL,
			.fsr_src = OMAP2_MCBSP_RXFSYNC_INTERNAL,
			.clkr_src = OMAP2_MCBSP_CLKRXSRC_INTERNAL,	
			.clkx_src = OMAP2_MCBSP_CLKTXSRC_INTERNAL,
#endif
			.fsx_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
			.fsr_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
			.srg_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
			.clkx_pol = OMAP2_MCBSP_CLKX_POLARITY_RISING,
			.clkr_pol = OMAP2_MCBSP_CLKR_POLARITY_RISING,
			.tx_params = {
				.data_type = OMAP2_MCBSP_WORDLEN_NONE,
				.skip_alt = OMAP2_MCBSP_SKIP_NONE,
				.auto_reset = OMAP2_MCBSP_AUTO_XRST,
				.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
				.data_delay = OMAP2_MCBSP_DATADELAY1,
				.reverse_compand = OMAP2_MCBSP_MSBFIRST,
				.word_length1 = OMAP2_MCBSP_WORDLEN_16,
				.word_length2 = 0,
				.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
				.frame_length2 = 0,
				.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
				.dxena = 0,
				.dxendly = 0,
				.callback = mcbsp_dma_cb,
			},
			.rx_params = {
				.data_type = OMAP2_MCBSP_WORDLEN_NONE,
				.skip_alt = OMAP2_MCBSP_SKIP_NONE,
				.auto_reset = OMAP2_MCBSP_AUTO_RRST,
				.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
				.data_delay = OMAP2_MCBSP_DATADELAY1,
				.reverse_compand = OMAP2_MCBSP_MSBFIRST,
				.word_length1 = OMAP2_MCBSP_WORDLEN_16,
				.word_length2 = 0,
				.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
				.frame_length2 = 0,
				.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
				.dxena = 0,
				.dxendly = 0,
				.callback = mcbsp_dma_cb,
				},
		},
	},
#endif
	{ NULL },
};

struct sample_rate_info_t {
	u16 rate;
	u8 apll;
};
/* Hint - optimization wise move the most used values up the list */
static const struct sample_rate_info_t valid_sample_rates[] = {
	{.rate = 8000,.apll = 0},
};

#define NUMBER_OF_RATES_SUPPORTED (sizeof (valid_sample_rates)/\
				   sizeof (struct sample_rate_info_t))

/* Give time for these to settle down 
 * one frame (worst case is at 8000hz) = 125 uSec
 * wait two frame duration each - giving dma a chance to push data thru also
 * =125*4= 500uSec
 */
#define TWL4030_MCBSP2_3430SDP_PRESCALE_TIME 500

/* if we would like to listen to mono sound as stereo on stereo devices
 */
#undef MONO_MODE_SOUNDS_STEREO

/* How long to wait for stream to remain active?in Jiffies */
#define TIMEOUT_WAIT_FOR_ACTIVE       20

/* T2 GPIO for External Mute control - required for pop noise */
#define T2_AUD_EXT_MUT_GPIO 6
#define GPIO_DATA_DIR  0x03
#define GPIO_CLR  0x09
#define GPIO_SET  0x0C

#endif	/* End of __OMAP_AUDIO_TWL4030_H__ */
