#ifndef __BOARD_AUDIO_H
#define __BOARD_AUDIO_H

#include <asm/arch/omap2_mcbsp.h>

#include "omap-mcbsp-alsa.h"
#include "twl4030-audio.h"

static struct snd_pcm_hardware voice_pcm_hw_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 2,
	.periods_max = 16,
	.fifo_size = 0,
};

static struct snd_pcm_hardware voice_pcm_hw_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000),
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
	.buffer_bytes_max = 64 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 2,
	.periods_max = 16,
	.fifo_size = 0,
};

inline struct snd_pcm_hardware *omap_mcbsp_voice_get_hw_playback(void)
{
	return &voice_pcm_hw_playback;
}

inline struct snd_pcm_hardware *omap_mcbsp_voice_get_hw_capture(void)
{
	return &voice_pcm_hw_capture;
}

struct mcbsp_audio_config twl4030_mcbsp_audio_config = {
	.mcbsp_id = OMAP2_MCBSP_INTERFACE2,
	.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
	.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
	.srg_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
	.clkx_src = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,
	.clkx_pol = OMAP2_MCBSP_CLKX_POLARITY_RISING,
	.clkr_src = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,
	.clkr_pol = OMAP2_MCBSP_CLKR_POLARITY_RISING,
	.fsx_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
	.fsx_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
	.fsr_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
	.fsr_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
	.tx_params = {
		.data_type = OMAP2_MCBSP_WORDLEN_NONE,
		.skip_alt = OMAP2_MCBSP_SKIP_NONE,
		.auto_reset = OMAP2_MCBSP_AUTO_XRST,
		.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP2_MCBSP_DATADELAY1,
		.reverse_compand = OMAP2_MCBSP_MSBFIRST,
		.word_length1 = OMAP2_MCBSP_WORDLEN_32,
		.word_length2 = 0,
		.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
		.frame_length2 = 0,
		.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
		.dxena = 0,
		.dxendly = 0,
	},
	.rx_params = {
		.data_type = OMAP2_MCBSP_WORDLEN_NONE,
		.skip_alt = OMAP2_MCBSP_SKIP_NONE,
		.auto_reset = OMAP2_MCBSP_AUTO_RRST,
		.phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP2_MCBSP_DATADELAY1,
		.reverse_compand = OMAP2_MCBSP_MSBFIRST,
		.word_length1 = OMAP2_MCBSP_WORDLEN_32,
		.word_length2 = 0,
		.frame_length1 = OMAP2_MCBSP_FRAMELEN_1,
		.frame_length2 = 0,
		.justification = OMAP2_MCBSP_RJUST_ZEROMSB,
		.dxena = 0,
		.dxendly = 0,
	},
};

struct omap_alsa_dev_private twl4030_alsa_dev_private = {
	.name = "twl4030-i2s (mcbsp2)",
	.mcbsp = &twl4030_mcbsp_audio_config,
	.get_codec_ops = twl4030_audio_get_codec_ops,
	.get_hw_playback = twl4030_audio_get_hw_playback,
	.get_hw_capture = twl4030_audio_get_hw_capture,
	.get_rate_constraint = twl4030_audio_get_rate_constraint,
};

struct mcbsp_audio_config btsco_mcbsp_audio_config = {
#if defined(CONFIG_MACH_SIRLOIN)
	.mcbsp_id = OMAP2_MCBSP_INTERFACE3,
#else
	.mcbsp_id = OMAP2_MCBSP_INTERFACE4,
#endif
	.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
	.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
	.clkx_src = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,
	.clkx_pol = OMAP2_MCBSP_CLKX_POLARITY_RISING,
	.clkr_src = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
	.clkr_pol = OMAP2_MCBSP_CLKR_POLARITY_RISING,
	.srg_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
	.fsx_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
	.fsx_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
	.fsr_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
	.fsr_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
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
	},
};

extern struct omap_alsa_codec_ops * sirloin_btsco_get_codec_ops(void);

struct omap_alsa_dev_private btsco_alsa_dev_private = {
#if defined(CONFIG_MACH_SIRLOIN)
	.name = "btsco (mcbsp3)",
	.get_codec_ops = sirloin_btsco_get_codec_ops,
#else
	.name = "btsco (mcbsp4)",
#endif
	.mcbsp = &btsco_mcbsp_audio_config,
	.get_hw_playback = omap_mcbsp_voice_get_hw_playback,
	.get_hw_capture = omap_mcbsp_voice_get_hw_capture,
};

struct mcbsp_audio_config modem_mcbsp_audio_config = {
	.mcbsp_id = OMAP2_MCBSP_INTERFACE5,
	.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
	.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
	.srg_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
	.clkx_src = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,
	.clkx_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
	.clkr_src = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
	.clkr_pol = OMAP2_MCBSP_CLKR_POLARITY_FALLING,
	.fsx_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
	.fsx_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
	.fsr_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
	.fsr_pol = OMAP2_MCBSP_FS_ACTIVE_LOW,
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
	},
};

struct omap_alsa_dev_private modem_alsa_dev_private = {
	.name = "modem (mcbsp5)",
	.mcbsp = &modem_mcbsp_audio_config,
	.get_hw_playback = omap_mcbsp_voice_get_hw_playback,
	.get_hw_capture = omap_mcbsp_voice_get_hw_capture,
};

#endif
