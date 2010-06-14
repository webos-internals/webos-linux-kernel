#ifndef __MCBSP_ALSA_H
#define __MCBSP_ALSA_H

#include <sound/driver.h>
#include <sound/core.h>
#include <asm/arch/omap2_mcbsp.h>

#define OMAP_MCBSP_AUDIO_NAME    "omap-mcbsp-audio"

struct mcbsp_audio_config {
	unsigned int mcbsp_id;
	bool acquired;

	u8 mcbsp_clk_src;    /* source it from prcm? */
	u8 srg_clk_src;      /* clks/fclk/clkr/clkx */
	u8 srg_clk_sync;     /* free_running or just running */
	u8 srg_clk_pol;      /* srg clk polarity */
	u8 clkx_src;         /* clkx src (internal/external) */
	u8 clkx_pol;         /* clkx polarity */
	u8 clkr_src;         /* clkr src (internal/external) */
	u8 clkr_pol;         /* clkr polarity */
	u8 fsx_src;          /* fsx src (internal/external) */
	u8 fsx_pol;          /* fsx polarity */
	u8 fsr_src;          /* fsr src (internal/external) */
	u8 fsr_pol;		     /* fsr polarity */
	omap2_mcbsp_transfer_params tx_params; /* Transmit parameters */
	omap2_mcbsp_transfer_params rx_params; /* Recieve parameters */
};

struct omap_alsa_codec_ops {
	int (*probe)(void);
	int (*mixer_init)(struct snd_card *);
	int (*configure)(unsigned int sample_rate, unsigned int data_width,
			unsigned int channels);
	int (*enable)(bool);
	int (*capture_enable)(void);
	int (*suspend)(void);
	int (*resume)(void);
	int (*mute)(bool);
};

struct omap_alsa_dev_private {
	char *name;
	struct mcbsp_audio_config *mcbsp;

	struct omap_alsa_codec_ops * (*get_codec_ops)(void);
	struct snd_pcm_hardware * (*get_hw_playback)(void);
	struct snd_pcm_hardware * (*get_hw_capture)(void);
	struct snd_pcm_hw_constraint_list * (*get_rate_constraint)(void);
};

#endif
