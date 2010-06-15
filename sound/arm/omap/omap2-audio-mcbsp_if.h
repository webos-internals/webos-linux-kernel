#ifndef __OMAP2_AUDIO_MCBSP_IF_H
#define __OMAP2_AUDIO_MCBSP_IF_H

#include <asm/arch/omap2_mcbsp.h>

struct mcbsp_config {
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

struct mcbsp_codec_data {
	unsigned int mcbsp_id;
	int acquired;
	int mixer_dev_id;
	struct mcbsp_config mcbsp_config;
};

#endif
