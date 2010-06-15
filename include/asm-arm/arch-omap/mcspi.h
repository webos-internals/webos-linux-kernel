#ifndef _OMAP2_MCSPI_H
#define _OMAP2_MCSPI_H

struct omap2_mcspi_platform_config {
	unsigned short	num_cs;
	unsigned short	mode;
};

struct omap2_mcspi_device_config {
	unsigned turbo_mode:1;

	/* Do we want one channel enabled at the same time? */
	unsigned single_channel:1;
};

#ifdef CONFIG_OMAP34XX_OFFMODE
#include <asm/arch/clock.h>
extern int context_restore_required(struct clk *clk);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#endif
