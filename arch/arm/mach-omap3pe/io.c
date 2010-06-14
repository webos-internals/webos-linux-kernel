/*
 * linux/arch/arm/mach-omap3pe/io.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP2/3 I/O mapping code
 *
 * Copyright (C) 2005 Nokia Corporation
 * Copyright (C) 2007 Texas Instruments
 *
 * Author:
 *	Juha Yrjola <juha.yrjola@nokia.com>
 *	Syed Khasim <x0khasim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/tlb.h>
#include <asm/io.h>

#include <asm/mach/map.h>

#include <asm/arch/mux.h>
#include <asm/arch/omapfb.h>

#include "resource.h"

extern void omap_sram_init(void);
#ifdef CONFIG_ARCH_OMAP34XX
extern int omap3_clk_init(void);
#endif

extern void omap2_check_revision(void);
extern void omap2_init_memory(void);
extern void gpmc_init(void);
extern void omapfb_reserve_sdram(void);

/*
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */
#ifdef CONFIG_ARCH_OMAP34XX
static struct map_desc omap34xx_io_desc[] __initdata = {
        {
                .virtual        = L4_VIRT,
                .pfn            = __phys_to_pfn(L4_PHYS),
                .length         = L4_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = L4_WK_VIRT,
                .pfn            = __phys_to_pfn(L4_WK_PHYS),
                .length         = L4_WK_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = L4_PER_VIRT,
                .pfn            = __phys_to_pfn(L4_PER_PHYS),
                .length         = L4_PER_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = L4_EMU_VIRT,
                .pfn            = __phys_to_pfn(L4_EMU_PHYS),
                .length         = L4_EMU_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = GFX_VIRT,
                .pfn            = __phys_to_pfn(GFX_PHYS),
                .length         = GFX_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = L3_VIRT,
                .pfn            = __phys_to_pfn(L3_PHYS),
                .length         = L3_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = SMS_VIRT,
                .pfn            = __phys_to_pfn(SMS_PHYS),
                .length         = SMS_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = SDRC_VIRT,
                .pfn            = __phys_to_pfn(SDRC_PHYS),
                .length         = SDRC_SIZE,
                .type           = IO_MAP_TYPE
        },
        {
                .virtual        = GPMC_VIRT,
                .pfn            = __phys_to_pfn(GPMC_PHYS),
                .length         = GPMC_SIZE,
                .type           = IO_MAP_TYPE
        },
};
#endif

void __init omap2_map_common_io(void)
{
	iotable_init(omap34xx_io_desc, ARRAY_SIZE(omap34xx_io_desc));
	/* Normally devicemaps_init() would flush caches and tlb after
	 * mdesc->map_io(), but we must also do it here because of the CPU
	 * revision check below.
	 */
	local_flush_tlb_all();
	flush_cache_all();
	omap2_check_revision();
	omap_sram_init();
	omapfb_reserve_sdram();
}

void __init omap2_init_common_hw(void)
{
	omap3_mux_init();
	omap3_resource_init();
	omap3_clk_init();
	omap2_init_memory();
	gpmc_init();
}
