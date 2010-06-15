/* arch/arm/mach-msm/io.c
 *
 * MSM7K io support
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/bootmem.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/arch/msm_iomap.h>
#include <asm/mach/map.h>

#include <asm/arch/board.h>

#define MSM_DEVICE(name) { \
		.virtual = MSM_##name##_BASE, \
		.pfn = __phys_to_pfn(MSM_##name##_PHYS), \
		.length = MSM_##name##_SIZE, \
		.type = MT_DEVICE_NONSHARED, \
	 }

static struct map_desc halibut_io_desc[] __initdata = {
	MSM_DEVICE(VIC),
	MSM_DEVICE(CSR),
	MSM_DEVICE(GPT),
	MSM_DEVICE(DMOV),
	MSM_DEVICE(UART1),
	MSM_DEVICE(UART2),
	MSM_DEVICE(UART3),
	MSM_DEVICE(I2C),
	MSM_DEVICE(GPIO1),
	MSM_DEVICE(GPIO2),
	MSM_DEVICE(HSUSB),
	MSM_DEVICE(CLK_CTL),
	MSM_DEVICE(PMDH),
	MSM_DEVICE(EMDH),
	MSM_DEVICE(MDP),
	MSM_DEVICE(AD5),
	MSM_DEVICE(MDC),
	{
		.virtual =  MSM_SHARED_RAM_BASE,
		.pfn =      __phys_to_pfn(HALIBUT_SHARED_RAM_PHYS),
		.length =   MSM_SHARED_RAM_SIZE,
		.type =     MT_DEVICE,
	},
	MSM_DEVICE(SDC1),
	MSM_DEVICE(SDC2),
	MSM_DEVICE(SDC3),
	MSM_DEVICE(SDC4),
};

static struct map_desc blenny_io_desc[] __initdata = {
	MSM_DEVICE(VIC),
	MSM_DEVICE(CSR),
	MSM_DEVICE(GPT),
	MSM_DEVICE(DMOV),
	MSM_DEVICE(UART1),
	MSM_DEVICE(UART2),
	MSM_DEVICE(UART3),
	MSM_DEVICE(I2C),
	MSM_DEVICE(GPIO1),
	MSM_DEVICE(GPIO2),
	MSM_DEVICE(HSUSB),
	MSM_DEVICE(CLK_CTL),
	MSM_DEVICE(PMDH),
	MSM_DEVICE(EMDH),
	MSM_DEVICE(MDP),
	MSM_DEVICE(AD5),
	MSM_DEVICE(MDC),
	{
		.virtual =  MSM_SHARED_RAM_BASE,
		.pfn =      __phys_to_pfn(BLENNY_SHARED_RAM_PHYS),
		.length =   MSM_SHARED_RAM_SIZE,
		.type =     MT_DEVICE,
	},
	MSM_DEVICE(SDC1),
	MSM_DEVICE(SDC2),
	MSM_DEVICE(SDC3),
	MSM_DEVICE(SDC4),
	MSM_DEVICE(CS4),
	MSM_DEVICE(UART1DM),
	MSM_DEVICE(UART2DM),
	MSM_DEVICE(WEB_TCXO4),
#ifdef CONFIG_CACHE_L2X0
	{
		.virtual = (unsigned long) MSM_L2CC_BASE,
		.pfn = __phys_to_pfn(MSM_L2CC_PHYS),
		.length = MSM_L2CC_SIZE,
		.type = MT_DEVICE,
	},
#endif
};

void __init msm_map_common_io(void)
{
	/* Make sure the peripheral register window is closed, since
	 * we will use PTE flags (TEX[1]=1,B=0,C=1) to determine which
	 * pages are peripheral interface or not.
	 */
	asm("mcr p15, 0, %0, c15, c2, 4" : : "r" (0));

	/* reserve the first page for suspend/resume */
	reserve_bootmem(0, 4096);
}

void __init msm_map_halibut_io(void)
{
	iotable_init(halibut_io_desc, ARRAY_SIZE(halibut_io_desc));
}

void __init msm_map_blenny_io(void)
{
	iotable_init(blenny_io_desc, ARRAY_SIZE(blenny_io_desc));
}

void __iomem *
__msm_ioremap(unsigned long phys_addr, size_t size, unsigned int mtype)
{
	if (mtype == MT_DEVICE) {
		/* The peripherals in the 88000000 - D0000000 range
		 * are only accessable by type MT_DEVICE_NONSHARED.
		 * Adjust mtype as necessary to make this "just work."
		 */
		if ((phys_addr >= 0x88000000) && (phys_addr < 0xD0000000))
			mtype = MT_DEVICE_NONSHARED;
	}

	return __arm_ioremap(phys_addr, size, mtype);
}
EXPORT_SYMBOL(__msm_ioremap);
