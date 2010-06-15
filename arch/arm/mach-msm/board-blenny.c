/* linux/arch/arm/mach-msm/board-blenny.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (C) 2008 Qualcomm, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>

#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/msm_fb.h>
#include <asm/arch/vreg.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C004400,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(132),
		.end	= MSM_GPIO_TO_INT(132),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

static struct resource msm_serial0_resources[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device msm_serial0_device = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(msm_serial0_resources),
	.resource	= msm_serial0_resources,
};

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_serial0_device,
#endif
	&smc91x_device,
};

extern struct sys_timer msm_timer;

static void __init blenny_init_irq(void)
{
	msm_init_irq();
}

static struct msm_clock_platform_data blenny_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
};

static unsigned int blenny_sdcc_slot_status(struct device *dev)
{
	/*
	 * TODO: Hook this up to the FPGA for reading slot status
	 * For now return 0. 
	 */
	/* PALM: set to 1 for now to assume a card is in the slot */
	return 1;
}

static struct mmc_platform_data blenny_sdcc_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.status		= blenny_sdcc_slot_status,
};

void msm_serial_debug_init(unsigned int base, int irq, 
			   const char *clkname, int signal_irq);

static void __init blenny_init_mmc(void)
{
	struct vreg *vreg_mmc;
	int rc;

	vreg_mmc = vreg_get(0, "mmc");
	rc = vreg_enable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg enable failed (%d)\n", __func__, rc);

	msm_add_sdcc(1, &blenny_sdcc_data);
/* TODO: Enable these once we have support for the SDC mux
 *	msm_add_sdcc(2, &blenny_sdcc_data);
 *	msm_add_sdcc(3, &blenny_sdcc_data);
 *	msm_add_sdcc(4, &blenny_sdcc_data);
 */
}


static void __init blenny_init(void)
{
#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      "uart1_clk", 1);
#endif
	msm_init_gpio();
	platform_add_devices(devices, ARRAY_SIZE(devices));
//	halibut_init_keypad(halibut_ffa);
	msm_add_devices();
	blenny_init_mmc();

	/* TODO: detect vbus and correctly notify USB about its presence 
	 * For now we just declare that VBUS is present at boot and USB
	 * copes, but this is not ideal.
	 */
	msm_hsusb_set_vbus_state(1);
}

static void __init blenny_map_io(void)
{
	msm_map_common_io();
	msm_map_blenny_io();
	msm_clock_init(&blenny_clock_data);
}

MACHINE_START(BLENNY, "Blenny Board (QCT SURF7625A)")

/* UART for LL DEBUG */
	.phys_io	= MSM_UART1_PHYS,
	.io_pg_offst	= ((MSM_UART1_BASE) >> 18) & 0xfffc,

	.boot_params	= 0x00200100,
	.map_io		= blenny_map_io,
	.init_irq	= blenny_init_irq,
	.init_machine	= blenny_init,
	.timer		= &msm_timer,
MACHINE_END
