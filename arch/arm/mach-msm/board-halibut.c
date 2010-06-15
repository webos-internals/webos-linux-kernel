/* linux/arch/arm/mach-msm/board-halibut.c
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
#include <asm/arch/msm_hsusb.h>
#include <asm/arch/vreg.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#define MSM_SMI_BASE		0x00000000
#define MSM_SMI_SIZE		0x900000

#define MSM_EBI_BASE		0x10000000
#define MSM_EBI_SIZE		0x6e00000

#define MSM_PMEM_GPU0_BASE	0x0
#define MSM_PMEM_GPU0_SIZE	0x800000

#define MSM_LINUX_BASE		MSM_EBI_BASE
#define MSM_LINUX_SIZE		0x4c00000

#define MSM_PMEM_MDP_BASE	MSM_LINUX_BASE + MSM_LINUX_SIZE
#define MSM_PMEM_MDP_SIZE	0x800000

#define MSM_PMEM_ADSP_BASE	MSM_PMEM_MDP_BASE + MSM_PMEM_MDP_SIZE
#define MSM_PMEM_ADSP_SIZE	0x800000

#define MSM_PMEM_GPU1_BASE	MSM_PMEM_ADSP_BASE + MSM_PMEM_ADSP_SIZE
#define MSM_PMEM_GPU1_SIZE	0x800000

#define MSM_FB_BASE		MSM_PMEM_GPU1_BASE + MSM_PMEM_GPU1_SIZE
#define MSM_FB_SIZE		0x200000

static int halibut_ffa;
module_param_named(ffa, halibut_ffa, int, S_IRUGO | S_IWUSR | S_IWGRP);

void halibut_init_keypad(int ffa);

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C004400,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(49),
		.end	= MSM_GPIO_TO_INT(49),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#ifdef CONFIG_FB_MSM

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define PWM_BLOCK_BASE         0x140000
#define DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define SYSCLKENA   (MDDI_CLIENT_CORE_BASE|0x2C)
#define START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define PWM0OFF     (PWM_BLOCK_BASE|0x1C)

static void mddi0_panel_power(struct mddi_panel_info *panel, int on)
{
	if(on) {
		mddi_remote_write(panel->mddi, 0, DPSUS);
		udelay(122);
		mddi_remote_write(panel->mddi, 1, SYSCLKENA);
		mddi_remote_write(panel->mddi, halibut_ffa ? 0 : 0x00001387, PWM0OFF);
	}
	else {
		mddi_remote_write(panel->mddi, halibut_ffa ? 0x00001387 : 0, PWM0OFF);
		udelay(122);
		mddi_remote_write(panel->mddi, 0, SYSCLKENA);
		mddi_remote_write(panel->mddi, 1, DPSUS);
	}
}

static struct msm_mddi_platform_data msm_mddi0_pdata = {
	.panel_power	= mddi0_panel_power,
	.has_vsync_irq	= 0,
	.fb_base = MSM_FB_BASE,
	.fb_size = MSM_FB_SIZE,
};

static struct platform_device msm_mddi0_device = {
	.name	= "msm_mddi",
	.id	= 0,
	.dev	= {
		.platform_data = &msm_mddi0_pdata
	},
};

#endif
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

static struct resource usb_resources[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

/* The HSUSB PHY on Halibut has a hardware bug where VBUS
 * interrupts can lock up the ULPI bus, causing USB to fail.
 * Disable these interrupts to avoid this issue.
 */
static int halibut_phy_init_seq[] = { 0x1D, 0x0D, 0x1D, 0x10, -1 };

static char *halibut_usb_functions[] = {
	"diag",
	"adb",
};

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq	= halibut_phy_init_seq,
	.vendor_id	= 0x18d1,
	.product_id	= 0xd00d,
	.version	= 0x0100,
	.product_name	= "Halibut",
	.functions	= halibut_usb_functions,
	.num_functions	= ARRAY_SIZE(halibut_usb_functions),
};

static struct platform_device msm_hsusb_device = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(usb_resources),
	.resource	= usb_resources,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &msm_hsusb_pdata,
	},
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.start = MSM_PMEM_MDP_BASE,
	.size = MSM_PMEM_MDP_SIZE,
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.start = MSM_PMEM_ADSP_BASE,
	.size = MSM_PMEM_ADSP_SIZE,
	.no_allocator = 0,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_gpu0_pdata = {
        .name = "pmem_gpu0",
        .start = MSM_PMEM_GPU0_BASE,
        .size = MSM_PMEM_GPU0_SIZE,
        .no_allocator = 1,
        .cached = 0,
};

static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
        .name = "pmem_gpu1",
        .start = MSM_PMEM_GPU1_BASE,
        .size = MSM_PMEM_GPU1_SIZE,
        .no_allocator = 1,
        .cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_gpu0_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_gpu0_pdata },
};

static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};

static struct platform_device *devices[] __initdata = {
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_serial0_device,
#endif
#ifdef CONFIG_FB_MSM
	&msm_mddi0_device,
#endif
	&msm_hsusb_device,
	&smc91x_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_gpu0_device,
	&android_pmem_gpu1_device,
};

/*
 * All halibut slots are the same(ish)
 */

static unsigned int halibut_sdcc_slot_status(struct device *dev)
{
	/*
	 * TODO: Hook this up to the FPGA for reading slot status
	 * For now return 0. 
	 */
	return 0;
}

static struct mmc_platform_data halibut_sdcc_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.status		= halibut_sdcc_slot_status,
};

extern struct sys_timer msm_timer;

static void __init halibut_init_irq(void)
{
	msm_init_irq();
}

static struct msm_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
};

void msm_serial_debug_init(unsigned int base, int irq, 
			   const char *clkname, int signal_irq);

static void __init halibut_init_mmc(void)
{
	struct vreg *vreg_mmc;
	int rc;

	vreg_mmc = vreg_get(0, "mmc");
	rc = vreg_enable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg enable failed (%d)\n", __func__, rc);

	msm_add_sdcc(1, &halibut_sdcc_data);
/* TODO: Enable these once we have support for the SDC mux
 *	msm_add_sdcc(2, &halibut_sdcc_data);
 *	msm_add_sdcc(3, &halibut_sdcc_data);
 *	msm_add_sdcc(4, &halibut_sdcc_data);
 */

}

static void __init halibut_init(void)
{
#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART1_PHYS, INT_UART1,
			      "uart1_clk", 1);
#endif
	msm_init_gpio();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	halibut_init_keypad(halibut_ffa);
	msm_add_devices();
	halibut_init_mmc();

	/* TODO: detect vbus and correctly notify USB about its presence 
	 * For now we just declare that VBUS is present at boot and USB
	 * copes, but this is not ideal.
	 */
	msm_hsusb_set_vbus_state(1);
}

static void __init halibut_map_io(void)
{
	msm_map_common_io();
	msm_map_halibut_io();
	halibut_clock_init();
	msm_clock_init(&halibut_clock_data);
}

MACHINE_START(HALIBUT, "Halibut Board (QCT SURF7200A)")

/* UART for LL DEBUG */
	.phys_io	= MSM_UART1_PHYS,
	.io_pg_offst	= ((MSM_UART1_BASE) >> 18) & 0xfffc,

	.boot_params	= 0x10000100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END
