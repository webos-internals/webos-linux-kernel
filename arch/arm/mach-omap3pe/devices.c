/*
 * linux/arch/arm/mach-omap3pe/devices.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on linux/arch/arm/mach-omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include <asm/arch/tc.h>
#include <asm/arch/board.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/eac.h>

#if defined(CONFIG_OMAP_DSP) || defined(CONFIG_OMAP_DSP_MODULE)
#define OMAP2_MBOX_BASE		IO_ADDRESS(OMAP24XX_MAILBOX_BASE)

static struct resource mbox_resources[] = {
	{
		.start		= OMAP2_MBOX_BASE,
		.end		= OMAP2_MBOX_BASE + 0x11f,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_MAIL_U0_MPU,
		.flags		= IORESOURCE_IRQ,
	},
	{
		.start		= INT_24XX_MAIL_U3_MPU,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device mbox_device = {
	.name		= "mailbox",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(mbox_resources),
	.resource	= mbox_resources,
};

static inline void omap_init_mbox(void)
{
	platform_device_register(&mbox_device);
}
#else
static inline void omap_init_mbox(void) { }
#endif

#if defined(CONFIG_OMAP_STI)

#define OMAP2_STI_BASE		IO_ADDRESS(0x48068000)
#define OMAP2_STI_CHANNEL_BASE	0x54000000
#define OMAP2_STI_IRQ		4

static struct resource sti_resources[] = {
	{
		.start		= OMAP2_STI_BASE,
		.end		= OMAP2_STI_BASE + 0x7ff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_STI_CHANNEL_BASE,
		.end		= OMAP2_STI_CHANNEL_BASE + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= OMAP2_STI_IRQ,
		.flags		= IORESOURCE_IRQ,
	}
};

static struct platform_device sti_device = {
	.name		= "sti",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sti_resources),
	.resource	= sti_resources,
};

static inline void omap_init_sti(void)
{
	platform_device_register(&sti_device);
}
#else
static inline void omap_init_sti(void) {}
#endif

#if defined(CONFIG_SPI_OMAP24XX_OMAP34XX)

#include <asm/arch/mcspi.h>

#define OMAP2_MCSPI1_BASE		0x48098000
#define OMAP2_MCSPI2_BASE		0x4809a000
#define OMAP2_MCSPI3_BASE		0x480b8000
#define OMAP2_MCSPI4_BASE		0x480ba000
#define OMAP2_MCSPI_MASTER		0
#define OMAP2_MCSPI_SLAVE		1

static struct omap2_mcspi_platform_config omap2_mcspi1_config = {
	.num_cs		= 4,
	.mode		= OMAP2_MCSPI_MASTER, /* master=0 slave=1*/
};

static struct resource omap2_mcspi1_resources[] = {
	{
		.start		= OMAP2_MCSPI1_BASE,
		.end		= OMAP2_MCSPI1_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},

	{
		.start		= INT_24XX_SPI1_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device omap2_mcspi1 = {
	.name		= "omap2_mcspi",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(omap2_mcspi1_resources),
	.resource	= omap2_mcspi1_resources,
	.dev		= {
		.platform_data = &omap2_mcspi1_config,
	},
};

static struct omap2_mcspi_platform_config omap2_mcspi2_config = {
	.num_cs		= 2,
	.mode		= OMAP2_MCSPI_SLAVE,
};

static struct resource omap2_mcspi2_resources[] = {
	{
		.start		= OMAP2_MCSPI2_BASE,
		.end		= OMAP2_MCSPI2_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},

	{
		.start		= INT_24XX_SPI2_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device omap2_mcspi2 = {
	.name		= "omap2_mcspi",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(omap2_mcspi2_resources),
	.resource	= omap2_mcspi2_resources,
	.dev		= {
		.platform_data = &omap2_mcspi2_config,
	},
};

static struct omap2_mcspi_platform_config omap2_mcspi3_config = {
	.num_cs		= 2,
	.mode		= OMAP2_MCSPI_MASTER,
};
static struct resource omap2_mcspi3_resources[] = {
	{
		.start		= OMAP2_MCSPI3_BASE,
		.end		= OMAP2_MCSPI3_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},

	{
		.start		= INT_24XX_SPI3_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
struct platform_device omap2_mcspi3 = {
	.name		= "omap2_mcspi",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(omap2_mcspi3_resources),
	.resource	= omap2_mcspi3_resources,
	.dev		= {
		.platform_data = &omap2_mcspi3_config,
	},
};

#ifdef CONFIG_ARCH_OMAP3
static struct omap2_mcspi_platform_config omap2_mcspi4_config = {
	.num_cs		= 1,
	.mode		= OMAP2_MCSPI_MASTER,
};
static struct resource omap2_mcspi4_resources[] = {
	{
		.start		= OMAP2_MCSPI4_BASE,
		.end		= OMAP2_MCSPI4_BASE + 0xff,
		.flags		= IORESOURCE_MEM,
	},

	{
		.start		= INT_34XX_SPI4_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
struct platform_device omap2_mcspi4 = {
	.name		= "omap2_mcspi",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(omap2_mcspi4_resources),
	.resource	= omap2_mcspi4_resources,
	.dev		= {
		.platform_data = &omap2_mcspi4_config,
	},
};
#endif
static void omap_init_mcspi(void)
{
	platform_device_register(&omap2_mcspi1);
	platform_device_register(&omap2_mcspi2);
	platform_device_register(&omap2_mcspi3);
#ifdef CONFIG_ARCH_OMAP3
	platform_device_register(&omap2_mcspi4);
#endif
}

#else
static inline void omap_init_mcspi(void) {}
#endif

#ifdef CONFIG_SND_OMAP24XX_EAC

#define OMAP2_EAC_BASE			0x48090000

static struct resource omap2_eac_resources[] = {
	{
		.start		= OMAP2_EAC_BASE,
		.end		= OMAP2_EAC_BASE + 0x109,
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap2_eac_device = {
	.name		= "omap24xx-eac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap2_eac_resources),
	.resource	= omap2_eac_resources,
	.dev = {
		.platform_data = NULL,
	},
};

void omap_init_eac(struct eac_platform_data *pdata)
{
	omap2_eac_device.dev.platform_data = pdata;
	platform_device_register(&omap2_eac_device);
}

#else
void omap_init_eac(struct eac_platform_data *pdata) {}
#endif

#if    defined(CONFIG_MMC_OMAP3430)  \
    || defined(CONFIG_OMAP3430_MMC1) \
    || defined(CONFIG_OMAP3430_MMC2)
#if defined(CONFIG_OMAP3430_MMC1)
#define	OMAP_HSMMC_1_BASE	0x4809c000
static struct omap_mmc_conf mmc1_conf;
static struct resource mmc1_resources[] = {
	{
		.start		= OMAP_HSMMC_1_BASE,
		.end		= OMAP_HSMMC_1_BASE + 0x01FC,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_MMC_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device mmc_omap_device1 = {
	.name		= "hsmmc-omap",
	.id		= 1,
	.dev = {
		.platform_data	= &mmc1_conf,
	},
	.num_resources	= ARRAY_SIZE(mmc1_resources),
	.resource	= mmc1_resources,
};
#endif
#if defined(CONFIG_OMAP3430_MMC2)
#define	OMAP_HSMMC_2_BASE	0x480B4000
static struct omap_mmc_conf mmc2_conf;
static struct resource mmc2_resources[] = {
	{
		.start		= OMAP_HSMMC_2_BASE,
		.end		= OMAP_HSMMC_2_BASE + 0x01FC,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= INT_24XX_MMC2_IRQ,
		.flags		= IORESOURCE_IRQ,
	},
};
static struct platform_device mmc_omap_device2 = {
	.name		= "hsmmc-omap",
	.id		= 2,
	.dev = {
		.platform_data	= &mmc2_conf,
	},
	.num_resources	= ARRAY_SIZE(mmc2_resources),
	.resource	= mmc2_resources,
};
#endif
static void __init omap2_3_init_mmc(void)
{
	const struct omap_mmc_config	*mmc_conf;
	const struct omap_mmc_conf	*mmc;

	mmc_conf = omap_get_config(OMAP_TAG_MMC, struct omap_mmc_config);
	if (!mmc_conf)
		return;

#if defined(CONFIG_OMAP3430_MMC1)
	mmc = &mmc_conf->mmc[0];
	if (mmc->enabled) {
		mmc1_conf = *mmc;
		(void) platform_device_register(&mmc_omap_device1);
	}
#endif

#if defined(CONFIG_OMAP3430_MMC2)
	mmc = &mmc_conf->mmc[1];
	if (mmc->enabled) {
		mmc2_conf = *mmc;
		(void) platform_device_register(&mmc_omap_device2);
	}
#endif
}	
#else
static inline void omap2_3_init_mmc(void) {}
#endif

#ifdef CONFIG_OMAP_BCI_BATTERY
static struct platform_device omap_bci_battery_device = {
	.name           = "omap-bci-battery",
	.id             = 1,
	.num_resources  = 0,
	.resource       = NULL,
};

static inline void omap_bci_battery_init(void)
{
	(void) platform_device_register(&omap_bci_battery_device);
}
#else
static inline void omap_bci_battery_init(void) {}
#endif

#if defined(CONFIG_OMAP_BQ27000_BATTERY) && defined(CONFIG_OMAP_HDQ)
static struct platform_device omap_bq2700_battery_device = {
	.name           = "omap-bq2700-battery",
	.id             = 1,
	.num_resources  = 0,
	.resource       = NULL,
};

static inline void omap_bq2700_battery_init(void)
{
	(void) platform_device_register(&omap_bq2700_battery_device);
}
#else
static inline void omap_bq2700_battery_init(void) {}
#endif

#ifdef CONFIG_OMAP_HDQ
static struct platform_device omap_hdq_dev = {
	.name = "omap_hdq",
	.id = 0,
	.dev = {
		.platform_data = NULL,
	},
};
static inline void omap_hdq_init(void)
{
	(void) platform_device_register(&omap_hdq_dev);
}
#else
static inline void omap_hdq_init(void) {}
#endif

/*-------------------------------------------------------------------------*/

static int __init omap2_init_devices(void)
{
	/* please keep these calls, and their implementations above,
	 * in alphabetical order so they're easier to sort through.
	 */
	omap_init_mbox();
	omap_init_mcspi();
	omap_hdq_init();
	omap_bci_battery_init();
	omap_bq2700_battery_init();
	omap_init_sti();
	omap2_3_init_mmc();

	return 0;
}
arch_initcall(omap2_init_devices);
