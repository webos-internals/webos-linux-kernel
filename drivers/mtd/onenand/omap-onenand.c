/*
 *  linux/drivers/mtd/onenand/omap-onenand.c
 *
 *  Copyright (c) 2005 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 *  Derived from linux/drivers/mtd/nand/omap-nand-flash.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  Overview:
 *  This is a device driver for the OneNAND flash device for TI OMAP boards.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/onenand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/sizes.h>

#include <asm/arch/gpmc.h>
#include <asm/arch/onenand.h>

#define DEVICE_NAME "omap2-onenand"

#define OMAP_ONENAND_FLASH_START1	ONENAND_MAP
#define OMAP_ONENAND_FLASH_START2	ONENAND_MAP
#define ONENAND_IO_SIZE			SZ_128K

struct omap_onenand {
	struct platform_device *pdev;
	int gpmc_cs;
	unsigned long phys_base;
	struct mtd_info mtd;
	struct mtd_partition *parts;
	struct onenand_chip onenand;
};

/*
 * Define partitions for flash devices
 */

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL,  };
#endif

static int __devinit omap_onenand_probe(struct platform_device *pdev)
{
	struct omap_onenand_platform_data *pdata;
	struct omap_onenand *info;
	int r;
	
	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct omap_onenand), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	
	info->phys_base = OMAP_ONENAND_FLASH_START1;
	
	if (request_mem_region(info->phys_base, ONENAND_IO_SIZE,
			       pdev->dev.driver->name) == NULL) {
		dev_err(&pdev->dev, "Cannot reserve memory region at 0x%08lx, size: 0x%x\n",
			info->phys_base, ONENAND_IO_SIZE);
		r = -EBUSY;
		goto err_kfree;
	}
	info->onenand.base = ioremap(info->phys_base, ONENAND_IO_SIZE);
	if (info->onenand.base == NULL) {
		r = -ENOMEM;
		goto err_release_mem_region;
	}

	if (pdata->onenand_setup != NULL) {
		r = pdata->onenand_setup(info->onenand.base);
		if (r < 0) {
			dev_err(&pdev->dev, "Onenand platform setup failed: %d\n", r);
			goto err_iounmap;                       
		}
        }

	dev_info(&pdev->dev, "initializing on CS%d, phys base 0x%08lx, virtual base %p\n",
		 pdata->cs, info->phys_base, info->onenand.base);

	info->gpmc_cs = pdata->cs;
	info->pdev = pdev;
	info->mtd.name = pdev->dev.bus_id;
	info->mtd.priv = &info->onenand;
	info->mtd.owner = THIS_MODULE;

	if ((r = onenand_scan(&info->mtd, 1)) < 0)
		goto err_iounmap;

#ifdef CONFIG_MTD_PARTITIONS
	r = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (r > 0)
		r = add_mtd_partitions(&info->mtd, info->parts, r);
	else if (r < 0 && pdata->parts)
		r = add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		r = add_mtd_device(&info->mtd);
	if ( r < 0)
		goto err_release_onenand;

	platform_set_drvdata(pdev, info);

	return 0;

err_release_onenand:
	onenand_release(&info->mtd);
err_iounmap:
	iounmap(info->onenand.base);
err_release_mem_region:
	release_mem_region(info->phys_base, ONENAND_IO_SIZE);
err_kfree:
	kfree(info);

	return r;
}

static int __devexit omap_onenand_remove(struct platform_device *pdev)
{
	struct omap_onenand *info = dev_get_drvdata(&pdev->dev);

	BUG_ON(info == NULL);

#ifdef CONFIG_MTD_PARTITIONS
	if (info->parts)
		del_mtd_partitions(&info->mtd);
	else
		del_mtd_device(&info->mtd);
#else
	del_mtd_device(&info->mtd);
#endif

	onenand_release(&info->mtd);
	platform_set_drvdata(pdev, NULL);
	iounmap(info->onenand.base);
	release_mem_region(info->phys_base, ONENAND_IO_SIZE);
	kfree(info);

	return 0;
}

static struct platform_driver omap_onenand_driver = {
	.probe		= omap_onenand_probe,
	.remove		= omap_onenand_remove,
	.driver		= {
		.name	= DEVICE_NAME,
		.owner  = THIS_MODULE,
	},
};

MODULE_ALIAS(DRIVER_NAME);

static int __init omap_onenand_init(void)
{
	return platform_driver_register(&omap_onenand_driver);
}

static void __exit omap_onenand_exit(void)
{
	platform_driver_unregister(&omap_onenand_driver);
}

module_init(omap_onenand_init);
module_exit(omap_onenand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kyungmin Park <kyungmin.park@samsung.com>");
MODULE_DESCRIPTION("Glue layer for OneNAND flash on OMAP boards");
