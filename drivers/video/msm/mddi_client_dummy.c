/* drivers/video/msm_fb/mddi_client_dummy.c
 *
 * Support for "dummy" mddi client devices which require no
 * special initialization code.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <asm/arch/msm_fb.h>

static void dummy_function(struct mddi_panel_info *panel)
{
}

static struct mddi_panel_ops dummy_panel_ops = {
	.enable = dummy_function,
	.disable = dummy_function,
};

static int mddi_dummy_probe(struct platform_device *pdev)
{
	mddi_add_panel(pdev->dev.platform_data, &dummy_panel_ops);
	return 0;
}

static struct platform_driver mddi_client_4474_c065 = {
	.probe = mddi_dummy_probe,
	.driver = { .name = "mddi_c_4474_c065" },
};

static struct platform_driver mddi_client_0000_0000 = {
	.probe = mddi_dummy_probe,
	.driver = { .name = "mddi_c_0000_0000" },
};

static int __init mddi_client_dummy_init(void)
{
	platform_driver_register(&mddi_client_4474_c065);
	platform_driver_register(&mddi_client_0000_0000);
	return 0;
}

module_init(mddi_client_dummy_init);

