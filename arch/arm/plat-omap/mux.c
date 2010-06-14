/*
 * linux/arch/arm/plat-omap/mux.c
 *
 * Utility to set the Omap MUX and PULL_DWN registers from a table in mux.h
 *
 * Copyright (C) 2003 - 2008 Nokia Corporation
 *
 * Written by Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include <asm/arch/mux.h>

#ifdef CONFIG_OMAP_MUX

static struct omap_mux_cfg *mux_cfg;

int __init omap_mux_register(struct omap_mux_cfg *arch_mux_cfg)
{
	if (!arch_mux_cfg || !arch_mux_cfg->pins || arch_mux_cfg->size == 0
			|| !arch_mux_cfg->cfg_reg) {
		printk(KERN_ERR "Invalid pin table\n");
		return -EINVAL;
	}

	mux_cfg = arch_mux_cfg;

	return 0;
}

static struct pin_config *omap_get_cfg(const char *mux_name)
{
	int i;

	if (!mux_cfg) {
		printk(KERN_ERR "Pin mux table not initialized\n");
		return NULL;
	}

	if (!mux_name)
		return NULL;

	/* Look for the cfg entry by pin name */
	for (i = 0; i < mux_cfg->size; i++) {
		if (0 == strcmp(mux_cfg->pins[i].name, mux_name))
			return &mux_cfg->pins[i];
	}

	return NULL;
}

unsigned int omap_get_mux_reg(const char *mux_name)
{
	struct pin_config *cfg;

	cfg = omap_get_cfg(mux_name);
	if (!cfg)
		return 0;

	return cfg->mux_reg;
}

/*
 * Sets the Omap MUX and PULL_DWN registers based on the table
 */
int omap_cfg_reg(const char *mux_name)
{
	struct pin_config *reg;

	if (!mux_cfg->cfg_reg)
		return -ENODEV;

	reg = omap_get_cfg(mux_name);
	if (!reg) {
		printk(KERN_ERR "*****************************************************\n");
		printk(KERN_ERR "Invalid pin mux name: %s\n",
				mux_name);
		dump_stack();
		printk(KERN_ERR "*****************************************************\n");
		return -ENODEV;
	}

	return mux_cfg->cfg_reg(reg);
}
EXPORT_SYMBOL(omap_cfg_reg);
#else
#define omap_mux_init() do {} while(0)
#define omap_cfg_reg(x)	do {} while(0)
#endif	/* CONFIG_OMAP_MUX */
