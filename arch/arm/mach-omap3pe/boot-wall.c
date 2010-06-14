/*
 * This file contains code used to throw args back at bootie when restarting
 *
 * Copyright (C) 2008 Palm, Inc.
 * Evan Geller
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/io.h>
#include <asm/string.h>
#include <asm/arch/prcm.h>

#define BOOT_WALL_MAGIC 0x0fee1fed

/*
 *   Reserve first 256 bytes of scratch pad to store boot argument
 *   The area starts with magic number in little endian format followed 
 *   by zero terminated boot argument string.
 */
void boot_wall_put_args(char *args)
{
	unsigned long flags;
	u8 *scratchpad_address;
	size_t len;
	u8 res;

	local_irq_save(flags);

	prcm_clock_control(PRCM_OMAP_CTRL, ICLK, PRCM_ENABLE, PRCM_TRUE);
	prcm_is_device_accessible(PRCM_OMAP_CTRL, &res);
	if (res == PRCM_FALSE) {
		printk("%s: cannot be accessed\n", __func__);
		goto done;
	}

	scratchpad_address = (u8 *)&SCRATCHPAD_BASE;

	/* This gets called on restart with args == NULL. Initialize
	 * the scratchpad with something, if there isn't already anything. 
	 * Otherwise, leave the existing contents alone. We can't initialize 
	 * on boot because the scratchpad is also used by PM code. */
	if (args == NULL) {
		if ((*(uint32_t *)scratchpad_address) != BOOT_WALL_MAGIC) {
			args = "running";
		} else {
			goto done;
		}
	}

	*(scratchpad_address++) = (u8)((BOOT_WALL_MAGIC)       & 0xFF);
	*(scratchpad_address++) = (u8)((BOOT_WALL_MAGIC >>  8) & 0xFF);
	*(scratchpad_address++) = (u8)((BOOT_WALL_MAGIC >> 16) & 0xFF);
	*(scratchpad_address++) = (u8)((BOOT_WALL_MAGIC >> 24) & 0xFF);

	len = strlen(args);
	if (len > 251)
		len = 251; 

	memcpy(scratchpad_address, args, len);
        scratchpad_address[len] = 0;

done:
	local_irq_restore(flags);
}

volatile static int panic_ref = 0;

/* 
 *   Panic handler
 */
static int
boot_wall_panic(struct notifier_block *this, unsigned long event, void *ptr)
{
	if ( panic_ref )
		return NOTIFY_DONE;
	panic_ref++;

	boot_wall_put_args("panic");
	return NOTIFY_DONE;
}

static struct notifier_block panic_block = {
	.notifier_call = boot_wall_panic,
};

static int __init  
boot_wall_init(void)
{ 
	// register panic callback
	atomic_notifier_chain_register(&panic_notifier_list, &panic_block);

	return 0;
} 

 
static void __exit 
boot_wall_exit(void) 
{ 
	// unregister panic callback
	atomic_notifier_chain_unregister(&panic_notifier_list, &panic_block);
	return;
} 

module_init(boot_wall_init); 
module_exit(boot_wall_exit); 
 
MODULE_DESCRIPTION("Boot wall handler");
MODULE_LICENSE("GPL");  
