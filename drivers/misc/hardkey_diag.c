/*
 *
 * Copyright (C) 2008-2009 Palm, Inc.
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
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/reboot.h>
#include <linux/sched.h>

#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

extern struct raw_notifier_head diag_key_notifier_list;

static int    diag_trig_max_cnt =   6; 
static int    diag_init_timeout =   0; //  immediately

static int    diag_trig_counter; 

static int hardkey_enabled = 1;
module_param_named(
    enabled, hardkey_enabled, int, S_IRUGO | S_IWUSR | S_IWGRP
	);

static int __init 
hardkey_diag_setup(char *str)
{
	diag_init_timeout = simple_strtoul(str, NULL, 0);
	return 1;
}
__setup("hardkey_diag=", hardkey_diag_setup);

static void 
hardkey_diag_timer_cb (unsigned long data)
{
	(void) data;
	
	printk( KERN_INFO "Initiate hardkey triggered Diag\n");
	show_state_filter(0);
}

static int 
hardkey_diag_cb(struct notifier_block *nb, unsigned long val, void *ctxt)
{
	PDBG("%s: cnt=%d, val=%ld\n", __func__, diag_trig_counter, val );

	if( val ) {
		if( diag_trig_counter )
			diag_trig_counter--;
			
		if( diag_trig_counter )
			return 0;   

		if (hardkey_enabled) {
			//  initiate diag;
			hardkey_diag_timer_cb(0);

			// reset counters
			diag_trig_counter = diag_trig_max_cnt;
		}
	} else {
		// reset counters
		diag_trig_counter = diag_trig_max_cnt;
	}

	return 0;
}

static struct notifier_block hardkey_diag_nb = {
	.notifier_call = hardkey_diag_cb,
};

static int __init 
hardkey_diag_init(void)
{
	PDBG("%s: \n", __func__ );

	diag_trig_counter = diag_trig_max_cnt;

	raw_notifier_chain_register( &diag_key_notifier_list, 
	                             &hardkey_diag_nb);
	
	return 0; 
}

static void __exit   
hardkey_diag_exit(void)
{
	PDBG("%s: \n", __func__ );

	raw_notifier_chain_unregister( &diag_key_notifier_list, 
	                               &hardkey_diag_nb);
}


module_init(hardkey_diag_init);
module_exit(hardkey_diag_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");





