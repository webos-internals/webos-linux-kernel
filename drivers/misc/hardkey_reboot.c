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

extern struct raw_notifier_head reboot_key_notifier_list;

#if 0
static int    reboot_trig_max_cnt =   1; 
static int    reboot_sync_timeout =   3; //  3 sec
static int    reboot_init_timeout =  10; // 10 sec
#else
static int    reboot_trig_max_cnt =   3; 
static int    reboot_sync_timeout =   0; //  immediately
static int    reboot_init_timeout =   7; //  after 7 seconds
#endif

static int    reboot_trig_counter; 
static struct timer_list reboot_init_timer; // reboot initialization timer
static struct timer_list reboot_sync_timer; // reboot sync timer


static int __init 
hardkey_reboot_setup(char *str)
{
	reboot_init_timeout = simple_strtoul(str, NULL, 0);
	reboot_sync_timeout = reboot_init_timeout - 7;
	if( reboot_sync_timeout < 0 ) 
	    reboot_sync_timeout = 0;
	return 1;
}
__setup("hardkey_reboot=", hardkey_reboot_setup);

static void 
hardkey_reboot_timer_cb (unsigned long data)
{
	(void) data;
	
	show_state_filter(0);

	printk( KERN_INFO "Initiate hardkey triggered Emergency reboot\n");
	emergency_restart();
}

static void 
hardkey_reboot_sync_timer_cb (unsigned long data)
{
	printk( KERN_INFO "Initiate hardkey triggered Emergency sync\n");
	emergency_sync();
}

static int 
hardkey_reboot_cb(struct notifier_block *nb, unsigned long val, void *ctxt)
{
	PDBG("%s: cnt=%d, val=%ld\n", __func__, reboot_trig_counter, val );

	if( val ) {
		if( reboot_trig_counter )
			reboot_trig_counter--;
			
		if( reboot_trig_counter )
			return 0;   
			
		//  initiate reboot;
		mod_timer(&reboot_init_timer, jiffies + 
		           msecs_to_jiffies(reboot_init_timeout*1000));
		
		if( reboot_sync_timeout ) {
			// start sync timer
			mod_timer(&reboot_sync_timer, jiffies + 
			          msecs_to_jiffies(reboot_sync_timeout*1000));
		} else {
			// do it immediately
			hardkey_reboot_sync_timer_cb (0);
		}
	} else {
		// reset counters stop timers if reset is not started yet
		if( reboot_trig_counter ) { 
			reboot_trig_counter = reboot_trig_max_cnt;
			del_timer(&reboot_init_timer);
			del_timer(&reboot_sync_timer);
		}
	}

	return 0;
}

static struct notifier_block hardkey_reboot_nb = {
	.notifier_call = hardkey_reboot_cb,
};

static int __init 
hardkey_reboot_init(void)
{
	PDBG("%s: \n", __func__ );

	setup_timer( &reboot_init_timer, hardkey_reboot_timer_cb, 0 );
	setup_timer( &reboot_sync_timer, hardkey_reboot_sync_timer_cb, 0 );

	reboot_trig_counter = reboot_trig_max_cnt;

	raw_notifier_chain_register( &reboot_key_notifier_list, 
	                             &hardkey_reboot_nb);
	
	return 0; 
}

static void __exit   
hardkey_reboot_exit(void)
{
	PDBG("%s: \n", __func__ );

	del_timer( &reboot_init_timer );
	del_timer( &reboot_sync_timer );

	raw_notifier_chain_unregister( &reboot_key_notifier_list, 
	                               &hardkey_reboot_nb);
}


module_init(hardkey_reboot_init);
module_exit(hardkey_reboot_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");





