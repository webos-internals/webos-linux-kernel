/*
 * tcp_fastpath_client.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/rtc.h>
#include <linux/battery_simple.h>

#include <linux/gen_timer.h>
#include <linux/fastpath_client.h>
#include <net/inet_timewait_sock.h>
#include <net/tcp.h>

/*
 * Debug
 */

#define DEBUG(...)                           \
do {                                                         \
        if (tcp_fastpath_debug_level)                \
                printk(__VA_ARGS__);                 \
} while (0)

#define INFO(...)                            \
do {                                         \
        printk(KERN_INFO __VA_ARGS__);           \
} while (0)

#define ERROR(...)                           \
do {                                         \
        printk(KERN_ERR __VA_ARGS__);            \
} while (0)


/**
 * Module parameters
 */

/** 
* @brief Turn on debug messages.
*/
static int32_t tcp_fastpath_debug_level = 0;
module_param_named(debug_level, tcp_fastpath_debug_level, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "tcp fastpath client debug level.");

#define MIN_TCP_TIMEOUT	15

extern gen_timer_stats_t gtimerstats;
extern unsigned long sleep_offset;
extern struct list_head timer_list_head;
extern spinlock_t timer_lock;

int is_local_socket(struct sock *sk) 
{
	struct inet_sock *inet;
	__be32 src,dest;

	inet = inet_sk(sk);
	if(inet) {
		src =  inet->rcv_saddr;
		dest = inet->daddr;
		if(!src || src == 0x0100007f || !dest)
			return 1;
	}

	return 0;
}

int tcp_prepare_handler(void) 
{
	struct timer_list *timer;
	int next_alarm=0,diff_jiffies;
	struct list_head *ptr;
	unsigned long flags;

	DEBUG("======%s\n", __FUNCTION__);
	/* Calculate the next alarm from the timer_list*/
	
	run_gen_timers();
#ifdef CONFIG_INTSOCK_NETFILTER
	check_idle_sockets();
#endif
	spin_lock_irqsave(&timer_lock,flags);
	list_for_each(ptr,&timer_list_head) {
	        timer=list_entry(ptr,struct timer_list,entry);
		if(timer->function == inet_twdr_hangman || timer->function == inet_twdr_twcal_tick) {
			DEBUG("%s:We wont wakeup for TIME_WAIT or FIN_WAIT_2 sockets\n",__FUNCTION__);
			continue;
		}
		if(is_local_socket((struct sock *)timer->data)) {
			DEBUG("%s:We wont wakeup for the local socket %lx\n",__FUNCTION__,timer->data);
			continue;
		}	
		diff_jiffies=timer->expires - (jiffies + sleep_offset*HZ);
		if(diff_jiffies < 0) {
			DEBUG("%s: Timer already expired \n",__FUNCTION__);
			continue;
		}
		next_alarm=(diff_jiffies)/HZ + 1;
		DEBUG("tcp handler :  <%p> for data : %lx next_alarm : %d\n",timer->function,
			timer->data,next_alarm);
			
		if(next_alarm < MIN_TCP_TIMEOUT)
			next_alarm = MIN_TCP_TIMEOUT;
		break;
	}
	spin_unlock_irqrestore(&timer_lock,flags);
	DEBUG("======%s  Returning alarm timeout : %d\n", __FUNCTION__,next_alarm);
	return next_alarm;	
}

int tcp_fastwake_handler(void) 
{
	struct timer_list *timer;
	struct list_head *ptr;
	unsigned long flags;

	DEBUG("======%s\n", __FUNCTION__);
	DEBUG("Orig jiffies : %lu rtc jiffies : %lu\n",jiffies,jiffies + (sleep_offset*HZ));
	if(tcp_fastpath_debug_level) {
		spin_lock_irqsave(&timer_lock,flags);
		list_for_each(ptr,&timer_list_head) {
			timer=list_entry(ptr,struct timer_list,entry);
			if(timer->function == inet_twdr_hangman || timer->function == inet_twdr_twcal_tick) {
				DEBUG("%s:We did not wakeup for TIME_WAIT or FIN_WAIT_2 sockets\n",__FUNCTION__);
				continue;
			}
			if(is_local_socket((struct sock *)timer->data)) {
				DEBUG("%s:We did not wakeup for the local socket %lx\n",__FUNCTION__,timer->data);
				continue;
			}	

			DEBUG("Expiring jiffies : %lu rtc jiffies : %lu\n",timer->expires,
					jiffies + (sleep_offset*HZ));
			if(timer->function == inet_twdr_hangman || timer->function == inet_twdr_twcal_tick) 
				DEBUG("%s: TCP RTC wakeup for TIME_WAIT socket \n",__FUNCTION__);
			else 
				DEBUG("%s: TCP RTC wakeup for Retransmission/Delayed ack/Keep-alive"
						" timeout for socket : %lx \n",__FUNCTION__,timer->data);

		}
		spin_unlock_irqrestore(&timer_lock,flags);
	}
	gtimerstats.tcp_timeouts++;
	run_gen_timers();
	return 0;
}

static fastpath_client_t tcp_client = {
	.name = "TCP Fastpath Client",
	.prepare = tcp_prepare_handler,
	.fastwake = tcp_fastwake_handler,
	.next_alarm = 0,
};

static int __init tcp_fastpath_init(void)
{
	register_fastpath_client(&tcp_client);
	return 0;
}


static int __init tcp_fastpath_module_init(void)
{
	int rc;


	rc = tcp_fastpath_init();
	if (rc) {
		ERROR("%s could not init fastpath: %d\n",
				__FUNCTION__, rc);
		return -1;
	}

	timerchk_sysfs_init();
	return 0;
}

static void __exit tcp_fastpath_module_exit(void)
{
	unregister_fastpath_client(&tcp_client);
}

module_init(tcp_fastpath_module_init);
module_exit(tcp_fastpath_module_exit);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("Provides the ability to periodically wake device to check for tcp timeouts");
MODULE_LICENSE("GPL");
