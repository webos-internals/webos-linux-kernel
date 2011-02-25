#ifndef _GEN_TIMER_H
#define _GEN_TIMER_H

#include <linux/list.h>
#include <linux/timer.h>

typedef struct gen_timer_stats {
        unsigned long battery_timeouts;
        unsigned long tcp_timeouts;
        unsigned long user_timeouts;
	unsigned long non_rtc_wakeup;
}gen_timer_stats_t;

void init_gen_timers(void);
int timerchk_sysfs_init(void);
void init_gen_timer(struct timer_list *timer,void (*function)(unsigned long),unsigned long data);
int mod_gen_timer(struct timer_list *timer, unsigned long expires);
int del_gen_timer(struct timer_list *timer);
void run_gen_timers(void);

#endif


