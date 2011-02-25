#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/pid_namespace.h>
#include <linux/notifier.h>
#include <linux/thread_info.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/posix-timers.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/tick.h>
#include <linux/kallsyms.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <asm/div64.h>
#include <asm/timex.h>
#include <asm/io.h>
#include <linux/kallsyms.h>

#include <linux/gen_timer.h>
#include <linux/rtc.h>

spinlock_t timer_lock; 
struct list_head timer_list_head;
extern struct kset power_subsys;
extern unsigned long sleep_offset;

gen_timer_stats_t gtimerstats;

static ssize_t timer_list_show(struct kset *subsys, char *buf)
{
	struct list_head *ptr;	
	unsigned long flags;
	struct timer_list *timer_ptr;
	int i=0,size_written=0,diff;
	unsigned long jiffies_rtc;
	
	spin_lock_irqsave(&timer_lock, flags);
	list_for_each(ptr,&timer_list_head) {
		timer_ptr=list_entry(ptr,struct timer_list,entry);
		jiffies_rtc=(jiffies+ (sleep_offset * HZ));
		diff=jiffies_to_msecs(timer_ptr->expires-jiffies_rtc);
		diff=diff/MSEC_PER_SEC;
		size_written+=sprintf(buf+size_written,"Timer %d : Expires in (%lu - %lu) %d sec : Handler : <%p> ",i++,timer_ptr->expires,jiffies_rtc,diff,timer_ptr->function);
		if(timer_ptr->data)
			size_written+=sprintf(buf+size_written,"data : %lx\n",timer_ptr->data);
		else
			size_written+=sprintf(buf+size_written,"data : NULL\n");
	}
	size_written+=sprintf(buf+size_written,"Battery Timeouts : %lu ::: TCP Timeouts : %lu ::: User Timeouts : %lu ::: Non-RTC Wakeups: %lu \n",
					gtimerstats.battery_timeouts,gtimerstats.tcp_timeouts,gtimerstats.user_timeouts,gtimerstats.non_rtc_wakeup);
	spin_unlock_irqrestore(&timer_lock, flags);
	return size_written;
}

/**
 * Init
 */

static struct subsys_attribute timerchk_sysfs = {
        .attr = {
                .name = __stringify(timer_list),
                .mode = 0644,
        },
        .show  = timer_list_show,
        .store = NULL,
};

int timerchk_sysfs_init(void)
{
        int retval;

        retval = subsys_create_file(&power_subsys, &timerchk_sysfs);
        if (retval)
                goto error;
	memset(&gtimerstats,0,sizeof(gen_timer_stats_t));

error:
        return retval;
}



void init_gen_timers(void) 
{	
	INIT_LIST_HEAD(&timer_list_head);
	spin_lock_init(&timer_lock);
}

void init_gen_timer(struct timer_list *timer,void (*function)(unsigned long),unsigned long data) 
{
	timer->function=function;
	timer->data=data;
	timer->entry.next=NULL;
}




/* Add the timer after sorting the expire values from all the timers */
static void internal_add_gen_timer(struct timer_list *timer)
{
	unsigned long expires = timer->expires;
	struct list_head *ptr;	
	unsigned long added_in_list=0;
	struct timer_list *timer_ptr;

	if(list_empty(&timer_list_head)){ 	// No entry in list yet ...
		list_add(&timer->entry,&timer_list_head);
		added_in_list=1;
	} else {
		list_for_each(ptr,&timer_list_head) {
			timer_ptr=list_entry(ptr,struct timer_list,entry);
			if(timer_ptr->expires > expires) {
				list_add_tail(&timer->entry,ptr);
				added_in_list=1;
				break;
			}
		}
	}
	if(!added_in_list) {	// Expire value greater than all timers in list so inserting in the end
		list_add_tail(&timer->entry,&timer_list_head);
	}
}


static inline void detach_gen_timer(struct timer_list *timer,
				int clear_pending)
{
	struct list_head *entry = &timer->entry;

	__list_del(entry->prev, entry->next);
	if (clear_pending)
		entry->next = NULL;
	entry->prev = LIST_POISON2;
}	

int __mod_gen_timer(struct timer_list *timer, unsigned long expires)
{
	unsigned long flags;
	int ret = 0;

	BUG_ON(!timer->function);

	spin_lock_irqsave(&timer_lock, flags);

	if (timer_pending(timer)) {
		detach_gen_timer(timer, 0);
		ret = 1;
	}
	timer->expires = expires + sleep_offset*HZ;

	internal_add_gen_timer(timer);
	spin_unlock_irqrestore(&timer_lock, flags);

	return ret;
}


int mod_gen_timer(struct timer_list *timer, unsigned long expires)
{
	BUG_ON(!timer->function);

	/*
	 * This is a common optimization triggered by the
	 * networking code - if the timer is re-modified
	 * to be the same thing then just return:
	 */
	if (timer->expires == expires && timer_pending(timer))
		return 1;

	return __mod_gen_timer(timer, expires);
}


int del_gen_timer(struct timer_list *timer)
{
	unsigned long flags;
	int ret = 0;

	if (timer_pending(timer)) {
		spin_lock_irqsave(&timer_lock,flags);
		if (timer_pending(timer)) {
			detach_gen_timer(timer, 1);
			ret = 1;
		}
		spin_unlock_irqrestore(&timer_lock, flags);
	}

	return ret;
}


void run_gen_timers(void)
{
	struct timer_list *timer_ptr;
	void (*fn)(unsigned long);
	unsigned long data;
	unsigned long jiffies_rtc;
	int diff_jiffies=0;

	spin_lock_irq(&timer_lock);

	while(!list_empty(&timer_list_head)) {
		timer_ptr=list_first_entry(&timer_list_head,struct timer_list,entry);
		jiffies_rtc = jiffies + sleep_offset*HZ;
		diff_jiffies=timer_ptr->expires - jiffies_rtc;
		if(diff_jiffies <=0) {
			fn=timer_ptr->function;
			data=timer_ptr->data;
			detach_gen_timer(timer_ptr,1);
			spin_unlock_irq(&timer_lock);
			fn(data);
			spin_lock_irq(&timer_lock);
		}
		else 
			break;
	}
	spin_unlock_irq(&timer_lock);
}

