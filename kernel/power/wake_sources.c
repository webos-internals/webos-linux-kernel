/*
 * wake_source.c - Wake source logging interface
 *
 * Copyright (C) 2009 Palm, Inc.
 * Copyright (C) 2009 Corey Tabaka
 *
 * License goes here
 */

#include <stdarg.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/kallsyms.h>
#include <linux/kobject.h>
#include <linux/hardirq.h>
#include "power.h"

#include <linux/wake_sources.h>

#define MAX_WAKEUP_EVENT_MSG_LEN 129

struct wakeup_event {
	struct list_head node;
	struct wakeup_source *source;

	unsigned long long timestamp;

	unsigned long caller;
	char message[MAX_WAKEUP_EVENT_MSG_LEN];
};

static LIST_HEAD(wakeup_sources);
static LIST_HEAD(wakeup_events);
static DEFINE_SPINLOCK(lock);

/*
 * Returns a pointer to the corresponding wakeup_source struct for the
 * given name or 0 if the name is not a registered wakeup source.
 */
struct wakeup_source *get_wakeup_source(const char *name)
{
	struct wakeup_source *source;
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry(source, &wakeup_sources, node) {
		if (!strcmp(name, source->name)) {
			spin_unlock_irqrestore(&lock, flags);
			return source;
		}
	}
	spin_unlock_irqrestore(&lock, flags);

	return NULL;
}
EXPORT_SYMBOL(get_wakeup_source);

void put_wakeup_source(struct wakeup_source *source)
{
	/* nothing needed at this time */
}
EXPORT_SYMBOL(put_wakeup_source);

void init_wakeup_sources(struct wakeup_source *sources,
		unsigned int count)
{
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&lock, flags);
	for (i=0; i < count; i++) {
		list_add_tail(&sources[i].node, &wakeup_sources);
		sources[i].count = 0;
	}
	spin_unlock_irqrestore(&lock, flags);

}
EXPORT_SYMBOL(init_wakeup_sources);

int wakeup_event_internal(int gfp_flags, struct wakeup_source *source, const char *fmt, va_list args)
	//__attribute__ ((format (printf, 2, 3)));
{
	struct wakeup_event *event;
	unsigned long flags;
	unsigned long caller = (unsigned long) __builtin_return_address(0);
	int ret;

	BUG_ON(!source);

	event = kmalloc(sizeof(struct wakeup_event), gfp_flags);
	if (!event) {
		printk(KERN_WARNING "%s: failed to allocate struct wakeup_event%s\n", __FUNCTION__,
				in_atomic() ? " in atomic context" : "");
		ret = -ENOMEM;
		goto done;
	}

	event->source = source;
	event->caller = caller;
	
	if (fmt) {
		vsnprintf(event->message, MAX_WAKEUP_EVENT_MSG_LEN, fmt, args); 
	} else {
		event->message[0] = '\0';
	}

	event->timestamp = cpu_clock(smp_processor_id());

	spin_lock_irqsave(&lock, flags);
	list_add_tail(&event->node, &wakeup_events);
	source->count++;
	spin_unlock_irqrestore(&lock, flags);

	ret = 0;
done:
	return ret;
}

int wakeup_event(struct wakeup_source *source, const char *fmt, ...)
{
	va_list args;
	int ret;
	
	if (fmt)
		va_start(args, fmt);

	ret = wakeup_event_internal(irqs_disabled() || in_atomic() ? GFP_ATOMIC : GFP_KERNEL, source, fmt, args);

	if (fmt)
		va_end(args);

	return ret;
}
EXPORT_SYMBOL(wakeup_event);

int wakeup_event_atomic(struct wakeup_source *source, const char *fmt, ...)
{
	va_list args;
	int ret;
	
	if (fmt)
		va_start(args, fmt);

	ret = wakeup_event_internal(GFP_ATOMIC, source, fmt, args);

	if (fmt)
		va_end(args);

	return ret;
}
EXPORT_SYMBOL(wakeup_event_atomic);

void clear_wakeup_events(void)
{
	unsigned long flags;
	struct wakeup_event *event, *next;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry_safe(event, next, &wakeup_events, node) {
		list_del(&event->node);
		kfree(event);
	}
	spin_unlock_irqrestore(&lock, flags);
}
EXPORT_SYMBOL(clear_wakeup_events);

static ssize_t wakeup_sources_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0;
	unsigned long flags;
	struct wakeup_source *source;

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry(source, &wakeup_sources, node) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%s %lu\n", source->name, source->count);
	}
	spin_unlock_irqrestore(&lock, flags);

	return len;
}

static ssize_t wakeup_events_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int len = 0, count;
	unsigned long flags, nanosec_rem;
	struct wakeup_event *event, *next;
	
	// only accessed while lock is held, so this should be ok
	static char temp[1024]; 
	static char sym[KSYM_SYMBOL_LEN];

	spin_lock_irqsave(&lock, flags);
	list_for_each_entry_safe(event, next, &wakeup_events, node) {
		if (event->caller)
			sprint_symbol(sym, (unsigned long)__builtin_extract_return_addr((void *) event->caller));
		else
			sym[0] = '\0';

		nanosec_rem = do_div(event->timestamp, 1000000000);
		count = snprintf(temp, sizeof(temp), "[%5lu.%06lu] %s (%s) %s (%08lx)\n",
				(unsigned long) event->timestamp,
				nanosec_rem / 1000,
				event->source->name,
				event->message,
				sym,
				event->caller
				);
		
		// only print as many entries as will fit in the buffer
		if (count <= PAGE_SIZE - len) {
			len += snprintf(buf + len, PAGE_SIZE - len, "%s", temp);
		} else {
			break; // no more entries will fit in the buffer. don't waste cycles holding the lock
		}
	}
	spin_unlock_irqrestore(&lock, flags);

	return len;
}

struct kobj_attribute registered_wakeup_sources_attr = {
	.attr = {
		.name = __stringify(registered_wakeup_sources),
		.mode = 0644,
	},
	.show = wakeup_sources_show,
	.store = NULL,
};

struct kobj_attribute wakeup_event_list_attr = {
	.attr = {
		.name = __stringify(wakeup_event_list),
		.mode = 0644,
	},
	.show = wakeup_events_show,
	.store = NULL,
};

static int __init wakeup_sources_init(void)
{
	return 0;
}

module_init(wakeup_sources_init);

