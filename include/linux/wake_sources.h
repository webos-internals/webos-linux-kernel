/*
 * wake_sources.h - Wake source logging interface
 *
 * Copyright (C) 2009 Palm, Inc.
 * Copyright (C) 2009 Corey Tabaka
 *
 * License goes here
 */

#ifndef __LINUX_WAKE_SOURCES_H
#define __LINUX_WAKE_SOURCES_H

#if defined(CONFIG_WAKE_SOURCES)

#include <linux/list.h>

struct wakeup_source {
	const char *name;
	struct list_head node;
	unsigned long count;
};

#define WAKEUP_SOURCE(source_name) { .name = source_name }

/*
 * Initializes the list of wakeup sources from the given array.
 * This can be called more than once, but the same array should not be
 * passed twice.
 */
void init_wakeup_sources(struct wakeup_source *sources,
		unsigned int count);

/*
 * Gets a reference to the wakeup_source struct for the given name.
 */
struct wakeup_source *get_wakeup_source(const char *name);

/*
 * Releases the reference to the given wakup_source struct.
 */
void put_wakeup_source(struct wakeup_source *source);

/*
 * Signals a wakeup event of the given source. Source should be obtained
 * by a call to get_wakeup_source(name).
 */
int wakeup_event(struct wakeup_source *source, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));

/*
 * Same as above except uses GFP_ATOMIC for kmalloc instead of detecting the
 * appropriate flags based on the situation.
 */
int wakeup_event_atomic(struct wakeup_source *source, const char *fmt, ...)
	__attribute__ ((format (printf, 2, 3)));

/*
 * Clears the list of wakeup sources for this period. This should be
 * called prior to going to sleep, after userspace is no longer able to
 * read the sysfs entries for this module.
 */
void clear_wakeup_events(void);

#else

/*
 * Only provide a default macro for wakeup_event to make drivers
 * cleaner. The other functions should be conditiinally compiled when
 * CONFIG_WAKE_SOURCES is defined.
 */

#define wakeup_event(args...)

#endif // defined(CONFIG_WAKE_SOURCES)

#endif

