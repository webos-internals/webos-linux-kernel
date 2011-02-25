#ifndef __AUDIO_DEBUG_H
#define __AUDIO_DEBUG_H

//#define LOCAL_TRACE
//#define TRACE

#ifdef TRACE
static enum {
	OMAP_ALSA_TRACE_DRIVER = (1 << 0),
	OMAP_ALSA_TRACE_TWL4030 = (1 << 1)
} omap_alsa_trace_group = 0xffff;

static int omap_alsa_trace_level = 30;
#endif

#ifdef LOCAL_TRACE
#define LTRACE_ENTRY \
	do { printk(KERN_INFO "%s: entry\n", __func__); } while (0)
#define LTRACE_EXIT \
	do { printk(KERN_INFO "%s: exit\n", __func__); } while (0)
#else
#define LTRACE_ENTRY
#define LTRACE_EXIT
#endif

#ifdef TRACE
#define TRACEF(level, group, x...) \
	do { \
		if ((group & omap_alsa_trace_group) && \
				(level <= omap_alsa_trace_level)) { \
			printk(KERN_INFO "%s: ", __func__); \
			printk(KERN_INFO x); \
		} \
	} while (0)
#else
#define TRACEF(level, group, x...)
#endif

#endif
