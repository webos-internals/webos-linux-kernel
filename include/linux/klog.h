#ifndef __KLOG_H
#define __KLOG_H

#if CONFIG_KERNEL_LOG
extern void klog_write(const char *s, unsigned int count);
extern void klog_printf(const char *fmt, ...);
#else
#define klog_write(s, count) ((void)0)
#define klog_printf(fmt, ...) ((void)0)
#endif

#endif

