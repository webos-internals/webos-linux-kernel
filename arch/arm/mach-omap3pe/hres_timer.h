#ifndef __ARCH__ARM__MACH_OMAP3PE__HRES_TIMER_H
#define __ARCH__ARM__MACH_OMAP3PE__HRES_TIMER_H

int omap_hres_timer_init(void **timer);
int omap_hres_timer_release(void *timer);
#ifdef CONFIG_PM
int omap_hres_timer_suspend(void *timer);
int omap_hres_timer_resume(void *timer);
#else
#define omap_hres_timer_suspend    NULL
#define omap_hres_timer_resume     NULL
#endif
u32 omap_hres_timer_read(void *timer);
u32 omap_hres_timer_convert(u32 count);

#endif
