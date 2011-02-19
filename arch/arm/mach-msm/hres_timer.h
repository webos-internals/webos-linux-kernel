#ifndef __ARCH__ARM__MACH_MSM__HRES_TIMER_H
#define __ARCH__ARM__MACH_MSM__HRES_TIMER_H

int msm_hres_timer_init(void **timer);
int msm_hres_timer_release(void *timer);
#ifdef CONFIG_PM
int msm_hres_timer_suspend(void *timer);
int msm_hres_timer_resume(void *timer);
#else
#define msm_hres_timer_suspend    NULL
#define msm_hres_timer_resume     NULL
#endif
u32 msm_hres_timer_read(void *timer);
u32 msm_hres_timer_convert(u32 count);

#endif
