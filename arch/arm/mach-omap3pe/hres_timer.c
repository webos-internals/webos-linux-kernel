#include <linux/clk.h>
#include <linux/hres_counter.h>
#include <asm/arch/dmtimer.h>

static int clk_rate = 0;

int omap_hres_timer_init(void **timer)
{
#ifndef CONFIG_HRES_COUNTER_TIMER32K
	struct omap_dm_timer *dmt;

	dmt = omap_dm_timer_request();
	if (dmt == NULL) {
		return -1;
	}

	omap_dm_timer_stop(dmt);
	omap_dm_timer_set_int_enable(dmt, 0);
	omap_dm_timer_write_counter(dmt, 0);
	omap_dm_timer_set_source(dmt, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_set_load(dmt, 1, 0);
	omap_dm_timer_start(dmt);

	clk_rate = clk_get_rate(omap_dm_timer_get_fclk(dmt));
	clk_rate /= 1000000; // in MHz

	*timer = dmt;

	printk(KERN_INFO "Initialized hres_timer (%dMHz)\n");
#else
	clk_rate = 32768;
	printk(KERN_INFO "Initialized hres_timer (32KHz)\n");
#endif

	return 0;
}

int omap_hres_timer_release(void *timer)
{
#ifndef CONFIG_HRES_COUNTER_TIMER32K
	omap_dm_timer_free(timer);
#endif
	return 0;
}

#ifdef CONFIG_PM
int omap_hres_timer_suspend(void *timer)
{
	omap_dm_timer_stop(timer);
	return 0;
}

int omap_hres_timer_resume(void *timer)
{
	omap_dm_timer_stop(timer);
	omap_dm_timer_set_int_enable(timer, 0);
	omap_dm_timer_write_counter(timer, 0);
	omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_set_load(timer, 1, 0);
	omap_dm_timer_start(timer);

	clk_rate = clk_get_rate(omap_dm_timer_get_fclk(timer));
	clk_rate /= 1000000; // in MHz
	return 0;
}
#endif

u32 omap_hres_timer_read(void *timer)
{
#ifndef CONFIG_HRES_COUNTER_TIMER32K
	return omap_dm_timer_read_counter(timer);
#else
	return omap_32k_sync_timer_read();
#endif
}

u32 omap_hres_timer_convert(u32 count)
{
#ifndef CONFIG_HRES_COUNTER_TIMER32K
	return count / clk_rate;
#else
	return count * 30, // 32K
#endif
}
