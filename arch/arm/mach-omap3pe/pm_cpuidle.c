/*
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP3 CPU idle implmentation.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu <karthik-dp@ti.com>
 *
 * Copyright (C) 2006 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/cpuidle.h>
#include <linux/seq_file.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>
#include <linux/mm.h>
#include <asm/mmu.h>
#include <asm/tlbflush.h>

#include <asm/arch/irqs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sram.h>
#include <asm/arch/dma.h>
#include <asm/arch/pm.h>
#include <asm/arch/omap24xx-uart.h>
#include <asm/arch/display.h>
#include <linux/tick.h>

#include <linux/types.h>

#include <asm/arch/prcm.h>
#include <asm/arch/resource.h>

#include "prcm-regs.h"
#include "prcm_pwr.h"
#include "prcm_util.h"
#include "pm.h"
#include "serial.h"
#include "sram_idle.h"
#include "scratchpad.h"
#include "pm_sysfs.h"
#include "context.h"

#include "pm_cpuidle.h"

/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

#define DEBUG_BAIL_STATS
//#define DEBUG_STATES

#ifdef DEBUG_CPUIDLE
# define DPRINTK(fmt, args...) \
	printk("CPUIDLE %s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

/******************************************************************************/

#ifdef DEBUG_STATES
/* print every nth capture. */
static int capture_count = 0;
#define DEBUG_STATE_PRINT_INTERVAL 30

static u32 _pwstctrl_core  = 0xdeadbeef;
static u32 _iclken1_core   = 0xdeadbeef;
static u32 _iclken2_core   = 0xdeadbeef;
static u32 _fclken1_core   = 0xdeadbeef;
static u32 _iclken3_core   = 0xdeadbeef;
static u32 _fclken3_core   = 0xdeadbeef;
static u32 _idlest1_core   = 0xdeadbeef;
static u32 _idlest2_core   = 0xdeadbeef;
static u32 _idlest3_core   = 0xdeadbeef;
static u32 _iclken_dss     = 0xdeadbeef;
static u32 _fclken_dss     = 0xdeadbeef;
static u32 _iclken_per     = 0xdeadbeef;
static u32 _fclken_per     = 0xdeadbeef;
static u32 _iclken_wkup    = 0xdeadbeef;
static u32 _fclken_wkup    = 0xdeadbeef;
static u32 _idlest_wkup    = 0xdeadbeef;
static u32 _idlest_per     = 0xdeadbeef;
static u32 _idlest_cam     = 0xdeadbeef;
static u32 _idlest_iva2    = 0xdeadbeef;
static u32 _idlest_dss     = 0xdeadbeef;
static u32 _idlest_usbhost = 0xdeadbeef;
static u32 _clkstst_core   = 0xdeadbeef;
static u32 _idlest_ckgen   = 0xdeadbeef;

static void DEBUG_STATE_CAPTURE(void)
{
	_pwstctrl_core  = PM_PWSTCTRL_CORE;
	_iclken1_core   = CM_ICLKEN1_CORE;
	_iclken2_core   = CM_ICLKEN2_CORE;
	_fclken1_core   = CM_FCLKEN1_CORE;
	_iclken3_core   = CM_ICLKEN3_CORE;
	_fclken3_core   = CM_FCLKEN3_CORE;
	_iclken_dss     = CM_ICLKEN_DSS;
	_fclken_dss     = CM_FCLKEN_DSS;
	_iclken_per     = CM_ICLKEN_PER;
	_fclken_per     = CM_FCLKEN_PER;
	_iclken_wkup    = CM_ICLKEN_WKUP;
	_fclken_wkup    = CM_FCLKEN_WKUP;
	_idlest1_core   = CM_IDLEST1_CORE;
	_idlest2_core   = CM_IDLEST2_CORE;
	_idlest3_core   = CM_IDLEST3_CORE;
	_idlest_wkup    = CM_IDLEST_WKUP;
	_idlest_per     = CM_IDLEST_PER;
	_idlest_cam     = CM_IDLEST_CAM;
	_idlest_iva2    = CM_IDLEST_IVA2;
	_idlest_dss     = CM_IDLEST_DSS;
	_idlest_usbhost = CM_IDLEST_USBHOST;
	_clkstst_core   = CM_CLKSTST_CORE;
	_idlest_ckgen   = CM_IDLEST_CKGEN;

	capture_count = (capture_count + 1) % DEBUG_STATE_PRINT_INTERVAL;
}

static void DEBUG_STATE_PRINT(int cond)
{
	if (!cond)
		return;

	if (1 != capture_count)
		return;

	printk("#########################################\n");
	printk("##### _pwstctrl_core  %08x\n", _pwstctrl_core);
	printk("##### _clkstst_core   %08x\n", _clkstst_core);
	printk("##### _iclken1_core   %08x\n", _iclken1_core);
	printk("##### _iclken2_core   %08x\n", _iclken2_core);
	printk("##### _fclken1_core   %08x\n", _fclken1_core);
	printk("##### _iclken3_core   %08x\n", _iclken3_core);
	printk("##### _fclken3_core   %08x\n", _fclken3_core);
	printk("##### _iclken_wkup    %08x\n", _iclken_wkup);
	printk("##### _fclken_wkup    %08x\n", _fclken_wkup);
	printk("##### _iclken_dss     %08x\n", _iclken_dss);
	printk("##### _fclken_dss     %08x\n", _fclken_dss);
	printk("##### _iclken_per     %08x\n", _iclken_per);
	printk("##### _fclken_per     %08x\n", _fclken_per);
	printk("##### _idlest1_core   %08x\n", _idlest1_core);
	printk("##### _idlest2_core   %08x\n", _idlest2_core);
	printk("##### _idlest3_core   %08x\n", _idlest3_core);
	printk("##### _idlest_wkup    %08x\n", _idlest_wkup);
	printk("##### _idlest_per     %08x\n", _idlest_per);
	printk("##### _idlest_cam     %08x\n", _idlest_cam);
	printk("##### _idlest_iva2    %08x\n", _idlest_iva2);
	printk("##### _idlest_dss     %08x\n", _idlest_dss);
	printk("##### _idlest_usbhost %08x\n", _idlest_usbhost);
	printk("##### _idlest_ckgen   %08x\n", _idlest_ckgen);

	printk("##### PM_PREPWSTST_MPU     %08x\n", PM_PREPWSTST_MPU);
	printk("##### PM_PREPWSTST_CORE    %08x\n", PM_PREPWSTST_CORE);
	printk("##### PM_PREPWSTST_SGX     %08x\n", PM_PREPWSTST_SGX);
	printk("##### PM_PREPWSTST_DSS     %08x\n", PM_PREPWSTST_DSS);
	printk("##### PM_PREPWSTST_CAM     %08x\n", PM_PREPWSTST_CAM);
	printk("##### PM_PREPWSTST_PER     %08x\n", PM_PREPWSTST_PER);
	printk("##### PM_PREPWSTST_NEON    %08x\n", PM_PREPWSTST_NEON);
	printk("##### PM_PREPWSTST_IVA2    %08x\n", PM_PREPWSTST_IVA2);
	printk("##### PM_PREPWSTST_USBHOST %08x\n", PM_PREPWSTST_USBHOST);
}

#else
#define DEBUG_STATE_CAPTURE()
#define DEBUG_STATE_PRINT(cond)
#endif

/******************************************************************************
 *
 * Not happy with those defines being in this file. Need to move them somewhere
 * else.
 *
 ******************************************************************************/

#define CORE_FCLK_MASK	0x3FFFE29   /* Mask of all functional clocks */
#define CORE3_FCLK_MASK		0x7 /* Mask of all functional clocks */
#define USBHOST_FCLK_MASK	0x3 /* Mask of all functional clocks in USB */
#define SGX_FCLK_MASK		0x2 /* Mask of all functional clocks in SGX */

#define DSS_FCLK_MASK		0x7 /* Mask of all functional clocks in DSS */
#define CAM_FCLK_MASK		0x1 /* Mask of all functional clocks in CAM */
#define PER_FCLK_MASK		0x17FF /* Mask of all functional clocks in PER */
				       /* except UART and GPIO */

#define CORE1_ICLK_VALID	0x3FFFFFF9 /*Ignore SDRC_ICLK*/
#define CORE2_ICLK_VALID	0x1F
#define CORE3_ICLK_VALID	0x4
#define USBHOST_ICLK_VALID	0x1
#define	SGX_ICLK_VALID		0x1

#define DSS_ICLK_VALID		0x1
#define CAM_ICLK_VALID		0x1
#define PER_ICLK_VALID		0x1
#define WKUP_ICLK_VALID		0x3E /* Ignore GPT1 ICLK as it is handled explicitly */

#define OMAP3_MAX_STATES 7
#define OMAP3_STATE_C1	1	/* MPU ACTIVE + CORE ACTIVE(only WFI)*/
#define OMAP3_STATE_C2	2	/* MPU WFI + Dynamic tick + CORE ACTIVE*/
#define OMAP3_STATE_C3	3	/* MPU CSWR + CORE ACTIVE*/
#define OMAP3_STATE_C4	4	/* MPU OFF + CORE ACTIVE*/
#define OMAP3_STATE_C5	5	/* MPU RET + CORE CSWR */
#define OMAP3_STATE_C6	6	/* MPU OFF + CORE CSWR */
#define OMAP3_STATE_C7	7	/* MPU OFF + CORE OFF */
#ifdef CONFIG_CORE_OFF_CPUIDLE
#define DEEPEST_SUPPORTED_STATE 6 /* deepest state == C7 (CORE OFF) */
#else
#define DEEPEST_SUPPORTED_STATE 4 /* deepest state == C5 (CORE RET) */
#endif

/* cpuidle_deepest_st is used in the cpuidle "menu" governor
 * drivers/cpuidle/governors/menu.c
 *
 * cpuidle_deepest_st index is offset by 1 from OMAP3_STATE_Cx as the first
 * entry (index 0) of the C-state table is C1.
 */
#ifdef CONFIG_PREVENT_MPU_RET
int cpuidle_deepest_st = 1; /* prevent states > C2 */
#elif defined(CONFIG_PREVENT_CORE_RET)
int cpuidle_deepest_st = 3; /* prevent states > C4 */
#else
int cpuidle_deepest_st = DEEPEST_SUPPORTED_STATE;
#endif


/******************************************************************************
 *
 * CPUIDLE power states table
 *
 ******************************************************************************/

struct omap3_processor_cx {
	u8 valid;
	u8 type;
	u32 sleep_latency;
	u32 wakeup_latency;
	u32 threshold;
	u32 mpu_state;
	u32 core_state;
	u32 flags;
};

static struct omap3_processor_cx omap3_power_states[OMAP3_MAX_STATES] = {
	{ /* C1 */
		1,
		OMAP3_STATE_C1,
		0,
		0,
		0,
		PRCM_MPU_ACTIVE,
		PRCM_CORE_ACTIVE,
		CPUIDLE_FLAG_SHALLOW,
	},
	{ /* C2 */
		1,
		OMAP3_STATE_C2,
		10,
		10,
		30,
		PRCM_MPU_ACTIVE,
		PRCM_CORE_ACTIVE,
		CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_SHALLOW,
	},
	{ /* C3 */
		1,
		OMAP3_STATE_C3,
		50,
		50,
		300,
		PRCM_MPU_CSWR_L2RET,
		PRCM_CORE_ACTIVE,
		CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_BALANCED,
	},
	{ /* C4 */
		1,
		OMAP3_STATE_C4,
		1500,
		1800,
		4000,
#ifdef _disabled_CONFIG_MPU_OFF
		PRCM_MPU_OFF,
#else
		PRCM_MPU_CSWR_L2RET,
#endif
		PRCM_CORE_ACTIVE,
		CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_BALANCED,
	},
	{ /* C5 */
		1,
		OMAP3_STATE_C5,
		2500,
		7500,
		12000,
		PRCM_MPU_CSWR_L2RET,
		PRCM_CORE_CSWR_MEMRET,
		CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_CHECK_BM,
	},
	{ /* C6 */
		0, /* DISABLED */
		OMAP3_STATE_C6,
		3000,
		8500,
		15000,
#ifdef _disabled_CONFIG_MPU_OFF
		PRCM_MPU_OFF,
#else
		PRCM_MPU_CSWR_L2RET,
#endif
		PRCM_CORE_CSWR_MEMRET,
		CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_BALANCED | CPUIDLE_FLAG_CHECK_BM,
	},
	{ /* C7 */
#ifdef CONFIG_CORE_OFF_CPUIDLE
		1,
#else
		0, /* DISABLED */
#endif
		OMAP3_STATE_C7,
		10000,
		30000,
		300000,
		PRCM_MPU_OFF,
		PRCM_CORE_OFF,
		CPUIDLE_FLAG_TIME_VALID | CPUIDLE_FLAG_DEEP | CPUIDLE_FLAG_CHECK_BM,
	},
};

/******************************************************************************
 *
 * SYSFS: deepest idle state
 *
 ******************************************************************************/

#ifdef CONFIG_SYSFS

#ifdef DEBUG_BAIL_STATS
/* Set in omap3_enter_idle() after wake-up.
 */
static struct timespec ts_last_wake_up = { 0 };

#define CLOCK_BAIL_TRACE_MAX	40
static struct {
	const char *desc;
	u32        cond;
	s64        first_entry;
	s64        total_residency;
	s64        sleep_residency;
	s64        wake_residency;
	u32        num_wakeups;
} cbt_list[CLOCK_BAIL_TRACE_MAX];

static int cbt_idx = 0;

static void clock_bail_trace(const char *s, u32 cond, int id)
{
	struct timespec ts_now;
	s64 now;
	static int last_id = 0;
	static s64 last_state_change = 0;

	getnstimeofday(&ts_now);
	now = timespec_to_ns(&ts_now);

	/* Update the total and sleep time for the current entry. Doing this
	 * here esnures that we have a consistent entry even if there was no
	 * state change yet.
	 */
	if (last_state_change != 0) {
		cbt_list[cbt_idx].total_residency = (now - last_state_change);
		cbt_list[cbt_idx].sleep_residency =
					cbt_list[cbt_idx].total_residency -
					cbt_list[cbt_idx].wake_residency;
	}

	if (id == last_id) {
		/* We woke up and came back here, but the bail condition has
		 * not changed. We add the time we were awake during this
		 * condition.
		 */
		s64 last_wake_up = timespec_to_ns(&ts_last_wake_up);

		cbt_list[cbt_idx].wake_residency += (now - last_wake_up);

		/* Count wake-up event.
		 */
		cbt_list[cbt_idx].num_wakeups++;
		return;
	}

	/* Record state change time.
	 */
	last_state_change = now;
	last_id = id;

	/* Switch to next entry, set first_entry timestamp and clear out the
	 * rest of the entry.
	 */
	cbt_idx = (cbt_idx +1) % CLOCK_BAIL_TRACE_MAX;

	cbt_list[cbt_idx].first_entry = now;
	cbt_list[cbt_idx].desc        = s;
	cbt_list[cbt_idx].cond        = cond;
	cbt_list[cbt_idx].total_residency = 0;
	cbt_list[cbt_idx].sleep_residency = 0;
	cbt_list[cbt_idx].wake_residency  = 0;
	cbt_list[cbt_idx].num_wakeups     = 0;
}

#define to_msec(n) (u32)((n) >> 20)
static ssize_t omap_pm_clock_bail_stat_show(struct kset *subsys, char *buf)
{
	int i;
	int len;
	int idx;
	int n = 0;

	len = sprintf(buf, "Time   Total Sleep ms/%% Wake ms/%%  WUp Condition\n");
	buf += len;
	n   += len;
	len = sprintf(buf, "==========================================================================\n");
	buf += len;
	n   += len;

	idx = (cbt_idx +1) % CLOCK_BAIL_TRACE_MAX;
	for (i = 0; i < CLOCK_BAIL_TRACE_MAX; i++) {
		u32 total_ms;
		u32 sleep_ms;
		u32 sleep_perc;

		if (!cbt_list[idx].desc) {
			idx = (idx +1) % CLOCK_BAIL_TRACE_MAX;
			continue;
		}

		total_ms = to_msec(cbt_list[idx].total_residency);
		sleep_ms = to_msec(cbt_list[idx].sleep_residency);
		if (total_ms)
			sleep_perc = (sleep_ms * 100) / total_ms;
		else
			sleep_perc = 0;

		len = sprintf(buf, "%6u %5u %5u %3u%% %5u %3u%% %3u %s %08x\n",
				to_msec(cbt_list[idx].first_entry) % 1000000,
				total_ms,
				sleep_ms, sleep_perc,
				to_msec(cbt_list[idx].wake_residency),
				100 - sleep_perc,
				cbt_list[idx].num_wakeups,
				cbt_list[idx].desc,
				cbt_list[idx].cond);
		buf += len;
		n   += len;
		idx = (idx +1) % CLOCK_BAIL_TRACE_MAX;

		/* Don't overwrite the page... */
		if (n > 4000)
			break;
	}
	return n;
}

static struct subsys_attribute clock_bail_stat = {
	.attr = {
		.name = __stringify(clock_bail_stat),
		.mode = 0444,
	},
	.show  = omap_pm_clock_bail_stat_show,
};
#else
#define clock_bail_trace(args...)
#endif /* #ifdef DEBUG_BAIL_STATS */

static ssize_t omap_pm_sleep_idle_state_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%hu\n", cpuidle_deepest_st);
}

static ssize_t omap_pm_sleep_idle_state_store(struct kset *subsys,
			const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 || (value > DEEPEST_SUPPORTED_STATE)) {
		printk(KERN_ERR "idle_sleep_store: Invalid value\n");
		return -EINVAL;
	}
	cpuidle_deepest_st = value;
	return n;
}

static struct subsys_attribute sleep_idle_state = {
	.attr = {
		.name = __stringify(cpuidle_deepest_state),
		.mode = 0644,
	},
	.show  = omap_pm_sleep_idle_state_show,
	.store = omap_pm_sleep_idle_state_store,
};

static int __init pm_cpuidle_sysfs_init(void)
{
	int rc;

	rc = subsys_create_file(&power_subsys, &sleep_idle_state);
	if (rc)
		printk(KERN_ERR "ERROR creating sysfs entry for "
				"'sleep_idle_state': %d\n", rc);

#ifdef DEBUG_BAIL_STATS
	rc = subsys_create_file(&power_subsys, &clock_bail_stat);
	if (rc)
		printk(KERN_ERR "ERROR creating sysfs entry for "
				"'clock_bail_stat': %d\n", rc);
#endif

	return rc;
}
__initcall(pm_cpuidle_sysfs_init);

#endif /* CONFIG_SYSFS */

/******************************************************************************
 *
 * Statistics / Previous Power States
 *
 ******************************************************************************/

#ifdef CONFIG_PROC_FS

#define LAST_IDLE_STATE_ARR_SIZE 32

static u32 mpu_ret_cnt  = 0;
static u32 mpu_off_cnt  = 0;
static u32 core_ret_cnt = 0;
static u32 core_off_cnt = 0;

static u32 wkup_wkup_cnt [32] = {0};
static u32 core1_wkup_cnt[32] = {0};
static u32 core3_wkup_cnt[32] = {0};
static u32 per_wkup_cnt  [32] = {0};
static u32 usbh_wkup_cnt [32] = {0};

static struct idle_state {
	u32 mpu_state;
	u32 core_state;
	u32 fclks;
	u32 iclks;
	u8  iva_state;
	u32 wkst_wkup ;
	u32 wkst1_core;
	u32 wkst3_core;
	u32 wkst_usbhost;
	u32 wkst_per;
 } last_idle_state[LAST_IDLE_STATE_ARR_SIZE];

static u32 idle_widx = 0;

static void *pm_cpuidle_prepwst_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *pm_cpuidle_prepwst_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void pm_cpuidle_count_wk_events ( u32 *cnt, u32 ev )
{
	int i, mask = 1;

	if(!ev) 
		return;

	for( i = 0; i < 32; i++, mask <<= 1) {
		if( mask & ev ) {
			cnt[i]++;
		}
	}
}

static void pm_cpuidle_prepwst_stop(struct seq_file *m, void *v)
{
}

static void pm_cpuidle_show_wkup_count (struct seq_file *m, char *header, u32 *cnt)
{
	int i;

	seq_printf(m, "%s:\n", header );
	for( i = 0; i < 32; i++ ) {
		if(cnt[i] == 0)
			continue;
		seq_printf(m, "[%2d]:  %d\n", i, cnt[i] );
	}
	
}


int pm_cpuidle_prepwst_show(struct seq_file *m, void *v)
{
	static char *pstates[4] = { "OFF", "RET", "INACT", "ON " };

	int i = LAST_IDLE_STATE_ARR_SIZE;

	u32 ridx = idle_widx;

	while (i--) {
		if (ridx == 0)
			ridx = LAST_IDLE_STATE_ARR_SIZE;

		ridx--;
		seq_printf(m, "MPU = %x - %s,  CORE = %x - %s, IVA = %x - %s,"
				"fclks: %x, iclks: %x "
				"wkup=%08x, core=%08x, per=%08x, core3=%08x, usbh=%08x"
				"\n",
				last_idle_state[ridx].mpu_state,
				pstates[last_idle_state[ridx].mpu_state & 0x3],
				last_idle_state[ridx].core_state,
				pstates[last_idle_state[ridx].core_state & 0x3],
				last_idle_state[ridx].iva_state,
				pstates[last_idle_state[ridx].iva_state & 0x3],
				last_idle_state[ridx].fclks,
				last_idle_state[ridx].iclks,
				last_idle_state[ridx].wkst_wkup,
				last_idle_state[ridx].wkst1_core,
				last_idle_state[ridx].wkst_per,
				last_idle_state[ridx].wkst3_core,
				last_idle_state[ridx].wkst_usbhost);
	}

	seq_printf(m, "\nMPU RET CNT = %d, MPU OFF CNT = %d, CORE RET CNT= %d,"
			" CORE OFF CNT= %d\n\n",
			mpu_ret_cnt, mpu_off_cnt, core_ret_cnt, core_off_cnt);

	seq_printf(m, "\nWKUP EVENTS:\n");
	pm_cpuidle_show_wkup_count ( m, "WKUP",     wkup_wkup_cnt  );
	pm_cpuidle_show_wkup_count ( m, "CORE1",    core1_wkup_cnt );
	pm_cpuidle_show_wkup_count ( m, "PER",      per_wkup_cnt   );
	pm_cpuidle_show_wkup_count ( m, "CORE3",    core3_wkup_cnt );
	pm_cpuidle_show_wkup_count ( m, "USBHOST",  usbh_wkup_cnt  );

	return 0;
}

static struct seq_operations omap_pm_prepwst_op = {
	.start = pm_cpuidle_prepwst_start,
	.next  = pm_cpuidle_prepwst_next,
	.stop  = pm_cpuidle_prepwst_stop,
	.show  = pm_cpuidle_prepwst_show
};

static int pm_cpuidle_prepwst_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &omap_pm_prepwst_op);
}

static struct file_operations proc_pm_prepwst_ops = {
	.open           = pm_cpuidle_prepwst_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = seq_release,
};

/* This API creates a proc entry for shared resources. */
static int create_pmproc_entry(void)
{
	struct proc_dir_entry *entry;

	/* Create a proc entry for shared resources */
	entry = create_proc_entry("pm_prepwst", 0, NULL);
	if (entry) {
		entry->proc_fops = &proc_pm_prepwst_ops;
		printk(KERN_ERR "create_proc_entry succeeded\n");
	} else
		printk(KERN_ERR "create_proc_entry failed\n");

	return 0;
}

static void store_prepwst(void)
{
	last_idle_state[idle_widx].mpu_state     = PM_PREPWSTST_MPU;
	last_idle_state[idle_widx].core_state    = PM_PREPWSTST_CORE;

	last_idle_state[idle_widx].wkst_wkup	 = PM_WKST_WKUP;
	last_idle_state[idle_widx].wkst1_core	 = PM_WKST1_CORE;
	last_idle_state[idle_widx].wkst_per	 = PM_WKST_PER;
	last_idle_state[idle_widx].wkst3_core    = PM_WKST3_CORE;
	last_idle_state[idle_widx].wkst_usbhost  = PM_WKST_USBHOST;

	pm_cpuidle_count_wk_events ( wkup_wkup_cnt,  last_idle_state[idle_widx].wkst_wkup  );
	pm_cpuidle_count_wk_events ( core1_wkup_cnt, last_idle_state[idle_widx].wkst1_core );
	pm_cpuidle_count_wk_events ( per_wkup_cnt,   last_idle_state[idle_widx].wkst_per   );
	pm_cpuidle_count_wk_events ( core3_wkup_cnt, last_idle_state[idle_widx].wkst3_core );
	pm_cpuidle_count_wk_events ( usbh_wkup_cnt,  last_idle_state[idle_widx].wkst_usbhost );
	

	if ((PM_PREPWSTST_MPU & 0x3) == 0x1)
		mpu_ret_cnt++;
	if ((PM_PREPWSTST_MPU & 0x3) == 0x0)
		mpu_off_cnt++;
	if ((PM_PREPWSTST_CORE & 0x3) == 0x1)
		core_ret_cnt++;
	if ((PM_PREPWSTST_CORE & 0x3) == 0x0)
		core_off_cnt++;

	last_idle_state[idle_widx].fclks =
		(CM_FCLKEN1_CORE & CORE_FCLK_MASK) |
		(CM_FCLKEN_SGX & SGX_FCLK_MASK) |
		(CM_FCLKEN_DSS & DSS_FCLK_MASK) |
		(CM_FCLKEN_USBHOST & USBHOST_FCLK_MASK) |
		(CM_FCLKEN3_CORE & CORE3_FCLK_MASK) |
		(CM_FCLKEN_CAM & CAM_FCLK_MASK) |
		(CM_FCLKEN_PER & PER_FCLK_MASK);

	last_idle_state[idle_widx].iclks =
		(CORE1_ICLK_VALID & (CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE)) |
		(CORE2_ICLK_VALID & (CM_ICLKEN2_CORE & ~CM_AUTOIDLE2_CORE)) |
		(CORE3_ICLK_VALID & (CM_ICLKEN3_CORE & ~CM_AUTOIDLE3_CORE)) |
		(SGX_ICLK_VALID & (CM_ICLKEN_SGX)) |
		(USBHOST_ICLK_VALID & (CM_ICLKEN_USBHOST)) |
		(DSS_ICLK_VALID & (CM_ICLKEN_DSS & ~CM_AUTOIDLE_DSS)) |
		(CAM_ICLK_VALID & (CM_ICLKEN_CAM & ~CM_AUTOIDLE_CAM)) |
		(PER_ICLK_VALID & (CM_ICLKEN_PER & ~CM_AUTOIDLE_PER)) |
		(WKUP_ICLK_VALID & (CM_ICLKEN_WKUP & ~CM_AUTOIDLE_WKUP));

	prcm_get_power_domain_state(
			DOM_IVA2, &(last_idle_state[idle_widx].iva_state));
	idle_widx++;

	if (idle_widx == LAST_IDLE_STATE_ARR_SIZE)
		idle_widx = 0;
}
#else
#define store_prepwst()
#define create_pmproc_entry()
int pm_cpuidle_prepwst_show(struct seq_file *m, void *v) { return 0; }
#endif /*#ifdef CONFIG_PROC_FS */

/******************************************************************************/

static int uart_awake = 0;
static unsigned long   uart_last_awake = 0;
static typeof(jiffies) uart_inactivity_timeout = 0;

static int pre_uart_activity(void)
{
	int  dev = 0, d, rc;
	unsigned long last_activity;

	if( uart_awake ) {
		uart_last_awake = jiffies;
		uart_awake = 0;
	}

	rc = are_driver8250_uarts_active (&dev, &d, &d, &d, &d, &last_activity);
	if ( rc ) {
		return rc; // right buzy
	}

	/* not busy - check time */
	if( time_before(jiffies, last_activity   + uart_inactivity_timeout)||
	    time_before(jiffies, uart_last_awake + uart_inactivity_timeout)) {
		return 1; // waiting for timeout
	}

	return 0;
}

static void post_uart_activity(void)
{
	u32 wkup_per = PM_WKST_PER;

	if( wkup_per & (1 << 11)) { // UART 3 is a debug uart
		uart_awake = 1;
	}
}

/******************************************************************************
 *
 * CPUIDLE callbacks
 *
 ******************************************************************************/

#ifdef DEBUG_STATES
static int bail_count = 0;
#define BAIL_ON(_cond) \
	do { \
		if (_cond) { \
			if (30 == bail_count++) { \
				bail_count = 0; \
				printk("##### BAIL %s (%08x)\n", #_cond, _cond); \
			} \
			clock_bail_trace(#_cond, _cond, __LINE__); \
			return 1; \
		} \
	} while (0)
#else
#define BAIL_ON(_cond) \
	do { \
		if (_cond) { \
			clock_bail_trace(#_cond, _cond, __LINE__); \
			return 1; \
		} \
       } while (0)
#endif

static int omap3_idle_bm_check(void)
{
	u8 state;

	/* Check if any modules other than debug uart and gpios are active*/
	BAIL_ON(CM_FCLKEN1_CORE   & CORE_FCLK_MASK);
	BAIL_ON(CM_FCLKEN_SGX     & SGX_FCLK_MASK);
	BAIL_ON(CM_FCLKEN_CAM     & CAM_FCLK_MASK);
	BAIL_ON(CM_FCLKEN_PER     & PER_FCLK_MASK);
	BAIL_ON(CM_FCLKEN_USBHOST & USBHOST_FCLK_MASK);
	BAIL_ON(CM_FCLKEN3_CORE   & CORE3_FCLK_MASK);

	/* To allow core retention during LPR scenario */
	BAIL_ON(!omap2_disp_lpr_is_enabled() && (CM_FCLKEN_DSS & DSS_FCLK_MASK));

	/* Work around an issue that popped up in ES3.1. For some reason the
	 * ICLKEN1_CORE bit 3 is set and AUTOIDLE1_CORE bit 3 is not. This is
	 * preventing the CORE from going to RET. If we detect this condition
	 * here we fix it...
	 */
	if (CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE & (1<<3)) {
		CM_AUTOIDLE1_CORE |= (1<<3);
	}

	/* Check if any modules have ICLK bit enabled and interface clock
	 * autoidle disabled
	 */
	BAIL_ON(CORE1_ICLK_VALID & (CM_ICLKEN1_CORE & ~CM_AUTOIDLE1_CORE));

	/* Check for secure modules which have only ICLK. Do not check for rng
	 * module. It has been ensured that if rng is active cpu idle will
	 * never be entered.
	 */
	BAIL_ON(CORE2_ICLK_VALID & CM_ICLKEN2_CORE & ~4);

	/* Enabling SGX ICLK will prevent CORE ret*/
	BAIL_ON(SGX_ICLK_VALID & (CM_ICLKEN_SGX));
	BAIL_ON(CORE3_ICLK_VALID   & (CM_ICLKEN3_CORE   & ~CM_AUTOIDLE3_CORE));
	BAIL_ON(USBHOST_ICLK_VALID & (CM_ICLKEN_USBHOST & ~CM_AUTOIDLE_USBHOST));
	BAIL_ON(DSS_ICLK_VALID     & (CM_ICLKEN_DSS     & ~CM_AUTOIDLE_DSS));
	BAIL_ON(CAM_ICLK_VALID     & (CM_ICLKEN_CAM     & ~CM_AUTOIDLE_CAM));
	BAIL_ON(PER_ICLK_VALID     & (CM_ICLKEN_PER     & ~CM_AUTOIDLE_PER));
	BAIL_ON(WKUP_ICLK_VALID    & (CM_ICLKEN_WKUP    & ~CM_AUTOIDLE_WKUP));

	/* Check if IVA power domain is ON */
	prcm_get_power_domain_state(DOM_IVA2, &state);
	BAIL_ON(state == PRCM_ON);

	/* Check if a DMA transfer is active */
	BAIL_ON(omap_dma_running());

	/* Check if debug UART is active */
	BAIL_ON(pre_uart_activity());

	clock_bail_trace("--- C5 ---", 0, __LINE__);
	return 0;
}

/* Correct target state based on inactivity timer expiry, etc */
static void adjust_target_states(struct system_power_state *target_state)
{
	switch (target_state->mpu_state) {
	case PRCM_MPU_ACTIVE:
	case PRCM_MPU_INACTIVE:
		target_state->neon_state = PRCM_ON;
		break;
	case PRCM_MPU_CSWR_L2RET:
	case PRCM_MPU_OSWR_L2RET:
	case PRCM_MPU_CSWR_L2OFF:
	case PRCM_MPU_OSWR_L2OFF:
		target_state->neon_state = PRCM_RET;
		break;
	case PRCM_MPU_OFF:
		target_state->neon_state = PRCM_OFF;
		if (!enable_off) {
			target_state->mpu_state  = PRCM_MPU_CSWR_L2RET;
			target_state->neon_state = PRCM_RET;
		}
		break;
	}


	if (target_state->core_state > PRCM_CORE_INACTIVE) {
#ifdef CONFIG_OMAP34XX_OFFMODE
#if 0
/* GPT context save in PER domain not yet implemented. Until it is, we do not
 * put PER to OFF.
 */
		/* Core can be put to RET/OFF - This means PER can be put to
		 * off if its inactivity timer has expired.
		 */
		if (perdomain_timer_pending())
			target_state->per_state = PRCM_RET;
		else
			target_state->per_state = PRCM_OFF;
#else
		target_state->per_state = PRCM_RET;
#endif

		if (target_state->core_state == PRCM_CORE_OFF) {
			if (coredomain_timer_pending())
				target_state->core_state = PRCM_CORE_CSWR_MEMRET;
			if (CM_FCLKEN_DSS & DSS_FCLK_MASK)
				target_state->core_state = PRCM_CORE_CSWR_MEMRET;
			if (!enable_off) {
				target_state->core_state = PRCM_CORE_CSWR_MEMRET;
				target_state->per_state  = PRCM_RET;
			}
		}
#else
		target_state->per_state = PRCM_RET;
#endif
	} else
		target_state->per_state = PRCM_ON;

	if (target_state->core_state == PRCM_CORE_CSWR_MEMRET)
		memory_logic_res_setting(target_state);
}

/******************************************************************************/

static void omap3_idle_setup_wkup_sources (void)
{
	/* Configure wakeup sources */
	PM_WKEN1_CORE   = 0
	                | (0x03 << 24)  // MMC  1-2
	                | (0x0F << 18)  // SPI  1-4
	                | (0x07 << 15)  // I2C  1-3
	                | (0x03 << 13)  // UART 1-2
	                | (0x03 << 11)  // GPT  10,11
	                | (0x01 << 10)  // MCBSP5  
	                | (0x01 <<  9)  // MCBSP1  
	                | (0x01 <<  4)  // USB OTG
	                | (0x08 <<  0)  // MUST write
	                ;
	                
	PM_WKEN3_CORE	= 0
	                | (0x01 <<  2)  // USB TLL
	                ;

	PM_WKEN_WKUP	= 0
	                | (0x03 <<  6) // Smart Reflex
	                | (0x01 <<  3) // GPIO1
	                | (0x01 <<  0) // GPT1
	                ;
	                
	PM_WKEN_PER	= 0
	                | (0x1F << 13) // GPIO  2-6
	                | (0x01 << 11) // UART  3
	                | (0xFF <<  3) // GPT   2-9
	                | (0x07 <<  0) // MCBSP 2-4
	                ;
	
	PM_WKEN_USBHOST = (0x01 << 0); // USB HOST


	/* */
	PM_MPUGRPSEL1_CORE = 0
	                | (0x03 << 24)  // MMC  1-2
	                | (0x0F << 18)  // SPI  1-4
	                | (0x07 << 15)  // I2C  1-3
	                | (0x03 << 13)  // UART 1-2
	                | (0x03 << 11)  // GPT  10,11
	                | (0x01 << 10)  // MCBSP5  
	                | (0x01 <<  9)  // MCBSP1  
	                | (0x01 <<  4)  // USB OTG
	                | (0x08 <<  0)  // MUST write
	                ;
	                   
	PM_MPUGRPSEL3_CORE = (1 <<  2); // USB TLL

	PM_MPUGRPSEL_PER   = 0
	                   | (0x1F << 13) // GPIO  2-6
	                   | (0x1  << 11) // UART  3
	                   | (0xFF <<  3) // GPT   2-9
	                   | (0x7  <<  0) // McBSP 2-4
	                   ;
	
	PM_MPUGRPSEL_WKUP  |= (1 << 0)    // GPT  1
	                   |  (1 << 3)    // GPIO 1
	                   ;

	PRM_IRQENABLE_MPU  |= 0x1;

//      PM_MPUGRPSEL_USBHOST

}

static int omap3_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_state *state)
{
	struct omap3_processor_cx *cx;
	struct timespec ts_preidle;
	struct timespec ts_postidle;
	struct timespec ts_idle;

	/* Used for LPR mode DSS context save/restore. */
	u32 pm_wken_dss      = 0;
	u32 pm_pwstctrl_dss  = 0;
	u32 cm_clkstctrl_dss = 0;
	u32 cm_fclken_dss    = 0;
	u32 cm_iclken_dss    = 0;
	u32 cm_autoidle_dss  = 0;

	u32 fclken_core;
	u32 iclken_core;
	u32 fclken_per;
	u32 iclken_per;

	int wakeup_latency;

	struct system_power_state target_state;
	struct system_power_state cur_state;

#ifdef CONFIG_HW_SUP_TRANS
	u32 sleepdep_per;
	u32 wakedep_per;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	u32 sdrc_power_register = 0;
	int core_sleep_flg = 0;
	int got_console_lock = 0;

	/* Disable interrupts.
	 */
	local_irq_disable();
	local_fiq_disable();

	/* If need resched - return immediately
	 */ 
	if( need_resched()) {
		local_fiq_enable();
		local_irq_enable();
		return 0;
	}

	/* Reset previous power state registers.
	 */
	clear_prepwstst();

	omap3_idle_setup_wkup_sources ();
	
	/* Set up target state from state context provided by cpuidle.
	 */
	cx = cpuidle_get_statedata(state);
	target_state.mpu_state  = cx->mpu_state;
	target_state.core_state = cx->core_state;
	target_state.neon_state = 0; /* Avoid gcc warning. Will be set in
					adjust_target_states(). */

	/* take a time marker for residency.
	 */
	getnstimeofday(&ts_preidle);

	/* If the requested state is C0, we bail here...
	 */
	if (cx->type == OMAP3_STATE_C1) {
		omap_sram_idle(target_state.mpu_state);
		goto return_sleep_time;
	}

	if (cx->type > OMAP3_STATE_C2)
		sched_clock_idle_sleep_event(); /* about to enter deep idle */

	/* Adjust PER and NEON domain target states as well as CORE domain
	 * target state depending on MPU/CORE setting, enable_off sysfs entry
	 * and PER timer status.
	 */
	adjust_target_states(&target_state);

	wakeup_latency = cx->wakeup_latency;
	/* NOTE:
	 *   We will never get the condition below as we are not supporting
	 *   CORE OFF right now. Keeping this code around for future reference.
	 */
	if (target_state.core_state != cx->core_state) {
		/* Currently, this can happen only for core_off. Adjust wakeup
		 * latency to that of core_cswr state. Hard coded now and needs
		 * to be made more generic omap3_power_states[4] is CSWR for
		 * core */
		wakeup_latency = omap3_power_states[4].wakeup_latency;
	}

	/* Reprogram next wake up tick to adjust for wake latency */
	if (wakeup_latency > 1000) {
		struct tick_device *d = tick_get_device(smp_processor_id());
		ktime_t now = ktime_get();

		if (ktime_to_ns(ktime_sub(d->evtdev->next_event, now)) >
					(wakeup_latency * 1000 + NSEC_PER_MSEC)) {
			ktime_t adjust = ktime_set(0, (wakeup_latency * 1000));
			ktime_t next   = ktime_sub(d->evtdev->next_event, adjust);
			clockevents_program_event(d->evtdev, next, now);
		}
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto return_sleep_time;


	/* Remember the current power states and clock settings.
	 */
	prcm_get_power_domain_state(DOM_PER,  &cur_state.per_state);
	prcm_get_power_domain_state(DOM_CAM,  &cur_state.cam_state);
	prcm_get_power_domain_state(DOM_SGX,  &cur_state.sgx_state);
	prcm_get_power_domain_state(DOM_NEON, &cur_state.neon_state);

	fclken_core = CM_FCLKEN1_CORE;
	iclken_core = CM_ICLKEN1_CORE;
	fclken_per  = CM_FCLKEN_PER;
	iclken_per  = CM_ICLKEN_PER;

#ifdef CONFIG_HW_SUP_TRANS
	/* Facilitating SWSUP RET, from HWSUP mode */
	sleepdep_per = CM_SLEEPDEP_PER;
	wakedep_per  = PM_WKDEP_PER;
#endif /* #ifdef CONFIG_HW_SUP_TRANS */

	/* If target state if core_off, save registers before changing
	 * anything.
	 */
	if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
		prcm_save_registers();
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto return_sleep_time;


	/* Program MPU and NEON to target state */
	if (target_state.mpu_state > PRCM_MPU_ACTIVE) {
		if ((cur_state.neon_state == PRCM_ON) &&
		    (target_state.neon_state != PRCM_ON)) {

			if (target_state.neon_state == PRCM_OFF)
				omap3_save_neon_context();

			prcm_transition_domain_to(DOM_NEON, target_state.neon_state);
		}
#ifdef _disabled_CONFIG_MPU_OFF
		/* Populate scratchpad restore address */
		scratchpad_set_restore_addr();
#endif
		/* TODO No support for OFF Mode yet
		if(target_state.core_state > PRCM_CORE_CSWR_MEMRET)
			omap3_save_secure_ram_context(target_state.core_state);
		*/
		prcm_set_mpu_domain_state(target_state.mpu_state);
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto restore;


	/* Program CORE and PER to target state */
	if (target_state.core_state > PRCM_CORE_ACTIVE) {
		/* Log core sleep attempt */
		core_sleep_flg = 1;

		/* Lock the console to prevent potential access to UARTs.
		 */
		if (0 == try_acquire_console_sem()) {
			got_console_lock = 1;
		}

		/* Handle PER, CAM and SGX domains. */
		if ((cur_state.per_state == PRCM_ON) &&
		    (target_state.per_state != PRCM_ON)) {

			if (target_state.per_state == PRCM_OFF) {
				omap3_save_per_context();
			}

			prcm_transition_domain_to(DOM_PER, target_state.per_state);
		}
		if (PRCM_ON == cur_state.cam_state)
			prcm_transition_domain_to(DOM_CAM, PRCM_RET);
		if (PRCM_ON == cur_state.sgx_state)
			prcm_transition_domain_to(DOM_SGX, PRCM_RET);

		disable_smartreflex(SR1_ID);
		disable_smartreflex(SR2_ID);

		prcm_set_core_domain_state(target_state.core_state);

		/* Enable Autoidle for GPT1 explicitly - Errata 1.4 */
		CM_AUTOIDLE_WKUP |= 0x1;

		/* Disable HSUSB OTG ICLK explicitly*/
		CM_ICLKEN1_CORE &= ~0x10;

		/* Enabling GPT1 wake-up capabilities */
		PM_WKEN_WKUP |= 0x1;

		/* Errata 2.15
		 * Configure UARTs to ForceIdle. Otherwise they can prevent
		 * CORE RET.
		 */
		omap24xx_uart_set_force_idle();

		CM_ICLKEN1_CORE &= ~(1<<7); /* MAILBOXES */
		CM_ICLKEN1_CORE &= ~(1<<6); /* OMAPCTRL */

		/* If we are in LPR mode we need to set up DSS accordingly.
		 */
		if (omap2_disp_lpr_is_enabled()) {
			pm_wken_dss      = PM_WKEN_DSS;
			pm_pwstctrl_dss  = PM_PWSTCTRL_DSS;
			cm_clkstctrl_dss = CM_CLKSTCTRL_DSS;
			cm_fclken_dss    = CM_FCLKEN_DSS;
			cm_iclken_dss    = CM_ICLKEN_DSS;
			cm_autoidle_dss  = CM_AUTOIDLE_DSS;
			PM_WKEN_DSS      = 0x00000001;
			PM_PWSTCTRL_DSS  = 0x00030107;
			CM_CLKSTCTRL_DSS = 0x00000003;
			CM_FCLKEN_DSS    = 0x00000001;
			CM_ICLKEN_DSS    = 0x00000001;
			CM_AUTOIDLE_DSS  = 0x00000001;
		}
	}

	/* Check for pending interrupts. If there is an interrupt, return */
	if (INTCPS_PENDING_IRQ0 | INTCPS_PENDING_IRQ1 | INTCPS_PENDING_IRQ2)
		goto restore;

#ifdef CONFIG_DISABLE_HFCLK
	PRM_CLKSRC_CTRL |= 0x18; /* set sysclk to stop */
#endif /* #ifdef CONFIG_DISABLE_HFCLK */


	DEBUG_STATE_CAPTURE();


	/* Errata 1.142:
	 * SDRC not sending auto-refresh when OMAP wakes-up from OFF mode
	 */
	if (!is_device_type_gp() && is_sil_rev_equal_to(OMAP3430_REV_ES3_0)) {
		sdrc_power_register = SDRC_POWER_REG;
		SDRC_POWER_REG &= ~(SDRC_PWR_AUTOCOUNT_MASK | SDRC_PWR_CLKCTRL_MASK);
		SDRC_POWER_REG |= 0x120;
		if (target_state.core_state == PRCM_CORE_OFF)
			save_scratchpad_contents();
	}

	omap_sram_idle(target_state.mpu_state);

	/* Errata 1.142:
	 * SDRC not sending auto-refresh when OMAP wakes-up from OFF mode
	 */
	if (!is_device_type_gp() && is_sil_rev_equal_to(OMAP3430_REV_ES3_0))
		SDRC_POWER_REG = sdrc_power_register;

restore:
#ifdef CONFIG_DISABLE_HFCLK
	PRM_CLKSRC_CTRL &= ~0x18;
#endif /* #ifdef CONFIG_DISABLE_HFCLK */
	/* Disabling IO_PAD capabilities */
	PM_WKEN_WKUP &= ~(0x100);

	CM_FCLKEN1_CORE = fclken_core;
	CM_ICLKEN1_CORE = iclken_core;

	if (target_state.mpu_state > PRCM_MPU_ACTIVE) {
#ifdef _disabled_CONFIG_MPU_OFF
		/* On ES 2.0, if scratchpad is populated with valid pointer,
		 * warm reset does not work So populate scratchpad restore
		 * address only in cpuidle and suspend calls
		*/
		scratchpad_clr_restore_addr();
#endif
		prcm_set_mpu_domain_state(PRCM_MPU_ACTIVE);

		if ((cur_state.neon_state == PRCM_ON) &&
		    (target_state.mpu_state > PRCM_MPU_INACTIVE)) {
			u8 pre_state;

			prcm_force_power_domain_state(DOM_NEON, cur_state.neon_state);
			prcm_get_pre_power_domain_state(DOM_NEON, &pre_state);

			if (pre_state == PRCM_OFF) {
				omap3_restore_neon_context();
			}

#ifdef CONFIG_HW_SUP_TRANS
			prcm_set_power_domain_state(DOM_NEON, POWER_DOMAIN_ON,
								PRCM_AUTO);
#endif
		}
	}

	/* Continue core restoration part, only if Core-Sleep is attempted */
	if ((target_state.core_state > PRCM_CORE_ACTIVE) && core_sleep_flg) {
		u8 pre_per_state;

		prcm_set_core_domain_state(PRCM_CORE_ACTIVE);

		omap24xx_uart_clr_force_idle();

		enable_smartreflex(SR1_ID);
		enable_smartreflex(SR2_ID);

		/* Turn PER back ON if it was ON before idle.
		 */
		if (cur_state.per_state == PRCM_ON) {
			prcm_force_power_domain_state(DOM_PER, cur_state.per_state);

			CM_ICLKEN_PER = iclken_per;
			CM_FCLKEN_PER = fclken_per;

			prcm_get_pre_power_domain_state(DOM_PER, &pre_per_state);
			if (pre_per_state == PRCM_OFF) {
				omap3_restore_per_context();
#ifdef CONFIG_OMAP34XX_OFFMODE
				context_restore_update(DOM_PER);
#endif
			}

#ifdef CONFIG_HW_SUP_TRANS
			/* Facilitating SWSUP RET, from HWSUP mode */
			CM_SLEEPDEP_PER = sleepdep_per;
			PM_WKDEP_PER    = wakedep_per;
			prcm_set_power_domain_state(DOM_PER, PRCM_ON, PRCM_AUTO);
#endif
		}

		/* Restore CAM and SGX. */
		if (PRCM_ON == cur_state.cam_state)
			prcm_transition_domain_to(DOM_CAM, PRCM_ON);
		if (PRCM_ON == cur_state.sgx_state)
			prcm_transition_domain_to(DOM_SGX, PRCM_ON);

		/* If we lost CORE context, restore it.
		 */
		if (target_state.core_state >= PRCM_CORE_OSWR_MEMRET) {
#ifdef CONFIG_OMAP34XX_OFFMODE
			context_restore_update(DOM_CORE1);
#endif
			prcm_restore_registers();
			prcm_restore_core_context(target_state.core_state);
#ifdef CONFIG_CORE_OFF
			omap3_restore_core_settings();
#endif
		}

		/* Restore DSS settings */
		if (omap2_disp_lpr_is_enabled()) {
			PM_WKEN_DSS      = pm_wken_dss;
			PM_PWSTCTRL_DSS  = pm_pwstctrl_dss;
			CM_CLKSTCTRL_DSS = cm_clkstctrl_dss;
			CM_FCLKEN_DSS    = cm_fclken_dss;
			CM_ICLKEN_DSS    = cm_iclken_dss;
			CM_AUTOIDLE_DSS  = cm_autoidle_dss;
		}

		/* At this point CORE and PER domain are back. We can release
		 * the console if we have it.
		 */
		if (got_console_lock) {
			release_console_sem();
		}

#ifdef CONFIG_OMAP_32K_TIMER
		/* Errata 1.4
		 * If a General Purpose Timer (GPTimer) is in posted mode
		 * (TSIRC.POSTED=1), due to internal resynchronizations, values
		 * read in TCRR, TCAR1 and TCAR2 registers right after the
		 * timer interface clock (L4) goes from stopped to active may
		 * not return the expected values. The most common event
		 * leading to this situation occurs upon wake up from idle.
		 *
		 * Software has to wait at least (2 timer interface clock
		 * cycles + 1 timer functional clock cycle) after L4 clock
		 * wakeup before reading TCRR, TCAR1 or TCAR2 registers for
		 * GPTimers in POSTED internal synchro- nization mode, and
		 * before reading WCRR register of the Watchdog timers . The
		 * same workaround must be applied before reading CR and
		 * 32KSYNCNT_REV registers of the synctimer module.
		 *
		 * Wait Period = 2 timer interface clock cycles +
		 *               1 timer functional clock cycle
		 * Interface clock  = L4 clock (50MHz worst case).
		 * Functional clock = 32KHz
		 * Wait Period = 2*10^-6/50 + 1/32768 = 0.000030557 = 30.557us
		 * Rounding off the delay value to a safer 50us.
		 */
		udelay(GPTIMER_WAIT_DELAY);
#endif

		/* Disable autoidling of GPT1.
		 */
		CM_AUTOIDLE_WKUP &= ~(0x1);
	}

	DPRINTK("MPU state:%x, CORE state:%x\n",
			PM_PREPWSTST_MPU, PM_PREPWSTST_CORE);

	/* Do wakeup event check s*/
	post_uart_activity();

	/* Update stats for sysfs entries.
	 */
	store_prepwst();

return_sleep_time:
	getnstimeofday(&ts_postidle);
#if defined(CONFIG_SYSFS) && defined(DEBUG_BAIL_STATS)
	ts_last_wake_up = ts_postidle;
#endif
	ts_idle = timespec_sub(ts_postidle, ts_preidle);

	if (cx->type > OMAP3_STATE_C2)
		sched_clock_idle_wakeup_event(timespec_to_ns(&ts_idle));


	DEBUG_STATE_PRINT(core_sleep_flg);


	local_irq_enable();
	local_fiq_enable();

	return (u32)timespec_to_ns(&ts_idle)/1000;
}


static int omap3_enter_idle_bm(struct cpuidle_device *dev,
				struct cpuidle_state *state)
{
	if (omap3_idle_bm_check()) {
		/* Someone's busy, pick a safe idle state. */
		if (dev->safe_state) {
			return dev->safe_state->enter(dev, dev->safe_state);
		}
		else {
			struct omap3_processor_cx *cx;

			cx = cpuidle_get_statedata(state);
			omap_sram_idle(cx->mpu_state);
			return 0;
		}
	}
	return omap3_enter_idle(dev, state);
}

/******************************************************************************
 *
 * CPUIDLE INIT
 *
 ******************************************************************************/

DEFINE_PER_CPU(struct cpuidle_device, omap3_idle_dev);

static struct cpuidle_driver omap3_idle_driver = {
	.name = 	"omap3_idle",
	.owner = 	THIS_MODULE,
};

static __init int omap3_idle_init(void)
{
	int i, count = 0;
	struct omap3_processor_cx *cx;
	struct cpuidle_state      *state;
	struct cpuidle_device     *dev;

	printk(KERN_INFO "OMAP CPU idle driver initializing.\n");

	cpuidle_register_driver(&omap3_idle_driver);

	dev = &per_cpu(omap3_idle_dev, smp_processor_id());

	for (i = 0; i < OMAP3_MAX_STATES; i++) {
		cx = &omap3_power_states[i];
		state = &dev->states[count];

		if (!cx->valid)
			continue;

		cpuidle_set_statedata(state, cx);
		state->exit_latency     = cx->sleep_latency + cx->wakeup_latency;
		state->target_residency = cx->threshold;
		state->flags            = cx->flags;
		state->enter            = (state->flags & CPUIDLE_FLAG_CHECK_BM) ?
					     omap3_enter_idle_bm : omap3_enter_idle;
		if (cx->type == OMAP3_STATE_C3)
			dev->safe_state = state;
		snprintf(state->name, CPUIDLE_NAME_LEN, "C%d", count+1);

		count++;
		BUG_ON(count == CPUIDLE_STATE_MAX); /* harsh... oh well */
	}

	if (!count)
		return -EINVAL; /* No valid states configured. */

	dev->state_count = count;

	if (cpuidle_register_device(dev)) {
		printk(KERN_ERR "%s: CPUidle register device failed\n",
			__FUNCTION__);
		return -EIO;
	}

	create_pmproc_entry();

	/* Initialize UART inactivity time */
	uart_inactivity_timeout = msecs_to_jiffies(UART_TIME_OUT);
	uart_last_awake  = jiffies;

	return 0;
}
__initcall(omap3_idle_init);
