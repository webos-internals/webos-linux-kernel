/*
 * linux/arch/arm/mach-omap3pe/prcm_pwr.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on OMAP 34xx Power Reset and Clock Management (PRCM) functions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Karthik Dasu/Rajendra Nayak/Pavan Chinnabhandar
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <asm/arch/pm.h>
#include <asm/arch/prcm.h>
#include <linux/delay.h>
#include <asm/io.h>

#include <asm/arch/power_companion.h>

#include "prcm-regs.h"
#include "prcm_mpu_core_ctl.h"
#include "prcm_slpwk.h"
#include "prcm_util.h"
#include "scratchpad.h"

#include "prcm_pwr.h"

extern void omap3_show_iopad_wkup_event(void);
#ifdef CONFIG_FASTPATH
extern void omap3_wakeup_sources_save(void);
#endif

/******************************************************************************
 *
 * LOGs
 *
 ******************************************************************************/

// logging levels
#define LOG_LVL_NONE                    0 
#define LOG_LVL_SUSPEND_EVENT           1 
#define LOG_LVL_WAKEUP_REASON           2 
#define LOG_LVL_WAKEUP_PIN_STATE        3 
#define LOG_LVL_SUSPEND_DOMAIN_STATE    4
#define LOG_LVL_SUSPEND_CLOCKS          5 
#define LOG_LVL_ALL                    99 

#define MAX_WKUP_LOGS 4

static struct wkup_log_t {
	// timestamps
	u32   ts_suspend;
	u32   ts_resume;

	//  SDRC 
	u32   sdrc_pwr;
	u32   sdrc_syscfg;
	u32   sdrc_refr_ctrl0;
	u32   sdrc_refr_ctrl1;

	//  PRM
	u32   prm_mpu_irq_en;
	u32   prm_mpu_irq_st;

	// previous domain power state 
	u32   prepwst_MPU;
	u32   prepwst_CORE;
	u32   prepwst_IVA2;
	u32   prepwst_PER;
	u32   prepwst_DSS;
	u32   prepwst_CAM;
	u32   prepwst_SGX;
	u32   prepwst_NEON;
	u32   prepwst_USBHOST;

	// clocks
	u32   ckgen;
	u32   idlest1_core;
	u32   idlest2_core;
	u32   idlest3_core;
	u32   iclken1_core;
	u32   iclken2_core;
	u32   iclken3_core;
	u32   fclken1_core;
	u32   fclken3_core;
	u32   autoidle1_core;
	u32   autoidle2_core;
	u32   autoidle3_core;
	
	// domain wakeup state
	u32   wkst_CORE1;
	u32   wkst_CORE3;
	u32   wkst_WKUP;
	u32   wkst_PER;
	u32   wkst_USBHOST;

	// gpio interrupt state
	u32   gpio1_irq_st;
	u32   gpio2_irq_st;
	u32   gpio3_irq_st;
	u32   gpio4_irq_st;
	u32   gpio5_irq_st;
	u32   gpio6_irq_st;

	// gpio data in
	u32   gpio1_data_in;
	u32   gpio2_data_in;
	u32   gpio3_data_in;
	u32   gpio4_data_in;
	u32   gpio5_data_in;
	u32   gpio6_data_in;
}
wkup_log[MAX_WKUP_LOGS];
static int wkup_log_lvl =  LOG_LVL_WAKEUP_REASON;
static int wkup_log_idx = -1;
static int wkup_log_cnt =  0;

inline static struct wkup_log_t *
wkup_log_get_current(void)
{
	return wkup_log + wkup_log_idx;
}

static void
wkup_log_get_next_log(void)
{
	wkup_log_idx++;
	if( wkup_log_idx >= MAX_WKUP_LOGS )
		wkup_log_idx = 0;

	wkup_log_cnt++;
	if( wkup_log_cnt > MAX_WKUP_LOGS) 
		wkup_log_cnt = MAX_WKUP_LOGS;

	memset( wkup_log + wkup_log_idx, 0, sizeof(struct wkup_log_t));

	wkup_log[wkup_log_idx].ts_suspend = omap_32k_sync_timer_read();
}

static void 
wkup_log_wakeup (void)
{
	struct wkup_log_t *log = wkup_log_get_current();

	/* Enable clock for dumping registers. */
	CM_ICLKEN1_CORE |= 0x40;
	WAIT_WHILE(CM_IDLEST1_CORE & 0x40, 1000);

	// domain state
	log->prepwst_MPU     = PM_PREPWSTST_MPU;
	log->prepwst_CORE    = PM_PREPWSTST_CORE;
	log->prepwst_IVA2    = PM_PREPWSTST_IVA2;
	log->prepwst_PER     = PM_PREPWSTST_PER;
	log->prepwst_DSS     = PM_PREPWSTST_DSS;
	log->prepwst_CAM     = PM_PREPWSTST_CAM;
	log->prepwst_SGX     = PM_PREPWSTST_SGX;
	log->prepwst_NEON    = PM_PREPWSTST_NEON;
	log->prepwst_USBHOST = PM_PREPWSTST_USBHOST;

	// wkup source
	log->wkst_CORE1      = PM_WKST1_CORE;
	log->wkst_CORE3      = PM_WKST3_CORE;
	log->wkst_WKUP       = PM_WKST_WKUP;
	log->wkst_PER        = PM_WKST_PER;
	log->wkst_USBHOST    = PM_WKST_USBHOST;

	// others
	log->sdrc_pwr        = SDRC_POWER_REG;
	log->sdrc_syscfg     = SDRC_SYSCONFIG;
	log->sdrc_refr_ctrl0 = SDRC_RFR_CTRL_0;
	log->sdrc_refr_ctrl1 = SDRC_RFR_CTRL_1;
	log->prm_mpu_irq_en  = PRM_IRQENABLE_MPU;
	log->prm_mpu_irq_st  = PRM_IRQSTATUS_MPU;

	/* Disable clock after dumping registers. */
	CM_ICLKEN1_CORE &= ~0x0040;
	WAIT_UNTIL(CM_IDLEST1_CORE & 0x40, 1000);
}

static void 
wkup_log_wkup_gpio (void)
{
	struct wkup_log_t *log = wkup_log_get_current();

	log->ts_resume      = omap_32k_sync_timer_read();

	log->gpio1_irq_st   = GPIOn_IRQSTATUS1(1);
	log->gpio2_irq_st   = GPIOn_IRQSTATUS1(2);
	log->gpio3_irq_st   = GPIOn_IRQSTATUS1(3);
	log->gpio4_irq_st   = GPIOn_IRQSTATUS1(4);
	log->gpio5_irq_st   = GPIOn_IRQSTATUS1(5);
	log->gpio6_irq_st   = GPIOn_IRQSTATUS1(6);

	log->gpio1_data_in  = GPIOn_DATAIN(1);
	log->gpio2_data_in  = GPIOn_DATAIN(2);
	log->gpio3_data_in  = GPIOn_DATAIN(3);
	log->gpio4_data_in  = GPIOn_DATAIN(4);
	log->gpio5_data_in  = GPIOn_DATAIN(5);
	log->gpio6_data_in  = GPIOn_DATAIN(6);
}

static void 
wkup_log_suspend_clocks(void)
{
	struct wkup_log_t *log = wkup_log_get_current();
	
	log->ckgen          = CM_IDLEST_CKGEN;
	log->idlest1_core   = CM_IDLEST1_CORE;
	log->idlest2_core   = CM_IDLEST2_CORE;
	log->idlest3_core   = CM_IDLEST3_CORE;
	log->iclken1_core   = CM_ICLKEN1_CORE;
	log->iclken2_core   = CM_ICLKEN2_CORE;
	log->iclken3_core   = CM_ICLKEN3_CORE;
	log->fclken1_core   = CM_FCLKEN1_CORE;
	log->fclken3_core   = CM_FCLKEN3_CORE;
	log->autoidle1_core = CM_AUTOIDLE1_CORE;
	log->autoidle2_core = CM_AUTOIDLE2_CORE;
	log->autoidle3_core = CM_AUTOIDLE3_CORE;
}

static void 
wkup_log_printk_suspend_event(int idx, int force)
{
	struct wkup_log_t *log = wkup_log + idx;
	int    log_lvl = force ?  LOG_LVL_ALL : wkup_log_lvl;

	if( log_lvl >= LOG_LVL_SUSPEND_EVENT ) {
		u32 msec_suspend = (u32)((u64) log->ts_suspend * 1000 / 32768);
		u32 msec_resume  = (u32)((u64) log->ts_resume  * 1000 / 32768);
		
		printk(KERN_INFO "%08d.%03u: enter suspend\n", 
		       (int)(msec_suspend / 1000), msec_suspend % 1000);
		printk(KERN_INFO "%08d.%03u: leave suspend\n",
		       (int)(msec_resume  / 1000),  msec_resume  % 1000);
	}

	if( log_lvl >= LOG_LVL_SUSPEND_CLOCKS )  {
		printk(KERN_INFO "CLOCKS BEFORE SLEEP\n");
		printk(KERN_INFO "idlest_ckgen   was %08x\n", log->ckgen);
		printk(KERN_INFO "idlest1_core   was %08x\n", log->idlest1_core);
		printk(KERN_INFO "idlest2_core   was %08x\n", log->idlest2_core);
		printk(KERN_INFO "idlest3_core   was %08x\n", log->idlest3_core);
		printk(KERN_INFO "iclken1_core   was %08x\n", log->iclken1_core);
		printk(KERN_INFO "iclken2_core   was %08x\n", log->iclken2_core);
		printk(KERN_INFO "iclken3_core   was %08x\n", log->iclken3_core);
		printk(KERN_INFO "fclken1_core   was %08x\n", log->fclken1_core);
		printk(KERN_INFO "fclken3_core   was %08x\n", log->fclken3_core);
		printk(KERN_INFO "autoidle1_core was %08x\n", log->autoidle1_core);
		printk(KERN_INFO "autoidle2_core was %08x\n", log->autoidle2_core);
		printk(KERN_INFO "autoidle3_core was %08x\n", log->autoidle3_core);
	}

	if( log_lvl >= LOG_LVL_SUSPEND_DOMAIN_STATE ) {
		printk(KERN_INFO "PREV DOMAIN STATE\n");
		printk(KERN_INFO "PM_PREPWSTST_MPU  %08x\n",   log->prepwst_MPU);
		printk(KERN_INFO "PM_PREPWSTST_CORE %08x\n",   log->prepwst_CORE);
		printk(KERN_INFO "PM_PREPWSTST_IVA2 %08x\n",   log->prepwst_IVA2);
		printk(KERN_INFO "PM_PREPWSTST_PER  %08x\n",   log->prepwst_PER);
		printk(KERN_INFO "PM_PREPWSTST_DSS  %08x\n",   log->prepwst_DSS);
		printk(KERN_INFO "PM_PREPWSTST_CAM  %08x\n",   log->prepwst_CAM);
		printk(KERN_INFO "PM_PREPWSTST_SGX  %08x\n",   log->prepwst_SGX);
		printk(KERN_INFO "PM_PREPWSTST_NEON %08x\n",   log->prepwst_NEON);
		printk(KERN_INFO "PM_PREPWSTST_USBHOST %08x\n",log->prepwst_USBHOST);
	}

	if( log_lvl >= LOG_LVL_WAKEUP_REASON ) {
		printk(KERN_INFO "WAKEUP STATE/REASON\n");

		if( log->wkst_CORE1 )
			printk(KERN_INFO "PM_WKST1_CORE     %08x\n",   log->wkst_CORE1);
		if( log->wkst_CORE3 )
			printk(KERN_INFO "PM_WKST3_CORE     %08x\n",   log->wkst_CORE3);
		if( log->wkst_WKUP )
			printk(KERN_INFO "PM_WKST_WKUP      %08x\n",   log->wkst_WKUP);
		if( log->wkst_PER  )
			printk(KERN_INFO "PM_WKST_PER       %08x\n",   log->wkst_PER);
		if( log->wkst_PER  )
			printk(KERN_INFO "PM_WKST_USBHOST   %08x\n",   log->wkst_USBHOST);

		omap3_show_iopad_wkup_event ();
	}

	if( log_lvl >= LOG_LVL_WAKEUP_PIN_STATE ) {
		printk(KERN_INFO "PIN STATE\n");
		printk(KERN_INFO "GPIO1_IRQ_ST      %08x\n",   log->gpio1_irq_st  );
		printk(KERN_INFO "GPIO2_IRQ_ST      %08x\n",   log->gpio2_irq_st  );
		printk(KERN_INFO "GPIO3_IRQ_ST      %08x\n",   log->gpio3_irq_st  );
		printk(KERN_INFO "GPIO4_IRQ_ST      %08x\n",   log->gpio4_irq_st  );
		printk(KERN_INFO "GPIO5_IRQ_ST      %08x\n",   log->gpio5_irq_st  );
		printk(KERN_INFO "GPIO6_IRQ_ST      %08x\n",   log->gpio6_irq_st  );
		printk(KERN_INFO "GPIO1_DATA_IN     %08x\n",   log->gpio1_data_in );
		printk(KERN_INFO "GPIO2_DATA_IN     %08x\n",   log->gpio2_data_in );
		printk(KERN_INFO "GPIO3_DATA_IN     %08x\n",   log->gpio3_data_in );
		printk(KERN_INFO "GPIO4_DATA_IN     %08x\n",   log->gpio4_data_in );
		printk(KERN_INFO "GPIO5_DATA_IN     %08x\n",   log->gpio5_data_in );
		printk(KERN_INFO "GPIO6_DATA_IN     %08x\n",   log->gpio6_data_in );
	}
	
}


/******************************************************************************
 *
 * DEBUG
 *
 ******************************************************************************/

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
# define DPRINTK(fmt, args...)
#endif

/* Keep debug during suspend enabled for Castles for now to make debugging of
 * driver suspend functions easier...
 */
#ifdef CONFIG_DISABLE_CONSOLE_SUSPEND
#  ifdef CONFIG_MACH_SIRLOIN
#  define KEEP_UART3_ALIVE_DURING_SUSPEND
#  define printd(args...) printk(args)
#  endif

#  ifdef CONFIG_MACH_FLANK
#  define KEEP_UART3_ALIVE_DURING_SUSPEND
#  define printd(args...) printk(args)
#  endif
#else
#  define printd(args...)
// -wgr- #  define printd(args...) printk(args)
// -wgr- #define KEEP_UART3_ALIVE_DURING_SUSPEND
// -wgr- #define KEEP_UART1_ALIVE_DURING_SUSPEND
#endif

static void DUMP_REGS_BEFORE_SLEEP(void)
{
	printd("########### BEFORE SLEEP ###################\n");
	printd("##### PRM_VC_SMPS_SA     %08x\n", PRM_VC_SMPS_SA);
	printd("##### PRM_VC_SMPS_VOL_RA %08x\n", PRM_VC_SMPS_VOL_RA);
	printd("##### PRM_VC_SMPS_CMD_RA %08x\n", PRM_VC_SMPS_CMD_RA);
	printd("##### PRM_VC_CMD_VAL_0   %08x\n", PRM_VC_CMD_VAL_0);
	printd("##### PRM_VC_CMD_VAL_1   %08x\n", PRM_VC_CMD_VAL_1);
	printd("##### PRM_VC_CH_CONF     %08x\n", PRM_VC_CH_CONF);
	printd("##### PRM_VC_I2C_CFG     %08x\n", PRM_VC_I2C_CFG);
	printd("##### PRM_VC_BYPASS_VAL  %08x\n", PRM_VC_BYPASS_VAL);
	printd("##### PRM_VOLTCTRL       %08x\n", PRM_VOLTCTRL);
	printd("##### PRM_VOLTOFFSET     %08x\n", PRM_VOLTOFFSET);
	printd("##### PRM_VOLTSETUP1     %08x\n", PRM_VOLTSETUP1);
	printd("##### PRM_VOLTSETUP2     %08x\n", PRM_VOLTSETUP2);
	printd("##### PRM_VP1_CONFIG     %08x\n", PRM_VP1_CONFIG);
	printd("##### PRM_VP1_VSTEPMIN   %08x\n", PRM_VP1_VSTEPMIN);
	printd("##### PRM_VP1_VSTEPMAX   %08x\n", PRM_VP1_VSTEPMAX);
	printd("##### PRM_VP1_VLIMITTO   %08x\n", PRM_VP1_VLIMITTO);
	printd("##### PRM_VP1_VOLTAGE    %08x\n", PRM_VP1_VOLTAGE);
	printd("##### PRM_VP1_STATUS     %08x\n", PRM_VP1_STATUS);
	printd("##### PRM_VP2_CONFIG     %08x\n", PRM_VP2_CONFIG);
	printd("##### PRM_VP2_VSTEPMIN   %08x\n", PRM_VP2_VSTEPMIN);
	printd("##### PRM_VP2_VSTEPMAX   %08x\n", PRM_VP2_VSTEPMAX);
	printd("##### PRM_VP2_VLIMITTO   %08x\n", PRM_VP2_VLIMITTO);
	printd("##### PRM_VP2_VOLTAGE    %08x\n", PRM_VP2_VOLTAGE);
	printd("##### PRM_VP2_STATUS     %08x\n", PRM_VP2_STATUS);
#ifdef CONFIG_MACH_SIRLOIN_3630
	printd("##### PRM_LDO_ABB_SETUP  %08x\n", PRM_LDO_ABB_SETUP);
	printd("##### PRM_LDO_ABB_CTRL   %08x\n", PRM_LDO_ABB_CTRL);
#endif
	printd("##### PRCM_GPIO1_SYSCONFIG %08x\n", PRCM_GPIO1_SYSCONFIG);
	printd("##### CM_CLKSTCTRL_MPU  %08x\n", CM_CLKSTCTRL_MPU);
	printd("##### CM_CLKSTCTRL_CORE %08x\n", CM_CLKSTCTRL_CORE);
	printd("##### CM_CLKSTCTRL_CAM  %08x\n", CM_CLKSTCTRL_CAM);
	printd("##### CM_CLKSTCTRL_DSS  %08x\n", CM_CLKSTCTRL_DSS);
	printd("##### CM_CLKSTCTRL_PER  %08x\n", CM_CLKSTCTRL_PER);
	printd("##### CM_CLKSTCTRL_NEON %08x\n", CM_CLKSTCTRL_NEON);
	printd("##### CM_CLKSTCTRL_IVA2 %08x\n", CM_CLKSTCTRL_IVA2);
	printd("##### CM_CLKSTCTRL_SGX  %08x\n", CM_CLKSTCTRL_SGX);
	printd("##### CM_CLKSTCTRL_EMU  %08x\n", CM_CLKSTCTRL_EMU);
	printd("##### CM_CLKSTCTRL_USBHOST %08x\n", CM_CLKSTCTRL_USBHOST);
	printd("##### CM_IDLEST_IVA2  %08x\n", CM_IDLEST_IVA2);
	printd("##### CM_IDLEST_MPU   %08x\n", CM_IDLEST_MPU);
	printd("##### CM_IDLEST1_CORE %08x\n", CM_IDLEST1_CORE);
	printd("##### CM_IDLEST2_CORE %08x\n", CM_IDLEST2_CORE);
	printd("##### CM_IDLEST3_CORE %08x\n", CM_IDLEST3_CORE);
	printd("##### CM_IDLEST_PER   %08x\n", CM_IDLEST_PER);
	printd("##### CM_IDLEST_CAM   %08x\n", CM_IDLEST_CAM);
	printd("##### CM_IDLEST_DSS   %08x\n", CM_IDLEST_DSS);
	printd("##### CM_IDLEST_NEON  %08x\n", CM_IDLEST_NEON);
	printd("##### CM_IDLEST_WKUP  %08x\n", CM_IDLEST_WKUP);
	printd("##### CM_IDLEST_SGX   %08x\n", CM_IDLEST_SGX);
	printd("##### CM_IDLEST_USBHOST  %08x\n", CM_IDLEST_USBHOST);
	printd("##### CM_AUTOIDLE1_CORE %08x\n", CM_AUTOIDLE1_CORE);
	printd("##### CM_AUTOIDLE2_CORE %08x\n", CM_AUTOIDLE2_CORE);
	printd("##### CM_AUTOIDLE3_CORE %08x\n", CM_AUTOIDLE3_CORE);
	printd("##### CM_AUTOIDLE_WKUP  %08x\n", CM_AUTOIDLE_WKUP);
	printd("##### CM_AUTOIDLE_DSS   %08x\n", CM_AUTOIDLE_DSS);
	printd("##### CM_AUTOIDLE_PER   %08x\n", CM_AUTOIDLE_PER);
	printd("##### CM_AUTOIDLE_CAM   %08x\n", CM_AUTOIDLE_CAM);
	printd("##### CM_AUTOIDLE_PLL_MPU  %08x\n", CM_AUTOIDLE_PLL_MPU);
	printd("##### CM_AUTOIDLE_PLL_IVA2 %08x\n", CM_AUTOIDLE_PLL_IVA2);
	printd("##### CM_AUTOIDLE_PLL      %08x\n", CM_AUTOIDLE_PLL);
	printd("##### PM_PWSTCTRL_MPU     %08x\n", PM_PWSTCTRL_MPU);
	printd("##### PM_PWSTCTRL_CORE    %08x\n", PM_PWSTCTRL_CORE);
	printd("##### PM_PWSTCTRL_SGX     %08x\n", PM_PWSTCTRL_SGX);
	printd("##### PM_PWSTCTRL_DSS     %08x\n", PM_PWSTCTRL_DSS);
	printd("##### PM_PWSTCTRL_CAM     %08x\n", PM_PWSTCTRL_CAM);
	printd("##### PM_PWSTCTRL_PER     %08x\n", PM_PWSTCTRL_PER);
	printd("##### PM_PWSTCTRL_NEON    %08x\n", PM_PWSTCTRL_NEON);
	printd("##### PM_PWSTCTRL_IVA2    %08x\n", PM_PWSTCTRL_IVA2);
	printd("##### PM_PWSTCTRL_USBHOST %08x\n", PM_PWSTCTRL_USBHOST);
	printd("##### PM_PWSTST_MPU     %08x\n", PM_PWSTST_MPU);
	printd("##### PM_PWSTST_CORE    %08x\n", PM_PWSTST_CORE);
	printd("##### PM_PWSTST_SGX     %08x\n", PM_PWSTST_SGX);
	printd("##### PM_PWSTST_DSS     %08x\n", PM_PWSTST_DSS);
	printd("##### PM_PWSTST_CAM     %08x\n", PM_PWSTST_CAM);
	printd("##### PM_PWSTST_PER     %08x\n", PM_PWSTST_PER);
	printd("##### PM_PWSTST_NEON    %08x\n", PM_PWSTST_NEON);
	printd("##### PM_PWSTST_EMU     %08x\n", PM_PWSTST_EMU);
	printd("##### PM_PWSTST_IVA2    %08x\n", PM_PWSTST_IVA2);
	printd("##### PM_PWSTST_USBHOST %08x\n", PM_PWSTST_USBHOST);
	printd("##### CM_ICLKEN1_CORE   %08x\n", CM_ICLKEN1_CORE);
	printd("##### CM_ICLKEN2_CORE   %08x\n", CM_ICLKEN2_CORE);
	printd("##### CM_ICLKEN3_CORE   %08x\n", CM_ICLKEN3_CORE);
	printd("##### CM_ICLKEN_SGX     %08x\n", CM_ICLKEN_SGX);
	printd("##### CM_ICLKEN_DSS     %08x\n", CM_ICLKEN_DSS);
	printd("##### CM_ICLKEN_CAM     %08x\n", CM_ICLKEN_CAM);
	printd("##### CM_ICLKEN_PER     %08x\n", CM_ICLKEN_PER);
	printd("##### CM_ICLKEN_WKUP    %08x\n", CM_ICLKEN_WKUP);
	printd("##### CM_ICLKEN_USBHOST %08x\n", CM_ICLKEN_USBHOST);
	printd("##### CM_CLKEN_PLL_MPU  %08x\n", CM_CLKEN_PLL_MPU);
	printd("##### CM_CLKEN_PLL_IVA2 %08x\n", CM_CLKEN_PLL_IVA2);
	printd("##### CM_FCLKEN1_CORE   %08x\n", CM_FCLKEN1_CORE);
	printd("##### CM_FCLKEN3_CORE   %08x\n", CM_FCLKEN3_CORE);
	printd("##### CM_FCLKEN_SGX     %08x\n", CM_FCLKEN_SGX);
	printd("##### CM_FCLKEN_DSS     %08x\n", CM_FCLKEN_DSS);
	printd("##### CM_FCLKEN_CAM     %08x\n", CM_FCLKEN_CAM);
	printd("##### CM_FCLKEN_PER     %08x\n", CM_FCLKEN_PER);
	printd("##### CM_FCLKEN_WKUP    %08x\n", CM_FCLKEN_WKUP);
	printd("##### CM_FCLKEN_USBHOST %08x\n", CM_FCLKEN_USBHOST);
	printd("##### CM_CLKSEL_CORE    %08x\n", CM_CLKSEL_CORE);
	printd("##### CM_CLKSEL_CAM     %08x\n", CM_CLKSEL_CAM);
	printd("##### CM_CLKSEL_DSS     %08x\n", CM_CLKSEL_DSS);
	printd("##### CM_CLKSEL_PER     %08x\n", CM_CLKSEL_PER);
	printd("##### CM_CLKSEL_SGX     %08x\n", CM_CLKSEL_SGX);
	printd("##### CM_CLKSEL_WKUP    %08x\n", CM_CLKSEL_WKUP);
	printd("##### CM_SLEEPDEP_DSS     %08x\n", CM_SLEEPDEP_DSS);
	printd("##### CM_SLEEPDEP_CAM     %08x\n", CM_SLEEPDEP_DSS);
	printd("##### CM_SLEEPDEP_PER     %08x\n", CM_SLEEPDEP_DSS);
	printd("##### CM_SLEEPDEP_SGX     %08x\n", CM_SLEEPDEP_DSS);
	printd("##### CM_SLEEPDEP_USBHOST %08x\n", CM_SLEEPDEP_DSS);
	printd("##### PM_WKDEP_IVA2    %08x\n", PM_WKDEP_IVA2);
	printd("##### PM_WKDEP_MPU     %08x\n", PM_WKDEP_MPU);
	printd("##### PM_WKDEP_DSS     %08x\n", PM_WKDEP_DSS);
	printd("##### PM_WKDEP_NEON    %08x\n", PM_WKDEP_NEON);
	printd("##### PM_WKDEP_CAM     %08x\n", PM_WKDEP_CAM);
	printd("##### PM_WKDEP_PER     %08x\n", PM_WKDEP_PER);
	printd("##### PM_WKDEP_SGX     %08x\n", PM_WKDEP_SGX);
	printd("##### PM_WKDEP_USBHOST %08x\n", PM_WKDEP_USBHOST);
	printd("##### PM_WKST1_CORE    %08x\n", PM_WKST1_CORE);
	printd("##### PM_WKST3_CORE    %08x\n", PM_WKST3_CORE);
	printd("##### PM_WKST_WKUP     %08x\n", PM_WKST_WKUP);
	printd("##### PM_WKST_PER      %08x\n", PM_WKST_PER);
	printd("##### PM_WKST_USBHOST  %08x\n", PM_WKST_USBHOST);
	printd("##### PM_WKEN_WKUP     %08x\n", PM_WKEN_WKUP);
	printd("##### PM_WKEN_PER      %08x\n", PM_WKEN_PER);
	printd("##### PRM_CLKSRC_CTRL %08x\n", PRM_CLKSRC_CTRL);
	printd("##### PRM_CLKSETUP    %08x\n", PRM_CLKSETUP);
	printd("##### PM_MPUGRPSEL_WKUP  %08x\n", PM_MPUGRPSEL_WKUP);
	printd("##### PM_MPUGRPSEL_PER   %08x\n", PM_MPUGRPSEL_PER);
	printd("##### PRM_IRQENABLE_MPU %08x\n", PRM_IRQENABLE_MPU);
	printd("##### PRM_IRQSTATUS_MPU %08x\n", PRM_IRQSTATUS_MPU);

	printd("##### SDRC_SYSCONFIG    %08x\n", SDRC_SYSCONFIG);
	printd("##### CONTROL_SYSCONFIG              %08x\n", CONTROL_SYSCONFIG);
	printd("##### CONTROL_PADCONF_I2C4_SCL       %08x\n", CONTROL_PADCONF_I2C4_SCL);
	printd("##### CONTROL_PADCONF_SYS_OFF_MODE   %08x\n", CONTROL_PADCONF_SYS_OFF_MODE);
	printd("##### CONTROL_PADCONF_SAD2D_IDLEACK  %08x\n", CONTROL_PADCONF_SAD2D_IDLEACK);
	printd("##### CONTROL_PADCONF_SAD2D_MSTDBY   %08x\n", CONTROL_PADCONF_SAD2D_MSTDBY);
	printd("##### CONTROL_PADCONF_SYS_32K        %08x\n", CONTROL_PADCONF_SYS_32K);
	printd("##### PRM_POLCTRL  %08x\n", PRM_POLCTRL);
	printd("##### PRM_VOLTCTRL %08x\n", PRM_VOLTCTRL);
	printd("##### SDRC_POWER_REG %08x\n", SDRC_POWER_REG);

	printd("##### CM_ICLKEN1_CORE %08x\n", CM_ICLKEN1_CORE);
	printd("##### CM_FCLKEN1_CORE %08x\n", CM_FCLKEN1_CORE);
	printd("##### CM_IDLEST1_CORE %08x\n", CM_IDLEST1_CORE);
}

static void DUMP_REGS_AFTER_WAKEUP(void)
{
	struct wkup_log_t *log = wkup_log_get_current();

	(void) log; // suppress unused var 

	printd("########### AFTER WAKE UP ##################\n");
	printd("##### PM_WKST1_CORE   %08x\n",   log->wkst_CORE1);
	printd("##### PM_WKST3_CORE   %08x\n",   log->wkst_CORE3);
	printd("##### PM_WKST_WKUP    %08x\n",   log->wkst_WKUP);
	printd("##### PM_WKST_PER     %08x\n",   log->wkst_PER);
	printd("##### PM_WKST_USBHOST %08x\n",   log->wkst_USBHOST);

	printd("##### PM_PREPWSTST_MPU  %08x\n", log->prepwst_MPU);
	printd("##### PM_PREPWSTST_CORE %08x\n", log->prepwst_CORE);
	printd("##### PM_PREPWSTST_IVA2 %08x\n", log->prepwst_IVA2);
	printd("##### PM_PREPWSTST_PER  %08x\n", log->prepwst_PER);
	printd("##### PM_PREPWSTST_DSS  %08x\n", log->prepwst_DSS);
	printd("##### PM_PREPWSTST_CAM  %08x\n", log->prepwst_CAM);
	printd("##### PM_PREPWSTST_SGX  %08x\n", log->prepwst_SGX);
	printd("##### PM_PREPWSTST_NEON %08x\n", log->prepwst_NEON);
	printd("##### PM_PREPWSTST_USBHOST %08x\n", log->prepwst_USBHOST);
	
	printd("##### SDRC_POWER_REG %08x\n",    log->sdrc_pwr);
	printd("##### SDRC_RFR_CTRL_0 %08x\n",   log->sdrc_refr_ctrl0);
	printd("##### SDRC_RFR_CTRL_1 %08x\n",   log->sdrc_refr_ctrl1);
	printd("##### SDRC_SYSCONFIG  %08x\n",   log->sdrc_syscfg);
	printd("##### PRM_IRQENABLE_MPU %08x\n", log->prm_mpu_irq_en);
	printd("##### PRM_IRQSTATUS_MPU %08x\n", log->prm_mpu_irq_st);

	printd("##### CM_IDLEST_CKGEN  was %08x\n", log->ckgen);
	printd("##### idlest1_core   was %08x\n",   log->idlest1_core);
	printd("##### idlest2_core   was %08x\n",   log->idlest2_core);
	printd("##### idlest3_core   was %08x\n",   log->idlest3_core);
	printd("##### iclken1_core   was %08x\n",   log->iclken1_core);
	printd("##### iclken2_core   was %08x\n",   log->iclken2_core);
	printd("##### iclken3_core   was %08x\n",   log->iclken3_core);
	printd("##### fclken1_core   was %08x\n",   log->fclken1_core);
	printd("##### fclken3_core   was %08x\n",   log->fclken3_core);
	printd("##### autoidle1_core was %08x\n",   log->autoidle1_core);
	printd("##### autoidle2_core was %08x\n",   log->autoidle2_core);
	printd("##### autoidle3_core was %08x\n",   log->autoidle3_core);

}

/******************************************************************************
 *
 * Tables to store sysconfig registers for each PRCM ID CORE
 *
 ******************************************************************************/

#define SYSCONFIG_REG_ENTRY(reg) {reg, &reg##_SYSCONFIG, #reg}

struct sysconf_reg {
	u32		device_id;
	volatile u32	*reg_addr;
	const char	*name;
};

static struct sysconf_reg sysc_reg_core1[] = {
	SYSCONFIG_REG_ENTRY(PRCM_SSI),
	SYSCONFIG_REG_ENTRY(PRCM_SDRC),
	SYSCONFIG_REG_ENTRY(PRCM_SDMA),
	SYSCONFIG_REG_ENTRY(PRCM_HSOTG),
	SYSCONFIG_REG_ENTRY(PRCM_OMAP_CTRL),
	SYSCONFIG_REG_ENTRY(PRCM_MBOXES),
	SYSCONFIG_REG_ENTRY(PRCM_MCBSP1),
	SYSCONFIG_REG_ENTRY(PRCM_MCBSP5),
	SYSCONFIG_REG_ENTRY(PRCM_GPT10),
	SYSCONFIG_REG_ENTRY(PRCM_GPT11),
	SYSCONFIG_REG_ENTRY(PRCM_UART1),
	SYSCONFIG_REG_ENTRY(PRCM_UART2),
	SYSCONFIG_REG_ENTRY(PRCM_I2C1),
	SYSCONFIG_REG_ENTRY(PRCM_I2C2),
	SYSCONFIG_REG_ENTRY(PRCM_I2C3),
	SYSCONFIG_REG_ENTRY(PRCM_MCSPI1),
	SYSCONFIG_REG_ENTRY(PRCM_MCSPI2),
	SYSCONFIG_REG_ENTRY(PRCM_MCSPI3),
	SYSCONFIG_REG_ENTRY(PRCM_MCSPI4),
	SYSCONFIG_REG_ENTRY(PRCM_HDQ),
	SYSCONFIG_REG_ENTRY(PRCM_MMC1),
	SYSCONFIG_REG_ENTRY(PRCM_MMC2),
	SYSCONFIG_REG_ENTRY(PRCM_SMS),
	SYSCONFIG_REG_ENTRY(PRCM_GPMC),
	SYSCONFIG_REG_ENTRY(PRCM_MPU_INTC),
	SYSCONFIG_REG_ENTRY(PRCM_MMC3),
	{0},
};

/* WKUP */
static struct sysconf_reg sysc_reg_wkup[] = {
	SYSCONFIG_REG_ENTRY(PRCM_GPIO1),
	SYSCONFIG_REG_ENTRY(PRCM_GPT1),
	SYSCONFIG_REG_ENTRY(PRCM_GPT12),
	SYSCONFIG_REG_ENTRY(PRCM_WDT2),
	{0},
};

/* DSS */
static struct sysconf_reg sysc_reg_dss[] = {
	SYSCONFIG_REG_ENTRY(PRCM_DSS),
	SYSCONFIG_REG_ENTRY(PRCM_DISPC),
	SYSCONFIG_REG_ENTRY(PRCM_RFBI),
	{0},
};

/* CAM */
static struct sysconf_reg sysc_reg_cam[] = {
	SYSCONFIG_REG_ENTRY(PRCM_CAM),
	SYSCONFIG_REG_ENTRY(PRCM_CSIA),
#ifndef CONFIG_ARCH_OMAP3410
	SYSCONFIG_REG_ENTRY(PRCM_CSIB),
#endif
	SYSCONFIG_REG_ENTRY(PRCM_MMU),
	{0},
};

/* PER */
static struct sysconf_reg sysc_reg_per[] = {
	SYSCONFIG_REG_ENTRY(PRCM_MCBSP2),
	SYSCONFIG_REG_ENTRY(PRCM_MCBSP3),
	SYSCONFIG_REG_ENTRY(PRCM_MCBSP4),
	SYSCONFIG_REG_ENTRY(PRCM_GPT2),
	SYSCONFIG_REG_ENTRY(PRCM_GPT3),
	SYSCONFIG_REG_ENTRY(PRCM_GPT4),
	SYSCONFIG_REG_ENTRY(PRCM_GPT5),
	SYSCONFIG_REG_ENTRY(PRCM_GPT6),
	SYSCONFIG_REG_ENTRY(PRCM_GPT7),
	SYSCONFIG_REG_ENTRY(PRCM_GPT8),
	SYSCONFIG_REG_ENTRY(PRCM_GPT9),
	SYSCONFIG_REG_ENTRY(PRCM_UART3),
	SYSCONFIG_REG_ENTRY(PRCM_WDT3),
	SYSCONFIG_REG_ENTRY(PRCM_GPIO2),
	SYSCONFIG_REG_ENTRY(PRCM_GPIO3),
	SYSCONFIG_REG_ENTRY(PRCM_GPIO4),
	SYSCONFIG_REG_ENTRY(PRCM_GPIO5),
	SYSCONFIG_REG_ENTRY(PRCM_GPIO6),
	{0},
};


/*============================================================================*/
/*======================== POWER DOMAIN STATUS ===============================*/
/*============================================================================*/
/* This function waits for the power domain to transition to the desired state*/
/* It polls on the power domain state and times out after a wait of ~500 micro*/
/* secs. It returns PRCM_PASS id the power domain transitions to the desired  */
/* state within the timeout period, else return PRCM_FAIL                     */
/*============================================================================*/

static int prcm_check_power_domain_status(u32 domainid, u8 desired_state)
{
	u8 curr_state;
	int retries;
	int ret;

	if ((domainid == DOM_CORE1) ||
	    (domainid == DOM_CORE2) ||
	    (domainid == DOM_MPU)) {
		printk(KERN_INFO "%s() is not supported for "
			 "DOM_CORE1/DOM_CORE2/DOM_MPU\n", __FUNCTION__);
		return PRCM_FAIL;
	}

	FAIL_ON_INVALID_DOMAINID(domainid);

	retries = 500;
	do {
		ret = prcm_get_power_domain_state(domainid, &curr_state);
		if (ret)
			return ret;

		if (curr_state == desired_state) {
			return PRCM_PASS;
		}

		udelay(1);
	} while (retries--);

#if 0
	printk(KERN_DEBUG "Timeout in prcm_check_power_domain_status(), "
			"domain %d, desired state %d, current state %d\n",
			 domainid, desired_state, curr_state);
#endif			 
	return PRCM_FAIL;
}

/******************************************************************************
 *
 * Prepare the MPU and CORE domains for suspend.
 *
 ******************************************************************************/

static int prcm_prepare_mpu_core_domains(struct system_power_state *state)
{
	int initiators;

	if (state->mpu_state > PRCM_MPU_ACTIVE) {
		prcm_set_mpu_domain_state(state->mpu_state);
		CM_AUTOIDLE_PLL_MPU = 0x1;
	}

	if (state->core_state > PRCM_CORE_ACTIVE) {
		u32 val;

		/* Disable functional clocks in core and wakeup domains.
		 * Enable interface clock autoidle.
		 */
		CM_FCLKEN1_CORE  = 0x0;
#ifdef KEEP_UART1_ALIVE_DURING_SUSPEND
		CM_FCLKEN1_CORE  = (1<<13); /* UART1 */
#endif
		CM_FCLKEN_WKUP  &= 0x9;    /* GPT1 + GPIO1 */
		/* Disable HSUSB ICLK explicitly. */
		CM_ICLKEN1_CORE &= ~0x10;  /* HS OTG USB */
		CM_FCLKEN3_CORE  = 0x0;

		/* Autoidle PLLs. */
		CM_AUTOIDLE_PLL_IVA2 = 0x1;
		CM_AUTOIDLE_PLL_MPU  = 0x1;
		CM_AUTOIDLE_PLL      = 0x9;
		CM_AUTOIDLE2_PLL     = 0x1;

		/* De-assert SYS_CLKREQ on SLEEP, RET and OFF. */
		val = PRM_CLKSRC_CTRL;
		val &= ~(3<<3);
		val |=  (1<<3);
		PRM_CLKSRC_CTRL = val;

		/* Autoidle domain clocks */
		CM_AUTOIDLE_WKUP  = 0x23f;
		CM_AUTOIDLE1_CORE = 0x7FFFFED1;
		CM_AUTOIDLE2_CORE = 0x1f;
		CM_AUTOIDLE3_CORE = 0x4;

		if (is_sil_rev_greater_than(OMAP3430_REV_ES3_0)) {
			PM_PWSTCTRL_CORE |= PM_PWSTCTRL_SARMASK;
//			printk(KERN_INFO "@ Set PM_PWSTCTRL_CORE=%d\n",
//			       PM_PWSTCTRL_SARMASK);
		}

		prcm_set_core_domain_state(state->core_state);

		prcm_get_initiators_not_standby(DOM_CORE1, &initiators);
		if (initiators) {
			printk(KERN_INFO "Initiators still active in core "
					"domain mask: 0x%x\n", initiators);
			return PRCM_FAIL;
		}
	}
	return PRCM_PASS;
}

/******************************************************************************
 *
 * Restore MPU and CORE settings after wake up.
 *
 ******************************************************************************/

static void prcm_restore_mpu_core_domains(struct system_power_state *state)
{
	if (state->mpu_state > PRCM_MPU_ACTIVE) {
		prcm_set_mpu_domain_state(PRCM_MPU_ACTIVE);
		CM_AUTOIDLE_PLL_MPU	= 0x0;
	}
	if (state->core_state > PRCM_CORE_ACTIVE) {
		prcm_set_core_domain_state(PRCM_CORE_ACTIVE);
		CM_AUTOIDLE_PLL_IVA2	= 0x0;
		CM_AUTOIDLE_PLL		= 0x0;
		CM_AUTOIDLE2_PLL	= 0x0;
		PRM_CLKSRC_CTRL		&= ~0x18;
	}
	/* Rest of registers are restored later in prcm_register_restore. */
}

/******************************************************************************
 *
 * Check parameters for prcm_set_chip_power_mode()
 *
 ******************************************************************************/

static int check_power_mode_parameters(struct system_power_state *state)
{
	u32 id_type;
	u32 core_inactive_allowed = 1;

	id_type = get_other_id_type(state->mpu_state);
	if (!(id_type & ID_MPU_DOM_STATE)) {
		printk(KERN_INFO "Invalid parameter for mpu state\n");
		return PRCM_FAIL;
	}

	id_type = get_other_id_type(state->core_state);
	if (!(id_type & ID_CORE_DOM_STATE)) {
		printk(KERN_INFO "Invalid parameter for core state\n");
		return PRCM_FAIL;
	}

	if (state->mpu_state > PRCM_MPU_OFF) {
		printk(KERN_INFO
			"Unsupported state for mpu: %d\n", state->mpu_state);
		return PRCM_FAIL;
	}

#ifdef CONFIG_ARCH_OMAP3410
	if ((state->iva2_state > PRCM_RET) ||
	    (state->dss_state  > PRCM_RET) ||
	    (state->cam_state  > PRCM_RET) ||
	    (state->per_state  > PRCM_RET))
#else
	if ((state->iva2_state > PRCM_RET) ||
	    (state->gfx_state  > PRCM_RET) ||
	    (state->dss_state  > PRCM_RET) ||
	    (state->cam_state  > PRCM_RET) ||
	    (state->per_state  > PRCM_RET) ||
	    (state->neon_state > PRCM_RET))
#endif
		core_inactive_allowed = 0;

	if ((!core_inactive_allowed) &&
	    (state->core_state > PRCM_CORE_ACTIVE)) {
		printk(KERN_INFO "Invalid combination of states: "
		                 "Core has to be active\n");
		return PRCM_FAIL;
	}

	return PRCM_PASS;
}

/******************************************************************************
 *
 * PUBLIC API
 *
 ******************************************************************************/

/*========================================================================*/
/*===================GET POWER DOMAIN STATE===============================*/
/*= This function returns the current state of a power domain. If the    =*/
/*= power domain is in the middle of a state transition, it waits for the=*/
/*= transition to complete.                                              =*/
/*========================================================================*/

int prcm_get_power_domain_state(u32 domainid, u8 *result)
{
	volatile u32 *addr;
	int rc;

	if (!result) {
		printk(KERN_ERR
			"NULL pointer passed in prcm_get_power_domain_state()\n");
		return PRCM_FAIL;
	}

	FAIL_ON_INVALID_DOMAINID(domainid);

	/* Core domain check is not supported */
	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk(KERN_INFO "Currently prcm_is_clock_domain_active for "
			"the following domains (DOM_CORE1/DOM_CORE2) "
			"is not supported\n");
		return PRCM_FAIL;
	}

	addr  = get_addr(domainid, REG_PWSTST);
	if (!addr)
		return PRCM_FAIL;

	rc = WAIT_WHILE(*addr & PWSTST_INTRANS_MASK, 5000);
	if (rc < 0) {
		printk(KERN_ERR "Timeout getting domain power state "
				"for domain: %d\n", domainid);
		return PRCM_FAIL;
	}

	*result = (u8) *addr & PWSTST_PWST_MASK;
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET POWER DOMAIN STATE ============================*/
/*============================================================================*/
/* This function sets the power domain state to the 'new_state' specified. If */
/* mode is 'PRCM_AUTO', the clock domain is programmed in Hardware supervised */
/* mode and the function waits for the power domain to transition to the      */
/* desired state. If mode is 'PRCM_FORCE' the clock domain is programmed in   */
/* sofware supervised mode and the function does not wait for the power       */
/* domain transition to happen.                                               */
/*============================================================================*/

int prcm_set_power_domain_state(u32 domainid, u8 new_state, u8 mode)
{
	volatile u32 *addr;
	int ret;

	if ((domainid == DOM_CORE1) || (domainid == DOM_CORE2)) {
		printk(KERN_INFO "Currently prcm_set_power_domain_state "
			"for the following domains (DOM_CORE1/DOM_CORE2) "
			"is not supported\n");
		return PRCM_FAIL;
	}

	addr = get_addr(domainid, REG_PWSTCTRL);
	if (!addr)
		return PRCM_FAIL;

	if (new_state == PRCM_ON) {
		volatile u32 *rstst_addr;

		rstst_addr = get_addr(domainid, REG_RSTST);
		if (rstst_addr)
			*rstst_addr |= DOM_WKUP_RST;
	}

	/* Enable SAR for USB domain.
	 */
	if ((domainid == DOM_USBHOST) && (new_state == PRCM_OFF))
		*addr |= PM_PWSTCTRL_SARMASK;

	switch (mode) {
	case PRCM_AUTO:
		/* Set the power domain state to new_state */
		*addr = (*addr & ~PWSTST_PWST_MASK) | new_state;
		ret = prcm_set_clock_domain_state(domainid,
						PRCM_HWSUP_AUTO, PRCM_FALSE);
		break;

	case PRCM_FORCE:
		if (domainid == DOM_MPU)
			return PRCM_FAIL; /* No force mode for MPU */

		*addr = (*addr & ~PWSTST_PWST_MASK) | new_state;

		if ((new_state == PRCM_OFF) || (new_state == PRCM_RET)) {
			ret = prcm_set_clock_domain_state(
					domainid, PRCM_SWSUP_SLEEP, PRCM_TRUE);
			if (ret != PRCM_PASS)
				return ret;
		} else {
			ret = prcm_set_clock_domain_state(
					domainid, PRCM_SWSUP_WKUP, PRCM_FALSE);
			if (ret != PRCM_PASS)
				return ret;
		}

		/* Wait for the power domain transition to complete */
		ret = prcm_check_power_domain_status(domainid, new_state);
		break;

	default:
		printk(KERN_WARNING
			"Unknown mode %d in prcm_set_clock_domain_state()\n", mode);
		ret = PRCM_FAIL;
	}

	return ret;
}

/*========================================================================*/
/*===============GET PREVIOUS POWER DOMAIN STATE==========================*/
/*= This function returns the previous state of a power domain.          =*/
/*========================================================================*/

int prcm_get_pre_power_domain_state(u32 domainid, u8 *result)
{
	volatile u32 *addr;

	FAIL_ON_INVALID_DOMAINID(domainid);

	addr = get_addr(domainid, REG_PREPWSTST);
	if (!addr)
		return PRCM_FAIL;

	*result = (u8) *addr & PWSTST_PWST_MASK;
	return PRCM_PASS;
}

/******************************************************************************
 *
 * Level 2 API for forcing the state of a particular power domain.
 *
 * This API disables clocks if required and uses software mode to change the
 * state of a power domain This API will be called during bootup to turn off
 * unwanted power domains.
 *
 ******************************************************************************/

int prcm_force_power_domain_state(u32 domain, u8 state)
{
	int ret;

	/* Not allowed for MPU */
	if (DOM_MPU == domain) {
		BUG();
	}

	if ((domain == DOM_CORE1) || (domain == DOM_CORE2)) {
		printk(KERN_INFO "Currently prcm_force_power_domain_state is not "
			"supported for CORE domain\n");
			return PRCM_FAIL;
	}

	if (state == PRCM_ON) {
		ret = prcm_set_power_domain_state(domain, PRCM_ON, PRCM_FORCE);
		if (ret != PRCM_PASS)
			return ret;
	}

	if ((state == PRCM_RET) || (state == PRCM_OFF)) {
		/* Force all clocks in the power domain to be off */
		prcm_set_domain_interface_clocks (domain, 0x0);
		prcm_set_domain_functional_clocks(domain, 0x0);

		ret = prcm_set_power_domain_state(domain, state, PRCM_FORCE);
		if (ret != PRCM_PASS)
			return ret;
	}

	if (state == PRCM_INACTIVE) {
		/* Force all clocks in the power domain to be off */
		prcm_set_domain_interface_clocks (domain, 0x0);
		prcm_set_domain_functional_clocks(domain, 0x0);
		ret = prcm_set_clock_domain_state(domain,
						PRCM_HWSUP_AUTO, PRCM_FALSE);
		if (ret != PRCM_PASS)
			return ret;
	}
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== DEVICES NOT IDLE  =================================*/
/*============================================================================*/
/*= This function returns a mask of devices that are not idle in the         =*/
/*= specified domain. It reads the CM_IDLEST_<DOMAIN> register to generate   =*/
/*= the mask. Each Bit in the mask which is set to 1, specifies the          =*/
/*= corresponding device is not idle. The function returns PRCM_FAIL if the  =*/
/*= specified device does not have a corresponding IDLEST register.          =*/
/*============================================================================*/

int prcm_get_devices_not_idle(u32 domainid, u32 *result)
{
	volatile u32 *addr;
	u32 valid;

	*result = 0x0;

	FAIL_ON_INVALID_DOMAINID(domainid);

	addr  = get_addr(domainid, REG_IDLEST);
	valid = get_val_bits(domainid, REG_IDLEST);

	if (!addr)
		return PRCM_FAIL;

	*result = (~(*addr & valid) & valid);
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== INITIATORS NOT STANDBY  ===========================*/
/*============================================================================*/
/*= This function returns a mask of initiators that are not in standby mode. =*/
/*= Each bit in the mask which is set to 1, specifies that the Standby is not=*/
/*= asserted for the corresponding initiator. The function returns PRCM_FAIL =*/
/*= if the specified device does not have a corresponding IDLEST register.   =*/
/*============================================================================*/

int prcm_get_initiators_not_standby(u32 domainid, u32 *result)
{
	static const u32 initiator_mask[PRCM_NUM_DOMAINS] = {
		IVA2_IMASK, MPU_IMASK, CORE1_IMASK, CORE2_IMASK,
#ifdef CONFIG_ARCH_OMAP3410
		0,
#else
		SGX_IMASK,
#endif
		WKUP_IMASK, DSS_IMASK, CAM_IMASK, PER_IMASK,
		0 /* EMU */, NEON_IMASK, CORE3_IMASK, USBHOST_IMASK,
	};
			
	FAIL_ON_INVALID_DOMAINID(domainid);

	if (prcm_get_devices_not_idle(domainid, result) != PRCM_PASS)
		return PRCM_FAIL;

	*result &= initiator_mask[domainid -1];
	return PRCM_PASS;
}

/*============================================================================*/
/*======================== SET POWER CONFIGURATION ===========================*/
/*============================================================================*/
/*= This function sets the power confiruration for all the modules in a power=*/
/*= domain by accessing the sysconfig register.                              =*/
/*= The modes set are standby, idle, and auto idle enable or disable.        =*/
/*============================================================================*/

int prcm_set_domain_power_configuration(u32 domainid, u8 idlemode,
					u8 standbymode, u8 autoidleenable)
{
	struct sysconf_reg *sysconf;
	volatile u32 *addr;
	volatile u32 *fclken_addr;
	volatile u32 *iclken_addr;

	u32 index;
	u32 device_id;
	u32 cm_fclken;
	u32 cm_iclken;
	u32 i_mask;

	u32 valid_func_clk;
	u32 valid_int_clk;
	u32 valid_idlest;
	u32 mask;

	int retries;
	u32 ret;

	FAIL_ON_INVALID_DOMAINID(domainid);

	switch (domainid) {
	case DOM_CORE1:
	case DOM_CORE2:
		/* Save the fclk/iclk registers */
		cm_fclken	= CM_FCLKEN1_CORE;
		cm_iclken	= CM_ICLKEN1_CORE;
		fclken_addr	= &CM_FCLKEN1_CORE;
		iclken_addr	= &CM_ICLKEN1_CORE;
		i_mask		= CORE1_IMASK;
		sysconf		= sysc_reg_core1;
		break;
	case DOM_WKUP:
		cm_fclken	= CM_FCLKEN_WKUP;
		cm_iclken	= CM_ICLKEN_WKUP;
		fclken_addr	= &CM_FCLKEN_WKUP;
		iclken_addr	= &CM_ICLKEN_WKUP;
		i_mask		= WKUP_IMASK;
		sysconf		= sysc_reg_wkup;
		break;
	case DOM_DSS:
		cm_fclken	= CM_FCLKEN_DSS;
		cm_iclken	= CM_ICLKEN_DSS;
		fclken_addr	= &CM_FCLKEN_DSS;
		iclken_addr	= &CM_ICLKEN_DSS;
		i_mask		= DSS_IMASK;
		sysconf		= sysc_reg_dss;
		break;
	case DOM_CAM:
		cm_fclken	= CM_FCLKEN_CAM;
		cm_iclken	= CM_ICLKEN_CAM;
		fclken_addr	= &CM_FCLKEN_CAM;
		iclken_addr	= &CM_ICLKEN_CAM;
		i_mask		= CAM_IMASK;
		sysconf		= sysc_reg_cam;
		break;
	case DOM_PER:
		cm_fclken	= CM_FCLKEN_PER;
		cm_iclken	= CM_ICLKEN_PER;
		fclken_addr	= &CM_FCLKEN_PER;
		iclken_addr	= &CM_ICLKEN_PER;
		i_mask		= PER_IMASK;
		sysconf		= sysc_reg_per;
		break;
	default:
		printk(KERN_INFO "Invalid domainid %d\n", domainid);
		return PRCM_FAIL;
	}

	/* enable Functional and interface clocks */
	valid_func_clk = get_val_bits(domainid, REG_FCLKEN);
	valid_int_clk  = get_val_bits(domainid, REG_ICLKEN);
	valid_idlest   = get_val_bits(domainid, REG_IDLEST);

	prcm_set_domain_functional_clocks(domainid, valid_func_clk);
	prcm_set_domain_interface_clocks(domainid, valid_int_clk);

	retries = 500;
	do {
		ret = prcm_get_devices_not_idle(domainid, &mask);
		if (ret == PRCM_FAIL)
			return ret;

		if ((mask | i_mask) == valid_idlest)
			break;

		udelay(1);
	} while (retries--);

	if (retries < 0) {
		printk(KERN_INFO "Timeout in set_power_configuration(), "
				"domain:%u  mask=%x, imask=%x, valid_idlest=%x\n",
				domainid, mask, i_mask, valid_idlest);
		return PRCM_FAIL;
	}

	for (index = 0; sysconf[index].reg_addr; index++) {
		addr      = sysconf[index].reg_addr;
		device_id = sysconf[index].device_id;

		DPRINTK("Domain is %d, Address is %p\n", domainid, addr);

		if (standbymode != PRCM_MIDLEMODE_DONTCARE) {
			 /* Setting the MIDLEMODE field */
			if (device_id == PRCM_SSI   || device_id == PRCM_SDMA ||
			    device_id == PRCM_HSOTG || device_id == PRCM_DISPC ||
#ifndef CONFIG_ARCH_OMAP3410
			    device_id == PRCM_CSIB  ||
#endif
			    device_id == PRCM_CAM   || device_id == PRCM_CSIA) {
				DPRINTK("MIDLEMODE SET\n");
				*addr |= (*addr & ~(PRCM_STANDBY_MASK)) |
					 (standbymode << PRCM_STANDBY_OFF);
				DPRINTK("TYPE %d *ADDR %x\n", INIT_TAR, *addr);
			}
		}


		if (idlemode != PRCM_SIDLEMODE_DONTCARE) {
			if (device_id == PRCM_SDRC && idlemode !=
							 PRCM_SMART_IDLE)
				printk("SDRC: Setting idle mode %u not"
						"supported\n", idlemode);


			/* Setting the SIDLEMODE field */
			if (device_id == PRCM_CAM ||
			    device_id == PRCM_HDQ ||
			    device_id == PRCM_MPU_INTC ||
#ifndef CONFIG_ARCH_OMAP3410
			    device_id == PRCM_CSIB ||
#endif
			    device_id == PRCM_CSIA)
				DPRINTK("SIDLEMODE cannot be set for %08x\n",
								 device_id);
			else
				*addr |= (*addr & ~(PRCM_IDLE_MASK)) |
					 (idlemode << PRCM_IDLE_OFF);

		}

		/* Setting the AUTOIDLE ENABLE field */
		if (device_id == PRCM_SDRC   || device_id == PRCM_MCBSP1 ||
		    device_id == PRCM_MCBSP2 || device_id == PRCM_MCBSP3 ||
		    device_id == PRCM_MCBSP4 || device_id == PRCM_MCBSP5) {
			DPRINTK("AUTOIDLEMODE cannot set for %08x\n", device_id);
		} else {
			*addr |= (*addr & ~(PRCM_AUTO_IDLE_MASK)) |
				 (autoidleenable << PRCM_AUTO_IDLE_OFF);
		}
	}

	/* Restore the fclk/iclk registers */
	*fclken_addr = cm_fclken;
	*iclken_addr = cm_iclken;

	return PRCM_PASS;
}

/******************************************************************************
 *
 * Set low power state for the whole chip
 *
 * Pre conditions for calling this API:
 *   a) Make sure that interrupts are disabled
 *   b) All initiators should be in standby mode
 *   c) System tick timer should be disabled
 * It is called from system wide suspend
 *
 ******************************************************************************/

static void wakeup_sources_activate(void)
{
	int i;
	int rc;

	printd("wakeup_sources_activate: enter \n");
	printd("##### INTCPS_MIR0       %08x\n", INTCPS_MIR0);
	printd("##### INTCPS_MIR1       %08x\n", INTCPS_MIR1);
	printd("##### INTCPS_MIR2       %08x\n", INTCPS_MIR2);
	printd("##### INTCPS_ITR0       %08x\n", INTCPS_ITR0);
	printd("##### INTCPS_ITR1       %08x\n", INTCPS_ITR1);
	printd("##### INTCPS_ITR2       %08x\n", INTCPS_ITR2);
	printd("##### PRM_IRQSTATUS_MPU %08x\n", PRM_IRQSTATUS_MPU);
	printd("##### PRM_IRQENABLE_MPU %08x\n", PRM_IRQENABLE_MPU);
 
	/* Mask all interrupts in MIR registers.
	 * Interrupts that are required for wakeup will be enabled below.
	 */
	INTCPS_MIR_SET0 = 0xffffffff;
	INTCPS_MIR_SET1 = 0xffffffff;
	INTCPS_MIR_SET2 = 0xffffffff;

	/* Clear MPU wake-up IRQ status.
	 */
	PRM_IRQENABLE_MPU = 0x00;
	PRM_IRQSTATUS_MPU = 0x3FFFFFD;

	/* Clear all group selection registers.
	 * Required ones will be set below.
	 */
	PM_MPUGRPSEL_WKUP	= 0x0;
	PM_MPUGRPSEL1_CORE	= 0x0;
	PM_MPUGRPSEL_PER	= 0x0;
	PM_MPUGRPSEL_USBHOST	= 0x0;
	PM_IVA2GRPSEL_WKUP	= 0x0;
	PM_IVA2GRPSEL1_CORE	= 0x0;
	PM_IVA2GRPSEL_PER	= 0x0;
	PM_IVA2GRPSEL_USBHOST	= 0x0;

	/* Disable all the wake-up sources of domains we don't care about.
	 */
	PM_WKEN_WKUP	= 0;
	PM_WKEN_DSS	= 0;
	PM_WKEN_PER	= 0;
	PM_WKEN_USBHOST	= 0;
	PM_WKEN1_CORE	= 0;
	PM_WKEN3_CORE	= 0;

	/* Enable select wake-up sources.
	 */
	PM_WKEN_WKUP		|= (1<<3); /* Enable Wake-Up for GPIO1 block */
	PM_MPUGRPSEL_WKUP	|= (1<<3); /* Enable Wake-Up Group for GPIO1 */
	INTCPS_MIR_CLEAR0	 = (1<<29);/* GPIO1 */

	PM_WKEN_WKUP		|= (1<<8); /* Enable Wake-Up for IO PAD */
	PM_MPUGRPSEL_WKUP	|= (1<<8); /* Enable Wake-Up Group for IO PAD */
	PRM_IRQENABLE_MPU	|= (1<<9); /* Enabling IO_PAD interrupts */ 
	INTCPS_MIR_CLEAR0	 = (1<<11);/* PRCM_MPU_IRQ */

	/* ES3.1 Change:
	 *   Need to enable the I/O Pad wake-up capability by triggering I/O
	 *   daisy chain control (Wu clock). This is done by setting the
	 *   PRCM.PM_WKEN_WKUP[16] EN_IO_CHAIN bit.
	 */
	if (is_sil_rev_greater_than(OMAP3430_REV_ES3_0)) {
		PM_WKEN_WKUP |= (1<<16);

		/* Wait for the I/O daisy chain to complete. */
		rc = WAIT_UNTIL(PM_WKST_WKUP & (1<<16), 500);
		if (rc < 0)
			printk("##### ERROR: I/O daisy chain did not complete.\n");

		/* Clear the status bit. */
		PM_WKST_WKUP &= ~(1<<16);
	}

#ifdef KEEP_UART3_ALIVE_DURING_SUSPEND
	/* If we keep on UART3 during suspend, we also make it a wake-up
	 * source.
	 */
	PM_WKEN_PER		|= (1<<11);
	PM_MPUGRPSEL_PER	|= (1<<11);
	INTCPS_MIR_CLEAR2	 = (1<<10);  // UART3 interrupt
#endif

	/* for all gpio banks in PER domain */
	for ( i=2; i<7; i++ ) 	{
		u32 wk = GPIOn_WAKEUPENABLE(i);
		if( wk ) {
			PM_WKEN_PER	  |= (1<<(13 +(i-2)));
			PM_MPUGRPSEL_PER  |= (1<<(13 +(i-2))); 
			prcm_set_wkup_dependency (DOM_MPU, PRCM_WKDEP_EN_PER );
			PRM_IRQENABLE_MPU |= 0x1;
			INTCPS_MIR_CLEAR0  = (1 << 11); // PRCM_MPU_IRQ
		}
	}

	printd("wakeup_sources_activate: done\n");
	{
		/* Note: GPIO indices are 1 based... */
	int i;
	for (i = 1; i < 7; i++)
		printd("##### GPIOn_WAKEUPENABLE(%d)  %08x\n",  i, GPIOn_WAKEUPENABLE(i));
	for (i = 1; i < 7; i++)
		printd("##### GPIOn_IRQENABLE1(%d)    %08x\n",  i, GPIOn_IRQENABLE1(i));
	for (i = 1; i < 7; i++)
		printd("##### GPIOn_RISINGDETECT(%d)  %08x\n",  i, GPIOn_RISINGDETECT(i));
	for (i = 1; i < 7; i++)
		printd("##### GPIOn_FALLINGDETECT(%d) %08x\n",  i, GPIOn_FALLINGDETECT(i));
	for (i = 1; i < 7; i++)
		printd("##### GPIOn_LEVELDETECT0(%d)  %08x\n",  i, GPIOn_LEVELDETECT0(i));
	for (i = 1; i < 7; i++)
		printd("##### GPIOn_LEVELDETECT1(%d)  %08x\n",  i, GPIOn_LEVELDETECT1(i));
	for (i = 1; i < 7; i++ )
		printd("##### GPIOn_IRQSTATUS1(%d)    %08x\n",  i, GPIOn_IRQSTATUS1(i));
	for (i = 1; i < 7; i++ )
		printd("##### GPIOn_DATAIN(%d)        %08x\n",  i, GPIOn_DATAIN(i));
	for (i = 1; i < 7; i++ )
		printd("##### GPIOn_DATAOUT(%d)       %08x\n",  i, GPIOn_DATAOUT(i));
	}
	printd("##### PM_MPUGRPSEL_WKUP     %08x\n", PM_MPUGRPSEL_WKUP);
	printd("##### PM_MPUGRPSEL1_CORE    %08x\n", PM_MPUGRPSEL1_CORE);
	printd("##### PM_MPUGRPSEL_PER      %08x\n", PM_MPUGRPSEL_PER);
	printd("##### PM_MPUGRPSEL_USBHOST  %08x\n", PM_MPUGRPSEL_USBHOST);
	printd("##### PM_IVA2GRPSEL_WKUP    %08x\n", PM_IVA2GRPSEL_WKUP);
	printd("##### PM_IVA2GRPSEL1_CORE   %08x\n", PM_IVA2GRPSEL1_CORE);
	printd("##### PM_IVA2GRPSEL_PER     %08x\n", PM_IVA2GRPSEL_PER);
	printd("##### PM_IVA2GRPSEL_USBHOST %08x\n", PM_IVA2GRPSEL_USBHOST);

	printd("##### INTCPS_MIR0 %08x\n", INTCPS_MIR0);
	printd("##### INTCPS_MIR1 %08x\n", INTCPS_MIR1);
	printd("##### INTCPS_MIR2 %08x\n", INTCPS_MIR2);
	printd("##### INTCPS_ITR0 %08x\n", INTCPS_ITR0);
	printd("##### INTCPS_ITR1 %08x\n", INTCPS_ITR1);
	printd("##### INTCPS_ITR2 %08x\n", INTCPS_ITR2);
	printd("##### INTCPS_PENDING_IRQ0 %08x\n", INTCPS_PENDING_IRQ0);
	printd("##### INTCPS_PENDING_IRQ1 %08x\n", INTCPS_PENDING_IRQ1);
	printd("##### INTCPS_PENDING_IRQ2 %08x\n", INTCPS_PENDING_IRQ2);
	printd("##### PRM_IRQSTATUS_MPU %08x\n", PRM_IRQSTATUS_MPU);
	printd("##### PRM_IRQENABLE_MPU %08x\n", PRM_IRQENABLE_MPU);
}

static void wakeup_sources_deactivate(void)
{
	PM_WKEN_WKUP &= ~(1<<3);  /* Disable GPIO1 bank */
	PM_WKEN_WKUP &= ~(1<<8);  /* Disable I/O pad */
	if (is_sil_rev_greater_than(OMAP3430_REV_ES3_0)) {
		PM_WKEN_WKUP &= ~(1<<16); /* Disable I/O daisy chain. */
	}
}

void clear_prepwstst(void)
{
	PM_PREPWSTST_MPU     = 0;
	PM_PREPWSTST_CORE    = 0;
	PM_PREPWSTST_IVA2    = 0;
	PM_PREPWSTST_PER     = 0;
	PM_PREPWSTST_DSS     = 0;
	PM_PREPWSTST_CAM     = 0;
	PM_PREPWSTST_SGX     = 0;
	PM_PREPWSTST_NEON    = 0;
	PM_PREPWSTST_USBHOST = 0;
}

int prcm_transition_domain_to(u32 domid, u8 state)
{
#ifdef CONFIG_HW_SUP_TRANS
	/* If we are using HW supported clock transitions, we need to put the
	 * domain into ON mode first, then we can put it into RET or OFF.
	 */
	prcm_set_clock_domain_state(domid, PRCM_NO_AUTO, PRCM_FALSE);

	if (DOM_NEON != domid) {
		prcm_clear_sleep_dependency(domid, PRCM_SLEEPDEP_EN_MPU);
		prcm_clear_wkup_dependency (domid, PRCM_WKDEP_EN_MPU);
	}
	prcm_set_power_domain_state(domid, PRCM_ON, PRCM_FORCE);
#endif
	return prcm_force_power_domain_state(domid, state);
}

int prcm_set_chip_power_mode(struct system_power_state *trg_st)
{
	struct system_power_state cur_st;
	u32 sdrc_power_register = 0x0;
	int ret;

	wkup_log_get_next_log();

//### Do we really need to do this. When we get here the state params should be fine.
	/* Check that parameters are valid */
	ret = check_power_mode_parameters(trg_st);
	if (ret != PRCM_PASS)
		return ret;

#ifdef DEBUG_SUSPEND
	prcm_debug("\nRegister dump before configuration:\n\n");
#endif

	/* Initiate save of padconf registers */
	CONTROL_PADCONF_OFF |= 0x02;
	ret = WAIT_UNTIL(CONTROL_GENERAL_PURPOSE_STATUS & 0x01, 1000);
	if (ret < 0) {
		printk(KERN_WARNING "Timeout saving pad configuration in %s\n",
				__FUNCTION__);
	}

	/* Save the current state of all domains.
	 */
	prcm_get_power_domain_state(DOM_IVA2, &cur_st.iva2_state);
	prcm_get_power_domain_state(DOM_DSS,  &cur_st.dss_state);
	prcm_get_power_domain_state(DOM_CAM,  &cur_st.cam_state);
	prcm_get_power_domain_state(DOM_PER,  &cur_st.per_state);
	prcm_get_power_domain_state(DOM_USBHOST, &cur_st.usbhost_state);
#ifndef CONFIG_ARCH_OMAP3410
	prcm_get_power_domain_state(DOM_NEON, &cur_st.neon_state);
	prcm_get_power_domain_state(DOM_SGX,  &cur_st.gfx_state);
#endif

	prcm_save_registers();

	/* Activate the wake-up sources. */
	wakeup_sources_activate();

	/* Force power domains to target state.
	 * IVA2 could be turned off by bridge code. Check if it is on. */
	if ((cur_st.iva2_state != PRCM_OFF) && (cur_st.iva2_state != PRCM_RET)) {
#ifdef CONFIG_RESET_IVA_IN_SUSPEND
		int rc;
		int idle_failed = 0;

		printk( KERN_NOTICE "IVA2 not in RET/OFF, forcing IDLE...\n");
		CM_CLKSTCTRL_IVA2  =  0x03;
		PM_PWSTCTRL_IVA2  &= ~0x03;

		/* Configure IVA to boot in idle mode */
		CONTROL_IVA2_BOOTMOD = 0x1;
		/* Assert reset for IVA2*/
		RM_RSTCTRL_IVA2      = 0x7;
		/* Clear reset of IVA2*/
		RM_RSTCTRL_IVA2      = 0x0;
		/* Clear reset status */
		RM_RSTST_IVA2        = 0xFFFFFFFF;

		/* Wait for the IVA2 to go idle and then to enter OFF.
		 */
		rc = WAIT_UNTIL(CM_IDLEST_IVA2 & (1<<0), 100000);
		if (rc < 0) {
			printk(KERN_NOTICE "IVA2 not idle after FORCE IDLE.\n");
			idle_failed++;
		}
		rc = WAIT_UNTIL((PM_PWSTST_IVA2 & 0x03) == 0x00, 100000);
		if (rc < 0) {
			printk(KERN_NOTICE "IVA2 not OFF after FORCE IDLE\n");
			idle_failed++;
		}
		printk(KERN_ERR "IVA2 FORCE IDLE %s\n", idle_failed ? "FAILED" : "OK");
#endif

		ret = prcm_force_power_domain_state(DOM_IVA2, trg_st->iva2_state);
		if (ret != PRCM_PASS)
			goto restore;
	} else {
//		printk( KERN_DEBUG "IVA2 is already in %s state. Not touching it.\n",
//			cur_st.iva2_state == PRCM_OFF ? "OFF" : "RET");
	}

#ifndef CONFIG_ARCH_OMAP3410
	if (cur_st.gfx_state == PRCM_ON) {
		ret = prcm_transition_domain_to(DOM_SGX, trg_st->gfx_state);
		if (ret != PRCM_PASS)
			goto restore;
	}
#endif

	if (cur_st.dss_state == PRCM_ON) {
		ret = prcm_transition_domain_to(DOM_DSS, trg_st->dss_state);
		if (ret != PRCM_PASS)
			goto restore;
	}

	if (cur_st.cam_state == PRCM_ON) {
		ret = prcm_transition_domain_to(DOM_CAM, trg_st->cam_state);
		if (ret != PRCM_PASS)
			goto restore;
	}

#ifndef KEEP_UART3_ALIVE_DURING_SUSPEND
	/* Only turn off the PER domain if we are not using UART3 for debugging.
	 */
	if (cur_st.per_state == PRCM_ON) {
		ret = prcm_transition_domain_to(DOM_PER, trg_st->per_state);
		if (ret != PRCM_PASS)
			goto restore;
	}
#endif

#ifndef CONFIG_ARCH_OMAP3410
	if (cur_st.neon_state == PRCM_ON) {
		ret = prcm_transition_domain_to(DOM_NEON, trg_st->neon_state);
		if (ret != PRCM_PASS)
			goto restore;
	}
#endif

	if (cur_st.usbhost_state == PRCM_ON) {
		ret = prcm_transition_domain_to(DOM_USBHOST, trg_st->usbhost_state);
		if (ret != PRCM_PASS)
			goto restore;
	}


#ifdef CONFIG_MPU_OFF
	/* On ES 2.0, if scratchpad is populated with valid pointer, warm reset
	 * does not work So we populate scratchpad restore address only in
	 * cpuidle and suspend calls.
	 */
	scratchpad_set_restore_addr();
#endif

	ret = prcm_prepare_mpu_core_domains(trg_st);

	/* Clear the domain RESET status registers.
	 */
	clear_domain_reset_status();

	/* Enable OMAPCTRL clock. */
	CM_ICLKEN1_CORE |= 0x40;
#ifdef KEEP_UART1_ALIVE_DURING_SUSPEND
	/* add UART1 for just $1.99 */
	CM_ICLKEN1_CORE = 0x02 | (1<<13);
#endif

	/* Clear the "previous power state" registers. */
	clear_prepwstst();

	/* Errata 1.107:
	 * After any Power on Reset, warm reset or wake-up reset, the ROM Code
	 * checks and sets the EMU Power domain in order to be able to check
	 * reading JTAG register if there is the need of performing any DFT
	 * test. When verifying the EMU power domain status, the ROM Code loops
	 * for eternity.
	 *
	 * Work-around:
	 * The applicative software must clear the D2DPOWERSTATE(bit8) bit in
	 * PM_PWSTST_EMU register.  (this is a reserved bit used by debug
	 * software for emulation purposes.).
	 */
	PM_PWSTST_EMU &= ~(1<<8);

	/* 
	 *  Save SDRC_POWER register to be restored later if required
	 */
	sdrc_power_register = SDRC_POWER_REG;

	/* - Enable self refresh on idle request.
	 * - Disable power down mode.
	 */
	SDRC_POWER_REG = 0xC1;

	/* Select the 32k clock as input for all GPTs to allow SYS_CLKREQ to
	 * de-assert.
	 * Note:
	 *   These registers have been saved in prcm_save_registers() and will
	 *   be restored after wake-up.
	 */
	CM_CLKSEL_WKUP &= ~0x01; /* GPT1 */
	CM_CLKSEL_PER   = 0x00;  /* GPT2-9 */
	CM_CLKSEL_CORE &= ~((1<<6) | (1<<7)); /* GPT10-11 */

	/* Set SmartIdle for SDRC */
	SDRC_SYSCONFIG = 0x10;

	/* Set the wakeup dependency for USBHOST to avoid a glitch
	 * on TLL SE0/DAT lines at wakeup.
	 */
	PM_WKDEP_USBHOST    = 0x10;

	/* Set the wakeup dependency for PER to avoid a glitch
	 * on GPIO lines at wakeup.
	 */
	PM_WKDEP_PER    = 0x10;

	/* We need to explicitely turn off the D2D clock here as it is being
	 * turned on automatically after OFF mode by the ROM code.
	 */
	CM_ICLKEN1_CORE &= ~(1<<3); /* D2D iclk */

	DUMP_REGS_BEFORE_SLEEP();

#if CONFIG_MACH_SIRLOIN_3630
	if( cpu_is_omap36xx() && is_omap3_rev_equal_to(OMAP3630_REV_ES1_1)) {
		/*  
		 * Errata 1.90. Override CKE1 padconfig in scratchpad memory
		 */
		__raw_writew(0x001f, IO_ADDRESS(0x48002834));
	}

	/*  Errata 1.102. RTA feature is not supported */
	/*  Enable RTA before suspend */
	CONTROL_MEM_RTA_CTRL = 1;
#endif

	/*   An alternative way to override CM_AUTOIDLE_PLL in scratchpad 
	 *   to reduce wakeup latency, (see scratchpad.c for more details) 
	 */
#if 0	  
	__raw_writel(0x1, IO_ADDRESS(0x48002950));
#endif

	/* Wait for clocks to shut down. */
	WAIT_WHILE(CM_IDLEST_CKGEN & 0xFFFFFFFE, 10000);

	wkup_log_suspend_clocks(); /* log clocks before suspend */

	/*
	 *
	 * DO SUSPEND: Jump to SRAM to execute idle
	 *
	 */
	if (ret == PRCM_PASS)
		omap_sram_idle(trg_st->mpu_state);

	/* Always restore an original value of SDRC_POWER_REG */
	SDRC_POWER_REG = sdrc_power_register;


#ifdef CONFIG_MACH_SIRLOIN_3630
	/*  Errata 1.102. RTA feature is not supported */
	/*  Disable RTA again */
	CONTROL_MEM_RTA_CTRL = 0;
#endif

	wkup_log_wakeup();  // log wakeup state 
	
	DUMP_REGS_AFTER_WAKEUP();


#ifdef CONFIG_MPU_OFF
	scratchpad_clr_restore_addr();
#endif

	prcm_restore_mpu_core_domains(trg_st);

restore:
	printd("##### restore:\n");

	/* Unset the wakeup dependency for USBHOST here.
	 * prcm_force_power_domain_state() below will send back
	 * the USBHOST domain to off.
	 *
	 * Note from TI:
	 * - When the USBHOST domain is woken up by a wakeup
	 *   dependency, there is no wakeup status logged in the PRCM.
	 * - The PRCM will not allow the USBHOST domain to go
	 *   back to a low power mode, until the wakeup has been
	 *   "acknowledged" in the PRM state machines.
	 * - Setting a sleep dependency between IVA/MPU and the USBHOST
	 *   domain, then clearing it will acknowledge the wakeup
	 * - This CLEAR_EVENT must be asserted for at least 2 system
	 *   clock cycles to be latched by the PRM state machine.
	 */
	PM_WKDEP_USBHOST = 0;
	CM_SLEEPDEP_USBHOST = 0x6;
	udelay(10);
	CM_SLEEPDEP_USBHOST = 0;

	/* Unset the wakeup dependency for PER here.
	 */
	PM_WKDEP_PER = 0;

	/* Force power domains back to original state */
	prcm_force_power_domain_state(DOM_IVA2,    cur_st.iva2_state);
#ifndef CONFIG_ARCH_OMAP3410
	prcm_force_power_domain_state(DOM_SGX,     cur_st.gfx_state);
#endif
	prcm_force_power_domain_state(DOM_USBHOST, cur_st.usbhost_state);
	prcm_force_power_domain_state(DOM_DSS,     cur_st.dss_state);
	prcm_force_power_domain_state(DOM_CAM,     cur_st.cam_state);
	prcm_force_power_domain_state(DOM_PER,     cur_st.per_state);
#ifndef CONFIG_ARCH_OMAP3410
	prcm_force_power_domain_state(DOM_NEON,    cur_st.neon_state);
#endif

	/* Disable wake-up sources. */
	wakeup_sources_deactivate();

	prcm_restore_registers();

	if (trg_st->core_state > PRCM_CORE_CSWR_MEMRET)
		prcm_restore_core_context(trg_st->core_state);

	CM_ICLKEN1_CORE |= 0x40;
	WAIT_WHILE(CM_IDLEST1_CORE & 0x40, 1000);

	wkup_log_wkup_gpio (); // log wakeup state of gpios

	/* Handle the case if we are using HW support clock transition.
	 */
#ifdef CONFIG_HW_SUP_TRANS
	if (cur_st.iva2_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_IVA2, PRCM_ON, PRCM_AUTO);
	if (cur_st.usbhost_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_USBHOST, PRCM_ON, PRCM_AUTO);
#ifndef CONFIG_ARCH_OMAP3410
	if (cur_st.gfx_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_SGX, PRCM_ON, PRCM_AUTO);
#endif
	if (cur_st.dss_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_DSS, PRCM_ON, PRCM_AUTO);
	if (cur_st.cam_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_CAM, PRCM_ON, PRCM_AUTO);
	if (cur_st.per_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_PER, PRCM_ON, PRCM_AUTO);
#ifndef CONFIG_ARCH_OMAP3410
	if (cur_st.neon_state == PRCM_ON)
		prcm_set_power_domain_state(DOM_NEON, PRCM_ON, PRCM_AUTO);
#endif
#endif

	if (trg_st->mpu_state > PRCM_MPU_INACTIVE) {
		u8 state;

		prcm_get_pre_power_domain_state(DOM_MPU, &state);
		if (trg_st->mpu_state == PRCM_MPU_OFF) {
			if (state != PRCM_OFF)
				ret = PRCM_FAIL;
		} else {
			if (state != PRCM_RET)
				ret = PRCM_FAIL;
		}
		if (ret == PRCM_FAIL) {
			printk(KERN_INFO "MPU did not enter target state %08x\n",
					trg_st->mpu_state);
			printk(KERN_INFO " -> Previous Power State: %x\n", 
			                PM_PREPWSTST_MPU);
			/* Reset previous power state registers and return
			 * error */

			 goto Fail;
		}
	}
	if (trg_st->core_state >= PRCM_CORE_CSWR_MEMRET) {
		u8 state;

		prcm_get_pre_power_domain_state(DOM_CORE1, &state);
		if (trg_st->core_state == PRCM_CORE_OFF) {
			if (state != PRCM_OFF)
				ret = PRCM_FAIL;
		} else {
			if (state != PRCM_RET)
				ret = PRCM_FAIL;
		}
		if (ret == PRCM_FAIL) {
			printk(KERN_INFO "CORE did not enter target state: %x\n",
					trg_st->core_state);
			printk(KERN_INFO " -> Previous Power State MPU:  %x\n"
					 " -> Previous Power State CORE: %x\n",
					PM_PREPWSTST_MPU, PM_PREPWSTST_CORE);
			/* Reset previous power state registers and return
			 * error */
			 goto Fail;
		}
	}
#ifdef DEBUG_SUSPEND
	prcm_debug("\nRegister dump after restore\n\n");
#endif

	// log wakeup event
	wkup_log_printk_suspend_event  ( wkup_log_idx, 0 );

#ifdef CONFIG_FASTPATH
	omap3_wakeup_sources_save();
#endif

	/**
	 * Not sure if we should clear the wakeup status, but
	 *  if we do, we need to do this after
	 *  wkup_log_printk_suspend_event & omap3_wakeup_sources_save
	 */
	// PM_WKST_WKUP |=  (1<<8);  /* Clear I/O daisy chain status. */

	return PRCM_PASS;

Fail:
	wkup_log_printk_suspend_event  ( wkup_log_idx, 1);

#ifdef CONFIG_FASTPATH
	omap3_wakeup_sources_save();
#endif
	return PRCM_FAIL;
}


