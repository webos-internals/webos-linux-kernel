/*
 * Copyright (C) 2005-2007 by Texas Instruments
 * Some code has been taken from tusb6010.c
 * Copyrights for that are attributable to:
 * Copyright (C) 2006 Nokia Corporation
 * Jarkko Nikula <jarkko.nikula@nokia.com>
 * Tony Lindgren <tony@atomide.com>
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/mach-types.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>

#include "musb_core.h"
#include "omap2430.h"

#ifdef CONFIG_ARCH_OMAP3430
#define	get_cpu_rev()	2
#endif

static int musb_enabled = 0;

#if !defined(CONFIG_ARCH_OMAP34XX)
#define MUSB_TIMEOUT_A_WAIT_BCON	1100

static struct timer_list musb_idle_timer;

static void musb_do_idle(unsigned long _musb)
{
	struct musb	*musb = (void *)_musb;
	unsigned long	flags;
//	u8	power;
	u8	devctl;

	spin_lock_irqsave(&musb->lock, flags);

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	switch (musb->xceiv.state) {
	case OTG_STATE_A_WAIT_BCON:
		devctl &= ~MUSB_DEVCTL_SESSION;
		musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl & MUSB_DEVCTL_BDEVICE) {
			musb->xceiv.state = OTG_STATE_B_IDLE;
			MUSB_DEV_MODE(musb);
		} else {
			musb->xceiv.state = OTG_STATE_A_IDLE;
			MUSB_HST_MODE(musb);
		}
		break;
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case OTG_STATE_A_SUSPEND:
		/* finish RESUME signaling? */
		if (musb->port1_status & MUSB_PORT_STAT_RESUME) {
			power = musb_readb(musb->mregs, MUSB_POWER);
			power &= ~MUSB_POWER_RESUME;
			DBG(1, "root port resume stopped, power %02x\n", power);
			musb_writeb(musb->mregs, MUSB_POWER, power);
			musb->is_active = 1;
			musb->port1_status &= ~(USB_PORT_STAT_SUSPEND
						| MUSB_PORT_STAT_RESUME);
			musb->port1_status |= USB_PORT_STAT_C_SUSPEND << 16;
			usb_hcd_poll_rh_status(musb_to_hcd(musb));
			/* NOTE: it might really be A_WAIT_BCON ... */
			musb->xceiv.state = OTG_STATE_A_HOST;
		}
		break;
#endif
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case OTG_STATE_A_HOST:
		devctl = musb_readb(musb->mregs, MUSB_DEVCTL);
		if (devctl &  MUSB_DEVCTL_BDEVICE)
			musb->xceiv.state = OTG_STATE_B_IDLE;
		else
			musb->xceiv.state = OTG_STATE_A_WAIT_BCON;
#endif
	default:
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);
}


void musb_platform_try_idle(struct musb *musb, unsigned long timeout)
{
	unsigned long		default_timeout = jiffies + msecs_to_jiffies(3);
	static unsigned long	last_timer;

	if (timeout == 0)
		timeout = default_timeout;

	/* Never idle if active, or when VBUS timeout is not set as host */
	if (musb->is_active || ((musb->a_wait_bcon == 0)
			&& (musb->xceiv.state == OTG_STATE_A_WAIT_BCON))) {
		DBG(4, "%s active, deleting timer\n", otg_state_string(musb));
		del_timer(&musb_idle_timer);
		last_timer = jiffies;
		return;
	}

	if (time_after(last_timer, timeout)) {
		if (!timer_pending(&musb_idle_timer))
			last_timer = timeout;
		else {
			DBG(4, "Longer idle timer already pending, ignoring\n");
			return;
		}
	}
	last_timer = timeout;

	DBG(4, "%s inactive, for idle timer for %lu ms\n",
		otg_state_string(musb),
		(unsigned long)jiffies_to_msecs(timeout - jiffies));
	mod_timer(&musb_idle_timer, timeout);
}
#endif

void musb_platform_enable(struct musb *musb)
{
	musb_enabled = 1;
}
void musb_platform_disable(struct musb *musb)
{
	musb_enabled = 0;
}
static void omap_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void omap_set_vbus(struct musb *musb, int is_on)
{
	u8		devctl;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv.default_a = 0;
		musb->xceiv.state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

#if 0
static int omap_set_power(struct otg_transceiver *x, unsigned mA)
{
	return 0;
}
#endif

int musb_platform_resume(struct musb *musb);

void musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
	u8	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	devctl |= MUSB_DEVCTL_SESSION;
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	switch (musb_mode) {
	case MUSB_HOST:
		otg_set_host(&musb->xceiv, musb->xceiv.host);
		break;
	case MUSB_PERIPHERAL:
		otg_set_peripheral(&musb->xceiv, musb->xceiv.gadget);
		break;
	case MUSB_OTG:
		break;
	}
}

static struct musb *the_musb;

int __init musb_platform_init(struct musb *musb)
{
	struct otg_transceiver *xceiv = otg_get_transceiver();

#if defined(CONFIG_ARCH_OMAP2430)
	omap_cfg_reg("AE5_2430_USB0HS_STP");
	/* get the clock */
	musb->clock = clk_get((struct device *)musb->controller, "usbhs_ick");
#else
	musb->clock = clk_get((struct device *)musb->controller, "hsotgusb_ick");
#endif
	if(IS_ERR(musb->clock))
		return PTR_ERR(musb->clock);

	musb->asleep = 0;

	/* The i-clk is AUTO gated. Hence there is no need
	 * to disable it until the driver is shutdown */

	clk_enable(musb->clock);

	musb->xceiv = *xceiv;
	//musb_platform_resume(musb); /* REVISIT */

	the_musb = musb; /* REVISIT */

	OTG_SYSCONFIG_REG  &= ~ENABLEWAKEUP;/* disable wakeup */
        OTG_FORCESTDBY_REG &= ~ENABLEFORCE; /* disable MSTANDBY */
        OTG_SYSCONFIG_REG &= ~NOSTDBY;      /* remove possible NOSTDBY */
        OTG_SYSCONFIG_REG |= SMARTSTDBY;    /* enable smart standby */
        OTG_SYSCONFIG_REG &= ~AUTOIDLE;     /* disable auto idle */
        OTG_SYSCONFIG_REG &= ~NOIDLE;       /* remove possible NOIDLE */
        OTG_SYSCONFIG_REG |= SMARTIDLE;     /* enable smart idle */
//      OTG_SYSCONFIG_REG |= AUTOIDLE;      /* enable auto idle */
// 	OTG_SYSCONFIG_REG |= ENABLEWAKEUP;  /* enable wakeup */
	OTG_INTERFSEL_REG  = ULPI_12PIN;

	printk(KERN_INFO "HS USB OTG: revision 0x%x, sysconfig 0x%02x, "
			"sysstatus 0x%x, intrfsel 0x%x, simenable  0x%x\n",
			OTG_REVISION_REG, OTG_SYSCONFIG_REG, OTG_SYSSTATUS_REG,
			OTG_INTERFSEL_REG, OTG_SIMENABLE_REG);

	omap_vbus_power(musb, musb->board_mode == MUSB_HOST, 1);

	if (is_host_enabled(musb))
		musb->board_set_vbus = omap_set_vbus;
#if 0
	/* we should not override this */
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = omap_set_power;
#endif
#if 0
	musb->a_wait_bcon = MUSB_TIMEOUT_A_WAIT_BCON;

	setup_timer(&musb_idle_timer, musb_do_idle, (unsigned long) musb);
#endif

	return 0;
}

static int musb_context_save(struct musb *musb);
static void musb_context_restore(void);

int musb_platform_suspend(struct musb *musb)
{
	if (!musb->clock)
		return 0;

//	printk("@@ %s: enter\n", __func__);

	musb_context_save(musb);

	/* Configure SYSCONFIG:
	 *     Force Standby Master  (0 << 12)
	 *     Force Idle Slave      (0 <<  3)
	 *     Autoidle clocks       (1 <<  0)
	 */
	OTG_SYSCONFIG_REG  &= ~ENABLEWAKEUP;          /* disable wakeup */
        OTG_FORCESTDBY_REG &= ~ENABLEFORCE;           /* disable MSTANDBY */
        OTG_SYSCONFIG_REG &= ~(SMARTSTDBY | NOSTDBY); /* enable force standby */
        OTG_SYSCONFIG_REG &= ~AUTOIDLE;               /* disable auto idle */
        OTG_SYSCONFIG_REG &= ~(NOIDLE | SMARTIDLE);   /* enable force idle */
        OTG_FORCESTDBY_REG |= ENABLEFORCE;            /* enable MSTANDBY   */
//      OTG_SYSCONFIG_REG |= AUTOIDLE;                /* enable auto idle  */

	if (musb->xceiv.set_suspend)
		musb->xceiv.set_suspend(&musb->xceiv, 1);

#if 0
	/* Clocks are autoidled. No need to explicitly disable */
	if (musb->set_clock)
		musb->set_clock(musb->clock, 0);
	else
		clk_disable(musb->clock);
#endif
	musb->asleep = 1;

//	printk("@@ %s: finished\n", __func__);

	return 0;
}

int musb_platform_resume(struct musb *musb)
{
	if (!musb->clock)
		return 0;

//	printk("@@ %s: enter\n", __func__);

	if (musb->xceiv.set_suspend)
		musb->xceiv.set_suspend(&musb->xceiv, 0);

	musb_context_restore();

#if 0
	/* Clocks are autoidled. No need to explicitly disable */
	if (musb->set_clock)
		musb->set_clock(musb->clock, 1);
	else
		clk_enable(musb->clock);
#endif

	/* Configure SYSCONFIG:
	 *     Smart Standby Master
	 *     Smart Idle Slave
	 *     Autoidle clocks
	 */
        OTG_FORCESTDBY_REG &= ~ENABLEFORCE; /* disable MSTANDBY */
        OTG_SYSCONFIG_REG &= ~NOSTDBY;      /* remove possible NOSTDBY */
        OTG_SYSCONFIG_REG |= SMARTSTDBY;    /* enable smart standby */
        OTG_SYSCONFIG_REG &= ~AUTOIDLE;     /* disable auto idle */
        OTG_SYSCONFIG_REG &= ~NOIDLE;       /* remove possible NOIDLE */
        OTG_SYSCONFIG_REG |= SMARTIDLE;     /* enable smart idle */
//      OTG_SYSCONFIG_REG |= AUTOIDLE;      /* enable auto idle */
// 	OTG_SYSCONFIG_REG |= ENABLEWAKEUP;  /* enable wakeup */

	musb->asleep = 0;

//	printk("@@ %s: finished\n", __func__);

	return 0;
}


int musb_platform_exit(struct musb *musb)
{

	omap_vbus_power(musb, 0 /*off*/, 1);

	musb_platform_suspend(musb);

	OTG_SYSCONFIG_REG &= ~ENABLEWAKEUP; /* Disable Wakeup */

	clk_put(musb->clock);
	musb->clock = 0;

	return 0;
}

int musb_otg_state(enum usb_otg_state *state)
{
	struct musb *musb = the_musb;
	unsigned long	flags;

	if (!musb)
		return -1;

	spin_lock_irqsave(&musb->lock, flags);
	*state = musb->xceiv.state;
	spin_unlock_irqrestore(&musb->lock, flags);
	return 0;
}

void musb_pullup(struct musb *musb, int is_on);

void musb_softconn(int is_on)
{
	struct musb *musb = the_musb;
	unsigned long	flags;

	/*
	 *  Do not enable pullup if musb is not started.
	 *  Somehow it prevents it entering standby mode which prevents C5 
	 */
	if (!musb || !musb_enabled)
		return;

	printk(KERN_INFO "%s: %s\n", __func__, is_on ? "on" : "off");

	spin_lock_irqsave(&musb->lock, flags);
	musb_pullup(musb, is_on);
	spin_unlock_irqrestore(&musb->lock, flags);
}

#if defined(CONFIG_OMAP34XX_OFFMODE)
struct musb_common_regs{
	/* common registers */
	u8 	faddr;
	u8	power;
	u16 	intrtx;
	u16 	intrrx;
	u16 	intrtxe;
	u16 	intrrxe;
	u8 	intrusbe;
	u8 	intrusb;
	u8	devctl;
};

struct musb_index_regs{
	/* Fifo registers */
	u8	rxfifosz;
	u8	txfifosz;
	u16	txfifoadd;
	u16	rxfifoadd;
} musb_index_regs_t;

struct musb_context{
	/* MUSB init context */
	struct musb_common_regs	common_regs;
	struct musb_index_regs 	index_regs[MUSB_C_NUM_EPS];
};


/* Global: MUSB Context data pointer */
struct musb_context *context_ptr = NULL;
struct musb *musb_ptr = NULL;

/* Context Save/Restore for OFF mode */
static int musb_context_save(struct musb *musb)
{
	u8 i;

	/* Save MUSB Context only once */
	if (!context_ptr) {
		context_ptr = kzalloc(sizeof(struct musb_context),
				      GFP_KERNEL);
		if (!context_ptr)
			return -ENOMEM;
	}

	/* Save musb ptr */
	musb_ptr = musb;

	/* Save Common registers */
	context_ptr->common_regs.faddr = musb_readb(musb_ptr->mregs, MUSB_FADDR);
	context_ptr->common_regs.power = musb_readb(musb_ptr->mregs, MUSB_POWER);
	context_ptr->common_regs.intrtx = musb_readw(musb_ptr->mregs, MUSB_INTRTX);
	context_ptr->common_regs.intrrx = musb_readw(musb_ptr->mregs, MUSB_INTRRX);
	context_ptr->common_regs.intrtxe = musb_readw(musb_ptr->mregs, MUSB_INTRTXE);
	context_ptr->common_regs.intrrxe = musb_readw(musb_ptr->mregs, MUSB_INTRRXE);
	context_ptr->common_regs.intrusbe = musb_readb(musb_ptr->mregs, MUSB_INTRUSBE);
	context_ptr->common_regs.intrusb = musb_readb(musb_ptr->mregs, MUSB_INTRUSB);
	context_ptr->common_regs.devctl = musb_readb(musb_ptr->mregs, MUSB_DEVCTL);

		/* Save FIFO setup details */
	for (i = 0; i < MUSB_C_NUM_EPS; i++) {
		musb_writeb(musb_ptr->mregs, MUSB_INDEX, i);
		context_ptr->index_regs[i].rxfifosz = musb_readb(musb_ptr->mregs,
								 MUSB_RXFIFOSZ);
		context_ptr->index_regs[i].txfifosz = musb_readb(musb_ptr->mregs,
								 MUSB_TXFIFOSZ);
		context_ptr->index_regs[i].txfifoadd = musb_readw(musb_ptr->mregs,
								  MUSB_TXFIFOADD);
		context_ptr->index_regs[i].rxfifoadd = musb_readw(musb_ptr->mregs,
								  MUSB_RXFIFOADD);
	}

	/* Reset the controller and keep in default state on power-up */
	OTG_SYSCONFIG_REG |= SOFTRST;

	return 0;
}

static void musb_context_restore(void)
{
	u8 i;

	/* Set System specific registers
	 * Init controller
	 */
//	OTG_SYSCONFIG_REG |= ENABLEWAKEUP;
	OTG_INTERFSEL_REG |= ULPI_12PIN;

	/* Restore MUSB specific registers */

	/* Restore: MUSB Common regs */
	musb_writeb(musb_ptr->mregs, MUSB_FADDR, context_ptr->common_regs.faddr);
	musb_writeb(musb_ptr->mregs, MUSB_POWER, context_ptr->common_regs.power);
	musb_writew(musb_ptr->mregs, MUSB_INTRTX, context_ptr->common_regs.intrtx);
	musb_writew(musb_ptr->mregs, MUSB_INTRRX, context_ptr->common_regs.intrrx);
	musb_writew(musb_ptr->mregs, MUSB_INTRTXE, context_ptr->common_regs.intrtxe);
	musb_writew(musb_ptr->mregs, MUSB_INTRRXE, context_ptr->common_regs.intrrxe);
	musb_writeb(musb_ptr->mregs, MUSB_INTRUSBE, context_ptr->common_regs.intrusbe);
	musb_writeb(musb_ptr->mregs, MUSB_INTRUSB, context_ptr->common_regs.intrusb);
	musb_writeb(musb_ptr->mregs, MUSB_DEVCTL, context_ptr->common_regs.devctl);

	/* Restore: FIFO setup details */
	for (i = 0; i < MUSB_C_NUM_EPS; i++) {
		musb_writeb(musb_ptr->mregs, MUSB_INDEX, i);
		musb_writeb(musb_ptr->mregs, MUSB_RXFIFOSZ,
					context_ptr->index_regs[i].rxfifosz);
		musb_writeb(musb_ptr->mregs, MUSB_TXFIFOSZ,
					context_ptr->index_regs[i].txfifosz);
		musb_writew(musb_ptr->mregs, MUSB_TXFIFOADD,
					context_ptr->index_regs[i].txfifoadd);
		musb_writew(musb_ptr->mregs, MUSB_RXFIFOADD,
					context_ptr->index_regs[i].rxfifoadd);
	}
}
#endif /* CONFIG_OMAP34XX_OFFMODE */
