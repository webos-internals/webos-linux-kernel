/*
 * OHCI HCD (Host Controller Driver) for USB.
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2005 David Brownell
 * (C) Copyright 2002 Hewlett-Packard Company
 * (C) Copyright 2007 Texas Instruments
 * (C) Copyright 2007 Vikram Pandita <vikram.pandita@ti.com>
 * (C) Copyright 2008 Romit Dasgupta <romit@ti.com>
 *
 * OMAP Bus Glue
 *
 * Modified for OMAP by Tony Lindgren <tony@atomide.com>
 * Based on the 2.4 OMAP OHCI driver originally done by MontaVista Software Inc.
 * and on ohci-sa1111.c by Christopher Hoover <ch@hpl.hp.com>
 *
 * This file is licenced under the GPL.
 */

#include <linux/signal.h>	/* IRQF_DISABLED */
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/mux.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/fpga.h>
#include <asm/arch/usb.h>

#include "ehci-omap.h"

#ifndef CONFIG_ARCH_OMAP
#error "This file is OMAP bus glue.  CONFIG_OMAP must be defined."
#endif

//Debugging Only
//#define KERN_DEBUG KERN_INFO

//#undef pr_debug
//#define pr_debug(fmt,arg...) printk(fmt,##arg)

extern int usb_disabled(void);
extern int ocpi_enable(void);

/* Define USBHOST clocks for clock management */
struct ohci_omap_clock_defs {
	struct clk	*usbhost_ick_clk;
	struct clk	*usbhost2_120m_fck_clk;
	struct clk	*usbhost1_48m_fck_clk;
	struct clk	*usbtll_fck_clk;
	struct clk	*usbtll_ick_clk;
	unsigned	suspended:1;
	/*
	 * TODO:
	 * host_enabled should be put in separate place.
	 */
	unsigned	host_enabled:1;
};

/* Clock names as per clock framework: May change so keep as #defs */
#define USBHOST_ICLK            "usbhost_ick"
#define USBHOST_120M_FCLK       "usbhost_120m_fck"
#define USBHOST_48M_FCLK        "usbhost_48m_fck"
#define USBHOST_TLL_ICLK        "usbtll_ick"
#define USBHOST_TLL_FCLK        "usbtll_fck"

#ifdef CONFIG_ARCH_OMAP15XX
	/* deleted for cleanup of code */
#else
#define omap_1510_local_bus_power(x)	{}
#define omap_1510_local_bus_init()	{}
#endif

/*-------------------------------------------------------------------------*/

static int ohci_omap_init(struct usb_hcd *hcd)
{
	struct ohci_hcd		*ohci = hcd_to_ohci(hcd);

	int			need_transceiver = 0; //(config->otg != 0);
	int			ret;

	dev_dbg(hcd->self.controller, "starting USB Controller\n");

	/* boards can use OTG transceivers in non-OTG modes */
	need_transceiver = need_transceiver
			|| machine_is_omap_h2() || machine_is_omap_h3();

	if (cpu_is_omap16xx())
		ocpi_enable();

/* #ifdef	CONFIG_ARCH_OMAP_OTG */
	if (need_transceiver) {
		ohci->transceiver = otg_get_transceiver();
		if (ohci->transceiver) {
			int	status = otg_set_host(ohci->transceiver,
						&ohci_to_hcd(ohci)->self);
			dev_dbg(hcd->self.controller, "init %s transceiver, status %d\n",
					ohci->transceiver->label, status);
			if (status) {
				if (ohci->transceiver)
					put_device(ohci->transceiver->dev);
				return status;
			}
		} else {
			dev_err(hcd->self.controller, "can't find transceiver\n");
			return -ENODEV;
		}
	}
/* #endif */

	if (cpu_is_omap15xx()) {
		omap_1510_local_bus_power(1);
		omap_1510_local_bus_init();
	}

	if ((ret = ohci_init(ohci)) < 0)
		return ret;

	/* board-specific power switching and overcurrent support */
	if (machine_is_omap_osk() || machine_is_omap_innovator()) {
		u32	rh = roothub_a(ohci);

		/* power switching (ganged by default) */
		rh &= ~RH_A_NPS;

		/* TPS2045 switch for internal transceiver (port 1) */
		if (machine_is_omap_osk()) {
			ohci_to_hcd(ohci)->power_budget = 250;

			rh &= ~RH_A_NOCP;

			/* gpio9 for overcurrent detction */
			omap_cfg_reg("W8_1610_GPIO9");
			omap_request_gpio(9);
			omap_set_gpio_direction(9, 1); /* IN */

			/* for paranoia's sake:  disable USB.PUEN */
			omap_cfg_reg("W4_USB_HIGHZ");
		}
		ohci_writel(ohci, rh, &ohci->regs->roothub.a);
		distrust_firmware = 0;
	} else if (machine_is_nokia770()) {
		/* We require a self-powered hub, which should have
		 * plenty of power. */
		ohci_to_hcd(ohci)->power_budget = 0;
	}

	/* board init will have already handled HMC and mux setup.
	 * any external transceiver should already be initialized
	 * too, so all configured ports use the right signaling now.
	 */

	return 0;
}

static void ohci_omap_stop(struct usb_hcd *hcd)
{
	dev_dbg(hcd->self.controller, "stopping USB Controller\n");
}


/*-------------------------------------------------------------------------*/
/**
 * usb_hcd_omap_probe - initialize OMAP-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 */
static int usb_hcd_omap_probe(const struct hc_driver *driver,
			  struct platform_device *pdev)
{
	int retval;
	int i;
	u32 uhh_hostconfig_value;
	u8 ohci_port_enable_mask = 0;
	struct usb_hcd *hcd = 0;
	struct ohci_hcd *ohci;
	struct ohci_omap_clock_defs *ohci_clocks;

	if (pdev->num_resources != 2) {
		printk(KERN_ERR "hcd probe: invalid num_resources: %i\n",
		       pdev->num_resources);
		return -ENODEV;
	}

	if (pdev->resource[0].flags != IORESOURCE_MEM
			|| pdev->resource[1].flags != IORESOURCE_IRQ) {
		printk(KERN_ERR "hcd probe: invalid resource type\n");
		return -ENODEV;
	}

	/* Enable Func Mux for 4-pin mode with ISP1301 on Port1 and Port3*/
#if CONFIG_OMAP_OHCI_TLL_MODE_PINS == 2
	pr_debug("OHCI-TLL mode 2-pin\n");
        /* Set Func mux for : */
	omap_cfg_reg("AG12_3430_USB1FS_TLL_MM1_TXSE0");
	omap_cfg_reg("AH12_3430_USB1FS_TLL_MM1_TXDAT");
#elif CONFIG_OMAP_OHCI_TLL_MODE_PINS == 3
	pr_debug("\n OHCI-TLL mode 3-pin\n");
        /* Set Func mux for : */
	omap_cfg_reg("AG12_3430_USB1FS_TLL_MM1_TXSE0");
	omap_cfg_reg("AH12_3430_USB1FS_TLL_MM1_TXDAT");
	omap_cfg_reg("AH14_3430_USB1FS_TLL_MM1_TXEN_N");
#else
#error	Not implemented
#endif

	hcd = usb_create_hcd (driver, &pdev->dev, pdev->dev.bus_id);
	if (!hcd) {
		retval = -ENOMEM;
		goto err0;
	}

	ohci_clocks = (struct ohci_omap_clock_defs *)
			(((char *)hcd_to_ohci(hcd)) + sizeof(struct ohci_hcd));

	/* Enable Clocks for USBHOST */
	ohci_clocks->usbhost_ick_clk = clk_get(&pdev->dev,
						USBHOST_ICLK);
	if (IS_ERR(ohci_clocks->usbhost_ick_clk))
		return PTR_ERR(ohci_clocks->usbhost_ick_clk);
	clk_enable(ohci_clocks->usbhost_ick_clk);

	ohci_clocks->usbhost2_120m_fck_clk = clk_get(&pdev->dev,
						USBHOST_120M_FCLK);
	if (IS_ERR(ohci_clocks->usbhost2_120m_fck_clk)) {
		clk_disable(ohci_clocks->usbhost_ick_clk);
		clk_put(ohci_clocks->usbhost_ick_clk);
		return PTR_ERR(ohci_clocks->usbhost2_120m_fck_clk);
	}
	clk_enable(ohci_clocks->usbhost2_120m_fck_clk);

	ohci_clocks->usbhost1_48m_fck_clk = clk_get(&pdev->dev,
						USBHOST_48M_FCLK);
	if (IS_ERR(ohci_clocks->usbhost1_48m_fck_clk)) {
		clk_disable(ohci_clocks->usbhost_ick_clk);
		clk_put(ohci_clocks->usbhost_ick_clk);
		clk_disable(ohci_clocks->usbhost2_120m_fck_clk);
		clk_put(ohci_clocks->usbhost2_120m_fck_clk);
		return PTR_ERR(ohci_clocks->usbhost1_48m_fck_clk);
	}
	clk_enable(ohci_clocks->usbhost1_48m_fck_clk);

	/* Configure TLL for 60Mhz clk for ULPI */
	ohci_clocks->usbtll_fck_clk = clk_get(&pdev->dev,
						USBHOST_TLL_FCLK);
	if (IS_ERR(ohci_clocks->usbtll_fck_clk)) {
		clk_disable(ohci_clocks->usbhost_ick_clk);
		clk_put(ohci_clocks->usbhost_ick_clk);
		clk_disable(ohci_clocks->usbhost2_120m_fck_clk);
		clk_put(ohci_clocks->usbhost2_120m_fck_clk);
		clk_disable(ohci_clocks->usbhost1_48m_fck_clk);
		clk_put(ohci_clocks->usbhost1_48m_fck_clk);
		return PTR_ERR(ohci_clocks->usbtll_fck_clk);
	}
	clk_enable(ohci_clocks->usbtll_fck_clk);

	ohci_clocks->usbtll_ick_clk = clk_get(&pdev->dev,
						USBHOST_TLL_ICLK);
	if (IS_ERR(ohci_clocks->usbtll_ick_clk)) {
		clk_disable(ohci_clocks->usbhost_ick_clk);
		clk_put(ohci_clocks->usbhost_ick_clk);
		clk_disable(ohci_clocks->usbhost2_120m_fck_clk);
		clk_put(ohci_clocks->usbhost2_120m_fck_clk);
		clk_disable(ohci_clocks->usbhost1_48m_fck_clk);
		clk_put(ohci_clocks->usbhost1_48m_fck_clk);
		clk_disable(ohci_clocks->usbtll_fck_clk);
		clk_put(ohci_clocks->usbtll_fck_clk);
		return PTR_ERR(ohci_clocks->usbtll_ick_clk);
	}
	clk_enable(ohci_clocks->usbtll_ick_clk);

	ohci_clocks->suspended = 0;

#if 0 /* REVISIT */
	printk(KERN_DEBUG "%s %d\n", __func__, __LINE__);
	/* Disable Auto Idle of USBTLL */
	CM_AUTOIDLE3_CORE = (0 << OMAP3430_AUTO_USBTLL_SHIFT);

	/* Wait for TLL to be Active */
	while (CM_IDLEST3_CORE & (1 << OMAP3430_ST_USBTLL_SHIFT));
	printk(KERN_INFO "%s %d\n", __func__, __LINE__);
#endif

	/* perform TLL soft reset, and wait until reset is complete */
	omap_writel(1 << OMAP_USBTLL_SYSCONFIG_SOFTRESET_SHIFT,
				OMAP_USBTLL_SYSCONFIG);
	/* Wait for TLL reset to complete */
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS) &
			(1 << OMAP_USBTLL_SYSSTATUS_RESETDONE_SHIFT)));

	/* smart idle mode */
	omap_writel((1 << OMAP_USBTLL_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(2 << OMAP_USBTLL_SYSCONFIG_SIDLEMODE_SHIFT) |
			(0 << OMAP_USBTLL_SYSCONFIG_CACTIVITY_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_AUTOIDLE_SHIFT),
						OMAP_USBTLL_SYSCONFIG);

	/* Put UHH in SmartIdle/SmartStandby mode */
	omap_writel((1 << OMAP_UHH_SYSCONFIG_AUTOIDLE_SHIFT) |
			(0 << OMAP_UHH_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(2 << OMAP_UHH_SYSCONFIG_SIDLEMODE_SHIFT) |
			(0 << OMAP_UHH_SYSCONFIG_CACTIVITY_SHIFT) |
			(2 << OMAP_UHH_SYSCONFIG_MIDLEMODE_SHIFT),
						OMAP_UHH_SYSCONFIG);

	/* TLL in FS-TLL mode operation */
	uhh_hostconfig_value = (1 << OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN_SHIFT) |
			(1 << OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN_SHIFT) |
			(1 << OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN_SHIFT) |
			(0 << OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN_SHIFT);

	uhh_hostconfig_value |=
			(1 << OMAP_UHH_HOSTCONFIG_P1_ULPI_BYPASS_SHIFT);
	omap_writel(uhh_hostconfig_value, OMAP_UHH_HOSTCONFIG);

	pr_debug("Entered TLL MODE: success\n");

	/* Program Common TLL register */
	omap_writel((1 << OMAP_TLL_SHARED_CONF_FCLK_IS_ON_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_DIVRATION_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN_SHIFT) |
			(0 << OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN_SHFT),
				OMAP_TLL_SHARED_CONF);

	ohci_port_enable_mask |= (1 << 0);

	for (i = 0; i < OMAP_TLL_CHANNEL_COUNT; i++) {

		/* Enable only required ports */
		if (!(ohci_port_enable_mask & (1 << i)))
			continue;

		/* CHANMODE: UTMI-to-serial FS/LS mode */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(1 << OMAP_TLL_CHANNEL_CONF_CHANMODE_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

		/* UTMIISADEV: UTMI side is host */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(1 << OMAP_TLL_CHANNEL_CONF_UTMIISADEV_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

		/* TLLATTACH: cable attach emulation */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(1 << OMAP_TLL_CHANNEL_CONF_TLLATTACH_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

		/* TLLCONNECT: Full/Low-speed connect emulation */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(1 << OMAP_TLL_CHANNEL_CONF_TLLCONNECT_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

		/* TLLFULLSPEED: Full-Speed emulation (D+ pullup) */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(1 << OMAP_TLL_CHANNEL_CONF_TLLFULLSPEED_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));

#if CONFIG_OMAP_OHCI_TLL_MODE_PINS == 2
		/* FSLSMODE: 2-pin TLL */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(0xa << OMAP_TLL_CHANNEL_CONF_FSLSMODE_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));
#elif CONFIG_OMAP_OHCI_TLL_MODE_PINS == 3
		/* FSLSMODE: 3-pin TLL */
		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			(6 << OMAP_TLL_CHANNEL_CONF_FSLSMODE_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));
#endif

		omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(i)) |
			    (1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT),
			    OMAP_TLL_CHANNEL_CONF(i));
	}

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	/*
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_dbg(&pdev->dev, "request_mem_region failed\n");
		retval = -EBUSY;
		goto err1;
	}
	 */

	hcd->regs = (void __iomem *) (int) IO_ADDRESS(hcd->rsrc_start);

	/*
	pr_debug("\n\n-->VIRT-OHCI-BASE [0x%x], [0x%x] irq[%d]\n\n",
			hcd->regs, (unsigned int)io_p2v( 0x48064400 ),
			pdev->resource[1].start);
	 */

	ohci = hcd_to_ohci(hcd);
	ohci_hcd_init(ohci);

	ohci_clocks->host_enabled = 1;

	retval = usb_add_hcd(hcd, pdev->resource[1].start, IRQF_DISABLED);
	if (retval)
		goto err2;

	return 0;
err2:
	usb_put_hcd(hcd);
	clk_disable(ohci_clocks->usbhost_ick_clk);
	clk_put(ohci_clocks->usbhost_ick_clk);
	clk_disable(ohci_clocks->usbhost2_120m_fck_clk);
	clk_put(ohci_clocks->usbhost2_120m_fck_clk);
	clk_disable(ohci_clocks->usbhost1_48m_fck_clk);
	clk_put(ohci_clocks->usbhost1_48m_fck_clk);
	clk_disable(ohci_clocks->usbtll_fck_clk);
	clk_put(ohci_clocks->usbtll_fck_clk);
	clk_disable(ohci_clocks->usbtll_ick_clk);
	clk_put(ohci_clocks->usbtll_ick_clk);
err0:
	return retval;
}

/* may be called with controller, bus, and devices active */

/**
 * usb_hcd_omap_remove - shutdown processing for OMAP-based HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 */
static inline void
usb_hcd_omap_remove(struct usb_hcd *hcd, struct platform_device *pdev)
{
	struct ohci_omap_clock_defs *ohci_clocks;
	struct ohci_hcd		*ohci = hcd_to_ohci (hcd);

	ohci_clocks = (struct ohci_omap_clock_defs *)
			(((char *)hcd_to_ohci(hcd)) + sizeof(struct ohci_hcd));

	usb_remove_hcd(hcd);
	if (ohci->transceiver) {
		(void) otg_set_host(ohci->transceiver, 0);
		put_device(ohci->transceiver->dev);
	}
	if (machine_is_omap_osk())
		omap_free_gpio(9);

	usb_put_hcd(hcd);

	/* Reset OMAP modules for insmod/rmmod to work */
	omap_writel((1 << 1), OMAP_UHH_SYSCONFIG);
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1 << 0)));
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1 << 1)));
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1 << 2)));
	pr_debug("UHH RESET DONE OMAP_UHH_SYSSTATUS %x !!\n",
			omap_readl(OMAP_UHH_SYSSTATUS));

	omap_writel((1<<1), OMAP_USBTLL_SYSCONFIG);
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS) & (1<<0)));
	pr_debug("TLL RESET DONE");

	if (ohci_clocks->usbtll_fck_clk != NULL) {
		clk_disable(ohci_clocks->usbtll_fck_clk);
		clk_put(ohci_clocks->usbtll_fck_clk);
		ohci_clocks->usbtll_fck_clk = NULL;
	}

	if (ohci_clocks->usbhost_ick_clk != NULL) {
		clk_disable(ohci_clocks->usbhost_ick_clk);
		clk_put(ohci_clocks->usbhost_ick_clk);
		ohci_clocks->usbhost_ick_clk = NULL;
	}

	if (ohci_clocks->usbhost1_48m_fck_clk != NULL) {
		clk_disable(ohci_clocks->usbhost1_48m_fck_clk);
		clk_put(ohci_clocks->usbhost1_48m_fck_clk);
		ohci_clocks->usbhost1_48m_fck_clk = NULL;
	}

	if (ohci_clocks->usbhost2_120m_fck_clk != NULL) {
		clk_disable(ohci_clocks->usbhost2_120m_fck_clk);
		clk_put(ohci_clocks->usbhost2_120m_fck_clk);
		ohci_clocks->usbhost2_120m_fck_clk = NULL;
	}

	if (ohci_clocks->usbtll_ick_clk != NULL) {
		clk_disable(ohci_clocks->usbtll_ick_clk);
		clk_put(ohci_clocks->usbtll_ick_clk);
		ohci_clocks->usbtll_ick_clk = NULL;
	}
}
/*-------------------------------------------------------------------------*/

static int
ohci_omap_start(struct usb_hcd *hcd)
{
	struct omap_usb_config *config;
	struct ohci_omap_clock_defs *ohci_clocks;
	int		ret;
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);

	ohci_clocks = (struct ohci_omap_clock_defs *)
		(((char *)hcd_to_ohci(hcd)) + sizeof(struct ohci_hcd));

	if (!ohci_clocks->host_enabled)
		return 0;
	config = hcd->self.controller->platform_data;
#ifdef HACK_OMAP_OHCI /* todo */
	if (config->otg || config->rwc) {
		ohci->hc_control = OHCI_CTRL_RWC;
		writel(OHCI_CTRL_RWC, &ohci->regs->control);
	}
#endif

	ohci->hc_control = OHCI_CTRL_RWC;
	writel(OHCI_CTRL_RWC, &ohci->regs->control);
	if ((ret = ohci_run(ohci)) < 0) {
		dev_err(hcd->self.controller, "can't start\n");
		ohci_stop(hcd);
		return ret;
	}
	return 0;
}

#ifdef CONFIG_PM

static int omap_ohci_bus_suspend(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	struct ohci_omap_clock_defs *ohci_clocks;
	unsigned long flags;
	int ret = 0;
	u32 control;

	ohci_clocks = (struct ohci_omap_clock_defs *)
			(((char *)ohci) + sizeof(struct ohci_hcd));

	ohci_writel(ohci, OHCI_INTR_MIE, &ohci->regs->intrdisable);
	(void)ohci_readl(ohci, &ohci->regs->control); // flush

	ret = ohci_bus_suspend(hcd);
	if (ret) {
		printk(KERN_INFO "%s: bail out (ret=%d)\n", __func__, ret);
		return ret;
	}

	msleep(6);

	control = ohci_readl(ohci, &ohci->regs->control);
	if ((control & OHCI_CTRL_HCFS) != OHCI_USB_SUSPEND) {
		printk(KERN_INFO "%s: HC is not suspended (control=%08x)\n",
		       __func__, control);
		ret = ohci_bus_resume(hcd);
		return -EBUSY;
	}

	spin_lock_irqsave(&ohci->lock, flags);
	if (!ohci_clocks->suspended) {
		clk_disable(ohci_clocks->usbhost_ick_clk);
		clk_disable(ohci_clocks->usbtll_ick_clk);
		clk_disable(ohci_clocks->usbtll_fck_clk);
		clk_disable(ohci_clocks->usbhost1_48m_fck_clk);
		clk_disable(ohci_clocks->usbhost2_120m_fck_clk);
		ohci_clocks->suspended = 1;
	}
	/* disable rh_status polling */
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	spin_unlock_irqrestore(&ohci->lock, flags);

	return ret;
}

static int omap_ohci_bus_resume(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	struct ohci_omap_clock_defs *ohci_clocks;
	unsigned long flags;
	int ret = 0;

	ohci_clocks = (struct ohci_omap_clock_defs *)
			(((char *)ohci) + sizeof(struct ohci_hcd));

	spin_lock_irqsave(&ohci->lock, flags);
	if (ohci_clocks->suspended) {
		clk_enable(ohci_clocks->usbtll_fck_clk);
		clk_enable(ohci_clocks->usbtll_ick_clk);
		clk_enable(ohci_clocks->usbhost_ick_clk);
		clk_enable(ohci_clocks->usbhost2_120m_fck_clk);
		clk_enable(ohci_clocks->usbhost1_48m_fck_clk);
		ohci_clocks->suspended = 0;
	}
	spin_unlock_irqrestore(&ohci->lock, flags);

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	ret = ohci_bus_resume(hcd);

	return ret;
}



static void omap_ohci_shutdown(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci (hcd);
	struct ohci_omap_clock_defs *ohci_clocks;
	unsigned long flags;
	
	ohci_clocks = (struct ohci_omap_clock_defs *)
			(((char *)ohci) + sizeof(struct ohci_hcd));

	spin_lock_irqsave(&ohci->lock, flags);
	if (ohci_clocks->suspended) {
		clk_enable(ohci_clocks->usbtll_fck_clk);
		clk_enable(ohci_clocks->usbtll_ick_clk);
		clk_enable(ohci_clocks->usbhost_ick_clk);
		clk_enable(ohci_clocks->usbhost2_120m_fck_clk);
		clk_enable(ohci_clocks->usbhost1_48m_fck_clk);
		ohci_clocks->suspended = 0;
	}
	spin_unlock_irqrestore(&ohci->lock, flags);

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	ohci_shutdown(hcd);
}

#endif

/*-------------------------------------------------------------------------*/

static const struct hc_driver ohci_omap_hc_driver = {
	.description =		hcd_name,
	.product_desc =		"OMAP OHCI",
	.hcd_priv_size =	sizeof(struct ohci_hcd)
					+ sizeof(struct ohci_omap_clock_defs),

	/*
	 * generic hardware linkage
	 */
	.irq =			ohci_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,

	/*
	 * basic lifecycle operations
	 */
	.reset =		ohci_omap_init,
	.start =		ohci_omap_start,
	.stop =			ohci_omap_stop,
	.shutdown = 		omap_ohci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		ohci_urb_enqueue,
	.urb_dequeue =		ohci_urb_dequeue,
	.endpoint_disable =	ohci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number =	ohci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data =	ohci_hub_status_data,
	.hub_control =		ohci_hub_control,
	.hub_irq_enable =	ohci_rhsc_enable,
#ifdef	CONFIG_PM
	.bus_suspend =		omap_ohci_bus_suspend, //ohci_bus_suspend,
	.bus_resume =		omap_ohci_bus_resume, // ohci_bus_resume,
#endif
	.start_port_reset =	ohci_start_port_reset,
};

/*-------------------------------------------------------------------------*/

static int ohci_hcd_omap_drv_probe(struct platform_device *dev)
{
	return usb_hcd_omap_probe(&ohci_omap_hc_driver, dev);
}

static int ohci_hcd_omap_drv_remove(struct platform_device *dev)
{
	struct usb_hcd		*hcd = platform_get_drvdata(dev);
	struct ohci_omap_clock_defs *ohci_clocks;
	ohci_clocks = (struct ohci_omap_clock_defs *)
			(((char *)hcd_to_ohci(hcd)) + sizeof(struct ohci_hcd));
	if (ohci_clocks->suspended) {
		clk_enable(ohci_clocks->usbhost_ick_clk);
		clk_enable(ohci_clocks->usbtll_ick_clk);
		clk_enable(ohci_clocks->usbtll_fck_clk);
		clk_enable(ohci_clocks->usbhost1_48m_fck_clk);
		clk_enable(ohci_clocks->usbhost2_120m_fck_clk);
		ohci_clocks->suspended = 0;
	}
	usb_hcd_omap_remove(hcd, dev);
	platform_set_drvdata(dev, NULL);

	return 0;
}

/*-------------------------------------------------------------------------*/

#ifdef	CONFIG_PM

static int ohci_omap_suspend(struct platform_device *dev, pm_message_t message)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(platform_get_drvdata(dev));

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;

	//omap_ohci_bus_suspend(ohci_to_hcd(ohci)); // REVISIT
	ohci_to_hcd(ohci)->state = HC_STATE_SUSPENDED;
	dev->dev.power.power_state = PMSG_SUSPEND;
	return 0;
}

static int ohci_omap_resume(struct platform_device *dev)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(platform_get_drvdata(dev));

	if (time_before(jiffies, ohci->next_statechange))
		msleep(5);
	ohci->next_statechange = jiffies;

	//omap_ohci_bus_resume(ohci_to_hcd(ohci));
	dev->dev.power.power_state = PMSG_ON;
	usb_hcd_resume_root_hub(platform_get_drvdata(dev));
	return 0;
}

#endif

/*-------------------------------------------------------------------------*/

/*
 * Driver definition to register with the OMAP bus
 */
static struct platform_driver ohci_hcd_omap_driver = {
	.probe		= ohci_hcd_omap_drv_probe,
	.remove		= ohci_hcd_omap_drv_remove,
	.shutdown 	= usb_hcd_platform_shutdown,
#ifdef	CONFIG_PM
	.suspend	= ohci_omap_suspend,
	.resume		= ohci_omap_resume,
#endif
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ohci-omap",
	},
};
