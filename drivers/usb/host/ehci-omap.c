/*
 * ehci-omap.c - driver for USBHOST on OMAP 34xx processor
 *
 * Bus Glue for OMAP34xx USBHOST 3 port EHCI controller
 * Tested on OMAP3430 ES2.0 SDP
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2008 Vikram Pandita <vikram.pandita@ti.com>
 * Based on "ehci-fsl.c" by David Brownell and
 * 	    "ehci-au1xxx.c" by K.Boge <karsten.boge@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/arch/gpio.h>

#include "ehci-omap.h"


#ifdef CONFIG_OMAP_EHCI_PHY_MODE
/* EHCI connected to External PHY */

/* External USB connectivity board: 750-2083-001
 * Connected to OMAP3430 SDP
 * The board has Port1 and Port2 connected to ISP1504 in 12-pin ULPI mode
 */

/* ISSUE1:
 *      ISP1504 for input clocking mode needs special reset handling
 * 	Hold the PHY in reset by asserting RESET_N signal
 * 	Then start the 60Mhz clock input to PHY
 * 	Release the reset after a delay -
 * 		to get the PHY state machine in working state
 */
#define EXTERNAL_PHY_RESET
#define	EXT_PHY_RESET_GPIO_PORT1	(57)
#define	EXT_PHY_RESET_GPIO_PORT2	(61)
#define	EXT_PHY_RESET_DELAY		(10)

/* ISSUE2:
 * USBHOST supports External charge pump PHYs only
 * Use the VBUS from Port1 to power VBUS of Port2 externally
 * So use Port2 as the working ULPI port
 */
#define VBUS_INTERNAL_CHARGEPUMP_HACK

#endif /* CONFIG_OMAP_EHCI_PHY_MODE */

/*-------------------------------------------------------------------------*/

/* Define USBHOST clocks for clock management */
struct ehci_omap_clock_defs {
	struct clk	*usbhost_ick_clk;
	struct clk	*usbhost2_120m_fck_clk;
	struct clk	*usbhost1_48m_fck_clk;
	struct clk	*usbtll_fck_clk;
	struct clk	*usbtll_ick_clk;
};

/* Clock names as per clock framework: May change so keep as #defs */
#define USBHOST_ICKL	"usbhost_ick"
#define USBHOST_120M_FCLK	"usbhost2_fck"
#define USBHOST_48M_FCLK	"usbhost1_fck"
#define USBHOST_TLL_ICKL	"usbtll_ick"
#define USBHOST_TLL_FCLK	"usbtll_host_sar_fck"
/*-------------------------------------------------------------------------*/

/* Debug support
 *
 */
static ssize_t show_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_OMAP_EHCI_PHY_MODE
	return sprintf(buf, "UHH:rev 0x%x\n"
				"UHH:sysconfig 0x%x\n"
				"UHH:sysstatus 0x%x\n"
				"UHH:hostconfig  0x%x\n",
				omap_readl(OMAP_UHH_REVISION),
				omap_readl(OMAP_UHH_SYSCONFIG),
				omap_readl(OMAP_UHH_SYSSTATUS),
				omap_readl(OMAP_UHH_HOSTCONFIG));
#else
	return sprintf(buf, "TLL-shared-conf 0x%x\n"
				"TLL_channel_conf[0] 0x%x\n"
				"func ctrl 0x%02x intf ctrl 0x%02x\n"
				"otg ctrl 0x%02x intnr ctrl 0x%02x\n"
				"intnf ctrl 0x%02x ints ctrl 0x%02x\n"
				"intl ctrl 0x%02x dbg ctrl 0x%02x\n"
				"UHH:rev 0x%x\n"
				"UHH:sysconfig 0x%x\n"
				"UHH:sysstatus 0x%x\n"
				"UHH:hostconfig  0x%x\n"
				"UHH:dbgcsr  0x%x\n ",
				omap_readl(OMAP_TLL_SHARED_CONF),
				omap_readl(OMAP_TLL_CHANNEL_CONF(0)),
				omap_readb(OMAP_TLL_ULPI_FUNCTION_CTRL(1)),
				omap_readb(OMAP_TLL_ULPI_INTERFACE_CTRL(1)),
				omap_readb(OMAP_TLL_ULPI_OTG_CTRL(1)),
				omap_readb(OMAP_TLL_ULPI_INT_EN_RISE(1)),
				omap_readb(OMAP_TLL_ULPI_INT_EN_FALL(1)),
				omap_readb(OMAP_TLL_ULPI_INT_STATUS(1)),
				omap_readb(OMAP_TLL_ULPI_INT_LATCH(1)),
				omap_readb(OMAP_TLL_ULPI_DEBUG(1)),
				omap_readl(OMAP_UHH_REVISION),
				omap_readl(OMAP_UHH_SYSCONFIG),
				omap_readl(OMAP_UHH_SYSSTATUS),
				omap_readl(OMAP_UHH_HOSTCONFIG),
				omap_readl(OMAP_UHH_DEBUG_CSR));
#endif
}

static DEVICE_ATTR(debug_omap_usbhost, S_IRUGO, show_debug, NULL);

/*-------------------------------------------------------------------------*/

/* omap_start_ehc
 * 	- Start the TI USBHOST controller
 */
static int omap_start_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_omap_clock_defs *ehci_clocks;

	dev_dbg(hcd->self.controller, ": starting TI EHCI USB Controller\n");

	ehci_clocks = (struct ehci_omap_clock_defs *)(
				((char *)hcd_to_ehci(hcd)) +
					sizeof(struct ehci_hcd));

#if 0 /* DPLL5 programming taken care by clock framework */
	/* Start DPLL5 Programming:
	 * Clock Framework is not stable yet
	 * This will be done in clock framework later
	 */
	/* Enable DPLL 5 : Based on Input of 13Mhz*/
	omap_writel((12 << CM_CLKSEL4_PLL_PERIPH2_DPLL_DIV_SHIFT)|
			(120 << CM_CLKSEL4_PLL_PERIPH2_DPLL_MULT_SHIFT),
				CM_CLKSEL4_PLL);

	omap_writel(1 << CM_CLKSEL5_PLL_DIV_120M_SHIFT, CM_CLKSEL5_PLL);

	omap_writel((7 << PERIPH2_DPLL_FREQSEL_EN_PERIPH2_DPLL_SHIFT) |
			(7 << CM_CLKEN2_PLL_PERIPH2_DPLL_FREQSEL_SHIFT),
				CM_CLKEN2_PLL);
	while (!(omap_readl(CM_IDLEST2_CKGEN) &
			CM_IDLEST2_CKGEN_ST_120M_CLK_SHIFT))
		dev_dbg(hcd->self.controller,
			"idlest2 = 0x%x\n", omap_readl(CM_IDLEST2_CKGEN));
	/* End DPLL5 programming */

	/* PRCM settings for USBHOST:
	 * Interface clk un-related to domain transition
	 */
	omap_writel(0 << CM_AUTOIDLE_USBHOST_AUTO_USBHOST_SHIFT,
						CM_AUTOIDLE_USBHOST);

	/* Disable sleep dependency with MPU and IVA */
	omap_writel((0 << CM_SLEEPDEP_USBHOST_EN_MPU_SHIFT) |
			(0 << CM_SLEEPDEP_USBHOST_EN_MPU_SHIFT),
			CM_SLEEPDEP_USBHOST);

	/* Disable Automatic transition of clock */
	omap_writel(0, CM_CLKSTCTRL_USBHOST);
#endif

	/* Enable Clocks for USBHOST */
	ehci_clocks->usbhost_ick_clk = clk_get(&dev->dev,
						USBHOST_ICKL);
	if (IS_ERR(ehci_clocks->usbhost_ick_clk))
		return PTR_ERR(ehci_clocks->usbhost_ick_clk);
	clk_enable(ehci_clocks->usbhost_ick_clk);


	ehci_clocks->usbhost2_120m_fck_clk = clk_get(&dev->dev,
							USBHOST_120M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost2_120m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost2_120m_fck_clk);
	clk_enable(ehci_clocks->usbhost2_120m_fck_clk);

	ehci_clocks->usbhost1_48m_fck_clk = clk_get(&dev->dev,
						USBHOST_48M_FCLK);
	if (IS_ERR(ehci_clocks->usbhost1_48m_fck_clk))
		return PTR_ERR(ehci_clocks->usbhost1_48m_fck_clk);
	clk_enable(ehci_clocks->usbhost1_48m_fck_clk);


#ifdef EXTERNAL_PHY_RESET
	/* Refer: ISSUE1 */
	omap_request_gpio(EXT_PHY_RESET_GPIO_PORT1);
	omap_set_gpio_direction(EXT_PHY_RESET_GPIO_PORT1, 0);
	omap_request_gpio(EXT_PHY_RESET_GPIO_PORT2);
	omap_set_gpio_direction(EXT_PHY_RESET_GPIO_PORT2, 0);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT1, 0);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT2, 0);
	/* Hold the PHY in RESET for enough time till DIR is high */
	udelay(EXT_PHY_RESET_DELAY);
#endif

	/* Configure TLL for 60Mhz clk for ULPI */
	ehci_clocks->usbtll_fck_clk = clk_get(&dev->dev, USBHOST_TLL_FCLK);
	if (IS_ERR(ehci_clocks->usbtll_fck_clk))
		return PTR_ERR(ehci_clocks->usbtll_fck_clk);
	clk_enable(ehci_clocks->usbtll_fck_clk);

	ehci_clocks->usbtll_ick_clk = clk_get(&dev->dev, USBHOST_TLL_ICKL);
	if (IS_ERR(ehci_clocks->usbtll_ick_clk))
		return PTR_ERR(ehci_clocks->usbtll_ick_clk);
	clk_enable(ehci_clocks->usbtll_ick_clk);

#if 0
	/* Wait for TLL to be Active */
	while ((omap_readl(CM_IDLEST3_CORE) &
		(1 << CM_IDLEST3_CORE_ST_USBTLL_SHIFT)));

	/* Disable Auto Idle of USBTLL */
	omap_writel((0 << CM_AUTOIDLE3_CORE_AUTO_USBTLL_SHIFT),
						CM_AUTOIDLE3_CORE);
#endif

	/* perform TLL soft reset, and wait until reset is complete */
	/* (1<<3) = no idle mode only for initial debugging */
	omap_writel((1 << OMAP_USBTLL_SYSCONFIG_SOFTRESET_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_SIDLEMODE_SHIFT) |
			(1 << OMAP_USBTLL_SYSCONFIG_CACTIVITY_SHIFT),
			OMAP_USBTLL_SYSCONFIG);
	/* Wait for TLL reset to complete */
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS) &
		(1 << OMAP_USBTLL_SYSSTATUS_RESETDONE_SHIFT)));

	dev_dbg(hcd->self.controller, "\n TLL RESET DONE");

	/* Put UHH in NoIdle/NoStandby mode */
	omap_writel((0 << OMAP_UHH_SYSCONFIG_AUTOIDLE_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_ENAWAKEUP_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_SIDLEMODE_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_CACTIVITY_SHIFT) |
			(1 << OMAP_UHH_SYSCONFIG_MIDLEMODE_SHIFT),
			OMAP_UHH_SYSCONFIG);

#ifdef CONFIG_OMAP_EHCI_PHY_MODE
	/* Bypass the TLL module for PHY mode operation */
	omap_writel((0 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT),
						OMAP_UHH_HOSTCONFIG);
	/* Ensure that BYPASS is set */
	while (omap_readl(OMAP_UHH_HOSTCONFIG) &
		(1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT));

	dev_dbg(hcd->self.controller, "Entered ULPI PHY MODE: success");

#else
	/* Use UTMI Ports of TLL */
	omap_writel((1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT),
						OMAP_UHH_HOSTCONFIG);
	/* Enusre bit is set */
	while (!(omap_readl(OMAP_UHH_HOSTCONFIG) &
		(1 << OMAP_UHH_HOSTCONFIG_ULPI_BYPASS_SHIFT)));

	dev_dbg(hcd->self.controller, "Entered UTMI MODE: success");

	/* Program the 3 TLL channels upfront */

	/* CHANNEL-1 */
	/* Disable AutoIdle */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(0)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT),
			OMAP_TLL_CHANNEL_CONF(0));
	/* Disable BitStuffing */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(0)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT),
			OMAP_TLL_CHANNEL_CONF(0));
	/* SDR Mode */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(0)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
			OMAP_TLL_CHANNEL_CONF(0));

	/* CHANNEL-2 */
	/* Disable AutoIdle */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(1)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT),
			OMAP_TLL_CHANNEL_CONF(1));
	/* Disable BitStuffing */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(1)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT),
			OMAP_TLL_CHANNEL_CONF(1));
	/* SDR Mode */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(1)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
			OMAP_TLL_CHANNEL_CONF(1));

	/* CHANNEL-3 */
	/* Disable AutoIdle */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(2)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_UTMIAUTOIDLE_SHIFT),
			OMAP_TLL_CHANNEL_CONF(2));
	/* Disable BitStuffing */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(2)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_ULPINOBITSTUFF_SHIFT),
			OMAP_TLL_CHANNEL_CONF(2));
	/* SDR Mode */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(2)) &
			~(1<<OMAP_TLL_CHANNEL_CONF_ULPIDDRMODE_SHIFT),
			OMAP_TLL_CHANNEL_CONF(2));

	/* Program Common TLL register */
	omap_writel((1 << OMAP_TLL_SHARED_CONF_FCLK_IS_ON_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_DIVRATION_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_180D_SDR_EN_SHIFT) |
			(1 << OMAP_TLL_SHARED_CONF_USB_90D_DDR_EN_SHFT),
				OMAP_TLL_SHARED_CONF);

	/* Enable All 3 channels now */
	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(0)) |
			(1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT),
			OMAP_TLL_CHANNEL_CONF(0));

	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(1)) |
			(1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT),
			OMAP_TLL_CHANNEL_CONF(1));

	omap_writel(omap_readl(OMAP_TLL_CHANNEL_CONF(2)) |
			(1<<OMAP_TLL_CHANNEL_CONF_CHANEN_SHIFT),
			OMAP_TLL_CHANNEL_CONF(2));

	/* test writing to ulpi scratch register */
	omap_writeb(0xBE, OMAP_TLL_ULPI_SCRATCH_REGISTER);
	dev_dbg(hcd->self.controller, "\nULPI_SCRATCH_REG 0x%02x\n",
			omap_readb(OMAP_TLL_ULPI_SCRATCH_REGISTER));

#endif

#ifdef EXTERNAL_PHY_RESET
	/* Refer ISSUE1:
	 * Hold the PHY in RESET for enough time till PHY is settled and ready
	 */
	udelay(EXT_PHY_RESET_DELAY);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT1, 1);
	omap_set_gpio_dataout(EXT_PHY_RESET_GPIO_PORT2, 1);
#endif

#ifdef VBUS_INTERNAL_CHARGEPUMP_HACK
	/* Refer ISSUE2: LINK assumes external charge pump */

	/* use Port1 VBUS to charge externally Port2:
	 * 	So for PHY mode operation use Port2 only
	 */
	omap_writel((0xA << EHCI_INSNREG05_ULPI_REGADD_SHIFT) |/* OTG ctrl reg*/
			(2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT) |/*   Write */
			(1 << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT) |/* Port1 */
			(1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT) |/* Start */
			(0x26),
			EHCI_INSNREG05_ULPI);

	while (!(omap_readl(EHCI_INSNREG05_ULPI) &
		(1<<EHCI_INSNREG05_ULPI_CONTROL_SHIFT)));

#endif

	return 0;
}

/*-------------------------------------------------------------------------*/

static void omap_stop_ehc(struct platform_device *dev, struct usb_hcd *hcd)
{
	struct ehci_omap_clock_defs *ehci_clocks;

	ehci_clocks = (struct ehci_omap_clock_defs *)
			(((char *)hcd_to_ehci(hcd)) + sizeof(struct ehci_hcd));

	dev_dbg(hcd->self.controller, ": stopping TI EHCI USB Controller\n");

	/* Reset OMAP modules for insmod/rmmod to work */
	omap_writel((1<<1), OMAP_UHH_SYSCONFIG);
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<0)));
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<1)));
	while (!(omap_readl(OMAP_UHH_SYSSTATUS) & (1<<2)));
	dev_dbg(hcd->self.controller,
		"UHH RESET DONE OMAP_UHH_SYSSTATUS %x !!\n",
			omap_readl(OMAP_UHH_SYSSTATUS));

	omap_writel((1<<1), OMAP_USBTLL_SYSCONFIG);
	while (!(omap_readl(OMAP_USBTLL_SYSSTATUS) & (1<<0)));
	dev_dbg(hcd->self.controller, ":TLL RESEET DONE");

	if (ehci_clocks->usbtll_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_fck_clk);
		clk_put(ehci_clocks->usbtll_fck_clk);
		ehci_clocks->usbtll_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbhost_ick_clk);
		clk_put(ehci_clocks->usbhost_ick_clk);
		ehci_clocks->usbhost_ick_clk = NULL;
	}

	if (ehci_clocks->usbhost1_48m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost1_48m_fck_clk);
		clk_put(ehci_clocks->usbhost1_48m_fck_clk);
		ehci_clocks->usbhost1_48m_fck_clk = NULL;
	}

	if (ehci_clocks->usbhost2_120m_fck_clk != NULL) {
		clk_disable(ehci_clocks->usbhost2_120m_fck_clk);
		clk_put(ehci_clocks->usbhost2_120m_fck_clk);
		ehci_clocks->usbhost2_120m_fck_clk = NULL;
	}

	if (ehci_clocks->usbtll_ick_clk != NULL) {
		clk_disable(ehci_clocks->usbtll_ick_clk);
		clk_put(ehci_clocks->usbtll_ick_clk);
		ehci_clocks->usbtll_ick_clk = NULL;
	}


#ifdef EXTERNAL_PHY_RESET
	omap_free_gpio(EXT_PHY_RESET_GPIO_PORT1);
	omap_free_gpio(EXT_PHY_RESET_GPIO_PORT2);
#endif

	dev_dbg(hcd->self.controller,
		": Clock to USB host has been disabled\n");
}

/*-------------------------------------------------------------------------*/

/* configure so an HC device and id are always provided */
/* always called with process context; sleeping is OK */

/**
 * usb_ehci_omap_probe - initialize TI-based HCDs
 * Context: !in_interrupt()
 *
 * Allocates basic resources for this USB host controller, and
 * then invokes the start() method for the HCD associated with it
 * through the hotplug entry's driver_data.
 *
 */
int usb_ehci_omap_probe(const struct hc_driver *driver,
			  struct usb_hcd **hcd_out, struct platform_device *dev)
{
	int retval = 0;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;


	if (dev->resource[1].flags != IORESOURCE_IRQ) {
		dev_dbg(&dev->dev, "resource[1] is not IORESOURCE_IRQ");
		retval = -ENOMEM;
	}

	hcd = usb_create_hcd(driver, &dev->dev, dev->dev.bus_id);
	if (!hcd)
		return -ENOMEM;

	retval = omap_start_ehc(dev, hcd);
	if (retval)
		return retval;

	hcd->rsrc_start = 0;
	hcd->rsrc_len = 0;
	hcd->rsrc_start = dev->resource[0].start;
	hcd->rsrc_len = dev->resource[0].end - dev->resource[0].start + 1;

	hcd->regs = (void __iomem *) (int) IO_ADDRESS(hcd->rsrc_start);

	ehci = hcd_to_ehci(hcd);
	ehci->caps = hcd->regs;

	ehci->regs = hcd->regs + HC_LENGTH(readl(&ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = readl(&ehci->caps->hcs_params);

	/* SET 1 micro-frame Interrupt interval */
	writel(readl(&ehci->regs->command) | (1<<16), &ehci->regs->command);

	retval = usb_add_hcd(hcd, dev->resource[1].start,
				IRQF_DISABLED | IRQF_SHARED);
	if (retval == 0)
		return retval;

	dev_dbg(hcd->self.controller, "ERR: add_hcd");
	omap_stop_ehc(dev, hcd);

	usb_put_hcd(hcd);
	return retval;
}

/*-------------------------------------------------------------------------*/

/* may be called without controller electrically present */
/* may be called with controller, bus, and devices active */

/**
 * usb_ehci_omap_remove - shutdown processing for EHCI HCDs
 * @dev: USB Host Controller being removed
 * Context: !in_interrupt()
 *
 * Reverses the effect of usb_ehci_hcd_omap_probe(), first invoking
 * the HCD's stop() method.  It is always called from a thread
 * context, normally "rmmod", "apmd", or something similar.
 *
 */
void usb_ehci_omap_remove(struct usb_hcd *hcd, struct platform_device *dev)
{
	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);
	omap_stop_ehc(dev, hcd);
}

/*-------------------------------------------------------------------------*/
#ifdef CONFIG_PM
static int omap_ehci_bus_suspend(struct usb_hcd *hcd)
{
	return ehci_bus_suspend(hcd);
}

static int omap_ehci_bus_resume(struct usb_hcd *hcd)
{
	return ehci_bus_resume(hcd);
}
#endif
/*-------------------------------------------------------------------------*/

static const struct hc_driver ehci_omap_hc_driver = {
	.description = hcd_name,
	.product_desc = "OMAP-EHCI Host Controller",
	.hcd_priv_size = sizeof(struct ehci_hcd)
				+ sizeof(struct ehci_omap_clock_defs),

	/*
	 * generic hardware linkage
	 */
	.irq = ehci_irq,
	.flags = HCD_MEMORY | HCD_USB2,

	/*
	 * basic lifecycle operations
	 */
	.reset = ehci_init,
	.start = ehci_run,
	.stop = ehci_stop,
	.shutdown = ehci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue = ehci_urb_enqueue,
	.urb_dequeue = ehci_urb_dequeue,
	.endpoint_disable = ehci_endpoint_disable,

	/*
	 * scheduling support
	 */
	.get_frame_number = ehci_get_frame,

	/*
	 * root hub support
	 */
	.hub_status_data = ehci_hub_status_data,
	.hub_control = ehci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend = omap_ehci_bus_suspend,
	.bus_resume = omap_ehci_bus_resume,
#endif
};

/*-------------------------------------------------------------------------*/
static int ehci_hcd_omap_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd *hcd = NULL;
	int ret;

	dev_dbg(&pdev->dev, "ehci_hcd_omap_drv_probe()");

	if (usb_disabled())
		return -ENODEV;

	/* Add Debug support */
	/* cat /sys/devices/platform/ehci-omap.0/debug_omap_usbhost */
	ret = device_create_file(&pdev->dev, &dev_attr_debug_omap_usbhost);

	ret = usb_ehci_omap_probe(&ehci_omap_hc_driver, &hcd, pdev);

	return ret;
}
/*-------------------------------------------------------------------------*/

static int ehci_hcd_omap_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "ehci_hcd_omap_drv_remove()");
	usb_ehci_omap_remove(hcd, pdev);

	/* Remove Debug support */
	device_remove_file(&pdev->dev, &dev_attr_debug_omap_usbhost);

	return 0;
}
/*-------------------------------------------------------------------------*/

MODULE_ALIAS("omap-ehci");
static struct platform_driver ehci_hcd_omap_driver = {
	.probe = ehci_hcd_omap_drv_probe,
	.remove = ehci_hcd_omap_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	/*.suspend      = ehci_hcd_omap_drv_suspend, */
	/*.resume       = ehci_hcd_omap_drv_resume, */
	.driver = {
		.name = "ehci-omap",
		.bus = &platform_bus_type
	}
};
