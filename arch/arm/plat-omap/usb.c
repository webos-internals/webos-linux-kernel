/*
 * arch/arm/plat-omap/usb.c -- platform level USB initialization
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#undef	DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/usb/otg.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/hardware.h>

#include <asm/arch/mux.h>
#include <asm/arch/usb.h>
#include <asm/arch/board.h>

/* These routines should handle the standard chip-specific modes
 * for usb0/1/2 ports, covering basic mux and transceiver setup.
 *
 * Some board-*.c files will need to set up additional mux options,
 * like for suspend handling, vbus sensing, GPIOs, and the D+ pullup.
 */

/* TESTED ON:
 *  - 1611B H2 (with usb1 mini-AB) using standard Mini-B or OTG cables
 *  - 5912 OSK OHCI (with usb0 standard-A), standard A-to-B cables
 *  - 5912 OSK UDC, with *nonstandard* A-to-A cable
 *  - 1510 Innovator UDC with bundled usb0 cable
 *  - 1510 Innovator OHCI with bundled usb1/usb2 cable
 *  - 1510 Innovator OHCI with custom usb0 cable, feeding 5V VBUS
 *  - 1710 custom development board using alternate pin group
 *  - 1710 H3 (with usb1 mini-AB) using standard Mini-B or OTG cables
 */

/*-------------------------------------------------------------------------*/

//#ifdef	CONFIG_ARCH_OMAP_OTG

static struct otg_transceiver *xceiv;

/**
 * otg_get_transceiver - find the (single) OTG transceiver driver
 *
 * Returns the transceiver driver, after getting a refcount to it; or
 * null if there is no such transceiver.  The caller is responsible for
 * releasing that count.
 */
struct otg_transceiver *otg_get_transceiver(void)
{
	if (xceiv)
		get_device(xceiv->dev);
	return xceiv;
}
EXPORT_SYMBOL(otg_get_transceiver);

int otg_set_transceiver(struct otg_transceiver *x)
{
	if (xceiv && x)
		return -EBUSY;
	xceiv = x;
	return 0;
}
EXPORT_SYMBOL(otg_set_transceiver);

//#endif

/*-------------------------------------------------------------------------*/

static int __init
omap_usb_init(void)
{
	return 0;
}

subsys_initcall(omap_usb_init);

/*-------------------------------------------------------------------------*/

/* Command line handling for USB MUX settings */
static omap_usbmux_t usbmux_mode = USBMUX_USB;
static DEFINE_SPINLOCK(usbmux_lock);

static int __init usbmux_setup(char *str)
{
        if (strcmp(str, "uart3") == 0) {
                printk("USBMUX: Using UART3 over USB for serial console.\n");
                usbmux_mode = USBMUX_UART3;
        }
        return 1;
}
__setup("usbmux=", usbmux_setup);

omap_usbmux_t omap_usbmux_mode(void)
{
	unsigned long flags;
	omap_usbmux_t mode;

	spin_lock_irqsave(&usbmux_lock, flags);
	mode = usbmux_mode;
	spin_unlock_irqrestore(&usbmux_lock, flags);

	return mode;
}

extern int board_usbmux_cfg(omap_usbmux_t mode);

int omap_usbmux_cfg(omap_usbmux_t mode) 
{
#if defined(CONFIG_MACH_SIRLOIN)
	unsigned long flags;

	spin_lock_irqsave(&usbmux_lock, flags);
	usbmux_mode = mode;
	spin_unlock_irqrestore(&usbmux_lock, flags);

	return board_usbmux_cfg(mode);
#else
	return 0;
#endif
}
