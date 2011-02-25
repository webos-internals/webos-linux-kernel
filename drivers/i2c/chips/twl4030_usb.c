/*
 * twl4030_usb - TWL4030 USB transceiver, talking to OMAP OTG controller
 *
 * Copyright (C) 2004-2007 Texas Instruments
 *
 * FS USB suport is based heavily on the isp1301_omap.c OTG transceiver driver.
 * Copyright (C) 2004 David Brownell
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Current status:
 *	- FS USB Gadget and Host modes work independently. No OTG support.
 *	- HS USB ULPI mode works.
 */


#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>

#include <linux/console.h>

#include <asm/arch/twl4030.h>
#include <asm/arch/usb.h>
#include <asm/arch/irqs.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>

#undef USE_USB_POWER_CONSTRAINT

#ifdef USE_USB_POWER_CONSTRAINT
#include <asm/arch/resource.h>
#endif

#ifdef CONFIG_USB_GADGET_EVENT
#include <linux/usb/gadget_event.h>
#endif
#include "charger_detector.h"

#define CONFIGURED_MA	CONFIG_USB_GADGET_VBUS_DRAW	/* 500mA */

/* Register defines */

#define VENDOR_ID_LO			0x00
#define VENDOR_ID_HI			0x01
#define PRODUCT_ID_LO			0x02
#define PRODUCT_ID_HI			0x03

#define FUNC_CTRL			0x04
#define FUNC_CTRL_SET			0x05
#define FUNC_CTRL_CLR			0x06
#define FUNC_CTRL_SUSPENDM		(1 << 6)
#define FUNC_CTRL_RESET			(1 << 5)
#define FUNC_CTRL_OPMODE_MASK		(3 << 3) /* bits 3 and 4 */
#define FUNC_CTRL_OPMODE_NORMAL		(0 << 3)
#define FUNC_CTRL_OPMODE_NONDRIVING	(1 << 3)
#define FUNC_CTRL_OPMODE_DISABLE_BIT_NRZI	(2 << 3)
#define FUNC_CTRL_TERMSELECT		(1 << 2)
#define FUNC_CTRL_XCVRSELECT_MASK	(3 << 0) /* bits 0 and 1 */
#define FUNC_CTRL_XCVRSELECT_HS		(0 << 0)
#define FUNC_CTRL_XCVRSELECT_FS		(1 << 0)
#define FUNC_CTRL_XCVRSELECT_LS		(2 << 0)
#define FUNC_CTRL_XCVRSELECT_FS4LS	(3 << 0)

#define IFC_CTRL			0x07
#define IFC_CTRL_SET			0x08
#define IFC_CTRL_CLR			0x09
#define IFC_CTRL_INTERFACE_PROTECT_DISABLE	(1 << 7)
#define IFC_CTRL_AUTORESUME		(1 << 4)
#define IFC_CTRL_CLOCKSUSPENDM		(1 << 3)
#define IFC_CTRL_CARKITMODE		(1 << 2)
#define IFC_CTRL_FSLSSERIALMODE_3PIN	(1 << 1)

#define TWL4030_OTG_CTRL		0x0A
#define TWL4030_OTG_CTRL_SET		0x0B
#define TWL4030_OTG_CTRL_CLR		0x0C
#define TWL4030_OTG_CTRL_DRVVBUS	(1 << 5)
#define TWL4030_OTG_CTRL_CHRGVBUS	(1 << 4)
#define TWL4030_OTG_CTRL_DISCHRGVBUS	(1 << 3)
#define TWL4030_OTG_CTRL_DMPULLDOWN	(1 << 2)
#define TWL4030_OTG_CTRL_DPPULLDOWN	(1 << 1)
#define TWL4030_OTG_CTRL_IDPULLUP	(1 << 0)

#define USB_INT_EN_RISE			0x0D
#define USB_INT_EN_RISE_SET		0x0E
#define USB_INT_EN_RISE_CLR		0x0F
#define USB_INT_EN_FALL			0x10
#define USB_INT_EN_FALL_SET		0x11
#define USB_INT_EN_FALL_CLR		0x12
#define USB_INT_STS			0x13
#define USB_INT_LATCH			0x14
#define USB_INT_IDGND			(1 << 4)
#define USB_INT_SESSEND			(1 << 3)
#define USB_INT_SESSVALID		(1 << 2)
#define USB_INT_VBUSVALID		(1 << 1)
#define USB_INT_HOSTDISCONNECT		(1 << 0)

#define DEBUG_REG			0x15
#define DEBUG_DM_LINESTATE		(1 << 1)
#define DEBUG_DP_LINESTATE		(1 << 0)

#define CARKIT_CTRL			0x19
#define CARKIT_CTRL_SET			0x1A
#define CARKIT_CTRL_CLR			0x1B
#define CARKIT_CTRL_MICEN		(1 << 6)
#define CARKIT_CTRL_SPKRIGHTEN		(1 << 5)
#define CARKIT_CTRL_SPKLEFTEN		(1 << 4)
#define CARKIT_CTRL_RXDEN		(1 << 3)
#define CARKIT_CTRL_TXDEN		(1 << 2)
#define CARKIT_CTRL_IDGNDDRV		(1 << 1)
#define CARKIT_CTRL_CARKITPWR		(1 << 0)
#define CARKIT_PLS_CTRL			0x22
#define CARKIT_PLS_CTRL_SET		0x23
#define CARKIT_PLS_CTRL_CLR		0x24
#define CARKIT_PLS_CTRL_SPKRRIGHT_BIASEN	(1 << 3)
#define CARKIT_PLS_CTRL_SPKRLEFT_BIASEN	(1 << 2)
#define CARKIT_PLS_CTRL_RXPLSEN		(1 << 1)
#define CARKIT_PLS_CTRL_TXPLSEN		(1 << 0)

#define MCPC_CTRL			0x30
#define MCPC_CTRL_SET			0x31
#define MCPC_CTRL_CLR			0x32
#define MCPC_CTRL_RTSOL			(1 << 7)
#define MCPC_CTRL_EXTSWR		(1 << 6)
#define MCPC_CTRL_EXTSWC		(1 << 5)
#define MCPC_CTRL_VOICESW		(1 << 4)
#define MCPC_CTRL_OUT64K		(1 << 3)
#define MCPC_CTRL_RTSCTSSW		(1 << 2)
#define MCPC_CTRL_HS_UART		(1 << 0)

#define MCPC_IO_CTRL			0x33
#define MCPC_IO_CTRL_SET		0x34
#define MCPC_IO_CTRL_CLR		0x35
#define MCPC_IO_CTRL_MICBIASEN		(1 << 5)
#define MCPC_IO_CTRL_CTS_NPU		(1 << 4)
#define MCPC_IO_CTRL_RXD_PU		(1 << 3)
#define MCPC_IO_CTRL_TXDTYP		(1 << 2)
#define MCPC_IO_CTRL_CTSTYP		(1 << 1)
#define MCPC_IO_CTRL_RTSTYP		(1 << 0)

#define MCPC_CTRL2			0x36
#define MCPC_CTRL2_SET			0x37
#define MCPC_CTRL2_CLR			0x38
#define MCPC_CTRL2_MCPC_CK_EN		(1 << 0)

#define OTHER_FUNC_CTRL			0x80
#define OTHER_FUNC_CTRL_SET		0x81
#define OTHER_FUNC_CTRL_CLR		0x82
#define OTHER_FUNC_CTRL_BDIS_ACON_EN	(1 << 4)
#define OTHER_FUNC_CTRL_FIVEWIRE_MODE	(1 << 2)

#define OTHER_IFC_CTRL			0x83
#define OTHER_IFC_CTRL_SET		0x84
#define OTHER_IFC_CTRL_CLR		0x85
#define OTHER_IFC_CTRL_OE_INT_EN	(1 << 6)
#define OTHER_IFC_CTRL_CEA2011_MODE	(1 << 5)
#define OTHER_IFC_CTRL_FSLSSERIALMODE_4PIN	(1 << 4)
#define OTHER_IFC_CTRL_HIZ_ULPI_60MHZ_OUT	(1 << 3)
#define OTHER_IFC_CTRL_HIZ_ULPI		(1 << 2)
#define OTHER_IFC_CTRL_ALT_INT_REROUTE	(1 << 0)

#define OTHER_INT_EN_RISE		0x86
#define OTHER_INT_EN_RISE_SET		0x87
#define OTHER_INT_EN_RISE_CLR		0x88
#define OTHER_INT_EN_FALL		0x89
#define OTHER_INT_EN_FALL_SET		0x8A
#define OTHER_INT_EN_FALL_CLR		0x8B
#define OTHER_INT_STS			0x8C
#define OTHER_INT_LATCH			0x8D
#define OTHER_INT_VB_SESS_VLD		(1 << 7)
#define OTHER_INT_DM_HI			(1 << 6) /* not valid for "latch" reg */
#define OTHER_INT_DP_HI			(1 << 5) /* not valid for "latch" reg */
#define OTHER_INT_BDIS_ACON		(1 << 3) /* not valid for "fall" regs */
#define OTHER_INT_MANU			(1 << 1)
#define OTHER_INT_ABNORMAL_STRESS	(1 << 0)

#define ID_STATUS			0x96
#define ID_RES_FLOAT			(1 << 4)
#define ID_RES_440K			(1 << 3)
#define ID_RES_200K			(1 << 2)
#define ID_RES_102K			(1 << 1)
#define ID_RES_GND			(1 << 0)

#define POWER_CTRL			0xAC
#define POWER_CTRL_SET			0xAD
#define POWER_CTRL_CLR			0xAE
#define POWER_CTRL_OTG_ENAB		(1 << 5)

#define OTHER_IFC_CTRL2			0xAF
#define OTHER_IFC_CTRL2_SET		0xB0
#define OTHER_IFC_CTRL2_CLR		0xB1
#define OTHER_IFC_CTRL2_ULPI_STP_LOW	(1 << 4)
#define OTHER_IFC_CTRL2_ULPI_TXEN_POL	(1 << 3)
#define OTHER_IFC_CTRL2_ULPI_4PIN_2430	(1 << 2)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_MASK	(3 << 0) /* bits 0 and 1 */
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT1N	(0 << 0)
#define OTHER_IFC_CTRL2_USB_INT_OUTSEL_INT2N	(1 << 0)

#define REG_CTRL_EN			0xB2
#define REG_CTRL_EN_SET			0xB3
#define REG_CTRL_EN_CLR			0xB4
#define REG_CTRL_ERROR			0xB5
#define ULPI_I2C_CONFLICT_INTEN		(1 << 0)

#define OTHER_FUNC_CTRL2		0xB8
#define OTHER_FUNC_CTRL2_SET		0xB9
#define OTHER_FUNC_CTRL2_CLR		0xBA
#define OTHER_FUNC_CTRL2_VBAT_TIMER_EN	(1 << 0)

/* following registers do not have separate _clr and _set registers */
#define VBUS_DEBOUNCE			0xC0
#define ID_DEBOUNCE			0xC1
#define VBAT_TIMER			0xD3
#define PHY_PWR_CTRL			0xFD
#define PHY_PWR_PHYPWD			(1 << 0)
#define PHY_CLK_CTRL			0xFE
#define PHY_CLK_CTRL_CLOCKGATING_EN	(1 << 2)
#define PHY_CLK_CTRL_CLK32K_EN		(1 << 1)
#define REQ_PHY_DPLL_CLK		(1 << 0)
#define PHY_CLK_CTRL_STS		0xFF
#define PHY_DPLL_CLK			(1 << 0)

/* In module TWL4030_MODULE_PM_MASTER */
#define PROTECT_KEY			0x0E

#define STS_HW_CONDITIONS		0x0F
#define STS_CHG				(1 << 1)
#define STS_USB				(1 << 2)
#define STS_VBUS			(1 << 7)

/* In module TWL4030_MODULE_PM_RECEIVER */
#define VUSB_DEDICATED1			0x7D
#define VUSB_DEDICATED2			0x7E
#define VUSB1V5_DEV_GRP			0x71
#define VUSB1V5_TYPE			0x72
#define VUSB1V5_REMAP			0x73
#define VUSB1V8_DEV_GRP			0x74
#define VUSB1V8_TYPE			0x75
#define VUSB1V8_REMAP			0x76
#define VUSB3V1_DEV_GRP			0x77
#define VUSB3V1_TYPE			0x78
#define VUSB3V1_REMAP			0x79

#define ID_STATUS			0x96
#define ID_RES_FLOAT			(1 << 4) /* mini-B */
#define ID_RES_440K			(1 << 3) /* type 2 charger */
#define ID_RES_200K			(1 << 2) /* 5-wire carkit or
						    type 1 charger */
#define ID_RES_102K			(1 << 1) /* phone */
#define ID_RES_GND			(1 << 0) /* mini-A */

/* In module TWL4030_MODULE_INTBR */
#define PMBR1				0x0D
#define GPIO_USB_4PIN_ULPI_2430C	(3 << 0)

/* In module TWL4030_MODULE_INT */
#define REG_PWR_ISR1			0x00
#define REG_PWR_IMR1			0x01
#define USB_PRES			(1 << 2)
#define REG_PWR_EDR1			0x05
#define USB_PRES_FALLING		(1 << 4)
#define USB_PRES_RISING			(1 << 5)
#define REG_PWR_SIH_CTRL		0x07
#define COR				(1 << 2)

/* internal define on top of container_of */
#define xceiv_to_twl(x)		container_of((x), struct twl4030_usb, otg);

/* bits in OTG_CTRL */

#define	OTG_XCEIV_OUTPUTS \
	(OTG_ASESSVLD|OTG_BSESSEND|OTG_BSESSVLD|OTG_VBUSVLD|OTG_ID)
#define	OTG_XCEIV_INPUTS \
	(OTG_PULLDOWN|OTG_PULLUP|OTG_DRV_VBUS|OTG_PD_VBUS|OTG_PU_VBUS|OTG_PU_ID)
#define	OTG_CTRL_BITS \
	(OTG_A_BUSREQ|OTG_A_SETB_HNPEN|OTG_B_BUSREQ|OTG_B_HNPEN|OTG_BUSDROP)
	/* and OTG_PULLUP is sometimes written */

#define	OTG_CTRL_MASK	(OTG_DRIVER_SEL| \
	OTG_XCEIV_OUTPUTS|OTG_XCEIV_INPUTS| \
	OTG_CTRL_BITS)


/*-------------------------------------------------------------------------*/

struct twl4030_usb {
	spinlock_t		lock;
	struct otg_transceiver	otg;
	int			irq;
	u8			usb_mode;	/* pin configuration */
#define T2_USB_MODE_ULPI		1
#define T2_USB_MODE_CEA2011_3PIN	2
#define T2_UART_MODE_CARKIT     3
	u8			asleep;
#ifdef USE_USB_POWER_CONSTRAINT
	struct constraint_handle	*usb_power_constraint;
#endif
	int			mA;
	struct work_struct	vbus_draw_work;
#ifdef CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
	struct work_struct usbmux_timer_cb_work;
#endif
	struct delayed_work	phy_suspend_work;
	int			phy_try_suspending;
};

static struct twl4030_usb *the_transceiver;

/*-------------------------------------------------------------------------*/

static int twl4030_i2c_write_u8_verify(u8 module, u8 data, u8 address)
{
	u8 check;

	if ((twl4030_i2c_write_u8(module, data, address) >= 0) &&
	    (twl4030_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	/* Failed once: Try again */
	if ((twl4030_i2c_write_u8(module, data, address) >= 0) &&
	    (twl4030_i2c_read_u8(module, &check, address) >= 0) &&
						(check == data))
		return 0;
	/* Failed again: Return error */
	return -EBUSY;
}

#define twl4030_usb_write_verify(address, data)	\
	twl4030_i2c_write_u8_verify(TWL4030_MODULE_USB, (data), (address))

static inline int twl4030_usb_write(u8 address, u8 data)
{
	int ret = 0;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_USB, data, address);
	if (ret >= 0) {
#if 0	/* debug */
		u8 data1;
		if (twl4030_i2c_read_u8(TWL4030_MODULE_USB, &data1,
					address) < 0)
			printk(KERN_ERR "re-read failed\n");
		else
			printk(KERN_INFO
			       "Write %s wrote %x read %x from reg %x\n",
			       (data1 == data) ? "succeed" : "mismatch",
			       data, data1, address);
#endif
	} else {
		printk(KERN_WARNING
			"TWL4030:USB:Write[0x%x] Error %d\n", address, ret);
	}
	return ret;
}

static inline int twl4030_usb_read(u8 address)
{
	u8 data;
	int ret = 0;
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_USB, &data, address);
	if (ret >= 0) {
		ret = data;
	} else {
		printk(KERN_WARNING
			"TWL4030:USB:Read[0x%x] Error %d\n", address, ret);
	}
	return ret;
}

/*-------------------------------------------------------------------------*/

static inline int
twl4030_usb_set_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(reg + 1, bits);
}

static inline int
twl4030_usb_clear_bits(struct twl4030_usb *twl, u8 reg, u8 bits)
{
	return twl4030_usb_write(reg + 2, bits);
}

/*-------------------------------------------------------------------------*/

static void enable_vbus_draw(struct twl4030_usb *twl, unsigned mA)
{
	twl->mA = mA;
	schedule_work(&twl->vbus_draw_work);
}

static int twl4030_set_power(struct otg_transceiver *dev, unsigned mA)
{
	if (!the_transceiver)
		return -ENODEV;
	if (dev->state == OTG_STATE_B_PERIPHERAL)
		enable_vbus_draw(the_transceiver, mA);
	return 0;
}

#ifdef CONFIG_MACH_BRISKET
static void dp_pullup(struct twl4030_usb *twl, int enable)
{
	if (enable) {
		twl4030_usb_clear_bits(twl, TWL4030_OTG_CTRL,
				       TWL4030_OTG_CTRL_DPPULLDOWN);
		twl4030_usb_set_bits(twl, FUNC_CTRL, FUNC_CTRL_TERMSELECT);
	} else {
		twl4030_usb_clear_bits(twl, FUNC_CTRL, FUNC_CTRL_TERMSELECT);
		twl4030_usb_set_bits(twl, TWL4030_OTG_CTRL,
				     TWL4030_OTG_CTRL_DPPULLDOWN);
	}
	//msleep(200);
}

static void b_peripheral(struct twl4030_usb *twl)
{
	if (twl->otg.state != OTG_STATE_B_IDLE) {
		printk(KERN_INFO "twl4030_usb: not in B_IDLE. ignore\n");
		return;
	}

	OTG_CTRL_REG = OTG_CTRL_REG & OTG_XCEIV_OUTPUTS;
	usb_gadget_vbus_connect(twl->otg.gadget); // omap_vbus_session()

	enable_vbus_draw(twl, 100);
	if (twl4030_usb_read(OTHER_INT_STS) & OTHER_INT_VB_SESS_VLD) {
		OTG_CTRL_REG &= ~OTG_BSESSEND;
		OTG_CTRL_REG |= OTG_BSESSVLD;
	}
	/* UDC driver just set OTG_BSESSVLD */
	dp_pullup(twl, 1);

	twl->otg.state = OTG_STATE_B_PERIPHERAL;

	printk(KERN_INFO "twl4030_usb: B_PERIPHERAL state\n");
}

static void b_idle(struct twl4030_usb *twl)
{
	if (twl->otg.state != OTG_STATE_B_PERIPHERAL) {
		printk(KERN_INFO "twl4030_usb: not in B_PERIPHERAL. ignore\n");
		return;
	}

	OTG_CTRL_REG = OTG_CTRL_REG & OTG_XCEIV_OUTPUTS;
	usb_gadget_vbus_disconnect(twl->otg.gadget); // omap_vbus_session()

	enable_vbus_draw(twl, 0);

	if (!(twl4030_usb_read(OTHER_INT_STS) & OTHER_INT_VB_SESS_VLD)) {
		OTG_CTRL_REG &= ~OTG_BSESSVLD;
		OTG_CTRL_REG |= OTG_BSESSEND;
	}

	dp_pullup(twl, 0);

	twl->otg.state = OTG_STATE_B_IDLE;

	printk(KERN_INFO "twl4030_usb: B_IDLE state\n");
}
#endif

#ifdef CONFIG_TWL4030_USB_FS_3_PIN
static void fs_usb_init(struct twl4030_usb *twl)
{
	twl->usb_mode = T2_USB_MODE_CEA2011_3PIN;
	return;
}
#endif	// CONFIG_TWL4030_USB_FS_3_PIN

/*-------------------------------------------------------------------------*/

static void twl4030_cea2011_3_pin_FS_setup(struct twl4030_usb *twl)
{
	u8 pmbr1;

	/* Important! - choose bet GPIO or USB */
	twl4030_i2c_read_u8(TWL4030_MODULE_INTBR, &pmbr1, PMBR1);
	twl4030_i2c_write_u8(TWL4030_MODULE_INTBR,
			     pmbr1 | GPIO_USB_4PIN_ULPI_2430C, PMBR1);

	/* Mux between UART and USB ULPI lines */

	twl4030_usb_clear_bits(twl, MCPC_CTRL2, MCPC_CTRL2_MCPC_CK_EN);

	twl4030_usb_clear_bits(twl, CARKIT_CTRL,
			       CARKIT_CTRL_CARKITPWR |
			       CARKIT_CTRL_TXDEN |
			       CARKIT_CTRL_RXDEN |
			       CARKIT_CTRL_SPKLEFTEN |
			       CARKIT_CTRL_MICEN);

	twl4030_usb_clear_bits(twl, CARKIT_PLS_CTRL,
			       CARKIT_PLS_CTRL_TXPLSEN |
			       CARKIT_PLS_CTRL_RXPLSEN |
			       CARKIT_PLS_CTRL_SPKRLEFT_BIASEN);

	twl4030_usb_clear_bits(twl, MCPC_CTRL,
			       MCPC_CTRL_HS_UART |
			       MCPC_CTRL_RTSCTSSW |
			       MCPC_CTRL_OUT64K |
			       MCPC_CTRL_VOICESW |
			       MCPC_CTRL_EXTSWC |
			       MCPC_CTRL_EXTSWR);

	twl4030_usb_set_bits(twl, MCPC_IO_CTRL, MCPC_IO_CTRL_TXDTYP);

	twl4030_usb_clear_bits(twl, MCPC_IO_CTRL,
			       MCPC_IO_CTRL_RTSTYP |
			       MCPC_IO_CTRL_CTSTYP |
			       MCPC_IO_CTRL_RXD_PU |
			       MCPC_IO_CTRL_CTS_NPU |
			       MCPC_IO_CTRL_MICBIASEN);

	twl4030_usb_set_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);

	/* setup transceiver mode for FS */

	twl4030_usb_clear_bits(twl, TWL4030_OTG_CTRL,
			       TWL4030_OTG_CTRL_DPPULLDOWN |
			       TWL4030_OTG_CTRL_DMPULLDOWN);
	twl4030_usb_clear_bits(twl, FUNC_CTRL,
			       FUNC_CTRL_XCVRSELECT_MASK |
			       FUNC_CTRL_OPMODE_MASK);
	twl4030_usb_set_bits(twl, FUNC_CTRL,
			     FUNC_CTRL_OPMODE_NORMAL |
			     FUNC_CTRL_XCVRSELECT_FS);

	twl4030_usb_clear_bits(twl, IFC_CTRL, IFC_CTRL_CARKITMODE);
	twl4030_usb_set_bits(twl, IFC_CTRL, IFC_CTRL_FSLSSERIALMODE_3PIN);

	twl4030_usb_clear_bits(twl, OTHER_IFC_CTRL,
			       OTHER_IFC_CTRL_CEA2011_MODE |
			       OTHER_IFC_CTRL_FSLSSERIALMODE_4PIN);

	twl4030_usb_clear_bits(twl, OTHER_IFC_CTRL2,
			       OTHER_IFC_CTRL2_ULPI_4PIN_2430);

	twl4030_usb_set_bits(twl, OTHER_IFC_CTRL2,
			     OTHER_IFC_CTRL2_ULPI_TXEN_POL);
}

static void twl4030_uart_carkit_setup(struct twl4030_usb *twl)
{
	twl4030_usb_write(MCPC_CTRL2, MCPC_CTRL2_MCPC_CK_EN);
	twl4030_usb_set_bits(twl, IFC_CTRL, IFC_CTRL_CARKITMODE);
	twl4030_usb_clear_bits(twl, IFC_CTRL, IFC_CTRL_FSLSSERIALMODE_3PIN);
	twl4030_usb_set_bits(twl, CARKIT_CTRL, CARKIT_CTRL_RXDEN | CARKIT_CTRL_TXDEN | CARKIT_CTRL_CARKITPWR);
	twl4030_usb_set_bits(twl, CARKIT_CTRL, CARKIT_CTRL_MICEN | CARKIT_CTRL_SPKLEFTEN | CARKIT_CTRL_RXDEN | CARKIT_CTRL_TXDEN | CARKIT_CTRL_CARKITPWR);
	twl4030_usb_set_bits(twl, CARKIT_PLS_CTRL, CARKIT_PLS_CTRL_SPKRLEFT_BIASEN | CARKIT_PLS_CTRL_RXPLSEN | CARKIT_PLS_CTRL_TXPLSEN);
	twl4030_usb_clear_bits(twl, MCPC_CTRL, MCPC_CTRL_EXTSWR | MCPC_CTRL_EXTSWC | MCPC_CTRL_VOICESW | MCPC_CTRL_OUT64K | MCPC_CTRL_RTSCTSSW | MCPC_CTRL_HS_UART);
	twl4030_usb_clear_bits(twl, MCPC_IO_CTRL, MCPC_IO_CTRL_MICBIASEN | MCPC_IO_CTRL_CTS_NPU | MCPC_IO_CTRL_RXD_PU);
	twl4030_usb_set_bits(twl, MCPC_IO_CTRL, MCPC_IO_CTRL_TXDTYP);
	twl4030_usb_clear_bits(twl, OTHER_IFC_CTRL2, OTHER_IFC_CTRL2_ULPI_4PIN_2430 | OTHER_IFC_CTRL2_ULPI_TXEN_POL);
}

static void twl4030_usb_set_mode(struct twl4030_usb *twl, int mode)
{
	twl->usb_mode = mode;

	switch (mode) {
	case T2_USB_MODE_ULPI:
		twl4030_usb_clear_bits(twl, IFC_CTRL, IFC_CTRL_CARKITMODE);
		twl4030_usb_set_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);
		/* yvonne: I suspect setting the PHY as high speed here is
		 * related to the musb sometimes not generating any interrupts,
		 * because the musb tries to configure the PHY also. The device
		 * will still enumerate as high-speed without this setting,
		 * because it is set by musb after it negotiates with the hub
		 * during reset. */
		twl4030_usb_clear_bits(twl, FUNC_CTRL, FUNC_CTRL_OPMODE_MASK);
		twl4030_usb_set_bits(twl, FUNC_CTRL, FUNC_CTRL_XCVRSELECT_FS);
		//twl4030_usb_clear_bits(twl, FUNC_CTRL,
		//		       FUNC_CTRL_XCVRSELECT_MASK |
		//		       FUNC_CTRL_OPMODE_MASK);
		break;
	case T2_USB_MODE_CEA2011_3PIN:
		twl4030_cea2011_3_pin_FS_setup(twl);
		break;
	case T2_UART_MODE_CARKIT:
		twl4030_uart_carkit_setup(twl);
		break;
	default:
		/* FIXME: power on defaults */
		break;
	};
}

#ifdef CONFIG_TWL4030_USB_HS_ULPI
static void hs_usb_init(struct twl4030_usb *twl)
{
	twl->usb_mode = T2_USB_MODE_ULPI;
	return;
}

#endif

static void twl4030_i2c_access(int on)
{
	unsigned long timeout;
	int val = twl4030_usb_read(PHY_CLK_CTRL);

	if (val >= 0) {
		if (on) {
			/* enable DPLL to access PHY registers over I2C */
			val |= REQ_PHY_DPLL_CLK;
			if (twl4030_usb_write_verify(PHY_CLK_CTRL,
								(u8)val) < 0) {
				printk(KERN_ERR "twl4030_usb: i2c write failed,"
						" line %d\n", __LINE__);
				return;
			}

			timeout = jiffies + HZ;
			while (!(twl4030_usb_read(PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK)
				&& time_before(jiffies, timeout))
					udelay(10);
			if (!(twl4030_usb_read(PHY_CLK_CTRL_STS) &
							PHY_DPLL_CLK))
				printk(KERN_ERR "Timeout setting T2 HSUSB "
						"PHY DPLL clock\n");
		} else {
			/* let ULPI control the DPLL clock */
			val &= ~REQ_PHY_DPLL_CLK;
			if (twl4030_usb_write_verify(PHY_CLK_CTRL,
								(u8)val) < 0) {
				printk(KERN_ERR "twl4030_usb: i2c write failed,"
						" line %d\n", __LINE__);
			}
		}
	}
	return;
}

static void usb_irq_enable(int rising, int falling)
{
	u8 val;

	/* edge setup */
	if (twl4030_i2c_read_u8(TWL4030_MODULE_INT, &val, REG_PWR_EDR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c read failed,"
				" line %d\n", __LINE__);
		return;
	}
	val &= ~(USB_PRES_RISING | USB_PRES_FALLING);
	if (rising)
		val = val | USB_PRES_RISING;
	if (falling)
		val = val | USB_PRES_FALLING;
	if (twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val,
							REG_PWR_EDR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c write failed,"
				" line %d\n", __LINE__);
		return;
	}

	/* un-mask interrupt */
	if (twl4030_i2c_read_u8(TWL4030_MODULE_INT, &val, REG_PWR_IMR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c read failed,"
				" line %d\n", __LINE__);
		return;
	}
	val &= ~USB_PRES;
	if (twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val,
							REG_PWR_IMR1) < 0)
		printk(KERN_ERR "twl4030_usb: i2c write failed,"
				" line %d\n", __LINE__);

	return;
}

static void usb_irq_disable(void)
{
	u8 val;

	/* undo edge setup */
	if (twl4030_i2c_read_u8(TWL4030_MODULE_INT, &val, REG_PWR_EDR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c read failed,"
				" line %d\n", __LINE__);
		return;
	}
	val &= ~(USB_PRES_RISING | USB_PRES_FALLING);
	if (twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val,
							REG_PWR_EDR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c write failed,"
				" line %d\n", __LINE__);
		return;
	}

	/* mask interrupt */
	if (twl4030_i2c_read_u8(TWL4030_MODULE_INT, &val, REG_PWR_IMR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c read failed,"
				" line %d\n", __LINE__);
		return;
	}
	val |= USB_PRES;
	if (twl4030_i2c_write_u8_verify(TWL4030_MODULE_INT, val,
							REG_PWR_IMR1) < 0)
		printk(KERN_ERR "twl4030_usb: i2c write failed,"
				" line %d\n", __LINE__);

	return;
}

static void twl4030_phy_power(struct twl4030_usb *twl, int on)
{
	u8 pwr;

	pwr = twl4030_usb_read(PHY_PWR_CTRL);
	if (on) {
		pwr &= ~PHY_PWR_PHYPWD;
		if (twl4030_usb_write_verify(PHY_PWR_CTRL, pwr) < 0) {
			printk(KERN_ERR "twl4030_usb: i2c write failed,"
					" line %d\n", __LINE__);
			return;
		}
		twl4030_usb_write(PHY_CLK_CTRL,
				  twl4030_usb_read(PHY_CLK_CTRL) |
					(PHY_CLK_CTRL_CLOCKGATING_EN |
						PHY_CLK_CTRL_CLK32K_EN));
	} else  {
		pwr |= PHY_PWR_PHYPWD;
		if (twl4030_usb_write_verify(PHY_PWR_CTRL, pwr) < 0) {
			printk(KERN_ERR "twl4030_usb: i2c write failed,"
					" line %d\n", __LINE__);
		}
	}
	return;
}

extern int musb_otg_state(enum usb_otg_state *state);
extern int musb_softconn(int is_on);

static int check_musb_idle(void)
{
	enum usb_otg_state state;

	if (musb_otg_state(&state)) {
		printk(KERN_INFO "%s: can't get otg_state\n", __func__);
		return -1;
	}
	if (state == OTG_STATE_B_IDLE || state == OTG_STATE_UNDEFINED)
		return 1;

	/* musb is active */
	return 0;
}

static void phy_suspend(struct twl4030_usb *twl)
{
	printk(KERN_INFO "%s: enter\n", __func__);

	musb_softconn(0);
	twl4030_phy_power(twl, 0);
	twl->asleep = 1;
#ifdef USE_USB_POWER_CONSTRAINT
	/* Release USB constraint on OFF/RET */
	if (twl->usb_power_constraint){
		constraint_remove(twl->usb_power_constraint);
	}
#endif
}

static void phy_suspend_work(struct work_struct *work)
{
	struct twl4030_usb *twl = container_of(work, struct twl4030_usb,
					       phy_suspend_work.work);
	unsigned long flags;

#define NRETRY	10

	spin_lock_irqsave(&twl->lock, flags);
	if (twl->phy_try_suspending == 0) { /* cancelled */
		spin_unlock_irqrestore(&twl->lock, flags);
		printk(KERN_INFO "%s: cancelled\n", __func__);
		return;
	}
	if (twl->phy_try_suspending >= NRETRY) { /* timeout */
		int n = twl->phy_try_suspending;
		twl->phy_try_suspending = 0;
		spin_unlock_irqrestore(&twl->lock, flags);
		printk(KERN_INFO "%s: musb is still active after %d retries\n",
		       __func__, n);
#ifdef CONFIG_USB_GADGET_EVENT
		gadget_event_host_connected_async(0, HZ); /* musb hasn't done this */
#endif
		phy_suspend(twl);
		return;
	}
	if (check_musb_idle() == 0) { /* musb is still active */
		twl->phy_try_suspending++;
		schedule_delayed_work(&twl->phy_suspend_work,
				      msecs_to_jiffies(10));
		spin_unlock_irqrestore(&twl->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&twl->lock, flags);
	printk(KERN_INFO "%s: musb went idle after %d retries\n",
	       __func__, twl->phy_try_suspending);
	phy_suspend(twl);
}

static void twl4030_phy_suspend(int irq_disable)
{
	struct twl4030_usb *twl = the_transceiver;
	unsigned long flags;

	printk(KERN_INFO "%s: irq_disable=%d\n", __func__, irq_disable);

	if (irq_disable)
		usb_irq_disable();
	else
		/* enable rising edge interrupt to detect cable attach */
		usb_irq_enable(1, 0);

	spin_lock_irqsave(&twl->lock, flags);
	if (twl->asleep) {
		spin_unlock_irqrestore(&twl->lock, flags);
		printk(KERN_INFO "%s: phy is already asleep\n", __func__);
		return;
	}

	if (twl->usb_mode == T2_USB_MODE_ULPI) {
		if (check_musb_idle() == 0) { /* musb is still active */
			twl->phy_try_suspending = 1;
			schedule_delayed_work(&twl->phy_suspend_work,
					      msecs_to_jiffies(10));
			spin_unlock_irqrestore(&twl->lock, flags);
			return;
		}
	}
	spin_unlock_irqrestore(&twl->lock, flags);

	phy_suspend(twl);
}

static void twl4030_phy_resume(void)
{
	struct twl4030_usb *twl = the_transceiver;
	unsigned long flags;

	cancel_delayed_work_sync(&twl->phy_suspend_work);

	spin_lock_irqsave(&twl->lock, flags);
	twl->phy_try_suspending = 0;

	if (!twl->asleep) {
		printk(KERN_INFO "%s: phy is not asleep (ignore)\n", __func__);
		spin_unlock_irqrestore(&twl->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&twl->lock, flags);

	printk(KERN_INFO "%s: mode=%d\n", __func__, twl->usb_mode);

#ifdef USE_USB_POWER_CONSTRAINT
	/* Acquire USB constraint on OFF/RET */
	if (twl->usb_power_constraint)
		constraint_set(twl->usb_power_constraint,
					CO_LATENCY_MPUOFF_COREON);
#endif
	/* enable falling edge interrupt to detect cable detach */
	usb_irq_enable(0, 1);

	twl4030_phy_power(twl, 1);
	twl4030_i2c_access(1);
	twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(0);
	twl->asleep = 0;
	musb_softconn(1);

	return;
}

int transceiver_vbus_presence(int *presence)
{
	struct twl4030_usb *twl = the_transceiver;
	u8 reg;
	int ret;

	if (twl->usb_mode == T2_UART_MODE_CARKIT) {
		*presence = 0;
		return 0;
	}

	// Read the USB presence.
	twl4030_i2c_access(1);
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &reg,
					STS_HW_CONDITIONS);
	twl4030_i2c_access(0);

	if (!ret)
		*presence = !!(STS_VBUS & reg);

	return (ret);
}
EXPORT_SYMBOL(transceiver_vbus_presence);

int transceiver_is_pullup_attached(int *attached)
{
	struct twl4030_usb *twl = the_transceiver;
	u8 reg;
	int ret;

	if (twl->usb_mode == T2_UART_MODE_CARKIT) {
		*attached = 0;
		return 0;
	}

	// Read the pull-up state.
	twl4030_i2c_access(1);
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_USB, &reg, FUNC_CTRL);
	twl4030_i2c_access(0);

	if (!ret)
		*attached = !!(FUNC_CTRL_TERMSELECT & reg);

	return (ret);
}
EXPORT_SYMBOL(transceiver_is_pullup_attached);

int transceiver_single_ended_state(int *dplus, int *dminus)
{
	u8 reg;
	int ret;

	// Read the single ended receiver state.
	twl4030_i2c_access(1);
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_USB, &reg, DEBUG_REG);
	twl4030_i2c_access(0);

	if (!ret)
	{
		*dplus = !!(DEBUG_DP_LINESTATE & reg);
		*dminus = !!(DEBUG_DM_LINESTATE & reg);
	}

	return (ret);
}
EXPORT_SYMBOL(transceiver_single_ended_state);

#if defined(CONFIG_TWL4030_USB_FS_3_PIN) && defined(CONFIG_ARCH_OMAP24XX)
void transceiver_reconnect(void)
{
	struct twl4030_usb *twl = the_transceiver;

	printk(KERN_DEBUG "transceiver_reconnect\n");
	dp_pullup(twl, 0);
	msleep(200); /* >10ms */
	dp_pullup(twl, 1);
}
EXPORT_SYMBOL(transceiver_reconnect);
#endif

static void vbus_draw_work(struct work_struct *work)
{
	struct twl4030_usb *twl = container_of(work, struct twl4030_usb,
					       vbus_draw_work);
	if (twl->mA == CONFIGURED_MA) {
		charger_cancel_detection();
	}
#ifdef CONFIG_USB_GADGET_EVENT
	gadget_event_power_state_changed(G_EV_SOURCE_BUS, twl->mA);
#endif
}

static void twl4030_usb_ldo_init(struct twl4030_usb *twl)
{
	/* Enable writing to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0xC0, PROTECT_KEY);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x0C, PROTECT_KEY);

	/* put VUSB3V1 LDO in active state */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB_DEDICATED2);

	/* input to VUSB3V1 LDO is from VBAT, not VBUS */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x14, VUSB_DEDICATED1);

	/* turn on 3.1V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x20, VUSB3V1_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB3V1_TYPE);

	/* turn on 1.5V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x20, VUSB1V5_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V5_TYPE);

	/* turn on 1.8V regulator */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x20, VUSB1V8_DEV_GRP);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, VUSB1V8_TYPE);

	/* disable access to power configuration registers */
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0, PROTECT_KEY);
}

static irqreturn_t twl4030_usb_irq(int irq, void *_twl)
{
	int ret = IRQ_NONE;
	u8 val;

	if (twl4030_i2c_read_u8(TWL4030_MODULE_INT, &val, REG_PWR_ISR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c read failed,"
				" line %d\n", __LINE__);
		goto done;
	}

	/* this interrupt line may be shared */
	if(!(val & USB_PRES))
		goto done;

	/* clear the interrupt */
	if (twl4030_i2c_write_u8(TWL4030_MODULE_INT, USB_PRES, REG_PWR_ISR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c write failed,"
				" line %d\n", __LINE__);
		goto done;
	}

	/* action based on cable attach or detach */
	if (twl4030_i2c_read_u8(TWL4030_MODULE_INT, &val, REG_PWR_EDR1) < 0) {
		printk(KERN_ERR "twl4030_usb: i2c read failed,"
				" line %d\n", __LINE__);
		goto done;
	}

	if (val & USB_PRES_RISING) {
		twl4030_phy_resume();
		charger_schedule_detection();
		/* twl4030charger_usb_en(1); */
#ifdef CONFIG_MACH_BRISKET
		b_peripheral((struct twl4030_usb *)_twl);
#endif
	} else {
#ifdef CONFIG_MACH_BRISKET
		b_idle((struct twl4030_usb *)_twl);
#endif
		charger_vbus_lost();
		/* twl4030charger_usb_en(0); */
		twl4030_phy_suspend(0);
	}

	ret = IRQ_HANDLED;

done:
	return ret;
}

static int twl4030_set_suspend(struct otg_transceiver *x, int suspend)
{
#ifdef CONFIG_MACH_BRISKET
	/* 
	 * BUG:
	 * On brisket, we can't do this because omap_udc_irq()
	 * calls us on an interrupt context :(
	 */
#else
	if (suspend)
		twl4030_phy_suspend(1);
	else {
		int presence;

		twl4030_phy_resume();
		transceiver_vbus_presence(&presence);
		if (presence)
			charger_schedule_detection();
		else
			twl4030_phy_suspend(0);
	}
#endif
	return 0;
}

static int twl4030_set_peripheral(struct otg_transceiver *xceiv,
		struct usb_gadget *gadget)
{
	u32 l;
	struct twl4030_usb *twl = xceiv_to_twl(xceiv);

	if (!xceiv)
		return -ENODEV;

	if (!gadget) {
		OTG_IRQ_EN_REG = 0;
		twl4030_phy_suspend(1);
		twl->otg.gadget = NULL;

		return -ENODEV;
	}

	twl->otg.gadget = gadget;
	/* we don't have to resume here - toshi */
	/* twl4030_phy_resume(); */

	l = OTG_CTRL_REG & OTG_CTRL_MASK;
	l &= ~(OTG_XCEIV_OUTPUTS|OTG_CTRL_BITS);
	l |= OTG_ID;
	OTG_CTRL_REG = l;

	twl->otg.state = OTG_STATE_B_IDLE;

	twl4030_usb_set_bits(twl, USB_INT_EN_RISE,
			USB_INT_SESSVALID | USB_INT_VBUSVALID);
	twl4030_usb_set_bits(twl, USB_INT_EN_FALL,
			USB_INT_SESSVALID | USB_INT_VBUSVALID);

#ifdef CONFIG_MACH_BRISKET
	if (!twl->asleep)
		b_peripheral(twl);
#endif
	return 0;
}

static int twl4030_set_host(struct otg_transceiver *xceiv, struct usb_bus *host)
{
	struct twl4030_usb *twl = xceiv_to_twl(xceiv);

	if (!xceiv)
		return -ENODEV;

	if (!host) {
		OTG_IRQ_EN_REG = 0;
		twl4030_phy_suspend(1);
		twl->otg.host = NULL;

		return -ENODEV;
	}

	twl->otg.host = host;
	twl4030_phy_resume();

	twl4030_usb_set_bits(twl, TWL4030_OTG_CTRL,
			TWL4030_OTG_CTRL_DMPULLDOWN
				| TWL4030_OTG_CTRL_DPPULLDOWN);
	twl4030_usb_set_bits(twl, USB_INT_EN_RISE, USB_INT_IDGND);
	twl4030_usb_set_bits(twl, USB_INT_EN_FALL, USB_INT_IDGND);
	twl4030_usb_set_bits(twl, FUNC_CTRL, FUNC_CTRL_SUSPENDM);
	twl4030_usb_set_bits(twl, TWL4030_OTG_CTRL, TWL4030_OTG_CTRL_DRVVBUS);

	return 0;
}

int twl4030_usb_mode_switch(struct twl4030_usb *twl, int switch_to, 
		int mux_mode)
{
	twl4030_phy_suspend(1);

	if (mux_mode != -1) {
		omap_usbmux_cfg(mux_mode);
	}

	twl->usb_mode = switch_to;
	/* This function reconfigures the PHY */
	twl4030_phy_resume();

	if (twl->usb_mode == T2_UART_MODE_CARKIT) {
		usb_irq_disable();
	} else {
		int presence;
		transceiver_vbus_presence(&presence);
		if (!presence) {
			twl4030_phy_suspend(0);
		}
	}

	return 0;
}

#ifdef CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
extern struct raw_notifier_head console_key_notifier_list;

static int console_init_timeout = 3;
static struct timer_list console_init_timer;

static void usbmux_timer_cb_wq_func(struct work_struct *work)
{
	struct twl4030_usb *twl = container_of(work, struct twl4030_usb, 
			usbmux_timer_cb_work);

	printk(KERN_INFO "Initiate usbmux switch to uart\n");

	if (omap_usbmux_mode() == USBMUX_UART3) {
		return;
	}

	twl4030_usb_mode_switch(twl, T2_UART_MODE_CARKIT, USBMUX_UART3);
}

static void usbmux_timer_cb(unsigned long data)
{
	struct twl4030_usb *twl = (struct twl4030_usb *)data;
	schedule_work(&twl->usbmux_timer_cb_work);
}

static int usbmux_cb(struct notifier_block *nb, unsigned long val,
		void *ctxt)
{
	if (val) {
		mod_timer(&console_init_timer, 
				jiffies + msecs_to_jiffies(console_init_timeout * 1000));
	} else {
		del_timer(&console_init_timer);
	}

	return 0;
}

static struct notifier_block usbmux_nb = {
	.notifier_call = usbmux_cb,
};
#endif

#undef TWL4030_USBMUX_SYSFS_DEBUG
#define TWL4030_USBMUX_SYSFS_DEBUG    1

#if TWL4030_USBMUX_SYSFS_DEBUG
static ssize_t twl4030_usb_usbmux_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);

	if ((twl->usb_mode == T2_USB_MODE_ULPI) ||
			(twl->usb_mode == T2_USB_MODE_CEA2011_3PIN)) {
		return sprintf(buf, "usb\n");
	} else if (twl->usb_mode == T2_UART_MODE_CARKIT) {
		return sprintf(buf, "uart\n");
	} else {
		return sprintf(buf, "error\n");
	}
}

static ssize_t twl4030_usb_usbmux_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);
	int switch_to = -1;
	omap_usbmux_t mode, new_mode;

	/* Eww, this is pretty gross */

	mode = omap_usbmux_mode();
	new_mode = -1;

	if (strncmp("usb", buf, 3) == 0) {
#if defined(CONFIG_TWL4030_USB_HS_ULPI)
		switch_to = T2_USB_MODE_ULPI;
#elif defined(CONFIG_TWL4030_USB_FS_3_PIN)
		switch_to = T2_USB_MODE_CEA2011_3PIN;
#else
#error "Unsupported configuration!"
#endif
		new_mode = USBMUX_USB;

	} else if (strncmp("uart", buf, 4) == 0) {
		switch_to = T2_UART_MODE_CARKIT;
		new_mode = USBMUX_UART3;
	}

	printk("Switching to %d/%d\n", switch_to, new_mode);

	if (switch_to != -1) {
		twl4030_usb_mode_switch(twl, switch_to, new_mode);
	}

	return count;
}

static ssize_t twl4030_usb_panic_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	//panic("User requested panic\n");
	return count;
}

static ssize_t twl4030_usb_sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", twl->asleep);
}

static ssize_t twl4030_usb_sleep_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct twl4030_usb *twl = dev_get_drvdata(dev);
	if (buf[0] == '0') {
		if (!twl->asleep) {
			twl4030_phy_suspend(0);
		}
		twl4030_phy_resume();
	} else {
		if (twl->asleep) {
			twl4030_phy_resume();
		}
		twl4030_phy_suspend(0);
	}
	return count;
}

DEVICE_ATTR(usbmux, S_IRUGO | S_IWUGO, twl4030_usb_usbmux_show,
		twl4030_usb_usbmux_store);
DEVICE_ATTR(panic, S_IWUGO, NULL, twl4030_usb_panic_store);
DEVICE_ATTR(sleep, S_IRUGO | S_IWUGO, twl4030_usb_sleep_show,
		twl4030_usb_sleep_store);
#endif

static int twl4030_usb_probe(struct platform_device *pdev)
{
#if TWL4030_USBMUX_SYSFS_DEBUG
	int r;
#endif

	platform_set_drvdata(pdev, the_transceiver);

#if TWL4030_USBMUX_SYSFS_DEBUG
	r = device_create_file(&pdev->dev, &dev_attr_usbmux);
	r = device_create_file(&pdev->dev, &dev_attr_panic);
	r = device_create_file(&pdev->dev, &dev_attr_sleep);
#endif

	return 0;
}

static int twl4030_usb_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
#define twl4030_usb_suspend    NULL
static int twl4030_usb_resume(struct platform_device *pdev)
{
	struct twl4030_usb *twl = platform_get_drvdata(pdev);

	if (twl->usb_mode == T2_UART_MODE_CARKIT) {
		twl4030_i2c_access(1);
		twl4030_usb_set_mode(twl, twl->usb_mode);
		twl4030_i2c_access(0);
	}
	return 0;
}
#else
#define twl4030_usb_suspend    NULL
#define twl4030_usb_resume     NULL
#endif

static struct platform_driver twl4030_usb_pdrv = {
	.probe = twl4030_usb_probe,
	.remove = twl4030_usb_remove,
	.suspend = twl4030_usb_suspend,
	.resume = twl4030_usb_resume,
	.driver = {
		.name = "twl4030_usb",
	},
};
static struct platform_device twl4030_usb_pdev = {
	.name = "twl4030_usb",
	.id = -1,
};

static int __init twl4030_usb_init(void)
{
	struct twl4030_usb	*twl;
	int status;

	if (the_transceiver)
		return 0;

	twl = kzalloc(sizeof *twl, GFP_KERNEL);
	if (!twl)
		return 0;

	spin_lock_init(&twl->lock);

	the_transceiver = twl;

	/* Register a platform device/driver to listen for PM messages when
	 * switched to UART mode. */
	platform_device_register(&twl4030_usb_pdev);
	platform_driver_register(&twl4030_usb_pdrv);

	/* If the USB port is muxed to serial mode we do not initialize USB.
	 * omap_usbmux_mode is set to USBMUX_UART3 if the command line
	 *   usbmux=uart3
	 * is given in the Linux boot commmand line.
	 */
	if (omap_usbmux_mode() != USBMUX_USB) {
		printk("twl4030_usb: USB not initializing. SERIAL mux mode is active.\n");
		twl4030_i2c_access(1);
		twl4030_usb_set_mode(twl, T2_UART_MODE_CARKIT);
		twl4030_i2c_access(0);

		return 0;
	}

	twl->irq		= TWL4030_MODIRQ_PWR;
	twl->otg.set_host	= twl4030_set_host;
	twl->otg.set_peripheral	= twl4030_set_peripheral;
	twl->otg.set_suspend	= twl4030_set_suspend;
	twl->otg.set_power	= twl4030_set_power;
	twl->otg.label		= "twl4030";

	usb_irq_disable();
	status = request_irq(twl->irq, twl4030_usb_irq,
		     IRQF_DISABLED | IRQF_SHARED, "twl4030_usb", twl);
	if (status < 0) {
		printk(KERN_DEBUG "can't get IRQ %d, err %d\n",
			twl->irq, status);
		kfree(twl);
		return -ENODEV;
	}

#if defined (CONFIG_TWL4030_USB_HS_ULPI)
	hs_usb_init(twl);
#elif defined (CONFIG_TWL4030_USB_FS_3_PIN)
	fs_usb_init(twl);
#endif
	twl4030_usb_ldo_init(twl);
	twl4030_phy_power(twl, 1);
	twl4030_i2c_access(1);
	twl4030_usb_set_mode(twl, twl->usb_mode);
	if (twl->usb_mode == T2_USB_MODE_ULPI)
		twl4030_i2c_access(0);

	twl->asleep = 0;

#ifdef USE_USB_POWER_CONSTRAINT
	twl->usb_power_constraint = NULL;
#endif

	/* Allow USB presence interrupt as wakeup event */
	if (twl->usb_mode == T2_USB_MODE_ULPI ||
	    twl->usb_mode == T2_USB_MODE_CEA2011_3PIN){
		int presence;

		/* Check for Cold plugging case: if device already connected */
		transceiver_vbus_presence(&presence);
		if (presence) {
			printk(KERN_INFO "twl4030_usb: Device ATTACHED: Cold plugging\n");
			twl4030_phy_suspend(0);
			twl4030_phy_resume();
			charger_schedule_detection();
		} else {
			printk(KERN_INFO "twl4030_usb: Device NOT-ATTACHED at bootup\n");
			/* Suspend PHY but keep PRES int enabled */
			twl4030_phy_suspend(0);
		}
	}

	/* Enable irq wake up */
	enable_irq_wake (twl->irq);

#ifdef USE_USB_POWER_CONSTRAINT
	/* Get handle to USB Constraint for OFF/RETENTION */
	twl->usb_power_constraint = constraint_get("usb", &cnstr_id);
#endif
	otg_set_transceiver(&twl->otg);

	INIT_WORK(&twl->vbus_draw_work, vbus_draw_work);
	INIT_DELAYED_WORK(&twl->phy_suspend_work, phy_suspend_work);

	twl->phy_try_suspending = 0;

#ifdef CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
	INIT_WORK(&twl->usbmux_timer_cb_work, usbmux_timer_cb_wq_func);
	setup_timer(&console_init_timer, usbmux_timer_cb, 
			(unsigned long)twl);
	raw_notifier_chain_register(&console_key_notifier_list, 
			&usbmux_nb);
#endif

	printk(KERN_INFO "Initialized TWL4030 USB module\n");

	return 0;
}


static void __exit twl4030_usb_exit(void)
{
	struct twl4030_usb *twl = the_transceiver;
	int val;

	if (omap_usbmux_mode() != USBMUX_USB) {
		printk("twl4030_usb: USB not de-initializing. SERIAL mux mode is active.\n");
		platform_device_unregister(&twl4030_usb_pdev);
		platform_driver_unregister(&twl4030_usb_pdrv);
		kfree(twl);
		return;
	}

	disable_irq_wake (twl->irq);
	usb_irq_disable();
	free_irq(twl->irq, twl);

	/* set transceiver mode to power on defaults */
	twl4030_usb_set_mode(twl, -1);

	/* autogate 60MHz ULPI clock,
	 * clear dpll clock request for i2c access,
	 * disable 32KHz
	 */
	val = twl4030_usb_read(PHY_CLK_CTRL);
	if (val >= 0) {
		val |= PHY_CLK_CTRL_CLOCKGATING_EN;
		val &= ~(PHY_CLK_CTRL_CLK32K_EN | REQ_PHY_DPLL_CLK);
		twl4030_usb_write(PHY_CLK_CTRL, (u8)val);
	}

	/* disable complete OTG block */
	twl4030_usb_clear_bits(twl, POWER_CTRL, POWER_CTRL_OTG_ENAB);

	twl4030_phy_power(twl, 0);

#ifdef USE_USB_POWER_CONSTRAINT
	if (twl->usb_power_constraint)
		constraint_put(twl->usb_power_constraint);
#endif

	kfree(twl);
}

subsys_initcall(twl4030_usb_init);
module_exit(twl4030_usb_exit);

MODULE_ALIAS("i2c:twl4030-usb");
MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("TWL4030 USB transceiver driver");
MODULE_LICENSE("GPL");
