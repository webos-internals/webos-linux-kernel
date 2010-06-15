/*
 * arch/arm/mach-omap2/board-joplin.c
 *
 * This file contains code specific to the Joplin board.
 *
 * Copyright (C) 2007 Palm, Inc.
 *
 * Based on from mach-omap2/board-2430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/major.h>
#include <linux/usb/musb.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2005.h>
#include <linux/gpio_keypad.h>
#include <linux/hsuart.h>
#include <linux/user-pins.h>
#include <linux/fb.h>
#include <linux/nduid.h>

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
#include <linux/modem_activity.h>
#endif

#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/dma.h>
#include <asm/arch/irda.h>
#include <asm/arch/clock.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/nand.h>
#include <asm/arch/mcspi.h>
#include <asm/arch/twl4030-rtc.h>
#include <asm/arch/power_companion.h>
#include <asm/arch/usb.h>
#include <asm/arch/display.h>
#include <asm/arch/lcd.h>

#ifdef CONFIG_MMC
extern struct omap_mmc_config board_mmc_config;
#endif  // CONFIG_MMC

/*
 *  UART support
 */
static struct omap_uart_config joplin_uart_config __initdata  = {
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

#if defined(CONFIG_OMAP_MISC_HSUART) || defined(CONFIG_OMAP_MISC_HSUART_MODULE)

static u64 mduart_dmamask = ~(u32)0;
static struct hsuart_platform_data mduart_data = {
	.dev_name    = "modemuart",
	.uart_mode   = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed  = HSUART_SPEED_115K,
	.options     = HSUART_OPTION_DEFERRED_LOAD 
	             | HSUART_OPTION_MODEM_DEVICE,
	.tx_buf_size = 1024,
	.tx_buf_num  = 16, 
	.rx_buf_size = 1024,
	.rx_buf_num  = 32,
	.max_packet_size = 1024, //
	.min_packet_size = 8,    // min packet size
	.rx_latency      = 128,  // in bytes at current speed
	.rts_pin         = 60,   // uart rts line pin
	.rts_act_mode    = "AB24_2430_UART1_RTS",  // uart rts line active mode 
	.rts_gpio_mode   = "AB24_2430_UART1_GPIO", // uart rts line gpio mode 
};

static struct platform_device mduart_device = {
	.name = "omap_hsuart",
	.id   =  0, // configure UART1 as an hi speed uart
	.dev  = {
		.dma_mask           = &mduart_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = &mduart_data,
	}
};

static u64 btuart_dmamask = ~(u32)0;

static struct hsuart_platform_data btuart_data = {
	.dev_name   = "btuart",
	.uart_mode  = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = HSUART_SPEED_115K,
	.options    = HSUART_OPTION_DEFERRED_LOAD,
	.tx_buf_size = 1024,
	.tx_buf_num  = 8,
	.rx_buf_size = 1024,
	.rx_buf_num  = 32,
	.max_packet_size = 450, // ~450
	.min_packet_size = 6,   // min packet size
	.rx_latency      = 512, // in bytes at current speed
	.rts_pin         = 78,  // uart rts line pin
	.rts_act_mode    = "AD10_2430_UART2_RTS",  // uart rts line active mode 
	.rts_gpio_mode   = "AD10_2430_UART2_GPIO", // uart rts line gpio mode 
};

static struct platform_device btuart_device = {
	.name = "omap_hsuart",
	.id   =  1, // configure UART2 as an hi speed uart
	.dev  = {
		.dma_mask           = &btuart_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = &btuart_data,
	}
};
#endif

/*
 *  USB support
 */
#ifdef CONFIG_USB_MUSB_HDRC

static struct resource musb_resources[] = {
#ifdef CONFIG_ARCH_OMAP34XX
	[0] = {
		.start  = HS_BASE,
		.end    = HS_BASE + SZ_8K,
		.flags  = IORESOURCE_MEM,
	},
#else
	[0] = {
		.start  = OMAP243X_HS_BASE,
		.end    = OMAP243X_HS_BASE + SZ_8K,
		.flags  = IORESOURCE_MEM,
},
#endif
	[1] = {    /* general IRQ */
		.start = INT_243X_HS_USB_MC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {    /* DMA IRQ */
		.start = INT_243X_HS_USB_DMA,
		.flags = IORESOURCE_IRQ,
	},
};

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode = MUSB_PERIPHERAL,
#endif
	.multipoint = 1,
	.clock      = NULL,
	.set_clock  = NULL,
};

static u64 musb_dmamask = ~(u32)0;

static struct platform_device musb_device = {
	.name = "musb_hdrc",
	.id   = 0,
	.dev  = {
		.dma_mask          = &musb_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data     = &musb_plat,
	},
	.num_resources = ARRAY_SIZE(musb_resources),
	.resource      = musb_resources,
};

static void __init joplin_usb_init(void)
{
	if (platform_device_register(&musb_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
		return;
	}
}


#elif  defined(CONFIG_USB_OMAP)

static void usb_release(struct device *dev)
{
	/* normally not freed */
}

// Full speed usb controller

/* USB Port 0 is a device. USB Port 1 is the host.  This is the only
   valid FS dual port configuration, as ports 1 and 2 can only be
   hosts. */
static struct omap_usb_config fsusb_config = {
	.otg           = 0,
};


// Device
#ifdef	CONFIG_USB_GADGET_OMAP

static struct resource udc_resources[] = {
	/* order is significant! */
	{    /* registers */
		.start   = UDC_BASE,
		.end     = UDC_BASE + 0xff,
		.flags   = IORESOURCE_MEM,
	}, {  /* general IRQ */
		.start   = INT_24XX_USB_IRQ_GEN,
		.flags   = IORESOURCE_IRQ,
	}, { /* PIO IRQ */
		.start   = INT_24XX_USB_IRQ_NISO,
		.flags   = IORESOURCE_IRQ,
	}, { /* SOF IRQ */
		.start   = INT_24XX_USB_IRQ_ISO,
		.flags   = IORESOURCE_IRQ,
	},
};

static u64 udc_dmamask = ~(u32)0;

static struct platform_device udc_device = {
	.name = "omap_udc",
	.id   = -1,
	.dev  = {
		.release  = usb_release,
		.dma_mask = &udc_dmamask,
		.coherent_dma_mask = 0xffffffff,
	    .platform_data = &fsusb_config,
	},
	.num_resources = ARRAY_SIZE(udc_resources),
	.resource      = udc_resources,
};

#endif


// Host
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32)0;

static struct resource ohci_resources[] = {
	{
		.start  = OMAP_OHCI_BASE,
		.end    = OMAP_OHCI_BASE + 0x1ff,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_24XX_USB_IRQ_HGEN,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device ohci_device = {
	.name = "ohci",
	.id   = -1,
	.dev  = {
		.release   = usb_release,
		.dma_mask  = &ohci_dmamask,
		.coherent_dma_mask = 0xffffffff,
	    .platform_data = &fsusb_config,
	},
	.num_resources = ARRAY_SIZE(ohci_resources),
	.resource      = ohci_resources,
};

#endif

static void __init modem_init(void)
{
	struct clk *usbclk;

	// Pin config for Modem Control
	omap_cfg_reg("R25_2430_SYS_CLKOUT");

	omap_cfg_reg("U25_2430_USB1_RCV");
	omap_cfg_reg("T23_2430_USB1_TXEN");
	omap_cfg_reg("U20_2430_USB1_SE0");
	omap_cfg_reg("T24_2430_USB1_DAT");

	omap_cfg_reg("AD24_2430_UART1_TX");
	omap_cfg_reg("AB24_2430_UART1_RTS");
	omap_cfg_reg("Y25_2430_UART1_CTS");
	omap_cfg_reg("AA26_2430_UART1_RX");

	// Configure SYS_CLKOUT for the 48 MHz USB clock

#define PRCM_CLKOUT_CTRL_CLKOUT_EN          (1 << 7)
#define PRCM_CLKOUT_CTRL_CLKOUT_DIV_2       (1 << 3)
#define PRCM_CLKOUT_CTRL_CLKOUT_SOURCE_96MHZ    2

#define PRCM_BASE             (OMAP2_PRCM_BASE)
#define PRCM_REG32(offset)  __REG32(PRCM_BASE + (offset))
#define PRCM_CLKOUT_CTRL      PRCM_REG32(0x070)

	__raw_writel(PRCM_CLKOUT_CTRL_CLKOUT_EN |
	             PRCM_CLKOUT_CTRL_CLKOUT_DIV_2 |
	             PRCM_CLKOUT_CTRL_CLKOUT_SOURCE_96MHZ, (int)&PRCM_CLKOUT_CTRL);

	usbclk = clk_get(NULL, "sys_clkout");
	clk_enable(usbclk);
}

static void __init joplin_usb_init(void)
{
	u32  syscon1 = OTG_SYSCON_1_REG & 0xffff;
	u32  syscon2;
	if(!(syscon1 & OTG_RESET_DONE))
		printk(  KERN_WARNING "USB reset is not complete\n");

	fsusb_config.otg          = 0;    /* not using otg */
	fsusb_config.register_dev = 1;    /* usb0 is a device */
	fsusb_config.register_host= 1;    /* usb1 is a host */
	fsusb_config.hmc_mode     = 0x14; /* 0:device 1:host 2:disable */
	fsusb_config.pins[0]      = 3;    /* usb0 is a 3 wire device */
	fsusb_config.pins[1]      = 2;    /* usb1 is a 2 wire device */
	fsusb_config.pins[2]      = 0;    /* usb2 is unsupported by driver */

	// the following code assumes the above configuration

	// usb0 ( 3-pin mode )
	CONTROL_DEVCONF_REG &= ~USBT0WRMODEI(USB_BIDIR_TLL);
	CONTROL_DEVCONF_REG |=  USBT0WRMODEI(USB_BIDIR);
	syscon1 |= 2 << 16; // 3 pin mode

	// usb1 ( bidirectional TLL DAT/SE0 mode )
	CONTROL_DEVCONF_REG |=  USBT1WRMODEI(USB_BIDIR_TLL);
	syscon1 |= 2 << 20; // 3 pin DAT/SE0 mode

	// usb2 (unused)
	CONTROL_DEVCONF_REG &= ~(USBT2WRMODEI(USB_BIDIR_TLL) | USBT2TLL5PI);

	CONTROL_DEVCONF_REG |= USBSTANDBYCTRL; /* TODO: is this the right place? */

	OTG_SYSCON_1_REG = syscon1;

	// OTG_SYSCON_2
	syscon2 = fsusb_config.hmc_mode | USBX_SYNCHRO | (4<<16) /*B_ASE_BRST*/ | UHOST_EN;
	syscon2 |= HMC_TLLSPEED | HMC_TLLATTACH;
	OTG_SYSCON_2_REG = syscon2;

	// disable all
	syscon1 |= HST_IDLE_EN | DEV_IDLE_EN | OTG_IDLE_EN;

#ifdef	CONFIG_USB_GADGET_OMAP
	syscon1 &= ~DEV_IDLE_EN;
	if (platform_device_register(&udc_device) < 0) {
		printk(KERN_ERR "Unable to register FS-USB (UDC) device\n");
		return;
	}
#endif

#if	defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	syscon1 &= ~HST_IDLE_EN;
	if (platform_device_register(&ohci_device) < 0) {
		printk(KERN_ERR "Unable to register FS-USB (HCD) device\n");
		return;
	}
#endif

	OTG_SYSCON_1_REG = syscon1;
}

#else  /* CONFIG_USB_OMAP */

static void __init modem_init(void) {}
static void __init joplin_usb_init(void) {}

#endif /* ALL USB */

static void __init joplin_mcbsp_init(void)
{
	/* TWL4030 TDM/I2S */
	omap_cfg_reg("AC10_2430_MCBSP2_FSX");
	omap_cfg_reg("AD16_2430_MCBSP2_CLX");
	omap_cfg_reg("AE13_2430_MCBSP2_DX");
	omap_cfg_reg("AD13_2430_MCBSP2_DR");

	/* BT */
	omap_cfg_reg("AC25_2430_MCBSP4_FSX");
	omap_cfg_reg("N3_2430_MCBSP4_CLX");
	omap_cfg_reg("AB25_2430_MCBSP4_DX");
	omap_cfg_reg("AD23_2430_MCBSP4_DR");

	/* Modem */
	omap_cfg_reg("AF12_2430_MCBSP5_FSX");
	omap_cfg_reg("AE16_2430_MCBSP5_CLX");
	omap_cfg_reg("K7_2430_MCBSP5_DX");
	omap_cfg_reg("M1_2430_MCBSP5_DR");
}


/*
 *  I2C subsystem
 */
#if  defined(CONFIG_I2C_OMAP)

#define OMAP2_I2C_BASE1   0x48070000
#define OMAP2_I2C_BASE2   0x48072000
#define OMAP2_I2C_INT1    56
#define OMAP2_I2C_INT2    57

static u32 omap2_i2c1_clkrate =  100;
static u32 omap2_i2c2_clkrate = 2600; // it is connected to TWL4030

static struct resource i2c_resources1[] = {
	{
		.start  = OMAP2_I2C_BASE1,
		.end    = OMAP2_I2C_BASE1 + 0x3f,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = OMAP2_I2C_INT1,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct resource i2c_resources2[] = {
	{
		.start  = OMAP2_I2C_BASE2,
		.end    = OMAP2_I2C_BASE2 + 0x3f,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = OMAP2_I2C_INT2,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device omap_i2c_device1 = {
	.name  = "i2c_omap",
	.id    =  1,
	.num_resources = ARRAY_SIZE(i2c_resources1),
	.resource = i2c_resources1,
	.dev      = {
		.platform_data = &omap2_i2c1_clkrate,
	},
};

static struct platform_device omap_i2c_device2 = {
	.name = "i2c_omap",
	.id   =  2,
	.num_resources = ARRAY_SIZE(i2c_resources2),
	.resource  = i2c_resources2,
	.dev       = {
		.platform_data    = &omap2_i2c2_clkrate,
	},
};

static int __init joplin_init_i2c(void)
{
	(void) platform_device_register(&omap_i2c_device1);
	(void) platform_device_register(&omap_i2c_device2);
	return 0;
}
#else

static int __init joplin_init_i2c(void) { return 0; }

#endif // I2C


/*
 *  Joplin keypad
 */
static int joplin_keymap[] = {
     /*0*/       /*1*/          /*2*/         /*3*/       /*4*/       /*5*/          /*6*/           /*7*/
/*0*/KEY_END,      KEY_F1,        KEY_F2,       KEY_F3,     KEY_F4,  KEY_CENTER,   KEY_PTT,        KEY_SEND,
/*1*/KEY_LEFT,     KEY_RIGHT,     KEY_UP,       KEY_DOWN,   KEY_P,   KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_O,
/*2*/KEY_RIGHTALT, KEY_LEFTSHIFT, KEY_0,        KEY_SPACE,  KEY_ALT, KEY_DOT,      KEY_ENTER,      KEY_BACKSPACE,
/*3*/KEY_L,        KEY_Z,         KEY_X,        KEY_C,      KEY_V,   KEY_B,        KEY_N,          KEY_M,
/*4*/KEY_A,        KEY_S,         KEY_D,        KEY_F,      KEY_G,   KEY_H,        KEY_J,          KEY_K,
/*5*/KEY_Q,        KEY_W,         KEY_E,        KEY_R,      KEY_T,   KEY_Y,        KEY_U,          KEY_I,
/*6*/0,            KEY_MENU,      KEY_LAUNCHER, 0,          0,       0,            0,              0
};

static int joplin_kp_gpio_rows[] = {3, 4,  5,  6, 29, 30, 31};
static int joplin_kp_gpio_cols[] = {8, 9, 10, 24, 25, 26, 27, 28};

static struct gpio_kp_config joplin_kp_data = {
	.output_num  = ARRAY_SIZE(joplin_kp_gpio_rows),
	.outputs     = joplin_kp_gpio_rows,
	.input_num   = ARRAY_SIZE(joplin_kp_gpio_cols),
	.inputs      = joplin_kp_gpio_cols,
	.keymap_cfg  = GPIO_KEYPAD_CFG_ROW_IS_OUTPUT,
	.keymap      = joplin_keymap,
	.rep_period  = 100,   // 100 ms repeat time (10 cps)
	.rep_delay   = 500,   // 500 ms repeat delay
	.gpio_delay  = 5,     // 5  usec to stabilize reading
	.debounce    = 20,    // 20 msec to stabilize reading
	.wakeup_row  = 0,     // row 0 can wakeup device
	.wakeup_mask = 0xFF   // all buttons on row 0 can wakeup device
};

static struct platform_device joplin_kp_device = {
	.name  = "gpio_keypad",
	.id    = -1,
	.dev   =  {
		.platform_data    = &joplin_kp_data,
	},
};

static u32 disp_vsync_gated = 1;
static u32 disp_use_sdi = 0;

static struct platform_device board_lcd_device = {
	.name = "lcd",
	.id = 0,
};

static struct controller_platform_data board_lcd_controller_data = {
	.parent = &(board_lcd_device.dev),
	.screen_info 	= {
		.xres 		= 320,
		.yres 		= 320,
		.xres_virtual 	= 320,
		.yres_virtual 	= (320 * 3),	/* triple buffering */
		.xoffset 	= 0,
		.yoffset 	= 0,
		.bits_per_pixel = 16,
		.grayscale 	= 0,
		.red 		= {11, 5, 0},
		.green 		= {5, 6, 0},
		.blue 		= {0, 5, 0},
		.transp 	= {0, 0, 0},
		.nonstd 	= 0,
		.activate 	= FB_ACTIVATE_NOW,
		.height 	= -1,
		.width 		= -1,
		.accel_flags 	= 0,
		.pixclock 	= 185186,		/* picoseconds */
		.left_margin 	= 19,			/* pixclocks   */
		.right_margin 	= 15,			/* pixclocks   */
		.upper_margin 	= 7,			/* line clocks */
		.lower_margin 	= 6,			/* line clocks */
		.hsync_len 	= 4,			/* pixclocks   */
		.vsync_len 	= 2,			/* line clocks */
		.sync 		= 0,
		.vmode 		= FB_VMODE_NONINTERLACED,
		.rotate 	= 0,
		.reserved[0] 	= 0,
	},
	.use_sdi 	= &disp_use_sdi,
	.vsync_gated 	= &disp_vsync_gated,

	.dss_reg_base	= DSS_REG_BASE,
	.dss_reg_offset	= DSS_REG_OFFSET,
	.dispc_reg_offset = DISPC_REG_OFFSET,

	.pixclk_min	= 185186,
	.pixclk_max	= 138888,
};

static struct platform_device board_lcd_controller = {
	.name = "lcd-controller",
	.id = 0,
	.dev = {
		.platform_data = &board_lcd_controller_data,
	},
};
static struct bl_platform_data board_bl_data = {
	.parent = &(board_lcd_device.dev),
	.timer_id 	= 12,
	.bl_params	= {
		.min_brightness = 0,
		.max_brightness = 100,
		.should_fade_out= 0,
		.fade_out_msecs	= 1024,
		.fade_out_step	= 5,
		.should_fade_in	= 0,
		.fade_in_msecs	= 1024,
		.fade_in_step	= 5,
	},
};
static struct platform_device board_lcd_backlight = {
	.name = "lcd-backlight",
	.id = -1,
	.dev = {
		.platform_data = &board_bl_data,
	},
};

static u32 dummy0 = 0;
static u32 dummy1 = 1;
static struct panel_platform_data board_panel_data = {
	.parent = &board_lcd_device.dev,
	.panel_use_gpio_enable = &dummy1,
	.panel_enable_gpio     = 85,
	.panel_use_gpio_reset  = &dummy0,
};

/*
 *  SPI devices
 */
static struct omap2_mcspi_device_config touchpanel_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,
};

static struct omap2_mcspi_device_config lcdpanel_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,
};

static struct  tsc2005_platform_data touchpanel_platform_data =
{
	.reset_gpio    = 102,   // connected to GPIO 102
	.reset_level   = 0,     // active low
	.xresistence   = 100,   //
	.penup_timeout = 40,    // msec
};

static struct spi_board_info joplin_spi_board_info[] __initdata = {
	{
		.modalias        = "tsc2005",
		.bus_num         = 1,
		.chip_select     = 0,
		.max_speed_hz    = 6000000,
		.platform_data   = &touchpanel_platform_data,
		.controller_data = &touchpanel_mcspi_config,
		.irq             = OMAP_GPIO_IRQ ( 95 ),
	},
	{
		.modalias        = "lcdpanel",
		.bus_num         = 3,
		.chip_select     = 0,
		.max_speed_hz    = 1500000,
		.platform_data   = &board_panel_data,
		.controller_data = &lcdpanel_mcspi_config,
	}
};

/*
 *  1-wire devices
 */
#ifdef CONFIG_W1_MASTER_OMAP
/* HDQ clock shutdown timeout in milliseconds. */
static u32 hdq_shutdown_timeout = 100;

static struct resource hdq_resources1[] = {
	{
		.start  = OMAP2_HDQ_BASE,
		.end    = OMAP2_HDQ_BASE + 0x18,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_24XX_HDQ_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device omap_hdq_device1 = {
	.name = "omap_hdq",
	.id   = 0,
	.num_resources = ARRAY_SIZE(hdq_resources1),
	.resource      = hdq_resources1,
	.dev = {
		.platform_data = &hdq_shutdown_timeout,
	}
};
#endif

/*
 *  RTC
 */
#ifdef CONFIG_RTC_DRV_TWL4030

#define TWL4030_MSECURE_GPIO 118

static int
twl4030_rtc_init(void)
{
	int ret = 0;

	ret = omap_request_gpio ( TWL4030_MSECURE_GPIO );
	if (ret < 0) {
		printk(KERN_ERR "twl4030_rtc_init: can't reserve GPIO:%d !\n",
			TWL4030_MSECURE_GPIO);
		goto out;
	}
	/*
	 * TWL4030 will be in secure mode if msecure line from OMAP is low.
	 * Make msecure line high in order to change the TWL4030 RTC time
	 * and calender registers.
	 */
	omap_set_gpio_direction ( TWL4030_MSECURE_GPIO, 0);	/*dir out */
	omap_set_gpio_dataout   ( TWL4030_MSECURE_GPIO, 1);
out:
	return ret;
}

static void
twl4030_rtc_exit(void)
{
	omap_free_gpio(TWL4030_MSECURE_GPIO);
}

static struct twl4030rtc_platform_data joplin_twl4030rtc_data = {
	.init = &twl4030_rtc_init,
	.exit = &twl4030_rtc_exit,
};

static struct platform_device joplin_twl4030rtc_device = {
	.name = "twl4030_rtc",
	.id   = -1,
	.dev  = {
		.platform_data	= &joplin_twl4030rtc_data,
	},
};

#endif // CONFIG_RTC_DRV_TWL4030

/*
 *  NAND
 */
#ifdef CONFIG_MTD_NAND
#define NAND_BLOCK_SIZE  SZ_128K

static struct mtd_partition
joplin_static_nand_partitions[] = {
	{.name = "xload",
	 .offset = 0,
	 .size = 1 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "xload2",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 1 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "uboot",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 3 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "tokens",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 5 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "mfgdata",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 16 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "bootlogo",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 2 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "uboot2",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 3 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 16 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "kernel2",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 16 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "recoveryrootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 32 * NAND_BLOCK_SIZE,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 1600 * NAND_BLOCK_SIZE,
//	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	},
	{.name = "userfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
//	 .mask_flags = MTD_WRITEABLE    /* force read-only */
	}
};

static struct omap_nand_platform_data joplin_nand_data =
{
	.cs        = 0,
	.parts     = joplin_static_nand_partitions,
	.nr_parts  = ARRAY_SIZE(joplin_static_nand_partitions),
    .gpmcBaseAddr = (void __iomem *) IO_ADDRESS(GPMC_BASE),
};

static struct platform_device joplin_nand_device = {
	.name = "omap2_nand",
	.id   = 0,
	.dev  = {
		.platform_data = &joplin_nand_data,
	}
};
#endif // CONFIG_MTD_NAND


/*
 *  User pins device
 */
#ifdef CONFIG_USER_PINS

#define MODEM_POWER_ON               152  // OUT
#define MODEM_POWER_ON_PIN_MODE       NULL

#define MODEM_BOOT_MODE              153  // OUT
#define MODEM_BOOT_MODE_PIN_MODE      NULL

#define MODEM_WAKEUP_MODEM            87  // OUT
#define MODEM_WAKEUP_MODEM_PIN_MODE   NULL

#define MODEM_WAKEUP_APP              49  // IN
#define MODEM_WAKEUP_APP_PIN_MODE     NULL

//#define MODEM_WAKE_USB               155 // IN
//#define MODEM_WAKE_USB_PIN_MODE       NULL

static struct user_pin modem_pins[] = {
	{
		.name       = "power_on",
		.gpio       =  MODEM_POWER_ON,  //
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  0,  // initialy low
		.pin_mode   =  MODEM_POWER_ON_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       = "boot_mode",
		.gpio       =  MODEM_BOOT_MODE,  //
		.act_level  =  -1,  // not applicable
		.direction  =  0,   // an output
		.def_level  =  0,   // initially low
		.pin_mode   =  MODEM_BOOT_MODE_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       =  "wakeup_modem",
		.gpio       =  MODEM_WAKEUP_MODEM,  //
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  0,  // initialy low
		.pin_mode   =  MODEM_WAKEUP_MODEM_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       =  "wakeup_app",
		.gpio       =  MODEM_WAKEUP_APP,
		.act_level  =  1,  // active high
		.direction  =  1,  // an output
		.def_level  = -1,  // unset
		.pin_mode   =  MODEM_WAKEUP_APP_PIN_MODE,
		.sysfs_mask =  0777,
	},
};


/*
 *   Bt Pins
 */
#define BT_RESET_GPIO                134
#define BT_RESET_GPIO_PIN_MODE        NULL

#define BT_WAKE_GPIO                 136
#define BT_WAKE_GPIO_PIN_MODE         NULL

#define BT_HOST_WAKE_GPIO             94
#define BT_HOST_WAKE_GPIO_PIN_MODE    NULL

static struct user_pin bt_pins[] = {
	{
		.name       =  "reset",
		.gpio       =  BT_RESET_GPIO,
		.act_level  =  0, // active low
		.direction  =  0, // an output
		.def_level  =  0, // default level (low)
		.pin_mode   =  BT_RESET_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       =  "wake",
		.gpio       =  BT_WAKE_GPIO,
		.act_level  =  0, // active Low
		.direction  =  0, // an output
		.def_level  =  0, // default level (low)
		.pin_mode   =  BT_WAKE_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       =  "host_wake",
		.gpio       =  BT_HOST_WAKE_GPIO,  //
		.act_level  =  1,  // active high
		.direction  =  1,  // an input
		.def_level  = -1,  // undefined
		.pin_mode   =  BT_HOST_WAKE_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
};

static struct user_pin_set  board_user_pins_sets[] = {
	{
		.set_name = "bt",
		.num_pins = ARRAY_SIZE(bt_pins),
		.pins     = bt_pins,
	},
	{
		.set_name = "modem",
		.num_pins = ARRAY_SIZE(modem_pins),
		.pins     = modem_pins,
	},
};

static struct user_pins_platform_data board_user_pins_pdata = {
	.num_sets = ARRAY_SIZE(board_user_pins_sets),
	.sets     = board_user_pins_sets,
};

static struct platform_device joplin_user_pins_device = {
	.name = "user-pins",
	.id   = -1,
	.dev  = {
		.platform_data	= &board_user_pins_pdata,
	}
};
#endif


#ifdef CONFIG_LEDS_OMAP_PWM
static struct omap_pwm_led_platform_data joplin_keypad_led_data = {
	.name = "keypad",
	.intensity_timer = 11,
	.blink_timer = 0,
	.set_power = NULL,
};

static struct platform_device joplin_keypad_led_device = {
	.name = "omap_pwm_led",
	.id   = -1,
	.dev  = {
		.platform_data = &joplin_keypad_led_data,
	},
};
#endif

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
/*
 * MODEM handshaking activity monitor.
 */
static struct modem_activity_monitor_config modem_activity_cfg = {
	.app_wake_modem_gpio = MODEM_WAKEUP_MODEM,
	.modem_wake_app_gpio = MODEM_WAKEUP_APP,
	.modem_wake_usb_gpio = -1,
#ifdef CONFIG_USER_PINS
	// if we are using user pins interface mark pins as configured
	.gpio_flags          = GPIO_FLG_AWM_CONFIGURED |
	                       GPIO_FLG_MWA_CONFIGURED |
	                       GPIO_FLG_MWU_CONFIGURED,
#endif
	.uart    = 0,        // uart to monitor
	.timeout = 120,      // timeout in msec 
};

static struct platform_device modem_activity_device = {
        .name           = "modem_activity",
        .id             = -1,
        .dev            = {
                .platform_data  = &modem_activity_cfg,
        },
};
#endif

#ifdef CONFIG_PALM_NDUID
extern unsigned int omap_nduid_get_device_salt(void);
extern int omap_nduid_get_cpu_id(char *id, unsigned int maxlen);

static struct nduid_config nduid_cfg[] = {
	{
		.dev_name = "nduid",
		.get_device_salt = omap_nduid_get_device_salt,
		.get_cpu_id = omap_nduid_get_cpu_id,
	},
};

static struct platform_device nduid_device = {
	.name = "nduid",
	.id = -1,
	.dev = {
		.platform_data = &nduid_cfg,
	},
};
#endif

/*
 *  Joplin devices
 */
static struct platform_device *joplin_devices[] __initdata = {
#ifdef CONFIG_USER_PINS
	&joplin_user_pins_device,
#endif
#ifdef CONFIG_MTD_NAND
	&joplin_nand_device,
#endif
	&joplin_kp_device,
#ifdef CONFIG_RTC_DRV_TWL4030
	&joplin_twl4030rtc_device,
#endif
#ifdef CONFIG_LEDS_OMAP_PWM
	&joplin_keypad_led_device,
#endif
#ifdef CONFIG_W1_MASTER_OMAP
	&omap_hdq_device1,
#endif
#if defined(CONFIG_OMAP_MISC_HSUART) || defined(CONFIG_OMAP_MISC_HSUART_MODULE)
	&mduart_device,
	&btuart_device,
#endif
#ifdef CONFIG_FB_OMAP
	&board_lcd_device,
	&board_lcd_controller,
	&board_lcd_backlight,
#endif
#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
	&modem_activity_device,
#endif
#ifdef CONFIG_PALM_NDUID
	&nduid_device,
#endif
};

/*
 *  Joplin configs
 */

static struct omap_board_config_kernel  joplin_config[] __initdata = {
	{ OMAP_TAG_UART,    &joplin_uart_config },
#ifdef CONFIG_USB_OMAP
	{ OMAP_TAG_USB,     &fsusb_config },
#endif
#ifdef CONFIG_MMC
	{ OMAP_TAG_MMC,     &board_mmc_config },
#endif
};

/*
 *  Map common IO
 */
static void __init joplin_map_io ( void )
{
	omap2_set_globals_243x();
	omap2_map_common_io();
}

/*
 *  Fixup Joplin machine
 */
static void __init
joplin_fixup( struct machine_desc *desc, struct tag *t,
              char **cmdline, struct meminfo *mi)
{
	if (t->hdr.tag != ATAG_CORE) {
		t->hdr.tag = ATAG_CORE;
		t->hdr.size = tag_size(tag_core);
		t->u.core.flags = 0;
		t->u.core.pagesize = PAGE_SIZE;
		t->u.core.rootdev = RAMDISK_MAJOR << 8 | 0;
		t = tag_next(t);

		t->hdr.tag = ATAG_MEM;
		t->hdr.size = tag_size(tag_mem32);
		t->u.mem.start = PHYS_OFFSET;
		t->u.mem.size  = 32 * 1024 * 1024;
		t = tag_next(t);

#ifdef	  CONFIG_BLK_DEV_RAM
		t->hdr.tag = ATAG_RAMDISK;
		t->hdr.size = tag_size(tag_ramdisk);
		t->u.ramdisk.flags = 1;
		t->u.ramdisk.size = CONFIG_BLK_DEV_RAM_SIZE;
		t->u.ramdisk.start = 0;
		t = tag_next(t);
#endif

		t->hdr.tag = ATAG_NONE;
		t->hdr.size = 0;
	}

	return;
}

static void __init joplin_mux ( void )
{
	omap_cfg_reg("Y19_2430_LCD_LED_PWM");
	omap_cfg_reg("AD19_2430_KEYPAD_LED_PWM");
#ifdef CONFIG_W1_MASTER_OMAP
	omap_cfg_reg("H20_24XX_HDQ_SIO");
#endif
}

/*
 *  Joplin init irq
 */
static void __init joplin_init_irq ( void )
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	joplin_mux();
}

/*
 *  Init Joplin mach
 */
static void __init joplin_init ( void )
{
	/* set omap config */
	omap_board_config      = joplin_config;
	omap_board_config_size = ARRAY_SIZE(joplin_config);

	/* Serial */
	omap_serial_init();

	/* add platform devices */
	platform_add_devices ( joplin_devices, ARRAY_SIZE(joplin_devices));

	/* modem control */
	modem_init();

	/* USB */
	joplin_usb_init();

	/* SPI */
	spi_register_board_info( joplin_spi_board_info,
	                         ARRAY_SIZE(joplin_spi_board_info));

	/* McBSP for audio */
	joplin_mcbsp_init();
}

arch_initcall(joplin_init_i2c);

MACHINE_START(BRISKET, "BRISKET (aka Joplin OMAP2430)")
	.phys_io      = JOPLIN_PHYS_IO_BASE,
	.io_pg_offst  = (((JOPLIN_PG_IO_VIRT)>>18)  & 0xfffc ),
	.boot_params  = JOPLIN_RAM_BASE+0x100,
	.fixup        = joplin_fixup,
	.map_io       = joplin_map_io,
	.init_irq     = joplin_init_irq,
	.init_machine = joplin_init,
	.timer        = &omap_timer,
MACHINE_END

