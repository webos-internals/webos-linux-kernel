/*
 * arch/arm/mach-omap2/board-joplin.c
 *
 * This file contains code specific to the Joplin board.
 *
 * Copyright (C) 2007 Palm, Inc.
 *
 * Based on from board-joplin.c
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
#include <linux/i2c.h>
#include <linux/i2c_maxim7359_keypad.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2005.h>
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
static struct omap_uart_config joplin_uart_config __initdata = {
	/* all 3 uarts are enabled */
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

#if defined(CONFIG_OMAP_MISC_HSUART) || defined(CONFIG_OMAP_MISC_HSUART_MODULE)

static u64 mduart_dmamask = ~(u32)0;
static struct hsuart_platform_data mduart_data = {
	.dev_name   = "modemuart",
	.uart_mode  = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = HSUART_SPEED_115K,
	.options    = HSUART_OPTION_DEFERRED_LOAD 
	            | HSUART_OPTION_MODEM_DEVICE,
	.tx_buf_size = 1024,
	.tx_buf_num  = 16, 
	.rx_buf_size = 1024,
	.rx_buf_num  = 32,
	.max_packet_size = 1024, //
	.min_packet_size = 8,    // min packet size
	.rx_latency      = 128,  // in bytes at current speed
	.rts_pin         = 149,  // uart rts line pin
	.rts_act_mode    = "AA9_3430_UART1_RTS",  // uart rts line active mode 
	.rts_gpio_mode   = "AA9_3430_UART1_GPIO", // uart rts line gpio mode 
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
	.rts_pin         = 145,   // uart rts line pin
	.rts_act_mode    = "AB25_3430_UART2_RTS",  // uart rts line active mode 
	.rts_gpio_mode   = "AB25_3430_UART2_GPIO", // uart rts line gpio mode 
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


/*
 *  MUSB Controller Platform data
 */
#ifdef CONFIG_USB_MUSB_HDRC
static struct resource musb_resources[] = {
	[0] = {
		.start = HS_BASE,
		.end   = HS_BASE + SZ_8K,
		.flags = IORESOURCE_MEM,
	},
	[1] = { /* general IRQ */
		.start = INT_243X_HS_USB_MC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = { /* DMA IRQ */
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
	.dev = {
		.dma_mask  = &musb_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data     = &musb_plat,
	},
	.num_resources = ARRAY_SIZE(musb_resources),
	.resource      = musb_resources,
};
#endif /* CONFIG_USB_MUSB_HDRC */

/*
 *  EHCI DEVICE for OMAP3430 ES2.0
 */
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
static struct resource ehci_resources[] = {
	[0] = {
		.start   = L4_BASE+0x64800,
		.end     = L4_BASE+0x64800 + SZ_1K,
		.flags   = IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start   = 77,
		.flags   = IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name  = "ehci-omap",
	.id    = 0,
	.dev = {
		.dma_mask            = &ehci_dmamask,
		.coherent_dma_mask   = 0xffffffff,
		.platform_data       = 0x0,
	},
	.num_resources  = ARRAY_SIZE(ehci_resources),
	.resource       = ehci_resources,
};
#endif /* CONFIG_USB_EHCI_HCD */

/*
 *  OHCI DEVICE for OMAP3430 ES2.0
 */
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct resource ohci_resources[] = {
	[0] = {
		.start   = L4_BASE+0x64400,
		.end     = L4_BASE+0x64400 + SZ_1K,
		.flags   = IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start   = 76,
		.flags   = IORESOURCE_IRQ,
	}
};

/* The dmamask must be set for OHCI to work */
static u64 ohci_dmamask = ~(u32) 0;

static void usb_release(struct device *dev)
{
	/* normally not freed */
}

static struct platform_device ohci_device = {
	.name = "ohci-omap",
	.id = 0,
	.dev = {
		.release = usb_release,
		.dma_mask = &ohci_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.platform_data = 0x0,
	},
	.num_resources = ARRAY_SIZE(ohci_resources),
	.resource = ohci_resources,
};
#endif /* CONFIG_USB_OHCI_HCD */

static void __init joplin3430_usb_init(void)
{
#if	defined(CONFIG_USB_MUSB_HDRC)
	if (platform_device_register(&musb_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
		return;
	}
#endif

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	if (platform_device_register(&ehci_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (EHCI) device\n");
		return;
	}
#endif

#if  defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	if (platform_device_register(&ohci_device) < 0) {
		printk(KERN_ERR "Unable to register FS-USB (OHCI) device\n");
		return;
	}
#endif
}

/*
 *  I2C subsystem
 */
#if  defined(CONFIG_I2C_OMAP)

static u32 omap3_i2c1_clkrate = CONFIG_I2C_OMAP34XX_HS_BUS1;
static u32 omap3_i2c2_clkrate = CONFIG_I2C_OMAP34XX_HS_BUS2;
static u32 omap3_i2c3_clkrate = CONFIG_I2C_OMAP34XX_HS_BUS3;

static struct resource i2c_resources1[] = {
	{
		.start = OMAP3_I2C_BASE1,
		.end   = OMAP3_I2C_BASE1 + 0x57,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = OMAP3_I2C_INT1,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource i2c_resources2[] = {
	{
		.start = OMAP3_I2C_BASE2,
		.end   = OMAP3_I2C_BASE2 + 0x57,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = OMAP3_I2C_INT2,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource i2c_resources3[] = {
	{
		.start = OMAP3_I2C_BASE3,
		.end   = OMAP3_I2C_BASE3 + 0x57,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = OMAP3_I2C_INT3,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device omap_i2c_device1 = {
	.name = "i2c_omap",
	.id = 1,
	.num_resources = ARRAY_SIZE(i2c_resources1),
	.resource = i2c_resources1,
	.dev = {
		.platform_data = &omap3_i2c1_clkrate,
	},
};

static struct platform_device omap_i2c_device2 = {
	.name = "i2c_omap",
	.id = 2,
	.num_resources = ARRAY_SIZE(i2c_resources2),
	.resource = i2c_resources2,
	.dev = {
		.platform_data = &omap3_i2c2_clkrate,
	},
};

static struct platform_device omap_i2c_device3 = {
	.name = "i2c_omap",
	.id = 3,
	.num_resources = ARRAY_SIZE(i2c_resources3),
	.resource = i2c_resources3,
	.dev = {
		.platform_data = &omap3_i2c3_clkrate,
	},
};

#ifdef CONFIG_KEYBOARD_MAXIM7359
/*
 *  Maxim7359 keypad
 */
static int joplin_keymap[] = {
     /*0*/          /*1*/    /*2*/   /*3*/    /*4*/
/*0*/KEY_RIGHTALT,  KEY_L,   KEY_A,  KEY_Q,   KEY_END,
/*1*/KEY_LEFTSHIFT, KEY_Z,   KEY_S,  KEY_W,   KEY_RESERVED,
/*2*/KEY_0,         KEY_X,   KEY_D,  KEY_E,   KEY_RESERVED,
/*3*/KEY_SPACE,     KEY_C,   KEY_F,  KEY_R,   KEY_RESERVED,
/*4*/KEY_ALT,       KEY_V,   KEY_G,  KEY_T,   KEY_O,
/*5*/KEY_DOT,       KEY_B,   KEY_H,  KEY_Y,   KEY_RESERVED,
/*6*/KEY_ENTER,     KEY_N,   KEY_J,  KEY_U,   KEY_RESERVED,
/*6*/KEY_BACKSPACE, KEY_M,   KEY_K,  KEY_I,   KEY_P,
};

static struct maxim7359_platform_data joplin_kbd_data = {
	.dev_name = "maxim_keypad",
	.row_num  = 8,
	.col_num  = 5,
	.keymap   = joplin_keymap,
	.rep_period = 100, // 100 ms repeat time (10 cps)
	.rep_delay  = 500, // 500 ms repeat delay
	.debounce   = 20,  // 20 msec
};

static struct i2c_board_info maxim7359_i2c_board_info = {
	I2C_BOARD_INFO(MAXIM7359_I2C_DEVICE, (0x70 >> 1)),
	.irq = OMAP_GPIO_IRQ(13),
	.platform_data = &joplin_kbd_data,
};

#endif

static int __init joplin3430_init_i2c(void)
{
#ifdef CONFIG_KEYBOARD_MAXIM7359
	i2c_register_board_info(3, &maxim7359_i2c_board_info, 1);
#endif

	(void)platform_device_register(&omap_i2c_device1);
	(void)platform_device_register(&omap_i2c_device2);
	(void)platform_device_register(&omap_i2c_device3);
	return 0;
}
#else

static int __init joplin3430_init_i2c(void) { return 0; }

#endif // I2C

static u32 disp_use_sdi = 0;
static u32 disp_vsync_gated = 1;

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
	.timer_id 	= 9,
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

static struct panel_platform_data board_lcd_data = {
	.parent = &(board_lcd_device.dev),
};

/*
 *  SPI devices
 */
static struct omap2_mcspi_device_config touchpanel_mcspi_config = {
	.turbo_mode = 0,
	.single_channel = 1,
};

static struct tsc2005_platform_data touchpanel_platform_data = {
	.reset_gpio    = 163, // connected to GPIO 163
	.reset_level   = 0,   // active low
	.xresistence   = 100, //
	.penup_timeout = 40,  // msec
};

static struct omap2_mcspi_device_config lcdpanel_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,
};

static struct spi_board_info joplin_spi_board_info[] __initdata = {
	{
		.modalias = "tsc2005",
		.bus_num  = 1,
		.chip_select = 0,
		.max_speed_hz = 6000000,
		.platform_data = &touchpanel_platform_data,
		.controller_data = &touchpanel_mcspi_config,
		.irq = OMAP_GPIO_IRQ(164),
	},
	{
		.modalias = "lcdpanel",
		.bus_num = 3,
		.chip_select = 0,
		.max_speed_hz = 1500000,
		.platform_data   = &board_lcd_data,
		.controller_data = &lcdpanel_mcspi_config,
	}
};

/*
 *  RTC
 */
#ifdef CONFIG_RTC_DRV_TWL4030

static int twl4030_rtc_init(void)
{
	int ret = 0;

	ret = omap_request_gpio(TWL4030_MSECURE_GPIO);
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

static void twl4030_rtc_exit(void)
{
	omap_free_gpio(TWL4030_MSECURE_GPIO);
}

static struct twl4030rtc_platform_data joplin_twl4030rtc_data = {
	.init = &twl4030_rtc_init,
	.exit = &twl4030_rtc_exit,
};

static struct platform_device joplin_twl4030rtc_device = {
	.name = "twl4030_rtc",
	.id = -1,
	.dev = {
		.platform_data = &joplin_twl4030rtc_data,
	},
};

#endif				// CONFIG_RTC_DRV_TWL4030

/*
 *  User pins device
 */
#ifdef CONFIG_USER_PINS

/* NOTE:
 *   Did not change the names in the pin definitions below as I do not know if
 *   any user space programs are already using those entries. - wgr
 */
struct user_pin audio_pins[] = {
	{
		.name       = "AUDIO_PCM_R_TO_BT_CLK",
		.gpio       =  AUDIO_PCM_R_TO_BT_CLK_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  0,  // initialy low
		.pin_mode   =  AUDIO_PCM_R_TO_BT_CLK_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       = "AUDIO_PCM_R_TO_BT_DATA",
		.gpio       =  AUDIO_PCM_R_TO_BT_DATA_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  1,  // initialy high
		.pin_mode   =  AUDIO_PCM_R_TO_BT_DATA_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       = "AUDIO_PCM_REMOTE_MASTER",
		.gpio       =  AUDIO_PCM_REMOTE_MASTER_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  1,  // initialy high
		.pin_mode   =  AUDIO_PCM_REMOTE_MASTER_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
};

/*
 *   Modem pins
 */
static struct user_pin modem_pins[] = {
	{
		.name       = "power_on",
		.gpio       =  MODEM_POWER_ON_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  0,  // initialy low
		.pin_mode   =  MODEM_POWER_ON_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       = "boot_mode",
		.gpio       =  MODEM_BOOT_MODE_GPIO,
		.act_level  =  -1,  // not applicable
		.direction  =  0,   // an output
		.def_level  =  0,   // initially low
		.pin_mode   =  MODEM_BOOT_MODE_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       =  "wakeup_modem",
		.gpio       =  MODEM_WAKE_MODEM_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  0,  // initialy low
		.pin_mode   =  MODEM_WAKE_MODEM_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       =  "wakeup_app",
		.gpio       =  MODEM_WAKE_APP_GPIO,
		.act_level  =  1,  // active high
		.direction  =  1,  // an output
		.def_level  = -1,  // unset
		.pin_mode   =  MODEM_WAKE_APP_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
	{
		.name       =  "wakeup_usb",
		.gpio       =  MODEM_WAKE_APP_USB_GPIO,
		.act_level  =  1,  // active high
		.direction  =  1,  // an output
		.def_level  = -1,  // unset
		.pin_mode   =  MODEM_WAKE_APP_USB_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
	},
};


/*
 *   Bt Pins
 */
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

static struct user_pin_set  board_user_pins_sets[]  = {
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
	{
		.set_name = "audio",
		.num_pins = ARRAY_SIZE(audio_pins),
		.pins     = audio_pins,
	},
};

static struct user_pins_platform_data board_user_pins_pdata  = {
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

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
/*
 * MODEM handshaking activity monitor.
 */
static struct modem_activity_monitor_config modem_activity_cfg = {
	.app_wake_modem_gpio = MODEM_WAKE_MODEM_GPIO,   // modem wakeup pin
	.modem_wake_app_gpio = MODEM_WAKE_APP_GPIO,     // 
	.modem_wake_usb_gpio = MODEM_WAKE_APP_USB_GPIO, // 
#ifdef CONFIG_USER_PINS
	// if we are using user pins interface mark pins as configured
	.gpio_flags          = GPIO_FLG_AWM_CONFIGURED |
	                       GPIO_FLG_MWA_CONFIGURED |
	                       GPIO_FLG_MWU_CONFIGURED,
#endif
	.uart    = 0,       // uart to monitor
	.timeout = 120,     // timeout in msec
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
#ifdef CONFIG_RTC_DRV_TWL4030
	&joplin_twl4030rtc_device,
#endif
#if  defined(CONFIG_OMAP_MISC_HSUART) || defined(CONFIG_OMAP_MISC_HSUART_MODULE)
	&btuart_device,
	&mduart_device,
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
static struct omap_board_config_kernel joplin_config[] __initdata = {
	{OMAP_TAG_UART,	&joplin_uart_config},
#ifdef CONFIG_MMC
	{OMAP_TAG_MMC,	&board_mmc_config},
#endif
};

/*
 *  Map common IO
 */
static void __init joplin3430_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

/*
 *  Fixup Joplin machine
 */
static void __init
joplin3430_fixup(struct machine_desc *desc, struct tag *t,
		 char **cmdline, struct meminfo *mi)
{
	printk("%s: %p\n", __FUNCTION__, t);

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
		t->u.mem.size = 32 * 1024 * 1024;
		t = tag_next(t);

#ifdef    CONFIG_BLK_DEV_RAM
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

/*
 * Do board specific muxing
 */
static void __init joplin3430_mux(void)
{
#ifdef CONFIG_MMC
	omap_cfg_reg("AE4_3430_GPIO136_SD_CD");	// SD0 CD IRQ
#endif

	// touch panel (tsc2005)
	omap_cfg_reg("H18_3430_GPIO163_TP_RESET");	// TP RESET (GPIO 163)
	omap_cfg_reg("H19_3430_GPIO164_TP_IRQ");	// TP IRQ   (GPIO 164)

	omap_cfg_reg("AB3_3430_SPI1_CLK");	// SPI1
	omap_cfg_reg("AB4_3430_SPI1_O");  	// SPI1
	omap_cfg_reg("AA4_3430_SPI1_I");  	// SPI1
	omap_cfg_reg("AC2_3430_SPI1_CS0");	// SPI1

	// LCD
	omap_cfg_reg("AG22_3430_DSS_DATA0");
	omap_cfg_reg("AH22_3430_DSS_DATA1");
	omap_cfg_reg("AG23_3430_DSS_DATA2");
	omap_cfg_reg("AH23_3430_DSS_DATA3");
	omap_cfg_reg("AG24_3430_DSS_DATA4");
	omap_cfg_reg("AH24_3430_DSS_DATA5");
	omap_cfg_reg("E26_3430_DSS_DATA6");
	omap_cfg_reg("F28_3430_DSS_DATA7");
	omap_cfg_reg("F27_3430_DSS_DATA8");
	omap_cfg_reg("G26_3430_DSS_DATA9");
	omap_cfg_reg("AD28_3430_DSS_DATA10");
	omap_cfg_reg("AD27_3430_DSS_DATA11");
	omap_cfg_reg("AB28_3430_DSS_DATA12");
	omap_cfg_reg("AB27_3430_DSS_DATA13");
	omap_cfg_reg("AA28_3430_DSS_DATA14");
	omap_cfg_reg("AA27_3430_DSS_DATA15");

	omap_cfg_reg("D28_3430_LCD_PCLK");
	omap_cfg_reg("D26_3430_LCD_PDA");
	omap_cfg_reg("D27_3430_LCD_RESET");
	omap_cfg_reg("E28_3430_LCD_DE");

	/* LCD EN */
	omap_cfg_reg("AC27_3430_LCD_EN");
	omap_cfg_reg("AC28_3430_LCD_PCI");
	omap_cfg_reg("T8_3430_LCD_LED_PWM");
	omap_cfg_reg("Y4_3430_LCD_KEY_PWM");

	/* LCD SPI3 */
	omap_cfg_reg("H26_3430_LCD_SPI3_CLK");
	omap_cfg_reg("H25_3430_LCD_SPI3_O");
	omap_cfg_reg("E28_3430_LCD_SPI3_I");
	omap_cfg_reg("J26_3430_LCD_SPI3_CS0");

	/* WiFi Pins */
	omap_cfg_reg("AE2_WL_SD1_CLK");
	omap_cfg_reg("AG5_WL_SD1_CMD");
	omap_cfg_reg("AH5_WL_SD1_D0");
	omap_cfg_reg("AH4_WL_SD1_D1");
	omap_cfg_reg("AG4_WL_SD1_D2");
	omap_cfg_reg("AF4_WL_SD1_D3");
	omap_cfg_reg("AF3_WL_IRQ");
	omap_cfg_reg("AE3_WL_RST_nPWD");

	/* BT Pins */
	omap_cfg_reg ("AB1_BT_HOST_WAKE");
	omap_cfg_reg ("AB2_BT_nWAKE");
	omap_cfg_reg ("AA3_BT_nRST");

	/* Modem Control */
	omap_cfg_reg ("AE1_3430_GPIO152");
	omap_cfg_reg ("AD1_3430_GPIO153");
	omap_cfg_reg ("AD2_3430_GPIO154");
	omap_cfg_reg ("AC1_3430_GPIO155");
	omap_cfg_reg ("AA21_3430_GPIO157");

	/* proximity sensor */
#ifdef CONFIG_HSDL9100_PROX_SENSOR
	/* This config option should not be set for joplin 3430 unless we run
	 * on specially modified board */
	omap_set_gpio_debounce(JOPLIN_HSDL9100_INT_GPIO, 1);
	omap_set_gpio_debounce_time(JOPLIN_HSDL9100_INT_GPIO, 0x02);

	omap_cfg_reg ("T8_3430_GPIO55");
	omap_cfg_reg ("Y21_3430_GPIO156");
	omap_cfg_reg ("V3_3430_GPT8_PWM_EVT");
#endif

	/* MMC1 */
	omap_cfg_reg ("N28_MMC1_CLK");
	omap_cfg_reg ("M27_MMC1_CMD");
	omap_cfg_reg ("N27_MMC1_DAT0");
	omap_cfg_reg ("N26_MMC1_DAT1");
	omap_cfg_reg ("N25_MMC1_DAT2");
	omap_cfg_reg ("P28_MMC1_DAT3");
	omap_cfg_reg ("P27_MMC1_DAT4");
	omap_cfg_reg ("P26_MMC1_DAT5");
	omap_cfg_reg ("R27_MMC1_DAT6");
	omap_cfg_reg ("R25_MMC1_DAT7");
}

/*
 *  Joplin init irq
 */
static void __init joplin3430_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	joplin3430_mux();
}

/*
 *  Init Joplin mach
 */
static void __init joplin3430_init(void)
{
	/* set omap config */
	omap_board_config	= joplin_config;
	omap_board_config_size	= ARRAY_SIZE(joplin_config);

	/* Serial */
	omap_serial_init();

	/* add platform devices */
	platform_add_devices(joplin_devices, ARRAY_SIZE(joplin_devices));

	/* USB */
	joplin3430_usb_init();

	/* SPI */
	spi_register_board_info(joplin_spi_board_info,
				ARRAY_SIZE(joplin_spi_board_info));

}

arch_initcall(joplin3430_init_i2c);

MACHINE_START(FLANK, "FLANK (aka Joplin OMAP3430) board")
	.phys_io	= JOPLIN_3430_PHYS_IO_BASE,
	.io_pg_offst	= (((JOPLIN_3430_PG_IO_VIRT) >> 18) & 0xfffc),
	.boot_params	= JOPLIN_3430_RAM_BASE + 0x100,
	.fixup		= joplin3430_fixup,
	.map_io		= joplin3430_map_io,
	.init_irq	= joplin3430_init_irq,
	.init_machine	= joplin3430_init,
	.timer		= &omap_timer,
MACHINE_END
