/*
 * arch/arm/mach-omap3pe/board-sirloin-3430.c
 *
 * Copyright (C) 2007-2009 Palm, Inc.
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
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/major.h>
#include <linux/usb/musb.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/fb.h>

#ifdef CONFIG_TOUCHSCREEN_CY8MRLN
#include <linux/spi/cy8mrln.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_TM1129
#include <linux/spi/tm1129.h>
#endif

#ifdef CONFIG_KEYBOARD_MAXIM7359
#include <linux/i2c_maxim7359_keypad.h>
#endif

#ifdef CONFIG_LIGHTSENSOR_AMBIENT6200
#include <linux/temt6200_lightsensor.h>
#endif

#ifdef CONFIG_KEYBOARD_GPIO_PE
#include <linux/gpio_keys_pe.h>
#endif

#ifdef CONFIG_LEDS_TPS6025X
#include <linux/i2c_tps6025x_led.h>
#endif

#ifdef CONFIG_LEDS_TPS6105X
#include <linux/i2c_tps6105x_led.h>
#endif

#ifdef CONFIG_LEDS_MAXIM8831
#include <linux/i2c_maxim8831_led.h>
#endif

#ifdef CONFIG_LEDS_LP8501
#include <linux/i2c_lp8501_led.h>
#endif

#ifdef CONFIG_ACCELEROMETER_KXSD9
#include <linux/i2c_kxsd9_accelerometer.h>
#endif

#ifdef CONFIG_ACCELEROMETER_KXTE9 
#include <linux/i2c_kxte9_accelerometer.h>
#endif

#ifdef CONFIG_HSDL9100_PROX_SENSOR
#include <linux/hsdl9100_proximity_sensor.h>
#endif

#ifdef CONFIG_OMAP_MISC_HSUART
#include <linux/hsuart.h>
#endif

#ifdef CONFIG_USER_PINS
#include <linux/user-pins.h>
#endif

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
#include <linux/modem_activity.h>
#endif

#include <linux/vibrator.h>

#include <media/vx6852.h>

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
#include <asm/arch/hdq.h>
#include <asm/arch/irqs.h>
#include <asm/arch/omap34xx.h>
#include <asm/arch/isp.h>
#include <asm/arch/display.h>
#include <asm/arch/lcd.h>
#include <asm/arch/nduid.h>


#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
#include <asm/arch/dmtimer.h>
#endif


#include "boot-wall.h"
#include "headset-detect.h"

/* Board type */
u32 board_type = 1;

#ifdef CONFIG_MMC
extern struct omap_mmc_config board_mmc_config;
#endif

/* Board type selection */
static int  __init board_args(char *str)
{
	if (!strcmp(str, "castle-evt0b")) {
		board_type = EVT0b;
	} else if (!strcmp(str, "castle-evt1")) {
		board_type = EVT1;
	} else if (!strcmp(str, "castle-evt2")) {
		board_type = EVT2;
	} else if (!strcmp(str, "castle-evt3")) {
		board_type = EVT3;
	} else if (!strcmp(str, "castle-dvt1")) {
		board_type = DVT;
	} else if (!strcmp(str, "castle-dvt2")) {
		board_type = DVT;
	} else if (!strcmp(str, "castle-dvt3")) {
		board_type = DVT3;
	} else {
		board_type = DVT3;
	}
        return 0;
}
__setup("boardtype=", board_args);

/*
 *  UART support
 */
static struct omap_uart_config board_uart_config __initdata  = {
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

#if defined(CONFIG_OMAP_MISC_HSUART) || defined(CONFIG_OMAP_MISC_HSUART_MODULE)

static u64 mduart_dmamask = ~(u32)0;
static struct hsuart_platform_data mduart_data = {
	.dev_name   = "modemuart",
	.uart_mode  = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = HSUART_SPEED_115K,
	.options    = HSUART_OPTION_MODEM_DEVICE,
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
	.idle_timeout      =   0, // no idle timeout,managed by handshaking
	.idle_poll_timeout =  10, // 
	.dbg_level         =   0, // dbg level 0 for now
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
	.options    = 0,
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
	.idle_timeout      =  500, // initial idle timeout
	.idle_poll_timeout =  500, // same as regular idle timeout
	.dbg_level         =    0, // dbg level 0 for now
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
		.start	= HS_BASE,
		.end	= HS_BASE + SZ_8K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = { /* general IRQ */
		.start	= INT_243X_HS_USB_MC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = { /* DMA IRQ */
		.start	= INT_243X_HS_USB_DMA,
		.flags	= IORESOURCE_IRQ,
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
		.dma_mask = &musb_dmamask,
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
	[1] = {  /* general IRQ */
		.start   = 77,
		.flags   = IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name  = "ehci-omap",
	.id    = 0,
	.dev = {
		.dma_mask           = &ehci_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = 0x0,
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
static u64 ohci_dmamask = ~(u32)0;

static void usb_release(struct device *dev)
{
        /* normally not freed */
}

static struct platform_device ohci_device = {
	.name = "ohci-omap",
	.id   = 0,
	.dev  = {
		.release  = usb_release,
		.dma_mask = &ohci_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = 0x0,
	},
	.num_resources  = ARRAY_SIZE(ohci_resources),
	.resource       = ohci_resources,
};
#endif /* CONFIG_USB_OHCI_HCD */

static void __init board_usb_init(void)
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
		.start  = OMAP3_I2C_BASE1,
		.end    = OMAP3_I2C_BASE1 + 0x57,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = OMAP3_I2C_INT1_GPIO,
		.flags  = IORESOURCE_IRQ,
	},
};
static struct resource i2c_resources2[] = {
	{
		.start  = OMAP3_I2C_BASE2,
		.end    = OMAP3_I2C_BASE2 + 0x57,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = OMAP3_I2C_INT2_GPIO,
		.flags  = IORESOURCE_IRQ,
	},
};
static struct resource i2c_resources3[] = {
	{
		.start  = OMAP3_I2C_BASE3,
		.end    = OMAP3_I2C_BASE3 + 0x57,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = OMAP3_I2C_INT3_GPIO,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device omap_i2c_device1 = {
	.name  = "i2c_omap",
	.id    =  1,
	.num_resources = ARRAY_SIZE(i2c_resources1),
	.resource = i2c_resources1,
	.dev   = {
		.platform_data = &omap3_i2c1_clkrate,
	},
};

static struct platform_device omap_i2c_device2 = {
	.name = "i2c_omap",
	.id   =  2,
	.num_resources = ARRAY_SIZE(i2c_resources2),
	.resource  = i2c_resources2,
	.dev  = {
		.platform_data = &omap3_i2c2_clkrate,
	},
};

static struct platform_device omap_i2c_device3 = {
	.name = "i2c_omap",
	.id   =  3,
	.num_resources = ARRAY_SIZE(i2c_resources3),
	.resource  = i2c_resources3,
	.dev  = {
		.platform_data = &omap3_i2c3_clkrate,
	},
};

#ifdef CONFIG_KEYBOARD_MAXIM7359
/*
 *  Maxim7359 keypad
 */
static int board_keymap[] = {
	KEY_Q,         KEY_E,        KEY_T,        KEY_U,        KEY_O,
	KEY_W,         KEY_R,        KEY_Y,        KEY_I,        KEY_P,
	KEY_S,         KEY_F,        KEY_H,        KEY_K,        KEY_BACKSPACE,
	KEY_A,         KEY_D,        KEY_G,        KEY_J,        KEY_L,
	KEY_Z,         KEY_C,        KEY_B,        KEY_M,        KEY_ENTER,
	KEY_RIGHTALT,  KEY_X,        KEY_V,        KEY_N,        KEY_COMMA,
	KEY_LEFTSHIFT, KEY_0,        KEY_SPACE,    KEY_ALT,      KEY_DOT,
	KEY_RESERVED,  KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED,
};

/*
 *   Key proximity map
 */
static u8  board_key_prox_map[][6] = {
	{  5, 15,             0xFF }, // q [ 0]
	{  5,  6, 16,         0xFF }, // e [ 1]
	{  6,  7, 17,         0xFF }, // t [ 2]
	{  7,  8, 18,         0xFF }, // u [ 3]
	{  8,  9, 19,         0xFF }, // o [ 4]
	{  0,  1, 10,         0xFF }, // w [ 5]
	{  1,  2, 11,         0xFF }, // r [ 6]
	{  2,  3, 12,         0xFF }, // y [ 7]
	{  3,  4, 13,         0xFF }, // i [ 8]
	{  4, 14,             0xFF }, // p [ 9]
	{  5, 15, 16, 20,     0xFF }, // s  [10]
	{  6, 16, 17, 21,     0xFF }, // f  [11]
	{  7, 17, 18, 22,     0xFF }, // h  [12]
	{  8, 18, 19, 23,     0xFF }, // k  [13]
	{  9, 19, 24,         0xFF }, // BS [14]
	{  0, 10, 25,         0xFF }, // a  [15]
	{  1, 10, 11, 26,     0xFF }, // d  [16]
	{  2, 11, 12, 27,     0xFF }, // g  [17]
	{  3, 12, 13, 28,     0xFF }, // j  [18]
	{  4, 13, 14, 29,     0xFF }, // l  [19]
	{ 10, 25, 26, 30,     0xFF }, // z  [20]
	{ 11, 26, 27, 31, 32, 0xFF }, // c  [21]
	{ 12, 27, 28, 32,     0xFF }, // b  [22]
	{ 13, 28, 29, 33, 34, 0xFF }, // m  [23]
	{ 14, 28, 29,         0xFF }, // En [24]
	{ 15, 20,             0xFF }, // rA [25]
	{ 16, 20, 21, 30, 31, 0xFF }, // x  [26]
	{ 17, 21, 22, 32,     0xFF }, // v  [27]
	{ 18, 22, 23, 32, 34, 0xFF }, // n  [28]
	{ 19, 23, 24, 33,     0xFF }, // ,  [29]
	{ 20, 26, 31,         0xFF }, // sh [30]
	{ 30, 26, 21, 32,     0xFF }, // 0  [31]
	{ 31, 27, 22, 29,     0xFF }, // sp [32]
	{ 32, 28, 23, 34,     0xFF }, // .  [33]
	{ 33, 23, 29,         0xFF }, // opt[34]
};

static struct maxim7359_platform_data board_kbd_data = {
	.dev_name     = "maxim_keypad",
	.row_num      = 8,
	.col_num      = 5,
	.keymap       = board_keymap,
	.key_prox_timeout = 50,
	.key_prox_width = 6,
	.key_prox_map   = board_key_prox_map,
	.rep_period  = 100,       // 100 ms repeat time (10 cps)
	.rep_delay   = 500,       // 500 ms repeat delay
	.hw_debounce    = 9,      // 9 ms
	.sw_debounce    = 40,     // 40 ms
	.wakeup_en   = 1,         // by default enable wakeup.
};

static struct i2c_board_info maxim7359_i2c_board_info = {
	I2C_BOARD_INFO( MAXIM7359_I2C_DEVICE, (0x70>>1)),
	.irq = OMAP_GPIO_IRQ(13),
	.platform_data = &board_kbd_data,
};


#endif

#ifdef CONFIG_KEYBOARD_GPIO_PE
static struct gpio_keys_button board_gpio_keys_buttons[] = {
	{
		.code        = KEY_VOLUMEUP,
		.gpio        = VOL_UP_GPIO,
		.active_low  = 1,
		.desc        = "volume up",
		.debounce    = 20,
		.type	     = EV_KEY,
		.wakeup      = 1,
		.pin         = "AE7_3430_GPIO24_KEY_VOL_UP",
#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
		.options     = OPT_CONSOLE_TRIGGER,
#endif
	},
	{
		.code        = KEY_VOLUMEDOWN,
		.gpio        = VOL_DN_GPIO,
		.active_low  = 1,
		.desc        = "volume down",
		.debounce    = 20,
		.type	     = EV_KEY,
		.wakeup      = 1,
		.pin         = "AF7_3430_GPIO25_KEY_VOL_DN",
	},
	{
		.code        = KEY_PTT,
		.gpio        = PTT_GPIO,
		.active_low  = 1,
		.desc        = "push to talk",
		.debounce    = 20,
		.type        = EV_KEY,
		.wakeup      = 0,
	},
	{
		.code        = SW_SLIDER,
		.gpio        = SLIDER_GPIO,
		.active_low  = 1,
		.desc        = "slider",
		.debounce    = 20,
		.type        = EV_SW,
		.wakeup      = 1,
		.pin         = "AH7_3430_GPIO27_SLIDER_OPEN",
	},
	{
		.code        = SW_RINGER,
		.gpio        = RING_GPIO,
		.active_low  = 1,
		.desc        = "ring silence",
		.debounce    = 20,
		.type        = EV_SW,
		.wakeup      = 1,
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
		.options     = OPT_REBOOT_TRIGGER | OPT_REBOOT_TRIGGER_EDGE,
#endif		
	},
	{
		.code        = KEY_END,
		.gpio        = POWER_GPIO,
		.active_low  = 1,
		.desc        = "power",
		.debounce    = 20,
		.type        = EV_KEY,
		.wakeup      = 1,
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
		.options     = OPT_REBOOT_TRIGGER | OPT_REBOOT_TRIGGER_LEVEL,
#endif		              
	},
	{
		.code        = KEY_CENTER,
		.gpio        = CORE_NAVI_GPIO,
		.active_low  = 1,
		.desc        = "core navi",
		.debounce    = 20,
		.type        = EV_KEY,
		.wakeup      = 0,
		.pin         = "AE13_3430_GPIO17_CORE_NAVI",
#ifdef  CONFIG_GPIO_KEYS_CONSOLE_TRIGGER
		.options     = OPT_CONSOLE_TRIGGER,
#endif
	},
	{
		.code        = SW_OPTICAL_SLIDER,
		.gpio        = 137,
		.active_low  = 1,
		.desc        = "optical slider",
		.debounce    = 20,
		.type        = EV_SW,
		.wakeup      = 0,
	},

};

static struct gpio_keys_platform_data board_gpio_keys = {
	.buttons  = board_gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(board_gpio_keys_buttons),
};

static struct platform_device board_gpio_keys_device = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data  = &board_gpio_keys,
	},
};
#endif

#ifdef CONFIG_KEYBOARD_GPIO_PE
static void __init board_gpio_keys_init(void)
{
	(void) platform_device_register(&board_gpio_keys_device);
}
#endif

#ifdef CONFIG_LIGHTSENSOR_AMBIENT6200
static struct temt6200_platform_data board_temt6200_platform_data = {
	.desc    = TEMT6200_DRIVER,
	.channel = 2,
	.average = 0,
	.enable_lgt = board_lgt_vcc_enable,
};

static struct platform_device board_temt6200_device = {
	.name = TEMT6200_DEVICE,
	.id   = -1,
	.dev  = {
		.platform_data = &board_temt6200_platform_data,
	},
};
#endif

#ifdef CONFIG_LEDS_TPS6025X
/* Keypad Backlight LED controller. */
static struct tps6025x_platform_data tps60252_data = {
	.leds[DM1] = {
		.cdev = {
			.name = "kbd_bl_led_dm1",
		},
		.enable = 0,
		.id = DM1,
	},
	.leds[DM2] = {
		.cdev = {
			.name = "kbd_bl_led_dm2",
		},
		.enable = 0,
		.id = DM2,
	},
	.leds[DM3] = {
		.cdev = {
			.name = "kbd_bl_led_right",
		},
		.enable = 1,
		.id = DM3,
	},
	.leds[DM4] = {
		.cdev = {
			.name = "kbd_bl_led_dm4",
		},
		.enable = 0,
		.id = DM4,
	},
	.leds[DM5] = {
		.cdev = {
			.name = "kbd_bl_led_dm5",
		},
		.enable = 0,
		.id = DM5,
	},
	.leds[DS1] = {
		.cdev = {
			.name = "kbd_bl_led_center",
		},
		.enable = 1,
		.id = DS1,
	},
	.leds[DS2] = {
		.cdev = {
			.name = "kbd_bl_led_left",
		},
		.enable = 1,
		.id = DS2,
	},
	.dm5_aux_display_mode = 0,
};
static struct i2c_board_info tps60252_i2c_board_info = {
	I2C_BOARD_INFO(TPS6025X_I2C_DEVICE, TPS60252_I2C_ADDR),
	.platform_data = &tps60252_data,
};
#endif

#ifdef CONFIG_LEDS_MAXIM8831
static struct i2c_client *bl_i2c_client = NULL;

static int board_maxim8831_backlight_probe(void* led)
{
	bl_i2c_client = (struct i2c_client*)led;
	return 0;
}
static int board_maxim8831_backlight_remove(void* led)
{
	bl_i2c_client = NULL;
	return 0;
}
static int board_maxim8831_backlight_suspend(void* led)
{
	return 0;
}
static int board_maxim8831_backlight_resume(void* led)
{
	return 0;
}
static struct maxim8831_platform_data maxim8831_data = {
	.leds[LED1] = {
		.cdev = {
			.name = "display_bl_led",
		},
		.used = 1,
		.default_state = 1,
		.id = LED1,
	},
	.leds[LED2] = {
		.cdev = {
			.name = "not_used0",
		},
		.used = 0, 
		.default_state = 0,
		.id = LED2,
	},
	.leds[LED3] = {
		.cdev = {
			.name = "corenavi_led_center",
		},
		.used = 1,
		.default_state = 0,
		.id = LED3,
	},
	.leds[LED4] = {
		.cdev = {
			.name = "corenavi_led_left",
		},
		.used = 1,
		.default_state = 0,
		.id = LED4,
	},
	.leds[LED5] = {
		.cdev = {
			.name = "corenavi_led_right",
		},
		.used = 1,
		.default_state = 0,
		.id = LED5,
	},
	.board_probe = board_maxim8831_backlight_probe,
	.board_remove = board_maxim8831_backlight_remove,
	.board_suspend = board_maxim8831_backlight_suspend,
	.board_resume = board_maxim8831_backlight_resume,
};
static struct i2c_board_info maxim8831_i2c_board_info = {
	I2C_BOARD_INFO(MAXIM8831_I2C_DEVICE, MAXIM8831_I2C_ADDR),
	.platform_data = &maxim8831_data,
};

void board_maxim8831_backlight_set_brightness(int brightness)
{
	if (bl_i2c_client)
		maxim8831_mod_brightness(bl_i2c_client, LED1, brightness);
}
EXPORT_SYMBOL(board_maxim8831_backlight_set_brightness);
#endif



#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT

// for secondary backlight control
#define TPS6160_ILED_BOOT_VALUE 16
#define TPS6160_ILED_MAX_VALUE  32
#define ILED_PULSE_DOWN         200
#define ILED_PULSE_UP           20

#define BACKLIGHT_OFF 0
#define BACKLIGHT_ON  1

static DEFINE_SPINLOCK(sbc_pulse_lock);

struct board_sbc_cfg {
	int timer_id ;
	char *pinmux_off;
	char *pinmux_on;
} acx567akm_tps6160_backlight_config = {
	.timer_id   = 9,
	.pinmux_off = "Y2_LCD_LED_PWM_OFFMODE",
	.pinmux_on  = "Y2_LCD_LED_PWM",
	};

struct board_sbc_data {
	int   timer_id;
	char *pinmux_off;
	char *pinmux_on;
	struct omap_dm_timer *pulse_timer;
	int   init;
	int   last_state;
	int current_brightness;
	int panel_brightness;
};


static void board_sbc_pulse ( struct board_sbc_data *sbc_data,
                              int polarity, int width, int yield)
{
	unsigned long flags;
	uint32_t mcount,count;

	if (width < 10) { 
		width = 10;
	}
	mcount = (ulong) 0xfffffffe - ((long) width- 10)/20;

	omap_dm_timer_enable(sbc_data->pulse_timer);

	spin_lock_irqsave(&sbc_pulse_lock, flags);
	omap_dm_timer_set_pwm_start(sbc_data->pulse_timer, !polarity,1, 1, 0);
	omap_dm_timer_set_load(sbc_data->pulse_timer, 1, 0);
	omap_dm_timer_set_counter(sbc_data->pulse_timer, mcount);

	omap_dm_timer_set_pwm_start(sbc_data->pulse_timer, polarity, 1, 1, 1);
	spin_unlock_irqrestore(&sbc_pulse_lock, flags);

	do {
		count = omap_dm_timer_read_counter(sbc_data->pulse_timer);
		if (yield) {
			msleep(1);
		}
	} while ( count >= mcount );

	if (!yield) {
		do {
			count = omap_dm_timer_read_counter(sbc_data->pulse_timer);
		} while ( count < 1 );
	}

	omap_dm_timer_set_pwm_start(sbc_data->pulse_timer, !polarity, 1, 1, 0);

	omap_dm_timer_disable(sbc_data->pulse_timer);
}



static void *board_sbc_init(void *config_data)
{
	struct board_sbc_data *sbc_data;
	struct board_sbc_cfg  *config = (struct board_sbc_cfg  *) config_data;
	int len;

	sbc_data = kzalloc(sizeof(struct board_sbc_data), GFP_KERNEL);
	if (unlikely(!sbc_data)) {
		printk(KERN_ERR "Error, no memeory for %s\n",__FUNCTION__);
		return NULL;
	}
	spin_lock_init(&sbc_pulse_lock);

	sbc_data->timer_id = config->timer_id; 
	sbc_data->init     = 0;
	sbc_data->last_state = 0;
	sbc_data->current_brightness = 0;
	sbc_data->panel_brightness   = 0;

	sbc_data->pulse_timer = omap_dm_timer_request_specific(sbc_data->timer_id);
	if (sbc_data->pulse_timer == NULL) {
		kfree(sbc_data);
		return NULL;
	}
	omap_dm_timer_enable(sbc_data->pulse_timer);
	omap_dm_timer_set_source(sbc_data->pulse_timer, OMAP_TIMER_SRC_SYS_CLK);
	omap_dm_timer_set_prescaler(sbc_data->pulse_timer, 7);
	
	omap_dm_timer_disable(sbc_data->pulse_timer);

	if (config->pinmux_off != NULL) {
		len = strlen(config->pinmux_off) + 1;
		sbc_data->pinmux_off = kzalloc(len, GFP_KERNEL);
		if (unlikely(!sbc_data->pinmux_off)) {
			printk(KERN_ERR "Error, no memeory for %s\n",__FUNCTION__);
			sbc_data->pinmux_off = NULL;
		}
		else {
			strcpy(sbc_data->pinmux_off,config->pinmux_off);
		}

	}

	if (config->pinmux_on != NULL) {
		len=strlen(config->pinmux_on) + 1;
		sbc_data->pinmux_on = kzalloc(len, GFP_KERNEL);
		if (unlikely(!sbc_data->pinmux_on)) {
			printk(KERN_ERR "Error, no memeory for %s\n",__FUNCTION__);
			sbc_data->pinmux_on = NULL;
		}
		else {
			strcpy(sbc_data->pinmux_on,config->pinmux_on);
		}

	}
	return sbc_data;
}


static void board_sbc_remove(void *data)
{
	struct board_sbc_data *sbc_data = (struct board_sbc_data *)data;
	
	if (sbc_data != NULL) {
		if ( sbc_data->pinmux_on != NULL) {
			kfree (sbc_data->pinmux_on );
		}
		if ( sbc_data->pinmux_off != NULL) {
			kfree (sbc_data->pinmux_off );
		}
		kfree(sbc_data);
	}
	return;
}


static void board_sbc_brightness(void *data, int brightness, int yield)
{
	struct board_sbc_data *sbc_data = (struct board_sbc_data *)data;
	uint32_t br;

	if (data==NULL) {
		return;
	}
	
	if (brightness == 1000) {
		board_sbc_pulse(sbc_data, 0, ILED_PULSE_DOWN, yield);
		return;
	}

	if (brightness == 2000) {
		board_sbc_pulse(sbc_data, 0, ILED_PULSE_UP, yield);
		return;
	}
	br = (brightness < 2) ? 2: ((brightness*30+99)/100)+2;

	if (br >= TPS6160_ILED_MAX_VALUE) {
		br = 40;
	}
	if (br == 1) {
		sbc_data->current_brightness = 33;
	}
	if (sbc_data->current_brightness < br) {
		for (;sbc_data->current_brightness < br; 
		      sbc_data->current_brightness++) {
			board_sbc_pulse(sbc_data, 0, ILED_PULSE_UP, yield);
		}
	} 
	if (sbc_data->current_brightness > br) {
		for (;sbc_data->current_brightness > br;
		      sbc_data->current_brightness--) {
			board_sbc_pulse(sbc_data, 0, ILED_PULSE_DOWN, yield);
		}
	}
	if (sbc_data->current_brightness >= TPS6160_ILED_MAX_VALUE) {
		sbc_data->current_brightness = TPS6160_ILED_MAX_VALUE;
	}
	sbc_data->panel_brightness = brightness;
}

static void board_sbc_preon(void *data)
{
	struct board_sbc_data *sbc_data;

	sbc_data = (struct board_sbc_data *)data;
	if (data == NULL) {
		return;
	}
	if (sbc_data->pinmux_on != NULL) {
		omap_cfg_reg ( sbc_data->pinmux_on );
		msleep(1);
	}
	omap_dm_timer_enable(sbc_data->pulse_timer);
	omap_dm_timer_set_prescaler(sbc_data->pulse_timer, 7);
	omap_dm_timer_disable(sbc_data->pulse_timer);
}

static void board_sbc_on(void *data)
{
	struct board_sbc_data *sbc_data;
	int old_brightness;

	sbc_data = (struct board_sbc_data *)data;
	if (data == NULL) {
		return;
	}

	old_brightness = sbc_data->panel_brightness;

	// TBD sync to display backlight enable in a better way...
	msleep(15);

	sbc_data->last_state = BACKLIGHT_ON;
	if (sbc_data->init == 0) {
		sbc_data->current_brightness = TPS6160_ILED_MAX_VALUE;
		sbc_data->init = 1;
	}
	else {
		sbc_data->current_brightness = TPS6160_ILED_BOOT_VALUE;
	}

	board_sbc_brightness(data, old_brightness, 0);
}

static void board_sbc_off(void *data)
{
	struct board_sbc_data *sbc_data;

	sbc_data = (struct board_sbc_data *)data;
	if (data == NULL) {
		return;
	}
	sbc_data->last_state = BACKLIGHT_OFF;
	if (sbc_data->pinmux_off != NULL) {
		omap_cfg_reg ( sbc_data->pinmux_off );
	}
}

#endif /* CONFIG_SPI_ACX567AKM_BACKLIGHT */





#if defined(CONFIG_ACCELEROMETER_KXSD9) \
	|| defined(CONFIG_ACCELEROMETER_KXSD9_MODULE)

/*
 * The translation of the X, Y, Z axis sensitivity units are done
 * using the following matrix multiplication.
 * flat on table x=0 y=0 z=-10000
 * face up table x=0 y=-10000 z=0
 * portrait (core-navi button on right) x=-10000 y=0 z=0
 *		  -	     -   - -  
 *		 |  a0	a1 a2 | | x | 
 *		 |  b0	b1 b2 | | y |	
 *		 |  c0	c1 c2 | | z |	
 *		  -	     -	 - -	
 */

static int kxsd9_xyz_translation_map [] = {
/*      0			1			2	*/
/*0*/   0,			1,			0,
/*1*/   1,			0,			0,
/*2*/   0,			0,			1,
};

static struct kxsd9_platform_data board_kxsd9_platform_data= {
	.dev_name = ACCELEROMETER_DEVICE,
	.xyz_translation_map = kxsd9_xyz_translation_map,
};

static struct i2c_board_info kxsd9_i2c_board_info = {
	I2C_BOARD_INFO( ACCELEROMETER_DEVICE, ACCELEROMETER_I2C_ADDRESS),
	.irq = OMAP_GPIO_IRQ(160),
	.platform_data = &board_kxsd9_platform_data,
};
#endif

#ifdef CONFIG_ACCELEROMETER_KXTE9

/*
 * The translation of the X, Y, Z axis sensitivity units are done
 * using the following matrix multiplication.
 * flat on table x=0 y=0 z=-10000
 * face up table x=0 y=-10000 z=0
 * portrait (core-navi button on right) x=-10000 y=0 z=0
 *		  -	     -   - -  
 *		 |  a0	a1 a2 | | x | 
 *		 |  b0	b1 b2 | | y |	
 *		 |  c0	c1 c2 | | z |	
 *		  -	     -	 - -	
 */
static int kxte9_xyz_translation_map [] = {
/*      0			1			2	*/
/*0*/   0,			1,			0,
/*1*/   1,			0,			0,
/*2*/   0,			0,			1,
};

static struct kxte9_platform_data board_kxte9_platform_data = {
	.dev_name = "kxte9_accelerometer",
	.xyz_translation_map = kxte9_xyz_translation_map,
	.gpio       = OMAP_GPIO_IRQ(160),
	.wuf_thresh = 1250, // 125mG
	.b2s_thresh = 20000, // 2G
	.wuf_timer  = 1,
	.b2s_timer  = 1,
	.tilt_timer = 3,
	.tilt_thresh = 25,
	.odr_main = ODR_40HZ,
	.odr_b2s = ODR_40HZ,
	.odr_wuf = ODR_40HZ,
};

static struct i2c_board_info kxte9_i2c_board_info = {
	I2C_BOARD_INFO( KXTE9_I2C_DEVICE, KXTE9_I2C_ADDRESS),
	.irq = OMAP_GPIO_IRQ(160),
	.platform_data = &board_kxte9_platform_data,
};
#endif


#if defined(CONFIG_VIDEO_VX6852) \
	|| defined(CONFIG_VIDEO_VX6852_MODULE)
static struct vx6852_platform_data board_vx6852_platform_data = {
	.fourcc = V4L2_PIX_FMT_SGRBG10,
	.bpp = 10,
	.ccp2_data_format = VX6852_CCP2_DATA_FORMAT_RAW10,
	.coarse_integration_time = 320,
	.analogue_gain_code_global = 192,	// 4x
	.vt_pix_clk_mhz = {			// extclk = 6.5 MHz
		.numerator = 637,
		.denominator = 10,
	},
	.vt_pix_clk_div = 10,
	.vt_sys_clk_div = 1,
	.pre_pll_clk_div = 1,
	.pll_multiplier = 98,
	.line_length_pck = 2500,
	.enable_vdig = board_cam_vcc_enable,
};

static struct i2c_board_info vx6852_i2c_board_info = {
	I2C_BOARD_INFO(VX6852_I2C_DEVICE, VX6852_I2C_ADDR),
	.platform_data = &board_vx6852_platform_data,
};
#endif // CONFIG_VIDEO_VX6852[_MODULE]

#if defined(CONFIG_LEDS_TPS6105X) \
	|| defined(CONFIG_LEDS_TPS6105X_MODULE)
static struct tps6105x_platform_data board_tps6105x_platform_data = {
	.enable_avin = board_cam_vcc_enable,
};

static struct i2c_board_info tps6105x_i2c_board_info = {
	I2C_BOARD_INFO(TPS6105X_I2C_DEVICE, TPS6105X_I2C_ADDR),
	.platform_data = &board_tps6105x_platform_data,
};
#endif // CONFIG_LEDS_TPS6105X[_MODULE]

#ifdef CONFIG_LEDS_LP8501 
/* Divides LED into group:
 * group 1 for core navi LED: LED_1, LED_2
 * group 2 for left white LED: LED_3
 * group 3 for right white LED: LED_5
 */
static struct led_cfg core_navi_center_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D2_PWM,
		.current_addr = D2_CURRENT_CTRL,
        	.control_addr = D2_CONTROL,
	},
	[1] = {
		.type  = WHITE,
		.pwm_addr = D1_PWM,
		.current_addr = D1_CURRENT_CTRL,
		.control_addr = D1_CONTROL,
	},
};

static struct led_cfg core_navi_right_group[] = {
	[0] = {
		.type = WHITE,
		.pwm_addr = D3_PWM,
		.current_addr = D3_CURRENT_CTRL,
		.control_addr = D3_CONTROL,
	},
};

static struct led_cfg core_navi_left_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D5_PWM,
		.current_addr = D5_CURRENT_CTRL,
		.control_addr = D5_CONTROL,
	},
};

static struct lp8501_led_config led_lp8501_data[] = {
	[GRP_1] = {
		.cdev = {
			.name = "core_navi_center",
		},
		.led_list = &core_navi_center_group[0],
		.nleds = ARRAY_SIZE(core_navi_center_group),
		.group_id = GRP_1,
		.hw_group = HW_GRP_NONE,
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
	[GRP_2] = {
		.cdev = {
			.name = "core_navi_left",
		},
		.led_list = &core_navi_left_group[0],
		.nleds = ARRAY_SIZE(core_navi_left_group),		
		.group_id = GRP_2,
		.hw_group = HW_GRP_NONE,
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
	[GRP_3] = {
		.cdev = {
			.name = "core_navi_right",
		},
		.led_list = &core_navi_right_group[0],
		.nleds = ARRAY_SIZE(core_navi_right_group),	
		.group_id = GRP_3,
		.hw_group = HW_GRP_NONE,
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
};

static struct lp8501_memory_config led_lp8501_memcfg = {
	.eng1_startpage = 0,
	.eng1_endpage = 1,
	.eng2_startpage = 2,
	.eng2_endpage = 3,
	.eng3_startpage = 4,
	.eng3_endpage = 5,
};

static struct lp8501_platform_data board_lp8501_data = {
	.leds = led_lp8501_data,
	.memcfg = &led_lp8501_memcfg,
	.nleds = ARRAY_SIZE(led_lp8501_data),
	.cp_mode = CONFIG_CPMODE_AUTO,
	.power_mode = CONFIG_POWER_SAVE_ON,
	.dev_name = "national_led",
};

static struct i2c_board_info lp8501_i2c_board_info = {
	I2C_BOARD_INFO(LP8501_I2C_DEVICE, LP8501_I2C_ADDR),
	.irq = OMAP_GPIO_IRQ(23),
	.platform_data = &board_lp8501_data,
};
#endif // CONFIG_LEDS_LP8501

static int __init board_init_i2c(void)
{
#ifdef CONFIG_KEYBOARD_MAXIM7359
	i2c_register_board_info( 3, &maxim7359_i2c_board_info, 1);
#endif

#ifdef CONFIG_LEDS_TPS6025X
	i2c_register_board_info( 3, &tps60252_i2c_board_info, 1);
#endif

	if (board_type == EVT0b) {
#ifdef CONFIG_LEDS_MAXIM8831
		i2c_register_board_info( 3, &maxim8831_i2c_board_info, 1);
#endif
	}

	if (board_type == EVT1 || board_type >= DVT3) {
#if defined(CONFIG_ACCELEROMETER_KXSD9) \
	|| defined(CONFIG_ACCELEROMETER_KXSD9_MODULE)
	i2c_register_board_info( 3, &kxsd9_i2c_board_info, 1);
#endif
	}

	if (board_type >= EVT2 && board_type  < DVT3 ) {
#if defined(CONFIG_ACCELEROMETER_KXTE9) \
	|| defined(CONFIG_ACCELEROMETER_KXTE9_MODULE) 
		i2c_register_board_info(3, &kxte9_i2c_board_info, 1);
#endif // CONFIG_ACCELEROMETER_KXTE9
	}

#if defined(CONFIG_VIDEO_VX6852) \
	|| defined(CONFIG_VIDEO_VX6852_MODULE)
	i2c_register_board_info(2, &vx6852_i2c_board_info, 1);
#endif // CONFIG_VIDEO_VX6852[_MODULE]

#if defined(CONFIG_LEDS_TPS6105X) \
	|| defined(CONFIG_LEDS_TPS6105X_MODULE)
	i2c_register_board_info(2, &tps6105x_i2c_board_info, 1);
#endif // CONFIG_LEDS_TPS6105X[_MODULE]

	if (board_type >= EVT1) {
#ifdef CONFIG_LEDS_LP8501 
		i2c_register_board_info(3, &lp8501_i2c_board_info, 1);
#endif // CONFIG_LEDS_LP8501
	}

	(void) platform_device_register(&omap_i2c_device1);
	(void) platform_device_register(&omap_i2c_device2);
	(void) platform_device_register(&omap_i2c_device3);
	return 0;
}
#else

static int __init board_init_i2c(void) { return 0; }

#endif // I2C


#if defined CONFIG_OMAP_VIBRATOR || defined CONFIG_OMAP_VIBRATOR_MODULE
static struct platform_device board_vibe_device = {
	.name = VIBE_DEVICE,
	.id   = -1,
};
#endif

#ifdef CONFIG_FB_OMAP

static u32 disp_type = PANEL_TYPE_ACX567AKM_TS2;
static u32 disp_use_gpio_reset  = 1;	/* use GPIO pin to reset display?  */
static u32 disp_use_gpio_enable = 0;	/* use GPIO pin to enable/dis display? */
static u32 disp_vsync_gated     = 0;
static u32 disp_use_sdi         = 1;
static u32 disp_right_margin    = 12;

static int  __init lcd_args(char *str)
{
	if (!strcmp(str, "excalibur_ts1")) {
		disp_type = PANEL_TYPE_ACX567AKM_TS1;
		disp_use_gpio_reset = 0;
		disp_vsync_gated = 1;
		disp_right_margin = 16;
	}
	return 0;
}
__setup("lcd=", lcd_args);

static struct platform_device board_lcd_device = {
	.name = "lcd",
	.id = 0,
};

static struct controller_platform_data board_lcd_controller_data = {
	.parent = &(board_lcd_device.dev),
	.screen_info 	= {
		.xres 		= 320,
		.yres 		= 480,
		.xres_virtual 	= 320,
		.yres_virtual 	= (480 * 3),	/* triple buffering */
		.xoffset 	= 0,
		.yoffset 	= 0,
		.bits_per_pixel = 32,
		.grayscale 	= 0,
		.red 		= {16, 8, 0},
		.green 		= {8, 8, 0},
		.blue 		= {0, 8, 0},
		.transp 	= {24, 8, 0},
		.nonstd 	= 0,
		.activate 	= FB_ACTIVATE_NOW,
		.height 	= -1,
		.width 		= -1,
		.accel_flags 	= 0,
		.pixclock 	= 95133,		/* picoseconds */
		.left_margin 	= 16,			/* pixclocks   */
		.right_margin 	= 12,			/* pixclocks   */
		.upper_margin 	= 3,			/* line clocks */
		.lower_margin 	= 3,			/* line clocks */
		.hsync_len 	= 4,			/* pixclocks   */
		.vsync_len 	= 2,			/* line clocks */
		.sync 		= 0,
		.vmode 		= FB_VMODE_NONINTERLACED,
		.rotate 	= 0,
		.reserved[0] 	= 0,
	},
	.dss_config_lpr_on = {
		.fifo_hi_thrs = 3000, /* FIFO high threshold */
		.fifo_lo_thrs = 2500, /* FIFO low  threshold */
		.logdiv = 9,          /* Logic clock divider (LCD) */
		.pixdiv = 1,          /* Pixel clock divider (PCD) */
	},
	.dss_config_lpr_off = {
		.fifo_hi_thrs = 1020, /* FIFO high threshold */
		.fifo_lo_thrs = 956,  /* FIFO low  threshold */
		.logdiv = 1,          /* Logic clock divider (LCD) */
		.pixdiv = 9,          /* Pixel clock divider (PCD) */
	},
	.use_sdi 	= &disp_use_sdi,
	.vsync_gated 	= &disp_vsync_gated,
	.right_margin   = &disp_right_margin,

	.dss_reg_base	= DSS_REG_BASE,
	.dss_reg_offset	= DSS_REG_OFFSET,
	.dispc_reg_offset = DISPC_REG_OFFSET,

	.pixclk_min	= 95133,
	.pixclk_max	= 93744,

	.sdi_color_depth 	= DSS_SDI_CONTROL_BWSEL_24B,
	.sdi_num_of_data_pairs 	= DSS_SDI_CONTROL_PRSEL_2PAIR,
	.sdi_pll_lock_method	= DSS_PLL_CONTROL_LOCKSEL_PHASE,
	.sdi_pll_to_pclk_ratio	= 0x0f,		/* PDIV */
	.sdi_freq_selector	= 0x7,		/* 1.75-2Mhz (optimal) */
	.sdi_power_enable	= board_sdi_vcc_enable,
};

static struct platform_device board_lcd_controller = {
	.name = "lcd-controller",
	.id = -1,
	.dev = {
		.platform_data = &board_lcd_controller_data,
	},
};

static struct bl_platform_data board_bl_data = {
	.parent = &(board_lcd_device.dev),
	.timer_id	= 9,
	.i2c_id		= 3,
	.led_id		= 0,
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
	.id   = -1,
	.dev  = {
		.platform_data = &board_bl_data,
	},
};

static struct panel_platform_data board_lcd_data = {
	.parent = &(board_lcd_device.dev),
	.panel_use_gpio_enable = &disp_use_gpio_enable,
	.panel_use_gpio_reset  = &disp_use_gpio_reset,
	.panel_reset_gpio      = 68,
	.panel_type            = &disp_type,

	/* some lcds control their own backlight */
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

#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
	.sb_config  = (void *) &acx567akm_tps6160_backlight_config,
	.sbc_init   = board_sbc_init,
	.sbc_remove = board_sbc_remove,
	.sbc_preon  = board_sbc_preon,
	.sbc_on     = board_sbc_on,
	.sbc_brightness = board_sbc_brightness,
	.sbc_off    = board_sbc_off,
	.use_sb     = 1,
	.on_full    = 1,
#endif
};
#endif /* CONFIG_FB_OMAP */

/*
 *  SPI devices
 */
#if defined(CONFIG_TOUCHSCREEN_CY8MRLN) || defined (CONFIG_TOUCHSCREEN_TM1129)
static struct omap2_mcspi_device_config touchpanel_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CY8MRLN
static int cy8mrln_switch_mode(int flag)
{
	if (flag == NORM_OP) {
		omap_cfg_reg ( "AB3_3430_SPI1_CLK" );         // SPI1 (GPIO 171)
		omap_cfg_reg ( "AB4_3430_SPI1_O" );           // SPI1 (GPIO 172)
		omap_cfg_reg ( "AC2_3430_SPI1_CS0" );         // SPI1 (GPIO 174)
	} else {
		omap_cfg_reg ( "AB3_3430_SPI1_SCL" );         // SPI1 (GPIO 171)
		omap_cfg_reg ( "AB4_3430_SPI1_MOSI_MIRROR" ); // SPI1 (GPIO 172)
		omap_cfg_reg ( "AC2_3430_SPI1_SDA" );         // SPI1 (GPIO 174)
	}

	return 0;
};
static struct  cy8mrln_platform_data touchpanel_platform_data = 
{
	//.xres_gpio = 0,	// Cypress PSoC XRES pin. Not connected.
	.enable_gpio = 163,	// Enable VDD
	.scl_gpio = 171,	// CLK signal in mode 0
	.mosi_gpio = 172,	// MOSI 
	.sda_gpio = 174,	// SC signal in mode 0
	.flip_x = 0,            // Don't flip the X coordinate value.
	.flip_y = 1,            // Flip the Y coordinate value.
	.switch_mode = cy8mrln_switch_mode,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_TM1129
static struct  tm1129_platform_data touchpanel_platform_data = 
{
	.penup_timeout = 40,    // msec
	.flip_x = 0,            // Don't flip the X coordinate value.
	.flip_y = 1,            // Flip the Y coordinate value.
};
#endif

#ifdef CONFIG_FB_OMAP
static struct omap2_mcspi_device_config lcdpanel_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,
};
#endif

static struct spi_board_info board_spi_board_info[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_CY8MRLN
	{
		.modalias        = "cy8mrln",
		.bus_num         = 1,
		.chip_select     = 0,
		.max_speed_hz    = 2000000,
		.platform_data   = &touchpanel_platform_data,
		.controller_data = &touchpanel_mcspi_config,
		.irq             = OMAP_GPIO_IRQ ( 164 ),
	},
#endif

#ifdef CONFIG_TOUCHSCREEN_TM1129
	{
		.modalias        = "tm1129",
		.bus_num         = 1,
		.chip_select     = 0,
		.max_speed_hz    = 2000000,
		.platform_data   = &touchpanel_platform_data,
		.controller_data = &touchpanel_mcspi_config,
		.irq             = OMAP_GPIO_IRQ ( 164 ),
	},
#endif

#ifdef CONFIG_FB_OMAP
	{
		.modalias        = "lcdpanel",
		.bus_num         = 3,
		.chip_select     = 0,
		.max_speed_hz    = 1500000,
		.platform_data   = &board_lcd_data,
		.controller_data = &lcdpanel_mcspi_config,
	},
#endif
};

/*
 *  1-wire devices
 */
#ifdef CONFIG_W1_MASTER_OMAP
/* HDQ clock shutdown timeout in milliseconds. */
static u32 hdq_shutdown_timeout = 100;

static struct resource hdq_resources1[] = {
	{
		.start  = HDQ_BASE,
		.end    = HDQ_BASE + 0x18,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_34XX_HDQ_IRQ,
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
 * Video Out
 */
#ifdef CONFIG_VIDEO_OMAP24XX_VIDEOOUT
static struct vout_mem_alloc_data vout_mem_data = {
	.mem_width		= 720,
	.mem_height		= 480,
	.mem_bytes_per_pixel	= 2,
	.forced_index = 2,
	.use_forced_index = 1,
};
static struct platform_device omap24xxv1out_dev = {
	.name = "omap24xxvout1",
	.id = 11,
	.dev = {
		.platform_data = &vout_mem_data,
	},
};
#endif	/* CONFIG_VIDEO_OMAP24xx_VIDEOOUT */


#ifdef CONFIG_FB_OMAP_ON_VIDEO_LAYER
struct omapfbv_platform_data omap_fbv_data =
{
	.video_layer = OMAP2_VIDEO2,
	.pixelformat = V4L2_PIX_FMT_RGB32,
	.colorspace = V4L2_COLORSPACE_JPEG,

	.screeninfo = {
		.xres		= 320,
		.yres 		= 480,
		.xres_virtual 	= 320,
		.yres_virtual	= (480*3),
		.bits_per_pixel = 32,
		.grayscale 	= 0,
		.red 		= {16, 8, 0},
		.green 		= {8, 8, 0},
		.blue 		= {0, 8, 0},
		.transp 	= {24, 8, 0},
		.nonstd 	= 0,
		.activate 	= FB_ACTIVATE_NOW,
		.height 	= -1,
		.width 		= -1,
	},
	.fix = {
		.type 		= FB_TYPE_PACKED_PIXELS,
		.visual 	= FB_VISUAL_TRUECOLOR,
		.xpanstep 	= 0,
		.ypanstep 	= 1,
		.line_length 	= (320*4),
		.accel 		= FB_ACCEL_NONE
	},
};

static struct platform_device omap_fbv_device = {
	.name = "omap_fbv",
	.id   = 0,
	.dev  = {
		.platform_data  = &omap_fbv_data,
	},
};

#endif /* CONFIG_FB_OMAP_ON_VIDEO_LAYER */

/*
 *  RTC
 */
#ifdef CONFIG_RTC_DRV_TWL4030

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

static struct twl4030rtc_platform_data board_twl4030rtc_data = {
	.init = &twl4030_rtc_init,
	.exit = &twl4030_rtc_exit,
};

static struct platform_device board_twl4030rtc_device = {
	.name = "twl4030_rtc",
	.id   = -1,
	.dev  = {
		.platform_data	= &board_twl4030rtc_data,
	},
};

#endif // CONFIG_RTC_DRV_TWL4030

#if defined(CONFIG_VIDEO_OMAP34XX_ISP) \
	|| defined(CONFIG_VIDEO_OMAP34XX_ISP_MODULE)
static struct resource omap34xx_isp_resources[] = {
	[OMAP34XX_CCP2_RESOURCE_MEM] = {
	.start = OMAP34XX_CCP2B_BASE,
	.end = OMAP34XX_CCP2B_BASE + 0x200,
	.flags = IORESOURCE_MEM,
	},
};

static struct omap34xx_isp_platform_data board_omap34xx_isp_data = {
	.isp_sysconfig_midle_mode =
		OMAP34XX_ISP_SYSCONFIG_MIDLE_MODE_SMART_STANDBY,
	.isp_sysconfig_auto_idle = 1,
	.isp_ctrl_sync_detect = OMAP34XX_ISP_CTRL_SYNC_DETECT_VS_FALLING,
	.isp_ctrl_cbuff_autogating = 0,
	.isp_ctrl_par_ser_clk_sel = OMAP34XX_ISP_CTRL_PAR_SER_CLK_SEL_CSIB,

	.tctrl_ctrl_insel = OMAP34XX_ISP_TCTRL_CTRL_INSEL_VP,
	.tctrl_ctrl_divc = 216,
	.tctrl_frame_strb = 1,
	.tctrl_strb_delay = 26839,
	.tctrl_strb_length = 73161,

	.ccdc_syn_mode_vdhden = 1,
	.ccdc_syn_mode_datsiz = OMAP34XX_ISP_CCDC_SYN_MODE_DATSIZ_10_BITS,
	.ccdc_cfg_bswd = 1,
	.ccdc_fmtcfg_vpin = OMAP34XX_ISP_CCDC_FMTCFG_VPIN_BITS_9_0,

	.prv_pcr_gamma_bypass = 1,
	.prv_pcr_cfaen = 1,
	.prv_wb_dgain = U10Q8(1, 0),
	.prv_wbgain_coef3 = U8Q5(1, 0),
	.prv_wbgain_coef2 = U8Q5(1, 0),
	.prv_wbgain_coef1 = U8Q5(1, 0),
	.prv_wbgain_coef0 = U8Q5(1, 0),
	.prv_rgb_mat1_mtx_rr = S12Q8(1, 0),
	.prv_rgb_mat3_mtx_gg = S12Q8(1, 0),
	.prv_rgb_mat5_mtx_bb = S12Q8(1, 0),
	.prv_csc0_cscry = S10Q8(0, 77),
	.prv_csc0_cscgy = S10Q8(0, 150),
	.prv_csc0_cscby = S10Q8(0, 29),
	.prv_csc1_cscrcb = -S10Q8(0, 43),
	.prv_csc1_cscgcb = -S10Q8(0, 85),
	.prv_csc1_cscbcb = S10Q8(0, 128),
	.prv_csc2_cscrcr = S10Q8(0, 128),
	.prv_csc2_cscgcr = -S10Q8(0, 107),
	.prv_csc2_cscbcr = -S10Q8(0, 21),

	.rsz_cnt_vstph = 0,
	.rsz_cnt_hstph = 0,

	.control_csirxfe_csib_selform =
		OMAP34XX_CONTROL_CSIRXFE_CSIB_SELFORM_DATA_STROBE,
	.ccp2_sysconfig_mstandby_mode =
		OMAP34XX_CCP2_SYSCONFIG_MSTANDBY_MODE_SMART_STANDBY,
	.ccp2_ctrl_io_out_sel = OMAP34XX_CCP2_CTRL_IO_OUT_SEL_PARALLEL,
	.ccp2_ctrl_phy_sel = OMAP34XX_CCP2_CTRL_PHY_SEL_DATA_STROBE,
	.ccp2_lc0_ctrl_format = OMAP34XX_CCP2_LCx_CTRL_FORMAT_RAW10_VP,
	.ccp2_lc0_ctrl_crc_en = 1,
	.ccp2_lc0_stat_size_sof = 2,
	.ccp2_lc0_dat_start_vert = 2,

	/* (((2048 + 8) * 10) / 8) * 2 */
	.smia10_sof_len = 5140,

	.video_width = 480,
	.video_height = 320,
	.video_fourcc = V4L2_PIX_FMT_UYVY,

	/* 1x capture, 2x stats */
	.dma_pool = (6291456 * 1) + (94208 * 2),
};

static struct platform_device board_omap34xx_isp_device = {
 	.name = OMAP34XX_ISP_DEVICE,
 	.id = -1,
 	.dev = {
		.platform_data = &board_omap34xx_isp_data,
	},
	.num_resources = ARRAY_SIZE(omap34xx_isp_resources),
	.resource = omap34xx_isp_resources,
};
#endif // CONFIG_VIDEO_OMAP34XX_ISP[_MODULE]

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
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       = "AUDIO_PCM_R_TO_BT_DATA",
		.gpio       =  AUDIO_PCM_R_TO_BT_DATA_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  1,  // initialy high
		.pin_mode   =  AUDIO_PCM_R_TO_BT_DATA_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       = "AUDIO_PCM_REMOTE_MASTER",
		.gpio       =  AUDIO_PCM_REMOTE_MASTER_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  1,  // initialy high
		.pin_mode   =  AUDIO_PCM_REMOTE_MASTER_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
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
		.def_level  =  -1,  // unset
		.pin_mode   =  MODEM_POWER_ON_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       = "boot_mode",
		.gpio       =  MODEM_BOOT_MODE_GPIO,
		.act_level  =  -1,  // not applicable
		.direction  =  0,   // an output
		.def_level  =  0,   // initially low
		.pin_mode   =  MODEM_BOOT_MODE_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       =  "wakeup_modem",
		.gpio       =  MODEM_WAKE_MODEM_GPIO,
#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
		.options    =  PIN_READ_ONLY, // controlled by handshaking code
#endif	
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  0,  // initialy low
		.pin_mode   =  MODEM_WAKE_MODEM_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       =  "wakeup_app",
		.gpio       =  MODEM_WAKE_APP_GPIO,
		.act_level  =  1,  // active high
		.direction  =  1,  // an input
		.def_level  = -1,  // unset
		.pin_mode   =  MODEM_WAKE_APP_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       =  "wakeup_app_usb",
		.gpio       =  MODEM_WAKE_APP_USB_GPIO,
		.act_level  =  1,  // active high
		.direction  =  1,  // an input
		.def_level  = -1,  // unset
		.pin_mode   =  MODEM_WAKE_APP_USB_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
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
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       =  "wake",
		.gpio       =  BT_WAKE_GPIO,
		.act_level  =  0, // active Low
		.direction  =  0, // an output
		.def_level  =  0, // default level (low)
		.pin_mode   =  BT_WAKE_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
	{
		.name       =  "host_wake",
		.gpio       =  BT_HOST_WAKE_GPIO,  //
		.act_level  =  1,  // active high
		.direction  =  1,  // an input
		.def_level  = -1,  // undefined
		.options    =  PIN_WAKEUP_SOURCE,
		.pin_mode   =  BT_HOST_WAKE_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  IRQF_TRIGGER_RISING,
		.irq_handle_mode = IRQ_HANDLE_AUTO,
	},
};

/*
 *   WiFi Pins
 */
static struct user_pin wifi_pins[] = {
	{
		.name       =  "reset",
		.gpio       =  WIFI_RESET_GPIO,
		.act_level  =  0, // active low
		.direction  =  0, // an output
		.def_level  =  0, // default level (low)
		.pin_mode   =  WIFI_RESET_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
};

/*
 *   Camera pins
 */
static struct user_pin camera_pins[] = {
	{
		.name       = "flash_sync",
		.gpio       =  CAM_FLASH_SYNC_GPIO,
		.act_level  =  1,  // active high
		.direction  =  0,  // an output
		.def_level  =  0,  // initialy low
		.pin_mode   =  CAM_FLASH_SYNC_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
	},
};


/*
 *   Power pins
 */
static struct user_pin power_pins[] = {
	{
		.name       =  "chg_bypass",
		.gpio       =  CHG_BYPASS_GPIO,
		.act_level  =  0,  // active low
		.direction  =  0,  // an output
		.def_level  = -1,  // unset
		.pin_mode   =  CHG_BYPASS_GPIO_PIN_MODE,
		.sysfs_mask =  0777,
		.options    =  0,
		.irq_config =  0,
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
	{
		.set_name = "audio",
		.num_pins = ARRAY_SIZE(audio_pins),
		.pins     = audio_pins,
	},
	{
		.set_name = "wifi",
		.num_pins = ARRAY_SIZE(wifi_pins),
		.pins     = wifi_pins,
	},
	{
		.set_name = "camera",
		.num_pins = ARRAY_SIZE(camera_pins),
		.pins     = camera_pins,
	},
	{
		.set_name = "power",
		.num_pins = ARRAY_SIZE(power_pins),
		.pins     = power_pins,
	},
};

static struct user_pins_platform_data board_user_pins_pdata = {
	.num_sets = ARRAY_SIZE(board_user_pins_sets),
	.sets     = board_user_pins_sets,
};

static struct platform_device board_user_pins_device = {
	.name = "user-pins",
	.id   = -1,
	.dev  = {
		.platform_data	= &board_user_pins_pdata,
	}
};
#endif

struct sirloin_headset_detect_platform_data board_headset_detect_pdata;

/*
 *  Headset detect
 */
static struct platform_device board_hs_det_device = {
	.name = "headset-detect",
	.id   = -1,
	.dev  = {
		.platform_data = &board_headset_detect_pdata,
	}
};


/*
 * HSDL9100 Proximity Sensor
 */
#if defined (CONFIG_HSDL9100_PROX_SENSOR) || \
    defined (CONFIG_HSDL9100_PROX_SENSOR_MODULE)

static struct hsdl9100_platform_data sirloin_hsdl9100_platform_data = {
	.dev_name = HSDL9100_DEVICE,

	.detect_gpio = SIRLOIN_HSDL9100_DETECT_GPIO,
	.enable_gpio = SIRLOIN_HSDL9100_ENABLE_GPIO,

	.ledon_dm_timer = 8,     /* GPT8 */
	.ledon_hz       = 18000, /* 18 kHz pulse frequency */
	.ledon_duty     = 50,    /* 50% duty cycle */

	.scan_off_time_no_det = 100, /* default time between scans in ms while
					no object has yet been detected */
	.scan_off_time_det    = 333, /* default time between scans in ms when
					an object has been detected */
	.scan_on_time         = 2,   /* default scan pulse length in ms */
};

/******************************************************************************
 *
 * prox_sense_device_release()
 *
 ******************************************************************************/
static void sirloin_prox_sense_device_release(struct device *dev)
{
	/* Nothing */
}

static struct platform_device sirloin_hsdl9100_prox_sense_device = {
	.name  = HSDL9100_DEVICE,
	.id    =  -1,
	.dev = {
		.platform_data = &sirloin_hsdl9100_platform_data,
		.release       = sirloin_prox_sense_device_release,
	},
};

#endif // CONFIG_HSDL9100_PROX_SENSOR

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
/*
 * MODEM handshaking activity monitor.
 */
static struct modem_activity_monitor_config modem_activity_cfg = {
	.app_wake_modem_gpio = MODEM_WAKE_MODEM_GPIO,   // modem wakeup pin
	.modem_wake_app_gpio = MODEM_WAKE_APP_GPIO,     // app wakeup   pin 
	.modem_wake_usb_gpio = MODEM_WAKE_APP_USB_GPIO, // usb wakeup   pin
#ifdef CONFIG_USER_PINS
	// if we are using user pins interface mark pins as configured
	.gpio_flags          = GPIO_FLG_AWM_CONFIGURED |
	                       GPIO_FLG_MWA_CONFIGURED |
	                       GPIO_FLG_MWU_CONFIGURED,
#endif
	.uart    = 0,       // uart to monitor
	.timeout = 250,     // timeout in msec
};

static struct platform_device modem_activity_device = {
        .name = "modem_activity",
        .id   = -1,
        .dev  = {
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
	.id   = -1,
	.dev  = {
		.platform_data = &nduid_cfg,
	},
};
#endif

/*
 *  sirloin devices
 */
static struct platform_device *board_devices[] __initdata = {
#ifdef CONFIG_USER_PINS
	&board_user_pins_device,
#endif
	&board_hs_det_device,
#ifdef CONFIG_RTC_DRV_TWL4030
	&board_twl4030rtc_device,
#endif
#if defined CONFIG_OMAP_VIBRATOR || defined CONFIG_OMAP_VIBRATOR_MODULE
	&board_vibe_device,
#endif
#if defined (CONFIG_HSDL9100_PROX_SENSOR) || \
    defined (CONFIG_HSDL9100_PROX_SENSOR_MODULE)
	&sirloin_hsdl9100_prox_sense_device,
#endif
#ifdef CONFIG_W1_MASTER_OMAP
	&omap_hdq_device1,
#endif
#ifdef CONFIG_VIDEO_OMAP24XX_VIDEOOUT
	&omap24xxv1out_dev,
#endif
#ifdef CONFIG_LIGHTSENSOR_AMBIENT6200
	&board_temt6200_device,
#endif
#if defined(CONFIG_OMAP_MISC_HSUART) || defined(CONFIG_OMAP_MISC_HSUART_MODULE)
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
#if defined(CONFIG_VIDEO_OMAP34XX_ISP) \
	|| defined(CONFIG_VIDEO_OMAP34XX_ISP_MODULE)
	&board_omap34xx_isp_device,
#endif // CONFIG_VIDEO_OMAP34XX_ISP[_MODULE]
#ifdef CONFIG_FB_OMAP_ON_VIDEO_LAYER
	&omap_fbv_device,
#endif // CONFIG_FB_OMAP_ON_VIDEO_LAYER

};

/*
 *  sirloin configs
 */

static struct omap_board_config_kernel  board_config[] __initdata = {
	{ OMAP_TAG_UART,    &board_uart_config},
#ifdef CONFIG_MMC
	{ OMAP_TAG_MMC,     &board_mmc_config },
#endif
};

/*
 *  Map common IO
 */
static void __init machine_map_io ( void )
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

/*
 *  Fixup machine
 */
static void __init
machine_fixup( struct machine_desc *desc, struct tag *t,
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

int board_usbmux_cfg(omap_usbmux_t mode)
{
	if (mode == USBMUX_USB) {
		omap_cfg_reg ("T27_HSUSB0_D0");
		omap_cfg_reg ("U28_HSUSB0_D1");
		omap_cfg_reg ("U27_HSUSB0_D2");
		omap_cfg_reg ("U26_HSUSB0_D3");
		omap_cfg_reg ("U25_HSUSB0_D4");
		omap_cfg_reg ("V28_HSUSB0_D5");
		omap_cfg_reg ("V27_HSUSB0_D6");
		omap_cfg_reg ("V26_HSUSB0_D7");
		omap_cfg_reg ("T28_USB0HS_CLK");
		omap_cfg_reg ("T26_USB0HS_NXT");
		omap_cfg_reg ("T25_USB0HS_STP");
		omap_cfg_reg ("R28_USB0HS_DIR");
		/* Configure UART3 pad configuration. */
		omap_cfg_reg("H20_3430_UART3_RX_IRRX");
		omap_cfg_reg("H21_3430_UART3_TX_IRTX");
	} else {
		/* UART3 over USB is active. Disable UART3 pad configuration. */
		omap_cfg_reg("H20_3430_GPIO165");
		omap_cfg_reg("H21_3430_GPIO166");
		/* Enable UART3 on USB */
		omap_cfg_reg("T27_UART3_TX_IRTX");
		omap_cfg_reg("U28_UART3_RX_IRRX");
		omap_cfg_reg("U27_UART3_RTS_SD");
		omap_cfg_reg("U26_UART3_CTS_RCTX");
	}

	return 0;
}

/*
 * Do board specific muxing
 */
static void __init board_mux ( void )
{
	/* SYS_CLKREQ */
	omap_cfg_reg("AF25_SYS_CLKREQ_GPIO_1");

	/* SYS_OFF_MODE */
	omap_cfg_reg("AF22_SYS_OFF_MODE_GPIO_9");

	// touch panel (tm1129)
	omap_cfg_reg ( "H18_3430_GPIO163_TP_RESET" ); // TP ENABLE (GPIO 163)
	omap_cfg_reg ( "H19_3430_GPIO164_TP_IRQ"   ); // TP IRQ   (GPIO 164)
	omap_cfg_reg ( "AB3_3430_SPI1_CLK" );         // SPI1 (GPIO 171)
	omap_cfg_reg ( "AB4_3430_SPI1_O"   );         // SPI1 (GPIO 172)
	omap_cfg_reg ( "AA4_3430_SPI1_I"   );         // SPI1 (GPIO 173)
	omap_cfg_reg ( "AC2_3430_SPI1_CS0" );         // SPI1 (GPIO 174)

	omap_cfg_reg ( "AG9_ETK_D9_GPIO_23" );

	/* backlight pin */
#if defined(CONFIG_FB_OMAP_BACKLIGHT_EVT) \
	|| defined(CONFIG_FB_OMAP_BACKLIGHT_EVT_MODULE)
	omap_cfg_reg ( "Y2_LCD_LED_PWM" );
#else /* !CONFIG_FB_OMAP_BACKLIGHT_EVT[_MODULE] */
	omap_cfg_reg ( "Y2_LCD_LED_PWM_OFFMODE" );
#endif /* CONFIG_FB_OMAP_BACKLIGHT_EVT[_MODULE] */

	if ( board_type >= DVT3 ) {
		omap_cfg_reg ( "Y2_LCD_LED_PWM" );
	}
	

	/* Charger Bypass */
	omap_cfg_reg ("V21_GPIO_158_CHG_BYPASS_EN");

	/* LCD SPI3 */
	omap_cfg_reg ( "H26_3430_LCD_SPI3_CLK" );
	omap_cfg_reg ( "H25_3430_LCD_SPI3_O"   );
	omap_cfg_reg ( "E28_3430_LCD_SPI3_I"   );
	omap_cfg_reg ( "J26_3430_LCD_SPI3_CS0" );

	/* WiFi Pins */
	omap_cfg_reg ("AE2_WL_SD1_CLK");
	omap_cfg_reg ("AG5_WL_SD1_CMD");
	omap_cfg_reg ("AH5_WL_SD1_D0");
	omap_cfg_reg ("AH4_WL_SD1_D1");
	omap_cfg_reg ("AG4_WL_SD1_D2");
	omap_cfg_reg ("AF4_WL_SD1_D3");
	omap_cfg_reg ("AF3_WL_IRQ");
	omap_cfg_reg ("AE3_WL_RST_nPWD");

	/* BT Pins */
	omap_cfg_reg ("AB1_BT_HOST_WAKE");
	omap_cfg_reg ("AB2_BT_nWAKE");
	omap_cfg_reg ("AA3_BT_nRST");

	/* MSECURE GPIO */
	omap_cfg_reg ("AF9_3430_MSECURE_GPIO");

	/* Modem Control */
	omap_cfg_reg ("AE1_3430_GPIO152");
	omap_cfg_reg ("AD1_3430_GPIO153");
	omap_cfg_reg ("AD2_3430_GPIO154");
	omap_cfg_reg ("AC1_3430_GPIO155");
	omap_cfg_reg ("AA21_3430_GPIO157");

	/* UART1 */
	omap_cfg_reg ("AA8_3430_UART1_TX");
	omap_cfg_reg ("Y8_3430_UART1_RX" );
	omap_cfg_reg ("AA9_3430_UART1_RTS");
	omap_cfg_reg ("W8_3430_UART1_CTS");

	/* UART2 */ 
	omap_cfg_reg ("AB26_3430_UART2_CTS");
	omap_cfg_reg ("AB25_3430_UART2_RTS");
	omap_cfg_reg ("AA25_3430_UART2_TX");
	omap_cfg_reg ("AD25_3430_UART2_RX");

	/* GPIO Keys*/
	omap_cfg_reg ("AE13_3430_GPIO17_CORE_NAVI");
	omap_cfg_reg ("AE7_3430_GPIO24_KEY_VOL_UP");
	omap_cfg_reg ("AF7_3430_GPIO25_KEY_VOL_DN");
	omap_cfg_reg ("AG7_3430_GPIO26_KEY_PTT");
	omap_cfg_reg ("AH7_3430_GPIO27_SLIDER_OPEN");
	omap_cfg_reg ("AG8_3430_GPIO28_RING");
	omap_cfg_reg ("AH8_3430_GPIO29_PWR_WIFI_KEY");
	omap_cfg_reg ("AH3_3430_GPIO137_OPTICAL_SLIDER");

	/* Light Sensor GPIO */
	omap_cfg_reg ("AF11_3430_GPIO14");

	/* Audio muxing happens in the audio (headset) driver. */


	/* proximity sensor */
#if defined (CONFIG_HSDL9100_PROX_SENSOR) || \
    defined (CONFIG_HSDL9100_PROX_SENSOR_MODULE)
	omap_set_gpio_debounce(SIRLOIN_HSDL9100_DETECT_GPIO, 1);
	omap_set_gpio_debounce_time(SIRLOIN_HSDL9100_DETECT_GPIO, 0x02);

	omap_cfg_reg ("T8_3430_GPIO55");       // INT   pin
	omap_cfg_reg ("R8_3430_GPIO56");       // GPT10_PWM
	omap_cfg_reg ("V3_3430_GPT8_PWM_EVT"); // GPT8_PWM
#endif

	/* McBSP1 */
	omap_cfg_reg("W21_MCBSP1_CLKX_GPIO_162");
	omap_cfg_reg("K26_MCBSP1_FSX_GPIO_161");
	omap_cfg_reg("U21_MCBSP1_DR_GPIO_159");

	/* McBSP2 */
	omap_cfg_reg("N21_MCBSP2_CLKX_GPIO_117");
	omap_cfg_reg("P21_MCBSP2_FSX_GPIO_116");
	omap_cfg_reg("R21_MCBSP2_DR_GPIO_118");
	omap_cfg_reg("M21_MCBSP2_DX_GPIO_119");

	/* McBSP3 */
        omap_cfg_reg("AF6_3430_GPIO_140");  //gpio 140 - mcbsp 3
        omap_cfg_reg("AE6_3430_GPIO_141");  //gpio 141 - mcbsp 3
        omap_cfg_reg("AF5_3430_GPIO_142");  //gpio 142 - mcbsp 3
        omap_cfg_reg("AE5_3430_GPIO_143");  //gpio 143 - mcbsp 3

	/* McBSP5 */
        /* audio GPIO's */
        omap_cfg_reg("AF10_ETK_CLK_GPIO_12");   //gpio 12 - mcbsp 5
        omap_cfg_reg("AH9_ETK_D5_GPIO_19");   //gpio 19 - mcbsp 5
        omap_cfg_reg("AF13_ETK_D6_GPIO_20");    //gpio 20 - mcbsp 5
        omap_cfg_reg("AE11_ETK_D4_GPIO_18");    //gpio 18 - mcbsp 5

        omap_cfg_reg("AB18_3430_GPIO41"); //R-TO-BT-CLK-SYNC~
        omap_cfg_reg("AC19_3430_GPIO42"); //R-TO-BT-DATA~
        omap_cfg_reg("AB19_3430_GPIO43"); //R-MASTER~

	/* POP-INT-1/2 */
	omap_cfg_reg("P8_GP_NCS6_GPIO_57");
	omap_cfg_reg("N8_GP_NCS7_GPIO_58");

	/* max8902a en */
	omap_cfg_reg("AG17_3430_GPIO99_CAM_PWD");
	omap_cfg_reg("B24_3430_GPIO101_CAM_PWD");

	/* ccp2 pins */
	omap_cfg_reg("K28_3430_CSIB_CLKP");
	omap_cfg_reg("L28_3430_CSIB_CLKN");
	omap_cfg_reg("K27_3430_CSIB_DATP");
	omap_cfg_reg("L27_3430_CSIB_DATN");

	/* tps6105x flash_sync */
	omap_cfg_reg("D25_3430_GPIO126");
	omap_cfg_reg("Y3_3430_GPIO_180");

	board_usbmux_cfg(omap_usbmux_mode());

        /* KXTE9 LED */
	omap_cfg_reg("T21_3430_GPIO160");

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

	/* Gas Gauge */
	omap_cfg_reg ("J25_HDQ_SIO");

	/* I2C */
	omap_cfg_reg ("K21_I2C1_SCL");
	omap_cfg_reg ("J21_I2C1_SDA");
	omap_cfg_reg ("AF15_I2C2_SCL");
	omap_cfg_reg ("AE15_I2C2_SDA");
	/* I2C3 Keypad */
	omap_cfg_reg ("AF14_I2C3_SCL");
	omap_cfg_reg ("AG14_I2C3_SDA");
	omap_cfg_reg ("AD26_I2C4_SCL");
	omap_cfg_reg ("AE26_I2C4_SDA");

	/* Keypad IRQ */
	omap_cfg_reg ("AE10_3430_GPIO13_KEY_INT");

	/* Boot mode pins */
	omap_cfg_reg ("AH26_SYS_BOOT0");
	omap_cfg_reg ("AG26_SYS_BOOT1");
	omap_cfg_reg ("AE14_SYS_BOOT2");
	omap_cfg_reg ("AF18_SYS_BOOT3");
	omap_cfg_reg ("AF19_SYS_BOOT4");
	omap_cfg_reg ("AE21_SYS_BOOT5");
	omap_cfg_reg ("AF21_SYS_BOOT6");

	/* SYS_NRESWARM */
	omap_cfg_reg ("AF24_SYS_NRESWARM_GPIO_30");
	
	/* Unconnected pins. Mux safely for suspend. */
	omap_cfg_reg ("AA28_GPIO84");
	omap_cfg_reg ("AA27_GPIO85");

	omap_cfg_reg ("Y21_GPIO_156");

	omap_cfg_reg ("AE4_GPIO_136");
	if (board_type < EVT3) {
		omap_cfg_reg ("AH3_GPIO_137");
	}

	omap_cfg_reg ("AE22_GPIO_186");
	omap_cfg_reg ("AH14_GPIO_21");

	omap_cfg_reg ("AC3_GPIO_175");

	omap_cfg_reg ("AH17_GPIO_100");
	omap_cfg_reg ("C24_GPIO_102");
	omap_cfg_reg ("D24_GPIO_103");
	omap_cfg_reg ("A25_GPIO_104");
	omap_cfg_reg ("B25_GPIO_109");
	omap_cfg_reg ("C26_GPIO_110");
	omap_cfg_reg ("C25_GPIO_96");
	omap_cfg_reg ("C27_GPIO_97");
	omap_cfg_reg ("A24_GPIO_94");
	omap_cfg_reg ("A23_GPIO_95");
	omap_cfg_reg ("C23_GPIO_98");
	omap_cfg_reg ("B23_GPIO_167");
	omap_cfg_reg ("AG19_GPIO_112");
	omap_cfg_reg ("AH19_GPIO_113");
	omap_cfg_reg ("AG18_GPIO_114");
	omap_cfg_reg ("AH18_GPIO_115");
	omap_cfg_reg ("AG22_GPIO_70");
	omap_cfg_reg ("AH22_GPIO_71");
	omap_cfg_reg ("AG23_GPIO_72");
	omap_cfg_reg ("AH23_GPIO_73");
	omap_cfg_reg ("AG24_GPIO_74");
	omap_cfg_reg ("AH24_GPIO_75");
	omap_cfg_reg ("E26_GPIO_76");
	omap_cfg_reg ("F28_GPIO_77");
	omap_cfg_reg ("F27_GPIO_78");
	omap_cfg_reg ("G26_GPIO_79");

	omap_cfg_reg ("AG25_GPIO_10");

	/* JTAG */
	omap_cfg_reg ("AA19_JTAG_TDO");
	omap_cfg_reg ("AA17_JTAG_nTRST");
	omap_cfg_reg ("AA18_JTAG_TMS");
	omap_cfg_reg ("AA20_JTAG_TDI");
	omap_cfg_reg ("AA13_JTAG_TCK");
	omap_cfg_reg ("AA12_JTAG_RTCK");
	omap_cfg_reg ("AA11_JTAG_EMU0");
	omap_cfg_reg ("AA10_JTAG_EMU1");

	omap_cfg_reg ("N4_GPMC_A1_GPIO_34");
	omap_cfg_reg ("M4_GPMC_A2_GPIO_35");
	omap_cfg_reg ("L4_GPMC_A3_GPIO_36");
	omap_cfg_reg ("K4_GPMC_A4_GPIO_37");
	omap_cfg_reg ("T3_GPMC_A5_GPIO_38");
	omap_cfg_reg ("R3_GPMC_A6_GPIO_39");
	omap_cfg_reg ("N3_GPMC_A7_GPIO_40");
	omap_cfg_reg ("Y1_GPMC_D15_GPIO_51");
	omap_cfg_reg ("W1_GPMC_D14_GPIO_50");
	omap_cfg_reg ("T2_GPMC_D13_GPIO_49");
	omap_cfg_reg ("R2_GPMC_D12_GPIO_48");
	omap_cfg_reg ("R1_GPMC_D11_GPIO_47");
	omap_cfg_reg ("P1_GPMC_D10_GPIO_46");
	omap_cfg_reg ("K2_GPMC_D9_GPIO_45");
	omap_cfg_reg ("H2_GPMC_D8_GPIO_44");
	omap_cfg_reg ("W2_GPMC_D7");
	omap_cfg_reg ("V2_GPMC_D6");
	omap_cfg_reg ("V1_GPMC_D5");
	omap_cfg_reg ("T1_GPMC_D4");
	omap_cfg_reg ("P2_GPMC_D3");
	omap_cfg_reg ("L2_GPMC_D2");
	omap_cfg_reg ("L1_GPMC_D1");
	omap_cfg_reg ("K1_GPMC_D0");

	omap_cfg_reg ("H3_GPMC_nCS1");
	omap_cfg_reg ("U8_GPMC_nCS3");
	omap_cfg_reg ("L8_GPMC_WAIT1");
	omap_cfg_reg ("J8_GPMC_WAIT3");

	omap_cfg_reg ("G4_GPMC_nCS0");
	omap_cfg_reg ("V8_GPMC_nCS2");
	omap_cfg_reg ("F4_GPMC_nWE");
	omap_cfg_reg ("G2_GPMC_nOE");
	omap_cfg_reg ("F3_GPMC_nADV_ALE");
	omap_cfg_reg ("G3_GPMC_nBE0_CLE");
	omap_cfg_reg ("U3_GPMC_nBE1");
	omap_cfg_reg ("H1_GPMC_nWP");
	omap_cfg_reg ("M8_GPMC_WAIT0");
	omap_cfg_reg ("K8_GPMC_WAIT2");
	omap_cfg_reg ("T4_GPMC_CLK");

}

static void scm_clk_init(void)
{
	struct clk *p_omap_ctrl_clk = NULL;

	p_omap_ctrl_clk = clk_get(NULL, "omapctrl_ick");
	if (p_omap_ctrl_clk != NULL) {
		if (clk_enable(p_omap_ctrl_clk) != 0) {
			printk(KERN_ERR "failed to enable scm clks\n");
			clk_put(p_omap_ctrl_clk);
		}
	}
	/* Sysconfig set to SMARTIDLE and AUTOIDLE */
	CONTROL_SYSCONFIG = 0x11;
}

/*
 *  sirloin init irq
 */
static void __init machine_init_irq ( void )
{
	omap2_init_common_hw();
	omap_init_irq();
	scm_clk_init();
	omap_gpio_init();
	board_mux();
}

/*
 *  Init machine
 */
static void __init machine_init ( void )
{
	/* set omap config */
	omap_board_config      = board_config;
	omap_board_config_size = ARRAY_SIZE(board_config);

	/* Serial */
	omap_serial_init();

	if (board_type >= EVT2) {
		board_headset_detect_pdata.polarity = 1;
	} else {
		board_headset_detect_pdata.polarity = 0;
	}

	if (board_type < DVT3) {
#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
		/* Reset for non-DVT3 devices */
		board_lcd_data.sb_config      = NULL;
		board_lcd_data.sbc_init       = NULL;
		board_lcd_data.sbc_remove     = NULL;
		board_lcd_data.sbc_preon      = NULL;
		board_lcd_data.sbc_on         = NULL;
		board_lcd_data.sbc_brightness = NULL;
		board_lcd_data.sbc_off        = NULL;
		board_lcd_data.use_sb  = 0;
		board_lcd_data.on_full = 0;
#endif
	}

	/* add platform devices */
	platform_add_devices   ( board_devices,
	                         ARRAY_SIZE(board_devices));
	/* USB */
	board_usb_init();

	/* SPI */
	spi_register_board_info( board_spi_board_info,
	                         ARRAY_SIZE(board_spi_board_info));

	/* Boot args */
	arm_put_reboot_args = boot_wall_put_args;

	/* enable panic on oops */
	panic_on_oops = 1;

#ifdef CONFIG_KEYBOARD_GPIO_PE
	/* GPIO KEYs*/
	board_gpio_keys_init();
#endif // CONFIG_KEYBOARD_GPIO_PE
}

arch_initcall(board_init_i2c);

MACHINE_START(SIRLOIN, "Sirloin OMAP3430 board")
	.phys_io      = SIRLOIN_3430_PHYS_IO_BASE,
	.io_pg_offst  = (((SIRLOIN_3430_PG_IO_VIRT)>>18)  & 0xfffc ),
	.boot_params  = SIRLOIN_3430_RAM_BASE+0x100,
	.fixup        = machine_fixup,
	.map_io       = machine_map_io,
	.init_irq     = machine_init_irq,
	.init_machine = machine_init,
	.timer        = &omap_timer,
MACHINE_END

