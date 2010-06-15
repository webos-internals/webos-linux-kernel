/* linux/arch/arm/mach-msm/board-chuck.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (C) 2008 Palm, Inc.
 * Author: Brian Swetland <swetland@google.com>
 * Author: Travis Geiselbrecht <travis@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/hsuart.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ctype.h>

#include <asm/setup.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/mach/mmc.h>

#include <asm/arch/board.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/msm_fb.h>
#include <asm/arch/vreg.h>
#include <asm/arch/gpio.h>
#include <asm/arch/msm_hsusb.h>
#include <asm/arch/msm_mmc.h>
#include <asm/arch/mpp.h>

#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>

#include <linux/gpio_keypad.h>
#include <linux/gpio_keys_pe.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <linux/isl29018_prox_sense.h>
#include <linux/user-pins.h>
#include <linux/hres_counter.h>
#include <linux/nduid.h>

#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/arch/dma.h>

#include "proc_comm.h"
#include "clock.h"
#include "clock-pcom.h"
#include "gpio.h"
#include "socinfo.h"
#include "pm.h"
#include "headset-detect.h"
#include "mux.h"
#include "board-chuck-gpios.h"
#include "smd_private.h"

#include <asm/arch/clock.h>

#ifdef CONFIG_LEDS_LP8501
#include <linux/i2c_lp8501_led.h>
#endif

#ifdef CONFIG_CY8C24894_TP
#include <linux/cy8c24894.h>
#endif

#ifdef CONFIG_MSM_VIBRATOR
#include <linux/vibrator.h>
#endif

#ifdef CONFIG_W1_MASTER_GPIO
#include <linux/w1-gpio.h>
#endif

#ifdef CONFIG_ACCELEROMETER_KXSD9 
#include <linux/i2c_kxsd9_accelerometer.h>
#endif

#ifdef CONFIG_THERMISTOR_TMP105
#include <linux/i2c_tmp105_thermistor.h>
#endif

#ifdef CONFIG_CHARGER_SMB339
#include <linux/i2c_smb339_charger.h>
#endif

#ifdef CONFIG_BLUETOOTH_POWER_STATE
#include <linux/bluetooth-power-pe.h>
#endif
 /*
 * Board type 
 */
#define EMU   	(0)
#define EVT0    (1)
#define EVT1    (2)
#define EVT2    (3)
#define EVT3    (4)
#define SPARE	(5)
#define DVT     (6)
#define DVT2	(7)
#define PVT		(8)

#define USE_HSUART_FOR_BT 1

#define MMC_DEV_ID_WIFI 2
#define MMC_DEV_ID_MMC  3

static u32 board_type = 0;

/* Board type selection */
static int  __init board_args(char *str)
{
	if (!strcmp(str, "pixie-emu")) {
		board_type = EMU;
	} else if (!strcmp(str, "pixie-evt0a")) {
		board_type = EVT0;
	} else if (!strcmp(str, "pixie-evt1")) {
		board_type = EVT1;
	} else if (!strcmp(str, "pixie-evt2")) {
		board_type = EVT2;
	} else if (!strcmp(str, "pixie-evt3")) {
		board_type = EVT3;
	} else if (!strcmp(str, "pixie-dvt1")) {
		board_type = DVT;
	} else if (!strcmp(str, "pixie-dvt2")) {
		board_type = DVT2;
	} else if (!strcmp(str, "pixie-pvt")) {
		board_type = PVT;
	} else {
		board_type = EMU;
	}
	return 0;
}
__setup("boardtype=", board_args);


static u8 rat_type = 0xFF;

#define	UMTS	0
#define CDMA	1

u8 chuck_get_rat_type(void)
{
	uint32_t *hwid = smem_alloc(SMEM_PALM_HW_INFO, sizeof(uint32_t));
	u8 rat = 0;

	if(hwid == NULL)
		return 0xFF;

	rat = *hwid >> 24;

	printk("Radio Air Technology = %s\n", rat == UMTS ? "UMTS" : "CDMA");

	return rat;
}

#ifdef CONFIG_FB_MSM

#define MDDI_CLIENT_CORE_BASE  0x108000
#define LCD_CONTROL_BLOCK_BASE 0x110000
#define PWM_BLOCK_BASE         0x140000
#define DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define SYSCLKENA   (MDDI_CLIENT_CORE_BASE|0x2C)
#define START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define PWM0OFF     (PWM_BLOCK_BASE|0x1C)

static struct msm_mddi_platform_data msm_mddi_lg_nt35380_pdata = {
	.has_vsync_irq	= 0,
	.vsync_gpio = 0,
	.skip_auto_detect = 1,
	.force_full_update = 0,
	.screen_width = 320,
	.screen_height = 400,
	.mfr_name = 0xDEAD,
	.product_code = 0xBEEF,
	.bits_per_pixel = 32,
};

/* Enable LCD use of vsync */
static int  __init lcd_enable_vsync(char* c)
{
	msm_mddi_lg_nt35380_pdata.has_vsync_irq = 1;
	msm_mddi_lg_nt35380_pdata.vsync_gpio = GPIO_VSYNC;

        return 0;
}
__setup("lcd_enable_vsync", lcd_enable_vsync);





static struct platform_device msm_mddi_lg_nt35380_device = {
	.name	= "msm_mddi",
	.id	= 0,
	.dev	= {
		.platform_data = &msm_mddi_lg_nt35380_pdata
	},
};

#endif

#if !USE_HSUART_FOR_BT
static struct resource msm_serial0_resources[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device msm_serial0_device = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(msm_serial0_resources),
	.resource	= msm_serial0_resources,
};
#endif
      
static struct resource msm_serial2_resources[] = {
	{
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART3_PHYS,
		.end	= MSM_UART3_PHYS + MSM_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device msm_serial2_device = {
	.name	= "msm_serial",
	.id	= 2,
	.num_resources	= ARRAY_SIZE(msm_serial2_resources),
	.resource	= msm_serial2_resources,
};

#ifdef CONFIG_MSM_UARTDM
/* 
 * 			UART DM resources 
 */

#if USE_HSUART_FOR_BT
/* UART1DM */
static struct resource msm_uart1dm_resources[] = {
	{
		.start	= INT_UART1DM_IRQ,
		.end	= INT_UART1DM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1DM_PHYS,
		.end	= MSM_UART1DM_PHYS + MSM_UART1DM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	 },
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	 },};
static struct platform_device msm_uart1dm_device = {
	.name	= "msm_uartdm",
 	.id	= 0,
 	.num_resources	= ARRAY_SIZE(msm_uart1dm_resources),
	.resource	= msm_uart1dm_resources,

};
#endif // USE_HSUART_FOR_BT

/* UART2DM */
static struct resource msm_uart2dm_resources[] = {
	{
		.start	= INT_UART2DM_IRQ,
		.end	= INT_UART2DM_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2DM_PHYS,
		.end	= MSM_UART2DM_PHYS + MSM_UART2DM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start = DMOV_HSUART2_TX_CHAN,
		.end   = DMOV_HSUART2_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	 },
	{
		.start = DMOV_HSUART2_TX_CRCI,
		.end   = DMOV_HSUART2_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	 },
};
static struct platform_device msm_uart2dm_device = {
	.name	= "msm_uartdm",
 	.id	= 1,
 	.num_resources	= ARRAY_SIZE(msm_uart2dm_resources),
	.resource	= msm_uart2dm_resources,

};
#endif // Of CONFIG_MSM_UARTDM

#ifdef CONFIG_MSM_KGSL
static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "kgsl_phys_memory",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
};
#endif


#ifdef CONFIG_MSM_VIBRATOR

void vibrator_power (int on)
{
	unsigned data1 = (PCOMM_PM_MPP_CFG_AOUT << 16) | PCOMM_PM_MPP_7;
	unsigned data2_on = ( (uint32_t) PCOMM_PM_MPP_AOUT_LEVEL_VREF_1p25_Volts << 16) |  ( (uint32_t) PCOMM_PM_MPP_AOUT_SWITCH_ON & 0xFFFF);
	unsigned data2_off = ( (uint32_t) PCOMM_PM_MPP_AOUT_LEVEL_VREF_1p25_Volts << 16) |  ( (uint32_t) PCOMM_PM_MPP_AOUT_SWITCH_ON & 0xFFFF);

	if (on) {
		msm_proc_comm (PCOM_PM_MPP_CONFIG, &data1, &data2_on);
	} else if (!on) {
		msm_proc_comm (PCOM_PM_MPP_CONFIG, &data1, &data2_off);
	}
}

static struct vibrator_platform_data msm_vibe_platform_data= {
	.vibrator_enable_gpio = GPIO_VIBRATOR_EN,
	.power = vibrator_power,
};

#endif

#ifdef CONFIG_W1_MASTER_GPIO

static struct w1_gpio_platform_data msm_w1_gpio_platform_data = {
	.pin = GPIO_BATTERY_GAUGE,
	.is_open_drain = 0,
	.w1_read_bit_drive_low_time = 6,
	.w1_read_bit_release_time = 9,
	.w1_read_bit_delay_time = 55,
};
#endif


static struct resource usb_resources[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

static int chuck_phy_init_seq[] = { 0x40, 0x06, -1 };
static int chuck_phy_post_reset_seq[] = { 0x2e, 0x31, 0x20, 0x32, -1 };

extern void msm_ulpi_init(void);

static void chuck_phy_reset(void)
{
#define APPS_RESET       (clkctl_base + 0x0210)
#define ULPI_TEST_CTL    (tlmmgpio01_base + 0x0274)
#define USB_PORTSC       (usb_base + 0x0184)
#define TLMMGPIO01_PHYS  0xa9000000
	void *clkctl_base, *tlmmgpio01_base, *usb_base;

	clkctl_base = ioremap(MSM_CLK_CTL_PHYS, MSM_CLK_CTL_SIZE);
	if (!clkctl_base) {
		return;
	}

	tlmmgpio01_base = ioremap(TLMMGPIO01_PHYS, MSM_GPIO1_SIZE);
	if (!tlmmgpio01_base) {
		iounmap(clkctl_base);
		return;
	}
	usb_base = ioremap(MSM_HSUSB_PHYS, MSM_HSUSB_SIZE);
	if (!usb_base) {
		iounmap(clkctl_base);
		iounmap(tlmmgpio01_base);
		return;
	}

	/* Force usbc to be in reset until PHY reset is done */
	writel(readl(APPS_RESET) | 0x800, APPS_RESET);

	/* Assert reset signal to the PHY while asserting reset to the core.
	* HW bug causes inverted logic---bit is cleared to reset PHY */
	writel(readl(APPS_RESET) & ~0x2000, APPS_RESET);

	/* Wait for PHY PLL lock - 300 us */
	udelay(300);

	/* De-assert PHY reset */
	writel(readl(APPS_RESET) | 0x2000, APPS_RESET);

	/* De-assert usbc reset */
	writel(readl(APPS_RESET) & ~0x800, APPS_RESET);

	writel(readl(ULPI_TEST_CTL) | 0x2, ULPI_TEST_CTL);

	msm_ulpi_init();

	/* Assert reset signal to the PHY while asserting reset to the core.
	* HW bug causes inverted logic---bit is cleared to reset PHY */
	writel(readl(APPS_RESET) & ~0x2000, APPS_RESET);

	/* Wait for PHY PLL lock - 300 us */
	udelay(300);

	/* De-assert PHY reset */
	writel(readl(APPS_RESET) | 0x2000, APPS_RESET);

	iounmap(clkctl_base);
	iounmap(tlmmgpio01_base);
	iounmap(usb_base);
}

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.phy_init_seq = chuck_phy_init_seq,
	.phy_post_reset_seq = chuck_phy_post_reset_seq,
	.phy_reset = chuck_phy_reset,
};

static struct platform_device msm_hsusb_device = {
	.name = "msm_hsusb",
	.id = -1,
	.num_resources = ARRAY_SIZE(usb_resources),
	.resource = usb_resources,
	.dev = {
		.coherent_dma_mask = 0xffffffff,
		.platform_data = &msm_hsusb_pdata,
	},
};

static struct android_pmem_platform_data android_pmem_fb_pdata = {
	.name = "fb",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_overlay_pdata = {
	.name = "pmem_overlay",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

#if defined(CONFIG_MSM_KGSL) && !defined(CONFIG_MSM_KGSL_MMU)
static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};
#endif

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

#if defined(CONFIG_MSM_KGSL) && !defined(CONFIG_MSM_KGSL_MMU)
static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};
#endif

#define MSM_PMEM_MDP_SIZE	0x800000
// There are fb0 and fb1.
// Each of them have 3 AGRB buffers.
// Total size: 3 x (320x400x32bpp=512K) = 1.5M
#define MSM_FB0_SIZE				0x180000
#define MSM_FB1_SIZE				0x180000

// Inernal overlay: 3 buffers, each buffer is 320x400x24bpp=375K
#define MSM_PMEM_OVERLAY_SIZE	0x120000

//Allocating physical regions that are NOT registered with pmem
static void __init chuck_allocate_mem_regions(void)
{
	void *addr;

	addr = alloc_bootmem_aligned(MSM_PMEM_OVERLAY_SIZE, 0x10000);
	msm_mddi_lg_nt35380_pdata.overlay_base = __pa(addr);
	msm_mddi_lg_nt35380_pdata.overlay_size = MSM_PMEM_OVERLAY_SIZE;
	printk(KERN_INFO "allocating 0x%lX bytes at %p (%lx physical) "
						"for overlay pmem\n", MSM_PMEM_OVERLAY_SIZE, addr, __pa(addr));

	// android_pmem_fb_pdata is not a real pmem device, it is never registered
	// with pmem driver. instead the data is passed to msm_fb and msm_adm
	addr = alloc_bootmem_aligned(MSM_FB0_SIZE, 0x10000);
       	android_pmem_fb_pdata.start = __pa(addr);
        android_pmem_fb_pdata.size = MSM_FB0_SIZE;
	msm_mddi_lg_nt35380_pdata.fb0_base = android_pmem_fb_pdata.start; 
	msm_mddi_lg_nt35380_pdata.fb0_size = android_pmem_fb_pdata.size;
	printk(KERN_INFO "allocating 0x%lX bytes at %p (%lx physical) "
						"for fb\n",	MSM_FB0_SIZE, addr, __pa(addr));

	addr = alloc_bootmem_aligned(MSM_FB1_SIZE, 0x10000);
	msm_mddi_lg_nt35380_pdata.fb1_base = __pa(addr);
	msm_mddi_lg_nt35380_pdata.fb1_size = MSM_FB1_SIZE;
	printk(KERN_INFO "allocating 0x%lX bytes at %p (%lx physical) "
						"for fb\n",	MSM_FB1_SIZE, addr, __pa(addr));
}

// ADSP pmem:
// Camera scenario -> total: 5.5M
//  Snapshot buffer: 1600x1200x12bpp ~= 3MB
//	Thumbnail buffer: 320x240x12bpp ~= 512K
//	Video buffers: 640x480x12bpp ~= 512K (x 4 buffers)
//	Jpeg engine internal allocations: ?
// Video decode scenario: TBD
// Currently set to 12MB
#define MSM_PMEM_ADSP_SIZE		0xC00000

#define MSM_PMEM_GPU1_SIZE	0x1C00000
#define MSM_GPU_PHYS_SIZE	SZ_2M

static void __init chuck_allocate_pmem_regions(void)
{
	void *addr;
	unsigned long size;
	
	size = MSM_PMEM_ADSP_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	android_pmem_adsp_pdata.start = __pa(addr);
	android_pmem_adsp_pdata.size = size;
	printk(KERN_INFO "allocating 0x%lX bytes at %p (%lx physical) "
						"for adsp pmem\n", size, addr, __pa(addr));

#if defined(CONFIG_MSM_KGSL)
	
#if !defined(CONFIG_MSM_KGSL_MMU)	
	size = MSM_PMEM_GPU1_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	android_pmem_gpu1_pdata.start = __pa(addr);
	android_pmem_gpu1_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for gpu1 pmem\n", size, addr, __pa(addr));
#endif
	
	size = MSM_GPU_PHYS_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	kgsl_resources[1].start = __pa(addr);
	kgsl_resources[1].end = kgsl_resources[1].start + size - 1;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
		"for KGSL pmem\n", size, addr, __pa(addr));
#endif
}

#ifdef CONFIG_MSM_CAMERA

void camera_flash_current_ma (int mode, int type, int current_ma)
{
	unsigned data1 = (PCOMM_PM_MPP_CFG_FLASH_SINK << 16) | 0;
	unsigned data2;
	
	data2	= (((uint32_t)mode & 0xFF) << 24) 	| 
			  (((uint32_t)type & 0xFF) << 16) 	|
			  ((uint32_t)current_ma & 0xFFFF);

	msm_proc_comm (PCOM_PM_MPP_CONFIG, &data1, &data2);
}

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_flash_current_ma = camera_flash_current_ma,
	.sensor_reset	= 1,
	.sensor_pwd	= 0,
	.vcm_pwd	= 0, /* Not used yet */
	.sensor_name = "ov265x"
};

static struct platform_device msm_camera_device = {
	.name		= "msm_camera",
	.id		= 0,
	.dev		= {
		.platform_data = &msm_camera_device_data,
	},
};
#endif

int msm_check_wake_source(int gpio)
{
	struct msm_pm_smem_t *msm_pm_smem_data = smem_alloc(SMEM_APPS_DEM_SLAVE_DATA,
		sizeof(*msm_pm_smem_data));

	if (msm_pm_smem_data == NULL) {
		printk(KERN_ERR "%s: failed to get smsm_data\n", __func__);
		return 0;
	}

	if((msm_pm_smem_data->wakeup_reason & DEM_WAKEUP_REASON_GPIO) &&
		(gpio == msm_pm_smem_data->gpio))
	{
		return 1;
	}

	return 0;
}

#ifdef CONFIG_KEYBOARD_GPIO_PE
static struct gpio_keys_button pixie_gpio_keys_buttons[] = {
	[0] = {
		.code        = KEY_VOLUMEUP,
		.gpio        = 123,
		.active_low  = 1,
		.desc        = "volume up",
		.debounce    = 10,
		.type        = EV_KEY,
		.wakeup      = 0,
		.options     = 0,
		.noise_mode  = MODE_MISSING_INT_INTERPRETATION
	},
	[1] = {
		.code        = KEY_VOLUMEDOWN,
		.gpio        = 122,
		.active_low  = 1,
		.desc        = "volume down",
		.debounce    = 10,
		.type        = EV_KEY,
		.wakeup      = 0,
		.options     = 0,
		.noise_mode  = MODE_MISSING_INT_INTERPRETATION
	},
	[2] = {
		.code        = SW_RINGER,
		.gpio        = 114,
		.active_low  = 0,
		.desc        = "ring silence",
		.debounce    = 100,
		.type        = EV_SW,
		.wakeup      = 1,
		.noise_mode  = MODE_NOISE_INTERPRETATION,
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
		.options     = OPT_REBOOT_TRIGGER | OPT_REBOOT_TRIGGER_EDGE,
#endif
	},
	[3] = {
		.code        = KEY_END,
		.gpio        = 124,
		.active_low  = 1,
		.desc        = "power",
		.debounce    = 10,
		.type        = EV_KEY,
		.wakeup      = 1,
		.noise_mode  = MODE_MISSING_INT_INTERPRETATION,
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
		.options     = OPT_REBOOT_TRIGGER | OPT_REBOOT_TRIGGER_LEVEL,
#endif		
	},};

static struct gpio_keys_platform_data pixie_gpio_keys = {
	.buttons  = pixie_gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(pixie_gpio_keys_buttons),
};

static struct platform_device pixie_gpio_keys_device = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data  = &pixie_gpio_keys,
	},
};
#endif

#ifdef CONFIG_KEYBOARD_GPIO_MATRIX
/*
 *  *   Key proximity map
 *   */
static u8  board_key_prox_map[][6] = {
        {  1,  7,             0xFF }, // q [ 0]
        {  0,  2,  8,         0xFF }, // w [ 1]
        {  1,  9,  3,         0xFF }, // e [ 2]
        {  2, 10,  4,         0xFF }, // r [ 3]
        {  3, 11,  5,         0xFF }, // t [ 4]
        {  4, 12,  6,         0xFF }, // y [ 5]
        {  5, 13, 28,         0xFF }, // u [ 6]
        {  0,  8, 21,         0xFF }, // a  [7]
        {  7,  1,  9, 15,     0xFF }, // s  [8]
        {  8,  2, 10, 16,     0xFF }, // d  [9]
        {  9,  3, 11, 17,     0xFF }, // f  [10]
        { 10,  4, 12, 18,     0xFF }, // g  [11]
        { 11,  5, 13, 19,     0xFF }, // h  [12]
        { 12,  6, 31, 20,     0xFF }, // j  [13]
        { 15, 16, 22,         0xFF }, // sh [14]
        { 21,  8, 16, 14,     0xFF }, // z  [15]
        { 15,  9, 17, 14, 22, 0xFF }, // x  [16]
        { 16, 10, 18, 22, 23, 0xFF }, // c  [17]
        { 17, 11, 19, 23,     0xFF }, // v  [18]
        { 18, 12, 20, 23,     0xFF }, // b  [19]
        { 19, 13, 26, 23, 24, 0xFF }, // n  [20]
        {  7, 15,             0xFF }, // rA [21]
        { 14, 16, 17, 23,     0xFF }, // 0  [22]
        { 17, 18, 19, 20,     0xFF }, // sp [23]
        { 23, 20, 26, 25,     0xFF }, // .  [24]
        { 24, 26, 27,         0xFF }, // opt[25]
        { 20, 31, 27, 24, 25, 0xFF }, // m  [26]
        { 26, 32, 34, 25,     0xFF }, // ,  [27]
        {  6, 31, 29,         0xFF }, // i  [28]
        { 28, 32, 30,         0xFF }, // o  [29]
        { 29, 34,             0xFF }, // p  [30]
        { 13, 28, 32, 26,     0xFF }, // k  [31]
        { 31, 29, 33, 27,     0xFF }, // l  [32]
        { 30, 32, 34,         0xFF }, // BS [33]
        { 33, 27,             0xFF }, // En [34]
};

static int keymap_emu[] = {
/*      0				1				2			3			4				5		6 */
/*0*/   KEY_O,			KEY_P,			KEY_K,		KEY_L,		KEY_BACKSPACE,	KEY_Y,	KEY_U, 
/*1*/   KEY_Q,			KEY_W,			KEY_E,		KEY_R,		KEY_T,			KEY_H,	KEY_J,
/*2*/   KEY_A,			KEY_S,			KEY_D,		KEY_F,		KEY_G,			KEY_B,	KEY_N,
/*3*/   KEY_LEFTSHIFT,	KEY_Z,			KEY_X,		KEY_DOT,	KEY_V,			KEY_M,	KEY_COMMA,
/*4*/   KEY_ALT,		KEY_RESERVED,	KEY_SPACE,	KEY_C,		KEY_RESERVED,	KEY_I,	KEY_ENTER
};

static int keymap_evt[] = {
/*      0				1				2			3			4				5				6 */
/*0*/   KEY_Q,			KEY_W,			KEY_E,		KEY_R,		KEY_T,			KEY_Y,			KEY_U, 
/*1*/   KEY_A,			KEY_S,			KEY_D,		KEY_F,		KEY_G,			KEY_H,			KEY_J,
/*2*/   KEY_LEFTSHIFT,	KEY_Z,			KEY_X,		KEY_C,		KEY_V,			KEY_B,			KEY_N,
/*3*/   KEY_RIGHTALT,		KEY_0,			KEY_SPACE,	KEY_DOT,	KEY_ALT,	KEY_M,			KEY_COMMA,
/*4*/   KEY_I,			KEY_O,			KEY_P,		KEY_K,		KEY_L,			KEY_BACKSPACE,	KEY_ENTER
};

static int kp_gpio_rows[] = {31, 32, 33, 34, 35};
static int kp_gpio_cols[] = {42, 41, 40, 39, 38, 37, 36};

static void msm_kp_remux (void)
{
	msm_set_mux("KEYPAD_ROW4", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW3", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW2", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW1", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW0", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL6", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL5", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL4", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL3", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL2", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL1", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL0", CONFIG_ACTIVE);
	printk("%s\n", "gpio_keypad: remuxed keys");
}


static struct gpio_kp_config kp_data = {
	.output_num  = ARRAY_SIZE(kp_gpio_rows),
	.outputs     = kp_gpio_rows,
	.input_num   = ARRAY_SIZE(kp_gpio_cols),
	.inputs      = kp_gpio_cols,
	.keymap_cfg  = GPIO_KEYPAD_CFG_ROW_IS_OUTPUT,
	.keymap      = keymap_evt,
	.rep_period  = 100,   // 100 ms repeat time (10 cps)
	.rep_delay   = 500,   // 500 ms repeat delay
	.gpio_delay  = 5,     // 5  usec to stabilize reading
	.debounce    = 20,    // 20 msec to stabilize reading
	.wakeup_row  = 0,     // row 0 can wakeup device
	.wakeup_mask = 0xFF,   // all buttons on row 0 can wakeup device
	.kp_remux_func = &msm_kp_remux,
	.kp_gpio_print_func = &msm_gpio_dump,
	.key_prox_timeout = 200,
	.key_prox_width = 6,
	.key_prox_map   = board_key_prox_map,

};

static struct platform_device pixie_gpio_keypad_device = {
	.name  = "gpio_keypad",
	.id    = -1,
	.dev   =  {
		.platform_data    = &kp_data,
	},
};
#endif

#ifdef CONFIG_HSUART
#if USE_HSUART_FOR_BT
static int btuart_pin_mux(int on)
{
	int cfg = on ? CONFIG_ACTIVE : CONFIG_SLEEP;

	if (!on) {
		//RTS line will be handled separately
		msm_set_mux("UART1DM_RTS", cfg);
	}

	msm_set_mux("UART1DM_CTS", cfg);
	msm_set_mux("UART1DM_RX",  cfg);
	msm_set_mux("UART1DM_TX",  cfg);

	return 0;
}

static int btuart_deassert_rts(int deassert)
{
	int cfg = deassert ? CONFIG_SLEEP : CONFIG_ACTIVE;

	msm_set_mux("UART1DM_RTS", cfg);

	return 0;
}


/*
 * BT High speed UART interface
 */
static struct hsuart_platform_data btuart_data = {
	.dev_name   = "bt_uart",
	.uart_mode  = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = HSUART_SPEED_115K,
	.options    = HSUART_OPTION_DEFERRED_LOAD | HSUART_OPTION_TX_PIO | HSUART_OPTION_RX_DM ,

	.tx_buf_size = 512,
	.tx_buf_num  = 64,
	.rx_buf_size = 512,
	.rx_buf_num  = 64,	
	.max_packet_size = 450, // ~450
	.min_packet_size = 6,   // min packet size
	.rx_latency      = 10, // in bytes at current speed
	.p_board_pin_mux_cb = btuart_pin_mux,
	.p_board_rts_pin_deassert_cb = btuart_deassert_rts,
//	.rts_pin         = 145,   // uart rts line pin
};

static u64 btuart_dmamask = ~(u32)0;
static struct platform_device btuart_device = {
	.name = "hsuart",
	.id   =  0, // configure UART2 as hi speed uart
	.dev  = {
		.dma_mask           = &btuart_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = &btuart_data,
	}
};
#endif // USE_HSUART_FOR_BT

static int tpuart_pin_mux(int on)
{
	int cfg = on ? CONFIG_ACTIVE : CONFIG_SLEEP;

	msm_set_mux("UART2DM_RX", cfg);

	return 0;
}
/*
 * Cypress (Touch Panel) High speed UART interface
 */
static struct hsuart_platform_data tpuart_data = {
	.dev_name   = "tp_uart",
	.uart_mode  = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = HSUART_SPEED_1228K,
	.options    = HSUART_OPTION_DEFERRED_LOAD | HSUART_OPTION_TX_PIO | HSUART_OPTION_RX_DM ,
	.tx_buf_size = 144, //4*1024,
	.tx_buf_num  = 2,
	.rx_buf_size = (144 * 2) ,//1000, //4*1024,
	.rx_buf_num  = 250, //1000,
	.max_packet_size = 144 * 2, //144,
	.min_packet_size = 74 , // packet that contains IDAC table 
	.rx_latency      = 10, // in bytes at current speed
	.p_board_pin_mux_cb = tpuart_pin_mux,
};

static u64 tpuart_dmamask = ~(u32)0;
static struct platform_device tpuart_device = {
	.name = "hsuart",
	.id   =  1, // configure UART2 as hi speed uart
	.dev  = {
		.dma_mask           = &tpuart_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = &tpuart_data,
	}
};


#endif // Of HSUART

// use this variable to decide if userspace will tell us when to config
// bt pins for power savings or whether we need to do it before sleep
static int bluetooth_on = 1;

/*
 * Bluetooth power state driver
 */
#ifdef CONFIG_BLUETOOTH_POWER_STATE

int bt_power (unsigned int on)
{
	int rc = 0;
	struct vreg *vreg_bt;

	if (on) {
		printk("Powering on BT\n");

		vreg_bt = vreg_get(0, "gp3");
		vreg_set_level(vreg_bt, 2600);
		rc = vreg_enable(vreg_bt);
		if (rc) {
			printk(KERN_ERR "%s: vreg \"BT\" enable failed (%d)\n", __func__, rc);
			return rc;
		}

		msm_set_mux("BT_PCM_OUT", CONFIG_ACTIVE);
		msm_set_mux("BT_PCM_IN", CONFIG_ACTIVE);
		msm_set_mux("BT_PCM_SYNC", CONFIG_ACTIVE);
		msm_set_mux("BT_PCM_CLK", CONFIG_ACTIVE);
		msm_set_mux("BT_RESET_OFF", CONFIG_ACTIVE);
		msm_set_mux("BT_BT_WAKE_HOST_OFF", CONFIG_ACTIVE);
		msm_set_mux("BT_HOST_WAKE_BT_OFF", CONFIG_ACTIVE);

		gpio_set_value(GPIO_BT_RESET, 0);

		bluetooth_on = 1;
	} 
	else {
		printk("Powering off BT\n");

		msm_set_mux("BT_PCM_OUT", CONFIG_SLEEP);
		msm_set_mux("BT_PCM_IN", CONFIG_SLEEP);
		msm_set_mux("BT_PCM_SYNC", CONFIG_SLEEP);
		msm_set_mux("BT_PCM_CLK", CONFIG_SLEEP);
		msm_set_mux("BT_RESET_OFF", CONFIG_SLEEP);
		msm_set_mux("BT_BT_WAKE_HOST_OFF", CONFIG_SLEEP);
		msm_set_mux("BT_HOST_WAKE_BT_OFF", CONFIG_SLEEP);

		vreg_bt = vreg_get(0, "gp3");
		rc = vreg_disable(vreg_bt);
		if (rc)
			printk(KERN_ERR "%s: vreg \"BT\" disable failed (%d)\n", __func__, rc);

		bluetooth_on = 0;
	}
	return rc;
}

static struct bluetooth_power_state_platform_data bluetooth_power_state_platform_data_pixie = {
	.dev_name	= "bt_power",
	.bt_power = bt_power,
};

static struct platform_device bluetooth_power_state_device = {
	.name	= "bt_power",
	.id	= 0,
	.dev	= {
		.platform_data = &bluetooth_power_state_platform_data_pixie
	},
};

#endif // CONFIG_BLUETOOTH_POWER_STATE

/*
 * ISL29018 Proximity Sensor
 */
#ifdef CONFIG_ISL29018_PROX_SENSOR
static struct isl29018_platform_data pixie_isl29018_platform_data = {
	.dev_name	= ISL29018_DEVICE,
	.gpio		= GPIO_ISL29018_INT,

	.threshold = 20,
	.debounce = 1, // 4 times to trigger
	.prox_gain = 2, // 16K lux
	.prox_width = 1, // 12 bit
	.prox_amp = 3, // 100mA 
	.prox_mod_freq = 1, // 360 KHz modulation frequency
	.scan_off_time_no_det = 150, // Scan interval when object is out of range
	.scan_off_time_det = 300, // Scan interval when object is in range
	.als_min_poll_interval = 350, 
	.prox_min_scan_time = 260, // minimum scan time
	.als_gain = 0, // 1000 lux
	.als_width = 0, // 16 bit
};

static struct i2c_board_info isl29018_i2c_board_info = {
	I2C_BOARD_INFO( ISL29018_DEVICE, (0x88>>1)),
	.irq = MSM_GPIO_TO_INT(GPIO_ISL29018_INT),
	.platform_data = &pixie_isl29018_platform_data,
};
#endif // CONFIG_ISL29018_PROX_SENSOR


#ifdef CONFIG_ACCELEROMETER_KXSD9

#define KXSD9_I2C_ADDRESS	0x19

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
/*0*/   0,			-1,			0,
/*1*/   1,			0,			0,
/*2*/   0,			0,			-1,
};

static int kxsd9_xyz_translation_map_evt3 [] = {
/*      0			1			2	*/
/*0*/   -1,			0,			0,
/*1*/   0,			1,			0,
/*2*/   0,			0,			1,
};

static struct kxsd9_platform_data board_kxsd9_platform_data= {
	.dev_name = ACCELEROMETER_DEVICE,
	.xyz_translation_map = kxsd9_xyz_translation_map,
};

static struct i2c_board_info kxsd9_i2c_board_info = {
	I2C_BOARD_INFO( ACCELEROMETER_DEVICE, KXSD9_I2C_ADDRESS),
	.irq = 0,
	.platform_data = &board_kxsd9_platform_data,
};

#endif // CONFIG_ACCELEROMETER_KXSD9

#ifdef CONFIG_MSM_VIBRATOR
static struct platform_device board_vibe_device = {
	.name = VIBE_DEVICE,
	.id   = -1,
	.dev  = {
		.platform_data  = &msm_vibe_platform_data,
	}
};
#endif

#ifdef CONFIG_W1_MASTER_GPIO
static struct platform_device board_1w_gpio_device = {
	.name = ONEWIRE_GPIO_DEVICE,
	.id   = -1,
	.dev  = {
		.platform_data  = &msm_w1_gpio_platform_data,
	}
};
#endif



/*****************************************************************************
*
*				User pins device
*
*****************************************************************************/
#ifdef CONFIG_USER_PINS

/*
 *   Bt Pins
 */
static struct user_pin bt_pins[] = {
	{
		.name       =  "reset",
		.gpio       =  16,
		.act_level  =  0, // active low
		.direction  =  0, // an output
		.def_level  =  0, // default level (low)
		.pin_mode   =  (void *)-1,// undefined
		.sysfs_mask =  0777,
		.irq_handler = NULL,
		.irq_config = 0,
		.irq_handle_mode = IRQ_HANDLE_NONE, 
	},
	{
		.name       =  "wake",
		.gpio       =  22,
		.act_level  =  0, // active Low
		.direction  =  0, // an output
		.def_level  =  0, // default level (low)
		.pin_mode   =  (void *)-1,// undefined
		.sysfs_mask =  0777,
		.irq_handler = NULL,
		.irq_config = 0,
		.irq_handle_mode = IRQ_HANDLE_NONE, 
	},
	{
		.name       =  "host_wake",
		.gpio       =  17,
		.act_level  =  1,  // active high
		.direction  =  1,  // an input
		.def_level  = -1,  // undefined
		.pin_mode   =  (void *)-1, // undefined
		.sysfs_mask =  0777,
		.options    =  PIN_WAKEUP_SOURCE,
		.irq_handler = NULL, 
		.irq_config = IRQF_TRIGGER_RISING,
		.irq_handle_mode = IRQ_HANDLE_AUTO, 
	},
};

static struct user_pin_set  board_user_pins_sets[] = {
	{
		.set_name = "bt",
		.num_pins = ARRAY_SIZE(bt_pins),
		.pins     = bt_pins,
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
/*********************End of User-pins device*******************************/

/*
 * CY8C24894 TP PSoC
 */
#ifdef CONFIG_CY8C24894_TP

void remux_i2c_mfp(bool gpio)
{
	if (gpio) {
		// re-mux i2c lines as gpio outputs...
		msm_set_mux("I2C_SDA_MUX_GPIO", CONFIG_ACTIVE);
		msm_set_mux("I2C_SCL_MUX_GPIO", CONFIG_ACTIVE);
	}
	else {
		msm_set_mux("I2C_SCL", CONFIG_ACTIVE);
		msm_set_mux("I2C_SDA", CONFIG_ACTIVE);
	}

	return;
}

int i2c_debus_master(void)
{
	int rc;

	remux_i2c_mfp(true);
	rc = gpio_direction_output(GPIO_I2C_SDA, 1);
	if (rc)
		goto exit0;
	udelay(5);
	rc = gpio_direction_output(GPIO_I2C_SCL, 1);
	if (rc)
		goto exit0;
	udelay(5);

exit0:
		return rc;
}

int i2c_embus_master(void)
{
	remux_i2c_mfp(false);
	udelay(10);

	return 0;
}

static struct cy8c24894_platform_data pixie_cy8c24894_platform_data = {
		.dev_name	= CY8C24894_DEVICE,
		.wake_tp_gpio = GPIO_CY8C24894_ACTIVE_SCAN,
		.wot_gpio = GPIO_CY8C24894_WOT,
		.pwr_gpio = GPIO_CY8C24894_PWR,
		.lvl_shift_gpio = 0,
		.lvl_shift_pu_fix = 0,
		.poweroff_mode = CY8C2489_DEEP_SLEEP,
		.debus = i2c_debus_master,
		.embus = i2c_embus_master,
};


static struct i2c_board_info cy8c24894_i2c_board_info = {
		I2C_BOARD_INFO( CY8C24894_DEVICE, (0x22>>1)),
		.irq = MSM_GPIO_TO_INT(GPIO_CY8C24894_WOT),
		.platform_data = &pixie_cy8c24894_platform_data,
};
#endif // CONFIG_CY8C24894_TP

#ifdef CONFIG_HRES_COUNTER
static int msm_hres_timer_init(void** timer)
{

	return 0;
}

static int msm_hres_timer_release(void* timer)
{

	return 0;
}

#ifdef CONFIG_PM
static int msm_hres_timer_suspend(void *timer)
{
	return 0;
}

static int msm_hres_timer_resume(void *timer)
{
	return 0;
}
#else
#define msm_hres_timer_suspend    NULL
#define msm_hres_timer_resume     NULL
#endif

extern u32 msm_dgt_read_count(void);
static u32 msm_hres_timer_read(void* timer)
{
	return msm_dgt_read_count();
}

static u32 msm_hres_timer_convert(u32 count)
{
	// Count is in 19.2Mhz terms
	// convert it to uSec
	return (count * 10) / 192;
}

static struct hres_counter_platform_data msm_hres_counter_platform_data = {
	.init_timer = msm_hres_timer_init,
	.release_timer = msm_hres_timer_release,
	.suspend_timer = msm_hres_timer_suspend,
	.resume_timer = msm_hres_timer_resume,

	.read_timer = msm_hres_timer_read,
	.convert_timer = msm_hres_timer_convert,
};

static struct platform_device hres_counter_device = {
	.name = "hres_counter",
	.id   = -1,
	.dev  = {
		.platform_data  = &msm_hres_counter_platform_data,
	}
};
#endif

#ifdef CONFIG_PALM_NDUID
extern unsigned int msm_nduid_get_device_salt(void);
extern int msm_nduid_get_cpu_id(char *id, unsigned int maxlen);

static struct nduid_config nduid_cfg[] = {
	{
		.dev_name = "nduid",
		.get_device_salt = msm_nduid_get_device_salt,
		.get_cpu_id = msm_nduid_get_cpu_id,
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


static struct platform_device msm_adm_device = {
	.name = "msm_adm",
	.id = -1,
	.dev  = {
		.platform_data = &android_pmem_fb_pdata,
	},
};

static struct msm_v4l2_pd msm_v4l2_device_data = {
	.screen_width = 320,
	.screen_height = 400,
	.max_internal_bufs = 3,
	.scaling_factor = 4,
};

static struct platform_device msm_v4l2_device = {
	.name	= "msmv4l2_pd",
	.id	= -1,
	.dev	= {
		.platform_data = &msm_v4l2_device_data
	},
};

static struct platform_device *w1gpiodevices[] __initdata = {
#ifdef CONFIG_W1_MASTER_GPIO
	&board_1w_gpio_device
#endif

};

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(HEADSET, 2),
	SND(SPEAKER, 6),
	SND(BT, 12),
	SND(CURRENT, 25),
};

static struct msm_snd_endpoints snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_snd_device = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &snd_endpoints
	},
};

static struct platform_device *devices[] __initdata = {
	&msm_serial2_device,
	&msm_hsusb_device,
	&android_pmem_adsp_device,
#if defined(CONFIG_MSM_KGSL) && !defined(CONFIG_MSM_KGSL_MMU)
	&android_pmem_gpu1_device,
#endif
	&msm_adm_device,
	&msm_v4l2_device,

#ifdef CONFIG_KEYBOARD_GPIO_PE
	&pixie_gpio_keys_device,
#endif

#ifdef CONFIG_MSM_UARTDM
 #if USE_HSUART_FOR_BT
	&msm_uart1dm_device,
 #endif
	&msm_uart2dm_device,
#endif // Of CONFIG_MSM_UARTDM

#ifdef CONFIG_HSUART
 #if USE_HSUART_FOR_BT
	&btuart_device,
 #else
	&msm_serial0_device,
 #endif
	&tpuart_device,
#endif // Of HSUART

#ifdef CONFIG_USER_PINS
	&board_user_pins_device,
#endif // CONFIG_USER_PINS
#ifdef CONFIG_MSM_VIBRATOR
	&board_vibe_device,
#endif	

#ifdef CONFIG_HRES_COUNTER
	&hres_counter_device,
#endif

#ifdef CONFIG_PALM_NDUID
	&nduid_device,
#endif
#ifdef CONFIG_MSM_CAMERA
	&msm_camera_device,
#endif

	&msm_snd_device,

#ifdef CONFIG_MSM_KGSL
	&msm_device_kgsl,
#endif
};


#ifdef CONFIG_LEDS_LP8501 

/* Divides LED into group:
 * group 1: LCD
 * group 2: Keypad
 * group 3: Core-Navi-Left
 * group 4: Core-Navi-Center
 * group 5: Core-Navi-Right
 */

static struct led_cfg lcd_group[] = {
	[0] = {
		.type = WHITE,
		.pwm_addr = D1_PWM,
		.current_addr = D1_CURRENT_CTRL,
		.control_addr = D1_CONTROL,
	},
	[1] = {
		.type = WHITE,
		.pwm_addr = D2_PWM,
		.current_addr = D2_CURRENT_CTRL,
		.control_addr = D2_CONTROL,
	},
	[2] = {
		.type = WHITE,
		.pwm_addr = D3_PWM,
		.current_addr = D3_CURRENT_CTRL,
		.control_addr = D3_CONTROL,
	},
	[3] = {
		.type = WHITE,
		.pwm_addr = D4_PWM,
		.current_addr = D4_CURRENT_CTRL,
		.control_addr = D4_CONTROL,
	},
};

static struct led_cfg keypad_group_evt1[] = {
	[0] = {
		.type = WHITE,
		.pwm_addr = D5_PWM,
		.current_addr = D5_CURRENT_CTRL,
		.control_addr = D5_CONTROL,
	},
};

static struct led_cfg core_navi_left_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D8_PWM,
		.current_addr = D8_CURRENT_CTRL,
		.control_addr = D8_CONTROL,
	},
};

static struct led_cfg core_navi_center_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D7_PWM,
		.current_addr = D7_CURRENT_CTRL,
		.control_addr = D7_CONTROL,
	},
};

static struct led_cfg core_navi_right_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D9_PWM,
		.current_addr = D9_CURRENT_CTRL,
		.control_addr = D9_CONTROL,
	},
};

static struct lp8501_led_config led_lp8501_data_evt1[] = {
	[GRP_1] = {
		.cdev = {
			.name = "lcd",
		},
		.led_list = &lcd_group[0],
		.nleds = ARRAY_SIZE(lcd_group),
		.group_id = GRP_1,
		.hw_group = HW_GRP_1,
		.default_current = 0xDC, //22.0mA
		.default_brightness = 40,
		.default_state = LED_ON,
	},
	[GRP_2] = {
		.cdev = {
			.name = "keypad",
		},
		.led_list = &keypad_group_evt1[0],
		.nleds = ARRAY_SIZE(keypad_group_evt1),
		.group_id = GRP_2,
		.hw_group = HW_GRP_2,
		.default_current  = 0x32, //5mA
		.default_brightness = 100,
		.default_state = LED_ON,
	},
	[GRP_3] = {
		.cdev = {
			.name = "core_navi_left",
		},
		.led_list = &core_navi_left_group[0],
		.nleds = ARRAY_SIZE(core_navi_left_group),
		.group_id = GRP_3,
		.hw_group = HW_GRP_NONE,
		.default_current  = 0x32, //5mA
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
	[GRP_4] = {
		.cdev = {
			.name = "core_navi_center",
		},
		.led_list = &core_navi_center_group[0],
		.nleds = ARRAY_SIZE(core_navi_center_group),
		.group_id = GRP_4,
		.hw_group = HW_GRP_NONE,
		.default_current  = 0x32, //5mA
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
	[GRP_5] = {
		.cdev = {
			.name = "core_navi_right",
		},
		.led_list = &core_navi_right_group[0],
		.nleds = ARRAY_SIZE(core_navi_right_group),
		.group_id = GRP_5,
		.hw_group = HW_GRP_NONE,
		.default_current  = 0x32, //5mA
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

static struct lp8501_platform_data board_lp8501_data_evt1 = {
	.leds = led_lp8501_data_evt1,
	.memcfg = &led_lp8501_memcfg,
	.nleds = ARRAY_SIZE(led_lp8501_data_evt1),
	.cp_mode = CONFIG_CPMODE_1x,
	.power_mode = CONFIG_POWER_SAVE_ON,
	.dev_name = "national_led",
};

static struct i2c_board_info lp8501_i2c_board_info_evt1 = {
	I2C_BOARD_INFO(LP8501_I2C_DEVICE, LP8501_I2C_ADDR),
	.platform_data = &board_lp8501_data_evt1,
	.irq = MSM_GPIO_TO_INT(GPIO_LP8501_INT),
};
#endif // CONFIG_LEDS_LP8501

#ifdef CONFIG_THERMISTOR_TMP105

static struct i2c_board_info tmp105_i2c_board_info = {
	I2C_BOARD_INFO(TMP105_I2C_DEVICE, TMP105_I2C_ADDR),
	.platform_data = NULL,
 	.irq = 0,
};

#endif // CONFIG_THERMISTOR_TMP105

#ifdef CONFIG_CHARGER_SMB339

static struct i2c_board_info smb339_i2c_board_info = {
	I2C_BOARD_INFO(SMB339_I2C_DEVICE, SMB339_I2C_ADDR),
	.platform_data = NULL,
 	.irq = 0,
};

#endif // CONFIG_CHARGER_SMB339


extern struct sys_timer msm_timer;

static void __init chuck_init_irq(void)
{
	msm_init_irq();
}

static struct clk chuck_clocks[] = {
	CLOCK("adm_clk",	ADM_CLK,	NULL, 0),
	CLOCK("adsp_clk",	ADSP_CLK,	NULL, OFF),
	CLOCK("ebi1_clk",	EBI1_CLK,	NULL, 0),
	CLOCK("ebi2_clk",	EBI2_CLK,	NULL, 0),
	CLOCK("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLOCK("gp_clk",		GP_CLK,		NULL, 0),
	CLOCK("grp_clk",	GRP_CLK,	NULL, OFF),
	CLOCK("i2c_clk",	I2C_CLK,	NULL, 0),
	CLOCK("icodec_rx_clk",	ICODEC_RX_CLK,	NULL, 0),
	CLOCK("icodec_tx_clk",	ICODEC_TX_CLK,	NULL, 0),
	CLOCK("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLOCK("mdc_clk",	MDC_CLK,	NULL, 0),
	CLOCK("mdp_clk",	MDP_CLK,	NULL, OFF),
	CLOCK("mdp_lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, NULL, OFF),
	CLOCK("mdp_lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, NULL, OFF),
	CLOCK("mdp_vsync_clk",  MDP_VSYNC_CLK,  NULL, OFF),
	CLOCK("pbus_clk",	PBUS_CLK,	NULL, 0),
	CLOCK("pcm_clk",	PCM_CLK,	NULL, 0),
	CLOCK("pmdh_clk",	PMDH_CLK,	NULL, OFF),
	CLOCK("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLOCK("sdc1_clk",	SDC1_CLK,	NULL, OFF),
	CLOCK("sdc1_pclk",	SDC1_PCLK,	NULL, OFF),
	CLOCK("sdc2_clk",	SDC2_CLK,	NULL, OFF),
	CLOCK("sdc2_pclk",	SDC2_PCLK,	NULL, OFF),
	CLOCK("sdc3_clk",	SDC3_CLK,	NULL, OFF),
	CLOCK("sdc3_pclk",	SDC3_PCLK,	NULL, OFF),
	CLOCK("sdc4_clk",	SDC4_CLK,	NULL, OFF),
	CLOCK("sdc5_pclk",	SDC4_PCLK,	NULL, OFF),
	CLOCK("uart1_clk",	UART1_CLK,	NULL, OFF),
	CLOCK("uart2_clk",	UART2_CLK,	NULL, 0),
	CLOCK("uart3_clk",	UART3_CLK,	NULL, OFF),
	CLOCK("uart1dm_clk",	UART1DM_CLK,	NULL, OFF),
	CLOCK("uart2dm_clk",	UART2DM_CLK,	NULL, 0),
	CLOCK("usb_hs_clk",	USB_HS_CLK,	NULL, OFF),
	CLOCK("usb_hs_pclk",	USB_HS_PCLK,	NULL, OFF),
	CLOCK("usb_otg_clk",	USB_OTG_CLK,	NULL, 0),
	CLOCK("vdc_clk",	VDC_CLK,	NULL, OFF),
	CLOCK("vfe_clk",	VFE_CLK,	NULL, OFF),
	CLOCK("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, OFF),
	CLOCK("grp_pclk",	GRP_PCLK, 	NULL, OFF),
};
unsigned msm_num_chuck_clocks = ARRAY_SIZE(chuck_clocks);

static struct msm_acpu_clock_platform_data chuck_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
	.max_axi_khz = 200000000,
};


/* saved host pointers */
static struct mmc_host *g_host[4] = {0};

static void
board_save_mmc_ptr ( struct device *dev, struct mmc_host *mmc  )
{
	int slot;
	struct platform_device *pdev = 
		container_of (dev, struct platform_device, dev );

	slot = pdev->id-1;
	if( slot >= 0 && slot < ARRAY_SIZE(g_host)) {
		g_host[slot] = mmc;
	}
	return ;
}

static int
board_rescan_slot ( int slot )
{
	if( slot < 0 || slot >= ARRAY_SIZE(g_host)) 
		return -EINVAL;

	if( g_host[slot] == NULL )
		return -ENODEV;

	mmc_detect_change ( g_host[slot], 0 );
		
	return 0;
}

static int
board_rescan_dev ( struct device *dev )
{
	struct platform_device *pdev = 
		container_of (dev, struct platform_device, dev );

	return board_rescan_slot ( pdev->id-1 );
}

/*
 *  An API to trigger mmc slot rescan
 */
int 
board_rescan_mmc_slot ( void )
{
	return board_rescan_slot (MMC_DEV_ID_MMC - 1);
}
EXPORT_SYMBOL(board_rescan_mmc_slot);

static int 
board_rescan_wifi_slot ( void )
{
	return board_rescan_slot (MMC_DEV_ID_WIFI - 1);
}

/*
 *  An API to enable wifi
 */
int board_sdio_wifi_enable(unsigned int param)
{
	printk(KERN_ERR "board_sdio_wifi_enable\n");
	//Assuming that WiFi is off - in this case re-scan mechanism will power up the card
	return board_rescan_wifi_slot();
}
EXPORT_SYMBOL(board_sdio_wifi_enable);

/*
 *  An API to disable wifi
 */
int board_sdio_wifi_disable(unsigned int param)
{
	printk(KERN_ERR "board_sdio_wifi_disable\n");
	//Put WiFi chip in RESET in order for the detection to fail and remove the card
	gpio_set_value(GPIO_SDIO_RESET,0);

	return board_rescan_wifi_slot();
}
EXPORT_SYMBOL(board_sdio_wifi_disable);



/* 
 * sysfs function to trigger card detect
 */
ssize_t 
board_mmc_cd_store( struct device *dev, 
                    struct device_attribute *attr, 
                    const char *buf, size_t count)
{
	int i = 0;

	/* skip leading white spaces */
	while( i < count && isspace(buf[i])) {
		i++;
	}
	if( count - i >= 6) {
		if( strncmp( buf+i, "Rescan", 6) == 0) {
			// Initiate slot rescan
			board_rescan_dev ( dev );
			return count;    
		} 
	}
	return count;
}
DEVICE_ATTR(mmc_card_detect, S_IWUSR, NULL, board_mmc_cd_store );

static int 
board_cd_init( struct mmc_host *mmc )
{
	if( device_create_file( mmc_dev(mmc), &dev_attr_mmc_card_detect) < 0) {
		dev_err( mmc_dev(mmc), 
		        "unable to create card detect sysfs attribute\n");
	}
	board_save_mmc_ptr ( mmc_dev(mmc), mmc );
	
	return 0;
}

static int
board_cd_free( struct mmc_host *mmc )
{
	board_save_mmc_ptr ( mmc_dev(mmc), NULL );
	device_remove_file ( mmc_dev(mmc), &dev_attr_mmc_card_detect);
	return 0;
}

/* 
 *  Driver callbacks
 */
static int 
board_mmc_probe ( void *mmc )
{
	return board_cd_init ((struct mmc_host *) mmc );
}

static int 
board_mmc_remove ( void *mmc )
{
	return board_cd_free ((struct mmc_host *) mmc );
}

static int 
board_mmc_suspend ( void *mmc )
{
	return 0; 
}

static int 
board_mmc_resume ( void *mmc )
{
	return 0; 
}


static unsigned long vreg_sts = 0, gpio_sts = 0;
static struct vreg *vreg_mmc;

#define MMC_DEV_IS_VREG_ON(devid)  (vreg_sts & (1 << devid ))
#define MMC_DEV_IS_VREG_OFF(devid) !MMC_DEV_IS_VREG_ON(devid)

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);

	switch(dev_id)
	{
		// wifi
		case MMC_DEV_ID_WIFI:
		{
			msm_set_mux("WL_SD_CLK", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("WL_SD_CMD", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("WL_SD_DATA3", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("WL_SD_DATA2", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("WL_SD_DATA1", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("WL_SD_DATA0", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			break;
		}
		// mmc
		case MMC_DEV_ID_MMC:
		{
			msm_set_mux("MMC_SD_CLK", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("MMC_SD_CMD", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("MMC_SD_DATA3", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("MMC_SD_DATA2", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("MMC_SD_DATA1", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			msm_set_mux("MMC_SD_DATA0", enable ? CONFIG_ACTIVE : CONFIG_SLEEP);
			break;
		}
		default:
			break;
	}
}

static void  board_power_wifi(bool on)
{
	struct vreg *vreg_wifi, *vreg_wifi_io;
	int rc;

	if(on)
	{
	#ifdef CONFIG_HIGH_RES_TIMERS
		struct timespec ts;
	#endif

		msm_set_mux("SDIO_RESET", CONFIG_ACTIVE);
		msm_set_mux("WIFI_EN", CONFIG_ACTIVE);

		gpio_set_value(GPIO_SDIO_RESET,0);
		gpio_set_value(GPIO_WIFI_EN, 1);		

		vreg_wifi_io = vreg_get(0, "synt");
		vreg_set_level(vreg_wifi_io, 3000);
		rc = vreg_enable(vreg_wifi_io);
		if (rc)
			printk(KERN_ERR "%s: vreg \"wifi-io\" enable failed (%d)\n", __func__, rc);

		if(board_type <= EVT2)
		{
			vreg_wifi = vreg_get(0, "gp6");
			vreg_set_level(vreg_wifi, 1800);
			rc = vreg_enable(vreg_wifi);
			if (rc)
				printk(KERN_ERR "%s: vreg \"wifi\" enable failed (%d)\n", __func__, rc);
		}

		//Wait for 200us for the 1.8 v power supply to settle plus 520us For Marvel to stabilize
		//Making it 1 msec to be on the safe side
#ifdef CONFIG_HIGH_RES_TIMERS
		ts.tv_sec  = 0;
		ts.tv_nsec = 1000000L;
		hrtimer_nanosleep( &ts, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
#else
		msleep(1);
#endif

		gpio_set_value(GPIO_SDIO_RESET,1);

		msm_set_mux("SDIO_INT", CONFIG_ACTIVE);
	}
	else
	{
		gpio_set_value(GPIO_SDIO_RESET,0);
		gpio_set_value(GPIO_WIFI_EN, 0);

		vreg_wifi_io = vreg_get(0, "synt");
		rc = vreg_disable(vreg_wifi_io);
		if (rc)
			printk(KERN_ERR "%s: vreg \"wifi-io\" disable failed (%d)\n", __func__, rc);

		if(board_type <= EVT2)
		{
			vreg_wifi = vreg_get(0, "gp6");
			rc = vreg_disable(vreg_wifi);
			if (rc)
				printk(KERN_ERR "%s: vreg \"wifi\" disable failed (%d)\n", __func__, rc);
		}

		msm_set_mux("SDIO_INT", CONFIG_SLEEP);
		msm_set_mux("SDIO_RESET", CONFIG_SLEEP);
		msm_set_mux("WIFI_EN", CONFIG_SLEEP);
	}

}

static uint32_t msm_sdcc_get_powersave_mode(struct device *dv)
{
	return 1;
}

/* 
*  Indicates whether the host should initiate card detection
*  after start
*     0 - initiate the detection
*     1 - do not initiate the detection
*/
static uint32_t msm_sdcc_get_detection_on_start_mode(struct device *dv)
{
	struct platform_device *pdev;
	uint32_t ret = 0;

	pdev = container_of(dv, struct platform_device, dev);

	//Do not initiate card detection for WiFi
	if ( pdev->id == MMC_DEV_ID_WIFI ) {
		ret = 1;
	}

	return ret;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;
	int dev_id = 0;

	pdev = container_of(dv, struct platform_device, dev);
	dev_id      = pdev->id;

	//printk(KERN_INFO "%s: vdd %d; vreg_sts:%lu id: %d\n", __func__, vdd, vreg_sts, pdev->id);

	if (vdd == 0) {

		msm_sdcc_setup_gpio(dev_id, !!vdd);

		if (!vreg_sts)
			return 0;

		if(dev_id == MMC_DEV_ID_WIFI && MMC_DEV_IS_VREG_ON(dev_id) ) {
			printk(KERN_INFO "%s: Powering off wifi vreg\n", __func__);
			board_power_wifi(false);
		}


		if (dev_id == MMC_DEV_ID_MMC && MMC_DEV_IS_VREG_ON(dev_id)) {
			rc = vreg_disable(vreg_mmc);
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}

		clear_bit(dev_id, &vreg_sts);

		return 0;
	}
	else
	{
		if (dev_id == MMC_DEV_ID_MMC && MMC_DEV_IS_VREG_OFF(dev_id)) {
			rc = vreg_set_level(vreg_mmc, 2850);
			if (!rc)
				rc = vreg_enable(vreg_mmc);
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}

		if(dev_id == MMC_DEV_ID_WIFI && MMC_DEV_IS_VREG_OFF(dev_id)) {
			printk(KERN_INFO "%s: Powering on wifi vreg\n", __func__);
			board_power_wifi(true);
		}
		
		set_bit(dev_id, &vreg_sts);

		msm_sdcc_setup_gpio(dev_id, !!vdd);
	}


	return 0;
}

static unsigned int chuck_sdcc_slot_status(struct device *dev)
{
	/* Device is always connected */
	return 1;
}

static struct msm_mmc_platform_data chuck_sdcc_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.status		= chuck_sdcc_slot_status,
	.board_probe 	= board_mmc_probe,
	.board_remove	= board_mmc_remove,
	.board_suspend	= board_mmc_suspend,
	.board_resume 	= board_mmc_resume,
	.translate_vdd	= msm_sdcc_setup_power,
	.get_powersave_mode = msm_sdcc_get_powersave_mode,
	.get_detection_on_start_mode = msm_sdcc_get_detection_on_start_mode,
};

void msm_serial_debug_init(unsigned int base, int irq, 
			   const char *clkname, int signal_irq);

static void __init chuck_init_mmc(void)
{
	vreg_mmc = vreg_get(0, "wlan");

	msm_add_sdcc(MMC_DEV_ID_MMC, &chuck_sdcc_data);
	
	//Configure Wifi reset pin 
	gpio_request (GPIO_SDIO_RESET, "wifi reset");
	gpio_direction_output (GPIO_SDIO_RESET, 0);

	//Register Wifi device
	msm_add_sdcc(MMC_DEV_ID_WIFI, &chuck_sdcc_data);
}

static void __init chuck_init_tp(void)
{
	int rc = 0;
	
	// request gpio for psoc wakeup (host -> psoc)...
	rc = gpio_request(GPIO_CY8C24894_ACTIVE_SCAN, "cy8c24894_as");
	if (rc) {
		printk(KERN_ERR "%s: Failed to request gpio for wakeup line.\n", __func__);
		return; 
	}
	gpio_direction_output(GPIO_CY8C24894_ACTIVE_SCAN, 0);
	
	
	// request gpio for psoc wake-on-touch (psoc -> host)...
	rc = gpio_request(GPIO_CY8C24894_WOT, "cy8c24894_wot");
	if (rc) {
		printk(KERN_ERR "%s: Failed to request gpio for wot line.\n", __func__);
		return; 
	}
	gpio_direction_input(GPIO_CY8C24894_WOT);
	printk(KERN_ERR "%s: success!\n", __func__);
	
	// request gpio for psoc pwr (host -> psoc)...
	rc = gpio_request(GPIO_CY8C24894_PWR, "cy8c24894_pwr");
	if (rc) {
		printk(KERN_ERR "%s: Failed to request gpio for pwr line.\n", __func__);
		return; 
	}

	if (board_type >= DVT) {
		rc = gpio_request(GPIO_CY8C24894_LVL_SHIFT_PWR, "cy8c24894_lvl_shift");
		if (rc) {
			printk(KERN_ERR "%s: Failed to request gpio for level shifter.\n", __func__);
			return;
		}

		pixie_cy8c24894_platform_data.lvl_shift_gpio = GPIO_CY8C24894_LVL_SHIFT_PWR;
		pixie_cy8c24894_platform_data.poweroff_mode = CY8C2489_POWER_OFF_PSOC;
	}

	if (board_type >= PVT) {
		pixie_cy8c24894_platform_data.lvl_shift_pu_fix = 1;
	}

}

#ifdef CONFIG_MSM_CAMERA
static struct i2c_board_info ov2650_i2c_board_info = {
	I2C_BOARD_INFO("ov265x", 0x60 >> 1),
};

void config_camera_on_gpios(void)
{
	msm_set_mux("CAM_DATA2", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA3", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA4", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA5", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA6", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA7", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA8", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA9", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA10", CONFIG_ACTIVE);
	msm_set_mux("CAM_DATA11", CONFIG_ACTIVE);
	msm_set_mux("CAM_PCLK", CONFIG_ACTIVE);
	msm_set_mux("CAM_HSYNC", CONFIG_ACTIVE);
	msm_set_mux("CAM_VSYNC", CONFIG_ACTIVE);
	msm_set_mux("CAM_MCLK", CONFIG_ACTIVE);
}

void config_camera_off_gpios(void)
{
	msm_set_mux("CAM_DATA2", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA3", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA4", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA5", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA6", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA7", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA8", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA9", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA10", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA11", CONFIG_SLEEP);
	msm_set_mux("CAM_PCLK", CONFIG_SLEEP);
	msm_set_mux("CAM_HSYNC", CONFIG_SLEEP);
	msm_set_mux("CAM_VSYNC", CONFIG_SLEEP);
	msm_set_mux("CAM_MCLK", CONFIG_SLEEP);
}
#endif // CONFIG_MSM_CAMERA

#ifdef CONFIG_PROFILING
static inline void init_perf(void)
{
	u32 data;
	asm volatile("mrc p15, 0, %0, c15, c12, 0" : "=r" (data));
	data = data & (~1);
	/* upper 4bits and 7, 11 are write-as-0 */
	data &= 0x0ffff77f;
	asm volatile("mcr p15, 0, %0, c15, c12, 0" : : "r" (data));

	data = 0;
	asm volatile("mcr p15, 0, %0, c15, c12, 1" : : "r" (data));
}
#endif //CONFIG_PROFILING

static struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};

void chuck_mux_sleep_config(void)
{
	struct mpp *cam_lvl_shifter;

	if((rat_type == UMTS) && (board_type >= DVT2)) {
		cam_lvl_shifter = mpp_get(NULL, "mpp3");
		if(cam_lvl_shifter) {
			mpp_config_digital_out(cam_lvl_shifter,
					MPP_CFG(MPP_DLOGIC_LVL_MSMP,
					MPP_DLOGIC_OUT_CTRL_LOW));
		}
	}

	msm_set_mux("CAM_DATA2", CONFIG_SLEEP);			// camera
	msm_set_mux("CAM_DATA3", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA4", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA5", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA6", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA7", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA8", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA9", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA10", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA11", CONFIG_SLEEP);
	msm_set_mux("CAM_PCLK", CONFIG_SLEEP);
	msm_set_mux("CAM_HSYNC", CONFIG_SLEEP);
	msm_set_mux("CAM_VSYNC", CONFIG_SLEEP);
	msm_set_mux("CAM_MCLK", CONFIG_SLEEP);

	msm_set_mux("KEYPAD_ROW4", CONFIG_SLEEP);		//keypad
	msm_set_mux("KEYPAD_ROW3", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_ROW2", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_ROW1", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_ROW0", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_COL6", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_COL5", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_COL4", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_COL3", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_COL2", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_COL1", CONFIG_SLEEP);
	msm_set_mux("KEYPAD_COL0", CONFIG_SLEEP);

	msm_set_mux("I2C_SCL", CONFIG_SLEEP);			// i2c
	msm_set_mux("I2C_SDA", CONFIG_SLEEP);

	msm_set_mux("UART3_RX", CONFIG_SLEEP);			// uart3
	msm_set_mux("UART3_TX", CONFIG_SLEEP);

#ifdef CONFIG_MSM_PWM
	msm_set_mux("GP_MN_OUT", CONFIG_SLEEP);			// pwm clock
#endif
	msm_set_mux("VIBRATOR_EN", CONFIG_SLEEP);

	if(bluetooth_on) {
		msm_set_mux("BT_PCM_OUT", CONFIG_SLEEP);	// bluetooth pcm
		msm_set_mux("BT_PCM_IN", CONFIG_SLEEP);
		msm_set_mux("BT_PCM_SYNC", CONFIG_SLEEP);
		msm_set_mux("BT_PCM_CLK", CONFIG_SLEEP);
		msm_set_mux("BT_RESET", CONFIG_SLEEP);
		msm_set_mux("BT_BT_WAKE_HOST", CONFIG_SLEEP);
		msm_set_mux("BT_HOST_WAKE_BT", CONFIG_SLEEP);
	}

	msm_set_mux("GPIO_VOL_DOWN", CONFIG_SLEEP);		// switches
	msm_set_mux("GPIO_VOL_UP", CONFIG_SLEEP);
	msm_set_mux("GPIO_RINGER", CONFIG_SLEEP);

	msm_set_mux("LP8501_INT", CONFIG_SLEEP);		// led controller
	msm_set_mux("LP8501_EN", CONFIG_SLEEP);

	msm_set_mux("GPIO_PROX_INT", CONFIG_SLEEP);		// prox

	if(msm_mddi_lg_nt35380_pdata.vsync_gpio)
		msm_set_mux("GPIO_VSYNC", CONFIG_SLEEP);
	else
		msm_set_mux("GPIO_VSYNC_BAD_PANEL", CONFIG_SLEEP);


	if(board_type >= DVT) {
		msm_set_mux("CY8C24894_WOT", CONFIG_SLEEP);		//touchpanel
		msm_set_mux("CY8C24894_ACTIVE_SCAN", CONFIG_SLEEP);
		msm_set_mux("CY8C24894_PWR_DVT", CONFIG_SLEEP);
		msm_set_mux("CY8C24894_LVL_SHIFT", CONFIG_SLEEP);
	}

}

void chuck_mux_wake_config(void)
{
	struct mpp *cam_lvl_shifter;

	if((rat_type == UMTS) && (board_type >= DVT2)) {
		cam_lvl_shifter = mpp_get(NULL, "mpp3");
		if(cam_lvl_shifter) {
			mpp_config_digital_out(cam_lvl_shifter,
					MPP_CFG(MPP_DLOGIC_LVL_MSMP,
					MPP_DLOGIC_OUT_CTRL_HIGH));
		}
	}

	msm_set_mux("KEYPAD_ROW4", CONFIG_ACTIVE);		//keypad
	msm_set_mux("KEYPAD_ROW3", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW2", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW1", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW0", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL6", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL5", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL4", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL3", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL2", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL1", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL0", CONFIG_ACTIVE);

	msm_set_mux("I2C_SCL", CONFIG_ACTIVE);			// i2c
	msm_set_mux("I2C_SDA", CONFIG_ACTIVE);

	msm_set_mux("UART3_RX", CONFIG_ACTIVE);			// uart3
	msm_set_mux("UART3_TX", CONFIG_ACTIVE);

#ifdef CONFIG_MSM_PWM
	msm_set_mux("GP_MN_OUT", CONFIG_ACTIVE);		// pwm clock
#endif
	msm_set_mux("VIBRATOR_EN", CONFIG_ACTIVE);

	if(bluetooth_on) {
		msm_set_mux("BT_PCM_OUT", CONFIG_ACTIVE);	// bluetooth pcm
		msm_set_mux("BT_PCM_IN", CONFIG_ACTIVE);
		msm_set_mux("BT_PCM_SYNC", CONFIG_ACTIVE);
		msm_set_mux("BT_PCM_CLK", CONFIG_ACTIVE);
		msm_set_mux("BT_RESET", CONFIG_ACTIVE);
		msm_set_mux("BT_BT_WAKE_HOST", CONFIG_ACTIVE);
		msm_set_mux("BT_HOST_WAKE_BT", CONFIG_ACTIVE);
	}

	msm_set_mux("GPIO_VOL_DOWN", CONFIG_ACTIVE);		// switches
	msm_set_mux("GPIO_VOL_UP", CONFIG_ACTIVE);
	msm_set_mux("GPIO_RINGER", CONFIG_ACTIVE);

	msm_set_mux("LP8501_INT", CONFIG_ACTIVE);		// led controller
	msm_set_mux("LP8501_EN", CONFIG_ACTIVE);

	msm_set_mux("GPIO_PROX_INT", CONFIG_ACTIVE);		// prox

	if(msm_mddi_lg_nt35380_pdata.vsync_gpio)
		msm_set_mux("GPIO_VSYNC", CONFIG_ACTIVE);
	else
		msm_set_mux("GPIO_VSYNC_BAD_PANEL", CONFIG_ACTIVE);


	if(board_type >= DVT)					// touchpanel
	{
		msm_set_mux("CY8C24894_WOT", CONFIG_ACTIVE);
		msm_set_mux("CY8C24894_ACTIVE_SCAN", CONFIG_ACTIVE);
		msm_set_mux("CY8C24894_PWR_DVT", CONFIG_ACTIVE);
		msm_set_mux("CY8C24894_LVL_SHIFT", CONFIG_ACTIVE);
	}
}

static void chuck_mux_init_config(void)
{
	struct mpp *cam_lvl_shifter;

	if((rat_type == UMTS) && (board_type >= DVT2)) {
		cam_lvl_shifter = mpp_get(NULL, "mpp3");
		if(cam_lvl_shifter) {
			mpp_config_digital_out(cam_lvl_shifter,
					MPP_CFG(MPP_DLOGIC_LVL_MSMP,
					MPP_DLOGIC_OUT_CTRL_HIGH));
		}
	}

	msm_set_mux("I2C_SCL", CONFIG_ACTIVE);
	msm_set_mux("I2C_SDA", CONFIG_ACTIVE);

	msm_set_mux("CY8C24894_ACTIVE_SCAN", CONFIG_ACTIVE);	// touchpanel
	msm_set_mux("CY8C24894_WOT", CONFIG_ACTIVE);
	msm_set_mux("CY8C24894_PWR", CONFIG_ACTIVE);
	if(board_type >= DVT) {
		msm_set_mux("CY8C24894_LVL_SHIFT", CONFIG_ACTIVE);
	}

	msm_set_mux("UART2DM_RX", CONFIG_SLEEP);		// uart 2 dm

	msm_set_mux("KEYPAD_ROW4", CONFIG_ACTIVE);		//keypad
	msm_set_mux("KEYPAD_ROW3", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW2", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW1", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_ROW0", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL6", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL5", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL4", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL3", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL2", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL1", CONFIG_ACTIVE);
	msm_set_mux("KEYPAD_COL0", CONFIG_ACTIVE);

#ifdef CONFIG_MSM_PWM
	msm_set_mux("GP_MN_OUT", CONFIG_ACTIVE);	// pwm clock
#endif

	//:PATCH: This is needed to avoid uart1 RTS and TX lines glitch ( the lines are driven low )
	//while configuring the lines to input from the active state.
	gpio_set_value(GPIO_UART1DM_RTS, 1);		
	gpio_set_value(GPIO_UART1DM_TX, 1);		

#if !USE_HSUART_FOR_BT
	msm_set_mux("UART1_RTS", CONFIG_SLEEP);	// uart 1
	msm_set_mux("UART1_CTS", CONFIG_SLEEP);
	msm_set_mux("UART1_RX", CONFIG_SLEEP);
	msm_set_mux("UART1_TX", CONFIG_SLEEP);
#else
	msm_set_mux("UART1DM_RTS", CONFIG_SLEEP);
	msm_set_mux("UART1DM_CTS", CONFIG_SLEEP);
	msm_set_mux("UART1DM_RX", CONFIG_SLEEP);
	msm_set_mux("UART1DM_TX", CONFIG_SLEEP);
#endif

#ifdef CONFIG_LEDS_LP8501
	msm_set_mux("LP8501_EN", CONFIG_ACTIVE);	// led controller enable
	msm_set_mux("LP8501_INT", CONFIG_ACTIVE);	// led controller interrupt
#endif

	msm_set_mux("SDIO_INT", CONFIG_SLEEP);		// wifi sdio
	msm_set_mux("SDIO_RESET", CONFIG_SLEEP);
	msm_set_mux("WIFI_EN", CONFIG_SLEEP);

	msm_set_mux("WL_SD_CLK",   CONFIG_SLEEP);
	msm_set_mux("WL_SD_CMD",   CONFIG_SLEEP);
	msm_set_mux("WL_SD_DATA3", CONFIG_SLEEP);
	msm_set_mux("WL_SD_DATA2", CONFIG_SLEEP);
	msm_set_mux("WL_SD_DATA1", CONFIG_SLEEP);
	msm_set_mux("WL_SD_DATA0", CONFIG_SLEEP);

#ifdef CONFIG_MSM_VIBRATOR
	msm_set_mux("VIBRATOR_EN", CONFIG_ACTIVE);	// vibrator enable
#endif

#ifdef CONFIG_W1_MASTER_GPIO
	msm_set_mux("BATTERY_GAUGE", CONFIG_ACTIVE);	// 1-wire for battery gas gauge
#endif
	
	msm_set_mux("BT_RESET", CONFIG_ACTIVE);		// bluetooth 
	msm_set_mux("BT_BT_WAKE_HOST", CONFIG_ACTIVE);

	msm_set_mux("CAM_DATA2", CONFIG_SLEEP);		// camera
	msm_set_mux("CAM_DATA3", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA4", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA5", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA6", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA7", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA8", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA9", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA10", CONFIG_SLEEP);
	msm_set_mux("CAM_DATA11", CONFIG_SLEEP);
	msm_set_mux("CAM_PCLK", CONFIG_SLEEP);
	msm_set_mux("CAM_HSYNC", CONFIG_SLEEP);
	msm_set_mux("CAM_VSYNC", CONFIG_SLEEP);
	msm_set_mux("CAM_MCLK", CONFIG_SLEEP);
}

static void __init chuck_init(void)
{
	rat_type = chuck_get_rat_type();
	chuck_mux_init_config();

	if(socinfo_init() < 0)
		printk("socinfo_init failed!\n");

	msm_acpu_clock_init(&chuck_clock_data);

#ifdef CONFIG_PROFILING 
	init_perf();
#endif //CONFIG_PROFILING

	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_add_devices();
	chuck_init_mmc();

#ifdef CONFIG_W1_MASTER_GPIO
	if (board_type >= EVT2) {
		
		if (board_type > EVT3) {
			msm_w1_gpio_platform_data.w1_read_bit_drive_low_time = 2;
			msm_w1_gpio_platform_data.w1_read_bit_release_time = 5;
			msm_w1_gpio_platform_data.w1_read_bit_delay_time = 63;
		}
		
		platform_add_devices(w1gpiodevices, ARRAY_SIZE(w1gpiodevices));
	}
#endif

#ifdef CONFIG_FB_MSM

	// EVT0 uses LG NT35380 LCD, EVT1 and up use LG NT35393
	// Since it is detected/initialized in bootie, for now linux will work with NT35380
	// settings, it will work for NT35393 as well.
	// TODO: Create a separate device (with mddi partial updates enabled and real vsync pin enabled)
	// and add parameter from bootie to linux to specify the exact model being used.
	platform_device_register(&msm_mddi_lg_nt35380_device);

#endif

#ifdef CONFIG_KEYBOARD_GPIO_MATRIX
	if (board_type == EMU) {
		kp_data.keymap = keymap_emu;
	}
	platform_device_register(&pixie_gpio_keypad_device);
#endif

	chuck_init_tp();	
	
#ifdef CONFIG_BLUETOOTH_POWER_STATE
	platform_device_register(&bluetooth_power_state_device);
#endif

#ifdef CONFIG_ISL29018_PROX_SENSOR
	i2c_register_board_info(0, &isl29018_i2c_board_info, 1);
#endif

#ifdef CONFIG_ACCELEROMETER_KXSD9
	if (board_type >= EVT2) {
		if (board_type >= EVT3) {
			board_kxsd9_platform_data.xyz_translation_map = kxsd9_xyz_translation_map_evt3;
		}
		i2c_register_board_info(0, &kxsd9_i2c_board_info, 1);
	}
#endif

#ifdef CONFIG_LEDS_LP8501
	i2c_register_board_info(0, &lp8501_i2c_board_info_evt1, 1);
#endif

#ifdef CONFIG_THERMISTOR_TMP105
	i2c_register_board_info(0, &tmp105_i2c_board_info, 1);
#endif

#ifdef CONFIG_CHARGER_SMB339
        if (board_type >= EVT3) {
	    i2c_register_board_info(0, &smb339_i2c_board_info, 1);
        }
#endif
	
#ifdef CONFIG_CY8C24894_TP
	i2c_register_board_info(0, &cy8c24894_i2c_board_info, 1);
#endif // CONFIG_CY8C24894_TP

#ifdef CONFIG_MSM_CAMERA
	i2c_register_board_info(0, &ov2650_i2c_board_info, 1);
	config_camera_off_gpios();	
#endif

	msm_pm_set_platform_data(msm7x27_pm_data);

	/* enable panic on oops */
	panic_on_oops = 1;
}

static void __init chuck_map_io(void)
{
	msm_map_common_io();
	msm_map_blenny_io();
	msm_clock_init(chuck_clocks, msm_num_chuck_clocks);

	chuck_allocate_mem_regions();

	chuck_allocate_pmem_regions();

#ifdef CONFIG_CACHE_L2X0
	l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

static void	__init chuck_fixup(struct machine_desc *m, struct tag *t, char **c, struct meminfo *mi)
{
	/* XXX total hack to work around problem with old bootloaders that report the first
	 * 2MB of the first bank as non existent.
	 */
	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_MEM) {
			if (t->u.mem.start == 0x00200000) {
				t->u.mem.start = 0;
				t->u.mem.size += 0x00200000;
			}
		}
	}
}

MACHINE_START(CHUCK, "Chuck")

/* UART for LL DEBUG */
	.phys_io	= MSM_UART3_PHYS,
	.io_pg_offst	= ((MSM_UART3_BASE) >> 18) & 0xfffc,

	.boot_params	= 0x00200100,
	.fixup		= chuck_fixup,
	.map_io		= chuck_map_io,
	.init_irq	= chuck_init_irq,
	.init_machine	= chuck_init,
	.timer		= &msm_timer,
MACHINE_END
