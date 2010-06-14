/*
 * linux/arch/arm/mach-omap3pe/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/irda.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/twl4030-rtc.h>
#include <asm/arch/mcspi.h>
#include <asm/arch/power_companion.h>

#include <asm/io.h>
#include <asm/delay.h>

#define CONTROL_SYSC_SMARTIDLE	(0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE	(0x1)

#define	SDP3430_SMC91X_CS	3

#define TWL4030_MSECURE_GPIO    22

/* GPIO used for TSC2046 (touchscreen)
 *
 * Also note that the tsc2046 is the same silicon as the ads7846, so
 * that driver is used for the touchscreen. */
#define TS_GPIO                2 

extern int twl4030_vaux1_ldo_use(void);
extern int twl4030_vaux1_ldo_unuse(void);

extern int twl4030_vaux3_ldo_use(void);
extern int twl4030_vaux3_ldo_unuse(void);

static struct resource sdp3430_smc91x_resources[] = {
	[0] = {
		.start	= OMAP34XX_ETHR_START,
		.end	= OMAP34XX_ETHR_START + SZ_4K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device sdp3430_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sdp3430_smc91x_resources),
	.resource	= sdp3430_smc91x_resources,
};

static void scm_clk_init(void)
{
	struct clk *p_omap_ctrl_clk = NULL;

	p_omap_ctrl_clk = clk_get (NULL, "omapctrl_ick");
	if (p_omap_ctrl_clk != NULL) {
		if (clk_enable(p_omap_ctrl_clk)	!= 0) {
			printk(KERN_ERR "failed to enable scm clks\n");
			clk_put(p_omap_ctrl_clk);
		}
	}
	/* Sysconfig set to SMARTIDLE and AUTOIDLE */ 
	CONTROL_SYSCONFIG = (CONTROL_SYSC_SMARTIDLE | CONTROL_SYSC_AUTOIDLE);
}
	
static void ads7846_dev_init(void)
{
	if (omap_request_gpio(TS_GPIO) < 0)
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");

	omap_set_gpio_direction(TS_GPIO, 1);

	omap_set_gpio_debounce(TS_GPIO, 1);
	omap_set_gpio_debounce_time(TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain(TS_GPIO);
}

/* This enable(1)/disable(0) the voltage for TS: uses twl4030 calls */
static int ads7846_vaux_control(int vaux_cntrl)
{
	int ret = 0;

	/* check for return value of ldo_use: if success it returns 0*/
	if (vaux_cntrl == VAUX_ENABLE)
		ret = twl4030_vaux3_ldo_use();
	else if (vaux_cntrl == VAUX_DISABLE)
		ret = twl4030_vaux3_ldo_unuse();

	return ret;
}

static struct ads7846_platform_data tsc2046_config __initdata = {
	.get_pendown_state = ads7846_get_pendown_state,
	.keep_vref_on	   = 1,
	.vaux_control	   = ads7846_vaux_control,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

#ifdef CONFIG_SPI_TI_OMAP_TEST
static struct omap2_mcspi_device_config sublcd_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

static struct omap2_mcspi_device_config dummy1_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};
static struct omap2_mcspi_device_config dummy2_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 0,  /* 0: slave, 1: master */
};
#endif

static struct spi_board_info sdp3430_spi_board_info[] __initdata = {
	[0] = {
		/* TSC2046 operates at a max freqency of 2MHz, so
		 * operate slightly below at 1.5MHz */
		.modalias	= "ads7846",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz   = 1500000,
		.controller_data= &tsc2046_mcspi_config,
		.irq		 = OMAP_GPIO_IRQ(TS_GPIO),
		.platform_data  = &tsc2046_config,
	},
#ifdef CONFIG_SPI_TI_OMAP_TEST
/* below info is only for test case */
	[1] = {
		.modalias	= "dummydevice1",
		.bus_num	= 2,
		.chip_select	= 0,
		.max_speed_hz   = 1500000,
		.controller_data= &dummy1_mcspi_config,
	},

	[2] = {
		.modalias	= "dummydevice2",
		.bus_num	= 3,
		.chip_select	= 0,
		.max_speed_hz   = 6000000,
		.controller_data= &dummy2_mcspi_config,
	},
	[3] = {
		.modalias	= "sublcd",
		.bus_num	= 1,
		.chip_select	= 2,
		.max_speed_hz   = 1500000,
		.controller_data= &sublcd_mcspi_config,
	},
#endif
};

/* IrDA
 */
#if defined(CONFIG_OMAP_IR) || defined(CONFIG_OMAP_IR_MODULE)

#define	IRDA_SD	164	/* gpio 164 */
#define	IRDA_TX	166	/* gpio 166 */
#define	IRDA_SD_PIN	T21_3430_GPIO164
#define	IRDA_TX_PIN	V21_3430_GPIO166


static int select_irda(struct device *dev, int state)
{
	if (state == IR_SEL) {
		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
			twl4030_vaux1_ldo_use();
		else
			twl4030_vaux3_ldo_use();

		omap_cfg_reg("R21_3430_UART3_CTS_RCTX");
		omap_cfg_reg("T21_3430_UART3_RTS_SD");
		omap_cfg_reg("U21_3430_UART3_RX_IRRX");
		omap_cfg_reg("V21_3430_UART3_TX_IRTX");

		omap_request_gpio(IRDA_SD);
		omap_request_gpio(IRDA_TX);
		omap_cfg_reg("IRDA_SD_PIN");
		omap_set_gpio_direction(IRDA_SD, GPIO_DIR_OUTPUT);
		omap_set_gpio_direction(IRDA_TX, GPIO_DIR_OUTPUT);
		omap_set_gpio_dataout(IRDA_SD, 0);
	} else {
		omap_free_gpio(IRDA_SD);
		omap_free_gpio(IRDA_TX);
		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
			twl4030_vaux1_ldo_use();
		else
			twl4030_vaux3_ldo_use();
	}

	return 0;
}

static int transceiver_mode(struct device *dev, int mode)
{
	omap_cfg_reg("IRDA_SD_PIN");
	omap_cfg_reg("IRDA_TX_PIN");

	if (mode & IR_SIRMODE) { /* SIR */
		omap_set_gpio_dataout(IRDA_SD, 1);
		udelay(1);
		omap_set_gpio_dataout(IRDA_TX, 0);
		udelay(1);
		omap_set_gpio_dataout(IRDA_SD, 0);
		udelay(1);
	} else { /* MIR/FIR */
		omap_set_gpio_dataout(IRDA_SD, 1);
		udelay(1);
		omap_set_gpio_dataout(IRDA_TX, 1);
		udelay(1);
		omap_set_gpio_dataout(IRDA_SD, 0);
		udelay(1);
		omap_set_gpio_dataout(IRDA_TX, 0);
		udelay(1);
	}

	omap_cfg_reg("T21_3430_UART3_RTS_SD");
	omap_cfg_reg("V21_3430_UART3_TX_IRTX");
	return 0;
}
#else
static int select_irda(struct device *dev, int state) { return 0; }
static int transceiver_mode(struct device *dev, int mode) { return 0; }
#endif

static struct omap_irda_config irda_data = {
	.transceiver_cap	= IR_SIRMODE | IR_MIRMODE | IR_FIRMODE,
	.transceiver_mode	= transceiver_mode,
	.select_irda	 	= select_irda,
	.rx_channel		= OMAP24XX_DMA_UART3_RX,
	.tx_channel		= OMAP24XX_DMA_UART3_TX,
	.dest_start		= OMAP_UART3_BASE,
	.src_start		= OMAP_UART3_BASE,
	.tx_trigger		= OMAP24XX_DMA_UART3_TX,
	.rx_trigger		= OMAP24XX_DMA_UART3_RX,
};

static struct resource irda_resources[] = {
	[0] = {
		.start	= INT_24XX_UART3_IRQ,
		.end	= INT_24XX_UART3_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device irda_device = {
	.name		= "omapirda",
	.id		= -1,
	.dev		= {
		.platform_data	= &irda_data,
	},
	.num_resources	= 1,
	.resource	= irda_resources,
};

/*
 * Key mapping for 3430 SDP board
 */

static int sdp3430_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(0, 4, KEY_C),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(1, 4, KEY_G),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(2, 4, KEY_3),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P),
	KEY(3, 4, KEY_Q),
	KEY(4, 0, KEY_R),
	KEY(4, 1, KEY_4),
	KEY(4, 2, KEY_T),
	KEY(4, 3, KEY_U),
	KEY(4, 4, KEY_D),
	KEY(5, 0, KEY_V),
	KEY(5, 1, KEY_W),
	KEY(5, 2, KEY_L),
	KEY(5, 3, KEY_S),
	KEY(5, 4, KEY_H),
	0
};

static struct omap_kp_platform_data sdp3430_kp_data = {
	.rows		= 5,
	.cols		= 6,
	.keymap 	= sdp3430_keymap,
	.keymapsize 	= ARRAY_SIZE(sdp3430_keymap),
	.rep		= 1,
};

static struct platform_device sdp3430_kp_device = {
	.name		= "omap_twl4030keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &sdp3430_kp_data,
	},
};

#ifdef CONFIG_RTC_DRV_TWL4030
static int twl4030_rtc_init(void)
{
	int ret = 0;

	/* 3430ES2.0 doesn't have msecure/gpio-22 line connected to T2 */
	if (is_device_type_gp() && is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		u32 msecure_pad_config_reg = OMAP2_CTRL_BASE + 0xA3C;
		int mux_mask = 0x04;
		u16 tmp;

		ret = omap_request_gpio(TWL4030_MSECURE_GPIO);
		if (ret < 0) {
			printk(KERN_ERR "twl4030_rtc_init: can't"
				"reserve GPIO:%d !\n", TWL4030_MSECURE_GPIO);
			goto out;
		}
		/*
		 * TWL4030 will be in secure mode if msecure line from OMAP
		 * is low. Make msecure line high in order to change the
		 * TWL4030 RTC time and calender registers.
		 */
		omap_set_gpio_direction(TWL4030_MSECURE_GPIO, 0);

		tmp = omap_readw(msecure_pad_config_reg);
		tmp &= 0xF8;	/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;/* To enable mux mode 03/04 = GPIO_RTC */
		omap_writew(tmp, msecure_pad_config_reg);

		omap_set_gpio_dataout(TWL4030_MSECURE_GPIO, 1);
	}
out:
	return ret;
}

static void twl4030_rtc_exit(void)
{
	omap_free_gpio(TWL4030_MSECURE_GPIO);
}

static struct twl4030rtc_platform_data sdp3430_twl4030rtc_data = {
	.init = &twl4030_rtc_init,
	.exit = &twl4030_rtc_exit,
};

static struct platform_device sdp3430_twl4030rtc_device = {
	.name		= "twl4030_rtc",
	.id		= -1,
	.dev		= {
		.platform_data	= &sdp3430_twl4030rtc_data,
	},
};
#endif

static struct platform_device *sdp3430_devices[] __initdata = {
	&sdp3430_smc91x_device,
	&irda_device,
	&sdp3430_kp_device,
#ifdef CONFIG_RTC_DRV_TWL4030
	&sdp3430_twl4030rtc_device,
#endif
};

static inline void __init sdp3430_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	unsigned int rate;
	struct clk *l3ck;
	int eth_gpio = 0;

	eth_cs	= SDP3430_SMC91X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		return;
	}

	sdp3430_smc91x_resources[0].start = cs_mem_base + 0x0;
	sdp3430_smc91x_resources[0].end   = cs_mem_base + 0xf;
	udelay(100);

	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV2;
	else
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV1;

	sdp3430_smc91x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

	if (omap_request_gpio(eth_gpio) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			eth_gpio);
		return;
	}
	omap_set_gpio_direction(eth_gpio, 1);
}

static void __init omap_3430sdp_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	sdp3430_init_smc91x();
}

static struct omap_uart_config sdp3430_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};
static int __init omap3430_i2c_init(void)
{
	omap_register_i2c_bus(1, CONFIG_I2C_OMAP34XX_HS_BUS1, NULL, 0);
	omap_register_i2c_bus(2, CONFIG_I2C_OMAP34XX_HS_BUS2, NULL, 0);
	omap_register_i2c_bus(3, CONFIG_I2C_OMAP34XX_HS_BUS3, NULL, 0);
	return 0;
}

static struct omap_mmc_config sdp3430_mmc_config __initdata = {
	.mmc [0] = {
		.enabled        = 1,
		.wire4          = 1,
		.wp_pin         = -1,
		.power_pin      = -1,
		.switch_pin     = 0,
	},
	.mmc [1] = {
		.enabled        = 1,
		.wire4          = 1,
		.wp_pin         = -1,
		.power_pin      = -1,
		.switch_pin     = 1,
	},
};

static struct omap_board_config_kernel sdp3430_config[] __initdata = {
	{ OMAP_TAG_UART,	&sdp3430_uart_config },
	{ OMAP_TAG_MMC,         &sdp3430_mmc_config },
};

extern void __init sdp_usb_init(void);
extern void __init sdp3430_flash_init(void);

static void __init omap_3430sdp_init(void)
{
	/* System Control module clock initialization */
	scm_clk_init();	
	platform_add_devices(sdp3430_devices, ARRAY_SIZE(sdp3430_devices));
	omap_board_config = sdp3430_config;
	omap_board_config_size = ARRAY_SIZE(sdp3430_config);
	spi_register_board_info(sdp3430_spi_board_info,
				ARRAY_SIZE(sdp3430_spi_board_info));
	sdp3430_flash_init();
	ads7846_dev_init();
	omap_serial_init();
	
	sdp_usb_init();
}
arch_initcall(omap3430_i2c_init);

static void __init omap_3430sdp_map_io(void)
{
	omap2_map_common_io();
}

MACHINE_START(OMAP_3430SDP, "OMAP3430 sdp3430 board")
	/* Maintainer: Syed Khasim - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_3430sdp_map_io,
	.init_irq	= omap_3430sdp_init_irq,
	.init_machine	= omap_3430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
