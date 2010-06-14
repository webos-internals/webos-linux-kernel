/*
 * arch/arm/mach-omap2/serial.c
 *
 * OMAP2 serial support.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Based off of arch/arm/mach-omap/omap1/serial.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/serial_8250.h>
#include <linux/serial_reg.h>
#include <linux/kgdb.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <asm/io.h>

#include <asm/arch/common.h>
#include <asm/arch/board.h>

static struct clk * uart1_ick = NULL;
static struct clk * uart1_fck = NULL;
static struct clk * uart2_ick = NULL;
static struct clk * uart2_fck = NULL;
static struct clk * uart3_ick = NULL;
static struct clk * uart3_fck = NULL;

static struct plat_serial8250_port serial_platform_data[] = {
	{
		.membase	= (char *)IO_ADDRESS(OMAP_UART1_BASE),
		.mapbase	= (unsigned long)OMAP_UART1_BASE,
		.irq		= 72,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP16XX_BASE_BAUD * 16,
	}, {
		.membase	= (char *)IO_ADDRESS(OMAP_UART2_BASE),
		.mapbase	= (unsigned long)OMAP_UART2_BASE,
		.irq		= 73,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP16XX_BASE_BAUD * 16,
	}, {
		.membase	= (char *)IO_ADDRESS(OMAP_UART3_BASE),
		.mapbase	= (unsigned long)OMAP_UART3_BASE,
		.irq		= 74,
		.flags		= UPF_BOOT_AUTOCONF,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= OMAP16XX_BASE_BAUD * 16,
	}, {
		.flags		= 0
	}
};

struct uart_params {
	volatile u32	*wken_regaddr;
	u8		wken_bitpos;
	volatile u32	*sysc_regaddr;
	volatile u32	*wer_regaddr;
	volatile u32	*scr_regaddr;
	volatile u32	*ssr_regaddr;
};

struct uart_params uart_wkup_params[OMAP_MAX_NR_PORTS];

static inline unsigned int serial_read_reg(struct plat_serial8250_port *up,
					   int offset)
{
	offset <<= up->regshift;
	return (unsigned int)__raw_readb(up->membase + offset);
}

static inline void serial_write_reg(struct plat_serial8250_port *p, int offset,
				    int value)
{
	offset <<= p->regshift;
	__raw_writeb(value, (unsigned long)(p->membase + offset));
}

/*
 * Internal UARTs need to be initialized for the 8250 autoconfig to work
 * properly. Note that the TX watermark initialization may not be needed
 * once the 8250.c watermark handling code is merged.
 */
static inline void __init omap_serial_reset(struct plat_serial8250_port *p)
{
	unsigned long timeout;
	int lcr = 0;
#ifdef CONFIG_DEBUG_LL
	int dll, dlh;
	
	lcr = serial_read_reg(p, UART_LCR);
	serial_write_reg(p, UART_LCR, UART_LCR_DLAB | lcr);
	dll = serial_read_reg(p, UART_DLL);
	dlh = serial_read_reg(p, UART_DLM);
#endif

	serial_write_reg(p, UART_OMAP_SYSC, 0x2);
	timeout = jiffies + msecs_to_jiffies(10);
	while (!(serial_read_reg(p, UART_OMAP_SYSS) & 1)
		&& time_before(jiffies, timeout)) {
	udelay(10);
	}
	if (unlikely(!(serial_read_reg(p, UART_OMAP_SYSS) & 1)))
		printk(KERN_WARNING ": timeout waiting for uart reset\n");

	serial_write_reg(p, UART_LCR, 0xBF);
	serial_write_reg(p, UART_EFR, UART_EFR_ECB);
#ifdef CONFIG_DEBUG_LL
	serial_write_reg(p, UART_DLL, dll);
	serial_write_reg(p, UART_DLM, dlh);
#endif
	serial_write_reg(p, UART_LCR, lcr & ~UART_LCR_DLAB);
	serial_write_reg(p, UART_IER, 0x00);
	serial_write_reg(p, UART_MCR, 0x00);
	serial_write_reg(p, UART_FCR, 7);

	serial_write_reg(p, UART_OMAP_MDR1, 0x07);
	serial_write_reg(p, UART_OMAP_SCR, 0x08);
	serial_write_reg(p, UART_OMAP_MDR1, 0x00);
	serial_write_reg(p, UART_OMAP_SYSC, (0x02 << 3) | (1 << 2) | (1 << 0));
}

#ifdef CONFIG_ARCH_OMAP3
void omap_serial_wken(void)
{
	int i;
	u32 uart_sysc, pm_wken, scr;
	u16 ssr;

	uart_wkup_params[0].wken_regaddr = &PM_WKEN1_CORE;
	uart_wkup_params[0].wken_bitpos = 13;
	uart_wkup_params[0].sysc_regaddr = &PRCM_UART1_SYSCONFIG;
	uart_wkup_params[0].wer_regaddr = &PRCM_UART1_WER;
	uart_wkup_params[0].scr_regaddr = &PRCM_UART1_SCR;
	uart_wkup_params[0].ssr_regaddr = &PRCM_UART1_SSR;

	uart_wkup_params[1].wken_regaddr = &PM_WKEN1_CORE;
	uart_wkup_params[1].wken_bitpos = 14;
	uart_wkup_params[1].sysc_regaddr = &PRCM_UART2_SYSCONFIG;
	uart_wkup_params[1].wer_regaddr = &PRCM_UART2_WER;
	uart_wkup_params[1].scr_regaddr = &PRCM_UART2_SCR;
	uart_wkup_params[1].ssr_regaddr = &PRCM_UART2_SSR;

	uart_wkup_params[2].wken_regaddr = &PM_WKEN_PER;
	uart_wkup_params[2].wken_bitpos = 11;
	uart_wkup_params[2].sysc_regaddr = &PRCM_UART3_SYSCONFIG;
	uart_wkup_params[2].wer_regaddr = &PRCM_UART3_WER;
	uart_wkup_params[2].scr_regaddr = &PRCM_UART3_SCR;
	uart_wkup_params[2].ssr_regaddr = &PRCM_UART3_SSR;

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		/* Enable module wakeup capability */
		pm_wken = *uart_wkup_params[i].wken_regaddr;
		pm_wken |= 1<<(uart_wkup_params[i].wken_bitpos);
		*uart_wkup_params[i].wken_regaddr = pm_wken;

		/* Enable smart idle in UART SYSC */
		uart_sysc = *uart_wkup_params[i].sysc_regaddr;
		uart_sysc |= (2<<3)|(1<<2);
		*uart_wkup_params[i].sysc_regaddr = uart_sysc;

		/* Enabling all wakeup bits in WER_REG */
		*uart_wkup_params[i].wer_regaddr = 0x3F;

		/* Check if wakeup status is set */
		ssr = *uart_wkup_params[i].ssr_regaddr;
		if (ssr & 0x2) {
			/*
			 * If RX_CTS wakeup has occurred, then clear SCR_REG for
			 * further wakeups
			 */
			scr = *uart_wkup_params[i].scr_regaddr;
			scr &= ~(0x10);
			*uart_wkup_params[i].scr_regaddr = scr;
		}
		/* Enabling interrupts for wakeup in SCR_REG */
		scr = *uart_wkup_params[i].scr_regaddr;
		scr |= 0x10;
		*uart_wkup_params[i].scr_regaddr = scr;
	}
}
#endif

void __init omap_serial_init()
{
	int i;
	const struct omap_uart_config *info;
	struct device *dev = NULL;

	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */

	info = omap_get_config(OMAP_TAG_UART,
			       struct omap_uart_config);

	if (info == NULL)
		return;

	for (i = 0; i < OMAP_MAX_NR_PORTS; i++) {
		struct plat_serial8250_port *p = serial_platform_data + i;

		if (!(info->enabled_uarts & (1 << i))) {
			p->membase = 0;
			p->mapbase = 0;
			continue;
		}

		switch (i) {
		case 0:
			uart1_ick = clk_get(dev, "uart1_ick");
			if (IS_ERR(uart1_ick))
				printk("Could not get uart1_ick\n");
			else {
				clk_enable(uart1_ick);
			}

			uart1_fck = clk_get(dev, "uart1_fck");
			if (IS_ERR(uart1_fck))
				printk("Could not get uart1_fck\n");
			else {
				clk_enable(uart1_fck);
			}
			break;
		case 1:
			uart2_ick = clk_get(dev, "uart2_ick");
			if (IS_ERR(uart2_ick))
				printk("Could not get uart2_ick\n");
			else {
				clk_enable(uart2_ick);
			}

			uart2_fck = clk_get(dev, "uart2_fck");
			if (IS_ERR(uart2_fck))
				printk("Could not get uart2_fck\n");
			else {
				clk_enable(uart2_fck);
			}
			break;
		case 2:
			uart3_ick = clk_get(dev, "uart3_ick");
			if (IS_ERR(uart3_ick))
				printk("Could not get uart3_ick\n");
			else {
				clk_enable(uart3_ick);
			}

			uart3_fck = clk_get(dev, "uart3_fck");
			if (IS_ERR(uart3_fck))
				printk("Could not get uart3_fck\n");
			else {
				clk_enable(uart3_fck);
			}
			break;
		}

		omap_serial_reset(p);
#ifdef CONFIG_KGDB_8250
		kgdb8250_add_platform_port(i, p);
#endif
	}
}


static struct platform_device serial_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev			= {
		.platform_data	= serial_platform_data,
	},
};

static int __init omap_init(void)
{
	return platform_device_register(&serial_device);
}
arch_initcall(omap_init);
