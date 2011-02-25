/*
 * driver/serial/pl011_kgdb.c
 *
 * Support for KGDB on ARM AMBA PL011 UARTs
 *
 * Authors: Manish Lachwani <mlachwani@mvista.com>
 *          Deepak Saxena <dsaxena@plexity.net>
 *
 * Copyright (c) 2005-2007 MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether expressor implied.
 *
 */
#include <linux/kgdb.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/io.h>

#include <asm/processor.h>
#include <asm/hardware.h>

static int kgdb_irq = CONFIG_KGDB_AMBA_IRQ;

#define UART_DIVISOR	(CONFIG_KGDB_AMBA_UARTCLK * 4 / CONFIG_KGDB_BAUDRATE)
/*
 * Todo: IO_ADDRESS is not very generic across ARM...
 */
static volatile unsigned char *kgdb_port =
	(unsigned char *)IO_ADDRESS(CONFIG_KGDB_AMBA_BASE);

/*
 * Init code taken from amba-pl011.c.
 */
static int kgdb_serial_init(void)
{
	writew(0, kgdb_port + UART010_CR);

	/* Set baud rate */
	writew(UART_DIVISOR & 0x3f, kgdb_port + UART011_FBRD);
	writew(UART_DIVISOR >> 6, kgdb_port + UART011_IBRD);

	writew(UART01x_LCRH_WLEN_8 | UART01x_LCRH_FEN, kgdb_port +
		UART010_LCRH);
	writew(UART01x_CR_UARTEN | UART011_CR_TXE | UART011_CR_RXE,
	       kgdb_port + UART010_CR);

	writew(UART011_RXIM, kgdb_port + UART011_IMSC);

	return 0;
}

static void kgdb_serial_putchar(u8 ch)
{
	unsigned int status;

	do {
		status = readw(kgdb_port + UART01x_FR);
	} while (status & UART01x_FR_TXFF);

	writew(ch, kgdb_port + UART01x_DR);
}

static int kgdb_serial_getchar(void)
{
	unsigned int status;
	int ch;

#ifdef CONFIG_DEBUG_LL_OLD
	printascii("Entering serial_getchar loop");
#endif
	do {
		status = readw(kgdb_port + UART01x_FR);
	} while (status & UART01x_FR_RXFE);
	ch = readw(kgdb_port + UART01x_DR);
#ifdef CONFIG_DEBUG_LL_OLD
	printascii("Exited serial_getchar loop");
	printascii("Read char: ");
	printch(ch);
	printascii("\n");
#endif
	return ch;
}

static irqreturn_t kgdb_interrupt(int irq, void *dev_id)
{
	int status = readw(kgdb_port + UART011_MIS);

#ifdef CONFIG_DEBUG_LL_OLD
	printascii("KGDB irq\n");
#endif
	if (irq != kgdb_irq)
		return IRQ_NONE;

	if (status & 0x40)
		breakpoint();

	return IRQ_HANDLED;
}

static void __init kgdb_hookup_irq(void)
{
	int retval;
	retval = request_irq(kgdb_irq, kgdb_interrupt, IRQF_SHARED,
		"KGDB-serial", (void *)kgdb_port);
	if (retval) {
		printk(KERN_ERR "pl011_kgdb: ERROR requesting irq\n");
	}
}

struct kgdb_io kgdb_io_ops = {
	.init = kgdb_serial_init,
	.write_char = kgdb_serial_putchar,
	.read_char  = kgdb_serial_getchar,
	.late_init  = kgdb_hookup_irq,
};
