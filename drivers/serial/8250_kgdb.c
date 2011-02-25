/*
 * 8250 interface for kgdb.
 *
 * This is a merging of many different drivers, and all of the people have
 * had an impact in some form or another:
 *
 * 2004-2005 (c) MontaVista Software, Inc.
 * 2005-2006 (c) Wind River Systems, Inc.
 *
 * Amit Kale <amitkale@emsyssoft.com>, David Grothe <dave@gcom.com>,
 * Scott Foehner <sfoehner@engr.sgi.com>, George Anzinger <george@mvista.com>,
 * Robert Walsh <rjwalsh@durables.org>, wangdi <wangdi@clusterfs.com>,
 * San Mehat, Tom Rini <trini@mvista.com>,
 * Jason Wessel <jason.wessel@windriver.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kgdb.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/serialP.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <asm/serial.h>		/* For BASE_BAUD and SERIAL_PORT_DFNS */

#include "8250.h"

#define GDB_BUF_SIZE	512	/* power of 2, please */

MODULE_DESCRIPTION("KGDB driver for the 8250");
MODULE_LICENSE("GPL");
/* These will conflict with early_param otherwise. */
#ifdef CONFIG_KGDB_8250_MODULE
static char config[256];
module_param_string(kgdb8250, config, 256, 0);
MODULE_PARM_DESC(kgdb8250,
		 " kgdb8250=<io or mmio>,<address>,<baud rate>,<irq>\n");
static struct kgdb_io local_kgdb_io_ops;
#endif				/* CONFIG_KGDB_8250_MODULE */

/* Speed of the UART. */
static int kgdb8250_baud;

/* Flag for if we need to call request_mem_region */
static int kgdb8250_needs_request_mem_region;

static char kgdb8250_buf[GDB_BUF_SIZE];
static atomic_t kgdb8250_buf_in_cnt;
static int kgdb8250_buf_out_inx;

/* Old-style serial definitions, if existant, and a counter. */
#ifdef CONFIG_KGDB_SIMPLE_SERIAL
static int should_copy_rs_table = 1;
static struct serial_state old_rs_table[] __initdata = {
#ifdef SERIAL_PORT_DFNS
	SERIAL_PORT_DFNS
#endif
};
#endif

/* Our internal table of UARTS. */
#define UART_NR	CONFIG_SERIAL_8250_NR_UARTS
static struct uart_port kgdb8250_ports[UART_NR];

static struct uart_port *current_port;

/* Base of the UART. */
static void *kgdb8250_addr;

/* Forward declarations. */
static int kgdb8250_uart_init(void);
static int __init kgdb_init_io(void);
static int __init kgdb8250_opt(char *str);

/* These are much shorter calls to ioread8/iowrite8 that take into
 * account our shifts, etc. */
static inline unsigned int kgdb_ioread(u8 mask)
{
	return ioread8(kgdb8250_addr + (mask << current_port->regshift));
}

static inline void kgdb_iowrite(u8 val, u8 mask)
{
	iowrite8(val, kgdb8250_addr + (mask << current_port->regshift));
}

/*
 * Wait until the interface can accept a char, then write it.
 */
static void kgdb_put_debug_char(u8 chr)
{
	while (!(kgdb_ioread(UART_LSR) & UART_LSR_THRE)) ;

	kgdb_iowrite(chr, UART_TX);
}

/*
 * Get a byte from the hardware data buffer and return it
 */
static int read_data_bfr(void)
{
	char it = kgdb_ioread(UART_LSR);

	if (it & UART_LSR_DR)
		return kgdb_ioread(UART_RX);

	/*
	 * If we have a framing error assume somebody messed with
	 * our uart.  Reprogram it and send '-' both ways...
	 */
	if (it & 0xc) {
		kgdb8250_uart_init();
		kgdb_put_debug_char('-');
		return '-';
	}

	return -1;
}

/*
 * Get a char if available, return -1 if nothing available.
 * Empty the receive buffer first, then look at the interface hardware.
 */
static int kgdb_get_debug_char(void)
{
	int retchr;

	/* intr routine has q'd chars */
	if (atomic_read(&kgdb8250_buf_in_cnt) != 0) {
		retchr = kgdb8250_buf[kgdb8250_buf_out_inx++];
		kgdb8250_buf_out_inx &= (GDB_BUF_SIZE - 1);
		atomic_dec(&kgdb8250_buf_in_cnt);
		return retchr;
	}

	do {
		retchr = read_data_bfr();
	} while (retchr < 0);

	return retchr;
}

/*
 * This is the receiver interrupt routine for the GDB stub.
 * All that we need to do is verify that the interrupt happened on the
 * line we're in charge of.  If this is true, schedule a breakpoint and
 * return.
 */
static irqreturn_t
kgdb8250_interrupt(int irq, void *dev_id)
{
	if (kgdb_ioread(UART_IIR) & UART_IIR_RDI) {
		/* Throw away the data if another I/O routine is active. */
		if (kgdb_io_ops.read_char != kgdb_get_debug_char &&
				(kgdb_ioread(UART_LSR) & UART_LSR_DR))
			kgdb_ioread(UART_RX);
		else
			breakpoint();
	}

	return IRQ_HANDLED;
}

/*
 *  Initializes the UART.
 *  Returns:
 *	0 on success, 1 on failure.
 */
static int kgdb8250_uart_init(void)
{
	unsigned int ier;
	unsigned int base_baud = current_port->uartclk ?
		current_port->uartclk / 16 : BASE_BAUD;

	/* test uart existance */
	if (kgdb_ioread(UART_LSR) == 0xff)
		return -1;

	/* disable interrupts */
	kgdb_iowrite(0, UART_IER);

#if defined(CONFIG_ARCH_OMAP1510)
	/* Workaround to enable 115200 baud on OMAP1510 internal ports */
	if (cpu_is_omap1510() && is_omap_port((void *)kgdb8250_addr)) {
		if (kgdb8250_baud == 115200) {
			base_baud = 1;
			kgdb8250_baud = 1;
			kgdb_iowrite(1, UART_OMAP_OSC_12M_SEL);
		} else
			kgdb_iowrite(0, UART_OMAP_OSC_12M_SEL);
	}
#endif
	/* set DLAB */
	kgdb_iowrite(UART_LCR_DLAB, UART_LCR);

	/* set baud */
	kgdb_iowrite((base_baud / kgdb8250_baud) & 0xff, UART_DLL);
	kgdb_iowrite((base_baud / kgdb8250_baud) >> 8, UART_DLM);

	/* reset DLAB, set LCR */
	kgdb_iowrite(UART_LCR_WLEN8, UART_LCR);

	/* set DTR and RTS */
	kgdb_iowrite(UART_MCR_OUT2 | UART_MCR_DTR | UART_MCR_RTS, UART_MCR);

	/* setup fifo */
	kgdb_iowrite(UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RCVR
		| UART_FCR_CLEAR_XMIT | UART_FCR_TRIGGER_8,
		UART_FCR);

	/* clear pending interrupts */
	kgdb_ioread(UART_IIR);
	kgdb_ioread(UART_RX);
	kgdb_ioread(UART_LSR);
	kgdb_ioread(UART_MSR);

	/* turn on RX interrupt only */
	kgdb_iowrite(UART_IER_RDI, UART_IER);

	/*
	 * Borrowed from the main 8250 driver.
	 * Try writing and reading the UART_IER_UUE bit (b6).
	 * If it works, this is probably one of the Xscale platform's
	 * internal UARTs.
	 * We're going to explicitly set the UUE bit to 0 before
	 * trying to write and read a 1 just to make sure it's not
	 * already a 1 and maybe locked there before we even start start.
	 */
	ier = kgdb_ioread(UART_IER);
	kgdb_iowrite(ier & ~UART_IER_UUE, UART_IER);
	if (!(kgdb_ioread(UART_IER) & UART_IER_UUE)) {
		/*
		 * OK it's in a known zero state, try writing and reading
		 * without disturbing the current state of the other bits.
		 */
		kgdb_iowrite(ier | UART_IER_UUE, UART_IER);
		if (kgdb_ioread(UART_IER) & UART_IER_UUE)
			/*
			 * It's an Xscale.
			 */
			ier |= UART_IER_UUE | UART_IER_RTOIE;
	}
	kgdb_iowrite(ier, UART_IER);
	return 0;
}

/*
 * Copy the old serial_state table to our uart_port table if we haven't
 * had values specifically configured in.  We need to make sure this only
 * happens once.
 */
static void __init kgdb8250_copy_rs_table(void)
{
#ifdef CONFIG_KGDB_SIMPLE_SERIAL
	int i;

	if (!should_copy_rs_table)
		return;

	for (i = 0; i < ARRAY_SIZE(old_rs_table); i++) {
		kgdb8250_ports[i].iobase = old_rs_table[i].port;
		kgdb8250_ports[i].irq = irq_canonicalize(old_rs_table[i].irq);
		kgdb8250_ports[i].uartclk = old_rs_table[i].baud_base * 16;
		kgdb8250_ports[i].membase = old_rs_table[i].iomem_base;
		kgdb8250_ports[i].iotype = old_rs_table[i].io_type;
		kgdb8250_ports[i].regshift = old_rs_table[i].iomem_reg_shift;
		kgdb8250_ports[i].line = i;
	}

	should_copy_rs_table = 0;
#endif
}

/*
 * Hookup our IRQ line now that it is safe to do so, after we grab any
 * memory regions we might need to.  If we haven't been initialized yet,
 * go ahead and copy the old_rs_table in.
 */
static void __init kgdb8250_late_init(void)
{
	/* Try and copy the old_rs_table. */
	kgdb8250_copy_rs_table();

#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	/* Take the port away from the main driver. */
	serial8250_unregister_by_port(current_port);

	/* Now reinit the port as the above has disabled things. */
	kgdb8250_uart_init();
#endif
	/* We may need to call request_mem_region() first. */
	if (kgdb8250_needs_request_mem_region)
		request_mem_region(current_port->mapbase,
				   8 << current_port->regshift, "kgdb");
	if (request_irq(current_port->irq, kgdb8250_interrupt, IRQF_SHARED,
			"GDB-stub", current_port) < 0)
		printk(KERN_ERR "KGDB failed to request the serial IRQ (%d)\n",
		       current_port->irq);
}

static __init int kgdb_init_io(void)
{
	/* Give us the basic table of uarts. */
	kgdb8250_copy_rs_table();

	/* We're either a module and parse a config string, or we have a
	 * semi-static config. */
#ifdef CONFIG_KGDB_8250_MODULE
	if (strlen(config)) {
		if (kgdb8250_opt(config))
			return -EINVAL;
	} else {
		printk(KERN_ERR "kgdb8250: argument error, usage: "
		       "kgdb8250=<io or mmio>,<address>,<baud rate>,<irq>\n");
		printk(KERN_ERR "kgdb8250: alt usage: "
		       "kgdb8250=<line #>,<baud rate>\n");
		return -EINVAL;
	}
#elif defined(CONFIG_KGDB_SIMPLE_SERIAL)
	kgdb8250_baud = CONFIG_KGDB_BAUDRATE;

	/* Setup our pointer to the serial port now. */
	current_port = &kgdb8250_ports[CONFIG_KGDB_PORT_NUM];
#else
	if (kgdb8250_opt(CONFIG_KGDB_8250_CONF_STRING))
		return -EINVAL;
#endif


	/* Internal driver setup. */
	switch (current_port->iotype) {
	case UPIO_MEM:
		if (current_port->mapbase)
			kgdb8250_needs_request_mem_region = 1;
		if (current_port->flags & UPF_IOREMAP) {
			current_port->membase = ioremap(current_port->mapbase,
						8 << current_port->regshift);
			if (!current_port->membase)
				return -EIO;	/* Failed. */
		}
		kgdb8250_addr = current_port->membase;
		break;
	case UPIO_PORT:
	default:
		kgdb8250_addr = ioport_map(current_port->iobase,
					   8 << current_port->regshift);
		if (!kgdb8250_addr)
			return -EIO;	/* Failed. */
	}

	if (kgdb8250_uart_init() == -1) {
		printk(KERN_ERR "kgdb8250: init failed\n");
		return -EIO;
	}
#ifdef CONFIG_KGDB_8250_MODULE
	/* Attach the kgdb irq. When this is built into the kernel, it
	 * is called as a part of late_init sequence.
	 */
	kgdb8250_late_init();
	if (kgdb_register_io_module(&local_kgdb_io_ops))
		return -EINVAL;

	printk(KERN_INFO "kgdb8250: debugging enabled\n");
#endif				/* CONFIG_KGD_8250_MODULE */

	return 0;
}

#ifdef CONFIG_KGDB_8250_MODULE
/* If it is a module the kgdb_io_ops should be a static which
 * is passed to the KGDB I/O initialization
 */
static void kgdb8250_pre_exception_handler(void);
static void kgdb8250_post_exception_handler(void);

static struct kgdb_io local_kgdb_io_ops = {
#else				/* ! CONFIG_KGDB_8250_MODULE */
struct kgdb_io kgdb_io_ops = {
#endif				/* ! CONFIG_KGD_8250_MODULE */
	.read_char = kgdb_get_debug_char,
	.write_char = kgdb_put_debug_char,
	.init = kgdb_init_io,
	.late_init = kgdb8250_late_init,
#ifdef CONFIG_KGDB_8250_MODULE
	.pre_exception = kgdb8250_pre_exception_handler,
	.post_exception = kgdb8250_post_exception_handler,
#endif
};

/**
 * 	kgdb8250_add_port - Define a serial port for use with KGDB
 * 	@i: The index of the port being added
 * 	@serial_req: The &struct uart_port describing the port
 *
 * 	On platforms where we must register the serial device
 * 	dynamically, this is the best option if a platform also normally
 * 	calls early_serial_setup().
 */
void kgdb8250_add_port(int i, struct uart_port *serial_req)
{
#ifdef CONFIG_KGDB_SIMPLE_SERIAL
	if (should_copy_rs_table)
		printk(KERN_ERR "8250_kgdb: warning will over write serial"
			   " port definitions at kgdb init time\n");
#endif

	/* Copy the whole thing over. */
	if (current_port != &kgdb8250_ports[i])
		memcpy(&kgdb8250_ports[i], serial_req,
		sizeof(struct uart_port));
}

/**
 * 	kgdb8250_add_platform_port - Define a serial port for use with KGDB
 * 	@i: The index of the port being added
 * 	@p: The &struct plat_serial8250_port describing the port
 *
 * 	On platforms where we must register the serial device
 * 	dynamically, this is the best option if a platform normally
 * 	handles uart setup with an array of &struct plat_serial8250_port.
 */
void __init kgdb8250_add_platform_port(int i, struct plat_serial8250_port *p)
{
	/* Make sure we've got the built-in data before we override. */
	kgdb8250_copy_rs_table();

	kgdb8250_ports[i].iobase = p->iobase;
	kgdb8250_ports[i].membase = p->membase;
	kgdb8250_ports[i].irq = p->irq;
	kgdb8250_ports[i].uartclk = p->uartclk;
	kgdb8250_ports[i].regshift = p->regshift;
	kgdb8250_ports[i].iotype = p->iotype;
	kgdb8250_ports[i].flags = p->flags;
	kgdb8250_ports[i].mapbase = p->mapbase;
}

/*
 * Syntax for this cmdline option is:
 * kgdb8250=<io or mmio>,<address>,<baud rate>,<irq>"
 */
static int __init kgdb8250_opt(char *str)
{
	/* We'll fill out and use the first slot. */
	current_port = &kgdb8250_ports[0];

	if (!strncmp(str, "io", 2)) {
		current_port->iotype = UPIO_PORT;
		str += 2;
	} else if (!strncmp(str, "mmap", 4)) {
		current_port->iotype = UPIO_MEM;
		current_port->flags |= UPF_IOREMAP;
		str += 4;
	} else if (!strncmp(str, "mmio", 4)) {
		current_port->iotype = UPIO_MEM;
		current_port->flags &= ~UPF_IOREMAP;
		str += 4;
	} else if (*str >= '0' || *str <= '9') {
		int line = *str - '0';
		/* UARTS in the list from 0 -> 9 */
		if (line >= UART_NR)
			goto errout;
		current_port = &kgdb8250_ports[line];
		if (serial8250_get_port_def(current_port, line))
			goto errout;
		str++;
		if (*str != ',')
			goto errout;
		str++;
		kgdb8250_baud = simple_strtoul(str, &str, 10);
		if (!kgdb8250_baud)
			goto errout;
		if (*str == ',')
			goto errout;
		goto finish;
	} else
		goto errout;

	if (*str != ',')
		goto errout;
	str++;

	if (current_port->iotype == UPIO_PORT)
		current_port->iobase = simple_strtoul(str, &str, 16);
	else {
		if (current_port->flags & UPF_IOREMAP)
			current_port->mapbase =
				(unsigned long) simple_strtoul(str, &str, 16);
		else
			current_port->membase =
				(void *) simple_strtoul(str, &str, 16);
	}

	if (*str != ',')
		goto errout;
	str++;

	kgdb8250_baud = simple_strtoul(str, &str, 10);
	if (!kgdb8250_baud)
		goto errout;

	if (*str != ',')
		goto errout;
	str++;

	current_port->irq = simple_strtoul(str, &str, 10);

	finish:
#ifdef CONFIG_KGDB_SIMPLE_SERIAL
	should_copy_rs_table = 0;
#endif

	return 0;

errout:
	printk(KERN_ERR "Invalid syntax for option kgdb8250=\n");
	return 1;
}

#ifdef CONFIG_KGDB_8250_MODULE
static void kgdb8250_pre_exception_handler(void)
{
	if (!module_refcount(THIS_MODULE))
		try_module_get(THIS_MODULE);
}

static void kgdb8250_post_exception_handler(void)
{
	if (!kgdb_connected && module_refcount(THIS_MODULE))
		module_put(THIS_MODULE);
}

static void cleanup_kgdb8250(void)
{
	kgdb_unregister_io_module(&local_kgdb_io_ops);

	/* Clean up the irq and memory */
	free_irq(current_port->irq, current_port);

	if (kgdb8250_needs_request_mem_region)
		release_mem_region(current_port->mapbase,
				   8 << current_port->regshift);
	/* Hook up the serial port back to what it was previously
	 * hooked up to.
	 */
#if defined(CONFIG_SERIAL_8250) || defined(CONFIG_SERIAL_8250_MODULE)
	/* Give the port back to the 8250 driver. */
	serial8250_register_port(current_port);
#endif
}

module_init(kgdb_init_io);
module_exit(cleanup_kgdb8250);
#else				/* ! CONFIG_KGDB_8250_MODULE */
early_param("kgdb8250", kgdb8250_opt);
#endif				/* ! CONFIG_KGDB_8250_MODULE */
