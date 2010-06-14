/*
 * arch/arm/mach-omap2/omap24xx-uart.c
 *
 * Support functions for the OMAP24xx uart controller.
 *
 * Copyright (C) 2004-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/serial.h>
#include <linux/dma-mapping.h>

#include <asm/system.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/arch/clock.h>
#include <asm/arch/serial.h>
#include <asm/arch/omap24xx-uart.h>

#ifdef CONFIG_HS_SERIAL_SUPPORT
#include <linux/serial_8250.h>
#endif

#ifdef CONFIG_ARCH_OMAP34XX
//#define HSUART_LAT_CONST
#endif

#ifdef HSUART_LAT_CONST
static struct constraint_handle *constr_handle;
static struct constraint_id cnstr_id = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};
#endif

/* Enable SUPPORT_NON_DMA_MODE for Non DMA operation */
#define SUPPORT_NON_DMA_MODE

/* ----- debug defines ----------------------------------------------- */
/* Debug - four macros:
 * FN_IN, FN_OUT(value),D1,D2,D3 enabled based on log level
 */

/* Log level standard used here:
 * Log level 3 all messages
 * Log level 2 all entry-exit points
 * Log level 1 major messages
 * Log level 0 no messages
 */
#define OMAP24XX_UART_LOG_LEVEL 0
/* detail - 0 - no detail
 *          1 - function name
 *          2 - function name, line number
 * prefix is added to every log message
 */
#define OMAP24XX_UART_DETAIL    4

/* kernel log level*/
//#define OMAP24XX_UART_K_LOG_LEVEL KERN_DEBUG
#define OMAP24XX_UART_K_LOG_LEVEL

#if ( OMAP24XX_UART_DETAIL > 0 )
#define DL1 "%s "
#define DR1 ,__FUNCTION__
#define DEBUG
#else
#define DL1
#define DR1
#endif
#if ( OMAP24XX_UART_DETAIL > 1 )
#define DL2 "[%d] "
#define DR2 ,__LINE__
#else
#define DL2
#define DR2
#endif

#define D(format,...)\
	printk(OMAP24XX_UART_K_LOG_LEVEL DL1 DL2 format "\n" DR1 DR2, ## __VA_ARGS__)

#if (OMAP24XX_UART_LOG_LEVEL >= 1)
#define D1(ARGS...) D(ARGS)
#else
#define D1(ARGS...)
#endif
#if (OMAP24XX_UART_LOG_LEVEL >= 2)
#define D2(ARGS...) D(ARGS)
#else
#define D2(ARGS...)
#endif
#if (OMAP24XX_UART_LOG_LEVEL >= 3)
#define D3(ARGS...) D(ARGS)
#else
#define D3(ARGS...)
#endif

#if (OMAP24XX_UART_LOG_LEVEL >= 2)
#define FN_IN printk("%s Entry\n",__FUNCTION__);
#define FN_OUT(ARG) printk("%s[%d]:Exit(%d)\n",__FUNCTION__,__LINE__,ARG);
#else
#define FN_IN
#define FN_OUT(ARG)
#endif

#define CONSOLE_NAME    "console="
#define FREE		0
#define USED		1
#define MAX_BUF_SIZE	12000

inline u32 omap24xx_uart_base_v(int uart_no)
{
	static u32 addr_v[MAX_UARTS +1] = {
			IO_ADDRESS(OMAP_UART1_BASE),
			IO_ADDRESS(OMAP_UART2_BASE),
			IO_ADDRESS(OMAP_UART3_BASE),
	};

	return addr_v[uart_no];
}

EXPORT_SYMBOL(omap24xx_uart_base_v);

inline u32 omap24xx_uart_base_p(int uart_no)
{
	static u32 addr_p[MAX_UARTS +1] = {
		OMAP_UART1_BASE, OMAP_UART2_BASE, OMAP_UART3_BASE,
	};

	return addr_p[uart_no];
}

EXPORT_SYMBOL(omap24xx_uart_base_p);


/* forward declartation */
static void uart_rx_dma_callback(int lch, u16 ch_status, void *data);
static void omap24xx_timeout_isr(unsigned long uart_no);

/* structure for storing UART DMA info */
struct omap24xx_uart {
	u8 uart_no;
	int rx_dma_channel;
	int tx_dma_channel;
	dma_addr_t rx_buf_dma_phys;	/* Physical adress of RX DMA buffer */
	dma_addr_t tx_buf_dma_phys;	/* Physical adress of TX DMA buffer */
	void *rx_buf_dma_virt;	/* Virtual adress of RX DMA buffer */
	void *tx_buf_dma_virt;	/* Virtual adress of TX DMA buffer */
	u8 tx_buf_state;
	u8 rx_buf_state;
	spinlock_t uart_lock;
	int in_use;

	/* timer to poll variable size release */
	struct timer_list timer;
	unsigned int timer_rate;

	u32 timeout;
	u32 timer_active;

};

static int get_uart_clocks(struct clk **uarti, struct clk **uartf, u8 uart_no);
static int uart_irq[MAX_UARTS + 1] =
    { INT_24XX_UART1_IRQ, INT_24XX_UART2_IRQ, INT_24XX_UART3_IRQ };
static u8 uart_dma_rx[MAX_UARTS + 1] =
    { OMAP24XX_DMA_UART1_RX, OMAP24XX_DMA_UART2_RX, OMAP24XX_DMA_UART3_RX };
static u8 uart_dma_tx[MAX_UARTS + 1] =
    { OMAP24XX_DMA_UART1_TX, OMAP24XX_DMA_UART2_TX, OMAP24XX_DMA_UART3_TX };
static struct omap24xx_uart ui[MAX_UARTS + 1];
static struct uart_callback uart_cb[MAX_UARTS + 1];
static u8 uart_mode[MAX_UARTS + 1];

/**
 * brief get_uart_clocks
 * used to enable/disable UART clocks.
 * param *uarti
 * param *uartf
 * param uart_no
 *
 * return
 */
static int get_uart_clocks(struct clk **uarti, struct clk **uartf, u8 uart_no)
{
	static const char *iclk_name[MAX_UARTS +1] =
				{"uart1_ick", "uart2_ick", "uart3_ick"};
	static const char *fclk_name[MAX_UARTS +1] =
				{"uart1_fck", "uart2_fck", "uart3_fck"};

	/* Cached clock handles */
	static struct clk *iclk[MAX_UARTS +1] = { 0 };
	static struct clk *fclk[MAX_UARTS +1] = { 0 };

	if (uart_no > MAX_UARTS)
		return -ENOENT;

	/* Do we have cached clocks? */
	if (iclk[uart_no] && fclk[uart_no]) {
		*uarti = iclk[uart_no];
		*uartf = fclk[uart_no];
		return 0;
	}

	/* No cache, need to look them up. */
	*uarti = clk_get(NULL, iclk_name[uart_no]);
	if (*uarti < 0) {
		printk(KERN_ERR "UART: %s: No entry in the clock table\n",
		       iclk_name[uart_no]);
		return -ENOENT;
	}

	*uartf = clk_get(NULL, fclk_name[uart_no]);
	if (*uartf < 0) {
		printk(KERN_ERR "UART: %s: No entry in the clock table\n",
		       fclk_name[uart_no]);
		return -ENOENT;
	}
	
	/* Update cache */
	iclk[uart_no] = *uarti;
	fclk[uart_no] = *uartf;
	return 0;
}

int omap24xx_uart_clk_enable(int uart_no)
{
	struct clk *iclk;
	struct clk *fclk;
	int rc;

	rc = get_uart_clocks(&iclk, &fclk, uart_no);
	if (unlikely(rc))
		return rc;

	clk_enable(iclk);
	clk_enable(fclk);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_clk_enable);

int omap24xx_uart_clk_disable(int uart_no)
{
	struct clk *iclk;
	struct clk *fclk;
	int rc;

	rc = get_uart_clocks(&iclk, &fclk, uart_no);
	if (unlikely(rc))
		return rc;

	clk_disable(iclk);
	clk_disable(fclk);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_clk_disable);

/* Handler for Errata 2.15.
 * Set ForceIdle on UARTs 1/2 so they will not prevent CORE retention.
 * NOTE:
 *   This function is either called by suspend or cpuidle with IRQs disabled.
 */
static u32 uart_save_sysconfig[MAX_UARTS +1];

void omap24xx_uart_set_force_idle(void)
{
	u32 uart_base;

	omap24xx_uart_clk_enable(0);
	uart_base = omap24xx_uart_base_v(0);
	uart_save_sysconfig[0] = readb(uart_base + REG_SYSC);
	writeb(0x0001, uart_base + REG_SYSC); /* ForceIdle */
	omap24xx_uart_clk_disable(0);

	omap24xx_uart_clk_enable(1);
	uart_base = omap24xx_uart_base_v(1);
	uart_save_sysconfig[1] = readb(uart_base + REG_SYSC);
	writeb(0x0001, uart_base + REG_SYSC); /* ForceIdle */
	omap24xx_uart_clk_disable(1);
}

EXPORT_SYMBOL(omap24xx_uart_set_force_idle);

void omap24xx_uart_clr_force_idle(void)
{
	u32 uart_base;

	omap24xx_uart_clk_enable(0);
	uart_base = omap24xx_uart_base_v(0);
	writeb(uart_save_sysconfig[0], uart_base + REG_SYSC); /* ForceIdle */
	omap24xx_uart_clk_disable(0);

	omap24xx_uart_clk_enable(1);
	uart_base = omap24xx_uart_base_v(1);
	writeb(uart_save_sysconfig[1], uart_base + REG_SYSC); /* ForceIdle */
	omap24xx_uart_clk_disable(1);
}

EXPORT_SYMBOL(omap24xx_uart_clr_force_idle);

/*
 * omap24xxx_timeout_isr
 * Looks at DMA destination register and calls receive
 * callback if the destination register is the same on
 * two timer isr.
 */
static void
omap24xx_timeout_isr(unsigned long uart_no)
{
	u32	w = 0;
	int	lch;
	int	curr_pos;
	static int	prev_pos = 0;
	struct	omap24xx_uart *hsuart = &ui[uart_no];
	FN_IN;
	lch = hsuart->rx_dma_channel;

	curr_pos = omap_get_dma_dst_pos(lch);
	if ((curr_pos == prev_pos) && (curr_pos != hsuart->rx_buf_dma_phys)) {
		omap_stop_dma(lch);
		w = OMAP_DMA_CSR_REG(lch);

		uart_rx_dma_callback(lch, w, uart_cb[hsuart->uart_no].dev);

		prev_pos = 0;
	}
	else {
		prev_pos = curr_pos;
		mod_timer(&hsuart->timer, jiffies +
				msecs_to_jiffies(hsuart->timeout));

	}
	FN_OUT(0);
}

/**
 * brief omap24xx_uart_isr
 * Identifes the source of interrupt(UART1, UART2, UART3)
 * and reads respective IIR register data.
 * Sends IIR data to the user driver.
 * param irq
 * param dev_id
 * param regs
 */
static irqreturn_t
omap24xx_uart_isr(int irq, void *dev_id)
{
	u8 iir_data; /* regular IRQ */
	u8 ssr_data; /* wake-up IRQ */
	u32 uart_base;
	int uart_no;
	FN_IN;
	switch (irq) {
	case INT_24XX_UART1_IRQ:
		uart_base = omap24xx_uart_base_v(UART1);
		uart_no = UART1;
		break;
	case INT_24XX_UART2_IRQ:
		uart_base = omap24xx_uart_base_v(UART2);
		uart_no = UART2;
		break;
	case INT_24XX_UART3_IRQ:
		uart_base = omap24xx_uart_base_v(UART3);
		uart_no = UART3;
		break;

	default:
		printk("UART interrupt from unknown source\n");
		return IRQ_NONE;
	}

	/* We got an inerrupt for our UART. At this time we do not know if it
	 * was a normal interrupt or a wake-up interrupt. Therefore we need to
	 * enable the clocks first as they might be off if this is a wake-up.
	 */
	omap24xx_uart_clk_enable(uart_no);

	/* Check for wake-up IRQ.
	 */
	ssr_data = readb(uart_base + REG_SSR);

	/* if IIR indicates RHR, start dma */
	iir_data = readb(uart_base + REG_IIR);

	uart_cb[uart_no].int_callback(iir_data, ssr_data, dev_id);

	omap24xx_uart_clk_disable(uart_no);

	FN_OUT(0);
	return IRQ_HANDLED;
}

/**
 * brief uart_rx_dma_callback
 *
 * param lch
 * param ch_status
 * param data
 */
static void uart_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	int uart_no = 0;
	FN_IN;

	if (lch == ui[UART1].rx_dma_channel)
		uart_no = UART1;
	else if (lch == ui[UART2].rx_dma_channel)
		uart_no = UART2;
	else if (lch == ui[UART3].rx_dma_channel)
		uart_no = UART3;

	del_timer(&ui[uart_no].timer);
	ui[uart_no].timer_active = 0;

	uart_cb[uart_no].uart_rx_dma_callback(lch, ch_status, data);
	ui[uart_no].rx_buf_state = FREE;
	FN_OUT(0);
}

/**
 * brief uart_tx_dma_callback
 *
 * param lch
 * param ch_status
 * param data
 */
static void uart_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	FN_IN;
	if (lch == ui[UART1].tx_dma_channel) {
		uart_cb[UART1].uart_tx_dma_callback(lch, ch_status, data);
		ui[UART1].tx_buf_state = FREE;
	} else if (lch == ui[UART2].tx_dma_channel) {
		uart_cb[UART2].uart_tx_dma_callback(lch, ch_status, data);
		ui[UART2].tx_buf_state = FREE;
	} else if (lch == ui[UART3].tx_dma_channel) {
		uart_cb[UART3].uart_tx_dma_callback(lch, ch_status, data);
		ui[UART3].tx_buf_state = FREE;
	}
	FN_OUT(0);
}

/**
 * brief omap24xx_uart_get_parms
 * reads requested register data
 * param uart_no
 * param data
 * param reg
 * param lcr_mode
 *
 * return
 */
int omap24xx_uart_get_parms(int uart_no, u8 * data, u8 reg, u8 lcr_mode)
{
	u32 uart_base;
	u8  lcr_data;
	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		FN_OUT(EPERM);
		return -EPERM;
	}

	uart_base = omap24xx_uart_base_v(uart_no);
	lcr_data = readb(uart_base + REG_LCR);

	outb(lcr_mode, uart_base + REG_LCR);
	*data = readb(uart_base + reg);
	/* Restore LCR data */
	outb(lcr_data, uart_base + REG_LCR);

	return 0;
	FN_OUT(0);
}

EXPORT_SYMBOL(omap24xx_uart_get_parms);

/**
 * brief omap24xx_uart_set_parms
 * writes values into requested UART register
 * param uart_no
 * param uart_set
 *
 * return
 */
int omap24xx_uart_set_parms(int uart_no, struct uart_setparm *uart_set)
{
	u32 uart_base;
	u8  lcr_data;
	FN_IN;

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		FN_OUT(EPERM);
		return -EPERM;
	}
	spin_lock(&(ui[uart_no].uart_lock));

	uart_base = omap24xx_uart_base_v(uart_no);
	lcr_data = readb(uart_base + REG_LCR);

	outb(uart_set->lcr, uart_base + REG_LCR);
	outb(uart_set->reg_data, uart_base + uart_set->reg);

	/* Restore LCR data */
	outb(lcr_data, uart_base + REG_LCR);

	spin_unlock(&(ui[uart_no].uart_lock));
	return 0;
	FN_OUT(0);
}

EXPORT_SYMBOL(omap24xx_uart_set_parms);

/**
 * brief omap24xx_uart_get_speed
 * reads DLL and DLH register values and
 * calculates UART speed.
 * param uart_no
 * param speed
 *
 * return
 */
int omap24xx_uart_get_speed(int uart_no, int *speed)
{
	u32 uart_base;
	u8  reg;
	u8  dll, dlh;
	u16 divisor;

	FN_IN;

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	uart_base = omap24xx_uart_base_v(uart_no);

	spin_lock(&(ui[uart_no].uart_lock));
	reg = LCR_MODE2;
	outb(reg, uart_base + REG_LCR);
	dll = readb(uart_base + REG_DLL);
	dlh = readb(uart_base + REG_DLH);

	divisor = (dlh << 8) + dll;

	if (!divisor) {
		printk(KERN_WARNING "UART: DLL and DLH read error\n");
		return -EPERM;
	}

	*speed = (BASE_CLK) / 16 * divisor;
	spin_unlock(&(ui[uart_no].uart_lock));
	return 0;
	FN_OUT(0);
}

EXPORT_SYMBOL(omap24xx_uart_get_speed);

/**
 * brief omap24xx_uart_set_speed
 * used to set the UART speed.
 * param uart_no
 * param speed
 *
 * return
 */
int omap24xx_uart_set_speed(int uart_no, int speed)
{
	u32 uart_base;
	u8 lcr_data, mdr1_data;
	int divisor;
	FN_IN;

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	uart_base = omap24xx_uart_base_v(uart_no);

	spin_lock(&(ui[uart_no].uart_lock));
	/* Disable UART before changing the clock speed - TRM - 18-52 */
	mdr1_data = readb(uart_base + REG_MDR1);
	outb(UART_DISABLE, uart_base + REG_MDR1);

	/* Enable access to DLL and DLH registers */
	lcr_data = readb(uart_base + REG_LCR);
	outb(LCR_MODE2, uart_base + REG_LCR);

	/* Only UART3 supports IrDA mode */
	if ((uart_no == UART3) && (uart_mode[uart_no] == IRDA_MODE)) {
		if (speed <= IR_SIR_SPEED) {
			D1("SIR Mode : Changing UART speed to %d...", speed);
			divisor = BASE_CLK / (16 * speed);
			outb(divisor & 0xFF, uart_base + REG_DLL);
			outb(divisor >> 8, uart_base + REG_DLH);
			mdr1_data &= MODE_SELECT_MASK;
			mdr1_data |= UART_SIR_MODE;
			outb(mdr1_data, uart_base + REG_MDR1);
			printk("Done\n");
		} else if (speed <= IR_MIR_SPEED) {
			D1("MIR Mode : Changing UART speed to %d...", speed);
			divisor = BASE_CLK / (41 * speed);
			outb(divisor & 0xFF, uart_base + REG_DLL);
			outb(divisor >> 8, uart_base + REG_DLH);
			mdr1_data &= MODE_SELECT_MASK;
			mdr1_data |= UART_MIR_MODE;
			outb(mdr1_data, uart_base + REG_MDR1);
			D1("Done\n");
		} else if (speed <= IR_FIR_SPEED) {
			printk("FIR Mode : Changing UART speed to %d...", speed);
			mdr1_data &= MODE_SELECT_MASK;
			mdr1_data |= UART_FIR_MODE;
			outb(mdr1_data, uart_base + REG_MDR1);
			D1("Done\n");
		} else {
			printk(KERN_ERR
			       "UART: Requested speed is not supported\n");
			goto exit_path1;
		}
	} else if (uart_mode[uart_no] == UART_MODE) {
		if (speed <= UART_16X_SPEED) {
			D1("16X Mode : Changing UART speed to %d...\n", speed);
			divisor = BASE_CLK / (16 * speed);
			outb(divisor & 0xFF, uart_base + REG_DLL);
			outb(divisor >> 8, uart_base + REG_DLH);
			mdr1_data &= MODE_SELECT_MASK;
			mdr1_data |= UART_16X_MODE;
			outb(mdr1_data, uart_base + REG_MDR1);
			D1("Done\n");
		} else if (speed <= UART_13X_SPEED) {
			D1("13X Mode : Changing UART speed to %d...\n", speed);
			divisor = BASE_CLK / (13 * speed);
			outb(divisor & 0xFF, uart_base + REG_DLL);
			outb(divisor >> 8, uart_base + REG_DLH);
			mdr1_data &= MODE_SELECT_MASK;
			mdr1_data |= UART_13X_MODE;
			outb(mdr1_data, uart_base + REG_MDR1);
			D1("Done\n");
		} else {
			printk(KERN_ERR
			       "UART: Requested speed is not supported\n");
			goto exit_path1;
		}
	} else if (uart_mode[uart_no] == UART_AUTOBAUD_MODE) {
		D1("UART%d 16X Auto baud Mode\n", uart_no);
		mdr1_data &= MODE_SELECT_MASK;
		mdr1_data |= UART_16XAUTO_MODE;
		outb(mdr1_data, uart_base + REG_MDR1);
	} else if (uart_mode[uart_no] == CIR_MODE) {
		D1(KERN_INFO "UART%d CIR Mode\n", uart_no);
		mdr1_data &= MODE_SELECT_MASK;
		mdr1_data |= UART_CIR_MODE;
		outb(mdr1_data, uart_base + REG_MDR1);
	} else {
		printk(KERN_ERR
		       "UART: Invalid parameters for UART speed change\n");
		goto exit_path1;
	}
	/* restore LCR values */
	outb(lcr_data, uart_base + REG_LCR);
	spin_unlock(&(ui[uart_no].uart_lock));
	FN_OUT(0);
	return 0;

      exit_path1:
	/* Restore LCR and MDR1 regisgters to original value */
	outb(mdr1_data, uart_base + REG_MDR1);
	outb(lcr_data, uart_base + REG_LCR);
	spin_unlock(&(ui[uart_no].uart_lock));
	FN_OUT(EPERM);
	return -EPERM;
}

EXPORT_SYMBOL(omap24xx_uart_set_speed);

/**
 * brief omap24xx_uart_config
 *  configures the requested UART
 * param uart_no
 * param uartcfg
 *
 * return
 */
int omap24xx_uart_config(int uart_no, struct uart_config *uartcfg)
{
	u32 uart_base;
	u8  reg_data;
	FN_IN;

	if (in_interrupt())
		BUG();

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	/* Start clock for requested UART */
	if (omap24xx_uart_clk_enable(uart_no)) {
		printk(KERN_ERR "UART clock configuration error\n");
		return -ENOENT;
	}

	uart_base = omap24xx_uart_base_v(uart_no);

	spin_lock(&(ui[uart_no].uart_lock));

	uart_mode[uart_no] = uartcfg->mode;

	/* Put UART in reset mode */
	reg_data = UART_DISABLE;
	outb(reg_data, uart_base + REG_MDR1);

	/* Clear DLL and DLH */
	reg_data = LCR_MODE2;
	outb(reg_data, uart_base + REG_LCR);
	reg_data = 0;
	outb(reg_data, uart_base + REG_DLL);
	reg_data = 0;
	outb(reg_data, uart_base + REG_DLH);

	reg_data = 0;
	outb(reg_data, uart_base + REG_SCR);

	reg_data = LCR_MODE3;
	outb(reg_data, uart_base + REG_LCR);
	reg_data = (uartcfg->efr);
	outb(reg_data, uart_base + REG_EFR);

	reg_data = LCR_MODE2;
	outb(reg_data, uart_base + REG_LCR);
	/* enable TCR and TLR Registers */
	reg_data = ENABLE_TCR_TLR;
	outb(reg_data, uart_base + REG_MCR);

	reg_data = (uartcfg->tlr);
	outb(reg_data, uart_base + REG_TLR);

	reg_data = (uartcfg->lcr);
	outb(reg_data, uart_base + REG_LCR);

	reg_data = (uartcfg->fcr);
	outb(reg_data, uart_base + REG_FCR);

	/* disable access to TCR and TLR registers */
	reg_data = DISABLE_TCR_TLR;
	outb(reg_data, uart_base + REG_MCR);

	reg_data = (uartcfg->scr) |
		(BIT_SCR_TX_TRIG_GRANU1_M | BIT_SCR_RX_TRIG_GRANU1_M);
	outb(reg_data, uart_base + REG_SCR);

	reg_data = (uartcfg->mdr1);
	outb(reg_data, uart_base + REG_MDR1);

	reg_data = (uartcfg->mdr2);
	outb(reg_data, uart_base + REG_MDR2);

	reg_data = (uartcfg->acreg);
	outb(reg_data, uart_base + REG_ACREG);

	reg_data = (uartcfg->ier);
	outb(reg_data, uart_base + REG_IER);

	if (uart_mode[uart_no] == IRDA_MODE) {
		reg_data = (uartcfg->rxfll);
		outb(reg_data, uart_base + REG_RXFLL);
		reg_data = (uartcfg->rxflh);
		outb(reg_data, uart_base + REG_RXFLH);
	}
	readb(uart_base + REG_RESUME);
	spin_unlock(&(ui[uart_no].uart_lock));

	omap24xx_uart_clk_disable(uart_no);

	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_config);

/**
 * brief omap24xx_uart_stop
 * stops/resets requested UART.
 * param uart_no
 *
 * return
 */
int omap24xx_uart_stop(int uart_no)
{
	u32 uart_base;
	u8  reg_data;

	FN_IN;

	if (in_interrupt())
		BUG();

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	uart_base = omap24xx_uart_base_v(uart_no);

	spin_lock(&(ui[uart_no].uart_lock));
	/* Put UART in reset mode */
	reg_data = (7 << BIT_MDR1_MODE_SELECT);
	outb(reg_data, uart_base + REG_MDR1);

	/* Clear DLL and DLH */
	reg_data = LCR_MODE2;
	outb(reg_data, uart_base + REG_LCR);
	reg_data = 0;
	outb(reg_data, uart_base + REG_DLL);
	reg_data = 0;
	outb(reg_data, uart_base + REG_DLH);

	/* Disable requested UART interrupts */
	reg_data = 0;
	outb(reg_data, uart_base + REG_IER);

	/* Stop DMA channels */
	if( ui[uart_no].rx_dma_channel != -1 ) {
		omap_stop_dma(ui[uart_no].rx_dma_channel);
		ui[uart_no].rx_buf_state = FREE;
	}

	if( ui[uart_no].tx_dma_channel != -1 ) {
		omap_stop_dma(ui[uart_no].tx_dma_channel);
		ui[uart_no].tx_buf_state = FREE;
	}

	spin_unlock(&(ui[uart_no].uart_lock));
	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_stop);

/**
 * brief omap24xx_uart_rx
 *  Copies data from DMA buffer to user
 *  driver buffer.
 * param uart_no
 * param data
 * param len
 *
 * return
 */
int omap24xx_uart_rx(int uart_no, void *data, int *len)
{
	unsigned long flags;

	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}
	spin_lock_irqsave(&(ui[uart_no].uart_lock), flags);
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		*len = omap_get_dma_dst_pos(ui[uart_no].rx_dma_channel);
		*len -= ui[uart_no].rx_buf_dma_phys;
		memcpy(data, ui[uart_no].rx_buf_dma_virt, *len);
		/* DMA data is copied to user driver buffer,
		 * now it is safe to move rx_buf_state to free.
		 */
		ui[uart_no].rx_buf_state = FREE;
	} else {
#ifdef SUPPORT_NON_DMA_MODE
		int count = 0;
		/* NON DMA Mode */
		u32 uart_base = omap24xx_uart_base_p(uart_no);
		u8 lsr_data, rhr_data;
		unsigned char *txaddr = data;

		lsr_data = readb(uart_base + REG_LSR);
		if (uart_mode[uart_no] == IRDA_MODE) {
			if ((lsr_data & 0x01)) {
				printk(KERN_ERR "Receive FIFO Empty\n");
				spin_unlock_irqrestore(
					&(ui[uart_no].uart_lock), flags);
				return -EIO;
			}
			while (!(lsr_data & 0x01)) {
				rhr_data = readb(uart_base + REG_RHR);
				txaddr[count] = rhr_data;
				/* Check if RHR contains last byte of frame */
				if ((lsr_data & 0x20) || count >= MAX_BUF_SIZE)
					break;
				count++;
				lsr_data = readb(uart_base + REG_LSR);
			}
		} else if (uart_mode[uart_no] == UART_MODE) {
			if (!(lsr_data & 0x01)) {
				printk(KERN_ERR "Receive FIFO Empty\n");
				spin_unlock_irqrestore(
					&(ui[uart_no].uart_lock), flags);
				return -EIO;
			}
			while ((lsr_data & 0x01)) {
				rhr_data = readb(uart_base + REG_RHR);
				txaddr[count] = rhr_data;
				count++;
				if (count > FIFO_SIZE)
					break;
				lsr_data = readb(uart_base + REG_LSR);
			}
		} else if (uart_mode[uart_no] == CIR_MODE) {
			if ((lsr_data & 0x01)) {
				printk(KERN_ERR "Receive FIFO Empty\n");
				spin_unlock_irqrestore(
					&(ui[uart_no].uart_lock), flags);
				return -EIO;
			}
			while (!(lsr_data & 0x01)) {
				rhr_data = readb(uart_base + REG_RHR);
				txaddr[count] = rhr_data;
				count++;
				if (count > FIFO_SIZE)
					break;
				/* Check if frame reception is complete */
				if (lsr_data & 0x02)
					break;
				lsr_data = readb(uart_base + REG_LSR);
			}
		}
#endif
	}
	spin_unlock_irqrestore(&(ui[uart_no].uart_lock), flags);
	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_rx);

/**
 * brief omap24xx_uart_tx
 *  copies data from client driver buffer
 *  to DMA buffer.
 * param uart_no
 * param data
 * param size
 *
 * return
 */
int omap24xx_uart_tx(int uart_no, void *data, int size)
{
	unsigned long flags;

	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}
	if (unlikely(size < 0 || size > uart_cb[uart_no].tx_buf_size)) {
		D3(KERN_INFO "Error...Invalid buffer size!!!!\n");
		return -EPERM;
	}
	spin_lock_irqsave(&(ui[uart_no].uart_lock), flags);
	D3("omap24xx_uart_tx : %s\n", data);
	D3("omap24xx_uart_tx : %d\n", size);
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		memcpy(ui[uart_no].tx_buf_dma_virt, data, size);
	} else {
#ifdef SUPPORT_NON_DMA_MODE
		/* NON DMA Mode */
		u32 uart_base = omap24xx_uart_base_p(uart_no);
		u8 lsr_data;
		int timeout = 0;
		int count = 0;
		unsigned char *txaddr = data;

		outb(LCR_MODE1, uart_base + REG_LCR);
		lsr_data = readb(uart_base + REG_LSR);
		if (uart_mode[uart_no] == UART_MODE) {
			if (!(lsr_data & 0x20)) {
				printk(KERN_ERR "UART: Transmit FIFO Full\n");
				spin_unlock_irqrestore(
					&(ui[uart_no].uart_lock), flags);
				return -EIO;
			}
			while (size <= 0) {
				lsr_data = readb(uart_base + REG_LSR);
				if (lsr_data & 0x20) {
					outb(txaddr[count],
					       uart_base + REG_THR);
					count++;
					size--;
				} else
					timeout++;
				if (timeout > 10000) {
					printk(KERN_ERR "UART: Tx timeout\n");
					spin_unlock_irqrestore(
					      &(ui[uart_no].uart_lock), flags);
					return -EIO;
				}
			}
		} else if (uart_mode[uart_no] == (IRDA_MODE | CIR_MODE)) {
			if (!(lsr_data & 0x80)) {
				printk(KERN_ERR "UART: Transmit FIFO Full\n");
				spin_unlock_irqrestore(
					&(ui[uart_no].uart_lock), flags);
				return -EIO;
			}
			while (size <= 0) {
				lsr_data = readb(uart_base + REG_LSR);
				if (lsr_data & 0x80) {
					outb(txaddr[count],
					       uart_base + REG_THR);
					count++;
					size--;
				} else
					timeout++;
				if (timeout > 10000) {
					printk(KERN_ERR "UART: Tx timeout\n");
					spin_unlock_irqrestore(
					      &(ui[uart_no].uart_lock), flags);
					return -EIO;
				}
			}

		}
#endif
	}
	spin_unlock_irqrestore(&(ui[uart_no].uart_lock), flags);
	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_tx);

/**
 * brief omap24xx_uart_start_tx
 *  Starts transferring data from DMA tx channel
 * param uart_no
 * param size
 *
 * return
 */
int omap24xx_uart_start_tx(int uart_no, int size)
{
	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	if (ui[uart_no].tx_buf_state == USED) {
		D3("Error : UART DMA buffer is not free...!!!\n");
		return -EAGAIN;
	}
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		u32 uart_base = omap24xx_uart_base_p(uart_no);

		ui[uart_no].tx_buf_state = USED;
		omap_set_dma_dest_params(ui[uart_no].tx_dma_channel,
						0,			// dest_port is only for OMAP1
					 OMAP_DMA_AMODE_CONSTANT, uart_base, 0,
					 0);
		omap_set_dma_src_params(ui[uart_no].tx_dma_channel,
						0,			// src_port is only for OMAP1
					OMAP_DMA_AMODE_POST_INC,
					ui[uart_no].tx_buf_dma_phys, 0, 0);
		omap_set_dma_transfer_params(ui[uart_no].tx_dma_channel,
					     OMAP_DMA_DATA_TYPE_S8, size, 1,
					     OMAP_DMA_SYNC_ELEMENT,
					     uart_dma_tx[uart_no], 0);
		omap_start_dma(ui[uart_no].tx_dma_channel);
	}
	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_start_tx);

/**
 * brief omap24xx_uart_start_rx
 *  Starts receiving data from DMA rx channel
 * param uart_no
 * param size
 *
 * return
 */
int omap24xx_uart_start_rx(int uart_no, int size)
{
	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}
	if (ui[uart_no].rx_buf_state == USED) {
		D3("Error : UART DMA buffer is not free...!!!\n");
		return -EAGAIN;
	}
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		u32 uart_base = omap24xx_uart_base_p(uart_no);

		ui[uart_no].rx_buf_state = USED;
		omap_set_dma_src_params(ui[uart_no].rx_dma_channel,
					0,			// src_port is only for OMAP1
					OMAP_DMA_AMODE_CONSTANT, uart_base, 0,
					0);
		omap_set_dma_dest_params(ui[uart_no].rx_dma_channel,
					0,			// dest_port is only for OMAP1
					 OMAP_DMA_AMODE_POST_INC,
					 ui[uart_no].rx_buf_dma_phys, 0, 0);
		omap_set_dma_transfer_params(ui[uart_no].rx_dma_channel,
					     OMAP_DMA_DATA_TYPE_S8, size, 1,
					     OMAP_DMA_SYNC_ELEMENT,
					     uart_dma_rx[uart_no], 0);
		OMAP_DMA_CDAC_REG(ui[uart_no].rx_dma_channel) =
						ui[uart_no].rx_buf_dma_phys;

		if (ui[uart_no].timeout !=0) {
			/* Enable RHR interrupt. */
			uart_base = omap24xx_uart_base_v(uart_no);
			outb(inb(uart_base+REG_IER) | BIT_IER_RHR_IT_M,
					uart_base+REG_IER);
		} else {
			/* Start dma */
			omap_start_dma(ui[uart_no].rx_dma_channel);
			ui[uart_no].rx_buf_state = USED;
		}
	}

	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_start_rx);

/**
 * brief omap24xx_uart_stop_rx
 *
 * param uart_no
 *
 * return
 */
int omap24xx_uart_stop_rx(int uart_no)
{
	FN_IN;

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		omap_stop_dma(ui[uart_no].rx_dma_channel);
	}
	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_stop_rx);

/**
 * brief omap24xx_uart_stop_tx
 *
 * param uart_no
 *
 * return
 */
int omap24xx_uart_stop_tx(int uart_no)
{
	FN_IN;

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		omap_stop_dma(ui[uart_no].tx_dma_channel);
	}
	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_stop_tx);

/**
 * brief omap24xx_uart_interrupt
 *  used to enable/disable
 *  UART interrupts.
 * param uart_no
 * param enable
 *
 * return
 */
int omap24xx_uart_interrupt(int uart_no, int enable)
{
	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -ENXIO;
	}

	if (enable)
		enable_irq(uart_irq[uart_no]);
	else
		disable_irq(uart_irq[uart_no]);

	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_interrupt);

/**
 * brief omap24xx_uart_reset
 * resets the UART
 * param uart_no
 */
int omap24xx_uart_reset(int uart_no)
{
	unsigned long flags;
	u32 uart_base

	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -ENXIO;
	}

	uart_base = omap24xx_uart_base_v(uart_no);

	spin_lock_irqsave(&(ui[uart_no].uart_lock), flags);
	/* UART reset sequence */
	outb(LCR_MODE3, uart_base + REG_LCR);
	outb(BIT_EFR_ENHANCED_EN_M, uart_base + REG_EFR);
	outb(LCR_MODE1, uart_base + REG_LCR);
	outb(DISABLE, uart_base + REG_IER);
	outb(DISABLE, uart_base + REG_MCR);
	outb(DISABLE, uart_base + REG_SCR);
	outb(UART_DISABLE, uart_base + REG_MDR1);
	spin_unlock_irqrestore(&(ui[uart_no].uart_lock), flags);

	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_reset);

/**
 * brief omap24xx_uart_request
 * Allocates requested UART.
 * param uart_no
 * param uart_cback
 */
int omap24xx_uart_request(int uart_no, struct uart_callback *uart_cback)
{
	int err;
	u32 uart_base;
	u8  sysc_val;
	
	FN_IN;

	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d \n", uart_no);
		return -EPERM;
	}

	spin_lock(&(ui[uart_no].uart_lock));
	if (ui[uart_no].in_use) {
		printk(KERN_ERR "UART: Requested UART is not available\n");
		spin_unlock(&(ui[uart_no].uart_lock));
		FN_OUT(EACCES);
		return -EACCES;
	}
	ui[uart_no].in_use = 1;
	spin_unlock(&(ui[uart_no].uart_lock));

	omap24xx_uart_clk_enable(uart_no);

	uart_base = omap24xx_uart_base_v(uart_no);
	/* Setting UART to NoIdle */
	sysc_val = readb(uart_base + REG_SYSC);
	sysc_val &= ~(0x3 << 3);
	sysc_val |=  (0x1 << 3);
	outb(sysc_val, uart_base + REG_SYSC);

	omap24xx_uart_clk_disable(uart_no);

#ifdef CONFIG_HS_SERIAL_SUPPORT
	serial8250_unregister_port(uart_no);
#else
	printk(KERN_ERR "UART: CONFIG_HS_SERIAL_SUPPORT is not enabled "
					"during configuration\n");
	return -EPERM;
#endif

	if ((uart_cback->txrx_req_flag == TXRX)
	    || (uart_cback->txrx_req_flag == RX_ONLY)) {
		if (unlikely
		    (uart_cback->rx_buf_size < 1
		     || uart_cback->rx_buf_size > MAX_BUF_SIZE)) {
			printk(KERN_ERR "UART: Invalid RX buffer size\n");
			FN_OUT(0);
			return -EACCES;
		}
	}

	if ((uart_cback->txrx_req_flag == TXRX)
	    || (uart_cback->txrx_req_flag == TX_ONLY)) {
		if (unlikely
		    (uart_cback->tx_buf_size < 1
		     || uart_cback->tx_buf_size > MAX_BUF_SIZE)) {
			printk(KERN_ERR "UART: Invalid TX buffer size\n");
			FN_OUT(0);
			return -EACCES;
		}
	}
	if ((uart_cback->mode != UART_DMA_MODE)
	    && (uart_cback->mode != UART_NONDMA_MODE)) {
		printk(KERN_ERR "UART: Invalid data transfer mode\n");
		return -EACCES;
	}
#ifndef SUPPORT_NON_DMA_MODE
	if (uart_cback->mode == UART_NONDMA_MODE) {
		printk("NON DMA mode is supported only in Debug mode. "
			"Enable Debug mode in UART library\n");
		return -EAGAIN;
	}
#endif
	/* Store the requested mode of tx/rx - DMA/Non DMA */

	uart_cb[uart_no].mode = uart_cback->mode;
	uart_cb[uart_no].dev = uart_cback->dev;
	uart_cb[uart_no].int_callback = uart_cback->int_callback;
	uart_cb[uart_no].dev_name = uart_cback->dev_name;

	/* reset uart */
	omap24xx_uart_reset ( uart_no );

	if (request_irq
	    (uart_irq[uart_no], omap24xx_uart_isr, 0,
	     uart_cb[uart_no].dev_name, uart_cb[uart_no].dev)) {
		printk(KERN_ERR "UART: IRQ allocation failed\n");
		err = -EPERM;
		goto exit_path0;
	}

	spin_lock(&(ui[uart_no].uart_lock));
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		uart_cb[uart_no].mode = uart_cback->mode;
		uart_cb[uart_no].txrx_req_flag = uart_cback->txrx_req_flag;

		if ((uart_cb[uart_no].txrx_req_flag == TXRX)
		    || (uart_cb[uart_no].txrx_req_flag == RX_ONLY)) {
			/* Request DMA Channels for requested UART */
			err = omap_request_dma(
					uart_dma_rx[uart_no],
					"UART Rx DMA",
					(void *)uart_rx_dma_callback,
					uart_cb[uart_no].dev,
					&(ui[uart_no].rx_dma_channel));
			if (err) {
				printk(KERN_ERR "UART: Failed to get DMA Channels\n");
				goto exit_path1;
			}
			uart_cb[uart_no].rx_buf_size = uart_cback->rx_buf_size;
			uart_cb[uart_no].uart_rx_dma_callback =
					    uart_cback->uart_rx_dma_callback;
			ui[uart_no].rx_buf_dma_virt =
				    dma_alloc_coherent(
					NULL, uart_cb[uart_no].rx_buf_size,
					(dma_addr_t *)&(ui[uart_no].rx_buf_dma_phys), 0);
			if (!ui[uart_no].rx_buf_dma_virt) {
				printk(KERN_ERR
				       "UART: Failed to allocate DMA Rx Buffer\n");
				goto exit_path2;
			}
		}

		if ((uart_cb[uart_no].txrx_req_flag == TXRX)
		    || (uart_cb[uart_no].txrx_req_flag == TX_ONLY)) {
			err = omap_request_dma
			    (uart_dma_tx[uart_no],
			     "UART Tx DMA",
			     (void *)uart_tx_dma_callback,
			     uart_cb[uart_no].dev,
			     &(ui[uart_no].tx_dma_channel));
			if (err) {
				printk(KERN_ERR "UART: Failed to get DMA Channels\n");
				goto exit_path3;
			}
			uart_cb[uart_no].tx_buf_size = uart_cback->tx_buf_size;
			uart_cb[uart_no].uart_tx_dma_callback =
			    uart_cback->uart_tx_dma_callback;
			ui[uart_no].tx_buf_dma_virt =
			    dma_alloc_coherent(NULL,
					       uart_cb[uart_no].
					       tx_buf_size,
					       (dma_addr_t *)&(ui
						 [uart_no].tx_buf_dma_phys), 0);
			if (!ui[uart_no].tx_buf_dma_virt) {
				printk(KERN_ERR
				       "UART: Failed to allocate DMA Tx Buffer\n");
				goto exit_path4;
			}
		}
	}

	/* set timer to use for variable length receive. */
	if (uart_cb[uart_no].mode == UART_DMA_MODE) {

		ui[uart_no].timeout = 10;
		init_timer(&ui[uart_no].timer);
		ui[uart_no].timer.function = omap24xx_timeout_isr;
		ui[uart_no].timer.expires = jiffies + msecs_to_jiffies(ui[uart_no].timeout);
		ui[uart_no].timer.data = uart_no;

		ui[uart_no].uart_no = uart_no;
		ui[uart_no].timer_active = 0;
	}

	ui[uart_no].tx_buf_state = FREE;
	ui[uart_no].rx_buf_state = FREE;

	spin_unlock(&(ui[uart_no].uart_lock));

#ifdef HSUART_LAT_CONST
	constr_handle = constraint_get("hsuart", &cnstr_id);
	constraint_set(constr_handle, CO_LATENCY_MPUOFF_COREON);
#endif

	FN_OUT(0);
	return 0;

      exit_path4:
	omap_free_dma(ui[uart_no].rx_dma_channel);
	if ((uart_cb[uart_no].txrx_req_flag == TXRX)
	    || (uart_cb[uart_no].txrx_req_flag == TX_ONLY)) {
		uart_cb[uart_no].tx_buf_size = 0;
		uart_cb[uart_no].uart_tx_dma_callback = NULL;
	}

      exit_path3:
	if ((uart_cb[uart_no].txrx_req_flag == TXRX)
	    || (uart_cb[uart_no].txrx_req_flag == RX_ONLY)) {
		dma_free_coherent(uart_cb[uart_no].dev,
				  uart_cb[uart_no].rx_buf_size,
				  ui[uart_no].
				  rx_buf_dma_virt, ui[uart_no].rx_buf_dma_phys);
		uart_cb[uart_no].rx_buf_size = 0;
		uart_cb[uart_no].uart_rx_dma_callback = NULL;

	}
      exit_path2:
	if (uart_cb[uart_no].txrx_req_flag == TXRX
	    || (uart_cb[uart_no].txrx_req_flag == RX_ONLY))
		omap_free_dma(ui[uart_no].rx_dma_channel);
      exit_path1:
	free_irq(uart_irq[uart_no], uart_cb[uart_no].dev);
      exit_path0:
	uart_cb[uart_no].dev = NULL;
	uart_cb[uart_no].int_callback = NULL;
	ui[uart_no].in_use = 0;
	spin_unlock(&(ui[uart_no].uart_lock));

	FN_OUT(0);
	return err;
}

EXPORT_SYMBOL(omap24xx_uart_request);

/**
 * brief omap24xx_uart_release
 * release UART
 * param uart_no
 *
 * return
 */
int omap24xx_uart_release(int uart_no)
{
	u32 uart_base;
	u8 sysc_val;
	
	FN_IN;
	if (unlikely(uart_no < 0 || uart_no > MAX_UARTS)) {
		D3(KERN_INFO "C Bad uart id %d\n", uart_no);
		return -EPERM;
	}
	
	spin_lock(&(ui[uart_no].uart_lock));
	if (!ui[uart_no].in_use) {
		printk(KERN_WARNING "Requested UART is already in free state\n");
		spin_unlock(&(ui[uart_no].uart_lock));
		FN_OUT(EACCES);
		return -EACCES;
	}
	/* MOVE requested UART to free state.
	 * Free DMA channels.
	 * Free DMA buffers.
	 */

	/* Free IRQ, DMA channels and allocated memory. */
	free_irq(uart_irq[uart_no], uart_cb[uart_no].dev);

	if (uart_cb[uart_no].mode == UART_DMA_MODE) {
		omap_free_dma(ui[uart_no].rx_dma_channel);
		omap_free_dma(ui[uart_no].tx_dma_channel);

		dma_free_coherent(uart_cb[uart_no].dev,
				  uart_cb[uart_no].rx_buf_size,
				  ui[uart_no].
				  rx_buf_dma_virt, ui[uart_no].rx_buf_dma_phys);
		dma_free_coherent(uart_cb[uart_no].dev,
				  uart_cb[uart_no].tx_buf_size,
				  ui[uart_no].
				  tx_buf_dma_virt, ui[uart_no].tx_buf_dma_phys);
		uart_cb[uart_no].uart_tx_dma_callback = NULL;
		uart_cb[uart_no].uart_rx_dma_callback = NULL;
	}
	uart_cb[uart_no].int_callback = NULL;
	uart_cb[uart_no].dev_name = NULL;
	uart_cb[uart_no].dev = NULL;
	ui[uart_no].in_use = 0;

	ui[uart_no].rx_dma_channel = -1;
	ui[uart_no].tx_dma_channel = -1;
	ui[uart_no].rx_buf_dma_phys = 0;
	ui[uart_no].tx_buf_dma_phys = 0;
	ui[uart_no].rx_buf_dma_virt = NULL;
	ui[uart_no].tx_buf_dma_virt = NULL;
	ui[uart_no].rx_buf_state = 0;
	ui[uart_no].tx_buf_state = 0;

	spin_unlock(&(ui[uart_no].uart_lock));
	
#ifdef HSUART_LAT_CONST
	constraint_remove(constr_handle);
	constraint_put(constr_handle);
#endif

	omap24xx_uart_clk_enable(uart_no);

	uart_base = omap24xx_uart_base_v(uart_no);
	/* Setting UART to ForceIdle */
	sysc_val = readb(uart_base + REG_SYSC);
	sysc_val &= ~(0x3 << 3);
	outb(0x0, uart_base + REG_SYSC);

	omap24xx_uart_clk_disable(uart_no);
	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(omap24xx_uart_release);

/**
 * omap24xx_uart_set_timeout
 * Set the GP Timer Divisor.
 * time_ms is in milisecond
 */

int omap24xx_uart_set_timeout(int uart_no, u32 time_ms)
{
	if ((!ui[uart_no].timer_active) && (time_ms != 0)) {
		ui[uart_no].timeout = time_ms;
		return 0;
	}
	else {
		ui[uart_no].timeout = time_ms;
		return -EPERM;
	}
}

EXPORT_SYMBOL(omap24xx_uart_set_timeout);


/**
 * brief console_detect
 * Detect Console UART using command line parameter.
 * param str
 *
 * return
 */
int console_detect(char *str){
	extern char *saved_command_line;
	char *next, *start = NULL;
	int i;

	FN_IN;
	i = strlen(CONSOLE_NAME);
	next = saved_command_line;
	while ((next = strchr(next, 'c')) != NULL) {
		if (!strncmp(next, CONSOLE_NAME, i)) {
			start = next;
			break;
		} else {
			next++;
		}

	}
	if (!start)
		return -EPERM;
	i = 0;
	start = strchr(start, '=') + 1;
	while (*start != ',') {
		str[i++] = *start++;
		if (i > 6){
			printk("Invalid Console Name\n");
			return -EPERM;
		}
	}
	str[i] = '\0';

	FN_OUT(0);
	return 0;
}

/**
 * brief omap24xx_init_uart
 *  UART initialization
 * return
 */
static int __init omap24xx_uart_init(void)
{
	char str[7];
	int i;
	FN_IN;
	spin_lock_init(&(ui[UART1].uart_lock));
	spin_lock_init(&(ui[UART2].uart_lock));
	spin_lock_init(&(ui[UART3].uart_lock));

	/* initialize tx/rx channel */
	for (i = UART1; i <= UART3; i++) {
		ui[i].rx_dma_channel = -1;
		ui[i].tx_dma_channel = -1;
	}

	/* Reserve the console uart */
	if ( console_detect(str)){
		printk(KERN_ERR "UART: Invalid console parameter.\n");
		return -EPERM;
	}
	if (!strcmp(str, "ttyS0"))
		ui[UART1].in_use = 1;
	else if (!strcmp(str, "ttyS1"))
		ui[UART2].in_use = 1;
	else if (!strcmp(str, "ttyS2"))
		ui[UART3].in_use = 1;
	else
		printk(KERN_WARNING
		       "UART: Unable to recongnize Console UART (%s).\n", str);

	D3(" Console : %s\n", str);
	D3("\n ********** Initial UART States *************\n");
	D3(" UART1 : %d\n", ui[UART1].in_use);
	D3(" UART2 : %d\n", ui[UART2].in_use);
	D3(" UART3 : %d\n", ui[UART3].in_use);

	FN_OUT(0);
	return 0;
}

/**
 * brief omap24xx_init_exit
 * exit function.
 * return
 */
static void __exit omap24xx_uart_exit(void)
{
	printk("UART Library Exit\n");
}

module_init(omap24xx_uart_init);
module_exit(omap24xx_uart_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("UART Library");
MODULE_LICENSE("GPL");
