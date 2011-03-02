/*
 * drivers/misc/omap_misc_hsuart.c
 *
 * High speed UART driver
 *
 * Copyright (C) 2008-2009 Palm, Inc.
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
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/timex.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/hsuart.h>
#include <linux/hres_counter.h>

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
#include <linux/modem_activity.h>
#endif

#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/dma.h>

#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/omap24xx-uart.h>
#include <asm/arch/pm.h>

#undef	 MODDEBUG
#define  MODDEBUG  1

#ifdef	 MODDEBUG

#define  PDBG1(args...)   if( ctxt->dbg_level >= 1 ) \
                              printk(args)

#define  PDBG2(args...)   if( ctxt->dbg_level >= 2 ) \
                              printk(args)

#define  PDBG3(args...)   if( ctxt->dbg_level >= 3 ) \
                              printk(args)
#else

#define  PDBG1(args...)
#define  PDBG2(args...)
#define  PDBG3(args...)
#endif

#define  DRIVER      "omap_hsuart"
#define  DRIVER_VERSION    (0x100)

#define  HSUART_TX_LVL  1
#define  HSUART_RX_LVL  1

#define  HSUART_AUTO_RTS_START  (0x8) // 32 characters
#define  HSUART_AUTO_RTS_HALT   (0xF) // 60 characters

#define  HSUART_RX_FLOW_USER     (1 << 0) // throttled by user
#define  HSUART_RX_FLOW_SPACE    (1 << 1) // throttled by lack of space
#define  HSUART_RX_FLOW_IDLE     (1 << 2) // throttled by idle
#define  HSUART_RX_FLOW_STOP     (1 << 3) // throttled by start/stop
#define  HSUART_RX_FLOW_HAND     (1 << 4) // throttled by handshaking
#define  HSUART_RX_FLOW_BUG      (1 << 5) // throttled by corrupted data


// Timer things
#define  RX_TIMER_MIN_TIMEOUT   250  // usec

#define  TXRX_STATE_IDLE       (0x0)
#define  TXRX_STATE_RW_LOCKED  (0x1)
#define  TXRX_STATE_TX_LOCKED  (0x2)
#define  TXRX_STATE_RX_LOCKED  (0x4)

typedef struct txrx_buf {
	size_t     pos;
	size_t     len;
	size_t     size;
	u8        *virt;
	dma_addr_t phys;
	u32        state;
	void      *private;
	struct list_head link;
} txrx_buf_t;

struct dev_ctxt {
	int   uart_no;
	
	/* idle state management */
	int   have_clk;
	int   rx_enabled;
	int   rx_active;
	int   tx_enabled;
	int   tx_active;
	int   idle_timer_set;
	struct timer_list idle_timer;
	typeof(jiffies)   idle_timeout;
	typeof(jiffies)   idle_poll_timeout;
	
	int           uart_speed;
	unsigned int  uart_flags;
	unsigned int  uart_fcr;
	dma_addr_t    uart_phys;
	u8 __iomem   *uart_base;
	unsigned long is_opened;
	unsigned long is_initialized;
	struct miscdevice       mdev;
	struct file_operations  fops;
	struct platform_device *pdev;
	const char             *dev_name; 
	struct hsuart_platform_data *pdata;
	spinlock_t              lock;
	struct mutex            rx_mlock;
	struct mutex            tx_mlock;

	unsigned long  rx_ttl;
	unsigned long  rx_dropped;
	unsigned long  tx_ttl;

	int   tx_buf_size;     
	int   tx_buf_num;      
	int   rx_buf_size;     
	int   rx_buf_num;
	int   rx_buf_shift;
	
	/* TX DMA */
	int        tx_cnt;
	int        tx_dma_ch;
	int        tx_dma_req;
	size_t     tx_dma_size;
	void      *tx_dma_virt;
	dma_addr_t tx_dma_phys; 
	int               tx_free_cnt;
	struct list_head  tx_free_list;
	struct list_head  tx_xfer_list;
	struct txrx_buf  *tx_all_buffs;
	wait_queue_head_t tx_wait;
	
	/* RX DMA */
	int        rx_flow;
	int        rx_timeout;
	int        rx_dma_ch;
	int        rx_dma_req;
	size_t     rx_dma_size;
	void      *rx_dma_virt;
	dma_addr_t rx_dma_phys;
	dma_addr_t rx_dma_last_phys;
	int        rx_dma_empty_count;
	int        rx_dma_empty_count_init;
	struct list_head  rx_free_list;
	struct list_head  rx_xfer_list;
	struct txrx_buf  *rx_all_buffs;
	struct txrx_buf  *rx_q[2];
	int               rx_q_head;
	int               rx_ch[2];
	int               rx_ch_head;
	wait_queue_head_t rx_wait;

	/* rx timer */
	struct omap_dm_timer *rx_timer;
	int         rx_timer_running;  
	u32         rx_timer_rate_msec; // tick rate per msec

	/* debug */
	u32         dbg_log_mask;
	int         dbg_level;
};

/*
 *	 Local prototypes
 */
static void hsuart_stop_tx_xfer    ( struct dev_ctxt *ctxt );
static void hsuart_start_tx_xfer   ( struct dev_ctxt *ctxt );
static void hsuart_stop_rx_xfer    ( struct dev_ctxt *ctxt );
static void hsuart_start_rx_xfer   ( struct dev_ctxt *ctxt );
static void hsuart_isr_callback    ( u8 iir_data, u8 ssr_data, void *dev_id );
static void hsuart_tx_dma_callback (int lch, u16 ch_status, void *dev_id);
static void hsuart_rx_dma_callback (int lch, u16 ch_status, void *dev_id);
static irqreturn_t hsuart_rx_timer_isr ( int irq, void *dev_id );
static struct txrx_buf* 
hsuart_put_and_get_next_rx_buffer  ( struct dev_ctxt *ctxt, 
                                     struct txrx_buf *last_buf, int new_len );

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
static int
hsuart_modem_busy_handler  ( unsigned long dev_id );
static int
hsuart_modem_awake_handler ( unsigned long dev_id ); 
static int
hsuart_modem_sleep_handler ( unsigned long dev_id );
#endif


#ifdef	CONFIG_HSUART_LOG_EVENTS

#define TXRX_EVENT_TX_DMA_START 	0
#define TXRX_EVENT_TX_DMA_STOP  	1
#define TXRX_EVENT_TX_DMA_IRQ   	2
#define TXRX_EVENT_TX_DMA_ERR   	3

#define TXRX_EVENT_RX_DMA_START 	4
#define TXRX_EVENT_RX_DMA_STOP		5 
#define TXRX_EVENT_RX_DMA_IRQ		6
#define TXRX_EVENT_RX_DMA_ERR		7

#define TXRX_EVENT_RX_TIMER_START	8
#define TXRX_EVENT_RX_TIMER_STOP	9
#define TXRX_EVENT_RX_TIMER_IRQ 	10
#define TXRX_EVENT_UNUSED_11    	11


#define TXRX_EVENT_RX_DROP_BYTES	12
#define TXRX_EVENT_RX_UART_ERR  	13
#define TXRX_TX_SLIP_FRAME      	14
#define TXRX_RX_SLIP_FRAME      	15

#define TXRX_EVENT_RX_GOT_BYTES 	16
#define TXRX_EVENT_RX_WAKE_USER 	17
#define TXRX_EVENT_RX_READ_ENTER	18
#define TXRX_EVENT_RX_READ_EXIT 	19

#define TXRX_EVENT_TX_WRITE_ENTER	20
#define TXRX_EVENT_TX_WRITE_EXIT	21
#define TXRX_EVENT_TX_XFER_START	22
#define TXRX_EVENT_TX_XFER_DONE 	23

#define TXRX_EVENT_RX_PIO_BYTE  	24
#define TXRX_EVENT_RX_PIO_BYTES 	25

#define TXRX_EVENT_RX_SLEEP_ENTER	26
#define TXRX_EVENT_RX_SLEEP_EXIT	27


static const char *event_names[] = {
	"tx: dma start",
	"tx: dma stop",
	"tx: dma irq",
	"tx: dma err",

	"rx: dma start",
	"rx: dma stop",
	"rx: dma irq",
	"rx: dma err",

	"rx: timer start",
	"rx: timer stop",
	"rx: timer irq",
	NULL,

	"rx: drop bytes",
	"rx: uart err",
	"tx: slip frame",
	"rx: slip frame",

	"rx: got bytes",
	"rx: wake user",
	"rx: read enter",
	"rx: read exit",

	"tx: write enter",
	"tx: write exit",
	"tx: xfer start",
	"tx: xfer done",

	"rx: pio byte",
	"rx: pio bytes",
	"rx: sleep enter",
	"rx: sleep exit",
};

static u32 event_log_mask = 0
//	  | (1 << TXRX_EVENT_TX_DMA_START)
//	  | (1 << TXRX_EVENT_TX_DMA_STOP)
//	  | (1 << TXRX_EVENT_TX_DMA_IRQ)
	  | (1 << TXRX_EVENT_TX_DMA_ERR)

//	  | (1 << TXRX_EVENT_RX_DMA_START)
//	  | (1 << TXRX_EVENT_RX_DMA_STOP)
//	  | (1 << TXRX_EVENT_RX_DMA_IRQ)
	  | (1 << TXRX_EVENT_RX_DMA_ERR)

//	  | (1 << TXRX_EVENT_RX_TIMER_START)
//	  | (1 << TXRX_EVENT_RX_TIMER_STOP)
//	  | (1 << TXRX_EVENT_RX_TIMER_IRQ)

	  | (1 << TXRX_EVENT_RX_DROP_BYTES)
	  | (1 << TXRX_EVENT_RX_UART_ERR)
//	  | (1 << TXRX_TX_SLIP_FRAME)
//	  | (1 << TXRX_RX_SLIP_FRAME)

//	  | (1 << TXRX_EVENT_RX_GOT_BYTES)
//	  | (1 << TXRX_EVENT_RX_WAKE_USER)
//	  | (1 << TXRX_EVENT_RX_READ_ENTER)
//	  | (1 << TXRX_EVENT_RX_READ_EXIT)

//	  | (1 << TXRX_EVENT_TX_WRITE_ENTER)
//	  | (1 << TXRX_EVENT_TX_WRITE_EXIT)
//	  | (1 << TXRX_EVENT_TX_XFER_START)
//	  | (1 << TXRX_EVENT_TX_XFER_DONE)
	  
//	  | (1 << TXRX_EVENT_RX_PIO_BYTE)
//	  | (1 << TXRX_EVENT_RX_PIO_BYTES)
//	  | (1 << TXRX_EVENT_RX_SLEEP_ENTER)
//	  | (1 << TXRX_EVENT_RX_SLEEP_EXIT)
; 
module_param_named(log_mask, event_log_mask,   uint, 0);
MODULE_PARM_DESC(log_mask, "Enable tx/rx event logging mask");

static inline void
log_txrx_event ( struct dev_ctxt *ctxt, u32 type, u32 arg1, u32 arg2 )
{
	if( unlikely(type >= ARRAY_SIZE(event_names)))
		return;
	if( ctxt->dbg_log_mask & (1 << type)) {
		hres_event((char*) event_names[type], arg1, arg2 );
	}
}

/*
 *  Debug attributes
 */


/*
 *  Debug level attribute
 */
static ssize_t 
hsuart_attr_debug_level_show(struct device *dev, 
                             struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct dev_ctxt        *ctxt;
	
	pdev = container_of(dev, struct platform_device, dev);
	ctxt = platform_get_drvdata ( pdev );
	
	return snprintf(buf, PAGE_SIZE, "%d\n", ctxt->dbg_level );
}

static ssize_t 
hsuart_attr_debug_level_store(struct device *dev, 
                              struct device_attribute *attr, 
                              const char *buf, size_t count)
{
	struct platform_device *pdev;
	struct dev_ctxt        *ctxt; 

	if(!count)
		return count;

	pdev = container_of(dev, struct platform_device, dev);
	ctxt = platform_get_drvdata ( pdev );
	
	ctxt->dbg_level = (int) simple_strtol ( buf, NULL, 0 );;
	
	return count;
}

static DEVICE_ATTR(debug_level, S_IRUGO | S_IWUSR, 
            hsuart_attr_debug_level_show, hsuart_attr_debug_level_store);


/*
 *  evlog_mask attribute
 */
static ssize_t 
hsuart_attr_evlog_mask_show(struct device *dev, 
                            struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev;
	struct dev_ctxt        *ctxt;
	
	pdev = container_of(dev, struct platform_device, dev);
	ctxt = platform_get_drvdata ( pdev );
	
	return snprintf(buf, PAGE_SIZE, "0x%08x\n", ctxt->dbg_log_mask );
}

static ssize_t 
hsuart_attr_evlog_mask_store(struct device *dev, 
                             struct device_attribute *attr, 
                             const char *buf, size_t count)
{
	struct platform_device *pdev;
	struct dev_ctxt        *ctxt; 

	if(!count)
		return count;

	pdev = container_of(dev, struct platform_device, dev);
	ctxt = platform_get_drvdata ( pdev );

	ctxt->dbg_log_mask = (u32) simple_strtol ( buf, NULL, 0 );
	
	return count;

}

static DEVICE_ATTR(evlog_mask, S_IRUGO | S_IWUSR, 
            hsuart_attr_evlog_mask_show, hsuart_attr_evlog_mask_store);


/*
 *  State attribute
 */
static void
hsuart_dump_state(struct dev_ctxt *ctxt)
{
	printk(KERN_INFO "hsuart%d: init     = %d\n", ctxt->uart_no, (int) ctxt->is_initialized );
	printk(KERN_INFO "hsuart%d: opened   = %d\n", ctxt->uart_no, (int) ctxt->is_opened );
	printk(KERN_INFO "hsuart%d: clk      = %d\n", ctxt->uart_no, (int) ctxt->have_clk );
	printk(KERN_INFO "hsuart%d: rxflow   = %d\n", ctxt->uart_no, (int) ctxt->rx_flow );
	printk(KERN_INFO "hsuart%d: rx_timer = %d\n", ctxt->uart_no, (int) ctxt->rx_timer_running );
	printk(KERN_INFO "hsuart%d: rx_ecnt  = %d\n", ctxt->uart_no, (int) ctxt->rx_dma_empty_count );
	printk(KERN_INFO "hsuart%d: tx_free  = %d\n", ctxt->uart_no, (int) ctxt->tx_free_cnt );
	printk(KERN_INFO "hsuart%d: tx_cnt   = %d\n", ctxt->uart_no, (int) ctxt->tx_cnt );
}

static void
hsuart_dump_tx_dma_state(struct dev_ctxt *ctxt)
{
	int ch = ctxt->tx_dma_ch;

	printk("DMA:\n");
	printk("%s = 0x%08x\n", "SYSCONFIG",    omap_readl(OMAP_DMA4_OCP_SYSCONFIG));
	printk("%s = 0x%08x\n", "SYSSTATUS",    omap_readl(OMAP_DMA4_SYSSTATUS));
	printk("%s = 0x%08x\n", "GCR      ",    omap_readl(OMAP_DMA4_GCR_REG));
	printk("%s = 0x%08x\n", "IRQSTATUS_L0", omap_readl(OMAP_DMA4_IRQSTATUS_L0));
	printk("%s = 0x%08x\n", "IRQENABLE_L0", omap_readl(OMAP_DMA4_IRQENABLE_L0));
	
	printk("DMA: Tx channel %d\n", ch );
	printk("%s = 0x%08x\n", "CCR ",  OMAP_DMA_CCR_REG(ch));
	printk("%s = 0x%08x\n", "CLNK",  OMAP_DMA_CLNK_CTRL_REG(ch));
	printk("%s = 0x%08x\n", "CICR",  OMAP_DMA_CICR_REG(ch));
	printk("%s = 0x%08x\n", "CSR ",  OMAP_DMA_CSR_REG (ch));
	printk("%s = 0x%08x\n", "CSDP",  OMAP_DMA_CSDP_REG(ch));
	printk("%s = 0x%08x\n", "CEN ",  OMAP_DMA_CEN_REG (ch));
	printk("%s = 0x%08x\n", "CFN ",  OMAP_DMA_CFN_REG (ch));
	printk("%s = 0x%08x\n", "CSSA",  OMAP2_DMA_CSSA_REG(ch));
	printk("%s = 0x%08x\n", "CDSA",  OMAP2_DMA_CDSA_REG(ch));
	printk("%s = 0x%08x\n", "CSAC",  OMAP_DMA_CSAC_REG(ch));
	printk("%s = 0x%08x\n", "CDAC",  OMAP_DMA_CDAC_REG(ch));
	printk("%s = 0x%08x\n", "CSEI",  OMAP_DMA_CSEI_REG(ch));
	printk("%s = 0x%08x\n", "CSFI",  OMAP_DMA_CSFI_REG(ch));
	printk("%s = 0x%08x\n", "CDEI",  OMAP_DMA_CDEI_REG(ch));
	printk("%s = 0x%08x\n", "CDFI",  OMAP_DMA_CDFI_REG(ch));
	printk("%s = 0x%08x\n", "CCEN",  OMAP2_DMA_CCEN_REG(ch));
	printk("%s = 0x%08x\n", "CCFN",  OMAP2_DMA_CCFN_REG(ch));
//	printk("%s = 0x%08x\n", "COLOR", OMAP2_DMA_COLOR_REG(ch));
}

static void 
hsuart_dump_uart_state(struct dev_ctxt *ctxt)
{
//	u8 saved_lcr = 0;
	
	omap24xx_uart_clk_enable(ctxt->uart_no);

	printk("reg dump: uart%d\n", ctxt->uart_no );
	printk("SYSC = %08x\n",   readl ( ctxt->uart_base + REG_SYSC));

//	saved_lcr = readb ( ctxt->uart_base + REG_LCR );

	printk("LCR  = 0x%02x\n", readb ( ctxt->uart_base + REG_LCR ));
	printk("MDR1 = 0x%02x\n", readb ( ctxt->uart_base + REG_MDR1));
	printk("MDR2 = 0x%02x\n", readb ( ctxt->uart_base + REG_MDR2));
	printk("MCR  = 0x%02x\n", readb ( ctxt->uart_base + REG_MCR ));
	printk("MSR  = 0x%02x\n", readb ( ctxt->uart_base + REG_MSR ));
	printk("TLR  = 0x%02x\n", readb ( ctxt->uart_base + REG_TLR ));
	printk("SCR  = 0x%02x\n", readb ( ctxt->uart_base + REG_SCR ));
	printk("SSR  = 0x%02x\n", readb ( ctxt->uart_base + REG_SSR ));
	
	printk("LSR  = 0x%02x\n", readb ( ctxt->uart_base + REG_LSR ));
	printk("IER  = 0x%02x\n", readb ( ctxt->uart_base + REG_IER ));
	printk("IIR  = 0x%02x\n", readb ( ctxt->uart_base + REG_IIR ));

	printk("WER  = 0x%02x\n", readb ( ctxt->uart_base + REG_WER ));

/*
	writeb( LCR_MODE2, ctxt->uart_base + REG_LCR );
	printk("DLL  = 0x%02x\n", readb ( ctxt->uart_base + REG_DLL ));
	printk("DLH  = 0x%02x\n", readb ( ctxt->uart_base + REG_DLH ));
	printk("MSR  = 0x%02x\n", readb ( ctxt->uart_base + REG_MSR ));
*/	
/*
	writeb( LCR_MODE3, ctxt->uart_base + REG_LCR );

	printk("EFR  = 0x%02x\n", readb ( ctxt->uart_base + REG_EFR ));
	printk("TLR  = 0x%02x\n", readb ( ctxt->uart_base + REG_TLR ));
	printk("TCR  = 0x%02x\n", readb ( ctxt->uart_base + REG_TCR ));
*/
}
 
static void
hsuart_dump_tx_state(struct dev_ctxt *ctxt)
{
	int  i = 0;
	txrx_buf_t *pbuf;
	
	printk(KERN_INFO "hsuart%d: Tx (enabled/active) = %d/%d\n",
	                  ctxt->uart_no, ctxt->tx_enabled,  ctxt->tx_active );
	list_for_each_entry( pbuf, &ctxt->tx_xfer_list, link) {
		printk(KERN_INFO "hsuart%d: txbuf[%2d]: 0x%08x %p %d %d %d\n", 
		          ctxt->uart_no, i++, pbuf->phys, pbuf->virt,
		          pbuf->pos, pbuf->len, pbuf->state); 
	}
}

static void
hsuart_dump_rx_state(struct dev_ctxt *ctxt)
{
	int  i = 0;
	txrx_buf_t *pbuf;
	
	printk(KERN_INFO "hsuart%d: Rx (enabled/active) = %d/%d\n",
	                  ctxt->uart_no, ctxt->tx_enabled,  ctxt->tx_active );
	list_for_each_entry( pbuf, &ctxt->rx_xfer_list, link) {
		printk(KERN_INFO "hsuart%d: rxbuf[%2d]: %d %d %d\n",
		          ctxt->uart_no, i++, pbuf->pos, pbuf->len, pbuf->state);
	}
}

 
static ssize_t 
hsuart_attr_debug_state_show(struct device *dev, 
                             struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	struct platform_device *pdev;
	struct dev_ctxt        *ctxt;
	
	pdev = container_of(dev, struct platform_device, dev);
	ctxt = platform_get_drvdata ( pdev );

	printk(KERN_INFO "hsuart%d: debug state:\n", ctxt->uart_no );
	spin_lock_irqsave ( &ctxt->lock, flags );
	hsuart_dump_state(ctxt);
	hsuart_dump_tx_state(ctxt);
	hsuart_dump_tx_dma_state(ctxt);
	hsuart_dump_uart_state(ctxt);
	hsuart_dump_rx_state(ctxt);
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	printk(KERN_INFO "hsuart%d: debug state: done\n", ctxt->uart_no );
	
	return 0;
}

static ssize_t 
hsuart_attr_debug_state_store(struct device *dev, 
                              struct device_attribute *attr, 
                              const char *buf, size_t count)
{
	u8 lsr1 = 0xFF, lsr2 = 0xFF, data = 0xFF;
	struct platform_device *pdev;
	struct dev_ctxt        *ctxt; 

	if(!count)
		return count;

	pdev = container_of(dev, struct platform_device, dev);
	ctxt = platform_get_drvdata ( pdev );

	printk("hsuart%d: ###### recover stall\n", ctxt->uart_no );
	do { 
		lsr1 = readb ( ctxt->uart_base + REG_LSR );
		data = readb ( ctxt->uart_base + REG_RHR );
		lsr2 = readb ( ctxt->uart_base + REG_LSR );
		printk("hsuart%d: lsr1 = %x, data = %x, lsr2 = %x\n", 
			ctxt->uart_no, (int)lsr1, (int) data, (int)lsr2);
	} while ( lsr2 & 1 );
	
	return count;
}


static DEVICE_ATTR(debug_state, S_IRUGO | S_IWUSR, 
            hsuart_attr_debug_state_show, hsuart_attr_debug_state_store );

static int
hsuart_create_debug_attrs ( struct platform_device *pdev )
{
	int  rc;
	rc = device_create_file(&pdev->dev, &dev_attr_debug_level);
	rc = device_create_file(&pdev->dev, &dev_attr_evlog_mask);
	rc = device_create_file(&pdev->dev, &dev_attr_debug_state);
	return rc;
}

static void
hsuart_remove_debug_attrs ( struct platform_device *pdev )
{
	device_remove_file(&pdev->dev, &dev_attr_debug_level);
	device_remove_file(&pdev->dev, &dev_attr_evlog_mask);
	device_remove_file(&pdev->dev, &dev_attr_debug_state);
}


#else  // CONFIG_HSUART_LOG_EVENTS

#define log_txrx_event(args...)
#define hsuart_create_debug_attrs(args...)
#define hsuart_remove_debug_attrs(args...)

#endif // CONFIG_HSUART_LOG_EVENTS


/* 
 * DEBUG DATA FRAMING
 */
#if 0
static void
slip_check_frame ( struct txrx_buf *buf, int tx )
{
	u8 *ptr = buf->virt;
	int pos = buf->pos; 
	int len = buf->len;

	while (pos < len) {
		if( ptr[pos] == 0xC0 ) {
			log_txrx_event ( tx, (u32) pos, (u32) buf );
		}
		pos++;
	}
}


#define log_slip_frame(buf, type) \
	if( event_log_mask & (1 << (type))) \
		slip_check_frame ( buf, type );
#endif


/*
 *	Uart Helpers
 */
static void
hsuart_module_reset ( struct dev_ctxt *ctxt ) 
{
	u8 reg;
	
	writel ( 0x2, ctxt->uart_base + REG_SYSC );
	do {
		reg = readl ( ctxt->uart_base + REG_SYSC );
	} while ( reg & 0x2 );
}


static void
hsuart_hw_flush_fifo  ( struct dev_ctxt *ctxt, int fifo )
{
	u8  lcr_saved;
	u8  mdr1_saved;
	u8  dll, dlh, lsr, rhr;

	if((fifo & (HSUART_TX_FIFO | HSUART_RX_FIFO)) == 0)
		return;

	/* save LCR */
	lcr_saved = readb ( ctxt->uart_base + REG_LCR );

	/* save mdr1, disable UART */
	mdr1_saved = readb( ctxt->uart_base + REG_MDR1);
	writeb ( UART_DISABLE, ctxt->uart_base + REG_MDR1);

	/* switch to mode 2*/
	writeb ( LCR_MODE2, ctxt->uart_base + REG_LCR );

	/* save dll,dlh, set to 0 */
	dll = readb( ctxt->uart_base + REG_DLL);
	dlh = readb( ctxt->uart_base + REG_DLH);
	writeb ( 0,  ctxt->uart_base + REG_DLL);
	writeb ( 0,  ctxt->uart_base + REG_DLH);
	
	if( fifo & HSUART_TX_FIFO ) {  // clear tx FIFO
		writeb( ctxt->uart_fcr | (1<<2), ctxt->uart_base + REG_FCR );
	}
	if( fifo & HSUART_RX_FIFO ) {  // clear rx FIFO
		writeb( ctxt->uart_fcr | (1<<1), ctxt->uart_base + REG_FCR );
	}

	/* restore dividers */
	writeb ( dll,  ctxt->uart_base + REG_DLL);
	writeb ( dlh,  ctxt->uart_base + REG_DLH);

	/* restore mdr1 */
	writeb ( mdr1_saved, ctxt->uart_base + REG_MDR1);

	/* restore lcr */
	writeb ( lcr_saved,  ctxt->uart_base + REG_LCR );

	/* read all chars from FIFO just in case, but at least once */
	do {
		lsr = readb( ctxt->uart_base + REG_LSR );
		rhr = readb( ctxt->uart_base + REG_RHR );
	} while ( lsr & 1 );

	/* restore lcr */
	writeb ( lcr_saved,  ctxt->uart_base + REG_LCR );
}

static void 
hsuart_hw_set_pfl ( struct dev_ctxt *ctxt, int uart_flags )
{
	int parity, flow, loop;
	u8  lcr_saved;
	u8  mcr_saved; 
	u8  mdr1_saved;
	u8  efr, dll, dlh, lsr, rhr;

	parity = uart_flags & HSUART_MODE_PARITY_MASK;
	flow   = uart_flags & HSUART_MODE_FLOW_CTRL_MASK;
	loop   = uart_flags & HSUART_MODE_LOOPBACK;

	/* assuming operating mode */

	/* save LCR */
	lcr_saved = readb ( ctxt->uart_base + REG_LCR );

	/* set required parity */
	lcr_saved &=~(BIT_LCR_PARITY_EN_M | 
	              BIT_LCR_PARITY_TYPE1_M | 
	              BIT_LCR_PARITY_TYPE2_M );
	if( parity == HSUART_MODE_PARITY_ODD ) {
		lcr_saved |= BIT_LCR_PARITY_EN_M;
	}
	if( parity == HSUART_MODE_PARITY_EVEN ) {
		lcr_saved |= BIT_LCR_PARITY_EN_M | BIT_LCR_PARITY_TYPE1_M;
	}

	/* save MCR */
	mcr_saved = readb ( ctxt->uart_base + REG_MCR );

	/* Save mdr1, disable UART */
	mdr1_saved = readb( ctxt->uart_base + REG_MDR1);
	writeb ( UART_DISABLE, ctxt->uart_base + REG_MDR1);

	/* switch to mode 2*/
	writeb ( LCR_MODE2, ctxt->uart_base + REG_LCR );

	/* save dll,dlh, set to 0 */
	dll = readb( ctxt->uart_base + REG_DLL);
	dlh = readb( ctxt->uart_base + REG_DLH);
	writeb ( 0,  ctxt->uart_base + REG_DLL);
	writeb ( 0,  ctxt->uart_base + REG_DLH);

	/* enable access to TCR */
	writeb ( mcr_saved | BIT_MCR_TCR_TLR_M, ctxt->uart_base + REG_MCR );

	/* set flow control */
	/* switch to mode 3, enable enchanced mode */
	writeb ( LCR_MODE3, ctxt->uart_base + REG_LCR );
	efr = readb ( ctxt->uart_base + REG_EFR );
	writeb ( efr | BIT_EFR_ENHANCED_EN_M, ctxt->uart_base + REG_EFR );

	efr &= ~(BIT_EFR_AUTO_CTS_EN_M | 
	         BIT_EFR_AUTO_RTS_EN_M | 
	         BIT_EFR_SW_FLOW_CONTROL_M);
	if( flow == HSUART_MODE_FLOW_CTRL_HW ) {
		efr  |= (BIT_EFR_AUTO_CTS_EN_M | BIT_EFR_AUTO_RTS_EN_M);
		writeb((HSUART_AUTO_RTS_START << 4) | 
		       (HSUART_AUTO_RTS_HALT ), 
		        ctxt->uart_base + REG_TCR );
	}
	if( flow == HSUART_MODE_FLOW_CTRL_SW ) {
		printk (KERN_ERR "%s: sw flow control is not implemented yet\n", 
		        DRIVER );
	}
	writeb ( efr, ctxt->uart_base + REG_EFR );

	/* back to mode 2 */
	writeb ( LCR_MODE2, ctxt->uart_base + REG_LCR );

	/* set required mcr mode (loopback) */
	if( loop )
		mcr_saved |=  (1 << BIT_MCR_LOOPBACK_EN);
	else
		mcr_saved &= ~(1 << BIT_MCR_LOOPBACK_EN);

	/* write mcr */
	writeb( mcr_saved, ctxt->uart_base + REG_MCR );

	/* clear FIFOs */
	writeb( ctxt->uart_fcr | (3<<2), ctxt->uart_base + REG_FCR );

	/* restore dividers */
	writeb ( dll,  ctxt->uart_base + REG_DLL);
	writeb ( dlh,  ctxt->uart_base + REG_DLH);

	/* restore mdr1 */
	writeb ( mdr1_saved, ctxt->uart_base + REG_MDR1);

	/* restore lcr */
	writeb ( lcr_saved,  ctxt->uart_base + REG_LCR );

	/* read all chars from FIFO just in case, but at least once */
	do {
		lsr = readb( ctxt->uart_base + REG_LSR );
		rhr = readb( ctxt->uart_base + REG_RHR );
	} while ( lsr & 1 );

	return;
}

static void
hsuart_set_ier ( struct dev_ctxt *ctxt, u8 mask )
{
	u8 data;
	data  = readb ( ctxt->uart_base + REG_IER );
	data |= mask;  
	writeb	( data, ctxt->uart_base + REG_IER );
}

static void
hsuart_clr_ier ( struct dev_ctxt *ctxt, u8 mask )
{
	u8 data;
	data  = readb ( ctxt->uart_base + REG_IER );
	data &= ~mask;	
	writeb	( data, ctxt->uart_base + REG_IER );
}

static void
hsuart_set_wkup ( struct dev_ctxt *ctxt)
{
	u8 data;
	data  = readb ( ctxt->uart_base + REG_SCR );
	data |= BIT_SCR_RX_CTS_DSR_WAKE_UP_ENABLE_M; 
	writeb	( data, ctxt->uart_base + REG_SCR );
}

static void
hsuart_clr_wkup ( struct dev_ctxt *ctxt )
{
	u8 data;
	data  = readb ( ctxt->uart_base + REG_SCR );
	data &= ~BIT_SCR_RX_CTS_DSR_WAKE_UP_ENABLE_M; 
	writeb	( data, ctxt->uart_base + REG_SCR );
}

static int 
hsuart_config_uart ( struct dev_ctxt *ctxt )
{
	int  rc;
	struct uart_config   uart_cfg;

	memset ( &uart_cfg,  0, sizeof(struct uart_config));

	// fill uart cfg
	uart_cfg.mode  = UART_MODE;
	uart_cfg.mdr1  = UART_16X_MODE;
	uart_cfg.mdr2  = 0; // Normal UART
	
	 /* Modem control register */
	uart_cfg.mcr   = 0; 

	/* Enhanced feature register */
	uart_cfg.efr   = (1 << BIT_EFR_ENHANCED_EN);
	 
	uart_cfg.scr   = (1 << BIT_SCR_TX_TRIG_GRANU1) | 
	                 (1 << BIT_SCR_RX_TRIG_GRANU1) |
	                 (1 << BIT_SCR_DMA_MODE_CTL)   | 
	                 (1 << BIT_SCR_DMA_MODE_2  );  // DMA Mode 1

	/* Trigger Level Control Register */
	uart_cfg.tlr   = ((HSUART_TX_LVL >> 2) << BIT_TLR_TX_FIFO_TRIG_DMA) |
	                 ((HSUART_RX_LVL >> 2) << BIT_TLR_RX_FIFO_TRIG_DMA);

	/* FIFO Control Register */
	ctxt->uart_fcr =   ((HSUART_TX_LVL & 0x3) << BIT_FCR_TX_FIFO_TRIG )
	                 | ((HSUART_RX_LVL & 0x3) << BIT_FCR_RX_FIFO_TRIG )
	                 | (1 << BIT_FCR_FIFO_EN);
	                 
	uart_cfg.fcr   = ctxt->uart_fcr
	                 | BIT_FCR_RX_FIFO_CLEAR_M
	                 | BIT_FCR_TX_FIFO_CLEAR_M;

	
	/* Interrupt enable register */	
	uart_cfg.ier = 0;

	/* 8N1 */
	uart_cfg.lcr = (3 << BIT_LCR_CHAR_LENGTH);
	
	// Reset and reconfigure UART
	rc = omap24xx_uart_reset ( ctxt->uart_no );
	if( rc ) 
		return rc;

	// set initial configuration
	rc = omap24xx_uart_config( ctxt->uart_no, &uart_cfg );
	if( rc ) 
		return rc;

	return 0;		 
}

static void 
hsuart_stop_uart ( struct dev_ctxt *ctxt )
{
	omap24xx_uart_stop(ctxt->uart_no);
}

static void
hsuart_set_rx_flow ( struct dev_ctxt *ctxt, int mask )
{
	int old_one = ctxt->rx_flow;
	
	ctxt->rx_flow |= mask;
	PDBG3("uart%d: %s: %d\n", ctxt->uart_no, __func__, ctxt->rx_flow );
	
	if(!old_one && ctxt->rx_flow ) {
		// need to throttle rx if we can
		int flow_mode  = ctxt->uart_flags & HSUART_MODE_FLOW_CTRL_MASK;
		if( flow_mode == HSUART_MODE_FLOW_CTRL_HW ) {
			// omap does not support AUTO-RTS and MCR RTS control 
			// simultaneously, so this gpio trick is required
			if( ctxt->pdata->rts_pin >= 0 ) {
				PDBG3("uart%d: %s:\n", ctxt->uart_no, __func__);
				omap_cfg_reg ( ctxt->pdata->rts_gpio_mode );
			}
		}
	}
}

static void
hsuart_clr_rx_flow ( struct dev_ctxt *ctxt, int mask )
{
	int old_one = ctxt->rx_flow;

	ctxt->rx_flow &= ~mask;

	PDBG3("uart%d: %s: %d\n", ctxt->uart_no, __func__, ctxt->rx_flow );
	if( old_one && !ctxt->rx_flow ) {
		// need to unthrottle rx if we can
		int flow_mode  = ctxt->uart_flags & HSUART_MODE_FLOW_CTRL_MASK;
		if( flow_mode == HSUART_MODE_FLOW_CTRL_HW ) {
			// omap does not support AUTO-RTS and MCR RTS control 
			// simultaneously, so this gpio trick is required
			if( ctxt->pdata->rts_pin >= 0 ) {
				PDBG3("uart%d: %s:\n", ctxt->uart_no, __func__);
				omap_cfg_reg ( ctxt->pdata->rts_act_mode );
			}
		}
	}
}


static void
hsuart_recalc_timeout ( struct dev_ctxt *ctxt )
{
	u32 timeout;
	
	// recalc timeout, keep reasonable accuracy
	timeout = (1000000U * 11 * 100) / ctxt->uart_speed;
	timeout = (ctxt->pdata->rx_latency * timeout) / 100; // in usec
	if( timeout < RX_TIMER_MIN_TIMEOUT )
	    timeout = RX_TIMER_MIN_TIMEOUT;

	ctxt->rx_timeout = (timeout * ctxt->rx_timer_rate_msec) / 1000;
	ctxt->rx_dma_empty_count_init = 2048 / ctxt->pdata->rx_latency;
}

static int 
hsuart_set_speed ( struct dev_ctxt *ctxt, int speed )
{
	int rc;
	
	ctxt->uart_speed = speed;
	
	// set initial speed
	rc = omap24xx_uart_set_speed ( ctxt->uart_no, ctxt->uart_speed );
	if( rc ) 
		return rc;
	
	hsuart_recalc_timeout ( ctxt );
	
	return 0;
}

static int hsuart_init_uart ( struct dev_ctxt *ctxt)
{
	int rc;
	struct uart_callback uart_ctxt;

	memset ( &uart_ctxt, 0, sizeof(struct uart_callback));

	// fill uart ctxt
	uart_ctxt.dev = ctxt;
	uart_ctxt.dev_name = (char*) ctxt->dev_name;
	uart_ctxt.mode = UART_NONDMA_MODE;
	uart_ctxt.txrx_req_flag = -1;
	uart_ctxt.int_callback	= hsuart_isr_callback;

	rc = omap24xx_uart_request ( ctxt->uart_no, &uart_ctxt);
	if( rc )
		return rc;

	rc = hsuart_config_uart ( ctxt );
	if( rc ) 
		return rc;

	// set uart speed 
	rc = hsuart_set_speed  ( ctxt, ctxt->uart_speed );
	if( rc ) 
		return rc;

	hsuart_hw_set_pfl ( ctxt, ctxt->uart_flags );

	// recalc timeout
	hsuart_recalc_timeout  ( ctxt );

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
	if( ctxt->pdata->options & HSUART_OPTION_MODEM_DEVICE ) {
		modem_activity_set_uart_handler ( ctxt->uart_no, 
		                                  hsuart_modem_busy_handler, 
		                                  hsuart_modem_sleep_handler,
		                                  NULL,
		                                  NULL,
		                                  hsuart_modem_awake_handler,
		                                  (unsigned long) ctxt );
	}
#endif

	ctxt->is_initialized = 1;
	
	return 0;
}

static inline void hsuart_set_sysconfig( struct dev_ctxt *ctxt, u32 val )
{
	writel ( val, ctxt->uart_base + REG_SYSC );
}


/*
 * Idle state management
 */
static inline void 
hsuart_cancel_idle_timer ( struct dev_ctxt *ctxt )
{
	if( ctxt->idle_timer_set ) {
		del_timer(&ctxt->idle_timer);
		ctxt->idle_timer_set = 0;
	}
}

static inline void 
hsuart_set_idle_timer ( struct dev_ctxt *ctxt, int timeout ) {
	if(!ctxt->idle_timer_set && timeout ) {
	    mod_timer(&ctxt->idle_timer, jiffies + timeout);
	    ctxt->idle_timer_set = 1;
	}
}
 
static void 
hsuart_get_clk( struct dev_ctxt *ctxt)
{
	if( ctxt->have_clk)
		return;
	
	PDBG3("uart%d: %s\n", ctxt->uart_no, __func__ );
	
	// restart the clock
	omap24xx_uart_clk_enable(ctxt->uart_no); // enable clock
	ctxt->have_clk = 1;
}

static void 
hsuart_put_clk( struct dev_ctxt *ctxt )
{
	if(!ctxt->have_clk)
		return;

	PDBG3("uart%d: %s\n", ctxt->uart_no, __func__ );

	// stop the clock 
	omap24xx_uart_clk_disable(ctxt->uart_no);
	ctxt->have_clk = 0;
}

/*
 *	Timers
 */

static void
hsuart_free_rx_timer ( struct dev_ctxt * ctxt ) 
{
	omap_dm_timer_enable        ( ctxt->rx_timer );
	omap_dm_timer_set_int_enable( ctxt->rx_timer, 0 );
	omap_dm_timer_stop          ( ctxt->rx_timer );
	ctxt->rx_timer_running = 0;
	free_irq ( omap_dm_timer_get_irq ( ctxt->rx_timer ), ctxt );
	omap_dm_timer_free          ( ctxt->rx_timer );
}


static void
hsuart_configure_timer ( struct dev_ctxt * ctxt )
{
	int rate;
	
	omap_dm_timer_enable ( ctxt->rx_timer );
	omap_dm_timer_stop   ( ctxt->rx_timer );
	omap_dm_timer_set_int_enable ( ctxt->rx_timer, 0 );
	omap_dm_timer_write_status   ( ctxt->rx_timer, 
	                               OMAP_TIMER_INT_CAPTURE | 
	                               OMAP_TIMER_INT_OVERFLOW | 
	                               OMAP_TIMER_INT_MATCH );
	omap_dm_timer_set_source     ( ctxt->rx_timer, OMAP_TIMER_SRC_SYS_CLK );
	omap_dm_timer_set_int_enable ( ctxt->rx_timer, OMAP_TIMER_INT_OVERFLOW);

	rate = clk_get_rate( omap_dm_timer_get_fclk(ctxt->rx_timer));
	ctxt->rx_timer_rate_msec = rate / 1000;
	ctxt->rx_timer_running   = 0;
	
	omap_dm_timer_disable ( ctxt->rx_timer );
}


static int
hsuart_alloc_rx_timer ( struct dev_ctxt * ctxt) 
{
	int rc;
	
	ctxt->rx_timer = omap_dm_timer_request ();
	if( ctxt->rx_timer == NULL ) 
		return -ENODEV;

	hsuart_configure_timer ( ctxt );

	rc  = request_irq  ( omap_dm_timer_get_irq ( ctxt->rx_timer ), 
	                     hsuart_rx_timer_isr, 0, ctxt->dev_name, ctxt );
	if( rc )
		goto err_free_timer;

	return 0;	 

err_free_timer:
	omap_dm_timer_free ( ctxt->rx_timer );	  
	
	return rc;	  
}


static inline int
hsuart_rx_timer_ack  ( struct dev_ctxt * ctxt )
{
	if(!ctxt->rx_timer_running)
		return -1;
	
	omap_dm_timer_write_status( ctxt->rx_timer, OMAP_TIMER_INT_OVERFLOW);
	return 0;
}


static inline void
hsuart_rx_timer_stop ( struct dev_ctxt * ctxt )
{
	omap_dm_timer_enable      ( ctxt->rx_timer );
	omap_dm_timer_stop        ( ctxt->rx_timer );
	omap_dm_timer_write_status( ctxt->rx_timer, OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_disable     ( ctxt->rx_timer );
	ctxt->rx_timer_running   = 0;
	log_txrx_event ( ctxt, TXRX_EVENT_RX_TIMER_STOP, 0, 0 );
}

static inline void
hsuart_rx_timer_start ( struct dev_ctxt * ctxt, int ticks )
{
	omap_dm_timer_enable  ( ctxt->rx_timer );
	omap_dm_timer_set_load( ctxt->rx_timer, 0, 0xffffffff - ticks );
	omap_dm_timer_start   ( ctxt->rx_timer );
	ctxt->rx_timer_running = 1;
	log_txrx_event ( ctxt, TXRX_EVENT_RX_TIMER_START, ticks, 0 );
}

/*
 *	RX/TX status
 */
static inline int
hsuart_tx_has_space ( struct dev_ctxt * ctxt )
{
	return (ctxt->tx_free_cnt >= ctxt->tx_buf_num / 2);
}

static inline int
hsuart_tx_is_empty ( struct dev_ctxt * ctxt )
{
	if( list_empty( &ctxt->tx_xfer_list))
		return 1;
	else
		return 0;
}

static inline int
hsuart_tx_fifo_is_empty( struct dev_ctxt * ctxt )
{
	u8 lsr = readb ( ctxt->uart_base + REG_LSR );

	if((lsr & BIT_LSR_TX_FIFO_E_M) &&
	   (lsr & BIT_LSR_TX_SR_E_M))
		return 1; // empty
	else
		return 0; 
}

static inline int
hsuart_rx_has_bytes ( struct dev_ctxt * ctxt )
{
	size_t count = 0;
	txrx_buf_t *pbuf;
	
	list_for_each_entry( pbuf, &ctxt->rx_xfer_list, link) {
		count += pbuf->len - pbuf->pos;
	}
	return count;
}

static inline int
hsuart_rx_has_bytes_locked ( struct dev_ctxt * ctxt )
{
	int rc;
	unsigned long flags;
	
	spin_lock_irqsave ( &ctxt->lock, flags );
	rc = hsuart_rx_has_bytes (ctxt);
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	return rc;
}

static inline int
hsuart_rx_fifo_has_bytes ( struct dev_ctxt * ctxt )
{
	u8 lsr = readb ( ctxt->uart_base + REG_LSR );

	if( lsr & BIT_LSR_RX_FIFO_E_M )
		return 1; // rx fifo has bytes
	else
		return 0; // empty
}

/*
 *	TX/RX Buffer helpers
 */
static void 
hsuart_enqueue_tx_buffer ( struct dev_ctxt * ctxt, struct txrx_buf *txbuf)
{
	unsigned long flags;

	spin_lock_irqsave ( &ctxt->lock, flags );
	list_move_tail ( &txbuf->link, &ctxt->tx_xfer_list );
	txbuf->state &= ~TXRX_STATE_RW_LOCKED;
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}

static txrx_buf_t *
hsuart_get_tx_buffer ( struct dev_ctxt * ctxt)
{
	unsigned long flags;
	txrx_buf_t *txbuf = NULL;

	spin_lock_irqsave ( &ctxt->lock, flags );
	list_for_each_entry( txbuf, &ctxt->tx_free_list, link) {
		if( txbuf->state == TXRX_STATE_IDLE ) {
			txbuf->state |= TXRX_STATE_RW_LOCKED;
			txbuf->pos = 0;
			txbuf->len = 0;
			ctxt->tx_free_cnt--;
			spin_unlock_irqrestore ( &ctxt->lock, flags );
			return txbuf;
		}
	}
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	return NULL;
}

static void
hsuart_put_tx_buffer ( struct dev_ctxt * ctxt, txrx_buf_t *txbuf )
{
	unsigned long flags;

	spin_lock_irqsave ( &ctxt->lock, flags );
	txbuf->state = TXRX_STATE_IDLE;
	list_move_tail ( &txbuf->link, &ctxt->tx_free_list );
	ctxt->tx_free_cnt++;
	if( hsuart_tx_has_space (ctxt)) {
		wake_up_interruptible( &ctxt->tx_wait );
	}
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}

static txrx_buf_t *
hsuart_get_rd_buffer ( struct dev_ctxt * ctxt, size_t *bytes )
{
	unsigned long flags;
	txrx_buf_t *rxbuf = NULL;

	spin_lock_irqsave ( &ctxt->lock, flags );
	if(!list_empty( &ctxt->rx_xfer_list))  {
		rxbuf = list_first_entry ( &ctxt->rx_xfer_list, txrx_buf_t, link);
		*bytes = rxbuf->len - rxbuf->pos;
		if( *bytes ) 
			rxbuf->state |= TXRX_STATE_RW_LOCKED;
		else
			rxbuf = NULL;
	}
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	return rxbuf;
}

static void
hsuart_put_rd_buffer ( struct dev_ctxt * ctxt, txrx_buf_t *rxbuf, int rx_len )
{
	unsigned long flags;

	spin_lock_irqsave ( &ctxt->lock, flags );
	rxbuf->pos += rx_len;
	rxbuf->state &= ~TXRX_STATE_RW_LOCKED;
	if((rxbuf->pos == rxbuf->len) && 
	   (rxbuf->state == TXRX_STATE_IDLE)) { // return it to free pool
		list_move_tail ( &rxbuf->link, &ctxt->rx_free_list );
		hsuart_clr_rx_flow (ctxt, HSUART_RX_FLOW_SPACE );
	}
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}

static void
hsuart_put_rx_buffer ( struct dev_ctxt * ctxt, txrx_buf_t *rxbuf )
{
	unsigned long flags;

	spin_lock_irqsave ( &ctxt->lock, flags );
	rxbuf->state &= ~TXRX_STATE_RX_LOCKED;
	if( rxbuf->state == TXRX_STATE_IDLE) {
		list_move_tail ( &rxbuf->link, &ctxt->rx_free_list );
	}
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}

static struct txrx_buf*
hsuart_put_and_get_next_rx_buffer ( struct dev_ctxt *ctxt, 
                                    struct txrx_buf *last_buf, int new_len)
{
	struct txrx_buf *next_buf;

	 // grab next free rx buffer
	if( unlikely(list_empty( &ctxt->rx_free_list))) {
		// no more free buffers
		if( unlikely(last_buf == NULL)) {
			// this is actually invalid state
			printk(KERN_ERR "%s: unexpected state\n",__FUNCTION__);
			return NULL;
		}
		// discard last recieved one, it will be reused for next rx
		ctxt->rx_ttl += new_len - last_buf->len;
		ctxt->rx_dropped += new_len;
		log_txrx_event( ctxt, TXRX_EVENT_RX_DROP_BYTES, new_len, 0 );
		last_buf->state = TXRX_STATE_IDLE;
		list_move_tail ( &last_buf->link, &ctxt->rx_free_list );
		last_buf = NULL;
	}

	/* update previous buffer */
	if( last_buf ) {
		int count;
		log_txrx_event( ctxt, TXRX_EVENT_RX_GOT_BYTES, new_len, last_buf->len);
		ctxt->rx_ttl += new_len - last_buf->len;
		last_buf->len = new_len;
//		log_slip_frame ( last_buf, TXRX_RX_SLIP_FRAME );
		last_buf->state &= ~TXRX_STATE_RX_LOCKED;
		if((last_buf->state == TXRX_STATE_IDLE) &&
		   (last_buf->len == last_buf->pos)) {
			list_move_tail ( &last_buf->link, &ctxt->rx_free_list);
		}
		count = hsuart_rx_has_bytes( ctxt );
		if( count ) {
			log_txrx_event( ctxt, TXRX_EVENT_RX_WAKE_USER, count, 0 );
			wake_up_interruptible( &ctxt->rx_wait );
		}
	}

	/* queue next buffer */
	next_buf = list_first_entry( &ctxt->rx_free_list, txrx_buf_t, link);
	next_buf->pos = 0;
	next_buf->len = 0;
	next_buf->state = TXRX_STATE_RX_LOCKED;
	list_move_tail ( &next_buf->link, &ctxt->rx_xfer_list );
	if( unlikely(list_empty( &ctxt->rx_free_list))) {
		hsuart_set_rx_flow (ctxt, HSUART_RX_FLOW_SPACE );
	}

	return next_buf;
}

static void 
hsuart_flush_rx_queue( struct dev_ctxt * ctxt )
{
	txrx_buf_t *pbuf = NULL;
	
	while (!list_empty(&ctxt->rx_xfer_list)) {
		pbuf = list_first_entry(&ctxt->rx_xfer_list, txrx_buf_t, link);
		hsuart_put_rx_buffer( ctxt, pbuf );
		hsuart_put_rd_buffer( ctxt, pbuf, pbuf->len - pbuf->pos );
	}
	ctxt->rx_q[0] = NULL;
	ctxt->rx_q[1] = NULL;
	ctxt->rx_q_head  = 0;
}

static void 
hsuart_flush_tx_queue( struct dev_ctxt * ctxt )
{
	txrx_buf_t *pbuf = NULL;
	
	// return all pending buffers to free list
	while (!list_empty(&ctxt->tx_xfer_list)) {
		pbuf = list_first_entry(&ctxt->tx_xfer_list, txrx_buf_t, link);
		hsuart_put_tx_buffer( ctxt, pbuf );
	}
}
 
/*
 *	 TX DMA
 */
static void
hsuart_stop_tx_dma ( struct dev_ctxt * ctxt )
{
	omap_stop_dma  ( ctxt->tx_dma_ch );
	log_txrx_event ( ctxt, TXRX_EVENT_TX_DMA_STOP, ctxt->tx_dma_ch, 0 );
}

static void
hsuart_start_tx_dma ( struct dev_ctxt * ctxt, struct txrx_buf *txbuf )
{
	int frame_len, frame_num;

//	log_slip_frame ( txbuf, TXRX_TX_SLIP_FRAME );

	frame_len = txbuf->len - txbuf->pos;
	frame_num = 1;

	omap_set_dma_callback( ctxt->tx_dma_ch, hsuart_tx_dma_callback, txbuf );
	
	omap_set_dma_transfer_params( ctxt->tx_dma_ch,
	                     OMAP_DMA_DATA_TYPE_S8, frame_len, frame_num,
	                     OMAP_DMA_SYNC_ELEMENT, ctxt->tx_dma_req, 0);

	omap_set_dma_src_params ( ctxt->tx_dma_ch, 
	                     0, OMAP_DMA_AMODE_POST_INC, 
	                     (unsigned long )(txbuf->phys + txbuf->pos), 0, 0 );

	omap_set_dma_dest_params( ctxt->tx_dma_ch,
	                     0, OMAP_DMA_AMODE_CONSTANT, 
	                     (unsigned long ) ctxt->uart_phys, 0, 0 );

	txbuf->pos += frame_len * frame_num;

	omap_start_dma ( ctxt->tx_dma_ch );
	
	log_txrx_event ( ctxt, TXRX_EVENT_TX_DMA_START, frame_len, frame_num );
}

static int 
hsuart_free_tx_dma	( struct dev_ctxt * ctxt )
{
	dma_free_coherent ( &ctxt->pdev->dev, 
	                     ctxt->tx_dma_size, 
	                     ctxt->tx_dma_virt, 
	                     ctxt->tx_dma_phys );

	omap_free_dma ( ctxt->tx_dma_ch );
	
	return 0;
}

static int 
hsuart_alloc_tx_dma ( struct dev_ctxt * ctxt)
{
	int rc;

	ctxt->tx_dma_req  = OMAP24XX_DMA_UART1_TX + ctxt->uart_no * 2;
	rc = omap_request_dma ( ctxt->tx_dma_req,  ctxt->dev_name,
	                        hsuart_tx_dma_callback, ctxt, &ctxt->tx_dma_ch );
	if( rc )
		return rc;

	ctxt->tx_dma_size = ctxt->tx_buf_num * ctxt->tx_buf_size;
	ctxt->tx_dma_virt = dma_alloc_coherent( &ctxt->pdev->dev, 
	                                         ctxt->tx_dma_size, 
	                                        (dma_addr_t *)&ctxt->tx_dma_phys, 0 );
	if( ctxt->tx_dma_virt == NULL )
		goto err_free_ch;	 

	return 0;

err_free_ch:
	omap_free_dma ( ctxt->tx_dma_ch );
	
	return -ENOMEM;
}

/*
 *	 RX DMA
 */
static void
hsuart_stop_rx_dma ( struct dev_ctxt *ctxt )
{
	omap_stop_dma_chain_transfers ( ctxt->rx_dma_ch );
	log_txrx_event( ctxt, TXRX_EVENT_RX_DMA_STOP, ctxt->rx_dma_ch, ctxt->rx_ttl );
}

static void
hsuart_start_rx_dma ( struct dev_ctxt *ctxt )
{
	int  idx;
	struct txrx_buf *rxbuf;

	ctxt->rx_ch_head = 0;
	ctxt->rx_dma_last_phys   = 0xFFFFFFFF;
	ctxt->rx_dma_empty_count = ctxt->rx_dma_empty_count_init;

	idx   = ctxt->rx_q_head;
	rxbuf = ctxt->rx_q[idx];
	omap_dma_chain_a_transfer( ctxt->rx_dma_ch, ctxt->uart_phys,
	                           rxbuf->phys + rxbuf->len,
	                           rxbuf->size - rxbuf->len,
	                           1, rxbuf );
	omap_start_dma_chain_transfers ( ctxt->rx_dma_ch );

	idx   = 1 - idx;
	rxbuf = ctxt->rx_q[idx];
	omap_dma_chain_a_transfer( ctxt->rx_dma_ch, ctxt->uart_phys,
	                           rxbuf->phys + rxbuf->len,
	                           rxbuf->size - rxbuf->len,
	                           1, rxbuf );
  
	log_txrx_event ( ctxt, TXRX_EVENT_RX_DMA_START, ctxt->rx_dma_ch, ctxt->rx_ttl );
}


static int 
hsuart_free_rx_dma( struct dev_ctxt * ctxt )
{
	dma_free_coherent ( &ctxt->pdev->dev, 
	                     ctxt->rx_dma_size, 
	                     ctxt->rx_dma_virt, 
	                     ctxt->rx_dma_phys );

	omap_free_dma_chain ( ctxt->rx_dma_ch );
	
	return 0;
}

static int 
hsuart_alloc_rx_dma_chain ( struct dev_ctxt * ctxt )
{
	int rc;
	int nchan = 2;
	struct omap_dma_channel_params params;
	 
	ctxt->rx_dma_req   = OMAP24XX_DMA_UART1_RX + ctxt->uart_no * 2;   
	 
	memset ( &params, 0, sizeof(params));
	 
	params.data_type   = OMAP_DMA_DATA_TYPE_S8;   
	params.elem_count  = 1;   
	params.frame_count = 1;   
   
	params.src_amode   = OMAP_DMA_AMODE_CONSTANT;
	params.src_start   = (unsigned long) ctxt->uart_phys;
	 
	params.dst_amode   = OMAP_DMA_AMODE_POST_INC;
   
	params.trigger     = ctxt->rx_dma_req;
	params.sync_mode   = OMAP_DMA_SYNC_ELEMENT;   
	params.src_or_dst_synch = 1;

	rc = omap_request_dma_chain ( ctxt->rx_dma_req,  ctxt->dev_name,
	                              hsuart_rx_dma_callback,
	                              &ctxt->rx_dma_ch, nchan,  
	                              OMAP_DMA_DYNAMIC_CHAIN, params );
	if( rc ) 
		return rc;

	ctxt->rx_ch_head = 0;
	omap_get_dma_chain_channels ( ctxt->rx_dma_ch, ctxt->rx_ch );
	printk("hsuart%d: grap dma chain %d\n", ctxt->uart_no, ctxt->rx_dma_ch);

	return 0;
}

static void
hsuart_free_rx_dma_chain ( struct dev_ctxt * ctxt )
{
	omap_free_dma_chain ( ctxt->rx_dma_ch );
}


static int 
hsuart_alloc_rx_dma ( struct dev_ctxt * ctxt )
{
	int    rc;

	rc = hsuart_alloc_rx_dma_chain ( ctxt );
	if( rc != 0 )
		return rc;

	ctxt->rx_dma_size = ctxt->rx_buf_num * ctxt->rx_buf_size;
	ctxt->rx_dma_virt = dma_alloc_coherent( &ctxt->pdev->dev, 
	                              ctxt->rx_dma_size, 
	                             (dma_addr_t *)&ctxt->rx_dma_phys, 0 );
	if( ctxt->rx_dma_virt == NULL )
		goto err_free_ch;

	return 0;

err_free_ch:
	hsuart_free_rx_dma_chain ( ctxt );
	
	return -ENOMEM;
}

static void 
hsuart_stop_tx_xfer ( struct dev_ctxt * ctxt )
{
	unsigned long flags;

	spin_lock_irqsave ( &ctxt->lock, flags );
	PDBG3("uart%d: %s\n", ctxt->uart_no, __func__);
	hsuart_stop_tx_dma ( ctxt );
	hsuart_clr_ier ( ctxt, BIT_IER_THR_IT_M );
	ctxt->tx_active  = 0;
	ctxt->tx_enabled = 0;
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}

static void 
hsuart_start_tx_xfer ( struct dev_ctxt * ctxt )
{
	unsigned long flags;
	txrx_buf_t *txbuf; 

	spin_lock_irqsave ( &ctxt->lock, flags );

	ctxt->tx_enabled = 1;

	if( list_empty( &ctxt->tx_xfer_list)) {
		// Tx queue is empty
		PDBG3("uart%d: %s: Tx-empty\n", ctxt->uart_no, __func__);
		ctxt->tx_active = 0;   // reset tx_active flag
		if(!ctxt->rx_active)   // restart idle timer
			hsuart_set_idle_timer(ctxt, ctxt->idle_timeout);
		goto done;
	}
		
	txbuf = list_first_entry ( &ctxt->tx_xfer_list, txrx_buf_t, link);
	if( txbuf->state & TXRX_STATE_TX_LOCKED ) {
		PDBG3("uart%d: %s: Tx-in-progress\n", ctxt->uart_no, __func__);
		goto done;    // tx is already in progress
	}
	
	ctxt->tx_active = 1;   // set tx in progress flag
	txbuf->state |= TXRX_STATE_TX_LOCKED;
	
	hsuart_get_clk(ctxt);  // make sure we have clocks
	hsuart_cancel_idle_timer(ctxt);

	log_txrx_event ( ctxt, TXRX_EVENT_TX_XFER_START, txbuf->len, (u32)txbuf);
	PDBG3("uart%d: %s: Tx-start\n", ctxt->uart_no, __func__);
	
	hsuart_start_tx_dma ( ctxt, txbuf );

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
	if( ctxt->pdata->options & HSUART_OPTION_MODEM_DEVICE ) {
		modem_activity_touch_uart_port ( ctxt->uart_no, 
		                                 0, __FUNCTION__ );
	}
#endif
done:
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}


static void 
hsuart_stop_rx_xfer ( struct dev_ctxt *ctxt )
{
	unsigned long flags;

	spin_lock_irqsave ( &ctxt->lock, flags );
	PDBG3("uart%d: %s\n", ctxt->uart_no, __func__);
	hsuart_set_rx_flow(ctxt, HSUART_RX_FLOW_STOP);
	hsuart_rx_timer_stop ( ctxt );
	hsuart_stop_rx_dma( ctxt );
	hsuart_clr_ier ( ctxt, BIT_IER_RHR_IT_M );
	ctxt->rx_active  = 0;
	ctxt->rx_enabled = 0;
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}

static void 
hsuart_start_rx_xfer ( struct dev_ctxt *ctxt )
{
	unsigned long flags;

	spin_lock_irqsave ( &ctxt->lock, flags );
	PDBG3("uart%d: %s\n", ctxt->uart_no, __func__);
	ctxt->rx_q_head = 0;
	ctxt->rx_q[0] = hsuart_put_and_get_next_rx_buffer( ctxt, NULL, 0 );
	ctxt->rx_q[1] = hsuart_put_and_get_next_rx_buffer( ctxt, NULL, 0 );
	hsuart_set_ier ( ctxt, BIT_IER_RHR_IT_M);
	ctxt->rx_enabled = 1;
	hsuart_set_wkup(ctxt);
	hsuart_clr_rx_flow(ctxt, HSUART_RX_FLOW_STOP);
	spin_unlock_irqrestore ( &ctxt->lock, flags );
}

static int
hsuart_do_pio_rx ( struct dev_ctxt *ctxt )
{
	u8  lsr; 
	u8  data;
	int cnt = 0;

	lsr  = readb ( ctxt->uart_base + REG_LSR );
	while ((lsr & BIT_LSR_RX_FIFO_E_M)) {
		if((lsr & 0x8E)) {
			log_txrx_event( ctxt, TXRX_EVENT_RX_UART_ERR, lsr, 0 );
		}
		data = readb ( ctxt->uart_base + REG_RHR ); 
		log_txrx_event ( ctxt, TXRX_EVENT_RX_PIO_BYTE,(int)lsr,(int)data );
		cnt++;
		lsr  = readb ( ctxt->uart_base + REG_LSR );
	}
	log_txrx_event ( ctxt, TXRX_EVENT_RX_PIO_BYTES, cnt, ctxt->rx_ttl );
	return cnt;
}

static int
hsuart_is_busy(struct dev_ctxt *ctxt) 
{
	if( ctxt->rx_enabled && ctxt->rx_active ) 
		return 1;
	
	if( ctxt->tx_enabled && ctxt->tx_active )
		return 1;
	
	if(!hsuart_tx_fifo_is_empty(ctxt))
		return 1;
	
	if( hsuart_rx_fifo_has_bytes(ctxt))
		return 1;
	
	return 0;
}

static void 
hsuart_idle_timeout(unsigned long data)
{
	unsigned long flags;
	struct dev_ctxt *ctxt  = (struct dev_ctxt *) data;
	
	spin_lock_irqsave ( &ctxt->lock, flags );
	ctxt->idle_timer_set = 0; // timer is not set

	if(!ctxt->have_clk)
		goto done;
	
	if( hsuart_is_busy(ctxt))
		goto busy;
	
	PDBG3("uart%d: enter idle\n", ctxt->uart_no );
	
	// Otherwise, enter sleep mode 
	hsuart_set_wkup(ctxt); // setup wakeup interrupt
	omap24xx_uart_clk_disable(ctxt->uart_no);
	ctxt->have_clk = 0;
done:	
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	return;
	
busy:
	PDBG3("uart%d: device busy\n", ctxt->uart_no );

	// restart idle timer and bail out
	mod_timer(&ctxt->idle_timer, jiffies + ctxt->idle_poll_timeout);
	ctxt->idle_timer_set = 1;
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	return;
}


static void 
hsuart_isr_callback ( u8 iir_data, u8 ssr_data, void *dev_id )
{
	u8  lsr;
	int type;
	struct dev_ctxt *ctxt = dev_id;

	/* Check for wake-up interrupt.
	 */
	if (ssr_data & BIT_SSR_RX_CTS_DSR_WAKE_UP_STS_M) {
		unsigned long flags;
		spin_lock_irqsave ( &ctxt->lock, flags );
		PDBG3("uart%d: hsuart_isr: wkup\n", ctxt->uart_no );
		hsuart_get_clk(ctxt);
		hsuart_clr_wkup(ctxt); // clear/disable wkup interrupt
		hsuart_cancel_idle_timer (ctxt);
		hsuart_set_idle_timer(ctxt, ctxt->idle_timeout);
		spin_unlock_irqrestore( &ctxt->lock, flags );
	}

	if((iir_data & BIT_IIR_IT_PENDING_M) == 1 )
		return; // no interrupt is pending

	lsr  = readb ( ctxt->uart_base + REG_LSR );
	if( lsr & 0x0E ) {
		PDBG3("uart%d: hsuart_isr: lsr=%x\n", ctxt->uart_no, lsr );
		log_txrx_event ( ctxt, TXRX_EVENT_RX_UART_ERR, lsr, 0 );
	}
	type = (iir_data & BIT_IIR_IT_TYPE_M) >> BIT_IIR_IT_TYPE;
	switch ( type ) 
	{
		case 0x2: // RHR interrupt
		case 0x6: // Rx-Timeout 
		{
			unsigned long flags;
			spin_lock_irqsave ( &ctxt->lock, flags );
			PDBG3("uart%d: hsuart_isr: RHR/Rx-Timeout lsr=%x\n", ctxt->uart_no, lsr);
			ctxt->rx_active = 1;
			hsuart_get_clk(ctxt);
			hsuart_cancel_idle_timer (ctxt);
			hsuart_clr_ier(ctxt, BIT_IER_RHR_IT_M );
			hsuart_start_rx_dma ( ctxt );
			hsuart_rx_timer_start ( ctxt, ctxt->rx_timeout );
			spin_unlock_irqrestore( &ctxt->lock, flags );
#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
			if( ctxt->pdata->options & HSUART_OPTION_MODEM_DEVICE ) {
				modem_activity_touch_uart_port ( ctxt->uart_no, 
				                                 0, __FUNCTION__ );
			}
#endif
		}
		break;

		case 0x3: // Line status interrupt
			hsuart_do_pio_rx ( ctxt );
		break;

//		case 0x1: // THR interrupt
//		break;
		
		default:
			PDBG1("%s: iir=%x lsr=%x\n", __FUNCTION__, iir_data, lsr);
		break;
	}
	return;
}


static dma_addr_t
hsuart_update_rxbuf_pos ( struct dev_ctxt *ctxt, struct txrx_buf *exbuf )
{
	int   bidx, new_len;
	dma_addr_t new_phys;
	txrx_buf_t *rxbuf = NULL;

//	new_phys = (dma_addr_t)omap_get_dma_chain_dst_pos ( ctxt->rx_dma_ch );
	new_phys = OMAP_DMA_CDAC_REG(ctxt->rx_ch[ctxt->rx_ch_head]);

	if( (new_phys >= ctxt->rx_dma_phys) &&
	    (new_phys <= ctxt->rx_dma_phys + ctxt->rx_dma_size) &&
	   ((new_phys -  ctxt->rx_dma_phys) & ((1 << ctxt->rx_buf_shift)-1))){
		// at this point the dma pos is inside the buffer
		bidx  = (new_phys - ctxt->rx_dma_phys) >> ctxt->rx_buf_shift;
		rxbuf = &(ctxt->rx_all_buffs[bidx]);
		new_len = new_phys - rxbuf->phys;
		if( new_len > rxbuf->len ) {
			log_txrx_event( ctxt, TXRX_EVENT_RX_GOT_BYTES, new_len, rxbuf->len );
			ctxt->rx_ttl += new_len - rxbuf->len;
			rxbuf->len = new_len;
//			log_slip_frame ( rxbuf, TXRX_RX_SLIP_FRAME );
		}
	}

	if( ctxt->rx_dma_last_phys == new_phys ) {
	    ctxt->rx_dma_empty_count--;
	} else {  
	    ctxt->rx_dma_empty_count = ctxt->rx_dma_empty_count_init;
	    ctxt->rx_dma_last_phys   = new_phys;
	}

	return new_phys;
}


static irqreturn_t
hsuart_rx_timer_isr ( int irq, void *dev_id )
{
	unsigned long flags;
	dma_addr_t pos1, pos2;
	struct txrx_buf *rxbuf = NULL;
	struct dev_ctxt *ctxt  = dev_id;

	spin_lock_irqsave( &ctxt->lock, flags );

	if( hsuart_rx_timer_ack ( ctxt )) {
		// is it spurious interrupt?
		printk(KERN_ERR "%s: unexpected timer interrupt\n",__func__);
		goto done;
	}
	log_txrx_event ( ctxt, TXRX_EVENT_RX_TIMER_IRQ, 0, 0 );

	pos1 = hsuart_update_rxbuf_pos ( ctxt, ctxt->rx_q[ctxt->rx_q_head]);
	if( ctxt->rx_dma_empty_count == 0 ) {
		// no dma rx progress
		hsuart_stop_rx_dma ( ctxt );
		hsuart_set_ier ( ctxt, BIT_IER_RHR_IT_M );
		/* Stop the timer. Even though it is a one shot timer, we still
		 * need to stop it to turn off the timer clocks.
		 */
		hsuart_rx_timer_stop ( ctxt );

		pos2 = hsuart_update_rxbuf_pos ( ctxt, ctxt->rx_q[ctxt->rx_q_head] );
		// check top of rx buffer queue, 
		// if it is more then half full, get a new one
		rxbuf = ctxt->rx_q[ctxt->rx_q_head];
//		if( pos1 != pos2 ) {
//			printk ("pos1/pos2/buf = 0x%08x/0x%08x/0x%08x/0x%08x\n", 
//			pos1, pos2, rxbuf->phys, rxbuf->phys + rxbuf->size );
//			pos2 = hsuart_update_rxbuf_pos ( ctxt, ctxt->rx_q[ctxt->rx_q_head] );
//		}
		if( rxbuf->len > rxbuf->size/2 ) {
			rxbuf = hsuart_put_and_get_next_rx_buffer( ctxt, rxbuf, rxbuf->len );
			ctxt->rx_q[ctxt->rx_q_head] = rxbuf;
			ctxt->rx_q_head = 1 - ctxt->rx_q_head;
		}
		ctxt->rx_active = 0; // rx path is not active
		if(!ctxt->tx_active )  // start idle timer
			hsuart_set_idle_timer(ctxt, ctxt->idle_timeout);
	} else {
		hsuart_rx_timer_start ( ctxt, ctxt->rx_timeout );
#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
		if( ctxt->pdata->options & HSUART_OPTION_MODEM_DEVICE ) {
			modem_activity_touch_uart_port ( ctxt->uart_no, 
			                                 0, __FUNCTION__ );
		}
#endif
	}
	if( hsuart_rx_has_bytes ( ctxt )) {
		log_txrx_event( ctxt, TXRX_EVENT_RX_WAKE_USER, hsuart_rx_has_bytes ( ctxt ), 0 );
		wake_up_interruptible ( &ctxt->rx_wait );
	}
done:
	spin_unlock_irqrestore( &ctxt->lock, flags );

	return IRQ_HANDLED;
}


static void 
hsuart_rx_dma_callback ( int lch, u16 ch_status, void *dev_id)
{
	u8 uart_status;
	unsigned long flags;
	struct txrx_buf *rxbuf = (struct txrx_buf *)dev_id;
	struct dev_ctxt *ctxt  = (struct dev_ctxt *)rxbuf->private;

	spin_lock_irqsave (&ctxt->lock, flags );
	log_txrx_event( ctxt, TXRX_EVENT_RX_DMA_IRQ, lch, ch_status );
	if( ch_status ) {
		log_txrx_event( ctxt, TXRX_EVENT_RX_DMA_ERR, lch, ch_status );
	}
	uart_status = readb ( ctxt->uart_base + REG_LSR );
	if( uart_status & 0x0E ) {
		log_txrx_event( ctxt, TXRX_EVENT_RX_UART_ERR, uart_status, 0 );
	}
	rxbuf = hsuart_put_and_get_next_rx_buffer ( ctxt, rxbuf, rxbuf->size );

	omap_dma_chain_a_transfer( ctxt->rx_dma_ch, 
	                           ctxt->uart_phys,
	                           rxbuf->phys + rxbuf->len,
	                           rxbuf->size - rxbuf->len,
	                           1, rxbuf );

	ctxt->rx_q[ctxt->rx_q_head] = rxbuf;
	ctxt->rx_q_head  = 1 - ctxt->rx_q_head;
	ctxt->rx_ch_head = 1 - ctxt->rx_ch_head;
	hsuart_rx_timer_start ( ctxt, ctxt->rx_timeout );
	spin_unlock_irqrestore( &ctxt->lock, flags );
	
	return;
}

static void 
hsuart_tx_dma_callback (int lch, u16 ch_status, void *dev_id)
{
	struct txrx_buf *txbuf = (struct txrx_buf *)dev_id;
	struct dev_ctxt *ctxt  = (struct dev_ctxt *)txbuf->private;

	log_txrx_event( ctxt, TXRX_EVENT_TX_DMA_IRQ, lch, ch_status );
	if( ch_status ) {
		log_txrx_event( ctxt, TXRX_EVENT_TX_DMA_ERR, lch, ch_status );
	}
	ctxt->tx_ttl += txbuf->len;
	ctxt->tx_cnt++;
	log_txrx_event( ctxt, TXRX_EVENT_TX_XFER_DONE, txbuf->len , (u32) txbuf );
	hsuart_put_tx_buffer ( ctxt, txbuf ); // put prev xfer
	hsuart_start_tx_xfer ( ctxt ); // start next xfer if any
	
	return;
}

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
static int
hsuart_modem_busy_handler ( unsigned long dev_id ) 
{
	int rc;
	unsigned long flags;
	struct dev_ctxt *ctxt = (struct dev_ctxt *) dev_id;
	
	spin_lock_irqsave ( &ctxt->lock, flags );
	rc = hsuart_is_busy(ctxt);
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	PDBG3("uart%d: %s: %d\n", ctxt->uart_no, __func__, rc );
	
	return rc;
}

static int
hsuart_modem_awake_handler ( unsigned long dev_id ) 
{
	unsigned long flags;
	struct dev_ctxt *ctxt = (struct dev_ctxt *) dev_id;
	
	PDBG3("uart%d: %s:\n", ctxt->uart_no, __func__ );
	spin_lock_irqsave ( &ctxt->lock, flags );
	hsuart_get_clk(ctxt);
	hsuart_cancel_idle_timer(ctxt);
	hsuart_clr_rx_flow ( ctxt, HSUART_RX_FLOW_HAND );
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	
	return 0;
}

static int
hsuart_modem_sleep_handler ( unsigned long dev_id ) 
{
	unsigned long flags;
	struct dev_ctxt *ctxt = (struct dev_ctxt *) dev_id;
	
	PDBG3("uart%d: %s:\n", ctxt->uart_no, __func__);
	spin_lock_irqsave ( &ctxt->lock, flags );
	hsuart_set_rx_flow ( ctxt, HSUART_RX_FLOW_HAND );
	hsuart_set_idle_timer (ctxt, ctxt->idle_poll_timeout);
	spin_unlock_irqrestore ( &ctxt->lock, flags );
	
	return 0;
}
#endif

/************************************************************************
 *
 *	 IO calls
 *
 ************************************************************************/


static void
dump_chunk (u8 *p, int len) 
{
	while ( len-- ) {
		printk("%02x ", (int) *p++ );
	}
	printk("\n");
}

static ssize_t 
hsuart_read ( struct file *file, char __user *buf, size_t count, 
              loff_t *ppos )
{
	ssize_t rc, nbytes = 0, rx_len = 0;
	struct dev_ctxt *ctxt = file->private_data;
	struct txrx_buf *rxbuf;

	log_txrx_event( ctxt, TXRX_EVENT_RX_READ_ENTER, count, (u32) buf );

	if (file->f_flags & O_NONBLOCK) {
		if(!mutex_trylock(&ctxt->rx_mlock)) {
			rc = -EAGAIN;
			goto Exit;
		}
	} else {
		if( mutex_lock_interruptible(&ctxt->rx_mlock)) {
			rc = -ERESTARTSYS;
			goto Exit;
		}
	}

	PDBG3("%s[%d]: enter read: %d\n", __FUNCTION__, ctxt->uart_no, count);
	while ( nbytes < count ) {
		rxbuf = hsuart_get_rd_buffer ( ctxt, &rx_len );
		if( rxbuf ) { 
			if( rx_len > (count - nbytes))
				rx_len = (count - nbytes);

			if( copy_to_user( buf + nbytes, 
			                  rxbuf->virt + rxbuf->pos, 
			                  rx_len )) {
				hsuart_put_rd_buffer ( ctxt, rxbuf, 0);
				rc = -EFAULT;
				goto Done;
			}
			nbytes  += rx_len;
			hsuart_put_rd_buffer ( ctxt, rxbuf, rx_len );
			if( rx_len )
				continue;
		}
		
		// no more bytes, would block
		if( nbytes ) {
			rc = nbytes;
			goto Done;
		}

		if( file->f_flags & O_NONBLOCK) {
			rc = -EAGAIN;
			goto Done;
		}

		log_txrx_event( ctxt, TXRX_EVENT_RX_SLEEP_ENTER, 0, 0 );
		rc = wait_event_interruptible( ctxt->rx_wait, 
		                               hsuart_rx_has_bytes_locked(ctxt));
		log_txrx_event( ctxt, TXRX_EVENT_RX_SLEEP_EXIT,  0, 0 );
		if( rc < 0)
			goto Done;
	}
	rc = nbytes;
Done:
	mutex_unlock(&ctxt->rx_mlock);
Exit:
	log_txrx_event( ctxt, TXRX_EVENT_RX_READ_EXIT, rc, 0 );

	PDBG2("%s[%d]: rc = %d / %d\n", __FUNCTION__, ctxt->uart_no, rc, count);

	if( rc > 0 && ctxt->dbg_level >= 4 ) {
			printk("DATA CHUNK:");
			dump_chunk (buf, rc );
	}
	
	return rc;
}

static ssize_t 
hsuart_write ( struct file *file, const char __user *buf, size_t count, 
               loff_t *ppos )
{
	ssize_t rc, nbytes = 0;
	struct dev_ctxt *ctxt = file->private_data;
	struct txrx_buf *txbuf;
	
	log_txrx_event( ctxt, TXRX_EVENT_TX_WRITE_ENTER, count, (u32) buf );
	
	if (file->f_flags & O_NONBLOCK) {
		if(!mutex_trylock(&ctxt->tx_mlock)) {
			printk("uart%d: %s: tx_mutex locked\n", 
			        ctxt->uart_no, __FUNCTION__ );
			rc = -EAGAIN;
			goto Exit;
		}
	} else {
		if( mutex_lock_interruptible(&ctxt->tx_mlock)) {
			rc = -ERESTARTSYS;
			goto Exit;
		}
	}
	
#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
	if( ctxt->pdata->options & HSUART_OPTION_MODEM_DEVICE ) {
		int timeout = 4000;
	
		if( file->f_flags & O_NONBLOCK ) {
			timeout = 0;
		}
		
		rc = modem_activity_touch_uart_port ( ctxt->uart_no, 
		                                      timeout, __FUNCTION__ );
		if( rc < 0) {
			PDBG2("uart%d: %s: modem rc %d\n", 
			       ctxt->uart_no, __FUNCTION__, rc );
			goto Done;
		}
	}
#endif
	
	while ( nbytes < count ) {
		txbuf = hsuart_get_tx_buffer ( ctxt );
		if( txbuf ) { 
			size_t tx_len = txbuf->size;
	
			if( tx_len > (count - nbytes))
				tx_len = (count - nbytes);
			if( copy_from_user( txbuf->virt + txbuf->pos,
			                    buf + nbytes, tx_len )) {
				hsuart_put_tx_buffer ( ctxt, txbuf);
				rc = -EFAULT;
				goto Done;
			}
			txbuf->pos = 0;
			txbuf->len = tx_len;
			hsuart_enqueue_tx_buffer ( ctxt, txbuf );
			hsuart_start_tx_xfer ( ctxt );
			nbytes += tx_len;
			continue;
		}

		// no more buffers, would block
		if( nbytes ) {
			rc = nbytes;
			goto Done;
		}

		if( file->f_flags & O_NONBLOCK ) {
			PDBG3("uart%d: %s: would block\n", 
			       ctxt->uart_no, __FUNCTION__);
			rc = -EAGAIN;
			goto Done;
		}

		rc = wait_event_interruptible( ctxt->tx_wait, 
			                       hsuart_tx_has_space(ctxt));
		if( rc < 0)
			goto Done;
	}
	rc = nbytes;
Done:	 
	mutex_unlock(&ctxt->tx_mlock);
Exit:	

	log_txrx_event( ctxt, TXRX_EVENT_TX_WRITE_EXIT, rc, 0 );
	PDBG2("%s[%d]: rc = %d / %d\n", __FUNCTION__, ctxt->uart_no, rc, count );
	
	return rc;
}

static unsigned int 
hsuart_poll (struct file *file, struct poll_table_struct *wait)
{
	unsigned long flags;
	unsigned int  mask = 0;
	struct dev_ctxt *ctxt = file->private_data;

	poll_wait ( file, &ctxt->tx_wait, wait );
	poll_wait ( file, &ctxt->rx_wait, wait );

	spin_lock_irqsave ( &ctxt->lock, flags );
	if( hsuart_tx_has_space (ctxt))
		mask |= POLLOUT | POLLWRNORM;

	if( hsuart_rx_has_bytes (ctxt))
		mask |= POLLIN  | POLLRDNORM;
	spin_unlock_irqrestore ( &ctxt->lock, flags );

	return mask;
}


static int
hsuart_ioctl_flush ( struct dev_ctxt *ctxt, int args )
{
	int fifo;
	int tx_stopped = 0;
	int rx_stopped = 0;

	fifo = args & ( HSUART_TX_FIFO | HSUART_RX_FIFO);

	/* stop Tx and Rx */
	if((args & HSUART_TX_QUEUE) | fifo ) {
		hsuart_stop_tx_xfer ( ctxt );
		tx_stopped = 1;
	}
		
	if((args & HSUART_RX_QUEUE) | fifo ) {
		hsuart_stop_rx_xfer  ( ctxt );
		rx_stopped = 1;
	}

	/* clear queues */	
	if( args & HSUART_TX_QUEUE ) {
		hsuart_flush_tx_queue( ctxt );
	}
		
	if( args & HSUART_RX_QUEUE ) {
		hsuart_flush_rx_queue( ctxt );
	}

	/* clear FIFOs */
	if( fifo ) {
		hsuart_hw_flush_fifo  ( ctxt, fifo );
	}

	/* restart Tx-Rx*/
	if( tx_stopped ) {
		hsuart_start_tx_xfer ( ctxt );
	}
	
	if( rx_stopped ) {
		hsuart_start_rx_xfer ( ctxt );
	}
		
	return 0;
}


static int
hsuart_tx_do_drain ( struct dev_ctxt *ctxt, unsigned long timeout )
{
	int rc = 0;

	if( timeout == 0 ) {
		// non blocking case
		if(!hsuart_tx_is_empty ( ctxt ))
			rc |= 2; 
	
		if(!hsuart_tx_fifo_is_empty ( ctxt ))
			rc |= 1; 
	
		return rc; 
	}
	
	// timeout in jiffies
	timeout = msecs_to_jiffies ( timeout );
	
	rc = wait_event_interruptible_timeout ( ctxt->tx_wait,
	                                        hsuart_tx_is_empty(ctxt),
	                                        timeout );
	if( rc < 0 )
		return rc; // interrupted by signal or error
		
	if( rc == 0 )
		return 2;  // expired but condition was not reached
	
	timeout  = jiffies + rc;
	while (time_before(jiffies, timeout)) {
		if( hsuart_tx_fifo_is_empty ( ctxt )) {
			return 0;
		}
		msleep (1);
	}
	return 1;
}

static int
hsuart_ioctl_tx_drain ( struct dev_ctxt *ctxt, unsigned long timeout )
{
	int rc;

	if(!mutex_trylock( &ctxt->tx_mlock))
		return -EAGAIN; // we have active writer

	rc = hsuart_tx_do_drain ( ctxt, timeout );

	mutex_unlock ( &ctxt->tx_mlock );

	PDBG1("%s[%d]: %d\n", __FUNCTION__, ctxt->uart_no, rc );

	return rc;
}

static int
hsuart_ioctl_rx_bytes ( struct dev_ctxt *ctxt )
{
	int rc = 0;

	if(!mutex_trylock( &ctxt->rx_mlock))
		return -EAGAIN; // we have active reader

	if( hsuart_rx_has_bytes_locked (ctxt))
		rc |= 2;
	
	if( hsuart_rx_fifo_has_bytes (ctxt)) 
		rc |= 1;
	
	mutex_unlock ( &ctxt->rx_mlock );

	PDBG1("%s[%d]: %d\n", __FUNCTION__, ctxt->uart_no, rc );
	
	return rc;
}

static int
hsuart_ioctl_rx_flow ( struct dev_ctxt *ctxt, int opcode )
{
	PDBG1("%s[%d]: opcode=%d\n", 
	    __FUNCTION__, ctxt->uart_no, opcode );

	if( opcode == HSUART_RX_FLOW_ON ) 
		hsuart_clr_rx_flow ( ctxt, HSUART_RX_FLOW_USER );
	else
		hsuart_set_rx_flow ( ctxt, HSUART_RX_FLOW_USER );

	return 0;
}

static int
hsuart_ioctl_set_uart_mode ( struct dev_ctxt *ctxt, 
                             void *usr_ptr, int usr_bytes )
{
	int rc = 0;
	unsigned int changed;
	struct hsuart_mode mode;

	if( copy_from_user ( &mode, usr_ptr, usr_bytes )) {
		rc = -EFAULT; 
		goto Done;
	}

	PDBG1("%s: \n", __FUNCTION__ );

	if( mode.speed != ctxt->uart_speed ) { // speed changed
		rc = hsuart_set_speed ( ctxt, mode.speed );
		if( rc != 0 )
			goto Done;
	}
	changed = ctxt->uart_flags ^ mode.flags;
	if( changed & HSUART_MODE_LOOPBACK ) { // loopback changed
		ctxt->uart_flags &= ~HSUART_MODE_LOOPBACK;
		ctxt->uart_flags |=  mode.flags & HSUART_MODE_LOOPBACK;
	}
	if( changed & HSUART_MODE_FLOW_CTRL_MASK ) { // flow control changed
		ctxt->uart_flags &= ~HSUART_MODE_FLOW_CTRL_MASK;
		ctxt->uart_flags |=  mode.flags & HSUART_MODE_FLOW_CTRL_MASK;
	}
	if( changed & HSUART_MODE_PARITY_MASK ) { // parity changed
		ctxt->uart_flags &= ~HSUART_MODE_PARITY_MASK;
		ctxt->uart_flags |=  mode.flags & HSUART_MODE_PARITY_MASK;
	}

	if( changed ) {
		hsuart_hw_set_pfl ( ctxt, ctxt->uart_flags );
	}

	PDBG1("%s: done\n", __FUNCTION__ );
Done:
	return rc;

}

static int 
hsuart_ioctl( struct inode * inode, struct file *file, 
              unsigned int cmd, unsigned long args)
{
	int rc = 0;
	struct dev_ctxt * ctxt;
	void *usr_ptr   = (void*) (args);
	int   usr_bytes = _IOC_SIZE(cmd);

	ctxt = container_of ( file->f_op, struct dev_ctxt, fops );

	omap24xx_uart_clk_enable(ctxt->uart_no);

	switch ( cmd )	   {
		case  HSUART_IOCTL_GET_VERSION: 
		{
			int ver = DRIVER_VERSION;
			if( copy_to_user ( usr_ptr, &ver, usr_bytes )) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case  HSUART_IOCTL_GET_BUF_INF: 
		{
			struct hsuart_buf_inf binf;

			binf.rx_buf_num  = ctxt->rx_buf_num;
			binf.tx_buf_num  = ctxt->tx_buf_num;
			binf.rx_buf_size = ctxt->rx_buf_size;
			binf.tx_buf_size = ctxt->tx_buf_size;
			if( copy_to_user ( usr_ptr, &binf, usr_bytes )) {
				rc = -EFAULT; 
				goto Done;
			}
		}
		break;

		case HSUART_IOCTL_GET_STATS:
		{
			struct hsuart_stat stat;
			stat.tx_bytes	= ctxt->tx_ttl;
			stat.rx_bytes	= ctxt->rx_ttl;
			stat.rx_dropped = ctxt->rx_dropped;
			if( copy_to_user ( usr_ptr, &stat, usr_bytes )) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case  HSUART_IOCTL_GET_UARTMODE: 
		{
			struct hsuart_mode mode;
			mode.speed = ctxt->uart_speed;
			mode.flags = ctxt->uart_flags;
			if( copy_to_user ( usr_ptr, &mode, usr_bytes )) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case HSUART_IOCTL_SET_RXLAT:
			ctxt->pdata->rx_latency = args;
			hsuart_recalc_timeout( ctxt );
		break;
		
		case  HSUART_IOCTL_SET_UARTMODE:
			rc = hsuart_ioctl_set_uart_mode ( ctxt, usr_ptr, usr_bytes );
		break;

		case  HSUART_IOCTL_CLEAR_FIFO:
		case  HSUART_IOCTL_FLUSH:
			rc = hsuart_ioctl_flush ( ctxt, args );
		break;

		case  HSUART_IOCTL_TX_DRAIN:
			rc = hsuart_ioctl_tx_drain ( ctxt, args );
		break;

		case  HSUART_IOCTL_RX_BYTES:
			rc = hsuart_ioctl_rx_bytes ( ctxt );
		break;

		case  HSUART_IOCTL_RX_FLOW:
			rc = hsuart_ioctl_rx_flow  ( ctxt, args );
		break;

		case  HSUART_IOCTL_RESET_UART:
			PDBG1("%s: reset_uart\n", __FUNCTION__ );
		break;
		
	}
Done:	
	omap24xx_uart_clk_disable(ctxt->uart_no);
	return rc;
}

static int hsuart_open (struct inode *inode, struct file *file)
{
	struct dev_ctxt * ctxt;

	ctxt = container_of ( file->f_op, struct dev_ctxt, fops );

	PDBG1("%s[%d]:\n", __FUNCTION__, ctxt->uart_no );

	// check if it is in use
	if( test_and_set_bit ( 0, &ctxt->is_opened))
		return -EBUSY;

	if(!ctxt->is_initialized ) {
		int rc;

		omap24xx_uart_clk_enable(ctxt->uart_no);
		rc = hsuart_init_uart ( ctxt );
		if( rc ) {
			omap24xx_uart_clk_disable(ctxt->uart_no);
			clear_bit(0, &ctxt->is_opened );
			return rc;
		}
		hsuart_start_rx_xfer(ctxt);
		omap24xx_uart_clk_disable(ctxt->uart_no);
	}

	// attach private data 
	file->private_data = ctxt;

	ctxt->tx_ttl = 0;
	ctxt->rx_ttl = 0;
	ctxt->rx_dropped = 0;

	return nonseekable_open  ( inode, file );
}

static int hsuart_close (struct inode *inode, struct file *file)
{
	struct dev_ctxt *ctxt;

	ctxt = container_of ( file->f_op, struct dev_ctxt, fops );

	PDBG1("%s[%d]:\n", __FUNCTION__, ctxt->uart_no );

	/* mark it as unused */
	clear_bit(0, &ctxt->is_opened);
	
	return 0;
}

struct file_operations hsuart_def_fops = {
	.llseek  = no_llseek,
	.read    = hsuart_read,
	.write   = hsuart_write,
//	.fsync   = hsuart_fsync,
	.poll    = hsuart_poll,
	.ioctl   = hsuart_ioctl,
	.open    = hsuart_open,
	.release = hsuart_close,
};


static void
hsuart_free_rts_pin (struct dev_ctxt *ctxt)
{
	if( ctxt->pdata->rts_pin < 0 )
		return;
		
	gpio_free( ctxt->pdata->rts_pin );
}

static int __devexit
hsuart_remove ( struct platform_device *dev )
{
	struct dev_ctxt *ctxt = platform_get_drvdata ( dev );
	
	PDBG1("%s:\n", __FUNCTION__);

	platform_set_drvdata ( dev, NULL );

	/* remove debug attributes */
	hsuart_remove_debug_attrs( dev );

	omap24xx_uart_clk_enable(ctxt->uart_no);

	/* stop all operations */
	hsuart_stop_tx_xfer ( ctxt );
	hsuart_stop_rx_xfer ( ctxt );
	hsuart_put_clk (ctxt);

	/* Free tx/rx dma channels and buffers */
	hsuart_free_tx_dma  ( ctxt );
	hsuart_free_rx_dma  ( ctxt );

	/* release timers */
	hsuart_free_rx_timer( ctxt );

	/* release uart */
	if( ctxt->is_initialized )
		omap24xx_uart_release ( ctxt->uart_no );

	omap24xx_uart_clk_disable(ctxt->uart_no);

	/* unregister misc device */
	misc_deregister ( &ctxt->mdev );

	/* free rts pin if any */
	hsuart_free_rts_pin ( ctxt );

	kfree ( ctxt->tx_all_buffs );
	kfree ( ctxt->rx_all_buffs );
	kfree ( ctxt );

	return 0;	 
}


static void
__init hsuart_init_rts_pin (struct dev_ctxt *ctxt)
{
	int rc;
	
	if( ctxt->pdata->rts_pin < 0 )
		return;

	rc = gpio_request ( ctxt->pdata->rts_pin, "rts_pin" );
	if( rc ) {
		printk (KERN_WARNING "%s: Failed to request rts gpio pin %d", 
		        DRIVER, ctxt->pdata->rts_pin );
		return;
	}

	gpio_direction_input  ( ctxt->pdata->rts_pin ); // it will be maintained with pullup
//	gpio_direction_output ( ctxt->pdata->rts_pin, 1 ); // RTS is active low

	return;
}

static int
__init hsuart_init_queues ( struct dev_ctxt * ctxt )
{
	int i;
	u8* virt;
	dma_addr_t phys;
	struct txrx_buf *pbuf;

	/* init wait queues */
	init_waitqueue_head ( &ctxt->tx_wait ); 
	init_waitqueue_head ( &ctxt->rx_wait ); 

	/* init tx lists */
	ctxt->tx_free_cnt = 0;
	INIT_LIST_HEAD ( &ctxt->tx_free_list );
	INIT_LIST_HEAD ( &ctxt->tx_xfer_list );

	ctxt->tx_all_buffs = kzalloc ( ctxt->tx_buf_num * sizeof(struct txrx_buf), GFP_KERNEL );
	if( ctxt->tx_all_buffs == NULL )  
		return -ENOMEM;

	virt = (u8*) ctxt->tx_dma_virt;
	phys = ctxt->tx_dma_phys;
	pbuf = ctxt->tx_all_buffs;
	for( i = 0; i < ctxt->tx_buf_num; i++ )
	{	/* Init entry */
		pbuf->virt = virt;
		pbuf->phys = phys;
		pbuf->pos  = 0;
		pbuf->len  = 0;
		pbuf->size = ctxt->tx_buf_size;
		pbuf->state = TXRX_STATE_IDLE;
		pbuf->private = ctxt;
		/* Add it to free list */	  
		list_add_tail ( &pbuf->link, &ctxt->tx_free_list );
		ctxt->tx_free_cnt++;
		/* next one */
		pbuf += 1;
		virt += ctxt->tx_buf_size;
		phys += ctxt->tx_buf_size;
	}

	ctxt->rx_all_buffs = kzalloc ( ctxt->rx_buf_num * sizeof(struct txrx_buf), GFP_KERNEL );
	if( ctxt->rx_all_buffs == NULL )  
		return -ENOMEM;

	/* init rx lists */
	INIT_LIST_HEAD ( &ctxt->rx_free_list );
	INIT_LIST_HEAD ( &ctxt->rx_xfer_list );

	virt = (u8*) ctxt->rx_dma_virt;
	phys = ctxt->rx_dma_phys;
	pbuf = ctxt->rx_all_buffs;
	for( i = 0; i < ctxt->rx_buf_num; i++ )
	{	/* Init entry */
		pbuf->virt = virt;
		pbuf->phys = phys;
		pbuf->pos  = 0;
		pbuf->len  = 0;
		pbuf->size = ctxt->rx_buf_size;
		pbuf->state = TXRX_STATE_IDLE;
		pbuf->private = ctxt;
		/* Add it to free list */
		list_add_tail ( &pbuf->link, &ctxt->rx_free_list );
		/* next one */
		pbuf += 1;
		virt += ctxt->rx_buf_size;
		phys += ctxt->tx_buf_size;
	}
	return 0;
}

static int __init 
hsuart_probe ( struct platform_device  *dev )
{
	int rc = 0;
	struct hsuart_platform_data *pdata;
	struct dev_ctxt *ctxt = NULL;
	unsigned long flags;

	pdata = dev->dev.platform_data;
	if( pdata == NULL ) {
		printk (KERN_ERR "%s: no platform data\n", DRIVER );
		return -ENODEV;
	}

	ctxt = kzalloc ( sizeof(struct dev_ctxt), GFP_KERNEL );
	if( ctxt == NULL ) 
		return -ENOMEM;

	// attach ctxt context to it to device
	platform_set_drvdata ( dev, ctxt );

	/* Attach platform device */
	ctxt->pdev  = dev; 
	ctxt->pdata = pdata;
	ctxt->uart_flags  = pdata->uart_mode;
	ctxt->uart_speed  = pdata->uart_speed;

	/* */
	ctxt->dbg_log_mask = event_log_mask;   // default log mask;
	ctxt->dbg_level    = pdata->dbg_level; // default debug level

	ctxt->rx_buf_shift = fls(pdata->rx_buf_size) - 1;
	ctxt->tx_buf_size  = (1 << (fls(pdata->tx_buf_size)-1));
	ctxt->rx_buf_size  = (1 << (fls(pdata->rx_buf_size)-1));
	ctxt->tx_buf_num   =  pdata->tx_buf_num;
	ctxt->rx_buf_num   =  pdata->rx_buf_num;

	ctxt->dev_name = pdata->dev_name;
	if( ctxt->dev_name == NULL ) 
		ctxt->dev_name = dev->name;

	/* id is reused as uart_no */
	ctxt->uart_no   = dev->id;
	ctxt->uart_phys = (dma_addr_t)     omap24xx_uart_base_p(ctxt->uart_no);
	ctxt->uart_base = (void __iomem *) omap24xx_uart_base_v(ctxt->uart_no);

	/* init rts pin  */
	hsuart_init_rts_pin ( ctxt );

	/* Allocate TX DMA */
	rc = hsuart_alloc_tx_dma   ( ctxt );
	if( rc ) 
		goto err_free_ctxt;

	/* Allocate RX DMA */
	rc = hsuart_alloc_rx_dma   ( ctxt );
	if( rc ) 
		goto err_free_tx_dma;

	/* Allocate RX Timer */
	rc = hsuart_alloc_rx_timer ( ctxt );
	if( rc )
		goto err_free_rx_dma;

	/* Init queues */
	rc = hsuart_init_queues ( ctxt );
	if( rc ) 
		goto err_free_rx_timer;

	/* Init misc device */
	memcpy ( &ctxt->fops, &hsuart_def_fops, sizeof(struct file_operations));
	ctxt->mdev.minor = MISC_DYNAMIC_MINOR;
	ctxt->mdev.name  = ctxt->dev_name;
	ctxt->mdev.fops  = &ctxt->fops;

	/* Register misc device */
	rc = misc_register ( &ctxt->mdev );
	if( rc ) 
		goto err_free_rx_timer;

	/* Init spinlock */
	spin_lock_init ( &ctxt->lock );
	mutex_init ( &ctxt->tx_mlock );
	mutex_init ( &ctxt->rx_mlock );

	/* Inactivity timer */
	setup_timer( &ctxt->idle_timer, hsuart_idle_timeout,(unsigned long)ctxt);
	ctxt->idle_timeout      =  msecs_to_jiffies(pdata->idle_timeout);
	ctxt->idle_poll_timeout =  msecs_to_jiffies(pdata->idle_poll_timeout);

	printk( KERN_INFO "%s: creating '%s' device on UART %d\n", 
	        DRIVER, ctxt->dev_name, ctxt->uart_no );

	/* create debug attributes */
	(void) hsuart_create_debug_attrs ( dev );

	/* start recv */
	if(!(ctxt->pdata->options & HSUART_OPTION_DEFERRED_LOAD)) {
		rc = hsuart_init_uart ( ctxt );
		if( rc ) {
			goto err_misc_unregister;
		}
		hsuart_start_rx_xfer(ctxt);
	}

	/* Stop RX timer even though it's not running at this point.  Stopping
	 * the timer explicitely here will turn off its clocks to save power.
	 */
	spin_lock_irqsave ( &ctxt->lock, flags );
	hsuart_rx_timer_stop ( ctxt );
	spin_unlock_irqrestore ( &ctxt->lock, flags );

	/* Release the clocks. At this point there should be only one ref on
	 * the clocks from early boot code so this should turn the clocks off.
	 */
	omap24xx_uart_clk_disable(ctxt->uart_no);

	return 0;


err_misc_unregister:
	misc_deregister ( &ctxt->mdev );

err_free_rx_timer:
	hsuart_free_rx_timer ( ctxt );

err_free_rx_dma:
	hsuart_free_rx_dma   ( ctxt );

err_free_tx_dma:
	hsuart_free_tx_dma   ( ctxt );

	hsuart_free_rts_pin  ( ctxt );

err_free_ctxt:
	kfree ( ctxt->tx_all_buffs );
	kfree ( ctxt->rx_all_buffs );
	kfree ( ctxt );

	printk( KERN_ERR"%s: Failed (%d) to initialize device\n", 
	        DRIVER, rc );
	
	return rc;	  
}

#ifdef CONFIG_PM
static int 
hsuart_suspend( struct platform_device *dev, pm_message_t state )
{
	struct dev_ctxt *ctxt = platform_get_drvdata(dev);

	PDBG1("uart%d: suspend\n", ctxt->uart_no);

	/* nothing to do if device is not initialized. */
	if (!ctxt->is_initialized) {
		return 0;
	}

	omap24xx_uart_clk_enable(ctxt->uart_no);

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
	if( ctxt->pdata->options & HSUART_OPTION_MODEM_DEVICE ) {
		hsuart_set_rx_flow ( ctxt, HSUART_RX_FLOW_HAND );
	}
#endif

	/* stop all operations */
	hsuart_cancel_idle_timer(ctxt);
	hsuart_stop_tx_xfer(ctxt);
	hsuart_stop_rx_xfer(ctxt);
	hsuart_clr_wkup(ctxt);
	hsuart_put_clk (ctxt);
	hsuart_flush_rx_queue(ctxt);

	/* Force idle UART in suspend. */
	hsuart_set_sysconfig(ctxt, 0x0001);

	/* disable uart */
	hsuart_stop_uart(ctxt);

	omap24xx_uart_clk_disable(ctxt->uart_no);
	return 0;
}

static int 
hsuart_resume ( struct platform_device *dev )
{
	struct dev_ctxt *ctxt = platform_get_drvdata(dev);

	PDBG1("uart%d: resume: %08d\n", ctxt->uart_no, omap_32k_sync_timer_read());

	/* nothing to do if device is not initialized. */
	if (!ctxt->is_initialized) {
		return 0;
	}

	omap24xx_uart_clk_enable(ctxt->uart_no);

	ctxt->tx_cnt = 0;

	/* reset module, set it to noIdle */
	hsuart_module_reset  ( ctxt );
	hsuart_set_sysconfig ( ctxt, 0x0009 ); 

	/* reconfig uart */
	hsuart_config_uart ( ctxt );

	/* set uart speed */
	hsuart_set_speed  ( ctxt, ctxt->uart_speed );

	/* set parity, flow control, loopback */
	hsuart_hw_set_pfl ( ctxt, ctxt->uart_flags );

	/* reconfigure timer */
	hsuart_configure_timer(ctxt);
	hsuart_recalc_timeout (ctxt);

//	hsuart_get_clk(ctxt);
//	hsuart_set_idle_timer(ctxt, ctxt->idle_timeout);

	hsuart_start_tx_xfer(ctxt); // start next xfer if any
	hsuart_start_rx_xfer(ctxt); // start rx
	
	omap24xx_uart_clk_disable(ctxt->uart_no);

	return 0;
}
#else
#define hsuart_suspend	NULL
#define hsuart_resume	NULL
#endif	/* CONFIG_PM */

static struct platform_driver hsuart_driver = {
	.driver   = {
		.name = DRIVER,
	},
	.probe	  = hsuart_probe,
	.remove   = __devexit_p(hsuart_remove),
	.suspend  = hsuart_suspend,
	.resume   = hsuart_resume,
};

/*
 *
 */
static int __init hsuart_init(void)
{
	return platform_driver_register ( &hsuart_driver );
}

/*
 *
 */
static void __exit hsuart_exit(void)
{
	platform_driver_unregister ( &hsuart_driver );
}

module_init(hsuart_init);
module_exit(hsuart_exit);

MODULE_DESCRIPTION("OMAP High speed UART platform driver");
MODULE_LICENSE("GPL");

