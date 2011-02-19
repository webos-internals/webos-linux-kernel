/*
 * Includes
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/spi/spi.h>
#include <linux/spi/cy8mrln.h>
#ifdef CONFIG_HIGH_RES_TIMERS
#include <linux/hrtimer.h>
#endif
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/gpio.h>

#define CONFIG_EXTERNAL_CS

#undef  MODDEBUG
#define MODDEBUG

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

#undef DO_MEASUREMENTS
//#define DO_MEASUREMENTS

#define TIMESTAMP_SIZE_BYTES       (sizeof(struct timespec))

/*
 * Constants
 */
#define  TSC_NAME       "cy8mrln"
#define  DEVICE         TSC_NAME
#define  DRIVER         TSC_NAME
#define  DRIVER_DESC    TSC_NAME" touchscreen driver"

/* Time conversion defines. */
#define  TSC_HZ_TO_USEC(x) (1000000/(x))
#define  TSC_HZ_TO_NSEC(x) (1000000000/(x))
#define  TSC_USEC_TO_HZ(x) (1000000/(x))
#define  TSC_USEC_TO_MILLIHZ(x) (1000000000/(x))
#define  TSC_USEC_TO_MILLIHZ_FRACTION(x) (TSC_USEC_TO_MILLIHZ(x) -\
					  TSC_USEC_TO_HZ(x)*1000)
#define  TSC_USEC_TO_ROUNDED_HZ(x) ((TSC_USEC_TO_MILLIHZ_FRACTION(x) >= 500) ?\
				    (TSC_USEC_TO_HZ(x) + 1) : TSC_USEC_TO_HZ(x))

/* Short hands */
#define  CY8MRLN_NUM_DATA_BYTES (dev->num_data_bytes)
#define  VECTOR(x) 	(dev->vector[x].data)
#define  BITS(x)	(dev->vector[x].bits)
#define  DATAOUT(x)	(dev->target_data_out[x])

/* Scan time outs are arbitrarily set to 0.5 seconds. */ 
#define  SCAN_TIMEOUT_NSEC	500000000
#define  SCAN_TIMEOUT_USEC	500000
#define  SCAN_TIMEOUT_JIFFY	(HZ/2)

/* Normal power-up takes a maximum of 50ms after which the
*  PSoC will be ready to accept commands
 */
#define  NORMAL_POWER_UP_TIMEOUT_MSEC  50
#define  NORMAL_POWER_UP_TIMEOUT_JIFFY	(NORMAL_POWER_UP_TIMEOUT_MSEC*HZ/1000)

/* Post programming power-up can take up to 5 seconds since
* the IDAC must be calibrated.  We double that and take 10s 
* just to be sure.  Programming takes 20 to 30 seconds anway 
* so the extra 5 seconds seems acceptable.
*/
#define  POST_PROG_POWER_UP_TIMEOUT_MSEC  10000
#define  POST_PROG_POWER_UP_TIMEOUT_JIFFY (POST_PROG_POWER_UP_TIMEOUT_MSEC*HZ/1000)


/* Timer setup overhead. It was measured just before the
 * hrtimer setup and again upon receiving of the timer exp
 * interrupt. If the time delay until the next scheduled
 * scan is less or equal to this, we will issue the next
 * scan immediately. Since there are function calls involved
 * to measure the time and print it out, this value is
 * conservative. This means the next scan command will be
 * issued rather earlier than the scheduled time if there
 * is timing contention.
 */
#define  TIMER_SETUP_NSEC	121951

/* The upper bound measurements of the time the PSoC takes
 * to process sleep and wake up commands. These values need
 * to be measured for each official firmware release to
 * optimize the performance. Based on these numbers, we
 * decide whether the PSoC should go to sleep in between
 * each scan. These values are conservative because they
 * include the overhead of ktime and printk function calls.
 *
 * I think the measurements overhead largely impact these
 * numbers. It will be best to get these numbers from Cypress.
 * Oh well..
 */ 
#define  WAKE_UP_TIME_NSEC      152439
#define  SLEEP_TIME_NSEC        121951

/* PSOC Chip Select (CS) pulse width:
 * 1. to eject from WOT mode: min. 1 msec (TP FW version 210F+ is needed) 
 * 2. to clear PSoC data buffer and return to
 *    idle state where new commands can be received:
 *    CS width is 15 usec.
 */

#define PULSE_PSOC_WOT_EJECT 1500
#define PULSE_PSOC_CS_TOGGLE 15

/* These numbers were chosen to fit one page. */ 
#define  CY8MRLN_BUF_LEN	256
#define  CY8MRLN_RX_BUF_NUM	15
#define  CY8MRLN_TX_BUF_NUM	1	

#define  TXRX_STATE_IDLE       (0x0)
#define  TXRX_STATE_RW_LOCKED  (0x1)
#define  TXRX_STATE_TX_LOCKED  (0x2)
#define  TXRX_STATE_RX_LOCKED  (0x4)

/* TX/RX buffer structure. */
typedef struct txrx_buf {
	size_t     pos;
	size_t     len;
	u32        state;
	unsigned char *ptr;
	struct list_head link;
} txrx_buf_t;

/* flags definitions. */
enum {
	GPIO_IRQ_DISABLED = 0,
	SUSPENDED,
	IS_OPENED,
	DATA_ACQUIRED,
	VERSION_ACQUIRED,
	INTR_HANDLED,
	SCAN_TIMEOUT,
	IN_QUICK_NAP,
	IDAC_CALIBRATED,
	IDAC_CALIBRATION_STORED,
	ILO_CALIBRATED
};

/*
 * Definitions and Prototypes
 */
typedef struct tsc_drv_data {
	struct spi_device		*spidev;
	struct cy8mrln_platform_data	*pdata;
#ifdef CONFIG_HIGH_RES_TIMERS
	struct hrtimer			scan_timer;
	struct hrtimer			timeout;
#else
	struct timer_list		scan_timer;
	struct timer_list		timeout;
#endif
	struct workqueue_struct *tp_wq; /* touchpanel/touchscreen Work Queue */
	struct work_struct		workq;
	struct work_struct		scanq;
	int				scan_rate;
	int 			wot_rawdata_mode;
	int				auto_scan;
	int				wot_sen;
	int				wot_scanrate;
	int				wot_baseline;
	int				wot_baseline_lo;
	int				wot_baseline_hi;
	int				sleep_mode;
	int             saved_mode;
	int				setup;
	int				xfer_option;
	int				read_option;
	int				verbose;
	int				timestamp;
	int				num_data_bytes;
	int				prog_phase;
	struct timespec			start_time;
	struct timespec			start_time_prev;
	struct list_head		rx_free_list;
	struct list_head		rx_xfer_list;
	struct txrx_buf			all_buffs[CY8MRLN_RX_BUF_NUM + CY8MRLN_TX_BUF_NUM];
	wait_queue_head_t 		version_wait;
	wait_queue_head_t 		rx_wait;
	wait_queue_head_t 		powerup_wait;
	wait_queue_head_t 		interrupt_wait;
	struct txrx_buf			*txbuf;
	struct file_operations		fops;
	struct miscdevice		mdev;
	spinlock_t			issplock;
	spinlock_t			buflock;
	struct mutex			mutex;
	struct mutex			scan_mutex;
	unsigned char			cmd;
	/* HACK: not all Idac tables are created equal (in size) */
	unsigned char			rev[CY8MRLN_BUF_LEN];
	cy8mrln_vector_t		vector[NUM_VECTORS];
	cy8mrln_fw_data_t		fw[NUM_FW_LINES];
	unsigned char			target_data_out[TARGET_DATABUFF_LEN];
	DECLARE_BITMAP(flags, 12);
	u8						tp_status_reg0;
} tsc_drv_data_t;

static int cy8mrln_reg_write(struct tsc_drv_data *dev, u16 reg, u8 data);
static int cy8mrln_write_three_bytes(struct tsc_drv_data *dev, u32 data);
static int cy8mrln_setup_and_read_data(struct tsc_drv_data *dev, u16 reg, u8 *txbuf, u8 *rxbuf, u8 length);
static int cy8mrln_read_data(struct tsc_drv_data *dev, u16 reg, u8 *txbuf, u8 *rxbuf, u8 length);
static int cy8mrln_setup_read(struct tsc_drv_data *dev, u16 reg);


static void sda_out(struct tsc_drv_data *dev, int val);

////////////////
/*
 * Sysfs area
 */

static ssize_t 
scan_rate_show(	struct device *dev, 
		struct device_attribute *attr, 
		char *buf)
{
	struct tsc_drv_data* 	p_dev;
	struct spi_device*	p_spi = (struct spi_device*) dev;

	/* get device */
	p_dev = spi_get_drvdata(p_spi);
	return snprintf(buf, PAGE_SIZE, "%d\n", (p_dev ? p_dev->scan_rate: 0xDEAD));
}

static ssize_t
scan_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct tsc_drv_data* 	p_dev;
	struct spi_device*	p_spi = (struct spi_device*) dev;

	/* get device */
	p_dev = spi_get_drvdata(p_spi);
	if (p_dev) {
		p_dev->scan_rate = simple_strtol(buf, NULL, 10);
	}
	return count;
}

static ssize_t 
tp_fw_ver_show(	struct device *dev, 
		struct device_attribute *attr, 
		char *buf)
{
	unsigned short rev;
	struct tsc_drv_data* 	p_dev;
	struct spi_device*	p_spi = (struct spi_device*) dev;

	/* get device */
	p_dev = spi_get_drvdata(p_spi);

	rev = (p_dev->rev[2] << 8) | p_dev->rev[3];
	return snprintf(buf, PAGE_SIZE, "%04x\n", (p_dev ? rev: 0xDEAD));
}

static ssize_t 
tp_mode_show(	struct device *dev, 
		struct device_attribute *attr, 
		char *buf)
{
	struct tsc_drv_data* 	p_dev;
	struct spi_device*	p_spi = (struct spi_device*) dev;

	/* get device */
	p_dev = spi_get_drvdata(p_spi);
	return snprintf(buf, PAGE_SIZE, "%d\n", (p_dev ? p_dev->sleep_mode: 0xDEAD));
}

static DEVICE_ATTR(scan_rate, S_IRUGO | S_IWUSR, scan_rate_show, scan_rate_store);
static DEVICE_ATTR(tp_mode, S_IRUGO, tp_mode_show, NULL );
static DEVICE_ATTR(tp_fw_ver, S_IRUGO, tp_fw_ver_show, NULL );
////////////////


/*
 *  Enable IRQ
 */
static void 
cy8mrln_enable_irq(struct tsc_drv_data *dev)
{
	if(dev->verbose )
	{
		printk("%s: EnableIrq. \n", DRIVER );
	}
	if (test_and_clear_bit(GPIO_IRQ_DISABLED, dev->flags)) {
		enable_irq(dev->spidev->irq);
	}
}

/*
 *  Disable IRQ
 */
static void 
cy8mrln_disable_irq(struct tsc_drv_data *dev)
{
	if(dev->verbose )
	{
		printk("%s: DisableIrq. \n", DRIVER );
	}
	if (!test_and_set_bit(GPIO_IRQ_DISABLED, dev->flags)) {
		disable_irq(dev->spidev->irq);
	}
}

/*
 *  cy8mrln sync chip select toggle routine
 */
static int 
cy8mrln_toggle_cs(struct tsc_drv_data *dev, u32 delay_usecs)
{
#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 0);
	udelay(delay_usecs);
	gpio_set_value(dev->pdata->external_cs, 1);
#else
	struct spi_message  m;
	struct spi_transfer t;
	u32 err;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	t.bits_per_word = 8;
	t.tx_buf = NULL;
	t.rx_buf = NULL;

	t.len    = 0;
	t.delay_usecs = delay_usecs ;
	spi_message_add_tail(&t, &m);

	err = spi_sync(dev->spidev, &m);

	if (dev->verbose) {
		printk("%s: Generated CS pulse\n", DRIVER);
	}
	return err;
#endif
}

/*
 *  cy8mrln sync write register routine
 */
static int 
cy8mrln_reg_write(struct tsc_drv_data *dev, u16 reg, u8 data)
{
	struct spi_message  m;
	struct spi_transfer t[2];
	u32 err;
	u8 tx[3], rx[3];

	/* Remember the command (scan vs. query). */
	dev->cmd = data;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	/* 16 bit wide address */
	tx[0] = (reg >> 8) & 0xff;
	tx[1] = reg & 0xff; 

	/* Data we want to write */
	tx[2] = data; 

#if 0
	/* Since the PSoC runs a lot slower than the host, it needs
	 * some time to get ready after the chip select gets
	 * asserted. Using an empty transfer, insert 15 usecs of
	 * delay after CS assertion. The total delay caused by this
	 * was measured to be ~21 usecs. Even though 18 usecs seems
	 * to be a sufficient delay, the extra 3 usecs will help
	 * with reliability without impacting the performance.
	 */
	t[0].bits_per_word = 8;
	t[0].tx_buf = NULL;
	t[0].rx_buf = NULL;
	t[0].len    = 0;
	t[0].delay_usecs = 15;
	spi_message_add_tail(&t[0], &m);
#endif

	/* Real SPI data to transfer. */
	t[1].bits_per_word = 8;
	t[1].tx_buf = tx;
	t[1].rx_buf = rx;
	t[1].len    = 3;
	t[1].delay_usecs = 0;
	spi_message_add_tail(&t[1], &m);

#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 0);
#endif
	err = spi_sync(dev->spidev, &m);
#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 1);
#endif

	if (dev->verbose) {
		printk("%s: reg_write tx: %02x %02x %02x\n",
		       DRIVER, tx[0], tx[1], tx[2]);
		printk("%s: reg_write rx: %02x %02x %02x\n",
		       DRIVER, rx[0], rx[1], rx[2]);
		printk("%s: retval:%d\n", DRIVER, err);
	}

	return err;
}

/*
 *  cy8mrln sync write register routine
 */
static int 
cy8mrln_write_three_bytes(struct tsc_drv_data *dev, u32 data)
{
	struct spi_message  m;
	struct spi_transfer t[2];
	u32 err;
	u8 tx[3], rx[3];

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	/* 16 bit wide address */
	tx[0] = (data >> 16) & 0xff; 
	tx[1] = (data >> 8) & 0xff;
	tx[2] = data & 0xff;

#if 0
	/* Since the PSoC runs a lot slower than the host, it needs
	 * some time to get ready after the chip select gets
	 * asserted. Using an empty transfer, insert 15 usecs of
	 * delay after CS assertion. The total delay caused by this
	 * was measured to be ~21 usecs. Even though 18 usecs seems
	 * to be a sufficient delay, the extra 3 usecs will help
	 * with reliability without impacting the performance.
	 */
	t[0].bits_per_word = 8;
	t[0].tx_buf = NULL;
	t[0].rx_buf = NULL;
	t[0].len    = 0;
	t[0].delay_usecs = 15;
	spi_message_add_tail(&t[0], &m);
#endif

	/* Real SPI data to transfer. */
	t[1].bits_per_word = 8;
	t[1].tx_buf = tx;
	t[1].rx_buf = rx;
	t[1].len    = 3;
	t[1].delay_usecs = 0;
	spi_message_add_tail(&t[1], &m);

#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 0);
#endif
	err = spi_sync(dev->spidev, &m);
#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 1);
#endif

	if (dev->verbose) {
		printk("%s: write_three_bytes tx: %02x %02x %02x\n",
		       DRIVER, tx[0], tx[1], tx[2]);
		printk("%s: write_three_bytes rx: %02x %02x %02x\n",
		       DRIVER, rx[0], rx[1], rx[2]);
	}
	return err;
}

/*
 *  cy8mrln sync read given consecutive register routine
 */
static int 
cy8mrln_setup_and_read_data(struct tsc_drv_data *dev, u16 reg,
			   u8 *txbuf, u8 *rxbuf, u8 length)
{   
	struct spi_message m;
	struct spi_transfer t;
	u32 err;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	/* 16 bit wide address */
	if (dev->read_option != CY8MRLN_ONE_INT_ZERO_SETUP_BYTES) {
		txbuf[0] = SPI_READ_BIT | ((reg >> 8) & 0xff);
		txbuf[1] = reg & 0xff;
	}

	/* For data read, we don't need extra delay between chip select
	 * assertion and the first edge of the SPI clock.
	 */
	t.bits_per_word = 8;
	t.tx_buf = txbuf;
	t.rx_buf = rxbuf;
	t.len    = length;
	t.delay_usecs = 0;

	spi_message_add_tail(&t, &m);
#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 0);
#endif
	err = spi_sync(dev->spidev, &m);
#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 1);
#endif

	return err;
}

/*
 *  cy8mrln sync read given consecutive register routine
 */
static int 
cy8mrln_read_data(struct tsc_drv_data *dev, u16 reg, u8 *txbuf,
		 u8 *rxbuf, u8 length)
{   
	struct spi_message m;
	struct spi_transfer t;
	u32 err;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	/* For data read, we don't need extra delay between chip select
	 * assertion and the first edge of the SPI clock.
	 */
	t.bits_per_word = 8;
	t.tx_buf = txbuf;
	t.rx_buf = rxbuf;
	t.len    = length;
	t.delay_usecs = 0;

	spi_message_add_tail(&t, &m);

#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 0);
#endif
	err = spi_sync(dev->spidev, &m);
#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 1);
#endif


	return err;
}

/*
 *  cy8mrln sync read given consecutive register routine
 */
static int 
cy8mrln_setup_read(struct tsc_drv_data *dev, u16 reg)
{   
	struct spi_message  m;
	struct spi_transfer t[2];
	u32 err;
	u8 tx[3], rx[3];

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	/* 16 bit wide address */
	tx[0] = SPI_READ_BIT | ((reg >> 8) & 0xff);
	tx[1] = reg & 0xff; 

	/* Data we want to write */
	tx[2] = 0; 

	/* Since the PSoC runs a lot slower than the host, it needs
	 * some time to get ready after the chip select gets
	 * asserted. Using an empty transfer, insert 15 usecs of
	 * delay after CS assertion. The total delay caused by this
	 * was measured to be ~21 usecs. Even though 18 usecs seems
	 * to be a sufficient delay, the extra 3 usecs will help
	 * with reliability without impacting the performance.
	 */
	t[0].bits_per_word = 8;
	t[0].tx_buf = NULL;
	t[0].rx_buf = NULL;
	t[0].len = 0;
	t[0].delay_usecs = 15;
	spi_message_add_tail(&t[0], &m);

	/* Real SPI data to transfer. */
	t[1].bits_per_word = 8;
	t[1].tx_buf = tx;
	t[1].rx_buf = rx;
	if (dev->read_option == CY8MRLN_TWO_INT_THREE_SETUP_BYTES) {
		t[1].len = 3;
	} else {
		t[1].len = 2;
	}
	t[1].delay_usecs = 0;
	spi_message_add_tail(&t[1], &m);

#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 0);
#endif
	err = spi_sync(dev->spidev, &m);
#ifdef CONFIG_EXTERNAL_CS
	gpio_set_value(dev->pdata->external_cs, 1);
#endif

	if (dev->verbose) {
		if (dev->read_option == CY8MRLN_TWO_INT_THREE_SETUP_BYTES) {
			printk("%s: read_setup tx: %02x %02x %02x\n", DRIVER,
			       tx[0], tx[1], tx[2]);
			printk("%s: read_setup rx: %02x %02x %02x\n", DRIVER,
			       rx[0], rx[1], rx[2]);
		} else {
			printk("%s: read_setup tx: %02x %02x\n", DRIVER, tx[0],
			       tx[1]);
			printk("%s: read_setup rx: %02x %02x\n", DRIVER, rx[0],
			       rx[1]);
		}
	}
	return err;
}

/* Return the number of data in the rx_xfer_list in byte. */
static inline int
cy8mrln_rx_has_bytes(struct tsc_drv_data *dev)
{
	size_t count = 0;
	txrx_buf_t *pbuf;

	list_for_each_entry(pbuf, &dev->rx_xfer_list, link) {
		count += pbuf->len - pbuf->pos;
	}
	return count;
}

/* Get the read buffer at the head of the rx_xfer_list. */
static txrx_buf_t *
cy8mrln_get_rd_buffer(struct tsc_drv_data *dev, size_t *bytes)
{
	txrx_buf_t *rxbuf = NULL;

	spin_lock(&dev->buflock);
	if(!list_empty(&dev->rx_xfer_list))  {
		rxbuf = list_first_entry(&dev->rx_xfer_list, txrx_buf_t, link);
		if (rxbuf->state == TXRX_STATE_IDLE) {
			*bytes = rxbuf->len - rxbuf->pos;
			if (*bytes) {
				rxbuf->state |= TXRX_STATE_RW_LOCKED;
			} else {
				rxbuf = NULL;
			}
		} else {
			/* This buffer is busy */
			rxbuf = NULL;
		}
	}
	spin_unlock(&dev->buflock);
	return rxbuf;
}

/* Update the read buffer. If the buffer is all read out,
 * return it to free list.
 */ 
static void
cy8mrln_put_rd_buffer(struct tsc_drv_data *dev, txrx_buf_t *rxbuf, int rx_len)
{
	spin_lock(&dev->buflock);
	rxbuf->pos += rx_len;
	rxbuf->state &= ~TXRX_STATE_RW_LOCKED;
	if((rxbuf->pos == rxbuf->len) && 
	   (rxbuf->state == TXRX_STATE_IDLE)) { // return it to free pool
		list_move_tail(&rxbuf->link, &dev->rx_free_list);
	}
	spin_unlock(&dev->buflock);
}

static void
cy8mrln_flush_rd_buffers(struct tsc_drv_data *dev)
{
	txrx_buf_t *rxbuf, *n;

	spin_lock(&dev->buflock);
	list_for_each_entry_safe(rxbuf, n, &dev->rx_xfer_list, link) {
		list_move_tail(&rxbuf->link, &dev->rx_free_list);
	}
	spin_unlock(&dev->buflock);
}

/* Get the next available buffer from free list. */
static struct txrx_buf*
cy8mrln_get_rx_buffer(struct tsc_drv_data *dev)
{
	struct txrx_buf *next_buf = NULL;

	spin_lock(&dev->buflock);

	/* Get the first entry of free list. */
	if (!(list_empty(&dev->rx_free_list))) {

		next_buf = list_first_entry(&dev->rx_free_list, txrx_buf_t, link);
		list_move_tail(&next_buf->link, &dev->rx_xfer_list);

	/* If there is no space in the free list, clobber the oldest
	 * entry from the xfer_list.
	 */
	} else {
		/* queue next buffer */
		dev->tp_status_reg0 |= DRV_STATUS_MASK__BUF_OVERFLOW ;		
		next_buf = list_first_entry(&dev->rx_xfer_list, txrx_buf_t, link);
		list_move_tail(&next_buf->link, &dev->rx_free_list);
		next_buf = list_first_entry(&dev->rx_free_list, txrx_buf_t, link);
		list_move_tail(&next_buf->link, &dev->rx_xfer_list);
	}
	next_buf->pos = 0;
	next_buf->len = 0;
	next_buf->state |= TXRX_STATE_RX_LOCKED;

	spin_unlock(&dev->buflock);
	return next_buf;
}

/* Buffer more data to the rx buffer given in the argument. */
static void 
cy8mrln_put_rx_buffer(struct tsc_drv_data *dev, txrx_buf_t *rxbuf, int rx_len)
{
	spin_lock(&dev->buflock);
	rxbuf->len = rx_len;
	rxbuf->state &= ~TXRX_STATE_RX_LOCKED;
	spin_unlock(&dev->buflock);
	return;
}

/* Issue scan command, get the time stamp for correct scan rate
 * support, and start time out timer just in case the interrupt
 * gets lost.
 */
static void
cy8mrln_issue_scan(struct tsc_drv_data *dev)
{
	mutex_lock(&dev->scan_mutex);

	/* Record the time the scan is issued. */
	dev->start_time_prev = dev->start_time; 
	ktime_get_ts(&dev->start_time);

	/* Set the timeout timer. */
#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_start(&dev->timeout,
		      ktime_set(0, SCAN_TIMEOUT_NSEC),
                      HRTIMER_MODE_REL);
#else
	mod_timer(&dev->timeout, jiffies +\
		  usecs_to_jiffies(SCAN_TIMEOUT_USEC));
#endif
	/* Issue a scan. */
	cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG,
			 CY8MRLN_SCAN_START);

	/* We are waiting for the interrupt. */
	clear_bit(INTR_HANDLED, dev->flags);

	mutex_unlock(&dev->scan_mutex);
}

/*
 *  Workq handler (FW version 080c)
 */
static void 
cy8mrln_work_handler_old(struct work_struct *work)
{
	int i, j = 0;
	struct tsc_drv_data *dev;
	txrx_buf_t *rxbuf = NULL;

	/* get device */
	dev = container_of(work, struct tsc_drv_data, workq);

	/* We need to perform read setup. */
	if (!dev->setup) {
		if (dev->verbose)
			printk("%s: Setting up for Read: \n", DRIVER);

		/* Read Setup */
		cy8mrln_setup_read(dev, CY8MRLN_DATA_REG_START);
		dev->setup = 1;

	/* Perform actual read of scan data. */
	} 
	else 
	{
		int ns;
		struct timespec now;
		struct timespec diff;

		if (dev->verbose) {
			printk("%s: Reading %d bytes data:\n", DRIVER,
			       CY8MRLN_NUM_DATA_BYTES);
		}

		mutex_lock(&dev->scan_mutex);
		rxbuf = cy8mrln_get_rx_buffer(dev);

		/* Read scan data. */
		cy8mrln_read_data(dev, CY8MRLN_DATA_REG_START,
				  dev->txbuf->ptr,
				  rxbuf->ptr, CY8MRLN_NUM_DATA_BYTES);

		/* Put the time stamp at the end of the scan data. */
		if (dev->timestamp) {
			ktime_get_ts(&now);
			memcpy(&rxbuf->ptr[CY8MRLN_NUM_DATA_BYTES],
			       &now, sizeof(now));
		}
		cy8mrln_put_rx_buffer(dev, rxbuf, CY8MRLN_NUM_DATA_BYTES +\
				      sizeof(now));
		dev->setup = 0;

		/* Set the flag to unblock the suspend. */
		set_bit(INTR_HANDLED, dev->flags);
		wake_up_interruptible(&dev->interrupt_wait);

		/* Set the flag to unblock the read. */
		set_bit(DATA_ACQUIRED, dev->flags);
		wake_up_interruptible(&dev->rx_wait);

		mutex_unlock(&dev->scan_mutex);

		/* The scan rate is set to zero. Don't schedule next scan. */
		if (!dev->scan_rate) {
			cy8mrln_disable_irq(dev);

		/* Schedule the next scan. */
		} else {

			/* Calculate the delta time between now and
			 * the next scan command per given scan rate.
			 */
			ktime_get_ts(&now);
			diff = timespec_sub(now, dev->start_time);
			ns = TSC_HZ_TO_NSEC(dev->scan_rate) -\
				timespec_to_ns(&diff);

			/* If scan rate is set to a positive value, schedule
		 	 * the next scan if the delay needs to be greater than
		 	 * the timer setup overhead.
			 */
			if (ns > TIMER_SETUP_NSEC) {

				/* We are not going to put PSoC to a quick nap
				 * because the 8C version doesn't support sleep
				 * and wake.
				 */
#ifdef CONFIG_HIGH_RES_TIMERS
				hrtimer_start(&dev->scan_timer,
					      ktime_set(0, ns),
					      HRTIMER_MODE_REL);
#else
				mod_timer(&dev->scan_timer, jiffies +\
					  usecs_to_jiffies(ns/1000));
#endif
				/* The scan rate was set to something close to
				 * or greater than the scan rate the PSoC can
				 * physically support. We need to top out the
				 * scan. Issue the next scan immediately.
				 */
			} else {
				cy8mrln_issue_scan(dev);
			}
		}

		if (dev->verbose && rxbuf ) {
			/* Rows of 16 bytes */
			for (i = 0; i < CY8MRLN_NUM_DATA_BYTES/16; i++) {
				for (j = 0; j < 16; j++)
					printk("%02x ", rxbuf->ptr[i*16+j]);
				printk("\n");
			}
			/* Remaining bytes. */
			for (j = 0; j < (CY8MRLN_NUM_DATA_BYTES - 16 * i); j++)
				printk("%02x ", rxbuf->ptr[16*i+j]);
			printk("\n");
		}
	}
	return;   
}

/*
 *  Workq handler (Firmware version 110a and above)
 */
static void 
cy8mrln_work_handler_new(struct work_struct *work)
{
	int i, j;
	struct tsc_drv_data *dev;
	txrx_buf_t *rxbuf = NULL;

	/* get device */
	dev = container_of(work, struct tsc_drv_data, workq);
	
	dev->tp_status_reg0 = 0 ;
	
	if (dev->verbose) {
		printk("%s: Setting up and Reading %d bytes data:\n",
		       DRIVER, CY8MRLN_NUM_DATA_BYTES);
	}

	/* The interrupt is due to scan command. */
	if( CY8MRLN_SCAN_START   == dev->cmd ||
	    CY8MRLN_AUTO_SCAN    == dev->cmd    ) 
	{
		struct timespec now;

		if(CY8MRLN_AUTO_SCAN == dev->cmd)
		{
			dev->auto_scan = 0;
			dev->tp_status_reg0 |= FRAME_TYPE_MASK__WOT_EXIT_SCAN ; //mark frame as Wakeup Scan or WOT exit scan.
			if(dev->verbose )
			{
				printk("%s: Clear start wot flag (auto_scan). \n", DRIVER );
			}
		}

		mutex_lock(&dev->scan_mutex);
		rxbuf = cy8mrln_get_rx_buffer(dev);

		/* Read scan data. */
		cy8mrln_setup_and_read_data(dev, CY8MRLN_DATA_REG_START,
					    dev->txbuf->ptr, rxbuf->ptr,
					    CY8MRLN_NUM_DATA_BYTES);

		//
		// Sanity check of the packet returned
		//
		if( rxbuf->ptr[0] != 0x0D || rxbuf->ptr[1] != 0x0A )
		{
			// Bad Frame Marker aka SOF or EOF
			//
			printk(KERN_INFO "%s: Bad SOF:Got[0x%02x,0x%02x] " \
				"Expected[0x0D, 0x0A](SCN)\n",
				DRIVER, rxbuf->ptr[0], rxbuf->ptr[1] );
			// TODO: Don't copy to Rx Buff -- bad packet.
		}
		// Issue new SCAN for fullscan mode 
		if( dev->auto_scan != 1 )
		{
			/* Record the time the scan is issued. */
			dev->start_time_prev = dev->start_time; 
			ktime_get_ts(&dev->start_time);
		
			/* Set the timeout timer. */
#ifdef CONFIG_HIGH_RES_TIMERS
			hrtimer_start(&dev->timeout,
				ktime_set(0, SCAN_TIMEOUT_NSEC),
				HRTIMER_MODE_REL);
#else
			mod_timer(&dev->timeout, jiffies +\
				usecs_to_jiffies(SCAN_TIMEOUT_USEC));
#endif
			/* Issue a scan. */
			cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG,
					CY8MRLN_SCAN_START);
		
			/* We are waiting for the interrupt. */
			clear_bit(INTR_HANDLED, dev->flags);
		}
		/* Put the time stamp at the end of the scan data. */
		if (dev->timestamp) {
			ktime_get_ts(&now);
			memcpy(&rxbuf->ptr[CY8MRLN_NUM_DATA_BYTES],
			       &now, sizeof(now));
		}
		
		/* TP Driver Status Register */
		if(	dev->auto_scan == 1 )
		{
			// mark frame as last full scan frame prior to entry into WOT mode.
			dev->tp_status_reg0 |= FRAME_TYPE_MASK__WOT_ENTRY_SCAN ; 
		}

		rxbuf->ptr[CY8MRLN_NUM_DATA_BYTES + TIMESTAMP_SIZE_BYTES] = dev->tp_status_reg0 ;
		
		cy8mrln_put_rx_buffer(dev, rxbuf, CY8MRLN_NUM_DATA_BYTES +\
				      TIMESTAMP_SIZE_BYTES + NUM_TP_DRV_STATUS_REG_BYTES );

		/* Set the flag to unblock the suspend. */
		set_bit(INTR_HANDLED, dev->flags);
		wake_up_interruptible(&dev->interrupt_wait);

		/* Set the flag to unblock the read. */
		set_bit(DATA_ACQUIRED, dev->flags);
		wake_up_interruptible(&dev->rx_wait);

		mutex_unlock(&dev->scan_mutex);

		/* Wake on touch mode */
		if(	dev->auto_scan == 1 && /* WOT is enabled    */
			dev->scan_rate         /* PSOC is turned on */ )
		{
			u16 regval, sensval;
			sensval = dev->wot_sen - WOT_THRESHOLD_MIN;
			sensval &= 0x3F;
			sensval <<= 2;

			if(dev->verbose )
			{
				printk("%s: WotIsON.\n", DRIVER );
			}

			/* Setup wot threshold level and wot scan frequency */
			regval = CY8MRLN_CONTROL_REG | (u16)sensval | (u16)(dev->wot_scanrate&0x03) ;

			/* Send autoscan SPI command */
			cy8mrln_reg_write( dev, regval,	CY8MRLN_AUTO_SCAN );
			/* Cancel timers */
#ifdef CONFIG_HIGH_RES_TIMERS
			hrtimer_cancel(&dev->scan_timer);
			hrtimer_cancel(&dev->timeout);
#else
			del_timer_sync(&dev->scan_timer);
			del_timer_sync(&dev->timeout);
#endif

			/* Enable interrupt. */
			cy8mrln_enable_irq(dev);
		}
		else if (!dev->scan_rate)
		{
			/* The scan rate is set to zero. Don't schedule next scan. */
			cy8mrln_disable_irq(dev);
		} 


	/* The interrupt is due to query command. */
	} else if( CY8MRLN_QUERY == dev->cmd ) {
		/* Read query data. */
		cy8mrln_setup_and_read_data(dev, CY8MRLN_DATA_REG_START,
					    dev->txbuf->ptr, dev->rev,
					    CY8MRLN_NUM_DATA_BYTES);
		//
		// Sanity check of the packet returned
		//
		if( dev->rev[0] != 0x0D || dev->rev[1] != 0x0C )
		{
			// Bad Frame Marker aka SOF or EOF
			//
			printk(KERN_INFO "%s: Bad SOF:Got[0x%02x,0x%02x] " \
				"Expected[0x0D, 0x0C](QRY)\n",
				DRIVER, dev->rev[0], dev->rev[1] );
		}
		if(dev->verbose )
		{
			printk( "%s: Query Command.[0x%02x,0x%02x,0x%02x,0x%02x]\n", DRIVER,
					dev->rev[0], dev->rev[1], dev->rev[2], dev->rev[3] );
		}
		/* Unblock GET_QUERY_DATA ioctl. */
		set_bit(VERSION_ACQUIRED, dev->flags);
		wake_up_interruptible(&dev->version_wait);
		if(dev->verbose )
		{
			printk("%s: Got IRQ for query command %x\n",
		       DRIVER, dev->cmd);
		}
	} else if ( CY8MRLN_CALIBRATE_IDAC == dev->cmd ){ 
		if(dev->verbose )
		{
			printk("%s: IDAC calibration finished\n", DRIVER);
		}

		/* Set the flag to unblock the suspend. */
		set_bit(INTR_HANDLED, dev->flags);
		set_bit(IDAC_CALIBRATED, dev->flags);

		wake_up_interruptible(&dev->interrupt_wait);
		
	} else if ( CY8MRLN_STORE_IDAC_CALIBRATION == dev->cmd ) {
		if(dev->verbose )
		{
			printk("%s: IDAC calibration stored\n", DRIVER);
		}

		/* Set the flag to unblock the suspend. */
		set_bit(INTR_HANDLED, dev->flags);
		set_bit(IDAC_CALIBRATION_STORED, dev->flags);

		wake_up_interruptible(&dev->interrupt_wait);

	} else if ( CY8MRLN_CALIBRATE_ILO == dev->cmd ) {
		if(dev->verbose )
		{
			printk("%s: ILO Calibrated\n", DRIVER);
		}

		/* Set the flag to unblock the suspend. */
		set_bit(INTR_HANDLED, dev->flags);
		set_bit(ILO_CALIBRATED, dev->flags);

		wake_up_interruptible(&dev->interrupt_wait);				
	/* This can happen if there is a glitch on interrupt line at startup. */
	} else if ( CY8MRLN_CMD_INVALID == dev->cmd) {
		printk(KERN_ERR "%s: Received interrupt due to startup "
				"glitch. No-op.\n", DRIVER);

	/* Invalid command. */
	} else {
		printk(KERN_ERR "%s: Invalid command %d. No-op.\n",
		       DRIVER, dev->cmd);
	}
	if (dev->verbose && rxbuf ) {
		/* Rows of 16 bytes */
		for (i = 0; i < CY8MRLN_NUM_DATA_BYTES/16; i++) {
			for (j = 0; j < 16; j++)
				printk("%02x ", rxbuf->ptr[i*16+j]);
			printk("\n");
		}
		/* Remaining bytes. */
		for (j = 0; j < (CY8MRLN_NUM_DATA_BYTES - 16 * i); j++)
			printk("%02x ", rxbuf->ptr[16*i+j]);
		printk("\n");
	}
	return;   
}

/* Handler for the workq called by ISR. */
static void
cy8mrln_work_handler(struct work_struct *work)
{
	struct tsc_drv_data *dev;

	/* get device */
	dev = container_of(work, struct tsc_drv_data, workq);

	/* Handle scan done interrupt. */
	if (dev->read_option >= CY8MRLN_ONE_INT_THREE_SETUP_BYTES) { 
		cy8mrln_work_handler_new(work);
	} else {
		cy8mrln_work_handler_old(work);
	}
	return;
}

/* Handler for the workq called by scan_timer/timeout expiration callback. */
static void 
cy8mrln_scan_handler(struct work_struct *scan)
{
	struct tsc_drv_data *dev;

	/* get device */
	dev = container_of(scan, struct tsc_drv_data, scanq);

	if (dev->verbose)
		printk("%s: In scan timer handler\n", DRIVER);

	/* Wake up the PSoC from sleep (IN_QUICK_NAP)
	 * or Break the PSoC out of read state (SCAN_TIMEOUT).
	 */
	// comment out NAP feature for now.
	//if (test_and_clear_bit(IN_QUICK_NAP, dev->flags) ||
	if ( 0 ||
	    (test_and_clear_bit(SCAN_TIMEOUT, dev->flags) &&
	     (dev->read_option == CY8MRLN_ONE_INT_ZERO_SETUP_BYTES))) {
                cy8mrln_toggle_cs(dev, PULSE_PSOC_CS_TOGGLE);
	}

	cy8mrln_issue_scan(dev);

	return;   
}


#ifdef CONFIG_HIGH_RES_TIMERS
/* scan_timer expiration call back. */
static enum hrtimer_restart
cy8mrln_scan_timer(struct hrtimer *handle)
{
	struct tsc_drv_data *dev = container_of(handle, struct tsc_drv_data,
						scan_timer);
	if (dev->verbose)
		printk("%s: scan timer expired:\n", DRIVER);

    queue_work(dev->tp_wq, &dev->scanq) ;

	return HRTIMER_NORESTART;
}
/* timeout expiration call back. */
static enum hrtimer_restart
cy8mrln_timeout(struct hrtimer *handle)
{
	struct tsc_drv_data *dev = container_of(handle, struct tsc_drv_data,
						timeout);

	if (dev->verbose)
		printk(KERN_ERR "%s: Scan timed out\n", DRIVER);

	set_bit(SCAN_TIMEOUT, dev->flags);

	if(1 != dev->auto_scan ) {
		if (dev->scan_rate > 0) {
		    queue_work(dev->tp_wq, &dev->scanq) ;
		}
	} else {
		if(dev->verbose) {
			printk(KERN_INFO "%s: scan timeout. Not scheduling work since auto_scan==1\n", DRIVER);
		}
	}

	return HRTIMER_NORESTART;
}
#else
/* scan_timer expiration call back. */
static void 
cy8mrln_scan_timer(unsigned long data)
{
	struct tsc_drv_data *dev = (struct tsc_drv_data *)data;

	if (dev->verbose)
		printk("%s: scan timer expired\n", DRIVER);

    queue_work(dev->tp_wq, &dev->scanq) ;

	return;
}
/* timeout expiration call back. */
static void 
cy8mrln_timeout(unsigned long data)
{
	struct tsc_drv_data *dev = (struct tsc_drv_data *)data;

	if (dev->verbose)
		printk(KERN_ERR "%s: Scan timed out\n", DRIVER);

	set_bit(SCAN_TIMEOUT, dev->flags);

	if (dev->scan_rate > 0) {
    	queue_work(dev->tp_wq, &dev->scanq) ;
	}

	return;
}
#endif

/*
 *  PSoC Interrupt handler
 */
static irqreturn_t 
cy8mrln_irq(int irq, void *dev_id )
{
	struct tsc_drv_data *dev = (struct tsc_drv_data *)dev_id;

	queue_work( dev->tp_wq, &dev->workq );

	return IRQ_HANDLED;
}

/* Power up the PSoC. */
static int 
cy8mrln_do_poweron(struct tsc_drv_data *dev, unsigned long start_up_delay)
{
	int rc = 0;

	dev->setup = 0;
	/* Switch from GPIO mode. */
	dev->pdata->switch_mode(NORM_OP);
	/* Enable the VDD. */
	gpio_set_value(dev->pdata->enable_gpio, 1);

	msleep(start_up_delay);

	rc = request_irq(dev->spidev->irq, cy8mrln_irq, 
			 IRQF_DISABLED | IRQF_TRIGGER_RISING ,
			 "touchscreen", dev );

	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get IRQ.\n", DRIVER);
		goto err;
	}

	clear_bit(GPIO_IRQ_DISABLED, dev->flags);

	cy8mrln_disable_irq(dev);
	
err:

	return rc;
}

/* Cancel timers and work queues. */
static void
cy8mrln_do_cleanup(struct tsc_drv_data *dev)
{
	/* Disable interrupt, cancel work, and delete timer. */
#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_cancel(&dev->scan_timer);
	hrtimer_cancel(&dev->timeout);
#else
	del_timer_sync(&dev->scan_timer);
	del_timer_sync(&dev->timeout);
#endif
	cy8mrln_disable_irq(dev);
	cancel_work_sync(&dev->workq);
	cancel_work_sync(&dev->scanq);
}

/* Power off the PSoC. */
static int 
cy8mrln_do_poweroff(struct tsc_drv_data *dev)
{
	dev->setup = 0;

	free_irq(dev->spidev->irq, dev);

	/* Cancel timers and work queues. */
	cy8mrln_do_cleanup(dev);

	/* Flush buffered data done on resume instead */

	/* Switch to GPIO mode. */
	dev->pdata->switch_mode(PROGRAM);
	gpio_set_value(dev->pdata->enable_gpio, 0);
	// CS, MOSI, set to low
	/*
	sda_out(dev, 1);
	gpio_set_value(dev->pdata->mosi_gpio, 0);
	gpio_set_value(dev->pdata->scl_gpio, 0);
	gpio_set_value(dev->pdata->sda_gpio, 0);
	gpio_set_value(dev->pdata->enable_gpio, 0);
	*/

	dev->cmd = CY8MRLN_CMD_INVALID ;
	//Keep scanrate over suspend and resumes
	//Don't clear dev->scan_rate.
	dev->auto_scan = 0 ;

	return 0;
}

/* Handle the resume of the driver and the PSoC. It can be called either
 * to start scanning after power up or to start scanning after wake up.
 */
static int 
cy8mrln_do_resume(struct tsc_drv_data *dev)
{
	u16 regval;

#ifdef DO_MEASUREMENTS
	struct timespec before, after, diff;
#endif

#ifdef DO_MEASUREMENTS
	ktime_get_ts(&before);
#endif
	/* Wake up the PSoC. Just toggle the chip select
	 * instead of sending the GO_ACTIVE command.
	 * The chip select edge is all the PSoC needs.
	 */
	//cy8mrln_toggle_cs(dev);
	cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG, CY8MRLN_GO_ACTIVE);
	/* Delay for pSoC*/
	msleep(10);

	/* Baseline setting */
	regval = CY8MRLN_CONTROL_REG | dev->wot_baseline_hi | dev->wot_baseline_lo << 8;
	cy8mrln_reg_write( dev, regval,	CY8MRLN_BASELINE_ADJUST );

	/* Flush read buffer to avoid stale leftover data */
	cy8mrln_flush_rd_buffers( dev );	
	
	/* Delay for pSoC */
	msleep(10);

#ifdef DO_MEASUREMENTS
	ktime_get_ts(&after);
	diff = timespec_sub(after, before);
	printk("%s: It took %lld nsec to process wake up command\n", DRIVER,
	       timespec_to_ns(&diff));
#endif
	dev->setup = 0;

	/* Start scan timer, if scan rate is set. */
	if (!dev->scan_rate) {
		return 0;
	}

	/* Enable interrupt. */
	cy8mrln_enable_irq(dev);

	/* Issue the scan and get the ball rolling. */
	cy8mrln_issue_scan(dev);

	return 0;
}

/* Handle the suspend of the driver and the PSoC. It should be only
 * used to put the PSoC from active mode to sleep. 
 */
static void
cy8mrln_do_suspend(struct tsc_drv_data *dev)
{
	long timeout = SCAN_TIMEOUT_JIFFY;

#ifdef DO_MEASUREMENTS
	struct timespec before, after, diff;
#endif

	/* If the PSoC is off, then the clean up has been done
	 * already. Also, we don't need to wait for interrupt
	 * since the PSoC is powered off. Nothing to do here.
	 */
	if (dev->sleep_mode == CY8MRLN_OFF_STATE) {
		return;
	}

	
	/* If we haven't received interrupt after issuing a scan command,
	 * we cannot suspend until we receive the interrupt. Wait until
	 * the INTR_HANDLED gets set or timeout.
	 */
	timeout = wait_event_interruptible_timeout(dev->interrupt_wait,
				test_bit(INTR_HANDLED, dev->flags), timeout);


		
	/* We timed out instead of getting the interrupt. The PSoC is stuck. */
	if (!timeout) {
		/* Only the new FW 110a and above supports sleep and wake. */
		if (dev->read_option == CY8MRLN_ONE_INT_ZERO_SETUP_BYTES) {

			/* In case the PSoC is in the Read Data state,
			 * get it out to default state. It should work,
			 * and we shouldn't need to power cycle. If it
			 * does not work, the PSoC firmware has a bug.
			 */
		        cy8mrln_toggle_cs(dev, PULSE_PSOC_CS_TOGGLE);
		}
		printk(KERN_ERR "%s: Missed interrupt and timed out for suspend\n",
		       DRIVER); 
	}

	/* Cancel timers and work queues. */
	cy8mrln_do_cleanup(dev);

	/* Only the new FW 110a and above supports sleep and wake. */
	if (dev->read_option == CY8MRLN_ONE_INT_ZERO_SETUP_BYTES) {

#ifdef DO_MEASUREMENTS
		ktime_get_ts(&before);
#endif
		/* Go to sleep. */
		cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG, CY8MRLN_SLEEP);
#ifdef DO_MEASUREMENTS
		ktime_get_ts(&after);
		diff = timespec_sub(after, before);
		printk("%s: It took %lld nsec to process sleep command\n", DRIVER,
		       timespec_to_ns(&diff));
#endif
	}
}

#ifdef CONFIG_PM
static int 
cy8mrln_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct tsc_drv_data *dev = spi_get_drvdata(spi);

	PDBG("%s:\n", __FUNCTION__ );

	if(dev->verbose) {
		printk(KERN_INFO "cy8mrln: start suspending sleep_mode:%d saved_mode:%d suspended?:%d\n", dev->sleep_mode, dev->saved_mode, test_bit(SUSPENDED, dev->flags));
	}


	/* check if it is in use */
	if (!test_and_set_bit(SUSPENDED, dev->flags)) {

		/* We can either power down the PSoC for power saving.
		 * or just place it to sleep for wake up speed. Currently
		 * we are powering the PSoC down for power saving.
		 *
		 * Wolfgang commented that the sleep current drawing was
		 * 13-15mA. My measurements gave me about 4mA. The power
		 * consumption needs to be measured for each updated
		 * firmware.
		 *
		 * See the sleep and wake timing number #defines
		 * at the beginning of this file. Those numbers also
		 * need to be kept up to date with new firmware.
		 */
		dev->saved_mode = dev->sleep_mode;
		switch(dev->sleep_mode) {
		case CY8MRLN_ON_STATE:
//			cy8mrln_do_suspend(dev);
			cy8mrln_do_poweroff(dev);
//			dev->sleep_mode = CY8MRLN_SLEEP_STATE;
			dev->sleep_mode = CY8MRLN_OFF_STATE;

			break;

		case CY8MRLN_SLEEP_STATE:
			cy8mrln_do_poweroff(dev);
			dev->sleep_mode = CY8MRLN_OFF_STATE;
			break;

		case CY8MRLN_OFF_STATE:
			/* The PSoC is already off. Nothing to do. */
			break;
		}
	}

	if(dev->verbose) {
		printk(KERN_INFO "cy8mrln: done suspending sleep_mode:%d saved_mode:%d suspended?:%d\n", dev->sleep_mode, dev->saved_mode, test_bit(SUSPENDED, dev->flags));
	}


	return 0;
}

static void vdd_high(struct tsc_drv_data *dev);
static int 
cy8mrln_resume(struct spi_device *spi)
{
	struct tsc_drv_data *dev = spi_get_drvdata(spi);

	if(dev->verbose) {
		printk(KERN_INFO "cy8mrln: start resuming sleep_mode:%d saved_mode:%d suspended?:%d\n", dev->sleep_mode, dev->saved_mode, test_bit(SUSPENDED, dev->flags));
	}

	PDBG("%s:\n", __FUNCTION__ );
	if (test_and_clear_bit(SUSPENDED, dev->flags)) {
	
		switch (dev->saved_mode) {
			case CY8MRLN_ON_STATE:
				//if (dev->sleep_mode == CY8MRLN_SLEEP_STATE) {
				//	/* We were sleeping. Wake up PSoC and resume. */
				//	cy8mrln_do_resume(dev);
				//} else 
				if (dev->sleep_mode == CY8MRLN_OFF_STATE) {
					/* The PSoC was off. Power up */
					cy8mrln_do_poweron(dev, NORMAL_POWER_UP_TIMEOUT_MSEC);;
					/* Restore the previous PSoC state by calling resume. */
					cy8mrln_do_resume(dev);
				} else {
					printk(KERN_ERR "cy8mrln: resume from invalid state %d\n",
							dev->sleep_mode);
					goto end;
				}
				break;

			default:
				/* Do nothing if not turning on */
				break;
		}
		dev->sleep_mode = dev->saved_mode;
	}
end:
	if(dev->verbose) {
		printk(KERN_INFO "cy8mrln: done resuming sleep_mode:%d saved_mode:%d suspended?:%d\n", dev->sleep_mode, dev->saved_mode, test_bit(SUSPENDED, dev->flags));
	}


	return 0;
}

#else

#define cy8mrln_suspend  NULL
#define cy8mrln_resume   NULL

#endif /* CONFIG_PM */


static ssize_t 
cy8mrln_read(struct file *file, char __user *buf, size_t count, 
	    loff_t *ppos )
{
	ssize_t rc, nbytes = 0, rx_len = 0;
	struct tsc_drv_data *dev = file->private_data;
	struct txrx_buf *rxbuf;

	if (dev->verbose)
		printk("%s: cy8mrln_read %d bytes\n", DRIVER, count);

	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&dev->mutex)) {
			rc = -EAGAIN;
			goto Exit;
		}
	} else {
		if (mutex_lock_interruptible(&dev->mutex)) {
			rc = -ERESTARTSYS;
			goto Exit;
		}
	}

	while (nbytes < count) {

		/* Get the valid buffer pointer */
		rxbuf = cy8mrln_get_rd_buffer(dev, &rx_len);

		if (rxbuf) { 
			/* We have more data than requested. */ 
			if (rx_len > (count - nbytes))
			{
				rx_len = (count - nbytes);
				if (dev->verbose)
					printk("%s: Got more data than requested.\n", DRIVER );
			}

			if (copy_to_user(buf + nbytes,
			                 rxbuf->ptr + rxbuf->pos,
			                 rx_len))
			{
				if (dev->verbose)
					printk("%s: cpToUsrFailed\n", DRIVER );
				cy8mrln_put_rd_buffer(dev, rxbuf, 0);
				rc = -EFAULT;
				goto Done;
			}

			nbytes  += rx_len;
			cy8mrln_put_rd_buffer(dev, rxbuf, rx_len);
			if (rx_len)
			{
				if (dev->verbose)
					printk("%s: Continue with read\n", DRIVER );
				continue;
			}
		}

		/* We read some, but not all. Just return the portion. */	
		if (nbytes) {
			if (dev->verbose)
				printk("%s: Got partial read\n", DRIVER );
			rc = nbytes;
			goto Done;
		}

		if (file->f_flags & O_NONBLOCK) {
			if (dev->verbose)
				printk("%s: O_NONBLOCK\n", DRIVER );
			rc = -EAGAIN;
			goto Done;
		}

		/* Clear flag to indicate that there is no pending data.
		 * We need to block.
		 */
		if (!cy8mrln_rx_has_bytes(dev))
		{
			if (dev->verbose)
				printk("%s: No pending data\n", DRIVER );
			clear_bit(DATA_ACQUIRED, dev->flags);
		}

		wait_event_interruptible(dev->rx_wait,
				test_bit(DATA_ACQUIRED, dev->flags) );

		/* We received signal or waken up. */
		/* Signal? */
		if (!test_bit(DATA_ACQUIRED, dev->flags) &&
			signal_pending(current))
		{
			rc = -ERESTARTSYS;
			goto Done;
		}

	}//e.o. while()
	rc = nbytes;
Done:
	mutex_unlock(&dev->mutex);
	
Exit:	
	return rc;
}

static unsigned int 
cy8mrln_poll (struct file *file, struct poll_table_struct *wait)
{
	unsigned long flags;
	unsigned int  mask = 0;
	struct tsc_drv_data *dev = file->private_data;

	if (dev->verbose)
		printk("%s: cy8mrln_poll\n", DRIVER);

	poll_wait(file, &dev->rx_wait, wait);

	spin_lock_irqsave(&dev->buflock, flags);

	if( cy8mrln_rx_has_bytes(dev) )
	{
		mask |= POLLIN	| POLLRDNORM;
	}

	spin_unlock_irqrestore(&dev->buflock, flags);

	return mask;
}

// ****************************************************************************
// ************************ ISSP GPIO FUNCTION SECTION ************************
// ****************************************************************************
/* Set the SCL pin High. */
static void scl_high(struct tsc_drv_data *dev)
{
 	gpio_set_value(dev->pdata->scl_gpio, 1);
}

/* Set the SCL pin Low. */
static void scl_low(struct tsc_drv_data *dev)
{
 	gpio_set_value(dev->pdata->scl_gpio, 0);
}

#ifndef RESET_MODE  // Only needed for power cycle mode
/* Set SCL pin to HighZ drive mode. */
static void scl_hiz(struct tsc_drv_data *dev)
{
        gpio_direction_input(dev->pdata->scl_gpio);
}
#endif

/* Set SCL to an output (Strong drive mode). */
static void scl_out(struct tsc_drv_data *dev, int val)
{
        gpio_direction_output(dev->pdata->scl_gpio, val);
}

/* During programming mode, the PSoC doesn't drive the
 * SDA line high enough for the host to recognize high
 * signal. So we added a pull up resistor to the latest
 * touch panel FPC (EVT2). The resistor is controlled
 * by the MOSI pin. Having this code won't hurt EVT1 touch
 * panel programming, which is working with marginal signal
 * level. But this is required for programming EVT2
 * touch panel, which has 5V VDD instead of 3.3V.
 * 
 * ****************************************************
 *             MOSI mirroring requirements
 * ****************************************************
 * During programming:
 *
 * 	When host drives the SDA (chip select pin), the
 * 	MOSI needs to mirror the SDA value.
 *
 * 	When PSoC drives the SDA, MOSI needs to stay low. 
 *
 * During touch panel operation:
 *
 *	MOSI talks while chip select is asserted low.
 *	MOSI needs to stay high when chip select is
 *	deasserted high. 
 */
/* Make SDA pin High. Mirror it with MOSI pin. */
static void sda_high(struct tsc_drv_data *dev)
{
 	gpio_set_value(dev->pdata->mosi_gpio, 1);
 	gpio_set_value(dev->pdata->sda_gpio, 1);
}

/* Make SDA pin Low. Mirror it with MOSI pin. */
static void sda_low(struct tsc_drv_data *dev)
{
 	gpio_set_value(dev->pdata->mosi_gpio, 0);
 	gpio_set_value(dev->pdata->sda_gpio, 0);
}

/* Set SDA pin to an input (HighZ drive mode).
 * The MISO pin needs to be set to output zero.
 */
static void sda_hiz(struct tsc_drv_data *dev)
{
        gpio_direction_output(dev->pdata->mosi_gpio, 0);
        gpio_direction_input(dev->pdata->sda_gpio);
}

/* Set SDA for transmission (Strong drive mode). Mirror
 * it with MOSI pin.
 */
static void sda_out(struct tsc_drv_data *dev, int val)
{
        gpio_direction_output(dev->pdata->mosi_gpio, val);
        gpio_direction_output(dev->pdata->sda_gpio, val);
}

#ifdef RESET_MODE
/* Set external reset (XRES) to an output (Strong drive mode). */
static void xres_out(struct tsc_drv_data *dev, int val)
{
        gpio_direction_output(dev->pdata->xres_gpio, val);
}

/* Set XRES pin High. */
static void xres_high(struct tsc_drv_data *dev)
{
   	gpio_set_value(dev->pdata->xres_gpio, 1);
}

/* Set XRES pin low. */
static void xres_low(struct tsc_drv_data *dev)
{
   	gpio_set_value(dev->pdata->xres_gpio, 0);
}
#endif

/* Provide power to the target PSoC's Vdd pin through a enable GPIO. */
static void vdd_high(struct tsc_drv_data *dev)
{
 	gpio_set_value(dev->pdata->enable_gpio, 1);
}

/* Remove power from the target PSoC's Vdd pin. */
static void vdd_low(struct tsc_drv_data *dev)
{
 	gpio_set_value(dev->pdata->enable_gpio, 0);
}

/* Check SDA pin for high or low logic level. */
static unsigned char sda_check(struct tsc_drv_data *dev)
{
 	return gpio_get_value(dev->pdata->sda_gpio);
}

// ****************************************************************************
// ********************** ISSP UTILITY FUNCTION SECTION ***********************
// ****************************************************************************
/* Run Clock without sending/receiving bits. Use this when transitioning from 
 * write to read and read to write.
 *
 * SCL cannot run faster than the specified maximum frequency of 8MHz. Some 
 * processors may need to have delays added after setting SCLK low and setting
 * SCLK high in order to not exceed this specification.
 */
static void run_clock(struct tsc_drv_data *dev, unsigned int num_cycles)
{
	int i;

	/* Loop ends with SCL high. */
	for(i = 0; i < num_cycles; i++) {
		scl_low(dev);
		udelay(5);
		scl_high(dev);
		udelay(5);
	}
	return;
}

/* Clocks the SCL pin (high-low-high) and reads the status of the SDA pin
 * after the rising edge.
 *
 * SCL cannot run faster than the specified maximum frequency of 8MHz. Some 
 * processors may need to have delays added after setting SCLK low and setting
 * SCLK high in order to not exceed this specification.
 *
 * Returns:
 *     0 if SDA was low
 *     1 if SDA was high
 */
static unsigned char receive_bit(struct tsc_drv_data *dev)
{
	unsigned long flags;
	unsigned char sda;

        spin_lock_irqsave(&dev->issplock, flags);

	scl_low(dev);
	udelay(5);
	scl_high(dev);
	udelay(5);

	sda = sda_check(dev);

        spin_unlock_irqrestore(&dev->issplock, flags);
	return (sda);
}

/* Calls ReceiveBit 8 times to receive one byte.
 * It returns 8-bit value received.
 */
static unsigned char receive_byte(struct tsc_drv_data *dev)
{
	unsigned char b;
	unsigned char curr_byte = 0x00;

	for (b = 0; b < 8; b++) {
		curr_byte = (curr_byte << 1) + receive_bit(dev);
	}
	return(curr_byte);
}                

/* This routine sends up to one byte of a vector, one bit at a time.
 *    curr_byte   the byte that contains the bits to be sent.
 *    size       the number of bits to be sent. Valid values are 1 to 8.
 *
 * SCL cannot run faster than the specified maximum frequency of 8MHz. Some 
 * processors may need to have delays added after setting SCLK low and setting
 * SCLK high in order to not exceed this specification.
 */
static void send_byte(struct tsc_drv_data *dev, unsigned char curr_byte,
		      unsigned char size)
{
	unsigned char b = 0;
	unsigned long flags;

        spin_lock_irqsave(&dev->issplock, flags);
	for(b = 0; b < size; b++) {
		if (curr_byte & 0x80) {
			/* Send a '1' */
			sda_high(dev);
			scl_high(dev);
			udelay(5);
			scl_low(dev);
			udelay(5);
		} else {
			/* Send a '0' */
			sda_low(dev);
			scl_high(dev);
			udelay(5);
			scl_low(dev);
			udelay(5);
		}
		curr_byte = curr_byte << 1;
	}
        spin_unlock_irqrestore(&dev->issplock, flags);
	return;
}

/* This routine sends the vector specifed.
 * The data line is returned to HiZ after the vector is
 */
static void send_vector(struct tsc_drv_data *dev, const unsigned char *vect,
			unsigned int numbits)
{
	sda_out(dev, 0);
	while(numbits > 0) {
		if (numbits >= 8) {
			send_byte(dev, *(vect), 8);
			numbits -= 8;
			vect++;
		} else {
			send_byte(dev, *(vect), numbits);
			numbits = 0;
		}
	}
	sda_hiz(dev);
	return;
}


/* Waits for transition from SDA = 1 to SDA = 0.  Has a 100 msec timeout.
 * TRANSITION_TIMEOUT is a loop counter for a 100msec timeout when waiting
 * for a high-to-low transition. This is used in the polling loop of 
 * detect_high_low().
 *
 * SCL cannot run faster than the specified maximum frequency of 8MHz. Some 
 * processors may need to have delays added after setting SCLK low and setting
 * SCLK high in order to not exceed this specification.
 */
static signed char detect_high_low(struct tsc_drv_data *dev)
{
	unsigned int timer_val;
	unsigned long flags;

	/* NOTE:
	 * These loops look unconventional, but it is necessary to
	 * check SDA pin as shown because the transition can be
	 * missed otherwise, due to the length of the SDA
	 * Low-High-Low after certain commands.
	 */

	/* Generate clocks and wait for the target to pull SDA High. */
	timer_val = TRANSITION_TIMEOUT;
	while(1) {
        	spin_lock_irqsave(&dev->issplock, flags);
		scl_low(dev);
		udelay(5);
		if (sda_check(dev)) {
        		spin_unlock_irqrestore(&dev->issplock, flags);
			break;
		}
		scl_high(dev);
		udelay(5);
        	spin_unlock_irqrestore(&dev->issplock, flags);

		/* Time out. */ 
		if (timer_val-- == 0) {
			printk(KERN_ERR "%s: Failed to detect high\n", DRIVER);
			return HILOW_ERROR; 
		}
		udelay(15);
	}

	/* Generate Clocks and wait for the target to pull SDA Low. */
	timer_val = TRANSITION_TIMEOUT;
	while(1) {
        	spin_lock_irqsave(&dev->issplock, flags);
		scl_low(dev);
		udelay(5);
		if (!sda_check(dev)) {
        		spin_unlock_irqrestore(&dev->issplock, flags);
			break;
		}
		scl_high(dev);
		udelay(5);
        	spin_unlock_irqrestore(&dev->issplock, flags);

		/* Time out. */ 
		if (timer_val-- == 0) {
			printk(KERN_ERR "%s: Failed to detect low\n", DRIVER);
			return HILOW_ERROR; 
		}
		udelay(15);
	}
	udelay(15);
	return PASS;
}

// ****************************************************************************
// ********************* ISSP PROCEDURE FUNCTION SECTION **********************
// ****************************************************************************
#ifdef RESET_MODE
/* Implements the intialization vectors for the device. */
static signed char xres_init(struct tsc_drv_data *dev)
{
	spinlock_t lock;

	spin_lock_init(&lock);

	/* Configure the pins for initialization. */
	sda_hiz(dev);
	scl_out(dev, 0);
	xres_out(dev, 1);


  
	/* Cycle reset and put the device in programming mode
	 * when it exits reset.
	 */
	udelay(XRES_CLK_DELAY);

	/* The spec says the first 8 bits of the INIT 1 vector should
	 * be transmitted within 125 usec. Hold the interrupts for the
	 * INIT1 vector.
	 */
        spin_lock(&lock);
	xres_low(dev);

	/* Send Initialization Vectors and detect Hi-Lo transition on SDATA. */
	send_vector(dev, VECTOR(INIT1), BITS(INIT1)); 
        spin_unlock(&lock);
	if (detect_high_low(dev)) {
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

	/* Send Initialize 2 Vector */
	send_vector(dev, VECTOR(INIT2), BITS(INIT2));
	if (detect_high_low(dev)) {
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));


     
	/* Send Initialize 3 Vector NOTE: the proper vector
	 * based on Vdd of target
	 */
#ifdef TARGET_VOLTAGE_IS_5V
	send_vector(dev, VECTOR(INIT3_5V), BITS(INIT3_5V));
#else
	send_vector(dev, VECTOR(INIT3_3V), BITS(INIT3_3V));
#endif
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

	/* NOTE: DO NOT not wait for HiLo on SDA after vector Init-3
	 * it does not occur (per spec).
	 */
	return PASS;
}
#else
/* Implements the intialization vectors for the device. The first
 * time detect_high_low is called the Clk pin is highZ because
 * the clock is not needed during acquire. 
 */
static signed char power_cycle_init(struct tsc_drv_data *dev)
{
	spinlock_t lock;

	spin_lock_init(&lock);

        spin_lock(&lock);  // disable preemption

	/* Set all pins to highZ to avoid back powering the PSoC
	 * through the GPIO protection diodes.
	 */
	scl_hiz(dev);   
	sda_hiz(dev);


      
	/* Turn on power to the target device before other signals. */
	vdd_high(dev);

	/* Wait 1 msec for the power to stabilize per spec. */
	mdelay(1);

	/* The first time detect. The scl is at hiz. */
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after power cycle\n", DRIVER);
        	spin_unlock(&lock); // reenable preemption  
		return HILOW_ERROR;
	}    

	/* Configure the pins for initialization. */
	sda_hiz(dev);
	scl_out(dev, 0);

	/* The spec says the INIT 1 vector should be transmitted within
	 * 3 msecs. We have preemption disabled so only thing that can break us if 
	 * somebody holds interrupts for long.   
	 */
	/* Send Initialization Vectors and detect Hi-Lo transition on SDATA. */
	send_vector(dev, VECTOR(INIT1), BITS(INIT1));
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after INIT1 vector.\n", DRIVER);
        	spin_unlock(&lock); // reenable preemption  
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

        spin_unlock(&lock); // reenable preemption  

	/* Send Initialize 2 Vector. */
	send_vector(dev, VECTOR(INIT2), BITS(INIT2));
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after INIT2 vector.\n", DRIVER);
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

	/* Send Initialize 3 Vector NOTE: the proper vector
	 * based on Vdd of target.
	 */
#ifdef TARGET_VOLTAGE_IS_5V
	send_vector(dev, VECTOR(INIT3_5V), BITS(INIT3_5V));
#else
	send_vector(dev, VECTOR(INIT3_3V), BITS(INIT3_3V));
#endif
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

	/* NOTE: DO NOT not wait for HiLo on SDA after vector Init-3
	 * it does not occur (per spec).
	 */
	return PASS;
}
#endif

/* Read the Silicon ID from the PSoC and check for valid ID. */
static signed char verify_silicon_id(struct tsc_drv_data *dev)
{
	unsigned char target_id[10];

	/* Send ID-Setup vector set. */
	send_vector(dev, VECTOR(ID_SETUP), BITS(ID_SETUP));
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after ID_SETUP vector.\n", DRIVER);
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

	/* Send Read ID vector and get Target ID. */

	/* Read-MSB Vector is the first 11-Bits. */
	send_vector(dev, dev->vector[READ_ID].data, 11);
	/* Two SCLK cycles between write & read. */
	run_clock(dev, 2);
	target_id[0] = receive_byte(dev);
	run_clock(dev, 1);
	/* 12 bits starting from the 3rd character. */
	send_vector(dev, &(dev->vector[READ_ID].data[2]), 12);

	/* Read-LSB command. */
	run_clock(dev, 2);
	target_id[1] = receive_byte(dev);

	/* 1 bit starting from the 5th character. */
	run_clock(dev, 1);
	send_vector(dev, &(dev->vector[READ_ID].data[4]), 1);

	printk(KERN_INFO "%s: Read Silicon ID %02x %02x.\n", DRIVER,
	       target_id[0], target_id[1]);

	/* Check the chip id. */
	if (target_id[0] != dev->vector[TARGET_ID].data[0] ||
	    target_id[1] != dev->vector[TARGET_ID].data[1]) {
		printk(KERN_ERR "%s: Hex file is for Silicon ID %02x %02x.\n",
		       DRIVER, dev->vector[TARGET_ID].data[0], dev->vector[TARGET_ID].data[1]);
		return SI_ID_ERROR; 
	} else {
		return PASS;
	}
}

/* Perform a bulk erase of the target device. */
static signed char erase_target(struct tsc_drv_data *dev)
{
	send_vector(dev, VECTOR(ERASE_ALL), BITS(ERASE_ALL));
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after ERASE_ALL vector.\n", DRIVER);
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));
	return PASS;
}

/* Program one block with data that has been loaded into a RAM buffer in the 
 * target device.
 */
static signed char program_target_block(struct tsc_drv_data *dev,
					unsigned char blk_num)
{
	send_vector(dev, VECTOR(SET_BLOCK_NUMBER), BITS(SET_BLOCK_NUMBER));

	/* Set the drive here because send_byte() does not. */
	sda_out(dev, 0);
	send_byte(dev, blk_num,8);
	send_byte(dev, *VECTOR(SET_BLOCK_NUMBER_END), BITS(SET_BLOCK_NUMBER_END));

	/* Send the program-block vector. */
	send_vector(dev, VECTOR(PROGRAM_BLOCK), BITS(PROGRAM_BLOCK));

	/* wait for acknowledge from target. */
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after PROGRAM_BLOCK vector.\n", DRIVER);
		return HILOW_ERROR;
	}
	/* Send the Wait-For-Poll-End vector. */
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));
	return PASS;
}

/* Verify the block just written to. This can be done byte-by-byte before the
 * protection bits are set.
 */
static signed char verify_target_block(struct tsc_drv_data *dev,
				       unsigned char blk_num)
{
	unsigned char target_addr;
	unsigned char target_data_in;
	unsigned char target_data_ptr;

	send_vector(dev, VECTOR(SET_BLOCK_NUMBER), BITS(SET_BLOCK_NUMBER));

	/* Set the drive here because send_byte() does not. */
	sda_out(dev, 0);
	send_byte(dev, blk_num,8);
	send_byte(dev, *VECTOR(SET_BLOCK_NUMBER_END), BITS(SET_BLOCK_NUMBER_END));

	send_vector(dev, VECTOR(VERIFY_SETUP), BITS(VERIFY_SETUP));
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after VERIFY_SETUP vector.\n", DRIVER);
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

	target_addr = 0;
	target_data_ptr = 0;

	while(target_data_ptr < TARGET_DATABUFF_LEN) {
		/* Send Read Byte vector and then get a byte from Target. */
		send_vector(dev, dev->vector[READ_BYTE].data, 5);

		/* Set the drive here because send_byte() does not. */
		sda_out(dev, 0);
		send_byte(dev, target_addr,6);

		/* Run two SCLK cycles between writing and reading. */
		run_clock(dev, 2);
		/* Set to HiZ so Target can drive SDA. */
		sda_hiz(dev);
		target_data_in = receive_byte(dev);

		/* Send the READ_BYTE vector end. */
		run_clock(dev, 1);
		send_vector(dev, &(dev->vector[READ_BYTE].data[1]), 1);

		/* Test the Byte that was read from the Target against
		 * the original value (already in the 64-Byte DATAOUT array).
		 * If it matches, then bump the address & pointer,loop-back
		 * and continue. If it does NOT match, abort the loop and
		 * return and error.
		 */
		if (target_data_in != DATAOUT(target_data_ptr)) {
			return VERIFY_ERROR;
		}

		target_data_ptr++;
		/* Increment the address by four to accomodate 6-Bit
		 * addressing (puts the 6-bit address into MSBit locations
		 * for "send_byte()").
		 */
		target_addr += 4;
	}
	yield();
	return PASS;
}


/* Before calling, load the array, DATAOUT, with the desired security. */
static signed char secure_target_flash(struct tsc_drv_data *dev)
{
	unsigned char target_addr;
	unsigned char temp;
	unsigned char target_data_ptr;

	/* Transfer the temporary RAM array into the target. */
	target_addr = 0x00;
	target_data_ptr = 0x00;
	sda_out(dev, 0);
	while(target_data_ptr < SECURITY_BYTES_PER_BANK) {     
		temp = DATAOUT(target_data_ptr);
		send_byte(dev, *VECTOR(WRITE_BYTE_START), BITS(WRITE_BYTE_START));
		send_byte(dev, target_addr, 6);
		send_byte(dev, temp, 8);
		send_byte(dev, *VECTOR(WRITE_BYTE_END), BITS(WRITE_BYTE_END));


           
		/* send_bytes() uses MSBits, so increment the address by '4' to put
		 * the 0..n address into the six MSBit locations
		 */
		target_addr += 4;
		target_data_ptr++;
	}

	send_vector(dev, VECTOR(SECURITY), BITS(SECURITY));
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after SECURITY vector.\n", DRIVER);
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));
	return PASS;
}

#ifdef MULTI_BANK
/* Set the bank number in the target device. */
static void set_bank_num(struct tsc_drv_data *dev, unsigned char bank_num)
{
	/* Send the bank-select vector. */
	send_vector(dev, VECTOR(SET_BANK_NUMBER), BITS(SET_BANK_NUMBER));

	/* Set the drive here because send_byte() does not. */
	sda_out(dev, 0);
	send_byte(dev, bank_num,8);
	send_vector(dev, VECTOR(SET_BANK_NUMBER_END), BITS(SET_BANK_NUMBER_END));
}
#endif

/* Transfers data from array in Host to RAM buffer in the target.
 * Returns the checksum of the data.
 */
static unsigned short load_target(struct tsc_drv_data *dev)
{
	unsigned char target_addr;
	unsigned char temp;
	unsigned short checksum = 0;
	unsigned char target_data_ptr;

	/* Set SDATA to Strong Drive here because send_byte() does not */
	sda_out(dev, 0);

	/* Transfer the temporary RAM array into the target. In this section,
	 * a 64-Byte array was specified by #define, so the entire
	 * 64-Bytes are written in this loop.
	 */
	target_addr = 0x00;
	target_data_ptr = 0x00;


          
	while(target_data_ptr < TARGET_DATABUFF_LEN) {     
		temp = DATAOUT(target_data_ptr);
		checksum += temp;

		send_byte(dev, *VECTOR(WRITE_BYTE_START), BITS(WRITE_BYTE_START));	
		send_byte(dev, target_addr, 6);
		send_byte(dev, temp, 8);
		send_byte(dev, *VECTOR(WRITE_BYTE_END), BITS(WRITE_BYTE_END));


               
		/* !!!NOTE:
		 * send_byte() uses MSbits, so inc by '4' to put the 0..63
		 * address into the six MSBit locations.
		 * This can be confusing, but check the logic:
		 * The address is only 6-Bits long. The send_byte() subroutine
		 * will send however-many bits, BUT...always reads them bits
		 * from left-to-right. So in order to pass a value of 0..63
		 * as the address using send_byte(), we have to left justify
		 * the address by 2-Bits. This can be done easily by
		 * incrementing the address each time by '4' rather
		 * than by '1'.
		 */
		target_addr += 4;
		target_data_ptr++;
	}
	return(checksum);
}

/* Load program data from HEX file generated by PSoC Designer into 
 * a 64 byte host ram buffer.
 *    1. Read data from next line in hex file into ram buffer. One record 
 *      (line) is 64 bytes of data.
 *    2. Check host ram buffer + record data (Address, # of bytes) against hex
 *       record checksum at end of record line
 *    3. If error reread data from file or abort
 *    4. Exit this Function and Program block or verify the block.
 */
static int load_program_data(struct tsc_drv_data *dev, int index) 
{
	unsigned char checksum = 0;
	unsigned char target_data_ptr;

	checksum += dev->fw[index].bytes & 0xff;
	checksum += (dev->fw[index].addr >> 8) & 0xff;
	checksum += dev->fw[index].addr & 0xff;
	checksum += dev->fw[index].type & 0xff;

	for (target_data_ptr = 0;
	     target_data_ptr < TARGET_DATABUFF_LEN;
	     target_data_ptr++) {
        	DATAOUT(target_data_ptr) = dev->fw[index].blk[target_data_ptr];
		checksum += DATAOUT(target_data_ptr);
    	}

	checksum = ~checksum + 1;

	if (checksum != dev->fw[index].checksum) {
		printk(KERN_ERR "%s: Block checksum: %02x. Hex file: %02x.\n",
		       DRIVER, checksum, dev->fw[index].checksum);
		 return CHECKSUM_ERROR;
	}
	return PASS;
}

/* Load security data from hex file into 64 byte host ram buffer. */
static void load_security_data(struct tsc_drv_data *dev, unsigned char bank_cnt,
			       unsigned char start, unsigned char length,
			       unsigned char *checksum)
{
	unsigned char target_data_ptr;

	/* Now, write the desired security-bytes for the range specified */
	for (target_data_ptr = start;
	     target_data_ptr < length;
	     target_data_ptr++) {
		/* Write the security bytes to buffer. */ 
		DATAOUT(target_data_ptr) =
			dev->fw[SECURITY_DATA].blk[bank_cnt * length +\
						   target_data_ptr];
		*checksum += DATAOUT(target_data_ptr);
	}
	return;
}

/* Reads and adds the target bank checksum to the referenced accumulator. */
static signed char accumulate_bank_checksum(struct tsc_drv_data *dev,
					    unsigned short* acc)
{
	send_vector(dev, VECTOR(CHECKSUM), BITS(CHECKSUM));
	if (detect_high_low(dev)) {
		printk(KERN_ERR "%s: Failed to detect Hi-Low transition "
		       "after CHECKSUM vector.\n", DRIVER);
		return HILOW_ERROR;
	}
	send_vector(dev, VECTOR(WAIT_AND_POLL_END), BITS(WAIT_AND_POLL_END));

	/* Send Read Checksum vector and get Target Checksum. */

	/* first 11-bits is ReadCKSum-MSB */
	send_vector(dev, dev->vector[READ_CHECKSUM].data, 11);
	/* Two SCLKs between write & read. */
	run_clock(dev, 2);
	*acc += (unsigned short)receive_byte(dev) << 8;
	/* See figure 6 */
	run_clock(dev, 1);
	/* 12 bits starting from 3rd character. */
	send_vector(dev, &(dev->vector[READ_CHECKSUM].data[2]), 12);
	/* Read-LSB Command. */
	run_clock(dev, 2);
	*acc += receive_byte(dev);
	run_clock(dev, 1);
	/* Send the final bit of the command. */
	send_vector(dev, &(dev->vector[READ_CHECKSUM].data[3]), 1);

	return PASS;
}    

/* After programming, the target PSoC must be reset to take it out of 
 * programming mode. This routine performs a reset.
 */
static void release_target(struct tsc_drv_data *dev)
{
#ifdef RESET_MODE
	/* Assert XRES, then release, then disable XRES-Enable */
	xres_high(dev);
	udelay(XRES_CLK_DELAY);
	xres_low(dev);
#else
	/* Set all pins to highZ to avoid back powering the PSoC through
	 * the GPIO protection diodes.
	 */
	scl_hiz(dev);   
	sda_hiz(dev);

	/* Cycle power on the target to cause a reset. */
	vdd_low(dev);

	/* It seems that it's important to leave the power off for
	 * some time value so that the calibration is performed
	 * in the next power up.
	 */
	udelay(POWER_CYCLE_DELAY);
#endif
}

// ****************************************************************************
// *********************** ISSP IOCTL FUNCTION SECTION ************************
// ****************************************************************************
/* Make sure to cut the power before programming. Initialize the vectors
 * and FW data buffers.
 */ 
static int cy8mrln_init_vectors(struct tsc_drv_data *dev)
{
	int i;

	if (dev->verbose) {
		printk("%s: In init_vectors phase.\n", DRIVER);
	}

	/* Turn off the power to start. */
	cy8mrln_do_poweroff(dev);

	for (i = 0; i < NUM_VECTORS; i++) {
		dev->vector[i].data = NULL;
	}
	for (i = 0; i < NUM_FW_LINES; i++) {
		dev->fw[i].blk = NULL;
	}
	return 0;
}

/* Configure SPI CS and CLK (functional mode) to SDA and SCL
 * (program mode). Then reset or power cycle the PSoC in a 
 * special manner to put the PSoC into programming mode.
 */
static int cy8mrln_init_target(struct tsc_drv_data *dev)
{
	int rc;

	if (dev->verbose) {
		printk("%s: In init_target phase.\n", DRIVER);
	}

	/* Requests GPIO for programming. */
	rc = gpio_request(dev->pdata->scl_gpio, "cy8mrln scl");
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get GPIO for scl line.\n",
		       DRIVER);
		return -EFAULT; 
	}
	rc = gpio_request(dev->pdata->mosi_gpio, "cy8mrln mosi");
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get GPIO for mosi line.\n",
		       DRIVER);
		gpio_free(dev->pdata->scl_gpio);
		return -EFAULT; 
	}
	rc = gpio_request(dev->pdata->sda_gpio, "cy8mrln sda");
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get GPIO for sda line.\n",
		       DRIVER);
		gpio_free(dev->pdata->scl_gpio);
		gpio_free(dev->pdata->mosi_gpio);
		return -EFAULT; 
	}

	/* Switch to GPIO mode. */
	dev->pdata->switch_mode(PROGRAM);

#ifdef RESET_MODE
	if (xres_init(dev)) {
		return -EFAULT;
	}
#else
	if ((power_cycle_init(dev))) {
		return -EFAULT;
	}
#endif
	return 0;
}

/* Read the silicon chip ID and match the given one. */
static int cy8mrln_verify_id(struct tsc_drv_data *dev)
{
	if (dev->verbose) {
		printk("%s: In verify_id phase.\n", DRIVER);
	}

	if (verify_silicon_id(dev)) {
		return -EFAULT;
	}
	return 0;
}

/* Program each block with the data in the hex file.
 * Each line in the hex file corresponds to a block data.
 * The hex file contains data for all blocks in the PSoC
 * even though some blocks are empty.
 */
static int cy8mrln_program(struct tsc_drv_data *dev)
{
	int index;
	unsigned char bank_cnt;
	unsigned int blk_cnt;
	unsigned short checksum, hex_chksum;

	if (dev->verbose) {
		printk("%s: In program phase.\n", DRIVER);
	}

	/* Bulk-Erase the Device. */
	if (erase_target(dev)) {
		return -EFAULT;
	}

	/* Program each Flash block with predetermined data. */
	checksum = 0;
	for (bank_cnt = 0; bank_cnt < NUM_BANKS; bank_cnt++) {
#ifdef MULTI_BANK
		/* Set the bank number. */
		set_bank_num(dev, bank_cnt);
#endif        
		for (blk_cnt = 0; blk_cnt < BLOCKS_PER_BANK; blk_cnt++) {

			index = bank_cnt*BLOCKS_PER_BANK+blk_cnt;

			if (load_program_data(dev, index)) {
				return -EFAULT;
			}
			/* Calculte the device checksum as you go. */
			checksum += load_target(dev);
			if (program_target_block(dev, (unsigned char)blk_cnt)) {
				return -EFAULT;
			}
			yield();
		}
	}

	/* Get the device checksum from the hex file. */
	hex_chksum = (dev->fw[CHECKSUM_DATA].blk[0] << 8) |
		     dev->fw[CHECKSUM_DATA].blk[1];

	/* Compare hex file checksum from the data we think we programmed.*/
	if (checksum != hex_chksum) {
		printk(KERN_ERR "%s: Checksum:  %04x. Hex file: %04x.\n",
		       DRIVER, checksum, hex_chksum);
		return -EFAULT;
	}
	return 0;
}

/* Verify procedure handler. We will read back the programmed
 * blocks and compare them against the program data.
 */
static int cy8mrln_verify(struct tsc_drv_data *dev)
{
	int index;
	unsigned char bank_cnt;
	unsigned int blk_cnt;

	if (dev->verbose) {
		printk("%s: In verify phase.\n", DRIVER);
	}

	/* Verify the data block-by-block after programming
	 * all of the blocks and before setting security.
	 */
	for (bank_cnt = 0; bank_cnt < NUM_BANKS; bank_cnt++) {
#ifdef MULTI_BANK
		/* Set the bank number. */
		set_bank_num(dev, bank_cnt);
#endif        
		for (blk_cnt = 0; blk_cnt < BLOCKS_PER_BANK; blk_cnt++) {
			index = bank_cnt*BLOCKS_PER_BANK+blk_cnt;
			if (load_program_data(dev, index)) {
				return -EFAULT;
			}
			if (verify_target_block(dev, (unsigned char)blk_cnt)) {
				return -EFAULT;
			}
		}
	}
	return 0;
}

/* Secure procedure handler. Each bank needs to be secured with
 * special secure data.
 */ 
static int cy8mrln_secure(struct tsc_drv_data *dev)
{
	unsigned char bank_cnt;
	unsigned char checksum = 0;

	if (dev->verbose) {
		printk("%s: In secure phase.\n", DRIVER);
	}

	checksum += dev->fw[SECURITY_DATA].bytes & 0xff;
	checksum += (dev->fw[SECURITY_DATA].addr >> 8) & 0xff;
	checksum += dev->fw[SECURITY_DATA].addr & 0xff;
	checksum += dev->fw[SECURITY_DATA].type & 0xff;

	/* Program security data into target PSoC. */
	for (bank_cnt = 0; bank_cnt < NUM_BANKS; bank_cnt++) {
#ifdef MULTI_BANK
		/* Set the bank number. */
		set_bank_num(dev, bank_cnt);
#endif        
		/* Load one bank of security data from hex file into buffer. */
		load_security_data(dev, bank_cnt, 0, SECURITY_BYTES_PER_BANK,
				   &checksum);

		/* Secure one bank of the target flash. */
		if (secure_target_flash(dev)) {
			return -EFAULT;
		}
	}

	/* Compare the checksum. */
	checksum = ~checksum + 1;

	if (checksum != dev->fw[SECURITY_DATA].checksum) {
		printk(KERN_ERR "%s: Security data checksum: %02x. "
				"Hex file: %02x.\n", DRIVER, checksum,
		       dev->fw[SECURITY_DATA].checksum);
		 return -EFAULT;
	}
	return 0;
}

/* Verify Checksum procedure handler. It will read checksum
 * per bank, but will add them to produce 16 bit device checksum.
 * We will compare that against the checksum we calculated during
 * the program phase and compare them. They need to match the
 * 16 bit checksum stored in the hex file.
 */
static int cy8mrln_verify_checksum(struct tsc_drv_data *dev)
{
	unsigned char bank_cnt;
	unsigned short hex_chksum, target_chksum;

	if (dev->verbose) {
		printk("%s: In verify_checksum phase.\n", DRIVER);
	}

	/* Run the Target-Checksum Verification, and proceed according to
	 * the result. Checksum only valid if every Flash block is programed.
	 */
	target_chksum = 0;
	for (bank_cnt = 0; bank_cnt < NUM_BANKS; bank_cnt++) {
#ifdef MULTI_BANK
		/* Set the bank number. */
		set_bank_num(dev, bank_cnt);
#endif        
		if (accumulate_bank_checksum(dev, &target_chksum)) {
			return -EFAULT;
		}
	}

	/* Get the device checksum from the hex file. */
	hex_chksum = (dev->fw[CHECKSUM_DATA].blk[0] << 8) |
		     dev->fw[CHECKSUM_DATA].blk[1];

	/* Compare the file checksum from the data we actually programmed.*/
	if (target_chksum != hex_chksum) {
		printk(KERN_ERR "%s: Checksum:  %04x. Hex file: %04x.\n",
		       DRIVER, target_chksum, hex_chksum);
		return -EFAULT;
	}
	return 0;
}
/* Free the memory used for vector data and FW data.
 * Switch to normal operation mode by going back to
 * the functional mode (Mode 0) for the SCL and SDA
 * GPIO.
 */
static int cy8mrln_exit_prog(struct tsc_drv_data *dev)
{
	int i;

	if (dev->verbose) {
		printk("%s: In exit_prog phase.\n", DRIVER);
	}

	/* Clean up the GPIO pin states */ 
	release_target(dev);

	/* Free GPIOs used in programming mode. */
	gpio_free(dev->pdata->scl_gpio);
	gpio_free(dev->pdata->mosi_gpio);
	gpio_free(dev->pdata->sda_gpio);

	/* Go back to normal operation mode. */
	dev->pdata->switch_mode(NORM_OP);

	/* Supply the power to perform static calibration.
	 */ 
	cy8mrln_do_poweron(dev, POST_PROG_POWER_UP_TIMEOUT_MSEC);

	/* Turn off the power to wrap up. */
	cy8mrln_do_poweroff(dev);

	/* Free memory. */
	for (i = 0; i < NUM_VECTORS; i++) {
		if (dev->vector[i].data) {
			kfree(dev->vector[i].data);
			dev->vector[i].data = NULL;
		}
	}
	for (i = 0; i < NUM_FW_LINES; i++) {
		if (dev->fw[i].blk) {
			kfree(dev->fw[i].blk);
			dev->fw[i].blk = NULL;
		}
	}

	return 0;
}

// ****************************************************************************
// ******************* ISSP IOCTL HANDLER FUNCTION SECTION ********************
// ****************************************************************************
/* Depending on the IOCTL argument, call correct program stage handler. */
static int cy8mrln_update_prog_state(struct tsc_drv_data *dev, int phase)
{
	int ret = 0;

	switch(phase) {
		case INIT_VECTORS:
			ret = cy8mrln_init_vectors(dev);
			break;
		case INIT_TARGET:
			ret = cy8mrln_init_target(dev);
			break;
		case VERIFY_ID:
			ret = cy8mrln_verify_id(dev);
			break;
		case PROGRAM:
			ret = cy8mrln_program(dev);
			break;
		case VERIFY:
			ret = cy8mrln_verify(dev);
			break;
		case SECURE:
			ret = cy8mrln_secure(dev);
			break;
		case VERIFY_CHECKSUM:
			ret = cy8mrln_verify_checksum(dev);
			break;
		case EXIT_PROG:
			ret = cy8mrln_exit_prog(dev);
			break;
		case NORM_OP:
			break;
		default:
			printk(KERN_ERR "%s: Invalid programming phase: %d\n",
			       DRIVER, phase);
			break;
	}

	/* Don't update the phase if there was an error. */
	if (!ret) {
		dev->prog_phase = phase;
	}
	return ret;
}
/* This IOCTL handler buffers the Cypress defined programming vectors. */
static int cy8mrln_copy_vector(struct tsc_drv_data *dev, cy8mrln_vector_t temp)
{
	if (temp.id >= NUM_VECTORS) {
		return -EINVAL;
	}
	dev->vector[temp.id] = temp;

	dev->vector[temp.id].data = (unsigned char*)kzalloc(temp.bytes, GFP_KERNEL);
	if (copy_from_user((void*)dev->vector[temp.id].data, temp.data, temp.bytes)) {
		kfree(dev->vector[temp.id].data);
		dev->vector[temp.id].data = NULL;
		return -EFAULT;
	}
	return 0;
}
/* This IOCTL handler buffers the Cypress FW data from a hex file. */
static int cy8mrln_copy_fw_data(struct tsc_drv_data *dev, cy8mrln_fw_data_t temp)
{
	if (temp.index >= NUM_FW_LINES) {
		return -EINVAL;
	}
	dev->fw[temp.index] = temp;
	dev->fw[temp.index].blk = (unsigned char*)kzalloc(temp.bytes, GFP_KERNEL);
	if (copy_from_user((void*)dev->fw[temp.index].blk, temp.blk, temp.bytes)) {
		kfree(dev->fw[temp.index].blk);
		dev->fw[temp.index].blk = NULL;
		return -EFAULT;
	}
	return 0;
}

static int 
cy8mrln_ioctl(struct inode * inode, struct file *file, 
              unsigned int cmd, unsigned long args)
{
	int rc = 0;
	struct tsc_drv_data *dev = file->private_data;
	void *usr_ptr   = (void*) (args);
	int   usr_bytes = _IOC_SIZE(cmd);

	PDBG("%s: cmd=0x%08x\n", __FUNCTION__, cmd );
	if (dev->verbose)
		printk("%s: cy8mrln_ioctl cmd=0x%08x\n", DRIVER, cmd);

	switch (cmd) {
		case CY8MRLN_IOCTL_EXIT_WOT_SCAN:
		{					
			/* Force retry into WOT mode with the 
			 * desired WOT sensitivity level if needed.
			 */
			if( dev->scan_rate == 0 )
			{
				/* no need to enter WOT if pSoC is off */
				printk(KERN_ERR "%s: WotEject: ScanRat is 0.\n", DRIVER);
				//rc = -EINVAL;
				//goto Done;
			}

			if(1 == dev->auto_scan) 
			{
				/* Currently in WOT mode, force an eject from WOT mode.
				 * (WOT eject requires TP FW version 210F+).
				 */
				cy8mrln_toggle_cs(dev, PULSE_PSOC_WOT_EJECT ); // 1.5msec pulse
				dev->auto_scan = 0 ;
				/* Start full scan */
				cy8mrln_enable_irq(dev);
				cy8mrln_issue_scan(dev);
			}
			else
			{
				printk(KERN_ERR "%s: WotEject: NotInWotMode.\n", DRIVER);
			}
		} break;
		case CY8MRLN_IOCTL_START_WOT_SCAN_FORCED:
		{
			int newsensval;		
						
			/* Force retry into WOT mode with the 
			 * desired WOT sensitivity level if needed.
			 */
			if( dev->scan_rate == 0 )
			{
				/* no need to enter WOT if pSoC is off */
				printk(KERN_ERR "%s: Can't start WOT. ScanRat is zero.\n", DRIVER);
				rc = -EINVAL;
				goto Done;
			}
			/* Extract wot sensitivity or threshold */

			if (copy_from_user(&newsensval, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: SetWotSenError\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}

			/* Enforce valid range of WOT_THRESHOLD_MIN to WOT_THRESHOLD_MAX */
			if( (newsensval < WOT_THRESHOLD_MIN) || (newsensval > WOT_THRESHOLD_MAX) )
			{
				rc = -EINVAL;
				printk( KERN_ERR "%s: WotSen: OutOfRange:%d (%d<=sens<=%d)\n",
						DRIVER, dev->wot_sen, WOT_THRESHOLD_MIN, WOT_THRESHOLD_MAX);
				goto Done;
			}

			dev->wot_sen = newsensval;
			if(1 == dev->auto_scan) 
			{
				/* Currently in WOT mode, force an eject from WOT mode.
				 * (WOT eject requires TP FW version 210F+).
				 */
				cy8mrln_toggle_cs(dev, PULSE_PSOC_WOT_EJECT ); // 1.5msec pulse
			}
			
			/* Prepare spi command to send to PSoC */
			{
				u16 regval, sensval;
				sensval = dev->wot_sen - WOT_THRESHOLD_MIN;
				sensval &= 0x3F;
				sensval <<= 2;
				/* Setup wot threshold level and wot scan frequency */
				regval = CY8MRLN_CONTROL_REG | 
						(u16)sensval | 
						(u16)(dev->wot_scanrate&0x03) ;

				/* Send autoscan SPI command */
				cy8mrln_reg_write( dev, regval,	CY8MRLN_AUTO_SCAN );

				/* Cancel timers */
#ifdef CONFIG_HIGH_RES_TIMERS
				hrtimer_cancel(&dev->scan_timer);
				hrtimer_cancel(&dev->timeout);
#else
				del_timer_sync(&dev->scan_timer);
				del_timer_sync(&dev->timeout);
#endif

				/* Enable interrupt. */
				cy8mrln_enable_irq(dev);
			}

			dev->auto_scan = 1 ;
		} break;

		case CY8MRLN_IOCTL_GET_WOT_BASELINE_LO:
		{
			if (dev->verbose) {
				printk("%s: GET_WOT_BASELINE_LOW: 0x%X \n",
					DRIVER, dev->wot_baseline_lo);
			}

			if( copy_to_user(usr_ptr, &dev->wot_baseline_lo, usr_bytes) )
			{
				rc = -EFAULT;
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_WOT_BASELINE_HI:
		{
			if (dev->verbose) {
				printk("%s: GET_WOT_BASELINE_HI: 0x%X \n",
					DRIVER, dev->wot_baseline_hi);
			}

			if( copy_to_user(usr_ptr, &dev->wot_baseline_hi, usr_bytes) )
			{
				rc = -EFAULT;
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_SET_WOT_BASELINE_LO:
		{
			int baseline ;

			if (copy_from_user(&baseline, usr_ptr, usr_bytes))
			{
				rc = -EFAULT;
				goto Done;
			}
			/* Enforce valid range of 0 127 */
			if( (baseline < 0) || (baseline > 127) )
			{
				rc = -EINVAL;
				printk( KERN_ERR "%s: SET_WOT_BASELINE_LO: Out Of Range:%d (0<=value<=127)\n", 
						DRIVER, dev->wot_baseline_lo);
				goto Done;
			}
			dev->wot_baseline_lo = baseline ;

			if(dev->verbose) {
				printk("%s: SET_WOT_BASELINE_LO:  %d\n",
					DRIVER, dev->wot_baseline_lo );
			}
		} break;

		
		case CY8MRLN_IOCTL_SET_WOT_BASELINE_HI:
		{
			int baseline ;

			if (copy_from_user(&baseline, usr_ptr, usr_bytes))
			{
				rc = -EFAULT;
				goto Done;
			}

			/* Enforce valid range of 128 to 255 */
			if( (baseline < 128) || (baseline > 255) )
			{
				rc = -EINVAL;
				printk( KERN_ERR"%s: SET_WOT_BASELINE_HI: Out Of Range:%d (128<=value<=255)\n", 
						DRIVER, dev->wot_baseline_hi);
				goto Done;
			}

			
			dev->wot_baseline_hi = baseline;

			if (dev->verbose) {
				printk("%s: SET_WOT_BASELINE_HI: %d \n",
					DRIVER, dev->wot_baseline_hi );
			}
		} break;

		case CY8MRLN_IOCTL_START_WOT_SCAN:
		{
			if( dev->scan_rate == 0 )
			{
				/* no need to enter WOT if pSoC is off */
				printk(KERN_ERR "%s: Can't start WOT:SR=0.\n", DRIVER);
				rc = -EINVAL;
				goto Done;
			}
			dev->auto_scan = 1 ; 

		} break;

		case CY8MRLN_IOCTL_SET_WOT_THRESHOLD:
		{
			int newsensval;
			if (copy_from_user(&newsensval, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: SetWotSenError\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}

			
			/* Enforce valid range of WOT_THRESHOLD_MIN to WOT_THRESHOLD_MAX */
			if( (newsensval < WOT_THRESHOLD_MIN) || (newsensval > WOT_THRESHOLD_MAX) )
			{
				rc = -EINVAL;
				printk( KERN_ERR "%s: WotSen: OutOfRange:%d (%d<=sens<=%d)\n",
						DRIVER, dev->wot_sen, WOT_THRESHOLD_MIN, WOT_THRESHOLD_MAX);
				goto Done;
			}
			dev->wot_sen = newsensval;

			if (dev->verbose) {
				printk("%s: WOT Sensitivity is now %d \n",
					DRIVER, dev->wot_sen );
			}
		} break;

		case CY8MRLN_IOCTL_GET_WOT_THRESHOLD:
		{
			if (dev->verbose) {
				printk("%s: GET_WOT_THRESHOLD: %d \n",
					DRIVER, dev->wot_sen);
			}

			if( copy_to_user(usr_ptr, &dev->wot_sen, usr_bytes ) )
			{
				rc = -EFAULT;
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_SET_WOT_SCANRATE:
		{
			int regval ;
			if (copy_from_user(&regval, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: ErrSetWotScanRate\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}

			/* Enforce valid values of 0, 1, 2, or 3 */
			if( (regval < 0) || (regval > 3) )
			{
				rc = -EINVAL;
				printk( KERN_ERR "%s: WOT Scan Freq: Out Of Range:%d (0<=WotScanFreq<=3)\n",
						DRIVER, regval);
				goto Done;
			}

			dev->wot_scanrate = regval ;
			if (dev->verbose)
			{
				printk("%s: WOT Scan Rate : %d \n",
					DRIVER, dev->wot_scanrate );
			}
		} break;

		case CY8MRLN_IOCTL_GET_WOT_SCANRATE:
		{
			if (dev->verbose) {
				printk("%s: CY8MRLN_IOCTL_GET_WOT_SCANRATE: %d \n",
					DRIVER, dev->wot_scanrate);
			}

			if( copy_to_user(usr_ptr, &dev->wot_scanrate,
				usr_bytes) )
			{
				if (dev->verbose) {
					printk("%s: ErrGetWotScanRate\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_SCANRATE:
		{
			int scan_rate, ns;
			struct timespec diff;

			if (!dev->scan_rate) {
				scan_rate = 0;
			} else {
				/* Calculate the effective scan rate. */
				mutex_lock(&dev->scan_mutex);
				diff = timespec_sub(dev->start_time,
						    dev->start_time_prev);
		     		ns = timespec_to_ns(&diff);
				mutex_unlock(&dev->scan_mutex);
				scan_rate = TSC_USEC_TO_ROUNDED_HZ(ns/1000);
			}

			if (dev->verbose) {
				printk("%s: GET_SCANRATE: %d SIZE: %d\n",
					DRIVER, scan_rate, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &scan_rate,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_SLEEPMODE: 
		{
			if (dev->verbose) {
				printk("%s: GET_SLEEPMODE: %d SIZE: %d\n",
					DRIVER, dev->sleep_mode, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->sleep_mode,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_NUM_DATA_BYTES: 
		{
			if (dev->verbose) {
				printk("%s: GET_NUM_DATA_BYTES: %d SIZE: %d\n",
					DRIVER, dev->num_data_bytes, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->num_data_bytes,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_XFER_OPTION: 
		{
			if (dev->verbose) {
				printk("%s: GET_XFER_OPTION: %d SIZE: %d\n",
					DRIVER, dev->xfer_option, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->xfer_option,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_SPI_MODE: 
		{
			if (dev->verbose) {
				printk("%s: GET_SPI_MODE: %d SIZE: %d\n",
					DRIVER, dev->spidev->mode, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->spidev->mode,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_SPI_CLOCK: 
		{
			if (dev->verbose) {
				printk("%s: GET_SPI_CLOCK: %d SIZE: %d\n",
					DRIVER, dev->spidev->max_speed_hz, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->spidev->max_speed_hz,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_READ_OPTION: 
		{
			if (dev->verbose) {
				printk("%s: GET_READ_OPTION: %d SIZE: %d\n",
					DRIVER, dev->read_option, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->read_option,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_VERBOSE_MODE: 
		{
			if (dev->verbose) {
				printk("%s: GET_VERBOSE_MODE: %d SIZE: %d\n",
					DRIVER, dev->verbose, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->verbose,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_GET_TIMESTAMP_MODE: 
		{
			if (dev->verbose) {
				printk("%s: GET_TIMESTAMP_MODE: %d SIZE: %d\n",
					DRIVER, dev->timestamp, usr_bytes);
			}

			if (copy_to_user(usr_ptr, &dev->timestamp,
					 usr_bytes)) {
				rc = -EFAULT; 
				goto Done;
			}
		} break;

		case CY8MRLN_IOCTL_IDAC_CALIBRATE:
		{
			// Since most of the longer start up time post programming
			// is due to IDAC calibration, we just use the 
			// same timeout here.
			long timeout = POST_PROG_POWER_UP_TIMEOUT_JIFFY;
	
			/* Check if the PSoC is on. */
			if (dev->sleep_mode != CY8MRLN_ON_STATE) {
			rc = -EINVAL;
				goto Done;
			}

			/* Check if the scan is running. */
			if (dev->scan_rate > 0) {
				rc = -EINVAL;
				goto Done;
			}

			/* Enable interrupt. */
			cy8mrln_enable_irq(dev);
			
			clear_bit(IDAC_CALIBRATED, dev->flags);

			cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG, CY8MRLN_CALIBRATE_IDAC);

			/* Go to sleep until we get an interrupt. */
			timeout = wait_event_interruptible_timeout(dev->interrupt_wait,
					test_bit(IDAC_CALIBRATED, dev->flags), timeout);

			/* We received signal or woken up. */
			if (timeout) {
				/* Signal? */
				if (!test_bit(IDAC_CALIBRATED, dev->flags) &&
				    signal_pending(current)) {
					rc = -ERESTARTSYS;
					goto Done;
				}
			/* We timed out. */
			} else {
				printk(KERN_ERR "%s: IDAC_CALIBRATE No interrupt.\n", DRIVER);
				rc = -ETIMEDOUT;
				goto Done;
			}		

		}
		break;

		case CY8MRLN_IOCTL_IDAC_CALIBRATION_STORE:
		{

			long timeout = 3*HZ;

			/* Check if the PSoC is on. */
			if (dev->sleep_mode != CY8MRLN_ON_STATE) {
				rc = -EINVAL;
				goto Done;
			}

			/* Check if the scan is running. */
			if (dev->scan_rate > 0) {
				rc = -EINVAL;
				goto Done;
			}

			/* Enable interrupt. */
			cy8mrln_enable_irq(dev);

			clear_bit(IDAC_CALIBRATION_STORED, dev->flags);

			cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG, CY8MRLN_STORE_IDAC_CALIBRATION);

			/* Go to sleep until we get an interrupt. */
			timeout = wait_event_interruptible_timeout(dev->interrupt_wait,
					test_bit(IDAC_CALIBRATION_STORED, dev->flags), timeout);

			/* Disable interrupt. */
			cy8mrln_disable_irq(dev);

			/* We received signal or woken up. */
			if (timeout) {
				/* Signal? */
				if (!test_bit(IDAC_CALIBRATION_STORED, dev->flags) &&
				    signal_pending(current)) {
					rc = -ERESTARTSYS;
					goto Done;
				}
			/* We timed out. */
			} else {
				printk(KERN_ERR "%s: IDAC_CALIBRATION_STORE No interrupt.\n", DRIVER);
				rc = -ETIMEDOUT;
				goto Done;
			}		
		}
		case CY8MRLN_IOCTL_ILO_CALIBRATE:
		{
			long timeout = HZ*3;

			/* Check if the PSoC is on. */
			if (dev->sleep_mode != CY8MRLN_ON_STATE) {
				rc = -EINVAL;
				goto Done;
			}

			/* Check if the scan is running. */
			if (dev->scan_rate > 0) {
				rc = -EINVAL;
				goto Done;
			}

			/* Enable interrupt. */
			cy8mrln_enable_irq(dev);

			clear_bit(ILO_CALIBRATED, dev->flags);

			cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG, CY8MRLN_CALIBRATE_ILO);

			/* Go to sleep until we get an interrupt. */
			timeout = wait_event_interruptible_timeout(dev->interrupt_wait,
					test_bit(ILO_CALIBRATED, dev->flags), timeout);

			/* Disable interrupt. */
			cy8mrln_disable_irq(dev);

			/* We received signal or woken up. */
			if (timeout) {
				/* Signal? */
				if (!test_bit(ILO_CALIBRATED, dev->flags) &&
				    signal_pending(current)) {
					rc = -ERESTARTSYS;
					goto Done;
				}
			/* We timed out. */
			} else {
				printk(KERN_ERR "%s: ILO Calibrate: No interrupt.\n", DRIVER);
				rc = -ETIMEDOUT;
				goto Done;
			}			
		}
		break;

		case CY8MRLN_IOCTL_GET_IDAC_TABLE:
		case CY8MRLN_IOCTL_GET_FPC_INFO:
		case CY8MRLN_IOCTL_GET_QUERY_DATA:
		{
			unsigned short rev;
			long timeout = SCAN_TIMEOUT_JIFFY;

			if (dev->verbose) {
				printk("%s: GET_QUERY_DATA: %d SIZE: %d\n",
					DRIVER, dev->verbose, usr_bytes);
			}

			/* Check if the PSoC is on. */
			if (dev->sleep_mode != CY8MRLN_ON_STATE) {
			rc = -EINVAL;
				goto Done;
			}

			/* Check if the scan is running. */
			if (dev->scan_rate > 0) {
				rc = -EINVAL;
				goto Done;
			}

			/* Enable interrupt. */
			cy8mrln_enable_irq(dev);

			/* Clear the version number. */
			memset(dev->rev, 0x00, sizeof(dev->rev));

			/* Issue Query command. */
			clear_bit(VERSION_ACQUIRED, dev->flags);
			cy8mrln_reg_write(dev, CY8MRLN_CONTROL_REG, CY8MRLN_QUERY);

			/* Go to sleep until we get an interrupt. */
			timeout = wait_event_interruptible_timeout(dev->version_wait,
					test_bit(VERSION_ACQUIRED, dev->flags), timeout);

			/* Disable interrupt. */
			cy8mrln_disable_irq(dev);

			/* We received signal or woken up. */
			if (timeout) {
				/* Signal? */
				if (!test_bit(VERSION_ACQUIRED, dev->flags) &&
				    signal_pending(current)) {
					rc = -ERESTARTSYS;
					goto Done;
				}
			/* We timed out. */
			} else {
				printk(KERN_ERR "%s: QUERYDATA timeout; No interrupt.", DRIVER);
				rc = -ETIMEDOUT;
				goto Done;
			}

			/* Capture the FW version data. */
			rev = (dev->rev[2] << 8) | dev->rev[3];

			if(CY8MRLN_IOCTL_GET_QUERY_DATA == cmd) {

				/* Just report what we get. The driver doesn't need
				* to know the actual version information.
				*/
				printk(KERN_INFO "%s: Read FW version %04x.\n", DRIVER, rev);

				if (copy_to_user(usr_ptr, &rev,
						 usr_bytes)) {
					rc = -EFAULT;
					goto Done;
				}
			} 
			else if(CY8MRLN_IOCTL_GET_FPC_INFO == cmd) {
				
				printk(KERN_INFO "%s: Read FPC info byte 0x%2x.\n", DRIVER, dev->rev[4]);

				if (copy_to_user(usr_ptr, &dev->rev[4],
						 usr_bytes)) {
					rc = -EFAULT;
					goto Done;
				}

			}
			else if(CY8MRLN_IOCTL_GET_IDAC_TABLE == cmd) {
				struct cy8mrln_idac_table idac;

				if (usr_bytes != sizeof(idac)) {
					rc = -EINVAL;
					goto Done;
				}

				if (copy_from_user(&idac, usr_ptr, usr_bytes)) {
					rc = -EFAULT;
					goto Done;
				}

				if (idac.length != dev->num_data_bytes) {
					rc = -EINVAL;
					goto Done;
				}

				if (copy_to_user(idac.table, dev->rev,
							dev->num_data_bytes)) {
					rc = -EFAULT;
					goto Done;
				}
			}
		} break;


		case CY8MRLN_IOCTL_SET_SCANRATE:
		{
			int new_hz, old_hz;
			long timeout = SCAN_TIMEOUT_JIFFY;

			old_hz = dev->scan_rate;

			if (copy_from_user(&new_hz, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: Set scan rate Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}

			if (dev->verbose) {
				printk("%s: SET_SCANRATE: %d SIZE: %d\n",
					DRIVER, new_hz, usr_bytes);
			}

			dev->scan_rate = new_hz;

			if(old_hz) {
				timeout = wait_event_interruptible_timeout(dev->interrupt_wait,
					test_bit(INTR_HANDLED, dev->flags), timeout);


				/* We received signal or woken up. */
				if (timeout) {
					/* Signal? */
					if (signal_pending(current)) {
						rc = -ERESTARTSYS;
						dev->scan_rate = old_hz ;
						goto Done;
					}
				/* We timed out. */
				} else {
					rc = -ETIMEDOUT;
					dev->scan_rate = old_hz ;

					goto Done;
				}

			}
			/* Start scan. */
			else if (!old_hz && new_hz) {
				/* Enable interrupt. */
				cy8mrln_enable_irq(dev);
				cy8mrln_issue_scan(dev);
			}


		} break;

		case CY8MRLN_IOCTL_SET_SLEEPMODE:
		{
			int sleep_mode;
			if (copy_from_user(&sleep_mode, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: Set Sleep Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}

			/* Check for valid state */
			if ((sleep_mode < CY8MRLN_OFF_STATE) ||
			    (sleep_mode > CY8MRLN_ON_STATE)) { 
				rc = -EINVAL;
				goto Done;
			}

			/* If the same state, silently ignore it. */
			if (sleep_mode == dev->sleep_mode)  {
				goto Done;
			}

		
			if (sleep_mode == CY8MRLN_ON_STATE) {
				if (dev->sleep_mode == CY8MRLN_OFF_STATE) {
					rc = cy8mrln_do_poweron(dev, NORMAL_POWER_UP_TIMEOUT_MSEC);
					if (rc) {
						goto Done;
					}
				}
				cy8mrln_do_resume(dev);

			} else if (sleep_mode == CY8MRLN_SLEEP_STATE) {
				if (dev->sleep_mode == CY8MRLN_OFF_STATE) {
					rc = cy8mrln_do_poweron(dev, NORMAL_POWER_UP_TIMEOUT_MSEC);
					if (rc) {
						goto Done;
					}
				}
				cy8mrln_do_suspend(dev);

			} else if (sleep_mode == CY8MRLN_OFF_STATE) {
				cy8mrln_do_poweroff(dev);
			}
			dev->sleep_mode = sleep_mode;


			if (dev->verbose) {
				printk("%s: SET_SLEEPMODE: %d SIZE: %d\n",
					DRIVER, dev->sleep_mode, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_NUM_DATA_BYTES:
		{
			int num;

			if (copy_from_user(&num, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: Set number data byte Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			dev->num_data_bytes = num;

			if (dev->verbose) {
				printk("%s: SET_NUM_DATA_BYTES: %d SIZE: %d\n",
					DRIVER, dev->num_data_bytes, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_XFER_OPTION:
		{
			int xfer_option;

			if (copy_from_user(&xfer_option, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: SetXferOptn Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			dev->xfer_option = xfer_option;

			if (dev->verbose) {
				printk("%s: SET_XFER_OPTION: %d SIZE: %d\n",
					DRIVER, dev->xfer_option, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_SPI_MODE:
		{
			char spi_mode;

			if (copy_from_user(&spi_mode, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: SetSpiMode Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			dev->spidev->mode = spi_mode;

			if (dev->verbose) {
				printk("%s: SET_SPI_MODE: %d SIZE: %d\n",
					DRIVER, dev->spidev->mode, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_SPI_CLOCK:
		{
			int spi_clock;

			if (copy_from_user(&spi_clock, usr_ptr, usr_bytes))
			{
				if (dev->verbose) {
					printk("%s: SetSpiMode Err\n", DRIVER);
				}

				rc = -EFAULT;
				goto Done;
			}
			dev->spidev->max_speed_hz = spi_clock;

			if (dev->verbose) {
				printk("%s: SET_SPI_CLOCK: %d SIZE: %d\n",
					DRIVER, dev->spidev->max_speed_hz, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_READ_OPTION:
		{
			int read_option;

			if (copy_from_user(&read_option, usr_ptr, usr_bytes)) {
				if (dev->verbose) {
					printk("%s: SetReadOption Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			dev->read_option = read_option;


			if (dev->verbose) {
				printk("%s: SET_READ_OPTION: %d SIZE: %d\n",
					DRIVER, dev->read_option, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_VERBOSE_MODE:
		{
			int verbose;

			if (copy_from_user(&verbose, usr_ptr, usr_bytes)) {
				rc = -EFAULT;
				goto Done;
			}

			dev->verbose = verbose;


			if (dev->verbose) {
				printk("%s: SET_VERBOSE_MODE: %d SIZE: %d\n",
					DRIVER, dev->verbose, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_TIMESTAMP_MODE:
		{
			int timestamp;

			if (copy_from_user(&timestamp, usr_ptr, usr_bytes)) {
				if (dev->verbose) {
					printk("%s: SetTimeStamp Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			dev->timestamp = timestamp;


			if (dev->verbose) {
				printk("%s: SET_TIMESTAMP_MODE: %d SIZE: %d\n",
					DRIVER, dev->timestamp, usr_bytes);
			}
		} break;

		case CY8MRLN_IOCTL_SET_BYTES:
		{
			int num;

			if (copy_from_user(&num, usr_ptr, usr_bytes)) {
				if (dev->verbose) {
					printk("%s: SetBytes Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}

			if (dev->verbose) {
				printk("%s: SET_BYTES: %x SIZE: %d\n",
					DRIVER, num, usr_bytes);
			}
			cy8mrln_write_three_bytes(dev, num);
			dev->setup = 0;
		} break;

		case CY8MRLN_IOCTL_SET_VECTORS:
		{
			cy8mrln_vector_t temp;

			if (copy_from_user(&temp, usr_ptr, sizeof(temp))) {
				if (dev->verbose) {
					printk("%s: SetVectors Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			if (dev->verbose) {
				printk("%s: SET_VECTORS: SIZE: %d\n",
					DRIVER, usr_bytes);
			}
			rc = cy8mrln_copy_vector(dev, temp);
		} break;

		case CY8MRLN_IOCTL_SET_FW_DATA:
		{
			cy8mrln_fw_data_t temp;

			if (copy_from_user(&temp, usr_ptr, sizeof(temp))) {
				if (dev->verbose) {
					printk("%s: SetFwData Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			if (dev->verbose) {
				printk("%s: SET_FW_DATA: SIZE: %d\n",
					DRIVER, usr_bytes);
			}
			rc = cy8mrln_copy_fw_data(dev, temp);
		} break;

		case CY8MRLN_IOCTL_SET_PROG_PHASE:
		{
			int phase;

			if (copy_from_user(&phase, usr_ptr, usr_bytes)) {
				if (dev->verbose) {
					printk("%s: SetProgPhase Err\n", DRIVER);
				}
				rc = -EFAULT;
				goto Done;
			}
			if (dev->verbose) {
				printk("%s: SET_PROG_PHASE: %x SIZE: %d\n",
					DRIVER, phase, usr_bytes);
			}
			rc = cy8mrln_update_prog_state(dev, phase);
		} break;

		default:
		{
			if (dev->verbose)
			{
				printk("%s: Invalid IOCTL", DRIVER);
			}
			rc = -EINVAL;
		} break;
	}
Done:	 
	return rc;
}

static int cy8mrln_open(struct inode *inode, struct file *file)
{
	struct tsc_drv_data *dev;

	/* get device */
	dev = container_of(file->f_op, struct tsc_drv_data, fops);

	PDBG("%s:\n", __FUNCTION__ );
	if (dev->verbose)
		printk("%s: cy8mrln_open\n", DRIVER);

	/* Allow only read. */
        if ((file->f_mode & (FMODE_READ|FMODE_WRITE))
	    != FMODE_READ) {
		return -EINVAL;
	}

	/* check if it is in use */
	if (test_and_set_bit(IS_OPENED, dev->flags)) {
		return -EBUSY;
	}

	/* attach private data */
	file->private_data = dev;

	return 0; 
}

static int cy8mrln_close(struct inode *inode, struct file *file)
{
	struct tsc_drv_data *dev = (struct tsc_drv_data *) file->private_data;

	PDBG("%s:\n", __FUNCTION__ );
	if (dev->verbose)
		printk("%s: cy8mrln_close\n", DRIVER);

	cy8mrln_do_poweroff(dev);
	//set scan rate to zero to disable scanning
	dev->scan_rate = 0;
	cy8mrln_flush_rd_buffers( dev );
	
	dev->sleep_mode = CY8MRLN_OFF_STATE;

	/* mark it as unused */
	clear_bit(IS_OPENED, dev->flags);

	return 0;
}

struct file_operations cy8mrln_fops = {
	.owner   = THIS_MODULE,
	.read    = cy8mrln_read,
	.poll    = cy8mrln_poll,
	.ioctl   = cy8mrln_ioctl,
	.open    = cy8mrln_open,
	.release = cy8mrln_close,
};

static ssize_t cy8mrln_attr_cal_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	ssize_t rc;
	struct spi_device* spi = to_spi_device(dev);
	struct tsc_drv_data* dat = spi_get_drvdata(spi);

	rc = test_and_set_bit(IS_OPENED, dat->flags);
	if (rc) {
		dev_err(dev, "error claiming device\n");
		rc = -EBUSY;
		goto exit;
	}

	rc = cy8mrln_do_poweron(dat, NORMAL_POWER_UP_TIMEOUT_MSEC);
	if (rc < 0)
		goto release;

	cy8mrln_enable_irq(dat);
	clear_bit(IDAC_CALIBRATED, dat->flags);
	dat->cmd = CY8MRLN_CALIBRATE_IDAC;

	rc = cy8mrln_write_three_bytes(dat, 0x27);
	if (rc < 0) {
		dev_err(dev, "error %d initiating calibration\n", rc);
		goto suspend;
	}

	rc = wait_event_interruptible_timeout(dat->interrupt_wait,
				test_bit(IDAC_CALIBRATED, dat->flags), HZ);
	if (!rc) {
		dev_err(dev, "timed out waiting for interrupt\n");
		rc = -ETIMEDOUT;
		goto suspend;
	}

	else if (!test_bit(IDAC_CALIBRATED, dat->flags)) {
		dev_err(dev, "interrupted by signal\n");
		rc = -ERESTARTSYS;
		goto suspend;
	}

	rc = count;
suspend:
	cy8mrln_do_poweroff(dat);
release:
	clear_bit(IS_OPENED, dat->flags);
exit:
	return (rc);
}

static DEVICE_ATTR(cal, S_IWUSR, NULL, cy8mrln_attr_cal_store);

static int __devexit
cy8mrln_remove(struct spi_device *spi)
{
	struct tsc_drv_data *dev = spi_get_drvdata(spi);

	PDBG("%s:\n", __FUNCTION__ );

	misc_deregister(&dev->mdev);

	/* detach drvdata */
	spi_set_drvdata(spi, NULL);

	cy8mrln_do_cleanup(dev);

	/* Kill workqueue */
	destroy_workqueue(dev->tp_wq);
	
	/* Remove sysfs entries */
	device_remove_file(&(spi->dev), &dev_attr_scan_rate);
	device_remove_file(&(spi->dev), &dev_attr_tp_mode);
	device_remove_file(&(spi->dev), &dev_attr_tp_fw_ver);
	device_remove_file(&(spi->dev), &dev_attr_cal);

	/* free irq and gpio */
	gpio_free(irq_to_gpio(spi->irq));
	gpio_free(dev->pdata->enable_gpio);

	kfree(dev->all_buffs[0].ptr);

	/* free context */
	if (dev) 
        	kfree(dev);

	return 0;
}


static int 
cy8mrln_init_queues(struct tsc_drv_data *dev)
{
	int i;
	unsigned char *buf;
	struct txrx_buf *pbuf;

	/* Allocate a 4KB page */
	buf = (unsigned char*)kzalloc(CY8MRLN_BUF_LEN * (CY8MRLN_RX_BUF_NUM +\
				      CY8MRLN_TX_BUF_NUM), GFP_KERNEL);
	if (!buf) {
		return -ENOMEM;
	}

	/* Set the memory with pattern. */
        memset(buf, 0xaa, CY8MRLN_BUF_LEN * (CY8MRLN_RX_BUF_NUM +\
               CY8MRLN_TX_BUF_NUM));

	/* init wait queues */
	init_waitqueue_head(&dev->version_wait); 
	init_waitqueue_head(&dev->rx_wait); 
	init_waitqueue_head(&dev->interrupt_wait); 
	init_waitqueue_head(&dev->powerup_wait); 

	/* init rx lists */
	INIT_LIST_HEAD(&dev->rx_free_list);
	INIT_LIST_HEAD(&dev->rx_xfer_list);

	/* Initialize RX buffers. */
	pbuf = dev->all_buffs;
	for (i = 0; i < CY8MRLN_RX_BUF_NUM; i++)
	{	/* Init entry */
		pbuf->pos  = 0;
		pbuf->len = 0;
		pbuf->state = TXRX_STATE_IDLE;
		pbuf->ptr = buf;

		/* Add it to free list */
		list_add_tail(&pbuf->link, &dev->rx_free_list);

		/* next one */
		buf += CY8MRLN_BUF_LEN;
		pbuf += 1;
	}

	/* Initialize TX buffer */
	dev->txbuf = pbuf;
	pbuf->pos = 0;
	pbuf->ptr = buf;

	return 0;
}

static int __init 
cy8mrln_probe(struct spi_device *spi)
{
	int    rc;
	struct tsc_drv_data *dev;
	struct cy8mrln_platform_data *pdata = spi->dev.platform_data;
    
	PDBG("%s:\n", __FUNCTION__ );

	if (pdata == NULL) {
		printk (KERN_ERR "%s: platform data is undefined\n", DRIVER);
		return -ENODEV;
	}

	/* allocate device context */
	dev = kzalloc(sizeof(struct tsc_drv_data), GFP_KERNEL);
	if (dev == NULL)
		return -ENOMEM;

	dev->spidev = spi;    /* attach spi device */
	dev->pdata  = pdata;  /* and platform specific data */

	/* Init spi interface
	 * CPOL=1 base value of the SCLK is one.
	 * CPHA=1 data read on the rising edge.
	 */
	/* Cypress's SPI_MODE_2 is really SPI_MODE_1 in standard sense. */
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8; 
	spi->max_speed_hz = 100000;
	rc = spi_setup(spi);
	if (rc < 0) {
		goto err1;
	}
	/*
	 * sysfs entries
	 */	
	rc = device_create_file(&(spi->dev), &dev_attr_scan_rate);
	if(rc) {
		goto err1;
	}
	rc = device_create_file(&(spi->dev), &dev_attr_tp_mode);
	if(rc) {
		goto err1;
	}
	rc = device_create_file(&(spi->dev), &dev_attr_tp_fw_ver);
	if(rc) {
		goto err1;
	}
	rc = device_create_file(&(spi->dev), &dev_attr_cal);
	if(rc) {
		goto err1;
	}
	/* Get enable line */
	rc = gpio_request(dev->pdata->enable_gpio, "cy8mrln enable");
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get GPIO for enable line.\n",
		       DRIVER);
		goto err1;
	}
	rc = gpio_direction_output(dev->pdata->enable_gpio, 1);
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to set enable line for output.\n",
		       DRIVER);
		goto err2;
	}

#ifdef CONFIG_EXTERNAL_CS
	rc = gpio_request(dev->pdata->external_cs, "cy8mrln ext cs");
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get GPIO for cs line.\n",
		       DRIVER);
		goto err1;
	}
	rc = gpio_direction_output(dev->pdata->external_cs, 1);
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to set cs line for output.\n",
		       DRIVER);
		goto err2;
	}
#endif

	/* set up interrupt handler */
	rc = gpio_request(irq_to_gpio(spi->irq), "cy8mrln irq");
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get GPIO for interrupt line.\n",
		       DRIVER);
		goto err2;
	}

	
	/* Give touchscreen it's own workqueue */
	dev->tp_wq = create_singlethread_workqueue("tp_wq");
    if( !(dev->tp_wq) ) {
        printk(KERN_ERR "%s:Failed to create workqueue\n", DRIVER);
        goto err3;
    }

	/* setup workq for interrupt task and scan setup task. */
	INIT_WORK(&dev->workq, cy8mrln_work_handler);
	INIT_WORK(&dev->scanq, cy8mrln_scan_handler);

	/* Init lock and mutex. */
	spin_lock_init(&dev->issplock);
	spin_lock_init(&dev->buflock);
	mutex_init(&dev->mutex);
	mutex_init(&dev->scan_mutex);

	/* attach drv data to spi device */
	spi_set_drvdata(spi, dev);

	memcpy(&dev->fops, &cy8mrln_fops, sizeof(struct file_operations));

	/* Register as misc device */
	dev->mdev.minor = MISC_DYNAMIC_MINOR;
	dev->mdev.name = "touchscreen";
	dev->mdev.fops = &dev->fops;
	rc = misc_register(&dev->mdev);
	if (rc < 0) {
		printk(KERN_ERR "%s: Failed to register as misc device\n",
		       DRIVER);
		goto err4;
	}

	/* setup scan timer and scan timeout timer */
#ifdef CONFIG_HIGH_RES_TIMERS
	hrtimer_init(&dev->scan_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->scan_timer.function = cy8mrln_scan_timer;
	hrtimer_init(&dev->timeout, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	dev->timeout.function = cy8mrln_timeout;
#else
	setup_timer(&dev->scan_timer, cy8mrln_scan_timer, (unsigned long)dev);
	setup_timer(&dev->timeout, cy8mrln_timeout, (unsigned long)dev);
#endif

	/* Init queues */
	rc = cy8mrln_init_queues(dev);
	if (rc < 0) {
		printk (KERN_ERR "%s: Can't allocate memory for DMA.\n",
			DRIVER);
		goto err4;
	}

	/* Set default values. */
	dev->scan_rate = 0;
	dev->auto_scan = 0; /* Wait-on-touch/auto-scan flag disabled */
	dev->wot_scanrate = WOT_SCANRATE_512HZ ;
	dev->wot_sen = 10;
	dev->wot_baseline_lo = 0;
	dev->wot_baseline_hi = 132;
	dev->wot_baseline = dev->wot_baseline_hi<<8 | dev->wot_baseline_lo ;
	dev->sleep_mode = CY8MRLN_OFF_STATE;
	dev->setup = 0;
	dev->prog_phase = NORM_OP;
	dev->cmd = CY8MRLN_CMD_INVALID ; 
	dev->timestamp = 0;
	clear_bit(DATA_ACQUIRED, dev->flags);
	clear_bit(VERSION_ACQUIRED, dev->flags);
	clear_bit(SUSPENDED, dev->flags);
	clear_bit(IS_OPENED, dev->flags);
	clear_bit(INTR_HANDLED, dev->flags);
	set_bit(GPIO_IRQ_DISABLED, dev->flags);
	//clear_bit(IN_QUICK_NAP, dev->flags);


	
	/* These are for the user app to change settings of the driver.
	 * We want to probably remove these later.
	 */
	dev->verbose = 0;
	dev->read_option = CY8MRLN_ONE_INT_ZERO_SETUP_BYTES; 
	dev->xfer_option = CY8MRLN_DMA_XFER_OPTION;
	dev->num_data_bytes = CY8MRLN_DEFALUT_NUM_DATA_BYTES;

	return 0;

err4:
	free_irq(spi->irq, dev);
err3:
	gpio_free(irq_to_gpio(spi->irq));
err2:
	gpio_free(dev->pdata->enable_gpio);
	gpio_free(dev->pdata->external_cs);
err1:
	kfree(dev);
	return -ENODEV;
}

static struct spi_driver cy8mrln_driver = {
	.driver = {
		.name   = DRIVER,
		.bus    = &spi_bus_type,
		.owner  = THIS_MODULE,
	},
	.suspend    = cy8mrln_suspend,
	.resume     = cy8mrln_resume,
	.probe      = cy8mrln_probe,
	.remove     = __devexit_p(cy8mrln_remove),
};

static int __init  cy8mrln_init(void)
{
	PDBG("%s:\n", __FUNCTION__ );
	return spi_register_driver(&cy8mrln_driver);
}


static void __exit cy8mrln_exit(void)
{
	PDBG("%s:\n", __FUNCTION__ );
	spi_unregister_driver(&cy8mrln_driver);
}

module_init(cy8mrln_init);
module_exit(cy8mrln_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

