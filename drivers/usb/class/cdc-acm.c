/*
 * cdc-acm.c
 *
 * Copyright (c) 1999 Armin Fuerst	<fuerst@in.tum.de>
 * Copyright (c) 1999 Pavel Machek	<pavel@suse.cz>
 * Copyright (c) 1999 Johannes Erdfelt	<johannes@erdfelt.com>
 * Copyright (c) 2000 Vojtech Pavlik	<vojtech@suse.cz>
 * Copyright (c) 2004 Oliver Neukum	<oliver@neukum.name>
 * Copyright (c) 2005 David Kubicek	<dave@awk.cz>
 * Copyright (c) 2008-2009 Palm, Inc.
 *
 * USB Abstract Control Model driver for USB modems and ISDN adapters
 *
 * Sponsored by SuSE
 *
 * ChangeLog:
 *	v0.9  - thorough cleaning, URBification, almost a rewrite
 *	v0.10 - some more cleanups
 *	v0.11 - fixed flow control, read error doesn't stop reads
 *	v0.12 - added TIOCM ioctls, added break handling, made struct acm kmalloced
 *	v0.13 - added termios, added hangup
 *	v0.14 - sized down struct acm
 *	v0.15 - fixed flow control again - characters could be lost
 *	v0.16 - added code for modems with swapped data and control interfaces
 *	v0.17 - added new style probing
 *	v0.18 - fixed new style probing for devices with more configurations
 *	v0.19 - fixed CLOCAL handling (thanks to Richard Shih-Ping Chan)
 *	v0.20 - switched to probing on interface (rather than device) class
 *	v0.21 - revert to probing on device for devices with multiple configs
 *	v0.22 - probe only the control interface. if usbcore doesn't choose the
 *		config we want, sysadmin changes bConfigurationValue in sysfs.
 *	v0.23 - use softirq for rx processing, as needed by tty layer
 *	v0.24 - change probe method to evaluate CDC union descriptor
 *	v0.25 - downstream tasks paralelized to maximize throughput
 *	v0.26 - multiple write urbs, writesize increased
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/list.h>

#include <linux/serial.h>

#include "cdc-acm.h"

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
#include <linux/modem_activity.h>
#endif

#define PALM_FLOWMSG_SUPPORT

#define ACM_CLOSE_TIMEOUT	15	/* seconds to let writes drain */

/* we still don't have these */
static inline u16 __get_unaligned_le16(const u8 *p)
{
        return p[0] | p[1] << 8;
}

static inline u32 __get_unaligned_le32(const u8 *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
}

static inline u16 get_unaligned_le16(const void *p)
{
        return __get_unaligned_le16((const u8 *)p);
}

static inline u32 get_unaligned_le32(const void *p)
{
	return __get_unaligned_le32((const u8 *)p);
}

/*
 * Version Information
 */
#define DRIVER_VERSION "v0.26"
#define DRIVER_AUTHOR "Armin Fuerst, Pavel Machek, Johannes Erdfelt, Vojtech Pavlik, David Kubicek"
#define DRIVER_DESC "USB Abstract Control Model driver for USB modems and ISDN adapters"

/* #define TIOCMIWAIT   0x545C */
/* #define TIOCGICOUNT  0x545D */

static struct usb_driver acm_driver;
static struct tty_driver *acm_tty_driver;
static struct acm *acm_table[ACM_TTY_MINORS];

static DEFINE_MUTEX(open_mutex);

struct workqueue_struct *acm_workq;

#define ACM_READY(acm)	(acm && acm->dev && acm->used)

#ifdef VERBOSE_DEBUG
#define verbose	1
#else
#define verbose	0
#endif

/*
 * Functions for ACM control messages.
 */

static int acm_ctrl_msg(struct acm *acm, int request, int value, void *buf, int len)
{
	int retval;

	if (!acm || !acm->dev)
		return -ENODEV;

	retval = usb_control_msg(acm->dev, usb_sndctrlpipe(acm->dev, 0),
		request, USB_RT_ACM, value,
		acm->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);
	dev_dbg(&acm->data->dev, "acm_control_msg: rq: 0x%02x val: %#x idx: %d len: %#x result: %d\n", request, value, acm->control->altsetting[0].desc.bInterfaceNumber, len, retval);
	return retval < 0 ? retval : 0;
}

/* devices aren't required to support these requests.
 * the cdc acm descriptor tells whether they do...
 */
static inline int acm_set_control(struct acm *acm, int control)
{
	dev_info(&acm->data->dev, "set control lines: dtr%c rts%c\n",
		 control & ACM_CTRL_DTR ? '+' : '-', control & ACM_CTRL_RTS ? '+' : '-');
	return acm_ctrl_msg(acm, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0);
}

static inline int acm_set_line(struct acm *acm, struct usb_cdc_line_coding *line) {
	dev_info(&acm->data->dev, "set line: dte_rate=%d char_format=%d parity_type=%d data_bits=%d\n",
		 le32_to_cpu(line->dwDTERate),
		 line->bCharFormat, line->bParityType, line->bDataBits);
	return acm_ctrl_msg(acm, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line));
}

static inline int acm_send_break(struct acm *acm, int ms) {
	dev_info(&acm->data->dev, "send break: duration=0x%x\n", ms);
	return acm_ctrl_msg(acm, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0);
}

#ifdef PALM_FLOWMSG_SUPPORT
#define USB_CDC_REQ_SET_FLOW_STATE	0x2A
#define USB_CDC_NOTIFY_GET_FLOW_STATE	0x2B

struct acm_flow_state {
	__le32 us_bit_rate;
	__le32 ds_bit_rate;
};

static int acm_palm_flowmsg(struct acm *acm, int on)
{
	int rc;
	struct acm_flow_state state;

	if (acm->rx_flow == on)
		return 0;
	acm->rx_flow = on;

	if (acm->notification_mode == NOTIFICATION_NOT_EXIST)
		return 0;

	state.us_bit_rate = __cpu_to_le32(4);
	if (on) {
		state.ds_bit_rate = __cpu_to_le32(4);
	} else {
		state.ds_bit_rate = 0;
	}

	rc = acm_ctrl_msg(acm, USB_CDC_REQ_SET_FLOW_STATE, 
	                    0, &state, sizeof(state));

	dev_dbg(&acm->data->dev, "%s: rx flow is %s (rc=%d)\n",
		 __func__, on ? "ON" : "OFF", rc);
	return rc;
}
#endif /* PALM_FLOWMSG_SUPPORT */

/*
 * Write buffer management.
 * All of these assume proper locks taken by the caller.
 */

static int acm_wb_alloc(struct acm *acm)
{
	int i, wbn;
	struct acm_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &acm->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			wb->started = 0;
			wb->t_alloc = jiffies;
			wb->t_delayed = 0;
			wb->t_start = 0;
			wb->t_done = 0;
			wb->t_killed = 0;
			return wbn;
		}
		wbn = (wbn + 1) % ACM_NW;
		if (++i >= ACM_NW)
			return -1;
	}
}

static int acm_wb_is_avail(struct acm *acm)
{
	int i, n;
	unsigned long flags;

	n = ACM_NW;
	spin_lock_irqsave(&acm->write_lock, flags);
	for (i = 0; i < ACM_NW; i++) {
		n -= acm->wb[i].use;
	}
	spin_unlock_irqrestore(&acm->write_lock, flags);
	return n;
}

/*
 * Finish write. Caller must hold acm->write_lock
 */
static void acm_write_done(struct acm *acm, struct acm_wb *wb)
{
	wb->use = 0;
	wb->t_done = jiffies;
	acm->transmitting--;
}

/*
 * Poke write.
 *
 * the caller is responsible for locking
 */

static int acm_start_wb(struct acm *acm, struct acm_wb *wb)
{
	int rc;

	acm->transmitting++;
	wb->started = 1;
	wb->t_start = jiffies;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = acm->dev;

	if ((rc = usb_submit_urb(wb->urb, GFP_ATOMIC)) < 0) {
		dev_err(&acm->data->dev, "usb_submit_urb(write bulk) failed: %d\n", rc);
		acm_write_done(acm, wb);
	}
	return rc;
}

static void usb_mark_last_busy_and_reset_delay(struct usb_device *udev)
{
	usb_mark_last_busy(udev);
	udev->autosuspend_delay = udev->autosuspend_delay_default;
}

static int acm_write_start(struct acm *acm, int wbn)
{
	unsigned long flags;
	struct acm_wb *wb = &acm->wb[wbn];
	int rc;

	spin_lock_irqsave(&acm->write_lock, flags);
	if (!acm->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return -ENODEV;
	}

	dbg("%s susp_count: %d", __func__, acm->susp_count);
	if (acm->susp_count || !list_empty(&acm->delayed_wb_list)) {
//		printk(KERN_DEBUG "%s: delayed wbn=%d (wb=%p)\n",
//		__func__, wbn, wb);
		wb->t_delayed = jiffies;
		list_add_tail(&wb->list, &acm->delayed_wb_list);
		queue_work(acm_workq, &acm->waker);
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return 0;	/* A white lie */
	}
	usb_mark_last_busy_and_reset_delay(acm->dev);

	rc = acm_start_wb(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);

	return rc;

}
/*
 * attributes exported through sysfs
 */
static ssize_t show_caps
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	return sprintf(buf, "%d", acm->ctrl_caps);
}
static DEVICE_ATTR(bmCapabilities, S_IRUGO, show_caps, NULL);

static ssize_t show_country_codes
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	memcpy(buf, acm->country_codes, acm->country_code_size);
	return acm->country_code_size;
}

static DEVICE_ATTR(wCountryCodes, S_IRUGO, show_country_codes, NULL);

static ssize_t show_country_rel_date
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);

	return sprintf(buf, "%d", acm->country_rel_date);
}

static DEVICE_ATTR(iCountryCodeRelDate, S_IRUGO, show_country_rel_date, NULL);

static ssize_t show_wbs
(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);
	int count = 0;
	int i;
	unsigned long flags;

	spin_lock_irqsave(&acm->write_lock, flags);

	for (i = 0; i < ACM_NW; i++) {
		struct acm_wb *wb = &acm->wb[i];

		count += snprintf(buf + count, PAGE_SIZE - count,
			  "wb%02d: use=%d %010lu:%010lu:%010lu:%010lu:%010lu len=%d\n",
				  i, wb->use, wb->t_alloc, wb->t_delayed,
				  wb->t_start, wb->t_done, wb->t_killed, wb->len);
	}

	count += snprintf(buf + count, PAGE_SIZE - count,
			  "transmit=%d\n", acm->transmitting);

	spin_unlock_irqrestore(&acm->write_lock, flags);

	return count;
}

static ssize_t store_wbs
(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct acm *acm = usb_get_intfdata(intf);
        ssize_t rc = count;
	int i;
	unsigned long flags;

	dev_info(&acm->data->dev, "Cancel wbs\n");

	spin_lock_irqsave(&acm->write_lock, flags);

	for (i = 0; i < ACM_NW; i++) {
		struct acm_wb *wb = &acm->wb[i];

		usb_kill_urb(wb->urb);
		/* if the write is not started,
		   we need to drop use flag explicity */
		if (!wb->started)
			wb->use = 0;
		wb->t_killed = jiffies;
	}

	spin_unlock_irqrestore(&acm->write_lock, flags);

        return rc;
}

static DEVICE_ATTR(wbs, S_IRUGO|S_IWUGO, show_wbs, store_wbs);
/*
 * Interrupt handlers for various ACM device responses
 */

/* control interface reports status changes with "interrupt" transfers */
static void acm_ctrl_irq(struct urb *urb)
{
	struct acm *acm_in_context = urb->context;
	struct acm *acm = NULL;
	struct usb_cdc_notification *dr = urb->transfer_buffer;
	unsigned char *data;
	int newctrl;
	int retval;
	int status = urb->status;
	int minor;
	u16 wIndex;
	u16 wValue;
	u16 wLength;

	switch (status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dbg("%s - urb shutting down with status: %d", __func__, status);
		return;
	default:
		dbg("%s - nonzero urb status received: %d", __func__, status);
		goto exit;
	}

	wIndex = get_unaligned_le16(&dr->wIndex);
	wValue = get_unaligned_le16(&dr->wValue);
	wLength = get_unaligned_le16(&dr->wLength);

	/*
	 * a single interrupt endpoint is shared with all interfaces of the modem.
	 * wIndex tells which interface the modem is notifying.
	 */
	for (minor = 0; minor < ACM_TTY_MINORS; minor++) {
		acm = acm_table[minor];
		if (!acm || !acm->dev)
			continue;
		if (acm->control->altsetting[0].desc.bInterfaceNumber == wIndex)
			goto found;
	}
	printk(KERN_ERR "%s: can't find the acm entry for index %d\n", __func__, wIndex);
	goto exit;

found:
	if (acm->notification_mode == NOTIFICATION_NOT_EXIST) {
		printk("%s: ignore notification for index %d", __func__, wIndex);
		goto exit;
	} else {
		if (acm->notification_ifnum != acm_in_context->control->altsetting[0].desc.bInterfaceNumber) {
			printk("%s: notification index %d mismatch (expected %d)\n", __func__,
			       acm_in_context->control->altsetting[0].desc.bInterfaceNumber, acm->notification_ifnum);
			goto exit;
		}
	}

	data = (unsigned char *)(dr + 1);
	switch (dr->bNotificationType) {

		case USB_CDC_NOTIFY_NETWORK_CONNECTION:

			dev_info(&acm->data->dev, "%s network\n", wValue ? "connected to" : "disconnected from");
			break;

		case USB_CDC_NOTIFY_SERIAL_STATE:

			newctrl = get_unaligned_le16(data);

			dev_info(&acm->data->dev, "input control lines: dcd%c dsr%c break%c ring%c framing%c parity%c overrun%c\n",
				newctrl & ACM_CTRL_DCD ? '+' : '-',	newctrl & ACM_CTRL_DSR ? '+' : '-',
				newctrl & ACM_CTRL_BRK ? '+' : '-',	newctrl & ACM_CTRL_RI  ? '+' : '-',
				newctrl & ACM_CTRL_FRAMING ? '+' : '-',	newctrl & ACM_CTRL_PARITY ? '+' : '-',
				newctrl & ACM_CTRL_OVERRUN ? '+' : '-');

			if (acm->tty && !acm->clocal && (acm->ctrlin & ~newctrl & ACM_CTRL_DCD)) {
				dev_info(&acm->data->dev, "DCD dropped. calling tty_hangup\n");
				tty_hangup(acm->tty);
			}

			spin_lock_irq(&acm->modem_lock);
			if ((acm->ctrlin & ACM_CTRL_DCD) != (newctrl & ACM_CTRL_DCD))
				acm->icount.dcd++;
			if ((acm->ctrlin & ACM_CTRL_DSR) != (newctrl & ACM_CTRL_DSR))
				acm->icount.dsr++;
			if ((acm->ctrlin & ACM_CTRL_RI) != (newctrl & ACM_CTRL_RI))
				acm->icount.rng++;
			spin_unlock_irq(&acm->modem_lock);
			wake_up_interruptible(&acm->modem_wait);

			acm->ctrlin = newctrl;

			break;

		case USB_CDC_NOTIFY_SPEED_CHANGE:

			dev_info(&acm->data->dev, "connection speed change: %u bps up, %u bps down\n",
				 get_unaligned_le32(data), get_unaligned_le32(data+4));

			break;

		default:
			dev_info(&acm->data->dev, "notification %#x received: index %d len %d data0 %d data1 %d\n",
				dr->bNotificationType, wIndex, wLength, data[0], data[1]);
			break;
	}
exit:
	if (acm && acm->dev)
		usb_mark_last_busy(acm->dev);
	retval = usb_submit_urb (urb, GFP_ATOMIC);
	if (retval)
		err ("%s - usb_submit_urb failed with result %d",
		     __func__, retval);
}

/* data interface returns incoming bytes, or we got unthrottled */
static void acm_read_bulk(struct urb *urb)
{
	struct acm_rb *buf;
	struct acm_ru *rcv = urb->context;
	struct acm *acm = rcv->instance;
	int status = urb->status;

	dbg("Entering acm_read_bulk with status %d", status);

	if (!ACM_READY(acm)) {
		dev_dbg(&acm->data->dev, "Aborting, acm not ready");
		return;
	}

	if (status) {
		if (status != -ENOENT) /* killed */
			dev_info(&acm->data->dev, "%s: req len: %d actual len: %d status: %d\n", __func__, urb->transfer_buffer_length, urb->actual_length, urb->status);
	}

	buf = rcv->buffer;
	buf->size = urb->actual_length;

	if (likely(status == 0)) {
		usb_mark_last_busy_and_reset_delay(acm->dev);
		spin_lock(&acm->read_lock);
		acm->processing++;
		list_add_tail(&rcv->list, &acm->spare_read_urbs);
		list_add_tail(&buf->list, &acm->filled_read_bufs);
		spin_unlock(&acm->read_lock);
	} else {
		/* we drop the buffer due to an error */
		spin_lock(&acm->read_lock);
		list_add_tail(&rcv->list, &acm->spare_read_urbs);
		list_add(&buf->list, &acm->spare_read_bufs);
		spin_unlock(&acm->read_lock);
		/* nevertheless the tasklet must be kicked unconditionally
		so the queue cannot dry up */
	}
	if (likely(!acm->susp_count))
		if (status != -ETIME) /* time out */
			tasklet_schedule(&acm->urb_task);
}

static void acm_rx_tasklet(unsigned long _acm)
{
	struct acm *acm = (void *)_acm;
	struct acm_rb *buf;
	struct tty_struct *tty = acm->tty;
	struct acm_ru *rcv;
	unsigned long flags;
	unsigned char throttled;

	dbg("Entering acm_rx_tasklet");

	if (!ACM_READY(acm))
	{
		dbg("acm_rx_tasklet: ACM not ready");
		return;
	}

	spin_lock_irqsave(&acm->throttle_lock, flags);
	throttled = acm->throttle;
	spin_unlock_irqrestore(&acm->throttle_lock, flags);
	if (throttled)
	{
		dbg("acm_rx_tasklet: throttled");
		return;
	}

next_buffer:
	spin_lock_irqsave(&acm->read_lock, flags);
	if (list_empty(&acm->filled_read_bufs)) {
		spin_unlock_irqrestore(&acm->read_lock, flags);
		goto urbs;
	}
	buf = list_entry(acm->filled_read_bufs.next,
			 struct acm_rb, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&acm->read_lock, flags);

	dbg("acm_rx_tasklet: procesing buf 0x%p, size = %d", buf, buf->size);

	tty_buffer_request_room(tty, buf->size);
	spin_lock_irqsave(&acm->throttle_lock, flags);
	throttled = acm->throttle;
	spin_unlock_irqrestore(&acm->throttle_lock, flags);
	if (!throttled)
		tty_insert_flip_string(tty, buf->base, buf->size);
	tty_flip_buffer_push(tty);

	if (throttled) {
		printk(KERN_INFO "%s: Throttling noticed\n", __func__);
		spin_lock_irqsave(&acm->read_lock, flags);
		list_add(&buf->list, &acm->filled_read_bufs);
		spin_unlock_irqrestore(&acm->read_lock, flags);
		return;
	}

	spin_lock_irqsave(&acm->read_lock, flags);
	list_add(&buf->list, &acm->spare_read_bufs);
	spin_unlock_irqrestore(&acm->read_lock, flags);
	goto next_buffer;

urbs:
	while (!list_empty(&acm->spare_read_bufs)) {
		spin_lock_irqsave(&acm->read_lock, flags);
		if (list_empty(&acm->spare_read_urbs)) {
			acm->processing = 0;
			spin_unlock_irqrestore(&acm->read_lock, flags);
			return;
		}
		rcv = list_entry(acm->spare_read_urbs.next,
				 struct acm_ru, list);
		list_del(&rcv->list);
		spin_unlock_irqrestore(&acm->read_lock, flags);

		buf = list_entry(acm->spare_read_bufs.next,
				 struct acm_rb, list);
		list_del(&buf->list);

		rcv->buffer = buf;

		usb_fill_bulk_urb(rcv->urb, acm->dev,
				  acm->rx_endpoint,
				  buf->base,
				  acm->readsize,
				  acm_read_bulk, rcv);
		rcv->urb->transfer_dma = buf->dma;
		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		/* This shouldn't kill the driver as unsuccessful URBs are returned to the
		   free-urbs-pool and resubmited ASAP */
		spin_lock_irqsave(&acm->read_lock, flags);
		if (acm->susp_count || usb_submit_urb(rcv->urb, GFP_ATOMIC) < 0) {
			list_add(&buf->list, &acm->spare_read_bufs);
			list_add(&rcv->list, &acm->spare_read_urbs);
			acm->processing = 0;
			spin_unlock_irqrestore(&acm->read_lock, flags);
			return;
		} else {
			spin_unlock_irqrestore(&acm->read_lock, flags);
			dbg("acm_rx_tasklet: sending urb 0x%p, rcv 0x%p, buf 0x%p", rcv->urb, rcv, buf);
		}
	}
	spin_lock_irqsave(&acm->read_lock, flags);
	acm->processing = 0;
	spin_unlock_irqrestore(&acm->read_lock, flags);
}

/* data interface wrote those outgoing bytes */
static void acm_write_bulk(struct urb *urb)
{
	struct acm_wb *wb = urb->context;
	struct acm *acm = wb->instance;
	unsigned long flags;

	if (verbose || urb->status
			|| (urb->actual_length != urb->transfer_buffer_length))
		dev_info(&acm->data->dev, "tx %d/%d bytes -- > %d\n",
			urb->actual_length,
			urb->transfer_buffer_length,
			urb->status);

	spin_lock_irqsave(&acm->write_lock, flags);
	acm_write_done(acm, wb);
	spin_unlock_irqrestore(&acm->write_lock, flags);
	if (ACM_READY(acm))
		queue_work(acm_workq, &acm->work);
	else
		wake_up_interruptible(&acm->drain_wait);
}

static void acm_softint(struct work_struct *work)
{
	struct acm *acm = container_of(work, struct acm, work);

	dev_vdbg(&acm->data->dev, "tx work\n");
	if (!ACM_READY(acm))
		return;
	tty_wakeup(acm->tty);
}

static void acm_waker(struct work_struct *waker)
{
	struct acm *acm = container_of(waker, struct acm, waker);
	int rv;
	struct acm_wb *wb;
	unsigned long flags;

	mutex_lock(&open_mutex);

	if (!acm || !acm->dev)
		goto done;

	rv = usb_autopm_get_interface(acm->control);
	if (rv < 0) {
		printk("Autopm failure in %s (rv=%d)\n", __func__, rv);
		goto done;
	}
	spin_lock_irqsave(&acm->write_lock, flags);
	while (!list_empty(&acm->delayed_wb_list)) {
		wb = list_entry(acm->delayed_wb_list.next, struct acm_wb, list);
		list_del(&wb->list);
//		printk(KERN_DEBUG "%s: wb=%p\n", __func__, wb);
		if (acm->dev)
			usb_mark_last_busy_and_reset_delay(acm->dev);
		acm_start_wb(acm, wb);
	}
	spin_unlock_irqrestore(&acm->write_lock, flags);
	usb_autopm_put_interface(acm->control);

done:
	mutex_unlock(&open_mutex);
}

/*
 * TTY handlers
 */

static int acm_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct acm *acm;
	int rv = -EINVAL;
	int i;
	int rc;
	dbg("Entering acm_tty_open.");

	mutex_lock(&open_mutex);

	acm = acm_table[tty->index];
	if (!acm || !acm->dev)
		goto err_out;
	else
		rv = 0;

	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	tty->driver_data = acm;
	acm->tty = tty;

	/* force low_latency on so that our tty_push actually forces the data through,
	   otherwise it is scheduled, and with high data rates data can get lost. */
	tty->low_latency = 1;

	if (usb_autopm_get_interface(acm->control) < 0) {
		printk("%s: usb_autopm_get_interface failed\n", __func__);
		goto early_bail;
	} else
		acm->control->needs_remote_wakeup = 1;

	mutex_lock(&acm->mutex);
	if (acm->used++) {
		usb_autopm_put_interface(acm->control);
		goto done;
        }

	if (acm->ctrlurb) {
		acm->ctrlurb->dev = acm->dev;
		if ((rc = usb_submit_urb(acm->ctrlurb, GFP_KERNEL))) {
			printk("%s: usb_submit_urb(ctrl irq) failed (rc=%d)\n",
			       __func__, rc);
			goto bail_out;
		}
	}

#ifdef  PALM_FLOWMSG_SUPPORT
	acm_palm_flowmsg(acm, 1); // flow on
#endif

	if (0 > (rc = acm_set_control(acm, acm->ctrlout = ACM_CTRL_DTR | ACM_CTRL_RTS)) &&
	    (acm->ctrl_caps & USB_CDC_CAP_LINE)) {
		printk("%s: acm_set_control failed (rc=%d)\n", __func__, rc);
		goto full_bailout;
	}
	usb_autopm_put_interface(acm->control);

	INIT_LIST_HEAD(&acm->spare_read_urbs);
	INIT_LIST_HEAD(&acm->spare_read_bufs);
	INIT_LIST_HEAD(&acm->filled_read_bufs);
	for (i = 0; i < acm->rx_buflimit; i++) {
		list_add(&(acm->ru[i].list), &acm->spare_read_urbs);
	}
	for (i = 0; i < acm->rx_buflimit; i++) {
		list_add(&(acm->rb[i].list), &acm->spare_read_bufs);
	}

	INIT_LIST_HEAD(&acm->delayed_wb_list);

	acm->throttle = 0;

	tasklet_schedule(&acm->urb_task);

done:
	mutex_unlock(&acm->mutex);
err_out:
	mutex_unlock(&open_mutex);
	return rv;

full_bailout:
	if (acm->ctrlurb)
		usb_kill_urb(acm->ctrlurb);
bail_out:
	usb_autopm_put_interface(acm->control);
	acm->used--;
	mutex_unlock(&acm->mutex);
early_bail:
	mutex_unlock(&open_mutex);
	return -EIO;
}

/*
 * The original acm_tty_unregister() was split into two functions:
 *
 *   acm_tty_unregister_device() - always called if the device is disconnected
 *   acm_free() - called if the device is disconnected *and* not used
 */
static void acm_tty_unregister_device(struct acm *acm)
{
	tty_unregister_device(acm_tty_driver, acm->minor);
	usb_put_intf(acm->control);
	acm_table[acm->minor] = NULL;
}

static void acm_free(struct acm *acm)
{
	int i,nr;

	nr = acm->rx_buflimit;
	if (acm->ctrlurb)
		usb_free_urb(acm->ctrlurb);
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
	for (i = 0; i < nr; i++)
		usb_free_urb(acm->ru[i].urb);
	kfree(acm->country_codes);
	kfree(acm);
}

static void acm_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct acm *acm = tty->driver_data;
	int i,nr;
	int rv;
	int autopm_fail = 0;

	if (!acm || !acm->used)
		return;

	printk("%s:\n", __func__);

	nr = acm->rx_buflimit;
	mutex_lock(&open_mutex);
	if (!--acm->used) {
	retry:
		if (acm->dev) {
			rv = usb_autopm_get_interface(acm->control);
			if (rv < 0) {
				printk("Autopm failure in %s (rv=%d)\n", __func__, rv);
				autopm_fail++;
				if (!acm->dev)
					goto retry;
			}
			acm_set_control(acm, acm->ctrlout = 0);

#ifdef  PALM_FLOWMSG_SUPPORT
			acm_palm_flowmsg(acm, 0); // flow off
#endif
			/* try letting the last writes drain naturally */
			wait_event_interruptible_timeout(acm->drain_wait,
					(ACM_NW == acm_wb_is_avail(acm))
						|| !acm->dev,
					ACM_CLOSE_TIMEOUT * HZ);
			
			if (acm->ctrlurb)
				usb_kill_urb(acm->ctrlurb);
			for (i = 0; i < ACM_NW; i++) {
				usb_kill_urb(acm->wb[i].urb);
				/* if the write is not started,
				   we need to drop use flag explicity */
				if (!acm->wb[i].started)
					acm->wb[i].use = 0;
				acm->wb[i].t_killed = jiffies;
			}
			for (i = 0; i < nr; i++)
				usb_kill_urb(acm->ru[i].urb);
			acm->control->needs_remote_wakeup = 0;
			if (autopm_fail == 0)
				usb_autopm_put_interface(acm->control);
		} else
			acm_free(acm); /* tty is already unregistered */
	}
	mutex_unlock(&open_mutex);
}

static int acm_tty_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct acm *acm = tty->driver_data;
	int stat;
	unsigned long flags;
	int wbn;
	struct acm_wb *wb;

	dbg("Entering acm_tty_write to write %d bytes,", count);

	if (!ACM_READY(acm))
		return -EINVAL;
	if (!count)
		return 0;

	spin_lock_irqsave(&acm->write_lock, flags);
	if ((wbn = acm_wb_alloc(acm)) < 0) {
		spin_unlock_irqrestore(&acm->write_lock, flags);
		return 0;
	}
	wb = &acm->wb[wbn];

	count = (count > acm->writesize) ? acm->writesize : count;
	dbg("Get %d bytes...", count);
	memcpy(wb->buf, buf, count);
	wb->len = count;
	spin_unlock_irqrestore(&acm->write_lock, flags);

	if ((stat = acm_write_start(acm, wbn)) < 0)
		return stat;
	return count;
}

static int acm_tty_write_room(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	if (!ACM_READY(acm))
		return -EINVAL;
	/*
	 * Do not let the line discipline to know that we have a reserve,
	 * or it might get too enthusiastic.
	 */
	return acm_wb_is_avail(acm) ? acm->writesize : 0;
}

static int acm_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	if (!ACM_READY(acm))
		return 0; /* -EINVAL */

	/*
	 * This is inaccurate (overcounts), but it works.
	 */
	return (ACM_NW - acm_wb_is_avail(acm)) * acm->writesize;
}

static void acm_tty_throttle(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	if (!ACM_READY(acm))
		return;
	spin_lock_bh(&acm->throttle_lock);
	acm->throttle = 1;
	spin_unlock_bh(&acm->throttle_lock);
}

static void acm_tty_unthrottle(struct tty_struct *tty)
{
	struct acm *acm = tty->driver_data;
	if (!ACM_READY(acm))
		return;
	spin_lock_bh(&acm->throttle_lock);
	acm->throttle = 0;
	spin_unlock_bh(&acm->throttle_lock);
	tasklet_schedule(&acm->urb_task);
}

static void /* int */ acm_tty_break_ctl(struct tty_struct *tty, int state)
{
	struct acm *acm = tty->driver_data;
	int rv;

	mutex_lock(&open_mutex);

	if (!ACM_READY(acm))
		goto done /* -EINVAL */;

	rv = usb_autopm_get_interface(acm->control);
	if (rv < 0) {
		printk("Autopm failure in %s (rv=%d)\n", __func__, rv);
		goto done;
	}
	rv = acm_send_break(acm, state ? 0xffff : 0);
	if (rv < 0)
		dbg("send break failed");
	usb_autopm_put_interface(acm->control);
	/* return retval; */
done:
	mutex_unlock(&open_mutex);
}

static int acm_tty_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct acm *acm = tty->driver_data;

	if (!ACM_READY(acm))
		return -EINVAL;

	return (acm->ctrlout & ACM_CTRL_DTR ? TIOCM_DTR : 0) |
	       (acm->ctrlout & ACM_CTRL_RTS ? TIOCM_RTS : 0) |
	       (acm->ctrlin  & ACM_CTRL_DSR ? TIOCM_DSR : 0) |
	       (acm->ctrlin  & ACM_CTRL_RI  ? TIOCM_RI  : 0) |
	       (acm->ctrlin  & ACM_CTRL_DCD ? TIOCM_CD  : 0) |
	       TIOCM_CTS;
}

static int acm_tty_tiocmset(struct tty_struct *tty, struct file *file,
			    unsigned int set, unsigned int clear)
{
	struct acm *acm = tty->driver_data;
	unsigned int newctrl;
	int rv;

	mutex_lock(&open_mutex);

	if (!ACM_READY(acm)) {
		rv = -EINVAL;
		goto done;
	}

	newctrl = acm->ctrlout;
	set = (set & TIOCM_DTR ? ACM_CTRL_DTR : 0) | (set & TIOCM_RTS ? ACM_CTRL_RTS : 0);
	clear = (clear & TIOCM_DTR ? ACM_CTRL_DTR : 0) | (clear & TIOCM_RTS ? ACM_CTRL_RTS : 0);

	newctrl = (newctrl & ~clear) | set;

	if (acm->ctrlout == newctrl) {
		rv = 0;
		goto done;
	}

	rv = usb_autopm_get_interface(acm->control);
	if (rv < 0) {
		printk("Autopm failure in %s (rv=%d)\n", __func__, rv);
		goto done;
	}
	rv = acm_set_control(acm, acm->ctrlout = newctrl);
	usb_autopm_put_interface(acm->control);

done:
	mutex_unlock(&open_mutex);
	return rv;
}

static int acm_wait_modem_status(struct acm *acm)
{
	DECLARE_WAITQUEUE(wait, current);
	struct async_icount cprev, cnow;
	int ret;

	spin_lock_irq(&acm->modem_lock);
	memcpy(&cprev, &acm->last_icount, sizeof(struct async_icount));
	memcpy(&cnow, &acm->icount, sizeof(struct async_icount));
	spin_unlock_irq(&acm->modem_lock);

	if ((cnow.rng != cprev.rng) ||
	    (cnow.dsr != cprev.dsr) ||
	    (cnow.dcd != cprev.dcd) ||
	    (cnow.cts != cprev.cts)) {
		/* changed since TIOCGICOUNT was called */
		ret = 0;
		return ret;
	}

	cprev = cnow;

	add_wait_queue(&acm->modem_wait, &wait);

	for (;;) {
		spin_lock_irq(&acm->modem_lock);
		memcpy(&cnow, &acm->icount, sizeof(struct async_icount));
		spin_unlock_irq(&acm->modem_lock);

		set_current_state(TASK_INTERRUPTIBLE);

		if ((cnow.rng != cprev.rng) ||
		    (cnow.dsr != cprev.dsr) ||
		    (cnow.dcd != cprev.dcd) ||
		    (cnow.cts != cprev.cts)) {
			ret = 0;
			break;
		}

		schedule();

		if (acm->dev == NULL) { /* disconnected */
			ret = -EIO;
			break;
		}

		/* see if a signal did it */
		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}

		cprev = cnow;
	}

	current->state = TASK_RUNNING;
	remove_wait_queue(&acm->modem_wait, &wait);

	return ret;
}

static int acm_get_count(struct acm *acm, struct serial_icounter_struct __user *icnt)
{
	struct serial_icounter_struct icount;
	struct async_icount cnow;

	spin_lock_irq(&acm->modem_lock);
	memcpy(&cnow, &acm->icount, sizeof(struct async_icount));
	memcpy(&acm->last_icount, &acm->icount, sizeof(struct async_icount));
	spin_unlock_irq(&acm->modem_lock);

	icount.cts         = cnow.cts;
	icount.dsr         = cnow.dsr;
	icount.rng         = cnow.rng;
	icount.dcd         = cnow.dcd;
	icount.rx          = cnow.rx;
	icount.tx          = cnow.tx;
	icount.frame       = cnow.frame;
	icount.overrun     = cnow.overrun;
	icount.parity      = cnow.parity;
	icount.brk         = cnow.brk;
	icount.buf_overrun = cnow.buf_overrun;

	return copy_to_user(icnt, &icount, sizeof(icount)) ? -EFAULT : 0;
}

static int acm_tty_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct acm *acm = tty->driver_data;
	void __user *uarg = (void __user *)arg;

	if (!ACM_READY(acm))
		return -EINVAL;

	switch (cmd) {
	case TIOCMIWAIT:
		return acm_wait_modem_status(acm);

	case TIOCGICOUNT:
		return acm_get_count(acm, uarg);
	}
	return -ENOIOCTLCMD;
}

static const __u32 acm_tty_speed[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600,
	1200, 1800, 2400, 4800, 9600, 19200, 38400,
	57600, 115200, 230400, 460800, 500000, 576000,
	921600, 1000000, 1152000, 1500000, 2000000,
	2500000, 3000000, 3500000, 4000000
};

static const __u8 acm_tty_size[] = {
	5, 6, 7, 8
};

static void acm_tty_set_termios(struct tty_struct *tty, struct ktermios *termios_old)
{
	struct acm *acm = tty->driver_data;
	struct ktermios *termios = tty->termios;
	struct usb_cdc_line_coding newline;
	int newctrl = acm->ctrlout;
	int rv;

	mutex_lock(&open_mutex);

	if (!ACM_READY(acm))
		goto done;

	newline.dwDTERate = cpu_to_le32p(acm_tty_speed +
		(termios->c_cflag & CBAUD & ~CBAUDEX) + (termios->c_cflag & CBAUDEX ? 15 : 0));
	newline.bCharFormat = termios->c_cflag & CSTOPB ? 2 : 0;
	newline.bParityType = termios->c_cflag & PARENB ?
		(termios->c_cflag & PARODD ? 1 : 2) + (termios->c_cflag & CMSPAR ? 2 : 0) : 0;
	newline.bDataBits = acm_tty_size[(termios->c_cflag & CSIZE) >> 4];

	acm->clocal = ((termios->c_cflag & CLOCAL) != 0);

	if (!newline.dwDTERate) {
		newline.dwDTERate = acm->line.dwDTERate;
		newctrl &= ~ACM_CTRL_DTR;
	} else  newctrl |=  ACM_CTRL_DTR;

	rv = usb_autopm_get_interface(acm->control);
	if (rv < 0) {
		printk("Autopm failure in %s (rv=%d)\n", __func__, rv);
		goto done;
	}

	if (newctrl != acm->ctrlout) {
		acm_set_control(acm, acm->ctrlout = newctrl);
	}

	if (memcmp(&acm->line, &newline, sizeof newline)) {
		memcpy(&acm->line, &newline, sizeof newline);
		dbg("set line: %d %d %d %d", le32_to_cpu(newline.dwDTERate),
			newline.bCharFormat, newline.bParityType,
			newline.bDataBits);
		acm_set_line(acm, &acm->line);
	}

	usb_autopm_put_interface(acm->control);

done:
	mutex_unlock(&open_mutex);
}

/*
 * USB probe and disconnect routines.
 */

/* Little helpers: write/read buffers free */
static void acm_write_buffers_free(struct acm *acm)
{
	int i;
	struct acm_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		usb_buffer_free(usb_dev, acm->writesize, wb->buf, wb->dmah);
	}
}

static void acm_read_buffers_free(struct acm *acm)
{
	struct usb_device *usb_dev = interface_to_usbdev(acm->control);
	int i, n = acm->rx_buflimit;

	for (i = 0; i < n; i++)
		usb_buffer_free(usb_dev, acm->readsize, acm->rb[i].base, acm->rb[i].dma);
}

/* Little helper: write buffers allocate */
static int acm_write_buffers_alloc(struct acm *acm)
{
	int i;
	struct acm_wb *wb;

	for (wb = &acm->wb[0], i = 0; i < ACM_NW; i++, wb++) {
		wb->buf = usb_buffer_alloc(acm->dev, acm->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_buffer_free(acm->dev, acm->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static int acm_probe (struct usb_interface *intf,
		      const struct usb_device_id *id)
{
	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_country_functional_desc *cfd = NULL;
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	struct usb_endpoint_descriptor *epctrl;
	struct usb_endpoint_descriptor *epread;
	struct usb_endpoint_descriptor *epwrite;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct acm *acm;
	int minor;
	int ctrlsize,readsize;
	u8 *buf;
	u8 ac_management_function = 0;
	u8 call_management_function = 0;
	int call_interface_num = -1;
	int data_interface_num;
	unsigned long quirks;
	int num_rx_buf;
	int i;
	int notification_mode = NOTIFICATION_NOT_EXIST;
	int notification_ifnum = -1;

	/* normal quirks */
	quirks = (unsigned long)id->driver_info;
	num_rx_buf = (quirks == SINGLE_RX_URB) ? 1 : ACM_NR;

	/* handle quirks deadly to normal probing*/
	if (quirks == NO_UNION_NORMAL) {
		data_interface = usb_ifnum_to_if(usb_dev, 1);
		control_interface = usb_ifnum_to_if(usb_dev, 0);
		goto skip_normal_probe;
	} else if (quirks == NO_SEPARATE_DATA_INTERFACE) {
		data_interface = control_interface = intf;
		dev_dbg(&intf->dev, "no separate data interface\n");
		goto skip_no_separate_data;
	}
	
	/* normal probing*/
	if (!buffer) {
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	if (!buflen) {
		if (intf->cur_altsetting->endpoint->extralen && intf->cur_altsetting->endpoint->extra) {
			dev_dbg(&intf->dev,"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		} else {
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

	while (buflen > 0) {
		if (buffer [1] != USB_DT_CS_INTERFACE) {
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer [2]) {
			case USB_CDC_UNION_TYPE: /* we've found it */
				if (union_header) {
					dev_err(&intf->dev, "More than one "
						"union descriptor, "
						"skipping ...\n");
					goto next_desc;
				}
				union_header = (struct usb_cdc_union_desc *)
							buffer;
				break;
			case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
				cfd = (struct usb_cdc_country_functional_desc *)buffer;
				break;
			case USB_CDC_HEADER_TYPE: /* maybe check version */ 
				break; /* for now we ignore it */ 
			case USB_CDC_ACM_TYPE:
				ac_management_function = buffer[3];
				break;
			case USB_CDC_CALL_MANAGEMENT_TYPE:
				call_management_function = buffer[3];
				call_interface_num = buffer[4];
				if ((call_management_function & 3) != 3)
					dev_err(&intf->dev, "This device "
						"cannot do calls on its own. "
						"It is no modem.\n");
				break;
			default:
				/* there are LOTS more CDC descriptors that
				 * could legitimately be found here.
				 */
				dev_dbg(&intf->dev, "Ignoring descriptor: "
						"type %02x, length %d\n",
						buffer[2], buffer[0]);
				break;
			}
next_desc:
		buflen -= buffer[0];
		buffer += buffer[0];
	}

	if (!union_header) {
		if (call_interface_num > 0) {
			dev_dbg(&intf->dev,"No union descriptor, using call management descriptor\n");
			data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = call_interface_num));
			control_interface = intf;
		} else {
			dev_dbg(&intf->dev,"No union descriptor, giving up\n");
			return -ENODEV;
		}
	} else {
		control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
		data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = union_header->bSlaveInterface0));
		if (!control_interface || !data_interface) {
			dev_dbg(&intf->dev,"no interfaces\n");
			return -ENODEV;
		}
	}
	
	if (data_interface_num != call_interface_num)
		dev_dbg(&intf->dev,"Separate call control interface. That is not fully supported.\n");

skip_normal_probe:

	/*workaround for switched interfaces */
	if (data_interface->cur_altsetting->desc.bInterfaceClass != CDC_DATA_INTERFACE_TYPE) {
		if (control_interface->cur_altsetting->desc.bInterfaceClass == CDC_DATA_INTERFACE_TYPE) {
			struct usb_interface *t;
			dev_dbg(&intf->dev,"Your device has switched interfaces.\n");

			t = control_interface;
			control_interface = data_interface;
			data_interface = t;
		} else {
			return -EINVAL;
		}
	}

	/* Accept probe requests only for the control interface */
	if (intf != control_interface)
		return -ENODEV;
	
	if (usb_interface_claimed(data_interface)) { /* valid in this context */
		dev_dbg(&intf->dev,"The data interface isn't available\n");
		return -EBUSY;
	}

skip_no_separate_data:
	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2)
		return -EINVAL;

	if (quirks == NO_SEPARATE_DATA_INTERFACE) {
		int i;
		struct usb_endpoint_descriptor *endpoint;

		epctrl = epread = epwrite = NULL;

		for (i = 0; i < data_interface->cur_altsetting->desc.bNumEndpoints; ++i) {
			endpoint = &data_interface->cur_altsetting->endpoint[i].desc;

			if (((endpoint->bEndpointAddress & 0x80) == USB_DIR_IN) &&
			    ((endpoint->bmAttributes & 3) == USB_ENDPOINT_XFER_BULK)) {
				/* we found a bulk in endpoint */
				dbg("found bulk in");
				epread = &data_interface->cur_altsetting->endpoint[i].desc;
			}
			else if (((endpoint->bEndpointAddress & 0x80) == USB_DIR_OUT) &&
			    ((endpoint->bmAttributes & 3) == USB_ENDPOINT_XFER_BULK)) {
				/* we found a bulk out endpoint */
				dbg("found bulk out");
				epwrite = &data_interface->cur_altsetting->endpoint[i].desc;
			}
			else if (((endpoint->bEndpointAddress & 0x80) == USB_DIR_IN) &&
			    ((endpoint->bmAttributes & 3) == USB_ENDPOINT_XFER_INT)) {
				/* we found a interrupt in endpoint */
				int j;
				dbg("found interrupt in");
				/* check if the endpoint is shared with other interfaces */
				for (minor = 0; minor < ACM_TTY_MINORS && acm_table[minor]; minor++) {
					acm = acm_table[minor];
					for (j = 0; j < acm->control->cur_altsetting->desc.bNumEndpoints; ++j) {
						if (acm->control->cur_altsetting->endpoint[i].desc.bEndpointAddress
						    == endpoint->bEndpointAddress) {
							notification_mode = NOTIFICATION_SHARED;
							notification_ifnum = acm->control->cur_altsetting->desc.bInterfaceNumber;
							dev_dbg(&intf->dev, "ep %02x is already claimed for interface %d. skip.\n",
								endpoint->bEndpointAddress, notification_ifnum);
							goto skip;
						}
					}
				}
			skip:
				if (notification_mode != NOTIFICATION_SHARED) {
					epctrl = &control_interface->cur_altsetting->endpoint[i].desc;
					notification_mode = NOTIFICATION_NORMAL;
					notification_ifnum = control_interface->cur_altsetting->desc.bInterfaceNumber;
				}
			}
			else {
				dev_dbg(&intf->dev, "unknown endpoint!?\n");
			}
		}
		if (!epread || !epwrite) {
			dev_dbg(&intf->dev, "some bulk endpoints are missing\n");
			return -EINVAL;
		}
		if (!epctrl) {
			dev_dbg(&intf->dev, "no control endpoint\n");
		}
	} else {
		epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
		epread = &data_interface->cur_altsetting->endpoint[0].desc;
		epwrite = &data_interface->cur_altsetting->endpoint[1].desc;
	}

	/* workaround for switched endpoints */
	if (!usb_endpoint_dir_in(epread)) {
		/* descriptors are swapped */
		struct usb_endpoint_descriptor *t;
		dev_dbg(&intf->dev,"The data interface has switched endpoints\n");
		
		t = epread;
		epread = epwrite;
		epwrite = t;
	}
	dbg("interfaces are valid");
	for (minor = 0; minor < ACM_TTY_MINORS && acm_table[minor]; minor++);

	if (minor == ACM_TTY_MINORS) {
		dev_err(&intf->dev, "no more free acm devices\n");
		return -ENODEV;
	}

	if (!(acm = kzalloc(sizeof(struct acm), GFP_KERNEL))) {
		dev_dbg(&intf->dev, "out of memory (acm kzalloc)\n");
		goto alloc_fail;
	}

	if (epctrl)
		ctrlsize = le16_to_cpu(epctrl->wMaxPacketSize);
	else
		ctrlsize = 0;
	readsize = le16_to_cpu(epread->wMaxPacketSize)* ( quirks == SINGLE_RX_URB ? 1 : 2);
	acm->writesize = le16_to_cpu(epwrite->wMaxPacketSize) * 20;
	acm->control = control_interface;
	acm->data = data_interface;
	acm->minor = minor;
	acm->dev = usb_dev;
	acm->ctrl_caps = ac_management_function;
	acm->ctrlsize = ctrlsize;
	acm->readsize = readsize;
	acm->rx_buflimit = num_rx_buf;
	acm->urb_task.func = acm_rx_tasklet;
	acm->urb_task.data = (unsigned long) acm;
	INIT_WORK(&acm->work, acm_softint);
	INIT_WORK(&acm->waker, acm_waker);
	init_waitqueue_head(&acm->drain_wait);
	spin_lock_init(&acm->throttle_lock);
	spin_lock_init(&acm->write_lock);
	spin_lock_init(&acm->read_lock);
	mutex_init(&acm->mutex);
	acm->rx_endpoint = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);

	spin_lock_init(&acm->modem_lock);
	init_waitqueue_head(&acm->modem_wait);

	acm->notification_mode = notification_mode;
	acm->notification_ifnum = notification_ifnum;

	if (notification_mode == NOTIFICATION_NORMAL) {
		buf = usb_buffer_alloc(usb_dev, ctrlsize, GFP_KERNEL, &acm->ctrl_dma);
		if (!buf) {
			dev_dbg(&intf->dev, "out of memory (ctrl buffer alloc)\n");
			goto alloc_fail2;
		}
		acm->ctrl_buffer = buf;
	} else
		acm->ctrl_buffer = NULL;

	if (acm_write_buffers_alloc(acm) < 0) {
		dev_dbg(&intf->dev, "out of memory (write buffer alloc)\n");
		goto alloc_fail4;
	}

	if (notification_mode == NOTIFICATION_NORMAL) {
		acm->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
		if (!acm->ctrlurb) {
			dev_dbg(&intf->dev, "out of memory (ctrlurb kmalloc)\n");
			goto alloc_fail5;
		}
	} else
		acm->ctrlurb = NULL;
	for (i = 0; i < num_rx_buf; i++) {
		struct acm_ru *rcv = &(acm->ru[i]);

		if (!(rcv->urb = usb_alloc_urb(0, GFP_KERNEL))) {
			dev_dbg(&intf->dev, "out of memory (read urbs usb_alloc_urb)\n");
			goto alloc_fail7;
		}

		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		rcv->instance = acm;
	}
	for (i = 0; i < num_rx_buf; i++) {
		struct acm_rb *rb = &(acm->rb[i]);

		rb->base = usb_buffer_alloc(acm->dev, readsize,
				GFP_KERNEL, &rb->dma);
		if (!rb->base) {
			dev_dbg(&intf->dev, "out of memory (read bufs usb_buffer_alloc)\n");
			goto alloc_fail7;
		}
	}
	for(i = 0; i < ACM_NW; i++)
	{
		struct acm_wb *snd = &(acm->wb[i]);

		if (!(snd->urb = usb_alloc_urb(0, GFP_KERNEL))) {
			dev_dbg(&intf->dev, "out of memory (write urbs usb_alloc_urb)");
			goto alloc_fail7;
		}

		usb_fill_bulk_urb(snd->urb, usb_dev, usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, acm->writesize, acm_write_bulk, snd);
		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = acm;
	}

	usb_set_intfdata (intf, acm);

	i = device_create_file(&intf->dev, &dev_attr_bmCapabilities);
	if (i < 0)
		goto alloc_fail8;
	i = device_create_file(&intf->dev, &dev_attr_wbs);
	if (i < 0)
		goto alloc_fail9;

	if (cfd) { /* export the country data */
		acm->country_codes = kmalloc(cfd->bLength - 4, GFP_KERNEL);
		if (!acm->country_codes)
			goto skip_countries;
		acm->country_code_size = cfd->bLength - 4;
		memcpy(acm->country_codes, (u8 *)&cfd->wCountyCode0, cfd->bLength - 4);
		acm->country_rel_date = cfd->iCountryCodeRelDate;

		i = device_create_file(&intf->dev, &dev_attr_wCountryCodes);
		if (i < 0) {
			kfree(acm->country_codes);
			goto skip_countries;
		}

		i = device_create_file(&intf->dev, &dev_attr_iCountryCodeRelDate);
		if (i < 0) {
			kfree(acm->country_codes);
			goto skip_countries;
		}
	}

skip_countries:
	if (notification_mode == NOTIFICATION_NORMAL) {
		usb_fill_int_urb(acm->ctrlurb, usb_dev, usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
				 acm->ctrl_buffer, ctrlsize, acm_ctrl_irq, acm, epctrl->bInterval);
		acm->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		acm->ctrlurb->transfer_dma = acm->ctrl_dma;
	}

	dev_info(&intf->dev, "ttyACM%d: USB ACM device\n", minor);

	acm_set_control(acm, acm->ctrlout);

	acm->line.dwDTERate = cpu_to_le32(9600);
	acm->line.bDataBits = 8;
	acm_set_line(acm, &acm->line);

	if (control_interface != data_interface) {
		usb_driver_claim_interface(&acm_driver, data_interface, acm);
		usb_set_intfdata(data_interface, acm);
	}

	usb_get_intf(control_interface);
	tty_register_device(acm_tty_driver, minor, &control_interface->dev);

	acm_table[minor] = acm;

	printk("%s: set parent->autosuspend_disabled = 0\n", __func__);
	usb_dev->parent->autosuspend_disabled = 0;

#ifdef  CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
	modem_activity_set_usb_dev((unsigned long)usb_dev);
#endif

	return 0;
alloc_fail9:
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
alloc_fail8:
	for (i = 0; i < ACM_NW; i++)
		usb_free_urb(acm->wb[i].urb);
alloc_fail7:
	acm_read_buffers_free(acm);
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(acm->ru[i].urb);
	if (acm->ctrlurb)
		usb_free_urb(acm->ctrlurb);
alloc_fail5:
	acm_write_buffers_free(acm);
alloc_fail4:
	if (acm->ctrl_buffer)
		usb_buffer_free(usb_dev, ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
alloc_fail2:
	kfree(acm);
alloc_fail:
	return -ENOMEM;
}

static void stop_data_traffic(struct acm *acm, int disconnected)
{
	int i;
	dbg("Entering stop_data_traffic");

	tasklet_disable(&acm->urb_task);

	if (acm->ctrlurb) {
		usb_kill_urb(acm->ctrlurb);
	}
	for(i = 0; i < ACM_NW; i++) {
		usb_kill_urb(acm->wb[i].urb);
		if (disconnected) {
			/* if the write is not started,
			   we need to drop use flag explicity */
			if (!acm->wb[i].started)
				acm->wb[i].use = 0;
		}
		acm->wb[i].t_killed = jiffies;
	}
	for (i = 0; i < acm->rx_buflimit; i++)
		usb_kill_urb(acm->ru[i].urb);

	tasklet_enable(&acm->urb_task);

	cancel_work_sync(&acm->work);
	if (disconnected)
		cancel_work_sync(&acm->waker);
}

static void acm_disconnect(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);

	/* sibling interface is already cleaning up */
	if (!acm)
		return;

	mutex_lock(&open_mutex);

#ifdef  CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
	modem_activity_set_usb_dev((unsigned long)0);
#endif

	if (acm->country_codes){
		device_remove_file(&acm->control->dev,
				&dev_attr_wCountryCodes);
		device_remove_file(&acm->control->dev,
				&dev_attr_iCountryCodeRelDate);
	}
	device_remove_file(&acm->control->dev, &dev_attr_bmCapabilities);
	device_remove_file(&acm->control->dev, &dev_attr_wbs);
	acm->dev = NULL;
	usb_set_intfdata(acm->control, NULL);
	if (acm->control != acm->data)
		usb_set_intfdata(acm->data, NULL);

	printk("%s: set parent->autosuspend_disabled = 1\n", __func__);
	usb_dev->parent->autosuspend_disabled = 1;

	stop_data_traffic(acm, 1);

	INIT_LIST_HEAD(&acm->delayed_wb_list);
	acm_write_buffers_free(acm);
	if (acm->ctrl_buffer)
		usb_buffer_free(usb_dev, acm->ctrlsize, acm->ctrl_buffer, acm->ctrl_dma);
	acm_read_buffers_free(acm);

	if (acm->control != acm->data)
		usb_driver_release_interface(&acm_driver, intf == acm->control ? acm->data : intf);

	wake_up_interruptible(&acm->modem_wait);

	acm_tty_unregister_device(acm);
	if (!acm->used) {
		acm_free(acm);
		mutex_unlock(&open_mutex);
		return;
	}

	mutex_unlock(&open_mutex);

	if (acm->tty)
		tty_hangup(acm->tty);
}

#ifdef CONFIG_PM
static int acm_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct acm *acm = usb_get_intfdata(intf);
	int cnt;

	if( acm->dev )
		acm->dev->autosuspend_delay = msecs_to_jiffies(300);
	if( acm->dev->parent )
		acm->dev->parent->autosuspend_delay = msecs_to_jiffies(300);

	if (acm->dev && acm->dev->auto_pm) {
		int b;

#ifdef CONFIG_PALM_QC_MODEM_HANDSHAKING_SUPPORT
		if (!modem_activity_handshake_is_enabled())
			return -EBUSY;
#endif
		spin_lock_irq(&acm->read_lock);
		spin_lock(&acm->write_lock);
		b = acm->processing + acm->transmitting;
		spin_unlock(&acm->write_lock);
		spin_unlock_irq(&acm->read_lock);
		if (b)
			return -EBUSY;
	}

	spin_lock_irq(&acm->read_lock);
	spin_lock(&acm->write_lock);
	cnt = acm->susp_count++;
	spin_unlock(&acm->write_lock);
	spin_unlock_irq(&acm->read_lock);

	if (cnt)
		return 0;
	/*
	we treat opened interfaces differently,
	we must guard against open
	*/
	mutex_lock(&acm->mutex);

	if (acm->used)
	{
#ifdef PALM_FLOWMSG_SUPPORT
		acm_palm_flowmsg(acm, 0); // flow off
#endif
		stop_data_traffic(acm, 0);
	}

	mutex_unlock(&acm->mutex);
	return 0;
}

static int acm_resume(struct usb_interface *intf)
{
	struct acm *acm = usb_get_intfdata(intf);
	int rv = 0;
	int cnt;

	spin_lock_irq(&acm->read_lock);
	acm->susp_count -= 1;
	cnt = acm->susp_count;
	spin_unlock_irq(&acm->read_lock);

	if (cnt)
		return 0;

	mutex_lock(&acm->mutex);
	if (acm->used) {
		if (acm->ctrlurb) {
			rv = usb_submit_urb(acm->ctrlurb, GFP_NOIO);
			if (rv < 0)
				goto err_out;
		}
#ifdef PALM_FLOWMSG_SUPPORT
		acm_palm_flowmsg(acm, 1); // flow on
#endif
		tasklet_schedule(&acm->urb_task);
	}

err_out:
	mutex_unlock(&acm->mutex);
	return rv;
}

#endif /* CONFIG_PM */
/*
 * USB driver structure.
 */

static struct usb_device_id acm_ids[] = {
	/* quirky and broken devices */
	{ USB_DEVICE(0x0870, 0x0001), /* Metricom GS Modem */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0e8d, 0x0003), /* FIREFLY, MediaTek Inc; andrey.arapov@gmail.com */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0482, 0x0203), /* KYOCERA AH-K3001V */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x079b, 0x000f), /* BT On-Air USB MODEM */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0ace, 0x1608), /* ZyDAS 56K USB MODEM */
	.driver_info = SINGLE_RX_URB, /* firmware bug */
	},
	{ USB_DEVICE(0x0ace, 0x1611), /* ZyDAS 56K USB MODEM - new version */
	.driver_info = SINGLE_RX_URB, /* firmware bug */
	},
	{ USB_DEVICE(0x22b8, 0x7000), /* Motorola Q Phone */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0803, 0x3095), /* Zoom Telephonics Model 3095F USB MODEM */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x0572, 0x1321), /* Conexant USB MODEM CX93010 */
	.driver_info = NO_UNION_NORMAL, /* has no union descriptor */
	},
	{ USB_DEVICE(0x05c6, 0x3197), /* Qualcomm Modem */
	.driver_info = NO_SEPARATE_DATA_INTERFACE, /* control and data are
						      on the same interface */
	},
	{ USB_DEVICE(0x05c6, 0x6000), /* Qualcomm Modem */
	.driver_info = NO_SEPARATE_DATA_INTERFACE, /* control and data are
						      on the same interface */
	},

	/* control interfaces with various AT-command sets */
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_V25TER) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_PCCA101_WAKE) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_GSM) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_3G	) },
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_CDMA) },

	/* NOTE:  COMM/ACM/0xff is likely MSFT RNDIS ... NOT a modem!! */
	{ }
};

MODULE_DEVICE_TABLE (usb, acm_ids);

static struct usb_driver acm_driver = {
	.name =		"cdc_acm",
	.probe =	acm_probe,
	.disconnect =	acm_disconnect,
#ifdef CONFIG_PM
	.suspend =	acm_suspend,
	.resume =	acm_resume,
#endif
	.id_table =	acm_ids,
#ifdef CONFIG_PM
	.supports_autosuspend = 1,
#endif
};

/*
 * TTY driver structures.
 */

static const struct tty_operations acm_ops = {
	.open =			acm_tty_open,
	.close =		acm_tty_close,
	.write =		acm_tty_write,
	.write_room =		acm_tty_write_room,
	.ioctl =		acm_tty_ioctl,
	.throttle =		acm_tty_throttle,
	.unthrottle =		acm_tty_unthrottle,
	.chars_in_buffer =	acm_tty_chars_in_buffer,
	.break_ctl =		acm_tty_break_ctl,
	.set_termios =		acm_tty_set_termios,
	.tiocmget =		acm_tty_tiocmget,
	.tiocmset =		acm_tty_tiocmset,
};

/*
 * Init / exit.
 */

static int __init acm_init(void)
{
	int retval;
	acm_tty_driver = alloc_tty_driver(ACM_TTY_MINORS);
	if (!acm_tty_driver)
		return -ENOMEM;
	acm_tty_driver->owner = THIS_MODULE,
	acm_tty_driver->driver_name = "acm",
	acm_tty_driver->name = "ttyACM",
	acm_tty_driver->major = ACM_TTY_MAJOR,
	acm_tty_driver->minor_start = 0,
	acm_tty_driver->type = TTY_DRIVER_TYPE_SERIAL,
	acm_tty_driver->subtype = SERIAL_TYPE_NORMAL,
	acm_tty_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	acm_tty_driver->init_termios = tty_std_termios;
	acm_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | HUPCL | CLOCAL;
	tty_set_operations(acm_tty_driver, &acm_ops);

	retval = tty_register_driver(acm_tty_driver);
	if (retval) {
		put_tty_driver(acm_tty_driver);
		return retval;
	}

	retval = usb_register(&acm_driver);
	if (retval) {
		tty_unregister_driver(acm_tty_driver);
		put_tty_driver(acm_tty_driver);
		return retval;
	}

	acm_workq = create_freezeable_workqueue("cdc-acm");
	if (!acm_workq)
		return -ENOMEM;

	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_VERSION ":"
	       DRIVER_DESC "\n");

	return 0;
}

static void __exit acm_exit(void)
{
	if (acm_workq)
		destroy_workqueue(acm_workq);

	usb_deregister(&acm_driver);
	tty_unregister_driver(acm_tty_driver);
	put_tty_driver(acm_tty_driver);
}

module_init(acm_init);
module_exit(acm_exit);

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

