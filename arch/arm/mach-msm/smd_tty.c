/* arch/arm/mach-msm/smd_tty.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/delay.h>

#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>

#include <asm/arch/msm_smd.h>
#include "smd_private.h"

#include <asm/uaccess.h>
#include <linux/serial.h>

#define MAX_SMD_TTYS 53

static DEFINE_MUTEX(smd_tty_lock);

struct smd_tty_info {
	smd_channel_t *ch;
	struct tty_struct *tty;
	int open_count;
	struct work_struct tty_work;
	unsigned char last_cnt_dcd, last_cnt_dsr;
};

static struct smd_tty_info smd_tty[MAX_SMD_TTYS];
static struct workqueue_struct *smd_tty_wq;

char *tty_map_tbl[] =
{
	/*  0 */  "DS",
	/*  1 */  "DIAG",
	/*  2 */  "RPCCALL",
	/*  3 */  "RPCRPY",
	/*  4 */  "BT",
	/*  5 */  "CONTROL",
	/*  6 */  "MCPY_RVD",
	/*  7 */  "DATA1",
	/*  8 */  "DATA2",
	/*  9 */  "DATA3",
	/* 10 */  "DATA4",
	/* 11 */  "DATA5",
	/* 12 */  "DATA6",
	/* 13 */  "DATA7",
	/* 14 */  "DATA8",
	/* 15 */  "DATA9",
	/* 16 */  "DATA10",
	/* 17 */  "DATA11",
	/* 18 */  "DATA12",
	/* 19 */  "DATA13",
	/* 20 */  "DATA14",
	/* 21 */  "DATA15",
	/* 22 */  "DATA16",
	/* 23 */  "DATA17",
	/* 24 */  "DATA18",
	/* 25 */  "DATA19",
	/* 26 */  "DATA20",
	/* 27 */  "GPSNMEA",
	/* 28 */  "BRG_1",
	/* 29 */  "BRG_2",
	/* 30 */  "BRG_3",
	/* 31 */  "BRG_4",
	/* 32 */  "BRG_5",
	/* 33 */  "CS_A2M",
	/* 34 */  "CS_A2Q6",
	/* 35 */  "CS_M2Q6",
	/* 36 */  "LOOPBACK",

	/* new dynamically allocated ctl ports */
	/* 37 */  "DATA0_CNTL",  /* name does not exist */
	/* 38 */  "DATA1_CNTL",
	/* 39 */  "DATA2_CNTL",
	/* 40 */  "DATA3_CNTL",
	/* 41 */  "DATA4_CNTL",
	/* 42 */  "DATA5_CNTL",  /* <- smdcntl0 */
	/* 43 */  "DATA6_CNTL",  /* <- smdcntl1 */
	/* 44 */  "DATA7_CNTL",  /* <- smdcntl2 */
	/* 45 */  "DATA8_CNTL",
	/* 46 */  "DATA9_CNTL",
	/* 47 */  "DATA10_CNTL",
	/* 48 */  "DATA11_CNTL",
	/* 49 */  "DATA12_CNTL",
	/* 50 */  "DATA13_CNTL",
	/* 51 */  "DATA14_CNTL",
	/* 52 */  "DATA15_CNTL",
};

static void smd_tty_work_func(struct work_struct *work)
{
	unsigned char *ptr;
	int avail;
	struct smd_tty_info *info = container_of(work,
						struct smd_tty_info,
						tty_work);
	struct tty_struct *tty = info->tty;

	if (!tty)
		return;

	for (;;) {
		if (test_bit(TTY_THROTTLED, &tty->flags)) break;

		mutex_lock(&smd_tty_lock);
		if (info->ch == 0) {
			mutex_unlock(&smd_tty_lock);
			break;
		}

		avail = smd_read_avail(info->ch);
		if (avail == 0) {
			mutex_unlock(&smd_tty_lock);
			break;
		}

		avail = tty_prepare_flip_string(tty, &ptr, avail);

		if (smd_read(info->ch, ptr, avail) != avail) {
			/* shouldn't be possible since we're in interrupt
			** context here and nobody else could 'steal' our
			** characters.
			*/
			printk(KERN_ERR "OOPS - smd_tty_buffer mismatch?!");
		}
		mutex_unlock(&smd_tty_lock);

		tty_flip_buffer_push(tty);
	}

	/* XXX only when writable and necessary */
	tty_wakeup(tty);
}

static void smd_tty_notify(void *priv, unsigned event)
{
	struct smd_tty_info *info = priv;

	if (event != SMD_EVENT_DATA)
		return;

	queue_work(smd_tty_wq, &info->tty_work);
}

static int smd_tty_open(struct tty_struct *tty, struct file *f)
{
	int res = 0;
	int n = tty->index;
	struct smd_tty_info *info;

	if ((n < 0) || (n >= MAX_SMD_TTYS))
		return -ENODEV;

	info = smd_tty + n;

	mutex_lock(&smd_tty_lock);
	tty->driver_data = info;

	if (info->open_count++ == 0) {
		info->tty = tty;
		if (!info->ch) {
			if (strncmp(tty_map_tbl[n], "LOOPBACK", 9) == 0) {
				/* set smsm state to SMSM_SMD_LOOPBACK state
				** and wait allowing enough time for Modem side
				** to open the loopback port (Currently, this is
				** this is effecient than polling).
				*/
				smsm_change_state(SMSM_APPS_STATE,
						  0, SMSM_SMD_LOOPBACK);
				msleep(100);
			} else if (strncmp(tty_map_tbl[n], "DS", 3) == 0) {
				tty->low_latency = 1;
			}
			res = smd_open(tty_map_tbl[n], &info->ch, info,
				       smd_tty_notify);
		}
	}
	mutex_unlock(&smd_tty_lock);

	return res;
}

static void smd_tty_close(struct tty_struct *tty, struct file *f)
{
	struct smd_tty_info *info = tty->driver_data;

	if (info == 0)
		return;

	mutex_lock(&smd_tty_lock);
	if (--info->open_count == 0) {
		info->tty = 0;
		tty->driver_data = 0;
		if (info->ch) {
			smd_close(info->ch);
			info->ch = 0;
		}
	}
	mutex_unlock(&smd_tty_lock);
}

static int smd_tty_write(struct tty_struct *tty, const unsigned char *buf, int len)
{
	struct smd_tty_info *info = tty->driver_data;
	int avail;

	/* if we're writing to a packet channel we will
	** never be able to write more data than there
	** is currently space for
	*/
	avail = smd_write_avail(info->ch);
	if (len > avail)
		len = avail;

	return smd_write(info->ch, buf, len);
}

static int smd_tty_write_room(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	return smd_write_avail(info->ch);
}

static int smd_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	return smd_read_avail(info->ch);
}

static void smd_tty_unthrottle(struct tty_struct *tty)
{
	struct smd_tty_info *info = tty->driver_data;
	queue_work(smd_tty_wq, &info->tty_work);
	return;
}

static int smd_tty_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct smd_tty_info *info = tty->driver_data;

	return smd_tiocmget(info->ch);
}

static int smd_tty_tiocmset(struct tty_struct *tty, struct file *file,
				unsigned int set, unsigned int clear)
{
	struct smd_tty_info *info = tty->driver_data;

	return smd_tiocmset(info->ch, set, clear);
}

static int smd_wait_serial_status(struct smd_tty_info *info)
{
	DECLARE_WAITQUEUE(wait, current);
	int ret;
	unsigned int cnt_dcd, cnt_dsr;
	unsigned int prev_cnt_dcd, prev_cnt_dsr;

	prev_cnt_dcd = info->last_cnt_dcd;
	prev_cnt_dsr = info->last_cnt_dsr;

	smd_get_serial_counters(info->ch, &cnt_dcd, &cnt_dsr);

	if ((cnt_dcd != prev_cnt_dcd) ||
	    (cnt_dsr != prev_cnt_dsr)) {
		/* changed since TIOCGICOUNT was called */
		ret = 0;
		return ret;
	}

	add_wait_queue(smd_get_serial_state_wait(info->ch), &wait);

	for (;;) {
		smd_get_serial_counters(info->ch, &cnt_dcd, &cnt_dsr);

		set_current_state(TASK_INTERRUPTIBLE);

		if ((cnt_dcd != prev_cnt_dcd) ||
		    (cnt_dsr != prev_cnt_dsr)) {
			ret = 0;
			break;
		}

		schedule();

		/* REVISIT return EIO if the modem is reset */

		/* see if a signal did it */
		if (signal_pending(current)) {
			ret = -EINTR;
			break;
		}
	}

	current->state = TASK_RUNNING;
	remove_wait_queue(smd_get_serial_state_wait(info->ch), &wait);

	return ret;
}

static int smd_get_serial_icounter(struct smd_tty_info *info,
				   struct serial_icounter_struct __user *icnt)
{
	unsigned int cnt_dcd, cnt_dsr;
	struct serial_icounter_struct icount;

	smd_get_serial_counters(info->ch, &cnt_dcd, &cnt_dsr);

	memset(&icount, 0, sizeof(icount));
	icount.dcd = info->last_cnt_dcd = cnt_dcd;
	icount.dsr = info->last_cnt_dsr = cnt_dsr;

	return copy_to_user(icnt, &icount, sizeof(icount)) ? -EFAULT : 0;
}

static int smd_tty_ioctl(struct tty_struct *tty, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct smd_tty_info *info = tty->driver_data;
	void __user *uarg = (void __user *)arg;

	switch (cmd) {
	case TIOCMIWAIT:
		return smd_wait_serial_status(info);

	case TIOCGICOUNT:
		return smd_get_serial_icounter(info, uarg);
	}
	return -ENOIOCTLCMD;
}

static struct tty_operations smd_tty_ops = {
	.open = smd_tty_open,
	.close = smd_tty_close,
	.write = smd_tty_write,
	.write_room = smd_tty_write_room,
	.chars_in_buffer = smd_tty_chars_in_buffer,
	.unthrottle = smd_tty_unthrottle,
	.tiocmget = smd_tty_tiocmget,
	.tiocmset = smd_tty_tiocmset,
	.ioctl = smd_tty_ioctl,
};

static struct tty_driver *smd_tty_driver;

static int __init smd_tty_init(void)
{
	int ret;
	int i;

	smd_tty_wq = create_singlethread_workqueue("smd_tty");
	if (smd_tty_wq == 0)
		return -ENOMEM;

	smd_tty_driver = alloc_tty_driver(MAX_SMD_TTYS);
	if (smd_tty_driver == 0) {
		destroy_workqueue(smd_tty_wq);
		return -ENOMEM;
	}

	smd_tty_driver->owner = THIS_MODULE;
	smd_tty_driver->driver_name = "smd_tty_driver";
	smd_tty_driver->name = "smd";
	smd_tty_driver->major = 0;
	smd_tty_driver->minor_start = 0;
	smd_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	smd_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	smd_tty_driver->init_termios = tty_std_termios;
	smd_tty_driver->init_termios.c_iflag = 0;
	smd_tty_driver->init_termios.c_oflag = 0;
	smd_tty_driver->init_termios.c_cflag = B38400 | CS8 | CREAD;
	smd_tty_driver->init_termios.c_lflag = 0;
	smd_tty_driver->flags = TTY_DRIVER_RESET_TERMIOS |
		TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(smd_tty_driver, &smd_tty_ops);

	ret = tty_register_driver(smd_tty_driver);
	if (ret) return ret;

	for (i = 0; i < MAX_SMD_TTYS; i++) {
		INIT_WORK(&smd_tty[i].tty_work, smd_tty_work_func);
	}

	/* this should be dynamic */
	tty_register_device(smd_tty_driver, SMD_PORT_DS, 0);
	tty_register_device(smd_tty_driver, SMD_PORT_DIAG, 0);
	tty_register_device(smd_tty_driver, SMD_PORT_DATA1, 0);
	tty_register_device(smd_tty_driver, SMD_PORT_DATA11, 0);
	tty_register_device(smd_tty_driver, SMD_PORT_GPS_NMEA, 0);
	tty_register_device(smd_tty_driver, SMD_PORT_LOOPBACK, 0);
	tty_register_device(smd_tty_driver, SMD_PORT_DATA19, 0);

	return 0;
}

module_init(smd_tty_init);
