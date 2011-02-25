/*
 * linux/drivers/misc/omap-twl4030-rtc.c
 * 
 * TWL4030 Real Time Clock interface for Linux
 * 
 * Based on drivers/char/omap-rtc.c:
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Author: George G. Davis <gdavis@mvista.com> or <source@mvista.com>
 *
 * Initially based on linux-2.4.20/drivers/char/rtc.c
 * Copyright (C) 1996 Paul Gortmaker
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/arch/hardware.h>
#include <asm/irq.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/bits.h>
#include <asm/arch/power_companion.h>
#include <asm/mach-types.h>
#include <asm/arch/cpu.h>

#include "twl4030-rtc.h"

#define OMAP_TWL4030RTC_NAME	"omap-twl4030rtc"
#define module			TWL4030_MODULE_RTC
#define ALL_TIME_REGS		6
#define TWL4030_IS_OPEN         0x01	/* means /dev/misc/omap-twl4030rtc is in use     */

#ifdef CONFIG_ARCH_OMAP34XX
#define GPIO_RTC		22
#endif

#ifdef CONFIG_ARCH_OMAP24XX
#define GPIO_RTC		118
#endif

#define PWR_RTC_INT_CLR          (0x08)
#define IRQ_DATA_MASK           ~(0xff)
#define IRQ_DATA_SAVE           IRQ_DATA_MASK+1

#define YEAR_START		1970
#define YEAR_END		169	/* 70 to 169 is the year range - 100 years */
#define Y2K			100
#define MONTH_MAX		12
#define HOUR_MAX		24
#define MINUTE_MAX		60
#define SECONDS_MAX		60

/* Local BCD/BIN conversion macros: */
#ifdef BCD_TO_BIN
#undef BCD_TO_BIN
#endif
#define BCD_TO_BIN(val) 	((val)=((val)&15) + ((val)>>4)*10)

#ifdef BIN_TO_BCD
#undef BIN_TO_BCD
#endif
#define BIN_TO_BCD(val) 	((val)=(((val)/10)<<4) + (val)%10)


//#define OMAP24XX_DIR_OUTPUT 1

static unsigned long rtc_freq = 0;	/* Current periodic IRQ rate:
					   bit 0: every second
					   bit 1: every minute
					   bit 2: every hour
					   bit 3: every day */

/***************** Function Prototypes ***********************/
static struct fasync_struct *rtc_async_queue;
static DECLARE_WAIT_QUEUE_HEAD(rtc_wait);
static int rtc_read_proc(char *page, char **start, off_t off,
			 int count, int *eof, void *data);
static ssize_t twl4030rtc_read(struct file *file, char *buf,
			       size_t count, loff_t * ppos);
static int twl4030rtc_open(struct inode *, struct file *);
static int twl4030rtc_release(struct inode *, struct file *);
static int twl4030rtc_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, unsigned long arg);
static unsigned int twl4030rtc_poll(struct file *file, poll_table * wait);
static int twl4030rtc_fasync(int, struct file *, int);
static int get_rtc_time(struct rtc_time *rtc_tm);
static int get_rtc_alm_time(struct rtc_time *alm_tm);
static int set_rtc_irq_bit(unsigned char bit);
static int mask_rtc_irq_bit(unsigned char bit);
/***************** Function Prototypes - END ***********************/


static int omap_twl4030rtc_probe(struct platform_device *pdev);
#ifdef CONFIG_PM
static unsigned char rtc_int_mask;	/* save interrupt mask during suspend */
static int rtc_suspended = 0;
static wait_queue_head_t rtc_suspend_wq;

#define omap_twl4030rtc_lockout(f) \
	if (rtc_suspended) {\
		int err;\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		err = wait_event_interruptible(rtc_suspend_wq,\
					rtc_suspended == 0);\
		if (err < 0)\
			return err;\
	}

static int omap_twl4030rtc_suspend(struct platform_device *pdev, pm_message_t state);
static int omap_twl4030rtc_resume(struct platform_device *pdev);
#else
#define omap_twl4030rtc_lockout(f)
#endif

/*
 * rtc_status is never changed by rtc_interrupt, and ioctl/open/close is
 * protected by the big kernel lock.
 */
static unsigned long rtc_status = 0;	/* bitmapped status byte.       */
/* Global variable for holding all RTC interrupts information */
static unsigned long rtc_irq_data = 0;	/* our output to the world      */

/*
 *	If this driver ever becomes modularised, it will be really nice
 *	to make the epoch retain its value across module reload...
 */

static int epoch = 1900;	/* year corresponding to 0x00   */

static const unsigned char days_in_mo[] =
    { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

struct semaphore twl4030rtc_lock;

/** 
 * Function : twl4030rtc_write_u8
 * Description : supports 1 byte write to TWL4030 RTC registers.
 * 
 * param data 
 * param reg 
 * 
 * returns on success, negative on failure 
 */

static int twl4030rtc_write_u8(u8 data, u8 reg)
{
	int ret;

	ret = twl4030_i2c_write_u8(module, data, reg);
	if (ret < 0) {
		printk(KERN_WARNING
		       "Could not write TWL4030 register %X - returned %d[%x]\n",
		       reg, ret, ret);
	}
	return ret;
}

/** 
 * Function : twl4030rtc_read_u8
 * Description : supports 1 byte read from TWL4030 RTC register.
 * 
 * param data 
 * param reg 
 * 
 * returns on success, negative on failure 
 */
static int twl4030rtc_read_u8(u8 * data, u8 reg)
{
	int ret;

	ret = twl4030_i2c_read_u8(module, data, reg);
	if (ret < 0) {
		printk(KERN_WARNING
		       "Could not read TWL4030 register %X - returned %d[%x]\n",
		       reg, ret, ret);
	}
	return ret;
}

/** 
 * Function :  twl4030rtc_interrupt
 * Description:
 * A very tiny interrupt handler. It runs with SA_INTERRUPT set.
 * 
 * param irq 
 * param dev_id 
 * param regs 
 * 
 * return 
 */

static irqreturn_t twl4030rtc_interrupt(int irq, void *dev_id)
{
	u8 RdReg;
	int ret;
	/*
	 *      Either an alarm interrupt or update complete interrupt.
	 *      We store the status in the low byte and the number of
	 *      interrupts received since the last read in the remainder
	 *      of rtc_irq_data.
	 */

	/* clear the RTC interrupt in TWL4030 power module */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &RdReg, R_PWR_ISR1);
	if (ret)
		return IRQ_NONE;		
	
	/* Addition to enable shared interrupt*/
	if(!(RdReg & PWR_RTC_INT_CLR) )
		return IRQ_NONE;

	RdReg |= PWR_RTC_INT_CLR;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_INT, RdReg, R_PWR_ISR1);
	if (ret)
		return IRQ_NONE;
			
	rtc_irq_data += IRQ_DATA_SAVE;
	rtc_irq_data &= IRQ_DATA_MASK;
	ret = twl4030rtc_read_u8(&RdReg, REG_RTC_STATUS_REG);
	if (ret)
		return IRQ_NONE;
	rtc_irq_data |= RdReg;
	ret = twl4030rtc_write_u8(RdReg | BIT_RTC_STATUS_REG_ALARM_M,
			    REG_RTC_STATUS_REG);
	if (ret)
		return IRQ_NONE;
				
	/* Strange behaviour with t2...need to write into ISR register one more
	 * time to clear the interrupt. Othwise, the same RTC event generates
	 * 2 interrupts in a row.
	 */
	ret = set_bits_companion_reg(TWL4030_MODULE_INT, RTC_IT, R_PWR_ISR1);
	if (ret)
		return IRQ_NONE;

	/* Now do the rest of the actions */
	wake_up_interruptible(&rtc_wait);

	kill_fasync(&rtc_async_queue, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

 /*     Now all the various file operations that we export. */


/** 
 * Function : twl4030rtc_read
 * Description:
 * Udates alarm and periodic timer interrupt events 
 * to application layer.
 * 
 * param file 
 * param buf 
 * param count 
 * param ppos 
 * 
 * returns greater than zero on success 
 * and negative value on error.
 */

static ssize_t twl4030rtc_read(struct file *file, char *buf,
			       size_t count, loff_t * ppos)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long data;
	ssize_t retval;

	omap_twl4030rtc_lockout(file);

	if (count < sizeof(unsigned long))
		return -EINVAL;

	add_wait_queue(&rtc_wait, &wait);
	set_current_state(TASK_INTERRUPTIBLE);

	for (;;) {
		down(&twl4030rtc_lock);
		data = rtc_irq_data;
		if (data != 0) {
			rtc_irq_data = 0;
			break;
		}
		up(&twl4030rtc_lock);

		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			goto out;
		}
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			goto out;
		}
		schedule();
	}

	up(&twl4030rtc_lock);
	retval = put_user(data, (unsigned long *)buf);
	if (!retval)
		retval = sizeof(unsigned long);
      out:
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&rtc_wait, &wait);

	return retval;
}

/** 
* Function : check_data
* Description : 
* Checks the user supplied data for maximum and minimum ranges
* for year,month,day, hour,minute and secconds. Also validates leap year.
* 
* param check 
* 
* returns 0 on success -EINVAL for invalid values.
*/
static int check_data(struct rtc_time *check)
{
	unsigned char leap_yr;

	check->tm_year += epoch;
	check->tm_mon += 1;	/* since month starts with 1 not zero */

	if (check->tm_year < YEAR_START)
		return -EINVAL;
	if ((check->tm_mon > MONTH_MAX) || (check->tm_mday == 0))
		return -EINVAL;
	leap_yr = ((!(check->tm_year % 4) && (check->tm_year % 100))
		   || !(check->tm_year % 400));
	if (check->tm_mday >
	    (days_in_mo[check->tm_mon] + ((check->tm_mon == 2) && leap_yr)))
		return -EINVAL;
	if ((check->tm_hour >= HOUR_MAX) || (check->tm_min >= MINUTE_MAX)
	    || (check->tm_sec >= SECONDS_MAX))
		return -EINVAL;
	if ((check->tm_year -= epoch) > YEAR_END)
		return -EINVAL;
	if (check->tm_year > Y2K)
		check->tm_year -= Y2K;
	return 0;
}

/** 
 * Function : twl4030rtc_ioctl
 * Description:
 * Supports Device IO control macros.
 * 
 * param inode 
 * param file 
 * param cmd 
 * param arg 
 * 
 * returns 0 on success and negative value on failure. 
 */

static int twl4030rtc_ioctl(struct inode *inode, struct file *file,
			    unsigned int cmd, unsigned long arg)
{
	struct rtc_time wtime;

	omap_twl4030rtc_lockout(file);
	
	switch (cmd) {
	case RTC_AIE_OFF:
		{
			int ret;
			/* Mask alarm int. enab. bit    */
			if ((ret =
			     mask_rtc_irq_bit
			     (BIT_RTC_INTERRUPTS_REG_IT_ALARM_M))
			    < 0)
				return ret;
			else
				return 0;
		}
	case RTC_AIE_ON:
		{
			int ret;
			/* Allow alarm interrupts.      */
			if ((ret =
			     set_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M))
			    < 0)
				return ret;
			else
				return 0;
		}
	case RTC_UIE_OFF:
	case RTC_PIE_OFF:
		{
			int ret;
			/* Mask ints from RTC updates.  */
			if ((ret =
			     mask_rtc_irq_bit
			     (BIT_RTC_INTERRUPTS_REG_IT_TIMER_M))
			    < 0)
				return ret;
			else
				return 0;
		}
	case RTC_UIE_ON:
	case RTC_PIE_ON:
		{
			int ret;
			/* Allow ints for RTC updates.  */
			if ((ret =
			     set_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M))
			    < 0)
				return ret;
			else
				return 0;
		}
	case RTC_ALM_READ:	/* Read the present alarm time */
		{
			int ret;
			/*
			 * This returns a struct rtc_time. Reading >= 0xc0
			 * means "don't care" or "match all". Only the tm_hour,
			 * tm_min, and tm_sec values are filled in.
			 */
			memset(&wtime, 0, sizeof(struct rtc_time));
			ret = get_rtc_alm_time(&wtime);
			break;
		}
	case RTC_ALM_SET:	/* Store a time into the alarm */
		{
			struct rtc_time alm_tm;
			unsigned char alarm_data[ALL_TIME_REGS + 1];
			int ret;

			if (copy_from_user(&alm_tm, (struct rtc_time *)arg,
					   sizeof(struct rtc_time)))
				return -EFAULT;

			/* Validate the user data */
			ret = check_data(&alm_tm);
			if (ret)
				return ret;

			alarm_data[1] = BIN_TO_BCD(alm_tm.tm_sec);
			alarm_data[2] = BIN_TO_BCD(alm_tm.tm_min);
			alarm_data[3] = BIN_TO_BCD(alm_tm.tm_hour);
			alarm_data[4] = BIN_TO_BCD(alm_tm.tm_mday);
			alarm_data[5] = BIN_TO_BCD(alm_tm.tm_mon);
			alarm_data[6] = BIN_TO_BCD(alm_tm.tm_year);

			down(&twl4030rtc_lock);
			/* update all the alarn registers in one shot */
			ret =
			    twl4030_i2c_write(TWL4030_MODULE_RTC, alarm_data,
					      REG_ALARM_SECONDS_REG,
					      ALL_TIME_REGS);

			if (ret < 0) {
				printk(KERN_ERR
				       "twl4030_i2c_write error...!!!\n");
			}
			up(&twl4030rtc_lock);

			return ret;
		}
	case RTC_RD_TIME:	/* Read the time/date from RTC  */
		{
			int ret;
			memset(&wtime, 0, sizeof(struct rtc_time));
			ret = get_rtc_time(&wtime);
			break;
		}
	case RTC_SET_TIME:	/* Set the RTC */
		{
			struct rtc_time rtc_tm;
			unsigned char save_control;
			unsigned char rtc_data[ALL_TIME_REGS + 1];
			int ret;

			if (!capable(CAP_SYS_TIME))
				return -EACCES;

			if (copy_from_user(&rtc_tm, (struct rtc_time *)arg,
					   sizeof(struct rtc_time)))
				return -EFAULT;

			/* Validate the user data */
			ret = check_data(&rtc_tm);
			if (ret)
				return ret;

			rtc_data[1] = BIN_TO_BCD(rtc_tm.tm_sec);
			rtc_data[2] = BIN_TO_BCD(rtc_tm.tm_min);
			rtc_data[3] = BIN_TO_BCD(rtc_tm.tm_hour);
			rtc_data[4] = BIN_TO_BCD(rtc_tm.tm_mday);
			rtc_data[5] = BIN_TO_BCD(rtc_tm.tm_mon);
			rtc_data[6] = BIN_TO_BCD(rtc_tm.tm_year);

			down(&twl4030rtc_lock);

			/* Stop RTC while updating the TC registers */
			if ((ret =
			     twl4030rtc_read_u8(&save_control,
						REG_RTC_CTRL_REG)) < 0) {
				goto err_out;
			}
			save_control &= ~BIT_RTC_CTRL_REG_STOP_RTC_M;
			if ((ret =
			     twl4030rtc_write_u8(save_control,
						 REG_RTC_CTRL_REG) < 0)) {
				goto err_out;
			}

			/* update all the alarm registers in one shot */
			ret =
			    twl4030_i2c_write(TWL4030_MODULE_RTC, rtc_data,
					      REG_SECONDS_REG, ALL_TIME_REGS);
			if (ret < 0) {
				printk(KERN_ERR
				       "twl4030_i2c_read error...!!!\n");
			}

			/* Start back RTC */
			save_control |= BIT_RTC_CTRL_REG_STOP_RTC_M;
			if ((ret =
			     twl4030rtc_write_u8(save_control,
						 REG_RTC_CTRL_REG)) < 0) {
				goto err_out;
			}
		      err_out:
			up(&twl4030rtc_lock);
			return ret;
		}
	case RTC_EPOCH_READ:	/* Read the epoch.      */
		{
			return put_user(epoch, (unsigned long *)arg);
		}
	case RTC_EPOCH_SET:	/* Set the epoch.       */
		{
			/*
			 * There were no RTC clocks before 1900.
			 */
			if (arg < 1900)
				return -EINVAL;

			if (!capable(CAP_SYS_TIME))
				return -EACCES;

			epoch = arg;
			return 0;
		}
	case RTC_IRQP_READ:	/* Read the periodic IRQ rate.  */
		{
			return put_user(rtc_freq, (unsigned long __user *)arg);
		}
	case RTC_IRQP_SET:	/* Set periodic IRQ rate.       */
		{
			u8 RdReg;
			int ret;
			/* The TWL4030 RTC supports periodic IRQ rate of maximum 1 HZ  
			 * and minimum of 1/60*60*24 Hz */
			/* RTC_INTERRUPTS_REG:
			 * BIT1 BIT0   update interrupt
			 *  0    0        Every Second
			 *  0    1        Every Minute
			 *  1    0        Every hour
			 *  1    1        Every day
			 */

			if (arg < 0 || arg > 3)
				return -EINVAL;

			down(&twl4030rtc_lock);
			if ((ret =
			     twl4030rtc_read_u8(&RdReg,
						REG_RTC_INTERRUPTS_REG)) < 0) {
				goto irqp_out;
			}

			RdReg &= ~(BIT_RTC_INTERRUPTS_REG_EVERY_M);
			RdReg |= arg;
			if ((ret =
			     twl4030rtc_write_u8(RdReg, REG_RTC_INTERRUPTS_REG))
			    < 0) {
				goto irqp_out;
			}
			rtc_freq = arg;
		      irqp_out:
			up(&twl4030rtc_lock);
			return ret;
		}

	default:
		return -EINVAL;
	}
	return copy_to_user((void *)arg, &wtime, sizeof(wtime)) ? -EFAULT : 0;
}

/** 
 * Function : twl4030rtc_open
 * Description:
 * We enforce only one user at a time here with the open/close.
 * Also clear the previous interrupt data on an open, and clean
 * up things on a close.
 * 
 * param inode 
 * param file 
 * 
 * returns on success, negative on failure 
 */

static int twl4030rtc_open(struct inode *inode, struct file *file)
{
	omap_twl4030rtc_lockout(file);
	
	down(&twl4030rtc_lock);
	if (rtc_status & TWL4030_IS_OPEN)
		goto out_busy;

	rtc_status |= TWL4030_IS_OPEN;

	rtc_irq_data = 0;
	up(&twl4030rtc_lock);
	return 0;

      out_busy:
	up(&twl4030rtc_lock);
	return -EBUSY;
}

/** 
 * Function : twl4030rtc_fasync
 * Description:
 * Updates asynchronous notifications to the application layer 
 * if the user program is waiting for IO operations 
 * (such as interrupt notifications).
 * 
 * param fd 
 * param filp 
 * param on 
 * 
 * return 
 */

static int twl4030rtc_fasync(int fd, struct file *file, int on)
{
	omap_twl4030rtc_lockout(file);
	return fasync_helper(fd, file, on, &rtc_async_queue);
}

/** 
 * Function : twl4030rtc_release
 * Description:
 * Disable all TWL4030 RTC module interrupts.
 * Sets status flag to free.
 * 
 * param inode 
 * param file 
 * 
 * returns on success, negative on failure 
 */

static int twl4030rtc_release(struct inode *inode, struct file *file)
{
	omap_twl4030rtc_lockout(file);

	if (file->f_flags & FASYNC) {
		twl4030rtc_fasync(-1, file, 0);
	}
	down(&twl4030rtc_lock);
	rtc_irq_data = 0;
	up(&twl4030rtc_lock);

	rtc_status &= ~TWL4030_IS_OPEN;
	return 0;
}

/** 
 * Function  : twl4030rtc_poll
 * Description : 
 * Adds the current process to given wait_queue 
 * to asynchronously signal some data ready condition.
 * 
 * param file 
 * param wait 
 * 
 * return 
 */

static unsigned int twl4030rtc_poll(struct file *file, poll_table * wait)
{
	unsigned long l;

	omap_twl4030rtc_lockout(file);

	poll_wait(file, &rtc_wait, wait);

	down(&twl4030rtc_lock);
	l = rtc_irq_data;
	up(&twl4030rtc_lock);

	if (l != 0)
		return POLLIN | POLLRDNORM;
	return 0;
}

/** 
 * Function  : omap_twl4030rtc_suspend
 * Description : 
 * Disables TWL4030 RTC timer and alarm interrupts.
 *
 * param dev 
 * param state 
 * param level 
 * 
 * returns on success, negative on failure 
 */

#ifdef CONFIG_PM
static int omap_twl4030rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	u8 val;
	int ret;

	if (rtc_suspended)
		return 0;
	/* lock-out applications during suspend */
	rtc_suspended = 1;

	if ((ret = twl4030rtc_read_u8(&val, REG_RTC_INTERRUPTS_REG)) < 0) {
		return ret;
	}

	/* save the mask */
	rtc_int_mask = val;  

	/* we don't want RTC interrupt during suspend */
	val &=
	    ~(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M |
	      BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	return twl4030rtc_write_u8(val, REG_RTC_INTERRUPTS_REG);
}

/** 
 * Function : omap_twl4030rtc_resume
 * Descripriton:
 * Enables TWL4030 RTC timer and alarm interrupts.
 * 
 * param dev 
 * param level 
 * 
 * returns on success, negative on failure 
 */

static int omap_twl4030rtc_resume(struct platform_device *pdev)
{
	int ret;

	if (!rtc_suspended)
		return 0;

	/* Restore RTC interrupt */
	ret = twl4030rtc_write_u8(rtc_int_mask, REG_RTC_INTERRUPTS_REG);
 	if (ret < 0)
		return ret;

	/* wake up applications waiting on suspend queue */
	rtc_suspended = 0;
	wake_up(&rtc_suspend_wq);
	return 0;
}
#else
#define omap_twl4030rtc_suspend NULL
#define omap_twl4030rtc_resume NULL
#endif

static struct file_operations twl4030rtc_fops = {
      llseek:no_llseek,
      read:twl4030rtc_read,
      poll:twl4030rtc_poll,
      ioctl:twl4030rtc_ioctl,
      open:twl4030rtc_open,
      release:twl4030rtc_release,
      fasync:twl4030rtc_fasync,
};

static struct miscdevice rtc_dev =
    { RTC_MINOR, "omap-twl4030rtc", &twl4030rtc_fops };

static int omap_twl4030rtc_probe(struct platform_device *pdev)
{
	int ret, err_val = 0;
	u8 RdReg;
        u16 tmp;
#ifdef CONFIG_ARCH_OMAP34XX
	u32 msecure_pad_config_reg = OMAP2_CTRL_BASE + 0xA3C;
        int mux_mask=0x04;
#else
       	u32 msecure_pad_config_reg = OMAP2_CTRL_BASE + 0x0132;
        int mux_mask=0x03;
#endif

	if (machine_is_omap_3430sdp() && is_sil_rev_greater_than(OMAP3430_REV_ES1_0)){
		/* 3430ES2.0 does not have msecure/GPIO-22 line connected to T2 */
		/* This line is used for USB mux now */
		/* Do nothing */
	} else {	

		if (omap_request_gpio(GPIO_RTC) != 0) {
			printk(KERN_ERR OMAP_TWL4030RTC_NAME
		       		"Could not reserve GPIO:%d !\n", GPIO_RTC);
			return 1;
		}

		/* TWL4030 will be in secure mode if msecure line from OMAP is low.
	 	* Make msecure line high in order to change the TWL4030 RTC time
	 	* and calender registers.
	 	*/
	
        	omap_set_gpio_direction(GPIO_RTC, 0);
	
 		/* Fix to be able to control correctly GPIO_22 */
		tmp = omap_readw(msecure_pad_config_reg);
		tmp &= 0xF8;		/* To enable mux mode 03/04 = GPIO_RTC */
		tmp |= mux_mask;		/* To enable mux mode 03/04 = GPIO_RTC */
		omap_writew(tmp, msecure_pad_config_reg);
       		omap_set_gpio_dataout(GPIO_RTC, 1);
	}
	
	ret = twl4030rtc_read_u8(&RdReg, REG_RTC_STATUS_REG);
	
	if (ret < 0) {
		err_val = ret;
		goto exit_path3;
	}
	
	if (RdReg & BIT_RTC_STATUS_REG_POWER_UP_M)
		printk(KERN_WARNING
			"%s:TWL4030-RTC Power up reset detected.\n",
			rtc_dev.name);
	
	if (RdReg & BIT_RTC_STATUS_REG_ALARM_M)
		printk(KERN_WARNING
			"%s: TWL4030-RTC Pending Alarm interrupt detected.\n",
			rtc_dev.name);

	/* Clear RTC Power up reset and pending alarm interrupts */
	ret = twl4030rtc_write_u8(RdReg, REG_RTC_STATUS_REG);
	if (ret < 0) {
		err_val = ret;
		goto exit_path3;
	}

	/*
	 * This ISR will always execute in kernel thread context because of
	 * the need to access the TWL4030 over the I2C bus.  The SA_NODELAY
	 * flag is passed to prevent the creation of a harmless but superfluous
	 * IRQ thread in case the CONFIG_PREEMPT_HARDIRQS kernel configuration
	 * option is enabled.
	 */
	ret = request_irq(TWL4030_MODIRQ_PWR, twl4030rtc_interrupt,
		IRQF_DISABLED | IRQF_SHARED, OMAP_TWL4030RTC_NAME, twl4030rtc_interrupt);
	if (ret < 0) {
		printk(KERN_ERR "%s:!!!!!....IRQ%d is not free.\n",
		       rtc_dev.name, TWL4030_MODIRQ_PWR);
		err_val = ret;
		goto exit_path3;
	}
	ret = misc_register(&rtc_dev);
	if (ret) {
		printk(KERN_ERR "....TWL4030-RTC registration failed!!!!\n");
		err_val = ret;
		goto exit_path4;
	}

#ifdef CONFIG_PM
	rtc_suspended = 0;
	init_waitqueue_head(&rtc_suspend_wq);
#endif
	/* Semaphore lock to prevent concurrent access */
	init_MUTEX(&twl4030rtc_lock);

	/* Check RTC module status, Enable if it is off */
	ret = twl4030rtc_read_u8(&RdReg, REG_RTC_CTRL_REG);
	if (ret < 0) {
		err_val = ret;
		goto exit_path5;
	}

	if (!(RdReg & BIT_RTC_CTRL_REG_STOP_RTC_M)) {
		printk(KERN_INFO "%s:Enabling TWL4030-RTC.\n",
		       rtc_dev.name);
		RdReg = BIT_RTC_CTRL_REG_STOP_RTC_M;
		ret = twl4030rtc_write_u8(RdReg, REG_RTC_CTRL_REG);
		if (ret < 0) {
			err_val = ret;
			goto exit_path5;
		}
	}

	/* To unmask RTC interrupt */
	ret = clear_bits_companion_reg(TWL4030_MODULE_INT, RTC_IT, R_PWR_IMR1);

	if (ret) {
		goto exit_path5;
	}

	/* To enable RTC rising edge detection */
	ret = set_bits_companion_reg(TWL4030_MODULE_INT, RTC_IT_RISING, R_PWR_EDR1);
	if (ret) {
		goto exit_path5;
	}	

	create_proc_read_entry("driver/twl4030-rtc", 0, 0, rtc_read_proc, NULL);
	return 0;

      exit_path5:
	misc_deregister(&rtc_dev);
      exit_path4:
	free_irq(TWL4030_MODIRQ_PWR, NULL);
      exit_path3:
	omap_free_gpio(GPIO_RTC);
	return err_val;
}

static int omap_twl4030rtc_remove(struct platform_device *pdev)
{
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);
	omap_free_gpio(GPIO_RTC);
	free_irq(TWL4030_MODIRQ_PWR, NULL);
	remove_proc_entry("driver/twl4030-rtc", NULL);
	misc_deregister(&rtc_dev);
	return 0;
}

/** 
 * Function : rtc_proc_output
 * Description : Info exported via "/proc/driver/rtc".
 *
 * param buf 
 * return 
 */

static int rtc_proc_output(char *buf)
{
#define YN(value) ((value) ? "yes" : "no")
#define NY(value) ((value) ? "no" : "yes")
	char *p;
	struct rtc_time tm;
	unsigned char alrmst, twl4030rtcst, updtrt;
	u8 RdReg;
	int ret;

	p = buf;

	get_rtc_time(&tm);

	/*
	 * There is no way to tell if the luser has the RTC set for local
	 * time or for Universal Standard Time (GMT). Probably local though.
	 */
	p += sprintf(p,
		     "rtc_time\t: %02d:%02d:%02d\n"
		     "rtc_date\t: %04d-%02d-%02d\n"
		     "rtc_epoch\t: %04d\n",
		     tm.tm_hour, tm.tm_min, tm.tm_sec,
		     tm.tm_year + epoch, tm.tm_mon + 1, tm.tm_mday, epoch);

	get_rtc_alm_time(&tm);

	/*
	 * We implicitly assume 24hr mode here. Alarm values >= 0xc0 will
	 * match any value for that particular field. Values that are
	 * greater than a valid time, but less than 0xc0 shouldn't appear.
	 */
	p += sprintf(p,
		     "alarm_time\t: %02d:%02d:%02d\n"
		     "alarm_date\t: %04d-%02d-%02d\n",
		     tm.tm_hour, tm.tm_min, tm.tm_sec,
		     tm.tm_year + epoch, tm.tm_mon + 1, tm.tm_mday);

	if ((ret = twl4030rtc_read_u8(&RdReg, REG_RTC_INTERRUPTS_REG)) < 0)
		return ret;

	alrmst = RdReg & BIT_RTC_INTERRUPTS_REG_IT_ALARM_M;
	twl4030rtcst = RdReg & BIT_RTC_INTERRUPTS_REG_IT_TIMER_M;
	updtrt = RdReg & BIT_RTC_INTERRUPTS_REG_EVERY_M;

	p += sprintf(p,
		     "BCD\t\t: %s\n"
		     "24hr\t\t: %s\n"
		     "alarm_IRQ\t: %s\n"
		     "update_IRQ\t: %s\n"
		     "update_rate\t: %u\n",
		     YN(1), YN(1), YN(alrmst), YN(twl4030rtcst), updtrt);
	return p - buf;
#undef YN
#undef NY
}

/** 
 * Function: rtc_read_proc
 * Description: Proc interface for exporting TWL4030 RTC information.
 * 
 * param page 
 * param *start 
 * param off 
 * param count 
 * param eof 
 * param data 
 * 
 * return len 
 */

static int rtc_read_proc(char *page, char **start, off_t off,
			 int count, int *eof, void *data)
{
	int len = rtc_proc_output(page);
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

/** 
 * Function: get_rtc_time
 * Description : Gets current TWL4030 RTC time and date parameters.
 * param alm_tm 
 * returns 0 on success, negavtive on failure.
 */

static int get_rtc_time(struct rtc_time *rtc_tm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;
	u8 save_control;

	down(&twl4030rtc_lock);

	if ((ret = twl4030rtc_read_u8(&save_control, REG_RTC_CTRL_REG)) < 0) {
		up(&twl4030rtc_lock);	
		return ret;
	}

	save_control |= BIT_RTC_CTRL_REG_GET_TIME_M;

	if ((ret = twl4030rtc_write_u8(save_control, REG_RTC_CTRL_REG)) < 0) {
		up(&twl4030rtc_lock);	
		return ret;
	}

	ret =
	    twl4030_i2c_read(TWL4030_MODULE_RTC, rtc_data,
			     REG_SECONDS_REG, ALL_TIME_REGS);
	up(&twl4030rtc_lock);

	if (ret < 0) {
		printk(KERN_ERR "twl4030_i2c_read error...!!!\n");
		return ret;
	}

	rtc_tm->tm_sec = BCD_TO_BIN(rtc_data[0]);
	rtc_tm->tm_min = BCD_TO_BIN(rtc_data[1]);
	rtc_tm->tm_hour = BCD_TO_BIN(rtc_data[2]);
	rtc_tm->tm_mday = BCD_TO_BIN(rtc_data[3]);
	rtc_tm->tm_mon = BCD_TO_BIN(rtc_data[4]);
	rtc_tm->tm_year = BCD_TO_BIN(rtc_data[5]);

	/*
	 * Account for differences between how the RTC uses the values
	 * and how they are defined in a struct rtc_time;
	 */

	if ((rtc_tm->tm_year += (epoch - 1900)) <= 69)
		rtc_tm->tm_year += 100;

	rtc_tm->tm_mon--;
	return ret;
}

/** 
 * Function: get_rtc_alm_time
 * Description : Gets current TWL4030 RTC alarm time.
 * param alm_tm 
 * returns 0 on success, negavtive on failure.
 */

static int get_rtc_alm_time(struct rtc_time *alm_tm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;

	down(&twl4030rtc_lock);

	ret =
	    twl4030_i2c_read(TWL4030_MODULE_RTC, rtc_data,
			     REG_ALARM_SECONDS_REG, ALL_TIME_REGS);
	up(&twl4030rtc_lock);

	if (ret < 0) {
		printk(KERN_ERR "twl4030_i2c_read error...!!!\n");
		return ret;
	}

	alm_tm->tm_sec = BCD_TO_BIN(rtc_data[0]);
	alm_tm->tm_min = BCD_TO_BIN(rtc_data[1]);
	alm_tm->tm_hour = BCD_TO_BIN(rtc_data[2]);
	alm_tm->tm_mday = BCD_TO_BIN(rtc_data[3]);
	alm_tm->tm_mon = BCD_TO_BIN(rtc_data[4]);
	alm_tm->tm_year = BCD_TO_BIN(rtc_data[5]);

	/*
	 * Account for differences between how the RTC uses the values
	 * and how they are defined in a struct rtc_time;
	 */
	if ((alm_tm->tm_year += (epoch - 1900)) <= 69)
		alm_tm->tm_year += 100;

	alm_tm->tm_mon--;
	return ret;
}

/** 
 * Function: mask_rtc_irq_bit
 * Description : Disables timer or alarm interrupts.
 *
 * param bit 
 * returns 0 on success, negavtive on failure.
 */

static int mask_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	down(&twl4030rtc_lock);

	if ((ret = twl4030rtc_read_u8(&val, REG_RTC_INTERRUPTS_REG)) < 0) {
		goto mask_irq_out;
	}
	val &= ~bit;
	if ((ret = twl4030rtc_write_u8(val, REG_RTC_INTERRUPTS_REG)) < 0) {
		goto mask_irq_out;
	}

	rtc_irq_data = 0;
      mask_irq_out:
	up(&twl4030rtc_lock);
	return ret;
}

/** 
 * Function: set_rtc_irq_bit
 * Description: Enables timer or alarm interrupts.
 * param bit 
 * returns 0 on success, negavtive on failure.
 */

static int set_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	down(&twl4030rtc_lock);

	if ((ret = twl4030rtc_read_u8(&val, REG_RTC_INTERRUPTS_REG)) < 0) {
		goto set_irq_out;
	}
	val |= bit;
	if ((ret = twl4030rtc_write_u8(val, REG_RTC_INTERRUPTS_REG)) < 0) {
		goto set_irq_out;
	}

	rtc_irq_data = 0;
      set_irq_out:
	up(&twl4030rtc_lock);
	return ret;
}

/* Driver Information */
static struct platform_driver omap_twl4030rtc_driver = {
	.probe = omap_twl4030rtc_probe,
	.remove	= omap_twl4030rtc_remove,
	.suspend = omap_twl4030rtc_suspend,
	.resume = omap_twl4030rtc_resume,
	.driver = {
		.name = "omap_twl4030rtc",
  },
};

/** 
 * Function : twl4030rtc_init
 * Description : 
 * Registers TWL4030 RTC driver with LDM.
 * Clears and masks RTC timer and alarm interrupts if any.
 * Checks RTC module status and starts RTC if it is not running.
 * Requests IRQ line for interrupt handler.
 * Registers TWL4030 RTC driver Linux misc subsystem.
 * Creates Proc Entry for TWL4030 RTC driver.
 * 
 * returns on success, negative on failure 
 */

static char __initdata banner[] = KERN_INFO "OMAP RTC Driver\n";

static int __init twl4030rtc_init(void)
{
	printk(banner);
	return platform_driver_register(&omap_twl4030rtc_driver);
}

/** 
 * Function : twl4030rtc_exit
 * Description : exit routine - releases all the resources.
 * 
 * return 
 */

static void __exit twl4030rtc_exit(void)
{
	/* Disable all the RTC interrupts */
	platform_driver_unregister(&omap_twl4030rtc_driver);


}

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(RTC_MINOR);

module_init(twl4030rtc_init);
module_exit(twl4030rtc_exit);
