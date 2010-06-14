/*
 * drivers/ssi/omap-hdq.c
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/hdq.h>

#define OMAP_HDQ_INTERRUPT_MODE
#undef OMAP_HDQ_DEBUG

#define MOD_NAME "OMAP_HDQ:"

#ifdef OMAP_HDQ_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ERR MOD_NAME format "\n", ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif


#ifdef CONFIG_ARCH_OMAP24XX
#define OMAP_HDQ_BASE_V		IO_ADDRESS(OMAP2_HDQ_BASE)
#define PRCM_BASE		(OMAP2_PRCM_BASE)
#define PRCM_REG32(offset)	__REG32(PRCM_BASE + (offset))
#define CM_ICLKEN1_CORE		PRCM_REG32(0x210)
#define CM_FCLKEN1_CORE		PRCM_REG32(0x200)
#elif CONFIG_ARCH_OMAP34XX
#define OMAP_HDQ_BASE_V		IO_ADDRESS(HDQ_BASE)
#define CM_REG32(offset)	__REG32(CM_BASE + (offset))
#define CM_ICLKEN1_CORE		CM_REG32(0xa10)
#define CM_FCLKEN1_CORE		CM_REG32(0xa00)
#else
#endif


#define HDQ_REVISION			0x00
#define HDQ_TX_DATA			0x04
#define HDQ_RX_DATA			0x08
#define HDQ_CTRL_STATUS			0x0c
#define HDQ_CTRL_STATUS_INTERRUPTMASK	(1<<6)
#define HDQ_CTRL_STATUS_CLOCKENABLE	(1<<5)
#define HDQ_CTRL_STATUS_GO		(1<<4)
#define HDQ_CTRL_STATUS_INITIALIZATION	(1<<2)
#define HDQ_CTRL_STATUS_DIR		(1<<1)
#define HDQ_CTRL_STATUS_MODE		(1<<0)
#define HDQ_INT_STATUS			0x10
#define HDQ_INT_STATUS_TXCOMPLETE	(1<<2)
#define HDQ_INT_STATUS_RXCOMPLETE	(1<<1)
#define HDQ_INT_STATUS_TIMEOUT		(1<<0)
#define HDQ_SYSCONFIG			0x14
#define HDQ_SYSCONFIG_SOFTRESET		(1<<1)
#define HDQ_SYSCONFIG_AUTOIDLE		(1<<0)
#define HDQ_SYSSTATUS			0x18
#define HDQ_SYSSTATUS_RESETDONE		(1<<0)

#define HDQ_CMD_READ			(0)
#define HDQ_CMD_WRITE			(1<<7)

#define HDQ_FLAG_CLEAR			0
#define HDQ_FLAG_SET			1
#define HDQ_TIMEOUT			(HZ/5)

#define OMAP_HDQ_MAX_USER		4

static struct semaphore hdq_semlock;
static int hdq_usecount;

static struct clk *hdq_ick, *hdq_fck;

#ifdef OMAP_HDQ_INTERRUPT_MODE
/* only needed for interrupt mode */
DECLARE_WAIT_QUEUE_HEAD(hdq_wait_queue);
spinlock_t hdq_spinlock;
static volatile u8 hdq_irqstatus;
#define HDQ_IRQFLAGS()		unsigned long irqflags
#define HDQ_SPINLOCK()   	spin_lock_irqsave(&hdq_spinlock, irqflags)
#define HDQ_SPINUNLOCK()  	spin_unlock_irqrestore(&hdq_spinlock, irqflags)
#define HDQ_CLEAR_IRQSTATUS()	(hdq_irqstatus = 0)
#define HDQ_MODE_STR		"interrupt"
#else
#define HDQ_IRQFLAGS()
#define HDQ_SPINLOCK()
#define HDQ_SPINUNLOCK()
#define HDQ_CLEAR_IRQSTATUS()
#define HDQ_MODE_STR		"poll I/O"
#endif

static int __init omap_hdq_probe(struct platform_device *pdev);
static int omap_hdq_remove(struct platform_device *pdev);

static struct platform_driver omap_hdq_driver = {
	.probe = omap_hdq_probe,
	.remove = omap_hdq_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "omap_hdq",
	},
};

/*
 * HDQ register I/O routines
 */
static __inline__ u8
hdq_reg_in(u32 offset)
{
	return readb(OMAP_HDQ_BASE_V + offset);
}

static __inline__ u8
hdq_reg_out(u32 offset, u8 val)
{
	writeb(val, OMAP_HDQ_BASE_V + offset);
	return val;
}

static __inline__ u8
hdq_reg_merge(u32 offset, u8 val, u8 mask)
{
	u8 new_val = (readb(OMAP_HDQ_BASE_V + offset) & ~mask) | (val & mask);
	writeb(new_val, OMAP_HDQ_BASE_V + offset);
	return new_val;
}

/*
 * Wait for one or more bits in flag change.
 * HDQ_FLAG_SET: wait until any bit in the flag is set.
 * HDQ_FLAG_CLEAR: wait until all bits in the flag are cleared.
 * return 0 on success and -ETIMEOUT in the case of timeout.
 */
static int
hdq_wait_for_flag(u32 offset, u8 flag, u8 flag_set, u8 * status)
{
	int ret = 0;
	unsigned long timeout = jiffies + HDQ_TIMEOUT;

	if (flag_set == HDQ_FLAG_CLEAR) {
		/* wait for the flag clear */
		while (((*status = hdq_reg_in(offset)) & flag)
		       && time_before(jiffies, timeout)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		}
		if (unlikely(*status & flag))
			ret = -ETIMEDOUT;
	} else if (flag_set == HDQ_FLAG_SET) {
		/* wait for the flag set */
		while (!((*status = hdq_reg_in(offset)) & flag)
		       && time_before(jiffies, timeout)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		}
		if (unlikely(!(*status & flag)))
			ret = -ETIMEDOUT;
	} else
		return -EINVAL;

	return ret;
}

/*
 * write out a byte and fill *status with HDQ_INT_STATUS
 */
static int
hdq_write_byte(u8 val, u8 * status)
{
	int ret;
	u8 tmp_status;
	HDQ_IRQFLAGS();

	*status = 0;

	HDQ_SPINLOCK();
	/* clear interrupt flags via a dummy read */
	hdq_reg_in(HDQ_INT_STATUS);
	/* ISR loads it with new INT_STATUS */
	HDQ_CLEAR_IRQSTATUS();
	HDQ_SPINUNLOCK();

	hdq_reg_out(HDQ_TX_DATA, val);

	/* set the GO bit */
	hdq_reg_merge(HDQ_CTRL_STATUS, HDQ_CTRL_STATUS_GO, 
		HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_GO);
	/* wait for the TXCOMPLETE bit */
#ifdef OMAP_HDQ_INTERRUPT_MODE
	ret = wait_event_interruptible_timeout(hdq_wait_queue,
					 hdq_irqstatus, HDQ_TIMEOUT);
	if (unlikely(ret < 0)) {
		DPRINTK("wait interrupted");
		return -EINTR;
	}

	HDQ_SPINLOCK();
	*status = hdq_irqstatus;
	HDQ_SPINUNLOCK();
	/* check irqstatus */
	if (!(*status & HDQ_INT_STATUS_TXCOMPLETE)) {
		DPRINTK("timeout waiting for TXCOMPLETE/RXCOMPLETE, %x",
			*status);
		return -ETIMEDOUT;
	}
#else
	ret = hdq_wait_for_flag(HDQ_INT_STATUS,
				HDQ_INT_STATUS_RXCOMPLETE |
				HDQ_INT_STATUS_TXCOMPLETE, HDQ_FLAG_SET,
				status);
	if (ret) {
		DPRINTK("timeout waiting TXCOMPLETE/RXCOMPLETE, %x", *status);
		return ret;
	}
#endif

	/* wait for the GO bit return to zero */
	ret = hdq_wait_for_flag(HDQ_CTRL_STATUS, HDQ_CTRL_STATUS_GO,
				HDQ_FLAG_CLEAR, &tmp_status);
	if (ret) {
		DPRINTK("timeout waiting GO bit return to zero, %x",
			tmp_status);
		return ret;
	}

	return ret;
}

#ifdef OMAP_HDQ_INTERRUPT_MODE
static irqreturn_t
hdq_isr(int irq, void *arg)
{
	HDQ_IRQFLAGS();
	u8 tmp_status;

	HDQ_SPINLOCK();
	tmp_status = hdq_reg_in(HDQ_INT_STATUS);
	hdq_irqstatus = tmp_status;
	HDQ_SPINUNLOCK();
	DPRINTK("hdq_isr: %x", tmp_status);

	if (tmp_status &
	    (HDQ_INT_STATUS_TXCOMPLETE | HDQ_INT_STATUS_TXCOMPLETE
	     | HDQ_INT_STATUS_TIMEOUT)) {
		/* wake up sleeping process */
		wake_up_interruptible(&hdq_wait_queue);
	}

	return IRQ_HANDLED;
}
#endif

static int
_omap_hdq_reset(void)
{
	int ret;
	u8 tmp_status;

	DPRINTK("_omap_hdq_reset");

	hdq_reg_out(HDQ_SYSCONFIG, HDQ_SYSCONFIG_SOFTRESET);
	/*
	 * Select HDQ mode & enable clocks.
	 * It is observed that INT flags can't be cleared via a read and GO/INIT
	 * won't return to zero if interrupt is disabled. So we always enable
	 * interrupt.
	 */
	hdq_reg_out(HDQ_CTRL_STATUS,
		    HDQ_CTRL_STATUS_CLOCKENABLE |
		    HDQ_CTRL_STATUS_INTERRUPTMASK);

	/* wait for reset to complete */
	if ((ret = hdq_wait_for_flag(HDQ_SYSSTATUS, HDQ_SYSSTATUS_RESETDONE,
				     HDQ_FLAG_SET, &tmp_status))) {
		DPRINTK("timeout waiting HDQ reset, %x", tmp_status);
	} else {
		hdq_reg_out(HDQ_CTRL_STATUS,
			    HDQ_CTRL_STATUS_CLOCKENABLE |
			    HDQ_CTRL_STATUS_INTERRUPTMASK);
		hdq_reg_out(HDQ_SYSCONFIG, HDQ_SYSCONFIG_AUTOIDLE);
	}

	return ret;
}

int
omap_hdq_reset(void)
{
	int ret;

	ret = down_interruptible(&hdq_semlock);
	if (ret < 0)
		return -EINTR;

	if (!hdq_usecount) {
		up(&hdq_semlock);
		return -EINVAL;
	}
	ret = _omap_hdq_reset();
	up(&hdq_semlock);
	return ret;
}
EXPORT_SYMBOL(omap_hdq_reset);

int
omap_hdq_break()
{
	int ret;
	u8 tmp_status;
	HDQ_IRQFLAGS();

	ret = down_interruptible(&hdq_semlock);
	if (ret < 0)
		return -EINTR;

	if (!hdq_usecount) {
		up(&hdq_semlock);
		return -EINVAL;
	}

	HDQ_SPINLOCK();
	/* clear interrupt flags via a dummy read */
	hdq_reg_in(HDQ_INT_STATUS);
	/* ISR loads it with new INT_STATUS */
	HDQ_CLEAR_IRQSTATUS();
	HDQ_SPINUNLOCK();

	/* set the INIT and GO bit */
	hdq_reg_merge(HDQ_CTRL_STATUS,
		      HDQ_CTRL_STATUS_INITIALIZATION | HDQ_CTRL_STATUS_GO,
		      HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_INITIALIZATION |
		      HDQ_CTRL_STATUS_GO);

	/* wait for the TIMEOUT bit */
#ifdef OMAP_HDQ_INTERRUPT_MODE
	ret = wait_event_interruptible_timeout(hdq_wait_queue, hdq_irqstatus,
					 HDQ_TIMEOUT);
	if (unlikely(ret < 0)) {
		DPRINTK("wait interrupted");
		up(&hdq_semlock);
		return -EINTR;
	}

	HDQ_SPINLOCK();
	tmp_status = hdq_irqstatus;
	HDQ_SPINUNLOCK();
	/* check irqstatus */
	if (!(tmp_status & HDQ_INT_STATUS_TIMEOUT)) {
		DPRINTK("timeout waiting for TIMEOUT, %x", tmp_status);
		up(&hdq_semlock);
		return -ETIMEDOUT;
	}
#endif
	/*
	 * wait for both INIT and GO bits rerurn to zero.
	 * zero wait time expected for interrupt mode.
	 */
	ret = hdq_wait_for_flag(HDQ_CTRL_STATUS,
				HDQ_CTRL_STATUS_INITIALIZATION |
				HDQ_CTRL_STATUS_GO, HDQ_FLAG_CLEAR,
				&tmp_status);
	if (ret)
		DPRINTK("timeout waiting INIT&GO bits return to zero, %x",
			tmp_status);

	up(&hdq_semlock);
	return ret;
}
EXPORT_SYMBOL(omap_hdq_break);

/*
 * Write a byte to a HDQ slave register
 */
int
omap_hdq_write(u8 reg, u8 val)
{
	int ret;
	u8 tmp_status;

	ret = down_interruptible(&hdq_semlock);
	if (ret < 0)
		return -EINTR;

	if (!hdq_usecount) {
		up(&hdq_semlock);
		return -EINVAL;
	}

	/* the first byte is the mix of reg offset and WRITE command */
	ret = hdq_write_byte(reg | HDQ_CMD_WRITE, &tmp_status);
	if (ret) {
		up(&hdq_semlock);
		return ret;
	}
	DPRINTK("INT_STATUS = %x after write comamnd sent", tmp_status);

	/* the second byte is the real data */
	ret = hdq_write_byte(val, &tmp_status);
	DPRINTK("INT_STATUS = %x after data sent", tmp_status);

	up(&hdq_semlock);
	return ret;
}
EXPORT_SYMBOL(omap_hdq_write);

/*
 * Read a byte from a HDQ slave register
 */
int
omap_hdq_read(u8 reg, u8 * val)
{
	int ret;
	u8 status;
	HDQ_IRQFLAGS();
	
	ret = down_interruptible(&hdq_semlock);
	if (ret < 0)
		return -EINTR;

	if (!hdq_usecount) {
		up(&hdq_semlock);
		return -EINVAL;
	}

	/* this byte is the mix of reg offset and READ command */
	ret = hdq_write_byte(reg | HDQ_CMD_READ, &status);
	if (ret) {
		up(&hdq_semlock);
		return ret;
	}
	DPRINTK("INT_STATUS = %x after read comamnd sent", status);

	if (!(status & HDQ_INT_STATUS_RXCOMPLETE)) {
		hdq_reg_merge(HDQ_CTRL_STATUS, 
			HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_GO,
			HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_GO);
#ifdef OMAP_HDQ_INTERRUPT_MODE
		/*
		 * The RX comes immediately after TX. It
	 	 * triggers another interrupt before we
	 	 * sleep. So we have to wait for RXCOMPLETE bit. 
		 */
		{
			unsigned long timeout = jiffies + HDQ_TIMEOUT;
			while (!(hdq_irqstatus & HDQ_INT_STATUS_RXCOMPLETE)
				&& time_before(jiffies, timeout)) {
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(1);
		}
		}
		hdq_reg_merge(HDQ_CTRL_STATUS, 0, 
			HDQ_CTRL_STATUS_DIR);
		HDQ_SPINLOCK();
		status = hdq_irqstatus;
		HDQ_SPINUNLOCK();
		/* check irqstatus */
		if (!(status & HDQ_INT_STATUS_RXCOMPLETE)) {
			DPRINTK("timeout waiting for RXCOMPLETE, %x",status);
			up(&hdq_semlock);
			return -ETIMEDOUT;
		}
#else
		ret = hdq_wait_for_flag(HDQ_INT_STATUS,
					HDQ_INT_STATUS_RXCOMPLETE, HDQ_FLAG_SET,
					&status);
		hdq_reg_merge(HDQ_CTRL_STATUS, 0, HDQ_CTRL_STATUS_DIR);
		if (ret) {
			DPRINTK("timeout waiting RXCOMPLETE, %x", status);
			up(&hdq_semlock);
			return ret;
		}
#endif
	}

	/* the data is ready. Read it in! */
	*val = hdq_reg_in(HDQ_RX_DATA);

	up(&hdq_semlock);
	return 0;
}
EXPORT_SYMBOL(omap_hdq_read);

int
omap_hdq_get()
{
	int ret = 0;

	ret = down_interruptible(&hdq_semlock);
	if (ret < 0)
		return -EINTR;

	if (OMAP_HDQ_MAX_USER == hdq_usecount) {
		DPRINTK("attempt to exceed the max use count");
		ret = -EINVAL;
	} else {
		hdq_usecount++;
		try_module_get(THIS_MODULE);
		if (1 == hdq_usecount) {
			if (clk_enable(hdq_ick)) {
				printk(KERN_ERR MOD_NAME 
					"Can not enable ick\n");
				return -ENODEV;
			}
			if (clk_enable(hdq_fck)) {
				printk(KERN_ERR MOD_NAME 
					"Can not enable fck\n");
				return -ENODEV;
			}
			DPRINTK("enable HDQ clocks");

			/* make sure HDQ is out of reset */
			if (!
			    (hdq_reg_in(HDQ_SYSSTATUS) &
			     HDQ_SYSSTATUS_RESETDONE)) {
				ret = _omap_hdq_reset();
				if (ret)
					/* back up the count */
					hdq_usecount--;
			} else {
				/* select HDQ mode & enable clocks */
				hdq_reg_out(HDQ_CTRL_STATUS,
					    HDQ_CTRL_STATUS_CLOCKENABLE |
					    HDQ_CTRL_STATUS_INTERRUPTMASK);
				hdq_reg_out(HDQ_SYSCONFIG,
					    HDQ_SYSCONFIG_AUTOIDLE);
				hdq_reg_in(HDQ_INT_STATUS);
			}
		}
	}
	up(&hdq_semlock);
	return ret;
}
EXPORT_SYMBOL(omap_hdq_get);

int
omap_hdq_put()
{
	int ret = 0;

	ret = down_interruptible(&hdq_semlock);
	if (ret < 0)
		return -EINTR;

	if (0 == hdq_usecount) {
		DPRINTK("attempt to decrement use count when it is zero");
		ret = -EINVAL;
	} else {
		hdq_usecount--;
		module_put(THIS_MODULE);
		if (0 == hdq_usecount) {
			hdq_reg_out(HDQ_CTRL_STATUS, hdq_reg_in(HDQ_CTRL_STATUS) & 
				~(HDQ_CTRL_STATUS_CLOCKENABLE));
			clk_disable(hdq_ick);
			clk_disable(hdq_fck);
			DPRINTK("shut down HDQ clocks");
		}
	}
	up(&hdq_semlock);
	return ret;
}
EXPORT_SYMBOL(omap_hdq_put);

void
omap_hdq_reg_dump()
{
	int iclk_enable = ((u32) CM_ICLKEN1_CORE) & (1 << 23);

	DPRINTK("Interface clock %s", iclk_enable ? "on" : "off");
	DPRINTK("Functional clock %s",
		((u32) CM_FCLKEN1_CORE) & (1 << 23) ? "on" : "off");
	if (iclk_enable) {
		DPRINTK("HDQ_CTRL_STATUS=%x", hdq_reg_in(HDQ_CTRL_STATUS));
		DPRINTK("HDQ_INT_STATUS=%x", hdq_reg_in(HDQ_INT_STATUS));
		DPRINTK("HDQ_SYSCONFIG=%x", hdq_reg_in(HDQ_SYSCONFIG));
		DPRINTK("HDQ_SYSSTATUS=%x", hdq_reg_in(HDQ_SYSSTATUS));
	}
}
EXPORT_SYMBOL(omap_hdq_reg_dump);

#define IS_HEX(c) ((c>='0'&&c<='9') || (c>='a'&&c<='f') || (c>='A'&&c<='F'))
#define HEX_TO_VAL(c) ((c>='0'&&c<='9')?(c-'0'):(c>='a'&&c<='f')?(c-'a'+0xa):(c-'A'+0xa))
#define VAL_TO_HEX(v) ((v>=0&&v<=9)?(v+'0'):(v+'A'-0xa))

#ifdef CONFIG_OMAP_HDQ_SYSFS
#define SET_CMD_OK		0
#define SET_CMD_ERR 		1
#define SET_CMD_INVAL 		2
#define SET_CMD_READ_OK 	0x100
const static char *result_string[] = { "OK", "Err", "Inval" };
static int result_code = SET_CMD_INVAL;
static int hdq_sysfs_open;
static struct semaphore hdq_sysfs_semlock;

static int
_convert_to_a_byte(const char *buf, int *i, u8 * val, int count)
{
	if (((*i) + 2) >= count)
		return 0;
	if (IS_HEX(buf[*i]) && IS_HEX(buf[*i + 1])) {
		*val = (HEX_TO_VAL(buf[*i]) << 4) + HEX_TO_VAL(buf[*i + 1]);
		*i += 3;
	} else
		return 0;
	return 1;
}

static ssize_t
set_cmd(struct device *dev, struct device_attribute *dev_attr, const char *buf,
	size_t count)
{
	char cmd[8];
	int ret, i = 0;
	u8 reg, val;

	ret = down_interruptible(&hdq_sysfs_semlock);
	if (ret < 0)
		return -EINTR;

	ret = SET_CMD_INVAL;
	if (count < 5)
		goto set_exit;

	while (buf[i] != ' ' && buf[i] != '\n' && i < count) {
		cmd[i] = buf[i];
		i++;
	}
	cmd[i] = '\0';
	i++;

	DPRINTK("sysfs cmd: *%s*, len=%d, count=%d", cmd, strlen(cmd), count);
	ret = SET_CMD_OK;
	if (!strcmp(cmd, "open")) {
		DPRINTK("process open cmd ...");
		if (!hdq_sysfs_open) {
			if (omap_hdq_get())
				ret = SET_CMD_ERR;
			else
				hdq_sysfs_open = 1;
		}
	} else if (!strcmp(cmd, "close")) {
		DPRINTK("process close cmd ...");
		if (hdq_sysfs_open) {
			if (omap_hdq_put())
				ret = SET_CMD_ERR;
			else
				hdq_sysfs_open = 0;
		}
	} else if (!strcmp(cmd, "reset") && hdq_sysfs_open) {
		DPRINTK("process reset cmd ...");
		if (omap_hdq_reset())
			ret = SET_CMD_ERR;
	} else if (!strcmp(cmd, "break") && hdq_sysfs_open) {
		DPRINTK("process break cmd ...");
		if (omap_hdq_break())
			ret = SET_CMD_ERR;
	} else if (!strcmp(cmd, "read") && hdq_sysfs_open) {
		DPRINTK("process read cmd ...");
		ret = SET_CMD_INVAL;
		if (_convert_to_a_byte(buf, &i, &reg, count)) {
			if (reg < 0x80) {
				DPRINTK("read from reg %x ...", reg);
				if (!omap_hdq_read(reg, &val))
					ret = SET_CMD_READ_OK | val;
				else
					ret = SET_CMD_ERR;
			}
		}
	} else if (!strcmp(cmd, "write") && hdq_sysfs_open) {
		DPRINTK("process write cmd ...");
		ret = SET_CMD_INVAL;
		if (_convert_to_a_byte(buf, &i, &reg, count) &&
		    _convert_to_a_byte(buf, &i, &val, count)) {
			if (reg < 0x80) {
				DPRINTK("write %x to reg %x ...", val, reg);
				if (!omap_hdq_write(reg, val))
					ret = SET_CMD_OK;
				else
					ret = SET_CMD_ERR;
			}
		}
	} else
		ret = SET_CMD_INVAL;

	set_exit:
		DPRINTK("exiting cmd_set, %x", ret);
		result_code = ret;
		up(&hdq_sysfs_semlock);
		return count;
}
static ssize_t
show_result(struct device *dev, struct device_attribute *dev_attr, char *buf)
{
	int count, ret;
	u8 val;

	ret = down_interruptible(&hdq_sysfs_semlock);
	if (ret < 0)
		return -EINTR;

	if (result_code & SET_CMD_READ_OK) {
		val = result_code & ~SET_CMD_READ_OK;
		buf[0] = VAL_TO_HEX(((val & 0xf0) >> 4));
		buf[1] = VAL_TO_HEX((val & 0x0f));
		buf[2] = '\0';
		count = 3;
	} else {
		strcpy(buf, result_string[result_code]);
		count = strlen(result_string[result_code]) + 1;
	}
	DPRINTK("show_result, %s", buf);
	up(&hdq_sysfs_semlock);
	return count;
}

static DEVICE_ATTR(cmd, S_IWUSR, NULL, set_cmd);
static DEVICE_ATTR(result, S_IRUGO, show_result, NULL);
#endif		/* CONFIG_OMAP_HDQ_SYSFS  */

static int __init omap_hdq_probe(struct platform_device *pdev)
{
	int ret;
	u8 rev;

	/* get interface & functional clock objects */
	hdq_ick = clk_get(&pdev->dev, "hdq_ick");
	hdq_fck = clk_get(&pdev->dev, "hdq_fck");

	if (IS_ERR(hdq_ick) || IS_ERR(hdq_fck)) {
		printk(KERN_ERR MOD_NAME "Can't get HDQ clock objects\n");
		if (IS_ERR(hdq_ick)){
			ret = PTR_ERR(hdq_ick);
			return ret;
		}
		if (IS_ERR(hdq_fck)){
			ret = PTR_ERR(hdq_fck);
			return ret;
		}
	}
	
	hdq_usecount = 0;
	sema_init(&hdq_semlock, 1);

	if (clk_enable(hdq_ick)) {
		printk(KERN_ERR MOD_NAME "Can not enable ick\n");
		return -ENODEV;
	}

	rev = hdq_reg_in(HDQ_REVISION);
	printk(KERN_INFO
		"OMAP HDQ Hardware Revision %c.%c. Driver in %s mode.\n",
		(rev >> 4) + '0', (rev & 0x0f) + '0', HDQ_MODE_STR);

#ifdef OMAP_HDQ_INTERRUPT_MODE
	if (request_irq(INT_24XX_HDQ_IRQ, hdq_isr, 0, "OMAP HDQ", &hdq_semlock)) {
		printk(KERN_ERR MOD_NAME "request_irq failed\n");
		clk_disable(hdq_ick);
		clk_put(hdq_ick);
		clk_put(hdq_fck);
		return -ENODEV;
	}
	spin_lock_init(&hdq_spinlock);
#endif
	/* don't clock the HDQ until it is needed */
	clk_disable(hdq_ick);

#ifdef CONFIG_OMAP_HDQ_SYSFS
	if (device_create_file(&pdev->dev, &dev_attr_cmd) < 0)
		printk(KERN_ERR "Creating sysfs entry"
				"for omap hdq driver failed\n");
	if (device_create_file(&pdev->dev, &dev_attr_result) < 0)
		printk(KERN_ERR "Creating sysfs entry"
				"for omap hdq driver failed\n");
	sema_init(&hdq_sysfs_semlock, 1);
#endif

	return 0;
}

static int omap_hdq_remove(struct platform_device *pdev)
{
	if (0 != hdq_usecount) {
		printk(KERN_WARNING MOD_NAME
		       "removed when use count is not zero\n");
		/* force clocks shut down */
		hdq_usecount = 1;
		omap_hdq_put();
	}
	/* remove module dependency */
	clk_put(hdq_ick);
	clk_put(hdq_fck);

#ifdef OMAP_HDQ_INTERRUPT_MODE
	free_irq(INT_24XX_HDQ_IRQ, &hdq_semlock);
#endif

#ifdef CONFIG_OMAP_HDQ_SYSFS
	device_remove_file(&pdev->dev, &dev_attr_cmd);
	device_remove_file(&pdev->dev, &dev_attr_result);
#endif

	return 0;
}

static int __init
omap_hdq_init(void)
{
	if (platform_driver_register(&omap_hdq_driver)) {
		printk(KERN_ERR ":failed to register HDQ driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit
omap_hdq_exit(void)
{
	platform_driver_unregister(&omap_hdq_driver);
	return;
}

module_init(omap_hdq_init);
module_exit(omap_hdq_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("HDQ driver Library");
MODULE_LICENSE("GPL");
