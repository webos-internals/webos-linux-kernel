/*
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/modem_activity.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#include <linux/usb.h>

#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pm.h>

#if 1
#define PDBG(args...)   if( g_log ) printk(args)
#else
#define PDBG(args...)
#endif

#define DEV_NAME  "mdm_act"

#define MAX_UART_PORTS	4

#define WKUP_TIMEOUT    10000
#define WKUP_RETRY      1000

// AWM states
#define AWM_STATE_ASLEEP       0 
#define AWM_STATE_WKUP_WAIT    1
#define AWM_STATE_AWAKE_WAIT   2
#define AWM_STATE_AWAKE        3
#define AWM_STATE_SLEEP_WAIT   4
#define AWM_STATE_ASLEEP_WAIT  5

struct modem_act_ops {
	int (*busy)  (unsigned long);
	int (*wkup)  (unsigned long);
	int (*awake) (unsigned long);
	int (*sleep) (unsigned long);
	int (*asleep)(unsigned long);
	unsigned long param;
};

/* Driver per instance local data. */
struct modem_act_ctxt {
	int	initialized;
	int	enabled;
	u32	timeout;
	int	awake_state;
	int	awake_pending;
	int	awake_usb_pending;
	int	awake_acked;
	int	warn_cnt;
	const char *awake_source;
	int	app_wake_modem;
	int	modem_wake_app;
	int	modem_wake_usb;
	int	gpio_flags;          
	u32	uart_port;
	spinlock_t lock;
	struct mutex mlock;
	struct timer_list  act_timer;
	struct timer_list  wkup_timer;
	struct workqueue_struct *workq;
	struct workqueue_struct *workq_rmw;
	struct work_struct  awake_work;
	struct work_struct  awake_usb_work;
	struct completion   awake_comp;
	struct modem_act_ops uarts[MAX_UART_PORTS];
	struct usb_device *usb_dev;
	int	usb_suspended;
};

/* Currently we only support one instance */
static struct modem_act_ctxt state = {0};
static struct modem_act_ctxt *g_ctxt = &state;
static int    g_log = 0;

/* 
 *  Notify registered ports about modem pre_wake
 */
static void
notify_modem_wkup(struct modem_act_ctxt *ctxt)
{
	struct modem_act_ops *ops;
	
	if(!ctxt) 
		return;

	if( ctxt->uart_port >= 0 &&  
	    ctxt->uart_port < ARRAY_SIZE(ctxt->uarts)) {
		ops = ctxt->uarts + ctxt->uart_port;
		if( ops->wkup ) {
			ops->wkup ( ops->param );
		}
	}
}


/* 
 *  Notify registered ports about modem awake
 */
static void
notify_modem_awake(struct modem_act_ctxt *ctxt)
{
	struct modem_act_ops *ops;
	
	if(!ctxt) 
		return;

	if( ctxt->uart_port >= 0 &&  
	    ctxt->uart_port < ARRAY_SIZE(ctxt->uarts)) {
		ops = ctxt->uarts + ctxt->uart_port;
		if( ops->awake ) {
			ops->awake ( ops->param );
		}
	}
}

/* 
 *  Notify registered ports that modem is about to sleep
 */
static void
notify_modem_sleep(struct modem_act_ctxt *ctxt)
{
	struct modem_act_ops *ops;
	
	if(!ctxt) 
		return;

	if( ctxt->uart_port >= 0 &&  
	    ctxt->uart_port < ARRAY_SIZE(ctxt->uarts)) {
		ops = ctxt->uarts + ctxt->uart_port;
		if( ops->sleep ) {
			ops->sleep ( ops->param );
		}
	}
}

/* 
 *  Notify registered ports that modem confirmed sleep
 */
static void
notify_modem_asleep(struct modem_act_ctxt *ctxt)
{
	struct modem_act_ops *ops;
	
	if(!ctxt) 
		return;

	if( ctxt->uart_port >= 0 &&  
	    ctxt->uart_port < ARRAY_SIZE(ctxt->uarts)) {
		ops = ctxt->uarts + ctxt->uart_port;
		if( ops->asleep ) {
			ops->asleep ( ops->param );
		}
	}
}

/*
 * 
 */
static void 
modem_act_work(struct work_struct *work)
{
	unsigned long flags;
	struct modem_act_ctxt *ctxt = 
	       container_of(work, struct modem_act_ctxt, awake_work );

	mutex_lock(&ctxt->mlock);
Again:
	if( ctxt->awake_state == AWM_STATE_ASLEEP) {
		if(!ctxt->awake_pending) {
			goto Exit;
		}
		msleep(5);
		ctxt->awake_state = AWM_STATE_WKUP_WAIT;
		// fall through
	}
	
	if( ctxt->awake_state == AWM_STATE_WKUP_WAIT ) {
		// trigger modem wakeup
		PDBG("%s: %10lu: wakeup modem (%s)\n", 
		      DEV_NAME, jiffies, ctxt->awake_source );
		
		// assert modem wakeup gpio
		spin_lock_irqsave(&ctxt->lock, flags);
		
		ctxt->awake_state = AWM_STATE_AWAKE_WAIT;
		ctxt->awake_acked = 0;

		PDBG("%s: %10lu: assert AWM\n", DEV_NAME, jiffies);
		gpio_set_value(ctxt->app_wake_modem, 1);

		mod_timer(&ctxt->wkup_timer, 
			   jiffies + msecs_to_jiffies(WKUP_RETRY));

		spin_unlock_irqrestore(&ctxt->lock, flags);

		if(!ctxt->enabled ) // assume awake
			goto Awake; 

		// fall through
	}

	if( ctxt->awake_state == AWM_STATE_AWAKE_WAIT ) {

		if (!ctxt->awake_acked)
			goto Exit;

Awake:
		// cancel timer reset warning count.
		del_timer(&ctxt->wkup_timer);
		ctxt->warn_cnt = 10;

		PDBG("%s: %10lu: modem is awake\n", DEV_NAME, jiffies );
	
		// notify pre_wake state
		notify_modem_wkup(ctxt);

		spin_lock_irqsave(&ctxt->lock, flags);
		ctxt->awake_state   = AWM_STATE_AWAKE;
		ctxt->awake_pending = 0;
		
		// start modem inactivity timer
		if( ctxt->enabled && ctxt->usb_suspended ) {
			mod_timer (&ctxt->act_timer, jiffies + ctxt->timeout);
		}

		spin_unlock_irqrestore(&ctxt->lock, flags);
		
		// notify awake state
		notify_modem_awake(ctxt);

		// release all waiting on completion
		complete_all(&ctxt->awake_comp);

		goto Exit;
	}
	
	if( ctxt->awake_state == AWM_STATE_SLEEP_WAIT ) {
		PDBG("%s: %10lu: allow modem sleep\n", DEV_NAME, jiffies );

		ctxt->awake_state = AWM_STATE_ASLEEP_WAIT;
		
		// notify client what we are about to allow sleep
		notify_modem_sleep( ctxt );
		
		// De-Assert APP_WAKE_MODEM line
		PDBG("%s: %10lu: deassert AWM\n", DEV_NAME, jiffies);
		gpio_set_value ( ctxt->app_wake_modem, 0 );
		
		// at this point just fall through
	}
	
	if( ctxt->awake_state == AWM_STATE_ASLEEP_WAIT ) {
		PDBG("%s: %10lu: assume modem is asleep\n", DEV_NAME, jiffies);
		ctxt->awake_state = AWM_STATE_ASLEEP;
		
		// notify client what modem is asleep
		notify_modem_asleep( ctxt );
		
		goto Again;
	}
Exit:
	mutex_unlock(&ctxt->mlock);
}

/*
 *   Called when modem inactivity timer has expired
 *
 */
static void 
modem_act_timer(unsigned long data)
{
	int busy = 0;
	unsigned long flags;
	struct modem_act_ops  *ops;
	struct modem_act_ctxt *ctxt = (struct modem_act_ctxt *)data;
 
	spin_lock_irqsave(&ctxt->lock, flags);
	// for configured uart port check if it is busy
	if( ctxt->uart_port >= 0 &&  
	    ctxt->uart_port < ARRAY_SIZE(ctxt->uarts)) {
		ops = ctxt->uarts + ctxt->uart_port;
		if( ops->busy ) {
			busy |= ops->busy ( ops->param );
		}
	}

	if( busy ) {
		mod_timer(&ctxt->act_timer, jiffies + ctxt->timeout);
		PDBG("%s: %10lu: port(s) busy\n", DEV_NAME, jiffies );
	} else {
		// this is only possible in awake state
		BUG_ON(ctxt->awake_state != AWM_STATE_AWAKE);
		ctxt->awake_state = AWM_STATE_SLEEP_WAIT;
		queue_work (ctxt->workq, &ctxt->awake_work);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
}


static void 
modem_wkup_timer(unsigned long data)
{
	unsigned long flags;
	struct modem_act_ctxt *ctxt = (struct modem_act_ctxt *)data;

	if( ctxt->warn_cnt ) {
		ctxt->warn_cnt--;
		printk( KERN_WARNING "%s: %10lu: modem wakeup timeout\n",
	                      DEV_NAME, jiffies );
	}

	spin_lock_irqsave(&ctxt->lock, flags);
	PDBG("%s: %10lu: deassert AWM\n", DEV_NAME, jiffies);
	gpio_set_value(ctxt->app_wake_modem, 0);
	ctxt->awake_state   = AWM_STATE_WKUP_WAIT;
	ctxt->awake_pending = 1;
	queue_work(ctxt->workq, &ctxt->awake_work);
	spin_unlock_irqrestore(&ctxt->lock, flags);
}


/*
 *  app_wake_modem:
 *
 *       Wakes up modem and and optionally wait up to specified timeout
 *       until modem responds.
 *       
 *       return:  0            the modem is awake 
 *               -EAGAIN       the modem is busy or not awake
 *               -ERESTARTSYS  signal received
 */
static int
modem_act_app_wake_modem (struct modem_act_ctxt *ctxt, 
			  const char *source, int timeout, int interruptible)
{
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&ctxt->lock, flags);

	del_timer(&ctxt->act_timer); // Cancel inactivity timer

	if( ctxt->awake_state == AWM_STATE_AWAKE ) {
//		PDBG("%s: %10lu: already awake (%s)\n", DEV_NAME, jiffies, source );
		if( ctxt->enabled && ctxt->usb_suspended ) {
			ctxt->act_timer.expires = jiffies + ctxt->timeout;
			add_timer(&ctxt->act_timer);
		}
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return 0;
	}
	if(!ctxt->awake_pending) {
		ctxt->awake_pending = 1;
		ctxt->awake_source  = source;
		init_completion(&ctxt->awake_comp);
		queue_work (ctxt->workq, &ctxt->awake_work);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);
	
	if( timeout == 0 )
		return -EAGAIN;  // no wait

	timeout = WKUP_TIMEOUT;
	
	// Wait for modem to respond
	PDBG("%s: %10lu: wait for wakeup (%s)(%d)\n", 
	      DEV_NAME, jiffies, source, timeout);
	      
	if (interruptible)
		rc = wait_for_completion_interruptible_timeout(&ctxt->awake_comp, 
							       msecs_to_jiffies(timeout));
	else {
		timeout = 500;
		rc = wait_for_completion_timeout(&ctxt->awake_comp, 
						 msecs_to_jiffies(timeout));
	}
	if( rc == 0 ) {
		// DOLATER: Toggle awm line one more time and try again
		//          Handle modem dead case
		printk( KERN_ERR "Timeout waiting for modem awake\n");
		return -EAGAIN;
	}
	if( rc < 0 ) {
		return  rc;
	}

	PDBG("%s: %10lu: modem is awake (%s)\n", DEV_NAME, jiffies, source );
	return 0; // modem is awake
}

/*
 *   Called by UART driver to indicate activity on uart port
 *   It is assumed that hardware handshaking is enabled so we do not 
 *   really care if modem is awake or not. 
 */
int 
modem_activity_touch_uart_port( u32 port, int msec, const char *source  )
{
	/* Sanity checks. */
	if(!g_ctxt->initialized)
		return -ENODEV;

	if( g_ctxt->uart_port != port )
		return -EINVAL;

	return modem_act_app_wake_modem (g_ctxt, "uart", msec, 1 );
}
EXPORT_SYMBOL(modem_activity_touch_uart_port);

/*
 *   Called by UART driver to register callbacks
 */
void 
modem_activity_set_uart_handler ( u32 port,
                                  int (*busy)(unsigned long param),
                                  int (*sleep) (unsigned long),
                                  int (*asleep)(unsigned long),
                                  int (*wkup)  (unsigned long),
                                  int (*awake) (unsigned long),
                                  unsigned long param )
{
	if( port >= ARRAY_SIZE(g_ctxt->uarts))
		return; // ignore
	
	PDBG("%s: %s handler for UART%d\n", 
	      DEV_NAME, busy ? "register" : "unregister", port );
	
	// set up busy handler
	g_ctxt->uarts[port].busy   = busy;
	g_ctxt->uarts[port].sleep  = sleep;
	g_ctxt->uarts[port].asleep = asleep;
	g_ctxt->uarts[port].wkup   = wkup;
	g_ctxt->uarts[port].awake  = awake;
	g_ctxt->uarts[port].param  = param;
}
EXPORT_SYMBOL(modem_activity_set_uart_handler);

/*
 *   Called by USB subsystem to register callbacks
 */
void
modem_activity_set_usb_dev(unsigned long udev)
{
	g_ctxt->usb_dev = (struct usb_device *)udev;

	PDBG("%s: set usb_dev=%p\n", DEV_NAME, g_ctxt->usb_dev);
}
EXPORT_SYMBOL(modem_activity_set_usb_dev);

static void
mw_handle_irq(struct modem_act_ctxt *ctxt, const char *source)
{
	unsigned long flags;

	del_timer(&ctxt->wkup_timer);

	if( !g_ctxt->enabled )
		return;

	spin_lock_irqsave(&ctxt->lock, flags);
	if( ctxt->awake_state == AWM_STATE_AWAKE_WAIT) {
		ctxt->awake_acked = 1;
		queue_work (ctxt->workq, &ctxt->awake_work);
	}
	else if( ctxt->awake_state == AWM_STATE_ASLEEP || 
	         ctxt->awake_state == AWM_STATE_SLEEP_WAIT ||
	         ctxt->awake_state == AWM_STATE_ASLEEP_WAIT ) {
		if(!ctxt->awake_pending) {
			ctxt->awake_pending = 1;
			ctxt->awake_source  = source;
			queue_work (ctxt->workq, &ctxt->awake_work);
		}
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);

}


static irqreturn_t 
mwa_interrupt(int irq, void *dev_id)
{
	struct modem_act_ctxt *ctxt = dev_id;
	PDBG("%s: %10lu:\n", __FUNCTION__, jiffies );
	mw_handle_irq(ctxt, "mwa");
	return IRQ_HANDLED;
}

static void
modem_act_usb_work(struct work_struct *work)
{
	struct modem_act_ctxt *ctxt = 
	       container_of(work, struct modem_act_ctxt, awake_usb_work);
	unsigned long flags;
	struct usb_device *udev;
	int usb_external_resume_device(struct usb_device *udev);
	int status = 0;

retry:
	spin_lock_irqsave(&ctxt->lock, flags);
	if (!ctxt->awake_usb_pending || !ctxt->usb_dev) {
		ctxt->awake_usb_pending = 0;
		spin_unlock_irqrestore(&ctxt->lock, flags);
		return;
	}
	/*
	 * acm_disconnct() may reset ctxt->usb_dev to NULL.
	 * copy it to a local variable.
	 */
	udev = usb_get_dev(ctxt->usb_dev); /* increment the refcount */
	spin_unlock_irqrestore(&ctxt->lock, flags);

	/* we avoid a potential deadlock */
	if (usb_trylock_device(udev) != 0) {
		printk(KERN_INFO "%s: usb remote wakeup: retry locking...\n", DEV_NAME );
		usb_put_dev(udev); /* decrement the refcount */
		/* just sleep & retry since we have a dedicated workqueue thread */
		msleep(15);
		goto retry;
	}

	if (udev->state == USB_STATE_SUSPENDED) {
		PDBG("%s: %10lu: usb remote wakeup start\n", DEV_NAME, jiffies);
		usb_mark_last_busy(udev);
		status = usb_external_resume_device(udev);
		if (status < 0) {
			printk(KERN_ERR "%s: usb remote wakeup failed (rc=%d)\n", DEV_NAME, status);
		} else {
			PDBG("%s: %10lu: usb remote wakeup done\n", DEV_NAME, jiffies);
		}
	}
	usb_unlock_device(udev);
	usb_put_dev(udev); /* decrement the refcount */

	spin_lock_irqsave(&ctxt->lock, flags);
	ctxt->awake_usb_pending = 0;
	spin_unlock_irqrestore(&ctxt->lock, flags);
}

static irqreturn_t 
mwau_interrupt(int irq, void *dev_id)
{
	struct modem_act_ctxt *ctxt = dev_id;
	unsigned long flags;

	PDBG("%s: %10lu:\n", __FUNCTION__, jiffies );
	mw_handle_irq(ctxt, "mwau");

	spin_lock_irqsave(&ctxt->lock, flags);
	if (!ctxt->awake_usb_pending) {
		ctxt->awake_usb_pending = 1;
		queue_work (ctxt->workq_rmw, &ctxt->awake_usb_work);
	}
	spin_unlock_irqrestore(&ctxt->lock, flags);

	return IRQ_HANDLED;
}


int
modem_activity_usb_suspend (void)
{
	PDBG("usb_suspend: %10lu:\n", jiffies);

	/* Sanity checks. */
	if(!g_ctxt->initialized) {
		printk( KERN_INFO "%s: not initialized\n", DEV_NAME);
		return -ENODEV;
	}

	mutex_lock(&g_ctxt->mlock);
	g_ctxt->usb_suspended = 1;
	if (g_ctxt->awake_state == AWM_STATE_AWAKE) {
		if (g_ctxt->enabled) {
			mod_timer (&g_ctxt->act_timer, jiffies + 1);
		}
	}
	mutex_unlock(&g_ctxt->mlock);

	return 0;
}
EXPORT_SYMBOL(modem_activity_usb_suspend);

int
modem_activity_usb_resume (int msec)
{
	unsigned long flags;

	PDBG("usb_resume: %10lu:\n", jiffies);

	/* Sanity checks. */
	if(!g_ctxt->initialized) {
		printk( KERN_INFO "%s: not initialized\n", DEV_NAME);
		return -ENODEV;
	}

	mutex_lock(&g_ctxt->mlock);
	spin_lock_irqsave(&g_ctxt->lock, flags);
	g_ctxt->usb_suspended = 0;
	if (g_ctxt->awake_state == AWM_STATE_AWAKE) {
		del_timer(&g_ctxt->act_timer);
	}
	spin_unlock_irqrestore(&g_ctxt->lock, flags);
	mutex_unlock(&g_ctxt->mlock);

	return modem_act_app_wake_modem(g_ctxt, "usb", msec, 0);
}
EXPORT_SYMBOL(modem_activity_usb_resume);


static void
modem_act_enable (struct modem_act_ctxt *ctxt)
{
	printk( KERN_INFO "%s: enable\n", DEV_NAME );
	
	mutex_lock(&ctxt->mlock);
	if(!ctxt->enabled) {
		ctxt->enabled = 1;
		if (ctxt->awake_state == AWM_STATE_AWAKE) {
			if (ctxt->usb_suspended) {
				mod_timer (&ctxt->act_timer,
					   jiffies + ctxt->timeout);
			}
		}
	}
	mutex_unlock(&ctxt->mlock);
}


static void
modem_act_disable (struct modem_act_ctxt *ctxt)
{
	unsigned long flags;
	int rc;
	int timeout;
	
	printk( KERN_INFO "%s: disable\n", DEV_NAME );

	mutex_lock(&ctxt->mlock);
	if( !ctxt->enabled ) {
		/* already disabled */
		mutex_unlock(&ctxt->mlock);
		return;
	}

	spin_lock_irqsave(&ctxt->lock, flags);
	ctxt->enabled = 0;

	if (ctxt->awake_state == AWM_STATE_AWAKE) {
		/* modem is awake */
		/* stop the inactivity timer */
		del_timer(&ctxt->act_timer);
		spin_unlock_irqrestore(&ctxt->lock, flags);
		mutex_unlock(&ctxt->mlock);
		return;
	}

	/* modem is not awake */

	if (ctxt->awake_state == AWM_STATE_AWAKE_WAIT) {
		ctxt->awake_acked = 1;
		queue_work (ctxt->workq, &ctxt->awake_work);
	}
	else {
		if(!ctxt->awake_pending) {
			ctxt->awake_pending = 1;
			ctxt->awake_source  = "modem_act_disable";
			init_completion(&ctxt->awake_comp);
			queue_work (ctxt->workq, &ctxt->awake_work);
		}
	}

	spin_unlock_irqrestore(&ctxt->lock, flags);
	mutex_unlock(&ctxt->mlock);

	/* wait until the modem is awake */

	timeout = WKUP_TIMEOUT;

	PDBG("%s: %10lu: wait for wakeup (%s)(%d)\n",
	      DEV_NAME, jiffies, ctxt->awake_source, timeout);

	rc = wait_for_completion_interruptible_timeout(&ctxt->awake_comp,
						   msecs_to_jiffies(timeout));
	if (rc == 0) {
		printk( KERN_ERR "Timeout waiting for modem awake\n");
	}
}

int
modem_activity_handshake_is_enabled(void)
{
	return g_ctxt->enabled;
}
EXPORT_SYMBOL(modem_activity_handshake_is_enabled);

static ssize_t 
modem_act_attr_enabled_show(struct device *dev, 
                            struct device_attribute *attr, char *buf)
{
	if( g_ctxt->enabled )
		return snprintf(buf, PAGE_SIZE, "enabled\n");
	else
		return snprintf(buf, PAGE_SIZE, "disabled\n");
}


static ssize_t 
modem_act_attr_enabled_store(struct device *dev, 
                             struct device_attribute *attr, 
                             const char *buf, size_t count)
{
	int enable = -1;
	
	if(!count)
		return count;

	if( count >=7 && !strncmp(buf, "disable", 7)) {
		enable = 0;
	} else 	if( count >=6 && !strncmp(buf, "enable", 6)) {
		enable = 1;
	} else 	if( count >=3 && !strncmp(buf, "off", 3)) {
		enable = 0;
	} else 	if( count >=2 && !strncmp(buf, "on", 2)) {
		enable = 1;
	} else  if( buf[0] == '0' ) {
		enable = 0;
	} else 	if( buf[0] == '1' ) {
		enable = 1;
	}

	if( enable )
		modem_act_enable (g_ctxt);
	else
		modem_act_disable (g_ctxt);

	return count;
}

DEVICE_ATTR(enabled, S_IRUGO | S_IWUSR, 
            modem_act_attr_enabled_show, modem_act_attr_enabled_store);


/*
 *  Debug level attribute
 */
static ssize_t 
modem_act_attr_debug_level_show(struct device *dev, 
                                struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_log );
}

static ssize_t 
modem_act_attr_debug_level_store(struct device *dev, 
                                 struct device_attribute *attr, 
                                 const char *buf, size_t count)
{
	int value;
	
	if(!count)
		return count;
	
	sscanf(buf, "%d", &value);
	
	g_log = value;
	
	return count;
}

DEVICE_ATTR(debug_level, S_IRUGO | S_IWUSR, 
            modem_act_attr_debug_level_show, modem_act_attr_debug_level_store);


/*
 *
 */
static int __init 
modem_act_probe(struct platform_device *pdev)
{
	struct modem_activity_monitor_config *cfg = NULL;
	int ret = 0;

	printk(KERN_INFO "%s: initializing driver.\n", DEV_NAME );

	if( g_ctxt->initialized ) {
		printk(KERN_ERR "%s: already initialized\n", DEV_NAME);
		return -ENODEV;
	}

	if (pdev->dev.platform_data == NULL ) {
		printk(KERN_ERR "%s: no platform data found.\n", DEV_NAME );
		return -ENODEV;
	}
	cfg = pdev->dev.platform_data;

	if (cfg->app_wake_modem_gpio < 0 ) {
		printk(KERN_ERR "%s: AWM pin is required.\n", DEV_NAME );
		return -ENODEV;
	}

	if (cfg->modem_wake_app_gpio < 0 && 
	    cfg->modem_wake_usb_gpio < 0 ) {
		printk(KERN_ERR "%s: need at least one MWU/MWA pin\n",DEV_NAME);
		return -ENODEV;
	}

	/* Right now we only support one instance. Initialize it. */
	g_ctxt->enabled        =  1;
	g_ctxt->warn_cnt       =  10;
	g_ctxt->gpio_flags     =  cfg->gpio_flags;
	g_ctxt->app_wake_modem =  cfg->app_wake_modem_gpio;
	g_ctxt->modem_wake_app =  cfg->modem_wake_app_gpio;
	g_ctxt->modem_wake_usb =  cfg->modem_wake_usb_gpio;
	g_ctxt->uart_port      =  cfg->uart;
	g_ctxt->timeout        =  msecs_to_jiffies(cfg->timeout);
	
	printk(KERN_INFO "%s: AWM=%d, MWA=%d, MWU=%d "
	         " Timeout %dms, Uart Port %d. Acm dev (0x%x)\n", DEV_NAME,
	         cfg->app_wake_modem_gpio, 
	         cfg->modem_wake_app_gpio, 
	         cfg->modem_wake_usb_gpio,
	         cfg->timeout, cfg->uart, 0 );
	
	spin_lock_init(&g_ctxt->lock);
	mutex_init(&g_ctxt->mlock);

	g_ctxt->workq = create_freezeable_workqueue("modem_act");
	if (!g_ctxt->workq)
		return -ENOMEM;
	g_ctxt->workq_rmw = create_freezeable_workqueue("modem_rmw");
	if (!g_ctxt->workq_rmw)
		return -ENOMEM;

	INIT_WORK(&g_ctxt->awake_work, modem_act_work);
	INIT_WORK(&g_ctxt->awake_usb_work, modem_act_usb_work);
	
	setup_timer   (&g_ctxt->act_timer, modem_act_timer, 
	                (unsigned long ) g_ctxt );

	setup_timer   (&g_ctxt->wkup_timer, modem_wkup_timer, 
	                (unsigned long ) g_ctxt );
	
	if(!(cfg->gpio_flags & GPIO_FLG_AWM_CONFIGURED)) {
		ret = gpio_request(cfg->app_wake_modem_gpio, "awm");
		if( ret )
			goto awm_gpio_request_err;
		gpio_direction_output(cfg->app_wake_modem_gpio, 0);
	}
	
	if( cfg->modem_wake_app_gpio >= 0 ) {
		if(!(cfg->gpio_flags & GPIO_FLG_MWA_CONFIGURED)) {
			ret = gpio_request(cfg->modem_wake_app_gpio, "mw_app");
			if( ret )
				goto mwa_gpio_request_err;
			gpio_direction_input(cfg->modem_wake_app_gpio);
		}
		ret = request_irq( gpio_to_irq(cfg->modem_wake_app_gpio), 
			               mwa_interrupt, IRQF_TRIGGER_RISING, 
			               "mw_app", g_ctxt );
		if( ret )
			goto mwa_irq_err;
	}
	
	if( cfg->modem_wake_usb_gpio >= 0 ) {
		if(!(cfg->gpio_flags & GPIO_FLG_MWU_CONFIGURED)) {
			ret = gpio_request( cfg->modem_wake_usb_gpio,"mw_usb");
			if( ret )
				goto mwu_gpio_request_err;
			gpio_direction_input(cfg->modem_wake_usb_gpio);
		}
		ret = request_irq( gpio_to_irq(cfg->modem_wake_usb_gpio), 
			               mwau_interrupt, IRQF_TRIGGER_RISING | 
			                               IRQF_TRIGGER_FALLING, 
			               "mw_usb", g_ctxt );
		if( ret ) {
			goto mwu_irq_err;
		}
	}
	
	// this device can wakeup device
	device_init_wakeup(&pdev->dev, 1);

	// always initialize completion
	init_completion(&g_ctxt->awake_comp);

	/* create device sysfs attributes */
	ret = device_create_file(&pdev->dev, &dev_attr_enabled);
	if( ret < 0 ) {
		goto attr_enabled_err;
	}

	/* create device sysfs attributes */
	ret = device_create_file(&pdev->dev, &dev_attr_debug_level);
	if( ret < 0 ) {
		goto attr_debug_level_err;
	}

	if( gpio_get_value(cfg->app_wake_modem_gpio)) {
		// assume that modem is awake
		g_ctxt->awake_state   = AWM_STATE_AWAKE; 
		g_ctxt->awake_pending = 0;
		// and start deassert timer
		if( g_ctxt->enabled && g_ctxt->usb_suspended ) {
			mod_timer ( &g_ctxt->act_timer, 
			             jiffies + g_ctxt->timeout );
		}
		PDBG("%s: assume modem awake\n", DEV_NAME);
	} else {
		// assume modem is asleep
		g_ctxt->awake_state   = AWM_STATE_ASLEEP;
		g_ctxt->awake_pending = 0;
		PDBG("%s: assume modem asleep\n", DEV_NAME);
	}

	g_ctxt->initialized = 1;
	
	return 0;

attr_debug_level_err:
	device_remove_file(&pdev->dev, &dev_attr_enabled);

attr_enabled_err:
	if( cfg->modem_wake_usb_gpio >= 0 ) {
		free_irq(gpio_to_irq(cfg->modem_wake_usb_gpio), g_ctxt );
	}

mwu_irq_err:
	if( cfg->modem_wake_usb_gpio >= 0 ) {
		if(!(cfg->gpio_flags & GPIO_FLG_MWU_CONFIGURED))
			gpio_free(cfg->modem_wake_usb_gpio);
	}

mwu_gpio_request_err:
	free_irq(gpio_to_irq(cfg->modem_wake_app_gpio), g_ctxt );

mwa_irq_err:
	if( cfg->modem_wake_app_gpio >= 0 ) {
		if(!(cfg->gpio_flags & GPIO_FLG_MWA_CONFIGURED))
			gpio_free(cfg->modem_wake_app_gpio);
	}

mwa_gpio_request_err:
	if(!(cfg->gpio_flags & GPIO_FLG_AWM_CONFIGURED))
		gpio_free(cfg->app_wake_modem_gpio);
		
awm_gpio_request_err:

	if( g_ctxt->workq )
		destroy_workqueue(g_ctxt->workq);
	if( g_ctxt->workq_rmw )
		destroy_workqueue(g_ctxt->workq_rmw);

	return -ENODEV;
}


static void
modem_act_stop (struct modem_act_ctxt *ctxt)
{
	if( ctxt->modem_wake_usb )
		disable_irq (gpio_to_irq(ctxt->modem_wake_usb));
	
	if( ctxt->modem_wake_app )
		disable_irq (gpio_to_irq(ctxt->modem_wake_app));

	/* delete the APP_WAKE_MODEM timer */
	del_timer(&ctxt->act_timer);

	/* Kill the work */
	cancel_work_sync(&ctxt->awake_work);

	/* De-Assert APP_WAKE_MODEM */
	PDBG("%s: %10lu: deassert AWM\n", __FUNCTION__, jiffies);
	gpio_set_value(ctxt->app_wake_modem, 0);

	ctxt->awake_state = AWM_STATE_ASLEEP;
	ctxt->awake_pending = 0;
}

static int 
modem_act_remove(struct platform_device *pdev)
{
	if(!g_ctxt->initialized)
		return 0;

	modem_act_stop (g_ctxt);

	device_remove_file(&pdev->dev, &dev_attr_debug_level);
	device_remove_file(&pdev->dev, &dev_attr_enabled);

	if( g_ctxt->modem_wake_app >= 0 ) {
		free_irq (gpio_to_irq(g_ctxt->modem_wake_app), g_ctxt );
		if(!(g_ctxt->gpio_flags & GPIO_FLG_MWA_CONFIGURED))
			gpio_free(g_ctxt->modem_wake_app);
	}

	if( g_ctxt->modem_wake_usb >= 0 ) {
		free_irq (gpio_to_irq(g_ctxt->modem_wake_usb), g_ctxt );
		if(!(g_ctxt->gpio_flags & GPIO_FLG_MWU_CONFIGURED))
			gpio_free(g_ctxt->modem_wake_usb);
	}

	if(!(g_ctxt->gpio_flags & GPIO_FLG_AWM_CONFIGURED))
		gpio_free(g_ctxt->app_wake_modem);

	if( g_ctxt->workq )
		destroy_workqueue(g_ctxt->workq);
	if( g_ctxt->workq_rmw )
		destroy_workqueue(g_ctxt->workq_rmw);

	g_ctxt->initialized = 0;

	return 0;
}

#ifdef CONFIG_PM
static int 
modem_act_suspend(struct platform_device *pdev, pm_message_t state)
{
	PDBG("%s:\n", __FUNCTION__ );

	modem_act_stop (g_ctxt);

	if( device_may_wakeup(&pdev->dev)) {
		if( g_ctxt->modem_wake_usb )
			enable_irq_wake(gpio_to_irq(g_ctxt->modem_wake_usb));
			
		if( g_ctxt->modem_wake_app )
			enable_irq_wake(gpio_to_irq(g_ctxt->modem_wake_app));
	}

	return 0;
}

static int 
modem_act_resume(struct platform_device *pdev)
{
	PDBG("%s:\n", __FUNCTION__ );

	if( device_may_wakeup(&pdev->dev)) {
		if( g_ctxt->modem_wake_usb )
			disable_irq_wake(gpio_to_irq(g_ctxt->modem_wake_usb));
			
		if( g_ctxt->modem_wake_app )
			disable_irq_wake(gpio_to_irq(g_ctxt->modem_wake_app));
	}

	if( g_ctxt->modem_wake_usb )
		enable_irq (gpio_to_irq(g_ctxt->modem_wake_usb));

	if( g_ctxt->modem_wake_app )
		enable_irq (gpio_to_irq(g_ctxt->modem_wake_app));

	return 0;
}
#else
#define modem_act_suspend  NULL
#define modem_act_resume   NULL
#endif

/******************************************************************************
 *
 * Init
 *
 ******************************************************************************/

static struct platform_driver modem_act_driver = {
	.probe    = modem_act_probe,
	.remove   = modem_act_remove,
	.suspend  = modem_act_suspend,
	.resume   = modem_act_resume,
	.driver = {
		.name = "modem_activity",
	},
};

static int __init 
modem_act_init(void)
{
	PDBG("%s:\n", __FUNCTION__);
	if (platform_driver_register(&modem_act_driver)) {
		printk(KERN_ERR "%s: Failed to register driver.\n", DEV_NAME );
		return -ENODEV;
	}
	return 0;
}

static void __exit 
modem_act_exit(void)
{
	PDBG("%s:\n", __FUNCTION__);
	platform_driver_unregister(&modem_act_driver);
}

module_init(modem_act_init);
module_exit(modem_act_exit);

MODULE_DESCRIPTION("Modem Activity Monitor Module");
MODULE_AUTHOR("Palm Inc.");
MODULE_LICENSE("GPL");

