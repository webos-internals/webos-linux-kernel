/*
 * linux/drivers/input/keyboard/maxim-keypad.c
 *
 * Keypad driver based on MAXIM7359 controller
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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c_maxim7359_keypad.h>

#ifdef CONFIG_KEYBOARD_MAXIM7359_CPUFREQ_TICKLE
#include <linux/cpufreq.h>
#endif

#include <asm/irq.h>
#include <asm/arch/gpio.h>

#define KEY_RELEASED            (0x40)

#define MAXIM7359_FIFO          (0x00)
#define MAXIM7359_CONFIG        (0x01)
#define MAXIM7359_DEBOUNCE      (0x02)
#define MAXIM7359_INTERRUPT     (0x03)
#define MAXIM7359_GPO_PORTS     (0x04)
#define MAXIM7359_KEY_REPEAT    (0x05)
#define MAXIM7359_SLEEP         (0x06)

#define GPO_PORT_DISABLE        (0x1F)	//GPO Ports disabled (full key-scan functionality)
#define FIFO_EMPTY		(0x3F)	//Keyboard controller FIFO is empty
#define FIFO_OVERFLOW	(0x7F) // Keyboard controller FIFO has overflowed.

#define MAX7359_MAX_DEBOUNCE 40
#define MAX7359_MIN_DEBOUNCE 9
#define MAX7359_REG_DEBOUNCE(x)	((x -MAX7359_MIN_DEBOUNCE )&0x1f)

#define MAXIM7359_MAX_KEYS      (64)

struct  key_event {
	int key;
	struct timeval tv;
};

struct key_list {
	struct list_head link;
	typeof(jiffies) timeout;
	int idx;
};

struct maxim_kp_state {
	struct i2c_client *i2c_dev;
	struct input_dev *inp_dev;
	struct maxim7359_platform_data *pdata;
	struct work_struct scan_work;
	int suspended;
	DECLARE_BITMAP(gpio_irq_disabled, 1);
	typeof(jiffies) key_down_tstamp[MAXIM7359_MAX_KEYS];
	struct list_head key_up_list_head;
	struct key_list key_up_list[MAXIM7359_MAX_KEYS];
	struct timer_list sw_debounce_timer;
	typeof(jiffies) prox_timeout;
	typeof(jiffies) sw_debounce_timeout;
	int             hw_debounce;
};

#ifdef CONFIG_INPUT_EVDEV_TRACK_QUEUE
extern int evdev_get_queue_state(struct input_dev *dev);
#endif


/******************************************************************************
*
* Enable (if it was disabled) IRQ.
*
******************************************************************************/

#ifdef CONFIG_PM
static void maxim_enable_irq(struct maxim_kp_state *state)
{
	if (test_and_clear_bit(0, state->gpio_irq_disabled))
		enable_irq(state->i2c_dev->irq);
}
#endif

/******************************************************************************
*
* Disable (if it was enabled) IRQ.
*
******************************************************************************/

static void maxim_disable_irq(struct maxim_kp_state *state)
{
	if (!test_and_set_bit(0, state->gpio_irq_disabled))
		disable_irq(state->i2c_dev->irq);
}

/******************************************************************************
*
* maxim_kp_interrupt()
*
* The handler for the irq when a key is pressed or released. Disable the
* irq lines and schedules workqueue to scan the key and report the event
* to the input subsystem
*
* Inputs
*   irq         The IRQ number being serviced. Not used.
*   dev_id      Device ID provided when IRQ was registered
*
* Returns
*   IRQ_HANDLED
*
******************************************************************************/

static irqreturn_t maxim_kp_interrupt(int irq, void *dev_id)
{
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_id;
	schedule_work(&state->scan_work);
	return IRQ_HANDLED;
}

/******************************************************************************
*
* maxim_i2c_read_u8()
*
*
* Inputs
*   
*
* Returns
*   
*
******************************************************************************/

static int maxim_i2c_read_u8(struct i2c_client *client, u8 index, u8 * out)
{
	int ret;
	struct i2c_msg msgs[2];

	// Write the register index.
	msgs[0].addr	= client->addr;
	msgs[0].len	= 1;
	msgs[0].flags	= 0;
	msgs[0].buf	= &index;

	// Read the register value.
	msgs[1].addr	= client->addr;
	msgs[1].len	= 1;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].buf	= out;

	// Make it so...
	ret = i2c_transfer(client->adapter, msgs, 2);

	return (ret);
}

/******************************************************************************
*
* maxim9_i2c_write_u8()
*
*
* Inputs
*   
*
* Returns
*   
*
******************************************************************************/

static int maxim_i2c_write_u8(struct i2c_client *client, u8 index, u8 value)
{
	u8 buf[2] = { index, value };
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr	= client->addr;
	msg[0].flags	= 0;
	msg[0].len	= 2;
	msg[0].buf	= buf;

	ret = i2c_transfer(client->adapter, msg, 1);
	return ret;
}

#ifdef MAXIM7359_DEBUG

void maxim_dump_regs(struct i2c_client *client) {
	u8 val;
	int x;

	for(x=0; x<=6; x++) {
		if(maxim_i2c_read_u8(client, x, &val) >= 0) {
			printk("%s: R%d=0x%x\n", MAXIM7359_I2C_DRIVER, x, val);
		} else{
			printk("%s: Failed to read R%d\n", MAXIM7359_I2C_DRIVER, x);
		}

	}

}

#endif

/******************************************************************************
*
* Sysfs
*
******************************************************************************/

static ssize_t
key_state_show(	struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);
	int idx;
	char *cur = buf;

	for(idx = 0 ; (idx < MAXIM7359_MAX_KEYS) && (cur < buf+PAGE_SIZE); idx++) {
		if(state->key_down_tstamp[idx]) {
			cur += snprintf(cur, buf+PAGE_SIZE-cur, "%d:%u\n", state->pdata->keymap[idx], jiffies_to_msecs(jiffies-state->key_down_tstamp[idx]));
		}
	}

	return (cur - buf);
}

static DEVICE_ATTR(key_state, S_IRUGO, key_state_show, NULL );

#ifdef MAXIM7359_DEBUG

static ssize_t
hw_debounce_show(	struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);
	u8 debounce;

	if(maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_DEBOUNCE, &debounce )<0) {
		printk(KERN_WARNING "%s: Failed to get debouce\n", MAXIM7359_I2C_DRIVER);
	} else {
		debounce += 9;
		state->hw_debounce = debounce;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", state->hw_debounce);

}

static ssize_t
hw_debounce_store( struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	char *endp;
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);
	int new_debounce_time = simple_strtoul(buf, &endp, 10);

	if ( new_debounce_time < MAX7359_MIN_DEBOUNCE) {
		printk(KERN_WARNING "MAX7359: Does not support debounce < %d mSec\n", MAX7359_MIN_DEBOUNCE);
		new_debounce_time = MAX7359_MIN_DEBOUNCE;
	}
	if ( new_debounce_time > MAX7359_MAX_DEBOUNCE) {
		printk(KERN_WARNING "MAX7359: Does not support debounce > %d mSec\n", MAX7359_MAX_DEBOUNCE);
		new_debounce_time = MAX7359_MAX_DEBOUNCE;
	}
	state->hw_debounce = new_debounce_time;
	(void)maxim_i2c_write_u8(state->i2c_dev, MAXIM7359_DEBOUNCE, MAX7359_REG_DEBOUNCE(new_debounce_time) );

	return count;
}

static DEVICE_ATTR(hw_debounce, S_IWUGO | S_IRUGO, hw_debounce_show, hw_debounce_store);

static ssize_t
sw_debounce_show(	struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", jiffies_to_msecs(state->sw_debounce_timeout));
}

static ssize_t
sw_debounce_store( struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	char *endp;
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);
	int new_debounce_time = simple_strtoul(buf, &endp, 10);

	state->sw_debounce_timeout = msecs_to_jiffies(new_debounce_time);

	return count;
}

static DEVICE_ATTR(sw_debounce, S_IWUGO | S_IRUGO, sw_debounce_show, sw_debounce_store);


static ssize_t
autosleep_show(	struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);
	u8 sleep = 0;

	if(maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_SLEEP, &sleep )<0) {
		printk(KERN_WARNING "%s: Failed to get sleep\n", MAXIM7359_I2C_DRIVER);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", sleep);
}

static ssize_t
autosleep_store( struct device *dev,
                struct device_attribute *attr,
                const char *buf, size_t count)
{
	char *endp;
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);
	int new_autosleep_time = simple_strtoul(buf, &endp, 10);

	(void)maxim_i2c_write_u8(state->i2c_dev, MAXIM7359_SLEEP, new_autosleep_time & 0x7);

	return count;
}

static DEVICE_ATTR(autosleep, S_IWUGO | S_IRUGO, autosleep_show, autosleep_store);

#endif

static ssize_t
fifo_read_show(	struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct maxim_kp_state *state = (struct maxim_kp_state *)dev_get_drvdata(dev);
	u8 val;
	int col, row, idx, key, nirq;

	if(maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_FIFO, &val )<0) {
		printk(KERN_WARNING "%s: Failed to get val from FIFO\n", MAXIM7359_I2C_DRIVER);
	}

	col = (val & 0x3F) / 8;
	row = (val & 0x3F) % 8;
	idx = state->pdata->col_num * row + col;
	key = state->pdata->keymap[idx];

	nirq = gpio_get_value(irq_to_gpio(state->i2c_dev->irq));

	return snprintf(buf, PAGE_SIZE, "0x%X:%d:%d:%d:%d:%d\n",val, idx, col, row, key, nirq);
}

static DEVICE_ATTR(fifo_read, S_IRUGO, fifo_read_show, NULL);



/******************************************************************************
 *****************************************************************************/

/* 
 *  Check if any adjacent key was pressed 
 */
static int
drop_key_down (struct maxim_kp_state *state, int idx)
{
	int  i, adj;
	int  wmap = state->pdata->key_prox_width;
	u8  *pmap = state->pdata->key_prox_map;
	typeof(jiffies) *kdts;

	pmap += idx * wmap;
	kdts  = state->key_down_tstamp;
	for( i = 0; i < wmap; i++ ) {
		adj = (int) pmap[i];
		if( adj == 0xFF ) 
			break;    // the end
		if(!kdts[adj]) 
			continue; // next in list
		if( time_before(jiffies, kdts[adj] + state->prox_timeout))
		    return 1;     // drop the key
	}
	return 0;
}

/*
 *  Check if it is a valid event
 */
static int 
is_valid_down_event (struct maxim_kp_state *state, int idx)
{

	if( idx < 0 || idx >= MAXIM7359_MAX_KEYS ) {
		printk(KERN_WARNING "%s: idx out of range (idx=%d)\n", MAXIM7359_I2C_DRIVER, idx);
		return 0; // drop it
	}

	 /* if there is no prox map just return a valid event */
	if(!state->pdata->key_prox_map) 
		return 1;

	/* Key mashing and inconsistent state rejection */
	if( state->key_down_tstamp[idx] ) {
		printk(KERN_NOTICE "%s: Two key down events (missing up). (idx=%d, time=%d)\n",
				MAXIM7359_I2C_DRIVER, idx, jiffies_to_msecs(jiffies-state->key_down_tstamp[idx]));
	}
	if( drop_key_down(state, idx)) {
		printk(KERN_NOTICE "%s: Rejecting neighbouring key (idx=%d)\n", MAXIM7359_I2C_DRIVER, idx);
		return 0;
	}

	return 1;

}

/*
 *  Timer handler for reporting deffered up keys
 */

static void
up_key_timer(unsigned long data)
{
	struct maxim_kp_state *state = (struct maxim_kp_state *)data;

	/* Use the same workhandler to simplify the synchronization */
	schedule_work(&state->scan_work);

	return;
}

static void send_key_event(struct input_dev *dev, unsigned int code, int value)
{
#ifdef CONFIG_KEYBOARD_MAXIM7359_CPUFREQ_TICKLE
	CPUFREQ_FLOOR_MILLIS(CONFIG_KEYBOARD_GPIO_PE_CPUFREQ_FLOOR_FREQ * 1000, 
	                     CONFIG_KEYBOARD_GPIO_PE_CPUFREQ_FLOOR_TIMEOUT);
#endif
	input_report_key(dev, code, value);
	input_sync(dev);
}

/******************************************************************************
*
* maxim_kp_scan()
*
* Scan FIFO for interpret the key-event.
* 
*
* Inputs
*   None.
*
* Returns
*   None.
*
******************************************************************************/

static void maxim_kp_scan(struct work_struct *work)
{
	u8  val;

	int res, col, row, key=0, idx=0, down=0;
	struct maxim_kp_state *state =
	    container_of(work, struct maxim_kp_state, scan_work);
	struct key_list *item = NULL, *titem;
	typeof(jiffies) ts;
	int readtimeout=10;

	for(;;) {
		res = maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_FIFO, &val);
		if( res < 0 ) {
			printk (KERN_ERR "%s: maxim_i2c_read_u8: failed (%d)\n", MAXIM7359_I2C_DRIVER, res );
			break;
		}
		if( val == FIFO_EMPTY ) {
			/* According to Maxim, it is not enough to check for FIFO empty  */
			/* We must keep reading until FIFO is empty and the irq has      */
			/* been deasserted                                               */
			if(gpio_get_value(irq_to_gpio(state->i2c_dev->irq))) {
				break;
			} else {
				/* If for some reason this does not happen for 'timeout' reads */
				/* then we will set up a timer to check again later and bail   */
				/* This should *rarely* to *never* happen                      */

				if(!readtimeout--) {
					/* reuse the same timer we use for debounce */
					mod_timer(&state->sw_debounce_timer, HZ >> 2);
					break;
				}

				continue;
			}
		}

		if( val == FIFO_OVERFLOW) {
			printk (KERN_WARNING "%s: FIFO overflow\n", MAXIM7359_I2C_DRIVER);
			/* Clear out all the keypresses and send up events for all keys currently held down */
			do {
				res = maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_FIFO, &val);
				if( res < 0 ) {
					printk (KERN_ERR "%s: maxim_i2c_read_u8: failed (%d)\n", MAXIM7359_I2C_DRIVER, res );
				}
			} while( val != FIFO_EMPTY);
			
			for(idx = 0 ; idx < MAXIM7359_MAX_KEYS; idx++) {
				if(state->key_down_tstamp[idx]) {
					key = state->pdata->keymap[idx];
					printk (KERN_NOTICE "%s: Sending key up for key code %d\n", MAXIM7359_I2C_DRIVER, key);
					send_key_event(state->inp_dev, key, 0);
					state->key_down_tstamp[idx] = 0;
				}
			}
			break;
		}	

		col = (val & 0x3F) / 8;
		row = (val & 0x3F) % 8;
		idx = state->pdata->col_num * row + col;
		key = state->pdata->keymap[idx];
		if( key == KEY_RESERVED ) {
			printk(KERN_WARNING "%s: received reserved key. (val=0x%X)\n", MAXIM7359_I2C_DRIVER, val);
			continue; // skip the key
		}	
		down  = (val & KEY_RELEASED) ? 0 : 1;

#ifdef MAXIM7359_DEBUG
		printk(KERN_INFO "%s: key %d %s\n", MAXIM7359_I2C_DRIVER, idx, down ? "down" : "up");
#endif

		if(!down) {
			/* don't report the up just yet */
			state->key_up_list[idx].timeout = jiffies + state->sw_debounce_timeout;

			/* Need to be careful not to create a circular link in our list */
			if(list_empty(&state->key_up_list[idx].link)) {
				list_add_tail(&state->key_up_list[idx].link, &state->key_up_list_head);
			}
			/* don't worry about the timer, it will be kicked off below if necessary */

		}else if( is_valid_down_event(state, idx)) {
			/* key down is not from button mashing */

			/* check to see if there was recently a key up */
			if(!list_empty(&state->key_up_list[idx].link)) {
				printk(KERN_NOTICE "%s: Rejecting key down bounce. (idx=%d)\n", MAXIM7359_I2C_DRIVER, idx);
				/* this must be a bounce, do not report the key up */
				list_del_init(&state->key_up_list[idx].link);
			} else {
				/* key down must be valid */

				/* update the last valid key down seen */
				ts = jiffies;
				state->key_down_tstamp[idx] = ts ? ts : 1;

				send_key_event(state->inp_dev, key, 1);
			}
		} else {
			printk(KERN_NOTICE "%s: key event not valid. (idx=%d down=%d)\n", MAXIM7359_I2C_DRIVER, idx, down);
		}
	}

	/* Now check if it is time to report key ups */
	list_for_each_entry_safe(item, titem, &state->key_up_list_head, link) {

		/* Is it time to check this up key? */
		if(time_before(jiffies, item->timeout)) {
			/* Not yet time, reset the timer and break */
			mod_timer(&state->sw_debounce_timer, item->timeout);
			break;
		}

		/* Check if the previous down was valid */
		if(!state->key_down_tstamp[item->idx]) {
			printk(KERN_NOTICE "%s: Got key up with no previous key down. (idx=%d)\n", MAXIM7359_I2C_DRIVER, item->idx);
		} else {
			/* the key is now officially up */
			state->key_down_tstamp[item->idx] = 0;

			key = state->pdata->keymap[item->idx];
			send_key_event(state->inp_dev, key, 0);
		}

		/* We are done with this key */
		list_del_init(&item->link);
	}

}


/******************************************************************************
*
* maxim_i2c_probe()
*
* Called by driver model to initialize device
*
* Returns
*   0 on success, or non-zero otherwise.
* 
******************************************************************************/

static int maxim_i2c_probe(struct i2c_client *client)
{
	int i, rc;
	struct maxim_kp_state *state = NULL;
	struct maxim7359_platform_data *pdata = NULL;
	int  debounce_time;

	printk(KERN_INFO "%s keyboard driver\n", MAXIM7359_I2C_DRIVER);

	/* get platform data */
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		printk(KERN_ERR "%s: missing platform data\n",
		       MAXIM7359_I2C_DRIVER);
		return (-ENODEV);
	}

	/* Create the keypad state */
	state = kzalloc(sizeof(struct maxim_kp_state), GFP_KERNEL);
	if (!state)
		return (-ENOMEM);

	/* attach i2c_dev */
	state->i2c_dev = client;

	/* attach platform data */
	state->pdata = pdata;

	/* init workq */
	INIT_WORK(&state->scan_work, maxim_kp_scan);

	/* set up key up debounce */
	INIT_LIST_HEAD(&state->key_up_list_head);
	setup_timer(&state->sw_debounce_timer, up_key_timer, (unsigned long)state);

	for(i=0; i<sizeof(state->key_up_list)/sizeof(state->key_up_list[0]);i++) {
		state->key_up_list[i].idx = i;
		INIT_LIST_HEAD(&state->key_up_list[i].link);
	}

	/* setup input device */
	state->inp_dev = input_allocate_device();
	if (state->inp_dev == NULL)
		goto err0;

	/* Enable key event and auto repeat feature of Linux input subsystem */
	set_bit(EV_KEY, state->inp_dev->evbit);
	set_bit(EV_REP, state->inp_dev->evbit);

	/* setup driver name */
	state->inp_dev->name = pdata->dev_name;

	/* setup keymap */
	state->inp_dev->keycode		= pdata->keymap;
	state->inp_dev->keycodemax	= pdata->row_num * pdata->col_num;
	state->inp_dev->keycodesize	= sizeof(int);

	for (i = 0; i < state->inp_dev->keycodemax; i++)
		set_bit(pdata->keymap[i] & KEY_MAX, state->inp_dev->keybit);

	/* Use input device default autorepeat settings */
	state->inp_dev->rep[REP_DELAY] = 0;
	state->inp_dev->rep[REP_PERIOD] = 0;

	/* setup key proximity timeout */
	state->prox_timeout = msecs_to_jiffies(pdata->key_prox_timeout); 

	/* setup key sw debounce timeout */
	state->sw_debounce_timeout = msecs_to_jiffies(pdata->sw_debounce);

	/* register input device */
	rc = input_register_device(state->inp_dev);
	if (rc != 0)
		goto err1;

	rc = device_create_file(&(state->i2c_dev->dev), &dev_attr_key_state);
	if(rc) {
		goto err2;
	}

	rc = device_create_file(&(state->i2c_dev->dev),  &dev_attr_fifo_read);
	if(rc) {
		goto err2;
	}

#ifdef MAXIM7359_DEBUG
	rc = device_create_file(&(state->i2c_dev->dev),  &dev_attr_hw_debounce);
	if(rc) {
		goto err2;
	}

	rc = device_create_file(&(state->i2c_dev->dev),  &dev_attr_sw_debounce);
	if(rc) {
		goto err2;
	}

	rc = device_create_file(&(state->i2c_dev->dev),  &dev_attr_autosleep);
	if(rc) {
		goto err2;
	}
#endif

	/* change autorepeat to actual values */
	state->inp_dev->rep[REP_DELAY]	= pdata->rep_delay;
	state->inp_dev->rep[REP_PERIOD]	= pdata->rep_period;

	/* Setting hw debounce time on part */
	debounce_time = pdata->hw_debounce;
	if ( debounce_time < MAX7359_MIN_DEBOUNCE) {
		printk(KERN_WARNING "%s: Does not support debounce < %d mSec\n",MAXIM7359_I2C_DRIVER, MAX7359_MIN_DEBOUNCE);
		debounce_time = MAX7359_MIN_DEBOUNCE;
	}
	if ( debounce_time > MAX7359_MAX_DEBOUNCE) {
		printk(KERN_WARNING "%s: Does not support debounce > %d mSec\n", MAXIM7359_I2C_DRIVER, MAX7359_MAX_DEBOUNCE);
		debounce_time = MAX7359_MAX_DEBOUNCE;
	}

	state->hw_debounce = debounce_time;
	(void)maxim_i2c_write_u8(client, MAXIM7359_DEBOUNCE, MAX7359_REG_DEBOUNCE(debounce_time) );

	/* Setting Interrupt type */
	(void)maxim_i2c_write_u8(client, MAXIM7359_INTERRUPT, 0x01);

	/* drain the fifo of any leftover bits */
	{
		u8 val;
		int timeout = 100;
		do {
			(void)maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_FIFO, &val);
		} while(val != FIFO_EMPTY && timeout--);

		if(!timeout)
			printk(KERN_ERR "%s: keys still in FIFO, keyboard will not work\n", MAXIM7359_I2C_DRIVER);
	}

	rc = gpio_request(irq_to_gpio(client->irq), pdata->dev_name);
	if (rc != 0)
		goto err2;

	/* Enable interrupt */
	rc = request_irq(client->irq, maxim_kp_interrupt,
	                 IRQF_TRIGGER_FALLING, pdata->dev_name, (void *)state);
	if (rc < 0)
		goto err3;

	/* attach driver data */
	i2c_set_clientdata(client, state);

	// init device wakeup attibute
	device_can_wakeup(&state->inp_dev->dev) = 1;
	device_set_wakeup_enable ( &state->inp_dev->dev, pdata->wakeup_en );
	
#ifdef MAXIM7359_DEBUG
	maxim_dump_regs(state->i2c_dev);
#endif

	return 0;

err3:
	gpio_free(irq_to_gpio(client->irq));
err2:
	input_unregister_device(state->inp_dev);
err1:
	input_free_device(state->inp_dev);
err0:
	kfree(state);

	printk(KERN_INFO "Failed to intialize %s keyboard driver\n",
	       MAXIM7359_I2C_DRIVER);
	return -ENODEV;
}

/******************************************************************************
*
* maxim_i2c_remove()
*
* Called by driver model to remove device
*
* 
******************************************************************************/

static int maxim_i2c_remove(struct i2c_client *client)
{
	struct maxim_kp_state *state = i2c_get_clientdata(client);

	/* disable irq first */
	maxim_disable_irq(state);

	/* cancel any pending work */
	cancel_work_sync(&state->scan_work);

	/* unregister input device */
	input_unregister_device(state->inp_dev);

	device_remove_file(&(state->i2c_dev->dev), &dev_attr_key_state);
	device_remove_file(&(state->i2c_dev->dev), &dev_attr_fifo_read);
#ifdef MAXIM7359_DEBUG
	device_remove_file(&(state->i2c_dev->dev), &dev_attr_sw_debounce);
	device_remove_file(&(state->i2c_dev->dev), &dev_attr_hw_debounce);
	device_remove_file(&(state->i2c_dev->dev), &dev_attr_autosleep);
#endif

	/* add hardware deinitialization here (lowest power mode please) */

	/* unregister interrupt handler and free gpio */
	free_irq(state->i2c_dev->irq, state);
	gpio_free(irq_to_gpio(state->i2c_dev->irq));

	/* detach state */
	i2c_set_clientdata(client, NULL);

	/* free state */
	kfree(state);

	return 0;
}

#ifdef CONFIG_PM

static int maxim_kp_suspend(struct i2c_client *dev, pm_message_t event);
static int maxim_kp_resume (struct i2c_client *dev);

/******************************************************************************
*
* maxim_i2c_suspend()
* 
******************************************************************************/

static int maxim_kp_suspend(struct i2c_client *dev, pm_message_t event)
{
	struct maxim_kp_state *state = i2c_get_clientdata(dev);

	/* check if already suspended */
	if( state->suspended )
		return 0;

	/* disable interrupt */
	maxim_disable_irq(state);

	/* cancel any pending work */
	cancel_work_sync(&state->scan_work);

	/* generate release event for all pressed keys (IMPLEMENT ME) */
	//  maxim_kp_release_all_keys(state);

	/* mark it suspended */
	state->suspended = 1;

	if( device_may_wakeup(&state->inp_dev->dev)) {
		enable_irq_wake (state->i2c_dev->irq);

#ifdef CONFIG_INPUT_EVDEV_TRACK_QUEUE
		{
			int rc = evdev_get_queue_state(state->inp_dev);
			if( rc ) {
				maxim_kp_resume (dev);
				return -EBUSY;
			}
		}
#endif			
	}

	return 0;
}

/******************************************************************************
*
* maxim_i2c_resume()
* 
******************************************************************************/

static int maxim_kp_resume(struct i2c_client *dev)
{
	struct maxim_kp_state *state = i2c_get_clientdata(dev);

	if(!state->suspended)
		return 0;	// already resumed

	if( device_may_wakeup(&state->inp_dev->dev)) {
		disable_irq_wake( state->i2c_dev->irq );
	}

	/* enable interrupts */
	maxim_enable_irq(state);

	/* reset suspended flag */
	state->suspended = 0;

	return 0;
}

#else	/* CONFIG_PM */
#  define maxim_kp_suspend  NULL
#  define maxim_kp_resume   NULL
#endif	/* CONFIG_PM */

static struct i2c_driver maxim_key_driver = {
	.driver		= { .name = MAXIM7359_I2C_DRIVER, },
	.probe		= maxim_i2c_probe,
	.remove		= __devexit_p(maxim_i2c_remove),
	.suspend	= maxim_kp_suspend,
	.resume		= maxim_kp_resume,
};

/******************************************************************************
*
* maxim_kp_init()
*
* This is the function called when the module is loaded.
* Only call register driver here.
*
* Returns
*   0 on success, or non-zero otherwise.
*
******************************************************************************/

static int __init maxim_kp_init(void)
{
	return i2c_add_driver(&maxim_key_driver);
}

/******************************************************************************
*
* maxim_kp_exit()
*
* This is the function called when the module is unloaded. 
* Only call unregister driver here.
*
* Returns
*   0 on success, or non-zero otherwise.
*
******************************************************************************/

static void __exit maxim_kp_exit(void)
{
	i2c_del_driver(&maxim_key_driver);
}

module_init(maxim_kp_init);
module_exit(maxim_kp_exit);

MODULE_DESCRIPTION("Maxim7359 Keypad Driver");
MODULE_LICENSE("GPL");
