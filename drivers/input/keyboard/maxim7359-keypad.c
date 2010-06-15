/*
 * linux/drivers/input/keyboard/maxim-keypad.c
 *
 * Keypad driver based on MAXIM7359 controller
 *
 * Copyright (C) 2007 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
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

#define MAX7359_MAX_DEBOUNCE 40
#define MAX7359_MIN_DEBOUNCE 9
#define MAX7359_REG_DEBOUNCE(x)	((x -MAX7359_MIN_DEBOUNCE )&0x1f)


#define MAXIM7359_MAX_KEYS      (64)

struct  key_event {
	int key;
	struct timeval tv;
};

struct maxim_kp_state {
	struct i2c_client *i2c_dev;
	struct input_dev *inp_dev;
	struct maxim7359_platform_data *pdata;
	struct work_struct scan_work;
	int suspended;
	DECLARE_BITMAP(gpio_irq_disabled, 1);
	typeof(jiffies) key_down_tstamp[MAXIM7359_MAX_KEYS];
	typeof(jiffies) prox_timeout;
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
	struct maxim_kp_state *state = dev_id;
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
is_valid_event (struct maxim_kp_state *state, int idx, int down )
{
	typeof(jiffies) ts;

	if( idx < 0 || idx >= MAXIM7359_MAX_KEYS )
		return 0; // drop it

	if(!state->pdata->key_prox_map) 
		return 1; // no prox map just return a valid event 

	if( down ) { 
		if( drop_key_down(state, idx))
			return 0;
		ts = jiffies;
		state->key_down_tstamp[idx] = ts ? ts : 1;
		return 1;
	} else {
		if(!state->key_down_tstamp[idx])
			return 0;  // key was not down (ignore event)
		state->key_down_tstamp[idx] = 0;
		return 1;
	}
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
	int res, col, row, key, idx, down;
	struct maxim_kp_state *state =
	    container_of(work, struct maxim_kp_state, scan_work);

	for(;;) {
		res = maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_FIFO, &val);
		if( res < 0 ) {
			printk (KERN_ERR "maxim_i2c_read_u8: failed (%d)\n", res );
			break;
		}
		if( val == FIFO_EMPTY ) {
			break;
		}
		col = (val & 0x3F) / 8;
		row = (val & 0x3F) % 8;
		idx = state->pdata->col_num * row + col;
		key = state->pdata->keymap[idx];
		if( key == KEY_RESERVED )
			continue; // skip the key
			
		down  = (val & KEY_RELEASED) ? 0 : 1; 
		if( is_valid_event(state, idx, down)) {
		    input_report_key(state->inp_dev, key, down);
		    input_sync(state->inp_dev);
		}
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

	// Use input device default autorepeat settings
	state->inp_dev->rep[REP_DELAY] = 0;
	state->inp_dev->rep[REP_PERIOD] = 0;

	// setup key proximity timeout
	state->prox_timeout = msecs_to_jiffies(pdata->key_prox_timeout); 

	/* register input device */
	rc = input_register_device(state->inp_dev);
	if (rc != 0)
		goto err1;

	/* change autorepeat to actual values */
	state->inp_dev->rep[REP_DELAY]	= pdata->rep_delay;
	state->inp_dev->rep[REP_PERIOD]	= pdata->rep_period;

	/* Setting debounce time */
	debounce_time = pdata->debounce;
	if ( debounce_time < MAX7359_MIN_DEBOUNCE) {
		printk(KERN_WARNING "MAX7359: Does not support debounce < %d mSec\n", MAX7359_MIN_DEBOUNCE);
		debounce_time = MAX7359_MIN_DEBOUNCE;
	}
	if ( debounce_time > MAX7359_MAX_DEBOUNCE) {
		printk(KERN_WARNING "MAX7359: Does not support debounce > %d mSec\n", MAX7359_MAX_DEBOUNCE);
		debounce_time = MAX7359_MAX_DEBOUNCE;
	}

	(void)maxim_i2c_write_u8(client, MAXIM7359_DEBOUNCE, MAX7359_REG_DEBOUNCE(debounce_time) );

	/* Setting Interrupt type */
	(void)maxim_i2c_write_u8(client, MAXIM7359_INTERRUPT, 0x01);

	/* drain the fifo of any leftover bits */
	/* DOLATER: The code below might not be enough */
	{
		u8 val;
		for (i=0; i < 16; i++) {
			(void)maxim_i2c_read_u8(state->i2c_dev, MAXIM7359_FIFO, &val);
		}
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
