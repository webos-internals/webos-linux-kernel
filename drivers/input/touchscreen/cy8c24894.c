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
#include <linux/cy8c24894.h>
#ifdef CONFIG_HIGH_RES_TIMERS
#include <linux/hrtimer.h>
#endif
#include <linux/cpufreq.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/msm_iomap.h>

#define BOOTLOADER_PROTOCOL_BLOCK_SIZE 		156

#define TP_DEBUG

#ifdef TP_DEBUG
#define ASSERT(i)  BUG_ON(!(i))

#else
#define ASSERT(i)  ((void)0)

#endif

#define PROFILE_USAGE
#if defined PROFILE_USAGE
bool reset_active = false;
uint32_t start_time;
int32_t diff_time;
#endif

#define DEEP_SLEEP_TRANSITION_HACK

enum {
	DEVICE_BUSY_BIT = 0,
	IS_OPENED,
	COMPLETION_PENDING_BIT,
	BOOTLOAD_ACTIVE_BIT,
	SIZE_FLAGS
};

/* psoc i2c ctl reg layout at slave addr: 0x11 (7-bit) */
struct i2c_ctl_reg_values {
		
//uint8_t reserved_for_protocol;
	uint16_t fw_version;
	uint8_t active_scan_rate;	// scan rate == 5Hz to maximum
	uint8_t wot_scan_rate;		// wake-on-touch scan rate == 5Hz to maximum
	uint8_t tx8_cfg;
	uint8_t panel_id;
	uint8_t wot_threshold;
	uint8_t wot_baseline_hi;	// RESERVED FOR FUTURE requirements
	uint8_t wot_baseline_lo;
	uint8_t command;
	uint16_t mode;			// ACTV_SCAN(2), WK_ON_TOUCH(1), SLEEP(0), HW_RST(other value)
	//uint32_t reserved_4;		// RESERVED FOR FUTURE requirements
	//uint32_t reserved_5;		// RESERVED FOR FUTURE requirements
};

struct cy8c24894_device_state {
	struct i2c_client *i2c_dev;
	struct cy8c24894_platform_data *plat_data;
	struct file_operations fops;
	struct miscdevice mdev;
	
	uint16_t current_mode;
	uint16_t new_mode;
	struct mutex dev_mutex;
	struct mutex mode_mutex;
	unsigned int timestamping;
	
	wait_queue_head_t 	dev_busyq;
	struct work_struct	tp_irq_work;
	struct completion	tp_irq_complete;
	
	struct i2c_ctl_reg_values i2c_ctl_regs;
	
	DECLARE_BITMAP(flags, SIZE_FLAGS);

	int cpufreq_hold;
};


uint16_t mode_map[] = {
	[CY8C2489_DEEP_SLEEP_STATE] 	= 0x00,		/* sleep */
	[CY8C2489_WOT_STATE] 		= 0x01,		/* wot */
	[CY8C2489_ACTIVE_SCAN_STATE] 	= 0x02, 	/* active-scan */
	[CY8C2489_HW_RESET_STATE] 	= 0x0100, 	/* reset */
	[CY8C2489_IDLE_STATE] 		= 0x03,		/* idle (init state) */
	[CY8C2489_WOT_RAW_DATA_STATE] 	= 0x04,		/* wot with raw data dump */
	[CY8C2489_IDLE_NO_CMD_STATE] 	= 0xff		/* (internal state) idle w/o i2c active */
};

static struct workqueue_struct* ktpd_workqueue;

int32_t cy8c24894_read_i2c_ctl_regs(struct i2c_client* client);
int32_t cy8c24894_write_i2c_ctl_regs(struct i2c_client* client);
int32_t _set_psoc_defaults(struct cy8c24894_device_state* state);
int cy8c24894_get_power_state(struct cy8c24894_device_state* state, enum cy8c24894_power_state* power_state);

enum {
	CY8C2489_DEBUG_VERBOSE = 0x01,
};

static int cy8c24894_debug_mask = 0x0;
module_param_named(
		   debug_mask, cy8c24894_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
		  );

static int cy8c24894_tp_irq_count = 0;
module_param_named(
		   tp_irq_count, cy8c24894_tp_irq_count, int, S_IRUGO | S_IWUSR | S_IWGRP
		  );

#define CY8C2489_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & cy8c24894_debug_mask) \
			printk(level message , ##__VA_ARGS__); \
} while (0)


/* Set PSOC PWR line to ON (1) or OFF (0) */
void _set_psoc_state(struct cy8c24894_device_state* state, int on)
{
	if(on != 0 && on != 1)
		return;

	if(on) {
		gpio_set_value(state->plat_data->pwr_gpio, 1);

		if(!state->plat_data->lvl_shift_pu_fix && state->plat_data->lvl_shift_gpio)
		{
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE,
					 KERN_ERR, "%s: No PU fix for PSoC: enable level xlator early.\n", __func__);
			gpio_set_value(state->plat_data->lvl_shift_gpio, 1);
		}
	}
	else {
		if(state->plat_data->lvl_shift_gpio)
		{
			gpio_set_value(state->plat_data->lvl_shift_gpio, 0);
		}

		gpio_set_value(state->plat_data->pwr_gpio, 0);
	}
	
}

void _toggle_MWTP(struct cy8c24894_device_state* state)
{
	/* assert wakeup signal... */	
	gpio_set_value(state->plat_data->wake_tp_gpio, 1);
	CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR, "wake_tp_gpio -> 1\n");
	/* ...and de-assert (PSoC is edge trigerred)*/
	gpio_set_value(state->plat_data->wake_tp_gpio, 0);
	CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR, "wake_tp_gpio -> 0\n");
}

void prepare_for_wait(struct cy8c24894_device_state* state, enum cy8c24894_power_state new_state)
{
	int ret_val;

	/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
	ret_val = test_and_set_bit(COMPLETION_PENDING_BIT, state->flags);
	ASSERT(!ret_val);

	state->new_mode = new_state;
}

int wait_cmd_done(struct cy8c24894_device_state* state, uint32_t timeout)
{
	int rc = 0;

	/* wait for TP_IRQ toggle indicating cmd completion */
	timeout = wait_for_completion_timeout(&state->tp_irq_complete, timeout);
	if (!timeout) {
		printk(KERN_ERR "*** %s: Failed to complete PSoC command.\n"
				"    PSoC non-responsive. ***\n", __func__);
		rc = -1;
		clear_bit(COMPLETION_PENDING_BIT, state->flags);
		goto err0;
	}

err0:
		return rc;
}

int to_cmd_intf_enabled(struct cy8c24894_device_state* state)
{
	int rc = 0;
	long timeout;
	bool power_cycled = false;
	int32_t retry_count = 0;

	/* acquire exclusive access to bus (single-master system) */
	mutex_lock_nested(&state->i2c_dev->adapter->bus_lock, state->i2c_dev->adapter->level);

	if ((CY8C2489_DEEP_SLEEP_STATE == state->current_mode) &&
		(state->plat_data->poweroff_mode == CY8C2489_POWER_OFF_PSOC)) {
		printk(KERN_ERR "%s: powering-up TP.\n", CY8C24894_DRIVER);
		power_cycled = true;

		if (!state->plat_data->lvl_shift_pu_fix) {
			/* unboard master off i2c bus */
			state->plat_data->debus();
		}

		/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
		prepare_for_wait(state, CY8C2489_IDLE_NO_CMD_STATE);

		_set_psoc_state(state, 1);
		#if defined PROFILE_USAGE
		start_time = jiffies;
		reset_active = true;
		#endif

		/* wait for TP_IRQ notification indicating power-up sequence complete */
		while (wait_cmd_done(state, HZ/2)) {
			printk(KERN_ERR "*** %s: Failed to reset PSoC.\n", __func__);

			if (retry_count >= 20) {
				break;
			}
			retry_count++;
			printk(KERN_ERR "*** %s: Re-trying PSoC reset. Retry count: %d\n", __func__, retry_count);
			printk(KERN_ERR "%s: powering-down PSoC.\n", __func__);
			_set_psoc_state(state, 0);
			msleep(10);
			/* prepare for completion */
			prepare_for_wait(state, CY8C2489_IDLE_NO_CMD_STATE);
			printk(KERN_ERR "%s: powering-up PSoC.\n", __func__);
			_set_psoc_state(state, 1);
		}

		if (!state->plat_data->lvl_shift_pu_fix) {
			/* re-board master on i2c bus */
			state->plat_data->embus();
		}
		else {
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: PSoC boot sequence complete. enable level xlator.\n", __func__);
			gpio_set_value(state->plat_data->lvl_shift_gpio, 1);
		}
	}

	mutex_lock(&state->mode_mutex);

	/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
	prepare_for_wait(state, CY8C2489_IDLE_STATE);
	/* MWTP toggle */
	_toggle_MWTP(state);
	
	/* current state: wot? */
	if (CY8C2489_WOT_STATE == state->current_mode ||
		   CY8C2489_WOT_RAW_DATA_STATE == state->current_mode) {
		int32_t sleep_ms;
		/* implicit transition to active-scan */
		state->current_mode = CY8C2489_ACTIVE_SCAN_STATE;
		state->i2c_ctl_regs.mode = mode_map[state->current_mode];
		
		CPUFREQ_HOLD_CHECK(&state->cpufreq_hold);

		/* pending touch in window between ioctl processing start and _toggle_MWTP?
		   cancel; as the intent is to transition out of WOT */
		mutex_unlock(&state->mode_mutex);
		cancel_work_sync(&state->tp_irq_work);
		mutex_lock(&state->mode_mutex);

		/* needs extra toggle to transtion from active-scan -> idle */
		/* delay computed from scan rate */
		sleep_ms = 1000/state->i2c_ctl_regs.active_scan_rate;
		CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
				 "%s: sleeping for %d ms\n", __func__, sleep_ms);
		msleep(sleep_ms);
		_toggle_MWTP(state);
	}
	mutex_unlock(&state->mode_mutex);

	/* wait for TP_IRQ toggle indicating cmd interface available */
	timeout = wait_for_completion_timeout(&state->tp_irq_complete, 3*HZ);
	if (!timeout) {
		printk(KERN_ERR "*** %s: Failed to enable PSoC command interface.\n"
				"    PSoC non-responsive. ***\n", __func__);
		rc = -1;
		clear_bit(COMPLETION_PENDING_BIT, state->flags);
		/* release exclusive access to bus */
		mutex_unlock(&state->i2c_dev->adapter->bus_lock);
		goto err0;
	}

	/* release exclusive access to bus */
	mutex_unlock(&state->i2c_dev->adapter->bus_lock);

	if (true == power_cycled) {
		enum cy8c24894_power_state power_state;

		/* read-in register values */
		if (cy8c24894_get_power_state(state, &power_state)) {
			printk(KERN_ERR "%s: Failed to read start mode.\n", __func__);
		}

		if (CY8C2489_IDLE_STATE != power_state) {
			printk(KERN_ERR "%s: Expected start mode: %d but device in mode: %d\n",
			       __func__, CY8C2489_IDLE_STATE, power_state);
		}

		/* set defaults */
		if (_set_psoc_defaults(state)) {
			printk(KERN_ERR "%s: Failed to set psoc defaults.\n", __func__);
		}
	}

err0:
	return rc;
}

int wait_cmd_intf_disabled(struct cy8c24894_device_state* state)
{
	int rc = 0;
	long timeout;
	
	/* wait for TP_IRQ toggle indicating cmd interface un-available */
	timeout = wait_for_completion_timeout(&state->tp_irq_complete, 6*HZ);
	if (!timeout) {
		printk(KERN_ERR "*** %s: Failed to disable PSoC command interface.\n"
				"    PSoC non-responsive. ***\n"
				"curr_state: %x, new_state: %x, compl. bit: %s",
    __func__, state->current_mode, state->new_mode, test_bit(COMPLETION_PENDING_BIT, state->flags) ? "set" : "clear");
		rc = -1;
		clear_bit(COMPLETION_PENDING_BIT, state->flags);
		goto err0;
	}

err0:
	return rc;
}

void _reset_psoc(struct cy8c24894_device_state* state)
{
	enum cy8c24894_power_state power_state;

	printk(KERN_ERR "Resetting PSOC\n");

	/* acquire exclusive access to bus (single-master system) */
	mutex_lock_nested(&state->i2c_dev->adapter->bus_lock, state->i2c_dev->adapter->level);

	if (!state->plat_data->lvl_shift_pu_fix) {
		/* unboard master off i2c bus */
		state->plat_data->debus();
	}

	_set_psoc_state(state, 0);
	msleep(5);

	/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
	prepare_for_wait(state, CY8C2489_IDLE_NO_CMD_STATE);

	_set_psoc_state(state, 1);
#if defined PROFILE_USAGE
	start_time = jiffies;
	reset_active = true;
#endif

	/* wait for TP_IRQ notification indicating power-up sequence complete */
	if (wait_cmd_done(state, 5*HZ)) {
		printk(KERN_ERR "*** %s: Failed to reset PSoC.\n", __func__);
	}

	if (!state->plat_data->lvl_shift_pu_fix) {
		/* re-board master on i2c bus */
		state->plat_data->embus();
	}
	else {
		CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
				 "%s: PSoC boot sequence complete. enable level xlator.\n", __func__);
		gpio_set_value(state->plat_data->lvl_shift_gpio, 1);
	}

	/* release exclusive access to bus */
	mutex_unlock(&state->i2c_dev->adapter->bus_lock);

	/* enable PSoC i2c interface */
	if (to_cmd_intf_enabled(state)) {
		printk(KERN_ERR "*** %s: Failed waiting for command interface active.\n",
		       __func__);
	}

	/* read-in register values */
	if (cy8c24894_get_power_state(state, &power_state)) {
		printk(KERN_ERR "%s: Failed to read start mode.\n", __func__);
	}

	if (CY8C2489_IDLE_STATE != power_state) {
		printk(KERN_ERR "%s: Expected start mode: %d but device in mode: %d\n",
		       __func__, CY8C2489_IDLE_STATE, power_state);
	}

	/* set defaults */
	if (_set_psoc_defaults(state)) {
		printk(KERN_ERR "%s: Failed to set psoc defaults.\n", __func__);
	}

	return;
}

int32_t _set_psoc_defaults(struct cy8c24894_device_state* state)
{
	int32_t rc = 0;

#if 0
	// Set the default baud rate to 1228800bps.
	state->i2c_ctl_regs.tx8_cfg = 1;
	rc = cy8c24894_write_i2c_ctl_regs(state->i2c_dev);
	if (rc) {
		printk(KERN_ERR "%s cy8c24894_write_i2c_ctl_regs failed."
		       " err: %08x\n", __func__, rc);
	}
#endif	
	return rc;
}


void display_i2c_ctl_reg_cached_values(struct cy8c24894_device_state* state)
{
	printk(KERN_ERR "\n**** TP PSoC reg values: ****\n");
	printk(KERN_ERR "fw version: 0x%04x\n", state->i2c_ctl_regs.fw_version);
	printk(KERN_ERR "active scan rate: %d\n", state->i2c_ctl_regs.active_scan_rate);
	printk(KERN_ERR "wot scan rate: %d\n", state->i2c_ctl_regs.wot_scan_rate);
	printk(KERN_ERR "tx8 config: 0x%02x\n", state->i2c_ctl_regs.tx8_cfg);
	printk(KERN_ERR "command: %d\n", state->i2c_ctl_regs.command);
	printk(KERN_ERR "panel id: 0x%02x\n", state->i2c_ctl_regs.panel_id);
	printk(KERN_ERR "wot threshold: %d\n", state->i2c_ctl_regs.wot_threshold);
	printk(KERN_ERR "mode: 0x%04x\n\n", state->i2c_ctl_regs.mode);
}


int cy8c24894_read_i2c_ctl_regs(struct i2c_client* client) 
{ 
	int32_t ret = 0; 
	struct i2c_msg msgs[2];
	struct cy8c24894_device_state* state = (struct cy8c24894_device_state*)i2c_get_clientdata(client);
	
	
	msgs[0].addr = client->addr; 
	msgs[0].len = sizeof(state->i2c_ctl_regs);
	msgs[0].flags = I2C_M_RD; 
	msgs[0].buf = ((uint8_t*)&(state->i2c_ctl_regs)); 
	
	ret = i2c_transfer(client->adapter, msgs, 1); 	
	if (ret < 0) {
		printk(KERN_ERR "%s: err code: %d\n", __func__, ret);
		goto err0;
	}
	else {
		ret = 0;
	}
	
	// swap multi-byte values...
	state->i2c_ctl_regs.fw_version = (state->i2c_ctl_regs.fw_version >> 8) | 
		(state->i2c_ctl_regs.fw_version << 8);
	
	state->i2c_ctl_regs.mode = (state->i2c_ctl_regs.mode >> 8) | 
		(state->i2c_ctl_regs.mode << 8);
	
	if (mode_map[state->current_mode] != state->i2c_ctl_regs.mode) {
		printk(KERN_ERR "[ALERT!!!] %s: state-mismatch between driver and psoc.\n"
				"Resetting driver state.", __func__);
		/* reset driver state: PSoC state must be IDLE for us to have read ctl_regs.
		   unbreaks bootloading over factory-installed test fw which has zeroed-out
		   regs */
		state->current_mode = CY8C2489_IDLE_STATE;
		state->i2c_ctl_regs.mode = mode_map[state->current_mode];
	}
	
	display_i2c_ctl_reg_cached_values(state);
	
err0:
	return ret; 
} 

int cy8c24894_write_i2c_ctl_regs(struct i2c_client* client) 
{ 
	int32_t ret = 0;
	struct i2c_msg msg[1]; 
	struct i2c_ctl_reg_values swb_reg_values;
	struct cy8c24894_device_state* state = (struct cy8c24894_device_state*)i2c_get_clientdata(client);
	
	msg[0].addr = client->addr; 
	msg[0].flags = 0; 
	msg[0].len = sizeof(swb_reg_values);
	msg[0].buf = (uint8_t*)&swb_reg_values; 
	
	// init write record: swap multi-byte values...
	swb_reg_values.fw_version = (state->i2c_ctl_regs.fw_version >> 8) |
		(state->i2c_ctl_regs.fw_version << 8);
	swb_reg_values.active_scan_rate = state->i2c_ctl_regs.active_scan_rate;
	swb_reg_values.wot_scan_rate = state->i2c_ctl_regs.wot_scan_rate;
	swb_reg_values.tx8_cfg = state->i2c_ctl_regs.tx8_cfg;
	swb_reg_values.panel_id = state->i2c_ctl_regs.panel_id;
	swb_reg_values.wot_threshold = state->i2c_ctl_regs.wot_threshold;
	swb_reg_values.wot_baseline_hi = state->i2c_ctl_regs.wot_baseline_hi;
	swb_reg_values.wot_baseline_lo = state->i2c_ctl_regs.wot_baseline_lo;
	swb_reg_values.command = state->i2c_ctl_regs.command;
	swb_reg_values.mode = state->i2c_ctl_regs.mode >> 8 |
			state->i2c_ctl_regs.mode << 8;
	//swb_reg_values.reserved_4 = state->i2c_ctl_regs.reserved_4;
	//swb_reg_values.reserved_5 = state->i2c_ctl_regs.reserved_5;
	
	ret = i2c_transfer(client->adapter, msg, 1); 
	if (ret < 0) {
		goto err0;
	}
	else {
		ret = 0;
	}
	
err0:
	return ret; 
} 


static ssize_t cy8c24894_read(struct file *file, char __user *buf, size_t count, loff_t *ppos )
{
	ssize_t rc = 0;
	//struct cy8c24894_device_state* state = file->private_data;
	
	return rc;
}

static unsigned int cy8c24894_poll(struct file* file, struct poll_table_struct* wait)
{
	//unsigned long flags;
	unsigned int  mask = 0;
	//struct cy8c24894_device_state* state = file->private_data;
			
	return mask;
}


int cy8c24894_get_power_state(struct cy8c24894_device_state* state, enum cy8c24894_power_state* power_state)
{
	int32_t rc = 0;
	int32_t index = SIZE_CY8C2489_POWER_STATE;
	uint16_t mode = 0xffff;

	if (CY8C2489_IDLE_STATE != state->current_mode) {	
		rc = to_cmd_intf_enabled(state);
		if (rc) {
			printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
			goto err0;
		}
	}

	rc = cy8c24894_read_i2c_ctl_regs(state->i2c_dev);
	if (rc) {
		printk(KERN_ERR "%s: cy8c24894_read_i2c_ctl_regs failed.\n", __func__);
		goto err0;
	}
	
	while (--index >= 0) {
		if (mode_map[index] == state->i2c_ctl_regs.mode) {
			mode = index;
			break;
		}
	}
	
	if (0xffff == mode) {
		printk(KERN_ERR "%s: invalid mode value.\n", __func__);
		rc = -EINVAL; 
		goto err0;
	}
	
	*power_state = mode;

err0:
	return rc;	
}

int cy8c24894_set_power_state(struct cy8c24894_device_state* state, enum cy8c24894_power_state power_state)
{
	int32_t rc = 0;
	uint16_t curr_reg_val, new_reg_val;
	
	/* valid state? */
	if ((power_state < CY8C2489_DEEP_SLEEP_STATE) ||
	    (power_state >= SIZE_CY8C2489_POWER_STATE)) {
		    rc = -EINVAL;
		    printk(KERN_ERR "%s: invalid power state: %d\n", __func__, power_state);
		    goto err0; 
	    }
	
	/* we can optimize that... */
	if (state->current_mode == power_state) {
		goto err0;
	}
	
	/* tell cpufreq to hold at max freq while in active scan mode */
	/*if (power_state == CY8C2489_ACTIVE_SCAN_STATE) {
		if (!state->cpufreq_hold) {
			CPUFREQ_HOLD();
			state->cpufreq_hold = 1;
		}
	} else {
		if (state->cpufreq_hold) {
			CPUFREQ_UNHOLD();
			state->cpufreq_hold = 0;
		}
	}*/

	/* map power_state to corresponding reg value... */
	new_reg_val = mode_map[power_state];
#ifdef DEEP_SLEEP_TRANSITION_HACK
	switch (power_state) {
		case CY8C2489_DEEP_SLEEP_STATE:
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: entering DEEP_SLEEP_STATE (mode -> %d)\n", __func__, new_reg_val);
			if(state->plat_data->poweroff_mode == CY8C2489_POWER_OFF_PSOC) {
				printk(KERN_ERR "%s: powering-down TP.\n", CY8C24894_DRIVER);
				if (CY8C2489_ACTIVE_SCAN_STATE == state->current_mode) {
					CPUFREQ_UNHOLD_CHECK(&state->cpufreq_hold);
				}

				/* acquire exclusive access to bus (single-master system) */
				mutex_lock_nested(&state->i2c_dev->adapter->bus_lock, state->i2c_dev->adapter->level);

				if (!state->plat_data->lvl_shift_pu_fix) {
					/* unboard master off i2c bus */
					state->plat_data->debus();
				}

				_set_psoc_state(state, 0);
				/* circuit stabilization delay */
				msleep(5);
				state->i2c_ctl_regs.mode = new_reg_val;
				state->current_mode = CY8C2489_DEEP_SLEEP_STATE;

				if (!state->plat_data->lvl_shift_pu_fix) {
					/* re-board master on i2c bus */
					state->plat_data->embus();
				}

				/* release exclusive access to bus */
				mutex_unlock(&state->i2c_dev->adapter->bus_lock);
				goto err0;
			}
			break;
		case CY8C2489_WOT_STATE:
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: entering WOT_STATE (mode -> %d)\n", __func__, new_reg_val);
			break;
		case CY8C2489_ACTIVE_SCAN_STATE:
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: entering ACTIVE_SCAN_STATE (mode -> %d)\n", __func__, new_reg_val);
			break;
		case CY8C2489_HW_RESET_STATE:
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: entering HW_RESET_STATE (mode -> %d)\n", __func__, new_reg_val);
			break;
		case CY8C2489_IDLE_STATE:
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: entering IDLE_STATE (mode -> %d)\n", __func__, new_reg_val);
			break;
		case CY8C2489_WOT_RAW_DATA_STATE:
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: entering WOT_RAW_DATA_STATE (mode -> %d)\n", __func__, new_reg_val);
			break;
		case CY8C2489_IDLE_NO_CMD_STATE:
			CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
					 "%s: entering CY8C2489_IDLE_NO_CMD_STATE (mode -> %d)\n"
					 "Not a valid state transition!!\n", __func__, new_reg_val);
			break;
		default:
			break;
	}

#endif
	if (CY8C2489_IDLE_STATE != state->current_mode) {	
		rc = to_cmd_intf_enabled(state);
		if (rc) {
			printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
			goto err0;
		}
	}

	/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
	/* we are now in "idle"; a command to switch to "idle" at this point will not result
	   in TP_IRQ toggle - hence skip the prepare for this case */
	if (CY8C2489_IDLE_STATE != power_state) {
		prepare_for_wait(state, CY8C2489_HW_RESET_STATE == power_state ?
			CY8C2489_IDLE_NO_CMD_STATE : power_state);
	}

	curr_reg_val = state->i2c_ctl_regs.mode;
	state->i2c_ctl_regs.mode = new_reg_val;
	rc = cy8c24894_write_i2c_ctl_regs(state->i2c_dev);
	if (rc) {
		state->i2c_ctl_regs.mode = curr_reg_val;
		clear_bit(COMPLETION_PENDING_BIT, state->flags);
		printk(KERN_ERR "%s cy8c24894_write_i2c_ctl_regs failed."
				" err: %08x\n", __func__, rc);
	}
	else {
		/* stash new power_state in global context... */
		if (CY8C2489_HW_RESET_STATE != power_state) {
			/* transitioning to state that disables the i2c cmd interface? */
			if (CY8C2489_IDLE_STATE != power_state) {
				rc = wait_cmd_intf_disabled(state);
				if (rc) {
					printk(KERN_ERR "%s (%s;%d) failed. Err: %d\n", __func__, __FILE__, __LINE__, rc);
					goto err0;
				}
			}
		}
		else {
			/* hw-reset: 
			   * wait for power-up sequence to complete
			   * bringup cmd interface on PSoC
			   * re-cache default values from PSoC 
			   * re-set psoc default values */

			/* wait for power-up completion */
			rc = wait_cmd_done(state, 5*HZ);
			if (!rc) {
				rc = to_cmd_intf_enabled(state);
			}
			if (rc) {
				printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
				goto err0;
			}
			/* re-read default register values */
			rc = cy8c24894_read_i2c_ctl_regs(state->i2c_dev);
			if (rc) {
				printk(KERN_ERR "%s cy8c24894_read_i2c_ctl_regs failed for hw-reset."
						" err: %08x\n", __func__, rc);
			}
			else {
				rc = _set_psoc_defaults(state);
			}
		}
	}

err0:
	return rc;
}

int cy8c24894_set_command(struct cy8c24894_device_state* state, enum cy8c24894_command command)
{
	int32_t rc = 0;

	if (CY8C2489_IDLE_STATE != state->current_mode) {	
		rc = to_cmd_intf_enabled(state);
		if (rc) {
			printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
			goto err0;
		}
	}

	/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
	if (CY8C2489_IDAC_CALIBRATE == command ||
		   CY8C2489_WRITE_IDAC_FLASH == command) {
		prepare_for_wait(state, CY8C2489_IDLE_NO_CMD_STATE);
	}

	state->i2c_ctl_regs.command = command;
	rc = cy8c24894_write_i2c_ctl_regs(state->i2c_dev);
	if (rc) {
		printk(KERN_ERR "%s cy8c24894_write_i2c_ctl_regs failed."
				" err: %08x\n", __func__, rc);
		clear_bit(COMPLETION_PENDING_BIT, state->flags);
	}
	// we're done: reset command
	state->i2c_ctl_regs.command = 0;

	if (CY8C2489_IDAC_CALIBRATE == command ||
		   CY8C2489_WRITE_IDAC_FLASH == command) {
		
		rc = wait_cmd_intf_disabled(state);
		if (!rc) {
			/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
			prepare_for_wait(state, CY8C2489_IDLE_NO_CMD_STATE);

			rc = wait_cmd_done(state, 5*HZ);
			if (!rc) {
				rc = to_cmd_intf_enabled(state);
			}
		}
		if (rc) {
			printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
		}
	}

err0:
	return rc;
}

uint8_t _convert_hex_char_to_decimal(uint8_t x)
{
	x -= '0';
	if (x > 9) {
		x = x - ('A' - ('9' + 1));
		if (x > 15) {
			x = x - ('a' - 'A');
		}
	}
	
	return x;
}

int read_single_ctl_reg_value(struct cy8c24894_device_state* state, void* dst, void* src, uint32_t read_size)
{
	int rc = 0;

	ASSERT(read_size <= sizeof(uint16_t));

	if (CY8C2489_IDLE_STATE != state->current_mode) {
		rc = to_cmd_intf_enabled(state);
		if (rc) {
			printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
			goto err0;
		}
	}

	rc = cy8c24894_read_i2c_ctl_regs(state->i2c_dev);
	if (rc) {
		printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_SCANRATE]: failed."
				" err: %d\n", __func__, rc);
		goto err0;
	}

	if (copy_to_user(dst, src, read_size)) {
		    rc =  -EFAULT; 
		    goto err0;
	}

err0:
	return rc;
}

int write_single_ctl_reg_value(struct cy8c24894_device_state* state, void* dst, void* src, uint32_t write_size)
{
	int rc = 0;
	uint32_t old_val;
	
	ASSERT(write_size <= sizeof(uint16_t));

	if (CY8C2489_IDLE_STATE != state->current_mode) {
		rc = to_cmd_intf_enabled(state);
		if (rc) {
			printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
			goto err0;
		}
	}

	memcpy(&old_val, dst, write_size);
	if (copy_from_user(dst, src, write_size)) {
		rc = -EFAULT;
		goto err0;
	}

	rc = cy8c24894_write_i2c_ctl_regs(state->i2c_dev);
	if (rc) {
		memcpy(dst, &old_val, write_size);
		printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
		goto err0;
	}

err0:
	return rc;
}

static int cy8c24894_ioctl(struct inode * inode, struct file *file,
                         unsigned int cmd, unsigned long args)
{
	int32_t rc = 0;
	struct cy8c24894_device_state* state = file->private_data;
	void* usr_ptr   = (void*)args;
	uint32_t usr_bytes = _IOC_SIZE(cmd);
	uint32_t usr_val = 0x0;
	
	CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR, "%s: cmd: %d, num bytes: %d, value: %d\n",
	       __func__, _IOC_NR(cmd), usr_bytes,
		usr_bytes ? rc = copy_from_user(&usr_val, usr_ptr, usr_bytes), usr_val: 0);
	
	rc = mutex_lock_interruptible(&state->dev_mutex);
	if (rc) {
		printk(KERN_ERR "%s: mutex_lock interrupted(1)\n", __func__);
		return -ERESTARTSYS;
	}
	
	while (test_and_set_bit(DEVICE_BUSY_BIT, state->flags)) {
		/* i2c-bootload sequence is multiple-entry:
		   bootload in progress skips busy_bit check */
		if (test_bit(BOOTLOAD_ACTIVE_BIT, state->flags) && CY8C2489_IOCTL_SET_FW_DATA == cmd) {
			break;
		}

		mutex_unlock(&state->dev_mutex);

		CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
				 "%s: about to wait for device non-busy...\n", __func__);

		// busy bit set? wait to be cleared (at least 3 seconds: in jiffies)
		rc = wait_event_interruptible_timeout(state->dev_busyq, 
		                                      !test_bit(DEVICE_BUSY_BIT, state->flags), 3*HZ);
		if (!rc) {
			printk(KERN_ERR "%s: wait on device busy timed-out/interrupted\n", __func__);
			// reset busy state
			clear_bit(DEVICE_BUSY_BIT, state->flags);
			// and continue...
		}

		rc = mutex_lock_interruptible(&state->dev_mutex);
		if (rc) {
			printk(KERN_ERR "%s: mutex_lock interrupted(2)\n", __func__);
			return -ERESTARTSYS;
		}
	}

	switch (cmd) {
		case CY8C2489_IOCTL_GET_SCANRATE: 
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.active_scan_rate,
     					min(usr_bytes, sizeof(state->i2c_ctl_regs.active_scan_rate)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_SCANRATE]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		} 
		break;
		
		case CY8C2489_IOCTL_GET_WOT_SCANRATE: 
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.wot_scan_rate,
					min(usr_bytes, sizeof(state->i2c_ctl_regs.wot_scan_rate)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_WOT_SCANRATE]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		}
		break;
			
		case CY8C2489_IOCTL_GET_SLEEPMODE: 
		{
			enum cy8c24894_power_state power_state;
			int32_t rc = 0;
			
			rc = cy8c24894_get_power_state(state, &power_state);
			if (rc) {
				goto Done;
			}
			
			if (copy_to_user(usr_ptr, &power_state, 
				                 min(usr_bytes, sizeof(power_state)))) {
					                 rc = -EFAULT; 
					                 goto Done;
			}
		}
		break;
			
		case CY8C2489_IOCTL_GET_FW_VERSION: 
		case CY8C2489_IOCTL_GET_QUERY_DATA:
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.fw_version,
					min(usr_bytes, sizeof(state->i2c_ctl_regs.fw_version)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_FW_VERSION[QUERY_DATA]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		}
		break;
		
		case CY8C2489_IOCTL_GET_TX8_CONFIG: 
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.tx8_cfg,
					min(usr_bytes, sizeof(state->i2c_ctl_regs.tx8_cfg)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_TX8_CONFIG]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		}
		break;
				
		case CY8C2489_IOCTL_GET_PANEL_ID:
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.panel_id,
					min(usr_bytes, sizeof(state->i2c_ctl_regs.panel_id)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_PANEL_ID]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		}
		break;
		
		case CY8C2489_IOCTL_GET_WOT_THRESHOLD:
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.wot_threshold,
					min(usr_bytes, sizeof(state->i2c_ctl_regs.wot_threshold)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_WOT_THRESHOLD]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		}
		break;
		
		case CY8C2489_IOCTL_GET_WOT_BASELINE_HI:
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.wot_baseline_hi,
					min(usr_bytes, sizeof(state->i2c_ctl_regs.wot_baseline_hi)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_WOT_BASELINE_HI]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		}
		break;

		case CY8C2489_IOCTL_GET_WOT_BASELINE_LO:
		{
			rc = read_single_ctl_reg_value(state, usr_ptr,
					&state->i2c_ctl_regs.wot_baseline_lo,
					min(usr_bytes, sizeof(state->i2c_ctl_regs.wot_baseline_lo)));
			if (rc) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_GET_WOT_BASELINE_LO]: failed."
						" err: %d\n", __func__, rc);
				goto Done;
			}
		}
		break;

		case CY8C2489_IOCTL_GET_NUM_DATA_BYTES: 
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_GET_NUM_DATA_BYTES]\n", __func__);
		} 
		break;
		
		case CY8C2489_IOCTL_GET_VERBOSE_MODE: 
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_GET_NUM_DATA_BYTES]\n", __func__);
		} 
		break;
			
		case CY8C2489_IOCTL_GET_TIMESTAMP_MODE: 
		{
			if (copy_to_user(usr_ptr, &state->timestamping, 
			                 min(usr_bytes, sizeof(state->timestamping)))) { 
				                 rc = -EFAULT; 
				                 goto Done;
			                 }
		} 
		break;

		case CY8C2489_IOCTL_SET_SCANRATE:
		{
			rc = write_single_ctl_reg_value(state, &state->i2c_ctl_regs.active_scan_rate, 
					usr_ptr, sizeof(uint8_t));
			if (rc < 0) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_SCANRATE]: failed."
						" err: %0d\n", __func__, rc);
				goto Done;
			}
		}
		break;
		
		case CY8C2489_IOCTL_SET_WOT_SCANRATE: 
		{
			rc = write_single_ctl_reg_value(state, &state->i2c_ctl_regs.wot_scan_rate,
					usr_ptr, sizeof(uint8_t));
			if (rc < 0) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_WOT_SCANRATE]: failed."
						" err: %0d\n", __func__, rc);
				goto Done;
			}
		} 
		break;
					
		case CY8C2489_IOCTL_SET_SLEEPMODE:
		{
			uint16_t power_state;
			
			if (copy_from_user(&power_state, usr_ptr, min(sizeof(power_state), usr_bytes))) {
				rc = -EFAULT;
				goto Done;
			}
									
			rc = cy8c24894_set_power_state(state, power_state);
			if (rc < 0) {
				goto Done;
			}					
		} 
		break;
		
		case CY8C2489_IOCTL_SET_TX8_CONFIG:
		{
			rc = write_single_ctl_reg_value(state, &state->i2c_ctl_regs.tx8_cfg,
					usr_ptr, sizeof(uint8_t));
			if (rc < 0) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_TX8_CONFIG]: failed."
						" err: %0d\n", __func__, rc);
				goto Done;
			}
		}
		break;
		
		case CY8C2489_IOCTL_SET_COMMAND:
		{
			uint8_t command;
			
			if (copy_from_user(&command, usr_ptr, min(sizeof(command), usr_bytes))) {
				rc = -EFAULT;
				goto Done;
			}
			
			rc = cy8c24894_set_command(state, command);
			if (rc < 0) {
				goto Done;
			}
		} 
		break;
		
		case CY8C2489_IOCTL_SET_WOT_THRESHOLD:
		{
			rc = write_single_ctl_reg_value(state, &state->i2c_ctl_regs.wot_threshold,
					usr_ptr, sizeof(uint8_t));
			if (rc < 0) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_WOT_THRESHOLD]: failed."
						" err: %0d\n", __func__, rc);
				goto Done;
			}
		}
		break;
		
		case CY8C2489_IOCTL_SET_WOT_BASELINE_HI:
		{
			rc = write_single_ctl_reg_value(state, &state->i2c_ctl_regs.wot_baseline_hi,
					usr_ptr, sizeof(uint8_t));
			if (rc < 0) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_WOT_BASELINE_HI]: failed."
						" err: %0d\n", __func__, rc);
				goto Done;
			}
		}
		break;

		case CY8C2489_IOCTL_SET_WOT_BASELINE_LO:
		{
			rc = write_single_ctl_reg_value(state, &state->i2c_ctl_regs.wot_baseline_lo,
					usr_ptr, sizeof(uint8_t));
			if (rc < 0) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_WOT_BASELINE_LO]: failed."
						" err: %0d\n", __func__, rc);
				goto Done;
			}
		}
		break;

		case CY8C2489_IOCTL_SET_NUM_DATA_BYTES:
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_SET_NUM_DATA_BYTES]\n", __func__);
		}
		break;

		case CY8C2489_IOCTL_SET_VERBOSE_MODE:
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_SET_VERBOSE_MODE]\n", __func__);
		}
		break;

		case CY8C2489_IOCTL_SET_TIMESTAMP_MODE:
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_SET_TIMESTAMP_MODE]\n", __func__);
		}
		break;

		case CY8C2489_IOCTL_SET_BYTES:
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_SET_BYTES]\n", __func__);
		}
		break;

		case CY8C2489_IOCTL_SET_VECTORS:
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_SET_VECTORS]\n", __func__);
		}
		break;
			
		case CY8C2489_IOCTL_SET_FW_DATA:
		{
			struct i2c_msg msg[6]; 
			uint16_t payload_size = 0, converted_size, msg_index;
			uint8_t *payload, *src_data, *tgt_data;
			uint8_t status = 0;
			int32_t ret_val;
			bool bootload_exit_rec = false;
			
			
			if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
				ret_val = test_and_set_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
				ASSERT(!ret_val);
			}
			
			if (copy_from_user(&payload_size, usr_ptr, sizeof(payload_size))) {
				rc = -EFAULT;
				goto Done;
			}
						
			// protocol constraint...
			if (payload_size > BOOTLOADER_PROTOCOL_BLOCK_SIZE) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_FW_DATA]"
						" payload size > BOOTLOADER_PROTOCOL_BLOCK_SIZE\n", __func__);
				rc = -EINVAL;
				goto Done;
			}
			
			payload = kmalloc(payload_size + 0x0a, GFP_KERNEL);
			if (!payload) {
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_FW_DATA]: out of memory!\n", __func__);
				rc = -ENOMEM;
				goto Done;
			}
			
			if (copy_from_user(payload + 0x02, (uint8_t*)usr_ptr + 2, payload_size)) {
				kfree(payload);
				rc = -EFAULT;
				goto Done;
			}
			
			/* bringup the i2c cmd interface on the PSoC */
			if (CY8C2489_IDLE_STATE != state->current_mode) {
				rc = to_cmd_intf_enabled(state);
				if (rc) {
					printk(KERN_ERR "%s failed. Err: %d\n", __func__, rc);
					kfree(payload);
					goto Done;
				}
			}

			msg_index = 0;
			converted_size = 0;
			tgt_data = payload;
			src_data = payload + 0x02;
			
			while (converted_size < payload_size) {
				if (!(converted_size & 0x1f)) {
					msg[msg_index].buf = tgt_data;
					if (msg_index) {
						msg[msg_index - 1].len = tgt_data - msg[msg_index - 1].buf;
						msg[msg_index - 1].flags = 0;
						msg[msg_index - 1].addr = (0x24>>1);
					}
					msg_index++;
					
					tgt_data[0] = 0xcc;
					tgt_data[1] = 0xcc; // stuff i2c control bytes (ignored)...
					tgt_data += 2;
				}
				
				tgt_data[0] = (_convert_hex_char_to_decimal(src_data[0]) << 4) |
					_convert_hex_char_to_decimal(src_data[1]);
				src_data += 2;
				tgt_data++;
				converted_size += 2;
			}
			
			msg[msg_index - 1].len = tgt_data - msg[msg_index - 1].buf;
			msg[msg_index - 1].flags = 0;
			msg[msg_index - 1].addr = (0x24>>1);

			// skip status read if exit bootloader command... 
			status = 0x20;
			if (0xff == msg[0].buf[2] && 0x3b == msg[0].buf[3]) {
				bootload_exit_rec = true;
			}

			rc = i2c_transfer(state->i2c_dev->adapter, msg, msg_index); 
			if (rc < 0) {
				kfree(payload);
				printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_FW_DATA]: i2c_transfer failed."
						" err: %08x\n", __func__, rc);
				goto Done;
			}

			kfree(payload);

			if (false == bootload_exit_rec) {
				/* worst-case overhead for flash write */
				msleep(100);

				/* read status for record-set now */
				msg[0].addr = (0x24>>1);
				msg[0].flags = I2C_M_RD;
				msg[0].buf = &status;
				msg[0].len = 1;

				rc = i2c_transfer(state->i2c_dev->adapter, msg, 1);
				if (rc < 0) {
					printk(KERN_ERR "%s [CY8C2489_IOCTL_SET_FW_DATA]: i2c_transfer failed."
							" err: %08x\n", __func__, rc);
					goto Done;
				}
			}

			/* reset our return code */
			rc = 0;

			/* bootload failure? */
			if (0x20 != status) {
				printk(KERN_ERR "cy8c24894 bootload failure. err code: 0x%02x\n", status);
				rc = -EINVAL;
				goto Done;
			}
			else if (true == bootload_exit_rec) {
				enum cy8c24894_power_state power_state;

				/* acquire exclusive access to bus (single-master system) */
				mutex_lock_nested(&state->i2c_dev->adapter->bus_lock, state->i2c_dev->adapter->level);

				if (!state->plat_data->lvl_shift_pu_fix) {
					/* unboard master off i2c bus */
					state->plat_data->debus();
				}
				else {
					CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
							"%s: PSoC boot sequence starting post-fw-bootload."
							" disable level xlator.\n",
					       __func__);
					gpio_set_value(state->plat_data->lvl_shift_gpio, 0);
				}


				/* prepare for completion */
				prepare_for_wait(state, CY8C2489_IDLE_NO_CMD_STATE);
				
				clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
				printk(KERN_ERR "** cy8c24894 bootload successful! **\n");

				/* wait for TP_IRQ notification indicating power-up sequence complete */
				if (wait_cmd_done(state, 5*HZ)) {
					printk(KERN_ERR "*** %s: Failed waiting for command done after bootload.\n",
					       __func__);
				}

				if (!state->plat_data->lvl_shift_pu_fix) {
					/* re-board master on i2c bus */
					state->plat_data->embus();
				}
				else {
					CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
							"%s: PSoC boot sequence complete."
							" enable level xlator.\n",
					       __func__);
					gpio_set_value(state->plat_data->lvl_shift_gpio, 1);
				}

				/* release exclusive access to bus */
				mutex_unlock(&state->i2c_dev->adapter->bus_lock);

				/* enable PSoC i2c interface */
				if (to_cmd_intf_enabled(state)) {
					printk(KERN_ERR "*** %s: Failed waiting for command interface active"
							" after bootload.\n", __func__);
				}

				/* read-in register values */
				if (cy8c24894_get_power_state(state, &power_state)) {
					printk(KERN_ERR "%s: Failed to read start mode.\n", __func__);
				}

				if (CY8C2489_IDLE_STATE != power_state) {
					printk(KERN_ERR "%s: Expected start mode: %d but device in mode: %d\n",
					       __func__, CY8C2489_IDLE_STATE, power_state);
				}

				/* set defaults */
				if (_set_psoc_defaults(state)) {
					printk(KERN_ERR "%s: Failed to set psoc defaults.\n", __func__);
				}
			}
		}
		break;
	
		case CY8C2489_IOCTL_SET_PROG_PHASE:
		{
			printk(KERN_ERR "%s: Unimplemented ioctl->[CY8C2489_IOCTL_SET_PROG_PHASE]\n", __func__);
		}
		break;

		case CY8C2489_IOCTL_RESET_PSOC:
		{
			_reset_psoc(state);
		} 
		break;

		default:
		{
			rc = -EINVAL;
		} 
		break;
	}
Done:
	if (rc) {
		if (test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			clear_bit(BOOTLOAD_ACTIVE_BIT, state->flags);
		}
		clear_bit(DEVICE_BUSY_BIT, state->flags);
	}
	else {
		if (!test_bit(BOOTLOAD_ACTIVE_BIT, state->flags)) {
			clear_bit(DEVICE_BUSY_BIT, state->flags);
		}
	}
	
	mutex_unlock(&state->dev_mutex);
	wake_up_interruptible(&state->dev_busyq);
	
	return rc;
}

static int cy8c24894_open(struct inode *inode, struct file *file)
{
	struct cy8c24894_device_state* state;
	
	/* get device */
	state = container_of(file->f_op, struct cy8c24894_device_state, fops);
	
	/* Allow only read. */
	if ((file->f_mode & (FMODE_READ|FMODE_WRITE)) != FMODE_READ) {
		    return -EINVAL;
	}
	
	/* check if it is in use */
	if (test_and_set_bit(IS_OPENED, state->flags)) {
		return -EBUSY;
	}
	
	/* attach private data */
	file->private_data = state;
	
	return 0; 
}

static int cy8c24894_close(struct inode *inode, struct file *file)
{
	struct cy8c24894_device_state* state = (struct  cy8c24894_device_state*) file->private_data;
	
	/* mark it as unused */
	clear_bit(IS_OPENED, state->flags);
	
	if (CY8C2489_ACTIVE_SCAN_STATE == state->current_mode) {
		CPUFREQ_UNHOLD_CHECK(&state->cpufreq_hold);
	}

	return 0;
}



struct file_operations cy8c24894_fops = {
	.owner   = THIS_MODULE,
	.read    = cy8c24894_read,
	.poll    = cy8c24894_poll,
	.ioctl   = cy8c24894_ioctl,
	.open    = cy8c24894_open,
	.release = cy8c24894_close,
};

/*
 *  wot interrupt handler
 */
static irqreturn_t cy8c24894_wot_irq(int irq, void *dev_id)
{
	struct cy8c24894_device_state* state = (struct cy8c24894_device_state *)dev_id;
	int rc;

	cy8c24894_tp_irq_count++;
#if defined PROFILE_USAGE
	if (true == reset_active) {
		diff_time = (long)jiffies - (long)start_time;
		reset_active = false;
		CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR, "TP_IRQ toggle post power-cycle after: %d ms\n",
				 diff_time*1000/HZ);
	}
#endif

	CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR, "%s: entry.\n", __func__);

	rc = queue_work(ktpd_workqueue, &state->tp_irq_work);
	if (!rc) {
		/* possible in one scenario: touch-irq comes in while we are about to
		   transition from "wot" to "idle" mode (by toggling MWTP) in order to
		   send an i2c command (potentially to transition PSoC to "deep-sleep"
		   as part of device suspend): in this case we ignore the error and let
		   the pending work-item be treated as the response to our "idle" transition. */
		printk(KERN_ERR "**** %s: failed queueing work item. current_mode: %x, new_mode: %x****)\n",
		       __func__, state->current_mode, state->new_mode);
	}
	return IRQ_HANDLED;
}

void tp_irq_work_handler(struct work_struct *work)
{
	struct cy8c24894_device_state* state =
			container_of(work, struct cy8c24894_device_state, tp_irq_work);
	uint16_t old_mode;

	// give cpufreq_ondemand a hint that the user is about to interact with the UI
	//CPUFREQ_TICKLE();

	mutex_lock(&state->mode_mutex);

	if (state->current_mode != CY8C2489_ACTIVE_SCAN_STATE && state->new_mode == CY8C2489_ACTIVE_SCAN_STATE) {
		CPUFREQ_HOLD_CHECK(&state->cpufreq_hold);
	}
	else if (state->current_mode == CY8C2489_ACTIVE_SCAN_STATE && state->new_mode != CY8C2489_ACTIVE_SCAN_STATE
		   && state->new_mode != 0xffff) {
		/* performance hack to allow some extra time at max freq. this gets removed when userspace
		 * is able to indicate what performance it wants */
		if (state->cpufreq_hold)
			CPUFREQ_TICKLE();
		CPUFREQ_UNHOLD_CHECK(&state->cpufreq_hold);
	}

	old_mode = state->current_mode;
	if (state->new_mode != 0xffff) {
		state->current_mode = state->new_mode;
	}
	state->i2c_ctl_regs.mode = mode_map[CY8C2489_IDLE_NO_CMD_STATE == state->current_mode ?
			CY8C2489_IDLE_STATE : state->current_mode];
	state->new_mode = 0xffff;

	/* optimize touch while in WOT by returning quickly... */
	if (CY8C2489_WOT_STATE == old_mode) {
		CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
				 "%s: internal transition -> ACTIVE_SCAN_STATE (mode -> %d)\n",
				 __func__, CY8C2489_ACTIVE_SCAN_STATE);
		mutex_unlock(&state->mode_mutex);
		return;
	}
	else {
		/* if transitioning to WOT, set new_mode -> active_scan
		   for implicit transition on touch */
		if (CY8C2489_WOT_STATE == state->current_mode) {
			state->new_mode = CY8C2489_ACTIVE_SCAN_STATE;
		}
	}
	mutex_unlock(&state->mode_mutex);
	
	if (test_bit(COMPLETION_PENDING_BIT, state->flags)) {
		CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
				 "%s: about to complete task pending on TP_IRQ.\n", __func__);
		clear_bit(COMPLETION_PENDING_BIT, state->flags);
		complete(&state->tp_irq_complete);
	}
	else {
		printk(KERN_ERR "**** %s: [protocol error] unexpected TP_IRQ toggle, old_mode %d, current_mode %d .****\n",
		       __func__, old_mode, state->current_mode);
	}
}

/****************************************************************************** 
* cy8c24894_i2c_probe()
******************************************************************************/ 
static int cy8c24894_i2c_probe(struct i2c_client *client) 
{ 
	int32_t rc;
	struct cy8c24894_device_state* state;
	struct cy8c24894_platform_data* plat_data = client->dev.platform_data;
	enum cy8c24894_power_state power_state;
	
	
	if (plat_data == NULL) 
		return (-ENODEV);
	
	state = kzalloc(sizeof(struct cy8c24894_device_state), GFP_KERNEL);
	if(!state)
		return (-ENOMEM);
	
	// store i2c client device in state
	state->i2c_dev = client;
		
	// set platform data in device-specific driver data
	state->plat_data = plat_data;

	// power-on state is "idle_no_cmd"...
	state->current_mode = state->new_mode = CY8C2489_IDLE_NO_CMD_STATE;

	mutex_init(&state->dev_mutex);
	mutex_init(&state->mode_mutex);

	// zero-init wait flags
	bitmap_zero(state->flags, SIZE_FLAGS);
	
	// waitq used when psoc executes commands that are exclusive: 
	// idac calibration, reset, etc.
	init_waitqueue_head(&state->dev_busyq);

	INIT_WORK(&state->tp_irq_work, tp_irq_work_handler);

	init_completion(&state->tp_irq_complete);

	// set device-specific driver data 
	i2c_set_clientdata(client, state);

	rc = request_irq(gpio_to_irq(plat_data->wot_gpio), cy8c24894_wot_irq,
			IRQF_TRIGGER_RISING,
			"cy8c24894_ts", state);
	CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
			 "%s: after request_irq for irq# %d (gpio# %d)\n",
			 __func__, gpio_to_irq(plat_data->wot_gpio), plat_data->wot_gpio);
	if (rc != 0) {
		printk(KERN_ERR "%s: Failed to get IRQ.\n", CY8C24894_DRIVER);
		goto err0;
	}

	/* power-down the PSoC to ensure quiescent state */
	_set_psoc_state(state, 0);
	/* hold time required for circuit */
	msleep(5);
	/* flush pending work items (basically any queued TP_IRQ handler) */
	flush_workqueue(ktpd_workqueue);

	/* acquire exclusive access to bus (single-master system) */
	mutex_lock_nested(&state->i2c_dev->adapter->bus_lock, state->i2c_dev->adapter->level);

	if (!state->plat_data->lvl_shift_pu_fix) {
		/* unboard master off i2c bus */
		state->plat_data->debus();
	}

	printk(KERN_ERR "%s: Resetting PSoC!\n", CY8C24894_DRIVER);
	/* prepare for completion _before_ triggering event that causes the TP_IRQ toggle */
	prepare_for_wait(state, CY8C2489_IDLE_NO_CMD_STATE);

	/* power-up the PSoC */
	_set_psoc_state(state, 1);
	
	/* wait for TP_IRQ notification indicating power-up sequence complete */
	rc = wait_cmd_done(state, 5*HZ);
	if (rc) {
		printk(KERN_ERR "*** %s: Failed to reset PSoC at boot.\n", CY8C24894_DRIVER);
	}

	if (!state->plat_data->lvl_shift_pu_fix) {
		/* re-board master on i2c bus */
		state->plat_data->embus();
	}
	else {
		CY8C2489_DPRINTK(CY8C2489_DEBUG_VERBOSE, KERN_ERR,
				 "%s: PSoC boot sequence complete. enable level xlator.\n", __func__);
		gpio_set_value(state->plat_data->lvl_shift_gpio, 1);
	}

	/* release exclusive access to bus */
	mutex_unlock(&state->i2c_dev->adapter->bus_lock);

	/* enable PSoC i2c interface */
	rc = to_cmd_intf_enabled(state);
	if (rc) {
		printk(KERN_ERR "*** %s: Failed waiting for command interface active at boot.\n"
				"Maybe a pre-rev700 fw version\n", CY8C24894_DRIVER);
		/* backward compat: figure out if pre-rev700 version fw by switching to
		   "idle" on timeout */
		state->current_mode = CY8C2489_IDLE_STATE;
	}

	/* retrieve start mode (assumption: this should also cache all ctl reg values because of the
	   nature of the implementation of the ctl regs) */
	rc = cy8c24894_get_power_state(state, &power_state);
	if (rc) {
		printk(KERN_ERR "%s: Failed to read start mode.\n", CY8C24894_DRIVER);
	}

	/* let the driver initialize fully, independent of state-mismatches or current-state
 	   retrieval errors. this allows a bootload to be initiated unconditionally which is 
	   important as a fallback mechanism. */
	if (CY8C2489_IDLE_STATE != power_state) {
		printk(KERN_ERR "%s: Expected start mode: %d but device in mode: %d\n",
				CY8C24894_DRIVER, CY8C2489_IDLE_STATE, power_state);
		// force sane state to enable bootload operation...
		state->current_mode = CY8C2489_IDLE_STATE;
	}

	/* set psoc default values... */
	_set_psoc_defaults(state);

	/* register as misc device */
	memcpy(&state->fops, &cy8c24894_fops, sizeof(struct file_operations));
	state->mdev.minor = MISC_DYNAMIC_MINOR;
	state->mdev.name = "touchscreen";
	state->mdev.fops = &state->fops;
	rc = misc_register(&state->mdev);
	if (rc < 0) {
		printk(KERN_ERR "%s: Failed to register as misc device\n", CY8C24894_DRIVER);
		goto err0;
	}

	printk(KERN_ERR "Touchpanel driver initialized successfully!\n");
	return 0;

err0:
	kfree(state);
	return -ENODEV;
}

/****************************************************************************** 
* cy8c24894_i2c_remove() 
******************************************************************************/ 
static int cy8c24894_i2c_remove(struct i2c_client *client) 
{ 
	return 0;
}

#ifdef CONFIG_PM
/******************************************************************************
* cy8c24894_i2c_suspend 
******************************************************************************/
static int cy8c24894_i2c_suspend(struct i2c_client *dev, pm_message_t event)
{
	return 0;
}

/******************************************************************************
* cy8c24894_i2c_resume 
******************************************************************************/
static int cy8c24894_i2c_resume (struct i2c_client *dev)
{
	return 0;
}
#else
#define cy8c24894_i2c_suspend  NULL
#define cy8c24894_i2c_resume   NULL
#endif  /* CONFIG_PM */

static struct i2c_driver cy8c24894_i2c_driver = { 
	.driver = { 
		.name = CY8C24894_DRIVER, 
	}, 
		.probe		= cy8c24894_i2c_probe, 
		.remove 	= __devexit_p(cy8c24894_i2c_remove), 
		.suspend 	= cy8c24894_i2c_suspend, 
		.resume 	= cy8c24894_i2c_resume, 
}; 

/*********************************************************************************
* cy8c24894_module_init(void)
***********************************************************************************/
static int __init cy8c24894_module_init(void)
{
	ktpd_workqueue = create_workqueue("ktpd");
	if (!ktpd_workqueue) {
		printk(KERN_ERR "%s: Failed to create kbtpd workqueue.\n", CY8C24894_DRIVER);
		return -EINVAL;
	}

	return i2c_add_driver(&cy8c24894_i2c_driver);
}

/*********************************************************************************
* cy8c24894_module_exit(void)
***********************************************************************************/
static void __exit cy8c24894_module_exit(void)
{
	i2c_del_driver(&cy8c24894_i2c_driver); 

	if (ktpd_workqueue) {
		destroy_workqueue(ktpd_workqueue);
	}
}

module_init(cy8c24894_module_init);
module_exit(cy8c24894_module_exit);

MODULE_DESCRIPTION("CY8C24894 touchpanel driver");
MODULE_LICENSE("GPL");
