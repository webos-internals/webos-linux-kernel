/* 
 * linux/drivers/input/kxte9_accelerometer.c 
 * 
 * Driver for the kxte9 accelerometer. 
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
 */ 

#include <linux/init.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <asm/arch/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>	
#include <linux/types.h> 
#include <linux/platform_device.h> 
#include <linux/i2c.h> 
#include <asm/irq.h> 
#include <linux/workqueue.h> 
#include <asm/mach-types.h> 
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <asm/arch/hardware.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/mach-types.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c_kxte9_accelerometer.h>

struct kxte9_device_state {
	struct i2c_client *i2c_dev;
	struct kxte9_platform_data *pdata;
	struct timer_list kxte9_timer;
	struct input_dev  *inp_dev;
	struct work_struct read_work;
	struct mutex fs_lock;
	struct mutex chip_lock;
	u16    poll_interval;
	int    suspended;	/* flag to signal suspend resume */
	int    stopping;	/* set if stop operation is in progress */
	u8     mode;	        /* select off or interrupt mode  */
	atomic_t opencount;     /* open count */
};

/****************************************************************************** 
* 
* kxte9_i2c_read_u8 
* 
* Inputs 
* struct i2c_client* client,  u8 index, u8 num, u8* out    
*
* Returns 
* 0 on success  or non-zero on fail
* 
******************************************************************************/ 
int 
kxte9_i2c_read_u8(struct i2c_client* client, u8 index, u8* out) 
{ 
	int ret; 
	struct i2c_msg msgs[2];

	msgs[0].addr = client->addr; 
	msgs[0].len = 1; 
	msgs[0].flags = 0; 
	msgs[0].buf = &index; 
 
	msgs[1].addr = client->addr; 
	msgs[1].len = 1; 
	msgs[1].flags = I2C_M_RD; 
	msgs[1].buf = out; 
 
	ret = i2c_transfer(client->adapter, msgs, 2); 
 
	return (ret); 
} 

/*************************************************************************** 
* 
* kxte9_i2c_write_u8 
*
* Write the requested register with the given value
* 
* Inputs 
* struct i2c_client* client, u8 index, u8 value
* 
* Returns
* 0 on success  or non-zero on fail
* 
***************************************************************************/ 
int 
kxte9_i2c_write_u8(struct i2c_client* client, u8 index, u8 value) 
{ 
	u8 buf[2] = {index, value}; 
	int ret; 
	struct i2c_msg msg[1]; 
	 
	msg[0].addr = client->addr; 
	msg[0].flags = 0; 
	msg[0].len = 2; 
	msg[0].buf = buf; 
 
	ret = i2c_transfer(client->adapter, msg, 1); 
	return ret; 
} 

/****************************************************************************** 
* 
* kxte9_enable_timer 
*
* enable and disable kernel timer for accelerometer
* 
* Inputs 
*  struct accelerometer_state *state, int enable
* 
* Returns
* None
* 
******************************************************************************/ 
static void 
kxte9_enable_timer(struct kxte9_device_state *state, int enable)
{
	if (enable)
		mod_timer(&state->kxte9_timer,
		           jiffies + (HZ * state->poll_interval)/1000);
	else
		del_timer_sync(&state->kxte9_timer);
}

/****************************************************************************** 
* 
* kxte9_thresh_g_to_count(u16 out) 
* 
* Convert the threshold from G into threshold counts
* Inputs 
*   u8 out 
* 
* Returns 
*   int. 
* 
******************************************************************************/ 
static u8 kxte9_thresh_g_to_count(u16 out)
{
	u8 count;
	count = (((out * 16)/10000)<<2);
	return count;
}

/****************************************************************************** 
* 
* kxte9_enable_interrupt 
* 
* Enable or disable the high g motion interrupt
*  
* Inputs 
*   struct i2c_client* client,u8 flag 
* 
* Returns 
*   None. 
* 
******************************************************************************/ 
static void 
kxte9_enable_interrupt(struct i2c_client *client, u8 enable)
{
	int rc;
	u8  val;
	struct kxte9_device_state *state = i2c_get_clientdata(client);
	
	if(enable)
	{
		// Clear the interrupt
		rc = kxte9_i2c_read_u8(client, INT_REL, &val);
		
		// Enable the interrupt
		rc = kxte9_i2c_write_u8(client, INT_CTRL_REG1, ENABLE_INTERRUPT);

		// Read the control register values and preserve only the ODR data
		rc = kxte9_i2c_read_u8(client, CTRL_REG1, &val);
		val &= ODR;

		// un-mask interrupt
		switch (state->mode) {
			case MODE_INT_AXIS:
				// Unmask xzy position state
				rc = kxte9_i2c_write_u8(client, INT_CTRL_REG2, XYZ_UNMASK);
				rc = kxte9_i2c_write_u8(client, CTRL_REG2, TILT_UNMASK);

				// Set to operating mode if it is in standby mode before
				rc = kxte9_i2c_write_u8(client, CTRL_REG1, (val|OPERATING_MODE));
				break;
			case MODE_INT_TILT:
				// Unmask tilt position state
				rc = kxte9_i2c_write_u8(client, INT_CTRL_REG2, XYZ_UNMASK);
				rc = kxte9_i2c_write_u8(client, CTRL_REG2, TILT_MASK);

				// Set to operating mode if it is in standby mode before
				rc = kxte9_i2c_write_u8(client, CTRL_REG1, (val|OPERATING_MODE|TPE_ENABLE));
				break;
			case MODE_INT_ALL: 
				// Unmask both tilt and xyzs
				rc = kxte9_i2c_write_u8(client, INT_CTRL_REG2, XYZ_UNMASK);
				rc = kxte9_i2c_write_u8(client, CTRL_REG2, TILT_MASK);

				// Set to operating mode if it is in standby mode before
				rc = kxte9_i2c_write_u8(client, CTRL_REG1, (val|OPERATING_MODE|TPE_ENABLE));
				break;
			case MODE_OFF:
			default:
				break;
		}
	}
	else
	{
		// Clear the interrupt
		rc = kxte9_i2c_read_u8(client, INT_REL, &val);

		// Disable interrupt
		rc = kxte9_i2c_read_u8(client, INT_CTRL_REG1, &val);
		rc = kxte9_i2c_write_u8(client, INT_CTRL_REG1, (val & DISABLE_INTERRUPT));
	}
}

/*****************************************************************************
*
* kxte9_isr
*
* Interrupt service routine to be called when accelerometer
* is being programmed for high g motion interrupt
* and  schedules the work to be done in bottom half
*
* Inputs int irq, void *dev_id 
*    
*  Returns IRQ_HANDLED
******************************************************************************/
static irqreturn_t
kxte9_isr(int irq, void *dev_id)
{
	struct kxte9_device_state *state = dev_id;
	schedule_work(&state->read_work); 
	return IRQ_HANDLED;
}

/****************************************************************************** 
* 
* kxte9_timeout() 
* 
* Poll the accelerometer for X,Y, Z coordinate at user requested timer interval
*
* Inputs 
* unsigned long statePtr
* 
* Returns 
* None
* 
******************************************************************************/ 
static void 
kxte9_timeout(unsigned long param )
{
	struct kxte9_device_state *state = (struct kxte9_device_state *)param;

	/* Queue the work */
	schedule_work( &state->read_work ); 
}

/****************************************************************************** 
* 
* kxte9_configure
* 
* Configure the accelerometer with the default values.
*  
* Inputs 
*   struct i2c_client *dev 
* 
* Returns 
*   None. 
* 
******************************************************************************/ 
static void 
kxte9_configure(struct i2c_client *dev)
{
	int rc;
	u8 temp;
	u8 thresh;	
	struct kxte9_device_state *state = i2c_get_clientdata(dev);

	// Configure the main ODR
	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG1, &temp);
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG1, ( (temp & 0xE7) | (state->pdata->odr_main << 3) ));

	// Configure the WUF ODR
	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG3, &temp);
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG3, ( (temp & 0xFC) | (state->pdata->odr_wuf) ));

	// Configure the B2S ODR
	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG3, &temp);
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG3, ( (temp & 0xF3) | (state->pdata->odr_b2s << 2) ));

	// Configure the tilt timer
	rc = kxte9_i2c_write_u8(state->i2c_dev, TILT_TIMER, state->pdata->tilt_timer);

	// Configure the wuf timer
	rc = kxte9_i2c_write_u8(state->i2c_dev, WUF_TIMER, state->pdata->wuf_timer);
	
	// Configure the b2s timer
	rc = kxte9_i2c_write_u8(state->i2c_dev, B2S_TIMER, state->pdata->b2s_timer);


	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_UNLOCK);

	// Configure wuf thresh
	thresh = kxte9_thresh_g_to_count(state->pdata->wuf_thresh);

	rc = kxte9_i2c_write_u8(state->i2c_dev, WUF_THRESH, thresh);

	// Configure b2s thresh
	thresh = kxte9_thresh_g_to_count(state->pdata->b2s_thresh);

	rc = kxte9_i2c_write_u8(state->i2c_dev, B2S_THRESH, thresh);

	// Configure the tilt angle
	temp = (state->pdata->tilt_thresh * 8055) / 9000;

	rc = kxte9_i2c_write_u8(state->i2c_dev, TILT_LOW_LIMIT_REG, temp);

	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_LOCK);

}

/****************************************************************************** 
* 
* kxte9_stop 
* 
* Stop the accelerometer running in any of mode 
*  
* Inputs 
*   struct i2c_client *dev 
* 
* Returns 
*   None. 
* 
******************************************************************************/ 
static void 
kxte9_stop(struct i2c_client *dev)
{
	struct kxte9_device_state *state = i2c_get_clientdata(dev);

	mutex_lock(&state->chip_lock);
	state->stopping = 1; // set stopping
	kxte9_enable_interrupt(state->i2c_dev, 0);
	kxte9_enable_timer(state, 0);
	mutex_unlock(&state->chip_lock);
	cancel_work_sync(&state->read_work); 
	state->stopping = 0; // stopped
}

/****************************************************************************** 
* 
* kxte9_start_nolock
* 
* Start the accelerometer with the select mode 
*  
* Inputs 
*   struct i2c_client *dev
* 
* Returns 
*   None. 
* 
******************************************************************************/ 
static void 
kxte9_start_nolock(struct i2c_client *dev) 
{
	struct kxte9_device_state *state = i2c_get_clientdata(dev);

	if (!atomic_read(&state->opencount))
		return;

	switch (state->mode) {
		case MODE_INT_ALL:
			kxte9_configure (state->i2c_dev);
			kxte9_enable_interrupt(state->i2c_dev, 1);
			kxte9_enable_timer(state, 1);
			break;
		case MODE_INT_AXIS: 
			kxte9_configure (state->i2c_dev);
			kxte9_enable_interrupt(state->i2c_dev, 0);
			kxte9_enable_timer(state, 1);
			break;
		case MODE_INT_TILT:
			kxte9_configure (state->i2c_dev);
			kxte9_enable_interrupt(state->i2c_dev, 1);
			kxte9_enable_timer(state, 0);
			break;
		case MODE_OFF:
			kxte9_enable_interrupt(state->i2c_dev, 0);
			kxte9_enable_timer(state, 0);
			break;
		default:
			break;
	}
}

/****************************************************************************** 
* 
*  show_mode
* 
* This function read the mode in which the accelerometer is working
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns
* 
******************************************************************************/ 
static ssize_t
show_mode(struct device *dev,
          struct device_attribute *attr,
          char *buf)
{
	ssize_t count = 0;
	char *mode = NULL;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	switch (state->mode)
	{
		case MODE_INT_ALL:
			mode = "all";
			break;
		case MODE_OFF:
			mode = "off";
			break;
		case MODE_INT_AXIS:
			mode = "axis";
			break;
		case MODE_INT_TILT:
			mode = "tilt";
			break;
		default:
			mode = "undefined";
			break;
		
	}
	count = snprintf(buf, PAGE_SIZE, "%s\n", mode);

	return (count+1);
}

/****************************************************************************** 
* 
* set_mode
* 
* This function set the mode of the accelerometer
*
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns count byte received
* 
******************************************************************************/ 
static ssize_t 
set_mode(struct device *dev, 
         struct device_attribute *attr, 
         const char *buf, size_t count)
{
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	mutex_lock(&state->fs_lock);
	kxte9_stop(state->i2c_dev); /*stop the previous mode*/
	mutex_lock(&state->chip_lock);
	if( count >= 3 && strncmp( buf, "all", 3) == 0 ) {
		state->mode = MODE_INT_ALL; 
	} else if( count >= 3 && strncmp( buf, "off", 3) == 0 ) {
		state->mode = MODE_OFF;
	} else if( count >= 4  && strncmp( buf, "tilt", 4) == 0 ) {
		state->mode = MODE_INT_TILT;
	} else if( count >= 4 && strncmp( buf, "axis", 4) == 0 ) {
		state->mode = MODE_INT_AXIS;
	}
	kxte9_start_nolock(state->i2c_dev); /*start in the new mode*/
	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);
	
	return count;
}

/****************************************************************************** 
* 
*  show_poll_interval
* 
* This function read the accelerometer polling interval
*
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns polling interval value,
* 
******************************************************************************/ 
static ssize_t 
show_poll_interval(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;
	return snprintf(buf, PAGE_SIZE, "%u\n", state->poll_interval);
}

/****************************************************************************** 
* 
* set_poll_interval
* 
* This function set the accelerometer polling interval
*
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns count byte received
* 
******************************************************************************/ 
static ssize_t 
set_poll_interval(struct device *dev,
                  struct device_attribute *attr, 
                  const char *buf, size_t count)
{
	u16 value;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;
	if (!buf || !count) 
		return 0; 
	value = simple_strtoul(buf, NULL, 10);
	state->poll_interval = value;
	return count;
}

/****************************************************************************** 
* 
* show_kxte9_wuf_thresh 
* 
* This function read the accelerometer motion wake up threshold
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_wuf_thresh(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->wuf_thresh);

	return (count);
}

/****************************************************************************** 
* 
* set_kxte9_wuf_thresh
* 
* This function set the accelerometer motion wake up threshold
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_wuf_thresh(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;
	u8 wuf_thresh;	
	int rc;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	state->pdata->wuf_thresh = simple_strtoul(buf, &endp, 10);

	state->pdata->wuf_thresh = (state->pdata->wuf_thresh > WUF_THRESH_MAX_VALUE) ? 
								WUF_THRESH_MAX_VALUE :
								state->pdata->wuf_thresh;

	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_UNLOCK);

	wuf_thresh = kxte9_thresh_g_to_count(state->pdata->wuf_thresh);

	rc = kxte9_i2c_write_u8(state->i2c_dev, WUF_THRESH, wuf_thresh);

	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_LOCK);
	
	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}

/****************************************************************************** 
* 
* show_b2s_thresh 
* 
* This function read the accelerometer motion wake up threshold
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_b2s_thresh(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->b2s_thresh);

	return (count);
	
}

/****************************************************************************** 
* 
* set_b2s_thresh
* 
* This function set the accelerometer motion wake up threshold
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_b2s_thresh(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;
	u8 b2s_thresh;
	int rc;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	
	state->pdata->b2s_thresh = simple_strtoul(buf, &endp, 10);

	state->pdata->b2s_thresh = (state->pdata->b2s_thresh > B2S_THRESH_MAX_VALUE) ?
									B2S_THRESH_MAX_VALUE :
									state->pdata->b2s_thresh;

	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_UNLOCK);
	
	b2s_thresh = kxte9_thresh_g_to_count(state->pdata->b2s_thresh);

	rc = kxte9_i2c_write_u8(state->i2c_dev, B2S_THRESH, b2s_thresh);

	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_LOCK);
	
	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}

/****************************************************************************** 
* 
* show_kxte9_wuf_timer 
* 
* This function read the kxte9 wake up timer
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_wuf_timer(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->wuf_timer);

	return (count);
}

/****************************************************************************** 
* 
* set_kxte9_wuf_timer
* 
* This function set the kxte9 wake up timer
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_wuf_timer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;
	u8 wuf_counts;
	int rc; 
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;
	
	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);

	state->pdata->wuf_timer = simple_strtoul(buf, &endp, 10);

	state->pdata->wuf_timer = (state->pdata->wuf_timer > WUF_TIMER_MAX_VALUE) ?
								WUF_TIMER_MAX_VALUE :
								state->pdata->wuf_timer;

	wuf_counts = (u8) state->pdata->wuf_timer;

	rc = kxte9_i2c_write_u8(state->i2c_dev, WUF_TIMER, wuf_counts);

	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}

/****************************************************************************** 
* 
* show_b2s_timer 
* 
* This function read the accelerometer motion wake up threshold
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_b2s_timer(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->b2s_timer);

	return (count);
	
}

/****************************************************************************** 
* 
* set_b2s_timer
* 
* This function set the accelerometer motion wake up threshold
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_b2s_timer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;
	u8 b2s_counts;
	int rc;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	
	state->pdata->b2s_timer = simple_strtoul(buf, &endp, 10);
	state->pdata->b2s_timer = (state->pdata->b2s_timer > B2S_TIMER_MAX_VALUE) ?
								B2S_TIMER_MAX_VALUE :
								state->pdata->b2s_timer;

	b2s_counts = (u8)state->pdata->b2s_timer;

	rc = kxte9_i2c_write_u8(state->i2c_dev, B2S_TIMER, b2s_counts);

	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);
	return count;
}


/****************************************************************************** 
* 
* show_odr_main 
* 
* This function reads the accelerometer main output data rate
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_odr_main (struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->odr_main);

	return (count);
	
}

/****************************************************************************** 
* 
* set_odr_main
* 
* This function sets the accelerometer main output data rate
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_odr_main(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	int rc;
	char *endp;
	u8 temp;
	struct kxte9_device_state *state;

	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	
	
	state->pdata->odr_main = simple_strtoul(buf, &endp, 10);

	if (state->pdata->odr_main > ODR_40HZ) {
		state->pdata->odr_main = ODR_40HZ;
	}

	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG1, &temp);
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG1, ( (temp & 0xE7) | (state->pdata->odr_main << 3) ));

	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}




/****************************************************************************** 
* 
* show_odr_b2s 
* 
* This function reads the accelerometer b2s output data rate
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_odr_b2s (struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->odr_b2s);

	return (count);
	
}

/****************************************************************************** 
* 
* set_odr_b2s
* 
* This function sets the accelerometer b2s output data rate
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_odr_b2s (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	int rc;
	char *endp;
	u8 temp;
	struct kxte9_device_state *state;

	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	
	
	state->pdata->odr_b2s = simple_strtoul(buf, &endp, 10);

	if (state->pdata->odr_b2s > ODR_40HZ) {
		state->pdata->odr_b2s = ODR_40HZ;
	}

	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG3, &temp);
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG3, ( (temp & 0xF3) | (state->pdata->odr_b2s << 2) ));

	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}



/****************************************************************************** 
* 
* show_odr_wuf 
* 
* This function reads the accelerometer wuf output data rate
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_odr_wuf (struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->odr_wuf);

	return (count);
	
}

/****************************************************************************** 
* 
* set_odr_wuf
* 
* This function sets the accelerometer wuf output data rate
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_odr_wuf (struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	int rc;
	char *endp;
	u8 temp;
	struct kxte9_device_state *state;

	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	
	state->pdata->odr_wuf = simple_strtoul(buf, &endp, 10);

	if (state->pdata->odr_wuf > ODR_40HZ) {
		state->pdata->odr_wuf = ODR_40HZ;
	}

	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG3, &temp);
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG3, ( (temp & 0xFC) | (state->pdata->odr_wuf) ));

	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}








/****************************************************************************** 
* 
* show_tilt_timer 
* 
* This function read the kxte9 tilt timer
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_tilt_timer(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->tilt_timer);

	return (count);
	
}

/****************************************************************************** 
* 
* set_tilt_timer
* 
* This function set the kxte9 tilt timer 
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_tilt_timer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;
	int rc;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	
	state->pdata->tilt_timer = simple_strtoul(buf, &endp, 10);
	rc = kxte9_i2c_write_u8(state->i2c_dev, TILT_TIMER, state->pdata->tilt_timer);

	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}



/****************************************************************************** 
* 
* show_tilt_thresh
* 
* This function displays the current kxte9 tilt angle
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_tilt_thresh(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	if (!buf)
		return 0;
	
	count = snprintf(buf, PAGE_SIZE, "%u\n", state->pdata->tilt_thresh);

	return (count);
	
}

/****************************************************************************** 
* 
* set_tilt_thresh
* 
* This function set the kxte9 tilt angle 
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_tilt_thresh(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *endp;
	u8 temp;
	int rc;
	struct kxte9_device_state *state;
	state = (struct kxte9_device_state*)dev->driver_data;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	
	state->pdata->tilt_thresh = simple_strtoul(buf, &endp, 10);

	if (state->pdata->tilt_thresh > TILT_LOW_LIMIT_MAX) {
		state->pdata->tilt_thresh = TILT_LOW_LIMIT_MAX;
	}

	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_UNLOCK);

	temp = (state->pdata->tilt_thresh * 8055) / 9000;

	rc = kxte9_i2c_write_u8(state->i2c_dev, TILT_LOW_LIMIT_REG, temp);

	rc = kxte9_i2c_write_u8(state->i2c_dev, KXTE9_THRESH_LOCK_UNLOCK_REG, KXTE9_THRESH_LOCK);

	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);

	return count;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode, set_mode);
static DEVICE_ATTR(wuf_thresh, S_IRUGO | S_IWUSR, show_wuf_thresh, set_wuf_thresh);
static DEVICE_ATTR(b2s_thresh, S_IRUGO | S_IWUSR, show_b2s_thresh, set_b2s_thresh);
static DEVICE_ATTR(wuf_timer, S_IRUGO | S_IWUSR, show_wuf_timer, set_wuf_timer);
static DEVICE_ATTR(b2s_timer, S_IRUGO | S_IWUSR, show_b2s_timer, set_b2s_timer);
static DEVICE_ATTR(tilt_timer, S_IRUGO | S_IWUSR, show_tilt_timer, set_tilt_timer);
static DEVICE_ATTR(tilt_thresh, S_IRUGO | S_IWUSR, show_tilt_thresh, set_tilt_thresh);
static DEVICE_ATTR(odr_main, S_IRUGO | S_IWUSR, show_odr_main, set_odr_main);
static DEVICE_ATTR(odr_b2s, S_IRUGO | S_IWUSR, show_odr_b2s, set_odr_b2s);
static DEVICE_ATTR(odr_wuf, S_IRUGO | S_IWUSR, show_odr_wuf, set_odr_wuf);
static DEVICE_ATTR(poll_interval, S_IRUGO | S_IWUSR, show_poll_interval, set_poll_interval);

/****************************************************************************** 
* 
* kxte9_count_to_g(u8 out) 
* 
* Convert the X-axis/Y-axis/Z-axis acceleration into G unit, it's range
* -20000 to 20000 which map to -2g to +2g
* Inputs 
*   u8 out 
* 
* Returns 
*   int. 
* 
******************************************************************************/ 
static int kxte9_count_to_g(u8 out)
{
	int gravity;
	gravity = (10000*((out>>2)-32))/16;
	return gravity;
}

/****************************************************************************** 
* 
* kxte9_tilt_cur_pos(u8 tilt_cur) 
* 
* Convert tilt_cur register value to following position
* 	1:	Face-up
*   2:  Face-down
*   3:  Up
*   4:  Down
*   5:  Right
*   6:  Left
* Inputs 
*   u8 tilt_cur 
* 
* Returns 
*   1 to 6. 
* 
******************************************************************************/ 
static int kxte9_tilt_cur_pos(u8 tilt_cur)
{
	int pos = ORIENTATION_UNKNOWN;
	switch (tilt_cur)
	{
		case TILT_POS_FU:
			pos = ORIENTATION_FACE_UP;
			break;
		case TILT_POS_FD:
			pos = ORIENTATION_FACE_DOWN;
			break;
		case TILT_POS_UP:
			pos = ORIENTATION_UP;
			break;
   		case TILT_POS_DO:
			pos = ORIENTATION_DOWN;
			break;
		case TILT_POS_RI:
			pos = ORIENTATION_RIGHT;
			break;
		case TILT_POS_LE:
			pos = ORIENTATION_LEFT;
			break;
		default:
			break;
	}
	return pos;
}


/****************************************************************************** 
* 
* kxte9_translate_xyz (struct kxte9_device_state *state, int *x_gravity, int * y_gravity, int * z_gravity) 
* 
* Translates the the X,Y and Z axis values as per the board file
* Inputs 
*   struct work_struct *work 
* 
* Returns 
*   None. 
* 
******************************************************************************/ 
void kxte9_translate_xyz (struct kxte9_device_state *state, int *x_gravity, int * y_gravity, int * z_gravity)
{
	int temp_x = 0, temp_y = 0, temp_z = 0;

	temp_x = state->pdata->xyz_translation_map [0] * (*x_gravity) +
				state->pdata->xyz_translation_map [1] * (*y_gravity) +
				state->pdata->xyz_translation_map [2] * (*z_gravity);

	temp_y = state->pdata->xyz_translation_map [3] * (*x_gravity) +
				state->pdata->xyz_translation_map [4] * (*y_gravity) +
				state->pdata->xyz_translation_map [5] * (*z_gravity);
	
	temp_z = state->pdata->xyz_translation_map [6] * (*x_gravity) +
				state->pdata->xyz_translation_map [7] * (*y_gravity) +
				state->pdata->xyz_translation_map [8] * (*z_gravity);

	*x_gravity = temp_x;
	*y_gravity = temp_y;
	*z_gravity = temp_z;

}

/****************************************************************************** 
* 
* kxte9_read_xyz(void) 
* 
* Read the X,Y and Z-register and report the values to input subsystem 
* Inputs 
*   struct work_struct *work 
* 
* Returns 
*   None. 
* 
******************************************************************************/ 
static void 
kxte9_read_xyz(struct work_struct *work) 
{
	int rc;
	u8 val, int_src;
	int x_gravity = 0;
	int y_gravity = 0; 
	int z_gravity = 0; 
	int tilt_pos = 0;
	struct kxte9_device_state *state = container_of(work, struct kxte9_device_state, read_work);

	mutex_lock(&state->chip_lock);
	if (state->stopping) {
		mutex_unlock(&state->chip_lock);
		return;
	}

	// Report tilt position change if detect
	rc = kxte9_i2c_read_u8(state->i2c_dev, INT_SRC_REG1, &int_src);

	// Report X,Y and Z Coordinate to input subsystem
	if ((MODE_INT_AXIS == state->mode) ||
		(MODE_INT_ALL == state->mode))
	{
		//Read X,Y and Z coordinate  
		rc = kxte9_i2c_read_u8(state->i2c_dev, XOUT, &val);
		x_gravity = kxte9_count_to_g(val);

		rc = kxte9_i2c_read_u8(state->i2c_dev, YOUT, &val);
		y_gravity = kxte9_count_to_g(val);

		rc = kxte9_i2c_read_u8(state->i2c_dev, ZOUT, &val);
		z_gravity = kxte9_count_to_g(val);

		// translate the xyz gravity units per the translation map
		kxte9_translate_xyz (state, &x_gravity, &y_gravity, &z_gravity);

		input_report_abs(state->inp_dev,  ABS_X, x_gravity);
		input_report_abs(state->inp_dev,  ABS_Y, y_gravity);
		input_report_abs(state->inp_dev,  ABS_Z, z_gravity);

	}

	if (int_src&TILT_INT)
	{
		// Read Tilt Position Registers
		rc = kxte9_i2c_read_u8(state->i2c_dev, TILT_POS_CUR, &val);
		tilt_pos = kxte9_tilt_cur_pos(val);

		// Report tilt position to input subsystem
		input_report_abs(state->inp_dev, ABS_ORIENTATION, tilt_pos);
	
	}

	input_sync(state->inp_dev);

	if ((state->mode == MODE_INT_AXIS) ||
		(state->mode == MODE_INT_ALL)) {
		// In poll mode we just unconditinally set up next sample
		mod_timer(&state->kxte9_timer, jiffies  + 
				msecs_to_jiffies(state->poll_interval));
	}

	// Clear interrupt
	rc = kxte9_i2c_read_u8(state->i2c_dev, INT_REL, &val);

	mutex_unlock(&state->chip_lock);
}


/*****************************************************************************
* kxte9_inputdev_open (struct input_dev *dev)
*
* Called when the accelerometer input event is being listened
*
* Inputs   None
* 
* return error code , 0 on success, or non-zero otherwise
******************************************************************************/
static int kxte9_inputdev_open(struct input_dev *dev)
{
	struct kxte9_device_state *state = dev_get_drvdata (dev->dev.parent);

	mutex_lock(&state->fs_lock);
	if ( atomic_add_return(1, &state->opencount) == 1 ) {
		mutex_lock(&state->chip_lock);
		kxte9_start_nolock (state->i2c_dev);
		mutex_unlock(&state->chip_lock);
	}
	mutex_unlock(&state->fs_lock);

	return 0;
}


/*********************************************************************************
* kxte9_inputdev_close (struct input_dev *dev)
*
* This is the function called when the accelerometer input event is being listened
*
* Inputs   None
* 
* return error code , 0 on success, or non-zero otherwise
***********************************************************************************/
static void kxte9_inputdev_close(struct input_dev *dev)
{
	struct kxte9_device_state *state = dev_get_drvdata (dev->dev.parent);

	mutex_lock(&state->fs_lock);
	if ( atomic_sub_return(1, &state->opencount) == 0 ) {
		kxte9_stop (state->i2c_dev);
	}
	mutex_unlock(&state->fs_lock);
}

/****************************************************************************** 
* 
* kxte9_i2c_probe() 
* 
* Called by driver model to initialize device 
*  
* Inputs 
*  i2c client pointer. 
* 
* Returns 
*   0 on success, or non-zero on otherwise. 
* 
******************************************************************************/ 
static int 
kxte9_i2c_probe(struct i2c_client *client) 
{ 
	u8 val;
	int rc;
	struct kxte9_device_state *state;
	struct kxte9_platform_data *pdata = client->dev.platform_data;

	/* Sanity Check */
	rc = kxte9_i2c_read_u8(client, WHO_AM_I, &val);
	if ((rc < 0)||(val !=0x00) )
		return -ENODEV;

	/* Check the platform data. */ 
	if (pdata == NULL) 
		return (-ENODEV);
	
	state = kzalloc(sizeof(struct kxte9_device_state), GFP_KERNEL);
	if(!state)
		return (-ENOMEM);
	
	// Attach i2c_dev 
	state->i2c_dev = client;

	// Init spinlock 
	mutex_init(&state->fs_lock);
	mutex_init(&state->chip_lock);
	state->suspended = 0;

	// Attach platform data 
	state->pdata = pdata;

	// By default interrupt mode is selected
	state->mode = MODE_OFF;

	state->poll_interval = 250; // Poll XYZ every 250ms
	atomic_set(&state->opencount, 0);

	// Init Workque interface  
	INIT_WORK(&state->read_work, kxte9_read_xyz); 

	// Setup input device
	state->inp_dev = input_allocate_device();
	if( state->inp_dev == NULL ) 
		goto err0; 
	
	//Enable input event of linux input subsystem  
	set_bit(EV_ABS, state->inp_dev->evbit); 
	set_bit(ABS_X, state->inp_dev->absbit); 
	set_bit(ABS_Y, state->inp_dev->absbit); 
	set_bit(ABS_Z, state->inp_dev->absbit); 
	set_bit(ABS_ORIENTATION, state->inp_dev->absbit);
	input_set_abs_params(state->inp_dev, ABS_X, ACCEL_MIN_ABS_X, ACCEL_MAX_ABS_X, 0, SENSITIVITY_UNITS_PER_ACCELERATION_1G);
	input_set_abs_params(state->inp_dev, ABS_Y, ACCEL_MIN_ABS_Y, ACCEL_MAX_ABS_Y, 0, SENSITIVITY_UNITS_PER_ACCELERATION_1G);
	input_set_abs_params(state->inp_dev, ABS_Z, ACCEL_MIN_ABS_Z, ACCEL_MAX_ABS_Z, 0, SENSITIVITY_UNITS_PER_ACCELERATION_1G);
	input_set_abs_params(state->inp_dev, ABS_ORIENTATION, ACCEL_TILT_POS_MIN, ACCEL_TILT_POS_MAX, 0, 0);
	
	state->inp_dev->name = KXTE9_I2C_DRIVER;
	state->inp_dev->id.bustype = BUS_VIRTUAL;
	state->inp_dev->id.vendor = 0x0;
	state->inp_dev->id.product = 0x0;
	state->inp_dev->id.version = 0x100;
	state->inp_dev->dev.parent = &client->dev;

	state->inp_dev->open = kxte9_inputdev_open;
	state->inp_dev->close = kxte9_inputdev_close;

	rc = input_register_device(state->inp_dev); 
	if( rc != 0 ) 
		goto err1;
	
	dev_set_drvdata(&state->inp_dev->dev, state);

#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXTE9
	if ((rc = kxte9_dbg_register_i2c_client(state->i2c_dev)))
		goto err1;
#endif // CONFIG_ACCELEROMETER_TEST_DEBUG_KXTE9

	// Initial configuration for that 
	rc = kxte9_i2c_write_u8(client, CTRL_REG1, (OPERATING_MODE|TPE_ENABLE|ODR));
	rc = kxte9_i2c_write_u8(client, CTRL_REG2, TILT_MASK);
	rc = kxte9_i2c_write_u8(client, CTRL_REG3, (OS2SA|OS2SB|OWUFA|OWUFB));
	
	// Init interrupt
	rc = gpio_request(state->pdata->gpio, KXTE9_I2C_DRIVER);
	if( rc != 0 ) 
		goto err2; 

	gpio_direction_input(state->pdata->gpio);

	//set up a repeating timer 
	init_timer_deferrable(&state->kxte9_timer);
	state->kxte9_timer.function = kxte9_timeout;
	state->kxte9_timer.data = ( unsigned long)state;

	// Request an interrupt
	rc = request_irq(client->irq, kxte9_isr,
				IRQF_TRIGGER_FALLING, KXTE9_I2C_DRIVER, 
				(void*)state);
	if( rc < 0 ) 
		goto err3;

	// Clear the interrupt
	rc = kxte9_i2c_read_u8(client, INT_REL, &val);

	//attach driver data 
	i2c_set_clientdata(client, state);

	kxte9_enable_interrupt(state->i2c_dev, 0);

	//create device sysfs attributes  
	rc = device_create_file(&state->inp_dev->dev, &dev_attr_mode); 
	if (rc != 0) 
		goto err4; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_wuf_thresh); 
	if (rc != 0)
		goto err5; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_b2s_thresh); 
	if (rc != 0)
		goto err6; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_wuf_timer); 
	if (rc != 0)
		goto err7; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_b2s_timer); 
	if (rc != 0)
		goto err8; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_tilt_timer); 
	if (rc != 0)
		goto err9; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_odr_main); 
	if (rc != 0)
		goto err10; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_odr_b2s); 
	if (rc != 0)
		goto err11; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_odr_wuf); 
	if (rc != 0)
		goto err12; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_poll_interval); 
	if (rc != 0)
		goto err13; 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_tilt_thresh); 
	if (rc != 0)
		goto err14; 

	return 0;

err14:
	device_remove_file(&state->inp_dev->dev, &dev_attr_poll_interval); 	
err13:
	device_remove_file(&state->inp_dev->dev, &dev_attr_odr_wuf); 	
err12:
	device_remove_file(&state->inp_dev->dev, &dev_attr_odr_b2s); 	
err11:
	device_remove_file(&state->inp_dev->dev, &dev_attr_odr_main); 	
err10:
	device_remove_file(&state->inp_dev->dev, &dev_attr_tilt_timer); 	
err9:
	device_remove_file(&state->inp_dev->dev, &dev_attr_b2s_timer); 	
err8:
	device_remove_file(&state->inp_dev->dev, &dev_attr_wuf_timer); 	
err7:
	device_remove_file(&state->inp_dev->dev, &dev_attr_b2s_thresh); 	
err6:
	device_remove_file(&state->inp_dev->dev, &dev_attr_wuf_thresh); 
err5:
	device_remove_file(&state->inp_dev->dev, &dev_attr_mode); 
err4:
	free_irq(state->i2c_dev->irq, state);	
err3:
	gpio_free(state->pdata->gpio);	
err2:
	input_unregister_device ( state->inp_dev );
err1:
	input_free_device(state->inp_dev);
err0:
	kfree(state);
	return -ENODEV;
}

/****************************************************************************** 
* 
* kxte9_i2c_remove() 
*
* Remove the accelerometer driver, does whatever cleanup  is required
* 
* Inputs  
*   struct i2c_client *client. 
* 
* Returns 0
* 
******************************************************************************/ 
static int 
kxte9_i2c_remove(struct i2c_client *client) 
{ 
	struct kxte9_device_state *state = i2c_get_clientdata(client);    

	device_remove_file(&state->inp_dev->dev, &dev_attr_wuf_thresh); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_b2s_thresh); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_wuf_timer); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_b2s_timer); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_tilt_timer);	
	device_remove_file(&state->inp_dev->dev, &dev_attr_mode); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_odr_wuf); 	
	device_remove_file(&state->inp_dev->dev, &dev_attr_odr_main); 	
	device_remove_file(&state->inp_dev->dev, &dev_attr_odr_b2s);
	device_remove_file(&state->inp_dev->dev, &dev_attr_poll_interval); 	
	device_remove_file(&state->inp_dev->dev, &dev_attr_tilt_thresh);	

	kxte9_stop(client);
	
	// Unregister input device
	input_unregister_device(state->inp_dev);

#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXTE9
	kxte9_dbg_unregister_i2c_client(state->i2c_dev);
#endif // CONFIG_ACCELEROMETER_TEST_DEBUG_KXTE9

	// Unregister interrupt handlers and free gpio
	free_irq(state->i2c_dev->irq, state);
	gpio_free(state->pdata->gpio);

	// Detach state 
	i2c_set_clientdata(client, NULL);

	// Free state 
	kfree(state);
	return 0;
}

#ifdef CONFIG_PM
/******************************************************************************
* kxte9_i2c_suspend 
* 
* Called by Power management to suspend the accelerometer  
*
* Inputs 
*  struct i2c_client *dev, pm_message_t event 
* 
* Returns 
* 0
******************************************************************************/
static int 
kxte9_i2c_suspend(struct i2c_client *dev, pm_message_t event)
{
	int rc;
	u8 val;
	struct kxte9_device_state *state = i2c_get_clientdata(dev);    

	mutex_lock(&state->fs_lock);
	
	// Check if already suspended */
	if (state->suspended) 
		goto err;
		
	kxte9_stop(dev);

	// Set to standby mode
	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG1, &val); 
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG1, (val & STANDBY_MODE));

	// Mark as suspended
	state->suspended = 1;

err:
	mutex_unlock(&state->fs_lock);
	return 0;
}

/******************************************************************************
* kxte9_i2c_resume 
* 
* Called by Power management to resume the accelerometer  
*
* Inputs 
*  struct i2c_client *dev 
* 
* Returns 
* 0
******************************************************************************/
static int 
kxte9_i2c_resume ( struct i2c_client *dev )
{
	int rc;
	u8 val;
	struct kxte9_device_state *state = i2c_get_clientdata(dev);    

	mutex_lock(&state->fs_lock);

	if(!state->suspended ) 
		goto err; // already resumed
		
	// Enable accelerometer 
	mutex_lock(&state->chip_lock);
	rc = kxte9_i2c_read_u8(state->i2c_dev, CTRL_REG1, &val); 
	rc = kxte9_i2c_write_u8(state->i2c_dev, CTRL_REG1, 
			        (val |OPERATING_MODE));

	kxte9_start_nolock(dev);
	mutex_unlock(&state->chip_lock);

	// Mark as resumed
	state->suspended = 0;
err:
	mutex_unlock(&state->fs_lock);
	return 0;
}
#else
#define kxte9_i2c_suspend  NULL
#define kxte9_i2c_resume   NULL
#endif  /* CONFIG_PM */

static struct i2c_driver kxte9_i2c_driver = { 
	.driver = { 
		.name = KXTE9_I2C_DRIVER, 
	}, 
	.probe		= kxte9_i2c_probe, 
	.remove 	= __devexit_p(kxte9_i2c_remove), 
	.suspend 	= kxte9_i2c_suspend, 
	.resume 	= kxte9_i2c_resume, 
}; 

/*********************************************************************************
* kxte9_mudule_init(void)
*
* This is the function called when the module is loaded.
*
* Inputs   None
* 
* return error code , 0 on success, or non-zero otherwise
***********************************************************************************/
static int __init 
kxte9_module_init(void)
{
	return i2c_add_driver(&kxte9_i2c_driver);
}

/*********************************************************************************
* kxte9_module_exit(void)
*
* This is the function called when the module is unloaded.
*
* Inputs None
*
* return None
***********************************************************************************/
static void __exit 
kxte9_module_exit(void)
{
	i2c_del_driver(&kxte9_i2c_driver); 
}

module_init(kxte9_module_init);
module_exit(kxte9_module_exit);

MODULE_DESCRIPTION("KXTE9 ACCELEROMETER driver");
MODULE_LICENSE("GPL");
