 /* 
 * linux/drivers/input/kxsd9_accelerometer.c 
 * 
 * Driver for the kxsd9 accelerometer. 
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
#include <asm/arch/mux.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c_kxsd9_accelerometer.h>
#include "kxsd9_accelerometer.h"

struct accelerometer_state { 
	struct i2c_client      *i2c_dev;
	struct input_dev       *inp_dev; 
	struct kxsd9_platform_data *pdata;
	struct timer_list       accelerometer_timer; 
	struct mutex fs_lock;
	struct mutex chip_lock;
	struct work_struct      read_work; 
	int    suspended;       /*flag to signal suspend resume*/
	int    stopping;	/* set if stop operation is in progress */
	atomic_t opencount;
	u8     motion_wake_up_threshold; /*motion threshold value*/
	u8     filter_frequency; /*Filter frequency bandwidth*/
	u8     poll_intr_mode; /*select Polling or Interrupt*/ 
	u16    poll_interval_val; /*poll interval*/
#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXSD9
	u8     test_id; /*test id to be executed.*/
	u8     test_result;/*result of the executed test*/
	u8     test_reg_name;
#endif
};
#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXSD9
atomic_t current_timer_test_val;
#endif

#define IS_POLLED_MODE(x) ( (x == MODE_POLL_AXIS) | (x == MODE_POLL_ALL) )

/****************************************************************************** 
* 
* accelerometer_i2c_read_u8 
* 
* Inputs 
* struct i2c_client* client,  u8 index, u8 num,u8* out    
*
* Returns 
* 0 on success  or non-zero on fail
* 
******************************************************************************/ 
static int 
accelerometer_i2c_read_u8(struct i2c_client* client,  u8 index, u8 num,u8* out) 
{ 
	int ret; 
	struct i2c_msg msgs[7];

	// Write the register index. 
	msgs[0].addr = client->addr; 
	msgs[0].len = 1; 
	msgs[0].flags = 0; 
	msgs[0].buf = &index; 
 
	// Read the register value. 
	msgs[1].addr = client->addr; 
	msgs[1].len = num; 
	msgs[1].flags = I2C_M_RD; 
	msgs[1].buf = &out[0]; 
 
	// Make it so... 
	ret = i2c_transfer(client->adapter, msgs, 2); 
 
	return (ret); 
} 

/*************************************************************************** 
* 
* accelerometer_i2c_write_u8 
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
static int 
accelerometer_i2c_write_u8(struct i2c_client* client, u8 index, u8 value) 
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
* accelerometer_enable_timer 
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
accelerometer_enable_timer(struct accelerometer_state *state, int enable)
{
	if(enable == TIMER_ENABLED) {
		mod_timer(&state->accelerometer_timer, 
		          jiffies + msecs_to_jiffies(state->poll_interval_val));
	} else {
		del_timer_sync(&state->accelerometer_timer);
	}
}

/****************************************************************************** 
* 
* accelerometer_enable_motion_intr 
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
accelerometer_enable_motion_intr(struct i2c_client* client,u8 enable)
{
	int res;
	u8  val;
	
	if(enable == ENABLE_MOTION_INTERRUPT)
	{
		// Clear the interrupt by reading CTRLREGA
		res = accelerometer_i2c_read_u8(client, CTRL_REGA,1,&val); 
		res = accelerometer_i2c_read_u8(client, CTRL_REGB,1,&val); 
		// RE Enable the Motion Interrupt by setting the motion bit
		val |= MOTION_INTERRUPT_BIT;
		res = accelerometer_i2c_write_u8(client, CTRL_REGB, val);
	}
	else
	{
		res = accelerometer_i2c_read_u8(client, CTRL_REGB,1,&val); 
		// Clear the Motion Interrupt bit
		val &= (~MOTION_INTERRUPT_BIT);
		res = accelerometer_i2c_write_u8(client, CTRL_REGB, val);
	}
}

/*****************************************************************************
*
* kxsd9_accelerometer_isr
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
kxsd9_accelerometer_isr(int irq, void *dev_id)
{
	struct accelerometer_state *state = dev_id;

	schedule_work( &state->read_work ); 
	return IRQ_HANDLED;
}

/****************************************************************************** 
* 
* accelerometer_timeout() 
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
accelerometer_timeout(unsigned long statePtr )
{
	struct accelerometer_state *state = (struct accelerometer_state *)statePtr;

	/* Queue the work */
	schedule_work( &state->read_work ); 

	#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXSD9
	atomic_inc(&current_timer_test_val);/*Test For Polling*/
	#endif
}

/****************************************************************************** 
* 
* accelerometer_stop 
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
accelerometer_stop(struct i2c_client *dev)
{
	struct accelerometer_state *state = i2c_get_clientdata(dev);

	mutex_lock(&state->chip_lock);
	state->stopping = 1; // set stopping
	accelerometer_enable_motion_intr( state->i2c_dev, DISABLE_MOTION_INTERRUPT );
	accelerometer_enable_timer ( state, TIMER_DISABLED );
	mutex_unlock(&state->chip_lock);
	cancel_work_sync  ( &state->read_work ); 
	state->stopping = 0; // stopped
}

/****************************************************************************** 
* 
* accelerometer_start_nolock 
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
accelerometer_start_nolock(struct i2c_client *dev) 
{
	struct accelerometer_state *state = i2c_get_clientdata(dev);

	if (atomic_read(&state->opencount) == 0)
		return;
	
	if(IS_POLLED_MODE (state->poll_intr_mode) ) { 
		/* Reactivate the timer again */
		accelerometer_enable_timer(state, TIMER_ENABLED);
	} 
	else if( state->poll_intr_mode == MODE_INTERRUPT ) {
	        accelerometer_enable_motion_intr(state->i2c_dev,
	                                         ENABLE_MOTION_INTERRUPT);
	}
}


//Following are the sysfs callback functions 
/****************************************************************************** 
* 
*  show_accelerometer_polling_interrupt_mode
* 
* This function read the mode in which the accelerometer is working
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns polling or interrupt mode,
* 
******************************************************************************/ 
static ssize_t
show_mode(struct device *dev,
	  struct device_attribute *attr,
	  char *buf)
{
	ssize_t count = 0;
	char *mode = NULL;
	struct accelerometer_state *state = dev->driver_data;

	switch (state->poll_intr_mode){
		case MODE_POLL_ALL:
			mode = "all";
			break;
		case MODE_POLL_AXIS:
			mode = "axis";
			break;
		case MODE_OFF:
			mode = "off";
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
* set_accelerometer_polling_interrupt_mode
* 
* This function set the mode of the accelerometer i.e polling or interrupt 
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
	struct accelerometer_state *state = dev->driver_data;

	mutex_lock(&state->fs_lock);
	accelerometer_stop(state->i2c_dev); /*stop the previous mode*/
	mutex_lock(&state->chip_lock);
	if( count >= 3 && strncmp( buf, "all", 3) == 0 ) {
		state->poll_intr_mode = MODE_POLL_ALL; 
	} else if( count >= 4 && strncmp( buf, "axis", 4) == 0 ) {
		state->poll_intr_mode = MODE_POLL_AXIS; 
	} else if( count >= 3 && strncmp( buf, "off", 3) == 0 ) {
		state->poll_intr_mode = MODE_OFF;
	}
	accelerometer_start_nolock(state->i2c_dev); /*start in the new mode*/
	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);
	
	return count;
}

/****************************************************************************** 
* 
*  show_accelerometer_polling_interval
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
show_accelerometer_polling_interval(struct device *dev,
	                            struct device_attribute *attr, char *buf)
{
	struct accelerometer_state *state = dev->driver_data;
	return snprintf(buf, PAGE_SIZE, "%u\n", state->poll_interval_val);
}

/****************************************************************************** 
* 
* set_accelerometer_polling_interval
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
set_accelerometer_polling_interval(struct device *dev,
	                           struct device_attribute *attr, 
	                           const char *buf, size_t count)
{
	u16 value;
	struct accelerometer_state *state = dev->driver_data;
	if (!buf || !count) 
		return 0; 
		
	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	value = simple_strtoul(buf, NULL, 10);
	state->poll_interval_val = value;
	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);
	
	return count;
}

/****************************************************************************** 
* 
* show_accelerometer_motion_wake_up_threshold 
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
show_accelerometer_motion_wake_up_threshold(struct device *dev,
	                                    struct device_attribute *attr, 
	                                    char *buf)
{
	struct accelerometer_state *data = dev->driver_data;
	return snprintf(buf,sizeof(buf), "%u",data->motion_wake_up_threshold);
}

/****************************************************************************** 
* 
* set_accelerometer_motion_wake_up_threshold
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
set_accelerometer_motion_wake_up_threshold(struct device *dev, 
	                                   struct device_attribute *attr,
	                                   const char *buf, size_t count)
{
	u8 value,val;
	int res;
	struct accelerometer_state *data = dev->driver_data;

	mutex_lock(&data->fs_lock);
	mutex_lock(&data->chip_lock);

	value = simple_strtoul(buf, NULL, 10);
	res = accelerometer_i2c_read_u8(data->i2c_dev, CTRL_REGC,1,&val);
	val &= (~CLEAR_MOTION_WAKE_UP_THRESHOLD);
	value &= (CLEAR_MOTION_WAKE_UP_THRESHOLD);
	data->motion_wake_up_threshold = value;
	val |= value;
	accelerometer_i2c_write_u8(data->i2c_dev, CTRL_REGC, val);

	mutex_unlock(&data->chip_lock);
	mutex_unlock(&data->fs_lock);
	return count;
}

/****************************************************************************** 
* 
* show_accelerometer_filter_frequency 
* 
* Read the requested register when sysfs node for this is being read 
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_accelerometer_filter_frequency(struct device *dev,
	                            struct device_attribute *attr, char *buf)
{
	struct accelerometer_state *data = (struct accelerometer_state *)dev->driver_data;
	return snprintf(buf, sizeof(buf),"%u",data->filter_frequency );
}

/****************************************************************************** 
* 
* set_accelerometer_filter_frequency
* 
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, 
*	const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t 
set_accelerometer_filter_frequency(struct device *dev,
	                           struct device_attribute *attr, 
	                           const char *buf, size_t count)
{
	u8 value,val;
	int res;
	struct accelerometer_state *data = dev->driver_data;

	mutex_lock(&data->fs_lock);
	mutex_lock(&data->chip_lock);
	
	value = simple_strtoul(buf, NULL, 10);
	res = accelerometer_i2c_read_u8(data->i2c_dev, CTRL_REGC,1,&val);
	val &= (~CLEAR_FILTER_FRQUENCY);
	value &= (CLEAR_FILTER_FRQUENCY);
	data->filter_frequency = value;
	val |= value;
	accelerometer_i2c_write_u8(data->i2c_dev, CTRL_REGC, val);

	mutex_unlock(&data->chip_lock);
	mutex_unlock(&data->fs_lock);

	return count;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode, set_mode);
static DEVICE_ATTR(poll_interval, S_IRUGO | S_IWUSR,
			show_accelerometer_polling_interval,
			set_accelerometer_polling_interval);
static DEVICE_ATTR(accelerometer_motion_wake_up_threshold, S_IRUGO | S_IWUSR,
			show_accelerometer_motion_wake_up_threshold,
			set_accelerometer_motion_wake_up_threshold);
static DEVICE_ATTR(accelerometer_filter_frequency, S_IRUGO | S_IWUSR,
			show_accelerometer_filter_frequency, 
			set_accelerometer_filter_frequency);

#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXSD9
/****************************************************************************** 
* 
*  show_accelerometer_reg_val
* 
* This function read the accelerometer register val
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns test id
* 
******************************************************************************/ 
static ssize_t
show_accelerometer_reg_val(struct device *dev, 
	                   struct device_attribute *attr, char *buf)
{
	u8 val,res;
	struct accelerometer_state *data = dev->driver_data;
	res = accelerometer_i2c_read_u8(data->i2c_dev, data->test_reg_name,1,&val);
	return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

/****************************************************************************** 
* 
* set_accelerometer_reg_val
* 
* This function set the accelerometer register 
*
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns test pass or fail status
* 
******************************************************************************/ 
static ssize_t 
set_accelerometer_reg_val(struct device *dev,
	                  struct device_attribute *attr,
	                  const char *buf, size_t count)
{
	u8 val;
	struct accelerometer_state *data = dev->driver_data;

	mutex_lock(&data->fs_lock);
	mutex_lock(&data->chip_lock);
	val = simple_strtoul(buf, NULL, 10);
	accelerometer_i2c_write_u8(data->i2c_dev, data->test_reg_name, val);
	mutex_unlock(&data->chip_lock);
	mutex_unlock(&data->fs_lock);

	return count;
}


/****************************************************************************** 
* 
* show_accelerometer_reg_name 
* 
* This function read the register name to be read or written
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns 
*   read value of the requested register. 
* 
******************************************************************************/ 
static ssize_t 
show_accelerometer_reg_name(struct device *dev,
	                    struct device_attribute *attr, 
	                    char *buf)
{
	struct accelerometer_state *data = dev->driver_data;
	return snprintf(buf, PAGE_SIZE, "%u",data->test_reg_name);
}

/****************************************************************************** 
* 
* set_accelerometer_reg_name
* 
* This function set the accelerometer regester name to be read or written
*  
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns 
*   set value to the requested register. 
* 
******************************************************************************/ 
static ssize_t
set_accelerometer_reg_name(struct device *dev, 
	                   struct device_attribute *attr,
	                   const char *buf, size_t count)
{
	u8 value;
	struct accelerometer_state *data = dev->driver_data;
	
	mutex_lock(&data->fs_lock);
	mutex_lock(&data->chip_lock);
	value = simple_strtoul(buf, NULL, 10);
	data->test_reg_name = value;
	mutex_unlock(&data->chip_lock);
	mutex_unlock(&data->fs_lock);
	return count;
}

/****************************************************************************** 
* 
*  show_accelerometer_test_poll
* 
* This function read the accelerometer pool count 
*  
* Inputs 
*   struct device *dev, struct device_attribute *attr, char *buf 
* 
* Returns test id
* 
******************************************************************************/ 
static ssize_t
show_accelerometer_test_poll(struct device *dev, 
	                     struct device_attribute *attr, char *buf)
{
	u16 val;
	if (!buf) 
		return 0; 
	val= atomic_read(&current_timer_test_val);
	return snprintf(buf, PAGE_SIZE, "%u\n", val);
}
/****************************************************************************** 
* 
* set_accelerometer_test_poll
* 
* This function set the accelerometer poll counter to a value typically 0
*
* Inputs 
*  struct device *dev, struct device_attribute *attr, const char *buf, size_t count
* 
* Returns test pass or fail status
* 
******************************************************************************/ 
static ssize_t 
set_accelerometer_test_poll(struct device *dev,
	                    struct device_attribute *attr,
	                    const char *buf, size_t count)
{
	u16 val;
	struct accelerometer_state *state = dev->driver_data;

	mutex_lock(&state->fs_lock);
	mutex_lock(&state->chip_lock);
	val = simple_strtoul(buf, NULL, 10);
	atomic_set(&current_timer_test_val,val);
	mutex_unlock(&state->chip_lock);
	mutex_unlock(&state->fs_lock);
	return count; 
}


static DEVICE_ATTR(accelerometer_test_reg_val, S_IRUGO | S_IWUSR,
			show_accelerometer_reg_val, set_accelerometer_reg_val);
static DEVICE_ATTR(accelerometer_test_reg_name, S_IRUGO | S_IWUSR,
			show_accelerometer_reg_name, set_accelerometer_reg_name);
static DEVICE_ATTR(accelerometer_test_poll, S_IRUGO | S_IWUSR,
			show_accelerometer_test_poll, set_accelerometer_test_poll);
#endif


/****************************************************************************** 
* 
* kxsd9_count_to_g(u8 out) 
* 
* Convert the X-axis/Y-axis/Z-axis acceleration into G unit, it's range
* -20000 to 20000 which map to -2g to +2g (which is the current sensitvity
* range configured via CTRL_REGC
* Inputs 
*   u32 out 
* 
* Returns 
*   int. 
* 
******************************************************************************/ 
static int kxsd9_count_to_g(u32 out)
{
	int gravity;
	// KXSD9 has 819 counts per G for the +/-2G
	gravity = ( (int) (10000 * (out - 2048) ) ) / 819; 
	return gravity;
}

/****************************************************************************** 
* 
* kxsd9_translate_xyz (struct accelerometer_state *state, 
* int *x_gravity, int * y_gravity, int * z_gravity) 
* 
* Translates the the X,Y and Z axis values as per the board file
* Inputs 
*   struct work_struct *work 
* 
* Returns 
*   None. 
* 
******************************************************************************/ 
void kxsd9_translate_xyz (struct accelerometer_state *state, 
	                  int *x_gravity, int * y_gravity, int * z_gravity)
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
* accelerometer_read_xyz(void) 
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
accelerometer_read_xyz(struct work_struct *work) 
{
	//u8 val;
	u8 out[6];
	int res;
	int x_gravity = 0;
	int y_gravity = 0; 
	int z_gravity = 0; 
	u32 accelerometer_x = 0;
	u32 accelerometer_y = 0;
	u32 accelerometer_z = 0;
	struct accelerometer_state *state = container_of(work, struct accelerometer_state, read_work);

	mutex_lock(&state->chip_lock);
	if (state->stopping) {
		mutex_unlock(&state->chip_lock);
		return;
	}

	//Read X,Y and Z coordinate by reading all the six consecutive X,Yand Z registers 
	res = accelerometer_i2c_read_u8(state->i2c_dev,XOUT_H_READ_REG, 6,&out[0]);
	// Get X Coordinate
	accelerometer_x =  0x00;
	accelerometer_x = out[0];
	accelerometer_x = ( accelerometer_x  << 4 );
	accelerometer_x |= (out[1]  >> 4 ) & 0x0F;
	// Get Y Coordinate
	accelerometer_y =  0x00;
	accelerometer_y = out[2];
	accelerometer_y = ( accelerometer_y  << 4 );
	accelerometer_y |= (out[3]  >> 4 ) & 0x0F;
	// Get Z Coordinate
	accelerometer_z =  0x00;
	accelerometer_z = out[4];
	accelerometer_z = ( accelerometer_z  << 4 );
	accelerometer_z |= (out[5]  >> 4 ) & 0x0F;


	x_gravity = kxsd9_count_to_g (accelerometer_x);
	y_gravity = kxsd9_count_to_g (accelerometer_y);
	z_gravity = kxsd9_count_to_g (accelerometer_z);

	// translate the xyz gravity units per the translation map
	kxsd9_translate_xyz (state, &x_gravity, &y_gravity, &z_gravity);

	// Report X,Y and Z Coordinate to input subsystem
	input_report_abs(state->inp_dev,  ABS_X, x_gravity);
	input_report_abs(state->inp_dev,  ABS_Y, y_gravity);
	input_report_abs(state->inp_dev,  ABS_Z, z_gravity);
	input_sync(state->inp_dev);

	if(IS_POLLED_MODE (state->poll_intr_mode) ) {
		// In poll mode we just unconditinally set up next sample
		mod_timer(&state->accelerometer_timer,
		          jiffies + msecs_to_jiffies(state->poll_interval_val));
	}
	if( state->poll_intr_mode == MODE_INTERRUPT ) {
		// In interrupt mode we should actually figure out when to stop sampling
		// DOLATER: Please implement ME
		printk ("Please: implement me\n" );
		mod_timer(&state->accelerometer_timer,
		           jiffies  + (HZ * state->poll_interval_val)/1000);
	}

	mutex_unlock(&state->chip_lock);
}


/*********************************************************************************
* accelerometer_inputdev_open (struct input_dev *dev)
*
* This is the function called when the accelerometer input event is being listened
*
* Inputs   None
* 
* return error code , 0 on success, or non-zero otherwise
***********************************************************************************/
static int accelerometer_inputdev_open(struct input_dev *dev)
{
	int rc;
	struct accelerometer_state *state = (dev_get_drvdata (dev->dev.parent));

	mutex_lock(&state->fs_lock);
	rc = atomic_add_return( 1, &state->opencount);
	if (1 == rc) {
		mutex_lock(&state->chip_lock);
		accelerometer_start_nolock (state->i2c_dev);
		mutex_unlock(&state->chip_lock);
	}
	mutex_unlock(&state->fs_lock);

	return 0;
}


/*********************************************************************************
* accelerometer_inputdev_close (struct input_dev *dev)
*
* This is the function called when the accelerometer input event is being listened
*
* Inputs   None
* 
* return error code , 0 on success, or non-zero otherwise
***********************************************************************************/
static void accelerometer_inputdev_close(struct input_dev *dev)
{
	int rc;
	struct accelerometer_state *state = dev_get_drvdata (dev->dev.parent);

	mutex_lock(&state->fs_lock);
	rc = atomic_sub_return(1,&state->opencount);
	if (0 == rc) {
		accelerometer_stop (state->i2c_dev);
	}
	mutex_unlock(&state->fs_lock);
}

/****************************************************************************** 
* 
* accelerometer_i2c_probe() 
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
accelerometer_i2c_probe(struct i2c_client *client) 
{ 
	int rc;
	u8 val;
	int res;
	struct accelerometer_state *state;
	struct kxsd9_platform_data *pdata = client->dev.platform_data;

	PDBG( KERN_INFO "%s accelerometer driver\n", ACCELEROMETER_DRIVER);

	/* Sanity Check */
	rc = accelerometer_i2c_read_u8(client, CTRL_REGC, 1, &val);
	if (rc < 0) {
		return -ENODEV;
	}

	//Create the accelerometer state 
	state = kzalloc ( sizeof(struct accelerometer_state), GFP_KERNEL);
	if(!state)
	{
		PDBG( KERN_ERR "accelerometer state memory allocation failed \n");
		return (-ENOMEM);
	}

	/*Attach i2c_dev */
	state->i2c_dev = client;

	// Init spinlock 
	mutex_init(&state->fs_lock);
	mutex_init(&state->chip_lock);
	state->suspended = 0;

	// Attach platform data 
	state->pdata = pdata;
	
	/*By default poll mode is selected*/
	state->poll_intr_mode = MODE_OFF;
	atomic_set ( &state->opencount, 0);

	/* Default value of poll interval is 250 ms*/
	state->poll_interval_val = POLL_INTERVAL; 
	state->motion_wake_up_threshold = CTRL_REGC_VAL & CLEAR_MOTION_WAKE_UP_THRESHOLD;
	state->filter_frequency = CTRL_REGC_VAL & CLEAR_FILTER_FRQUENCY; 
	
	//Init Workque ineterface  
	INIT_WORK(&state->read_work, accelerometer_read_xyz); 

	res = accelerometer_i2c_write_u8(client, RESET_WRITE_REG, RESET_WRITE_VAL); 
	res = accelerometer_i2c_read_u8 (client, CTRL_REGA,1, &val); 
	val = CTRL_REGB_VAL;
	res = accelerometer_i2c_write_u8(client, CTRL_REGB,  val);
	res = accelerometer_i2c_write_u8(client, CTRL_REGC, CTRL_REGC_VAL); 

	// setup input device  
	state->inp_dev = input_allocate_device(); 
	if( state->inp_dev == NULL ) 
	{
		PDBG(KERN_ERR "accelerometer input device Mem alloc fail"); 
		goto err0; 
	}

	//Enable key events and auto repeate features of linux input subsystem  
	set_bit(EV_ABS,  state->inp_dev->evbit); 
	set_bit(ABS_X,   state->inp_dev->absbit ); 
	set_bit(ABS_Y,   state->inp_dev->absbit ); 
	set_bit(ABS_Z,   state->inp_dev->absbit ); 
	
	input_set_abs_params(state->inp_dev, ABS_X, ACCEL_MIN_ABS_X, ACCEL_MAX_ABS_X, 0, SENSITIVITY_UNITS_PER_ACCELERATION_1G);
	input_set_abs_params(state->inp_dev, ABS_Y, ACCEL_MIN_ABS_Y, ACCEL_MAX_ABS_Y, 0, SENSITIVITY_UNITS_PER_ACCELERATION_1G);
	input_set_abs_params(state->inp_dev, ABS_Z, ACCEL_MIN_ABS_Z, ACCEL_MAX_ABS_Z, 0, SENSITIVITY_UNITS_PER_ACCELERATION_1G);

	state->inp_dev->name = ACCELEROMETER_DRIVER;
	state->inp_dev->id.bustype = BUS_I2C; 
	state->inp_dev->id.vendor  = ACCELEROMETER_VENDOR_ID;
	state->inp_dev->id.product = ACCELEROMETER_PRODUCT_ID;  
	state->inp_dev->id.version = ACCELEROMETER_VERSION_ID;
	state->inp_dev->dev.parent = &client->dev;

	state->inp_dev->open = accelerometer_inputdev_open;
	state->inp_dev->close = accelerometer_inputdev_close;
  
	rc = input_register_device(state->inp_dev); 
	if( rc != 0 ) {
		PDBG(KERN_ERR "accelerometer input device registration failed"); 
		goto err1;
	}

	dev_set_drvdata(&state->inp_dev->dev, state);

	rc = gpio_request( irq_to_gpio(client->irq), ACCELEROMETER_DRIVER);
	if( rc != 0 ) 
	{
		PDBG(KERN_ERR "accelerometer gpio request failed"); 
		goto err2; 
	}

	//set up a repeating timer 
	init_timer_deferrable(&state->accelerometer_timer);
	state->accelerometer_timer.function = accelerometer_timeout;
	state->accelerometer_timer.data = ( unsigned long)state;

	// request an interrupt
	rc = request_irq( client->irq, kxsd9_accelerometer_isr,
				IRQF_TRIGGER_RISING, ACCELEROMETER_DRIVER, 
				(void*)state );
	if( rc < 0 ) 
	{
		PDBG(KERN_ERR "accelerometer request irq failed"); 
		goto err3;
	}
	if( state->poll_intr_mode == MODE_INTERRUPT ) 
	{   // enable interrupt
		accelerometer_enable_motion_intr ( client, ENABLE_MOTION_INTERRUPT );
	}
	else 
	{   // disable interrupt
		accelerometer_enable_motion_intr ( client, DISABLE_MOTION_INTERRUPT );
	}
	if(IS_POLLED_MODE (state->poll_intr_mode) )/*polling mode*/ 
	{   // if we are in polling mode - start timer (interrupts are disabled)
		mod_timer(&state->accelerometer_timer, 
		           jiffies  + (HZ * state->poll_interval_val)/1000);
	}
	
	//attach driver data 
	i2c_set_clientdata ( client, state );

	//create device sysfs attributes  
	rc = device_create_file(&state->inp_dev->dev, &dev_attr_mode); 
	if (rc != 0) { 
		PDBG(KERN_ERR "kxsd9 accelerometer: Error in creating device sysfs entries1...\n"); 
		goto err4; 
	} 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_poll_interval); 
	if (rc != 0) { 
		PDBG(KERN_ERR "kxsd9 accelerometer: Error in creating device sysfs entries2...\n"); 
		goto err5; 
	} 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_accelerometer_motion_wake_up_threshold); 
	if (rc != 0) { 
		PDBG(KERN_ERR "kxsd9 accelerometer: Error in creating device sysfs entries3...\n"); 
		goto err6; 
	} 

	rc = device_create_file(&state->inp_dev->dev, &dev_attr_accelerometer_filter_frequency); 
	if (rc != 0) { 
		PDBG(KERN_ERR "kxsd9 accelerometer: Error in creating device sysfs entries4...\n"); 
		goto err7; 
	} 
	
	//Test Interface attributes
	#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXSD9
	rc = device_create_file(&state->inp_dev->dev, &dev_attr_accelerometer_test_poll); 
	rc = device_create_file(&state->inp_dev->dev, &dev_attr_accelerometer_test_reg_name); 
	rc = device_create_file(&state->inp_dev->dev, &dev_attr_accelerometer_test_reg_val); 
	#endif
	
	return 0;

err7:
	device_remove_file(&state->inp_dev->dev, &dev_attr_accelerometer_motion_wake_up_threshold); 
err6:
	device_remove_file(&state->inp_dev->dev, &dev_attr_poll_interval); 
err5:
	device_remove_file(&state->inp_dev->dev, &dev_attr_mode); 
err4:
	free_irq  ( state->i2c_dev->irq, state );
err3:
    gpio_free ( irq_to_gpio(client->irq));
err2:
    input_unregister_device ( state->inp_dev );
err1:
    input_free_device ( state->inp_dev );
err0:
    kfree ( state );

    printk( KERN_INFO "Failed to intialize %s accelerometer driver\n",  
                       ACCELEROMETER_DRIVER );

    return -ENODEV;    
} 
/****************************************************************************** 
* 
* accelerometer_i2c_remove() 
*
* Remove the accelerometer driver, does whatever cleanup  is required
* 
* Inputs  
*   i2c client pointer. 
* 
* Returns 0
* 
******************************************************************************/ 
static int 
accelerometer_i2c_remove(struct i2c_client *client) 
{ 
	struct accelerometer_state *state = i2c_get_clientdata(client);    

	mutex_lock(&state->fs_lock);

	device_remove_file(&state->inp_dev->dev, &dev_attr_accelerometer_filter_frequency); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_accelerometer_motion_wake_up_threshold); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_poll_interval); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_mode); 

	#ifdef CONFIG_ACCELEROMETER_TEST_DEBUG_KXSD9
	device_remove_file(&state->inp_dev->dev, &dev_attr_accelerometer_test_poll); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_accelerometer_test_reg_name); 
	device_remove_file(&state->inp_dev->dev, &dev_attr_accelerometer_test_reg_val); 
	#endif

	accelerometer_stop(client);

	//unregister input device
	input_unregister_device ( state->inp_dev );

	//unregister interrupt handlers and free gpio
	free_irq  ( state->i2c_dev->irq, state );
	gpio_free ( irq_to_gpio(state->i2c_dev->irq));

	mutex_unlock(&state->fs_lock);

	//Detach state 
	i2c_set_clientdata(client, NULL );    

	//Free state 
	kfree ( state );

	return 0;
}

/******************************************************************************
* accelerometer_i2c_suspend 
* 
* Called by Power management to suspend the accelerometer  
*
* Inputs 
*  struct i2c_client *dev, pm_message_t event 
* 
* Returns 
* 0
******************************************************************************/
#ifdef CONFIG_PM
static int 
accelerometer_i2c_suspend( struct i2c_client *dev, pm_message_t event )
{
	int res;
	u8 val;
	struct accelerometer_state *state = i2c_get_clientdata(dev);    

	mutex_lock(&state->fs_lock);

	//Check if already suspended */
	if( state->suspended  ) 
		goto err;
		
	accelerometer_stop(dev);
	
	// Put accelerometer in suspend mode by clear the ENABLE  bit  
	res = accelerometer_i2c_read_u8(state->i2c_dev, CTRL_REGB,1,&val); 
	val &= (~ACCELEROMETER_ENABLE_BIT);
	res = accelerometer_i2c_write_u8(state->i2c_dev, CTRL_REGB, val);

	//Mark it suspended */
	state->suspended = 1;

err:
	mutex_unlock(&state->fs_lock);
	return 0;
}

/******************************************************************************
* accelerometer_i2c_resume 
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
accelerometer_i2c_resume ( struct i2c_client *dev )
{
	int res;
	u8 val;
	struct accelerometer_state *state = i2c_get_clientdata(dev);    

	mutex_lock(&state->fs_lock);

	if(!state->suspended ) 
		goto err; // already resumed
		

	mutex_lock(&state->chip_lock);
	accelerometer_start_nolock(dev);
	
	// Put accelerometer in normal working mode
	res = accelerometer_i2c_read_u8(state->i2c_dev, CTRL_REGB,1,&val); 
	val |= ACCELEROMETER_ENABLE_BIT;
	res = accelerometer_i2c_write_u8(state->i2c_dev, CTRL_REGB, val);
	mutex_unlock(&state->chip_lock);
	
	//Reset suspended flag 
	state->suspended = 0;

err:
	mutex_unlock(&state->fs_lock);
	return 0;
}
#else
#define accelerometer_i2c_suspend  NULL
#define accelerometer_i2c_resume   NULL
#endif  /* CONFIG_PM */

static struct i2c_driver accelerometer_i2c_driver = { 
	.driver = { 
		.name = ACCELEROMETER_DRIVER, 
	}, 
	.probe = accelerometer_i2c_probe, 
	.remove = __devexit_p(accelerometer_i2c_remove), 
	.suspend = accelerometer_i2c_suspend, 
	.resume = accelerometer_i2c_resume, 
}; 

/*********************************************************************************
* kxsd9_accelerometer_init(void)
*
* This is the function called when the module is loaded.
*
* Inputs   None
* 
* return error code , 0 on success, or non-zero otherwise
***********************************************************************************/
static int __init 
kxsd9_accelerometer_init(void)
{
	//Register i2c bus for accelerometer 
	return i2c_add_driver(&accelerometer_i2c_driver);
}

/*********************************************************************************
* kxsd9_accelerometer_exit(void)
*
*This is the function called when the module is unloaded.
*
*Inputs None
*
* return None
***********************************************************************************/
static void __exit 
kxsd9_accelerometer_exit(void)
{
	//unregister i2c bus for accelerometer 
	i2c_del_driver(&accelerometer_i2c_driver); 
}

module_init(kxsd9_accelerometer_init);
module_exit(kxsd9_accelerometer_exit);
MODULE_DESCRIPTION("kxsd9 accelerometer driver");
MODULE_LICENSE("GPL");
