/*
 *  linux/drivers/leds/leds-lp8501.c - NS LED driver for the LP8501
 *
 *  Copyright (C) 2008 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors: Kevin McCray (kevin.mccray@palm.com)
 *          Brian Xiong (brian.xiong@palm.com)
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/i2c.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/i2c_lp8501_led.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>


#define AUTO_INCR_WRITE

static int lp8501_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg);

DECLARE_WAIT_QUEUE_HEAD(lp8501_wait_queue);
DECLARE_WAIT_QUEUE_HEAD(lp8501_wait_stop_engine);

struct lp8501_device_state {
	struct i2c_client	*i2c_dev;
	struct lp8501_platform_data pdata;
	struct work_struct notify_work;
	struct mutex lock;
	struct file_operations fops;
	struct miscdevice mdev;
	u16 instruct[INSTR_LEN];
	int suspended;
	int interrupt_state;
	int stop_engine_state;
	u8 stopengine;
	u8 config_reg;
};

struct engine_cmd {
	struct lp8501_device_state *context;
	struct work_struct workitem;
	int engine;
	int cmd;
};	

int lp8501_i2c_read_reg(struct i2c_client* client, u8 addr, u8* out)
{
	int ret;
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].len = 1;
	msg[0].flags = 0;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].len = 1;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = out;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_ERR "lp8501_i2c_read_reg: error reading addr=0x%X\n", addr);
	}
	
	return ret;
}

int lp8501_i2c_write_reg(struct i2c_client* client, u8 addr, u8 val)
{
	u8 buf[2] = {addr, val};
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;
	
	ret = i2c_transfer(client->adapter, msg, 1);

	if (ret < 0) {
		printk(KERN_ERR "lp8501_i2c_write_reg: error writing addr=0x%X val=0x%X\n", addr, val);
	}
	return ret;
}


int lp8501_i2c_write_reg_auto_incr (struct i2c_client* client, u8 * address_and_data, u8 length)
{
	int ret;
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = length;
	msg[0].buf = address_and_data;
	
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		printk(KERN_ERR "lp8501_i2c_write_reg_auto_incr: error writing addr=0x%X length=0x%X\n", address_and_data[0], length);
	}
	return ret;
}

static int convert_percent_to_pwm(int percent, int *pwm)
{
#define DEFAULT_BRIGHTNESS	40
	int ret = 0;

	if(percent < 0 || percent > 100)
	{
		ret = -EINVAL;
		goto done;
	}
	// For a given percentage value of brightness, Broadway is less
	// brighter than Castle, especially in dimm / dark regions (NOV-113971).
	// Scale up the brightness, for percentage values below half of the default
	// brightness.
	if ( (percent > 0) && (percent < (DEFAULT_BRIGHTNESS/2) ) ) {
		percent = percent + ( (DEFAULT_BRIGHTNESS - percent) / 5 );
	}

	*pwm = (255 * percent) / 100;

done:
	return ret;
}


// ENGINE_CNTRL1 and ENGINE_CNTRL2 have the same bit mappings
// bits[5:4] = Engine 1
// bits[3:2] = Engine 2
// bits[1:0] = Engine 3
static int lp8501_config_engine_control_mode(struct lp8501_device_state *p_state, int eng, u8 engine_cntrl, u8 mode)
{
	u8 reg;
	int ret = 0;

	lp8501_i2c_read_reg(p_state->i2c_dev, engine_cntrl, &reg);

	switch(eng)
	{
		case 1:
			// clear existing bits for ENG1 control
			reg = reg &~ (3 << ENGINE_CNTRL_ENG1_SHIFT);
			lp8501_i2c_write_reg(p_state->i2c_dev, engine_cntrl, reg | (mode << ENGINE_CNTRL_ENG1_SHIFT));
			break;
		case 2:
			// clear existing bits for ENG2 control
			reg = reg &~ (3 << ENGINE_CNTRL_ENG2_SHIFT);
			lp8501_i2c_write_reg(p_state->i2c_dev, engine_cntrl, reg | (mode << ENGINE_CNTRL_ENG2_SHIFT));
			break;
		case 3:
			// clear existing bits for ENG3 control
			reg = reg &~ (3 << ENGINE_CNTRL_ENG3_SHIFT);
			lp8501_i2c_write_reg(p_state->i2c_dev, engine_cntrl, reg | (mode << ENGINE_CNTRL_ENG3_SHIFT));
			break;
		default:
			break;
	}

	return ret;
}


void setup_page_sizes(struct lp8501_device_state *p_state, int eng, int *startpage, int *endpage)
{
	switch(eng)
	{
		case 1:
			*startpage = p_state->pdata.memcfg->eng1_startpage;
			*endpage = p_state->pdata.memcfg->eng1_endpage;
			break;
		case 2:
			*startpage = p_state->pdata.memcfg->eng2_startpage;
			*endpage = p_state->pdata.memcfg->eng2_endpage;
			break;
		case 3:
			*startpage = p_state->pdata.memcfg->eng3_startpage;
			*endpage = p_state->pdata.memcfg->eng3_endpage;
			break;
		default:
			break;
	}
}

void setup_engine_params(int eng, int *engine_pc, int *engine_prog_start_addr)
{
	switch(eng)
	{
		case 1:
			*engine_pc = ENGINE1_PC;
			*engine_prog_start_addr = ENG1_PROG_START_ADDR;
			break;
		case 2:
			*engine_pc = ENGINE2_PC;
			*engine_prog_start_addr = ENG2_PROG_START_ADDR;
			break;
		case 3:
			*engine_pc = ENGINE3_PC;
			*engine_prog_start_addr = ENG3_PROG_START_ADDR;
			break;
		default:
			break;
	}
}

static int
start_engine(struct lp8501_device_state *p_state, int eng)
{
#ifdef AUTO_INCR_WRITE
	u8 reg, address_and_data [LP8501_INSTR_LEN_PER_PAGE * 2 + 1];
	int k = 0;
#else
	u8 upper, lower;
#endif
	int i, j, ret=0;
	int page_start=0, page_end=0, engine_pc=0, engine_prog_start_addr=0;
	// configure start/end page address and engine pc/start address
	setup_page_sizes(p_state, eng, &page_start, &page_end);
	setup_engine_params(eng, &engine_pc, &engine_prog_start_addr);

	//read and write to the pc needs to be in hold mode
	lp8501_config_engine_control_mode(p_state, eng, ENGINE_CNTRL1, ENGINE_CNTRL1_HOLD);	

	//disable current engine
	lp8501_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_DISABLE);

	// configure engine_pc, program start address and page number
	lp8501_i2c_write_reg(p_state->i2c_dev, engine_pc, page_start * 16);
	lp8501_i2c_write_reg(p_state->i2c_dev, engine_prog_start_addr, page_start * 16);
	lp8501_i2c_write_reg(p_state->i2c_dev, PROG_MEM_PAGE_SELECT, page_start);
	
	// configure load program mode
	lp8501_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_LOAD);

#ifdef AUTO_INCR_WRITE
	// Enable the serial bus address auto increment
	lp8501_i2c_read_reg(p_state->i2c_dev, CONFIG, &reg);

	// Restore the expected CONFIG register value, if it's not there
	if (reg != p_state->config_reg) {
		reg = p_state->config_reg;
	}
	lp8501_i2c_write_reg(p_state->i2c_dev, CONFIG, reg | CONFIG_AUTO_INCR_ON);
#endif

	for (i = page_start; i <= page_end; i++) {

		/* Configure the memory page */
		lp8501_i2c_write_reg(p_state->i2c_dev, PROG_MEM_PAGE_SELECT, i);

		msleep(1);
		
	#ifdef AUTO_INCR_WRITE
		k = 0;
		// Assemble the data to be written
		address_and_data [k++] = PROG_MEM_START;

		for (j = 0; j < 16; j++)
		{
			// Arm has oppose upper 8 bits and lower 8 bits, needs to be reverted
			address_and_data [k++] = p_state->instruct[i*16 + j]>>8;
			address_and_data [k++] = p_state->instruct[i*16 + j];
		}

		// Write the entire page
		lp8501_i2c_write_reg_auto_incr (p_state->i2c_dev, address_and_data, k);

	#else
		for (j = 0; j < 16; j++)
		{
			// Arm has oppose upper 8 bits and lower 8 bits, needs to be reverted
			lower = p_state->instruct[i*16 + j];
			upper = p_state->instruct[i*16 + j]>>8;

			//printk(KERN_INFO "page=%d, offset=%d, instruction= %x, data= %x\n", i, j, upper, lower);

			lp8501_i2c_write_reg(p_state->i2c_dev, (PROG_MEM_START+(2*j)%32), upper);
			lp8501_i2c_write_reg(p_state->i2c_dev, (PROG_MEM_START+(2*j+1)%32), lower);
	
		}
	#endif

	}

#ifdef AUTO_INCR_WRITE
	// Disable the serial bus address auto increment
	lp8501_i2c_write_reg(p_state->i2c_dev, CONFIG, reg);
#endif
			

	// Run the program
	lp8501_config_engine_control_mode(p_state, eng, ENGINE_CNTRL1, ENGINE_CNTRL1_FREERUN);
	lp8501_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_RUN);	

	// allow the engine time to start before processing the next request
	msleep(1);

	return ret;
}

static int
eng_to_bitmask(int eng)
{
	if(eng == 1)
		return 1;
	else if(eng == 2)
		return 2;
	else if(eng == 3)
		return 4;
	else	
		return 0;	
}

static int
stop_engine(struct lp8501_device_state *p_state, int eng)
{
	int ret=0;

	// hold execution and disable the engine
	// NOTE: The user space must make sure all LEDs are put back into the proper state
	// if an engine was terminated before it finished.
	lp8501_config_engine_control_mode(p_state, eng, ENGINE_CNTRL1, ENGINE_CNTRL1_HOLD);
	lp8501_config_engine_control_mode(p_state, eng, ENGINE_CNTRL2, ENGINE_CNTRL2_DISABLE);

	// allow the engine time to stop before processing the next request
	msleep(1);

	p_state->stop_engine_state |= eng_to_bitmask(eng);

	if(p_state->stop_engine_state > 0)
	{
		wake_up_interruptible(&lp8501_wait_stop_engine);
	}

	return ret;
}

static void lp8501_process_command(struct work_struct *work)
{
	struct engine_cmd *command = container_of(work, struct engine_cmd, workitem);

	mutex_lock(&command->context->lock);

	switch(command->cmd)
	{
		case LP8501_START_ENGINE:
			start_engine(command->context, command->engine);			
			break;
		case LP8501_STOP_ENGINE:
			stop_engine(command->context, command->engine);
			break;
		default:
			break;
	}

	mutex_unlock(&command->context->lock);

	kfree(command);
}

void lp8501_allocate_workitem(struct lp8501_device_state *context, int engine, int cmd)
{
	struct engine_cmd *newcmd = kzalloc(sizeof(struct engine_cmd), GFP_KERNEL);

	newcmd->context = context;
	newcmd->engine = engine;
	newcmd->cmd = cmd;

	INIT_WORK(&newcmd->workitem, lp8501_process_command);
	schedule_work(&newcmd->workitem);
}

static void lp8501_mod_brightness(struct work_struct *work)
{
	struct lp8501_led_config *led_config = 
		container_of(work, struct lp8501_led_config, brightness_work);
	struct lp8501_device_state *state = i2c_get_clientdata(led_config->client);
		
	int pwm_value;
	int i = 0;

	if(convert_percent_to_pwm(led_config->brightness, &pwm_value) != 0)
	{
		printk(KERN_ERR "lp8501_mod_brightness: invalid brightness value (%d)\n", led_config->brightness);
		return;
	}
	
	mutex_lock(&state->lock);

	switch(led_config->hw_group)
	{
		case HW_GRP_NONE:			
			// Loop through each led within group_id
			for(i = 0; i < led_config->nleds; i++)
			{
				lp8501_i2c_write_reg(led_config->client, led_config->led_list[i].pwm_addr, pwm_value);
			}
			break;
		/* HW Groups can change brightness for all LEDs with a single command */
		case HW_GRP_1:			
			lp8501_i2c_write_reg(led_config->client, GROUP_FADER1, pwm_value);
			break;
		case HW_GRP_2:			
			lp8501_i2c_write_reg(led_config->client, GROUP_FADER2, pwm_value);
			break;	
		case HW_GRP_3:					
			lp8501_i2c_write_reg(led_config->client, GROUP_FADER3, pwm_value);
			break;
		default:
			break;
	}		
	mutex_unlock(&state->lock);
	return;
}

/* Program current control register, ramp registers, blink
 * registers, and then turn on/off the LED regulators.
 */
static void lp8501_set_brightness(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	struct lp8501_led_config *led_config;
	struct lp8501_platform_data *pdata; 
	struct lp8501_led_config *leds = NULL;

	struct i2c_client *client;

	/* Get the instance of LED configuration block. */
	led_config = container_of(led_cdev, struct lp8501_led_config, cdev);

	pdata = led_cdev->dev->platform_data;
	leds = pdata->leds;
	
	/* Sanity check. We can't be here without these set up correctly. */
	if ((!led_config) || (!led_config->client)) {
		printk(KERN_ERR "%s: Invalid LED. Can't set led brightness.\n",
		       LP8501_I2C_DRIVER);
		return;
	}

	/* Get the handle to the I2C client. */
	client = led_config->client;

	/* Change the brightness */
	led_config->brightness = value;
	schedule_work(&led_config->brightness_work);

	return;
}

/******************************************************************************
*
* lp8501_notify()
*
* Send out EV_LED event to user space.
* 
* Inputs
*   None.
*
* Returns
*   None.
*
******************************************************************************/
static void lp8501_notify(struct work_struct *work)
{
	u8 reg;
	int interrupt = 0;
	struct lp8501_device_state *p_state =
		container_of(work, struct lp8501_device_state, notify_work);

	mutex_lock(&p_state->lock);
	
	// clear interrrupt
	lp8501_i2c_read_reg(p_state->i2c_dev, STATUS, &reg);

	// figure out which engine fired the interrupt
	if(reg & 0x01)
		p_state->interrupt_state = 3;
	else if(reg & 0x02)
		p_state->interrupt_state = 2;
	else if(reg & 0x04)
		p_state->interrupt_state = 1;
	else	
		p_state->interrupt_state = 0;

	interrupt = p_state->interrupt_state;

	if(p_state->interrupt_state > 0)
	{
		wake_up_interruptible(&lp8501_wait_queue);
	}

	if(p_state->stopengine > 0)
	{			
		// If the user space requested the engine be stopped after the next interrupt
		// then call stop_engine directly and clear the appropriate bits in p_state->stopengine
		if( (p_state->stopengine & LP8501_STOP_ENGINE1) && (interrupt == 1))
		{
			p_state->stopengine = p_state->stopengine &~ LP8501_STOP_ENGINE1;
			lp8501_allocate_workitem(p_state, 1, LP8501_STOP_ENGINE);
		}
		if( (p_state->stopengine & LP8501_STOP_ENGINE2) && (interrupt == 2))
		{
			p_state->stopengine = p_state->stopengine &~ LP8501_STOP_ENGINE2;
			lp8501_allocate_workitem(p_state, 2, LP8501_STOP_ENGINE);
		}
		if( (p_state->stopengine & LP8501_STOP_ENGINE3) && (interrupt == 3))
		{	
			p_state->stopengine = p_state->stopengine &~ LP8501_STOP_ENGINE3;
			lp8501_allocate_workitem(p_state, 3, LP8501_STOP_ENGINE);
		}
	}

	mutex_unlock(&p_state->lock);

}

/******************************************************************************
*
* lp8501_interrupt()
*
* The handler for the irq when the LED instructions is finished. It report the
* the event to the input subsystem
*
* Inputs
*   irq         The IRQ number being serviced. Not used.
*   dev_id      Device ID provided when IRQ was registered
*
* Returns
*   IRQ_HANDLED
*
******************************************************************************/
static irqreturn_t lp8501_isr(int irq, void *dev_id)
{
	struct lp8501_device_state *state = dev_id;	
	schedule_work(&state->notify_work);
	return IRQ_HANDLED;
}


static int lp8501_ioctl(struct inode *ip, struct file *fp,
		     unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct lp8501_device_state *p_state = container_of(fp->f_op, struct lp8501_device_state, fops);

	if(!p_state)
	{
		printk(KERN_ERR "lp8501_ioctl: invalid context\n");
		return -EINVAL;
	}

	switch(cmd)
	{
		case LP8501_READ_PWM:
		{
			struct lp8501_read_pwm pwm;
			u8 addr;
			if(copy_from_user(&pwm, (struct read_pwm *)arg, sizeof(struct lp8501_read_pwm)))
			{
				printk(KERN_ERR "ERROR: lp8501_ioctl: LP8501_READ_PWM parameter is invalid\n");
				ret = -EINVAL;
				goto done;
			}

			addr = pwm.led + D1_PWM - 1;
			if((addr < D1_PWM) || (addr > D9_PWM))
			{
				printk(KERN_ERR "ERROR: lp8501_ioctl: LP8501_READ_PWM addr is out of range\n");
				ret = -EINVAL;
				goto done;
			}
 
			lp8501_i2c_read_reg(p_state->i2c_dev, addr, &pwm.value);

			if(copy_to_user((void *)arg, &pwm, sizeof(struct lp8501_read_pwm)))
			{
				printk(KERN_ERR "ERROR: lp8501_ioctl: LP8501_READ_PWM failed to copy result\n");
				ret = -EINVAL;
			}

			break;
			
		}	
		case LP8501_DOWNLOAD_MICROCODE:
			mutex_lock(&p_state->lock);
			if(copy_from_user(&p_state->instruct, (u16 *)arg, sizeof(u16)*INSTR_LEN))
			{
				printk(KERN_ERR "ERROR: lp8501_ioctl: LP8501_DOWNLOAD_MICROCODE parameter is invalid\n");
				ret = -EINVAL;
			}
			mutex_unlock(&p_state->lock);
			break;

		case LP8501_START_ENGINE:
		case LP8501_STOP_ENGINE:
			lp8501_allocate_workitem(p_state, (int)arg, cmd);
			break;

		case LP8501_STOP_ENGINE_AFTER_INTERRUPT:
			// lp8501 will stop the engine execution if stopengine > 0
			mutex_lock(&p_state->lock);
			switch((u8)arg)
			{
				case 1:
					p_state->stopengine |= LP8501_STOP_ENGINE1;
					break;
				case 2:
					p_state->stopengine |= LP8501_STOP_ENGINE2;
					break;
				case 3:
					p_state->stopengine |= LP8501_STOP_ENGINE3;
					break;
				default:
					ret = -EINVAL;
					break;
			}
			mutex_unlock(&p_state->lock);
			break;

		case LP8501_WAIT_FOR_INTERRUPT:
			wait_event_interruptible(lp8501_wait_queue, p_state->interrupt_state > 0);

			mutex_lock(&p_state->lock);
			if(copy_to_user((void *)arg, &p_state->interrupt_state, sizeof(int)))
			{
				printk(KERN_ERR "ERROR: lp8501_ioctl: LP8501_WAIT_FOR_INTERRUPT parameter is invalid\n");
				ret = -EINVAL;
			}

			// clear internal interrupt state
			p_state->interrupt_state = 0;
			mutex_unlock(&p_state->lock);
			break;

		case LP8501_WAIT_FOR_ENGINE_STOPPED:
			wait_event_interruptible(lp8501_wait_stop_engine, p_state->stop_engine_state > 0);

			mutex_lock(&p_state->lock);
			if(copy_to_user((void *)arg, &p_state->stop_engine_state, sizeof(int)))
			{
				printk(KERN_ERR "ERROR: lp8501_ioctl: LP8501_WAIT_FOR_ENGINE_STOPPED parameter is invalid\n");
				ret = -EINVAL;
			}

			// clear internal interrupt state
			p_state->stop_engine_state = 0;
			mutex_unlock(&p_state->lock);
			break;

		case LP8501_CONFIGURE_MEMORY:
			mutex_lock(&p_state->lock);
			if(copy_from_user(p_state->pdata.memcfg, (void *)arg, sizeof(struct lp8501_memory_config)))
			{
				printk(KERN_ERR "ERROR: lp8501_ioctl: LP8501_CONFIGURE_MEMORY parameter is invalid\n");
				ret = -EINVAL;
			}
			mutex_unlock(&p_state->lock);


			break;

		default:
			break;
	}

done: 
	return ret;

}

static int lp8501_open(struct inode *ip, struct file *fp)
{
	struct lp8501_device_state * p_state;
	int ret = 0;

	p_state = container_of(fp->f_op, struct lp8501_device_state, fops);

	if(!p_state)
	{
		printk("ERROR: lp8501_open: p_state is bad\n");
		return -EINVAL;
	}

	return ret;
}


static const struct file_operations lp8501_fops = {
	.open = lp8501_open,
	.ioctl = lp8501_ioctl,
};

static int lp8501_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lp8501_device_state *state = NULL;
	struct lp8501_platform_data *pdata = client->dev.platform_data;
	struct lp8501_led_config *leds = NULL;
	
	u8 reg;
	int i, j, ret;
	
	
	struct i2c_adapter *adapter;

	adapter = to_i2c_adapter(client->dev.parent);
	
	/* Check the platform data. */ 
	if (pdata == NULL) {
		printk(KERN_ERR "%s: missing platform data.\n",
		       LP8501_I2C_DRIVER);
		return (-ENODEV);
	}
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)){
		return -EIO;
	}
	/* Create the device state */
	state = kzalloc(sizeof(struct lp8501_device_state), GFP_KERNEL);
	if (!state) { 
		return (-ENOMEM);
	}

	/* Attach i2c_dev */
	state->i2c_dev = client;

	/* Attach platform data */
	memcpy(&state->pdata, pdata, sizeof(struct lp8501_platform_data));

	/* Init workq */
	INIT_WORK(&state->notify_work, lp8501_notify);

	/* Default interrupt state */
	state->interrupt_state = 0;
	state->stop_engine_state = 0;

	state->stopengine = 0;

	/* store fops */
	memcpy(&state->fops, &lp8501_fops, sizeof(struct file_operations));

	state->mdev.name = "lp8501";
	state->mdev.minor = MISC_DYNAMIC_MINOR;
	state->mdev.fops = &state->fops;

	/* Create misc device */
	ret = misc_register(&state->mdev);

	if(ret)
	{
		printk(KERN_ERR "LP8501: Failed to create /dev/lp8501\n");
		goto err0;
	}

#ifdef CONFIG_LEDS_LP8501_DBG
	if ((ret = lp8501_dbg_register_i2c_client(state->i2c_dev)))
		goto err1;
#endif // CONFIG_LEDS_LP8501_DBG

	
	/* Enable interrupt */

	
	ret = request_irq(client->irq, lp8501_isr,
			 IRQF_TRIGGER_FALLING, pdata->dev_name, (void *)state);
	if (ret < 0)		
		goto err1;
	
	
	/* Attach driver data. */
	i2c_set_clientdata(client, state);

	leds = pdata->leds;

	/* Init mutex */
	mutex_init(&state->lock);
	
	/* Enable LED chip. */
	lp8501_i2c_read_reg(client, ENGINE_CNTRL1, &reg);
	lp8501_i2c_write_reg(client, ENGINE_CNTRL1, (reg|0x40) );

	/* Configure power savings mode */
	lp8501_i2c_read_reg(client, CONFIG, &reg);
	lp8501_i2c_write_reg(client, CONFIG, (reg | pdata->cp_mode | pdata->power_mode) );
	state->config_reg = (reg | pdata->cp_mode | pdata->power_mode);

	/* Register to led class. */
	for (i = 0; i < pdata->nleds; i++)
	{
		/* check to see if this group can be put into a single hw group */
		if(leds[i].hw_group != HW_GRP_NONE)
		{
			/* assign hardware groups to each led in the group */
			for(j = 0; j < leds[i].nleds; j++)
			{
				
				lp8501_i2c_write_reg(client, leds[i].led_list[j].control_addr, 
						    (leds[i].hw_group << 6) );			
			}

		}

		/* Set Max current for LEDs*/
		for(j = 0; j < leds[i].nleds; j++)
		{
			lp8501_i2c_write_reg(client, leds[i].led_list[j].current_addr, 
						       leds[i].default_current);
		}

		INIT_WORK(&leds[i].brightness_work, lp8501_mod_brightness);

		if(leds[i].default_state == LED_ON)
		{
			printk(KERN_INFO "LP8501: Turning %s on to default %d duty cycle\n", 
				leds[i].cdev.name, leds[i].default_brightness);

			leds[i].brightness = leds[i].default_brightness;
			schedule_work(&leds[i].brightness_work);
		}

		leds[i].cdev.brightness_set = lp8501_set_brightness;

		/* Save the i2c client information for each led. */
		leds[i].client = client;

		/* Register this LED instance to the LED class. */
		ret = led_classdev_register(&client->dev, &leds[i].cdev);
		if (ret < 0)
			goto err2;						
	}

	return 0;

err2:
	/* Unregister previously created attribute files and return error. */
	if (i > 1) 
	{
		for (i = i - 1; i >= 0; i--) 
		{
			led_classdev_unregister(&leds[i].cdev);
		}
	}
err1:
	misc_deregister(&state->mdev);
err0:
	kfree(state);

	return (ret);
}

static int lp8501_i2c_remove(struct i2c_client *client)
{
	struct lp8501_device_state *state = i2c_get_clientdata(client);
	struct lp8501_platform_data *pdata = client->dev.platform_data;
	struct lp8501_led_config *leds = NULL;
	int i;

	cancel_work_sync(&state->notify_work);

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < pdata->nleds; i++) 
			led_classdev_unregister(&leds[i].cdev);
	}

#ifdef CONFIG_LEDS_LP8501_DBG
	lp8501_dbg_unregister_i2c_client(state->i2c_dev);
#endif // CONFIG_LEDS_LP8501_DBG

	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int lp8501_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	struct lp8501_device_state *devstate = i2c_get_clientdata(client);
	struct lp8501_platform_data *pdata = client->dev.platform_data;
	struct lp8501_led_config *leds = NULL;
	int i;

	if (devstate->suspended) {
		return 0;
	}

	/* cancel any pending work */
	cancel_work_sync(&devstate->notify_work);

	if (pdata) {
		leds = pdata->leds;
		for (i = 0; i < pdata->nleds; i++) 
			led_classdev_suspend(&leds[i].cdev);
	}

	devstate->suspended = 1;

	return 0;
}

static int lp8501_i2c_resume(struct i2c_client *client)
{
	struct lp8501_device_state *state = i2c_get_clientdata(client);
	struct lp8501_platform_data *pdata = client->dev.platform_data;
	struct lp8501_led_config *leds = NULL;
	int i;
	u8 reg;

	if (!state->suspended) {
		return 0;
	}
	
	if (pdata) {
 
		/* Re-enable LED chip. */
		lp8501_i2c_read_reg(client, ENGINE_CNTRL1, &reg);
		lp8501_i2c_write_reg(client, ENGINE_CNTRL1, (reg|0x40) );

		leds = pdata->leds;
		for (i = 0; i < pdata->nleds; i++) 
			led_classdev_resume(&leds[i].cdev);
	}
	state->suspended = 0;

	return 0;
}
#else 
#define lp8501_i2c_suspend NULL
#define lp8501_i2c_resume  NULL
#endif


static struct i2c_device_id lp8501_i2c_id[] = {
	{ LP8501_I2C_DEVICE, 0 },
	{ }

};

static struct i2c_driver lp8501_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name = LP8501_I2C_DRIVER,
	},
	.probe		= lp8501_i2c_probe,
	.remove 	= lp8501_i2c_remove,
	.suspend 	= lp8501_i2c_suspend,
	.resume 	= lp8501_i2c_resume,
	.id_table 	= lp8501_i2c_id,
};

static int __init
lp8501_module_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&lp8501_i2c_driver); 
	return (ret);
}

static void __exit
lp8501_module_exit(void)
{
	i2c_del_driver(&lp8501_i2c_driver); 
}

module_init(lp8501_module_init);
module_exit(lp8501_module_exit);


MODULE_DESCRIPTION("NS LP8501 LED driver");
MODULE_LICENSE("GPL");
