 /* 
 * linux/drivers/misc/msm_vibrator_plat.c
 * 
 * MSM Vibrator platform-dependent file.
 * 
 * Copyright (C) 2008 Palm, Inc. 
 * 
 * This program is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public License. 
 */ 
 
#include <linux/module.h> 
#include <linux/init.h> 
#include <linux/types.h> 
#include <linux/kernel.h> 
#include <linux/platform_device.h> 
#include <linux/errno.h> 

#include <asm/io.h> 
#include <asm/mach-types.h> 

#include <mach/gpio.h> 
#include <linux/msm_pwm.h>

#include "vibrator_plat.h"

#undef  VIBE_DEBUG 
//#define VIBE_DEBUG 

struct vib_context
{
	struct vibrator_platform_data	data;
	int				init_done;
	unsigned int			duty_cycle;
	int				direction;
};


int vibrator_init(struct vib_context* io_p_context) 
{
	int rc = 0, on = 1;

#ifdef VIBE_DEBUG
	printk("In %s...\n", __func__);
#endif

	msm_pwm_enable ();

	// Configure the MPP7 as an analog output with the reference voltage as 1.25 V
	if(io_p_context->data.power)
		io_p_context->data.power (on);
	
	gpio_set_value(io_p_context->data.vibrator_enable_gpio, 0);
	io_p_context->init_done = 1;

	return rc;
}


int vibrator_deinit(struct vib_context* io_p_context) 
{
	int rc = 0, off = 0;

#ifdef VIBE_DEBUG
	printk("In %s...\n", __func__);
#endif
	msm_pwm_disable ();
	
	// Configure the MPP7 as an analog output with the reference voltage as 1.25 V
	if(io_p_context->data.power)
		io_p_context->data.power (off);
	
	gpio_set_value(io_p_context->data.vibrator_enable_gpio, 0);
	io_p_context->init_done = 0;

	return rc;
}


int vibrator_set_direction(struct vib_context* io_p_context, int dir_control)
{
	int rc = 0;
	unsigned int duty_cycle = io_p_context->duty_cycle;

#ifdef VIBE_DEBUG
	printk("In %s...\n", __func__);
#endif

	/* Set the direction only if the duty cycle is non-zero */
	if (duty_cycle) {

		/* Turn off the vibrator if the direction is set as zero */
		if (!dir_control) {
		
		#ifdef VIBE_DEBUG
			printk("In %s: stop...\n", __func__);
		#endif
			vibrator_deinit (io_p_context);
		} 
		else {
		
			if (dir_control * io_p_context->direction < 0) {

			#ifdef VIBE_DEBUG
				printk("In %s: reverse direction ...\n", __func__);
			#endif

			} else {
			#ifdef VIBE_DEBUG
				printk("In %s: same direction ...\n", __func__);
			#endif
			}

			/* Just to make the MSM vibrator duty cycle look like OMAP, convert the 0 - 100 duty cycle numbers
			to suit MSM, wherein duty cycle > 50 runs in one direction, and duty cycle < 50 runs in other direction */
			duty_cycle = 50 + ( (dir_control > 0) ? 1 : -1) * (duty_cycle / 2);

			/* turn h-bridge power control on */
			if (!io_p_context->init_done) {
				vibrator_init (io_p_context);
			}
			msm_pwm_configure ( (u16) duty_cycle);
			gpio_set_value(io_p_context->data.vibrator_enable_gpio, 1);
			
		}
		// Store the direction
		io_p_context->direction = dir_control;
	}

	return rc;
}

int vibrator_set_duty_cycle(struct vib_context* io_p_context, unsigned int duty_cycle)
{
	int rc = 0;
	u16 value = (u16) (duty_cycle);

	if (value > 100) {
		value = 100;
	}


	/* Turn off the vibrator if the duty cycle is zero */
	if (!value) {
		vibrator_deinit (io_p_context);
	} else {

		if (io_p_context->direction) {

			value = 50 + ( (io_p_context->direction > 0) ? 1 : -1) * (value / 2);
			
			/* If the vibrator was turned off by zero duty cycle last time, turn it on, for a new duty cycle value */
			if (!io_p_context->duty_cycle) {
				vibrator_init (io_p_context);
			}

			/* If the vibrator is initialized, configure the pwm */
			if (io_p_context->init_done) {
				msm_pwm_configure ( (u16) value);
			}

			/* If the vibrator was turned off by zero duty cycle last time, turn it on, for a new duty cycle value */
			if (!io_p_context->duty_cycle) {
				gpio_set_value(io_p_context->data.vibrator_enable_gpio, 1);
			}
		}
		// Store the duty cycle, even if the direction is zero
		io_p_context->duty_cycle = duty_cycle;
	}

#ifdef VIBE_DEBUG
	printk("In %s: input: %u, reg_val: %u\n", __func__, duty_cycle, value);
#endif
	return rc;
}

int vibrator_enable(struct vib_context* io_p_context, int enable)
{
	gpio_set_value(io_p_context->data.vibrator_enable_gpio, !!enable);
	return 0;
}

static struct vib_context vib_data;
/*
 * Abstraction layer for vibetonz - the api layer is all wrong, but will get
 * us started in the meantime.
 */
int plat_vibrator_register(struct vibrator_platform_data* i_p_data)
{
	memcpy(&(vib_data.data), i_p_data, sizeof(struct vibrator_platform_data));
	return 0;
}
int plat_vibrator_init()
{
	return vibrator_init(&(vib_data));
}
int plat_vibrator_deinit(void)
{
	return vibrator_deinit(&(vib_data));
}
int plat_vibrator_set_direction(int dir_control)
{
	return vibrator_set_direction(&(vib_data), dir_control);
}
int plat_vibrator_set_duty_cycle(unsigned int duty_cycle)
{
	return vibrator_set_duty_cycle(&(vib_data), duty_cycle);
}
int plat_vibrator_enable(int enable)
{
	return vibrator_enable(&(vib_data), enable);
}


