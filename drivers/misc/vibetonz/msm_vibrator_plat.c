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

#include <linux/i2c_lm8502_led.h>

#include "vibrator_plat.h"

#undef  VIBE_DEBUG
// #define VIBE_DEBUG

struct vib_context
{
	struct vibrator_platform_data	data;
	int				init_done;
	unsigned int			duty_cycle;
	int				direction;
};


int vibrator_init(struct vib_context* io_p_context) 
{
	int rc = 0;

#ifdef VIBE_DEBUG
	printk("In %s...\n", __func__);
#endif
	if (!io_p_context->init_done) {
		if (!lm8502_vib_enable(true))
			io_p_context->init_done = 1;
		else
			rc = -EIO;
	}
	return rc;
}


int vibrator_deinit(struct vib_context* io_p_context) 
{
	int rc = 0;

#ifdef VIBE_DEBUG
	printk("In %s...\n", __func__);
#endif
	if (io_p_context->init_done) {
		if (!lm8502_vib_start(false))
			io_p_context->init_done = 0;
		else
			rc = -EIO;
	}
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

			if (!io_p_context->init_done) {
				vibrator_init (io_p_context);
			}

			lm8502_vib_start (true);
			lm8502_vib_direction (dir_control == 1);

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

			/* If the vibrator was turned off by zero duty cycle last time, turn it on, for a new duty cycle value */
			if (!io_p_context->duty_cycle) {
				vibrator_init (io_p_context);
			}

			lm8502_vib_duty_cycle (duty_cycle);

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
#ifdef VIBE_DEBUG
	printk("In %s enable: %d ...\n", __func__, enable);
#endif
	if (enable) {
		if (!lm8502_vib_enable (enable) ) {
			lm8502_vib_start (enable);
		}
	} else {
		if (!lm8502_vib_start (enable) ) {
			lm8502_vib_enable (enable);
		}
	}
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

/*
 * Currently there is no need of msm specific
 * suspend / resume, as vibrator driver
 * handles suspend / resume by itself.
 */
int plat_vibrator_suspend(void)
{
	return 0;
}
int plat_vibrator_resume(void)
{
	return 0;
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


