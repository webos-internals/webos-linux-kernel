/*
 * w1-gpio interface to platform code
 *
 * Copyright (C) 2007 Ville Syrjala <syrjala@sci.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */
#ifndef _LINUX_W1_GPIO_H
#define _LINUX_W1_GPIO_H

#define ONEWIRE_GPIO_DEVICE   "w1-gpio"
#define ONEWIRE_GPIO_DRIVER   "w1-gpio"
/**
 * struct w1_gpio_platform_data - Platform-dependent data for w1-gpio
 * @pin: GPIO pin to use
 * @is_open_drain: GPIO pin is configured as open drain
 */
struct w1_gpio_platform_data {
	unsigned int pin;
	unsigned int is_open_drain:1;
	unsigned int w1_read_bit_drive_low_time;
	unsigned int w1_read_bit_release_time;
	unsigned int w1_read_bit_delay_time;
};

#endif /* _LINUX_W1_GPIO_H */

