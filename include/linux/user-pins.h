/*
 * Copyright (C) 2008-2009 Palm, Inc
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
 *
 */

#ifndef  __USER_PINS_INCLUDED__
#define  __USER_PINS_INCLUDED__

#include <linux/interrupt.h>

struct user_pin {
	const char *name;    // pin name 
	int gpio;            // gpio num/id
	int options;         // options
	int act_level;       // active level
	int direction;       // 1 - an input, 0 - output
	int def_level;       // default level: 0, 1 or -1 if undefined
	int sysfs_mask;      // sysfs file mode
	char *pin_mode;      // board specific pin mode
	int irq_config;      //
	int irq_handle_mode; // irq_handle_mode: 0 undef, 1 auto, 2 no
};

struct user_pin_set {
	const char  *set_name;   // pin set name  
	int          num_pins;   // number of pins in the group 
	struct user_pin *pins;   // pins array.
};

struct user_pins_platform_data {
	int              num_sets;   // number of pin sets 
	struct user_pin_set *sets;   // pin sets.
};

/* Pin option constants */
#define PIN_READ_ONLY		(1 << 0)    //  pin is read only
#define PIN_WAKEUP_SOURCE	(1 << 1)    //  pin is a wakeup source

#define IRQ_HANDLE_NONE		(0)         //  IRQ handling is not defined
#define IRQ_HANDLE_AUTO		(1 << 0)    //  IRQ handling is automatic
#define IRQ_HANDLE_OFF		(1 << 1)    //  IRQ handling is off


#endif // __USER_PINS_INCLUDED__
