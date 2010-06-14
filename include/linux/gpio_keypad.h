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

#ifndef _GPIO_KEYPAD_H
#define _GPIO_KEYPAD_H

/* gpio keypad platform data structure */
struct gpio_kp_config {
	int  row_num;    // number of rows in matrix
	int  col_num;    // number of cols in matrix
	int *rows;       // row gpios 
	int *cols;       // col gpios
 	int *keymap;     // keymap 
 	int  rep_period; // repeat period (msec) 
 	int  rep_delay;  // repeat delay  (msec)
	int  gpio_delay; // gpio select delay (usec)
	int  debounce;   // debounce interval (msec)
	int  wakeup_row; // single wakeup row is supported
	int  wakeup_mask;// 
};

#endif
