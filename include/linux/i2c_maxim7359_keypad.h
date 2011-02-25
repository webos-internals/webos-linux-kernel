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

#ifndef _MAXIM7359_KEYPAD_H
#define _MAXIM7359_KEYPAD_H

#define  MAXIM7359_I2C_DEVICE   "MAXIM7359"
#define  MAXIM7359_I2C_DRIVER   "MAXIM7359"

/* maxim7359 keypad platform data structure */
struct maxim7359_platform_data {
    char *dev_name;   // device name
    int   row_num;    // number of rows in matrix
    int   col_num;    // number of cols in matrix
    int  *keymap;     // keymap 
    int   key_prox_timeout; // key proximity timeout
    int   key_prox_width;   // key proximity width
    void *key_prox_map;     // key proximity map
    int   rep_period; // repeat period (msec) 
    int   rep_delay;  // repeat delay  (msec)
    int   hw_debounce;   // hw debounce interval (msec)
    int   sw_debounce;   // sw debounce interval (msec)
    int   wakeup_en;  // wake up source enable/disable
};

#endif // MAXIM7359
