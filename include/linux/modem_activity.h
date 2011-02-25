/*
 * linux/modem_activity.h
 *
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

#ifndef __MODEM_ACTIVITY_INCLUDED__
#define __MODEM_ACTIVITY_INCLUDED__

#ifdef __KERNEL__

#define GPIO_FLG_AWM_CONFIGURED   (1 << 0) // assume app_wake_modem gpio configured
#define GPIO_FLG_MWA_CONFIGURED   (1 << 1) // assume modem_wake_app gpio configured 
#define GPIO_FLG_MWU_CONFIGURED   (1 << 2) // assume modem_wake_usb gpio configured 

// activity monitor configuration 
struct  modem_activity_monitor_config {
	int  app_wake_modem_gpio; // is required 
	int  modem_wake_app_gpio; // at least one is required
	int  modem_wake_usb_gpio; // at least one is required
	u32  gpio_flags;
	u32  uart;       // ports/sources to monitor                
	u32  timeout;    // Timeout for de-asserting APP_WAKE_MODEM 
};

void modem_activity_set_uart_handler ( u32 port,
                                       int (*busy)  ( unsigned long param ),
                                       int (*sleep) ( unsigned long param ),
                                       int (*asleep)( unsigned long param ),
                                       int (*wkup)  ( unsigned long param ),
                                       int (*awake) ( unsigned long param ),
                                       unsigned long param ); 

int  modem_activity_touch_uart_port  ( u32 port, int timeout, const char *source );

void modem_activity_set_usb_dev (unsigned long udev);
int modem_activity_usb_suspend (void);
int modem_activity_usb_resume (int msec);
int modem_activity_handshake_is_enabled (void);

#endif  // __KERNEL__
#endif  // __MODEM_ACTIVITY_INCLUDED__

