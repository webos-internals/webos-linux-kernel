/*
 * drivers/video/omap/lcd.h
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Author: john chen (jchen1996@gmail.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef _LCD_H_
#define _LCD_H_

#define DISPLAY_NAME_SIZE	16

struct display_device;

struct display_device_ops
{
	/**
	 * sets the state of the LCD.  1=on, 0=off
	 */
	void (*display_device_set_state)(struct display_device *, unsigned int);
	/**
	 * gets the state of the LCD.
	 * @return -1=Error, 0=off, 1=on
	 */
	int (*display_device_get_state)(struct display_device *);
	/**
	 * sets the brightness of the LCD. (between 0 and 100)
	 */
	void (*display_device_set_brightness)(struct display_device *, int);
	/**
	 * gets the current brightness of the LCD.
	 * @return brightness
	 */
	int (*display_device_get_brightness)(struct display_device *);

};

struct display_device
{
	struct class_device class_dev;

	char name[DISPLAY_NAME_SIZE];

	struct mutex ops_lock;
	struct display_device_ops *ops;
};

struct lcd_backlight_ops
{
	/**
	 * sets the display to a particular brightness
	 */
	void (*bl_set_state)(void*, unsigned int);

	/**
	 * sets and gets the display brightness
	 */
	void (*bl_set_brightness)(void*, int);
	int  (*bl_get_brightness)(void*);

	void (*bl_set_max_brightness)(void*);
};

struct lcd_panel_ops
{
	/* turns the panel on (1) or off(0) */
	void (*pnl_set_state)(void*, unsigned int);
};

struct lcd_controller_ops
{
	/* turns the controller on (1) or off(0) */
	void (*ctrl_set_state)(void*, unsigned int);
};


extern struct display_device *display_device_register(const char *name,
				struct device *dev,
				struct display_device_ops *ops,
				void *devdata);
extern void display_device_unregister(struct display_device *disp);

extern void add_backlight_device(struct device *dev,
				struct lcd_backlight_ops *ops,
				struct device *parent);
extern void remove_backlight_device(struct device *dev, struct device *parent);

extern void add_panel_device(struct device *dev,
				struct lcd_panel_ops *ops,
				struct device *parent);
extern void remove_panel_device(struct device *dev, struct device *parent);

extern void add_controller_device(struct device *dev,
				struct lcd_controller_ops *ops,
				struct device *parent);
extern void remove_controller_device(struct device *dev, struct device *parent);

#define to_display_device(d) container_of(d, struct display_device, class_dev)

#endif /* _DISPLAY_H_ */
