/*
 * linux/include/linux/isl29018_prox_sensor.h
 *
 * Driver for the ISL29018 Proximity Sensor.
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Dmitry Fink <dmitry.fink@palm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#ifndef _ISL29018_PROXIMITY_SENSOR_H
#define _ISL29018_PROXIMITY_SENSOR_H

#define  ISL29018_DEVICE   "isl29018_proximity"
#define  ISL29018_DRIVER   "isl29018_proximity"

/* proximity sensor platform data structure */
struct isl29018_platform_data {
	char*	dev_name;   // device name
	int		gpio;

	unsigned int threshold;	// Default interrupt threshold
	unsigned debounce:2; // Fire interrupt after 0=1 conversion, 1=4, 2=8, 3=16

	unsigned prox_gain:2;	// 0=1000lux, 1=4000lux, 2=16000lux, 3=64000lux
	unsigned prox_width:2; // 0=16(15)bit, 1=12bit, 2=8bit, 3=4bit
	unsigned prox_amp:2; // 0=12.5mA, 1=25mA, 2=50mA, 3=100mA
	unsigned prox_mod_freq:1; // 0=DC  1=360 KHz
	int scan_off_time_no_det;
	int scan_off_time_det;
	int als_min_poll_interval;
	int prox_min_scan_time;
	unsigned als_gain:2;	// 0=1000lux, 1=4000lux, 2=16000lux, 3=64000lux
	unsigned als_width:2; // 0=16(15)bit, 1=12bit, 2=8bit, 3=4bit
};


/* ISL29018 light sensor platform data structure */
struct isl29018_light_sensor_platform_data {
	char*	dev_name;   // device name
	struct input_dev *	idev;	// input dev pointer
};

#endif // _ISL29018_PROXIMITY_SENSOR
