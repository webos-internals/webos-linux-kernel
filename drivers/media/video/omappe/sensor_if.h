
/*
 * drivers/media/video/omap/sensor_if.h
 *
 * Copyright (C) 2004-2007 Texas Instruments, Inc. 
 * 
 * Sensor interface to OMAP camera capture drivers
 * Sensor driver should implement this interface
 *
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 *
 *  Oct 2004:  v1
 *  Dec 2005:  v2 added double context and crop
 *  Jan 2007:  v3 added raw sensor 
 */
 
#ifndef OMAP_SENSOR_IF_H
#define OMAP_SENSOR_IF_H
 
#define LEN_SENSOR_NAME		31

#define PAR_MODE_NOBT8			0
#define PAR_MODE_NOBT10		1
#define PAR_MODE_NOBT12		2
#define PAR_MODE_BT8			4
#define PAR_MODE_BT10			5

#define SENSOR_PARALLEL		0
#define SENSOR_SERIAL1			1
#define SENSOR_SERIAL2			2

#define SENSOR_ISP			0
#define SENSOR_RAW			1

#define SYNC_ACTIVE_HIGH		0
#define SYNC_ACTIVE_LOW		1

#define V4L2_BUF_TYPE_STILL_CAPTURE 	V4L2_BUF_TYPE_PRIVATE

struct camera_sensor {
	unsigned int version;
	char name[LEN_SENSOR_NAME + 1];

	int sensor_type;   /* isp or raw sensor */
	int sensor_interface;   /* parallel or serial sensor */

	int parallel_mode;  /* parallel I/F mode */
	int hs_polarity;    /* horizontal sync polarity */ 	
	int vs_polarity;    /* vertical sync polarity */
	int image_swap;	    /* image swap or not */
	int bt_correction;  /* BT correction enabled or not */

	/* init the sensor with the passed pix format. A none zero private
	   pointer must be returned on success. The same pointer is passed
	   back for all other functions. This gives a sensor driver the
	   chance to handle multiple sensor. */ 
	void *(*init)(struct v4l2_pix_format *, struct v4l2_pix_format *);
	/* clean up the sensor */
	int (*cleanup)(void *);

	/* These are for power management */
	int (*power_on)(void *);
	int (*power_off)(void *);

	/* Handle V4L2 fmt IOCTLs.*/ 
	/* Raw sensor driver doesn't implement enum_pixformat. camera driver should
	   handle it */
	int (*enum_pixformat)(struct v4l2_fmtdesc *, void *);
	/* Raw sensor driver needs to implement try_format. But this is filtered by
	   camera driver who konws ISP */
	int (*try_format) (struct v4l2_pix_format *, void *);

	/* Calculated the needed xclk for the pix format passed and the
	   desired capture rate. */ 
	unsigned long (*calc_xclk) (struct v4l2_pix_format *,
		struct v4l2_fract *, void *);

	/* Configure the sensor to generate the passed pix format at the
	   passed capture rate with the passed xclk */   
	int (*configure) (struct v4l2_pix_format *, unsigned long,
					  struct v4l2_fract *, void *);
	/* These handle V4L2 control IOCTLs */
	int (*query_control) (struct v4l2_queryctrl *, void *);
	int (*get_control) (struct  v4l2_control *, void *);
	int (*set_control) (struct  v4l2_control *, void *);

	/* Optionally. These handle V4L2 crop IOCTLs with 
	   V4L2_BUF_TYPE_VIDEO_CAPTURE buffer type */
	int (*cropcap) (struct v4l2_cropcap *, void *);
	int (*get_crop) (struct  v4l2_crop *, void *);
	int (*set_crop) (struct  v4l2_crop *, void *);

	/* These are only for those sensors that use a different sensor context
	   for still image capture. A simple sensor driver doesn't have to
	   implement them. */
	int (*try_format_still_capture) (struct v4l2_pix_format *, void *);
	int (*configure_still_capture) (struct v4l2_pix_format *, unsigned long,
					  struct v4l2_fract *, void *);
	unsigned long (*calc_xclk_still_capture) (struct v4l2_pix_format *,
		struct v4l2_fract *, void *);
	int (*enter_still_capture) (int, void *);
	int (*exit_still_capture) (void *);
	int (*set_exposure_time) (int, u32);
	int (*set_gain) (u16);
	int (*restore) (void *);
};


int omap_cam_register_sensor(struct camera_sensor *sensor); 
int omap_cam_unregister_sensor(struct camera_sensor *sensor); 

#endif
