/*
 * drivers/media/video/omap/omap24xxcam_user.h
 *
 * Copyright (C) 2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 */

/* This is needed by user apps to use non-standard V4L2 ioctls
 * that are specific to OMAP2 camera driver.
 */
#ifndef OMAP24XXCAM_USER_H
#define OMAP24XXCAM_USER_H

#define PREVIEW_ROTATION_NO		0
#define PREVIEW_ROTATION_0		1
#define PREVIEW_ROTATION_90		2
#define PREVIEW_ROTATION_180		3
#define PREVIEW_ROTATION_270		4

#define VIDIOC_S_OVERLAY_ROT		_IOW ('O', 1,  int)
#define VIDIOC_G_OVERLAY_ROT		_IOR ('O', 2,  int)
#define VIDIOC_ISP_2ACFG        	_IOWR ('O', 6,  struct isph3a_aewb_config)
#define VIDIOC_ISP_2AREQ        	_IOWR ('O', 7,  struct isph3a_aewb_data)
#define VIDIOC_ISP_HIST_CFG		_IOWR ('O', 8,  struct isp_hist_config)
#define VIDIOC_ISP_HISTREQ	        _IOWR ('O', 9,  struct isp_hist_data)
#define VIDIOC_ISP_STANDBY		_IOWR ('O', 10, int)



#define V4L2_BUF_TYPE_STILL_CAPTURE 	V4L2_BUF_TYPE_PRIVATE

#endif	/* ifndef OMAP24XXCAM_USER_H */
