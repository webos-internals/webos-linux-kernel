/*
 * linux/include/linux/cy8c24894.h
 *
 * Driver for the CY8C24894 TP.
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Raj Mojumder <raj.mojumder@palm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#ifndef _CY8C24894_H
#define _CY8C24894_H

#include <linux/ioctl.h>
#include <linux/types.h>


#define  CY8C24894_DEVICE   "cy8c24894"
#define  CY8C24894_DRIVER   "cy8c24894"


/* IOCTLs */
#define CY8C2489_IOCTL_GET_SCANRATE		_IOR('c', 0x01, int)
#define CY8C2489_IOCTL_GET_SLEEPMODE		_IOR('c', 0x02, int)
#define CY8C2489_IOCTL_GET_VERBOSE_MODE		_IOR('c', 0x07, int)

#define CY8C2489_IOCTL_SET_SCANRATE		_IOW('c', 0x08, int)
#define CY8C2489_IOCTL_SET_SLEEPMODE		_IOW('c', 0x09, int)
#define CY8C2489_IOCTL_SET_VERBOSE_MODE		_IOW('c', 0x0e, int)
#define CY8C2489_IOCTL_SET_BYTES		_IOW('c', 0x0f, int)
#define CY8C2489_IOCTL_GET_NUM_DATA_BYTES	_IOR('c', 0x10, int)
#define CY8C2489_IOCTL_SET_NUM_DATA_BYTES	_IOW('c', 0x11, int)
#define CY8C2489_IOCTL_GET_QUERY_DATA		_IOR('c', 0x12, unsigned short)
#define CY8C2489_IOCTL_SET_VECTORS		_IOW('c', 0x13, int)
#define CY8C2489_IOCTL_SET_PROG_PHASE		_IOW('c', 0x14, int)
#define CY8C2489_IOCTL_SET_FW_DATA		_IOW('c', 0x15, int)
#define CY8C2489_IOCTL_GET_TIMESTAMP_MODE 	_IOR('c', 0x16, int)
#define CY8C2489_IOCTL_SET_TIMESTAMP_MODE 	_IOW('c', 0x17, int)


/* new ioctls */
#define CY8C2489_IOCTL_GET_WOT_SCANRATE		_IOR('c', 0x18, int)
#define CY8C2489_IOCTL_SET_WOT_SCANRATE		_IOW('c', 0x19, int)
#define CY8C2489_IOCTL_GET_FW_VERSION		_IOR('c', 0x1a, int)
#define CY8C2489_IOCTL_GET_TX8_CONFIG		_IOR('c', 0x1c, int)
#define CY8C2489_IOCTL_SET_TX8_CONFIG		_IOW('c', 0x1d, int)
#define CY8C2489_IOCTL_SET_COMMAND		_IOW('c', 0x1f, int)
#define CY8C2489_IOCTL_GET_PANEL_ID		_IOW('c', 0x20, int)
#define CY8C2489_IOCTL_GET_WOT_THRESHOLD	_IOW('c', 0x22, int)
#define CY8C2489_IOCTL_SET_WOT_THRESHOLD	_IOW('c', 0x23, int)
#define CY8C2489_IOCTL_GET_WOT_BASELINE_LO	_IOW('c', 0x25, int)
#define CY8C2489_IOCTL_GET_WOT_BASELINE_HI	_IOW('c', 0x26, int)
#define CY8C2489_IOCTL_SET_WOT_BASELINE_LO	_IOW('c', 0x27, int)
#define CY8C2489_IOCTL_SET_WOT_BASELINE_HI	_IOW('c', 0x28, int)
#define CY8C2489_IOCTL_RESET_PSOC		_IOW('c', 0x29, int)


#define CY8C2489_DEEP_SLEEP			0
#define CY8C2489_POWER_OFF_PSOC			1


/* psoc power state */
enum cy8c24894_power_state {
	CY8C2489_DEEP_SLEEP_STATE = 0,
	CY8C2489_WOT_STATE,
	CY8C2489_ACTIVE_SCAN_STATE,
	CY8C2489_HW_RESET_STATE,
	CY8C2489_IDLE_STATE,
	CY8C2489_WOT_RAW_DATA_STATE,
	CY8C2489_IDLE_NO_CMD_STATE,
 
	SIZE_CY8C2489_POWER_STATE
};

enum cy8c24894_command {
	CY8C2489_ILO_CALIBRATE = 1,
	CY8C2489_OUTPUT_IDAC,
	CY8C2489_READ_IDAC,
	CY8C2489_WRITE_IDAC_FLASH,
	CY8C2489_IDAC_CALIBRATE
};

/* Touch panel platform data structure */
struct cy8c24894_platform_data {
	char*	dev_name;   // device name
	int 	wake_tp_gpio;
	int	wot_gpio;
	int	pwr_gpio;
	int 	lvl_shift_gpio;
	int 	lvl_shift_pu_fix;
	int	poweroff_mode;
	int	(*debus)(void);
	int	(*embus)(void);
};

#endif // _CY8C24894_H
