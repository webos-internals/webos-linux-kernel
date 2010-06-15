/*
 * Copyright (C) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 * Author: Haibo Jeff Zhong <hzhong@qualcomm.com>
 *
 */
#ifndef __LINUX_MSM_CAMERA_SENSOR_H
#define __LINUX_MSM_CAMERA_SENSOR_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif
#include <linux/ioctl.h>
#include <asm/sizes.h>

#define MSM_CAMSENSOR_IOCTL_MAGIC 's'

#define MSM_CAMSENSOR_IO_CFG \
	_IOW(MSM_CAMSENSOR_IOCTL_MAGIC, 1, unsigned)

#define MSM_CAMERA_CONFIG_CURRENT \
	_IOW(MSM_CAMSENSOR_IOCTL_MAGIC, 2, unsigned)

#define MSM_CAMERA_READ_SENSOR_REGISTER \
	_IOWR(MSM_CAMSENSOR_IOCTL_MAGIC, 3, unsigned)

#define MSM_CAMERA_CONFIG_CURRENT_EX \
	_IOW(MSM_CAMSENSOR_IOCTL_MAGIC, 4, unsigned)


#define MAX_SENSOR_NUM  1
#define MAX_SENSOR_NAME 32

enum sensor_cfg_t {
	CFG_SET_MODE,
	CFG_SET_EFFECT,
	CFG_START,
	CFG_PWR_UP,
	CFG_PWR_DOWN,
	CFG_WRITE_EXPOSURE_GAIN,
	CFG_SET_DEFAULT_FOCUS,
	CFG_MOVE_FOCUS,
	CFG_REGISTER_TO_REAL_GAIN,
	CFG_REAL_TO_REGISTER_GAIN,
	CFG_SET_FPS,
	CFG_SET_PICT_FPS,
	CFG_SET_BRIGHTNESS,
	CFG_SET_CONTRAST,
	CFG_SET_ZOOM,
	CFG_SET_EXPOSURE_MODE,
	CFG_SET_WB,
	CFG_SET_ANTIBANDING,
	CFG_SET_EXP_GAIN,
	CFG_SET_PICT_EXP_GAIN,

	CFG_GET_PICT_FPS,
	CFG_GET_PREV_L_PF,
	CFG_GET_PREV_P_PL,
	CFG_GET_PICT_L_PF,
	CFG_GET_PICT_P_PL,

	CFG_GET_PICT_MAX_EXP_LC,
	CFG_SET_MODE_EX,
	CFG_MAX
	
};

enum sensor_move_focus_t {
  MOVE_NEAR,
  MOVE_FAR
};

enum sensor_mode_t {
	SENSOR_PREVIEW_MODE,
	SENSOR_SNAPSHOT_MODE,
	SENSOR_RAW_SNAPSHOT_MODE
};

enum sensor_resolution_t {
	SENSOR_QTR_SIZE,
	SENSOR_FULL_SIZE,
	SENSOR_INVALID_SIZE,
};

enum camera_effect_t {
	CAMERA_EFFECT_MIN_MINUS_1,
	CAMERA_EFFECT_OFF = 1,  /* This list must match aeecamera.h */
	CAMERA_EFFECT_MONO,
	CAMERA_EFFECT_NEGATIVE,
	CAMERA_EFFECT_SOLARIZE,
	CAMERA_EFFECT_PASTEL,
	CAMERA_EFFECT_MOSAIC,
	CAMERA_EFFECT_RESIZE,
	CAMERA_EFFECT_SEPIA,
	CAMERA_EFFECT_POSTERIZE,
	CAMERA_EFFECT_WHITEBOARD,
	CAMERA_EFFECT_BLACKBOARD,
	CAMERA_EFFECT_AQUA,
	CAMERA_EFFECT_MAX_PLUS_1
};

struct sensor_pict_fps {
	uint16_t prevfps;
	uint16_t pictfps;
};

struct exp_gain_cfg {
	uint16_t gain;
	uint32_t line;
};

struct focus_cfg {
	int32_t steps;
	enum sensor_move_focus_t dir;
};

struct fps_cfg {
	uint16_t f_mult;
	uint16_t fps_div;
	uint32_t pict_fps_div;
};

enum flash_mode_cfg{
 	DEFAULT,
	PREFLASH,
	MAINFLASH,	
};

struct flash_cfg{
	uint16_t flash_led_mode_type;
	uint16_t flash_led_polarity_type;
	uint16_t flash_led_current_ma;
	enum flash_mode_cfg cam_flash_mode ;
};

struct register_cfg{
	uint16_t register_address;
	uint16_t register_value;
};


struct camd_reg_cfg{
	struct register_cfg* ov_reg;
	uint16_t length;		
};

struct sensor_cfg_data_t {
	enum sensor_cfg_t  cfgtype;
	enum sensor_mode_t mode;
	enum sensor_resolution_t rs;

	union {
		int8_t effect;
		uint16_t prevl_pf;
		uint16_t prevp_pl;
		uint16_t pictl_pf;
		uint16_t pictp_pl;
		uint32_t pict_max_exp_lc;
		uint16_t p_fps;
		struct sensor_pict_fps gfps;
		struct exp_gain_cfg    exp_gain;
		struct focus_cfg       focus;
		struct fps_cfg	       fps;
		struct flash_cfg       flash;
		struct register_cfg    reg;
		struct camd_reg_cfg    camd_reg;
	} cfg;

	

};


enum sensor_get_info_t {
	GET_NAME,
	GET_PREVIEW_LINE_PER_FRAME,
	GET_PREVIEW_PIXELS_PER_LINE,
	GET_SNAPSHOT_LINE_PER_FRAME,
	GET_SNAPSHOT_PIXELS_PER_LINE,
	GET_SNAPSHOT_FPS,
	GET_SNAPSHOT_MAX_EP_LINE_CNT,
};

struct sensor_info_t {
	char name[MAX_SENSOR_NAME];
};

struct msm_camsensor_info_t {
	int16_t num;
	struct sensor_info_t sensor[MAX_SENSOR_NUM];
};
#endif /* __LINUX_MSM_CAMERA_SENSOR_H */

