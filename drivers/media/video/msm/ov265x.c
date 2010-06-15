/*
 * Copyright (C) 2008 Palm, Inc.
 * Author: Dmitry Fink <dmitry.fink@palm.com>
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
 *
 */



#include <asm/arch/gpio.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <media/msm_camera_sensor.h>
#include <asm/arch/camera.h>
//#include <asm/arch/msm_camio.h>
#include "ov265x.h"

/*  I2C Address of the Sensor  */
#define OV265X_I2C_ADDR                 (0x60 >> 1)


/* Model ID address and value */
#define OV265X_ADDR_PIDH                (0x300A)
#define OV265X_ADDR_PIDL                (0x300B)



/*CAM SENSOR SPECIFICS */

#define  DEFAULT_SVGA_LINE_WIDTH  	  970
#define  DEFAULT_UXGA_LINE_WIDTH  	  970
#define  DEFAULT_UXGA_MAXIMUM_SHUTTER 	  1236
#define  PREVIEW_PCLK_FREQUENCY 	  72000000 
#define  CAPTURE_PCLK_FREQUENCY		  24000000
#define  CAPTURE_MAX_GAIN 		  1024


/* CAM SENSOR SPECIFIC REGISTER ADDRESS*/

#define SHUTTERL 		(0x3003)
#define SHUTTERH 		(0x3002)
#define GAIN	 		(0x3000)
#define AUTOGAINEXPOSURE	(0x3013)
#define LIGHTFREQUENCY		(0x308e)

#define PREFLASH_MINCURRENT 200
#define PREFLASH_MAXCURRENT 300

#define MAINFLASH_MINCURRENT 400
#define MAINFLASH_MAXCURRENT 600

#undef OVPRINT 
//#define OVPRINT 1

#ifdef OVPRINT
	#define printov(fmt, args...) printk(KERN_ERR "msm_camera: " fmt, ##args)

#else

 	#define printov(fmt, args...)
#endif

static uint8_t  preview_shutterh, preview_shutterl, preview_Gain;

static uint16_t enable_flash;
static uint8_t gshutterh;
static uint8_t gshutterl;
static uint16_t gshutter; 
static uint8_t gpreview_gain;

struct ov265x_work_t {
	struct work_struct work;
};

struct ov265x_ctrl_t {
	int8_t  opened;
	struct  msm_camera_device_platform_data *sensordata;
	struct  ov265x_work_t *sensorw;
	struct  i2c_client *client;
};

static struct ov265x_ctrl_t *ov265x_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(ov265x_wait_queue);
DECLARE_MUTEX(ov265x_sem);

static struct register_address_value_pair_t init_reg_settings_array[] = {

	{0x3012 , 0x80},
	{0x300e , 0x34},
	{0x300f , 0xa6},
	{0x3010 , 0x81}, 
	{0x3011 , 0x0}, 
	{0x3012 , 0x10}, 
	{0x3013 , 0xf7}, 
	{0x3014 , 0x6d}, 
	{0x3015 , 0x23}, 
	{0x3016 , 0x42}, 
	{0x3017 , 0x40}, 
	{0x3018 , 0x2c}, 
	{0x3019 , 0x28}, 
	{0x301a , 0x61}, 
	{0x301c , 0x3}, 
	{0x301d , 0x4}, 
	{0x3020 , 0x1}, 
	{0x3021 , 0x18}, 
	{0x3022 , 0x0}, 
	{0x3023 , 0x6}, 
	{0x3024 , 0x6}, 
	{0x3025 , 0x58}, 
	{0x3026 , 0x2}, 
	{0x3027 , 0x5e}, 
	{0x302a , 0x2}, 
	{0x302b , 0x6a}, 
	{0x3030 , 0x28}, 
	{0x3031 , 0xbe}, 
	{0x3032 , 0x7d}, 
	{0x3033 , 0x14}, 
	{0x304a , 0x30}, 
	{0x304b , 0x02},
	{0x304c , 0x00},
	{0x304d , 0x62}, 
	{0x304e , 0x88}, 
	{0x304f , 0xff}, 
	{0x3069 , 0x84}, 
	{0x306a , 0xc}, 
	{0x306d , 0x0}, 
	{0x306f , 0x14}, 
	{0x3070 , 0xba}, 
	{0x3072 , 0x9c}, 
	{0x3076 , 0x6a}, 
	{0x3079 , 0x0}, 
	{0x307c , 0x10}, 
	{0x307d , 0x20}, 
	{0x307e , 0xe5}, 
	{0x3082 , 0x1}, 
	{0x3087 , 0x2}, 
	{0x3088 , 0x3}, 
	{0x3089 , 0x20}, 
	{0x308a , 0x2}, 
	{0x308b , 0x58}, 
	{0x308c , 0x80}, 
	{0x308d , 0xe}, 
	{0x308e , 0x0}, 
	{0x308f , 0x0}, 
	{0x3090 , 0x33}, 
	{0x3091 , 0xc0}, 
	{0x3093 , 0x0},
	{0x3095 , 0x0},
	{0x3096 , 0x16},
	{0x3097 , 0x1a},
	{0x30a2 , 0x45},
	{0x30a3 , 0x11},
	{0x30a7 , 0x1}, 
	{0x30a8 , 0x54},  
	{0x30aa , 0x52}, 
	{0x30ac , 0x42}, 
	{0x30af , 0x00}, 
	{0x30b0 , 0xff}, 
	{0x30b1 , 0xff}, 
	{0x30b2 , 0x2c}, 
	{0x30d1 , 0x8}, 
	{0x30d9 , 0x8c}, 
	{0x30f0 , 0x0}, 
	{0x30f1 , 0x82}, 
	{0x30f3 , 0x83}, 
	{0x30f4 , 0x1}, 
	{0x3100 , 0x0}, 
	{0x3300 , 0xfc}, 
	{0x3301 , 0xff}, 
	{0x3302 , 0x11}, 
	{0x3316 , 0x64}, 
	{0x3317 , 0x25}, 
	{0x3318 , 0x80}, 
	{0x3319 , 0x8}, 
	{0x331a , 0x64}, 
	{0x331b , 0x4b}, 
	{0x331c , 0x0}, 
	{0x331d , 0x38}, 
	{0x3320 , 0xfa}, 
	{0x3324 , 0x97}, 
	{0x3327 , 0x10}, 
	{0x3328 , 0x16}, 
	{0x3329 , 0x23}, 
	{0x332a , 0x58},
	{0x332b , 0x50}, 
	{0x332c , 0xbe},
	{0x332d , 0xce}, 
	{0x332e , 0x28}, 
	{0x332f , 0x32}, 
	{0x3330 , 0x4d}, 
	{0x3331 , 0x4e}, 
	{0x3332 , 0xf0}, 
	{0x3333 , 0xa}, 
	{0x3340 , 0x5}, 
	{0x3341 , 0xd}, 
	{0x3342 , 0x1c}, 
	{0x3343 , 0x29}, 
	{0x3344 , 0x35}, 
	{0x3345 , 0x42}, 
	{0x3346 , 0x4c}, 
	{0x3347 , 0x56}, 
	{0x3348 , 0x61}, 
	{0x3349 , 0x74}, 
	{0x334a , 0x86}, 
	{0x334b , 0x96}, 
	{0x334c , 0xb6}, 
	{0x334d , 0xd1}, 
	{0x334e , 0xe5}, 
	{0x334f , 0x24},
	{0x3350 , 0x31}, 
	{0x3351 , 0x24}, 
	{0x3352 , 0x44}, 
	{0x3353 , 0x19}, 
	{0x3354 , 0x0}, 
	{0x3355 , 0x84}, 
	{0x3356 , 0x31}, 
	{0x3357 , 0x24}, 
	{0x3358 , 0x44}, 
	{0x3359 , 0x15}, 
	{0x335a , 0x0}, 
	{0x335b , 0x84}, 
	{0x335c , 0x31}, 
	{0x335d , 0x24}, 
	{0x335e , 0x44}, 
	{0x335f , 0x13}, 
	{0x3360 , 0x0}, 
	{0x3361 , 0x84}, 
	{0x3362 , 0x90}, 
	{0x3363 , 0xfc}, 
	{0x3364 , 0xff}, 
	{0x3365 , 0xf}, 
	{0x3366 , 0x0}, 
	{0x336a , 0x3c}, 
	{0x3370 , 0xd0}, 
	{0x3371 , 0x0}, 
	{0x3372 , 0x0}, 
	{0x3373 , 0x30}, 
	{0x3374 , 0x30}, 
	{0x3375 , 0x2}, 
	{0x3376 , 0x6}, 
	{0x3377 , 0x0}, 
	{0x3378 , 0x30}, 
	{0x3379 , 0x70}, 
	{0x3380 , 0x20}, 
	{0x3381 , 0x64}, 
	{0x3382 , 0x8}, 
	{0x3383 , 0x2a}, 
	{0x3384 , 0xc0}, 
	{0x3385 , 0xec}, 
	{0x3386 , 0xf2}, 
	{0x3387 , 0xf8}, 
	{0x3388 , 0x8}, 
	{0x3389 , 0x98},
	{0x338a , 0x0}, 
	{0x338b , 0x12}, 
	{0x338c , 0x38}, 
	{0x338d , 0x40}, 
	{0x338e , 0x8}, 
	{0x3400 , 0x0}, 
	{0x3601 , 0x30}, 
	{0x3606 , 0x20}, 
	{0x360b , 0x0}, 
	{0x363b , 0x1}, 
	{0x363c , 0xf2}, 
	{0x3086 , 0x0f},
	{0x3086 , 0x00},
	
};


static struct register_address_value_pair_t init_preview_preflash_array[]={

	{0x308c  , 0x08},  // group hold enable
	{0x3353  , 0x2f},  
	{0x3359  , 0x2b}, 
	{0x335f  , 0x29},  
	{0x3030  , 0x00},  
	{0x3031  , 0x3c},  
	{0x3032  , 0x3c},  
	{0x3033  , 0x00},  
	{0x308c  , 0x00},  // group hold disable
	{0x30ff  , 0xff},  // group hold release


};

static struct register_address_value_pair_t init_previewtosnapshot_array[] = { 

	{0x3010 , 0x81},
	{0x3012 , 0x0}, 
	{0x3014 , 0x65},
	{0x3015 , 0x22},
	{0x3023 , 0xc}, 
	{0x3026 , 0x4}, 
	{0x3027 , 0xbc},
	{0x302a , 0x4}, 
	{0x302b , 0xd4},
	{0x302d , 0x00},
	{0x302e , 0x00},
	{0x3048 , 0x1f},
	{0x3049 , 0x4e},
	{0x304a , 0x40},
	{0x304b , 0x2}, 
	{0x304d , 0x42},
	{0x304f , 0x40},
	{0x3050 , 0x40},
	{0x3069 , 0x86},
	{0x306f , 0x54},
	{0x3088 , 0x6}, 
	{0x3089 , 0x40},
	{0x308a , 0x4}, 
	{0x308b , 0xb0},
	{0x308e , 0x74},
	{0x3090 , 0x3}, 
	{0x30a1 , 0x41},
	{0x30a3 , 0x80},
	{0x30aa , 0x52},
	{0x30af , 0x0}, 
	{0x30b2 , 0x7}, 
	{0x30c0 , 0xc}, 
	{0x30c1 , 0xb6},
	{0x30d9 , 0x95},
	{0x30de , 0x95},
	{0x3302 , 0x1}, 
	{0x3317 , 0x4b},
	{0x3318 , 0x0}, 
	{0x3319 , 0x4c},
	{0x331d , 0x6c},
	{0x3362 , 0x80},
	{0x3373 , 0x50},
	{0x3378 , 0x20},
	{0x3379 , 0x36},
	{0x338c , 0x30},
	{0x338d , 0x3e},
	{0x338e , 0xc}, 
 	
	
};

static struct register_address_value_pair_t init_previewtosnapshot_FlashON_array[] = { 

	{0x3010 , 0x81},
	{0x3012 , 0x0}, 
	{0x3015 , 0x2}, 
	{0x3023 , 0xc}, 
	{0x3026 , 0x4}, 
	{0x3027 , 0xbc},
	{0x302a , 0x4}, 
	{0x302b , 0xd4},
	{0x3048 , 0x1f},
	{0x3049 , 0x4e},
	{0x304a , 0x40},
	{0x304b , 0x2}, 
	{0x304d , 0x42},
	{0x304f , 0x40},
	{0x3050 , 0x40},
	{0x3069 , 0x86},
	{0x306f , 0x54},
	{0x3088 , 0x6}, 
	{0x3089 , 0x40},
	{0x308a , 0x4}, 
	{0x308b , 0xb0},
	{0x308e , 0xd4},
	{0x3090 , 0x3},
	{0x30a1 , 0x41},
	{0x30a3 , 0x80},
	{0x30aa , 0x52},
	{0x30af , 0x0},
	{0x30b2 , 0x7}, 
	{0x30c0 , 0xc}, 
	{0x30c1 , 0x81},
	{0x30d9 , 0x95},
	{0x30de , 0xa2},
	{0x30df , 0x6b},
	{0x3302 , 0x1}, 
	{0x3317 , 0x4b},
	{0x3318 , 0x0},
	{0x3319 , 0x4c},
	{0x331d , 0x6c},
	{0x3353 , 0x19},
	{0x3359 , 0x15},
	{0x335f , 0x13},
	{0x3362 , 0x80},
	{0x3373 , 0x50},
	{0x3376 , 0x4}, 
	{0x3378 , 0x20},
	{0x3379 , 0x36},
	{0x338c , 0x30},
	{0x338d , 0x3e},
	{0x338e , 0xc}, 


};

static struct register_address_value_pair_t init_snapshottopreview_array[] = {
		
	{0x3000 , 0x0f},
	{0x3002 , 0x00},
	{0x3003 , 0x9c},
	{0x3010 , 0x82},
	{0x3012 , 0x10},
	{0x3014 , 0x6d},
	{0x3015 , 0x23},
	{0x3023 , 0x6}, 
	{0x3026 , 0x2}, 
	{0x3027 , 0x5e},
	{0x302a , 0x2}, 
	{0x302b , 0x6a},
	{0x3030 , 0x28},
	{0x3031 , 0xbe},
	{0x3032 , 0x7d},
	{0x3033 , 0x14},
	{0x3069 , 0x84},
	{0x306f , 0x14},
	{0x3088 , 0x3}, 
	{0x3089 , 0x20},
	{0x308a , 0x2}, 
	{0x308b , 0x58},
	{0x308e , 0x0}, 
	{0x3090 , 0x33},
	{0x30a1 , 0x1}, 
	{0x30a2 , 0x45}, 
	{0x30a3 , 0x11}, 
	{0x30aa , 0x52},
	{0x30af , 0x00},
	{0x30b2 , 0x2c},
	{0x30c0 , 0x07},
	{0x30c1 , 0x73},
	{0x30d9 , 0x8c},
	{0x3302 , 0x11},
	{0x3317 , 0x25},
	{0x3318 , 0x80},
	{0x3319 , 0x8}, 
	{0x331d , 0x38},
	{0x3353 , 0x19},
	{0x3359 , 0x15},
	{0x335f , 0x13},
	{0x3362 , 0x90},
	{0x3363 , 0xfc},
	{0x3373 , 0x30},
	{0x3376 , 0x6}, 
	{0x3378 , 0x30},
	{0x3379 , 0x70},
	{0x338c , 0x38},
	{0x338d , 0x40},
	{0x338e , 0x8}, 

	

};







static int ov265x_reset(struct msm_camera_device_platform_data *dev)
{
	int rc = 0;

	rc = gpio_request(dev->sensor_reset, "ov265x");

	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
		mdelay(50);
		rc = gpio_direction_output(dev->sensor_reset, 1);
	}

	gpio_free(dev->sensor_reset);
	return rc;
}

static int32_t ov265x_i2c_txdata(unsigned short saddr, unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(ov265x_ctrl->client->adapter, msg, 1) < 0) {
		CDBG("ov265x_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

static int32_t ov265x_i2c_write(unsigned short saddr, unsigned short waddr, unsigned char wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));

	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = wdata;

	rc = ov265x_i2c_txdata(saddr, buf, 3);

	if (rc < 0)
		CDBG(
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int ov265x_i2c_rxdata(unsigned short saddr, unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(ov265x_ctrl->client->adapter, msgs, 2) < 0) {
		CDBG("ov265x_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t ov265x_i2c_read(unsigned short saddr, unsigned short raddr, unsigned char *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov265x_i2c_rxdata(saddr, buf, 1);
	if (rc < 0)
		return rc;

	*rdata = buf[0];


	if (rc < 0)
		CDBG("ov265x_i2c_read failed!\n");

	return rc;
}

static long ov265x_reg_init(void)
{
	int32_t array_length;
	int32_t i;
	long rc;

	array_length =
		sizeof(init_reg_settings_array) /
		sizeof(init_reg_settings_array[0]);

	/* Soft reset */
	rc = ov265x_i2c_write(OV265X_I2C_ADDR, 0x3012, 0x80);	

	mdelay(10);

	/* Configure sensor initialy for Preview mode */
	for (i = 0; i < array_length; i++) {
		rc = ov265x_i2c_write(OV265X_I2C_ADDR,
			init_reg_settings_array[i].register_address,
			init_reg_settings_array[i].register_value);
	
		if (rc < 0)
			return rc;
	}

	return 0;
}

static long ov265x_set_sensor_mode(enum sensor_mode_t mode, struct camd_reg_cfg *camd_reg)
{
	int32_t i;
	int32_t array_length;
	long rc = 0;

	uint8_t auto_gain_exposure =0;
	uint16_t shutter=0, preview_exposure=0;
	uint8_t preview_gain=0;
	uint16_t preview_gain16=0;
	uint8_t shutterl=0;
	uint8_t shutterh=0;
	uint8_t luminance_frequency = 0;
	uint16_t default_svga_line_width = DEFAULT_SVGA_LINE_WIDTH;
	uint16_t default_uxga_line_width = DEFAULT_UXGA_LINE_WIDTH;
	uint16_t default_uxga_maximum_shutter = DEFAULT_UXGA_MAXIMUM_SHUTTER;
	uint16_t capture_max_gain= CAPTURE_MAX_GAIN;
	uint16_t preview_line_width, preview_dummy_pixel=0, capture_dummy_pixel=0;
	uint16_t capture_line_width, capture_maximum_shutter;		 
	uint32_t capture_banding_filter, gain_exposure, capture_gain, gain;
	uint32_t capture_exposure = 0;
 	uint16_t luminance_avg = 0;
	uint16_t snapshotEXVTS=0;
	uint8_t exVTSH=0, exVTSL=0;	
	struct register_cfg ov_reg;
	struct register_cfg* ov_reg_ptr;	
	
	switch (mode) {
	case SENSOR_PREVIEW_MODE:		
		printov(" PREVIEW MODE \n");		
		printov("Shutterh 0x%x Shutterl 0x%x \n", preview_shutterh, preview_shutterl);	
		printov("Gain 0x%x \n", preview_Gain);			
          	array_length =
			sizeof(init_snapshottopreview_array) /
			sizeof(init_snapshottopreview_array[0]);

		for (i = 0; i < array_length; i++) {
			rc = ov265x_i2c_write(OV265X_I2C_ADDR,
				init_snapshottopreview_array[i].register_address,
				init_snapshottopreview_array[i].register_value);
		
			if (rc < 0)
				return rc;
		}
	
		
				
		if(camd_reg && camd_reg->length){
		
			for( i=0, ov_reg_ptr = camd_reg->ov_reg; i < camd_reg->length ; i++, ov_reg_ptr++){
			
		 		if (copy_from_user(&ov_reg, ov_reg_ptr, sizeof(ov_reg)))
			     		return -EFAULT;

				printov("Register Address 0x%x Value 0x%x \n",
							ov_reg.register_address,
							ov_reg.register_value);

				rc = ov265x_i2c_write(OV265X_I2C_ADDR,
							ov_reg.register_address,
							ov_reg.register_value);
				if(rc < 0)
				   return rc;

				
			}

		}

		
		//Enable AE/AG
    		rc = ov265x_i2c_read(OV265X_I2C_ADDR, AUTOGAINEXPOSURE, &auto_gain_exposure);
	        if (rc < 0)
		    return rc;
		printov("AutoGainExposure 0x%x \n ", auto_gain_exposure);
		auto_gain_exposure = auto_gain_exposure | 0x05;

 		printov(" Updated AutoGainExposure 0x%x \n", auto_gain_exposure);
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, AUTOGAINEXPOSURE, auto_gain_exposure);
		if(rc < 0)
		   return rc;	

		mdelay(50);

		
		
		break;

	
	case SENSOR_SNAPSHOT_MODE:      

	   if(!enable_flash){
		printov("FLASHOFF SNAPSHOT MODE \n");
		
		// The sequence and configuration is done as per the OV data sheet
				
		//stop AE/AG
	 	rc = ov265x_i2c_read(OV265X_I2C_ADDR, AUTOGAINEXPOSURE, &auto_gain_exposure);
	        if (rc < 0)
		    return rc;
		printov("auto gain exposure 0x%x \n", auto_gain_exposure);
		auto_gain_exposure = auto_gain_exposure & 0xfa;

 		printov("updated auto gain exposure 0x%x \n", auto_gain_exposure);
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, AUTOGAINEXPOSURE, auto_gain_exposure);
		if(rc < 0)
		   return rc;
		
		
		//Read back preview shutter
		rc = ov265x_i2c_read(OV265X_I2C_ADDR, SHUTTERH, &shutterh);		
		preview_shutterh = shutterh; 
		rc = ov265x_i2c_read(OV265X_I2C_ADDR, SHUTTERL, &shutterl);
		if (rc < 0)
		    return rc;
		preview_shutterl = shutterl; 	   	
 		shutter = (shutterh << 8) + shutterl;
		preview_exposure = shutter ;
		
		printov("Preview Shutter  0x%x \n", shutter);		
		printov("Preview Exposure 0x%x \n", preview_exposure );

		//Read Back Gain for preview_color_effect
		rc = ov265x_i2c_read(OV265X_I2C_ADDR, GAIN, &preview_gain);
		preview_Gain = preview_gain;
	        if (rc < 0)
		    return rc;
				
		preview_gain16 = (( (preview_gain & 0xf0) >> 4 ) + 1) *
				(16 + (preview_gain & 0x0f));

		printov("Preview_Gain 0x%x \n", preview_gain);
		printov("Preview Gain16 0x%x \n", preview_gain16 );


		//Calculate capture exposure
		preview_line_width = default_svga_line_width + preview_dummy_pixel;
		capture_line_width = default_uxga_line_width + capture_dummy_pixel;
		capture_maximum_shutter = default_uxga_maximum_shutter;

		capture_exposure = preview_exposure;

		printov("Initial capture exposure %d \n", capture_exposure );	

		rc = ov265x_i2c_read(OV265X_I2C_ADDR, LIGHTFREQUENCY, &luminance_frequency);
		
	        if (rc < 0)
		    return rc;
		
	
		if(0 == (luminance_frequency & 0x01)){

			capture_banding_filter = 100;
		}else{					
			capture_banding_filter = 120;

		}
		
 		printov( "capture_banding_filter %d \n", capture_banding_filter );

		capture_max_gain = CAPTURE_MAX_GAIN;
			
		//redistribute gain and exposure
		gain_exposure = preview_gain16 * capture_exposure;
		
		printov("Gain Exposure %d \n", gain_exposure);
		
		if(gain_exposure < capture_banding_filter*16 ){
		
 			printov("Gain Exposure < capture_banding_filter*16\n");
			capture_exposure = gain_exposure/16;
			printov("Capture Exposure %d \n", capture_exposure );
			capture_gain = (gain_exposure*2 + 1)/capture_exposure/2;
			printov("Capture_gain %d \n", capture_gain );


		}
		else{
			if(gain_exposure > capture_maximum_shutter*16){
				
				printov("Gain Exposure > capture_max_shutter*16\n"); ;
				capture_exposure = capture_maximum_shutter;
				printov("Capture Exposure %d \n", 					capture_exposure );
				capture_gain = (gain_exposure*2 + 1)/ 		
						capture_maximum_shutter/2;
				printov("Capture_gain %d \n", capture_gain );
				
				if( capture_gain > capture_max_gain){
					printov( "capture_gain > capture_max_gain\n");
					capture_exposure =  
 						gain_exposure/16/capture_banding_filter;
					capture_exposure = 
						capture_exposure*capture_banding_filter;
					printov("Capture Exposure %d \n", capture_exposure );
					capture_gain = (gain_exposure*2 + 1)/capture_exposure/2;
					printov("Capture_gain %d \n", capture_gain );

				}else{	

					printov("Capture_gain < capture_max_gain\n");

 					capture_exposure = 
						capture_exposure/capture_banding_filter;
					capture_exposure = capture_exposure * 	
						capture_banding_filter;
					printov("Capture Exposure %d \n", capture_exposure );	
					capture_gain = (gain_exposure*2 + 1)/		
							capture_maximum_shutter/2;
					printov("Capture_gain %d \n", capture_gain );


				 }				

			}
			else{
				printov("gain_exposure > capture_banding_filter*16 \n");

 				capture_exposure = gain_exposure/16/capture_banding_filter;
				capture_exposure = capture_exposure * capture_banding_filter;

				printov("capture Exposure %d \n", capture_exposure );	
				capture_gain = (gain_exposure*2 + 1)/capture_exposure/2;
				printov("Capture_gain %d \n", capture_gain );

			}
			


		}

		
		rc = ov265x_i2c_read(OV265X_I2C_ADDR, 0x302D, &exVTSH);
		printov(" exVTSH value 0x%x \n", exVTSH);
        	if (rc < 0)
		    return rc;

		rc = ov265x_i2c_read(OV265X_I2C_ADDR, 0x302E, &exVTSL);
		printov("exVTSL value 0x%x \n", exVTSL);
        	if (rc < 0)
		    return rc;
	
		printov("write snapshot array \n");
		array_length =
			sizeof(init_previewtosnapshot_array) /
			sizeof(init_previewtosnapshot_array[0]);

		for (i = 0; i < array_length; i++) {
			rc = ov265x_i2c_write(OV265X_I2C_ADDR,
				init_previewtosnapshot_array[i].register_address,
				init_previewtosnapshot_array[i].register_value);
		
			if (rc < 0)
				return rc;

		}
		
		if(camd_reg && camd_reg->length){			
			
			for( i=0, ov_reg_ptr = camd_reg->ov_reg; i < camd_reg->length ; i++, ov_reg_ptr++){
			
		 		if (copy_from_user(&ov_reg, ov_reg_ptr, sizeof(ov_reg)))
			     		return -EFAULT;

				printov("Register Address 0x%x Value 0x%x \n",
							ov_reg.register_address,
							ov_reg.register_value);

				rc = ov265x_i2c_write(OV265X_I2C_ADDR,
							ov_reg.register_address,
							ov_reg.register_value);
				if(rc < 0)
				   return rc;

				
			}
			

		}
		//night mode calculations
		
		snapshotEXVTS = ( (exVTSH << 8) + exVTSL ) * 2	;
		exVTSH = snapshotEXVTS >> 8 ;
		exVTSL = snapshotEXVTS - (exVTSH *256) ;

		printov("SnapshotExtvs 0x%x exVTSH 0x%x exVTSL 0x%x \n",snapshotEXVTS,										exVTSH, exVTSL);
		
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, 0x302D, exVTSH);
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, 0x302E, exVTSL);				
		rc = ov265x_i2c_read(OV265X_I2C_ADDR, 0x302D, &exVTSH);
		printov("exVTSH value 0x%x ", exVTSH);
       		 if (rc < 0)
		    return rc;

		rc = ov265x_i2c_read(OV265X_I2C_ADDR, 0x302E, &exVTSL);
		printov("exVTSL value 0x%x ", exVTSL);
        	if (rc < 0)
		    return rc;		
				
		if(capture_exposure > capture_maximum_shutter){
			shutter = capture_maximum_shutter;
		
		}else {			
			shutter = capture_exposure;	
			
		}
		
		printov("Capture Shutter %d \n", shutter );
		shutterl = shutter & 0x00ff;		
		shutterh = (shutter >> 8) & 0x00ff;
		printov("shutterh 0x%x 0x%x\n", shutterh, shutterl );
		
		//write shutter time
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, SHUTTERL, shutterl);
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, SHUTTERH, shutterh);
		if(rc < 0)
		  return rc;
		
				
		//Calculate Gain		
		gain = 0;
		if(capture_gain > 31){
			capture_gain = capture_gain/2;
			gain = 0x10;
		
		}
		
		if(capture_gain > 31){
			capture_gain = capture_gain/2;
			gain = gain | 0x20;
		
		}

		if(capture_gain > 31){			
			capture_gain = capture_gain/2;
			gain = gain | 0x40;
		
		}

		if(capture_gain > 31){			
			capture_gain = capture_gain/2;
			gain = gain | 0x80;
		
		}

		printov("Gain 0x%x %d  capture_gain %d\n", gain, gain, capture_gain );
		if(capture_gain > 16){
			gain = gain | ((capture_gain-16) & 0x000f);
		}
				
		printov("Final Capture Gain 0x%x  \n", gain);
		
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, GAIN, gain);
		if(rc < 0)
		   return rc;	
		
			
		
	 }else{

		printov("FLASHON SNAPSHOT MODE \n");
		
		// The sequence and configuration is done as per the OV data sheet			
		//stop AE/AG
	 	rc = ov265x_i2c_read(OV265X_I2C_ADDR, AUTOGAINEXPOSURE, &auto_gain_exposure);
	        if (rc < 0)
		    return rc;
		printov("auto gain exposure 0x%x \n", auto_gain_exposure);
		auto_gain_exposure = auto_gain_exposure & 0xfa;

 		printov("updated auto gain exposure 0x%x \n", auto_gain_exposure);
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, AUTOGAINEXPOSURE, auto_gain_exposure);
		if(rc < 0)
		   return rc;
		
		preview_shutterh = gshutterh; 
		preview_shutterl = gshutterl; 	   	
 		shutter = (gshutterh << 8) + gshutterl;
		preview_exposure = shutter ;
		
		printov("Preview Shutter  0x%x \n", shutter);		
		printov("Preview Exposure 0x%x \n", preview_exposure );

		//Read Back Gain for preview_color_effect
		//rc = ov265x_i2c_read(OV265X_I2C_ADDR, GAIN, &preview_gain);
		preview_gain = gpreview_gain;
		preview_Gain = preview_gain;
	
		preview_gain16 = (( (preview_gain & 0xf0) >> 4 ) + 1) *
				(16 + (preview_gain & 0x0f));

		printov("Preview_Gain 0x%x \n", preview_gain);
		printov("Preview Gain16 0x%x \n", preview_gain16 );


		//Calculate capture exposure
		preview_line_width = default_svga_line_width + preview_dummy_pixel;
		capture_line_width = default_uxga_line_width + capture_dummy_pixel;
		capture_maximum_shutter = default_uxga_maximum_shutter;

		capture_exposure = preview_exposure;

		printov("Initial capture exposure %d \n", capture_exposure );	

		rc = ov265x_i2c_read(OV265X_I2C_ADDR, LIGHTFREQUENCY, &luminance_frequency);
		
	        if (rc < 0)
		    return rc;
		
	
		if(0 == (luminance_frequency & 0x01)){

			capture_banding_filter = 100;
		}else{					
			capture_banding_filter = 120;

		}
		
 		printov( "capture_banding_filter %d \n", capture_banding_filter );

		capture_max_gain = CAPTURE_MAX_GAIN;
			
		//redistribute gain and exposure
		gain_exposure = preview_gain16 * capture_exposure;
		
		printov("Gain Exposure %d \n", gain_exposure);
		
		if(gain_exposure < capture_banding_filter*16 ){
		
 			printov("Gain Exposure < capture_banding_filter*16\n");
			capture_exposure = gain_exposure/16;
			printov("Capture Exposure %d \n", capture_exposure );
			capture_gain = (gain_exposure*2 + 1)/capture_exposure/2;
			printov("Capture_gain %d \n", capture_gain );


		}
		else{
			if(gain_exposure > capture_maximum_shutter*16){
				
				printov("Gain Exposure > capture_max_shutter*16\n"); ;
				capture_exposure = capture_maximum_shutter;
				printov("Capture Exposure %d \n", 					capture_exposure );
				capture_gain = (gain_exposure*2 + 1)/ 		
						capture_maximum_shutter/2;
				printov("Capture_gain %d \n", capture_gain );
				
				if( capture_gain > capture_max_gain){
					printov( "capture_gain > capture_max_gain\n");
					capture_exposure =  
 						gain_exposure/16/capture_banding_filter;
					capture_exposure = 
						capture_exposure*capture_banding_filter;
					printov("Capture Exposure %d \n", capture_exposure );
					capture_gain = (gain_exposure*2 + 1)/capture_exposure/2;
					printov("Capture_gain %d \n", capture_gain );

				}else{	

					printov("Capture_gain < capture_max_gain\n");

 					capture_exposure = 
						capture_exposure/capture_banding_filter;
					capture_exposure = capture_exposure * 	
						capture_banding_filter;
					printov("Capture Exposure %d \n", capture_exposure );	
					capture_gain = (gain_exposure*2 + 1)/		
							capture_maximum_shutter/2;
					printov("Capture_gain %d \n", capture_gain );


				 }

				

			}
			else{
				printov("gain_exposure > capture_banding_filter*16 \n");

 				capture_exposure = gain_exposure/16/capture_banding_filter;
				capture_exposure = capture_exposure * capture_banding_filter;

				printov("capture Exposure %d \n", capture_exposure );	
				capture_gain = (gain_exposure*2 + 1)/capture_exposure/2;
				printov("Capture_gain %d \n", capture_gain );

			}
			


		}


		printov("write init_previewtosnapshot_FlashON_array snapshot array \n");
		array_length =
			sizeof(init_previewtosnapshot_FlashON_array) /
			sizeof(init_previewtosnapshot_FlashON_array[0]);

		for (i = 0; i < array_length; i++) {
			rc = ov265x_i2c_write(OV265X_I2C_ADDR,
				init_previewtosnapshot_FlashON_array[i].register_address,
				init_previewtosnapshot_FlashON_array[i].register_value);
		
			if (rc < 0)
				return rc;
		}

		
		if(camd_reg && camd_reg->length ){
		
			for( i=0, ov_reg_ptr = camd_reg->ov_reg; i < camd_reg->length ; i++, ov_reg_ptr++){
			
		 		if (copy_from_user(&ov_reg, ov_reg_ptr, sizeof(ov_reg)))
			     		return -EFAULT;

				printov("Register Address 0x%x Value 0x%x \n",
							ov_reg.register_address,
							ov_reg.register_value);

				rc = ov265x_i2c_write(OV265X_I2C_ADDR,
							ov_reg.register_address,
							ov_reg.register_value);
				if(rc < 0)
				   return rc;

				
			}

		}

		if(capture_exposure > capture_maximum_shutter){
			shutter = capture_maximum_shutter;
		
		}else {			
			shutter = capture_exposure;	
			
		}
		
		
		if(luminance_avg > 0x58 && luminance_avg < 0x68){

			shutter = shutter/2;
			printov("shutter divided by 3 and shutter value 0x%x \n", shutter);

		}else if(luminance_avg > 0x68 && luminance_avg < 0x88){
			
			shutter = shutter/3;
			printov("shutter divided by 3 and shutter value 0x%x \n", shutter);

		}else if(luminance_avg > 0x88){

			shutter = shutter/4;
			printov("shutter divided by 4 and shutter value 0x%x \n", shutter);
		}
		
		

		printov("Capture Shutter %d \n", shutter );
		shutterl = shutter & 0x00ff;		
		shutterh = (shutter >> 8) & 0x00ff;
		printov("shutterh 0x%x 0x%x\n", shutterh, shutterl );
		
		//write shutter time
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, SHUTTERL, shutterl);
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, SHUTTERH, shutterh);
		if(rc < 0)
		  return rc;
		
				
		//Calculate Gain		
		gain = 0;
		if(capture_gain > 31){
			capture_gain = capture_gain/2;
			gain = 0x10;
		
		}
		
		if(capture_gain > 31){
			capture_gain = capture_gain/2;
			gain = gain | 0x20;
		
		}

		if(capture_gain > 31){			
			capture_gain = capture_gain/2;
			gain = gain | 0x40;
		
		}

		if(capture_gain > 31){			
			capture_gain = capture_gain/2;
			gain = gain | 0x80;
		
		}

		printov("Gain 0x%x %d  capture_gain %d\n", gain, gain, capture_gain );
		if(capture_gain > 16){
			gain = gain | ((capture_gain-16) & 0x000f);
		}
				
		printov("Final Capture Gain 0x%x  \n", gain);
		
	
		rc = ov265x_i2c_write(OV265X_I2C_ADDR, GAIN, gain);
		if(rc < 0)
		   return rc;	
					
	

	}
	
	enable_flash = 0;
	break;
	

	default:
		return -EFAULT;
	}

	return 0;
}


static long ov265x_set_effect(
	enum sensor_mode_t mode,
	int8_t effect
)
{

	long rc = 0;

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		break;

	case SENSOR_SNAPSHOT_MODE:
		break;

	default :
		break;
	}

	switch ((enum camera_effect_t)effect) {
	case CAMERA_EFFECT_OFF:
			break;

	case CAMERA_EFFECT_MONO:
		break;

	case CAMERA_EFFECT_NEGATIVE: 
		break;

	case CAMERA_EFFECT_SOLARIZE:
		break;

	case CAMERA_EFFECT_SEPIA:
		break;

	case CAMERA_EFFECT_PASTEL:
	case CAMERA_EFFECT_MOSAIC:
	case CAMERA_EFFECT_RESIZE:
	default:
		break;
	}

	return rc;
}

int32_t ov265x_sensor_init(void)
{
	uint8_t pidh, pidl;
	uint32_t ov265x_camclk_po_hz;
	int rc = 0;

	CDBG("init entry \n");

	//Power the sensor on
	CDBG("ov265x_sensor_ini: Powering on camera sensor.\n");
	rc = gpio_direction_output(ov265x_ctrl->sensordata->sensor_pwd, 0);	

	/* Input MCLK = 24MHz */
	ov265x_camclk_po_hz = 24000000;
	msm_camio_clk_rate_set(ov265x_camclk_po_hz);
	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	rc = ov265x_reset(ov265x_ctrl->sensordata);
	if (rc < 0) {
		CDBG("reset failed!\n");
		return rc;
	}

	mdelay(5);

	/* Read the Model ID of the sensor */
	rc = ov265x_i2c_read(OV265X_I2C_ADDR, OV265X_ADDR_PIDH, &pidh);
	if (rc < 0)
		return rc;
	rc = ov265x_i2c_read(OV265X_I2C_ADDR, OV265X_ADDR_PIDL, &pidl);
	if (rc < 0)
		return rc;

	CDBG("ov265x model_id = 0x%x\n", (pidh << 8) | pidl);

	/* Check if it matches it with the value in Datasheet */
	if (pidh != 0x26)
		return -EFAULT;

	/* The Sensor is indeed OmniVision */
	/* Initialize Sensor registers */
	rc = ov265x_reg_init();
	if (rc < 0)
		return rc;

	CDBG("init exit\n");

	return 0;
}

static int ov265x_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&ov265x_wait_queue);
	return 0;
}

static int ov265x_open(struct inode *inode, struct file *fp)
{
	int32_t rc = 0;

	/* down(&ov265x_sem); */

	CDBG("ov265x: open = %d\n",
		ov265x_ctrl->opened);

	if (ov265x_ctrl->opened) {
		rc = 0;
		goto open_done;
	}

	rc = ov265x_sensor_init();

	if (rc >= 0)
		ov265x_ctrl->opened = 1;
	else
		CDBG("sensor init failed!\n");

open_done:
	/* up(&ov265x_sem); */
	return rc;
}


static int ov265x_flashconfiguration(struct sensor_cfg_data_t* cfg_data, unsigned int cmd){
	int i = 0;
	int array_length=0;
	long   rc = 0;

	if(cfg_data->cfg.flash.flash_led_current_ma != 0){

		enable_flash = 1;
		
	}else{
		enable_flash = 0;
	}

	switch (cmd) {

		case MSM_CAMERA_CONFIG_CURRENT:
			
			if(cfg_data->cfg.flash.flash_led_current_ma > PREFLASH_MINCURRENT &&
				cfg_data->cfg.flash.flash_led_current_ma < PREFLASH_MAXCURRENT  ){

				//write preflash array

				array_length =
					 sizeof(init_preview_preflash_array) /
					 sizeof(init_preview_preflash_array[0]);

				printov("Preflash array size %d\n", array_length);
				printov("Writing Preflash preview array \n");

				for (i = 0; i < array_length; i++) {
						rc = ov265x_i2c_write(OV265X_I2C_ADDR,
						init_preview_preflash_array[i].register_address,
						init_preview_preflash_array[i].register_value);

				}
		
				if (rc < 0)
					return rc;
			}
			
			//Need to read preview gain and Exposure in the preflash and not in mainflash
			//Hence reading just before main flash is turned on
			printov("Flash current %d \n ",cfg_data->cfg.flash.flash_led_current_ma);	

			if(cfg_data->cfg.flash.flash_led_current_ma > MAINFLASH_MINCURRENT &&
				cfg_data->cfg.flash.flash_led_current_ma < MAINFLASH_MAXCURRENT){

				rc = ov265x_i2c_read(OV265X_I2C_ADDR, SHUTTERH, &gshutterh);		
				printov("gshutterh 0x%x \n", gshutterh);
				
				rc = ov265x_i2c_read(OV265X_I2C_ADDR, SHUTTERL, &gshutterl);
				if (rc < 0)
				    return rc;

				printov("gshutterl 0x%x \n", gshutterl);
					   	
 				gshutter = (gshutterh << 8) + gshutterl;
				
		
				printov("Preview Shutter  0x%x \n", gshutter);		
				
				//Read Back Gain for preview_color_effect
				rc = ov265x_i2c_read(OV265X_I2C_ADDR, GAIN, &gpreview_gain);		
	       			if (rc < 0)
				    return rc;
				printov("gpreview gain 0x%x \n", gpreview_gain);			


			}
			break ;

		case MSM_CAMERA_CONFIG_CURRENT_EX:
			
			if(cfg_data->cfg.flash.cam_flash_mode == PREFLASH){
				
				array_length =
					 sizeof(init_preview_preflash_array) /
					 sizeof(init_preview_preflash_array[0]);

				printov("Preflash array size %d\n", array_length);
				printov("Writing Preflash preview array \n");

				for (i = 0; i < array_length; i++) {
						rc = ov265x_i2c_write(OV265X_I2C_ADDR,
						init_preview_preflash_array[i].register_address,
						init_preview_preflash_array[i].register_value);

				}
		
				if (rc < 0)
					return rc;

			}
			//Need to read preview gain and Exposure in the preflash and not in mainflash
			//Hence reading just before main flash is turned on
			printov("Flash current %d \n ",cfg_data->cfg.flash.flash_led_current_ma);	

			if(cfg_data->cfg.flash.cam_flash_mode == MAINFLASH){
				
				rc = ov265x_i2c_read(OV265X_I2C_ADDR, SHUTTERH, &gshutterh);		
				printov("gshutterh 0x%x \n", gshutterh);
				
				rc = ov265x_i2c_read(OV265X_I2C_ADDR, SHUTTERL, &gshutterl);
				if (rc < 0)
				    return rc;

				printov("gshutterl 0x%x \n", gshutterl);
					   	
 				gshutter = (gshutterh << 8) + gshutterl;
				
		
				printov("Preview Shutter  0x%x \n", gshutter);		
				
				//Read Back Gain for preview_color_effect
				rc = ov265x_i2c_read(OV265X_I2C_ADDR, GAIN, &gpreview_gain);		
	       			if (rc < 0)
				    return rc;
				printov("gpreview gain 0x%x \n", gpreview_gain);			


			}
			break;

		default: 
			rc = -EFAULT;
			break;

	}		

	return rc;
}

static long ov265x_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct sensor_cfg_data_t cfg_data;
	long   rc = 0;
	

	if (copy_from_user(&cfg_data,
				(void *)argp,
				sizeof(struct sensor_cfg_data_t)))
		return -EFAULT;

	/* down(&ov265x_sem); */

	CDBG("ov265x_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	switch (cmd) {

		case MSM_CAMSENSOR_IO_CFG: {

			switch (cfg_data.cfgtype) {
			case CFG_SET_MODE:
				rc = ov265x_set_sensor_mode(
							cfg_data.mode, NULL);
				break;

			case CFG_SET_MODE_EX:
				rc = ov265x_set_sensor_mode(
							cfg_data.mode, &cfg_data.cfg.camd_reg);
				break;

			case CFG_SET_EFFECT:
				rc = ov265x_set_effect(
							cfg_data.mode,
							cfg_data.cfg.effect);
				break;

			default:
				rc = -EFAULT;
				break;
			}
		}
		break;
		
		case MSM_CAMERA_CONFIG_CURRENT:
			
			rc = ov265x_flashconfiguration(&cfg_data,cmd); 
			if(rc < 0)
			     return rc;			
			ov265x_ctrl->sensordata->camera_flash_current_ma 					(cfg_data.cfg.flash.flash_led_mode_type, \
					cfg_data.cfg.flash.flash_led_polarity_type, cfg_data.cfg.flash.flash_led_current_ma);
			break;
		
		case MSM_CAMERA_CONFIG_CURRENT_EX:

			if(cfg_data.cfg.flash.cam_flash_mode != DEFAULT){
				rc = ov265x_flashconfiguration(&cfg_data,cmd); 
				if(rc < 0)
			     		return rc;	
			}
			ov265x_ctrl->sensordata->camera_flash_current_ma 					(cfg_data.cfg.flash.flash_led_mode_type, \
					cfg_data.cfg.flash.flash_led_polarity_type, cfg_data.cfg.flash.flash_led_current_ma);
	
			break;
		case MSM_CAMERA_READ_SENSOR_REGISTER:

			
			rc = ov265x_i2c_read(OV265X_I2C_ADDR, cfg_data.cfg.reg.register_address, 
 				(uint8_t *)(&cfg_data.cfg.reg.register_value));

			if (copy_to_user( 
				(void *)argp, &cfg_data,
				sizeof(struct sensor_cfg_data_t)))
			return -EFAULT;
			
			break;

		
		default:
			rc = -EFAULT;
			break;
	}

	/* up(&ov265x_sem); */

	return rc;
}

static int ov265x_standby(void)
{
	// Power off sequence
	ov265x_i2c_write(OV265X_I2C_ADDR, 0x30AB, 0x00);
	ov265x_i2c_write(OV265X_I2C_ADDR, 0x30AD, 0x0A);
	ov265x_i2c_write(OV265X_I2C_ADDR, 0x30AE, 0x27);
	ov265x_i2c_write(OV265X_I2C_ADDR, 0x363B, 0x01);

	mdelay(5);	

	// Put it to standby
	CDBG("ov265x_release: Putting camera sensor to standby.\n");
	gpio_direction_output(ov265x_ctrl->sensordata->sensor_pwd, 1);

	return 0;
}

static int ov265x_release(struct inode *ip, struct file *fp)
{
	int rc = -EBADF;

	/* down(&ov265x_sem); */
	if (ov265x_ctrl->opened)
	{
		ov265x_ctrl->opened = 0;
		rc = ov265x_standby();	
	}

	/* up(&ov265x_sem); */

	return rc;
}

static struct file_operations ov265x_fops = {
	.owner 	= THIS_MODULE,
	.open 	= ov265x_open,
	.release = ov265x_release,
	.unlocked_ioctl = ov265x_ioctl,
};

static struct miscdevice ov265x_device = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "ov265x",
	.fops 	= &ov265x_fops,
};

static int ov265x_probe(struct i2c_client *client)
{
	int rc = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	ov265x_ctrl->sensorw =
		kzalloc(sizeof(struct ov265x_work_t), GFP_KERNEL);

	if (!ov265x_ctrl->sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov265x_ctrl->sensorw);
	ov265x_init_client(client);
	ov265x_ctrl->client = client;

	// Camera sensor is active by default, in order to put
	// it into the correct deep standby mode we have to
	// completely enable it, configure the power down sequence
	// and then put it into HW standby by pulling PWDN pin high
	// again.

	msm_camio_enable();

	gpio_direction_output(ov265x_ctrl->sensordata->sensor_pwd, 0);	

	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	ov265x_reset(ov265x_ctrl->sensordata);

	ov265x_standby();	

	msm_camio_disable();

	/* Register a misc device */
	rc = misc_register(&ov265x_device);
	if (rc)
		goto probe_failure;

	return 0;

probe_failure:
	if (ov265x_ctrl->sensorw != NULL) {
		kfree(ov265x_ctrl->sensorw);
		ov265x_ctrl->sensorw = NULL;
	}
	return rc;
}

static int ov265x_remove(struct i2c_client *client)
{
	struct ov265x_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	i2c_detach_client(client);
	ov265x_ctrl->client = NULL;
	misc_deregister(&ov265x_device);
	kfree(sensorw);
	return 0;
}

#ifdef CONFIG_PM
static int ov265x_resume(struct i2c_client *client)
{
	// The sensor is woken up in a high power state again
	// Need to put it to sleep again.
	msm_camio_enable();

	gpio_direction_output(ov265x_ctrl->sensordata->sensor_pwd, 0);	

	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	msm_camio_camif_pad_reg_reset();

	ov265x_reset(ov265x_ctrl->sensordata);

	ov265x_standby();	

	msm_camio_disable();

	return 0;
}

static int ov265x_suspend(struct i2c_client *client, pm_message_t state)
{
	// Do nothing
	return 0;
}
#else 
#define ov265x_suspend NULL
#define ov265x_resume  NULL
#endif

static struct i2c_driver ov265x_driver = {
	.probe  = ov265x_probe,
	.remove = ov265x_remove,
	.suspend = ov265x_suspend, 
	.resume  = ov265x_resume, 
	.driver = {
		.name = "ov265x",
	},
};

int32_t ov265x_init(void *pdata)
{
	int32_t rc = 0;
	struct  msm_camera_device_platform_data *data =
		(struct  msm_camera_device_platform_data *)pdata;

	ov265x_ctrl = kzalloc(sizeof(struct ov265x_ctrl_t), GFP_KERNEL);
	if (!ov265x_ctrl) {
		rc = -ENOMEM;
		goto init_failure;
	}

	if (data) {
		ov265x_ctrl->sensordata = data;
		rc = i2c_add_driver(&ov265x_driver);

	}

init_failure:
	return rc;
}

