/*
 *  include/linux/i2c_lp8501_led.h
 *
 *  Copyright (C) 2008 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Authors: Kevin McCray (kevin.mccray@palm.com)
 *          Brian Xiong (brian.xiong@palm.com)
 *
 */

#ifndef _LP8501_LED_H
#define _LP8501_LED_H

#include <linux/i2c.h>
#include <linux/leds.h> 

#define LED_OFF		0
#define LED_ON		1


#define  LP8501_I2C_DEVICE   	"LP8501"
#define  LP8501_I2C_DRIVER   	"LP8501"
#define LP8501_I2C_DBG_DEVICE   "lp8501-dbg"
#define LP8501_I2C_DBG_DRIVER   "lp8501-dbg"
#define  LP8501_I2C_ADDR	0x32
#define INSTR_LEN		96
#define LP8501_INSTR_LEN_PER_PAGE	16

#define ENGINE_CNTRL1		0x00
#define ENGINE_CNTRL2		0x01
#define GROUP_FADING1		0x02
#define GROUP_FADING2		0x03
#define GROUP_FADING3		0x04
#define POWER_CONFIG		0x05
#define D1_CONTROL		0x06
#define D2_CONTROL		0x07
#define D3_CONTROL		0x08
#define D4_CONTROL		0x09
#define D5_CONTROL		0x0A
#define D6_CONTROL		0x0B
#define D7_CONTROL		0x0C
#define D8_CONTROL		0x0D
#define D9_CONTROL		0x0E

#define GPO_CONTROL		0x15
#define D1_PWM			0x16
#define D2_PWM			0x17
#define D3_PWM			0x18
#define D4_PWM			0x19
#define D5_PWM			0x1A
#define D6_PWM			0x1B
#define D7_PWM			0x1C
#define D8_PWM			0x1D
#define D9_PWM			0x1E

#define GPO_PWM			0x25
#define D1_CURRENT_CTRL		0x26
#define D2_CURRENT_CTRL		0x27
#define D3_CURRENT_CTRL		0x28
#define D4_CURRENT_CTRL 	0x29
#define D5_CURRENT_CTRL 	0x2A
#define D6_CURRENT_CTRL 	0x2B
#define D7_CURRENT_CTRL 	0x2C
#define D8_CURRENT_CTRL 	0x2D
#define D9_CURRENT_CTRL 	0x2E

#define CONFIG			0x36
#define ENGINE1_PC 		0x37
#define ENGINE2_PC 		0x38
#define ENGINE3_PC 		0x39
#define STATUS			0x3A
#define GPO			0x3B

#define RESET			0x3D

#define LED_TEST_CONTROL	0x41
#define LED_TEST_ADC		0x42

#define GROUP_FADER1		0x48
#define GROUP_FADER2		0x49
#define GROUP_FADER3		0x4A

#define ENG1_PROG_START_ADDR	0x4C
#define ENG2_PROG_START_ADDR	0x4D
#define ENG3_PROG_START_ADDR	0x4E
#define PROG_MEM_PAGE_SELECT	0x4F

#define PROG_MEM_START		0x50
#define PROG_MEM_END		0x6F

#define GAIN_CONTROL_CHANGE	0x76

#define ENGINE_CNTRL_ENG1_SHIFT	4
#define ENGINE_CNTRL_ENG2_SHIFT	2
#define ENGINE_CNTRL_ENG3_SHIFT	0

#define ENGINE_CNTRL1_HOLD	0
#define ENGINE_CNTRL1_STEP	1
#define ENGINE_CNTRL1_FREERUN	2
#define ENGINE_CNTRL1_EXECONCE	3

#define ENGINE_CNTRL2_DISABLE	0
#define ENGINE_CNTRL2_LOAD	1
#define ENGINE_CNTRL2_RUN	2
#define ENGINE_CNTRL2_HALT	3

#define CONFIG_CPMODE_OFF	(0 << 3)	
#define CONFIG_CPMODE_1x	(1 << 3)
#define CONFIG_CPMODE_15x	(2 << 3)
#define CONFIG_CPMODE_AUTO	(3 << 3)

#define CONFIG_POWER_SAVE_ON	(1 << 5)
#define CONFIG_POWER_SAVE_OFF	(0 << 5)

#define CONFIG_AUTO_INCR_ON	(1 << 6)
#define CONFIG_AUTO_INCR_OFF	(0 << 6)


// ioctl codes
#define LP8501_DOWNLOAD_MICROCODE		1
#define LP8501_START_ENGINE			2
#define LP8501_STOP_ENGINE			3
#define LP8501_WAIT_FOR_INTERRUPT		4
#define LP8501_CONFIGURE_MEMORY			5
#define LP8501_STOP_ENGINE_AFTER_INTERRUPT 	6
#define LP8501_READ_PWM				7
#define LP8501_WAIT_FOR_ENGINE_STOPPED		8

#define LP8501_STOP_ENGINE1	0x01
#define LP8501_STOP_ENGINE2	0x02
#define LP8501_STOP_ENGINE3	0x04


// The LP8501 only controls 9 LEDs so there are a max of 9 groups.
// NOTE: This is referring to software groups.  The LP8501
// is only capable of 3 hardware groups.
enum {
	GRP_1 = 0,
	GRP_2,
	GRP_3,
	GRP_4,
	GRP_5,
	GRP_6,
	GRP_7,
	GRP_8,
	GRP_9,
};

enum {
	HW_GRP_NONE = 0,
	HW_GRP_1,
	HW_GRP_2,
	HW_GRP_3,
};

enum {
	NONE = 0,
	RED,
	GREEN,
	BLUE,
	WHITE,
};

struct lp8501_read_pwm {
	u8 led;
	u8 value;
};

struct led_cfg {
	int type;
	u8 pwm_addr;
	u8 current_addr;
	u8 control_addr;
};


struct lp8501_memory_config {
	int eng1_startpage;
	int eng1_endpage;
	int eng2_startpage;
	int eng2_endpage;
	int eng3_startpage;
	int eng3_endpage;
};

struct lp8501_led_config {
	struct led_classdev cdev;
	struct i2c_client *client;
	struct led_cfg *led_list;
	struct work_struct brightness_work;
	int nleds;
	int brightness;
	int group_id;
	int hw_group;
	u8 default_current;
	int default_brightness;
	int default_state;
};

/* lp8501 LED platform data structure */
struct lp8501_platform_data {
    	struct lp8501_led_config *leds;
	struct lp8501_memory_config *memcfg;
	int nleds;
	u8 cp_mode;
	u8 power_mode;
	char *dev_name;
};

extern int
lp8501_i2c_read_reg(struct i2c_client * client, u8 addr, u8 * out);

extern int
lp8501_i2c_write_reg(struct i2c_client * client, u8 addr, u8 val);

#ifdef CONFIG_LEDS_LP8501_DBG
int  lp8501_dbg_register_i2c_client(struct i2c_client *i2c);
void lp8501_dbg_unregister_i2c_client(struct i2c_client *i2c);
#endif

#endif // LP8501
