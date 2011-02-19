/*
 *  include/linux/i2c_lm8502_led.h
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

#ifndef _LM8502_H
#define _LM8502_H

#include <linux/i2c.h>
#include <linux/leds.h>

#define LED_OFF		0
#define LED_ON		1

#define LM8502_I2C_DEVICE   	"LM8502"
#define LM8502_I2C_DRIVER   	"LM8502"
#define LM8502_I2C_DBG_DEVICE   "lm8502-dbg"
#define LM8502_I2C_DBG_DRIVER   "lm8502-dbg"
#define LM8502_I2C_ADDR		0x33
#define INSTR_LEN		96
#define LM8502_INSTR_LEN_PER_PAGE 16

/* Registers */
#define ENGINE_CNTRL1		0x00
#define ENGINE_CNTRL2		0x01
#define GROUP_FADING1		0x02
#define GROUP_FADING2		0x03
#define GROUP_FADING3		0x04

#define D1_CONTROL		0x06
#define D2_CONTROL		0x07
#define D3_CONTROL		0x08
#define D4_CONTROL		0x09
#define D5_CONTROL		0x0A
#define D6_CONTROL		0x0B
#define D7_CONTROL		0x0C
#define D8_CONTROL		0x0D
#define D9_CONTROL		0x0E
#define D10_CONTROL		0x0F

#define HAPTIC_CONTROL 		0x10

#define ALS_CONTROL 		0x11
#define ZLINE0			0x12
#define ZLINE1			0x13
#define ZLINE2			0x14
#define ZLINE3			0x15
#define TARGET_LIGHT_Z0		0x16
#define TARGET_LIGHT_Z1		0x17
#define TARGET_LIGHT_Z2		0x18
#define TARGET_LIGHT_Z3		0x19
#define TARGET_LIGHT_Z4		0x1A
#define ALS_START_VALUE		0x1B
#define DBC_CONTROL		0x1D

#define HAPTIC_FEEDBACK_CTRL	0x21
#define HAPTIC_PWM_DUTY_CYCLE	0x22

#define D1_CURRENT_CTRL		0x26
#define D2_CURRENT_CTRL		0x27
#define D3_CURRENT_CTRL		0x28
#define D4_CURRENT_CTRL 	0x29
#define D5_CURRENT_CTRL 	0x2A
#define D6_CURRENT_CTRL 	0x2B
#define D7_CURRENT_CTRL 	0x2C
#define D8_CURRENT_CTRL 	0x2D
#define D9_CURRENT_CTRL 	0x2E
#define D10_CURRENT_CTRL 	0x2F

#define ADAPT_FLASH_CTRL	0x35
#define MISC			0x36

#define ENGINE1_PC 		0x37
#define ENGINE2_PC 		0x38

#define STATUS			0x3A
#define INT			0x3B
#define I2C_VARIABLE		0x3C
#define RESET			0x3D

#define LED_TEST_CONTROL	0x41
#define LED_TEST_ADC		0x42

#define GROUP_FADER1		0x48
#define GROUP_FADER2		0x49
#define GROUP_FADER3		0x4A

#define ENG1_PROG_START_ADDR	0x4C
#define ENG2_PROG_START_ADDR	0x4D
#define PROG_MEM_PAGE_SELECT	0x4F

#define PROG_MEM_START		0x50
#define PROG_MEM_END		0x6F

#define TORCH_BRIGHTNESS	0xA0
#define FLASH_BRIGHTNESS	0xB0
#define FLASH_DURATION		0xC0
#define FLAG_REGISTER		0xD0
#define CONFIG_REG1		0xE0
#define CONFIG_REG2		0xF0

#define ENGINE_CNTRL_ENG1_SHIFT	4
#define ENGINE_CNTRL_ENG2_SHIFT	2

#define ENGINE_CNTRL1_HOLD	0
#define ENGINE_CNTRL1_STEP	1
#define ENGINE_CNTRL1_FREERUN	2
#define ENGINE_CNTRL1_EXECONCE	3

#define ENGINE_CNTRL2_DISABLE	0
#define ENGINE_CNTRL2_LOAD	1
#define ENGINE_CNTRL2_RUN	2
#define ENGINE_CNTRL2_HALT	3

#define MISC_POWER_SAVE_ON	(1 << 5)
#define MISC_POWER_SAVE_OFF	(0 << 5)

#define STROBE_TIMEOUT		(1 << 8)
#define FLASH_MODE		(3 << 0)
#define TORCH_MODE		(2 << 0)

/*ioctl codes */
#define LM8502_DOWNLOAD_MICROCODE		1
#define LM8502_START_ENGINE			2
#define LM8502_STOP_ENGINE			3
#define LM8502_WAIT_FOR_INTERRUPT		4
#define LM8502_CONFIGURE_MEMORY			5
#define LM8502_STOP_ENGINE_AFTER_INTERRUPT 	6
#define LM8502_READ_PWM				7
#define LM8502_WAIT_FOR_ENGINE_STOPPED		8

#define LM8502_STOP_ENGINE1	0x01
#define LM8502_STOP_ENGINE2	0x02
#define LM8502_STOP_ENGINE3	0x04


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

struct lm8502_read_pwm {
	u8 led;
	u8 value;
};

struct led_cfg {
	int type;
	u8 current_addr;
	u8 control_addr;
};


struct lm8502_memory_config {
	int eng1_startpage;
	int eng1_endpage;
	int eng2_startpage;
	int eng2_endpage;
};

struct lm8502_led_config {
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
struct lm8502_platform_data {
	int enable_gpio;
	int interrupt_gpio;
	int strobe_gpio;
	u8 vib_default_duty_cycle;
	u8 vib_default_direction;
	u8 vib_invert_direction;
	u16 flash_default_duration;
	u16 flash_default_current;
	u16 torch_default_current;
	void (*select_flash) (struct i2c_client* client);
	void (*select_vibrator) (struct i2c_client* client);
    	struct lm8502_led_config *leds;
	struct lm8502_memory_config *memcfg;
	int nleds;
	u8 power_mode;
	char *dev_name;
};

int lm8502_i2c_write_reg(struct i2c_client* client, u8 addr, u8 val);
int lm8502_i2c_read_reg(struct i2c_client* client, u8 addr, u8* out);

#ifdef CONFIG_LEDS_LM8502_DBG
int  lm8502_dbg_register_i2c_client(struct i2c_client *i2c);
void lm8502_dbg_unregister_i2c_client(struct i2c_client *i2c);
#endif

#endif // LP8502
