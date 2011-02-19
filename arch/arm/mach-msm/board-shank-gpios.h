/* arch/arm/mach-msm/board-shank-gpios.h
 *
 * Copyright (C) 2009 Palm, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_MSM_SHANK_GPIOS_H
#define __ARCH_ARM_MACH_MSM_SHANK_GPIOS_H

#define GPIO_WIFI_RESET 	(162)
#define GPIO_WIFI_EN		(164)
#define GPIO_WIFI_INT		(142)

#define GPIO_BT_RESET		(163)
#define GPIO_BT_WAKE_HOST	(145)

#define UART1DM_RTS		(134)
#define UART1DM_CTS		(135)
#define UART1DM_RX		(136)
#define UART1DM_TX		(137)

#define UART3RTS		(57)
#define UART3CTS		(55)
#define UART3RX			(53)
#define UART3TX			(54)

#define GPIO_SLIDER_OPEN	(20)
#define GPIO_SLIDER_CLOSE	(44)
#define GPIO_RINGER		(180)
#define GPIO_VOL_UP		(37)
#define GPIO_VOL_DOWN		(43)
#define GPIO_POWER		(40)

#define GPIO_LCD_RESET		(125)
#define GPIO_LCD_TE		(24)

#define GPIO_VIBRATOR_EN	(56)
#define GPIO_VIBRATOR_PWM	(98)

#define GPIO_LP8501_INT		(42)
#define GPIO_LP8501_EN		(124)

#define GPIO_CTP_RESET		128
#define GPIO_CTP_INT		147
#define GPIO_CTP_SCLK		45
#define GPIO_CTP_MOSI		47
#define GPIO_CTP_MISO		48
#define GPIO_CTP_SS_EVT1	132
#define GPIO_CTP_SS_EVT2	46
#define GPIO_CTP_SHDN		30

#define GPIO_I2C_SCL	16
#define GPIO_I2C_SDA	17

#define GPIO_PROX_INT_EVT1	130
#define GPIO_PROX_INT		18

#define GPIO_MOTION_INT1		26
#define GPIO_MOTION_INT2		130
#define GPIO_MOTION_INT2_EVT1	18

#define GPIO_MMC_PWR_EVT2       132

#define GPIO_I2C_SCL_CHG	(70)
#define GPIO_I2C_SDA_CHG	(71)

#define GPIO_LDO1	(168)
#define GPIO_LDO2	(167)

int __init board_shank_gpios_init(void);

#endif
