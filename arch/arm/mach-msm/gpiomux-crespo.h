/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2011 Hewlett-Packard Development Company, L.P.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#ifndef __ARCH_ARM_MACH_MSM_GPIOMUX_CRESPO_H
#define __ARCH_ARM_MACH_MSM_GPIOMUX_CRESPO_H
#include "gpiomux.h"

void __init msm8x60_init_gpiomux(struct msm_gpiomux_configs *cfgs);
void msm8x60_gpiomux_lcdc_steadycfg(void);

extern struct msm_gpiomux_configs crespo_gpiomux_cfgs[] __initdata;

#define UART1DM_RTS_GPIO 56
#define UART1DM_CTS_GPIO 55
#define UART1DM_RX_GPIO 54
#define UART1DM_TX_GPIO 53

/* BT */
#define BT_RST_N 122
#define BT_POWER 110
#define BT_WAKE 131
#define BT_HOST_WAKE 50

#ifdef CONFIG_KEYBOARD_GPIO_PE
/* GPIO Keys */
#define CORE_NAVI_GPIO  (40)
#define VOL_UP_GPIO     (PM8058_GPIO_PM_TO_SYS(6-1))
#define VOL_DN_GPIO     (PM8058_GPIO_PM_TO_SYS(7-1))
#define JACK_DET_GPIO           67
#endif

#ifdef CONFIG_A6
/* a6 */
#define CRESPO_A6_0_TCK			115
#define CRESPO_A6_0_WAKEUP		78
#define CRESPO_A6_0_TDIO		116
#define CRESPO_A6_0_MSM_IRQ		94
#endif

#ifdef CONFIG_WEBCAM_MT9M114
/* WebCamera MTM114 */
#define CRESPO_CAM_I2C_DATA		47
#define CRESPO_CAM_I2C_CLK		48
#define CRESPO_CAMIF_MCLK		32
#endif

/* touch */
#define MXT1386_TS_PEN_IRQ_GPIO		123
#define MXT1386_TS_PEN_IRQ_GPIO_3G	45
#define MXT1386_TS_PWR_RST_GPIO		70

#define GPIO_CTP_INT			123
#define GPIO_CTP_INT_3G			45
#define GPIO_CTP_SCL			73
#define GPIO_CTP_SDA			72
#define GPIO_CTP_RX			71
#define GPIO_CY8CTMA395_XRES		70
/*WLAN*/
#define CRESPO_GPIO_WL_HOST_WAKE	93		// input
#define CRESPO_GPIO_HOST_WAKE_WL	80		// output
#define CRESPO_GPIO_WLAN_RST_N	28		// output


/* gyro */
#define CRESPO_GYRO_INT		125

/* audio */
#define CRESPO_AUD_HEAD_MIC_DET_IRQ_GPIO 57
#define CRESPO_AUD_LDO1_EN		66
#define CRESPO_AUD_LDO2_EN		108

/* cover detect */
#define CRESPO_COVER_DET_INT		31

/* sdc3 */
#define CRESPO_SDC3_DET_INT		37

#ifdef CONFIG_USB_PEHCI_HCD
/* ISP1763 usb host */
#define ISP1763_INT_GPIO		31
#define ISP1763_DACK_GPIO		104
#define ISP1763_DREQ_GPIO		29
#define ISP1763_RST_GPIO		152
#define GPIO_3G_3V3_EN			82

#define GPIO_3G_DISABLE_N		103
#define GPIO_3G_WAKE_N			38
#define GPIO_3G_UIM_CD_N		61
#endif

/* pmic */
#define PMIC1_APC_USR_IRQ_N		88
#define PMIC2_APC_USR_IRQ_N		91

/* sensors i2c gpio */
#define GPIO_SENSORS_SDA       43
#define GPIO_SENSORS_SCL       44
#define I2C_RECOVER_CLOCK_CYCLES   36

/*Proximity I2C*/
#define CRESPO_PROX_I2C_DATA		68
#define CRESPO_PROX_I2C_CLK		69

/*System force boot dis pin*/
#define GPIO_FORCE_BOOT_DIS		154

#if defined(CONFIG_KEYBOARD_MA87712) || defined(CONFIG_KEYBOARD_MA87712_MODULE)
/* ma87712 keyboard gpio */
#define MA87712_KBC_RST_N		PM8058_GPIO_PM_TO_SYS(23-1)	// KBC_RST_N_PM8058_GPIO23, reset pin
#define MA87712_KBC_PD_N		PM8058_GPIO_PM_TO_SYS(22-1)	// KBC_PD_N_PM8058_GPIO22, power down pin
#define MA87712_KBC_INT 		105	// GPIO105, key board int
#endif

#endif
