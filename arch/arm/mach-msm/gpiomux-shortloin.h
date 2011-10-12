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
#ifndef __ARCH_ARM_MACH_MSM_GPIOMUX_SHORTLOIN_H
#define __ARCH_ARM_MACH_MSM_GPIOMUX_SHORTLOIN_H
#include "gpiomux.h"

void __init msm8x60_init_gpiomux(struct msm_gpiomux_configs *cfgs);
void msm8x60_gpiomux_lcdc_steadycfg(void);

extern struct msm_gpiomux_configs shortloin_gpiomux_cfgs[] __initdata;
extern struct msm_gpiomux_configs shortloin_3g_gpiomux_cfgs[] __initdata;
extern struct msm_gpiomux_configs shortloin_3g_gpiomux_cfgs_evt1[] __initdata;
extern struct msm_gpiomux_configs shortloin_3g_gpiomux_cfgs_proto[] __initdata;

#define UART1DM_RTS_GPIO 56
#define UART1DM_CTS_GPIO 55
#define UART1DM_RX_GPIO 54
#define UART1DM_TX_GPIO 53

/* brcm 4751 */
#define PM8058_GPIO4   3
#define PM8058_GPIO5   4
#define BRCM4751_REGPU_GPIO   PM8058_GPIO4
#define BRCM4751_RESET_GPIO   PM8058_GPIO5
#define BRCM4751_UART_RFR_GPIO     103
#define BRCM4751_UART_CTS_GPIO     104
#define BRCM4751_UART_RX_GPIO      105
#define BRCM4751_UART_TX_GPIO      106

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

#ifdef CONFIG_MAX8903B_CHARGER
/* max8903b control GPIOs */
#define MAX8903B_GPIO_DC_CHG_MODE	42
#define MAX8903B_GPIO_USB_CHG_MODE 	134
#define MAX8903B_GPIO_USB_CHG_SUS	33
#define MAX8903B_GPIO_CHG_D_ISET_1	34
#define MAX8903B_GPIO_CHG_D_ISET_2	30
#define MAX8903B_GPIO_CHG_EN		41
#define MAX8903B_GPIO_DC_OK		86
#define MAX8903B_GPIO_STATUS_N		36
#define MAX8903B_GPIO_FAULT_N		35
#endif  //CONFIG_MAX8903B_CHARGER

/* a6 */
#define SHORTLOIN_A6_0_TCK_EVT1		68
#define SHORTLOIN_A6_0_TCK		156
#define SHORTLOIN_A6_0_WAKEUP       155
#define SHORTLOIN_A6_0_TDIO		170
#define SHORTLOIN_A6_0_MSM_IRQ_EVT1		156
#define SHORTLOIN_A6_0_MSM_IRQ		37

#define SHORTLOIN_A6_1_TCK		115
#define SHORTLOIN_A6_1_WAKEUP		78
#define SHORTLOIN_A6_1_TDIO		116
#define SHORTLOIN_A6_1_MSM_IRQ_EVT1		132
#define SHORTLOIN_A6_1_MSM_IRQ		94

/* camera */
#define SHORTLOIN_CAM_I2C_DATA		47
#define SHORTLOIN_CAM_I2C_CLK		48
#define SHORTLOIN_CAMIF_MCLK		32
#define SHORTLOIN_WEBCAM_PWDN		107
#define SHORTLOIN_WEBCAM_RESET		(PM8058_GPIO_PM_TO_SYS(8-1))
#define SHORTLOIN_MAINCAM_PWDN		(PM8058_GPIO_PM_TO_SYS(9-1))


/* camera flash */
#define SHORTLOIN_WEBCAM_FLASH_EVT1	69
#define SHORTLOIN_WEBCAM_FLASH		158

/* touch */
#define MXT1386_TS_PEN_IRQ_GPIO		45
#define MXT1386_TS_PWR_RST_GPIO		70

#define GPIO_CTP_WAKE			45
#define GPIO_CTP_SCL			73
#define GPIO_CTP_SDA			72
#define GPIO_CTP_RX			71
#define GPIO_CY8CTMA395_XRES		70
/*WLAN*/
#define SHORTLOIN_GPIO_WL_HOST_WAKE	93		// input
#define SHORTLOIN_GPIO_HOST_WAKE_WL	80		// output
#define SHORTLOIN_GPIO_WLAN_RST_N	28		// output


/* gyro */
#define SHORTLOIN_GYRO_INT		75

/* proximity */
#define SHORTLOIN_PROX_INT		39

/* audio */
#define SHORTLOIN_AUD_HEAD_MIC_DET_IRQ_GPIO 57
#define SHORTLOIN_AUD_LDO1_EN		66
#define SHORTLOIN_AUD_LDO2_EN		108

/* cover detect */
#define SHORTLOIN_COVER_DET_INT		31

/* sdc3 */
#define SHORTLOIN_SDC3_DET_INT		37

/* lighting */
#define LM8502_LIGHTING_INT_IRQ_GPIO	77
#define LM8502_LIGHTING_EN_GPIO		121

/* usb host */
#define ISP1763_INT_GPIO		172
#define ISP1763_DACK_GPIO		169
#define ISP1763_DREQ_GPIO		29
#define ISP1763_RST_GPIO		152
#define GPIO_3G_3V3_EN_PROTO		158
#define GPIO_3G_3V3_EN_PRE_EVT3		82
#define GPIO_3G_3V3_EN			106

#define GPIO_3G_DISABLE_N		171
#define GPIO_3G_WAKE_N			38
#define GPIO_3G_UIM_CD_N		61

/* pmic */
#define PMIC1_APC_USR_IRQ_N		88
#define PMIC2_APC_USR_IRQ_N		91

/* sensors i2c gpio */
#define GPIO_SENSORS_SDA       43
#define GPIO_SENSORS_SCL       44
#define I2C_RECOVER_CLOCK_CYCLES   36

/*Proximity I2C*/
#define SHORTLOIN_PROX_I2C_DATA		68
#define SHORTLOIN_PROX_I2C_CLK		69

/*System force boot dis pin*/
#define GPIO_FORCE_BOOT_DIS		154

enum shortloin_pins_type {
	SHORTLOIN_A6_0_TCK_PIN,
	SHORTLOIN_A6_0_WAKEUP_PIN,
	SHORTLOIN_A6_0_TDIO_PIN,
	SHORTLOIN_A6_0_MSM_IRQ_PIN,
	SHORTLOIN_A6_1_TCK_PIN,
	SHORTLOIN_A6_1_WAKEUP_PIN,
	SHORTLOIN_A6_1_TDIO_PIN,
	SHORTLOIN_A6_1_MSM_IRQ_PIN,
	NUM_SHOTLOIN_PINS
};

/* PN544 NFC chip */
#define PM8058_NFC_IRQOUT	15
#define PM8058_NFC_WAKEUP	16
#define PM8058_NFC_GPIO4	17

#endif
