/*
 * Copyright (C) 2007-2009 Palm, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef _BOARD_SIRLOIN_3430_H_
#define _BOARD_SIRLOIN_3430_H_

/* Project/Program specific definitions */
#define  PALM_PROJECT_CODENAME    ""
#define  PALM_DRIVER_PREFIX       ""
#define  PALM_DEVICE_PREFIX       ""

#define  SIRLOIN_3430_PHYS_IO_BASE   L4_BASE
#define  SIRLOIN_3430_PG_IO_VIRT     L4_VIRT
#define  SIRLOIN_3430_RAM_BASE       0x80000000

#ifdef CONFIG_TWL4030_CORE

/* Board specific addon to IRQs (same as sdp3430)     */
#define TWL4030_IRQNUM         INT_34XX_SYS_NIRQ

/* TWL4030 Primary Interrupt Handler (PIH) interrupts */
#define IH_TWL4030_BASE       (IH_BOARD_BASE)
#define IH_TWL4030_END        (IH_TWL4030_BASE+8)

#ifdef CONFIG_TWL4030_GPIO
/* TWL4030 GPIO Interrupts */
#define IH_TWL4030_GPIO_BASE  (IH_TWL4030_END)
#define IH_TWL4030_GPIO_END   (IH_TWL4030_BASE+18)
#define NR_IRQS               (IH_TWL4030_GPIO_END)
#else
#define NR_IRQS               (IH_TWL4030_END)
#endif /* CONFIG_I2C_TWL4030_GPIO */

#endif /* End of support for TWL4030 */

#ifdef CONFIG_KEYBOARD_GPIO_PE
/* GPIO Keys */
#define CORE_NAVI_GPIO  (17)
#define VOL_UP_GPIO     (24)
#define VOL_DN_GPIO     (25)
#define PTT_GPIO        (26)
#define SLIDER_GPIO     (27)
#define RING_GPIO       (28)
#define POWER_GPIO      (29)
#endif


/*
 *  I2C subsystem
 */
#if  defined(CONFIG_I2C_OMAP)

#define OMAP3_I2C_BASE1  0x48070000
#define OMAP3_I2C_BASE2  0x48072000
#define OMAP3_I2C_BASE3  0x48060000

#define OMAP3_I2C_INT1_GPIO   56
#define OMAP3_I2C_INT2_GPIO   57
#define OMAP3_I2C_INT3_GPIO   61

#endif

/*
 *  RTC
 */
#ifdef CONFIG_RTC_DRV_TWL4030

#define TWL4030_MSECURE_GPIO 22

#endif

/*
 *  User pins device
 */
#ifdef CONFIG_USER_PINS

#define AUDIO_PCM_R_TO_BT_CLK_GPIO             41
#define AUDIO_PCM_R_TO_BT_CLK_GPIO_PIN_MODE    "AB18_3430_GPIO41"

#define AUDIO_PCM_R_TO_BT_DATA_GPIO            42
#define AUDIO_PCM_R_TO_BT_DATA_GPIO_PIN_MODE   "AC19_3430_GPIO42"

#define AUDIO_PCM_REMOTE_MASTER_GPIO           43
#define AUDIO_PCM_REMOTE_MASTER_GPIO_PIN_MODE  "AB19_3430_GPIO43"

/*
 *   Modem pins
 */
#define MODEM_POWER_ON_GPIO               153 // OUT
#define MODEM_POWER_ON_GPIO_PIN_MODE      "AD1_3430_GPIO153"

#define MODEM_BOOT_MODE_GPIO              154 // OUT
#define MODEM_BOOT_MODE_GPIO_PIN_MODE     "AD2_3430_GPIO154"

#define MODEM_WAKE_MODEM_GPIO             157 // OUT
#define MODEM_WAKE_MODEM_GPIO_PIN_MODE    "AA21_3430_GPIO157"

#define MODEM_WAKE_APP_GPIO               152 // IN
#define MODEM_WAKE_APP_GPIO_PIN_MODE      "AE1_3430_GPIO152"

#define MODEM_WAKE_APP_USB_GPIO           155 // IN
#define MODEM_WAKE_APP_USB_GPIO_PIN_MODE  "AC1_3430_GPIO155"

/*
 * Charger bypass GPIO
 */
#define CHG_BYPASS_GPIO			158
#define CHG_BYPASS_GPIO_PIN_MODE	"V21_GPIO_158_CHG_BYPASS_EN"

/*
 *   Bt Pins
 */
#ifdef CONFIG_MACH_SIRLOIN_3630
  #define BT_RESET_GPIO                175
  #define BT_RESET_GPIO_PIN_MODE       "AC3_BT_nRST"
#else
  #define BT_RESET_GPIO                178
  #define BT_RESET_GPIO_PIN_MODE       "AA3_BT_nRST"
#endif

#define BT_WAKE_GPIO                 177
#define BT_WAKE_GPIO_PIN_MODE        "AB2_BT_nWAKE"

#define BT_HOST_WAKE_GPIO            176
#define BT_HOST_WAKE_GPIO_PIN_MODE   "AB1_BT_HOST_WAKE"

/*
 * WiFi Pins
 */

#ifdef CONFIG_MACH_SIRLOIN_3630
  #define WIFI_RESET_GPIO              127
  #define WIFI_RESET_GPIO_PIN_MODE     "P26_WL_RST_nPWD"

  #define WIFI_IRQ_GPIO                126
  #define WIFI_IRQ_GPIO_PIN_MODE      "P27_WL_IRQ"
#else

  #define WIFI_RESET_GPIO              139
  #define WIFI_RESET_GPIO_PIN_MODE     "AE3_WL_RST_nPWD"

  #define WIFI_IRQ_GPIO                138
  #define WIFI_IRQ_GPIO_PIN_MODE      "AF3_WL_IRQ"
#endif

/*
 *   Camera Pin
 */
#ifdef CONFIG_MACH_SIRLOIN_3630
#define CAM_FLASH_SYNC_GPIO          45
#define CAM_FLASH_SYNC_GPIO_PIN_MODE "K2_GPMC_D9_GPIO_45"
#else
#define CAM_FLASH_SYNC_GPIO          126
#define CAM_FLASH_SYNC_GPIO_PIN_MODE "D25_3430_GPIO126"
#endif

#endif

/*
 * HSDL9100 Proximity Sensor
 */
#if defined (CONFIG_HSDL9100_PROX_SENSOR) ||  \
    defined (CONFIG_HSDL9100_PROX_SENSOR_MODULE)

#ifdef CONFIG_MACH_SIRLOIN_3630
#define SIRLOIN_HSDL9100_DETECT_GPIO            (49)
#define SIRLOIN_HSDL9100_DETECT_GPIO_PIN_MODE   "T2_3430_GPIO49"
#else
#define SIRLOIN_HSDL9100_DETECT_GPIO            (55)
#define SIRLOIN_HSDL9100_DETECT_GPIO_PIN_MODE   "T8_3430_GPIO55"
#endif

#define SIRLOIN_HSDL9100_ENABLE_GPIO            (56)
#define SIRLOIN_HSDL9100_ENABLE_GPIO_PIN_MODE   "R8_3430_GPIO56"
#define SIRLOIN_HSDL9100_ENABLE_PWM_PIN_MODE    "R8_3430_GPT10_PWM_EVT"

#define SIRLOIN_HSDL9100_LEDON_GPIO             (182)
#define SIRLOIN_HSDL9100_LEDON_PWM_PIN_MODE     "V3_3430_GPT8_PWM_EVT"

#endif // CONFIG_HSDL9100_PROX_SENSOR


/* Board specific power control APIs */
int board_mmc1_vcc_30_enable (void);
int board_mmc1_vcc_disable   (void);

int board_mmc2_vcc_30_enable (void);
int board_mmc2_vcc_disable   (void);

int board_aux1_vcc_30_enable (void);
int board_aux1_vcc_disable   (void);

int board_aux2_vcc_30_enable (void);
int board_aux2_vcc_disable   (void);

int board_aux3_vcc_18_enable (void);
int board_aux3_vcc_disable   (void);

int board_aux4_vcc_30_enable (void);
int board_aux4_vcc_disable   (void);

int board_pll2_vcc_18_enable (void);
int board_pll2_vcc_disable   (void);

/*  functional area power control APIs */
int board_bt_vcc_enable      (int);
int board_wl_vcc_enable      (int);
int board_wl_rf_vcc_enable   (int);
int board_io_vcc_enable      (int);
int board_cam_vcc_enable     (int);
int board_sdi_vcc_enable     (int);
int board_lvds_enable        (int);
void board_lgt_vcc_enable    (int); 
void board_prox_vcc18_enable (void);
void board_prox_vcc18_disable(void);

/*  functional area mmc slot control APIs */
int board_rescan_mmc_slot   (void);
int board_rescan_wifi_slot  (void);

/* board type (set in arch/arm/mach-omap3pe/board-sirloin-3430.c) */
#define EVT0b	(0)
#define EVT1	(1)
#define EVT2	(2)
#define EVT3	(3)
#define DVT	(4)
#define DVT3	(5)
extern u32 board_type;

#endif /* _BOARD_SIRLOIN_3430_H_ */
