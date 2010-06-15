#ifndef _BOARD_JOPLIN_3430_H_
#define _BOARD_JOPLIN_3430_H_

/* Project/Program specific definitions */
#define  PALM_PROJECT_CODENAME    ""
#define  PALM_DRIVER_PREFIX       ""
#define  PALM_DEVICE_PREFIX       ""

#define  JOPLIN_3430_PHYS_IO_BASE	L4_BASE
#define  JOPLIN_3430_PG_IO_VIRT  	L4_VIRT
#define  JOPLIN_3430_RAM_BASE    	0x80000000

#ifdef CONFIG_TWL4030_CORE

/* Board specific addon to IRQs (same as sdp3430)     */
#define TWL4030_IRQNUM  	INT_34XX_SYS_NIRQ

/* TWL4030 Primary Interrupt Handler (PIH) interrupts */
#define IH_TWL4030_BASE 	(IH_BOARD_BASE)
#define IH_TWL4030_END  	(IH_TWL4030_BASE+8)

#ifdef CONFIG_TWL4030_GPIO
/* TWL4030 GPIO Interrupts */
#define IH_TWL4030_GPIO_BASE	(IH_TWL4030_END)
#define IH_TWL4030_GPIO_END 	(IH_TWL4030_BASE+18)
#define NR_IRQS 		(IH_TWL4030_GPIO_END)
#else
#define NR_IRQS 		(IH_TWL4030_END)
#endif /* CONFIG_I2C_TWL4030_GPIO */

#endif /* End of support for TWL4030 */


#define  JOPLIN_SD_CD_GPIO   136

/*
 *  I2C subsystem
 */
#if  defined(CONFIG_I2C_OMAP)

#define OMAP3_I2C_BASE1 	0x48070000
#define OMAP3_I2C_BASE2 	0x48072000
#define OMAP3_I2C_BASE3 	0x48060000

#define OMAP3_I2C_INT1  	56
#define OMAP3_I2C_INT2  	57
#define OMAP3_I2C_INT3  	61

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
 *   Bt Pins
 */
#define BT_RESET_GPIO                178
#define BT_RESET_GPIO_PIN_MODE       "AA3_BT_nRST"

#define BT_WAKE_GPIO                 177
#define BT_WAKE_GPIO_PIN_MODE        "AB2_BT_nWAKE"

#define BT_HOST_WAKE_GPIO            176
#define BT_HOST_WAKE_GPIO_PIN_MODE   "AB1_BT_HOST_WAKE"

#endif

/*
 * HSDL9100 Proximity Sensor
 */
#ifdef CONFIG_HSDL9100_PROX_SENSOR

#define JOPLIN_HSDL9100_INT_GPIO (55)
#define JOPLIN_HSDL9100_ENABLE_GPIO (156)

#endif


/*  functoinal area mmc slot control APIs */
int board_rescan_mmc_slot   (void);
int board_rescan_wifi_slot  (void);


#endif /* _BOARD_JOPLIN_3430_H_ */
