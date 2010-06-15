#ifndef _BOARD_JOPLIN_H_
#define _BOARD_JOPLIN_H_

/* Project/Program specific definitions */
#define  PALM_PROJECT_CODENAME    ""
#define  PALM_DRIVER_PREFIX       ""
#define  PALM_DEVICE_PREFIX       ""

#define  JOPLIN_PHYS_IO_BASE	OMAP2_L4_BASE
#define  JOPLIN_PG_IO_VIRT	L4_24XX_VIRT
#define  JOPLIN_RAM_BASE	0x80000000


#ifdef CONFIG_TWL4030_CORE

/* Board specific addon to IRQs (same as sdp2430)     */
#define TWL4030_IRQNUM  	INT_24XX_SYS_NIRQ

/* TWL4030 Primary Interrupt Handler (PIH) interrupts */
#define IH_TWL4030_BASE 	(IH_BOARD_BASE)
#define IH_TWL4030_END  	(IH_TWL4030_BASE+8)

#ifdef CONFIG_TWL4030_GPIO
/* TWL4030 GPIO Interrupts */
#define IH_TWL4030_GPIO_BASE	(IH_TWL4030_END)
#define IH_TWL4030_GPIO_END	(IH_TWL4030_BASE+18)
#define NR_IRQS  		(IH_TWL4030_GPIO_END)
#else
#define NR_IRQS  		(IH_TWL4030_END)
#endif /* CONFIG_I2C_TWL4030_GPIO */

#endif /* End of support for TWL4030 */


#define  JOPLIN_SD_CD_GPIO   93  


/*  functoinal area mmc slot control APIs */
int board_rescan_mmc_slot   (void);
int board_rescan_wifi_slot  (void);


#endif /* _BOARD_JOPLIN_H_ */
