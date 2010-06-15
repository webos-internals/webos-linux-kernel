/*
 *  linux/include/asm-arm/arch-msm/msm_mmc.h
 */
#ifndef ASMARM_ARCHMSM_MMC_H
#define ASMARM_ARCHMSM_MMC_H

#include <asm/mach/mmc.h>

struct msm_mmc_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	u32 (*translate_vdd)(struct device *, unsigned int);
	unsigned int (*status)(struct device *);
	struct embedded_sdio_data *embedded_sdio;
	unsigned int status_irq;
	int (*register_status_notify)(void (*callback)(int card_present, void *dev_id), void *dev_id);
	u32 (*get_powersave_mode)(struct device *);

	/* 
	*  Indicates whether the host should initiate card detection
	*  after start
	*     0 - initiate the detection
	*     1 - do not initiate the detection
	*/
	u32 (*get_detection_on_start_mode)(struct device *);

	// Board specific callbacks
	int (*board_power_mode) (void *, int mode   );
	int (*board_probe)      (void * );
	int (*board_remove)     (void * );
	int (*board_suspend)    (void * );
	int (*board_resume)     (void * );
};

#endif
