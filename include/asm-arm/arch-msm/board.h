/* linux/include/asm-arm/arch-msm/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <linux/fb.h>

#include <asm/arch/msm_mmc.h>

/* platform device data structures */

struct mddi_panel_info;
struct msm_mddi_platform_data
{
	void (*mddi_client_power)(int on);
	void (*mddi_enable)(struct mddi_panel_info *panel, int on);
	void (*panel_power)(struct mddi_panel_info *panel, int on);
	unsigned has_vsync_irq:1;
	
	/* Some conrollers do not support partial screen updates */
	unsigned force_full_update:1;

	/* Some conroller do not support GETCAP command, force screen parameters
		to the values provided below in the structure */
	unsigned skip_auto_detect:1;

	/* Forced screen characteristics, applied only when skip_auto_detect is used */
	unsigned long screen_width;
	unsigned long screen_height;
	unsigned short mfr_name;
	unsigned short product_code;

	unsigned long fb_base;
	unsigned long fb_size;
	uint8_t bits_per_pixel;
};

struct msm_camera_device_platform_data{
	void (*camera_flash_current_ma)(int mode, int type, int current_ma);
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	const char *sensor_name;
};

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_map_halibut_io(void);
void __init msm_map_blenny_io(void);
void __init msm_init_irq(void);
void __init msm_init_gpio(void);

struct mmc_platform_data;
int __init msm_add_sdcc(unsigned int controller, struct msm_mmc_platform_data *plat);

/*  functional area mmc slot control APIs */
int board_rescan_mmc_slot   (void);
int board_rescan_wifi_slot  (void);

#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K) || defined(CONFIG_USB_GADGET_MSM_HSUSB)
void msm_hsusb_set_vbus_state(int online);
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif

void config_camera_on_gpios(void);
void config_camera_off_gpios(void);
#endif
