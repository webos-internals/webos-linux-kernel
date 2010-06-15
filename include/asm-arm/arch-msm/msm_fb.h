/* drivers/video/msm_fb/msm_fb.h
 *
 * Internal shared definitions for various MSM framebuffer parts.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _MSM_FB_H_
#define _MSM_FB_H_

#include <linux/fb.h>
#include <linux/android_pmem.h>
#include <asm/arch/board.h>


#define PIPELINE_GRAPHICS 	1
#define PIPELINE_VIDEO		2

#define SUSPEND_DEEPSLEEP_MODE 0
#define SUSPEND_SHUTDOWN_MODE 1


struct msm_v4l2_pd  {
	uint32_t screen_width;
	uint32_t screen_height;
	uint32_t max_internal_bufs;
	uint32_t scaling_factor;
};

struct mddi_info;

struct mddi_panel_info
{
	struct mddi_info *mddi;
	struct mddi_panel_ops *panel_ops;
	uint16_t width;
	uint16_t height;
	unsigned force_full_update:1;
	unsigned long fb0_base;
	unsigned long fb0_size;
	unsigned long fb1_base;
	unsigned long fb1_size;
	unsigned long ibuf_base;
	unsigned long ibuf_size;
	uint8_t bits_per_pixel;
};

struct msmfb_callback {
	void (*func)(struct msmfb_callback *);
};

struct mddi_panel_ops
{
	void (*power)(struct mddi_panel_info *panel, int on);
	void (*enable)(struct mddi_panel_info *panel);
	void (*disable)(struct mddi_panel_info *panel);
	void (*wait_vsync)(struct mddi_panel_info *panel);
	void (*request_vsync)(struct mddi_panel_info *panel,
			      struct msmfb_callback *callback);
	void (*display_off)(struct mddi_panel_info *panel,
				struct msmfb_callback *callback);
	void (*display_on)(struct mddi_panel_info *panel,
				struct msmfb_callback *callback);
};

int mddi_suspend_mode(struct mddi_info *mddi);
bool mddi_use_vsync(struct mddi_info *mddi);
int mddi_vsync_gpio(struct mddi_info *mddi);
int mddi_add_panel(struct mddi_info *mddi, struct mddi_panel_ops *ops);
void mddi_remote_write(struct mddi_info *mddi, unsigned val, unsigned reg);
unsigned mddi_remote_read(struct mddi_info *mddi, unsigned reg);
void mddi_activate_link(struct mddi_info *mddi);
void mddi_hibernate_disable(struct mddi_info *mddi, int on);

void mdp_dma_to_mddi(uint32_t addr, uint32_t stride, uint32_t width,
		     uint32_t height, uint32_t x, uint32_t y,
		     struct msmfb_callback* callback, uint32_t dma2_ibufformat, bool blocking);
void mdp_dma_wait(void);
int mdp_ppp_wait(void);
int enable_mdp_irq(uint32_t mask);
int disable_mdp_irq(uint32_t mask);
void mdp_set_grp_disp(unsigned disp_id);

struct fb_info;
struct mdp_blit_int_req;
int mdp_blit(struct mdp_blit_int_req *req);

int msmfb_switch_pipeline(int pipeline);
int msmfb_overlay(struct mdp_blit_int_req *req, struct msmfb_callback* user_callback, bool new_update);
void msmfb_get_last_frame(unsigned long *addr, unsigned long *len);

void mdp_print_isr_status(void);

#endif
