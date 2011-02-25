#ifndef _ARCH_LCD_H_
#define _ARCH_LCD_H_

#include <asm/arch/display.h>

#define PANEL_TYPE_ACX567AKM_TS1	0
#define PANEL_TYPE_ACX567AKM_TS2	1

struct bl_parameters
{
	u32 min_brightness;
	u32 max_brightness;

	u32 should_fade_in;
	u32 fade_in_step;
	u32 fade_in_msecs;

	u32 should_fade_out;	/* should we fade when the screen goes out? */
	u32 fade_out_step;	/* how much each fade step should be */
	u32 fade_out_msecs;	/* how much TOTAL time fade should be */
};

struct bl_platform_data
{
	struct device *parent;

	u32 timer_id;
	u32 i2c_id;
	u32 led_id;

	struct bl_parameters bl_params;

};

struct controller_platform_data
{
	struct device *parent;
	struct fb_var_screeninfo screen_info;
	struct omap_dss_config dss_config_lpr_on;
	struct omap_dss_config dss_config_lpr_off;

	/* controller functionalities */
	u32 *use_sdi;
	u32 *vsync_gated;
	u32 *right_margin;

	/* controller settings */
	u32 dss_reg_base;
	u32 dispc_reg_offset;
	u32 dss_reg_offset;

	u32 pixclk_min;
	u32 pixclk_max;

	u32 dispc_pol_freq;
	u32 dispc_control;

	/* sdi settings (if used) */
	u32 sdi_color_depth;
	u32 sdi_num_of_data_pairs;
	u32 sdi_pll_lock_method;
	u32 sdi_pll_to_pclk_ratio;
	u32 sdi_freq_selector;
	int (*sdi_power_enable)(int enable);
	int (*lvds_power_enable)(int enable);
};

struct panel_platform_data
{
	struct device *parent;

	u32 *panel_type;

	u32 *panel_use_gpio_enable;
	u32 panel_enable_gpio;

	u32 *panel_use_gpio_reset;
	u32 panel_reset_gpio;

	/* for displays that have their own backlight control */
	struct bl_parameters bl_params;

	/* for displays that do not completely control their own backlight*/
	void *sb_config;
	u8  use_sb;
	u8  on_full;

	void *(*sbc_init)(void *config_data);
	void (*sbc_remove)(void *sbc_data);
	void (*sbc_preon)(void *data);
	void (*sbc_on)(void *data);
	void (*sbc_brightness)(void *data, int brightness, int yield);
	void (*sbc_off)(void *sbc_data);

};

struct vout_mem_alloc_data
{
	u32 mem_width;
	u32 mem_height;
	u32 mem_bytes_per_pixel;
	s32 forced_index;
	s32 use_forced_index;
};

struct omapfbv_platform_data
{
	int	  					video_layer;
	u32						pixelformat;
	enum v4l2_colorspace	colorspace;
	struct fb_var_screeninfo  screeninfo;
	struct fb_fix_screeninfo  fix;
};




#endif /* _ARCH_LCD_H_ */
