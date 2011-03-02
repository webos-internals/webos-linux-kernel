/*
 * arch/arm/mach-omap2/display.c
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP24xx camera driver
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * History:
 * 20-APR-2006  Khasim		Modified VRFB based Rotation equations,
 *				The image data is always read from 0 degree
 *				view and written to the virtual space of desired
 *				rotation angle
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/hardware.h>
#include <asm/arch/display.h>
#include <asm/arch/venc.h>
#include <asm/arch/clock.h>

#define DPRINTK(args...)
//#define DPRINTK(args...) printk(args)

#ifndef CONFIG_ARCH_OMAP3410
static int omap2_current_tvstandard = NTSC_M;
#endif
static spinlock_t dss_lock;
static struct omap_dss_regs dss_ctx;
static struct tvlcd_status_t tvlcd_status;

/***** CLOCKS *****************************************************************/

static struct clk *dss1i;
static struct clk *dss1f;
#ifdef CONFIG_OMAP_DSI
static struct clk *dss2f;
#endif
static struct clk *tv_clk;

/******************************************************************************/

static struct {
	int mode;
	int state;
	int cfg_valid;;
	struct omap_dss_config on;
	struct omap_dss_config off;
} lpr = { 0 };

static struct {
	int height;
	int width;
} lcd_panel_size = { 0 };

void omap2_disp_handle_lpr_auto_mode(void);
static void omap2_disp_set_fifo_thresholds(void);
static void omap2_disp_set_logic_dividers(void);

/******************************************************************************/

struct omap2_disp_dma_params {
	u32 ba0;
	u32 ba1;
	int row_inc;
	int pix_inc;
};

/******************************************************************************/

static struct layer_t {
	int output_dev;
	int in_use;
	int ctx_valid;

	/* one set of dma parameters each for LCD and TV */
	struct omap2_disp_dma_params dma[2];

	int size_x;
	int size_y;
} layer[DSS_CTX_NUMBER] = {
	{ .ctx_valid = 0,}, { .ctx_valid = 0,}, { .ctx_valid = 0,},
	{ .ctx_valid = 0,}, { .ctx_valid = 0,},
};

/******************************************************************************/

#define MAX_ISR_NR   8
static int omap2_disp_irq;
static struct {
	omap2_disp_isr_t isr;
	void *arg;
	unsigned int mask;
} registered_isr[MAX_ISR_NR];

/* VRFB offset computation parameters */
#define SIDE_H		1
#define SIDE_W		0

/*
 * DSS register I/O routines
 */
static __inline__ u32  dss_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + DSS_REG_OFFSET + offset);
}
static __inline__ void dss_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + DSS_REG_OFFSET + offset);
}
static __inline__ u32  dss_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + DSS_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/*
 * Display controller register I/O routines
 */
static __inline__ u32 dispc_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + DISPC_REG_OFFSET + offset);
}

static __inline__ u32 dispc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + DISPC_REG_OFFSET + offset);
	return val;
}
static __inline__ u32 dispc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + DISPC_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/*
 * RFBI controller register I/O routines
 */
static __inline__ u32 rfbi_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + RFBI_REG_OFFSET + offset);
}

static __inline__ u32 rfbi_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + RFBI_REG_OFFSET + offset);
	return val;
}

/*
 * VENC register I/O Routines
 */
static __inline__ u32 venc_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + VENC_REG_OFFSET + offset);
}
static __inline__ u32 venc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + VENC_REG_OFFSET + offset);
	return val;
}
static __inline__ u32 venc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = DSS_REG_BASE + VENC_REG_OFFSET + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/* Write the color space conversion coefficients to the display controller
 * registers.  Each coefficient is a signed 11-bit integer in the range
 * [-1024, 1023].  The matrix coefficients are:
 *	[ RY  RCr  RCb ]
 *	[ GY  GCr  GCb ]
 *	[ BY  BCr  BCb ]
 */
s16 current_colorconv_values[2][3][3];

static void set_colorconv(int v, int colorspace)
{
	unsigned long ccreg;
	s16 mtx[3][3];
	int i, j;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			mtx[i][j] = current_colorconv_values[v][i][j];
		}
	}

	ccreg = (mtx[0][0] & 0x7ff) | ((mtx[0][1] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF0(v), ccreg);
	ccreg = (mtx[0][2] & 0x7ff) | ((mtx[1][0] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF1(v), ccreg);
	ccreg = (mtx[1][1] & 0x7ff) | ((mtx[1][2] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF2(v), ccreg);
	ccreg = (mtx[2][0] & 0x7ff) | ((mtx[2][1] & 0x7ff) << 16);
	dispc_reg_out(DISPC_VID_CONV_COEF3(v), ccreg);
	ccreg = mtx[2][2] & 0x7ff;
	dispc_reg_out(DISPC_VID_CONV_COEF4(v), ccreg);

	if (colorspace == V4L2_COLORSPACE_JPEG
	    || colorspace == V4L2_COLORSPACE_SRGB) {
		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
				DISPC_VID_ATTRIBUTES_VIDFULLRANGE,
				DISPC_VID_ATTRIBUTES_VIDFULLRANGE);
	}
	else
	{
		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
				0,
				DISPC_VID_ATTRIBUTES_VIDFULLRANGE);
	}
}

static void update_colorconv_mtx(int v, const s16 mtx[3][3])
{
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			current_colorconv_values[v][i][j] = mtx[i][j];
}

void omap2_disp_set_default_colorconv(int ltype, struct v4l2_pix_format *pix)
{
	int v;

	if (ltype == OMAP2_VIDEO1)
		v = 0;
	else if (ltype == OMAP2_VIDEO2)
		v = 1;
	else
		return;

	switch (pix->colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
	case V4L2_COLORSPACE_SMPTE240M:
	case V4L2_COLORSPACE_BT878:
	case V4L2_COLORSPACE_470_SYSTEM_M:
	case V4L2_COLORSPACE_470_SYSTEM_BG:
		/* luma (Y) range lower limit is 16, BT.601 standard */
		update_colorconv_mtx(v, cc_bt601);
		set_colorconv(v, pix->colorspace);
		break;
	case V4L2_COLORSPACE_REC709:
		/* luma (Y) range lower limit is 16, BT.709 standard */
		update_colorconv_mtx(v, cc_bt709);
		set_colorconv(v, pix->colorspace);
		break;
	case V4L2_COLORSPACE_JPEG:
	case V4L2_COLORSPACE_SRGB:
		/* full luma (Y) range, assume BT.601 standard */
		update_colorconv_mtx(v, cc_bt601_full);
		set_colorconv(v, pix->colorspace);
		break;
	}
}

void omap2_disp_set_colorconv(int ltype, struct v4l2_pix_format *pix)
{
	int v;

	if (ltype == OMAP2_VIDEO1)
		v = 0;
	else if (ltype == OMAP2_VIDEO2)
		v = 1;
	else
		return;

	set_colorconv(v, pix->colorspace);
}

/* Write the horizontal and vertical resizing coefficients to the display
 * controller registers. Each coefficient is a signed 8-bit integer in the
 * range [-128, 127] except for the middle coefficient (vc[1][i] and hc[3][i])
 * which is an unsigned 8-bit integer in the range [0, 255].  The first index
 * of the matrix is the coefficient number (0 to 2 vertical or 0 to 4
 * horizontal) and the second index is the phase (0 to 7).
 */
static void
omap2_disp_set_resize(int v, const char (*vc)[8], const char (*hc)[8], int ratio)
{
	int i;
	u32 reg;

	for (i = 0; i < 8; i++) {
		if (ratio <= 2) {
			reg = (  hc[4][i] & 0xff) |
			       ((hc[3][i] & 0xff) << 8) |
			       ((hc[2][i] & 0xff) << 16) |
			       ((hc[1][i] & 0xff) << 24);
			dispc_reg_out(DISPC_VID_FIR_COEF_H(v, i), reg);
			// printk("H%d - 0x%X\n", i, reg);

			reg = (  hc[0][i] & 0xff) |
			       ((vc[2][i] & 0xff) << 8) |
			       ((vc[1][i] & 0xff) << 16) |
			       ((vc[0][i] & 0xff) << 24);
			dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);
			// printk("HV%d - 0x%X\n", i, reg);
		} else {
			reg = ((hc[1][i] & 0xff) << 24) |
			      ((hc[2][i] & 0xff) << 16) |
			      ((hc[3][i] & 0xff) << 8) |
			       (hc[4][i] & 0xff);
			dispc_reg_out(DISPC_VID_FIR_COEF_H(v, i), reg);
			// printk("H%d - 0x%X\n", i, reg);

			reg = (  hc[0][i] & 0xff) |
			       ((vc[3][i] & 0xff) << 8) |
			       ((vc[2][i] & 0xff) << 16) |
			       ((vc[1][i] & 0xff) << 24);
			dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);
			// printk("HV%d - 0x%X\n", i, reg);

			reg = ((vc[0][i] & 0xff) << 8) | (vc[4][i] & 0xff);
			dispc_reg_out(DISPC_VID_FIR_COEF_V(v, i), reg);
			// printk("V%d - 0x%X\n", i, reg);
		}
	}
}

/*---------------------------------------------------------------------------*/

void omap2_disp_get_panel_size(int output_dev, int *width, int *height)
{
	unsigned long size;

	if (output_dev == OMAP2_OUTPUT_TV) {
		size = dispc_reg_in(DISPC_SIZE_DIG);
		*width = 1 + ((size & DISPC_SIZE_DIG_PPL)
			      >> DISPC_SIZE_DIG_PPL_SHIFT);
		*height = 1 + ((size & DISPC_SIZE_DIG_LPP)
			       >> DISPC_SIZE_DIG_LPP_SHIFT);
		*height = *height << 1;
	} else if (output_dev == OMAP2_OUTPUT_LCD) {
		size = dispc_reg_in(DISPC_SIZE_LCD);
		if (lcd_panel_size.width == 0) {
			*width = 1 + ((size & DISPC_SIZE_LCD_PPL)
					  >> DISPC_SIZE_LCD_PPL_SHIFT);
		}
		else {
			*width = lcd_panel_size.width;
		}
		if (lcd_panel_size.height == 0) {
			*height = 1 + ((size & DISPC_SIZE_LCD_LPP)
			       >> DISPC_SIZE_LCD_LPP_SHIFT);
		}
		else {
			*height = lcd_panel_size.height;
		}
	}
}
void omap2_disp_set_panel_size(int output_dev, int width, int height)
{
	unsigned long size;

	if (output_dev == OMAP2_OUTPUT_TV) {
		height = height >> 1;
		size =
		    ((width -
		      1) << DISPC_SIZE_DIG_PPL_SHIFT) & DISPC_SIZE_DIG_PPL;
		size |= ((height - 1) << DISPC_SIZE_DIG_LPP_SHIFT)
		    & DISPC_SIZE_DIG_LPP;
		dispc_reg_out(DISPC_SIZE_DIG, size);
	} else if (output_dev == OMAP2_OUTPUT_LCD) {
		size =
		    ((width -
		      1) << DISPC_SIZE_LCD_PPL_SHIFT) & DISPC_SIZE_LCD_PPL;
		size |= ((height - 1) << DISPC_SIZE_LCD_LPP_SHIFT)
		    & DISPC_SIZE_LCD_LPP;
		dispc_reg_out(DISPC_SIZE_LCD, size);
		lcd_panel_size.width = width;
		lcd_panel_size.height = height;
	}
}

static int graphics_in_use = 0;

/* Turn off the GFX, or video1, or video2 layer. */
void omap2_disp_disable_layer(int ltype)
{
	unsigned long attributes;
	int digital, v;

	if (ltype == OMAP2_GRAPHICS) {
		attributes = dispc_reg_merge(DISPC_GFX_ATTRIBUTES, 0,
					     DISPC_GFX_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT;
		graphics_in_use = 0;
	} else {
		if (ltype == OMAP2_VIDEO1)
			v = 0;
		else if (ltype == OMAP2_VIDEO2)
			v = 1;
		else
			return;

		attributes = dispc_reg_merge(DISPC_VID_ATTRIBUTES(v), 0,
					     DISPC_VID_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;
	}
	if (digital) {
		/* digital output */
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);
	} else {
		/* LCD */
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
				DISPC_CONTROL_GOLCD);
	}

	dispc_reg_merge(DISPC_CONTROL, 0, DISPC_CONTROL_OVERLAYOPTIMIZATION);

	DPRINTK("DISPLAY: Layer %d DISABLED\n", ltype);
}

int omap2_disp_is_video_layer_enabled(int ltype)
{
	int v = 0;
	int enabled = 0;
	if (ltype == OMAP2_GRAPHICS) {
		enabled = graphics_in_use;
	} else {
		if (ltype == OMAP2_VIDEO1)
			v = 0;
		else if (ltype == OMAP2_VIDEO2)
			v = 1;
		else
			return 0;
		enabled = ((dispc_reg_in(DISPC_VID_ATTRIBUTES(v)))&DISPC_GFX_ATTRIBUTES_ENABLE)?1:0;
	}
	return enabled;
}

/* Turn on the GFX, or video1, or video2 layer. */
void omap2_disp_enable_layer(int ltype)
{
	unsigned long attributes;
	int digital, v;

	DPRINTK("DISPLAY: Layer %d ENABLING\n", ltype);

	if (ltype == OMAP2_GRAPHICS) {
		attributes = dispc_reg_merge(DISPC_GFX_ATTRIBUTES,
					     DISPC_GFX_ATTRIBUTES_ENABLE,
					     DISPC_GFX_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT;
		graphics_in_use = 1;
	} else {
		if (ltype == OMAP2_VIDEO1)
			v = 0;
		else if (ltype == OMAP2_VIDEO2)
			v = 1;
		else
			return;

		attributes = dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
					     DISPC_VID_ATTRIBUTES_ENABLE,
					     DISPC_VID_ATTRIBUTES_ENABLE);
		digital = attributes & DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;
	}
	if (digital) {
		/* digital output */
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);
	} else {
		/* LCD */
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
				DISPC_CONTROL_GOLCD);
	}
}

/******************************************************************************
 *
 * SAVE / RESTORE context
 *
 ******************************************************************************/

static void omap2_disp_save_ctx(int ltype)
{
	int v1 = 0, v2 = 1;
	struct omap24xx_dispc_regs *dispc = &dss_ctx.dispc;

	switch (ltype) {
	case OMAP_DSS_GENERIC:
		dss_ctx.sysconfig   = dss_reg_in(DSS_SYSCONFIG);
		dss_ctx.control     = dss_reg_in(DSS_CONTROL);
#ifdef CONFIG_ARCH_OMAP3430
		dss_ctx.sdi_control = dss_reg_in(DSS_SDI_CONTROL);
		dss_ctx.pll_control = dss_reg_in(DSS_PLL_CONTROL);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc->revision       = dispc_reg_in(DISPC_REVISION);
		dispc->sysconfig      = dispc_reg_in(DISPC_SYSCONFIG);
		dispc->sysstatus      = dispc_reg_in(DISPC_SYSSTATUS);
		dispc->irqstatus      = dispc_reg_in(DISPC_IRQSTATUS);
		dispc->irqenable      = dispc_reg_in(DISPC_IRQENABLE);
		dispc->control        = dispc_reg_in(DISPC_CONTROL);
		dispc->config         = dispc_reg_in(DISPC_CONFIG);
		dispc->capable        = dispc_reg_in(DISPC_CAPABLE);
		dispc->default_color0 = dispc_reg_in(DISPC_DEFAULT_COLOR0);
		dispc->default_color1 = dispc_reg_in(DISPC_DEFAULT_COLOR1);
		dispc->trans_color0   = dispc_reg_in(DISPC_TRANS_COLOR0);
		dispc->trans_color1   = dispc_reg_in(DISPC_TRANS_COLOR1);
		dispc->line_status    = dispc_reg_in(DISPC_LINE_STATUS);
		dispc->line_number    = dispc_reg_in(DISPC_LINE_NUMBER);
		dispc->data_cycle1    = dispc_reg_in(DISPC_DATA_CYCLE1);
		dispc->data_cycle2    = dispc_reg_in(DISPC_DATA_CYCLE2);
		dispc->data_cycle3    = dispc_reg_in(DISPC_DATA_CYCLE3);
		dispc->timing_h       = dispc_reg_in(DISPC_TIMING_H);
		dispc->timing_v       = dispc_reg_in(DISPC_TIMING_V);
		dispc->pol_freq       = dispc_reg_in(DISPC_POL_FREQ);
		dispc->divisor        = dispc_reg_in(DISPC_DIVISOR);
		dispc->size_lcd       = dispc_reg_in(DISPC_SIZE_LCD);
		dispc->size_dig       = dispc_reg_in(DISPC_SIZE_DIG);
		break;

	case OMAP2_GRAPHICS:
		dispc->gfx_ba0            = dispc_reg_in(DISPC_GFX_BA0);
		dispc->gfx_ba1            = dispc_reg_in(DISPC_GFX_BA1);
		dispc->gfx_position       = dispc_reg_in(DISPC_GFX_POSITION);
		dispc->gfx_size           = dispc_reg_in(DISPC_GFX_SIZE);
		dispc->gfx_attributes     = dispc_reg_in(DISPC_GFX_ATTRIBUTES);
		dispc->gfx_fifo_size      = dispc_reg_in(DISPC_GFX_FIFO_SIZE);
		dispc->gfx_fifo_threshold = dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD);
		dispc->gfx_row_inc        = dispc_reg_in(DISPC_GFX_ROW_INC);
		dispc->gfx_pixel_inc      = dispc_reg_in(DISPC_GFX_PIXEL_INC);
		dispc->gfx_window_skip    = dispc_reg_in(DISPC_GFX_WINDOW_SKIP);
		dispc->gfx_table_ba       = dispc_reg_in(DISPC_GFX_TABLE_BA);
		break;

	case OMAP2_VIDEO1:
		dispc->vid1_ba0            = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_ba1            = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_position       = dispc_reg_in(DISPC_VID_POSITION(v1));
		dispc->vid1_size           = dispc_reg_in(DISPC_VID_SIZE(v1));
		dispc->vid1_attributes     = dispc_reg_in(DISPC_VID_ATTRIBUTES(v1));
		dispc->vid1_fifo_size      = dispc_reg_in(DISPC_VID_FIFO_SIZE(v1));
		dispc->vid1_fifo_threshold = dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v1));
		dispc->vid1_row_inc        = dispc_reg_in(DISPC_VID_ROW_INC(v1));
		dispc->vid1_pixel_inc      = dispc_reg_in(DISPC_VID_PIXEL_INC(v1));
		dispc->vid1_fir            = dispc_reg_in(DISPC_VID_FIR(v1));
		dispc->vid1_accu0          = dispc_reg_in(DISPC_VID_ACCU0(v1));
		dispc->vid1_accu1          = dispc_reg_in(DISPC_VID_ACCU1(v1));
		dispc->vid1_picture_size   = dispc_reg_in(DISPC_VID_PICTURE_SIZE(v1));
		dispc->vid1_fir_coef_h0    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 0));
		dispc->vid1_fir_coef_h1    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 1));
		dispc->vid1_fir_coef_h2    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 2));
		dispc->vid1_fir_coef_h3    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 3));
		dispc->vid1_fir_coef_h4    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 4));
		dispc->vid1_fir_coef_h5    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 5));
		dispc->vid1_fir_coef_h6    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 6));
		dispc->vid1_fir_coef_h7    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 7));
		dispc->vid1_fir_coef_hv0   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 0));
		dispc->vid1_fir_coef_hv1   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 1));
		dispc->vid1_fir_coef_hv2   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 2));
		dispc->vid1_fir_coef_hv3   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 3));
		dispc->vid1_fir_coef_hv4   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 4));
		dispc->vid1_fir_coef_hv5   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 5));
		dispc->vid1_fir_coef_hv6   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 6));
		dispc->vid1_fir_coef_hv7   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 7));
#ifdef CONFIG_ARCH_OMAP34XX
		dispc->vid1_fir_coef_v0    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 0));
		dispc->vid1_fir_coef_v1    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 1));
		dispc->vid1_fir_coef_v2    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 2));
		dispc->vid1_fir_coef_v3    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 3));
		dispc->vid1_fir_coef_v4    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 4));
		dispc->vid1_fir_coef_v5    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 5));
		dispc->vid1_fir_coef_v6    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 6));
		dispc->vid1_fir_coef_v7    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v1, 7));
#endif
		dispc->vid1_conv_coef0     = dispc_reg_in(DISPC_VID_CONV_COEF0(v1));
		dispc->vid1_conv_coef1     = dispc_reg_in(DISPC_VID_CONV_COEF1(v1));
		dispc->vid1_conv_coef2     = dispc_reg_in(DISPC_VID_CONV_COEF2(v1));
		dispc->vid1_conv_coef3     = dispc_reg_in(DISPC_VID_CONV_COEF3(v1));
		dispc->vid1_conv_coef4     = dispc_reg_in(DISPC_VID_CONV_COEF4(v1));
		break;

	case OMAP2_VIDEO2:
		dispc->vid2_ba0            = dispc_reg_in(DISPC_VID_BA0(v2));
		dispc->vid2_ba1            = dispc_reg_in(DISPC_VID_BA1(v2));
		dispc->vid2_position       = dispc_reg_in(DISPC_VID_POSITION(v2));
		dispc->vid2_size           = dispc_reg_in(DISPC_VID_SIZE(v2));
		dispc->vid2_attributes     = dispc_reg_in(DISPC_VID_ATTRIBUTES(v2));
		dispc->vid2_fifo_size      = dispc_reg_in(DISPC_VID_FIFO_SIZE(v2));
		dispc->vid2_fifo_threshold = dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v2));
		dispc->vid2_row_inc        = dispc_reg_in(DISPC_VID_ROW_INC(v2));
		dispc->vid2_pixel_inc      = dispc_reg_in(DISPC_VID_PIXEL_INC(v2));
		dispc->vid2_fir            = dispc_reg_in(DISPC_VID_FIR(v2));
		dispc->vid2_accu0          = dispc_reg_in(DISPC_VID_ACCU0(v2));
		dispc->vid2_accu1          = dispc_reg_in(DISPC_VID_ACCU1(v2));
		dispc->vid2_picture_size   = dispc_reg_in(DISPC_VID_PICTURE_SIZE(v2));
		dispc->vid2_fir_coef_h0    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 0));
		dispc->vid2_fir_coef_h1    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 1));
		dispc->vid2_fir_coef_h2    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 2));
		dispc->vid2_fir_coef_h3    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 3));
		dispc->vid2_fir_coef_h4    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 4));
		dispc->vid2_fir_coef_h5    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 5));
		dispc->vid2_fir_coef_h6    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 6));
		dispc->vid2_fir_coef_h7    = dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 7));
		dispc->vid2_fir_coef_hv0   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 0));
		dispc->vid2_fir_coef_hv1   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 1));
		dispc->vid2_fir_coef_hv2   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 2));
		dispc->vid2_fir_coef_hv3   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 3));
		dispc->vid2_fir_coef_hv4   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 4));
		dispc->vid2_fir_coef_hv5   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 5));
		dispc->vid2_fir_coef_hv6   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 6));
		dispc->vid2_fir_coef_hv7   = dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 7));
#ifdef CONFIG_ARCH_OMAP34XX
		dispc->vid2_fir_coef_v0    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 0));
		dispc->vid2_fir_coef_v1    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 1));
		dispc->vid2_fir_coef_v2    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 2));
		dispc->vid2_fir_coef_v3    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 3));
		dispc->vid2_fir_coef_v4    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 4));
		dispc->vid2_fir_coef_v5    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 5));
		dispc->vid2_fir_coef_v6    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 6));
		dispc->vid2_fir_coef_v7    = dispc_reg_in(DISPC_VID_FIR_COEF_V(v2, 7)); 
#endif
		dispc->vid2_conv_coef0     = dispc_reg_in(DISPC_VID_CONV_COEF0(v2));
		dispc->vid2_conv_coef1     = dispc_reg_in(DISPC_VID_CONV_COEF1(v2));
		dispc->vid2_conv_coef2     = dispc_reg_in(DISPC_VID_CONV_COEF2(v2));
		dispc->vid2_conv_coef3     = dispc_reg_in(DISPC_VID_CONV_COEF3(v2));
		dispc->vid2_conv_coef4     = dispc_reg_in(DISPC_VID_CONV_COEF4(v2));
		break;
	}
	layer[ltype].ctx_valid = 1;
}

/*  Note, that VENC registers are not restored here
 *  Note, that DISPC_CONTROL register is not restored here
 */
static void omap2_disp_restore_ctx(int ltype)
{
	int v1 = 0, v2 = 1;
	struct omap24xx_dispc_regs *dispc = &dss_ctx.dispc;

	if (layer[ltype].ctx_valid == 0)
		return;

	switch (ltype) {
	case OMAP_DSS_GENERIC:
		dss_reg_out(DSS_SYSCONFIG,   dss_ctx.sysconfig);
		dss_reg_out(DSS_CONTROL,     dss_ctx.control);
#ifdef CONFIG_ARCH_OMAP3430
		dss_reg_out(DSS_SDI_CONTROL, dss_ctx.sdi_control);
		dss_reg_out(DSS_PLL_CONTROL, dss_ctx.pll_control);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc_reg_out(DISPC_SYSCONFIG,      dispc->sysconfig);
#if 0
	/* Don't restore IRQ setting here. This can cause interrupts to fire
	 * prematurely, causing races and LCD misconfiguration.
	 */
		dispc_reg_out(DISPC_IRQENABLE,      dispc->irqenable);
#endif
		dispc_reg_out(DISPC_CONFIG,         dispc->config);
		dispc_reg_out(DISPC_DEFAULT_COLOR0, dispc->default_color0);
		dispc_reg_out(DISPC_DEFAULT_COLOR1, dispc->default_color1);
		dispc_reg_out(DISPC_TRANS_COLOR0,   dispc->trans_color0);
		dispc_reg_out(DISPC_TRANS_COLOR1,   dispc->trans_color1);
		dispc_reg_out(DISPC_LINE_NUMBER,    dispc->line_number);
		dispc_reg_out(DISPC_DATA_CYCLE1,    dispc->data_cycle1);
		dispc_reg_out(DISPC_DATA_CYCLE2,    dispc->data_cycle2);
		dispc_reg_out(DISPC_DATA_CYCLE3,    dispc->data_cycle3);
		dispc_reg_out(DISPC_TIMING_H,       dispc->timing_h);
		dispc_reg_out(DISPC_TIMING_V,       dispc->timing_v);
		dispc_reg_out(DISPC_POL_FREQ,       dispc->pol_freq);
		dispc_reg_out(DISPC_DIVISOR,        dispc->divisor);
		dispc_reg_out(DISPC_SIZE_LCD,       dispc->size_lcd);
		dispc_reg_out(DISPC_SIZE_DIG,       dispc->size_dig);
		break;

	case OMAP2_GRAPHICS:
		dispc_reg_out(DISPC_GFX_BA0,            dispc->gfx_ba0);
		dispc_reg_out(DISPC_GFX_BA1,            dispc->gfx_ba1);
		dispc_reg_out(DISPC_GFX_POSITION,       dispc->gfx_position);
		dispc_reg_out(DISPC_GFX_SIZE,           dispc->gfx_size);
		dispc_reg_out(DISPC_GFX_ATTRIBUTES,     dispc->gfx_attributes);
		dispc_reg_out(DISPC_GFX_FIFO_SIZE,      dispc->gfx_fifo_size);
		dispc_reg_out(DISPC_GFX_FIFO_THRESHOLD, dispc->gfx_fifo_threshold);
		dispc_reg_out(DISPC_GFX_ROW_INC,        dispc->gfx_row_inc);
		dispc_reg_out(DISPC_GFX_PIXEL_INC,      dispc->gfx_pixel_inc);
		dispc_reg_out(DISPC_GFX_WINDOW_SKIP,    dispc->gfx_window_skip);
		dispc_reg_out(DISPC_GFX_TABLE_BA,       dispc->gfx_table_ba);
		break;

	case OMAP2_VIDEO1:
		dispc_reg_out(DISPC_VID_BA0(v1),            dispc->vid1_ba0);
		dispc_reg_out(DISPC_VID_BA1(v1),            dispc->vid1_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v1),       dispc->vid1_position);
		dispc_reg_out(DISPC_VID_SIZE(v1),           dispc->vid1_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v1),     dispc->vid1_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v1), dispc->vid1_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v1),        dispc->vid1_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v1),      dispc->vid1_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v1),            dispc->vid1_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v1),          dispc->vid1_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v1),          dispc->vid1_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v1),   dispc->vid1_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 0), dispc->vid1_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 1), dispc->vid1_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 2), dispc->vid1_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 3), dispc->vid1_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 4), dispc->vid1_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 5), dispc->vid1_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 6), dispc->vid1_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 7), dispc->vid1_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 0), dispc->vid1_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 1), dispc->vid1_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 2), dispc->vid1_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 3), dispc->vid1_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 4), dispc->vid1_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 5), dispc->vid1_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 6), dispc->vid1_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 7), dispc->vid1_fir_coef_hv7);
#ifdef CONFIG_ARCH_OMAP34XX
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 0), dispc->vid1_fir_coef_v0);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 1), dispc->vid1_fir_coef_v1);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 2), dispc->vid1_fir_coef_v2);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 3), dispc->vid1_fir_coef_v3);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 4), dispc->vid1_fir_coef_v4);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 5), dispc->vid1_fir_coef_v5);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 6), dispc->vid1_fir_coef_v6);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v1, 7), dispc->vid1_fir_coef_v7);
#endif
		dispc_reg_out(DISPC_VID_CONV_COEF0(v1), dispc->vid1_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v1), dispc->vid1_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v1), dispc->vid1_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v1), dispc->vid1_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v1), dispc->vid1_conv_coef4);
		break;

	case OMAP2_VIDEO2:
		dispc_reg_out(DISPC_VID_BA0(v2),            dispc->vid2_ba0);
		dispc_reg_out(DISPC_VID_BA1(v2),            dispc->vid2_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v2),       dispc->vid2_position);
		dispc_reg_out(DISPC_VID_SIZE(v2),           dispc->vid2_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v2),     dispc->vid2_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v2), dispc->vid2_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v2),        dispc->vid2_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v2),      dispc->vid2_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v2),            dispc->vid2_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v2),          dispc->vid2_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v2),          dispc->vid2_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v2),   dispc->vid2_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 0), dispc->vid2_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 1), dispc->vid2_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 2), dispc->vid2_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 3), dispc->vid2_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 4), dispc->vid2_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 5), dispc->vid2_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 6), dispc->vid2_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 7), dispc->vid2_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 0), dispc->vid2_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 1), dispc->vid2_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 2), dispc->vid2_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 3), dispc->vid2_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 4), dispc->vid2_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 5), dispc->vid2_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 6), dispc->vid2_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 7), dispc->vid2_fir_coef_hv7);
#ifdef CONFIG_ARCH_OMAP34XX
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 0), dispc->vid2_fir_coef_v0);		
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 1), dispc->vid2_fir_coef_v1);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 2), dispc->vid2_fir_coef_v2);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 3), dispc->vid2_fir_coef_v3);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 4), dispc->vid2_fir_coef_v4);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 5), dispc->vid2_fir_coef_v5);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 6), dispc->vid2_fir_coef_v6);
		dispc_reg_out(DISPC_VID_FIR_COEF_V(v2, 7), dispc->vid2_fir_coef_v7);
#endif

		dispc_reg_out(DISPC_VID_CONV_COEF0(v2), dispc->vid2_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v2), dispc->vid2_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v2), dispc->vid2_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v2), dispc->vid2_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v2), dispc->vid2_conv_coef4);
		break;
	}
}

void omap2_disp_save_initstate(int ltype)
{
	unsigned long flags;

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_save_ctx(ltype);
	spin_unlock_irqrestore(&dss_lock, flags);
}

void omap2_disp_restore_initstate(int ltype)
{
	unsigned long flags;

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_restore_ctx(ltype);
	spin_unlock_irqrestore(&dss_lock, flags);
}

/*
 * Sync Lost interrupt handler
 */
static void dss_init_post_synclost(void)
{
	u32 val;

	val = dispc_reg_in(DISPC_CONFIG);
	val &= ~DISPC_CONFIG_LOADMODE_MASK;
	val |= DISPC_CONFIG_LOADMODE_FRDATLFFR;
	val |= DISPC_CONFIG_LCDALPHAENABLE;
	dispc_reg_out(DISPC_CONFIG, val);

	/* configure LCD timing */
	omap2_disp_config_lcd(1, 15, 11, 3, 3, 3, 1, 32);

	/* configure polarity */
	val = DISPC_POL_FREQ_IHS | DISPC_POL_FREQ_IVS |
	      DISPC_POL_FREQ_RF  | DISPC_POL_FREQ_ONOFF;
	dispc_reg_out(DISPC_POL_FREQ, val);

	/* set the panel size */
	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD, 320, 480);
	
	/* setup the graphics layer */
	omap2_disp_config_gfxlayer(320, 480, 32);
	dispc_reg_out(DISPC_GFX_ROW_INC,   1);
	dispc_reg_out(DISPC_GFX_PIXEL_INC, 1);

	val = DISPC_GLOBAL_ALPHA_GFX_GALPHA | DISPC_GLOBAL_ALPHA_VID2_GALPHA;
	dispc_reg_out(DISPC_GLOBAL_ALPHA, val);

	/* Setup dispc_control */
	val =	DISPC_CONTROL_GPOUT0 |
		DISPC_CONTROL_GPOUT1 |
		DISPC_CONTROL_TFTDATALINES_OALSB24B |
		DISPC_CONTROL_STNTFT |
		DISPC_CONTROL_PCKFREEENABLE |
		DISPC_CONTROL_LCDENABLEPOL_ACTIVEHIGH;
	dispc_reg_out(DISPC_CONTROL, val);
}

#define SYSCONFIG_REG32(base, offset)	__REG32(base + offset)
#define IA_DSS_L3_IA_COMPONENT_L	SYSCONFIG_REG32(0x68005400, 0x00)
#define IA_DSS_L3_IA_COMPONENT_H	SYSCONFIG_REG32(0x68005400, 0x04)
#define IA_DSS_L3_IA_CORE_L		SYSCONFIG_REG32(0x68005400, 0x18)
#define IA_DSS_L3_IA_CORE_H		SYSCONFIG_REG32(0x68005400, 0x1C)
#define IA_DSS_L3_IA_AGENT_CONTROL_L	SYSCONFIG_REG32(0x68005400, 0x20)
#define IA_DSS_L3_IA_AGENT_CONTROL_H	SYSCONFIG_REG32(0x68005400, 0x24)
#define IA_DSS_L3_IA_AGENT_STATUS_L	SYSCONFIG_REG32(0x68005400, 0x28)
#define IA_DSS_L3_IA_AGENT_STATUS_H	SYSCONFIG_REG32(0x68005400, 0x2C)
#define IA_DSS_L3_IA_LOG_L		SYSCONFIG_REG32(0x68005400, 0x58)
#define IA_DSS_L3_IA_LOG_H 		SYSCONFIG_REG32(0x68005400, 0x5C)
#define IA_DSS_L3_IA_LOG_ADDR_L		SYSCONFIG_REG32(0x68005400, 0x60)
#define IA_DSS_L3_IA_LOG_ADDR_H		SYSCONFIG_REG32(0x68005400, 0x64)

#define L3_SI_FLAG_STATUS_0_L		SYSCONFIG_REG32(0x68000500, 0x10)
#define L3_SI_FLAG_STATUS_0_H		SYSCONFIG_REG32(0x68000500, 0x14)

static void omap2_synclost_isr(void *arg, struct pt_regs *regs)
{
	int poll;
	u32 status;

	status = dispc_reg_in(DISPC_IRQSTATUS);

	if (status & DISPC_IRQSTATUS_GFXFIFOUNDERFLOW) {
		printk("##### DISPC_IRQSTATUS_GFXFIFOUNDERFLOW\n");
	}

	if (status & DISPC_IRQSTATUS_OCPERROR) {
		printk("##### OCPERROR DETECTED (%08x)\n", status);
		printk("##### IA_DSS_L3_IA_COMPONENT_H      %08x\n", IA_DSS_L3_IA_COMPONENT_H);
		printk("##### IA_DSS_L3_IA_COMPONENT_L      %08x\n", IA_DSS_L3_IA_COMPONENT_L);
		printk("##### IA_DSS_L3_IA_CORE_H           %08x\n", IA_DSS_L3_IA_CORE_H);
		printk("##### IA_DSS_L3_IA_CORE_L           %08x\n", IA_DSS_L3_IA_CORE_L);
		printk("##### IA_DSS_L3_IA_AGENT_CONTROL_H  %08x\n", IA_DSS_L3_IA_AGENT_CONTROL_H);
		printk("##### IA_DSS_L3_IA_AGENT_CONTROL_L  %08x\n", IA_DSS_L3_IA_AGENT_CONTROL_L);
		printk("##### IA_DSS_L3_IA_AGENT_STATUS_H   %08x\n", IA_DSS_L3_IA_AGENT_STATUS_H);
		printk("##### IA_DSS_L3_IA_AGENT_STATUS_L   %08x\n", IA_DSS_L3_IA_AGENT_STATUS_L);
		printk("##### IA_DSS_L3_IA_LOG_H            %08x\n", IA_DSS_L3_IA_LOG_H);
		printk("##### IA_DSS_L3_IA_LOG_L            %08x\n", IA_DSS_L3_IA_LOG_L);
		printk("##### IA_DSS_L3_IA_LOG_ADDR_H       %08x\n", IA_DSS_L3_IA_LOG_ADDR_H);
		printk("##### IA_DSS_L3_IA_LOG_ADDR_L       %08x\n", IA_DSS_L3_IA_LOG_ADDR_L);

		/* RESET L3 OCP DSS interface.
		 */
		IA_DSS_L3_IA_AGENT_CONTROL_L |=  0x01;
		IA_DSS_L3_IA_AGENT_CONTROL_L &= ~0x01;

		printk("##### L3_SI_FLAG_STATUS_0_H %08x\n", L3_SI_FLAG_STATUS_0_H);
		printk("##### L3_SI_FLAG_STATUS_0_L %08x\n", L3_SI_FLAG_STATUS_0_L);

		omap2_disp_start_gfxlayer();
	}

	if (!graphics_in_use) {
		printk(KERN_WARNING "DISPLAY: Ignoring LCD Sync Lost IRQ.\n");
		return;
	}

	printk(KERN_WARNING
		"DISPLAY: ***** Sync Lost LCD (DISPC_IRQSTATUS %x) *****\n",
							       status);
	/* Disable and Clear all the interrupts before we start
	 */
	dispc_reg_out(DISPC_IRQENABLE, 0x00000000);
	dispc_reg_out(DISPC_IRQSTATUS, 0x0000FFFF);

	if (!(status & DISPC_IRQSTATUS_SYNCLOST))
		return;

	/* Disable the display controller:
	 *  - Clear enable bits.
	 *  - Wait for FRAMEDONE interrupt.
	 */
	dispc_reg_merge(DISPC_CONTROL, 0,
			DISPC_CONTROL_DIGITALENABLE | DISPC_CONTROL_LCDENABLE);
	poll = 30000;
	while (!(dispc_reg_in(DISPC_IRQSTATUS) & DISPC_IRQSTATUS_FRAMEDONE) &&
			poll--)
		udelay(1);

	if (poll < 0)
		printk(KERN_WARNING "DISPLAY: FRAMEDONE timed out in "
				"SYNCLOST handler\n");

	/* RESET display controller.
	 */
	dispc_reg_out(DISPC_SYSCONFIG, DISPC_SYSCONFIG_SOFTRESET);
	poll = 500;
	while (!(dispc_reg_in(DISPC_SYSSTATUS) & DISPC_SYSSTATUS_RESETDONE) && poll--)
		udelay(1);

	if (poll < 0)
		printk(KERN_WARNING "DISPLAY: Failed to soft reset DSS\n");

	dss_init_post_synclost();

	/* Restore previous context.
	 */
	omap2_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_restore_ctx(OMAP2_GRAPHICS);
	omap2_disp_restore_ctx(OMAP2_VIDEO1);
	omap2_disp_restore_ctx(OMAP2_VIDEO2);

	/* Force BSOD on bad BA0 value...
	 */
	if (0 == dss_ctx.dispc.gfx_ba0) {
		printk("##### Synclost: BA0 is NULL\n");
		BUG();
	}

	omap2_disp_set_fifo_thresholds();
	omap2_disp_set_logic_dividers();

	/* Clear interrupts here again.
	 */
	dispc_reg_out(DISPC_IRQSTATUS, 0x0000FFFF);

	/* Re-enable controller.
	 */
	if (layer[OMAP_DSS_DISPC_GENERIC].ctx_valid) {
		dss_ctx.dispc.control |= DISPC_CONTROL_LCDENABLE;
		dss_ctx.dispc.control |= DISPC_CONTROL_GOLCD;
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);

		/* Restore interrupt settings. */
		dispc_reg_out(DISPC_IRQENABLE, dss_ctx.dispc.irqenable);
	}

	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);

	poll = 20000;
	while (!(dispc_reg_in(DISPC_IRQSTATUS) & DISPC_IRQSTATUS_VSYNC) && poll--)
		udelay(1);

	if (poll < 0)
		printk(KERN_WARNING "DISPLAY: Wait for VSYNC failed in %s\n",
				__FUNCTION__);
}

static inline u32 pages_per_side(u32 img_side, u32 page_exp)
{
	/*  page_side = 2 ^ page_exp
	 * (page_side - 1) is added for rounding up
	 */
	return (u32) (img_side + (1 << page_exp) - 1) >> page_exp;
}

static int get_vrfb_offset(u32 img_len, u32 bytes_per_pixel, int side)
{
	int page_width_exp, page_height_exp, pixel_size_exp, offset = 0;

	/* Maximum supported is 4 bytes (RGB32) */
	if (bytes_per_pixel > 4)
		return -EINVAL;

	page_width_exp = PAGE_WIDTH_EXP;
	page_height_exp = PAGE_HEIGHT_EXP;
	pixel_size_exp = bytes_per_pixel >> 1;

	if (side == SIDE_W) {
		offset = ((1 << page_width_exp) *
			  (pages_per_side
			   (img_len * bytes_per_pixel, page_width_exp))
		    ) >> pixel_size_exp;	/* in pixels */
	} else {
		offset = (1 << page_height_exp) *
		    (pages_per_side(img_len, page_height_exp));
	}

	return (offset);
}

/* Flip the video overlay framebuffer.  The video overlay window may initially
 * be either enabled or disabled.  The overlay window will be enabled by this
 * routine.  fb_base_phys is the physical base address of the framebuffer for
 * the video overlay.  The address programmed into the base address register of
 * the video overlay window is calculated based on the cropped size and the full
 * size of the overlay framebuffer.
 */
void
omap2_disp_start_vlayer(int ltype, struct v4l2_pix_format *pix,
			struct v4l2_rect *crop, struct v4l2_window *win,
			unsigned long fb_base_phys, int rotation_deg,
			int mirroring)
{
	unsigned long cropped_base_phys;
	int v, ps = 2, temp_ps = 2, vr_ps = 1;
	int offset = 0, ctop = 0, cleft = 0, line_length = 0;
	int overlay_opt_delta, overlay_opt_enable = 0;
	int gfx_posX, gfx_posY, vid_posX, vid_posY;
	int gfx_sizeX, gfx_sizeY, vid_sizeX, vid_sizeY;
	int screen_width, screen_height;
	int bytesperpixel;
	int X, Y, skip_val;
	int gfx_pix_inc, gfx_row_inc;
	u32 gfx_format;
	int flicker_filter = 0;

	gfx_format = dispc_reg_in(DISPC_GFX_ATTRIBUTES);

	if ((omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV) &&
	    ((win->w.width == crop->width) && (win->w.height == crop->height)))
		flicker_filter = 1;

	switch (gfx_format & DISPC_GFX_ATTRIBUTES_GFXFORMAT) {
	case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP1:
	case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP2:
	case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP4:
	case DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP8:
		bytesperpixel = 1;
		break;
	case DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB12:
	case DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB16:
		bytesperpixel = 2;
		break;
	case DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB24:
	default:
		bytesperpixel = 4;
		break;
	}

	if (ltype == OMAP2_VIDEO1)
		v = 0;
	else if (ltype == OMAP2_VIDEO2)
		v = 1;
	else
		return;

	/*
	 * If pixel format is YUV then PS = 4, for RGB16 PS = 2 RGB24 Unpack PS =4
	 */

	if (V4L2_PIX_FMT_YUYV == pix->pixelformat ||
	    V4L2_PIX_FMT_UYVY == pix->pixelformat) {
		if (rotation_deg >= 0 || mirroring == 1) {
			/*
			 * ps    - Actual pixel size for YUYV/UYVY for VRFB/Mirroring is 4 bytes
			 * vr_ps - Virtually pixel size for YUYV/UYVY  is 2 bytes
			 */
			ps = 4;
			vr_ps = 2;
		} else
			ps = 2;	/* otherwise the pixel size is 2 byte */
	} else if (V4L2_PIX_FMT_RGB32 == pix->pixelformat)
		ps = 4;
	else if (V4L2_PIX_FMT_RGB24 == pix->pixelformat)
		ps = 3;

	/*
	 * If rotation is selected then compute the rotation parameters
	 */
	if (rotation_deg >= 0) {
		line_length = MAX_PIXELS_PER_LINE;
		ctop = (pix->height - crop->height) - crop->top;
		cleft = (pix->width - crop->width) - crop->left;
	} else {
		line_length = pix->width;
	}
	switch (rotation_deg) {
	case 90:
		offset =
		    (get_vrfb_offset(pix->width, ps, SIDE_H) -
		     (pix->width / vr_ps)) * ps * line_length;
		temp_ps = ps / vr_ps;
		if (mirroring == 0) {
			cropped_base_phys = fb_base_phys + offset +
			    line_length * temp_ps * cleft + crop->top * temp_ps;
		} else {
			cropped_base_phys =
			    fb_base_phys + offset +
			    line_length * temp_ps * cleft +
			    crop->top * temp_ps +
			    (line_length * ((crop->width / (vr_ps)) - 1) * ps);
		}
		break;

	case 180:
		offset =
		    (get_vrfb_offset(pix->height, ps, SIDE_H) -
		     pix->height) * ps * line_length +
		    (get_vrfb_offset(pix->width, ps, SIDE_W) -
		     (pix->width / vr_ps)) * ps;
		if (mirroring == 0) {
			cropped_base_phys = fb_base_phys + offset +
			    (line_length * ps * ctop) + (cleft / vr_ps) * ps;
		} else {
			cropped_base_phys =
			    fb_base_phys + offset + (line_length * ps * ctop) +
			    (cleft / vr_ps) * ps +
			    (line_length * (crop->height - 1) * ps);
		}
		break;

	case 270:
		offset =
		    (get_vrfb_offset(pix->height, ps, SIDE_W) -
		     pix->height) * ps;
		temp_ps = ps / vr_ps;
		if (mirroring == 0) {
			cropped_base_phys = fb_base_phys + offset +
			    line_length * temp_ps * crop->left + ctop * ps;
		} else {
			cropped_base_phys =
			    fb_base_phys + offset +
			    line_length * temp_ps * crop->left + ctop * ps +
			    (line_length * ((crop->width / vr_ps) - 1) * ps);
		}
		break;

	case 0:
		if (mirroring == 0) {
			cropped_base_phys = fb_base_phys
			    + (line_length * ps) * crop->top +
			    (crop->left / vr_ps) * ps;
		} else {
			cropped_base_phys =
			    fb_base_phys + (line_length * ps) * crop->top +
			    (crop->left / vr_ps) * ps +
			    (line_length * (crop->height - 1) * ps);
		}
		break;

	default:
		if (mirroring == 0) {
			cropped_base_phys = fb_base_phys
			    + line_length * ps * crop->top + crop->left * ps;
		} else {
			cropped_base_phys = fb_base_phys
			    + (line_length * ps * crop->top) / vr_ps +
			    (crop->left * ps) / vr_ps +
			    ((crop->width / vr_ps) - 1) * ps;
		}
		break;
	}

	/*
	 * We store the information in the layer structure for : If user
	 * dynamically switches the pipeline from LCD to TV or vice versa
	 * we should have the necessary configurations for the output device
	 * (LCD/TV)
	 */

	/*
	 * For LCD BA0 and BA1 are same
	 */
	layer[ltype].dma[0].ba0 = cropped_base_phys;
	layer[ltype].dma[0].ba1 = cropped_base_phys;

	/*
	 * Store BA0 BA1 for TV, BA1 points to the alternate row
	 */
	layer[ltype].dma[1].ba0 = cropped_base_phys;

	if (flicker_filter == 1) {
		layer[ltype].dma[1].ba1 = cropped_base_phys;
	} else if (rotation_deg >= 0) {
		if (mirroring == 1)
			layer[ltype].dma[1].ba1 =
			    cropped_base_phys - line_length * ps;
		else
			layer[ltype].dma[1].ba1 =
			    cropped_base_phys + line_length * ps;
	} else {
		if (mirroring == 1)
			layer[ltype].dma[1].ba1 =
			    cropped_base_phys + line_length * ps / vr_ps;
		else
			layer[ltype].dma[1].ba1 =
			    cropped_base_phys + line_length * ps;
	}

	/* If output path is set to TV */
	if (omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV) {
		dispc_reg_out(DISPC_VID_BA0(v), cropped_base_phys);
		if (flicker_filter == 1) {
			dispc_reg_out(DISPC_VID_BA1(v), cropped_base_phys);
		} else {
			if (rotation_deg >= 0) {
				if (mirroring == 1)
					dispc_reg_out(DISPC_VID_BA1(v),
						      cropped_base_phys -
						      line_length * ps);
				else
					dispc_reg_out(DISPC_VID_BA1(v),
						      cropped_base_phys +
						      line_length * ps);
			} else {
				if (mirroring == 1)
					dispc_reg_out(DISPC_VID_BA1(v),
						      cropped_base_phys +
						      line_length * ps / vr_ps);
				else
					dispc_reg_out(DISPC_VID_BA1(v),
						      cropped_base_phys +
						      line_length * ps);
			}
		}

		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
				DISPC_VID_ATTRIBUTES_ENABLE,
				DISPC_VID_ATTRIBUTES_ENABLE);
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);

		/* if not enabled transparency and alpha blending enable overlay optimization */
		if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
			if (!
			    (dispc_reg_in(DISPC_CONFIG) &
			     DISPC_CONFIG_TCKDIGENABLE))
				overlay_opt_enable = 1;
		} else {
			if ((!
			     (dispc_reg_in(DISPC_CONFIG) &
			      DISPC_CONFIG_TCKDIGENABLE))
			    &&
			    (!(dispc_reg_in(DISPC_CONFIG) &
			       DISPC_CONFIG_TVALPHAENABLE)))
				overlay_opt_enable = 1;
		}
	}
	/* If output path is set to LCD */
	else {
		dispc_reg_out(DISPC_VID_BA0(v), cropped_base_phys);
		dispc_reg_out(DISPC_VID_BA1(v), cropped_base_phys);
		dispc_reg_merge(DISPC_VID_ATTRIBUTES(v),
				DISPC_VID_ATTRIBUTES_ENABLE,
				DISPC_VID_ATTRIBUTES_ENABLE);
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
				DISPC_CONTROL_GOLCD);

		/* if not enabled transparency enable overlay optimization */
		if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
			if (!
			    (dispc_reg_in(DISPC_CONFIG) &
			     DISPC_CONFIG_TCKLCDENABLE))
				overlay_opt_enable = 1;
		} else {
			if ((!
			     (dispc_reg_in(DISPC_CONFIG) &
			      DISPC_CONFIG_TCKLCDENABLE))
			    &&
			    (!(dispc_reg_in(DISPC_CONFIG) &
			       DISPC_CONFIG_LCDALPHAENABLE)))
				overlay_opt_enable = 1;
		}
	}

	/** if GFX pipeline enabled and no transparency enabled
	  * check for overlapping of GFX and Video pipelines to
	  * enable overlay optimization
	  */
	if ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) & DISPC_GFX_ATTRIBUTES_ENABLE)
	    && overlay_opt_enable && (ltype == OMAP2_VIDEO1)) {

		gfx_posX =
		    (dispc_reg_in(DISPC_GFX_POSITION) &
		     DISPC_GFX_POSITION_GFXPOSX) >>
		    DISPC_GFX_POSITION_GFXPOSX_SHIFT;
		gfx_posY =
		    (dispc_reg_in(DISPC_GFX_POSITION) &
		     DISPC_GFX_POSITION_GFXPOSY) >>
		    DISPC_GFX_POSITION_GFXPOSY_SHIFT;
		gfx_sizeX =
		    (dispc_reg_in(DISPC_GFX_SIZE) & DISPC_GFX_SIZE_GFXSIZEX) >>
		    DISPC_GFX_SIZE_GFXSIZEX_SHIFT;
		gfx_sizeY =
		    (dispc_reg_in(DISPC_GFX_SIZE) & DISPC_GFX_SIZE_GFXSIZEY) >>
		    DISPC_GFX_SIZE_GFXSIZEY_SHIFT;
		vid_posX =
		    (dispc_reg_in(DISPC_VID_POSITION(v)) &
		     DISPC_VID_POSITION_VIDPOSX) >>
		    DISPC_VID_POSITION_VIDPOSX_SHIFT;
		vid_posY =
		    (dispc_reg_in(DISPC_VID_POSITION(v)) &
		     DISPC_VID_POSITION_VIDPOSY) >>
		    DISPC_VID_POSITION_VIDPOSY_SHIFT;
		vid_sizeX =
		    ((dispc_reg_in(DISPC_VID_SIZE(v)) & DISPC_VID_SIZE_VIDSIZEX)
		     >> DISPC_VID_SIZE_VIDSIZEX_SHIFT) + 1;
		vid_sizeY =
		    ((dispc_reg_in(DISPC_VID_SIZE(v)) & DISPC_VID_SIZE_VIDSIZEY)
		     >> DISPC_VID_SIZE_VIDSIZEY_SHIFT) + 1;
		gfx_pix_inc = dispc_reg_in(DISPC_GFX_PIXEL_INC);
		gfx_row_inc = dispc_reg_in(DISPC_GFX_ROW_INC);

		omap2_disp_get_panel_size(omap2_disp_get_output_dev(ltype),
					  &screen_width, &screen_height);

		if (((gfx_posX + gfx_sizeX) <= vid_posX)
		    || ((gfx_posY + gfx_sizeY) <= vid_posY)) {
			overlay_opt_enable = 0;
		}
	} else {
		overlay_opt_enable = 0;
	}

	/* Enable overlay optimization */
	if (overlay_opt_enable) {
		X = vid_sizeX;

		if ((vid_posX == 0) && (gfx_sizeX <= (vid_posX + vid_sizeX))) {
			X = gfx_sizeX + 1;
			Y = vid_sizeY + 1;
			skip_val =
			    Y * ((X - 1) * (gfx_pix_inc - 1 + bytesperpixel) +
				 (gfx_row_inc - 1 + bytesperpixel));
			goto skip;
		}

		if ((vid_posX + vid_sizeX) >= gfx_sizeX) {
			X = gfx_sizeX - vid_posX + 1;
		}

		if ((vid_posX == 0) || (gfx_sizeX <= (vid_posX + vid_sizeX))) {
			overlay_opt_delta = 0;
		} else {
			overlay_opt_delta = 1;
		}
		skip_val =
		    (X * (gfx_pix_inc + bytesperpixel - 1) + overlay_opt_delta);

skip:
		dispc_reg_out(DISPC_GFX_WINDOW_SKIP,
			      skip_val);
		dispc_reg_merge(DISPC_CONTROL,
				DISPC_CONTROL_OVERLAYOPTIMIZATION,
				DISPC_CONTROL_OVERLAYOPTIMIZATION);
	}
}

/* Configure VIDEO1 or VIDEO2 layer parameters*/
void
omap2_disp_config_vlayer(int ltype, struct v4l2_pix_format *pix,
			 struct v4l2_rect *crop, struct v4l2_window *win,
			 int rotation_deg, int mirroring)
{
	int vid_position_x, vid_position_y, ps = 2, vr_ps = 1;
	unsigned long vid_position, vid_size, vid_picture_size;
	unsigned long vid_attributes;
	unsigned long firvinc, firhinc;
	int winheight, winwidth, cropheight, cropwidth, pixheight, pixwidth;
	int cleft, ctop;
	int panelwidth, panelheight, row_inc_value = 0, pixel_inc_value = 0;
	int flicker_filter = 0;
	int ratio = 0;
	u32 rot_attr;


	/* vertical resizing matrix */
	// bi-linear algorithm
	static const char vc_u[3][8] = {
		{   0,  16, 32, 48,  0,   0,  0,   0 },
		{ 128, 112, 96, 80, 64,  80, 96, 112 },
		{   0,   0,  0,  0, 64,  48, 32,  16 }
	};

/*
	// TI's original algorithm
	{ {   0,   3,  12,  32,   0,   7,   5,   2 },
	  {128, 123, 111, 89, 64, 89, 111, 123},
	  {   0,   2,   5,   7,  64,  32,  12,   3 } };
*/

	/* horizontal resizing matrix */
	// bi-linear algorithm
	static const char hc_u[5][8] = {
		{ 0,    0,   0,   0,   0,  0,  0,   0 },
		{ 0,   16,  32,  48,   0,  0,  0,   0 },
		{ 128, 112, 96,  80,  64, 80, 96, 112 },
		{ 0,     0,  0,   0,  64, 48, 32,  16 },
		{ 0,    0,   0,   0,   0,  0,  0,   0 }
	};

/*
	// TI's original algorithm
	{ {   0,  -1,  -2,  -5,   0,  -2,  -1,   0 },
	  {0, 13, 30, 51, -9, -11, -11, -8},
	  {128, 124, 112, 95, 73, 95, 112, 124},
	  {0, -8, -11, -11, 73, 51, 30, 13},
	  {   0,   0,  -1,  -2,  -9,  -5,  -2,  -1 } };
*/

	static const char vc_d[3][8] = {
		{ 36,  40, 45, 50,  18,  23,  27,  31 },
		{ 56,  57, 56, 55,  55,  55,  56,  57 },
		{ 36,  31, 27, 23,  55,  50,  45,  40 }
	};

	static const char vc_d_fivetap[5][8] = {
		{0,   4,  8, 12, -9, -7, -5, -2 },  //VC22
		{36, 40, 44,  48, 17, 22, 27, 31},  //VC2
		{56, 55, 54,  53, 52, 53, 54, 55},  //VC1
		{36, 31, 27,  22, 51, 48, 44, 40},  //VC0
		{0,  -2, -5,  -7, 17, 12,  8,  4}   //VC00
	};

	/* horizontal resizing matrix */
	static const char hc_d[5][8] = {
		{   0,   4, 8,  12, -9, -7, -5, -2 },
		{   36, 40, 44, 48, 17, 22, 27, 31 },
		{   56, 55, 54, 53, 52, 53, 54, 55 },
		{   36, 31, 27, 22, 51, 48, 44, 40 },
		{   0,  -2, -5, -7, 17, 12, 8,  4 }
	};

	const char (*vc)[8] = vc_u;
	const char (*hc)[8] = hc_u;

	int v;

	if (ltype == OMAP2_VIDEO1)
		v = 0;
	else if (ltype == OMAP2_VIDEO2)
		v = 1;
	else {
		printk(KERN_WARNING "DISPLAY: Invalid video layer type "
			"%d in %s\n", ltype, __FUNCTION__);
		return;
	}

	if ((omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV) &&
	    ((win->w.width == crop->width) && (win->w.height == crop->height)))
		flicker_filter = 1;

	/* make sure the video overlay is disabled before we reconfigure it */
	omap2_disp_disable_layer(ltype);

	/* configure the video attributes register */
	vid_attributes = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		if (pix->pixelformat == V4L2_PIX_FMT_YUYV) {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_YUV2;
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDCOLORCONVENABLE;
		} else {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_UYVY;
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDCOLORCONVENABLE;
		}

		if (mirroring == 1 || rotation_deg >= 0) {
			/*
			 * ps      - In VRFB space the pixel size for YUYV/UYVY is 4 bytes
			 * vr_ps - Actual pixel size for YUYV/UYVY  is 2 bytes
			 */
			ps = 4;
			vr_ps = 2;
		}

		rot_attr = (((360 - rotation_deg) / 90) % 4);

		if (mirroring == 1) {
			rot_attr ^= !(rot_attr & 1) << 1;
		}

		if ((mirroring == 1) && (rotation_deg == -1))
			rot_attr = 2;

		vid_attributes |= rot_attr << DISPC_VID_ATTRIBUTES_VIDROT;

		/* If the rotation is 90 or 270 degrees we also need to set the
		 * DISPC_VID_ATTRIBUTES_VIDROWREPEAT bit.
		 */
		vid_attributes |=
			(rot_attr & 1) << DISPC_VID_ATTRIBUTES_VIDROWREPEAT;
		break;

	case V4L2_PIX_FMT_RGB24:
		ps = 3;		/* pixel size is 3 bytes */
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24P;
		vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) &
				    DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE)
				   << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		break;

		/* The picture format is a bit confusing in V4L2.. as per the V4L2 spec
		 * RGB32 and BGR32 are always with alpha bits enabled.. (i.e always in
		 * packed mode) */
	case V4L2_PIX_FMT_RGB32:
		ps = 4;		/* pixel size is 4 bytes */
		if ((is_sil_rev_less_than(OMAP3430_REV_ES2_0))
		    || (ltype == OMAP2_VIDEO1)) {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24;
			vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) &
					    DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE)
					   << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		} else {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_ARGB32;
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDENDIANNESS;
		}
		break;
	case V4L2_PIX_FMT_BGR32:
		ps = 4;		/* pixel size is 4 bytes */
		if ((is_sil_rev_less_than(OMAP3430_REV_ES2_0))
		    || (ltype == OMAP2_VIDEO1)) {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB24;
			vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) &
					    DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE)
					   << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		} else {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_ARGB32;
		}
		break;
	case V4L2_PIX_FMT_RGB565:
	default:
		ps = 2;		/* pixel size is 2 bytes */
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB16;
		vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) &
				    DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE)
				   << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		break;
	case V4L2_PIX_FMT_RGB565X:
		ps = 2;		/* pixel size is 2 bytes */
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_RGB16;
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDENDIANNESS;
		vid_attributes |= ((dispc_reg_in(DISPC_GFX_ATTRIBUTES) &
				    DISPC_GFX_ATTRIBUTES_GFXREPLICATIONENABLE)
				   << DISPC_GFX_ATTRIBUTES_GFXREPEN);
		break;
	}

	if (omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV)
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDCHANNELOUT;

	/* Enable 16 x 32 burst size */
	vid_attributes |= DISPC_VID_ATTRIBUTES_VIDBURSTSIZE_BURST16X32;

	omap2_disp_set_fifo_thresholds();

	if (rotation_deg == 90 || rotation_deg == 270) {
		winheight  = win->w.width;
		winwidth   = win->w.height;
		cropheight = crop->width;
		cropwidth  = crop->height;
		pixheight  = pix->width;
		pixwidth   = pix->height;
		cleft      = crop->top;
		ctop       = crop->left;
	} else {
		winwidth   = win->w.width;
		winheight  = win->w.height;
		cropwidth  = crop->width;
		cropheight = crop->height;
		pixheight  = pix->height;
		pixwidth   = pix->width;
		ctop       = crop->top;
		cleft      = crop->left;
	}

#ifdef DMA_DECIMATE
	printk("winwidth = %d \n", win->w.width);
	printk("winheight = %d \n", win->w.height);
	printk("pixwidth = %d \n", pix->width);
	printk("pixheigth = %d \n", pix->height);
	printk("cropwidth = %d \n", crop->width);
	printk("cropheigth = %d \n", crop->height);
	printk("croptop = %d \n", crop->top);
	printk("cropleft = %d \n", crop->left);
#endif

	if (winheight != 0) {
		// ratio = cropheight / winheight;
		if (winheight < cropheight) {

			if ((winheight *2) <= cropheight && (cropwidth < 1024) ) {
				ratio = 3;
			}
			else
				ratio = 2;
		}
	}

	if (winwidth != cropwidth) {
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_HRESIZE;
		if (winwidth < cropwidth){
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDHRESIZECONF;
			hc = hc_d;
		}
	}
	if (winheight != cropheight) {
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_VRESIZE;
		if (winheight < cropheight){
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVRESIZECONF;
			if (ratio <= 2){
				vc = vc_d;
			} else {
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS;
				vc = vc_d_fivetap;
			}
		}
	}

	if (flicker_filter == 1) {
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_VRESIZE;
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVRESIZECONF;
		if (ratio <= 2){
				vc = vc_d;
		} else {
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS;
				vc = vc_d_fivetap;
		}
	}

	dispc_reg_out(DISPC_VID_ATTRIBUTES(v), vid_attributes);

	/* Set the color converion parameters - note: do this AFTER we set
	 * DISPC_VID_ATTRIBUTES so we don't overwrite the full range bit
	 */
	set_colorconv(v, pix->colorspace);

	/* initialize the resizing filter */
	omap2_disp_set_resize(v, vc, hc, ratio);

	dispc_reg_out(DISPC_VID_ACCU0(v), 0);
	if (flicker_filter == 1)
		dispc_reg_out(DISPC_VID_ACCU1(v), 0x01000000);
	else
		dispc_reg_out(DISPC_VID_ACCU1(v), 0);

	firhinc = (1024 * (cropwidth - 1)) / (winwidth - 1);
	if (firhinc < 1)
		firhinc = 1;
	else if (firhinc > 4095)
		firhinc = 4095;
	firvinc = (1024 * (cropheight - 1)) / (winheight - 1);

	if (firvinc < 1)
		firvinc = 1;
	else if (firvinc > 4095)
		firvinc = 4095;

	if (flicker_filter == 0)
		dispc_reg_out(DISPC_VID_FIR(v), firhinc | (firvinc << 16));
	else
		dispc_reg_out(DISPC_VID_FIR(v), 0x08000000);

	omap2_disp_get_panel_size(omap2_disp_get_output_dev(ltype), &panelwidth,
				  &panelheight);

	/* configure the target window on the display */
	switch (rotation_deg) {

	case 90:
		vid_position_y = (panelheight - win->w.width) - win->w.left;
		vid_position_x = win->w.top;
		break;

	case 180:
		vid_position_x = (panelwidth - win->w.width) - win->w.left;
		vid_position_y = (panelheight - win->w.height) - win->w.top;
		break;

	case 270:
		vid_position_y = win->w.left;
		vid_position_x = (panelwidth - win->w.height) - win->w.top;
		break;

	default:
		vid_position_x = win->w.left;
		vid_position_y = win->w.top;
		break;
	}

	if (omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV)
		vid_position_y = vid_position_y / 2;

	/*
	 *      If Scaling is enabled for TV then the window height should be divided by two
	 */

	if (((omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV) &&
	     (winheight != cropheight)) || flicker_filter) {
		vid_size = (((winwidth - 1) << DISPC_VID_SIZE_VIDSIZEX_SHIFT)
			    & DISPC_VID_SIZE_VIDSIZEX)
		    | ((((winheight - 1) / 2) << DISPC_VID_SIZE_VIDSIZEY_SHIFT)
		       & DISPC_VID_SIZE_VIDSIZEY);
	} else {
		vid_size = (((winwidth - 1) << DISPC_VID_SIZE_VIDSIZEX_SHIFT)
			    & DISPC_VID_SIZE_VIDSIZEX)
		    | (((winheight - 1) << DISPC_VID_SIZE_VIDSIZEY_SHIFT)
		       & DISPC_VID_SIZE_VIDSIZEY);
#ifdef DMA_DECIMATE
		vid_size =
		    (((winwidth - 1) / 2 << DISPC_VID_SIZE_VIDSIZEX_SHIFT)
		     & DISPC_VID_SIZE_VIDSIZEX)
		    | ((((winheight - 1) / 2) << DISPC_VID_SIZE_VIDSIZEY_SHIFT)
		       & DISPC_VID_SIZE_VIDSIZEY);
#endif
	}

	/* configure the source window in the framebuffer */
	if (flicker_filter == 1) {
		vid_picture_size =
		    (((cropwidth -
		       1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
		     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) | (((cropheight - 1)
							       <<
							       DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
							      &
							      DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
	} else if ((omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV)
		   && (flicker_filter == 0)) {
		vid_picture_size =
		    (((cropwidth -
		       1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
		     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
		    (((cropheight / 2 -
		       1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
		     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
	} else {
		vid_picture_size =
		    (((cropwidth -
		       1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
		     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
		    (((cropheight -
		       1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
		     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
#ifdef DMA_DECIMATE
		vid_picture_size =
		    (((cropwidth / 2 -
		       1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
		     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
		    (((cropheight / 2 -
		       1) << DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
		     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
#endif
	}

	switch (mirroring) {
	case 0:		/* No mirroring */
		if (rotation_deg == 90 || rotation_deg == 270) {
			row_inc_value =
			    1 + (MAX_PIXELS_PER_LINE - pixwidth +
				 (pixwidth - cropwidth - cleft) + cleft) * ps;

		} else if (rotation_deg == 180 || rotation_deg == 0) {
			if (V4L2_PIX_FMT_YUYV == pix->pixelformat
			    || V4L2_PIX_FMT_UYVY == pix->pixelformat)
				row_inc_value =
				    1 + (MAX_PIXELS_PER_LINE -
					 (pixwidth / vr_ps) +
					 ((pixwidth - cropwidth -
					   cleft) / vr_ps)
					 + (cleft / vr_ps)) * ps;

			else
				row_inc_value =
				    1 + (MAX_PIXELS_PER_LINE - pixwidth +
					 (pixwidth -
					  cropwidth - cleft) + cleft) * ps;
#ifdef DMA_DECIMATE
			row_inc_value =
			    row_inc_value + (MAX_PIXELS_PER_LINE * ps);
#endif
		} else {
			row_inc_value = 1 + (pix->width * ps) - cropwidth * ps;
#ifdef DMA_DECIMATE
			row_inc_value = row_inc_value + ((pix->width + 1) * ps);
#endif
		}
		pixel_inc_value = 1;
#ifdef DMA_DECIMATE
		pixel_inc_value = 1 + (1 * ps);
#endif
		break;

	case 1:		/* Mirroring */
		if (rotation_deg == 90 || rotation_deg == 270) {
			row_inc_value =
			    (-(MAX_PIXELS_PER_LINE + cropwidth) * ps) + 1;
			pixel_inc_value = 1;
		} else if (rotation_deg == 180 || rotation_deg == 0) {
			row_inc_value =
			    (-(MAX_PIXELS_PER_LINE + (cropwidth / vr_ps))
			     * ps) + 1;
			pixel_inc_value = 1;
		} else {
			row_inc_value =
			    2 * ((cropwidth / vr_ps) -
				 1) * ps + 1 +
			    ((pix->width * ps) / vr_ps) -
			    (cropwidth / vr_ps) * ps;
			pixel_inc_value = (-2 * ps) + 1;
		}
		break;
	}			/* Mirroring Switch */

	/*
	 * For LCD row inc and pixel inc
	 */

	layer[ltype].dma[0].row_inc = row_inc_value;
	layer[ltype].dma[0].pix_inc = pixel_inc_value;

	if (omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_LCD
	    || flicker_filter == 1) {
		dispc_reg_out(DISPC_VID_ROW_INC(v), row_inc_value);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v), pixel_inc_value);
	}

	/*
	 * For TV the row increment should be done twice as the
	 * TV operates in interlaced mode
	 */

	else {
		if (rotation_deg >= 0) {
			if (mirroring == 1)
				row_inc_value = row_inc_value -
				    MAX_PIXELS_PER_LINE * ps;
			else
				row_inc_value =
				    row_inc_value + MAX_PIXELS_PER_LINE * ps;
		} else {
			if (mirroring == 1)
				row_inc_value = row_inc_value +
				    pix->width * ps / vr_ps;
			else
				row_inc_value = row_inc_value + pix->width * ps;
		}
		dispc_reg_out(DISPC_VID_ROW_INC(v), row_inc_value);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v), pixel_inc_value);
	}

	/*
	 * Store BA0 BA1 for TV, BA1 points to the alternate row
	 */
	if (flicker_filter == 1) {
		layer[ltype].dma[1].row_inc = row_inc_value;
	} else if (rotation_deg >= 0) {
		if (mirroring == 1)
			layer[ltype].dma[1].row_inc = row_inc_value -
			    MAX_PIXELS_PER_LINE * ps;
		else
			layer[ltype].dma[1].row_inc = row_inc_value +
			    MAX_PIXELS_PER_LINE * ps;
	} else {
		if (mirroring == 1)
			layer[ltype].dma[1].row_inc =
			    row_inc_value + pix->width * ps / vr_ps;
		else
			row_inc_value = row_inc_value + pix->width * ps;

	}
	layer[ltype].dma[1].pix_inc = pixel_inc_value;
	layer[ltype].size_x = cropwidth;
	layer[ltype].size_y = cropheight;

	if ((omap2_disp_get_output_dev(ltype) == OMAP2_OUTPUT_TV)) {
		vid_position_x += 44;
		vid_position_y += 14;
	}

	vid_position = ((vid_position_x << DISPC_VID_POSITION_VIDPOSX_SHIFT)
			& DISPC_VID_POSITION_VIDPOSX)
	    | ((vid_position_y << DISPC_VID_POSITION_VIDPOSY_SHIFT)
	       & DISPC_VID_POSITION_VIDPOSY);

	dispc_reg_out(DISPC_VID_POSITION(v), vid_position);
	dispc_reg_out(DISPC_VID_SIZE(v), vid_size);
	dispc_reg_out(DISPC_VID_PICTURE_SIZE(v), vid_picture_size);
	omap2_disp_save_initstate(ltype);

}

/*---------------------------------------------------------------------------*/
/* Many display controller registers are shadowed. Setting the GO bit causes
 * changes to these registers to take effect in hardware.
 */
void omap2_disp_reg_sync(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
				DISPC_CONTROL_GOLCD);
	else
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GODIGITAL,
				DISPC_CONTROL_GODIGITAL);
}


/* This function provides the status of the GO bit. After the GO bit is set
 * through software, register changes take affect at the next VFP (vertical
 * front porch) or EVSYNC. Per the specs, no further register changes
 * must be done until the GO bit is reset by hardware. This function allows
 * drivers to poll the status of the GO bit, and wait until it is reset if they
 * wish to.
 */
int omap2_disp_reg_sync_done(int output_dev)
{
	u32 control = dispc_reg_in(DISPC_CONTROL);

	if (output_dev == OMAP2_OUTPUT_LCD)
		return ~(control & DISPC_CONTROL_GOLCD);
	else
		return ~(control & DISPC_CONTROL_GODIGITAL);
}

/*
 * Disable the display controller. May be called in interrupt or process
 * context. However, this function should not be called with interrupts
 * disabled as jiffies will not increment.
 */
void omap2_disp_disable(unsigned long timeout_ticks)
{
	unsigned long timeout;

	if (dispc_reg_in(DISPC_CONTROL)
	    & (DISPC_CONTROL_DIGITALENABLE | DISPC_CONTROL_LCDENABLE)) {
		/* disable the display controller */
		dispc_reg_merge(DISPC_CONTROL, 0,
				DISPC_CONTROL_DIGITALENABLE |
				DISPC_CONTROL_LCDENABLE);

		/* wait for any frame in progress to complete */
		dispc_reg_out(DISPC_IRQSTATUS, DISPC_IRQSTATUS_FRAMEDONE);
		timeout = jiffies + timeout_ticks;
		while (!(dispc_reg_in(DISPC_IRQSTATUS)
			 & DISPC_IRQSTATUS_FRAMEDONE)
		       && time_before(jiffies, timeout)) {
			if (!in_atomic()) {
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(1);
			} else
				udelay(100);
		}
		if (!(dispc_reg_in(DISPC_IRQSTATUS)
		      & DISPC_IRQSTATUS_FRAMEDONE)) {
			printk(KERN_WARNING "timeout waiting for "
			       "frame-done interrupt\n");
		}
#ifndef CONFIG_ARCH_OMAP3410
#ifdef CONFIG_ARCH_OMAP3430
		omap2_tv_clock_disable();
#endif
#endif
	}

	return;
}

void omap2_disp_set_gfx_palette(u32 palette_ba)
{
	dispc_reg_out(DISPC_GFX_TABLE_BA, palette_ba);
	dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_LOADMODE_PGTABUSETB,
			DISPC_CONFIG_LOADMODE_PGTABUSETB);
}

/* Configure Graphics layer parameters */
void omap2_disp_config_gfxlayer(u32 size_x, u32 size_y, int color_depth)
{
	u32 config;
	u32 gfx_attributes, gfx_format = 0;
	u32 gfx_position, gfx_window_skip, gfx_size = 0;

	config = dispc_reg_in(DISPC_CONFIG);

	config |= (DISPC_CONFIG_LOADMODE_PGTABUSETB |
		   DISPC_CONFIG_LOADMODE_FRDATLEFR);

	/* This driver doesn't currently support the video windows, so
	 * we force the palette/gamma table to be a palette table and
	 * force both video windows to be disabled.
	 */
	config &= ~DISPC_CONFIG_PALETTEGAMMATABLE;

	gfx_attributes = DISPC_GFX_ATTRIBUTES_GFXBURSTSIZE_BURST16X32;

	/* enable the graphics window only if its size is not zero */
	if (size_x > 0 && size_y > 0) {
		gfx_attributes |= DISPC_GFX_ATTRIBUTES_ENABLE;

		gfx_size = ((size_x - 1) << DISPC_GFX_SIZE_GFXSIZEX_SHIFT) |
			((size_y - 1) << DISPC_GFX_SIZE_GFXSIZEY_SHIFT);
	}

	omap2_disp_set_fifo_thresholds();

	gfx_position = 0;
	gfx_window_skip = 0;

	switch (color_depth) {
	case 1:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP1;
		break;
	case 2:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP2;
		break;
	case 4:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP4;
		break;
	case 8:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_BITMAP8;
		break;
	case 12:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB12;
		break;
	case 16:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB16;
		break;
	case 24:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_RGB24;
		break;
	case 32:
		gfx_format = DISPC_GFX_ATTRIBUTES_GFXFORMAT_ARGB32;
		break;
	default:
		gfx_format = dispc_reg_in(DISPC_GFX_ATTRIBUTES)
		    & DISPC_GFX_ATTRIBUTES_GFXFORMAT;
		break;
	}

	gfx_attributes |= gfx_format;

	dispc_reg_out(DISPC_GFX_POSITION, gfx_position);
	dispc_reg_out(DISPC_GFX_WINDOW_SKIP, gfx_window_skip);

	dispc_reg_out(DISPC_CONFIG, config);
	dispc_reg_out(DISPC_GFX_ATTRIBUTES, gfx_attributes);
	dispc_reg_out(DISPC_GFX_SIZE, gfx_size);

	layer[OMAP2_GRAPHICS].size_x = size_x;
	layer[OMAP2_GRAPHICS].size_y = size_y;
}

/* Calculate the number of pixels sent to the display per pixel clock as
 * (nom/den) pixels per clock.
 */
void omap2_disp_pixels_per_clock(unsigned int *nom, unsigned int *den)
{
	u32 dispc_control;

	dispc_control = dispc_reg_in(DISPC_CONTROL);

	if (dispc_control & DISPC_CONTROL_STNTFT) {
		/* active display (TFT) */
		if (dispc_control & DISPC_CONTROL_TDMENABLE) {
			/* TFT with TDM */
			switch (dispc_control & DISPC_CONTROL_TDMCYCLEFORMAT) {
			case DISPC_CONTROL_TDMCYCLEFORMAT_1CYCPERPIX:
				*nom = 1;
				*den = 1;
				break;
			case DISPC_CONTROL_TDMCYCLEFORMAT_2CYCPERPIX:
				*nom = 1;
				*den = 2;
				break;
			case DISPC_CONTROL_TDMCYCLEFORMAT_3CYCPERPIX:
				*nom = 1;
				*den = 3;
				break;
			case DISPC_CONTROL_TDMCYCLEFORMAT_3CYCPER2PIX:
				*nom = 2;
				*den = 3;
				break;
			}
		} else {
			/* TFT without TDM */
			*nom = 1;
			*den = 1;
		}
	} else {
		/* passive display (STN) */
		if (dispc_control & DISPC_CONTROL_MONOCOLOR) {
			/* STN mono */
			if (dispc_control & DISPC_CONTROL_M8B) {
				/* 8 pixels per pixclock */
				*nom = 8;
				*den = 1;
			} else {
				/* 4 pixels per pixclock */
				*nom = 4;
				*den = 1;
			}
		} else {
			/* STN color--8 pixels per 3 pixclocks */
			*nom = 8;
			*den = 3;
		}
	}
}

/* Configure the signal configuration for LCD panel */
void
omap2_disp_lcdcfg_polfreq(u32 hsync_high, u32 vsync_high,
			  u32 acb, u32 ipc, u32 onoff)
{
	u32 pol_freq = 0;
	/* set the sync polarity */
	if (!hsync_high)
		pol_freq &= ~DISPC_POL_FREQ_IHS;
	else
		pol_freq |= DISPC_POL_FREQ_IHS;
	if (!vsync_high)
		pol_freq &= ~DISPC_POL_FREQ_IVS;
	else
		pol_freq |= DISPC_POL_FREQ_IVS;

	pol_freq |= acb | (ipc << DISPC_POL_FREQ_IPC_SHIFT) <<
	    (onoff << DISPC_POL_FREQ_ONOFF_SHIFT);
	dispc_reg_out(DISPC_POL_FREQ, pol_freq);

#ifdef CONFIG_OMAP34XX_OFFMODE
	/* set the sync polarity */
	if (!hsync_high)
		pol_freq &= ~DISPC_POL_FREQ_IHS;
	else
		pol_freq |= DISPC_POL_FREQ_IHS;

	if (!vsync_high)
		pol_freq &= ~DISPC_POL_FREQ_IVS;
	else
		pol_freq |= DISPC_POL_FREQ_IVS;

	pol_freq |= acb | (ipc << DISPC_POL_FREQ_IPC_SHIFT) <<
	    (onoff << DISPC_POL_FREQ_ONOFF_SHIFT);
	dispc_reg_out(DISPC_POL_FREQ, pol_freq);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */

}

/* Configure LCD parameters */
void omap2_disp_config_lcd(u32 pixdiv, u32 hbp, u32 hfp, u32 hsw,
				u32 vbp, u32 vfp, u32 vsw, u32 bpp)
{
	u32 control, timing_h, timing_v;
	u32 ctl_col_depth;

	/* Adjust out of bounds params */
	if (hbp > 255) hbp = 255;
	if (hfp > 255) hfp = 255;
	if (hsw > 63)  hsw = 63;
	if (vbp > 255) vbp = 255;
	if (vfp > 255) vfp = 255;
	if (vsw > 63)  vsw = 63;

	switch (bpp) {
		case 24:
		case 32: ctl_col_depth = DISPC_CONTROL_TFTDATALINES_OALSB24B; break;
		case 16:
		default: ctl_col_depth = DISPC_CONTROL_TFTDATALINES_OALSB16B; break;
	}

	timing_h = (hbp << DISPC_TIMING_H_HBP_SHIFT) |
		   (hfp << DISPC_TIMING_H_HFP_SHIFT) |
		   (hsw << DISPC_TIMING_H_HSW_SHIFT);

	timing_v = (vbp << DISPC_TIMING_V_VBP_SHIFT) |
		   (vfp << DISPC_TIMING_V_VFP_SHIFT) |
		   (vsw << DISPC_TIMING_V_VSW_SHIFT);

	dispc_reg_out(DISPC_TIMING_H, timing_h);
	dispc_reg_out(DISPC_TIMING_V, timing_v);

	omap2_disp_set_logic_dividers();

	control = dispc_reg_in(DISPC_CONTROL);
	control |= DISPC_CONTROL_GPOUT1 |
		   DISPC_CONTROL_GPOUT0 |
		   ctl_col_depth |
		   DISPC_CONTROL_STNTFT;
	dispc_reg_out(DISPC_CONTROL, control);
}

/* Set the pixel clock divisor for the LCD */
void omap2_disp_set_pcd(u32 pcd)
{
	dispc_reg_out(DISPC_DIVISOR, (1 << DISPC_DIVISOR_LCD_SHIFT) |
		      (pcd << DISPC_DIVISOR_PCD_SHIFT));
}

/*
 * Set the DSS Functional clock
 * The DSS clock should be 4 times the Panel's Pixel clock
 * For TV the Pixel clock required is 13.5Mhz
 * For LCD the Pixel clock is 6Mhz
 */
void omap2_disp_set_dssfclk(void)
{
	/* TODO set the LCD pixel clock rate based on the LCD configuration */
	static int LCD_pixel_clk = 10000000;	/* to get more bandwidth */
#ifdef CONFIG_VIDEO_OMAP24XX_TVOUT
	static int TV_pixel_clk = 14000000;	/* rounded 13.5 to 14 */
#endif
	u32 ask_clkrate = 0, sup_clkrate = 0, tgt_clkrate = 0, i;

	ask_clkrate = LCD_pixel_clk * 4;

#ifdef CONFIG_VIDEO_OMAP24XX_TVOUT
	if (ask_clkrate < (TV_pixel_clk * 4))
		ask_clkrate = TV_pixel_clk * 4;
#endif

	tgt_clkrate = ask_clkrate;

	sup_clkrate = clk_round_rate(dss1f, ask_clkrate);
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		if (clk_get_rate(dss1f) == 96000000) {
			/*96M already, dont do anything for ES 1.0 */
			return;
		}
	} else {
		for (i = 1; i <= 20; i++) {
			sup_clkrate = clk_round_rate(dss1f, ask_clkrate);
			if (sup_clkrate >= tgt_clkrate)
				break;
			ask_clkrate = ask_clkrate + 1000000;
		}
		if (clk_set_rate(dss1f, sup_clkrate) == -EINVAL)
			printk(KERN_ERR "Unable to set the DSS"
			       "functional clock to %d\n", sup_clkrate);
	}
	return;
}

static void omap2_disp_clock_enable(void)
{
	clk_enable(dss1i);
	clk_enable(dss1f);
#ifdef CONFIG_OMAP_DSI
	clk_enable(dss2f);
#endif
}

static void omap2_disp_clock_disable(void)
{
	clk_disable(dss1i);
	clk_disable(dss1f);
#ifdef CONFIG_OMAP_DSI
	clk_disable(dss2f);
#endif
}

void omap2_tv_clock_enable(void)
{
	clk_enable(tv_clk);
}

void omap2_tv_clock_disable(void)
{
	clk_disable(tv_clk);
}

/* HACK HACK HACK */
#define CONFIG_HW_SUP_TRANS

/******************************************************************************/

static void __init omap2_disp_configure(void)
{
	u32 idle_dispc;

#ifndef CONFIG_ARCH_OMAP3410
	omap2_disp_set_tvref(TVREF_ON);
#ifdef CONFIG_OMAP34XX_OFFMODE
	/* Set the TV standard first */
	omap2_disp_set_tvstandard(omap2_current_tvstandard);
#endif
#endif

	/* Enable AUTOIDLE for DSS domain. */
#ifdef CONFIG_HW_SUP_TRANS
	dss_reg_out(DSS_SYSCONFIG, DSS_SYSCONFIG_AUTOIDLE);
#else
	dss_reg_out(DSS_SYSCONFIG, 0);
#endif

	/* Enable SmartIdle and SmartStandby for display controller. */
	idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
	idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE |
			DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
	idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
		      DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
		      DISPC_SYSCONFIG_ENABLE_WKUP |
		      DISPC_SYSCONFIG_AUTOIDLE;
#else
	idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
		      DISPC_SYSCONFIG_SIDLEMODE_NIDLE |
		      DISPC_SYSCONFIG_AUTOIDLE;
#endif
	dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);
}

/******************************************************************************/

static int dss_usage_count = 0;
static int dss_tv_usage_count = 0;

static DEFINE_SPINLOCK(dss_usage_lock);

/* Enable all DSS clocks. */
void omap2_disp_get_dss(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dss_usage_lock, flags);
	if (0 == dss_usage_count++) {
		omap2_disp_clock_enable();
#ifndef CONFIG_ARCH_OMAP3410
		if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) ||
		    (omap2_disp_get_output_dev(OMAP2_VIDEO1)   == OMAP2_OUTPUT_TV) ||
		    (omap2_disp_get_output_dev(OMAP2_VIDEO2)   == OMAP2_OUTPUT_TV)) {
			if (0 == dss_tv_usage_count++) {
				omap2_tv_clock_enable();
			}
		}
#endif
	}
	spin_unlock_irqrestore(&dss_usage_lock, flags);
}

/* Disable all DSS clocks. */
void omap2_disp_put_dss(void)
{
	unsigned long flags;

	spin_lock_irqsave(&dss_usage_lock, flags);
	if (!dss_usage_count) {
		spin_unlock_irqrestore(&dss_usage_lock, flags);
		printk(KERN_WARNING "WARNING: Unbalanced omap2_disp_put_dss()\n");
		return;
	}

	if (0 == --dss_usage_count) {
		omap2_disp_clock_disable();
		if (0 == --dss_tv_usage_count) {
			omap2_tv_clock_disable();
		}
	}
	spin_unlock_irqrestore(&dss_usage_lock, flags);
}

/******************************************************************************/

/* This function must be called by any driver that wishes to use a particular
 * display pipeline (layer).
 */
int omap2_disp_request_layer(int ltype)
{
	int ret = 0;

	spin_lock(&dss_lock);
	if (!layer[ltype].in_use) {
		layer[ltype].in_use = 1;
		ret = 1;
	}
	spin_unlock(&dss_lock);

	DPRINTK("DISPLAY: Layer %d REQUESTED\n", ltype);

	/* Check whether or not we can enable LPR mode or need to disable it.
	 */
	if (ret)
		omap2_disp_handle_lpr_auto_mode();

	return ret;
}

/* This function must be called by a driver when it is done using a particular
 * display pipeline (layer).
 */
void omap2_disp_release_layer(int ltype)
{
	spin_lock(&dss_lock);
	layer[ltype].in_use = 0;
	layer[ltype].ctx_valid = 0;
	spin_unlock(&dss_lock);

	/* Check whether or not we can enable LPR mode or need to disable it.
	 */
	omap2_disp_handle_lpr_auto_mode();

	DPRINTK("DISPLAY: Layer %d RELEASED\n", ltype);
}

int omap2_disp_is_layer_used(int ltype)
{
	return  (layer[ltype].in_use)?1:0;
}


/******************************************************************************/

/* Used to enable LCDENABLE or DIGITALENABLE of the display controller.
 */
void omap2_disp_enable_output_dev(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_LCDENABLE,
				DISPC_CONTROL_LCDENABLE);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_DIGITALENABLE,
				DISPC_CONTROL_DIGITALENABLE);
	}
#endif
}

/* Used to disable LCDENABLE or DIGITALENABLE of the display controller.
 */
void omap2_disp_disable_output_dev(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		dispc_reg_merge(DISPC_CONTROL, ~DISPC_CONTROL_LCDENABLE,
				DISPC_CONTROL_LCDENABLE);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		dispc_reg_merge(DISPC_CONTROL, ~DISPC_CONTROL_DIGITALENABLE,
				DISPC_CONTROL_DIGITALENABLE);
	}
#endif
}

int omap2_disp_get_output_dev(int ltype)
{
	return layer[ltype].output_dev;
}

/******************************************************************************/
/*
 * Used by sysfs entries.
 */
int omap2_disp_get_gfx_fifo_low_threshold(void)
{
	return ((dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD) &
			DISPC_GFX_FIFO_THRESHOLD_LOW) >>
				DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);
}

void omap2_disp_set_gfx_fifo_low_threshold(int thrs)
{
	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD,
			thrs << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT,
			DISPC_GFX_FIFO_THRESHOLD_LOW);

	dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
			DISPC_CONTROL_GOLCD);

}

int omap2_disp_get_gfx_fifo_high_threshold(void)
{
	return ((dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD) &
			DISPC_GFX_FIFO_THRESHOLD_HIGH) >>
				DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT);
}

void omap2_disp_set_gfx_fifo_high_threshold(int thrs)
{
	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD,
			thrs << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT,
			DISPC_GFX_FIFO_THRESHOLD_HIGH);

	dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
			DISPC_CONTROL_GOLCD);

}

/******************************************************************************/
/*
 * This is used to dynamically switch the output of a particular layer to
 * either the LCD or TV.
 */
void omap2_disp_set_output_dev(int ltype, int output_dev)
{
	struct omap2_disp_dma_params *dma_param = 0;
	int vid_pic_size = 0;
	int val = (output_dev == OMAP2_OUTPUT_LCD) ? 0 : ~0;

	layer[ltype].output_dev = output_dev;

	switch (ltype) {
	case OMAP2_GRAPHICS:
		if (layer[ltype].in_use) {
			if (output_dev == OMAP2_OUTPUT_LCD) {
				int gfx_size = 0;
				dma_param = &layer[OMAP2_GRAPHICS].dma[0];
				gfx_size = (((layer[OMAP2_GRAPHICS].size_x - 1)
					     << DISPC_GFX_SIZE_GFXSIZEX_SHIFT)
					    & DISPC_GFX_SIZE_GFXSIZEX)
				    | (((layer[OMAP2_GRAPHICS].size_y - 1)
					<< DISPC_GFX_SIZE_GFXSIZEY_SHIFT)
				       & DISPC_GFX_SIZE_GFXSIZEY);
				dispc_reg_out(DISPC_GFX_SIZE, gfx_size);

			}
#ifndef CONFIG_ARCH_OMAP3410
			else if (output_dev == OMAP2_OUTPUT_TV) {
				int gfx_size = 0;
				dma_param = &layer[OMAP2_GRAPHICS].dma[1];
				/* dividing the size_y by two,
				 * because TV operates in interleaved mode
				 */
				gfx_size = (((layer[OMAP2_GRAPHICS].size_x - 1)
					     << DISPC_GFX_SIZE_GFXSIZEX_SHIFT)
					    & DISPC_GFX_SIZE_GFXSIZEX)
				    | (((layer[OMAP2_GRAPHICS].size_y / 2 - 1)
					<< DISPC_GFX_SIZE_GFXSIZEY_SHIFT)
				       & DISPC_GFX_SIZE_GFXSIZEY);

				/* move graphics display position to cover
				 * TV overscan
				 */
				dispc_reg_out(DISPC_GFX_SIZE, gfx_size);
			}
#endif

			dispc_reg_out(DISPC_GFX_BA0, dma_param->ba0);
			dispc_reg_out(DISPC_GFX_BA1, dma_param->ba1);
			dispc_reg_out(DISPC_GFX_ROW_INC, dma_param->row_inc);
			dispc_reg_out(DISPC_GFX_PIXEL_INC, dma_param->pix_inc);
		}

		dispc_reg_merge(DISPC_GFX_ATTRIBUTES,
				DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT & val,
				DISPC_GFX_ATTRIBUTES_GFXCHANNELOUT);

		if (layer[ltype].in_use)
			omap2_disp_reg_sync(output_dev);
		break;

	case OMAP2_VIDEO1:
		if (layer[ltype].in_use) {
			tvlcd_status.output_dev = output_dev;
			tvlcd_status.ltype = ltype;
			tvlcd_status.status = TVLCD_STOP;
			if (output_dev == OMAP2_OUTPUT_LCD) {
				dma_param = &layer[OMAP2_VIDEO1].dma[0];
				vid_pic_size =
				    (((layer[OMAP2_VIDEO1].size_x -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
				    (((layer[OMAP2_VIDEO1].size_y -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(0),
					      vid_pic_size);
			}
#ifndef CONFIG_ARCH_OMAP3410
			else if (output_dev == OMAP2_OUTPUT_TV) {
				dma_param = &layer[OMAP2_VIDEO1].dma[1];
				vid_pic_size =
				    (((layer[OMAP2_VIDEO1].size_x -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
				    (((layer[OMAP2_VIDEO1].size_y / 2 -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(0),
					      vid_pic_size);
			}
#endif

			dispc_reg_out(DISPC_VID_BA0(0), dma_param->ba0);
			dispc_reg_out(DISPC_VID_BA1(0), dma_param->ba1);
			dispc_reg_out(DISPC_VID_ROW_INC(0), dma_param->row_inc);
			dispc_reg_out(DISPC_VID_PIXEL_INC(0),
				      dma_param->pix_inc);

			dispc_reg_merge(DISPC_VID_ATTRIBUTES(0),
					DISPC_VID_ATTRIBUTES_VIDCHANNELOUT &
					val,
					DISPC_VID_ATTRIBUTES_VIDCHANNELOUT);
			break;
		}

	case OMAP2_VIDEO2:
		if (layer[ltype].in_use) {

			tvlcd_status.output_dev = output_dev;
			tvlcd_status.ltype = ltype;
			tvlcd_status.status = TVLCD_STOP;
			if (output_dev == OMAP2_OUTPUT_LCD) {
				dma_param = &layer[OMAP2_VIDEO2].dma[0];
				vid_pic_size =
				    (((layer[OMAP2_VIDEO2].size_x -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
				    (((layer[OMAP2_VIDEO2].size_y -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(1),
					      vid_pic_size);
			}
#ifndef CONFIG_ARCH_OMAP3410
			else if (output_dev == OMAP2_OUTPUT_TV) {
				dma_param = &layer[OMAP2_VIDEO2].dma[1];
				vid_pic_size =
				    (((layer[OMAP2_VIDEO2].size_x -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEX_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEX) |
				    (((layer[OMAP2_VIDEO2].size_y / 2 -
				       1) <<
				      DISPC_VID_PICTURE_SIZE_VIDORGSIZEY_SHIFT)
				     & DISPC_VID_PICTURE_SIZE_VIDORGSIZEY);
				dispc_reg_out(DISPC_VID_PICTURE_SIZE(1),
					      vid_pic_size);
			}
#endif

			dispc_reg_out(DISPC_VID_BA0(1), dma_param->ba0);
			dispc_reg_out(DISPC_VID_BA1(1), dma_param->ba1);
			dispc_reg_out(DISPC_VID_ROW_INC(1), dma_param->row_inc);
			dispc_reg_out(DISPC_VID_PIXEL_INC(1),
				      dma_param->pix_inc);

			dispc_reg_merge(DISPC_VID_ATTRIBUTES(1),
					DISPC_VID_ATTRIBUTES_VIDCHANNELOUT &
					val,
					DISPC_VID_ATTRIBUTES_VIDCHANNELOUT);
			break;
		}
	}
}

/* Used to save the DMA parameter settings for a particular layer to be
 * displayed on a particular output device. These values help the
 * omap2_disp_set_output_dev() function to dynamically switch the output of a
 * layer to any output device.
 */
void
omap2_disp_set_dma_params(int ltype, int output_dev,
			  u32 ba0, u32 ba1, u32 row_inc, u32 pix_inc)
{
	struct omap2_disp_dma_params *dma;

	if (output_dev == OMAP2_OUTPUT_LCD)
		dma = &layer[ltype].dma[0];
	else
		dma = &layer[ltype].dma[1];

	dma->ba0 = ba0;
	dma->ba1 = ba1;
	dma->row_inc = row_inc;
	dma->pix_inc = pix_inc;
}

void omap2_disp_start_gfxlayer(void)
{
	omap2_disp_set_output_dev(OMAP2_GRAPHICS,
				  layer[OMAP2_GRAPHICS].output_dev);
	omap2_disp_reg_sync(layer[OMAP2_GRAPHICS].output_dev);
}

/******************************************************************************/

/* Sets the background color */
void omap2_disp_set_bg_color(int output_dev, int color)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_out(DISPC_DEFAULT_COLOR0, color);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		dispc_reg_out(DISPC_DEFAULT_COLOR1, color);
#endif

	omap2_disp_reg_sync(output_dev);
}

/* Returns the current background color */
void omap2_disp_get_bg_color(int output_dev, int *color)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		*color = dispc_reg_in(DISPC_DEFAULT_COLOR0);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		*color = dispc_reg_in(DISPC_DEFAULT_COLOR1);
#endif
}

/******************************************************************************/

/* Enable/Disable the Dithering block */
void omap2_disp_set_dithering(int dither_state)
{
	omap2_disp_get_dss();
	switch (dither_state) {
	case DITHERING_ON:
		dispc_reg_out(DISPC_CONTROL,
			      (dispc_reg_in(DISPC_CONTROL) |
			       DISPC_CONTROL_TFTDITHERENABLE));
		break;
	case DITHERING_OFF:
		dispc_reg_out(DISPC_CONTROL,
			      (dispc_reg_in(DISPC_CONTROL) &
			       ~DISPC_CONTROL_TFTDITHERENABLE));
		break;
	}
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
	// omap2_disp_reg_sync(OMAP2_OUTPUT_TV);
	omap2_disp_put_dss();
}

/* Get the Dithering state */
int omap2_disp_get_dithering(void)
{
	omap2_disp_get_dss();
	if (dispc_reg_in(DISPC_CONTROL) & 0x00000080) {
		return (DITHERING_ON);
	} else {
		return (DITHERING_OFF);
	}
	omap2_disp_put_dss();
}

/******************************************************************************/

/* Get the number of data lines connected to LCD panel*/
int omap2_disp_get_lcddatalines(void)
{
	u32 tft_data_lines = 0;

	omap2_disp_get_dss();
	tft_data_lines = dispc_reg_in(DISPC_CONTROL)
	    & (DISPC_CONTROL_TFTDATALINES);

	switch (tft_data_lines) {
	case DISPC_CONTROL_TFTDATALINES_OALSB12B:
		return (LCD_DATA_LINE_12BIT);
	case DISPC_CONTROL_TFTDATALINES_OALSB16B:
		return (LCD_DATA_LINE_16BIT);
	case DISPC_CONTROL_TFTDATALINES_OALSB18B:
		return (LCD_DATA_LINE_18BIT);
	case DISPC_CONTROL_TFTDATALINES_OALSB24B:
		return (LCD_DATA_LINE_24BIT);
	}
	omap2_disp_put_dss();
	return (LCD_DATA_LINE_16BIT);
}

/* Set number of data lines to be connected to LCD panel*/
void omap2_disp_set_lcddatalines(int no_of_lines)
{
	omap2_disp_get_dss();
	dispc_reg_out(DISPC_CONTROL,
		      (dispc_reg_in(DISPC_CONTROL) &
		       ~DISPC_CONTROL_TFTDATALINES));

	switch (no_of_lines) {
	case LCD_DATA_LINE_12BIT:
		dispc_reg_out(DISPC_CONTROL, (dispc_reg_in(DISPC_CONTROL)
					      |
					      DISPC_CONTROL_TFTDATALINES_OALSB12B));
		break;
	case LCD_DATA_LINE_16BIT:
		dispc_reg_out(DISPC_CONTROL, (dispc_reg_in(DISPC_CONTROL)
					      |
					      DISPC_CONTROL_TFTDATALINES_OALSB16B));
		break;
	case LCD_DATA_LINE_18BIT:
		dispc_reg_out(DISPC_CONTROL, (dispc_reg_in(DISPC_CONTROL)
					      |
					      DISPC_CONTROL_TFTDATALINES_OALSB18B));
		break;
	case LCD_DATA_LINE_24BIT:
		dispc_reg_out(DISPC_CONTROL, (dispc_reg_in(DISPC_CONTROL)
					      |
					      DISPC_CONTROL_TFTDATALINES_OALSB24B));
		break;
	}
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
	omap2_disp_put_dss();
}

/******************************************************************************/

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430) && !defined(CONFIG_ARCH_OMAP3410)
/* Turn on/off the TV reference voltage from OMAP */
void omap2_disp_set_tvref(int tvref_state)
{
	switch (tvref_state) {
	case TVREF_ON:
		dss_reg_out(DSS_CONTROL, (dss_reg_in(DSS_CONTROL)
					  | DSS_CONTROL_TV_REF));
		break;
	case TVREF_OFF:
		dss_reg_out(DSS_CONTROL, (dss_reg_in(DSS_CONTROL) &
					  ~(DSS_CONTROL_TV_REF)));
		break;
	}
}
#endif

/* Sets the SMS settings for rotation using the VRFB.
 */
int
omap2_disp_set_vrfb(int context, u32 phy_addr,
		    u32 width, u32 height, u32 bytes_per_pixel)
{
	int page_width_exp, page_height_exp, pixel_size_exp;

	if (bytes_per_pixel > 4)
		return -EINVAL;

	page_width_exp = PAGE_WIDTH_EXP;
	page_height_exp = PAGE_HEIGHT_EXP;
	pixel_size_exp = bytes_per_pixel >> 1;

	width = ((1 << page_width_exp) *
		 (pages_per_side(width * bytes_per_pixel, page_width_exp))
	    ) >> pixel_size_exp;	// in pixels

	height = (1 << page_height_exp) *
	    (pages_per_side(height, page_height_exp));

	SMS_ROT0_PHYSICAL_BA(context) = phy_addr;
	SMS_ROT0_SIZE(context) = 0;
	SMS_ROT0_SIZE(context) |= (width << SMS_IMAGEWIDTH_OFFSET)
	    | (height << SMS_IMAGEHEIGHT_OFFSET);
	SMS_ROT_CONTROL(context) = 0;

	SMS_ROT_CONTROL(context) |= pixel_size_exp << SMS_PS_OFFSET
	    | (page_width_exp - pixel_size_exp) << SMS_PW_OFFSET
	    | page_height_exp << SMS_PH_OFFSET;

	return 0;
}

#ifndef CONFIG_ARCH_OMAP3410
/* Sets VENC registers for TV operation.
*/
static void config_venc(struct tv_standard_config *tvstd)
{
	venc_reg_out(VENC_F_CONTROL, F_CONTROL_GEN);
	venc_reg_out(VENC_SYNC_CONTROL, SYNC_CONTROL_GEN);
	venc_reg_out(VENC_LLEN, tvstd->venc_llen);
	venc_reg_out(VENC_FLENS, tvstd->venc_flens);
	venc_reg_out(VENC_HFLTR_CTRL, tvstd->venc_hfltr_ctrl);
	venc_reg_out(VENC_CC_CARR_WSS_CARR, tvstd->venc_cc_carr_wss_carr);
	venc_reg_out(VENC_C_PHASE, tvstd->venc_c_phase);
	venc_reg_out(VENC_GAIN_U, tvstd->venc_gain_u);
	venc_reg_out(VENC_GAIN_V, tvstd->venc_gain_v);
	venc_reg_out(VENC_GAIN_Y, tvstd->venc_gain_y);
	venc_reg_out(VENC_BLACK_LEVEL, tvstd->venc_black_level);
	venc_reg_out(VENC_BLANK_LEVEL, tvstd->venc_blank_level);
	venc_reg_out(VENC_X_COLOR, tvstd->venc_x_color);
	venc_reg_out(VENC_M_CONTROL, tvstd->venc_m_control);
	venc_reg_out(VENC_BSTAMP_WSS_DATA, tvstd->venc_bstamp_wss_data);
	venc_reg_out(VENC_S_CARR, tvstd->venc_s_carr);
	venc_reg_out(VENC_LINE21, tvstd->venc_line21);
	venc_reg_out(VENC_LN_SEL, tvstd->venc_ln_sel);
	venc_reg_out(VENC_L21_WC_CTL, tvstd->venc_l21_wc_ctl);
	venc_reg_out(VENC_HTRIGGER_VTRIGGER, tvstd->venc_htrigger_vtrigger);
	venc_reg_out(VENC_SAVID_EAVID, tvstd->venc_savid_eavid);
	venc_reg_out(VENC_FLEN_FAL, tvstd->venc_flen_fal);
	venc_reg_out(VENC_LAL_PHASE_RESET, tvstd->venc_lal_phase_reset);
	venc_reg_out(VENC_HS_INT_START_STOP_X, tvstd->venc_hs_int_start_stop_x);
	venc_reg_out(VENC_HS_EXT_START_STOP_X, tvstd->venc_hs_ext_start_stop_x);
	venc_reg_out(VENC_VS_INT_START_X, tvstd->venc_vs_int_start_x);
	venc_reg_out(VENC_VS_INT_STOP_X_VS_INT_START_Y,
		     tvstd->venc_vs_int_stop_x_vs_int_start_y);
	venc_reg_out(VENC_VS_INT_STOP_Y_VS_EXT_START_X,
		     tvstd->venc_vs_int_stop_y_vs_ext_start_x);
	venc_reg_out(VENC_VS_EXT_STOP_X_VS_EXT_START_Y,
		     tvstd->venc_vs_ext_stop_x_vs_ext_start_y);
	venc_reg_out(VENC_VS_EXT_STOP_Y, tvstd->venc_vs_ext_stop_y);
	venc_reg_out(VENC_AVID_START_STOP_X, tvstd->venc_avid_start_stop_x);
	venc_reg_out(VENC_AVID_START_STOP_Y, tvstd->venc_avid_start_stop_y);
	venc_reg_out(VENC_FID_INT_START_X_FID_INT_START_Y,
		     tvstd->venc_fid_int_start_x_fid_int_start_y);
	venc_reg_out(VENC_FID_INT_OFFSET_Y_FID_EXT_START_X,
		     tvstd->venc_fid_int_offset_y_fid_ext_start_x);
	venc_reg_out(VENC_FID_EXT_START_Y_FID_EXT_OFFSET_Y,
		     tvstd->venc_fid_ext_start_y_fid_ext_offset_y);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_X,
		     tvstd->venc_tvdetgp_int_start_stop_x);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_Y,
		     tvstd->venc_tvdetgp_int_start_stop_y);
	venc_reg_out(VENC_GEN_CTRL, tvstd->venc_gen_ctrl);
	venc_reg_out(VENC_DAC_TST, tvstd->venc_dac_tst);
	venc_reg_out(VENC_DAC, venc_reg_in(VENC_DAC));
}

int omap2_disp_get_tvstandard(void)
{
	return (omap2_current_tvstandard);
}

#else
int omap2_disp_get_tvstandard(void)
{
	return 0;
}
#endif

void omap2_disp_get_tvlcd(struct tvlcd_status_t *status)
{
	status->status = tvlcd_status.status;
	status->output_dev = tvlcd_status.output_dev;
	status->ltype = tvlcd_status.ltype;
}

void omap2_disp_set_tvlcd(int status)
{
	tvlcd_status.status = status;
}

#ifndef CONFIG_ARCH_OMAP3410
void omap2_disp_set_tvstandard(int tvstandard)
{
	omap2_current_tvstandard = tvstandard;
	switch (tvstandard) {
	case PAL_BDGHI:
		config_venc(&pal_bdghi_cfg);
		break;
	case PAL_NC:
		config_venc(&pal_nc_cfg);
		break;
	case PAL_N:
		config_venc(&pal_n_cfg);
		break;
	case PAL_M:
		config_venc(&pal_m_cfg);
		break;
	case PAL_60:
		config_venc(&pal_60_cfg);
		omap2_disp_set_panel_size(OMAP2_OUTPUT_TV, 720, 574);
		break;
	case NTSC_M:
		config_venc(&ntsc_m_cfg);
		break;
	case NTSC_J:
		config_venc(&ntsc_j_cfg);
		break;
	case NTSC_443:
		config_venc(&ntsc_443_cfg);
		omap2_disp_set_panel_size(OMAP2_OUTPUT_TV, 720, 480);
		break;
	}

	msleep(50);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);
}

#else
void omap2_disp_set_tvstandard(int tvstandard)
{
	return;
}
#endif

/******************************************************************************/

/* Sets the transparency color key type and value.
*/
void omap2_disp_set_colorkey(int output_dev, int key_type, int key_val)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		if (key_type == OMAP2_VIDEO_SOURCE)
			dispc_reg_merge(DISPC_CONFIG,
					DISPC_CONFIG_TCKLCDSELECTION,
					DISPC_CONFIG_TCKLCDSELECTION);
		else
			dispc_reg_merge(DISPC_CONFIG, 0,
					DISPC_CONFIG_TCKLCDSELECTION);
		dispc_reg_out(DISPC_TRANS_COLOR0, key_val);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if (key_type == OMAP2_VIDEO_SOURCE)
			dispc_reg_merge(DISPC_CONFIG,
					DISPC_CONFIG_TCKDIGSELECTION,
					DISPC_CONFIG_TCKDIGSELECTION);
		else
			dispc_reg_merge(DISPC_CONFIG, 0,
					DISPC_CONFIG_TCKDIGSELECTION);
		dispc_reg_out(DISPC_TRANS_COLOR1, key_val);
	}
#endif

	omap2_disp_reg_sync(output_dev);
}

/* Returns the current transparency color key type and value.
*/
void omap2_disp_get_colorkey(int output_dev, int *key_type, int *key_val)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		if (dispc_reg_in(DISPC_CONFIG) & DISPC_CONFIG_TCKLCDSELECTION)
			*key_type = OMAP2_VIDEO_SOURCE;
		else
			*key_type = OMAP2_GFX_DESTINATION;
		*key_val = dispc_reg_in(DISPC_TRANS_COLOR0);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if (dispc_reg_in(DISPC_CONFIG) & DISPC_CONFIG_TCKDIGSELECTION)
			*key_type = OMAP2_VIDEO_SOURCE;
		else
			*key_type = OMAP2_GFX_DESTINATION;
		*key_val = dispc_reg_in(DISPC_TRANS_COLOR1);
	}
#endif
}

void omap2_disp_enable_colorkey(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TCKLCDENABLE,
				DISPC_CONFIG_TCKLCDENABLE);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_TCKDIGENABLE,
				DISPC_CONFIG_TCKDIGENABLE);
#endif

	omap2_disp_reg_sync(output_dev);
}

void omap2_disp_disable_colorkey(int output_dev)
{
	if (output_dev == OMAP2_OUTPUT_LCD)
		dispc_reg_merge(DISPC_CONFIG, ~DISPC_CONFIG_TCKLCDENABLE,
				DISPC_CONFIG_TCKLCDENABLE);
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV)
		dispc_reg_merge(DISPC_CONFIG, ~DISPC_CONFIG_TCKDIGENABLE,
				DISPC_CONFIG_TCKDIGENABLE);
#endif

	omap2_disp_reg_sync(output_dev);
}

/******************************************************************************/

#ifdef CONFIG_ARCH_OMAP34XX
void omap2_disp_set_alphablend(int output_dev, int value)
{
	if (output_dev == OMAP2_OUTPUT_LCD) {
		if (value)
			dispc_reg_merge(DISPC_CONFIG,
					DISPC_CONFIG_LCDALPHAENABLE,
					DISPC_CONFIG_LCDALPHAENABLE);
		else
			dispc_reg_merge(DISPC_CONFIG,
					~DISPC_CONFIG_LCDALPHAENABLE,
					DISPC_CONFIG_LCDALPHAENABLE);
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if (value)
			dispc_reg_merge(DISPC_CONFIG,
					DISPC_CONFIG_TVALPHAENABLE,
					DISPC_CONFIG_TVALPHAENABLE);
		else
			dispc_reg_merge(DISPC_CONFIG,
					~DISPC_CONFIG_TVALPHAENABLE,
					DISPC_CONFIG_TVALPHAENABLE);
	}
#endif
	omap2_disp_reg_sync(output_dev);
}

void omap2_disp_set_global_alphablend_value(int ltype, int value)
{
	unsigned int alpha_value;

	if (ltype == OMAP2_GRAPHICS) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (~DISPC_GLOBAL_ALPHA_GFX_GALPHA);
		alpha_value |= (value << DISPC_GLOBAL_ALPHA_GFX_GALPHA_SHIFT);
		dispc_reg_out(DISPC_GLOBAL_ALPHA, alpha_value);

	} else if (ltype == OMAP2_VIDEO2) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (~DISPC_GLOBAL_ALPHA_VID2_GALPHA);
		alpha_value |= (value << DISPC_GLOBAL_ALPHA_VID2_GALPHA_SHIFT);
		dispc_reg_out(DISPC_GLOBAL_ALPHA, alpha_value);

	}
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
#ifndef CONFIG_ARCH_OMAP3410
	omap2_disp_reg_sync(OMAP2_OUTPUT_TV);
#endif
}

unsigned char omap2_disp_get_global_alphablend_value(int ltype)
{
	unsigned int alpha_value = 0;

	if (ltype == OMAP2_GRAPHICS) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (DISPC_GLOBAL_ALPHA_GFX_GALPHA);
		alpha_value =
		    alpha_value >> DISPC_GLOBAL_ALPHA_GFX_GALPHA_SHIFT;
	} else if (ltype == OMAP2_VIDEO2) {
		alpha_value = dispc_reg_in(DISPC_GLOBAL_ALPHA);
		alpha_value &= (DISPC_GLOBAL_ALPHA_VID2_GALPHA);
		alpha_value =
		    alpha_value >> DISPC_GLOBAL_ALPHA_VID2_GALPHA_SHIFT;
	}
	return (unsigned char)alpha_value;
}

int omap2_disp_get_alphablend(int output_dev)
{

	if (output_dev == OMAP2_OUTPUT_LCD) {
		if (dispc_reg_in(DISPC_CONFIG) & 0x00040000)
			return 1;
		else
			return 0;
	}
#ifndef CONFIG_ARCH_OMAP3410
	else if (output_dev == OMAP2_OUTPUT_TV) {
		if (dispc_reg_in(DISPC_CONFIG) & 0x00080000)
			return 1;
		else
			return 0;
	}
#endif
	return 0;
}
#endif

int omap2_disp_reg_sync_bit(int output_dev)
{
	u32 control = dispc_reg_in(DISPC_CONTROL);

	if (output_dev == OMAP2_OUTPUT_LCD) {
		return (control & DISPC_CONTROL_GOLCD) >> 5;
	} else {
		return (control & DISPC_CONTROL_GODIGITAL) >> 6;
	}
}

/*
 * This function is required for 2420 errata 1.97
 */

#ifndef CONFIG_ARCH_OMAP3410
static void omap2_reset_venc(void)
{
	int retry;

	omap2_tv_clock_enable();

	/*
	 * Write 1 to the 8th bit of the F_Control register to reset the VENC
	 */
	venc_reg_merge(VENC_F_CONTROL, VENC_FCONTROL_RESET, VENC_FCONTROL_RESET);

	/* wait for reset to complete */
	retry = 100;
	while ((venc_reg_in(VENC_F_CONTROL) & VENC_FCONTROL_RESET) && retry--)
		udelay(1);

	if (retry < 0) {
		printk(KERN_WARNING "omap2_disp: timeout waiting for venc reset\n");

		/* remove the reset */
		venc_reg_merge(VENC_F_CONTROL, (0 << 8), VENC_FCONTROL_RESET);
	}

	/* disable the VENC clock */
	omap2_tv_clock_disable();
}
#endif

/******************************************************************************/

/*
 * Enables an IRQ in DSPC_IRQENABLE.
 *
 * WARNING: It is assumed that each isr routine installed is unique
 * so if the same routine must be installed more then one time - it 
 * should be handled externally.
 *
 */
int omap2_disp_irqenable(omap2_disp_isr_t isr, unsigned int mask)
{
	int i;
	unsigned long flags;

	if (omap2_disp_irq == 0 || mask == 0)
		return -EINVAL;

	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == isr) {
			registered_isr[i].mask |= mask;
			dispc_reg_out(DISPC_IRQENABLE,
				      dispc_reg_in(DISPC_IRQENABLE) | mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	return -EBUSY;
}

/*
 * Disables an IRQ in DISPC_IRQENABLE,
 * The IRQ will be active if any other ISR is still using the same.
 * mask : should contain '0' for irq to be disable and rest should be '1'.
 */
int omap2_disp_irqdisable(omap2_disp_isr_t isr, unsigned int mask)
{
	int i;
	unsigned long flags;
	unsigned int new_mask = 0;

	if (omap2_disp_irq == 0)
		return -EINVAL;

	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++)
		if (registered_isr[i].isr == isr)
			break;

	if (i == MAX_ISR_NR) {
		spin_unlock_irqrestore(&dss_lock, flags);
		return -EINVAL;
	}

	registered_isr[i].mask &= mask;

	/* disable an IRQ if every one wishes to do so */
	for (i = 0; i < MAX_ISR_NR; i++) {
		new_mask |= registered_isr[i].mask;
	}

	dispc_reg_out(DISPC_IRQENABLE, new_mask);
	spin_unlock_irqrestore(&dss_lock, flags);
	return 0;
}

/* Display controller interrupts are handled first by this display library.
 * Drivers that need to use certain interrupts should register their ISRs and
 * interrupt enable mask with the display library.
 */
int omap2_disp_register_isr(omap2_disp_isr_t isr, void *arg, unsigned int mask)
{
	int i;
	unsigned long flags;

	if (omap2_disp_irq == 0 || isr == 0 || arg == 0)
		return -EINVAL;

	/* Clear all the interrupt, so that you dont get an immediate interrupt */
	dispc_reg_out(DISPC_IRQSTATUS, 0xFFFFFFFF);
	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == NULL) {
			registered_isr[i].isr = isr;
			registered_isr[i].arg = arg;
			registered_isr[i].mask = mask;

			/* Clear previous interrupts if any */
			dispc_reg_out(DISPC_IRQSTATUS, mask);
			dispc_reg_out(DISPC_IRQENABLE,
				      dispc_reg_in(DISPC_IRQENABLE) | mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	return -EBUSY;
}

int omap2_disp_unregister_isr(omap2_disp_isr_t isr)
{
	int i, j;
	unsigned long flags;
	unsigned int new_mask = 0;

	if (omap2_disp_irq == 0)
		return -EINVAL;

	spin_lock_irqsave(&dss_lock, flags);
	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == isr) {
			registered_isr[i].isr = NULL;
			registered_isr[i].arg = NULL;
			registered_isr[i].mask = 0;

			/* The interrupt may no longer be valid, re-set the IRQENABLE */
			for (j = 0; j < MAX_ISR_NR; j++) {
				new_mask |= registered_isr[j].mask;
			}
			dispc_reg_out(DISPC_IRQENABLE, new_mask);
			spin_unlock_irqrestore(&dss_lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&dss_lock, flags);
	return -EINVAL;
}

/* DSS Interrupt master service routine. */
static irqreturn_t
omap2_disp_master_isr(int irq, void *arg, struct pt_regs *regs)
{
	unsigned long dispc_irqstatus;
	int i;

	/* Enable DSS clocks */
	omap2_disp_get_dss();

	/* get irq status */
	dispc_irqstatus = dispc_reg_in(DISPC_IRQSTATUS);

	if (dispc_irqstatus) {
		for (i = 0; i < MAX_ISR_NR; i++) {
			if (registered_isr[i].isr == NULL)
				continue;
			if (registered_isr[i].mask & dispc_irqstatus)
				registered_isr[i].isr(registered_isr[i].arg, regs);
		}

		/* ack the interrupt */
		dispc_reg_out(DISPC_IRQSTATUS, dispc_irqstatus);
	}

	/* Disable DSS clocks */
	omap2_disp_put_dss();

	return IRQ_HANDLED;
}

/******************************************************************************/

static int omap2_disp_get_clks(void)
{
	dss1i  = clk_get(NULL, "dss_ick");
	dss1f  = clk_get(NULL, "dss1_alwon_fck");
#ifdef CONFIG_OMAP_DSI
	dss2f  = clk_get(NULL, "dss2_alwon_fck");
#endif
	tv_clk = clk_get(NULL, "dss_tv_fck");

	if (IS_ERR(dss1i) ||
	    IS_ERR(dss1f) ||
#ifdef CONFIG_OMAP_DSI
	    IS_ERR(dss2f) ||
#endif
	    IS_ERR(tv_clk)) {
		printk(KERN_ERR "ERROR getting Display clocks.\n");
		return -1;
	}
	return 0;
}

int __init omap2_disp_init(void)
{
	int rev, i;
	u32 dss_control;

	spin_lock_init(&dss_lock);

	/* Get DSS clocks.  */
	if (0 != omap2_disp_get_clks())
		goto err_get_clks;

	/* Enable DSS clocks. */
	omap2_disp_get_dss();

	/* Configure DSS.  */
	omap2_disp_configure();

	rev = dss_reg_in(DSS_REVISION);
	printk(KERN_INFO "OMAP: Display hardware version %d.%d\n",
	       (rev & DISPC_REVISION_MAJOR) >> DISPC_REVISION_MAJOR_SHIFT,
	       (rev & DISPC_REVISION_MINOR) >> DISPC_REVISION_MINOR_SHIFT);

	/* Disable RFBI mode, which is not currently supported. */
	dispc_reg_merge(DISPC_CONTROL, 0, DISPC_CONTROL_RFBIMODE);

	/* For 2420 VENC errata 1.97 */
	/* For 2430 VENC errata 1.20 */
#ifndef CONFIG_ARCH_OMAP3410
	omap2_reset_venc();
#endif
	/* enable DAC_DEMEN and VENC_4X_CLOCK in DSS for TV operation */
	dss_control = dss_reg_in(DSS_CONTROL);
	dss_control &= ~DSS_CONTROL_VENC_CLOCK_MODE;
	dss_reg_out(DSS_CONTROL, dss_control);

	/* By default, all layers go to LCD */
	layer[OMAP2_GRAPHICS].output_dev = OMAP2_OUTPUT_LCD;
	layer[OMAP2_VIDEO1].output_dev   = OMAP2_OUTPUT_LCD;
	layer[OMAP2_VIDEO2].output_dev   = OMAP2_OUTPUT_LCD;

	/* Set the default color conversion parameters for Video pipelines
	 * by default the color space is set to JPEG
	 */
	update_colorconv_mtx(0, cc_bt709);
	set_colorconv(0, V4L2_COLORSPACE_JPEG);

	update_colorconv_mtx(1, cc_bt709);
	set_colorconv(1, V4L2_COLORSPACE_JPEG);

	/* Set the default background color to be black */
	omap2_disp_set_bg_color(OMAP2_OUTPUT_LCD, 0x000000);
#ifndef CONFIG_ARCH_OMAP3410
	omap2_disp_set_bg_color(OMAP2_OUTPUT_TV, 0x000000);
#endif

	if (request_irq(INT_24XX_DSS_IRQ, (void *)omap2_disp_master_isr,
				IRQF_SHARED, "OMAP2 Display", registered_isr)) {
		printk(KERN_WARNING "omap2_disp: request_irq failed\n");
		omap2_disp_irq = 0;
	} else {
		omap2_disp_irq = 1;
		for (i = 0; i < MAX_ISR_NR; i++) {
			registered_isr[i].isr  = NULL;
			registered_isr[i].mask = 0;
		}
		/* Clear all the pending interrupts, if any */
		dispc_reg_out(DISPC_IRQSTATUS, 0xFFFFFFFF);
		omap2_disp_register_isr(omap2_synclost_isr, layer,
					DISPC_IRQSTATUS_SYNCLOST |
					DISPC_IRQSTATUS_OCPERROR |
					DISPC_IRQSTATUS_GFXFIFOUNDERFLOW);
	}

#ifndef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	/* If the display was already on, we are not disabling the clocks here.
	 * This is equivalent of having handled a LCD state on request. We will
	 * turn off the clocks when we get a LCD state off request.
	 */

	/* Disable DSS clocks. */
	omap2_disp_put_dss();
#endif

#ifdef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	/* If the display is already initialized, we need to fake enable on the
	 * graphics layer.
	 */
	graphics_in_use = 1;
#endif

	return 0;

err_get_clks:
	printk(KERN_ERR "OMAP: Display initialization failed.\n");
	return -1;
}

void omap2_disp_hack_synclost_irq_enable(void)
{
	dispc_reg_out(DISPC_IRQSTATUS, DISPC_IRQSTATUS_SYNCLOST | DISPC_IRQSTATUS_OCPERROR |
					DISPC_IRQSTATUS_GFXFIFOUNDERFLOW);

	omap2_disp_irqenable(omap2_synclost_isr,
			DISPC_IRQSTATUS_SYNCLOST | DISPC_IRQSTATUS_OCPERROR |
					DISPC_IRQSTATUS_GFXFIFOUNDERFLOW);
}

void omap2_disp_hack_synclost_irq_disable(void)
{
	omap2_disp_irqdisable(omap2_synclost_isr, 
			~(DISPC_IRQSTATUS_SYNCLOST | DISPC_IRQSTATUS_OCPERROR |
					DISPC_IRQSTATUS_GFXFIFOUNDERFLOW));
}

/******************************************************************************/

static DEFINE_MUTEX(lpr_mutex);

static struct dss_run_mode {
	u32 divisors;
	u32 timing_h;
	u32 timing_v;
	u32 fifo_thrs;
} rmode;

/**
 * LPR handling
 */
static void omap2_disp_set_logic_dividers(void)
{
	u32 div;

	if (!lpr.cfg_valid)
		return;

	if (DISPLAY_LPR_STATE_ON == lpr.state) {
		div = lpr.on.logdiv << DISPC_DIVISOR_LCD_SHIFT |
		      lpr.on.pixdiv << DISPC_DIVISOR_PCD_SHIFT ;
	}
	else {
		div = lpr.off.logdiv << DISPC_DIVISOR_LCD_SHIFT |
		      lpr.off.pixdiv << DISPC_DIVISOR_PCD_SHIFT ;
	}
	dispc_reg_merge(DISPC_DIVISOR, div,
				DISPC_DIVISOR_LCD | DISPC_DIVISOR_PCD);
}

static void omap2_disp_set_fifo_thresholds(void)
{
	u32 fifo_thrs;

	if (!lpr.cfg_valid)
		return;

	if (DISPLAY_LPR_STATE_ON == lpr.state) {
		fifo_thrs =
		    (lpr.on.fifo_hi_thrs << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
		    (lpr.on.fifo_lo_thrs << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);
	}
	else {
		/*  use safe values */
		fifo_thrs =
		    (lpr.off.fifo_hi_thrs << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
		    (lpr.off.fifo_lo_thrs << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);
	}
	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD, fifo_thrs,
				(DISPC_GFX_FIFO_THRESHOLD_HIGH |
					 DISPC_GFX_FIFO_THRESHOLD_LOW));
}

int omap2_disp_lpr_mode(void)
{
	return lpr.mode;
}

int omap2_disp_lpr_state(void)
{
	return lpr.state;
}

int omap2_disp_lpr_is_enabled(void)
{
	return DISPLAY_LPR_STATE_ON == lpr.state;
}

static int omap2_disp_lpr_enable(void)
{
	int v_attr;
	unsigned long flags;

	/*
	 * Check whether LPR can be triggered
	 *   - gfx pipeline is routed to LCD
	 *   - both video pipelines are disabled (this allows FIFOs merge)
	 */

	/* may be more meaningful error code is required */
	if (omap2_disp_get_output_dev(OMAP2_GRAPHICS) != OMAP2_OUTPUT_LCD) {
		return -1;
	}

	omap2_disp_get_dss();

	v_attr = dispc_reg_in(DISPC_VID_ATTRIBUTES(0)) |
		 dispc_reg_in(DISPC_VID_ATTRIBUTES(1));

	if (v_attr & DISPC_VID_ATTRIBUTES_ENABLE) {
		goto lpr_out_clk;
	}

	/*
	 * currently DSS is running on DSS1 by default. just warn if it has
	 * changed in the future
	 */
	if (dss_reg_in(DSS_CONTROL) & DSS_CONTROL_APLL_CLK)
		BUG();


	rmode.divisors = dispc_reg_in(DISPC_DIVISOR) & (DISPC_DIVISOR_LCD |
						        DISPC_DIVISOR_PCD);

	rmode.timing_h = dispc_reg_in(DISPC_TIMING_H) & (DISPC_TIMING_H_HBP |
							 DISPC_TIMING_H_HFP |
							 DISPC_TIMING_H_HSW);

	rmode.timing_v = dispc_reg_in(DISPC_TIMING_V) & (DISPC_TIMING_V_VBP |
							 DISPC_TIMING_V_VFP |
							 DISPC_TIMING_V_VSW);

	rmode.fifo_thrs = dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD) &
					(DISPC_GFX_FIFO_THRESHOLD_HIGH |
					 DISPC_GFX_FIFO_THRESHOLD_LOW);

	/* Update the state before we set the dividers and FIFO thresholds...
	 */
	lpr.state = DISPLAY_LPR_STATE_ON;

	/* Configure LCD and PCD. */
	omap2_disp_set_logic_dividers();

	/* Configure FIFO thresholds and enable FIFO merging. */
	omap2_disp_set_fifo_thresholds();

	/* Merge the FIFOs. */
	dispc_reg_merge(DISPC_CONFIG,
			DISPC_CONFIG_FIFOMERGE, DISPC_CONFIG_FIFOMERGE);

	/* let LPR settings to take effect */
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);

	/* Save LPR configuration of DISPC and GFX register in case synclost
	 * happens during LPR. If synclost happens LPR parameters get reset to
	 * last-known-good LPR parameters.
	 *
	 * May be useful to restore known-good parameters if FIFO underrun
	 * occurs as well.
	 */
	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_save_ctx(OMAP2_GRAPHICS);
	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);

	omap2_disp_save_ctx(OMAP2_VIDEO1);
	omap2_disp_save_ctx(OMAP2_VIDEO2);

	spin_unlock_irqrestore(&dss_lock, flags);

	omap2_disp_put_dss();

	return 0;

lpr_out_clk:
	omap2_disp_put_dss();
	return -1;
}

static void omap2_disp_lpr_disable(void)
{
	unsigned long flags;

	omap2_disp_get_dss();

	/* restore DSS  divisors */
	dispc_reg_merge(DISPC_DIVISOR, rmode.divisors,
			DISPC_DIVISOR_LCD | DISPC_DIVISOR_PCD);

	/* split FIFOs and restore FIFO thresholds */
	dispc_reg_merge(DISPC_CONFIG, 0, DISPC_CONFIG_FIFOMERGE);

	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD, rmode.fifo_thrs,
			(DISPC_GFX_FIFO_THRESHOLD_HIGH |
			 DISPC_GFX_FIFO_THRESHOLD_LOW));

	/* Restore timings. */
	dispc_reg_merge(DISPC_TIMING_H, rmode.timing_h, (DISPC_TIMING_H_HBP |
							 DISPC_TIMING_H_HFP |
							 DISPC_TIMING_H_HSW));

	dispc_reg_merge(DISPC_TIMING_V, rmode.timing_v, (DISPC_TIMING_V_VBP |
							 DISPC_TIMING_V_VFP |
							 DISPC_TIMING_V_VSW));

	/* TODO: adjust porches and pulses if bigger fps is not acceptable */

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_save_ctx(OMAP2_GRAPHICS);
	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);

	omap2_disp_save_ctx(OMAP2_VIDEO1);
	omap2_disp_save_ctx(OMAP2_VIDEO2);

	spin_unlock_irqrestore(&dss_lock, flags);

	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);

	omap2_disp_put_dss();

	lpr.state = DISPLAY_LPR_STATE_OFF;
}

void omap2_disp_handle_lpr_auto_mode(void)
{
	int lpr_possible;

	if (!graphics_in_use)
		return;

	if (DISPLAY_LPR_MODE_AUTO != lpr.mode)
		return;

	/* In order to be able to configure LPR, only the GRAPHICS layer can be
	 * active and the VIDEO layers must be inactive.
	 */
	lpr_possible =	omap2_disp_is_video_layer_enabled(OMAP2_GRAPHICS) &&
			layer[OMAP2_GRAPHICS].in_use &&
			!layer[OMAP2_VIDEO1].in_use  &&
			!layer[OMAP2_VIDEO2].in_use;

	DPRINTK("DISPLAY: LPR mode is %spossible.\n", lpr_possible ? "" : "NOT ");
	if (lpr_possible) {
		if (!lpr.cfg_valid) {
			/* LPR is possible but no valid configuration has been
			 * set. Bail.
			 */
			return;
		}

		if (DISPLAY_LPR_STATE_OFF == lpr.state) {
			DPRINTK("DISPLAY: Auto enabling LPR\n");
			omap2_disp_lpr_enable();
		}
	}
	else {
		if (DISPLAY_LPR_STATE_ON == lpr.state) {
			DPRINTK("DISPLAY: Auto disabling LPR\n");
			omap2_disp_lpr_disable();
		}
	}
}

int omap2_disp_lpr_set_state(int state)
{
	int rc = -1;

	mutex_lock(&lpr_mutex);

	if (lpr.mode == DISPLAY_LPR_MODE_AUTO) {
		printk(KERN_WARNING "DISPLAY: Trying to set LPR state "
			"while in AUTO mode.\n");
		goto out_err;
	}

	/* Cannot enable lpr if DSS is inactive */
	if (!graphics_in_use) {
		printk(KERN_WARNING "DISPLAY: LPR change to '%s' attempted while "
					"display is OFF.\n",
					DISPLAY_LPR_STATE_ON == state ? "on" : "off");
		goto out_err;
	}

	if (lpr.state == state) {
		printk(KERN_WARNING "DISPLAY: LPR already '%s'. Not changing.\n",
				state == DISPLAY_LPR_STATE_ON ? "on" : "off");
		goto out_err;
	}

	switch (state) {
	case DISPLAY_LPR_STATE_ON:
		if (!lpr.cfg_valid) {
			printk(KERN_WARNING "DISPLAY: LPR enable attempt without "
				"valid configuration.\n");
			goto out_err;
		}

		rc = omap2_disp_lpr_enable();
		if (rc)
			goto out_err;
		break;

	case  DISPLAY_LPR_STATE_OFF:
		omap2_disp_lpr_disable();
		break;

	default:
		printk(KERN_WARNING "DISPLAY: Invalid state (%d) requested "
			"in %s.\n", state, __FUNCTION__);
		goto out_err;
	}

	rc = 0;
out_err:
	mutex_unlock(&lpr_mutex);
	return rc;
}

int omap2_disp_lpr_set_mode(int mode)
{
	mutex_lock(&lpr_mutex);

	if (lpr.mode == mode) {
		printk(KERN_WARNING "DISPLAY: LPR mode already '%s'. Not changing.\n",
			mode == DISPLAY_LPR_MODE_MANUAL ? "manual" : "auto");

		mutex_unlock(&lpr_mutex);
		return -1;
	}

	lpr.mode = mode;

	/* If we switch to auto mode we need to check if we need to adjust LPR
	 * mode.
	 */
	if (DISPLAY_LPR_MODE_AUTO == mode) {
		omap2_disp_handle_lpr_auto_mode();
	}

	mutex_unlock(&lpr_mutex);
	return 0;
}

void omap2_disp_lpr_set_cfg(const struct omap_dss_config *lpr_cfg_on,
			    const struct omap_dss_config *lpr_cfg_off)
{
	if (!lpr_cfg_on && !lpr_cfg_off)
		return;

	lpr.on        = *lpr_cfg_on;
	lpr.off       = *lpr_cfg_off;
	lpr.cfg_valid = 1;
}

/******************************************************************************/

/* Start before devices */
subsys_initcall(omap2_disp_init);

EXPORT_SYMBOL(omap2_disp_request_layer);
EXPORT_SYMBOL(omap2_disp_release_layer);

EXPORT_SYMBOL(omap2_disp_disable_layer);
EXPORT_SYMBOL(omap2_disp_is_video_layer_enabled);
EXPORT_SYMBOL(omap2_disp_enable_layer);
EXPORT_SYMBOL(omap2_disp_config_vlayer);
EXPORT_SYMBOL(omap2_disp_config_gfxlayer);
EXPORT_SYMBOL(omap2_disp_start_vlayer);
EXPORT_SYMBOL(omap2_disp_start_gfxlayer);

EXPORT_SYMBOL(omap2_disp_set_bg_color);
EXPORT_SYMBOL(omap2_disp_get_bg_color);
EXPORT_SYMBOL(omap2_disp_set_dithering);
EXPORT_SYMBOL(omap2_disp_get_dithering);
EXPORT_SYMBOL(omap2_disp_get_panel_size);
EXPORT_SYMBOL(omap2_disp_set_panel_size);
EXPORT_SYMBOL(omap2_disp_disable_output_dev);
EXPORT_SYMBOL(omap2_disp_enable_output_dev);
EXPORT_SYMBOL(omap2_disp_set_tvstandard);
EXPORT_SYMBOL(omap2_disp_get_tvstandard);
EXPORT_SYMBOL(omap2_disp_set_tvlcd);
EXPORT_SYMBOL(omap2_disp_get_tvlcd);
EXPORT_SYMBOL(omap2_disp_config_lcd);
EXPORT_SYMBOL(omap2_disp_lcdcfg_polfreq);
EXPORT_SYMBOL(omap2_disp_set_pcd);

EXPORT_SYMBOL(omap2_disp_set_dma_params);
EXPORT_SYMBOL(omap2_disp_get_output_dev);
EXPORT_SYMBOL(omap2_disp_set_output_dev);

#ifdef CONFIG_ARCH_OMAP34XX
EXPORT_SYMBOL(omap2_disp_set_alphablend);
EXPORT_SYMBOL(omap2_disp_get_alphablend);
EXPORT_SYMBOL(omap2_disp_set_global_alphablend_value);
EXPORT_SYMBOL(omap2_disp_get_global_alphablend_value);
#endif

EXPORT_SYMBOL(omap2_disp_set_dssfclk);
EXPORT_SYMBOL(omap2_disp_get_dss);
EXPORT_SYMBOL(omap2_disp_put_dss);

EXPORT_SYMBOL(omap2_disp_set_default_colorconv);
EXPORT_SYMBOL(omap2_disp_set_colorconv);
EXPORT_SYMBOL(omap2_disp_set_colorkey);
EXPORT_SYMBOL(omap2_disp_get_colorkey);
EXPORT_SYMBOL(omap2_disp_enable_colorkey);
EXPORT_SYMBOL(omap2_disp_disable_colorkey);
EXPORT_SYMBOL(omap2_disp_reg_sync_bit);

EXPORT_SYMBOL(omap2_disp_set_gfx_palette);
EXPORT_SYMBOL(omap2_disp_pixels_per_clock);

EXPORT_SYMBOL(omap2_disp_set_vrfb);
EXPORT_SYMBOL(omap2_disp_save_initstate);
EXPORT_SYMBOL(omap2_disp_restore_initstate);
EXPORT_SYMBOL(omap2_disp_reg_sync);
EXPORT_SYMBOL(omap2_disp_reg_sync_done);
EXPORT_SYMBOL(omap2_disp_disable);

EXPORT_SYMBOL(omap2_disp_register_isr);
EXPORT_SYMBOL(omap2_disp_unregister_isr);
EXPORT_SYMBOL(omap2_disp_irqenable);
EXPORT_SYMBOL(omap2_disp_irqdisable);

EXPORT_SYMBOL(current_colorconv_values);
