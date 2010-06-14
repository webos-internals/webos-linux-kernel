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

void omap24xx_ll_config_tv_clocks(int sleep_state);

#ifdef CONFIG_ARCH_OMAP34XX
extern int lpr_enabled;
#endif

/* usage count for DSS power management */
static int disp_usage;
#ifndef CONFIG_ARCH_OMAP3410
static int omap2_current_tvstandard = NTSC_M;
#endif
static spinlock_t dss_lock;
short int current_colorconv_values[2][3][3];
static struct omap_dss_regs dss_ctx;
static struct clk *dss1f_scale;
static struct tvlcd_status_t tvlcd_status;

static struct clk *dss1f, *dss1i;

struct omap2_disp_dma_params {
	u32 ba0;
	u32 ba1;
	int row_inc;
	int pix_inc;
};

static struct layer_t {
	int output_dev;
	int in_use;
	int ctx_valid;

	/* one set of dma parameters each for LCD and TV */
	struct omap2_disp_dma_params dma[2];

	int size_x;
	int size_y;
} layer[DSS_CTX_NUMBER] = {
	{
	.ctx_valid = 0,}, {
	.ctx_valid = 0,}, {
	.ctx_valid = 0,}, {
	.ctx_valid = 0,}, {
.ctx_valid = 0,},};

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

/* GFX FIFO thresholds */
#define RMODE_GFX_FIFO_HIGH_THRES	0x3FC
#define RMODE_GFX_FIFO_LOW_THRES	0x3BC

/*
 * DSS register I/O routines
 */
static __inline__ u32 dss_reg_in(u32 offset)
{
	return omap_readl(DSS_REG_BASE + DSS_REG_OFFSET + offset);
}
static __inline__ u32 dss_reg_out(u32 offset, u32 val)
{
	omap_writel(val, DSS_REG_BASE + DSS_REG_OFFSET + offset);
	return val;
}
static __inline__ u32 dss_reg_merge(u32 offset, u32 val, u32 mask)
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

static void set_colorconv(int v, int colorspace)
{
	unsigned long ccreg;
	short int mtx[3][3];
	int i, j;
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			mtx[i][j] = current_colorconv_values[v][i][j];
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
}

static void update_colorconv_mtx(int v, const short int mtx[3][3])
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
 * controller registers.  Each coefficient is a signed 8-bit integer in the
 * range [-128, 127] except for the middle coefficient (vc[1][i] and hc[3][i])
 * which is an unsigned 8-bit integer in the range [0, 255].  The first index of
 * the matrix is the coefficient number (0 to 2 vertical or 0 to 4 horizontal)
 * and the second index is the phase (0 to 7).
 */
static void
omap2_disp_set_resize(int v, const short int vc[][8], const short int hc[5][8], int ratio)
{
	int i;
	unsigned long reg;

	for (i = 0; i < 8; i++) {
		if(ratio <= 2){
			reg = (hc[4][i] & 0xff) | ((hc[3][i] & 0xff) << 8)
				| ((hc[2][i] & 0xff) << 16) | ((hc[1][i] & 0xff) << 24);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v, i), reg);
			// printk("H%d - 0x%X\n", i, reg);

			reg = (hc[0][i] & 0xff) | ((vc[2][i] & 0xff) << 8)
				| ((vc[1][i] & 0xff) << 16) | ((vc[0][i] & 0xff) << 24);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);
			// printk("HV%d - 0x%X\n", i, reg);
		} else {
			reg = ((hc[1][i] & 0xff) << 24) | ((hc[2][i] & 0xff) << 16)
				 | ((hc[3][i] & 0xff) << 8) | (hc[4][i] & 0xff);
			dispc_reg_out(DISPC_VID_FIR_COEF_H(v, i), reg);
			// printk("H%d - 0x%X\n", i, reg);

			reg = (hc[0][i] & 0xff) | ((vc[3][i] & 0xff) << 8)
				| ((vc[2][i] & 0xff) << 16) | ((vc[1][i] & 0xff) << 24);
			dispc_reg_out(DISPC_VID_FIR_COEF_HV(v, i), reg);
			// printk("HV%d - 0x%X\n", i, reg);

			reg =  ((vc[0][i] & 0xff) << 8) | (vc[4][i] & 0xff);
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
		*width = 1 + ((size & DISPC_SIZE_LCD_PPL)
			      >> DISPC_SIZE_LCD_PPL_SHIFT);
		*height = 1 + ((size & DISPC_SIZE_LCD_LPP)
			       >> DISPC_SIZE_LCD_LPP_SHIFT);
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
	}
}

static int graphics_in_use;

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

/*
 * Save the DSS state before doing a GO LCD/DIGITAL
 */

void omap2_disp_save_ctx(int ltype)
{
	int v1 = 0, v2 = 1;
	struct omap24xx_dispc_regs *dispc = &dss_ctx.dispc;

	switch (ltype) {
	case OMAP_DSS_GENERIC:
		dss_ctx.sysconfig = dss_reg_in(DSS_SYSCONFIG);
		dss_ctx.control = dss_reg_in(DSS_CONTROL);
#ifdef CONFIG_ARCH_OMAP3430
		dss_ctx.sdi_control = dss_reg_in(DSS_SDI_CONTROL);
		dss_ctx.pll_control = dss_reg_in(DSS_PLL_CONTROL);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc->revision = dispc_reg_in(DISPC_REVISION);
		dispc->sysconfig = dispc_reg_in(DISPC_SYSCONFIG);
		dispc->sysstatus = dispc_reg_in(DISPC_SYSSTATUS);
		dispc->irqstatus = dispc_reg_in(DISPC_IRQSTATUS);
		dispc->irqenable = dispc_reg_in(DISPC_IRQENABLE);
		dispc->control = dispc_reg_in(DISPC_CONTROL);
		dispc->config = dispc_reg_in(DISPC_CONFIG);
		dispc->capable = dispc_reg_in(DISPC_CAPABLE);
		dispc->default_color0 = dispc_reg_in(DISPC_DEFAULT_COLOR0);
		dispc->default_color1 = dispc_reg_in(DISPC_DEFAULT_COLOR1);
		dispc->trans_color0 = dispc_reg_in(DISPC_TRANS_COLOR0);
		dispc->trans_color1 = dispc_reg_in(DISPC_TRANS_COLOR1);
		dispc->line_status = dispc_reg_in(DISPC_LINE_STATUS);
		dispc->line_number = dispc_reg_in(DISPC_LINE_NUMBER);
		dispc->data_cycle1 = dispc_reg_in(DISPC_DATA_CYCLE1);
		dispc->data_cycle2 = dispc_reg_in(DISPC_DATA_CYCLE2);
		dispc->data_cycle3 = dispc_reg_in(DISPC_DATA_CYCLE3);
		dispc->timing_h = dispc_reg_in(DISPC_TIMING_H);
		dispc->timing_v = dispc_reg_in(DISPC_TIMING_V);
		dispc->pol_freq = dispc_reg_in(DISPC_POL_FREQ);
		dispc->divisor = dispc_reg_in(DISPC_DIVISOR);
		dispc->size_lcd = dispc_reg_in(DISPC_SIZE_LCD);
		dispc->size_dig = dispc_reg_in(DISPC_SIZE_DIG);

	case OMAP2_GRAPHICS:
		dispc->gfx_ba0 = dispc_reg_in(DISPC_GFX_BA0);
		dispc->gfx_ba1 = dispc_reg_in(DISPC_GFX_BA1);
		dispc->gfx_position = dispc_reg_in(DISPC_GFX_POSITION);
		dispc->gfx_size = dispc_reg_in(DISPC_GFX_SIZE);
		dispc->gfx_attributes = dispc_reg_in(DISPC_GFX_ATTRIBUTES);
		dispc->gfx_fifo_size = dispc_reg_in(DISPC_GFX_FIFO_SIZE);
		dispc->gfx_fifo_threshold =
		    dispc_reg_in(DISPC_GFX_FIFO_THRESHOLD);
		dispc->gfx_row_inc = dispc_reg_in(DISPC_GFX_ROW_INC);
		dispc->gfx_pixel_inc = dispc_reg_in(DISPC_GFX_PIXEL_INC);
		dispc->gfx_window_skip = dispc_reg_in(DISPC_GFX_WINDOW_SKIP);
		dispc->gfx_table_ba = dispc_reg_in(DISPC_GFX_TABLE_BA);
		break;

	case OMAP2_VIDEO1:
		dispc->vid1_ba0 = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_ba1 = dispc_reg_in(DISPC_VID_BA0(v1));
		dispc->vid1_position = dispc_reg_in(DISPC_VID_POSITION(v1));
		dispc->vid1_size = dispc_reg_in(DISPC_VID_SIZE(v1));
		dispc->vid1_attributes = dispc_reg_in(DISPC_VID_ATTRIBUTES(v1));
		dispc->vid1_fifo_size = dispc_reg_in(DISPC_VID_FIFO_SIZE(v1));
		dispc->vid1_fifo_threshold =
		    dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v1));
		dispc->vid1_row_inc = dispc_reg_in(DISPC_VID_ROW_INC(v1));
		dispc->vid1_pixel_inc = dispc_reg_in(DISPC_VID_PIXEL_INC(v1));
		dispc->vid1_fir = dispc_reg_in(DISPC_VID_FIR(v1));
		dispc->vid1_accu0 = dispc_reg_in(DISPC_VID_ACCU0(v1));
		dispc->vid1_accu1 = dispc_reg_in(DISPC_VID_ACCU1(v1));
		dispc->vid1_picture_size =
		    dispc_reg_in(DISPC_VID_PICTURE_SIZE(v1));
		dispc->vid1_fir_coef_h0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 0));
		dispc->vid1_fir_coef_h1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 1));
		dispc->vid1_fir_coef_h2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 2));
		dispc->vid1_fir_coef_h3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 3));
		dispc->vid1_fir_coef_h4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 4));
		dispc->vid1_fir_coef_h5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 5));
		dispc->vid1_fir_coef_h6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 6));
		dispc->vid1_fir_coef_h7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v1, 7));
		dispc->vid1_fir_coef_hv0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 0));
		dispc->vid1_fir_coef_hv1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 1));
		dispc->vid1_fir_coef_hv2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 2));
		dispc->vid1_fir_coef_hv3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 3));
		dispc->vid1_fir_coef_hv4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 4));
		dispc->vid1_fir_coef_hv5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 5));
		dispc->vid1_fir_coef_hv6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 6));
		dispc->vid1_fir_coef_hv7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v1, 7));
		dispc->vid1_conv_coef0 = dispc_reg_in(DISPC_VID_CONV_COEF0(v1));
		dispc->vid1_conv_coef1 = dispc_reg_in(DISPC_VID_CONV_COEF1(v1));
		dispc->vid1_conv_coef2 = dispc_reg_in(DISPC_VID_CONV_COEF2(v1));
		dispc->vid1_conv_coef3 = dispc_reg_in(DISPC_VID_CONV_COEF3(v1));
		dispc->vid1_conv_coef4 = dispc_reg_in(DISPC_VID_CONV_COEF4(v1));
		break;

	case OMAP2_VIDEO2:
		dispc->vid2_ba0 = dispc_reg_in(DISPC_VID_BA0(v2));
		dispc->vid2_ba1 = dispc_reg_in(DISPC_VID_BA1(v2));
		dispc->vid2_position = dispc_reg_in(DISPC_VID_POSITION(v2));
		dispc->vid2_size = dispc_reg_in(DISPC_VID_SIZE(v2));
		dispc->vid2_attributes = dispc_reg_in(DISPC_VID_ATTRIBUTES(v2));
		dispc->vid2_fifo_size = dispc_reg_in(DISPC_VID_FIFO_SIZE(v2));
		dispc->vid2_fifo_threshold =
		    dispc_reg_in(DISPC_VID_FIFO_THRESHOLD(v2));
		dispc->vid2_row_inc = dispc_reg_in(DISPC_VID_ROW_INC(v2));
		dispc->vid2_pixel_inc = dispc_reg_in(DISPC_VID_PIXEL_INC(v2));
		dispc->vid2_fir = dispc_reg_in(DISPC_VID_FIR(v2));
		dispc->vid2_accu0 = dispc_reg_in(DISPC_VID_ACCU0(v2));
		dispc->vid2_accu1 = dispc_reg_in(DISPC_VID_ACCU1(v2));
		dispc->vid2_picture_size =
		    dispc_reg_in(DISPC_VID_PICTURE_SIZE(v2));
		dispc->vid2_fir_coef_h0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 0));
		dispc->vid2_fir_coef_h1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 1));
		dispc->vid2_fir_coef_h2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 2));
		dispc->vid2_fir_coef_h3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 3));
		dispc->vid2_fir_coef_h4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 4));
		dispc->vid2_fir_coef_h5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 5));
		dispc->vid2_fir_coef_h6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 6));
		dispc->vid2_fir_coef_h7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_H(v2, 7));
		dispc->vid2_fir_coef_hv0 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 0));
		dispc->vid2_fir_coef_hv1 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 1));
		dispc->vid2_fir_coef_hv2 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 2));
		dispc->vid2_fir_coef_hv3 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 3));
		dispc->vid2_fir_coef_hv4 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 4));
		dispc->vid2_fir_coef_hv5 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 5));
		dispc->vid2_fir_coef_hv6 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 6));
		dispc->vid2_fir_coef_hv7 =
		    dispc_reg_in(DISPC_VID_FIR_COEF_HV(v2, 7));
		dispc->vid2_conv_coef0 = dispc_reg_in(DISPC_VID_CONV_COEF0(v2));
		dispc->vid2_conv_coef1 = dispc_reg_in(DISPC_VID_CONV_COEF1(v2));
		dispc->vid2_conv_coef2 = dispc_reg_in(DISPC_VID_CONV_COEF2(v2));
		dispc->vid2_conv_coef3 = dispc_reg_in(DISPC_VID_CONV_COEF3(v2));
		dispc->vid2_conv_coef4 = dispc_reg_in(DISPC_VID_CONV_COEF4(v2));
		break;
	}
	layer[ltype].ctx_valid = 1;
}

void omap2_disp_save_initstate(int ltype)
{
	unsigned long flags;

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_save_ctx(ltype);
	spin_unlock_irqrestore(&dss_lock, flags);
}

/*
 *  NOte, that VENC registers are not restored here
 *  Note, that DISPC_CONTROL register is not restored here
 */
void omap2_disp_restore_ctx(int ltype)
{
	int v1 = 0, v2 = 1;
	struct omap24xx_dispc_regs *dispc = &dss_ctx.dispc;

	if (layer[ltype].ctx_valid == 0)
		return;

	switch (ltype) {
	case OMAP_DSS_GENERIC:
		dss_reg_out(DSS_SYSCONFIG, dss_ctx.sysconfig);
		dss_reg_out(DSS_CONTROL, dss_ctx.control);
#ifdef CONFIG_ARCH_OMAP3430
		dss_reg_out(DSS_SDI_CONTROL, dss_ctx.sdi_control);
		dss_reg_out(DSS_PLL_CONTROL, dss_ctx.pll_control);
#endif
		break;

	case OMAP_DSS_DISPC_GENERIC:
		dispc_reg_out(DISPC_SYSCONFIG, dispc->sysconfig);
		dispc_reg_out(DISPC_IRQENABLE, dispc->irqenable);
		dispc_reg_out(DISPC_CONFIG, dispc->config);
		dispc_reg_out(DISPC_DEFAULT_COLOR0, dispc->default_color0);
		dispc_reg_out(DISPC_DEFAULT_COLOR1, dispc->default_color1);
		dispc_reg_out(DISPC_TRANS_COLOR0, dispc->trans_color0);
		dispc_reg_out(DISPC_TRANS_COLOR1, dispc->trans_color1);
		dispc_reg_out(DISPC_LINE_NUMBER, dispc->line_number);
		dispc_reg_out(DISPC_DATA_CYCLE1, dispc->data_cycle1);
		dispc_reg_out(DISPC_DATA_CYCLE2, dispc->data_cycle2);
		dispc_reg_out(DISPC_DATA_CYCLE3, dispc->data_cycle3);
		dispc_reg_out(DISPC_TIMING_H, dispc->timing_h);
		dispc_reg_out(DISPC_TIMING_V, dispc->timing_v);
		dispc_reg_out(DISPC_POL_FREQ, dispc->pol_freq);
		dispc_reg_out(DISPC_DIVISOR, dispc->divisor);
		dispc_reg_out(DISPC_SIZE_LCD, dispc->size_lcd);
		dispc_reg_out(DISPC_SIZE_DIG, dispc->size_dig);
		break;

	case OMAP2_GRAPHICS:
		dispc_reg_out(DISPC_GFX_BA0, dispc->gfx_ba0);
		dispc_reg_out(DISPC_GFX_BA1, dispc->gfx_ba1);
		dispc_reg_out(DISPC_GFX_POSITION, dispc->gfx_position);
		dispc_reg_out(DISPC_GFX_SIZE, dispc->gfx_size);
		dispc_reg_out(DISPC_GFX_ATTRIBUTES, dispc->gfx_attributes);
		dispc_reg_out(DISPC_GFX_FIFO_SIZE, dispc->gfx_fifo_size);
		dispc_reg_out(DISPC_GFX_FIFO_THRESHOLD,
			      dispc->gfx_fifo_threshold);
		dispc_reg_out(DISPC_GFX_ROW_INC, dispc->gfx_row_inc);
		dispc_reg_out(DISPC_GFX_PIXEL_INC, dispc->gfx_pixel_inc);
		dispc_reg_out(DISPC_GFX_WINDOW_SKIP, dispc->gfx_window_skip);
		dispc_reg_out(DISPC_GFX_TABLE_BA, dispc->gfx_table_ba);

		break;

	case OMAP2_VIDEO1:
		dispc_reg_out(DISPC_VID_BA0(v1), dispc->vid1_ba0);
		dispc_reg_out(DISPC_VID_BA1(v1), dispc->vid1_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v1), dispc->vid1_position);
		dispc_reg_out(DISPC_VID_SIZE(v1), dispc->vid1_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v1), dispc->vid1_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v1),
			      dispc->vid1_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v1), dispc->vid1_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v1), dispc->vid1_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v1), dispc->vid1_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v1), dispc->vid1_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v1), dispc->vid1_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v1),
			      dispc->vid1_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 0),
			      dispc->vid1_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 1),
			      dispc->vid1_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 2),
			      dispc->vid1_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 3),
			      dispc->vid1_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 4),
			      dispc->vid1_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 5),
			      dispc->vid1_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 6),
			      dispc->vid1_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v1, 7),
			      dispc->vid1_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 0),
			      dispc->vid1_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 1),
			      dispc->vid1_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 2),
			      dispc->vid1_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 3),
			      dispc->vid1_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 4),
			      dispc->vid1_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 5),
			      dispc->vid1_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 6),
			      dispc->vid1_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v1, 7),
			      dispc->vid1_fir_coef_hv7);

		dispc_reg_out(DISPC_VID_CONV_COEF0(v1), dispc->vid1_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v1), dispc->vid1_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v1), dispc->vid1_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v1), dispc->vid1_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v1), dispc->vid1_conv_coef4);
		break;

	case OMAP2_VIDEO2:
		dispc_reg_out(DISPC_VID_BA0(v2), dispc->vid2_ba0);
		dispc_reg_out(DISPC_VID_BA1(v2), dispc->vid2_ba1);
		dispc_reg_out(DISPC_VID_POSITION(v2), dispc->vid2_position);
		dispc_reg_out(DISPC_VID_SIZE(v2), dispc->vid2_size);
		dispc_reg_out(DISPC_VID_ATTRIBUTES(v2), dispc->vid2_attributes);
		dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v2),
			      dispc->vid2_fifo_threshold);
		dispc_reg_out(DISPC_VID_ROW_INC(v2), dispc->vid2_row_inc);
		dispc_reg_out(DISPC_VID_PIXEL_INC(v2), dispc->vid2_pixel_inc);
		dispc_reg_out(DISPC_VID_FIR(v2), dispc->vid2_fir);
		dispc_reg_out(DISPC_VID_ACCU0(v2), dispc->vid2_accu0);
		dispc_reg_out(DISPC_VID_ACCU1(v2), dispc->vid2_accu1);
		dispc_reg_out(DISPC_VID_PICTURE_SIZE(v2),
			      dispc->vid2_picture_size);

		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 0),
			      dispc->vid2_fir_coef_h0);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 1),
			      dispc->vid2_fir_coef_h1);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 2),
			      dispc->vid2_fir_coef_h2);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 3),
			      dispc->vid2_fir_coef_h3);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 4),
			      dispc->vid2_fir_coef_h4);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 5),
			      dispc->vid2_fir_coef_h5);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 6),
			      dispc->vid2_fir_coef_h6);
		dispc_reg_out(DISPC_VID_FIR_COEF_H(v2, 7),
			      dispc->vid2_fir_coef_h7);

		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 0),
			      dispc->vid2_fir_coef_hv0);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 1),
			      dispc->vid2_fir_coef_hv1);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 2),
			      dispc->vid2_fir_coef_hv2);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 3),
			      dispc->vid2_fir_coef_hv3);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 4),
			      dispc->vid2_fir_coef_hv4);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 5),
			      dispc->vid2_fir_coef_hv5);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 6),
			      dispc->vid2_fir_coef_hv6);
		dispc_reg_out(DISPC_VID_FIR_COEF_HV(v2, 7),
			      dispc->vid2_fir_coef_hv7);

		dispc_reg_out(DISPC_VID_CONV_COEF0(v2), dispc->vid2_conv_coef0);
		dispc_reg_out(DISPC_VID_CONV_COEF1(v2), dispc->vid2_conv_coef1);
		dispc_reg_out(DISPC_VID_CONV_COEF2(v2), dispc->vid2_conv_coef2);
		dispc_reg_out(DISPC_VID_CONV_COEF3(v2), dispc->vid2_conv_coef3);
		dispc_reg_out(DISPC_VID_CONV_COEF4(v2), dispc->vid2_conv_coef4);
		break;
	}
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
static void omap2_synclost_isr(void *arg, struct pt_regs *regs)
{
	int i = 0;

	printk(KERN_WARNING "Sync Lost LCD %x\n",
	       dispc_reg_in(DISPC_IRQSTATUS));
	arg = NULL;
	regs = NULL;

	/*
	 * Disable and Clear all the interrupts before we start
	 */
	dispc_reg_out(DISPC_IRQENABLE, 0x00000000);
	dispc_reg_out(DISPC_IRQSTATUS, 0x0000FFFF);

	/* disable the display controller */
	omap2_disp_disable(HZ / 5);

	/*
	 * Update the state of the display controller.
	 */
#if 0
	dss_ctx.dispc.sysconfig &= ~DISPC_SYSCONFIG_SOFTRESET;
	dss_ctx.dispc.control &= ~(DISPC_CONTROL_GODIGITAL |
				   DISPC_CONTROL_GOLCD);
#endif
	/* TV is disabled when LPR is enabled */
	dss_ctx.dispc.control |= DISPC_CONTROL_GOLCD;
#ifdef CONFIG_ARCH_OMAP34XX
	if (!lpr_enabled)
#endif
		dss_ctx.dispc.control |= DISPC_CONTROL_GODIGITAL;

	dispc_reg_out(DISPC_SYSCONFIG, DISPC_SYSCONFIG_SOFTRESET);
	while (!(dispc_reg_in(DISPC_SYSSTATUS) & DISPC_SYSSTATUS_RESETDONE)) {
		udelay(100);
		if (i++ > 5) {
			printk(KERN_WARNING
			       "Failed to soft reset the DSS !! \n");
			break;
		}
	}

	/* Configure the VENC for the default standard */
#ifndef CONFIG_ARCH_OMAP3410
	if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) ||
	    (omap2_disp_get_output_dev(OMAP2_VIDEO1) == OMAP2_OUTPUT_TV) ||
	    (omap2_disp_get_output_dev(OMAP2_VIDEO2) == OMAP2_OUTPUT_TV)) {
		omap2_disp_set_tvstandard(NTSC_M);
	}
#endif

	/* Restore the registers */
	omap2_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_restore_ctx(OMAP2_GRAPHICS);
	omap2_disp_restore_ctx(OMAP2_VIDEO1);
	omap2_disp_restore_ctx(OMAP2_VIDEO2);

	/* enable the display controller */
	if (layer[OMAP_DSS_DISPC_GENERIC].ctx_valid)
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);

	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);
	if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) ||
	    (omap2_disp_get_output_dev(OMAP2_VIDEO1) == OMAP2_OUTPUT_TV) ||
	    (omap2_disp_get_output_dev(OMAP2_VIDEO2) == OMAP2_OUTPUT_TV)) {
		omap2_disp_reg_sync(OMAP2_OUTPUT_TV);
	}
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

	      skip:dispc_reg_out(DISPC_GFX_WINDOW_SKIP,
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


	/* vertical resizing matrix */
	// bi-linear algorithm
	const static short int vc_u[3][8] =
	{ {   0,  16, 32, 48,  0,   0,  0,   0 },
	  { 128, 112, 96, 80, 64,  80, 96, 112 },
	  {   0,   0,  0,  0, 64,  48, 32,  16 } };
/*
	// TI's original algorithm
	{ {   0,   3,  12,  32,   0,   7,   5,   2 },
	  {128, 123, 111, 89, 64, 89, 111, 123},
	  {   0,   2,   5,   7,  64,  32,  12,   3 } };
*/

	/* horizontal resizing matrix */
	// bi-linear algorithm
	const static short int hc_u[5][8] =
	{ { 0,    0,   0,   0,   0,  0,  0,   0 },
	{ 0,   16,  32,  48,   0,  0,  0,   0 },
	{ 128, 112, 96,  80,  64, 80, 96, 112 },
	{ 0,     0,  0,   0,  64, 48, 32,  16 },
	{ 0,    0,   0,   0,   0,  0,  0,   0 }	};
/*
	// TI's original algorithm
	{ {   0,  -1,  -2,  -5,   0,  -2,  -1,   0 },
	  {0, 13, 30, 51, -9, -11, -11, -8},
	  {128, 124, 112, 95, 73, 95, 112, 124},
	  {0, -8, -11, -11, 73, 51, 30, 13},
	  {   0,   0,  -1,  -2,  -9,  -5,  -2,  -1 } };
*/
	const static short int vc_d[3][8] =
		{ {   36,   40,  45,  50,   18,   23,   27,   31 },
		  {   56,   57,  56,  55,   55,   55,   56,   57 },
		  {   36,   31,  27,  23,   55,   50,   45,   40 } };

	const static short int vc_d_fivetap[5][8] =
		{ {0,   4,  8, 12, -9, -7, -5, -2},  //VC22
			{36, 40, 44,  48, 17, 22, 27, 31},  //VC2
			{56, 55, 54,  53, 52, 53, 54, 55},  //VC1
			{36, 31, 27,  22, 51, 48, 44, 40},  //VC0
			{0,  -2, -5,  -7, 17, 12,  8,  4}}; //VC00

	/* horizontal resizing matrix */
	const static short int hc_d[5][8] =
		{ {   0,  4,  8,  12,   -9,  -7,  -5,  -2 },
		  {   36,  40,  44,  48,  17, 22, 27, 31 },
		  { 56, 55, 54,  53,  52,  53, 54, 55 },
		  {   36,  31, 27, 22,  51,  48,  44,  40 },
		  {   0,   -2,  -5,  -7,  17,  12,  8,  4 } };

	short int* vc= (short int *) vc_u;
	short int* hc= (short int *) hc_u;

	int v;

	if (ltype == OMAP2_VIDEO1)
		v = 0;
	else if (ltype == OMAP2_VIDEO2)
		v = 1;
	else
		return;

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
			vid_attributes |=
			    DISPC_VID_ATTRIBUTES_VIDCOLORCONVENABLE;
		} else {
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDFORMAT_UYVY;
			vid_attributes |=
			    DISPC_VID_ATTRIBUTES_VIDCOLORCONVENABLE;
		}

		if (mirroring == 1 || rotation_deg >= 0) {
			/*
			 * ps      - In VRFB space the pixel size for YUYV/UYVY is 4 bytes
			 * vr_ps - Actual pixel size for YUYV/UYVY  is 2 bytes
			 */
			ps = 4;
			vr_ps = 2;
		}
		if (rotation_deg >= 0) {
			if (mirroring == 1) {
				vid_attributes |= (rotation_deg == 90) ?
				    ((0x3) << DISPC_VID_ATTRIBUTES_VIDROT) :
				    (rotation_deg ==
				     270) ? ((0x1) <<
					     DISPC_VID_ATTRIBUTES_VIDROT)
				    : (rotation_deg ==
				       0) ? (0x2 << DISPC_VID_ATTRIBUTES_VIDROT)
				    : (0 << DISPC_VID_ATTRIBUTES_VIDROT);
			} else {
				vid_attributes |= (rotation_deg == 90) ?
				    ((0x3) << DISPC_VID_ATTRIBUTES_VIDROT) :
				    (rotation_deg ==
				     270) ? ((0x1) <<
					     DISPC_VID_ATTRIBUTES_VIDROT)
				    : ((rotation_deg /
					90) << DISPC_VID_ATTRIBUTES_VIDROT);
			}
			vid_attributes |= (rotation_deg == 90
					   || rotation_deg ==
					   270) ? (1 <<
						   DISPC_VID_ATTRIBUTES_VIDROWREPEAT)
			    : (0 << DISPC_VID_ATTRIBUTES_VIDROWREPEAT);
		}
		if (mirroring == 1 && rotation_deg == -1) {
			vid_attributes |= (0x2 << DISPC_VID_ATTRIBUTES_VIDROT);
		}

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

	/* Set FIFO threshold to 0xFF (high) and 0xFF - (16x4bytes) = 0xC0 (low) */
	// dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v),0x00FF00C0);
	dispc_reg_out(DISPC_VID_FIFO_THRESHOLD(v), 0x03FC03BC);

	if (rotation_deg == 90 || rotation_deg == 270) {
		winheight = win->w.width;
		winwidth = win->w.height;
		cropheight = crop->width;
		cropwidth = crop->height;
		pixheight = pix->width;
		pixwidth = pix->height;
		cleft = crop->top;
		ctop = crop->left;
	} else {
		winwidth = win->w.width;
		winheight = win->w.height;
		cropwidth = crop->width;
		cropheight = crop->height;
		pixheight = pix->height;
		pixwidth = pix->width;
		ctop = crop->top;
		cleft = crop->left;
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

	if(winheight != 0) {
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
			hc = (short int *) hc_d;
		}
	}
	if (winheight != cropheight) {
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_VRESIZE;
		if (winheight < cropheight){
			vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVRESIZECONF;
			if(ratio <= 2){
				vc = (short int *) vc_d;
			}else{
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS;
				vc = (short int *) vc_d_fivetap;
			}
		}
	}

	if (flicker_filter == 1) {
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDRESIZEENABLE_VRESIZE;
		vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVRESIZECONF;
		if(ratio <= 2){
				vc = (short int *) vc_d;
		}else{
				vid_attributes |= DISPC_VID_ATTRIBUTES_VIDVERTICALTAPS;
				vc = (short int *) vc_d_fivetap;
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

#ifndef CONFIG_ARCH_OMAP3410
void omap24xx_ll_config_tv_clocks(int sleep_state)
{
	static int got_clocks = 0;
	static struct clk *tv_clk;
	static int disabled = 0;
	static int enabled  = 0;

	if (!got_clocks) {
#if defined (CONFIG_MACH_OMAP_2430SDP)
		tv_clk = clk_get(NULL, "dss_54m_fck");
#endif
#if defined (CONFIG_MACH_OMAP_3430SDP)
		tv_clk = clk_get(NULL, "dss_tv_fck");
#endif

#if defined(CONFIG_MACH_BRISKET)
		tv_clk = clk_get(NULL, "dss_54m_fck");
#endif

#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
		tv_clk = clk_get(NULL, "dss_tv_fck");
#endif

		if (IS_ERR(tv_clk)) {
			printk("\n UNABLE to get dss TV fclk \n");
			return;
		}
		got_clocks = 1;
	}

	if (sleep_state == 1) {
		if (!disabled) {
			clk_disable(tv_clk);
			disabled = 1;
		}
		enabled = 0;
	} else {
		if (!enabled) {
			if (clk_enable(tv_clk) != 0) {
				printk("\n UNABLE to enable dss TV fclk \n");
				return;
			}
			enabled = 1;
		}
		disabled = 0;
	}
}
#endif

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
		omap24xx_ll_config_tv_clocks(1);
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
	u32 config = 0;
	u32 gfx_attributes = 0, gfx_fifo_threshold = 0, gfx_format = 0;
	u32 gfx_position = 0, gfx_window_skip = 0, gfx_size = 0;

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

	gfx_fifo_threshold =
	    (RMODE_GFX_FIFO_HIGH_THRES << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
	    (RMODE_GFX_FIFO_LOW_THRES << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);

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

	dispc_reg_out(DISPC_GFX_FIFO_THRESHOLD, gfx_fifo_threshold);
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
void
omap2_disp_config_lcd(u32 clkdiv, u32 hbp, u32 hfp, u32 hsw,
		      u32 vbp, u32 vfp, u32 vsw, u32 bpp)
{
	u32 control, divisor, timing_h, timing_v;
	u32 ctl_col_depth;

	divisor = (1 << DISPC_DIVISOR_LCD_SHIFT)
	    | (clkdiv << DISPC_DIVISOR_PCD_SHIFT);

	// printk("HARRRI:config_lcd: clkdiv=0x%x, hbp=0x%x,hfp=0x%x,hsw=0x%x,vbp=0x%x,vfp=0x%x,vsw=0x%x \n",clkdiv,hbp,hfp,hsw,vbp,vfp,vsw);

	if (hbp > 255)
		hbp = 255;
	if (hfp > 255)
		hfp = 255;
	if (hsw > 63)
		hsw = 63;
	if (vbp > 255)
		vbp = 255;
	if (vfp > 255)
		vfp = 255;
	if (vsw > 63)
		vsw = 63;

	switch (bpp) {
		case 24:
		case 32: ctl_col_depth = DISPC_CONTROL_TFTDATALINES_OALSB24B; break;
		case 16:
		default: ctl_col_depth = DISPC_CONTROL_TFTDATALINES_OALSB16B; break;
	}

	timing_h =
	    (hbp << DISPC_TIMING_H_HBP_SHIFT) | (hfp <<
						 DISPC_TIMING_H_HFP_SHIFT)
	    | (hsw << DISPC_TIMING_H_HSW_SHIFT);
	timing_v =
	    (vbp << DISPC_TIMING_V_VBP_SHIFT) | (vfp <<
						 DISPC_TIMING_V_VFP_SHIFT)
	    | (vsw << DISPC_TIMING_V_VSW_SHIFT);

	dispc_reg_out(DISPC_TIMING_H, timing_h);
	dispc_reg_out(DISPC_TIMING_V, timing_v);
	dispc_reg_out(DISPC_DIVISOR, divisor);
	control = dispc_reg_in(DISPC_CONTROL);
	control |= DISPC_CONTROL_GPOUT1 | DISPC_CONTROL_GPOUT0 |
		   ctl_col_depth | DISPC_CONTROL_STNTFT;

	dispc_reg_out(DISPC_CONTROL, control);
#if 0
	printk("contr %8x\n", dispc_reg_in(DISPC_CONTROL));
	printk("TIMINH_H 0x%x \n", dispc_reg_in(DISPC_TIMING_H));
	printk("TIMINH_V 0x%x \n", dispc_reg_in(DISPC_TIMING_V));
	printk("DIVISOR 0x%x \n", dispc_reg_in(DISPC_DIVISOR));
#endif
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
#ifdef CONFIG_MACH_OMAP_3430SDP
	static int LCD_pixel_clk = 26000000;	/* to get more bandwidth */
#else
	static int LCD_pixel_clk = 10000000;	/* to get more bandwidth */
#endif
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

	sup_clkrate = clk_round_rate(dss1f_scale, ask_clkrate);
	if (is_sil_rev_less_than(OMAP3430_REV_ES2_0)) {
		if (clk_get_rate(dss1f_scale) == 96000000) {
			/*96M already, dont do anything for ES 1.0 */
			return;
		}
	} else {
		for (i = 1; i <= 20; i++) {
			sup_clkrate = clk_round_rate(dss1f_scale, ask_clkrate);
			if (sup_clkrate >= tgt_clkrate)
				break;
			ask_clkrate = ask_clkrate + 1000000;
		}
		if (clk_set_rate(dss1f_scale, sup_clkrate) == -EINVAL)
			printk(KERN_ERR "Unable to set the DSS"
			       "functional clock to %d\n", sup_clkrate);
	}
	return;
}

static void omap24xx_ll_config_disp_clocks(int sleep_state)
{
	struct device *dev = NULL;
	static int got_clocks = 0;

	if (!got_clocks) {
		omap2_disp_set_dssfclk();
		dss1i = clk_get(dev,"dss_ick");
#if defined(CONFIG_ARCH_OMAP24XX)
		dss1f = clk_get(dev,"dss1_fck");
#else		
		dss1f = clk_get(dev,"dss1_alwon_fck");
#ifdef CONFIG_OMAP_DSI
		dss2f = clk_get(dev,"dss2_alwon_fck");
#endif
#endif
		if(IS_ERR(dss1i) || IS_ERR(dss1f)) {
			printk("Could not get DSS clocks\n");
			return;
		}
		got_clocks = 1;
	}

	if(sleep_state == 1){
		clk_disable(dss1i);
		clk_disable(dss1f);
#ifdef CONFIG_OMAP_DSI
		clk_disable(dss2f);
#endif
	} else {
		if(clk_enable(dss1i) != 0) {
			printk("Unable to enable DSS ICLK\n");
			return;
		}
		if(clk_enable(dss1f) != 0) {
			printk("Unable to enable DSS FCLK\n");
			return;
		}
#ifdef CONFIG_OMAP_DSI
		if(clk_enable(dss2f) != 0) {
			printk("Unable to enable DSS FCLK\n");
			return;
		}
#endif
	}
}

/* This function must be called by any driver that needs to use the display
 * controller before calling any routine that accesses the display controller
 * registers. It increments the count of the number of users of the display
 * controller, and turns the clocks ON only when required.
 */
void omap2_disp_get_all_clks(void)
{
	u32 idle_dispc;
#ifdef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	if (0 == disp_usage) {
		/* turn on DSS clock */
		omap24xx_ll_config_disp_clocks(0);
#ifndef CONFIG_ARCH_OMAP3410
		omap2_disp_set_tvref(TVREF_ON);
		omap24xx_ll_config_tv_clocks(0);
#endif
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* Set the TV standard first */
#ifndef CONFIG_ARCH_OMAP3410
		omap2_disp_set_tvstandard(omap2_current_tvstandard);
#endif
		/* restore dss context */
		omap2_disp_restore_ctx(OMAP_DSS_GENERIC);
		omap2_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
		omap2_disp_restore_ctx(OMAP2_GRAPHICS);
		omap2_disp_restore_ctx(OMAP2_VIDEO1);
		omap2_disp_restore_ctx(OMAP2_VIDEO2);

#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifdef CONFIG_HW_SUP_TRANS
		/* Set smart idle for Display subsystem */
		idle_dss = dss_reg_in(DSS_SYSCONFIG);
		idle_dss |= DSS_SYSCONFIG_AUTOIDLE;
		dss_reg_out(DSS_SYSCONFIG, idle_dss);
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		/* Set smart idle, autoidle for Display controller */
		idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE |
				DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
		idle_dispc |= (DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
			       DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
			       DISPC_SYSCONFIG_ENABLE_WKUP);
		idle_dispc |= DISPC_SYSCONFIG_AUTOIDLE;
#else
		idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
		    DISPC_SYSCONFIG_SIDLEMODE_NIDLE;
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);
#ifdef CONFIG_OMAP34XX_OFFMODE
		dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
	}
#ifndef CONFIG_ARCH_OMAP3410
	else {
		/* enable the TV clocks, since we are not if they are */
		omap2_disp_set_tvref(TVREF_ON);
		omap24xx_ll_config_tv_clocks(0);
		{
			omap2_disp_set_tvstandard(omap2_current_tvstandard);
		}
	}
#endif
	disp_usage++;
	spin_unlock(&dss_lock);
}

/* This function must be called by a driver when it not going to use the
 * display controller anymore. E.g., when a driver suspends, it must call
 * omap2_disp_put_dss. When it wakes up, it must call omap2_disp_get_dss again.
 * It decrements the count of the number of users of the display
 * controller, and turns the clocks OFF when not required.
 */
void omap2_disp_put_all_clks(void)
{
#ifndef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifndef CONFIG_HW_SUP_TRANS */


	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		printk(KERN_ERR "trying to put DSS when usage count is zero\n");
		spin_unlock(&dss_lock);
		return;
	}

	disp_usage--;

	if (disp_usage == 0) {
#ifdef CONFIG_OMAP34XX_OFFMODE
		/* save dss context */
		omap2_disp_save_ctx(OMAP_DSS_GENERIC);
		omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
		omap2_disp_save_ctx(OMAP2_GRAPHICS);
		omap2_disp_save_ctx(OMAP2_VIDEO1);
		omap2_disp_save_ctx(OMAP2_VIDEO2);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifndef CONFIG_HW_SUP_TRANS
		idle_dss = dispc_reg_in(DISPC_SYSCONFIG);
		idle_dss &=
		    ~(DISPC_SYSCONFIG_MIDLEMODE | DISPC_SYSCONFIG_SIDLEMODE);
		idle_dss |=
		    DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
		    DISPC_SYSCONFIG_SIDLEMODE_SIDLE;
		dispc_reg_out(DISPC_SYSCONFIG, idle_dss);
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

		omap2_disp_disable(HZ / 5);
		/* turn off TV clocks */
#ifndef CONFIG_ARCH_OMAP3410
		omap24xx_ll_config_tv_clocks(1);
		omap2_disp_set_tvref(TVREF_OFF);
#endif
		mdelay(4);

		omap24xx_ll_config_disp_clocks(1);
	}
	spin_unlock(&dss_lock);
}

/* This function must be called by any driver that needs to use the display
 * controller before calling any routine that accesses the display controller
 * registers. It increments the count of the number of users of the display
 * controller, and turns the clocks ON only when required.
 */
void omap2_disp_get_dss(void)
{
	u32 idle_dispc;
#ifdef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	if (0 != disp_usage++) {
		/* Display already set up, nothing to do here */
		spin_unlock(&dss_lock);
		return;
	}

	/* turn on DSS clock */
	omap24xx_ll_config_disp_clocks(0);

#ifndef CONFIG_ARCH_OMAP3410
	if ((omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) ||
	    (omap2_disp_get_output_dev(OMAP2_VIDEO1)   == OMAP2_OUTPUT_TV) ||
	    (omap2_disp_get_output_dev(OMAP2_VIDEO2)   == OMAP2_OUTPUT_TV)) {
		omap2_disp_set_tvref(TVREF_ON);
		omap24xx_ll_config_tv_clocks(0);
#ifdef CONFIG_OMAP34XX_OFFMODE
		omap2_disp_set_tvstandard(omap2_current_tvstandard);
#endif
	}
#endif

#ifdef CONFIG_OMAP34XX_OFFMODE
	/* restore dss context */
	omap2_disp_restore_ctx(OMAP_DSS_GENERIC);
	omap2_disp_restore_ctx(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_restore_ctx(OMAP2_GRAPHICS);
	omap2_disp_restore_ctx(OMAP2_VIDEO1);
	omap2_disp_restore_ctx(OMAP2_VIDEO2);
#endif

#ifdef CONFIG_HW_SUP_TRANS
	/* Set Autoidle for Display subsystem */
	idle_dss = dss_reg_in(DSS_SYSCONFIG);
	idle_dss |= DSS_SYSCONFIG_AUTOIDLE;
	dss_reg_out(DSS_SYSCONFIG, idle_dss);
#endif

	/* Set smart idle, autoidle for Display controller */
	idle_dispc = dispc_reg_in(DISPC_SYSCONFIG);
	idle_dispc &= ~(DISPC_SYSCONFIG_MIDLEMODE |
			DISPC_SYSCONFIG_SIDLEMODE);

#ifdef CONFIG_HW_SUP_TRANS
	idle_dispc |= (DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
		       DISPC_SYSCONFIG_SIDLEMODE_SIDLE |
		       DISPC_SYSCONFIG_ENABLE_WKUP);
	idle_dispc |= DISPC_SYSCONFIG_AUTOIDLE;
#else
	idle_dispc |= DISPC_SYSCONFIG_MIDLEMODE_NSTANDBY |
		      DISPC_SYSCONFIG_SIDLEMODE_NIDLE;
#endif
	dispc_reg_out(DISPC_SYSCONFIG, idle_dispc);

#ifdef CONFIG_OMAP34XX_OFFMODE
	dispc_reg_out(DISPC_CONTROL, dss_ctx.dispc.control);
#endif

	spin_unlock(&dss_lock);
}

/* This function must be called by a driver when it not going to use the
 * display controller anymore. E.g., when a driver suspends, it must call
 * omap2_disp_put_dss. When it wakes up, it must call omap2_disp_get_dss again.
 * It decrements the count of the number of users of the display
 * controller, and turns the clocks OFF when not required.
 */
void omap2_disp_put_dss(void)
{
#ifndef CONFIG_HW_SUP_TRANS
	u32 idle_dss;
#endif				/* #ifndef CONFIG_HW_SUP_TRANS */

	spin_lock(&dss_lock);
	if (disp_usage == 0) {
		printk(KERN_ERR "trying to put DSS when usage count is zero\n");
		spin_unlock(&dss_lock);
		return;
	}

	if (--disp_usage) {
		/* Not the last user. Nothing to do. */
		spin_unlock(&dss_lock);
		return;
	}

#ifdef CONFIG_OMAP34XX_OFFMODE
	/* save dss context */
	omap2_disp_save_ctx(OMAP_DSS_GENERIC);
	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_save_ctx(OMAP2_GRAPHICS);
	omap2_disp_save_ctx(OMAP2_VIDEO1);
	omap2_disp_save_ctx(OMAP2_VIDEO2);
#endif				/* #ifdef CONFIG_OMAP34XX_OFFMODE */
#ifndef CONFIG_HW_SUP_TRANS
	idle_dss = dispc_reg_in(DISPC_SYSCONFIG);
	idle_dss &= ~(DISPC_SYSCONFIG_MIDLEMODE | DISPC_SYSCONFIG_SIDLEMODE);
	idle_dss |= DISPC_SYSCONFIG_MIDLEMODE_SSTANDBY |
		    DISPC_SYSCONFIG_SIDLEMODE_SIDLE;
	dispc_reg_out(DISPC_SYSCONFIG, idle_dss);
#endif				/* #ifdef CONFIG_HW_SUP_TRANS */

	omap2_disp_disable(HZ / 5);

#ifndef CONFIG_ARCH_OMAP3410
	// printk("put_dss: Disable TV clocks \n");
	omap24xx_ll_config_tv_clocks(1);
	omap2_disp_set_tvref(TVREF_OFF);
#endif
	mdelay(4);
	omap24xx_ll_config_disp_clocks(1);

	spin_unlock(&dss_lock);
}

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
}

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

/* This is used to dynamically switch the output of a particular layer to
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
	omap2_disp_enable_layer(OMAP2_GRAPHICS);
}

/*---------------------------------------------------------------------------*/

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
	u32 i = 0;
	struct clk *dss_tv_fck;
#ifdef CONFIG_MACH_OMAP_2430SDP
	dss_tv_fck = clk_get(NULL, "dss_54m_fck");
#endif
#ifdef CONFIG_MACH_BRISKET
	dss_tv_fck = clk_get(NULL, "dss_54m_fck");
#endif
#ifdef CONFIG_MACH_OMAP_3430SDP
	dss_tv_fck = clk_get(NULL, "dss_tv_fck");
#endif
#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
	dss_tv_fck = clk_get(NULL, "dss_tv_fck");
#endif

	if (IS_ERR(dss_tv_fck)) {
		printk("\n UNABLE to get dss TV fclk \n");
		return;
	}

	/* Enable VENC clock  */
	if (clk_enable(dss_tv_fck) != 0) {
		printk("\n UNABLE to enable dss TV fclk \n");
		return;
	}

	/*
	 * Write 1 to the 8th bit of the F_Control register to reset the VENC
	 */
	venc_reg_merge(VENC_F_CONTROL, VENC_FCONTROL_RESET,
		       VENC_FCONTROL_RESET);

	/* wait for reset to complete */
	while ((venc_reg_in(VENC_F_CONTROL) & VENC_FCONTROL_RESET) ==
	       0x00000100) {
		udelay(10);
		if (i++ > 10)
			break;
	}

	if (venc_reg_in(VENC_F_CONTROL) & VENC_FCONTROL_RESET) {
		printk(KERN_WARNING
		       "omap2_disp: timeout waiting for venc reset\n");

		/* remove the reset */
		venc_reg_merge(VENC_F_CONTROL, (0 << 8), VENC_FCONTROL_RESET);
	}

	/* disable the VENC clock */
	clk_disable(dss_tv_fck);
}
#endif

/*
 * Enables an IRQ in DSPC_IRQENABLE.
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
	return -EBUSY;
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
	unsigned long dispc_irqstatus = dispc_reg_in(DISPC_IRQSTATUS);
	int i;

	for (i = 0; i < MAX_ISR_NR; i++) {
		if (registered_isr[i].isr == NULL)
			continue;
		if (registered_isr[i].mask & dispc_irqstatus)
			registered_isr[i].isr(registered_isr[i].arg, regs);
	}

	/* ack the interrupt */
	dispc_reg_out(DISPC_IRQSTATUS, dispc_irqstatus);

	return IRQ_HANDLED;
}

int __init omap2_disp_init(void)
{
	int rev, i;
	u32 dss_control;

	spin_lock_init(&dss_lock);

	/* Required for scale call */
#if defined(CONFIG_ARCH_OMAP24XX)	
	dss1f_scale = clk_get(NULL,"dss1_fck");
	printk("omap2_disp_init : dss1fscale = 0x%x \n",dss1f_scale);
#else
	dss1f_scale = clk_get(NULL,"dss1_alwon_fck");
	printk("omap2_disp_init : dss1fscale = %p\n",dss1f_scale);
#endif	
	if(IS_ERR(dss1f_scale)) {
		printk("Could not get DSS1 FCLK\n");
		return PTR_ERR(dss1f_scale);
	}

	omap2_disp_get_all_clks();

	/* disable the display controller */
	omap2_disp_disable(HZ / 5);

	rev = dss_reg_in(DSS_REVISION);
	printk(KERN_INFO "OMAP Display hardware version %d.%d\n",
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

	/* Should be replaced by FPGA register read  ADD A 2420 ifdef here */

#ifdef CONFIG_ARCH_OMAP2420
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif

#ifdef CONFIG_MACH_OMAP_2430SDP
#ifdef CONFIG_TWL4030_CORE_T2
	dss_control |= (DSS_CONTROL_TV_REF | DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif

#ifdef CONFIG_TWL4030_CORE_M1
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE);
#endif
#endif

#ifdef CONFIG_MACH_OMAP_3430SDP
	/* enabling S-video connector for 3430 SDP */
#ifndef CONFIG_ARCH_OMAP3410
	dss_control |= (DSS_CONTROL_DAC_DEMEN | DSS_CONTROL_TV_REF |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE |
			DSS_CONTROL_VENC_OUT);
#else
	dss_control |= (DSS_CONTROL_DAC_DEMEN |
			DSS_CONTROL_VENC_CLOCK_4X_ENABLE |
			DSS_CONTROL_VENC_OUT);
#endif
#endif

	dss_control &= ~DSS_CONTROL_VENC_CLOCK_MODE;
	dss_reg_out(DSS_CONTROL, dss_control);

	/* By default, all layers go to LCD */
	layer[OMAP2_GRAPHICS].output_dev = OMAP2_OUTPUT_LCD;
	layer[OMAP2_VIDEO1].output_dev = OMAP2_OUTPUT_LCD;
	layer[OMAP2_VIDEO2].output_dev = OMAP2_OUTPUT_LCD;

	/*
	 * Set the default color conversion parameters for Video pipelines
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

	if (request_irq
	    (INT_24XX_DSS_IRQ, (void *)omap2_disp_master_isr, IRQF_SHARED,
	     "OMAP2 Display", registered_isr)) {
		printk(KERN_WARNING "omap2_disp: request_irq failed\n");
		omap2_disp_irq = 0;
	} else {
		omap2_disp_irq = 1;
		for (i = 0; i < MAX_ISR_NR; i++) {
			registered_isr[i].isr = NULL;
			registered_isr[i].mask = 0;
		}
		/* Clear all the pending interrupts, if any */
		dispc_reg_out(DISPC_IRQSTATUS, 0xFFFFFFFF);
		omap2_disp_register_isr(omap2_synclost_isr, layer,
					DISPC_IRQSTATUS_SYNCLOST);
	}

	omap2_disp_register_isr(omap2_synclost_isr, layer,
				DISPC_IRQSTATUS_SYNCLOST);
	omap2_disp_put_all_clks();

	return 0;

}

#ifdef CONFIG_ARCH_OMAP34XX

/*
 * optimal precalculated frequency for DSS1_ALWON_CLK. The freq is optimal
 * in the sense that there is a divider that gives pixel clock rate closest to
 * the lowest possible pixel clock rate for a given panel
 */
#ifdef CONFIG_FB_OMAP_LCD_VGA
#define LPR_DSS1_ALWON_FCLK_OPT_FREQ	96000000
#define LPR_DSS_LOGIC_DIV		2
#define LPR_HBP				79
#define LPR_HFP				59
#define LPR_HSW				2
#define LPR_VBP				0
#define LPR_VFP				0
#define LPR_VSW				1
#define LPR_GFX_FIFO_LOW_THRES		0x99C
#define LPR_GFX_FIFO_HIGH_THRES		0xB9C
#else
/* These values are validated only on 3430 ES2 */
/* Might not work on 3430 ES1 */
#define LPR_DSS1_ALWON_FCLK_OPT_FREQ	108000000
#define LPR_DSS_LOGIC_DIV		9
#define LPR_HBP				31
#define LPR_HFP				37
#define LPR_HSW				3
#define LPR_VBP				8
#define LPR_VFP				2
#define LPR_VSW				1
#define LPR_GFX_FIFO_LOW_THRES		0x7F8
#define LPR_GFX_FIFO_HIGH_THRES		0xB9C
#endif				/* CONFIG_FB_OMAP_LCD_VGA */

#define LPR_DEFAULT_FPS	60

#ifdef CONFIG_MACH_OMAP_3430SDP

#endif				/* CONFIG_MACH_OMAP_3430SDP */

struct dss_run_mode {
	int rate;
	int divisors;
	int timing_h;
	int timing_v;
};

static struct dss_run_mode rmode;
int lpr_enabled;

/*
 * if set, the lowest dss logic frequency to achieve given(lowest) pixel clock
 * frequency is set. Otherwise 'lpr_lowest_dss_logic_freq' value is used.
 * The flag is set by default and can be reset via sysfs interface
 */
int lpr_lowest_dss_logic_freq_enabled = 1;

/*
 * dss logic freq. initially initialized to be equal to DSS1_ALWON_FCLK.
 * the value may be adjusted via sysfs interface if needed
 */
int lpr_dss_logic_freq = LPR_DSS1_ALWON_FCLK_OPT_FREQ;

/* FPS value in LPR. may be adjusted via sysfs interface */
int lpr_fps = LPR_DEFAULT_FPS;

static DECLARE_MUTEX(lpr_mutex);

/*
 * adjust porches and pulses to get desired FPS after pixel clock is adjusted
 * to LPR value
 */
static void lpr_set_fps(int fps)
{
	int timing_h, timing_v;

	if (lpr_lowest_dss_logic_freq_enabled) {

		/* adjust porches and pulses to get 60 FPS */
		timing_h = ((LPR_HBP - 1) << DISPC_TIMING_H_HBP_SHIFT) |
		    ((LPR_HFP - 1) << DISPC_TIMING_H_HFP_SHIFT) |
		    ((LPR_HSW - 1) << DISPC_TIMING_H_HSW_SHIFT);
		timing_v = (LPR_VBP << DISPC_TIMING_V_VBP_SHIFT) |
		    (LPR_VFP << DISPC_TIMING_V_VFP_SHIFT) |
		    (LPR_VSW << DISPC_TIMING_V_VSW_SHIFT);

		dispc_reg_merge(DISPC_TIMING_H, timing_h,
				(DISPC_TIMING_H_HBP |
				 DISPC_TIMING_H_HFP | DISPC_TIMING_H_HSW));

		dispc_reg_merge(DISPC_TIMING_V, timing_v,
				(DISPC_TIMING_V_VBP |
				 DISPC_TIMING_V_VFP | DISPC_TIMING_V_VSW));

	}
	return;
}

/**
 * lpr_set_dss_logic_freq - chose lcd div and pcd based on user input
 *
 * If lpr_lowest_dss_logic_freq_enabled flag is set (default) the routine sets
 * dss logic frequency equal to twice pixel clock frequency. If the default
 * setup does not work for an application (dss logic frequency equal to twice
 * pixel clock frequency is not enough) user can affect the settings using
 * sysfs interface:
 * if lpr_lowest_dss_logic_freq_enabled flag is reset the logic frequency is
 * set to user specified value (may be adjusted using sysfs interface).
 * Default value in this case is DSS1_ALWON_FCLK frequency.
 */
static int lpr_set_dss_logic_freq(void)
{
	int lcd_div, pcd;

	if (lpr_lowest_dss_logic_freq_enabled) {
		/*
		 * set pcd and lcd div to get lowest _both_ pixel clock freq
		 * _and_ logic frequency
		 */
		pcd = 2;
		lcd_div = LPR_DSS_LOGIC_DIV;
	} else {
		/* set requested DSS logic freq */
#if 0
		TODO, not tested !
		    lcd_div = LPR_DSS1_ALWON_FCLK_OPT_FREQ / lpr_dss_logic_freq;
		if (lcd_div < 1)
			return -EINVAL;

		logic_freq = LPR_DSS1_ALWON_FCLK_OPT_FREQ / lcd_div;

		/*
		 * (LPR_DSS1_ALWON_FCLK_OPT_FREQ / LPR_DSS_LOGIC_DIV) gives best
		 * minimal pixel clock freq for a panel
		 */
		pcd =
		    logic_freq / (LPR_DSS1_ALWON_FCLK_OPT_FREQ /
				  LPR_DSS_LOGIC_DIV);
		if (pcd < 1)
			return -EINVAL;
#else
		return -ENOTSUPP;
#endif				/* 0 */
	}

	dispc_reg_merge(DISPC_DIVISOR,
			(lcd_div << DISPC_DIVISOR_LCD_SHIFT) |
			(pcd << DISPC_DIVISOR_PCD_SHIFT),
			DISPC_DIVISOR_LCD | DISPC_DIVISOR_PCD);

	return 0;
}

/**
 * omap2_disp_lpr_enable - trigger Low Power Refresh mode
 *
 * TODO: desc of LPR
 */

int omap2_disp_lpr_enable(void)
{
	int dss1_rate;
	int gfx_fifo_thresholds;
	int rc = 0;
	int v_attr;
	int digitalen;
	unsigned long flags;

	/* Cannot enable lpr if DSS is inactive */
	if (!graphics_in_use)
		return -1;

	down(&lpr_mutex);

	if (lpr_enabled)
		goto lpr_out;
	/*
	 * Check whether LPR can be triggered
	 *   - gfx pipeline is routed to LCD
	 *   - both video pipelines are disabled (this allows FIFOs merge)
	 */

	/* may be more meaningful error code is required */
	if (omap2_disp_get_output_dev(OMAP2_GRAPHICS) != OMAP2_OUTPUT_LCD) {
		rc = -1;
		goto lpr_out;
	}

	omap2_disp_get_dss();

	v_attr = dispc_reg_in(DISPC_VID_ATTRIBUTES(0)) |
	    dispc_reg_in(DISPC_VID_ATTRIBUTES(1));

	if (v_attr & DISPC_VID_ATTRIBUTES_ENABLE) {
		rc = -1;
		goto lpr_out_clk;
	}

	/*
	 * currently DSS is running on DSS1 by default. just warn if it has
	 * changed in the future
	 */
	if (dss_reg_in(DSS_CONTROL) & DSS_CONTROL_APLL_CLK)
		BUG();

	/* save run mode rate */
	rmode.rate = dss1_rate = clk_get_rate(dss1f);

	digitalen = dispc_reg_in(DISPC_CONTROL) & DISPC_CONTROL_DIGITALENABLE;

	/* disable DSS before adjusting DSS clock */
	omap2_disp_disable(HZ / 5);

	/* set DSS1_ALWON_FCLK freq */
	rc = clk_set_rate(dss1f, LPR_DSS1_ALWON_FCLK_OPT_FREQ);
	if (rc != 0)
		goto lpr_out_clk_en;

	rmode.divisors = (dispc_reg_in(DISPC_DIVISOR) & (DISPC_DIVISOR_LCD |
							 DISPC_DIVISOR_PCD));

	rmode.timing_h = (dispc_reg_in(DISPC_TIMING_H) & (DISPC_TIMING_H_HBP |
							  DISPC_TIMING_H_HFP |
							  DISPC_TIMING_H_HSW));

	rmode.timing_v = (dispc_reg_in(DISPC_TIMING_V) & (DISPC_TIMING_V_VBP |
							  DISPC_TIMING_V_VFP |
							  DISPC_TIMING_V_VSW));

	/* chose lcd div and pcd based on user input */
	rc = lpr_set_dss_logic_freq();
	if (rc != 0)
		goto lpr_out_clk_en_rate;

	lpr_set_fps(lpr_fps);

	/* set up LPR default  FIFO thresholds */
	gfx_fifo_thresholds =
	    (LPR_GFX_FIFO_HIGH_THRES << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
	    (LPR_GFX_FIFO_LOW_THRES << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);

	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD, gfx_fifo_thresholds,
			(DISPC_GFX_FIFO_THRESHOLD_HIGH |
			 DISPC_GFX_FIFO_THRESHOLD_LOW));

	dispc_reg_merge(DISPC_CONFIG, DISPC_CONFIG_FIFOMERGE,
			DISPC_CONFIG_FIFOMERGE);

	/*
	 * save LPR configuration of DISPC and GFX register in case synclost
	 * happens during LPR. If synclost happens LPR parameters get reset
	 * to last-known-good LPR parameters
	 *
	 * may be useful to restore known-good parameters if FIFO underrun
	 * occurs as well
	 */
	omap2_disp_save_ctx(OMAP2_GRAPHICS);

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	if (digitalen)
		omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);

	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
	spin_unlock_irqrestore(&dss_lock, flags);

	/* let LPR settings to take effect */
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);

	lpr_enabled = 1;

	omap2_disp_put_dss();
	up(&lpr_mutex);

	return 0;

      lpr_out_clk_en_rate:
	clk_set_rate(dss1f, rmode.rate);

      lpr_out_clk_en:
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	if (digitalen)
		omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);
      lpr_out_clk:
	omap2_disp_put_dss();
      lpr_out:
	up(&lpr_mutex);
	return rc;
}

int omap2_disp_lpr_disable(void)
{
	int rc = 0;
	int gfx_fifo_thresholds;
	int digitalen;
	unsigned long flags;

	if (!graphics_in_use)
		return -1;

	down(&lpr_mutex);

	if (!lpr_enabled) {
		up(&lpr_mutex);
		return rc;
	}

	omap2_disp_get_dss();

	/* restore DSS  divisors */
	dispc_reg_merge(DISPC_DIVISOR, rmode.divisors,
			DISPC_DIVISOR_LCD | DISPC_DIVISOR_PCD);

	/* split FIFOs and restore FIFO thresholds */
	dispc_reg_merge(DISPC_CONFIG, 0, DISPC_CONFIG_FIFOMERGE);

	gfx_fifo_thresholds =
	    (RMODE_GFX_FIFO_HIGH_THRES << DISPC_GFX_FIFO_THRESHOLD_HIGH_SHIFT) |
	    (RMODE_GFX_FIFO_LOW_THRES << DISPC_GFX_FIFO_THRESHOLD_LOW_SHIFT);

	dispc_reg_merge(DISPC_GFX_FIFO_THRESHOLD, gfx_fifo_thresholds,
			(DISPC_GFX_FIFO_THRESHOLD_HIGH |
			 DISPC_GFX_FIFO_THRESHOLD_LOW));

	dispc_reg_merge(DISPC_TIMING_H, rmode.timing_h, (DISPC_TIMING_H_HBP |
							 DISPC_TIMING_H_HFP |
							 DISPC_TIMING_H_HSW));

	dispc_reg_merge(DISPC_TIMING_V, rmode.timing_v, (DISPC_TIMING_V_VBP |
							 DISPC_TIMING_V_VFP |
							 DISPC_TIMING_V_VSW));
	/* TODO: adjust porches and pulses if bigger fps is not acceptable */

	digitalen = dispc_reg_in(DISPC_CONTROL) & DISPC_CONTROL_DIGITALENABLE;

	/* disable DSS before adjusting DSS clock */
	omap2_disp_disable(HZ / 5);

	/* restore DSS run mode rate */
	rc = clk_set_rate(dss1f, rmode.rate);

	omap2_disp_save_ctx(OMAP2_GRAPHICS);

	spin_lock_irqsave(&dss_lock, flags);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	if (digitalen)
		omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);

	omap2_disp_save_ctx(OMAP_DSS_DISPC_GENERIC);
	spin_unlock_irqrestore(&dss_lock, flags);

	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);

	omap2_disp_put_dss();

	lpr_enabled = 0;

	up(&lpr_mutex);
	return rc;
}
#endif				/* CONFIG_ARCH_OMAP34XX */

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

EXPORT_SYMBOL(omap24xx_ll_config_disp_clocks);
EXPORT_SYMBOL(omap2_disp_set_dssfclk);
EXPORT_SYMBOL(omap2_disp_get_dss);
EXPORT_SYMBOL(omap2_disp_put_dss);

EXPORT_SYMBOL(omap2_disp_get_all_clks);
EXPORT_SYMBOL(omap2_disp_put_all_clks);

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
EXPORT_SYMBOL(omap24xx_ll_config_tv_clocks);
EXPORT_SYMBOL(omap2_disp_disable);

EXPORT_SYMBOL(omap2_disp_register_isr);
EXPORT_SYMBOL(omap2_disp_unregister_isr);
EXPORT_SYMBOL(omap2_disp_irqenable);
EXPORT_SYMBOL(omap2_disp_irqdisable);

EXPORT_SYMBOL(current_colorconv_values);
