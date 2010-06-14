/*
 * drivers/video/omap/ctrl_omap3.c
 *
 * driver for the omap controller initializations
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Author: john chen (jchen1996@gmail.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/clk.h>

#include <asm/arch/mcspi.h>
#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/display.h>
#include <asm/arch/lcd.h>

#include "lcd.h"

#define MOD_NAME 			"LCD Ctrl: "

#undef  MODDEBUG
//#define MODDEBUG    1

#ifdef  MODDEBUG
#define DPRINTK(format,...)\
	printk(KERN_INFO MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

/* Pix clocks */
#if defined(CONFIG_MACH_SIRLOIN)
#define LCD_PIXCLOCK_MAX	95133	// 10.512 MHz
#define LCD_PIXCLOCK_MIN	93744	// 10.667 MHz
#else
#define LCD_PIXCLOCK_MAX	185186	/* freq 5.4 MHz */
#define LCD_PIXCLOCK_MIN	138888	/* freq 7.2 MHz */
#endif

#define CONTROLLER_STATE_ON		1
#define CONTROLLER_STATE_OFF		0

static u32 boot_lcd_pixclock = 0;
static u32 boot_lcd_pixclock_min = 70000;
static u32 boot_lcd_pixclock_max = 93700;
static u32 boot_porch[6] = { 0, 0, 0, 0, 0, 0 };

static u32  lcd_pixclock_min = LCD_PIXCLOCK_MIN;
static u32  lcd_pixclock_max = LCD_PIXCLOCK_MAX;


static struct fb_var_screeninfo *panel_lcd_var;

extern void omap24xx_ll_config_tv_clocks(int sleep_state);
extern int omap24xx_get_dss1_clock(void);
extern int omap2_disp_is_video_layer_enabled(int ltype);
extern int omap2_disp_is_layer_used(int ltype);
extern void omap2_disp_handle_lpr_auto_mode(void);

struct omap_lcd_controller {
	struct device *parent;
	struct display_device *disp_dev;
	struct platform_device *pdev;

	struct mutex lock;
	int controller_state;

	struct fb_var_screeninfo panel_info;
	struct omap_dss_config dss_config_lpr_on;
	struct omap_dss_config dss_config_lpr_off;
	u32 use_sdi;
	u32 vsync_gated;

	u32 dss_reg_base;
	u32 dss_reg_offset;
	u32 dispc_reg_offset;

	u32 sdi_color_depth;
	u32 sdi_num_of_data_pairs;
	u32 sdi_pll_lock_method;
	u32 sdi_pll_to_pclk_ratio;
	u32 sdi_freq_selector;
	int (*sdi_power_enable)(int);
};

static struct omap_lcd_controller *localct = NULL;



static int  __init boot_lcd_args(char *str)
{
	u32 pixclk = 0;

	if (sscanf(str,"%u,%u,%u",&pixclk,&boot_lcd_pixclock_min,&boot_lcd_pixclock_max)>=1)
	{
		if ( pixclk != 0 &&
			 pixclk >= boot_lcd_pixclock_min &&
			 pixclk <= boot_lcd_pixclock_max ) {
			boot_lcd_pixclock = pixclk;
		}
	}
    return 0;
}
__setup("boot_lcd_pixclock=", boot_lcd_args);



static int  __init boot_lcd_porch_args(char *str)
{
	int i;

	if (sscanf(str,"%u,%u,%u,%u,%u,%u",&boot_porch[0],&boot_porch[1],&boot_porch[2],
			   &boot_porch[3],&boot_porch[4],&boot_porch[5])<6)
	{
		for (i=0;i<6;i++) {
			boot_porch[i]=0;
		}
	}
    return 0;
}
__setup("boot_lcd_porch=", boot_lcd_porch_args);


/*
 *   Exportable function for omapfb.c
 */
void get_panel_default_var(struct fb_var_screeninfo *var, int output_dev)
{
	DPRINTK("%s\n", __FUNCTION__);

	if (output_dev == OMAP2_OUTPUT_LCD) {
		memcpy((struct fb_var_screeninfo *)var,
		       panel_lcd_var, sizeof(*var));
	}
}

u32 get_panel_pixclock_max(int output_dev)
{
	DPRINTK("%s\n", __FUNCTION__);

	if (output_dev == OMAP2_OUTPUT_LCD) {
		return lcd_pixclock_max;
	} else if (output_dev == OMAP2_OUTPUT_TV) {
		return ~0;
	}

	return -EINVAL;
}

u32 get_panel_pixclock_min(int output_dev)
{
	DPRINTK("%s\n", __FUNCTION__);

	if (output_dev == OMAP2_OUTPUT_LCD) {
		return lcd_pixclock_min;
	} else if (output_dev == OMAP2_OUTPUT_TV) {
		return 0;
	}

	return -EINVAL;
}

void enable_backlight(void)
{
	DPRINTK("%s\n", __FUNCTION__);
}
void disable_backlight(void)
{
	DPRINTK("%s\n", __FUNCTION__);
}

EXPORT_SYMBOL(enable_backlight);
EXPORT_SYMBOL(disable_backlight);
EXPORT_SYMBOL(get_panel_default_var);
EXPORT_SYMBOL(get_panel_pixclock_max);
EXPORT_SYMBOL(get_panel_pixclock_min);

static inline u32 dss_reg_read(struct omap_lcd_controller *ct, u32 reg)
{
    return omap_readl(ct->dss_reg_base + ct->dss_reg_offset + (reg));
}
static inline void dss_reg_write(struct omap_lcd_controller *ct, u32 val, u32 reg)
{
    omap_writel((val), ct->dss_reg_base + ct->dss_reg_offset + (reg));
}

static inline u32 dispc_reg_read(struct omap_lcd_controller *ct, u32 reg)
{
    return omap_readl(ct->dss_reg_base + ct->dispc_reg_offset + (reg));
}
static inline void dispc_reg_write(struct omap_lcd_controller *ct, u32 val, u32 reg)
{
    omap_writel((val), ct->dss_reg_base + ct->dispc_reg_offset + (reg));
}

static void dss_init(struct omap_lcd_controller *ct)
{
	/* display controller programming */
	u32 val;
	u32 pixdiv;

	DPRINTK("%s +++++\n", __FUNCTION__);

	val = dispc_reg_read(ct, DISPC_CONFIG);
	val &= ~DISPC_CONFIG_LOADMODE_MASK;
	val |= DISPC_CONFIG_LOADMODE_FRDATLFFR;
	val |= DISPC_CONFIG_LCDALPHAENABLE;
	dispc_reg_write(ct, val, DISPC_CONFIG);

	/* configure LCD timing */
	pixdiv = ct->panel_info.pixclock / (1000000000UL /
					(omap24xx_get_dss1_clock() / 1000));

	DPRINTK("%s: dss1 clk = %d, pixdiv = 0x%x\n",
			__FUNCTION__, omap24xx_get_dss1_clock(), pixdiv);

	omap2_disp_config_lcd(pixdiv,
				ct->panel_info.left_margin - 1,	// hbp
				ct->panel_info.right_margin - 1,// hfp
				ct->panel_info.hsync_len - 1,	// hsw
				ct->panel_info.upper_margin,	// vbp
				ct->panel_info.lower_margin,	// vfp
				ct->panel_info.vsync_len - 1,	// vsw
				ct->panel_info.bits_per_pixel	// bpp
				);

	/* configure polarity */
	val = DISPC_POL_FREQ_IHS | DISPC_POL_FREQ_IVS |
	      DISPC_POL_FREQ_RF  | DISPC_POL_FREQ_ONOFF;
	dispc_reg_write(ct, val, DISPC_POL_FREQ);

	/* set the panel size */
	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD,
					ct->panel_info.xres,
					ct->panel_info.yres);

	/* setup the graphics layer */
	omap2_disp_config_gfxlayer(ct->panel_info.xres, ct->panel_info.yres, 32);
	dispc_reg_write(ct, 1, DISPC_GFX_ROW_INC);
	dispc_reg_write(ct, 1, DISPC_GFX_PIXEL_INC);

	val = DISPC_GLOBAL_ALPHA_GFX_GALPHA | DISPC_GLOBAL_ALPHA_VID2_GALPHA;
	dispc_reg_write(ct, val, DISPC_GLOBAL_ALPHA);

	/* Setup dispc_control */
	val =	DISPC_CONTROL_GPOUT0 |
		DISPC_CONTROL_GPOUT1 |
		DISPC_CONTROL_TFTDATALINES_OALSB24B |
		DISPC_CONTROL_STNTFT |
		DISPC_CONTROL_PCKFREEENABLE |
		DISPC_CONTROL_LCDENABLEPOL_ACTIVEHIGH;
	dispc_reg_write(ct, val, DISPC_CONTROL);
}

#ifdef CONFIG_ARCH_OMAP3430
/******************************************************************************/
/*
 * sdi start sequence
 */
static void sdi_init(struct omap_lcd_controller *ct, int m, int n, int freqsel)
{
	u32 val;

	/* color depth must be 24 bits for FL3G */
	val = dss_reg_read(ct, DSS_SDI_CONTROL);
	val &= ~DSS_SDI_CONTROL_BWSEL;
	val |= ct->sdi_color_depth;
	dss_reg_write(ct, val, DSS_SDI_CONTROL);

	/* set the lock method */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ~DSS_PLL_CONTROL_LOCKSEL;
	val |= ct->sdi_pll_lock_method;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	/* using 2 data pairs */
	/* Nominal data rate = nominal pixel clock * 15 bits/pair (2 pairs) */
	val = dss_reg_read(ct, DSS_SDI_CONTROL);
	val &= ~(DSS_SDI_CONTROL_PRSEL | DSS_SDI_CONTROL_PDIV);
	val |= ct->sdi_num_of_data_pairs |
		(ct->sdi_pll_to_pclk_ratio << DSS_SDI_CONTROL_PDIV_SHIFT);
	dss_reg_write(ct, val, DSS_SDI_CONTROL);

	/* set sdi parameters */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ~(DSS_PLL_CONTROL_REGM |
		 DSS_PLL_CONTROL_REGN |
		 DSS_PLL_CONTROL_FREQSEL_SHIFT);
	val |= (n << DSS_PLL_CONTROL_REGN_SHIFT) |
	       (m << DSS_PLL_CONTROL_REGM_SHIFT) |
	       (freqsel << DSS_PLL_CONTROL_FREQSEL_SHIFT);
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	/* select low power mode */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val |= DSS_PLL_CONTROL_PLLLPMODE;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);
}

static void sdi_power_up(struct omap_lcd_controller *ct)
{
	u32 val;

	/* Release SDI PLL reset */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val |= DSS_PLL_CONTROL_SYSRESET;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);
}
static void sdi_power_down(struct omap_lcd_controller *ct)
{
	u32 val;

	/* Assert SDI PLL reset */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ~DSS_PLL_CONTROL_SYSRESET;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);
}

static void sdi_start(struct omap_lcd_controller *ct)
{
	u32 val;
	int n;
	unsigned long pixdiv, pclk, ndiv, mdiv, pdiv;
	u32 pixclk = ct->panel_info.pixclock;

	/* Configure the PLL */
	pixdiv = pixclk / (1000000000 / (omap24xx_get_dss1_clock() / 1000));
	pclk = omap24xx_get_dss1_clock() / pixdiv;
	pdiv = ct->sdi_pll_to_pclk_ratio;
	ndiv = pclk / 2100000;
	mdiv = pdiv * (ndiv + 1);

	DPRINTK("%s: pixdiv=%lu pclk=%lu ndiv=%lu mdiv=%lu Fint=%lu.\n",
	     __FUNCTION__, pixdiv, pclk, ndiv, mdiv, pclk / (ndiv + 1));

	/* program the SDI initial configuration */
	/* Fint = 0x7 = 1.75Mhz - 2.1Mhz (optimal) */
	sdi_init(ct, mdiv, ndiv, ct->sdi_freq_selector);
	sdi_power_up(ct);

	/* Wait for PLL to idle */
	n = 10;
	while ((dss_reg_read(ct, DSS_SDI_STATUS) & DSS_SDI_STATUS_PLL_BUSYFLAG) && n--)
		msleep(1);

	/* Request SDI PLL to lock */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val |= DSS_PLL_CONTROL_GOBIT;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);


	n = 10;
	while ((dss_reg_read(ct, DSS_SDI_STATUS) & DSS_SDI_STATUS_PLL_BUSYFLAG) && n--)
		msleep(1);

	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ~DSS_PLL_CONTROL_GOBIT;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	n = 10;
	while ((!dss_reg_read(ct, DSS_SDI_STATUS) & DSS_SDI_STATUS_PLL_LOCK) && n--)
		msleep(1);


	/* Enable the SDI */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val |= DISPC_CONTROL_LCDENABLESIGNAL;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	n = 10;
	while ((!dss_reg_read(ct, DSS_SDI_STATUS) & DSS_SDI_STATUS_RESET_DONE) && n--)
		msleep(1);

	/* Wait wake-up time (2 ms) for the PLL of the serial receiver to lock */
	msleep(2);

	DPRINTK("%s: SDI start sequence complete.\n", __FUNCTION__);

}

static void sdi_stop(struct omap_lcd_controller *ct)
{
	u32 val;
	int n;

	/* disable the display controller */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_LCDENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* Wait for the FRAMEDONE IRQ to happen.
	 */
	n = 20;
	while (n-- &&
	       !(dispc_reg_read(ct, DISPC_IRQSTATUS) & DISPC_IRQSTATUS_FRAMEDONE)) {
		msleep(1);
	}
	if (!(dispc_reg_read(ct, DISPC_IRQSTATUS) & DISPC_IRQSTATUS_FRAMEDONE)) {
		printk(KERN_ERR MOD_NAME "Missing FRAMEDONE IRQ.\n");
	}

	/* Disable SDI */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_LCDENABLESIGNAL;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* disable the free running clock */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_PCKFREEENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* reset SDI PLL */
 	sdi_power_down(ct);

	DPRINTK("%s: SDI stop sequence complete.\n", __FUNCTION__);
}
#endif

static void omap_controller_init(struct omap_lcd_controller *ct)
{
	u32 val;

	DPRINTK("%s:\n", __FUNCTION__);

	/* Program the display controller configuration */

	/*
	 * VSYNC is used as the reset line.
	 * That line can't be configured as a GPIO so we need
	 * to control it using the DISPC_CONFIG VSYNCGATED bit */

	if (ct->use_sdi) {
#ifdef CONFIG_ARCH_OMAP3430
		val = DISPC_POL_FREQ_IHS | DISPC_POL_FREQ_IVS |
			DISPC_POL_FREQ_RF | DISPC_POL_FREQ_ONOFF;
		dispc_reg_write(ct, val, DISPC_POL_FREQ);

		if (ct->vsync_gated) {
			DPRINTK("Using Vsync as reset line\n");
			val = dispc_reg_read(ct, DISPC_CONFIG);
			val |= DISPC_CONFIG_VSYNCGATED;
			dispc_reg_write(ct, val, DISPC_CONFIG);
			mdelay(3);
		}


		val = DISPC_CONTROL_LCDENABLEPOL_ACTIVEHIGH
		    | DISPC_CONTROL_PCKFREEENABLE
		    | DISPC_CONTROL_GPOUT1
		    | DISPC_CONTROL_GPOUT0
		    | DISPC_CONTROL_TFTDATALINES_OALSB24B | DISPC_CONTROL_STNTFT;
		dispc_reg_write(ct, val, DISPC_CONTROL);

		sdi_start(ct);
#endif
	} else {

		if (ct->vsync_gated) {
			DPRINTK("Using Vsync as reset line\n");
			val = dispc_reg_read(ct, DISPC_CONFIG);
			val |= DISPC_CONFIG_VSYNCGATED;
			dispc_reg_write(ct, val, DISPC_CONFIG);
		}

		omap2_disp_lcdcfg_polfreq(1,	// horizontal sync active low
					  1,	// vertical sync active low
					  0,	// ACB
					  1,	// IPC
					  0	// ONOFF
	    	);

		if (ct->vsync_gated) {
			DPRINTK("Using Vsync as reset line\n");
			val = dispc_reg_read(ct, DISPC_CONFIG);
			val |= DISPC_CONFIG_VSYNCGATED;
			dispc_reg_write(ct, val, DISPC_CONFIG);
			mdelay(3);
		}

		val = dispc_reg_read(ct, DISPC_CONTROL);
		val |= (DISPC_CONTROL_GOLCD | DISPC_CONTROL_LCDENABLE);
		dispc_reg_write(ct, val, DISPC_CONTROL);

		/* wait for gocld to be reset */
		do {
			val = dispc_reg_read(ct, DISPC_CONTROL);
		} while (val & DISPC_CONTROL_GOLCD);
	}
}

static int lcd_controller_set_freq(void *dev, u32 pixclk)
{
#ifdef CONFIG_ARCH_OMAP3430
	u32 val;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	int n;
	unsigned long pixdiv, pclk, ndiv, mdiv, pdiv;

	DPRINTK("%s:\n", __FUNCTION__);

	if (!ct->use_sdi) {
		printk(KERN_ERR MOD_NAME
			"Error: Not supported in non-sdi mode\n");
		return 0;
	}

	/* disable the display controller */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_LCDENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* read the irq status framedone bit */
	for (n = 0, val = dispc_reg_read(ct, DISPC_IRQSTATUS);
	     (DISPC_IRQSTATUS_FRAMEDONE & val) && 10 > n;
	     ++n, val = dispc_reg_read(ct, DISPC_IRQSTATUS))
		msleep(1);

	DPRINTK("%s: FRAMEDONE loops=%d.\n", __FUNCTION__, n);

	/* disable the SDI , free running clock, and reset SDI PLL */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_LCDENABLESIGNAL;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_PCKFREEENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ~DSS_PLL_CONTROL_SYSRESET;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	DPRINTK("%s: Finished freq change part A\n", __FUNCTION__);
	/* --- completed sequence part A --- */

	/* disable all pipes */
	omap2_disp_disable_layer(OMAP2_GRAPHICS);
	omap2_disp_disable_layer(OMAP2_VIDEO1);
	omap2_disp_disable_layer(OMAP2_VIDEO2);

	/* hold all lines in inactive state */
	val = dispc_reg_read(ct, DISPC_CONFIG);
	val |= DISPC_CONFIG_VSYNCGATED | DISPC_CONFIG_HSYNCGATED |
		DISPC_CONFIG_PIXELDATAGATED | DISPC_CONFIG_ACBIASGATED ;
	dispc_reg_write(ct, val, DISPC_CONFIG);

	/* schedule an update of the new parameters for the next vertical
	   blanking period */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val |= DISPC_CONTROL_GOLCD;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	for (n = 0, val = dispc_reg_read(ct, DISPC_CONTROL);
	     !(DISPC_CONTROL_GOLCD & val) && 10 > n;
	     ++n, val = dispc_reg_read(ct, DISPC_CONTROL))
		msleep(1);

	DPRINTK("%s: GOLCD loops=%d.\n", __FUNCTION__, n);

	/* disable the SDI module */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_LCDENABLESIGNAL;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* set the stop mode */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val |= DSS_PLL_CONTROL_STOPMODE;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	/* stop the free running clock */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_PCKFREEENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* update new parameters */
	pixdiv = pixclk / (1000000000 / (omap24xx_get_dss1_clock() / 1000));
	pclk = omap24xx_get_dss1_clock() / pixdiv;
	pdiv = ct->sdi_pll_to_pclk_ratio;
	ndiv = pclk / 2100000;
	mdiv = pdiv * (ndiv + 1);

	DPRINTK("%s: pixdiv=%lu pclk=%lu ndiv=%lu mdiv=%lu Fint=%lu.\n",
	     __FUNCTION__, pixdiv, pclk, ndiv, mdiv, pclk / (ndiv + 1));

	/* program the SDI initial configuration */
	sdi_init(ct, mdiv, ndiv, ct->sdi_freq_selector); /* Fint = 0x7 = 1.75Mhz - 2.1Mhz (optimal) */


	DPRINTK("%s: new freq = %d dss1 clk = %d\n",
				__FUNCTION__, pixclk, omap24xx_get_dss1_clock());

	/* change the registers */
	omap2_disp_config_lcd(pixdiv,
				ct->panel_info.left_margin - 1,	// hbp
				ct->panel_info.right_margin - 1,// hfp
				ct->panel_info.hsync_len - 1,	// hsw
				ct->panel_info.upper_margin,	// vbp
				ct->panel_info.lower_margin,	// vfp
				ct->panel_info.vsync_len - 1,	// vsw
				ct->panel_info.bits_per_pixel	// bpp
				);


	ct->panel_info.pixclock=pixclk;

	if (pixclk < lcd_pixclock_min)
	{
		lcd_pixclock_min = (pixclk < 1001) ? 1:(pixclk-5000);
	}
	if (pixclk > lcd_pixclock_max)
	{
		lcd_pixclock_max =  pixclk + 5000;
	}

	/* read sdi status register */
	for (n = 0, val = dss_reg_read(ct, DSS_SDI_STATUS);
	     !(DSS_SDI_STATUS_PLL_LOCK & val) && 10 > n;
	     ++n, val = dss_reg_read(ct, DSS_SDI_STATUS))
		msleep(1);

	DPRINTK("%s: PLL stopped loops=%d.\n", __FUNCTION__, n);

	sdi_power_down(ct);
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	sdi_power_up(ct);

	/* schedule an update of the new parameters */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val |= DISPC_CONTROL_GOLCD;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	for (n = 0, val = dispc_reg_read(ct, DISPC_CONTROL);
	     !(DISPC_CONTROL_GOLCD & val) && 10 > n;
	     ++n, val = dispc_reg_read(ct, DISPC_CONTROL))
		msleep(1);

	DPRINTK("%s: GOLCD loops=%d.\n", __FUNCTION__, n);

	val = dispc_reg_read(ct, DISPC_CONTROL);
	val |= DISPC_CONTROL_PCKFREEENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* Wait 2x PCLK */
	msleep(1);

	/* Request SDI PLL to lock */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val |= DSS_PLL_CONTROL_GOBIT;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	for (n = 0, val = dss_reg_read(ct, DSS_SDI_STATUS);
	     (DSS_SDI_STATUS_PLL_BUSYFLAG & val) && 10 > n;
	     ++n, val = dss_reg_read(ct, DSS_SDI_STATUS))
		msleep(1);

	DPRINTK("%s: busy loops=%d.\n", __FUNCTION__, n);

	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ~DSS_PLL_CONTROL_GOBIT;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	for (n = 0, val = dss_reg_read(ct, DSS_SDI_STATUS);
	     !(DSS_SDI_STATUS_PLL_LOCK & val) && 10 > n;
	     ++n, val = dss_reg_read(ct, DSS_SDI_STATUS))
		msleep(1);

	DPRINTK("%s: PLL lock loops=%d.\n", __FUNCTION__, n);

	/* Enable the SDI */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val |= DISPC_CONTROL_LCDENABLESIGNAL;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	for (n = 0, val = dss_reg_read(ct, DSS_SDI_STATUS);
	     !(DSS_SDI_STATUS_RESET_DONE & val) && 10 > n;
	     ++n, val = dss_reg_read(ct, DSS_SDI_STATUS))
		msleep(1);

	DPRINTK("%s: reset loops=%d.\n", __FUNCTION__, n);

	/* Wait wake-up time (2 ms) for the PLL of the serial receiver to lock */
	msleep(2);

	DPRINTK("%s: SDI start sequence complete.\n", __FUNCTION__);

	/* ungate the lines */
	val = dispc_reg_read(ct, DISPC_CONFIG);
	val &= ~DISPC_CONFIG_VSYNCGATED;
	val &= ~DISPC_CONFIG_HSYNCGATED;
	val &= ~DISPC_CONFIG_PIXELDATAGATED;
	val &= ~DISPC_CONFIG_ACBIASGATED;
	dispc_reg_write(ct, val, DISPC_CONFIG);

	omap2_disp_enable_layer(OMAP2_GRAPHICS);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);

	DPRINTK("%s:  Done\n", __FUNCTION__);
#endif

	return 1;
}

/******************************************************************************/
/*
 * sysfs entries
 */
static ssize_t controller_store_freq(struct device *dev,
					struct device_attribute *dev_attr,
					const char *buf,
					size_t count)
{
	char *endp;
	u32 new_freq = 0;
	u32 pixclk_max = 0;
	u32 pixclk_min = 0;
	size_t size;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	new_freq = simple_strtoul(buf, &endp, 0);
	while (*endp && isspace(*endp)) endp++;
	size = endp - buf;

	if (*endp) {
		pixclk_min = simple_strtoul(endp, &endp, 0);
	}
	while (*endp && isspace(*endp)) endp++;

	if (*endp) {
		pixclk_max = simple_strtoul(endp, &endp, 0);
	}
	while (*endp && isspace(*endp)) endp++;


	if (pixclk_min != 0 && pixclk_min < new_freq)
	{
		lcd_pixclock_min = pixclk_min;
	}
	if ( pixclk_max != 0 && pixclk_max > new_freq)
	{
		lcd_pixclock_max = pixclk_max;
	}

	size = endp - buf;

	if ( new_freq <= lcd_pixclock_max  && new_freq >= lcd_pixclock_min )
	{
		ct->panel_info.pixclock = new_freq;
	}

	if ( new_freq == 0)
	{
		return -EINVAL;
	}

	mutex_lock(&ct->lock);
	lcd_controller_set_freq((void*)dev, new_freq);
	mutex_unlock(&ct->lock);

	return count;
}

static ssize_t controller_show_freq(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	int count;

	mutex_lock(&ct->lock);
	printk("ct->panel_info.pixclk = %d\n", ct->panel_info.pixclock);
	printk("lcd_pixclock_min = %d\n",      lcd_pixclock_min);
	printk("lcd_pixclock_max = %d\n",      lcd_pixclock_max);


	count = snprintf(buf, PAGE_SIZE, " %d %d %d\nOK\n",
			ct->panel_info.pixclock, lcd_pixclock_min,
			lcd_pixclock_max);
	mutex_unlock(&ct->lock);

	return count;
}


DEVICE_ATTR(freq_change, S_IWUSR|S_IRUGO, controller_show_freq, controller_store_freq);


static ssize_t controller_show_dss_pll_control(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	int rc;
#ifdef CONFIG_ARCH_OMAP3430
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	mutex_lock(&ct->lock);
	omap2_disp_get_dss();
	rc = snprintf(buf, PAGE_SIZE, "0x%x\n", dss_reg_read(ct,
							DSS_PLL_CONTROL));
	omap2_disp_put_dss();
	mutex_unlock(&ct->lock);
#else
	rc = snprintf(buf, PAGE_SIZE, "Error: Unsupported - OMAP3430 ONLY\n");
#endif
	return rc;
}
DEVICE_ATTR(reg_dss_pll_control, S_IRUGO, controller_show_dss_pll_control, NULL);

static ssize_t controller_show_dispc_divisor(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	int rc;
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	mutex_lock(&ct->lock);
	omap2_disp_get_dss();
	rc = snprintf(buf, PAGE_SIZE, "0x%x\n", dispc_reg_read(ct,
							DISPC_DIVISOR));
	omap2_disp_put_dss();
	mutex_unlock(&ct->lock);

	return rc;
}
DEVICE_ATTR(reg_dispc_divisor, S_IRUGO, controller_show_dispc_divisor, NULL);

/* test functions */
static ssize_t controller_show_ctrl_stop(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	mutex_lock(&ct->lock);
	sdi_stop(ct);
	mutex_unlock(&ct->lock);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
DEVICE_ATTR(ctrl_stop, S_IRUGO, controller_show_ctrl_stop, NULL);

static ssize_t controller_show_ctrl_start(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	u32 val;

	mutex_lock(&ct->lock);
	omap2_disp_get_dss();
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val |= DISPC_CONTROL_PCKFREEENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	sdi_start(ct);

	omap2_disp_enable_layer(OMAP2_GRAPHICS);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	omap2_disp_put_dss();
	mutex_unlock(&ct->lock);

	DPRINTK("%s:  Done\n", __FUNCTION__);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
DEVICE_ATTR(ctrl_start, S_IRUGO, controller_show_ctrl_start, NULL);

static void controller_reg_dump(struct omap_lcd_controller *ct)
{
	int i;

	omap2_disp_get_dss();
	printk("DSS registers:\n");
	printk("DSS_SYSCONFIG   = 0x%08x\n", dss_reg_read(ct, DSS_SYSCONFIG));
	printk("DSS_SYSSTATUS   = 0x%08x\n", dss_reg_read(ct, DSS_SYSSTATUS));
	printk("DSS_IRQSTATUS   = 0x%08x\n", dss_reg_read(ct, DSS_IRQSTATUS));
	printk("DSS_CONTROL     = 0x%08x\n", dss_reg_read(ct, DSS_CONTROL));
	printk("DSS_SDI_CONTROL = 0x%08x\n", dss_reg_read(ct, DSS_SDI_CONTROL));
	printk("DSS_PLL_CONTROL = 0x%08x\n", dss_reg_read(ct, DSS_PLL_CONTROL));
	printk("DSS_SDI_STATUS  = 0x%08x\n", dss_reg_read(ct, DSS_SDI_STATUS));

	printk("\nDISPC registers:\n");
	printk("DISPC_SYSCONFIG   = 0x%08x\n", dispc_reg_read(ct, DISPC_SYSCONFIG));
	printk("DISPC_SYSSTATUS   = 0x%08x\n", dispc_reg_read(ct, DISPC_SYSSTATUS));
	printk("DISPC_IRQSTATUS   = 0x%08x\n", dispc_reg_read(ct, DISPC_IRQSTATUS));
	printk("DISPC_IRQENABLE   = 0x%08x\n", dispc_reg_read(ct, DISPC_IRQENABLE));
	printk("DISPC_CONTROL     = 0x%08x\n", dispc_reg_read(ct, DISPC_CONTROL));
	printk("DISPC_CONFIG      = 0x%08x\n", dispc_reg_read(ct, DISPC_CONFIG));
	printk("DISPC_CAPABLE     = 0x%08x\n", dispc_reg_read(ct, DISPC_CAPABLE));
	printk("DISPC_DEF_COLOR0  = 0x%08x\n", dispc_reg_read(ct, DISPC_DEFAULT_COLOR0));
	printk("DISPC_DEF_COLOR1  = 0x%08x\n", dispc_reg_read(ct, DISPC_DEFAULT_COLOR1));
	printk("DISPC_TRAN_COLOR0 = 0x%08x\n", dispc_reg_read(ct, DISPC_TRANS_COLOR0));
	printk("DISPC_TRAN_COLOR1 = 0x%08x\n", dispc_reg_read(ct, DISPC_TRANS_COLOR1));
	printk("DISPC_LINE_STATUS = 0x%08x\n", dispc_reg_read(ct, DISPC_LINE_STATUS));
	printk("DISPC_LINE_NUMBER = 0x%08x\n", dispc_reg_read(ct, DISPC_LINE_NUMBER));
	printk("DISPC_TIMING_H    = 0x%08x\n", dispc_reg_read(ct, DISPC_TIMING_H));
	printk("DISPC_TIMING_V    = 0x%08x\n", dispc_reg_read(ct, DISPC_TIMING_V));
	printk("DISPC_POL_FREQ    = 0x%08x\n", dispc_reg_read(ct, DISPC_POL_FREQ));
	printk("DISPC_DIVISOR     = 0x%08x\n", dispc_reg_read(ct, DISPC_DIVISOR));
	printk("DISPC_GLOBAL_ALPHA= 0x%08x\n", dispc_reg_read(ct, DISPC_GLOBAL_ALPHA));
	printk("DISPC_SIZE_DIG    = 0x%08x\n", dispc_reg_read(ct, DISPC_SIZE_DIG));
	printk("DISPC_SIZE_LCD    = 0x%08x\n", dispc_reg_read(ct, DISPC_SIZE_LCD));

	printk("\n");
	printk("DISPC_GFX_BA0     = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_BA0));
	printk("DISPC_GFX_BA1     = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_BA1));
	printk("DISPC_GFX_POSITION= 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_POSITION));
	printk("DISPC_GFX_SIZE    = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_SIZE));
	printk("DISPC_GFX_ATTRIB  = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_ATTRIBUTES));
	printk("DISPC_GFX_FIFO_TH = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_FIFO_THRESHOLD));
	printk("DISPC_GFX_FIFO_SZ = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_FIFO_SIZE));
	printk("DISPC_GFX_ROW_INC = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_ROW_INC));
	printk("DISPC_GFX_WIN_SKP = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_WINDOW_SKIP));
	printk("DISPC_GFX_TBL_BA  = 0x%08x\n", dispc_reg_read(ct, DISPC_GFX_TABLE_BA));

	for (i=0; i<=1; i++) {
		printk("DISPC_VID%d_BA0    = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_BA0(i)));
		printk("DISPC_VID%d_BA1    = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_BA1(i)));
		printk("DISPC_VID%d_POS    = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_POSITION(i)));
		printk("DISPC_VID%d_SIZE   = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_SIZE(i)));
		printk("DISPC_VID%d_ATTRIB = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_ATTRIBUTES(i)));
		printk("DISPC_VID%d_FIFO_TH= 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_FIFO_THRESHOLD(i)));
		printk("DISPC_VID%d_FIFO_SZ= 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_FIFO_SIZE(i)));
		printk("DISPC_VID%d_ROW_INC= 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_ROW_INC(i)));
		printk("DISPC_VID%d_PXL_INC= 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_PIXEL_INC(i)));
		printk("DISPC_VID%d_FIR    = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_FIR(i)));
		printk("DISPC_VID%d_PIC_SZ = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_PICTURE_SIZE(i)));
		printk("DISPC_VID%d_ACCU0  = 0x%08x\n", i,
				dispc_reg_read(ct, DISPC_VID_ACCU0(i)));
		printk("DISPC_VID%d_ACCU1  = 0x%08x\n\n", i,
				dispc_reg_read(ct, DISPC_VID_ACCU1(i)));
	}

	printk("\nRFBI registers:\n");
	printk("RFBI_SYSCONFIG   = 0x%08x\n", dispc_reg_read(ct, RFBI_SYSCONFIG));
// 	printk("RFBI_CONTROL     = 0x%08x\n", dispc_reg_read(ct, RFBI_CONTROL));
	omap2_disp_put_dss();

	DPRINTK("%s:  Done\n", __FUNCTION__);
}

static ssize_t controller_show_ctrl_reg_dump(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	mutex_lock(&ct->lock);
	controller_reg_dump(ct);
	mutex_unlock(&ct->lock);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
DEVICE_ATTR(ctrl_reg_dump, S_IRUGO, controller_show_ctrl_reg_dump, NULL);

static ssize_t controller_show_restart_controller(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	u32 pixdiv;

	mutex_lock(&ct->lock);
	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD,
					ct->panel_info.xres,
					ct->panel_info.yres);
	pixdiv = ct->panel_info.pixclock / (1000000000UL /
					(omap24xx_get_dss1_clock() / 1000));

	DPRINTK("dss1 clk = %d, pixdiv = 0x%x\n",
				omap24xx_get_dss1_clock(), pixdiv);

	omap2_disp_config_lcd(pixdiv,
				ct->panel_info.left_margin - 1,	// hbp
				ct->panel_info.right_margin - 1,// hfp
				ct->panel_info.hsync_len - 1,	// hsw
				ct->panel_info.upper_margin,	// vbp
				ct->panel_info.lower_margin,	// vfp
				ct->panel_info.vsync_len - 1,	// vsw
				ct->panel_info.bits_per_pixel	// bpp
				);

	/* initialize the controller */
	omap_controller_init(ct);
	mutex_unlock(&ct->lock);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
DEVICE_ATTR(restart_ctrl, S_IRUGO, controller_show_restart_controller, NULL);




static ssize_t controller_show_porch_control(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	int count;

	mutex_lock(&ct->lock);
	count = snprintf(buf, PAGE_SIZE, " left margin = %d\n"
			" right margin = %d\n"
			" hsync=%d\n"
			" upper margin = %d\n"
			" lower margin = %d\n"
			" vsync=%d\n",
			ct->panel_info.left_margin,
			ct->panel_info.right_margin,
			ct->panel_info.hsync_len,
			ct->panel_info.upper_margin,
			ct->panel_info.lower_margin,
			ct->panel_info.vsync_len);
	mutex_unlock(&ct->lock);

	return count;
}

static ssize_t controller_set_porch_control(struct device *dev,
					struct device_attribute *dev_attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	u32 pixdiv;
	const char *endp;
	u32 cmd[6];
	int i = 0;
	size_t size;

	DPRINTK("%s\n", __FUNCTION__);

	cmd[0] = ct->panel_info.left_margin ;
	cmd[1] = ct->panel_info.right_margin;
	cmd[2] = ct->panel_info.hsync_len   ;
	cmd[3] = ct->panel_info.upper_margin;
	cmd[4] = ct->panel_info.lower_margin;
	cmd[5] = ct->panel_info.vsync_len   ;


	endp=buf;
	for (i=0;i<6 && *endp;i++) {
		char *p;
		cmd[i] = simple_strtoul(endp, &p, 0);
		while (*p && isspace(*p)) p++;
		endp = p;
	}
	while (*endp ) endp++;

	size= endp - buf;

	ct->panel_info.left_margin  = cmd[0] ;
	ct->panel_info.right_margin = cmd[1] ;
	ct->panel_info.hsync_len    = cmd[2] ;
	ct->panel_info.upper_margin = cmd[3] ;
	ct->panel_info.lower_margin = cmd[4] ;
	ct->panel_info.vsync_len    = cmd[5] ;

	mutex_lock(&ct->lock);

	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD,
					ct->panel_info.xres,
					ct->panel_info.yres);
	pixdiv = ct->panel_info.pixclock / (1000000000UL /
					(omap24xx_get_dss1_clock() / 1000))-1;

	DPRINTK("dss1 clk = %d, pixdiv = 0x%x\n",
				omap24xx_get_dss1_clock(), pixdiv);

	omap2_disp_config_lcd(pixdiv,
				ct->panel_info.left_margin - 1,	// hbp
				ct->panel_info.right_margin - 1,// hfp
				ct->panel_info.hsync_len - 1,	// hsw
				ct->panel_info.upper_margin,	// vbp
				ct->panel_info.lower_margin,	// vfp
				ct->panel_info.vsync_len - 1,	// vsw
				ct->panel_info.bits_per_pixel	// bpp
				);

	printk("ct->panel_info.left_margin = %d\n"	,ct->panel_info.left_margin );
	printk("ct->panel_info.right_margin = %d\n"	,ct->panel_info.right_margin );
	printk("ct->panel_info.hsync_len = %d\n"	,ct->panel_info.hsync_len  );
	printk("ct->panel_info.upper_margin = %d\n"	,ct->panel_info.upper_margin );
	printk("ct->panel_info.lower_margin = %d\n"	,ct->panel_info.lower_margin );
	printk("ct->panel_info.vsync_len = %d\n"	,ct->panel_info.vsync_len  );
	printk("ct->panel_info.bits_per_pixel = %d\n",ct->panel_info.bits_per_pixel  );

	/* initialize the controller */
	omap2_disp_reg_sync(OMAP2_OUTPUT_LCD);

	omap2_disp_save_initstate(OMAP_DSS_GENERIC);
	omap2_disp_save_initstate(OMAP_DSS_DISPC_GENERIC);

	mutex_unlock(&ct->lock);
	return size;
}

DEVICE_ATTR(porch_ctrl, S_IRUGO|S_IWUSR, controller_show_porch_control, controller_set_porch_control);

static ssize_t controller_show_lpr_mode(struct device *dev,
				   struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	int mode;

	mutex_lock(&ct->lock);
	mode = omap2_disp_lpr_mode();
	mutex_unlock(&ct->lock);

	return sprintf(buf, "%s\n",
			DISPLAY_LPR_MODE_AUTO == mode ? "auto" : "manual");
}

static ssize_t controller_store_lpr_mode(struct device *dev,
				    struct device_attribute *dev_attr,
				    const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	if (!buf || (count == 0))
		return 0;

	mutex_lock(&ct->lock);
	if (0 == strncmp(buf, "auto", 4)) {
		omap2_disp_lpr_set_mode(DISPLAY_LPR_MODE_AUTO);
	}
	else if (0 == strncmp(buf, "manual", 6)) {
		omap2_disp_lpr_set_mode(DISPLAY_LPR_MODE_MANUAL);
	}
	else {
		count = -EINVAL;
	}
	mutex_unlock(&ct->lock);

	return count;
}

DEVICE_ATTR(lpr_mode, S_IWUSR|S_IRUGO, controller_show_lpr_mode, controller_store_lpr_mode);

static ssize_t controller_show_lpr_state(struct device *dev,
				   struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	int enabled;

	mutex_lock(&ct->lock);
	enabled = omap2_disp_lpr_is_enabled();
	mutex_unlock(&ct->lock);

	return sprintf(buf, "%s\n", enabled ? "on" : "off");
}

static ssize_t controller_store_lpr_state(struct device *dev,
				    struct device_attribute *dev_attr,
				    const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	if (!buf || (count == 0))
		return 0;

	mutex_lock(&ct->lock);
	if (0 == strncmp(buf, "on", 2)) {
		omap2_disp_lpr_set_state(DISPLAY_LPR_STATE_ON);
	}
	else if (0 == strncmp(buf, "off", 3)) {
		omap2_disp_lpr_set_state(DISPLAY_LPR_STATE_OFF);
	}
	else {
		count = -EINVAL;
	}
	mutex_unlock(&ct->lock);

	return count;
}

DEVICE_ATTR(lpr_state, S_IWUSR|S_IRUGO, controller_show_lpr_state, controller_store_lpr_state);

static ssize_t controller_show_fifo_lo(struct device *dev,
				   struct device_attribute *dev_attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	int thrs;

	mutex_lock(&ct->lock);
	omap2_disp_get_dss();
	thrs = omap2_disp_get_gfx_fifo_low_threshold();
	omap2_disp_put_dss();
	mutex_unlock(&ct->lock);

	return sprintf(buf, "%d\n", thrs);
}

static ssize_t controller_store_fifo_lo(struct device *dev,
				    struct device_attribute *dev_attr,
				    const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);
	int thrs;

	if (!buf || (count == 0))
		return 0;

	sscanf(buf, "%d\n", &thrs);
	if ((thrs < 0) || (thrs > 3000)) {
		printk(KERN_ERR "LCD: Invalid FIFO LOW THRESHOLD (%d) "
				"for LPR mode.\n", thrs);
		return -EINVAL;
	}

	mutex_lock(&ct->lock);
	omap2_disp_get_dss();
	omap2_disp_set_gfx_fifo_low_threshold(thrs);
	omap2_disp_put_dss();
	mutex_unlock(&ct->lock);

	return count;
}

DEVICE_ATTR(fifo_lo, S_IWUSR|S_IRUGO, controller_show_fifo_lo, controller_store_fifo_lo);

/******************************************************************************/
/*
 * suspend / resume
 */
#ifdef CONFIG_PM
static int lcd_controller_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	DPRINTK("%s\n", __FUNCTION__);
	return 0;
}

static int lcd_controller_resume(struct platform_device *pdev)
{
	DPRINTK("%s\n", __FUNCTION__);
	return 0;
}
#else
#define lcd_controller_suspend	NULL
#define lcd_controller_resume	NULL
#endif

/******************************************************************************/

void omap2_disp_hack_synclost_irq_enable(void);
void omap2_disp_hack_synclost_irq_disable(void);

/* This flag tracks if we already enabled LPR AUTO mode at boot. */
#ifdef CONFIG_OMAP_DISPLAY_ENABLE_AUTO_LPR_AT_BOOT
static int lpr_auto_enabled_at_boot = 0;
#else
static int lpr_auto_enabled_at_boot = 1; /* mark as already handled */
#endif

static void lcd_controller_enable(struct omap_lcd_controller *ct)
{
	bool display_changed = false;
	if (CONTROLLER_STATE_ON == ct->controller_state) {
#ifdef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
		/* We need to set LPR AUTO mode at this point as we will not
		 * handle the lcd_controller_enable() call if
		 * ct->controller_state is set to CONTROLLER_STATE_ON.
		 */
		if (!lpr_auto_enabled_at_boot) {
			omap2_disp_lpr_set_mode(DISPLAY_LPR_MODE_AUTO);
			lpr_auto_enabled_at_boot = 1;
		}
#endif
		return;
	}

	DPRINTK(" %s: Enabling clks\n", __FUNCTION__);
	omap2_disp_get_dss();

	DPRINTK(" %s: enable dss\n", __FUNCTION__);
	dss_init(ct);

	/* Restore previous context.
	 * NOTE:
	 *   If no context has been saved yet, i.e. this is the first call to
	 *   enable, then the omap2_disp_restore_initstate() functions will
	 *   have no effect.
	 */
	omap2_disp_restore_initstate(OMAP_DSS_GENERIC);
	omap2_disp_restore_initstate(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_restore_initstate(OMAP2_GRAPHICS);

	/* Force BSOD if BA0 is bad...
	 */
	if (0 == dispc_reg_read(ct, DISPC_GFX_BA0)) {
		printk("##### Restore value for BA0 is NULL\n");
		BUG();
	}


#ifdef CONFIG_ARCH_OMAP3430
	if (ct->use_sdi) {
		ct->sdi_power_enable(1);
		sdi_start(ct);
	}
#endif

	omap2_disp_enable_layer(OMAP2_GRAPHICS);

	/* Restore state of video layers. TBD do more...it is only a partial fix....*/


	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);

	ct->controller_state = CONTROLLER_STATE_ON;

	/* If this is the first time we enable the display, we set LPR AUTO
	 * mode.
	 */
	if (!lpr_auto_enabled_at_boot) {
		omap2_disp_lpr_set_mode(DISPLAY_LPR_MODE_AUTO);
		lpr_auto_enabled_at_boot = 1;
	}

	omap2_disp_handle_lpr_auto_mode();

	/* Re-enable IRQ for sync lost handler here. See notes in
	 * lcd_controller_disable().
	 */
	omap2_disp_hack_synclost_irq_enable();

	DPRINTK("%s:  Done\n", __FUNCTION__);
}

static void lcd_controller_disable(struct omap_lcd_controller *ct)
{
	if (CONTROLLER_STATE_OFF == ct->controller_state)
		return;

	/* Disabling DSS Sync Lost IRQ here fixes a race at wake-up time.
	 * 
	 * If we do not disable IRQs here it is possible that a sync lost irq
	 * occurs while we are bringing up the display. The sync lost handler
	 * will then try to restore the DSS context while we are in the middle
	 * of restoring it in lcd_controller_enable(). This will cause the LCD
	 * restore to fail, potentially causing an OCP error.
	 */
	omap2_disp_hack_synclost_irq_disable();

	/* Save the context. */
	omap2_disp_save_initstate(OMAP_DSS_GENERIC);
	omap2_disp_save_initstate(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_save_initstate(OMAP2_GRAPHICS);
	omap2_disp_save_initstate(OMAP2_VIDEO1);
	omap2_disp_save_initstate(OMAP2_VIDEO2);

#ifdef CONFIG_ARCH_OMAP3430
	DPRINTK(" %s: Stopping SDI...\n", __FUNCTION__);
	if (ct->use_sdi) {
		sdi_stop(ct);
		ct->sdi_power_enable(0);
	}
#endif

	omap2_disp_disable_layer(OMAP2_GRAPHICS);
	omap2_disp_disable_output_dev(OMAP2_OUTPUT_LCD);
	omap2_disp_disable_output_dev(OMAP2_OUTPUT_TV);

	omap2_disp_put_dss();
	ct->controller_state = CONTROLLER_STATE_OFF;

	DPRINTK(" %s: Shutting down clks\n", __FUNCTION__);
}

static void lcd_controller_set_state(void *dev, unsigned int state)
{
	struct platform_device     *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct   = platform_get_drvdata(pdev);

	DPRINTK("%s:\n", __FUNCTION__);

	mutex_lock(&ct->lock);
	if (state) {
		lcd_controller_enable(ct);
	}
	else {
		lcd_controller_disable(ct);
	}
	mutex_unlock(&ct->lock);
}

static struct lcd_controller_ops omap3_lcd_controller_ops = {
	.ctrl_set_state = lcd_controller_set_state,
};

static int __init lcd_controller_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct omap_lcd_controller *ct;
	struct controller_platform_data *ctrl_plat;
#ifndef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	u32 pixdiv;
#endif	
	int i;

	DPRINTK("%s\n", __FUNCTION__);

	ct = kzalloc(sizeof(struct omap_lcd_controller), GFP_KERNEL);
	if (unlikely(!ct)) {
		ret = -ENOMEM;
		goto err0;
	}

	platform_set_drvdata(pdev, ct);

	DPRINTK("%s: ct at 0x%x\n", __FUNCTION__, (int)ct);
	DPRINTK("%s: pdev at 0x%x, dev at 0x%x\n", __FUNCTION__, (int)pdev,
							(int)(&pdev->dev));

	if (pdev->dev.platform_data != NULL) {
		ctrl_plat = (struct controller_platform_data*)
				pdev->dev.platform_data;
	} else {
		printk(KERN_ERR MOD_NAME "Error - platform data not set\n");
		ret = -EINVAL;
		goto err1;
	}
	localct = ct;

	ct->panel_info         = ctrl_plat->screen_info;
	ct->dss_config_lpr_on  = ctrl_plat->dss_config_lpr_on;
	ct->dss_config_lpr_off = ctrl_plat->dss_config_lpr_off;

	/* Set the LPR configuration. This will not enable LPR just configure
	 * it. LPR will be enabled later if all conditions are met...
	 */
	omap2_disp_lpr_set_cfg(&ct->dss_config_lpr_on, &ct->dss_config_lpr_off);

	/* make some adjustments to the screen */
	if (ctrl_plat->right_margin)
		ct->panel_info.right_margin = *(ctrl_plat->right_margin);

	panel_lcd_var   = &(ct->panel_info);
	ct->parent      = ctrl_plat->parent;
	ct->use_sdi     = *(ctrl_plat->use_sdi);
	ct->vsync_gated = *(ctrl_plat->vsync_gated);

	ct->dss_reg_base     = ctrl_plat->dss_reg_base;
	ct->dss_reg_offset   = ctrl_plat->dss_reg_offset;
	ct->dispc_reg_offset = ctrl_plat->dispc_reg_offset;

	if (ct->use_sdi) {
		ct->sdi_color_depth       = ctrl_plat->sdi_color_depth;
		ct->sdi_freq_selector     = ctrl_plat->sdi_freq_selector;
		ct->sdi_num_of_data_pairs = ctrl_plat->sdi_num_of_data_pairs;
		ct->sdi_pll_lock_method   = ctrl_plat->sdi_pll_lock_method;
		ct->sdi_pll_to_pclk_ratio = ctrl_plat->sdi_pll_to_pclk_ratio;
		ct->sdi_power_enable      = ctrl_plat->sdi_power_enable;
	}

	ct->pdev = pdev;

	/* make sure that the power to the sdi is on */
	if (ct->sdi_power_enable)
		ct->sdi_power_enable(1);

	add_controller_device (&pdev->dev, &omap3_lcd_controller_ops, ct->parent);

	DPRINTK("%s: Display device at 0x%x\n", __FUNCTION__, (int)ct->disp_dev);
	DPRINTK("Starting OMAP initialization\n");
	DPRINTK("Screen res: %dx%d\n", ct->panel_info.xres, ct->panel_info.yres);

	for (i=0;i<6;i++) {
		if (boot_porch[i]!=0) {
			if (boot_porch[0] != 0 ) ct->panel_info.left_margin  = boot_porch[0] ;
			if (boot_porch[1] != 0 ) ct->panel_info.right_margin = boot_porch[1] ;
			if (boot_porch[2] != 0 ) ct->panel_info.hsync_len    = boot_porch[2] ;
			if (boot_porch[3] != 0 ) ct->panel_info.upper_margin = boot_porch[3] ;
			if (boot_porch[4] != 0 ) ct->panel_info.lower_margin = boot_porch[4] ;
			if (boot_porch[5] != 0 ) ct->panel_info.vsync_len    = boot_porch[5] ;

			printk("boot_porch override\n");
			printk("ct->panel_info.left_margin = %d\n"	,ct->panel_info.left_margin );
			printk("ct->panel_info.right_margin = %d\n"	,ct->panel_info.right_margin );
			printk("ct->panel_info.hsync_len = %d\n"	,ct->panel_info.hsync_len  );
			printk("ct->panel_info.upper_margin = %d\n"	,ct->panel_info.upper_margin );
			printk("ct->panel_info.lower_margin = %d\n"	,ct->panel_info.lower_margin );
			printk("ct->panel_info.vsync_len = %d\n"	,ct->panel_info.vsync_len  );
			printk("ct->panel_info.bits_per_pixel = %d\n",ct->panel_info.bits_per_pixel  );

			break;
		}
	}

#ifdef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	/* If the display has been already initialized by the bootloader, we
	 * don't have to do anything here. Just mark the state as ON.
	 */
	ct->controller_state = CONTROLLER_STATE_ON;

	omap2_disp_get_dss();

	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD,
					ct->panel_info.xres,
					ct->panel_info.yres);

	omap2_disp_put_dss();

#else
	/* Display has not yet been initialized. We need to initialize here.
	 */
	omap2_disp_get_dss();

#ifdef CONFIG_ARCH_OMAP3430
	if (ct->use_sdi)
		sdi_stop(ct);
#endif

	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD,
					ct->panel_info.xres,
					ct->panel_info.yres);
	pixdiv = ct->panel_info.pixclock / (1000000000UL /
					(omap24xx_get_dss1_clock() / 1000));

	DPRINTK("dss1 clk = %d, pixdiv = 0x%x\n",
				omap24xx_get_dss1_clock(), pixdiv);

	omap2_disp_config_lcd(pixdiv,
				ct->panel_info.left_margin - 1,	// hbp
				ct->panel_info.right_margin - 1,// hfp
				ct->panel_info.hsync_len - 1,	// hsw
				ct->panel_info.upper_margin,	// vbp
				ct->panel_info.lower_margin,	// vfp
				ct->panel_info.vsync_len - 1,	// vsw
				ct->panel_info.bits_per_pixel	// bpp
				);

	/* initialize the controller */
	omap_controller_init(ct);


	if (boot_lcd_pixclock != 0) {
		printk("boot_lcd_pixclock %d\n",boot_lcd_pixclock);
		printk("boot_lcd_pixclock_min %d\n",boot_lcd_pixclock_min);
		printk("boot_lcd_pixclock_max %d\n",boot_lcd_pixclock_max);

		ct->panel_info.pixclock = boot_lcd_pixclock;
		lcd_pixclock_min = boot_lcd_pixclock_min;
		lcd_pixclock_max = boot_lcd_pixclock_max;

		lcd_controller_set_freq(&pdev->dev, ct->panel_info.pixclock);
	}
	/* Init done. Release the clocks. */
	omap2_disp_put_dss();

	ct->controller_state = CONTROLLER_STATE_OFF;
#endif

	mutex_init(&ct->lock);

	/* initialize the sysfs entries */
	if (device_create_file(&pdev->dev, &dev_attr_reg_dss_pll_control) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for dss_pll_control failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_reg_dispc_divisor) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for dispc_divisor failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_porch_ctrl) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for porch_ctrl failed\n");
		goto err2;
	}

	if (device_create_file(&pdev->dev, &dev_attr_freq_change) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for dispc_divisor failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_ctrl_start) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for sdi_start failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_ctrl_stop) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for sdi_stop failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_restart_ctrl) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for restart_ctrl failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_ctrl_reg_dump) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for ctrl_reg_dump failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_lpr_mode) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for lpr_mode failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_lpr_state) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for lpr_state failed\n");
		goto err2;
	}
	if (device_create_file(&pdev->dev, &dev_attr_fifo_lo) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for fifo_lo failed\n");
		goto err2;
	}

	printk(KERN_INFO "OMAP LCD controller initialized\n");
	return 0;

err2:
	device_remove_file(&pdev->dev, &dev_attr_reg_dss_pll_control);
	device_remove_file(&pdev->dev, &dev_attr_reg_dispc_divisor);
	device_remove_file(&pdev->dev, &dev_attr_porch_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_freq_change);
	device_remove_file(&pdev->dev, &dev_attr_ctrl_start);
	device_remove_file(&pdev->dev, &dev_attr_ctrl_stop);
	device_remove_file(&pdev->dev, &dev_attr_restart_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_ctrl_reg_dump);
	device_remove_file(&pdev->dev, &dev_attr_lpr_mode);
	device_remove_file(&pdev->dev, &dev_attr_lpr_state);
	device_remove_file(&pdev->dev, &dev_attr_fifo_lo);

	display_device_unregister(ct->disp_dev);
err1:
	kfree(ct);
err0:
	return ret;
}

static int lcd_controller_remove(struct platform_device *pdev)
{
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	DPRINTK("%s\n", __FUNCTION__);

	device_remove_file(&pdev->dev, &dev_attr_reg_dss_pll_control);
	device_remove_file(&pdev->dev, &dev_attr_reg_dispc_divisor);
	device_remove_file(&pdev->dev, &dev_attr_freq_change);
	device_remove_file(&pdev->dev, &dev_attr_ctrl_start);
	device_remove_file(&pdev->dev, &dev_attr_ctrl_stop);
	device_remove_file(&pdev->dev, &dev_attr_restart_ctrl);
	device_remove_file(&pdev->dev, &dev_attr_ctrl_reg_dump);
	device_remove_file(&pdev->dev, &dev_attr_lpr_mode);
	device_remove_file(&pdev->dev, &dev_attr_lpr_state);
	device_remove_file(&pdev->dev, &dev_attr_fifo_lo);

	display_device_unregister(ct->disp_dev);

	kfree(ct);

	printk(KERN_INFO "OMAP LCD controller removed\n");

	return 0;
}

static struct platform_driver lcd_controller_driver = {
	.probe   = lcd_controller_probe,
	.remove  = lcd_controller_remove,
	.suspend = lcd_controller_suspend,
	.resume  = lcd_controller_resume,
	.driver  = {
		   .name = "lcd-controller",
	},
};

static int __init lcd_controller_init(void)
{
	DPRINTK("%s\n", __FUNCTION__);
	return platform_driver_register(&lcd_controller_driver);
}

static void __exit lcd_controller_exit(void)
{
	DPRINTK("%s\n", __FUNCTION__);
	platform_driver_unregister(&lcd_controller_driver);
}

module_init(lcd_controller_init);
module_exit(lcd_controller_exit);

MODULE_AUTHOR("John Chen <jchen1996@gmail.com>");
MODULE_DESCRIPTION("LCD controller driver");
MODULE_LICENSE("GPL");
