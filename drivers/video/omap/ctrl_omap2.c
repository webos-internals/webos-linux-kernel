/*
 * drivers/video/omap/ctrl_omap3.c
 *
 * driver for the omap controller initializations
 *
 * Copyright (c) 2008 John Chen  <jchen1996@gmail.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <asm/arch/mcspi.h>
#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/display.h>
#include <asm/arch/lcd.h>

#include "lcd.h"

#define MOD_NAME 			"LCD Ctrl: "

#undef MODDEBUG
// #define MODDEBUG			1

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

static struct fb_var_screeninfo *panel_lcd_var;
extern int omap24xx_get_dss1_clock(void);

struct omap_lcd_controller {
	struct device *parent;
	struct display_device *disp_dev;
	struct platform_device *pdev;
	int controller_state;

	struct fb_var_screeninfo panel_info;
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
		return LCD_PIXCLOCK_MAX;
	} else if (output_dev == OMAP2_OUTPUT_TV) {
		return ~0;
	}

	return -EINVAL;
}

u32 get_panel_pixclock_min(int output_dev)
{
	DPRINTK("%s\n", __FUNCTION__);

	if (output_dev == OMAP2_OUTPUT_LCD) {
		return LCD_PIXCLOCK_MIN;
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
void omap2_dss_rgb_disable(void)
{
	DPRINTK("%s\n", __FUNCTION__);
}
void omap2_dss_rgb_enable(void)
{
	DPRINTK("%s\n", __FUNCTION__);
}

EXPORT_SYMBOL(omap2_dss_rgb_disable);
EXPORT_SYMBOL(omap2_dss_rgb_enable);
EXPORT_SYMBOL(enable_backlight);
EXPORT_SYMBOL(disable_backlight);
EXPORT_SYMBOL(get_panel_default_var);
EXPORT_SYMBOL(get_panel_pixclock_max);
EXPORT_SYMBOL(get_panel_pixclock_min);

static inline u32 dss_reg_read(struct omap_lcd_controller *ct, u32 reg)
{
    return omap_readl(ct->dss_reg_base + ct->dss_reg_offset + (reg));
}
static inline void dss_reg_write(struct omap_lcd_controller *ct, u32 val,
								u32 reg)
{
    omap_writel((val), ct->dss_reg_base + ct->dss_reg_offset + (reg));
}

static inline u32 dispc_reg_read(struct omap_lcd_controller *ct, u32 reg)
{
    return omap_readl(ct->dss_reg_base + ct->dispc_reg_offset + (reg));
}
static inline void dispc_reg_write(struct omap_lcd_controller *ct, u32 val,
								u32 reg)
{
    omap_writel((val), ct->dss_reg_base + ct->dispc_reg_offset + (reg));
}

#ifdef CONFIG_ARCH_OMAP3430
/*
 * sdi start sequence
 */
static void sdi_init(struct omap_lcd_controller *ct, int m, int n, int freqsel)
{
	u32 val;

	/* color depth must be 24 bits for FL3G */
	val = dss_reg_read(ct, DSS_SDI_CONTROL);
	val |= ct->sdi_color_depth;
	dss_reg_write(ct, val, DSS_SDI_CONTROL);

	/* set the lock method */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ct->sdi_pll_lock_method;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);

	/* using 2 data pairs */
	/* Nominal data rate = nominal pixel clock * 15 bits/pair (2 pairs) */
	val = dss_reg_read(ct, DSS_SDI_CONTROL);
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

	/* Release SDI PLL reset */
	val = dss_reg_read(ct, DSS_PLL_CONTROL);
	val &= ~DSS_PLL_CONTROL_SYSRESET;
	dss_reg_write(ct, val, DSS_PLL_CONTROL);
}

static void sdi_start(struct omap_lcd_controller *ct)
{
	u32 val;
	int n;
	unsigned long pcd;
	unsigned long pclk;
	unsigned long ndiv;
	unsigned long mdiv;
	u32 pixclk = ct->panel_info.pixclock;

	/* Configure the PLL */
	pcd = pixclk / (1000000000 / (omap24xx_get_dss1_clock() / 1000));
	pclk = omap24xx_get_dss1_clock() / pcd;
	ndiv = pclk / 2100000;
	mdiv = 15 * (ndiv + 1);

	DPRINTK("%s: pcd=%lu pclk=%lu ndiv=%lu mdiv=%lu Fint=%lu.\n",
	     __FUNCTION__, pcd, pclk, ndiv, mdiv, pclk / (ndiv + 1));

	/* program the SDI initial configuration */
	/* Fint = 0x7 = 1.75Mhz - 2.1Mhz (optimal) */
	sdi_init(ct, mdiv, ndiv, ct->sdi_freq_selector);
	sdi_power_up(ct);

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

}

static void sdi_stop(struct omap_lcd_controller *ct)
{
	u32 val;
	int n;

	/* disable the display controller */
	val = dispc_reg_read(ct, DISPC_CONTROL);
	val &= ~DISPC_CONTROL_LCDENABLE;
	dispc_reg_write(ct, val, DISPC_CONTROL);

	/* read the irq status framedone bit */
	for (n = 0, val = dispc_reg_read(ct, DISPC_IRQSTATUS);
	     !(DISPC_IRQSTATUS_FRAMEDONE & val) && 10 > n;
	     ++n, val = dispc_reg_read(ct, DISPC_IRQSTATUS))
		msleep(1);

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

	if (ct->vsync_gated) {
		DPRINTK("Using Vsync as reset line\n");
		val = dispc_reg_read(ct, DISPC_CONFIG);
		val |= DISPC_CONFIG_VSYNCGATED;
		dispc_reg_write(ct, val, DISPC_CONFIG);
	}

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
	unsigned long pcd, pclk, ndiv, mdiv;

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
	pcd = pixclk / (1000000000 / (omap24xx_get_dss1_clock() / 1000));
	pclk = omap24xx_get_dss1_clock() / pcd;
	ndiv = pclk / 2100000;
	mdiv = 15 * (ndiv + 1);

	DPRINTK("%s: pcd=%lu pclk=%lu ndiv=%lu mdiv=%lu Fint=%lu.\n",
	     __FUNCTION__, pcd, pclk, ndiv, mdiv, pclk / (ndiv + 1));

	/* program the SDI initial configuration */
	sdi_init(ct, mdiv, ndiv, ct->sdi_freq_selector); /* Fint = 0x7 = 1.75Mhz - 2.1Mhz (optimal) */


	DPRINTK("%s: new freq = %d dss1 clk = %d\n",
				__FUNCTION__, pixclk, omap24xx_get_dss1_clock());

	/* change the registers */
	omap2_disp_config_lcd(pcd,
				ct->panel_info.left_margin - 1,	// hbp
				ct->panel_info.right_margin - 1,// hfp
				ct->panel_info.hsync_len - 1,	// hsw
				ct->panel_info.upper_margin,	// vbp
				ct->panel_info.lower_margin,	// vfp
				ct->panel_info.vsync_len - 1,	// vsw
				ct->panel_info.bits_per_pixel	// bpp
				);

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

/* sysfs entries */
static ssize_t controller_store_freq(struct device *dev,
					struct device_attribute *dev_attr,
					const char *buf,
					size_t count)
{
	char *endp;
	u32 new_freq = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	lcd_controller_set_freq((void*)dev, new_freq);

	return count;
}
DEVICE_ATTR(freq_change, S_IWUSR, NULL, controller_store_freq);


static ssize_t controller_show_dss_pll_control(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
#ifdef CONFIG_ARCH_OMAP3430
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", dss_reg_read(ct,
							DSS_PLL_CONTROL));
#else
	return snprintf(buf, PAGE_SIZE, "Error: Unsupported - OMAP3430 ONLY\n");
#endif
}
DEVICE_ATTR(reg_dss_pll_control, S_IRUGO, controller_show_dss_pll_control, NULL);

static ssize_t controller_show_dispc_divisor(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", dispc_reg_read(ct,
							DISPC_DIVISOR));
}
DEVICE_ATTR(reg_dispc_divisor, S_IRUGO, controller_show_dispc_divisor, NULL);


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

static void lcd_controller_set_state(void* dev, unsigned int state)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_lcd_controller *ct = platform_get_drvdata(pdev);

	DPRINTK("%s:\n", __FUNCTION__);

	if (state) {
		ct->controller_state = CONTROLLER_STATE_ON;
	} else {
		ct->controller_state = CONTROLLER_STATE_OFF;
	}

}

static struct lcd_controller_ops omap3_lcd_controller_ops = {
	.ctrl_set_state = lcd_controller_set_state,
};

static int lcd_controller_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct omap_lcd_controller *ct;
	struct controller_platform_data *ctrl_plat;
	u32 clkdiv;

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

	memcpy(&ct->panel_info, &ctrl_plat->screen_info,
					sizeof(struct fb_var_screeninfo));
	/* make some adjustments to the screen */
	if (ctrl_plat->right_margin)
		ct->panel_info.right_margin = *(ctrl_plat->right_margin);

	panel_lcd_var = &(ct->panel_info);
	ct->parent = ctrl_plat->parent;
	ct->use_sdi = *(ctrl_plat->use_sdi);
	ct->vsync_gated = *(ctrl_plat->vsync_gated);

	ct->dss_reg_base = ctrl_plat->dss_reg_base;
	ct->dss_reg_offset = ctrl_plat->dss_reg_offset;
	ct->dispc_reg_offset = ctrl_plat->dispc_reg_offset;

	if (ct->use_sdi) {
		ct->sdi_color_depth = ctrl_plat->sdi_color_depth;
		ct->sdi_freq_selector = ctrl_plat->sdi_freq_selector;
		ct->sdi_num_of_data_pairs = ctrl_plat->sdi_num_of_data_pairs;
		ct->sdi_pll_lock_method = ctrl_plat->sdi_pll_lock_method;
		ct->sdi_pll_to_pclk_ratio = ctrl_plat->sdi_pll_to_pclk_ratio;
		if (ctrl_plat->sdi_power_enable)
			ct->sdi_power_enable = ctrl_plat->sdi_power_enable;
	}

	ct->pdev = pdev;

	/* make sure that the power to the sdi is on */
	if (ct->sdi_power_enable)
		(void)ct->sdi_power_enable(1);

	add_controller_device (&pdev->dev, &omap3_lcd_controller_ops, ct->parent);

	DPRINTK("%s: Display device at 0x%x\n", __FUNCTION__, (int)ct->disp_dev);

	DPRINTK("Starting OMAP initialization\n");

	DPRINTK("Screen res: %dx%d\n",
		ct->panel_info.xres, ct->panel_info.yres);

	omap2_disp_get_dss();

#ifdef CONFIG_ARCH_OMAP3430
	if (ct->use_sdi)
		sdi_stop(ct);
#endif

	omap2_disp_set_panel_size(OMAP2_OUTPUT_LCD,
					ct->panel_info.xres,
					ct->panel_info.yres);
	clkdiv = ct->panel_info.pixclock / (1000000000UL /
					(omap24xx_get_dss1_clock() / 1000));

	DPRINTK("dss1 clk = %d, clkdiv = 0x%x\n",
				omap24xx_get_dss1_clock(), clkdiv);

	omap2_disp_config_lcd(clkdiv,
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

	// omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	omap2_disp_put_dss();

	msleep(50);

	ct->controller_state = CONTROLLER_STATE_ON;

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
	if (device_create_file(&pdev->dev, &dev_attr_freq_change) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for dispc_divisor failed\n");
		goto err2;
	}

	printk(KERN_INFO "OMAP LCD controller initialized\n");

	return 0;


err2:
	device_remove_file(&pdev->dev, &dev_attr_reg_dss_pll_control);
	device_remove_file(&pdev->dev, &dev_attr_reg_dispc_divisor);
	device_remove_file(&pdev->dev, &dev_attr_freq_change);

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

	display_device_unregister(ct->disp_dev);

	kfree(ct);

	printk(KERN_INFO "OMAP LCD controller removed\n");

	return 0;
}

static struct platform_driver lcd_controller_driver = {
	.probe = lcd_controller_probe,
	.remove = lcd_controller_remove,
	.suspend = lcd_controller_suspend,
	.resume = lcd_controller_resume,
	.driver = {
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
