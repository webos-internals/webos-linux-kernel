/*
 * drivers/video/omap24xxfb.c
 *
 * Framebuffer driver for OMAP24xx display controller.
 *
 * Copyright (C) 2004-2005-2006 Texas Instruments, Inc.
 *
 * Author: Andy Lowe (source@mvista.com)
 * Copyright (C) 2004 MontaVista Software, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/console.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
// #include <asm/arch/bus.h>
#include <asm/arch/clock.h>
#include <asm/uaccess.h>

#ifdef CONFIG_PM
#define PM_DEBUG 1
#include <linux/notifier.h>
#include <linux/pm.h>
#endif

#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif

#include <asm/arch/display.h>
#include "omap_fb.h"

#undef DEBUG

#ifdef DEBUG
#define DBGENTER printk(KERN_INFO "%s: Enter\n", __FUNCTION__);
#define DBGENTER_c printk(KERN_INFO "%s: Enter, con=%d\n", __FUNCTION__, con);
#define DBGLEAVE printk(KERN_INFO "%s: Exit\n", __FUNCTION__);
#else
#define DBGENTER
#define DBGLEAVE
#define DBGENTER_c
#endif

#define OMAPFB_DEVICE	"omap24xxfb"
#define OMAPFB_DRIVER	"omap24xxfb"
#define FB_NAME 	"omap24xxfb"	/* 16 chars max */

#define SCHEDULE_WAIT	0
#define BUSY_WAIT	1

/* To use the rotation feature, include this in your boot params:
	video=omap24xxfb:rotation=[0|90|180|270]
*/

int omap24xxfb_rotation = -1;	// -1 = no rotation support
int omap24xxfb_mirroring = 0;	// the status of mirroring

#define omap_rotation_index(rotation_deg) \
		(rotation_deg == 90)?(270/90): \
		(rotation_deg == 270)?(90/90): \
		(rotation_deg == 180)?(180/90): \
		(0/90)

#define omap_rot_mirror_index(rotation_deg) \
		(rotation_deg == 90)?(90/90): \
		(rotation_deg == 270)?(270/90): \
		(rotation_deg == 180)?(0/90): \
		(180/90)

struct omap24xxfb_info {
	/* The fb_info structure must be first! */
	struct fb_info info;

	dma_addr_t mmio_base_phys;
	dma_addr_t fb_base_phys;
	unsigned long fb_size;
	unsigned long mmio_base;
	unsigned long fb_base;

	wait_queue_head_t vsync_wait;
	unsigned long vsync_cnt;
	unsigned int  vsync_ref;  /* vsync irq ref count */
	spinlock_t    vsync_lock; /* vsync irq lock */

	u32 pseudo_palette[17];
	u32 *palette;
	dma_addr_t palette_phys;

	/* Period of the graphics clock in picoseconds.
	 * This is is the input to the pixel clock divider.
	 */
	unsigned long gfx_clk_period;
	unsigned int hsync;	/* horizontal sync frequency in Hz */
	unsigned int vsync;	/* vertical sync frequency in Hz */
	unsigned long timeout;	/* register update timeout period in ticks */

	int alloc_fb_mem;
	int asleep;
	int blanked;

	int rotation_support;
	int rotation_deg;
	dma_addr_t sms_rot_phy[4];
	unsigned long sms_rot_virt[4];
	unsigned long vrfb_size;
};

static struct omap24xxfb_info *saved_oinfo;
static struct fb_var_screeninfo default_var;

int	fb_out_layer = OMAP2_GRAPHICS;
extern void get_panel_default_var(struct fb_var_screeninfo *var, int output_dev);
extern u32 get_panel_pixclock_max(int output_dev);
extern u32 get_panel_pixclock_min(int output_dev);
extern void enable_backlight(void);
extern void disable_backlight(void);
int omap24xx_get_dss1_clock(void);
extern int omap24xxfb_set_output_layer(int layer);

/******************************************************************************/
/* Platform-specific customization for the framebuffer driver.
 */

/* omap24xxfb_gfx_clk_period() must return the period (in picoseconds) of the
 * graphics timebase clock.  This clock is the input to the pixel clock
 * divider.
 *
 * prototype:
 * unsigned long omap24xxfb_gfx_clk_period(void);
 */
#define omap24xxfb_gfx_clk_period() (1000000000UL/(omap24xx_get_dss1_clock()/1000))

/* omap24xxfb_fb_base() must return the physical base address of the
 * framebuffer.  If the address is non-zero, then the framebuffer memory is
 * ioremap'd.  If the address is zero, then the framebuffer memory is
 * dynamically allocated by the driver.
 *
 * prototype:
 * unsigned long omap24xxfb_fb_base(void);
 */
#define omap24xxfb_fb_base() 0

/* omap24xxfb_fb_size() must return the size of the framebuffer in bytes.
 * The framebuffer is only ioremap'd (or kmalloc'd) at driver initialization
 * time.  It does not change when the video mode (resolution and color depth)
 * changes, so it must be large enough for all video modes that are to be
 * supported.
 *
 * In non-rotation mode, we're allocating a framebuffer 3 times the size of
 * the physical display. This is to support triple buffering. The panning ioctl
 * can be used to switch between the three different framebuffer regions, so
 * you effectively have an onscreen framebuffer and two offscreen framebuffers.
 *
 * prototype:
 * unsigned long omap24xxfb_fb_size(void);
 */
#define omap24xxfb_fb_size(rotation) \
	rotation ? (2048 * 576 * (16/8)) : (720 * 576 * (16/8) * 3)
/* 720 x 576 is the max framebuffer size we allow for TV (PAL) */

/* omap24xxfb_vrfb_size() must return the size of the virtual rotated
 * framebuffer.
 *
 * prototype:
 * unsigned long omap24xxfb_fb_size(void);
 */
#define omap24xxfb_vrfb_size()	(MAX_PIXELS_PER_LINE * 576 * (16/8))

/* omap24xx_display_pixclock_max() must return the maximum pixclock period
 * supported by the display.
 *
 * prototype:
 * unsigned int omap24xx_display_pixclock_max(void);
 */
#define omap24xx_display_pixclock_max(ouput_dev) \
		(get_panel_pixclock_max(output_dev))

/* omap24xx_display_pixclock_min() must return the minimum pixclock period
 * supported by the display.
 *
 * prototype:
 * unsigned int omap24xx_display_pixclock_min(void);
 */
#define omap24xx_display_pixclock_min(output_dev) \
		(get_panel_pixclock_min(output_dev))

/* omap24xxfb_default_var() must return a pointer to a default
 * fb_var_screeninfo struct that will be used to set the initial video mode.
 * If this video mode is invalid (as determined by omap24xxfb_check_var) then
 * the driver will fail to initialize.
 *
 * prototype:
 * struct fb_var_screeninfo *omap24xxfb_default_var(void);
 */

static struct fb_var_screeninfo *
omap24xxfb_default_var(void)
{
	struct fb_var_screeninfo *v = &default_var;
	int output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);
	u32 tmp;

	get_panel_default_var(v, output_dev);

	if (omap24xxfb_rotation >= 0) {
		v->xres_virtual = v->yres_virtual = max(v->xres, v->yres);

		switch(omap24xxfb_rotation) {
		case 0:
		default:
			v->xoffset	= 0;
			v->yoffset	= 0;
			v->rotate	= 0;
			break;
		case 90:
			tmp = v->xres, v->xres = v->yres, v->yres = tmp;
			v->xoffset	= 0;
			v->yoffset	= 0;
			v->rotate	= 90;
			break;
		case 180:
			v->xoffset	= 0;
			v->yoffset	= 0;
			v->rotate 	= 180;
			break;
		case 270:
			tmp = v->xres, v->xres = v->yres, v->yres = tmp;
			v->xoffset	= 0;
			v->yoffset	= 0;
			v->rotate 	= 270;
			break;
		}
	}
	return v;
}

/* omap24xxfb_check_mode() must check the video mode specified in a
 * fb_var_screeninfo struct and return 0 if the mode is supported and non-zero
 * if the mode is not supported.  omap24xxfb_check_mode() is permitted to
 * modify the var to make it conform to a supported mode.
 *
 * prototype:
 * int omap24xxfb_check_mode(const struct omap24xxfb_info *oinfo,
 *			     struct fb_var_screeninfo *var);
 */
static int
omap24xxfb_check_mode(const struct omap24xxfb_info *oinfo,
		      struct fb_var_screeninfo *var)
{
	u32 pixclock, clkdiv;
	u32 display_xres, display_yres;
	int output_dev;

	omap2_disp_get_dss();
	output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);

	omap2_disp_get_panel_size(output_dev, &display_xres, &display_yres);
	if (oinfo->rotation_support) {

		if (var->rotate % 90 != 0) {
			omap2_disp_put_dss();
			return -EINVAL;
		}

		if (!((var->bits_per_pixel == 8) ||
		      (var->bits_per_pixel == 16))) {
			omap2_disp_put_dss();
			return -EINVAL;
		}

		switch (var->rotate) {
		case 0:
		case 180:
		default:
			if ((var->xres > display_xres) ||
			    (var->yres > display_yres)) {
				omap2_disp_put_dss();
				return -EINVAL;
			}
			break;
		case 90:
		case 270:
			if ((var->xres > display_yres) ||
			    (var->yres > display_xres)) {
				omap2_disp_put_dss();
				return -EINVAL;
		}
			break;
		}
	}
	else {
		if ((var->xres > display_xres) ||
		    (var->yres > display_yres)) {
			omap2_disp_put_dss();
			return -EINVAL;
		}
		}

	pixclock = (var->pixclock > 0) ? var->pixclock :
				omap24xx_display_pixclock_max(output_dev);

	if (pixclock < omap24xx_display_pixclock_min(output_dev)){
		omap2_disp_put_dss();
		return -EINVAL;
	}

	clkdiv = pixclock / oinfo->gfx_clk_period;
	pixclock = oinfo->gfx_clk_period * clkdiv;
	if (pixclock > omap24xx_display_pixclock_max(output_dev)) {
		omap2_disp_put_dss();
		return -EINVAL;
	}

	/* due to round-down error in division, the pixclock may fall below
	   the lower threshold of the panel. Fix that by adding 1 to clkdiv.
	*/
	if (pixclock < omap24xx_display_pixclock_min(output_dev))
		clkdiv = clkdiv + 1;

	if (clkdiv < 2)		/* minimum divisor is 2 */
		clkdiv = 2;
	else if (clkdiv > 255)
		clkdiv = 255;	/* maximum divisor is 255 */

	/* recalculate pixclock and change the var structure */
	pixclock = oinfo->gfx_clk_period * clkdiv;
	var->pixclock = pixclock;
	omap2_disp_put_dss();

	return 0;
}

#ifdef CONFIG_PM
struct omap24xxfb_suspend_data {
	/* Power management suspend lockout stuff */
	int suspended;
	wait_queue_head_t suspend_wq;
};
static struct omap24xxfb_suspend_data fb_suspend_data;

#define omap24xxfb_suspend_lockout_fp(s,f) \
	if ((s)->suspended) {\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		wait_event_interruptible((s)->suspend_wq,\
					 (s)->suspended == 0);\
	}
#define omap24xxfb_suspend_lockout(s) \
	if ((s)->suspended) {\
		wait_event_interruptible((s)->suspend_wq,\
					 (s)->suspended == 0);\
	}
#else
#define omap24xxfb_suspend_lockout_fp(s, f) do {} while(0)
#define omap24xxfb_suspend_lockout(s) do {} while(0)
#endif	/*ifdef CONFIG_PM */


/******************************************************************************/

/* Bits-per-pixel and color depth aren't quite the same thing.  The OMAP24xx
 * display controller supports color depths of 1, 2, 4, 8, 12, 16, and 24 bits.
 * Color depth and bits-per-pixel are the same for depths of 1, 2, 4, 8, and
 * 16 bits, but for a color depth of 12 bits the pixel data is padded to
 * 16 bits-per-pixel, and for a color depth of 24 bits the pixel data is padded
 * to 32 bits-per-pixel.
 */
static inline int
var_to_depth(const struct fb_var_screeninfo *var)
{
	DBGENTER;
	switch (var->bits_per_pixel) {
		case 1:
		case 2:
		case 4:
		case 8:
		default:
			return var->bits_per_pixel;
		case 16:
			if ((var->red.length + var->green.length
				+ var->blue.length) == 12)
			{
				return 12;
			}
			else
				return 16;
		case 32:
			if (var->transp.length > 0)
				return 32;
			else
				return 24;
	}
	DBGLEAVE;
}


/* Calculate the horizontal sync frequency in Hertz
 * with a resolution of 250Hz.
 */
static unsigned int
horizontal_sync_freq(const struct fb_var_screeninfo *var)
{
	unsigned int hsf, hs, nom, den;
	unsigned int xres, yres;
	int output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);

	DBGENTER;

	/* Calculate the number of pixels per clock. */
	omap2_disp_pixels_per_clock(&nom, &den);

	/* get the horizontal display resolution */
	omap2_disp_get_panel_size(output_dev, &xres, &yres);

	hs = (xres*den)/nom
	 	+ (var->left_margin + var->right_margin + var->hsync_len);

	if ((var->pixclock > 0) && (hs > 0))
		hsf = (4000000000UL/(var->pixclock*hs))*250;
		/* pixclock is in picoseconds
		 * 10^12 / (pixclock*hs) = 4 * 10^9 * 250 / (pixclock*hs)
		*/
	else
		hsf = 0;

	DBGLEAVE;
	return hsf;
}

/* Calculate the vertical sync frequency in Hertz. */
static unsigned int
vertical_sync_freq(const struct fb_var_screeninfo *var)
{
	unsigned int vsf, vs;
	unsigned int xres, yres;

	int output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);

	DBGENTER;
	/* get the vertical display resolution */
	omap2_disp_get_panel_size(output_dev, &xres, &yres);

	vs = yres + var->upper_margin + var->lower_margin + var->vsync_len;

	if (vs > 0)
		vsf = horizontal_sync_freq(var)/vs;
	else
		vsf = 0;

	DBGLEAVE;
	return vsf;
}

/* Interrupt service routine. */
static void
omap24xxfb_isr(void *arg, struct pt_regs *regs)
{
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) arg;

	++oinfo->vsync_cnt;
	wake_up_interruptible_all(&oinfo->vsync_wait);
}

/* Wait for a vsync interrupt.  This routine sleeps so it can only be called
 * from process context.
 */
static int
omap24xxfb_wait_for_vsync(struct omap24xxfb_info *oinfo)
{
	int ret;
	unsigned long cnt;
	unsigned long flags;
	unsigned int  mask = 0;

	DBGENTER;

	mask = (DISPC_IRQSTATUS_EVSYNC_ODD 
			| DISPC_IRQSTATUS_EVSYNC_EVEN
			| DISPC_IRQSTATUS_VSYNC);

	spin_lock_irqsave(&oinfo->vsync_lock, flags );
	if(++oinfo->vsync_ref == 1) {
		omap2_disp_irqenable(omap24xxfb_isr,mask);
	}
	cnt = oinfo->vsync_cnt;
	spin_unlock_irqrestore(&oinfo->vsync_lock, flags );
	
	ret = wait_event_interruptible_timeout(oinfo->vsync_wait,
				cnt != oinfo->vsync_cnt, oinfo->timeout);

	/*
	  * If the GFX is on TV, then wait for another VSYNC
	  * to compensate for Interlaced scan
	  */
	if(omap2_disp_get_output_dev(OMAP2_GRAPHICS) == OMAP2_OUTPUT_TV) {
		if( ret < 0) {
			cnt = oinfo->vsync_cnt;
			ret = wait_event_interruptible_timeout(oinfo->vsync_wait,
				cnt != oinfo->vsync_cnt, oinfo->timeout);
		}
	}

	spin_lock_irqsave(&oinfo->vsync_lock, flags );
	if(--oinfo->vsync_ref == 0) {
		omap2_disp_irqdisable(omap24xxfb_isr,~(mask));
	}
	spin_unlock_irqrestore(&oinfo->vsync_lock, flags );

	DBGLEAVE;

	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}


int omap24xxfb_display_wait_for_vsync(void)
{
	if (saved_oinfo!=NULL) {
		return omap24xxfb_wait_for_vsync(saved_oinfo);
	}
	return 0;

}

#ifdef DEBUG
static void
omap24xxfb_synclost_isr(void *arg, struct pt_regs *regs)
{
	printk("<sl digital>");
}
#endif

/* The GO bit needs to be set for the shadowed registers to take effect in
 * hardware. Once the hardware is ready, the GO bit will be reset. Per the
 * hardware specifications, we should not change any display controller
 * registers until the GO bit is reset.
 * This function polls the GO bit and waits until it is reset.
 * If the function may be called when the interrupts are disabled (jiffies
 * not running). In such cases, the 'count' variable helps to exit the loop
 * incase the bit gets stuck.
 */
static void
wait_for_reg_sync(int busy_wait, unsigned long timeout)
{
	int output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);
	int count = 200;

	timeout = jiffies + timeout;
	while (!omap2_disp_reg_sync_done(output_dev) &&
	       time_before(jiffies, timeout) && count) {

		if (busy_wait || in_interrupt()) {
			udelay(100);
			count--;
		}
		else {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
#ifdef CONFIG_PM
			if (fb_suspend_data.suspended)
				return;
#endif
		}
	}
        if (!omap2_disp_reg_sync_done(output_dev)) {
		printk(KERN_WARNING "timeout waiting for display controller "
		       "to save registers\n");
	}
}

	/*
	 * fbops functions
	 */

/*
 *	omap24xxfb_check_var - Validates a var passed in.
 *	@var: frame buffer variable screen structure
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Returns negative errno on error, or zero on success.  The var info on
 *	exit may be different than the var info on entry.
 *
 *	This function accepts any bits-per-pixel value in the range 0 to 32
 *	(0 to 8 for monochrome displays).  If the specified number of
 *	bits-per-pixel isn't supported, then the next greater number of
 *	bits-per-pixel will be substituted.  This function differentiates
 *	between color depths of 12 and 16 (which both have 16 bits-per-pixel)
 *	by checking to see if the sum of the lengths of the RGB fields is 12,
 *	in which case the color depth is assumed to be 12.  Except for
 *	differentiating between 12-bit and 16-bit color depths, the values
 *	passed in var->red, var->green, and var->blue are ignored and replaced
 *	with the correct values for the specified color depth.
 *
 *	The xres/yres variables in the var screeninfo specify the size of the
 *	graphics window.  The graphics window size must not be larger than the
 *	physical display size nor larger than the size of the framebuffer.
 *	The xres_virtual/yres_virtual values in the var screeninfo specify the
 *	size of the framebuffer in memory. The framebuffer must not be smaller
 *	than the size of the graphics window.  The display must not be smaller
 *	than the graphics window.  The display size depends on whether the
 *	framebuffer is displayed on LCD or a TV. The xoffset/yoffset variables
 *	in the var screeninfo specify the offset of the graphics window within
 *	the framebuffer.  There is no means for the user to specify the offset
 *	of the graphics window on the display, so that offset will always be
 *	zero unless the board-specific mode changing function implements some
 *	other behavior, such as centering.
 */
static int
omap24xxfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	const struct omap24xxfb_info *oinfo =
				(const struct omap24xxfb_info *) info->par;
	struct fb_var_screeninfo v;
	u32 hbp, hfp, hsw, vbp, vfp, vsw;
	u32 display_xres, display_yres;
	int output_dev;

	omap2_disp_get_dss();
	output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);

	DBGENTER;
	memcpy(&v, var, sizeof(v));

	/* do board-specific checks on the var */
	if (omap24xxfb_check_mode(oinfo, &v)){
		omap2_disp_put_dss();
		return -EINVAL;
	}

	switch (v.bits_per_pixel) {
		case 0 ... 1:
			v.bits_per_pixel = 1;
			break;
		case 2:
			v.bits_per_pixel = 2;
			break;
		case 3 ... 4:
			v.bits_per_pixel = 4;
			break;
		case 5 ... 8:
			v.bits_per_pixel = 8;
			break;
		case 9 ... 16:
			if (v.grayscale) {
				omap2_disp_put_dss();
				return -EINVAL;
			}
			v.bits_per_pixel = 16;
			break;
		case 17 ... 32:
			if (v.grayscale) {
				omap2_disp_put_dss();
				return -EINVAL;
			}
			v.bits_per_pixel = 32;
			break;
		default:
			omap2_disp_put_dss();
			return -EINVAL;
	}

	switch (var_to_depth(&v)) {
		case 1:
			v.red.offset = v.green.offset = v.blue.offset = 0;
			v.red.length = v.green.length = v.blue.length = 1;
			v.transp.offset = v.transp.length = 0;
			break;
		case 2:
			v.red.offset = v.green.offset = v.blue.offset = 0;
			v.red.length = v.green.length = v.blue.length = 2;
			v.transp.offset = v.transp.length = 0;
			break;
		case 4:
			v.red.offset = v.green.offset = v.blue.offset = 0;
			v.red.length = v.green.length = v.blue.length = 4;
			v.transp.offset = v.transp.length = 0;
			break;
		case 8:
			v.red.offset = v.green.offset = v.blue.offset = 0;
			v.red.length = v.green.length = v.blue.length = 8;
			v.transp.offset = v.transp.length = 0;
			break;
		case 12:
			v.red.offset = 8;
			v.green.offset = 4;
			v.blue.offset = 0;
			v.red.length = v.green.length = v.blue.length = 4;
			v.transp.offset = v.transp.length = 0;
			break;
		case 16:
			v.red.offset = 11;
			v.green.offset = 5;
			v.blue.offset = 0;
			v.red.length = 5;
			v.green.length = 6;
			v.blue.length = 5;
			v.transp.offset = v.transp.length = 0;
			break;
		case 24:
			v.red.offset = 16;
			v.green.offset = 8;
			v.blue.offset = 0;
			v.red.length = v.blue.length = v.green.length = 8;
			v.transp.offset = v.transp.length = 0;
			break;
		case 32:
			v.red.offset = 16;
			v.green.offset = 8;
			v.blue.offset = 0;
			v.red.length = v.blue.length = v.green.length = 8;
			v.transp.offset = 24;
			v.transp.length = 8;
			break;
		default:
			omap2_disp_put_dss();
			return -EINVAL;
	}

	omap2_disp_get_panel_size(output_dev, &display_xres, &display_yres);
	if (display_xres > MAX_PIXELS_PER_LINE ||
	    display_yres > MAX_LINES) {
		omap2_disp_put_dss();
		return -EINVAL;
	}

	if (display_xres == 0 || display_yres == 0) {
		omap2_disp_put_dss();
		return -EINVAL;
	}
	if (v.xres_virtual < v.xres || v.yres_virtual < v.yres) {
		omap2_disp_put_dss();
		return -EINVAL;
	}

 	if (!oinfo->rotation_support) {
		if (display_xres < v.xres || display_yres < v.yres) {
			omap2_disp_put_dss();
			return -EINVAL;
		}
	} else {
		switch(v.rotate) {
		case 0:
		case 180:
		default:
			if (display_xres < v.xres ||
			    display_yres < v.yres) {
				omap2_disp_put_dss();
				return -EINVAL;
			}
			break;
		case 90:
		case 270:
			if (display_xres < v.yres ||
			    display_yres < v.xres) {
				omap2_disp_put_dss();
				return -EINVAL;
		}
			break;
		}
	}

	if (v.xoffset > v.xres_virtual - v.xres) {
		omap2_disp_put_dss();
		return -EINVAL;
	}
	if (v.yoffset > v.yres_virtual - v.yres) {
		omap2_disp_put_dss();
		return -EINVAL;
	}
	if ((v.bits_per_pixel < 8) && (((v.xres_virtual % 8) != 0)
		|| ((v.xres % 8) != 0) || ((v.xoffset % 8) != 0)))
	{
		omap2_disp_put_dss();
		return -EINVAL;
	}

	/* check if we have enough video memory to support this framebuffer */
	if (((!oinfo->rotation_support) &&
		(((v.xres_virtual*v.yres_virtual*v.bits_per_pixel)/8)
		> info->fix.smem_len))
	   ||
	   ((oinfo->rotation_support) &&
		(((MAX_PIXELS_PER_LINE*v.yres_virtual*v.bits_per_pixel)/8)
		> info->fix.smem_len)))

	{
		omap2_disp_put_dss();
		return -EINVAL;
	}

	/* calculate horizontal timing parameters in pixclocks */
	hbp = (v.left_margin > 0) ? (v.left_margin - 1) : 0;
	if (hbp > 255)
		hbp = 255;
	hfp = (v.right_margin > 0) ? (v.right_margin - 1) : 0;
	if (hfp > 255)
		hfp = 255;
	hsw = (v.hsync_len > 0) ? (v.hsync_len - 1) : 0;
	if (hsw > 63)
		hsw = 63;
	v.left_margin = hbp + 1;
	v.right_margin = hfp + 1;
	v.hsync_len = hsw + 1;

	/* calculate vertical timing parameters in line clocks */
	vbp = v.upper_margin;
	if (vbp > 255)
		vbp = 255;
	vfp = v.lower_margin;
	if (vfp > 255)
		vfp = 255;
	vsw = (v.vsync_len > 0) ? (v.vsync_len - 1) : v.vsync_len;
	if (vsw > 63)
		vsw = 63;
	v.upper_margin = vbp;
	v.lower_margin = vfp;
	v.vsync_len = vsw + 1;

	v.red.msb_right = v.green.msb_right = v.blue.msb_right
		= v.transp.msb_right = 0;

	v.nonstd = 0;
	v.accel_flags = 0;

	memcpy(var, &v, sizeof(v));
	omap2_disp_put_dss();

	DBGLEAVE;
	return 0;
}

/* Calculates the Graphics pipleline DMA parameters */

static int
omap24xxfb_set_dma_params(const struct fb_var_screeninfo *var,
			  const struct fb_fix_screeninfo *fix,
			  int rotation, int mirroring)
{
	int context = 0;
	u32 view_address_base;
	u32 start, total_width, img_width;
	u32 row_inc, pix_inc;
	u32 xoffset, yoffset;

	if ((rotation > 0) && (rotation % 90 != 0))
		return -EINVAL;

        if (!mirroring) {
		if (rotation < 0) {
			view_address_base = fix->smem_start;

			start = view_address_base
				+ var->yoffset*fix->line_length
				+ (var->xoffset*var->bits_per_pixel)/8;

			total_width = fix->line_length;

			img_width = (var->xres *
				     var->bits_per_pixel)/8;
		}
		else {
			view_address_base = SMS_ROT_VIRT_BASE(context, 0);

			switch (rotation) {
			/* to the user, xres and yres are interchanged between
			   0/180 deg and 90/270 deg, but the DMA still has to use the
			   panel's dimension and orientation.
			*/
			case 0:
			case 180:
			default:
				xoffset = var->xoffset;
				if (rotation == 180)
					xoffset += var->xres_virtual - var->xres;

				start = view_address_base
					+ var->yoffset*fix->line_length
					+ (xoffset*var->bits_per_pixel)/8;

				total_width = fix->line_length;

				img_width = (var->xres *
					     var->bits_per_pixel)/8;
				break;
			case 90:
			case 270:
				yoffset = var->yoffset;
				if (rotation == 90)
					yoffset += var->yres_virtual - var->yres;

				start = view_address_base
					+ var->xoffset*fix->line_length
					+ (yoffset*var->bits_per_pixel)/8;

				total_width = fix->line_length;
				img_width = (var->yres *
					     var->bits_per_pixel)/8;
				break;
			}
		}
		pix_inc = 1;
		row_inc = 1 + total_width - img_width;

		omap2_disp_set_dma_params(OMAP2_GRAPHICS, OMAP2_OUTPUT_LCD,
					  start, start, row_inc, pix_inc);

		omap2_disp_set_dma_params(OMAP2_GRAPHICS, OMAP2_OUTPUT_TV,
					  start, (start + total_width),
					  (row_inc + total_width), pix_inc);
	}
	else {	/* mirroring */
		if (rotation < 0) {
			view_address_base = fix->smem_start;

			img_width = (var->xres *
				     var->bits_per_pixel)/8;

			start = view_address_base
				+ var->yoffset*fix->line_length
				+ (var->xoffset*var->bits_per_pixel)/8

				+ (var->xres - 1) * var->bits_per_pixel/8;

			total_width = fix->line_length;

			pix_inc = - 2 * (var->bits_per_pixel/8) + 1;
			row_inc = total_width
				  + (var->xres - 2) * var->bits_per_pixel/8 + 1;

			omap2_disp_set_dma_params(OMAP2_GRAPHICS,
						  OMAP2_OUTPUT_LCD,
                                                  start, start,
						  row_inc,
						  pix_inc);

			omap2_disp_set_dma_params(OMAP2_GRAPHICS,
						  OMAP2_OUTPUT_TV,
						  start, (start + total_width),
						  (row_inc + total_width),
						  pix_inc);
		}
		else {
			view_address_base = SMS_ROT_VIRT_BASE(context, 0);
			switch (rotation) {
			case 0:
			case 180:
			default:
                                xoffset = var->xoffset;
				if (rotation == 0) {
                                        xoffset += var->xres_virtual - var->xres;
				}

				start = view_address_base
					+ (var->yres - var->yoffset - 1) * fix->line_length
					- (xoffset*var->bits_per_pixel)/8;

				total_width = fix->line_length;

				img_width = (var->xres *
					     var->bits_per_pixel)/8;
				break;
			case 90:
			case 270:
				yoffset = var->yoffset;
				if (rotation == 270) {
                                        yoffset += var->yres_virtual - var->yres;
				}

				start = view_address_base
					+ (var->xres - var->xoffset - 1) * fix->line_length
					+ (yoffset*var->bits_per_pixel)/8;

				total_width = fix->line_length;

				img_width = (var->yres *
					     var->bits_per_pixel)/8;
				break;
			}
			pix_inc = 1;
			row_inc = - (total_width + img_width) + 1;

			omap2_disp_set_dma_params(OMAP2_GRAPHICS,
						  OMAP2_OUTPUT_LCD,
                                                  start, start,
						  row_inc,
						  pix_inc);

			omap2_disp_set_dma_params(OMAP2_GRAPHICS,
						  OMAP2_OUTPUT_TV,
						  start, (start - total_width),
						  (row_inc - total_width),
						  pix_inc);
		}
	}

	return 0;
}

/*
 *	omap24xxfb_set_par - Alters the hardware state.
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution of
 *	this particular framebuffer. This function alters the par AND the
 *	fb_fix_screeninfo stored in fb_info. It does not alter var in
 *	fb_info since we are using that data. This means we depend on the
 *	data in var inside fb_info to be supported by the hardware.
 *	omap24xxfb_check_var is always called before omap24xxfb_set_par to
 *	ensure this.
 *
 */

static int
omap24xxfb_set_par(struct fb_info *info)
{
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) info->par;
	struct fb_var_screeninfo *var = &info->var;
	u32 clkdiv, hbp, hfp, hsw, vbp, vfp, vsw, bpp;
	int ret = 0;

	DBGENTER;

	if (oinfo->asleep)
		return 0;

	omap2_disp_get_dss();
	/* update the fix screeninfo */
	if (oinfo->rotation_support) {
		oinfo->rotation_deg = var->rotate;
		info->fix.line_length = MAX_PIXELS_PER_LINE *
					var->bits_per_pixel / 8;
	}
	else
		info->fix.line_length =
				(var->xres_virtual*var->bits_per_pixel)/8;

	info->fix.visual = ((var_to_depth(var) <= 8) ?
			    FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR);

	/* Graphics window parameters */
        if ((oinfo->rotation_support) &&
	    ((oinfo->rotation_deg == 90) || (oinfo->rotation_deg == 270))) {
		omap2_disp_config_gfxlayer(var->yres, var->xres,
					   var_to_depth(var));
	} else {	// no rotation, or 0 or 180 degrees rotation
		omap2_disp_config_gfxlayer(var->xres, var->yres,
					   var_to_depth(var));
	}

	omap2_disp_set_gfx_palette(oinfo->palette_phys);

	/* Rotation */
	if (oinfo->rotation_support) {
		int width = 0, height = 0;

		if ((var->rotate == 90) || (var->rotate == 270)) {
			width = var->xres_virtual;
			height = var->yres_virtual;
		} else {
			width = var->yres_virtual;
			height = var->xres_virtual;
		}
		omap2_disp_set_vrfb(0, oinfo->fb_base_phys,
				    width, height, var->bits_per_pixel/8);

		if(omap24xxfb_mirroring){

			info->fix.smem_start =
				oinfo->sms_rot_phy[omap_rot_mirror_index(oinfo->rotation_deg)];
			info->screen_base    =
				(char *)(oinfo->sms_rot_virt[omap_rot_mirror_index(oinfo->rotation_deg)]);
		}
		else{
			info->fix.smem_start =
				oinfo->sms_rot_phy[omap_rotation_index(oinfo->rotation_deg)];
			info->screen_base    =
				(char *)(oinfo->sms_rot_virt[omap_rotation_index(oinfo->rotation_deg)]);
		}
	}

	/* Graphics DMA parameters */
	ret = omap24xxfb_set_dma_params(var, &info->fix,
			      (oinfo->rotation_support ? var->rotate : -1),
			      omap24xxfb_mirroring);

	/* LCD parameters */
	clkdiv = var->pixclock / oinfo->gfx_clk_period;

	/* horizontal timing parameters in pixclocks */
	hbp = (var->left_margin  > 0) ? (var->left_margin  - 1) : 0;
	hfp = (var->right_margin > 0) ? (var->right_margin - 1) : 0;
	hsw = (var->hsync_len    > 0) ? (var->hsync_len    - 1) : 0;

	/* vertical timing parameters in line clocks */
	vbp = var->upper_margin;
	vfp = var->lower_margin;
	vsw = (var->vsync_len > 0) ? (var->vsync_len - 1) : var->vsync_len;
	bpp = var->bits_per_pixel;

	oinfo->hsync = horizontal_sync_freq(var);
	oinfo->vsync = vertical_sync_freq(var);
	if (oinfo->vsync > 0)
		oinfo->timeout = ((HZ + oinfo->vsync - 1)/oinfo->vsync)*2;
	else
		oinfo->timeout = HZ/5;	/* 200ms default timeout */

	omap2_disp_config_lcd(clkdiv, hbp, hfp, hsw, vbp, vfp, vsw, bpp);

	if (!ret) {
		if(fb_out_layer == OMAP2_GRAPHICS)
			omap2_disp_start_gfxlayer();
		else{
			omap24xxfb_set_output_layer(fb_out_layer);
			omap2_disp_start_gfxlayer();
		}
		wait_for_reg_sync(SCHEDULE_WAIT, oinfo->timeout);
	}
	omap2_disp_put_dss();
	return ret;
}


int
omap24xxfb_set_output_layer(int layer)
{
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) saved_oinfo;
	int ret = 0;
	struct fb_var_screeninfo *var = &oinfo->info.var;

	struct v4l2_pix_format pix;
	struct v4l2_rect crop;
	struct v4l2_window win;
	int rotate = -1;

	memset(&pix, 0, sizeof(pix));
	memset(&crop, 0, sizeof(crop));
	memset(&win, 0, sizeof(win));

	pix.pixelformat = V4L2_PIX_FMT_RGB565;
	pix.colorspace = V4L2_COLORSPACE_SRGB;

	pix.field = V4L2_FIELD_NONE;
	pix.width = var->xres;
	pix.height = var->yres;
	pix.bytesperline = pix.width * 2;
	pix.priv = 0;
	pix.sizeimage = pix.bytesperline * pix.height;

	crop.left = 0;
	crop.top = 0;
	crop.width  = var->xres;
	crop.height = var->yres;

	win.w.width = var->xres;
	win.w.height = var->yres;
	win.w.left = var->xoffset;
	win.w.top = var->yoffset;


	DBGENTER;

	if(layer == OMAP2_GRAPHICS){
		#ifdef DEBUG
			printk("Switching FB to GFX pipeline \n");
		#endif
		omap24xxfb_set_par(&saved_oinfo->info);
	}
	else{
		#ifdef DEBUG
			printk("Switching FB to video pipeline%d \n",layer);
			printk("var.xres=%d, var.yres=%d \n",oinfo->info.var.xres,oinfo->info.var.yres);
			printk("var.xoffset=%d, var.yoffset=%d \n",oinfo->info.var.xoffset,oinfo->info.var.yoffset);
		#endif
			mdelay(1);

			rotate = (oinfo->rotation_support) ? var->rotate : -1;
			omap2_disp_config_vlayer(layer,&pix,&crop,&win,rotate,omap24xxfb_mirroring);
			mdelay(1);
			omap2_disp_start_vlayer(layer,&pix,&crop,&win, (rotate>=0)? SMS_ROT_VIRT_BASE(0,0):oinfo->info.fix.smem_start,rotate,omap24xxfb_mirroring);
			wait_for_reg_sync(SCHEDULE_WAIT, oinfo->timeout);
	}

	DBGLEAVE;
	return ret;

}

/*
 *  	omap24xxfb_setcolreg - Sets a color register.
 *	@regno: Which register in the CLUT we are programming
 *	@red: The red value which can be up to 16 bits wide
 *	@green: The green value which can be up to 16 bits wide
 *	@blue:  The blue value which can be up to 16 bits wide.
 *	@transp: If supported the alpha value which can be up to 16 bits wide.
 *	@info: frame buffer info structure
 *
 *	Returns non-zero on error, or zero on success.
 */
static int
omap24xxfb_setcolreg(unsigned regno, unsigned red, unsigned green,
		     unsigned blue, unsigned transp, struct fb_info *info)
{
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) info->par;
	int output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);

	if (regno >= 256)  /* maximum number of palette entries */
		return 1;

	omap2_disp_get_dss();
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Truecolor has hardware-independent 16-entry pseudo-palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16) {
			omap2_disp_put_dss();
			return 1;
		}

		red    >>= (16 - info->var.red.length);
		green  >>= (16 - info->var.green.length);
		blue   >>= (16 - info->var.blue.length);
		transp >>= (16 - info->var.transp.length);

		v = (red    << info->var.red.offset)   |
		    (green  << info->var.green.offset) |
		    (blue   << info->var.blue.offset)  |
		    (transp << info->var.transp.offset);

		switch (info->var.bits_per_pixel) {
			case 16:
		   		((u16*)(info->pseudo_palette))[regno] = v;
				break;
			case 32:
		   		((u32*)(info->pseudo_palette))[regno] = v;
				break;
			default:
				omap2_disp_put_dss();
				return 1;
		}
		omap2_disp_put_dss();
		return 0;
	}

	red >>= 8;
	green >>= 8;
	blue >>= 8;
	(oinfo->palette)[regno] = (red << 16) | (green << 8) | (blue << 0);
	if(regno == 255){
		omap2_disp_set_gfx_palette(oinfo->palette_phys);
		omap2_disp_reg_sync(output_dev);
	}
	omap2_disp_put_dss();

	DBGLEAVE;
	return 0;
}


/*
 *	omap24xxfb_pan_display - Pans the display.
 *	@var: frame buffer variable screen structure
 *	@info: frame buffer structure that represents a single frame buffer
 *
 *	Pan the display using the `xoffset' and `yoffset' fields of the `var'
 *	structure.  We don't support wrapping and ignore the FB_VMODE_YWRAP
 *	flag.
 *
 *  	If the values don't fit, return -EINVAL.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int
omap24xxfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) info->par;
	int ret;

	DBGENTER;

	if (var->xoffset > info->var.xres_virtual - info->var.xres)
		return -EINVAL;
	if (var->yoffset > info->var.yres_virtual - info->var.yres)
		return -EINVAL;
	if ((info->var.bits_per_pixel < 8) && ((var->xoffset % 8) != 0))
	{
		return -EINVAL;
	}

	if (oinfo->asleep)
		return 0;

	omap2_disp_get_dss();
	/* Graphics DMA parameters */
	ret = omap24xxfb_set_dma_params(var, &info->fix,
		(oinfo->rotation_support ? var->rotate : -1),
		omap24xxfb_mirroring);

	if (!ret) {
		if (fb_out_layer == OMAP2_GRAPHICS)
			omap2_disp_start_gfxlayer();
		else{
			omap2_disp_start_gfxlayer();
			omap24xxfb_set_output_layer(fb_out_layer);
		}

		wait_for_reg_sync(SCHEDULE_WAIT, oinfo->timeout);
	}

	omap2_disp_put_dss();

	DBGLEAVE;
	return ret;
}

/**
 *	  omap24xxfb_blank - NOT a required function. Blanks the display.
 *
 *	  @blank_mode: the blank mode we want.
 *	  @info: frame buffer structure that represents a single frame buffer
 *
 *	  Blank the screen if blank_mode != 0, else unblank.
 */
static int
omap24xxfb_blank(int blank_mode, struct fb_info *info)
{
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) info->par;

	DBGENTER;

	/* fb0 should not be blanked in WebOS as it needs to be available at all times.
	 * Simply return 0 and let fbmem continues to send out fb notifications, but
	 * no actions will be performed */
#if defined(CONFIG_MACH_SIRLOIN) || defined(CONFIG_MACH_SIRLOIN_3630)
	(void) oinfo; /* unused */
#else
	/* No need for suspend lockout because if the framebuffer device is
	 * suspended, then the console is already blanked.
	 */
	if (oinfo->asleep)
		return 0;

	if ((blank_mode) && (!oinfo->blanked)) {
		/* blank--disable display controller */
		/* wait until Vsync */
		if (omap24xxfb_wait_for_vsync(oinfo) != 0)
			printk("omap24xxfb_blank: vsyn wait failed \n");
		omap2_disp_disable_layer(OMAP2_GRAPHICS);
		omap2_disp_disable_layer(fb_out_layer);
		omap2_disp_disable(HZ/5);
		disable_backlight();
		omap2_disp_put_dss();
		omap2_dss_rgb_disable();
		oinfo->blanked = 1;
	}
	else if ((!blank_mode) && (oinfo->blanked)) {
		/* unblank--enable display controller */
		printk(KERN_ERR "Unblanking the display\n");
		omap2_dss_rgb_enable();
		// omap2_disp_get_dss();
		omap2_disp_enable_layer(OMAP2_GRAPHICS);
		// omap2_disp_enable_layer(fb_out_layer);
		omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
		// omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);
		oinfo->blanked = 0;
		udelay(20);
		enable_backlight();
	}
#endif

	DBGLEAVE;
	return 0;
}

/**
 *	omap24xxfb_sync - NOT a required function. Normally the accel engine
 *			  for a graphics card take a specific amount of time.
 *			  Often we have to wait for the accelerator to finish
 *			  its operation before we can write to the framebuffer
 *			  so we can have consistent display output.
 *
 *	@info: frame buffer structure that represents a single frame buffer
 */
static int
omap24xxfb_sync(struct fb_info *info)
{
	return 0;
}

/**
 *	omap24xxfb_set_mirroring - enables and disables mirroring
 */
static int
omap24xxfb_set_mirroring (struct fb_info *info, int mirroring)
{
	int ret = 0;
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) info->par;

	omap24xxfb_mirroring = mirroring;
	ret = omap24xxfb_set_dma_params(&info->var, &info->fix, (oinfo->rotation_support ? info->var.rotate : -1), omap24xxfb_mirroring);
	if (!ret)
	{
		if (fb_out_layer == OMAP2_GRAPHICS)
			omap2_disp_start_gfxlayer();
		else{
			omap24xxfb_set_output_layer(fb_out_layer);
			omap2_disp_start_gfxlayer();
		}


		wait_for_reg_sync(SCHEDULE_WAIT, oinfo->timeout);
	}

	return ret;
}

/**
 *	omap24xxfb_ioctl - handler for private ioctls.
 */
static int
omap24xxfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)

{
	struct omap24xxfb_info *oinfo = (struct omap24xxfb_info *) info->par;
	void __user *argp = (void __user *)arg;
	int mirroring;
	int ret;
#if defined(CONFIG_LCD_IOCTL) && \
    !defined(CONFIG_MACH_FLANK) && \
    !defined(CONFIG_MACH_BRISKET) && \
    !defined(CONFIG_MACH_SIRLOIN)
	struct omap_lcd_info *lcd_inf;
#endif

	DBGENTER;

	omap24xxfb_suspend_lockout(&fb_suspend_data);

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		/* This ioctl accepts an integer argument to specify a
		 * display.  We only support one display, so we will
		 * simply ignore the argument.
		 */
		omap2_disp_get_dss();
		ret = omap24xxfb_wait_for_vsync(oinfo);
		omap2_disp_put_dss();
		return ret;

	case FBIO_MIRROR:
		/* This ioctl accepts an integer argument to specify the
		 * mirroring status.
		 */
		if (!copy_from_user(&mirroring, argp, sizeof(mirroring))) {
			omap2_disp_get_dss();
			ret = omap24xxfb_set_mirroring (info, mirroring);
			omap2_disp_put_dss();
			return ret;
		}
		break;

#if defined(CONFIG_LCD_IOCTL) && \
    !defined(CONFIG_MACH_FLANK) && \
    !defined(CONFIG_MACH_BRISKET) && \
    !defined(CONFIG_MACH_SIRLOIN)
		/* This ioctl accepts a structure of type lcd_info and
		 * uses that to configure the LCD
		 */
	case FBIO_LCD_PUT_SCREENINFO:
		lcd_inf = (struct omap_lcd_info *)arg;
		omap2_disp_get_dss();
		ret = omap_lcd_init(lcd_inf);
		omap2_disp_put_dss();
		return ret;

#endif
	}
	DBGLEAVE;
	return -EINVAL;
}

/*---------------------------------------------------------------------------*/
	/*
	 *  Frame buffer operations
	 */

static struct fb_ops omap24xxfb_ops = {
	.owner		=	THIS_MODULE,
	.fb_check_var	= omap24xxfb_check_var,
	.fb_set_par	= omap24xxfb_set_par,
	.fb_setcolreg	= omap24xxfb_setcolreg,
	.fb_blank	= omap24xxfb_blank,
	.fb_pan_display	= omap24xxfb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,

	.fb_rotate	= NULL,
	.fb_sync	= omap24xxfb_sync,
	.fb_ioctl	= omap24xxfb_ioctl,
};

/*---------------------------------------------------------------------------*/
#ifdef CONFIG_PM

#ifndef CONFIG_MACH_SIRLOIN
static int awake = 0;

/* PM suspend */
static int omap24xxfb_suspend(struct platform_device *dev, pm_message_t state)
{
	struct omap24xxfb_info *oinfo = saved_oinfo;

	acquire_console_sem();
	fb_set_suspend(&oinfo->info, 1);
	release_console_sem();

	printk(KERN_DEBUG "Suspend Check Awake\n");
	if (!awake)
		return 0;

	printk(KERN_DEBUG "Disable LCD\n");

	omap24xxfb_blank(1, &oinfo->info);
	awake = 0;
	oinfo->asleep = 1;

	return 0;
}

/* PM resume */
	/* Don't implement resume for LCD here. Resume will be handled by a
	 * userspace component via sysfs.
	 */
static int omap24xxfb_resume(struct platform_device *dev)
{
	struct omap24xxfb_info *oinfo = saved_oinfo;

	printk(KERN_DEBUG "Resume Check Awake\n");

	/* If already awake then leave */
	if (awake)
		return 0;

	printk(KERN_DEBUG "Enable LCD\n");
	oinfo->asleep = 0;

	/* unblank the screen */
	omap24xxfb_blank(0, &oinfo->info);
	awake = 1;
	acquire_console_sem();
	fb_set_suspend(&oinfo->info, 0);
	release_console_sem();

	return 0;
}

static ssize_t show_omap24xxfb_suspend(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	int count;

	struct omap24xxfb_info *oinfo = saved_oinfo;

	acquire_console_sem();
	fb_set_suspend(&oinfo->info, 1);
	release_console_sem();

	if (!awake) {
		count = snprintf(buf, PAGE_SIZE, "Err:  Not awake!\n");
		return count;
	}

	omap24xxfb_blank(1, &oinfo->info);
	awake = 0;
	oinfo->asleep = 1;

	count = snprintf(buf, PAGE_SIZE, "omap24xxfb suspended\n");

	return count;

}

static ssize_t show_omap24xxfb_resume(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	int count;
	struct omap24xxfb_info *oinfo = saved_oinfo;


	/* If already awake then leave */
	if (awake) {
		count = snprintf(buf, PAGE_SIZE, "Err:  already awake!\n");
		return 0;
	}

	oinfo->asleep = 0;

	/* unblank the screen */
	omap24xxfb_blank(0, &oinfo->info);
	awake = 1;
	acquire_console_sem();
	fb_set_suspend(&oinfo->info, 0);
	release_console_sem();

	count = snprintf(buf, PAGE_SIZE, "omap24xxfb resumed\n");

	return count;

}

DEVICE_ATTR(omap24xxfb_suspend, S_IRUGO, show_omap24xxfb_suspend, NULL);
DEVICE_ATTR(omap24xxfb_resume,  S_IRUGO, show_omap24xxfb_resume,  NULL);


#endif /* #ifndef CONFIG_MACH_SIRLOIN */

#endif /* #ifdef CONFIG_PM */

#ifdef CONFIG_DPM

static struct constraints omap24xxfb_constraints = {
	.count = 2,
	.param = {
		{DPM_MD_V, OMAP24XX_V_MIN, OMAP24XX_V_MAX},
		{DPM_MD_SLEEP_MODE, PM_SUSPEND_STANDBY, PM_SUSPEND_MEM},
	},
};

/* This routine is called by DPM to notify the driver before and after a
 * frequency scale. The pixel clock divisor has to be changed after a
 * frequency change to maintain the refresh rate.
 */
static int
omap24xxfb_scale(struct notifier_block *op, unsigned long level, void *ptr)
{
	struct omap24xxfb_info *oinfo = saved_oinfo;
	struct fb_info *info;
	u32 clkdiv, pixclock;
	struct fb_videomode mode;
	int output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);
	info = &oinfo->info;

	switch (level) {
	case SCALE_PRECHANGE:
		/* disable the graphics layer because there should be
		   no memory accesses (possibly through DMA) while
		   scaling frequency, on boards with DDR memory - Errata
		*/
		omap2_disp_disable_layer(OMAP2_GRAPHICS);
		wait_for_reg_sync(BUSY_WAIT, oinfo->timeout);
	break;
	case SCALE_POSTCHANGE:
		/*
		 * The Core frequency might have dropped resulting in DSS fclk drop
		 * But we would like to continue with the same DSS fclk
		 */
		omap2_disp_set_dssfclk();

		/* set the PCD divisor according to the new gfx_clock_period.
		   gfx_clock_period is in picoseconds
		   dss1_clk is in kHz */
		oinfo->gfx_clk_period = omap24xxfb_gfx_clk_period();

		clkdiv = info->var.pixclock / oinfo->gfx_clk_period;
		pixclock = oinfo->gfx_clk_period * clkdiv;

		/* due to round-down error in division, the pixclock
		   may fall below the lower threshold of the panel.
		   Fix that by adding 1 to clkdiv.
		*/
		if (pixclock < omap24xx_display_pixclock_min(output_dev))
			clkdiv = clkdiv + 1;
		if (clkdiv < 2)		/* minimum divisor is 2 */
			clkdiv = 2;
		else if (clkdiv > 255)
			clkdiv = 255;	/* maximum divisor is 255 */

		/* recalculate pixclock and change the var structure */
		pixclock = oinfo->gfx_clk_period * clkdiv;
		info->var.pixclock = pixclock;

		fb_var_to_videomode(&mode, &info->var);
		fb_add_videomode(&mode, &info->modelist);

		omap2_disp_set_pcd(clkdiv);
		/* enable the graphics layer and the display controller */
		omap2_disp_enable_layer(OMAP2_GRAPHICS);
	break;
	}
	return 0;
}

static struct notifier_block omap24xxfb_pre_scale = {
	.notifier_call = omap24xxfb_scale,
};

static struct notifier_block omap24xxfb_post_scale = {
	.notifier_call = omap24xxfb_scale,
};

#endif

static int
omap24xxfb_probe (struct platform_device *pdev)
{
#ifdef CONFIG_PM
#ifndef CONFIG_MACH_SIRLOIN

	if (device_create_file(&pdev->dev, &dev_attr_omap24xxfb_suspend) < 0)
		printk(KERN_ERR
			"Creating sysfs entry for omap24xxfb_suspend failed\n");
	if (device_create_file(&pdev->dev, &dev_attr_omap24xxfb_resume) < 0)
		printk(KERN_ERR
			"Creating sysfs entry for omap24xxfb_resume failed\n");

#endif
#endif
	return 0;
}

static int
omap24xxfb_release (struct platform_device *pdev)
{
#ifdef CONFIG_PM
#ifndef CONFIG_MACH_SIRLOIN

	device_remove_file(&pdev->dev, &dev_attr_omap24xxfb_suspend);
	device_remove_file(&pdev->dev, &dev_attr_omap24xxfb_resume);

#endif
#endif
	return 0;
}

static struct platform_driver omap24xxfb_driver = {
	.driver = {
		.name   = OMAPFB_DRIVER,
	},
	.probe          = omap24xxfb_probe,
	.remove		= omap24xxfb_release,
#ifdef CONFIG_PM
#ifdef CONFIG_MACH_SIRLOIN
	/* Don't implement resume for LCD here. Resume will be handled by a
	 * userspace component via sysfs.
	 */
	.suspend         = NULL,
	.resume         = NULL,
#else
	.suspend        = omap24xxfb_suspend,
	.resume         = omap24xxfb_resume,
#endif
#endif
};

static struct platform_device omap24xxfb_device = {
	.name     = OMAPFB_DEVICE,
	.id    = 6,

	.dev = {
#ifdef CONFIG_DPM
		.constraints    = &omap24xxfb_constraints,
#endif
	},
};

int omap24xx_get_dss1_clock(void)
{
	struct clk *dss1f;

#if defined(CONFIG_ARCH_OMAP24XX)
	dss1f = clk_get(NULL, "dss1_fck");
#else
	dss1f = clk_get(NULL, "dss1_alwon_fck");
#endif	
	return clk_get_rate(dss1f);
}

/*---------------------------------------------------------------------------*/

#ifndef MODULE
int omap24xxfb_setup(char *options)
{
	char *this_opt;

	printk(KERN_INFO "omap24xxfb: Options \"%s\"\n", options);

	if (!options || !*options)
		return 0;

	while((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)	continue;
		if (!strncmp(this_opt, "rotation=", 9)) {
			int deg = simple_strtoul(this_opt + 9, NULL, 0);

			if (deg == 0 || deg == 90 || deg == 180 || deg == 270)
				omap24xxfb_rotation = deg;
			else
				omap24xxfb_rotation = -1;

			printk(KERN_INFO "omap24xxfb: Rotation %s\n",
				(omap24xxfb_rotation == -1) ?
				"none (supported: \"rotation=[0|90|180|270]\")":
				this_opt);
		} else
			printk(KERN_INFO "omap24xxfb: Invalid parameter \"%s\" "
				"(supported: \"rotation=[0|90|180|270]\")\n",
				this_opt);
	}
	return 0;
}
#endif

#ifdef DEBUG_VRFB
void test_vrfb_line(struct omap24xxfb_info *oinfo,
		    int line, int from, int upto, int total)
{
	int j=0;

	for (j=from; j<upto; j+=4) {
		*((u32 *)(oinfo->sms_rot_virt[0] + line*total + j)) = 0xf1f1e1e1;
	}
}

void test_vrfb_space(struct omap24xxfb_info *oinfo,
		     int lines, int from, int upto, int total)
{
	int i=0;
	printk("test_vrfb_space: lines %d, from %d to %d\n", lines, from, upto);
	for (i=0; i<lines; i++) {
		test_vrfb_line(oinfo, i, from, upto, total);
	}
}
#endif

static ssize_t
read_fb_out(char *buf)
{
	int p = 0;

	switch(fb_out_layer){
		case OMAP2_GRAPHICS: p = sprintf(buf, "gfx\n");
		break;
		case OMAP2_VIDEO1: p = sprintf(buf, "video1\n");
		break;
		case OMAP2_VIDEO2: p = sprintf(buf, "video2\n");
		break;
	}

	return(p);
}

static ssize_t
write_fb_out(const char *buffer, size_t count)
{
	int out_layer;

	if(!buffer || (count == 0))
		return 0;

	/* Only 'lcd' or 'tv' are valid inputs */
	if(strncmp(buffer, "gmx", 3) == 0)
		out_layer = OMAP2_GRAPHICS;
	else if(strncmp(buffer, "video1", 6) == 0)
		out_layer = OMAP2_VIDEO1;
	else if(strncmp(buffer, "video2", 6) == 0)
		out_layer = OMAP2_VIDEO2;
	else
		return -EINVAL;

	if(out_layer != fb_out_layer){
		/* switch the output layer */
		/* check if the pipeline is available */
		if(omap2_disp_request_layer(out_layer)){
			omap2_disp_start_gfxlayer();
			fb_out_layer = out_layer;
			omap24xxfb_set_output_layer(out_layer);
		}
	}

	return count;
}

ssize_t
fb_out_show(struct class_device *cdev, char *buf)
{
	return(read_fb_out(buf));
}

ssize_t
fb_out_store(struct class_device *cdev, const char *buffer, size_t count)
{
	return(write_fb_out(buffer,count));
}

/*
 * PALM
 * allow the framebuffer's address to be passed from the bootloader on the command line
 */
struct fbargs {
	int valid;
	unsigned long fb_phys;
};

static struct fbargs fbargs;

static int __init
fb_args(char *str)
{
	/*
	 * format is
	 * fb=<address>
	 */
	fbargs.fb_phys = simple_strtoul(str, NULL, 0);
	fbargs.valid = 1;

	return 0;
}

__setup("fb=", fb_args);

/*
 *  Initialization
 */
int __init omap24xxfb_init(void)
{
#ifdef CONFIG_MACH_SIRLOIN
	int idx;
	u32 col;
#endif // CONFIG_MACH_SIRLOIN
	struct omap24xxfb_info *oinfo;
	struct fb_info *info;
	int output_dev = omap2_disp_get_output_dev(OMAP2_GRAPHICS);

	DBGENTER;

	/* Register the driver with LDM */
	if (platform_driver_register(&omap24xxfb_driver)) {
		printk(KERN_ERR FB_NAME ": failed to register omapfb driver\n");
		return -ENODEV;
	}

	/* Register the device with LDM */
	if (platform_device_register(&omap24xxfb_device)) {
		printk(KERN_ERR FB_NAME ": failed to register omapfb device\n");
		goto unregister_driver;
	}

#ifdef CONFIG_DPM
	dpm_register_scale(&omap24xxfb_pre_scale,SCALE_PRECHANGE);
	dpm_register_scale(&omap24xxfb_post_scale,SCALE_POSTCHANGE);
#endif

#ifdef CONFIG_PM
#ifndef CONFIG_MACH_SIRLOIN
	awake = 1;
#endif
	fb_suspend_data.suspended = 0;
	init_waitqueue_head(&fb_suspend_data.suspend_wq);
#endif

#ifndef MODULE
	{
		char *option;

		if (fb_get_options("omap24xxfb", &option)) {
			return -ENODEV;
		}
		omap24xxfb_setup(option);     // parse options
	}
#endif

	oinfo = kmalloc(sizeof(struct omap24xxfb_info), GFP_KERNEL);
	if (!oinfo) {
		printk(KERN_ERR FB_NAME ": could not allocate memory\n");
		goto unregister_device;
	}
	memset(oinfo, 0, sizeof(struct omap24xxfb_info));
	saved_oinfo = oinfo;

	oinfo->rotation_support = (omap24xxfb_rotation >= 0) ? 1 : 0;

	/* set the period (in picoseconds) of the graphics timebase */
	oinfo->gfx_clk_period = omap24xxfb_gfx_clk_period();

	/* Set base addrs */
	if (fbargs.valid) {
		/* passed in from the bootloader */
		oinfo->fb_base_phys = fbargs.fb_phys;
	} else {
		oinfo->fb_base_phys = omap24xxfb_fb_base();
	}

	oinfo->fb_size = omap24xxfb_fb_size(oinfo->rotation_support);

	if (oinfo->rotation_support) {
		oinfo->vrfb_size = omap24xxfb_vrfb_size();
		oinfo->rotation_deg = omap24xxfb_rotation;
	}

	oinfo->mmio_base_phys = DSS_REG_BASE;

	/* A null base address indicates that the framebuffer memory should be
	 * dynamically allocated.
	 */
	if (!oinfo->fb_base_phys)
		oinfo->alloc_fb_mem = 1;

	/* request the mem regions */
	if (!request_mem_region(oinfo->mmio_base_phys, DSS_REG_SIZE, FB_NAME))
	{
		printk(KERN_ERR FB_NAME ": cannot reserve MMIO region\n");
		goto free_par;
	}

	if (!oinfo->alloc_fb_mem) {
		if (!request_mem_region(oinfo->fb_base_phys,
					oinfo->fb_size, FB_NAME))
		{
			printk(KERN_ERR FB_NAME ": cannot reserve FB region\n");
			goto release_mmio;
		}
	}

	if (oinfo->rotation_support) {
		oinfo->sms_rot_phy[0] = SMS_ROT_VIRT_BASE(0, 0);
		oinfo->sms_rot_phy[1] = SMS_ROT_VIRT_BASE(0, 90);
		oinfo->sms_rot_phy[2] = SMS_ROT_VIRT_BASE(0, 180);
		oinfo->sms_rot_phy[3] = SMS_ROT_VIRT_BASE(0, 270);

		if (!request_mem_region(oinfo->sms_rot_phy[0],
					oinfo->vrfb_size, FB_NAME)) {
			printk(KERN_ERR FB_NAME
			       ": cannot reserve FB region for 0 deg\n");
			goto release_fb;
		}
		if (!request_mem_region(oinfo->sms_rot_phy[1],
					oinfo->vrfb_size, FB_NAME)) {
			printk(KERN_ERR FB_NAME
			       ": cannot reserve FB region for 90 deg\n");
			goto release_rot_0;
		}
		if (!request_mem_region(oinfo->sms_rot_phy[2],
					oinfo->vrfb_size, FB_NAME)) {
			printk(KERN_ERR FB_NAME
			       ": cannot reserve FB region for 180 deg\n");
			goto release_rot_90;
		}
		if (!request_mem_region(oinfo->sms_rot_phy[3],
					oinfo->vrfb_size, FB_NAME)) {
			printk(KERN_ERR FB_NAME
			       ": cannot reserve FB region for 270 deg\n");
			goto release_rot_180;
		}
	}

	/* map the regions */
	oinfo->mmio_base =
		(unsigned long) ioremap(oinfo->mmio_base_phys, DSS_REG_SIZE);
	if (!oinfo->mmio_base) {
		printk(KERN_ERR FB_NAME ": cannot map MMIO\n");
		if (oinfo->rotation_support)
			goto release_rot_270;
		else
			goto release_fb;
	}

	if (!oinfo->alloc_fb_mem) {
		oinfo->fb_base =
			(unsigned long) ioremap(oinfo->fb_base_phys,
						oinfo->fb_size);
		if (!oinfo->fb_base) {
			printk(KERN_ERR FB_NAME ": cannot map framebuffer\n");
			goto unmap_mmio;
		}
	}
	else {
		/* allocate coherent memory for the framebuffer */
		oinfo->fb_base = (unsigned long) dma_alloc_coherent(NULL,
				oinfo->fb_size, &oinfo->fb_base_phys,
				GFP_KERNEL | GFP_DMA);
		if (!oinfo->fb_base) {
			printk(KERN_ERR FB_NAME ": cannot allocate "
				"framebuffer\n");
			goto unmap_mmio;
		}
		memset((void *)oinfo->fb_base, 0, oinfo->fb_size);
	}

	if (oinfo->rotation_support) {
	       if (!(oinfo->sms_rot_virt[0] =
			(unsigned long) ioremap(oinfo->sms_rot_phy[0],
						oinfo->vrfb_size)) 	||
		   !(oinfo->sms_rot_virt[1] =
			(unsigned long) ioremap(oinfo->sms_rot_phy[1],
						oinfo->vrfb_size)) 	||
		   !(oinfo->sms_rot_virt[2] =
			(unsigned long) ioremap(oinfo->sms_rot_phy[2],
						oinfo->vrfb_size)) 	||
		   !(oinfo->sms_rot_virt[3] =
			(unsigned long) ioremap(oinfo->sms_rot_phy[3],
						oinfo->vrfb_size)))
	       {
		       printk(KERN_ERR FB_NAME ": cannot map rotated view(s)\n");
		       goto unmap_rot;
	       }
	}

	/* allocate coherent memory for the palette */
	oinfo->palette = (u32 *) dma_alloc_coherent(NULL, sizeof(u32)*256,
				&oinfo->palette_phys, GFP_KERNEL | GFP_DMA);
	if (!oinfo->palette) {
		printk(KERN_ERR FB_NAME ": cannot allocate palette memory\n");
		goto unmap_rot;
	}
#ifdef CONFIG_MACH_SIRLOIN
	for (idx = 0, col = 0; 256 > idx; ++idx, col = idx >> 2)
		oinfo->palette[idx] = (col << 16) | (col << 8) | col;
#else // !CONFIG_MACH_SIRLOIN
	memset(oinfo->palette, 0, sizeof(u32)*256);
#endif // CONFIG_MACH_SIRLOIN

        if(!omap2_disp_request_layer(OMAP2_GRAPHICS)) {
		printk(KERN_ERR FB_NAME ": cannot use hw graphics layer\n");
		goto free_palette;
	}

	/* first enable the power to RGB lines.. then turhn on the LCD..
	 * as per the SHARP LCD specs */
	omap2_dss_rgb_enable();
	enable_backlight();

	/* initialize the fb_info structure */
	info = &oinfo->info;
	info->flags = FBINFO_DEFAULT;
	info->fbops = &omap24xxfb_ops;

	if (oinfo->rotation_support){
		/* If mirroring in rotation is enabled by default */
		if(omap24xxfb_mirroring == 1){
			info->screen_base =
				(char *) (oinfo->sms_rot_virt[omap_rot_mirror_index(oinfo->rotation_deg)]);
		}
		else{
			info->screen_base = (char *)
				(oinfo->sms_rot_virt[omap_rotation_index(oinfo->rotation_deg)]);
		}
	}
	else
		info->screen_base = (char *)(oinfo->fb_base);

	info->pseudo_palette = oinfo->pseudo_palette;
	info->par = oinfo;
	/* Initialize variable screeninfo.
	 * The variable screeninfo can be directly specified by the user
	 * via an ioctl.
	 */
	memcpy(&info->var, omap24xxfb_default_var(), sizeof(info->var));
	info->var.activate = FB_ACTIVATE_NOW;

	/* Initialize fixed screeninfo.
	 * The fixed screeninfo cannot be directly specified by the user, but
	 * it may change to reflect changes to the var info.
	 */
	strlcpy(info->fix.id, FB_NAME, sizeof(info->fix.id));
	if (oinfo->rotation_support) {
		/* If mirroring in rotation is enabled by default */
		if(omap24xxfb_mirroring == 1){
			info->fix.smem_start =
				oinfo->sms_rot_phy[omap_rot_mirror_index(oinfo->rotation_deg)];
		}
		else{
			info->fix.smem_start =
				oinfo->sms_rot_phy[omap_rotation_index(oinfo->rotation_deg)];
		}
		info->fix.line_length = MAX_PIXELS_PER_LINE *
					info->var.bits_per_pixel / 8;
	} else {
		info->fix.smem_start = oinfo->fb_base_phys;
		info->fix.line_length =
			(info->var.xres_virtual*info->var.bits_per_pixel)/8;
	}

	if (oinfo->rotation_support)
		info->fix.smem_len = oinfo->vrfb_size;
	else
		info->fix.smem_len = oinfo->fb_size;

	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = ((var_to_depth(&info->var) <= 8) ?
				FB_VISUAL_PSEUDOCOLOR: FB_VISUAL_TRUECOLOR);
	info->fix.xpanstep = 0;
	info->fix.ypanstep = 1;
	if(oinfo->rotation_support)
	info->fix.ypanstep = 0;
	info->fix.ywrapstep = 0;
	info->fix.line_length =
		(info->var.xres_virtual*info->var.bits_per_pixel)/8;
	info->fix.type_aux = 0;
	info->fix.mmio_start = oinfo->mmio_base_phys;
	info->fix.mmio_len = DSS_REG_SIZE;
	info->fix.accel = FB_ACCEL_NONE;

	/* Allocate the color map to the maximum size.
	 * Color depths greater than 8 bits are true color and use a 16-bit
	 * pseudo-palette instead of an actual palette, so the maximum color
	 * map size is 256 colors.
	 */

	if (fb_alloc_cmap(&info->cmap, 256, 0) < 0) {
		printk(KERN_ERR FB_NAME ": cannot allocate color map\n");
		goto put_dss;
	}
	if (omap24xxfb_check_var(&info->var, info)) {
		printk(KERN_ERR FB_NAME ": invalid default video mode\n");
		goto dealloc_cmap;
	}

	oinfo->timeout = HZ/5;

	/* initialize the vsync wait queue */
	init_waitqueue_head(&oinfo->vsync_wait);

	/* and related variables */
	spin_lock_init(&oinfo->vsync_lock);
	oinfo->vsync_ref = 0;
	

	/* install our interrupt service routine */
	if (omap2_disp_register_isr(omap24xxfb_isr, oinfo,0)) {
		printk(KERN_ERR FB_NAME
			": could not install interrupt service routine\n");
		goto dealloc_cmap;
	};
#ifdef DEBUG
	omap2_disp_register_isr(omap24xxfb_synclost_isr, oinfo,
			DISPC_IRQSTATUS_SYNCLOSTDIGITAL);
#endif

	omap24xxfb_set_par(info);

	if (register_framebuffer((struct fb_info *) oinfo) < 0) {
		printk(KERN_ERR FB_NAME ": could not register framebuffer\n");
		goto uninstall_isr;
	}

#ifdef CONFIG_ARCH_OMAP34XX
	omap2_disp_set_alphablend(OMAP2_OUTPUT_LCD, 1);
	omap2_disp_set_global_alphablend_value(OMAP2_GRAPHICS, 0xff);
	omap2_disp_set_global_alphablend_value(OMAP2_VIDEO2, 0xff);
#endif

	/* unblank--enable display controller */
	omap2_disp_enable_layer(OMAP2_GRAPHICS);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_LCD);
	omap2_disp_enable_output_dev(OMAP2_OUTPUT_TV);

	printk(KERN_INFO FB_NAME ": fb%d frame buffer device %s\n",
			info->node,
			(oinfo->rotation_support) ? "(rotation support)" : "");

	printk(KERN_INFO FB_NAME ": display mode %dx%dx%d",
		info->var.xres, info->var.yres,
		var_to_depth(&info->var));

	if (oinfo->rotation_support)
		printk("@%d ", info->var.rotate);

	if (output_dev == OMAP2_OUTPUT_LCD){
		printk(" hsync %dkHz vsync %dHz",
			oinfo->hsync/1000, oinfo->vsync);
	}
	printk("\n");

#ifdef DEBUG_VRFB
	if (oinfo->rotation_support) {
		printk("lines %d, width %d  line_length %d \n",
		       info->var.yres, info->var.xres, info->fix.line_length);
		test_vrfb_space(oinfo, info->var.yres, 0, info->var.xres*2,
				info->fix.line_length);
		test_vrfb_space(oinfo, info->var.yres, info->var.xres*2,
				info->fix.line_length, info->fix.line_length);
	}
#endif
	omap2_disp_save_initstate(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_save_initstate(OMAP2_GRAPHICS);
	DBGLEAVE;
	return 0;

uninstall_isr:
		free_irq(INT_24XX_DSS_IRQ, oinfo);


dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);

put_dss:

	omap2_disp_release_layer(fb_out_layer);
	disable_backlight();
	omap2_dss_rgb_disable();

free_palette:
	dma_free_coherent(NULL, sizeof(u32)*256, oinfo->palette,
				oinfo->palette_phys);

	if (!oinfo->rotation_support)
		goto free_fb;
unmap_rot:
	if (oinfo->sms_rot_virt[0])
		iounmap((void*)oinfo->sms_rot_virt[0]);
	if (oinfo->sms_rot_virt[1])
		iounmap((void*)oinfo->sms_rot_virt[1]);
	if (oinfo->sms_rot_virt[2])
		iounmap((void*)oinfo->sms_rot_virt[2]);
	if (oinfo->sms_rot_virt[3])
		iounmap((void*)oinfo->sms_rot_virt[3]);

free_fb:
	if (!oinfo->alloc_fb_mem)
		iounmap((void*)oinfo->fb_base);
	else
		dma_free_coherent(NULL, oinfo->fb_size, (void *)oinfo->fb_base,
					oinfo->fb_base_phys);
unmap_mmio:
	iounmap((void*)oinfo->mmio_base);

	if (!oinfo->rotation_support)
		goto release_fb;
release_rot_270:
	release_mem_region(oinfo->sms_rot_phy[0], oinfo->vrfb_size);
release_rot_180:
	release_mem_region(oinfo->sms_rot_phy[1], oinfo->vrfb_size);
release_rot_90:
	release_mem_region(oinfo->sms_rot_phy[2], oinfo->vrfb_size);
release_rot_0:
	release_mem_region(oinfo->sms_rot_phy[3], oinfo->vrfb_size);

release_fb:
	if (!oinfo->alloc_fb_mem)
		release_mem_region(oinfo->fb_base_phys, oinfo->fb_size);

release_mmio:
	release_mem_region(oinfo->mmio_base_phys, DSS_REG_SIZE);

free_par:
	kfree(oinfo);
	saved_oinfo = NULL;

unregister_device:
#ifdef CONFIG_DPM
	dpm_unregister_scale(&omap24xxfb_pre_scale,SCALE_PRECHANGE);
	dpm_unregister_scale(&omap24xxfb_post_scale,SCALE_POSTCHANGE);
#endif
	platform_device_unregister(&omap24xxfb_device);
unregister_driver:
	platform_driver_unregister(&omap24xxfb_driver);
	DBGLEAVE;
	return -ENODEV;
}

/*
 *  Cleanup
 */
static void __exit omap24xxfb_cleanup(void)
{
	struct omap24xxfb_info *oinfo = saved_oinfo;

	DBGENTER;


	if (!oinfo)
		return;

	if(unregister_framebuffer((struct fb_info *) oinfo) < 0)
		printk(KERN_ERR "unregister framebuffer error\n");

#ifdef CONFIG_DPM
	dpm_unregister_scale(&omap24xxfb_pre_scale,SCALE_PRECHANGE);
	dpm_unregister_scale(&omap24xxfb_post_scale,SCALE_POSTCHANGE);
#endif
	platform_device_unregister(&omap24xxfb_device);
	platform_driver_unregister(&omap24xxfb_driver);


	omap2_disp_disable_layer(OMAP2_GRAPHICS);
	omap2_disp_disable_layer(fb_out_layer);
	disable_backlight();
	omap2_dss_rgb_disable();

	omap2_disp_release_layer(fb_out_layer);


	/* uninstall isr */
 	free_irq(INT_24XX_DSS_IRQ, oinfo);

	fb_dealloc_cmap(&oinfo->info.cmap);

	dma_free_coherent(NULL, sizeof(u32)*256, oinfo->palette,
			  oinfo->palette_phys);

	if (!oinfo->alloc_fb_mem)
		iounmap((void*)oinfo->fb_base);
	else
		dma_free_coherent(NULL, oinfo->fb_size,
				(void *)oinfo->fb_base, oinfo->fb_base_phys);
	iounmap((void*)oinfo->mmio_base);

	if (!oinfo->alloc_fb_mem)
		release_mem_region(oinfo->fb_base_phys, oinfo->fb_size);

	release_mem_region(oinfo->mmio_base_phys, DSS_REG_SIZE);

	if (oinfo->rotation_support) {
		iounmap((void*)oinfo->sms_rot_virt[0]);
		iounmap((void*)oinfo->sms_rot_virt[1]);
		iounmap((void*)oinfo->sms_rot_virt[2]);
		iounmap((void*)oinfo->sms_rot_virt[3]);

		release_mem_region(oinfo->sms_rot_phy[0], oinfo->vrfb_size);
		release_mem_region(oinfo->sms_rot_phy[1], oinfo->vrfb_size);
		release_mem_region(oinfo->sms_rot_phy[2], oinfo->vrfb_size);
		release_mem_region(oinfo->sms_rot_phy[3], oinfo->vrfb_size);
	}

	kfree(oinfo);
	saved_oinfo = NULL;


	printk(KERN_DEBUG "Removed framebuffer module\n");
	DBGLEAVE;
}

/*
 *  Modularization
 */

module_init(omap24xxfb_init);
module_exit(omap24xxfb_cleanup);
EXPORT_SYMBOL(omap24xxfb_display_wait_for_vsync);

MODULE_LICENSE("GPL");

