
/*
 * drivers/video/omap/omap_fbv.c
 *
 * driver for the omap video frame buffer
 *
 * Copyright (c) 2009 Bennett Chan  <bennett.chan@palm.com>
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

#include <linux/types.h>
#include <linux/wait.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/dma-mapping.h>

#ifdef CONFIG_PM

#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#include <asm/arch/display.h>
#include <asm/arch/lcd.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/semaphore.h>
#include <asm/processor.h>
#include <asm/arch/dma.h>

#if 0
#define DPRINTK(args...) printk(KERN_DEBUG __FILE__": " ##args)
#else
#define DPRINTK(args...)
#endif


#define MODULENAME		"omap_fbv"
#define DEVICENAME		MODULENAME

#define OMAP_FBV_DEVICE_FIX_ID "omap_fbv-"

MODULE_SUPPORTED_DEVICE(DEVNAME);

#define FBIO_WAITFORVSYNC	_IOW('F', 0x20, u_int32_t)

#define LOCAL_ID  0x6f6d6676

extern int omap24xxfb_display_wait_for_vsync(void);

/* There are many reasons for implmenting this as a separate module 
 * with limited capabilites as opposed to being more generic.  
 */

struct omapfbv_device {
	u32    local_id;
	int	   	video_layer;
	struct device	*dev;
	struct fb_info	*fb_info;
	
	unsigned long framebuffer_base;
	unsigned long framebuffer_base_phys;
	unsigned long framebuffer_size;
	unsigned long framebuffer_pan_base_phys;

	u32	xres;
	u32	yres;
	u32	xres_virtual;
	u32	yres_virtual;
	u32	pixelformat;
	enum v4l2_colorspace	colorspace;

	struct v4l2_pix_format	pix;
	struct v4l2_rect	crop;
	struct v4l2_window	win;

	int mirror;
	int rotation;

	struct mutex lock;
	int open_count;
	int suspended;
	int blank_mode;
	int open_blank_mode;
	int has_dss_clock;
};

static void wait_for_go(struct omapfbv_device *fbvdev);

static struct fb_var_screeninfo  __initdata omap_fbv_default_screeninfo = {
	.xres		= 320,
	.yres 		= 480,
	.xres_virtual 	= 320,
	.yres_virtual	= (480*3),
	.bits_per_pixel = 32,
	.grayscale 	= 0,
	.red 		= {16, 8, 0},
	.green 		= {8, 8, 0},
	.blue 		= {0, 8, 0},
	.transp 	= {24, 8, 0},
	.nonstd 	= 0,
	.activate 	= FB_ACTIVATE_NOW,
	.height 	= -1,
	.width 		= -1,
};

static struct fb_fix_screeninfo __initdata omap_fbv_default_fix = {
	.id 		= "Omapv-fbfixd",
	.type 		= FB_TYPE_PACKED_PIXELS,
	.visual 	= FB_VISUAL_TRUECOLOR,
	.xpanstep 	= 0,
	.ypanstep 	= 1,
	.line_length 	= (320*4),
	.accel 		= FB_ACCEL_NONE
};


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


static void wait_for_go(struct omapfbv_device *fbvdev)
{
	unsigned long timeout;

	if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
		timeout = HZ / 5;
		timeout += jiffies;
		while(omap2_disp_reg_sync_bit(OMAP2_OUTPUT_LCD )
			&& time_before(jiffies, timeout))
		{
			if ((!in_interrupt()) && (!irqs_disabled())) {
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(1);
			} else
				udelay(10);
		}
	}
}



static void setup_mode_parameters(struct omapfbv_device *fbvdev)
{
	struct v4l2_pix_format *pix;
	struct v4l2_rect *crop;
	struct v4l2_window *win;
	struct fb_info *info;

	info = fbvdev->fb_info;

	pix = &fbvdev->pix;
	crop = &fbvdev->crop;
	win = &fbvdev->win;

	fbvdev->rotation = -1;
	fbvdev->mirror = 0;

	pix->width = info->var.xres;
	pix->height = info->var.yres;

	pix->pixelformat = fbvdev->pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = (pix->width * info->var.bits_per_pixel ) >> 3;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = fbvdev->colorspace;

	crop->left = 0;
	crop->top = 0;
	crop->width  = info->var.xres;
	crop->height = info->var.yres;

	win->w.width = info->var.xres;
	win->w.height = info->var.yres;
	win->w.left = 0;
	win->w.top = 0;
}



static int omap_fbv_open(struct fb_info *info, int init)
{
	struct omapfbv_device *fbvdev = dev_get_drvdata(info->device);

	if (!fbvdev) {
		return 0;
	}

	mutex_lock(&fbvdev->lock);
	fbvdev->open_count++;

	if (fbvdev->open_count == 1) {
		fbvdev->blank_mode = fbvdev->open_blank_mode;

		setup_mode_parameters(fbvdev);

		memset ((void *) fbvdev->framebuffer_base, 0, 
		                 fbvdev->framebuffer_size);
		fbvdev->framebuffer_pan_base_phys = fbvdev->framebuffer_base_phys;

		omap2_disp_get_dss();
		omap2_disp_request_layer(fbvdev->video_layer);
		omap2_disp_config_vlayer (fbvdev->video_layer, 
			                 &fbvdev->pix, 
			                 &fbvdev->crop, 
			                 &fbvdev->win,
			                  fbvdev->rotation, 
			                  fbvdev->mirror);

		omap2_disp_start_vlayer (fbvdev->video_layer, 
			                &fbvdev->pix, 
			                &fbvdev->crop, 
			                &fbvdev->win,
			                 fbvdev->framebuffer_pan_base_phys, 
			                 fbvdev->rotation, 
			                 fbvdev->mirror);

		if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
			omap2_disp_enable_layer(fbvdev->video_layer);
			fbvdev->has_dss_clock = 1;
			omap2_disp_get_dss();
		} else {
			omap2_disp_disable_layer(fbvdev->video_layer);
			omap2_disp_release_layer(fbvdev->video_layer);
		}
		wait_for_go(fbvdev);
		omap2_disp_save_initstate(fbvdev->video_layer);
		omap2_disp_put_dss(); 
	}

	mutex_unlock(&fbvdev->lock);
	
	return 0;
}


static int omap_fbv_release(struct fb_info *info, int init)
{
	struct omapfbv_device *fbvdev = dev_get_drvdata(info->device);

	mutex_lock(&fbvdev->lock);
	
	if (fbvdev->open_count > 0) 
		fbvdev->open_count--;

	if (fbvdev->open_count == 0) {
		if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
			omap2_disp_get_dss();
			fbvdev->blank_mode = FB_BLANK_NORMAL;
			omap2_disp_disable_layer(fbvdev->video_layer);
			omap2_disp_release_layer(fbvdev->video_layer);
			omap2_disp_put_dss();
		}
		if (fbvdev->has_dss_clock) {
			fbvdev->has_dss_clock = 0;
			omap2_disp_put_dss();
		}
	}

	mutex_unlock(&fbvdev->lock);


	return 0;
}


static int omap_fbv_setcolreg(u_int regno,  u_int red, u_int green, u_int blue,
	                      u_int transp, struct fb_info *info)
{
	return 0;
}

static int
omap_fbv_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)

{
	int ret;
	switch(cmd)
	{
		case FBIO_WAITFORVSYNC:
			/* This ioctl accepts an integer argument to specify a
			 * display.  We only support one display, so we will
			 * simply ignore the argument.
		 	 */
			omap2_disp_get_dss();
			ret = omap24xxfb_display_wait_for_vsync();
			omap2_disp_put_dss();
			return ret;
	}
	return -EINVAL;
}

static int omap_fbv_pan_display(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	struct omapfbv_device *fbvdev = dev_get_drvdata(info->device);
	u32 view_address_base;
	int reg_bank=0;

	if (!fbvdev) {
		return 0;
	}
	
	if (fbvdev->video_layer == OMAP2_VIDEO1)
		reg_bank = 0;
	else if (fbvdev->video_layer == OMAP2_VIDEO2)
		reg_bank = 1;
	else
		return 0;

	if (var->xoffset > info->var.xres_virtual - info->var.xres)
		return -EINVAL;
	if (var->yoffset > info->var.yres_virtual - info->var.yres)
		return -EINVAL;

	view_address_base = info->fix.smem_start;

	fbvdev->framebuffer_pan_base_phys = view_address_base 
		+ (var->yoffset * info->fix.line_length)
		+ (var->xoffset * var->bits_per_pixel)/8;

	if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
		omap2_disp_get_dss();
		
		// in future determine if really want to wait...
		wait_for_go(fbvdev);
	
		dispc_reg_out(DISPC_VID_BA0(reg_bank), 
				fbvdev->framebuffer_pan_base_phys);
		dispc_reg_out(DISPC_VID_BA1(reg_bank), 
				fbvdev->framebuffer_pan_base_phys);

/*		dispc_reg_merge(DISPC_VID_ATTRIBUTES(reg_bank),
				DISPC_VID_ATTRIBUTES_ENABLE,
				DISPC_VID_ATTRIBUTES_ENABLE);
*/
		dispc_reg_merge(DISPC_CONTROL, DISPC_CONTROL_GOLCD,
				DISPC_CONTROL_GOLCD);
		omap2_disp_put_dss();
	}


	return 0;
}


static int omap_fbv_blank(int blank_mode, struct fb_info *info)
{
	struct omapfbv_device *fbvdev = dev_get_drvdata(info->device);

	omap2_disp_get_dss();
	mutex_lock(&fbvdev->lock);
	switch (blank_mode) 
	{
	case FB_BLANK_UNBLANK:
		if (!fbvdev->has_dss_clock) {
			fbvdev->has_dss_clock = 1;
			omap2_disp_get_dss();
		}

		omap2_disp_request_layer( fbvdev->video_layer);
		omap2_disp_config_vlayer( fbvdev->video_layer, 
					 &fbvdev->pix, 
					 &fbvdev->crop, 
					 &fbvdev->win,
					  fbvdev->rotation, 
					  fbvdev->mirror);

		omap2_disp_start_vlayer (fbvdev->video_layer, 
					&fbvdev->pix, 
					&fbvdev->crop, 
					&fbvdev->win,
					 fbvdev->framebuffer_pan_base_phys, 
					 fbvdev->rotation, 
					 fbvdev->mirror);

		omap2_disp_enable_layer(fbvdev->video_layer);
		break;
	case FB_BLANK_NORMAL:
	default:
		if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
			fbvdev->blank_mode = blank_mode;

			/* wait until Vsync */
			omap24xxfb_display_wait_for_vsync();
			omap2_disp_disable_layer(fbvdev->video_layer);
			omap2_disp_release_layer(fbvdev->video_layer);
		}

		if (fbvdev->has_dss_clock) {
			fbvdev->has_dss_clock = 0;
			omap2_disp_put_dss();
		}
		break;
	}

	fbvdev->blank_mode = blank_mode;
	mutex_unlock(&fbvdev->lock);

	wait_for_go(fbvdev);
	omap2_disp_save_initstate(fbvdev->video_layer);
	omap2_disp_put_dss();

	return 0;
}

/* 
 * the library that this driver intends to support, does not fit into the
 * current power management model of the device.
 * 
 * for risk managment,  adding a function is the best solution at this time
 */
void omapfbv_fb_pan_address(struct fb_info *info, unsigned long  pan_address)
{
	struct omapfbv_device *fbvdev = dev_get_drvdata(info->device);
	int reg_bank=0;

	if (!fbvdev) return;
	if (fbvdev->local_id != LOCAL_ID) return;
	if (fbvdev->video_layer == OMAP2_VIDEO1)
		reg_bank = 0;
	else if (fbvdev->video_layer == OMAP2_VIDEO2)
		reg_bank = 1;
	else
		return;

	fbvdev->framebuffer_pan_base_phys = pan_address;

	if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
		omap2_disp_get_dss();
		dispc_reg_out(DISPC_VID_BA0(reg_bank), 
				fbvdev->framebuffer_pan_base_phys);
		dispc_reg_out(DISPC_VID_BA1(reg_bank), 
				fbvdev->framebuffer_pan_base_phys);
		/* the driver driver calling this function will hit the go bit */
		omap2_disp_put_dss();
	}
}


static struct fb_ops omap_fbv_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= omap_fbv_open,
	.fb_release	= omap_fbv_release,
	.fb_setcolreg	= omap_fbv_setcolreg,
	.fb_pan_display	= omap_fbv_pan_display,
	.fb_blank	= omap_fbv_blank,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_ioctl	= omap_fbv_ioctl,
};


static int __init omap_fbv_probe(struct platform_device *pdev)
{
	struct omapfbv_device *fbvdev;
	struct fb_var_screeninfo *sinfo;
	struct fb_info *info;
	u32  fbsize;

	// tbd check for initalized lcd controller

	fbvdev = kzalloc(sizeof(struct omapfbv_device), GFP_KERNEL);
	if (fbvdev == NULL) {
		dev_err(&pdev->dev,"unable to allocate device info\n");
		return -ENOMEM;
	}

	fbvdev->dev = &(pdev->dev);
	platform_set_drvdata(pdev, fbvdev);

	fbvdev->fb_info = (struct fb_info *) framebuffer_alloc(0, fbvdev->dev);
	if (!fbvdev->fb_info) {
		dev_err(&pdev->dev, "unable to allocate fb_info info\n");
		goto fb_alloc_err;
	}

	info = fbvdev->fb_info;
	fbvdev->local_id = LOCAL_ID;
	fbvdev->video_layer = OMAP2_VIDEO2;
	fbvdev->pixelformat = V4L2_PIX_FMT_RGB32;
	fbvdev->colorspace  = V4L2_COLORSPACE_JPEG;

	fbvdev->fb_info->var = omap_fbv_default_screeninfo;
	fbvdev->fb_info->fix = omap_fbv_default_fix;

	fbvdev->open_count = 0;

	mutex_init(&fbvdev->lock);

	fbvdev->suspended  = 0;
	fbvdev->blank_mode = FB_BLANK_NORMAL;

	fbvdev->open_blank_mode = FB_BLANK_NORMAL;  

	if (pdev->dev.platform_data != NULL) {
		// has platform data, so setup the requested parameters
		struct omapfbv_platform_data *plat_data;
		
		plat_data = (struct omapfbv_platform_data *) pdev->dev.platform_data;

		fbvdev->video_layer = plat_data->video_layer;
		fbvdev->pixelformat = plat_data->pixelformat;
		fbvdev->colorspace  = plat_data->colorspace;

		fbvdev->fb_info->var = plat_data->screeninfo;
		fbvdev->fb_info->fix = plat_data->fix;


		fbvdev->xres = fbvdev->fb_info->var.xres;
		fbvdev->yres = fbvdev->fb_info->var.yres;
		fbvdev->xres_virtual = fbvdev->fb_info->var.xres_virtual;
		fbvdev->yres_virtual = fbvdev->fb_info->var.yres_virtual;

	}

	sinfo = &info->var;

	snprintf(fbvdev->fb_info->fix.id,15,"%s%1d", 
	         OMAP_FBV_DEVICE_FIX_ID, fbvdev->video_layer );

	fbvdev->framebuffer_size = (sinfo->xres * sinfo->yres * sinfo->bits_per_pixel + 7 ) / 8;
	fbsize = (sinfo->xres_virtual * sinfo->yres_virtual * sinfo->bits_per_pixel   + 7 ) / 8;

	if ( fbvdev->framebuffer_size < fbsize) {
		fbvdev->framebuffer_size = fbsize;
	}

	fbvdev->framebuffer_base = (unsigned long)
			dma_alloc_coherent(NULL, fbvdev->framebuffer_size,
			          (dma_addr_t *) &fbvdev->framebuffer_base_phys,
			                   GFP_KERNEL | GFP_DMA);

	if (!fbvdev->framebuffer_base) {
		dev_err(&pdev->dev, "unable to allocate frame buffer\n");
		goto dma_alloc_err;
	}

	info->flags = FBINFO_DEFAULT | FBINFO_HWACCEL_YPAN;
	info->fix.line_length = (sinfo->xres_virtual * sinfo->bits_per_pixel + 7) / 8;
	
	info->fbops = &omap_fbv_ops;
	info->screen_base = (char __iomem *)   fbvdev->framebuffer_base;

	info->fix.smem_start = (unsigned long) fbvdev->framebuffer_base_phys;
	info->fix.smem_len = fbvdev->framebuffer_size;

	info->fix.mmio_start = DSS_REG_BASE;
	info->fix.mmio_len = DSS_REG_SIZE;
	info->fix.accel = FB_ACCEL_NONE;

	if (register_framebuffer(info) < 0) {
		dev_err(&pdev->dev, "unable to register framebuffer\n");
		goto reg_frame_buf_err;
	}

	return 0;

reg_frame_buf_err:
	 dma_free_coherent(NULL, fbvdev->framebuffer_size,
	                (void *) fbvdev->framebuffer_base,
	                         fbvdev->framebuffer_base_phys );
dma_alloc_err:
	framebuffer_release(fbvdev->fb_info);
fb_alloc_err:
	kfree(fbvdev);
	return -ENODEV;
}


static void omap_fbv_dev_release(struct device  *dev)
{
	struct omapfbv_device *fbvdev = dev_get_drvdata(dev);
	struct fb_info *info = NULL;

	if (!fbvdev)
		return;

	info = fbvdev->fb_info;
	if (info) {
		unregister_framebuffer(info);
		dma_free_coherent(NULL, fbvdev->framebuffer_size,
			       (void *) fbvdev->framebuffer_base,
			                fbvdev->framebuffer_base_phys );
		framebuffer_release(info);
	}
	kfree (fbvdev);
	dev_set_drvdata(dev, NULL);
}

static int omap_fbv_remove(struct platform_device *pdev)
{
	omap_fbv_dev_release(&pdev->dev);
	return 0;
}


static int omap_fbv_suspend (struct platform_device *dev, pm_message_t state)
{
	struct omapfbv_device *fbvdev = platform_get_drvdata(dev);

	if(fbvdev->suspended == 1) 
		return 0;
		
	if (fbvdev->open_count > 0 ) {
		omap2_disp_get_dss ();
		omap2_disp_save_initstate(fbvdev->video_layer);
		if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
			omap2_disp_disable_layer(fbvdev->video_layer);
		}
		if (fbvdev->has_dss_clock) {
			omap2_disp_put_dss ();
		}
		omap2_disp_put_dss ();
	}
	
	fbvdev->suspended = 1;

	return 0;
}

static int omap_fbv_resume (struct platform_device *dev)
{
	struct omapfbv_device *fbvdev = platform_get_drvdata(dev);

	if(fbvdev->suspended == 0) 
		return 0;
		
	if (fbvdev->open_count > 0 && fbvdev->has_dss_clock) {
		omap2_disp_get_dss ();
		omap2_disp_restore_initstate(fbvdev->video_layer);
		if (fbvdev->blank_mode == FB_BLANK_UNBLANK) {
			omap2_disp_enable_layer(fbvdev->video_layer);
		}
	}
	fbvdev->suspended = 0;

	return 0;
}



static struct platform_driver omap_fbv_driver = {
	.driver = {
		.name	= MODULENAME,
	},
	.probe		= omap_fbv_probe,
	.remove		= omap_fbv_remove,
#ifdef CONFIG_PM
	.suspend	= omap_fbv_suspend,
	.resume		= omap_fbv_resume,
#endif

};

#ifdef MODULE
static struct platform_device omap_fbv_device = {
	.name = DEVICENAME,
	.dev = {
		.release = omap_fbv_dev_release,
	}
};
#endif


static int __init omap_fbv_init(void)
{
	int ret = 0;

	if (platform_driver_register(&omap_fbv_driver)) {
		pr_debug("failed to register omapfbv driver\n");
		return -ENODEV;
	}

#ifdef MODULE
	if ((ret = platform_device_register(&omap_fbv_device)) != 0) {
		pr_debug( "Unable to register platform device omapfbv\n");
		platform_driver_unregister(&omap_fbv_driver);
		return -ENODEV;
	}
#endif
	return ret;
}


static void __exit omap_fbv_exit(void)
{
#ifdef MODULE
	platform_device_unregister(&omap_fbv_device);
#endif
	platform_driver_unregister(&omap_fbv_driver);
}

MODULE_AUTHOR("Bennett Chan");
MODULE_DESCRIPTION("Omap frame buffer for video layer");
MODULE_LICENSE("GPL");


module_init(omap_fbv_init);
module_exit(omap_fbv_exit);

EXPORT_SYMBOL(omapfbv_fb_pan_address);

