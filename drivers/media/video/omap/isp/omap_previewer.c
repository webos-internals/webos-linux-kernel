/*
 * drivers/media/video/omap/isp/omap_previewer.c
 *
 * Wrapper for Preview module in TI's OMAP3430 ISP
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/videodev.h>
#include <media/video-buf.h>
#include <media/v4l2-dev.h>
#include <asm/cacheflush.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/io.h>
#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "omap_previewer.h"

#define OMAP_PREV_NAME		"omap-previewer"

/* global variables */
static int prev_major = -1;
static u32 prev_bufsize = 0;
static struct device *prev_dev;
static struct class *prev_class = NULL;
static struct prev_device *prevdevice;
static struct platform_driver omap_previewer_driver;

/*
 * This function is used to calculate frame size reduction depending on
 * the features enabled by the application.
 */
static void prev_calculate_crop(struct prev_params *config,
				struct prev_cropsize *crop)
{
	dev_dbg(prev_dev, "prev_calculate_crop E\n");

	if (!config || !crop) {
		dev_err(prev_dev, "\nErron in argument");
		return;
	}

	crop->hcrop = crop->vcrop = 0;
	/* Horizontal medial filter reduces image width by 4 pixels 
	   2 right and 2 left */
	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		crop->hcrop += 4;
	}
	/* Noise filter reduces image height and width 2 pixels from 
	   top, left, right and bottom */
	if (config->features & PREV_NOISE_FILTER) {
		crop->hcrop += 4;
		crop->vcrop += 4;
	}
	/* CFA Interpolation reduces image height and width 2 pixels 
	   from top, left, right and bottom */
	if (config->features & PREV_CFA) {
		crop->hcrop += 4;
		crop->vcrop += 4;
	}
	/* Luma enhancement reduces image width 1 pixels from left, right */
	if (config->features & (PREV_LUMA_ENHANCE | PREV_CHROMA_SUPPRESS)) {
		crop->hcrop += 2;
	}
	dev_dbg(prev_dev, "prev_calculate_crop L\n");
}


static int prev_get_status(struct prev_status *status)
{
	if (!status) {
		dev_err(prev_dev, "get_status: invalid parameter\n");
		return -EINVAL;
	}
	status->hw_busy = (char)isppreview_busy();
	return 0;
}


static int prev_hw_setup(struct prev_params *config)
{
	dev_dbg(prev_dev, "prev_hw_setup E\n");

	if (config->features & PREV_AVERAGER) {
		isppreview_config_averager(config->average);
	} else
		isppreview_config_averager(0);

	if (config->features & PREV_INVERSE_ALAW)
		isppreview_enable_invalaw(1);
	else
		isppreview_enable_invalaw(0);

	if (config->features & PREV_HORZ_MEDIAN_FILTER) {
		isppreview_config_hmed(config->hmf_params);
		isppreview_enable_hmed(1);
	} else
		isppreview_enable_hmed(0);

	if (config->features & PREV_DARK_FRAME_SUBTRACT) {
		isppreview_set_darkaddr(config->drkf_params.addr);
		isppreview_config_darklineoffset(config->drkf_params.offset);
		isppreview_enable_drkframe(1);
	} else
		isppreview_enable_drkframe(0);

	if (config->features & PREV_LENS_SHADING) {
		isppreview_config_drkf_shadcomp(config->lens_shading_shift);
		isppreview_enable_shadcomp(1);
	} else
		isppreview_enable_shadcomp(0);
	
	dev_dbg(prev_dev, "prev_hw_setup L\n");	
	return 0;
}

static int prev_validate_params(struct prev_params *params)
{
	struct prev_cropsize crop;

	if (!params) {
		printk("validate_params: error in argument");
		return -EINVAL;
	}

	prev_calculate_crop(params, &crop);

	/* check whether down sampling rate is one of the supported */
	if ((params->features & PREV_AVERAGER) == PREV_AVERAGER) {
		if ((params->average != NO_AVE)
		    && (params->average != AVE_2_PIX)
		    && (params->average != AVE_4_PIX)
		    && (params->average != AVE_8_PIX)) {
			/* if not return error */
			printk("validate_params: wrong pix average\n");
			return -EINVAL;
		} else if (((params->average == AVE_2_PIX)
				&& (params->size_params.hsize % 2)) ||
				((params->average == AVE_4_PIX)
			    	&& (params->size_params.hsize % 4)) ||
				((params->average == AVE_8_PIX)
			    	&& (params->size_params.hsize % 8))) {
			printk("validate_params: "
				"wrong pix average for input size\n");
			return -EINVAL;	
		}
	}

	/* check for valid values of pixel size */
	if (params->size_params.pixsize != PREV_INWIDTH_8BIT
	    && params->size_params.pixsize != PREV_INWIDTH_10BIT) {
		printk("validate_params: wrong pixsize\n");
		return -EINVAL;
	}

	/* check whether size of the image is within limit */
	if ((params->size_params.hsize) > MAX_IMAGE_WIDTH + crop.hcrop
	    || (params->size_params.hsize) < 0) {
		printk("validate_params: wrong hsize\n");
		return -EINVAL;
	}

	/* check for valid values output pixel format */
	if (params->pix_fmt != YCPOS_YCrYCb &&
	    YCPOS_YCbYCr != params->pix_fmt &&
	    YCPOS_CbYCrY != params->pix_fmt &&
	    YCPOS_CrYCbY != params->pix_fmt) {
		printk("validate_params: wrong pix_fmt");
		return -EINVAL;
	}

	/* dark frame capture and subtract should not be enabled 
	   at the same time */
	if ((params->features & PREV_DARK_FRAME_SUBTRACT) &&
	    (params->features & PREV_DARK_FRAME_CAPTURE)) {
	    	printk("validate_params: DARK FRAME CAPTURE and "
		       "SUBSTRACT cannot be enabled at same time\n");
		return -EINVAL;
	}

	/* check to see dark frame address should not be null */
	if (params->features & PREV_DARK_FRAME_SUBTRACT)
		if (!(params->drkf_params.addr)
		    || (params->drkf_params.offset % 32)) {
			printk("validate_params: dark frame address\n");
			return -EINVAL;
		}

	/* check to see lens shading shift value should not be greater 
	   than 7 */
	if (params->features & PREV_LENS_SHADING)
		if ((params->lens_shading_shift > 7)
		    || !(params->drkf_params.addr)
		    || (params->drkf_params.offset % 32)) {
		    	printk("validate_params: lens shading shift\n");
			return -EINVAL;
		}

	/* if pitch is zero assign it to the width of the image */
	if (params->size_params.in_pitch <= 0
	    || params->size_params.in_pitch % 32) {
		params->size_params.in_pitch = 
				(params->size_params.hsize * 2) & 0xFFE0;
		printk("\nError in in_pitch; new value = %d",
			params->size_params.in_pitch);
	}

	if (params->size_params.out_pitch <= 0
	    || params->size_params.out_pitch % 32) {
		params->size_params.out_pitch = 
				((params->size_params.hsize - crop.hcrop) * 2)
				& 0xFFE0;
		printk("\nError in out_pitch; new value = %d",
			params->size_params.in_pitch);
	}

	return 0;
}

/*
 * Callback from ISP driver for ISP Preview Interrupt
 * status 	: IRQ0STATUS
 * arg1		: struct prev_device
 * arg2		: Not used as of now.
 */
static void preview_isr(unsigned long status, void *arg1, void *arg2)
{
	struct prev_device *device = (struct prev_device *)arg1;

	if ((PREV_DONE & status) != PREV_DONE)
		return;

	if (device)
		complete(&(device->wfc));
}

/*
 * Performs the Preview process
 */ 
static int prev_do_preview(struct prev_device *device, int *arg)
{
	int bpp, size, cropsize;
	struct prev_cropsize crop;
	int ret = 0;
	u32 out_hsize, out_vsize, out_line_offset;

	dev_dbg(prev_dev, "prev_do_preview E\n");

	if (!device) {
		dev_err(prev_dev, "preview: invalid parameters\n");
		return -EINVAL;
	}

	prev_calculate_crop(device->params, &crop);
	/* Set bytes per pixel */
	if (device->params->size_params.pixsize == PREV_INWIDTH_8BIT)
		bpp = 1;
	else
		bpp = 2;

	size = device->params->size_params.hsize *
		device->params->size_params.vsize * bpp;
	cropsize =
		 2 * (crop.vcrop * device->params->size_params.hsize +
		 crop.hcrop * (device->params->size_params.vsize - crop.vcrop));

	/* Set input - output addresses, using just one buffer */
	ret = isppreview_set_inaddr(device->isp_addr_read);
	if (ret)
		return ret;

	ret = isppreview_set_outaddr(device->isp_addr_read);
	if (ret)
		return ret;

	isppreview_try_size(device->params->size_params.hsize,
			    device->params->size_params.vsize,
			    &out_hsize, &out_vsize);

	ret = isppreview_config_inlineoffset(device->params->size_params.hsize
						* bpp);
	if (ret)
		return ret;

	out_line_offset = out_hsize * bpp;
	while ((out_line_offset & PREV_32BYTES_ALIGN_MASK)!= out_line_offset)
		out_line_offset++;

	ret = isppreview_config_outlineoffset(out_line_offset);
	if (ret)
		return ret;

	ret = isppreview_config_size(device->params->size_params.hsize,
			       device->params->size_params.vsize,
			       out_hsize, out_vsize);
	if (ret)
		return ret;

	/* Set input source to DDRAM */
	isppreview_config_datapath(PRV_RAW_MEM, PREVIEW_MEM);

	ret = isp_set_callback(CBK_PREV_DONE, preview_isr, (void *)device,
		       (void *)NULL);
	if (ret) {
		printk("ERROR while setting Previewer callback!\n");
		return ret;
	}
	/* enable previewer which starts previewing */
	isppreview_enable(1);

	/* wait untill processing is not completed */
	wait_for_completion_interruptible(&(device->wfc));

	if (device->isp_addr_read) {
		ispmmu_unmap(device->isp_addr_read);
		device->isp_addr_read = 0;
	}

	ret = isp_unset_callback(CBK_PREV_DONE);

	dev_dbg(prev_dev, "prev_do_preview L\n");
	return ret;
}

/*
 * Videobuffer queue release
 */ 
static void previewer_vbq_release(struct videobuf_queue *q,
				  struct videobuf_buffer *vb)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;

	ispmmu_unmap(device->isp_addr_read);
	device->isp_addr_read = 0;
	spin_lock(&device->vbq_lock);
	vb->state = STATE_NEEDS_INIT;
	spin_unlock(&device->vbq_lock);
	dev_dbg(prev_dev, "previewer_vbq_release\n");
}

/*
 * Sets up the videobuffer size and validates count.
 */
static int previewer_vbq_setup(struct videobuf_queue *q,
			       unsigned int *cnt,
			       unsigned int *size)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;
	u32 bpp = 1;

	spin_lock(&device->vbq_lock);
	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;	/* supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;
				
	if (!device->params->size_params.hsize ||
		!device->params->size_params.vsize) {
		printk("Can't setup buffer size\n");
		spin_unlock(&device->vbq_lock);
		return -EINVAL;
	} else {
		if (device->params->size_params.pixsize == PREV_INWIDTH_10BIT)
			bpp = 2;
		*size = prev_bufsize = (device->params->size_params.hsize
			 * device->params->size_params.vsize
			 * bpp);
	}
	spin_unlock(&device->vbq_lock);
	dev_dbg(prev_dev, "previewer_vbq_setup\n");
	return 0;
}

/*
 * Videobuffer is prepared and mmapped.
 */ 
static int previewer_vbq_prepare(struct videobuf_queue *q,
				 struct videobuf_buffer *vb,
				 enum v4l2_field field)
{
	struct prev_fh *fh = q->priv_data;
	struct prev_device *device = fh->device;
	int err = -EINVAL;
	unsigned int isp_addr;

	dev_dbg(prev_dev, "previewer_vbq_prepare E\n");	
	spin_lock(&device->vbq_lock);
	if (vb->baddr) {
		vb->size = prev_bufsize;
		/* video-buf uses bsize (buffer size) instead of
		 * size (image size) to generate sg slist. Work
		 * around this bug by making bsize the same as
		 * size.
		 */
		vb->bsize = prev_bufsize;
	} else {
		spin_unlock(&device->vbq_lock);	
		printk("No user buffer allocated\n");
		return err;
	}

	vb->width = device->params->size_params.hsize;
	vb->height = device->params->size_params.vsize;
	vb->field = field;
	spin_unlock(&device->vbq_lock);

	if (vb->state == STATE_NEEDS_INIT) {
		/* FIXME: timing error? around videobuf_iolock */
		printk(" Videobuf mapping to begin \n");
		err = videobuf_iolock(q, vb, NULL);
		if (!err) {
			isp_addr = ispmmu_map_sg(vb->dma.sglist, vb->dma.sglen);
			if (!isp_addr)
				err = -EIO;
			else
		    		device->isp_addr_read = isp_addr;
		}
	}

	if (!err) {
		vb->state = STATE_PREPARED;
		flush_cache_user_range(NULL, vb->baddr,
					(vb->baddr + vb->bsize));
	} else
		previewer_vbq_release(q, vb);

	dev_dbg(prev_dev, "previewer_vbq_prepare L\n");	
	return err;
}

static int previewer_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct prev_device *device = prevdevice;
	struct prev_params *config = isppreview_get_config();
	struct prev_fh *fh;

	if (config == NULL) {
		printk("Unable to initialize default config "
			"from isppreviewer\n\n");
		return -EACCES;
	}

	if (device->opened || filp->f_flags & O_NONBLOCK) {
		dev_err
		    (prev_dev, "previewer_open: device is already openend\n");
		return -EBUSY;
	}

	/* allocate per-filehandle data */
	fh = kmalloc(sizeof (struct prev_fh), GFP_KERNEL);
	if (NULL == fh)
		return -ENOMEM;

	isp_get();
	ret = isppreview_request();
	if (ret) {
		isp_put();
		printk("Can't acquire isppreview\n");
		return ret;
	}

	/* initialize mutex to 0 */
	device->params = config;
	device->opened = 1;

	filp->private_data = fh;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fh->device = device;

	videobuf_queue_init(&fh->vbq, &device->vbq_ops, NULL, &device->vbq_lock,
			    fh->type, V4L2_FIELD_NONE,
			    sizeof (struct videobuf_buffer), fh);

	init_completion(&device->wfc);
	device->wfc.done = 0;
	init_MUTEX(&device->sem);
	
	return 0;
}


static int previewer_release(struct inode *inode, struct file *filp)
{
	struct prev_fh *fh = filp->private_data;
	struct prev_device *device = fh->device;
	struct videobuf_queue *q = &fh->vbq;

	/* change the device status to available */
	device->opened = 0;
	device->params = NULL;
	isppreview_free();
	isp_put();
	/* free video_buffer objects */
	videobuf_mmap_free(q);
	prev_bufsize = 0;
	filp->private_data = NULL;
	kfree(fh);

	dev_dbg(prev_dev, "previewer_release\n");	
	return 0;
}

static int previewer_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct prev_fh *fh = file->private_data;
	dev_dbg(prev_dev, "previewer_mmap\n");

	return videobuf_mmap_mapper(&fh->vbq, vma);
}

static int previewer_ioctl(struct inode *inode, struct file *file,
			   unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct prev_params params;
	/* get the address of global object of prev_device structure */
	struct prev_fh *fh = file->private_data;
	struct prev_device *device = fh->device;

	dev_dbg(prev_dev,"Entering previewer_ioctl()\n");

	/* Before decoding check for correctness of cmd */
	if (_IOC_TYPE(cmd) != PREV_IOC_BASE) {
		dev_err(prev_dev, "Bad command Value \n");
		return -1;
	}
	if (_IOC_NR(cmd) > PREV_IOC_MAXNR) {
		dev_err(prev_dev, "Bad command Value\n");
		return -1;
	}

	/* Verify accesses       */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		dev_err(prev_dev, "access denied\n");
		return -1;	/*error in access */
	}

	/* switch according value of cmd */
	switch (cmd) {
	case PREV_REQBUF:
		down_interruptible(&(device->sem));
		ret = videobuf_reqbufs(&fh->vbq, (void *)arg);
		up(&(device->sem));
		break;

	case PREV_QUERYBUF:
		down_interruptible(&(device->sem));
		ret = videobuf_querybuf(&fh->vbq, (void *)arg);
		up(&(device->sem));
		break;

	case PREV_QUEUEBUF:
		down_interruptible(&(device->sem));
		ret = videobuf_qbuf(&fh->vbq, (void *)arg);
		up(&(device->sem));
		break;

		/* if case is to set configuration parameters */
	case PREV_SET_PARAM:
		down_interruptible(&(device->sem));
		/* copy the parameters to the configuration */
		if (copy_from_user
		    (&params, (struct prev_params *)arg,
		     sizeof(struct prev_params))) {
			up(&(device->sem));
			return -EFAULT;
		}
		/* check for errors */
		ret = prev_validate_params(&params);
		if (ret < 0) {
			printk("Error validating parameters!\n");
			up(&(device->sem));
			return ret;
		}
		/* copy the values to device params */
		if (device->params)
			memcpy(device->params, &params,
			       sizeof(struct prev_params));
		else {
			up(&(device->sem));
			return -EINVAL;
		}

		ret = prev_hw_setup(device->params);
		up(&(device->sem));
		break;

	case PREV_GET_PARAM:
		if (copy_to_user
		    ((struct prev_params *)arg, (device->params),
		     sizeof(struct prev_params)))
			ret = -EFAULT;
		break;

	case PREV_GET_STATUS:
		ret = prev_get_status((struct prev_status *)arg);
		break;

	case PREV_PREVIEW:
		down_interruptible(&(device->sem));
		ret = prev_do_preview(device, (int *)arg);
		up(&(device->sem));
		break;

	case PREV_GET_CROPSIZE:
		prev_calculate_crop(device->params,
				    (struct prev_cropsize *)arg);
		break;

	default:
		dev_err(prev_dev, "previewer_ioctl: Invalid Command Value\n");
		ret = -EINVAL;
	}
	return ret;
}

static void previewer_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
	dev_err(prev_dev, "previewer_platform_release()\n");
}

static struct file_operations prev_fops = {
	.owner = THIS_MODULE,
	.open = previewer_open,
	.release = previewer_release,
	.mmap = previewer_mmap,
	.ioctl = previewer_ioctl,
};

static struct platform_device omap_previewer_device = {
	.name = OMAP_PREV_NAME,
	.id = -1,
	.dev = {
		/* we may add later */
		.release = previewer_platform_release,
	}
};

static int __init previewer_probe(struct platform_device *pdev)
{
	return 0;
}

static int previewer_remove(struct platform_device *pdev)
{
	dev_err(prev_dev, "previewer_remove()\n");

	platform_device_unregister(&omap_previewer_device); 
	platform_driver_unregister(&omap_previewer_driver);
	unregister_chrdev(prev_major, OMAP_PREV_NAME);
	return 0;
}

static struct platform_driver omap_previewer_driver = {
	.probe = previewer_probe,
	.remove = previewer_remove,
//	.shutdown = previewer_shutdown,
//	.suspend = previewer_suspend,
//	.resume = previewer_resume,
	.driver = {
		.owner	= THIS_MODULE,
		.name = OMAP_PREV_NAME,
	},
};


static int __init omap_previewer_init(void)
{
	int ret;
	struct prev_device *device;

	device = kmalloc(sizeof (struct prev_device), GFP_KERNEL);
	if (!device) {
		printk(KERN_ERR OMAP_PREV_NAME ": could not allocate memory\n");
		return -ENOMEM;
	}
	/* Register the driver in the kernel */
	prev_major = register_chrdev(0, OMAP_PREV_NAME, &prev_fops);

	if (prev_major < 0) {
		printk(OMAP_PREV_NAME ": initialization "
                	"failed. could not register character device\n");
		return -ENODEV;
	}

	ret = platform_driver_register(&omap_previewer_driver);
	if (ret) {
		printk(OMAP_PREV_NAME
			": failed to register platform driver!\n");
		goto fail2;
	}
	/* Register the drive as a platform device */
	ret = platform_device_register(&omap_previewer_device); 
	if (ret) {
		printk(OMAP_PREV_NAME
			": failed to register platform device!\n");
		goto fail3;
	}

	prev_class = class_create(THIS_MODULE, OMAP_PREV_NAME);
	if (!prev_class)
		goto fail4;

	prev_dev = device_create(prev_class, prev_dev, (MKDEV(prev_major, 0)),
			       OMAP_PREV_NAME);
	printk(OMAP_PREV_NAME ": Registered Previewer Wrapper\n");
	device->opened = 0;

	/* initialize the videobuf queue ops */
	device->vbq_ops.buf_setup = previewer_vbq_setup;
	device->vbq_ops.buf_prepare = previewer_vbq_prepare;
	device->vbq_ops.buf_release = previewer_vbq_release;
	spin_lock_init(&device->vbq_lock);

	prevdevice = device;
	return 0;

fail4:
	platform_device_unregister(&omap_previewer_device);
fail3:
	platform_driver_unregister(&omap_previewer_driver);
fail2:
	unregister_chrdev(prev_major, OMAP_PREV_NAME);

	return ret;
}

static void __exit omap_previewer_exit(void)
{
	previewer_remove(&omap_previewer_device);
	kfree(prevdevice);
	prev_major = -1;
}

module_init(omap_previewer_init);
module_exit(omap_previewer_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP ISP Previewer");
MODULE_LICENSE("GPL");
