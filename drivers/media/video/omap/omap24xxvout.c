/*
 * drivers/media/video/omap24xx/omap24xxvout.c
 *
 * Copyright (C) 2005-2006 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 * Leveraged code from the OMAP2 camera driver
 * Video-for-Linux (Version 2) camera capture driver for 
 * the OMAP24xx camera controller.
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * History:
 * 20-APR-2006	Khasim		Modified VRFB based Rotation,
 *				The image data is always read from 0 degree view and written
 *				to the virtual space of desired rotation angle
 * 4-DEC-2006 Jian		Changed to support better memory management 
 *
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/videodev.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/dma-mapping.h>
#include <media/videobuf-core.h>
#include <media/v4l2-dev.h>
#include <media/videobuf-dma-sg.h>
 
#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
#endif
#ifdef CONFIG_DPM
#include <linux/dpm.h>
#endif

#include <asm/arch/display.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/irq.h>
#include <asm/semaphore.h>
#include <asm/processor.h>
// #include <asm/arch/bus.h>
#include <asm/arch/dma.h>

#include "omap24xxlib.h"

/*
 * Un-comment this to use Debug Write call
 */
/* #define DEBUG_ALLOW_WRITE */

#include "omap24xxvoutdef.h"

extern const char *global_mode_option;
unsigned long timeout;

/* 
 * Uncomment this if debugging support needs to be enabled
 */

/* #define DEBUG */

#undef DEBUG
#ifdef DEBUG
#define DPRINTK(ARGS...)  printk("<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DPRINTK( x... )
#endif

/* 
 * -1 means rotation support is disabled
 * 0/90/180/270 are initial rotation angles 
 */

static int rotation_support = -1;

/* configuration macros */
#define VOUT_NAME		"omap24xxvout"
#define V1OUT_NAME		"omap24xxvout1"
#define V2OUT_NAME		"omap24xxvout2"

#define D1_PAL_WIDTH		720
#define D1_PAL_HEIGHT		576
#define D1_NTSC_WIDTH		720
#define D1_NTSC_HEIGHT		486

#define D1_PAL_VSTAT_WIDTH		832
#define D1_PAL_VSTAT_HEIGHT		672

#define D1_NTSC_VSTAT_WIDTH		832
#define D1_NTSC_VSTAT_HEIGHT	560

#define WVGA_WIDTH		854
#define WVGA_HEIGHT		480

#define VGA_WIDTH		640
#define VGA_HEIGHT		480
#define QQVGA_WIDTH		160
#define QQVGA_HEIGHT		120
#define BYTESPERPIXEL		2
#define DMA_CHAN_ALLOTED	1
#define DMA_CHAN_NOT_ALLOTED	0
#define NUM_OF_VIDEO_CHANNELS	2
#define VRF_SIZE		MAX_PIXELS_PER_LINE * MAX_LINES * 4
#define SMS_RGB_PIXSIZE		2
#define SMS_YUYV_PIXSIZE	4
#define VRFB_TX_TIMEOUT		1000

#define VID_MAX_WIDTH		WVGA_WIDTH /* Largest width */
#define VID_MAX_HEIGHT		D1_PAL_VSTAT_HEIGHT /* Largest height */


static struct omap24xxvout_device *saved_v1out, *saved_v2out;

#define STREAMING_IS_ON()	((saved_v1out && saved_v1out->streaming) || \
				(saved_v2out && saved_v2out->streaming))

/* 
 * this is the layer being linked to (slave layer). possible values are:
 * OMAP2_VIDEO1:  V1 is linked to V2. V1 uses V2's pix and crop.
 * OMAP2_VIDEO2:  V2 is linked to V1. V2 uses V1's pix and crop.
 * -1: no link.
 */

static int vout_linked;
static spinlock_t vout_link_lock;

static struct videobuf_queue_ops dummy_vbq_ops;

/* module parameters */

/* 
 * Maximum amount of memory to use for rendering buffers.
 * Default is enough to four (RGB24) VGA buffers. 
 */
#define MAX_ALLOWED_VIDBUFFERS            4
static int render_mem = VID_MAX_WIDTH * VID_MAX_HEIGHT * 4 * MAX_ALLOWED_VIDBUFFERS;

static struct tvlcd_status_t tvlcd_status;

/* list of image formats supported by OMAP2 video pipelines */
const static struct v4l2_fmtdesc omap2_formats[] = {
{
	/* Note:  V4L2 defines RGB565 as:
	 *
	 *      Byte 0                    Byte 1
	 *      g2 g1 g0 r4 r3 r2 r1 r0   b4 b3 b2 b1 b0 g5 g4 g3
	 *
	 * We interpret RGB565 as:
	 *
	 *      Byte 0                    Byte 1
	 *      g2 g1 g0 b4 b3 b2 b1 b0   r4 r3 r2 r1 r0 g5 g4 g3
	 */
	.description = "RGB565, le",
	.pixelformat = V4L2_PIX_FMT_RGB565,
},
{
	/* Note:  V4L2 defines RGB565X as:
	 *
	 *      Byte 0                    Byte 1
	 *      b4 b3 b2 b1 b0 g5 g4 g3   g2 g1 g0 r4 r3 r2 r1 r0
	 *
	 * We interpret RGB565X as:
	 *
	 *      Byte 0                    Byte 1
	 *      r4 r3 r2 r1 r0 g5 g4 g3   g2 g1 g0 b4 b3 b2 b1 b0
	 */
	.description = "RGB565, be",
	.pixelformat = V4L2_PIX_FMT_RGB565X,
},
{
	/* Note:  V4L2 defines RGB32 as: RGB-8-8-8-8  we use 
	 *        this for RGB24 unpack mode, the last 8 bits are ignored
	 *
	 */
	.description = "RGB32, le",
	.pixelformat = V4L2_PIX_FMT_RGB32,
},
{
	/* Note:  V4L2 defines RGB24 as: RGB-8-8-8  we use 
	 *        this for RGB24 packed mode
	 *
	 */
	.description = "RGB24, le",
	.pixelformat = V4L2_PIX_FMT_RGB24,
},
{
	.description = "YUYV (YUV 4:2:2), packed",
	.pixelformat = V4L2_PIX_FMT_YUYV,
},
{
	.description = "UYVY, packed",
	.pixelformat = V4L2_PIX_FMT_UYVY,
},
};

#define NUM_OUTPUT_FORMATS (sizeof(omap2_formats)/sizeof(omap2_formats[0]))

/* CONFIG_PM */
#ifdef CONFIG_PM
#define omap24xxvout_suspend_lockout(s,f) \
	if ((s)->suspended) {\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		wait_event_interruptible((s)->suspend_wq,\
					(s)->suspended == 0);\
	}
#else
#define omap24xxvout_suspend_lockout(s, f) do {(s)->suspended = 0;} while(0)
#endif

/* -------------------------------------------------------------------------- */

static int
try_format (struct v4l2_pix_format *pix)
{
	int ifmt,bpp =0;
	if (pix->width > VID_MAX_WIDTH)
		pix->width = VID_MAX_WIDTH;
	if (pix->height > VID_MAX_HEIGHT)
		pix->height = VID_MAX_HEIGHT;
	for (ifmt = 0; ifmt < NUM_OUTPUT_FORMATS; ifmt++){
		if (pix->pixelformat == omap2_formats[ifmt].pixelformat)
			break;
	}

	if (ifmt == NUM_OUTPUT_FORMATS)
		ifmt = 0;

	pix->pixelformat = omap2_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->priv = 0;

	switch (pix->pixelformat){
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		bpp = YUYV_BPP;
	break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB565_BPP;
	break;
	case V4L2_PIX_FMT_RGB24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB24_BPP;
	break;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		bpp = RGB32_BPP;
	break;
	}
	pix->bytesperline = pix->width * bpp;
	pix->sizeimage = pix->bytesperline * pix->height;
	return(bpp);
}

static void
vrfb_dma_tx_callback (int lch, u16 ch_status, void *data)
{
	struct vid_vrfb_dma *t = (struct vid_vrfb_dma *) data;
	t->tx_status = 1;
	wake_up_interruptible (&t->wait);
}

static void
omap24xxvout_sync (struct omap24xxvout_device *dest,
		   struct omap24xxvout_device *src)
{
	/* 
	 * once linked, dest shares src's framebuffer, pix and crop 
	 */

	dest->pix = src->pix;
	dest->crop = src->crop;

	if (src->streaming)
		omap2_disp_config_vlayer (dest->vid, &dest->pix, &dest->crop, &dest->win,
			dest->rotation, dest->mirror);
}

static int
omap24xxvout_do_ioctl (struct inode *inode, struct file *file,
		       unsigned int cmd, void *arg)
{
	struct omap24xxvout_fh *fh = (struct omap24xxvout_fh *)file->private_data;
	struct omap24xxvout_device *vout = fh->vout;
	int err;
	unsigned int vrfb_buf_size = 0;

	switch (cmd){
	case VIDIOC_ENUMOUTPUT:
	{
		struct v4l2_output *output = (struct v4l2_output *) arg;
		int index = output->index;

		if (index > 0)
			return -EINVAL;

		memset (output, 0, sizeof (*output));
		output->index = index;

		strncpy (output->name, "video out", sizeof (output->name));
		output->type = V4L2_OUTPUT_TYPE_MODULATOR;
		return 0;
	}

	case VIDIOC_G_OUTPUT:
	{
		unsigned int *output = arg;
		*output = 0;
		return 0;
	}

	case VIDIOC_S_OUTPUT:
	{
		unsigned int *output = arg;

		if (*output > 0)
			return -EINVAL;
		return 0;
	}

	case VIDIOC_QUERYCAP:
	{
		struct v4l2_capability *cap = (struct v4l2_capability *) arg;
		memset (cap, 0, sizeof (*cap));
		strncpy (cap->driver, VOUT_NAME, sizeof (cap->driver));
		strncpy (cap->card, vout->vfd->name, sizeof (cap->card));
		cap->bus_info[0] = '\0';
		cap->capabilities = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT;
		return 0;
	}
	case VIDIOC_ENUM_FMT:
	{
		struct v4l2_fmtdesc *fmt = arg;
		int index = fmt->index;
		enum v4l2_buf_type type = fmt->type;
		memset (fmt, 0, sizeof (*fmt));
		fmt->index = index;
		fmt->type = type;

		switch (fmt->type){
			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	  		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
			if (index >= NUM_OUTPUT_FORMATS)
				return -EINVAL;
			break;
			default:
			return -EINVAL;
		}

		fmt->flags = omap2_formats[index].flags;
		strncpy (fmt->description, omap2_formats[index].description,
			sizeof (fmt->description));
		fmt->pixelformat = omap2_formats[index].pixelformat;

		return 0;
	}
	case VIDIOC_G_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;

		switch (f->type){
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			struct v4l2_pix_format *pix = &f->fmt.pix;
			memset (pix, 0, sizeof (*pix));
			*pix = vout->pix;
			return 0;
		}

		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			struct v4l2_window *win = &f->fmt.win;
			memset (win, 0, sizeof (*win));

			/* 
			 * The API has a bit of a problem here. 
			 * We're returning a v4l2_window 
			 * structure, but that structure 
			 * contains pointers to variable-sized 
			 * objects for clipping rectangles and 
			 * clipping bitmaps.  We will just 
			 * return NULLs for those pointers.
			 */

			win->w = vout->win.w;
			win->field = vout->win.field;
			win->chromakey = vout->win.chromakey;
			return 0;
		}
		default:
		return -EINVAL;
		}
	}
	case VIDIOC_TRY_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;

		if (vout->streaming)
			return -EBUSY;
			
		/* We dont support RGB24-packed mode if vrfb rotation is enabled*/
		if(vout->rotation != -1 && f->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
			return -EINVAL;
			
		switch (f->type){

		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			struct v4l2_window *win = &f->fmt.win;
			err = omap24xxvout_try_window (&vout->fbuf, win);
			return err;
		}

		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			/* don't allow to change img for the linked layer */
			if (vout->vid == vout_linked)
				return -EINVAL;
			try_format (&f->fmt.pix);
			return 0;
		}
		default:
			return -EINVAL;
		}
	}
	case VIDIOC_S_FMT:
	{
		struct v4l2_format *f = (struct v4l2_format *) arg;
		
		if (vout->streaming)
			return -EBUSY;
		
		/* We dont support RGB24-packed mode if vrfb rotation is enabled*/
		if(vout->rotation != -1 && f->fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24 )
			return -EINVAL;

		omap2_disp_get_dss();

		/* get the framebuffer parameters */
		if(vout->rotation == 90 || vout->rotation == 270){
			omap2_disp_get_panel_size (omap2_disp_get_output_dev (vout->vid),
				&(vout->fbuf.fmt.height),&(vout->fbuf.fmt.width));
		}
		else{
			omap2_disp_get_panel_size (omap2_disp_get_output_dev (vout->vid),
				&(vout->fbuf.fmt.width), &(vout->fbuf.fmt.height));
			
		}

		omap2_disp_put_dss();

		switch (f->type){
		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		{
			struct v4l2_window *win = &f->fmt.win;


			err = omap24xxvout_new_window (&vout->crop, &vout->win, 
				&vout->fbuf, win);
			return err;
		}
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			int bpp;
			/* 
			 * don't allow to change img for the linked layer 
			 */
			if (vout->vid == vout_linked)
				return -EINVAL;

			/* change to samller size is OK */
			bpp = try_format (&f->fmt.pix);
			f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height *bpp;  

			/* try & set the new output format */
			vout->bpp = bpp;
			vout->pix = f->fmt.pix;
			vout->vrfb_bpp = 1;
			/* If YUYV then vrfb bpp is 2, for others its 1*/
			if (V4L2_PIX_FMT_YUYV == vout->pix.pixelformat
				|| V4L2_PIX_FMT_UYVY == vout->pix.pixelformat)
				vout->vrfb_bpp = 2;

			/* set default crop and win */
			omap24xxvout_new_format (&vout->pix, &vout->fbuf, &vout->crop,
				&vout->win);

			return 0;
		}

		default:
			return -EINVAL;
		}
	}
	case VIDIOC_CROPCAP:
	{
		struct v4l2_cropcap *cropcap = (struct v4l2_cropcap *) arg;
		enum v4l2_buf_type type = cropcap->type;

		memset (cropcap, 0, sizeof (*cropcap));
		cropcap->type = type;
		switch (type){
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			struct v4l2_pix_format *pix = &vout->pix;
			cropcap->bounds.width = pix->width & ~1;
			cropcap->bounds.height = pix->height & ~1;
			omap24xxvout_default_crop (&vout->pix, &vout->fbuf,
					 &cropcap->defrect);
			cropcap->pixelaspect.numerator = 1;
			cropcap->pixelaspect.denominator = 1;
			return 0;
		}
		default:
			return -EINVAL;
		}
	}
	case VIDIOC_G_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;

		switch (crop->type) {
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			crop->c = vout->crop;
			return 0;
		}
		default:
			return -EINVAL;
		}
	}
	case VIDIOC_S_CROP:
	{
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;
		if (vout->streaming)
			return -EBUSY;
		
		omap2_disp_get_dss();

		/* get the framebuffer parameters */
		if(vout->rotation == 90 || vout->rotation == 270){
			omap2_disp_get_panel_size (omap2_disp_get_output_dev (vout->vid),
				&(vout->fbuf.fmt.height),&(vout->fbuf.fmt.width));
		}
		else{
			omap2_disp_get_panel_size (omap2_disp_get_output_dev (vout->vid),
				&(vout->fbuf.fmt.width), &(vout->fbuf.fmt.height));
			
		}
		omap2_disp_put_dss();

		switch (crop->type){
		case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		{
			err =
			omap24xxvout_new_crop (&vout->pix, &vout->crop, &vout->win,
				&vout->fbuf, &crop->c);
			return err;
		}
		default:
			return -EINVAL;
		}
	}
	case VIDIOC_REQBUFS:
	{
		struct v4l2_requestbuffers *req = (struct v4l2_requestbuffers *) arg;
		struct videobuf_queue *q = &fh->vbq;
		unsigned int phy_addr, virt_addr;
		unsigned int count, i;

		/* don't allow to buffer request for the linked layer */
		if (vout->vid == vout_linked)
			return -EINVAL;

		if (req->count > VIDEO_MAX_FRAME)
			req->count = VIDEO_MAX_FRAME;

		/* we allow req->count == 0 */
		if ((req->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) || (req->count < 0))
			return -EINVAL;

		if (req->memory != V4L2_MEMORY_MMAP)
			return -EINVAL;

		if (vout->streaming)
			return -EBUSY;

		/* reuse old buffers */
		if (vout->buffer_allocated && vout->buffer_size >= vout->pix.sizeimage){
			if (req->count) {
				req->count = vout->buffer_allocated;
				return 0;
			}
		}

		/* 
		 * We allow re-request as long as old buffers are not mmaped.
		 * We will need to free old buffers first.
		 */

		/* either no buffer allocated or too small */
		if (vout->buffer_allocated){
			if (vout->mmap_count)
				return -EBUSY;
			mutex_lock(&q->lock);
			for (i = 0; i < vout->buffer_allocated; i++){

				dma_free_coherent(NULL, vout->buffer_size,
					(void *) q->bufs[i]->dma.vmalloc,
					q->bufs[i]->dma.bus_addr);
				q->bufs[i] = NULL;
			}
			vout->buffer_allocated = 0;
			vout->buffer_size = 0;
			mutex_unlock(&q->lock);
			videobuf_mmap_free(q);
		}
		/* user intends to free buffer */
		if (!req->count)
			return 0;

		mutex_lock(&q->lock);
		count = req->count;
		while (PAGE_ALIGN(vout->pix.sizeimage) * count > render_mem)
			count--;

		// vout->buffer_size = PAGE_ALIGN(vout->pix.sizeimage); 
		/* Allocate for maximum size */
		vout->buffer_size = PAGE_ALIGN(VID_MAX_WIDTH * VID_MAX_HEIGHT * 2); 
		for (i = 0; i < count; i++) {
			virt_addr =
			(unsigned int)dma_alloc_coherent(NULL,
				vout->buffer_size,
				(dma_addr_t *) & phy_addr,
				GFP_KERNEL | GFP_DMA);

			if (!virt_addr)
				break;
			memset((void *) virt_addr, 0, vout->buffer_size);

			DPRINTK("REQBUFS: buffer %d: virt=0x%x, phy=0x%x, size=0x%x\n",
				i, virt_addr, phy_addr, vout->buffer_size);

			q->bufs[i] = videobuf_alloc (q->msize);
			if (q->bufs[i] == NULL){
				dma_free_coherent(NULL, vout->buffer_size,
					(void *) virt_addr,
					(dma_addr_t) phy_addr);
				break;
			}
			q->bufs[i]->i = i;
			q->bufs[i]->input = UNSET;
			q->bufs[i]->memory = req->memory;
			q->bufs[i]->bsize = vout->buffer_size;
			q->bufs[i]->boff = vout->buffer_size * i;
			q->bufs[i]->dma.vmalloc = (void *) (virt_addr);
			q->bufs[i]->dma.bus_addr = phy_addr;
			q->bufs[i]->state = STATE_PREPARED;

			vout->buf_virt_addr[i] = (unsigned long) q->bufs[i]->dma.vmalloc;
			vout->buf_phy_addr[i] = q->bufs[i]->dma.bus_addr;

			vout->buffer_allocated++;
		}
		vout->buf_memory_type = req->memory; 

		/* return -ENOMEM if we are not able to allocate a single buffer
		 * we have to seperate the case which user requests 0 buffer which
		 * means user wants to free the already allocated buffers
		 */

		if (vout->buffer_allocated == 0 && req->count != 0){
			mutex_unlock(&q->lock);
			vout->buffer_size = 0;
			return -ENOMEM;
		}
		req->count = vout->buffer_allocated;
		mutex_unlock(&q->lock);
		return 0;
	}
	case VIDIOC_QUERYBUF:
		return videobuf_querybuf (&fh->vbq, arg);

	case VIDIOC_QBUF:
	{
		struct v4l2_buffer *buffer = (struct v4l2_buffer *) arg;
		struct videobuf_queue *q = &fh->vbq;
		struct omap24xxvout_device *dest;
		int dest_frame_index = 0, src_element_index = 0;
		int dest_element_index = 0, src_frame_index = 0;
		int elem_count = 0, frame_count = 0, pixsize = 2,mir_rot_deg = 0;
		int output_dev = omap2_disp_get_output_dev(vout->vid);	
		int streaming_on = STREAMING_IS_ON(); 	

		timeout = HZ / 5;
		timeout += jiffies; 
	
		omap2_disp_get_tvlcd(&tvlcd_status);
		
		if (!streaming_on)	
		omap2_disp_get_dss();
		if (tvlcd_status.status == TVLCD_STOP){
			
			if (tvlcd_status.ltype == vout->vid){
				omap2_disp_disable_layer (vout->vid);
				vout->streaming = NULL;
			} else if (vout_linked != -1 && vout_linked != vout->vid){
				omap2_disp_disable_layer ((vout->vid == OMAP2_VIDEO1)
				  ? OMAP2_VIDEO2 : OMAP2_VIDEO1);
			}

			omap2_disp_set_output_dev(tvlcd_status.ltype, tvlcd_status.output_dev);
			omap2_disp_set_tvlcd(TVLCD_CONTINUE);
			if (!streaming_on)
			omap2_disp_put_dss();

			return 0;
		}
		
		if (tvlcd_status.status == TVLCD_CONTINUE){

			if (tvlcd_status.ltype == vout->vid){
				vout->streaming = fh;
				omap2_disp_config_vlayer (vout->vid, &vout->pix, &vout->crop,
					  &vout->win, vout->rotation, vout->mirror);
			} else if (vout_linked != -1 && vout_linked != vout->vid){
				dest = (vout_linked == OMAP2_VIDEO1) ? saved_v1out : saved_v2out;
				omap2_disp_config_vlayer (dest-> vid, &dest->pix, &dest-> crop,
					&dest->win, dest->rotation, dest->mirror);
			}

			omap2_disp_set_tvlcd(0);
		}
		
		
		/* don't allow to queue buffer for the linked layer */
		if (vout->vid == vout_linked) {
			if (!streaming_on)
			omap2_disp_put_dss();
			return -EINVAL;
		}

		if (buffer->index >= vout->buffer_allocated) {
			if (!streaming_on)
			omap2_disp_put_dss();
			return -EINVAL;
		}

		if (vout->rotation >= 0
			&& vout->vrfb_dma_tx.req_status == DMA_CHAN_NOT_ALLOTED) {
			if (!streaming_on)
			omap2_disp_put_dss();
			return -EINVAL;
		}

		DPRINTK ("queuing buffer %d ...\n", buffer->index);

		if (!vout->streaming){
			if (!streaming_on)
			omap2_disp_put_dss();
			return -EINVAL;
		}
			
		mutex_lock(&q->lock);

		/* 
		 * If rotation is enabled then copy the image data from the  
		 * MMAPPED area to SMS area 
		 */

		if (vout->rotation >= 0){
		
		
			/*
			 * WAIT FOR GO bit to be cleared or in other terms wait till current 
			 * frame is renderned
			 */
			
			while(omap2_disp_reg_sync_bit(output_dev ) && time_before(jiffies, timeout));
			
			
			pixsize = vout->bpp * vout->vrfb_bpp;
			/*
			 * DMA transfer in double index mode
			 */
	
			/* Frame index */
			dest_frame_index =
				((MAX_PIXELS_PER_LINE * pixsize) - (vout->pix.width * vout->bpp)) + 1;

			/* Source and destination parameters */
			src_element_index = 0;
			src_frame_index = 0;
			dest_element_index = 1;

			/* Number of elements per frame */
			elem_count = vout->pix.width * vout->bpp;
			frame_count = vout->pix.height;
			vout->vrfb_dma_tx.tx_status = 0;
			omap_set_dma_transfer_params (vout->vrfb_dma_tx.dma_ch,
				OMAP_DMA_DATA_TYPE_S32,(elem_count / 4), frame_count,
				OMAP_DMA_SYNC_ELEMENT,vout->vrfb_dma_tx.dev_id, 0x0); 

			omap_set_dma_src_params (vout->vrfb_dma_tx.dma_ch,
				0,	// src_port required only for OMAP1
				OMAP_DMA_AMODE_POST_INC,q->bufs[buffer->index]->dma.bus_addr,
				src_element_index, src_frame_index);
				
			/*set dma source burst mode for VRFB*/
			omap_set_dma_src_burst_mode(vout->vrfb_dma_tx.dma_ch, OMAP_DMA_DATA_BURST_16);

			if (vout->mirror == 1){
				mir_rot_deg = (vout->rotation == 90)?(270/90):
						(vout->rotation == 270)?(90/90):
							(vout->rotation == 180)?(0/90):(180/90);
				
				omap_set_dma_dest_params (vout->vrfb_dma_tx.dma_ch,
						0,	// dest_port required only for OMAP1
						OMAP_DMA_AMODE_DOUBLE_IDX,
						vout->sms_rot_phy[vout->pos][mir_rot_deg],dest_element_index, dest_frame_index);
			}
			else{ /* No Mirroring */
				omap_set_dma_dest_params (vout->vrfb_dma_tx.dma_ch,
					0,	// dest_port required only for OMAP1
					OMAP_DMA_AMODE_DOUBLE_IDX,
					vout->sms_rot_phy[vout->pos][vout->rotation/90],dest_element_index, dest_frame_index);
			}

			/*set dma dest burst mode for VRFB*/
			omap_set_dma_dest_burst_mode(vout->vrfb_dma_tx.dma_ch, OMAP_DMA_DATA_BURST_16);
			omap_dma_set_global_params(DMA_DEFAULT_ARB_RATE, 0x20, 0);

			omap_start_dma (vout->vrfb_dma_tx.dma_ch);
			interruptible_sleep_on_timeout (&vout->vrfb_dma_tx.wait,
					    VRFB_TX_TIMEOUT);
					    
			if (vout->vrfb_dma_tx.tx_status == 0){
				omap_stop_dma (vout->vrfb_dma_tx.dma_ch);
				mutex_unlock(&q->lock);
				if (!streaming_on)
				omap2_disp_put_dss();
				return -EINVAL;
			}
			if (vout->mirror == 1){
				mir_rot_deg = (vout->rotation == 90)? 270:
						(vout->rotation == 270)? 90:
							(vout->rotation == 180)? 0:180;
				omap2_disp_start_vlayer (vout->vid,&vout->pix, &vout->crop, &vout->win,
						vout->sms_rot_phy[vout->pos][0], mir_rot_deg, vout->mirror);
			}
			else{
				omap2_disp_start_vlayer (vout->vid,&vout->pix,&vout->crop, &vout->win,
				vout->sms_rot_phy[vout->pos][0], vout->rotation, vout->mirror);
			}
		}/*if rotation */
		else{
			omap2_disp_start_vlayer (vout->vid, &vout->pix, &vout->crop, &vout->win,
				q->bufs[buffer->index]->dma.bus_addr,vout->rotation, vout->mirror);
		}

		/* start slave layer if V1 and V2 are linked */
		if (vout_linked != -1){
			dest = (vout_linked == OMAP2_VIDEO1) ? saved_v1out : saved_v2out;
			if (dest->streaming){
				if (dest->rotation >= 0){
					/* if rotation and mirroring */
					if (dest->mirror == 1){
						mir_rot_deg = (dest->rotation == 90)? 270:
							(dest->rotation == 270)? 90:
								(dest->rotation == 180)? 0:180;
						
						omap2_disp_start_vlayer(dest->vid,&dest->pix, 
						&dest->crop, &vout->win, vout->sms_rot_phy[vout->pos][0], mir_rot_deg,
						dest->mirror);
					}
					/* if no mirroring but rotation */
					else{
						omap2_disp_start_vlayer(dest->vid,&dest->pix,
						&dest->crop, &vout->win, vout->sms_rot_phy[vout->pos][0], 
						dest->rotation,dest->mirror);
					}
				}
				else {
					omap2_disp_start_vlayer (dest-> vid, &dest->pix, &dest-> crop, &vout->win,
					q->bufs[buffer->index]->dma.bus_addr, dest->rotation,
					dest->mirror);
				}		/* if no rotation and mirroring */
			}
		}

		vout->pos = (vout->pos == 0)? 1 : 0;		
		mutex_unlock(&q->lock);
		if (!streaming_on)
		omap2_disp_put_dss();

		return 0;
	}
	case VIDIOC_DQBUF:
	{
		/* don't allow to dequeue buffer for the linked layer */
		if (vout->vid == vout_linked)
			return -EINVAL;
		return 0;
	}
	case VIDIOC_STREAMON:
	{
		int k;
		if (vout->streaming)
			return -EBUSY;
		/* 
		 * If rotation mode is selected then allocate a 
		 *  dummy or hidden buffer to map the SMS virtual space 
		 */
		if (vout->vid != vout_linked){
			if (vout->rotation >= 0){
				
				/* Align the image size with respect to tile size */
				u32 image_width = 0, image_height = 0;
				
		    image_width = VGA_WIDTH / TILE_SIZE; 
		    						if(VGA_WIDTH % TILE_SIZE)						
		                        image_width++;
		                image_width = image_width * TILE_SIZE;
				
				image_height = VGA_HEIGHT / TILE_SIZE; 
		    						if (VGA_HEIGHT % TILE_SIZE)
		                        image_height++;
		                image_height = image_height * TILE_SIZE;
				
				/* Required to free */
				vout->tile_aligned_psize = image_width * image_height * vout->bpp * vout->vrfb_bpp;
				
				/* have to free and re-allocate */
				if (vout->smsshado_size && vout->smsshado_size < vout->tile_aligned_psize) { 
				
					/* First context per video pipeline */
					dma_free_coherent(NULL, vout->smsshado_size,
						(void *)vout->smsshado_virt_addr[0],
						(dma_addr_t)vout->smsshado_phy_addr[0]);
					
				
					dma_free_coherent(NULL, vout->smsshado_size,
						(void *)vout->smsshado_virt_addr[1],
						(dma_addr_t)vout->smsshado_phy_addr[1]);
					
					vout->smsshado_size = 0;
					vout->smsshado_virt_addr[0] = 0;
					vout->smsshado_virt_addr[1] = 0;
				}

				/* Allocate the actual physical space for VRFB */
				/* 
				 * The allocation is rounded to tile size as we are 
				 * taking care of the offset while reading the data 
				 * and also we will always fill the 0 degree space with 
				 * the image data and read the data requested space (rotation degree)
				 */
				 
				for(k = 0; k < 2; k++){
						if (!vout->smsshado_virt_addr[k]) {
								/* allocate for worst case size */
								image_width = VID_MAX_WIDTH / TILE_SIZE;  
								if (VID_MAX_WIDTH % TILE_SIZE)
										image_width++;

								image_width = image_width * TILE_SIZE;
								image_height = VID_MAX_HEIGHT / TILE_SIZE; 
								 
								if (VID_MAX_HEIGHT % TILE_SIZE)
										image_height++;
								
								image_height = image_height * TILE_SIZE;
								vrfb_buf_size = image_width * image_height * 2 * 2; // allocate for YUV format (worst case)
								vout->smsshado_virt_addr[k] = (unsigned int)
										dma_alloc_coherent(NULL, PAGE_ALIGN(vrfb_buf_size),
														(dma_addr_t *) &vout->smsshado_phy_addr[k], GFP_KERNEL | GFP_DMA);
								vout->smsshado_size = PAGE_ALIGN(vrfb_buf_size);				 		

						}
						
					if (!vout->smsshado_virt_addr[k])
						return -ENOMEM;
					
					memset((void *) vout->smsshado_virt_addr[k], 0, vout->smsshado_size);
	
					if(vout->rotation == 90 || vout->rotation == 270){
						omap2_disp_set_vrfb (vout->vrfb_context[k], vout->smsshado_phy_addr[k], 
						vout->pix.height, vout->pix.width, vout->bpp * vout->vrfb_bpp);
					}
					else{
						omap2_disp_set_vrfb (vout->vrfb_context[k], vout->smsshado_phy_addr[k], 
						vout->pix.width, vout->pix.height, vout->bpp * vout->vrfb_bpp);
					}
				}
			}
		}
		/* set flag here. Next QBUF will start DMA */
		vout->streaming = fh;
		vout->pos	= 0;

		omap2_disp_get_dss();
		/* DMA will be started by QBUF */
		omap2_disp_config_vlayer (vout->vid, &vout->pix, &vout->crop,
				  &vout->win, vout->rotation, vout->mirror);
		return 0;
	}

	case VIDIOC_STREAMOFF:
	{
		/* 
		 * only allow the file handler that started streaming to
		 * stop streaming
		 */

		if (vout->streaming == fh){
			omap2_disp_disable_layer (vout->vid);
			vout->streaming = NULL;

			/* stop the slave layer */
			if (vout_linked != -1 && vout_linked != vout->vid){
				omap2_disp_disable_layer ((vout->vid == OMAP2_VIDEO1)
					  ? OMAP2_VIDEO2 : OMAP2_VIDEO1);
			}
			omap2_disp_put_dss();
			return 0;
		}
		return -EINVAL;
	}


	case VIDIOC_S_OMAP2_LINK:
	{
		int *link = arg;

		spin_lock (&vout_link_lock);

		if ((*link == 0) && (vout_linked == vout->vid))
			vout_linked = -1;

		omap2_disp_get_dss();

		if ((*link == 1) && (vout_linked == -1 || vout_linked == vout->vid)){
			vout_linked = vout->vid;

			if (vout_linked == OMAP2_VIDEO2){
				/* sync V2 to V1 for img and crop */
				omap24xxvout_sync (saved_v2out, saved_v1out);
			}
			else{
				/* sync V1 to V2 */
				omap24xxvout_sync (saved_v1out, saved_v2out);
			}
		}

		omap2_disp_put_dss();

		spin_unlock (&vout_link_lock);
		return 0;
	}
	case VIDIOC_G_OMAP2_LINK:
	{
		int *link = arg;

		spin_lock (&vout_link_lock);
		if (vout_linked == vout->vid) *link = 1;
		else *link = 0;
		spin_unlock (&vout_link_lock);
		return 0;
	}
	case VIDIOC_S_OMAP2_MIRROR:
	{
		int *mirror = arg;
		if ((*mirror == 0) && (vout->mirror == 1)){
			vout->mirror = 0;
			return 0;
		}
		else if ((*mirror == 1) && (vout->mirror == 0)){
			vout->mirror = 1;
			return 0;
		}
		return -EINVAL;
	}
	case VIDIOC_G_OMAP2_MIRROR:
	{
		int *mirror = arg;
		*mirror = vout->mirror;
		return 0;
	}

	case VIDIOC_S_OMAP2_ROTATION:
	{
		int *rotation = arg;

		if ((*rotation == 0) || (*rotation == 90) || (*rotation == 180)
			|| (*rotation == 270) || (*rotation == -1)){
			vout->rotation = (*rotation == 90)?270:(*rotation == 270)?90:*rotation;
			rotation_support = vout->rotation;
			return 0;
		}
		else
			return -EINVAL;
	}
	
	case VIDIOC_G_OMAP2_ROTATION:
	{
		int *rotation = arg;
		*rotation = (vout->rotation == 90)?270:(vout->rotation == 270)?90:vout->rotation;
		return 0;
	}
	case VIDIOC_S_OMAP2_COLORKEY:
	{
		struct omap24xxvout_colorkey *colorkey =
		(struct omap24xxvout_colorkey *) arg;

		if ((colorkey->output_dev != OMAP2_OUTPUT_LCD &&
			colorkey->output_dev != OMAP2_OUTPUT_TV) ||
			(colorkey->key_type != OMAP2_GFX_DESTINATION
			&& colorkey->key_type != OMAP2_VIDEO_SOURCE))
			return -EINVAL;

		omap2_disp_get_dss();

		omap2_disp_set_colorkey (colorkey->output_dev, colorkey->key_type,
			colorkey->key_val);

		omap2_disp_put_dss();

		return 0;

	}
	case VIDIOC_G_OMAP2_COLORKEY:
	{
		struct omap24xxvout_colorkey *colorkey =
		(struct omap24xxvout_colorkey *) arg;

		if (colorkey->output_dev != OMAP2_OUTPUT_LCD
			&& colorkey->output_dev != OMAP2_OUTPUT_TV)
		return -EINVAL;

		omap2_disp_get_dss();

		omap2_disp_get_colorkey (colorkey->output_dev, &colorkey->key_type,
				 &colorkey->key_val);

		omap2_disp_put_dss();

		return 0;

	}
	case VIDIOC_S_OMAP2_BGCOLOR:
	{
		struct omap24xxvout_bgcolor *bgcolor =
		(struct omap24xxvout_bgcolor *) arg;

		if (bgcolor->output_dev != OMAP2_OUTPUT_LCD
			&& bgcolor->output_dev != OMAP2_OUTPUT_TV)
			return -EINVAL;

		omap2_disp_get_dss();

		omap2_disp_set_bg_color (bgcolor->output_dev, bgcolor->color);

		omap2_disp_put_dss();
		
		return 0;
	}
	case VIDIOC_G_OMAP2_BGCOLOR:
	{
		struct omap24xxvout_bgcolor *bgcolor =
			(struct omap24xxvout_bgcolor *) arg;

		if (bgcolor->output_dev != OMAP2_OUTPUT_LCD
			&& bgcolor->output_dev != OMAP2_OUTPUT_TV)
		return -EINVAL;

		omap2_disp_get_dss();

		omap2_disp_get_bg_color (bgcolor->output_dev, &bgcolor->color);

		omap2_disp_put_dss();

		return 0;
	}
	case VIDIOC_OMAP2_COLORKEY_ENABLE:
	{
		int *output_dev = arg;

		if (*output_dev != OMAP2_OUTPUT_LCD && *output_dev != OMAP2_OUTPUT_TV)
		return -EINVAL;

		omap2_disp_get_dss();

		omap2_disp_enable_colorkey (*output_dev);

		omap2_disp_put_dss();

		return 0;
	}

	case VIDIOC_OMAP2_COLORKEY_DISABLE:
	{
		int *output_dev = arg;

		if (*output_dev != OMAP2_OUTPUT_LCD && *output_dev != OMAP2_OUTPUT_TV)
			return -EINVAL;

		omap2_disp_get_dss();

		omap2_disp_disable_colorkey (*output_dev);

		omap2_disp_put_dss();

		return 0;
	}
	
	case VIDIOC_S_OMAP2_COLORCONV:
	{
		int v; 
		struct omap24xxvout_colconv *ccmtx =
                        (struct omap24xxvout_colconv *) arg;

		if(vout->vid == OMAP2_VIDEO1) v =0;
		else v =1;

		current_colorconv_values[v][0][0]=ccmtx->RY;
		current_colorconv_values[v][0][1]=ccmtx->RCr;
		current_colorconv_values[v][0][2]=ccmtx->RCb;
		current_colorconv_values[v][1][0]=ccmtx->GY;
		current_colorconv_values[v][1][1]=ccmtx->GCr;
		current_colorconv_values[v][1][2]=ccmtx->GCb;
		current_colorconv_values[v][2][0]=ccmtx->BY;
		current_colorconv_values[v][2][1]=ccmtx->BCr;
		current_colorconv_values[v][2][2]=ccmtx->BCb;
		omap2_disp_get_dss();
		omap2_disp_set_colorconv(vout->vid, &vout->pix);	
		omap2_disp_put_dss();
		return 0;
	}

	case VIDIOC_G_OMAP2_COLORCONV:
	{
		int v; 
		struct omap24xxvout_colconv *ccmtx =
                        (struct omap24xxvout_colconv *) arg;

		if(vout->vid == OMAP2_VIDEO1) v =0;
		else v =1;

		ccmtx->RY = current_colorconv_values[v][0][0];
		ccmtx->RCr= current_colorconv_values[v][0][1];
		ccmtx->RCb= current_colorconv_values[v][0][2];
		ccmtx->GY = current_colorconv_values[v][1][0];
		ccmtx->GCr= current_colorconv_values[v][1][1];
		ccmtx->GCb= current_colorconv_values[v][1][2];
		ccmtx->BY = current_colorconv_values[v][2][0];
		ccmtx->BCr= current_colorconv_values[v][2][1];
		ccmtx->BCb= current_colorconv_values[v][2][2];

		return 0;
	}

	case VIDIOC_S_OMAP2_DEFCOLORCONV:
	{
		omap2_disp_get_dss();

		omap2_disp_set_default_colorconv(vout->vid, &vout->pix);	

		omap2_disp_put_dss();

		return 0;
	}

	default:
		/* unrecognized ioctl */
		return -ENOIOCTLCMD;

	}				/* switch */

	return 0;
}

/* -------------------------------------------------------------------------- */

/*
 *  file operations
 */

#ifdef DEBUG_ALLOW_WRITE

static ssize_t
omap24xxvout_write (struct file *file, const char *data, size_t count,
		    loff_t * ppos)
{
	struct omap24xxvout_fh *fh = file->private_data;
	struct omap24xxvout_device *vout = fh->vout;

	omap2_disp_get_dss();

	omap24xxvout_suspend_lockout (vout, file);
	if ((*ppos) >= vout->pix.sizeimage) {
		omap2_disp_put_dss();
		return 0;
	}
	if (count + (*ppos) > vout->pix.sizeimage)
		count = vout->pix.sizeimage - (*ppos);
	if (copy_from_user
		((void *) (vout->framebuffer_base + (*ppos)), data, count)){
		printk ("error in copying data");
	}
	*ppos += count;

	#if 1
		if (*ppos == vout->pix.sizeimage)
		mdelay (5000);
	#endif

	omap2_disp_put_dss();

	return count;
}

#endif

static void
omap24xxvout_vm_open (struct vm_area_struct *vma)
{
	struct omap24xxvout_device *vout = vma->vm_private_data;
	DPRINTK ("vm_open [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count++;
}

static void
omap24xxvout_vm_close (struct vm_area_struct *vma)
{
	struct omap24xxvout_device *vout = vma->vm_private_data;
	DPRINTK ("vm_close [vma=%08lx-%08lx]\n", vma->vm_start, vma->vm_end);
	vout->mmap_count--;
}

static struct vm_operations_struct omap24xxvout_vm_ops = {
	.open = omap24xxvout_vm_open,
	.close = omap24xxvout_vm_close,
};

static int
omap24xxvout_mmap (struct file *file, struct vm_area_struct *vma)
{
	struct omap24xxvout_fh *fh = file->private_data;
	struct omap24xxvout_device *vout = fh->vout;
	struct videobuf_queue *q = &fh->vbq;
	unsigned long size = (vma->vm_end - vma->vm_start), start = vma->vm_start;
	int i;
	void *pos; 

	DPRINTK ("pgoff=0x%x, start=0x%x, end=0x%x\n", vma->vm_pgoff, 
		vma->vm_start,  vma->vm_end);


	mutex_lock(&q->lock);

	/* look for the buffer to map */
	for (i = 0; i < VIDEO_MAX_FRAME; i++){
		if (NULL == q->bufs[i])	continue;
		if (V4L2_MEMORY_MMAP != q->bufs[i]->memory)
			continue;
		if (q->bufs[i]->boff == (vma->vm_pgoff << PAGE_SHIFT))
			break;
	}

	if (VIDEO_MAX_FRAME == i){
		DPRINTK ("offset invalid [offset=0x%lx]\n",
	       (vma->vm_pgoff << PAGE_SHIFT));
		mutex_unlock(&q->lock);
		return -EINVAL;
	}

	vma->vm_flags |= VM_RESERVED;
	vma->vm_page_prot = pgprot_writecombine (vma->vm_page_prot);
	vma->vm_ops = &omap24xxvout_vm_ops;
	vma->vm_private_data = (void *) vout;
	pos = q->bufs[i]->dma.vmalloc;

	while (size > 0) {      /* size is page-aligned */
	if (vm_insert_page(vma, start, vmalloc_to_page(pos) )){
		mutex_unlock(&q->lock);
		return -EAGAIN;
	}
	start += PAGE_SIZE;
	pos += PAGE_SIZE;
	size -= PAGE_SIZE;
	}
	vma->vm_flags &= ~VM_IO; /* using shared anonymous pages */
	/* under investigation. 
	clearing this bit allows video-buf to accept buffers alloacted here but has troube when unmap */
	//vma->vm_flags &= ~VM_PFNMAP; 

	mutex_unlock(&q->lock);
	vout->mmap_count++;
	return 0;
}

static int
omap24xxvout_ioctl (struct inode *inode, struct file *file, unsigned int cmd,
		    unsigned long arg)
{
	struct omap24xxvout_fh *fh = file->private_data;
	struct omap24xxvout_device *vout = fh->vout;

	omap24xxvout_suspend_lockout (vout, file);
	return video_usercopy (inode, file, cmd, arg, omap24xxvout_do_ioctl);
}

static int
omap24xxvout_release (struct inode *inode, struct file *file)
{
	struct omap24xxvout_fh *fh = file->private_data;
	struct omap24xxvout_device *vout;
	struct videobuf_queue *q;

	DPRINTK ("entering\n");

	if (fh == 0)
		return 0;
	if ((vout = fh->vout) == 0)
		return 0;
	q = &fh->vbq;

	omap2_disp_get_dss();
	omap24xxvout_suspend_lockout (vout, file);

	/* 
	 * Check if the hidden buffer transfer is happening with DMA
	 * if yes then stop it
	 */
	
	if (vout->rotation != -1){
		if (vout->vrfb_dma_tx.tx_status == 0){

			/* 
			 * DMA will be stopped once here and again after wakeup to 
			 * avoid race conditions due to time taken to wakeup the 
			 * sleeping process
			 */
			
			omap_stop_dma (vout->vrfb_dma_tx.dma_ch);
			wake_up_interruptible(&vout->vrfb_dma_tx.wait);	
		}
	}

	omap2_disp_disable_layer (vout->vid);

	if ((vout_linked != -1) && (vout->vid != vout_linked))
	omap2_disp_disable_layer ((vout->vid ==
			       OMAP2_VIDEO1) ? OMAP2_VIDEO2 : OMAP2_VIDEO1);

#ifdef DEBUG_ALLOW_WRITE
	if (vout->framebuffer_base){

		dma_free_coherent(NULL, vout->framebuffer_size,
					(void *) vout->framebuffer_base,
					vout->framebuffer_base_phys);
		vout->framebuffer_base = 0;
	}
	vout->framebuffer_base_phys = 0;
#endif

	if (vout->streaming == fh) vout->streaming = NULL;

	if (vout->mmap_count != 0){
		vout->mmap_count = 0;
		printk("mmap count is not zero!\n");
	}

	omap2_disp_release_layer (vout->vid);
	omap2_disp_put_dss ();
	vout->opened -= 1;
	file->private_data = NULL;

	if (vout->buffer_allocated)
		videobuf_mmap_free(q);

	kfree (fh);

	/* need to remove the link when the either slave or master is gone */
	spin_lock (&vout_link_lock);
	if (vout_linked != -1) {
		vout_linked = -1;
	}
	spin_unlock (&vout_link_lock);

	return 0;
}

/* define this to allow videobuf_mmap_free to work */
static void dummy_vbq_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
}
static int
omap24xxvout_open (struct inode *inode, struct file *file)
{
	int minor = MINOR (file->f_dentry->d_inode->i_rdev);
	struct omap24xxvout_device *vout = NULL;
	struct omap24xxvout_fh *fh;
	struct videobuf_queue *q;
	int i;

	DPRINTK ("entering\n");

	if (saved_v1out && saved_v1out->vfd && (saved_v1out->vfd->minor == minor)){
		vout = saved_v1out;
	}

	if (vout == NULL){
		if (saved_v2out && saved_v2out->vfd
			&& (saved_v2out->vfd->minor == minor)){
		vout = saved_v2out;
		}
	}

	if (vout == NULL)
		return -ENODEV;

	/* for now, we only support single open */
	if (vout->opened)
		return -EBUSY;

	vout->opened += 1;
	if (!omap2_disp_request_layer (vout->vid)){
		vout->opened -= 1;
		return -ENODEV;
	}

	omap2_disp_get_dss ();
	omap24xxvout_suspend_lockout (vout, file);
	/* allocate per-filehandle data */
	fh = kmalloc (sizeof (*fh), GFP_KERNEL);
	if (NULL == fh){
		omap2_disp_release_layer (vout->vid);
		omap2_disp_put_dss ();
		vout->opened -= 1;
		return -ENOMEM;
	}

	file->private_data = fh;
	fh->vout = vout;
	fh->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

#ifdef DEBUG_ALLOW_WRITE
	vout->framebuffer_size = VID_MAX_HEIGHT * VID_MAX_HEIGHT * 2;
	vout->framebuffer_base = dma_alloc_coherent(NULL,vout->framebuffer_size,
			(dma_addr_t *) & vout->	framebuffer_base_phys,
			GFP_KERNEL | GFP_DMA);

	if (!vout->framebuffer_base){
		kfree (fh);
		omap2_disp_release_layer (vout->vid);
		omap2_disp_put_dss ();
		vout->opened -= 1;
		return -ENOMEM;
	}
	memset ((void *) vout->framebuffer_base, 0, vout->framebuffer_size);
		omap2_disp_config_vlayer (vout->vid, &vout->pix, &vout->crop, &vout->win,
		vout->rotation, vout->mirror);
	omap2_disp_start_vlayer (vout->vid, &vout->pix, &vout->crop,
		vout->framebuffer_base_phys, vout->rotation,
		vout->mirror);
#endif

	q = &fh->vbq;
	dummy_vbq_ops.buf_release = dummy_vbq_release;
	videobuf_queue_init (q, &dummy_vbq_ops, NULL, &vout->vbq_lock,
		fh->type, V4L2_FIELD_NONE, sizeof (struct videobuf_buffer), fh);

	/* restore info for mmap */
	for (i = 0; i < vout->buffer_allocated; i++) {
		q->bufs[i] = videobuf_alloc (q->msize);
		if (q->bufs[i] == NULL){
			dma_free_coherent(NULL, vout->buffer_size,
					(void *) vout->buf_virt_addr[i],
					(dma_addr_t) vout->buf_phy_addr[i]);
			/* we have to give up some old buffers */
			vout->buffer_allocated = i;
			break;
		}
		q->bufs[i]->i = i;
		q->bufs[i]->input = UNSET;
		q->bufs[i]->memory = vout->buf_memory_type;
		q->bufs[i]->bsize = vout->buffer_size;
		q->bufs[i]->boff = vout->buffer_size * i;
		q->bufs[i]->dma.vmalloc = (void *) vout->buf_virt_addr[i];
		q->bufs[i]->dma.bus_addr = vout->buf_phy_addr[i];
		q->bufs[i]->state = STATE_PREPARED;
	}

	omap2_disp_put_dss();

	return 0;
}

static struct file_operations omap24xxvout_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
#ifdef DEBUG_ALLOW_WRITE
	.write = omap24xxvout_write,
#endif
	.ioctl = omap24xxvout_ioctl,
	.mmap = omap24xxvout_mmap,
	.open = omap24xxvout_open,
	.release = omap24xxvout_release,
};

/* -------------------------------------------------------------------------- */
#ifdef CONFIG_PM
static int
omap24xxvout_suspend (struct platform_device *dev, pm_message_t state)
{
	struct omap24xxvout_device *vout = platform_get_drvdata (dev);
	
	/* lock-out applications during suspend */
	if(vout->suspended == 1) return 0;
	if (vout->opened){
		/* stall vid DMA */
		if (vout->streaming){
			omap2_disp_disable_layer (vout->vid);
			/*
			 * Check if the hidden buffer transfer is happening with DMA
			 * if yes then stop it
			 */
			
			if (vout->rotation != -1){
				if (vout->vrfb_dma_tx.tx_status == 0){
					/*
					 * DMA will be stopped once here and again after wakeup to
					 * avoid race conditions due to time taken to wakeup the
				 	 * sleeping process
				 	 */
					omap_stop_dma (vout->vrfb_dma_tx.dma_ch);
					wake_up_interruptible (&vout->vrfb_dma_tx.wait);
				}
			}
		}
		vout->suspended = 1;
		omap2_disp_put_dss ();
	}
	
	
	return 0;
}

static int
omap24xxvout_resume (struct platform_device *dev)
{
	struct omap24xxvout_device *vout = platform_get_drvdata (dev);
	if(vout->suspended == 0) return 0;
	if (vout->opened)
	{
		omap2_disp_get_dss ();
		
		/* resume vid DMA */
		if (vout->streaming)
			omap2_disp_enable_layer (vout->vid);
			
		/* wake up applications waiting on suspend queue */
		vout->suspended = 0;
		wake_up (&vout->suspend_wq);
	}
	return 0;
}


#ifdef CONFIG_DPM

static struct constraints omap24xxvout_constraints = {
	.count = 2,
	.param = {
		{DPM_MD_V, OMAP24XX_V_MIN, OMAP24XX_V_MAX},
		{DPM_MD_SLEEP_MODE, PM_SUSPEND_STANDBY, PM_SUSPEND_MEM},
	},
};

static int
omap24xxvout_scale (int vid, struct notifier_block *op, unsigned long level)
{
	struct omap24xxvout_device *vout;
	vout = (vid == OMAP2_VIDEO1) ? saved_v1out : saved_v2out;
	if (!vout->opened)
		return 0;
	switch (level){
	case SCALE_PRECHANGE:
		if (vout->streaming){
			omap2_disp_disable_layer (vout->vid);
			if (vout->rotation != -1){
				if (vout->vrfb_dma_tx.tx_status == 0){
					omap_stop_dma (vout->vrfb_dma_tx.dma_ch);
				}
			}
		}
	break;
	case SCALE_POSTCHANGE:
		if (vout->streaming){
			omap2_disp_enable_layer (vout->vid);
			if (vout->rotation != -1){
				if (vout->vrfb_dma_tx.tx_status == 0){
					omap_start_dma (vout->vrfb_dma_tx.dma_ch);
				}
			}
		}
	break;
	}
	return 0;
}

static int
omap24xxv1out_scale (struct notifier_block *op, unsigned long level, void *ptr)
{
	return omap24xxvout_scale (OMAP2_VIDEO1, op, level);
}

static int
omap24xxv2out_scale (struct notifier_block *op, unsigned long level, void *ptr)
{
	return omap24xxvout_scale (OMAP2_VIDEO2, op, level);
}

static struct notifier_block omap24xxv1out_pre_scale = {
	.notifier_call = omap24xxv1out_scale,
};

static struct notifier_block omap24xxv1out_post_scale = {
	.notifier_call = omap24xxv1out_scale,
};

static struct notifier_block omap24xxv2out_pre_scale = {
	.notifier_call = omap24xxv2out_scale,
};

static struct notifier_block omap24xxv2out_post_scale = {
	.notifier_call = omap24xxv2out_scale,
};

#endif
#endif /* PM */

static int
omap24xxvout_probe (struct platform_device *dev)
{
	return 0;
}


#if 0

static struct platform_device omap24xxv1out_dev = {
	.name = V1OUT_NAME,
	.devid = OMAP24xx_V1OUT_DEVID,
	.busid = OMAP_BUS_L3,
	.dev = {
#ifdef CONFIG_DPM
		.constraints = &omap24xxvout_constraints,
#endif
	},
};

static struct platform_device omap24xxv2out_dev = {
	.name = V2OUT_NAME,
	.devid = OMAP24xx_V2OUT_DEVID,
	.busid = OMAP_BUS_L3,
	.dev = {
#ifdef CONFIG_DPM
		.constraints = &omap24xxvout_constraints,
#endif
	},
};

static struct omap_driver omap24xxv1out_driver = {
	.drv = {
		.name = V1OUT_NAME
	},
	.devid   = OMAP24xx_V1OUT_DEVID,
	.busid   = OMAP_BUS_L3,
	.clocks  = 0,
	.probe   = omap24xxvout_probe,
#ifdef CONFIG_PM
	.suspend = omap24xxvout_suspend,
	.resume  = omap24xxvout_resume,
#endif
};

static struct omap_driver omap24xxv2out_driver = {
	.drv = {
		.name = V2OUT_NAME
	},
	.devid   = OMAP24xx_V2OUT_DEVID,
	.busid   = OMAP_BUS_L3,
	.clocks  = 0,
	.probe   = omap24xxvout_probe,
#ifdef CONFIG_PM
	.suspend = omap24xxvout_suspend,
	.resume  = omap24xxvout_resume,
#endif
};

#else
static struct platform_device omap24xxv1out_dev = {
	.name = V1OUT_NAME,
	.id = 11,
	//.devid = OMAP24xx_V1OUT_DEVID,
	// .busid = OMAP_BUS_L3,
	.dev = {
#ifdef CONFIG_DPM
		.constraints = &omap24xxvout_constraints,
#endif
	},
};

static struct platform_device omap24xxv2out_dev = {
	.name = V2OUT_NAME,
	.id = 12,
	//.devid = OMAP24xx_V2OUT_DEVID,
	//.busid = OMAP_BUS_L3,
	.dev = {
#ifdef CONFIG_DPM
		.constraints = &omap24xxvout_constraints,
#endif
	},
};

static struct platform_driver omap24xxv1out_driver = {
	.driver = {
		.name = V1OUT_NAME,
	},
	//.devid   = OMAP24xx_V1OUT_DEVID,
	// .id   = 11,
	//.busid   = OMAP_BUS_L3,
	// .clocks  = 0,
	.probe   = omap24xxvout_probe,
#ifdef CONFIG_PM
	.suspend = omap24xxvout_suspend,
	.resume  = omap24xxvout_resume,
#endif
};

static struct platform_driver omap24xxv2out_driver = {
	.driver = {
		.name = V2OUT_NAME,
	},
	//.id   = 12,
	//.devid   = OMAP24xx_V2OUT_DEVID,
	//.busid   = OMAP_BUS_L3,
	//.clocks  = 0,
	.probe   = omap24xxvout_probe,
#ifdef CONFIG_PM
	.suspend = omap24xxvout_suspend,
	.resume  = omap24xxvout_resume,
#endif
};

#endif
/* -------------------------------------------------------------------------- */

static void
cleanup_vout_device (int vid)
{
	struct video_device *vfd;
	struct omap24xxvout_device *vout;
	int k;

	vout = (vid == OMAP2_VIDEO1) ? saved_v1out : saved_v2out;
	if (!vout)
		return;
	vfd = vout->vfd;

	if (vfd){
		if (vfd->minor == -1){
			/* 
			 * The device was never registered, so release the 
			 * video_device struct directly. 
			 */
			video_device_release (vfd);
		}
		else{
			/* 
			 * The unregister function will release the video_device
			 * struct as well as unregistering it. 
			 */
			video_unregister_device (vfd);
		}
	}

	if (vout->rotation != -1){
		for(k = 0; k < 2; k++){
			release_mem_region (vout->sms_rot_phy[k][3], VRF_SIZE);
			release_mem_region (vout->sms_rot_phy[k][2], VRF_SIZE);
			release_mem_region (vout->sms_rot_phy[k][1], VRF_SIZE);
			release_mem_region (vout->sms_rot_phy[k][0], VRF_SIZE);
		}
	}

	if (vout->vrfb_dma_tx.req_status == DMA_CHAN_ALLOTED){
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
		omap_free_dma (vout->vrfb_dma_tx.dma_ch);
	}
#if 0
	platform_deviceice_unregister ((vid ==	OMAP2_VIDEO1) ? &omap24xxv1out_dev :
		&omap24xxv2out_dev);
	omap_driver_unregister ((vid == OMAP2_VIDEO1) ? &omap24xxv1out_driver :
		&omap24xxv2out_driver);
#endif
	platform_device_unregister ((vid ==	OMAP2_VIDEO1) ? &omap24xxv1out_dev :
		&omap24xxv2out_dev);
	platform_driver_unregister ((vid == OMAP2_VIDEO1) ? &omap24xxv1out_driver :
		&omap24xxv2out_driver);



#ifdef CONFIG_DPM
	if (vid == OMAP2_VIDEO1){
		dpm_unregister_scale(&omap24xxv1out_pre_scale,SCALE_PRECHANGE);
		dpm_unregister_scale(&omap24xxv1out_post_scale,SCALE_POSTCHANGE);
	}
	else{
		dpm_unregister_scale(&omap24xxv2out_pre_scale,SCALE_PRECHANGE);
		dpm_unregister_scale(&omap24xxv2out_pre_scale,SCALE_POSTCHANGE);
	}
#endif

	kfree (vout);

	if (vid == OMAP2_VIDEO1)
		saved_v1out = NULL;
	else
		saved_v2out = NULL;
}

static struct omap24xxvout_device *
init_vout_device (int vid)
{
	int r, k;
	struct omap24xxvout_device *vout;
	struct video_device *vfd;
	struct v4l2_pix_format *pix;
	struct platform_driver *this_driver;
	struct platform_device *this_dev;

	vout = kmalloc (sizeof (struct omap24xxvout_device), GFP_KERNEL);
	if (!vout){
		printk (KERN_ERR VOUT_NAME ": could not allocate memory\n");
		return NULL;
	}

	memset (vout, 0, sizeof (struct omap24xxvout_device));
	vout->vid = vid;
	vout->rotation = rotation_support;

	/* set the default pix */
	pix = &vout->pix;
	pix->width = QQVGA_WIDTH;
	pix->height = QQVGA_HEIGHT;

	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_JPEG;

	vout->bpp = RGB565_BPP;
	vout->vrfb_bpp = 1;
	
	/* get the screen parameters */
	omap2_disp_get_panel_size (omap2_disp_get_output_dev (vout->vid),
		&(vout->fbuf.fmt.width),&(vout->fbuf.fmt.height));

	/* set default crop and win */
	omap24xxvout_new_format (pix, &vout->fbuf, &vout->crop, &vout->win);

	/* initialize the video_device struct */
	vfd = vout->vfd = video_device_alloc ();
	if (!vfd){
		printk (KERN_ERR VOUT_NAME": could not allocate video device struct\n");
		kfree (vout);
		return NULL;
	}
	vfd->release = video_device_release;

	strncpy (vfd->name, VOUT_NAME, sizeof (vfd->name));
	vfd->type = VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;
	/* need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->hardware = 0;
	vfd->fops = &omap24xxvout_fops;
	video_set_drvdata (vfd, vout);
	vfd->minor = -1;
	
	/* SMS memory starts from 0x70000000 - 256 MB region */
	vout->sms_rot_phy[0][0] = (vid == OMAP2_VIDEO1) ? 0x74000000 : 0x7C000000;
	vout->sms_rot_phy[0][1] = (vid == OMAP2_VIDEO1) ? 0x75000000 : 0x7D000000;
	vout->sms_rot_phy[0][2] = (vid == OMAP2_VIDEO1) ? 0x76000000 : 0x7E000000;
	vout->sms_rot_phy[0][3] = (vid == OMAP2_VIDEO1) ? 0x77000000 : 0x7F000000;
	vout->sms_rot_phy[1][0] = (vid == OMAP2_VIDEO1) ? 0x78000000 : 0xE0000000;
	vout->sms_rot_phy[1][1] = (vid == OMAP2_VIDEO1) ? 0x79000000 : 0xE1000000;
	vout->sms_rot_phy[1][2] = (vid == OMAP2_VIDEO1) ? 0x7A000000 : 0xE2000000;
	vout->sms_rot_phy[1][3] = (vid == OMAP2_VIDEO1) ? 0x7B000000 : 0xE3000000;

	/* let's request memory region for them */
	/* request the mem region for the SMS space */
	for(k = 0; k < 2; k++){
		if (!request_mem_region (vout->sms_rot_phy[k][0], VRF_SIZE, vfd->name)){
		printk (KERN_ERR VOUT_NAME": cannot reserve sms I/O region - 0 \n");
		goto init_error;
	}
		if (!request_mem_region (vout->sms_rot_phy[k][1], VRF_SIZE, vfd->name)){
		printk (KERN_ERR VOUT_NAME ": cannot reserve sms I/O region - 1\n");
		goto rotation_sms_free_0;
	}
		if (!request_mem_region (vout->sms_rot_phy[k][2], VRF_SIZE, vfd->name)){
		printk (KERN_ERR VOUT_NAME ": cannot reserve sms I/O region - 2\n");
		goto rotation_sms_free_90;
	}
		if (!request_mem_region (vout->sms_rot_phy[k][3], VRF_SIZE, vfd->name)){
		printk (KERN_ERR VOUT_NAME ": cannot reserve sms I/O region - 3\n");
		goto rotation_sms_free_180;	
		/* free them in reverse order - otherewise you will free
		   extra stuff and get caught */
	}
	}
	
	vout->suspended = 0;
	init_waitqueue_head (&vout->suspend_wq);

	if (video_register_device (vfd, VFL_TYPE_GRABBER, vid) < 0){
		printk (KERN_ERR VOUT_NAME": could not register Video for Linux device\n");
		vfd->minor = -1;
		if (vout->rotation != -1)
			goto rotation_sms_free_270;
		else
			goto init_error;
	}

	this_driver = (vid == OMAP2_VIDEO1) ? &omap24xxv1out_driver : &omap24xxv2out_driver;
	this_dev = (vid == OMAP2_VIDEO1) ? &omap24xxv1out_dev : &omap24xxv2out_dev;
	if (platform_driver_register (this_driver) != 0){
		printk (KERN_ERR VOUT_NAME": could not register Video driver\n");
		cleanup_vout_device (vid);
		return NULL;
	}
	if (platform_device_register (this_dev) != 0){
		printk (KERN_ERR VOUT_NAME": could not register Video device\n");
		cleanup_vout_device (vid);
		return NULL;
	}
	/* set driver specific data to use in power mgmt functions */
	// omap_set_drvdata (this_dev, vout);
	platform_set_drvdata (this_dev, vout);

	if(vid == OMAP2_VIDEO1){
		vout->vrfb_context[0] = 1;
		vout->vrfb_context[1] = 2;
	}else{
		vout->vrfb_context[0] = 3;
		vout->vrfb_context[1] = 4;
	}


#ifdef CONFIG_DPM
	/* Scaling is enabled only when DPM is enabled */
	if(vid == OMAP2_VIDEO1){
		dpm_register_scale(&omap24xxv1out_pre_scale,SCALE_PRECHANGE);
		dpm_register_scale(&omap24xxv1out_post_scale,SCALE_POSTCHANGE);
	}
	else{
		dpm_register_scale(&omap24xxv2out_pre_scale,SCALE_PRECHANGE);
		dpm_register_scale(&omap24xxv2out_post_scale,SCALE_POSTCHANGE);
	}
#endif

	/*
	 * Request and Initialize DMA, for DMA based VRFB transfer
	 */
	vout->vrfb_dma_tx.dev_id = OMAP_DMA_NO_DEVICE;
	vout->vrfb_dma_tx.dma_ch = -1;
	vout->vrfb_dma_tx.req_status = DMA_CHAN_ALLOTED;
	r = omap_request_dma (vout->vrfb_dma_tx.dev_id,
		"VRFB DMA TX", vrfb_dma_tx_callback,(void *) &vout->vrfb_dma_tx,
			&vout->vrfb_dma_tx.dma_ch);
	if (r)
		vout->vrfb_dma_tx.req_status = DMA_CHAN_NOT_ALLOTED;
	init_waitqueue_head (&vout->vrfb_dma_tx.wait);
	
	/*if rotation support */
	printk (KERN_INFO VOUT_NAME ": registered device video%d [v4l2]\n",
		vfd->minor);
	return vout;

rotation_sms_free_270:
	release_mem_region (vout->sms_rot_phy[0][3], VRF_SIZE);
	release_mem_region (vout->sms_rot_phy[1][3], VRF_SIZE);
rotation_sms_free_180:
	release_mem_region (vout->sms_rot_phy[0][2], VRF_SIZE);
	release_mem_region (vout->sms_rot_phy[1][2], VRF_SIZE);
rotation_sms_free_90:
	release_mem_region (vout->sms_rot_phy[0][1], VRF_SIZE);
	release_mem_region (vout->sms_rot_phy[1][1], VRF_SIZE);
rotation_sms_free_0:
	release_mem_region (vout->sms_rot_phy[0][0], VRF_SIZE);
	release_mem_region (vout->sms_rot_phy[1][0], VRF_SIZE);

init_error:
	video_device_release (vfd);
	kfree (vout);
	return NULL;
}

static int __init
omap24xxvout_init (void)
{
	omap2_disp_get_dss();
	saved_v1out = init_vout_device (OMAP2_VIDEO1);
	if (saved_v1out == NULL) {
		omap2_disp_put_dss();
		return -ENODEV;
	}
	omap2_disp_save_initstate(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_save_initstate(OMAP2_VIDEO1);

	saved_v2out = init_vout_device (OMAP2_VIDEO2);
	if (saved_v2out == NULL){
		cleanup_vout_device (OMAP2_VIDEO1);
		omap2_disp_put_dss();
		return -ENODEV;
	}
	omap2_disp_save_initstate(OMAP_DSS_DISPC_GENERIC);
	omap2_disp_save_initstate(OMAP2_VIDEO2);
	omap2_disp_put_dss();

	vout_linked = -1;
	spin_lock_init (&vout_link_lock);
	return 0;
}

static void
omap24xxvout_cleanup (void)
{
	omap2_disp_get_dss();
	cleanup_vout_device (OMAP2_VIDEO1);
	cleanup_vout_device (OMAP2_VIDEO2);
	omap2_disp_put_dss();
}

#ifndef MODULE
/*
 *	omap24xxvout_setup - process command line options
 *	@options: string of options
 *
 *	NOTE: This function is a __setup and __init function.
 *
 *	Returns zero.
 */
int __init
omap24xxvout_setup (char *options)
{
	char *this_opt;
	int i;

	if (!options || !*options)return 0;

	DPRINTK ("Options \"%s\"\n", options);
	i = strlen (VOUT_NAME);
	if (!strncmp (options, VOUT_NAME, i) && options[i] == ':'){
		this_opt = options + i + 1;
		if (!this_opt || !*this_opt)
			return 0;

		if (!strncmp (this_opt, "rotation=", 9)){
			int deg = simple_strtoul (this_opt + 9, NULL, 0);
			switch (deg){
			case 0:
			case 90:
			case 180:
			case 270:
				rotation_support = (deg == 90)?270:(deg == 270)?90:deg;
			break;
			default:
				rotation_support = -1;
			break;
			}
			printk (KERN_INFO VOUT_NAME ": Rotation %s\n",
			(rotation_support == -1) ? "none (supported: \"rotation=[-1|0|90|180|270]\")" :
			this_opt + 9);
		}
		else
			printk (KERN_INFO VOUT_NAME ": Invalid parameter \"%s\" "
			"(supported: \"rotation=[-1|0|90|180|270]\")\n", this_opt);
			return 0;
	}

	/*
	 * If we get here no fb was specified.
	 * We consider the argument to be a global video mode option.
	 */
	/* TODO - remove when FB is configured */
#if 1
	global_mode_option = options;
#endif
	return 0;
}

__setup ("videoout=", omap24xxvout_setup);
#endif

MODULE_AUTHOR ("Texas Instruments.");
MODULE_DESCRIPTION ("OMAP24xx Video for Linux Video out driver");
MODULE_LICENSE ("GPL");
/* TODO -- Enabling it results in build erros, why?? */
#if 0
module_param(render_mem, uint, VID_MAX_WIDTH * VID_MAX_HEIGHT * 4 * MAX_ALLOWED_VIDBUFFERS);
MODULE_PARM_DESC (render_mem,
		  "Maximum rendering memory size (default 1.2MB)");
#endif
module_init (omap24xxvout_init);
module_exit (omap24xxvout_cleanup);
