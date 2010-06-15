/* drivers/video/msm_fb/msm_fb.c
 *
 * Core MSM framebuffer driver.
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <linux/freezer.h>
#include <linux/wait.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <linux/msm_mdp.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/arch/msm_fb.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/msm_mdp.h>
#include <linux/file.h>

#include "mdp_hw.h"

#define MSMFB_DEBUG 0
#ifdef CONFIG_FB_MSM_LOGO
#define INIT_IMAGE_FILE "/logo.rle"
extern int load_565rle_image( char *filename );
#endif

//Debug flags
#define PRINT_FPS 0
#define PRINT_VFPS 0
#define PRINT_BLIT_TIME 0
#define PRINT_DMA_TIMING 0

#define SLEEPING 0x4
#define UPDATING 0x3
#define FULL_UPDATE_DONE 0x2
#define WAKING 0x1
#define AWAKE 0x0

#define NONE 0
#define SUSPEND_RESUME 0x1
#define FPS 0x2
#define BLIT_TIME 0x4
#define VIDEO_IDLE_THRESHOLD_MS 32
#define DMA_IDLE_THRESHOLD_MS 500

#define ARGB_BUFFER_SIZE 320*400*4
#define RGB_BUFFER_SIZE 320*400*3
#define FB0_BUFFER_NUM	3

#if MSMFB_DEBUG
#define	MDP_LOG_ENTER()		(printk(KERN_INFO"%s: called\n",\
					__PRETTY_FUNCTION__))
#define	MDP_LOG_EXIT()		(printk(KERN_INFO"%s: exit\n",\
					__PRETTY_FUNCTION__))
#define MDP_LOG_INFO(args...)	(printk(KERN_INFO args))					
#else
#define MDP_LOG_ENTER()
#define MDP_LOG_EXIT()
#define MDP_LOG_INFO(args...)
#endif // MSMFB_DEBUG			

/* define the custom FBIO_WAITFORVSYNC ioctl */
#define FBIO_WAITFORVSYNC	_IOW('F', 0x20, u_int32_t)		

#define DLOG(mask,fmt,args...) \
do { \
if (msmfb_debug_mask & mask) \
	printk(KERN_INFO "msmfb: "fmt, ##args); \
} while (0)

#define MSMVIDEO_DEBUG_MSGS 0
#if MSMVIDEO_DEBUG_MSGS
#define VIDEOLOG(fmt,args...) \
	do { printk(KERN_INFO "[%s:%s:%d] "fmt, __FILE__, __func__, __LINE__, \
		    ##args); } \
	while (0)
#else
#define VIDEOLOG(x...) do {} while (0)
#endif


#define FB0_ENABLED	(1)	
#define FB1_ENABLED	(1<<1)
#define VIDEO_ENABLED	(1<<2)

static int msmfb_debug_mask;
module_param_named(msmfb_debug_mask, msmfb_debug_mask, int,
		   S_IRUGO | S_IWUSR | S_IWGRP);

struct msmfb_info {
	struct fb_info *fb_info;
	struct msmfb_context_info *fb_context_info;
	unsigned yoffset;
};

/****************************************
	Global information
****************************************/
struct msmfb_context_info {

	struct fb_info *fb[FB_MAX];
	int fb_num;

	//Internal buffer info
	unsigned long ibuf_start; 	//phsyical addr
	unsigned long ibuf_len;
	char __iomem *ibuf_base;	//virtual addr
	int ibuf_offset; 		//current internal buffer

	struct task_struct *task;
	struct mddi_panel_info *panel_info;
	int active_fbs;

	unsigned update_frame;
	unsigned frame_requested;
	unsigned frame_done;
	int sleeping;

#ifdef CONFIG_ANDROID_POWER
	android_early_suspend_t early_suspend;
	android_early_suspend_t slightly_earlier_suspend;
	android_suspend_lock_t idle_lock;
#endif
	spinlock_t update_lock;
	wait_queue_head_t frame_wq;
	struct work_struct resume_work;
	struct hrtimer fake_vsync;

	unsigned int autoupdate;
	wait_queue_head_t update_wq;
	struct mutex autoupdate_lock;
	struct mutex videoupdate_lock;
	struct mutex graphicsmerge_lock;
	
	unsigned int frameupdate;
	wait_queue_head_t wq;

	struct mdp_blit_int_req last_video_req;
	uint32_t last_addr; 
	ktime_t last_update_time;
	ktime_t last_check_time;
	ktime_t last_dma_time;
	struct hrtimer video_idle_check;
	struct work_struct push_video_work;

	int pipeline;
	uint32_t dma2_ibufformat;

	struct msmfb_callback dma_callback;
	struct msmfb_callback vsync_callback;
	struct msmfb_callback displayon_callback;
	struct msmfb_callback displayoff_callback;

	struct {
		int left;
		int top;
		int eright; /* exclusive */
		int ebottom; /* exclusive */
	} update_info;
	char* black;
	bool wait_for_vsync;
};

struct msmfb_context_info *msmfb_context;

#define FB_READY	0
#define FB_UPDATING	1


#define FB_VSYNC_SIGNALED	0
#define FB_VSYNC_UNSIGNALED	1

#define PIPELINE_LOCKED		0
#define PIPELINE_UNLOCKED	1

struct video_info {
	struct msmfb_info *info;
	struct msmfb_callback vsync_callback;
	struct msmfb_callback dma_callback;
	uint32_t addr;
	uint32_t dma2_ibufformat;
	struct msmfb_callback* user_callback;
};

static struct video_info *queued_video_info;

static uint32_t bytes_per_pixel[] = {
	[MDP_RGB_565] = 2,
	[MDP_RGB_888] = 3,
	[MDP_XRGB_8888] = 4,
	[MDP_ARGB_8888] = 4,
	[MDP_RGBA_8888] = 4,
	[MDP_Y_CBCR_H2V1] = 1,
	[MDP_Y_CBCR_H2V2] = 1,
	[MDP_Y_CRCB_H2V1] = 1,
	[MDP_Y_CRCB_H2V2] = 1,
	[MDP_YCRYCB_H2V1] = 2
};


static int msmfb_force_overlay(void);
static void msmfb_force_update(void);
static int merge_fb0_fb1(void);


static int msmfb_open(struct fb_info *info, int user)
{
	return 0;
}

static int msmfb_release(struct fb_info *info, int user)
{
	return 0;
}

static void msmfb_start_video_dma(struct video_info *vinfo)
{
	uint32_t stride;
	unsigned long irq_flags;

	VIDEOLOG("msm_fb_start_video_dma: address=0x%x\n", vinfo->addr);

	if (msmfb_context->pipeline == PIPELINE_GRAPHICS) {
		VIDEOLOG("msm_fb_start_video_dma:Already switched to graphics mode.\n");
		return;
	}

	spin_lock_irqsave(&msmfb_context->update_lock, irq_flags);
	msmfb_context->frame_requested++;
	spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);

	switch(vinfo->dma2_ibufformat)
	{
		case DMA_IBUF_FORMAT_RGB888:
			stride = 0x3C0;
			break;
		case DMA_IBUF_FORMAT_ARGB8888:
			stride = 0x500;
			break;
		default:
			stride = 0x3C0;
			break;
	}

	mdp_dma_to_mddi(vinfo->addr, stride, 320, 400, 0, 0, &vinfo->dma_callback,
		vinfo->dma2_ibufformat, false);


}

#if PRINT_VFPS
static ktime_t s;
static ktime_t f;
static uint64_t fcount= 0;
static uint64_t d;
#endif
static void msmfb_video_handle_dma_interrupt(struct msmfb_callback *callback)
{
	unsigned long irq_flags;
	struct video_info *vinfo = container_of(callback, struct video_info,
                                                dma_callback);
#if PRINT_VFPS

        f = ktime_get();
        d += ktime_to_ns(ktime_sub(f, s));
        fcount++;
        if (fcount == 100) {
 		do_div(d, 1000000*100);
		printk("avg dma time : %llu ms\n", d);
		fcount = 0;
		d = 0;
	}
#endif

	VIDEOLOG("msmfb_video_handle_dma_interrupt+++\n");

	if(vinfo->user_callback)
		vinfo->user_callback->func(vinfo->user_callback);

	spin_lock_irqsave(&msmfb_context->update_lock, irq_flags);
	msmfb_context->frame_done = msmfb_context->frame_requested;
	spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
	wake_up_all(&msmfb_context->frame_wq);

	msmfb_context->frameupdate = FB_READY;
	wake_up_all(&msmfb_context->wq);

	kfree(vinfo);
}

#if PRINT_DMA_TIMING
ktime_t vsync_irq;
ktime_t dma_complete;
static int count;
static uint64_t sum;
static uint64_t sample;
#endif

static void msmfb_handle_dma_interrupt(struct msmfb_callback *callback)
{
	unsigned long irq_flags;

#if PRINT_FPS
	int64_t dt;
	ktime_t now;
	static int64_t frame_count;
	static ktime_t last_sec;
#endif

#if PRINT_DMA_TIMING
 	dma_complete = ktime_get();
        sample = ktime_to_ns(ktime_sub(dma_complete, vsync_irq));
	sum += sample;
	if(sample > 15000000)
                printk(KERN_ERR"%s: vsync to dma done exceeds 15ms: %llu ns\n", __func__,sample);
        if(count > 100) {
                do_div(sum, 100);
                printk(KERN_ERR"%s: vsync to dma done: %llu ns\n", __func__,sum);
                sum = 0;
		count = 0;
        }
	count++;
#endif

	spin_lock_irqsave(&msmfb_context->update_lock, irq_flags);
	msmfb_context->frame_done = msmfb_context->frame_requested;
	if (msmfb_context->sleeping == UPDATING && msmfb_context->frame_done == msmfb_context->update_frame) {
		DLOG(SUSPEND_RESUME, "full update completed\n");
		schedule_work(&msmfb_context->resume_work);
	}
#if PRINT_FPS
	now = ktime_get();
	dt = ktime_to_ns(ktime_sub(now, last_sec));
	frame_count++;
	if (dt > NSEC_PER_SEC) {
		int64_t fps = frame_count * NSEC_PER_SEC * 100;
		frame_count = 0;
		last_sec = ktime_get();
		do_div(fps, dt);
		DLOG(FPS, "fps * 100: %llu\n", fps);
	}
#endif
	spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
	wake_up_all(&msmfb_context->frame_wq);

	msmfb_context->frameupdate = FB_READY;
	wake_up_all(&msmfb_context->wq);
}

static int msmfb_start_dma(struct msmfb_info *par)
{
	uint32_t x, y, w, h;
	unsigned addr;
	unsigned long irq_flags;
	uint32_t yoffset;
	struct mddi_panel_info *pi = msmfb_context->panel_info;

	spin_lock_irqsave(&msmfb_context->update_lock, irq_flags);
	if (msmfb_context->frame_done == msmfb_context->frame_requested) {
		spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
		return -1;
	}

	if (pi->force_full_update && par) {
		/* Force full update of the screen for clients that do not support partial */
		x = 0;
		y = 0;
		w = par->fb_info->var.xres;
		h = par->fb_info->var.yres;
		yoffset = par->yoffset;
	} else {
		// the partial update width and starting x position must be even.
		// if the x position of the update box is odd then we make it
		// even by expanding the box by 1 pixel (subtract 1).  the width is calculated
		// by taking the box width - starting position.  if that result is
		// odd we will add 1 to increase the box size by a pixel.
		x = msmfb_context->update_info.left;
		if((x % 2) != 0) x-=1;
		y = msmfb_context->update_info.top;
		w = msmfb_context->update_info.eright - x;
		if((w % 2) != 0) w+=1;
		h = msmfb_context->update_info.ebottom - y;
	}
	
	msmfb_context->update_info.left = pi->width + 1;
	msmfb_context->update_info.top = pi->height + 1;
	msmfb_context->update_info.eright = 0;
	msmfb_context->update_info.ebottom = 0;
	if (unlikely(w > pi->width || h > pi->height || w == 0 || h == 0)) {
		printk(KERN_INFO "invalid update: %d %d %d %d\n", x, y, w, h);
		msmfb_context->frame_done = msmfb_context->frame_requested;
		goto error;
	}
	spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
	
	msmfb_context->last_dma_time = ktime_get();

	/*
 	 * DMA addr depends on if the active state of fb0, fb1 and video 
 	 * If blending is need, the addr would be the internal buffer, which 
 	 * holds the blending results.
 	 * If no blending is needed, the addr would be the fb itself
 	 */
	
	if(!par) {
	
		//BLENDED! internal buffer at the end of fb0 (result of fb0+fb1)
	
		addr = (RGB_BUFFER_SIZE*msmfb_context->ibuf_offset)+ 	
			+  (320*3*y) + (x * (24 >> 3)); 
		mdp_dma_to_mddi(addr + msmfb_context->ibuf_start,0x3C0,
			w, h, x, y, &msmfb_context->dma_callback, DMA_IBUF_FORMAT_RGB888, false);
	}
	else { //NO BLEND! par would be fb0 

		addr = (par->fb_info->fix.line_length * (par->yoffset + y)) + (x * (pi->bits_per_pixel >> 3));
		mdp_dma_to_mddi(addr + par->fb_info->fix.smem_start,
		                par->fb_info->fix.line_length, w, h, x, y, &msmfb_context->dma_callback,
				DMA_IBUF_FORMAT_ARGB8888, false);
	}
	
	MDP_LOG_INFO("addr:0x%x x:%d, y:%d, w:%d, h:%d\n", addr + par->fb_info->fix.smem_start, x, y, w, h);

	return 0;

error:
	spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
	/* some clients clear their vsync interrupt
	 * when the link activates */
	mddi_activate_link(pi->mddi);
	return 0;
}

/* Called from esync interrupt handler, must not sleep */
static void msmfb_video_handle_vsync_interrupt(struct msmfb_callback *callback)
{
	struct video_info *info  = container_of(callback, struct video_info,
					       vsync_callback);

	msmfb_start_video_dma(info);
}


/* Called from esync interrupt handler, must not sleep */
static void msmfb_handle_vsync_interrupt(struct msmfb_callback *callback)
{

#if PRINT_DMA_TIMING
	vsync_irq = ktime_get();
#endif
				
#ifdef CONFIG_ANDROID_POWER
	android_unlock_suspend(&par->idle_lock);
#endif
 	if(msmfb_context->active_fbs == (FB0_ENABLED|FB1_ENABLED)) {
		msmfb_start_dma(NULL);
        }
        else if(msmfb_context->active_fbs == FB0_ENABLED) {
		msmfb_start_dma( msmfb_context->fb[0]->par);
        }

	return;

}

static void msmfb_push_frame(struct work_struct *work)
{
	msmfb_force_update();

	return;
}

static enum hrtimer_restart msmfb_video_idle_check(struct hrtimer *timer)
{
	/* Push frame with the last update time remains the same */
 	if(ktime_equal(msmfb_context->last_check_time,msmfb_context->last_update_time))
	{
		VIDEOLOG("msmfb_video_idle_check push video frame\n");
		schedule_work( &msmfb_context->push_video_work );
	}
		
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart msmfb_fake_vsync(struct hrtimer *timer)
{
	struct msmfb_info *par =  msmfb_context->fb[0]->par; 
				

	if ((msmfb_context->pipeline == PIPELINE_VIDEO) &&
		(NULL != queued_video_info)) {
		msmfb_start_video_dma(queued_video_info);
		queued_video_info = NULL;
	}
	else
		msmfb_start_dma(par);
		
	return HRTIMER_NORESTART;
}

static void msmfb_pan_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom, uint32_t yoffset,
			 int pan_display)
{
	struct msmfb_info *par = info->par;
	struct mddi_panel_info *pi = par->fb_context_info->panel_info;
	unsigned long irq_flags;
	int sleeping;
	ktime_t time; 
	long rc;
#if PRINT_FPS
	ktime_t t1, t2;
	static uint64_t pans;
	static uint64_t dt;
	t1 = ktime_get();
#endif

	if(msmfb_context->pipeline == PIPELINE_VIDEO) {
		uint64_t video_idle_time; 
		VIDEOLOG("In Video Mode: offset=%d\n", yoffset);
		mutex_lock(&msmfb_context->videoupdate_lock);
		par->yoffset = yoffset;
		msmfb_context->frameupdate = FB_UPDATING;
		mutex_unlock(&msmfb_context->videoupdate_lock);

		time = ktime_get();
		video_idle_time = ktime_to_ns(ktime_sub(time, msmfb_context->last_update_time));
		do_div(video_idle_time, 1000000);

		if(video_idle_time > VIDEO_IDLE_THRESHOLD_MS && 
			msmfb_context->last_addr > 0 ) {
			/*
		 	*Will force an update in the video pipeline based on the last
		 	*video frame if the video pipeline is detected to be stalled.
		 	*/
			VIDEOLOG("msmfb_pan_update push video frame\n");
			msmfb_force_overlay();
		}
		else {
			/*
 			 *Will check to make sure the video pipeline did merge and push 
                         *the frame out in expected time. If not, the frame will be forced out
			 */
 			if (!hrtimer_active(&msmfb_context->video_idle_check)) {
				msmfb_context->last_check_time = msmfb_context->last_update_time;
                        	hrtimer_start(&msmfb_context->video_idle_check,
                                      ktime_set(0, NSEC_PER_SEC/30),
                                      HRTIMER_MODE_REL);
            		}
		}

		
		//  Block until DMA finishes
		rc = wait_event_interruptible_timeout(msmfb_context->wq, msmfb_context->frameupdate == FB_READY,
                	msecs_to_jiffies(100));

		if(rc == 0) 
			printk("msmfb video mode: timed out waiting for dma completion\n");

		return;
	}

	MDP_LOG_ENTER();
	MDP_LOG_INFO("left %d, top %d, eright %d, ebottom %d\n",
	       		left, top, eright, ebottom);
	MDP_LOG_INFO("f_req %d, f_done %d\n",
		     msmfb_context->frame_requested, msmfb_context->frame_done);
restart:

	spin_lock_irqsave(&msmfb_context->update_lock, irq_flags);

	/* if we are sleeping, on a pan_display wait 10ms (to throttle back
	 * drawing otherwise return */
	if (msmfb_context->sleeping == SLEEPING) {
		DLOG(SUSPEND_RESUME, "drawing while asleep\n");
		spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
		if (pan_display)
			wait_event_interruptible_timeout(msmfb_context->frame_wq,
				msmfb_context->sleeping == AWAKE, HZ/10);
		return;
	}


	sleeping = msmfb_context->sleeping;
	/* on a full update, if the last frame has not completed, wait for it */
	if (pan_display && msmfb_context->frame_requested != msmfb_context->frame_done) {
		spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
		if (wait_event_interruptible_timeout(msmfb_context->frame_wq,
		    msmfb_context->frame_done == msmfb_context->frame_requested, HZ) <= 0) {

 			uint64_t dma_time;
			ktime_t time;

			printk(KERN_WARNING "msmfb_pan_display timeout waiting "
					    "for frame start, %d %d\n",
					    msmfb_context->frame_requested,
					    msmfb_context->frame_done);

			time = ktime_get();
			dma_time = ktime_to_ns(ktime_sub(time, msmfb_context->last_dma_time));
			do_div(dma_time, 1000000);

			if(dma_time < DMA_IDLE_THRESHOLD_MS)
				return;

			printk(KERN_WARNING "msmfb_pan_display: Start time of last DMA was over %dms ago\n", DMA_IDLE_THRESHOLD_MS); 
			msmfb_context->frame_done = msmfb_context->frame_requested;
			mdp_print_isr_status();
		}
		goto restart;
	}

#if PRINT_FPS
	t2 = ktime_get();
	if (pan_display) {
		uint64_t temp = ktime_to_ns(ktime_sub(t2, t1));
		do_div(temp, 1000);
		dt += temp;
		pans++;
		if (pans > 1000) {
			do_div(dt, pans);
			DLOG(FPS, "ave_wait_time: %lld\n", dt);
			dt = 0;
			pans = 0;
		}
	}
#endif

	msmfb_context->frame_requested++;
	/* if necessary, update the y offset, if this is the
	 * first full update on resume, set the sleeping state */
	if (pan_display) {
		
		par->yoffset = yoffset;

		if (left == 0 && top == 0 && eright == info->var.xres &&
		    ebottom == info->var.yres) {
			if (sleeping == WAKING) {
				msmfb_context->update_frame = msmfb_context->frame_requested;
				DLOG(SUSPEND_RESUME, "full update starting\n");
				msmfb_context->sleeping = UPDATING;
			}
		}
	}

	/* set the update request */
	if (left < msmfb_context->update_info.left)
		msmfb_context->update_info.left = left;
	if (top < msmfb_context->update_info.top)
		msmfb_context->update_info.top = top;
	if (eright > msmfb_context->update_info.eright)
		msmfb_context->update_info.eright = eright;
	if (ebottom > msmfb_context->update_info.ebottom)
		msmfb_context->update_info.ebottom = ebottom;

	msmfb_context->frameupdate = FB_UPDATING;
	spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);

	//BLEND fb0 and fb1
	mutex_lock(&msmfb_context->graphicsmerge_lock);
	if(msmfb_context->active_fbs == (FB0_ENABLED|FB1_ENABLED)) {
		merge_fb0_fb1();
	}
	mutex_unlock(&msmfb_context->graphicsmerge_lock);

	//DMA now (no wait for vsync) or later (wait for vsync)
	if(!msmfb_context->wait_for_vsync) {
		if(msmfb_context->active_fbs == (FB0_ENABLED|FB1_ENABLED)) {
			msmfb_start_dma(NULL);
		}
		else if(msmfb_context->active_fbs == FB0_ENABLED) {
			msmfb_start_dma( msmfb_context->fb[0]->par);
		}
	}
	else {
		if (pi->panel_ops->request_vsync && (sleeping == AWAKE)) {
#ifdef CONFIG_ANDROID_POWER
			android_lock_idle_auto_expire(&msmfb_context->idle_lock, HZ/20);
#endif
			pi->panel_ops->request_vsync(pi, &msmfb_context->vsync_callback);
		} else {
			if (!hrtimer_active(&msmfb_context->fake_vsync)) {
				hrtimer_start(&msmfb_context->fake_vsync,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
			}
		}
	}

	//  Block until DMA finishes
	rc = wait_event_interruptible_timeout(msmfb_context->wq, msmfb_context->frameupdate == FB_READY,
		msecs_to_jiffies(100));


	if(rc == 0) 
		printk("msmfb graphics mode: timed out waiting for dma completion\n");

	msmfb_context->wait_for_vsync = false;
}

static void msmfb_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom)
{
	msmfb_pan_update(info, left, top, eright, ebottom, 0, 0);
}

static void msmfb_force_update(void)
{
	struct fb_info *fb = NULL;

	mutex_lock(&msmfb_context->graphicsmerge_lock);
 	if(msmfb_context->active_fbs == (FB0_ENABLED|FB1_ENABLED)) {
                fb =  msmfb_context->fb[1]; 
        }
        else if(msmfb_context->active_fbs == FB0_ENABLED) {
                fb =  msmfb_context->fb[0];
        }
	mutex_unlock(&msmfb_context->graphicsmerge_lock);

	if(msmfb_context->pipeline == PIPELINE_VIDEO) {

		msmfb_force_overlay();

	}else {

		msmfb_update(fb, 0, 0, fb->var.xres, fb->var.yres);
	}

	return;
}

static void msmfb_display_on(struct msmfb_callback *callback)
{
	printk("msmfb_display_on()\n");

	msmfb_context->sleeping = AWAKE;
	msmfb_force_update();
}

static void msmfb_display_off(struct msmfb_callback *callback)
{
	long rc;

	printk("msmfb_display_off()\n");

	msmfb_context->sleeping = SLEEPING;

	rc = wait_event_interruptible_timeout(msmfb_context->wq, msmfb_context->frameupdate == FB_READY,
		msecs_to_jiffies(100));

	if(rc == 0)
		printk("msmfb_display_off: timed out waiting for dma completion\n");
}

#ifdef CONFIG_ANDROID_POWER
static void power_on_panel(struct work_struct *work)
{
	unsigned long irq_flags;

	struct mddi_panel_info *pi = msmfb_context->panel_info;
	DLOG(SUSPEND_RESUME, "turning on panel\n");
	pi->panel_ops->power(pi, 1);
	spin_lock_irqsave(&msmfb_context->update_lock, irq_flags);
	msmfb_context->sleeping = AWAKE;
	spin_unlock_irqrestore(&msmfb_context->update_lock, irq_flags);
}

/* turn off the panel */
static void msmfb_slightly_earlier_suspend(android_early_suspend_t *h)
{
	struct msmfb_info *par = container_of(h, struct msmfb_info,
					      slightly_earlier_suspend);
	struct mddi_panel_info *pi = par->panel_info;
	unsigned int irq_flags;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->sleeping = SLEEPING;
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	wait_event_timeout(par->frame_wq,
			   par->frame_requested == par->frame_done, HZ/10);

	/* blank the screen */
	msmfb_update(par->fb_info, 0, 0, par->fb_info->var.xres,
		     par->fb_info->var.yres);
	mdp_dma_to_mddi(virt_to_phys(par->black), 0,
			par->fb_info->var.xres, par->fb_info->var.yres, 0, 0,
			NULL, par->dma2_ibufformat, true);
	mdp_dma_wait();
	/* turn off the backlight and the panel */
	pi->panel_ops->power(pi, 0);
}

/* userspace has stopped drawing */
static void msmfb_early_suspend(android_early_suspend_t *h)
{
}

/* turn on the panel */
static void msmfb_early_resume(android_early_suspend_t *h)
{
	struct msmfb_info *par = container_of(h, struct msmfb_info,
				 early_suspend);
	unsigned int irq_flags;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->frame_requested = par->frame_done = par->update_frame = 0;
	par->sleeping = WAKING;
	DLOG(SUSPEND_RESUME, "ready, waiting for full update\n");
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
}

/* userspace has started drawing */
static void msmfb_slightly_later_resume(android_early_suspend_t *h)
{
}
#endif

static int msmfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var->xres != info->var.xres) return -EINVAL;
	if (var->yres != info->var.yres) return -EINVAL;
	if (var->xres_virtual != info->var.xres_virtual) return -EINVAL;
	if (var->yres_virtual != info->var.yres_virtual) return -EINVAL;
	if (var->xoffset != info->var.xoffset) return -EINVAL;
	if (var->bits_per_pixel != info->var.bits_per_pixel) return -EINVAL;
	if (var->grayscale != info->var.grayscale) return -EINVAL;
	return 0;
}

int msmfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/* "UPDT" */
	if (var->reserved[0] == 0x54445055) {
#if 0
		printk(KERN_INFO "pan rect %d %d %d %d\n",
		       var->reserved[1] & 0xffff,
		       var->reserved[1] >> 16, var->reserved[2] & 0xffff,
		       var->reserved[2] >> 16);
#endif
		msmfb_pan_update(info, var->reserved[1] & 0xffff,
				 var->reserved[1] >> 16,
				 var->reserved[2] & 0xffff,
				 var->reserved[2] >> 16, var->yoffset, 1);
	} else {
		msmfb_pan_update(info, 0, 0, info->var.xres, info->var.yres,
		var->yoffset, 1);
	}
	return 0;
}

static void msmfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	cfb_fillrect(p, rect);
	msmfb_update(p, rect->dx, rect->dy, rect->dx + rect->width,
		     rect->dy + rect->height);
}

static void msmfb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	cfb_copyarea(p, area);
	msmfb_update(p, area->dx, area->dy, area->dx + area->width,
		     area->dy + area->height);
}

static void msmfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	msmfb_update(p, image->dx, image->dy, image->dx + image->width,
		     image->dy + image->height);
}

/*
 * Normally returns file pointer of the given fd. However:
 * Returns NULL if the file pointer of the given fd is not found
 * Returns -1 if the fd is the frame buffer's fd
 *
 */
static  struct file* msmfb_get_filp(int fd)
{
	int put_needed;
	struct file *f = NULL;

	if(!fd)
		return f;	

	f = fget_light(fd, &put_needed);
	
	if(!f)
	{
              printk(KERN_INFO "msmfb_get_filp failed, cannot retrieve filp from fd: %d\n", fd);
		
	}

	fput_light(f, put_needed);

 	if(MAJOR(f->f_dentry->d_inode->i_rdev) == FB_MAJOR)
		f = (struct file*)-1;



	return f;
}

/*
 * Blending fb0 (luna) and fb1 (gaming)
 */
static int merge_fb0_fb1(void)
{
	struct mdp_blit_int_req ireq;
	struct fb_info *fb0_info, *fb1_info;
	struct msmfb_info *fb0_par, *fb1_par;
	int ret = 0;

	fb0_info =  msmfb_context->fb[0];
	fb1_info =  msmfb_context->fb[1];

	if(!fb0_info || !fb1_info) {
		printk(KERN_ERR"msmfb: fb0 or fb1 not registered properly\n");
		return -1;
	}

	fb0_par = fb0_info->par;
	fb1_par = fb1_info->par;

	//Rotate through all 3 internal buffers
	msmfb_context->ibuf_offset = 
		(msmfb_context->ibuf_offset==2)? 0 : msmfb_context->ibuf_offset+1;

	//fb0 luna
	ireq.src.width = 320; 
	ireq.src.height = 400; 
	ireq.src.format = MDP_ARGB_8888; 
	ireq.src.offset = fb0_par->yoffset*bytes_per_pixel[ireq.src.format]*ireq.src.width; 
	ireq.src.filp = (struct file*)0;
	ireq.src.smem_start = fb0_info->fix.smem_start;
	ireq.src.smem_len = fb0_info->fix.smem_len;
			

	//fb1 gaming 
	ireq.bg.width  = 320; 
	ireq.bg.height = 400; 
	ireq.bg.format = MDP_ARGB_8888; 
	ireq.bg.offset = fb1_par->yoffset*bytes_per_pixel[ireq.bg.format]*ireq.src.width;
	ireq.bg.filp = (struct file*)0;
	ireq.bg.smem_start = fb1_info->fix.smem_start;
	ireq.bg.smem_len = fb1_info->fix.smem_len;

	//internal buffer at the end of fb0 for now
	ireq.dst.width  = 320; 
	ireq.dst.height = 400; 
	ireq.dst.format = MDP_RGB_888; 
	ireq.dst.offset = 
		msmfb_context->ibuf_offset*ireq.dst.width*ireq.dst.height*bytes_per_pixel[ireq.dst.format];
	ireq.dst.filp = (struct file*)0; 
	ireq.dst.smem_start = msmfb_context->ibuf_start; 
	ireq.dst.smem_len = msmfb_context->ibuf_len; 

	ireq.transp_mask =  0xFFFFFFFF;
	ireq.flags = 0; 
	ireq.alpha  = 0xFF; 

    	ireq.src_rect.x = 0;
    	ireq.src_rect.y = 0;
    	ireq.src_rect.w = 320;
    	ireq.src_rect.h = 400;

    	ireq.dst_rect.x = 0;
    	ireq.dst_rect.y = 0;
    	ireq.dst_rect.w = 320;
    	ireq.dst_rect.h = 400;

    	ireq.bg_rect.x = 0;
    	ireq.bg_rect.y = 0;
    	ireq.bg_rect.w = 320;
    	ireq.bg_rect.h = 400;

	ret = mdp_blit(&ireq);

	if (ret)
		printk("Bad blit!\n");

	return ret;
}
static int msmfb_blit(struct fb_info *info, void __user *p)
{
	struct mdp_blit_req_list ereq_list;
 	struct mdp_blit_req ereq; 
	struct mdp_blit_int_req ireq;
	int i;
	int ret;

	if (copy_from_user(&ereq_list, p, sizeof(ereq_list)))
		return -EFAULT;

	for (i = 0; i < ereq_list.count; i++) 
	{
		struct mdp_blit_req_list *list =
			(struct mdp_blit_req_list *)p;

		if (copy_from_user(&ereq, &list->req[i], sizeof(ereq)))
			return -EFAULT;

		ireq.src.width = ereq.src.width;
		ireq.src.height = ereq.src.height;
		ireq.src.format = ereq.src.format;
		ireq.src.offset = ereq.src.offset;
		ireq.src.filp = msmfb_get_filp(ereq.src.memory_id);
		ireq.src.smem_start = 0;
		ireq.src.smem_len = 0;

		ireq.bg.width  = ereq.dst.width;
		ireq.bg.height = ereq.dst.height;
		ireq.bg.format = ereq.dst.format;
		ireq.bg.offset = ereq.dst.offset;
		ireq.bg.filp = msmfb_get_filp(ereq.bg.memory_id);
		ireq.bg.smem_start = 0;
		ireq.bg.smem_len = 0;

		ireq.dst.width  = ereq.dst.width;
		ireq.dst.height = ereq.dst.height;
		ireq.dst.format = ereq.dst.format;
		ireq.dst.offset = ereq.dst.offset;
		ireq.dst.filp = msmfb_get_filp(ereq.dst.memory_id);
		ireq.dst.smem_start = 0;
		ireq.dst.smem_len = 0;

		ireq.transp_mask = ereq.transp_mask;
		ireq.flags = ereq.flags;
		ireq.alpha  = ereq.alpha;

		ireq.dst_rect = ereq.dst_rect;
		ireq.src_rect = ereq.src_rect;

		ret = mdp_blit(&ireq);
		if (ret)
			return ret;
	}

	return 0;
}


int msmfb_switch_pipeline(int pipeline)
{
	int ret = 0;
	long rc;

	rc = wait_event_interruptible_timeout(msmfb_context->wq,
			msmfb_context->frameupdate == FB_READY, msecs_to_jiffies(100));

	mutex_lock(&msmfb_context->videoupdate_lock);

	if(rc == 0) {
		printk("%s: timeout waiting for pipeline to free up\n", __func__);
	}

	if(msmfb_context->pipeline == pipeline) {
		VIDEOLOG("Already setup for %s\n", msmfb_context->pipeline == 1 ? "GRAPHICS" : "VIDEO");
	}
	else if(pipeline == PIPELINE_GRAPHICS) { 
		VIDEOLOG("Switching FB to Graphics pipeline\n");
		msmfb_context->pipeline = pipeline;

		//Clear internal buffers
        	memset(msmfb_context->ibuf_base, 0, 3*RGB_BUFFER_SIZE);
	}
	else if(pipeline == PIPELINE_VIDEO) {
		VIDEOLOG("Switching FB to Video pipeline\n");
		msmfb_context->pipeline = pipeline;

		//Clear internal buffers
        	memset(msmfb_context->ibuf_base, 0, 3*RGB_BUFFER_SIZE);
	}
	else {
		printk("msmfb_switch_pipeline: %d is an invalid pipeline\n", pipeline);
		ret = -EINVAL;
	}

	mutex_unlock(&msmfb_context->videoupdate_lock);
	
	return ret;
}
EXPORT_SYMBOL(msmfb_switch_pipeline);


void queue_video_frame(uint32_t address, uint32_t format, struct msmfb_callback* user_callback)
{
	struct video_info *frame = kzalloc(sizeof(struct video_info), GFP_KERNEL);

	VIDEOLOG("queue_video_frame+++\n");

	frame->vsync_callback.func = msmfb_video_handle_vsync_interrupt;
	frame->dma_callback.func = msmfb_video_handle_dma_interrupt;
	frame->addr = address;
	frame->dma2_ibufformat = format;
	frame->user_callback = user_callback;

	queued_video_info = frame;

	/* if the panel is all the way on wait for vsync, otherwise sleep
	 * for 16 ms (long enough for the dma to panel) and then begin dma */
	if (msmfb_context->panel_info->panel_ops->request_vsync) {
		msmfb_context->panel_info->panel_ops->request_vsync(msmfb_context->panel_info, &frame->vsync_callback);
	} else {
		if (!hrtimer_active(&msmfb_context->fake_vsync)) {
			hrtimer_start(&msmfb_context->fake_vsync,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
		}
	}
}

void msmfb_get_last_frame(unsigned long *addr, unsigned long *len)
{
	*addr = msmfb_context->last_addr;
	*len = RGB_BUFFER_SIZE;

	return;
}
EXPORT_SYMBOL(msmfb_get_last_frame);

/*
 * Internal function to force the replay of the last
 * video frame with the new fb0
 */
int msmfb_force_overlay()
{
	struct mdp_blit_int_req *req = &msmfb_context->last_video_req;
        int ret = 0;

	ret = msmfb_overlay(req,NULL,false);

	return ret;
}

/*
 * Function used by the video path to play video.
 * It can be driven either by v4l2 with new_update set to true;
 * or it can be driven internally (when the video is stalled)
 * with new_update set to false. Depending on the path, the
 * strategy to use the internal buffers (ibuf<x>) for blending is
 * different.
 *
 * 1) V4l2 update (new_update set) and fb1(gaming) enabled
 *
 * Stage 1: Rotation
 * vbuf -> ibuf0
 * Stage 2: Blending with Gaming layer 
 * ibuf0 + fb1 -> ibuf1
 * Stage 3: Blending with Luna 
 * ibuf1 + fb0->ibuf2
 *
 * 2) V4l2 update (new_update set) and fb1(gaming) disabled
 *
 * Stage 1: Rotation
 * vbuf -> ibuf0
 * Stage 2: Blending with Luna 
 * ibuf0 + fb0->ibuf1/ibuf2 (rotate between the two buffers)
 *
 * 3) Internal update (new_update unset) and fb1(gaming) enabled
 *
 * Stage 1: Blending with Gaming layer 
 * ibuf0(last good rotated frame) + fb1 -> ibuf1
 * Stage 2: Blending with Luna 
 * ibuf1 + fb0->ibuf2
 *
 * 4) Internal update (new_update unset) and fb1(gaming) disabled
 *
 * Stage 1: Blending with Luna 
 * ibuf0 + fb0->ibuf1/ibuf2 (rotate between the two buffers)
 *
 */ 
int msmfb_overlay(struct mdp_blit_int_req *req,  struct msmfb_callback* user_callback, bool new_update)
{
	int ret = 0;
	struct fb_info *fb0_info;
	struct fb_info *fb1_info;
	struct msmfb_info *fb0_par;
	struct msmfb_info *fb1_par;

	fb0_info =  msmfb_context->fb[0];
	fb1_info =  msmfb_context->fb[1];
	fb0_par = fb0_info->par;
	fb1_par = fb1_info->par;

#if PRINT_VFPS
	int64_t dt;
	ktime_t now;
	static int64_t frame_count= 0;
	static ktime_t last_sec;

        s = ktime_get();
        now = ktime_get();
        dt = ktime_to_ns(ktime_sub(now, last_sec));
        frame_count++;
        if (dt > NSEC_PER_SEC ) {
                int64_t fps = frame_count * NSEC_PER_SEC ;
                frame_count = 0;
                last_sec = ktime_get();
                do_div(fps, dt);
                printk("fps: %llu\n", fps);
        }
#endif

	mutex_lock(&msmfb_context->videoupdate_lock);

	msmfb_context->frameupdate = FB_UPDATING;

	if(msmfb_context->pipeline == PIPELINE_GRAPHICS) {
		printk("%s: switched to graphics, bail out\n", __func__);
		goto done;
	}

	VIDEOLOG("msmfb_overlay+++\n");
	
	//stage 1 - rotation (Only needed for v4l2 path)
	//vbuf -> ibuf0
	if(new_update) {
		req->dst.offset = 0; 
		req->dst.smem_start =  msmfb_context->ibuf_start;
		req->dst.smem_len = msmfb_context->ibuf_len;
		req->dst.filp = (struct file*)0;
		ret = mdp_blit(req);

		msmfb_context->last_video_req = *req;
		msmfb_context->last_update_time = ktime_get();
		msmfb_context->last_addr = 
			(unsigned long)msmfb_context->ibuf_base; 
	}

	//Optional stage 2 if fb1 is enabled 
	//ibuf0 + fb1 -> ibuf1; 
  	memset (req, 0, sizeof(struct mdp_blit_int_req));

	//Take from src rotation buffer: ibuf 0
        req->bg_rect.w    = req->bg.width    = 320;
        req->bg_rect.h    = req->bg.height   = 400;
        req->bg.format    = MDP_RGB_888;
	req->bg.offset = 0;
	req->bg.smem_start =  msmfb_context->ibuf_start;
	req->bg.smem_len = msmfb_context->ibuf_len;
	req->bg.filp = (struct file*)0;

	mutex_lock(&msmfb_context->graphicsmerge_lock);
	if(msmfb_context->active_fbs == (FB1_ENABLED|FB0_ENABLED)) {
		
        	req->src_rect.w     = req->src.width  = 320;
        	req->src_rect.h     = req->src.height = 400; 
        	req->src.format     = MDP_ARGB_8888;
		req->src.offset = req->src.width * bytes_per_pixel[req->src.format] * fb1_par->yoffset;
		req->src.smem_start = fb1_info->fix.smem_start;
		req->src.smem_len = fb1_info->fix.smem_len;
		req->src.filp = (struct file*)0;
		
		//Dump stuff to output buffer: buf 1 
        	req->dst_rect.w    = req->dst.width = 320;
        	req->dst_rect.h    = req->dst.height = 400;
        	req->dst.format    = MDP_RGB_888;
		req->dst.offset = RGB_BUFFER_SIZE ; //ibuf 1 
		req->dst.smem_start =  msmfb_context->ibuf_start;
		req->dst.smem_len = msmfb_context->ibuf_len;
		req->dst.filp = (struct file*)0;

 		req->alpha = MDP_ALPHA_NOP;
        	req->transp_mask = MDP_TRANSP_NOP;
        	req->flags = MDP_ROT_NOP;

		ret = mdp_blit(req);
	
		//Take from ibuf 1
        	req->bg_rect.w    = req->src.width    = 320;
        	req->bg_rect.h    = req->src.height   = 400;
        	req->bg.format    = MDP_RGB_888;
		req->bg.offset = RGB_BUFFER_SIZE;
		req->bg.smem_start =  msmfb_context->ibuf_start;
		req->bg.smem_len = msmfb_context->ibuf_len;
		req->bg.filp = (struct file*)0;

		msmfb_context->ibuf_offset = 2;
	}
	else {
	
		//Rotate to use ibuf1 or ibuf2
		msmfb_context->ibuf_offset = 
			(msmfb_context->ibuf_offset==2)? 1 : 2;
	}
	mutex_unlock(&msmfb_context->graphicsmerge_lock);

	//stage 3
	//if fb1 on: 	ibuf1 + fb0->ibuf2
	//if fb1 off: 	ibuf0 + fb0 -> ibuf1/ibuf2
        req->src_rect.w     = req->src.width  = 320;
        req->src_rect.h     = req->src.height = 400; 
        req->src.format     = MDP_ARGB_8888;
	req->src.offset = fb0_par->yoffset*bytes_per_pixel[req->src.format]*320;
	req->src.smem_start = fb0_info->fix.smem_start;
	req->src.smem_len = fb0_info->fix.smem_len;
	req->src.filp = (struct file*)0;
		
        req->dst_rect.w    = req->dst.width = 320;
        req->dst_rect.h    = req->dst.height = 400;
        req->dst.format    = MDP_RGB_888;
	req->dst.offset = RGB_BUFFER_SIZE * msmfb_context->ibuf_offset;
	req->dst.smem_start =  msmfb_context->ibuf_start;
	req->dst.smem_len = msmfb_context->ibuf_len;
	req->dst.filp = (struct file*)0;

 	req->alpha = MDP_ALPHA_NOP;
        req->transp_mask = MDP_TRANSP_NOP;
        req->flags = MDP_ROT_NOP;

	ret = mdp_blit(req);
	
	//Queue up frame for DMA
	queue_video_frame(msmfb_context->ibuf_start + RGB_BUFFER_SIZE * msmfb_context->ibuf_offset,
		DMA_IBUF_FORMAT_RGB888, user_callback);
done:
	mutex_unlock(&msmfb_context->videoupdate_lock);
	
	return ret;
}
EXPORT_SYMBOL(msmfb_overlay);

static int msmfb_constant_update(void *data)
{
	struct fb_info *fb = (struct fb_info *)data;

	VIDEOLOG("msmfb_constant_update+++\n");
	
	while (!kthread_should_stop()) 
	{

		wait_event_interruptible(msmfb_context->update_wq, msmfb_context->autoupdate == 1);

		// initiate DMA transfer of entire frame buffer
		msmfb_update(fb, 0, 0, fb->var.xres, fb->var.yres);

		// sleep for 33ms
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(33));
	}

	printk("msmfb_constant_update---\n");

	return 0;
}

DECLARE_MUTEX(mdp_ppp_sem);

static int msmfb_ioctl(struct fb_info *p, unsigned int cmd, unsigned long arg)
{
	int userarg=0;
	int ret;
#if PRINT_BLIT_TIME
	ktime_t t1, t2;
#endif

	switch (cmd)
	{
		case MSMFB_AUTO_UPDATE:
			userarg = (int)arg;
			if(userarg == 1)
			{
				mutex_lock(&msmfb_context->autoupdate_lock);
				msmfb_context->autoupdate = 1;
				mutex_unlock(&msmfb_context->autoupdate_lock);
				wake_up(&msmfb_context->update_wq);
			}
			else	
			{
				mutex_lock(&msmfb_context->autoupdate_lock);
				msmfb_context->autoupdate = 0;
				mutex_unlock(&msmfb_context->autoupdate_lock);
			}
			break;
		case FBIO_WAITFORVSYNC:
		{
			if(msmfb_context->sleeping == SLEEPING) {
				DLOG(SUSPEND_RESUME, "SLEEPING: don't wait for vsync!\n");
				break;
			}

			msmfb_context->wait_for_vsync = true;

			break;
		}
		case MSMFB_GRP_DISP:
			mdp_set_grp_disp(arg);
			break;
		case MSMFB_BLIT:
		{
#if PRINT_BLIT_TIME
			t1 = ktime_get();
#endif
			down(&mdp_ppp_sem);
			ret = msmfb_blit(p, (void __user *)arg);
			up(&mdp_ppp_sem);
			if (ret)
				return ret;
#if PRINT_BLIT_TIME
			t2 = ktime_get();
			DLOG(BLIT_TIME, "total %lld\n",
			       ktime_to_ns(t2) - ktime_to_ns(t1));
#endif
			break;
		}
		default:
			printk(KERN_INFO "msmfb unknown ioctl: %d\n", cmd);
			return -EINVAL;
	}
	return 0;
}

int msmfb_blank(int blank, struct fb_info *info)
{
	long rc;
	
	//Dont care about fb0 (Luna)
	if(info->node == 0)
		return 0;

        rc = wait_event_interruptible_timeout(msmfb_context->wq,
                        msmfb_context->frameupdate == FB_READY, msecs_to_jiffies(100));

	mutex_lock(&msmfb_context->graphicsmerge_lock);

	if(info->node == 1) {
		//Policy is to blend fb0 and fb1 at all times if fb1 is enabled
		if(blank == FB_BLANK_UNBLANK)
			msmfb_context->active_fbs = (FB0_ENABLED|FB1_ENABLED);
		else if(blank == FB_BLANK_NORMAL)
			msmfb_context->active_fbs = (FB0_ENABLED);
	}

	mutex_unlock(&msmfb_context->graphicsmerge_lock);

	printk("msmfb: active framebuffer status: %d\n",  msmfb_context->active_fbs);

	return 0;
}

static struct fb_ops msmfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = msmfb_open,
	.fb_release = msmfb_release,
	.fb_check_var = msmfb_check_var,
	.fb_pan_display = msmfb_pan_display,
	.fb_fillrect = msmfb_fillrect,
	.fb_copyarea = msmfb_copyarea,
	.fb_imageblit = msmfb_imageblit,
	.fb_ioctl = msmfb_ioctl,
	.fb_blank = msmfb_blank,
};

static unsigned PP[16];


#if MSMFB_DEBUG
static ssize_t debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t debug_read(struct file *file, char __user *buf, size_t count,
                          loff_t *ppos)
{
	const int debug_bufmax = 4096;
	static char buffer[4096];
	int n = 0;
	struct msmfb_info *par = (struct msmfb_info *)file->private_data;
	unsigned long irq_flags;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	n = scnprintf(buffer, debug_bufmax, "yoffset %d\n", par->yoffset);
	n += scnprintf(buffer + n, debug_bufmax, "frame_requested %d\n", par->frame_requested);
	n += scnprintf(buffer + n, debug_bufmax, "frame_done %d\n", par->frame_done);
	n += scnprintf(buffer + n, debug_bufmax, "sleeping %d\n", par->sleeping);
	n += scnprintf(buffer + n, debug_bufmax, "update_frame %d\n", par->update_frame);
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	n++;
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations debug_fops = {
        .read = debug_read,
        .open = debug_open,
};
#endif

#ifdef CONFIG_PM 

static int msmfb_suspend(struct platform_device* pdev, pm_message_t state)
{
	return 0;
}

static int msmfb_resume(struct platform_device* pdev)
{
	msmfb_force_update();
	
	return 0;
}
#else
#define msmfb_suspend  NULL
#define msmfb_resume   NULL
#endif  /* CONFIG_PM */


static int init_fb_info(
	struct fb_info* info, 
	const char* name, 
	unsigned char* fbram,
	struct mddi_panel_info* pi,
	unsigned long fb_base,
	unsigned long fb_size)
{
	struct msmfb_info *par;
	int ret,i;

	par = info->par;
	par->fb_info = info;


	info->screen_base = fbram;
	strncpy(info->fix.id, name, 16);

	info->fix.smem_start = fb_base;
	info->fix.smem_len = fb_size;

	info->fix.ypanstep = 1;
	info->fbops = &msmfb_ops;
	info->flags = FBINFO_DEFAULT;

	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = pi->width * (pi->bits_per_pixel >> 3); 	// width * bytes_per_pixel

	info->var.xres = pi->width;
	info->var.yres = pi->height;
	info->var.xres_virtual = pi->width;
	
	// yres_virtual is height * number of available framebuffers
	info->var.yres_virtual = pi->height *  (fb_size / (info->fix.line_length * pi->height));
	info->var.bits_per_pixel = pi->bits_per_pixel;
	info->var.accel_flags = 0;

	info->var.yoffset = 0;
	info->var.reserved[0] = 0x54445055;
	info->var.reserved[1] = 0;
	info->var.reserved[2] = (uint16_t)pi->width |
				((uint32_t)pi->height << 16);

	switch(pi->bits_per_pixel)
	{
		case 32:
			info->var.red.offset = 16;
			info->var.red.length = 8;
			info->var.red.msb_right = 0;
			info->var.green.offset = 8;
			info->var.green.length = 8;
			info->var.green.msb_right = 0;
			info->var.blue.offset = 0;
			info->var.blue.length = 8;
			info->var.blue.msb_right = 0;
			break;
		case 16:
			info->var.red.offset = 11;
			info->var.red.length = 5;
			info->var.red.msb_right = 0;
			info->var.green.offset = 5;
			info->var.green.length = 6;
			info->var.green.msb_right = 0;
			info->var.blue.offset = 0;
			info->var.blue.length = 5;
			info->var.blue.msb_right = 0;
			break;
		default:
			info->var.red.offset = 11;
			info->var.red.length = 5;
			info->var.red.msb_right = 0;
			info->var.green.offset = 5;
			info->var.green.length = 6;
			info->var.green.msb_right = 0;
			info->var.blue.offset = 0;
			info->var.blue.length = 5;
			info->var.blue.msb_right = 0;
			break;			
	}		

	ret = fb_alloc_cmap(&info->cmap, 16, 0);
	info->pseudo_palette = PP;

	PP[0] = 0;
	for (i = 1; i < 16; i++)
		PP[i] = 0xffffffff;


	par->fb_context_info = msmfb_context;

	
	return 0;
}

/*
 * msmfb has been restructured to expose two set of buffers
 * for userspace - fb0 and fb1. Each set of buffers will have
 * its own fb_info and msmfb_info; but they will share one
 * msmfb_context_info, which holds the contextual information
 * such as hardware state, locks, etc.
 */

static int msmfb_probe(struct platform_device *pdev)
{
	unsigned char *fb0ram;
	unsigned char *fb1ram;
	unsigned char *ibufram;
	struct fb_info *fb0_info = NULL;
	struct fb_info *fb1_info = NULL;
	struct mddi_panel_info *pi = pdev->dev.platform_data;

	printk(KERN_INFO "msmfb_probe() installing %d x %d panel\n",
	       pi->width, pi->height);

	/*
	 * msmfb has been restructured to expose two set of buffers
	 * for userspace - fb0 and fb1. Each set of buffers will have
 	 * its own fb_info and msmfb_info; but they will share one
	 * msmfb_context_info, which holds the contextual information
	 * such as hardware state, locks, etc.
	 */

	//Allocate structure to hold shared global info for all framebuffers
	msmfb_context = kzalloc(sizeof(struct msmfb_context_info), GFP_KERNEL);

	if(!msmfb_context) {
		printk(KERN_ERR "msmfb: cannot allocate msmfb_context!\n");
	}
	

	fb0ram = ioremap(pi->fb0_base, pi->fb0_size);
	fb1ram = ioremap(pi->fb1_base, pi->fb1_size);
	ibufram = ioremap(pi->ibuf_base, pi->ibuf_size);

	if (fb0ram == 0 || fb1ram ==0 || ibufram==0) {
		printk(KERN_ERR "msmfb: cannot allocate fbram!\n");
		return -ENOMEM;
	}

	fb0_info = framebuffer_alloc(sizeof(struct msmfb_info), &pdev->dev);
	fb1_info = framebuffer_alloc(sizeof(struct msmfb_info), &pdev->dev);

	if (!fb0_info || !fb1_info ) {
		printk(KERN_ERR "msmfb: cannot allocate fb_info\n");
		return -ENOMEM;
	}

	//Allocate and initailize the two fb_info associated
	init_fb_info(fb0_info,"msmfb0", fb0ram, pi, pi->fb0_base, pi->fb0_size);
	init_fb_info(fb1_info,"msmfb1", fb1ram, pi, pi->fb1_base, pi->fb1_size);

	register_framebuffer(fb0_info);
	register_framebuffer(fb1_info);
	msmfb_context->fb[0] = fb0_info;
	msmfb_context->fb[1] = fb1_info;
	msmfb_context->fb_num = 2;

	printk(KERN_INFO "msmfb_probe() registered fb0 and fb1\n");

	msmfb_context->wait_for_vsync = true;

 	//Context information
	msmfb_context->ibuf_start = pi->ibuf_base;
	msmfb_context->ibuf_len = pi->ibuf_size;
	msmfb_context->ibuf_base = ibufram;
	msmfb_context->ibuf_offset = 0;

	msmfb_context->pipeline = PIPELINE_GRAPHICS;
	msmfb_context->dma2_ibufformat = DMA_IBUF_FORMAT_ARGB8888;
	msmfb_context->panel_info = pi;
	hrtimer_init(&msmfb_context->fake_vsync, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#ifdef CONFIG_ANDROID_POWER
	msmfb_context->idle_lock.name = "msmfb_idle_lock";
	android_init_suspend_lock(&msmfb_context->idle_lock);
#endif
	msmfb_context->dma_callback.func = msmfb_handle_dma_interrupt; 
	msmfb_context->vsync_callback.func = msmfb_handle_vsync_interrupt;
	msmfb_context->fake_vsync.function = msmfb_fake_vsync;
	hrtimer_init(&msmfb_context->video_idle_check, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	msmfb_context->video_idle_check.function = msmfb_video_idle_check;
	spin_lock_init(&msmfb_context->update_lock);
	init_waitqueue_head(&msmfb_context->frame_wq);
	init_waitqueue_head(&msmfb_context->update_wq);
	init_waitqueue_head(&msmfb_context->wq);
	msmfb_context->frameupdate = FB_READY;


#ifdef CONFIG_ANDROID_POWER
	INIT_WORK(&msmfb_context->resume_work, power_on_panel);
#endif

#ifdef CONFIG_FB_MSM_LOGO
        if (!load_565rle_image( INIT_IMAGE_FILE )) {
                /* Flip buffer */
                msmfb_context->update_info.left = 0;
                msmfb_context->update_info.top = 0;
                msmfb_context->update_info.eright = info->var.xres;
                msmfb_context->update_info.ebottom = info->var.yres;
                msmfb_pan_update( info, 0, 0, info->var.xres, info->var.yres,
                                    0, 1 );
        }
#endif

#ifdef CONFIG_ANDROID_POWER
	msmfb_context->slightly_earlier_suspend.suspend = msmfb_slightly_earlier_suspend;
	msmfb_context->slightly_earlier_suspend.resume = msmfb_slightly_later_resume;
	msmfb_context->slightly_earlier_suspend.level =
		ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	android_register_early_suspend(&msmfb_context->slightly_earlier_suspend);
	msmfb_context->early_suspend.suspend = msmfb_early_suspend;
	msmfb_context->early_suspend.resume = msmfb_early_resume;
	msmfb_context->early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	android_register_early_suspend(&msmfb_context->early_suspend);
#endif

#if MSMFB_DEBUG
	debugfs_create_file("msm_fb", S_IFREG | S_IRUGO, NULL,
			    (void *)msmfb_context, &debug_fops);
#endif

	mutex_init(&msmfb_context->autoupdate_lock);
	mutex_init(&msmfb_context->videoupdate_lock);
	mutex_init(&msmfb_context->graphicsmerge_lock);
	msmfb_context->autoupdate = 0;

	msmfb_context->last_addr = -1;

	// Start a thread that will keep updating the display 30 times per second
	msmfb_context->task = kthread_run(msmfb_constant_update, msmfb_context->fb[0], "msm_fb");
	if (IS_ERR(msmfb_context->task)) 
	{
		printk(KERN_INFO "msm_fb: failed to create update thread!\n");
	}

	// register display on callback
	if(pi->panel_ops->display_on)
	{
		msmfb_context->displayon_callback.func = msmfb_display_on;
		pi->panel_ops->display_on(pi, &msmfb_context->displayon_callback);
	}

	// register display off callback
	if(pi->panel_ops->display_off)
	{
		msmfb_context->displayoff_callback.func = msmfb_display_off;
		pi->panel_ops->display_off(pi, &msmfb_context->displayoff_callback);
	}

	INIT_WORK(&msmfb_context->push_video_work, msmfb_push_frame);

	msmfb_context->active_fbs = (FB0_ENABLED);
	msmfb_context->sleeping = AWAKE;

	return 0;
}

static int msmfb_remove(struct platform_device *pdev)
{
	return 0;
}



static struct platform_driver mddi_panel_driver = {
	.probe   = msmfb_probe,
	.remove  = msmfb_remove,
	.suspend	= msmfb_suspend,
	.resume = msmfb_resume,
	.driver  = { .name = "mddi_panel" },
};

extern int mdp_init(void);

static int __init msmfb_init(void)
{
	int ret;

	ret = mdp_init();
	if (ret)
		return ret;


	return platform_driver_register(&mddi_panel_driver);
}

module_init(msmfb_init);
