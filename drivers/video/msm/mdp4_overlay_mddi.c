/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/io.h>

#include <asm/system.h>
#include <asm/mach-types.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>

#include <linux/fb.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"


static struct mdp4_overlay_pipe *mddi_pipe = NULL;
static struct mdp4_overlay_pipe *pending_pipe;
static struct mdp4_overlay_pipe *vsync_wait_pipe;
static struct msm_fb_data_type *mddi_mfd;

/*vsync related variables*/
ktime_t mdp4_overlay0_last_update_time = {0};
uint32 mdp4_overlay0_update_time_in_usec;
uint32 mdp4_overlay0_completed_time_in_usec;
ktime_t mdp4_dmas_last_update_time = {0};
uint32 mdp4_dmas_update_time_in_usec;
uint32 mdp4_dmas_completed_time_in_usec;

int mdp4_lcd_rd_cnt_offset_slow = 20;
int mdp4_lcd_rd_cnt_offset_fast = 20;
int mdp4_vsync_usec_wait_line_too_short = 5;
uint32 mdp4_total_vdopkts;

bool initialized = false;

//#define WHOLESCREEN
//#define MDP4_DMA_TIMING_DEBUG

#ifdef MDP4_DMA_TIMING_DEBUG
static int count_overlay;
static uint32_t sum_overlay;
static int count_dmas;
static uint32_t sum_dmas;
#endif

void mdp4_dma_s_update_lcd(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe);
void mdp4_mddi_dma_s_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe);

void mdp4_overlay_update_lcd(struct msm_fb_data_type *mfd)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint8 *src;
	int bpp;
	uint32 mddi_ld_param;
	uint16 mddi_vdo_packet_reg;
	struct fb_info *fbi = NULL;

	if (mfd->key != MFD_KEY)
		return;

	mddi_mfd = mfd;		/* keep it */
	bpp = iBuf->bpp;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

	if (!initialized) {

		mddi_ld_param = 0;
		mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

		if (mfd->panel_info.type == MDDI_PANEL) {
			if (mfd->panel_info.pdest == DISPLAY_1)
				mddi_ld_param = 0;
			else
				mddi_ld_param = 1;
		} else {
			mddi_ld_param = 2;
		}

		MDP_OUTP(MDP_BASE + 0x00090, mddi_ld_param);

		if (mfd->panel_info.bpp == 24)
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_24 << 16) | mddi_vdo_packet_reg);
		else if (mfd->panel_info.bpp == 16)
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_16 << 16) | mddi_vdo_packet_reg);
		else
			MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

		MDP_OUTP(MDP_BASE + 0x00098, 0x01);

		initialized = true;
	} 

	
	if(mfd->enabled_fbs == LAYER_FB0) {
		fbi = mfd->fbi[0]; //fb0 as base
		mddi_pipe = 
			mdp4_overlay_ndx2pipe(mfd->overlay_g1_pipe_index);
	}
	else if(mfd->enabled_fbs == (LAYER_FB0|LAYER_FB1)) {
		fbi = mfd->fbi[1]; //fb1 as base
		mddi_pipe = 
			mdp4_overlay_ndx2pipe(mfd->overlay_g2_pipe_index);
	}
	else if(mfd->enabled_fbs == (LAYER_FB0|LAYER_VIDEO)) {
		mddi_pipe = 
			mdp4_overlay_ndx2pipe(mfd->overlay_v1_pipe_index);
	}
	else if(mfd->enabled_fbs == (LAYER_FB1|LAYER_FB0|LAYER_VIDEO)) {
		mddi_pipe = 
			mdp4_overlay_ndx2pipe(mfd->overlay_v1_pipe_index);
	}


#ifdef WHOLESCREEN
	{

		mddi_pipe->src_height = fbi->var.yres;
		mddi_pipe->src_width = fbi->var.xres;
		mddi_pipe->src_h = fbi->var.yres;
		mddi_pipe->src_w = fbi->var.xres;
		mddi_pipe->src_y = 0;
		mddi_pipe->src_x = 0;
		mddi_pipe->dst_h = fbi->var.yres;
		mddi_pipe->dst_w = fbi->var.xres;
		mddi_pipe->dst_y = 0;
		mddi_pipe->dst_x = 0;
		mddi_pipe->srcp0_addr = (uint32)src;
		mddi_pipe->srcp0_ystride = fbi->fix.line_length;
	}

#else
	if (mddi_pipe->pipe_type == OVERLAY_TYPE_RGB) {

		src = (uint8 *) iBuf->buf;

		/* As of 1220 drop, Qualcomm seems to have disabled 
		 * partial update, we will do full screen update for now.
		 */
		if (1) {

			//Full screen update when blending is needed

			mddi_pipe->src_height = fbi->var.yres;
			mddi_pipe->src_width = fbi->var.xres;
			mddi_pipe->src_h = fbi->var.yres;
			mddi_pipe->src_w = fbi->var.xres;
			mddi_pipe->src_y = 0;
			mddi_pipe->src_x = 0;
			mddi_pipe->dst_y = 0;
			mddi_pipe->dst_x = 0;
			mddi_pipe->dst_h = fbi->var.yres;
			mddi_pipe->dst_w = fbi->var.xres;
			mddi_pipe->srcp0_addr = (uint32) src;
			mddi_pipe->srcp0_ystride = fbi->fix.line_length;

		}else {
			//Partial screen update when blending is NOT needed (just fb0)

			src += (iBuf->dma_x + iBuf->dma_y * iBuf->ibuf_width) * bpp;

			mddi_pipe->src_height = iBuf->dma_h;
			mddi_pipe->src_width = iBuf->dma_w;
			mddi_pipe->src_h = iBuf->dma_h;
			mddi_pipe->src_w = iBuf->dma_w;
			mddi_pipe->src_y = 0;
			mddi_pipe->src_x = 0;
			mddi_pipe->dst_h = iBuf->dma_h;
			mddi_pipe->dst_w = iBuf->dma_w;
			mddi_pipe->dst_y = iBuf->dma_y;
			mddi_pipe->dst_x = iBuf->dma_x;
			mddi_pipe->srcp0_addr = (uint32) src;
			mddi_pipe->srcp0_ystride = iBuf->ibuf_width * bpp;

		}
	}
#endif
	mddi_pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;


	if (mddi_pipe->pipe_type == OVERLAY_TYPE_VG) {

		//Base layer for video is filled with black
		mddi_pipe->solid_fill = 1;
		mdp4_overlay_vg_setup(mddi_pipe);	/* video/graphic pipe */

	}
	else {
		mdp4_overlay_rgb_setup(mddi_pipe);
	}

	mdp4_mixer_stage_up(mddi_pipe);

	mdp4_overlayproc_cfg(mddi_pipe);

	mdp4_overlay_dmap_xy(mddi_pipe);

	mdp4_overlay_dmap_cfg(mfd, 0);
		
	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

}


/*
 * mdp4_overlay0_done_mddi: called from isr
 */
void mdp4_overlay0_done_mddi()
{
#ifdef MDP4_NONBLOCKING
	mdp_disable_irq_nosync(MDP_OVERLAY0_TERM);
#endif

	if (pending_pipe) {
		mdp_disable_irq(MDP_OVERLAY0_TERM);
		complete(&pending_pipe->comp);
		pending_pipe = NULL;
	}

#ifdef MDP4_DMA_TIMING_DEBUG
	mdp4_overlay0_completed_time_in_usec =
		MDP_KTIME2USEC(ktime_get_real());
	mdp4_overlay0_update_time_in_usec =
		MDP_KTIME2USEC(mdp4_overlay0_last_update_time);
	sum_overlay+= (mdp4_overlay0_completed_time_in_usec - mdp4_overlay0_update_time_in_usec); 
	count_overlay++;
	if(count_overlay%100 == 0) {
		do_div(sum_overlay, 100000);
		printk("Avg DMA over last 100 frames for Overlay: %d ms\n", sum_overlay);
		sum_overlay = 0;
		count_overlay = 0;
	}
#endif
}

void mdp4_mddi_overlay_restore(void)
{
#ifdef MDP4_MDDI_DMA_SWITCH
	mdp4_mddi_overlay_dmas_restore();
#else
	/* mutex holded by caller */
	if (mddi_mfd && mddi_pipe) {
		mdp4_overlay_update_lcd(mddi_mfd);
		mdp4_mddi_overlay_kickoff(mddi_mfd, mddi_pipe);
	}
#endif
}

void mdp4_overlay_workqueue_handler(struct work_struct *work)
{
	struct msm_fb_data_type *mfd = NULL;

	mfd = container_of(work, struct msm_fb_data_type, overlay_vsync_worker);

	//Do overlay + DMA
	if(vsync_wait_pipe) {
#ifdef MDP4_MDDI_DMA_SWITCH
		if (mdp4_overlay_pipe_staged(mddi_pipe->mixer_num) <= 1) {
			mdp4_mddi_dma_s_kickoff(mfd, vsync_wait_pipe);
		}else 
#endif
		{
			mdp4_mddi_overlay_kickoff(mfd, vsync_wait_pipe);
		}
	}

	vsync_wait_pipe = NULL;
}

extern struct workqueue_struct *mdp_vsync_wq;	/*mdp vsync wq */
enum hrtimer_restart mdp4_overlay_vsync_hrtimer_handler(struct hrtimer *ht)
{
	struct msm_fb_data_type *mfd = NULL;

	mfd = container_of(ht, struct msm_fb_data_type, dma_hrtimer);

	if (!queue_work(mdp_vsync_wq, &mfd->overlay_vsync_worker)) {
		printk(KERN_ERR"mdp4_overlay_vsync_hrtimer_handler: can't queue_work! ->\n");
	}

	return HRTIMER_NORESTART;
}


void mdp4_overlay_schedule(
	struct msm_fb_data_type *mfd, 
	struct mdp4_overlay_pipe *pipe, 
	uint32 term)
{
	/*
	 * dma2 configure VSYNC block
	 * vsync supported on Primary LCD only for now
	 */
	int32 mdp4_lcd_rd_cnt;
	uint32 usec_wait_time;

	/* SW vsync logic starts here */

	/* get current rd counter */
	mdp4_lcd_rd_cnt = mdp_get_lcd_line_counter(mfd);

	if (mdp4_lcd_rd_cnt < 0)
		mdp4_lcd_rd_cnt = mfd->total_lcd_lines + mdp4_lcd_rd_cnt;
	else if (mdp4_lcd_rd_cnt > mfd->total_lcd_lines)
		mdp4_lcd_rd_cnt = mdp4_lcd_rd_cnt - mfd->total_lcd_lines - 1;

	/* Predict when the next vsync will happen
	 * If it is really soon, just fire off dma
	 */
	if (((mfd->total_lcd_lines - mdp4_lcd_rd_cnt)) <=
		mdp4_vsync_usec_wait_line_too_short) {
		usec_wait_time = 0;
	} else {
		usec_wait_time =
			( (mfd->total_lcd_lines -
			   mdp4_lcd_rd_cnt) * 1000000) /
			( (mfd->total_lcd_lines *
			   mfd->panel_info.lcd.refx100) / 100);
	}


	if (usec_wait_time == 0) {
#ifdef MDP4_MDDI_DMA_SWITCH
		if (mdp4_overlay_pipe_staged(mddi_pipe->mixer_num) <= 1) {
			mdp4_mddi_dma_s_kickoff(mfd, pipe);
		}else 
#endif
		{
			mdp4_mddi_overlay_kickoff(mfd, pipe);
		}
	} else {
		ktime_t wait_time;

		wait_time.tv.sec = 0;
		wait_time.tv.nsec = usec_wait_time * 1000;

		vsync_wait_pipe = pipe;

		hrtimer_start(&mfd->dma_hrtimer, wait_time, HRTIMER_MODE_REL);

	}
}

void mdp4_mddi_overlay_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
#ifdef MDP4_NONBLOCKING
	unsigned long flag;

	if (pipe == mddi_pipe) {  /* base layer */
		if (mdp4_overlay_pipe_staged(pipe->mixer_num) > 1) {
			if (time_before(jiffies,
				(mddi_last_kick + mddi_kick_interval/2))) {
				mdp4_stat.kickoff_mddi_skip++;
				return;	/* let other pipe to kickoff */
			}
		}
	}

	spin_lock_irqsave(&mdp_spin_lock, flag);
	if (mfd->dma->busy == TRUE) {
		INIT_COMPLETION(pipe->comp);
		pending_pipe = pipe;
	}
	spin_unlock_irqrestore(&mdp_spin_lock, flag);

	if (pending_pipe != NULL) {
		/* wait until DMA finishes the current job */
		wait_for_completion_killable(&pipe->comp);
		pending_pipe = NULL;
	}
	down(&mfd->sem);
	mdp_enable_irq(MDP_OVERLAY0_TERM);
	mfd->dma->busy = TRUE;
	/* start OVERLAY pipe */
	mdp_pipe_kickoff(MDP_OVERLAY0_TERM, mfd);
	if (pipe != mddi_pipe) { /* non base layer */
		int intv;

		intv = jiffies - mddi_last_kick;
		mddi_kick_interval += intv;
		mddi_kick_interval /= 2;	/* average */
		mddi_last_kick = jiffies;
	}
	up(&mfd->sem);
#else

	down(&mfd->sem);
	mdp_enable_irq(MDP_OVERLAY0_TERM);
	pending_pipe = pipe;

	/* start OVERLAY pipe */
	mfd->dma->busy = TRUE;
	mdp_pipe_kickoff(MDP_OVERLAY0_TERM, mfd);
	up(&mfd->sem);

#endif
}

#ifdef MDP4_MDDI_DMA_SWITCH

void mdp4_dma_s_done_mddi()
{
	if (pending_pipe) {
		mdp_disable_irq(MDP_DMA_S_TERM);
		complete(&pending_pipe->dmas_comp);
		pending_pipe = NULL;
	}

#ifdef MDP4_DMA_TIMING_DEBUG
	mdp4_dmas_completed_time_in_usec =
		MDP_KTIME2USEC(ktime_get_real());
	mdp4_dmas_update_time_in_usec =
		MDP_KTIME2USEC(mdp4_dmas_last_update_time);
	sum_dmas+= (mdp4_dmas_completed_time_in_usec - mdp4_dmas_update_time_in_usec); 
	count_dmas++;
	if(count_dmas%100 == 0) {
		do_div(sum_dmas, 100000);
		printk("MDP4: Avg DMA time over last 100 frames for DMA S: %d ms\n", sum_dmas);
		sum_dmas = 0;
		count_dmas = 0;
	}
#endif
}

void mdp4_dma_s_update_lcd(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	uint32 outBpp = iBuf->bpp;
	uint16 mddi_vdo_packet_reg;
	uint32 dma_s_cfg_reg;

	dma_s_cfg_reg = 0;

	if (mfd->fb_imgType == MDP_RGBA_8888)
		dma_s_cfg_reg |= DMA_PACK_PATTERN_BGR; /* on purpose */
	else if (mfd->fb_imgType == MDP_BGR_565)
		dma_s_cfg_reg |= DMA_PACK_PATTERN_BGR;
	else
		dma_s_cfg_reg |= DMA_PACK_PATTERN_RGB;

	if (outBpp == 4)
		dma_s_cfg_reg |= (1 << 26); /* xRGB8888 */
	else if (outBpp == 2)
		dma_s_cfg_reg |= DMA_IBUF_FORMAT_RGB565;

	dma_s_cfg_reg |= DMA_DITHER_EN;

	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	/* PIXELSIZE */
	MDP_OUTP(MDP_BASE + 0xa0004, (pipe->dst_h << 16 | pipe->dst_w));
	MDP_OUTP(MDP_BASE + 0xa0008, pipe->srcp0_addr);	/* ibuf address */
	MDP_OUTP(MDP_BASE + 0xa000c, pipe->srcp0_ystride);/* ystride */

	if (mfd->panel_info.bpp == 24) {
		dma_s_cfg_reg |= DMA_DSTC0G_8BITS |	/* 666 18BPP */
		    DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
	} else if (mfd->panel_info.bpp == 18) {
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 666 18BPP */
		    DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
	} else {
		dma_s_cfg_reg |= DMA_DSTC0G_6BITS |	/* 565 16BPP */
		    DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
	}

	MDP_OUTP(MDP_BASE + 0xa0010, (pipe->dst_y << 16) | pipe->dst_x);
	MDP_OUTP(MDP_BASE + 0x00090, 1); /* do not change this */

	mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

	if (mfd->panel_info.bpp == 24)
		MDP_OUTP(MDP_BASE + 0x00094,
			(MDDI_VDO_PACKET_DESC_24 << 16) | mddi_vdo_packet_reg);
	else if (mfd->panel_info.bpp == 16)
		MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC_16 << 16) | mddi_vdo_packet_reg);
	else
		MDP_OUTP(MDP_BASE + 0x00094,
			 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

	MDP_OUTP(MDP_BASE + 0x00098, 0x01);

	MDP_OUTP(MDP_BASE + 0xa0000, dma_s_cfg_reg);

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
}

void mdp4_mddi_dma_s_kickoff(struct msm_fb_data_type *mfd,
				struct mdp4_overlay_pipe *pipe)
{
	down(&mfd->sem);
	mdp_enable_irq(MDP_DMA_S_TERM);
	mfd->ibuf_flushed = TRUE;
	pending_pipe = pipe;
	/* start dma_s pipe */
	mfd->dma->busy = TRUE;
	mdp_pipe_kickoff(MDP_DMA_S_TERM, mfd);
	up(&mfd->sem);
}

void mdp4_mddi_overlay_dmas_restore(void)
{
	/* mutex holded by caller */
	if (mddi_mfd && mddi_pipe) {
		mdp4_dma_s_update_lcd(mddi_mfd, mddi_pipe);
		mdp4_mddi_dma_s_kickoff(mddi_mfd, mddi_pipe);
	}
}
#endif

void mdp4_mddi_overlay(struct msm_fb_data_type *mfd)
{
	int rc=0;
	mutex_lock(&mfd->dma->ov_mutex);

#ifdef MDP4_NONBLOCKING
	if (mfd && mfd->panel_power_on) {
#else
	if ((mfd) && (!mfd->dma->busy) && (mfd->panel_power_on)) {
	
		down(&mfd->sem);
		INIT_COMPLETION(mddi_pipe->dmas_comp);
		INIT_COMPLETION(mddi_pipe->comp);
		up(&mfd->sem);

		/*Always do this so that all the data structures are in sync
		for both Overlay and DMA paths*/
		mdp4_overlay_update_lcd(mfd);

#ifdef MDP4_MDDI_DMA_SWITCH

		/* Use DMA S path to save power when only fb0 is active */

		if (mdp4_overlay_pipe_staged(mddi_pipe->mixer_num) <= 1) {

			mdp4_dma_s_update_lcd(mfd, mddi_pipe);

			if(mfd->wait_for_vsync & mfd->update_fb) 
				mdp4_overlay_schedule(mfd, mddi_pipe, MDP_DMA_S_TERM);
			else {
 				/* If only fb0 is active, use DMA S */
				mdp4_mddi_dma_s_kickoff(mfd, mddi_pipe);
			} 

			rc = wait_for_completion_interruptible_timeout(&mddi_pipe->dmas_comp,100);

			if(rc == 0)
				printk("mdp4_mddi_overlay: timed out waiting for DMA S complete\n");
		} else
#endif
		
		{

			if(mfd->wait_for_vsync & mfd->update_fb) 
				mdp4_overlay_schedule(mfd, mddi_pipe, MDP_OVERLAY0_TERM);
			else {
 				/* If other than fb0 is active, use the overlay engine */
				mdp4_mddi_overlay_kickoff(mfd, mddi_pipe);
			}

			rc = wait_for_completion_interruptible_timeout(&mddi_pipe->comp,100);

			if(rc == 0)
				printk("mdp4_mddi_overlay: timed out waiting for Overlay 0 complete\n");
		
		}

		/* reset the vsync idle count to prevent disabling of the vsync */
		mfd->vsync_idle_count = 0;
		if(mfd->wait_for_vsync & mfd->update_fb) 
			mfd->wait_for_vsync &= (~mfd->update_fb); 

#endif

		/* signal if pan function is waiting for the update completion */
		if (mfd->pan_waiting) {
			mfd->pan_waiting = FALSE;
			complete(&mfd->pan_comp);
		}
	}



	mutex_unlock(&mfd->dma->ov_mutex);
}
