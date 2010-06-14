/*
 * drivers/media/video/omap24xx/omap24xxlib.h
 *
 * Copyright (C) 2005 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 */

#ifndef OMAP24XXLIB_VIDEO_H
#define OMAP24XXLIB_VIDEO_H

extern void
 omap24xxvout_default_crop(struct v4l2_pix_format *pix,	struct v4l2_framebuffer *fbuf,
				struct v4l2_rect *crop);

extern int
 omap24xxvout_new_crop(struct v4l2_pix_format *pix, struct v4l2_rect *crop,
			struct v4l2_window *win, struct v4l2_framebuffer *fbuf,
			const struct v4l2_rect *new_crop);

extern int
 omap24xxvout_try_window(struct v4l2_framebuffer *fbuf, struct v4l2_window *new_win);

extern int
 omap24xxvout_new_window(struct v4l2_rect *crop,struct v4l2_window *win,
			struct v4l2_framebuffer *fbuf, struct v4l2_window *new_win);

extern void
 omap24xxvout_new_format(struct v4l2_pix_format *pix, struct v4l2_framebuffer *fbuf,
			 struct v4l2_rect *crop, struct v4l2_window *win);
#endif
