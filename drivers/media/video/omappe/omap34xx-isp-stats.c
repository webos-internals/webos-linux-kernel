/*
 * drivers/media/video/omap/omap34xx-isp-stats.c
 *
 * Copyright (C) Palm, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <asm/arch/isp.h>
#include "omap34xx.h"
#include "omap34xx-isp.h"

#define ISP_BASE			0x480BC000
#define ISP_REG(off)			(ISP_BASE + (off))

#define ISP_IRQ0ENABLE			ISP_REG(0x0C)
#define ISP_IRQ0STATUS			ISP_REG(0x10)
#define	H3A_AWB_DONE_IRQ_MASK		(0x1 << 13)

#define ISP_CTRL			ISP_REG(0x40)
#define	H3A_CLK_EN_MASK			(0x1 << 10)

#define H3A_AEAWB_ALIGN			64
#define H3A_AEAWB_PACKET_SIZE		144

/* NOTE: includes non-optional black row */
#define H3A_AEAWB_WINDOW_COUNT(fmt)	\
	(((fmt)->aewin1_winhc + 1) * ((fmt)->aewin1_winvc + 2))

#define H3A_AEAWB_PACKET_COUNT(fmt)	\
	((H3A_AEAWB_WINDOW_COUNT(fmt) + 7) / 8)

#define H3A_AEAWB_BUFFER_SIZE(fmt)	\
	(H3A_AEAWB_PACKET_SIZE * H3A_AEAWB_PACKET_COUNT(fmt))

#define H3A_BASE			0x480BCC00
#define H3A_REG(off)			(H3A_BASE + (off))

#define H3A_PCR				H3A_REG(0x04)
#define	PCR_AVE2LMT_MASK		(0x3FF << 22)
#define	PCR_AVE2LMT(val)		((0x3FF & (val)) << 22)
#define	PCR_BUSYAEAWB_MASK		(1 << 18)
#define	PCR_AEW_EN_MASK			(1 << 16)
#define	PCR_AEW_EN(val)			((0x1 & (val)) << 16)

#define H3A_AEWWIN1			H3A_REG(0x4C)
#define	AEWWIN1_WINH_MASK		(0x7F << 24)
#define	AEWWIN1_WINH(val)		((0x7F & (val)) << 24)
#define	AEWWIN1_WINW_MASK		(0x7F << 13)
#define	AEWWIN1_WINW(val)		((0x7F & (val)) << 13)
#define	AEWWIN1_WINVC_MASK		(0x7F << 6)
#define	AEWWIN1_WINVC(val)		((0x7F & (val)) << 6)
#define	AEWWIN1_WINHC_MASK		(0x3F << 0)
#define	AEWWIN1_WINHC(val)		((0x3F & (val)) << 0)

#define H3A_AEWINSTART			H3A_REG(0x50)
#define	AEWINSTART_WINSV_MASK		(0xFFF << 16)
#define	AEWINSTART_WINSV(val)		((0xFFF & (val)) << 16)
#define	AEWINSTART_WINSH_MASK		(0xFFF << 0)
#define	AEWINSTART_WINSH(val)		((0xFFF & (val)) << 0)

#define H3A_AEWINBLK			H3A_REG(0x54)
#define	AEWINBLK_WINSV_MASK		(0xFFF << 16)
#define	AEWINBLK_WINSV(val)		((0xFFF & (val)) << 16)
#define	AEWINBLK_WINH_MASK		(0x7F << 0)
#define	AEWINBLK_WINH(val)		((0x7F & (val)) << 0)

#define H3A_AEWSUBWIN			H3A_REG(0x58)
#define	AEWSUBWIN_AEWINCV_MASK		(0xF << 8)
#define	AEWSUBWIN_AEWINCV(val)		((0xF & (val)) << 8)
#define	AEWSUBWIN_AEWINCH_MASK		(0xF << 0)
#define	AEWSUBWIN_AEWINCH(val)		((0xF & (val)) << 0)

#define H3A_AEWBUFST			H3A_REG(0x5C)
#define	AEWBUFST_AEWBUFST_MASK		0xFFFFFFD0
#define	AEWBUFST_AEWBUFST(val)		(0xFFFFFFD0 & (val))

#define SPEW(level, args...)					\
	do {							\
		if (omap34xx_isp_stats_spew_level >= level)	\
			printk(KERN_DEBUG "STATS:\t" args);	\
	} while (0)

struct omap34xx_isp_stats_buffer {
	struct videobuf_buffer	qbuf;
	dma_addr_t		aeawb_dma;
	dma_addr_t		stats_dev;
	size_t			stats_len;
	void			*sof_cpu;
	size_t			sof_len;
};

struct omap34xx_isp_stats_data {
	unsigned long				stats;
	unsigned long				delays;
	struct omap34xx_v4l2_device		dev;
	struct omap34xx_isp_stats_format	fmt;
	struct omap34xx_isp_stats_buffer	*buf;
};

static int omap34xx_isp_stats_spew_level = 0;

#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
static ssize_t
omap34xx_isp_stats_show_spew_level(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t i;

	i = snprintf(buf, PAGE_SIZE, "%d\n", omap34xx_isp_stats_spew_level);

	return (i + 1);
}

static ssize_t
omap34xx_isp_stats_store_spew_level(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char *endp;

	omap34xx_isp_stats_spew_level = simple_strtoul(buf, &endp, 10);

	return (count);
}

static struct device_attribute omap34xx_isp_stats_attr_spew_level =
	__ATTR(stats_spew_level, S_IRUGO|S_IWUGO,
		omap34xx_isp_stats_show_spew_level,
		omap34xx_isp_stats_store_spew_level);

static inline int
omap34xx_isp_stats_create_spew_level(struct device *dev)
{
	return (device_create_file(dev, &omap34xx_isp_stats_attr_spew_level));
}

static inline void
omap34xx_isp_stats_remove_spew_level(struct device *dev)
{
	device_remove_file(dev, &omap34xx_isp_stats_attr_spew_level);
}
#else // !CONFIG_VIDEO_OMAP34XX_ISP_DBG
static inline int
omap34xx_isp_stats_create_spew_level(struct device *dev)
{
	return (0);
}

static inline void
omap34xx_isp_stats_remove_spew_level(struct device *dev)
{
}
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG

static void omap34xx_isp_stats_videobuf_start(struct omap34xx_v4l2_device *dev,
						struct videobuf_buffer *qbuf)
{
	struct omap34xx_isp_input *input;
	struct omap34xx_isp_stats_data *data = dev->priv;
	struct omap34xx_isp_stats_buffer *buf;

	buf = container_of(qbuf, struct omap34xx_isp_stats_buffer, qbuf);
	input = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	BUG_ON(data->buf);

	SPEW(3, "%s: index=%u\n", __func__, qbuf->i);

	data->buf = buf;
	wmb();

	omap_writel(AEWBUFST_AEWBUFST(buf->aeawb_dma), H3A_AEWBUFST);
	omap_setl(H3A_PCR, PCR_AEW_EN_MASK);
}

static int omap34xx_isp_stats_isr(struct omap34xx_isp_v4l2_device *dev)
{
	unsigned int irq0;
	struct omap34xx_isp_stats_data *data = dev->dev.priv;
	struct omap34xx_isp_stats_buffer *buf;

	irq0 = omap_readl(ISP_IRQ0STATUS);
	irq0 &= H3A_AWB_DONE_IRQ_MASK;
	omap_writel(irq0, ISP_IRQ0STATUS);

	if (!irq0)
		goto exit;

	SPEW(3, "ISP_IRQSTATUS=0x%08X(%s)\n", irq0,
		H3A_AWB_DONE_IRQ_MASK & irq0 ? __stringify(H3A_AWB_DONE) : "");

	omap_clearl(H3A_PCR, PCR_AEW_EN_MASK);

	if (!data->buf)
		goto exit;

	/*
	 * NOTE: in the (unlikely) case AE/AWB is still busy, we have missed
	 * the frame blanking period and we must not touch our DMA buffer
	 */
	if (omap_testl(H3A_PCR, PCR_BUSYAEAWB_MASK)) {
		data->delays++;
		goto exit;
	}

	buf = data->buf;
	data->buf = NULL;

	if (OMAP34XX_ISP_SMIA10_INPUT == dev->input->type)
		dev->input->phy.ccp2.copy_sof(&dev->input->phy.ccp2,
						buf->sof_cpu, buf->sof_len);

	dev->input->finalize_stat_buf = 1;
	data->stats++;

exit:
	return (!!irq0);
}

static void omap34xx_isp_stats_configure(struct omap34xx_isp_v4l2_device *dev)
{
	omap_setl(ISP_IRQ0ENABLE, H3A_AWB_DONE_IRQ_MASK);
}

static unsigned int
omap34xx_isp_stats_videobuf_size(struct omap34xx_v4l2_device *dev)
{
	unsigned int size;
	struct omap34xx_isp_stats_data *data = dev->priv;

	/* TODO: assumes format has previously been set */
	size = data->fmt.size;

	return (size);
}

static size_t
omap34xx_isp_stats_videobuf_align(struct omap34xx_v4l2_device *dev)
{
	size_t align;

	align = H3A_AEAWB_ALIGN;

	return (align);
}

static void omap34xx_isp_stats_videobuf_sync(struct videobuf_buffer *qbuf)
{
	struct omap34xx_isp_stats_buffer *buf;

	buf = container_of(qbuf, struct omap34xx_isp_stats_buffer, qbuf);
	dma_sync_single_for_cpu(NULL, buf->stats_dev, buf->stats_len,
				DMA_FROM_DEVICE);
}

static int omap34xx_isp_stats_videobuf_init(struct omap34xx_v4l2_device *dev,
						struct videobuf_buffer *qbuf)
{
	int rc;
	size_t sof;
	size_t stats = 0;
	struct omap34xx_dma_block *block = get_omap34xx_buf(qbuf)->block;
	struct omap34xx_isp_stats_data *data = dev->priv;
	struct omap34xx_isp_stats_buffer *buf;

	buf = container_of(qbuf, struct omap34xx_isp_stats_buffer, qbuf);
	stats = PAGE_ALIGN(data->fmt.h3a_aeawb.size);
	sof = PAGE_ALIGN(data->fmt.smia10.sof_size);

	if (!block) {
		rc = -EINVAL;
		goto exit;
	}

	BUG_ON((stats + sof) > block->size);

	/* TODO: use ERR_PTR */
	if (!(buf->aeawb_dma = omap34xx_isp_mmu_map(block->dev_addr, stats))) {
		rc = -ENOMEM;
		goto exit;
	}

	/*
	 * the next line relies on this function being called after
	 * omap34xx_videobuf_init. passing the sync routine during dma pool
	 * creation would be less brittle, but would require separating the
	 * video and stats pools.
	 */
	get_omap34xx_buf(qbuf)->sync = omap34xx_isp_stats_videobuf_sync;
	buf->stats_dev = block->dev_addr;
	buf->stats_len = stats;
	buf->sof_cpu = block->cpu_addr + stats;
	buf->sof_len = data->fmt.smia10.sof_size;
	rc = 0;
exit:
	SPEW(1, "%s: index=%u aeawb=0x%08X(size=%u) sof=%p(size=%u) rc=%d\n",
		__func__, qbuf->i, buf->aeawb_dma, stats, buf->sof_cpu,
		data->fmt.smia10.sof_size, rc);

	return (rc);
}

static void omap34xx_isp_stats_videobuf_release(
		struct omap34xx_v4l2_device *dev,
		struct videobuf_buffer *qbuf
		)
{
	struct omap34xx_isp_stats_buffer *buf;

	buf = container_of(qbuf, struct omap34xx_isp_stats_buffer, qbuf);

	SPEW(1, "%s: index=%u\n", __func__, qbuf->i);

	omap34xx_isp_mmu_unmap(buf->aeawb_dma);
}

static int
omap34xx_isp_stats_vidioc_querycap(struct omap34xx_v4l2_device *dev,
					struct v4l2_capability *cap)
{
	memset(cap, 0, sizeof (*cap));
	strncpy(cap->driver, OMAP34XX_ISP_DRIVER, sizeof(cap->driver));
	/* TODO: revisit */
	strncpy(cap->card, "OMAP34xx ISP", sizeof(cap->card));
	cap->version = LINUX_VERSION_CODE;
	cap->capabilities = V4L2_CAP_STATS_CAPTURE | V4L2_CAP_STREAMING;

	return (0);
}

static int
omap34xx_isp_stats_try_format(struct omap34xx_isp_v4l2_device *dev,
				struct omap34xx_isp_stats_format *fmt)
{
	int i;
	int j;
	int rc;
	struct v4l2_ifparm parm;

	SPEW(1, "+++ %s\n", __func__);

	fmt->h3a_aeawb.offset = 0;
	fmt->h3a_aeawb.size = H3A_AEAWB_BUFFER_SIZE(&fmt->h3a_aeawb);
	fmt->size = fmt->h3a_aeawb.size;

	rc = omap34xx_v4l2_int_master_ioctl_g_ifparm(&dev->input->mst, &parm);
	if (rc) goto exit;

	switch (dev->input->type) {
	case OMAP34XX_ISP_SMIA10_INPUT:
		fmt->smia10.width = 0;
		fmt->smia10.sof_offset = 0;
		fmt->smia10.sof_lines = 0;

		for (i = 0; i < parm.u.smia10.nr_cols; ++i)
			fmt->smia10.width += parm.u.smia10.descs[i].nr;

		for (j = 0; j < parm.u.smia10.nr_rows; ++i, ++j) {
			if (V4L2_IF_TYPE_SMIA10_CODE_EMBEDDED_DATA
				== parm.u.smia10.descs[i].code) {
				fmt->smia10.sof_lines =
					parm.u.smia10.descs[i].nr;
				break;
			}
		}
		if (fmt->smia10.sof_lines) {
			fmt->smia10.sof_offset = PAGE_ALIGN(fmt->size);
			fmt->smia10.sof_size = ((fmt->smia10.width * 10) / 8)
						* fmt->smia10.sof_lines;
			fmt->size = fmt->smia10.sof_offset
					+ fmt->smia10.sof_size;
		}
		break;
	default:
		BUG_ON(1);
	}
exit:
	SPEW(1, "--- %s: rc=%d size=%u h3a_aeawb.offset=%u h3a_aeawb.size=%u"
		" smia10.width=%u smia10.sof_offset=%u smia10.sof_lines=%u\n",
		__func__, rc, fmt->size, fmt->h3a_aeawb.offset,
		fmt->h3a_aeawb.size, fmt->smia10.width, fmt->smia10.sof_offset,
		fmt->smia10.sof_lines);

	return (rc);
}

static int
omap34xx_isp_stats_vidioc_g_fmt(struct omap34xx_v4l2_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	struct omap34xx_isp_stats_data *data = dev->priv;
	struct omap34xx_isp_v4l2_device *idev;

	idev = container_of(dev, struct omap34xx_isp_v4l2_device, dev);

	SPEW(1, "+++ %s\n", __func__);

	if ((rc = omap34xx_isp_stats_try_format(idev, &data->fmt)))
		goto exit;

	memcpy(fmt->fmt.raw_data, &data->fmt, sizeof(data->fmt));
exit:
	SPEW(1, "--- %s: h3a_aeawb.pcr_ave2lmt=%u h3a_aeawb.aewin1_winw=%u"
		" h3a_aeawb.aewin1_winh=%u h3a_aeawb.aewin1_winhc=%u"
		" h3a_aeawb.aewin1_winvc=%u h3a_aeawb.aewinstart_winsh=%u"
		" h3a_aeawb.aewinstart_winsv=%u h3a_aeawb.aewinblack_winh=%u"
		" h3a_aeawb.aewinblack_winsv=%u h3a_aeawb.aewsubwin_aewinch=%u"
		" h3a_aeawb.aewsubwin_aewincv=%u\n", __func__,
		data->fmt.h3a_aeawb.pcr_ave2lmt,
		data->fmt.h3a_aeawb.aewin1_winw,
		data->fmt.h3a_aeawb.aewin1_winh,
		data->fmt.h3a_aeawb.aewin1_winhc,
		data->fmt.h3a_aeawb.aewin1_winvc,
		data->fmt.h3a_aeawb.aewinstart_winsh,
		data->fmt.h3a_aeawb.aewinstart_winsv,
		data->fmt.h3a_aeawb.aewinblack_winh,
		data->fmt.h3a_aeawb.aewinblack_winsv,
		data->fmt.h3a_aeawb.aewsubwin_aewinch,
		data->fmt.h3a_aeawb.aewsubwin_aewincv);

	return (0);
}

static int
omap34xx_isp_stats_vidioc_try_fmt(struct omap34xx_v4l2_device *dev,
					struct v4l2_format *fmt)
{
	int rc;
	struct omap34xx_isp_v4l2_device *idev;
	struct omap34xx_isp_stats_format *stats;

	idev = container_of(dev, struct omap34xx_isp_v4l2_device, dev);
	stats = (struct omap34xx_isp_stats_format *)fmt->fmt.raw_data;

	SPEW(1, "+++ %s\n", __func__);

	rc = omap34xx_isp_stats_try_format(idev, stats);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
omap34xx_isp_stats_vidioc_s_fmt(struct omap34xx_v4l2_device *dev,
				struct v4l2_format *fmt)
{
	int rc;
	struct omap34xx_isp_stats_data *data = dev->priv;
	struct omap34xx_isp_v4l2_device *idev;
	struct omap34xx_isp_stats_format *stats;

	idev = container_of(dev, struct omap34xx_isp_v4l2_device, dev);
	stats = (struct omap34xx_isp_stats_format *)fmt->fmt.raw_data;

	SPEW(1, "+++ %s: h3a_aeawb.pcr_ave2lmt=%u h3a_aeawb.aewin1_winw=%u"
		" h3a_aeawb.aewin1_winh=%u h3a_aeawb.aewin1_winhc=%u"
		" h3a_aeawb.aewin1_winvc=%u h3a_aeawb.aewinstart_winsh=%u"
		" h3a_aeawb.aewinstart_winsv=%u h3a_aeawb.aewinblack_winh=%u"
		" h3a_aeawb.aewinblack_winsv=%u h3a_aeawb.aewsubwin_aewinch=%u"
		" h3a_aeawb.aewsubwin_aewincv=%u\n", __func__,
		stats->h3a_aeawb.pcr_ave2lmt, stats->h3a_aeawb.aewin1_winw,
		stats->h3a_aeawb.aewin1_winh, stats->h3a_aeawb.aewin1_winhc,
		stats->h3a_aeawb.aewin1_winvc,
		stats->h3a_aeawb.aewinstart_winsh,
		stats->h3a_aeawb.aewinstart_winsv,
		stats->h3a_aeawb.aewinblack_winh,
		stats->h3a_aeawb.aewinblack_winsv,
		stats->h3a_aeawb.aewsubwin_aewinch,
		stats->h3a_aeawb.aewsubwin_aewincv);

	if ((rc = omap34xx_isp_stats_try_format(idev, stats)))
		goto exit;

	data->fmt = *stats;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int
omap34xx_isp_stats_vidioc_streamon(struct omap34xx_v4l2_device *dev)
{
	int rc;
	unsigned int val;
	unsigned int bits;
	struct omap34xx_isp_stats_data *data = dev->priv;
	struct omap34xx_isp_v4l2_device *idev;

	idev = container_of(dev, struct omap34xx_isp_v4l2_device, dev);

	SPEW(1, "+++ %s\n", __func__);

	if ((rc = omap34xx_isp_stats_try_format(idev, &data->fmt)))
		goto exit;

	val = PCR_AVE2LMT(data->fmt.h3a_aeawb.pcr_ave2lmt);
	bits = PCR_AVE2LMT_MASK;
	omap_masked_writel(H3A_PCR, val, bits);
	SPEW(2, "H3A_PCR: AVE2LMT=%u\n", data->fmt.h3a_aeawb.pcr_ave2lmt);

	val = AEWWIN1_WINH(data->fmt.h3a_aeawb.aewin1_winh);
	val |= AEWWIN1_WINW(data->fmt.h3a_aeawb.aewin1_winw);
	val |= AEWWIN1_WINVC(data->fmt.h3a_aeawb.aewin1_winvc);
	val |= AEWWIN1_WINHC(data->fmt.h3a_aeawb.aewin1_winhc);
	omap_writel(val, H3A_AEWWIN1);
	SPEW(2, "H3A_AEWWIN1: WINH=%u WINW=%u WINVC=%u WINHC=%u\n",
		data->fmt.h3a_aeawb.aewin1_winh,
		data->fmt.h3a_aeawb.aewin1_winw,
		data->fmt.h3a_aeawb.aewin1_winvc,
		data->fmt.h3a_aeawb.aewin1_winhc);

	val = AEWINSTART_WINSV(data->fmt.h3a_aeawb.aewinstart_winsv);
	val |= AEWINSTART_WINSH(data->fmt.h3a_aeawb.aewinstart_winsh);
	omap_writel(val, H3A_AEWINSTART);
	SPEW(2, "H3A_AEWINSTART: WINSV=%u WINSH=%u\n",
		data->fmt.h3a_aeawb.aewinstart_winsv,
		data->fmt.h3a_aeawb.aewinstart_winsh);

	val = AEWINBLK_WINSV(data->fmt.h3a_aeawb.aewinblack_winsv);
	val |= AEWINBLK_WINH(data->fmt.h3a_aeawb.aewinblack_winh);
	omap_writel(val, H3A_AEWINBLK);
	SPEW(2, "H3A_AEWINBLK: WINSV=%u WINH=%u\n",
		data->fmt.h3a_aeawb.aewinblack_winsv,
		data->fmt.h3a_aeawb.aewinblack_winh);

	val = AEWSUBWIN_AEWINCV(data->fmt.h3a_aeawb.aewsubwin_aewincv);
	val |= AEWSUBWIN_AEWINCH(data->fmt.h3a_aeawb.aewsubwin_aewinch);
	omap_writel(val, H3A_AEWSUBWIN);
	SPEW(2, "H3A_AEWSUBWIN: AEWINCV=%u AEWINCH=%u\n",
		data->fmt.h3a_aeawb.aewsubwin_aewincv,
		data->fmt.h3a_aeawb.aewsubwin_aewinch);

	omap_setl(ISP_CTRL, H3A_CLK_EN_MASK);
	data->delays = data->stats = 0;
exit:
	SPEW(1, "--- %s\n", __func__);

	return (rc);
}

static int
omap34xx_isp_stats_vidioc_streamoff(struct omap34xx_v4l2_device *dev)
{
	struct omap34xx_isp_input *input;
	struct omap34xx_isp_stats_data *data = dev->priv;
	u32 irq_flags;

	input = container_of(dev, struct omap34xx_isp_v4l2_device, dev)->input;

	SPEW(1, "+++ %s\n", __func__);

	omap_clearl(H3A_PCR, PCR_AEW_EN_MASK);

	while (omap_testl(H3A_PCR, PCR_BUSYAEAWB_MASK))
		msleep(jiffies_to_msecs(1));

	omap_clearl(ISP_CTRL, H3A_CLK_EN_MASK);

	spin_lock_irqsave(&input->lock,irq_flags);
	if (input->finalize_stat_buf) {
 		omap34xx_v4l2_device_videobuf_done(&input->isp_v4l2_dev->dev,STATE_ERROR);
		input->finalize_stat_buf = 0;
	}

	/* TODO: not SMP friendly... */
	if (data->buf) {
		data->buf = NULL;
		omap34xx_v4l2_device_videobuf_done(dev, STATE_ERROR);
	}
	spin_unlock_irqrestore(&input->lock,irq_flags);

	SPEW(1, "--- %s: stats=%lu delays=%lu\n", __func__, data->stats,
		data->delays);

	return (0);
}

int
omap34xx_isp_stats_device_probe(struct omap34xx_isp_v4l2_device *dev)
{
	int rc;
	struct omap34xx_isp_stats_data *data;

	if (!(data = kzalloc(sizeof(*data), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	data->fmt.h3a_aeawb.aewinblack_winsv = 2;

	dev->isr = omap34xx_isp_stats_isr;
	dev->input->finalize_stat_buf = 0;
	spin_lock_init(&dev->input->lock);

	dev->input->isp_v4l2_dev = dev;
	dev->configure = omap34xx_isp_stats_configure;
	dev->dev.priv = data;
	dev->dev.qtype = V4L2_BUF_TYPE_STATS_CAPTURE;
	dev->dev.qsize = sizeof(struct omap34xx_isp_stats_buffer);
	dev->dev.file_open = omap34xx_isp_file_open;
	dev->dev.file_release = omap34xx_isp_file_release;
	dev->dev.videobuf_size = omap34xx_isp_stats_videobuf_size;
	dev->dev.videobuf_align = omap34xx_isp_stats_videobuf_align;
	dev->dev.videobuf_init = omap34xx_isp_stats_videobuf_init;
	dev->dev.videobuf_start = omap34xx_isp_stats_videobuf_start;
	dev->dev.videobuf_release = omap34xx_isp_stats_videobuf_release;
	dev->dev.vidioc_querycap = omap34xx_isp_stats_vidioc_querycap;
	dev->dev.vidioc_g_fmt = omap34xx_isp_stats_vidioc_g_fmt;
	dev->dev.vidioc_s_fmt = omap34xx_isp_stats_vidioc_s_fmt;
	dev->dev.vidioc_try_fmt = omap34xx_isp_stats_vidioc_try_fmt;
	dev->dev.vidioc_streamoff = omap34xx_isp_stats_vidioc_streamoff;
	dev->dev.vidioc_streamon = omap34xx_isp_stats_vidioc_streamon;

	if ((rc = omap34xx_v4l2_device_register(&dev->dev)))
		goto omap34xx_v4l2_device_register_failed;

	if ((rc = omap34xx_isp_stats_create_spew_level(&dev->dev.parent->dev)))
		goto omap34xx_isp_stats_create_spew_level_failed;

	goto exit;

omap34xx_isp_stats_create_spew_level_failed:
	video_unregister_device(&dev->dev.dev);
omap34xx_v4l2_device_register_failed:
	kfree(data);
exit:
	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_stats_device_probe);

int
omap34xx_isp_stats_device_remove(struct omap34xx_isp_v4l2_device *dev)
{
	struct omap34xx_isp_stats_data *data = dev->dev.priv;

	omap34xx_isp_stats_remove_spew_level(&dev->dev.parent->dev);
	video_unregister_device(&dev->dev.dev);
	kfree(data);

	return (0);
}
EXPORT_SYMBOL(omap34xx_isp_stats_device_remove);
