#ifndef OMAP34XX_ISP_H
#define OMAP34XX_ISP_H

#include "omap34xx-v4l2.h"
#include <asm/scatterlist.h>

#define ISP_REVISION_1_0		0x10
#define ISP_REVISION_2_0		0x20
#define ISP_REVISION_ISP2P		0xF0

enum omap34xx_isp_input_type {
	OMAP34XX_ISP_SMIA10_INPUT,
	OMAP34XX_ISP_INPUTS
};

struct omap34xx_isp_buffer {
	/*
	 * NOTE: This structure is allocated by the videobuf routines as struct
	 * videobuf_buffer, so qbuf MUST be the first member.
	 */
	struct videobuf_buffer	qbuf;
	dma_addr_t		addr;
};

struct omap34xx_isp_ccp2_phy {
	void (*copy_sof)(struct omap34xx_isp_ccp2_phy *, void *, size_t);
};

struct omap34xx_isp_input {
	enum omap34xx_isp_input_type	type;
	struct omap34xx_v4l2_int_master	mst;
	union {
		struct omap34xx_isp_ccp2_phy ccp2;
	} phy;

	int (*isr)(struct omap34xx_isp_input *);
	void (*configure)(struct omap34xx_isp_input *);
	int finalize_stat_buf;
	spinlock_t lock;
	struct omap34xx_isp_v4l2_device *isp_v4l2_dev;
	u32 isp_revision;
};

static inline int omap34xx_isp_input_isr(struct omap34xx_isp_input *in)
{
	return (in->isr ? in->isr(in) : 0);
}

static inline void omap34xx_isp_input_configure(struct omap34xx_isp_input *in)
{
	if (in->configure)
		in->configure(in);
}

struct omap34xx_isp_v4l2_device {
	struct omap34xx_isp_input	*input;
	struct omap34xx_v4l2_device	dev;

	int (*isr)(struct omap34xx_isp_v4l2_device *);
	void (*configure)(struct omap34xx_isp_v4l2_device *);
};

static inline int omap34xx_isp_v4l2_device_isr(
			struct omap34xx_isp_v4l2_device *dev
			)
{
	return (dev->isr ? dev->isr(dev) : 0);
}

static inline void omap34xx_isp_v4l2_device_configure(
			struct omap34xx_isp_v4l2_device *dev
			)
{
	if (dev->configure)
		dev->configure(dev);
}

extern int
omap34xx_isp_videobuf_init(struct omap34xx_v4l2_device *,
				struct videobuf_buffer *);
extern void
omap34xx_isp_videobuf_release(struct omap34xx_v4l2_device *,
				struct videobuf_buffer *);

extern int
omap34xx_isp_file_open(struct omap34xx_v4l2_device *);
extern int
omap34xx_isp_file_release(struct omap34xx_v4l2_device *);

extern int
omap34xx_isp_vidioc_enum_input(struct omap34xx_v4l2_device *,
				struct v4l2_input *);
extern int
omap34xx_isp_vidioc_g_input(struct omap34xx_v4l2_device *, unsigned int *);
extern int
omap34xx_isp_vidioc_s_input(struct omap34xx_v4l2_device *, unsigned int);

extern int
omap34xx_isp_smia10_input_probe(struct omap34xx_isp_input *);
extern int
omap34xx_isp_smia10_input_remove(struct omap34xx_isp_input *);

extern int
omap34xx_isp_video_device_probe(struct omap34xx_isp_v4l2_device *);
extern int
omap34xx_isp_video_device_remove(struct omap34xx_isp_v4l2_device *);
extern int
omap34xx_isp_stats_device_probe(struct omap34xx_isp_v4l2_device *);
extern int
omap34xx_isp_stats_device_remove(struct omap34xx_isp_v4l2_device *);

extern int omap34xx_isp_reset(struct platform_device *, long);

extern void
omap34xx_isp_mmu_enable(int);
extern void
omap34xx_isp_mmu_disable(void);
extern dma_addr_t
omap34xx_isp_mmu_map(u32, int);
extern dma_addr_t
omap34xx_isp_mmu_map_sg(const struct scatterlist *, int);
extern int
omap34xx_isp_mmu_unmap(dma_addr_t);

#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
extern int
omap34xx_isp_dbg_register_device(struct device *);
extern void
omap34xx_isp_dbg_unregister_device(struct device *);
#else // !CONFIG_VIDEO_OMAP34XX_ISP_DBG
static inline int
omap34xx_isp_dbg_register_device(struct device *)
{
	return (0);
}

static inline void
omap34xx_isp_dbg_unregister_device(struct device *)
{}
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG
#endif // OMAP34XX_ISP_H
