#ifndef OMAP34XX_V4L2_H
#define OMAP34XX_V4L2_H

#include <linux/platform_device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-int-device.h>
#include "omap34xx-videobuf.h"

struct recursive_mutex {
	struct mutex		mutex;
	unsigned long		depth;
	struct task_struct	*owner;
};

#define recursive_mutex_init(lock) \
	do { \
		mutex_init(&(lock)->mutex); \
		(lock)->depth = 0; \
		(lock)->owner = NULL; \
	} while(0)

#define recursive_mutex_lock(lock) \
	do { \
		if (current != (lock)->owner) { \
			mutex_lock(&(lock)->mutex); \
			(lock)->owner = current; \
		} \
		++(lock)->depth; \
	} while(0)

#define recursive_mutex_unlock(lock) \
	do { \
		BUG_ON(current != (lock)->owner); \
		if (!(--(lock)->depth)) { \
			(lock)->owner = NULL; \
			mutex_unlock(&(lock)->mutex); \
		} \
	} while(0)

struct omap34xx_v4l2_int_master {
	void			*priv;
	struct recursive_mutex	lock;
	struct v4l2_int_device	dev;
	struct v4l2_int_master	mst;
	struct v4l2_int_device	*slv;
	struct platform_device	*parent;

	int (*ioctl_streamon)(struct omap34xx_v4l2_int_master *);
	int (*ioctl_streamoff)(struct omap34xx_v4l2_int_master *);
	int (*ioctl_s_ctrl)(struct v4l2_control *);
};

#define omap34xx_v4l2_int_master_lock(mst) \
	recursive_mutex_lock(&(mst)->lock)

#define omap34xx_v4l2_int_master_unlock(mst) \
	recursive_mutex_unlock(&(mst)->lock)

extern int
omap34xx_v4l2_int_master_register(struct omap34xx_v4l2_int_master *, char *);
extern int
omap34xx_v4l2_int_slave_ioctl(struct omap34xx_v4l2_int_master *, int, void *);

#define __omap34xx_v4l2_int_slave_ioctl(m, c, a) \
	omap34xx_v4l2_int_slave_ioctl(m, c, (void*)a)

#define omap34xx_v4l2_int_slave_ioctl_g_fmt(m, f) \
	__omap34xx_v4l2_int_slave_ioctl((m), vidioc_int_g_fmt_cap_num, (f))

#define omap34xx_v4l2_int_slave_ioctl_streamon(m) \
	__omap34xx_v4l2_int_slave_ioctl((m), vidioc_int_streamon_num, 0)

#define omap34xx_v4l2_int_slave_ioctl_streamoff(m) \
	__omap34xx_v4l2_int_slave_ioctl((m), vidioc_int_streamoff_num, 0)

#define omap34xx_v4l2_int_slave_ioctl_s_power_on(m) \
	__omap34xx_v4l2_int_slave_ioctl((m), vidioc_int_s_power_num, 1)

#define omap34xx_v4l2_int_slave_ioctl_s_power_off(m) \
	__omap34xx_v4l2_int_slave_ioctl((m), vidioc_int_s_power_num, 0)

extern int
omap34xx_v4l2_int_master_ioctl(struct omap34xx_v4l2_int_master *, int, void *);

#define __omap34xx_v4l2_int_master_ioctl(m, c, a) \
	omap34xx_v4l2_int_master_ioctl(m, c, (void*)a)

#define omap34xx_v4l2_int_master_ioctl_g_fmt(m, f) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_g_fmt_cap_num, (f))

#define omap34xx_v4l2_int_master_ioctl_s_fmt(m, f) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_s_fmt_cap_num, (f))

#define omap34xx_v4l2_int_master_ioctl_try_fmt(m, f) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_try_fmt_cap_num, (f))

#define omap34xx_v4l2_int_master_ioctl_g_ctrl(m, c) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_g_ctrl_num, (c))

#define omap34xx_v4l2_int_master_ioctl_s_ctrl(m, c) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_s_ctrl_num, (c))

#define omap34xx_v4l2_int_master_ioctl_g_parm(m, p) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_g_parm_num, (p))

#define omap34xx_v4l2_int_master_ioctl_s_parm(m, p) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_s_parm_num, (p))

#define omap34xx_v4l2_int_master_ioctl_streamon(m) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_streamon_num, 0)

#define omap34xx_v4l2_int_master_ioctl_streamoff(m) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_streamoff_num, 0)

#define omap34xx_v4l2_int_master_ioctl_g_ifparm(m, p) \
	__omap34xx_v4l2_int_master_ioctl((m), vidioc_int_g_ifparm_num, (p))

struct omap34xx_v4l2_device {
	void				*priv;
	spinlock_t			qlock;
	struct file			*owner;
	unsigned int			qsize;
	struct list_head		qbufs;
	enum v4l2_buf_type		qtype;
	struct video_device		dev;
	struct recursive_mutex		lock;
	struct platform_device		*parent;
	struct omap34xx_dma_pool	*pool;

	int (*file_open)(struct omap34xx_v4l2_device *);
	int (*file_release)(struct omap34xx_v4l2_device *);
	unsigned int (*videobuf_size)(struct omap34xx_v4l2_device *);
	size_t (*videobuf_align)(struct omap34xx_v4l2_device *);
	int (*videobuf_init)(struct omap34xx_v4l2_device *,
				struct videobuf_buffer *);
	void (*videobuf_start)(struct omap34xx_v4l2_device *,
				struct videobuf_buffer *);
	void (*videobuf_release)(struct omap34xx_v4l2_device *,
					struct videobuf_buffer *);
	int (*vidioc_querycap)(struct omap34xx_v4l2_device *,
				struct v4l2_capability *);
	int (*vidioc_enum_fmt)(struct omap34xx_v4l2_device *,
				struct v4l2_fmtdesc *);
	int (*vidioc_g_fmt)(struct omap34xx_v4l2_device *,
				struct v4l2_format *);
	int (*vidioc_s_fmt)(struct omap34xx_v4l2_device *,
				struct v4l2_format *);
	int (*vidioc_try_fmt)(struct omap34xx_v4l2_device *,
				struct v4l2_format *);
	int (*vidioc_enum_input)(struct omap34xx_v4l2_device *,
					struct v4l2_input *);
	int (*vidioc_g_input)(struct omap34xx_v4l2_device *,
				unsigned int *i);
	int (*vidioc_s_input)(struct omap34xx_v4l2_device *,
				unsigned int i);
	int (*vidioc_g_ctrl)(struct omap34xx_v4l2_device *,
				struct v4l2_control *);
	int (*vidioc_s_ctrl)(struct omap34xx_v4l2_device *,
				struct v4l2_control *);
	int (*vidioc_g_parm)(struct omap34xx_v4l2_device *,
				struct v4l2_streamparm *);
	int (*vidioc_s_parm)(struct omap34xx_v4l2_device *,
				struct v4l2_streamparm *);
	int (*vidioc_streamoff)(struct omap34xx_v4l2_device *);
	int (*vidioc_streamon)(struct omap34xx_v4l2_device *);
};

#define omap34xx_v4l2_device_lock(dev) \
	recursive_mutex_lock(&(dev)->lock)

#define omap34xx_v4l2_device_unlock(dev) \
	recursive_mutex_unlock(&(dev)->lock)

extern int omap34xx_v4l2_device_register(struct omap34xx_v4l2_device *);

extern int
omap34xx_v4l2_device_vidioc_streamoff(struct omap34xx_v4l2_device *);
extern void
omap34xx_v4l2_device_videobuf_done(struct omap34xx_v4l2_device *,
					enum videobuf_state);

#endif // OMAP34XX_V4L2_H
