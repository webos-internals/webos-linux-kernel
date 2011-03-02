/*
 * drivers/media/video/omap/omap34xx-v4l2.c
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

#include <linux/time.h>
#include "omap34xx-v4l2.h"

#define INVARIANT(cond)						\
	do {							\
		if (!(cond)) {					\
			printk(KERN_DEBUG "!(" # cond ")\n");	\
			BUG();					\
		}						\
	} while (0)

#define SPEW(level, args...)					\
	do {							\
		if (omap34xx_v4l2_spew_level >= level)		\
			printk(KERN_DEBUG "V4L2:\t" args);	\
	} while (0)

static int omap34xx_v4l2_spew_level = 0;

static int omap34xx_v4l2_master_attach(struct v4l2_int_device *master,
					struct v4l2_int_device *slave)
{
	int rc;
	struct omap34xx_v4l2_int_master *mst;

	mst = container_of(master, struct omap34xx_v4l2_int_master, dev);

	SPEW(1, "+++ %s: slave=%s\n", __func__, slave->name);

	omap34xx_v4l2_int_master_lock(mst);

	if (mst->slv) {
		rc = -EBUSY;
		goto unlock;
	}

	mst->slv = slave;
	rc = 0;
unlock:
	omap34xx_v4l2_int_master_unlock(mst);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static void omap34xx_v4l2_master_detach(struct v4l2_int_device *master)
{
	struct omap34xx_v4l2_int_master *mst;

	mst = container_of(master, struct omap34xx_v4l2_int_master, dev);

	SPEW(1, "+++ %s\n", __func__);

	omap34xx_v4l2_int_master_lock(mst);
	mst->slv = NULL;
	omap34xx_v4l2_int_master_unlock(mst);

	SPEW(1, "--- %s\n", __func__);
}

int omap34xx_v4l2_int_master_register(struct omap34xx_v4l2_int_master *mst,
					char *name)
{
	int rc;

	BUG_ON(!mst->ioctl_streamon);
	BUG_ON(!mst->ioctl_streamoff);

	recursive_mutex_init(&mst->lock);
	mst->mst.attach = omap34xx_v4l2_master_attach;
	mst->mst.detach = omap34xx_v4l2_master_detach;
	strncpy(mst->dev.name, name, V4L2NAMESIZE);
	mst->dev.type = v4l2_int_type_master;
	mst->dev.u.master = &mst->mst;

	if ((rc = v4l2_int_device_register(&mst->dev)))
		goto exit;

	printk(KERN_INFO "%s: registered v4l2 interface: name=%s\n",
		mst->parent->name, name);
exit:
	return (rc);
}
EXPORT_SYMBOL(omap34xx_v4l2_int_master_register);

int omap34xx_v4l2_int_slave_ioctl(struct omap34xx_v4l2_int_master *mst,
					int cmd, void *arg)
{
	int rc = -ENODEV;

	SPEW(3, "+++ %s: cmd=%d\n", __func__, cmd);

	omap34xx_v4l2_int_master_lock(mst);

	if (mst->slv)
		rc = v4l2_int_ioctl_1(mst->slv, cmd, arg);

	omap34xx_v4l2_int_master_unlock(mst);

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_v4l2_int_slave_ioctl);

int omap34xx_v4l2_int_master_ioctl(struct omap34xx_v4l2_int_master *mst,
					int cmd, void *arg)
{
	int rc = -ENODEV;

	SPEW(3, "+++ %s\n", __func__);

	omap34xx_v4l2_int_master_lock(mst);

	switch (cmd) {
	case vidioc_int_streamon_num:
		if (mst->ioctl_streamon) {
			rc = mst->ioctl_streamon(mst);
			goto unlock;
		}
		break;
	case vidioc_int_streamoff_num:
		if (mst->ioctl_streamoff) {
			rc = mst->ioctl_streamoff(mst);
			goto unlock;
		}
		break;
	}

	if (mst->slv)
		rc = v4l2_int_ioctl_1(mst->slv, cmd, arg);
unlock:
	omap34xx_v4l2_int_master_unlock(mst);

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_v4l2_int_master_ioctl);

static int omap34xx_v4l2_videobuf_setup(struct videobuf_queue *q,
					unsigned int *count,
					unsigned int *size)
{
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: count=%u size=%u\n", __func__, *count, *size);

	*size = dev->videobuf_size(dev);

	SPEW(1, "--- %s: count=%u size=%u\n", __func__, *count, *size);

	return (0);
}

static int omap34xx_v4l2_videobuf_init(struct videobuf_queue *q,
					struct videobuf_buffer *buf)
{
	int rc;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: index=%u bsize=%u baddr=0x%08lX\n", __func__, buf->i,
		buf->bsize, buf->baddr);

	if (buf->baddr & (dev->videobuf_align(dev) - 1)) {
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = omap34xx_videobuf_init(q, buf)))
		goto exit;

	if ((rc = dev->videobuf_init(dev, buf))) {
		omap34xx_videobuf_release(q, buf);
		goto exit;
	}

	buf->size = dev->videobuf_size(dev);
exit:
	SPEW(1, "--- %s: rc=%d size=%lu\n", __func__, rc, buf->size);

	return (rc);
}

static int omap34xx_v4l2_videobuf_prepare(struct videobuf_queue *q,
						struct videobuf_buffer *buf,
						enum v4l2_field field)
{
	int rc;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(3, "+++ %s: index=%u state=%d\n", __func__, buf->i, buf->state);

	if (buf->bsize < dev->videobuf_size(dev)) {
		rc = -EINVAL;
		goto exit;
	}

	if ((STATE_NEEDS_INIT == buf->state)
		&& (rc = omap34xx_v4l2_videobuf_init(q, buf)))
		goto exit;

	buf->state = STATE_PREPARED;
	rc = 0;
exit:
	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static void omap34xx_v4l2_videobuf_queue(struct videobuf_queue *q,
						struct videobuf_buffer *buf)
{
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(3, "+++ %s: index=%u\n", __func__, buf->i);

	/* NOTE: called when streaming is enabled with the queue spin lock held */
	buf->state = STATE_QUEUED;

	if (list_empty(&dev->qbufs)) {
		buf->state = STATE_ACTIVE;
		dev->videobuf_start(dev, buf);
	}

	list_add_tail(&buf->queue, &dev->qbufs);

	SPEW(3, "--- %s\n", __func__);
}

static void omap34xx_v4l2_videobuf_release(struct videobuf_queue *q,
						struct videobuf_buffer *buf)
{
	unsigned long flags;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: index=%u state=%d\n", __func__, buf->i, buf->state);

	spin_lock_irqsave(&dev->qlock, flags);

	/* NOTE: This case occurs when a process exits during streaming. */
	if (STATE_QUEUED == buf->state
		|| STATE_ACTIVE == buf->state) {
		spin_unlock_irqrestore(&dev->qlock, flags);
		goto exit;
	}

	spin_unlock_irqrestore(&dev->qlock, flags);

	if (STATE_NEEDS_INIT != buf->state) {
		dev->videobuf_release(dev, buf);
		omap34xx_videobuf_release(q, buf);
		buf->state = STATE_NEEDS_INIT;
	}
exit:
	SPEW(1, "--- %s\n", __func__);
}

static struct videobuf_queue_ops omap34xx_v4l2_device_videobuf_qops = {
	.buf_setup = omap34xx_v4l2_videobuf_setup,
	.buf_prepare = omap34xx_v4l2_videobuf_prepare,
	.buf_queue = omap34xx_v4l2_videobuf_queue,
	.buf_release = omap34xx_v4l2_videobuf_release,
};

static ssize_t omap34xx_v4l2_file_read(struct file *file, char __user *buf,
					size_t len, loff_t *off)
{
	ssize_t rc;
	unsigned long flags;
	struct videobuf_queue *q = file->private_data;
	struct videobuf_buffer *qbuf;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: buf=%p len=%u\n", __func__, buf, len);

	omap34xx_v4l2_device_lock(dev);

	if (dev->owner) {
		rc = -EBUSY;
		goto unlock;
	}

	if (!(qbuf = videobuf_alloc(q))) {
		rc = -ENOMEM;
		goto unlock;
	}

	qbuf->memory = V4L2_MEMORY_USERPTR;
	qbuf->baddr = (unsigned long)buf;
	qbuf->bsize = len;

	if ((rc = omap34xx_v4l2_videobuf_prepare(q, qbuf, q->field)))
		goto cleanup;

	if ((rc = dev->vidioc_streamon(dev)))
		goto cleanup;

	spin_lock_irqsave(&dev->qlock, flags);
	omap34xx_v4l2_videobuf_queue(q, qbuf);
	spin_unlock_irqrestore(&dev->qlock, flags);
	dev->owner = file;
	omap34xx_v4l2_device_unlock(dev);
	rc = videobuf_waiton(qbuf, 0, 1);
	omap34xx_v4l2_device_lock(dev);
	(void)dev->vidioc_streamoff(dev);

	if (rc)
		goto unown;

	if (STATE_ERROR == qbuf->state) {
		rc = -EIO;
		goto unown;
	}

	if (q->int_ops->sync)
		q->int_ops->sync(q, qbuf);

	rc = dev->videobuf_size(dev);

unown:
	dev->owner = NULL;
cleanup:
	omap34xx_v4l2_videobuf_release(q, qbuf);
	kfree(qbuf);
unlock:
	omap34xx_v4l2_device_unlock(dev);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_file_mmap(struct file *file,
					struct vm_area_struct *vma)
{
	int rc;
	struct videobuf_queue *q = file->private_data;

	SPEW(1, "+++ %s\n", __func__);

	rc = videobuf_mmap_mapper(q, vma);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_file_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct video_device *vdev = video_devdata(file);
	struct videobuf_queue *q;
	struct omap34xx_v4l2_device *dev;

	dev = container_of(vdev, struct omap34xx_v4l2_device, dev);

	SPEW(1, "+++ %s\n", __func__);

	if (!(q = kzalloc(sizeof(*q), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	omap34xx_videobuf_queue_init(q, &omap34xx_v4l2_device_videobuf_qops,
					dev->pool, &dev->qlock, dev->qtype,
					V4L2_FIELD_NONE, dev->qsize, dev);

	file->private_data = q;

	if (dev->file_open) {
		omap34xx_v4l2_device_lock(dev);

		if ((rc = dev->file_open(dev)))
			kfree(q);

		omap34xx_v4l2_device_unlock(dev);
	}
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static unsigned int omap34xx_v4l2_file_poll(struct file *file,
						struct poll_table_struct *wait)
{
	int rc;
	struct videobuf_queue *q = file->private_data;

	SPEW(3, "+++ %s\n", __func__);

	rc = videobuf_poll_stream(file, q, wait);

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return rc;
}

static int omap34xx_v4l2_vidioc_streamoff_locked(
		struct omap34xx_v4l2_device *dev,
		struct file *file
		)
{
	int rc = 0;
	struct videobuf_queue *q;

	SPEW(1, "+++ %s\n", __func__);

	if (!dev->owner)
		goto exit;

	if (file != dev->owner) {
		rc = -EINVAL;
		goto exit;
	}

	q = dev->owner->private_data;

	videobuf_queue_cancel(q);
	(void)dev->vidioc_streamoff(dev);
	BUG_ON(!list_empty(&dev->qbufs));
	(void)videobuf_streamoff(q);

	dev->owner = NULL;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_file_release(struct inode *inode, struct file *file)
{
	int rc = 0;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s\n", __func__);

	omap34xx_v4l2_device_lock(dev);

	omap34xx_v4l2_vidioc_streamoff_locked(dev, file);
	rc = videobuf_mmap_free(q);
	BUG_ON(rc);
	kfree(q);

	if (dev->file_release)
		rc = dev->file_release(dev);

	omap34xx_v4l2_device_unlock(dev);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct file_operations omap34xx_v4l2_device_fops = {
	.owner = THIS_MODULE,
	/* TODO: no_llseek? */
	.read = omap34xx_v4l2_file_read,
	.ioctl = video_ioctl2,
	.mmap = omap34xx_v4l2_file_mmap,
	.poll = omap34xx_v4l2_file_poll,
	.open = omap34xx_v4l2_file_open,
	.release = omap34xx_v4l2_file_release,
};

static void omap34xx_v4l2_release(struct video_device *vfd)
{
}

static int omap34xx_v4l2_vidioc_querycap(struct file *file, void *notused,
						struct v4l2_capability *cap)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s\n", __func__);

	if (dev->vidioc_querycap) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_querycap(dev, cap);
		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_enum_fmt(struct file *file, void *notused,
						struct v4l2_fmtdesc *desc)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s\n", __func__);

	if (dev->vidioc_enum_fmt) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_enum_fmt(dev, desc);
		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_g_fmt(struct file *file, void *notused,
					struct v4l2_format *fmt)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: type=%d\n", __func__, fmt->type);

	if (fmt->type != q->type)
		goto exit;

	if (dev->vidioc_g_fmt) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_g_fmt(dev, fmt);
		omap34xx_v4l2_device_unlock(dev);
	}
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_s_fmt(struct file *file, void *notused,
					struct v4l2_format *fmt)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: type=%d\n", __func__, fmt->type);

	if (fmt->type != q->type)
		goto exit;

	if (dev->vidioc_s_fmt) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_s_fmt(dev, fmt);
		omap34xx_v4l2_device_unlock(dev);
	}
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_try_fmt(struct file *file, void *notused,
					struct v4l2_format *fmt)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: type=%d\n", __func__, fmt->type);

	if (fmt->type != q->type)
		goto exit;

	if (dev->vidioc_try_fmt) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_try_fmt(dev, fmt);
		omap34xx_v4l2_device_unlock(dev);
	}
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_reqbufs(struct file *file, void *notused,
					struct v4l2_requestbuffers *req)
{
	int rc;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: count=%u type=%d memory=%d\n", __func__, req->count,
		req->type, req->memory);

	omap34xx_v4l2_device_lock(dev);
	rc = videobuf_reqbufs(q, req);
	omap34xx_v4l2_device_unlock(dev);

	SPEW(1, "--- %s: rc=%d count=%u\n", __func__, rc, req->count);

	return rc < 0 ? rc : 0;
}

extern int omap34xx_v4l2_vidioc_querybuf(struct file *file, void *notused,
						struct v4l2_buffer *buf)
{
	int rc;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: index=%u type=%d\n", __func__, buf->index, buf->type);

	omap34xx_v4l2_device_lock(dev);
	rc = videobuf_querybuf(q, buf);
	omap34xx_v4l2_device_unlock(dev);

	SPEW(1, "--- %s: rc=%d flags=0x%08X memory=%d m.offset=%u"
		" m.userptr=0x%08lX length=%u\n", __func__, rc, buf->flags,
		buf->memory, buf->m.offset, buf->m.userptr, buf->length);

	return (rc);
}

int omap34xx_v4l2_vidioc_qbuf(struct file *file, void *notused,
				struct v4l2_buffer *buf)
{
	int rc;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(3, "+++ %s: index=%u type=%d\n", __func__, buf->index, buf->type);

	omap34xx_v4l2_device_lock(dev);
	rc = videobuf_qbuf(q, buf);
	omap34xx_v4l2_device_unlock(dev);

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

int omap34xx_v4l2_vidioc_dqbuf(struct file *file, void *notused,
				struct v4l2_buffer *buf)
{
	int rc;
	struct videobuf_queue *q = file->private_data;

	SPEW(3, "+++ %s: index=%u type=%d\n", __func__, buf->index, buf->type);

	rc = videobuf_dqbuf(q, buf, O_NONBLOCK & file->f_flags);

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_streamon(struct file *file, void *notused,
						enum v4l2_buf_type type)
{
	int rc;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: type=%d\n", __func__, type);

	if (type != q->type) {
		rc = -EINVAL;
		goto exit;
	}

	omap34xx_v4l2_device_lock(dev);

	if (dev->owner) {
		rc = -EBUSY;
		goto unlock;
	}
	if ((rc = dev->vidioc_streamon(dev)))
		goto unlock;

	if ((rc = videobuf_streamon(q)))
		goto unlock;

	dev->owner = file;
unlock:
	omap34xx_v4l2_device_unlock(dev);
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_streamoff(struct file *file, void *notused,
						enum v4l2_buf_type type)
{
	int rc;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: type=%d\n", __func__, type);

	if (type != q->type) {
		rc = -EINVAL;
		goto exit;
	}

	omap34xx_v4l2_device_lock(dev);
	rc = omap34xx_v4l2_vidioc_streamoff_locked(dev, file);
	omap34xx_v4l2_device_unlock(dev);
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

int omap34xx_v4l2_device_vidioc_streamoff(struct omap34xx_v4l2_device *dev)
{
	int rc;

	SPEW(1, "+++ %s\n", __func__);

	omap34xx_v4l2_device_lock(dev);
	rc = omap34xx_v4l2_vidioc_streamoff_locked(dev, dev->owner);
	omap34xx_v4l2_device_unlock(dev);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_v4l2_device_vidioc_streamoff);

static int omap34xx_v4l2_vidioc_enum_input(struct file *file, void *notused,
						struct v4l2_input *input)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: index=%u\n", __func__, input->index);

	if (dev->vidioc_enum_input) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_enum_input(dev, input);
		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_g_input(struct file *file, void *notused,
					unsigned int *i)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s\n", __func__);

	if (dev->vidioc_g_input) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_g_input(dev, i);
		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(1, "--- %s: rc=%d i=%u\n", __func__, rc, *i);

	return (rc);
}

static int omap34xx_v4l2_vidioc_s_input(struct file *file, void *notused,
					unsigned int i)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: i=%u\n", __func__, i);

	if (dev->vidioc_s_input) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_s_input(dev, i);
		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_g_ctrl(struct file *file, void *notused,
					struct v4l2_control *ctl)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(3, "+++ %s: id=0x%08X\n", __func__, ctl->id);

	if (dev->vidioc_g_ctrl) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_g_ctrl(dev, ctl);
		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_s_ctrl(struct file *file, void *notused,
					struct v4l2_control *ctl)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(3, "+++ %s: id=0x%08X\n", __func__, ctl->id);

	if (dev->vidioc_s_ctrl) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_s_ctrl(dev, ctl);
		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_g_ext_ctrls(struct file *file, void *notused,
						struct v4l2_ext_controls *ctls)
{
	int i;
	int rc = -EINVAL;
	struct v4l2_control ctl;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(3, "+++ %s: count=%u\n", __func__, ctls->count);

	if (dev->vidioc_g_ctrl) {
		omap34xx_v4l2_device_lock(dev);

		for (i = 0; i < ctls->count; ++i) {
			ctl.id = ctls->controls[i].id;

			if ((rc = dev->vidioc_g_ctrl(dev, &ctl)))
				break;

			ctls->controls[i].value = ctl.value;
		}

		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_s_ext_ctrls(struct file *file, void *notused,
						struct v4l2_ext_controls *ctls)
{
	int i;
	int rc = -EINVAL;
	struct v4l2_control ctl;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(3, "+++ %s: count=%u\n", __func__, ctls->count);

	if (dev->vidioc_s_ctrl) {
		omap34xx_v4l2_device_lock(dev);

#ifdef CONFIG_VIDEO_VX6852_I2C_LOG
		printk("vx6852-frame-cfg-start\n");
#endif
		for (i = 0; i < ctls->count; ++i) {
			ctl.id = ctls->controls[i].id;
			ctl.value = ctls->controls[i].value;

			if ((rc = dev->vidioc_s_ctrl(dev, &ctl)))
				break;
		}

		omap34xx_v4l2_device_unlock(dev);
	}

	SPEW(3, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_g_parm(struct file *file, void *notused,
					struct v4l2_streamparm *parm)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: type=%d\n", __func__, parm->type);

	if (parm->type != q->type)
		goto exit;

	if (dev->vidioc_g_parm) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_g_parm(dev, parm);
		omap34xx_v4l2_device_unlock(dev);
	}
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static int omap34xx_v4l2_vidioc_s_parm(struct file *file, void *notused,
					struct v4l2_streamparm *parm)
{
	int rc = -EINVAL;
	struct videobuf_queue *q = file->private_data;
	struct omap34xx_v4l2_device *dev = q->priv_data;

	SPEW(1, "+++ %s: type=%d\n", __func__, parm->type);

	if (parm->type != q->type)
		goto exit;

	if (dev->vidioc_s_parm) {
		omap34xx_v4l2_device_lock(dev);
		rc = dev->vidioc_s_parm(dev, parm);
		omap34xx_v4l2_device_unlock(dev);
	}
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

int omap34xx_v4l2_device_register(struct omap34xx_v4l2_device *dev)
{
	int rc;

	BUG_ON(!dev->vidioc_streamon);
	BUG_ON(!dev->vidioc_streamoff);

	recursive_mutex_init(&dev->lock);
	spin_lock_init(&dev->qlock);
	INIT_LIST_HEAD(&dev->qbufs);

	dev->dev.fops = &omap34xx_v4l2_device_fops;
	dev->dev.release = omap34xx_v4l2_release;
	dev->dev.vidioc_querycap = omap34xx_v4l2_vidioc_querycap;
	dev->dev.vidioc_enum_fmt_cap = omap34xx_v4l2_vidioc_enum_fmt;
	dev->dev.vidioc_g_fmt_cap = omap34xx_v4l2_vidioc_g_fmt;
	dev->dev.vidioc_g_fmt_type_private = omap34xx_v4l2_vidioc_g_fmt;
	dev->dev.vidioc_s_fmt_cap = omap34xx_v4l2_vidioc_s_fmt;
	dev->dev.vidioc_s_fmt_type_private = omap34xx_v4l2_vidioc_s_fmt;
	dev->dev.vidioc_try_fmt_cap = omap34xx_v4l2_vidioc_try_fmt;
	dev->dev.vidioc_try_fmt_type_private = omap34xx_v4l2_vidioc_try_fmt;
	dev->dev.vidioc_reqbufs = omap34xx_v4l2_vidioc_reqbufs;
	dev->dev.vidioc_querybuf = omap34xx_v4l2_vidioc_querybuf;
	dev->dev.vidioc_qbuf = omap34xx_v4l2_vidioc_qbuf;
	dev->dev.vidioc_dqbuf = omap34xx_v4l2_vidioc_dqbuf;
	dev->dev.vidioc_streamon = omap34xx_v4l2_vidioc_streamon;
	dev->dev.vidioc_streamoff = omap34xx_v4l2_vidioc_streamoff;
	dev->dev.vidioc_enum_input = omap34xx_v4l2_vidioc_enum_input;
	dev->dev.vidioc_g_input = omap34xx_v4l2_vidioc_g_input;
	dev->dev.vidioc_s_input = omap34xx_v4l2_vidioc_s_input;
	dev->dev.vidioc_g_ctrl = omap34xx_v4l2_vidioc_g_ctrl;
	dev->dev.vidioc_s_ctrl = omap34xx_v4l2_vidioc_s_ctrl;
	dev->dev.vidioc_g_ext_ctrls = omap34xx_v4l2_vidioc_g_ext_ctrls;
	dev->dev.vidioc_s_ext_ctrls = omap34xx_v4l2_vidioc_s_ext_ctrls;
	dev->dev.vidioc_g_parm = omap34xx_v4l2_vidioc_g_parm;
	dev->dev.vidioc_s_parm = omap34xx_v4l2_vidioc_s_parm;

	if ((rc = video_register_device(&dev->dev, VFL_TYPE_GRABBER, -1)))
		goto exit;

	printk(KERN_INFO "%s: registered v4l2 device: type=%d minor=%d\n",
		dev->parent->name, dev->qtype, dev->dev.minor);
exit:
	return (rc);
}
EXPORT_SYMBOL(omap34xx_v4l2_device_register);

void omap34xx_v4l2_device_videobuf_done(struct omap34xx_v4l2_device *dev,
					enum videobuf_state state)
{
	struct list_head *entry;
	struct videobuf_buffer *buf;
	struct timespec tspec;
	u32 flags;

	SPEW(3, "+++ %s: state=%d\n", __func__, state);
	ktime_get_ts(&tspec); // get monotonic time


	spin_lock_irqsave(&dev->qlock,flags);
	if( list_empty(&dev->qbufs)) {
		/* list was emptied by another isr call to this function.*/
		goto exit;  /* safe to ignore.  list was resolved. */
	}

	buf = list_entry(dev->qbufs.next, struct videobuf_buffer, queue);
	INVARIANT(STATE_ACTIVE == buf->state);

	// convert to timeval
	buf->ts.tv_sec = tspec.tv_sec;
	buf->ts.tv_usec = tspec.tv_nsec / 1000;

	buf->state = state;
	wake_up(&buf->done);
	list_del(&buf->queue);

	if (!list_empty(&dev->qbufs)) {
		entry = dev->qbufs.next;
		buf = list_entry(entry, struct videobuf_buffer, queue);
		buf->state = STATE_ACTIVE;
		dev->videobuf_start(dev, buf);
	}

exit:
	spin_unlock_irqrestore(&dev->qlock,flags);

	SPEW(3, "--- %s\n", __func__);
}
EXPORT_SYMBOL(omap34xx_v4l2_device_videobuf_done);
