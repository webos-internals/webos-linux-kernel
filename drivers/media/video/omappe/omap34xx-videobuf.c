/*
 * drivers/media/video/omappe/omap34xx-videobuf.c
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

#include "omap34xx-videobuf.h"
#include <linux/dma-mapping.h>

#define SPEW_LEVEL	0

#define SPEW(level, args...)						\
	do {								\
		if ((level) <= SPEW_LEVEL)				\
			printk(KERN_DEBUG "omap34xx-videobuf: " args);	\
	} while (0)

static void * omap34xx_videobuf_alloc(size_t msize)
{
	size_t size = msize + sizeof(struct omap34xx_videobuf);
	struct videobuf_buffer *buf;

	if (!(buf = kzalloc(size, GFP_KERNEL)))
		goto exit;

	buf->priv = (void *)buf + msize;
exit:
	return (buf);
}

static inline struct omap34xx_videobuf * __priv(struct videobuf_buffer *buf)
{
	return (buf->priv);
}

static int omap34xx_videobuf_sync(struct videobuf_queue *q,
					struct videobuf_buffer *buf)
{
	if (__priv(buf)->sync)
		__priv(buf)->sync(buf);

	return (0);
}

static void omap34xx_videobuf_sync_single(struct videobuf_buffer *buf)
{
	dma_sync_single_for_cpu(NULL, __priv(buf)->dev_addr, buf->bsize,
				DMA_FROM_DEVICE);
}

static void omap34xx_videobuf_sync_sg(struct videobuf_buffer *buf)
{
	dma_sync_sg_for_cpu(NULL, __priv(buf)->sg_list,
				__priv(buf)->nr_pages, DMA_FROM_DEVICE);
}

static const char * omap34xx_videobuf_spew_sync(omap34xx_videobuf_sync_t sync)
{
	const char *str;

	if (sync == omap34xx_videobuf_sync_single)
		str = "single";

	else if (sync == omap34xx_videobuf_sync_sg)
		str = "sg";

	else
		str = "none";

	return (str);
}

static int omap34xx_videobuf_init_mmap(struct videobuf_queue *q,
					struct videobuf_buffer *buf)
{
	int rc;
	struct omap34xx_dma_pool *pool = q->dev;

	if (!buf->map) {
		rc = -EINVAL;
		goto exit;
	}

	__priv(buf)->dev_addr = __priv(buf)->block->dev_addr;
	__priv(buf)->sync = pool->sync;

	rc = 0;
exit:
	return (rc);
}

static int omap34xx_videobuf_init_userptr(struct videobuf_queue *q,
						struct videobuf_buffer *buf)
{
	int i;
	int rc;
	int nr_pages;
	struct page **pages;
	struct scatterlist *sg_list;
	struct vm_area_struct *vma;

	down_read(&current->mm->mmap_sem);

	if (buf->baddr != PAGE_ALIGN(buf->baddr)) {
		rc = -EINVAL;
		goto exit;
	}
	if (!(vma = find_vma(current->mm, buf->baddr))
		|| (vma->vm_start > buf->baddr)
		|| (vma->vm_end < (buf->baddr + buf->bsize))) {
		rc = -EINVAL;
		goto exit;
	}

	BUG_ON(__priv(buf)->pages);

	nr_pages = buf->bsize >> PAGE_SHIFT;

	if (!(pages = kcalloc(nr_pages, sizeof(*pages), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	if (!(sg_list = kcalloc(nr_pages, sizeof(*sg_list), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto kcalloc_sg_list_failed;
	}

	if ((rc = get_user_pages(current, current->mm, buf->baddr, nr_pages,
					1, 0, pages, NULL)) < 0)
		goto get_user_pages_failed;

	if (rc < nr_pages) {
		rc = -EFAULT;
		goto get_user_pages_failed;
	}

	sg_init_table(sg_list, nr_pages);

	for (i = 0; i < nr_pages; i++) {
		sg_set_page(&sg_list[i], pages[i], PAGE_SIZE, 0);
		sg_dma_address(&sg_list[i]) = page_to_dma(NULL, pages[i]);
	}

	__priv(buf)->nr_pages = nr_pages;
	__priv(buf)->pages = pages;
	__priv(buf)->sg_list = sg_list;

	if (vma->vm_page_prot & L_PTE_CACHEABLE)
		__priv(buf)->sync = omap34xx_videobuf_sync_sg;

	rc = 0;
	goto exit;

get_user_pages_failed:
	while (nr_pages--) {
		if (pages[nr_pages])
			put_page(pages[nr_pages]);
	}

	kfree(sg_list);

kcalloc_sg_list_failed:
	kfree(pages);
exit:
	up_read(&current->mm->mmap_sem);

	return (rc);
}

int omap34xx_videobuf_init(struct videobuf_queue *q,
				struct videobuf_buffer *buf)
{
	int rc;

	switch (buf->memory) {
	case V4L2_MEMORY_MMAP:
		if ((rc = omap34xx_videobuf_init_mmap(q, buf)))
			goto exit;

		break;
	case V4L2_MEMORY_USERPTR:
		if ((rc = omap34xx_videobuf_init_userptr(q, buf)))
			goto exit;

		break;
	case V4L2_MEMORY_OVERLAY:
		rc = -EINVAL;
		goto exit;
	default:
		BUG();
	}

	rc = 0;
exit:
	SPEW(1, "%s: rc=%d memory=%d baddr=0x%08lX bsize=%d dev_addr=0x%08X"
		" nr_pages=%d sync=%s\n", __func__, rc, buf->memory,
		buf->baddr, buf->bsize, __priv(buf)->dev_addr,
		__priv(buf)->nr_pages,
		omap34xx_videobuf_spew_sync(__priv(buf)->sync));

	return (rc);
}
EXPORT_SYMBOL(omap34xx_videobuf_init);

void omap34xx_videobuf_release(struct videobuf_queue *q,
				struct videobuf_buffer *buf)
{
	int i;

	switch (buf->memory) {
	case V4L2_MEMORY_MMAP:
		break;
	case V4L2_MEMORY_USERPTR:
		if (__priv(buf)->pages) {
			for (i = 0; i < __priv(buf)->nr_pages; i++)
				put_page(__priv(buf)->pages[i]);

			kfree(__priv(buf)->pages);
			kfree(__priv(buf)->sg_list);

			__priv(buf)->nr_pages = 0;
			__priv(buf)->pages = NULL;
			__priv(buf)->sg_list = NULL;
		}

		break;
	case V4L2_MEMORY_OVERLAY:
		break;
	default:
		BUG();
	}

	__priv(buf)->dev_addr = 0;
	__priv(buf)->sync = NULL;
}
EXPORT_SYMBOL(omap34xx_videobuf_release);

#define SPEW_FREE_LIST(l, p)						\
	do {								\
		struct omap34xx_dma_block *b;				\
									\
		if ((l) > SPEW_LEVEL)					\
			break;						\
									\
		list_for_each_entry(b, &(p)->free_list, free_list) {	\
			SPEW(l, "block=%p size=%u cpu=%p dev=0x%08X\n", \
				b, b->size, b->cpu_addr, b->dev_addr);	\
		}							\
	} while(0)

static struct omap34xx_dma_block * omap34xx_dma_block_alloc(
					struct omap34xx_dma_pool *pool,
					size_t size
					)
{
	struct omap34xx_dma_block *free;
	struct omap34xx_dma_block *used = NULL;

	list_for_each_entry(free, &pool->free_list, free_list) {
		if (PAGE_ALIGN(size) <= free->size) {
			if (!(used = kzalloc(sizeof(*used), GFP_KERNEL)))
				goto exit;

			used->size = PAGE_ALIGN(size);
			used->cpu_addr = free->cpu_addr;
			used->dev_addr = free->dev_addr;
			free->size -= PAGE_ALIGN(size);
			free->cpu_addr += PAGE_ALIGN(size);
			free->dev_addr += PAGE_ALIGN(size);
			break;
		}
	}
exit:
	SPEW(1, "%s: block=%p (size=%u cpu=%p dev=0x%08X)\n", __func__,
		used, used ? used->size : 0, used ? used->cpu_addr : NULL,
		used ? used->dev_addr : 0);

	SPEW_FREE_LIST(2, pool);

	return (used);
}

static void omap34xx_dma_block_free(struct omap34xx_dma_pool *pool,
					struct omap34xx_dma_block *block)
{
	struct list_head *pos;
	struct omap34xx_dma_block *free;

	SPEW(1, "%s: block=%p size=%u cpu=%p dev=0x%08X\n", __func__, block,
		block->size, block->cpu_addr, block->dev_addr);

	list_for_each(pos, &pool->free_list) {
		free = list_entry(pos, struct omap34xx_dma_block, free_list);

		if ((block->cpu_addr + block->size) < free->cpu_addr) {
			list_add_tail(&block->free_list, &free->free_list);
			break;
		}

		else if ((block->cpu_addr + block->size) == free->cpu_addr) {
			free->size += block->size;
			free->cpu_addr = block->cpu_addr;
			free->dev_addr = block->dev_addr;
			kfree(block);
			break;
		}

		else if ((free->cpu_addr + free->size) == block->cpu_addr) {
			free->size += block->size;
			kfree(block);
			pos = free->free_list.prev;
			list_del(&free->free_list);
			block = free;
		}
	}

	SPEW_FREE_LIST(2, pool);
}

static int omap34xx_videobuf_mmap_free(struct videobuf_queue *q)
{
	int i;
	int rc;
	struct omap34xx_dma_pool *pool;
	struct omap34xx_dma_block *block;

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (q->bufs[i] && q->bufs[i]->map) {
			rc = -EBUSY;
			goto exit;
		}
	}

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if (q->bufs[i] && (block = __priv(q->bufs[i])->block)) {
			pool = q->dev;
			omap34xx_dma_block_free(pool, block);
			__priv(q->bufs[i])->block = NULL;
		}
	}

	rc = 0;
exit:
	return (rc);
}

static void omap34xx_videobuf_vm_open(struct vm_area_struct *vma)
{
	struct videobuf_mapping *map = vma->vm_private_data;

	SPEW(1, "%s: pgoff=%lu start=%lu end=%lu\n", __func__, vma->vm_pgoff,
		vma->vm_start, vma->vm_end);

	map->count++;
}

static void omap34xx_videobuf_vm_close(struct vm_area_struct *vma)
{
	int i;
	struct videobuf_queue *q;
	struct videobuf_mapping *map = vma->vm_private_data;

	SPEW(1, "%s: pgoff=%lu start=0x%08lX end=0x%08lX\n", __func__,
		vma->vm_pgoff, vma->vm_start, vma->vm_end);

	if (!(--map->count)) {
		q = map->q;
		mutex_lock(&q->lock);

		for (i = 0; i < VIDEO_MAX_FRAME; i++) {
			if (q->bufs[i] && (q->bufs[i]->map == map)) {
				q->bufs[i]->map = NULL;
				q->bufs[i]->baddr = 0;
				break;
			}
		}

		mutex_unlock(&q->lock);
		kfree(map);
	}
}

static struct vm_operations_struct omap34xx_videobuf_vm_ops = {
	.open = omap34xx_videobuf_vm_open,
	.close = omap34xx_videobuf_vm_close,
};

static int omap34xx_videobuf_mmap_mapper(struct videobuf_queue *q,
						struct vm_area_struct *vma)
{
	int i;
	int rc;
	struct videobuf_buffer *buf;
	struct videobuf_mapping *map;
	struct omap34xx_dma_pool *pool = q->dev;
	struct omap34xx_dma_block *block;

	for (i = 0; i < VIDEO_MAX_FRAME; i++) {
		if ((buf = q->bufs[i])
			&& (buf->memory == V4L2_MEMORY_MMAP)
			&& (buf->boff == (vma->vm_pgoff << PAGE_SHIFT))
			/* NOTE: forces 1-to-1 mapping */
			&& (buf->bsize == (vma->vm_end - vma->vm_start)))
			break;
	}

	if (!buf) {
		rc = -EINVAL;
		goto exit;
	}

	if (buf->map) {
		rc = -EBUSY;
		goto exit;
	}

	BUG_ON(__priv(buf)->block);

	if (!(block = omap34xx_dma_block_alloc(pool, buf->bsize))) {
		rc = -ENOMEM;
		goto exit;
	}

	if (!(map = kzalloc(sizeof(*map), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto kzalloc_failed;
	}

	if ((rc = pool->mmap(block, vma)))
		goto mmap_failed;

	buf->baddr = vma->vm_start;
	buf->map = map;
	buf->map->count = 1;
	buf->map->start = vma->vm_start;
	buf->map->end = vma->vm_end;
	buf->map->q = q;
	__priv(buf)->block = block;
	vma->vm_ops = &omap34xx_videobuf_vm_ops;
	vma->vm_flags |= VM_DONTEXPAND;
	vma->vm_private_data = buf->map;

	rc = 0;
	goto exit;

mmap_failed:
	kfree(map);
kzalloc_failed:
	omap34xx_dma_block_free(pool, block);
exit:
	SPEW(1, "%s: rc=%d pgoff=%lu start=0x%08lX end=0x%08lX\n", __func__,
		rc, vma->vm_pgoff, vma->vm_start, vma->vm_end);

	return (rc);
}

struct omap34xx_videobuf * get_omap34xx_buf(struct videobuf_buffer *buf)
{
	return (__priv(buf));
}
EXPORT_SYMBOL(get_omap34xx_buf);

static struct videobuf_qtype_ops omap34xx_videobuf_ops = {
	.magic = MAGIC_QTYPE_OPS,
	.alloc = omap34xx_videobuf_alloc,
	.sync = omap34xx_videobuf_sync,
	.mmap_free = omap34xx_videobuf_mmap_free,
	.mmap_mapper = omap34xx_videobuf_mmap_mapper,
};

void omap34xx_videobuf_queue_init(struct videobuf_queue *q,
					struct videobuf_queue_ops *ops,
					struct omap34xx_dma_pool *pool,
					spinlock_t *lock,
					enum v4l2_buf_type type,
					enum v4l2_field field,
					unsigned int size, void *priv)
{
	BUG_ON(!pool);

	videobuf_queue_core_init(q, ops, pool, lock, type, field, size, priv,
					&omap34xx_videobuf_ops);
}
EXPORT_SYMBOL(omap34xx_videobuf_queue_init);

static int omap34xx_dma_block_mmap(struct omap34xx_dma_block *block,
					struct vm_area_struct *vma)
{
	int rc;

	rc = remap_pfn_range(vma, vma->vm_start, block->dev_addr >> PAGE_SHIFT,
				block->size, vma->vm_page_prot);

	SPEW(1, "%s: start=0x%08lX size=%u cpu=%p dev=0x%08X rc=%d\n",
		__func__, vma->vm_start, block->size, block->cpu_addr,
		block->dev_addr, rc);

	return (rc);
}

static void omap34xx_dma_pool_deinit(struct omap34xx_dma_pool *pool)
{
	struct list_head *item;
	struct omap34xx_dma_block *block;

	while (!list_empty(&pool->free_list)) {
		item = pool->free_list.next;
		list_del(item);
		block = list_entry(item, struct omap34xx_dma_block, free_list);
		kfree(block);
	}
}

void omap34xx_dma_pool_destroy(struct omap34xx_dma_pool *pool)
{
	size_t off;

	for (off = 0; off < pool->size; off += PAGE_SIZE)
		free_page((unsigned long)(pool->cpu_addr + off));

	omap34xx_dma_pool_deinit(pool);
}
EXPORT_SYMBOL(omap34xx_dma_pool_destroy);

static int omap34xx_dma_pool_init(struct omap34xx_dma_pool *pool,
					unsigned int size, void *cpu_addr,
					dma_addr_t dev_addr,
					omap34xx_dma_block_mmap_t mmap,
					omap34xx_videobuf_sync_t sync)
{
	int rc;
	struct omap34xx_dma_block *block;

	pool->size = size;
	pool->cpu_addr = cpu_addr;
	pool->dev_addr = dev_addr;
	INIT_LIST_HEAD(&pool->free_list);
	pool->mmap = mmap;
	pool->sync = sync;

	if (!(block = kzalloc(sizeof(*block), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	block->size = size;
	block->cpu_addr = cpu_addr;
	block->dev_addr = dev_addr;
	list_add(&block->free_list, &pool->free_list);

	SPEW_FREE_LIST(2, pool);

	rc = 0;
exit:
	return (rc);
}

int omap34xx_dma_pool_create(struct omap34xx_dma_pool *pool, unsigned int size)
{
	int rc;
	int order = get_order(size);
	void *cpu_addr;
	size_t off;
	size_t alloc_size;
	dma_addr_t dev_addr;

	if (!(cpu_addr = (void *)__get_dma_pages(GFP_KERNEL, order))) {
		rc = -ENOMEM;
		goto exit;
	}

	size = PAGE_ALIGN(size);
	dev_addr = virt_to_dma(NULL, cpu_addr);
	alloc_size = PAGE_SIZE << order;

	/*
	 * this is a workaround for drivers which do not honor the VM_PFNMAP
	 * flag set by remap_pfn_range.
	 */
	split_page(virt_to_page(cpu_addr), order);

	/* free unrequested pages */
	for (off = size; off < alloc_size; off += PAGE_SIZE)
		free_page(cpu_addr + off);

	if ((rc = omap34xx_dma_pool_init(pool, size, cpu_addr, dev_addr,
					omap34xx_dma_block_mmap,
					omap34xx_videobuf_sync_single))) {
		omap34xx_dma_pool_destroy(pool);
		goto exit;
	}
exit:
	return (rc);
}
EXPORT_SYMBOL(omap34xx_dma_pool_create);

int omap34xx_dma_block_mmap_coherent(struct omap34xx_dma_block *block,
					struct vm_area_struct *vma)
{
	int rc;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	rc = omap34xx_dma_block_mmap(block, vma);

	return (rc);
}

void omap34xx_dma_pool_destroy_coherent(struct omap34xx_dma_pool *pool)
{
	dma_free_coherent(NULL, pool->size, pool->cpu_addr, pool->dev_addr);
	omap34xx_dma_pool_deinit(pool);
}
EXPORT_SYMBOL(omap34xx_dma_pool_destroy_coherent);

int omap34xx_dma_pool_create_coherent(struct omap34xx_dma_pool *pool,
					unsigned int size)
{
	int rc;
	void *cpu_addr;
	dma_addr_t dev_addr;

	if (!(cpu_addr = dma_alloc_coherent(NULL, size, &dev_addr,
						GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	if ((rc = omap34xx_dma_pool_init(pool, size, cpu_addr, dev_addr,
			omap34xx_dma_block_mmap_coherent, NULL))) {
		omap34xx_dma_pool_destroy_coherent(pool);
		goto exit;
	}
exit:
	return (rc);
}
EXPORT_SYMBOL(omap34xx_dma_pool_create_coherent);
