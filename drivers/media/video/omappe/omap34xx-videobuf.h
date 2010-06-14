#ifndef OMAP34XX_VIDEOBUF_H
#define OMAP34XX_VIDEOBUF_H

#include <media/videobuf-core.h>

struct omap34xx_dma_block {
	size_t			size;
	void			*cpu_addr;
	dma_addr_t		dev_addr;
	struct list_head	free_list;
};

typedef void (*omap34xx_videobuf_sync_t)(struct videobuf_buffer *);

struct omap34xx_videobuf {
	/* userptr pages */
	int				nr_pages;
	struct page			**pages;
	struct scatterlist		*sg_list;

	/* userptr i/o & mmap */
	dma_addr_t			dev_addr;

	/* mmap */
	struct omap34xx_dma_block	*block;

	omap34xx_videobuf_sync_t	sync;
};

typedef int (*omap34xx_dma_block_mmap_t)(struct omap34xx_dma_block *,
						struct vm_area_struct *);

struct omap34xx_dma_pool {
	size_t			size;
	void			*cpu_addr;
	dma_addr_t		dev_addr;
	struct list_head	free_list;

	omap34xx_dma_block_mmap_t	mmap;
	omap34xx_videobuf_sync_t	sync;
};

extern struct omap34xx_videobuf * get_omap34xx_buf(struct videobuf_buffer *);

extern int omap34xx_videobuf_init(struct videobuf_queue *,
					struct videobuf_buffer *);
extern void omap34xx_videobuf_release(struct videobuf_queue *,
					struct videobuf_buffer *);

extern void omap34xx_videobuf_queue_init(
		struct videobuf_queue *q,
		struct videobuf_queue_ops *ops,
		struct omap34xx_dma_pool *pool,
		spinlock_t *lock,
		enum v4l2_buf_type type,
		enum v4l2_field field,
		unsigned int buf_size,
		void *priv
		);

extern int omap34xx_dma_pool_create(struct omap34xx_dma_pool *pool,
					unsigned int size);

extern void omap34xx_dma_pool_destroy(struct omap34xx_dma_pool *pool);

extern int omap34xx_dma_pool_create_coherent(struct omap34xx_dma_pool *pool,
						unsigned int size);

extern void omap34xx_dma_pool_destroy_coherent(struct omap34xx_dma_pool *pool);

#endif /* OMAP34XX_VIDEOBUF_H */
