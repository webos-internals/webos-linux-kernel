/* arch/arm/mach-msm/msm_adm.c
 *
 * MSM7x25 Application Data Mover Interface
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Kevin McCray <kevin.mccray@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/android_pmem.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>

#include <linux/msm_adm_external.h>
#include "msm_adm.h"

#define MODULE_NAME "msm_adm"

DECLARE_MUTEX(adm_sem);

#define ADM_DEBUG_MSGS 0
#if ADM_DEBUG_MSGS
#define DLOG(fmt,args...) \
	do { printk(KERN_INFO "[%s:%s:%d] "fmt, __FILE__, __func__, __LINE__, \
		    ##args); } \
	while (0)
#else
#define DLOG(x...) do {} while (0)
#endif

int get_pmem_info(struct adm_device_state *p_state, struct adm_box_img *img, unsigned long *start, unsigned long *len)
{
	int put_needed, ret = -1;
	struct file *file = fget_light(img->fd, &put_needed);

	if (file == NULL)
		return -1;

	// Framebuffer is not a real pmem device
	// Cheat here and return framebuffer start and size obtained from the platform data.
	if (MAJOR(file->f_dentry->d_inode->i_rdev) == FB_MAJOR) {
		*start = p_state->fb_start;
		*len = p_state->fb_size;
		ret = 0;
		goto end;
	}

#ifdef CONFIG_ANDROID_PMEM
	if (!get_pmem_fd(img->fd, start, len))
		ret = 0;
#endif
end:
	fput_light(file, put_needed);
	return ret;
}

static void
adm_free_data(struct work_struct *work)
{
	struct adm_dma_data *dma = container_of(work, struct adm_dma_data, workitem);


	// allow next dma operation to start
	dma->state->dma_state = DMA_READY;
	wake_up(&dma->state->wq);

	dma_free_coherent(NULL, sizeof(struct pmem_adm_nc_dmadata), dma->nc, dma->nc_busaddr);

	kfree(dma);

}

static void
adm_dma_complete_func(struct msm_dmov_cmd *cmd,
			  unsigned int result,
			  struct msm_dmov_errdata *err)
{
	struct adm_dma_data *dma = container_of(cmd, struct adm_dma_data, hdr);

	DLOG("adm_dma_complete_func: result=0x%x\n", result);

	if(err)
	{
		DLOG("ERRDATA: FLUSH0=0x%x, FLUSH1=0x%x, FLUSH2=0x%x \
			FLUSH3=0x%x, FLUSH4=0x%x FLUSH5=0x%x\n",
			err->flush[0], err->flush[1], err->flush[2], err->flush[3], 
			err->flush[4], err->flush[5]);
	}

	// irq's are disabled so we can't free the dma memory here.
	schedule_work(&dma->workitem);

}

static int msm_adm_ioctl(struct inode *ip, struct file *fp,
		     unsigned int cmd, unsigned long arg)
{
	struct adm_device_state *p_state = container_of(fp->f_op, struct adm_device_state, fops);
	int ret = 0;

	if(!p_state)
	{
		printk(KERN_ERR "msm_adm_ioctl: invalid context\n");
		return -EINVAL;
	}

	switch(cmd)
	{
		case MSMADM_PMEM_BOX_COPY:
		{
			struct pmem_adm_box_transfer pmembox;
			struct adm_dma_data *dma;
			dmov_box *box;			
			unsigned long src_start, src_len, dst_start, dst_len;

			down(&adm_sem);

			if(copy_from_user(&pmembox, (struct pmem_adm_box_transfer *)arg, sizeof(struct pmem_adm_box_transfer)))
			{
				printk(KERN_ERR "ERROR: msm_adm_ioctl: MSMADM_PMEM_BOX_COPY, failed to copy args\n");
				ret = -EINVAL;
			}

			DLOG("PMEM_BOX_COPY: src(r:%d, c:%d, rowsize:%d, o:%d, id:%d)\n",
				pmembox.src.rows, pmembox.src.columns, pmembox.src.row_size, pmembox.src.offset,
				pmembox.src.fd);
			DLOG("PMEM_BOX_COPY: dst(r:%d, c:%d, rowsize:%d, o:%d, id:%d)\n",
				pmembox.dst.rows, pmembox.dst.columns, pmembox.dst.row_size, pmembox.dst.offset,
				pmembox.dst.fd);

			if (unlikely(get_pmem_info(p_state, &pmembox.src, &src_start, &src_len)))
			{
				printk(KERN_ERR "Failed to get src image!\n");
				return -EINVAL;
			}
			DLOG("SRC_IMG: start=0x%x, len=0x%x\n", src_start + pmembox.src.offset, src_len);

			if (unlikely(get_pmem_info(p_state, &pmembox.dst, &dst_start, &dst_len)))
			{
				printk(KERN_ERR "Failed to get dst image!\n");
				return -EINVAL;
			}
			DLOG("DST_IMG: start=0x%x, len=0x%x\n", dst_start + pmembox.dst.offset, dst_len);

			dma = kzalloc(sizeof(struct adm_dma_data), GFP_KERNEL);

			dma->nc = dma_alloc_coherent(NULL,
				  sizeof(struct pmem_adm_nc_dmadata),
				  &dma->nc_busaddr,
				  GFP_KERNEL);
			if (dma->nc == NULL) {
				printk(KERN_ERR "Unable to allocate DMA buffer\n");
				return -ENOMEM;
			}
			memset(dma->nc, 0x00, sizeof(struct pmem_adm_nc_dmadata));

			// init dma transfer
			dma->cmd_busaddr = dma->nc_busaddr;
			dma->cmdptr_busaddr = dma->nc_busaddr +
				offsetof(struct pmem_adm_nc_dmadata, cmdptr);
			dma->channel = 4;


			// configure dma
			dma->dir = DMA_TO_DEVICE;
			box = &dma->nc->cmd[0];


			box->cmd = CMD_MODE_BOX | CMD_LC;

			box->src_row_addr = src_start + pmembox.src.offset;
			box->dst_row_addr = dst_start + pmembox.dst.offset;
			box->src_dst_len = pmembox.dst.row_size | (pmembox.src.row_size << 16);
			box->num_rows = pmembox.dst.rows | (pmembox.src.rows << 16);
			box->row_offset = pmembox.dst.row_size | (pmembox.src.row_size << 16);

			dma->nc->cmdptr = (dma->cmd_busaddr >> 3) | CMD_PTR_LP;
			dma->hdr.cmdptr = DMOV_CMD_PTR_LIST |
			       DMOV_CMD_ADDR(dma->cmdptr_busaddr);
			dma->hdr.complete_func = adm_dma_complete_func;

			INIT_WORK(&dma->workitem, adm_free_data);
			dma->state = p_state;
			p_state->dma_state = DMA_BUSY;

			msm_dmov_enqueue_cmd(dma->channel, &dma->hdr);

			// TODO: Make asychronous?
			// wait for dma operation to complete before returning
			wait_event_interruptible(p_state->wq, p_state->dma_state == DMA_READY);

			up(&adm_sem);

			break;
		}
		default:
		{
			printk("msm_adm_state: unknown ioctl\n");
			break;
		}
	}

	return 0;
}

static int msm_adm_open(struct inode *ip, struct file *fp)
{

	return 0;
}


static const struct file_operations msm_adm_fops = {
	.open = msm_adm_open,
	.ioctl = msm_adm_ioctl,
};

static int __init msm_adm_probe(struct platform_device *pdev)
{
	struct adm_device_state *state = NULL;
	int ret = 0;

	struct android_pmem_platform_data *pi = pdev->dev.platform_data;

	printk("msm_adm_init()\n");
	state = kzalloc(sizeof(struct adm_device_state), GFP_KERNEL);
	if (!state) {
		return (-ENOMEM);
	}

	state->dma_state = DMA_READY;
	init_waitqueue_head(&state->wq);

	/* store fops */
	memcpy(&state->fops, &msm_adm_fops, sizeof(struct file_operations));

	state->mdev.name = "msm_adm";
	state->mdev.minor = MISC_DYNAMIC_MINOR;
	state->mdev.fops = &state->fops;

	state->fb_start = pi->start;	
	state->fb_size = pi->size;

	ret = misc_register(&state->mdev);
	if(ret)
	{
		printk(KERN_ERR "Failed to create /dev/adm\n");
		goto error;
	}
	

	return 0;

error:
	printk(KERN_ERR "msm_adm_probe failed!\n");
	kfree(state);
	return ret;
}



static struct platform_driver msm_adm_driver = {
	.probe = msm_adm_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_adm_init(void)
{
	return platform_driver_register(&msm_adm_driver);
}

module_init(msm_adm_init);


MODULE_DESCRIPTION("ADM Character Device");
MODULE_AUTHOR("Kevin McCray <kevin.mccray@palm.com>");
MODULE_LICENSE("GPL");
