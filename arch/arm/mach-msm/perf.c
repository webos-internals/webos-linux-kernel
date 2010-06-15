/* arch/arm/mach-msm/perf.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/msm_perf.h>

#define DEV_NAME "msm_perf"

static dev_t driver_device_no;
static struct cdev driver_cdev;
static struct class *driver_class;

static inline uint32_t msm_read_pmnc(void)
{
	uint32_t pmnc = 0;

	asm volatile ("mrc p15, 0, %0, c15, c12, 0" : "=r" (pmnc));

	return pmnc;
}

static inline void msm_read_perf(struct msm_perf_snapshot *snapshot)
{
	uint32_t	pmnc;
	asm volatile (
	    "mrc p15, 0, %0, c15, c12, 1\n\t"
	    "mrc p15, 0, %1, c15, c12, 2\n\t"
	    "mrc p15, 0, %2, c15, c12, 3\n\t"
	    : "=r" (snapshot->ccnt),
	      "=r" (snapshot->pmn0),
	      "=r" (snapshot->pmn1));

	pmnc = msm_read_pmnc();
	snapshot->pmn0_ov = (pmnc & (1 << 8));
	snapshot->pmn1_ov = (pmnc & (1 << 9));
	snapshot->ccnt_ov = (pmnc & (1 << 10));
}

static inline void msm_write_pmnc(uint32_t pmnc)
{
	asm volatile("mcr p15, 0, %0, c15, c12, 0" : : "r" (pmnc));
}


static int msm_perf_open(struct inode *inode, struct file *filp)
{
	int rc;

	rc = nonseekable_open(inode, filp);
	if (rc < 0)
		return rc;

	return 0;
}

static int msm_perf_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int msm_perf_read(struct file *filp, char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct msm_perf_snapshot snapshot;

	if (count != sizeof(struct msm_perf_snapshot))
		return -EINVAL;

	memset(&snapshot, 0, sizeof(snapshot));
	msm_read_perf(&snapshot);
	if (copy_to_user(buf, &snapshot, sizeof(struct msm_perf_snapshot)))
		return -EFAULT;

	return sizeof(struct msm_perf_snapshot);
}

static ssize_t msm_perf_write(struct file *filp, const char __user *buf,
			      size_t count, loff_t *ppos)
{
	return -ENOSYS;
}

static int msm_perf_ioctl(struct inode *inode, struct file *filep,
			  unsigned int cmd, unsigned long arg)
{
	return -ENOSYS;
}

/*  ================================
 *  Module init / exit
 *  ================================
 */
static struct file_operations msm_perf_fops = {
	.owner	 = THIS_MODULE,
	.open	 = msm_perf_open,
	.release = msm_perf_release,
	.read	 = msm_perf_read,
	.write	 = msm_perf_write,
	.ioctl	 = msm_perf_ioctl,
};

static int __init msm_perf_init(void)
{
	struct class_device *class_dev;
	int rc;

	printk(KERN_INFO "msm_perf_init():\n");
	rc = alloc_chrdev_region(&driver_device_no, 0, 1, DEV_NAME);
	if (rc < 0)
		return rc;

	driver_class = class_create(THIS_MODULE, DEV_NAME);
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		goto fail_unregister;
	}

	class_dev = class_device_create(driver_class, NULL,
					driver_device_no,
					NULL, DEV_NAME);
	if (!class_dev) {
		rc = -ENOMEM;
		goto fail_destroy_class;
	}

	cdev_init(&driver_cdev, &msm_perf_fops);
	driver_cdev.owner = THIS_MODULE;

	rc = cdev_add(&driver_cdev, driver_device_no, 1);
	if (rc < 0)
		goto fail_cleanup_all;

	/* Enable the cycle counter */
	msm_write_pmnc(0x01);

	return 0;

fail_cleanup_all:
	class_device_destroy(driver_class, driver_device_no);
fail_destroy_class:
	class_destroy(driver_class);
fail_unregister:
	unregister_chrdev_region(driver_device_no, 1);
	return rc;
}

static void __exit msm_perf_exit(void)
{
	cdev_del(&driver_cdev);
	class_device_destroy(driver_class, driver_device_no);
	class_destroy(driver_class);
	unregister_chrdev_region(driver_device_no, 1);
}


module_init(msm_perf_init);
module_exit(msm_perf_exit);

MODULE_DESCRIPTION("Performance Counter Driver");
MODULE_AUTHOR("San Mehat <san@android.com>");
MODULE_LICENSE("GPL");
