/*
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/crypto.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <asm/arch/nduid.h>
#include <asm/io.h>

static void *nduid_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *nduid_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void nduid_stop(struct seq_file *m, void *v)
{
	/* nothing to do */
}

struct nduid_ctxt {
	struct miscdevice mdev;
	struct file_operations fops;

	unsigned char *nduid_string;
	unsigned char *nduid_binary;
	size_t nduid_binary_len;
};
/* XXX kind of gross */
static struct nduid_ctxt *_nduid_ctxt = NULL;

static int nduid_show(struct seq_file *m, void *v)
{
	if (_nduid_ctxt) {
		seq_printf(m, "%s\n", _nduid_ctxt->nduid_string);
	}
	return 0;
}

static struct seq_operations nduid_ops = {
	.start = nduid_start,
	.next = nduid_next,
	.stop = nduid_stop,
	.show = nduid_show
};

static int nduid_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &nduid_ops);
}

static struct file_operations nduid_proc_ops = {
	.open = nduid_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static ssize_t nduid_mdev_read(struct file *file, char __user *buf, 
		size_t count, loff_t *ptr)
{
	struct nduid_ctxt *ctxt;

	ctxt = container_of(file->f_op, struct nduid_ctxt, fops);

	if (count < ctxt->nduid_binary_len) {
		return -EINVAL;
	}

	if (copy_to_user(buf, ctxt->nduid_binary, ctxt->nduid_binary_len)) {
		return -EFAULT;
	}

	return ctxt->nduid_binary_len;
}

static struct file_operations nduid_mdev_fops = {
	.owner = THIS_MODULE,
	.read = nduid_mdev_read,
	.open = nonseekable_open,
};

static unsigned int
setup_sg(struct scatterlist *sg, const void *address, unsigned int length)
{
	sg_set_buf(sg, address, length);
	return length;
}

static int __init nduid_probe(struct platform_device *pdev)
{
	int r = 0;
	struct nduid_config *pcfg;
	struct nduid_ctxt *ctxt;

	struct crypto_hash *sha1;
	unsigned int palm_salt, dev_salt;
	char id[256];
	unsigned int idlen;
	struct scatterlist sg[3];
	unsigned int nbytes;
	struct hash_desc desc;
	char *sptr;
	int i;

	struct proc_dir_entry *entry;

	if (_nduid_ctxt) {
		printk(KERN_ERR "nduid: more than one nduid device\n");
		return -EINVAL;
	}

	pcfg = pdev->dev.platform_data;
	if (!pcfg) {
		printk(KERN_ERR "nduid: no platform data\n");
		return -ENODEV;
	}

	ctxt = kzalloc(sizeof(struct nduid_ctxt), GFP_KERNEL);
	if (!ctxt) {
		printk(KERN_ERR "nduid: can't alloc ctxt\n");
		return -ENOMEM;
	}
	_nduid_ctxt = ctxt;
	platform_set_drvdata(pdev, ctxt);

	/* compute the nduid on init and store */
	sha1 = crypto_alloc_hash("sha1", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(sha1)) {
		printk(KERN_ERR "nduid: error allocating sha1 alg %d\n", (int)sha1);
		goto nduid_probe_fail;
	}

	ctxt->nduid_binary_len = crypto_hash_digestsize(sha1);
	/* XXX check digest size? */
	printk("digest size %u\n", ctxt->nduid_binary_len);

	ctxt->nduid_binary = kmalloc(ctxt->nduid_binary_len, GFP_KERNEL);
	if (!ctxt->nduid_binary) {
		printk(KERN_ERR "nduid: error allocating sha1 digest\n");
		goto nduid_probe_fail;
	}
	/* Need 2 bytes to print out a byte as a string */
	ctxt->nduid_string = kmalloc(ctxt->nduid_binary_len*2, GFP_KERNEL);
	if (!ctxt->nduid_string) {
		printk(KERN_ERR "nduid: error allocating nduid\n");
		goto nduid_probe_fail;
	}

	palm_salt = 0x0830aa55;
	dev_salt = pcfg->get_device_salt();
	idlen = pcfg->get_cpu_id(id, sizeof(id));

	nbytes = setup_sg(&sg[0], &palm_salt, sizeof(uint32_t));
	nbytes += setup_sg(&sg[1], &dev_salt, sizeof(uint32_t));
	nbytes += setup_sg(&sg[2], id, idlen);

	desc.tfm = sha1;
	desc.flags = 0;

	crypto_hash_digest(&desc, sg, nbytes, ctxt->nduid_binary);

	/* Turn it into a string */
	sptr = ctxt->nduid_string;
	for (i = 0; i < ctxt->nduid_binary_len; i++) {
		snprintf(sptr, 3, "%02x", ctxt->nduid_binary[i]);
		sptr += 2;
	}
	*sptr = '\0';

	crypto_free_hash(sha1);

	printk("nduid: %s\n", ctxt->nduid_string);

	/* create the entry in /proc */
	entry = create_proc_entry("nduid", 0, NULL);
	if (entry) {
		entry->proc_fops = &nduid_proc_ops;
	}

	/* init misc device */
	memcpy(&ctxt->fops, &nduid_mdev_fops, sizeof(struct file_operations));
	ctxt->mdev.minor = MISC_DYNAMIC_MINOR;
	ctxt->mdev.name = pcfg->dev_name;
	ctxt->mdev.fops = &ctxt->fops;

	/* register misc device */
	r = misc_register(&ctxt->mdev);
	if (r) {
		printk(KERN_ERR "nduid: failed to register misc device\n");
		goto nduid_probe_fail;
	}

	return 0;

nduid_probe_fail:
	if (ctxt) {
		if (ctxt->nduid_string) {
			kfree(ctxt->nduid_string);
		}
		if (ctxt->nduid_binary) {
			kfree(ctxt->nduid_binary);
		}
		kfree(ctxt);
	}

	return -1;
}

char *nduid_string_get(void)
{
	return _nduid_ctxt->nduid_string ?: 0;
}
EXPORT_SYMBOL(nduid_string_get);

static int __devexit nduid_remove(struct platform_device *pdev)
{
	struct nduid_ctxt *ctxt;

	ctxt = platform_get_drvdata(pdev);
	if (!ctxt) {
		return 0;
	}

	misc_deregister(&ctxt->mdev);

	kfree(ctxt->nduid_string);
	kfree(ctxt->nduid_binary);
	kfree(ctxt);

	return 0;
}

#ifdef CONFIG_PM
static int nduid_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* nothing to do */
	return 0;
}

static int nduid_resume(struct platform_device *pdev)
{
	/* nothing to do */
	return 0;
}
#else
#define nduid_suspend        NULL
#define nduid_resume         NULL
#endif


static struct platform_driver nduid_driver = {
	.driver = {
		.name = "nduid",
	},
	.probe = nduid_probe,
	.remove = __devexit_p(nduid_remove),
	.suspend = nduid_suspend,
	.resume = nduid_resume,
};

static int __init nduid_init(void)
{
	return platform_driver_register(&nduid_driver);
}

static void __exit nduid_exit(void)
{
	platform_driver_unregister(&nduid_driver);
}

module_init(nduid_init);
module_exit(nduid_exit);

MODULE_DESCRIPTION("Nova device UID driver");
MODULE_LICENSE("GPL");
