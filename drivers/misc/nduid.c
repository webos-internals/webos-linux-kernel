/* 
 * drivers/misc/nduid.c
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static void *nduid_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *nduid_next(struct seq_file *m, void *v, loff_t *pos)
{
	++pos;
	return NULL;
}

static void nduid_stop(struct seq_file *m, void *v)
{
	/* nothing to do */
}

static unsigned char nduid[128];

static int nduid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", nduid);
	return 0;
}

static struct seq_operations nduid_ops = {
	.start = nduid_start,
	.next = nduid_next,
	.stop = nduid_stop,
	.show = nduid_show,
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

char *nduid_string_get(void)
{
	return nduid;
}
EXPORT_SYMBOL(nduid_string_get);

static int __init nduid_setup(char *str)
{
	strncpy(nduid, str, sizeof(nduid));
	return 0;
}
__setup("nduid=", nduid_setup);

static int __init nduid_init(void)
{
	struct proc_dir_entry *entry;

	entry = create_proc_entry("nduid", S_IRUGO, NULL);
	if (entry == NULL) {
		return -1;
	}

	entry->proc_fops = &nduid_proc_ops;

	printk(KERN_INFO "nduid: %s\n", nduid);

	return 0;
}
__initcall(nduid_init);
