/*
 * badfs - the Bad Filesystem
 *
 * Author - Tigran Aivazian <tigran@veritas.com>
 *
 * Thanks to:
 *     Manfred Spraul <manfred@colorfullife.com>:
 *             useful comments.
 *     Scott Wood <scott.wood@timesys.com>:
 *             port to 2.6.
 *     Daniel McNeil <daniel@osdl.org>:
 *     Nick Wilson <njw@osdl.org>:
 *             cleanup for inclusion into mainline.
 *
 * The badfs filesystem is used by forced unmount ('umount -f' command).
 * Open files that keep the filesystem busy are redirected to badfs
 * inodes that return EIO for all operations.
 *
 * This file is released under the GPL.
 */
/*
 * Ported to 2.6.24. No munmap support.
 */

#include <linux/file.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/sched.h>
#include <linux/fs.h>

#define BADFS_MAGIC	0xBADF5001

static struct vfsmount *badfs_mnt;
static struct dentry *badfs_dentry_pwd;
static struct dentry *badfs_dentry_root;
static struct dentry *badfs_dentry_file;

static struct inode *badfs_get_inode(struct super_block *sb, int mode)
{
	struct inode *inode = new_inode(sb);

	if (inode) {
		make_bad_inode(inode);
		inode->i_mode = mode;
	}
	return inode;
}

static struct dentry * __init new_badfs_dentry_pwd(void)
{
	struct inode *inode;
	struct dentry *dentry;
	struct qstr name;
	int mode = S_IFDIR | S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH;

	inode = badfs_get_inode(badfs_mnt->mnt_sb, mode);
	if (!inode) {
		printk(KERN_ERR "FU: %s: can't allocate inode\n", __func__);
		return NULL;
	}

	name.name = "dead_pwd";
	name.len = strlen(name.name);
	dentry = d_alloc(badfs_mnt->mnt_sb->s_root, &name);
	if (!dentry) {
		iput(inode);
		printk(KERN_ERR "FU: %s: can't allocate dentry\n", __func__);
		return NULL;
	}
	d_instantiate(dentry, inode);

	return dentry;
}

static void disable_pwd(struct fs_struct *fs)
{
	set_fs_pwd(fs, badfs_mnt, badfs_dentry_pwd);
}

static struct dentry * __init new_badfs_dentry_root(void)
{
	struct inode *inode;
	struct dentry *dentry;
	struct qstr name;

	inode = badfs_get_inode(badfs_mnt->mnt_sb, S_IFDIR | 0755);
	if (!inode) {
		printk(KERN_ERR "FU: %s: can't allocate inode\n", __func__);
		return NULL;
	}

	name.name = "dead_root";
	name.len = strlen(name.name);
	dentry = d_alloc(badfs_mnt->mnt_sb->s_root, &name);
	if (!dentry) {
		iput(inode);
		printk(KERN_ERR "FU: %s: can't allocate dentry\n", __func__);
		return NULL;
	}
	d_instantiate(dentry, inode);

	return dentry;
}

static void disable_root(struct fs_struct *fs)
{
	set_fs_root(fs, badfs_mnt, badfs_dentry_root);
}

static int check_dirs(struct vfsmount *mnt)
{
	struct task_struct *p;
	unsigned long expires;

	expires = jiffies + HZ;

repeat:
	if (time_after(jiffies, expires)) {
		printk(KERN_INFO "FU: %s: timeout\n", __func__);
		return -1;
	}

	if (signal_pending(current))
		return -1;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct fs_struct *fs;

		/* get a reference to p->fs */
		task_lock(p);
		fs = p->fs;
		if (!fs) {
			task_unlock(p);
			continue;
		} else
			atomic_inc(&fs->count);
		task_unlock(p);

		if (fs->pwdmnt == mnt) {
			read_unlock(&tasklist_lock);
			printk(KERN_WARNING "FU: disable pwd of %s\n", p->comm);
			disable_pwd(fs);	/* may sleep */
			put_fs_struct(fs);
			goto repeat;
		}
		if (fs->rootmnt == mnt) {
			read_unlock(&tasklist_lock);
			printk(KERN_WARNING "FU: disable root of %s\n",
			       p->comm);
			disable_root(fs);	/* may sleep */
			put_fs_struct(fs);
			goto repeat;
		}
		put_fs_struct(fs);
	}
	read_unlock(&tasklist_lock);

	return 0;
}

static struct dentry * __init new_badfs_dentry_file(void)
{
	struct inode *inode;
	struct dentry *dentry;
	struct qstr name;

	inode = badfs_get_inode(badfs_mnt->mnt_sb, 0755);
	if (!inode) {
		printk(KERN_ERR "FU: %s: can't allocate inode\n", __func__);
		return NULL;
	}

	name.name = "dead_file";
	name.len = strlen(name.name);
	dentry = d_alloc(badfs_mnt->mnt_sb->s_root, &name);
	if (!dentry) {
		printk(KERN_ERR "FU: %s: can't allocate dentry\n", __func__);
		iput(inode);
		return NULL;
	}

	d_instantiate(dentry, inode);

	return dentry;
}

static struct file *get_bad_file(void)
{
	struct file *f;

	f = get_empty_filp();
	if (!f) {
		printk(KERN_ERR "FU: %s: can't allocate file\n", __func__);
		return NULL;
	}

	f->f_vfsmnt = mntget(badfs_mnt);
	f->f_dentry = dget(badfs_dentry_file);
	f->f_mapping = badfs_dentry_file->d_inode->i_mapping;
	f->f_op = badfs_dentry_file->d_inode->i_fop;

	return f;
}

#define RET_OK		0
#define RET_CLOSED	1
#define RET_F_LIGHT	2

/*
 * Close a file if it belongs to the specified fs.
 * (Inspired by sys_close())
 */
static int close_file_on_fs(unsigned int fd, struct files_struct *files,
			    struct vfsmount *mnt, struct file *badfilp)
{
	struct fdtable *fdt;
	struct inode *inode;
	struct file *filp;
	int ret = RET_OK;

	spin_lock(&files->file_lock);
	fdt = files_fdtable(files);
	if (fd >= fdt->max_fds)
		goto out_unlock;
	filp = fdt->fd[fd];
	if (!filp)
		goto out_unlock;

	inode = filp->f_dentry->d_inode;
	if (!inode || filp->f_vfsmnt != mnt)
		goto out_unlock;

	if (filp->f_light) {
		/* we have to wait for fput_light() */
		ret = RET_F_LIGHT;
		goto out_unlock;
	}

	/* replace the filp with the badfilp.
	 * preserve mode so as to get EIO rather than EBADF.
	 */
	badfilp->f_mode = filp->f_mode;
	rcu_assign_pointer(fdt->fd[fd], badfilp);
	spin_unlock(&files->file_lock);

	/* we have to unlock tasklist_lock here because
	 * filp_close() calls fput() which may sleep.
	 */
	read_unlock(&tasklist_lock);

	filp_close(filp, files);

	return RET_CLOSED; /* we closed a file */

out_unlock:
	spin_unlock(&files->file_lock);
	return ret;
}

/*
 * Close files if any of them belongs to the specified fs.
 * Inspired by flush_old_files()
 */
static int close_files_on_fs(struct files_struct * files,
			     struct vfsmount *mnt, struct file *badfilp)
{
	long j = -1;
	struct fdtable *fdt;
	int ret = RET_OK;

	spin_lock(&files->file_lock);
	for (;;) {
		unsigned long set, i;

		j++;
		i = j * __NFDBITS;
		fdt = files_fdtable(files);
		if (i >= fdt->max_fds)
			break;
		set = fdt->open_fds->fds_bits[j];
		if (!set)
			continue;
		spin_unlock(&files->file_lock);
		for ( ; set ; i++,set >>= 1) {
			if (set & 1) {
				int val;

				val = close_file_on_fs(i, files, mnt, badfilp);
				if (val == RET_CLOSED) {
					/* if we closed a file, we need to
					 * exit all the nested loop.
					 */
					return RET_CLOSED;
				} else if (val == RET_F_LIGHT) {
					ret = RET_F_LIGHT;
				}
			}
		}
		spin_lock(&files->file_lock);
	}
	spin_unlock(&files->file_lock);
	return ret;
}

static int check_files(struct vfsmount *mnt)
{
	struct task_struct *p;
	unsigned long expires;
	struct file *badfilp = NULL;
	int retry = 0;

	expires = jiffies + HZ;

repeat:
	if (time_after(jiffies, expires)) {
		printk(KERN_INFO "FU: %s: timeout\n", __func__);
		return -1;
	}

	if (signal_pending(current))
		return -1;

	/* Allocate a bad file in advance */
	if (!badfilp) {
		badfilp = get_bad_file();
		if (!badfilp)
			return -1;
	}

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct files_struct *files;
		int val;

		files = get_files_struct(p);
		if (!files)
			continue;
		val = close_files_on_fs(files, mnt, badfilp);
		put_files_struct(files);

		if (val == RET_CLOSED) {
			/* we closed a file. we need to repeat from the
			 * beginning with a new badfilp.
			 * NOTE: tasklist_lock has been unlocked.
			 */
			badfilp = NULL;
			printk(KERN_WARNING
			       "FU: closed a file opened by %s\n", p->comm);
			goto repeat;
		} else if (val == RET_F_LIGHT) {
			retry = 1;
			printk(KERN_WARNING
			       "FU: f_light call in progress by %s\n", p->comm);
		}
	}
	read_unlock(&tasklist_lock);
	if (retry) {
		retry = 0;
		schedule(); /* give a chance for fput_light() */
		goto repeat;
	}

	if (badfilp)
		filp_close(badfilp, NULL);

	return 0;
}

/* called from do_umount() if MNT_FORCE is specified */
void quiesce_filesystem(struct vfsmount *mnt)
{
	/* We do three passes through the task list, examining:
	 *   1. p->fs->{pwd,root}mnt that can keep this mnt busy
	 *   2. p->files, i.e. open files
	 *   3. p->mm->mmap, i.e. mmaped files (we simply do_munmap them)
	 * There is no guarantee that by the time we restart the loop
	 * the amount of work to do in the loop has not increased.
	 *
	 * There's also no guarantee that all activity has ceased by
	 * the time this is done, or that further references will not be
	 * obtained after this is done (the latter could be fixed by
	 * detaching from the namespace first).  If either happens, the
	 * unmount will fail as if it weren't forced, and the user needs
	 * to try again.
	 */

	printk(KERN_INFO "FU: %s\n", __func__);

	if (check_dirs(mnt) < 0)
		return;

	if (check_files(mnt) < 0)
		return;

	/*
	 * if (check_mmaps(mnt) < 0)
	 *	return;
	 */
}

static int badfs_get_sb(struct file_system_type *fstype,
			int flags, const char *dev_name, void *data,
			struct vfsmount *mnt)
{
	return get_sb_pseudo(fstype, "bad:", NULL, BADFS_MAGIC, mnt);
}

struct file_system_type badfs_fs_type = {
	.name = "badfs",
	.get_sb = badfs_get_sb,
	.kill_sb = kill_anon_super
};

static int __init init_badfs_fs(void)
{
	int err = register_filesystem(&badfs_fs_type);

	if (!err) {
		badfs_mnt = kern_mount(&badfs_fs_type);
		if (IS_ERR(badfs_mnt)) {
			err = PTR_ERR(badfs_mnt);
			goto err1;
		}
	}

	badfs_dentry_pwd = new_badfs_dentry_pwd();
	if (!badfs_dentry_pwd) {
		err = -ENOMEM;
		goto err1;
	}
	badfs_dentry_root = new_badfs_dentry_root();
	if (!badfs_dentry_root) {
		err = -ENOMEM;
		goto err2;
	}

	badfs_dentry_file = new_badfs_dentry_file();
	if (!badfs_dentry_file) {
		err = -ENOMEM;
		goto err3;
	}

	return 0;
err3:
	dput(badfs_dentry_root);
err2:
	dput(badfs_dentry_pwd);
err1:
	unregister_filesystem(&badfs_fs_type);

	return err;
}

static void __exit exit_badfs_fs(void)
{
	dput(badfs_dentry_file);
	dput(badfs_dentry_root);
	dput(badfs_dentry_pwd);

	unregister_filesystem(&badfs_fs_type);
}

module_init(init_badfs_fs);
module_exit(exit_badfs_fs);

MODULE_LICENSE("GPL");
