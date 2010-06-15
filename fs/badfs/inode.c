/*
 * badfs - the Bad Filesystem
 *
 * Author - Tigran Aivazian <tigran@veritas.com>
 *
 * Ported to 2.6.24 by toshi.kikuchi@palm.com
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

#include <linux/file.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/sched.h>

#define BADFS_MAGIC	0xBADF5001

static struct vfsmount *badfs_mnt;

static struct inode *badfs_get_inode(struct super_block *sb, int mode)
{
	struct inode *inode = new_inode(sb);

	if (inode) {
		make_bad_inode(inode);
		inode->i_mode = mode;
	}
	return inode;
}

static void disable_pwd(struct fs_struct *fs)
{
	struct inode *inode;
	struct dentry *dentry;
	struct qstr name;
	int mode = S_IFDIR | S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH;

	inode = badfs_get_inode(badfs_mnt->mnt_sb, mode);
	if (!inode) {
		printk(KERN_ERR "disable_pwd(): can't allocate inode\n");
		return;
	}

	name.name = "dead_pwd";
	name.len = strlen(name.name);
	dentry = d_alloc(badfs_mnt->mnt_sb->s_root, &name);
	if (!dentry) {
		iput(inode);
		printk(KERN_ERR "disable_pwd(): can't allocate dentry\n");
		return;
	}
	d_instantiate(dentry, inode);
	dget(dentry);

	set_fs_pwd(fs, badfs_mnt, dentry);
}

static void disable_root(struct fs_struct *fs)
{
	struct inode *inode;
	struct dentry *dentry;
	struct qstr name;

	inode = badfs_get_inode(badfs_mnt->mnt_sb, S_IFDIR | 0755);
	if (!inode) {
		printk(KERN_ERR "disable_root(): can't allocate inode\n");
		return;
	}

	name.name = "dead_root";
	name.len = strlen(name.name);
	dentry = d_alloc(badfs_mnt->mnt_sb->s_root, &name);
	if (!dentry) {
		iput(inode);
		printk(KERN_ERR "disable_root(): can't allocate dentry\n");
		return;
	}
	d_instantiate(dentry, inode);
	dget(dentry);

	set_fs_root(fs, badfs_mnt, dentry);
}

static int check_dirs(struct vfsmount *mnt)
{
	struct task_struct *p;
	unsigned long expires;

	expires = jiffies + HZ;

repeat:
	if (time_after(jiffies, expires)) {
		printk(KERN_INFO "check_dirs: timeout\n");
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
			disable_pwd(fs);	/* may sleep */
			put_fs_struct(fs);
			goto repeat;
		}
		if (fs->rootmnt == mnt) {
			read_unlock(&tasklist_lock);
			disable_root(fs);	/* may sleep */
			put_fs_struct(fs);
			goto repeat;
		}
		put_fs_struct(fs);
	}
	read_unlock(&tasklist_lock);

	return 0;
}

static struct file *get_bad_file(void)
{
	struct inode *i;
	struct dentry *d;
	struct qstr name;
	struct file *f = get_empty_filp();

	if (!f) {
		printk(KERN_ERR "get_bad_file(): can't allocate file\n");
		goto err;
	}

	i = badfs_get_inode(badfs_mnt->mnt_sb, 0755);
	if (!i) {
		printk(KERN_ERR "get_bad_file(): can't allocate inode\n");
		goto err_inode;
	}

	name.name = "dead_file";
	name.len = strlen(name.name);
	d = d_alloc(badfs_mnt->mnt_sb->s_root, &name);
	if (!d) {
		printk(KERN_ERR "get_bad_file(): can't allocate dentry\n");
		goto err_dentry;
	}

	d_instantiate(d, i);

	f->f_vfsmnt = mntget(badfs_mnt);
	f->f_dentry = dget(d);
	f->f_mapping = i->i_mapping;
	f->f_op = i->i_fop;

	return f;

err_dentry:
	iput(i);
err_inode:
	filp_close(f, NULL);
err:
	return NULL;
}

static int check_file(unsigned int fd, struct files_struct *files,
		      struct fdtable *fdt, struct vfsmount *mnt,
		      struct file *badfile)
{
	struct inode *inode;
	struct file *file = fcheck_files(files, fd);

	if (file) {
		inode = file->f_dentry->d_inode;
		if (inode && file->f_vfsmnt == mnt) {
			/* If allocation failed, the forced unmount
			   fails.  This seems safer than just closing
			   the fd.  Note that it will retry until
			   either it succeeds or a signal is received. */
			if (!badfile)
				return 1;

			/* Preserve mode so as to get EIO rather
			 * than EBADF. */
			badfile->f_mode = file->f_mode;

			atomic_inc(&badfile->f_count);
			rcu_assign_pointer(fdt->fd[fd], badfile);

			while (file->f_light)
				schedule();
			filp_close(file, files);

			return 1;
		}
	}

	return 0;
}

static int check_files(struct vfsmount *mnt)
{
	struct task_struct *p;
	struct file *badfile = NULL;
	unsigned long expires;

	expires = jiffies + HZ;

repeat:
	if (time_after(jiffies, expires)) {
		printk(KERN_INFO "check_files: timeout\n");
		return -1;
	}

	if (signal_pending(current))
		return -1;

	/* Allocate a bad file in advance, so it's not done with
	   files_lock held.  I'm not sure what the lock order issues are
	   with doing that stuff with the lock held. */
	if (!badfile)
		badfile = get_bad_file();

	read_lock(&tasklist_lock);
	for_each_process(p) {
		unsigned int i, j;
		struct files_struct *files;
		struct fdtable *fdt;

		/* get a reference to p->files */
		files = get_files_struct(p);
		if (!files)
			continue;

		j = 0;

		rcu_read_lock();
		spin_lock(&files->file_lock);
		fdt = files_fdtable(files);
		/* check if this process has open files here */
		/* see close_files() */
		for (;;) {
			unsigned long set;
			i = j * __NFDBITS;
			if (i >= fdt->max_fds)
				break;
			set = fdt->open_fds->fds_bits[j++];
			while (set) {
				if (set & 1) {
					if (check_file(i, files, fdt, mnt,
						       badfile)) {
						spin_unlock(&files->file_lock);
						rcu_read_unlock();
						put_files_struct(files);
						read_unlock(&tasklist_lock);
						goto repeat;
					}
				}
				i++;
				set >>= 1;
			}
		}
		spin_unlock(&files->file_lock);
		rcu_read_unlock();
		put_files_struct(files);
	}
	read_unlock(&tasklist_lock);

	if (badfile)
		filp_close(badfile, NULL);

	return 0;
}

static struct vm_area_struct *find_mmap_on_fs(struct mm_struct *mm,
					      struct vfsmount *mnt)
{
	struct vm_area_struct *vma;
	struct file *file;
	struct inode *inode;

	/* check for mmap'd files on this vfsmount */
	for (vma = mm->mmap; vma; vma = vma->vm_next) {
		file = vma->vm_file;
		if (!file)
			continue;
		inode = file->f_dentry->d_inode;
		if (!inode || !inode->i_sb)
			continue;
		if (file->f_vfsmnt == mnt)
			return vma;
	}

	return NULL;
}

static int check_mmaps(struct vfsmount *mnt)
{
	struct task_struct *p;
	int retry = 0;
	unsigned long expires;

	expires = jiffies + HZ;

repeat:
	if (time_after(jiffies, expires)) {
		printk(KERN_INFO "check_mmaps: timeout\n");
		return -1;
	}

	if (signal_pending(current))
		return -1;

	read_lock(&tasklist_lock);
	for_each_process(p) {
		struct mm_struct *mm;
		struct vm_area_struct *vma;

		mm = get_task_mm(p);
		if (!mm)
			continue;

		if (down_read_trylock(&mm->mmap_sem)) {
			vma = find_mmap_on_fs(mm, mnt);
			up_read(&mm->mmap_sem);
		} else {
			mmput(mm);
			retry = 1;
			continue;
		}

		if (vma) {
			read_unlock(&tasklist_lock);

			down_write(&mm->mmap_sem);

			/* vma may have gone away while mmap_sem was not held */
			vma = find_mmap_on_fs(mm, mnt);
			if (vma)
				do_munmap(mm, vma->vm_start,
					  vma->vm_end - vma->vm_start);

			up_write(&mm->mmap_sem);
			mmput(mm);
			goto repeat;
		}

		mmput(mm);
	}

	read_unlock(&tasklist_lock);
	if (retry) {
		/* We had to skip one or more processes.  Try again. */
		retry = 0;
		goto repeat;
	}

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

	if (check_dirs(mnt) < 0)
		return;

	if (check_files(mnt) < 0)
		return;

	if (check_mmaps(mnt) < 0)
		return;
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
			unregister_filesystem(&badfs_fs_type);
		}
	}

	return err;
}

static void __exit exit_badfs_fs(void)
{
	unregister_filesystem(&badfs_fs_type);
}

module_init(init_badfs_fs);
module_exit(exit_badfs_fs);

MODULE_LICENSE("GPL");
