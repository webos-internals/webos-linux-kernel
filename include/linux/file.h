/*
 * Wrapper functions for accessing the file_struct fd array.
 */

#ifndef __LINUX_FILE_H
#define __LINUX_FILE_H

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/posix_types.h>

struct file;

extern void __fput(struct file *);
extern void fput(struct file *);
extern void drop_file_write_access(struct file *file);

struct file_operations;
struct vfsmount;
struct dentry;
extern int init_file(struct file *, struct vfsmount *mnt,
		struct dentry *dentry, fmode_t mode,
		const struct file_operations *fop);
extern struct file *alloc_file(struct vfsmount *, struct dentry *dentry,
		fmode_t mode, const struct file_operations *fop);

#ifdef CONFIG_FORCED_UNMOUNT

#define clear_f_light(file)			\
	do {					\
		(file)->f_light = 0;		\
	} while (0)				\

#define set_f_light(file)			\
	do {					\
		if (file)			\
			(file)->f_light = 1;	\
	} while (0)				\

extern void fput_light(struct file *file, int fput_needed);

#else

#define clear_f_light(file)	do { } while (0)
#define set_f_light(file)	do { } while (0)

static inline void fput_light(struct file *file, int fput_needed)
{
	if (unlikely(fput_needed))
		fput(file);
}

#endif

extern struct file *fget(unsigned int fd);
extern struct file *fget_light(unsigned int fd, int *fput_needed);
extern void set_close_on_exec(unsigned int fd, int flag);
extern void put_filp(struct file *);
extern int alloc_fd(unsigned start, unsigned flags);
extern int get_unused_fd(void);
#define get_unused_fd_flags(flags) alloc_fd(0, (flags))
extern void put_unused_fd(unsigned int fd);

extern void fd_install(unsigned int fd, struct file *file);

#endif /* __LINUX_FILE_H */
