#ifndef _RAMZSWAP_COMPAT_H_
#define _RAMZSWAP_COMPAT_H_

#include <linux/version.h>

/* Uncomment this if you are using swap free notify patch */
#define CONFIG_SWAP_NOTIFIERS

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)
#define BIO_IO_ERROR(bio)	bio_io_error(bio, PAGE_SIZE)
#define BIO_ENDIO(bio, error)	bio_endio(bio, PAGE_SIZE, error)
#else
#define BIO_IO_ERROR(bio)	bio_io_error(bio)
#define BIO_ENDIO(bio, error)	bio_endio(bio, error)
#endif

#ifndef pr_err
#define pr_err(fmt, arg...) \
	printk(KERN_ERR fmt, ##arg)
#endif

#ifndef pr_warning
#define pr_warning(fmt, arg...) \
	printk(KERN_WARNING fmt, ##arg)
#endif

#ifndef pr_info
#define pr_info(fmt, arg...) \
	printk(KERN_ERR fmt, ##arg)
#endif

#endif

#ifndef _XVMALLOC_COMPAT_H_
#define _XVMALLOC_COMPAT_H_

#endif

