/*
 * sound/arm/omap/omap2_dasf_host.h
 *
 * OMAP alsa dsp driver
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 */

#ifndef __OMAP_DASF_HOST_H
#define __OMAP_DASF_HOST_H

#include <linux/config.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/initval.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/bus.h>
#include <asm/atomic.h>
#include <asm/semaphore.h>
#include <asm/irq.h>

#define DASF_TIMEOUT		100000

static inline int test_and_set(volatile void *ptr, int val)
{
	int ret = val;
	asm volatile (" swp %0, %0, [%1]":"+r" (ret):"r"(ptr):"memory");
	return (ret);
}

typedef enum {
	ALSA_STRM_OPEN,
	ALSA_STRM_XFER,
	ALSA_STRM_PAUSE,
	ALSA_STRM_TERMINATE,
	ALSA_STRM_STOPPED
} alsa_strm_state_t;

#endif				/* End of #ifndef __OMAP_DASF_HOST_H */
