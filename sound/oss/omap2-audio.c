/*
 * linux/drivers/sound/omap-audio.c
 *
 * Common audio handling for the OMAP processors
 *
 * Copyright (C) 2004-2006 Texas Instruments, Inc.
 *
 * Copyright (C) 2000, 2001 Nicolas Pitre <nico@cam.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 * 2004/08/12   Nishanth Menon - Modified to integrate Audio requirements on 
 *                               1610,1710 platforms
 * 2004-11-01   Nishanth Menon - modified to support 16xx and 17xx
 *                               platform multi channel chaining.
 * 2004-11-04   Nishanth Menon - Added support for power management
 * 2004-12-17   Nishanth Menon - Provided proper module handling support
 * 2005-12-30   Nishanth Menon - Further abstraction of data transfers to
 *                               Codec drivers
 */

/***************************** INCLUDES ************************************/

// #include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/semaphore.h>

/* PM headers */
#ifdef CONFIG_PM
#include <linux/notifier.h>
#include <linux/pm.h>
// #include <asm/arch/bus.h>
#ifdef CONFIG_DPM
#include <linux/dpm.h>	/* to specify constraints */
#endif
#endif

#include "omap2-audio-buf-intfc.h"
#include "omap2-audio.h"

/***************************** MACROS ************************************/

/* Change to define if need be */
#undef DEBUG

/* Short Circuit DMA operation for write - needed for new boards */
#undef DMA_SHORT_CIRCUIT

#ifdef DEBUG
#define DPRINTK  printk
#define FN_IN printk("[omap_audio.c:[%s] start\n", __FUNCTION__)
#define FN_OUT(n) printk("[omap_audio.c:[%s] end(%d)\n", __FUNCTION__ , n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(x)
#endif

#define OMAP_AUDIO_NAME		"omap-audio"
#define AUDIO_NBFRAGS_DEFAULT	8
#define AUDIO_FRAGSIZE_DEFAULT	8192

#define AUDIO_ACTIVE(state)	((state)->use_ref)

#ifdef CONFIG_PM
/* WARNING: In MV Code base has DPM which is
 * different from the Bitkeeper Audio code base 
 * Maintaining old code base for flexibility in syncing up with Bitkeeper.
 */
// #define AUDIO_DEV omap_dev
// #define AUDIO_GET_DRV_DATA(ARG) omap_get_drvdata(ARG)
#define AUDIO_DEV platform_device
#define AUDIO_GET_DRV_DATA(ARG) platform_get_drvdata(ARG)
#else
#define AUDIO_DEV device
#define AUDIO_GET_DRV_DATA(ARG) (ARG)->driver_data
#endif

#ifdef DMA_SHORT_CIRCUIT
#define MAX_DMA_SHORT_BUF_SIZE 8192
static unsigned char buf_short_buf[MAX_DMA_SHORT_BUF_SIZE] = { 0 };
#endif


/***************************** MODULES SPECIFIC FUNCTION PROTOTYPES ***********/

static int audio_write(struct file *file, const char *buffer,
		       size_t count, loff_t * ppos);

static int audio_read(struct file *file, char *buffer, size_t count,
		      loff_t * ppos);

static unsigned int audio_poll(struct file *file,
			       struct poll_table_struct *wait);

static int audio_ioctl(struct inode *inode, struct file *file, uint cmd,
		       ulong arg);

static int audio_open(struct inode *inode, struct file *file);

static int audio_release(struct inode *inode, struct file *file);

static int audio_probe(struct AUDIO_DEV *dev);

static int audio_remove(struct AUDIO_DEV *dev);

static void audio_free(struct device *dev);

#ifdef CONFIG_PM
static int audio_suspend(struct AUDIO_DEV *dev, pm_message_t omap_bus_state);

static int audio_resume(struct AUDIO_DEV *dev);

#ifdef CONFIG_DPM
/* Freq scaling functionality with DPM only */
static int audio_scale(struct notifier_block *op, unsigned long level, void * newop);
#endif

#else
/* we need these only on platform device model */
static void audio_shutdown(struct AUDIO_DEV *dev);
#endif

/***************************** Data Structures ********************************/

static int audio_dev_id;

/*
 * The function pointer set to be registered by the codec.
 */
static audio_state_t audio_state = { 0 };

/* File Ops structure */
static struct file_operations omap_audio_fops = {
	.open = audio_open,
	.release = audio_release,
	.write = audio_write,
	.read = audio_read,
	.poll = audio_poll,
	.ioctl = audio_ioctl,
	.owner = THIS_MODULE
};

#ifdef CONFIG_PM
static struct platform_driver omap_audio_driver = {
	.driver = {
		.name = OMAP_AUDIO_NAME,
		},
	.probe = audio_probe,
	.suspend = audio_suspend,
	.resume = audio_resume,
	.remove = audio_remove,
};

#ifdef CONFIG_DPM
static struct constraints omap_audio_constraints = {
	.count = 2,
	/* TODO: Relook with DPM */
	.param = {
		  {DPM_MD_V, OMAP24XX_V_MIN, OMAP24XX_V_MAX},
		  {DPM_MD_SLEEP_MODE, PM_SUSPEND_STANDBY, PM_SUSPEND_MEM},
		  },
};
#endif

static struct platform_device omap_audio_device = {
	.name = OMAP_AUDIO_NAME,
	.id = 7,
	.dev = {
		/* We might add additional things in future.. */
#ifdef CONFIG_DPM
		.constraints = &omap_audio_constraints,
#endif
		.release = audio_free,
		},
};

#ifdef CONFIG_DPM
static struct notifier_block omap_audio_pre_scale = {
	         .notifier_call = audio_scale,
#ifdef CONFIG_DPM_SCALING_STATS
                .dpm_scale_id = DPM_SCALE_AUDIO,
#endif
};

static struct notifier_block omap_audio_post_scale = {
	         .notifier_call = audio_scale,
#ifdef CONFIG_DPM_SCALING_STATS
                .dpm_scale_id = DPM_SCALE_AUDIO,
#endif
};
#endif

#else
/*  act as a LDM Legacy device */
/* Driver information */
static struct device_driver omap_audio_driver = {
	.name = OMAP_AUDIO_NAME,
	.bus = &platform_bus_type,
	.probe = audio_probe,
	.remove = audio_remove,
	.shutdown = audio_shutdown,
};

/* Device Information */
static struct platform_device omap_audio_device = {
	.name = OMAP_AUDIO_NAME,
	.dev = {
		.driver_data = &audio_state,
		.release = audio_free,
		},
	.id = 0,
};

#endif

/***************************** GLOBAL FUNCTIONs *******************************/

/** 
 * @brief audio_free - The Audio driver release function
 * This is a dummy function required.
 * 
 * @param dev 
 */
static void audio_free(struct device *dev)
{
	/* Nothing to Release! */
}

/* Power Management Functions for Linux Device Model  */
#ifdef CONFIG_PM
/** 
 * @brief audio_ldm_suspend - Suspend operation
 * 
 * @param data 
 * 
 * @return 
 */
static int audio_ldm_suspend(void *data)
{
	audio_state_t *state = data;
	int ret = 0;

	FN_IN;

	if (AUDIO_ACTIVE(state) && state->hw_init) {
		audio_stream_t *is = state->input_stream;
		audio_stream_t *os = state->output_stream;

		if (os && os->buffers) {
#ifdef AUDIO_DRAIN_ON_SUSPEND
			audio_sync(state, os);
#endif
			ret = state->hw_transfer_stop(os);
			if (ret) {
				printk(KERN_ERR
				       "AUDIO: Suspend failed for Tx Stop\n");
				goto suspend_fail;
			}
			audio_reset(state, os);
		}
		if (is && is->buffers) {
			/* clean up stream usage */
			/* we need to set the inuse to 0
			 * here so that we can do prevent further transfers
			 * from going ahead at this point
			 */
			is->in_use = 0;
			ret = state->hw_transfer_stop(is);
			if (ret) {
				printk(KERN_ERR
				       "AUDIO: Suspend failed for Rx Stop\n");
				goto suspend_fail;
			}
			audio_reset(state, is);

		}
		ret = audio_state.hw_suspend(state);
		if (ret) {
			printk(KERN_ERR "AUDIO: Suspend failed for codec\n");
			goto suspend_fail;
		}
	}

      suspend_fail:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief audio_ldm_resume - Resume Operations
 * Strategy: We empty the audio buffer in suspend - DMAs stop as a result of 
 *           this. This allows the writes (DMA start) to start by already 
 *           present mechanisms. Hence DMA need not be enabled in this flow
 * 
 * @param data 
 * 
 * @return 
 */
static int audio_ldm_resume(void *data)
{
	audio_state_t *state = data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	int ret = 0;

	FN_IN;
	if (AUDIO_ACTIVE(state) && state->hw_init) {
		ret = audio_state.hw_resume(state);
		if (ret) {
			printk(KERN_ERR "AUDIO: Resume failed for codec\n");
			goto resume_fail;
		}
		if (os && os->buffers) {
			ret = state->hw_transfer_init(state, os,
						      audio_buf_irq_handler);
			if (ret) {
				printk(KERN_ERR
				       "AUDIO: Resume failed for tx init\n");
				goto resume_fail;
			}
		}

		if (is && is->buffers) {
			/* Prime the RX for recieve mode again.. */
			ret = state->hw_transfer_init(state, is,
						      audio_buf_irq_handler);
			if (ret) {
				printk(KERN_ERR
				       "AUDIO: Resume failed for rx init\n");
				goto resume_fail;
			}
			/* Set the suspended state to 0 here
			 * Reason: audio_process_buf - which initiates a transfer
			 * will not start otherwise. we need to initate a transfer
			 * so that the recieve queue will be active
			 */
			state->suspended = 0;
			audio_process_buf(state, is);
		}
	}

      resume_fail:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief audio_suspend - Function to handle suspend operations
 * 
 * @param dev 
 * @param state 
 * 
 * @return 
 */
// static int audio_suspend(struct AUDIO_DEV *dev, u32 state)
static int audio_suspend(struct AUDIO_DEV *dev, pm_message_t omap_bus_state)
{
	int ret = 0;
	void *data = AUDIO_GET_DRV_DATA(dev);
	audio_state_t *state;

	FN_IN;
	DPRINTK("%s: \n", __FUNCTION__);

	state = data;

	DPRINTK("audio_suspend: Received suspend level 0x%x \n",omap_bus_state);

	/* TODO -- verify with PM */
#if 0
	// if (omap_bus_state != DEV_SUSPEND_OFF) {
	if (omap_bus_state != SNDRV_CTL_POWER_D3) {
		if (AUDIO_ACTIVE(state) && state->hw_init) {
			/* pass the Idle level to the codec driver */
			if (state->hw_sidle) {
				state->hw_sidle(state, omap_bus_state);
			}
		} 
	}
	else 
#endif
	{
		/* need to work along with freq scaling too */
		if (audio_state.suspended == 0) {
			audio_state.suspended = 1;
			if (audio_state.hw_suspend) {
				ret = audio_ldm_suspend(data);
			}
		}
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief audio_resume - Function to handle resume operations
 * 
 * @param dev 
 * 
 * @return 
 */
static int audio_resume(struct AUDIO_DEV *dev)
{
	int ret = 0;
	void *data = AUDIO_GET_DRV_DATA(dev);

	FN_IN;
	DPRINTK("%s: woken up\n", __FUNCTION__);
	/* Woke up all, now wake up clients */
	if (audio_state.suspended == 1) {
		if (audio_state.hw_resume) {
			ret = audio_ldm_resume(data);
		}
		audio_state.suspended = 0;
		wake_up(&audio_state.suspend_wq);
	}

	FN_OUT(ret);
	return ret;
}

#ifdef CONFIG_DPM
/** 
 * @brief audio_scale - DPM Scale operation
 * 
 * @param op 
 * 
 * @return 
 */
static int audio_scale(struct notifier_block *op, unsigned long level, void * newop)
{
	int ret = 0;
	FN_IN;

	/* Bother only if data stream is active */
	if (!(AUDIO_ACTIVE(&audio_state) && audio_state.hw_init)) {
		DPRINTK("Audio Not Being Used.\n");
		FN_OUT(ret);
		return (ret);
	}

	switch (level) {
	case SCALE_PRECHANGE:
		DPRINTK("NOTIFIED\n");
		/* Go into suspended mode if not already suspended 
		 * - we dont want buf to be running 
		 * Standard stuff we can do in prescaling 
		 */
		if (audio_state.suspended == 0) {
			audio_state.suspended = 1;
			if (audio_state.hw_prescale) {
				ret = audio_state.hw_prescale();
			}
		}
		break;

	case SCALE_POSTCHANGE:
		DPRINTK("SCALED\n");
		if (audio_state.suspended == 1) {
			if (audio_state.hw_postscale) {
				ret = audio_state.hw_postscale();
			}
			audio_state.suspended = 0;
			wake_up(&audio_state.suspend_wq);
		}
		break;
	}
	FN_OUT(0);
	return 0;
}
#endif                          /* CONFIG_DPM */


#endif				/* CONFIG_PM */

/** 
 * @brief audio_probe - The Audio driver probe function
 * WARNING!!!!  : It is expected that the codec would have registered with us 
 *                by now
 * 
 * @param dev 
 * 
 * @return 
 */
static int audio_probe(struct AUDIO_DEV *dev)
{
	int ret;

	FN_IN;
	if (!audio_state.hw_probe) {
		printk(KERN_ERR "Probe Function Not Registered\n");
		return -ENODEV;
	}
	/* The -1 means that we get the next available dsp device */
	ret = audio_dev_id = register_sound_dsp(&omap_audio_fops, -1);
	if (audio_dev_id > 0) {
		ret = audio_state.hw_probe();
		if (ret) {
			unregister_sound_dsp(audio_dev_id);
		}
	}
	if (ret) {
		printk(KERN_ERR
		       "AUDIO: Registration of device or probe of codec "
		       "failed[%d]\n", ret);
	}

	FN_OUT(ret);
	return ret;
}

/** 
 * @brief audio_remove - Function to handle removal operations
 * 
 * @param dev 
 * 
 * @return 
 */
static int audio_remove(struct AUDIO_DEV *dev)
{
	FN_IN;
	if (audio_state.hw_remove) {
		audio_state.hw_remove();
	}
	/* Un-Register the codec with the audio driver */
	unregister_sound_dsp(audio_dev_id);
	FN_OUT(0);
	return 0;
}

#ifndef CONFIG_PM


/** 
 * @brief audio_shutdown - Function to handle shutdown operations
 * 
 * @param dev 
 */
static void audio_shutdown(struct AUDIO_DEV *dev)
{
	FN_IN;
	if (audio_state.hw_cleanup) {
		audio_state.hw_cleanup();
	}
	FN_OUT(0);
	return;
}

#endif				/* CONFIG_PM */

/** 
 * @brief audio_register_codec - Register a Codec fn points using this function
 * WARNING!!!!!          : Codecs should ensure that they do so! no sanity 
 *                         checks during runtime is done due to obvious 
 *                         performance penalties.
 * 
 * @param codec_state 
 * 
 * @return 
 */
int audio_register_codec(audio_state_t * codec_state)
{
	int ret;

	FN_IN;

	/* We dont handle multiple codecs now */
	if (audio_state.hw_init) {
		printk(KERN_ERR " Codec Already registered\n");
		return -EPERM;
	}

	/* Sanity checks */
	if (!codec_state) {
		printk(KERN_ERR "NULL ARGUMENT!\n");
		return -EPERM;
	}

	if (!codec_state->hw_probe || !codec_state->hw_init
	    || !codec_state->hw_shutdown || !codec_state->client_ioctl ||
	    !codec_state->hw_transfer ||
	    !codec_state->hw_transfer_stop ||
	    !codec_state->hw_transfer_posn || !codec_state->hw_transfer_init) {
		printk(KERN_ERR
		       "Required Fn Entry point Missing probe=%p init=%p,"
		       "down=%p,ioctl=%p transfer=%p tr_stop=%p tr_pos=%p tr_init=%p!\n",
		       codec_state->hw_probe, codec_state->hw_init,
		       codec_state->hw_shutdown, codec_state->client_ioctl,
		       codec_state->hw_transfer,
		       codec_state->hw_transfer_stop,
		       codec_state->hw_transfer_posn,
		       codec_state->hw_transfer_init);
		return -EPERM;
	}

	memcpy(&audio_state, codec_state, sizeof(audio_state_t));

#ifdef CONFIG_PM
	
#ifdef CONFIG_DPM
	/* Scale on DPM */
	dpm_register_scale(&omap_audio_pre_scale,SCALE_PRECHANGE);
	dpm_register_scale(&omap_audio_post_scale,SCALE_POSTCHANGE);
#endif

	ret = platform_driver_register(&omap_audio_driver);
	if (ret != 0) {
		printk(KERN_ERR "OMAP Driver Register failed =%d\n", ret);
		ret = -ENODEV;
		goto register_out;
	}
	ret = platform_device_register(&omap_audio_device);
	if (ret != 0) {
		printk(KERN_ERR "OMAP device register failed =%d\n", ret);
		platform_driver_unregister(&omap_audio_driver);
		ret = -ENODEV;
		goto register_out;
	}
	platform_set_drvdata(&omap_audio_device, &audio_state);
#else
	ret = platform_device_register(&omap_audio_device);
	if (ret != 0) {
		printk(KERN_ERR "Platform dev_register failed =%d\n", ret);
		ret = -ENODEV;
		goto register_out;
	}

	ret = platform_driver_register(&omap_audio_driver);
	if (ret != 0) {
		printk(KERN_ERR "Driver Register failed =%d\n", ret);
		ret = -ENODEV;
		platform_device_unregister(&omap_audio_device);
		goto register_out;
	}
#endif
	audio_state.use_ref = 0;
	sema_init(&audio_state.sem, 1);
#ifdef CONFIG_PM
	/* init the pm vars */
	audio_state.suspended = 0;
	init_waitqueue_head(&audio_state.suspend_wq);
#endif

	if (ret) {
		/* reset the pointers */
		memset(&audio_state, 0, sizeof(audio_state_t));
	}

      register_out:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief audio_unregister_codec - Un-Register a Codec using this function
 * 
 * @param codec_state 
 * 
 * @return 
 */
int audio_unregister_codec(audio_state_t * codec_state)
{
	FN_IN;

	/* We dont handle multiple codecs now */
	if (!audio_state.hw_init) {
		printk(KERN_ERR " No Codec registered\n");
		return -EPERM;
	}
	/* Security check */
	if (audio_state.hw_init != codec_state->hw_init) {
		printk(KERN_ERR
		       " Attempt to unregister codec which was not "
		       "registered with us\n");
		return -EPERM;
	}
	/* refuse to unregister the driver if already being used?? */
	if (AUDIO_ACTIVE(&audio_state)) {
		printk(KERN_WARNING "AUDIO DEVICE IS CURRENTLY ACTIVE...!!\n");
		return -EBUSY;
	}
#ifdef CONFIG_PM
	platform_device_unregister(&omap_audio_device);
	platform_driver_unregister(&omap_audio_driver);

#ifdef CONFIG_DPM
	/* Unregister scale */
	dpm_unregister_scale(&omap_audio_pre_scale,SCALE_PRECHANGE);
	dpm_unregister_scale(&omap_audio_post_scale,SCALE_POSTCHANGE);
#endif /* CONFIG_DPM */

#else /* CONFIG_PM */
	driver_unregister(&omap_audio_driver);
	platform_device_unregister(&omap_audio_device);
#endif

	memset(&audio_state, 0, sizeof(audio_state_t));

	FN_OUT(0);
	return 0;
}

/************************** MODULES SPECIFIC FUNCTION *************************/

/** 
 * @brief audio_write - Exposed to write() call
 * 
 * @param file 
 * @param buffer 
 * @param count 
 * @param ppos 
 * 
 * @return 
 */
static int
audio_write(struct file *file, const char *buffer, size_t count, loff_t * ppos)
{
	const char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->output_stream;
	int chunksize, ret = 0;

	DPRINTK("audio_write: count=%d\n", count);
	if (*ppos != file->f_pos) {
		printk(KERN_ERR "FPOS not ppos ppos=0x%x fpos =0x%x\n",
		       (u32) * ppos, (u32) file->f_pos);
		return -ESPIPE;
	}
	audio_suspend_lockout(state, file);

	if (s->mapped) {
		printk(KERN_ERR "s already mapped\n");
		return -ENXIO;
	}
	if (!s->buffers && audio_setup_buf(state, s)) {
		printk(KERN_ERR "NO MEMORY\n");
		return -ENOMEM;
	}
#ifdef DMA_SHORT_CIRCUIT
	/* DEBUG CODE */
	/* Logic to short circuit DMA- We want to leave this for the future platforms */
	{
		int count_1 = 0;
		int count_2 = count;

		mcbsp_hack_reset(0);
		mcbsp_hack_reset(1);
		while (count) {
			int i = 0;

			count_1 =
			    (count >
			     MAX_DMA_SHORT_BUF_SIZE) ? MAX_DMA_SHORT_BUF_SIZE
			    : count;
			if (copy_from_user(buf_short_buf, buffer, count_1)) {
				printk(KERN_ERR
				       "Audio: CopyFrom User failed \n");
				mcbsp_hack_reset(0);
				return -EFAULT;
			}
			i = 0;
			while (i < count_1) {
				u16 val = buf_short_buf[i];

				val |= buf_short_buf[i + 1] << 8;
				if ((ret = omap_mcbsp_pollwrite(1, val))) {
					printk(KERN_ERR
					       "Unable to copy to mcbsp=%d\n",
					       ret);
					mcbsp_hack_reset(0);
					return ret;
				}
				i += 2;
			}
			count -= count_1;
			buffer += count_1;
		}
		if ((buffer - buffer0))
			ret = buffer - buffer0;
		mcbsp_hack_reset(0);
		return count_2;
	}
#endif				/* end of DMA short circuit */
	while (count > 0) {
		audio_buf_t *b = &s->buffers[s->usr_head];

		/* Wait for a buffer to become free */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&s->sem))
				break;
		} else {
			ret = audio_timed_get_sem(&s->sem);
			if (ret) {
				printk(KERN_ERR "audio_write: timedout\n");
				break;
			}
		}
		audio_suspend_lockout(state, file);
		/* Check if the semaphore already got reset while we were in suspended state */
		if (atomic_read(&s->sem.count) == s->nbfrags) {
			/* Wait for a buffer to become free */
			if (file->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				if (down_trylock(&s->sem))
					break;
			} else {
				ret = audio_timed_get_sem(&s->sem);
				if (ret) {
					printk(KERN_ERR
					       "audio_write: timedout\n");
					break;
				}
			}
		}

		/* Feed the current buffer */
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		DPRINTK("write %d to %d\n", chunksize, s->usr_head);
		if (copy_from_user(b->data + b->offset, buffer, chunksize)) {
			printk(KERN_ERR "Audio: CopyFrom User failed \n");
			up(&s->sem);
			return -EFAULT;
		}

		buffer += chunksize;
		count -= chunksize;
		b->offset += chunksize;

		if (b->offset < s->fragsize) {
			up(&s->sem);
			break;
		}

		/* Update pointers and send current fragment to DMA */
		b->offset = 0;
		if (++s->usr_head >= s->nbfrags)
			s->usr_head = 0;
		/* Add the num of frags pending */
		s->pending_frags++;
		s->active = 1;

		if (audio_process_buf(state, s)) {
			printk(KERN_ERR "Process_buf failed\n");
		}

	}

	if ((buffer - buffer0))
		ret = buffer - buffer0;
	DPRINTK("audio_write: return=%d\n", ret);
	return ret;

}

/** 
 * @brief audio_read - Exposed as read() function
 * 
 * @param file 
 * @param buffer 
 * @param count 
 * @param ppos 
 * 
 * @return 
 */
static int
audio_read(struct file *file, char *buffer, size_t count, loff_t * ppos)
{
	char *buffer0 = buffer;
	audio_state_t *state = file->private_data;
	audio_stream_t *s = state->input_stream;
	int chunksize, ret = 0;
	unsigned long flags;

	DPRINTK("audio_read: count=%d\n", count);

	if (*ppos != file->f_pos) {
		printk(KERN_ERR
		       "AudioRead - FPOS not ppos ppos=0x%x fpos =0x%x\n",
		       (u32) * ppos, (u32) file->f_pos);
		return -ESPIPE;
	}
#ifdef DMA_SHORT_CIRCUIT
	/* DEBUG CODE */
	/* Logic to short circuit DMA- We want to leave this for the future platforms */
	{
		extern void mcbsp_hack_reset(int x);
		int count_1 = 0;
		int count_2 = count;
		mcbsp_hack_reset(0);
		mcbsp_hack_reset(1);

		while (count) {
			int i = 0;

			count_1 =
			    (count >
			     MAX_DMA_SHORT_BUF_SIZE) ? MAX_DMA_SHORT_BUF_SIZE
			    : count;
			i = 0;
			while (i < count_1) {
				if ((ret = omap_mcbsp_pollread(1)) < 0) {
					printk(KERN_ERR
					       "Unable to read from mcbsp=%d\n",
					       ret);
					mcbsp_hack_reset(0);
					return ret;
				}
				buf_short_buf[i] = (ret & 0xFF);

				buf_short_buf[i + 1] = (ret & 0xFF00) >> 8;

				i += 2;
			}
			if (copy_to_user(buffer, buf_short_buf, count_1)) {
				mcbsp_hack_reset(0);
				printk(KERN_ERR
				       "Audio: Copy TO User failed \n");
				return -EFAULT;
			}
			count -= count_1;
			buffer += count_1;
		}
		if ((buffer - buffer0))
			ret = buffer - buffer0;
		mcbsp_hack_reset(0);
		return count_2;
	}
#endif				/* end of DMA short circuit */
	audio_suspend_lockout(state, file);
	if (s->mapped) {
		printk(KERN_ERR "AudioRead - s already mapped\n");
		return -ENXIO;
	}

	if (!s->active) {
		if (!s->buffers && audio_setup_buf(state, s)) {
			printk(KERN_ERR "AudioRead - No Memory\n");
			return -ENOMEM;
		}
		audio_prime_rx(state);
	}

	while (count > 0) {
		audio_buf_t *b = &s->buffers[s->usr_head];

		/* Wait for a buffer to become full */
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			if (down_trylock(&s->sem))
				break;
		} else {
			ret = -ERESTARTSYS;
			if (audio_timed_get_sem(&s->sem)) {
				printk(KERN_ERR "audio_write: timedout\n");
				break;
			}
		}
		ret = 0;
		/* if we are already in process of recieving buffer, 
		 * then wait before submitting the next buffer 
		 */
		audio_suspend_lockout(state, file);
		/* Did we get reset while we were suspended.. no real callbacks!! */
		while (s->fragcount == 0) {
			DPRINTK
			    ("while suspended.. got killed -getting again\n");
			/* Wait for a buffer to become full */
			if (file->f_flags & O_NONBLOCK) {
				ret = -EAGAIN;
				if (down_trylock(&s->sem))
					break;
				ret = 0;
			} else {
				ret = audio_timed_get_sem(&s->sem);
				if (ret) {
					printk(KERN_ERR
					       "audio_write: timedout\n");
					break;
				}
			}
			audio_suspend_lockout(state, file);
		}
		/* if timed out */
		if (ret) {
			break;
		}

		/* Grab data from the current buffer */
		chunksize = s->fragsize - b->offset;
		if (chunksize > count)
			chunksize = count;
		DPRINTK("read %d from %d\n", chunksize, s->usr_head);
		if (copy_to_user(buffer, b->data + b->offset, chunksize)) {
			up(&s->sem);
			return -EFAULT;
		}
		buffer += chunksize;
		count -= chunksize;
		b->offset += chunksize;
		if (b->offset < s->fragsize) {
			up(&s->sem);
			break;
		}

		/* Update pointers and return current fragment to DMA */
		local_irq_save(flags);
		b->offset = 0;
		if (++s->usr_head >= s->nbfrags)
			s->usr_head = 0;

		s->pending_frags++;
		local_irq_restore(flags);
		audio_process_buf(state, s);
	}
	if ((buffer - buffer0))
		ret = buffer - buffer0;
	DPRINTK("audio_read: return=%d\n", ret);
	return ret;
}

/** 
 * @brief audio_poll - Exposed as poll function
 * 
 * @param file 
 * @param wait 
 * 
 * @return 
 */
static unsigned int
audio_poll(struct file *file, struct poll_table_struct *wait)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *is = state->input_stream;
	audio_stream_t *os = state->output_stream;
	unsigned int mask = 0;

	DPRINTK("audio_poll(): mode=%s%s\n",
		(file->f_mode & FMODE_READ) ? "r" : "",
		(file->f_mode & FMODE_WRITE) ? "w" : "");

	audio_suspend_lockout(state, file);

	if (file->f_mode & FMODE_READ) {
		/* Start audio input if not already active */
		if (!is->active) {
			if (!is->buffers && audio_setup_buf(state, is))
				return -ENOMEM;
			audio_prime_rx(state);
		}
		poll_wait(file, &is->wq, wait);
	}

	if (file->f_mode & FMODE_WRITE) {
		if (!os->buffers && audio_setup_buf(state, os))
			return -ENOMEM;
		poll_wait(file, &os->wq, wait);
	}

	if (file->f_mode & FMODE_READ)
		if ((is->mapped && is->bytecount > 0) ||
		    (!is->mapped && atomic_read(&is->sem.count) > 0))
			mask |= POLLIN | POLLRDNORM;

	if (file->f_mode & FMODE_WRITE)
		if ((os->mapped && os->bytecount > 0) ||
		    (!os->mapped && atomic_read(&os->sem.count) > 0))
			mask |= POLLOUT | POLLWRNORM;

	DPRINTK("audio_poll() returned mask of %s%s\n",
		(mask & POLLIN) ? "r" : "", (mask & POLLOUT) ? "w" : "");

	FN_OUT(mask);
	return mask;
}

/** 
 * @brief audio_ioctl - Handles generic ioctls. If there is a request for something 
 *                this fn cannot handle, its then given to client specific 
 *                ioctl routine, that will take up platform specific requests
 * 
 * @param inode 
 * @param file 
 * @param cmd 
 * @param arg 
 * 
 * @return 
 */
static int
audio_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	long val;

	DPRINTK(__FILE__ " audio_ioctl 0x%08x\n", cmd);

	audio_suspend_lockout(state, file);

	/* dispatch based on command */
	switch (cmd) {
	case OSS_GETVERSION:
		return put_user(SOUND_VERSION, (int *)arg);

	case SNDCTL_DSP_GETBLKSIZE:
		if (file->f_mode & FMODE_WRITE)
			return put_user(os->fragsize, (int *)arg);
		else
			return put_user(is->fragsize, (int *)arg);

	case SNDCTL_DSP_GETCAPS:
		val = DSP_CAP_REALTIME | DSP_CAP_TRIGGER;
		if (is && os)
			val |= DSP_CAP_DUPLEX;
		FN_OUT(1);
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETFRAGMENT:
		if (get_user(val, (long *)arg)) {
			FN_OUT(2);
			return -EFAULT;
		}
		if (file->f_mode & FMODE_READ) {
			int ret = audio_set_fragments(state, is, val);

			if (ret < 0) {
				FN_OUT(3);
				return ret;
			}
			ret = put_user(ret, (int *)arg);
			if (ret) {
				FN_OUT(4);
				return ret;
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			int ret = audio_set_fragments(state, os, val);

			if (ret < 0) {
				FN_OUT(5);
				return ret;
			}
			ret = put_user(ret, (int *)arg);
			if (ret) {
				FN_OUT(6);
				return ret;
			}
		}
		FN_OUT(7);
		return 0;

	case SNDCTL_DSP_SYNC:
		{
			int ret = 1;

			FN_OUT(8);
			if (file->f_mode & FMODE_WRITE) {
				ret = audio_sync(state, os);
				state->user_sync = 1;	/* user controlled sync */
			}
			return ret;
		}
	case SNDCTL_DSP_SETDUPLEX:
		FN_OUT(9);
		return 0;

	case SNDCTL_DSP_POST:
		FN_OUT(10);
		return 0;

	case SNDCTL_DSP_GETTRIGGER:
		val = 0;
		if (file->f_mode & FMODE_READ && is->active && !is->stopped)
			val |= PCM_ENABLE_INPUT;
		if (file->f_mode & FMODE_WRITE && os->active && !os->stopped)
			val |= PCM_ENABLE_OUTPUT;
		FN_OUT(11);
		return put_user(val, (int *)arg);

	case SNDCTL_DSP_SETTRIGGER:
		if (get_user(val, (int *)arg)) {
			FN_OUT(12);
			return -EFAULT;
		}
		if (file->f_mode & FMODE_READ) {
			if (val & PCM_ENABLE_INPUT) {
				unsigned long flags;

				if (!is->active) {
					if (!is->buffers
					    && audio_setup_buf(state, is)) {
						FN_OUT(13);
						return -ENOMEM;
					}
					audio_prime_rx(state);
				}
				local_irq_save(flags);
				is->stopped = 0;
				local_irq_restore(flags);
				audio_process_buf(state, is);

			} else {
				state->hw_transfer_stop(is);
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			if (val & PCM_ENABLE_OUTPUT) {
				unsigned long flags;

				if (!os->buffers && audio_setup_buf(state, os)) {
					FN_OUT(14);
					return -ENOMEM;
				}
				local_irq_save(flags);
				if (os->mapped && !os->pending_frags) {
					os->pending_frags = os->nbfrags;
					sema_init(&os->sem, 0);
					os->active = 1;
				}
				os->stopped = 0;
				local_irq_restore(flags);
				audio_process_buf(state, os);

			} else {
				state->hw_transfer_stop(os);
			}
		}
		FN_OUT(15);
		return 0;

	case SNDCTL_DSP_GETOPTR:
	case SNDCTL_DSP_GETIPTR:
		{
			count_info inf = { 0, };
			audio_stream_t *s =
			    (cmd == SNDCTL_DSP_GETOPTR) ? os : is;
			int bytecount, offset;
			unsigned long flags;

			if ((s == is && !(file->f_mode & FMODE_READ)) ||
			    (s == os && !(file->f_mode & FMODE_WRITE))) {
				FN_OUT(16);
				return -EINVAL;
			}
			if (s->active) {
				local_irq_save(flags);
				offset = audio_get_buf_pos(state, s);
				inf.ptr = s->buf_tail * s->fragsize + offset;
				bytecount = s->bytecount + offset;
				s->bytecount = -offset;
				inf.blocks = s->fragcount;
				s->fragcount = 0;
				local_irq_restore(flags);
				if (bytecount < 0)
					bytecount = 0;
				inf.bytes = bytecount;
			}
			FN_OUT(17);
			return copy_to_user((void *)arg, &inf, sizeof(inf));
		}

	case SNDCTL_DSP_GETOSPACE:
	case SNDCTL_DSP_GETISPACE:
		{
			audio_buf_info inf = { 0, };
			audio_stream_t *s =
			    (cmd == SNDCTL_DSP_GETOSPACE) ? os : is;

			if ((s == is && !(file->f_mode & FMODE_READ)) ||
			    (s == os && !(file->f_mode & FMODE_WRITE))) {
				FN_OUT(18);
				return -EINVAL;
			}
			if (!s->buffers && audio_setup_buf(state, s)) {
				FN_OUT(19);
				return -ENOMEM;
			}
			inf.bytes = atomic_read(&s->sem.count) * s->fragsize;

			inf.fragments = inf.bytes / s->fragsize;
			inf.fragsize = s->fragsize;
			inf.fragstotal = s->nbfrags;
			FN_OUT(20);
			return copy_to_user((void *)arg, &inf, sizeof(inf));
		}

	case SNDCTL_DSP_NONBLOCK:
		file->f_flags |= O_NONBLOCK;
		FN_OUT(21);
		return 0;

	case SNDCTL_DSP_RESET:
		if (file->f_mode & FMODE_READ) {
			audio_reset(state, is);
			if (state->need_tx_for_rx) {
				audio_reset(state, os);
			}
		}
		if (file->f_mode & FMODE_WRITE) {
			audio_reset(state, os);
		}
		FN_OUT(22);
		return 0;

	default:
		/*
		 * Let the client of this module handle the
		 * non generic ioctls
		 */
		FN_OUT(23);
		return state->client_ioctl(inode, file, cmd, arg);
	}

	FN_OUT(0);
	return 0;
}

/** 
 * @brief audio_open - Exposed as open() function
 * 
 * @param inode 
 * @param file 
 * 
 * @return 
 */
static int audio_open(struct inode *inode, struct file *file)
{
	audio_state_t *state = (&audio_state);
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;
	int err, need_tx_buf;

	FN_IN;

	if (!state) {
		printk(KERN_ERR "Audio: no device!!\n");
		return -ENODEV;
	}

	audio_suspend_lockout(state, file);

	/* Lock the module */
	if (!try_module_get(THIS_MODULE)) {
		printk(KERN_CRIT "Failed to get module\n");
		return -ESTALE;
	}
	/* Lock the codec module */
	if (!try_module_get(state->owner)) {
		printk(KERN_CRIT "Failed to get codec module\n");
		module_put(THIS_MODULE);
		return -ESTALE;
	}
	down(&state->sem);

	/* access control */
	err = -ENODEV;
	if ((file->f_mode & FMODE_WRITE) && !os)
		goto out1;
	if ((file->f_mode & FMODE_READ) && !is)
		goto out1;
	err = -EBUSY;
	if (state->use_ref)
		goto out1;
	err = -EINVAL;
	if ((file->f_mode & FMODE_READ) && state->need_tx_for_rx && !os)
		goto out1;

	/* request Buffers.. */
	need_tx_buf = ((file->f_mode & FMODE_WRITE) ||
		       ((file->f_mode & FMODE_READ) && state->need_tx_for_rx));

	state->fmode = file->f_mode;
	state->user_sync = 0;
	/* now complete initialisation */
	if (!AUDIO_ACTIVE(state)) {
		if (state->hw_init) {
			err = state->hw_init(state, state->data);
			if (err) {
				printk(KERN_ERR "Unable to initialize "
				       "the codec[%d]\n", err);
				goto out;
			}
		}

	}
	if ((need_tx_buf)
	    && (err =
		state->hw_transfer_init(state, os, audio_buf_irq_handler))) {
		printk(KERN_ERR " OS init failed!! %x\n", err);
		goto out;
	}
	if ((file->f_mode & FMODE_WRITE)) {
		os->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		os->nbfrags = AUDIO_NBFRAGS_DEFAULT;
		audio_reset(state, os);
		os->mapped = 0;
		init_waitqueue_head(&os->wq);
	}

	if (file->f_mode & FMODE_READ) {
		if ((err =
		     state->hw_transfer_init(state, is,
					     audio_buf_irq_handler))) {
			printk(KERN_ERR " IS init failed!! 0x%x[%d]\n", err,
			       err);
			goto out;
		}
		audio_reset(state, is);
		is->fragsize = AUDIO_FRAGSIZE_DEFAULT;
		is->nbfrags = AUDIO_NBFRAGS_DEFAULT;
		is->mapped = 0;
		init_waitqueue_head(&is->wq);
	}

	file->private_data = state;
	err = 0;
	state->use_ref = 1;

      out:
	if (err) {
		if (state->hw_shutdown)
			state->hw_shutdown(state->data);
		module_put(state->owner);
		module_put(THIS_MODULE);
	}
      out1:
	up(&state->sem);
	FN_OUT(err);
	return err;
}

/** 
 * @brief audio_release - Exposed as release function()
 * 
 * @param inode 
 * @param file 
 * 
 * @return 
 */
static int audio_release(struct inode *inode, struct file *file)
{
	audio_state_t *state = file->private_data;
	audio_stream_t *os = state->output_stream;
	audio_stream_t *is = state->input_stream;

	FN_IN;

	audio_suspend_lockout(state, file);

	down(&state->sem);

	if (file->f_mode & FMODE_READ) {
		state->hw_transfer_stop(is);
		audio_discard_buf(state, is);
		if (state->need_tx_for_rx) {
			state->hw_transfer_stop(os);
		}
	}

	if (file->f_mode & FMODE_WRITE) {
		/* if user space app does not control sync */
		if (!state->user_sync)
			audio_sync(state, os);
		audio_reset(state, os);
		audio_discard_buf(state, os);
		state->hw_transfer_stop(os);
	}

	state->use_ref = 0;
	if (!AUDIO_ACTIVE(state)) {
		if (state->hw_shutdown)
			state->hw_shutdown(state->data);
	}

	up(&state->sem);

	module_put(state->owner);
	module_put(THIS_MODULE);

	FN_OUT(0);
	return 0;
}

EXPORT_SYMBOL(audio_register_codec);
EXPORT_SYMBOL(audio_unregister_codec);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Common OSS audio handling for OMAP processors");
MODULE_LICENSE("GPL");
