/*
 * sound/arm/omap/omap2_dasf.c
 *
 * Common ALSA DASF audio handling for the OMAP processors
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 * Based on omap linux open source community alsa driver 
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 *
 */

#include "omap2_dasf_host.h"
#include "omap2_dasf_std.h"
#include "omap2-audio_if.h"
#include "omap2_dasf_mailbox.h"

/*
 * Buffer management for alsa and dma
 */
#define CMD_QUE_DEPTH	16

#define PCM_PLAYBACK_STRMS	16
#define PCM_PLAYBACK_STRMS_PWR 4


typedef struct {
	dasf_trapped_cmd_t cmd;
	/* any call back function pointer here ? */
} dasf_cmd_element_t;

typedef struct {
	int requested;
	int serviced;
	dasf_cmd_element_t cmds[CMD_QUE_DEPTH];
} dasf_cmd_queue_t;

struct omap_alsa_stream {
	int stream_id;		/* numeric identification */
	int active:1;		/* we are using this stream for transfer now */
	int period;		/* current transfer period */
	int periods;		/* current count of periods registerd wth the DASF */
	int offset;		/* store start position of the last period in the alsa buffer */
	snd_pcm_substream_t *stream;	/* the pcm stream */
	int en_period;		/* current period serviced by the DASF */
	int en_periods;		/* current count of periods serviced by the DASF */
	alsa_strm_state_t state;
	int gain;
	boolean_t mute;
};

struct omap_alsa_state {
	snd_card_t *card;
	snd_pcm_t *pcm;
	long samplerate;
	dasf_cmd_queue_t cmd_queue;

	struct omap_alsa_stream s[PCM_PLAYBACK_STRMS+1];	/* playback & capture */
	struct omap_alsa_codec *codec;
#ifdef CONFIG_PM
	int suspended;
	wait_queue_head_t suspend_wq;
#endif
};

/***************************** MACROS ************************************/
/* Change to define if need be */
#define DEBUG
// #undef DEBUG

#ifdef DEBUG
#define DPRINTK  printk
#define FN_IN printk("[omap_dasf.c:[%s] start\n", __FUNCTION__)
#define FN_OUT(n) printk("[omap_dasf.c:[%s] end(%d)\n", __FUNCTION__ , n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(x)
#endif

#define OMAP_AUDIO_NAME		"omap-dasf"

#ifdef CONFIG_PM
#define alsa_suspend_lockout(s,f) \
	if ((s)->suspended) {\
		if ((f)->f_flags & O_NONBLOCK)\
			return -EBUSY;\
		DPRINTK("%s[%d]: Waiting for suspend completion\n",__FUNCTION__,__LINE__);\
		wait_event((s)->suspend_wq,\
		(s)->suspended == 0);\
		DPRINTK("%s[%d]: wait done (susp=%d)\n",__FUNCTION__,__LINE__,s->suspended);\
	}
#else
#define audio_suspend_lockout(s, f) do {} while(0)
#endif

static int audio_probe(struct omap_dev *dev);
static int audio_remove(struct omap_dev *dev);
static void audio_release(struct device *dev);
static int asnd_pcm_free_vmalloc_buffer(struct snd_pcm_substream *subs);
static int asnd_pcm_alloc_vmalloc_buffer(struct snd_pcm_substream *subs,
					 size_t size);
#ifdef CONFIG_PM
static int audio_suspend(struct omap_dev *dev, u32 state);
static int audio_resume(struct omap_dev *dev);
#endif
static void run_work_queue(struct omap_alsa_state *state);
static void audio_period_handler(struct omap_alsa_stream *s);

int driver_major = 0;
int driver_minor = 0;
char *driver_name = "alsaDasf";
struct dasf_dev {
	struct cdev cdev;
};

struct dasf_dev *dasf_device = NULL;
static struct class *dasf_class = NULL;

void *dasf_drv_buf_region_beg;
void *dasf_drv_pb_queue;
void *dasf_drv_cap_queue;

static dasf_mailbox_t dasf_mailbox;

static struct work_struct cmd_work;


/* TODO: no suspend/resume until we have CONFIG_PM=y */
static struct omap_driver omap_audio_driver = {
	.drv = {
		.name = OMAP_AUDIO_NAME,
		},
	.devid = OMAP24xx_AUDIO_DEVID,
	.busid = OMAP_BUS_L3,
	.clocks = 0,
	.probe = audio_probe,
#ifdef CONFIG_PM
	.suspend = audio_suspend,
	.resume = audio_resume,
#endif
	.remove = audio_remove,
};

static struct omap_dev omap_audio_device = {
	.name = OMAP_AUDIO_NAME,
	.devid = OMAP24xx_AUDIO_DEVID,
	.busid = OMAP_BUS_L3,
	.dev = {
		/* We might add additional things in future.. */
		.release = audio_release,
		},
};

static struct omap_alsa_state *omap_audio_state;
#ifdef CONFIG_PM

/** 
 * @brief audio_suspend - Function to handle suspend operations
 * 
 * @param dev 
 * @param state 
 * 
 * @return 
 */
static int audio_suspend(struct omap_dev *dev, u32 state)
{
        struct omap_alsa_state *alsa_state;
        snd_card_t *card = omap_get_drvdata(dev);
        alsa_state = card->private_data;

        if (DEV_SUSPEND_OFF ==  state) {
                if (card->power_state != SNDRV_CTL_POWER_D3) {
                        snd_power_change_state(alsa_state->card,
					       SNDRV_CTL_POWER_D3);
                        snd_pcm_suspend_all(alsa_state->pcm);
                }
                alsa_state->suspended = 1;
        }
        return 0;
}

/** 
 * @brief audio_resume - Function to handle resume operations
 * 
 * @param dev 
 * 
 * @return 
 */

static int audio_resume(struct omap_dev *dev)
{
        struct omap_alsa_state *alsa_state;
        snd_card_t *card = omap_get_drvdata(dev);
        alsa_state = card->private_data;

        if (alsa_state->suspended == 1) {
                if (card->power_state != SNDRV_CTL_POWER_D0) {
                        snd_power_change_state(alsa_state->card,
					       SNDRV_CTL_POWER_D0);
                }
                alsa_state->suspended = 0;
                wake_up(&alsa_state->suspend_wq);
        } 
        return 0;
}

#endif

static int dasf_open(struct inode *ip, struct file *filp)
{
	int status = 0;

	FN_IN;

	FN_OUT(status);

	return status;
}

/*
 * Purpose:
 *  This function is called when an application closes handle to the bridge driver.
 */
static int dasf_release(struct inode *ip, struct file *filp)
{
	int status = 0;

	FN_IN;

	FN_OUT(status);

	return status;
}

static ALSA_STATUS dasf_pb_dequeue(dasf_buf_t * dasf_usr_buf_ptr)
{
	ALSA_STATUS status = ALSA_EERROR;
	dasf_buf_t dasf_buf;
	snd_pcm_substream_t *substream;
	snd_pcm_runtime_t *runtime;
	unsigned int dma_size;
	unsigned int offset;
	int stream_id;
	int stream_num;

	struct omap_alsa_state *state;
	struct omap_alsa_codec *codec;
	struct omap_alsa_stream *s;

	substream = (snd_pcm_substream_t *) dasf_usr_buf_ptr->stream_ptr;
	stream_id = substream->pstr->stream;
	stream_num = substream->number;
	state = snd_pcm_substream_chip(substream);
	codec = state->codec;
	s = &state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num];
	runtime = substream->runtime;
     
	if (s->state == ALSA_STRM_XFER) {

		dma_size = frames_to_bytes(runtime, runtime->period_size);
		offset = dma_size * s->period;
		// memcpy(dasf_buf.drvPtr,runtime->dma_area + offset,dma_size);
		dasf_buf.drvPtr = runtime->dma_area + offset;
		dasf_buf.userPtr = NULL;
		dasf_buf.stream_ptr = (void *)substream;
		dasf_buf.size = dma_size;
		consistent_sync(dasf_buf.drvPtr, dma_size, DMA_TO_DEVICE);
		/* copy the buffer to the user space */
		if ((dasf_usr_buf_ptr == NULL)
		    || copy_to_user(dasf_usr_buf_ptr, &dasf_buf,
				    sizeof(dasf_buf_t))) {
			printk
			    ("dasf_pb_dequeue: Failed to copy dequeu buffer desc to user space \n");
			status = -1;
		}
		DPRINTK("dasf_pb_dequeue: Dequed and filled a PB buffer \n");
		s->period++;
		s->period %= runtime->periods;
		s->periods++;
		s->offset = offset;

		status = ALSA_SOK;
	} else if (s->state == ALSA_STRM_PAUSE)
		status = ALSA_ESTRM_PAUSED;

	return status;
}

static ALSA_STATUS dasf_pb_enqueue(dasf_buf_t * dasf_usr_buf_ptr)
{
	ALSA_STATUS status = ALSA_EERROR;
	snd_pcm_substream_t *substream;
	int stream_id;
	int stream_num;
	struct omap_alsa_state *state;
	struct omap_alsa_stream *s;
	snd_pcm_runtime_t *runtime;
	dasf_buf_t dasf_buf;

	copy_from_user(&dasf_buf, (void *)dasf_usr_buf_ptr, sizeof(dasf_buf_t));
	
	substream = (snd_pcm_substream_t *) dasf_buf.stream_ptr;
	stream_id = substream->pstr->stream;
	stream_num = substream->number;
	state = snd_pcm_substream_chip(substream);
	s = &state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num];
	runtime = substream->runtime;

	if (s->active) {
		/* We need to wake up the ALSA audio layer.. i.e 'period_elapsed' */
		if (s->en_periods != 0) {

			s->en_period++;
			s->en_period %= runtime->periods;
		}
		s->en_periods++;
		audio_period_handler(s);
		status = ALSA_SOK;
	}
	return status;
}

static ALSA_STATUS dasf_cap_dequeue(dasf_buf_t * dasf_usr_buf_ptr)
{
	ALSA_STATUS status = ALSA_ENOTIMPL;

	return status;
}

static ALSA_STATUS dasf_cap_enqueue(dasf_buf_t * dasf_usr_buf_ptr)
{
	ALSA_STATUS status = ALSA_ENOTIMPL;

	return status;
}

static ALSA_STATUS dasf_get_command(unsigned long args)
{
	ALSA_STATUS status = ALSA_EERROR;
	dasf_trapped_cmd_t cmd;

	DPRINTK("dasf_get_command, invoked !!! \n");
	status = dasf_get_mailbox_cmd(&dasf_mailbox, &cmd);
	if (status == ALSA_SOK) {
		DPRINTK
		    ("dasf_get_command: Retrieved the mailbox command !!! \n");
		/* copy the command to the OMX user space */
		copy_to_user((void *)args, &cmd, sizeof(dasf_trapped_cmd_t));
	}

	return status;
}

static ALSA_STATUS dasf_put_command(dasf_trapped_cmd_t * cmd_ptr,
				    dasf_trapped_response_t * res_ptr)
{
	ALSA_STATUS status;

	DPRINTK("dasf_put_command : Invoked \n");

	status = dasf_put_mailbox_cmd(&dasf_mailbox, cmd_ptr, res_ptr);
	if (status == ALSA_SOK) {
		DPRINTK
		    ("dasf_put_command: Successfully loaded the command to mailbox !!!! \n");
	}
	return status;
}

static ALSA_STATUS dasf_put_response(unsigned long args)
{
	ALSA_STATUS status = ALSA_EERROR;
	dasf_trapped_response_t response;

	DPRINTK("dasf_put_response : Invoked \n");
	copy_from_user(&response, (void *)args,
		       sizeof(dasf_trapped_response_t));

	status = dasf_put_mailbox_response(&dasf_mailbox, &response);
	if (status == ALSA_SOK) {
		DPRINTK
		    ("dasf_put_response: Successfully loaded the command to mailbox !!!! \n");
	}

	return status;
}

/*
 * Purpose:
 *  This function provides IO interface to the bridge driver.
 */
static int dasf_ioctl(struct inode *ip, struct file *filp, unsigned int code,
		      unsigned long args)
{
	int status = -1;
	DPRINTK("-> ALSA-DASF driver_ioctl 0x%x \n", code);
#ifdef CONFIG_PM
	alsa_suspend_lockout(omap_audio_state, filp);
#endif
	/* depending on the 'cmd/code' perform the opearation */

	switch (code) {
	case DASF_PB_ENQUEUE:
		status = dasf_pb_enqueue((dasf_buf_t *) args);
		break;

	case DASF_PB_DEQUEUE:
		status = dasf_pb_dequeue((dasf_buf_t *) args);
		break;

	case DASF_CAP_ENQUEUE:
		status = dasf_cap_enqueue((dasf_buf_t *) args);
		break;

	case DASF_CAP_DEQUEUE:
		status = dasf_cap_dequeue((dasf_buf_t *) args);
		break;

	case DASF_GET_COMMAND:
		status = dasf_get_command(args);
		break;

	case DASF_PUT_RESPONSE:
		status = dasf_put_response(args);
		break;
	default:
		printk("Unrecognized ioctl code from user space \n");
		break;
	}

	DPRINTK("<- ALSA-DASF driver_ioctl 0x%x, status=0x%x \n", code, status);
	return (status);
}

struct file_operations dasf_fops = {
      open:dasf_open,
      release:dasf_release,
      ioctl:dasf_ioctl,
};

int register_alsa_drv(void)
{
	int status;
	dev_t dev = 0;
	int result;

	DPRINTK("Enetered 'register_alsa_drv' \n");
	/* use 2.6 device model */
	if (driver_major) {
		dev = MKDEV(driver_major, driver_minor);
		result = register_chrdev_region(dev, 1, driver_name);
	} else {
		result =
		    alloc_chrdev_region(&dev, driver_minor, 1, driver_name);
		driver_major = MAJOR(dev);
	}

	if (result < 0) {
		printk("register_alsa_drv: Can't get Major %d \n",
		       driver_major);
		return result;
	}

	dasf_device = kmalloc(sizeof(struct dasf_dev), GFP_KERNEL);
	if (!dasf_device) {
		result = -ENOMEM;
		unregister_chrdev_region(dev, 1);
		return result;
	}
	memset(dasf_device, 0, sizeof(struct dasf_dev));
	cdev_init(&dasf_device->cdev, &dasf_fops);
	dasf_device->cdev.owner = THIS_MODULE;
	dasf_device->cdev.ops = &dasf_fops;

	status = cdev_add(&dasf_device->cdev, dev, 1);

	if (status) {
		printk("Failed to add the ALSA DASF device \n");
		return status;
	}

	/* udev support */
	dasf_class = class_create(THIS_MODULE, "ti_dasf");
	if (IS_ERR(dasf_class)) {
		printk(KERN_ERR "Error creating DASF class \n");
	}
	printk("Register_alsa_drv: Created ti_dasf class \n");

	class_device_create(dasf_class, MKDEV(driver_major, driver_minor), NULL,
			    "alsaDasf");
	printk("Register_alsa_drv: Created alsaDasf class \n");

	return status;
}

void unregister_alsa_drv(void)
{
	dev_t	devno;

	printk("Removing the ALSA-DASF driver nodes \n");

	devno = MKDEV(driver_major,driver_minor);

	if (dasf_device) {
		cdev_del(&dasf_device->cdev);
		kfree(dasf_device);
	}
	unregister_chrdev_region(devno,1);

	if (dasf_class) {
		/* remove the device from sysfs */
		class_device_destroy(dasf_class,MKDEV(driver_major,driver_minor));
		class_destroy(dasf_class);
	}
}
	
/* 
 *  This is called when dma IRQ occurs at the end of each transmited block
 */
static void audio_period_handler(struct omap_alsa_stream *s)
{
	FN_IN;
	
	DPRINTK("audio_period_handler invoked for stream 0x%x \n",s->stream);
	snd_pcm_period_elapsed(s->stream);

}

/* 
 * Alsa section
 * PCM settings and callbacks
 */
static int snd_omap_alsa_trigger(snd_pcm_substream_t * substream, int cmd)
{
	int stream_id = substream->pstr->stream;
	int stream_num = substream->number;
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_stream *s = &state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num];
	int err = 0;
	snd_pcm_runtime_t *runtime;
	dasf_trapped_cmd_t trapped_cmd;
	int requested = 0;

	FN_IN;
	/* note local interrupts are already disabled in the midlevel code */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		DPRINTK("SNDDRV_PCM_TRIGGER_START received \n");
		/* requested stream startup */

		if ((stream_id == SNDRV_PCM_STREAM_PLAYBACK) || (SNDRV_PCM_STREAM_CAPTURE == 1)) {
			s->active = 1;
			runtime = substream->runtime;
			/* send the command to the OMX */
			trapped_cmd.cmd = DASF_STREAM_OPEN;
			trapped_cmd.stream_handle = (unsigned int)substream;
			trapped_cmd.args.ARGS_STREAM_OPEN.stream_type =
			    substream->pstr->stream;
			trapped_cmd.args.ARGS_STREAM_OPEN.rate =
			    substream->runtime->rate;
			trapped_cmd.args.ARGS_STREAM_OPEN.channels =
			    substream->runtime->channels;
			trapped_cmd.args.ARGS_STREAM_OPEN.format =
			    substream->runtime->format;
			/* Queue the command and return */
			DPRINTK
			    ("Staring PCM sub-stream ptr=0x%x ,number=0x%x ,stream_id=0x%x ,period_size=0x%x \n",
			     (unsigned int)substream, substream->number, stream_id,
			     (unsigned int)runtime->period_size);
			if (in_atomic())
				DPRINTK
				    ("PCM_TRIGGER_START in 'atomic' mode \n");
			else
				DPRINTK
				    ("PCM_TRIGGER_START NOT in 'atomic' mode !!!!! \n");

			requested = state->cmd_queue.requested;
			state->cmd_queue.cmds[requested].cmd = trapped_cmd;
			requested++;
			if (requested == CMD_QUE_DEPTH)
				requested = 0;

			state->cmd_queue.requested = requested;
			/* schedule the work queue here */
			schedule_work(&cmd_work);
			s->state = ALSA_STRM_XFER;
		} else
			err = -EINVAL;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		DPRINTK("SNDDRV_PCM_TRIGGER_STOP received \n");
		s->active = 0;
		s->state = ALSA_STRM_TERMINATE;
		trapped_cmd.cmd = DASF_STREAM_TERMINATE;
		trapped_cmd.stream_handle = (unsigned int)substream;
		/* Queue the command and return */
		if (in_atomic())
			DPRINTK("PCM_TRIGGER_STOP in 'atomic' mode \n");
		else
			DPRINTK
			    ("PCM_TRIGGER_STOP NOT in 'atomic' mode !!!!! \n");

		requested = state->cmd_queue.requested;
		state->cmd_queue.cmds[requested].cmd = trapped_cmd;
		requested++;
		if (requested == CMD_QUE_DEPTH)
			requested = 0;
		state->cmd_queue.requested = requested;
		/* schedule the work queue here */
		schedule_work(&cmd_work);
		s->active = 0;
		s->period = 0;
		s->periods = 0;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		DPRINTK("SNDDRV_PCM_TRIGGER_PAUSE_PUSH received \n");
		/* TODO Set the stream state to 'Pause' */
		s->state = ALSA_STRM_PAUSE;
		trapped_cmd.cmd = DASF_STREAM_PAUSE;
		trapped_cmd.stream_handle = (unsigned int)substream;
		/* Queue the command and return */
		if (in_atomic())
			DPRINTK("PCM_TRIGGER_PAUSE_PUSH in 'atomic' mode \n");
		else
			DPRINTK
			    ("PCM_TRIGGER_PAUSE_PUSH NOT in 'atomic' mode !!!!! \n");

		requested = state->cmd_queue.requested;
		state->cmd_queue.cmds[requested].cmd = trapped_cmd;
		requested++;
		if (requested == CMD_QUE_DEPTH)
			requested = 0;
		state->cmd_queue.requested = requested;
		/* schedule the work queue here */
		schedule_work(&cmd_work);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		DPRINTK("SNDDRV_PCM_TRIGGER_PAUSE_RELEASE received \n");
		/* TODO Set the stream state to 'Active' */
		trapped_cmd.cmd = DASF_STREAM_RESUME;
		trapped_cmd.stream_handle = (unsigned int)substream;
		/* Queue the command and return */
		if (in_atomic())
			DPRINTK
			    ("PCM_TRIGGER_PAUSE_RELEASE in 'atomic' mode \n");
		else
			DPRINTK
			    ("PCM_TRIGGER_PAUSE_RELEASE NOT in 'atomic' mode !!!!! \n");

		requested = state->cmd_queue.requested;
		state->cmd_queue.cmds[requested].cmd = trapped_cmd;
		requested++;
		if (requested == CMD_QUE_DEPTH)
			requested = 0;
		state->cmd_queue.requested = requested;
		/* schedule the work queue here */
		schedule_work(&cmd_work);
		s->state = ALSA_STRM_XFER;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
		DPRINTK("PCM_TRIGGER_SUSPEND -- invoked \n");
		break;	
	case SNDRV_PCM_TRIGGER_RESUME:
		DPRINTK("PCM_TRIGGER_RESUME -- invoked \n");
		break;	
	default:
		err = -EINVAL;
		break;
	}
	FN_OUT(err);
	return err;
}

static int snd_omap_alsa_prepare(snd_pcm_substream_t * substream)
{
	int stream_id = substream->pstr->stream;
	int stream_num = substream->number;
	
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_stream *s = &state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num];

	FN_IN;

	s->period = 0;
	s->periods = 0;
	s->en_period = 0;
	s->en_periods = 0;
	return 0;
}

static snd_pcm_uframes_t snd_omap_alsa_pointer(snd_pcm_substream_t * substream)
{
	int stream_id = substream->pstr->stream;
	int stream_num = substream->number;
	
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_stream *s = &state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num];
	int period;
	int frames;
	snd_pcm_runtime_t *runtime = substream->runtime;

	period = s->en_period;

	frames = period * (runtime->period_size);
	printk("snd_omap_alsa_pointer, frames position = 0x%x \n", frames);

	return frames;
}

#if 1
static int snd_omap_alsa_copy(snd_pcm_substream_t * substream,int channel, snd_pcm_uframes_t pos,
			      void * src, snd_pcm_uframes_t count)
{
	int stream_id = substream->pstr->stream;
	int stream_num = substream->number;
	
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_stream *s = &state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num];
	int period;
	int frames;
	void * toCopy;

	snd_pcm_runtime_t *runtime = substream->runtime;

	period = s->en_period;
	
	printk("snd_omap_alsa_copy: substream =0x%x, channel=0x%x, pos=0x%x \n",substream,channel,pos);

	toCopy = runtime->dma_area+frames_to_bytes(runtime,pos);

	memcpy(toCopy,src,frames_to_bytes(runtime,count));
	printk("snd_omap_alsa_copy: toCopy=0x%x, src=0x%x, bytes=0x%x \n",toCopy,src,frames_to_bytes(runtime,count));

	return 0;
}
#endif





static int snd_card_omap_alsa_open(snd_pcm_substream_t * substream)
{
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	struct omap_alsa_codec *codec = state->codec;
	snd_pcm_runtime_t *runtime = substream->runtime;
	int stream_id = substream->pstr->stream;
	int stream_num = substream->number;
	int err;

	FN_IN;


	/* Check if we can open the stream */
	if (stream_id == SNDRV_PCM_STREAM_CAPTURE && stream_num > 0)
		return -1;

	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK && stream_num >= PCM_PLAYBACK_STRMS)
	       return -1;	

	codec->codec_init();

	printk
	    ("Opening PCM sub-stream ptr=0x%x ,number=0x%x ,stream_id=0x%x ,period_size=0x%x \n",
	     (unsigned int)substream, substream->number, stream_id,
	     (unsigned int)runtime->period_size);


	state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num].stream = substream;
	if (stream_id == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = *(codec->pcm_hardware_playback);
	} else {
		runtime->hw = *(codec->pcm_hardware_capture);
	}

	if ((err = snd_pcm_hw_constraint_integer(runtime,
						 SNDRV_PCM_HW_PARAM_PERIODS)) <
	    0)
		return err;

	if ((err = snd_pcm_hw_constraint_list(runtime,
					      0,
					      SNDRV_PCM_HW_PARAM_RATE,
					      codec->pcm_hw_constraint_list)) <
	    0)
		return err;

	/* Add a constraint on the priod size and buffer size , this is to take
	 * care of the Cache alignment needed for C64 DSP */
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   128);
	snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				   128);

	state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num].state = ALSA_STRM_OPEN;

	return 0;
}

static int snd_card_omap_alsa_close(snd_pcm_substream_t * substream)
{
	int stream_id = substream->pstr->stream;
	struct omap_alsa_state *state = snd_pcm_substream_chip(substream);
	int stream_num = substream->number;
	struct omap_alsa_codec *codec = state->codec;
	struct omap_alsa_stream *s;
	dasf_trapped_cmd_t trapped_cmd;
	dasf_trapped_response_t trapped_response;

	FN_IN;
	/* check the stream state and based on that send the command to the OMX
	 * daemon process */
	s = &state->s[(stream_id<<PCM_PLAYBACK_STRMS_PWR)+stream_num];
	if ((s->state != ALSA_STRM_OPEN) && (s->state != ALSA_STRM_STOPPED)) {
		/* requested stream shutdown */
		/* send the command to the OMX */
		trapped_cmd.cmd = DASF_STREAM_CLOSE;
		trapped_cmd.stream_handle = (unsigned int)substream;
		if ((dasf_put_command(&trapped_cmd, &trapped_response)) == 0) {
			DPRINTK
			    ("snd_card_omap_alsa_close: Successfully posted the command to mailbox \n");
		} else
			DPRINTK
			    ("snd_card_omap_alsa_close: Failed to post the command to mailbox \n");
	}

	s->stream = NULL;
	s->active = 0;
	s->state = ALSA_STRM_STOPPED;

	codec->codec_shutdown();
	/* It is critical that the DASF/DSP relinquishes all the transit buffer
	 * by now */
	/* now free the allocated buffers */
	asnd_pcm_free_vmalloc_buffer(substream);

	return 0;
}

static int asnd_pcm_alloc_vmalloc_buffer(struct snd_pcm_substream *subs,
					 size_t size)
{
	struct snd_pcm_runtime *runtime = subs->runtime;
	if (runtime->dma_area) {
		/* already allocated */
		if (runtime->dma_bytes >= size)
			return 0;	/* already large enough */
		vfree(runtime->dma_area);
	}
	runtime->dma_area = vmalloc_32(size);
	if (!runtime->dma_area)
		return -ENOMEM;
	memset(runtime->dma_area, 0, size);
	runtime->dma_bytes = size;
	return 1;
}

static int asnd_pcm_free_vmalloc_buffer(struct snd_pcm_substream *subs)
{
	struct snd_pcm_runtime *runtime = subs->runtime;

	vfree(runtime->dma_area);
	runtime->dma_area = NULL;
	return 0;
}

/* get the physical page pointer on the given offset */
static struct page *snd_omap_get_vmalloc_page(struct snd_pcm_substream *subs,
					      unsigned long offset)
{
	void *pageptr = subs->runtime->dma_area + offset;
	DPRINTK("snd_omap_get_vmalloc_page, called with base+offset=0x%x \n",
		pageptr);
	return vmalloc_to_page(pageptr);
}

/* HW params & free */
static int snd_omap_alsa_hw_params(snd_pcm_substream_t * substream,
				   snd_pcm_hw_params_t * hw_params)
{
	FN_IN;

	DPRINTK
	    ("snd_omap_alsa_hw_params: Requesting playback buffer size = 0x%x \n",
	     params_buffer_bytes(hw_params));
	return asnd_pcm_alloc_vmalloc_buffer(substream,
					     params_buffer_bytes(hw_params));
}

static int snd_omap_alsa_hw_free(snd_pcm_substream_t * substream)
{
	FN_IN;

#if 0
	/* Moved to pcm_close  *** */
	return asnd_pcm_free_vmalloc_buffer(substream);
#endif
	return 0;
}

/* pcm operations */
static snd_pcm_ops_t snd_card_omap_alsa_playback_ops = {
	.open = snd_card_omap_alsa_open,
	.close = snd_card_omap_alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_omap_alsa_hw_params,
	.hw_free = snd_omap_alsa_hw_free,
	.prepare = snd_omap_alsa_prepare,
	.trigger = snd_omap_alsa_trigger,
	.pointer = snd_omap_alsa_pointer,
	.page = snd_omap_get_vmalloc_page,
#if 1
	.copy = snd_omap_alsa_copy,
#endif
};

static snd_pcm_ops_t snd_card_omap_alsa_capture_ops = {
	.open = snd_card_omap_alsa_open,
	.close = snd_card_omap_alsa_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_omap_alsa_hw_params,
	.hw_free = snd_omap_alsa_hw_free,
	.prepare = snd_omap_alsa_prepare,
	.trigger = snd_omap_alsa_trigger,
	.pointer = snd_omap_alsa_pointer,
	.page = snd_omap_get_vmalloc_page,
};

/*
 *  Alsa init and exit section
 *  
 *  Inits pcm alsa structures, allocate the alsa buffer, suspend, resume
 */
static int snd_card_omap_alsa_pcm(struct omap_alsa_state *state, int device)
{
	snd_pcm_t *pcm;
	int err;

	FN_IN;
	if ((err =
	     snd_pcm_new(state->card, "OMAP PCM", device, PCM_PLAYBACK_STRMS, 1,
			 &pcm)) < 0)
		return err;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_card_omap_alsa_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_card_omap_alsa_capture_ops);
	pcm->private_data = state;
	pcm->info_flags = 0;
	strcpy(pcm->name, "omap alsa pcm");

	state->pcm = pcm;

	return 0;
}

static void snd_omap_alsa_free(snd_card_t * card)
{
	FN_IN;

	/*
	 * Turn off codec after it is done.
	 * Can't do it immediately, since it may still have
	 * buffered data.
	 */
	schedule_timeout_interruptible(2);
}

static int __dasf_playback_volume_info(snd_kcontrol_t * kcontrol,
				       snd_ctl_elem_info_t * uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;	/*AUDIO_MAX_INPUT_VOLUME */
	return 0;
}

static int __dasf_playback_volume_get(snd_kcontrol_t * kcontrol,
				      snd_ctl_elem_value_t * ucontrol,unsigned int strm_num)
{

	struct omap_alsa_stream *s;

	s = &omap_audio_state->s[strm_num];
	ucontrol->value.integer.value[0] = s->gain;

	return 0;
}

static int __dasf_playback_volume_put(snd_kcontrol_t * kcontrol,
		snd_ctl_elem_value_t * ucontrol,unsigned int strm_num)
{
	struct omap_alsa_stream *s;
	dasf_trapped_cmd_t trapped_cmd;
	struct omap_alsa_state *state = omap_audio_state;
	int requested;
	int gain;
	int status = 0;

	s = &state->s[strm_num];
	/* check if the stream is active */
	if (s->active) {
		gain = ucontrol->value.integer.value[0];
		// if ((ucontrol->value.integer.value[0]) != s->gain)
		if (gain != s->gain) {
			/* Send the command to the DASF to change gain */
			trapped_cmd.cmd = DASF_STREAM_GAIN;
			trapped_cmd.stream_handle = (unsigned int)s->stream;
			trapped_cmd.args.ARGS_STREAM_GAIN.gain = gain * 256;
			/* Queue the command and return */
			printk
			    ("Requesting Gain control for PCM sub-stream ptr=0x%x ,gain=0x%x \n",
			     (unsigned int)s->stream, (unsigned int) ucontrol->value.integer.value[0]);
			requested = state->cmd_queue.requested;
			state->cmd_queue.cmds[requested].cmd = trapped_cmd;
			requested++;
			if (requested == CMD_QUE_DEPTH)
				requested = 0;

			state->cmd_queue.requested = requested;
			/* schedule the work queue here */
			schedule_work(&cmd_work);
			s->gain = ucontrol->value.integer.value[0];
			status = 1;
		}
	} else {
		printk("Invalid stream for volume control !!!! \n");
	}

	return status;
}

static int __dasf_capture_volume_info(snd_kcontrol_t * kcontrol,
				       snd_ctl_elem_info_t * uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;	/*AUDIO_MAX_INPUT_VOLUME */
	return 0;
}

static int __dasf_capture_volume_get(snd_kcontrol_t * kcontrol,
				      snd_ctl_elem_value_t * ucontrol)
{

	struct omap_alsa_stream *s;

	s = &omap_audio_state->s[1<<PCM_PLAYBACK_STRMS_PWR];
	ucontrol->value.integer.value[0] = s->gain;

	return 0;
}

static int __dasf_capture_volume_put(snd_kcontrol_t * kcontrol,
		snd_ctl_elem_value_t * ucontrol)
{
	struct omap_alsa_stream *s;
	dasf_trapped_cmd_t trapped_cmd;
	struct omap_alsa_state *state = omap_audio_state;
	int requested;
	int gain;
	int status = 0;

	s = &state->s[1<<PCM_PLAYBACK_STRMS_PWR];
	/* check if the stream is active */
	if (s->active) {
		gain = ucontrol->value.integer.value[0];
		if (gain != s->gain) {
			/* Send the command to the DASF to change gain */
			trapped_cmd.cmd = DASF_STREAM_GAIN;
			trapped_cmd.stream_handle = (unsigned int)s->stream;
			trapped_cmd.args.ARGS_STREAM_GAIN.gain = gain * 256;
			/* Queue the command and return */
			printk
			    ("Requesting Gain control for PCM sub-stream ptr=0x%x ,gain=0x%x \n",
			     (unsigned int)s->stream, (unsigned int) ucontrol->value.integer.value[0]);
			requested = state->cmd_queue.requested;
			state->cmd_queue.cmds[requested].cmd = trapped_cmd;
			requested++;
			if (requested == CMD_QUE_DEPTH)
				requested = 0;

			state->cmd_queue.requested = requested;
			/* schedule the work queue here */
			schedule_work(&cmd_work);
			s->gain = ucontrol->value.integer.value[0];
			status = 1;
		}
	} else {
		printk("Invalid stream for volume control !!!! \n");
	}

	return status;
}

static int __dasf_playback_mute_info(snd_kcontrol_t * kcontrol,
				     snd_ctl_elem_info_t * uinfo)
{

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int __dasf_playback_mute_get(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol,unsigned int strm_num)
{
	struct omap_alsa_stream *s;

	s = &omap_audio_state->s[strm_num];
	ucontrol->value.integer.value[0] = s->mute;

	return 0;
}

static int __dasf_playback_mute_put(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol, unsigned int strm_num)
{
	struct omap_alsa_stream *s;
	dasf_trapped_cmd_t trapped_cmd;
	struct omap_alsa_state *state = omap_audio_state;
	int requested;
	int status = 0;

	s = &state->s[strm_num];

	if (s->active) {
		if ((ucontrol->value.integer.value[0]) != s->mute) {
			s->mute = ucontrol->value.integer.value[0];
			/* Send the command to the DASF to change gain */
			if (s->mute)
				trapped_cmd.cmd = DASF_STREAM_MUTE;
			else
				trapped_cmd.cmd = DASF_STREAM_UNMUTE;
			trapped_cmd.stream_handle = (unsigned int)s->stream;
			/* Queue the command and return */
			printk ("Requesting Mute control for PCM sub-stream ptr=0x%x ,mute=0x%x \n",
				(unsigned int)s->stream, (unsigned int) ucontrol->value.integer.value[0]);
			requested = state->cmd_queue.requested;
			state->cmd_queue.cmds[requested].cmd = trapped_cmd;
			requested++;
			if (requested == CMD_QUE_DEPTH)
				requested = 0;

			state->cmd_queue.requested = requested;
			/* schedule the work queue here */
			schedule_work(&cmd_work);

			status = 1;
		}
	} else {
		printk("dasf_playback_mute_put: Invalid stream for mute control !!!! \n");
	}

	return status;
}

/*** substream 0 ***/
static int __dasf_playback_volume_get0(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,0);
}

static int __dasf_playback_volume_put0(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,0);
}

static int __dasf_playback_mute_get0(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,0);
}

static int __dasf_playback_mute_put0(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,0);
}

/*** substream 1 ***/
static int __dasf_playback_volume_get1(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,1);
}

static int __dasf_playback_volume_put1(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,1);
}

static int __dasf_playback_mute_get1(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,1);
}

static int __dasf_playback_mute_put1(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,1);
}

/*** substream 2 ***/
static int __dasf_playback_volume_get2(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,2);
}

static int __dasf_playback_volume_put2(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,2);
}
static int __dasf_playback_mute_get2(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,2);
}

static int __dasf_playback_mute_put2(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,2);
}

/*** substream 3 ***/
static int __dasf_playback_volume_get3(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,3);
}

static int __dasf_playback_volume_put3(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,3);
}
static int __dasf_playback_mute_get3(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,3);
}

static int __dasf_playback_mute_put3(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,3);
}

/*** substream 4 ***/
static int __dasf_playback_volume_get4(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,4);
}

static int __dasf_playback_volume_put4(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,4);
}
static int __dasf_playback_mute_get4(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,4);
}

static int __dasf_playback_mute_put4(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,4);
}

/*** substream 5 ***/
static int __dasf_playback_volume_get5(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,5);
}

static int __dasf_playback_volume_put5(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,5);
}
static int __dasf_playback_mute_get5(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,5);
}

static int __dasf_playback_mute_put5(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,5);
}

/*** substream 6 ***/
static int __dasf_playback_volume_get6(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,6);
}

static int __dasf_playback_volume_put6(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,6);
}
static int __dasf_playback_mute_get6(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,6);
}

static int __dasf_playback_mute_put6(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,6);
}

/*** substream 7 ***/
static int __dasf_playback_volume_get7(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,7);
}

static int __dasf_playback_volume_put7(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,7);
}
static int __dasf_playback_mute_get7(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,7);
}

static int __dasf_playback_mute_put7(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,7);
}

/*** substream 8 ***/
static int __dasf_playback_volume_get8(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,8);
}

static int __dasf_playback_volume_put8(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,8);
}
static int __dasf_playback_mute_get8(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,8);
}

static int __dasf_playback_mute_put8(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,8);
}

/*** substream 9 ***/
static int __dasf_playback_volume_get9(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,9);
}

static int __dasf_playback_volume_put9(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,9);
}
static int __dasf_playback_mute_get9(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,9);
}

static int __dasf_playback_mute_put9(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,9);
}

/*** substream 10 ***/
static int __dasf_playback_volume_get10(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,10);
}

static int __dasf_playback_volume_put10(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,10);
}
static int __dasf_playback_mute_get10(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,10);
}

static int __dasf_playback_mute_put10(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,10);
}

/*** substream 11 ***/
static int __dasf_playback_volume_get11(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,11);
}

static int __dasf_playback_volume_put11(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,11);
}
static int __dasf_playback_mute_get11(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,11);
}

static int __dasf_playback_mute_put11(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,11);
}

/*** substream 12 ***/
static int __dasf_playback_volume_get12(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,12);
}

static int __dasf_playback_volume_put12(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,12);
}
static int __dasf_playback_mute_get12(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,12);
}

static int __dasf_playback_mute_put12(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,12);
}

/*** substream 13 ***/
static int __dasf_playback_volume_get13(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,13);
}

static int __dasf_playback_volume_put13(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,13);
}
static int __dasf_playback_mute_get13(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,13);
}

static int __dasf_playback_mute_put13(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,13);
}

/*** substream 14 ***/
static int __dasf_playback_volume_get14(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_get(kcontrol, ucontrol,14);
}

static int __dasf_playback_volume_put14(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_volume_put(kcontrol, ucontrol,14);
}
static int __dasf_playback_mute_get14(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_get(kcontrol, ucontrol,14);
}

static int __dasf_playback_mute_put14(snd_kcontrol_t * kcontrol,
				    snd_ctl_elem_value_t * ucontrol)
{
	return __dasf_playback_mute_put(kcontrol, ucontrol,14);
}



static snd_kcontrol_new_t dasf_control[] __devinitdata = {
	
	{
	 .name = "PCM Playback Volume - substream 0",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get0,.put = __dasf_playback_volume_put0,
	 },
	{
	 .name = "PCM Playback Volume - substream 1",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get1,.put = __dasf_playback_volume_put1,
	 },
	{
	 .name = "PCM Playback Volume - substream 2",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get2,.put = __dasf_playback_volume_put2,
	 },
	{
	 .name = "PCM Playback Volume - substream 3",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get3,.put = __dasf_playback_volume_put3,
	 },
	{
	 .name = "PCM Playback Volume - substream 4",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get4,.put = __dasf_playback_volume_put4,
	 },
	{
	 .name = "PCM Playback Volume - substream 5",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get5,.put = __dasf_playback_volume_put5,
	 },
	{
	 .name = "PCM Playback Volume - substream 6",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get6,.put = __dasf_playback_volume_put6,
	 },
	{
	 .name = "PCM Playback Volume - substream 7",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get7,.put = __dasf_playback_volume_put7,
	 },
	{
	 .name = "PCM Playback Volume - substream 8",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get8,.put = __dasf_playback_volume_put8,
	 },
	{
	 .name = "PCM Playback Volume - substream 9",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get9,.put = __dasf_playback_volume_put9,
	 },
	{
	 .name = "PCM Playback Volume - substream 10",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get10,.put = __dasf_playback_volume_put10,
	 },
	{
	 .name = "PCM Playback Volume - substream 11",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get11,.put = __dasf_playback_volume_put11,
	 },
	{
	 .name = "PCM Playback Volume - substream 12",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get12,.put = __dasf_playback_volume_put12,
	 },
	{
	 .name = "PCM Playback Volume - substream 13",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get13,.put = __dasf_playback_volume_put13,
	 },
	{
	 .name = "PCM Playback Volume - substream 14",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_volume_info,.get = __dasf_playback_volume_get14,.put = __dasf_playback_volume_put14,
	 },

	{
	 .name = "PCM Playback Mute/UnMute - substream 0", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info,.get = __dasf_playback_mute_get0,.put = __dasf_playback_mute_put0,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 1", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get1, .put = __dasf_playback_mute_put1,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 2", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get2, .put = __dasf_playback_mute_put2,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 3", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get3, .put = __dasf_playback_mute_put3,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 4", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get4, .put = __dasf_playback_mute_put4,
	},
	{	 
	.name = "PCM Playback Mute/UnMute - substream 5", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info,.get = __dasf_playback_mute_get5,.put = __dasf_playback_mute_put5,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 6", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get6, .put = __dasf_playback_mute_put6,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 7", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get7, .put = __dasf_playback_mute_put7,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 8", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get8, .put = __dasf_playback_mute_put8,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 9", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get9, .put = __dasf_playback_mute_put9,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 10", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get10, .put = __dasf_playback_mute_put10,
	},
	{	 
	.name = "PCM Playback Mute/UnMute - substream 11", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info,.get = __dasf_playback_mute_get11,.put = __dasf_playback_mute_put11,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 12", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get12, .put = __dasf_playback_mute_put12,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 13", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get13, .put = __dasf_playback_mute_put13,
	},
	{
	 .name = "PCM Playback Mute/UnMute - substream 14", .iface = SNDRV_CTL_ELEM_IFACE_PCM, .index = 0,
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
	 .info = __dasf_playback_mute_info, .get = __dasf_playback_mute_get14, .put = __dasf_playback_mute_put14,
	},
	{
		.name = "PCM substream Capture Volume",.iface = SNDRV_CTL_ELEM_IFACE_PCM,.index = 0,
		.access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
		.info = __dasf_capture_volume_info,.get = __dasf_capture_volume_get,.put = __dasf_capture_volume_put,
	}
};

static int dasf_pcm_strm_init(struct snd_card *card)
{
	int i = 0;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(dasf_control); i++) {
		if ((err = snd_ctl_add(card,
				       snd_ctl_new1(&dasf_control[i],
						    card))) < 0) {
			return err;
		}
	}
	return 0;
}

/** 
 * @brief audio_probe - The Audio driver probe function
 * WARNING!!!!  : It is expected that the codec would have registered with us 
 *                by now
 * 
 * @param dev 
 * 
 * @return 
 */
static int audio_probe(struct omap_dev *dev)
{
	int err = 0;
	snd_card_t *card;
	struct omap_alsa_state *state = dev->dev.platform_data;
	struct omap_alsa_codec *codec = state->codec;

	FN_IN;

	if (codec->codec_probe())
		return -ENODEV;

	/* register the soundcard */
	card = snd_card_new(0, NULL, THIS_MODULE, sizeof(state));
	if (card == NULL)
		goto nodev1;

	card->private_data = (void *)state;
	card->private_free = snd_omap_alsa_free;

	state->card = card;
	state->samplerate = codec->codec_default_samplerate();

	/* PCM */
	if ((err = snd_card_omap_alsa_pcm(state, 0)) < 0)
		goto nodev3;

	strcpy(card->driver, "OMAP_ALSA");
	strcpy(card->shortname, codec->name);
	sprintf(card->longname, codec->name);

	snd_card_set_dev(card, (struct device *)dev);

	if ((err = snd_card_register(card)) == 0) {
		printk(KERN_INFO "audio support initialized\n");
		omap_set_drvdata(dev, card);
		codec->mixer_init(card);

		/* add dasf controls here */
		dasf_pcm_strm_init(card);

		return 0;
	}

      nodev3:
	snd_card_free(card);
      nodev1:
	FN_OUT(err);
	return err;
}

/** 
 * @brief audio_remove - Function to handle removal operations
 * 
 * @param dev 
 * 
 * @return 
 */
static int audio_remove(struct omap_dev *dev)
{
	snd_card_t *card = omap_get_drvdata(dev);

	FN_IN;
	card->private_data = NULL;
	snd_card_free(card);
	omap_set_drvdata(dev, NULL);

	FN_OUT(0);
	return 0;
}

static void audio_release(struct device *dev)
{
	/* Nothing to Release! */
}

/** 
 * @brief audio_register_codec - Register a Codec fn points using this function
 * WARNING!!!!!          : Codecs should ensure that they do so! no sanity 
 *                         checks during runtime is done due to obvious 
 *                         performance penalties.
 */
int audio_register_codec(struct omap_alsa_codec *codec)
{
	int ret;

	FN_IN;

	/* We really want one driver to support multiple codecs. But omap bus
	 * match code won't allow us to do it */
	if (omap_audio_state) {
		printk(KERN_ERR "OMAP Audio only supports one codec now\n");
		return -EINVAL;
	}

	if (!codec)
		return -EINVAL;

	/* TODO: more sanity check on the codec passed to us */

	omap_audio_state = kmalloc(sizeof(*omap_audio_state), GFP_KERNEL);
	if (!omap_audio_state)
		return -ENOMEM;
	memset(omap_audio_state, 0, sizeof(*omap_audio_state));

	/* setup pointers */
	omap_audio_state->codec = codec;
	omap_audio_device.dev.platform_data = omap_audio_state;

	ret = omap_device_register(&omap_audio_device);
	if (ret) {
		printk(KERN_ERR "OMAP Audio Device Register failed =%d\n", ret);
		kfree(omap_audio_state);
		omap_audio_state = NULL;
		return ret;
	}
	/* Register ALSA-DASF driver */
	register_alsa_drv();

	/* Initialize the DASF driver data structures */
	dasf_init_mailbox(&dasf_mailbox);
	omap_audio_state->cmd_queue.requested = 0;
	omap_audio_state->cmd_queue.serviced = 0;

	INIT_WORK(&cmd_work, run_work_queue, omap_audio_state);

#ifdef CONFIG_PM
	omap_audio_state->suspended = 0;
	init_waitqueue_head(&omap_audio_state->suspend_wq);
#endif

	return 0;
}

static void run_work_queue(struct omap_alsa_state *state)
{
	dasf_trapped_cmd_t trapped_cmd;
	dasf_trapped_response_t trapped_response;
	int serviced;
	int requested;

	serviced = state->cmd_queue.serviced;
	requested = state->cmd_queue.requested;

	FN_IN;
	DPRINTK("Entered 'run_work_queue: serviced=%d , requested=%d \n",
		serviced, requested);
	while (serviced != requested) {
		trapped_cmd = state->cmd_queue.cmds[serviced].cmd;
		serviced++;
		if (serviced == CMD_QUE_DEPTH)
			serviced = 0;
		if ((dasf_put_command(&trapped_cmd, &trapped_response)) == 0) {
			DPRINTK
			    ("run_work_queue: Successfully posted the command to mailbox \n");
		} else
			DPRINTK
			    ("run_work_queue: Failed to post the command to mailbox \n");
	}
	state->cmd_queue.serviced = serviced;
	requested = state->cmd_queue.requested;
	if (serviced != requested)
		schedule_work(&cmd_work);
	FN_OUT(serviced);
}

/** 
 * @brief audio_unregister_codec - Un-Register a Codec using this function
 * 
 * @param codec_state 
 * 
 * @return 
 */
int audio_unregister_codec(struct omap_alsa_codec *codec)
{

	if (!omap_audio_state || omap_audio_state->codec != codec) {
		printk(KERN_ERR "Bad codec unregister call\n");
		return -EINVAL;
	}

	omap_device_unregister(&omap_audio_device);
	kfree(omap_audio_state);
	omap_audio_state = NULL;
	unregister_alsa_drv();

	return 0;
}

static int __init omap_audio_init(void)
{
	int ret;

	ret = omap_driver_register(&omap_audio_driver);
	if (ret) {
		printk(KERN_ERR "OMAP Audio Driver register failed =%d\n", ret);
		return ret;
	}

	return 0;
}

static void __exit omap_audio_exit(void)
{
	/* clean up any codec device that isn't removed by its driver */
	if (omap_audio_state) {
		omap_device_unregister(&omap_audio_device);
		kfree(omap_audio_state);
	}
	omap_driver_unregister(&omap_audio_driver);
}

module_init(omap_audio_init);
module_exit(omap_audio_exit);

EXPORT_SYMBOL(audio_register_codec);
EXPORT_SYMBOL(audio_unregister_codec);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ALSA audio handling for OMAP processors");
MODULE_LICENSE("GPL");
