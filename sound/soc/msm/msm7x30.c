/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/tlv.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/msm_audio.h>

#include "msm7kv2-pcm.h"
#include <asm/mach-types.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/debug_mm.h>

#ifdef CONFIG_MFD_WM8994
#include "../codecs/wm8994.h"
#include <sound/jack.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/input.h>
#include <sound/pcm_params.h>

#define WM_FS 48000
#define WM_CHANNELS 2
#define WM_BITS 16
#define WM_FLL_MULT 8 /* 2*16*8 = 256, clock rates must be >= 256*fs */
#define WM_BCLK (WM_FS * WM_CHANNELS * WM_BITS) /* 1.536MHZ */
#define WM_FLL (WM_FLL_MULT * WM_BCLK) /* 12.288 MHZ */
#define WM_FLL_MIN_RATE 4096000 /* The minimum clk rate required for AIF's */

#endif /* CONFIG_MFD_WM8994 */

static struct platform_device *msm_audio_snd_device;
struct audio_locks the_locks;
EXPORT_SYMBOL(the_locks);
struct msm_volume msm_vol_ctl;
EXPORT_SYMBOL(msm_vol_ctl);
static struct snd_kcontrol_new snd_msm_controls[];

char snddev_name[AUDIO_DEV_CTL_MAX_DEV][44];
#define MSM_MAX_VOLUME 0x2000
#define MSM_VOLUME_STEP ((MSM_MAX_VOLUME+17)/100) /* 17 added to avoid
						      more deviation */
static int device_index; /* Count of Device controls */
static int simple_control; /* Count of simple controls*/

static int msm_scontrol_count_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	return 0;
}

static int msm_scontrol_count_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = simple_control;
	return 0;
}

static int msm_v_call_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}

static int msm_v_call_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int msm_v_call_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int start = ucontrol->value.integer.value[0];
	if (start)
		broadcast_event(AUDDEV_EVT_START_VOICE, DEVICE_IGNORE,
							SESSION_IGNORE);
	else
		broadcast_event(AUDDEV_EVT_END_VOICE, DEVICE_IGNORE,
							SESSION_IGNORE);
	return 0;
}

static int msm_v_mute_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 2;
	return 0;
}

static int msm_v_mute_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int msm_v_mute_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int dir = ucontrol->value.integer.value[0];
	int mute = ucontrol->value.integer.value[1];
	return msm_set_voice_mute(dir, mute);
}

static int msm_v_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2; /* Volume */
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;
	return 0;
}

static int msm_v_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int msm_v_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int dir = ucontrol->value.integer.value[0];
	int volume = ucontrol->value.integer.value[1];

	return msm_set_voice_vol(dir, volume);
}

static int msm_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3; /* Volume and 10-base multiply factor*/
	uinfo->value.integer.min = 0;

	/* limit the muliply factor to 4 decimal digit */
	uinfo->value.integer.max = 1000000;
	return 0;
}
static int msm_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

static int msm_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int ret = 0;
	int session_id = ucontrol->value.integer.value[0];
	int volume = ucontrol->value.integer.value[1];
	int factor = ucontrol->value.integer.value[2];
	u32 session_mask = 0;


	if (factor > 10000)
		return -EINVAL;

	if ((volume < 0) || (volume/factor > 100))
		return -EINVAL;

	volume = (MSM_VOLUME_STEP * volume);

	/* Convert back to original decimal point by removing the 10-base factor
	* and discard the fractional portion
	*/

	volume = volume/factor;

	if (volume > MSM_MAX_VOLUME)
		volume = MSM_MAX_VOLUME;

	/* Only Decoder volume control supported */
	session_mask = (0x1 << (session_id) << (8 * ((int)AUDDEV_CLNT_DEC-1)));
	msm_vol_ctl.volume = volume;
	MM_DBG("session_id %d, volume %d", session_id, volume);
	broadcast_event(AUDDEV_EVT_STREAM_VOL_CHG, DEVICE_IGNORE,
							session_mask);

	return ret;
}

static int msm_voice_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3; /* Device */

	uinfo->value.integer.min = 1;
	uinfo->value.integer.max = msm_snddev_devcount();
	return 0;
}

static int msm_voice_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	uint32_t rx_dev_id;
	uint32_t tx_dev_id;
	struct msm_snddev_info *rx_dev_info;
	struct msm_snddev_info *tx_dev_info;
	int set = ucontrol->value.integer.value[2];
	u32 session_mask;

	if (!set)
		return -EPERM;
	/* Rx Device Routing */
	rx_dev_id = ucontrol->value.integer.value[0];
	rx_dev_info = audio_dev_ctrl_find_dev(rx_dev_id);

	if (IS_ERR(rx_dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(rx_dev_info);
		return rc;
	}

	if (!(rx_dev_info->capability & SNDDEV_CAP_RX)) {
		MM_ERR("First Dev is supposed to be RX\n");
		return -EFAULT;
	}

	MM_DBG("route cfg %d STREAM_VOICE_RX type\n",
		rx_dev_id);

	msm_set_voc_route(rx_dev_info, AUDIO_ROUTE_STREAM_VOICE_RX,
				rx_dev_id);

	session_mask =	0x1 << (8 * ((int)AUDDEV_CLNT_VOC-1));

	broadcast_event(AUDDEV_EVT_DEV_CHG_VOICE, rx_dev_id, session_mask);


	/* Tx Device Routing */
	tx_dev_id = ucontrol->value.integer.value[1];
	tx_dev_info = audio_dev_ctrl_find_dev(tx_dev_id);

	if (IS_ERR(tx_dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(tx_dev_info);
		return rc;
	}

	if (!(tx_dev_info->capability & SNDDEV_CAP_TX)) {
		MM_ERR("Second Dev is supposed to be Tx\n");
		return -EFAULT;
	}

	MM_DBG("route cfg %d %d type\n",
		tx_dev_id, AUDIO_ROUTE_STREAM_VOICE_TX);

	msm_set_voc_route(tx_dev_info, AUDIO_ROUTE_STREAM_VOICE_TX,
				tx_dev_id);

	broadcast_event(AUDDEV_EVT_DEV_CHG_VOICE, tx_dev_id, session_mask);

	if (rx_dev_info->opened)
		broadcast_event(AUDDEV_EVT_DEV_RDY, rx_dev_id,	session_mask);

	if (tx_dev_info->opened)
		broadcast_event(AUDDEV_EVT_DEV_RDY, tx_dev_id, session_mask);

	return rc;
}

static int msm_voice_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	/* TODO: query Device list */
	return 0;
}

static int msm_device_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1; /* Device */

	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = msm_snddev_devcount();
	return 0;
}

#if defined(CONFIG_MFD_WM8994)
static int configure_wm_hw(struct msm_snddev_info *dev_info)
{
	int rc = 0;

	/* if device is internal pcm, then configure wolfson codec */
	if (dev_info->copp_id == 0) {
		struct snd_pcm_substream substream;
		struct snd_pcm_hw_params params;
		struct snd_soc_dai *codec_dai;
		struct wm8994_priv *wm8994;
		int fll = 0, fll_sysclk = 0, fll_rate = 0;
		int aifclk = 0;
		int bclk_rate = 0;

		if (dev_info->capability & SNDDEV_CAP_RX) {
			codec_dai = &wm8994_dai[0];
			substream.stream = SNDRV_PCM_STREAM_PLAYBACK;
			fll = WM8994_FLL1;
			fll_sysclk = WM8994_SYSCLK_FLL1;
			aifclk = WM8994_AIF1_CLOCKING_1;
		} else {
			codec_dai = &wm8994_dai[1];
			substream.stream = SNDRV_PCM_STREAM_CAPTURE;
			fll = WM8994_FLL2;
			fll_sysclk = WM8994_SYSCLK_FLL2;
			aifclk = WM8994_AIF2_CLOCKING_1;
		}

		wm8994 = snd_soc_codec_get_drvdata(codec_dai->codec);

		params.intervals[SNDRV_PCM_HW_PARAM_CHANNELS
			- SNDRV_PCM_HW_PARAM_FIRST_INTERVAL].min = dev_info->channel_mode;
		params.intervals[SNDRV_PCM_HW_PARAM_CHANNELS
			- SNDRV_PCM_HW_PARAM_FIRST_INTERVAL].max = dev_info->channel_mode;
		params.intervals[SNDRV_PCM_HW_PARAM_RATE
			- SNDRV_PCM_HW_PARAM_FIRST_INTERVAL].min = dev_info->sample_rate;
		params.intervals[SNDRV_PCM_HW_PARAM_RATE
			- SNDRV_PCM_HW_PARAM_FIRST_INTERVAL].max = dev_info->sample_rate;
		snd_mask_set(&params.masks[SNDRV_PCM_HW_PARAM_FORMAT
			- SNDRV_PCM_HW_PARAM_FIRST_MASK], SNDRV_PCM_FORMAT_S16_LE);

		/* Set fll rate by multiplying 2 channels even if its mono
		 * aifclk rates need to be atleast 256*fs, 1 channel would make it 128*fs
		 * also, must be >= 4.096Mhz and <= 12.5Mhz, refer to datasheet.
		 */
		bclk_rate = dev_info->sample_rate * WM_CHANNELS * WM_BITS;
		fll_rate = bclk_rate * WM_FLL_MULT;
		if (fll_rate < WM_FLL_MIN_RATE)
			fll_rate = WM_FLL_MIN_RATE;

		if (dev_info->capability & SNDDEV_CAP_RX) {
			if (wm8994->aifclk[0] == fll_rate) {
				pr_info("aif1 codec rates are already configured\n");
				return rc;
			}
		} else {
			if (wm8994->aifclk[1] == fll_rate) {
				pr_info("aif2 codec rates are already configured\n");
				return rc;
			}
		}

		/* aif clocks are disabled when reconfiguring fll and bclk rates */
		rc = snd_soc_dai_set_pll(codec_dai, fll,
			WM8994_FLL_SRC_BCLK, bclk_rate, fll_rate);
		if (rc < 0) {
			pr_err("Failed to set DAI FLL to rate %d: ret %d\n",
				WM_FLL_MULT * bclk_rate, rc);
			return rc;
		}

		rc = snd_soc_dai_set_sysclk(codec_dai, fll_sysclk,
						fll_rate, 0);
		if (rc < 0) {
			pr_err("Failed to set sysclk: ret %d\n", rc);
			return rc;
		}

		wm8994_hw_params(&substream, &params, codec_dai);
	}

	return rc;
}
#endif

static int msm_device_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	int set = 0;
	struct msm_audio_route_config route_cfg;
	struct msm_snddev_info *dev_info;
	u32 set_freq = 0;

	set = ucontrol->value.integer.value[0];
	route_cfg.dev_id = ucontrol->id.numid - device_index;
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);
	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}
	MM_INFO("device %s set %d\n", dev_info->name, set);

	if (set) {
		if (!dev_info->opened) {
			set_freq = dev_info->sample_rate;

			MM_ERR("device freq =%d\n", set_freq);
			rc = dev_info->dev_ops.set_freq(dev_info, set_freq);
			if (rc < 0) {
				MM_ERR("device freq failed!\n");
				return rc;
			}
			dev_info->set_sample_rate = rc;
			rc = 0;
			rc = dev_info->dev_ops.open(dev_info);
			if (rc < 0) {
				MM_ERR("Enabling %s failed", dev_info->name);
				return rc;
			}
			dev_info->opened = 1;
			broadcast_event(AUDDEV_EVT_DEV_RDY, route_cfg.dev_id,
							SESSION_IGNORE);
			/* Event to notify client for device info */
			broadcast_event(AUDDEV_EVT_DEVICE_INFO,
					route_cfg.dev_id, SESSION_IGNORE);
#if defined(CONFIG_MFD_WM8994)
			if (configure_wm_hw(dev_info))
				pr_err("%s: Could not configure wolfson hw properly!", __func__);
#endif
		}
	} else {
		if (dev_info->opened) {
			broadcast_event(AUDDEV_EVT_REL_PENDING,
						route_cfg.dev_id,
						SESSION_IGNORE);
			rc = dev_info->dev_ops.close(dev_info);
			if (rc < 0) {
				MM_ERR("Snd device failed close!\n");
				return rc;
			} else {
				dev_info->opened = 0;
				broadcast_event(AUDDEV_EVT_DEV_RLS,
					route_cfg.dev_id,
					SESSION_IGNORE);
			}
		}

	}
	return rc;
}

static int msm_device_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	struct msm_audio_route_config route_cfg;
	struct msm_snddev_info *dev_info;

	route_cfg.dev_id = ucontrol->id.numid - device_index;
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);

	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}

	ucontrol->value.integer.value[0] = dev_info->copp_id;
	ucontrol->value.integer.value[1] = dev_info->capability;

	return 0;
}

static int msm_route_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3; /* Device */

	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = msm_snddev_devcount();
	return 0;
}

static int msm_route_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	/* TODO: query Device list */
	return 0;
}

static int msm_route_put(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	int enc_freq = 0;
	int requested_freq = 0;
	struct msm_audio_route_config route_cfg;
	struct msm_snddev_info *dev_info;
	int session_id = ucontrol->value.integer.value[0];
	int set = ucontrol->value.integer.value[2];
	u32 session_mask = 0;
	route_cfg.dev_id = ucontrol->value.integer.value[1];

	if (ucontrol->id.numid == 2)
		route_cfg.stream_type =	AUDIO_ROUTE_STREAM_PLAYBACK;
	else
		route_cfg.stream_type =	AUDIO_ROUTE_STREAM_REC;

	MM_DBG("route cfg %d %d type for popp %d set value %d\n",
		route_cfg.dev_id, route_cfg.stream_type, session_id, set);
	dev_info = audio_dev_ctrl_find_dev(route_cfg.dev_id);

	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		rc = PTR_ERR(dev_info);
		return rc;
	}
	if (route_cfg.stream_type == AUDIO_ROUTE_STREAM_PLAYBACK) {
		rc = msm_snddev_set_dec(session_id, dev_info->copp_id, set);
		session_mask =
			(0x1 << (session_id) << (8 * ((int)AUDDEV_CLNT_DEC-1)));
		if (!set) {
			if (dev_info->opened) {
				broadcast_event(AUDDEV_EVT_REL_PENDING,
						route_cfg.dev_id,
						session_mask);

				broadcast_event(AUDDEV_EVT_DEV_RLS,
							route_cfg.dev_id,
							session_mask);
			}
			dev_info->sessions &= ~(session_mask);
		} else {
			dev_info->sessions = dev_info->sessions | session_mask;
			if (dev_info->opened) {
				broadcast_event(AUDDEV_EVT_DEV_RDY,
							route_cfg.dev_id,
							session_mask);
				/* Event to notify client for device info */
				broadcast_event(AUDDEV_EVT_DEVICE_INFO,
							route_cfg.dev_id,
							session_mask);
			}
		}
	} else {
		rc = msm_snddev_set_enc(session_id, dev_info->copp_id, set);
		session_mask =
			(0x1 << (session_id)) << (8 * ((int)AUDDEV_CLNT_ENC-1));
		if (!set) {
			if (dev_info->opened)
				broadcast_event(AUDDEV_EVT_DEV_RLS,
							route_cfg.dev_id,
							session_mask);
			dev_info->sessions &= ~(session_mask);
		} else {
			dev_info->sessions = dev_info->sessions | session_mask;
			enc_freq = msm_snddev_get_enc_freq(session_id);
			requested_freq = enc_freq;
			if (enc_freq > 0) {
				rc = msm_snddev_request_freq(&enc_freq,
						session_id,
						SNDDEV_CAP_TX,
						AUDDEV_CLNT_ENC);
				MM_DBG("sample rate configured %d"
					"sample rate requested %d\n",
					enc_freq, requested_freq);
				if ((rc <= 0) || (enc_freq != requested_freq)) {
					MM_DBG("msm_snddev_withdraw_freq\n");
					rc = msm_snddev_withdraw_freq
						(session_id,
						SNDDEV_CAP_TX, AUDDEV_CLNT_ENC);
					broadcast_event(AUDDEV_EVT_FREQ_CHG,
							route_cfg.dev_id,
							SESSION_IGNORE);
				}
			}
			if (dev_info->opened) {
				broadcast_event(AUDDEV_EVT_DEV_RDY,
							route_cfg.dev_id,
							session_mask);
				/* Event to notify client for device info */
				broadcast_event(AUDDEV_EVT_DEVICE_INFO,
							route_cfg.dev_id,
							session_mask);
			}
		}
	}

	if (rc < 0) {
		MM_ERR("device could not be assigned!\n");
		return -EFAULT;
	}

	return rc;
}

static int msm_device_volume_info(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 100;
	return 0;
}

static int msm_device_volume_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct msm_snddev_info *dev_info;

	int dev_id; 
   
	if(!ucontrol){
		printk(KERN_ERR"%s: missing ucontrol\n",__func__); 
		return -1;
	}

	dev_id = ucontrol->value.integer.value[0];

	dev_info = audio_dev_ctrl_find_dev(dev_id);

	if(!dev_info) {
		printk(KERN_ERR"%s: cannot find device\n",__func__); 
		return -1;
	}

	ucontrol->value.integer.value[0] = dev_info->dev_volume;

	return 0;
}

static int msm_device_volume_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int rc = -EPERM;
	struct msm_snddev_info *dev_info;

	int dev_id = ucontrol->value.integer.value[0];
	int volume = ucontrol->value.integer.value[1];

	MM_DBG("dev_id = %d, volume = %d\n", dev_id, volume);

	dev_info = audio_dev_ctrl_find_dev(dev_id);

	if (IS_ERR(dev_info)) {
		rc = PTR_ERR(dev_info);
		MM_ERR("audio_dev_ctrl_find_dev failed. %ld\n",
				PTR_ERR(dev_info));
		return rc;
	}

	MM_DBG("dev_name = %s dev_id = %d, volume = %d\n",
				dev_info->name, dev_id, volume);

	if (dev_info->dev_ops.set_device_volume)
		rc = dev_info->dev_ops.set_device_volume(dev_info, volume);
	else {
		MM_INFO("device %s does not support device volume "
				"control.", dev_info->name);
		return -EPERM;
	}

	return rc;
}


static struct snd_kcontrol_new snd_dev_controls[AUDIO_DEV_CTL_MAX_DEV];

static int snd_dev_ctl_index(int idx)
{
	struct msm_snddev_info *dev_info;

	dev_info = audio_dev_ctrl_find_dev(idx + 0);
	if (IS_ERR(dev_info)) {
		MM_ERR("pass invalid dev_id\n");
		return PTR_ERR(dev_info);
	}
	if (sizeof(dev_info->name) <= 44)
		sprintf(&snddev_name[idx][0] , "%s", dev_info->name);

	snd_dev_controls[idx].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	snd_dev_controls[idx].access = SNDRV_CTL_ELEM_ACCESS_READWRITE;
	snd_dev_controls[idx].name = &snddev_name[idx][0];
	snd_dev_controls[idx].index = 5 + idx;
	snd_dev_controls[idx].info = msm_device_info;
	snd_dev_controls[idx].get = msm_device_get;
	snd_dev_controls[idx].put = msm_device_put;
	snd_dev_controls[idx].private_value = 0;
	return 0;

}

#define MSM_EXT(xname, xindex, fp_info, fp_get, fp_put, addr) \
{ .iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
  .access = SNDRV_CTL_ELEM_ACCESS_READWRITE, \
  .name = xname, .index = xindex, \
  .info = fp_info,\
  .get = fp_get, .put = fp_put, \
  .private_value = addr, \
}

static struct snd_kcontrol_new snd_msm_controls[] = {
	MSM_EXT("Count", 1, msm_scontrol_count_info, msm_scontrol_count_get, \
						NULL, 0),
	MSM_EXT("Stream", 2, msm_route_info, msm_route_get, \
						 msm_route_put, 0),
	MSM_EXT("Record", 3, msm_route_info, msm_route_get, \
						 msm_route_put, 0),
	MSM_EXT("Voice", 4, msm_voice_info, msm_voice_get, \
						 msm_voice_put, 0),
	MSM_EXT("Volume", 5, msm_volume_info, msm_volume_get, \
						 msm_volume_put, 0),
	MSM_EXT("VoiceVolume", 6, msm_v_volume_info, msm_v_volume_get, \
						 msm_v_volume_put, 0),
	MSM_EXT("VoiceMute", 7, msm_v_mute_info, msm_v_mute_get, \
						 msm_v_mute_put, 0),
	MSM_EXT("Voice Call", 8, msm_v_call_info, msm_v_call_get, \
						msm_v_call_put, 0),
	MSM_EXT("Device_Volume", 9, msm_device_volume_info,
			msm_device_volume_get, msm_device_volume_put, 0),
};

static int msm_new_mixer(struct snd_card *card)
{
	unsigned int idx;
	int err;
	int dev_cnt;

	strcpy(card->mixername, "MSM Mixer");
	for (idx = 0; idx < ARRAY_SIZE(snd_msm_controls); idx++) {
		err = snd_ctl_add(card,	snd_ctl_new1(&snd_msm_controls[idx],
					NULL));
		if (err < 0)
			MM_ERR("ERR adding ctl\n");
	}
	dev_cnt = msm_snddev_devcount();

	for (idx = 0; idx < dev_cnt; idx++) {
		if (!snd_dev_ctl_index(idx)) {
			err = snd_ctl_add(card, snd_ctl_new1(
				&snd_dev_controls[idx], NULL));
			if (err < 0)
				MM_ERR("ERR adding ctl\n");
		} else
			return 0;
	}
	simple_control = ARRAY_SIZE(snd_msm_controls);
	device_index = simple_control + 1;
	return 0;
}

#ifdef CONFIG_MFD_WM8994

static const struct snd_soc_dapm_widget rib_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Internal Mic", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static struct snd_soc_dapm_route rib_dapm_routes[] = {
	{ "Headphone", NULL, "HPOUT1L" },
	{ "Headphone", NULL, "HPOUT1R" },

	{ "Speaker", NULL, "SPKOUTLP" },
	{ "Speaker", NULL, "SPKOUTLN" },

	{ "DMIC1DAT", NULL, "MICBIAS1" },
	{ "MICBIAS1", NULL, "Internal Mic" },

	{ "IN1LN", NULL, "Headset Mic" },
	{ "IN1LP", NULL, "Headset Mic" },
};

static int msm_soc_dai_init(struct snd_soc_codec *codec)
{
	int ret = 0;

	ret = msm_new_mixer(codec->card);
	if (ret < 0)
		pr_err("%s: ALSA MSM Mixer Fail\n", __func__);

	wm8994_add_controls(codec);

	snd_soc_dapm_enable_pin(codec, "DMIC1DAT");
	snd_soc_dapm_enable_pin(codec, "IN1LN");
	snd_soc_dapm_enable_pin(codec, "IN1LP");

	snd_soc_dapm_nc_pin(codec, "DMIC2DAT");
	snd_soc_dapm_nc_pin(codec, "IN2LN");
	snd_soc_dapm_nc_pin(codec, "IN1RN");
	snd_soc_dapm_nc_pin(codec, "IN1RP");
	snd_soc_dapm_nc_pin(codec, "IN2RN");
	snd_soc_dapm_nc_pin(codec, "IN2LP:VXRN");
	snd_soc_dapm_nc_pin(codec, "IN2RP:VXRP");
	snd_soc_dapm_nc_pin(codec, "LINEOUT1N");
	snd_soc_dapm_nc_pin(codec, "LINEOUT1P");
	snd_soc_dapm_nc_pin(codec, "LINEOUT2N");
	snd_soc_dapm_nc_pin(codec, "LINEOUT2P");

	snd_soc_dapm_new_controls(codec, rib_dapm_widgets,
		ARRAY_SIZE(rib_dapm_widgets));

	snd_soc_dapm_add_routes(codec, rib_dapm_routes,
		ARRAY_SIZE(rib_dapm_routes));

	return ret;
}

static int rib_hw_params(struct snd_pcm_substream *substream,
						struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_CBS_CFS |
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
	if (ret != 0) {
		pr_err("Failed to set DAI format: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops rib_ops = {
	.hw_params = rib_hw_params,
};

static struct snd_soc_dai_link msm_dai[] = {
	{
		.name = "Playback",
		.stream_name = "AIF1",
		.codec_dai = &wm8994_dai[0],
		.cpu_dai = &msm_dais[1],
		.init   = msm_soc_dai_init,
		.ops = &rib_ops,
	},
	{
		.name = "Record",
		.stream_name = "AIF2",
		.codec_dai = &wm8994_dai[1],
		.cpu_dai = &msm_dais[1],
		.ops = &rib_ops,
	},
};


struct snd_soc_card snd_soc_card_msm = {
	.name           = "msm-audio",
	.dai_link       = &msm_dai[0],
	.num_links = ARRAY_SIZE(msm_dai),
	.platform = &msm_soc_platform,
};

/* msm_audio audio subsystem */
static struct snd_soc_device msm_audio_snd_devdata = {
	.card = &snd_soc_card_msm,
	.codec_dev = &soc_codec_dev_wm8994,
};

#else

static int msm_soc_dai_init(struct snd_soc_codec *codec)
{

	int ret = 0;
	ret = msm_new_mixer(codec->card);
	if (ret < 0)
		MM_ERR("msm_soc: ALSA MSM Mixer Fail\n");

	return ret;
}

static struct snd_soc_dai_link msm_dai = {
	.name = "ASOC",
	.stream_name = "ASOC",
	.codec_dai = &msm_dais[0],
	.cpu_dai = &msm_dais[1],
	.init	= msm_soc_dai_init,
};

struct snd_soc_card snd_soc_card_msm = {
	.name		= "msm-audio",
	.dai_link	= &msm_dai,
	.num_links = 1,
	.platform = &msm_soc_platform,
};

/* msm_audio audio subsystem */
static struct snd_soc_device msm_audio_snd_devdata = {
	.card = &snd_soc_card_msm,
	.codec_dev = &soc_codec_dev_msm,
};

#endif /* CONFIG_MFD_8994 */

static int __init msm_audio_init(void)
{
	int ret;

	msm_audio_snd_device = platform_device_alloc("soc-audio", 0);
	if (!msm_audio_snd_device)
		return -ENOMEM;

	platform_set_drvdata(msm_audio_snd_device, &msm_audio_snd_devdata);
	msm_audio_snd_devdata.dev = &msm_audio_snd_device->dev;
	ret = platform_device_add(msm_audio_snd_device);
	if (ret) {
		platform_device_put(msm_audio_snd_device);
		return ret;
	}
	mutex_init(&the_locks.lock);
	mutex_init(&the_locks.write_lock);
	mutex_init(&the_locks.read_lock);
	spin_lock_init(&the_locks.read_dsp_lock);
	spin_lock_init(&the_locks.write_dsp_lock);
	spin_lock_init(&the_locks.mixer_lock);
	init_waitqueue_head(&the_locks.enable_wait);
	init_waitqueue_head(&the_locks.eos_wait);
	init_waitqueue_head(&the_locks.write_wait);
	init_waitqueue_head(&the_locks.read_wait);

	return ret;
}

static void __exit msm_audio_exit(void)
{
	platform_device_unregister(msm_audio_snd_device);
}

module_init(msm_audio_init);
module_exit(msm_audio_exit);

MODULE_DESCRIPTION("PCM module");
MODULE_LICENSE("GPL v2");
