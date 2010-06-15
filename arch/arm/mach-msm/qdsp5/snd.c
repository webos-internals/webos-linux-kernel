/* arch/arm/mach-msm/qdsp5/snd.c
 *
 * interface to "snd" service on the baseband cpu
 *
 * Copyright (C) 2008 HTC Corporation
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include <asm/arch/qdsp5/debug_audio_mm.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/msm_audio.h>

#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <asm/arch/board.h>
#include <asm/arch/msm_rpcrouter.h>
#include <asm/arch/vreg.h>

struct snd_ctxt {
	struct mutex lock;
	int opened;
	struct msm_rpc_endpoint *ept;
	struct msm_snd_endpoints *snd_epts;

	struct vreg *vreg;
	unsigned int vreg_enabled;
};

static struct snd_ctxt the_snd;

#define RPC_SND_PROG	0x30000002
#define RPC_SND_CB_PROG	0x31000002

#define RPC_SND_VERS                    0x00020001

#define SND_SET_DEVICE_PROC 2
#define SND_SET_VOLUME_PROC 3
#define SND_AVC_CTL_PROC 29
#define SND_AGC_CTL_PROC 30

struct rpc_snd_set_device_args {
	uint32_t device;
	uint32_t ear_mute;
	uint32_t mic_mute;

	uint32_t cb_func;
	uint32_t client_data;
};

struct rpc_snd_set_volume_args {
	uint32_t device;
	uint32_t method;
	uint32_t volume;

	uint32_t cb_func;
	uint32_t client_data;
};

struct rpc_snd_avc_ctl_args {
	uint32_t avc_ctl;
	uint32_t cb_func;
	uint32_t client_data;
};

struct rpc_snd_agc_ctl_args {
	uint32_t agc_ctl;
	uint32_t cb_func;
	uint32_t client_data;
};

struct snd_set_device_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_set_device_args args;
};

struct snd_set_volume_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_set_volume_args args;
};

struct snd_avc_ctl_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_avc_ctl_args args;
};

struct snd_agc_ctl_msg {
	struct rpc_request_hdr hdr;
	struct rpc_snd_agc_ctl_args args;
};

struct snd_endpoint *get_snd_endpoints(int *size);

static inline int check_mute(int mute)
{
	return (mute == SND_MUTE_MUTED ||
		mute == SND_MUTE_UNMUTED) ? 0 : -EINVAL;
}

static int get_endpoint(struct snd_ctxt *snd, unsigned long arg)
{
	int rc = 0, index;
	struct msm_snd_endpoint ept;

	if (copy_from_user(&ept, (void __user *)arg, sizeof(ept))) {
		MM_ERR("snd_ioctl get endpoint: invalid read pointer\n");
		return -EFAULT;
	}

	index = ept.id;
	if (index < 0 || index >= snd->snd_epts->num) {
		MM_ERR("snd_ioctl get endpoint: invalid index!\n");
		return -EINVAL;
	}

	ept.id = snd->snd_epts->endpoints[index].id;
	strncpy(ept.name,
		snd->snd_epts->endpoints[index].name,
		sizeof(ept.name));

	if (copy_to_user((void __user *)arg, &ept, sizeof(ept))) {
		MM_ERR("snd_ioctl get endpoint: invalid write pointer\n");
		rc = -EFAULT;
	}

	return rc;
}

static long snd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct snd_set_device_msg dmsg;
	struct snd_set_volume_msg vmsg;
	struct snd_avc_ctl_msg avc_msg;
	struct snd_agc_ctl_msg agc_msg;

	struct msm_snd_device_config dev;
	struct msm_snd_volume_config vol;
	struct snd_ctxt *snd = file->private_data;
	int rc = 0;

	uint32_t avc, agc;

	mutex_lock(&snd->lock);
	switch (cmd) {
	case SND_SET_DEVICE:
		if (copy_from_user(&dev, (void __user *) arg, sizeof(dev))) {
			MM_ERR("set device: invalid pointer\n");
			rc = -EFAULT;
			break;
		}

		dmsg.args.device = cpu_to_be32(dev.device);
		dmsg.args.ear_mute = cpu_to_be32(dev.ear_mute);
		dmsg.args.mic_mute = cpu_to_be32(dev.mic_mute);
		if (check_mute(dev.ear_mute) < 0 ||
				check_mute(dev.mic_mute) < 0) {
			MM_ERR("set device: invalid mute status\n");
			rc = -EINVAL;
			break;
		}
		dmsg.args.cb_func = -1;
		dmsg.args.client_data = 0;

		MM_INFO("snd_set_device %d %d %d\n", dev.device,
				dev.ear_mute, dev.mic_mute);

		rc = msm_rpc_call(snd->ept,
			SND_SET_DEVICE_PROC,
			&dmsg, sizeof(dmsg), 5 * HZ);
		break;

	case SND_SET_VOLUME:
		if (copy_from_user(&vol, (void __user *) arg, sizeof(vol))) {
			MM_ERR("set volume: invalid pointer\n");
			rc = -EFAULT;
			break;
		}

		vmsg.args.device = cpu_to_be32(vol.device);
		vmsg.args.method = cpu_to_be32(vol.method);
		if (vol.method != SND_METHOD_VOICE) {
			MM_ERR("set volume: invalid method\n");
			rc = -EINVAL;
			break;
		}

		vmsg.args.volume = cpu_to_be32(vol.volume);
		vmsg.args.cb_func = -1;
		vmsg.args.client_data = 0;

		MM_INFO("snd_set_volume %d %d %d\n", vol.device,
				vol.method, vol.volume);

		rc = msm_rpc_call(snd->ept,
			SND_SET_VOLUME_PROC,
			&vmsg, sizeof(vmsg), 5 * HZ);
		break;

	case SND_AVC_CTL:
		if (get_user(avc, (uint32_t __user *) arg)) {
			rc = -EFAULT;
			break;
		} else if ((avc != 1) && (avc != 0)) {
			rc = -EINVAL;
			break;
		}

		avc_msg.args.avc_ctl = cpu_to_be32(avc);

		vmsg.args.cb_func = -1;
		vmsg.args.client_data = 0;

		MM_INFO("snd_avc_ctl %d\n", avc);

		rc = msm_rpc_call(snd->ept,
			SND_AVC_CTL_PROC,
			&avc_msg, sizeof(avc_msg), 5 * HZ);
		break;

	case SND_AGC_CTL:
		if (get_user(agc, (uint32_t __user *) arg)) {
			rc = -EFAULT;
			break;
		} else if ((agc != 1) && (agc != 0)) {
			rc = -EINVAL;
			break;
		}
		agc_msg.args.agc_ctl = cpu_to_be32(agc);

		agc_msg.args.cb_func = -1;
		agc_msg.args.client_data = 0;

		MM_INFO("snd_agc_ctl %d\n", agc);

		rc = msm_rpc_call(snd->ept,
			SND_AGC_CTL_PROC,
			&agc_msg, sizeof(agc_msg), 5 * HZ);
		break;

	case SND_GET_NUM_ENDPOINTS:
		if (copy_to_user((void __user *)arg,
				&snd->snd_epts->num, sizeof(unsigned))) {
			MM_ERR("get endpoint: invalid pointer\n");
			rc = -EFAULT;
		}
		break;

	case SND_GET_ENDPOINT:
		rc = get_endpoint(snd, arg);
		break;

	case SND_VREG_ENABLE:
		if (arg) {
			if (!IS_ERR(snd->vreg) && !snd->vreg_enabled) {
				vreg_enable(snd->vreg);
				snd->vreg_enabled = 1;
				pr_info("SND_VREG_ENABLE: enabling usb vreg\n");
			}
		}
		else {
			if (!IS_ERR(snd->vreg) && snd->vreg_enabled) {
				vreg_disable(snd->vreg);
				snd->vreg_enabled = 0;
				pr_info("SND_VREG_ENABLE: disabling usb vreg\n");
			}
		}
		break;

	default:
		MM_ERR("unknown command\n");
		rc = -EINVAL;
		break;
	}
	mutex_unlock(&snd->lock);

	return rc;
}

static int snd_release(struct inode *inode, struct file *file)
{
	struct snd_ctxt *snd = file->private_data;
	int rc;

	mutex_lock(&snd->lock);
	rc = msm_rpc_close(snd->ept);
	if (rc < 0)
		MM_ERR("msm_rpc_close failed\n");
	snd->ept = NULL;
	snd->opened = 0;
	mutex_unlock(&snd->lock);
	return 0;
}

static int snd_open(struct inode *inode, struct file *file)
{
	struct snd_ctxt *snd = &the_snd;
	int rc = 0;

	mutex_lock(&snd->lock);
	if (snd->opened == 0) {
		if (snd->ept == NULL) {
			snd->ept = msm_rpc_connect_compatible(RPC_SND_PROG,
					RPC_SND_VERS, 0);
			if (IS_ERR(snd->ept)) {
				rc = PTR_ERR(snd->ept);
				snd->ept = NULL;
				MM_ERR("failed to connect snd svc\n");
				goto err;
			}
		}
		file->private_data = snd;
		snd->opened = 1;
	} else {
		MM_ERR("snd already opened\n");
		rc = -EBUSY;
	}

err:
	mutex_unlock(&snd->lock);
	return rc;
}

static struct file_operations snd_fops = {
	.owner		= THIS_MODULE,
	.open		= snd_open,
	.release	= snd_release,
	.unlocked_ioctl	= snd_ioctl,
};

struct miscdevice snd_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_snd",
	.fops	= &snd_fops,
};

static int snd_probe(struct platform_device *pdev)
{
	struct snd_ctxt *snd = &the_snd;
	mutex_init(&snd->lock);
	snd->snd_epts = (struct msm_snd_endpoints *)pdev->dev.platform_data;

	snd->vreg = vreg_get(NULL, "usb");
	if (IS_ERR(snd->vreg) || (!snd->vreg)) {
		pr_err("%s: vreg get failed\n", __func__);
		snd->vreg = NULL;
	}

	return misc_register(&snd_misc);
}

static struct platform_driver snd_plat_driver = {
	.probe = snd_probe,
	.driver = {
		.name = "msm_snd",
		.owner = THIS_MODULE,
	},
};

static int __init snd_init(void)
{
	return platform_driver_register(&snd_plat_driver);
}

module_init(snd_init);
