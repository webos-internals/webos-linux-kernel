/* arch/arm/mach-msm/board-chuck-headset-detect.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2009, Palm Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <asm/mach-types.h>

#include <asm/arch/gpio.h>
#include <asm/arch/msm_rpcrouter.h>
#include <asm/arch/board.h>

#include "headset-detect.h"
#include "proc_comm.h"

#define DRIVER_NAME	"headset-detect"

#define HS_SERVER_PROG 0x30000062
#define HS_SERVER_VERS 0x00010001


#define RPC_KEYPAD_NULL_PROC 0
#define RPC_KEYPAD_PASS_KEY_CODE_PROC 2
#define RPC_KEYPAD_SET_PWR_KEY_STATE_PROC 3

#define HS_PWR_K			0x6F	/* Power key */
#define HS_END_K			0x51	/* End key or Power key */
#define HS_STEREO_HEADSET_INSERT_K	0x82
#define HS_STEREO_HEADSET_REMOVE_K	0x83
#define HS_HEADSET_SWITCH_PRESS_K	0x84
#define HS_HEADSET_SWITCH_RELEASE_K	0x85
#define HS_REL_K			0xFF	/* key release */

#define HS_MICBIAS_THRESHOLD        0x50
#define HS_BUTTON_DEBOUNCE_PERIOD   100

struct msm_headset {
	struct input_dev *ipdev;
	int inserted;
	int mic_present;

	work_func_t headset_detect_func;
	struct work_struct headset_detect_work;

	struct timer_list button_dispatch_timer;  /* Button debounce */

	int button_pressed;
#define BUTTON_EVENT_QUEUE_SIZE    16
	enum {
		BUTTON_UP,
		BUTTON_DOWN,
	} button_events[BUTTON_EVENT_QUEUE_SIZE];
	int next_button_event;
};

static struct platform_device *hs_pdev;
static struct msm_headset *hs;

static uint32_t adc_detect_mic(struct msm_headset *dev)
{
	uint32_t data1;
	uint32_t data2;
	uint32_t ret;
	uint32_t mic_value;

	// turn on mic bias for detection
	data1 = PALM_VREG_ID_MIC;
	data2 = 1;
	ret = msm_proc_comm(PCOM_VREG_SWITCH, &data1, &data2);

	data1 = PALM_VREG_CODEC_WRITE;
	data2 = (ADIE_CODEC_EN3_R << 16) | ( ADIE_CODEC_EN3_MICBIAS_ENA_V << 8) | (ADIE_CODEC_EN3_EN_MICBIAS_M);
	ret = msm_proc_comm(PCOM_VREG_SWITCH, &data1, &data2);

	msleep(50);

	data1 = PALM_VREG_ADC_READ;
	data2 = PCOMM_PALM_ADC_HS_SWITCH_DETECT;
	ret = msm_proc_comm(PCOM_VREG_SWITCH, &data1, &data2);
	mic_value = data1;

	data1 = PALM_VREG_CODEC_WRITE;
	data2 = (ADIE_CODEC_EN3_R << 16) | ( ADIE_CODEC_EN3_MICBIAS_DIS_V << 8) | (ADIE_CODEC_EN3_EN_MICBIAS_M);
	ret = msm_proc_comm(PCOM_VREG_SWITCH, &data1, &data2);

	// turn off mic bias
	data1 = PALM_VREG_ID_MIC;
	data2 = 0;
	ret = msm_proc_comm(PCOM_VREG_SWITCH, &data1, &data2);
	

	if((mic_value > HS_MICBIAS_THRESHOLD) || !dev->inserted) {
		dev->mic_present = 0;
		return 0;
	}
	else {
		printk(KERN_INFO "headset_detect: headset mic found\n");
		input_report_switch(hs->ipdev, SW_HEADPHONE_MIC_INSERT, 1);
		/* Report headset removed if headset is inserted then mic is found
		*   so that states are synced up properly in hidd */
		input_report_switch(hs->ipdev, SW_HEADPHONE_INSERT, 0);
		dev->mic_present = 1;
		return 1;
	}
}

static ssize_t hsmic_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct msm_headset *d = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", adc_detect_mic(d));
}

static ssize_t headset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct msm_headset *d = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", d->inserted);
}

static ssize_t adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint32_t data1 = PALM_VREG_ID_MIC;
	uint32_t data2 = 1;
	uint32_t ret;

	data1 = PALM_VREG_ADC_READ;
	data2 = PCOMM_PALM_ADC_HS_SWITCH_DETECT;
	ret = msm_proc_comm(PCOM_VREG_SWITCH, &data1, &data2);

	return sprintf(buf, "0x%x\n", data1);
}

DEVICE_ATTR(headset_detect, S_IRUGO, headset_show, NULL);
DEVICE_ATTR(hsmic_detect, S_IRUGO, hsmic_show, NULL);
DEVICE_ATTR(adc_read, S_IRUGO, adc_show, NULL);


static void button_dispatch_handler(unsigned long arg)
{
	struct msm_headset *hs = (struct msm_headset *)arg;
	int i;

	if (!(hs->inserted && hs->mic_present)) {
		/* No button events if the headset is not inserted */
		hs->next_button_event = 0;
		hs->button_pressed = 0;
		return;
	}

	for (i = 0; i < hs->next_button_event; i++) {
		if (hs->button_events[i] == BUTTON_UP) {
            printk(KERN_INFO "headset_detect: reporting headset button released\n");
			input_report_key(hs->ipdev, KEY_PLAYPAUSE, BUTTON_UP);
		} else {
            printk(KERN_INFO "headset_detect: reporting headset button pressed\n");
			input_report_key(hs->ipdev, KEY_PLAYPAUSE, BUTTON_DOWN);
		}
	}
	hs->next_button_event = 0;

}

static void button_press_handler(struct msm_headset *hs, int state)
{
	if (hs->next_button_event >= (BUTTON_EVENT_QUEUE_SIZE)) {
		printk(KERN_WARNING "headset_detect: Too many button events, ignoring last event\n");
		return;
	}

	if (state) {
		if (!hs->button_pressed) {
			hs->button_pressed = 1;

			hs->button_events[hs->next_button_event] = BUTTON_DOWN;
			hs->next_button_event += 1;
			mod_timer(&hs->button_dispatch_timer,
					jiffies + msecs_to_jiffies(HS_BUTTON_DEBOUNCE_PERIOD));
		}
	}
	else {
		if (hs->button_pressed) {
			hs->button_pressed = 0;

			hs->button_events[hs->next_button_event] = BUTTON_UP;
			hs->next_button_event += 1;
			mod_timer(&hs->button_dispatch_timer,
					jiffies + msecs_to_jiffies(HS_BUTTON_DEBOUNCE_PERIOD));
		}
	}
}


static void report_hs_key(uint32_t key_code)
{
	switch (key_code) {
	case HS_STEREO_HEADSET_INSERT_K:
		printk(KERN_INFO "headset_detect: headset inserted\n");
		input_report_switch(hs->ipdev, SW_HEADPHONE_INSERT, 1);
		hs->inserted = 1;
		hs->mic_present = 0;
		break;
	case HS_STEREO_HEADSET_REMOVE_K:
		if(hs->mic_present) {
			printk(KERN_INFO "headset_detect: headset mic removed\n");
			del_timer(&hs->button_dispatch_timer);  /* cancel any pending timer and immediately disable */
			/* If the button is down, send a button up event */
			if (hs->button_pressed)
				input_report_key(hs->ipdev, KEY_PLAYPAUSE, BUTTON_UP);

			input_report_switch(hs->ipdev, SW_HEADPHONE_MIC_INSERT, 0);
		}
		else {
			printk(KERN_INFO "headset_detect: headset removed\n");
			input_report_switch(hs->ipdev, SW_HEADPHONE_INSERT, 0);
		}
		hs->mic_present = 0;
		hs->inserted = 0;
		hs->button_pressed = 0;
		hs->next_button_event = 0;
		break;
	case HS_HEADSET_SWITCH_PRESS_K:
		if(hs->mic_present) {
			printk(KERN_INFO "headset_detect: button press received from modem\n");
			button_press_handler(hs, BUTTON_DOWN);
		}
		else {
			printk(KERN_WARNING "headset_detect: button press from modem with no mic, do not send\n");
		}
		break;
	case HS_HEADSET_SWITCH_RELEASE_K:
		if(hs->mic_present) {
			printk(KERN_INFO "headset_detect: button release received from modem\n");
			button_press_handler(hs, BUTTON_UP);
		}
		else {
			printk(KERN_WARNING "headset_detect: button release from modem with no mic, do not send\n");
		}
		break;
	case -1:
		printk(KERN_ERR "%s: No mapping for remote headset event %d\n",
				 __func__, key_code);
		break;
	}
}

static int handle_hs_rpc_call(struct msm_rpc_server *server,
			   struct rpc_request_hdr *req, unsigned len)
{
	struct rpc_keypad_pass_key_code_args {
		uint32_t key_code;
		uint32_t key_parm;
	};

	switch (req->procedure) {
	case RPC_KEYPAD_NULL_PROC:
		return 0;

	case RPC_KEYPAD_PASS_KEY_CODE_PROC: {
		struct rpc_keypad_pass_key_code_args *args;

		args = (struct rpc_keypad_pass_key_code_args *)(req + 1);
		args->key_code = be32_to_cpu(args->key_code);
		args->key_parm = be32_to_cpu(args->key_parm);

		report_hs_key(args->key_code);

		return 0;
	}

	case RPC_KEYPAD_SET_PWR_KEY_STATE_PROC:
		/* This RPC function must be available for the ARM9
		 * to function properly.  This function is redundant
		 * when RPC_KEYPAD_PASS_KEY_CODE_PROC is handled. So
		 * input_report_key is not needed.
		 */
		return 0;
	default:
		return -ENODEV;
	}
}

static void headset_detect_handler(struct work_struct *work)
{
	struct msm_headset *dev;
	uint32_t cmd = PCOMM_PALM_DIAG_HEADSET_STATUS;
	uint32_t data2 = 0;
	uint32_t ret = 0;

	dev = container_of(work, struct msm_headset, headset_detect_work);

	ret = msm_proc_comm(PCOM_PALM_DIAG_CMDS, &cmd, &data2);

	if(ret)
	{
		printk(KERN_ERR "headset_detect_handler: failed to read status from proc comm\n");
		return;
	}

	printk(KERN_INFO "headset_detect_handler: status=0x%x\n", data2);

	if((data2 == PALM_HEADSET_DETECTED) || (data2 == PALM_STEREO_HEADSET_DETECTED))
	{
		printk(KERN_INFO "headset_detect: headset inserted on boot\n");
		report_hs_key(HS_STEREO_HEADSET_INSERT_K);
	}
}

static int headset_publish_sysfs(void)
{
	int r;

	/* Add entries in /sys to query state of headset */
	r = device_create_file(&hs_pdev->dev, &dev_attr_headset_detect);
	if (r) {
		printk(KERN_ERR "HS-DET: Failed to create headset_detect in /sys: %d\n", r);
	}

	r = device_create_file(&hs_pdev->dev, &dev_attr_adc_read);
	if(r) {
		printk(KERN_ERR "ADC-READ: Failed to create adc_show in /sys %d\n", r);
	}

	r = device_create_file(&hs_pdev->dev, &dev_attr_hsmic_detect);
	if(r) {
		printk(KERN_ERR "HS-MIC: Failed to create hsmic_detect in /sys: %d\n", r);
	}

	return 0;
}


static struct msm_rpc_server hs_rpc_server = {
	.prog		= HS_SERVER_PROG,
	.vers		= HS_SERVER_VERS,
	.rpc_call	= handle_hs_rpc_call,
};

static int __devinit hs_rpc_init(void)
{
	return msm_rpc_create_server(&hs_rpc_server);
}

static void __devexit hs_rpc_deinit(void)
{
}

static int __devinit hs_probe(struct platform_device *pdev)
{
	int rc;
	struct input_dev *ipdev;

	hs = kzalloc(sizeof(struct msm_headset), GFP_KERNEL);
	if (!hs)
		return -ENOMEM;

	ipdev = input_allocate_device();
	if (!ipdev) {
		rc = -ENOMEM;
		goto err_alloc_input_dev;
	}
	input_set_drvdata(ipdev, hs);

	hs->headset_detect_func = headset_detect_handler;
	INIT_WORK(&hs->headset_detect_work, hs->headset_detect_func);

	init_timer(&hs->button_dispatch_timer);
	hs->button_dispatch_timer.function = button_dispatch_handler;
	hs->button_dispatch_timer.data = (unsigned long)hs;

	hs->ipdev = ipdev;
	hs->inserted = 0;
	hs->mic_present = 0;
	hs->button_pressed = 0;
	hs->next_button_event = 0;

	ipdev->name = "headset";

	ipdev->id.bustype = BUS_VIRTUAL;
	ipdev->id.vendor = 0x0;
	ipdev->id.product = 0x0;
	ipdev->id.version = 0x100;

	//ipdev->dev.parent = &dev->pdev->dev;
	ipdev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SW);
	set_bit(KEY_PLAYPAUSE, ipdev->keybit);
	set_bit(SW_HEADPHONE_INSERT, ipdev->swbit);
	set_bit(SW_HEADPHONE_MIC_INSERT, ipdev->swbit);

	rc = input_register_device(ipdev);
	if (rc) {
		dev_err(&ipdev->dev,
				"hs_probe: input_register_device rc=%d\n", rc);
		goto err_reg_input_dev;
	}

	platform_set_drvdata(pdev, hs);

	headset_publish_sysfs();

	rc = hs_rpc_init();
	if (rc)
		goto err_hs_rpc_init;

	/* check headset state on wake */
	schedule_work(&hs->headset_detect_work);

	return 0;

err_hs_rpc_init:
	input_unregister_device(ipdev);
	ipdev = NULL;
err_reg_input_dev:
	input_free_device(ipdev);
err_alloc_input_dev:
	kfree(hs);
	return rc;
}

static int __devexit hs_remove(struct platform_device *pdev)
{
	struct msm_headset *hs = platform_get_drvdata(pdev);

	input_unregister_device(hs->ipdev);
	kfree(hs);
	hs_rpc_deinit();
	return 0;
}

static struct platform_driver hs_driver = {
	.probe		= hs_probe,
	.remove		= __devexit_p(hs_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init hs_init(void)
{
	int rc;

	hs_pdev = platform_device_register_simple(DRIVER_NAME,
					-1, NULL, 0);
	if (IS_ERR(hs_pdev))
		return PTR_ERR(hs_pdev);

	rc = platform_driver_register(&hs_driver);
	if (rc) {
		platform_device_unregister(hs_pdev);
		return rc;
	}

	return 0;
}
module_init(hs_init);

static void __exit hs_exit(void)
{
	platform_driver_unregister(&hs_driver);
	platform_device_unregister(hs_pdev);
}
module_exit(hs_exit);

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm-headset");
