#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/twl4030-audio.h>
#include <asm/arch/twl4030-madc.h>

#include "headset-detect.h"

#define SIRLOIN_GPIO_HS_PLUG_IN       111
#define SIRLOIN_GPIO_HS_AMP_SD_N      86
#define SIRLOIN_GPIO_MIC_ANS_INT_PWM  87

#define MIC_ANS_PWM_GPT     11
#define MIC_ANS_PWM_TLDR    0xfffffce0 /* 25 ms with 32-khz srcclk */

#define HS_INSERT_DELAY            1000
#define HS_BUTTON_POLL_PERIOD      100
#define HS_BUTTON_DEBOUNCE_PERIOD  500
#define HS_BUTTON_UP_TIMEOUT       50
#define HS_TURNON_DELAY		   40 // time to wait for hardware to settle before powering amplifier

/* udev */
#define HS_DEVICE_HS       "HS_DEVICE=headset"
#define HS_DEVICE_HSMIC    "HS_DEVICE=headset-mic"
#define HS_DEVICE_NONE     "HS_DEVICE="
#define HS_BUTTON_DOWN     "HS_BUTTON=1"
#define HS_BUTTON_UP       "HS_BUTTON=0"
#define HS_BUTTON_NONE     "HS_BUTTON="

#define HS_BUTTON_USE_PWM_IN_WAKE    1

#define INPUT_DEVICE_PHYS   "headset/input0"

#define DEBUG

#ifdef DEBUG
#define dprintk(x ...)    printk(x)
#else
#define dprintk(x ...)
#endif

enum {
	NONE,
	HEADSET,
	HEADSET_MIC,
} forced_device = -1;

struct headset_device {
	const char *name;
	struct platform_device *pdev;
	struct input_dev *idev;

	/* Well, this is completely sirloin specific anyway */
	unsigned gpio_hs_plug_in;
	unsigned gpio_hs_amp_sd_n;
	unsigned gpio_mic_ans_int_pwm;

	struct omap_dm_timer *pwm;

	int audio_on;

	int polarity;    /* EVT0/1 active low, EVT2+ active high */
	int headset_inserted;
	int mic_present;
	work_func_t headset_detect_func;
	struct work_struct headset_detect_work;

	work_func_t pwm_int_func;
	struct work_struct pwm_int_work;

	struct timer_list headset_detect_timer;
	struct timer_list button_up_timer;        /* To fake the button up event */
	struct timer_list button_dispatch_timer;  /* Button debounce */
	struct timer_list enable_headset_timer;   /* headset turn on delay */

	int button_pressed;

#define BUTTON_EVENT_QUEUE_SIZE    16
	enum {
		BUTTON_UP,
		BUTTON_DOWN,
	} button_events[BUTTON_EVENT_QUEUE_SIZE];
	int next_button_event;
	int last_polled_button;
};

static void headset_amp_enable(struct headset_device *dev, int enable)
{
	gpio_set_value(dev->gpio_hs_amp_sd_n, enable);
}

static void delayed_headset_enable_handler(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;
	headset_amp_enable(dev, dev->audio_on && dev->headset_inserted);
}

static void headset_codec_event(void *cookie, int event)
{
	struct headset_device *dev = cookie;

	dev->audio_on = (event == CODEC_ENABLED);
	if (event== CODEC_ENABLED) {
		mod_timer(&dev->enable_headset_timer,
			jiffies + msecs_to_jiffies(HS_TURNON_DELAY));
	} else {
		del_timer(&dev->enable_headset_timer);  // cancel any pending timer and immediately disable
		headset_amp_enable(dev, dev->audio_on && dev->headset_inserted);
	}
}

static uint16_t adcin_read(int adc)
{
	int r = 0;
	uint16_t v;

	r = madc_start_conversion(adc);
	if (r < 0) {
		printk(KERN_ERR "HS-DET: Failed to read ADCIN0\n");
		return 0;
	}
	v = (uint16_t)r;

	/* If there is no mic micbias will be pulled low */
	return v;
}

#define ADCIN0_READ()    adcin_read(0)
#define MIC_PRESENT()    (adcin_read(0) >= 0x100)

static void pwm_enable(struct headset_device *dev)
{
	omap_dm_timer_enable(dev->pwm);

	omap_dm_timer_stop(dev->pwm);
	omap_dm_timer_set_load(dev->pwm, 1, MIC_ANS_PWM_TLDR);
	omap_dm_timer_set_pwm(dev->pwm, 1, 1, OMAP_TIMER_TRIGGER_OVERFLOW);
	omap_dm_timer_start(dev->pwm);
}

static void pwm_disable(struct headset_device *dev)
{
	omap_dm_timer_enable(dev->pwm);
	omap_dm_timer_stop(dev->pwm);
	omap_dm_timer_disable(dev->pwm);
}

static ssize_t headset_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct headset_device *d = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", d->headset_inserted);
}

static ssize_t hsmic_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct headset_device *d = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", d->mic_present);
	return sprintf(buf, "%#x\n", ADCIN0_READ());
}

static ssize_t adc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%#x\n", ADCIN0_READ());
}

static ssize_t pol_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct headset_device *hdev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", hdev->polarity);
}

static ssize_t pol_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct headset_device *hdev = dev_get_drvdata(dev);
	if (count > 0) {
		hdev->polarity = !(buf[0] == '0');
	}
	return count;
}

DEVICE_ATTR(headset_detect, S_IRUGO, headset_show, NULL);
DEVICE_ATTR(hsmic_detect, S_IRUGO, hsmic_show, NULL);

DEVICE_ATTR(adc, S_IRUGO, adc_show, NULL);
DEVICE_ATTR(pol_ctl, S_IRUGO | S_IWUGO, pol_show, pol_store);

extern int twl4030_audio_mic_bias_enable(bool);

static void headset_insert_handler(struct headset_device *dev)
{
	int mic;
	char *envp[3];

	/* Check if there is a mic on the headset */
	twl4030_audio_mic_bias_enable(true);
	mic = MIC_PRESENT();
	twl4030_audio_mic_bias_enable(false);

	if (mic != dev->mic_present) {
		dev->mic_present = mic;
		if (mic) {
			twl4030_audio_mic_bias_enable(true);
			pwm_enable(dev);
		} else {
			twl4030_audio_mic_bias_enable(false);
		}
	}

	headset_amp_enable(dev, dev->audio_on);

	if (dev->mic_present) {
		input_report_switch(dev->idev, SW_HEADPHONE_MIC_INSERT, 1);
	} else {
		input_report_switch(dev->idev, SW_HEADPHONE_INSERT, 1);
	}

	/* Send a udev event also.
	 * hidd uses the input events but audiod still uses the udev events. */
	envp[0] = HS_BUTTON_NONE;
	if (dev->mic_present) {
		envp[1] = HS_DEVICE_HSMIC;
	} else {
		envp[1] = HS_DEVICE_HS;
	}
	envp[2] = NULL;
	kobject_uevent_env(&dev->pdev->dev.kobj, KOBJ_CHANGE, envp);
}

static void headset_remove_handler(struct headset_device *dev)
{
	int mic_was_present;
	char *envp[3];

	mic_was_present = dev->mic_present;

	/* Can't have a headset button or mic if nothing is inserted, 
	 * cancel the work related to it */
	if (dev->mic_present) {
		dev->mic_present = false;
		pwm_disable(dev);
		twl4030_audio_mic_bias_enable(false);
	}

	/* FIXME only turn on the amp if audio is streaming */
	gpio_set_value(dev->gpio_hs_amp_sd_n, 0);

	/* If the button is down, send a button up event */
	if (dev->button_pressed) {
		input_report_key(dev->idev, KEY_PLAYPAUSE, 0);
	}
	if (mic_was_present) {
		input_report_switch(dev->idev, SW_HEADPHONE_MIC_INSERT, 0);
	} else {
		input_report_switch(dev->idev, SW_HEADPHONE_INSERT, 0);
	}

	/* FIXME Send a udev event also until hidd, audiod makes the
	 * corresponding changes */
	if (dev->button_pressed) {
		envp[0] = HS_BUTTON_UP;
	} else {
		envp[0] = HS_BUTTON_NONE;
	}
	envp[1] = HS_DEVICE_NONE;
	envp[2] = NULL;
	kobject_uevent_env(&dev->pdev->dev.kobj, KOBJ_CHANGE, envp);

	dev->button_pressed = 0;
}

static inline int headset_inserted(struct headset_device *dev)
{
	int ins = !(omap_get_gpio_level(dev->gpio_hs_plug_in));
	if (dev->polarity) {
		ins = !ins;
	}
	return ins;
}

static void headset_detect_handler(struct work_struct *work)
{
	struct headset_device *dev;
	int ins;

	dev = container_of(work, struct headset_device, headset_detect_work);

	/* Check HS-PLUG-IN. low = inserted, high = not inserted */
	ins = headset_inserted(dev);

	if (ins != dev->headset_inserted) {
		if (ins) {
			headset_insert_handler(dev);
		} else {
			headset_remove_handler(dev);
		}
		dev->headset_inserted = ins;
	}
}

static void pwm_int_handler(struct work_struct *work)
{
	struct headset_device *dev;

	dev = container_of(work, struct headset_device, pwm_int_work);

	if (!dev->button_pressed) {
		dev->button_pressed = 1;

		dev->button_events[dev->next_button_event] = BUTTON_DOWN;
		dev->next_button_event += 1;

		mod_timer(&dev->button_dispatch_timer, 
				jiffies + msecs_to_jiffies(HS_BUTTON_DEBOUNCE_PERIOD));
	}
}

static struct headset_device sirloin_headset_device = {
	.name = "headset-detect",
	.gpio_hs_plug_in = SIRLOIN_GPIO_HS_PLUG_IN,
	.gpio_hs_amp_sd_n = SIRLOIN_GPIO_HS_AMP_SD_N,
	.gpio_mic_ans_int_pwm = SIRLOIN_GPIO_MIC_ANS_INT_PWM,
	.headset_inserted = 0,
	.mic_present = 0,
	.button_pressed = 0,
	.next_button_event = 0,
	.pwm_int_func = pwm_int_handler,
	.headset_detect_func = headset_detect_handler,
};

static irqreturn_t mic_ans_int_pwm_irq(int irq, void *dev_id)
{
	struct headset_device *dev = (struct headset_device *)dev_id;

	schedule_work(&dev->pwm_int_work);
	mod_timer(&dev->button_up_timer,
			jiffies + msecs_to_jiffies(HS_BUTTON_UP_TIMEOUT));

	return IRQ_HANDLED;
}

static void button_dispatch(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;
	int i;
	char *envp[3];

	if (!(dev->headset_inserted && dev->mic_present) ||
			!headset_inserted(dev)) {
		/* No button events if the headset is not inserted */
		dev->next_button_event = 0;
		return;
	}

	/* FIXME Send a udev event also until hidd, audiod makes the
	 * corresponding changes */
	envp[1] = HS_DEVICE_HSMIC;
	envp[2] = NULL;
	for (i = 0; i < dev->next_button_event; i++) {
		if (dev->button_events[i] == BUTTON_UP) {
			envp[0] = HS_BUTTON_UP;
		} else {
			envp[0] = HS_BUTTON_DOWN;
		}
		kobject_uevent_env(&dev->pdev->dev.kobj, KOBJ_CHANGE, envp);
	}

	for (i = 0; i < dev->next_button_event; i++) {
		if (dev->button_events[i] == BUTTON_UP) {
			input_report_key(dev->idev, KEY_PLAYPAUSE, 0);
		} else {
			input_report_key(dev->idev, KEY_PLAYPAUSE, 1);
		}
	}
	dev->next_button_event = 0;

}

static void button_up_timeout_handler(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;

	if (dev->button_pressed) {
		dev->button_pressed = 0;

		dev->button_events[dev->next_button_event] = BUTTON_UP;
		dev->next_button_event += 1;

		mod_timer(&dev->button_dispatch_timer, 
				jiffies + msecs_to_jiffies(HS_BUTTON_DEBOUNCE_PERIOD));
	}
}

static irqreturn_t hs_plug_in_irq(int irq, void *dev_id)
{
	struct headset_device *dev = (struct headset_device *)dev_id;

	mod_timer(&dev->headset_detect_timer, 
			jiffies + msecs_to_jiffies(HS_INSERT_DELAY));

	return IRQ_HANDLED;
}

static void headset_detect_debounced(unsigned long arg)
{
	struct headset_device *dev = (struct headset_device *)arg;

	schedule_work(&dev->headset_detect_work);
}

static int headset_register_input_device(struct headset_device *dev, struct input_dev **input_device)
{
	int r;
	struct input_dev *idev = NULL;

	/* Publish as an input device */
	idev = input_allocate_device();
	if (!idev) {
		return -ENOMEM;
	}

	idev->name = "headset";

	idev->id.bustype = BUS_VIRTUAL;
	idev->id.vendor = 0x0;
	idev->id.product = 0x0;
	idev->id.version = 0x100;

	idev->dev.parent = &dev->pdev->dev;
	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_SW);
	set_bit(KEY_PLAYPAUSE, idev->keybit);
	set_bit(SW_HEADPHONE_INSERT, idev->swbit);
	set_bit(SW_HEADPHONE_MIC_INSERT, idev->swbit);

	r = input_register_device(idev);
	if (r) {
		goto fail;
	}

	*input_device = idev;
	return 0;
fail:
	input_free_device(idev);
	return r;
}

static int headset_publish_sysfs(struct headset_device *dev)
{
	int r;

	/* Add entries in /sys to query state of headset */
	r = device_create_file(&dev->pdev->dev, &dev_attr_headset_detect);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to create headset_detect in /sys: %d\n", r);
	}
	r = device_create_file(&dev->pdev->dev, &dev_attr_hsmic_detect);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to create hsmic_detect in /sys: %d\n", r);
	}

	r = device_create_file(&dev->pdev->dev, &dev_attr_adc);
	if (r) {
		printk(KERN_ERR
				"HS-DET: Failed to create adc in /sys: %d\n", r);
	}
	r = device_create_file(&dev->pdev->dev, &dev_attr_pol_ctl);
	if (r) {
		printk(KERN_ERR
				"HS-DET: Failed to create pol_ctl in /sys: %d\n", r);
	}
	return 0;
}

static int __init headset_probe(struct platform_device *pdev)
{
	int r = 0;
	struct headset_device *dev = &sirloin_headset_device;
	struct sirloin_headset_detect_platform_data *pdata = 
		pdev->dev.platform_data;
	struct input_dev *idev = NULL;

	r = gpio_request(dev->gpio_hs_plug_in, "gpio");
	if (r) {
		printk(KERN_ERR "HS-DET: Couldn't request HS-PLUG-IN (gpio_%u)\n",
				dev->gpio_hs_plug_in);
		goto fail;
	}

	r = gpio_request(dev->gpio_hs_amp_sd_n, "gpio");
	if (r) {
		printk(KERN_ERR "HS-DET: Couldn't request HS-AMP-SD~ (gpio_%u)\n",
				dev->gpio_hs_amp_sd_n);
		goto fail;
	}

	r = gpio_request(dev->gpio_mic_ans_int_pwm, "gpio");
	if (r) {
		printk(KERN_ERR "HS-DET: Couldn't request MIC-ANS-INT-PWM (gpio_%u)\n",
				dev->gpio_mic_ans_int_pwm);
		goto fail;
	}

	/* Interrupt sources are inputs */
	gpio_direction_input(dev->gpio_hs_plug_in);
	gpio_direction_input(dev->gpio_mic_ans_int_pwm);

	/* Headset amp is off by default */
	gpio_direction_output(dev->gpio_hs_amp_sd_n, 0);

	/* Configure the pinmux for the GPIOs */
	omap_cfg_reg("B26_3430_GPIO111");    /* HS-PLUG-IN */
	omap_cfg_reg("G25_3430_GPIO086");    /* HS-AMP-SD~ */
	omap_cfg_reg("Y4_3430_MIC_ANS_PWM"); /* MIC-ANS-PWM */
	omap_cfg_reg("H27_3430_GPIO087");    /* MIC-ANS-INT-PWM */

	dev->pwm = omap_dm_timer_request_specific(MIC_ANS_PWM_GPT);
	if (dev->pwm == NULL) {
		printk(KERN_ERR "HS-DET: Couldn't request gpt%d\n", MIC_ANS_PWM_GPT);
		return r;
	}

	omap_dm_timer_stop(dev->pwm);
	omap_dm_timer_set_int_enable(dev->pwm, 0);
	omap_dm_timer_write_counter(dev->pwm, 0);
	omap_dm_timer_set_match(dev->pwm, 0, 0);    /* Not using match */
	omap_dm_timer_set_load(dev->pwm, 1, 0);
	omap_dm_timer_set_source(dev->pwm, OMAP_TIMER_SRC_32_KHZ);
	pwm_disable(dev);

	dev->polarity = pdata->polarity;

	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);

	/* Init the workqueues */
	INIT_WORK(&dev->headset_detect_work, dev->headset_detect_func);
	INIT_WORK(&dev->pwm_int_work, dev->pwm_int_func);

	init_timer(&dev->headset_detect_timer);
	dev->headset_detect_timer.function = headset_detect_debounced;
	dev->headset_detect_timer.data = (unsigned long)dev;

	init_timer(&dev->button_up_timer);
	dev->button_up_timer.function = button_up_timeout_handler;
	dev->button_up_timer.data = (unsigned long)dev;

	init_timer(&dev->button_dispatch_timer);
	dev->button_dispatch_timer.function = button_dispatch;
	dev->button_dispatch_timer.data = (unsigned long)dev;

	init_timer(&dev->enable_headset_timer);
	dev->enable_headset_timer.function = delayed_headset_enable_handler;
	dev->enable_headset_timer.data = (unsigned long)dev;


	/* Request HS-PLUG-IN IRQ on both edges to get both insert and
	 * remove interrupts */
	r = request_irq(OMAP_GPIO_IRQ(dev->gpio_hs_plug_in), hs_plug_in_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, dev->name, dev);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to request irq on hs_plug_in (gpio_%u)\n",
				dev->gpio_hs_plug_in);
		/* FIXME cleanup */
	}

	/* Request IRQ for MIC-ANS-INT-PWM to get interrupts for the
	 * headset button when the PWM is used */
	/* FIXME Don't know if we need both edges */
	r = request_irq(OMAP_GPIO_IRQ(dev->gpio_mic_ans_int_pwm), 
			mic_ans_int_pwm_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			dev->name, dev);
	if (r) {
		printk(KERN_ERR 
				"HS-DET: Failed to request irq on MIC-ANS-INT-PWM (gpio_%u)\n",
				dev->gpio_mic_ans_int_pwm);
		/* FIXME cleanup */
	}

	r = headset_register_input_device(dev, &idev);
	if (r) {
		goto fail;
	}
	dev->idev = idev;

	headset_publish_sysfs(dev);

	/* mark this input device as wakeup capable */
	device_init_wakeup(&dev->idev->dev, 1);

	/* Check initial headset state */
	schedule_work(&dev->headset_detect_work);

	return r;

fail:
	if (dev->idev) {
		input_unregister_device(dev->idev);
		input_free_device(dev->idev);
	}

	free_irq(OMAP_GPIO_IRQ(dev->gpio_hs_plug_in), NULL);
	free_irq(OMAP_GPIO_IRQ(dev->gpio_mic_ans_int_pwm), NULL);

	omap_dm_timer_free(dev->pwm);

	gpio_free(dev->gpio_hs_plug_in);
	gpio_free(dev->gpio_hs_amp_sd_n);
	gpio_free(dev->gpio_mic_ans_int_pwm);

	return r;
}

static int __init headset_probe_forced(struct platform_device *pdev)
{
	int r = 0;
	struct headset_device *dev = &sirloin_headset_device;
	struct input_dev *idev = NULL;

	switch (forced_device) {
		case NONE:
			dev->headset_inserted = 0;
			dev->mic_present = 0;
			break;
		case HEADSET:
			dev->headset_inserted = 1;
			dev->mic_present = 0;
			break;
		case HEADSET_MIC:
			dev->headset_inserted = 1;
			dev->mic_present = 1;
			break;
		default:
			printk(KERN_ERR "headset: Cannot force device %d\n", forced_device);
			headset_probe(pdev);
	}

	dev->pdev = pdev;
	platform_set_drvdata(pdev, dev);

	r = headset_register_input_device(dev, &idev);
	if (r) {
		return r;
	}
	dev->idev = idev;

	headset_publish_sysfs(dev);

	return 0;
}

static int __init _headset_probe(struct platform_device *pdev)
{
	int r = 0;

	if (forced_device != -1) {
		r = headset_probe_forced(pdev);
	} else {
		r = headset_probe(pdev);
	}

	if (r == 0) {
		twl4030_register_codec_event_callback(headset_codec_event, platform_get_drvdata(pdev));
	}

	return r;
}


static int headset_remove(struct platform_device *pdev)
{
	struct headset_device *dev = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_headset_detect);
	device_remove_file(&pdev->dev, &dev_attr_hsmic_detect);
	device_remove_file(&pdev->dev, &dev_attr_adc);
	device_remove_file(&pdev->dev, &dev_attr_pol_ctl);

	free_irq(OMAP_GPIO_IRQ(dev->gpio_hs_plug_in), NULL);
	free_irq(OMAP_GPIO_IRQ(dev->gpio_mic_ans_int_pwm), NULL);

	cancel_work_sync(&dev->headset_detect_work);

	gpio_free(dev->gpio_hs_plug_in);
	gpio_free(dev->gpio_hs_amp_sd_n);
	gpio_free(dev->gpio_mic_ans_int_pwm);

	return 0;
}


static int 
headset_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct headset_device *dev = platform_get_drvdata(pdev);

	/* Do nothing if we're faking state */
	if (forced_device != -1) {
		return 0;
	}

	if( device_may_wakeup(&dev->idev->dev)) {
		// disable irq, but enable irq wake
		disable_irq    (gpio_to_irq(dev->gpio_hs_plug_in));
		enable_irq_wake(gpio_to_irq(dev->gpio_hs_plug_in));

		disable_irq    (gpio_to_irq(dev->gpio_mic_ans_int_pwm));
		enable_irq_wake(gpio_to_irq(dev->gpio_mic_ans_int_pwm));
	}

	return 0;
}

static int headset_resume(struct platform_device *pdev)
{
	struct headset_device *dev = platform_get_drvdata(pdev);

	/* Do nothing if we're faking state */
	if (forced_device != -1) {
		return 0;
	}

	if( device_may_wakeup(&dev->idev->dev)) {
		// disable wake and reenable irq
		disable_irq_wake(gpio_to_irq(dev->gpio_hs_plug_in));
		enable_irq (gpio_to_irq(dev->gpio_hs_plug_in));

		disable_irq_wake(gpio_to_irq(dev->gpio_mic_ans_int_pwm));
		enable_irq (gpio_to_irq(dev->gpio_mic_ans_int_pwm));
	}

	/* check headset state on wake */
	schedule_work(&dev->headset_detect_work);
	
	return 0;
}

static int __init force_headset_device(char *device)
{
	if (strcmp("headset", device) == 0) {
		forced_device = HEADSET;
	} else if (strcmp("headset-mic", device) == 0) {
		forced_device = HEADSET_MIC;
	} else if (strcmp("none", device) == 0) {
		forced_device = NONE;
	}
	return 0;
}

__setup("force_headset_device=", force_headset_device);


static struct platform_driver headset_driver = {
	.driver = {
		.name = "headset-detect",
	},
	.probe = _headset_probe,
	.remove = __devexit_p(headset_remove),
	.suspend = headset_suspend,
	.resume  = headset_resume,
};

static int __init headset_init(void)
{
	return platform_driver_register(&headset_driver);
}

static void __exit headset_exit(void)
{
	platform_driver_unregister(&headset_driver);
}

module_init(headset_init);
module_exit(headset_exit);

MODULE_DESCRIPTION("headset");
MODULE_LICENSE("GPL");
