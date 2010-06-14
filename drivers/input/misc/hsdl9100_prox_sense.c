/*
 * linux/drivers/misc/hsdl9100_prox_sense.c
 *
 * Driver for the HSDL9100 Proximity Sensor.
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/hsdl9100_proximity_sensor.h>

#include <asm/arch/gpio.h>

#undef PROX_SENSE_DEBUG
//#define PROX_SENSE_DEBUG

#ifdef PROX_SENSE_DEBUG
#define DPRINTK(args...)	printk(args)
#else
#define DPRINTK(args...)
#endif

typedef struct {
	struct mutex lock;

	spinlock_t   timer_lock;

	int last_scan_state;    /* to remember state during suspend. */
	int scan_state;         /* Current state of the sensor */
#define SCAN_STATE_DISABLED   0 /* Sensor is disabled */
#define SCAN_STATE_IDLE       1 /* Sensor is enabled but idle */
#define SCAN_STATE_SCANNING   2 /* Sensor is enabled and scanning */

	struct hrtimer scan_timer;
	int scan_off_time_no_det; /* time between scans in ms while no object
				     has yet been detected */
	int scan_off_time_det;    /* time between scans in ms when an object
				     has been detected */
	int scan_on_time;         /* length of scan pulse in ms */
	int detect_gpio;
	int last_detected;
	/* LEDON PWM configuration context. */
	void *ledon_ctx;

	struct input_dev *inputdevice;
} prox_sense_state;

static inline int idle_interval_for_state(prox_sense_state *state)
{
	/* Depending on the detected state we need to change the idle interval
	 * to be short or long. We have a shorter interval for the time when we
	 * have not yet detected an object and a longer interval when we
	 * detected an object.
	 */
	return state->last_detected ?
			state->scan_off_time_det : state->scan_off_time_no_det;
}

/*
 * prox_sense_enable/disable() are called with state->lock mutex held.
 */
static void prox_sense_enable(prox_sense_state *state)
{
	DPRINTK(KERN_INFO "%s.\n", __func__);

	if (state->scan_state != SCAN_STATE_DISABLED)
		return;

	/* Turn on VCC. */
	hsdl9100_vcc_enable();

	/* configure pwm */
	hsdl9100_ledon_pwm_configure(state->ledon_ctx);

	/* Schedule a reading. First reading is in 100ms to allow capacitor in
	 * detection circuit (Schmitt Inverter) to charge after power on.
	 */
	hrtimer_start(&state->scan_timer,
		      ktime_set(0, 100 * 1000000), HRTIMER_MODE_REL);
	state->scan_state = SCAN_STATE_IDLE;
}

static void prox_sense_disable(prox_sense_state *state)
{
	unsigned long flags;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	if (state->scan_state == SCAN_STATE_DISABLED)
		return;

	spin_lock_irqsave(&state->timer_lock, flags);

	hrtimer_cancel(&state->scan_timer);

	if (SCAN_STATE_SCANNING == state->scan_state) {
		/* If we are currently in the scanning phase, we also need to
		 * stop the LEDON PWM.
		 */
		hsdl9100_ledon_pwm_stop(state->ledon_ctx);
	}
	state->scan_state = SCAN_STATE_DISABLED;

	spin_unlock_irqrestore(&state->timer_lock, flags);

	hsdl9100_vcc_disable();
}

/******************************************************************************
 *
 * SYSFS entries
 *
 ******************************************************************************/

static ssize_t enable_detection_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	prox_sense_state *state;

	if (!buf)
		return 0;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	DPRINTK(KERN_INFO "%s.\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%s\n", 
			state->scan_state == SCAN_STATE_DISABLED ? "0" : "1");
}

static ssize_t enable_detection_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	prox_sense_state *state;

	if (!buf || !count)
		return 0;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	DPRINTK(KERN_INFO "%s.\n", __func__);

	mutex_lock(&state->lock);

	/* enable or disable detection based on input buffer[0] */
	if ('0' == buf[0]) {
		prox_sense_disable(state);
	} else {
		prox_sense_enable(state);
	}

	mutex_unlock(&state->lock);

	return count;

}

static ssize_t ledon_duty_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			hsdl9100_ledon_pwm_get_duty_cycle(state->ledon_ctx));
}

static ssize_t ledon_duty_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;
	int en = 0;
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf || !count)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	sscanf(buf, "%d", &value);

	/* duty cycle always <= 100 */
	if ((value < 10) || (value > 90)) {
		printk(KERN_ERR "%s: valid range: 10%% <= duty cycle <= 90%%.\n",
		       HSDL9100_DRIVER);
		goto out;
	}

	mutex_lock(&state->lock);

	/* save detection state before disable */
	if (state->scan_state != SCAN_STATE_DISABLED) {
		en = 1;
		prox_sense_disable(state);
	}

	hsdl9100_ledon_pwm_set_duty_cycle(state->ledon_ctx, value);

	/* base decision to enable detection on saved detection state */
	if (en) {
		prox_sense_enable(state);
	}

out:
	mutex_unlock(&state->lock);

	return count;
}

static ssize_t ledon_frequency_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			hsdl9100_ledon_pwm_get_hz(state->ledon_ctx));
}

static ssize_t ledon_frequency_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	int value;
	int en = 0;
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf || !count)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	sscanf(buf, "%d", &value);

	mutex_lock(&state->lock);

	/* 1 kHz <= ledon pulse frequency <= 40 kHz? */
	if (value < 1000 || value > 40000) {
		printk(KERN_ERR
		       "%s: valid range: 1 kHz <= ledon pulse freq. <= 40 kHz.\n",
		       HSDL9100_DRIVER);
		goto out;
	}

	/* save detection state before disable */
	if (state->scan_state != SCAN_STATE_DISABLED) {
		en = 1;
		prox_sense_disable(state);
	}

	hsdl9100_ledon_pwm_set_hz(state->ledon_ctx, value);

	/* base decision to enable detection on saved detection state */
	if (en) {
		prox_sense_enable(state);
	}

out:
	mutex_unlock(&state->lock);

	return count;
}

static ssize_t scan_off_time_no_det_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n", state->scan_off_time_no_det);
}

static ssize_t scan_off_time_no_det_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int value;
	unsigned long flags;
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf || !count)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	sscanf(buf, "%d", &value);

	mutex_lock(&state->lock);

	/* Set the new value if it is valid (>= 50ms).
	 * Otherwise set the default value.
	 */
	if (value >= 50) {
		state->scan_off_time_no_det = value;
	} else {
		printk("ERROR: Value must be >= 50ms.\n");
		return 0;
	}

	/* If we are in state SCAN_STATE_IDLE, re-start the timer to reflect
	 * the new interval timeout.
	 */
	spin_lock_irqsave(&state->timer_lock, flags);
	if (state->scan_state == SCAN_STATE_IDLE) {
		hrtimer_start(&state->scan_timer,
			      ktime_set(0, idle_interval_for_state(state) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&state->timer_lock, flags);

	mutex_unlock(&state->lock);

	return count;
}

static ssize_t scan_off_time_det_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n", state->scan_off_time_det);
}

static ssize_t scan_off_time_det_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int value;
	unsigned long flags;
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf || !count)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	sscanf(buf, "%d", &value);

	mutex_lock(&state->lock);

	/* Set the new value if it is valid (>= 100ms).
	 * Otherwise set the default value.
	 */
	if (value >= 50) {
		state->scan_off_time_det = value;
	} else {
		printk("ERROR: Value must be >= 50ms.\n");
		return 0;
	}

	/* If we are in state SCAN_STATE_IDLE, re-start the timer to reflect
	 * the new interval timeout.
	 */
	spin_lock_irqsave(&state->timer_lock, flags);
	if (state->scan_state == SCAN_STATE_IDLE) {
		hrtimer_start(&state->scan_timer,
			      ktime_set(0, idle_interval_for_state(state) * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&state->timer_lock, flags);

	mutex_unlock(&state->lock);

	return count;
}

static ssize_t scan_on_time_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	return snprintf(buf, PAGE_SIZE, "%d\n", state->scan_on_time);
}

static ssize_t scan_on_time_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int value;
	unsigned long flags;
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(dev);

	if (!buf || !count)
		return 0;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	sscanf(buf, "%d", &value);

	if ((value > 0) && (value <= 10)) {
		state->scan_on_time = value;
	} else {
		printk(KERN_WARNING "%s: Invalid scan interval, must be 0 < n <= 10...\n",
				HSDL9100_DRIVER);
		return 0;
	}

	mutex_lock(&state->lock);

	/* If we are in SCAN_STATE_SCANNING state, re-start the timer to
	 * reflect the new scan interval timeout.
	 */
	spin_lock_irqsave(&state->timer_lock, flags);
	if (state->scan_state == SCAN_STATE_SCANNING) {
		hrtimer_start(&state->scan_timer,
			      ktime_set(0, state->scan_on_time * 1000000),
			      HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&state->timer_lock, flags);

	mutex_unlock(&state->lock);

	return count;
}

DEVICE_ATTR(enable_detection, S_IRUGO | S_IWUSR,
	    enable_detection_show, enable_detection_store);

DEVICE_ATTR(ledon_duty, S_IRUGO | S_IWUSR,
	    ledon_duty_show, ledon_duty_store);

DEVICE_ATTR(ledon_frequency, S_IRUGO | S_IWUSR,
	    ledon_frequency_show, ledon_frequency_store);

DEVICE_ATTR(scan_off_time_no_det, S_IRUGO | S_IWUSR,
	    scan_off_time_no_det_show, scan_off_time_no_det_store);

DEVICE_ATTR(scan_off_time_det, S_IRUGO | S_IWUSR,
	    scan_off_time_det_show, scan_off_time_det_store);

DEVICE_ATTR(scan_on_time, S_IRUGO | S_IWUSR,
	    scan_on_time_show, scan_on_time_store);

/******************************************************************************
 *
 * TIMER handler
 *
 ******************************************************************************/

static enum hrtimer_restart prox_timer_cb(struct hrtimer *h)
{
	int detected;

	prox_sense_state *state = container_of(h, prox_sense_state, scan_timer);

	spin_lock(&state->timer_lock);

	switch (state->scan_state) {
	case SCAN_STATE_DISABLED:
		/* This can happen if we get preempted just after the timer
		 * fired but we had not yet aquired the lock.
		 * We got disabled -> ignore this event.
		 */
		break;

	case SCAN_STATE_IDLE:
		/* Turn on the LEDON PWM and set the timer to the scan duration
		 * time.
		 */
		hsdl9100_ledon_pwm_start(state->ledon_ctx);
		
		/* Forward the timer by scan_on_time milliseconds.
		 * Note:
		 *   state->scan_timer.base->get_time() is ktime_t of now.
		 */
		state->scan_timer.expires =
			ktime_add_ns(state->scan_timer.base->get_time(),
						state->scan_on_time * 1000000);

		state->scan_state = SCAN_STATE_SCANNING;
		break;

	case SCAN_STATE_SCANNING:
		/* Turn off the LEDON PWM.
		 */
		hsdl9100_ledon_pwm_stop(state->ledon_ctx);

		/* Take a reading from the proximity GPIO and see if we
		 * detected something.
		 */
		detected = gpio_get_value(state->detect_gpio);
		DPRINTK(KERN_INFO "Prox scan detected: %d\n", detected);

		/* Compare to last reading. We only send an input event if the
		 * reading changed.
		 */
		if (state->last_detected != detected) {
			DPRINTK(KERN_INFO "Sending input event: %d\n", detected);
			input_event(state->inputdevice, EV_MSC, MSC_PULSELED, detected);
			state->last_detected = detected;
		}

		/* Schedule the next scan.
		 */
		state->scan_timer.expires =
			ktime_add_ns(state->scan_timer.base->get_time(),
				     idle_interval_for_state(state) * 1000000);

		state->scan_state = SCAN_STATE_IDLE;
		break;

	default:
		printk(KERN_ERR "%s: Invalid state %d in %s\n",
				HSDL9100_DRIVER, state->scan_state, __func__);
	}
	spin_unlock(&state->timer_lock);

	return HRTIMER_RESTART;
}

/******************************************************************************
 *
 * PROBE / REMOVE / SUSPEND / RESUME
 *
 ******************************************************************************/

static int __init prox_sense_probe(struct platform_device *pdev)
{
	int rc;
	prox_sense_state *state;
	struct hsdl9100_platform_data *pdata;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	pdata = (struct hsdl9100_platform_data *)pdev->dev.platform_data;

	state = kzalloc(sizeof(prox_sense_state), GFP_KERNEL);
	if (!state) {
		return -ENOMEM;
	}

	dev_set_drvdata(&pdev->dev, state);

	/* Initialize local state.
	 */
	mutex_init(&state->lock);

	spin_lock_init(&state->timer_lock);

	state->scan_state           = SCAN_STATE_DISABLED;
	state->last_scan_state      = SCAN_STATE_DISABLED;

	state->scan_off_time_no_det = pdata->scan_off_time_no_det;
	state->scan_off_time_det    = pdata->scan_off_time_det;
	state->scan_on_time         = pdata->scan_on_time;
	state->detect_gpio          = pdata->detect_gpio;
	state->last_detected        = 0;

	/* Allocate and configure input device.
	 */
	state->inputdevice = input_allocate_device();
	if (!state->inputdevice) {
		goto input_device_alloc_failed;
	}
	state->inputdevice->name       = HSDL9100_DEVICE;
	state->inputdevice->id.bustype = BUS_VIRTUAL;
	state->inputdevice->id.vendor  = 0x0;
	state->inputdevice->id.product = 0x0;
	state->inputdevice->id.version = 0x100;
	state->inputdevice->dev.parent = &pdev->dev;

	set_bit(EV_MSC, state->inputdevice->evbit);
	set_bit(MSC_PULSELED, state->inputdevice->mscbit);
	rc = input_register_device(state->inputdevice);
	if (rc < 0) {
		printk(KERN_ERR "%s: Error in registering input device...\n",
		       HSDL9100_DRIVER);
		goto input_device_register_failed;
	}

	dev_set_drvdata(&state->inputdevice->dev, state);

	/* Setup timer.
	 */
	hrtimer_init(&state->scan_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	state->scan_timer.function = prox_timer_cb;

	/* Setup GPIOs.
	 */
	rc = gpio_request(state->detect_gpio, "prox int");
	if (rc < 0) {
		printk(KERN_ERR "%s: Error in requesting gpio: %d...\n",
		       HSDL9100_DRIVER, state->detect_gpio);
		goto detect_gpio_request_failed;
	}
	gpio_direction_input(state->detect_gpio);

	/* Setup LEDON PWM.
	 */
	rc = hsdl9100_ledon_pwm_init(pdata, &state->ledon_ctx);
	if (rc != 0) {
		printk(KERN_ERR "%s: Error ininitializing PWM...\n",
		       HSDL9100_DRIVER);
		goto ledon_pwm_init_failed;
	}

	/* Create device sysfs attributes.
	 */
	rc = device_create_file(&state->inputdevice->dev,
				&dev_attr_enable_detection);
	if (rc != 0) {
		printk(KERN_ERR "%s: Error in creating device sysfs entries\n",
		       HSDL9100_DRIVER);
		goto attr_enable_detection_failed;
	}

	rc = device_create_file(&state->inputdevice->dev,
				&dev_attr_ledon_duty);
	if (rc != 0) {
		printk(KERN_ERR "%s: Error in creating device sysfs entries\n",
		       HSDL9100_DRIVER);
		goto attr_ledon_duty_failed;
	}

	rc = device_create_file(&state->inputdevice->dev,
				&dev_attr_ledon_frequency);
	if (rc != 0) {
		printk(KERN_ERR "%s: Error in creating device sysfs entries\n",
		       HSDL9100_DRIVER);
		goto attr_ledon_frequency_failed;
	}

	rc = device_create_file(&state->inputdevice->dev,
				&dev_attr_scan_off_time_no_det);
	if (rc != 0) {
		printk(KERN_ERR "%s: Error in creating device sysfs entries\n",
		       HSDL9100_DRIVER);
		goto attr_scan_off_time_no_det_failed;
	}

	rc = device_create_file(&state->inputdevice->dev,
				&dev_attr_scan_off_time_det);
	if (rc != 0) {
		printk(KERN_ERR "%s: Error in creating device sysfs entries\n",
		       HSDL9100_DRIVER);
		goto attr_scan_off_time_det_failed;
	}

	rc = device_create_file(&state->inputdevice->dev,
				&dev_attr_scan_on_time);
	if (rc != 0) {
		printk(KERN_ERR "%s: Error in creating device sysfs entries\n",
		       HSDL9100_DRIVER);
		goto attr_scan_on_time_failed;
	}

	printk(KERN_INFO "%s: Proximity Sensor driver initialized....\n",
			HSDL9100_DRIVER);

	return 0;

attr_scan_on_time_failed:
	device_remove_file(&state->inputdevice->dev,
			   &dev_attr_scan_off_time_det);
attr_scan_off_time_det_failed:
	device_remove_file(&state->inputdevice->dev,
			   &dev_attr_scan_off_time_no_det);
attr_scan_off_time_no_det_failed:
	device_remove_file(&state->inputdevice->dev,
			   &dev_attr_ledon_frequency);
attr_ledon_frequency_failed:
	device_remove_file(&state->inputdevice->dev,
			   &dev_attr_ledon_duty);
attr_ledon_duty_failed:
	device_remove_file(&state->inputdevice->dev,
			   &dev_attr_enable_detection);
attr_enable_detection_failed:
	hsdl9100_ledon_pwm_free(state->ledon_ctx);
ledon_pwm_init_failed:
	gpio_free(state->detect_gpio);
detect_gpio_request_failed:
	input_unregister_device(state->inputdevice);
input_device_register_failed:
	input_free_device(state->inputdevice);
input_device_alloc_failed:
	kfree(state);

	printk(KERN_INFO "%s: Failed to intialize Proximity Sensor driver\n",
	       HSDL9100_DRIVER);
	return -ENODEV;
}

static int __devexit prox_sense_remove(struct platform_device *pdev)
{
	prox_sense_state *state;

	state = (prox_sense_state *) dev_get_drvdata(&pdev->dev);

	mutex_lock(&state->lock);
	prox_sense_disable(state);
	mutex_unlock(&state->lock);

	device_remove_file(&state->inputdevice->dev, &dev_attr_scan_on_time);
	device_remove_file(&state->inputdevice->dev, &dev_attr_scan_off_time_no_det);
	device_remove_file(&state->inputdevice->dev, &dev_attr_scan_off_time_det);
	device_remove_file(&state->inputdevice->dev, &dev_attr_ledon_frequency);
	device_remove_file(&state->inputdevice->dev, &dev_attr_ledon_duty);
	device_remove_file(&state->inputdevice->dev, &dev_attr_enable_detection);

	hsdl9100_ledon_pwm_free(state->ledon_ctx);

	gpio_free(state->detect_gpio);

	input_unregister_device(state->inputdevice);
	input_free_device(state->inputdevice);

	kfree(dev_get_drvdata(&pdev->dev));
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int prox_sense_suspend(struct platform_device *pdev, pm_message_t pm_state)
{
	prox_sense_state *state;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	state = (prox_sense_state *) dev_get_drvdata(&pdev->dev);

	/* Remember state before suspend and disable detection.
	 */
	state->last_scan_state = state->scan_state;
	prox_sense_disable(state);
	return 0;
}

static int prox_sense_resume(struct platform_device *pdev)
{
	prox_sense_state *state;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	state = (prox_sense_state *) dev_get_drvdata(&pdev->dev);

	/* Check if detection was enabled before suspend and if so, re-enable
	 * it.
	 */
	if (state->last_scan_state != SCAN_STATE_DISABLED) {
		prox_sense_enable(state);
	}
	return 0;
}
#else
#define prox_sense_suspend  NULL
#define prox_sense_resume   NULL
#endif /* CONFIG_PM */

static struct platform_driver prox_sense_driver = {
	.probe   = prox_sense_probe,
	.remove  = prox_sense_remove,
	.suspend = prox_sense_suspend,
	.resume  = prox_sense_resume,
	.driver  = {
			.name = HSDL9100_DRIVER,
		   },
};

/******************************************************************************
 *
 * INIT / EXIT
 *
 ******************************************************************************/

static int __init prox_sense_init(void)
{
	int rc;

	rc = platform_driver_register(&prox_sense_driver);

	return rc ? -ENODEV : 0;
}

static void __exit prox_sense_exit(void)
{
	platform_driver_unregister(&prox_sense_driver);
}

module_init(prox_sense_init);
module_exit(prox_sense_exit);

MODULE_DESCRIPTION("HSDL9100 Proximity Sensor Driver");
MODULE_LICENSE("GPL");
