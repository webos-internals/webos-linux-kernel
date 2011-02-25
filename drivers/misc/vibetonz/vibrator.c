 /*
 * linux/drivers/misc/vibrator.c
 *
 * Vibrator
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
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/vibrator.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

#include <asm/arch/gpio.h>
#include <asm/arch/twl4030.h>

#include "vibrator_plat.h"
#include "vtmdrv.h"
#include "vibrator.h"

#define DPRINTK(args...)
//#define DPRINTK(args...) printk(args)

struct vibe_state {
	/* Direction of rotation:
	 *    >0 : Forward
	 *    =0 : Off
	 *    <0 : Backward
	 */
	int direction;

	/* Duty cycle in %: 0 <= duty_cycle <= 100
	 */
	unsigned int duty_cycle;

	struct mutex mutex;
};

static ssize_t direction_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vibe_state *state;

	DPRINTK("In %s...\n", __func__);

	if (!buf)
		return 0;

	state = (struct vibe_state *) dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", state->direction);
}

int _direction_store(struct vibe_state *state, int value)
{
	int rc;
	mutex_lock(&state->mutex);

	rc = plat_vibrator_set_direction(value);
	if (rc != 0){
		mutex_unlock(&state->mutex);
		return -EIO;
	}

	/* Check if we have an OFF->ON or ON->OFF transition. Enable/disable
	 * vibrator accordingly.
	 */
	if (!state->direction != !value) {
		plat_vibrator_enable(!!value);
	}
	state->direction = value;

	mutex_unlock(&state->mutex);
	return 0;
}

static ssize_t direction_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct vibe_state *state;
	int rc, value;

	if (!buf || !count)
		return 0;

	state = (struct vibe_state*)dev_get_drvdata(dev);

	sscanf(buf, "%d", &value);

	DPRINTK("In %s: value: %d...\n", __func__, value);
	rc = _direction_store(state,value);
	if(rc < 0)
		return rc;

	return count;
}

static ssize_t duty_cycle_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vibe_state *state;

	DPRINTK("In %s...\n", __func__);

	if (!buf)
		return 0;

	state = (struct vibe_state *) dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", state->duty_cycle);
}


int _duty_cycle_store(struct vibe_state * state, int value)
{
	int rc;
	/* 0 <= duty_cycle <= 100 */
	if (value > 100)
		value = 100;

	mutex_lock(&state->mutex);

	rc = plat_vibrator_set_duty_cycle(value);
	if (rc != 0){
		mutex_unlock(&state->mutex);
		return -EIO;
	}

	state->duty_cycle = value;

	mutex_unlock(&state->mutex);
	return 0;
}

static ssize_t duty_cycle_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int rc;
	unsigned int value;
	struct vibe_state *state;

	DPRINTK("In %s...\n", __func__);

	if (!buf || !count)
		return 0;

	state = (struct vibe_state *) dev_get_drvdata(dev);

	sscanf(buf, "%u", &value);

	rc = _duty_cycle_store(state,value);
	if(rc < 0)
		return rc;

	return count;
}

static DEVICE_ATTR(direction, S_IRUGO | S_IWUSR, direction_show, direction_store);
static DEVICE_ATTR(duty_cycle, S_IRUGO | S_IWUSR, duty_cycle_show, duty_cycle_store);

static int __init vibe_probe(struct platform_device *pdev)
{
	int rc;
	struct vibe_state *state;

	DPRINTK("In %s...\n", __func__);

	state = kzalloc(sizeof(struct vibe_state), GFP_KERNEL);
	if (!state)
		return (-ENOMEM);

	/* init phys. vibe device */
	plat_vibrator_init();

	/* Set default values: 0 = disabled, 50% duty cycle
	 */
	state->direction  = 0;
	state->duty_cycle = 50;

	/* Configure hardware.
	 */
	rc = plat_vibrator_set_direction(state->direction);
	if (rc)
		goto err0;

	rc = plat_vibrator_set_duty_cycle(state->duty_cycle);
	if (rc)
		goto err0;

	dev_set_drvdata(&pdev->dev, state);

	mutex_init(&state->mutex);

	/* create sysfs attribute */
	if ((rc = device_create_file(&pdev->dev, &dev_attr_direction)))
		goto err0;

	if ((rc = device_create_file(&pdev->dev, &dev_attr_duty_cycle)))
		goto err1;

	register_vibetonz(state);
	printk(KERN_INFO "Vibrator driver initialized...\n");
	return 0;

err1:
	device_remove_file(&pdev->dev, &dev_attr_direction);
err0:
	kfree(state);
	printk(KERN_ERR "ERROR: Vibrator driver intitialization failed.\n");
	return rc;
}

static int __devexit vibe_remove(struct platform_device *pdev)
{
	struct vibe_state *state;

	state = (struct vibe_state*)pdev->dev.driver_data;

	DPRINTK("In %s...\n", __func__);

	cleanup_vibetonz();
	mutex_lock(&state->mutex);

	/* de-init phys. vibe device */
	plat_vibrator_deinit();

	device_remove_file(&pdev->dev, &dev_attr_direction);
	device_remove_file(&pdev->dev, &dev_attr_duty_cycle);

	mutex_unlock(&state->mutex);

	kfree(dev_get_drvdata(&pdev->dev));
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int vibe_suspend(struct platform_device *pdev, pm_message_t pm_state)
{
	struct vibe_state *state;
	int rc = 0;

	state = (struct vibe_state*)pdev->dev.driver_data;

	DPRINTK("*** in %s.\n", __func__);
	mutex_lock(&state->mutex);
	if(state->direction) {
		rc = -EBUSY;
	} else {
		plat_vibrator_deinit();
	}
	mutex_unlock(&state->mutex);
	return rc;
}

static int vibe_resume (struct platform_device *pdev)
{
	DPRINTK("*** in %s.\n", __func__);

	plat_vibrator_init();

	return 0;
}
#else
#define vibe_suspend  NULL
#define vibe_resume   NULL
#endif  /* CONFIG_PM */

struct platform_driver vibe_driver = {
    .probe        = vibe_probe,
    .remove       = __devexit_p(vibe_remove),
    .suspend 	  = vibe_suspend,
    .resume 	  = vibe_resume,
    .driver       = {
        .name     = VIBE_DRIVER,
    },
};

static int __init vibe_init(void)
{
	DPRINTK("In %s...\n", __func__);

	return platform_driver_register(&vibe_driver);
}

static void __exit vibe_exit(void)
{
	DPRINTK("In %s...\n", __func__);

	return platform_driver_unregister(&vibe_driver);
}



module_init(vibe_init);
module_exit(vibe_exit);

MODULE_DESCRIPTION("Vibrator Driver");
MODULE_LICENSE("GPL");
