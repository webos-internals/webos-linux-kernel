/* arch/arm/mach-msm/pwm.c
 *
 * MSM7x25 pwm support
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Kevin McCray <kevin.mccray@palm.com>
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
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/io.h>

#include <linux/msm_pwm.h>

#define MODULE_NAME "msm_pwm"

// debug switch
#define MSM_PWM_DEBUG_ENABLE	1


int msm_pwm_configure(u16 duty_percent)
{
	u16 duty;
	// make sure the web clock is enabled before we access any of its regs
	if( !(readl(PRPH_WEB_NS_REG) & (PRPH_WEB_ROOT_ENA | PRPH_CLK_BRANCH_ENA)))
	{
		printk(KERN_ERR "msm_pwm_configure: web clock is not enabled!");
		return -EIO;
	}

	if(duty_percent > 100)
		return -EINVAL;

	// calcuate duty cycle reg value based on given percent
	duty = ((GP_N_VALUE_20KHZ - GP_M_VALUE_20KHZ) * (duty_percent)) / 100;

	writel(GP_M_VALUE_20KHZ, GP_MN_CLK_MDIV);
	writel(GP_N_1COMP_VALUE_20KHZ, GP_MN_CLK_NDIV);
	writel(duty, GP_MN_CLK_DUTY);

	return 0;
}
EXPORT_SYMBOL(msm_pwm_configure);

int msm_pwm_enable(void)
{
	// enable web clock
	writel(readl(PRPH_WEB_NS_REG) | (PRPH_WEB_ROOT_ENA | PRPH_CLK_BRANCH_ENA), PRPH_WEB_NS_REG);

	return 0;
}
EXPORT_SYMBOL(msm_pwm_enable);

int msm_pwm_disable(void)
{
	// The peripheral web clock must not be disable on 7x27 devices because the modem is using
	// it on a hw io block.  It is only disabled when TCXO is shutoff

	return 0;
}
EXPORT_SYMBOL(msm_pwm_disable);

#if MSM_PWM_DEBUG_ENABLE

static ssize_t 
msm_pwm_configure_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long duty = 0;
	char *endptr;
	
	duty = simple_strtoul(buf, &endptr, 10);

	msm_pwm_configure(duty);

	return len;
}
	

static DEVICE_ATTR(configure, S_IWUGO, NULL, msm_pwm_configure_store);


static ssize_t 
msm_pwm_disable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	msm_pwm_disable();

	return len;
}
	

static DEVICE_ATTR(disable, S_IWUGO, NULL, msm_pwm_disable_store);


static ssize_t 
msm_pwm_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	msm_pwm_enable();

	return len;
}	

static DEVICE_ATTR(enable, S_IWUGO, NULL, msm_pwm_enable_store);


#endif  // MSM_PWM_DEBUG_ENABLE


static int __init msm_pwm_probe(struct platform_device *pdev)
{
#if MSM_PWM_DEBUG_ENABLE
	int ret = 0;
#endif
	// make sure the GP_MN counter clock is off to start
	msm_pwm_disable();

#if MSM_PWM_DEBUG_ENABLE
	if ((ret = device_create_file(&(pdev->dev), &dev_attr_enable)))
	{
		printk(KERN_ERR "Failed to create sysfs device for msm_pwm\n");
		
		// TODO: clean up before we return
		return -EINVAL;
	}	

	if ((ret = device_create_file(&pdev->dev, &dev_attr_disable)))
	{
		printk(KERN_ERR "Failed to create sysfs device for msm_pwm\n");
		
		// TODO: clean up before we return
		return -EINVAL;
	}	

	if ((ret = device_create_file(&pdev->dev, &dev_attr_configure)))
	{
		printk(KERN_ERR "Failed to create sysfs device for msm_pwm\n");
		
		// TODO: clean up before we return
		return -EINVAL;
	}
#endif

	return 0;

}

static struct platform_driver msm_pwm_driver = {
	.probe = msm_pwm_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_pwm_init(void)
{
	printk(KERN_INFO "msm_pwm_init()\n");
	if(platform_driver_register(&msm_pwm_driver) != 0)
	{
		printk("PWM:Failed to init pwm driver\n");
	}

	return 0;
}

module_init(msm_pwm_init);

MODULE_DESCRIPTION("MSM PWM");
MODULE_AUTHOR("Kevin McCray <kevin.mccray@palm.com>");
MODULE_LICENSE("GPL");

