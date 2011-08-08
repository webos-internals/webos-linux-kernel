/* arch/arm/mach-msm/pwm.c
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
#include <mach/gpio.h>
#include <mach/msm_iomap.h>

#define MODULE_NAME "msm_pwm"

// debug switch
#define MSM_PWM_DEBUG_ENABLE	1


/**
 * GP clock. 
 */

#define GP_M_VALUE   (1)
#define GP_N_VALUE   (1000)

#define GP_MD_REG    (MSM_CLK_CTL_BASE + 0x0058)
#define GP_NS_REG    (MSM_CLK_CTL_BASE + 0x005C)
#define GP_ROOT_ENA        (1 << 11)
#define GP_CLK_INV         (1 << 10)   
#define GP_CLK_BRANCH_ENA  (1 << 9)
#define GP_MNCNTR_EN       (1 << 8)
#define GP_MNCNTR_MODE_6   (1 << 6)
#define GP_MNCNTR_MODE_5   (1 << 5)

/**
 * GP_MN clock.
 */

#define PRPH_WEB_NS_REG     (MSM_CLK_CTL_BASE + 0x0080)
#define PRPH_WEB_ROOT_ENA   (1 << 11)
#define PRPH_CLK_BRANCH_ENA     (1 << 9)
#define PRPH_CLK_INV        (1 << 10) 

// GP_MN_CLK_MDIV bits[8:0] = M value
// GP_MN_CLK_NDIV bits[12:0] = 1's complement of N-M value
// GP_MN_CLK_DUTY bits [12:0] = D value which must be between M and (N-M)
#define GP_MN_CLK_MDIV  (MSM_WEB_TCXO4_BASE + 0x4C)
#define GP_MN_CLK_NDIV  (MSM_WEB_TCXO4_BASE + 0x50)
#define GP_MN_CLK_DUTY  (MSM_WEB_TCXO4_BASE + 0x54)
#define WEB_TCXO4_TEST  (MSM_WEB_TCXO4_BASE + 0x58)

#define GP_M_VALUE_20KHZ   2
#define GP_N_VALUE_20KHZ   0x1DE
//#define GP_N_1COMP_VALUE_20KHZ   0x1E21

int msm_pwm_gp_set(u16 duty_percent)
{
 	int d = 0;
	int not_2_d = 0;
	int not_n_minus_m = 0;
	
	if(duty_percent > 100)
		return -EINVAL;

	// make sure the general clock is enabled before we access any of its regs
	if( !(readl(GP_NS_REG) & (GP_ROOT_ENA | GP_CLK_BRANCH_ENA | GP_MNCNTR_EN  | GP_MNCNTR_MODE_6  )))
	{
		printk(KERN_ERR "%s: general clock is not enabled!", __FUNCTION__);
		return -EIO;
	}

	/* 
	 *	7x30
	 *  calcuate duty cycle reg value based on given percent 
	 * ^(2*D) with D = N * percent
	 */
		
	not_n_minus_m = (~(GP_N_VALUE - GP_M_VALUE));
	d = (GP_N_VALUE * duty_percent) / 100;
	not_2_d = ~(2 * d);

	//Set M, N and duty cycle
	writel((GP_M_VALUE << 16)|(not_2_d&0x0000FFFF), GP_MD_REG);
	writel(readl(GP_NS_REG) | (not_n_minus_m) << 16, GP_NS_REG);

	return 0;
}

int msm_pwm_gp_mn_set(u16 duty_percent)
{
	u16 duty;
	u16 N, M;

	if(duty_percent > 100)
		return -EINVAL;

	// make sure the web clock is enabled before we access any of its regs
	if( !(readl(PRPH_WEB_NS_REG) & (PRPH_WEB_ROOT_ENA | PRPH_CLK_BRANCH_ENA))){
		printk(KERN_ERR "%s: web clock is not enabled!", __FUNCTION__);
		return -EIO;
	}

	M = GP_M_VALUE_20KHZ;
	N = GP_N_VALUE_20KHZ;

	/* calcuate duty cycle reg value based on given percent */
	duty = M + ((N - 2 * M) * (100 - duty_percent)) / 100;
	writel(M, GP_MN_CLK_MDIV);
	writel(~(N - M) & 0x1FFF, GP_MN_CLK_NDIV);
	writel(duty, GP_MN_CLK_DUTY);

	return 0;
}

int msm_pwm_gp_enable(void)
{
	writel( (readl(GP_NS_REG) | (GP_ROOT_ENA | GP_CLK_BRANCH_ENA | GP_MNCNTR_EN  | GP_MNCNTR_MODE_6 )), GP_NS_REG );
	return 0;
}

int msm_pwm_gp_disable(void)
{
	return 0;
}

int msm_pwm_gp_mn_enable(void)
{
	writel(readl(PRPH_WEB_NS_REG) | (PRPH_WEB_ROOT_ENA | PRPH_CLK_BRANCH_ENA), PRPH_WEB_NS_REG);
	return 0;
}

int msm_pwm_gp_mn_disable(void)
{
	return 0;
}

#if MSM_PWM_DEBUG_ENABLE

static ssize_t 
gp_duty_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long duty = 0;
	char *endptr;
	
	duty = simple_strtoul(buf, &endptr, 10);

	msm_pwm_gp_set(duty);

	return len;
}

static ssize_t 
gp_mn_duty_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long duty = 0;
	char *endptr;
	
	duty = simple_strtoul(buf, &endptr, 10);

	msm_pwm_gp_mn_set(duty);

	return len;
}

static ssize_t 
gp_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long duty = 0;
	char *endptr;
	
	duty = simple_strtoul(buf, &endptr, 10);

	if (0 == duty)
		msm_pwm_gp_disable();
	else
		msm_pwm_gp_enable();

	return len;
}

static ssize_t 
gp_mn_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long duty = 0;
	char *endptr;
	
	duty = simple_strtoul(buf, &endptr, 10);

	if (0 == duty)
		msm_pwm_gp_mn_disable();
	else
		msm_pwm_gp_mn_enable();

	return len;
}

static DEVICE_ATTR(gp_duty, S_IWUGO, NULL, gp_duty_store);
static DEVICE_ATTR(gp_mn_duty, S_IWUGO, NULL, gp_mn_duty_store);
static DEVICE_ATTR(gp_enable, S_IWUGO, NULL, gp_enable_store);
static DEVICE_ATTR(gp_mn_enable, S_IWUGO, NULL, gp_mn_enable_store);

static struct device_attribute *attrs[] = {
	&dev_attr_gp_duty,
	&dev_attr_gp_mn_duty,
	&dev_attr_gp_enable,
	&dev_attr_gp_mn_enable,
};

#endif  // MSM_PWM_DEBUG_ENABLE


static int __init msm_pwm_probe(struct platform_device *pdev)
{
#if MSM_PWM_DEBUG_ENABLE
	int ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(attrs); i++) {
		if ((ret = device_create_file(&(pdev->dev), attrs[i])))
		{
			printk(KERN_ERR "Failed to create sysfs device for msm_pwm\n");
			return -EINVAL;
		}
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

