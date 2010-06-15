/*
 *  linux/drivers/thermistor/thermistor-tmp105.c - TI Thermistor driver for the TMP105
 *  
 *  Copyright (C) 2009 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Chris Anderson (chris.anderson@palm.com)
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c_tmp105_thermistor.h>

#define TMP105_DEBUG_ENABLE 0
#define TMP105_DEBUG(x...) do { if(TMP105_DEBUG_ENABLE) { printk(KERN_INFO x);} } while (0)

#define sysnode_show(name, reg) \
    static ssize_t show_##name(struct device *dev, struct device_attribute *attr, char *buf) { \
        u8 data = 0; \
        ssize_t count = 0; \
        struct i2c_client *client = (struct i2c_client *)dev->platform_data; \
        data = tmp105_read_reg(client, reg);\
        count = snprintf(buf, PAGE_SIZE, "%i\n", data);\
        return count;\
    }
      
#define sysnode_set(name, reg) \
    static ssize_t set_##name(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) { \
        int rc = 0; \
        u8 data = simple_strtol(buf, NULL, 10); \
        struct i2c_client *client = (struct i2c_client *)dev->platform_data; \
        rc = tmp105_write_reg(client, reg, data); \
        return count; \
    }

static int tmp105_i2c_write (struct i2c_client *client, u8 val);
static int tmp105_write_reg (struct i2c_client *client, u8 reg, u8 val);
static int tmp105_i2c_read (struct i2c_client *client, u8 *data);
static int tmp105_read_reg (struct i2c_client *client, u8 reg);
static int tmp105_platform_probe(struct platform_device *pdev);
static int tmp105_platform_remove (struct platform_device *pdev);
static int tmp105_i2c_probe (struct i2c_client *client);
static int tmp105_i2c_remove (struct i2c_client *client);

static struct platform_device tmp105_device = {
	.name	= "tmp105",
 	.id	= -1,
        .dev	= {
            .platform_data = NULL
 	},
};

static struct i2c_driver tmp105_i2c_driver = {
    .driver = {
        .name = TMP105_I2C_DRIVER,
    },
    .probe      = tmp105_i2c_probe,
    .remove     = tmp105_i2c_remove,
    .suspend    = NULL,
    .resume     = NULL,
};

static struct platform_driver tmp105_platform_driver = {
    .probe = tmp105_platform_probe,
    .remove = tmp105_platform_remove,
    .driver = {
        .name = "tmp105",
        .owner = THIS_MODULE,
    },
};


/*
 * I2C read/write functions
 */
static int tmp105_i2c_write (struct i2c_client *client, u8 val)
{
    int ret;
    u8 buf[1] = {val};
    struct i2c_msg msg[1];
    TMP105_DEBUG("tmp105: %s write value is %i %02X\n", __FUNCTION__, buf[0], buf[0]);
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = buf;
    ret = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(msg[0]));
    return ret;
}

static int tmp105_i2c_read (struct i2c_client *client, u8 *data)
{
    int ret;
    struct i2c_msg msg[1];
    TMP105_DEBUG("tmp105: %s reading\n", __FUNCTION__);
    msg[0].addr = client->addr;
    msg[0].len = 1;
    msg[0].flags = I2C_M_RD;
    msg[0].buf = data;

    ret = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(msg[0]));
    return ret;
}

static int tmp105_write_reg (struct i2c_client *client, u8 reg, u8 val)
{
    int ret;
    u8 buf[2] = {reg, val};
    struct i2c_msg msg[1];
    TMP105_DEBUG("tmp105: %s writing 0x%02X to register %i\n", __FUNCTION__, val, reg); 
    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = buf;

    ret = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(msg[0]));
    return ret;
}

static int tmp105_read_reg(struct i2c_client *client, u8 reg) {
    u8 data = 0;
    int rc = 0;
    rc = tmp105_i2c_write(client, reg);
    rc = tmp105_i2c_read(client, &data);
    return data;
}

static void tmp105_set_os_bit(struct i2c_client *client)
{
    int rc = 0;
    u8 data = 0;
    data = tmp105_read_reg(client, TMP105_REG_CONFIG);
    data |= TMP105_CFG_OS;
    TMP105_DEBUG("tmp105: %s setting one-shot bit, config is now 0x%02X\n", __FUNCTION__, data);
    rc = tmp105_write_reg(client, TMP105_REG_CONFIG, data);
}

/*
 * Functions for handling sysfs nodes in /sys/devices/platform/tmp105/
 */ 
static ssize_t show_celsius(struct device *dev, struct device_attribute *attr, char *buf) 
{ 
    u8 data = 0; 
    ssize_t count = 0; 
    struct i2c_client *client = (struct i2c_client *)dev->platform_data;
    tmp105_set_os_bit(client);
    data = tmp105_read_reg(client, TMP105_REG_TEMP);
    count = snprintf(buf, PAGE_SIZE, "%i\n", data);
    return count;
}

static ssize_t show_raw_temp(struct device *dev, struct device_attribute *attr, char *buf) 
{ 
    u8 data = 0; 
    ssize_t count = 0; 
    struct i2c_client *client = (struct i2c_client *)dev->platform_data;
    tmp105_set_os_bit(client);
    data = tmp105_read_reg(client, TMP105_REG_TEMP);
    count = snprintf(buf, PAGE_SIZE, "0x%02X\n", data);
    return count;
}

sysnode_show (config, TMP105_REG_CONFIG);
sysnode_show (tlow, TMP105_REG_TLOW);
sysnode_show (thigh, TMP105_REG_THIGH);

sysnode_set (config, TMP105_REG_CONFIG);
sysnode_set (tlow, TMP105_REG_TLOW);
sysnode_set (thigh, TMP105_REG_THIGH);

static DEVICE_ATTR(raw_temp, S_IRUGO, show_raw_temp, NULL);
static DEVICE_ATTR(celsius, S_IRUGO, show_celsius, NULL);
static DEVICE_ATTR(config, S_IRUGO | S_IWUGO, show_config, set_config);
static DEVICE_ATTR(tlow, S_IRUGO | S_IWUGO, show_tlow, set_tlow);
static DEVICE_ATTR(thigh, S_IRUGO | S_IWUGO, show_thigh, set_thigh);

/*
 * Driver initialization, probing, etc functions
 */
static int tmp105_platform_probe(struct platform_device *pdev)
{
    int rc;
    printk(KERN_INFO "tmp105: platform initialized\n");
    rc = device_create_file(&(pdev->dev), &dev_attr_raw_temp);
    rc = device_create_file(&(pdev->dev), &dev_attr_celsius);
    rc = device_create_file(&(pdev->dev), &dev_attr_config);
    rc = device_create_file(&(pdev->dev), &dev_attr_tlow);
    rc = device_create_file(&(pdev->dev), &dev_attr_thigh);
    
    return 0;
}

static int tmp105_i2c_probe (struct i2c_client *client)
{
    int rc;
    TMP105_DEBUG("tmp105: probe called\n");

    // For easy i2c lookups elsewhere
    tmp105_device.dev.platform_data = client;

    rc = platform_device_register(&tmp105_device);
    rc = platform_driver_register(&tmp105_platform_driver);

    // Config is 0x00 at power on, enable shutdown mode
    tmp105_write_reg(client, TMP105_REG_CONFIG, TMP105_CFG_SD);
    
    return rc;
}

static int tmp105_i2c_remove (struct i2c_client *client)
{
    i2c_set_clientdata(client, NULL);
    return 0;
}

static int tmp105_platform_remove (struct platform_device *pdev)
{
    TMP105_DEBUG("tmp105: platform cleanup\n");
    device_remove_file(&(pdev->dev), &dev_attr_raw_temp);
    device_remove_file(&(pdev->dev), &dev_attr_celsius);
    device_remove_file(&(pdev->dev), &dev_attr_config);
    device_remove_file(&(pdev->dev), &dev_attr_tlow);
    device_remove_file(&(pdev->dev), &dev_attr_thigh);
    pdev->dev.driver_data = NULL;
    return 0;
}

static int __init tmp105_module_init (void) {
    int ret;
    ret = i2c_add_driver(&tmp105_i2c_driver);
    TMP105_DEBUG("tmp105: init finished: %i\n", ret);
    return ret;
}

static void __exit tmp105_module_exit (void) {
    i2c_del_driver(&tmp105_i2c_driver);
}

// Generic driver setup
module_init(tmp105_module_init);
module_exit(tmp105_module_exit);

MODULE_DESCRIPTION("TI TMP105 Thermistor Driver");
MODULE_AUTHOR("Chris Anderson <chris.anderson@palm.com>");
MODULE_LICENSE("GPL");
