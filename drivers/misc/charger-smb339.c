/*
 *  linux/drivers/misc/charger-smb339.c - Linux driver for SMB339 Li+ Battery Charger
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
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/i2c_smb339_charger.h>

#define SMB339_DEBUG_ENABLE 1
#define SMB339_DEBUG(x...) do { if(SMB339_DEBUG_ENABLE) { printk(KERN_INFO x);} } while (0)

static struct mutex lock;
static int smb339_i2c_write_reg (struct i2c_client *client, u8 addr, u8 val);
static int smb339_i2c_read_reg (struct i2c_client *client, u8 addr, u8 *data);
static int smb339_platform_probe(struct platform_device *pdev);
static int smb339_platform_remove (struct platform_device *pdev);
static int smb339_i2c_probe (struct i2c_client *client);
static int smb339_i2c_remove (struct i2c_client *client);
static int smb339_ioctl(struct inode *ip, struct file *fp, unsigned int cmd, unsigned long arg);

struct smb339_charge_setting {
	int current_ma;
	smb339_current_values reg;
} charge_settings[] = {
	{ 0, SMB339_CURRENT_DISABLE },
	{ 550, SMB339_CURRENT_550mA },
	{ 650, SMB339_CURRENT_650mA },
	{ 750, SMB339_CURRENT_750mA },
	{ 850, SMB339_CURRENT_850mA },
	{ 950, SMB339_CURRENT_950mA },
	{ 1050, SMB339_CURRENT_1050mA },
	{ 1150, SMB339_CURRENT_1150mA },
	{ 1250, SMB339_CURRENT_1250mA },
};

static struct i2c_device_id smb339_i2c_id[] = {
	{ SMB339_I2C_DEVICE, 0 },
	{ }

};

// i2c driver information
static struct i2c_driver smb339_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name = SMB339_I2C_DRIVER,
	},
	.id_table   = smb339_i2c_id,
	.probe      = smb339_i2c_probe,
	.remove     = smb339_i2c_remove,
	.suspend    = NULL,
	.resume     = NULL,
};

// Global state information for the /dev entry and passing the i2c client to
// ioctls
struct smb339_device_state {
	struct file_operations fops;
	struct miscdevice mdev;
	struct i2c_client *i2c_dev;
};

// What file operations are permitted with /dev/charger
struct file_operations smb339_fops = {
	.ioctl   = smb339_ioctl,
};

/*
 * I2C read/write functions
 */
static int smb339_i2c_write_reg (struct i2c_client *client, u8 addr, u8 val)
{
	int ret;
	u8 buf[2] = {addr, val};
	struct i2c_msg msg[1];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	ret = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(msg[0]));
	return ret;
}

static int smb339_i2c_read_reg (struct i2c_client *client, u8 addr, u8 *data)
{
	int ret;
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data;

	ret = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(msg[0]));
	return ret;
}

static int smb339_set_reg(struct i2c_client *client, u8 smb339_current_value)
{
	int rc;
	u8 curr = 0;
	u8 reg2 = SMB339_FLOAT_VOLTAGE_420;
	u8 reg3 = SMB339_AUTO_RCHRG_DISABLE | SMB339_DISABLE_GLITCH_FILTER | SMB339_CRNT_NO_END_CHRG;
	u8 reg4 = SMB339_SAFETY_TIMER_DISABLE;
	u8 reg5 = 0;
	// enable volatile writes
	u8 reg31 = SMB339_VOLATILE_WRITE; 

	if (smb339_current_value >= SMB339_CURRENT_END) {
		return -EINVAL;
	}

	// If it's not 550 or higher we're disabling charging
	if (smb339_current_value == SMB339_CURRENT_DISABLE) {
		reg31 |= SMB339_DISABLE_CHARGING;
		mutex_lock(&lock);
		rc = smb339_i2c_write_reg(client, SMB339_REG_COMMAND, reg31);
		mutex_unlock(&lock);
		printk("smb339: disabling charging due to user request\n");
	} else {
		// Re-enable usb mode for 550, otherwise set
		if (smb339_current_value == 0) {
			reg5 |= SMB339_PIN_CONTROL_ENABLE;
		} else {
			reg31 |= SMB339_AC_MODE; // Set AC mode
		}

		// Some of these could likely be set once on driver init, but we want to
		// be sure since removing usb has shown in the past to reinitialize some
		// of the registers
		mutex_lock(&lock);
		rc = smb339_i2c_write_reg(client, SMB339_REG_COMMAND, reg31);
		// If we can't write the volatile bit there is no point in trying
		if (rc >= 0) {
		    smb339_i2c_write_reg(client, SMB339_REG_FLOAT, reg2);
		    smb339_i2c_write_reg(client, SMB339_REG_CONTROL_A, reg3);
		    smb339_i2c_write_reg(client, SMB339_REG_CONTROL_B, reg4);
		    smb339_i2c_write_reg(client, SMB339_REG_PIN_CONTROL, reg5);
		    smb339_i2c_read_reg(client, SMB339_REG_INPUT, &curr);
		    printk("smb339: setting input current to %02X\n", smb339_current_value);
		    curr = (curr & 0x1F) | (smb339_current_value << 5);
		    smb339_i2c_write_reg(client, SMB339_REG_INPUT, curr);
		} else {
		    printk("smb339: failed to write 0x%02X to 0x31\n", reg31);
		}
		mutex_unlock(&lock);
	}
	return 0;
}

static int smb339_set_current(struct i2c_client *client, int current_ma)
{
	struct smb339_charge_setting *target = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(charge_settings); i++) {
		if (charge_settings[i].current_ma > current_ma) {
			break;
		}
		target = &charge_settings[i];
	}

	if (target) {
		smb339_set_reg(client, target->reg);
		return 0;
	}
	return -1;
}

/* 
 * Sysfs functions
 */

static ssize_t sysfs_dump_registers(struct device *dev, struct device_attribute *attr, char *buf) {
	u8 data = 0;
	int i, rc, registers[] = {0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x30, 0x31, 0x34, 0x35, 0x36, 0x37, 0x3B};
	ssize_t count = 0;
	struct i2c_client *client = to_i2c_client(dev);
	for (i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
		rc = smb339_i2c_read_reg(client, registers[i], &data);
		if (count < PAGE_SIZE)
			count += snprintf(buf + count, PAGE_SIZE, "%02X: 0x%02X\n", registers[i], data);
	}
	return count;
}

static ssize_t sysfs_show_current(struct device *dev, struct device_attribute *attr, char *buf) {
	int rc;
	u8 data = 0;
	ssize_t count = 0;
	struct i2c_client *client = to_i2c_client(dev);
	rc = smb339_i2c_read_reg(client, SMB339_REG_INPUT, &data);
	if (!rc) {
		count = snprintf(buf, PAGE_SIZE, "Error");
	} else {
		data = (data >> 5);
		if (data < SMB339_CURRENT_END) {
			// +1 to offset the DISABLE index that now exists
			count = snprintf(buf, PAGE_SIZE, "%i\n",
					charge_settings[data + 1].current_ma);
		}
	}
	return count;
}

static ssize_t sysfs_set_current(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	int rc = 0;
	int current_ma = simple_strtol(buf, NULL, 10);
	struct i2c_client *client = to_i2c_client(dev);

	rc = smb339_set_current(client, current_ma);
	if (rc) {
		printk("smb339: Invalid current input.\n");
	}

	return count;
}

static DEVICE_ATTR(max_current_mA, S_IRUGO | S_IWUGO, sysfs_show_current, sysfs_set_current);
static DEVICE_ATTR(dump_reg, S_IRUGO, sysfs_dump_registers, NULL);

/* 
 * Ioctl handling
 */

static int smb339_ioctl(struct inode *ip, struct file *fp, unsigned int cmd, unsigned long arg) {
	int ret = 0;
	struct smb339_device_state *state = container_of(fp->f_op, struct smb339_device_state, fops);
	struct i2c_client *client = (struct i2c_client *)state->i2c_dev;

	SMB339_DEBUG("smb339: %s, cmd %i arg %ld\n", __FUNCTION__, cmd, arg);  
	switch (cmd) {
		case SMB339_SET_MAX_CURRENT:
			ret = smb339_set_reg(client, arg);
			if (ret) {
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

/*
 * Driver initialization, probing, etc functions
 */
static int smb339_platform_probe(struct platform_device *pdev)
{
	int rc;
	printk(KERN_INFO "smb339: platform initialized\n");
	rc = device_create_file(&(pdev->dev), &dev_attr_max_current_mA);
	rc = device_create_file(&(pdev->dev), &dev_attr_dump_reg);
	SMB339_DEBUG("smb339: created node: %i\n", rc);

	return 0;
}

static int smb339_i2c_probe (struct i2c_client *client)
{
	int rc;
	struct smb339_device_state *state;
	SMB339_DEBUG("smb339: probe called\n");
	state = kzalloc(sizeof(struct smb339_device_state), GFP_KERNEL);
	if (!state)
		return (-ENOMEM);
	state->i2c_dev = client;
	SMB339_DEBUG("smb339: %s state i2c addr %p\n", __FUNCTION__, state->i2c_dev);
	i2c_set_clientdata(client, state);

	// init the mutex
	mutex_init(&lock);

	rc = device_create_file(&(client->dev), &dev_attr_max_current_mA);
	rc = device_create_file(&(client->dev), &dev_attr_dump_reg);

	// initialize file operations and create /dev/charger
	memcpy(&state->fops, &smb339_fops, sizeof(struct file_operations));
	state->mdev.minor = MISC_DYNAMIC_MINOR;
	state->mdev.name = "charger";
	state->mdev.fops = &state->fops;
	rc = misc_register(&state->mdev);
	SMB339_DEBUG("smb339: registering misc dev: %i\n", rc);

	return rc;
}

static int smb339_i2c_remove (struct i2c_client *client)
{
	i2c_set_clientdata(client, NULL);
	return 0;
}

static int smb339_platform_remove (struct platform_device *pdev)
{
	SMB339_DEBUG("smb339: platform cleanup\n");
	pdev->dev.driver_data = NULL;
	device_remove_file(&(pdev->dev), &dev_attr_max_current_mA);
	device_remove_file(&(pdev->dev), &dev_attr_dump_reg);
	return 0;
}

static int __init smb339_module_init (void) {
	int ret;
	ret = i2c_add_driver(&smb339_i2c_driver);
	SMB339_DEBUG("smb339: init finished: %i\n", ret);
	return ret;
}

static void __exit smb339_module_exit (void) {
	i2c_del_driver(&smb339_i2c_driver);
}

// Generic driver setup
module_init(smb339_module_init);
module_exit(smb339_module_exit);

MODULE_DESCRIPTION("SMB339 Li+ Battery Charger Driver");
MODULE_AUTHOR("Chris Anderson <chris.anderson@palm.com>");
MODULE_LICENSE("GPL");
