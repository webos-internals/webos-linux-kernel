/*
 * drivers/leds/leds-lp8501-dbg.c
 *
 * Copyright (C) Palm, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/i2c.h>
#include <linux/i2c_lp8501_led.h>
#include <linux/platform_device.h>

#define LP8501_DBG_ATTR_8_RO(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO, \
		}, \
		.show = _dbg_attribute_8_show, \
	}, \
	.index = _idx,

#define LP8501_DBG_ATTR_8_RW(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO | S_IWUGO, \
		}, \
		.show = lp8501_dbg_attribute_8_show, \
		.store = lp8501_dbg_attribute_8_store, \
	}, \
	.index = _idx,

struct lp8501_dbg_attribute {
	struct device_attribute	attr;
	u8			index;
};

static ssize_t
lp8501_dbg_attribute_8_show(struct device *dev, struct device_attribute *attr,
				char *begin)
{
	u8 value;
	char *end = begin;
	struct i2c_client *i2c;
	struct lp8501_dbg_attribute *reg;
	int ret;
	
	i2c = (struct i2c_client *)dev_get_drvdata(dev);
	reg = (struct lp8501_dbg_attribute *)attr;
	
	ret = lp8501_i2c_read_reg(i2c, reg->index, &value);
	end += sprintf(end, "0x%02X(%u)\n", value, value);
	
	return (end - begin);
}

ssize_t
lp8501_dbg_attribute_8_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	u8 value;
	unsigned int inval;
	struct i2c_client *i2c;
	struct lp8501_dbg_attribute *reg;
	
	i2c = (struct i2c_client *)dev_get_drvdata(dev);
	reg = (struct lp8501_dbg_attribute *)attr;
	
	if (1 != sscanf(buf, "0x%x", &inval) && 1 != sscanf(buf, "%u", &inval))
		goto exit;

	value = inval;
	inval = lp8501_i2c_write_reg(i2c, reg->index, value);

exit:	
	return (count);
}


static struct lp8501_dbg_attribute lp8501_dbg_attrs[] = {
	{ LP8501_DBG_ATTR_8_RW(engine_cntrl1, 0x00) },
	{ LP8501_DBG_ATTR_8_RW(engine_cntrl2, 0x01) },
	{ LP8501_DBG_ATTR_8_RW(group1_fading, 0x02) },
	{ LP8501_DBG_ATTR_8_RW(group2_fading, 0x03) },
	{ LP8501_DBG_ATTR_8_RW(group3_fading, 0x04) },
	{ LP8501_DBG_ATTR_8_RW(power_config, 0x05) },
	{ LP8501_DBG_ATTR_8_RW(d1_control, 0x06) },
	{ LP8501_DBG_ATTR_8_RW(d2_control, 0x07) },
	{ LP8501_DBG_ATTR_8_RW(d3_control, 0x08) },	
	{ LP8501_DBG_ATTR_8_RW(d4_control, 0x09) },	
	{ LP8501_DBG_ATTR_8_RW(d5_control, 0x0A) },		
	{ LP8501_DBG_ATTR_8_RW(d6_control, 0x0B) },	
	{ LP8501_DBG_ATTR_8_RW(d7_control, 0x0C) },
	{ LP8501_DBG_ATTR_8_RW(d8_control, 0x0D) },
	{ LP8501_DBG_ATTR_8_RW(d9_control, 0x0E) },	

	{ LP8501_DBG_ATTR_8_RW(gpo_control, 0x15) },
	{ LP8501_DBG_ATTR_8_RW(d1_pwm, 0x16) },
	{ LP8501_DBG_ATTR_8_RW(d2_pwm, 0x17) },
	{ LP8501_DBG_ATTR_8_RW(d3_pwm, 0x18) },
	{ LP8501_DBG_ATTR_8_RW(d4_pwm, 0x19) },
	{ LP8501_DBG_ATTR_8_RW(d5_pwm, 0x1A) },
	{ LP8501_DBG_ATTR_8_RW(d6_pwm, 0x1B) },
	{ LP8501_DBG_ATTR_8_RW(d7_pwm, 0x1C) },
	{ LP8501_DBG_ATTR_8_RW(d8_pwm, 0x1D) },
	{ LP8501_DBG_ATTR_8_RW(d9_pwm, 0x1E) },
	
	{ LP8501_DBG_ATTR_8_RW(gpo_pwm, 0x25) },
	{ LP8501_DBG_ATTR_8_RW(d1_current, 0x26) },
	{ LP8501_DBG_ATTR_8_RW(d2_current, 0x27) },
	{ LP8501_DBG_ATTR_8_RW(d3_current, 0x28) },
	{ LP8501_DBG_ATTR_8_RW(d4_current, 0x29) },
	{ LP8501_DBG_ATTR_8_RW(d5_current, 0x2A) },
	{ LP8501_DBG_ATTR_8_RW(d6_current, 0x2B) },
	{ LP8501_DBG_ATTR_8_RW(d7_current, 0x2C) },
	{ LP8501_DBG_ATTR_8_RW(d8_current, 0x2D) },
	{ LP8501_DBG_ATTR_8_RW(d9_current, 0x2E) },

	{ LP8501_DBG_ATTR_8_RW(config, 0x36) },
	{ LP8501_DBG_ATTR_8_RW(engine1_pc, 0x37) },
	{ LP8501_DBG_ATTR_8_RW(engine2_pc, 0x38) },	
	{ LP8501_DBG_ATTR_8_RW(engine3_pc, 0x39) },
	{ LP8501_DBG_ATTR_8_RW(status, 0x3A) },
	{ LP8501_DBG_ATTR_8_RW(gpo, 0x3B) },
	{ LP8501_DBG_ATTR_8_RW(reset, 0x3D) },
	{ LP8501_DBG_ATTR_8_RW(test_control, 0x41) },	
	{ LP8501_DBG_ATTR_8_RW(test_adc, 0x42) },	

	{ LP8501_DBG_ATTR_8_RW(group_fader1, 0x48) },	
	{ LP8501_DBG_ATTR_8_RW(group_fader2, 0x49) },	
	{ LP8501_DBG_ATTR_8_RW(group_fader3, 0x4A) },	
	{ LP8501_DBG_ATTR_8_RW(eng1_prog_start_addr, 0x4C) },	
	{ LP8501_DBG_ATTR_8_RW(eng2_prog_start_addr, 0x4D) },
	{ LP8501_DBG_ATTR_8_RW(eng3_prog_start_addr, 0x4E) },
	{ LP8501_DBG_ATTR_8_RW(prog_mem_page_sel, 0x4F) },	
};

static int
lp8501_dbg_driver_probe(struct platform_device *pdev)
{
	int i;
	int rc;
	struct device_attribute *attr;
	
	for (i = 0, rc = 0; ARRAY_SIZE(lp8501_dbg_attrs) > i; ++i) {
		attr = &lp8501_dbg_attrs[i].attr;
		if ((rc = device_create_file(&pdev->dev, attr))) {
			while (i--) {
				attr = &lp8501_dbg_attrs[i].attr;
				device_remove_file(&pdev->dev, attr);
			}	
			break;
		}
	}

	return (rc);
}

static void
lp8501_dbg_device_release(struct device *dev)
{
}

static struct platform_device lp8501_dbg_device = {
	.name = LP8501_I2C_DBG_DEVICE,
	.id = -1,
	.dev = {
		.release = __devexit_p(lp8501_dbg_device_release),
	},
};

int
lp8501_dbg_register_i2c_client(struct i2c_client *i2c)
{
	int rc;
	
	if (platform_get_drvdata(&lp8501_dbg_device)) {
		rc = -EBUSY;
		goto busy;
	}
	
	platform_set_drvdata(&lp8501_dbg_device, i2c);
	
	if ((rc = platform_device_register(&lp8501_dbg_device)))
		platform_set_drvdata(&lp8501_dbg_device, NULL);
		
busy:
	return (rc);
}

EXPORT_SYMBOL(lp8501_dbg_register_i2c_client);

void
lp8501_dbg_unregister_i2c_client(struct i2c_client *i2c)
{
	if (platform_get_drvdata(&lp8501_dbg_device) == i2c) {
		platform_device_unregister(&lp8501_dbg_device);
		platform_set_drvdata(&lp8501_dbg_device, NULL);
	}
}

EXPORT_SYMBOL(lp8501_dbg_unregister_i2c_client);

static struct platform_driver lp8501_dbg_driver = {
	.driver = {
		.name = LP8501_I2C_DBG_DRIVER,
	},
	.probe = lp8501_dbg_driver_probe,
};

static int __init
lp8501_dbg_module_init(void)
{
 	return (platform_driver_register(&lp8501_dbg_driver));
}

static void __exit
lp8501_dbg_module_exit(void)
{
	platform_driver_unregister(&lp8501_dbg_driver);
}

module_init(lp8501_dbg_module_init);
module_exit(lp8501_dbg_module_exit);

MODULE_DESCRIPTION("NS LP8501 LED debug driver");
MODULE_LICENSE("GPL");
