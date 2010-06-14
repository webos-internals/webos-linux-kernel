/*
 * drivers/input/accelerometer/kxte9_accelerometer_dbg.c
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
 */

#include <linux/i2c.h>
#include <linux/i2c_kxte9_accelerometer.h>
#include <linux/platform_device.h>

#define KXTE9_DBG_ATTR_8_RO(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO, \
		}, \
		.show = _dbg_attribute_8_show, \
	}, \
	.index = _idx,

#define KXTE9_DBG_ATTR_8_RW(_name, _idx) \
	.attr = { \
		.attr = { \
			.name = #_name, \
			.owner = THIS_MODULE, \
			.mode = S_IRUGO | S_IWUGO, \
		}, \
		.show = kxte9_dbg_attribute_8_show, \
		.store = kxte9_dbg_attribute_8_store, \
	}, \
	.index = _idx,

struct kxte9_dbg_attribute {
	struct device_attribute	attr;
	u8			index;
};

static ssize_t
kxte9_dbg_attribute_8_show(struct device *dev, struct device_attribute *attr,
				char *begin)
{
	u8 value;
	char *end = begin;
	struct i2c_client *i2c;
	struct kxte9_dbg_attribute *reg;
	int ret;
	
	i2c = (struct i2c_client *)dev_get_drvdata(dev);
	reg = (struct kxte9_dbg_attribute *)attr;
	
	ret = kxte9_i2c_read_u8(i2c, reg->index, &value);
	end += sprintf(end, "0x%02X(%u)\n", value, value);
	
	return (end - begin);
}

ssize_t
kxte9_dbg_attribute_8_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	u8 value;
	unsigned int inval;
	struct i2c_client *i2c;
	struct kxte9_dbg_attribute *reg;
	
	i2c = (struct i2c_client *)dev_get_drvdata(dev);
	reg = (struct kxte9_dbg_attribute *)attr;
	
	if (1 != sscanf(buf, "0x%x", &inval) && 1 != sscanf(buf, "%u", &inval))
		goto exit;

	value = inval;
	inval = kxte9_i2c_write_u8(i2c, reg->index, value);

exit:	
	return (count);
}


static struct kxte9_dbg_attribute kxte9_dbg_attrs[] = {
	{ KXTE9_DBG_ATTR_8_RW(st_resp, 0x0C) },
	{ KXTE9_DBG_ATTR_8_RW(b2s_thresh, 0x0D) },
	{ KXTE9_DBG_ATTR_8_RW(wuf_thresh, 0x0E) },
	{ KXTE9_DBG_ATTR_8_RW(who_am_i, 0x0F) },
	
	{ KXTE9_DBG_ATTR_8_RW(tilt_pos_cur, 0x10) },
	{ KXTE9_DBG_ATTR_8_RW(tilt_pos_pre, 0x11) },
	{ KXTE9_DBG_ATTR_8_RW(xout, 0x12) },
	{ KXTE9_DBG_ATTR_8_RW(yout, 0x13) },
	{ KXTE9_DBG_ATTR_8_RW(zout, 0x14) },	
	{ KXTE9_DBG_ATTR_8_RW(int_src_reg1, 0x16) },	
	{ KXTE9_DBG_ATTR_8_RW(int_src_reg2, 0x17) },		
	{ KXTE9_DBG_ATTR_8_RW(status_reg, 0x18) },	
	{ KXTE9_DBG_ATTR_8_RW(int_rel, 0x1A) },
	{ KXTE9_DBG_ATTR_8_RW(ctrl_reg1, 0x1B) },
	{ KXTE9_DBG_ATTR_8_RW(ctrl_reg2, 0x1C) },	
	{ KXTE9_DBG_ATTR_8_RW(ctrl_reg3, 0x1D) },
	{ KXTE9_DBG_ATTR_8_RW(int_ctrl_reg1, 0x1E) },
	{ KXTE9_DBG_ATTR_8_RW(int_ctrl_reg2, 0x1F) },
	
	{ KXTE9_DBG_ATTR_8_RW(tilt_timer, 0x28) },
	{ KXTE9_DBG_ATTR_8_RW(wuf_timer, 0x29) },
	{ KXTE9_DBG_ATTR_8_RW(b2s_timer, 0x2A) },
};

static int
kxte9_dbg_driver_probe(struct platform_device *pdev)
{
	int i;
	int rc;
	struct device_attribute *attr;
	
	for (i = 0, rc = 0; ARRAY_SIZE(kxte9_dbg_attrs) > i; ++i) {
		attr = &kxte9_dbg_attrs[i].attr;
		if ((rc = device_create_file(&pdev->dev, attr))) {
			while (i--) {
				attr = &kxte9_dbg_attrs[i].attr;
				device_remove_file(&pdev->dev, attr);
			}	
			break;
		}
	}

	return (rc);
}

static void
kxte9_dbg_device_release(struct device *dev)
{
}

static struct platform_device kxte9_dbg_device = {
	.name = KXTE9_I2C_DBG_DEVICE,
	.id = -1,
	.dev = {
		.release = __devexit_p(kxte9_dbg_device_release),
	},
};

int
kxte9_dbg_register_i2c_client(struct i2c_client *i2c)
{
	int rc;
	
	if (platform_get_drvdata(&kxte9_dbg_device)) {
		rc = -EBUSY;
		goto busy;
	}
	
	platform_set_drvdata(&kxte9_dbg_device, i2c);
	
	if ((rc = platform_device_register(&kxte9_dbg_device)))
		platform_set_drvdata(&kxte9_dbg_device, NULL);
		
busy:
	return (rc);
}

EXPORT_SYMBOL(kxte9_dbg_register_i2c_client);

void
kxte9_dbg_unregister_i2c_client(struct i2c_client *i2c)
{
	if (platform_get_drvdata(&kxte9_dbg_device) == i2c) {
		platform_device_unregister(&kxte9_dbg_device);
		platform_set_drvdata(&kxte9_dbg_device, NULL);
	}
}

EXPORT_SYMBOL(kxte9_dbg_unregister_i2c_client);

static struct platform_driver kxte9_dbg_driver = {
	.driver = {
		.name = KXTE9_I2C_DBG_DRIVER,
	},
	.probe = kxte9_dbg_driver_probe,
};

static int __init
kxte9_dbg_module_init(void)
{
 	return (platform_driver_register(&kxte9_dbg_driver));
}

static void __exit
kxte9_dbg_module_exit(void)
{
	platform_driver_unregister(&kxte9_dbg_driver);
}

module_init(kxte9_dbg_module_init);
module_exit(kxte9_dbg_module_exit);

MODULE_DESCRIPTION("KIONIX TE9 Accelerometer debug driver");
MODULE_LICENSE("GPL");
