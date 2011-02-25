/*
 * drivers/leds/leds-tps6105x.c
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

#include <linux/i2c_tps6105x_led.h>

#include <asm/arch/gpio.h>

#define REGISTER0			0x00
#define REGISTER1			0x01

#define	MODE_CONTROL_MASK		(0x3 << 6)
#define	MODE_CONTROL(val)		((0x3 & (val)) << 6)
#define MODE_CONTROL_SHUTDOWN		0x0
#define MODE_CONTROL_TORCH		0x1
#define MODE_CONTROL_TORCH_AND_FLASH	0x2

#define	TORCH_CURRENT_MASK		(0x7 << 0)
#define	TORCH_CURRENT(val)		((0x7 & (val)) << 0)
#define TORCH_CURRENT_0_MA		0x0
#define TORCH_CURRENT_50_MA		0x1
#define TORCH_CURRENT_75_MA		0x2
#define TORCH_CURRENT_100_MA		0x3
#define TORCH_CURRENT_150_MA		0x4
#define TORCH_CURRENT_200_MA		0x5
#define TORCH_CURRENT_250_400_MA	0x6
#define TORCH_CURRENT_250_500_MA	0x7

#define	FLASH_CURRENT_MASK		(0x7 << 0)
#define	FLASH_CURRENT(val)		((0x7 & (val)) << 0)
#define FLASH_CURRENT_150_MA		0x0
#define FLASH_CURRENT_200_MA		0x1
#define FLASH_CURRENT_300_MA		0x2
#define FLASH_CURRENT_400_MA		0x3
#define FLASH_CURRENT_500_MA		0x4
#define FLASH_CURRENT_700_MA		0x5
#define FLASH_CURRENT_900_MA		0x6
#define FLASH_CURRENT_1200_MA		0x7

#define SPEW(level, args...) \
	if (tps6105x_spew_level >= level) \
		printk(KERN_DEBUG "TPS6105X: " args)

#define TPS6105X_ATTR(_name) \
	__ATTR(_name, S_IRUGO|S_IWUGO, tps6105x_attribute_show, \
		tps6105x_attribute_store)

struct tps6105x_client_data {
	struct i2c_client	*i2c;
	int			avin;
	int			(*enable_avin)(int);
};

struct tps6105x_attribute_value {
	u8		val;
	const char	*desc;
};

struct tps6105x_attribute {
	struct device_attribute		attr;
	u8				index;
	u8				mask;
	struct tps6105x_attribute_value	*values;
	int				count;
};

static int tps6105x_spew_level = 0;

static struct tps6105x_attribute_value tps6105x_mode_control_values[] = {
	{MODE_CONTROL(MODE_CONTROL_SHUTDOWN), "shutdown"},
	{MODE_CONTROL(MODE_CONTROL_TORCH), "torch"},
	{MODE_CONTROL(MODE_CONTROL_TORCH_AND_FLASH), "torch/flash"},
};

static struct tps6105x_attribute_value tps6105x_torch_current_values[] = {
	{TORCH_CURRENT(TORCH_CURRENT_0_MA), "0mA"},
	{TORCH_CURRENT(TORCH_CURRENT_50_MA), "50mA"},
	{TORCH_CURRENT(TORCH_CURRENT_75_MA), "75mA"},
	{TORCH_CURRENT(TORCH_CURRENT_100_MA), "100mA"},
	{TORCH_CURRENT(TORCH_CURRENT_150_MA), "150mA"},
	{TORCH_CURRENT(TORCH_CURRENT_200_MA), "200mA"},
	{TORCH_CURRENT(TORCH_CURRENT_250_400_MA), "250/400mA"},
	{TORCH_CURRENT(TORCH_CURRENT_250_500_MA), "250/500mA"},
};

static struct tps6105x_attribute_value tps6105x_flash_current_values[] = {
	{FLASH_CURRENT(FLASH_CURRENT_150_MA), "150mA"},
	{FLASH_CURRENT(FLASH_CURRENT_200_MA), "200mA"},
	{FLASH_CURRENT(FLASH_CURRENT_300_MA), "300mA"},
	{FLASH_CURRENT(FLASH_CURRENT_400_MA), "400mA"},
	{FLASH_CURRENT(FLASH_CURRENT_500_MA), "500mA"},
	{FLASH_CURRENT(FLASH_CURRENT_700_MA), "700mA"},
	{FLASH_CURRENT(FLASH_CURRENT_900_MA), "900mA"},
	{FLASH_CURRENT(FLASH_CURRENT_1200_MA), "1200mA"},
};

int
tps6105x_i2c_read_u8(struct i2c_client* i2c, u8 index, u8* value)
{
	int rc;
	struct i2c_msg msgs[2];

	msgs[0].addr = i2c->addr;
	msgs[0].len = 1;
	msgs[0].flags = 0;
	msgs[0].buf = &index;

	msgs[1].addr = i2c->addr;
	msgs[1].len = 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = value;

	rc = i2c_transfer(i2c->adapter, msgs, 2);

	return (min(rc, 0));
}
EXPORT_SYMBOL(tps6105x_i2c_read_u8);

int
tps6105x_i2c_write_u8(struct i2c_client* i2c, u8 index, u8 value)
{
	u8 in[2] = {index, value};
	int rc;
	struct i2c_msg msgs[1];

	msgs[0].addr = i2c->addr;
	msgs[0].len = 2;
	msgs[0].flags = 0;
	msgs[0].buf = in;

	rc = i2c_transfer(i2c->adapter, msgs, 1);

	return (min(rc, 0));
}
EXPORT_SYMBOL(tps6105x_i2c_write_u8);

#ifdef CONFIG_LEDS_TPS6105X_DBG
static ssize_t
tps6105x_show_spew_level(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	ssize_t count = 0;

	count = snprintf(buf, PAGE_SIZE, "%d\n", tps6105x_spew_level);

	return (count + 1);
}

static ssize_t
tps6105x_store_spew_level(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	char *endp;

	tps6105x_spew_level = simple_strtoul(buf, &endp, 10);

	return (count);
}

static struct device_attribute tps6105x_attr_spew_level =
	__ATTR(spew_level, S_IRUGO|S_IWUGO, tps6105x_show_spew_level,
		tps6105x_store_spew_level);

static inline int
tps6105x_create_spew_level(struct device *dev)
{
	return (device_create_file(dev, &tps6105x_attr_spew_level));
}

static inline void
tps6105x_remove_spew_level(struct device *dev)
{
	device_remove_file(dev, &tps6105x_attr_spew_level);
}
#else // !CONFIG_LEDS_TPS6105X_DBG
static inline int
tps6105x_create_spew_level(struct device *dev)
{
	return (0);
}

static inline void
tps6105x_remove_spew_level(struct device *dev)
{
}
#endif // CONFIG_LEDS_TPS6105X_DBG

static ssize_t
tps6105x_attribute_show(struct device *dev, struct device_attribute *_attr,
			char *buf)
{
	u8 val;
	int i;
	ssize_t rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct tps6105x_attribute *attr = (struct tps6105x_attribute *)_attr;

	SPEW(1, "--> %s: attr=%s\n", __func__, _attr->attr.name);

	if ((rc = tps6105x_i2c_read_u8(i2c, attr->index, &val)))
		goto exit;

	for (i = 0, rc = 0; i < attr->count; ++i) {
		rc += snprintf(buf + rc, PAGE_SIZE - rc, "%s%s",
				attr->values[i].desc,
				attr->values[i].val == (attr->mask & val)
					? "* " : " ");
	}

	rc += snprintf(buf + rc, PAGE_SIZE - rc, "\n");

exit:
	SPEW(1, "<-- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static ssize_t
tps6105x_attribute_store(struct device *dev, struct device_attribute *_attr,
				const char *buf, size_t count)
{
	u8 val;
	int i;
	ssize_t rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct tps6105x_attribute *attr = (struct tps6105x_attribute *)_attr;

	SPEW(1, "--> %s: attr=%s\n", __func__, _attr->attr.name);

	for (i = 0; i < attr->count; ++i) {
		if ((count == strlen(attr->values[i].desc))
			&& !strncmp(buf, attr->values[i].desc, count))
			break;
	}

	if (i == attr->count) {
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = tps6105x_i2c_read_u8(i2c, attr->index, &val)))
		goto exit;

	val &= ~attr->mask;
	val |= attr->values[i].val;

	if ((rc = tps6105x_i2c_write_u8(i2c, attr->index, val)))
		goto exit;

	rc = count;

exit:
	SPEW(1, "<-- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct tps6105x_attribute tps6105x_attrs[] = {
	{
	.attr = TPS6105X_ATTR(mode),
	.index = REGISTER0,
	.mask = MODE_CONTROL_MASK,
	.values = tps6105x_mode_control_values,
	.count = ARRAY_SIZE(tps6105x_mode_control_values),
	},

	{
	.attr = TPS6105X_ATTR(torch_current),
	.index = REGISTER0,
	.mask = TORCH_CURRENT_MASK,
	.values = tps6105x_torch_current_values,
	.count = ARRAY_SIZE(tps6105x_torch_current_values),
	},

	{
	.attr = TPS6105X_ATTR(flash_current),
	.index = REGISTER1,
	.mask = FLASH_CURRENT_MASK,
	.values = tps6105x_flash_current_values,
	.count = ARRAY_SIZE(tps6105x_flash_current_values),
	},
};

static ssize_t
tps6105x_attr_show_avin(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int rc;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct tps6105x_client_data *data = i2c_get_clientdata(i2c);

	rc = sprintf(buf, "%d\n", data->avin);

	return (rc);
}

static int
tps6105x_enable_avin(struct tps6105x_client_data *data, int enable)
{
	int rc = 0;

	if ((enable != data->avin) && data->enable_avin)
		rc = data->enable_avin((data->avin = enable));

	return (rc);
}

static ssize_t
tps6105x_attr_store_avin(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int rc;
	int avin;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct tps6105x_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "+++ %s: buf=%s\n", __func__, buf);

	if (!sscanf(buf, "%d", &avin)) {
		rc = -EINVAL;
		goto exit;
	}

	if ((rc = tps6105x_enable_avin(data, !!avin)))
		goto exit;

	rc = count;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static struct device_attribute tps6105x_attr_avin =
	__ATTR(avin, S_IRUGO|S_IWUGO, tps6105x_attr_show_avin,
		tps6105x_attr_store_avin);

static int
tps6105x_i2c_probe(struct i2c_client *i2c)
{
	int i;
	int rc;
	struct device_attribute *attr;
	struct tps6105x_client_data *data;
	struct tps6105x_platform_data *pdata;

	if (!(pdata = i2c->dev.platform_data)) {
		rc = -ENODEV;
		goto platform_data_missing;
	}

	if (!(data = kzalloc(sizeof(struct tps6105x_client_data), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto kzalloc_failed;
	}

	data->i2c = i2c;
	data->avin = !pdata->enable_avin;
	data->enable_avin = pdata->enable_avin;
	i2c_set_clientdata(i2c, data);

	for (i = 0; ARRAY_SIZE(tps6105x_attrs) > i; ++i) {
		attr = &tps6105x_attrs[i].attr;
		if ((rc = device_create_file(&i2c->dev, attr)))
			goto device_create_file_failed;
	}

	if ((rc = device_create_file(&i2c->dev, &tps6105x_attr_avin)))
		goto device_create_avin_file_failed;

	if ((rc = tps6105x_create_spew_level(&i2c->dev)))
		goto tps6105x_create_spew_level_failed;

	if ((rc = tps6105x_dbg_register_i2c_client(i2c)))
		goto tps6105x_dbg_register_i2c_client_failed;

	printk(KERN_INFO "TI TPS6105x LED driver\n");

	return (0);

tps6105x_dbg_register_i2c_client_failed:
	tps6105x_remove_spew_level(&i2c->dev);
tps6105x_create_spew_level_failed:
	device_remove_file(&i2c->dev, &tps6105x_attr_avin);
device_create_avin_file_failed:
device_create_file_failed:
	while (i--) {
		attr = &tps6105x_attrs[i].attr;
		device_remove_file(&i2c->dev, attr);
	}

	kfree(data);
kzalloc_failed:
platform_data_missing:
	printk(KERN_ERR "%s: %s failed, rc=%d\n", TPS6105X_I2C_DRIVER,
		__func__, rc);

	return (rc);
}

static int
tps6105x_i2c_remove(struct i2c_client *i2c)
{
	int i;
	struct device_attribute *attr;
	struct tps6105x_client_data *data = i2c_get_clientdata(i2c);

	tps6105x_dbg_unregister_i2c_client(i2c);
	tps6105x_remove_spew_level(&i2c->dev);
	device_remove_file(&i2c->dev, &tps6105x_attr_avin);

	for (i = 0; ARRAY_SIZE(tps6105x_attrs) > i; ++i) {
		attr = &tps6105x_attrs[i].attr;
		device_remove_file(&i2c->dev, attr);
	}

	(void)tps6105x_enable_avin(data, 0);
	kfree(data);

	return (0);
}

#ifdef CONFIG_PM
static int
tps6105x_i2c_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	struct tps6105x_client_data *data = i2c_get_clientdata(i2c);

	SPEW(1, "--> %s: event=%d\n", __func__, mesg.event);

	if (PM_EVENT_SUSPEND == mesg.event)
		(void)tps6105x_enable_avin(data, 0);

	SPEW(1, "<-- %s\n", __func__);

	return (0);
}
#endif // CONFIG_PM

static struct i2c_driver tps6105x_i2c_driver = {
	.driver = {
		.name = TPS6105X_I2C_DRIVER,
	},
	.probe = tps6105x_i2c_probe,
	.remove = __devexit_p(tps6105x_i2c_remove),
#ifdef CONFIG_PM
	.suspend = tps6105x_i2c_suspend,
#endif // CONFIG_PM
};

static int __init
tps6105x_module_init(void)
{
	return (i2c_add_driver(&tps6105x_i2c_driver));
}

static void __exit
tps6105x_module_exit(void)
{
	i2c_del_driver(&tps6105x_i2c_driver);
}

module_init(tps6105x_module_init);
module_exit(tps6105x_module_exit);

MODULE_DESCRIPTION("TI TPS6105x family LED driver");
MODULE_LICENSE("GPL");
