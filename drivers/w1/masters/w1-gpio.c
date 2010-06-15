/*
 * w1-gpio - GPIO w1 bus master driver
 *
 * Copyright (C) 2007 Ville Syrjala <syrjala@sci.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/w1-gpio.h>

#include "../w1.h"
#include "../w1_int.h"


#include <asm/gpio.h>
#include <linux/delay.h>

static struct w1_bus_master gpio_w1_master;

static void w1_gpio_write_bit_dir(void *data, u8 bit)
{
	struct w1_gpio_platform_data *pdata = data;
	// printk(KERN_INFO "\n *** %s: bit: %d ***", __func__, bit);

	if (bit)
		gpio_direction_input(pdata->pin);
	else
		gpio_direction_output(pdata->pin, 0);
}

static void w1_gpio_write_bit_val(void *data, u8 bit)
{
	struct w1_gpio_platform_data *pdata = data;
	// printk(KERN_INFO "\n *** %s: 0x00 *** \n", __func__);

	gpio_set_value(pdata->pin, bit);
}

static u8 w1_gpio_read_bit(void *data)
{
	struct w1_gpio_platform_data *pdata = data;

	return gpio_get_value(pdata->pin);
	// printk(KERN_INFO "\n *** %s: value: %d *** \n", __func__, result);
}

static void w1_write_bit(void *data, int bit)
{
	u32 flags;

	if (bit) {
		local_irq_save (flags);
		gpio_w1_master.write_bit(data, 0);
		udelay(6);
		gpio_w1_master.write_bit(data, 1);
		local_irq_restore (flags);
		udelay(64);
	} else {
		local_irq_save (flags);
		gpio_w1_master.write_bit(data, 0);
		udelay(60);
		gpio_w1_master.write_bit(data, 1);
		local_irq_restore (flags);
		udelay(10);
	}
}

static u8 w1_read_bit (void *data)
{
	int result;
	u32 flags;
	struct w1_gpio_platform_data *pdata = data;

	local_irq_save (flags);

	gpio_w1_master.write_bit(data, 0);
	udelay(pdata->w1_read_bit_drive_low_time);
	
	gpio_w1_master.write_bit(data, 1);
	udelay(pdata->w1_read_bit_release_time);

	result = gpio_w1_master.read_bit(data);
	local_irq_restore (flags);
	udelay(pdata->w1_read_bit_delay_time);
	return result & 0x1;
}

static u8 w1_gpio_touch_bit(void *data, int bit)
{
	if(bit) {
		return w1_read_bit (data);
	}
	else {
		w1_write_bit(data, 0);
		return 0;
	}
}

static void w1_gpio_write_byte(void *data, u8 val)
{
	int i;

	for(i=0; i < 8; i++)
		w1_gpio_touch_bit(data, (val >> i) & 0x1);
}

static u8 w1_gpio_read_byte(void *data)
{
	u8 res = 0;
	int i;

	for(i=0; i < 8; i++)
		res |= (w1_gpio_touch_bit(data, 1) << i);

	return res;
}


static int __init w1_gpio_probe(struct platform_device *pdev)
{
	struct w1_gpio_platform_data *pdata = pdev->dev.platform_data;
	int err;

	if (!pdata)
		return -ENXIO;

	err = gpio_request(pdata->pin, "w1");
	if (err) {
		printk(KERN_INFO "\n *** %s: 0x01 Err: %d *** \n", __func__, err);
		goto error;
	}

	gpio_w1_master.data = pdata;

	if (pdata->is_open_drain) {
		gpio_direction_output(pdata->pin, 1);
		gpio_w1_master.write_bit = w1_gpio_write_bit_val;
	} else {
		gpio_direction_input(pdata->pin);
		gpio_w1_master.write_bit = w1_gpio_write_bit_dir;
	}

	gpio_w1_master.write_byte = w1_gpio_write_byte;
	gpio_w1_master.read_byte = w1_gpio_read_byte;
	gpio_w1_master.read_bit = w1_gpio_read_bit;

	err = w1_add_master_device(&gpio_w1_master);
	if (err) {
		printk(KERN_INFO "\n *** %s: 0x02 *** \n", __func__);
		goto free_gpio;
	}

	platform_set_drvdata(pdev, &gpio_w1_master);

	return 0;

free_gpio:
	gpio_free(pdata->pin);
error:
	return err;
}

static int __exit w1_gpio_remove(struct platform_device *pdev)
{
	struct w1_bus_master *master = platform_get_drvdata(pdev);
	struct w1_gpio_platform_data *pdata = pdev->dev.platform_data;

	// printk(KERN_INFO "\n *** %s: *** \n", __func__);
	w1_remove_master_device(master);
	gpio_free(pdata->pin);
	kfree(master);

	return 0;
}

static struct platform_driver w1_gpio_driver = {
	.driver = {
		.name	= ONEWIRE_GPIO_DRIVER,
		.owner	= THIS_MODULE,
	},
	.remove	= __exit_p(w1_gpio_remove),
};

static int __init w1_gpio_init(void)
{
	// printk(KERN_INFO "\n *** %s: 0x00 *** \n", __func__);
	return platform_driver_probe(&w1_gpio_driver, w1_gpio_probe);
}

static void __exit w1_gpio_exit(void)
{
	// printk(KERN_INFO "\n *** %s: 0x00 *** \n", __func__);
	platform_driver_unregister(&w1_gpio_driver);
}

module_init(w1_gpio_init);
module_exit(w1_gpio_exit);

MODULE_DESCRIPTION("GPIO w1 bus master driver");
MODULE_AUTHOR("Ville Syrjala <syrjala@sci.fi>");
MODULE_LICENSE("GPL");

