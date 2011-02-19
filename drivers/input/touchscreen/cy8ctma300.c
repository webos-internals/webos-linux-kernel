/*
 * drivers/input/touchscreen/cy8ctma300.c
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

#include <linux/cy8ctma300.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/ihex.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>

struct silicon_id {
	u8 id[2];
	u8 family;
	u8 rev;
};

static const int id_setup_1_bits = 616;
static const uint8_t id_setup_1_vec[] = {
	0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x0D, 0xEE, 0x21, 0xF7, 0xF0, 0x27, 0xDC, 0x40,
	0x9F, 0x70, 0x01, 0xFD, 0xEE, 0x01, 0xE7, 0xC1, 0xD7, 0x9F, 0x20, 0x7E,
	0x3F, 0x9D, 0x78, 0xF6, 0x21, 0xF7, 0xB8, 0x87, 0xDF, 0xC0, 0x1F, 0x71,
	0x00, 0x7D, 0xC0, 0x07, 0xF7, 0xB8, 0x07, 0xDE, 0x80, 0x7F, 0x7A, 0x80,
	0x7D, 0xEC, 0x01, 0xF7, 0x80, 0x4F, 0xDF, 0x00, 0x1F, 0x7C, 0xA0, 0x7D,
	0xF4, 0x61, 0xF7, 0xF8, 0x97
};

static const int id_setup_2_bits = 418;
static const uint8_t id_setup_2_vec[] = {
	0xDE, 0xE2, 0x1F, 0x7F, 0x02, 0x7D, 0xC4, 0x09, 0xF7, 0x00, 0x1F, 0x9F,
	0x07, 0x5E, 0x7C, 0x81, 0xF9, 0xF4, 0x01, 0xF7, 0xF0, 0x07, 0xDC, 0x40,
	0x1F, 0x70, 0x01, 0xFD, 0xEE, 0x01, 0xF7, 0xA0, 0x1F, 0xDE, 0xA0, 0x1F,
	0x7B, 0x00, 0x7D, 0xE0, 0x0D, 0xF7, 0xC0, 0x07, 0xDF, 0x28, 0x1F, 0x7D,
	0x18, 0x7D, 0xFE, 0x25, 0xC0
};

static const int wait_and_poll_end_bits = 40;
static const uint8_t wait_and_poll_end_vec[] = {
	0x00, 0x00, 0x00, 0x00, 0x00
};

static const int tsync_enable_bits = 110;
static const uint8_t tsync_enable_vec[] = {
	0xde, 0xe2, 0x1f, 0x7f, 0x02, 0x7d, 0xc4, 0x09, 0xf7, 0x00, 0x1f, 0xde,
	0xe0, 0x1c
};

static const int tsync_disable_bits = 110;
static const uint8_t tsync_disable_vec[] = {
	0xde, 0xe2, 0x1f, 0x71, 0x00, 0x7d, 0xfc, 0x01, 0xf7, 0x00, 0x1f, 0xde,
	0xe0, 0x1c
};

static const uint8_t read_id_vec[] = {
	0xBF, 0x00, 0xDF, 0x90, 0x00, 0xFE, 0x60, 0xFF, 0x00
};

static const int read_msb_bits = 11;
static const uint8_t read_msb_vec[] = {
	0xBF, 0x00
};

static const int read_lsb_bits = 12;
static const uint8_t read_lsb_vec[] = {
	0xDF, 0x90
};

static const int read_x_bits = 11;
static const uint8_t read_x_vec[] = {
	0xFE, 0x60
};

static const int read_a_bits = 12;
static const uint8_t read_a_vec[] = {
	0xFF, 0x00
};

static const int read_term_bits = 1;
static const uint8_t read_term_vec[] = {
	0x00
};

static const int erase_bits = 396;
static const uint8_t erase_vec[] = {
	0xde, 0xe2, 0x1f, 0x7f, 0x02, 0x7d, 0xc4, 0x09, 0xf7, 0x00, 0x1f, 0x9f,
	0x07, 0x5e, 0x7c, 0x85, 0xfd, 0xfc, 0x01, 0xf7, 0x10, 0x07, 0xdc, 0x00,
	0x7f, 0x7b, 0x80, 0x7d, 0xe0, 0x0b, 0xf7, 0xa0, 0x1f, 0xde, 0xa0, 0x1f,
	0x7b, 0x04, 0x7d, 0xf0, 0x01, 0xf7, 0xc9, 0x87, 0xdf, 0x48, 0x1f, 0x7f,
	0x89, 0x70
};

static const int rw_setup_bits = 66;
static const uint8_t rw_setup_vec[] = {
	0xde, 0xf0, 0x1f, 0x78, 0x00, 0x7d, 0xa0, 0x03, 0xc0
};

static const int write_byte_start_bits = 4;
static const uint8_t write_byte_start_vec[] = {
	0x90
};

static const int write_byte_end_bits = 3;
static const uint8_t write_byte_end_vec[] = {
	0xE0
};

static const int set_block_num_start_bits = 33;
static const uint8_t set_block_num_start_vec[] = {
	0xde, 0xe0, 0x1e, 0x7d, 0x00
};

static const int set_block_num_end_bits = 3;
static const uint8_t set_block_num_end_vec[] = {
	0xE0
};

static const int program_and_verify_bits = 440;
static const uint8_t program_and_verify_vec[] = {
	0xde, 0xe2, 0x1f, 0x7f, 0x02, 0x7d, 0xc4, 0x09, 0xf7, 0x00, 0x1f, 0x9f,
	0x07, 0x5e, 0x7c, 0x81, 0xf9, 0xf7, 0x01, 0xf7, 0xf0, 0x07, 0xdc, 0x40,
	0x1f, 0x70, 0x01, 0xfd, 0xee, 0x01, 0xf6, 0xa0, 0x0f, 0xde, 0x80, 0x7f,
	0x7a, 0x80, 0x7d, 0xec, 0x01, 0xf7, 0x80, 0x57, 0xdf, 0x00, 0x1f, 0x7c,
	0xa0, 0x7d, 0xf4, 0x61, 0xf7, 0xf8, 0x97
};

static const int checksum_setup_bits = 418;
static const uint8_t checksum_setup_vec[] = {
	0xde, 0xe2, 0x1f, 0x7f, 0x02, 0x7d, 0xc4, 0x09, 0xf7, 0x00, 0x1f, 0x9f,
	0x07, 0x5e, 0x7c, 0x81, 0xf9, 0xf4, 0x01, 0xf7, 0xf0, 0x07, 0xdc, 0x40,
	0x1f, 0x70, 0x01, 0xfd, 0xee, 0x01, 0xf7, 0xa0, 0x1f, 0xde, 0xa0, 0x1f,
	0x7b, 0x00, 0x7d, 0xe0, 0x0f, 0xf7, 0xc0, 0x07, 0xdf, 0x28, 0x1f, 0x7d,
	0x18, 0x7d, 0xfe, 0x25, 0xc0
};

static int clock_bit_in(struct device *dev)
{
	int rc;
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	gpio_set_value(pdat->sclk, 1);
	rc = gpio_get_value(pdat->sdata);
	ndelay(pdat->ssclk_ns);
	gpio_set_value(pdat->sclk, 0);
	ndelay(pdat->dsclk_ns);

	return (!!rc);
}

static u8 recv_byte(struct device *dev)
{
	u8 rc;

	clock_bit_in(dev);
	clock_bit_in(dev);
	rc = clock_bit_in(dev) << 7;
	rc |= clock_bit_in(dev) << 6;
	rc |= clock_bit_in(dev) << 5;
	rc |= clock_bit_in(dev) << 4;
	rc |= clock_bit_in(dev) << 3;
	rc |= clock_bit_in(dev) << 2;
	rc |= clock_bit_in(dev) << 1;
	rc |= clock_bit_in(dev);

	return (rc);
}

static void clock_bit_out(struct device *dev, int val)
{
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	gpio_set_value(pdat->sclk, 1);
	gpio_set_value(pdat->sdata, val);
	ndelay(pdat->ssclk_ns);
	gpio_set_value(pdat->sclk, 0);
	ndelay(pdat->hsclk_ns);
}

static void send_vector(struct device *dev, const u8 *vec, int bits)
{
	int i;
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	gpio_direction_output(pdat->sdata, 0);

	for (i = 0; i < bits; ++i)
		clock_bit_out(dev, !!((vec[i >> 3] << (i & 0x7)) & 0x80));

	gpio_direction_input(pdat->sdata);
}

static int wait_and_poll(struct device *dev)
{
	int rc;
	struct timespec now;
	struct timespec expiry;
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	udelay(1); /* allow sdata to settle */
	ktime_get_ts(&expiry);
	timespec_add_ns(&expiry, (s64)pdat->wait_and_poll_ms * NSEC_PER_MSEC);

	while (!clock_bit_in(dev)) {
		ktime_get_ts(&now);

		if (timespec_compare(&now, &expiry) > 0) {
			rc = -ETIME;
			dev_err(dev, "timed out waiting for high\n");
			goto exit;
		}
	}

	while (gpio_get_value(pdat->sdata)) {
		ktime_get_ts(&now);

		if (timespec_compare(&now, &expiry) > 0) {
			rc = -ETIME;
			dev_err(dev, "timed out waiting for low\n");
			goto exit;
		}
	}

	send_vector(dev, wait_and_poll_end_vec, wait_and_poll_end_bits);
	rc = 0;
exit:
	return (rc);
}

static int start_issp(struct device *dev)
{
	int rc;
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	rc = pdat->sclk_request(1);
	if (rc < 0)
		goto exit;

	rc = pdat->sdata_request(1);
	if (rc < 0)
		goto request_sdata_failed;

	gpio_direction_output(pdat->sclk, 0);
	gpio_direction_input(pdat->sdata);

	pdat->xres_assert(1);
	udelay(pdat->xres_us);

	CPUFREQ_HOLD_SYNC();
	
	local_irq_disable();
	{
		pdat->xres_assert(0);
		ndelay(pdat->reset_ns);
		/*
		 * according to the spec, only the first 8 bits of the
		 * id-setup-1 vector need to be sent within the Txresini
		 * window. however, the positive clock edge of the 9th bit
		 * appears to be required to demarcate the first 8 bits, so we
		 * send the first 16 bits here.
		 */
		send_vector(dev, &id_setup_1_vec[0], 16);
	}
	local_irq_enable();

	CPUFREQ_UNHOLD();	

	send_vector(dev, &id_setup_1_vec[2], id_setup_1_bits - 16);
	rc = wait_and_poll(dev);
	if (rc < 0) {
		dev_err(dev, "error %d during id-setup-1 wait-and-poll\n", rc);
		goto wait_and_poll_failed;
	}

	rc = 0;
	goto exit;

wait_and_poll_failed:
	pdat->sdata_request(0);
request_sdata_failed:
	pdat->sclk_request(0);
exit:
	return (rc);
}

static int read_silicon_id(struct device *dev, struct silicon_id *id)
{
	int rc;

	send_vector(dev, id_setup_2_vec, id_setup_2_bits);
	rc = wait_and_poll(dev);
	if (rc < 0) {
		dev_err(dev, "error %d during id-setup-2 wait-and-poll\n", rc);
		goto exit;
	}

	send_vector(dev, tsync_enable_vec, tsync_enable_bits);
	send_vector(dev, read_msb_vec, read_msb_bits);
	id->id[0] = recv_byte(dev);

	send_vector(dev, read_lsb_vec, read_lsb_bits);
	id->id[1] = recv_byte(dev);

	send_vector(dev, read_term_vec, read_term_bits);
	send_vector(dev, read_x_vec, read_x_bits);
	id->family = recv_byte(dev);

	send_vector(dev, read_a_vec, read_a_bits);
	id->rev = recv_byte(dev);

	send_vector(dev, read_term_vec, read_term_bits);
	send_vector(dev, tsync_disable_vec, tsync_disable_bits);
	rc = 0;
exit:
	return (rc);
}

static void end_issp(struct device *dev)
{
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	pdat->sdata_request(0);
	pdat->sclk_request(0);

	pdat->xres_assert(1);
	udelay(pdat->xres_us);
	pdat->xres_assert(0);
}

static ssize_t cy8ctma300_attr_vcpin_store(struct device *dev,
										struct device_attribute *attr,
										const char *buf, size_t count)
{
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	pdat->vcpin_enable(!!simple_strtoul(buf, NULL, 10));

	return (count);
}

static struct device_attribute cy8ctma300_attr_vcpin = {
	.attr = {
		.name = "vcpin",
  		.mode = S_IWUGO,
	},
	.store = cy8ctma300_attr_vcpin_store,
};

static ssize_t cy8ctma300_attr_xres_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int assert = !!simple_strtoul(buf, NULL, 10);
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	pdat->xres_assert(assert);

	if (assert)
		udelay(pdat->xres_us);

	return (count);
}

static struct device_attribute cy8ctma300_attr_xres = {
	.attr = {
		.name = "xres",
		.mode = S_IWUGO,
	},
	.store = cy8ctma300_attr_xres_store,
};

static ssize_t cy8ctma300_attr_id_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t rc;
	struct silicon_id id;

	rc = start_issp(dev);
	if (rc < 0)
		goto exit;

	rc = read_silicon_id(dev, &id);
	if (rc < 0)
		goto end;

	rc = snprintf(buf, PAGE_SIZE, "%02x%02x %02x %02x\n",
			id.id[0], id.id[1], id.family, id.rev);
end:
	end_issp(dev);
exit:
	return (rc);
}

static struct device_attribute cy8ctma300_attr_id = {
	.attr = {
		.name = "id",
		.mode = S_IRUGO,
	},
	.show = cy8ctma300_attr_id_show,
};

static void write_bytes(struct device *dev, u8 addr, const u8 *data, size_t nr)
{
	u8 vec[1];
	size_t i;

	for (i = 0; i < nr; i++, addr++) {
		send_vector(dev, write_byte_start_vec, write_byte_start_bits);
		vec[0] = addr << 1;
		send_vector(dev, vec, 7);
		send_vector(dev, &data[i], 8);
		send_vector(dev, write_byte_end_vec, write_byte_end_bits);
	}
}

static ssize_t cy8ctma300_attr_flash_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	u8 vec[1];
	u8 addr;
	int i;
	char name[FIRMWARE_NAME_MAX];
	ssize_t rc;
	struct silicon_id id;
	const struct firmware *fw = NULL;
	const struct ihex_binrec *rec = NULL;
	struct cy8ctma300_platform_data *pdat = dev->platform_data;

	strlcpy(name, buf, FIRMWARE_NAME_MAX);
	rc = request_ihex_firmware(&fw, name, dev);
	if (rc < 0) {
		dev_err(dev, "error %d requesting firmware %s\n", rc, name);
		goto exit;
	}

	rc = start_issp(dev);
	if (rc < 0)
		goto release;

	rc = read_silicon_id(dev, &id);
	if (rc < 0)
		goto end;

	dev_info(dev, "programming id %02x%02x\n", id.id[0], id.id[1]);

	send_vector(dev, erase_vec, erase_bits);
	rc = wait_and_poll(dev);
	if (rc < 0) {
		dev_err(dev, "error %d during erase\n", rc);
		goto end;
	}

	rec = (struct ihex_binrec *)fw->data;

	for (i = 0; i < pdat->nr_blocks; i++) {
		send_vector(dev, rw_setup_vec, rw_setup_bits);

		for (addr = 0; addr < pdat->block_len; addr += pdat->prec_len) {
			if (i || addr)
				rec = ihex_next_binrec(rec);

			if (!rec) {
				rc = -EINVAL;
				dev_err(dev, "missing program record\n");
				goto end;
			}

			else if (be16_to_cpu(rec->len) != pdat->prec_len) {
				rc = -EINVAL;
				dev_err(dev, "bad program record\n");
				goto end;
			}

			write_bytes(dev, addr, rec->data, pdat->prec_len);
		}

		send_vector(dev, tsync_enable_vec, tsync_enable_bits);
		send_vector(dev, set_block_num_start_vec, set_block_num_start_bits);
		vec[0] = i;
		send_vector(dev, vec, 8); /* block */
		send_vector(dev, set_block_num_end_vec, set_block_num_end_bits);
		send_vector(dev, tsync_disable_vec, tsync_disable_bits);
		send_vector(dev, program_and_verify_vec, program_and_verify_bits);
		rc = wait_and_poll(dev);
		if (rc < 0) {
			dev_err(dev, "error %d during program\n", rc);
			goto end;
		}

		send_vector(dev, tsync_enable_vec, tsync_enable_bits);
		send_vector(dev, read_msb_vec, read_msb_bits);

		rc = recv_byte(dev);
		if (rc != 0) {
			rc = -EIO;
			dev_err(dev, "error 0x%02x during program\n", rc);
			goto end;
		}

		send_vector(dev, read_lsb_vec, read_lsb_bits);
		recv_byte(dev);
		send_vector(dev, read_term_vec, read_term_bits);
		send_vector(dev, tsync_disable_vec, tsync_disable_bits);
	}

	/* TODO: security */
	for (i = 0; i < pdat->nr_srecs; i++) {
		rec = ihex_next_binrec(rec);
		if (!rec) {
			rc = -EINVAL;
			dev_err(dev, "missing security record\n");
			goto end;
		}
	}

	/* TODO: checksum */
	rec = ihex_next_binrec(rec);
	if (!rec) {
		rc = -EINVAL;
		dev_err(dev, "missing checksum record\n");
		goto end;
	}

	else if (be16_to_cpu(rec->len) != 2) {
		rc = -EINVAL;
		dev_err(dev, "bad checksum record\n");
		goto end;
	}

	dev_info(dev, "programmed checksum %02x%02x\n", rec->data[0], rec->data[1]);
	rc = count;
end:
	end_issp(dev);
release:
	release_firmware(fw);
exit:
	return (rc);
}

static struct device_attribute cy8ctma300_attr_flash = {
        .attr = {
                .name = "flash",
                .mode = S_IWUGO,
        },
        .store = cy8ctma300_attr_flash_store,
};


static int cy8ctma300_device_probe(struct platform_device *pdev)
{
	int rc;
	struct cy8ctma300_platform_data *pdat = pdev->dev.platform_data;

	if (!pdat) {
		rc = -ENODEV;
		goto failed;
	}

	BUG_ON(!pdat->sclk_request);
	BUG_ON(!pdat->sdata_request);
	BUG_ON(!pdat->xres_assert);
	
	pdat->xres_assert(0);

	if (pdat->vcpin_enable)
		pdat->vcpin_enable(1);
	
	rc = device_create_file(&pdev->dev, &cy8ctma300_attr_xres);
	if (rc < 0)
		goto failed;

	rc = device_create_file(&pdev->dev, &cy8ctma300_attr_id);
	if (rc < 0)
		goto attr_id_failed;

	rc = device_create_file(&pdev->dev, &cy8ctma300_attr_flash);
	if (rc < 0) 
		goto attr_flash_failed;

	if (pdat->vcpin_enable) {
		rc = device_create_file(&pdev->dev, &cy8ctma300_attr_vcpin);
		if (rc < 0)
			goto attr_vcpin_failed;
	}
	
	rc = 0;
	goto exit;

attr_vcpin_failed:
	device_remove_file(&pdev->dev, &cy8ctma300_attr_flash);		
attr_flash_failed:
	device_remove_file(&pdev->dev, &cy8ctma300_attr_id);
attr_id_failed:
	device_remove_file(&pdev->dev, &cy8ctma300_attr_xres);
failed:
	dev_err(&pdev->dev, "probe failed with %d\n", rc);
exit:
	return (rc);
}

static int cy8ctma300_device_remove(struct platform_device *pdev)
{
	return (0);
}

#ifdef CONFIG_PM
static int cy8ctma300_device_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct cy8ctma300_platform_data *pdat = pdev->dev.platform_data;

	if (pdat->vcpin_enable)
		pdat->vcpin_enable(0);

	return (0);
}

static int cy8ctma300_device_resume(struct platform_device *pdev)
{
	struct cy8ctma300_platform_data *pdat = pdev->dev.platform_data;

	if (pdat->vcpin_enable)
		pdat->vcpin_enable(1);

	return (0);
}
#endif /* CONFIG_PM */

static struct platform_driver cy8ctma300_driver = {
	.driver = {
		.name = CY8CTMA300_DRIVER,
	},
	.probe = cy8ctma300_device_probe,
	.remove = __devexit_p(cy8ctma300_device_remove),
#ifdef CONFIG_PM
	.suspend = cy8ctma300_device_suspend,
	.resume = cy8ctma300_device_resume,
#endif /* CONFIG_PM */
};

static int __init cy8ctma300_module_init(void)
{
	int rc;

	rc = platform_driver_register(&cy8ctma300_driver);

	return (rc);
}

static void __exit cy8ctma300_module_exit(void)
{
	platform_driver_unregister(&cy8ctma300_driver);
}

module_init(cy8ctma300_module_init);
module_exit(cy8ctma300_module_exit);

MODULE_DESCRIPTION("cy8ctma300 driver");
MODULE_LICENSE("GPL");
