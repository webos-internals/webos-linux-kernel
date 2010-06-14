/*
 * drivers/video/omap/panel_fpd95220.c
 *
 * lcd panel driver for LCD's based on National Semi's FPD95220 display
 * controller chips on OMAP based boards.
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * Author: john chen (jchen1996@gmail.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/spi/spi.h>

#include <asm/arch/mcspi.h>
#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/lcd.h>

#include "lcd.h"
#include "lcd_controller_keuka.h"

#define MOD_NAME 		"LCD Panel: "

// #define MODDEBUG		1
#undef MODDEBUG

#ifdef  MODDEBUG
#define DPRINTK(format,...)\
	printk(KERN_INFO MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define PANEL_STATE_ON		1
#define PANEL_STATE_OFF	0

/*
 * spi related functions start here...
 */
struct panel_params
{
	struct device *parent_dev;
	struct mutex ops_lock;
	int cur_state;
};

/*
 *  read reg8
 */
static int lcdpanel_read_reg8(struct spi_device *spi, u32 reg, u8 * pdata)
{
	int rc;
	u16 data = 0;
	struct spi_message m;
	struct spi_transfer t;

	reg = (reg << 8) | 0xFF;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	t.bits_per_word = 16;
	t.tx_buf = &reg;
	t.rx_buf = &data;
	t.len = 2;
	spi_message_add_tail(&t, &m);

	rc = spi_sync(spi, &m);
	if (rc) {
		printk(KERN_ERR MOD_NAME
		       "read reg (0x%x) failed (%d)", (reg >> 8), rc);
		return rc;
	}

	if (pdata) {
		*pdata = (u8) (data & 0xFF);
	}
	return 0;
}

/*
 *  write no arg command
 */
static int lcdpanel_write_val8(struct spi_device *spi, u8 cmd)
{
	int rc = spi_write(spi, &cmd, 1);
	if (rc) {
		printk(KERN_ERR MOD_NAME
		       "write command (0x%x) failed (%d)\n", cmd, rc);
	}
	return rc;
}

#if 0 /* #warning patrol: Defined but not used... - wgr */
/*
 *  write 16 bit value
 */
static int lcdpanel_write_val16(struct spi_device *spi, u16 val)
{
	int rc;
	struct spi_message m;
	struct spi_transfer t;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	t.bits_per_word = 16;
	t.tx_buf = &val;
	t.len = 2;
	spi_message_add_tail(&t, &m);

	rc = spi_sync(spi, &m);
	if (rc) {
		DPRINTK("write val16 (0x%x) failed (%d)\n", (int)val, rc);
		return rc;
	}
	return 0;
}

/*
 *  write reg
 */
static int lcdpanel_write_reg8(struct spi_device *spi, u16 reg, u8 val)
{
	return lcdpanel_write_val16(spi, (reg << 8) | val);
}
#endif

/*
 *  Read panel id
 */
static int lcdpanel_read_id(struct spi_device *spi)
{
	u8 rx[3] = { 0xFF, 0xFF, 0xFF };
	struct panel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);

	mutex_lock(&params->ops_lock);

	lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

	lcdpanel_read_reg8(spi, 0xDA, rx + 0);
	lcdpanel_read_reg8(spi, 0xDB, rx + 1);
	lcdpanel_read_reg8(spi, 0xDC, rx + 2);

	mutex_unlock(&params->ops_lock);

	printk(KERN_INFO "lcd panel ID: %x %x %x\n",
	       (int)rx[0], (int)rx[1], (int)rx[2]);

	return 0;
}

/*
 *  Read panel status
 */
static int lcdpanel_read_status(struct spi_device *spi)
{
	u8 rx[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	struct panel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);

	mutex_lock(&params->ops_lock);

	lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

	lcdpanel_read_reg8(spi, 0xA, rx + 0);
	lcdpanel_read_reg8(spi, 0xB, rx + 1);
	lcdpanel_read_reg8(spi, 0xC, rx + 2);
	lcdpanel_read_reg8(spi, 0xD, rx + 3);
	lcdpanel_read_reg8(spi, 0xE, rx + 4);
	lcdpanel_read_reg8(spi, 0xF, rx + 5);

	mutex_unlock(&params->ops_lock);

	printk(KERN_INFO "lcd panel status: %x %x %x %x %x %x\n",
	       (int)rx[0], (int)rx[1], (int)rx[2],
	       (int)rx[3], (int)rx[4], (int)rx[5]);

	return 0;
}

static void lcd_panel_enable(struct spi_device *spi)
{
	struct panel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);
	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	if (params->cur_state == PANEL_STATE_ON)
		return;

	mutex_lock(&params->ops_lock);

	/* command mode */
	lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

	/* Sleep out */
	lcdpanel_write_val8(spi, KEUKA_OP_SLEEP_OUT);
	mdelay(15);

	/* Display on */
	lcdpanel_write_val8(spi, KEUKA_OP_DISPLAY_ON);
	mdelay(15);

	/* Normal mode */
	lcdpanel_write_val8(spi, KEUKA_OP_NORMAL_MODE_ON);

	params->cur_state = PANEL_STATE_ON;

	mutex_unlock(&params->ops_lock);
}

static void lcd_panel_disable(struct spi_device *spi)
{
	struct panel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);
	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	if (params->cur_state == PANEL_STATE_OFF)
		return;

	mutex_lock(&params->ops_lock);

	/* command mode */
	lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

	/* Display off */
	lcdpanel_write_val8(spi, KEUKA_OP_DISPLAY_OFF);

	/* Sleep in */
	lcdpanel_write_val8(spi, KEUKA_OP_SLEEP_IN);

	params->cur_state = PANEL_STATE_OFF;

	mutex_unlock(&params->ops_lock);
}

/*
 * panel specific sysfs entries
 */
static ssize_t lcd_panel_show_panel_on(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct spi_device *spi = to_spi_device(dev);

	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	lcd_panel_enable(spi);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
static DEVICE_ATTR(panel_on, S_IRUGO, lcd_panel_show_panel_on, NULL);

static ssize_t lcd_panel_show_panel_off(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct spi_device *spi = to_spi_device(dev);

	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	lcd_panel_disable(spi);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
static DEVICE_ATTR(panel_off, S_IRUGO, lcd_panel_show_panel_off, NULL);

static ssize_t lcd_panel_show_panel_id(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct spi_device *spi = to_spi_device(dev);

	lcdpanel_read_id(spi);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
static DEVICE_ATTR(panel_id, S_IRUGO, lcd_panel_show_panel_id, NULL);

static ssize_t lcd_panel_show_panel_status(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct spi_device *spi = to_spi_device(dev);

	lcdpanel_read_status(spi);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
static DEVICE_ATTR(panel_status, S_IRUGO, lcd_panel_show_panel_status, NULL);

static void lcd_panel_set_state(void *dev, unsigned int state)
{
	struct spi_device *spi = to_spi_device(dev);

	DPRINTK("%s:\n", __FUNCTION__);

	if (state == PANEL_STATE_OFF) {
		lcd_panel_disable(spi);
	} else {
		lcd_panel_enable(spi);
	}
}
static struct lcd_panel_ops lcd_panel_fpd95220_ops = {
	.pnl_set_state = lcd_panel_set_state,
};

#ifdef CONFIG_PM
static int lcdspi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	DPRINTK("%s\n", __FUNCTION__);

	lcd_panel_disable(spi);

	return 0;
}

static int lcdspi_resume(struct spi_device *spi)
{
	DPRINTK("%s\n", __FUNCTION__);

	lcd_panel_enable(spi);

	return 0;
}
#else
#define lcdspi_suspend	NULL
#define lcdspi_resume	NULL
#endif

static int __devexit lcdspi_remove(struct spi_device *spi)
{
	struct panel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s:\n", __FUNCTION__);

	device_remove_file(&spi->dev, &dev_attr_panel_on);
	device_remove_file(&spi->dev, &dev_attr_panel_off);
	device_remove_file(&spi->dev, &dev_attr_panel_id);
	device_remove_file(&spi->dev, &dev_attr_panel_status);

	remove_panel_device(&spi->dev, params->parent_dev);

	kfree(params);

	return 0;
}

static int __init lcdspi_probe(struct spi_device *spi)
{
	int rc;
	struct panel_platform_data *plat_data;
	struct panel_params *params;

	DPRINTK("%s:\n", __FUNCTION__);

	params = kzalloc(sizeof(struct panel_params), GFP_KERNEL);
	if (unlikely(!params)) {
		printk(KERN_ERR MOD_NAME "Error, no memeory\n");
		return -ENOMEM;
	}
	spi_set_drvdata(spi, params);

	if (spi->dev.platform_data != NULL) {
		plat_data = (struct panel_platform_data*)spi->dev.platform_data;
	} else {
		printk(KERN_ERR MOD_NAME "Error, no platform data\n");
		rc = -EINVAL;
		goto err0;
	}

	params->parent_dev = plat_data->parent;
	mutex_init(&params->ops_lock);

	/* Init spi interface */
	spi->mode = SPI_CPOL | SPI_CPHA,	// PHA:even, POL:low, CS_POL:low
		spi->bits_per_word = 8;
	rc = spi_setup(spi);
	if (rc) {
		printk(KERN_ERR MOD_NAME "Error setting up spi\n");
		rc = -EINVAL;
		goto err0;
	}

	/* print the panel revision number */
	lcdpanel_read_id(spi);

	/* add the panel to the lcd driver */
	add_panel_device(&spi->dev, &lcd_panel_fpd95220_ops, params->parent_dev);

	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	/* enable the panel */
	// lcd_panel_set_state(pdev, PANEL_STATE_ON);
	params->cur_state = PANEL_STATE_ON;

	/* add sysfs entries */
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_on)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_on failed\n");

		goto err1;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_off)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_off failed\n");
		goto err1;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_id)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_off failed\n");
		goto err1;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_status)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_off failed\n");
		goto err1;
	}

	printk(KERN_INFO "LCD panel initialized\n");
	return 0;

err1:
	device_remove_file(&spi->dev, &dev_attr_panel_on);
	device_remove_file(&spi->dev, &dev_attr_panel_off);
	device_remove_file(&spi->dev, &dev_attr_panel_id);
	device_remove_file(&spi->dev, &dev_attr_panel_status);

	remove_panel_device(&spi->dev, params->parent_dev);

err0:
	kfree(params);

	return rc;
}

static struct spi_driver lcdpanel_spi_driver = {
	.driver = {
		.name =		"lcdpanel",
		.bus = 		&spi_bus_type,
		.owner = 	THIS_MODULE,
	},
	.probe = 	lcdspi_probe,
	.remove = 	lcdspi_remove,
	.suspend = 	lcdspi_suspend,
	.resume = 	lcdspi_resume,
};

static int __init lcd_panel_init(void)
{
	return spi_register_driver(&lcdpanel_spi_driver);
}

static void __exit lcd_panel_exit(void)
{
	spi_unregister_driver(&lcdpanel_spi_driver);
}

module_init(lcd_panel_init);
module_exit(lcd_panel_exit);

MODULE_AUTHOR("John Chen <jchen1996@gmail.com>");
MODULE_DESCRIPTION("FPD95220 LCD panel driver");
MODULE_LICENSE("GPL");
