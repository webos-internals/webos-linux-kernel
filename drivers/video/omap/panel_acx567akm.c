/*
 * panel_acx567akm.c
 *
 * LCD panel driver for Sony ACX567AKM series LCDs
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
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/spi/acx567akm.h>

#include <asm/arch/mcspi.h>
#include <asm/arch/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/lcd.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/dmtimer.h>

#include "lcd.h"
#include "lcd_controller_keuka.h"

#define MOD_NAME 		"LCD Panel: "

#undef MODDEBUG
//#define MODDEBUG

#ifdef  MODDEBUG
#define DPRINTK(format,...)\
	printk(KERN_INFO MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define PANEL_STATE_ON		1
#define PANEL_STATE_OFF	0

#define ACX567AKM_REG_SLPIN		0x10
#define ACX567AKM_REG_SLPOUT		0x11
#define ACX567AKM_REG_DISPOFF		0x28
#define ACX567AKM_REG_DISPON		0x29

#define ACX567AKM_REG_RDDPM		0x0a
#define ACX567AKM_REG_RDDMADCTL	0x0b
#define ACX567AKM_REG_RDDCOLMOD	0x0c
#define ACX567AKM_REG_RDDSM		0x0e
#define ACX567AKM_REG_RDDSDR		0x0f

#define WRLCDBV		0x51			/* write brightness value */
#define RDLCDBV		0x52			/* read brightness value */
#define LCDBV_LBV(v)	(0xFF & (v))		/* brightness value [0, 255] */

#define WRCTRLL		0x53			/* write brightness control */
#define RDCTRLL		0x54			/* read brightness control */
#define CTRLL_BCTRL(v)	((0x1 & (v)) << 5)	/* control on/off */
#define CTRLL_BL(v)	((0x1 & (v)) << 2)	/* backlight on/off */

#define ACX567AKM_REG_ID0		0xda
#define ACX567AKM_REG_ID1		0xdb
#define ACX567AKM_REG_ID2		0xdc

#define WRCABC_OFF			0
#define WRCABC_UI_MODE			1
#define WRCABC_STILL_MODE		2
#define WRCABC_MOVIE_MODE		3

#define MAX_DISPLAY_NAME_LENGTH	13






/*
 * spi related functions start here...
 */
struct lcdpanel_params
{
	struct device *parent_dev;
	struct mutex ops_lock;
	u32 cur_state;

#ifndef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	u32 panel_use_gpio_enable;
	u32 panel_enable_gpio;
	u32 panel_use_gpio_reset;
	u32 panel_reset_gpio;
#endif

	int panel_type;
	u8 panel_id[3];

	u32 reg_to_read;

#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
	int	bl_brightness;	/* [0, 100] */

	void *sbc_data; 
	u8 use_sb;
	u8 on_full;
	void *(*sbc_init)(void *config_data);
	void (*sbc_remove)(void *sbc_data);
	void (*sbc_preon)(void *data);
	void (*sbc_on)(void *data);
	void (*sbc_brightness)(void *data, int brightness, int yield);
	void (*sbc_off)(void *sbc_data);
#endif /* CONFIG_SPI_ACX567AKM_BACKLIGHT */
};






/*
 *  write 8 bit value
 */
static int lcdpanel_write_val8(struct spi_device *spi, u16 val)
{
	int rc;
	u8 cmd;

	DPRINTK("%s\n", __FUNCTION__);

	cmd = (u8)val;
	rc = spi_write(spi, &cmd, 1);
	if (rc) {
		printk(KERN_ERR MOD_NAME
			"write command (0x%x) failed (%d)\n", cmd, rc);
	}
	return 0;
}

/*
 *  read reg8
 */
static int lcdpanel_read_reg8(struct spi_device *spi,
					u32 reg,
					u8 * pdata)
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

int acx567akm_spi_read_u8(struct spi_device *spi, u8 reg, u8* val)
{
	return (lcdpanel_read_reg8(spi, reg, val));
}
EXPORT_SYMBOL(acx567akm_spi_read_u8);

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
 *  write 16 bit value
 */
static int lcdpanel_write_val17(struct spi_device *spi, u32 val)
{
	int rc;
	struct spi_message m;
	struct spi_transfer t;

	memset(&t, 0, sizeof(t));
	spi_message_init(&m);

	t.bits_per_word = 17;

	t.tx_buf = &val;
	t.len = 4;
	spi_message_add_tail(&t, &m);

	rc = spi_sync(spi, &m);
	if (rc) {
		DPRINTK("write val17 (0x%x) failed (%d)\n", (int)val, rc);
		return rc;
	}
	return 0;
}

int acx567akm_spi_write_u8(struct spi_device *spi, u8 reg, u8 val)
{
	int rc;
	struct lcdpanel_params *params = spi_get_drvdata(spi);

	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2)
		rc = lcdpanel_write_val17(spi, ((u32)reg << 9)| 0x100 | val);

	else
		rc = lcdpanel_write_val16(spi, ((u16)reg << 8) | val);

	return (rc);
}
EXPORT_SYMBOL(acx567akm_spi_write_u8);

/*
 *  Read panel id
 */
static int lcdpanel_read_id(struct spi_device *spi)
{
	struct lcdpanel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);

	mutex_lock(&params->ops_lock);

	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2) {
		/* not sure yet whtat to read here */
		lcdpanel_read_reg8(spi, ACX567AKM_REG_ID0, &params->panel_id[0]);
		lcdpanel_read_reg8(spi, ACX567AKM_REG_ID1, &params->panel_id[1]);
		lcdpanel_read_reg8(spi, ACX567AKM_REG_ID2, &params->panel_id[2]);

	} else {
		lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

		lcdpanel_read_reg8(spi, KEUKA_OP_READ_ID0, &params->panel_id[0]);
		lcdpanel_read_reg8(spi, KEUKA_OP_READ_ID1, &params->panel_id[1]);
		lcdpanel_read_reg8(spi, KEUKA_OP_READ_ID2, &params->panel_id[2]);
	}

	mutex_unlock(&params->ops_lock);

	printk(KERN_INFO "lcd panel ID: %x %x %x\n",
		(int)params->panel_id[0],
		(int)params->panel_id[1],
		(int)params->panel_id[2]);

	return 0;
}

/*
 *  Read panel status
 */

static int lcdpanel_read_status(struct spi_device *spi)
{
	struct lcdpanel_params *params = spi_get_drvdata(spi);
	u8 rx[8] = { 	0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xff, 0xff };

	DPRINTK("%s\n", __FUNCTION__);

	mutex_lock(&params->ops_lock);

	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2) {
		lcdpanel_read_reg8(spi, ACX567AKM_REG_RDDPM, rx + 0);
		lcdpanel_read_reg8(spi, ACX567AKM_REG_RDDMADCTL, rx + 1);
		lcdpanel_read_reg8(spi, ACX567AKM_REG_RDDCOLMOD, rx + 2);
		lcdpanel_read_reg8(spi, ACX567AKM_REG_RDDSM, rx + 3);
		lcdpanel_read_reg8(spi, ACX567AKM_REG_RDDSDR, rx + 4);

		printk(KERN_INFO "lcd panel status: %x %x %x %x %x\n",
		       (int)rx[0], (int)rx[1], (int)rx[2],
		       (int)rx[3], (int)rx[4]);

	} else {

		lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

		lcdpanel_read_reg8(spi, 0xA, rx + 0);
		lcdpanel_read_reg8(spi, 0xB, rx + 1);
		lcdpanel_read_reg8(spi, 0xC, rx + 2);
		lcdpanel_read_reg8(spi, 0xD, rx + 3);
		lcdpanel_read_reg8(spi, 0xE, rx + 4);
		lcdpanel_read_reg8(spi, 0xF, rx + 5);
		printk(KERN_INFO "lcd panel status: %x %x %x %x %x %x\n",
		       (int)rx[0], (int)rx[1], (int)rx[2],
		       (int)rx[3], (int)rx[4], (int)rx[5]);
	}

	mutex_unlock(&params->ops_lock);

	return 0;
}

static void lcdpanel_enable(struct spi_device *spi)
{
	struct lcdpanel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);
	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	mutex_lock(&params->ops_lock);

	if (params->cur_state == PANEL_STATE_ON)
		goto out;

#ifdef CONFIG_SPI_ACX567AKM_EXTERNAL_PLL_EXTRA_DELAY
	msleep(3);
#endif

	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2) {
		/* sleepout */
		lcdpanel_write_val8(spi, ACX567AKM_REG_SLPOUT);

		/* display on */
		lcdpanel_write_val8(spi, ACX567AKM_REG_DISPON);
	} else {

		/* command mode */
		lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

		/* Sleep out */
		lcdpanel_write_val8(spi, KEUKA_OP_SLEEP_OUT);

		/* Display on */
		lcdpanel_write_val8(spi, KEUKA_OP_DISPLAY_ON);

		/* Normal mode */
		lcdpanel_write_val8(spi, KEUKA_OP_NORMAL_MODE_ON);
	}

	params->cur_state = PANEL_STATE_ON;

out:
	mutex_unlock(&params->ops_lock);
}

static void lcdpanel_disable(struct spi_device *spi)
{
	struct lcdpanel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);
	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	mutex_lock(&params->ops_lock);

	if (params->cur_state == PANEL_STATE_OFF)
		goto out;

	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2) {
		/* display off*/
		lcdpanel_write_val8(spi, ACX567AKM_REG_DISPOFF);

		/* sleep in */
		lcdpanel_write_val8(spi, ACX567AKM_REG_SLPIN);
	} else {

		/* command mode */
		lcdpanel_write_val8(spi, KEUKA_OP_ENTER_COMMAND_MODE);

		/* Display off */
		lcdpanel_write_val8(spi, KEUKA_OP_DISPLAY_OFF);

		/* Sleep in */
		lcdpanel_write_val8(spi, KEUKA_OP_SLEEP_IN);
	}

	/* We need to wait for 2 frames after the SLEEP_IN command for the LCD
	 * panel to enter low power mode.
	 */
	msleep(40);

	/* We also need to wait 120 ms after sending a SLEEP_IN command before
	 * we can send a SLEEP_OUT command.
	 */
	msleep(120);

	params->cur_state = PANEL_STATE_OFF;

out:
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

	lcdpanel_enable(spi);

	return snprintf(buf, PAGE_SIZE, "OK\n");
}
static DEVICE_ATTR(panel_on, S_IRUGO, lcd_panel_show_panel_on, NULL);

static ssize_t lcd_panel_show_panel_off(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct spi_device *spi = to_spi_device(dev);

	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	lcdpanel_disable(spi);

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

static ssize_t lcd_panel_show_register_read(struct device *dev,
				struct device_attribute *dev_attr,
				char *buf)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lcdpanel_params *params = spi_get_drvdata(spi);
	u8 val;

	DPRINTK("%s\n", __FUNCTION__);

	lcdpanel_read_reg8(spi, params->reg_to_read, &val );

	return snprintf(buf, PAGE_SIZE, "0x%x\n", val);
}
DEVICE_ATTR(panel_reg_read, S_IRUGO, lcd_panel_show_register_read, NULL);

static ssize_t lcd_panel_store_register(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	int reg = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;
	struct spi_device *spi = to_spi_device(dev);
	struct lcdpanel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	params->reg_to_read = reg;

	return count;
}
DEVICE_ATTR(panel_reg_set, S_IWUSR, NULL, lcd_panel_store_register);

static ssize_t lcd_panel_store_register_write(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	char *endp;
	u8 val = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;
	struct spi_device *spi = to_spi_device(dev);
	struct lcdpanel_params *params = spi_get_drvdata(spi);
	u32 write_val;

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2) {
		write_val = (params->reg_to_read << 9) | 0x100 | val;
		lcdpanel_write_val17(spi, write_val);
	} else {
		write_val = (params->reg_to_read << 8) | val;
		lcdpanel_write_val16(spi, (u16)write_val);
	}

	return count;
}
DEVICE_ATTR(panel_reg_write, S_IWUSR, NULL, lcd_panel_store_register_write);

static ssize_t lcd_panel_store_test_state(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lcdpanel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s\n", __FUNCTION__);
	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2) {
		if ( strncmp(buf, "off", 3) == 0 ) {
			mutex_lock(&params->ops_lock);
			lcdpanel_write_val8(spi, ACX567AKM_REG_DISPOFF);
		}
		if ( strncmp(buf, "on", 2) == 0 ) {
			lcdpanel_write_val8(spi, ACX567AKM_REG_DISPON);
		}
	mutex_unlock(&params->ops_lock);
	}

	return count;
}
DEVICE_ATTR(panel_test_state, S_IWUSR, NULL, lcd_panel_store_test_state);



/*
 * panel controls
 */
static void lcd_panel_set_state(void *dev, unsigned int state)
{
	struct spi_device *spi = to_spi_device(dev);

	DPRINTK("%s:\n", __FUNCTION__);

	if (state == PANEL_STATE_OFF) {
		lcdpanel_disable(spi);
	} else {
		lcdpanel_enable(spi);
	}
}
static struct lcd_panel_ops lcd_panel_acx567akm_ops = {
	.pnl_set_state = lcd_panel_set_state,
};

#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
static void acx567akm_backlight_set_state(void *dev, unsigned int state)
{
	u8 val;
	struct spi_device *spi = to_spi_device(dev);
	struct lcdpanel_params *dat = spi_get_drvdata(spi);

	DPRINTK("%s: state=%u\n", __func__, state);

	if (state) {

		if (dat->sbc_preon)
		{
			dat->sbc_preon(dat->sbc_data);
		}

		val = CTRLL_BCTRL(1) | CTRLL_BL(1);
		acx567akm_spi_write_u8(spi, WRCTRLL, val);

		if (dat->on_full)
		{
			acx567akm_spi_write_u8(spi, WRLCDBV, 255);
		}

		if (dat->sbc_on)
		{
			dat->sbc_on(dat->sbc_data);
		}
	}
	else
	{
		acx567akm_spi_write_u8(spi, WRCTRLL, 0);
		if (dat->sbc_off)
		{
			dat->sbc_off(dat->sbc_data);
		}
	}
}

static void acx567akm_backlight_set_brightness(void *dev, int brightness)
{
	u8 val;
	struct spi_device *spi = to_spi_device(dev);
	struct lcdpanel_params *dat = spi_get_drvdata(spi);

	DPRINTK("%s: brightness=%d\n", __func__, brightness);

	if (dat->sbc_brightness != NULL) {
		if (brightness == 1000|| brightness==2000) {
			dat->sbc_brightness(dat->sbc_data, brightness,0);
			return;
		}
	}

	dat->bl_brightness = min(max(0, brightness), 100);

	if (dat->sbc_brightness != NULL) {
		dat->sbc_brightness(dat->sbc_data, dat->bl_brightness,0);
	}
	if (dat->use_sb) {
		return;
	}

	val = ((dat->bl_brightness * 255) + 50) / 100;
	acx567akm_spi_write_u8(spi, WRLCDBV, val);
}

static int acx567akm_backlight_get_brightness(void *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct lcdpanel_params *dat = spi_get_drvdata(spi);

	DPRINTK("%s\n", __func__);

	return (dat->bl_brightness);
}

static struct lcd_backlight_ops acx567akm_backlight_ops = {
	.bl_set_state = acx567akm_backlight_set_state,
	.bl_set_brightness = acx567akm_backlight_set_brightness,
	.bl_get_brightness = acx567akm_backlight_get_brightness,
};
#endif /* CONFIG_SPI_ACX567AKM_BACKLIGHT */

#ifdef CONFIG_PM
static int lcdspi_suspend(struct spi_device *spi, pm_message_t mesg)
{
	DPRINTK("%s\n", __FUNCTION__);

	/* don't suspend here - suspend from lcd_panel.c
	 *
	 * lcdpanel_disable(spi);
	 */

	return 0;
}

static int lcdspi_resume(struct spi_device *spi)
{
	DPRINTK("%s\n", __FUNCTION__);

	/* don't resume here - resume from lcd_panel.c
	 *
	 *  lcdpanel_enable(spi);
	 */

	return 0;
}
#else
#define lcdspi_suspend	NULL
#define lcdspi_resume	NULL
#endif

static int __devexit lcdspi_remove(struct spi_device *spi)
{
	struct lcdpanel_params *params = spi_get_drvdata(spi);

	DPRINTK("%s:\n", __FUNCTION__);

	acx567akm_dbg_unregister_spi_device(spi);
	device_remove_file(&spi->dev, &dev_attr_panel_on);
	device_remove_file(&spi->dev, &dev_attr_panel_off);
	device_remove_file(&spi->dev, &dev_attr_panel_id);
	device_remove_file(&spi->dev, &dev_attr_panel_status);
	device_remove_file(&spi->dev, &dev_attr_panel_reg_read);
	device_remove_file(&spi->dev, &dev_attr_panel_reg_write);
	device_remove_file(&spi->dev, &dev_attr_panel_test_state);
	device_remove_file(&spi->dev, &dev_attr_panel_reg_set);

#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
	if (params->sbc_remove != NULL) {
		params->sbc_remove(params->sbc_data);
	}
	remove_backlight_device(&spi->dev, params->parent_dev);
#endif /* CONFIG_SPI_ACX567AKM_BACKLIGHT */
	remove_panel_device(&spi->dev, params->parent_dev);

	kfree(params);

	return 0;
}

static int __init lcdspi_probe(struct spi_device *spi)
{
	int rc;
	struct panel_platform_data *plat_data;
	struct lcdpanel_params *params;

	DPRINTK("%s:\n", __FUNCTION__);

	params = kzalloc(sizeof(struct lcdpanel_params), GFP_KERNEL);
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

	/* get the display type first */
	params->parent_dev            =   plat_data->parent;
	params->panel_type            = *(plat_data->panel_type);

#ifndef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	params->panel_use_gpio_enable = *(plat_data->panel_use_gpio_enable);
	params->panel_enable_gpio     =   plat_data->panel_enable_gpio;
	params->panel_use_gpio_reset  = *(plat_data->panel_use_gpio_reset);
	params->panel_reset_gpio      =   plat_data->panel_reset_gpio;

	DPRINTK("%s: Panel Type = %d\n", __FUNCTION__, params->panel_type);

	if (params->panel_use_gpio_reset) {
		rc = omap_request_gpio(params->panel_reset_gpio);
		if (rc) {
			printk(KERN_ERR MOD_NAME
				"Error failed to reserve GPIO %d\n",
				params->panel_reset_gpio);
			rc = -ENODEV;
			goto err0;
		}

		/* set as output */
		omap_set_gpio_direction(params->panel_use_gpio_reset,
							GPIO_DIR_OUTPUT);
	}

	if (params->panel_use_gpio_enable) {
		rc = omap_request_gpio(params->panel_enable_gpio);
		if (rc) {
			printk(KERN_ERR MOD_NAME
				"Error failed to reserve GPIO %d\n",
				params->panel_enable_gpio);
			rc = -ENODEV;
			goto err1;
		}

		/* set as output */
		omap_set_gpio_direction(params->panel_enable_gpio,
							GPIO_DIR_OUTPUT);
	}
#endif

	/* Init spi interface */
	if (params->panel_type == PANEL_TYPE_ACX567AKM_TS2) {
		// PHA:even, POL:low, CS_POL:low, SBE: YES
		spi->mode = SPI_CPOL | SPI_CPHA | SPI_SBE;
		spi->bits_per_word = 8;
	} else {
		// PHA:even, POL:low, CS_POL:low, SBE: NO
		spi->mode = SPI_CPOL | SPI_CPHA;
		spi->bits_per_word = 8;
	}

	DPRINTK("SPI Mode = %x, bits per word = %d\n", spi->mode, spi->bits_per_word);

	rc = spi_setup(spi);
	if (rc) {
		printk(KERN_ERR MOD_NAME "Error setting up spi\n");
		rc = -EINVAL;
		goto err2;
	}

	mutex_init(&params->ops_lock);

#ifndef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	/* startup sequence for TS2 */
	lcdpanel_disable(spi);
	DPRINTK("%s: LCD panel disabled %d\n", __FUNCTION__, jiffies);

	/* reset the display */
	if (params->panel_use_gpio_reset) {
		omap_set_gpio_dataout(params->panel_reset_gpio, 0);
		DPRINTK("%s: LCD panel reset %d\n", __FUNCTION__, jiffies);
		msleep(50);
		omap_set_gpio_dataout( params->panel_reset_gpio, 1);
		DPRINTK("%s: LCD panel normal %d\n", __FUNCTION__, jiffies);
		msleep(50);
	}

	lcdpanel_enable(spi);
	DPRINTK("%s: LCD panel enabled %d\n", __FUNCTION__, jiffies);
	msleep(10);
#endif

#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
	params->use_sb = 
		plat_data->use_sb;
	params->on_full = 
		plat_data->on_full;
	params->sbc_init		= plat_data->sbc_init;
	params->sbc_remove		= plat_data->sbc_remove;
	params->sbc_preon		= plat_data->sbc_preon;
	params->sbc_on			= plat_data->sbc_on;
	params->sbc_brightness	= plat_data->sbc_brightness;
	params->sbc_off			= plat_data->sbc_off;

	if (params->sbc_init != NULL) {
		params->sbc_data = params->sbc_init(plat_data->sb_config);
	}
	else
	{
		params->sbc_data = NULL;
	}

	acx567akm_backlight_set_brightness(&spi->dev, 100);
	acx567akm_backlight_set_state(&spi->dev, 1);
#endif /* CONFIG_SPI_ACX567AKM_BACKLIGHT */

	/* print the panel revision number */
	lcdpanel_read_id(spi);

	/* add the panel to the lcd driver */
	add_panel_device(&spi->dev, &lcd_panel_acx567akm_ops, params->parent_dev);

#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
	add_backlight_device(&spi->dev, &acx567akm_backlight_ops,
				params->parent_dev);
#endif /* CONFIG_SPI_ACX567AKM_BACKLIGHT */

	DPRINTK("spi_device at 0x%x dev at 0x%x\n", (int)spi, (int)(&spi->dev));

	/* enable the panel */
	params->cur_state = PANEL_STATE_ON;

	/* add sysfs entries */
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_on)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_on failed\n");
		goto err3;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_off)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_off failed\n");
		goto err3;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_id)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_id failed\n");
		goto err3;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_status)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_status failed\n");
		goto err3;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_reg_read)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_reg_read failed\n");
		goto err3;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_reg_write)) < 0){
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_reg_write failed\n");
		goto err3;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_test_state)) < 0){
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_test_state failed\n");
		goto err3;
	}
	if ((rc = device_create_file(&spi->dev, &dev_attr_panel_reg_set)) < 0) {
		printk(KERN_ERR MOD_NAME
			"Creating sysfs entry for panel_reg_set failed\n");
		goto err3;
	}

	if ((rc = acx567akm_dbg_register_spi_device(spi)))
		goto err3;

	printk(KERN_INFO "LCD panel initialized\n");
	return 0;

err3:
	device_remove_file(&spi->dev, &dev_attr_panel_on);
	device_remove_file(&spi->dev, &dev_attr_panel_off);
	device_remove_file(&spi->dev, &dev_attr_panel_id);
	device_remove_file(&spi->dev, &dev_attr_panel_status);
	device_remove_file(&spi->dev, &dev_attr_panel_reg_read);
	device_remove_file(&spi->dev, &dev_attr_panel_reg_write);
	device_remove_file(&spi->dev, &dev_attr_panel_test_state);
	device_remove_file(&spi->dev, &dev_attr_panel_reg_set);

#ifdef CONFIG_SPI_ACX567AKM_BACKLIGHT
	remove_backlight_device(&spi->dev, params->parent_dev);
#endif /* CONFIG_SPI_ACX567AKM_BACKLIGHT */
	remove_panel_device(&spi->dev, params->parent_dev);
err2:
#ifndef CONFIG_OMAP_DISPLAY_ALREADY_ON_AT_BOOT
	if (params->panel_use_gpio_enable)
		omap_free_gpio(params->panel_enable_gpio);
err1:
	if (params->panel_use_gpio_reset)
		omap_free_gpio(params->panel_reset_gpio);
#endif

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
MODULE_DESCRIPTION("Sony ACX567AKM series LCD panel driver");
MODULE_LICENSE("GPL");
