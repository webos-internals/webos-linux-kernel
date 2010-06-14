/*
 * linux/arch/arm/mach-omap3pe/board-3430sdp-hsmmc.c
 *
 * Copyright (C) 2007 Texas Instruments
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <asm/hardware.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/twl4030-gpio.h>
#include <asm/arch/board.h>
#include <asm/io.h>

#define mmc_slot1		1
#define mmc_slot2		2
#define T2_MMC1_CD		0
#define T2_MMC2_CD		1
#define VMMC1_DEV_GRP		0x27
#define P1_DEV_GRP		0x20
#define VMMC1_DEDICATED		0x2A
#define VSEL_3V			0x02
#define VSEL_18V		0x00
#define PBIAS_3V		0x03
#define PBIAS_18V		0x02
#define PBIAS_CLR		0x00
#define TWL_GPIO_PUPDCTR1	0x13
#define TWL_GPIO_IMR1A		0x1C
#define TWL_GPIO_ISR1A		0x19
#define VMMC1_DEV_GRP		0x27
#define VMMC1_DEDICATED		0x2A
#define VMMC2_DEV_GRP		0x2B
#define VMMC2_DEDICATED		0x2E
#define VSEL_S2_18V		0x05
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		1 << 0
#define GPIO_1_BIT_POS		1 << 1

#define VSIM_DEV_GRP		0x37
#define VSIM_DEDICATED		0x3A

extern int twl4030_request_gpio(int gpio);
extern int twl4030_set_gpio_edge_ctrl(int gpio, int edge);
extern int twl4030_set_gpio_debounce(int gpio, int enable);
extern int twl4030_get_gpio_datain(int gpio);

/*
 * Enable power to MMC controller from T2. 
 * slot - MMC1 or MMC2.
 */
int enable_mmc_power(int slot)
{
	int ret = 0;
	u8 reg = 0;

	if (slot == mmc_slot1) {
		/* Enable SLOT1 and Power up */
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				P1_DEV_GRP, VMMC1_DEV_GRP);
		if (ret != 0) {
			printk(KERN_ERR
				"Configuring MMC1 device group failed\n");
			return ret;
		}

		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
	      			VSEL_3V, VMMC1_DEDICATED);
		if (ret != 0) {
			printk(KERN_ERR
				"Configuring MMC1 dedicated failed\n");
			return ret;
		}
		
		/* Enable VSIM to support MMC 8-bit on ES2 */
		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) {
			ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER,
				&reg, VSIM_DEV_GRP);
			if (ret != 0) {
				printk(KERN_ERR
					"Reading VSIM DEV GRP failed\n");
				return ret;
			}
			ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				reg | (1<<5), VSIM_DEV_GRP);
			if (ret != 0) {
				printk(KERN_ERR
					"Setting VSIM DEV GRP failed\n");
				return ret;
			}
			reg = 0;
			ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER,
				&reg, VSIM_DEDICATED);
			if (ret != 0) {
				printk(KERN_ERR
					"Reading VSIM DEDICATED failed\n");
				return ret;
			}
			ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				reg | 0x7, VSIM_DEDICATED);
			if (ret != 0) {
				printk(KERN_ERR
					"Enabling VSIM LDO failed\n");
				return ret;
			}
		}

	} else if (slot == mmc_slot2) {
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, P1_DEV_GRP,
				VMMC2_DEV_GRP);
		if (ret != 0) {
			printk(KERN_ERR
				"Configuring MMC2 device group  failed\n");
			return ret;
		}
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,VSEL_S2_18V,
				VMMC2_DEDICATED);
		if (ret != 0) {
			printk(KERN_ERR "Configuring MMC2 dedicated  failed\n");
			return ret;
		}
	}
	return 0;
}
/*
 * Disable power to MMC controller from T2. 
 * slot - MMC1 or MMC2.
 */
int disable_mmc_power(int slot)
{
	int ret = 0;

	if (slot == mmc_slot1) {
		/* Disable MMC SLOT1 and Power off */
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, LDO_CLR,
	       			VMMC1_DEV_GRP);
		if (ret != 0) {
			printk(KERN_ERR "Configuring MMC1 dev grp failed\n");
			return ret;
		}
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,VSEL_S2_CLR,
				VMMC1_DEDICATED);
		if (ret != 0) {
			printk(KERN_ERR "Configuring MMC1 dedicated failed\n");
			return ret;
		}

		if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) {
			ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				LDO_CLR, VSIM_DEV_GRP);
			if (ret != 0) {
				printk(KERN_ERR "Clearing VSIM GRP failed\n");
				return ret;
			}
			ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				LDO_CLR, VSIM_DEDICATED);
			if (ret != 0) {
				printk(KERN_ERR "Clearing VSIM LDO failed\n");
				return ret;
			}
		}

	} else if (slot == mmc_slot2) {
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, LDO_CLR,
				VMMC2_DEV_GRP);
		if (ret != 0) {
			printk(KERN_ERR "Configuring MMC2 dev grp failed\n");
			return ret;
		}
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,VSEL_S2_CLR,
				VMMC2_DEDICATED);
		if (ret != 0) {
			printk(KERN_ERR "Configuring MMC2 dedicated failed\n");
			return ret;
		}
	}
	return 0;
}

/* 
 * Configure the GPIO parameters for the MMC hotplug irq
 */
int setup_mmc_carddetect_irq(int irq)
{
	int ret = 0;

	/* configure T2 GPIO interrupts to detect card insertion/removal */
	ret = twl4030_request_gpio(irq);
	if (ret != 0) {
		printk(KERN_ERR "failed in request gpio =%d\n", ret);
		return ret;
	}
	ret =  twl4030_set_gpio_edge_ctrl(irq,TWL4030_GPIO_EDGE_RISING |
					TWL4030_GPIO_EDGE_FALLING);
	if (ret != 0) {
		printk(KERN_ERR "failed in gpio ctrl edge =%d\n", ret);
		return ret;
	}
	/* hack: the pullup configuration seems to be not behaving correctly */
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x02,
				  	TWL_GPIO_PUPDCTR1);
	if (ret != 0) {
		printk(KERN_ERR "failed in gpio pull =%d\n", ret);
		return ret;
	}
	ret = twl4030_set_gpio_debounce(irq,TWL4030_GPIO_IS_ENABLE);
	if (ret != 0) {
		printk(KERN_ERR "failed in gpio ctrl edge =%d\n", ret);
		return ret;
	}
	return 0;
}

/* 
 * Sysfs entry function to show wether card is inserted or removed for MMC1
 */
ssize_t mmc_omap_show_cover_switch(struct device *dev, struct
					   device_attribute *attr, char *buf)
{
	struct omap_mmc_conf *minfo = dev->platform_data;

	if (twl4030_get_gpio_datain(minfo->switch_pin))
		return sprintf(buf, "%s\n", "open");
	else
		return sprintf(buf, "%s\n", "Closed");
	return 0;
}

/* 
 * sysfs function to enable/disable card detect feature for MMC1
 */
ssize_t set_mmc_carddetect(struct device *dev, struct device_attribute
				   *attr, const char *buf, size_t count)
{
	struct omap_mmc_conf *minfo = dev->platform_data;
	char cmd[25];
	int i = 0;

	if (count < 6) {
		printk(KERN_WARNING "Invalid string\n");
		return count;
	}

	while (buf[i] != ' ' && buf[i] != '\n' && i < count) {
		cmd[i] = buf[i];
		i++;
	}
	cmd[i] = '\0';
	i++;

	if (!strcmp(cmd, "Enable")) {
		enable_irq(TWL4030_GPIO_IRQ_NO(minfo->switch_pin));
	} else if (!strcmp(cmd, "Disable")) {
		disable_irq(TWL4030_GPIO_IRQ_NO(minfo->switch_pin));
	} else {
		dev_dbg(dev, "Unrecognized string\n");
		dev_dbg(dev, "Usage:\n");
		dev_dbg(dev, "echo Enable >"
			"/sys/devices/platform/hsmmc-omap/"
			"mmc_card_detect\n");
		dev_dbg(dev, "echo Disable >"
			"/sys/devices/platform/hsmmc-omap/mmc_card_detect\n");
	}

	return count;
}
#ifdef CONFIG_PM
int mask_carddetect_int(int slot)
{
	u8 reg = 0, ret = 0;
	int val;

	if (slot == mmc_slot1) 
       		val = GPIO_0_BIT_POS;
	else
		val = GPIO_1_BIT_POS;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg,
					TWL_GPIO_IMR1A);
	if (ret) {
		printk(KERN_ERR "Error reading T2 GPIO IMR1A\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, (reg | val),
					TWL_GPIO_IMR1A);
	if (ret) {
		printk(KERN_ERR "Error writing  T2 GPIO IMR1A\n");
		return ret;
	}

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg,
				  	TWL_GPIO_ISR1A);
	if (ret) {
		printk(KERN_ERR "Error reading T2 GPIO ISR1A\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, (reg | val),
				   	TWL_GPIO_ISR1A);
	if (ret) {
		printk(KERN_ERR "Error writing T2 GPIO ISR1A\n");
		return ret;
	}

	return ret;
}

int unmask_carddetect_int(int slot)
{
	u8 reg = 0, ret = 0;
	int val;

	if (slot == mmc_slot1) 
       		val = GPIO_0_BIT_POS;
	else
		val = GPIO_1_BIT_POS;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg,
					TWL_GPIO_IMR1A);
	if (ret) {
		printk(KERN_ERR "Error reading T2 GPIO IMR1A\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, (reg & ~val),
				   	TWL_GPIO_IMR1A);
	if (ret) {
		printk(KERN_ERR "Error writing T2 GPIO IMR1A\n");
		return ret;
	}

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &reg,
				  	TWL_GPIO_ISR1A);
	if (ret) {
		printk(KERN_ERR "Error reading T2 GPIO ISR1A\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, (reg & ~val),
				   	TWL_GPIO_ISR1A);
	if (ret) {
		printk(KERN_ERR "Error writing T2 GPIO ISR1A\n");
		return ret;
	}

	return ret;
}

#endif

int switch_power_mode(int power_mode)
{
	int ret = 0;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, P1_DEV_GRP,
						VMMC1_DEV_GRP);
	if (ret != 0) {
		printk(KERN_ERR
			       "Configuring MMC1 device group failed\n");
		return -1;
	}

	if (power_mode == 0) {
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, VSEL_18V,
					 	VMMC1_DEDICATED);
		if (ret != 0) {
			printk(KERN_ERR
			       "Configuring MMC1 dedicated failed %x\n", ret);
			return -1;
		}
	} else {
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, VSEL_3V,
					 	VMMC1_DEDICATED);
		if (ret != 0) {
			printk(KERN_ERR
			       "Configuring MMC1 dedicated failed %x\n", ret);
			return -1;
		}
	}

	return 0;
}

EXPORT_SYMBOL(enable_mmc_power);
EXPORT_SYMBOL(disable_mmc_power);
EXPORT_SYMBOL(setup_mmc_carddetect_irq);
EXPORT_SYMBOL(mmc_omap_show_cover_switch);
EXPORT_SYMBOL(set_mmc_carddetect);
EXPORT_SYMBOL(switch_power_mode);
#ifdef CONFIG_PM
EXPORT_SYMBOL(mask_carddetect_int);
EXPORT_SYMBOL(unmask_carddetect_int);
#endif
