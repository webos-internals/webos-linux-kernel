/*
 * linux/arch/arm/mach-omap2/board-joplin-3430-hsmmc.c
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
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/mmc/host.h>

#include <asm/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/mux.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/gpio.h>
#include <asm/io.h>


#undef  MODDEBUG
// #define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)	printk(args)
#else
#define PDBG(args...)   
#endif

#define P1_DEV_GRP		0x20
#define LDO_CLR         	0x00
#define VSEL_S2_CLR     	0x40


#ifndef  MODDEBUG
#define hsmmc_pwr_check_state()
#else
static void 
hsmmc_pwr_check_state(void)
{
	u8  vsel;
	u8  dgrp;

	// mmc1
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &dgrp, TWL4030_VMMC1_DEV_GRP);
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &vsel, TWL4030_VMMC1_DEDICATED);
	PDBG(KERN_INFO "VMMC1=0x%0x (grp=0x%0x)\n", vsel, dgrp );

	// mmc2
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &dgrp, TWL4030_VMMC2_DEV_GRP);
	(void) twl4030_i2c_read_u8(  TWL4030_MODULE_PM_RECEIVER,
	                           &vsel, TWL4030_VMMC2_DEDICATED);
	PDBG(KERN_INFO "VMMC2=0x%0x (grp=0x%0x)\n", vsel, dgrp );

	// vaux1
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &dgrp, TWL4030_VAUX1_DEV_GRP);
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &vsel, TWL4030_VAUX1_DEDICATED);
	PDBG(KERN_INFO "VAUX1=0x%0x (grp=0x%0x)\n", vsel, dgrp );

	// vaux2
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &dgrp, TWL4030_VAUX2_DEV_GRP);
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &vsel, TWL4030_VAUX2_DEDICATED);
	PDBG(KERN_INFO "VAUX2=0x%0x (grp=0x%0x)\n", vsel, dgrp );

	// vaux3
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &dgrp, TWL4030_VAUX3_DEV_GRP);
	(void) twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                           &vsel, TWL4030_VAUX3_DEDICATED);
	PDBG(KERN_INFO "VAUX3=0x%0x (grp=0x%0x)\n", vsel, dgrp );

	// aux3 should be set to 1.8V 
}
#endif

/*
 * Enable power to MMC controller from T2 for slot 1
 */
static int 
board_mmc_enable_power_slot1( int mode )
{
	int ret = 0;
	int vsel, val;

	PDBG( KERN_INFO "%s:\n", __FUNCTION__);

	// Mark VDDS as unstable
	CONTROL_PBIAS_1 &= ~(1 << 1); 

	// set pbias vmode
	if( mode == 2 ) {
		vsel = TWL_VMMC1_3P00;
		CONTROL_PBIAS_1 |=  1; // 3.0V
	}
	if( mode == 1 ) {
		vsel = TWL_VMMC1_1P85;
		CONTROL_PBIAS_1 &= ~1; // 1.8V
	}

	/* Enable SLOT1 and Power up */
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            P1_DEV_GRP, TWL4030_VMMC1_DEV_GRP);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC1 device group failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            vsel, TWL4030_VMMC1_DEDICATED);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC1 dedicated failed\n");
		return ret;
	}

	msleep (10);

	// Mark VDDS as stable
	CONTROL_PBIAS_1 |= (1 << 1);

	msleep (1);

	/* check if voltage matches */

	val = CONTROL_PBIAS_1;
	if( CONTROL_PBIAS_1 & 0x08 ) {
		printk ( KERN_ERR "MMC1: PBIAS voltage mismatch (0x%08x)\n",
			 CONTROL_PBIAS_1 );
		CONTROL_PBIAS_1 &= ~(1 << 1);
		return -EINVAL;
	}

	printk ( KERN_INFO "MMC1: configure PBIAS for %sV (0x%08x)\n",
		 (vsel == TWL_VMMC1_3P00) ? "3.0" : "1.8",
		 CONTROL_PBIAS_1 );

	return 0;
}

static int 
board_mmc_enable_power_slot2( void )
{
	int ret = 0;

	PDBG( KERN_INFO "%s:\n", __FUNCTION__);

	/* Enable SLOT2,  Power up vmmc2 (3V) and vaux3 (1.8V) */
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            P1_DEV_GRP, TWL4030_VMMC2_DEV_GRP);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC2 device group failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            TWL_VMMC1_3P00, TWL4030_VMMC2_DEDICATED);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC2 dedicated failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            P1_DEV_GRP, TWL4030_VAUX3_DEV_GRP);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 device group failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            TWL_VAUX3_1P80, TWL4030_VAUX3_DEDICATED);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 dedicated failed\n");
		return ret;
	}

	return ret;
}

/*
 * Disable power to MMC controller from T2. Slot 1 
 */
static int 
board_mmc_disable_power_slot1(void)
{
	int ret = 0;

	PDBG( KERN_INFO "%s:\n", __FUNCTION__);

	// Mark VDDS as unstable
	CONTROL_PBIAS_1 &= ~(1 << 1); 

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 
	                           LDO_CLR, TWL4030_VMMC1_DEV_GRP );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC1 dev grp failed\n");
		return ret;
	}
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VSEL_S2_CLR, TWL4030_VMMC1_DEDICATED );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC1 dedicated failed\n");
		return ret;
	}

	return ret;
}


/*
 * Disable power to MMC controller from T2. Slot 2
 */
static int 
board_mmc_disable_power_slot2(void)
{
	int ret = 0;

	PDBG( KERN_INFO "%s:\n", __FUNCTION__ );

	/* Disable MMC SLOT2 and Power it off */
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 
	                            LDO_CLR, TWL4030_VMMC2_DEV_GRP );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC1 dev grp failed\n");
		return ret;
	}
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VSEL_S2_CLR, TWL4030_VMMC2_DEDICATED );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC1 dedicated failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER, 
	                            LDO_CLR, TWL4030_VAUX3_DEV_GRP );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 dev grp failed\n");
		return ret;
	}
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VSEL_S2_CLR, TWL4030_VAUX3_DEDICATED );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 dedicated failed\n");
		return ret;
	}

	return ret;
}


/*
 *   power_mode:
 *
 *      0 - OFF
 *      1 - 1.8V
 *      2 - 3.0V
 */
static int
board_mmc_power_slot1 ( void *mmc, int power_mode  )
{
	int rc;

	if( power_mode ) {
		rc = board_mmc_enable_power_slot1 ( power_mode );
	} else {
		rc = board_mmc_disable_power_slot1();
	}

	hsmmc_pwr_check_state ();

	return rc;
}

static int
board_mmc_power_slot2 ( void *mmc, int power_mode )
{
	int rc;

	if( power_mode ) {
		rc = board_mmc_enable_power_slot2 ();
	} else {
		rc = board_mmc_disable_power_slot2();
	}

	hsmmc_pwr_check_state ();

	return rc;
}

/* saved host pointers */
static struct mmc_host *g_host[2] = {0};

static int
board_get_dev_id ( struct device *dev )
{
	struct platform_device *pdev = 
		container_of (dev, struct platform_device, dev );

	return pdev->id;
}

/*
 *   Some joplin3430 boards are hardwired in a such way that 
 *   3.0V is not really supported. This code verifies if 3.0V is 
 *   really supported by the board.
 */
static void
board_checkup_ocr (struct mmc_host * mmc)
{
	int rc;
	int id = board_get_dev_id(mmc_dev(mmc));

	if( id != 1 ) 
		return; // Only MMC1 is affected

	if((mmc->ocr_avail & (MMC_VDD_29_30 | MMC_VDD_30_31)) == 0 ) {
		return; // 3.0V is not supported, so we do not care
	}

	// set 3.0V 
	rc = board_mmc_enable_power_slot1( 2 );
	if( rc == -EINVAL ) { 
		mmc->ocr_avail &= MMC_VDD_165_195; // only 1.8 is supported
	}
	board_mmc_disable_power_slot1();
	
	return;
}


static void
board_save_mmc_ptr( struct device *dev, struct mmc_host *mmc  )
{
	int slot = board_get_dev_id(dev)-1;
	if( slot >= 0 && slot < ARRAY_SIZE(g_host)) {
		g_host[slot] = mmc;
	}
	return;
}

static int
board_rescan_slot ( int slot )
{
	if( slot < 0 || slot >= ARRAY_SIZE(g_host)) 
		return -EINVAL;

	if( g_host[slot] == NULL )
		return -ENODEV;

	mmc_detect_change ( g_host[slot], 0 );

	return 0;
}

static int
board_rescan_dev ( struct device *dev )
{
	struct platform_device *pdev = 
		container_of (dev, struct platform_device, dev );

	return board_rescan_slot ( pdev->id-1 );
}

/*
 *  An API to trigger mmc slot rescan
 */
int 
board_rescan_mmc_slot ( void )
{
	return board_rescan_slot (0);
}
EXPORT_SYMBOL(board_rescan_mmc_slot);

/*
 *  An API to trigger wifi slot rescan
 */
int 
board_rescan_wifi_slot ( void )
{
	return board_rescan_slot (1);
}
EXPORT_SYMBOL(board_rescan_wifi_slot);


/* 
 * Sysfs entry function to show card state
 */
static ssize_t 
board_mmc_switch_show( struct device *dev, 
                       struct device_attribute *attr, char *buf)
{
	struct omap_mmc_conf *minfo = dev->platform_data;

	if( minfo->switch_pin < 0 )
		return snprintf(buf, PAGE_SIZE, "%s\n", "unavailable");

	if( gpio_get_value ( irq_to_gpio(minfo->switch_pin)))
		return snprintf(buf, PAGE_SIZE, "%s\n", "open");
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", "closed");

	return 0;
}

/* 
 * sysfs function to enable/disable/trigger card detect
 */
static ssize_t 
board_mmc_cd_store( struct device *dev, 
                    struct device_attribute *attr, 
                    const char *buf, size_t count )
{
//	struct omap_mmc_conf *minfo = dev->platform_data;
	int i = 0;

	/* skip leading white spaces */
	while( i < count && isspace(buf[i])) {
		i++;
	}

	if( count - i >= 6) {
		if( strncmp( buf+i, "Rescan", 6) == 0) 
		{   // Initiate slot rescan
			board_rescan_dev ( dev );
			return count;
		} 
	}
	if( count - i >= 6) {
		if( strncmp( buf+i, "Enable", 6) == 0) {
			// enable pin detect  interrupt
//			MMC card detect IRQ handling disabled on purpose.
//			enable_irq ( minfo->switch_pin ); 
			return count;
		}
	}

	if( count - i >= 7) {
		if( strncmp( buf+i, "Disable", 7) == 0) {
			// disable pin detect interrupt
//			MMC card detect IRQ handling disabled on purpose.
//			disable_irq( minfo->switch_pin );
			return count;
		} 
	}
	return count;
}

DEVICE_ATTR(mmc_cover_switch, S_IRUGO, board_mmc_switch_show, NULL );
DEVICE_ATTR(mmc_card_detect , S_IWUSR, NULL, board_mmc_cd_store    );

/*
 * Interrupt service routine for handling card insertion and removal
 */
static irqreturn_t 
board_mmc_cd_irq( int irq, void *dev_id )
{
	struct mmc_host *mmc = (struct mmc_host *)dev_id;
	mmc_detect_change ( mmc, 10 );
	return IRQ_HANDLED;
}

static int 
board_cd_init( struct mmc_host *mmc )
{
	int rc;
	struct omap_mmc_conf *minfo =  mmc_dev(mmc)->platform_data;

	if( device_create_file( mmc_dev(mmc), &dev_attr_mmc_cover_switch) < 0){
		dev_err( mmc_dev(mmc), 
		        "unable to create mmc switch sysfs attribute\n");
	}

	if( device_create_file( mmc_dev(mmc), &dev_attr_mmc_card_detect) < 0) {
		dev_err( mmc_dev(mmc), 
		        "unable to create card detect sysfs attribute\n");
	}

	if( minfo->switch_pin == -1 )
		goto Done;

	rc = gpio_request( irq_to_gpio ( minfo->switch_pin ), "SD/MMC CD" );
	if( rc < 0 )
		return rc;

	rc = request_irq ( minfo->switch_pin, board_mmc_cd_irq, 
	         IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, 
		 mmc_hostname(mmc),  mmc );
	if( rc < 0 ) {
		dev_err ( mmc_dev(mmc), "unable to grab CD IRQ\n" );
		gpio_free ( irq_to_gpio ( minfo->switch_pin ));
		return rc;
	}

Done:
	board_checkup_ocr ( mmc );

	board_save_mmc_ptr ( mmc_dev(mmc), mmc );

	return 0;
}

static int
board_cd_free( struct mmc_host *mmc )
{
	struct omap_mmc_conf *minfo =  mmc_dev(mmc)->platform_data;

	board_save_mmc_ptr ( mmc_dev(mmc), NULL );

	device_remove_file( mmc_dev(mmc), &dev_attr_mmc_cover_switch );
	device_remove_file( mmc_dev(mmc), &dev_attr_mmc_card_detect  );

	if( minfo->switch_pin != -1 ) {
		free_irq ( minfo->switch_pin, mmc );
		gpio_free ( irq_to_gpio ( minfo->switch_pin ));
	}
	return 0;
}

/* 
 *  Driver callbacks
 */
static int 
board_mmc_probe ( void *mmc )
{
	return board_cd_init ((struct mmc_host *) mmc );
}

static int 
board_mmc_remove ( void *mmc )
{
	return board_cd_free ((struct mmc_host *) mmc );
}

static int 
board_mmc_suspend ( void *mmc )
{
	return 0; 
}

static int 
board_mmc_resume ( void *mmc )
{
	return 0; 
}

/*
 *  MMC
 */
struct omap_mmc_config  board_mmc_config __initdata = {
	.mmc[0] = {             // is used for moviNAND
		.enabled    =  1,   // enabled 
		.wire4      =  1,   // 4 wires
		.wp_pin     = -1,   // no write protect
		.power_pin  = -1,   // no power pin
		.switch_pin =  OMAP_GPIO_IRQ(JOPLIN_SD_CD_GPIO),
		.host_dma_ch=  8,   // up to 8 channels
		.host_caps  =  MMC_CAP_4_BIT_DATA,
		.host_ocr   =  MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_165_195,
		.host_fmax  =  24000000,
		.host_retry_max   = 20,
		.board_power_mode =  board_mmc_power_slot1,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
	.mmc[1] = {             // is used for WiFi
		.enabled    =  1,   // enable
		.wire4      =  1,   // 4 wires
		.wp_pin     = -1,
		.power_pin  = -1,
		.switch_pin = -1,
		.host_dma_ch=  1,   // just 1 channel
		.host_caps  =  MMC_CAP_4_BIT_DATA,
		.host_ocr   =  MMC_VDD_165_195,
		.host_fmax  =  26000000,
		.host_retry_max   =  0,
		.board_power_mode =  board_mmc_power_slot2,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
};


