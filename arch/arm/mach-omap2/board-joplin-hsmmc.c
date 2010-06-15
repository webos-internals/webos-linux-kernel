/*
 * linux/arch/arm/mach-omap2/board-joplin-hsmmc.c
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
#include <asm/arch/twl4030.h>
#include <asm/arch/gpio.h>
#include <asm/arch/control.h>
#include <asm/io.h>


#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)   	printk(args)
#else
#define PDBG(args...)
#endif

#define mmc_slot1       	1
#define mmc_slot2       	2

#define P1_DEV_GRP		0x20

#define VMMC1_DEV_GRP		0x27
#define VMMC1_DEDICATED 	0x2A

#define VMMC2_DEV_GRP		0x2B
#define VMMC2_DEDICATED 	0x2E

#define VAUX1_DEV_GRP   	0x17
#define VAUX1_DEDICATED 	0x1A

#define VAUX2_DEV_GRP   	0x1B
#define VAUX2_DEDICATED 	0x1E

#define VAUX3_DEV_GRP   	0x1F
#define VAUX3_DEDICATED 	0x22

#define VMMC1_VSEL_3V   	0x02
#define VMMC1_VSEL_18V  	0x00

#define VMMC2_VSEL_18V  	0x05
#define VMMC2_VSEL_3V   	0x0B

#define VAUX3_VSEL_18V  	0x01

#define LDO_CLR         	0x00
#define VSEL_S2_CLR     	0x40

#define MMC1_ACTIVE_OVERWRITE (1<<31)

static void hsmmc_pwr_check_state(void)
{
	u8 vsel;
	u8 dgrp;

	// mmc1
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &dgrp, VMMC1_DEV_GRP);
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &vsel, VMMC1_DEDICATED);
	PDBG(KERN_INFO "VMMC1=0x%0x (grp=0x%0x)\n", vsel, dgrp);

	// mmc2
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &dgrp, VMMC2_DEV_GRP);
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &vsel, VMMC2_DEDICATED);
	PDBG(KERN_INFO "VMMC2=0x%0x (grp=0x%0x)\n", vsel, dgrp);

	// vaux1
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &dgrp, VAUX1_DEV_GRP);
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &vsel, VAUX1_DEDICATED);
	PDBG(KERN_INFO "VAUX1=0x%0x (grp=0x%0x)\n", vsel, dgrp);

	// vaux2
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &dgrp, VAUX2_DEV_GRP);
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &vsel, VAUX2_DEDICATED);
	PDBG(KERN_INFO "VAUX2=0x%0x (grp=0x%0x)\n", vsel, dgrp);

	// vaux3
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &dgrp, VAUX3_DEV_GRP);
	(void)twl4030_i2c_read_u8( TWL4030_MODULE_PM_RECEIVER,
	                          &vsel, VAUX3_DEDICATED);
	PDBG(KERN_INFO "VAUX3=0x%0x (grp=0x%0x)\n", vsel, dgrp);
	// aux3 should be set to 1.8V 
}



/*
 * Enable power to MMC controller from T2. Slot 1.
 */
static int
board_mmc_enable_power_slot1( int power_mode )
{
	u32 val;
	u32 vsel;
	int ret = 0;

	if( power_mode == 0 )
		vsel  = VMMC1_VSEL_18V;
	else
		vsel  = VMMC1_VSEL_3V;

	PDBG(KERN_INFO "mmc0: enable power (%s)\n",
	     (vsel == VMMC1_VSEL_3V) ? "3.0V" : "1.8V");

	if( is_sil_rev_equal_to(OMAP2430_REV_ES2_0)) {
		// for ES2.0 we should never request 3V mode
		BUG_ON( vsel == VMMC1_VSEL_3V );
	}

	if( is_sil_rev_greater_than(OMAP2430_REV_ES2_0)) {
		// for ES2.1 there is an errata
		val = omap_ctrl_readl(OMAP243X_CONTROL_DEVCONF1);
		if( vsel == VMMC1_VSEL_18V )
			val &= ~MMC1_ACTIVE_OVERWRITE;
		else
			val |=  MMC1_ACTIVE_OVERWRITE;
		omap_ctrl_writel( val, OMAP243X_CONTROL_DEVCONF1);
	}

	/* For MMC1, Mark vdd as unstable   */
	val  = omap_readl( OMAP2_CONTROL_PBIAS_1 );
	val &= ~(1 << 1); //
	omap_writel( val, OMAP2_CONTROL_PBIAS_1 );

	/* configure pbias */
	val  = omap_readl ( OMAP2_CONTROL_PBIAS_1 );
	if( vsel == VMMC1_VSEL_3V )
	    val |=  1; // 3V
	else    
	    val &= ~1; // 1.8V
	omap_writel( val, OMAP2_CONTROL_PBIAS_1 );

	/* Enable SLOT1 and Power On */
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            P1_DEV_GRP, VMMC1_DEV_GRP );
	if (ret != 0) {
		printk(KERN_ERR "Configuring MMC1 device group failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            vsel, VMMC1_DEDICATED);
	if (ret != 0) {
		printk(KERN_ERR "Configuring MMC1 dedicated failed\n");
		return ret;
	}

	/* 100ms delay required for PBIAS configuration */
	mdelay(100);

	/* mark it stable */
	val  = omap_readl ( OMAP2_CONTROL_PBIAS_1 );
	val |= (1 << 1); // 
	omap_writel( val, OMAP2_CONTROL_PBIAS_1 );

	return 0;
}


static int
board_mmc_enable_power_slot2(void)
{
	int ret = 0;

	/* Enable SLOT2,  Power up vmmc2 (3V) */
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            P1_DEV_GRP, VMMC2_DEV_GRP);
	if( ret != 0 ) {
		printk(KERN_ERR
		       "Configuring MMC2 device group failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VMMC2_VSEL_3V, VMMC2_DEDICATED);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC2 dedicated failed\n");
		return ret;
	}

	/* And VAUX3 (VCC-1.8-WL) */
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            P1_DEV_GRP, VAUX3_DEV_GRP);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 device group failed\n");
		return ret;
	}

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VAUX3_VSEL_18V, VAUX3_DEDICATED);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 dedicated failed\n");
		return ret;
	}

	return 0;
}


/*
 * Disable power to MMC controller from T2. Slot 1.
 */
static int 
board_mmc_disable_power_slot1(void)
{
	int ret = 0;
	u32 val;

	/*  Disable MMC SLOT1 and Power off */

	PDBG(KERN_INFO "mmc0: disable power\n");

	val  = omap_readl ( OMAP2_CONTROL_PBIAS_1 );
	val &= ~(1 << 1); // 
	omap_writel( val, OMAP2_CONTROL_PBIAS_1 );

	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            LDO_CLR, VMMC1_DEV_GRP);
	if (ret != 0) {
		printk(KERN_ERR "Configuring MMC1 dev grp failed\n");
		return ret;
	}
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VSEL_S2_CLR, VMMC1_DEDICATED );
	if (ret != 0) {
		printk(KERN_ERR "Configuring MMC1 dedicated failed\n");
		return ret;
	}
	
	return ret;
}


/*
 * Disable power to MMC controller from T2. Slot 2.
 */
static int 
board_mmc_disable_power_slot2(void)
{
	int ret = 0;

	/* Disable MMC SLOT2 and Power it off */
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            LDO_CLR, VMMC2_DEV_GRP );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC1 dev grp failed\n");
		return ret;
	}
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VSEL_S2_CLR, VMMC2_DEDICATED);
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring MMC2 dedicated failed\n");
		return ret;
	}

	// power off VAUX3 (VCC-1.8-WL)
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            LDO_CLR, VAUX3_DEV_GRP );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 dev grp failed\n");
		return ret;
	}
	ret = twl4030_i2c_write_u8( TWL4030_MODULE_PM_RECEIVER,
	                            VSEL_S2_CLR, VAUX3_DEDICATED );
	if( ret != 0 ) {
		printk(KERN_ERR "Configuring VAUX3 dedicated failed\n");
		return ret;
	}

	return ret;
}

/*
 * 0 = OFF
 * 1 = 1.8V
 * 2 = 3.0V
 */
static int
board_mmc_power_slot1 ( void *mmc, int power_mode  )
{
	int rc;
	
	if( power_mode ) {
		BUG_ON( power_mode != 2 );
		rc = board_mmc_enable_power_slot1 ( power_mode );
	}
	else {
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
		if( power_mode != 1 )
			panic("MMC2: unexpected voltage setting\n" );
		rc = board_mmc_enable_power_slot2();
	}
	else {
		rc = board_mmc_disable_power_slot2();
	}

	hsmmc_pwr_check_state ();

	return rc;
}


/* saved host pointers */
static struct mmc_host *g_host[2] = {0};

static void
board_save_mmc_ptr ( struct device *dev, struct mmc_host *mmc  )
{
	int slot;
	struct platform_device *pdev = 
		container_of (dev, struct platform_device, dev );

	slot = pdev->id-1;
	if( slot >= 0 && slot < ARRAY_SIZE(g_host)) {
		g_host[slot] = mmc;
	}
	return ;
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
		goto Done; // no IRQ to setup

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
struct omap_mmc_config board_mmc_config __initdata = {
	.mmc [0] = {  // is used for normal SD
		.enabled    =  1,  // enabled 
		.wire4      =  1,  // 4 wires
		.wp_pin     = -1,  // no write protect
		.power_pin  = -1,  // no power pin
		.switch_pin =  OMAP_GPIO_IRQ(JOPLIN_SD_CD_GPIO),
		.host_dma_ch=  8,  // up to 8 channels
		.host_caps  =  MMC_CAP_4_BIT_DATA,
		.host_ocr   =  MMC_VDD_29_30 | MMC_VDD_30_31,
		.host_fmax  =  24000000,
		.board_power_mode =  board_mmc_power_slot1,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
	.mmc [1] = {  // is used for WiFi
		.enabled    =  1, // WiFi
		.wire4      =  1,
		.wp_pin     = -1,
		.power_pin  = -1,
		.switch_pin = -1,
		.host_dma_ch=  1, // just 1
		.host_caps  =  MMC_CAP_4_BIT_DATA,
		.host_ocr   =  MMC_VDD_165_195,
		.host_fmax  =  26000000,
		.board_power_mode =  board_mmc_power_slot2,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
};


