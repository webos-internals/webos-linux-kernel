/*
 * linux/arch/arm/mach-omap3pe/board-sirloin-3430-hsmmc.c
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

/*
 *	power_mode:
 *
 *	0 - OFF
 *	1 - 1.8V
 *	2 - 3.0V
 */
static int 
board_mmc1_power_enable( void *mmc, int power_mode )
{
	u32 reg;

//	printk("%s: mode = %d\n", __func__, power_mode );
//	printk("%s: PBIAS_1   = 0x%08x\n", __func__, CONTROL_PBIAS_1);
//	printk("%s: WKUP_CTRL = 0x%08x\n", __func__, CONTROL_WKUP_CTRL);

	if( power_mode ) {
		// VCC of MMC1 slot is fixed at 1.8V
		// there is no way to change that
		if( power_mode != 1 ) {
		    panic ("MMC1: unxpected power setting\n" );
		}

		// Mark VDDS as unstable
		reg = CONTROL_PBIAS_1;
		reg &= ~(1 << 1); // 4-bit
#ifndef CONFIG_MACH_SIRLOIN_3630
		reg &= ~(1 << 9); // 8-bit - 34XX only
#endif		
		CONTROL_PBIAS_1 = reg;

		// set PBIAS VMODE to 1.8V
		reg = CONTROL_PBIAS_1;
		reg &= ~(1 << 0);  // 4-bit
#ifndef CONFIG_MACH_SIRLOIN_3630
		reg &= ~(1 << 8);  // 8-bit - 34XX only
#endif		
		CONTROL_PBIAS_1 = reg;
		
		mdelay(1);

		// Mark VDDS as stable
		reg = CONTROL_PBIAS_1;
		reg |= (1 << 1);  // 4-bit
#ifndef CONFIG_MACH_SIRLOIN_3630
		reg |= (1 << 9);  // 8-bit - 34XX only
#endif		
		CONTROL_PBIAS_1 = reg;

//		printk ( KERN_INFO "MMC1: configure PBIAS for 1.8V (0x%08x)\n",
//			            CONTROL_PBIAS_1 );
	}
	else {
		// Mark VDDS as unstable
		reg = CONTROL_PBIAS_1;
		reg &= ~(1 << 1); // 4-bit
#ifndef CONFIG_MACH_SIRLOIN_3630
		reg &= ~(1 << 9); // 8-bit - 34XX only
#endif	
		CONTROL_PBIAS_1 = reg;
	}
	return 0;
}

static int 
board_wifi_slot_power_enable( void *mmc, int power_mode )
{
	int rc = 0;
	
	BUG_ON( power_mode && power_mode != 1 );
#ifdef CONFIG_MACH_SIRLOIN_3630
	rc = board_mmc1_power_enable ( mmc, power_mode );
#else
	/* No PBIAS on mmc2 */
#endif
	return rc;
}

static int 
board_mmc_slot_power_enable( void *mmc, int power_mode )
{
	int rc = 0;
	BUG_ON( power_mode && power_mode != 1 );
#ifdef CONFIG_MACH_SIRLOIN_3630
	/* No PBIAS on mmc2 */
#else
	rc = board_mmc1_power_enable ( mmc, power_mode );
#endif
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
#ifdef CONFIG_MACH_SIRLOIN_3630
	return board_rescan_slot (1);
#else
	return board_rescan_slot (0);
#endif	
}
EXPORT_SYMBOL(board_rescan_mmc_slot);

/*
 *  An API to trigger wifi slot rescan
 */
int 
board_rescan_wifi_slot ( void )
{
#ifdef CONFIG_MACH_SIRLOIN_3630
	return board_rescan_slot (0);
#else
	return board_rescan_slot (1);
#endif	
}
EXPORT_SYMBOL(board_rescan_wifi_slot);


/* 
 * sysfs function to trigger card detect
 */
ssize_t 
board_mmc_cd_store( struct device *dev, 
                    struct device_attribute *attr, 
                    const char *buf, size_t count)
{
	int i = 0;

	/* skip leading white spaces */
	while( i < count && isspace(buf[i])) {
		i++;
	}
	if( count - i >= 6) {
		if( strncmp( buf+i, "Rescan", 6) == 0) {
			// Initiate slot rescan
			board_rescan_dev ( dev );
			return count;    
		} 
	}
	return count;
}
DEVICE_ATTR(mmc_card_detect, S_IWUSR, NULL, board_mmc_cd_store );

static int 
board_cd_init( struct mmc_host *mmc )
{
	if( device_create_file( mmc_dev(mmc), &dev_attr_mmc_card_detect) < 0) {
		dev_err( mmc_dev(mmc), 
		        "unable to create card detect sysfs attribute\n");
	}
	board_save_mmc_ptr ( mmc_dev(mmc), mmc );
	
	return 0;
}

static int
board_cd_free( struct mmc_host *mmc )
{
	board_save_mmc_ptr ( mmc_dev(mmc), NULL );
	device_remove_file ( mmc_dev(mmc), &dev_attr_mmc_card_detect);
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

#define MMC_REMOVAL_DELAY_MS 1200

static int 
board_mmc_shutdown ( void *mmc )
{
	int removal_delay_ms = MMC_REMOVAL_DELAY_MS;
	
	if ( removal_delay_ms ) {
		msleep(removal_delay_ms);
	}

	return 0;
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
#ifdef CONFIG_MMC
#ifdef CONFIG_MACH_SIRLOIN_3630
struct omap_mmc_config board_mmc_config __initdata = {
	.mmc [0] = {  // is used for mmc NAND
		.enabled    =  1,  // enable
		.wire4      =  1,  // 4 wires
		.wp_pin     = -1,
		.power_pin  = -1,
		.switch_pin = -1,
		.host_dma_ch=  1,  // just 1 channel
		.host_caps  =  MMC_CAP_4_BIT_DATA,
		.host_ocr   =  MMC_VDD_165_195,
		.host_fmax  =  26000000,
		.host_retry_max   =  0,
		.host_cmd52_poll  =  1, // use poll mode
		
		.board_power_mode =  board_wifi_slot_power_enable,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
	.mmc [1] = {  // is used for WiFi
		.enabled    =  1,  // enable
		.wire4      =  1,  // 4 wires
		.wp_pin     = -1,
		.power_pin  = -1,
		.switch_pin = -1,
		.host_dma_ch=  8,  // just 1 channel
		.host_caps  =  MMC_CAP_8_BIT_DATA,
		.host_ocr   =  MMC_VDD_165_195,
		.host_fmax  =  48000000,
		.host_retry_max   =  0,
		.board_power_mode =  board_mmc_slot_power_enable,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_shutdown   =  board_mmc_shutdown,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
};
#else
struct omap_mmc_config board_mmc_config __initdata = {
	.mmc [0] = {  // is used for mmc NAND
		.enabled    =  1,  // enabled 
		.wire4      =  1,  // 4 wires
		.wp_pin     = -1,  // no write protect
		.power_pin  = -1,  // no power pin
		.switch_pin = -1,  // no CD switch
		.host_dma_ch=  8,  // up to 8 channels
		.host_caps  =  MMC_CAP_8_BIT_DATA,
		.host_fmax  =  48000000,
		.host_ocr   =  MMC_VDD_165_195,
		.host_retry_max   = 20,
		.board_power_mode =  board_mmc_slot_power_enable,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_shutdown   =  board_mmc_shutdown,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
	.mmc [1] = {  // is used for WiFi
		.enabled    =  1,  // enable
		.wire4      =  1,  // 4 wires
		.wp_pin     = -1,
		.power_pin  = -1,
		.switch_pin = -1,
		.host_dma_ch=  1,  // just 1 channel
		.host_caps  =  MMC_CAP_4_BIT_DATA,
		.host_ocr   =  MMC_VDD_165_195,
		.host_fmax  =  26000000,
		.host_retry_max   =  0,
		.host_cmd52_poll  =  1, // use poll mode
		.board_power_mode =  board_wifi_slot_power_enable,
		.board_probe      =  board_mmc_probe,
		.board_remove     =  board_mmc_remove,
		.board_suspend    =  board_mmc_suspend,
		.board_resume     =  board_mmc_resume,
	},
};
#endif // CONFIG_MACH_SIRLOIN_3630
#endif // CONFIG_MMC



