/*
 * drivers/mtd/nand/omap2-nand.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * Based on omap-nand-flash.c
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

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/flash.h>
#include <asm/arch/gpmc.h>

#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/arch/hardware.h>
#include <asm/arch/nand.h>

#define  DRIVER_NAME    "omap2_nand"
#define	 DRIVER_DESC    "Omap2 NAND driver"

#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

/* IO window size */
#define	 NAND_IO_SIZE	SZ_4K

/* Some GPMC related constants */
#define  GPMC_FIFO_HAS_SPACE   (0x1 <<  0)

/* ECC */
#define  GPMC_ECC_CLEAR        (0x1 <<  8)
#define  GPMC_ECC_POINTER_1    (0x1)
#define  GPMC_ECC_16B          (0x1 <<  7)
#define  GPMC_ECC_ENABLE       (0x1)

#define  GPMC_ECC_CS_SHIFT     (1)
#define  GPMC_ECC_SIZE0_SHIFT  (12)
#define  GPMC_ECC_SIZE1_SHIFT  (22)

/* Cs specifig */
#define  GPMC_CS_ENABLED_MASK  (0x1 <<  6)
#define  GPMC_CS_DEV_TYPE(v)   (((v) >> 10) & 0x3)
#define  GPMC_CS_DEV_SIZE(v)   (((v) << 12) & 0x3)
#define  GPMC_CS_WAIT_PIN(v)   (((v) >> 16) & 0x3)


/*  */
#ifdef   CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL }; 
#endif

/*  */
struct nand_info
{
    int    gpmc_cs;
    u32    wait_mask;
    u32    wait_active;
    struct mtd_info            mtd;
    struct nand_hw_control     controller;
    struct nand_chip           nand;
    struct nand_platform_data *pdata;
    struct mtd_partition      *parts;
    struct platform_device    *pdev;
    unsigned long              gpmcBase;   // virt addr of gpmc base
    unsigned long              gpmcCsBase; // virt addr of gpmc cs
};

/* Define a BB pattern we are using on Joplin NAND */
static uint8_t scan_ff_pattern[] = { 0xff, 0xff };
static struct  nand_bbt_descr memorybased_8bit = {
	.options = 0,
	.offs    = 0,
	.len     = 1,
	.pattern = scan_ff_pattern
};
static struct  nand_bbt_descr memorybased_16bit = {
	.options = 0,
	.offs    = 0,
	.len     = 2,
	.pattern = scan_ff_pattern
};

/* new oob placement block for use with hardware ecc generation */ 
static struct nand_ecclayout ecclayout16 = {
    .eccbytes = 12,
    .eccpos = {
        2, 3, 4 ,5 ,6 ,7 ,8, 9, 
        10, 11, 12, 13 },
    .oobfree = {
        {.offset = 14,
         .length = 50}}
};

static struct nand_ecclayout ecclayout8 = {
    .eccbytes = 12,
    .eccpos = {
        1, 2, 3, 4, 5 ,6, 7, 8,
	    9, 10, 11, 12 },
    .oobfree = {
        {.offset = 14,
         .length = 50}}
};


/*
 * omap2_hwcontrol - This function is to finally write the command or 
 * the nand address in the GPMC registers. If the ctrl is CLE it is for 
 * sending command to NAND and if it is ALE, it is for sending addresses.
 * @mtd:  MTD device structure
 * @cmd:  command to be sent to NAND
 * @ctrl: Control operation deciding whether you send addr or cmd.
 */
static void 
omap2_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl) 
{
    struct nand_info *info = container_of ( mtd, struct nand_info, mtd );

    switch (ctrl) 
    {
        case NAND_CTRL_CHANGE | NAND_CTRL_CLE:
            info->nand.IO_ADDR_W = (void __iomem *)(info->gpmcCsBase + GPMC_CS_NAND_COMMAND);
            info->nand.IO_ADDR_R = (void __iomem *)(info->gpmcCsBase + GPMC_CS_NAND_DATA);
            break;
        
        case NAND_CTRL_CHANGE | NAND_CTRL_ALE:
            info->nand.IO_ADDR_W = (void __iomem *)(info->gpmcCsBase + GPMC_CS_NAND_ADDRESS);
            info->nand.IO_ADDR_R = (void __iomem *)(info->gpmcCsBase + GPMC_CS_NAND_DATA);
            break;

        case NAND_CTRL_CHANGE | NAND_NCE:
            info->nand.IO_ADDR_W = (void __iomem *)(info->gpmcCsBase + GPMC_CS_NAND_DATA);
            info->nand.IO_ADDR_R = (void __iomem *)(info->gpmcCsBase + GPMC_CS_NAND_DATA);
            break;
    }

    if( cmd != NAND_CMD_NONE )
        __raw_writeb( cmd, info->nand.IO_ADDR_W );
 }

/*
 * omap2_hwecc - Contols the hardware ecc functionality
 */
static void 
omap2_nand_hwecc ( struct mtd_info *mtd , int mode ) 
{
    u32 ctrl, cfg, size;
    struct nand_info *info = container_of(mtd, struct nand_info, mtd);
   
    switch (mode) {
        case NAND_ECC_READ:
        case NAND_ECC_WRITE:
            break;
        default:
            BUG();
            break;
    }

    // clear all results
    ctrl = GPMC_ECC_CLEAR | GPMC_ECC_POINTER_1;

    // config sizes
    size  = ((info->nand.ecc.size >> 1) - 1) << GPMC_ECC_SIZE0_SHIFT;
    size |= ((info->nand.ecc.size >> 1) - 1) << GPMC_ECC_SIZE1_SHIFT;
    size |= 0xFF; // select all SIZE1

    // set up chip select
    cfg  = ( info->gpmc_cs << GPMC_ECC_CS_SHIFT) | GPMC_ECC_ENABLE;
    if( info->nand.options & NAND_BUSWIDTH_16 )
        cfg |= GPMC_ECC_16B;

    __raw_writel ( ctrl, info->gpmcBase + GPMC_ECC_CONTROL );
    __raw_writel ( size, info->gpmcBase + GPMC_ECC_SIZE_CONFIG );
    __raw_writel ( cfg , info->gpmcBase + GPMC_ECC_CONFIG  );
} 

/*
 *  omap2_nand_calculate_ecc - Generate non-inverted ECC bytes.
 */
static int 
omap2_nand_calculate_ecc ( struct mtd_info *mtd, 
                            const  u_char *dat, 
                            u_char *ecc_code ) 
{
    u32 eccval;
    struct nand_info *info = container_of(mtd, struct nand_info, mtd);

    /* read result */
    eccval = __raw_readl ( info->gpmcBase + GPMC_ECC1_RESULT );

	/* convert eccval to 3 bytes */
	ecc_code[0] = ((eccval) & 0xff);
	ecc_code[1] = ((eccval >> 16) & 0xff);
	ecc_code[2] = ((eccval >> 20) & 0xf0) | 
	              ((eccval >>  8) & 0x0f);
    return 0;
} 

/*
 * omap2_nand_correct_data - Detect and correct bit error(s)
 */
static int 
omap2_nand_correct_data( struct mtd_info *mtd, u_char *dat,
			              u_char *read_ecc,u_char *calc_ecc)
{
	u16 read_ecc_even;
	u16 read_ecc_odd;
	u16 calc_ecc_even;
	u16 calc_ecc_odd;
	u16 result;
	u8  bitmask;
	u16 bit_index;
	u16 byte_index;
    
	/* extract even and odd parities */
	read_ecc_even = read_ecc[0] | ((read_ecc[2] << 8) & 0x0f00);
	calc_ecc_even = calc_ecc[0] | ((calc_ecc[2] << 8) & 0x0f00);
	read_ecc_odd  = read_ecc[1] | ((read_ecc[2] << 4) & 0x0f00);
	calc_ecc_odd  = calc_ecc[1] | ((calc_ecc[2] << 4) & 0x0f00);
	result = read_ecc_even ^ read_ecc_odd ^ calc_ecc_even ^ calc_ecc_odd;
	if (result == 0)
		return 0;
		
	if (result == 0x0fff) {
		/* correct 1 bit error */
		result = read_ecc_odd ^ calc_ecc_odd;
		bit_index  = result & 0x07;
		byte_index = result >> 3;
		bitmask = 1 << bit_index;
		/* flip bit */
		if (dat[byte_index] & bitmask)
			dat[byte_index] &= ~bitmask;
		else
			dat[byte_index] |= bitmask;
		return 0;
	}
	return -1;
}

/*
 *  write 8 bit buffer
 */
static void 
omap2_nand_write_buf8( struct mtd_info *mtd,
                        const unsigned char *buf, int len )
{
	int i;
    struct nand_info *info = container_of( mtd, struct nand_info, mtd );

	for (i = 0; i < len; i++) {
	    while (!(__raw_readw(info->gpmcBase + GPMC_STATUS) & GPMC_FIFO_HAS_SPACE));
	    __raw_writeb ( buf[i], info->nand.IO_ADDR_W );
	}
}

/*
 *  write 16 bit buffer
 */
static void 
omap2_nand_write_buf16( struct mtd_info *mtd,
                         const unsigned char *buf, int len )
{
	int  i;
	u16 *p = (u16 *) buf;
    struct nand_info *info = container_of( mtd, struct nand_info, mtd );

	len >>= 1;
	for (i = 0; i < len; i++) {
	    while (!(__raw_readw(info->gpmcBase + GPMC_STATUS) & GPMC_FIFO_HAS_SPACE));
	    __raw_writew ( p[i], info->nand.IO_ADDR_W );
	}
}

/* 
 * omap2_dev_ready - the platform specific dev_ready function
 */
static int 
omap2_nand_dev_ready ( struct mtd_info *mtd ) 
{
    struct nand_info *info = container_of(mtd, struct nand_info, mtd );
    u32 val = __raw_readl ( info->gpmcBase + GPMC_STATUS );
    if((val & info->wait_mask) == info->wait_active) 
        return 0; // busy
    else
        return 1; // ready
} 


/*
 * NAND probe
 */
static int __devinit 
omap2_nand_probe ( struct platform_device *pdev ) 
{
    int err;
    unsigned long val;
    unsigned long phys;
    struct nand_info *info;
    struct omap_nand_platform_data *pdata;

    pdata = pdev->dev.platform_data;
    if( pdata == NULL )
        return -ENODEV;

    /* allocate device info */
    info = kzalloc( sizeof(struct nand_info), GFP_KERNEL );
    if( info == NULL ) 
        return -ENOMEM;

    info->pdev       = pdev;                // attach pdev
    info->gpmc_cs    = pdata->cs;           // set cs
    info->gpmcBase   = (u32) pdata->gpmcBaseAddr; // gpmc base addr
    info->gpmcCsBase = (u32) pdata->gpmcBaseAddr + 
                             GPMC_CS0_BASE + GPMC_CS_SIZE * pdata->cs;

    info->mtd.priv   = &info->nand;
    info->mtd.name   = pdev->dev.bus_id;
    info->mtd.owner  = THIS_MODULE;

    /* 
     * Let's see if specified cs is enabled. 
     * If it is not, we have to set it up from scratch and it is 
     * completely different story.
     */
	val = gpmc_cs_read_reg ( info->gpmc_cs, GPMC_CS_CONFIG7 );
	if( val & GPMC_CS_ENABLED_MASK ) 
	{   // just consider it properly setup 
        int wait_pin;
	
	    // will be the best guess, unless it is exactly known
	    val =  gpmc_cs_read_reg ( info->gpmc_cs, GPMC_CS_CONFIG1 );

        // lets do so sanity check to see if it is configired as NAND 
        if( GPMC_CS_DEV_TYPE(val) != 0x2) {
            // it is not NAND, bail out
	        PDBG ( KERN_ERR "%s: unsupported device type\n", DRIVER_NAME );
            err = -ENODEV;
	        goto out_free_info;
        }

	    // the bus width will be the best guess
	    if( GPMC_CS_DEV_SIZE(val) == 0 ) {
	        pdata->options &= ~NAND_BUSWIDTH_16;
            PDBG( KERN_INFO "%s: bus width=8\n", __FUNCTION__ );
	    }
	    else if( GPMC_CS_DEV_SIZE(val) == 1 ) {
            pdata->options |=  NAND_BUSWIDTH_16;
            PDBG( KERN_INFO "%s: bus width=16\n", __FUNCTION__ );
        }
        else {   
            // unsupported
	        PDBG ( KERN_ERR "%s: unsupported device size\n", DRIVER_NAME );
            err = -ENODEV;
	        goto out_free_info;
        }    

	    // ready pin and active polarity
        wait_pin = GPMC_CS_WAIT_PIN(val);
	    val = __raw_readl ( info->gpmcBase + GPMC_CONFIG );
	    info->wait_mask   = (1 << (wait_pin + 8));
	    info->wait_active = (val & info->wait_mask);
        PDBG( KERN_INFO "%s: wait_pin=%d (active %s)\n", __FUNCTION__, 
                             wait_pin, info->wait_active ? "high" : "low" );

        // DOLATER: there is also write protect
	}
	else
	{   // if chip select is not enabled lets give up for now.
	    PDBG ( KERN_ERR "%s: chip select is not enabled\n", DRIVER_NAME );
	    err = -ENODEV;
	    goto out_free_info;
	}

    /* grab and enable cs */
    err = gpmc_cs_request ( info->gpmc_cs, NAND_IO_SIZE, &phys );
    if( err < 0 ) {
        goto out_free_info;
    }

    /* set up hw control routine */
    info->nand.cmd_ctrl = omap2_nand_hwcontrol;

    /* setup dev ready */
    info->nand.dev_ready  = omap2_nand_dev_ready; 
    info->nand.chip_delay = 0;

    /* setup ecc */
    info->nand.ecc.mode      = NAND_ECC_HW;
    info->nand.ecc.size      = 512;
    info->nand.ecc.bytes     = 3;
    info->nand.ecc.hwctl     = omap2_nand_hwecc;
    info->nand.ecc.calculate = omap2_nand_calculate_ecc;
    info->nand.ecc.correct   = omap2_nand_correct_data;

    /* options */
    info->nand.options  = pdata->options;

    /* Try the default bus width */
    if( info->nand.options & NAND_BUSWIDTH_16 )
    {   // for 16 bit bus 
        info->nand.write_buf  =  omap2_nand_write_buf16;
        info->nand.badblock_pattern = &memorybased_16bit;
        info->nand.ecc.layout       = &ecclayout16;
    } else {
        info->nand.write_buf  =  omap2_nand_write_buf8;
        info->nand.badblock_pattern = &memorybased_8bit;
        info->nand.ecc.layout       = &ecclayout8;
    }

    /* do scan */
    if( nand_scan(&info->mtd, 1) == 0) 
        goto scan_done;

    // nope, does not work
    err = -ENXIO;
    goto out_free_cs;

scan_done: 
     

#ifdef CONFIG_MTD_PARTITIONS
    err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
    if (err > 0)
        add_mtd_partitions(&info->mtd, info->parts, err);
    else if (err < 0 && pdata->parts)
        add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
    else
#endif

    add_mtd_device ( &info->mtd );

    platform_set_drvdata ( pdev, &info->mtd );

    return 0;

out_free_cs:
    gpmc_cs_free ( info->gpmc_cs );
    
out_free_info:
    kfree ( info );

    return err;
}

/*
 *  Remove device
 */
static int 
omap2_nand_remove (struct platform_device *pdev) 
{
    struct mtd_info  *mtd  = platform_get_drvdata ( pdev );
    struct nand_info *info = mtd->priv;

    platform_set_drvdata ( pdev, NULL );
    
    /* Release NAND device, its internal structures and partitions */
    nand_release ( &info->mtd );
    kfree   (&info->mtd );
    
    return 0;
}

static struct platform_driver omap2_nand_driver = {
    .probe  = omap2_nand_probe,
    .remove = omap2_nand_remove,
    .driver = {
        .name  = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init omap2_nand_init(void)
{
    printk(KERN_INFO "Initializing %s\n", DRIVER_DESC );
    return platform_driver_register ( &omap2_nand_driver );
}

static void __exit omap2_nand_exit(void) 
{
    platform_driver_unregister ( &omap2_nand_driver );
}

module_init( omap2_nand_init );
module_exit( omap2_nand_exit );

MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_DESCRIPTION(DRIVER_DESC);

