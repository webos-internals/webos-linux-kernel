/*
* drivers/mmc/omap_hsmmc.h
*
* Header file for OMAP2430/3430 HSMMC controller.
*
* Copyright (C) 2007 Texas Instruments.
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

#ifndef	DRIVERS_MEDIA_MMC_OMAP_H
#define	DRIVERS_MEDIA_MMC_OMAP_H

#define RSP_TYPE(x)	((x) & ~(MMC_RSP_BUSY|MMC_RSP_OPCODE))
#define OMAP_MMC1_DEVID	1
#define OMAP_MMC2_DEVID 2

#define OMAP_MMC_DATADIR_NONE	0
#define OMAP_MMC_DATADIR_READ	1
#define OMAP_MMC_DATADIR_WRITE	2
#define MMC1_ACTIVE_OVERWRITE (1<<31)

#define OMAP_MMC_MASTER_CLOCK	96000000
#define OMAP_USE_DMA		1
#define MMC1			1
#define MMC2			2

/* HSMMC Host Controller Registers */
#define OMAP_HSMMC_SYSCONFIG	0x0010
#define OMAP_HSMMC_SYSSTATUS	0x0014
#define OMAP_HSMMC_CSRE		0x0024
#define OMAP_HSMMC_SYSTEST	0x0028
#define OMAP_HSMMC_CON		0x002C
#define OMAP_HSMMC_BLK		0x0104
#define OMAP_HSMMC_ARG		0x0108
#define OMAP_HSMMC_CMD		0x010C
#define OMAP_HSMMC_RSP10	0x0110
#define OMAP_HSMMC_RSP32	0x0114
#define OMAP_HSMMC_RSP54	0x0118
#define OMAP_HSMMC_RSP76	0x011C
#define OMAP_HSMMC_DATA		0x0120
#define OMAP_HSMMC_PSTATE	0x0124
#define OMAP_HSMMC_HCTL		0x0128
#define OMAP_HSMMC_SYSCTL	0x012C
#define OMAP_HSMMC_STAT		0x0130
#define OMAP_HSMMC_IE		0x0134
#define OMAP_HSMMC_ISE		0x0138
#define OMAP_HSMMC_AC12		0x013C
#define OMAP_HSMMC_CAPA		0x0140
#define OMAP_HSMMC_CUR_CAPA	0x0148
#define OMAP_HSMMC_REV		0x01FC

/* HSMMC controller bit definitions */
#define VS18		(1<<26)
#define VS30		(1<<25)
#define VS33		(1<<24)
#define SDVS18		(0x5<<9)
#define SDVS30		(0x6<<9)
#define SDVSCLR		0xFFFFF1FF
#define SDVSDET		0x00000400
#define SIDLE_MODE	(0x2<<3)
#define AUTOIDLE	0x1
#define SDBP		(1<<8)
#define DTO		0xe
#define ICE		0x1
#define ICS		0x2
#define CEN		(1<<2)
#define CLKD_MASK	0x0000FFC0
#define INT_EN_MASK	0x307F0033
#define SDIO_CARD_INT_EN_MASK	0x307F0133
#define INIT_STREAM	(1<<1)
#define DP_SELECT	(1<<21)
#define DDIR		(1<<4)
#define DMA_EN		0x1
#define MSBS		1<<5
#define BCE		1<<1
#define EIGHT_BIT	1 << 5
#define FOUR_BIT	1 << 1
#define CC		0x1
#define TC		0x02
#define OD		0x1
#define BRW		0x400
#define BRR		0x800
#define BRE		(1<<11)
#define BWE		(1<<10)
#define SBGR		(1<<16)
#define CT		(1<<17)
#define OMAP_SDIO_WRITE	(1<<31)
#define SDIO_BLKMODE	(1<<27)
#define OMAP_HSMMC_ERR	(1 << 15)          /* Any error */
#define OMAP_HSMMC_CMD_TIMEOUT	(1 << 16)  /* Command response time-out */
#define OMAP_HSMMC_DATA_TIMEOUT	(1 << 20)  /* Data response time-out */
#define OMAP_HSMMC_CMD_CRC	(1 << 17)  /* Command CRC error */
#define OMAP_HSMMC_DATA_CRC	(1 << 21)  /* Date CRC error */
#define OMAP_HSMMC_CARD_ERR	(1 << 28)  /* Card ERR */
#define OMAP_HSMMC_CARD_INT	(1 << 8)   /* SDIO Card INT */
#define OMAP_HSMMC_STAT_CLEAR	0xFFFFFFFF
#define INIT_STREAM_CMD		0x00000000
#define INT_CLEAR		0x00000000
#define BLK_CLEAR		0x00000000

#define sdio_blkmode_regaddr1	0x2000
#define sdio_blkmode_regaddr2	0x2200
#define sdio_blkmode_mask	0x03F1FE00
#define sdio_function_mask	0x70000000
#define sdio_rw_function_mask	0x000E0000
#define IO_RW_DIRECT_MASK	0xF000FF00
#define IO_RW_DIRECT_ARG_MASK	0x80001A00
#define SDIO_CARD_INT_ENABLE	1
#define SDIO_CARD_INT_DISABLE	0

#define MMC_TIMEOUT_MS	20
#define NO_OF_MMC_HOSTS 2

/* MMC Host controller read/write API's */
#define OMAP_HSMMC_READ(base, reg)	__raw_readl((base) + OMAP_HSMMC_##reg)
#define OMAP_HSMMC_WRITE(base, reg, val)	__raw_writel((val), (base) + OMAP_HSMMC_##reg)

#ifdef CONFIG_OMAP34XX_OFFMODE
extern int context_restore_required(struct clk *clk);
extern void modify_timeout_value(struct clk *clk, u32 value);
#endif /* #ifdef CONFIG_OMAP34XX_OFFMODE */
#endif
