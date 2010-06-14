/*
 * include/asm-arm/arch-omap2/hdq.h
 *
 * Copyright (C) 2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 *
 */

#ifndef __ASM_OMAP2_HDQ_H__
#define __ASM_OMAP2_HDQ_H__

#define HDQ_REVISION				0x00
#define HDQ_TX_DATA				0x04
#define HDQ_RX_DATA				0x08

#define HDQ_CTRL_STATUS				0x0c
#define HDQ_CTRL_STATUS_1WIRE_SINGLE_BIT	(1<<7)
#define HDQ_CTRL_STATUS_INTERRUPTMASK		(1<<6)
#define HDQ_CTRL_STATUS_CLOCKENABLE		(1<<5)
#define HDQ_CTRL_STATUS_GO			(1<<4)
#define HDQ_CTRL_STATUS_PRESENCEDETECT 	 	(1<<3)
#define HDQ_CTRL_STATUS_INITIALIZATION		(1<<2)
#define HDQ_CTRL_STATUS_DIR			(1<<1)
#define HDQ_CTRL_STATUS_MODE			(1<<0)

#define HDQ_INT_STATUS				0x10
#define HDQ_INT_STATUS_TXCOMPLETE		(1<<2)
#define HDQ_INT_STATUS_RXCOMPLETE		(1<<1)
#define HDQ_INT_STATUS_TIMEOUT			(1<<0)

#define HDQ_SYSCONFIG				0x14
#define HDQ_SYSCONFIG_SOFTRESET			(1<<1)
#define HDQ_SYSCONFIG_AUTOIDLE			(1<<0)

#define HDQ_SYSSTATUS				0x18
#define HDQ_SYSSTATUS_RESETDONE			(1<<0)

#define HDQ_CMD_READ				(0)
#define HDQ_CMD_WRITE				(1<<7)

#define HDQ_FLAG_CLEAR				0
#define HDQ_FLAG_SET				1
#define HDQ_TIMEOUT				(HZ/5)

#define OMAP_HDQ_INTERRUPT_MODE
#endif
