/*
 *  linux/arch/arm/mm/userflush.c
 *
 *  Copyright (C) 2009 Palm Inc.
 *  Author: Chris McKillop <cdm@palm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/mm.h>

#include <asm/mmu_context.h>
#include <asm/tlbflush.h>
#include <linux/module.h>

extern int v7_dma_flush_range_user(unsigned int start, unsigned int end);
extern int v7_dma_inv_range_user(unsigned int start, unsigned int end);

void dma_flush_range_user(unsigned int start, unsigned int end)
{
	int rc;

	rc = v7_dma_flush_range_user(start, end);
	if (rc) {
		printk("Exception received in dma_flush_range_user()\n");
		//rc = send_sig(SIGSEGV, current, 1);
		//rc = force_sig(SIGSEGV, current);
		//printk("...force_sig(SIGSEGV) returning %d\n", rc);
	}
}

void dma_inv_range_user(unsigned int start, unsigned int end)
{
	int rc;

	rc = v7_dma_inv_range_user(start, end);
	if (rc) {
		printk("Exception received in dma_inv_range_user()\n");
		//rc = send_sig(SIGSEGV, current, 1);
		//rc = force_sig(SIGSEGV, current);
		//printk("...force_sig(SIGSEGV) returning %d\n",rc);
	}
}


EXPORT_SYMBOL(dma_flush_range_user);
EXPORT_SYMBOL(dma_inv_range_user);
