/* linux/include/asm-arm/arch-msm/memory.h
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

#include <asm/page.h>
#include <asm/sizes.h>

/* physical offset of RAM */
#if defined(CONFIG_ARCH_MSM7X00A)
#define PHYS_OFFSET		UL(0x10000000)
#elif defined(CONFIG_ARCH_MSM7X25)
#define PHYS_OFFSET		UL(0x00000000)
#else
#define PHYS_OFFSET		UL(0x10000000)
#endif

#define PAGE_OFFSET		UL(0xc0000000)

#ifdef CONFIG_DISCONTIGMEM
/*
 * 	node 0:  0x00000000-0x20000000 -> 0xc0000000-0xd0000000
 * 	node 1:  0x20000000-0x40000000 -> 0xd0000000-0xe0000000
 *
 * This needs a node mem size of 29 bits.
 */
#define PHYS_NODE_SIZE_SHIFT 29
#define VIRT_NODE_SIZE_SHIFT 28

/* 0x00000000 <-> 0x00000000, 0x20000000 <-> 0x10000000 */
#define __msm_align_banks(x)	((((x) & 0x20000000) >> 1) | ((x) & 0x0FFFFFFF))
#define __msm_unalign_banks(x)	((((x) & 0x10000000) << 1) | ((x) & 0x0FFFFFFF))

#define __virt_to_phys(x)	(__msm_unalign_banks((x) - PAGE_OFFSET + PHYS_OFFSET))
#define __phys_to_virt(x)	(__msm_align_banks(x) - PHYS_OFFSET + PAGE_OFFSET)

/*
 * Given a kernel address, find the home node of the underlying memory.
 */
#define KVADDR_TO_NID(addr) \
	(((unsigned long)(addr) - PAGE_OFFSET) >> VIRT_NODE_SIZE_SHIFT)

/*
 * Given a page frame number, convert it to a node id.
 */
#define PFN_TO_NID(pfn) \
	(((pfn) - PHYS_PFN_OFFSET) >> (PHYS_NODE_SIZE_SHIFT - PAGE_SHIFT))

/*
 * Given a kaddr, LOCAL_MEM_MAP finds the owning node of the memory
 * and returns the index corresponding to the appropriate page in the
 * node's mem_map.
 */
#define VIRT_NODE_MASK ((1 << VIRT_NODE_SIZE_SHIFT) - 1)
#define LOCAL_MAP_NR(addr) \
	(((unsigned long)(addr) & VIRT_NODE_MASK) >> PAGE_SHIFT)

#endif

#ifndef __ASSEMBLY__
void *alloc_bootmem_aligned(unsigned long size, unsigned long alignment);
void clean_and_invalidate_caches(unsigned long, unsigned long, unsigned long);
void clean_caches(unsigned long, unsigned long, unsigned long);
void invalidate_caches(unsigned long, unsigned long, unsigned long);
#endif

/* bus address and physical addresses are identical */
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

/* Qualcomm seems sure that on 7x27 the write to strongly ordered memory is
 * needed to flush the AXI bus properly.  The arch_barrier_extra is added to
 * dmb() in include/asm-arm/system.h.
 * Since there is no #define for 7x27 we're using a board check instead.
 */
#ifndef __ASSEMBLY__
#ifdef CONFIG_MACH_CHUCK
void write_to_strongly_ordered_memory(void);

#include <asm/mach-types.h>

#define arch_barrier_extra() do \
	{ \
		write_to_strongly_ordered_memory(); \
	} while (0)
#endif
#endif

#endif

