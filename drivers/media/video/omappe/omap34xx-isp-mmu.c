/*
 * drivers/media/video/omap/omap34xx-isp-mmu.c
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/delay.h>
#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
#include <linux/device.h>
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/scatterlist.h>
#include <asm/arch/isp.h>
#include "omap34xx.h"

// TODO: this should come from the board file
#define MMU_BASE			0x480BD400
#define MMU_REG(off)			(MMU_BASE + (off))

#define MMU_REVISION			MMU_REG(0x00)
#define	MAJORREV_SHIFT			4
#define	MINORREV_MASK			(0xF << 0)

#define MMU_SYSCONFIG			MMU_REG(0x10)
#define	IDLEMODE_MASK			(0x3 << 3)
#define	IDLEMODE(val)			((0x3 & (val)) << 3)
#define	SOFTRESET_MASK			(0x1 << 1)
#define	AUTOIDLE_MASK			(0x1 << 0)
#define	AUTOIDLE(val)			((0x1 & (val)) << 0)

#define MMU_SYSSTATUS			MMU_REG(0x14)
#define	RESETDONE_MASK			(0x1 << 0)

#define MMU_CNTL			MMU_REG(0x44)
#define	TWLENABLE_MASK			(0x1 << 2)
#define	MMUENABLE_MASK			(0x1 << 1)

#define MMU_TTB				MMU_REG(0x4C)
#define	TTBADDRESS(val)			(0xFFFFFF80 & (val))

#define MMU_GFLUSH			MMU_REG(0x60)
#define	GLOBALFLUSH_MASK		(0x1 << 0)

#define ISPMMU_L1D_TYPE_SHIFT		0
#define ISPMMU_L1D_TYPE_MASK		0x3
#define ISPMMU_L1D_TYPE_FAULT		0
#define ISPMMU_L1D_TYPE_FAULT1	3
#define ISPMMU_L1D_TYPE_PAGE		1
#define ISPMMU_L1D_TYPE_SECTION	2
#define ISPMMU_L1D_PAGE_ADDR_SHIFT	10

#define ISPMMU_L2D_TYPE_SHIFT		0
#define ISPMMU_L2D_TYPE_MASK		0x3
#define ISPMMU_L2D_TYPE_FAULT		0
#define ISPMMU_L2D_TYPE_LARGE_PAGE	1
#define ISPMMU_L2D_TYPE_SMALL_PAGE	2
#define ISPMMU_L2D_SMALL_ADDR_SHIFT	12
#define ISPMMU_L2D_SMALL_ADDR_MASK	0xFFFFF000
#define ISPMMU_L2D_M_ACCESSBASED	(1<<11)
#define ISPMMU_L2D_E_BIGENDIAN	(1<<9)
#define ISPMMU_L2D_ES_SHIFT		4
#define ISPMMU_L2D_ES_MASK		~(3<<4)
#define ISPMMU_L2D_ES_8BIT		0
#define ISPMMU_L2D_ES_16BIT		1
#define ISPMMU_L2D_ES_32BIT		2
#define ISPMMU_L2D_ES_NOENCONV	3

#define ISPMMU_TTB_ENTRIES_NR		4096

/* Number 1MB entries in TTB in one 32MB region */
#define ISPMMU_REGION_ENTRIES_NR	32

/* 128 region entries */
#define ISPMMU_REGION_NR		(ISPMMU_TTB_ENTRIES_NR 		\
					/ISPMMU_REGION_ENTRIES_NR)

/* Each region is 32MB */
#define ISPMMU_REGION_SIZE		(ISPMMU_REGION_ENTRIES_NR*(1<<20))

/* Number of entries per L2 Page table */
#define ISPMMU_L2D_ENTRIES_NR		256

/*
 * Statically allocate 16KB for L2 page tables. 16KB can be used for
 * up to 16 L2 page tables which cover up to 16MB space. We use an array of 16
 * to keep track of these 16 L2 page table's status. 
 */
#define L2P_TABLE_SIZE		1024
#define L2P_TABLE_NR 		41 // Currently supports 4*5MP shots
#define L2P_TABLES_SIZE 	(L2P_TABLE_SIZE*L2P_TABLE_NR)

/* Extra memory allocated to get ttb aligned on 16KB */
#define ISPMMU_TTB_MISALIGN_SIZE	0x3000

#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
#define SPEW(level, args...) \
	do { \
		if (omap34xx_isp_mmu_spew_level >= level) \
			printk(KERN_DEBUG "MMU: " args); \
	} while (0)
#else // !CONFIG_VIDEO_OMAP34XX_ISP_DBG
#define SPEW(level, args...)
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG

enum ISPMMU_MAP_ENDIAN {
	L_ENDIAN,
	B_ENDIAN
};

enum ISPMMU_MAP_ELEMENTSIZE {
	ES_8BIT,
	ES_16BIT,
	ES_32BIT,
	ES_NOENCONV
};

enum ISPMMU_MAP_MIXEDREGION {
	ACCESS_BASED,
	PAGE_BASED
};

enum ISPMMU_MAP_SIZE {
	L1DFAULT,
	PAGE,
	SECTION,
	SUPERSECTION,
	L2DFAULT,
	LARGEPAGE,
	SMALLPAGE
};

/* Page structure for statically allocated l1 and l2 page tables */
static struct page *ttb_page;
static struct page *l2p_page;

/*
* Allocate the same number as of TTB entries for easy tracking 
* even though L2P tables are limited to 16 or so 
*/
static u32 l2p_table_addr[4096];

/* An array of flags to keep the L2P table allotted */
static int l2p_table_allotted[L2P_TABLE_NR];

/* TTB virtual and physical address */
static u32 *ttb, ttb_p;

/* Worst case allocation for TTB for 16KB alignment */
static u32 ttb_aligned_size;

/* L2 page table base virtural and physical address */
static u32 l2_page_cache, l2_page_cache_p;

/* Structure for Mapping Attributes in the L1, L2 descriptor*/
struct omap34xx_isp_mmu_mapattr{
	enum ISPMMU_MAP_ENDIAN endianism;
	enum ISPMMU_MAP_ELEMENTSIZE element_size;
	enum ISPMMU_MAP_MIXEDREGION mixed_size;
	enum ISPMMU_MAP_SIZE map_size;
};

static struct omap34xx_isp_mmu_mapattr l1_mapattr_obj, l2_mapattr_obj;

#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
static int omap34xx_isp_mmu_spew_level = 0;

static ssize_t
omap34xx_isp_mmu_show_spew_level(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t i;

	i = snprintf(buf, PAGE_SIZE, "%d\n", omap34xx_isp_mmu_spew_level);
	return (i + 1);
}

static ssize_t
omap34xx_isp_mmu_store_spew_level(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char *endp;

	omap34xx_isp_mmu_spew_level = simple_strtoul(buf, &endp, 10);
	return (count);	
}

DEVICE_ATTR(mmu_spew_level, S_IRUGO|S_IWUGO,
		omap34xx_isp_mmu_show_spew_level,
		omap34xx_isp_mmu_store_spew_level);
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG

/*
 * Sets the L1,L2 descriptor with section/supersection/Largepage/Smallpage
 * base address or with L2 Page table address depending on the size parameter.
 * Returns the written L1/L2 descriptor.
 * pte_addr	: Pointer to the Indexed address in the L1 Page table ie TTB.
 * phy_addr	: Section/Supersection/L2page table physical address.
 * mapattr	: Mapping attributes applicable for Section/Supersections.
 */
static u32 
omap34xx_isp_mmu_set_pte(u32* pte_addr, u32 phy_addr, struct omap34xx_isp_mmu_mapattr mapattr)
{
	u32 pte =0;

	switch(mapattr.map_size){
		case PAGE :
			pte = ISPMMU_L1D_TYPE_PAGE << ISPMMU_L1D_TYPE_SHIFT;
			pte |= (phy_addr >> ISPMMU_L1D_PAGE_ADDR_SHIFT) 
				<< ISPMMU_L1D_PAGE_ADDR_SHIFT;
			break;
		case SMALLPAGE:
			pte = ISPMMU_L2D_TYPE_SMALL_PAGE <<
				ISPMMU_L2D_TYPE_SHIFT;
			pte &= ~ISPMMU_L2D_M_ACCESSBASED;
			if(mapattr.endianism)
				pte |= ISPMMU_L2D_E_BIGENDIAN ;
			else
				pte &= ~ISPMMU_L2D_E_BIGENDIAN ;
			pte &= ISPMMU_L2D_ES_MASK;
			pte |= mapattr.element_size << ISPMMU_L2D_ES_SHIFT;
			pte |= (phy_addr >> ISPMMU_L2D_SMALL_ADDR_SHIFT)
					<< ISPMMU_L2D_SMALL_ADDR_SHIFT;
			break;
		case L1DFAULT:
			pte = ISPMMU_L1D_TYPE_FAULT << ISPMMU_L1D_TYPE_SHIFT;
			break;
		case L2DFAULT:
			pte = ISPMMU_L2D_TYPE_FAULT << ISPMMU_L2D_TYPE_SHIFT;
			break;
		default:
			break;
	};

	*pte_addr = pte;
	return pte;
}

/*
 * Returns the index in the ttb for a free 32MB region
 * Returns 0 as an error code, if run out of regions.
 */
static u32
find_free_region_index(void)
{
	int idx =0;
	/* Find the first free 32M region in ttb. skip region 0 to avoid NULL pointer */
	for (idx = ISPMMU_REGION_ENTRIES_NR; idx < ISPMMU_TTB_ENTRIES_NR;
			idx += ISPMMU_REGION_ENTRIES_NR){
		if (((*(ttb + idx)) & ISPMMU_L1D_TYPE_MASK) ==
			(ISPMMU_L1D_TYPE_FAULT << ISPMMU_L1D_TYPE_SHIFT))
			break;
	}
	if (idx == ISPMMU_TTB_ENTRIES_NR) {
		SPEW(2, "run out of virtual space\n");
		return 0;
	}
	return idx;
}

/*
 * Returns the Page aligned address
 * addr		:Address to be page aligned
 */
static inline u32
page_aligned_addr(u32 addr)
{
	u32 paddress;
	paddress = addr & ~(PAGE_SIZE-1) ;
	return paddress;
}

/*
 * Returns the physical address of the allocated L2 page Table.
 * l2_table	: Virtual address of the allocated l2 table.
 */
static inline u32
l2_page_paddr(u32 l2_table)
{
	return (l2_page_cache_p + (l2_table - l2_page_cache));
}

/*
 * Allocates contigous memory for L2 page tables.
 */
static int
init_l2_page_cache(void)
{
	int i;
	u32 *l2p;

	l2p_page = alloc_pages(GFP_KERNEL,get_order(L2P_TABLES_SIZE));
	if (!l2p_page){
		SPEW(2, "ISP_ERR : No Memory for L2 page tables\n");
		return -ENOMEM;
	}
	l2p = page_address(l2p_page);
	l2_page_cache = (u32)l2p;
	l2_page_cache_p = __pa(l2p);
	l2_page_cache = (u32)ioremap_nocache(l2_page_cache_p,L2P_TABLES_SIZE);
	
	for (i = 0; i < L2P_TABLE_NR; i++)
		l2p_table_allotted[i] = 0;

	SPEW(2, "Mem for L2 page tables at l2_paddr = %x, \
			l2_vaddr = 0x%x, of bytes = 0x%x\n",
		l2_page_cache_p, l2_page_cache,L2P_TABLES_SIZE);
	
	/*HW Errata 1.40. Camera ISP: MMU endianess polarity inverted */
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0))
		l2_mapattr_obj.endianism = B_ENDIAN;
	else
		l2_mapattr_obj.endianism = L_ENDIAN;
	
	l2_mapattr_obj.element_size = ES_8BIT;
	l2_mapattr_obj.mixed_size = ACCESS_BASED;
	l2_mapattr_obj.map_size = L2DFAULT;
	return 0;
}

/*
 * Frees the memory of L2 page tables.
 */
static void
cleanup_l2_page_cache(void)
{
	if(l2p_page){
		ioremap_cached(l2_page_cache_p,L2P_TABLES_SIZE);
		__free_pages(l2p_page,get_order(L2P_TABLES_SIZE));
	}
}

/*
 * Finds the free L2 Page table slot.
 * Fills the allotted L2 Page table with default entries.
 * Returns the virtual address of the allotted L2 Pagetable,
 */
static u32
request_l2_page_table(void)
{
	int i,j;
	u32 l2_table;

	for (i = 0; i < L2P_TABLE_NR; i++) {
		if (!l2p_table_allotted[i])
			break;
	}
	if (i < L2P_TABLE_NR) {
		l2p_table_allotted[i] = 1;
		l2_table = l2_page_cache + (i * L2P_TABLE_SIZE);
		l2_mapattr_obj.map_size = L2DFAULT;
		/*Fill up all the entries with fault */
		for(j=0;j<ISPMMU_L2D_ENTRIES_NR;j++)
			omap34xx_isp_mmu_set_pte((u32*)l2_table+j,0,l2_mapattr_obj);
		SPEW(2, "Allotted l2 page table at 0x%x\n",
					(u32)l2_table);
		return l2_table;
	}
	else{
		SPEW(2, "ISP_ERR : Cannot allocate more than 16 L2\
				Page Tables");
		return 0;
	}
}

/*
 * Frees the allotted L2 Page table slot.
 */
static int
free_l2_page_table(u32 l2_table)
{
	int i;

	SPEW(2, "Free l2 page table at 0x%x\n", l2_table);
	for (i = 0; i < L2P_TABLE_NR; i++)
		if (l2_table == (l2_page_cache + (i * L2P_TABLE_SIZE))) {
			if (!l2p_table_allotted[i]){
				SPEW(2, "L2 page not in use\n");
			}
			l2p_table_allotted[i] = 0;
			return 0;
		}
	SPEW(2, "L2 table not found\n");
	return -EINVAL;
}

/*
 * Map a physically contiguous buffer to ISP space. This call is used to
 * map a frame buffer
 * p_addr	: Physical address of the contigous mem to be mapped.
 * size		: Size of the contigous mem to be mapped.
 */
dma_addr_t
omap34xx_isp_mmu_map(u32 p_addr, int size)
{
	int i, j, idx, num;
	u32 sz,first_padding;
	u32 p_addr_align, p_addr_align_end;
	u32 pd;
	u32 *l2_table;

	SPEW(2, "map: p_addr = 0x%x, size = 0x%x\n",p_addr,size);

	p_addr_align = page_aligned_addr(p_addr);

	first_padding = p_addr - p_addr_align;
	sz = size -first_padding;

	num = (sz/PAGE_SIZE) + ((sz%PAGE_SIZE)?1:0) + (first_padding ?1:0);
	p_addr_align_end = p_addr_align + num*PAGE_SIZE;

	SPEW(2, "buffer at 0x%x of size 0x%x spans to %d pages\n",
			p_addr, size, num);

	idx = find_free_region_index();
	if(!idx){
		SPEW(2, "Runs out of virtual space");
		return 0;
	}
	SPEW(2, "allocating region %d\n", idx/ISPMMU_REGION_ENTRIES_NR);

	/* how many second-level page tables we need */
	num = num/ISPMMU_L2D_ENTRIES_NR +
			((num%ISPMMU_L2D_ENTRIES_NR)?1:0);
	SPEW(2, "need %d second-level page tables (1KB each)\n", num);

	/* create second-level page tables */
	for (i = 0; i < num; i++) {
		l2_table = (u32 *)request_l2_page_table();
		if (!l2_table) {
			SPEW(2, "no memory\n");
			i--;
			goto release_mem;
		}

		/* Make the first level page descriptor */
		l1_mapattr_obj.map_size = PAGE;
		pd = omap34xx_isp_mmu_set_pte(ttb+idx+i, l2_page_paddr((u32)l2_table),
			l1_mapattr_obj);
		SPEW(2, "L1 pte[%d] = 0x%x\n", idx+i, pd);

		/* Make the second Level page descriptors */
		l2_mapattr_obj.map_size = SMALLPAGE;
		for (j = 0; j < ISPMMU_L2D_ENTRIES_NR; j++) {
			pd = omap34xx_isp_mmu_set_pte(l2_table + j,p_addr_align,
				l2_mapattr_obj);
			//SPEW(2, "L2 pte[%d] = 0x%x\n", j, pd);
			/*Contigous memory, just increment with Page size */
			p_addr_align += PAGE_SIZE;
			if (p_addr_align == p_addr_align_end)
				break;
		}
		/* save it so we can free this l2 table later */
		l2p_table_addr[idx + i] = (u32)l2_table;
	}

	SPEW(2, "mapped to ISP virtual address 0x%x\n",
		(u32)((idx << 20) + (p_addr & (PAGE_SIZE - 1))));

	omap_setl(MMU_GFLUSH, GLOBALFLUSH_MASK);
	return (dma_addr_t)((idx<<20) + (p_addr & (PAGE_SIZE - 1)));

release_mem:
	for (; i >= 0; i--) {
		free_l2_page_table(l2p_table_addr[idx + i]);
		l2p_table_addr[idx + i] = 0;
	}
	return 0;
}

EXPORT_SYMBOL_GPL(omap34xx_isp_mmu_map);

/*
 * Map a physically discontiguous buffer to ISP space. This call is used to
 * map a user buffer or a vmalloc buffer. The sg list is a set of pages.
 * sg_list		: Address of the Scatter gather linked list.
 * sglen		: Number of elements in the sg list.
 */
dma_addr_t
omap34xx_isp_mmu_map_sg(const struct scatterlist *sglist, int sglen)
{
	int i, j, idx, num, sg_num = 0;
	u32 pd,sg_element_addr;
	u32 *l2_table;

	SPEW(2, "Map_sg: sglen (num of pages) = %d\n", sglen);

	idx = find_free_region_index();
	if(!idx){
		SPEW(2, "Runs out of virtual space");
		return -EINVAL;
	}

	SPEW(2, "allocating region %d\n", idx/ISPMMU_REGION_ENTRIES_NR);

	/* How many second-level page tables we need */
	/*
	* Size of each sglist element does not exceed a page size
	* so consider the number of elements in the list for calcuating
	* number of L2P tables
	*/
	num = sglen/ISPMMU_L2D_ENTRIES_NR +
			((sglen%ISPMMU_L2D_ENTRIES_NR)?1:0);
	SPEW(2, "Need %d second-level page tables (1KB each)\n", num);

	/* create second-level page tables */
	for (i = 0; i < num; i++) {
		l2_table = (u32 *)request_l2_page_table();
		if (!l2_table) {
			SPEW(2, "No memory\n");
			i--;
			goto release_mem;
		}
		/* Make the first level page descriptor */
		l1_mapattr_obj.map_size = PAGE;
		pd = omap34xx_isp_mmu_set_pte(ttb+idx+i, l2_page_paddr((u32)l2_table),
			l1_mapattr_obj);
		SPEW(2, "L1 pte[%d] = 0x%x\n", idx+i, pd);

		/* Make the second Level page descriptors */
		l2_mapattr_obj.map_size = SMALLPAGE;
		for (j = 0; j < ISPMMU_L2D_ENTRIES_NR; j++) {
			/*Assuming that sglist elements are always page aligned */
			sg_element_addr = sg_dma_address(sglist + sg_num);
			if((sg_num>0) &&
				page_aligned_addr(sg_element_addr)
				!= sg_element_addr)
				SPEW(2, "ISP_ERR : Intermediate SG elements  \
					are not page aligned = 0x%x\n"
					,sg_element_addr);
			pd = omap34xx_isp_mmu_set_pte(l2_table+j,sg_element_addr,l2_mapattr_obj);

			//SPEW(2, "L2 pte[%d] = 0x%x\n", j, pd);

			sg_num++;
			if (sg_num == sglen)
				break;
		}
		/* save it so we can free this l2 table later */
		l2p_table_addr[idx + i] = (u32)l2_table;
	}

	SPEW(2, "mapped sg list to ISP virtual address 0x%x, idx=%d\n",
		(u32)((idx << 20) + (sg_dma_address(sglist+0) & (PAGE_SIZE - 1))), idx);

	omap_setl(MMU_GFLUSH, GLOBALFLUSH_MASK);
	return (dma_addr_t)((idx<<20) + (sg_dma_address(sglist+0) & (PAGE_SIZE - 1)));

release_mem:
	for (; i >= 0; i--) {
		free_l2_page_table(l2p_table_addr[idx + i]);
		l2p_table_addr[idx + i] = 0;
	}
	return 0;
}

EXPORT_SYMBOL_GPL(omap34xx_isp_mmu_map_sg);

/*
 * Unmap a ISP space that is mapped before via omap34xx_isp_mmu_map and
 * omap34xx_isp_mmu_map_sg.
 * v_addr	: Virtural address to be unmapped
 */
int
omap34xx_isp_mmu_unmap(dma_addr_t v_addr)
{
	u32 v_addr_align;
	int idx;

	SPEW(2, "+omap34xx_isp_mmu_unmap: 0x%x\n", v_addr);

	v_addr_align = page_aligned_addr(v_addr);
	idx = v_addr_align >> 20;
	if ((idx < ISPMMU_REGION_ENTRIES_NR) ||
		(idx >(ISPMMU_REGION_ENTRIES_NR*(ISPMMU_REGION_NR -1)))
		|| ((idx << 20) != v_addr_align)
		|| (idx%ISPMMU_REGION_ENTRIES_NR)) {
		SPEW(2, "Cannot unmap a non region-aligned space \
				0x%x\n", v_addr);
		return -EINVAL;
	}

	if (((*(ttb + idx)) & (ISPMMU_L1D_TYPE_MASK << ISPMMU_L1D_TYPE_SHIFT)) !=
			(ISPMMU_L1D_TYPE_PAGE << ISPMMU_L1D_TYPE_SHIFT)) {
		SPEW(2, "unmap a wrong region\n");
		return -EINVAL;
	}

	/* free the associated level-2 page tables */
	while (((*(ttb + idx)) & (ISPMMU_L1D_TYPE_MASK << ISPMMU_L1D_TYPE_SHIFT)) ==
			(ISPMMU_L1D_TYPE_PAGE << ISPMMU_L1D_TYPE_SHIFT)) {
		*(ttb + idx) = (ISPMMU_L1D_TYPE_FAULT << ISPMMU_L1D_TYPE_SHIFT);
		free_l2_page_table(l2p_table_addr[idx]);
		l2p_table_addr[idx++] = 0;
		if (!(idx%ISPMMU_REGION_ENTRIES_NR)) {
			SPEW(2, "Do not exceed this 32M region\n");
			break;
		}
	}
	
	omap_setl(MMU_GFLUSH, GLOBALFLUSH_MASK);

	SPEW(2, "-omap34xx_isp_mmu_unmap()\n");
	return 0;
}

EXPORT_SYMBOL_GPL(omap34xx_isp_mmu_unmap);

static void
omap34xx_isp_mmu_reset(void)
{
	int i;
	
	omap_setl(MMU_SYSCONFIG, SOFTRESET_MASK);
	
	for (i = 0; i < 1000; ++i, udelay(1))
		if (omap_testl(MMU_SYSSTATUS, RESETDONE_MASK))
			break;
}

void
omap34xx_isp_mmu_enable(int reset)
{
	unsigned int val;

	if (reset)
		omap34xx_isp_mmu_reset();
	
	val = IDLEMODE(OMAP34XX_MMU_SYSCONFIG_IDLEMODE_SMART_IDLE);
	val |= AUTOIDLE(1);
	omap_masked_writel(MMU_SYSCONFIG, val, IDLEMODE_MASK | AUTOIDLE_MASK);
	
	omap_writel(TTBADDRESS(ttb_p), MMU_TTB);
	omap_setl(MMU_CNTL, TWLENABLE_MASK | MMUENABLE_MASK);
}

EXPORT_SYMBOL(omap34xx_isp_mmu_enable);

void
omap34xx_isp_mmu_disable(void)
{
	omap_clearl(MMU_CNTL, TWLENABLE_MASK | MMUENABLE_MASK);
}

EXPORT_SYMBOL(omap34xx_isp_mmu_disable);

/*
 * Reserves memory for L1 and L2 Page tables.
 * Initializes the ISPMMU with TTB address, fault entries as default in the TTB table.
 * Enables MMU and TWL.
 * Sets the callback for the MMU error events.
 */
static int __init
omap34xx_isp_mmu_module_init(void)
{
	int i;
	int rc;
	
	ttb_page = alloc_pages(GFP_KERNEL,
				get_order(ISPMMU_TTB_ENTRIES_NR * 4));
	
	if (!ttb_page){
		SPEW(2, "No Memory for TTB\n");
		return -ENOMEM;
	}

	ttb = page_address(ttb_page);
	ttb_p = __pa(ttb);
	ttb_aligned_size = ISPMMU_TTB_ENTRIES_NR * 4;
	ttb = ioremap_nocache(ttb_p,ttb_aligned_size);
	if ((ttb_p & 0xFFFFC000) != ttb_p) {
		SPEW(2, "ISP_ERR : TTB address not aligned at 16KB\n");
		__free_pages(ttb_page,get_order(ISPMMU_TTB_ENTRIES_NR * 4));
		ttb_aligned_size = (ISPMMU_TTB_ENTRIES_NR * 4)
				+ (ISPMMU_TTB_MISALIGN_SIZE);
		ttb_page = alloc_pages(GFP_KERNEL,
					get_order(ttb_aligned_size));
		if (!ttb_page){
			SPEW(2, "No Memory for TTB\n");
			return -ENOMEM;
		}
		ttb = page_address(ttb_page);
		ttb_p = __pa(ttb);
		ttb = ioremap_nocache(ttb_p,ttb_aligned_size);
		if ((ttb_p & 0xFFFFC000) != ttb_p){
			/* Move the unaligned address to the next 16KB alignment */
			ttb = (u32*)(((u32)ttb & 0xFFFFC000) + 0x4000);
			ttb_p = __pa(ttb);
		}
	}

	SPEW(2, "TTB allocated at p = 0x%x, v = 0x%x, size = 0x%x\n",
		ttb_p, (u32)ttb, ttb_aligned_size);
	/*HW Errata 1.40. Camera ISP: MMU endianess polarity inverted */
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0))
		l1_mapattr_obj.endianism = B_ENDIAN;
	else
		l1_mapattr_obj.endianism = L_ENDIAN;
	l1_mapattr_obj.element_size = ES_8BIT;
	l1_mapattr_obj.mixed_size = ACCESS_BASED;
	l1_mapattr_obj.map_size = L1DFAULT;

	if ((rc = init_l2_page_cache())) {
		SPEW(2, "ISP_ERR : init l2 page cache\n");
		ttb = page_address(ttb_page);
		ttb_p = __pa(ttb);
		ioremap_cached(ttb_p,ttb_aligned_size);
		__free_pages(ttb_page,get_order(ttb_aligned_size));

		return rc;
	}

	/* Setting all the entries to generate fault by default */
	for (i = 0; i < ISPMMU_TTB_ENTRIES_NR; i++) {
		omap34xx_isp_mmu_set_pte(ttb + i, 0, l1_mapattr_obj);
	}

	return 0;
}

static void __exit
omap34xx_isp_mmu_module_cleanup(void)
{
	ttb = page_address(ttb_page);
	ttb_p = __pa(ttb);
	ioremap_cached(ttb_p,ttb_aligned_size);
	__free_pages(ttb_page,get_order(ttb_aligned_size));
	cleanup_l2_page_cache();
}

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP34xx ISP MMU Driver");
MODULE_LICENSE("GPL");

module_init(omap34xx_isp_mmu_module_init);
module_exit(omap34xx_isp_mmu_module_cleanup);
