/*
 * drivers/media/video/omap/isp/ispmmu.h
 *
 * OMAP3430 Camera ISP MMU API
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

#ifndef OMAP_ISP_MMU_H
#define OMAP_ISP_MMU_H

#include <asm/scatterlist.h>

dma_addr_t ispmmu_map(unsigned int p_addr,int size);

/*
* To be called from camera driver with scatter gather list
*/
dma_addr_t ispmmu_map_sg(const struct scatterlist *sglist, int sglen);
int ispmmu_unmap(dma_addr_t isp_addr);

void ispmmu_print_status(void);

enum 
ISPMMU_MAP_ENDIAN{L_ENDIAN, B_ENDIAN};

enum 
ISPMMU_MAP_ELEMENTSIZE{ES_8BIT,ES_16BIT,ES_32BIT,ES_NOENCONV};

enum
ISPMMU_MAP_MIXEDREGION{ACCESS_BASED, PAGE_BASED};

enum
ISPMMU_MAP_SIZE{L1DFAULT,PAGE, SECTION, SUPERSECTION,L2DFAULT,
			LARGEPAGE,SMALLPAGE};

/*
 * Saves mmu context
 */			
void ispmmu_save_context(void);

/*
 * Restores mmu context
 */	
void ispmmu_restore_context(void);
			
#endif /* OMAP_ISP_MMU_H */













