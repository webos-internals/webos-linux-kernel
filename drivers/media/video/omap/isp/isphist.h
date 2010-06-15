/*
 * drivers/media/video/omap/isp/isphist.h
 *
 * Include file for HISTOGRAM module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_HIST_H
#define OMAP_ISP_HIST_H

/* Flags for number of bins */
#define BINS_32                 0x0
#define BINS_64                 0x1
#define BINS_128                0x2
#define BINS_256                0x3

#define MAX_REGIONS             0x4
#define MAX_WB_GAIN             255
#define MIN_WB_GAIN             0x0
#define MAX_BIT_WIDTH           14
#define MIN_BIT_WIDTH           8

#define ISPHIST_PCR_EN		(1 << 0)
#define HIST_MEM_SIZE          	1024
#define ISPHIST_CNT_CLR_EN    	(1 << 7)

#define WRITE_SOURCE(reg, source)	\
		reg = (reg & ~ISPHIST_CNT_SOURCE_MASK) \
    | (source << ISPHIST_CNT_SOURCE_SHIFT)

#define WRITE_HV_INFO(reg, hv_info) \
    reg = ((reg & ~ISPHIST_HV_INFO_MASK) \
    | (hv_info & ISPHIST_HV_INFO_MASK))

#define WRITE_RADD(reg, radd) \
		reg = (reg & ~ISPHIST_RADD_MASK) \
    | (radd << ISPHIST_RADD_SHIFT)

#define WRITE_RADD_OFF(reg, radd_off) \
    reg = (reg & ~ISPHIST_RADD_OFF_MASK) \
    | (radd_off << ISPHIST_RADD_OFF_SHIFT)

#define WRITE_BIT_SHIFT(reg, bit_shift) \
    reg = (reg & ~ISPHIST_CNT_SHIFT_MASK) \
    | (bit_shift << ISPHIST_CNT_SHIFT_SHIFT)

#define WRITE_DATA_SIZE(reg, data_size) \
    reg = (reg & ~ISPHIST_CNT_DATASIZE_MASK) \
    | (data_size << ISPHIST_CNT_DATASIZE_SHIFT)

#define WRITE_NUM_BINS(reg, num_bins) \
    reg = (reg & ~ISPHIST_CNT_BINS_MASK) \
    | (num_bins << ISPHIST_CNT_BINS_SHIFT)

#define WRITE_WB_R(reg, reg_wb_gain) \
    reg = (reg & ~ISPHIST_WB_GAIN_WG00_MASK) \
    | (reg_wb_gain << ISPHIST_WB_GAIN_WG00_SHIFT)

#define WRITE_WB_RG(reg, reg_wb_gain) \
    reg = (reg & ~ISPHIST_WB_GAIN_WG01_MASK) \
    | (reg_wb_gain << ISPHIST_WB_GAIN_WG01_SHIFT)

#define WRITE_WB_B(reg, reg_wb_gain) \
    reg = (reg & ~ISPHIST_WB_GAIN_WG02_MASK) \
    | (reg_wb_gain << ISPHIST_WB_GAIN_WG02_SHIFT)

#define WRITE_WB_BG(reg, reg_wb_gain) \
    reg = (reg & ~ISPHIST_WB_GAIN_WG03_MASK) \
    | (reg_wb_gain << ISPHIST_WB_GAIN_WG03_SHIFT)

#define WRITE_REG_HORIZ(reg, reg_n_hor) \
    reg = ((reg & ~ISPHIST_REGHORIZ_MASK) \
    | (reg_n_hor & ISPHIST_REGHORIZ_MASK))

#define WRITE_REG_VERT(reg, reg_n_vert) \
    reg = ((reg & ~ISPHIST_REGVERT_MASK) \
    | (reg_n_vert & ISPHIST_REGVERT_MASK))

struct isp_hist_config {
	u8 hist_source;		/* CCDC or Memory */
	u8 input_bit_width;	/* Needed o know the size per pixel */
	u8 hist_frames;		/* Numbers of frames to be processed and accumulated */
	u8 hist_h_v_info;	/* frame-input width and height if source is memory */
	u16 hist_radd;		/* frame-input address in memory */
	u16 hist_radd_off;	/* line-offset for frame-input */
	u16 hist_bins;		/* number of bins: 32, 64, 128, or 256 */
	u16 wb_gain_R;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_RG;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_B;		/* White Balance Field-to-Pattern Assignments */
	u16 wb_gain_BG;		/* White Balance Field-to-Pattern Assignments */
	u8 num_regions;		/* number of regions to be configured */
	u16 reg0_hor;		/* Region 0 size and position */
	u16 reg0_ver;		/* Region 0 size and position */
	u16 reg1_hor;		/* Region 1 size and position */
	u16 reg1_ver;		/* Region 1 size and position */
	u16 reg2_hor;		/* Region 2 size and position */
	u16 reg2_ver;		/* Region 2 size and position */
	u16 reg3_hor;		/* Region 3 size and position */
	u16 reg3_ver;		/* Region 3 size and position */
};

struct isp_hist_data {

	u32 *hist_statistics_buf;	/* Pointer to pass to user */

};

/*
 * Validate parameters to be stored in HIST registers
 */
int isp_hist_configure(struct isp_hist_config *histcfg);

/*
 * Requests Histrogram statistics
 */
int isp_hist_request_statistics(struct isp_hist_data *histdata);

/*
 * Saves hist context
 */
void isphist_save_context(void);

/*
 * Restores hist context
 */
void isphist_restore_context(void);

#endif				/* OMAP_ISP_HIST */
