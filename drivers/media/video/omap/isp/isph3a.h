/*
 * drivers/media/video/omap/isp/isph3a.h
 *
 * Include file for H3A module in TI's OMAP3430 Camera ISP
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

#ifndef OMAP_ISP_H3A_H
#define OMAP_ISP_H3A_H

#define AEWB_PACKET_SIZE	16
#define H3A_MAX_BUFF		5

/* Flags for changed registers */
#define PCR_CHNG		(1 << 0)
#define AEWWIN1_CHNG		(1 << 1)
#define AEWINSTART_CHNG		(1 << 2)
#define AEWINBLK_CHNG		(1 << 3)
#define AEWSUBWIN_CHNG		(1 << 4)
#define PRV_WBDGAIN_CHNG	(1 << 5)
#define PRV_WBGAIN_CHNG		(1 << 6)

/* Flags for update field */
#define REQUEST_STATISTICS      (1 << 0)
#define SET_COLOR_GAINS         (1 << 1)
#define SET_DIGITAL_GAIN        (1 << 2)
#define SET_EXPOSURE            (1 << 3)
#define SET_ANALOG_GAIN         (1 << 4)

#define MAX_SATURATION_LIM	1023
#define MIN_WIN_H		2
#define MAX_WIN_H		256
#define MIN_WIN_W		6
#define MAX_WIN_W		256
#define MAX_WINVC		128
#define MAX_WINHC		36
#define MAX_WINSTART		4095
#define MIN_SUB_INC		2
#define MAX_SUB_INC		32

#define MAX_FRAME_COUNT         0x0FFF
#define MAX_FUTURE_FRAMES	10

/* ISPH3A REGISTERS bits */
#define ISPH3A_PCR_AF_EN    		(1 << 0)
#define ISPH3A_PCR_AF_ALAW_EN		(1 << 1)
#define ISPH3A_PCR_AF_MED_EN		(1 << 2)
#define ISPH3A_PCR_AF_BUSY		(1 << 15)
#define ISPH3A_PCR_AEW_EN    		(1 << 16)
#define ISPH3A_PCR_AEW_ALAW_EN		(1 << 17)
#define ISPH3A_PCR_AEW_BUSY  		(1 << 18)

#define WRITE_SAT_LIM(reg, sat_limit)	\
		reg = (reg & ~ISPH3A_PCR_AEW_AVE2LMT_MASK) \
			| (sat_limit << ISPH3A_PCR_AEW_AVE2LMT_SHIFT)

#define WRITE_ALAW(reg, alaw_en) \
		reg = (reg & ~ISPH3A_PCR_AEW_ALAW_EN) \
			| ((alaw_en & ISPH3A_PCR_AF_ALAW_EN) \
			<< ISPH3A_PCR_AEW_ALAW_EN_SHIFT)

#define WRITE_WIN_H(reg, height) \
		reg = (reg & ~ISPH3A_AEWWIN1_WINH_MASK) \
			| (((height >> 1) - 1) << ISPH3A_AEWWIN1_WINH_SHIFT)

#define WRITE_WIN_W(reg, width) \
		reg = (reg & ~ISPH3A_AEWWIN1_WINW_MASK) \
			| (((width >> 1) - 1) << ISPH3A_AEWWIN1_WINW_SHIFT)

#define WRITE_VER_C(reg, ver_count) \
		reg = (reg & ~ISPH3A_AEWWIN1_WINVC_MASK) \
			| ((ver_count - 1) << ISPH3A_AEWWIN1_WINVC_SHIFT)

#define WRITE_HOR_C(reg, hor_count) \
		reg = (reg & ~ISPH3A_AEWWIN1_WINHC_MASK) \
			| ((hor_count - 1) << ISPH3A_AEWWIN1_WINHC_SHIFT)
			
#define WRITE_VER_WIN_ST(reg, ver_win_st) \
		reg = (reg & ~ISPH3A_AEWINSTART_WINSV_MASK) \
			| (ver_win_st << ISPH3A_AEWINSTART_WINSV_SHIFT)

#define WRITE_HOR_WIN_ST(reg, hor_win_st) \
		reg = (reg & ~ISPH3A_AEWINSTART_WINSH_MASK) \
			| (hor_win_st << ISPH3A_AEWINSTART_WINSH_SHIFT)

#define WRITE_BLK_VER_WIN_ST(reg, blk_win_st) \
		reg = (reg & ~ISPH3A_AEWINBLK_WINSV_MASK) \
			| (blk_win_st << ISPH3A_AEWINBLK_WINSV_SHIFT)

#define WRITE_BLK_WIN_H(reg, height) \
		reg = (reg & ~ISPH3A_AEWINBLK_WINH_MASK) \
			| (((height >> 1) - 1) << ISPH3A_AEWINBLK_WINH_SHIFT)

#define WRITE_SUB_VER_INC(reg, sub_ver_inc) \
		reg = (reg & ~ISPH3A_AEWSUBWIN_AEWINCV_MASK) \
		| (((sub_ver_inc >> 1) - 1) << ISPH3A_AEWSUBWIN_AEWINCV_SHIFT)			

#define WRITE_SUB_HOR_INC(reg, sub_hor_inc) \
		reg = (reg & ~ISPH3A_AEWSUBWIN_AEWINCH_MASK) \
		| (((sub_hor_inc >> 1) - 1) << ISPH3A_AEWSUBWIN_AEWINCH_SHIFT)


struct isph3a_aewb_config {
	u16 saturation_limit;    
	u16 win_height;          /* Range: 2 - 256 */
	u16 win_width;           /* Range: 2 - 256 */
	u16 ver_win_count;       /* vertical window count: 1 - 128 */
	u16 hor_win_count;       /* horizontal window count: 1 - 36 */
	u16 ver_win_start;       /* ver window start position: 0 - 4095 */
	u16 hor_win_start;       /* hor window start position: 0 - 4095 */  
	u16 blk_ver_win_start;   /* black line ver window start pos: 0 -4095 */
	u16 blk_win_height;      /* black line height: 2 - 256 */
	u16 subsample_ver_inc;   /* ver distance between subsamples: 2 - 32 */
	u16 subsample_hor_inc;   /* hor distance between subsamples: 2 - 32 */
	u8   alaw_enable;        /* enable AEW ALAW flag */
	u8   aewb_enable;        /* AE AWB enable flag */
};

struct isph3a_aewb_data {
	void * h3a_aewb_statistics_buf;  /* Pointer to pass to user */
	u32 shutter;                 /* Shutter speed */
	u16 gain;                    /* Sensor analog Gain */
	u32 shutter_cap;             /* Shutter speed for capture */
	u16 gain_cap;                /* Sensor Gain for capture */
	
	u16 dgain;                   /* White balance digital gain */
	u16 wb_gain_b;               /* White balance color gain blue */
	u16 wb_gain_r;               /* White balance color gain red */
	u16 wb_gain_gb;              /* White balance color gain green blue */
	u16 wb_gain_gr;              /* White balance color gain green red */

	u16 frame_number;            /* Frame number of requested stats */
	u16 curr_frame;              /* Current frame number being processed */
	u8 update;                   /* Bitwise flags to update parameters */
};

/*
 * Sets the parameters in H3A registers
 */
int isph3a_aewb_configure(struct isph3a_aewb_config *aewbcfg);

/*
 * Requests AE and AWB statistics
 */
int isph3a_aewb_request_statistics(struct isph3a_aewb_data *aewbdata);  

/*
 * Saves h3a context
 */
void isph3a_save_context(void);

/*
 * Restores h3a context
 */
void isph3a_restore_context(void);

/*
 * Update WB values after a H3A statistics request
 */
void isph3a_update_wb(void);

#endif   /* OMAP_ISP_H3A_H */
