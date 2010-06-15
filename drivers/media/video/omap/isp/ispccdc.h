/*
 * drivers/media/video/ispccdc.h
 *
 * Driver include file for CCDC module in TI's OMAP3430 Camera ISP
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

#ifndef OMAP_ISP_CCDC_H
#define OMAP_ISP_CCDC_H


/*
 * Reserve the CCDC module.
 * Only one user at a time.
 */
int ispccdc_request(void);

/*
 * Marks CCDC module free.
 */
int ispccdc_free(void);

/*Enumeration constants for CCDC input output format */
enum ccdc_input {
	CCDC_RAW,
	CCDC_YUV_SYNC,
	CCDC_YUV_BT,
	CCDC_OTHERS
};
enum ccdc_output {
	CCDC_YUV_RSZ,
	CCDC_YUV_MEM_RSZ,
	CCDC_OTHERS_VP,
	CCDC_OTHERS_MEM,
	CCDC_OTHERS_VP_MEM
};

/*
 * Sets up the default CCDC configuration according to the arguments.
 */
int ispccdc_config_datapath(enum ccdc_input input,
			enum ccdc_output output);

/* 
 * Configures the crop settings in the CCDC module.
 */
void ispccdc_config_crop(u32 left, u32 top, u32 height, u32 width);

/* Enumeration constants for the sync interface parameters */
enum inpmode {
	RAW,
	YUV16,
	YUV8
};
enum datasize {
	DAT8,
	DAT10,
	DAT11,
	DAT12
};

/* Structure for the Sync Interface between the sensor and CCDC*/
struct ispccdc_syncif {
	/* 1 - Master, 0- Slave */
	u8 ccdc_mastermode;
	/* 0 - Odd Field, 1- Even Field */
	u8 fldstat;
	enum inpmode ipmod;
	enum datasize datsz;
	/* 0 -Progressive Mode, 1 -Interlaced Mode */
	u8 fldmode;
	/* 0 -Positive, 1 - Negative */
	u8 datapol;
	/* 0 -Positive, 1 - Negative */
	u8 fldpol;
	/* 0 -Positive, 1 - Negative */
	u8 hdpol;
	/* 0 -Positive, 1 - Negative */
	u8 vdpol;
	/* 0 -Input, 1 - Output */
	u8 fldout;
	/* Width of the Horizontal Sync pulse - used for HS/VS Output*/
	u8 hs_width;
	/* Width of the Vertical Sync pulse - used for HS/VS Output*/
	u8 vs_width;
	/*Number of pixels per line - used for HS/VS Output*/
	u8 ppln;
	/*Number of half lines per frame - used for HS/VS Output*/
	u8 hlprf;
	/*1 - Enable ITU-R BT656 mode, 0 - Sync mode*/
	u8 bt_r656_en;
};

/* Structure for LSC configuration*/
struct ispccdc_lsc_config {
	u8 offset;
	u8 gain_mode_n;
	u8 gain_mode_m;
	u8 gain_format;
	u16 fmtsph;
	u16 fmtlnh;
	u16 fmtslv;
	u16 fmtlnv;
	u8 initial_x;
	u8 initial_y;
};

/*
 * Configures the sync interface parameters between the sensor and the CCDC.
 */
void ispccdc_config_sync_if(struct ispccdc_syncif syncif);

/* Structure for the optical black Clamp and Digital black Clamp subtract*/
struct ispccdc_bclamp{
	/*Optical black average gain*/
	u8 obgain;
	/*Start Pixel w.r.t. HS pulse in Optical black sample*/
	u8 obstpixel;
	/*Optical Black Sample lines*/
	u8 oblines;
	/*Optical Black Sample Length*/
	u8 oblen;
	/*Digital Black Clamp subtract value */
	u16 dcsubval;
	};

/*
 * Configures the optical/digital black clamp parameters in CCDC.
 */
int ispccdc_config_black_clamp(struct ispccdc_bclamp bclamp);

/*
 * Enables  the optical or Digital black clamp.
 */
void ispccdc_enable_black_clamp(u8 enable);

/* Structure for FPC */
struct ispccdc_fpc{
	/* Number of faulty pixels to be corrected in the frame*/
	u16 fpnum;
	/* Memory address of the FPC Table */
	u32 fpcaddr;
	};

/*
 * Configures the Faulty Pixel Correction parameters.
 */
int ispccdc_config_fpc( struct ispccdc_fpc fpc);

/*
 * Enables  the Faulty Pixel Correction.
 * enable	:	: 1- Enables FPC
 */
void ispccdc_enable_fpc(u8 enable);

/* Structure for Black Level Compensation parameters*/
struct ispccdc_blcomp{
	u8 b_mg;
	u8 gb_g;
	u8 gr_cy;
	u8 r_ye;
	};

/*
 * Configures the Black Level Compensation parameters.
 */
void ispccdc_config_black_comp( struct ispccdc_blcomp blcomp);

/* Enumeration constants for Video Port */
enum vpin {
	BIT12_3 = 3,
	BIT11_2 = 4,
	BIT10_1 = 5,
	BIT9_0 = 6
};
enum vpif_freq {
	PIXCLKBY2,
	PIXCLKBY3_5,
	PIXCLKBY4_5,
	PIXCLKBY5_5,
	PIXCLKBY6_5
};

/*Structure for Video Port parameters */
struct ispccdc_vp {
	enum vpin bitshift_sel;
	enum vpif_freq freq_sel;
};

/*
 * Configures the Video Port Configuration parameters.
 */
void ispccdc_config_vp( struct ispccdc_vp vp);

/*
 * Enables  the Video Port.
 */
void ispccdc_enable_vp(u8 enable);

/* Structure for Reformatter parameters */
struct ispccdc_refmt{
	u8 lnalt;
	u8 lnum;
	u8 plen_even;
	u8 plen_odd;
	u32 prgeven0;
	u32 prgeven1;
	u32 prgodd0;
	u32 prgodd1;
	u32 fmtaddr0;
	u32 fmtaddr1;
	u32 fmtaddr2;
	u32 fmtaddr3;
	u32 fmtaddr4;
	u32 fmtaddr5;
	u32 fmtaddr6;
	u32 fmtaddr7;
};

/*
 * Configures the Reformatter register values if line alternating is disabled.
 * else just enabling the line alternating is enough.
 */
void ispccdc_config_reformatter( struct ispccdc_refmt refmt);

/*
 * Enables  the Reformatter
 */
void ispccdc_enable_reformatter(u8 enable);

/* Structure for Culling parameters */
struct ispccdc_culling{
	/* Vertical culling pattern */
	u8 v_pattern;
	/* Horizontal Culling pattern for odd lines */
	u16 h_odd;
	/* Horizontal Culling pattern for even lines */
	u16 h_even;
};

/*
 * Configures the Culling parameters.
 */
void ispccdc_config_culling(struct ispccdc_culling culling);

/*
 * Enables  the Low pass Filter
 */
void ispccdc_enable_lpf(u8 enable);

/* Enumeration constants for Alaw input width */
enum alaw_ipwidth{
	ALAW_BIT12_3 = 0x3,
	ALAW_BIT11_2 = 0x4,
	ALAW_BIT10_1 = 0x5,
	ALAW_BIT9_0 = 0x6
};

/*
 * Configures the input width for A-law.
 */
void ispccdc_config_alaw(enum alaw_ipwidth ipwidth);

/*
 * Enables  the A-law compression
 */
void ispccdc_enable_alaw(u8 enable);

/*
 * Loads LSC gain table.
 */
int ispccdc_load_lsc(void);

/*
 * Configures LSC module.
 */
void ispccdc_config_lsc(struct ispccdc_lsc_config * lsc_cfg);

/*
 * Enables LSC module
 */
void ispccdc_enable_lsc(u8 enable);

/*
 * Configures the sensor image specific attribute.
 */
void ispccdc_config_imgattr(u32 colptn);

/*
 * Programs the shadow registers associated with CCDC.
 */
void ispccdc_config_shadow_registers(void);

/*
 * Calculates the number of pixels cropped if the reformater is disabled,
 * Fills up the output widht height variables in the isp_ccdc structure .
*/
int ispccdc_try_size(u32 input_w, u32 input_h, u32 * output_w,
			u32 * output_h);

/*
 * Configures the appropriate values stored in the isp_ccdc structure to
 * HORZ/VERT_INFO registers and the VP_OUT depending on whether the image
 * is stored in memory or given to the another module in the ISP pipeline.
 */
int ispccdc_config_size(u32 input_w, u32 input_h, u32 output_w,
			u32 output_h);

/*
 * Configures the output line offset when stored in memory.
 * Configures the num of even and odd line fields in case of rearranging
 * the lines
 */
int ispccdc_config_outlineoffset(u32 offset, u8 oddeven, u8 numlines);

/*
 * Configures the memory address where the output should be stored.
 */
int ispccdc_set_outaddr(u32 addr);

/*
 *
 * Enables the CCDC module.
 * Client should configure all the sub modules in CCDC before this.
 */
void ispccdc_enable(u8 enable);
int ispccdc_busy(void);

/*
 * Saves CCDC context
 */
void ispccdc_save_context(void);

/*
 * Restores CCDC context
 */
void ispccdc_restore_context(void);

/*
 * Prints the values of the CCDC Module registers
 * Also prints other debug information stored in the CCDC module
 */
void ispccdc_print_status(void);

#endif		/* OMAP_ISP_CCDC_H */
