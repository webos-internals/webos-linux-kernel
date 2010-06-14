/*
 * drivers/media/video/omap/isp/isp.h
 *
 * Top level public header file for ISP Control module in
 * TI's OMAP3430 Camera ISP
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

#ifndef OMAP_ISP_TOP_H
#define OMAP_ISP_TOP_H

typedef void (*isp_callback_t) (unsigned long status, void *arg1, void *arg2);

enum isp_interface_type{
	ISP_PARLL = 1,
	ISP_CSIA = 2,
	ISP_CSIB = 4
};
enum isp_irqevents{
	CCDC_VD0 = 0x100,
	CCDC_VD1 = 0x200,
	CCDC_VD2 = 0x400,
	CCDC_ERR = 0x800,
	H3A_AWB_DONE = 0x2000,
	HIST_DONE = 0x10000,
	PREV_DONE = 0x100000,
	RESZ_DONE = 0x1000000,
	SBL_OVF = 0x2000000,
	MMU_ERR = 0x10000000,
	OCP_ERR = 0x20000000,
	HS_VS = 0x80000000
};

enum isp_callback_type{
	CBK_CCDC_VD0,
	CBK_CCDC_VD1,
	CBK_PREV_DONE,
	CBK_RESZ_DONE,
	CBK_MMU_ERR,
	CBK_H3A_AWB_DONE,
	CBK_HIST_DONE,
	CBK_HS_VS
};

#define ISP_TOK_TERM 	0xFFFFFFFF /* terminating token for ISP modules reg list */

/* defines a structure for isp registers values */
struct isp_reg {
	u32 	reg;    /* 32-bit address */
	u32 	val;	/* 32-bit value */
};

/*
 *Sets the callback for the ISP module done events.*/
int isp_set_callback(enum isp_callback_type type, isp_callback_t callback,
		void *arg1, void *arg2);


/*Clears  the callback for the ISP module done events. */
int isp_unset_callback(enum isp_callback_type type);

/* Verifies the clock range passed with the max/min values for MCLK divisor.*/
u32 isp_negotiate_xclka(u32 xclk);

/* Verifies the clock range passed with the max/min values for MCLK divisor.*/
u32 isp_negotiate_xclkb(u32 xclk);

/* Configures the clock with the passed MCLK divisor A.*/
u32 isp_set_xclka(u32 xclk);

/* Configures the clock with the passed MCLK divisor B.*/
u32 isp_set_xclkb(u32 xclk);

/* Returns the XCLKA in Hz. */
u32 isp_get_xclka(void);

/* Returns the XCLKB in Hz. */
u32 isp_get_xclkb(void);

/*
 * Reserves the parallel or serial interface requested for.
 * At most 2 interfaces shall be enabled at a time.
 */
int isp_request_interface(enum isp_interface_type if_t);

/* Frees the parallel or serial interface that is passed.*/
int isp_free_interface(enum isp_interface_type if_t);

struct isp_interface_config {
	/*0 - Parallell  1- CSIA, 2-CSIB to CCDC */
	enum isp_interface_type ccdc_par_ser;
	/*0- Disable, 1 - Enable, first byte->cam_d[7:0],*/
	/*2 - Enable, first byte -> cam_d[15:8]*/
	u8 par_bridge;
	/* 0 - Non Inverted, 1- Inverted*/
	u8 para_clk_pol;
	/* 0 - No Shift, 1 -CAMEXT[11:2]->CAM[8:0]*/
	/* 2 - [11:4]->[7:0]*/
	u8 dataline_shift;
	/* 0 - HS Falling, 1-HS rising, 2 - VS falling, 3 - VS rising*/
	u8 hsvs_syncdetect;
	/* VD0 Interrupt timing */
	u16 vdint0_timing;
	/* VD1 Interrupt timing */
	u16 vdint1_timing;
	/* Strobe related parameter*/
	int strobe;
	/* PreStrobe related parameter*/
	int prestrobe;
	/* Shutter related parameter*/
	int shutter;
};

struct isp_sysc	{
	char reset;
	char idle_mode;
};

/* sysconfig settings */
void isp_power_settings(struct isp_sysc);

/* Configures the ISP Control interace related parameters.*/
int isp_configure_interface(struct isp_interface_config *config);

void isp_CCDC_VD01_disable(void);
void isp_CCDC_VD01_enable(void);

/*  Acquires the ISP resource. Initialises the clocks for the first aquire.*/
int isp_get(void);

/* Releases the ISP resource. Releases the clocks also for the last release.*/
int isp_put(void);

/*Saves ISP context*/
void isp_save_context(struct isp_reg* );

/*Restores ISP context*/
void isp_restore_context(struct isp_reg* );

/*
 * Prints the values of the ISP Control Module registers
 * Also prints other debug information stored in the ISP module structure
 */
void isp_print_status(void);



#endif	/* OMAP_ISP_TOP_H */
