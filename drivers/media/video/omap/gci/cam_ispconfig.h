/*
 * drivers/media/video/omap/gci/cam_ispconfig.h
 *
 * Header file for Camera configurations 
 * for TI's OMAP3430 Camera ISP
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

#ifndef CAM_ISPCONFIG_H
#define CAM_ISPCONFIG_H

struct aexp_cfg{
	unsigned int dummy;
};

/*****************************************************************/
/*APIs to be called from ISP state controller */
/*****************************************************************/

/* Sets the divisor for xclka, xclkb */
int camispcfg_set_xclk(unsigned int xclka_b,unsigned int div);

/* Sets the sensor related settings in ISPCTRL registers */
int camispcfg_set_ispif(unsigned char addr, unsigned int val);

/* Sets the sensor related settings in CCDC registers */
int camispcfg_set_ccdc(unsigned char addr, unsigned int val);

/* Sets the sensor related settings in Auto Exposure registers */
int camispcfg_set_aexp(struct aexp_cfg);

/* Sets the sensor related settings in Auto WhiteBalance registers */
int camispcfg_set_awb(struct awb_cfg);

#endif	/* ifndef CAM_ISPCONFIG_H */

