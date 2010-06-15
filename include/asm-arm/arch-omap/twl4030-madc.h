/*
 * twl4030-madc.h - header for TWL4030-madc
 *
 */

#ifndef __TWL4030_MADC_H_
#define __TWL4030_MADC_H_

extern int madc_enable_averaging(int channel, int enable);
extern int madc_start_conversion(int channel);
#endif


