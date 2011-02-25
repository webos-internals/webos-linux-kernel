/*
 * linux/arch/arm/mach-omap3pe/smartreflex.h
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


/* SR Modules */
#define SR1             1
#define SR2             2

#define SR_FAIL         1
#define SR_PASS         0

#define SR_TRUE         1
#define SR_FALSE        0

#define USE_EFUSE_NVALUES       1
#define USE_TEST_NVALUES        2

#define GAIN_MAXLIMIT   16
#define R_MAXLIMIT      256

#define SR1_CLK_ENABLE  (0x1 << 6)
#define SR2_CLK_ENABLE  (0x1 << 7)

/* PRM_VP1_CONFIG */
#define PRM_VP1_CONFIG_ERROROFFSET      (0x00 << 24)
#define PRM_VP1_CONFIG_ERRORGAIN_MASK   (0xFF << 16)
#ifdef CONFIG_MACH_SIRLOIN_3630
 #define PRM_VP1_CONFIG_ERRORGAIN_OPP2   (0x0C << 16)
 #define PRM_VP1_CONFIG_ERRORGAIN_OPP3   (0x16 << 16)
 #define PRM_VP1_CONFIG_ERRORGAIN_OPP4   (0x23 << 16)
 #define PRM_VP1_CONFIG_ERRORGAIN_OPP5   (0x27 << 16)
#else
 #define PRM_VP1_CONFIG_ERRORGAIN        (0x18 << 16) /*20-4 18-3mV/% */
#endif

#define PRM_VP1_CONFIG_INITVOLTAGE      (0x30 << 8) /* 1.2 volt */
#define PRM_VPX_CONFIG_INITVOLTAGE_MASK (0xff << 8)
#define PRM_VP1_CONFIG_TIMEOUTEN        (0x1 << 3)
#define PRM_VP1_CONFIG_INITVDD          (0x1 << 2)
#define PRM_VP1_CONFIG_FORCEUPDATE      (0x1 << 1)
#define PRM_VP1_CONFIG_VPENABLE         (0x1 << 0)

#ifdef CONFIG_MACH_SIRLOIN_3630
 /* PRM_VP1_VSTEPMIN */
 #define PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN        (0x3C << 8)
 #define PRM_VP1_VSTEPMIN_VSTEPMIN               (0x01 << 0)

 /* PRM_VP1_VSTEPMAX */
 #define PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX        (0x3C << 8)
 #define PRM_VP1_VSTEPMAX_VSTEPMAX               (0x08 << 0)

 /* PRM_VP1_VLIMITTO */
 #define PRM_VP1_VLIMITTO_VDDMAX         (0x3C << 24)
 #define PRM_VP1_VLIMITTO_VDDMIN         (0x12 << 16)
 #define PRM_VP1_VLIMITTO_TIMEOUT        (0xF00 << 0) /* 200 usec */
#else
 /* PRM_VP1_VSTEPMIN */
 #define PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN        (0x01F4 << 8)
 #define PRM_VP1_VSTEPMIN_VSTEPMIN               (0x01 << 0)

 /* PRM_VP1_VSTEPMAX */
 #define PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX        (0x01F4 << 8)
 #define PRM_VP1_VSTEPMAX_VSTEPMAX               (0x04 << 0)

 /* PRM_VP1_VLIMITTO */
 #define PRM_VP1_VLIMITTO_VDDMAX         (0x3C << 24)
 #define PRM_VP1_VLIMITTO_VDDMIN         (0x0 << 16)
 #define PRM_VP1_VLIMITTO_TIMEOUT        (0xFFFF << 0)
#endif

/* PRM_VP2_CONFIG */
#define PRM_VP2_CONFIG_ERROROFFSET      (0x00 << 24)
#define PRM_VP2_CONFIG_ERRORGAIN_MASK   (0xFF << 16)
#ifdef CONFIG_MACH_SIRLOIN_3630
#define PRM_VP2_CONFIG_ERRORGAIN_OPP2   (0x0C << 16)
#define PRM_VP2_CONFIG_ERRORGAIN_OPP3   (0x16 << 16)
#else
#define PRM_VP2_CONFIG_ERRORGAIN        (0x10 << 10) /* 20-4 10-2mV/% */
#endif

#define PRM_VP2_CONFIG_INITVOLTAGE      (0x30 << 8) /* 1.2 volt */
#define PRM_VP2_CONFIG_TIMEOUTEN        (0x1 << 3)
#define PRM_VP2_CONFIG_INITVDD          (0x1 << 2)
#define PRM_VP2_CONFIG_FORCEUPDATE      (0x1 << 1)
#define PRM_VP2_CONFIG_VPENABLE         (0x1 << 0)

#ifdef CONFIG_MACH_SIRLOIN_3630
/* PRM_VP2_VSTEPMIN */
#define PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN        (0x3C << 8)
#define PRM_VP2_VSTEPMIN_VSTEPMIN               (0x01 << 0)

/* PRM_VP2_VSTEPMAX */
#define PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX        (0x3C << 8)
#define PRM_VP2_VSTEPMAX_VSTEPMAX               (0x08 << 0)

/* PRM_VP2_VLIMITTO */
#define PRM_VP2_VLIMITTO_VDDMAX         (0x30 << 24)
#define PRM_VP2_VLIMITTO_VDDMIN         (0x12 << 16)
#define PRM_VP2_VLIMITTO_TIMEOUT        (0xF00 << 0) /* 200 usec */
#else
/* PRM_VP2_VSTEPMIN */
#define PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN        (0x01F4 << 8)
#define PRM_VP2_VSTEPMIN_VSTEPMIN               (0x01 << 0)

/* PRM_VP2_VSTEPMAX */
#define PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX        (0x01F4 << 8)
#define PRM_VP2_VSTEPMAX_VSTEPMAX               (0x04 << 0)

/* PRM_VP2_VLIMITTO */
#define PRM_VP2_VLIMITTO_VDDMAX         (0x2C << 24)
#define PRM_VP2_VLIMITTO_VDDMIN         (0x0 << 16)
#define PRM_VP2_VLIMITTO_TIMEOUT        (0xFFFF << 0)
#endif

/* SRCONFIG */
#define SR1_SRCONFIG_ACCUMDATA          (0x1F4 << 22)
#define SR2_SRCONFIG_ACCUMDATA          (0x1F4 << 22)

#define SRCLKLENGTH_12MHZ_SYSCLK        0x3C
#define SRCLKLENGTH_13MHZ_SYSCLK        0x41
#define SRCLKLENGTH_19MHZ_SYSCLK        0x60
#define SRCLKLENGTH_26MHZ_SYSCLK        0x82
#define SRCLKLENGTH_38MHZ_SYSCLK        0xC0

#define SRCONFIG_SRCLKLENGTH_SHIFT      12
#ifdef CONFIG_MACH_SIRLOIN_3630
#define SRCONFIG_SENNENABLE_SHIFT       1
#define SRCONFIG_SENPENABLE_SHIFT       0
#else
#define SRCONFIG_SENNENABLE_SHIFT       5
#define SRCONFIG_SENPENABLE_SHIFT       3
#endif

#define SRCONFIG_SRENABLE               (0x01 << 11)
#define SRCONFIG_SRDISABLE              (0x00 << 11)
#define SRCONFIG_SENENABLE              (0x01 << 10)
#define SRCONFIG_ERRGEN_EN              (0x01 << 9)
#define SRCONFIG_MINMAXAVG_EN           (0x01 << 8)

#define SRCONFIG_DELAYCTRL              (0x01 << 2)
#define SRCONFIG_CLKCTRL                (0x00 << 0)

/* AVGWEIGHT */
#define SR1_AVGWEIGHT_SENPAVGWEIGHT     (0x03 << 2)
#define SR1_AVGWEIGHT_SENNAVGWEIGHT     (0x03 << 0)

#define SR2_AVGWEIGHT_SENPAVGWEIGHT     (0x01 << 2)
#define SR2_AVGWEIGHT_SENNAVGWEIGHT     (0x01 << 0)

/* NVALUERECIPROCAL */
#define NVALUERECIPROCAL_SENPGAIN_SHIFT 20
#define NVALUERECIPROCAL_SENNGAIN_SHIFT 16
#define NVALUERECIPROCAL_RNSENP_SHIFT   8
#define NVALUERECIPROCAL_RNSENN_SHIFT   0

/* ERRCONFIG */
#define SR_CLKACTIVITY_MASK             (0x03 << 20)
#define SR_ERRWEIGHT_MASK               (0x07 << 16)
#define SR_ERRMAXLIMIT_MASK             (0xFF << 8)
#define SR_ERRMINLIMIT_MASK             0xFF

#define SR_CLKACTIVITY_IOFF_FOFF        (0x00 << 20)
#define SR_CLKACTIVITY_IOFF_FON         (0x02 << 20)

#ifdef CONFIG_MACH_SIRLOIN_3630
 /* 3630 */
 #define ERRCONFIG_VPBOUNDINTEN          (0x1 << 22)
 #define ERRCONFIG_VPBOUNDINTST          (0x1 << 23)

 #define SR1_ERRWEIGHT                   (0x04 << 16)
 #define SR1_ERRMAXLIMIT                 (0x02 << 8)
 #define SR1_ERRORMINLIMIT_MASK          0xFF
 #define SR1_ERRMINLIMIT_OPP2            0xF4
 #define SR1_ERRMINLIMIT_OPP3            0xF9
 #define SR1_ERRMINLIMIT_OPP4            0xFA
 #define SR1_ERRMINLIMIT_OPP5            0xFA

 #define SR2_ERRWEIGHT                   (0x04 << 16)
 #define SR2_ERRMAXLIMIT                 (0x02 << 8)
 #define SR2_ERRORMINLIMIT_MASK          0xFF
 #define SR2_ERRMINLIMIT_OPP2            0xF4
 #define SR2_ERRMINLIMIT_OPP3            0xF9
#else
 /* 3430 */
 #define ERRCONFIG_VPBOUNDINTEN          (0x1 << 31)
 #define ERRCONFIG_VPBOUNDINTST          (0x1 << 30)

 #define SR1_ERRWEIGHT                   (0x07 << 16)
 #define SR1_ERRMAXLIMIT                 (0x02 << 8)
 #define SR1_ERRMINLIMIT                 0xFA

 #define SR2_ERRWEIGHT                   (0x07 << 16)
 #define SR2_ERRMAXLIMIT                 (0x02 << 8)
 #define SR2_ERRMINLIMIT                 0xF9
#endif


#ifdef CONFIG_MACH_SIRLOIN_3630

#define CONTROL_FUSE_OPP1G_VDD1          0x48002380
#define CONTROL_FUSE_OPP50_VDD1          0x48002384
#define CONTROL_FUSE_OPP100_VDD1         0x48002388
#define CONTROL_FUSE_OPP130_VDD1         0x48002390

#define CONTROL_FUSE_OPP50_VDD2          0x48002398
#define CONTROL_FUSE_OPP100_VDD2         0x4800239C

#else

#define CONTROL_FUSE_OPP1_VDD1           0x48002380
#define CONTROL_FUSE_OPP2_VDD1           0x48002384
#define CONTROL_FUSE_OPP3_VDD1           0x48002388
#define CONTROL_FUSE_OPP4_VDD1           0x4800238C
#define CONTROL_FUSE_OPP5_VDD1           0x48002390

#define CONTROL_FUSE_OPP1_VDD2           0x48002394
#define CONTROL_FUSE_OPP2_VDD2           0x48002398
#define CONTROL_FUSE_OPP3_VDD2           0x4800239C

#endif

#define CONTROL_FUSE_SR                  0x480023A0

/* 
 * Definitions of SR modes 
 *     SR_MODE_0   - disabled
 *     SR_MODE_15  - smartreflex v1.5 
 *     SR_MODE_30  - smartreflex v3.0 
 */
#define SR_MODE_0     0   
#define SR_MODE_15   15
#define SR_MODE_30   30

extern u8 get_mpu_iva2_vdd1_volts(int opp_num);
extern u8 get_core_l3_vdd2_volts(int opp_num);

extern struct kset power_subsys;

extern inline int loop_wait(u32 *lcnt, u32 *rcnt, u32 delay);
extern void omap_udelay(u32 udelay);

