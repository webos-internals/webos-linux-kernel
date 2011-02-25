/*
 * Copyright (C) 2008-2009 Palm, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#ifndef __TSC2005_H
#define __TSC2005_H

#include <linux/spi/spi.h>

struct tsc2005_platform_data {
    int reset_gpio;     // reset gpio
    int reset_level;    // reset active level (0 for active low)
    int xresistence;    // Resistance on x-plate for pressure calculations.
    int penup_timeout;  // msec
};

/* Command and Control */
#define TSC_CTL_CMD                 (1 << 7)
#define TSC_CTL_READ                (1 << 0)
#define TSC_CTL_WRITE               (0 << 0)
#define TSC_CTL_SIZE_BITS           (8)
#define TSC_CMD_12BIT               (0x1 << 2)
#define TSC_CMD_STOP_AND_POWER_DOWN (TSC_CTL_CMD | (0x1 << 0))
#define TSC_CMD_XYZSCAN             (TSC_CTL_CMD | (0x0 << 3) | TSC_CMD_12BIT)
#define TSC_CMD_XYSCAN              (TSC_CTL_CMD | (0x1 << 3) | TSC_CMD_12BIT)
#define TSC_CMD_AUXONCE             (TSC_CTL_CMD | (0x5 << 3))
#define TSC_CMD_AUXSCAN             (TSC_CTL_CMD | (0x8 << 3))
#define TSC_CMD_XTEST               (TSC_CTL_CMD | (0x9 << 3))
#define TSC_CMD_YTEST               (TSC_CTL_CMD | (0xa << 3))
#define TSC_CMD_SHORTTEST           (TSC_CTL_CMD | (0xc << 3))

/* Registers */
#define TSC_REG_XDATA               (0x0 << 3)
#define TSC_REG_YDATA               (0x1 << 3)
#define TSC_REG_Z1DATA              (0x2 << 3)
#define TSC_REG_Z2DATA              (0x3 << 3)
#define TSC_REG_AUXDATA             (0x4 << 3)
#define TSC_REG_TEMP1DATA           (0x5 << 3)
#define TSC_REG_TEMP2DATA           (0x6 << 3)
#define TSC_REG_STATUS              (0x7 << 3)
#define TSC_REG_CFR0                (0xc << 3)
#define TSC_REG_CFR1                (0xd << 3)
#define TSC_REG_CFR2                (0xe << 3)
#define TSC_REG_SIZE_BITS           (16)

/* Configuration */
#define TSC_CFR0_PSM_AUTO           (0x1 << 15)
#define TSC_CFR0_PSM_HOST           (0x0 << 15)
#define TSC_CFR0_RM_12BIT           (0x1 << 13)
#define TSC_CFR0_CLK_4MHZ           (0x0 << 11)
#define TSC_CFR0_CLK_2MHZ           (0x1 << 11)
#define TSC_CFR0_CLK_1MHZ           (0x2 << 11)
#define TSC_CFR0_VSTAB_100US        (0x1 << 8)
#define TSC_CFR0_VSTAB_500MS        (0x7 << 8)
#define TSC_CFR0_CHRG_84US          (0x1 << 5)
#define TSC_CFR0_SNS_96US           (0x1 << 2)
#define TSC_CFR0_LSM                (0x1 << 0)

#define TSC_CFR1_TBM_DEFAULT        (0x1 << 8)
#define TSC_CFR1_BDLY_NONE          (0x0 << 0)
#define TSC_CFR1_BDLY_1000_SSPS     (0x1 << 0)
#define TSC_CFR1_BDLY_500_SSPS      (0x2 << 0)
#define TSC_CFR1_BDLY_250_SSPS      (0x3 << 0)
#define TSC_CFR1_BDLY_100_SSPS      (0x4 << 0)
#define TSC_CFR1_BDLY_50_SSPS       (0x5 << 0)
#define TSC_CFR1_BDLY_25_SSPS       (0x6 << 0)
#define TSC_CFR1_BDLY_10_SSPS       (0x7 << 0)

#define TSC_CFR2_PINTS_BOTH         (0x0 << 14)
#define TSC_CFR2_PINTS_DAVONLY      (0x1 << 14)
#define TSC_CFR2_PINTS_PENONLY      (0x2 << 14)
#define TSC_CFR2_MAVCTL_FLTSZ_3     (0x1 << 12)
#define TSC_CFR2_MAVCTL_WINSZ_1     (0x0 << 10)
#define TSC_CFR2_MAVENX             (0x1 << 4)
#define TSC_CFR2_MAVENY             (0x1 << 3)
#define TSC_CFR2_MAVENZ             (0x1 << 2)
#define TSC_CFR2_MAVENAUX           (0x1 << 1)
#define TSC_CFR2_MAVENTMP           (0x1 << 0)

#endif

