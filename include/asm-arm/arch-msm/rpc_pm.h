/* include/asm-arm/arch-msm/rpc_pm.h
 *
 * Public interface for power management
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007 QUALCOMM Incorporated
 * Author: San Mehat <san@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _INCLUDE_ASM_ARM_ARCH_MSM_RPC_PM_H
#define _INCLUDE_ASM_ARM_ARCH_MSM_RPC_PM_H

#define PM_VREG_MSMA_ID			 0
#define PM_VREG_MSMP_ID			 1
#define PM_VREG_MSME1_ID		 2
#define PM_VREG_MSMC1_ID		 3
#define PM_VREG_MSMC2_ID		 4
#define PM_VREG_GP3_ID			 5
#define PM_VREG_MSME2_ID		 6
#define PM_VREG_GP4_ID			 7
#define PM_VREG_GP1_ID			 8
#define PM_VREG_TCXO_ID			 9
#define PM_VREG_PA_ID			 10
#define PM_VREG_RFTX_ID			 11
#define PM_VREG_RFRX1_ID		 12
#define PM_VREG_RFRX2_ID		 13
#define PM_VREG_SYNT_ID			 14
#define PM_VREG_WLAN_ID			 15
#define PM_VREG_USB_ID			 16
#define PM_VREG_BOOST_ID		 17
#define PM_VREG_MMC_ID			 18
#define PM_VREG_RUIM_ID			 19
#define PM_VREG_MSMC0_ID		 20
#define PM_VREG_GP2_ID			 21
#define PM_VREG_GP5_ID			 22
#define PM_VREG_GP6_ID			 23
#define PM_VREG_RF_ID			 24
#define PM_VREG_RF_VCO_ID		 26
#define PM_VREG_MPLL_ID			 27
#define PM_VREG_S2_ID			 28
#define PM_VREG_S3_ID			 29
#define PM_VREG_RFUBM_ID		 30
#define PM_VREG_NCP_ID			 31
#define PM_VREG_ID_INVALID		 32
#define PM_VREG_MSME_ID 		 PM_VREG_MSME1_ID
#define PM_VREG_MSME_BUCK_SMPS_ID	 PM_VREG_MSME1_ID
#define PM_VREG_MSME1_LDO_ID		 PM_VREG_MSME1_ID
#define PM_VREG_MSMC_ID			 PM_VREG_MSMC1_ID
#define PM_VREG_MSMC_LDO_ID		 PM_VREG_MSMC1_ID
#define PM_VREG_MSMC1_BUCK_SMPS_ID	 PM_VREG_MSMC1_ID
#define PM_VREG_MSME2_LDO_ID		 PM_VREG_MSME2_ID
#define PM_VREG_CAM_ID			 PM_VREG_GP1_ID
#define PM_VREG_MDDI_ID			 PM_VREG_GP2_ID
#define PM_VREG_RUIM2_ID		 PM_VREG_GP3_ID
#define PM_VREG_AUX_ID			 PM_VREG_GP4_ID
#define PM_VREG_AUX2_ID			 PM_VREG_GP5_ID
#define PM_VREG_BT_ID			 PM_VREG_GP6_ID

#define PM_VOTE_VREG_MMC_APP__MINI_SD		 1
#define PM_VOTE_VREG_MMC_APP__LCD		 2
#define PM_VOTE_VREG_MMC_APP__BT			 8
#define PM_VOTE_VREG_MMC_APP__MISC_SENSOR	 4
#define PM_VOTE_VREG_MMC_APP__MMC		 16
#define PM_VOTE_VREG_BOOST_APP__LCD		 1
#define PM_VOTE_VREG_BOOST_APP__OTG		 2
#define PM_VOTE_VREG_SYNTH_APP__CHAIN0		 1
#define PM_VOTE_VREG_SYNTH_APP__CHAIN1		 2
#define PM_VOTE_VREG_SYNTH_APP__RF_GSM		 4
#define PM_VOTE_VREG_RFRX1_APP__RF_CHAIN0	 1
#define PM_VOTE_VREG_RFRX1_APP__RF_CHAIN1	 2
#define PM_VOTE_VREG_RFRX1_APP__GPS		 4
#define PM_VOTE_VREG_RFRX1_APP__RF_GSM		 8
#define PM_VOTE_VREG_RFRX2_APP__RF		 1
#define PM_VOTE_VREG_RFRX2_APP__GPS		 2
#define PM_VOTE_VREG_PA_APP__RF			 1
#define PM_VOTE_VREG_RFTX_APP__RF		 1
#define PM_VOTE_VREG_RFTX_APP__PA_THERM		 2
#define PM_VOTE_VREG_RFTX_APP__RF_GSM		 4
#define PM_VOTE_VREG_SYNTH_APP_GPS__CHAIN0	 1
#define PM_VOTE_VREG_SYNTH_APP_GPS__CHAIN1	 2
#define PM_VOTE_VREG_WLAN_APP__BT		 1
#define PM_VOTE_VREG_RUIM_APP__RUIM		 1
#define PM_VOTE_VREG_RUIM2_APP__CAMIF		 1
#define PM_VOTE_VREG_MDDI_APP__CAMIF		 1
#define PM_VOTE_VREG_CAM_APP__CAMIF		 1
#define PM_VOTE_VREG_AUX_APP__AUX		 1
#define PM_VOTE_VREG_AUX2_APP__AUX2		 1
#define PM_VOTE_VREG_BT_APP__BT			 1

#endif
