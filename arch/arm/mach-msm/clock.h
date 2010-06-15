/* arch/arm/mach-msm/clock.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2009, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_H
#define __ARCH_ARM_MACH_MSM_CLOCK_H

#include <linux/list.h>

#include "clock-pcom.h"

#define CLKFLAG_INVERT			0x00000001
#define CLKFLAG_NOINVERT		0x00000002
#define CLKFLAG_NONEST			0x00000004
#define CLKFLAG_NORESET			0x00000008

#define CLK_FIRST_AVAILABLE_FLAG	0x00000100
#define CLKFLAG_AUTO_OFF		0x00000200
#define CLKFLAG_MIN			0x00000400
#define CLKFLAG_MAX			0x00000800

struct clk_ops {
	int (*enable)(unsigned id);
	void (*disable)(unsigned id);
	int (*set_rate)(unsigned id, unsigned rate);
	int (*set_min_rate)(unsigned id, unsigned rate);
	int (*set_max_rate)(unsigned id, unsigned rate);
	int (*set_flags)(unsigned id, unsigned flags);
	unsigned (*get_rate)(unsigned id);
	unsigned (*is_enabled)(unsigned id);
};

struct clk {
	uint32_t id;
	uint32_t count;
	uint32_t flags;
	const char *name;
	struct clk_ops *ops;
	const char *dbg_name;
	struct list_head list;
	struct device *dev;
};

#define A11S_CLK_CNTL_ADDR		(MSM_CSR_BASE + 0x100)
#define A11S_CLK_SEL_ADDR		(MSM_CSR_BASE + 0x104)
#define A11S_VDD_SVS_PLEVEL_ADDR	(MSM_CSR_BASE + 0x124)

#define A11S_CLK_CNTL_TCXO		0


#define A11S_CLK_SRC0_SEL_MASK		0x00007000
#define A11S_CLK_SRC0_SEL_SHIFT		12
#define A11S_CLK_SRC0_DIV_MASK		0x00000F00
#define A11S_CLK_SRC0_DIV_SHIFT		8
#define A11S_CLK_SRC1_SEL_MASK		0x00000070
#define A11S_CLK_SRC1_SEL_SHIFT		4
#define A11S_CLK_SRC1_DIV_MASK		0x0000000F
#define A11S_CLK_SRC1_DIV_SHIFT		0

#define PMDH_CLK_ADDR 			(MSM_CLK_CTL_BASE + 0x008C)
#define EBI1_CLK_ADDR 			(MSM_CLK_CTL_BASE + 0x002C)
#define PLL0_L_VAL			(MSM_CLK_CTL_BASE + 0x0304 + 28*0)
#define PLL1_L_VAL			(MSM_CLK_CTL_BASE + 0x0304 + 28*1)
#define PLL2_L_VAL			(MSM_CLK_CTL_BASE + 0x0304 + 28*2)
#define ARM11_RAW_CLK_DIV_REG		(MSM_CLK_CTL_BASE + 0x03BC)

/* clock IDs used by the modem processor */

#define ACPU_CLK	0   /* Applications processor clock */
#define ADM_CLK		1   /* Applications data mover clock */
#define ADSP_CLK	2   /* ADSP clock */
#define EBI1_CLK	3   /* External bus interface 1 clock */
#define EBI2_CLK	4   /* External bus interface 2 clock */
#define ECODEC_CLK	5   /* External CODEC clock */
#define EMDH_CLK	6   /* External MDDI host clock */
#define GP_CLK		7   /* General purpose clock */
#define GRP_CLK		8   /* Graphics clock */
#define I2C_CLK		9   /* I2C clock */
#define ICODEC_RX_CLK	10  /* Internal CODEX RX clock */
#define ICODEC_TX_CLK	11  /* Internal CODEX TX clock */
#define IMEM_CLK	12  /* Internal graphics memory clock */
#define MDC_CLK		13  /* MDDI client clock */
#define MDP_CLK		14  /* Mobile display processor clock */
#define PBUS_CLK	15  /* Peripheral bus clock */
#define PCM_CLK		16  /* PCM clock */
#define PMDH_CLK	17  /* Primary MDDI host clock */
#define SDAC_CLK	18  /* Stereo DAC clock */
#define SDC1_CLK	19  /* Secure Digital Card clocks */
#define SDC1_PCLK	20
#define SDC2_CLK	21
#define SDC2_PCLK	22
#define SDC3_CLK	23
#define SDC3_PCLK	24
#define SDC4_CLK	25
#define SDC4_PCLK	26
#define TSIF_CLK	27  /* Transport Stream Interface clocks */
#define TSIF_REF_CLK	28
#define TV_DAC_CLK	29  /* TV clocks */
#define TV_ENC_CLK	30
#define UART1_CLK	31  /* UART clocks */
#define UART2_CLK	32
#define UART3_CLK	33
#define UART1DM_CLK	34
#define UART2DM_CLK	35
#define USB_HS_CLK	36  /* High speed USB core clock */
#define USB_HS_PCLK	37  /* High speed USB pbus clock */
#define USB_OTG_CLK	38  /* Full speed USB clock */
#define VDC_CLK		39  /* Video controller clock */
#define VFE_MDC_CLK	40  /* Camera / Video Front End clock */
#define VFE_CLK		41  /* VFE MDDI client clock */

#define MDP_LCDC_PCLK_CLK 42
#define MDP_LCDC_PAD_PCLK_CLK 43
#define MDP_VSYNC_CLK	44

#define SPI_CLK		45
#define VFE_AXI_CLK	46

#define USB_HS2_CLK	47  /* High speed USB 2 core clock */
#define USB_HS2_PCLK	48  /* High speed USB 2 pbus clock */
#define USB_HS3_CLK	49  /* High speed USB 3 core clock */
#define USB_HS3_PCLK	50  /* High speed USB 3 pbus clock */

#define GRP_PCLK	51  /* Graphics pbus clock */

#define NR_CLKS		52

#ifdef CONFIG_DEBUG_FS
#define CLOCK_DBG_NAME(x) .dbg_name = x,
#else
#define CLOCK_DBG_NAME(x)
#endif

struct clk_ops;
extern struct clk_ops clk_ops_pcom;

#define CLOCK(clk_name, clk_id, clk_dev, clk_flags) {	\
	.name = clk_name, \
	.id = clk_id, \
	.flags = clk_flags, \
	.ops = &clk_ops_pcom, \
	.dev = clk_dev, \
	 CLOCK_DBG_NAME(#clk_id) \
	}

#define OFF CLKFLAG_AUTO_OFF
#define CLK_MIN CLKFLAG_MIN
#define CLK_MAX CLKFLAG_MAX
#define CLK_MINMAX (CLK_MIN | CLK_MAX)
//#define NR_CLKS	P_NR_CLKS

enum clkvote_client {
	CLKVOTE_ACPUCLK = 0,
	CLKVOTE_PMQOS,
	CLKVOTE_MAX,
};

int msm_clock_require_tcxo(unsigned long *reason, int nbits);
int msm_clock_get_name(uint32_t id, char *name, uint32_t size);
int ebi1_clk_set_min_rate(enum clkvote_client client, unsigned long rate);

#endif

