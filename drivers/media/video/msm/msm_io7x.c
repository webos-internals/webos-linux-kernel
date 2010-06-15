/*
 * Copyright (c) 2008 QUALCOMM Incorporated.
 * Copyright (c) 2008 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * except where indicated.
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 * Author: Haibo Jeff Zhong <hzhong@qualcomm.com>
 */

#include <linux/delay.h>
#include <asm/arch/msm_iomap.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board.h>
#include <asm/arch/camera.h>

#define CAMIF_CFG_RMSK 0x1fffff
#define CAM_SEL_BMSK 0x2
#define CAM_PCLK_SRC_SEL_BMSK 0x60000
#define CAM_PCLK_INVERT_BMSK 0x80000
#define CAM_PAD_REG_SW_RESET_BMSK 0x100000

#define EXT_CAM_HSYNC_POL_SEL_BMSK 0x10000
#define EXT_CAM_VSYNC_POL_SEL_BMSK 0x8000
#define MDDI_CLK_CHICKEN_BIT_BMSK  0x80

#define CAM_SEL_SHFT 0x1
#define CAM_PCLK_SRC_SEL_SHFT 0x11
#define CAM_PCLK_INVERT_SHFT 0x13
#define CAM_PAD_REG_SW_RESET_SHFT 0x14

#define EXT_CAM_HSYNC_POL_SEL_SHFT 0x10
#define EXT_CAM_VSYNC_POL_SEL_SHFT 0xF
#define MDDI_CLK_CHICKEN_BIT_SHFT  0x7
#define APPS_RESET_OFFSET 0x00000210

static struct clk *camio_vfe_mdc_clk;
static struct clk *camio_mdc_clk;
static struct clk *camio_vfe_clk;

void msm_camio_gpio_enable(void)
{
	config_camera_on_gpios();
}

void msm_camio_gpio_disable()
{
	config_camera_off_gpios();
}

int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = -1;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk = clk_get(NULL, "vfe_mdc_clk");
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk = clk_get(NULL, "mdc_clk");
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk = clk_get(NULL, "vfe_clk");
		break;

	default:
		break;
	}

	if (clk != NULL && clk != ERR_PTR(-ENOENT)) {
		clk_enable(clk);
		rc = 0;
	}

	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = -1;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk;
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		break;

	default:
		break;
	}

	if (clk != NULL && clk != ERR_PTR(-ENOENT)) {
		clk_disable(clk);
		clk_put(clk);
		rc = 0;
	}

	return rc;
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_vfe_clk;
	if (clk != ERR_PTR(-ENOENT))
		clk_set_rate(clk, rate);
}

void msm_camio_enable(void)
{
	msm_camio_gpio_enable();

	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
}

void msm_camio_disable(void)
{
	msm_camio_gpio_disable();
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
}

void msm_camio_camif_pad_reg_reset(void)
{
	uint32_t reg;
	uint32_t mask, value;

	/* select CLKRGM_VFE_SRC_CAM_VFE_SRC:  internal source */
	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_INTERNAL);

	reg = (readl(MSM_MDC_BASE)) & CAMIF_CFG_RMSK;

	mask = CAM_SEL_BMSK |
		CAM_PCLK_SRC_SEL_BMSK |
		CAM_PCLK_INVERT_BMSK;

	value = 1 << CAM_SEL_SHFT |
		3 << CAM_PCLK_SRC_SEL_SHFT |
		0 << CAM_PCLK_INVERT_SHFT;

	writel((reg & (~mask)) | (value & mask), MSM_MDC_BASE);
	mdelay(10);

	reg = (readl(MSM_MDC_BASE)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), MSM_MDC_BASE);
	mdelay(10);

	reg = (readl(MSM_MDC_BASE)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), MSM_MDC_BASE);
	mdelay(10);

	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_EXTERNAL);
	mdelay(10);
}

void msm_camio_vfe_blk_reset(void)
{
	uint32_t val;

	/* do apps reset */
	val = readl(MSM_CLK_CTL_BASE + 0x00000210);
	val |= 0x1;
	writel(val, MSM_CLK_CTL_BASE + 0x00000210);
	mdelay(10);

	val = readl(MSM_CLK_CTL_BASE + 0x00000210);
	val &= ~0x1;
	writel(val, MSM_CLK_CTL_BASE + 0x00000210);
	mdelay(10);

	/* do axi reset */
	val = readl(MSM_CLK_CTL_BASE + 0x00000208);
	val |= 0x1;
	writel(val, MSM_CLK_CTL_BASE + 0x00000208);
	mdelay(10);

	val = readl(MSM_CLK_CTL_BASE + 0x00000208);
	val &= ~0x1;
	writel(val, MSM_CLK_CTL_BASE + 0x00000208);
	mdelay(10);
}

void msm_camio_camif_pad_reg_reset_2(void)
{
	uint32_t reg;
	uint32_t mask, value;

	reg = (readl(MSM_MDC_BASE)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), MSM_MDC_BASE);
	mdelay(10);

	reg = (readl(MSM_MDC_BASE)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), MSM_MDC_BASE);
	mdelay(10);
}

void msm_camio_clk_sel(enum msm_camio_clk_src_type srctype)
{
	struct clk *clk = NULL;

	clk = camio_vfe_clk;

	if (clk != NULL && clk != ERR_PTR(-ENOENT)) {
		switch (srctype) {
		case MSM_CAMIO_CLK_SRC_INTERNAL:
			clk_set_flags(clk, 0x00000100 << 1);
			break;

		case MSM_CAMIO_CLK_SRC_EXTERNAL:
			clk_set_flags(clk, 0x00000100);
			break;

		default:
			break;
		}
	}
}
