/*
 * drivers/media/video/omap/omap34xx-isp-smia10.c
 *
 * Copyright (C) Palm, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <asm/arch/isp.h>
#include <asm/arch/omap34xx.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include "omap34xx.h"
#include "omap34xx-isp.h"

#define CCP2_REG(base, off)		((base) + (off))

#define CCP2_CTRL(base)			CCP2_REG((base), 0x040)
#define	CTRL_MODE_MASK			(1 << 4)
#define	CTRL_MODE_CCP2			(1 << 4)
#define	CTRL_IO_OUT_SEL_MASK		(1 << 2)
#define	CTRL_IO_OUT_SEL(val)		((0x1 & (val)) << 2)
#define	CTRL_PHY_SEL_MASK		(1 << 1)
#define	CTRL_PHY_SEL(val)		((0x1 & (val)) << 1)
#define	CTRL_IF_EN_MASK			(1 << 0)
#define	CTRL_IF_EN(val)			((0x1 & (val)) << 0)

#define CCP2_LCx_IRQENABLE(base, x)	CCP2_REG((base), 0x00C + (0x2 * (x)))

#define CCP2_LCx_IRQSTATUS(base, x)	CCP2_REG((base), 0x010 + (0x2 * (x)))
#define LCx_IRQ_SHIFT(x, shift)		((shift) + ((1 & (x)) ? 16 : 0))
#define LCx_FS_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 11))
#define LCx_FS_IRQ(x, val)		((1 & (val)) << LCx_IRQ_SHIFT((x), 11))
#define LCx_LE_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 10))
#define LCx_LS_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 9))
#define LCx_FE_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 8))
#define LCx_FE_IRQ(x, val)		((1 & (val)) << LCx_IRQ_SHIFT((x), 8))
#define LCx_FW_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 2))
#define LCx_FW_IRQ(x, val)		((1 & (val)) << LCx_IRQ_SHIFT((x), 2))
#define LCx_COUNT_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 7))
#define LCx_COUNT_IRQ(x, val)		((1 & (val)) << LCx_IRQ_SHIFT((x), 7))
#define LCx_FIFO_OVF_IRQ_MASK(x)	(1 << LCx_IRQ_SHIFT((x), 5))
#define LCx_CRC_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 4))
#define LCx_FSP_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 3))
#define LCx_FW_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 2))
#define LCx_FSC_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 1))
#define LCx_FSC_IRQ(x, val)		((1 & (val)) << LCx_IRQ_SHIFT((x), 1))
#define LCx_SSC_IRQ_MASK(x)		(1 << LCx_IRQ_SHIFT((x), 0))

#define CCP2_LCx_CTRL(base, x)		CCP2_REG((base), 0x050 + (0x30 * (x)))
#define	LCx_CTRL_COUNT_MASK		(0xF << 24)
#define	LCx_CTRL_COUNT(val)		((0xF & (val)) << 24)
#define	LCx_CTRL_PING_PONG_MASK		(1 << 17)
#define	LCx_CTRL_COUNT_UNLOCK_MASK	(1 << 16)
#define	LCx_CTRL_COUNT_UNLOCK(val)	((0x1 & (val)) << 16)
#define	LCx_CTRL_FORMAT_MASK		(0x1F << 3)
#define	LCx_CTRL_FORMAT(val)		((0x1F & (val)) << 3)
#define	LCx_CTRL_FORMAT_RAW10		0x14
#define	LCx_CTRL_FORMAT_RAW10_VP	0x16
#define	LCx_CTRL_CRC_EN_MASK		(1 << 2)
#define	LCx_CTRL_CRC_EN(val)		((0x1 & (val)) << 2)
#define LCx_CTRL_REGION_EN_MASK		(1 << 1)
#define LCx_CTRL_REGION_EN(val)		((0x1 & (val)) << 1)
#define LCx_CTRL_CHAN_EN_MASK		(1 << 0)
#define LCx_CTRL_CHAN_EN(val)		((0x1 & (val)) << 0)

#define CCP2_LCx_DAT_OFST(base, x)	CCP2_REG((base), 0x078 + (0x30 * (x)))

#define CCP2_LCx_SOF_ADDR(base, x)	CCP2_REG((base), 0x060 + (0x30 * (x)))
#define CCP2_LCx_EOF_ADDR(base, x)	CCP2_REG((base), 0x064 + (0x30 * (x)))
#define CCP2_LCx_DAT_PING_ADDR(base, x)	CCP2_REG((base), 0x070 + (0x30 * (x)))
#define CCP2_LCx_DAT_PONG_ADDR(base, x)	CCP2_REG((base), 0x074 + (0x30 * (x)))
#define LCx_ADDR_MASK			0xFFFFFFD0
#define LCx_ADDR(val)			(0xFFFFFFD0 & (val))
#define LCx_ADDR_ALIGN			64

#define CCP2_LCx_DAT_START(base, x)	CCP2_REG((base), 0x068 + (0x30 * (x)))
#define CCP2_LCx_DAT_SIZE(base, x)	CCP2_REG((base), 0x06C + (0x30 * (x)))
#define	LCx_VERT(val)			((0xFFF & (val)) << 16)

#define CCP2_LCx_STAT_START(base, x)	CCP2_REG((base), 0x058 + (0x30 * (x)))
#define CCP2_LCx_STAT_SIZE(base, x)	CCP2_REG((base), 0x05C + (0x30 * (x)))
#define	LCx_SOF_MASK			(0xFFF << 0)
#define	LCx_SOF(val)			((0xFFF & (val)) << 0)

#define CCP2_REVISION(base)		CCP2_REG((base), 0x000)

#define CCP2_SYSCONFIG(base)		CCP2_REG((base), 0x004)
#define MSTANDBY_MODE_MASK		(3 << 12)
#define	MSTANDBY_MODE(val)		((0x3 & (val)) << 12)
#define MSTANDBY_MODE_FORCE_STANDBY	0x0
#define MSTANDBY_MODE_NO_STANDBY	0x1
#define MSTANDBY_MODE_SMART_STANDBY	0x2
#define AUTO_IDLE_MASK			(1 << 0)
#define	AUTO_IDLE(val)			((0x1 & (val)) << 0)

#define CCP2_SYSSTATUS(base)		CCP2_REG((base), 0x008)

#define	CSIRXFE_CSIB_RESET_MASK		(1 << 13)
#define	CSIRXFE_CSIB_PWRDNZ_MASK	(1 << 12)
#define	CSIRXFE_CSIB_SELFORM_MASK	(1 << 10)
#define	CSIRXFE_CSIB_SELFORM(val)	((0x3 & (val)) << 10)

#define ISP_BASE			0x480BC000
#define ISP_REG(off)			(ISP_BASE + (off))

#define ISP_IRQ0ENABLE			ISP_REG(0x0C)
#define ISP_IRQ0STATUS			ISP_REG(0x10)
#define	CSIB_IRQ_MASK			(0x1 << 4)

#define SPEW(level, args...)					\
	do {							\
		if (omap34xx_isp_smia10_spew_level >= level)	\
			printk(KERN_DEBUG "SMIA10:\t" args);	\
	} while (0)

struct omap34xx_isp_smia10_data {
	u8	control_csirxfe_csib_selform;
	u8	ccp2_sysconfig_mstandby_mode;
	/* TODO: revisit */
	u8	ccp2_ctrl_io_out_sel;
	u8	ccp2_ctrl_phy_sel;
	u8	ccp2_lc0_ctrl_format;
	u8	ccp2_lc0_ctrl_crc_en;
	u16	ccp2_lc0_stat_size_sof;
	u16	ccp2_lc0_dat_start_vert;
	u16	ccp2_lc0_dat_size_vert;

	spinlock_t		lock;
	unsigned int		base;
	unsigned long		frames;
	unsigned long		errors;
	void			*sof_cpu;
	size_t			sof_len;
	dma_addr_t		sof_dma;
	dma_addr_t		sof_isp;
};

static int omap34xx_isp_smia10_spew_level = 0;

#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
static ssize_t
omap34xx_isp_smia10_show_spew_level(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t i;

	i = snprintf(buf, PAGE_SIZE, "%d\n", omap34xx_isp_smia10_spew_level);

	return (i + 1);
}

static ssize_t
omap34xx_isp_smia10_store_spew_level(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char *endp;

	omap34xx_isp_smia10_spew_level = simple_strtoul(buf, &endp, 10);

	return (count);
}

DEVICE_ATTR(smia10_spew_level, S_IRUGO|S_IWUGO,
		omap34xx_isp_smia10_show_spew_level,
		omap34xx_isp_smia10_store_spew_level);

static inline int
omap34xx_isp_smia10_create_spew_level(struct device *dev)
{
	return (device_create_file(dev, &dev_attr_smia10_spew_level));
}

static inline void
omap34xx_isp_smia10_remove_spew_level(struct device *dev)
{
	device_remove_file(dev, &dev_attr_smia10_spew_level);
}
#else // !CONFIG_VIDEO_OMAP34XX_ISP_DBG
static inline int
omap34xx_isp_smia10_create_spew_level(struct device *dev)
{
	return (1);
}

static inline void
omap34xx_isp_smia10_remove_spew_level(struct device *dev)
{
}
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG

static void omap34xx_isp_smia10_copy_sof(struct omap34xx_isp_ccp2_phy *phy,
						void *buf, size_t len)
{
	struct omap34xx_isp_smia10_data *dat;

	dat = container_of(phy, struct omap34xx_isp_input, phy.ccp2)->mst.priv;

	/* invalidate sof cache lines, then copy: ~50us */
	dma_sync_single_for_cpu(NULL, dat->sof_dma, dat->sof_len, DMA_FROM_DEVICE);
	memcpy(buf, dat->sof_cpu, min(len, dat->sof_len));
}

static int
omap34xx_isp_smia10_isr(struct omap34xx_isp_input *input)
{
	unsigned int lc0;
	unsigned int irq0;
	struct omap34xx_isp_smia10_data *data = input->mst.priv;

	irq0 = omap_readl(ISP_IRQ0STATUS);
	irq0 &= CSIB_IRQ_MASK;
	omap_writel(irq0, ISP_IRQ0STATUS);

	if (!irq0)
		goto exit;

	lc0 = omap_readl(CCP2_LCx_IRQSTATUS(data->base, 0));
	omap_writel(lc0, CCP2_LCx_IRQSTATUS(data->base, 0));

	SPEW(3, "CCP2_LC0_IRQSTATUS=0x%08X(%s,%s,%s,%s)\n",
		lc0,
		LCx_FS_IRQ_MASK(0) & lc0 ? __stringify(FS) : "",
		LCx_FE_IRQ_MASK(0) & lc0 ? __stringify(FE) : "",
		LCx_FSC_IRQ_MASK(0) & lc0 ? __stringify(FSC) : "",
		LCx_FW_IRQ_MASK(0) & lc0 ? __stringify(FW) : "");

	if ((LCx_FS_IRQ_MASK(0) & lc0) && input->finalize_stat_buf) {
		omap34xx_v4l2_device_videobuf_done(&input->isp_v4l2_dev->dev,
				STATE_DONE);
		input->finalize_stat_buf = 0;
	}

	if (LCx_FE_IRQ_MASK(0) & lc0)
		++data->frames;

	if ((LCx_FW_IRQ_MASK(0) | LCx_FSC_IRQ_MASK(0)) & lc0)
		++data->errors;
exit:
	return (!!irq0);
}

static void omap34xx_isp_smia10_configure(struct omap34xx_isp_input *in)
{
	unsigned int val;
	unsigned int bits;
	struct omap34xx_isp_smia10_data *data = in->mst.priv;

	SPEW(1, "+++ %s\n", __func__);

	val = MSTANDBY_MODE(data->ccp2_sysconfig_mstandby_mode);
	bits = MSTANDBY_MODE_MASK;
	omap_masked_writel(CCP2_SYSCONFIG(data->base), val, bits);

	val = CTRL_MODE_CCP2;
	bits = CTRL_MODE_MASK;
	val |= CTRL_IO_OUT_SEL(data->ccp2_ctrl_io_out_sel);
	bits |= CTRL_IO_OUT_SEL_MASK;
	val |= CTRL_PHY_SEL(data->ccp2_ctrl_phy_sel);
	bits |= CTRL_PHY_SEL_MASK;
	omap_masked_writel(CCP2_CTRL(data->base), val, bits);

	val = LCx_CTRL_FORMAT(data->ccp2_lc0_ctrl_format);
	bits = LCx_CTRL_FORMAT_MASK;
	val |= LCx_CTRL_CRC_EN(data->ccp2_lc0_ctrl_crc_en);
	bits |= LCx_CTRL_CRC_EN_MASK;
	val |= LCx_CTRL_REGION_EN(1);
	bits |= LCx_CTRL_REGION_EN_MASK;
	omap_masked_writel(CCP2_LCx_CTRL(data->base, 0), val, bits);

	val = LCx_VERT(data->ccp2_lc0_dat_start_vert);
	omap_writel(val, CCP2_LCx_DAT_START(data->base, 0));

	val = LCx_SOF(data->ccp2_lc0_stat_size_sof);
	bits = LCx_SOF_MASK;
	omap_masked_writel(CCP2_LCx_STAT_SIZE(data->base, 0), val, bits);

	bits = LCx_FS_IRQ(0, 1) | LCx_FE_IRQ(0, 1) | LCx_FSC_IRQ(0, 1)
		| LCx_FW_IRQ(0, 1);
	omap_setl(CCP2_LCx_IRQENABLE(data->base, 0), bits);
	omap_setl(ISP_IRQ0ENABLE, CSIB_IRQ_MASK);

	bits = CSIRXFE_CSIB_SELFORM(data->control_csirxfe_csib_selform);
	CONTROL_CSIRXFE |= bits;

	SPEW(1, "--- %s\n", __func__);
}

static int
omap34xx_isp_smia10_ioctl_streamon(struct omap34xx_v4l2_int_master *mst)
{
	int rc;
	unsigned int val;
	struct v4l2_format fmt;
	struct omap34xx_isp_smia10_data *data = mst->priv;

	SPEW(1, "+++ %s\n", __func__);

	if ((rc = omap34xx_v4l2_int_slave_ioctl_g_fmt(mst, &fmt)))
		goto exit;

	if (!(data->sof_isp =
		omap34xx_isp_mmu_map(data->sof_dma, data->sof_len))) {
		rc = -ENOMEM;
		goto exit;
	}

	if ((rc = omap34xx_v4l2_int_slave_ioctl_streamon(mst))) {
		omap34xx_isp_mmu_unmap(data->sof_isp);
		goto exit;
	}

	val = LCx_VERT(fmt.fmt.pix.height);
	omap_writel(val, CCP2_LCx_DAT_SIZE(data->base, 0));
	SPEW(2, "CCP2_LC0_DAT_SIZE: VERT=%u\n", fmt.fmt.pix.height);

	omap_writel(LCx_ADDR(data->sof_isp), CCP2_LCx_SOF_ADDR(data->base, 0));
	SPEW(2, "CCP2_LC0_SOF_ADDR: ADDR=0x%08X\n", LCx_ADDR(data->sof_isp));

	/* Status line memory writing offset is set from DAT_OFST aswell */
	val = ALIGN(fmt.fmt.pix.bytesperline, 0x20);
	omap_writel(val, CCP2_LCx_DAT_OFST(data->base, 0));
	data->frames = data->errors = 0;

	CONTROL_CSIRXFE |= CSIRXFE_CSIB_PWRDNZ_MASK;
	udelay(20);
	CONTROL_CSIRXFE |= CSIRXFE_CSIB_RESET_MASK;
	omap_setl(CCP2_CTRL(data->base), CTRL_IF_EN_MASK);
exit:
	SPEW(1, "--- %s: rc=%d sof=0x%08X\n", __func__, rc, data->sof_isp);

	return (rc);
}

static int
omap34xx_isp_smia10_ioctl_streamoff(struct omap34xx_v4l2_int_master *mst)
{
	struct omap34xx_isp_smia10_data *data = mst->priv;

	SPEW(1, "+++ %s\n", __func__);

	omap_clearl(CCP2_CTRL(data->base), CTRL_IF_EN_MASK);
	CONTROL_CSIRXFE &= ~CSIRXFE_CSIB_RESET_MASK;
	CONTROL_CSIRXFE &= ~CSIRXFE_CSIB_PWRDNZ_MASK;

	(void)omap34xx_v4l2_int_slave_ioctl_streamoff(mst);

	omap34xx_isp_mmu_unmap(data->sof_isp);

	SPEW(1, "--- %s: frames=%lu errors=%lu\n", __func__, data->frames,
		data->errors);

	return (0);
}

int
omap34xx_isp_smia10_input_probe(struct omap34xx_isp_input *input)
{
	int rc;
	unsigned int idx;
	struct resource *res;
	struct platform_device *pdev = input->mst.parent;
	struct omap34xx_isp_smia10_data *data;
	struct omap34xx_isp_platform_data *pdata = pdev->dev.platform_data;

	idx = OMAP34XX_CCP2_RESOURCE_MEM;
	res = platform_get_resource(pdev, IORESOURCE_MEM, idx);

	if (!pdata || !res) {
		rc = -ENODEV;
		goto exit;
	}
	if (!(data = kzalloc(sizeof(*data), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	data->base = res->start;
	data->sof_len = pdata->smia10_sof_len;
	data->control_csirxfe_csib_selform =
		pdata->control_csirxfe_csib_selform;
	data->ccp2_sysconfig_mstandby_mode =
		pdata->ccp2_sysconfig_mstandby_mode;
	data->ccp2_ctrl_io_out_sel = pdata->ccp2_ctrl_io_out_sel;
	data->ccp2_ctrl_phy_sel = pdata->ccp2_ctrl_phy_sel;
	data->ccp2_lc0_ctrl_format = pdata->ccp2_lc0_ctrl_format;
	data->ccp2_lc0_ctrl_crc_en = pdata->ccp2_lc0_ctrl_crc_en;
	data->ccp2_lc0_stat_size_sof = pdata->ccp2_lc0_stat_size_sof;
	data->ccp2_lc0_dat_start_vert = pdata->ccp2_lc0_dat_start_vert;
	spin_lock_init(&data->lock);

	input->isr = omap34xx_isp_smia10_isr;
	input->configure = omap34xx_isp_smia10_configure;
	input->phy.ccp2.copy_sof = omap34xx_isp_smia10_copy_sof;
	input->mst.priv = data;
	input->mst.ioctl_streamon = omap34xx_isp_smia10_ioctl_streamon;
	input->mst.ioctl_streamoff = omap34xx_isp_smia10_ioctl_streamoff;

	data->sof_cpu = (void *)__get_dma_pages(GFP_KERNEL,
						get_order(data->sof_len));
	if (!data->sof_cpu) {
		rc = -ENOMEM;
		goto __get_dma_pages_failed;
	}

	data->sof_dma = virt_to_dma(NULL, data->sof_cpu);

	rc = omap34xx_v4l2_int_master_register(&input->mst, V4L2_INT_SMIA10);
	if (rc) goto omap34xx_v4l2_int_master_register_failed;

	rc = omap34xx_isp_smia10_create_spew_level(&input->mst.parent->dev);
	if (rc)	goto omap34xx_isp_smia10_create_spew_level_failed;

	goto exit;

omap34xx_isp_smia10_create_spew_level_failed:
	v4l2_int_device_unregister(&input->mst.dev);
omap34xx_v4l2_int_master_register_failed:
	free_pages((unsigned long)data->sof_cpu, get_order(data->sof_len));
__get_dma_pages_failed:
	kfree(data);
exit:
	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_smia10_input_probe);

int
omap34xx_isp_smia10_input_remove(struct omap34xx_isp_input *input)
{
	struct omap34xx_isp_smia10_data *data = input->mst.priv;

	omap34xx_isp_smia10_remove_spew_level(&input->mst.parent->dev);
	v4l2_int_device_unregister(&input->mst.dev);
	free_pages((unsigned long)data->sof_cpu, get_order(data->sof_len));
	kfree(data);

	return (0);
}
EXPORT_SYMBOL(omap34xx_isp_smia10_input_remove);
