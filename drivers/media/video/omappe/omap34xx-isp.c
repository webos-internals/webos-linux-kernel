/*
 * drivers/media/video/omap/omap34xx-isp.c
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/arch/isp.h>
#include <asm/arch/resource.h>
#include "omap34xx.h"
#include "omap34xx-isp.h"

#define ISP_BASE		0x480BC000
#define ISP_REG(off)		(ISP_BASE + (off))

#define ISP_SYSCONFIG		ISP_REG(0x04)
#define	MIDLE_MODE_MASK		(0x3 << 12)
#define	MIDLE_MODE(val)		((0x3 & (val)) << 12)
#define	SOFT_RESET_MASK		(0x1 << 1)
#define	AUTO_IDLE_MASK		(0x1 << 0)
#define	AUTO_IDLE(val)		((0x1 & (val)) << 0)

#define ISP_SYSSTATUS		ISP_REG(0x08)
#define	RESET_DONE_MASK		(0x1 << 0)

#define ISP_CTRL		ISP_REG(0x40)
#define	SBL_WR1_RAM_EN_MASK	(0x1 << 19)
#define	SBL_WR1_RAM_EN(val)	((0x1 & (val)) << 19)
#define	SYNC_DETECT_MASK	(0x3 << 14)
#define	SYNC_DETECT(val)	((0x3 & (val)) << 14)
#define	CBUFF_AUTOGATING_MASK	(0x1 << 9)
#define	CBUFF_AUTOGATING(val)	((0x1 & (val)) << 9)
#define	PAR_SER_CLK_SEL_MASK	(0x3 << 0)
#define	PAR_SER_CLK_SEL(val)	((0x3 & (val)) << 0)

#define TCTRL_CTRL		ISP_REG(0x50)
#define	INSEL_MASK		(0x3 << 27)
#define	INSEL(val)		((0x3 & (val)) << 27)
#define	STRBEN_MASK		(0x1 << 23)
#define	STRBEN(val)		((0x1 & (val)) << 23)
#define	DIVC_MASK		(0x1FF << 10)
#define	DIVC(val)		((0x1FF & (val)) << 10)

#define TCTRL_FRAME		ISP_REG(0x54)
#define	STRB_MASK		(0x3F << 12)
#define	STRB(val)		((0x3F & (val)) << 12)

#define TCTRL_STRB_DELAY	ISP_REG(0x5C)
#define	DELAY_MASK		(0x1FFFFFF << 0)
#define	DELAY(val)		((0x1FFFFFF & (val)) << 0)

#define TCTRL_STRB_LENGTH	ISP_REG(0x68)
#define	LENGTH_MASK		(0xFFFFFF << 0)
#define	LENGTH(val)		((0xFFFFFF & (val)) << 0)

#define CCDC_BASE		0x480BC600
#define CCDC_REG(off)		(CCDC_BASE + (off))

#define CCDC_SYN_MODE		CCDC_REG(0x08)
#define VDHDEN_MASK		(0x1 << 16)
#define	VDHDEN(val)		((0x1 & (val)) << 16)
#define DATSIZ_MASK		(0x7 << 8)
#define	DATSIZ(val)		((0x7 & (val)) << 8)

#define CCDC_DCSUB		CCDC_REG(0x34)
#define DCSUB_MASK		(0x3FFF << 0)
#define	DCSUB(val)		((0x3FFF & (val)) << 0)

#define CCDC_CFG		CCDC_REG(0x54)
#define VDLC_MASK		(0x1 << 15)
#define BSWD_MASK		(0x1 << 12)
#define BSWD(val)		((0x1 & (val)) << 12)

#define CCDC_FMTCFG		CCDC_REG(0x58)
#define VPIN_MASK		(0x7 << 12)
#define	VPIN(val)		((0x7 & (val)) << 12)

#define INVARIANT(cond)						\
	do {							\
		if (!(cond)) {					\
			printk(KERN_DEBUG "!(" # cond ")\n");	\
			BUG();					\
		}						\
	} while (0)

#define SPEW(level, args...)					\
	do {							\
		if (omap34xx_isp_spew_level >= level)		\
			printk(KERN_DEBUG "ISP:\t" args);	\
	} while (0)

enum {
	OMAP34XX_ISP_VIDEO_DEVICE,
	OMAP34XX_ISP_STATS_DEVICE,
	OMAP34XX_ISP_DEVICES
};

struct omap34xx_isp_data {
	u8	isp_sysconfig_midle_mode;
	u8	isp_sysconfig_auto_idle;
	u8	isp_ctrl_sync_detect;
	u8	isp_ctrl_cbuff_autogating;
	u8	isp_ctrl_par_ser_clk_sel;

	u8	tctrl_frame_strb;
	u32	tctrl_strb_delay;
	u32	tctrl_strb_length;
	u8	tctrl_ctrl_insel;
	u16	tctrl_ctrl_divc;

	u8	ccdc_pcr_busy;
	u8	ccdc_syn_mode_vdhden;
	u8	ccdc_syn_mode_datsiz;
	u8	ccdc_cfg_bswd;
	u8	ccdc_fmtcfg_vpin;

	struct clk			*iclk;
	struct mutex			lock;
	unsigned int			files;
	struct constraint_handle	*vdd1_co;
	struct constraint_handle	*vdd2_co;
	struct constraint_handle	*lat_co;
	struct omap34xx_dma_pool	dma_pool;
	struct omap34xx_isp_input	inputs[OMAP34XX_ISP_INPUTS];
	struct omap34xx_isp_v4l2_device	devs[OMAP34XX_ISP_DEVICES];
};

static struct constraint_id vdd1_co_id = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd1_opp",
};

static struct constraint_id vdd2_co_id = {
	.type = RES_OPP_CO,
	.data = (void *)"vdd2_opp",
};

static struct constraint_id lat_co_id = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};

static int omap34xx_isp_spew_level = 0;

#ifdef CONFIG_VIDEO_OMAP34XX_ISP_DBG
static ssize_t
omap34xx_isp_show_spew_level(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t i;

	i = snprintf(buf, PAGE_SIZE, "%d\n", omap34xx_isp_spew_level);

	return (i + 1);
}

static ssize_t
omap34xx_isp_store_spew_level(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	char *endp;

	omap34xx_isp_spew_level = simple_strtoul(buf, &endp, 10);

	return (count);
}

static struct device_attribute omap34xx_isp_attr_spew_level =
	__ATTR(spew_level, S_IRUGO|S_IWUGO, omap34xx_isp_show_spew_level,
		omap34xx_isp_store_spew_level);

static inline int
omap34xx_isp_create_spew_level(struct device *dev)
{
	return (device_create_file(dev, &omap34xx_isp_attr_spew_level));
}

static inline void
omap34xx_isp_remove_spew_level(struct device *dev)
{
	device_remove_file(dev, &omap34xx_isp_attr_spew_level);
}
#else // !CONFIG_VIDEO_OMAP34XX_ISP_DBG
static inline int
omap34xx_isp_create_spew_level(struct device *dev)
{
	return (1);
}

static inline void
omap34xx_isp_remove_spew_level(struct device *dev)
{
}
#endif // CONFIG_VIDEO_OMAP34XX_ISP_DBG

int
omap34xx_isp_videobuf_init(struct omap34xx_v4l2_device *dev,
				struct videobuf_buffer *qbuf)
{
	int rc = -ENOMEM;
	dma_addr_t addr = 0;
	struct omap34xx_videobuf *vbuf = get_omap34xx_buf(qbuf);
	struct omap34xx_isp_buffer *buf;

	buf = container_of(qbuf, struct omap34xx_isp_buffer, qbuf);

	SPEW(1, "+++ %s\n", __func__);

	/* TODO: use ERR_PTR */
	if (vbuf->dev_addr) {
		if (!(addr = omap34xx_isp_mmu_map(vbuf->dev_addr,
							qbuf->bsize)))
			goto exit;
	}
	else if (vbuf->sg_list) {
		if (!(addr = omap34xx_isp_mmu_map_sg(vbuf->sg_list,
							vbuf->nr_pages)))
			goto exit;
	}
	else
		BUG();

	buf->addr = addr;
	rc = 0;
exit:
	SPEW(1, "%s: index=%d isp=0x%08X rc=%d\n", __func__, qbuf->i,
		buf->addr, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_videobuf_init);

void
omap34xx_isp_videobuf_release(struct omap34xx_v4l2_device *dev,
				struct videobuf_buffer *qbuf)
{
	struct omap34xx_isp_buffer *buf;

	buf = container_of(qbuf, struct omap34xx_isp_buffer, qbuf);

	SPEW(1, "+++ %s: addr=0x%08X\n", __func__, buf->addr);

	omap34xx_isp_mmu_unmap(buf->addr);

	SPEW(1, "--- %s\n", __func__);
}
EXPORT_SYMBOL(omap34xx_isp_videobuf_release);

int
omap34xx_isp_vidioc_enum_input(struct omap34xx_v4l2_device *dev,
				struct v4l2_input *input)
{
	int rc;
	struct omap34xx_isp_data *data = platform_get_drvdata(dev->parent);

	SPEW(1, "+++ %s\n", __func__);

	if (OMAP34XX_ISP_INPUTS <= input->index) {
		rc = -EINVAL;
		goto exit;
	}

	strncpy(input->name, data->inputs[input->index].mst.dev.name,
		sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	/* NOTE: we use this to mean disconnected */
	if (!data->inputs[input->index].mst.slv)
		input->status = V4L2_IN_ST_NO_SIGNAL;

	rc = 0;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_vidioc_enum_input);

int
omap34xx_isp_vidioc_g_input(struct omap34xx_v4l2_device *dev,
				unsigned int *i)
{
	int rc;
	struct omap34xx_isp_data *data = platform_get_drvdata(dev->parent);
	struct omap34xx_isp_v4l2_device *idev;

	idev = container_of(dev, struct omap34xx_isp_v4l2_device, dev);

	SPEW(1, "+++ %s\n", __func__);

	for (*i = 0;; *i += 1) {
		if (idev->input == &data->inputs[*i])
			break;
	}

	rc = 0;

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_vidioc_g_input);

int
omap34xx_isp_vidioc_s_input(struct omap34xx_v4l2_device *dev,
				unsigned int i)
{
	int rc;
	struct omap34xx_isp_data *data = platform_get_drvdata(dev->parent);
	struct omap34xx_isp_v4l2_device *idev;

	idev = container_of(dev, struct omap34xx_isp_v4l2_device, dev);

	/* TODO: handle 1st in device to invalidate format */
	SPEW(1, "+++ %s\n", __func__);

	if (OMAP34XX_ISP_INPUTS <= i) {
		rc = -EINVAL;
		goto exit;
	}

	idev->input = &data->inputs[i];
	rc = 0;
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_vidioc_s_input);

static void omap34xx_isp_configure(struct omap34xx_isp_data *data)
{
	unsigned int val;
	unsigned int bits;

	val = MIDLE_MODE(data->isp_sysconfig_midle_mode);
	val |= AUTO_IDLE(data->isp_sysconfig_auto_idle);
	bits = MIDLE_MODE_MASK | AUTO_IDLE_MASK;
	omap_masked_writel(ISP_SYSCONFIG, val, bits);
	SPEW(2, "ISP_SYSCONFIG: MIDLE_MODE=%u AUTO_IDLE=%u\n",
		data->isp_sysconfig_midle_mode, data->isp_sysconfig_auto_idle);

	/* TODO: is this video only? */
	val = SBL_WR1_RAM_EN_MASK;
	bits = SBL_WR1_RAM_EN_MASK;
	val |= SYNC_DETECT(data->isp_ctrl_sync_detect);
	bits |= SYNC_DETECT_MASK;
	val |= CBUFF_AUTOGATING(data->isp_ctrl_cbuff_autogating);
	bits |= CBUFF_AUTOGATING_MASK;
	val |= PAR_SER_CLK_SEL(data->isp_ctrl_par_ser_clk_sel);
	bits |= PAR_SER_CLK_SEL_MASK;
	omap_masked_writel(ISP_CTRL, val, bits);
	SPEW(2, "ISP_CTRL: SYNC_DETECT=%u CBUFF_AUTOGATING=%u"
		" PAR_SER_CLK_SEL=%u\n", data->isp_ctrl_sync_detect,
		data->isp_ctrl_cbuff_autogating,
		data->isp_ctrl_par_ser_clk_sel);

	val = INSEL(data->tctrl_ctrl_insel);
	val |= DIVC(data->tctrl_ctrl_divc);
	omap_masked_writel(TCTRL_CTRL, val, INSEL_MASK | DIVC_MASK);
	SPEW(2, "TCTRL_CTRL: INSEL=%u DIVC=%u\n", data->tctrl_ctrl_insel,
		data->tctrl_ctrl_divc);

	val = STRB(data->tctrl_frame_strb);
	omap_masked_writel(TCTRL_FRAME, val, STRB_MASK);
	SPEW(2, "TCTRL_FRAME: STRB=%u\n", data->tctrl_frame_strb);

	val = DELAY(data->tctrl_strb_delay);
	omap_masked_writel(TCTRL_STRB_DELAY, val, DELAY_MASK);
	SPEW(2, "TCTRL_STRB_DELAY: DELAY=%u\n", data->tctrl_strb_delay);

	val = LENGTH(data->tctrl_strb_length);
	omap_masked_writel(TCTRL_STRB_LENGTH, val, LENGTH_MASK);
	SPEW(2, "TCTRL_STRB_LENGTH: LENGTH=%u\n", data->tctrl_strb_length);

	/* NOTE: this bit must be set to 1 if the CCDC is to be used */
	omap_setl(CCDC_CFG, VDLC_MASK);

	val = VDHDEN(data->ccdc_syn_mode_vdhden);
	val |= DATSIZ(data->ccdc_syn_mode_datsiz);
	omap_masked_writel(CCDC_SYN_MODE, val, VDHDEN_MASK | DATSIZ_MASK);
	SPEW(2, "CCDC_SYN_MODE: VDHDEN=%u DATSIZ=%u\n",
		data->ccdc_syn_mode_vdhden, data->ccdc_syn_mode_datsiz);

	/* NOTE: this bit must be set to 1 if the CCDC is to be used */
	val = VDLC_MASK;
	val |= BSWD(data->ccdc_cfg_bswd);
	omap_masked_writel(CCDC_CFG, val, VDLC_MASK | BSWD_MASK);
	SPEW(2, "CCDC_CFG: BSWD=%u\n", data->ccdc_cfg_bswd);

	val = VPIN(data->ccdc_fmtcfg_vpin);
	omap_masked_writel(CCDC_FMTCFG, val, VPIN_MASK);
	SPEW(2, "CCDC_FMTCFG: VPIN=%u\n", data->ccdc_fmtcfg_vpin);
}

static int omap34xx_isp_resume(struct omap34xx_isp_data *data)
{
	int i;
	int rc;
	struct omap34xx_v4l2_int_master *mst;

	SPEW(1, "+++ %s\n", __func__);

	/* constraint values are from ti reference code */
	if ((rc = constraint_set(data->vdd1_co, CO_VDD1_OPP3)))
		goto exit;

	if ((rc = constraint_set(data->vdd2_co, CO_VDD2_OPP3)))
		goto constraint_set_vdd2_failed;

	if ((rc = constraint_set(data->lat_co, CO_LATENCY_WFI)))
		goto constraint_set_lat_failed;

	if ((rc = clk_enable(data->iclk)))
		goto iclk_enable_failed;

	omap34xx_isp_mmu_enable(1);
	omap34xx_isp_configure(data);

	for (i = 0; OMAP34XX_ISP_INPUTS > i; ++i) {
		mst = &data->inputs[i].mst;
		omap34xx_isp_input_configure(&data->inputs[i]);

		if ((rc = omap34xx_v4l2_int_slave_ioctl_s_power_on(mst)))
			goto omap34xx_v4l2_int_slave_ioctl_s_power_on_failed;
	}

	for (i = 0; OMAP34XX_ISP_DEVICES > i; ++i)
		omap34xx_isp_v4l2_device_configure(&data->devs[i]);

	goto exit;

omap34xx_v4l2_int_slave_ioctl_s_power_on_failed:
	while (i--) {
		mst = &data->inputs[i].mst;
		(void)omap34xx_v4l2_int_slave_ioctl_s_power_off(mst);
	}

	omap34xx_isp_mmu_disable();
	clk_disable(data->iclk);
iclk_enable_failed:
	constraint_remove(data->lat_co);
constraint_set_lat_failed:
	constraint_remove(data->vdd2_co);
constraint_set_vdd2_failed:
	constraint_remove(data->vdd1_co);
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}

static void omap34xx_isp_suspend(struct omap34xx_isp_data *data)
{
	int i;
	struct omap34xx_v4l2_device *dev;
	struct omap34xx_v4l2_int_master *mst;

	SPEW(1, "+++ %s\n", __func__);

	for (i = 0; OMAP34XX_ISP_DEVICES > i; ++i) {
		dev = &data->devs[i].dev;
		(void)omap34xx_v4l2_device_vidioc_streamoff(dev);
	}

	for (i = 0; OMAP34XX_ISP_INPUTS > i; ++i) {
		mst = &data->inputs[i].mst;
		(void)omap34xx_v4l2_int_slave_ioctl_s_power_off(mst);
	}

	omap34xx_isp_mmu_disable();
	clk_disable(data->iclk);
	constraint_remove(data->lat_co);
	constraint_remove(data->vdd2_co);
	constraint_remove(data->vdd1_co);

	SPEW(1, "--- %s\n", __func__);
}

int omap34xx_isp_reset(struct platform_device *pdev, long to)
{
	int i;
	int rc;
	unsigned long expiry;
	struct omap34xx_isp_data *data = platform_get_drvdata(pdev);

	SPEW(1, "+++ %s\n", __func__);

	omap_setl(ISP_SYSCONFIG, SOFT_RESET_MASK);
	expiry = jiffies + to;

	while (!omap_testl(ISP_SYSSTATUS, RESET_DONE_MASK)) {
		if (time_after(jiffies, expiry)) {
			rc = -ETIMEDOUT;
			goto exit;
		}

		msleep(jiffies_to_msecs(1));
	}

	omap34xx_isp_mmu_enable(0);
	omap34xx_isp_configure(data);

	for (i = 0; OMAP34XX_ISP_INPUTS > i; ++i)
		omap34xx_isp_input_configure(&data->inputs[i]);

	for (i = 0; OMAP34XX_ISP_DEVICES > i; ++i)
		omap34xx_isp_v4l2_device_configure(&data->devs[i]);
exit:
	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_reset);

int
omap34xx_isp_file_open(struct omap34xx_v4l2_device *dev)
{
	int rc;
	struct omap34xx_isp_data *data = platform_get_drvdata(dev->parent);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);

	if (!data->files && (rc = omap34xx_isp_resume(data)))
		goto unlock;

	++data->files;
	rc = 0;
unlock:
	mutex_unlock(&data->lock);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
EXPORT_SYMBOL(omap34xx_isp_file_open);

int
omap34xx_isp_file_release(struct omap34xx_v4l2_device *dev)
{
	struct omap34xx_isp_data *data = platform_get_drvdata(dev->parent);

	SPEW(1, "+++ %s\n", __func__);

	mutex_lock(&data->lock);
	data->files--;

	if (!data->files)
		omap34xx_isp_suspend(data);

	mutex_unlock(&data->lock);

	SPEW(1, "--- %s\n", __func__);

	return (0);
}
EXPORT_SYMBOL(omap34xx_isp_file_release);

static irqreturn_t
omap34xx_isp_isr(int irq, void *priv)
{
	int i;
	irqreturn_t rc = IRQ_NONE;
	struct omap34xx_isp_data *data = priv;

	for (i = 0; OMAP34XX_ISP_INPUTS > i; ++i) {
		if (omap34xx_isp_input_isr(&data->inputs[i]))
			rc = IRQ_HANDLED;
	}

	for (i = 0; OMAP34XX_ISP_DEVICES > i; ++i) {
		if (omap34xx_isp_v4l2_device_isr(&data->devs[i]))
			rc = IRQ_HANDLED;
	}

	return (rc);
}

static int
omap34xx_isp_device_probe(struct platform_device *pdev)
{
	int rc;
	struct omap34xx_isp_data *data;
	struct omap34xx_isp_input *input;
	struct omap34xx_isp_v4l2_device *dev;
	struct omap34xx_isp_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		rc = -ENODEV;
		goto exit;
	}

	if (!(data = kzalloc(sizeof(*data), GFP_KERNEL))) {
		rc = -ENOMEM;
		goto exit;
	}

	mutex_init(&data->lock);
	data->isp_sysconfig_midle_mode = pdata->isp_sysconfig_midle_mode;
	data->isp_sysconfig_auto_idle = pdata->isp_sysconfig_auto_idle;
	data->isp_ctrl_sync_detect = pdata->isp_ctrl_sync_detect;
	data->isp_ctrl_cbuff_autogating = pdata->isp_ctrl_cbuff_autogating;
	data->isp_ctrl_par_ser_clk_sel = pdata->isp_ctrl_par_ser_clk_sel;
	data->tctrl_frame_strb = pdata->tctrl_frame_strb;
	data->tctrl_strb_delay = pdata->tctrl_strb_delay;
	data->tctrl_strb_length = pdata->tctrl_strb_length;
	data->tctrl_ctrl_insel = pdata->tctrl_ctrl_insel;
	data->tctrl_ctrl_divc = pdata->tctrl_ctrl_divc;
	data->ccdc_syn_mode_vdhden = pdata->ccdc_syn_mode_vdhden;
	data->ccdc_syn_mode_datsiz = pdata->ccdc_syn_mode_datsiz;
	data->ccdc_cfg_bswd = pdata->ccdc_cfg_bswd;
	data->ccdc_fmtcfg_vpin = pdata->ccdc_fmtcfg_vpin;

	if (IS_ERR(data->vdd1_co = constraint_get("cam_vdd1", &vdd1_co_id))) {
		rc = PTR_ERR(data->vdd1_co);
		goto constraint_get_vdd1_failed;
	}

	if (IS_ERR(data->vdd2_co = constraint_get("cam_vdd2", &vdd2_co_id))) {
		rc = PTR_ERR(data->vdd2_co);
		goto constraint_get_vdd2_failed;
	}

	if (IS_ERR(data->lat_co = constraint_get("cam_lat", &lat_co_id))) {
		rc = PTR_ERR(data->lat_co);
		goto constraint_get_lat_failed;
	}

	if (IS_ERR(data->iclk = clk_get(NULL, "cam_ick"))) {
		rc = PTR_ERR(data->iclk);
		goto iclk_get_failed;
	}

	if ((rc = omap34xx_dma_pool_create(&data->dma_pool, pdata->dma_pool)))
		goto omap34xx_dma_pool_create_failed;

	platform_set_drvdata(pdev, data);

	input = &data->inputs[OMAP34XX_ISP_SMIA10_INPUT];
	input->type = OMAP34XX_ISP_SMIA10_INPUT;
	input->mst.parent = pdev;

	if ((rc = omap34xx_isp_smia10_input_probe(input)))
		goto omap34xx_isp_smia10_input_probe_failed;

	dev = &data->devs[OMAP34XX_ISP_VIDEO_DEVICE];
	dev->input = input;
	dev->dev.pool = &data->dma_pool;
	dev->dev.parent = pdev;

	if ((rc = omap34xx_isp_video_device_probe(dev)))
		goto omap34xx_isp_video_device_probe_failed;

	dev = &data->devs[OMAP34XX_ISP_STATS_DEVICE];
	dev->input = input;
	dev->dev.pool = &data->dma_pool;
	dev->dev.parent = pdev;

	if ((rc = omap34xx_isp_stats_device_probe(dev)))
		goto omap34xx_isp_stats_device_probe_failed;

	if ((rc = request_irq(INT_34XX_CAM_IRQ, omap34xx_isp_isr, 0,
				OMAP34XX_ISP_DEVICE, data)))
		goto request_irq_failed;

	if ((rc = omap34xx_isp_dbg_register_device(&pdev->dev)))
		goto omap34xx_isp_dbg_register_device_failed;

	if ((rc = omap34xx_isp_create_spew_level(&pdev->dev)))
		goto omap34xx_isp_create_spew_level_failed;

	return (0);

omap34xx_isp_create_spew_level_failed:
	omap34xx_isp_dbg_unregister_device(&pdev->dev);
omap34xx_isp_dbg_register_device_failed:
	free_irq(INT_34XX_CAM_IRQ, data);
request_irq_failed:
	dev = &data->devs[OMAP34XX_ISP_STATS_DEVICE];
	omap34xx_isp_stats_device_remove(dev);
omap34xx_isp_stats_device_probe_failed:
	dev = &data->devs[OMAP34XX_ISP_VIDEO_DEVICE];
	omap34xx_isp_video_device_remove(dev);
omap34xx_isp_video_device_probe_failed:
	input = &data->inputs[OMAP34XX_ISP_SMIA10_INPUT];
	omap34xx_isp_smia10_input_remove(input);
omap34xx_isp_smia10_input_probe_failed:
	omap34xx_dma_pool_destroy(&data->dma_pool);
omap34xx_dma_pool_create_failed:
	clk_put(data->iclk);
iclk_get_failed:
	constraint_put(data->lat_co);
constraint_get_lat_failed:
	constraint_put(data->vdd2_co);
constraint_get_vdd2_failed:
	constraint_put(data->vdd1_co);
constraint_get_vdd1_failed:
	kfree(data);
exit:
	printk(KERN_ERR OMAP34XX_ISP_DRIVER ": probe failed, rc=%d\n", rc);

	return (rc);
}

static int
omap34xx_isp_device_remove(struct platform_device *pdev)
{
	struct omap34xx_isp_data *data = platform_get_drvdata(pdev);
	struct omap34xx_isp_input *input;
	struct omap34xx_isp_v4l2_device *dev;

	SPEW(1, "+++ %s\n", __func__);

	omap34xx_isp_remove_spew_level(&pdev->dev);
	omap34xx_isp_dbg_unregister_device(&pdev->dev);
	/* TODO: is the device suspended? */
	free_irq(INT_34XX_CAM_IRQ, data);

	dev = &data->devs[OMAP34XX_ISP_STATS_DEVICE];
	omap34xx_isp_stats_device_remove(dev);

	dev = &data->devs[OMAP34XX_ISP_VIDEO_DEVICE];
	omap34xx_isp_video_device_remove(dev);

	input = &data->inputs[OMAP34XX_ISP_SMIA10_INPUT];
	omap34xx_isp_smia10_input_remove(input);
	omap34xx_dma_pool_destroy(&data->dma_pool);
	clk_put(data->iclk);
	constraint_put(data->lat_co);
	constraint_put(data->vdd2_co);
	constraint_put(data->vdd1_co);
	kfree(data);

	SPEW(1, "--- %s\n", __func__);

	return (0);
}

#ifdef CONFIG_PM
static int
omap34xx_isp_device_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct omap34xx_isp_data *data = platform_get_drvdata(pdev);

	SPEW(1, "+++ %s\n", __func__);

	if ((PM_EVENT_SUSPEND == state.event) && data->files)
		omap34xx_isp_suspend(data);

	SPEW(1, "--- %s\n", __func__);

	return (0);
}

static int
omap34xx_isp_device_resume(struct platform_device *pdev)
{
	int rc = 0;
	struct omap34xx_isp_data *data = platform_get_drvdata(pdev);

	SPEW(1, "+++ %s\n", __func__);

	if (data->files)
		rc = omap34xx_isp_resume(data);

	SPEW(1, "--- %s: rc=%d\n", __func__, rc);

	return (rc);
}
#endif // CONFIG_PM

static struct platform_driver omap34xx_isp_driver = {
	.driver = {
		.name = OMAP34XX_ISP_DRIVER,
	},
	.probe = omap34xx_isp_device_probe,
	.remove = __devexit_p(omap34xx_isp_device_remove),
#ifdef CONFIG_PM
	.suspend = omap34xx_isp_device_suspend,
	.resume = omap34xx_isp_device_resume,
#endif // CONFIG_PM
};

static int __init
omap34xx_isp_module_init(void)
{
	return (platform_driver_register(&omap34xx_isp_driver));
}

static void __exit
omap34xx_isp_module_exit(void)
{
	platform_driver_unregister(&omap34xx_isp_driver);
}

module_init(omap34xx_isp_module_init);
module_exit(omap34xx_isp_module_exit);

MODULE_DESCRIPTION("OMAP34XX ISP driver");
MODULE_LICENSE("GPL");
