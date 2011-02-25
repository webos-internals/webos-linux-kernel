#ifndef _ASM_ARCH_ISP_H
#define _ASM_ARCH_ISP_H

#include <linux/videodev2.h>

#define V4L2_BUF_TYPE_STATS_CAPTURE	V4L2_BUF_TYPE_PRIVATE
#define V4L2_CAP_STATS_CAPTURE		0x00000400

#define V4L2_CID_OMAP34XX_ISP_BASE	(V4L2_CID_PRIVATE_BASE | 0x3A0000)
#define V4L2_CID_OMAP34XX_ISP(index)	(V4L2_CID_OMAP34XX_ISP_BASE + (index))

#define V4L2_CID_OMAP34XX_ISP_CCDC_CLAMP_CFG	V4L2_CID_OMAP34XX_ISP(0)
#define V4L2_CID_OMAP34XX_ISP_CCDC_BLKCOMP_CFG	V4L2_CID_OMAP34XX_ISP(1)
#define V4L2_CID_OMAP34XX_ISP_PRV_DCOR_CFG	V4L2_CID_OMAP34XX_ISP(2)
#define V4L2_CID_OMAP34XX_ISP_PRV_NF_CFG	V4L2_CID_OMAP34XX_ISP(3)
#define V4L2_CID_OMAP34XX_ISP_PRV_WB_CFG	V4L2_CID_OMAP34XX_ISP(4)
#define V4L2_CID_OMAP34XX_ISP_PRV_CFA_CFG	V4L2_CID_OMAP34XX_ISP(5)
#define V4L2_CID_OMAP34XX_ISP_PRV_BLKADJ_CFG	V4L2_CID_OMAP34XX_ISP(6)
#define V4L2_CID_OMAP34XX_ISP_PRV_RGB_CFG	V4L2_CID_OMAP34XX_ISP(7)
#define V4L2_CID_OMAP34XX_ISP_PRV_GAMMA_CFG	V4L2_CID_OMAP34XX_ISP(8)
#define V4L2_CID_OMAP34XX_ISP_PRV_YUV_CFG	V4L2_CID_OMAP34XX_ISP(9)
#define V4L2_CID_OMAP34XX_ISP_PRV_OUT_HORZ	V4L2_CID_OMAP34XX_ISP(10)
#define V4L2_CID_OMAP34XX_ISP_PRV_OUT_VERT	V4L2_CID_OMAP34XX_ISP(11)
#define V4L2_CID_OMAP34XX_ISP_SMIA10_USE_DPCM	V4L2_CID_OMAP34XX_ISP(12)

#define OMAP34XX_ISP_PRV_PCR_CFAFMT_BAYER	0x0
#define OMAP34XX_ISP_PRV_GAMMA_TBL_LEN		1024
#define OMAP34XX_ISP_PRV_NF_TBL_LEN		64
#define OMAP34XX_ISP_PRV_CFA_TBL_LEN		576

#define UwQx(i, w, f, x)	(((1 << (w)) - 1) & (((i) << (x)) | (f)))
#define U5Q4(i, f)		(__u8)UwQx(i, 5, f, 4)
#define U8Q4(i, f)		(__u8)UwQx(i, 8, f, 4)
#define U8Q5(i, f)		(__u8)UwQx(i, 8, f, 5)
#define S8Q6(i, f)		(__s8)UwQx(i, 8, f, 6)
#define U10Q8(i, f)		(__u16)UwQx(i, 10, f, 8)
#define S10Q8(i, f)		(__s16)UwQx(i, 10, f, 8)
#define S12Q8(i, f)		(__s16)UwQx(i, 12, f, 8)

struct omap34xx_isp_ccdc_clamp_cfg {
	__u8	clamp_clampen;
	__u8	clamp_obslen;
	__u8	clamp_obsln;
	__u16	clamp_obst;
	__u8	clamp_obgain;	/* U5Q4 */
	__u16	dcsub;
};

struct omap34xx_isp_ccdc_blkcomp_cfg {
	__s8	blkcmp_r_ye;
	__s8	blkcmp_gr_cy;
	__s8	blkcmp_gb_g;
	__s8	blkcmp_b_mg;
};

struct omap34xx_isp_prv_dcor_cfg {
	__u8	pcr_dcor_method;
	__u8	pcr_dcoren;
	__u16	cdc_thr0_correct;
	__u16	cdc_thr0_detect;
	__u16	cdc_thr1_correct;
	__u16	cdc_thr1_detect;
	__u16	cdc_thr2_correct;
	__u16	cdc_thr2_detect;
	__u16	cdc_thr3_correct;
	__u16	cdc_thr3_detect;
};

struct omap34xx_isp_prv_nf_cfg {
	__u8	pcr_nfen;
	__u8	nf_spr;
	__u32	tbl[OMAP34XX_ISP_PRV_NF_TBL_LEN];
};

struct omap34xx_isp_prv_wb_cfg {
	__u16	wb_dgain;	/* U10Q8 */
	__u8	wbgain_coef3;	/* U8Q5 */
	__u8	wbgain_coef2;	/* U8Q5 */
	__u8	wbgain_coef1;	/* U8Q5 */
	__u8	wbgain_coef0;	/* U8Q5 */
};

struct omap34xx_isp_prv_cfa_cfg {
	__u8	pcr_cfafmt;
	__u8	pcr_cfaen;
	__u8	cfa_gradth_ver;
	__u8	cfa_gradth_hor;
	__u32	tbl[OMAP34XX_ISP_PRV_CFA_TBL_LEN];
};

struct omap34xx_isp_prv_blkadj_cfg {
	__s8	blkadjoff_r;
	__s8	blkadjoff_g;
	__s8	blkadjoff_b;
};

struct omap34xx_isp_prv_rgb_cfg {
	__s16	rgb_mat1_mtx_gr;	/* S12Q8 */
	__s16	rgb_mat1_mtx_rr;	/* S12Q8 */
	__s16	rgb_mat2_mtx_rg;	/* S12Q8 */
	__s16	rgb_mat2_mtx_br;	/* S12Q8 */
	__s16	rgb_mat3_mtx_bg;	/* S12Q8 */
	__s16	rgb_mat3_mtx_gg;	/* S12Q8 */
	__s16	rgb_mat4_mtx_gb;	/* S12Q8 */
	__s16	rgb_mat4_mtx_rb;	/* S12Q8 */
	__s16	rgb_mat5_mtx_bb;	/* S12Q8 */
	__s16	rgb_off1_mtx_offr;
	__s16	rgb_off1_mtx_offg;
	__s16	rgb_off2_mtx_offb;
};

struct omap34xx_isp_prv_gamma_cfg {
	__u8	pcr_gamma_bypass;
	__u32	red_tbl[OMAP34XX_ISP_PRV_GAMMA_TBL_LEN];
	__u32	green_tbl[OMAP34XX_ISP_PRV_GAMMA_TBL_LEN];
	__u32	blue_tbl[OMAP34XX_ISP_PRV_GAMMA_TBL_LEN];
};

struct omap34xx_isp_prv_yuv_cfg {
	__s16	csc0_cscby;		/* S10Q8 */
	__s16	csc0_cscgy;		/* S10Q8 */
	__s16	csc0_cscry;		/* S10Q8 */
	__s16	csc1_cscbcb;		/* S10Q8 */
	__s16	csc1_cscgcb;		/* S10Q8 */
	__s16	csc1_cscrcb;		/* S10Q8 */
	__s16	csc2_cscbcr;		/* S10Q8 */
	__s16	csc2_cscgcr;		/* S10Q8 */
	__s16	csc2_cscrcr;		/* S10Q8 */
	__s8	csc_offset_yofst;	/* S8Q0 */
	__s8	csc_offset_ofstcb;	/* S8Q0 */
	__s8	csc_offset_ofstcr;	/* S8Q0 */
};

struct omap34xx_isp_smia10_format {
	__u32	width;
	__u32	sof_offset;
	__u32	sof_lines;
	__u32	sof_size;
};

struct omap34xx_isp_h3a_aeawb_format {
	__u16	pcr_ave2lmt;
	__u8	aewin1_winh;
	__u8	aewin1_winw;
	__u8	aewin1_winvc;
	__u8	aewin1_winhc;
	__u16	aewinstart_winsv;
	__u16	aewinstart_winsh;
	__u16	aewinblack_winsv;
	__u8	aewinblack_winh;
	__u8	aewsubwin_aewincv;
	__u8	aewsubwin_aewinch;
	__u32	offset;
	__u32	size;
};

struct omap34xx_isp_stats_format {
	struct omap34xx_isp_smia10_format	smia10;
	struct omap34xx_isp_h3a_aeawb_format	h3a_aeawb;
	__u32					size;
};

#ifdef __KERNEL__
#include <asm/types.h>

#define OMAP34XX_ISP_DEVICE	"omap34xx-isp"
#define OMAP34XX_ISP_DRIVER	"omap34xx-isp"

#define OMAP34XX_CCP2_RESOURCE_MEM	0

#define OMAP34XX_ISP_SYSCONFIG_MIDLE_MODE_SMART_STANDBY	0x2
#define OMAP34XX_ISP_CTRL_SYNC_DETECT_VS_FALLING	0x2
#define OMAP34XX_ISP_CTRL_SYNC_DETECT_VS_RISING		0x3
#define OMAP34XX_ISP_CTRL_PAR_SER_CLK_SEL_CSIB		0x2
#define OMAP34XX_ISP_CCDC_FMTCFG_VPIN_BITS_9_0		0x6
#define OMAP34XX_ISP_CCDC_SYN_MODE_DATSIZ_10_BITS	0x6
#define OMAP34XX_ISP_PRV_AVE_DIST_2			0x1
#define OMAP34XX_ISP_TCTRL_CTRL_INSEL_VP		0x0
#define OMAP34XX_ISP_TCTRL_CTRL_INSEL_CSIB		0x2
#define OMAP34XX_MMU_SYSCONFIG_IDLEMODE_SMART_IDLE	0x2

#define OMAP34XX_CCP2B_BASE					0x480BC400
#define OMAP34XX_CCP2_SYSCONFIG_MSTANDBY_MODE_SMART_STANDBY	0x2
#define OMAP34XX_CCP2_CTRL_IO_OUT_SEL_PARALLEL			0x1
#define OMAP34XX_CCP2_CTRL_PHY_SEL_DATA_STROBE			0x1
#define	OMAP34XX_CCP2_LCx_CTRL_FORMAT_RAW8_DPCM10_VP		0x12
#define	OMAP34XX_CCP2_LCx_CTRL_FORMAT_RAW10_VP			0x16

#define OMAP34XX_CONTROL_CSIRXFE_CSIB_SELFORM_DATA_STROBE	0x1

struct omap34xx_isp_phy_lane {
	u8 pol;
	u8 pos;
};

struct omap34xx_isp_platform_data {
	u8	isp_sysconfig_midle_mode;
	u8	isp_sysconfig_auto_idle;
	
	u8	isp_ctrl_sync_detect;
	u8	isp_ctrl_cbuff_autogating;
	u8	isp_ctrl_par_ser_clk_sel;
	
	u8	tctrl_ctrl_insel;
	u16	tctrl_ctrl_divc;
	/* TODO: calculate these */
	u8	tctrl_frame_strb;
	u32	tctrl_strb_delay;
	u32	tctrl_strb_length;
	
	u8	ccdc_syn_mode_vdhden;
	u8	ccdc_syn_mode_datsiz;
	u8	ccdc_cfg_bswd;
	u8	ccdc_fmtcfg_vpin;
	
	u8	prv_pcr_gamma_bypass;
	u8	prv_pcr_cfaen;
	u16	prv_wb_dgain;		/* U10Q8 */
	u8	prv_wbgain_coef3;	/* U8Q5 */
	u8	prv_wbgain_coef2;	/* U8Q5 */
	u8	prv_wbgain_coef1;	/* U8Q5 */
	u8	prv_wbgain_coef0;	/* U8Q5 */
	s16	prv_rgb_mat1_mtx_rr;	/* S12Q8 */
	s16	prv_rgb_mat3_mtx_gg;	/* S12Q8 */
	s16	prv_rgb_mat5_mtx_bb;	/* S12Q8 */
	s16	prv_csc0_cscby;		/* S10Q8 */
	s16	prv_csc0_cscgy;		/* S10Q8 */
	s16	prv_csc0_cscry;		/* S10Q8 */
	s16	prv_csc1_cscbcb;	/* S10Q8 */
	s16	prv_csc1_cscgcb;	/* S10Q8 */
	s16	prv_csc1_cscrcb;	/* S10Q8 */
	s16	prv_csc2_cscbcr;	/* S10Q8 */
	s16	prv_csc2_cscgcr;	/* S10Q8 */
	s16	prv_csc2_cscrcr;	/* S10Q8 */
	
	u8	rsz_cnt_vstph;
	u8	rsz_cnt_hstph;
	
	u8	control_csirxfe_csib_selform;
	u8	ccp2_sysconfig_mstandby_mode;
	/* TODO: revisit */
	u8	ccp2_ctrl_io_out_sel;
	u8	ccp2_ctrl_phy_sel;
	u32	ccp2_ctrl_fracdiv;
	u8	ccp2_lc0_ctrl_format;
	u8	ccp2_lc0_ctrl_crc_en;
	u16	ccp2_lc0_stat_size_sof;
	u16	ccp2_lc0_dat_start_vert;
	u8	ccp2_lc0_ctrl_dpcm_pred;

	struct omap34xx_isp_phy_lane phy_clock_lane;
	struct omap34xx_isp_phy_lane phy_data_lanes[2];

	size_t	smia10_sof_len;
	
	u32	video_width;
	u32	video_height;
	u32	video_fourcc;

	size_t	dma_pool;
};

#define ISP_BASE_5MP_IN_WIDTH		2608
#define ISP_BASE_5MP_IN_HEIGHT		1960
#define ISP_BASE_5MP_OUT_WIDTH		2592
#define ISP_BASE_5MP_OUT_HEIGHT		1944

#define ISP_BASE_5MPHALF_IN_WIDTH	1296 /* was: ISP_BASE_5MP_IN_WIDTH / 2 */
#define ISP_BASE_5MPHALF_IN_HEIGHT	(ISP_BASE_5MP_IN_HEIGHT / 2)

#endif // __KERNEL__
#endif // _ASM_ARCH_ISP_H
