#ifndef __TWL4030_AUDIO_HW_H
#define __TWL4030_AUDIO_HW_H

/****************************************
 *  AUDIO_VOICE                           
 ****************************************/

/**** Register Definitions REG_BASE=0x0 DevAdd=0x49 */
/*** CONVENTION:
 * REG_xxx - Register offset
 * BIT_xxx - Bit field bit location
 * BIT_xxx_M - Mask for that Field.
 * Valid values are posted next to the bit definition
 */

#define REG_CODEC_MODE                           (0x1)
#define REG_OPTION                               (0x2)
#define REG_MICBIAS_CTL                          (0x4)
#define REG_ANAMICL                              (0x5)
#define REG_ANAMICR                              (0x6)
#define REG_AVADC_CTL                            (0x7)
#define REG_ADCMICSEL                            (0x8)
#define REG_DIGMIXING                            (0x9)
#define REG_ATXL1PGA                             (0xA)
#define REG_ATXR1PGA                             (0xB)
#define REG_AVTXL2PGA                            (0xC)
#define REG_AVTXR2PGA                            (0xD)
#define REG_AUDIO_IF                             (0xE)
#define REG_VOICE_IF                             (0xF)
#define REG_ARXR1PGA                             (0x10)
#define REG_ARXL1PGA                             (0x11)
#define REG_ARXR2PGA                             (0x12)
#define REG_ARXL2PGA                             (0x13)
#define REG_VRXPGA                               (0x14)
#define REG_VSTPGA                               (0x15)
#define REG_VRX2ARXPGA                           (0x16)
#define REG_AVDAC_CTL                            (0x17)
#define REG_ARX2VTXPGA                           (0x18)
#define REG_ARXL1_APGA_CTL                       (0x19)
#define REG_ARXR1_APGA_CTL                       (0x1A)
#define REG_ARXL2_APGA_CTL                       (0x1B)
#define REG_ARXR2_APGA_CTL                       (0x1C)
#define REG_ATX2ARXPGA                           (0x1D)
#define REG_BT_IF                                (0x1E)
#define REG_BTPGA                                (0x1F)
#define REG_BTSTPGA                              (0x20)
#define REG_EAR_CTL                              (0x21)
#define REG_HS_SEL                               (0x22)
#define REG_HS_GAIN_SET                          (0x23)
#define REG_HS_POPN_SET                          (0x24)
#define REG_PREDL_CTL                            (0x25)
#define REG_PREDR_CTL                            (0x26)
#define REG_PRECKL_CTL                           (0x27)
#define REG_PRECKR_CTL                           (0x28)
#define REG_HFL_CTL                              (0x29)
#define REG_HFR_CTL                              (0x2A)
#define REG_ALC_CTL                              (0x2B)
#define REG_ALC_SET1                             (0x2C)
#define REG_ALC_SET2                             (0x2D)
#define REG_BOOST_CTL                            (0x2E)
#define REG_SOFTVOL_CTL                          (0x2F)
#define REG_DTMF_FREQSEL                         (0x30)
#define REG_DTMF_TONEXT1H                        (0x31)
#define REG_DTMF_TONEXT1L                        (0x32)
#define REG_DTMF_TONEXT2H                        (0x33)
#define REG_DTMF_TONEXT2L                        (0x34)
#define REG_DTMF_TONOFF                          (0x35)
#define REG_DTMF_WANONOFF                        (0x36)
#define REG_I2S_RX_SCRAMBLE_H                    (0x37)
#define REG_I2S_RX_SCRAMBLE_M                    (0x38)
#define REG_I2S_RX_SCRAMBLE_L                    (0x39)
#define REG_APLL_CTL                             (0x3A)
#define REG_DTMF_CTL                             (0x3B)
#define REG_DTMF_PGA_CTL2                        (0x3C)
#define REG_DTMF_PGA_CTL1                        (0x3D)
#define REG_MISC_SET_1                           (0x3E)
#define REG_PCMBTMUX                             (0x3F)
#define REG_RX_PATH_SEL                          (0x43)
#define REG_VDL_APGA_CTL                         (0x44)
#define REG_VIBRA_CTL                            (0x45)
#define REG_VIBRA_SET                            (0x46)
#define REG_VIBRA_PWM_SET                        (0x47)
#define REG_ANAMIC_GAIN                          (0x48)
#define REG_MISC_SET_2                           (0x49)

#define MAX_NUM_REG_CLEAN                        (REG_MISC_SET_2 - REG_CODEC_MODE)

/**** BitField Definitions */

/* CODEC_MODE Fields */
#define BIT_CODEC_MODE_OPT_MODE                  (0x000)
#define BIT_CODEC_MODE_OPT_MODE_M                (0x00000001)

#define CODEC_OPTION_1                           (0x1)
#define CODEC_OPTION_2                           (0x0)

#define BIT_CODEC_MODE_CODECPDZ                  (0x001)
#define BIT_CODEC_MODE_CODECPDZ_M                (0x00000002)
#define BIT_CODEC_MODE_SPARE                     (0x002)
#define BIT_CODEC_MODE_SPARE_M                   (0x00000004)
#define BIT_CODEC_MODE_SEL_16K                   (0x003)
#define BIT_CODEC_MODE_SEL_16K_M                 (0x00000008)

#define VOICE_MODE_RATE_08_000K                  (0x0)
#define VOICE_MODE_RATE_16_000K                  (0x1)

#define BIT_CODEC_MODE_APLL_RATE                 (0x004)
#define BIT_CODEC_MODE_APLL_RATE_M               (0x000000F0)

#define AUDIO_MODE_RATE_08_000                   (0x0)
#define AUDIO_MODE_RATE_11_025                   (0x1)
#define AUDIO_MODE_RATE_12_000                   (0x2)
#define AUDIO_MODE_RATE_16_000                   (0x4)
#define AUDIO_MODE_RATE_22_050                   (0x5)
#define AUDIO_MODE_RATE_24_000                   (0x6)
#define AUDIO_MODE_RATE_32_000                   (0x8)
#define AUDIO_MODE_RATE_44_100                   (0x9)
#define AUDIO_MODE_RATE_48_000                   (0xA)
#define AUDIO_MODE_RATE_96_000                   (0xE)

/* OPTION Fields */
#define BIT_OPTION_ATXL1_EN                      (0x000)
#define BIT_OPTION_ATXL1_EN_M                    (0x00000001)
#define BIT_OPTION_ATXR1_EN                      (0x001)
#define BIT_OPTION_ATXR1_EN_M                    (0x00000002)
#define BIT_OPTION_ATXL2_VTXL_EN                 (0x002)
#define BIT_OPTION_ATXL2_VTXL_EN_M               (0x00000004)
#define BIT_OPTION_ATXR2_VTXR_EN                 (0x003)
#define BIT_OPTION_ATXR2_VTXR_EN_M               (0x00000008)
#define BIT_OPTION_ARXL1_VRX_EN                  (0x004)
#define BIT_OPTION_ARXL1_VRX_EN_M                (0x00000010)
#define BIT_OPTION_ARXR1_EN                      (0x005)
#define BIT_OPTION_ARXR1_EN_M                    (0x00000020)
#define BIT_OPTION_ARXL2_EN                      (0x006)
#define BIT_OPTION_ARXL2_EN_M                    (0x00000040)
#define BIT_OPTION_ARXR2_EN                      (0x007)
#define BIT_OPTION_ARXR2_EN_M                    (0x00000080)
/* MICBIAS_CTL Fields */
#define BIT_MICBIAS_CTL_MICBIAS1_EN              (0x000)
#define BIT_MICBIAS_CTL_MICBIAS1_EN_M            (0x00000001)
#define BIT_MICBIAS_CTL_MICBIAS2_EN              (0x001)
#define BIT_MICBIAS_CTL_MICBIAS2_EN_M            (0x00000002)
#define BIT_MICBIAS_CTL_HSMICBIAS_EN             (0x002)
#define BIT_MICBIAS_CTL_HSMICBIAS_EN_M           (0x00000004)
#define BIT_MICBIAS_CTL_MICBIAS1_CTL             (0x005)
#define BIT_MICBIAS_CTL_MICBIAS1_CTL_M           (0x00000020)
#define BIT_MICBIAS_CTL_MICBIAS2_CTL             (0x006)
#define BIT_MICBIAS_CTL_MICBIAS2_CTL_M           (0x00000040)
#define BIT_MICBIAS_CTL_SPARE                    (0x007)
#define BIT_MICBIAS_CTL_SPARE_M                  (0x00000080)
/* ANAMICL Fields */
#define BIT_ANAMICL_MAINMIC_EN                   (0x000)
#define BIT_ANAMICL_MAINMIC_EN_M                 (0x00000001)
#define BIT_ANAMICL_HSMIC_EN                     (0x001)
#define BIT_ANAMICL_HSMIC_EN_M                   (0x00000002)
#define BIT_ANAMICL_AUXL_EN                      (0x002)
#define BIT_ANAMICL_AUXL_EN_M                    (0x00000004)
#define BIT_ANAMICL_CKMIC_EN                     (0x003)
#define BIT_ANAMICL_CKMIC_EN_M                   (0x00000008)
#define BIT_ANAMICL_MICAMPL_EN                   (0x004)
#define BIT_ANAMICL_MICAMPL_EN_M                 (0x00000010)
#define BIT_ANAMICL_OFFSET_CNCL_SEL              (0x005)
#define BIT_ANAMICL_OFFSET_CNCL_SEL_M            (0x00000060)
#define BIT_ANAMICL_CNCL_OFFSET_START            (0x007)
#define BIT_ANAMICL_CNCL_OFFSET_START_M          (0x00000080)
/* ANAMICR Fields */
#define BIT_ANAMICR_SUBMIC_EN                    (0x000)
#define BIT_ANAMICR_SUBMIC_EN_M                  (0x00000001)
#define BIT_ANAMICR_AUXR_EN                      (0x002)
#define BIT_ANAMICR_AUXR_EN_M                    (0x00000004)
#define BIT_ANAMICR_MICAMPR_EN                   (0x004)
#define BIT_ANAMICR_MICAMPR_EN_M                 (0x00000010)
/* AVADC_CTL Fields */
#define BIT_AVADC_CTL_ADCR_EN                    (0x001)
#define BIT_AVADC_CTL_ADCR_EN_M                  (0x00000002)
#define BIT_AVADC_CTL_AVADC_CLK_PRIORITY         (0x002)
#define BIT_AVADC_CTL_AVADC_CLK_PRIORITY_M       (0x00000004)
#define BIT_AVADC_CTL_ADCL_EN                    (0x003)
#define BIT_AVADC_CTL_ADCL_EN_M                  (0x00000008)
/* ADCMICSEL Fields */
#define BIT_ADCMICSEL_TX1IN_SEL                  (0x000)
#define BIT_ADCMICSEL_TX1IN_SEL_M                (0x00000001)
#define BIT_ADCMICSEL_DIGMIC0_EN                 (0x001)
#define BIT_ADCMICSEL_DIGMIC0_EN_M               (0x00000002)
#define BIT_ADCMICSEL_TX2IN_SEL                  (0x002)
#define BIT_ADCMICSEL_TX2IN_SEL_M                (0x00000004)
#define BIT_ADCMICSEL_DIGMIC1_EN                 (0x003)
#define BIT_ADCMICSEL_DIGMIC1_EN_M               (0x00000008)
/* DIGMIXING Fields */
#define BIT_DIGMIXING_VTX_MIXING                 (0x002)
#define BIT_DIGMIXING_VTX_MIXING_M               (0x0000000C)
#define BIT_DIGMIXING_ARX2_MIXING                (0x004)
#define BIT_DIGMIXING_ARX2_MIXING_M              (0x00000030)
#define BIT_DIGMIXING_ARX1_MIXING                (0x006)
#define BIT_DIGMIXING_ARX1_MIXING_M              (0x000000C0)
#define INPUT_GAIN_MIN                           (0x00)
#define INPUT_GAIN_MAX                           (0x1F)
/* ATXL1PGA Fields */
#define BIT_ATXL1PGA_ATXL1PGA_GAIN               (0x000)
#define BIT_ATXL1PGA_ATXL1PGA_GAIN_M             (0x0000001F)
/* ATXR1PGA Fields */
#define BIT_ATXR1PGA_ATXR1PGA_GAIN               (0x000)
#define BIT_ATXR1PGA_ATXR1PGA_GAIN_M             (0x0000001F)
/* AVTXL2PGA Fields */
#define BIT_AVTXL2PGA_AVTXL2PGA_GAIN             (0x000)
#define BIT_AVTXL2PGA_AVTXL2PGA_GAIN_M           (0x0000001F)
/* AVTXR2PGA Fields */
#define BIT_AVTXR2PGA_AVTXR2PGA_GAIN             (0x000)
#define BIT_AVTXR2PGA_AVTXR2PGA_GAIN_M           (0x0000001F)
/* AUDIO_IF Fields */
#define BIT_AUDIO_IF_AIF_EN                      (0x000)
#define BIT_AUDIO_IF_AIF_EN_M                    (0x00000001)
#define BIT_AUDIO_IF_CLK256FS_EN                 (0x001)
#define BIT_AUDIO_IF_CLK256FS_EN_M               (0x00000002)
#define BIT_AUDIO_IF_AIF_TRI_EN                  (0x002)
#define BIT_AUDIO_IF_AIF_TRI_EN_M                (0x00000004)

#define AUDIO_DATA_FORMAT_I2S                    (0x0)
#define AUDIO_DATA_FORMAT_LJUST                  (0x1)
#define AUDIO_DATA_FORMAT_RJUST                  (0x2)
#define AUDIO_DATA_FORMAT_TDM                    (0x3)

#define BIT_AUDIO_IF_AIF_FORMAT                  (0x003)
#define BIT_AUDIO_IF_AIF_FORMAT_M                (0x00000018)

#define AUDIO_DATA_WIDTH_16SAMPLE_16DATA         (0x0)
#define AUDIO_DATA_WIDTH_32SAMPLE_16DATA         (0x2)
#define AUDIO_DATA_WIDTH_32SAMPLE_24DATA         (0x3)

#define BIT_AUDIO_IF_DATA_WIDTH                  (0x005)
#define BIT_AUDIO_IF_DATA_WIDTH_M                (0x00000060)
#define BIT_AUDIO_IF_AIF_SLAVE_EN                (0x007)
#define BIT_AUDIO_IF_AIF_SLAVE_EN_M              (0x00000080)
/* VOICE_IF Fields */
#define BIT_VOICE_IF_VIF_EN                      (0x000)
#define BIT_VOICE_IF_VIF_EN_M                    (0x00000001)
#define BIT_VOICE_IF_VIF_SUB_EN                  (0x001)
#define BIT_VOICE_IF_VIF_SUB_EN_M                (0x00000002)
#define BIT_VOICE_IF_VIF_TRI_EN                  (0x002)
#define BIT_VOICE_IF_VIF_TRI_EN_M                (0x00000004)
#define BIT_VOICE_IF_VIF_FORMAT                  (0x003)
#define BIT_VOICE_IF_VIF_FORMAT_M                (0x00000008)
#define BIT_VOICE_IF_VIF_SWAP                    (0x004)
#define BIT_VOICE_IF_VIF_SWAP_M                  (0x00000010)
#define BIT_VOICE_IF_VIF_DOUT_EN                 (0x005)
#define BIT_VOICE_IF_VIF_DOUT_EN_M               (0x00000020)
#define BIT_VOICE_IF_VIF_DIN_EN                  (0x006)
#define BIT_VOICE_IF_VIF_DIN_EN_M                (0x00000040)
#define BIT_VOICE_IF_VIF_SLAVE_EN                (0x007)
#define BIT_VOICE_IF_VIF_SLAVE_EN_M              (0x00000080)
/* volume range */
#define OUTPUT_GAIN_MIN                          (0x00)
#define OUTPUT_GAIN_MAX                          (0x3F)
#define AUDIO_OUTPUT_COARSE_GAIN_LOW             (0x0)
#define AUDIO_OUTPUT_COARSE_GAIN_6DB             (0x1)
#define AUDIO_OUTPUT_COARSE_GAIN_12DB            (0x2)

/* ARXR1PGA Fields */
#define BIT_ARXR1PGA_ARXR1PGA_FGAIN              (0x000)
#define BIT_ARXR1PGA_ARXR1PGA_FGAIN_M            (0x0000003F)
#define BIT_ARXR1PGA_ARXR1PGA_CGAIN              (0x006)
#define BIT_ARXR1PGA_ARXR1PGA_CGAIN_M            (0x000000C0)
/* ARXL1PGA Fields */
#define BIT_ARXL1PGA_ARXL1PGA_FGAIN              (0x000)
#define BIT_ARXL1PGA_ARXL1PGA_FGAIN_M            (0x0000003F)
#define BIT_ARXL1PGA_ARXL1PGA_CGAIN              (0x006)
#define BIT_ARXL1PGA_ARXL1PGA_CGAIN_M            (0x000000C0)
/* ARXR2PGA Fields */
#define BIT_ARXR2PGA_ARXR2PGA_FGAIN              (0x000)
#define BIT_ARXR2PGA_ARXR2PGA_FGAIN_M            (0x0000003F)
#define BIT_ARXR2PGA_ARXR2PGA_CGAIN              (0x006)
#define BIT_ARXR2PGA_ARXR2PGA_CGAIN_M            (0x000000C0)
/* ARXL2PGA Fields */
#define BIT_ARXL2PGA_ARXL2PGA_FGAIN              (0x000)
#define BIT_ARXL2PGA_ARXL2PGA_FGAIN_M            (0x0000003F)
#define BIT_ARXL2PGA_ARXL2PGA_CGAIN              (0x006)
#define BIT_ARXL2PGA_ARXL2PGA_CGAIN_M            (0x000000C0)
/* VRXPGA Fields */
#define BIT_VRXPGA_VRXPGA_GAIN                   (0x000)
#define BIT_VRXPGA_VRXPGA_GAIN_M                 (0x0000003F)
/* VSTPGA Fields */
#define BIT_VSTPGA_VSTPGA_GAIN                   (0x000)
#define BIT_VSTPGA_VSTPGA_GAIN_M                 (0x0000003F)
#define SIDETONE_MAX_GAIN                        (0x29)
/* VRX2ARXPGA Fields */
#define BIT_VRX2ARXPGA_VRX2ARXPGA_GAIN           (0x000)
#define BIT_VRX2ARXPGA_VRX2ARXPGA_GAIN_M         (0x0000001F)
/* AVDAC_CTL Fields */
#define BIT_AVDAC_CTL_ADACR1_EN                  (0x000)
#define BIT_AVDAC_CTL_ADACR1_EN_M                (0x00000001)
#define BIT_AVDAC_CTL_ADACL1_EN                  (0x001)
#define BIT_AVDAC_CTL_ADACL1_EN_M                (0x00000002)
#define BIT_AVDAC_CTL_ADACR2_EN                  (0x002)
#define BIT_AVDAC_CTL_ADACR2_EN_M                (0x00000004)
#define BIT_AVDAC_CTL_ADACL2_EN                  (0x003)
#define BIT_AVDAC_CTL_ADACL2_EN_M                (0x00000008)
#define BIT_AVDAC_CTL_VDAC_EN                    (0x004)
#define BIT_AVDAC_CTL_VDAC_EN_M                  (0x00000010)
/* ARX2VTXPGA Fields */
#define BIT_ARX2VTXPGA_ARX2VTXPGA_GAIN           (0x000)
#define BIT_ARX2VTXPGA_ARX2VTXPGA_GAIN_M         (0x0000003F)

#define ARX_APGA_MIN                             (0x12)
#define ARX_APGA_MAX                             (0x00)

/* ARXL1_APGA_CTL Fields */
#define BIT_ARXL1_APGA_CTL_ARXL1_PDZ             (0x000)
#define BIT_ARXL1_APGA_CTL_ARXL1_PDZ_M           (0x00000001)
#define BIT_ARXL1_APGA_CTL_ARXL1_DA_EN           (0x001)
#define BIT_ARXL1_APGA_CTL_ARXL1_DA_EN_M         (0x00000002)
#define BIT_ARXL1_APGA_CTL_ARXL1_FM_EN           (0x002)
#define BIT_ARXL1_APGA_CTL_ARXL1_FM_EN_M         (0x00000004)
#define BIT_ARXL1_APGA_CTL_ARXL1_GAIN_SET        (0x003)
#define BIT_ARXL1_APGA_CTL_ARXL1_GAIN_SET_M      (0x000000F8)
/* ARXR1_APGA_CTL Fields */
#define BIT_ARXR1_APGA_CTL_ARXR1_PDZ             (0x000)
#define BIT_ARXR1_APGA_CTL_ARXR1_PDZ_M           (0x00000001)
#define BIT_ARXR1_APGA_CTL_ARXR1_DA_EN           (0x001)
#define BIT_ARXR1_APGA_CTL_ARXR1_DA_EN_M         (0x00000002)
#define BIT_ARXR1_APGA_CTL_ARXR1_FM_EN           (0x002)
#define BIT_ARXR1_APGA_CTL_ARXR1_FM_EN_M         (0x00000004)
#define BIT_ARXR1_APGA_CTL_ARXR1_GAIN_SET        (0x003)
#define BIT_ARXR1_APGA_CTL_ARXR1_GAIN_SET_M      (0x000000F8)
/* ARXL2_APGA_CTL Fields */
#define BIT_ARXL2_APGA_CTL_ARXL2_PDZ             (0x000)
#define BIT_ARXL2_APGA_CTL_ARXL2_PDZ_M           (0x00000001)
#define BIT_ARXL2_APGA_CTL_ARXL2_DA_EN           (0x001)
#define BIT_ARXL2_APGA_CTL_ARXL2_DA_EN_M         (0x00000002)
#define BIT_ARXL2_APGA_CTL_ARXL2_FM_EN           (0x002)
#define BIT_ARXL2_APGA_CTL_ARXL2_FM_EN_M         (0x00000004)
#define BIT_ARXL2_APGA_CTL_ARXL2_GAIN_SET        (0x003)
#define BIT_ARXL2_APGA_CTL_ARXL2_GAIN_SET_M      (0x000000F8)
/* ARXR2_APGA_CTL Fields */
#define BIT_ARXR2_APGA_CTL_ARXR2_PDZ             (0x000)
#define BIT_ARXR2_APGA_CTL_ARXR2_PDZ_M           (0x00000001)
#define BIT_ARXR2_APGA_CTL_ARXR2_DA_EN           (0x001)
#define BIT_ARXR2_APGA_CTL_ARXR2_DA_EN_M         (0x00000002)
#define BIT_ARXR2_APGA_CTL_ARXR2_FM_EN           (0x002)
#define BIT_ARXR2_APGA_CTL_ARXR2_FM_EN_M         (0x00000004)
#define BIT_ARXR2_APGA_CTL_ARXR2_GAIN_SET        (0x003)
#define BIT_ARXR2_APGA_CTL_ARXR2_GAIN_SET_M      (0x000000F8)
/* ATX2ARXPGA Fields */
#define BIT_ATX2ARXPGA_ATX2ARXR_PGA              (0x000)
#define BIT_ATX2ARXPGA_ATX2ARXR_PGA_M            (0x00000007)
#define BIT_ATX2ARXPGA_ATX2ARXL_PGA              (0x003)
#define BIT_ATX2ARXPGA_ATX2ARXL_PGA_M            (0x00000038)
/* BT_IF Fields */
#define BIT_BT_IF_BT_EN                          (0x000)
#define BIT_BT_IF_BT_EN_M                        (0x00000001)
#define BIT_BT_IF_BT_TRI_EN                      (0x002)
#define BIT_BT_IF_BT_TRI_EN_M                    (0x00000004)
#define BIT_BT_IF_BT_SWAP                        (0x004)
#define BIT_BT_IF_BT_SWAP_M                      (0x00000010)
#define BIT_BT_IF_BT_DOUT_EN                     (0x005)
#define BIT_BT_IF_BT_DOUT_EN_M                   (0x00000020)
#define BIT_BT_IF_BT_DIN_EN                      (0x006)
#define BIT_BT_IF_BT_DIN_EN_M                    (0x00000040)
#define BIT_BT_IF_SPARE                          (0x007)
#define BIT_BT_IF_SPARE_M                        (0x00000080)
/* BTPGA Fields */
#define BIT_BTPGA_BTRXPGA_GAIN                   (0x000)
#define BIT_BTPGA_BTRXPGA_GAIN_M                 (0x0000000F)
#define BIT_BTPGA_BTTXPGA_GAIN                   (0x004)
#define BIT_BTPGA_BTTXPGA_GAIN_M                 (0x000000F0)
/* BTSTPGA Fields */
#define BIT_BTSTPGA_BTSTPGA_GAIN                 (0x000)
#define BIT_BTSTPGA_BTSTPGA_GAIN_M               (0x0000003F)
/* EAR_CTL Fields */
#define BIT_EAR_CTL_EAR_VOICE_EN                 (0x000)
#define BIT_EAR_CTL_EAR_VOICE_EN_M               (0x00000001)
#define BIT_EAR_CTL_EAR_AL1_EN                   (0x001)
#define BIT_EAR_CTL_EAR_AL1_EN_M                 (0x00000002)
#define BIT_EAR_CTL_EAR_AL2_EN                   (0x002)
#define BIT_EAR_CTL_EAR_AL2_EN_M                 (0x00000004)
#define BIT_EAR_CTL_EAR_AR1_EN                   (0x003)
#define BIT_EAR_CTL_EAR_AR1_EN_M                 (0x00000008)
#define BIT_EAR_CTL_EAR_GAIN                     (0x004)
#define BIT_EAR_CTL_EAR_GAIN_M                   (0x00000030)
#define BIT_EAR_CTL_SPARE                        (0x006)
#define BIT_EAR_CTL_SPARE_M                      (0x00000040)
#define BIT_EAR_CTL_EAR_OUTLOW_EN                (0x007)
#define BIT_EAR_CTL_EAR_OUTLOW_EN_M              (0x00000080)
/* HS_GAIN_SET Fields */
#define BIT_HS_GAIN_SET_HSL_GAIN                 (0x000)
#define BIT_HS_GAIN_SET_HSL_GAIN_M               (0x00000003)
#define BIT_HS_GAIN_SET_HSR_GAIN                 (0x002)
#define BIT_HS_GAIN_SET_HSR_GAIN_M               (0x0000000C)
#define BIT_HS_GAIN_SET_SPARE                    (0x006)
#define BIT_HS_GAIN_SET_SPARE_M                  (0x00000040)
/* HS_SEL Fields */
#define BIT_HS_SEL_HSOL_VOICE_EN                 (0x000)
#define BIT_HS_SEL_HSOL_VOICE_EN_M               (0x00000001)
#define BIT_HS_SEL_HSOL_AL1_EN                   (0x001)
#define BIT_HS_SEL_HSOL_AL1_EN_M                 (0x00000002)
#define BIT_HS_SEL_HSOL_AL2_EN                   (0x002)
#define BIT_HS_SEL_HSOL_AL2_EN_M                 (0x00000004)
#define BIT_HS_SEL_HSOR_VOICE_EN                 (0x003)
#define BIT_HS_SEL_HSOR_VOICE_EN_M               (0x00000008)
#define BIT_HS_SEL_HSOR_AR1_EN                   (0x004)
#define BIT_HS_SEL_HSOR_AR1_EN_M                 (0x00000010)
#define BIT_HS_SEL_HSOR_AR2_EN                   (0x005)
#define BIT_HS_SEL_HSOR_AR2_EN_M                 (0x00000020)
#define BIT_HS_SEL_HS_OUTLOW_EN                  (0x006)
#define BIT_HS_SEL_HS_OUTLOW_EN_M                (0x00000040)
#define BIT_HS_SEL_HSR_INV_EN                    (0x007)
#define BIT_HS_SEL_HSR_INV_EN_M                  (0x00000080)
/* HS_POPN_SET Fields */
#define BIT_HS_POPN_SET_RAMP_EN                  (0x001)
#define BIT_HS_POPN_SET_RAMP_EN_M                (0x00000002)
#define BIT_HS_POPN_SET_RAMP_DELAY               (0x002)
#define BIT_HS_POPN_SET_RAMP_DELAY_M             (0x0000001C)
#define BIT_HS_POPN_SET_EXTMUTE                  (0x005)
#define BIT_HS_POPN_SET_EXTMUTE_M                (0x00000020)
#define BIT_HS_POPN_SET_VMID_EN                  (0x006)
#define BIT_HS_POPN_SET_VMID_EN_M                (0x00000040)
/* PREDL_CTL Fields */
#define BIT_PREDL_CTL_PREDL_VOICE_EN             (0x000)
#define BIT_PREDL_CTL_PREDL_VOICE_EN_M           (0x00000001)
#define BIT_PREDL_CTL_PREDL_AL1_EN               (0x001)
#define BIT_PREDL_CTL_PREDL_AL1_EN_M             (0x00000002)
#define BIT_PREDL_CTL_PREDL_AL2_EN               (0x002)
#define BIT_PREDL_CTL_PREDL_AL2_EN_M             (0x00000004)
#define BIT_PREDL_CTL_PREDL_AR2_EN               (0x003)
#define BIT_PREDL_CTL_PREDL_AR2_EN_M             (0x00000008)
#define BIT_PREDL_CTL_PREDL_GAIN                 (0x004)
#define BIT_PREDL_CTL_PREDL_GAIN_M               (0x00000030)
#define BIT_PREDL_CTL_PREDL_OUTLOW_EN            (0x007)
#define BIT_PREDL_CTL_PREDL_OUTLOW_EN_M          (0x00000080)
/* PREDR_CTL Fields */
#define BIT_PREDR_CTL_PREDR_VOICE_EN             (0x000)
#define BIT_PREDR_CTL_PREDR_VOICE_EN_M           (0x00000001)
#define BIT_PREDR_CTL_PREDR_AR1_EN               (0x001)
#define BIT_PREDR_CTL_PREDR_AR1_EN_M             (0x00000002)
#define BIT_PREDR_CTL_PREDR_AR2_EN               (0x002)
#define BIT_PREDR_CTL_PREDR_AR2_EN_M             (0x00000004)
#define BIT_PREDR_CTL_PREDR_AL2_EN               (0x003)
#define BIT_PREDR_CTL_PREDR_AL2_EN_M             (0x00000008)
#define BIT_PREDR_CTL_PREDR_GAIN                 (0x004)
#define BIT_PREDR_CTL_PREDR_GAIN_M               (0x00000030)
#define BIT_PREDR_CTL_PREDR_OUTLOW_EN            (0x007)
#define BIT_PREDR_CTL_PREDR_OUTLOW_EN_M          (0x00000080)
/* PRECKL_CTL Fields */
#define BIT_PRECKL_CTL_PRECKL_VOICE_EN           (0x000)
#define BIT_PRECKL_CTL_PRECKL_VOICE_EN_M         (0x00000001)
#define BIT_PRECKL_CTL_PRECKL_AL1_EN             (0x001)
#define BIT_PRECKL_CTL_PRECKL_AL1_EN_M           (0x00000002)
#define BIT_PRECKL_CTL_PRECKL_AL2_EN             (0x002)
#define BIT_PRECKL_CTL_PRECKL_AL2_EN_M           (0x00000004)
#define BIT_PRECKL_CTL_PRECKL_GAIN               (0x004)
#define BIT_PRECKL_CTL_PRECKL_GAIN_M             (0x00000030)
#define BIT_PRECKL_CTL_PRECKL_EN                 (0x006)
#define BIT_PRECKL_CTL_PRECKL_EN_M               (0x00000040)
/* PRECKR_CTL Fields */
#define BIT_PRECKR_CTL_PRECKR_VOICE_EN           (0x000)
#define BIT_PRECKR_CTL_PRECKR_VOICE_EN_M         (0x00000001)
#define BIT_PRECKR_CTL_PRECKR_AR1_EN             (0x001)
#define BIT_PRECKR_CTL_PRECKR_AR1_EN_M           (0x00000002)
#define BIT_PRECKR_CTL_PRECKR_AR2_EN             (0x002)
#define BIT_PRECKR_CTL_PRECKR_AR2_EN_M           (0x00000004)
#define BIT_PRECKR_CTL_PRECKR_GAIN               (0x004)
#define BIT_PRECKR_CTL_PRECKR_GAIN_M             (0x00000030)
#define BIT_PRECKR_CTL_PRECKR_EN                 (0x006)
#define BIT_PRECKR_CTL_PRECKR_EN_M               (0x00000040)
#define HANDS_FREEL_VOICE                        (0x0)  
#define HANDS_FREEL_AL1                          (0x1)
#define HANDS_FREEL_AL2                          (0x2)
#define HANDS_FREEL_AR2                          (0x3)
#define HANDS_FREER_VOICE                        (0x0)
#define HANDS_FREER_AR1                          (0x1)
#define HANDS_FREER_AR2                          (0x2)
#define HANDS_FREER_AL2                          (0x3)
/* HFL_CTL Fields */
#define BIT_HFL_CTL_HFL_INPUT_SEL                (0x000)
#define BIT_HFL_CTL_HFL_INPUT_SEL_M              (0x00000003)
#define BIT_HFL_CTL_HFL_HB_EN                    (0x002)
#define BIT_HFL_CTL_HFL_HB_EN_M                  (0x00000004)
#define BIT_HFL_CTL_HFL_LOOP_EN                  (0x003)
#define BIT_HFL_CTL_HFL_LOOP_EN_M                (0x00000008)
#define BIT_HFL_CTL_HFL_RAMP_EN                  (0x004)
#define BIT_HFL_CTL_HFL_RAMP_EN_M                (0x00000010)
#define BIT_HFL_CTL_HFL_REF_EN                   (0x005)
#define BIT_HFL_CTL_HFL_REF_EN_M                 (0x00000020)
/* HFR_CTL Fields */
#define BIT_HFR_CTL_HFR_INPUT_SEL                (0x000)
#define BIT_HFR_CTL_HFR_INPUT_SEL_M              (0x00000003)
#define BIT_HFR_CTL_HFR_HB_EN                    (0x002)
#define BIT_HFR_CTL_HFR_HB_EN_M                  (0x00000004)
#define BIT_HFR_CTL_HFR_LOOP_EN                  (0x003)
#define BIT_HFR_CTL_HFR_LOOP_EN_M                (0x00000008)
#define BIT_HFR_CTL_HFR_RAMP_EN                  (0x004)
#define BIT_HFR_CTL_HFR_RAMP_EN_M                (0x00000010)
#define BIT_HFR_CTL_HFR_REF_EN                   (0x005)
#define BIT_HFR_CTL_HFR_REF_EN_M                 (0x00000020)
/* ALC_CTL Fields */
#define BIT_ALC_CTL_ALC_WAIT                     (0x000)
#define BIT_ALC_CTL_ALC_WAIT_M                   (0x00000007)
#define BIT_ALC_CTL_MAINMIC_ALC_EN               (0x003)
#define BIT_ALC_CTL_MAINMIC_ALC_EN_M             (0x00000008)
#define BIT_ALC_CTL_SUBMIC_ALC_EN                (0x004)
#define BIT_ALC_CTL_SUBMIC_ALC_EN_M              (0x00000010)
#define BIT_ALC_CTL_ALC_MODE                     (0x005)
#define BIT_ALC_CTL_ALC_MODE_M                   (0x00000020)
#define BIT_ALC_CTL_SPARE1                       (0x006)
#define BIT_ALC_CTL_SPARE1_M                     (0x00000040)
#define BIT_ALC_CTL_SPARE2                       (0x007)
#define BIT_ALC_CTL_SPARE2_M                     (0x00000080)
/* ALC_SET1 Fields */
#define BIT_ALC_SET1_ALC_MIN_LIMIT               (0x000)
#define BIT_ALC_SET1_ALC_MIN_LIMIT_M             (0x00000007)
#define BIT_ALC_SET1_ALC_MAX_LIMIT               (0x003)
#define BIT_ALC_SET1_ALC_MAX_LIMIT_M             (0x00000038)
/* ALC_SET2 Fields */
#define BIT_ALC_SET2_ALC_RELEASE                 (0x000)
#define BIT_ALC_SET2_ALC_RELEASE_M               (0x00000007)
#define BIT_ALC_SET2_ALC_ATTACK                  (0x003)
#define BIT_ALC_SET2_ALC_ATTACK_M                (0x00000038)
#define BIT_ALC_SET2_ALC_STEP                    (0x006)
#define BIT_ALC_SET2_ALC_STEP_M                  (0x00000040)
/* BOOST_CTL Fields */
#define BIT_BOOST_CTL_EFFECT                     (0x000)
#define BIT_BOOST_CTL_EFFECT_M                   (0x00000003)
/* SOFTVOL_CTL Fields */
#define BIT_SOFTVOL_CTL_SOFTVOL_EN               (0x000)
#define BIT_SOFTVOL_CTL_SOFTVOL_EN_M             (0x00000001)
#define BIT_SOFTVOL_CTL_SOFTVOL_SET              (0x005)
#define BIT_SOFTVOL_CTL_SOFTVOL_SET_M            (0x000000E0)
/* DTMF_FREQSEL Fields */
#define BIT_DTMF_FREQSEL_FREQSEL_EXT_M           (0x00000014)
#define BIT_DTMF_FREQSEL_FREQSEL                 (0x000)
#define BIT_DTMF_FREQSEL_FREQSEL_M               (0x0000001F)
#define BIT_DTMF_FREQSEL_SPARE1                  (0x006)
#define BIT_DTMF_FREQSEL_SPARE1_M                (0x00000040)
#define BIT_DTMF_FREQSEL_SPARE2                  (0x007)
#define BIT_DTMF_FREQSEL_SPARE2_M                (0x00000080)
/* DTMF_TONEXT1H Fields */
#define BIT_DTMF_TONEXT1H_EXT_TONE1H             (0x000)
#define BIT_DTMF_TONEXT1H_EXT_TONE1H_M           (0x000000FF)
/* DTMF_TONEXT1L Fields */
#define BIT_DTMF_TONEXT1L_EXT_TONE1L             (0x000)
#define BIT_DTMF_TONEXT1L_EXT_TONE1L_M           (0x000000FF)
/* DTMF_TONEXT2H Fields */
#define BIT_DTMF_TONEXT2H_EXT_TONE2H             (0x000)
#define BIT_DTMF_TONEXT2H_EXT_TONE2H_M           (0x000000FF)
/* DTMF_TONEXT2L Fields */
#define BIT_DTMF_TONEXT2L_EXT_TONE2L             (0x000)
#define BIT_DTMF_TONEXT2L_EXT_TONE2L_M           (0x000000FF)
/* DTMF_TONOFF Fields */
#define BIT_DTMF_TONOFF_TONE_ON_TIME             (0x000)
#define BIT_DTMF_TONOFF_TONE_ON_TIME_M           (0x0000000F)
#define BIT_DTMF_TONOFF_TONE_OFF_TIME            (0x004)
#define BIT_DTMF_TONOFF_TONE_OFF_TIME_M          (0x000000F0)
/* DTMF_WANONOFF Fields */
#define BIT_DTMF_WANONOFF_WAMBLE_ON_TIME         (0x000)
#define BIT_DTMF_WANONOFF_WAMBLE_ON_TIME_M       (0x0000000F)
#define BIT_DTMF_WANONOFF_WAMBLE_OFF_TIME        (0x004)
#define BIT_DTMF_WANONOFF_WAMBLE_OFF_TIME_M      (0x000000F0)
/* I2S_RX_SCRAMBLE_H Fields */
#define BIT_I2S_RX_SCRAMBLE_H_I2S_RX_SCRAMBLE_H  (0x000)
#define BIT_I2S_RX_SCRAMBLE_H_I2S_RX_SCRAMBLE_H_M (0x000000FF)
/* I2S_RX_SCRAMBLE_M Fields */
#define BIT_I2S_RX_SCRAMBLE_M_I2S_RX_SCRAMBLE_M  (0x000)
#define BIT_I2S_RX_SCRAMBLE_M_I2S_RX_SCRAMBLE_M_M (0x000000FF)
/* I2S_RX_SCRAMBLE_L Fields */
#define BIT_I2S_RX_SCRAMBLE_L_I2S_RX_SCRAMBLE_L  (0x000)
#define BIT_I2S_RX_SCRAMBLE_L_I2S_RX_SCRAMBLE_L_M (0x000000FF)
/* APLL_CTL Fields */
#define APLL_CTL_FREQ_19_2MHZ                    (0x5)
#define APLL_CTL_FREQ_26_0MHZ                    (0x6)
#define APLL_CTL_FREQ_38_4MHZ                    (0xF)
#define BIT_APLL_CTL_APLL_INFREQ                 (0x000)
#define BIT_APLL_CTL_APLL_INFREQ_M               (0x0000000F)
#define BIT_APLL_CTL_APLL_EN                     (0x004)
#define BIT_APLL_CTL_APLL_EN_M                   (0x00000010)
/* DTMF_CTL Fields */
#define BIT_DTMF_CTL_TONE_EN                     (0x000)
#define BIT_DTMF_CTL_TONE_EN_M                   (0x00000001)
#define BIT_DTMF_CTL_DUAL_TONE_EN                (0x001)
#define BIT_DTMF_CTL_DUAL_TONE_EN_M              (0x00000002)
#define BIT_DTMF_CTL_TONE_PATTERN                (0x002)
#define BIT_DTMF_CTL_TONE_PATTERN_M              (0x00000004)
#define BIT_DTMF_CTL_TONE_MODE                   (0x003)
#define BIT_DTMF_CTL_TONE_MODE_M                 (0x00000008)
/* DTMF_PGA_CTL2 Fields */
#define BIT_DTMF_PGA_CTL2_TONE3_GAIN             (0x000)
#define BIT_DTMF_PGA_CTL2_TONE3_GAIN_M           (0x00000007)
#define BIT_DTMF_PGA_CTL2_TONE4_GAIN             (0x004)
#define BIT_DTMF_PGA_CTL2_TONE4_GAIN_M           (0x00000070)
#define BIT_DTMF_PGA_CTL2_TONE_18DB_ATT          (0x007)
#define BIT_DTMF_PGA_CTL2_TONE_18DB_ATT_M        (0x00000080)
/* DTMF_PGA_CTL1 Fields */
#define BIT_DTMF_PGA_CTL1_TONE1_GAIN             (0x000)
#define BIT_DTMF_PGA_CTL1_TONE1_GAIN_M           (0x0000000F)
#define BIT_DTMF_PGA_CTL1_TONE2_GAIN             (0x004)
#define BIT_DTMF_PGA_CTL1_TONE2_GAIN_M           (0x000000F0)
/* MISC_SET_1 Fields */
#define BIT_MISC_SET_1_DIGMIC_LR_SWAP_EN         (0x000)
#define BIT_MISC_SET_1_DIGMIC_LR_SWAP_EN_M       (0x00000001)
#define BIT_MISC_SET_1_SPARE1                    (0x001)
#define BIT_MISC_SET_1_SPARE1_M                  (0x00000002)
#define BIT_MISC_SET_1_SPARE2                    (0x002)
#define BIT_MISC_SET_1_SPARE2_M                  (0x00000004)
#define BIT_MISC_SET_1_FMLOOP_EN                 (0x005)
#define BIT_MISC_SET_1_FMLOOP_EN_M               (0x00000020)
#define BIT_MISC_SET_1_SCRAMBLE_EN               (0x006)
#define BIT_MISC_SET_1_SCRAMBLE_EN_M             (0x00000040)
#define BIT_MISC_SET_1_CLK64_EN                  (0x007)
#define BIT_MISC_SET_1_CLK64_EN_M                (0x00000080)
/* PCMBTMUX Fields */
#define BIT_PCMBTMUX_TNRXACT_BT                  (0x000)
#define BIT_PCMBTMUX_TNRXACT_BT_M                (0x00000001)
#define BIT_PCMBTMUX_TNTXACT_BT                  (0x001)
#define BIT_PCMBTMUX_TNTXACT_BT_M                (0x00000002)
#define BIT_PCMBTMUX_TNRXACT_PCM                 (0x002)
#define BIT_PCMBTMUX_TNRXACT_PCM_M               (0x00000004)
#define BIT_PCMBTMUX_TNTXACT_PCM                 (0x003)
#define BIT_PCMBTMUX_TNTXACT_PCM_M               (0x00000008)
#define BIT_PCMBTMUX_MUXRX_BT                    (0x005)
#define BIT_PCMBTMUX_MUXRX_BT_M                  (0x00000020)
#define BIT_PCMBTMUX_MUXRX_PCM                   (0x006)
#define BIT_PCMBTMUX_MUXRX_PCM_M                 (0x00000040)
#define BIT_PCMBTMUX_MUXTX_PCM                   (0x007)
#define BIT_PCMBTMUX_MUXTX_PCM_M                 (0x00000080)
/* RX_PATH_SEL Fields */
#define BIT_RX_PATH_SEL_RXR1_SEL                 (0x000)
#define BIT_RX_PATH_SEL_RXR1_SEL_M               (0x00000003)
#define BIT_RX_PATH_SEL_RXL1_SEL                 (0x002)
#define BIT_RX_PATH_SEL_RXL1_SEL_M               (0x0000000C)
#define BIT_RX_PATH_SEL_RXR2_SEL                 (0x004)
#define BIT_RX_PATH_SEL_RXR2_SEL_M               (0x00000010)
#define BIT_RX_PATH_SEL_RXL2_SEL                 (0x005)
#define BIT_RX_PATH_SEL_RXL2_SEL_M               (0x00000020)
/* VDL_APGA_CTL Fields */
#define BIT_VDL_APGA_CTL_VDL_PDZ                 (0x000)
#define BIT_VDL_APGA_CTL_VDL_PDZ_M               (0x00000001)
#define BIT_VDL_APGA_CTL_VDL_DA_EN               (0x001)
#define BIT_VDL_APGA_CTL_VDL_DA_EN_M             (0x00000002)
#define BIT_VDL_APGA_CTL_VDL_FM_EN               (0x002)
#define BIT_VDL_APGA_CTL_VDL_FM_EN_M             (0x00000004)
#define BIT_VDL_APGA_CTL_VDL_GAIN_SET            (0x003)
#define BIT_VDL_APGA_CTL_VDL_GAIN_SET_M          (0x000000F8)
/* VIBRA_CTL Fields */
#define BIT_VIBRA_CTL_VIVRA_EN                   (0x000)
#define BIT_VIBRA_CTL_VIVRA_EN_M                 (0x00000001)
#define BIT_VIBRA_CTL_VIBRA_DIR                  (0x001)
#define BIT_VIBRA_CTL_VIBRA_DIR_M                (0x00000002)
#define BIT_VIBRA_CTL_VIBRA_AUDIO_SEL            (0x002)
#define BIT_VIBRA_CTL_VIBRA_AUDIO_SEL_M          (0x0000000C)
#define BIT_VIBRA_CTL_VIBRA_SEL                  (0x004)
#define BIT_VIBRA_CTL_VIBRA_SEL_M                (0x00000010)
#define BIT_VIBRA_CTL_VIBRA_DIR_SEL              (0x005)
#define BIT_VIBRA_CTL_VIBRA_DIR_SEL_M            (0x00000020)
#define BIT_VIBRA_CTL_SPARE                      (0x006)
#define BIT_VIBRA_CTL_SPARE_M                    (0x00000040)
/* VIBRA_SET Fields */
#define BIT_VIBRA_SET_VIBRA_AVG_VAL              (0x000)
#define BIT_VIBRA_SET_VIBRA_AVG_VAL_M            (0x000000FF)
/* VIBRA_PWM_SET Fields */
#define BIT_VIBRA_PWM_SET_PWM_ON                 (0x000)
#define BIT_VIBRA_PWM_SET_PWM_ON_M               (0x000000FF)
#define MICAM_GAIN_MIN                           (0x0)
#define MICAM_GAIN_MAX                           (0x5)
/* ANAMIC_GAIN Fields */
#define BIT_ANAMIC_GAIN_MICAMPL_GAIN             (0x000)
#define BIT_ANAMIC_GAIN_MICAMPL_GAIN_M           (0x00000007)
#define BIT_ANAMIC_GAIN_MICAMPR_GAIN             (0x003)
#define BIT_ANAMIC_GAIN_MICAMPR_GAIN_M           (0x00000038)
/* MISC_SET_2 Fields */
#define BIT_MISC_SET_2_SPARE1                    (0x000)
#define BIT_MISC_SET_2_SPARE1_M                  (0x00000001)
#define BIT_MISC_SET_2_VRX_HPF_BYP               (0x001)
#define BIT_MISC_SET_2_VRX_HPF_BYP_M             (0x00000002)
#define BIT_MISC_SET_2_VTX_HPF_BYP               (0x002)
#define BIT_MISC_SET_2_VTX_HPF_BYP_M             (0x00000004)
#define BIT_MISC_SET_2_ARX_HPF_BYP               (0x003)
#define BIT_MISC_SET_2_ARX_HPF_BYP_M             (0x00000008)
#define BIT_MISC_SET_2_VRX_3RD_HPF_BYP           (0x004)
#define BIT_MISC_SET_2_VRX_3RD_HPF_BYP_M         (0x00000010)
#define BIT_MISC_SET_2_ATX_HPF_BYP               (0x005)
#define BIT_MISC_SET_2_ATX_HPF_BYP_M             (0x00000020)
#define BIT_MISC_SET_2_VTX_3RD_HPF_BYP           (0x006)
#define BIT_MISC_SET_2_VTX_3RD_HPF_BYP_M         (0x00000040)
#define BIT_MISC_SET_2_SPARE2                    (0x007)
#define BIT_MISC_SET_2_SPARE2_M                  (0x00000080)

/************** INTERNAL DETAILS ********************/
/*
 * Mapping of OSS devices to TWL devices
 * TWL Device                      OSS MASK
 * ==============================================================
 * OUTPUT:
 * =======
 * OUTPUT_STEREO_HEADSET           SOUND_MASK_LINE1
 * OUTPUT_HANDS_FREE_CLASSD        SOUND_MASK_SPEAKER
 * OUTPUT_MONO_EARPIECE            SOUND_MASK_PHONEOUT (mono Sink)
 * 
 * INPUT:
 * ======
 * INPUT_HEADSET_MIC               SOUND_MASK_LINE
 * INPUT_MAIN_MIC + INPUT_SUB_MIC  SOUND_MASK_MIC
 *
 * CURRENT SOURCES:
 * ===============
 * SOUND_MIXER_OUTSRC - output source
 * SOUND_MIXER_RECSRC - input source
 * Operations:
 * MIXER_READ() and MIXER_WRITE() to control the sources
 *
 * VOLUME CONTROL:
 * ===============
 * SOUND_MIXER_RECLEV - control the gain level of recording
 * SOUND_MIXER_VOLUME - control the gain level of playback
 * MIXER_WRITE and MIXER_READ with each of the device masks will 
 *                            control the coarse volume control of the device
 */
#define DIR_OUT                                  (0)
#define DIR_IN                                   (1<<7)

#define OUTPUT_VOLUME                            (DIR_OUT | 0)
#define OUTPUT_STEREO_HEADSET                    (DIR_OUT | (1 << 0))
#define OUTPUT_HANDS_FREE_CLASSD                 (DIR_OUT | (1 << 1))
#define OUTPUT_MONO_EARPIECE                     (DIR_OUT | (1 << 2))
#define OUTPUT_SIDETONE                          (DIR_OUT | (1 << 3))
#define OUTPUT_CARKIT                            (DIR_OUT | (1 << 4))

#define INPUT_VOLUME                             (DIR_IN | 0)
#define INPUT_HEADSET_MIC                        (DIR_IN | (1 << 0))
#define INPUT_MAIN_MIC                           (DIR_IN | (1 << 1))
#define INPUT_SUB_MIC                            (DIR_IN | (1 << 2))
#define INPUT_AUX                                (DIR_IN | (1 << 3))
#define INPUT_CARKIT                             (DIR_IN | (1 << 4))

#define DEFAULT_INPUT_TWL_DEVICE                 INPUT_HEADSET_MIC
#define DEFAULT_INPUT_LNX_DEVICE                 SOUND_MASK_LINE
#define DEFAULT_OUTPUT_TWL_DEVICE                OUTPUT_STEREO_HEADSET
#define DEFAULT_OUTPUT_LNX_DEVICE                SOUND_MASK_LINE1
/* Recording devices */
#define REC_SRC_MASK                             (SOUND_MASK_LINE | \
						  SOUND_MASK_MIC)
#define REC_MASK                                 (SOUND_MASK_RECLEV | \
						  REC_SRC_MASK)
/* play back devices */
#define OUT_SRC_MASK                             (SOUND_MASK_LINE1 | \
						  SOUND_MASK_SPEAKER | \
						  SOUND_MASK_PHONEOUT)
#define OUT_MASK                                 (SOUND_MASK_VOLUME | \
						  OUT_SRC_MASK)
#define DEV_MASK                                 (REC_MASK | \
						  OUT_MASK)
#define DEV_STEREO_DEV                           (SOUND_MASK_VOLUME | \
						  SOUND_MASK_RECLEV | \
		                                  SOUND_MASK_LINE1 | \
						  SOUND_MASK_SPEAKER | \
						  SOUND_MASK_MIC)

#define STEREO_MODE                              (0x1)
#define MONO_MODE                                (0x2)

#define MIXER_DEVICE                             (0x0)
#define DSP_DEVICE                               (0x1)

#define AUDIO_MAX_INPUT_VOLUME                   100
#define AUDIO_MAX_OUTPUT_VOLUME                  100
#define COMPUTE_PRECISION                        100
#define AUDIO_INPUT_INCREMENT                    ((AUDIO_MAX_INPUT_VOLUME * \
						   COMPUTE_PRECISION)/\
						   (INPUT_GAIN_MAX))
#define AUDIO_OUTPUT_INCREMENT                   ((AUDIO_MAX_OUTPUT_VOLUME * \
						   COMPUTE_PRECISION)/\
						   (OUTPUT_GAIN_MAX))
#define AUDIO_DEF_COARSE_VOLUME_LEVEL            AUDIO_OUTPUT_COARSE_GAIN_6DB
#define MIC_AMP_INCR                             ((AUDIO_MAX_INPUT_VOLUME * \
						   COMPUTE_PRECISION)/\
						   (MICAM_GAIN_MAX))
#define ARX_APGA_INCR                            ((AUDIO_MAX_OUTPUT_VOLUME * \
		       				   COMPUTE_PRECISION)/\
						   (ARX_APGA_MIN))
/* EAR/ HS Values */
#define NON_LIN_VALS                             { 3, 2, 1 }
#define NON_LIN_GAIN_MAX                         3
#define NON_LIN_GAIN_MAX_R                       2
#define NON_LIN_INCREMENT                        ((AUDIO_MAX_OUTPUT_VOLUME * \
						   COMPUTE_PRECISION)/\
						   (NON_LIN_GAIN_MAX_R))
#define NON_LIN_MUTE                             0

#define DEFAULT_OUTPUT_VOLUME                    70
#define DEFAULT_OUTPUT_SPK_VOLUME                50
#define DEFAULT_OUTPUT_HSET_VOLUME               70
#define DEFAULT_OUTPUT_EAR_VOLUME                60
#define DEFAULT_SIDETONE_VOLUME                  20
#define DEFAULT_OUTPUT_CARKIT_VOLUME             50

#define DEFAULT_INPUT_VOLUME                     60
#define DEFAULT_INPUT_MIC_VOLUME                 80
#define DEFAULT_INPUT_LINE_VOLUME                80
#define DEFAULT_INPUT_CARKIT_VOLUME              50
#define AUDIO_RATE_DEFAULT                       8000

/* supports only two sample sizes - 16 and 24 */
#define AUDIO_SAMPLE_DATA_WIDTH_16               16
#define AUDIO_SAMPLE_DATA_WIDTH_24               24

 /* From OSS spec:
  * The least significant
  * byte gives volume for the left channel and the next 8 bits for 
  * the right channel.
  */
#define READ_LEFT_VOLUME(ARG)                    ( (ARG) & 0xFF )
#define READ_RIGHT_VOLUME(ARG)                   ( ((ARG) & 0xFF00)>>8 )
#define WRITE_LEFT_VOLUME(ARG)                   ( (ARG) & 0xFF )
#define WRITE_RIGHT_VOLUME(ARG)                  ( ((ARG) & 0xFF)<<8 )
#define WRITE_LR_VOLUME(ARG)                     ( WRITE_RIGHT_VOLUME(ARG) | \
						   WRITE_LEFT_VOLUME(ARG) )

#define DTMF_FREQ_MULTIPLIER                     10000
#define DTMF_FREQ_INCR                           78125
#define DTMF_FREQ_HIGH_FREQ_SUB                  256
#define DTMF_FREQ_HIGH_FREQ_HZ                   2000
#define DTMF_FREQ_MAX                            3992

#define TWL4030_FREQSEL_DEFAULT                  0x13
enum {
	TWL4030_DTMF_PRESET_FREQ_DUAL_TONE_START,
	TWL4030_DTMF_PRESET_FREQ_1209HZ_697HZ =
		TWL4030_DTMF_PRESET_FREQ_DUAL_TONE_START,
	TWL4030_DTMF_PRESET_FREQ_1336HZ_697HZ,
	TWL4030_DTMF_PRESET_FREQ_1477HZ_697HZ,
	TWL4030_DTMF_PRESET_FREQ_1633HZ_697HZ,
	TWL4030_DTMF_PRESET_FREQ_1209HZ_770HZ,
	TWL4030_DTMF_PRESET_FREQ_1336HZ_770HZ,
	TWL4030_DTMF_PRESET_FREQ_1477HZ_770HZ,
	TWL4030_DTMF_PRESET_FREQ_1633HZ_770HZ,
	TWL4030_DTMF_PRESET_FREQ_1209HZ_852HZ,
	TWL4030_DTMF_PRESET_FREQ_1336HZ_852HZ,
	TWL4030_DTMF_PRESET_FREQ_1477HZ_852HZ,
	TWL4030_DTMF_PRESET_FREQ_1633HZ_852HZ,
	TWL4030_DTMF_PRESET_FREQ_1209HZ_941HZ,
	TWL4030_DTMF_PRESET_FREQ_1336HZ_941HZ,
	TWL4030_DTMF_PRESET_FREQ_1477HZ_941HZ,
	TWL4030_DTMF_PRESET_FREQ_1633HZ_941HZ,

	TWL4030_DTMF_PRESET_FREQ_SINGLE_TONE_START,
	TWL4030_DTMF_PRESET_FREQ_400HZ =
		TWL4030_DTMF_PRESET_FREQ_SINGLE_TONE_START,
	TWL4030_DTMF_PRESET_FREQ_425HZ,
	TWL4030_DTMF_PRESET_FREQ_2000HZ,
	TWL4030_DTMF_PRESET_FREQ_2670HZ,

	TWL4030_DTMF_PRESET_FREQ_END,
};

#define DTMF_TONOFF_OFF_TIME_TABLE               { 0, 50, 100, 200, 250, 300, \
	                                               400, 500, 750, 1000, 1500, \
												   2000, 2500, 3000, 3500 }
#define DTMF_TONOFF_ON_TIME_TABLE                { 0, 50, 100, 150, 200, 250, \
												   300, 400, 450, 500, 750, \
												   1000, 1500, 1750, 2000 }
#define DTMF_WOBBLE_TIME_TABLE                   { 0, 31, 50, 100, 200, 250, \
												   300, 350, 400, 450, 500 }

#define CODEC_NAME                               "TWL4030"
#define MIXER_NAME                               "TWL4030 Mixer"

#if defined (CONFIG_ARCH_OMAP243X)
#define PLATFORM_NAME                            "OMAP243X"
#elif defined(CONFIG_ARCH_OMAP3430)
#define PLATFORM_NAME                            "OMAP3430"
#else
#error "UnSupported platform"
#endif

/* Define to set the twl as the master w.r.t McBSP 
 * - remove to make mcbsp master 
 */
#define TWL_MASTER

/* Select the McBSP For Audio */
#define AUDIO_MCBSP                              OMAP2_MCBSP_INTERFACE2

#if defined (CONFIG_MACH_OMAP_2430SDP) || defined (CONFIG_ARCH_OMAP34XX)
#define AUDIO_APLL_DEFAULT                       APLL_CTL_FREQ_26_0MHZ
#else
#define AUDIO_APLL_DEFAULT	            	 APLL_CTL_FREQ_19_2MHZ
#endif


#if 0
/***************** MODULE DATA STRUCTURES **/
static unsigned int twl4030_rates[] = {
	8000, 11025, 12000,
	16000, 22050, 24000,
	32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list twl4030_pcm_hw_constraint_list = {
	.count = ARRAY_SIZE(twl4030_rates),
	.list = twl4030_rates,
	.mask = 0,
};

static struct snd_pcm_hardware twl4030_pcm_hardware_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),	
 	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
#ifdef CONFIG_SND_OMAP_3430DASF
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 128,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 125,
#else
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
#endif
	.fifo_size = 0,
};

static struct snd_pcm_hardware twl4030_pcm_hardware_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

static int mixer_dev_id;

struct twl_local_info {
	/* Global volume control */
	u16 play_volume; /* Store the play volume */
	u16 rec_volume; /* Store the record volume */
	/* Device specific volume control */
	u16 hset; /* Store the hset volume */
	u16 classd; /* Store the classd volume */
	u16 ear; /* Store the ear volume */
	u16 line; /* Store the line volume */
	u16 mic; /* Store the mic volume */
	u16 aux; /* Store aux/FM volume */
	u16 sidetone; /* Store sidetone volume */
	u16 carkit_out; /* Store carkit output gain */
	u16 carkit_in; /* Store carkit input gain */
	/* Source Management */
	int recsrc; /* current active record sources */
	int outsrc; /* current active playback sources */
	int mod_cnt;/* how many usages */
};
static struct twl_local_info twl4030_local = {
	.play_volume =  WRITE_LR_VOLUME(DEFAULT_OUTPUT_VOLUME),
	.rec_volume = WRITE_LR_VOLUME(DEFAULT_INPUT_VOLUME),
	.line = WRITE_LR_VOLUME(DEFAULT_INPUT_LINE_VOLUME),
	.mic = WRITE_LR_VOLUME(DEFAULT_INPUT_MIC_VOLUME),
	.aux = WRITE_LR_VOLUME(DEFAULT_INPUT_LINE_VOLUME),
	.hset = WRITE_LR_VOLUME(DEFAULT_OUTPUT_HSET_VOLUME),
	.classd = WRITE_LR_VOLUME(DEFAULT_OUTPUT_SPK_VOLUME),
	.ear = WRITE_LEFT_VOLUME(DEFAULT_OUTPUT_EAR_VOLUME),
	.sidetone = WRITE_LEFT_VOLUME(DEFAULT_SIDETONE_VOLUME),
	.carkit_out = WRITE_LR_VOLUME(DEFAULT_OUTPUT_CARKIT_VOLUME),
	.carkit_in = WRITE_LEFT_VOLUME(DEFAULT_INPUT_CARKIT_VOLUME),
	.recsrc = DEFAULT_INPUT_LNX_DEVICE,
	.outsrc = DEFAULT_OUTPUT_LNX_DEVICE,
	.mod_cnt = 0,
};

/* Configured count - 
 * Codec is configured when this reaches 1
 * Codec is shutdown when this reaches 0
 */
static char twl4030_configured = 0;

struct mcbsp_config {
	u8 mcbsp_clk_src;	/* source it from prcm? */
	u8 srg_clk_src;		/* clks/fclk/clkr/clkx */
	u8 srg_clk_sync;	/* free_running or just running */
	u8 srg_clk_pol;		/* polarity of srg clk */
	u8 tx_clk_pol;          /* TX clock polarity */
	u8 tx_clk_src;		/* internal/external (master based) */
	u8 rx_clk_pol;          /* TX clock polarity */
	u8 rx_clk_src;		/* internal/external (master based) */
	u8 fs_clk_pol;		/* Frame sync polarity */
	u8 tx_polarity;		/* TX polarity */
	u8 rx_polarity;		/* RX polarity */
	u8 rx_ip_clk;           /* Rx input clock */
	u8 tx_ip_clk;           /* Tx input clock */
	omap2_mcbsp_transfer_params tx_params; /* Transmit parameters */
	omap2_mcbsp_transfer_params rx_params; /* Recieve parameters */
};

/* Default volume */
static int current_input = DEFAULT_INPUT_TWL_DEVICE;
static int current_output = DEFAULT_OUTPUT_TWL_DEVICE;
/* Switch flags */
u8 handsfree_en = 1;
u8 hsmic_en     = 1;
u8 main_mic_en  = 1;
u8 sub_mic_en   = 1;
/* To Store the default sample rate */
static long audio_samplerate = AUDIO_RATE_DEFAULT;
struct sample_rate_info_t {
	u16 rate;
	u8 apll;
};
/* Hint - optimization wise move the most used values up the list */
static const struct sample_rate_info_t valid_sample_rates[] = {
	{.rate = 8000,.apll = AUDIO_MODE_RATE_08_000},
	{.rate = 16000,.apll = AUDIO_MODE_RATE_16_000},
	{.rate = 44100,.apll = AUDIO_MODE_RATE_44_100},
	{.rate = 11025,.apll = AUDIO_MODE_RATE_11_025},
	{.rate = 12000,.apll = AUDIO_MODE_RATE_12_000},
	{.rate = 22050,.apll = AUDIO_MODE_RATE_22_050},
	{.rate = 24000,.apll = AUDIO_MODE_RATE_24_000},
	{.rate = 32000,.apll = AUDIO_MODE_RATE_32_000},
	{.rate = 48000,.apll = AUDIO_MODE_RATE_48_000},
	/* Dont support 96Khz -requires HSCLK >26Mhz
	   { .rate = 96000, .apll = AUDIO_MODE_RATE_96_000 }, */
};

#define NUMBER_OF_RATES_SUPPORTED (sizeof (valid_sample_rates)/\
				   sizeof (struct sample_rate_info_t))

/* Give time for these to settle down 
 * one frame (worst case is at 8000hz) = 125 uSec
 * wait two frame duration each - giving dma a chance to push data thru also
 * =125*4= 500uSec
 */
#define TWL4030_MCBSP2_3430SDP_PRESCALE_TIME 500

/* if we would like to listen to mono sound as stereo on stereo devices
 */
#undef MONO_MODE_SOUNDS_STEREO

/* How long to wait for stream to remain active?in Jiffies */
#define TIMEOUT_WAIT_FOR_ACTIVE       20

/* T2 GPIO for External Mute control - required for pop noise */
#define T2_AUD_EXT_MUT_GPIO 6
#define GPIO_DATA_DIR  0x03
#define GPIO_CLR  0x09
#define GPIO_SET  0x0C

static u8 current_bitspersample = AUDIO_SAMPLE_DATA_WIDTH_16;

static u8 current_stereomode = STEREO_MODE;

/* To handle transfer errors.. */
static int tx_err, rx_err;

/******************** MODULES SPECIFIC FUNCTION PROTOTYPES ********************/
static inline int audio_twl4030_write(u8 address, u8 data);
static inline int audio_twl4030_read(u8 address);
static inline int twl4030_codec_on(void);
static inline int twl4030_codec_off(void);
static int twl4030_enable_output(void);
static int twl4030_disable_output(void);
static int twl4030_enable_input(void);
static int twl4030_disable_input(void);
static int twl4030_disable_input(void);
static int twl4030_select_source(int flag, int val);
static int twl4030_setvolume(int flag, u8 gain_l, u8 gain_r);
static int twl4030_codec_conf_data_path(void);
static int twl4030_conf_data_interface(void);
static int twl4030_set_samplerate(long sample_rate, void *id);
static int twl4030_stereomode_set(int mode, int dsp, void *);
static void twl4030_unconfigure(void);
static int twl4030_configure(void);
static void twl4030_mcbsp_dma_cb(u16 ch_status, void *arg);
static int omap_twl4030_transfer(int mode, void *buffer_phy,u32 size, void *arg, void *id);
static int omap_twl4030_transfer_stop(int mode, void *id);
static int omap_twl4030_transfer_posn(int mode, void *id);
static int omap_twl4030_transfer_init(int mode, void *id);
static int omap_twl4030_initialize(void *id);
static void omap_twl4030_shutdown(void *id);
static int omap_twl4030_sidle(u32 level);
static int omap_twl4030_probe(void);

static int omap_twl4030_mixer_init(struct snd_card *card); 
static int omap_twl4030_mixer_shutdown(struct snd_card *card); 

#ifdef TONE_GEN
void toneGen(void);
#endif

#ifdef TWL_DUMP_REGISTERS
static void twl4030_dumpRegisters(void);
#endif

/******************** DATA STRUCTURES USING FUNCTION POINTERS *****************/

/*
 * The TWL4030 will always use stereo I2S protocol to communicate
 *
 * McBSP Configuration Required:
 * Stereo 16 bit:(default)
 * -------------
 * Single phase, FSYNC=Rising, words=1 DMA->Normal,32bit DXR
 *
 * Stereo 24 bit:
 * -------------
 * Single phase, FSYNC=Falling, words=2 DMA->Normal,32bit DXR
 *
 * Mono 16 bit:
 * ------------
 * Single phase, FSYNC=Rising, words=1 DMA->Normal,16 bit DXR+2
 * OR
 * Single phase, FSYNC=Falling, words=1 DMA->Normal,32bit DXR
 *
 * Mono 24 bit:
 * ------------
 * Single phase, FSYNC=Falling, words=2 DMA-> ei=1,fi=-1,32bit DXR
 *
 */
static struct mcbsp_config plat_mcbsp_config = {
#if defined(CONFIG_ARCH_OMAP243X) || defined(CONFIG_ARCH_OMAP3430)
	.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
#ifdef TWL_MASTER
	/* Driven by twl4030 */
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
	.tx_clk_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
	.rx_clk_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
	.rx_ip_clk = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
	.tx_ip_clk = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,

	.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
#else
	/* Driven by OMAP */
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
	.tx_clk_src = OMAP2_MCBSP_TXFSYNC_INTERNAL,
	.rx_clk_src = OMAP2_MCBSP_RXFSYNC_INTERNAL,
	.rx_ip_clk = OMAP2_MCBSP_CLKRXSRC_INTERNAL,	
	.tx_ip_clk = OMAP2_MCBSP_CLKTXSRC_INTERNAL,

	.srg_clk_sync = OMAP2_MCBSP_SRG_RUNNING,
#endif
	.tx_polarity = OMAP2_MCBSP_FS_ACTIVE_LOW,
	.rx_polarity = OMAP2_MCBSP_FS_ACTIVE_LOW,
	.srg_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
	.tx_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_RISING,
	.rx_clk_pol = OMAP2_MCBSP_CLKR_POLARITY_RISING,
	/* we will start with right channel and transmit the MSB of DXR */
	.fs_clk_pol = OMAP2_MCBSP_FS_ACTIVE_HIGH,
	/* I2S */
	.tx_params = {
			       .data_type = OMAP2_MCBSP_WORDLEN_NONE,
			       .skip_alt = OMAP2_MCBSP_SKIP_NONE,
			       .auto_reset = OMAP2_MCBSP_AUTO_XRST,
			       .phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
			       .data_delay = OMAP2_MCBSP_DATADELAY1,	/* 1 bit delay expected */
			       .reverse_compand = OMAP2_MCBSP_MSBFIRST,	/* Set msb first */
			       .word_length1 = OMAP2_MCBSP_WORDLEN_32,	/* RWDLEN1 */
			       .word_length2 = OMAP2_MCBSP_WORDLEN_8,	/* RWDLEN2 -dnt care*/
			       .frame_length1 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN1 */
			       .frame_length2 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN2 -dnt care*/
			       .justification = OMAP2_MCBSP_RJUST_ZEROMSB,	/* RJUST and fill 0s */
			       .dxena = 0,
			       .dxendly = 0,
			       .callback = twl4030_mcbsp_dma_cb,
			       },
	.rx_params = {
			       .data_type = OMAP2_MCBSP_WORDLEN_NONE,
			       .skip_alt = OMAP2_MCBSP_SKIP_NONE,
			       .auto_reset = OMAP2_MCBSP_AUTO_RRST,
			       .phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
			       .data_delay = OMAP2_MCBSP_DATADELAY1,	/* 1 bit delay expected */
			       .reverse_compand = OMAP2_MCBSP_MSBFIRST,	/* Set msb first */
			       .word_length1 = OMAP2_MCBSP_WORDLEN_32,	/* RWDLEN1 */
			       .word_length2 = OMAP2_MCBSP_WORDLEN_8,	/* RWDLEN2 -dntcare*/
			       .frame_length1 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN1 */
			       .frame_length2 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN2 */
			       .justification = OMAP2_MCBSP_RJUST_ZEROMSB,	/* RJUST and fill 0s */
			       .dxena = 0,
			       .dxendly = 0,
			       .callback = twl4030_mcbsp_dma_cb,
			       },
#else
#error "Unsupported Platform"
#endif
};

static int twl4030_default_samplerate(void)
{
	return AUDIO_RATE_DEFAULT;
}
/* To store characteristic info regarding the codec for the audio driver */
static struct omap_alsa_codec omap_twl_codec = {
	.name = CODEC_NAME,

	.pcm_hw_constraint_list = &twl4030_pcm_hw_constraint_list,
	.pcm_hardware_playback = &twl4030_pcm_hardware_playback,
	.pcm_hardware_capture = &twl4030_pcm_hardware_capture,

	.codec_probe = omap_twl4030_probe,
	.codec_init = omap_twl4030_initialize,
	.codec_shutdown = omap_twl4030_shutdown,
	.codec_sidle = omap_twl4030_sidle,      

	.codec_set_samplerate = twl4030_set_samplerate, 
#ifdef CONFIG_SND_OMAP_3430SDP
	.codec_set_stereomode = twl4030_stereomode_set, 
#endif	
	.codec_default_samplerate = twl4030_default_samplerate,

	.codec_transfer_init = omap_twl4030_transfer_init,
	.codec_transfer_start = omap_twl4030_transfer,
	.codec_transfer_stop = omap_twl4030_transfer_stop,
	.codec_transfer_posn = omap_twl4030_transfer_posn,

	.mixer_init = omap_twl4030_mixer_init,
	.mixer_shutdown = omap_twl4030_mixer_shutdown,
};
#endif

#endif	/* End of __OMAP_AUDIO_TWL4030_H__ */
