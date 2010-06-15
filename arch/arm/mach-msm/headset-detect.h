#ifndef __ARCH_MSM_HEADSET_DETECT_H
#define __ARCH_MSM_HEADSET_DETECT_H

struct chuck_headset_detect_platform_data {
	    int polarity;
};


#define ADIE_CODEC_EN3_R                                  0x37

#define ADIE_CODEC_EN3_INIT_M                             0xFF

#define ADIE_CODEC_EN3_EN_MICBIAS_M                       0x80
#define ADIE_CODEC_EN3_MICBIAS_ENA_V                      0x80
#define ADIE_CODEC_EN3_MICBIAS_DIS_V                      0x00

#define ADIE_CODEC_EN3_EN_TXCK_L_M                        0x40
#define ADIE_CODEC_EN3_TXCK_L_ENA_V                       0x40
#define ADIE_CODEC_EN3_TXCK_L_DIS_V                       0x00

#define ADIE_CODEC_EN3_EN_DITHER_L_M                      0x20
#define ADIE_CODEC_EN3_DITHER_L_ENA_V                     0x20
#define ADIE_CODEC_EN3_DITHER_L_DIS_V                     0x00

#define ADIE_CODEC_EN3_EN_TXADC_L_M                       0x10
#define ADIE_CODEC_EN3_TXADC_L_ENA_V                      0x10
#define ADIE_CODEC_EN3_TXADC_L_DIS_V                      0x00

#define ADIE_CODEC_EN3_EN_AAF_L_M                         0x08
#define ADIE_CODEC_EN3_AAF_L_ENA_V                        0x08
#define ADIE_CODEC_EN3_AAF_L_DIS_V                        0x00

#define ADIE_CODEC_EN3_EN_MICAMP2_L_M                     0x04
#define ADIE_CODEC_EN3_MICAMP2_L_ENA_V                    0x04
#define ADIE_CODEC_EN3_MICAMP2_L_DIS_V                    0x00

#define ADIE_CODEC_EN3_EN_MICAMP1_L_M                     0x02
#define ADIE_CODEC_EN3_MICAMP1_L_ENA_V                    0x02
#define ADIE_CODEC_EN3_MICAMP1_L_DIS_V                    0x00

#define ADIE_CODEC_EN3_EN_REF_TXFE_L_M                    0x01
#define ADIE_CODEC_EN3_REF_TXFE_L_ENA_V                   0x01
#define ADIE_CODEC_EN3_REF_TXFE_L_DIS_V                   0x00

#define ADIE_CODEC_EN3_INIT_V                             0x00

#endif
