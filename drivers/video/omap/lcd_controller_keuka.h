#ifndef LCD_CONTROLLER_KEUKA_H
#define LCD_CONTROLLER_KEUKA_H

/* Command, no arguments */
#define KEUKA_OP_NONE                                   0x00
#define KEUKA_OP_INIT_DISPLAY                           0x01
#define KEUKA_OP_SLEEP_IN                               0x10
#define KEUKA_OP_SLEEP_OUT                              0x11
#define KEUKA_OP_PARTIAL_MODE_ON                        0x12
#define KEUKA_OP_NORMAL_MODE_ON                         0x13
#define KEUKA_OP_DISPLAY_OFF                            0x28
#define KEUKA_OP_DISPLAY_ON                             0x29
#define KEUKA_OP_READ_ID0                               0xDA
#define KEUKA_OP_READ_ID1                               0xDB
#define KEUKA_OP_READ_ID2                               0xDC
#define KEUKA_OP_ENTER_REGISTER_ACCESS_MODE             0xDE
#define KEUKA_OP_ENTER_COMMAND_MODE                     0xDF

/* Write, 0..n arguments */
#define KEUKA_OP_MEMORY_WRITE                           0x2C
/* Write, 4 arguments */
#define KEUKA_OP_PARTIAL_AREA_LINE                      0x30
/* Write, 4 arguments */
#define KEUKA_OP_PARTIAL_AREA_COLUMN                    0x31

/* Register */
#define KEUKA_REG_ACD_MD_GR                             0x15
/* Register, Color used for pixels outside partial refresh window (MSB) */
#define KEUKA_REG_BORDER_MSB_PM                         0x1B
/* Register, Color used for pixels outside partial refresh window (LSB) */
#define KEUKA_REG_BORDER_LSB_PM                         0x1C
/* Register, Operating mode for some of the glass control signals */
#define KEUKA_REG_SCAN                                  0x1D
/* Register, Inversion modes */
#define KEUKA_REG_INVERSION_MODE                        0x1E
/* Register, Partial mode timing (CKH1 high time) */
#define KEUKA_REG_CKH1_HI_PM                            0x1F
/* Register, Partial mode timing (CKH2 and CKH3 high time) */
#define KEUKA_REG_CKH_HI_PM                             0x20
/* Register, Partial mode timing (CKH non overlap time) */
#define KEUKA_REG_CKH_NOV_PM                            0x21
/* Register, Partial mode timing (Number of OSC cycles
 * from the end of a line to CKH1 rising)
 */
#define KEUKA_REG_DE_CKH1_PM                            0x22
/* Register, Partial mode timing (GOE) */
#define KEUKA_REG_DE_GOE_RE_PM                          0x23
/* Register, REC */
#define KEUKA_REG_REC_RE_CTL                            0x28
/* Register, REC */
#define KEUKA_REG_REC_FE_CTL                            0x29
#define KEUKA_REG_BITS_PER_PIXEL                        0x2A
#define KEUKA_REG_VBG_ADJ                               0x2B
#define KEUKA_REG_RAM_PORT                              0x2C
#define KEUKA_REG_I_OSC_ADJ                             0x2D

/* Register, Gamma circuit Vref adjustment */
#define KEUKA_REG_VDD_ADJ                               0x2E

#define KEUKA_REG_MODE                                  0x30
#define KEUKA_REG_CKH1_HI                               0x31
#define KEUKA_REG_CKH_HI                                0x32
#define KEUKA_REG_CKH_NOV                               0x33
#define KEUKA_REG_DE_CKH1                               0x34
#define KEUKA_REG_DE_GOE_RE                             0x35
#define KEUKA_REG_REC_RE_CTL_PM                         0x36
#define KEUKA_REG_DE_CKV                                0x37
#define KEUKA_REG_REC_FE_CTL_PM                         0x38
/* Register, GPO function */
#define KEUKA_REG_GPO                                   0x39
/* Register, Gamma Polarity Toggle Control */
#define KEUKA_REG_DE_VCOM                               0x3B
/* Register, STV */
#define KEUKA_REG_STV_CTL                               0x3C
/* Register, Timing relationship between line start signal
 * and the vertical clock in partial mode
 */
#define KEUKA_REG_DE_CKV_PM                             0x3D
/* Register, VCOM in Partial Mode */
#define KEUKA_REG_DE_VCOM_PM                            0x3E

/* Register, number of columns the output data is shifted on the display */
#define KEUKA_REG_COL_OFFSET                            0x3F
#define KEUKA_REG_OSC_PER_LINE_LSB                      0x40
#define KEUKA_REG_LINES_PER_FRAME_LSB                   0x41
#define KEUKA_REG_OPL_LPF_MSBS                          0x42
#define KEUKA_REG_VDDGR_ADJ                             0x43
#define KEUKA_REG_VSSGR_ADJ                             0x44
#define KEUKA_REG_VCOMH_ADJ                             0x46
#define KEUKA_REG_VCS_ADJ                               0x47
#define KEUKA_REG_V7_0_POS_RED                          0x48
#define KEUKA_REG_V24_7_POS_RED                         0x49
#define KEUKA_REG_V63_56_POS_RED                        0x4A
#define KEUKA_REG_V0_N_63_P_RED                         0x4B
#define KEUKA_REG_V24_7_NEG_RED                         0x4C

/* RED Gamma Curve adjustments */
#define KEUKA_REG_V56_24_NEG_RED                        0x4D
#define KEUKA_REG_GAMMA_V63_NEG_RED                     0x4E

#define KEUKA_REG_GAMMA_CFG1                            0x52
#define KEUKA_REG_BLANK                                 0x55
#define KEUKA_REG_COLOR_IN                              0x62

#define KEUKA_REG_PARTIAL_AREA_SL_LSB                   0x78
#define KEUKA_REG_PARTIAL_AREA_SL_MSB                   0x79
#define KEUKA_REG_PARTIAL_AREA_EL_LSB                   0x7A
#define KEUKA_REG_PARTIAL_AREA_EL_MSB                   0x7B
#define KEUKA_REG_PARTIAL_AREA_SC_LSB                   0x7C
#define KEUKA_REG_PARTIAL_AREA_SC_MSB                   0x7D
#define KEUKA_REG_PARTIAL_AREA_EC_LSB                   0x7E
#define KEUKA_REG_PARTIAL_AREA_EC_MSB                   0x7F

#endif /* ifndef LCD_CONTROLLER_KEUKA_H */
