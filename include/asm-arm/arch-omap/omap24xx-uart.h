/*
 * linux/include/asm-arm/arch-omap24xx/omap24xx-uart.h
 *
 * register definitions for omap24xx uart controller.
 *
 * Copyright (C) 2004-2005 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __OMAP24XX_UART_H__
#define __OMAP24XX_UART_H__

#include <asm/arch/hardware.h>

struct uart_config {
	u8 mode;
	u8 mdr1;		/* Mode registers */
	u8 mdr2;
	u8 dll;			/* divisor registers - generates baudrates */
	u8 dlh;
	u8 mcr;			/* Modem control register */
	u8 efr;			/* Enhanced feature register */
	u8 scr;			/* Scratchpad register */
	u8 tlr;			/* Trigger level register */
	u8 fcr;			/* FIFO control register */
	u8 ier;			/* interrupt enable register */
	u8 lcr;
	u8 rxfll;		/* Received frame length register */
	u8 rxflh;
	u8 txfll;
	u8 txflh;
	u8 acreg;
	u8 eblr;
};

struct uart_setparm {
	u8 lcr;
	u8 reg;
	u8 reg_data;
};	


struct uart_callback {
	/* tx/rx config - DMA/NonDMA mode */
	int mode;
	void (*uart_tx_dma_callback)(int lch, u16 ch_status, void *data);
	void (*uart_rx_dma_callback)(int lch, u16 ch_status, void *data);
	int tx_buf_size;
	int rx_buf_size;
	/* ISR config */
	void (*int_callback) (u8 iir, u8 ssr, void*);
	char *dev_name;
	void *dev;
	int txrx_req_flag;
};	
/* Useful Macros */

/* UART numbers */
#define UART1                                    (0x0)
#define UART2                                    (0x1)
#define UART3                                    (0x2)
#define MAX_UARTS                                UART3


#define BASE_CLK				 (48000000)
#define DISABLE                                  0
#define ENABLE                                   1
#define UART_MODE	                         0
#define IRDA_MODE	                         1
#define UART_AUTOBAUD_MODE                       2
#define CIR_MODE	                         3

#define UART_DMA_MODE	                         0
#define UART_NONDMA_MODE	                 1

#define IR_SIR_SPEED	                          115200
#define IR_MIR_SPEED	                          1152000
#define IR_FIR_SPEED	                          4000000
#define UART_16X_SPEED                             230400
#define UART_13X_SPEED                            3686400

#define SIP_PULSE                                   0x40
#define BOF_1                                       0x01
#define BOF_2                                       0x02
#define BOF_3                                       0x03
/* MDR1 - Defines */
#define UART_16X_MODE                             0x00
#define UART_SIR_MODE                             0x01
#define UART_16XAUTO_MODE                         0x02
#define UART_13X_MODE                             0x03
#define UART_MIR_MODE                             0x04
#define UART_FIR_MODE                             0x05
#define UART_CIR_MODE                             0x06
#define UART_DISABLE	                          0x07
#define MODE_SELECT_MASK                         ~(0x07)
#define FIFO_SIZE				  64
	
#define TXRX					  0
#define RX_ONLY					  1
#define TX_ONLY					  2	

#define LCR_MODE1                                (0<<7)
#define LCR_MODE2                                (1<<7)
#define LCR_MODE3                                (0xbf)

/* Function Templates */
u32 omap24xx_uart_base_v(int uart_no);
u32 omap24xx_uart_base_p(int uart_no);
int omap24xx_uart_release(int uart_no);
int omap24xx_uart_request(int uart_no, struct uart_callback*);
int omap24xx_uart_stop(int uart_no);
int omap24xx_uart_clk_enable(int uart_no);
int omap24xx_uart_clk_disable(int uart_no);
void omap24xx_uart_set_force_idle(void);
void omap24xx_uart_clr_force_idle(void);
int omap24xx_uart_config(int uart_no, struct uart_config *uart_cfg);
int omap24xx_uart_set_speed(int uart_no, int speed);
int omap24xx_uart_get_speed(int uart_no, int *speed);
int omap24xx_uart_set_parms(int uart_no, struct uart_setparm *uart_set);
int omap24xx_uart_get_parms(int uart_no, u8 *param, u8 reg, u8 lcr_mode);
int omap24xx_uart_start_rx(int uart_no, int size);
int omap24xx_uart_start_tx(int uart_no, int size);
int omap24xx_uart_interrupt(int uart_no, int enable);
int omap24xx_uart_reset(int uart_no);
int omap24xx_uart_stop_tx(int uart_no);
int omap24xx_uart_stop_rx(int uart_no);
int omap24xx_uart_rx(int uart_no, void *data, int *len);
int omap24xx_uart_tx(int uart_no, void *data, int size );
int console_detect(char *str);
#ifdef CONFIG_HS_SERIAL_SUPPORT	
void serial8250_unregister_port(int line);
#endif

/****************************************
 *  UARTIRDACIROCP
 ****************************************/

/**** Register Definitions */

#define REG_RHR                                  (0x0)
#define REG_THR                                  (0x0)
#define REG_DLL                                  (0x0)
#define REG_IER                                  (0x4)
#define REG_DLH                                  (0x4)
#define REG_IIR                                  (0x8)
#define REG_FCR                                  (0x8)
#define REG_EFR                                  (0x8)
#define REG_LCR                                  (0xC)
#define REG_MCR                                  (0x10)
#define REG_XON1_ADDR1                           (0x10)
#define REG_LSR                                  (0x14)
#define REG_XON2_ADDR2                           (0x14)
#define REG_TCR                                  (0x18)
#define REG_MSR                                  (0x18)
#define REG_XOFF1                                (0x18)
#define REG_TLR                                  (0x1C)
#define REG_SPR                                  (0x1C)
#define REG_XOFF2                                (0x1C)
#define REG_MDR1                                 (0x20)
#define REG_MDR2                                 (0x24)
#define REG_SFLSR                                (0x28)
#define REG_TXFLL                                (0x28)
#define REG_RESUME                               (0x2C)
#define REG_TXFLH                                (0x2C)
#define REG_SFREGL                               (0x30)
#define REG_RXFLL                                (0x30)
#define REG_SFREGH                               (0x34)
#define REG_RXFLH                                (0x34)
#define REG_BLR                                  (0x38)
#define REG_UASR                                 (0x38)
#define REG_ACREG                                (0x3C)
#define REG_SCR                                  (0x40)
#define REG_SSR                                  (0x44)
#define REG_EBLR                                 (0x48)
#define REG_MVR                                  (0x50)
#define REG_SYSC                                 (0x54)
#define REG_SYSS                                 (0x58)
#define REG_WER                                  (0x5C)
#define REG_CFPS                                 (0x60)

/**** BitField Definitions */

/* DLL Fields */
#define BIT_DLL_CLOCK_LSB                        (0x00)
#define BIT_DLL_CLOCK_LSB_M                      (0xFF)
/* RHR Fields */
#define BIT_RHR_RHR                              (0x00)
#define BIT_RHR_RHR_M                            (0xFF)
/* THR Fields */
#define BIT_THR_THR                              (0x00)
#define BIT_THR_THR_M                            (0xFF)
/* DLH Fields */
#define BIT_DLH_CLOCK_MSB                        (0x00)
#define BIT_DLH_CLOCK_MSB_M                      (0x3F)
/* IER Fields */
#define BIT_IER_RHR_IT                           (0x00)
#define BIT_IER_RHR_IT_M                         (0x01)
#define BIT_IER_THR_IT                           (0x01)
#define BIT_IER_THR_IT_M                         (0x02)
#define BIT_IER_LINE_STS_IT_U                    (0x02)
#define BIT_IER_LINE_STS_IT_U_M                  (0x04)
#define BIT_IER_LAST_RX_BYTE_IT                  (0x02)
#define BIT_IER_LAST_RX_BYTE_IT_M                (0x04)
#define BIT_IER_RX_STOP_IT                       (0x02)
#define BIT_IER_RX_STOP_IT_M                     (0x04)
#define BIT_IER_MODEM_STS_IT                     (0x03)
#define BIT_IER_MODEM_STS_IT_M                   (0x08)
#define BIT_IER_RX_OVERRUN_IT                    (0x03)
#define BIT_IER_RX_OVERRUN_IT_M                  (0x08)
#define BIT_IER_SLEEP_MODE                       (0x04)
#define BIT_IER_SLEEP_MODE_M                     (0x10)
#define BIT_IER_STS_FIFO_TRIG_IT                 (0x04)
#define BIT_IER_STS_FIFO_TRIG_IT_M               (0x10)
#define BIT_IER_XOFF_IT                          (0x05)
#define BIT_IER_XOFF_IT_M                        (0x20)
#define BIT_IER_TX_STATUS_IT                     (0x05)
#define BIT_IER_TX_STATUS_IT_M                   (0x20)
#define BIT_IER_RTS_IT                           (0x06)
#define BIT_IER_RTS_IT_M                         (0x40)
#define BIT_IER_LINE_STS_IT_I                    (0x06)
#define BIT_IER_LINE_STS_IT_I_M                  (0x40)
#define BIT_IER_CTS_IT                           (0x07)
#define BIT_IER_CTS_IT_M                         (0x80)
#define BIT_IER_EOF_IT                           (0x07)
#define BIT_IER_EOF_IT_M                         (0x80)
/* FCR Fields */
#define BIT_FCR_FIFO_EN                          (0x00)
#define BIT_FCR_FIFO_EN_M                        (0x01)
/* IIR Fields */
#define BIT_IIR_IT_PENDING                       (0x00)
#define BIT_IIR_IT_PENDING_M                     (0x01)
#define BIT_IIR_RHR_IT                           (0x00)
#define BIT_IIR_RHR_IT_M                         (0x01)
/* EFR Fields */
#define BIT_EFR_SW_FLOW_CONTROL                  (0x00)
#define BIT_EFR_SW_FLOW_CONTROL_M                (0x0F)
/* FCR Fields */
#define BIT_FCR_RX_FIFO_CLEAR                    (0x01)
#define BIT_FCR_RX_FIFO_CLEAR_M                  (0x02)
/* IIR Fields */
#define BIT_IIR_IT_TYPE                          (0x01)
#define BIT_IIR_IT_TYPE_M                        (0x3E)
#define BIT_IIR_THR_IT                           (0x01)
#define BIT_IIR_THR_IT_M                         (0x02)
/* FCR Fields */
#define BIT_FCR_TX_FIFO_CLEAR                    (0x02)
#define BIT_FCR_TX_FIFO_CLEAR_M                  (0x04)
/* IIR Fields */
#define BIT_IIR_RX_FIFO_LAST_BYTE_IT             (0x02)
#define BIT_IIR_RX_FIFO_LAST_BYTE_IT_M           (0x04)
#define BIT_IIR_RX_STOP_IT                       (0x02)
#define BIT_IIR_RX_STOP_IT_M                     (0x04)
/* FCR Fields */
#define BIT_FCR_DMA_MODE                         (0x03)
#define BIT_FCR_DMA_MODE_M                       (0x08)
#define FCR_NO_DMA				 ~(0x08)
/* IIR Fields */
#define BIT_IIR_RX_OE_IT                         (0x03)
#define BIT_IIR_RX_OE_IT_M                       (0x08)
/* FCR Fields */
#define BIT_FCR_TX_FIFO_TRIG                     (0x04)
#define BIT_FCR_TX_FIFO_TRIG_M                   (0x30)
#define FCR_TX_FIFO_TRIG_8                       (0x00)
#define FCR_TX_FIFO_TRIG_16                      (0x10)
#define FCR_TX_FIFO_TRIG_32                      (0x20)
#define FCR_TX_FIFO_TRIG_56                      (0x30)
/* IIR Fields */
#define BIT_IIR_STS_FIFO_IT                      (0x04)
#define BIT_IIR_STS_FIFO_IT_M                    (0x10)
/* EFR Fields */
#define BIT_EFR_ENHANCED_EN                      (0x04)
#define BIT_EFR_ENHANCED_EN_M                    (0x10)
/* IIR Fields */
#define BIT_IIR_TX_STATUS_IT                     (0x05)
#define BIT_IIR_TX_STATUS_IT_M                   (0x20)
/* EFR Fields */
#define BIT_EFR_SPECIAL_CHAR_DETECT              (0x05)
#define BIT_EFR_SPECIAL_CHAR_DETECT_M            (0x20)
/* FCR Fields */
#define BIT_FCR_RX_FIFO_TRIG                     (0x06)
#define BIT_FCR_RX_FIFO_TRIG_M                   (0xC0)
#define FCR_RX_FIFO_TRIG_8                       (0x00)
#define FCR_RX_FIFO_TRIG_16                      (0x40)
#define FCR_RX_FIFO_TRIG_56                      (0x80)
#define FCR_RX_FIFO_TRIG_60                      (0xC0)
/* IIR Fields */
#define BIT_IIR_FCR_MIRROR                       (0x06)
#define BIT_IIR_FCR_MIRROR_M                     (0xC0)
#define BIT_IIR_LINE_STS_IT                      (0x06)
#define BIT_IIR_LINE_STS_IT_M                    (0x40)
/* EFR Fields */
#define BIT_EFR_AUTO_RTS_EN                      (0x06)
#define BIT_EFR_AUTO_RTS_EN_M                    (0x40)
/* IIR Fields */
#define BIT_IIR_EOF_IT                           (0x07)
#define BIT_IIR_EOF_IT_M                         (0x80)
/* EFR Fields */
#define BIT_EFR_AUTO_CTS_EN                      (0x07)
#define BIT_EFR_AUTO_CTS_EN_M                    (0x80)

/* LCR Fields */
#define BIT_LCR_CHAR_LENGTH                      (0x00)
#define BIT_LCR_CHAR_LENGTH_M                    (0x03)
#define BIT_LCR_NB_STOP                          (0x02)
#define BIT_LCR_NB_STOP_M                        (0x04)
#define BIT_LCR_PARITY_EN                        (0x03)
#define BIT_LCR_PARITY_EN_M                      (0x08)
#define BIT_LCR_PARITY_TYPE1                     (0x04)
#define BIT_LCR_PARITY_TYPE1_M                   (0x10)
#define BIT_LCR_PARITY_TYPE2                     (0x05)
#define BIT_LCR_PARITY_TYPE2_M                   (0x20)
#define BIT_LCR_BREAK_EN                         (0x06)
#define BIT_LCR_BREAK_EN_M                       (0x40)
#define BIT_LCR_DIV_EN                           (0x07)
#define BIT_LCR_DIV_EN_M                         (0x80)
/* XON1_ADDR1 Fields */
#define BIT_XON1_ADDR1_XON_WORD1                 (0x00)
#define BIT_XON1_ADDR1_XON_WORD1_M               (0xFF)
/* MCR Fields */
#define BIT_MCR_DTR                              (0x00)
#define BIT_MCR_DTR_M                            (0x01)
#define BIT_MCR_RTS                              (0x01)
#define BIT_MCR_RTS_M                            (0x02)
#define BIT_MCR_RI_STS_CH                        (0x02)
#define BIT_MCR_RI_STS_CH_M                      (0x04)
#define BIT_MCR_CD_STS_CH                        (0x03)
#define BIT_MCR_CD_STS_CH_M                      (0x08)
#define BIT_MCR_LOOPBACK_EN                      (0x04)
#define BIT_MCR_LOOPBACK_EN_M                    (0x10)
#define BIT_MCR_XON_EN                           (0x05)
#define BIT_MCR_XON_EN_M                         (0x20)
#define BIT_MCR_TCR_TLR                          (0x06)
#define BIT_MCR_TCR_TLR_M                        (0x40)
#define ENABLE_TCR_TLR                           BIT_MCR_TCR_TLR_M
#define DISABLE_TCR_TLR                          (0x00)
/* LSR Fields */
#define BIT_LSR_RX_FIFO_E                        (0x00)
#define BIT_LSR_RX_FIFO_E_M                      (0x01)
/* XON2_ADDR2 Fields */
#define BIT_XON2_ADDR2_XON_WORD2                 (0x00)
#define BIT_XON2_ADDR2_XON_WORD2_M               (0xFF)
/* LSR Fields */
#define BIT_LSR_RX_OE                            (0x01)
#define BIT_LSR_RX_OE_M                          (0x02)
#define BIT_LSR_STS_FIFO_E                       (0x01)
#define BIT_LSR_STS_FIFO_E_M                     (0x02)
#define BIT_LSR_RX_PE                            (0x02)
#define BIT_LSR_RX_PE_M                          (0x04)
#define BIT_LSR_CRC                              (0x02)
#define BIT_LSR_CRC_M                            (0x04)
#define BIT_LSR_RX_FE                            (0x03)
#define BIT_LSR_RX_FE_M                          (0x08)
#define BIT_LSR_ABORT                            (0x03)
#define BIT_LSR_ABORT_M                          (0x08)
#define BIT_LSR_RX_BI                            (0x04)
#define BIT_LSR_RX_BI_M                          (0x10)
#define BIT_LSR_FRAME_TOO_LONG                   (0x04)
#define BIT_LSR_FRAME_TOO_LONG_M                 (0x10)
#define BIT_LSR_TX_FIFO_E                        (0x05)
#define BIT_LSR_TX_FIFO_E_M                      (0x20)
#define BIT_LSR_RX_LAST_BYTE                     (0x05)
#define BIT_LSR_RX_LAST_BYTE_M                   (0x20)
#define BIT_LSR_RX_STOP                          (0x05)
#define BIT_LSR_RX_STOP_M                        (0x20)
#define BIT_LSR_TX_SR_E                          (0x06)
#define BIT_LSR_TX_SR_E_M                        (0x40)
#define BIT_LSR_STS_FIFO_FULL                    (0x06)
#define BIT_LSR_STS_FIFO_FULL_M                  (0x40)
#define BIT_LSR_RX_FIFO_STS                      (0x07)
#define BIT_LSR_RX_FIFO_STS_M                    (0x80)
#define BIT_LSR_THR_EMPTY                        (0x07)
#define BIT_LSR_THR_EMPTY_M                      (0x80)
/* TCR Fields */
#define BIT_TCR_RX_FIFO_TRIG_HALT                (0x00)
#define BIT_TCR_RX_FIFO_TRIG_HALT_M              (0x0F)
/* MSR Fields */
#define BIT_MSR_CTS_STS                          (0x00)
#define BIT_MSR_CTS_STS_M                        (0x01)
/* XOFF1 Fields */
#define BIT_XOFF1_XOFF_WORD1                     (0x00)
#define BIT_XOFF1_XOFF_WORD1_M                   (0xFF)
/* MSR Fields */
#define BIT_MSR_DSR_STS                          (0x01)
#define BIT_MSR_DSR_STS_M                        (0x02)
#define BIT_MSR_RI_STS                           (0x02)
#define BIT_MSR_RI_STS_M                         (0x04)
#define BIT_MSR_DCD_STS                          (0x03)
#define BIT_MSR_DCD_STS_M                        (0x08)
/* TCR Fields */
#define BIT_TCR_RX_FIFO_TRIG_START               (0x04)
#define BIT_TCR_RX_FIFO_TRIG_START_M             (0xF0)
/* MSR Fields */
#define BIT_MSR_NCTS_STS                         (0x04)
#define BIT_MSR_NCTS_STS_M                       (0x10)
#define BIT_MSR_NDSR_STS                         (0x05)
#define BIT_MSR_NDSR_STS_M                       (0x20)
#define BIT_MSR_NRI_STS                          (0x06)
#define BIT_MSR_NRI_STS_M                        (0x40)
#define BIT_MSR_NCD_STS                          (0x07)
#define BIT_MSR_NCD_STS_M                        (0x80)
/* XOFF2 Fields */
#define BIT_XOFF2_XOFF_WORD2                     (0x00)
#define BIT_XOFF2_XOFF_WORD2_M                   (0xFF)
/* SPR Fields */
#define BIT_SPR_SPR_WORD                         (0x00)
#define BIT_SPR_SPR_WORD_M                       (0xFF)
/* TLR Fields */
#define BIT_TLR_TX_FIFO_TRIG_DMA                 (0x00)
#define BIT_TLR_TX_FIFO_TRIG_DMA_M               (0x0F)
#define BIT_TLR_RX_FIFO_TRIG_DMA                 (0x04)
#define BIT_TLR_RX_FIFO_TRIG_DMA_M               (0xF0)
#define TLR_FCR_TXRX_CONFIG			 (0x00)
/* MDR1 Fields */
#define BIT_MDR1_MODE_SELECT                     (0x00)
#define BIT_MDR1_MODE_SELECT_M                   (0x07)
#define BIT_MDR1_IR_SLEEP                        (0x03)
#define BIT_MDR1_IR_SLEEP_M                      (0x08)
#define BIT_MDR1_SET_TXIR                        (0x04)
#define BIT_MDR1_SET_TXIR_M                      (0x10)
#define BIT_MDR1_SCT                             (0x05)
#define BIT_MDR1_SCT_M                           (0x20)
#define BIT_MDR1_SIP_MODE                        (0x06)
#define BIT_MDR1_SIP_MODE_M                      (0x40)
#define BIT_MDR1_FRAME_END_MODE                  (0x07)
#define BIT_MDR1_FRAME_END_MODE_M                (0x80)
/* MDR2 Fields */
#define BIT_MDR2_IRTX_UNDERRUN                   (0x00)
#define BIT_MDR2_IRTX_UNDERRUN_M                 (0x01)
#define BIT_MDR2_STS_FIFO_TRIG                   (0x01)
#define BIT_MDR2_STS_FIFO_TRIG_M                 (0x06)
#define BIT_MDR2_UART_PULSE                      (0x03)
#define BIT_MDR2_UART_PULSE_M                    (0x08)
#define BIT_MDR2_CIR_PULSE_MODE                  (0x04)
#define BIT_MDR2_CIR_PULSE_MODE_M                (0x30)
#define BIT_MDR2_IRRXINVERT                      (0x06)
#define BIT_MDR2_IRRXINVERT_M                    (0x40)
#define MDR2_STS_FIFO_TRIG_1			 (0x00)
/* TXFLL Fields */
#define BIT_TXFLL_TXFLL                          (0x00)
#define BIT_TXFLL_TXFLL_M                        (0xFF)
/* SFLSR Fields */
#define BIT_SFLSR_CRC_ERROR                      (0x01)
#define BIT_SFLSR_CRC_ERROR_M                    (0x02)
#define BIT_SFLSR_ABORT_DETECT                   (0x02)
#define BIT_SFLSR_ABORT_DETECT_M                 (0x04)
#define BIT_SFLSR_FRAME_TOO_LONG_ERROR           (0x03)
#define BIT_SFLSR_FRAME_TOO_LONG_ERROR_M         (0x08)
#define BIT_SFLSR_OE_ERROR                       (0x04)
#define BIT_SFLSR_OE_ERROR_M                     (0x10)
/* TXFLH Fields */
#define BIT_TXFLH_TXFLH                          (0x00)
#define BIT_TXFLH_TXFLH_M                        (0x1F)
/* RESUME Fields */
#define BIT_RESUME_RESUME                        (0x00)
#define BIT_RESUME_RESUME_M                      (0xFF)
/* RXFLL Fields */
#define BIT_RXFLL_RXFLL                          (0x00)
#define BIT_RXFLL_RXFLL_M                        (0xFF)
/* SFREGL Fields */
#define BIT_SFREGL_SFREGL                        (0x00)
#define BIT_SFREGL_SFREGL_M                      (0xFF)
/* RXFLH Fields */
#define BIT_RXFLH_RXFLH                          (0x00)
#define BIT_RXFLH_RXFLH_M                        (0x0F)
#define RXFLH_2048				 (0x08)
/* SFREGH Fields */
#define BIT_SFREGH_SFREGH                        (0x00)
#define BIT_SFREGH_SFREGH_M                      (0x0F)
/* UASR Fields */
#define BIT_UASR_SPEED                           (0x00)
#define BIT_UASR_SPEED_M                         (0x1F)
#define BIT_UASR_BIT_BY_CHAR                     (0x05)
#define BIT_UASR_BIT_BY_CHAR_M                   (0x20)
#define BIT_UASR_PARITY_TYPE                     (0x06)
#define BIT_UASR_PARITY_TYPE_M                   (0xC0)
/* BLR Fields */
#define BIT_BLR_XBOF_TYPE                        (0x06)
#define BIT_BLR_XBOF_TYPE_M                      (0x40)
#define BIT_BLR_STS_FIFO_RESET                   (0x07)
#define BIT_BLR_STS_FIFO_RESET_M                 (0x80)
/* ACREG Fields */
#define BIT_ACREG_EOT_EN                         (0x00)
#define BIT_ACREG_EOT_EN_M                       (0x01)
#define BIT_ACREG_ABORT_EN                       (0x01)
#define BIT_ACREG_ABORT_EN_M                     (0x02)
#define BIT_ACREG_SCTX_EN                        (0x02)
#define BIT_ACREG_SCTX_EN_M                      (0x04)
#define BIT_ACREG_SEND_SIP                       (0x03)
#define BIT_ACREG_SEND_SIP_M                     (0x08)
#define BIT_ACREG_DIS_TX_UNDERRUN                (0x04)
#define BIT_ACREG_DIS_TX_UNDERRUN_M              (0x10)
#define BIT_ACREG_DIS_IR_RX                      (0x05)
#define BIT_ACREG_DIS_IR_RX_M                    (0x20)
#define BIT_ACREG_SD_MOD                         (0x06)
#define BIT_ACREG_SD_MOD_M                       (0x40)
#define BIT_ACREG_PULSE_TYPE                     (0x07)
#define BIT_ACREG_PULSE_TYPE_M                   (0x80)
/* SCR Fields */
#define BIT_SCR_DMA_MODE_CTL                     (0x00)
#define BIT_SCR_DMA_MODE_CTL_M                   (0x01)
#define BIT_SCR_DMA_MODE_2                       (0x01)
#define BIT_SCR_DMA_MODE_2_M                     (0x06)
#define BIT_SCR_TX_EMPTY_CTL_IT                  (0x03)
#define BIT_SCR_TX_EMPTY_CTL_IT_M                (0x08)
#define BIT_SCR_RX_CTS_DSR_WAKE_UP_ENABLE        (0x04)
#define BIT_SCR_RX_CTS_DSR_WAKE_UP_ENABLE_M      (0x10)
#define BIT_SCR_DSR_IT                           (0x05)
#define BIT_SCR_DSR_IT_M                         (0x20)
#define BIT_SCR_TX_TRIG_GRANU1                   (0x06)
#define BIT_SCR_TX_TRIG_GRANU1_M                 (0x40)
#define BIT_SCR_RX_TRIG_GRANU1                   (0x07)
#define BIT_SCR_RX_TRIG_GRANU1_M                 (0x80)
/* SSR Fields */
#define BIT_SSR_TX_FIFO_FULL                     (0x00)
#define BIT_SSR_TX_FIFO_FULL_M                   (0x01)
#define BIT_SSR_RX_CTS_DSR_WAKE_UP_STS           (0x01)
#define BIT_SSR_RX_CTS_DSR_WAKE_UP_STS_M         (0x02)
/* EBLR Fields */
#define BIT_EBLR_EBLR                            (0x00)
#define BIT_EBLR_EBLR_M                          (0xFF)
/* MVR Fields */
#define BIT_MVR_MINOR_REV                        (0x00)
#define BIT_MVR_MINOR_REV_M                      (0x0F)
#define BIT_MVR_MAJOR_REV                        (0x04)
#define BIT_MVR_MAJOR_REV_M                      (0xF0)
/* SYSC Fields */
#define BIT_SYSC_AUTOIDLE                        (0x00)
#define BIT_SYSC_AUTOIDLE_M                      (0x01)
#define BIT_SYSC_SOFTRESET                       (0x01)
#define BIT_SYSC_SOFTRESET_M                     (0x02)
#define BIT_SYSC_ENAWAKEUP                       (0x02)
#define BIT_SYSC_ENAWAKEUP_M                     (0x04)
#define BIT_SYSC_IDLEMODE                        (0x03)
#define BIT_SYSC_IDLEMODE_M                      (0x18)
/* SYSS Fields */
#define BIT_SYSS_RESETDONE                       (0x00)
#define BIT_SYSS_RESETDONE_M                     (0x01)
/* WER Fields */
#define BIT_WER_EVENT_0_CTS_ACTIVITY             (0x00)
#define BIT_WER_EVENT_0_CTS_ACTIVITY_M           (0x01)
#define BIT_WER_EVENT_1_DSR_ACTIVITY             (0x01)
#define BIT_WER_EVENT_1_DSR_ACTIVITY_M           (0x02)
#define BIT_WER_EVENT_2_RI_ACTIVITY              (0x02)
#define BIT_WER_EVENT_2_RI_ACTIVITY_M            (0x04)
#define BIT_WER_EVENT_3_DCD_CD_ACTIVITY          (0x03)
#define BIT_WER_EVENT_3_DCD_CD_ACTIVITY_M        (0x08)
#define BIT_WER_EVENT_4_RX_ACTIVITY              (0x04)
#define BIT_WER_EVENT_4_RX_ACTIVITY_M            (0x10)
#define BIT_WER_EVENT_5_RHR_INTERRUPT            (0x05)
#define BIT_WER_EVENT_5_RHR_INTERRUPT_M          (0x20)
#define BIT_WER_EVENT_6_RECEIVER_LINE_STATUS_INTERRUPT (0x06)
#define BIT_WER_EVENT_6_RECEIVER_LINE_STATUS_INTERRUPT_M (0x40)
/* CFPS Fields */
#define BIT_CFPS_CFPS                            (0x00)
#define BIT_CFPS_CFPS_M                          (0xFF)

/* Library specific defines */
#define LCR_MODE1                                (0<<7)
#define LCR_MODE2                                (1<<7)
#define LCR_MODE3                                (0xbf)


#endif				/* End of __OMAP24XX-UART_H__ */
