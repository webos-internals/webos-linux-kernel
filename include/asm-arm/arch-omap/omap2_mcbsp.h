/*
 * include/asm-arm/arch-omap24xx/omap2_mcbsp.h
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */
#ifndef __ASM_ARCH_OMAP2_McBSP_H
#define __ASM_ARCH_OMAP2_McBSP_H

/* Global definitions */

#if defined(CONFIG_ARCH_OMAP2430) || defined(CONFIG_ARCH_OMAP3430) 

#define OMAP2_MAX_MCBSP_COUNT		5
#define OMAP2_MCBSP_INTERFACE3		2
#define OMAP2_MCBSP_INTERFACE4		3
#define OMAP2_MCBSP_INTERFACE5		4

#else
#define OMAP2_MAX_MCBSP_COUNT		2
#endif

/* McBSP interface ids */
#define OMAP2_MCBSP_INTERFACE1		0
#define OMAP2_MCBSP_INTERFACE2		1

/* Word Length */
#define OMAP2_MCBSP_WORDLEN_NONE        255
#define OMAP2_MCBSP_WORDLEN_8		0
#define OMAP2_MCBSP_WORDLEN_12		1
#define OMAP2_MCBSP_WORDLEN_16		2
#define OMAP2_MCBSP_WORDLEN_20		3
#define OMAP2_MCBSP_WORDLEN_24		4
#define OMAP2_MCBSP_WORDLEN_32		5

/* Word Length */
#define OMAP2_MCBSP_FRAMELEN_1		0
#define OMAP2_MCBSP_FRAMELEN_2		1
#define OMAP2_MCBSP_FRAMELEN_3		2
#define OMAP2_MCBSP_FRAMELEN_4 		3
#define OMAP2_MCBSP_FRAMELEN_5 		4
#define OMAP2_MCBSP_FRAMELEN_6 		5
#define OMAP2_MCBSP_FRAMELEN_N(NUM_WORDS) ((NUM_WORDS - 1) & 0x7F)

/* Data Delay */
#define OMAP2_MCBSP_DATADELAY0		0
#define OMAP2_MCBSP_DATADELAY1		1
#define OMAP2_MCBSP_DATADELAY2		2

/* Reverse mode 243x */
#define OMAP2_MCBSP_MSBFIRST		0
#define OMAP2_MCBSP_LSBFIRST		1
/* Compand mode 242x */
#define OMAP2_MCBSP_COMPAND_NONE_MSB    0
#define OMAP2_MCBSP_COMPAND_NONE_8_LSB  1
#define OMAP2_MCBSP_COMPAND_U_LAW       2
#define OMAP2_MCBSP_COMPAND_A_LAW       3

/* Frame Ignore 242x */
#define OMAP2_MCBSP_FIG_NONE            0
#define OMAP2_MCBSP_FIG_IGNORE          1

/* Justification */
#define OMAP2_MCBSP_RJUST_ZEROMSB	0
#define OMAP2_MCBSP_RJUST_SIGNMSB	1
#define OMAP2_MCBSP_LJUST_ZEROLSB	2

/* Clock Mode */
#define OMAP2_MCBSP_CLK_RISING		0
#define OMAP2_MCBSP_CLK_FALLING		1

/* Frame Sync Polarity */
#define OMAP2_MCBSP_FS_ACTIVE_HIGH	0
#define OMAP2_MCBSP_FS_ACTIVE_LOW	1

/* Frame Sync Source */
#define OMAP2_MCBSP_TXFSYNC_EXTERNAL	0
#define OMAP2_MCBSP_TXFSYNC_INTERNAL	1

#define OMAP2_MCBSP_RXFSYNC_EXTERNAL	0
#define OMAP2_MCBSP_RXFSYNC_INTERNAL	1

/* McBSP interface operating mode */
#define OMAP2_MCBSP_MASTER		1
#define OMAP2_MCBSP_SLAVE		0

/* SRG ENABLE/DISABLE */
#define OMAP2_MCBSP_SRG_DISABLE		0
#define OMAP2_MCBSP_SRG_ENABLE		1

/* FSG ENABLE/DISABLE */
#define OMAP2_MCBSP_FSG_DISABLE		0
#define OMAP2_MCBSP_FSG_ENABLE		1

/* RRST STATE */
#define OMAP2_MCBSP_RRST_DISABLE	0
#define OMAP2_MCBSP_RRST_ENABLE		1

/*XRST STATE */
#define OMAP2_MCBSP_XRST_DISABLE	0
#define OMAP2_MCBSP_XRST_ENABLE		1
/*RXEN STATE */
#define OMAP2_MCBSP_RX_DISABLE		0
#define OMAP2_MCBSP_RX_ENABLE		1
/*TXEN STATE */
#define OMAP2_MCBSP_TX_DISABLE		0
#define OMAP2_MCBSP_TX_ENABLE		1

/* Global behaviour of data rx/tx mode */
#define OMAP2_MCBSP_ALB			0 /* Analog Loop back mode  */
#define OMAP2_MCBSP_DLB			1 /* Digital Loop back mode */
#define OMAP2_MCBSP_SYNCTXRX		2 /* Synchronous Tx Rx mode */

/* Rx Multichannel selection */
#define OMAP2_MCBSP_RXMUTICH_DISABLE	0 /* Multichannel Disable */
#define OMAP2_MCBSP_RXMUTICH_ENABLE	1 /* Multichannel Enable */

/* Tx Multichannel selection */
#define OMAP2_MCBSP_TXMUTICH_DISABLE    0 /* Multichannel Disable */
#define OMAP2_MCBSP_TXMUTICH_ENABLE     1 /* Multichannel Enable */

#define OMAP2_MCBSP_TWOPARTITION_MODE		0
#define OMAP2_MCBSP_EIGHTPARTITION_MODE		1

#define OMAP2_MCBSP_MULTICH_BLK0		0
#define OMAP2_MCBSP_MULTICH_BLK1		1
#define OMAP2_MCBSP_MULTICH_BLK2		2
#define OMAP2_MCBSP_MULTICH_BLK3		3
#define OMAP2_MCBSP_MULTICH_BLK4		4
#define OMAP2_MCBSP_MULTICH_BLK5		5
#define OMAP2_MCBSP_MULTICH_BLK6		6
#define OMAP2_MCBSP_MULTICH_BLK7		7

#define OMAP2_MCBSP_FCLKSRC_PRCM		1
#define OMAP2_MCBSP_FCLKSRC_EXTERNAL		2

/* Sample Rate Generator Clock source */
#define OMAP2_MCBSP_SRGCLKSRC_CLKS		1
#define OMAP2_MCBSP_SRGCLKSRC_FCLK		2
#define OMAP2_MCBSP_SRGCLKSRC_CLKR		3
#define OMAP2_MCBSP_SRGCLKSRC_CLKX		4

/* SRG Clock synchronization mode */
#define OMAP2_MCBSP_SRG_FREERUNNING		1
#define OMAP2_MCBSP_SRG_RUNNING			2

/* SRG input clock polarity */
#define OMAP2_MCBSP_CLKS_POLARITY_RISING	1
#define OMAP2_MCBSP_CLKS_POLARITY_FALLING	2

#define OMAP2_MCBSP_CLKX_POLARITY_RISING	1
#define OMAP2_MCBSP_CLKX_POLARITY_FALLING	2

#define OMAP2_MCBSP_CLKR_POLARITY_RISING	1
#define OMAP2_MCBSP_CLKR_POLARITY_FALLING	2

#define OMAP2_MCBSP_CLKRXSRC_EXTERNAL		1
#define OMAP2_MCBSP_CLKRXSRC_INTERNAL		2

#define OMAP2_MCBSP_CLKTXSRC_EXTERNAL		1
#define OMAP2_MCBSP_CLKTXSRC_INTERNAL		2

#define OMAP2_MCBSP_FRAME_SINGLEPHASE		1
#define OMAP2_MCBSP_FRAME_DUALPHASE		2

#define OMAP2_MCBSP_DXEN_ENABLE			1
#define OMAP2_MCBSP_DXEN_DISABLE		0

#define OMAP2_MCBSP_IRQEN_XOVFL                 (1<<12)
#define OMAP2_MCBSP_IRQEN_XUNDFL                (1<<11)
#define OMAP2_MCBSP_IRQEN_ROVFL                 (1<<5)
#define OMAP2_MCBSP_IRQEN_RUNDFL                (1<<4)

#define OMAP2_MCBSP_IRQSTAT_XOVFL               (1<<12)
#define OMAP2_MCBSP_IRQSTAT_XUNDFL              (1<<11)
#define OMAP2_MCBSP_IRQSTAT_XRDY                (1<<10)
#define OMAP2_MCBSP_IRQSTAT_XEOF                (1<<9)
#define OMAP2_MCBSP_IRQSTAT_XFSX                (1<<8)
#define OMAP2_MCBSP_IRQSTAT_XSYNCERR            (1<<7)
#define OMAP2_MCBSP_IRQSTAT_ROVFL               (1<<5)
#define OMAP2_MCBSP_IRQSTAT_RUNDFL              (1<<4)
#define OMAP2_MCBSP_IRQSTAT_RRDY                (1<<3)
#define OMAP2_MCBSP_IRQSTAT_REOF                (1<<2)
#define OMAP2_MCBSP_IRQSTAT_RFSX                (1<<1)
#define OMAP2_MCBSP_IRQSTAT_RSYNCERR            (1<<0)

/* Interrupt Handling */
typedef void (*omap2_mcbsp_isr_t) (void *arg, u32 irq_status);
	
typedef void (*omap2_mcbsp_dma_cb) (u16 ch_status, void *arg);

#define OMAP2_MCBSP_AUTO_RST_NONE               (0x0)
#define OMAP2_MCBSP_AUTO_RRST                   (0x1<<1)
#define OMAP2_MCBSP_AUTO_XRST                   (0x1<<2)

#define OMAP2_MCBSP_SKIP_NONE                   (0x0)
#define OMAP2_MCBSP_SKIP_FIRST                  (0x1<<1)
#define OMAP2_MCBSP_SKIP_SECOND                 (0x1<<2)
#define OMAP2_MCBSP_SKIP_ADD                    (0x1<<3)

/* Structure defnitions */
typedef struct omap2_mcbsp_transfer_parameters
{
	/* Data type -if we want to skip auto decision */
	u32 data_type;
	/* Skip the alternate element */
	u8 skip_alt;

	/* Automagically handle Transfer [XR]RST? */
	u8   auto_reset; 

	/* phase singe/dual */
	u8   phase; 
	
	/* data delay */
	u8   data_delay;
	
	/* Reverse Enable /Disable 
	 * reverse in 243x
	 * compand in 242x
	 */
	u8   reverse_compand;

	/* Frame Ignore -242x only, ignored in 243x */
	u8   fig;
	
	/* Word length */
	u32  word_length1; 
	u32  word_length2; 
	
	/* Frame length */
	u32  frame_length1; 
	u32  frame_length2; 
	
	/* Justification RJUST*/
	u32  justification;
		
	/* Transmitter can be configured for dxend delay */
	u32  dxena;   /* enable the dxena */
	u32  dxendly; /* delay */
	
	/* callback function executed for every tx/rx completion */
	omap2_mcbsp_dma_cb callback;
	
}omap2_mcbsp_transfer_params;

/*
 * Reset and Enable/Disable Sample Rate Generator 
 */
int omap2_mcbsp_set_srg(u32 mcbsp_id,u8 state);

/*
 * Reset and Enable/Disable Frame Sync Generator 
 */
int omap2_mcbsp_set_fsg(u32 mcbsp_id,u8 state);

/*
 * Enable/Disable (Reset) Transmitter  
 */
int omap2_mcbsp_set_xrst(u32 mcbsp_id,u8 state);

/*
 * Enable/Disable (Reset) Receiver  
 */
int omap2_mcbsp_set_rrst(u32 mcbsp_id,u8 state);

/*
 * Basic enable/disable Transmitter  
 */
int omap2_mcbsp_set_xen(u32 mcbsp_id, u8 state);

/*
 * Basic enable/disable Reciever  
 */
int omap2_mcbsp_set_ren(u32 mcbsp_id, u8 state);

/*
 * Enable/Disable  Transmitter  
 */
int omap2_mcbsp_set_xen(u32 mcbsp_id,u8 state);

/*
 * Enable/Disable  Receiver  
 */
int omap2_mcbsp_set_ren(u32 mcbsp_id,u8 state);

/*
 * Interface Reset
 */
int omap2_mcbsp_interface_reset(u32 mcbsp_id);

/*
 * Auto idle enable
 */
int omap2_mcbsp_autoidle_enable(u32 mcbsp_id);

/*
 * Auto idle disable
 */
int omap2_mcbsp_autoidle_disable(u32 mcbsp_id);

/*
 * Suspend, support for Client driver's power managment 
 * Stops the clocks (interface and functional), 
 * Interrupts are disabled
 * Note: Data transmission can be stopped using the stop API's
 */
int omap2_mcbsp_suspend(u32 mcbsp_id);

/*
 * Resume ,support for Client driver's power managment
 * Starts the clocks (interface and functional), Interrupts are enabled
 */
int omap2_mcbsp_resume(u32 mcbsp_id);

/*
 * Enable/Disable the Global data rx/tx behaviour
 */
int omap2_mcbsp_datatxrx_mode(u32 mcbsp_id,u32 mode);

/*
 * Receive multichannel selection 
 */
int omap2_mcbsp_rxmultich_enable(u32 mcbsp_id,u8 state);

/*
 * Transmit multichannel selection
 */
int omap2_mcbsp_txmultich_enable(u32 mcbsp_id,u32 state);

/*
 * Transmit multichannel configuration
 */
int omap2_mcbsp_txmultich_cfg(u32 mcbsp_id, u8 part_mode,
			u8 parta_enable, u8 partb_enable, u32 ch_enable);
/*
 * Receive multichannel configuration
 */						
int omap2_mcbsp_rxmultich_cfg(u32 mcbsp_id, u8 part_mode,
			u8 parta_enable, u8 partb_enable, u32 ch_enable);
						
/*
 * Configure the Frame Sync Generator
 */
int
omap2_mcbsp_fsync_cfg(u32 mcbsp_id,u8 tx_fsync_src,u8 rx_fsync_src, u8 tx_polarity, u8 rx_polarity,
							u32 period, u32 pulse_width, u8 fsgm);
/*
 * Configure the receiver clock
 */
int omap2_mcbsp_rxclk_cfg(u32 mcbsp_id,u8 clk_mode,u8 polarity);

/*
 * Configure the transmitter clock
 */
int omap2_mcbsp_txclk_cfg(u32 mcbsp_id,u8 clk_mode,u8 polarity);

/*
 * Configure the Sample rate and data clock CLKG
 */
int omap2_mcbsp_srg_cfg(u32 mcbsp_id, u32 sample_rate, u32 bits_per_sample, 
				u32 srg_src, u32 clk_rate, u8 sync_mode, u8 polarity);

/*
 * Configure the receive parameters
 */
int omap2_mcbsp_set_recv_params(u32 mcbsp_id, omap2_mcbsp_transfer_params *rp);

/*
 * Configure the transfer parameters
 */
int omap2_mcbsp_set_trans_params(u32 mcbsp_id , omap2_mcbsp_transfer_params *tp);

/*
 * Start receving data on a McBSP interface
 */
int omap2_mcbsp_receive_data(u32 mcbsp_id,void *cbdata, dma_addr_t buf_start_addr, u32 buf_size);

/*
 * Start transmitting data through a McBSP interface
 */
int omap2_mcbsp_send_data(u32 mcbsp_id,void *cbdata, dma_addr_t buf_start_addr, u32 buf_size);

/*
 * Stop transmitting data on a McBSP interface
 */
int omap2_mcbsp_stop_datatx(u32 mcbsp_id);

/*
 * Stop receiving data on a McBSP interface
 */
int omap2_mcbsp_stop_datarx(u32 mcbsp_id);

/* Get the element index and frame index of transmitter */
int omap2_mcbsp_transmitter_index(int mcbsp_id, int *ei, int *fi);

/* Get the transmitter destination position */
int omap2_mcbsp_transmitter_destpos(int mcbsp_id);

/* Get the transmitter source position */
int omap2_mcbsp_transmitter_srcpos(int mcbsp_id);

/* Get the element index and frame index of receiver */
int omap2_mcbsp_receiver_index(int mcbsp_id, int *ei, int *fi);

/* Get the receiver destination position */
int omap2_mcbsp_receiver_destpos(int mcbsp_id);

/* Get the receiver source position */
int omap2_mcbsp_receiver_srcpos(int mcbsp_id);

/*
 * Register a ISR for McBSP interrupts.
 */
int omap2_mcbsp_register_isr(u32 mcbsp_id, omap2_mcbsp_isr_t isr, void *arg, unsigned int mask);

/*
 * Un-Register the ISR for McBSP interrupts.
 */
int omap2_mcbsp_unregister_isr(u32 mcbsp_id);

/*
 * Request / reserve a MCBSP interface
 */ 
int omap2_mcbsp_request_interface(u32 mcbsp_id,u8 interface_mode,u8 fclk_source);

/*
 * Release a reserved McBSP interface 
 */
int omap2_mcbsp_release_interface(u32 mcbsp_id);

/*
 * Receive a word in polling mode
 * returns received value
 */
int omap_mcbsp_pollread(u32 mcbsp_id);

/*
 * Transmit a word in polling mode
 */
int omap_mcbsp_pollwrite(u32 mcbsp_id, u32 buf);

/* 
 * Dump the McBSP register values
 */
int omap_mcbsp_dump_reg(u32 mcbsp_id);


#endif	
	
