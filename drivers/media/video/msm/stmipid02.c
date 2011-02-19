#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <asm/mach-types.h>


///////////////////////////////
/////  STMIPID Registers - 
///////////////////////////////

// TODO:: This should come from the board file - 
// MCLK configuration - 
#define STMIPID_MCLK_RATE_HZ  12000000

#pragma pack(1)

	struct stmipid_clk_lane_reg1 {
		uint8_t	enable				: 1;	// enable clock lane module 
		uint8_t	swap_pins_clk_lane 		: 1;	// Swap P and N pins 
		uint8_t ui_x4_clk_lane_0 		: 1;	// CSI control (unused in CCP mode) : Unit intervaltime multiplied by four
		uint8_t	ui_x4_clk_lane_1 		: 1;
		uint8_t	ui_x4_clk_lane_2 		: 1;
		uint8_t	ui_x4_clk_lane_3 		: 1;
		uint8_t	ui_x4_clk_lane_4 		: 1;
		uint8_t	ui_x4_clk_lane_5 		: 1;
	};

	struct stmipid_clk_lane_reg3 {
		uint8_t reserved			: 1;
		uint8_t	cntrl_mipi_subLVDS_clk_lane	: 1;	// Select CSI or CCP mode; 0= SMIA CCP; 1= MIPI CSI
		uint8_t	hs_rx_wakeup_subLVDS_clk_lane	: 1;	// High Speed Receiver wake-up enable for CCP mode (unsused in CSI mode)
		uint8_t hs_rx_e_subLVDS_clk_lane	: 1; 	// High Speed Receiver enable for CCP mode (unsused in CSI mode)
		uint8_t hs_rx_term_e_subLVDS_clk_lane	: 1; 	// High Speed termination enable for CCP mode (unsused in CSI mode)
		uint8_t reserved2			: 3; 	// High Speed termination enable for CCP mode (unsused in CSI mode)
	};
	
	struct stmipid_clk_lane_wr_reg1 {
		uint8_t stop_state_clk_lane		: 1;	// CSI Lane in stop state; This signal indicates that the lane module is in STOP state
		uint8_t ulp_active_not_clk_lane		: 1; 	// CSI Ultra low-power state active; 
								//  0 = The clock lane is not in ULP state or prepare to leave ULP state
								//  1 = The clock lane has reached the ULP state.
		uint8_t reserved			: 6;
	};
	
	struct stmipid_data_lane_reg1 {
		uint8_t enable_data_lane		: 1;		// Enable data lane 1.1 (DATA1P1 and DATA1N1)
									// 	0 = Disable data lane 1.1
									// 	1 = Enable data lane 1.1
		uint8_t swap_pins_data_lane		: 1;		// Swap P and N pins
									// 	0 = Swap enabled (DATA1P1 and DATA1N1 are swapped)
									// 	1 = Swap disabled
		uint8_t reserved			: 6;

	};

	struct stmipid_data_lane_reg2 {
		uint8_t cntrl_mipi_subLVDS_data_lane	: 1;		// Select CSI or CCP mode
									// 0 = SMIA CCP
									// 1 = MIPI CSI
		uint8_t hs_rx_wakeup_subLVDS_data_lane	: 1; 		// High Speed Receiver wake-up enable for CCP mode (unsused in CSI mode)
		uint8_t hs_rx_e_subLVDS			: 1;		// High Speed Receiver enable for CCP mode (unsused in CSI mode)
		uint8_t hs_rx_term_e_subLVDS		: 1; 		// High Speed termination enable for CCP mode (unsused in CSI mode)
									// 0 = Disable HS termination
									// 1 = Enable HS termination, mandatory for CCP mode
		uint8_t reserved			: 4;
	};

	struct stmipid_data_lane_reg3 {
		uint8_t stop_state_data_lane		: 1;		// CSI Lane in stop state. This signal indicates that the lane module is in STOP state
		uint8_t ulp_active_not_data_lane	: 1; 		// CSI Ultra low-power state active
									// 	0 = The clock lane is not in ULP state or prepare to leave ULP state
									// 	1 = The clock lane has reached the ULP state.
		uint8_t reserved			: 6;
	};

	struct stmipid_data_lane_reg4 {
		uint8_t err_sot_hs			: 1; 		// Error during high-speed start of transmission (SoT) If the high-speed SoT leader sequence is corrupted, but in such a way that proper synchronization can still be achieved, this error signal is asserted for one cycle of 8*UI. This is considered to be a â€œsoft errorâ€ in the leader sequence and confidence in the payload data is reduced.
		uint8_t err_sot_sync_hs			: 1;		// Synchronization error during high-speed start of transmission (SoT) If the high-speed SoT leader sequence is corrupted in a way that proper synchronization cannot be expected, this error signal is asserted for one cycle of 8*UI.
		uint8_t err_eot_sync_hs			: 1;		// Error during high-speed end of transmission (EoT) If a high-speed transmission ends when the number of bits received during that transmission is not a multiple of eight, this signal is asserted for one cycle of 8*UI.
		uint8_t err_esc				: 1;		// Error during escape command If an unrecognized escape entry command is received, this signal is asserted and remains high until the next change in line state.
		uint8_t err_sync_esc			: 1;		// Escape synchronization error If the number of bits received during a low-power data transmission is not a multiple of eight when the transmission ends, this signal is asserted and remains high until the next change in line state.
		uint8_t err_control			: 1;		// Unexpected control sequence error This signal is asserted when an incorrect line state sequence is detected. For example, if a turn-around request or escape mode request is immediately followed by a stop state instead of the required bridge state, this signal is asserted and remains high until the next change in line state.
		uint8_t reserved			: 2;		
	};
	
	struct stmipid_mode_reg1 {
		uint8_t csi_ccp				: 1;		// 1 = ccp; 0 = csi 
		uint8_t lane_ctrl_0			: 1; 		// 0 =	1-lane system 
									// 1 =  2-lane system 
		uint8_t lane_ctrl_1			: 1; 		// 0 = No swap, lane1 remains lane1 & lane2 remains lane2
									// 1 = Lanes are swapped, lane1 becomes lane2 & lane2 becomes lane1
		uint8_t decompression			: 3;		// 0 = disabled; 3 = 7 to 12
		uint8_t bypass_mode			: 1;		// 1 = no bypass; 0 = Bypass of the pixel generation & the decompression
		uint8_t justification_control		: 1; 		// Data Justification on output Data: 
									// 	0 = right justified (data on lower bits of bus)
									//	1 = left justified (Data on upper bits)
									// In bypass mode, this control is invalid	
	};

	struct stmipid_mode_reg2 {
		uint8_t interrupt_polarity		: 1; 		// Polarity for Interrupt signal
									//	0 = Non Inverted
									//	1 = Inverted	
		uint8_t output_polarity_hsync		: 1;		// Polarity control of HSYNC signal
									//	0= Non Inverted
									//	1= Inverted
		uint8_t output_polarity_vsync		: 1;		// Polarity control of VSYNC signal
									//	0= Non Inverted
									//	1= Inverted
		uint8_t output_polarity_clk		: 1;		// Polarity control of PCLK signal
									//	0= Non Inverted
									//	1= Inverted
		uint8_t clock_gating_control		: 1;		// Continuous or gated clock control
									//	0 = continuous clock
									// 	1 = clock gated
		uint8_t error_signal_polarity		: 1; 		// Polarity for Error signal
									//	0 = Non Inverted
									//	1 = Inverted
		uint8_t clear_error_signal		: 1; 		// Control to reset the error flag output
									//	0 = Reset the Error flag
									//	1 = Do not reset keep value as it is
		uint8_t tristate_output			: 1; 		// Programmation of the parallel interface in ouput or tristate mode
									//	0 = Tristated output
									//	1 = Normal output
	};

	struct stmipid_mode_reg3 {
		uint8_t camera_select			: 1;		// 0 = main camera 	(CLKP1,CLKN1,DATA1P1,DATA1N1,DATA2P1,DATA2N1)	
									// 1 = second camera 	(CLKP2,CLKN2,DATA1P2,DATA1N2)
		uint8_t reserved			: 4; 		
		uint8_t i2c_comp_leakage		: 1;		// Enable compensation macro
									//	0 = Disable IO compensation macro (push in IDDQmode)
									//	1 = Enable IO compensation macro (MANDATORY to push to normal mode)
		uint8_t	reserved2			: 2;		
	};

	struct stmipid_clock_control_reg1 {
		uint8_t reserved			: 4;
		uint8_t clr_csi2_error			: 1;		// I2C control to clear CSI error. It stops streaming data till this bit is reseted.
		uint8_t clr_csi2_interrupt		: 1;		// I2C control to clear CSI interrupt
		uint8_t reserved2			: 2;
	};

	struct stmipid_error_regs {
		uint8_t ecc_failed			: 1;		// ECC in low level protocol status
									//	0 = OK
									//	1 = Failed
		uint8_t checksum_failed			: 1;		// CHECKSUM in low level protocol status
									//	0 = OK
									//	1 = Failed
		uint8_t reserved			: 6;		
	};
	
	struct stmipid_data_selection_ctrl {
		uint8_t vc0				: 1;
		uint8_t vc1				: 1;
		uint8_t data_type			: 1;		// Selection of data type
									//	0 = Data type from data stream (readable in Data_ID_Wreg 0x11)
									//	1 = Data type from I2C programmed register (Data_ID_Rreg)
		uint8_t pixel_width_selection		: 1; 		// Selection of pixel width
									//	0 = Pixel width extracted from data type decided with Data_selection_ctrl[2]
									//	1 = Pixel width from I2C reg Pix_width_ctrl
		uint8_t reserved			: 4;		
	};

	struct stmipid_pix_width_ctrl {
		uint8_t pix_width			: 4; 		// pixel width in bits (8,10,12,16)
		uint8_t decompression_enable		: 1;		// Decompression enable for active data
									//	0= Decompression OFF
									//	1= Decompression ON
		uint8_t reserved			: 3;		
	};

	union stmipid_reg {
		uint8_t 				value; 
		uint8_t 				v;
		uint8_t 				val;
		
		// clock lane 1 : 
		#define STMIPID_ADDR_CLK_LANE_REG1		(0x0002)
		struct stmipid_clk_lane_reg1 		clk_lane_reg1;
		#define STMIPID_ADDR_CLK_LANE_REG3		(0x0004)
		struct stmipid_clk_lane_reg3		clk_lane_reg3;
		#define STMIPID_ADDR_CLK_LANE_WR_REG1		(0x0001)
		struct stmipid_clk_lane_wr_reg1		clk_lane_wr_reg1;

		// clock lane 2 : 
		#define STMIPID_ADDR_CLK_LANE_REG1_C2		(0x0031)
		struct stmipid_clk_lane_reg1 		clk_lane_reg1_c2;
		#define STMIPID_ADDR_CLK_LANE_REG2_C2		(0x0033)
		struct stmipid_clk_lane_reg3		clk_lane_reg3_c2;
		#define STMIPID_ADDR_CLK_LANE_WR_REG1_C2	(0x0039)
		struct stmipid_clk_lane_wr_reg1		clk_lane_wr_reg1_c2;
		
		// data lane 0 : 
		#define STMIPID_ADDR_DATA_LANE0_REG1		(0x0005)
		struct stmipid_data_lane_reg1	 	data_lane0_reg1;
		#define STMIPID_ADDR_DATA_LANE0_REG2		(0x0006)
		struct stmipid_data_lane_reg2	 	data_lane0_reg2;
		#define STMIPID_ADDR_DATA_LANE0_REG3		(0x0007)
		struct stmipid_data_lane_reg3	 	data_lane0_reg3;
		#define STMIPID_ADDR_DATA_LANE0_REG4		(0x000C)
		struct stmipid_data_lane_reg4	 	data_lane0_reg4;
		
		// data lane 1 : 
		#define STMIPID_ADDR_DATA_LANE1_REG1		(0x0009)
		struct stmipid_data_lane_reg1	 	data_lane1_reg1;
		#define STMIPID_ADDR_DATA_LANE1_REG2		(0x000A)
		struct stmipid_data_lane_reg2	 	data_lane1_reg2;
		#define STMIPID_ADDR_DATA_LANE1_REG3		(0x000B)
		struct stmipid_data_lane_reg3	 	data_lane1_reg3;
		#define STMIPID_ADDR_DATA_LANE1_REG4		(0x0008)
		struct stmipid_data_lane_reg4	 	data_lane1_reg4;

		// data lane 2 :
		#define STMIPID_ADDR_DATA_LANE3_REG1		(0x0034)
		struct stmipid_data_lane_reg1	 	data_lane3_reg1;
		#define STMIPID_ADDR_DATA_LANE3_REG2		(0x0035)
		struct stmipid_data_lane_reg2	 	data_lane3_reg2;
		#define STMIPID_ADDR_DATA_LANE3_REG3		(0x003A)
		struct stmipid_data_lane_reg3	 	data_lane3_reg3;
		#define STMIPID_ADDR_DATA_LANE3_REG4		(0x003B)
		struct stmipid_data_lane_reg4	 	data_lane3_reg4;

		// CCP config (not used) 
		#define STMIPID_ADDR_CCP_RX_REG1		(0x000D)
		uint8_t 				ccp_rx_reg1;
		#define STMIPID_ADDR_CCP_RX_REG2		(0x000E)
		uint8_t 				ccp_rx_reg2;
		#define STMIPID_ADDR_CCP_RX_REG3		(0x000F)
		uint8_t 				ccp_rx_reg3;
		#define STMIPID_ADDR_CCP_RX_REG1_C2		(0x0038)
		uint8_t 				ccp_rx_reg1_c2;

		// Mode registers - 
		#define STMIPID_ADDR_MODE_REG1			(0x0014)
		struct stmipid_mode_reg1		mode_reg1;
		#define STMIPID_ADDR_MODE_REG2			(0x0015)
		struct stmipid_mode_reg2		mode_reg2;
		#define STMIPID_ADDR_MODE_REG3			(0x0036)
		struct stmipid_mode_reg3		mode_reg3;

		// Clock control registers - 
		#define STMIPID_ADDR_CLOCK_CONTROL_REG1		(0x0016)
		struct stmipid_clock_control_reg1	clock_control_reg1;

		#define STMIPID_ADDR_ERROR_REGS			(0x0010)
		struct stmipid_error_regs		error_regs;
	
		#define STMIPID_ADDR_DATA_ID_WREG		(0x0011)
		uint8_t 				data_id_wreg;
		#define STMIPID_ADDR_DATA_ID_RREG		(0x0017)
		uint8_t 				data_id_rreg;
		#define STMIPID_ADDR_DATA_ID_RREG_EMB		(0x0018)
		uint8_t 				data_id_rreg_emb;
		#define STMIPID_ADDR_DATA_SELECTION_CTRL	(0x0019)
		struct stmipid_data_selection_ctrl	data_selection_ctrl;
		#define STMIPID_ADDR_FRAME_NO_LSB		(0x0012)
		uint8_t 				frame_no_lsb;
		#define STMIPID_ADDR_FRAME_NO_MSB		(0x0013)
		uint8_t 				frame_no_msb;
		#define STMIPID_ADDR_ACTIVE_LINE_NO_LSB		(0x001B)
		uint8_t 				active_line_no_lsb;
		#define STMIPID_ADDR_ACTIVE_LINE_NO_MSB		(0x001A)
		uint8_t 				active_line_no_msb;
		#define STMIPID_ADDR_SOF_LINE_NO_LSB		(0x001D)
		uint8_t 				SOF_line_no_lsb;
		#define STMIPID_ADDR_SOF_LINE_NO_MSB		(0x001C)
		uint8_t 				SOF_line_no_msb;
		#define STMIPID_ADDR_PIX_WIDTH_CTRL		(0x001E)
		struct stmipid_pix_width_ctrl		pix_width_ctrl;	
		#define STMIPID_ADDR_PIX_WIDTH_CTRL_EMB		(0x001F)
		struct stmipid_pix_width_ctrl		pix_width_ctrl_emb;
		#define STMIPID_ADDR_DATA_FIELD_LSB		(0x0021)
		uint8_t					dafa_dield_lsb;
		#define STMIPID_ADDR_DATA_FIELD_MSB		(0x0020)
		uint8_t					dafa_dield_msb;
	};

#pragma pack()


// i2c client to communicate with stmipid - 
static struct  i2c_client* stmipid_i2c_client = NULL;

// driver instance 
struct stmipid_device {
	struct device* 	pDev; 
	struct cdev 	cdev; 	
};

// sysfs class - 
static struct class* stmipid_class = NULL;

/** read data from a given device */
static int32_t stmipid_i2c_read(	unsigned short deviceAddress,
					unsigned char *buffer, 
					int length)
{
	int rc = 0;

	// prepare a write message -
	struct i2c_msg msgs[] = {
		{
			.addr   = deviceAddress,
			.flags = 0,
			.len   = 2,
			.buf   = buffer,
		},
		{
			.addr  = deviceAddress,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = buffer,
		},
	};

	if (buffer != NULL) {
		int rc = i2c_transfer(stmipid_i2c_client->adapter, msgs, 2);
	
		if (rc < 0) {
			CERR("can't i2c_transfer deviceAddress(0x%x) length(0x%x) = rc(%d)\n", 
						deviceAddress, 
						length,
						rc);
		}
	}
	else {
		rc = -EIO;
		CERR("bad buffer\n");
	}

	return rc;
}

/** read a register from the 16 bit address on the device */
static int32_t stmipid_i2c_read_reg16(	unsigned short deviceAddress,
					unsigned short registerAddress, 
					unsigned char* buffer)
{
	int32_t rc = 0;
	unsigned char buf[4] = {0};

	if (NULL == buffer) {
		CERR("Invalid buffer\n");
		rc = -EIO;
	}
	else {
		
		// prepare a read transaction - 
		buf[0] = (registerAddress & 0xFF00)>>8;
		buf[1] = (registerAddress & 0x00FF);

		rc = stmipid_i2c_read(deviceAddress, buf, 1);
		
		// output the data - 
		buffer[0] = buf[0];

		if (rc < 0) {
			CERR("i2c read failed rc = %d\n", rc);
			// clear the output - 
			buffer[0] = 0;
		}
		
		dbg_printf("  STMIPID 0x%04X R 0x%04X => 0x%04X\n", 
				deviceAddress, 
				registerAddress, 
				buffer[0]);
	}

	return rc;
}

/** write a given buffer to the device */
static int32_t stmipid_i2c_write(	uint16_t deviceAddress,
					uint8_t* buffer, 
					int32_t length)
{
	int ret = 0;
	struct i2c_msg msg[] = {
		{
			.addr 		= deviceAddress,
			.flags 		= 0,
			.len 		= length,
			.buf 		= buffer,
		},
	};

	ret = i2c_transfer(stmipid_i2c_client->adapter, msg, 1);

	if (ret < 0) {
		CERR("failed to write addr 0x%x length %d ret = %d\n", 
					deviceAddress, 
					length,
					ret);
		ret = -EIO;
	}

	return 0;
}

static int32_t stmipid_i2c_write_reg16(	unsigned short deviceAddress,
					unsigned short registerAddress, 
					uint8_t value)
{
	int32_t rc = -EIO;
	unsigned char buf[4];

	dbg_printf("  STMIPID 0x%04X W 0x%04X <= 0x%04X\n", 
				deviceAddress, 
				registerAddress, 
				value);

	// convert to big endian - 
	buf[0] = (uint8_t)((registerAddress >> 8) & 0xFF);
	buf[1] = (uint8_t)((registerAddress >> 0) & 0xFF);
	buf[2] = value;
	buf[3] = 0;

	// write to i2c - 
	rc = stmipid_i2c_write(deviceAddress, buf, 3);

	if (rc < 0) {
		CERR("failed to write addr(0x%x) register(0x%x) value(0x%x)\n", 
					deviceAddress, 
					registerAddress,
					value);
	}

	return rc;
}

static void stmipid_print_state(void)
{
	union stmipid_reg pix_width_ctrl = {0};
	union stmipid_reg pix_width_ctrl_emb = {0};
	uint8_t frame_no_lsb =0;
	uint8_t frame_no_msb =0;
	uint8_t active_lines_lsb =0;
	uint8_t active_lines_msb =0;
	union stmipid_reg r1;
	union stmipid_reg r2;
	union stmipid_reg r3;
	union stmipid_reg r4;
	union stmipid_reg r5;
	union stmipid_reg r6;
	union stmipid_reg r7;

	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_PIX_WIDTH_CTRL, 		&pix_width_ctrl.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_PIX_WIDTH_CTRL_EMB, 	&pix_width_ctrl_emb.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_FRAME_NO_MSB, 		&frame_no_msb);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_FRAME_NO_LSB, 		&frame_no_lsb);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_ACTIVE_LINE_NO_MSB, 	&active_lines_msb);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_ACTIVE_LINE_NO_LSB, 	&active_lines_lsb);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_DATA_LANE0_REG1, 		&r1.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_DATA_LANE0_REG2, 		&r2.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_DATA_LANE0_REG3, 		&r3.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_DATA_LANE0_REG4, 		&r4.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_CLK_LANE_WR_REG1, 	&r5.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_DATA_ID_RREG, 		&r6.value);
	stmipid_i2c_read_reg16(stmipid_i2c_client->addr, STMIPID_ADDR_DATA_ID_RREG_EMB, 	&r7.value);
	
	dbg_printf("=================================================================\n");
	dbg_printf("Bridge regs :\n");	
	dbg_printf(" pix_width_ctrl                   0x%04x \n", 	pix_width_ctrl.value);
	dbg_printf("   pix_width                      0x%04x \n", 	pix_width_ctrl.pix_width_ctrl.pix_width);
	dbg_printf("   decompression                  0x%04x \n", 	pix_width_ctrl.pix_width_ctrl.decompression_enable);
	dbg_printf(" pix_width_ctrl_emb               0x%04x \n", 	pix_width_ctrl_emb.value);
	dbg_printf("   pix_width                      0x%04x \n", 	pix_width_ctrl_emb.pix_width_ctrl.pix_width);
	dbg_printf("   decompression                  0x%04x \n", 	pix_width_ctrl_emb.pix_width_ctrl.decompression_enable);
	dbg_printf(" frame NO                         0x%04x \n", 	(frame_no_msb << 8) | frame_no_lsb);
	dbg_printf(" active lines  NO                 0x%04x \n", 	(active_lines_msb << 8) | active_lines_lsb);
	dbg_printf(" id                               0x%04x \n", 	r6.value);
	dbg_printf(" id emb                           0x%04x \n", 	r7.value);
	dbg_printf(" clk_lane_wr_reg1                 0x%04x \n", 	r5.value);	
	dbg_printf("   ulp_active_not_clk_lane        0x%04x \n", 	r5.clk_lane_wr_reg1.ulp_active_not_clk_lane);	
	dbg_printf("   stop_state_clk_lane            0x%04x \n", 	r5.clk_lane_wr_reg1.stop_state_clk_lane);	
	dbg_printf(" lane0,reg1                       0x%04x \n", 	r1.value);	
	dbg_printf("   enable_data_lane               0x%04x \n", 	r1.data_lane0_reg1.enable_data_lane);	
	dbg_printf("   swap_pins_data_lane            0x%04x \n", 	r1.data_lane0_reg1.swap_pins_data_lane);	
	dbg_printf(" lane0,reg2                       0x%04x \n", 	r2.value);	
	dbg_printf("   cntrl_mipi_subLVDS_data_lane   0x%04x \n", 	r2.data_lane0_reg2.cntrl_mipi_subLVDS_data_lane);	
	dbg_printf("   hs_rx_wakeup_subLVDS_data_lane 0x%04x \n", 	r2.data_lane0_reg2.hs_rx_wakeup_subLVDS_data_lane);	
	dbg_printf("   hs_rx_e_subLVDS                0x%04x \n", 	r2.data_lane0_reg2.hs_rx_e_subLVDS);	
	dbg_printf("   hs_rx_term_e_subLVDS           0x%04x \n", 	r2.data_lane0_reg2.hs_rx_term_e_subLVDS);	
	dbg_printf(" lane0,reg3                       0x%04x \n", 	r3.value);	
	dbg_printf("   ulp_active_not_data_lane       0x%04x \n", 	r3.data_lane0_reg3.ulp_active_not_data_lane);	
	dbg_printf("   stop_state_data_lane           0x%04x \n", 	r3.data_lane0_reg3.stop_state_data_lane);	
	dbg_printf(" lane0,reg4                       0x%04x \n", 	r4.value);	
	dbg_printf("   err_control                    0x%04x \n", 	r4.data_lane0_reg4.err_control);	
	dbg_printf("   err_sync_esc                   0x%04x \n", 	r4.data_lane0_reg4.err_sync_esc);	
	dbg_printf("   err_esc                        0x%04x \n", 	r4.data_lane0_reg4.err_esc);	
	dbg_printf("   err_eot_sync_hs                0x%04x \n", 	r4.data_lane0_reg4.err_eot_sync_hs);	
	dbg_printf("   err_sot_sync_hs                0x%04x \n", 	r4.data_lane0_reg4.err_sot_sync_hs);	
	dbg_printf("   err_sot_hs                     0x%04x \n", 	r4.data_lane0_reg4.err_sot_hs);	
	dbg_printf("=================================================================\n");	
}

static int stmipid_initialize(void);

static ssize_t _stmipid_read(	struct file* filep, 
				char* pUserData, 
				size_t iSizeToRead, 
				loff_t* pOffset)
{
	dbg_printf("Read start\n");
	
	stmipid_print_state();	

	dbg_printf("Read end\n");
	return 0;
}

static ssize_t _stmipid_write(	struct file* filep, 
				const char __user * pUserData, 
				size_t bytesNum, 
				loff_t* pOffset)
{
	dbg_printf("Writing Configuration Start\n");

	stmipid_initialize();
	
	dbg_printf("Writing Configuration End\n");

	return 0;
}


static int _stmipid_fopen(struct inode *inode, struct file *filep)
{
	int rc = 0;
	struct stmipid_device *pStmipidDevice =
		container_of(inode->i_cdev, struct stmipid_device, cdev);

	rc = pStmipidDevice != NULL ? 0 : -1;

	dbg_printf("open stmipid\n");

	if (pStmipidDevice != NULL) {
		dbg_printf("open stmipid %p\n", pStmipidDevice);
		// TODO:: Call print_state.

		// stmipid_print_state();
	}

	if (rc >= 0) {
		rc = nonseekable_open(inode, filep);
		if (rc < 0) {
			CERR("failed to nonseekable_open() = %d\n", rc);
		}
		else {
			// link to the device - 
			filep->private_data = pStmipidDevice;
		}
	}
	
	dbg_printf("open stmipid - rc %d\n", rc);

	// TODO:: clean up in case of an error - 

	return rc;
}


// stmipid file ops - 
static const struct file_operations stmipid_fops = {
	.owner 		= THIS_MODULE,
	.open 		= _stmipid_fopen,
	.read		= _stmipid_read,
	.write		= _stmipid_write,
	.unlocked_ioctl = NULL, 
	.release 	= NULL,
};

static void stmipid_delay(void)
{
	mdelay(100);
}

// i2c probe - 
static int stmipid_i2c_probe(	struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = 0;

	dbg_printf("Probing STMIPID i2c client(%p) id(%p) - start\n", client, id);

	ret = i2c_check_functionality(client->adapter, I2C_FUNC_I2C); 

	// check i2c connection - 
	if (!ret) {
		CERR("can't check i2c_check_functionality %d\n", ret);	
		ret = -ENOTSUPP;
	}
	else {
		ret = 0;
	}

//	mt9t013_sensorw = kzalloc(sizeof(struct mt9t013_work), GFP_KERNEL);
//	if (!mt9t013_sensorw) {
//		rc = -ENOMEM;
//		ret = -ENOTSUPP;
//		goto probe_failure;
//	}

	if (!ret) {
		// Store some private data - 
		i2c_set_clientdata(client, (void*)0x12345678);
		// mt9t013_init_client(client);

		// store the client - 
		stmipid_i2c_client 		= client;
	}

	if (stmipid_i2c_client != NULL) {
		dbg_printf("probe STMIPID i2c 7bit(0x%x) 8bit(0x%x)\n", 
					stmipid_i2c_client->addr,
					stmipid_i2c_client->addr << 1);
	}
	else {
		CERR("ERROR: probe STMIPID\n");
	}

	stmipid_delay();

	dbg_printf("Probing STMIPID i2c - end\n");

	return ret;
}


// i2c connection -  
static const struct i2c_device_id stmipid_i2c_id[] = {
	{ "stmipid", 0},
	{ }
};

// i2c connetion - 
static struct i2c_driver stmipid_i2c_driver = {
	.id_table 	= stmipid_i2c_id,
	.probe  	= stmipid_i2c_probe,
	.remove 	= __exit_p(stmipid_i2c_remove),
	.driver = {
		.name = "stmipid",
	},
};


static int stmipid_create_sysfs(void)
{
	int rc = 0;
	int iDev; 
	int dev_num = 0;
	struct device* pDev = NULL;
	struct stmipid_device* pStmipidDevice = NULL;
	dev_t stmipid_devno; 

	pStmipidDevice = kzalloc(sizeof(*pStmipidDevice), GFP_ATOMIC);
	rc = pStmipidDevice != NULL ? 0 : -ENOMEM ;
	CASSERT(0 == rc);

	dbg_printf("Allocated pStmipidDevice = %p\n", pStmipidDevice);

	if (rc >= 0) {
		dbg_printf("Allocating CHRDEV\n");
		rc = alloc_chrdev_region(&stmipid_devno, 0, 1, "stmipid");
		if (rc < 0) {
			CERR("Can't allocate chrdev %d\n", rc);
			return rc;
		}
	}

	iDev = MKDEV(MAJOR(stmipid_devno), dev_num);
	dbg_printf("devno 0x%x major 0x%x iDev 0x%x\n", 
				stmipid_devno,
				MAJOR(stmipid_devno),
				iDev);

	if (rc >= 0 && NULL == stmipid_class) {

		dbg_printf("Creating class\n");
		stmipid_class = class_create(THIS_MODULE, "stmipid_class");
	
		if (IS_ERR(stmipid_class)) {
			rc = PTR_ERR(stmipid_class);
			CERR("filaed to create class %d\n", rc);
			stmipid_class = NULL;
			return rc;
		}
	}

	if (rc >= 0) {
		pDev = device_create(	stmipid_class, 
					NULL, 
					stmipid_devno, 
					NULL, 
					"stmipid_dev");

		if (IS_ERR(pDev)) {
			rc = PTR_ERR(pDev);
			CERR("can't create a device %d\n", rc);
			return rc;
		}
	}

	if (rc >= 0) {
		dbg_printf("add cdev device\n");
		// initialize cdev - 
		cdev_init(&pStmipidDevice->cdev, &stmipid_fops);
		pStmipidDevice->cdev.owner = THIS_MODULE;
		rc = cdev_add(&pStmipidDevice->cdev, stmipid_devno, 1);

		if (rc < 0) {
			CERR("can't add cdev %d\n", rc);
			return rc;
		}
	}


	// TODO:: add cleanup here in case of an error - 

	return rc;
}

/** configure the stmipid for MIPI => Paraller parallel conversion */
static int stmipid_initialize(void)
{
	int rc = 0;
	union stmipid_reg r = {0};
		
	dbg_printf("init stmipid sizeof(r) = %d\n", sizeof(r));

	// TODO:: constant address should come from the board file. 
	// TODO:: make this a function form a board file.
	// stop VD6953 sensor streaming while i configure the bridge - 
	// stmipid_i2c_write_reg16(0x10,0x100, 0x0);

	// enable clock lane, ui programed for 800Mps
	r.value = 0;
	r.clk_lane_reg1.ui_x4_clk_lane_2	= 1;
	r.clk_lane_reg1.ui_x4_clk_lane_0	= 1;
	r.clk_lane_reg1.enable			= 1;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_CLK_LANE_REG1, r.value);
	stmipid_delay();

	// mipi mode clock 
	r.value = 0;
	r.clk_lane_reg3.hs_rx_term_e_subLVDS_clk_lane	= 1;
	r.clk_lane_reg3.hs_rx_e_subLVDS_clk_lane	= 1;
	r.clk_lane_reg3.hs_rx_wakeup_subLVDS_clk_lane	= 1;
	r.clk_lane_reg3.cntrl_mipi_subLVDS_clk_lane	= 1;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_CLK_LANE_REG3, r.value);
	stmipid_delay();

	r.value = 0;
	r.data_lane0_reg1.enable_data_lane		= 1;	// enable DATA1P1 and DATA1N1
	r.data_lane0_reg1.swap_pins_data_lane		= 1;	// swap is disabled. 
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_DATA_LANE0_REG1, r.value);
	stmipid_delay();

	r.value = 0;
	r.data_lane0_reg2.cntrl_mipi_subLVDS_data_lane		= 1;
	r.data_lane0_reg2.hs_rx_wakeup_subLVDS_data_lane	= 1;
	r.data_lane0_reg2.hs_rx_e_subLVDS			= 1;
	r.data_lane0_reg2.hs_rx_term_e_subLVDS			= 1;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_DATA_LANE0_REG2, r.value);
	stmipid_delay();

	// 
	r.value = 0;
	r.data_lane1_reg1.enable_data_lane			= 0;	// don't use the second line. 
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_DATA_LANE1_REG1, r.value);
	stmipid_delay();
		
	// WriteByte(0x14, 0x40); //ModeControlRegisters-- no bypass & no decomp, 1 Lane System, CSI streaming
	r.value = 0;
	r.mode_reg1.csi_ccp 			= 0;
	r.mode_reg1.bypass_mode			= 1;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_MODE_REG1, r.value);
	stmipid_delay();

	// WriteByte(0x15, 0x48); //ModeControlRegisters-- 
	r.value = 0;
	r.mode_reg2.clear_error_signal 		= 1;
	r.mode_reg2.output_polarity_clk		= 1;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_MODE_REG2, r.value);
	stmipid_delay();	

	// WriteByte(0x36, 0x00);
	r.value = 0;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_MODE_REG3, r.value);
	stmipid_delay();

	// WriteByte(0x17, 0x2b); //Data_ID_Rreg
	r.value = 0x2b;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_DATA_ID_RREG, r.value);
	stmipid_delay();

	// WriteByte(0x18, 0x2b); //Data_ID_Rreg_emb
	r.value = 0x2b;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_DATA_ID_RREG_EMB, r.value);
	stmipid_delay();

	// WriteByte(0x19, 0x0c);
	r.value = 0;
	r.data_selection_ctrl.pixel_width_selection		= 1;
	r.data_selection_ctrl.data_type				= 1;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_DATA_SELECTION_CTRL, r.value);
	stmipid_delay();

	// WriteByte(0x1e, 0x0a);
	r.value = 0;
	r.pix_width_ctrl.pix_width				= 0xa;
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_PIX_WIDTH_CTRL, r.value);
	stmipid_delay();

	// WriteByte(0x1f, 0x0a);
	stmipid_i2c_write_reg16(stmipid_i2c_client->addr,STMIPID_ADDR_PIX_WIDTH_CTRL_EMB, r.value);
	stmipid_delay();

	return rc;
}

// driver probe - 
static int32_t stmipid_driver_probe(struct platform_device *pdev)
{
	int ret; 
	int isDriverAdded = 0;
	
	dbg_printf("Probing STMIPID driver - start\n");

	// ret = msm_camera_drv_start(pdev, mt9t013_sensor_probe);
	ret = 0;


	// add i2c stmipid driver - 
	ret = i2c_add_driver(&stmipid_i2c_driver);
	
	dbg_printf("adding i2c driver  ret(%d) i2c_client(0x%p)\n",
				ret, 
				stmipid_i2c_client);

	if (ret < 0 || stmipid_i2c_client == NULL) {
		ret = -ENOTSUPP;
	}
	else {
		isDriverAdded = 1;
	}

	// activate the mclk - 
	if (0 == ret) {
		dbg_printf("Setting MCLK to %d Hz\n", STMIPID_MCLK_RATE_HZ);
		// make sure the clock is enabled -  
		msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
		msm_camio_clk_rate_set(STMIPID_MCLK_RATE_HZ);
		// geve the stmipid / sensor time to wake up - 
		stmipid_delay();

		if (ret != 0 ) {
			CERR("can't set MCLK to %d Hz\n", STMIPID_MCLK_RATE_HZ);
		}
		else {
			dbg_printf("Setting MCLK to %d Hz - done\n", STMIPID_MCLK_RATE_HZ);
		}
	}

	// read the chipid register - 
	if (0 == ret) {
		unsigned char id = 0xAA;

		// write id - 
		ret = stmipid_i2c_write_reg16(	stmipid_i2c_client->addr,
						STMIPID_ADDR_DATA_ID_RREG,
						id);

		// read id back - 
		ret = stmipid_i2c_read_reg16(	stmipid_i2c_client->addr,
						STMIPID_ADDR_DATA_ID_RREG, 
						&id);

		dbg_printf("STMIPID ret(%d) id(0x%x)\n", ret, id);
	}

	if (0 == ret) {
		// configure the stmipid bridge - 
		ret = stmipid_initialize();
		CASSERT(0 == ret);
	}

	if (0 == ret) {
		ret = stmipid_create_sysfs();
		CASSERT(0 == ret);
	}



///////////////////
//// clean up -- 

	if (ret != 0) {
		CERR("Failed - clening up\n");
		if (isDriverAdded) {
			i2c_del_driver(&stmipid_i2c_driver);
			isDriverAdded = 0;
		}
	}


	dbg_printf("Probing STMIPID driver - end\n");

	return ret;
}

static struct platform_driver msm_camera_stmipitd_driver = {
	.probe 		= stmipid_driver_probe,
	.driver 	= {
		.name = "msm_camera_stmipid",
		.owner = THIS_MODULE,
	},
};

// Driver init function to register a driver. 
static int __init stmipid_init(void)
{
	int ret;

	dbg_printf("initialize STMIPID - start\n");
	ret = platform_driver_register(&msm_camera_stmipitd_driver);
	dbg_printf("initialize STMIPID - end\n");

	return ret;
}

// register my new and cool driver - 
module_init(stmipid_init);






