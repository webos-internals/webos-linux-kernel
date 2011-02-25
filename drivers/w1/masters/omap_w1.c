/*
 * omap_w1.c
 *
 * Copyright (c) 2008 Palm Inc.
 * 	Modified from the original HDQ driver that came from TI
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/hdq.h>
#include <asm/arch/io.h>


#include "../w1.h"
#include "../w1_int.h"
#include "../w1_log.h"

/* defines */
//#define OMAP_HDQ_DEBUG 1

#define MOD_NAME "OMAP_HDQ: "

#ifdef OMAP_HDQ_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ALERT MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define SEARCH_NET_ADDRESS_CMD			0xF0
#define ROM_CODE_SIZE				0x08

#define W1_RESET_BUS_ERROR			-1
#define W1_RESET_BUS_DEVICE_PRESENT		0
#define W1_RESET_BUS_NO_DEVICE_PRESENT		1

struct omap_hdq_data
{
	u32				hdq_base_addr;

	int				hdq_irq;

	struct clk 			*hdq_ick;
	struct clk 			*hdq_fck;

	struct platform_device 		*pdev;

	volatile u8 			hdq_irq_status;

	struct timer_list		hdq_timer;
	u8				hdq_is_up;
	u32				hdq_timeout;

	struct completion		reset_done;
	struct completion		read_done;
	struct completion		write_done;
};

/*
 * HDQ register I/O routines
 */
static __inline__ u8 hdq_reg_in(struct omap_hdq_data *omap_hdq_data,
				u32 offset)
{
	return readb(omap_hdq_data->hdq_base_addr + offset);
}

static __inline__ void hdq_reg_out(struct omap_hdq_data *omap_hdq_data,
				   u32 offset, u8 val)
{
	writeb(val, omap_hdq_data->hdq_base_addr + offset);
}

static __inline__ void hdq_reg_merge(struct omap_hdq_data *omap_hdq_data,
				     u32 offset, u8 val, u8 mask)
{
	u8 new_val = (readb(omap_hdq_data->hdq_base_addr + offset) & ~mask) |
			(val & mask);
	writeb(new_val, omap_hdq_data->hdq_base_addr + offset);
}

/*
 * Wait for one or more bits in flag change.
 * HDQ_FLAG_SET: wait until any bit in the flag is set.
 * HDQ_FLAG_CLEAR: wait until all bits in the flag are cleared.
 * return 0 on success and -ETIMEOUT in the case of timeout.
 */
static int hdq_wait_for_flag(struct omap_hdq_data *omap_hdq_data,
				u32 offset,
				u8 flag,
				u8 flag_set)
{
	int ret = 0;
	unsigned long timeout = jiffies + HDQ_TIMEOUT;
	u8 status;

	if (flag_set == HDQ_FLAG_CLEAR) {
		/* wait for the flag clear */
		while (((status = hdq_reg_in(omap_hdq_data, offset)) & flag)
		       && time_before(jiffies, timeout)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		}
		if (unlikely(status & flag))
			ret = -ETIMEDOUT;

	} else if (flag_set == HDQ_FLAG_SET) {
		/* wait for the flag set */
		while (!((status = hdq_reg_in(omap_hdq_data, offset)) & flag)
		       && time_before(jiffies, timeout)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
		}
		if (unlikely(!(status & flag)))
			ret = -ETIMEDOUT;
	} else
		return -EINVAL;

	return ret;
}

static irqreturn_t hdq_isr(int irq, void *data)
{
	struct omap_hdq_data *omap_hdq_data = data;
	u8 intr;

	intr = hdq_reg_in(omap_hdq_data, HDQ_INT_STATUS);

	if (intr & HDQ_INT_STATUS_TIMEOUT)
		complete(&omap_hdq_data->reset_done);

	if (intr & HDQ_INT_STATUS_RXCOMPLETE)
		complete(&omap_hdq_data->read_done);

	if (intr & HDQ_INT_STATUS_TXCOMPLETE)
		complete(&omap_hdq_data->write_done);

	return IRQ_HANDLED;
}

static u8 hdq_read_bit(void *data)
{
	struct omap_hdq_data *omap_hdq_data = data;
	u8 val;
	unsigned long timeleft;

	/* clear interrupt flags via a dummy read */
	hdq_reg_in(omap_hdq_data, HDQ_INT_STATUS);

	hdq_reg_out(omap_hdq_data, HDQ_CTRL_STATUS,
		    HDQ_CTRL_STATUS_1WIRE_SINGLE_BIT | HDQ_CTRL_STATUS_DIR |
		    HDQ_CTRL_STATUS_CLOCKENABLE | HDQ_CTRL_STATUS_MODE |
		    HDQ_CTRL_STATUS_INTERRUPTMASK);

	INIT_COMPLETION(omap_hdq_data->read_done);

	hdq_reg_merge(omap_hdq_data, HDQ_CTRL_STATUS,
		      HDQ_CTRL_STATUS_GO, HDQ_CTRL_STATUS_GO);

	timeleft = wait_for_completion_timeout(&omap_hdq_data->read_done,
								HDQ_TIMEOUT);
	if (!timeleft) {
		printk(KERN_ERR MOD_NAME
			"%s timeout waiting for RXCOMPLETE\n", __FUNCTION__);
	}

	/* The data is ready. Read it in */
	val = hdq_reg_in(omap_hdq_data, HDQ_RX_DATA) & 0x1;
	return val;
}

static void hdq_write_bit(void *data, u8 val)
{
	struct omap_hdq_data *omap_hdq_data = data;
	int ret;
	unsigned long timeleft;

	/* clear interrupt flags via a dummy read */
	hdq_reg_in(omap_hdq_data,HDQ_INT_STATUS);

	hdq_reg_out(omap_hdq_data,
		    HDQ_CTRL_STATUS,
		    HDQ_CTRL_STATUS_1WIRE_SINGLE_BIT | HDQ_CTRL_STATUS_DIR |
		    HDQ_CTRL_STATUS_CLOCKENABLE | HDQ_CTRL_STATUS_MODE |
		    HDQ_CTRL_STATUS_INTERRUPTMASK);

	hdq_reg_out(omap_hdq_data,HDQ_TX_DATA, val&0x1);

	INIT_COMPLETION(omap_hdq_data->write_done);

	hdq_reg_merge(omap_hdq_data,
			HDQ_CTRL_STATUS,
			HDQ_CTRL_STATUS_GO,
			HDQ_CTRL_STATUS_DIR|HDQ_CTRL_STATUS_GO);

	timeleft = wait_for_completion_timeout(&omap_hdq_data->write_done,
								HDQ_TIMEOUT);
	if (!timeleft) {
		printk(KERN_ERR MOD_NAME
			"%s: timeout waiting for TXCOMPLETE\n", __FUNCTION__);
	}

	/* wait for the GO bit return to zero */
	ret = hdq_wait_for_flag(omap_hdq_data,
				HDQ_CTRL_STATUS,
				HDQ_CTRL_STATUS_GO,
			        HDQ_FLAG_CLEAR);
	if (ret) {
		printk(KERN_ERR MOD_NAME
			"%s: timeout waiting GO bit to zero\n", __FUNCTION__);
	}
}

/*
 * Address/slave searching routines
 */
/*
 *
 * omap_hdq_searchROMAccelerator()
 *
 * Before calling this routine a 1-wire reset must be issued along
 * with the Search ROM command.
 * This functions requires that two functions - readbit and writebit
 * exist to write and read individual bits to and from the 1-wire bus.
 * Readbit must return either a 1 or a 0, and writebit must accept either
 * a 1 or a 0.
 * This function is designed to operate directly with the
 * omap_hdq_recoverROM routine. The TransmitData array generated
 * by omap_hdq_recoverROM is passed in as Input. The data stored
 * in Ouput is then given back to omap_hdq_recoverROM as RecieveData.
 * Revision 1:
 * First Release. Unpack the Input data into 64 seperate bits. Follow the
 * search ROM procedure of reading the first bit, reading the inverse,
 * then writing the selection. Repeat for each bit. Then pack the data to
 * be returned as Output.
 * Choices for each bit are:
 *	00 -> bit conflict, write the conflict bit to the bus, set the
 *		  discrepancy flag for this location.
 *	01 -> all devices are 0, write and return a 0, do not set discrepancy
 *	10 -> all devices are 1, write and return a 1, do not set discrepancy
 *	11 -> bus error, write and return a 1, set discrepancy
 *
 * Inputs
 *   Input       128-bit array given to the 1-wire master.
 *   Output      128-bit response
 *
 * Returns
 *   None
 *
 */
static void hdq_searchROMAccelerator(struct omap_hdq_data *omap_hdq_data,
					int *Input, int *Output)
{
	int loop,bit,inverse;
	int discrepancy[64];
	int conflict[64];
	int selection[64];

	/* Unpack the conflict bits from the input */
	for (loop = 0; loop < 64; loop++)
		if (Input[loop/4] & (0x02 << (2 * (loop % 4))))
			conflict[loop] = 1;
		else
			conflict[loop] = 0;


	/* Run the search ROM procedure */
	for (loop = 0; loop < 64; loop++) {
		bit = hdq_read_bit(omap_hdq_data);
		inverse = hdq_read_bit(omap_hdq_data);
		/* Bit conflict at this location */
		if ((bit == 0) && (inverse == 0)) {
			hdq_write_bit(omap_hdq_data, conflict[loop]);
			discrepancy[loop]=1;
			selection[loop]=conflict[loop];
		}
		/* All devices have 0 at this location */
		if ((bit == 0) && (inverse == 1)) {
			hdq_write_bit(omap_hdq_data, 0);
			discrepancy[loop]=0;
			selection[loop]=0;
		}
		/* All devices have 1 at this location */
		if ((bit == 1) && (inverse == 0)) {
			hdq_write_bit(omap_hdq_data, 1);
			discrepancy[loop]=0;
			selection[loop]=1;
		}
		/* Bus error */
		if ((bit == 1) && (inverse == 1)) {
			hdq_write_bit(omap_hdq_data, 1);
			discrepancy[loop]=1;
			selection[loop]=1;
		}
	}

	/* Pack results into the Output */
	for (loop = 0; loop < 16; loop ++) {
		Output[loop] = 	discrepancy[loop*4] +
				(selection[loop*4] << 1) +
				(discrepancy[loop*4+1] << 2) +
				(selection[loop*4+1] << 3) +
				(discrepancy[loop*4+2] << 4) +
				(selection[loop*4+2] << 5) +
				(discrepancy[loop*4+3] << 6) +
				(selection[loop*4+3] << 7);
	}
}

/*
 * omap_hdq_recoverROM()
 *
 * This routine performs two functions. Given 16 bytes of receive
 * data taken from the 1-Wire Master during a Search ROM function, it will
 * extract the ROM code found into an 8 byte array and it will generate
 * the next 16 bytes to be transmitted during the next Search ROM.
 * omap_hdq_recoverROM must be initialized by sending a NULL pointer in
 * ReceivedData. It will write 16 bytes of zeros into TransmitData and clear
 * the discrepancy tree. The discrepancy tree keeps track of which ROM
 * discrepancies have already been explored.
 * omap_hdq_recoverROM also returns a value telling whether there are any
 * more ROM codes to be found. If a zero is returned, there are still
 * discrepancies. If a one is returned all ROMs on the bus have been found.
 * Running RecoverROM again in this case will result in repeating ROM codes
 * already found
 *
 * Inputs
 *   ReceiveData   16-bytes array from the 1-Wire Master.
 *   TransmitData  Next generated 16 bytes to be transmitted
 *                 during the next Search ROM.
 *   ROMCode       8-byte array ROM code that is extracted from ReceiveData
 *
 * Returns
 *   0 on success.
 *   1 either on the first run to initialize TransmitData or there is no
 *   slave to be searched.
 *
 */
static int hdq_recoverROM(struct omap_hdq_data *omap_hdq_data,
			int *ReceiveData,
			int *TransmitData,
			int *ROMCode)
{
	int loop;
	int result;
	int TROM[64];		/* the transmit value being generated */
	int RROM[64];		/* the ROM recovered from the received data */
	int RDIS[64];		/* the discrepancy bits in the received data */
	static int TREE[64];	/* used to keep track of which discrepancy bits*/
				/* already been flipped. */

	/* If receivedata is NULL, this is the first run. Transmit data
	 * should be all zeros, and the discrepancy tree must also be reset.
	 */
	if (ReceiveData == NULL) {
		for (loop = 0; loop < 64; loop++)
			TREE[loop] = 0;
		for (loop = 0; loop < 16; loop++)
			TransmitData[loop] = 0;
		return 1;
	}

	/* De-interleave the received data into the new ROM code and
	 * the discrepancy bits
	 */
	for (loop = 0; loop < 16; loop++) {
		if (!(ReceiveData[loop] & 0x02))
			RROM[loop*4]   = 0;
		else
			RROM[loop*4  ] = 1;

		if (!(ReceiveData[loop] & 0x08))
			RROM[loop*4+1] = 0;
		else
			RROM[loop*4+1] = 1;

		if (!(ReceiveData[loop] & 0x20))
			RROM[loop*4+2] = 0;
		else
			RROM[loop*4+2] = 1;

		if (!(ReceiveData[loop] & 0x80))
			RROM[loop*4+3] = 0;
		else
			RROM[loop*4+3] = 1;

		if (!(ReceiveData[loop] & 0x01))
			RDIS[loop*4]   = 0;
		else
			RDIS[loop*4  ] = 1;

		if (!(ReceiveData[loop] & 0x04))
			RDIS[loop*4+1] = 0;
		else
			RDIS[loop*4+1] = 1;

		if (!(ReceiveData[loop] & 0x10))
			RDIS[loop*4+2] = 0;
		else
			RDIS[loop*4+2] = 1;

		if (!(ReceiveData[loop] & 0x40))
			RDIS[loop*4+3] = 0;
		else
			RDIS[loop*4+3] = 1;
	}

	/* Initialize the transmit ROM to the recovered ROM */
	for (loop = 0; loop < 64; loop++)
		TROM[loop] = RROM[loop];

	/* Work through the new transmit ROM backwards setting every bit
	 * to 0 until the most significant discrepancy bit which has not
	 * yet been flipped is found. The transmit ROM bit at that location
	 * must be flipped.
	 */
	for (loop = 63; loop >= 0; loop--) {
		/* This is a new discrepancy bit. Set the indicator in
		 * the tree, flip the transmit bit, and then break
		 * from the loop.
		 */
		if ((TREE[loop] == 0) && (RDIS[loop] == 1) &&
		    (TROM[loop] == 0)) {
			TREE[loop] = 1;
			TROM[loop] = 1;
			break;
		}
		if ((TREE[loop] == 0) && (RDIS[loop] == 1) &&
		    (TROM[loop] == 1)) {
			TREE[loop] = 1;
			TROM[loop] = 0;
			break;
		}

		/* This bit has already been flipped, remove it from the tree
		 * and continue setting the transmit bits to zero.
		 */
		if ((TREE[loop] == 1) && (RDIS[loop] == 1))
			TREE[loop] = 0;
		TROM[loop] = 0;
	}
	result = loop; /* if loop made it to -1, then there are no more
			* discrepancy bits and the search can end.
			*/

	/* Convert the individual transmit ROM bit into a 16 byte format
	 * every other bit is don't care.
	 */
	for (loop = 0; loop < 16; loop++) {
		TransmitData[loop] = (TROM[loop*4]<<1) +
				     (TROM[loop*4+1]<<3) +
				     (TROM[loop*4+2]<<5) +
				     (TROM[loop*4+3]<<7);
	}

	/* Convert the individual recovered ROM bits into an 8 byte format */
	for (loop = 0; loop < 8; loop++) {
		ROMCode[loop] = (RROM[loop*8]) +
				(RROM[loop*8+1]<<1) +
				(RROM[loop*8+2]<<2) +
				(RROM[loop*8+3]<<3) +
				(RROM[loop*8+4]<<4) +
				(RROM[loop*8+5]<<5) +
				(RROM[loop*8+6]<<6) +
				(RROM[loop*8+7]<<7);
	}

	if (result == -1) {
		/*
		 * There are no DIS bits that haven't been flipped
		 * Tell the loop the seach is over
		 */
		 return 1;
	} else {
		/* otherwise, continue */
		return 0;
	}
}

/*
 * OMAP HDQ controller functions
 */
static int hdq_reset(struct omap_hdq_data *omap_hdq_data)
{
	int ret;
	unsigned long timeleft;

	hdq_reg_out(omap_hdq_data, HDQ_SYSCONFIG, HDQ_SYSCONFIG_SOFTRESET);

	/*
	 * Select the mode & enable clocks.
	 * It is observed that INT flags can't be cleared via a read and GO/INIT
	 * won't return to zero if interrupt is disabled. So we always enable
	 * interrupt.
	 */
#ifdef CONFIG_OMAP3430_1WIRE_PROTOCOL
	/* Select 1Wire mode & enable clocks */
	hdq_reg_out(omap_hdq_data,
		    HDQ_CTRL_STATUS,
		    HDQ_CTRL_STATUS_CLOCKENABLE |
		    HDQ_CTRL_STATUS_MODE |
		    HDQ_CTRL_STATUS_INTERRUPTMASK);
#else
	/* Select HDQ mode & enable clocks */
	hdq_reg_out(omap_hdq_data,
		    HDQ_CTRL_STATUS,
		    HDQ_CTRL_STATUS_CLOCKENABLE |
		    HDQ_CTRL_STATUS_INTERRUPTMASK);
#endif

	/* wait for reset to complete */
	if ((ret = hdq_wait_for_flag(omap_hdq_data,
				     HDQ_SYSSTATUS,
				     HDQ_SYSSTATUS_RESETDONE,
				     HDQ_FLAG_SET))) {
		DPRINTK("%s: timeout waiting for HDQ reset_done bit\n",
			__FUNCTION__);
		return ret;
	}

	/* set the clock to auto-idle */
	hdq_reg_out(omap_hdq_data, HDQ_SYSCONFIG, HDQ_SYSCONFIG_AUTOIDLE);

	/* clear interrupt flags via a dummy read */
	hdq_reg_in(omap_hdq_data, HDQ_INT_STATUS);

	INIT_COMPLETION(omap_hdq_data->reset_done);

	/* set the INIT and GO bit */
	hdq_reg_merge(omap_hdq_data,
		      HDQ_CTRL_STATUS,
		      HDQ_CTRL_STATUS_INITIALIZATION | HDQ_CTRL_STATUS_GO,
		      HDQ_CTRL_STATUS_INITIALIZATION | HDQ_CTRL_STATUS_GO);

	/* checking for presence pulse */
	timeleft = wait_for_completion_timeout(&omap_hdq_data->reset_done,
								HDQ_TIMEOUT);
	if (!timeleft) {
		printk(KERN_ERR MOD_NAME
			"%s: Slave presence is not detected\n", __FUNCTION__);
		return -ETIMEDOUT;
	}

	/* If the slave indicates it is present, it set PRESENCEDETECT
	 * bit. Check if present pulse is received
	 */

	ret = hdq_wait_for_flag(omap_hdq_data,
				HDQ_CTRL_STATUS,
				HDQ_CTRL_STATUS_PRESENCEDETECT,
				HDQ_FLAG_SET);
	if (ret) {
		DPRINTK("%s: Present pulse is not received \n", __FUNCTION__);
		return ret;
	}
	/* wait for both INIT and GO bits rerurn to zero.
	 * zero wait time expected for interrupt mode.
	 */
	ret = hdq_wait_for_flag(omap_hdq_data,
				HDQ_CTRL_STATUS,
				HDQ_CTRL_STATUS_INITIALIZATION |
				HDQ_CTRL_STATUS_GO,
				HDQ_FLAG_CLEAR);
	if (ret)
		printk(KERN_ERR MOD_NAME
			"%s: timeout-INIT&GO bits are not zero\n", __FUNCTION__);

	return ret;
}

static void hdq_write_byte(void *data, u8 val)
{
	struct omap_hdq_data *omap_hdq_data = data;
	int ret;
	unsigned long timeleft;

	/* clear interrupt flags via a dummy read */
	hdq_reg_in(omap_hdq_data, HDQ_INT_STATUS);

#ifdef CONFIG_OMAP3430_1WIRE_PROTOCOL
	hdq_reg_out(omap_hdq_data,
		    HDQ_CTRL_STATUS,
		    HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_CLOCKENABLE |
		    HDQ_CTRL_STATUS_MODE | HDQ_CTRL_STATUS_INTERRUPTMASK);
#endif

	hdq_reg_out(omap_hdq_data, HDQ_TX_DATA, val);

	INIT_COMPLETION(omap_hdq_data->write_done);

	/* set the GO bit */
	hdq_reg_merge(omap_hdq_data,
		HDQ_CTRL_STATUS,
		HDQ_CTRL_STATUS_GO,
		HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_GO);

	/* wait for the TXCOMPLETE bit */
	timeleft = wait_for_completion_timeout(&omap_hdq_data->write_done,
								HDQ_TIMEOUT);
	if (!timeleft) {
		printk(KERN_ERR MOD_NAME
			"%s: timeout waiting for TXCOMPLETE\n", __FUNCTION__);
		return;
	}

	/* wait for the GO bit return to zero */
	ret = hdq_wait_for_flag(omap_hdq_data,
				HDQ_CTRL_STATUS,
				HDQ_CTRL_STATUS_GO,
				HDQ_FLAG_CLEAR);
	if (ret) {
		DPRINTK("%s: timeout waiting for GO bit to zero\n",
			__FUNCTION__);
	}
}

static u8 hdq_read_byte(void *data)
{
	struct omap_hdq_data *omap_hdq_data = data;
	unsigned long timeleft;

	/* clear interrupt flags via a dummy read */
	hdq_reg_in(omap_hdq_data, HDQ_INT_STATUS);

#ifdef CONFIG_OMAP3430_1WIRE_PROTOCOL

	/* setup Tx output for the 1-wire read command*/
	hdq_reg_out(omap_hdq_data, HDQ_TX_DATA, 0xff);
#else
	/* this byte is the mix of reg offset and READ command */
	hdq_write_byte(omap_hdq_data, HDQ_CMD_READ);
#endif

	INIT_COMPLETION(omap_hdq_data->read_done);

	/* Start the read */
	hdq_reg_merge(omap_hdq_data,
			HDQ_CTRL_STATUS,
			HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_GO,
			HDQ_CTRL_STATUS_DIR | HDQ_CTRL_STATUS_GO);


	/* wait for RXCOMPLETE */
	timeleft = wait_for_completion_timeout(&omap_hdq_data->read_done,
								HDQ_TIMEOUT);
	if (!timeleft) {
		printk(KERN_ERR MOD_NAME
			"%s: timeout waiting for RXCOMPLETE\n", __FUNCTION__);
	}

	/* the data is ready. Read it in! */
	return hdq_reg_in(omap_hdq_data, HDQ_RX_DATA);
}

static void hdq_net_address_search(void *data, u8 seachType,
			w1_slave_found_callback slave_found_callback)
{
	struct omap_hdq_data *omap_hdq_data = data;
	int ret;
	int result = 0, max_code_rom = 0;
	int TransmitData[16];
	int ReceiveData[16];
	int ROM_Code[ROM_CODE_SIZE];
	u64 rn;

	/* Send INIT pulse and wait for a presence detect interrupt */
	ret = hdq_reset(omap_hdq_data);
	if (ret) {
		DPRINTK("%s: no Devices Found. \n", __FUNCTION__);
		return;
	}

	/* Initialize RecoverROM */
	hdq_recoverROM(omap_hdq_data, NULL, TransmitData, NULL);

	/* Loop up to 256 times */
	while ((!result) && (max_code_rom < 256)) {

		max_code_rom++;
		/* Reset, send INIT pulse and wait for a presence
		 * detect interrupt
		 */
		ret = hdq_reset(omap_hdq_data);
		if (ret) {
			printk(KERN_ERR MOD_NAME
				"%s: no Devices Found in loop \n",
				__FUNCTION__);
			continue;
		}
		/* Send search net address command */
		hdq_write_byte(omap_hdq_data,
					SEARCH_NET_ADDRESS_CMD);
		if (ret) {
			printk(KERN_ERR MOD_NAME
				"%s: send net address command failed\n",
				__FUNCTION__);
		}

		hdq_searchROMAccelerator(omap_hdq_data,
					TransmitData,
					ReceiveData);

		result = hdq_recoverROM(omap_hdq_data,
					ReceiveData,
					TransmitData,
					ROM_Code);

		/* register each "slave" that we find */
		rn = ((u64)ROM_Code[0] & 0xff) |
		     (((u64)ROM_Code[1] & 0xff) << 8) |
		     (((u64)ROM_Code[2] & 0xff) << 16) |
		     (((u64)ROM_Code[3] & 0xff) << 24) |
		     (((u64)ROM_Code[4] & 0xff) << 32) |
		     (((u64)ROM_Code[5] & 0xff) << 40) |
		     (((u64)ROM_Code[6] & 0xff) << 48) |
		     (((u64)ROM_Code[7] & 0xff) << 56);

		DPRINTK("Found device, addr = %02x %02x %02x %02x "
					"%02x %02x %02x %02x\n",
				ROM_Code[0], ROM_Code[1],
				ROM_Code[2], ROM_Code[3],
				ROM_Code[4], ROM_Code[5],
				ROM_Code[6], ROM_Code[7]);

		slave_found_callback(data, rn);
	}
}

/*
 * NOTE: This function must be called with hdq_timer disabled!
 */
static int hdq_up(struct omap_hdq_data *omap_hdq_data)
{
	if (omap_hdq_data->hdq_is_up) {
		DPRINTK("%s: hdq already up\n", __FUNCTION__);
		return 0;
	}

	/* enable the clocks */
	if (clk_enable(omap_hdq_data->hdq_ick)) {
		printk(KERN_ERR MOD_NAME
			"%s: Can not enable ick\n", __FUNCTION__);
		return -ENODEV;
	}

	if (clk_enable(omap_hdq_data->hdq_fck)) {
		printk(KERN_ERR MOD_NAME
			"%s: Can not enable fck\n", __FUNCTION__);
		return -ENODEV;
	}

	omap_hdq_data->hdq_is_up = 1;
	DPRINTK("%s: Starting hdq\n", __FUNCTION__);

	return 0;	/* returns 0 on success */
}

/*
 * NOTE: This function must be called with hdq_timer disabled!
 */
static void hdq_down(struct omap_hdq_data *omap_hdq_data)
{
	if (!omap_hdq_data->hdq_is_up) {
		DPRINTK("%s: hdq already down\n", __FUNCTION__);
		return;
	}

	/* Turn off the functional clock to the HDQ state machine. */
	hdq_reg_out(omap_hdq_data, HDQ_CTRL_STATUS,
		    hdq_reg_in(omap_hdq_data, HDQ_CTRL_STATUS) &
					~(HDQ_CTRL_STATUS_CLOCKENABLE));

	clk_disable(omap_hdq_data->hdq_ick);
	clk_disable(omap_hdq_data->hdq_fck);

	omap_hdq_data->hdq_is_up = 0;
	DPRINTK("%s: Shutting down hdq\n", __FUNCTION__);
}

/*
 * hdq timer callback function to shutdown the hdq
 */
static void hdq_timer_callback(unsigned long data)
{
	/* timer has expired - turn off the hdq */

	DPRINTK("Timer Expired...\n");
	hdq_down((struct omap_hdq_data*)data);
}

static u8 omap_hdq_reset_bus(void *data)
{
	struct omap_hdq_data *omap_hdq_data = data;
	int ret;

	/* make sure that the timer is not running */
	if (del_timer_sync(&omap_hdq_data->hdq_timer)) {
		DPRINTK("%s deleted Active timer\n", __FUNCTION__);
	}

	/* make sure that the hdq is up before we proceed */
	if (0 != hdq_up(omap_hdq_data)) {
		printk(KERN_ERR MOD_NAME "%s: hdq_up failed\n", __FUNCTION__);
		return W1_RESET_BUS_ERROR;
	}

	ret = hdq_reset(omap_hdq_data);

	/* start the countdown timer */
	mod_timer(&omap_hdq_data->hdq_timer,
		  jiffies + omap_hdq_data->hdq_timeout);

	if (ret == 0)
		return W1_RESET_BUS_DEVICE_PRESENT;
	else if (ret == -ETIMEDOUT)
		return W1_RESET_BUS_NO_DEVICE_PRESENT; 	/* no slaves present */
	else
		return W1_RESET_BUS_ERROR;	/* error condition */

}

static void omap_hdq_write_byte(void *data, u8 val)
{
	struct omap_hdq_data *omap_hdq_data = data;

	/* make sure that the timer is not running */
	if (del_timer_sync(&omap_hdq_data->hdq_timer)) {
		DPRINTK("%s deleted Active timer\n", __FUNCTION__);
	}

	/* make sure that the hdq is up before we proceed */
	if (0 != hdq_up(omap_hdq_data)) {
		printk(KERN_ERR MOD_NAME "%s: hdq_up failed\n", __FUNCTION__);
		return;
	}

	hdq_write_byte(omap_hdq_data, val);

	/* start the countdown timer */
	DPRINTK("%s: Setting timeout to %u jiffies\n",
			__FUNCTION__, omap_hdq_data->hdq_timeout);
	mod_timer(&omap_hdq_data->hdq_timer,
		  jiffies + omap_hdq_data->hdq_timeout);
}

static u8 omap_hdq_read_byte(void *data)
{
	struct omap_hdq_data *omap_hdq_data = data;
	u8 val;

	/* make sure that the timer is not running */
	if (del_timer_sync(&omap_hdq_data->hdq_timer)) {
		DPRINTK("%s deleted Active timer\n", __FUNCTION__);
	}

	/* make sure that the hdq is up before we proceed */
	if (0 != hdq_up(omap_hdq_data)) {
		printk(KERN_ERR MOD_NAME "%s: hdq_up failed\n", __FUNCTION__);
		return 0;
	}

	val = hdq_read_byte(omap_hdq_data);

	/* start the countdown timer */
	mod_timer(&omap_hdq_data->hdq_timer,
		  jiffies + omap_hdq_data->hdq_timeout);

	return val;

}

static void omap_hdq_net_address_search(void *data, u8 searchType,
				w1_slave_found_callback slave_found_callback)
{
	struct omap_hdq_data *omap_hdq_data = data;

	/*
	 * delete the timer so that we don't get interrupted
	 * in the middle of hdq_reset operation
	 */
	if (del_timer_sync(&omap_hdq_data->hdq_timer)) {
		DPRINTK("%s deleted Active timer\n", __FUNCTION__);
	}

	/* make sure that the hdq is up before we proceed */
	if (0 != hdq_up(omap_hdq_data)) {
		printk(KERN_ERR MOD_NAME "%s: hdq_up failed\n", __FUNCTION__);
		return;
	}

	hdq_net_address_search(omap_hdq_data, searchType,
				slave_found_callback);

	/* start the countdown timer */
	mod_timer(&omap_hdq_data->hdq_timer,
		  jiffies + omap_hdq_data->hdq_timeout);
}

/*
 * Driver functions
 */
static struct w1_bus_master omap_hdq_master = {
	.write_byte		= omap_hdq_write_byte,
	.read_byte		= omap_hdq_read_byte,
 	.reset_bus		= omap_hdq_reset_bus,
	.search			= omap_hdq_net_address_search,
};

static int __init omap_hdq_probe(struct platform_device *pdev)
{
	int ret;
	u8  rev;
	u32 timeout;
	struct omap_hdq_data *omap_hdq_data;
	struct resource *res;

	DPRINTK("Starting hdq_probe\n");

	omap_hdq_data  = kzalloc(sizeof(*omap_hdq_data), GFP_KERNEL);
	if (!omap_hdq_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, omap_hdq_data);

	/* get the clock shutdown timeout value */
	if (pdev->dev.platform_data) {
		timeout = *(u32 *) pdev->dev.platform_data;
		if (timeout > 2000) {
			printk(KERN_WARNING MOD_NAME
				"Platform data HDQ timeout value (%ums) does "
				"not make sense. Using default 100ms.\n", timeout);
			timeout = 100;
		}
	} else {
		printk(KERN_WARNING MOD_NAME
			"No HDQ clock timeout defined. Using default 100ms\n");
		timeout = 100;
	}
	omap_hdq_data->hdq_timeout = msecs_to_jiffies(timeout);

	/* get the hdq base addr from the board file */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENXIO;
		goto err0;
	}
	omap_hdq_data->hdq_base_addr = IO_ADDRESS(res->start);

	/* get the hdq irq number from the board file */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		ret = -ENXIO;
		goto err0;
	}
	omap_hdq_data->hdq_irq = res->start;
	omap_hdq_data->pdev    = pdev;

	/* start the hdq controller initialization sequence */
	DPRINTK("setting up hdq clocks\n");

	omap_hdq_data->hdq_ick = clk_get(&pdev->dev, "hdq_ick");
	if (IS_ERR(omap_hdq_data->hdq_ick)) {
		printk(KERN_ERR MOD_NAME
			"%s: Can't get HDQ ick clock objects\n", __FUNCTION__);
		ret = PTR_ERR(omap_hdq_data->hdq_ick);
		goto err0;
	}

	omap_hdq_data->hdq_fck = clk_get(&pdev->dev, "hdq_fck");
	if (IS_ERR(omap_hdq_data->hdq_fck)) {
		printk(KERN_ERR MOD_NAME
			"%s: Can't get HDQ fck clock objects\n", __FUNCTION__);
		ret = PTR_ERR(omap_hdq_data->hdq_fck);
		goto err1;
	}

	/* enabling the iclk so that we can read the hdq revision */
	if (clk_enable(omap_hdq_data->hdq_ick)) {
		printk(KERN_ERR MOD_NAME
			"%s: Can't enable ick\n", __FUNCTION__);
		ret = -ENODEV;
		goto err2;
	}

	rev = hdq_reg_in(omap_hdq_data, HDQ_REVISION);

	/* don't clock the HDQ until it is needed */
	clk_disable(omap_hdq_data->hdq_ick);

	init_completion(&omap_hdq_data->reset_done);
	init_completion(&omap_hdq_data->read_done);
	init_completion(&omap_hdq_data->write_done);

	if (request_irq(omap_hdq_data->hdq_irq, hdq_isr, IRQF_DISABLED,
						"OMAP HDQ", omap_hdq_data)) {

		printk(KERN_ERR MOD_NAME "request_irq failed\n");
		ret = -ENODEV;
		goto err2;
	}

	DPRINTK("Adding omap hdq master device\n");
	omap_hdq_master.data = (void *) omap_hdq_data;

	ret = w1_add_master_device(&omap_hdq_master);
	if (ret) {
		printk(KERN_ERR MOD_NAME
			"%s: failed to add master device\n", __FUNCTION__);
		goto err3;
	}

	/* Set up clock shutdown timer */
	omap_hdq_data->hdq_timer.function = hdq_timer_callback;
	omap_hdq_data->hdq_timer.data     = (unsigned long)omap_hdq_data;
	init_timer(&omap_hdq_data->hdq_timer);

	/* turn on HDQ for initial battery discovery */
	ret = hdq_up(omap_hdq_data);
	if (ret != 0) {
		printk(KERN_ERR MOD_NAME
			"%s: omap hdq up failed\n", __FUNCTION__);
		goto err3;
	}

	ret = hdq_reset(omap_hdq_data);
	if ((ret != 0) && (ret != -ETIMEDOUT)) {
		printk(KERN_ERR MOD_NAME
			"%s: omap hdq initialization failed\n", __FUNCTION__);
		goto err4;
	}

	mod_timer(&omap_hdq_data->hdq_timer,
		  jiffies + omap_hdq_data->hdq_timeout);

	printk(KERN_INFO MOD_NAME
		"OMAP HDQ Hardware Revision %c.%c. Clock shutdown timeout: %ums\n",
				(rev >> 4) + '0', (rev & 0x0f) + '0', timeout);

	return 0;

err4:
	hdq_down(omap_hdq_data);
err3:
	free_irq(omap_hdq_data->hdq_irq, omap_hdq_data);
err2:
	clk_put(omap_hdq_data->hdq_fck);
err1:
	clk_put(omap_hdq_data->hdq_ick);
err0:
	kfree(omap_hdq_data);
	return ret;
}

static int omap_hdq_remove(struct platform_device *pdev)
{
	struct omap_hdq_data *omap_hdq_data = platform_get_drvdata(pdev);

	w1_remove_master_device(&omap_hdq_master);

	if (del_timer_sync(&omap_hdq_data->hdq_timer)) {
		DPRINTK("%s: Active timer deleted\n", __FUNCTION__);
	}

	hdq_down(omap_hdq_data);

	free_irq(omap_hdq_data->hdq_irq, omap_hdq_data);

	/* remove module dependency */
	clk_put(omap_hdq_data->hdq_ick);
	clk_put(omap_hdq_data->hdq_fck);

	kfree(omap_hdq_data);
	return 0;
}

#ifdef CONFIG_PM
static int omap_hdq_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct omap_hdq_data *omap_hdq_data = platform_get_drvdata(pdev);

	if (del_timer_sync(&omap_hdq_data->hdq_timer)) {
		DPRINTK("%s: Active timer deleted\n", __FUNCTION__);
	}

	hdq_down(omap_hdq_data);
	return 0;
}

static int omap_hdq_resume(struct platform_device *pdev)
{
	/*
	 * NOTE:  nothing needs to be done here - we don't turn anything on
	 *        until we actually need to read some data from the 1-wire
	 *        device.  The "turning on" of the clocks will be handled by
	 *        individual omap_hdq_xxxxx functions.
	 */
	return 0;
}
#else
#define omap_hdq_suspend	NULL
#define omap_hdq_resume		NULL
#endif

static struct platform_driver omap_hdq_driver = {
	.probe		= omap_hdq_probe,
	.remove		= omap_hdq_remove,
	.suspend	= omap_hdq_suspend,
	.resume		= omap_hdq_resume,
	.driver = {
		.name = "omap_hdq",
	},
};

static int __init omap_w1_init(void)
{
	DPRINTK("OMAP_W1: Registering HDQ controller 1\n");

	if (platform_driver_register(&omap_hdq_driver)) {
		printk(KERN_ERR MOD_NAME
			"failed to register HDQ Controller\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit omap_w1_exit(void)
{
	DPRINTK("OMAP_W1: Unregistering HDQ Controller\n");

	platform_driver_unregister(&omap_hdq_driver);
}

module_init(omap_w1_init);
module_exit(omap_w1_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("John Chen, Palm Inc.");
MODULE_DESCRIPTION("Master Driver for the TI OMAP 1-Wire interface.");
