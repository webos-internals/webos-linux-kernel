/*
 * drivers/media/video/omap/isp/omap_resizer.c
 *
 * Wrapper for Resizer module in TI's OMAP3430 ISP
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/io.h>
#include <asm/scatterlist.h>

#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "ispresizer.h"
#include "omap_resizer.h"

#define OMAP_REZR_NAME		"omap-resizer"

struct device_params device_config;
/* For registeration of	charatcer device*/
static struct cdev c_dev;
/* device structure	to make	entry in device*/
static dev_t dev;
/* for holding device entry*/
struct device *rsz_device = NULL;

static struct rsz_mult multipass;

/* inline function to free reserver pages  */
void inline rsz_free_pages(unsigned long addr, unsigned long bufsize)
{
	unsigned long size;
	unsigned long tempaddr;
	tempaddr = addr;
	if (!addr)
		return;
	size = PAGE_SIZE << (get_order(bufsize));
	while (size > 0) {
		ClearPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	free_pages(tempaddr, get_order(bufsize));
}

/*
Function to	allocate memory	to input and output	buffers
*/
int malloc_buff(struct rsz_reqbufs *reqbuff,
		struct channel_config *rsz_conf_chan)
{
	/* For looping purpose */
	int buf_index = ZERO;
	/* For calculating the difference between new buffers to be allocated */
	int diff;
	/* for pointing to input or output buffer pointer */
	int *buf_ptr, *buf_start;
	/* To calculate no of max buffers; */
	int maxbuffers;
	/* to calculate number of buffers allocated */
	unsigned int numbuffers = ZERO;
	/* for storing buffer size */
	int *buf_size;
	/* to free the number of allocared buffers */
	int free_index;
	/* to make sure buffer pointer never swapped */
	unsigned long adr;
	unsigned long size;

	/* assigning the buf_ptr to input buffer which is array of void
	   pointer */
	if (reqbuff->buf_type == RSZ_BUF_IN) {
		buf_ptr = (int *)rsz_conf_chan->input_buffer;
		buf_size = &rsz_conf_chan->in_bufsize;
		maxbuffers = MAX_INPUT_BUFFERS;
	} else if (reqbuff->buf_type == RSZ_BUF_OUT) {
		buf_ptr = (int *)rsz_conf_chan->output_buffer;
		buf_size = &rsz_conf_chan->out_bufsize;
		maxbuffers = MAX_OUTPUT_BUFFERS;
	} else {
		printk("Invalid type \n");
		return -EINVAL;
	}

	/* Check the request for number of buffers */
	if (reqbuff->count > maxbuffers)
		return -EINVAL;

	/* Counting the number of  buffers allocated */
	buf_start = buf_ptr;
	while ((*(buf_ptr) != (int)NULL) && (numbuffers < maxbuffers)) {
		numbuffers++;
		buf_ptr++;
	}

	buf_ptr = buf_start;

	/* Free all     the     buffers if the count is zero */
	if (reqbuff->count == FREE_BUFFER) {
		/* Free all     the     buffers */
		for (buf_index = ZERO; buf_index < numbuffers; buf_index++) {
			/* free memory allocate for the image */

			/* Free buffers using free_pages */
			rsz_free_pages(*buf_ptr, *buf_size);

			/* assign buffer zero to indicate its free */
			*buf_ptr = (int)NULL;
			buf_ptr++;
		}
		return SUCESS;
	}

	/* If buffers are previously allocated, size has to be same */
	if (numbuffers) {

		if (reqbuff->size != *buf_size) {
			for (buf_index = ZERO; buf_index < numbuffers;
			     buf_index++) {
				/* free memory allocate for the image */
				/* Free buffers using free_pages */
				rsz_free_pages(*buf_ptr, *buf_size);

				/* assign buffer zero to indicate its free */
				*buf_ptr = (int)NULL;
				buf_ptr++;
			}
			numbuffers = ZERO;
			buf_ptr = buf_start;
		}

	}

	/* get the difference to know how mnay buffers to allocate */
	diff = numbuffers - reqbuff->count;
	if (diff > ZERO) {
		buf_ptr = buf_ptr + reqbuff->count;
		for (buf_index = reqbuff->count; buf_index < numbuffers;
		     buf_index++) {
			/* if difference is positive than deallocate that
			   much memory of input buff */

			/* free buffer using free_pages */
			rsz_free_pages(*buf_ptr, *buf_size);

			/* assign buffer zero to indicate its free */
			*buf_ptr = (int)NULL;
			buf_ptr++;
		}
	} else {
		/* make the difference positive */
		diff = reqbuff->count - numbuffers;
		buf_ptr = buf_ptr + numbuffers;
		for (buf_index = numbuffers; buf_index < reqbuff->count;
		     buf_index++) {

			/* assign memory to buffer */
			*buf_ptr =
			    (int)(__get_free_pages
				  (GFP_KERNEL | GFP_DMA,
				   get_order(reqbuff->size)));

			if (!(*buf_ptr)) {

				buf_ptr = (buf_ptr - (buf_index - numbuffers));

				for (free_index = numbuffers;
				     free_index < buf_index; free_index++) {

					rsz_free_pages(*buf_ptr, *buf_size);
					buf_ptr++;

				}
				printk("requestbuffer: not enough memory");
				return -ENOMEM;
			}

			adr = *buf_ptr;
			size = PAGE_SIZE << (get_order(reqbuff->size));
			while (size > 0) {
				/* make sure the frame buffers
				   are never swapped out of     memory */
				SetPageReserved(virt_to_page(adr));
				adr += PAGE_SIZE;
				size -= PAGE_SIZE;
			}
			buf_ptr++;
		}
	}

	/* set the buffer size to requested size */
	/* this will be useful only when numbuffers = 0 */
	*buf_size = reqbuff->size;

	return SUCESS;
}				/*     end     of function     Main_buff */

/* Function to query the  physical address of the buffer  requested by index*/

int get_buf_address(struct rsz_buffer *buffer,
		    struct channel_config *rsz_conf_chan)
{
	int buffer_index = 0;

	if (buffer == NULL)
		return -EINVAL;

	if (buffer->buf_type == RSZ_BUF_IN) {

		/* Check the request for number of input buffers */
		if (buffer->index > MAX_INPUT_BUFFERS)
			return -EINVAL;
		/*count number of input buffer allocated */
		while ((rsz_conf_chan->input_buffer[buffer_index] != NULL)
		       && (buffer_index < MAX_INPUT_BUFFERS)) {
			buffer_index++;
		}
		/*checking the index requested */
		if (buffer->index >= buffer_index) {
			printk("Requested buffer not allocated \n");
			return -EINVAL;
		}

		/* assignning the  input address to offset which will be
		   used in mmap */
		buffer->offset =
		    (int)(rsz_conf_chan->input_buffer[buffer->index]);
		buffer->size = rsz_conf_chan->in_bufsize;
	}

	else if (buffer->buf_type == RSZ_BUF_OUT) {

		/* Check the request for number of output buffers */
		if (buffer->index > MAX_OUTPUT_BUFFERS)
			return -EINVAL;

		/* counting     number of output buffers */
		while ((rsz_conf_chan->output_buffer[buffer_index] != NULL)
		       && (buffer_index < MAX_OUTPUT_BUFFERS)) {
			buffer_index++;
		}
		/* checking the index requested */
		if (buffer->index >= buffer_index) {
			printk("Requested buffer not allocated \n");
			return -EINVAL;
		}

		/* assignning the output address to offset which will be
		   used in mmap */
		buffer->offset =
		    ((int)(rsz_conf_chan->output_buffer[buffer->index]));
		buffer->size = rsz_conf_chan->out_bufsize;

	} else {
		printk("	Invalid	input type! \n");
		return -EINVAL;
	}

	/* look up physical     address of the buffer */
	buffer->offset = virt_to_phys((void *)buffer->offset);

	return SUCESS;

}

/*
 Function to set hardware configuration registers
*/

void rsz_hardware_setup(struct channel_config *rsz_conf_chan)
{
	/* Counter to set the value     of horizonatl and vertical coeff */
	int coeffcounter;
	/* for getting the coefficient offset */
	int coeffoffset = ZERO;

	/* setting the hardware register rszcnt */
	omap_writel(rsz_conf_chan->register_config.rsz_cnt, ISPRSZ_CNT);

	/* setting the hardware register instart */
	omap_writel(rsz_conf_chan->register_config.rsz_in_start,
		    ISPRSZ_IN_START);
	/* setting the hardware register insize */
	omap_writel(rsz_conf_chan->register_config.rsz_in_size, ISPRSZ_IN_SIZE);

	/* setting the hardware register outsize */
	omap_writel(rsz_conf_chan->register_config.rsz_out_size,
		    ISPRSZ_OUT_SIZE);
	/* setting the hardware register inaddress */
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_inadd,
		    ISPRSZ_SDR_INADD);
	/* setting the hardware register in     offset */
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_inoff,
		    ISPRSZ_SDR_INOFF);
	/* setting the hardware register out address */
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_outadd,
		    ISPRSZ_SDR_OUTADD);
	/* setting the hardware register out offset */
	omap_writel(rsz_conf_chan->register_config.rsz_sdr_outoff,
		    ISPRSZ_SDR_OUTOFF);
	/* setting the hardware register yehn */
	omap_writel(rsz_conf_chan->register_config.rsz_yehn, ISPRSZ_YENH);

	/* setting the hardware registers     of coefficients */
	for (coeffcounter = ZERO; coeffcounter < MAX_COEF_COUNTER;
	     coeffcounter++) {
		omap_writel(rsz_conf_chan->register_config.
			    rsz_coeff_horz[coeffcounter],
			    ((ISPRSZ_HFILT10 + coeffoffset)));

		omap_writel(rsz_conf_chan->register_config.
			    rsz_coeff_vert[coeffcounter],
			    ((ISPRSZ_VFILT10 + coeffoffset)));
		coeffoffset = coeffoffset + COEFF_ADDRESS_OFFSET;
	}
}

/*
 This function enable the resize bit after doing the hardware register
 configuration after which resizing	will be	carried	on.
*/
int rsz_start(struct rsz_resize *resize, struct channel_config *rsz_conf_chan)
{

	/* Holds the input address to the resizer */
	int in_address;
	/* Holds the output     address to resizer */
	int out_address;

	unsigned long in_addr_mmu, out_addr_mmu = 0;
	/* Conatains the input put and output buffer allocated size */
	int out_bufsize, in_bufsize;

	/* Conatins the pitch and vertical size of input and output image */
	int in_vsize, in_pitch, out_vsize, out_pitch;
	/* holds the return value; */
	int ret;
	/* For calculating the number of input buffers allocated */
	int buffer_in_index = ZERO;

	/* For calculating the number of output buffers allocated */
	int buffer_out_index = ZERO;

	/* checking     the     configuartion status */
	if (rsz_conf_chan->config_state) {
		printk("State not configured \n");
		return -EINVAL;
	}

	/* Taking the inpitch of the image */
	in_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_inoff
	    & ISPRSZ_SDR_INOFF_OFFSET_MASK;
	/* Taking the out pitch of image */
	in_vsize =
	    ((rsz_conf_chan->register_config.rsz_in_size
	      & ISPRSZ_IN_SIZE_VERT_MASK) >> ISPRSZ_IN_SIZE_VERT_SHIFT);

	in_bufsize = in_vsize * in_pitch;

	/*getting the outpitch */
	out_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_outoff
	    & ISPRSZ_SDR_OUTOFF_OFFSET_MASK;
	/* getting the vertical size  */
	out_vsize =
	    ((rsz_conf_chan->register_config.rsz_out_size
	      & ISPRSZ_OUT_SIZE_VERT_MASK) >> ISPRSZ_OUT_SIZE_VERT_SHIFT);

	out_bufsize = out_vsize * out_pitch;

	if (resize->in_buf.index < ZERO) {
		/* assignning the address to the register configuration */
		if (resize->in_buf.size >= in_bufsize) {
			if (resize->in_buf.offset % 32)
				return -EINVAL;

			in_addr_mmu =
			    ispmmu_map(resize->in_buf.offset, in_bufsize);

		} else {
			printk(" invalid size \n");
			return -EINVAL;
		}
	} else {
		if (resize->in_buf.index > MAX_INPUT_BUFFERS)
			return -EINVAL;
		/*count number of input buffer allocated */
		while ((rsz_conf_chan->input_buffer[buffer_in_index] !=
			NULL) && (buffer_in_index < MAX_INPUT_BUFFERS)) {
			buffer_in_index++;
		}
		/*checking the index requested */
		if (resize->in_buf.index >= buffer_in_index) {
			printk("Requested buffer not allocated \n");
			return -EINVAL;
		}

		in_address = virt_to_phys(((void *)
					   rsz_conf_chan->
					   input_buffer[resize->in_buf.index]));
		in_addr_mmu = ispmmu_map(in_address, in_bufsize);

	}
	if(multipass.out_hsize > multipass.in_hsize){
	if (resize->out_buf.index < ZERO) {
		if (resize->out_buf.size >= out_bufsize) {
			if (resize->out_buf.offset % 32)
				return -EINVAL;

			out_addr_mmu =
			    ispmmu_map(resize->out_buf.offset, out_bufsize);

		} else {
			printk("Invalid	output size \n");
			return -EINVAL;
		}
	} else {
		if (resize->out_buf.index > MAX_OUTPUT_BUFFERS)
			return -EINVAL;
		/*count number of input buffer allocated */
		while ((rsz_conf_chan->output_buffer[buffer_out_index] !=
			NULL) && (buffer_out_index < MAX_OUTPUT_BUFFERS)) {
			buffer_out_index++;
		}
		/*checking the index requested */
		if (resize->out_buf.index >= buffer_out_index) {
			printk("Requested buffer not allocated \n");
			return -EINVAL;
		}
		out_address = virt_to_phys(((void *)
					    (rsz_conf_chan->
					     output_buffer[resize->out_buf.
							   index])));
		out_addr_mmu = ispmmu_map(out_address, out_bufsize);
	}
	}
	rsz_conf_chan->register_config.rsz_sdr_inadd = in_addr_mmu;
	rsz_conf_chan->register_config.rsz_sdr_outadd = out_addr_mmu;
	if(multipass.out_hsize < multipass.in_hsize){
	rsz_conf_chan->register_config.rsz_sdr_outadd = in_addr_mmu;
	}
	/* Channel is busy */
	rsz_conf_chan->status = CHANNEL_BUSY;

	/* Function call to add the entry of application in array */
	ret = add_to_array(rsz_conf_chan);

	/*Function call to set up the hardware */
	rsz_hardware_setup(rsz_conf_chan);

	if (isp_set_callback(CBK_RESZ_DONE, rsz_isr, (void *)NULL, (void *)NULL)) {
	printk(KERN_ERR "No callback for RSZR\n");
	return -EINVAL;
	}
mult:
	/* Initialize the interrupt ISR to ZER0 */
	device_config.sem_isr.done = ZERO;

	/*Function call to enable resizer hardware */
	ispresizer_enable(1);

	/* Waiting for resizing to be complete */
	wait_for_completion_interruptible(&(device_config.sem_isr));

	if(multipass.active)
	{
		rsz_set_multipass(rsz_conf_chan);
		goto mult;
	}
	rsz_conf_chan->status = CHANNEL_FREE;

	delete_from_array(rsz_conf_chan);

	isp_unset_callback(CBK_RESZ_DONE);

	return ret;

}

/*
 Function to add the current channel configuration into	array
according to priority.
*/
int add_to_array(struct channel_config *rsz_conf_chan)
{
	int array_index, device_index;

	/* locking the configuartion aaray */
	down_interruptible(&device_config.array_sem);

	/* Add configuration to the     queue according to its priority */
	if (device_config.array_count == EMPTY) {
		/* If array     empty insert at top     position */
		device_config.channel_configuration[device_config.array_count]
		    = rsz_conf_chan;
	} else {
		/* Check the priority and insert according to the priority */
		/* it will start from first     index */
		for (array_index = SECONDENTRY;
		     array_index < device_config.array_count; array_index++) {
			if (device_config.
			    channel_configuration[array_index]->priority <
			    rsz_conf_chan->priority)
				break;
		}
		/* Shift all the elements one step down in array */
		/* IF firstelement and second have same prioroty than insert */
		/* below first */
		for (device_index = device_config.array_count;
		     device_index > array_index; device_index--) {
			device_config.channel_configuration[device_index] =
			    device_config.
			    channel_configuration[device_index - NEXT];
		}

		device_config.channel_configuration[array_index] =
		    rsz_conf_chan;
	}

	/* incrementing number of requests for resizing */
	device_config.array_count++;

	if (device_config.array_count != SECONDENTRY) {
		up(&device_config.array_sem);

		down_interruptible(&(rsz_conf_chan->channel_sem));

	} else {
		up(&device_config.array_sem);
	}

	return SUCESS;
}

/*
 Function	to delete the processed	array entry	form the array
*/
int delete_from_array(struct channel_config *rsz_conf_chan)
{
	int array_index = FIRSTENTRY, device_index;

	down_interruptible(&(device_config.array_sem));

	/*shift the     entried in array */
	if (device_config.array_count != SECONDENTRY) {
		/* decrementing the     request count */
		device_config.array_count--;

		/* Shift all the elements one step up in array */
		for (device_index = array_index;
		     device_index < device_config.array_count; device_index++) {

			device_config.channel_configuration[device_index] =
			    device_config.
			    channel_configuration[device_index + NEXT];
		}
		/* making last entry NULL; */
		device_config.channel_configuration[device_index + NEXT] = NULL;
	}
	/* remove the top entry */
	else {

		device_config.array_count--;
		device_config.channel_configuration[FIRSTENTRY] = NULL;
	}

	if (device_config.array_count != FIRSTENTRY) {
		/* Get config having highest priority in array
		   resizer_device.config
		   and unlock config.sem of that config */

		up(&(device_config.channel_configuration
		     [FIRSTENTRY]->channel_sem));
		up(&(device_config.array_sem));
	} else {
		up(&(device_config.array_sem));
	}

	return SUCESS;
}

int rsz_set_multipass(struct channel_config *rsz_conf_chan)
{	
	multipass.in_hsize  = multipass.out_hsize;
	multipass.in_vsize  = multipass.out_vsize;
	multipass.out_hsize = multipass.end_hsize;
	multipass.out_vsize = multipass.end_vsize;
	
	multipass.out_pitch = ((multipass.inptyp)? 
				multipass.out_hsize: (multipass.out_hsize * 2));		
	multipass.in_pitch = ((multipass.inptyp)? 
				multipass.in_hsize: (multipass.in_hsize * 2));		
	rsz_set_ratio(rsz_conf_chan);
	rsz_config_ratio(rsz_conf_chan);
	rsz_hardware_setup(rsz_conf_chan);
	return 0;
}

void rsz_copy_data(struct rsz_params *params)
{
	int i;
	multipass.in_hsize  = params->in_hsize;
	multipass.in_vsize  = params->in_vsize;
	multipass.out_hsize = params->out_hsize;
	multipass.out_vsize = params->out_vsize;
	multipass.end_hsize = params->out_hsize;
	multipass.end_vsize = params->out_vsize;
	multipass.in_pitch  = params->in_pitch;
	multipass.out_pitch = params->out_pitch;
	multipass.hstph     = params->hstph;
	multipass.vstph     = params->vstph;
	multipass.inptyp    = params->inptyp;
	multipass.pix_fmt   = params->pix_fmt;
	multipass.cbilin    = params->cbilin;

	for (i = 0; i < 32; i++) {
	multipass.tap4filt_coeffs[i] = params->tap4filt_coeffs[i];
	multipass.tap7filt_coeffs[i] = params->tap7filt_coeffs[i];
	}
}

int rsz_set_params(struct rsz_params *params,
		   struct channel_config *rsz_conf_chan)
{
	/*copy to local structure to be ready if multipass is required*/
	rsz_copy_data(params);

	if (0 != rsz_set_ratio(rsz_conf_chan))
		return -EINVAL;

	/* if input is from ram that vertical pixel should be zero */
	if (INPUT_RAM) {
		params->vert_starting_pixel = ZERO;
	}

	/* Configuring the starting pixel in vertical direction */
	rsz_conf_chan->register_config.rsz_in_start =
	    ((params->vert_starting_pixel << ISPRSZ_IN_SIZE_VERT_SHIFT)
	     & ISPRSZ_IN_SIZE_VERT_MASK);

	/* if input is 8 bit that start pixel should be <= to than 31 */
	if (params->inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_8BIT)
			return -EINVAL;
	}
	/* if input     is 16 bit that start pixel should be <= than 15 */
	if (params->inptyp == RSZ_INTYPE_YCBCR422_16BIT) {
		if (params->horz_starting_pixel > MAX_HORZ_PIXEL_16BIT)
			return -EINVAL;
	}

	/* Configuring the      starting pixel in horizontal direction */
	rsz_conf_chan->register_config.rsz_in_start |=
	    params->horz_starting_pixel & ISPRSZ_IN_START_HORZ_ST_MASK;

	/* Coefficinets of parameters for luma :- algo configuration */
	rsz_conf_chan->register_config.rsz_yehn =
	    ((params->yenh_params.type << ISPRSZ_YENH_ALGO_SHIFT) &
	     ISPRSZ_YENH_ALGO_MASK);

	/* Coefficinets of parameters for luma :- core configuration */
	if (params->yenh_params.type) {
		rsz_conf_chan->register_config.rsz_yehn |=
		    params->yenh_params.core & ISPRSZ_YENH_CORE_MASK;

		/* Coefficinets of parameters for luma :- gain configuration */

		rsz_conf_chan->register_config.rsz_yehn |=
		    ((params->yenh_params.gain << ISPRSZ_YENH_GAIN_SHIFT)
		     & ISPRSZ_YENH_GAIN_MASK);

		/* Coefficinets of parameters for luma :- gain configuration */
		rsz_conf_chan->register_config.rsz_yehn |=
		    ((params->yenh_params.slop << ISPRSZ_YENH_SLOP_SHIFT)
		     & ISPRSZ_YENH_SLOP_MASK);
	}

	rsz_config_ratio(rsz_conf_chan);

	/*Setting the configuration status */
	rsz_conf_chan->config_state = STATE_CONFIGURED;
	return SUCESS;
}


int rsz_set_ratio (struct channel_config *rsz_conf_chan)
{
	int alignment = 0;

	rsz_conf_chan->register_config.rsz_cnt = 0;

	/* Configuring the chrominance algorithm */
	if (multipass.cbilin) {
		rsz_conf_chan->register_config.rsz_cnt =
		    BITSET(rsz_conf_chan->register_config.rsz_cnt,
			   SET_BIT_CBLIN);
	}
	/* Configuring the input source */
	if (INPUT_RAM) {
		rsz_conf_chan->register_config.rsz_cnt =
		    BITSET(rsz_conf_chan->register_config.rsz_cnt,
			   SET_BIT_INPUTRAM);
	}
	/* Configuring the input type */
	if (multipass.inptyp == RSZ_INTYPE_PLANAR_8BIT) {
		rsz_conf_chan->register_config.rsz_cnt =
		    BITSET(rsz_conf_chan->register_config.rsz_cnt,
			   SET_BIT_INPTYP);
	} else {
		rsz_conf_chan->register_config.rsz_cnt =
		    BITRESET(rsz_conf_chan->register_config.rsz_cnt,
			     SET_BIT_INPTYP);

		/* Configuring the chrominace position type */
		if (multipass.pix_fmt == RSZ_PIX_FMT_UYVY) {
			rsz_conf_chan->register_config.rsz_cnt =
			    BITRESET(rsz_conf_chan->register_config.rsz_cnt,
				     SET_BIT_YCPOS);
		} else if (multipass.pix_fmt == RSZ_PIX_FMT_YUYV) {
			rsz_conf_chan->register_config.rsz_cnt =
			    BITSET(rsz_conf_chan->register_config.rsz_cnt,
				   SET_BIT_YCPOS);
		}

	}
	multipass.vrsz = 
		(multipass.in_vsize * RATIO_MULTIPLIER)/multipass.out_vsize;
	multipass.hrsz =
		(multipass.in_hsize * RATIO_MULTIPLIER)/multipass.out_hsize;
	if (UP_RSZ_RATIO > multipass.vrsz || UP_RSZ_RATIO > multipass.hrsz){
		printk("Upscaling ratio not supported!");
		return -EINVAL;
	}
	/* calculating the horizontal and vertical ratio */
	multipass.vrsz = (multipass.in_vsize - NUM_D2TAPS) * RATIO_MULTIPLIER 
		/ (multipass.out_vsize - 1);
	multipass.hrsz = ((multipass.in_hsize - NUM_D2TAPS) * RATIO_MULTIPLIER)
	    	/ (multipass.out_hsize - 1);

	/* recalculating Horizontal     ratio */
	if (multipass.hrsz <= 512) {	/* 4-tap     8-phase filter */
		multipass.hrsz = (multipass.in_hsize - NUM_TAPS) * RATIO_MULTIPLIER
		    / (multipass.out_hsize - 1);
		if (multipass.hrsz < 64)
			multipass.hrsz = 64;
		if (multipass.hrsz > 512)
			multipass.hrsz = 512;
		if (multipass.hstph > NUM_PHASES)
			return -EINVAL;
		multipass.num_tap = 1;
	} else if (multipass.hrsz >= 513 && multipass.hrsz <= 1024) {
		/* 7-tap        4-phase filter */
		if (multipass.hstph > NUM_D2PH)
			return -EINVAL;
		multipass.num_tap = 0;
	}

	/* recalculating vertical ratio */
	if (multipass.vrsz <= 512) {	/* 4-tap     8-phase filter */
		multipass.vrsz = (multipass.in_vsize - NUM_TAPS) * RATIO_MULTIPLIER /
		    (multipass.out_vsize - 1);
		if (multipass.vrsz < 64)
			multipass.vrsz = 64;
		if (multipass.vrsz > 512)
			multipass.vrsz = 512;
		if (multipass.vstph > NUM_PHASES)
			return -EINVAL;
	} else if (multipass.vrsz >= 513 && multipass.vrsz <= 1024) {
		if (multipass.vstph > NUM_D2PH)
			return -EINVAL;
	}
	/* Filling the input pitch in the structure */
	if ((multipass.in_pitch) % ALIGN32) {
		printk("Inavlid input pitch: %d \n", multipass.in_pitch);
		return -EINVAL;
	}
	if ((multipass.out_pitch) % ALIGN32) {
		printk("Inavlid	output pitch %d \n", multipass.out_pitch);
		return -EINVAL;
	}

	/* If vertical upsizing then */
	if (multipass.vrsz < 256 && 
			(multipass.in_vsize < multipass.out_vsize)) {
		/* checking     for     both types of format */
		if (multipass.inptyp == RSZ_INTYPE_PLANAR_8BIT) {
			alignment = ALIGNMENT;
		} else if (multipass.inptyp == RSZ_INTYPE_YCBCR422_16BIT) {
			alignment = (ALIGNMENT / 2);
		} else {
			printk("Invalid input type	\n");
		}
		/* errror checking for output size */
		if (!(((multipass.out_hsize % PIXEL_EVEN) == ZERO)
		      && (multipass.out_hsize % alignment) == ZERO)) {
			printk("wrong hsize	\n");

			return -EINVAL;
		}
	}
	if (multipass.hrsz >= 64 && multipass.hrsz <= 1024) {
		if (multipass.out_hsize > MAX_IMAGE_WIDTH) {
			printk("wrong width	\n");
			return -EINVAL;
		}
		multipass.active = 0;

	} else if (multipass.hrsz > 1024){
		if (multipass.out_hsize > MAX_IMAGE_WIDTH) {
			printk("wrong width	\n");
			return -EINVAL;
		}
		if (multipass.hstph > NUM_D2PH)
			return -EINVAL;
		multipass.num_tap = 0;
		multipass.out_hsize = multipass.in_hsize *256/1024;
		if((multipass.out_hsize) % ALIGN32){
			multipass.out_hsize += 
				abs((multipass.out_hsize % ALIGN32) - ALIGN32);	
		}
		multipass.out_pitch = ((multipass.inptyp)? 
				multipass.out_hsize: (multipass.out_hsize * 2));
		multipass.hrsz = ((multipass.in_hsize - NUM_D2TAPS) * RATIO_MULTIPLIER)
	    	/ (multipass.out_hsize - 1);
		multipass.active = 1;


	}

	if (multipass.vrsz > 1024) {
		if (multipass.out_vsize > MAX_IMAGE_WIDTH_HIGH) {
		printk("wrong width	\n");
		return -EINVAL;
		}
		multipass.vrsz = 1024;
		multipass.out_vsize = multipass.in_vsize *256 / 1024;
		multipass.active = 1;
		multipass.num_tap = 0; //7tap

	}
	rsz_conf_chan->register_config.rsz_out_size =
	    (multipass.out_hsize & ISPRSZ_OUT_SIZE_HORZ_MASK);

	rsz_conf_chan->register_config.rsz_out_size |=
	    ((multipass.out_vsize << ISPRSZ_OUT_SIZE_VERT_SHIFT) &
	     ISPRSZ_OUT_SIZE_VERT_MASK);

	rsz_conf_chan->register_config.rsz_sdr_inoff =
	    ((multipass.in_pitch) & ISPRSZ_SDR_INOFF_OFFSET_MASK);

	rsz_conf_chan->register_config.rsz_sdr_outoff =
	    multipass.out_pitch & ISPRSZ_SDR_OUTOFF_OFFSET_MASK;

	/* checking the validity of the horizontal phase value */
	if (multipass.hrsz >= 64 && multipass.hrsz <= 512) {
		if (multipass.hstph > NUM_PHASES)
			return -EINVAL;
	} else if (multipass.hrsz >= 64 && multipass.hrsz <= 512) {
		if (multipass.hstph > NUM_D2PH)
			return -EINVAL;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
	    ((multipass.hstph << ISPRSZ_CNT_HSTPH_SHIFT) & ISPRSZ_CNT_HSTPH_MASK);

	/* checking     the     validity of     the     vertical phase value */
	if (multipass.vrsz >= 64 && multipass.hrsz <= 512) {
		if (multipass.vstph > NUM_PHASES)
			return -EINVAL;
	} else if (multipass.vrsz >= 64 && multipass.vrsz <= 512) {
		if (multipass.vstph > NUM_D2PH)
			return -EINVAL;
	}

	rsz_conf_chan->register_config.rsz_cnt |=
	    ((multipass.vstph << ISPRSZ_CNT_VSTPH_SHIFT) & ISPRSZ_CNT_VSTPH_MASK);

	/* Configuring the horizonatl ratio */
	rsz_conf_chan->register_config.rsz_cnt |=
	    ((multipass.hrsz - 1) & ISPRSZ_CNT_HRSZ_MASK);

	/* Configuring the vertical     ratio */
	rsz_conf_chan->register_config.rsz_cnt |=
	    (((multipass.vrsz - 1) << ISPRSZ_CNT_VRSZ_SHIFT) & ISPRSZ_CNT_VRSZ_MASK);

	return 0;
}



void rsz_config_ratio (struct channel_config *rsz_conf_chan)
{
	int hsize;
	int vsize;
	int coeffcounter;
	
	if (multipass.hrsz <= 512) {	/*4-tap filter */
		hsize =
		    ((32 * multipass.hstph + (multipass.out_hsize - 1) * multipass.hrsz +
		      16) >> 8) + 7;
	} else {
		hsize =
		    ((64 * multipass.hstph + (multipass.out_hsize - 1) * multipass.hrsz +
		      32) >> 8) + 7;
	}
	if (multipass.vrsz <= 512) {	/*4-tap filter */
		vsize =
		    ((32 * multipass.vstph + (multipass.out_vsize - 1) * multipass.vrsz +
		      16) >> 8) + 4;
	} else {
		vsize =
		    ((64 * multipass.vstph + (multipass.out_vsize - 1) * multipass.vrsz +
		      32) >> 8) + 7;
	}
	/* Configuring the Horizontal size of inputframn in MMR */
	rsz_conf_chan->register_config.rsz_in_size = hsize;

	rsz_conf_chan->register_config.rsz_in_size |=
	    ((vsize << ISPRSZ_IN_SIZE_VERT_SHIFT)
	     & ISPRSZ_IN_SIZE_VERT_MASK);

	for (coeffcounter = ZERO; coeffcounter < MAX_COEF_COUNTER;
	     coeffcounter++) {
		/* Configuration of     horizontal coefficients */
		if(multipass.num_tap){    //4tap
		rsz_conf_chan->register_config.
		    rsz_coeff_horz[coeffcounter] =
		    (multipass.tap4filt_coeffs[2 * coeffcounter]
		     & ISPRSZ_HFILT10_COEF0_MASK);

		rsz_conf_chan->register_config.
		    rsz_coeff_horz[coeffcounter] |=
		    ((multipass.tap4filt_coeffs[2 * coeffcounter + NEXT]
		      << ISPRSZ_HFILT10_COEF1_SHIFT) &
		     ISPRSZ_HFILT10_COEF1_MASK);

		} else {   //7tap
		
		rsz_conf_chan->register_config.
		    rsz_coeff_horz[coeffcounter] =
		    (multipass.tap7filt_coeffs[2 * coeffcounter]
		     & ISPRSZ_HFILT10_COEF0_MASK);
		
		rsz_conf_chan->register_config.
		    rsz_coeff_horz[coeffcounter] |=
		    ((multipass.tap7filt_coeffs[2 * coeffcounter + NEXT]
		      << ISPRSZ_HFILT10_COEF1_SHIFT) &
		     ISPRSZ_HFILT10_COEF1_MASK);
		}
		
		/* Configuration of     Vertical coefficients */
		if(multipass.num_tap){    //4tap
		rsz_conf_chan->register_config.
		    rsz_coeff_vert[coeffcounter] =
		    (multipass.tap4filt_coeffs[2 * coeffcounter] 
		     & ISPRSZ_VFILT10_COEF0_MASK);
		
		rsz_conf_chan->register_config.
		    rsz_coeff_vert[coeffcounter] |=
		    ((multipass.tap4filt_coeffs[2 * coeffcounter + NEXT] 
		      << ISPRSZ_VFILT10_COEF1_SHIFT) &
		     ISPRSZ_VFILT10_COEF1_MASK);
		} else {   //7tap
		
		rsz_conf_chan->register_config.
		    rsz_coeff_vert[coeffcounter] =
		    (multipass.tap7filt_coeffs[2 * coeffcounter] 
		     & ISPRSZ_VFILT10_COEF0_MASK);
		
		rsz_conf_chan->register_config.
		    rsz_coeff_vert[coeffcounter] |=
		    ((multipass.tap7filt_coeffs[2 * coeffcounter + NEXT] 
		       << ISPRSZ_VFILT10_COEF1_SHIFT) &
		     ISPRSZ_VFILT10_COEF1_MASK);
		}
	}
	
			
			
}
/*
 Function to get the parameters	values
*/
int rsz_get_params(struct rsz_params *params,
		   struct channel_config *rsz_conf_chan)
{
	int coeffcounter;

	if (rsz_conf_chan->config_state) {
		printk("	state not configured \n");
		return -EINVAL;
	}

	/* getting the horizontal size */
	params->in_hsize =
	    (rsz_conf_chan->register_config.rsz_in_size &
	    ISPRSZ_IN_SIZE_HORZ_MASK);
	/* getting the vertical size */
	params->in_vsize =
	    ((rsz_conf_chan->register_config.rsz_in_size
	      & ISPRSZ_IN_SIZE_VERT_MASK) >> ISPRSZ_IN_SIZE_VERT_SHIFT);

	/* getting the input pitch */
	params->in_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_inoff
	    & ISPRSZ_SDR_INOFF_OFFSET_MASK;

	/* getting the output horizontal size */
	params->out_hsize =
	    rsz_conf_chan->register_config.rsz_out_size
	    & ISPRSZ_OUT_SIZE_HORZ_MASK;

	/* getting the vertical size   */
	params->out_vsize =
	    ((rsz_conf_chan->register_config.rsz_out_size
	      & ISPRSZ_OUT_SIZE_VERT_MASK) >> ISPRSZ_OUT_SIZE_VERT_SHIFT);

	/* getting the output pitch */
	params->out_pitch =
	    rsz_conf_chan->register_config.rsz_sdr_outoff
	    & ISPRSZ_SDR_OUTOFF_OFFSET_MASK;

	/* getting the chrominance algorithm  */
	params->cbilin =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & SET_BIT_CBLIN) >> SET_BIT_CBLIN);

	/* getting the input type */
	params->inptyp =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & ISPRSZ_CNT_INPTYP_MASK) >> SET_BIT_INPTYP);
	/* getting the  starting pixel in horizontal direction */
	params->horz_starting_pixel =
	    ((rsz_conf_chan->register_config.rsz_in_start
	      & ISPRSZ_IN_START_HORZ_ST_MASK));
	/* getting the  starting pixel in vertical direction */
	params->vert_starting_pixel =
	    ((rsz_conf_chan->register_config.rsz_in_start
	      & ISPRSZ_IN_START_VERT_ST_MASK) >> ISPRSZ_IN_START_VERT_ST_SHIFT);

	/* getting the horizontal starting phase */
	params->hstph =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & ISPRSZ_CNT_HSTPH_MASK >> ISPRSZ_CNT_HSTPH_SHIFT));
	/* getting the vertical starting phase */
	params->vstph =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & ISPRSZ_CNT_VSTPH_MASK >> ISPRSZ_CNT_VSTPH_SHIFT));

	for (coeffcounter = ZERO; coeffcounter < MAX_COEF_COUNTER;
	     coeffcounter++) {
		/* getting the horizontal coefficients 0 */
		params->tap4filt_coeffs[2 * coeffcounter] =
		    rsz_conf_chan->register_config.rsz_coeff_horz[coeffcounter]
		    & ISPRSZ_HFILT10_COEF0_MASK;

		/* getting the horizontal coefficients 1 */
		params->tap4filt_coeffs[2 * coeffcounter + NEXT] =
		    ((rsz_conf_chan->register_config.
		      rsz_coeff_horz[coeffcounter]
		      & ISPRSZ_HFILT10_COEF1_MASK) >>
		     ISPRSZ_HFILT10_COEF1_SHIFT);

		/* getting the vertical coefficients 0 */
		params->tap7filt_coeffs[2 * coeffcounter] =
		    rsz_conf_chan->register_config.rsz_coeff_vert[coeffcounter]
		    & ISPRSZ_VFILT10_COEF0_MASK;

		/* getting the vertical coefficients 1 */
		params->tap7filt_coeffs[2 * coeffcounter + NEXT] =
		    ((rsz_conf_chan->register_config.
		      rsz_coeff_vert[coeffcounter]
		      & ISPRSZ_VFILT10_COEF1_MASK) >>
		     ISPRSZ_VFILT10_COEF1_SHIFT);

	}

	/* getting the parameters for luma :- algo */
	params->yenh_params.type =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ISPRSZ_YENH_ALGO_MASK) >> ISPRSZ_YENH_ALGO_SHIFT);

	/* getting the parameters for luma :- core      */
	params->yenh_params.core =
	    (rsz_conf_chan->register_config.rsz_yehn & ISPRSZ_YENH_CORE_MASK);

	/* Coefficinets of parameters for luma :- gain  */
	params->yenh_params.gain =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ISPRSZ_YENH_GAIN_MASK) >> ISPRSZ_YENH_GAIN_SHIFT);

	/* Coefficinets of parameters for luma :- SLOP configuration */
	params->yenh_params.slop =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ISPRSZ_YENH_SLOP_MASK) >> ISPRSZ_YENH_SLOP_SHIFT);

	/* getting the input type */
	params->pix_fmt =
	    ((rsz_conf_chan->register_config.rsz_cnt
	      & ISPRSZ_CNT_PIXFMT_MASK) >> SET_BIT_YCPOS);

	if (params->pix_fmt)
		params->pix_fmt = RSZ_PIX_FMT_UYVY;
	else
		params->pix_fmt = RSZ_PIX_FMT_YUYV;

	return SUCESS;
}

void rsz_calculate_crop(struct channel_config *rsz_conf_chan,
			struct rsz_cropsize *cropsize)
{
	int luma_enable;

	cropsize->hcrop = ZERO;
	cropsize->vcrop = ZERO;

	luma_enable =
	    ((rsz_conf_chan->register_config.rsz_yehn
	      & ISPRSZ_YENH_ALGO_MASK) >> ISPRSZ_YENH_ALGO_SHIFT);

	/* Luma enhancement reduces image width 1 pixels from left,right */
	if (luma_enable) {
		cropsize->hcrop += 2;
	}
}

/*
 this	function free the input	and output buffers alloated
*/
int free_buff(struct channel_config *rsz_conf_chan)
{
	int buffercounter = ZERO;

	/* Free all     the     input buffers */
	while (rsz_conf_chan->input_buffer[buffercounter] != NULL
	       && buffercounter < MAX_INPUT_BUFFERS) {
		/* free the     memory */
		rsz_free_pages((unsigned long)rsz_conf_chan->input_buffer
			       [buffercounter], rsz_conf_chan->in_bufsize);
		/* assign buffer zero to indicate its free */
		rsz_conf_chan->input_buffer[buffercounter] = NULL;
		buffercounter++;
	}
	buffercounter = ZERO;
	/* free all the output buffers */
	while (rsz_conf_chan->output_buffer[buffercounter] != NULL
	       && buffercounter < MAX_INPUT_BUFFERS) {
		/* free the memory */
		rsz_free_pages((unsigned long)rsz_conf_chan->output_buffer
			       [buffercounter], rsz_conf_chan->out_bufsize);
		/*  assign buffer zero to indicate its  free */
		rsz_conf_chan->output_buffer[buffercounter] = NULL;
		buffercounter++;
	}

	return SUCESS;
}

/*
This function creates a channels.
*/
static int rsz_open(struct inode *inode, struct file *filp)
{
	struct channel_config *rsz_conf_chan;
	int buffercounter;

	if (filp->f_flags == O_NONBLOCK)
		return -1;
	/* if usage counter is greater than maximum supported channels
	   return error */
	if (device_config.module_usage_count >= MAX_CHANNELS) {
		printk("\n modules usage count is greater than supported ");
		return -EBUSY;
	}

	isp_get();

	/* allocate     memory for a new configuration */
	rsz_conf_chan = kmalloc(sizeof(struct channel_config), GFP_KERNEL);

	if (rsz_conf_chan == NULL) {
		printk("\n cannot allocate memory ro channel config");
		return -ENOMEM;
	}

	/* zeroing register     config */
	memset(&(rsz_conf_chan->register_config), ZERO,
	       sizeof(struct resizer_config));

	/* increment usage counter */
	/* Lock the     global variable and increment the counter */
	down_interruptible(&device_config.device_mutex);
	device_config.module_usage_count++;
	up(&device_config.device_mutex);

	/*STATE_NOT_CONFIGURED and priority to zero */
	rsz_conf_chan->config_state = STATE_NOT_CONFIGURED;

	/* Set priority to lowest for that configuration channel */
	rsz_conf_chan->priority = MIN_PRIORITY;

	rsz_conf_chan->status = CHANNEL_FREE;
	/*Set configuration     structure's    input_buffer and output_buffer */
	/*pointers to NULL */

	for (buffercounter = ZERO; buffercounter < MAX_INPUT_BUFFERS;
	     buffercounter++) {
		/* Help to initialize the input buffer to zero */
		rsz_conf_chan->input_buffer[buffercounter] = NULL;
	}

	for (buffercounter = ZERO; buffercounter < MAX_OUTPUT_BUFFERS;
	     buffercounter++) {
		/* Help to initialize the output buffer to zero */
		rsz_conf_chan->output_buffer[buffercounter] = NULL;

	}

/* Initializing of application mutex */
	init_MUTEX_LOCKED(&(rsz_conf_chan->channel_sem));
	init_MUTEX(&(rsz_conf_chan->chanprotection_sem));
	/* taking the configuartion     structure in private data */
	filp->private_data = rsz_conf_chan;

	return SUCESS;

}

/*
 The Function	is used	to release the number of resources occupied
 by the channel
*/
static int rsz_release(struct inode *inode, struct file *filp)
{
	/* get the configuratin of this channel from private_date member of
	   file */
	int ret = 0;
	struct channel_config *rsz_conf_chan =
	    (struct channel_config *)filp->private_data;

	ret = down_trylock(&(rsz_conf_chan->chanprotection_sem));
	if (ret != 0) {

		printk("Channel in use\n");
		return -EBUSY;
	}

	/* It will free all the input and output buffers */
	free_buff(rsz_conf_chan);

	/* Decrements the module usage count; */
	/* Lock the global variable and decrement variable */
	down_interruptible(&device_config.device_mutex);
	device_config.module_usage_count--;
	up(&device_config.device_mutex);

	kfree(rsz_conf_chan);

	up(&(rsz_conf_chan->chanprotection_sem));

	isp_put();

	return SUCESS;
}

/*
Function to map device memory into user	space
 */ 
static int rsz_mmap(struct file *filp, struct vm_area_struct *vma)
{

	/* get the configuratin of this channel from private_date
	   member of file */
	/* for looping purpuse */
	int buffercounter = ZERO;

	/* for checking purpose */
	int flag = ZERO;
	/* Hold number of input and output buffer allocated */
	int in_numbuffers = ZERO, out_numbuffers = ZERO;
	int buffer_offset;

	unsigned int offset = vma->vm_pgoff << PAGE_SHIFT;

	struct channel_config *rsz_conf_chan =
	    (struct channel_config *)filp->private_data;

	/* Count the number of input buffers allocated */
	while ((rsz_conf_chan->input_buffer[buffercounter]) != NULL) {
		in_numbuffers++;
		buffercounter++;
	}
	buffercounter = ZERO;

	/* To Count the number of output buffers allocated */
	while ((rsz_conf_chan->output_buffer[buffercounter]) != NULL) {
		out_numbuffers++;
		buffercounter++;
	}

	/*Find the input address which  is to be mapped */
	for (buffercounter = ZERO; buffercounter < in_numbuffers;
	     buffercounter++) {
		buffer_offset =
		    virt_to_phys(rsz_conf_chan->input_buffer[buffercounter]);
		if (buffer_offset == offset) {
			flag = ADDRESS_FOUND;
			break;
		}
	}
	/*Find the output address which is to be mapped */
	if (flag == ZERO) {
		for (buffercounter = ZERO; buffercounter < out_numbuffers;
		     buffercounter++) {
			buffer_offset =
			    virt_to_phys(rsz_conf_chan->
					 output_buffer[buffercounter]);
			if (buffer_offset == offset) {
				flag = ADDRESS_FOUND;
				break;
			}
		}
	}
	/* The address to be mapped     is not found so return error */

	if (flag == ZERO)
		return -EAGAIN;

	/* map the address from user space to kernel space */
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		return -EAGAIN;
	}

	return SUCESS;
}

/*
This function	will process IOCTL commands sent by
the application	and
control the device IO operations.
*/
static int rsz_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int ret = ZERO;
	/*get the configuratin of this channel from
	   private_date member of file */
	struct channel_config *rsz_conf_chan =
	    (struct channel_config *)file->private_data;

	/* Create the structures of
	   different parameters passed by user */
	struct rsz_priority *prio;
	struct rsz_status *status;
	struct rsz_resize *resize;

	ret = down_trylock(&(rsz_conf_chan->chanprotection_sem));
	if (ret != 0) {
		printk("Channel in use\n");
		return -EBUSY;
	}

	/* Before decoding check for correctness of cmd */
	if (_IOC_TYPE(cmd) != RSZ_IOC_BASE) {
		printk("Bad	command	Value \n");
		return -1;
	}
	if (_IOC_NR(cmd) > RSZ_IOC_MAXNR) {
		printk("Bad	command	Value\n");
		return -1;
	}

	/*veryfying     access permission of commands */
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, _IOC_SIZE(cmd));
	if (ret) {
		printk("access denied\n");
		return -1;	/*error in access */
	}

	/* switch according     value of cmd */
	switch (cmd) {
		/*This ioctl is used to request frame buffers to be
		   allocated by the RSZ module. The allocated buffers
		   are channel  specific and can be     addressed
		   by indexing */
	case RSZ_REQBUF:

		/* Function to allocate the memory to input
		   or output buffer. */
		ret = malloc_buff((struct rsz_reqbufs *)arg, rsz_conf_chan);
		break;

		/*This ioctl is used to query the physical address of a
		   particular frame buffer. */
	case RSZ_QUERYBUF:

		/* Function     to query the  physical address of
		   the buffer  requested by index. */
		ret = get_buf_address((struct rsz_buffer *)arg, rsz_conf_chan);
		break;

		/* This ioctl is used to set the priority of the current
		   logical channel. If multiple resizing tasks from multiple
		   logical channels are currently *pending, the task
		   associated with the highest priority logical channel
		   will be executed first. */
	case RSZ_S_PRIORITY:

		prio = (struct rsz_priority *)arg;
		/* Check the prioroty range and assign the priority */
		if (prio->priority > MAX_PRIORITY ||
		    prio->priority < MIN_PRIORITY)
			return -EINVAL;
		else {
			rsz_conf_chan->priority = prio->priority;
		}
		break;
		/* This ioctl is used to get the priority of
		   the current logic channel */
	case RSZ_G_PRIORITY:

		prio = (struct rsz_priority *)arg;
		/* Get the priority     from the channel */
		prio->priority = rsz_conf_chan->priority;
		break;

		/* This ioctl is used to set the parameters
		   of the Resizer hardware, including input and output
		   image size, horizontal    and     vertical poly-phase
		   filter coefficients,luma enchancement filter coefficients etc */
	case RSZ_S_PARAM:

		/* Function to set the hardware configuration */
		ret = rsz_set_params((struct rsz_params *)arg, rsz_conf_chan);
		break;

		/*This ioctl is used to get the Resizer hardware settings
		   associated with the current logical channel represented
		   by fd. */
	case RSZ_G_PARAM:
		/* Function to get the hardware configuration */
		ret = rsz_get_params((struct rsz_params *)arg, rsz_conf_chan);
		break;

		/* This ioctl is used to check the current status
		   of the Resizer hardware */
	case RSZ_G_STATUS:
		status = (struct rsz_status *)arg;
		status->chan_busy = rsz_conf_chan->status;
		status->hw_busy = ispresizer_busy();
		status->src = INPUT_RAM;
		break;

		/*This ioctl submits a resizing task specified by the
		   rsz_resize structure.The call can either be blocked until
		   the task is completed or returned immediately based
		   on the value of the blocking argument in the rsz_resize
		   structure. If  it is blocking, the     status of the task
		   can be checked by calling ioctl   RSZ_G_STATUS. Only one task
		   can  be outstanding for each logical channel. */
	case RSZ_RESIZE:

		resize = (struct rsz_resize *)arg;
		ret = rsz_start((struct rsz_resize *)arg, rsz_conf_chan);
		break;

	case RSZ_GET_CROPSIZE:

		rsz_calculate_crop(rsz_conf_chan, (struct rsz_cropsize *)arg);
		break;

	default:
		printk("resizer_ioctl: Invalid Command Value");
		ret = -EINVAL;
	}

	up(&(rsz_conf_chan->chanprotection_sem));

	return ret;
}

static struct file_operations rsz_fops = {
	.owner = THIS_MODULE,
	.open = rsz_open,
	.release = rsz_release,
	.mmap = rsz_mmap,
	.ioctl = rsz_ioctl,
};

/*
Function to register the Resizer character device	driver
*/
void rsz_isr(unsigned long status, void *arg1, void *arg2)
{

	if ((RESZ_DONE & status) != RESZ_DONE)
		return;

	complete(&(device_config.sem_isr));
	
}


static void resizer_platform_release(struct device *device)
{
	/* This is called when the reference count goes to zero */
}

static int __init resizer_probe(struct platform_device *device)
{
	return 0;
}

static int resizer_remove(struct platform_device *omap_resizer_device)
{
	return 0;
}

static struct class *rsz_class = NULL;

static struct platform_device omap_resizer_device = {
	.name = OMAP_REZR_NAME,
	.id = 2,
	.dev = {
		.release = resizer_platform_release,}
};

static struct platform_driver omap_resizer_driver = {
	.probe = resizer_probe,
	.remove = resizer_remove,
	.driver = {
		   .bus = &platform_bus_type,
		   .name = OMAP_REZR_NAME,
		   },
};

/*
function to	register resizer character driver
*/
static int __init omap_rsz_init(void)
{

	int ret;
	int major;
	device_config.module_usage_count = ZERO;
	device_config.array_count = ZERO;

	/* Register     the     driver in the kernel */
	major = register_chrdev(0, OMAP_REZR_NAME, &rsz_fops);

	if (major < 0) {
		printk(OMAP_REZR_NAME ": initialization "
		       "failed. could not register character device\n");
		return -ENODEV;
	}
	/* Initialize of character device */
	cdev_init(&c_dev, &rsz_fops);
	c_dev.owner = THIS_MODULE;
	c_dev.ops = &rsz_fops;

	/* addding character device */
	ret = cdev_add(&c_dev, dev, 1);

	if (ret) {
		printk(OMAP_REZR_NAME ": Error adding \
		OMAP ISP Resizer... error no:%d\n", ret);
		unregister_chrdev_region(dev, 1);
		return ret;
	}

	/* register driver as a platform driver */
	ret = platform_driver_register(&omap_resizer_driver);
	if (ret) {
		printk(OMAP_REZR_NAME
		       ": failed to register platform driver!\n");
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(major, OMAP_REZR_NAME);
		cdev_del(&c_dev);
		return -EINVAL;
	}

	/* Register the drive as a platform device */
	ret = platform_device_register(&omap_resizer_device);
	if (ret) {
		printk(OMAP_REZR_NAME
		       ": failed to register platform device!\n");
		platform_driver_unregister(&omap_resizer_driver);
		unregister_chrdev_region(dev, 1);
		unregister_chrdev(MAJOR(dev), OMAP_REZR_NAME);
		cdev_del(&c_dev);
		return -EINVAL;
	}

	rsz_class = class_create(THIS_MODULE, OMAP_REZR_NAME);

	if (!rsz_class) {

		platform_device_unregister(&omap_resizer_device);
		cdev_del(&c_dev);
		unregister_chrdev(MAJOR(dev), OMAP_REZR_NAME);

		return -EIO;
	}

	/* make entry in the devfs */
	rsz_device = device_create(rsz_class, rsz_device, MKDEV(major, 0),
				   OMAP_REZR_NAME);

	/* register     simple device class     */

	init_completion(&(device_config.sem_isr));

	device_config.sem_isr.done = ZERO;

	/* Initialize the device mutex */
	init_MUTEX(&device_config.array_sem);
	init_MUTEX(&device_config.device_mutex);

	return ret;
}

/*
Function	is called by the kernel. It	unregister the device.
*/
void __exit omap_rsz_cleanup(void)
{
	platform_device_unregister(&omap_resizer_device);
	platform_driver_unregister(&omap_resizer_driver);
	cdev_del(&c_dev);
	unregister_chrdev_region(dev, 1);
	isp_unset_callback(CBK_RESZ_DONE);
}

module_init(omap_rsz_init)
    module_exit(omap_rsz_cleanup)

    MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP ISP Resizer");
MODULE_LICENSE("GPL");
