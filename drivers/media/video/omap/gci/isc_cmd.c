/*
 * drivers/media/video/omap/ise/isc_cmd.c
 *
 * ISP State Controller for TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/module.h>
#include <asm/types.h>
#include <asm/arch/gci/isc_cmd.h>

#include "isc.h"
#include "cam_config.h"

#if 0
#define DPRINTK_ISCCMD(format,...)\
	printk("ISCCMD: " format, ## __VA_ARGS__)
#else
#define DPRINTK_ISCCMD(format,...)
#endif


#define TRUE 1
#define FALSE 0

#define CAM_INIT		 1 << 0
#define CAM_POWER_ON		 1 << 1
#define CAM_POWER_OFF		 1 << 2
#define CAM_POWER_SUSP		 1 << 3
#define CAM_IMG_STOPPED		 1 << 4
#define CAM_IMG_RUNNING		 1 << 5
#define CAM_PRV_STOPPED		 1 << 6
#define CAM_PRV_RUNNING		 1 << 7

#define PAGE0_DIRTY	(0x1 << PAGE0)
#define PAGE1_DIRTY	(0x1 << PAGE1)
#define PAGE2_DIRTY	(0x1 << PAGE2)
#define PAGE3_DIRTY	(0x1 << PAGE3)
#define PAGE4_DIRTY	(0x1 << PAGE4)

static int pipeline_config;
typedef int (*cmd_funcptr)(u8*,u8);

static int cam_reg_prepare(void);
static int cam_reg_update(void);
static int cam_power(u8* data, u8 read_flag);
static void isc_prev_config(u8* data, u8 ndata);
static int prepare_prv_format(u8 curr_sensor);
static void prepare_prv_size(u8 curr_sensor);
static int prepare_img_format(u8 curr_sensor);
static void prepare_img_size(u8 curr_sensor);
static void update_cap_time(u8 curr_sensor, u8 reset);

/* TBD 
static void prepare_img_size(u8 curr_sensor);
*/

/* Utility functions */
static void check_cam_preview_config_changes(u8 *outfmt_change, 
			u8 *outsz_change);
static u32 inline get_prv_fmt(u8 *page1_ptr);

static void check_cam_img_config_changes(u8 *outfmt_change, 
			u8 *outsz_change);
static u32 inline get_img_fmt(u8 *page1_ptr);

static void check_cam_sensor_control_changes(u8 *inpfmt_change,
			u8 *inpsz_change);

/* Structure to maintain the state of the ISC */
static struct isc_cmdinfo{
	u8 current_page;
	u32 cam_page_dirty;
	u8 cam_shad_update;
	/* Shall be power-on/off/susp, prv-start/stop, img-start/stop */
	u16 cam_system_state;
	u16 cam_system_oldstate;
	u32 cam_error;
	u8 prvcontext;
	u8 active_sensor; //populated from the SENSOR_SELECT[0]
	u32 cam_prv_regevent;
	u32 cam_img_regevent;
	u32 cam_regevent;
}isc_obj;



struct isc_tuning_ctrl{
	u8 brightness;
	u8 contrast;
	u8 sharpness;
};

struct isc_aexp{
	u32 dummy;
};

struct isc_awb{
	u32 dummy;
};

/* Structure to instantiated for each Physical imager/sensor
 * supported on the board simulataneously
 */
static struct imager_settings{
	struct isc_sensor_info sensor_info;
	struct isc_sizes isp_img_size;
	struct isc_sizes isp_prv_size;
	struct isc_tuning_ctrl isp_ctrl;
	struct isc_aexp isp_exp;
	struct isc_awb isp_wb;
}isc_imager[2];


static u8 camcfg_started = 0;

volatile u8* data_ptr;
volatile u8* attr_ptr;

volatile CAMERA_REG_STATE cam_regState={0,0,0,0,0,0,0};

static void prepare_sensor(u8 curr_sensor);
void isc_resetpoweron(void);
void isc_prv_defaultsettings(void);
void isc_img_defaultsettings(void);
static void native_addr_readwrite(u8 *data, u8 ndata, u8 attr);

/* To be used for the secondary sensor 
static u8 page5_regptr[2][0x44];*/

static u8 curr_page0_regptr[2][PAGE0_SIZE + 1];
static u8 curr_page1_regptr[2][PAGE1_SIZE + 1];
static u8 curr_page2_regptr[2][PAGE2_SIZE + 1];
static u8 curr_page3_regptr[2][PAGE3_SIZE + 1];

static u8 page0_regptr[2][PAGE0_SIZE + 1];
static u8 page1_regptr[2][PAGE1_SIZE + 1];
static u8 page2_regptr[2][PAGE2_SIZE + 1];
static u8 page3_regptr[2][PAGE3_SIZE + 1];

static u8 def_page0_regptr[2][PAGE0_SIZE + 1]={
/* DATA */
/*	0	1	2	3	4	5	6	7	8*/
	{0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	9	A	B	C	D	E	F	10	11*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	12	13	14	15	16	17	18	19	1A*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},
/* ATTR */
/*	0	1	2	3	4	5	6	7	8*/
	{0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	9	A	B	C	D	E	F	10	11*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	12	13	14	15	16	17	18	19	1A*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3}
};
static u8 def_page1_regptr[2][PAGE1_SIZE + 1]= {
/* DATA */
/*	0	1	2	3	4	5	6	7	8*/
	{0x00,	0x02,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0xFF,

/*	9	A	B	C	D	E	F	10	11*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	12	13	14	15	16	17	18	19	1A*/
	0x00,	0x00,	0xFF,	0xFF,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	1B	1C	1D	1E	1F	20	21	22	23*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	24	25	26	27	28	29	2A	2B	2C*/
	0xFF,	0xFF,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	2D	2E	2F	30	31	32	33	34	35*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	36	37	38	39	3A	3B	3C	3D	3E*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	3F	40	41	42	43	44*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},
/* ATTR */
/*	0	1	2	3	4	5	6	7	8*/
	{0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	9	A	B	C	D	E	F	10	11*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	12	13	14	15	16	17	18	19	1A*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	1B	1C	1D	1E	1F	20	21	22	23*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	24	25	26	27	28	29	2A	2B	2C*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	2D	2E	2F	30	31	32	33	34	35*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	36	37	38	39	3A	3B	3C	3D	3E*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	3F	40	41	42	43	44*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3}
};

static u8 def_page2_regptr[2][PAGE2_SIZE + 1]={
/* DATA */
/*	0	1	2	3	4	5	6	7	8*/
	{0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	9	A	B	C	D	E	F	10	11*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	12	13	14	15	16	17	18	19*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},
/* ATTR */
/*	0	1	2	3	4	5	6	7	8*/
	{0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	9	A	B	C	D	E	F	10	11*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	12	13	14	15	16	17	18	19*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3}
};

static u8 def_page3_regptr[2][PAGE3_SIZE + 1]={
/* DATA */
/*	0	1	2	3	4	5	6	7	8*/
	{0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
/*	9	A	B	C	D	E	F*/
	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00},
/* ATTR */
/*	0	1	2	3	4	5	6	7	8*/
	{0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,
/*	9	A	B	C	D	E	F*/
	0x3,	0x3,	0x3,	0x3,	0x3,	0x3,	0x3}
};

/*
 * Takes one regtrans() at a time and 
 * sets regState variable to indicate which section 
 * of which page has been modified.
 * Calls cam_reg_prepare()to indicate the end 
 * of changes within a page.
 * Calls cam_reg_update() to apply the HW values to take effect
 * Returns the error (KErrNotSupported) if any from prepare/update calls
 */
int
isc_regtrans( u8*data,u8 ndata, u8 attr)
{
	int ret = 0;
	unsigned int data32 = 0;
	unsigned int data16 = 0;

	DPRINTK_ISCCMD("+ ISC_CMD isc_regtrans\n");
	if(data[0] == GET_ADDRESS(PAGE_SWITCH))
		isc_obj.current_page = data[1];

	if(isc_obj.current_page == PAGE0) {
		isc_obj.cam_page_dirty |= PAGE0_DIRTY;
		/* power on or off isp*/
		if((data[0] == 0x00)) {
			cam_power(&data[1], FALSE);
		}
		/* camera general settings*/
		else if((data[0] >= 0x01) && (data[0] <= 0x0C)){
			cam_regState.cam_general_register = 1;
		}
		else if((data[0] >= 0x10) && (data[0] <= 0x1A)){
			if(data[0]== GET_ADDRESS(CAM_PMA))
				camispcfg_set_xclk(1,data[1]);
			cam_regState.cam_clock_control = 1;
		}
	}
	else if(isc_obj.current_page == PAGE1) {
		isc_obj.cam_page_dirty |= PAGE1_DIRTY;
		/*sensor info needed by isp*/
		if(((data[0] >= 0x01) && (data[0] <= 0x08)) 
			/* data[0] is assumed to be initialised */
			|| (data[0] == 0x0)){
			cam_regState.cam_sensor_control = 1;
		}
		/* Bit 0 of CAM_PRV_FORMAT_L determine whether
		 * previewing has to be turned on or not.
		 * Checking for it
		 */
		else if(data[0] == GET_ADDRESS(CAM_PRV_FORMAT_L))
			isc_prev_config(data,ndata);
		/*Preview settings*/
		else if((data[0] >= 0x10) &&  (data[0] <= 0x19)){
			cam_regState.cam_prev_config = 1;
		}
		/* Image Capture Settings*/
		else if((data[0] >= 0x20) &&  (data[0] <= 0x2F)){
			cam_regState.cam_img_config = 1;
		}
		/* Tuning Control Settings*/
		else if((data[0] >= 0x40) && (data[0] <= 0x44)){
			cam_regState.cam_tuning_control = 1;
		}
	}
	else if(isc_obj.current_page == PAGE2){
		isc_obj.cam_page_dirty |= PAGE2_DIRTY;
		cam_regState.cam_sensor_control = 1;
	}
	else if(isc_obj.current_page == PAGE3){
		isc_obj.cam_page_dirty |= PAGE3_DIRTY;
		cam_regState.cam_wb_config = 1;
	}
	else if(isc_obj.current_page == PAGEA){
		native_addr_readwrite(data, ndata, attr);
	}
	else if(isc_obj.current_page == PAGEC){
		data32 = data[1] | data[2] << 8 
			| data[3] << 16 | data[4] << 24;
		if((data[0] >= 0x80) && (data[0] <= 0xFE))
			camispcfg_set_ccdc(data[0],data32);
		return ret;
	}
	else if (isc_obj.current_page == PAGEE){
		data32 = data[1] | data[2] << 8 
			| data[3] << 16 | data[4] << 24;
		if((data[0] >= 0x80) && (data[0] <= 0xFE))
			camispcfg_set_ispif(data[0],data32);
		return ret;
	}
	
	DPRINTK_ISCCMD("data[0] = 0x%x,data[1]=0x%x, ndata=%d,curr_page = %d\n"
			,data[0],data[1],ndata,isc_obj.current_page );
	if((data[0] == 0xFE) && (ndata <= 2) &&
			(isc_obj.current_page < 0x10) && (!attr)) {
		/* Apply the prepared settings on the hardware */
		cam_reg_update();
		return ret;
	}
	else if((data[0] == 0x3F) && (ndata <= 2) &&
			(isc_obj.current_page == 0x01) && (!attr)) {
		if(data[1]&0x02) {
			isc_obj.cam_regevent = CAPTURE_STOP_EVENT;
		}
		else if(data[1]&0x01) {
			isc_obj.cam_regevent = CAPTURE_MODE_EVENT;
			isc_obj.prvcontext = 0;
		}
	}

	switch(isc_obj.current_page){
		case PAGE0:
			if(data[0] <= PAGE0_SIZE) {
				if(attr)
					data[1] = page0_regptr[0][data[0]];
				else
					page0_regptr[0][data[0]] = data[1];
			}
			else
			DPRINTK_ISCCMD("Out of bound on page 0 for addr 0x%x",
						data[0]);
			break;
		case PAGE1:
			if(data[0] <= PAGE1_SIZE)
			if(attr)
				data[1] = page1_regptr[0][data[0]];
			else
				page1_regptr[0][data[0]] = data[1];
			else
			DPRINTK_ISCCMD("Out of bound on page 1 for addr 0x%x",
						data[0]);
			break;
		case PAGE2:
			if(data[0] <= PAGE2_SIZE)
			if(attr)
				data[1] = page2_regptr[0][data[0]];
			else
				page2_regptr[0][data[0]] = data[1];
			else
			DPRINTK_ISCCMD("Out of bound on page 2 for addr 0x%x",
						data[0]);
			break;
		case PAGE3:
			if(data[0] <= PAGE3_SIZE)
			if(attr)
				data[1] = page3_regptr[0][data[0]];
			else
				page3_regptr[0][data[0]] = data[1];
			else
			DPRINTK_ISCCMD("Out of bound on page 3 for addr 0x%x",
						data[0]);
			break;
	}
	DPRINTK_ISCCMD("- ISC_CMD isc_regtrans\n");
	return ret;
}

/*
 * Prepares the HW register values to be 
 * configured by camcfg by calling prepare routines.
 */
static int
cam_reg_prepare(void)
{
	/* TBD Currently this CONFIG_EVENTs are not used
	 * May be to remove later 
	 */
	u32 events=0;
	int ret = 0 ;
	u8 inpfmt_change = 0;
	u8 outfmt_change =0;
	u8 inpsz_change =0;
	u8 outsz_change =0;

	DPRINTK_ISCCMD("+ ISC_CMD cam_reg_prepare\n");
	if((isc_obj.cam_page_dirty & PAGE0_DIRTY) == PAGE0_DIRTY) {
		DPRINTK_ISCCMD("page0regptr(campower) = %d\n",
				page0_regptr[0][GET_ADDRESS(CAM_POWER)]);
		if(cam_regState.cam_general_register == 0x1){
			events|=GENERAL_SETTINGS_EVENT;
			isc_obj.active_sensor = page0_regptr[0][0x1];
		}
	}
	if((isc_obj.cam_page_dirty & PAGE1_DIRTY) == PAGE1_DIRTY) {
		if(cam_regState.cam_sensor_control==0x1){
			events|=SENSOR_CONFIG_EVENT;
			check_cam_sensor_control_changes
				(&inpfmt_change,&inpsz_change);
			cam_regState.cam_sensor_control = 0;
		}
		if(cam_regState.cam_prev_config==0x01){
			events|=PREVIEW_CONFIG_EVENT;
			check_cam_preview_config_changes
				(&outfmt_change, &outsz_change);
			cam_regState.cam_prev_config = 0;
		}
		if(cam_regState.cam_img_config==0x01){
			events|=IMAGE_CONFIG_EVENT;
			check_cam_img_config_changes
				(&outfmt_change, &outsz_change);
			cam_regState.cam_img_config = 0;
		}
		if(inpfmt_change || outfmt_change) {
				ret = prepare_prv_format
					(isc_obj.active_sensor);
				if (!pipeline_config) {
					camcfg_set_pipeline();
					pipeline_config = 1;
				}
				if(!ret)
					camcfg_configure_pixformat();
				isc_obj.cam_prv_regevent |=
					PREVIEW_CONFIG_FMT_EVENT;
		}
		if(inpsz_change || outsz_change) {
			if (isc_obj.cam_regevent == PREVIEW_MODE_EVENT) {
				prepare_prv_size(isc_obj.active_sensor);
				isc_obj.cam_prv_regevent |=
					PREVIEW_CONFIG_SZ_EVENT;
			}
			else if (isc_obj.cam_regevent ==CAPTURE_MODE_EVENT) {
				prepare_img_size(isc_obj.active_sensor);
				isc_obj.cam_img_regevent |=
					IMAGE_CONFIG_EVENT;
			}
			else
				printk(KERN_ERR "wrong call to update\n");
		}
		if(cam_regState.cam_tuning_control==0x01)
			events|=TUNING_CONFIG_EVENT;
	}
	if((isc_obj.cam_page_dirty & PAGE3_DIRTY) == PAGE3_DIRTY) {
		if(cam_regState.cam_exp_config==0x01)
			events|=EXPOSURE_CONFIG_EVENT;
		if(cam_regState.cam_wb_config==0x01)
			events|=WHITEBALANCE_CONFIG_EVENT;
	}

	if(cam_regState.cam_clock_control==0x1)
		events|=CLOCK_CONTROL_EVENT;


	//return value should be shad_update << 1 | reg_dirty;
	DPRINTK_ISCCMD("- ISC_CMD cam_reg_prepare\n");
	return ret;
}

/* TBD Need to rewrite this according to the register updates 
 * handling in the interrupt code 
 * Applies the prepared values to the HW to take effect,
 * Also maintains the Isp State Machine from init to 
 * running to stop.
 */
int
cam_reg_update(void)
{
	int i;
	DPRINTK_ISCCMD("+ ISC_CMD cam_reg_update\n");
	DPRINTK_ISCCMD("state = %d, cam_prv_format_l = %d, cam_regevent = %d,\
				regptr = %d",isc_obj.cam_system_state,
				page1_regptr[0][GET_ADDRESS(CAM_PRV_FORMAT_L)],
				isc_obj.cam_regevent,
				page1_regptr[0][GET_ADDRESS(CAM_PRV_FORMAT_L)]);
	switch(isc_obj.cam_system_state)
	{
		case CAM_INIT:
			cam_reg_prepare();
			if((isc_obj.cam_page_dirty & PAGE1_DIRTY) ==
					PAGE1_DIRTY) {
				if(isc_obj.cam_prv_regevent &
						PREVIEW_CONFIG_SZ_EVENT) {
				/* Its too late to do the pixformat configuration here
				 * So its done along with prepare_pixformat itself so that 
				 * prepare_size would be take effective pipeline/pixformat.
				 */
					camcfg_configure_size
						(&isc_imager[isc_obj.active_sensor].isp_prv_size);
					isc_obj.cam_prv_regevent &=
						~(PREVIEW_CONFIG_SZ_EVENT);
				}
				/* Do not start here till cambuf can do the STREAMON
				* without issue*/ 
				if(isc_obj.cam_regevent == PREVIEW_MODE_EVENT){
					if(!camcfg_started)
					camcfg_start();
					camcfg_started = 1;
					isc_obj.cam_system_state =
						CAM_PRV_RUNNING;
				}
				if (isc_obj.cam_img_regevent &
						IMAGE_CONFIG_EVENT) {
					camcfg_configure_size
						(&isc_imager[isc_obj.active_sensor].isp_img_size);
				isc_obj.cam_img_regevent &=
						~(IMAGE_CONFIG_EVENT);
				}
				if(isc_obj.cam_regevent ==CAPTURE_MODE_EVENT){
					if(!camcfg_started) {
					update_cap_time(isc_obj.active_sensor
							,0);
					camcfg_start();
					}
					camcfg_started = 1;
					isc_obj.cam_system_state =
						CAM_IMG_RUNNING;
				}
			}
		break;
		case CAM_POWER_ON:
			/* Use this if any init to be done otherwise its
			 * dummy state */
			isc_obj.cam_system_state = CAM_INIT;
		break;
		case CAM_IMG_RUNNING:
			cam_reg_prepare();
			if(isc_obj.cam_regevent == CAPTURE_MODE_EVENT) {
				camcfg_stop();
				if (isc_obj.cam_img_regevent &
						IMAGE_CONFIG_EVENT) {
					camcfg_configure_size
						(&isc_imager[isc_obj.active_sensor].isp_img_size);
				}
				update_cap_time(isc_obj.active_sensor,0);
				camcfg_start();
				camcfg_started = 1;
			}
			if (isc_obj.cam_regevent == CAPTURE_STOP_EVENT) {
				camcfg_stop();
				update_cap_time(isc_obj.active_sensor,1);
				camcfg_started = 0;
				isc_obj.cam_system_state = CAM_INIT;
			}
			if(isc_obj.cam_regevent == PREVIEW_MODE_EVENT){
				camcfg_stop();
				if(isc_obj.cam_prv_regevent &
						PREVIEW_CONFIG_SZ_EVENT) {
					camcfg_configure_size
						(&isc_imager[isc_obj.active_sensor].isp_prv_size);
					isc_obj.cam_prv_regevent = 0;
				}
				camcfg_start();
				camcfg_started = 1;
				isc_obj.cam_system_state = CAM_PRV_RUNNING;
				DPRINTK_ISCCMD("from prv_running\n");
			}
		break;			
		case CAM_PRV_RUNNING:
			/* We are in the preview state and some update has come.
			 * so stop and apply the changes and then start again
			 */
			cam_reg_prepare();
			if(isc_obj.cam_regevent == PREVIEW_MODE_EVENT){
				camcfg_stop();
				/* No need to respond to CONFIG_FMT_EVENT since
				 * configure_pixformat() is done along with
				 * prepare itself
				 */
				if(isc_obj.cam_prv_regevent &
						PREVIEW_CONFIG_SZ_EVENT) {
					camcfg_configure_size
						(&isc_imager[isc_obj.active_sensor].isp_prv_size);
					isc_obj.cam_prv_regevent = 0;
				}
				camcfg_start();
				camcfg_started = 1;
				DPRINTK_ISCCMD("from prv_running\n");
			}
			if(isc_obj.cam_regevent == PREVIEW_STOP_EVENT) {
				DPRINTK_ISCCMD("going to stop\n");
				/* PRV_FORMAT_L[0] is 0 so stop the camcfg*/
				camcfg_stop();
				camcfg_started = 0;
				isc_obj.cam_system_state = CAM_INIT;
			}
			if(isc_obj.cam_regevent == CAPTURE_MODE_EVENT) {
				camcfg_stop();
				camcfg_started = 0;
				if (isc_obj.cam_img_regevent &
						IMAGE_CONFIG_EVENT) {
					camcfg_configure_size
						(&isc_imager[isc_obj.active_sensor].isp_img_size);
					isc_obj.cam_img_regevent = 0;
				}
				update_cap_time(isc_obj.active_sensor,0);
				camcfg_start();
				camcfg_started = 1;
				isc_obj.cam_system_state = CAM_IMG_RUNNING;
			}
		break;
		case CAM_POWER_OFF:
			DPRINTK_ISCCMD("stopped in POWER OFF\n");
			camcfg_stop();
			update_cap_time(isc_obj.active_sensor,1);
			camcfg_started = 0;
			for(i = 0;i<=PAGE0_SIZE;i++)
			curr_page0_regptr[0][i] = page0_regptr[0][i]
				= def_page0_regptr[0][i];
			for(i = 0;i<=PAGE1_SIZE;i++)
			curr_page1_regptr[0][i] = page1_regptr[0][i]
				= def_page1_regptr[0][i];
			for(i = 0;i<=PAGE2_SIZE;i++)
			curr_page2_regptr[0][i] = page2_regptr[0][i]
				= def_page2_regptr[0][i];
			for(i = 0;i<=PAGE3_SIZE;i++)
			curr_page3_regptr[0][i] = page3_regptr[0][i]
				= def_page3_regptr[0][i];
			isc_obj.cam_system_state = CAM_INIT;
		break;
	};

	
	for(i = 0;i<=PAGE0_SIZE;i++)
	curr_page0_regptr[0][i] = page0_regptr[0][i];
	for(i = 0;i<=PAGE1_SIZE;i++)
	curr_page1_regptr[0][i] = page1_regptr[0][i];
	for(i = 0;i<=PAGE2_SIZE;i++)
	curr_page2_regptr[0][i] = page2_regptr[0][i];
	for(i = 0;i<=PAGE3_SIZE;i++)
	curr_page3_regptr[0][i] = page3_regptr[0][i];
	cam_regState.cam_general_register= 0;
	cam_regState.cam_sensor_control= 0;
	cam_regState.cam_prev_config= 0;
	cam_regState.cam_tuning_control= 0;
	cam_regState.cam_exp_config= 0;
	cam_regState.cam_wb_config = 0;
	cam_regState.cam_clock_control= 0;
	DPRINTK_ISCCMD("- ISC_CMD cam_reg_update\n");
	return 0;
}

/* 
 * Indicates the system_state changes.
 */
static int
cam_power(u8* data, u8 read_flag)
{
	DPRINTK_ISCCMD("+ ISC_CMD cam_power *data = %d\n",*data);
	if((*data & 0x3) == 0x00) {
		/* Power ON Run Cam */
		if (isc_obj.cam_system_state == CAM_POWER_SUSP) {
			camcfg_out_of_standby();
			isc_obj.cam_system_state = isc_obj.cam_system_oldstate;
		}
	}
	else if((*data & 0x3) == 0x01) {
		/* Power OFF Stop Cam HW reg reset to default HW values */
		isc_obj.cam_system_state = CAM_POWER_OFF;
	}
	else if((*data & 0x3) == 0x3) {
		/* Sleep  low power Suspend Cam HW reg 
		 * maintain the configured values
		 */
		if (isc_obj.cam_system_state) {
			camcfg_in_standby();
		}
		isc_obj.cam_system_oldstate = isc_obj.cam_system_state;
		isc_obj.cam_system_state = CAM_POWER_SUSP;
	}
	DPRINTK_ISCCMD("- ISC_CMD cam_power\n");
	return 0;
}

/* 
 * Triggers the Preview mode changes.
 */
static void
isc_prev_config(u8 *data, u8 ndata)
{
	DPRINTK_ISCCMD("isc_prev_config %d, %d",data[0],data[1]);
	cam_regState.cam_prev_config = 1;
	switch(data[0])
	{
	case GET_ADDRESS(CAM_PRV_FORMAT_L):
		if(data[1] & 0x1){
			/* TBD set this event only if this is the
			 * change w.r.t previous config
			 */
			isc_obj.cam_regevent = PREVIEW_MODE_EVENT;
		}
		else{
			isc_obj.cam_regevent = PREVIEW_STOP_EVENT;
		}
	break;
	};
}

static void
check_cam_sensor_control_changes(u8 *inpfmt_change, u8 *inpsz_change)
{
	u8 sensorfmtaddr = GET_ADDRESS(CAM_SENSOR_FORMAT);
	u8 addr;
	
	if(curr_page1_regptr[0][sensorfmtaddr] != page1_regptr[0][sensorfmtaddr])					
		*inpfmt_change = 1;

	/* addr shall be from CAM_SENSOR_WIDTH to CAM_IMG_VIEWAREA_HEIGHT */
	for(addr = 0; addr <= 0x07;addr++)
		if(curr_page1_regptr[0][addr] != page1_regptr[0][addr])
			*inpsz_change = 1;
		else
			continue;
}

static u32 inline
get_prv_fmt(u8 *page1_ptr)
{
	/* Including FORMAT_H makes it mandatory for the user to send
	a default value for this so that prepare_format shall be called */
	return ((page1_ptr[GET_ADDRESS(CAM_PRV_FORMAT_H)] << 8)
		| page1_ptr[GET_ADDRESS(CAM_PRV_FORMAT_L)]);
}

static void
check_cam_preview_config_changes(u8 *outfmt_change, u8 *outsz_change)
{
	/* if the format requested i 0 ie SENSOR_BAYER10 it
	 * has to be compared again a 0xFF or -1 to 
	 * set the BAYER format.
	 */
	u32 prvfmt_old = get_prv_fmt(curr_page1_regptr[0]);
	u32 prvfmt_new = get_prv_fmt(page1_regptr[0]);
	u8 addr;

	DPRINTK_ISCCMD("prvfmt_old, new = %d, %d",prvfmt_old, prvfmt_new);
	
	if(prvfmt_old != prvfmt_new)
		*outfmt_change = 1;

	/* addr shall be from CAM_SENSOR_WIDTH to CAM_IMG_VIEWAREA_HEIGHT */
	for(addr = 0x10; addr <= 0x13;addr++)
		if(curr_page1_regptr[0][addr] != page1_regptr[0][addr])

			*outsz_change = 1;
		else
			continue;
}
static u32 inline
get_img_fmt(u8 *page1_ptr)
{
	return ((page1_ptr[GET_ADDRESS(CAM_IMG_CAP_FORMAT_H)] << 8)
		| page1_ptr[GET_ADDRESS(CAM_IMG_CAP_FORMAT_L)]);
}

/*
 * Calculates the preview inp/out data format
 * from the register values.
 */
static int
prepare_prv_format(u8 curr_sensor)
{
	struct isc_sensor_info sensor;
	enum cam_fmt outfmt;
	u32 prv_fmt;
	void *pix_inpfmt;
	int ret = 0;
	int sensor_format;
	/* Assume till a parallel/serial field
	 * is added in the register map 
	 */
	if (page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)] == 0xFF) {
		ret = 1;
		return ret;
	}
	sensor.interface = (((page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)])
		& CAM_SENSOR_FORMAT_SENSOR_INTERFACE_MASK)
		>> CAM_SENSOR_FORMAT_SENSOR_INTERFACE_SHIFT);

	sensor.dataformat = (((page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)])
		& CAM_SENSOR_FORMAT_SENSOR_FORMAT_MASK)
		>> CAM_SENSOR_FORMAT_SENSOR_FORMAT_SHIFT);
	
	if(sensor.dataformat == SENSOR_BAYER10){
		sensor.datasize = 10;
	}
	else if(sensor.dataformat == SENSOR_BAYER9){
		ret = -1 ; /* KErrNot supported format */
		return ret;
	}
	else if(sensor.dataformat == SENSOR_YUV422){
		sensor.datasize = 10;
	}
	else {/* default settings */
		sensor.dataformat = SENSOR_BAYER10;
		sensor.datasize = 10;
	}
	isc_imager[isc_obj.active_sensor].sensor_info.interface
						= sensor.interface;
	isc_imager[isc_obj.active_sensor].sensor_info.dataformat
						= sensor.dataformat;
	isc_imager[isc_obj.active_sensor].sensor_info.datasize
						= sensor.datasize;
	
	/* THIS SENSOR info has to be sent for pix format */
	pix_inpfmt = (void *)(&(isc_imager[isc_obj.active_sensor].sensor_info));

	prv_fmt = get_prv_fmt(page1_regptr[0]);
	if (prv_fmt == 0xFFFF) {
		prv_fmt = get_img_fmt(page1_regptr[0]);
		if (prv_fmt == 0xFFFF) {
			ret = 1;
			return ret;
		}
	}
	if(prv_fmt & CAM_PRV_FORMAT_L_PRV_FORMAT)
		outfmt = PIX_FMT_BAYER10_GrRBGb;
	else 
		/* cam_fmt is aligned with the PRV_YUV_FORMAT in reg map */
		outfmt = (prv_fmt & CAM_PRV_FORMAT_L_PRV_YUV_FORMAT_MASK) >>
				CAM_PRV_FORMAT_L_PRV_YUV_FORMAT_SHIFT;
		

	camcfg_prepare_pixformat(outfmt, pix_inpfmt);

	return ret;
}

/*
 * Calculates the preview inp/out resoultion
 * from the register values.
 */
static void
prepare_prv_size(u8 curr_sensor)
{
	struct frame_size prv, sensor;

	/*u8 *page_ptr; TBD for secondary sensor*/
	DPRINTK_ISCCMD("+ ISC_CMD prepare_prv_size\n");

	/* TBD To support Secondary sensor */
#if 0
	if(curr_sensor == PRIMARY_SENSOR)
		page_ptr = page1_regptr;
	else /* Secondary sensor */
		page_ptr = page5_regptr;
#endif
	sensor.width = (page1_regptr[0][GET_ADDRESS(CAM_SENSOR_WIDTH_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_SENSOR_WIDTH_L)];
	sensor.height = (page1_regptr[0][GET_ADDRESS(CAM_SENSOR_HEIGHT_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_SENSOR_HEIGHT_L)];
	prv.width = (page1_regptr[0][GET_ADDRESS(CAM_PRV_WIDTH_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_PRV_WIDTH_L)];
	prv.height = (page1_regptr[0][GET_ADDRESS(CAM_PRV_HEIGHT_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_PRV_HEIGHT_L)];

	DPRINTK_ISCCMD("sw%d sh%d pw%d ph%d\n",sensor.width,sensor.height
				,prv.width,prv.height);
	if ((sensor.width == 0) || (sensor.height == 0) ||
			(prv.width == 0) || (prv.height == 0))
		return;
	camcfg_prepare_size(&prv, &sensor,
			&(isc_imager[curr_sensor].isp_prv_size));
	DPRINTK_ISCCMD("- ISC_CMD prepare_prv_size\n");
}



static void
check_cam_img_config_changes(u8 *outfmt_change, u8 *outsz_change)
{
	u32 imgfmt_old = get_img_fmt(curr_page1_regptr[0]);
	u32 imgfmt_new = get_img_fmt(page1_regptr[0]);
	u8 addr;

	DPRINTK_ISCCMD("imgfmt_old, new = %d, %d",imgfmt_old, imgfmt_new);
	
	if(imgfmt_old != imgfmt_new)
		*outfmt_change = 1;

	/* addr shall be from CAM_SENSOR_WIDTH to CAM_IMG_VIEWAREA_HEIGHT */
	for(addr = 0x20; addr <= 0x23 ;addr++)
		if(curr_page1_regptr[0][addr] != page1_regptr[0][addr])
			*outsz_change = 1;
		else
			continue;
}
static int
prepare_img_format(u8 curr_sensor)
{
	struct isc_sensor_info sensor;
	enum cam_fmt outfmt;
	u32 img_fmt;
	void *pix_inpfmt;
	int ret = 0;
	int sensor_format;
	/* Assume till a parallel/serial field
	 * is added in the register map 
	 */
	if (page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)] == 0xFF) {
		ret = 1;
		return ret;
	}
	sensor.interface = (((page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)])
		& CAM_SENSOR_FORMAT_SENSOR_INTERFACE_MASK)
		>> CAM_SENSOR_FORMAT_SENSOR_INTERFACE_SHIFT);

	sensor.dataformat = (((page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)])
		& CAM_SENSOR_FORMAT_SENSOR_FORMAT_MASK)
		>> CAM_SENSOR_FORMAT_SENSOR_FORMAT_SHIFT);
	
	if(sensor.dataformat == SENSOR_BAYER10){
		sensor.datasize = 10;
	}
	else if(sensor.dataformat == SENSOR_BAYER9){
		ret = -1 ; /* KErrNot supported format */
		return ret;
	}
	else{/* default settings */
		sensor.dataformat = SENSOR_BAYER10;
		sensor.datasize = 10;
	}
	isc_imager[isc_obj.active_sensor].sensor_info = sensor;
	/* THIS SENSOR info has to be sent for pix format */
	pix_inpfmt = (void *)(&(isc_imager[isc_obj.active_sensor].sensor_info));

	img_fmt = get_img_fmt(page1_regptr[0]);
	if (img_fmt == 0xFFFF) {
		ret = 1;
		return ret;
	}
	if(img_fmt & CAM_PRV_FORMAT_L_PRV_FORMAT)
		outfmt = PIX_FMT_BAYER10_GrRBGb;
	else 
		/* cam_fmt is aligned with the PRV_YUV_FORMAT in reg map */
		outfmt = (img_fmt & CAM_PRV_FORMAT_L_PRV_YUV_FORMAT_MASK) >>
				CAM_PRV_FORMAT_L_PRV_YUV_FORMAT_SHIFT;
	camcfg_prepare_pixformat(outfmt, pix_inpfmt);

	return ret;
}

static void
prepare_img_size(u8 curr_sensor)
{
	struct frame_size img, sensor;

	/*u8 *page_ptr; TBD for secondary sensor*/
	
	DPRINTK_ISCCMD("+ ISC_CMD prepare_prv_size\n");

	/* TBD To support Secondary sensor */
#if 0
	if(curr_sensor == PRIMARY_SENSOR)
		page_ptr = page1_regptr;
	else /* Secondary sensor */
		page_ptr = page5_regptr;
#endif
	sensor.width = (page1_regptr[0][GET_ADDRESS(CAM_SENSOR_WIDTH_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_SENSOR_WIDTH_L)];
	sensor.height = (page1_regptr[0][GET_ADDRESS(CAM_SENSOR_HEIGHT_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_SENSOR_HEIGHT_L)];
	img.width = (page1_regptr[0][GET_ADDRESS(CAM_IMG_CAP_WIDTH_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_IMG_CAP_WIDTH_L)];
	img.height = (page1_regptr[0][GET_ADDRESS(CAM_IMG_CAP_HEIGHT_H)] << 8)
			| page1_regptr[0][GET_ADDRESS(CAM_IMG_CAP_HEIGHT_L)];

	DPRINTK_ISCCMD("sw%d sh%d pw%d ph%d\n",sensor.width,sensor.height
				,img.width,img.height);
	if ((sensor.width == 0) || (sensor.height == 0) ||
			(img.width == 0) || (img.height == 0))
		return;
	camcfg_prepare_size(&img, &sensor,
			&(isc_imager[curr_sensor].isp_img_size));
	DPRINTK_ISCCMD("- ISC_CMD prepare_prv_size\n");
}

static void 
native_addr_readwrite(u8 *data, u8 ndata, u8 attr)
{
	static volatile u32 native_addr = 0,data32;
	static u8 auto_inc = 0;
	
	if(data[0] == CAM_NATIVE_ADDR){
		native_addr = 0;
		native_addr = data[1]| data[2]<< 8
				| data[3]<< 16 | data[4] << 24;
	}
	else if (data[0] == CAM_NATIVE_DATA_WRITE){
		if(auto_inc)
			native_addr + 4;
		omap_writel(native_addr,
				data[1]| data[2]<< 8
				| data[3]<< 16 | data[4] << 24);
			
	}
	else if (data[0] == CAM_NATIVE_DATA_READ){
		if(auto_inc)
			native_addr++;
		data32 = omap_readl(native_addr);
	}
	else if (data[0] == CAM_NATIVE_AUTO_INCREMENT){
		auto_inc = data[1];
	}
}
static void 
update_cap_time(u8 curr_sensor,u8 reset)
{
	struct captureparams cap_params;
	if(reset){
		cap_params.cap_delay_1stshot = 0;
		cap_params.cap_shots = 0;
		cap_params.cap_delay_intershot = 0;
	}
	else{
		cap_params.cap_delay_1stshot = 
			page1_regptr[0][GET_ADDRESS(CAM_IMG_CAP_DELAY_FRAME)];
		cap_params.cap_shots = 
			page1_regptr[0][GET_ADDRESS(CAM_IMG_CAP_SHOTS)];
		cap_params.cap_delay_intershot = 
			page1_regptr[0][GET_ADDRESS(CAM_IMG_CAP_PERIOD)];
	}
	camcfg_update_capture_params(&cap_params);
}

void
isc_resetpoweron(void){
	DPRINTK_ISCCMD("+ ISC_CMD isc_resetpoweron\n");
	camcfg_init();
	DPRINTK_ISCCMD("- ISC_CMD isc_resetpoweron\n");
}

void
isc_prv_defaultsettings(void){
	struct isc_sensor_info sensor;

	DPRINTK_ISCCMD("+ ISC_CMD isc_prv_defaultsettings\n");
	sensor.interface = SENSOR_PARLL;
	if((((page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)]) & 0x1C) >> 2)
			== SENSOR_BAYER10)
		sensor.datasize = 10;
	if((((page1_regptr[0][GET_ADDRESS(CAM_SENSOR_FORMAT)]) & 0x1C) >> 2)
			== SENSOR_BAYER9)
		sensor.datasize = 9;
	sensor.dataformat = SENSOR_BAYER10;
	sensor.datasize = 10;
	isc_imager[isc_obj.active_sensor].sensor_info = sensor;
	camcfg_settings();
	DPRINTK_ISCCMD("- ISC_CMD isc_prv_defaultsettings\n");
}

int
isc_open(void){
	int i =0;
	for(i = 0;i<=PAGE0_SIZE;i++)
	curr_page0_regptr[0][i] = page0_regptr[0][i] = def_page0_regptr[0][i];
	for(i = 0;i<=PAGE1_SIZE;i++)
	curr_page1_regptr[0][i] = page1_regptr[0][i] = def_page1_regptr[0][i];
	for(i = 0;i<=PAGE2_SIZE;i++)
	curr_page2_regptr[0][i] = page2_regptr[0][i] = def_page2_regptr[0][i];
	for(i = 0;i<=PAGE3_SIZE;i++)
	curr_page2_regptr[0][i] = page3_regptr[0][i] = def_page3_regptr[0][i];
	isc_resetpoweron();
//	isc_prv_defaultsettings();
	/* Clocks are initialised so register 
	 * updation shall start
	 */
	isc_obj.cam_system_state = CAM_INIT;
	return 0;
}

int
isc_close(void){
	camcfg_cleanup();
	isc_obj.cam_system_state = 0;
	pipeline_config = 0;
	return 0;
}


static int __init 
isccmd_init(void){
	return 0;
}

static void __exit 
isccmd_exit(void)
{
}

/* TBD Try to make ISC_CMD a module seperated from CAMCFG */
module_init(isccmd_init);
module_exit(isccmd_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP State Controller Library");
MODULE_LICENSE("GPL");


