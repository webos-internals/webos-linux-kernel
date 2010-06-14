/*
 * drivers/media/video/omap/ise/omap34xxcam_palkern.c
 *
 * PAL kern interface for TI's OMAP3430 Camera ISP
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
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/init.h>
#include <asm/delay.h>
#include <linux/delay.h>

#include <asm/arch/gpio.h>
#include <asm/arch/gci/omap34xxcam_palkern.h>
#include <asm/arch/gci/isc_cmd.h>
#include <asm/arch/omap34xx.h>

#include <asm/arch/resource.h>

#include <asm/arch/twl4030.h>
#include "../isp/isp.h"

#ifdef PAL_KERN_DEBUG
#define FN_IN printk("%s Entry\n",__FUNCTION__);
#define FN_OUT(ARG) printk("%s[%d]:Exit(%d)\n",__FUNCTION__,__LINE__,ARG);
#else
#define FN_IN
#define FN_OUT(ARG)
#endif

#ifdef PAL_KERN_DEBUG
#define DPRINTK_PALKERN(format,...)\
	printk("PALKERN: " format, ## __VA_ARGS__)
#else
#define DPRINTK_PALKERN(format,...)
#endif

#define PALKERN_IS_OPEN		1

#define NUM_CAM_GPIOS		5

static unsigned int palkern_status = 0;
static int i2c_slaveid = -1;
static int prim_i2c_slaveid = -1;
static int prim_i2c_slaveid_focus = -1;
static int prim_i2c_num;
static u32 prim_i2c_busspeed_khz;
static u8 acquired_gpios[NUM_CAM_GPIOS] = {0xFF,0xFF,0xFF,0xFF,0xFF};
static u8 reg_trans_type;

static int omap_palkern_probe(struct platform_device *pdev);
static u8 i2c_data[8];

struct constraint_handle *co_opp_camera_latency;

static struct constraint_id cnstr_id_latency = {
	.type = RES_LATENCY_CO,
	.data = (void *)"latency",
};


static int 
process_sensor_passt_cmd(char *data,u8 len, u8 attr)
{
	static u8 regadd_recv = 0;
	u8 count = 0, j= 0, ret = 0;
	switch (data[0])
	{
	case GET_ADDRESS(CAM_PASST_SLAVE_ADDR):
		if(i2c_slaveid != -1) {
			/* Should not attach the slave id here
			 * since it has to come through Open 
			 * Just reassign the slaveid for debugging purpose
			 * if it has a valid id already
			 */
			i2c_slaveid = data[1];
		}
	return 0;
	break;
	case GET_ADDRESS(CAM_PASST_FORMAT):
		p_i2c_set_reglength(data[1] & CAM_PASST_FORMAT_MASK);
		p_i2c_set_datalength((data[1] >>
			CAM_PASST_FORMAT_DATA_WIDTH_SHIFT)
			& CAM_PASST_FORMAT_MASK);
	return 0;
	break;
	case GET_ADDRESS(CAM_PASST_REG_ADDR_L):
		regadd_recv = 1;
		if (i2c_slaveid == -1 ) {
			printk(KERN_ERR "i2c slave id not yet assigned\n");
			return -EINVAL;
		}
		for(count = 0;count < len - 1;count++){
			i2c_data[count] = data[count + 1];
		}
	return 0;
	break;
	case GET_ADDRESS(CAM_PASST_REG_DATA_L):
		if (regadd_recv) {
			regadd_recv = 0;
			for(j = 0;j < len - 1;j++){
				i2c_data[count++] = data[j+1];
			}
			if (attr == ISE_PAL_REG_TRANS_TYPE_WRITE) {
				ret = p_i2c_write_slave(
					i2c_slaveid,i2c_data,
					count);
				if (ret) {
					printk (KERN_ERR "ERROR IN I2C\
							WRITE\n");
					return -2;
				}
			}
			else if (attr == ISE_PAL_REG_TRANS_TYPE_READ) {
				ret = p_i2c_read_slave(
					i2c_slaveid,i2c_data,
					count);
				return ret;
			}
			else {
				printk (KERN_ERR "INVALID ARGUMENT TO\
						palkern_ioctl\n");
				return -EINVAL;
			}
		}
		else {
			printk(KERN_ERR"Wrong sequence of commands\n");
			return -EINVAL;
		}
	break;
	default:
		return -EINVAL; /* invalid addr for Page6*/
	break;
	};/* End of switch data[0]*/
	return 0;
}

#define DEBUG_BASE		0x08000000
#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

static void __iomem *fpga_map_addr;

static void
enable_fpga_vio_1v8(u8 enable)
{
	u16 reg_val;
	fpga_map_addr = ioremap(DEBUG_BASE, 4096);
	reg_val = readw(fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
	/* Ensure that the SPR_GPIO1_3v3 is 0 - powered off.. 1 is on */
	if (reg_val & FPGA_SPR_GPIO1_3v3) {
		reg_val &= ~FPGA_SPR_GPIO1_3v3;
		reg_val |= FPGA_GPIO6_DIR_CTRL; /* output mode */
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
		/* give a few milli sec to settle down
		 * Let the sensor also settle down.. if required.. 
		 */
		if (enable)
			mdelay(10);
	}
	if (enable) {
		reg_val |= FPGA_SPR_GPIO1_3v3 | FPGA_GPIO6_DIR_CTRL;
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
	}
	/* Vrise time for the voltage - should be less than 1 ms */
	mdelay(1);
}


static void board_init(int t2_vaux)
{
	//isp_get();
	omap_writew(0x11c,0x480020bc);
	omap_writew(0x11c,0x480020b6);
	if ((t2_vaux >> 8))
		enable_fpga_vio_1v8(1);
	else if(t2_vaux == VAUX_2_8_V) {
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);

	}else if(t2_vaux == VAUX_DEV_GRP_P1) {
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			VAUX_DEV_GRP_P1,TWL4030_VAUX2_DEV_GRP);
	udelay(100);
	}
}

static int palkern_open(struct inode *inode, struct file *file)
{
	FN_IN;
	if (palkern_status & PALKERN_IS_OPEN){
		FN_OUT(-EBUSY);
		return -EBUSY;
	}
	prim_i2c_slaveid = -1;
	prim_i2c_slaveid_focus = -1;
	prim_i2c_num = -1;
	prim_i2c_busspeed_khz = -1;
	isc_open();
	palkern_status |= PALKERN_IS_OPEN;
	constraint_set(co_opp_camera_latency, CO_LATENCY_MPURET_COREON);

	FN_OUT(0);
	return 0;
}
static int palkern_release(struct inode *inode, struct file *file)
{
	char data[2];
	int i;
	FN_IN;
	palkern_status &= ~PALKERN_IS_OPEN;
	data[0] = GET_ADDRESS(PAGE_SWITCH);
	data[1] = PAGE0;
	isc_regtrans(data, 2, 0);
	data[0] = GET_ADDRESS(CAM_POWER);
	data[1] = 0x1;
	isc_regtrans(data, 2, 0);
	data[0] = 0xFD;
	data[1] = 0x1;
	isc_regtrans(data, 2, 0);
	data[0] = 0xFE;
	data[1] = 0x1;
	isc_regtrans(data, 2, 0);
	isc_close();
	for(i = 0;i < NUM_CAM_GPIOS; i++)
		if(acquired_gpios[i]!= 0xFF){
			omap_free_gpio(acquired_gpios[i]);
			acquired_gpios[i] = 0xFF;
		}
	if(prim_i2c_slaveid != -1)
		p_i2c_detach_slave(prim_i2c_slaveid);
	if(prim_i2c_slaveid_focus != -1)
		p_i2c_detach_slave(prim_i2c_slaveid_focus);
	constraint_remove(co_opp_camera_latency);
	FN_OUT(0);
	return 0;
}

/* Handles ioctls such as GEN_CAM_PAL_REG_TRANSFER,GEN_CAM_PAL_GPIO_GET
 * GEN_CAM_PAL_GPIO_SET
 */
static int palkern_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	static int sensor_config = 0,count = 0;
	switch(cmd) {
	case GEN_CAM_PAL_REG_OPEN:
	{
		ISE_PAL_REG_CONFIG_T cfg;
		DPRINTK_PALKERN("reg open\n");
		if (copy_from_user(&cfg,
			(ISE_PAL_REG_CONFIG_T*)arg,
			sizeof(ISE_PAL_REG_CONFIG_T))) {
			printk(KERN_ERR "copyfromuser i2c config error\n");
			return -EFAULT;
		}
		reg_trans_type = cfg.reg_type;
		if(cfg.reg_type == ISE_PAL_REG_TYPE_I2C){
		if((prim_i2c_slaveid == -1)){
			prim_i2c_slaveid = cfg.type_config.i2c_config.write_addr;
			prim_i2c_num = cfg.type_config.i2c_config.i2c_num;
			prim_i2c_busspeed_khz = 
				cfg.type_config.i2c_config.bus_speed_khz;
			pseudoi2c_init(prim_i2c_num);
			p_i2c_attach_slave(prim_i2c_slaveid);
		}
		if((prim_i2c_slaveid != -1) && 
			(cfg.type_config.i2c_config.write_addr 
			!= prim_i2c_slaveid) &&
			(prim_i2c_slaveid_focus == -1)){
			prim_i2c_slaveid_focus = 
				cfg.type_config.i2c_config.write_addr;
			p_i2c_attach_slave(prim_i2c_slaveid_focus);
		}
		i2c_slaveid = cfg.type_config.i2c_config.write_addr;
		}
		return ret;
	}
	case GEN_CAM_PAL_REG_CLOSE:
	{	i2c_slaveid = 0;
		/* This wont get called at all untill paluser does it*/
		return ret;
	}
	case GEN_CAM_PAL_REG_TRANSFER:
	{
		struct pal_nregtrans pal_reginfo;
		ISE_PAL_REG_TRANS_T *regtrans;
		u8 data[5];
		int i,j;
		DPRINTK_PALKERN("reg transfer\n");
		if (copy_from_user(&pal_reginfo,(struct pal_nregtrans *)arg,
			sizeof(struct pal_nregtrans))) {
			printk(KERN_ERR "copyfromuser nregtrans error\n");
			return -EFAULT;
		}
		regtrans = (ISE_PAL_REG_TRANS_T*)kmalloc(sizeof
			(ISE_PAL_REG_TRANS_T)*pal_reginfo.nregtrans,
			GFP_KERNEL);
	switch(reg_trans_type)
	{
	case ISE_PAL_REG_TYPE_I2C:
	for(i = 0;i<pal_reginfo.nregtrans;i++) {
		if (copy_from_user(&regtrans[i],
			&(pal_reginfo.alltrans[i]),
			sizeof(ISE_PAL_REG_TRANS_T))) {
				printk(KERN_ERR"i2ctrans error in\
						copyfromuser");
				return -EFAULT;
		}
		for(j = 0;j<regtrans[i].size;j++){
			data[j] = regtrans[i].data[j];
		}
		for(j = 0;j < regtrans[i].size - 1;j++){
			i2c_data[j] = data[j + 1];
		}
		if(regtrans[i].trans_type == ISE_PAL_REG_TRANS_TYPE_WRITE){
			
			ret = p_i2c_write_slave(i2c_slaveid,i2c_data,
				regtrans[i].size-1,data[0]);
			if(ret)
				printk(KERN_ERR "I2C Write error = 0x%x",ret);
		}
		else if(regtrans[i].trans_type == ISE_PAL_REG_TRANS_TYPE_READ) {
			ret = p_i2c_read_slave(prim_i2c_slaveid,i2c_data,
				regtrans[i].size-1,data[0]);
			if (!ret) {
				DPRINTK_PALKERN("The read value is ");
				DPRINTK_PALKERN("%x %x \n",(i2c_data[0] | (i2c_data[1] << 8)),i2c_data[2]);
				for(j = 1;j<regtrans[i].size;j++) {
					regtrans[i].data[j] = i2c_data[j - 1];
				}
				if(copy_to_user(
					&(pal_reginfo.alltrans[i]),
					&regtrans[i],
					sizeof(ISE_PAL_REG_TRANS_T))) {
						printk(KERN_ERR "copytouser\
							i2cread error\n");
						return -EFAULT;
				}
			}
				else return ret; /* i2c error*/
		}
	}/* forloop end of all regtrans */
	break;
	case ISE_PAL_REG_TYPE_CAM:
	for(i = 0;i<pal_reginfo.nregtrans;i++) {
		if (copy_from_user(&regtrans[i],&(pal_reginfo.alltrans[i]),
			sizeof(ISE_PAL_REG_TRANS_T))) {
				return -EFAULT;
		}
		for(j = 0;j<regtrans[i].size;j++){
			data[j] = regtrans[i].data[j];
		}
		if(data[0] == GET_ADDRESS(PAGE_SWITCH)){
			if (data[1] == PAGE6) {
				sensor_config = 1;
				continue;
			}
			else
				sensor_config = 0;
		}

		/* Used for debugging sensor values through PAGE6 */
		/* Register address comes in a seperate transaction followed
		 * by the register data transaction */
		if (sensor_config){
			ret = process_sensor_passt_cmd(data,regtrans[i].size
					,regtrans[i].trans_type);
			if(regtrans[i].trans_type ==
					ISE_PAL_REG_TRANS_TYPE_READ) {
				if (!ret) {
				DPRINTK_PALKERN("The read value is ");
				DPRINTK_PALKERN("%x %x \n",data[2],data[3]);
				for(j = 0;j<regtrans[i].size - 1;j++) {
					regtrans[i].data[j] = i2c_data[count++];
				}
				if(copy_to_user(
					&(pal_reginfo.alltrans[i]),
					&regtrans[i],
					sizeof(ISE_PAL_REG_TRANS_T))) {
						printk(KERN_ERR "copytouser\
							i2cread error\n");
						return -EFAULT;
					}
				}
				else return ret; /* i2c error*/
			}
		}
		else {
			/* reg_trans_type CAMCFG + ISC*/
			isc_regtrans(data,
				regtrans[i].size,
				regtrans[i].trans_type);
		}
	}/* forloop end of all regtrans */
	break;
	};/* End of switch */
		kfree(regtrans);
		return ret;
	}
	case GEN_CAM_PAL_GPIO_SET:
	{
		unsigned int i,acquired_index = 0;;
		struct gpio_data gpio_info;
		if (copy_from_user(&gpio_info,
			(struct gpio_data *)arg,
			sizeof(struct gpio_data))) {
			printk(KERN_ERR "copyfromuser gpio_data error\n");
			return -EFAULT;
		}
		for(i = 0; i< NUM_CAM_GPIOS;i++){
			if(acquired_gpios[i] == gpio_info.gpio_num)
				break;
			if(acquired_gpios[i] != 0xFF)
				acquired_index++;
		}
		if (i == NUM_CAM_GPIOS){
			/* Request and configure gpio pins*/
			if (omap_request_gpio(gpio_info.gpio_num) != 0)
				/* Reserved by another peripheral*/
				return -EIO;
			else{
				acquired_gpios[acquired_index] = 
						gpio_info.gpio_num;
			}
		}
		/* Set the direction of GPIO */
		DPRINTK_PALKERN("GPIO num = %d, gpio val = %d", 
					gpio_info.gpio_num,gpio_info.value);
		DPRINTK_PALKERN("acquired_gpios[%d]= %d",
					acquired_index,
					acquired_gpios[acquired_index]);
		omap_set_gpio_direction(gpio_info.gpio_num,
					gpio_info.is_input);
		omap_set_gpio_dataout(gpio_info.gpio_num,gpio_info.value);
		if (gpio_info.gpio_num == 98) {
			CONTROL_PADCONF_CAM_FLD = 0x01003B1C;
			omap_set_gpio_direction(gpio_info.gpio_num, GPIO_DIR_INPUT);
		}
		return 0;
	}
	case GEN_CAM_PAL_GPIO_GET:
	{
		struct gpio_data gpio_info;
		struct gpio_data *arg_gpio_data = (struct gpio_data*)arg;
		if (copy_from_user(&gpio_info,
			(struct gpio_data *)arg,
			sizeof(struct gpio_data))) {
			printk(KERN_ERR "copyfromuser\
					set_gpio_data error\n");
			return -EFAULT;
		}
		/* Hardcode as of now since no API for omap_get_dataout()*/
		arg_gpio_data->value = 1; 
		gpio_info.value = 1;
		/* TBD check the semantics of copy_to_user */
		if (copy_to_user(&gpio_info, (struct gpio_data *)arg,
			sizeof(struct gpio_data))) {
			printk(KERN_ERR "copytouser get_gpio_data error\n");
			return -EFAULT;
		}
		return 0;
	}
	case GEN_CAM_PAL_T2_VAUX_SET:
		board_init(arg);
		return 0;
	case GEN_CAM_PAL_UDELAY:
		udelay((int)arg);
		return 0;
	default :
		return -EINVAL;
	}
}

static struct file_operations palkern_fops = {
	ioctl:palkern_ioctl,
	open:palkern_open,
	release:palkern_release,
};

static struct miscdevice palkern_dev =
	{ PALKERN_MINOR, "omap_palkern", &palkern_fops };

static int omap_palkern_probe(struct platform_device *pdev)
{
	int ret = 0;
	FN_IN
	ret = misc_register(&palkern_dev);
	if (ret) {
		printk(KERN_ERR "....PALKERN registration failed!!!!\n");
		return ret;
	}
	FN_OUT(ret);
	return ret;
}

static int omap_palkern_suspend(struct platform_device *pdev, pm_message_t state)
{
	char data[2];
	int i;
	if(palkern_status & PALKERN_IS_OPEN){
		data[0] = GET_ADDRESS(PAGE_SWITCH);
		data[1] = PAGE0;
		isc_regtrans(data, 2, 0);
		data[0] = GET_ADDRESS(CAM_POWER);
		data[1] = 0x3;
		isc_regtrans(data, 2, 0);
	}
	return 0;
}

static int omap_palkern_resume(struct platform_device *dev)
{
	char data[2];
	int i;
	if(palkern_status & PALKERN_IS_OPEN){
		data[0] = GET_ADDRESS(PAGE_SWITCH);
		data[1] = PAGE0;
		isc_regtrans(data, 2, 0);
		data[0] = GET_ADDRESS(CAM_POWER);
		data[1] = 0x0;
		isc_regtrans(data, 2, 0);
	}
	return 0;
}

static struct platform_driver omap_palkern_driver = {
	.probe = omap_palkern_probe,
//	.remove	= omap_palkern_remove,
	.suspend = omap_palkern_suspend,
	.resume = omap_palkern_resume,
	.driver = {
		.name = "omap_palkern",
	},
};

static int __init palkern_init(void)
{
	DPRINTK_PALKERN("PALKERN::init\n");
	co_opp_camera_latency = constraint_get("omap34xxcam", &cnstr_id_latency);
	return platform_driver_register(&omap_palkern_driver);
}

static void __exit palkern_exit(void)
{
	DPRINTK_PALKERN("PALKERN::exit\n");
	pseudoi2c_exit();
	platform_driver_unregister(&omap_palkern_driver);
	constraint_put(co_opp_camera_latency);
}

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

module_init(palkern_init);
module_exit(palkern_exit);


