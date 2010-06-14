/*
 * drivers/media/video/omap/ise/pseudoi2c.c
 *
 * I2C slave driver for TI's OMAP3430 Camera ISP
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

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/delay.h>


#ifdef P_I2C_DEBUG
#define FN_IN printk("%s Entry\n",__FUNCTION__);
#define FN_OUT(ARG) printk("%s[%d]:Exit(%d)\n",__FUNCTION__,__LINE__,ARG);
#else
#define FN_IN
#define FN_OUT(ARG)
#endif

#ifdef P_I2C_DEBUG
#define DPRINTK_P_I2C(format,...)\
	printk("PI2C: " format, ## __VA_ARGS__)
#else
#define DPRINTK_P_I2C(format,...)
#endif

#define SENSOR_NAME	"CAMSENSOR"
#define MAX_CAM_SENSOR 100;
#define SENSOR_USED 1
#define SENSOR_UNUSED 0
#define PI2C_8BIT 1
#define PI2C_16BIT 2
#define PI2C_32BIT 4

static u8 i2c_adapter_num;
static int driver_registered = 0;

static int pseudoi2c_attach_adapter(struct i2c_adapter *adapter);
//static int pseudo_i2c_detach_client(struct i2c_client *iclient);

/* One Client Driver , multiple camera sensors */
static struct i2c_driver pseudoi2c_driver = {
	.driver.name = "PSEUDO I2C",
	.attach_adapter = pseudoi2c_attach_adapter,
//	.detach_client = pseudo_i2c_detach_client,
};

/* Internal structure for the driver
 * pseudo_i2c_adapter - the i2c adapter to which the client using this
 * 			driver is going to talk to
 * cam_sensors - list maintaining all the sensors using this driver.
 * reg_length - address length of the sensor registers
 * data_length - data width of teh sensor registers
 */
struct pseudoi2c {
	struct i2c_adapter *pseudo_i2c_adapter;
	struct list_head cam_sensors;
	int reg_length;
	int data_length;
} p_i2c;

/* Structure to define camera sensors Slave ID 
 * i2c_client - client structre for i2c controller
 * client_name - name of the i2c client
 * address - slave id of the i2c client
 * inuse - current sensor is in use ie attached to the i2c framework or not
 * list - list head to be added to the gloabal list pseudoi2c::cam_sensors
 * xfer_msg - i2c messages to be passed to i2c framework
 * xfer_lock - lock to make sure that next access to i2c is made only after
 * 		current access.
 * */
struct pseudoi2c_client {
	struct i2c_client client;
	char client_name[sizeof(SENSOR_NAME) + 1];
 	u8 address;
	unsigned char inuse;
	struct list_head list;
	/* max numb of i2c_msg required is for read =2 */	
	struct i2c_msg xfer_msg[2];	
	struct semaphore xfer_lock;/* To lock access to xfer_msg */
};

/* adapter's callback */
/*static int pseudo_i2c_detach_client(struct i2c_client *iclient)
{
	int err;
	FN_IN;
	if ((err = i2c_detach_client(iclient))) {
		printk(KERN_ERR
		       "P-I2C: Client deregistration failed, client not detached.\n");
		FN_OUT(err);
		return err;
	}
	FN_OUT(0);
	return 0;
}*/

/* attach a client to the adapter 
 * slaveid - slave id of the client to be attached
 * ret - 0 if the client is successfully registered with i2c framework. error
 * 	otherwise
 */
static int p_i2c_detect_client (u8 slaveid)
{
	int err = 0;
	struct pseudoi2c_client *client = NULL;
	struct list_head *item;

	FN_IN;
	list_for_each(item,&(p_i2c.cam_sensors)) {
		client = list_entry(item,struct pseudoi2c_client,list);
		if(client->address == slaveid)
			break;
	}
	if (client->address != slaveid) {
		printk(KERN_ERR "Invalid slave id\n");
		return -EINVAL;
	}
	memset(&(client->client), 0, sizeof(struct i2c_client));
	client->client.addr = client->address;
	client->client.adapter = p_i2c.pseudo_i2c_adapter;
	client->client.driver = &pseudoi2c_driver;
	memcpy(&(client->client.name), client->client_name,
	       sizeof(SENSOR_NAME));

	if ((err = i2c_attach_client(&(client->client)))) {
		printk(KERN_WARNING "P-I2C: Couldn't attach Slave %s \
			on Adapter %s[%d][%x]\n",
			client->client_name,(p_i2c.pseudo_i2c_adapter)->name,
			err, err);
		FN_OUT(err);
		return err;
	}
	client->inuse = SENSOR_USED;
	init_MUTEX(&client->xfer_lock);
	FN_OUT(0);
	return 0;
}

/* detach a client sensor from the i2c adapter 
 * slaveid - slave id of the client to be dettached
 * ret - 0 if the client is successfully unregistered with i2c framework. error
 * 	otherwise
 */
int p_i2c_detach_slave(u8 slaveid)
{
	struct list_head *item;
	struct pseudoi2c_client *client = NULL;
	int err = 0;
	FN_IN;
	list_for_each(item,&(p_i2c.cam_sensors)) {
		client = list_entry(item,struct pseudoi2c_client,list);
		if(client->address == slaveid)
			break;
	}
	if (client->address != slaveid) {
		printk(KERN_ERR "Invalid slave id\n");
		return -EINVAL;
	}
	if ((err = i2c_detach_client(&(client->client)))) {
		printk(KERN_WARNING "P-I2C: Couldn't detach Slave %s \
			on Adapter %s[%d][%x]\n",
			client->client_name, (p_i2c.pseudo_i2c_adapter)->name,
			err, err);
	FN_OUT(err);
	return err;
	}
	client->inuse = SENSOR_UNUSED;
	list_del(& (client->list));
	kfree(client);
	FN_OUT(0);
	return 0;
}
EXPORT_SYMBOL(p_i2c_detach_slave);

/* attach a client to the adapter 
 * slaveid - slave id of the client to be attached
 * ret - 0 if the client is successfully registered with i2c framework. error
 * 	otherwise
 */
int p_i2c_attach_slave(u8 slaveid)
{
	struct pseudoi2c_client *i2c_client,*client = NULL;
	struct list_head *item;
	int ret;
	FN_IN;
	list_for_each(item,&(p_i2c.cam_sensors)) {
		client = list_entry(item,struct pseudoi2c_client,list);
		if(client->address == slaveid) {
			printk(KERN_ERR "Sensor: Client is already in Use.\n");
			printk("%s[ID=0x%x] NOT attached to I2c Adapter %s\n",
		       		client->client_name, client->address,
				(p_i2c.pseudo_i2c_adapter)->name);
			FN_OUT(EPERM);
			return -EPERM;
		}
	}
	
	i2c_client = kmalloc(sizeof(struct pseudoi2c_client),
				GFP_DMA | GFP_KERNEL);
	i2c_client->address = slaveid;
	memcpy(&(i2c_client->client_name), SENSOR_NAME,
	       sizeof(SENSOR_NAME));

	i2c_client->inuse = SENSOR_UNUSED;
	list_add_tail(& (i2c_client->list) ,&(p_i2c.cam_sensors));
	if ((ret = p_i2c_detect_client(slaveid))) {
		printk(KERN_ERR "Error from p_i2c_detect_client\n");
		list_del(& (i2c_client->list));
		kfree(i2c_client);
	}
	FN_OUT(ret);
	return ret;
}

EXPORT_SYMBOL(p_i2c_attach_slave);

/* adapter callback 
 * Called from i2c framework as part of i2c_register_driver() from
 * pseudoi2c_init
 */
static int pseudoi2c_attach_adapter(struct i2c_adapter *adapter)
{
	static int pseudo_i2c_adapter_id = 0;
	FN_IN;
		if (pseudo_i2c_adapter_id == i2c_adapter_num) {
			p_i2c.pseudo_i2c_adapter = adapter;
		}
	pseudo_i2c_adapter_id++;
FN_OUT(0);
return 0;
}

/* init function.Called from PalKern init
 */
int pseudoi2c_init(u8 i2c_num)
{
	int ret = 0;
	FN_IN;
	i2c_adapter_num = i2c_num;
	if (!driver_registered) {
		INIT_LIST_HEAD(&(p_i2c.cam_sensors));
		ret = i2c_register_driver(THIS_MODULE, &pseudoi2c_driver);
		if (ret) {
			printk(KERN_ERR
			"pseudo12c: Driver registration failed, module not \
			inserted.\n");
		}
		else
			driver_registered = 1;
	}
	FN_OUT(ret);
	return ret;
}
EXPORT_SYMBOL(pseudoi2c_init);

void pseudoi2c_exit()
{	
	FN_IN;
	i2c_adapter_num = 0;
	driver_registered = 0;
	i2c_del_driver(&pseudoi2c_driver);
	FN_OUT(0);
}
EXPORT_SYMBOL(pseudoi2c_exit);

/* Populates the i2c framework structures and issues a write to a register
 * command to the i2c framework
 * slave_id - slave id of the sensor to be written into
 * value - contains the register address and the valuse to be written to that
 * 	    register
 * num_bytes - number of bytes to be written by i2c frame work
 * ret - 0 if the transaction is a success else error value
 */ 	    
int p_i2c_write_slave(short slave_id, u8 *value,int num_bytes, u8 data_width)
{
	int i;
	int ret;
	struct pseudoi2c_client *client = NULL;
	struct i2c_msg *msg;
	unsigned char data[num_bytes];
	struct list_head *item;
	FN_IN;
	list_for_each(item,&(p_i2c.cam_sensors)) {
		client = list_entry(item,struct pseudoi2c_client,list);
		if(client->address == slave_id)
			break;
	}
	if (client->address != slave_id)
	{
		printk(KERN_ERR "PI2C: Invalid module Number\n");
		FN_OUT(EPERM);
		return -EPERM;
	}
	down(&(client->xfer_lock));
	if(data_width != 0)
		p_i2c.data_length = data_width;
	/* [MSG1]: fill the register address data 
	 * fill the data Tx buffer 
	 */
	msg = &(client->xfer_msg[0]);
	msg->addr = client->address;
	msg->flags = 0;//I2C_M_WR;	/* write the register value */
	msg->buf = data;
	msg->len = num_bytes;
	i = 0;

	if (p_i2c.reg_length == PI2C_8BIT) {
		data[i] = value[i];
		i+=1;
	}
	else if (p_i2c.reg_length == PI2C_16BIT) {
		data[i] = value[i + 1];
		data[i+1] = value[i];
		i+=2;
	/*	if (num_bytes == 3)
			p_i2c.data_length = PI2C_8BIT;
		else
			p_i2c.data_length = PI2C_16BIT;*/
	}
	else if (p_i2c.reg_length == PI2C_32BIT) {
		data[i] = value[i + 3];
		data[i + 1] = value[i + 2];
		data[i + 2] = value[i + 1];
		data[i + 3] = value[i];
		i+=4;
	}
	else {
		printk(KERN_ERR "PI2c - write :reglen Invalid argument\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	if (p_i2c.data_length == PI2C_8BIT) {
		data[i] = value[i];	
	}
	else if (p_i2c.data_length == PI2C_16BIT) {
		data[i] = value[i+1];
		data[i+1] = value[i];
	}
	else if (p_i2c.data_length == PI2C_32BIT) {
		data[i] = value[i + 3];
		data[i + 1] = value[i + 2];
		data[i + 2] = value[i + 1];
		data[i + 3] = value[i];
	}
	else {
		printk(KERN_ERR "PI2c - write :datalen Invalid argument\n");
		FN_OUT(-EINVAL);
		return -EINVAL;
	}
	DPRINTK_P_I2C(" 0x%x 0x%x 0x%x 0x%x, 0x%x 0x%x 0x%x\n",msg->addr,
			p_i2c.reg_length,
			p_i2c.data_length,
			data[0],data[1],data[2],data[3]);
	/* one message */
	ret = i2c_transfer(client->client.adapter, client->xfer_msg, 1);
	udelay(150);
	up(&(client->xfer_lock));
	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret>=0)
		ret=0;
	FN_OUT(ret);
	return ret;
}
EXPORT_SYMBOL(p_i2c_write_slave);

/* Populates the i2c framework structures and issues a read from a register
 * command to the i2c framework
 * slave_id - slave id of the sensor to be read from
 * value - contains the register address whch has to be read from.The read
 * 		value is written back to value
 * num_bytes - number of bytes to be written/read by i2c frame work
 * ret - 0 if the transaction is a success else error value
 */ 
int p_i2c_read_slave(short slave_id, u8 *value, u8 num_bytes, u8 data_width)
{
	int ret;
	int i = 0;
	struct i2c_msg *msg;
	struct pseudoi2c_client *client = NULL;
	unsigned char data[num_bytes - p_i2c.reg_length];
	struct list_head *item;
	FN_IN;
	list_for_each(item,&(p_i2c.cam_sensors)) {
		client = list_entry(item,struct pseudoi2c_client,list);
		if(client->address == slave_id)
			break;
	}
	if (client->address != slave_id)
	{
		printk(KERN_ERR "PI2C: Invalid module Number\n");
		FN_OUT(EPERM);
		return -EPERM;
	}
	down(&(client->xfer_lock));
	if(data_width != 0)
		p_i2c.data_length = data_width;
	/* [MSG1] fill the register address data */
	msg = &(client->xfer_msg[0]);
	msg->addr = client->address;
	msg->flags = 0;//I2C_M_WR;
	msg->len = p_i2c.reg_length;
	if (p_i2c.reg_length == PI2C_8BIT) {
		msg->buf[i] = value[i];
		i++;
	}
	else if (p_i2c.reg_length == PI2C_16BIT) {
		msg->buf[i] = value[i + 1];
		msg->buf[i + 1] = value[i];
		i+=2;
	/*	if (num_bytes == 3)
			p_i2c.data_length = PI2C_8BIT;
		else
			p_i2c.data_length = PI2C_16BIT;*/
	}
	else if (p_i2c.reg_length == PI2C_32BIT) {
		msg->buf[i] = value[i + 3];
		msg->buf[i + 1] = value[i + 2];
		msg->buf[i + 2] = value[i + 1];
		msg->buf[i + 3] = value[i];
		i+=4;
	}
	/* [MSG2] fill the data rx buffer */
	msg = &(client->xfer_msg[1]);
	msg->addr = client->address;
	msg->flags = I2C_M_RD;
	msg->len = num_bytes - p_i2c.reg_length;
	msg->buf = data;
	/* two messages */
	ret = i2c_transfer(client->client.adapter, client->xfer_msg, 2);
	up(&(client->xfer_lock));
	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret>=0) {
		if (p_i2c.data_length == PI2C_8BIT) {
			value[i] = data[0];
		}
		else if (p_i2c.data_length == PI2C_16BIT) {
			value[i] = data[1];
			value[i + 1] = data[0];
		}
		else if (p_i2c.data_length == PI2C_32BIT) {
			value[i] = data[3];
			value[i + 1] = data[2];
			value[i + 2] = data[1];
			value[i + 3] = data[0];
		}
		ret=0;
	}
	FN_OUT(ret);
	return ret;	
}
EXPORT_SYMBOL(p_i2c_read_slave);

/* Sets the address length of the sensor registers
 * reg_length - the address length of the sensor registers
 */
void p_i2c_set_reglength(unsigned int reg_length)
{
	DPRINTK_P_I2C("REG_len = %d bytes", reg_length);
	p_i2c.reg_length = reg_length;
}
EXPORT_SYMBOL(p_i2c_set_reglength);

/* Sets the data length of the sensor registers
 * data_length - the data length of the sensor registers
 */
void p_i2c_set_datalength(unsigned int data_length)
{
	p_i2c.data_length = data_length;
}
EXPORT_SYMBOL(p_i2c_set_datalength);

