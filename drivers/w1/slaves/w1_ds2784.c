/*
 *	w1_ds2784.c - w1 family 23 (DS2784) driver
 *
 * Copyright (c) 2008 Palm Inc.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2. See the file COPYING for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/battery_simple.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_family.h"

#define W1_GASGAUGE_FAMILY		0x32
#define W1_GASGAUGE_SIZE 		(1024*4)

#define DS2784_REG_STATUS		0x01
#define DS2784_REG_RARC 		0x06
#define DS2784_REG_RSRC 		0x07
#define DS2784_REG_TEMPERATURE_MSB	0x0A
#define DS2784_REG_TEMPERATURE_LSB	0x0B
#define DS2784_REG_VOLTAGE_MSB		0x0C
#define DS2784_REG_VOLTAGE_LSB		0x0D
#define DS2784_REG_CURRENT_MSB		0x0E
#define DS2784_REG_CURRENT_LSB		0x0F
#define DS2784_REG_COULOMB_MSB      0x10
#define DS2784_REG_COULOMB_LSB      0x11
#define DS2784_REG_FULL40_MSB 		0x6A

#define DS2784_REG_AVG_CURRENT_MSB  0x08
#define DS2784_REG_AVG_CURRENT_LSB  0x09

#define DS2784_REG_AS				0x14
#define DS2784_REG_RAAC_MSB			0x02
#define DS2784_REG_RAAC_LSB			0x03
#define DS2784_REG_FULL_MSB			0x16
#define DS2784_REG_FULL_LSB			0x17



#define DS2784_ROM_CODE_SIZE		0x08

#define DS2784_MATCH_NET_ADDRESS_CMD	0x55
#define DS2784_READ_DATA_CMD		0x69
#define DS2784_WRITE_DATA_CMD		0x6C
#define DS2784_RECALL_DATA_CMD		0xB8
#define DS2784_COPY_DATA_CMD		0x48

#define DS2784_EEPPROM_ADDR_BLOCK0  0x20
#define DS2784_EEPPROM_ADDR_BLOCK1  0x60
#define DS2784_EEPPROM_ADDR_BLOCK2  0xB0
#define DS2784_REG_RSENSE			0x69

#define DS2784_WRITE_CHALLENGE_CMD	0x0C
#define DS2784_COMPUTE_MAC_NO_ROMID	0x36
#define DS2784_COMPUTE_MAC_WITH_ROMID	0x35

#define DS2784_CHALLENGE_SIZE		8
#define DS2784_MAC_SIZE		20

// define R sense as 20 mohms, as opposed as a variable and reading it from the pack,
// not all packs have the correct R sense programmed
#define DEFAULT_RSENSE 20
#define DEFAULT_FULL40 0x0ECA

#define SIGN_EXTEND16(x)		(((s32)(x))-(((x)&0x8000)?65536:0))
#define CURRENT_VALUE(x,rsense)	((SIGN_EXTEND16(x)*3125)/2/rsense) // in uA 
#define VOLTAGE_VALUE(x)		(4880*((x)>>5)) // in micro volt
#define COULOMB_VALUE(x,rsense)	((6250*SIGN_EXTEND16(x))/((s32) rsense))
#define REG_COULOMB_VALUE(x,rsense)	((rsense*SIGN_EXTEND16(x))/6250)

#define CAPACITY_VALUE(x)		(1600*SIGN_EXTEND16(x))      // in micro Ahr
#define CAPACITY_VALUE_MA(x)		((1000*SIGN_EXTEND16(x))/625)      // in m Ahr
#define CAPACITY_PERCENT(x)		(392*x)		// in thousands of %

// #define DS2784_DEBUG 1

#define MOD_NAME "DS2784: "

#ifdef DS2784_DEBUG
#define DPRINTK(format,...)\
	printk(KERN_ERR MOD_NAME format, ## __VA_ARGS__)
#else
#define DPRINTK(format,...)
#endif

#define W1_BUS_ERROR			-1
#define W1_DEVICE_PRESENT		0
#define W1_NO_DEVICE_PRESENT		1

/*
 * mac (Message Authentication Code) = 160 bits
 */
static u8 mac[DS2784_MAC_SIZE] = {0};

struct ds2784_cmd
{
	u8 cmd;
	bool cmd_continue;
	u8 reg;
	u8 data;
};


struct w1_ds2764_driver_data{
	u8  rsense;
	u16 full40;
} ;


static u8 w1_ds2784_reg_read(struct w1_master *master, u8 addr)
{
	u8 data;

	/* send the read command first */
	w1_write_8(master, DS2784_READ_DATA_CMD);

	/* send the address of the register to read */
	w1_write_8(master, addr);

	w1_read_block(master, &data, sizeof(u8));

	return data;
}

static u8 w1_ds2784_reg_read_continue(struct w1_master *master)
{
	u8 data;

	w1_read_block(master, &data, sizeof(u8));

	return data;
}

static bool w1_ds2784_reg8_read(struct w1_slave *w1_dev, u8 addr, u8 *regP)
{
	*regP = 0;
	if (( w1_reset_select_slave(w1_dev)) != W1_DEVICE_PRESENT) {
		printk(KERN_ERR MOD_NAME "Error resetting & selecting slave\n");
		return true;
	}
	*regP = w1_ds2784_reg_read( w1_dev->master, addr);
	return false;
}


static bool  w1_ds2784_reg16_read(struct w1_slave *w1_dev, u8 addr, u16 *valP)
{
	u8 reg1,reg2;
	*valP = 0;
	if (( w1_reset_select_slave(w1_dev)) != W1_DEVICE_PRESENT) {
		printk(KERN_ERR MOD_NAME "Error resetting & selecting slave\n");
		return true;
	}
	reg1 = w1_ds2784_reg_read( w1_dev->master, addr);
	reg2 = w1_ds2784_reg_read_continue( w1_dev->master);

	*valP =   (((( u16 ) reg1) << 8) | reg2);

	return false;
}


static void w1_ds2784_reg_write(struct w1_master *master, u8 addr, u8 data)
{
	/* send the write command first */
	w1_write_8(master, DS2784_WRITE_DATA_CMD);

	/* send the address of the register to read */
	w1_write_8(master, addr);

	/* write the desired data */
	w1_write_8(master, data);
}
 

static int w1_ds2784_get_measurement(struct device *dev, 
					struct ds2784_cmd *cmdlist, 
					int count)
{
	int i;
	struct w1_slave	*w1_dev = dev_to_w1_slave(dev);
	int ret;
	u8 last_addr = 0;

	/* lock the master first */
	mutex_lock(&w1_dev->master->mutex);


	/* loop through to send all commands */
	for (i=0; i<count; i++) {
		if ( !cmdlist[i].cmd_continue || i==0) {
			if ((ret = w1_reset_select_slave(w1_dev)) != W1_DEVICE_PRESENT) {
				printk(KERN_ERR MOD_NAME "Error resetting & selecting slave\n");
				goto out;
			}
		}
		if (!cmdlist[i].cmd_continue) {
			if (cmdlist[i].cmd == DS2784_READ_DATA_CMD)
			{
				cmdlist[i].data = w1_ds2784_reg_read(
							w1_dev->master, 
							cmdlist[i].reg);
				last_addr = cmdlist[i].reg ;
			}
			else if (cmdlist->cmd == DS2784_WRITE_DATA_CMD)
			{
				w1_ds2784_reg_write(w1_dev->master, 
							cmdlist[i].reg, cmdlist[i].data);
				last_addr = cmdlist[i].reg ;
			}
			else
				printk(KERN_ERR MOD_NAME 
						"Unknown Cmd %x\n", cmdlist[i].cmd);
		}
		else
		{
			last_addr++;
			if ( i==0 ) {
				printk(KERN_ERR MOD_NAME 
						"Register continue cannot be on first command\n");
			}
			if ( last_addr != cmdlist[i].reg) {
				printk(KERN_ERR MOD_NAME 
						"Register continue (%d) on different register (%d)\n", 
					   last_addr, cmdlist[i].reg);
				continue;
			}

			if (cmdlist[i].cmd == DS2784_READ_DATA_CMD)
				cmdlist[i].data = w1_ds2784_reg_read_continue(w1_dev->master);
			else if (cmdlist->cmd == DS2784_WRITE_DATA_CMD)
				w1_write_8(w1_dev->master, cmdlist[i].data);
			else
				printk(KERN_ERR MOD_NAME 
						"Unknown Cmd %x\n", cmdlist[i].cmd);
		}
	}

out:
	mutex_unlock(&w1_dev->master->mutex);
	return ret;
}

/* 
 * use the challenge question to compute the MAC (Message Authentication Code)
 */
static int w1_ds2784_get_mac(struct device *dev, 
				u8 *challenge_question, 
				u8 challenge_cmd)
{
	struct w1_slave	*w1_dev = dev_to_w1_slave(dev);
	int ret;
#ifdef DS2784_DEBUG
	int count;
#endif

	/* lock the master first */
	mutex_lock(&w1_dev->master->mutex);

	if ((ret = w1_reset_select_slave(w1_dev)) != W1_DEVICE_PRESENT) {
		printk(KERN_ERR MOD_NAME "Error resetting & selecting slave\n");
		goto out;
	}

	/* send the write command first */
	w1_write_8(w1_dev->master, DS2784_WRITE_CHALLENGE_CMD);

	w1_write_block(w1_dev->master, 
			challenge_question, 
			DS2784_CHALLENGE_SIZE);

	if ((ret = w1_reset_select_slave(w1_dev)) != W1_DEVICE_PRESENT) {
		printk(KERN_ERR MOD_NAME "Error resetting & selecting slave\n");
		goto out;
	}

	/* 
	 * only valid commands are:
	 *    DS2784_COMPUTE_MAC_NO_ROMID
	 *    DS2784_COMPUTE_MAC_WITH_ROMID
	 */
	if (	(challenge_cmd == DS2784_COMPUTE_MAC_NO_ROMID) ||
		(challenge_cmd == DS2784_COMPUTE_MAC_WITH_ROMID) ) {
		w1_write_8(w1_dev->master, challenge_cmd);
	} else {
		printk(KERN_ERR MOD_NAME "Error invalid cmd %d\n", challenge_cmd);
		goto out;
	}

	msleep(15);

	w1_write_8(w1_dev->master, 0);

	w1_read_block(w1_dev->master, mac, DS2784_MAC_SIZE);

#ifdef DS2784_DEBUG
	for (count = 0; count < DS2784_MAC_SIZE; count++)
		DPRINTK("Response %d = %x\n", count, mac[count]);
#endif

out:
	mutex_unlock(&w1_dev->master->mutex);
	return ret;
}

static int ds2784_getvoltage_dev(struct device *dev, int *ret_voltage_uV)
{
	u16 voltage;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_VOLTAGE_MSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_VOLTAGE_LSB,
		},
	};

	if (!ret_voltage_uV)
		return -1;

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		return -1;
	}

	voltage = (cmdlist[0].data << 8) | cmdlist[1].data;

	*ret_voltage_uV = VOLTAGE_VALUE(voltage);
	return 0;
}

static ssize_t show_ds2784_voltage(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	int count;
	int retval;
	int voltage;

	retval = ds2784_getvoltage_dev(dev, &voltage);
	if (retval < 0)	{
		count = snprintf(buf, PAGE_SIZE, "-1\n");
	}
	else {
		count = snprintf(buf, PAGE_SIZE, "%d\n", voltage);
	}

	DPRINTK("Current volt: %d uVolts\n", voltage);

	return count;
}
static DEVICE_ATTR(getvoltage, S_IRUGO, show_ds2784_voltage, NULL);



static ssize_t show_ds2784_rsense(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	struct w1_ds2764_driver_data *data = dev_get_drvdata (dev);
	int count;

	count = snprintf(buf, PAGE_SIZE, "%d\n", data->rsense);

	DPRINTK("Current rsense: %d mohms\n", data->rsense);

	return count;
}
static DEVICE_ATTR(getrsense, S_IRUGO, show_ds2784_rsense, NULL);


static ssize_t show_ds2784_full40(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	struct w1_ds2764_driver_data *data = dev_get_drvdata (dev);
	int count;
	u32 full40;

	full40 = ((((u32)data->full40 ) *  25000L) >> 2) / data->rsense  ;

	count = snprintf(buf, PAGE_SIZE, "%d.%03d\n", full40/1000, full40%1000);

	DPRINTK("Current full40: %d uAh\n", data->full40);

	return count;
}
static DEVICE_ATTR(getfull40, S_IRUGO, show_ds2784_full40, NULL);


static int ds2784_getcurrent_dev(struct device *dev, int *ret_current)
{
	struct w1_ds2764_driver_data *data = dev_get_drvdata (dev);
	u16 cur;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_CURRENT_MSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_CURRENT_LSB,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		return -1;
	}

	cur = (cmdlist[0].data << 8) | cmdlist[1].data;

	*ret_current = CURRENT_VALUE(cur,data->rsense);
	return 0;
}

static ssize_t show_ds2784_current(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	int count = 0;
	int current_mA;
	int retval;
	
	retval = ds2784_getcurrent_dev(dev, &current_mA);
	if (retval < 0)
	{
		count = snprintf(buf, PAGE_SIZE, "FF\n");
	}
	else
	{
		count = snprintf(buf, PAGE_SIZE, "%d\n", current_mA);
	}

	DPRINTK("Current cur: %d uA\n", current_mA);

	return count;
}
static DEVICE_ATTR(getcurrent, S_IRUGO, show_ds2784_current, NULL);



static ssize_t show_ds2784_avg_current(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	struct w1_ds2764_driver_data *data = dev_get_drvdata (dev);
	int count = 0;
	u16 cur;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_AVG_CURRENT_MSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_AVG_CURRENT_LSB,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		count = snprintf(buf, PAGE_SIZE, "FF\n");
		return count;
	}

	cur = (cmdlist[0].data << 8) | cmdlist[1].data;

	count = snprintf(buf, PAGE_SIZE, "%d\n", CURRENT_VALUE(cur,data->rsense));

	DPRINTK("Avg Current cur: %d uA\n", CURRENT_VALUE(cur,data->rsense));

	return count;
}
static DEVICE_ATTR(getavgcurrent, S_IRUGO, show_ds2784_avg_current, NULL);



static ssize_t show_ds2784_reg(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	struct w1_slave	*w1_dev = dev_to_w1_slave(dev);
	int addr;
	int count = 0;
	u8  reg_val;
	int ret;
	int printbytes = 0;

	mutex_lock(&w1_dev->master->mutex);

	for (addr = 0;addr <= 0xb1;addr++,printbytes++) {
		if (addr == 0 || addr == 0x30 || addr == 0x80 ) {
			if (addr == 0x30) {
				addr = 0x60;
			}
			if ( addr == 0x80 ) {
				addr = 0xb0;
			}
			if ((ret = w1_reset_select_slave(w1_dev)) != W1_DEVICE_PRESENT) {
				printk(KERN_ERR MOD_NAME "Error resetting & selecting slave\n");
				goto out;
			}
			if (PAGE_SIZE-count >2) {
				count+= snprintf(&buf[count], PAGE_SIZE-count, "\n%02x:", addr);
			}
			reg_val = w1_ds2784_reg_read( w1_dev->master, addr);
			printbytes = 0;
		}
		else {
			reg_val = w1_ds2784_reg_read_continue(w1_dev->master);
		}

		if ( printbytes >=16) {
			if (PAGE_SIZE-count >2) {
				count+= snprintf(&buf[count], PAGE_SIZE-count, "\n%02x:", addr);
			}
			printbytes = 0;
		}

		if (PAGE_SIZE-count >2) {
			count+= snprintf(&buf[count], PAGE_SIZE-count, " %02x",reg_val);
		}
		else
		{
			break;
		}
	}
	if (PAGE_SIZE-count >2) {
		count+= snprintf(&buf[count], PAGE_SIZE-count, "\n");
	}
out:
	mutex_unlock(&w1_dev->master->mutex);

	return count;
}
static DEVICE_ATTR(dumpreg, S_IRUGO, show_ds2784_reg, NULL);


static ssize_t show_ds2784_status(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	int count,i;
	u8 status;

	static const  char *ds2784_status_bits[] =
	{
		NULL,
		"power-on-reset",
		"undervoltage",
		NULL,
		"learn",
		"standby-empty",
		"active-empty",
		"charge-termination"
	};

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.reg = DS2784_REG_STATUS,
			.cmd_continue = false,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		count = snprintf(buf, PAGE_SIZE, "-1\n");
		return count;
	}

	status = cmdlist[0].data;

	DPRINTK("Battery status reg = %x\n",status);

	for (i=count=0;i<ARRAY_SIZE(ds2784_status_bits);i++) {
		if ((status & (1<<i)) && ds2784_status_bits[i]!=NULL) {
			count += snprintf(&buf[count], PAGE_SIZE-count, "%s\n",ds2784_status_bits[i]);
		}
	}

	return count;
}

static ssize_t set_ds2784_status(struct device *dev,
					struct device_attribute *dev_attr,
					const char *buf,
					size_t count)
{

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_WRITE_DATA_CMD,
			.reg = DS2784_REG_STATUS,
			.cmd_continue = false,
			.data = 0,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		return count;
	}

	return count;
}
static DEVICE_ATTR(status, S_IRUGO|S_IWUSR, show_ds2784_status, set_ds2784_status);



static ssize_t show_ds2784_age(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	int count;
	u32 age;
	u32 percent;
	u32 res;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.reg = DS2784_REG_AS,
			.cmd_continue = false,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		count = snprintf(buf, PAGE_SIZE, "-1\n");
		return count;
	}

	age = cmdlist[0].data * 78125;
	percent = age/100000;
	res = age - percent * 100000;

	count = snprintf(buf, PAGE_SIZE, "%d.%05d\n", percent,res);

	DPRINTK("Current batt life left: %d.%05d percent\n", percent,res);

	return count;
}
static DEVICE_ATTR(getage, S_IRUGO, show_ds2784_age, NULL);


static ssize_t show_ds2784_coulomb(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	int count = 0;
	u16 cur;
	s32 cap;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_RAAC_MSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_RAAC_LSB,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		count = snprintf(buf, PAGE_SIZE, "FF\n");
		return count;
	}

	// Ds2784 returns in units of 1.6mAh
	cur = (cmdlist[0].data << 8) | cmdlist[1].data;
	cap = CAPACITY_VALUE(cur);   

	count = snprintf(buf, PAGE_SIZE, "%d.%03d\n", cap/1000, (u32)(cap%1000));

	DPRINTK("Coulomb : %d uAh\n", cap);

	return count;
}
static DEVICE_ATTR(getcoulomb, S_IRUGO, show_ds2784_coulomb, NULL);



static ssize_t show_ds2784_capacity(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	struct w1_ds2764_driver_data *data = dev_get_drvdata (dev);
	int count = 0;
	u32 cur;
	u32 cap;
	u32 age;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_FULL_MSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_FULL_LSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.reg = DS2784_REG_AS,
			.cmd_continue = false,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		count = snprintf(buf, PAGE_SIZE, "FF\n");
		return count;
	}

	cur = (cmdlist[0].data << 8) | cmdlist[1].data;
	cap = (((( cur * data->full40 * 25) >> 10) *1000)>>7 ) / data->rsense   ;
	DPRINTK("raw capacity %d.%03d\n", cap/1000, cap%1000);

	age = cmdlist[2].data * 25;  
	DPRINTK("age  %d.%03d\n", (age >> 5) , (((age & 0x1f) * 1000)>>5));
 
	cap = (( cap * age ) >> 5) /100;

	count = snprintf(buf, PAGE_SIZE, "%d.%03d\n", cap/1000, cap%1000);

	DPRINTK("capacity : %d uAh\n", cap);

	return count;
}
static DEVICE_ATTR(getcapacity, S_IRUGO, show_ds2784_capacity, NULL);





static ssize_t show_ds2784_raw_coulomb(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	struct w1_ds2764_driver_data *data = dev_get_drvdata (dev);
	int count = 0;
	u16 cur;
	s32 cou;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_COULOMB_MSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_COULOMB_LSB,
		},
	};

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		count = snprintf(buf, PAGE_SIZE, "FF\n");
		return count;
	}

	cur = (cmdlist[0].data << 8) | cmdlist[1].data;
	cou= COULOMB_VALUE(cur,data->rsense);

	count = snprintf(buf, PAGE_SIZE, "%d.%03d\n", cou/1000, ((cou>=0)? cou:-cou)%1000); 

	DPRINTK("Raw Coulomb : %d uAh\n", COULOMB_VALUE(cur,data->rsense));

	return count;
}
static DEVICE_ATTR(getrawcoulomb, S_IRUGO, show_ds2784_raw_coulomb, NULL);



static ssize_t set_ds2784_register(struct device *dev,
					struct device_attribute *dev_attr,
					const char *buf,
					size_t count)
{
	char *endp;
	u32 regchange = simple_strtoul(buf, &endp, 0);
	size_t size = endp - buf;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_WRITE_DATA_CMD,
			.cmd_continue = false,
		},
	};

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	cmdlist[0].reg  = (regchange >> 8 )&0xff;
	cmdlist[0].data = (regchange      )&0xff;

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		return count;
	}

	printk(KERN_WARNING
		"DS2784: register changed by user %02x: 0x%02x\n", cmdlist[0].reg, cmdlist[0].data);

	return count;
}
DEVICE_ATTR(setreg, S_IWUSR, NULL, set_ds2784_register);




#ifdef DS2784_DEBUG

static ssize_t set_ds2784_raw_coulomb(struct device *dev,
					struct device_attribute *dev_attr,
					const char *buf,
					size_t count)
{
	u8 rsense;
	char *endp;
	u32 new_coulomb = simple_strtoul(buf, &endp, 0);
	u32 cur;
	size_t size = endp - buf;
	static struct ds2784_cmd cmdlist_read[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_RSENSE,
		},
	};
	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_WRITE_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_COULOMB_MSB,
		},
		{
			.cmd = DS2784_WRITE_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_COULOMB_LSB,
		},
	};

	DPRINTK("%s\n", __FUNCTION__);

	if (*endp && isspace(*endp))
		size++;
	if (size != count)
		return -EINVAL;

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist_read, ARRAY_SIZE(cmdlist)) ) {
		return count;
	}
	rsense = cmdlist[0].data;
	if (rsense == 0) {
		rsense = DEFAULT_RSENSE;
		printk(KERN_WARNING
			"battery is programmed with rsense = 0 - using default of %d\n", 
			rsense);
	}
    else 
	{
  		rsense = 1000 / rsense;
	}

	cmdlist[0].data = ((REG_COULOMB_VALUE(new_coulomb,rsense))>>8) & 0xff;
	cmdlist[1].data = ((REG_COULOMB_VALUE(new_coulomb,rsense))   ) & 0xff;

	cur = (cmdlist[0].data << 8) | cmdlist[1].data;
	printk(KERN_WARNING
		"DS2784: coulomb counter set to %d.\n", cur);

	DPRINTK("Write coulomb : %d uAh\n", COULOMB_VALUE(cur,rsense));

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		return count;
	}

	return count;
}
DEVICE_ATTR(setrawcoulomb, S_IWUSR, NULL, set_ds2784_raw_coulomb);
#endif


static int ds2784_getpercent_dev(struct device *dev, int *ret_percent)
{
	u16 percentage;

	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.reg = DS2784_REG_RARC,
			.cmd_continue = false,
		},
	};

	if (!ret_percent) return -1;

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		return -1;
	}

	percentage = cmdlist[0].data;

	*ret_percent = percentage;

	return 0;
}

static ssize_t show_ds2784_percentage(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	int count;
	int percentage;
	int retval;

	retval = ds2784_getpercent_dev(dev, &percentage);
	if (retval < 0)
	{
		count = snprintf(buf, PAGE_SIZE, "-1\n");
	}
	else
	{
		count = snprintf(buf, PAGE_SIZE, "%d\n", percentage);
	}

	DPRINTK("Current batt life left: %d percent\n", percentage);

	return count;
}
static DEVICE_ATTR(getpercent, S_IRUGO, show_ds2784_percentage, NULL);

static int ds2784_gettemperature_dev(struct device *dev, int *ret_temperature)
{
	static struct ds2784_cmd cmdlist[] = {
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = false,
			.reg = DS2784_REG_TEMPERATURE_MSB,
		},
		{
			.cmd = DS2784_READ_DATA_CMD,
			.cmd_continue = true,
			.reg = DS2784_REG_TEMPERATURE_LSB,
		},
	};

	if (!ret_temperature) return -1;

	if ( W1_DEVICE_PRESENT !=
		w1_ds2784_get_measurement(dev, cmdlist, ARRAY_SIZE(cmdlist)) ) {
		return -1;
	}

	/* Temperature value is a 11-bit two's-complement integer with a 1/8
	 * degrees Centigrade resolution:
	 *   MSB: 7 6 5 4 3 2 1 0  LSB: 7 6 5 4 3 2 1 0
	 *        s i i i i i i i       f f f _ _ _ _ _
	 *
	 *  s: sign
	 *  i: integer part
	 *  f: fraction part
	 *  _: N/A
	 *
	 * We discard the fraction and only return the integer value.
	 */

	*ret_temperature = (s8) cmdlist[0].data;
	return 0;
}

static ssize_t show_ds2784_temperature(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	int count = 0;
	int temperature;
	int retval;

	retval = ds2784_gettemperature_dev(dev, &temperature);
	if (retval < 0)
	{
		count = snprintf(buf, PAGE_SIZE, "FF\n");
	}
	else
	{
		count = snprintf(buf, PAGE_SIZE, "%d\n", temperature);
	}

	DPRINTK("Current temp: %d degrees\n", temperature);

	return count;
}
static DEVICE_ATTR(gettemp, S_IRUGO, show_ds2784_temperature, NULL);

/*
 * NOTE:  this is a sample of how to read and validate the battery
 */
static ssize_t show_ds2784_valid_battery(struct device *dev, 
					struct device_attribute *dev_attr, 
					char *buf)
{
	static u8 challenge_buf[] = { 0x74, 0xCA, 0x85, 0x99, 0x19, 0xDE, 0xD1, 0xB3 };
	static u8 response_buf[]  = { 0x87, 0xED, 0x20, 0x89, 0xAD, 0x68, 0xA2, 0xCD,
				      0x6F, 0x93, 0x13, 0x03, 0x07, 0x5A, 0x29, 0x85,
				      0xDC, 0x2E, 0xE9, 0x50 };

	w1_ds2784_get_mac(dev, challenge_buf, DS2784_COMPUTE_MAC_NO_ROMID);

	/* validate the response */
	if (0 == memcmp(response_buf, mac, ARRAY_SIZE(response_buf))) {
		/* response validated, return OK */
		strcpy(buf, "OK\n");
		DPRINTK("Battery Validated OK\n");
	} else {
		/* Non-Palm battery */
		strcpy(buf, "ERROR\n");
		DPRINTK("Error validating battery\n");
	}

	return strlen(buf);
}
static DEVICE_ATTR(validate_battery, S_IRUGO, show_ds2784_valid_battery, NULL);

/*
 * compute the mac (message authentication code) using a challenge question 
 */
static ssize_t store_ds2784_mac(struct device *dev,
				struct device_attribute *dev_attr,
				const char *buf,
				size_t count)
{
	/* parse the buf */
	char tmp[3];	
	int i;
	u8 challenge_buf[DS2784_CHALLENGE_SIZE];

	/* we are expecting a hex number with
	 *   DS2784_CHALLENGE_SIZE * 2 digits.
	 */
	if (2 * DS2784_CHALLENGE_SIZE != strlen(buf)) {
		printk(KERN_WARNING
			"DS2784: Illegal hex number '%s'.\n", buf);
		return 0;
	}
	
	/* convert challenge string into challenge_buf */	
	tmp[2] = '\0';
	for (i=0; i<DS2784_CHALLENGE_SIZE; i++) {
		tmp[0] = *buf++;
		tmp[1] = *buf++;
		
		DPRINTK("str to be converted: %s\n", tmp);	
		
		challenge_buf[i] = (u8) simple_strtol(tmp, NULL, 16);
	}
	
	DPRINTK("input = %x %x %x %x %x %x %x %x\n", 
		challenge_buf[0], 
		challenge_buf[1], 
		challenge_buf[2], 
		challenge_buf[3], 
		challenge_buf[4], 
		challenge_buf[5], 
		challenge_buf[6], 
		challenge_buf[7] );
						
	w1_ds2784_get_mac(dev, challenge_buf, DS2784_COMPUTE_MAC_NO_ROMID);
	
	return 2 * DS2784_CHALLENGE_SIZE;

}
static ssize_t show_ds2784_mac(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	int count = 0;
	char tmpStr[3];
	int i;
	
	/* show the calculated MAC */
	for (i=0; i<DS2784_MAC_SIZE; i++) {
		count += sprintf(tmpStr,  "%02x", mac[i]);
		strcat(buf, tmpStr);
	}
	
	strcat(buf, "\n");
	count++;
	
	return count;
}
static DEVICE_ATTR(mac, S_IRUGO|S_IWUSR, show_ds2784_mac, store_ds2784_mac);

#ifdef CONFIG_BATTERY_SIMPLE
/** 
* @brief This will only work for devices with single batteries.
*/
static struct device *battery_device = NULL;

static int ds2784_getpercent(int *ret_percent)
{
	if (!battery_device) return -1;
	return ds2784_getpercent_dev(battery_device, ret_percent);
}

static int ds2784_getvoltage(int *ret_voltage)
{
	if (!battery_device) return -1;
	return ds2784_getvoltage_dev(battery_device, ret_voltage);
}

static int ds2784_gettemperature(int *ret_temperature)
{
	if (!battery_device) return -1;
	return ds2784_gettemperature_dev(battery_device, ret_temperature);
}

static int ds2784_getcurrent(int *ret_current)
{
	if (!battery_device) return -1;
	return ds2784_getcurrent_dev(battery_device, ret_current);
}

static struct battery_ops ds2784_battery_ops = {
	.get_percent       = ds2784_getpercent,
	.get_voltage       = ds2784_getvoltage,
	.get_temperature   = ds2784_gettemperature,
	.get_current       = ds2784_getcurrent,
};

#endif



static int w1_ds2784_add_slave(struct w1_slave *sl)
{
	int err = 0;
	u8  rsense;
	u16 full40;
	struct w1_ds2764_driver_data *data;

	DPRINTK("Adding slave...\n");

	data = kzalloc(sizeof(struct w1_ds2764_driver_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	dev_set_drvdata(&sl->dev, data);

	// read the static registers
	if (w1_ds2784_reg8_read(sl, DS2784_REG_RSENSE, &rsense) || rsense == 0)
	{
		data->rsense = DEFAULT_RSENSE;
		printk(KERN_WARNING
			"unable to read battery rsense or rsense = 0 - using default of %d\n", 
			data->rsense);
	}
	else
	{
		data->rsense = (rsense != 0 ) ? (1000/rsense):DEFAULT_RSENSE;
	}
	printk("------------------Rsense = %d\n",data->rsense);
	if (w1_ds2784_reg16_read(sl, DS2784_REG_FULL40_MSB, &full40) || full40 == 0)
	{
		data->full40 = DEFAULT_FULL40;
		printk(KERN_WARNING
			"unable to read battery full40 or full40 = 0 - using default of %d\n", 
			data->full40);
	}
	else
	{
		data->full40 = (full40 != 0)? full40:DEFAULT_FULL40;
	}
	printk("------------------Full40 = %d\n",data->full40);


	if (device_create_file(&sl->dev, &dev_attr_getvoltage) < 0)
		printk(KERN_ERR "Creating sysfs entry for voltage failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getrsense) < 0)
		printk(KERN_ERR "Creating sysfs entry for rsense failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getfull40) < 0)
		printk(KERN_ERR "Creating sysfs entry for full40 failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getcurrent) < 0)
		printk(KERN_ERR "Creating sysfs entry for current failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getavgcurrent) < 0)
		printk(KERN_ERR "Creating sysfs entry for voltage failed\n");
	if (device_create_file(&sl->dev, &dev_attr_dumpreg) < 0)
		printk(KERN_ERR "Creating sysfs entry for dumpreg failed\n");
	if (device_create_file(&sl->dev, &dev_attr_status) < 0)
		printk(KERN_ERR "Creating sysfs entry for status failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getage) < 0)
		printk(KERN_ERR "Creating sysfs entry for age failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getcoulomb) < 0)
		printk(KERN_ERR "Creating sysfs entry for coulomb failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getcapacity) < 0)
		printk(KERN_ERR "Creating sysfs entry for capacity failed\n");
	if (device_create_file(&sl->dev, &dev_attr_getrawcoulomb) < 0)
		printk(KERN_ERR "Creating sysfs entry for rawcoulomb failed\n");
	if (device_create_file(&sl->dev, &dev_attr_setreg) < 0)
		printk(KERN_ERR "Creating sysfs entry for setreg failed\n");
#ifdef DS2784_DEBUG

	if (device_create_file(&sl->dev, &dev_attr_setrawcoulomb) < 0)
		printk(KERN_ERR "Creating sysfs entry for setrawcoulomb failed\n");
#endif
	if (device_create_file(&sl->dev, &dev_attr_getpercent) < 0)
		printk(KERN_ERR "Creating sysfs entry for percentage failed\n");
	if (device_create_file(&sl->dev, &dev_attr_gettemp) < 0)
		printk(KERN_ERR "Creating sysfs entry for temperature failed\n");
	if (device_create_file(&sl->dev, &dev_attr_validate_battery) < 0)
		printk(KERN_ERR "Creating sysfs validate_battery failed\n");
	if (device_create_file(&sl->dev, &dev_attr_mac) < 0)
		printk(KERN_ERR "Creating sysfs mac failed\n");

	battery_device = &sl->dev;

	return err;
}

static void w1_ds2784_remove_slave(struct w1_slave *sl)
{
	DPRINTK("Removing slave...\n");

	device_remove_file(&sl->dev, &dev_attr_getvoltage);
	device_remove_file(&sl->dev, &dev_attr_getrsense);
	device_remove_file(&sl->dev, &dev_attr_getfull40);
	device_remove_file(&sl->dev, &dev_attr_getcurrent);
	device_remove_file(&sl->dev, &dev_attr_getavgcurrent);
	device_remove_file(&sl->dev, &dev_attr_dumpreg);
	device_remove_file(&sl->dev, &dev_attr_status);
	device_remove_file(&sl->dev, &dev_attr_getage);
	device_remove_file(&sl->dev, &dev_attr_getcoulomb);
	device_remove_file(&sl->dev, &dev_attr_getcapacity);
	device_remove_file(&sl->dev, &dev_attr_getrawcoulomb);
	device_remove_file(&sl->dev, &dev_attr_setreg);
#ifdef DS2784_DEBUG
	device_remove_file(&sl->dev, &dev_attr_setrawcoulomb);
#endif
	device_remove_file(&sl->dev, &dev_attr_getpercent);
	device_remove_file(&sl->dev, &dev_attr_gettemp);
	device_remove_file(&sl->dev, &dev_attr_validate_battery);
	device_remove_file(&sl->dev, &dev_attr_mac);

	battery_device = NULL;
}

static struct w1_family_ops w1_ds2784_fops = {
	.add_slave      = w1_ds2784_add_slave,
	.remove_slave   = w1_ds2784_remove_slave,
};

static struct w1_family w1_family_gasgauge = {
	.fid = W1_GASGAUGE_FAMILY,
	.fops = &w1_ds2784_fops,
};

static int __init w1_ds2784_init(void)
{
#ifdef CONFIG_BATTERY_SIMPLE
	battery_set_ops(&ds2784_battery_ops);
#endif
	return w1_register_family(&w1_family_gasgauge);
}

static void __exit w1_ds2784_exit(void)
{
	w1_unregister_family(&w1_family_gasgauge);
}

module_init(w1_ds2784_init);
module_exit(w1_ds2784_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("PALM Inc.");
MODULE_DESCRIPTION("w1 family driver for DS2784, 4kb gasgauge");

