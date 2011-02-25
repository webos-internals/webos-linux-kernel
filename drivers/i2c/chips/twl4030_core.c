/*
 * twl4030_core.c - driver for TWL4030 PM and audio CODEC device
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Modifications to defer interrupt handling to a kernel thread:
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Based on tlv320aic23.c:
 * Copyright (c) by Kai Svahn <kai.svahn@nokia.com>
 *
 * Code cleanup and modifications to IRQ handler.
 * by syed khasim <x0khasim@ti.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>

#include <asm/arch/twl4030.h>
#include <asm/arch/power_companion.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>

#undef  USE_GPIO_IRQ
#undef  USE_GPIO_IRQ_PIN_MUX

#ifdef CONFIG_ARCH_OMAP34XX
  #define USE_GPIO_IRQ          0   //  gpio 0
  #define USE_GPIO_IRQ_PIN_MUX  "AF26_GPIO0"
#endif

#ifdef CONFIG_ARCH_OMAP24XX
//#define USE_GPIO_IRQ          56  //  gpio 56
//#define USE_GPIO_IRQ_PIN_MUX  ?? 
#endif

#define DRIVER_NAME			"twl4030"

#undef  pr_err
#define pr_err(fmt, arg...)	printk(KERN_ERR DRIVER_NAME ": " fmt, ##arg);

/**** Macro Definitions */
#define TWL_CLIENT_STRING		"TWL4030-ID"
#define TWL_CLIENT_USED			1
#define TWL_CLIENT_FREE			0

/* IRQ Flags */
#define FREE				0
#define USED				1

/** Primary Interrupt Handler on TWL4030 Registers */

/**** Register Definitions */

#define REG_PIH_ISR_P1			(0x1)
#define REG_PIH_ISR_P2			(0x2)
#define REG_PIH_SIR			(0x3)

/* Triton Core internal information (BEGIN) */

/* Last - for index max*/
#define TWL4030_MODULE_LAST		TWL4030_MODULE_SECURED_REG

/* Slave address */
#define TWL4030_NUM_SLAVES		0x04
#define TWL4030_SLAVENUM_NUM0		0x00
#define TWL4030_SLAVENUM_NUM1		0x01
#define TWL4030_SLAVENUM_NUM2		0x02
#define TWL4030_SLAVENUM_NUM3		0x03
#define TWL4030_SLAVEID_ID0		0x48
#define TWL4030_SLAVEID_ID1		0x49
#define TWL4030_SLAVEID_ID2		0x4A
#define TWL4030_SLAVEID_ID3		0x4B

/* Base Address defns */
/* USB ID */
#define TWL4030_BASEADD_USB		0x0000
/* AUD ID */
#define TWL4030_BASEADD_AUDIO_VOICE	0x0000
#define TWL4030_BASEADD_GPIO		0x0098

#define TWL4030_BASEADD_INTBR		0x0085
#define TWL4030_BASEADD_PIH		0x0080
#define TWL4030_BASEADD_TEST		0x004C
/* AUX ID */
#define TWL4030_BASEADD_INTERRUPTS	0x00B9
#define TWL4030_BASEADD_LED		0x00EE
#define TWL4030_BASEADD_MADC		0x0000
#define TWL4030_BASEADD_MAIN_CHARGE	0x0074
#define TWL4030_BASEADD_PRECHARGE	0x00AA
#define TWL4030_BASEADD_PWM0		0x00F8
#define TWL4030_BASEADD_PWM1		0x00FB
#define TWL4030_BASEADD_PWMA		0x00EF
#define TWL4030_BASEADD_PWMB		0x00F1
#define TWL4030_BASEADD_KEYPAD		0x00D2
/* POWER ID */
#define TWL4030_BASEADD_BACKUP		0x0014
#define TWL4030_BASEADD_INT		0x002E
#define TWL4030_BASEADD_PM_MASTER	0x0036
#define TWL4030_BASEADD_PM_RECEIVER	0x005B
#define TWL4030_BASEADD_RTC		0x001C
#define TWL4030_BASEADD_SECURED_REG	0x0000

/* Triton Core internal information (END) */

#ifdef CONFIG_MACH_OMAP_2430SDP 
#define CONFIG_I2C_TWL4030_ID	2   /* on I2C2 for 2430SDP */
#elif defined(CONFIG_MACH_BRISKET) 
#define CONFIG_I2C_TWL4030_ID	2   /* on I2C1 for Brisket (aka Joplin) */
#elif	defined(CONFIG_MACH_OMAP_3430SDP) || \
	defined(CONFIG_MACH_OMAP_3430LABRADOR)
#define CONFIG_I2C_TWL4030_ID	1   /* on I2C1 for 3430SDP & 3430LABRADOR*/
#elif defined(CONFIG_MACH_FLANK) 
#define CONFIG_I2C_TWL4030_ID	1   /* on I2C0 for Flank (aka Joplin3430) */
#elif defined(CONFIG_MACH_SIRLOIN)
#define CONFIG_I2C_TWL4030_ID	1   /* on I2C0 for Sirloin */
#else
#error "Unsupported platform!!!"
#endif


#ifdef CONFIG_TWL4030_DBG_SYSFS

static struct platform_device *twl4030_debug_dev;

/* 255 Max bytes in a field register */
#define READ_REG_SIZE 255

static ssize_t show_mod(int mod, struct device *dev, char *buf)
{
	u8 temp_buffer[READ_REG_SIZE + 1];
	struct timeval stv, stv1, stv2;
	int timespent1, timespent2, j;
	char *sval = buf;

	/* Read from I2c first 255 bytes (the max we can write in the reg) */
	do_gettimeofday(&stv);
	if ((j = twl4030_i2c_read(mod, temp_buffer, 0x0, READ_REG_SIZE)) < 0) {
		printk(KERN_ERR
		       "unable to read %d bytes returned %d in module %d\n",
		       READ_REG_SIZE, j, mod);
		return j;
	}
	do_gettimeofday(&stv1);

	/* do a read of the last 256th byte */
	if ((j = twl4030_i2c_read_u8(mod, temp_buffer + READ_REG_SIZE,
				     READ_REG_SIZE)) < 0) {
		printk(KERN_ERR
		       "unable to read %d reg returned %d in module %d\n",
		       READ_REG_SIZE, j, mod);
		return j;
	}
	do_gettimeofday(&stv2);

	sval += sprintf(sval, "  | ");

	for (j = 0; j < 0x10; j++)
		sval += sprintf(sval, "%02X ", j);
	sval += sprintf(sval, "\n--+");

	for (j = 0; j < 0x10; j++)
		sval += sprintf(sval, " --");
	sval += sprintf(sval, "\n00| ");

	for (j = 0; j <= READ_REG_SIZE; j++) {
		sval += sprintf(sval, "%02X", temp_buffer[j]);
		if (j < READ_REG_SIZE) {
			sval += ((j + 1) % 0x10) ?
				sprintf(sval, " ") :
				sprintf(sval, "\n%02X| ", j + 1);
		}
	}
	timespent1 = (stv1.tv_sec - stv.tv_sec) * 1000000
			+ (stv1.tv_usec - stv.tv_usec);
	timespent2 = (stv2.tv_sec - stv1.tv_sec) * 1000000
			+ (stv2.tv_usec - stv1.tv_usec);
	sval += sprintf(sval, "\nTime Taken(uSec): 255bytes=%d 1byte=%d\n",
			timespent1, timespent2);
	sval += 1;
	*sval = 0;
	return sval - buf + 1;
}

/* MSB 8 bits are reg address[module reg offset] and LSB 8 bits the value */
static ssize_t set_mod(int mod, struct device *dev,
		       const char *buf, size_t count)
{
	u16 val = (u16) simple_strtoul(buf, NULL, 16);
	printk("Reg=0x%02x, val=0x%02x,mod=%d\n", (val & 0xFF00) >> 8,
	       (val & 0x00FF), mod);
	if (twl4030_i2c_write_u8(mod, (val & 0x00FF), (val & 0xFF00) >> 8) < 0)
		printk("write failed!\n");
	else
		printk("write success\n");
	return count;
}

/* function generator macros */
#define MAK_MOD(num, name)						\
static ssize_t show_mod_##name(struct device *dev,			\
			struct device_attribute *attr, char *buf)	\
{									\
	return show_mod(num, dev, buf);					\
}									\
									\
static ssize_t set_mod_##name(struct device *dev,			\
			struct device_attribute *attr, const char *buf,	\
			size_t count)					\
{									\
	return set_mod(num, dev, buf, count);				\
}									\
									\
static DEVICE_ATTR(module_##name, S_IWUSR | S_IRUGO,			\
		   show_mod_##name, set_mod_##name);

MAK_MOD(0, usb)
MAK_MOD(1, audio_voice)
MAK_MOD(2, gpio)
MAK_MOD(3, intbr)
MAK_MOD(4, pih)
MAK_MOD(5, test)
MAK_MOD(6, keypad)
MAK_MOD(7, madc)
MAK_MOD(8, interrupts)
MAK_MOD(9, led)
MAK_MOD(10, main_charge)
MAK_MOD(11, precharge)
MAK_MOD(12, pwm0)
MAK_MOD(13, pwm1)
MAK_MOD(14, pwma)
MAK_MOD(15, pwmb)
MAK_MOD(16, backup)
MAK_MOD(17, int)
MAK_MOD(18, pm_master)
MAK_MOD(19, pm_receiver)
MAK_MOD(20, rtc)
MAK_MOD(21, secured_reg)

static int twl4030_sysfs_debug_create(void)
{
	struct device *dev;
	int ret;

	if (twl4030_debug_dev)
		return 0;

	twl4030_debug_dev = kzalloc(sizeof *twl4030_debug_dev, GFP_KERNEL);
	twl4030_debug_dev->name = "twl4030_registers";

	if ((ret = platform_device_register(twl4030_debug_dev))) {
		kfree(twl4030_debug_dev);
		printk(KERN_ERR "Platform dev_register failed, err %d\n", ret);
		return ret;
	}

	dev = &twl4030_debug_dev->dev;
	ret = ! (!device_create_file(dev, &dev_attr_module_usb) &&
		 !device_create_file(dev, &dev_attr_module_audio_voice) &&
		 !device_create_file(dev, &dev_attr_module_gpio) &&
		 !device_create_file(dev, &dev_attr_module_intbr) &&
		 !device_create_file(dev, &dev_attr_module_pih) &&
		 !device_create_file(dev, &dev_attr_module_test) &&
		 !device_create_file(dev, &dev_attr_module_interrupts) &&
		 !device_create_file(dev, &dev_attr_module_led) &&
		 !device_create_file(dev, &dev_attr_module_madc) &&
		 !device_create_file(dev, &dev_attr_module_main_charge) &&
		 !device_create_file(dev, &dev_attr_module_precharge) &&
		 !device_create_file(dev, &dev_attr_module_pwm0) &&
		 !device_create_file(dev, &dev_attr_module_pwm1) &&
		 !device_create_file(dev, &dev_attr_module_pwma) &&
		 !device_create_file(dev, &dev_attr_module_pwmb) &&
		 !device_create_file(dev, &dev_attr_module_keypad) &&
		 !device_create_file(dev, &dev_attr_module_backup) &&
		 !device_create_file(dev, &dev_attr_module_int) &&
		 !device_create_file(dev, &dev_attr_module_pm_master) &&
		 !device_create_file(dev, &dev_attr_module_pm_receiver) &&
		 !device_create_file(dev, &dev_attr_module_rtc) &&
		 !device_create_file(dev, &dev_attr_module_secured_reg));

	return ret;
}

static void twl4030_sysfs_debug_remove(void)
{
	if (!twl4030_debug_dev)
		return;

	platform_device_unregister(twl4030_debug_dev);
	kfree(twl4030_debug_dev);
}

#else
#define	twl4030_sysfs_debug_create()	/* no definition */
#define	twl4030_sysfs_debug_remove()	/* no definition */
#endif

extern int power_companion_init(void);

/**** Helper functions */
static int
twl4030_detect_client(struct i2c_adapter *adapter, unsigned char sid);
static int twl4030_attach_adapter(struct i2c_adapter *adapter);
static int twl4030_detach_client(struct i2c_client *client);
static void do_twl4030_irq(unsigned int irq, irq_desc_t *desc);

static void twl_init_irq(void);

/**** Data Structures */
/* To have info on T2 IRQ substem activated or not */
static unsigned char twl_irq_used = FREE;
static struct completion irq_event;

/* Structure to define on TWL4030 Slave ID */
struct twl4030_client {
	struct i2c_client client;
	const char client_name[sizeof(TWL_CLIENT_STRING) + 1];
	const unsigned char address;
	const char adapter_index;
	unsigned char inuse;

	/* max numb of i2c_msg required is for read =2 */
	struct i2c_msg xfer_msg[2];

	/* To lock access to xfer_msg */
	struct mutex xfer_lock;
};

/* Module Mapping */
struct twl4030mapping {
	unsigned char sid;	/* Slave ID */
	unsigned char base;	/* base address */
};

/* mapping the module id to slave id and base address */
static struct twl4030mapping twl4030_map[TWL4030_MODULE_LAST + 1] = {
	{ TWL4030_SLAVENUM_NUM0, TWL4030_BASEADD_USB },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_AUDIO_VOICE },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_GPIO },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_INTBR },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_PIH },
	{ TWL4030_SLAVENUM_NUM1, TWL4030_BASEADD_TEST },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_KEYPAD },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_MADC },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_INTERRUPTS },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_LED },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_MAIN_CHARGE },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PRECHARGE },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWM0 },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWM1 },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWMA },
	{ TWL4030_SLAVENUM_NUM2, TWL4030_BASEADD_PWMB },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_BACKUP },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_INT },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_PM_MASTER },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_PM_RECEIVER },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_RTC },
	{ TWL4030_SLAVENUM_NUM3, TWL4030_BASEADD_SECURED_REG },
};

static struct twl4030_client twl4030_modules[TWL4030_NUM_SLAVES] = {
	{
		.address	= TWL4030_SLAVEID_ID0,
		.client_name	= TWL_CLIENT_STRING "0",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
	{
		.address	= TWL4030_SLAVEID_ID1,
		.client_name	= TWL_CLIENT_STRING "1",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
	{
		.address	= TWL4030_SLAVEID_ID2,
		.client_name	= TWL_CLIENT_STRING "2",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
	{
		.address	= TWL4030_SLAVEID_ID3,
		.client_name	= TWL_CLIENT_STRING "3",
		.adapter_index	= CONFIG_I2C_TWL4030_ID,
	},
};

/* One Client Driver , 4 Clients */
static struct i2c_driver twl4030_driver = {
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.attach_adapter	= twl4030_attach_adapter,
	.detach_client	= twl4030_detach_client,
};

/*
 * TWL4030 doesn't have PIH mask, hence dummy function for mask
 * and unmask.
 */

static void twl4030_i2c_ackirq(unsigned int irq) {}
static void twl4030_i2c_disableint(unsigned int irq) {}
static void twl4030_i2c_enableint(unsigned int irq) {}

#ifdef USE_GPIO_IRQ
static int 
twl4030_i2c_set_wake(unsigned int irq, unsigned int on)
{
	if( on )
		return enable_irq_wake(gpio_to_irq(USE_GPIO_IRQ));
	else 
		return disable_irq_wake(gpio_to_irq(USE_GPIO_IRQ));
}
#else
#define twl4030_i2c_set_wake NULL
#endif

/* information for processing in the Work Item */
static struct irq_chip twl4030_irq_chip = {
	.name	  = "TWL4030",
	.ack	  = twl4030_i2c_ackirq,
	.mask	  = twl4030_i2c_disableint,
	.unmask	  = twl4030_i2c_enableint,
	.set_wake = twl4030_i2c_set_wake,
};

/* Global Functions */
/*
 * @brief twl4030_i2c_write - Writes a n bit register in TWL4030
 *
 * @param mod_no - module number
 * @param *value - an array of num_bytes+1 containing data to write
 * IMPORTANT - Allocate value num_bytes+1 and valid data starts at
 *		 Offset 1.
 * @param reg - register address (just offset will do)
 * @param num_bytes - number of bytes to transfer
 *
 * @return result of operation - 0 is success
 */
int twl4030_i2c_write(u8 mod_no, u8 * value, u8 reg, u8 num_bytes)
{
	int ret;
	int sid;
	struct twl4030_client *twl;
	struct i2c_msg *msg;

	if (unlikely(mod_no > TWL4030_MODULE_LAST)) {
		pr_err("Invalid module Number\n");
		return -EPERM;
	}
	sid = twl4030_map[mod_no].sid;
	twl = &twl4030_modules[sid];

	if (unlikely(twl->inuse != TWL_CLIENT_USED)) {
		pr_err("I2C Client[%d] is not initialized[%d]\n",
		       sid,__LINE__);
		return -EPERM;
	}
	mutex_lock(&twl->xfer_lock);
	/*
	 * [MSG1]: fill the register address data
	 * fill the data Tx buffer
	 */
	msg = &twl->xfer_msg[0];
	msg->addr = twl->address;
	msg->len = num_bytes + 1;
	msg->flags = 0;
	msg->buf = value;
	/* over write the first byte of buffer with the register address */
	*value = twl4030_map[mod_no].base + reg;
	ret = i2c_transfer(twl->client.adapter, twl->xfer_msg, 1);
	mutex_unlock(&twl->xfer_lock);

	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret >= 0)
		ret = 0;
	return ret;
}

/**
 * @brief twl4030_i2c_read - Reads a n bit register in TWL4030
 *
 * @param mod_no - module number
 * @param *value - an array of num_bytes containing data to be read
 * @param reg - register address (just offset will do)
 * @param num_bytes - number of bytes to transfer
 *
 * @return result of operation - num_bytes is success else failure.
 */
int twl4030_i2c_read(u8 mod_no, u8 * value, u8 reg, u8 num_bytes)
{
	int ret;
	u8 val;
	int sid;
	struct twl4030_client *twl;
	struct i2c_msg *msg;

	if (unlikely(mod_no > TWL4030_MODULE_LAST)) {
		pr_err("Invalid module Number\n");
		return -EPERM;
	}
	sid = twl4030_map[mod_no].sid;
	twl = &twl4030_modules[sid];

	if (unlikely(twl->inuse != TWL_CLIENT_USED)) {
		pr_err("I2C Client[%d] is not initialized[%d]\n", sid,
		       __LINE__);
		return -EPERM;
	}
	mutex_lock(&twl->xfer_lock);
	/* [MSG1] fill the register address data */
	msg = &twl->xfer_msg[0];
	msg->addr = twl->address;
	msg->len = 1;
	msg->flags = 0;	/* Read the register value */
	val = twl4030_map[mod_no].base + reg;
	msg->buf = &val;
	/* [MSG2] fill the data rx buffer */
	msg = &twl->xfer_msg[1];
	msg->addr = twl->address;
	msg->flags = I2C_M_RD;	/* Read the register value */
	msg->len = num_bytes;	/* only n bytes */
	msg->buf = value;
	ret = i2c_transfer(twl->client.adapter, twl->xfer_msg, 2);
	mutex_unlock(&twl->xfer_lock);

	/* i2cTransfer returns num messages.translate it pls.. */
	if (ret >= 0)
		ret = 0;
	return ret;
}

/**
 * @brief twl4030_i2c_write_u8 - Writes a 8 bit register in TWL4030
 *
 * @param mod_no - module number
 * @param value - the value to be written 8 bit
 * @param reg - register address (just offset will do)
 *
 * @return result of operation - 0 is success
 */
int twl4030_i2c_write_u8(u8 mod_no, u8 value, u8 reg)
{

	/* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
	u8 temp_buffer[2] = { 0 };
	/* offset 1 contains the data */
	temp_buffer[1] = value;
	return twl4030_i2c_write(mod_no, temp_buffer, reg, 1);
}

/**
 * @brief twl4030_i2c_read_u8 - Reads a 8 bit register from TWL4030
 *
 * @param mod_no - module number
 * @param *value - the value read 8 bit
 * @param reg - register address (just offset will do)
 *
 * @return result of operation - 0 is success
 */
int twl4030_i2c_read_u8(u8 mod_no, u8 * value, u8 reg)
{
	return twl4030_i2c_read(mod_no, value, reg, 1);
}

/**** Helper Functions */

/*
 * do_twl4030_module_irq() is the desc->handle method for each of the twl4030
 * module interrupts.  It executes in kernel thread context.
 * On entry, cpu interrupts are disabled.
 */
static void do_twl4030_module_irq(unsigned int irq, irq_desc_t *desc)
{
	struct irqaction *action;
	const unsigned int cpu = smp_processor_id();

	/*
	 * Earlier this was desc->triggered = 1;
	 */
	desc->status |= IRQ_LEVEL;

	/*
	 * The desc->handle method would normally call the desc->chip->ack
	 * method here, but we won't bother since our ack method is NULL.
	 */

	if (!desc->depth) {
		kstat_cpu(cpu).irqs[irq]++;

		action = desc->action;
		if (action) {
			int ret;
			int status = 0;
			int retval = 0;

			local_irq_enable();

			do {
				/* Call the ISR with cpu interrupts enabled */
				ret = action->handler(irq, action->dev_id);
				if (ret == IRQ_HANDLED)
					status |= action->flags;
				retval |= ret;
				action = action->next;
			} while (action);

			if (status & IRQF_SAMPLE_RANDOM)
				add_interrupt_randomness(irq);

			local_irq_disable();

			if (retval != IRQ_HANDLED)
				printk(KERN_ERR "ISR for TWL4030 module"
					" irq %d can't handle interrupt\n", irq);

			/*
			 * Here is where we should call the unmask method, but
			 * again we won't bother since it is NULL.
			 */
		} else
			printk(KERN_CRIT "TWL4030 module irq %d has no ISR"
					" but can't be masked!\n", irq);
	} else
		printk(KERN_CRIT "TWL4030 module irq %d is disabled but can't"
				" be masked!\n", irq);
}

/*
 * twl4030_irq_thread() runs as a kernel thread.  It queries the twl4030
 * interrupt controller to see which modules are generating interrupt requests
 * and then calls the desc->handle method for each module requesting service.
 */
static int twl4030_irq_thread(void *data)
{
	int irq = (int)data;
	irq_desc_t *desc = irq_desc + irq;
	static unsigned i2c_errors = 0;
	const static unsigned max_i2c_errors = 100;

	daemonize("twl4030-irq");
	current->flags |= PF_NOFREEZE;

	set_user_nice(current, -5);

	while (!kthread_should_stop()) {
		int ret;
		int module_irq;
		u8  pih_isr;

		wait_for_completion_interruptible(&irq_event);

again:
		ret = twl4030_i2c_read_u8(TWL4030_MODULE_PIH, &pih_isr,
					  REG_PIH_ISR_P1);
		if (ret) {
			printk(KERN_WARNING "I2C error %d while reading TWL4030"
					" PIH ISR register.\n", ret);
			if (++i2c_errors >= max_i2c_errors) {
				printk(KERN_ERR "Maximum I2C error count"
						" exceeded.  Terminating %s.\n",
						__FUNCTION__);
				break;
			}
			continue;
		}

		for (module_irq = IH_TWL4030_BASE; 0 != pih_isr;
			 pih_isr >>= 1, module_irq++) {
			if (pih_isr & 0x1) {
				irq_desc_t *d = irq_desc + module_irq;

				local_irq_disable();

				d->handle_irq(module_irq, d);

				local_irq_enable();
			}
		}

		ret = twl4030_i2c_read_u8(TWL4030_MODULE_PIH, &pih_isr,
					  REG_PIH_ISR_P1);
		if( ret != 0 || pih_isr ) { 
		        // an error or some interrupt is still pending
			goto again;
		}

		desc->chip->unmask(irq);
	}

	return 0;
}

/*
 * do_twl4030_irq() is the desc->handle method for the twl4030 interrupt.
 * This is a chained interrupt, so there is no desc->action method for it.
 * Now we need to query the interrupt controller in the twl4030 to determine
 * which module is generating the interrupt request.  However, we can't do i2c
 * transactions in interrupt context, so we must defer that work to a kernel
 * thread.  All we do here is acknowledge and mask the interrupt and wakeup
 * the kernel thread.
 */
static void do_twl4030_irq(unsigned int irq, irq_desc_t *desc)
{
	const unsigned int cpu = smp_processor_id();

	/*
	 * Earlier this was desc->triggered = 1;
	 */
	desc->status |= IRQ_LEVEL;

	/*
	 * Acknowledge, clear _AND_ disable the interrupt.
	 */
	desc->chip->mask(irq);
	desc->chip->ack(irq);

	if (!desc->depth) {
		kstat_cpu(cpu).irqs[irq]++;

		complete(&irq_event);
	}
}

/* attach a client to the adapter */
static int twl4030_detect_client(struct i2c_adapter *adapter, unsigned char sid)
{
	int err = 0;
	struct twl4030_client *twl;

	if (unlikely(sid >= TWL4030_NUM_SLAVES)) {
		pr_err("sid[%d] > MOD_LAST[%d]\n", sid, TWL4030_NUM_SLAVES);
		return -EPERM;
	}

	/* Check basic functionality */
	if (!(err = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA |
						I2C_FUNC_SMBUS_WRITE_BYTE))) {
		pr_err("SlaveID=%d functionality check failed\n", sid);
		return err;
	}
	twl = &twl4030_modules[sid];
	if (unlikely(twl->inuse)) {
		pr_err("Client %s is already in use\n", twl->client_name);
		return -EPERM;
	}

	memset(&twl->client, 0, sizeof(struct i2c_client));

	twl->client.addr	= twl->address;
	twl->client.adapter	= adapter;
	twl->client.driver	= &twl4030_driver;

	memcpy(&twl->client.name, twl->client_name,
			sizeof(TWL_CLIENT_STRING) + 1);

	pr_info("TWL4030: TRY attach Slave %s on Adapter %s [%x]\n",
				twl->client_name, adapter->name, err);

	if ((err = i2c_attach_client(&twl->client))) {
		pr_err("Couldn't attach Slave %s on Adapter"
		       "%s [%x]\n", twl->client_name, adapter->name, err);
	} else {
		twl->inuse = TWL_CLIENT_USED;
		mutex_init(&twl->xfer_lock);
	}

	return err;
}

/* adapter callback */
static int twl4030_attach_adapter(struct i2c_adapter *adapter)
{
	int i;
	int ret = 0;
	static int twl_i2c_adapter = 1;

	for (i = 0; i < TWL4030_NUM_SLAVES; i++) {
		/* Check if I need to hook on to this adapter or not */
		if (twl4030_modules[i].adapter_index == twl_i2c_adapter) {
			if ((ret = twl4030_detect_client(adapter, i)))
				goto free_client;
		}
	}
	twl_i2c_adapter++;

	/*
	 * Check if the PIH module is initialized, if yes, then init
	 * the T2 Interrupt subsystem
	 */
	if ((twl4030_modules[twl4030_map[TWL4030_MODULE_PIH].sid].inuse ==
		TWL_CLIENT_USED) && (twl_irq_used != USED)) {
		twl_init_irq();
		twl_irq_used = USED;
	}

	twl4030_sysfs_debug_create();

	return 0;

free_client:
	pr_err("TWL_CLIENT(Idx=%d] registration failed[0x%x]\n",i,ret);

	/* ignore current slave..it never got registered */
	i--;
	while (i >= 0) {
		/* now remove all those from the current adapter... */
		if (twl4030_modules[i].adapter_index == twl_i2c_adapter)
			(void)twl4030_detach_client(&(twl4030_modules[i].client));
		i--;
	}
	return ret;
}

/* adapter's callback */
static int twl4030_detach_client(struct i2c_client *iclient)
{
	int err;

	twl4030_sysfs_debug_remove();
	if ((err = i2c_detach_client(iclient))) {
		pr_err("Client detach failed\n");
		return err;
	}
	return 0;
}

struct task_struct *start_twl4030_irq_thread(int irq)
{
	struct task_struct *thread;

	init_completion(&irq_event);
	thread = kthread_run(twl4030_irq_thread, (void *)irq,
			     "twl4030 irq %d", irq);
	if (!thread)
		pr_err("%s: could not create twl4030 irq %d thread!\n",
		       __FUNCTION__,irq);

	return thread;
}

static void twl_init_irq(void)
{
	int	i = 0;
	int	res = 0;
	char	*msg = "Unable to register interrupt subsystem";

#ifdef USE_GPIO_IRQ
	omap_cfg_reg (USE_GPIO_IRQ_PIN_MUX);
        gpio_request (USE_GPIO_IRQ, "sys_nreq_gpio");
	gpio_direction_input(USE_GPIO_IRQ);
#endif	
	/*
	 * We end up with interrupts from other modules before
	 * they get a chance to handle them...
	 */
	/* PWR_ISR1 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INT, 0xFF, 0x00);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* PWR_ISR2 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INT, 0xFF, 0x02);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* PWR_IMR1 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INT, 0xFF, 0x1);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* PWR_IMR2 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INT, 0xFF, 0x3);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* Clear off any other pending interrupts on power */
	/* PWR_ISR1 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INT, 0xFF, 0x00);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* PWR_ISR2 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INT, 0xFF, 0x02);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}
	/* POWER HACK (END) */
	/* Slave address 0x4A */

	/* BCIIMR1_1 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 0xFF, 0x3);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* BCIIMR1_2 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 0xFF, 0x4);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* BCIIMR2_1 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 0xFF, 0x7);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* BCIIMR2_2 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 0xFF, 0x8);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* MAD C */
	/* MADC_IMR1 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_MADC, 0xFF, 0x62);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* MADC_IMR2 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_MADC, 0xFF, 0x64);
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* key Pad */
	/* KEYPAD - IMR1 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_KEYPAD, 0xFF, (0x12));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}
	{
		u8 clear;
		/* Clear ISR */
		twl4030_i2c_read_u8(TWL4030_MODULE_KEYPAD, &clear, 0x11);
		twl4030_i2c_read_u8(TWL4030_MODULE_KEYPAD, &clear, 0x11);
	}

	/* KEYPAD - IMR2 */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_KEYPAD, 0xFF, (0x14));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* Slave address 0x49 */
	/* GPIO_IMR1A */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0xFF, (0x1C));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* GPIO_IMR2A */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0xFF, (0x1D));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* GPIO_IMR3A */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0xFF, (0x1E));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* GPIO_IMR1B */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0xFF, (0x22));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* GPIO_IMR2B */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0xFF, (0x23));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* GPIO_IMR3B */
	res = twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0xFF, (0x24));
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	res = power_companion_init();
	if (res < 0) {
		pr_err("%s[%d][%d]\n", msg, res, __LINE__);
		return;
	}

	/* install an irq handler for each of the PIH modules */
	for (i = IH_TWL4030_BASE; i < IH_TWL4030_END; i++) {
		set_irq_chip(i, &twl4030_irq_chip);
		set_irq_handler(i, do_twl4030_module_irq);
		set_irq_flags(i, IRQF_VALID);
	}

#if 0
#if defined(CONFIG_OMAP34XX_OFFMODE)
	/* Enable T2 SYS_NIRQ as wakeup capable in OFF mode: 
	 * Enable I/O wakeup for Off mode
         */
	omap_writel( omap_readl(0x480021E0)|
			(1<<9)|(1<<12)|(1<<13)|(1<<14), /* Off mode enable */
			0x480021E0);
#endif
#endif

	/* install an irq handler to demultiplex the TWL4030 interrupt */
#ifdef USE_GPIO_IRQ
	start_twl4030_irq_thread(gpio_to_irq(USE_GPIO_IRQ));
	set_irq_data(gpio_to_irq(USE_GPIO_IRQ), NULL);
//	set_irq_type(gpio_to_irq(USE_GPIO_IRQ), IRQ_TYPE_LEVEL_LOW);
	set_irq_type(gpio_to_irq(USE_GPIO_IRQ), IRQT_FALLING);
	set_irq_chained_handler(gpio_to_irq(USE_GPIO_IRQ), do_twl4030_irq);
#else
	set_irq_data(TWL4030_IRQNUM, start_twl4030_irq_thread(TWL4030_IRQNUM));
	set_irq_type(TWL4030_IRQNUM, IRQT_FALLING);
	set_irq_chained_handler(TWL4030_IRQNUM, do_twl4030_irq);
#endif	

}

static int __init twl4030_init(void)
{
	int res;

	if ((res = i2c_add_driver(&twl4030_driver))) {
		printk(KERN_ERR "TWL4030: Driver registration failed \n");
		return res;
	}

	pr_info("TWL4030: Driver registration complete.\n");

	return 0;
}

static void __exit twl4030_exit(void)
{
	i2c_del_driver(&twl4030_driver);
	twl_irq_used = FREE;
}

subsys_initcall(twl4030_init);
module_exit(twl4030_exit);

EXPORT_SYMBOL(twl4030_i2c_write_u8);
EXPORT_SYMBOL(twl4030_i2c_read_u8);
EXPORT_SYMBOL(twl4030_i2c_read);
EXPORT_SYMBOL(twl4030_i2c_write);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("I2C Core interface for TWL4030");
MODULE_LICENSE("GPL");
