/*
 * twl4030_madc - TWL4030 MADC
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Current status:
 */

#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/device.h>

#include <asm/arch/twl4030.h>
#include <asm/arch/irqs.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <asm/arch/mux.h>
#include <asm/arch/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>

#include <asm/arch/gpio.h>

#include <asm/arch/twl4030-madc.h>

#include <asm/arch/resource.h>

/****************************************
* MADC Block Register definitions
****************************************/

#define REG_CTRL1			(0x00)
#define REG_SW1SELECT_LSB		(0x06)
#define REG_SW1SELECT_MSB		(0x07)
#define REG_SW1AVERAGE_LSB		(0x08)
#define REG_SW1AVERAGE_MSB		(0x09)
#define REG_GPBR1			(0x0C)
#define REG_CTRL_SW1			(0x12)


#define REG_GPCH0_LSB			(0x37)
#define REG_GPCH0_MSB			(0x38)

#define REG_GPCH1_LSB			(0x39)
#define REG_GPCH1_MSB			(0x3A)
#define REG_GPCH2_LSB			(0x3B)
#define REG_GPCH2_MSB			(0x3C)
#define REG_GPCH3_LSB			(0x3D)
#define REG_GPCH3_MSB			(0x3E)
#define REG_GPCH4_LSB			(0x3F)
#define REG_GPCH4_MSB			(0x40)
#define REG_GPCH5_LSB			(0x41)
#define REG_GPCH5_MSB			(0x42)
#define REG_GPCH6_LSB			(0x43)
#define REG_GPCH6_MSB			(0x44)
#define REG_GPCH7_LSB			(0x45)
#define REG_GPCH7_MSB			(0x46)
#define REG_GPCH8_LSB			(0x47)
#define REG_GPCH8_MSB			(0x48)
#define REG_GPCH9_LSB			(0x49)
#define REG_GPCH9_MSB			(0x4A)
#define REG_GPCH10_LSB			(0x4B)
#define REG_GPCH10_MSB			(0x4C)
#define REG_GPCH11_LSB			(0x4D)
#define REG_GPCH11_MSB			(0x4E)
#define REG_GPCH12_LSB			(0x4F)
#define REG_GPCH12_MSB			(0x50)
#define REG_GPCH13_LSB			(0x51)
#define REG_GPCH13_MSB			(0x52)
#define REG_GPCH14_LSB			(0x53)
#define REG_GPCH14_MSB			(0x54)
#define REG_GPCH15_LSB			(0x55)
#define REG_GPCH15_MSB			(0x56)



#define REG_MADC_IMR1			(0x62)
#define REG_MADC_ISR1			(0x61)
#define REG_MADC_EDR			(0x66)

/**** BitField Definitions */
#define MADC_ON				(1<<0)
#define SW1_IMR1			(1<<1)
#define SW1_ISR1			(1<<1)
#define SW1_CH2				(1<<2)
#define SW1_EDRRISING			(1<<2)
#define SW1_EDRFALLING			(1<<3)
#define MADC_HFCLK_EN			(1<<7)
#define AV_CH2				(1<<2)
#define MADC_SW1			(1<<5)

#define MADC_NAME			"twl4030-madc"
#define TYPE_NUM			(4)
#define NR_CHANNELS			16

struct twl4030_madc_channel {
	int lsb_addr;
	int msb_addr;
};

struct twl4030_madc_state{
	struct semaphore madc_sem;
	struct completion *madc_complete;
	int cur_channel;
};

static struct twl4030_madc_state *twl;
static struct twl4030_madc_channel twl4030_madc_channels[] = {
	[0] = {
		.lsb_addr = REG_GPCH0_LSB,
		.msb_addr = REG_GPCH0_MSB,
			},	
	[1] = {
		.lsb_addr = REG_GPCH1_LSB,
		.msb_addr = REG_GPCH1_MSB,
		},
	[2] = {
		.lsb_addr = REG_GPCH2_LSB,
		.msb_addr = REG_GPCH2_MSB,
			},
	[3] = {
		.lsb_addr = REG_GPCH3_LSB,
		.msb_addr = REG_GPCH3_MSB,
			},
	[4] = {
		.lsb_addr = REG_GPCH4_LSB,
		.msb_addr = REG_GPCH4_MSB,
			},
	[5] = {
		.lsb_addr = REG_GPCH5_LSB,
		.msb_addr = REG_GPCH5_MSB,
			},
	[6] = {
		.lsb_addr = REG_GPCH6_LSB,
		.msb_addr = REG_GPCH6_MSB,
			},
	[7] = {
		.lsb_addr = REG_GPCH7_LSB,
		.msb_addr = REG_GPCH7_MSB,
			},

	[8] = {
		.lsb_addr = REG_GPCH8_LSB,
		.msb_addr = REG_GPCH8_MSB,
		},
	[9] = {
		.lsb_addr = REG_GPCH9_LSB,
		.msb_addr = REG_GPCH9_MSB,
			},
	[10] = {
		.lsb_addr = REG_GPCH10_LSB,
		.msb_addr = REG_GPCH10_MSB,
			},
	[11] = {
		.lsb_addr = REG_GPCH11_LSB,
		.msb_addr = REG_GPCH11_MSB,
			},
	[12] = {
		.lsb_addr = REG_GPCH12_LSB,
		.msb_addr = REG_GPCH12_MSB,
			},
	[13] = {
		.lsb_addr = REG_GPCH13_LSB,
		.msb_addr = REG_GPCH13_MSB,
			},
	[14] = {
		.lsb_addr = REG_GPCH14_LSB,
		.msb_addr = REG_GPCH14_MSB,
			},	
	[15] = {
		.lsb_addr = REG_GPCH15_LSB,
		.msb_addr = REG_GPCH15_MSB,
			},	
};

/*
 * To set TWL4030 MADC module registers bits
 */
static int twl4030_madc_set_bits(u8 mod_no, u8 bits, u8 reg)
{
	u8 val;
	int ret;

	ret = twl4030_i2c_read_u8(mod_no, &val, reg);

	if (!ret)
	{
		val |= bits;
		ret = twl4030_i2c_write_u8(mod_no, val, reg);
	}

	return ret;
}

/*
 * To clear TWL4030 MADC module registers bits
 */
static int twl4030_madc_clear_bits(u8 mod_no, u8 bits, u8 reg)
{
	u8 val;
	int ret;

	ret = twl4030_i2c_read_u8(mod_no, &val, reg);

	if (!ret)
	{
		val &= ~bits;
		ret = twl4030_i2c_write_u8(mod_no, val, reg);
	}

	return ret;
}

/*
 * To read a TWL4030 MADC module register
 */
static int twl4030_madc_read(u8 address)
{
	u8 data;
	int ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_MADC, &data, address);
	if (ret >= 0)
		ret = data;
	return ret;
}

/*
 * To configure TWL4030 MADC module registers
 */
static int twl4030_madc_write(u8 address, u8 data)
{
	int ret = 0;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_MADC, data, address);
	return ret;
}

/*
 * To turn on/off TWL4030 MADC module registers
 */
static int twl4030_madc_clk(int enable)
{
	int ret;

	if (enable) {
	    /* Enable MADC_HFCLK Clock */
		ret = twl4030_madc_set_bits(TWL4030_MODULE_INTBR, MADC_HFCLK_EN, REG_GPBR1);
		if (ret < 0) 
			goto err;
	} else {
		ret = twl4030_madc_clear_bits(TWL4030_MODULE_INTBR, MADC_HFCLK_EN, REG_GPBR1);
		if (ret < 0) 
			goto err;
	}

	return ret;
err:
	printk(KERN_ERR "Unable to turn on/off madc clk[%d]\n", ret);
	return ret; 
}

/*
 * Enabled MADC 
 */

int madc_enable(int index)
{
	int ret;
	/* Enable MADC_HFLK Clock */
	ret = twl4030_madc_clk(1);
	if (ret < 0)
       	goto err;
		
	/* Set up the MADC IMR1 */ 
	ret = twl4030_madc_clear_bits(TWL4030_MODULE_MADC, SW1_IMR1, REG_MADC_IMR1);
	if (ret < 0)
		goto err1;
	
	/* Set up the MADC EDR */ 
	ret = twl4030_madc_set_bits(TWL4030_MODULE_MADC,(SW1_EDRRISING|SW1_EDRFALLING), REG_MADC_EDR);
	if (ret < 0)
		goto err1;
	
	/* Power on the MADC */
	ret = twl4030_madc_set_bits(TWL4030_MODULE_MADC, MADC_ON, REG_CTRL1);
	if (ret < 0) 
		goto err1;
	
	return 0;
	
err1:
	twl4030_madc_clk(0);

err:
	return ret;
}

/*
 * Reset MADC 
 */

void madc_reset(void)
{
	/* Disable MADC_HFLK Clock */
	twl4030_madc_clk(0);

	/* Clear up the MADC IMR1 */ 
	twl4030_madc_set_bits(TWL4030_MODULE_MADC, SW1_IMR1, REG_MADC_IMR1);

	/* Clear up the MADC EDR */ 
	twl4030_madc_clear_bits(TWL4030_MODULE_MADC,(SW1_EDRRISING|SW1_EDRFALLING), REG_MADC_EDR);

	/* Power off the MADC */
	twl4030_madc_clear_bits(TWL4030_MODULE_MADC, MADC_ON, REG_CTRL1);
}

/*
 * Start MADC conversion process
 */
int madc_start_conversion(int index)
{
	u8 chan[3];
	int ret;
	int lsb, msb;	
	DECLARE_COMPLETION_ONSTACK(convert_done);

	if (index >= NR_CHANNELS)
		return (-EINVAL);

	down(&twl->madc_sem);

	twl->cur_channel = index;

	twl->madc_complete = &convert_done;
	
	ret = madc_enable(index);
	if (ret < 0) 
		goto err;

	/* select channel */
	chan[1] = index < 8 ? 1 << index : 0;
	chan[2] = index >= 8 ? 1 << (index - 8) : 0;
	ret = twl4030_i2c_write(TWL4030_MODULE_MADC, chan, REG_SW1SELECT_LSB,
				2);

	/* Start the conversion on SW1 channel */
	ret = twl4030_madc_set_bits(TWL4030_MODULE_MADC, MADC_SW1, REG_CTRL_SW1);
	if (ret < 0){
		printk(KERN_ERR "Unable to start the conversion on MADC channel[%d]\n", ret);
		goto err;
	}

	/* Wait for completion here */
	ret = wait_for_completion_interruptible_timeout(&convert_done, 
			msecs_to_jiffies(1000));
	if (ret == 0) {
#define NREGS    255
		u8 regbuf[NREGS + 1];
		printk(KERN_ERR "madc: timed out\n");
		if (twl4030_i2c_read(TWL4030_MODULE_MADC, regbuf, 0, NREGS) < 0) {
			printk(KERN_ERR "madc: unable to read registers\n");
		} else {
			int j;
			char *b = kmalloc(NREGS * 4, GFP_KERNEL);
			char *buf;

			buf = b;
			buf += sprintf(buf, "  | ");
			for (j = 0; j < 0x10; j++) {
				buf += sprintf(buf, "%02X ", j);
			}
			buf += sprintf(buf, "\n--+");

			for (j = 0; j < 0x10; j++) {
				buf += sprintf(buf, " --");
			}
			buf += sprintf(buf, "\n00| ");

			for (j = 0; j <= NREGS; j++) {
				buf += sprintf(buf, "%02X", regbuf[j]);
				if (j < NREGS) {
					if ((j + 1) % 0x10) {
						buf += sprintf(buf, " ");
					} else {
						buf += sprintf(buf, "\n%02X| ", j + 1);
					}
				}
			}
			*buf = '\0';
			printk(KERN_ERR "\n%s\n", b);
			kfree(b);
		}
		ret = -1;
		goto err;
	}

	/* Read conversion result */
	lsb = twl4030_madc_read(twl4030_madc_channels[index].lsb_addr);	
	msb = twl4030_madc_read(twl4030_madc_channels[index].msb_addr);
	
	if (lsb < 0){
		ret = lsb;
	} else if ( msb < 0) {
		ret = msb;
	} else {
		ret = (msb << 2)|((lsb >> 6) & 0x3);
	} 
	
	madc_reset();
	
err:
	twl->madc_complete = NULL;
	up(&twl->madc_sem);
	return ret;	
}

/*
 * Enable option of averaging the conversion result
 */

int madc_enable_averaging(int index, int on)
{
	int ret; 

	if (on)
	{
		if (index < 8){
			ret = twl4030_madc_set_bits(TWL4030_MODULE_MADC, (1<<index), REG_SW1AVERAGE_LSB);
		} else {
			ret = twl4030_madc_set_bits(TWL4030_MODULE_MADC, (1<<(index-8)), REG_SW1AVERAGE_MSB);
		}
		if (ret < 0)
			goto err;
	} else {
		if (index < 8){
			ret = twl4030_madc_clear_bits(TWL4030_MODULE_MADC, (1<<index), REG_SW1AVERAGE_LSB);
		} else {
			ret = twl4030_madc_clear_bits(TWL4030_MODULE_MADC, (1<<(index-8)), REG_SW1AVERAGE_MSB);
		}
		if (ret < 0) 
			goto err;
	}
	return ret;
err:
	printk(KERN_ERR "Unable to set madc average[%d]\n", ret);
	return ret;
}

irqreturn_t twl4030_madc_isr(int isr, void *dev_id)
{
	int ret;

	ret = twl4030_madc_read(REG_MADC_ISR1);
	if ((ret & SW1_ISR1 )&& (twl->madc_complete)) {
		complete(twl->madc_complete);
		twl->madc_complete = NULL;
	}
	
	return IRQ_HANDLED;
}

/* TWL4030 Initialization module */
static int __init twl4030_madc_init(void)
{
	int ret = 0;	

	twl = kzalloc ( sizeof(struct twl4030_madc_state), GFP_KERNEL);
   	if(!twl)
        	return (-ENOMEM);

	init_MUTEX(&twl->madc_sem);

	/* Register MADC interrupt. */
	ret = request_irq(TWL4030_MODIRQ_MADC, twl4030_madc_isr,
				IRQF_SHARED,
				MADC_NAME,
				twl);

	if (ret < 0){
		printk(KERN_ERR "can't get IRQ %d, err %d\n", TWL4030_MODIRQ_MADC, ret);
		kfree(twl);
		return -ENODEV;
	}

	return ret;
}
/* TWL MADC exit module */
static void __exit twl4030_madc_exit(void)
{
	free_irq(TWL4030_MODIRQ_MADC, twl4030_madc_isr);
	kfree(twl);
}

EXPORT_SYMBOL(madc_start_conversion);
EXPORT_SYMBOL(madc_enable_averaging);

subsys_initcall(twl4030_madc_init);
module_exit(twl4030_madc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brian Xiong <zyxiong@gmail.com>");
MODULE_DESCRIPTION("MADC driver");
