/* arch/arm/mach-msm/power_debug.c
 *
 * MSM7xxx power debugging support
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Kevin McCray <kevin.mccray@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/vreg.h>
#include "clock.h"

#include "proc_comm.h"

#define MODULE_NAME "msm_power_debug"


enum {
	MSM_POWER_DEBUG_CLOCKS = 1U << 0,
	MSM_POWER_DEBUG_VREGS = 1U << 1,
};

static int msm_power_debug_mask = 0x00;
module_param_named(
	debug_mask, msm_power_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

#define MSM_POWER_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & msm_power_debug_mask) \
			printk(level message, ## __VA_ARGS__); \
	} while (0)

char * clock_names[] = {
	"acpu",
	"adm",
	"adsp",
	"ebi1",
	"ebi2",
	"ecodec_clk",
	"emdh_clk",
	"gp_clk",
	"grp_clk",
	"i2c_clk",
	"icodec_rx_clk",
	"icodec_tx_clk",
	"imem_clk",
	"mdc_clk",
	"mdp_clk",
	"pbus_clk",
	"pcm_clk",
	"pmdh_clk",
	"sdac_clk",
	"sdc1_clk",
	"sdc1_pclk",
	"sdc2_clk",
	"sdc2_pclk",
	"sdc3_clk",
	"sdc3_pclk",
	"sdc4_clk",
	"sdc4_pclk",
	"tsif_clk",
	"tsif_ref_clk",
	"tv_dac_clk",
	"tv_enc_clk",
	"uart1_clk",
	"uart2_clk",
	"uart3_clk",
	"uart1dm_clk",
	"uart2dm_clk",
	"usb_hs_clk",
	"usb_hs_pclk",
	"usb_otg_clk",
	"vdc_clk",
	"vfe_mdc_clk",
	"vfe_clk",
	"mdp_lcdc_pclk_clk",
	"mdp_lcdc_pad_pclk_clk",
	"mdp_vsync_clk",
	"spi_clk",
	"vfe_axi_clk"
};


char *vreg_names[] = {
	"msma",
	"msmp",
	"msme1",
	"msmc1",
	"msmc2",
	"gp3",
	"msme2",
	"gp4",
	"gp1",
	"tcxo",
	"pa",
	"rftx",
	"rfrx1",
	"rfrx2",
	"synt",
	"wlan",
	"usb",
	"boost",
	"mmc",
	"ruim",
	"msmc0",
	"gp2",
	"gp5",
	"gp6",
	"rf",
	"rf_vco",
	"mpll",
	"s2",
	"s3",
	"rfubm",
	"ncp"
};


static unsigned pc_clk_get_rate(unsigned id)
{
	if (msm_proc_comm(PCOM_CLKCTL_RPC_RATE, &id, 0))
		return 0;
	else
		return id;
}

void get_enabled_clocks(char *msg)
{
	int i;
	unsigned id;

	if(!(msm_power_debug_mask & MSM_POWER_DEBUG_CLOCKS))
			return;

	MSM_POWER_DPRINTK(MSM_POWER_DEBUG_CLOCKS, KERN_INFO, "%s", msg);

 
	for(i=1; i < NR_CLKS; i++)
	{
		id = i;
		if (msm_proc_comm(PCOM_CLKCTL_RPC_ENABLED, &id, NULL)) {
			MSM_POWER_DPRINTK(MSM_POWER_DEBUG_CLOCKS, KERN_INFO,
				"%s: N/A\n", clock_names[i]);
		}
		else {
			if(id)
				MSM_POWER_DPRINTK(MSM_POWER_DEBUG_CLOCKS, KERN_INFO,
					"  %s\n", clock_names[i]);
		}
	}		
}

void get_enabled_vregs(char *msg)
{
	int i;
	unsigned id_status;
	uint32_t cmd_status = PCOMM_PALM_DIAG_VREG_GET_STATUS;
	struct vreg *vreg;

	if(!(msm_power_debug_mask & MSM_POWER_DEBUG_VREGS))
			return;

	MSM_POWER_DPRINTK(MSM_POWER_DEBUG_VREGS, KERN_INFO, "%s", msg);

	for(i=0; i < ARRAY_SIZE(vreg_names); i++)
	{
		id_status = i;
		if(msm_proc_comm(PCOM_PALM_DIAG_CMDS, &cmd_status, &id_status)) {
			MSM_POWER_DPRINTK(MSM_POWER_DEBUG_VREGS, KERN_INFO, 
				"%s: N/A\n", vreg_names[i]);
		}
		else {
			// only print if the VREG is enabled
			if(id_status) {
				vreg = vreg_get(0, vreg_names[i]);
				MSM_POWER_DPRINTK(MSM_POWER_DEBUG_VREGS, KERN_INFO,
					"  %s count=%d\n", vreg_names[i], vreg_refcount(vreg));
			}
		}
	}
}

static ssize_t vregs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int save_mask = msm_power_debug_mask;
	msm_power_debug_mask = 0xF;

	get_enabled_vregs("Dumping VRegs");

	msm_power_debug_mask = save_mask;

	return 0;
}

DEVICE_ATTR(vregs, S_IRUGO, vregs_show, NULL);

static ssize_t clocks_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int save_mask = msm_power_debug_mask;
	msm_power_debug_mask = 0xF;

	get_enabled_clocks("Dumping Clocks");

	msm_power_debug_mask = save_mask;

	return 0;
}

DEVICE_ATTR(clocks, S_IRUGO, clocks_show, NULL);

void dump_arm11_raw_div(void)
{
	uint32_t clk;

	clk = readl(ARM11_RAW_CLK_DIV_REG);
	printk("ARM11_RAW_CLK_DIV_REG(0x%x) = 0x%x\n", ARM11_RAW_CLK_DIV_REG, clk);
	printk("\tPLL2_BIST_DIV_SEL(5:4)= %d\n", ((clk & 0x00000030) >> 4) + 1);
	printk("\tPLL1_BIST_DIV_SEL(3:2)= %d\n", ((clk & 0x0000000C) >> 2) + 1);
	printk("\tPLL0_BIST_DIV_SEL(1:0)= %d\n", ((clk & 0x00000003) + 1));
}

void dump_pll_clk(void)
{
	uint32_t clk;

	clk = readl(PLL0_L_VAL);
	printk("PLL0_L_VAL: %d\n", clk & 0x3F);

	clk = readl(PLL1_L_VAL);
	printk("PLL1_L_VAL: %d\n", clk & 0x3F);
	
	clk = readl(PLL2_L_VAL);
	printk("PLL2_L_VAL: %d\n", clk & 0x3F);
}

void dump_pmdh_clk(void)
{
	uint32_t clk;

	clk = readl(PMDH_CLK_ADDR);
	
	printk("PMDH Clock Speed from Modem = %u\n", pc_clk_get_rate(PMDH_CLK)); 
	printk("PMDH_NS_REG(0x%x) = 0x%x\n",PMDH_CLK_ADDR, clk);
	printk("\tPMDH_HW_GATE_ENA(12)= %s\n", (clk & 0x00001000) ? "Enabled" : "Not Enabled");
	printk("\tPMDH_ROOT_ENA(11)= %s\n", (clk & 0x00000800) ? "Enabled" : "Not Enabled");
	printk("\tPMDH_CLK_INV(10)= %s\n", (clk & 0x00000400) ? "Inverted" : "Not Inverted");
	printk("\tPMDH_CLK_BRANCH_ENA(9)= %s\n", (clk & 0x00000200) ? "Enabled" : "Not Enabled");
	printk("\tSRC_DIV(6:3)= %d\n", ((clk & 0x00000078) >> 3) + 1);
	printk("\tSRC_SEL(2:0)= ");

	switch (clk & 0x00000007)
	{
		case 0:
			printk("TCXO\n");
			break;
		
		case 1:
			printk("Global PLL\n");
			break;
		case 2:
			printk("Backup PLL0\n");
			break;
		case 3:
			printk("Backup PLL1\n");
			break;
		case 4:
			printk("Modem PLL\n");
			break;
		case 5:
			printk("CLK Test Core In\n");
			break;
		case 6:
			printk("Sleep CLK\n");
			break;
		case 7:
			printk("PLL Test Core In\n");
			break;
		default:
			printk("Unknown\n");
			break;
	}	
}


void dump_ebi1_clk(void)
{
	uint32_t clk;

	clk = readl(EBI1_CLK_ADDR);
	

	printk("EBI1 Clock Speed from Modem = %u\n", pc_clk_get_rate(EBI1_CLK)); 
	printk("EBI1_NS_REG(0x%x)= 0x%x\n", EBI1_CLK_ADDR, clk);
	printk("\tCLK_SEL(14)= %s\n", (clk & 0x00004000) ? "EBI1" : "Global");
	printk("\tEBI1_CLK_DIV(13:12)= %d\n", ((clk & 0x00003000) >> 12) + 1);
	printk("\tEBI1_ROOT_ENA(11)= %s\n", (clk & 0x00000800) ? "Enabled" : "Not Enabled");
	printk("\tEBI1_CLK_INV(10)= %s\n", (clk & 0x00000400) ? "Inverted" : "Not Inverted");
	printk("\tEBI1_CLK_BRANCH_ENA(9)= %s\n", (clk & 0x00000200) ? "Enabled" : "Not Enabled");
	printk("\tEBI1_2X_CLK_INV(8)= %s\n", (clk & 0x00000100) ? "Inverted" : "Not Inverted");
	printk("\tEBI1_2X_CLK_BRANCH_ENA(7)= %s\n", (clk & 0x00000080) ? "Enabled" : "Not Enabled");
	printk("\tSRC_DIV(6:3)= %d\n", ((clk & 0x00000078) >> 3) + 1);
	printk("\tSRC_SEL(2:0)= ");
	switch (clk & 0x00000007)
	{
		case 0:
			printk("TCXO\n");
			break;
		case 1:
			printk("Global PLL\n");
			break;
		case 2:
			printk("Backup PLL0\n");
			break;
		case 3:
			printk("Backup PLL1\n");
			break;
		case 4:
			printk("Modem PLL\n");
			break;
		case 5:
			printk("CLK Test Core In\n");
			break;
		case 6:
			printk("Sleep CLK\n");
			break;
		case 7:
			printk("PLL Test Core In\n");
			break;
		default:
			printk("Unknown\n");
			break;
	}	
	

}

void dump_a11s_vdd_level(void)
{
	uint32_t vdd;

	vdd = readl(A11S_VDD_SVS_PLEVEL_ADDR);

	printk("A11S_VDD_SVS_PLEVEL(0x%x)= 0x%x\n", A11S_VDD_SVS_PLEVEL_ADDR, vdd);
	printk("\tSVS_CTL_STATUS(2:0)= %d\n", vdd & 0x07);
}

void dump_clk_sel(void)
{
	uint32_t clk_sel;

	clk_sel = readl(A11S_CLK_SEL_ADDR);
	printk("A11S_CLK_SEL_ADDR(0x%x)= 0x%x\n", A11S_CLK_SEL_ADDR, clk_sel);
	printk("\tABH_CLK_DIV(2:1)= %d\n", ((clk_sel & 0x00000006) >> 1) + 1);
	printk("\tACPU CLOCK SOURCE(0)= %s\n", clk_sel & 0x00000001 ? "SRC1" : "SRC0");
}	

void dump_clk_cntl(void)
{
	uint32_t clk_cntl;

	clk_cntl = readl(A11S_CLK_CNTL_ADDR);
	printk("A11S_CLK_CNTL_ADDR(0x%x)=0x%x\n", A11S_CLK_CNTL_ADDR, clk_cntl);
	printk("\tWT_ST_CNT(31:16)= %d\n", (clk_cntl >> 16));
	
	printk("\tCLK_SRC0_SEL(14:12)= ");
	switch ((clk_cntl & 0x00007000) >> 12)
	{
		case 0:
			printk("TCXO\n");
			break;
		case 1:
			printk("Global PLL\n");
			break;
		case 2:
			printk("Backup PLL0\n");
			break;
		case 3:
			printk("Backup PLL1\n");
			break;
		case 4:
			printk("Modem PLL\n");
			break;
		case 5:
			printk("PLL Test\n");
			break;
		case 6:
			printk("USB\n");
			break;
		case 7:
			printk("Sleep CLK\n");
			break;
		default:
			printk("Unknown\n");
			break;
	}

	printk("\tCLK_SRC0_DIV(11:8)= %d\n", ((clk_cntl & 0x00000F00) >> 8) + 1);

	printk("\tCLK_SRC1_SEL(6:4)= ");
	switch ((clk_cntl & 0x00000070) >> 4)
	{
		case 0:
			printk("TCXO\n");
			break;
		case 1:
			printk("Global PLL\n");
			break;
		case 2:
			printk("Backup PLL0\n");
			break;
		case 3:
			printk("Backup PLL1\n");
			break;
		case 4:
			printk("Modem PLL\n");
			break;
		case 5:
			printk("PLL Test\n");
			break;
		case 6:
			printk("USB\n");
			break;
		case 7:
			printk("Sleep CLK\n");
			break;
		default:
			printk("Unknown\n");
			break;
	}

	printk("\tCLK_SRC1_DIV(3:0)= %d\n", (clk_cntl & 0x0000000F) + 1);
}

static ssize_t 
msm_clock_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	printk("-----------------------------\n");
	printk("Dumping Clock Registers\n");
	printk("-----------------------------\n");


	dump_pll_clk();
	dump_arm11_raw_div();
	dump_clk_cntl();
	dump_clk_sel();
	dump_a11s_vdd_level();
	dump_ebi1_clk();
	dump_pmdh_clk();

	return len;
}	

static DEVICE_ATTR(a11_dump, S_IWUGO, NULL, msm_clock_dump);


static int __init msm_power_debug_probe(struct platform_device *pdev)
{
	int ret = 0;

	if ((ret = device_create_file(&(pdev->dev), &dev_attr_a11_dump)))
	{
		printk(KERN_ERR "Failed to create sysfs device for msm_clock_debug\n");
		
		return -EINVAL;
	}

	if ((ret = device_create_file(&(pdev->dev), &dev_attr_vregs)))
	{
		printk(KERN_ERR "Failed to create sysfs device for msm_clock_debug\n");
		
		return -EINVAL;
	}

	if ((ret = device_create_file(&(pdev->dev), &dev_attr_clocks)))
	{
		printk(KERN_ERR "Failed to create sysfs device for msm_clock_debug\n");
		
		return -EINVAL;
	}		

	return 0;
}


static struct platform_driver msm_power_debug_driver = {
	.probe = msm_power_debug_probe,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_power_debug_init(void)
{
	printk("msm_power_debug_init()\n");

	if(platform_driver_register(&msm_power_debug_driver) != 0)
	{
		printk("CLOCK_DEBUG:Failed to init power debug driver\n");
	}

	return 0;
}

module_init(msm_power_debug_init);

MODULE_DESCRIPTION("MSM POWER DEBUG");
MODULE_AUTHOR("Kevin McCray <kevin.mccray@palm.com>");
MODULE_LICENSE("GPL");

