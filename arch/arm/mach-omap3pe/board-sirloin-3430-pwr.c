/*
 * linux/arch/arm/mach-omap3pe/board-sirloin-3430-pwr.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include <asm/hardware.h>
#include <asm/arch/board.h>
#include <asm/arch/gpio.h>
#include <asm/arch/twl4030.h>

#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)
#endif

/* twl4030 specific definitions */
#define VSEL_S2_CLR         0x40

#define PULLTYPESELECT1_MASK	(1 << 20)

static struct mutex pwr_mutex;
static int vcc_mmc1_ref_cnt = 0;
static int vcc_aux1_ref_cnt = 0;
static int vcc_aux2_ref_cnt = 0;
static int vcc_aux3_ref_cnt = 0;
static int vcc_aux4_ref_cnt = 0;
static int vcc_pll2_ref_cnt = 0;
static int vcc_cam_ref_cnt = 0;
static int vcc_lgt_ref_cnt = 0;

static void twl4030_unlock_pm_master(void)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x0E, TWL4030_PROTECT_KEY);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0xE0, TWL4030_PROTECT_KEY);
}

static void twl4030_lock_pm_master(void)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_MASTER, 0x00, TWL4030_PROTECT_KEY);
}

/*
 *  Disable specified ldo
 */
static int twl4030_ldo_disable(int grp, int ldo)
{
	int rc1, rc2;

	rc1 = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, TWL_DEV_GROUP_NONE, grp);
	twl4030_unlock_pm_master();
	rc2 = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, VSEL_S2_CLR, ldo);
	twl4030_lock_pm_master();

	if (rc1 || rc2) {
		printk(KERN_ERR "Failed (%d, %d) to disable ldo 0x%x\n",
		       rc1, rc2, ldo);
		return -1;
	}
	return 0;
}

/*
 *  Enable specified ldo
 */
static int twl4030_ldo_enable(int grp, int ldo, int vsel)
{
	int rc1, rc2;

	/* Enable ldo and power up */
	twl4030_unlock_pm_master();
	rc1 = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, TWL_DEV_GROUP_P3, grp);
	rc2 = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, vsel, ldo);
	twl4030_lock_pm_master();
	if (rc1 || rc2) {
		printk(KERN_ERR "Failed (%d, %d) for enable ldo 0x%x\n",
		       rc1, rc2, ldo);
		return -1;
	}
	return 0;
}

/*
 *  VMMC1 power supply
 */
static int twl4030_vmmc1_disable(void)
{
	return twl4030_ldo_disable(TWL4030_VMMC1_DEV_GRP, TWL4030_VMMC1_DEDICATED);
}

static int twl4030_vmmc1_30_enable(void)
{
	return twl4030_ldo_enable(TWL4030_VMMC1_DEV_GRP,
				  TWL4030_VMMC1_DEDICATED,
				  TWL_VMMC1_3P00);
}

/*
 *  VMMC2 power supply
 */
static int twl4030_vmmc2_disable(void)
{
	return twl4030_ldo_disable(TWL4030_VMMC2_DEV_GRP, TWL4030_VMMC2_DEDICATED);
}

/*
 *  VAUX1 power supply
 */
static int twl4030_vaux1_disable(void)
{
	return twl4030_ldo_disable(TWL4030_VAUX1_DEV_GRP, TWL4030_VAUX1_DEDICATED);
}

static int twl4030_vaux1_30_enable(void)
{
	return twl4030_ldo_enable(TWL4030_VAUX1_DEV_GRP,
				  TWL4030_VAUX1_DEDICATED,
				  TWL_VAUX1_3P00);
}

/*
 *  VAUX2 power supply
 */
static int twl4030_vaux2_disable(void)
{
	return twl4030_ldo_disable(TWL4030_VAUX2_DEV_GRP, TWL4030_VAUX2_DEDICATED);
}

static int twl4030_vaux2_30_enable(void)
{
	return twl4030_ldo_enable(TWL4030_VAUX2_DEV_GRP,
				  TWL4030_VAUX2_DEDICATED,
	                          TWL_VAUX2_2P80);
}

/*
 *  VAUX3 power supply
 */
static int twl4030_vaux3_disable(void)
{
	return twl4030_ldo_disable(TWL4030_VAUX3_DEV_GRP, TWL4030_VAUX3_DEDICATED);
}

static int twl4030_vaux3_18_enable(void)
{
	return twl4030_ldo_enable(TWL4030_VAUX3_DEV_GRP,
				  TWL4030_VAUX3_DEDICATED,
				  TWL_VAUX3_1P80);
}

/*
 *  VAUX4 power supply
 */
static int twl4030_vaux4_disable(void)
{
	return twl4030_ldo_disable(TWL4030_VAUX4_DEV_GRP, TWL4030_VAUX4_DEDICATED);
}

static int twl4030_vaux4_30_enable(void)
{
	return twl4030_ldo_enable(TWL4030_VAUX4_DEV_GRP,
				  TWL4030_VAUX4_DEDICATED,
				  TWL_VAUX4_2P80);
}

/*
 *  VPLL2 power supply
 */
static int twl4030_vpll2_disable(void)
{
	return twl4030_ldo_disable(TWL4030_VPLL2_DEV_GRP, TWL4030_VPLL2_DEDICATED);
}

static int twl4030_vpll2_18_enable(void)
{
	return twl4030_ldo_enable(TWL4030_VPLL2_DEV_GRP,
				  TWL4030_VPLL2_DEDICATED,
				  TWL_VPLL2_1P80);
}

/************************************************************
 *
 *   Exported (by resource)s
 *
 ************************************************************/

/*
 *  Enable 3.0V out of MMC1 power supply
 */
int board_mmc1_vcc_30_enable(void)
{
	int rc = 0;

	mutex_lock(&pwr_mutex);
	vcc_mmc1_ref_cnt++;
	if (vcc_mmc1_ref_cnt == 1) {	// enable power supply
		rc = twl4030_vmmc1_30_enable();
		if (rc) {	// failed for some reason
			twl4030_vmmc1_disable();
			vcc_mmc1_ref_cnt--;
		}
	}
	mutex_unlock(&pwr_mutex);
	return rc;
}

/*
 *  Disable MMC1 power supply
 */
int board_mmc1_vcc_disable(void)
{
	mutex_lock(&pwr_mutex);
	vcc_mmc1_ref_cnt--;
	if (vcc_mmc1_ref_cnt <= 0) {	// we can shut it off
		(void)twl4030_vmmc1_disable();
		vcc_mmc1_ref_cnt = 0;
	}
	mutex_unlock(&pwr_mutex);
	return 0;
}

EXPORT_SYMBOL(board_mmc1_vcc_30_enable);
EXPORT_SYMBOL(board_mmc1_vcc_disable);

/*
 *  Enable 3.0V MMC2 power supply
 */
int board_mmc2_vcc_30_enable(void)
{
	printk(KERN_ERR "VMMC2 is not connected and should not be used\n");
	return -EINVAL;
}

/*
 *  Disable MMC2 power supply
 */
int board_mmc2_vcc_disable(void)
{
	(void)twl4030_vmmc2_disable();
	return 0;
}

EXPORT_SYMBOL(board_mmc2_vcc_30_enable);
EXPORT_SYMBOL(board_mmc2_vcc_disable);

/*
 *  Enable 3.0V out of VAUX1 power supply
 */
int board_aux1_vcc_30_enable(void)
{
	int rc = 0;

	mutex_lock(&pwr_mutex);
	vcc_aux1_ref_cnt++;
	if (vcc_aux1_ref_cnt == 1) {	// enable power supply
		rc = twl4030_vaux1_30_enable();
		if (rc) {	// failed for some reason
			twl4030_vaux1_disable();
			vcc_aux1_ref_cnt--;
		}
	}
	mutex_unlock(&pwr_mutex);
	return rc;
}

/*
 *   Disable VAUX1 power supply
 */
int board_aux1_vcc_disable(void)
{
	mutex_lock(&pwr_mutex);
	vcc_aux1_ref_cnt--;
	if (vcc_aux1_ref_cnt <= 0) {	// we can shut it off
		(void)twl4030_vaux1_disable();
		vcc_aux1_ref_cnt = 0;
	}
	mutex_unlock(&pwr_mutex);
	return 0;
}

EXPORT_SYMBOL(board_aux1_vcc_30_enable);
EXPORT_SYMBOL(board_aux1_vcc_disable);

/*
 *  Enable 3.0V out of VAUX2 power supply
 */
int board_aux2_vcc_30_enable(void)
{
	int rc = 0;

	mutex_lock(&pwr_mutex);
	vcc_aux2_ref_cnt++;
	if (vcc_aux2_ref_cnt == 1) {	// enable power supply
//		printk(KERN_DEBUG "%s: before calling twl4030_vaux2_30_enable", __func__);
		rc = twl4030_vaux2_30_enable();
		if (rc) {	// failed for some reason
			twl4030_vaux2_disable();
			vcc_aux2_ref_cnt--;
		}
	}
	mutex_unlock(&pwr_mutex);
	return rc;
}

/*
 *   Disable VAUX2 power supply
 */
int board_aux2_vcc_disable(void)
{
	mutex_lock(&pwr_mutex);
	vcc_aux2_ref_cnt--;
	if (vcc_aux2_ref_cnt <= 0) {	// we can shut it off
		(void)twl4030_vaux2_disable();
		vcc_aux2_ref_cnt = 0;
	}
	mutex_unlock(&pwr_mutex);
	return 0;
}

EXPORT_SYMBOL(board_aux2_vcc_30_enable);
EXPORT_SYMBOL(board_aux2_vcc_disable);

/*
 *  Enable 1.8V out of VAUX3 power supply
 */
int board_aux3_vcc_18_enable(void)
{
	int rc = 0;

	mutex_lock(&pwr_mutex);
	vcc_aux3_ref_cnt++;
	if (vcc_aux3_ref_cnt == 1) {	// enable power supply
		rc = twl4030_vaux3_18_enable();
		if (rc) {	// failed for some reason
			twl4030_vaux3_disable();
			vcc_aux3_ref_cnt--;
		}
	}
	mutex_unlock(&pwr_mutex);
	return rc;
}

/*
 *   Disable VAUX4 power supply
 */
int board_aux3_vcc_disable(void)
{
	mutex_lock(&pwr_mutex);
	vcc_aux3_ref_cnt--;
	if (vcc_aux3_ref_cnt <= 0) {	// we can shut it off
		(void)twl4030_vaux3_disable();
		vcc_aux3_ref_cnt = 0;
	}
	mutex_unlock(&pwr_mutex);
	return 0;
}

EXPORT_SYMBOL(board_aux3_vcc_18_enable);
EXPORT_SYMBOL(board_aux3_vcc_disable);

/*
 *  Enable 3.0V out of VAUX4 power supply
 */
int board_aux4_vcc_30_enable(void)
{
	int rc = 0;

	mutex_lock(&pwr_mutex);
	vcc_aux4_ref_cnt++;
	if (vcc_aux4_ref_cnt == 1) {	// enable power supply
		rc = twl4030_vaux4_30_enable();
		if (rc) {	// failed for some reason
			twl4030_vaux4_disable();
			vcc_aux4_ref_cnt--;
		}
	}
	mutex_unlock(&pwr_mutex);
	return rc;
}

/*
 *   Disable VAUX4 power supply
 */
int board_aux4_vcc_disable(void)
{
	mutex_lock(&pwr_mutex);
	vcc_aux4_ref_cnt--;
	if (vcc_aux4_ref_cnt <= 0) {	// we can shut it off
		(void)twl4030_vaux4_disable();
		vcc_aux4_ref_cnt = 0;
	}
	mutex_unlock(&pwr_mutex);
	return 0;
}

EXPORT_SYMBOL(board_aux4_vcc_30_enable);
EXPORT_SYMBOL(board_aux4_vcc_disable);

/*
 *  Enable 1.8V out of VPLL2 power supply
 */
static int board_pll2_vcc_18_enable_locked(void)
{
	int rc = 0;

	vcc_pll2_ref_cnt++;
	if (vcc_pll2_ref_cnt == 1) {
		rc = twl4030_vpll2_18_enable();
		if (rc) {
			twl4030_vpll2_disable();
			vcc_pll2_ref_cnt--;
		}
	}
	
	return rc;
}

int board_pll2_vcc_18_enable(void)
{
	int rc = 0;

	mutex_lock(&pwr_mutex);
	rc = board_pll2_vcc_18_enable_locked();
	mutex_unlock(&pwr_mutex);
	
	return rc;
}

/*
 *   Disable VPLL2 power supply
 */
static void board_pll2_vcc_disable_locked(void)
{
	vcc_pll2_ref_cnt--;
	if (vcc_pll2_ref_cnt <= 0) {
		(void)twl4030_vpll2_disable();
		vcc_pll2_ref_cnt = 0;
	}
}

int board_pll2_vcc_disable(void)
{
	mutex_lock(&pwr_mutex);
	board_pll2_vcc_disable_locked();
	mutex_unlock(&pwr_mutex);
	return 0;
}

EXPORT_SYMBOL(board_pll2_vcc_18_enable);
EXPORT_SYMBOL(board_pll2_vcc_disable);

/************************************************************
 *
 *   Exported (By functional area)
 *
 ************************************************************/

/*
 *  Enable/Disable BT VCC. 
 *  
 *  BT-3.0-VCC is powerd by twl4030 aux1 power supply
 */
int board_bt_vcc_enable(int enable)
{
	if (enable)
		return board_aux1_vcc_30_enable();
	else
		return board_aux1_vcc_disable();
}

EXPORT_SYMBOL(board_bt_vcc_enable);

/*
 *  Enable/disable WL VCC
 *
 *  WL-3.0-VCC is powerd by twl4030 aux1 power supply
 */
int board_wl_vcc_enable(int enable)
{
	if (enable)
		return board_aux1_vcc_30_enable();
	else
		return board_aux1_vcc_disable();
}

EXPORT_SYMBOL(board_wl_vcc_enable);

/*
 *  Enable/Disable WL-RF-VCC
 *
 *  WL-1.8V-RF is powered by twl4030 aux3 power supply
 */
int board_wl_rf_vcc_enable(int enable)
{
	if (enable)
		return board_aux3_vcc_18_enable();
	else
		return board_aux3_vcc_disable();
	return 0;
}

EXPORT_SYMBOL(board_wl_rf_vcc_enable);

/*
 *  Enable/Disable  3.0V IO VCC
 *
 *  VCC-3.0-IO is powered by vmmc1 power supply
 *
 */
int board_io_vcc_enable(int enable)
{
	if (enable)
		return board_mmc1_vcc_30_enable();
	else
		return board_mmc1_vcc_disable();
}

EXPORT_SYMBOL(board_io_vcc_enable);

/*
 *  Enable/Disable CAM VCC
 *
 */

#define CAM_ENABLE_GPIO  101
 
static void cam_vcc_enable(void)
{
	if (++vcc_cam_ref_cnt == 1) {
		/* evt0: enable pull-up on gpio 99 */
		CONTROL_PADCONF_CAM_FLD |= PULLTYPESELECT1_MASK;

		gpio_set_value( CAM_ENABLE_GPIO, 1 );

		/* 
		 * regulator ramp time: < 600us
		 * oscillator start time: 10ms
		 */
		msleep(10);
	}
}

static void cam_vcc_disable(void)
{
	if (--vcc_cam_ref_cnt <= 0) {
		/* evt0: disable pull-up on gpio 99 */
		CONTROL_PADCONF_CAM_FLD &= ~PULLTYPESELECT1_MASK;

		gpio_set_value( CAM_ENABLE_GPIO, 0 );
		vcc_cam_ref_cnt = 0;

		/* capacitor discharge: ??? */
		msleep(1);
	}
}

int board_cam_vcc_enable(int enable)
{
	mutex_lock(&pwr_mutex);
	if (enable)
		cam_vcc_enable();
	else
		cam_vcc_disable();
	mutex_unlock(&pwr_mutex);
	return 0;
}
EXPORT_SYMBOL(board_cam_vcc_enable);

int board_sdi_vcc_enable(int enable)
{
	if (enable)
		return board_pll2_vcc_18_enable();
	else
		return board_pll2_vcc_disable();
}
EXPORT_SYMBOL(board_sdi_vcc_enable);

/************************************************************
 *
 *  Enable/Disable Light Senor/Prox Sensor VCC. 
 *
 *  NOTE:
 *    Starting with EVT3s GPIO 14 is only used for the
 *    light sensor.
 *
 ************************************************************/

#define LTG_ENABLE_GPIO  14

static void lgt_vcc_enable(void)
{
	if (++vcc_lgt_ref_cnt == 1) {
		gpio_set_value(LTG_ENABLE_GPIO, 1);
	}
}

static void lgt_vcc_disable(void)
{
	if (--vcc_lgt_ref_cnt <= 0) {
		gpio_set_value(LTG_ENABLE_GPIO, 0);
		vcc_lgt_ref_cnt = 0;
	}
}

void board_lgt_vcc_enable(int enable)
{
	mutex_lock(&pwr_mutex);
	if (enable)
		lgt_vcc_enable();
	else
		lgt_vcc_disable();
	mutex_unlock(&pwr_mutex);
}

EXPORT_SYMBOL(board_lgt_vcc_enable);

void board_prox_vcc18_enable(void)
{
	if (board_type <= EVT2)
		board_lgt_vcc_enable(1);
}

EXPORT_SYMBOL(board_prox_vcc18_enable);

void board_prox_vcc18_disable(void)
{
	if (board_type <= EVT2)
		board_lgt_vcc_enable(0);
}

EXPORT_SYMBOL(board_prox_vcc18_disable);

/*
 *  sirloin init gpio
 */
static void __init gpio_enable_init( void )
{
	int ret;
	
	/* Setup GPIO for Light Sensor */ 
	ret = gpio_request(LTG_ENABLE_GPIO, NULL);
	if (ret)
		goto err1;

	/* configure as output and set low */
	gpio_direction_output (LTG_ENABLE_GPIO, 0);

	/* Setup GPIO for CAM Sensor */ 
	ret = gpio_request(CAM_ENABLE_GPIO, NULL);
	if (ret)
		goto err2;

	/* configure as output and set low */
	gpio_direction_output (CAM_ENABLE_GPIO, 0);

	return;

err2:
	gpio_free(CAM_ENABLE_GPIO);
err1:
	return;
}

/*
 *  sirloin exit gpio
 */
static void __exit gpio_disable_exit( void )
{
	/* Free GPIO for Light Sensor & Proximity Sensor */ 
	gpio_set_value(LTG_ENABLE_GPIO, 0);
	gpio_free(LTG_ENABLE_GPIO);
        vcc_lgt_ref_cnt = 0;

	/* Free GPIO for CAM Sensor */ 
	gpio_set_value(CAM_ENABLE_GPIO, 0);
	gpio_free(CAM_ENABLE_GPIO);
	vcc_cam_ref_cnt = 0;
}


static int __init board_pwr_init(void)
{
	printk(KERN_INFO "Initialize power support module\n");

	mutex_init(&pwr_mutex);
	gpio_enable_init();
	
	return 0;
}

static void __exit board_pwr_exit(void)
{
	printk(KERN_INFO "Exit power support module\n");
	gpio_disable_exit();
}

module_init(board_pwr_init);
module_exit(board_pwr_exit);
