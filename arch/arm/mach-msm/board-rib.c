/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/gpio_keys_pe.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/hsuart.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/i2c/isa1200.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/i2c/tsc2007.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pmic8058.h>
#include <linux/input/cy8c_ts.h>
#include <linux/msm_adc.h>
#include <linux/clk.h>


#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <linux/cy8ctma300.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <linux/cyttsp.h>

#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include <mach/clk.h>
#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include "pm.h"
#include "spm.h"
#include <linux/msm_kgsl.h>
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_reqs.h>

#include <linux/pinmux.h>

#include "smd_private.h"

#ifdef CONFIG_LEDS_LM3528
#include <linux/i2c_lm3528_led.h>
#endif

#ifdef CONFIG_LEDS_LM8502
#include <linux/i2c_lm8502_led.h>
#endif

#ifdef CONFIG_CHARGER_SMB339
#include <linux/i2c_smb339_charger.h>
#endif

#ifdef CONFIG_HRES_COUNTER
#include <linux/hres_counter.h>
#endif

#ifdef CONFIG_A6
#include <linux/a6_sbw_interface.h>
#include <linux/a6.h>
#include "a6_sbw_impl_rib.h"
#endif

#ifdef CONFIG_BLUETOOTH_POWER_STATE
#include <linux/bluetooth-power-pe.h>
#endif

#ifdef CONFIG_USER_PINS
#include <linux/user-pins.h>
#endif

#ifdef CONFIG_MSM_VIBRATOR
#include <linux/vibrator.h>
#endif

#ifdef CONFIG_NDUID
#include <linux/nduid.h>
#endif

#include "board-rib-gpios.h"

#ifdef CONFIG_MT9P013
#include <linux/mt9p013.h>
#endif

#ifdef CONFIG_MFD_WM8994
#include <linux/mfd/wm8994/pdata.h>
#include <linux/mfd/wm8994/core.h>
#endif

enum rib_board_types {
        RIB_PROTO = 0,
        RIB_EVT1,
        RIB_EMU2,
        RIB_EVT2,
        RIB_EVT3,
        RIB_DVT,
        RIB_PVT,
};

enum wlan_reset_pin_owner {
	WLAN_NONE      = 0,
	WLAN_BT        = 1,
	WLAN_WIFI      = 1<<1,
};



#define MSM_PMEM_SF_SIZE	0x1700000

/*
 * Frame Buffer Size:
 *
 * FB0:
 * Each buffer is 480x800x32bpp=1500K.
 * We have three of these, so total: 4.5M
 *
 * FB1:
 * Same as FB1, but with 64k extra for
 * alignment issues with GPU and scaling
 *
 */
#define MSM_FB0_SIZE          0x480000
#define MSM_FB1_SIZE          0x490000

#define MSM_GPU_PHYS_SIZE       SZ_2M
#define MSM_PMEM_GPU1_SIZE      0x1000000

#define MSM_PMEM_ADSP_SIZE       0x2D00000
#define MSM_FLUID_PMEM_ADSP_SIZE 0x2B00000

#define PMEM_KERNEL_EBI1_SIZE   0x600000
#define MSM_PMEM_AUDIO_SIZE     0x200000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900
#define PMIC_GPIO_SD_DET	36
#define PMIC_GPIO_SDC4_EN	17  /* PMIC GPIO Number 18 */
#define PMIC_GPIO_HDMI_5V_EN	39  /* PMIC GPIO Number 40 */

#define FPGA_SDCC_STATUS       0x8E0001A8

#define FPGA_OPTNAV_GPIO_ADDR	0x8E000026
#define OPTNAV_I2C_SLAVE_ADDR	(0xB0 >> 1)
#define OPTNAV_IRQ		20
#define OPTNAV_CHIP_SELECT	19

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define PMIC_GPIO_HAP_ENABLE   16  /* PMIC GPIO Number 17 */

#define HAP_LVL_SHFT_MSM_GPIO 24

#define	PM_FLIP_MPP 5 /* PMIC MPP 06 */

#define PM8058_SUBDEV_KPD       0



static int pm8058_gpios_init(void)
{
	int rc;
        struct pm8058_gpio pwm_gpio = {
                .direction      = PM_GPIO_DIR_OUT,
                .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
                .output_value   = 1,
                .pull           = PM_GPIO_PULL_NO,
                .out_strength   = PM_GPIO_STRENGTH_HIGH,
                .function       = PM_GPIO_FUNC_2,
        };

	if (machine_is_rib()) {
		rc = pm8058_gpio_config(24, &pwm_gpio); /* pmic gpio 25 */
		if (rc) {
			pr_err("%s PMIC GPIO 25 write failed\n", __func__);
			return rc;
		}
	}

	return 0;
}

/* Key proximity map */
static u8  board_key_prox_map[][6] = {
        {  1,  3, 38,         0xFF }, // i  [ 0]
        {  2,  4,  0,         0xFF }, // o  [ 1]
        {  5,  1,             0xFF }, // p  [ 2]
        {  0,  4, 13, 30,     0xFF }, // k  [ 3]
        {  1,  5, 14,  3,     0xFF }, // l  [ 4]
        {  2,  6,  4,         0xFF }, // Bs [ 5]
        {  5, 14,             0xFF }, // En [ 6]
        {                     0xFF }, //    [ 7]
        { 24, 17,             0xFF }, // rA [ 8]
        { 18, 10, 16,         0xFF }, // 0  [ 9]
        { 20, 21, 11,  9,     0xFF }, // sp [10]
        { 22, 13, 12, 10,     0xFF }, // .  [11]
        { 14, 11,             0xFF }, // opt[12]
        {  3, 14, 12, 11, 22, 0xFF }, // m  [13]
        {  4,  6, 12, 13,     0xFF }, // ,  [14]
        {                     0xFF }, //    [15]
        { 17, 18,  9,         0xFF }, // Ls [16]
        { 25, 18, 16,  8,     0xFF }, // z  [17]
        { 26, 19,  9, 16, 17, 0xFF }, // x  [18]
        { 27, 37, 10,  9, 18, 0xFF }, // c  [19]
        { 28, 21, 10, 19,     0xFF }, // v  [20]
        { 29, 22, 10, 37,     0xFF }, // b  [21]
        { 21, 30, 13, 11, 10, 0xFF }, // n  [22]
        {                     0xFF }, //    [23]
        { 32, 25,  8,         0xFF }, // a  [24]
        { 33, 26, 17, 24,     0xFF }, // s  [25]
        { 34, 27, 18, 25,     0xFF }, // d  [26]
        { 35, 28, 19, 26,     0xFF }, // f  [27]
        { 36, 29, 20, 27,     0xFF }, // g  [28]
        { 37, 30, 21, 28,     0xFF }, // h  [29]
        { 38,  3, 22, 29,     0xFF }, // j  [30]
        {                     0xFF }, //    [31]
        { 33, 24,             0xFF }, // q  [32]
        { 34, 25, 32,         0xFF }, // w  [33]
        { 35, 26, 33,         0xFF }, // e  [34]
        { 36, 27, 34,         0xFF }, // r  [35]
        { 37, 28, 35,         0xFF }, // t  [36]
        { 38, 29, 36,         0xFF }, // y  [37]
        {  0, 30, 37,         0xFF }, // u  [38]
};

static const unsigned int rib_keymap[] = {
        KEY(4, 0, KEY_Q),
        KEY(4, 1, KEY_W),
        KEY(4, 2, KEY_E),
        KEY(4, 3, KEY_R),
        KEY(4, 4, KEY_T),
        KEY(4, 5, KEY_Y),
        KEY(4, 6, KEY_U),

        KEY(3, 0, KEY_A),
        KEY(3, 1, KEY_S),
        KEY(3, 2, KEY_D),
        KEY(3, 3, KEY_F),
        KEY(3, 4, KEY_G),
        KEY(3, 5, KEY_H),
        KEY(3, 6, KEY_J),

        KEY(2, 0, KEY_LEFTSHIFT),
        KEY(2, 1, KEY_Z),
        KEY(2, 2, KEY_X),
        KEY(2, 3, KEY_C),
        KEY(2, 4, KEY_V),
        KEY(2, 5, KEY_B),
        KEY(2, 6, KEY_N),

        KEY(1, 0, KEY_RIGHTALT),
        KEY(1, 1, KEY_0),
        KEY(1, 2, KEY_SPACE),
        KEY(1, 3, KEY_DOT),
        KEY(1, 4, KEY_ALT),
        KEY(1, 5, KEY_M),
        KEY(1, 6, KEY_COMMA),
        KEY(0, 0, KEY_I),
        KEY(0, 1, KEY_O),
        KEY(0, 2, KEY_P),
        KEY(0, 3, KEY_K),
        KEY(0, 4, KEY_L),
        KEY(0, 5, KEY_BACKSPACE),
        KEY(0, 6, KEY_ENTER)
};

/* REVISIT - this needs to be done through add_subdevice
 * API
 */
static struct resource resources_keypad[] = {
        {
                .start  = PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
                .end    = PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
                .flags  = IORESOURCE_IRQ,
        },
        {
                .start  = PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
                .end    = PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
                .flags  = IORESOURCE_IRQ,
        },
};

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
        .gpio_base      = PM8058_GPIO_PM_TO_SYS(0),
        .irq_base       = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, 0),
        .init           = pm8058_gpios_init,
};


static struct matrix_keymap_data rib_keymap_data = {
        .keymap_size    = ARRAY_SIZE(rib_keymap),
        .keymap         = rib_keymap,
};

static struct pmic8058_keypad_data rib_keypad_data = {
        .input_name             = "rib-keypad",
        .input_phys_device      = "rib-keypad/input0",
        .num_rows               = 5,
        .num_cols               = 7,
        .rows_gpio_start        = 8,
        .cols_gpio_start        = 0,
        .debounce_ms            = {8, 10},
        .scan_delay_ms          = 32,
        .row_hold_ns            = 91500,
        .wakeup                 = 1,
        .keymap_data            = &rib_keymap_data,
	.rep_delay		= 500,
	.rep_period		= 100,
	.key_prox_timeout       = 50,
	.key_prox_width         = 6,
	.key_prox_map           = board_key_prox_map,
};


#ifdef CONFIG_KEYBOARD_GPIO_PE
static struct gpio_keys_button rib_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_VOLUMEUP,
		.gpio			= GPIO_VOL_UP,
		.active_low		= 1,
		.desc			= "volume up",
		.debounce		= 10,
		.type			= EV_KEY,
		.wakeup			= 1,
		.options		= 0,
		.noise_mode		= MODE_MISSING_INT_INTERPRETATION
	},
	[1] = {
		.code			= KEY_VOLUMEDOWN,
		.gpio			= GPIO_VOL_DOWN,
		.active_low		= 1,
		.desc			= "volume down",
		.debounce		= 10,
		.type			= EV_KEY,
		.wakeup			= 1,
		.options		= 0,
		.noise_mode		= MODE_MISSING_INT_INTERPRETATION
	},
	[2] = {
		.code			= SW_RINGER,
		.gpio			= GPIO_RINGER,
		.active_low		= 1,
		.desc			= "ring silence",
		.debounce		= 100,
		.type			= EV_SW,
		.wakeup			= 1,
		.noise_mode		= MODE_NOISE_INTERPRETATION,
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
		.options		= OPT_REBOOT_TRIGGER | OPT_REBOOT_TRIGGER_EDGE,
#endif
	},
	[3] = {
		.code			= KEY_END,
		.gpio			= GPIO_POWER,
		.active_low		= 1,
		.desc			= "power",
		.debounce		= 10,
		.type			= EV_KEY,
		.wakeup			= 1,
		.noise_mode		= MODE_MISSING_INT_INTERPRETATION,
#ifdef  CONFIG_GPIO_KEYS_REBOOT_TRIGGER
		.options    	= OPT_REBOOT_TRIGGER | OPT_REBOOT_TRIGGER_LEVEL,
#endif
	},
	[4] = {
		.code			= KEY_SLIDER_OPEN,
		.gpio			= GPIO_SLIDER_OPEN,
		.active_low		= 1,
		.desc			= "slider open",
		.debounce		= 10,
		.type			= EV_KEY,
		.wakeup			= 1,
		.options		= 0,
	},
	[5] = {
		.code			= KEY_SLIDER_CLOSE,
		.gpio			= GPIO_SLIDER_CLOSE,
		.active_low		= 1,
		.desc			= "slider close",
		.debounce		= 10,
		.type			= EV_KEY,
		.wakeup			= 1,
		.options		= 0,
	}
};

static struct gpio_keys_platform_data rib_gpio_keys = {
	.buttons  = rib_gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(rib_gpio_keys_buttons),
};

static struct platform_device rib_gpio_keys_device = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data  = &rib_gpio_keys,
	},
};
#endif

static struct mutex wlan_reset_lock; 

static void wlan_reset_pin_control_init(void)
{
	mutex_init(&wlan_reset_lock);
}

#define WLAN_RESET_ON (true)
#define WLAN_RESET_OFF (false)
//Wlan reset pin on DVT should be set to 1 if either WiFi or BT are on
static void wlan_reset_pin_control(enum wlan_reset_pin_owner owner, bool bOn)
{
	static int active = 0;
	static int pin_owner = WLAN_NONE;

	bool need_active;

	mutex_lock(&wlan_reset_lock);

	pin_owner = bOn ? (pin_owner | owner) :
								 (pin_owner & ~owner);
	need_active = (pin_owner != WLAN_NONE);

	if (!active && need_active) {

		pinmux_config("WIFI_RESET", PINMUX_CONFIG_ACTIVE);
		gpio_set_value(GPIO_WIFI_RESET, 1);
		active = 1;
	}
	else if (active && !need_active) {

		gpio_set_value(GPIO_WIFI_RESET, 0);
		pinmux_config("WIFI_RESET", PINMUX_CONFIG_SLEEP);
		active = 0;
	}

	mutex_unlock(&wlan_reset_lock);
}

static struct mfd_cell pm8058_subdevs[] = {
        {       .name = "pm8058-keypad",
                .id             = -1,
                .num_resources  = ARRAY_SIZE(resources_keypad),
                .resources      = resources_keypad,
        },
		{		.name = "pm8058-gpio",
				.id     = -1,
				.platform_data  = &pm8058_gpio_data,
				.data_size  = sizeof(pm8058_gpio_data),
		},
		{       .name = "pm8058-pwm",
        },
};

static struct pm8058_platform_data pm8058_7x30_data = {
        .irq_base = PMIC8058_IRQ_BASE,

        .num_subdevs = ARRAY_SIZE(pm8058_subdevs),
        .sub_devices = pm8058_subdevs,
};


static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0),
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_7x30_data,
	},
};

#ifdef CONFIG_MSM_CAMERA

#define CAM_MCLK_FREQ 24000000

/* These are logical state, voltage. e_turn_on/off will translate
 * properly to pulling the line high or low as appropriate */
enum t_line_state {
	e_turn_off = 0,
	e_turn_on = 1,
};

enum t_cam_select {
	e_ov7739 = 0,
	e_mt9p013 = 1,
};

static uint32_t camera_off_mclk_gpio_table[] = {
	GPIO_CFG(GPIO_CAM_MCLK,    0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* MCLK */
};

static uint32_t camera_on_mclk_gpio_table[] = {
	GPIO_CFG(GPIO_CAM_MCLK,   1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA), 	/* MCLK */
};

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(GPIO_CAM_MCLK         , 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* MCLK */
	GPIO_CFG(GPIO_CAM_MT9P013_PWR_D, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* cam1_pwd */
	GPIO_CFG(GPIO_CAM_OV7739_PWR_D,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* cam2_pwd */
	GPIO_CFG(GPIO_CAM_MT9P013_RESET, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* cam1_rst */
	GPIO_CFG(GPIO_CAM_OV7739_RESET,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* cam2_rst */
	GPIO_CFG(GPIO_CAM_CAMERA_SELECT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* sel_cam1 */
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(GPIO_CAM_MCLK         , 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA), /* MCLK */
	GPIO_CFG(GPIO_CAM_MT9P013_PWR_D, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,   GPIO_CFG_4MA), /* cam1_pwd */
	GPIO_CFG(GPIO_CAM_OV7739_PWR_D , 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,   GPIO_CFG_4MA), /* cam2_pwd */
	GPIO_CFG(GPIO_CAM_MT9P013_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,   GPIO_CFG_4MA), /* cam1_rst */
	GPIO_CFG(GPIO_CAM_OV7739_RESET , 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,   GPIO_CFG_4MA), /* cam2_rst */
	GPIO_CFG(GPIO_CAM_CAMERA_SELECT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,   GPIO_CFG_4MA), /* sel_cam1 */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

#define SMPS_CAMERA_ID "CAMD"

static int config_camera_off_vreg_gp2( void )
{
	int rc = 0;
	struct vreg* vreg_gp2 = NULL;

	vreg_gp2  = vreg_get(NULL, "gp2");
	if ( IS_ERR(vreg_gp2) ) {
		printk(KERN_ERR "%s: gp2 get failed (%ld)\n", __func__, PTR_ERR(vreg_gp2));
		return -1;
	}

	CDBG("disable gp2\n");
	rc = vreg_disable(vreg_gp2);
	if (rc) {
		printk(KERN_ERR "%s: gp2 enable failed (%d)\n", __func__, rc);
		return -1;
	}
	return 0;
}

static int config_camera_off_vreg_wlan( void )
{
	int rc = 0;
	struct vreg* vreg_wlan = NULL;

	vreg_wlan  = vreg_get(NULL, "wlan");
	if ( IS_ERR(vreg_wlan) ) {
		printk(KERN_ERR "%s: wlan get failed (%ld)\n", __func__, PTR_ERR(vreg_wlan));
		return -1;
	}

	CDBG("disable wlan\n");
	rc = vreg_disable(vreg_wlan);
	if (rc) {
		printk(KERN_ERR "%s: wlan enable failed (%d)\n", __func__, rc);
		return -1;
	}
	return 0;
}


static int config_camera_on_vreg_wlan( void )
{
	int rc = 0;
	struct vreg* vreg_wlan = NULL;

	vreg_wlan = vreg_get(NULL, "wlan");
	if ( IS_ERR(vreg_wlan) ) {
		printk(KERN_ERR "%s: wlan get failed (%ld)\n", __func__, PTR_ERR(vreg_wlan));
		return -1;
	}

	CDBG("config wlan\n");
	rc = vreg_set_level(vreg_wlan, 1800);
	if (rc) {
		printk(KERN_ERR "%s: wlan set level failed (%d)\n", __func__, rc);
		return -1;
	}

	CDBG("enable wlan\n");
	rc = vreg_enable(vreg_wlan);
	if (rc) {
		printk(KERN_ERR "%s: wlan enable failed (%d)\n", __func__, rc);
		return -1;
	}

	return 0;
}

static int config_camera_on_vreg_gp2( void )
{
	int rc = 0;
	struct vreg* vreg_gp2 = NULL;

	vreg_gp2  = vreg_get(NULL, "gp2");
	if ( IS_ERR(vreg_gp2) ) {
		printk(KERN_ERR "%s: gp2 get failed (%ld)\n", __func__, PTR_ERR(vreg_gp2));
		return -1;
	}

	CDBG("config gp2\n");
	rc = vreg_set_level(vreg_gp2, 2850);
	if (rc) {
		printk(KERN_ERR "%s: gp2 set level failed (%d)\n", __func__, rc);
		return -1;
	}

	CDBG("enable gp2\n");
	rc = vreg_enable(vreg_gp2);
	if (rc) {
		printk(KERN_ERR "%s: gp2 enable failed (%d)\n", __func__, rc);
		return -1;
	}
	return 0;
}

static int camera_vregs_ref_cnt = 0; 

static atomic_t camera_i2c_clock_state;

#define I2C_CLOCK_STATE_SLEEP 20
#define I2C_2_POWER_TIMEOUT_MS 200

static int camera_set_vreg_state(int state)
{
	atomic_set(&camera_i2c_clock_state,state);
	return 0;
}

static int camera_wait_i2c_2_clk_off(void)
{
	int i = (2 * I2C_2_POWER_TIMEOUT_MS) / I2C_CLOCK_STATE_SLEEP;

	do {
		if( 0 == atomic_read(&camera_i2c_clock_state) )
			break;
		msleep(I2C_CLOCK_STATE_SLEEP);
		i-=1;
	} while(i > 0);

	return atomic_read(&camera_i2c_clock_state) ;
}

static int config_camera_off_vreg( void )
{
	int rc = 0;

	CDBG("%s : %d : vreg counter:%d \n", __func__ , __LINE__ , camera_vregs_ref_cnt);

	/* if the camera vregs are already thought to be off, there is nothing to do */
	if(camera_vregs_ref_cnt < 1)
		return 0;

	camera_vregs_ref_cnt -= 1;
	if( camera_vregs_ref_cnt) {
		CDBG("%s : %d : vreg counter:%d \n", __func__ , __LINE__ , camera_vregs_ref_cnt);
	       	return rc;
	}

	/* The lines are on and need to be turned off, but wait for i2c to be done */
	rc = camera_wait_i2c_2_clk_off();
	if(rc != 0) {
		printk(KERN_ERR "%s: timeout waiting for i2c clk state:%d\n", __func__, rc);
	}

	/* Turn off the 1.8v line */
	rc = config_camera_off_vreg_wlan();
	if (rc != 0) {
		printk(KERN_ERR "%s: wlan disable failed (%d)\n", __func__, rc);
		return rc;
	}

	/* Turn off the 2.8v line */
	rc = config_camera_off_vreg_gp2();
	if (rc != 0) {
		printk(KERN_ERR "%s: gp2 disable failed (%d)\n", __func__, rc);
		return rc;
	}

	// allow S4 to enter PFM mode when Camera is off.
	rc = pmapp_smps_mode_vote(SMPS_CAMERA_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_DONTCARE);
	if (rc != 0) {
		printk("pmapp_smps_mode_vote error %d\n", rc);
	}

	/* delay for 1_8v line to drop */
	CDBG("%s: Delay after vreg powerdown\n", __func__ );
	msleep(200);

	return 0;
}

static int config_camera_on_vreg( void )
{
	int rc = 0;

	CDBG("%s : %d : vreg counter:%d \n", __func__ , __LINE__ , camera_vregs_ref_cnt);

	camera_vregs_ref_cnt += 1;
	if( 1 != camera_vregs_ref_cnt) {
		CDBG("%s : %d : vreg counter:%d \n", __func__ , __LINE__ , camera_vregs_ref_cnt);
	       	return 0;
	}

	// keep S4 regulator in PWM mode
	rc = pmapp_smps_mode_vote(SMPS_CAMERA_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_PWM);
	if (rc != 0) {
		printk("pmapp_smps_mode_vote error %d\n", rc);
	}

	/* Turn on the 1.8v line */
	rc = config_camera_on_vreg_wlan();
	if (rc != 0) {
		printk(KERN_ERR "%s: wlan enable failed (%d)\n", __func__, rc);
		return rc;
	}

	/* Turn on the 2.8v line */
	rc = config_camera_on_vreg_gp2();
	if (rc != 0) {
		printk(KERN_ERR "%s: gp2 enable failed (%d)\n", __func__, rc);
	}

	return rc;
}

/* Put the gpio in "on" mode for cameras */
static void config_camera_off_gpios( void )
{
	CDBG("CAMERA - disable gpios\n");
	config_gpio_table(camera_off_gpio_table, ARRAY_SIZE(camera_off_gpio_table));
}

/* Put the gpio in "on" mode for cameras */
static void config_camera_on_gpios( void )
{
	CDBG("CAMERA - enable gpios\n");
	config_gpio_table( camera_on_gpio_table, ARRAY_SIZE(camera_on_gpio_table) );
}

static void config_camera_set_mclk_pins( enum t_line_state state )
{
	CDBG("CAMERA - %s MCLK\n" , state == e_turn_on ? "enable" : "disable" );

	if(state == e_turn_off) {
		CDBG("CAMERA - disable MCLK gpios\n");
		config_gpio_table(camera_off_mclk_gpio_table, ARRAY_SIZE(camera_off_mclk_gpio_table));
	} else {
		CDBG("CAMERA - enable MCLK gpios\n");
		config_gpio_table( camera_on_mclk_gpio_table, ARRAY_SIZE(camera_on_mclk_gpio_table) );
	}
	
	return;
}
static void camera_select(enum t_cam_select cam)
{
	int real_state = ( cam == e_ov7739 ? 0 : 1 );

	CDBG("set %d - SEL_CAM %d\n" , GPIO_CAM_CAMERA_SELECT, cam);
	gpio_set_value(GPIO_CAM_CAMERA_SELECT, real_state );
}

static void mt9p013_set_reset(enum t_line_state state)
{
	int real_state = ( state == e_turn_on ? 1 : 0 );

	CDBG("set %d - CAM1_RST_%s\n", GPIO_CAM_MT9P013_RESET, state == e_turn_on ? "ON" : "OFF" );
	gpio_set_value(GPIO_CAM_MT9P013_RESET, real_state);
}

static void mt9p013_set_camera(enum t_line_state state)
{
	int real_state = ( state == e_turn_on ? 0 : 1 );

	CDBG("set %d - CAM_%s\n", GPIO_CAM_MT9P013_PWR_D, state == e_turn_on ? "ON" : "OFF");
	gpio_set_value(GPIO_CAM_MT9P013_PWR_D , real_state);
}

static void ov7739_set_reset(enum t_line_state state)
{
	int real_state = ( state == e_turn_on ? 0 : 1 );

	CDBG("set %d - CAM2_RST_%s\n", GPIO_CAM_OV7739_RESET, state == e_turn_on ? "ON" : "OFF");
	gpio_set_value( GPIO_CAM_OV7739_RESET , real_state );
}

static void ov7739_set_camera(enum t_line_state state)
{
	int real_state = ( state == e_turn_on ? 0 : 1 );

	CDBG("set %d - CAM_%s\n", GPIO_CAM_OV7739_PWR_D, state == e_turn_on ? "ON" : "OFF");
	gpio_set_value(GPIO_CAM_OV7739_PWR_D , real_state );
}

static void camera_disable_mclk( void )
{
	struct clk *clk = NULL;

	clk = clk_get(NULL, "cam_m_clk");

	if (!IS_ERR(clk)) {
		clk_disable(clk);
	}

	return;
}

static void camera_enable_mclk( void )
{
	struct clk *clk = NULL;

	clk = clk_get(NULL, "cam_m_clk");

	clk_set_rate(clk, CAM_MCLK_FREQ);

	if (!IS_ERR(clk)) {
		clk_enable(clk);
	}
	return;
}



#ifdef CONFIG_WEBCAM_OV7739
static void ov7739_config_camera_on_gpios(void)
{
	config_camera_on_vreg();

	config_camera_on_gpios();
	/* State is now reset=0 & power=0 */

	config_camera_set_mclk_pins( e_turn_off );
	mdelay(3); /* MCLK state change seems to take ~3ms*/

	ov7739_set_camera(e_turn_off);/* PWDN go HIGH for 5ms*/
	ov7739_set_reset(e_turn_on);  /* RESETB active low */

	mdelay(5);                    /* 5ms delay for hard reset per spec. */

	ov7739_set_camera(e_turn_on); /* PWDN go LOW */
	ov7739_set_reset(e_turn_off); /* TODO: is needed?*/
	camera_select(e_ov7739);      /* Select This camera */

	mdelay(1);                    /* 1ms hold for stability */

	camera_enable_mclk();
	config_camera_set_mclk_pins( e_turn_on );
	mdelay(3); /* MCLK state change seems to take ~3ms*/
	
	mdelay(1);                    /* 1ms delay to hold clock on per spec */
}

static void ov7739_config_camera_off_gpios(void)
{
	ov7739_set_camera(e_turn_off);

	config_camera_set_mclk_pins( e_turn_off );
	camera_disable_mclk();

	config_camera_off_gpios();

	config_camera_off_vreg();
}
#endif

#ifdef CONFIG_MT9P013
static void mt9p013_config_camera_on_gpios(void)
{
	config_camera_on_vreg();

	config_camera_on_gpios();
	config_camera_set_mclk_pins( e_turn_off );
	mdelay(3); /* MCLK state change seems to take ~3ms*/

	mt9p013_set_reset(e_turn_on);
	mt9p013_set_camera(e_turn_on);

	camera_enable_mclk();
	config_camera_set_mclk_pins( e_turn_on );
	mdelay(3); /* MCLK state change seems to take ~3ms*/

	
	mt9p013_set_reset(e_turn_off);
	usleep(1000); /* T4 - active hard reset minimum per spec*/

	mt9p013_set_reset(e_turn_on);
	usleep(1100); /* T5 + T6 :: 2400 ticks @ 24MHz + PLL clock of 1ms */
	
	camera_select(e_mt9p013);
}

static void mt9p013_config_camera_off_gpios(void)
{
	mt9p013_set_reset(e_turn_off);

	/* reset needs to go down before power down of lines */
	mt9p013_set_camera(e_turn_off);

	config_camera_set_mclk_pins( e_turn_off );
	camera_disable_mclk();

	config_camera_off_gpios();

	config_camera_off_vreg();
}
#endif

static void rib_init_camera(void)
{
	ov7739_set_reset(e_turn_on);
	mt9p013_set_reset(e_turn_on);

	mt9p013_set_camera(e_turn_off);
	ov7739_set_camera(e_turn_off);

	config_camera_off_gpios();
}

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};


#ifdef CONFIG_MT9P013
struct msm_camera_device_platform_data mt9p013_camera_device_data = {
	.camera_gpio_on  = mt9p013_config_camera_on_gpios,
	.camera_gpio_off = mt9p013_config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = CAM_MCLK_FREQ,
	.ioclk.vfe_clk_rate  = 160000000,
};
#endif

#ifdef CONFIG_WEBCAM_OV7739
struct msm_camera_device_platform_data ov7739_camera_device_data = {
	.camera_gpio_on  = ov7739_config_camera_on_gpios,
	.camera_gpio_off = ov7739_config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = CAM_MCLK_FREQ,
	.ioclk.vfe_clk_rate  = 122880000,
};
#endif

#ifdef CONFIG_MT9P013
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_USER,
	._fsrc.user_src.low_current = 200,
	._fsrc.user_src.high_current = 600,
	._fsrc.user_src.set_current = lm8502_set_current,
	._fsrc.user_src.enable_flash = mt9p013_enable_flash,
};
#endif

#ifdef CONFIG_WEBCAM_OV7739
static struct msm_camera_sensor_flash_data msm_flash_none = {
       .flash_type = MSM_CAMERA_FLASH_NONE,
       .flash_src  = NULL
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7739_data = {
	.sensor_name    = "ov7739",
	.sensor_reset   = 127,
	.sensor_pwd     = 126,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &ov7739_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &msm_flash_none,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_ov7739 = {
	.name      = "msm_camera_ov7739",
	.dev       = {
		.platform_data = &msm_camera_sensor_ov7739_data,
	},
};
#endif

#ifdef CONFIG_MT9P013
static struct msm_camera_sensor_flash_data flash_mt9p013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p013_data = {
	.sensor_name    = "mt9p013",
	.sensor_reset   = 92,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &mt9p013_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p013,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9p013 = {
	.name      = "msm_camera_mt9p013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p013_data,
	},
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#ifdef CONFIG_MT9P013
static int mt9p013_init(void)
{
	int rc;
	rc = lm8502_strobe_input_enable(true);
	if (rc < 0)
		return rc;
	rc = lm8502_set_led_current(10, 0xFF);
	return rc;
}

static int mt9p013_deinit(void)
{
	int rc;
	rc = lm8502_strobe_input_enable(false);
	if (rc < 0)
		return rc;
	rc = lm8502_set_led_current(10, 0x00);
	return rc;
}

static struct mt9p013_platform_data mt9p013_pdata = {
	.init		  = mt9p013_init,
	.deinit		  = mt9p013_deinit,
	.power_shutdown	  = NULL,
	.power_resume	  = NULL,
};
#endif

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_WEBCAM_OV7739
	{
		I2C_BOARD_INFO("ov7739", 	0x78),
	},
#endif
#ifdef CONFIG_MT9P013
	{
		I2C_BOARD_INFO("mt9p013", 	0x6C),
		.platform_data = &mt9p013_pdata,
	},
#endif
};

#endif /*CONFIG_MSM_CAMERA*/

#ifdef CONFIG_MFD_WM8994

#define WM8994_I2C_SLAVE_ADDR 0x1a
#define WM8994_DEVICE "wm8958"
#define WM8994_I2C_BUS 0x4

static void wm8994_ldo_power(int enable)
{
	gpio_tlmm_config(GPIO_CFG(GPIO_LDO1, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_LDO2, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (enable) {
		pr_err("%s: Power up the WM8994 LDOs\n", __func__);
		gpio_set_value_cansleep(GPIO_LDO1, 1);
		gpio_set_value_cansleep(GPIO_LDO2, 1);
	} else {
		pr_err("%s: Power down the WM8994 LDOs\n", __func__);
		gpio_set_value_cansleep(GPIO_LDO1, 0);
		gpio_set_value_cansleep(GPIO_LDO2, 0);
	}
}

static unsigned int msm_wm8994_setup_power(void)
{
	wm8994_ldo_power(1);
	msleep(200);

	return 0;
}

static void msm_wm8994_shutdown_power(void)
{
	pr_err("%s: codec power shutdown\n", __func__);
	wm8994_ldo_power(0);
}

static struct wm8994_drc_cfg rib_drc_cfgs[] = {
	{
		.name = "Speaker",
		.regs = {0x0124, 0x0868, 0x0022, 0x0101, 0x02e8},
	}
};

static struct wm8994_pdata wm8994_pdata = {
	.gpio_defaults = {
		0xa141, 0xa000, 0xa100, 0xa100, 0xa101, 0xa100,
		0xa100, 0xa101, 0xa101, 0xa101, 0xa101,
	},

	/* Put the line outputs into differential mode so that the driver
	 * knows it can power the chip down to cold without pop/click issues.
	 * Line outputs are not actually connected on the board.
	 */

	.wm8994_setup = msm_wm8994_setup_power,
	.wm8994_shutdown = msm_wm8994_shutdown_power,
	.force_route = 0,
	.num_drc_cfgs = 1,
	.drc_cfgs = rib_drc_cfgs,
};

static struct i2c_board_info wm8994_i2c_board_info = {
	I2C_BOARD_INFO(WM8994_DEVICE, WM8994_I2C_SLAVE_ADDR),
	.platform_data = &wm8994_pdata,
};

static void __init wm8994_init(void)
{
	printk(KERN_ERR "Registering wm8994 device.\n");
	i2c_register_board_info(WM8994_I2C_BUS, &wm8994_i2c_board_info, 1);
}

#endif /* CONFIG_MFD_WM8994 */

#ifdef CONFIG_MSM7KV2_AUDIO
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static uint32_t audio_fluid_icodec_tx_config =
  GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}
	return rc;
}

void msm_snddev_tx_route_config(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		} else
			gpio_set_value(85, 0);
	}
}

void msm_snddev_tx_route_deconfig(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		}
	}
}

void msm_snddev_poweramp_on(void)
{
	gpio_set_value(82, 1);	/* enable spkr poweramp */
	pr_info("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off(void)
{
	gpio_set_value(82, 0);	/* disable spkr poweramp */
	pr_info("%s: power off amplifier\n", __func__);
}

static struct vreg *snddev_vreg_ncp, *snddev_vreg_gp4;

void msm_snddev_hsed_voltage_on(void)
{
	int rc;

	snddev_vreg_gp4 = vreg_get(NULL, "gp4");
	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_enable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_enable(gp4) failed (%d)\n", __func__, rc);

	snddev_vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_enable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_enable(ncp) failed (%d)\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc;

	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_disable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_disable(ncp) failed (%d)\n", __func__, rc);
	vreg_put(snddev_vreg_ncp);

	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_disable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_disable(gp4) failed (%d)\n", __func__, rc);

	vreg_put(snddev_vreg_gp4);

}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
			= &rib_keypad_data;
	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
			= sizeof(rib_keypad_data);

	i2c_register_board_info(6 /* I2C_SSBI ID */, pm8058_boardinfo,
				ARRAY_SIZE(pm8058_boardinfo));

	return 0;
}

#ifdef CONFIG_TIMPANI_CODEC
static uint32_t timpani_reset_on_gpio[] = {
	GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)};

static uint32_t timpani_reset_off_gpio[] = {
	GPIO_CFG(1, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)};

static void config_timpani_reset_on(void)
{
	config_gpio_table(timpani_reset_on_gpio,
		ARRAY_SIZE(timpani_reset_on_gpio));
}

static void config_timpani_reset_off(void)
{
	config_gpio_table(timpani_reset_off_gpio,
		ARRAY_SIZE(timpani_reset_off_gpio));
}
#endif


#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

#ifdef CONFIG_NDUID
static struct nduid_config nduid_cfg[] = {
	{
		.dev_name = "nduid",
	},
};

static struct platform_device nduid_device = {
	.name = "nduid",
	.id   = -1,
	.dev  = {
		.platform_data = &nduid_cfg,
	},
};
#endif

#ifdef CONFIG_HRES_COUNTER
static int msm_hres_timer_init(void** timer)
{
    return 0;
}

static int msm_hres_timer_release(void* timer)
{
    return 0;
}

#ifdef CONFIG_PM
static int msm_hres_timer_suspend(void *timer)
{
    return 0;
}

static int msm_hres_timer_resume(void *timer)
{
    return 0;
}
#else
#define msm_hres_timer_suspend    NULL
#define msm_hres_timer_resume     NULL
#endif

extern u32 msm_dgt_read_count(void);
static u32 msm_hres_timer_read(void* timer)
{
    return msm_dgt_read_count();
}

extern u32 msm_dgt_convert_usec(u32);
static u32 msm_hres_timer_convert(u32 count)
{
    // Count is in 24.576Mhz terms
    // convert it to uSec
    // return (count * 400) / 2457;
    return msm_dgt_convert_usec(count);
}

static struct hres_counter_platform_data msm_hres_counter_platform_data = {
    .init_hres_timer = msm_hres_timer_init,
    .release_hres_timer = msm_hres_timer_release,
    .suspend_hres_timer = msm_hres_timer_suspend,
    .resume_hres_timer = msm_hres_timer_resume,
    .read_hres_timer = msm_hres_timer_read,
    .convert_hres_timer = msm_hres_timer_convert,
};

static struct platform_device hres_counter_device = {
    .name = "hres_counter",
    .id   = -1,
    .dev  = {
        .platform_data  = &msm_hres_counter_platform_data,
    }
};
#endif


#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID
static char *usb_functions_default[] = {
	"diag",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_default_adb[] = {
	"diag",
	"adb",
	"modem",
	"nmea",
	"rmnet",
	"usb_mass_storage",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
	"usb_mass_storage",
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x9026,
		.num_functions	= ARRAY_SIZE(usb_functions_default),
		.functions	= usb_functions_default,
	},
	{
		.product_id	= 0x9025,
		.num_functions	= ARRAY_SIZE(usb_functions_default_adb),
		.functions	= usb_functions_default_adb,
	},
	{
		.product_id	= 0xf00e,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= 0x9024,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "Qualcomm Incorporated",
	.product        = "Mass storage",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x05C6,
	.vendorDescr	= "Qualcomm Incorporated",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0x9026,
	.version	= 0x0100,
	.product_name		= "Qualcomm HSUSB Device",
	.manufacturer_name	= "Qualcomm Incorporated",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
};

static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},

};
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM,
	.vendor_id	= 0x5c6,
	.product_name	= "Qualcomm HSUSB Device",
	.serial_number	= "1234567890ABCDEF",
	.manufacturer_name
			= "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions
			= ARRAY_SIZE(usb_func_composition),
	.function_map	= usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.core_clk	= 1,
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "headset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "headset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].latency = 500,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].residency = 6000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

#ifdef CONFIG_LEDS_LM3528

static struct lm3528_platform_data board_lm3528_data = {
	.cdev = {
		.name = "lcd",
	},
	.default_brightness = 40,
	.boost_wait_time = 2, //minimum wait time in ms
	.dev_name = "lm3528",
};

static struct i2c_board_info lm3528_i2c_board_info = {
	I2C_BOARD_INFO(LM3528_I2C_DEVICE, LM3528_I2C_ADDR),
	.platform_data = &board_lm3528_data,
};

#endif // CONFIG_LEDS_LM3528

#ifdef CONFIG_LEDS_LM8502

void lm8502_select_flash(struct i2c_client* client)
{
    lm8502_i2c_write_reg(client, MISC, 0x28);//enable boost, enable powersave
    lm8502_i2c_write_reg(client, D10_CURRENT_CTRL, 0xFF);// select FLASH
    lm8502_i2c_write_reg(client, HAPTIC_CONTROL, 0);//disable Vibrator
}

void lm8502_select_vibrator(struct i2c_client* client)
{
    lm8502_i2c_write_reg(client, TORCH_BRIGHTNESS, 0x4);//disable Torch
    lm8502_i2c_write_reg(client, FLASH_BRIGHTNESS, 0x4);//disable Flash
    lm8502_i2c_write_reg(client, MISC, 0x2A);//enable boost, enable powersave, external PWM input select
    lm8502_i2c_write_reg(client, D10_CURRENT_CTRL, 0);//select vibrator
}

static struct led_cfg keypad_group[] = {
    [0] = {
        .type = WHITE,
        .current_addr = D4_CURRENT_CTRL,
        .control_addr = D4_CONTROL,
    },
    [1] = {
        .type = WHITE,
        .current_addr = D5_CURRENT_CTRL,
        .control_addr = D5_CONTROL,
    },
};

static struct led_cfg core_navi_left_group[] = {
    [0] = {
        .type  = WHITE,
        .current_addr = D3_CURRENT_CTRL,
        .control_addr = D3_CONTROL,
    },
};

static struct led_cfg core_navi_center_group[] = {
    [0] = {
        .type  = WHITE,
        .current_addr = D2_CURRENT_CTRL,
        .control_addr = D2_CONTROL,
    },
};

static struct led_cfg core_navi_right_group[] = {
    [0] = {
        .type  = WHITE,
        .current_addr = D1_CURRENT_CTRL,
        .control_addr = D1_CONTROL,
    },
};
static struct lm8502_led_config led_lm8502_data[] = {
    [GRP_1] = {
        .cdev = {
            .name = "keypad",
            .max_brightness = 100,
            .flags = LED_CORE_SUSPENDRESUME,
         },
        .led_list = &keypad_group[0],
        .nleds = ARRAY_SIZE(keypad_group),
        .group_id = GRP_1,
        .hw_group = HW_GRP_1,
        .default_max_current = 0x3, //25.5mA
        .default_brightness = 0,
        .default_state = LED_ON,
    },
    [GRP_2] = {
        .cdev = {
            .name = "core_navi_right",
            .max_brightness = 100,
            .flags = LED_CORE_SUSPENDRESUME,
        },
        .led_list = &core_navi_right_group[0],
        .nleds = ARRAY_SIZE(core_navi_right_group),
        .group_id = GRP_2,
        .hw_group = HW_GRP_NONE,
        .default_max_current  = 0x2, //12.5mA
        .default_brightness = 0,
        .default_state = LED_OFF,
    },
    [GRP_3] = {
        .cdev = {
            .name = "core_navi_center",
            .max_brightness = 100,
            .flags = LED_CORE_SUSPENDRESUME,
        },
        .led_list = &core_navi_center_group[0],
        .nleds = ARRAY_SIZE(core_navi_center_group),
        .group_id = GRP_3,
        .hw_group = HW_GRP_NONE,
        .default_max_current  = 0x2, //12.5mA
        .default_brightness = 0,
        .default_state = LED_OFF,
    },
    [GRP_4] = {
        .cdev = {
            .name = "core_navi_left",
            .max_brightness = 100,
            .flags = LED_CORE_SUSPENDRESUME,
        },
        .led_list = &core_navi_left_group[0],
        .nleds = ARRAY_SIZE(core_navi_left_group),
        .group_id = GRP_4,
        .hw_group = HW_GRP_NONE,
        .default_max_current  = 0x2, //12.5mA
        .default_brightness = 0,
        .default_state = LED_OFF,
    },
};

static struct lm8502_memory_config led_lm8502_memcfg = {
    .eng1_startpage = 0,
    .eng1_endpage = 0,
    .eng2_startpage = 1,
    .eng2_endpage = 1,
};

static struct lm8502_platform_data board_lm8502_data = {
    .enable_gpio = GPIO_LM8502_EN,
    .interrupt_gpio = GPIO_LM8502_INT,
    .flash_default_current = 600,
    .flash_default_duration = 512,
    .torch_default_current = 150,
    .vib_default_duty_cycle = 50,
    .vib_default_direction = 1,
    .vib_invert_direction = 1,
    .select_flash = lm8502_select_flash,
    .select_vibrator = lm8502_select_vibrator,
    .nleds = ARRAY_SIZE(led_lm8502_data),
    .leds = led_lm8502_data,
    .memcfg = &led_lm8502_memcfg,
    .power_mode = MISC_POWER_SAVE_ON,
    .dev_name = "lm8502",
};

static struct i2c_board_info lm8502_board_info = {
    I2C_BOARD_INFO(LM8502_I2C_DEVICE, LM8502_I2C_ADDR),
    .platform_data = &board_lm8502_data,
    .irq = MSM_GPIO_TO_INT(GPIO_LM8502_INT),
};

#endif // CONFIG_LEDS_LM8502

/* The following #defines facilitate selective inclusion of a specific a6 wakeup strategy:
   [constraint]: A6_PMIC_EXTERNAL_WAKE and A6_INTERNAL_WAKE are mutually exclusive
   A6_PMIC_EXTERNAL_WAKE: configures a6 driver to use PMIC-based LPG PWM for wakeup
   A6_INTERNAL_WAKE: configures for a6-based internal wake
   if neither defined: configures A6 driver to keep a6 constantly awake using SBW_WAKEUP pin
*/
//#define A6_PMIC_EXTERNAL_WAKE
#define A6_INTERNAL_WAKE
/* end a6 wakeup selection */

#ifdef CONFIG_A6

static struct a6_sbw_interface sbw_ops_impl_0;
#if (defined A6_PMIC_EXTERNAL_WAKE || defined A6_INTERNAL_WAKE)
static struct a6_wake_ops a6_wake_ops_impl_0;
#endif

static struct msm_gpio a6_sbw_init_gpio_config_evt1[] = {
    {GPIO_CFG(61, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "A6_TCK"},
    {GPIO_CFG(165, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "A6_WAKEUP"},
    /* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
       this and it floats in input mode unless explicitly pulled-up by host. */
    {GPIO_CFG(62, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "A6_TDIO"},
};

static struct msm_gpio a6_sbw_deinit_gpio_config_evt1[] = {
    {GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "A6_TCK"},
    {GPIO_CFG(165, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "A6_WAKEUP"},
    {GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "A6_TDIO"},
};

static int a6_sbw_init_imp(struct a6_platform_data* plat_data)
{
    int rc = 0;

    rc = msm_gpios_enable((struct msm_gpio*)plat_data->sbw_init_gpio_config,
                  plat_data->sbw_init_gpio_config_size);
    if (rc < 0) {
        printk(KERN_ERR "%s: failed to configure A6 SBW gpios.\n", __func__);
    }

    return rc;
}

static int a6_sbw_deinit_imp(struct a6_platform_data* plat_data)
{
    int rc = 0;

    rc = msm_gpios_enable((struct msm_gpio*)plat_data->sbw_deinit_gpio_config,
                  plat_data->sbw_deinit_gpio_config_size);
    if (rc < 0) {
        printk(KERN_ERR "%s: failed to de-configure A6 SBW gpios.\n", __func__);
    }

    return rc;
}

static struct a6_platform_data rib_a6_platform_data_evt1_0 = {
    .dev_name       = A6_DEVICE,
    .pwr_gpio           = 51,
    .sbw_tck_gpio       = 61,
    .sbw_tdio_gpio      = 62,
    .sbw_wkup_gpio      = 165,
    .sbw_ops        = &sbw_ops_impl_0,

    .sbw_init_gpio_config   = a6_sbw_init_gpio_config_evt1,
    .sbw_init_gpio_config_size = ARRAY_SIZE(a6_sbw_init_gpio_config_evt1),
    .sbw_deinit_gpio_config = a6_sbw_deinit_gpio_config_evt1,
    .sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_sbw_deinit_gpio_config_evt1),

    .sbw_init       = a6_sbw_init_imp,
    .sbw_deinit     = a6_sbw_deinit_imp,
};


static struct msm_gpio a6_config_data_evt1[] = {
	{GPIO_CFG(51, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "A6_MSM_IRQ"}
};

static struct i2c_board_info a6_i2c_board_info_0 = {
    I2C_BOARD_INFO( A6_DEVICE, (0x62>>1)),
    .platform_data = NULL,
};

#if defined A6_PMIC_EXTERNAL_WAKE
static struct a6_pmic_wake_interface_data {
    int pwm_channel;
    int pwm_period;
    int pwm_duty;
    struct pwm_device *pwm_data;
} a6_wi_data_0 =

{
    .pwm_channel = 1,
    .pwm_period = 10000,
    .pwm_duty = 20,
};

static int a6_enable_pwm(void *data)
{
    struct a6_pmic_wake_interface_data *pdata = (struct a6_pmic_wake_interface_data *)data;
    int rc;

    pdata->pwm_data = pwm_request(pdata->pwm_channel, "a6_pwm");
    if (IS_ERR(pdata->pwm_data)) {
        printk(KERN_ERR "%s: failed to request pwm channel %d, error %d\n",
               __func__, pdata->pwm_channel, (int)pdata->pwm_data);
        return 1;
    }

    rc = pwm_config_ex(pdata->pwm_data, pdata->pwm_duty, pdata->pwm_period);
    if (rc) {
        printk(KERN_ERR "%s: failed to configure pwm channel %d, error %d\n",
               __func__, pdata->pwm_channel, rc);
        return 2;
    }

    rc = pwm_enable(pdata->pwm_data);
    if (rc) {
        printk(KERN_ERR "%s: failed to enable pwm channel %d, error %d\n",
               __func__, pdata->pwm_channel, rc);
        return 3;
    }

    return 0;
}

static int a6_disable_pwm(void *data)
{
    struct a6_pmic_wake_interface_data *pdata = (struct a6_pmic_wake_interface_data *)data;

    pwm_disable(pdata->pwm_data);
    pwm_free(pdata->pwm_data);
    return 0;
}

enum a6_force_state {
    a6_force_sleep_state,
    a6_force_wake_state
};

static int a6_force_sleep_wake(enum a6_force_state state)
{
    int32_t rc = 0;
    struct pm8058_gpio gpio_cfg = {
        .direction = PM_GPIO_DIR_OUT,
        .output_buffer = PM_GPIO_OUT_BUF_CMOS,
        .output_value = (a6_force_sleep_state == state) ? 1 : 0,
        .pull = (a6_force_sleep_state == state) ? PM_GPIO_PULL_NO : PM_GPIO_PULL_DN,
        .out_strength   = PM_GPIO_STRENGTH_HIGH,
        .function       = PM_GPIO_FUNC_2,
    };

    rc = pm8058_gpio_config(24, &gpio_cfg); /* pmic gpio 25 */
    if (rc) {
        printk(KERN_ERR "%s PMIC GPIO 25 write failed\n", __func__);
        return rc;
    }

    return rc;
}

static int32_t a6_force_wake(void* data)
{
    int32_t rc;
    data = data;

    rc = a6_force_sleep_wake(a6_force_wake_state);
    if (!rc) {
        msleep(30);
    }

    return rc;
}

static int a6_force_sleep(void* data)
{
    data = data;

    return a6_force_sleep_wake(a6_force_sleep_state);
}
#elif defined A6_INTERNAL_WAKE
static struct a6_internal_wake_interface_data {
    int wake_enable: 1;
    int wake_period: 9;
    int wake_gpio;
} a6_wi_data_0 =
{
    .wake_enable = 1,
    .wake_period = 16,
};

static int32_t a6_force_wake(void* data)
{
	struct a6_internal_wake_interface_data *pdata = (struct a6_internal_wake_interface_data *)data;
	unsigned long flags = 0;

	a6_disable_interrupts(flags);
	gpio_set_value(pdata->wake_gpio, 1);
	udelay(1);
	gpio_set_value(pdata->wake_gpio, 0);
	udelay(1);
	gpio_set_value(pdata->wake_gpio, 1);
	a6_enable_interrupts(flags);

	msleep(30);

	return 0;
}

static int a6_force_sleep(void* data)
{
    struct a6_internal_wake_interface_data *pdata = (struct a6_internal_wake_interface_data *)data;

    gpio_set_value(pdata->wake_gpio, 0);

    return 0;
}


static int a6_internal_wake_enable_state(void* data)
{
    return (((struct a6_internal_wake_interface_data *)data)->wake_enable);
}

static int a6_internal_wake_period(void* data)
{
    return (((struct a6_internal_wake_interface_data *)data)->wake_period);
}
#endif // A6_PMIC_EXTERNAL_WAKE


static void __init rib_init_a6(void)
{
	struct msm_gpio* config_data;
    int32_t config_data_size;

    printk(KERN_ERR "Registering a6_0 device.\n");
    a6_i2c_board_info_0.platform_data = &rib_a6_platform_data_evt1_0;
    i2c_register_board_info(0, &a6_i2c_board_info_0, 1);

    config_data = a6_sbw_init_gpio_config_evt1;
    config_data_size = ARRAY_SIZE(a6_sbw_init_gpio_config_evt1);

    msm_gpios_enable(config_data, config_data_size);

    /* no change for dvt boards */
    config_data = a6_config_data_evt1;
    config_data_size = ARRAY_SIZE(a6_config_data_evt1);
    msm_gpios_enable(config_data, config_data_size);

#if defined A6_PMIC_EXTERNAL_WAKE
    ((struct a6_platform_data*)a6_i2c_board_info_0.platform_data)->wake_ops = &a6_wake_ops_impl_0;

    a6_wake_ops_impl_0.data = &a6_wi_data_0;
    a6_wake_ops_impl_0.enable_periodic_wake = &a6_enable_pwm;
    a6_wake_ops_impl_0.disable_periodic_wake = &a6_disable_pwm;
    a6_wake_ops_impl_0.internal_wake_enable_state = NULL;
    a6_wake_ops_impl_0.internal_wake_period = NULL;
    a6_wake_ops_impl_0.force_wake = &a6_force_wake;
    a6_wake_ops_impl_0.force_sleep = &a6_force_sleep;
#elif defined A6_INTERNAL_WAKE
	((struct a6_platform_data*)a6_i2c_board_info_0.platform_data)->wake_ops = &a6_wake_ops_impl_0;

    // for non-PMIC external wakes, use sbw_wkup_gpio for force wakes...
    a6_wi_data_0.wake_gpio =
        ((struct a6_platform_data *)a6_i2c_board_info_0.platform_data)->sbw_wkup_gpio;
    a6_wake_ops_impl_0.data = &a6_wi_data_0;
    a6_wake_ops_impl_0.enable_periodic_wake = NULL;
    a6_wake_ops_impl_0.disable_periodic_wake = NULL;
	a6_wake_ops_impl_0.internal_wake_enable_state = &a6_internal_wake_enable_state;
	a6_wake_ops_impl_0.internal_wake_period = &a6_internal_wake_period;
    a6_wake_ops_impl_0.force_wake = &a6_force_wake;
    a6_wake_ops_impl_0.force_sleep = &a6_force_sleep;

#else
    ((struct a6_platform_data*)a6_i2c_board_info_0.platform_data)->wake_ops = NULL;
#endif

    sbw_ops_impl_0.a6_per_device_interface.SetSBWTCK = &a6_set_sbwtck_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.ClrSBWTCK = &a6_clr_sbwtck_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.SetSBWTDIO = &a6_set_sbwtdio_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.ClrSBWTDIO = &a6_clr_sbwtdio_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.SetInSBWTDIO = &a6_set_in_sbwtdio_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.SetOutSBWTDIO = &a6_set_out_sbwtdio_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.GetSBWTDIO = &a6_get_sbwtdio_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.SetSBWAKEUP = &a6_set_sbwakeup_evt1_0;
    sbw_ops_impl_0.a6_per_device_interface.ClrSBWAKEUP = &a6_clr_sbwakeup_evt1_0;
    sbw_ops_impl_0.a6_per_target_interface.delay = a6_delay_impl;

}
#endif // CONFIG_A6

#ifdef CONFIG_MSM_VIBRATOR

static struct platform_device board_vibe_device = {
	.name = VIBE_DEVICE,
	.id   = -1,
	.dev  = {
		.platform_data  = NULL,
	}
};
#endif

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;
	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start  = DMOV_USB_CHAN;
	qsd_spi_resources[4].end    = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};


static struct msm_spi_platform_data qsd_spi_pdata = {
    .max_clock_speed = 3000000,
    .dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
    qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}


#if defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
       || defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE)
static struct user_pin ctp_pins[] = {
	{
		.name = "wake",
  		.gpio = GPIO_CTP_WAKE,
		.act_level = 0,
		.direction = 0,
		.def_level = 1,
		.sysfs_mask = 0660,
	},
};

static int rib_cy8ctma300_gpio_request(unsigned gpio, char *name)
{
	int rc;

	rc = gpio_request(gpio, name);
	if (rc < 0)
		pr_err("error %d requesting gpio %u (%s)\n", rc, gpio, name);

	return (rc);
}

static int __init rib_cy8ctma300_init(void)
{
	int rc;

	rc = rib_cy8ctma300_gpio_request(GPIO_CTP_RESET, "CTP_RESET");
	if (rc < 0)
		goto exit;

	rc = rib_cy8ctma300_gpio_request(GPIO_CTP_SHDN, "CTP_SHDN");
exit:
	return (rc);
}

/* this needs to be called after gpio init (postcore) but before device init */
arch_initcall(rib_cy8ctma300_init);

static int rib_cy8ctma300_sclk_request(int request)
{
	int rc;

	if (request)
		rc = rib_cy8ctma300_gpio_request(GPIO_CY8CTMA300_SCLK,
							"CY8CTMA300_SCLK");

	else {
		gpio_free(GPIO_CY8CTMA300_SCLK);
		rc = 0;
	}

	return (rc);
}

static int rib_cy8ctma300_sdata_request(int request)
{
	if (request)
		pinmux_config("CY8CTMA300_SDATA", PINMUX_CONFIG_ACTIVE);

	else
		pinmux_config("CTP_WAKE", PINMUX_CONFIG_ACTIVE);

	return (0);
}

static void rib_cy8ctma300_xres_assert(int assert)
{
    (void)gpio_direction_output(GPIO_CTP_RESET, !assert);
}


#define BOOST_MIN_PWM 0x1
static void rib_cy8ctma300_vcpin_enable(int enable)
{
	int rc = 0;

	if(enable){
		rc = lm3528_boost_request(BOOST_MIN_PWM);
	}else{
		rc = lm3528_boost_release();
	}

	if(rc < 0)
		pr_err("error requesting lm3528 boost\n");

    (void)gpio_direction_output(GPIO_CTP_SHDN, enable);
}

static struct cy8ctma300_platform_data rib_cy8ctma300_data = {
    .prec_len = 64,
    .nr_srecs = 2,
    .block_len = 128,
    .nr_blocks = 256,
    .sclk_request = rib_cy8ctma300_sclk_request,
    .sdata_request = rib_cy8ctma300_sdata_request,
    .vdd_enable = NULL,
    .vcpin_enable = rib_cy8ctma300_vcpin_enable,
    .xres_assert = rib_cy8ctma300_xres_assert,
    .sclk = GPIO_CY8CTMA300_SCLK,
    .sdata = GPIO_CY8CTMA300_SDATA,
    .xres_us = 263,
    .reset_ns = 500,
    .ssclk_ns = 40,
    .hsclk_ns = 40,
    .dsclk_ns = 70,
    .wait_and_poll_ms = 200,
};

static struct platform_device rib_cy8ctma300_device = {
    .name = CY8CTMA300_DEVICE,
    .id = -1,
    .dev = {
        .platform_data = &rib_cy8ctma300_data,
    },
};

#define CTP_UART_SPEED	3000000

static struct hsuart_platform_data ctp_uart_data = {
	.dev_name = "ctp_uart",
	.uart_mode = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = CTP_UART_SPEED,
	.options = HSUART_OPTION_RX_DM | HSUART_OPTION_SCHED_RT,

	.tx_buf_size = 4096,
	.tx_buf_num = 1,
	.rx_buf_size = 4096,
	.rx_buf_num = 2,
	.max_packet_size = 1152,
	.min_packet_size = 1,
	.rx_latency = CTP_UART_SPEED/13333,	/* bytes per 750 us */
};

static struct platform_device ctp_uart_device = {
    .name = "hsuart",
    .id =  1,
    .dev  = {
        .platform_data = &ctp_uart_data,
    }
};
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA300[_MODULE] */

#ifdef CONFIG_HSUART
static int btuart_pin_mux(int on)
{
	int cfg = on ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP;

	if (!on) {
		//RTS line will be handled separately
		pinmux_config("UART1DM_RTS", cfg);
	}

	pinmux_config("UART1DM_CTS", cfg);
	pinmux_config("UART1DM_RX",  cfg);
	pinmux_config("UART1DM_TX",  cfg);

	return 0;
}

static int btuart_deassert_rts(int deassert)
{
	int cfg = deassert ? PINMUX_CONFIG_SLEEP : PINMUX_CONFIG_ACTIVE;

	pinmux_config("UART1DM_RTS", cfg);

	return 0;
}

/*
 * BT High speed UART interface
 */
static struct hsuart_platform_data btuart_data = {
	.dev_name   = "bt_uart",
	.uart_mode  = HSUART_MODE_FLOW_CTRL_NONE | HSUART_MODE_PARITY_NONE,
	.uart_speed = HSUART_SPEED_115K,
	.options    = HSUART_OPTION_DEFERRED_LOAD | HSUART_OPTION_TX_PIO | HSUART_OPTION_RX_DM ,

	.tx_buf_size = 512,
	.tx_buf_num  = 64,
	.rx_buf_size = 512,
	.rx_buf_num  = 64,
	.max_packet_size = 450, // ~450
	.min_packet_size = 6,   // min packet size
	.rx_latency      = 10, // in bytes at current speed
	.p_board_pin_mux_cb = btuart_pin_mux,
	.p_board_rts_pin_deassert_cb = btuart_deassert_rts,
};

static u64 btuart_dmamask = ~(u32)0;
static struct platform_device btuart_device = {
	.name = "hsuart",
	.id   =  0, // configure UART2 as hi speed uart
	.dev  = {
		.dma_mask           = &btuart_dmamask,
		.coherent_dma_mask  = 0xffffffff,
		.platform_data      = &btuart_data,
	}
};
#endif // Of HSUART

#ifdef CONFIG_BLUETOOTH_POWER_STATE
/*
 * Bluetooth power state driver
 */

#define SMPS_BLUEOOTH_ID "BLUE"

static int bt_power (unsigned int on)
{
	int rc = 0;

	if (on) {
		printk(KERN_INFO "Powering on BT\n");

		// keep S4 regulator in PWM mode so we can handle paging cycles during TCXO shutdown
		rc = pmapp_smps_mode_vote(SMPS_BLUEOOTH_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_PWM);
		if (rc != 0) {
			printk("pmapp_smps_mode_vote error %d\n", rc);
			return rc;
		}

		wlan_reset_pin_control(WLAN_BT, WLAN_RESET_ON);
		pinmux_config("BT_RESET", PINMUX_CONFIG_ACTIVE);
		pinmux_config("BT_BT_WAKE_HOST", PINMUX_CONFIG_ACTIVE);
	}
	else {
		printk(KERN_INFO "Powering off BT\n");

		pinmux_config("BT_RESET", PINMUX_CONFIG_SLEEP);
		pinmux_config("BT_BT_WAKE_HOST", PINMUX_CONFIG_SLEEP);

		wlan_reset_pin_control(WLAN_BT, WLAN_RESET_OFF);

		// allow S4 to enter PFM mode when BT is off.
		rc = pmapp_smps_mode_vote(SMPS_BLUEOOTH_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_DONTCARE);
		if (rc != 0) {
			printk("pmapp_smps_mode_vote error %d\n", rc);
			return rc;
		}
	}
	return rc;
}

static struct bluetooth_power_state_platform_data bluetooth_power_state_platform_data_rib = {
        .dev_name       = "bt_power",
        .bt_power = bt_power,
};

static struct platform_device bluetooth_power_state_device = {
        .name   = "bt_power",
        .id     = 0,
        .dev    = {
                .platform_data = &bluetooth_power_state_platform_data_rib
        },
};

#endif // CONFIG_BLUETOOTH_POWER_STATE




#ifdef CONFIG_USER_PINS

/*
 *   Bt Pins
 */
static struct user_pin bt_pins[] = {
        {
                .name       =  "reset",
                .gpio       =  163,
                .act_level  =  0, // active low
                .direction  =  0, // an output
                .def_level  =  0, // default level (low)
                .pin_mode   =  (void *)-1,// undefined
                .sysfs_mask =  0777,
				.options    =  0,
                .irq_handler = NULL,
                .irq_config =  0,
        },
        {
                .name       =  "host_wake",
                .gpio       =  145,
                .act_level  =  1,  // active high
                .direction  =  1,  // an input
                .def_level  = -1,  // undefined
                .pin_mode   =  (void *)-1, // undefined
                .sysfs_mask =  0777,
                .options    =  PIN_IRQ | PIN_WAKEUP_SOURCE,
                .irq_handler = NULL,
                .irq_config = IRQF_TRIGGER_RISING,
				.irq_handle_mode = IRQ_HANDLE_AUTO
        },
};

/*
 *   USD Pins
 */
static struct user_pin usd_pins [] = {
    {
        .name = "gpio_26",      // st331dlh - accelerometer int 1
        .gpio = 26,
        .act_level = 1,
        .direction = 1,
        .def_level = 0,
        .pin_mode = (void*) -1,
        .sysfs_mask = 0777,
        .options = PIN_IRQ,
        .irq_config = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
        .irq_handle_mode = IRQ_HANDLE_AUTO
    },
    {
        .name = "gpio_18",		// ils29040 - proximity / als
        .gpio = 18,
        .act_level = 1,
        .direction = 1,
        .def_level = 0,
        .pin_mode = (void*) -1,
        .sysfs_mask = 0777,
        .options = PIN_IRQ | PIN_WAKEUP_SOURCE,
        .irq_config = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
        .irq_handle_mode = IRQ_HANDLE_AUTO
    },
	{
        .name = "gpio_130",		// st331dlh - accelerometer int 2
        .gpio = 130,
        .act_level = 1,
        .direction = 1,
        .def_level = 1,
        .pin_mode = (void*) -1,
        .sysfs_mask = 0777,
        .options = PIN_IRQ,
        .irq_config = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,   // both rising and falling
        .irq_handle_mode = IRQ_HANDLE_AUTO
    },
};

/*
 *   Audio Pins for Wolfson Codec
 */
static struct user_pin audio_pins[] = {
        {
                .name       =  "LDO1",
                .gpio       =  168,
                .act_level  =  1, // active high
                .direction  =  0, // an output
                .def_level  =  0, // default level (low)
                .pin_mode   =  (void *)-1,// undefined
                .sysfs_mask =  0777,
                .options    =  0,
                .irq_handler = NULL,
                .irq_config =  0,
        },
		{
                .name       =  "LDO2",
                .gpio       =  167,
                .act_level  =  1, // active high
                .direction  =  0, // an output
                .def_level  =  0, // default level (low)
                .pin_mode   =  (void *)-1,// undefined
                .sysfs_mask =  0777,
                .options    =  0,
                .irq_handler = NULL,
                .irq_config =  0,
        },
};

static struct user_pin_set  board_user_pins_sets[] = {
        {
                .set_name = "bt",
                .num_pins = ARRAY_SIZE(bt_pins),
                .pins     = bt_pins,
        },
        {
                .set_name = "gpios",
                .num_pins = ARRAY_SIZE(usd_pins),
                .pins     = usd_pins,
        },
        {
                .set_name = "audio",
                .num_pins = ARRAY_SIZE(audio_pins),
                .pins     = audio_pins,
        },
#if defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
       || defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE)
        {
                .set_name = "ctp",
                .num_pins = ARRAY_SIZE(ctp_pins),
                .pins     = ctp_pins,
        },
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA300[_MODULE] */
};

static struct user_pins_platform_data board_user_pins_pdata = {
        .num_sets = ARRAY_SIZE(board_user_pins_sets),
        .sets     = board_user_pins_sets,
};

static struct platform_device board_user_pins_device = {
        .name = "user-pins",
        .id   = -1,
        .dev  = {
                .platform_data  = &board_user_pins_pdata,
        }
};
#endif


#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc;
	static int vbus_is_on;
	struct pm8058_gpio usb_vbus = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.vin_sel        = 2,
		.out_strength   = PM_GPIO_STRENGTH_MED,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};

	/* If VBUS is already on (or off), do nothing. */
	if (unlikely(on == vbus_is_on))
		return;

	if (on) {
		rc = pm8058_gpio_config(36, &usb_vbus);
		if (rc) {
			pr_err("%s PMIC GPIO 36 write failed\n", __func__);
			return;
		}
	} else
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(36), 0);

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
	.power_budget   = 180,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	printk("hsusb_rpc_connect++\n");
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
#ifndef CONFIG_USB_EHCI_MSM
static int ldo_on;
static struct vreg *usb_vreg;
static int msm_pmic_enable_ldo(int enable)
{
	if (ldo_on == enable)
		return 0;

	ldo_on = enable;

	if (enable)
		return vreg_enable(usb_vreg);
	else
		return vreg_disable(usb_vreg);
}

static int msm_pmic_notify_init(void)
{
	usb_vreg = vreg_get(NULL, "usb");
	if (IS_ERR(usb_vreg)) {
		pr_err("%s: usb vreg get failed\n", __func__);
		vreg_put(usb_vreg);
		return PTR_ERR(usb_vreg);
	}

	return 0;
}

static void msm_pmic_notify_deinit(void)
{
	msm_pmic_enable_ldo(0);
	vreg_put(usb_vreg);
}
#endif

static int phy_reset(void __iomem * mem)
{
	return 0;
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM
	/* vbus notification through pmic call backs */
	.pmic_notif_init         = msm_pmic_notify_init,
	.pmic_notif_deinit       = msm_pmic_notify_deinit,
	.pmic_enable_ldo         = msm_pmic_enable_ldo,
	.pmic_vbus_irq	= 1,
#else
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.core_clk		 = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.hsdrvslope		 = 0x0f,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
	.phy_reset       = phy_reset,
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	},
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = NULL,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
       .name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	* the only valid choice at this time. The board structure is
	* set to all zeros by the C runtime initialization and that is now
	* the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	* include/linux/android_pmem.h.
	*/
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 1,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
       .name = "android_pmem",
       .id = 1,
       .dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct kgsl_platform_data kgsl_pdata = {
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
	/* NPA Flow IDs */
	.high_axi_3d = MSM_AXI_FLOW_3D_GPU_HIGH,
	.high_axi_2d = MSM_AXI_FLOW_2D_GPU_HIGH,
#else
	/* AXI rates in KHz */
	.high_axi_3d = 192000,
	.high_axi_2d = 192000,
#endif
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL, /* HW workaround, run Z180 SYNC @ 192 MHZ */
	.max_grp3d_freq = 245760000,
	.min_grp3d_freq = 192 * 1000*1000,
	.set_grp3d_async = set_grp3d_async,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
};

static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA3500000, /* 3D GRP address */
		.end = 0xA351ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "kgsl_phys_memory",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRP_3D,
		.end = INT_GRP_3D,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "kgsl_g12_reg_memory",
		.start = 0xA3900000, /* Z180 base address */
		.end = 0xA3900FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "kgsl_g12_irq",
		.start = INT_GRP_2D,
		.end = INT_GRP_2D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = NULL,
	.mddi_sel_clk = NULL,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 24,
	.mdp_core_clk_rate = 192000000,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
}

#if CONFIG_SENSORS_MSM_ADC
static char *msm_adc_rib_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};
#endif


static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	//&msm_device_nand,
#ifdef CONFIG_USB_FUNCTION
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
#endif
#ifdef CONFIG_HSUART
	&btuart_device,
#endif // Of HSUART
	&bluetooth_power_state_device,
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
	&qsd_device_spi,
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi6,
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&android_pmem_kernel_ebi1_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,

#ifdef CONFIG_USER_PINS
    &board_user_pins_device,
#endif
#ifdef CONFIG_MSM_VIBRATOR
	&board_vibe_device,
#endif

	&hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
#ifdef CONFIG_HRES_COUNTER
    &hres_counter_device,
#endif
#ifdef CONFIG_KEYBOARD_GPIO_PE
	&rib_gpio_keys_device,
#endif
	&msm_device_adspdec,
	&qup_device_i2c,

	&msm_device_kgsl,
	&msm_device_vidc_720p,
#if defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
    || defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE)
	&rib_cy8ctma300_device,
	&msm_device_uart_dm2,
	&ctp_uart_device,
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA300[_MODULE] */
#ifdef CONFIG_NDUID
	&nduid_device,
#endif
#ifdef CONFIG_WEBCAM_OV7739
	&msm_camera_sensor_ov7739,
#endif
#ifdef CONFIG_MT9P013
	&msm_camera_sensor_mt9p013,
#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#ifdef CONFIG_SENSORS_MSM_ADC
	&msm_adc_device,
#endif
};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}

static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 0,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
	.power_timeout_ms =  I2C_2_POWER_TIMEOUT_MS,
	.msm_i2c_clock_set_state = camera_set_vreg_state,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 400000,
	.pclk = "camif_pad_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
}

#ifdef CONFIG_I2C_SSBI
static struct msm_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
};

static struct msm_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static struct msm_acpu_clock_platform_data msm7x30_clock_data = {
	.acpu_switch_time_us = 50,
	.vdd_switch_time_us = 62,
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct mmc_host *wifi_mmc;

static void rib_probe_wifi(int id, struct mmc_host *mmc)
{
    printk("%s: id %d mmc %p\n", __PRETTY_FUNCTION__, id, mmc);
    wifi_mmc = mmc;
}

static void rib_remove_wifi(int id, struct mmc_host *mmc)
{
    printk("%s: id %d mmc %p\n", __PRETTY_FUNCTION__, id, mmc);
    wifi_mmc = NULL;
}

#define SMPS_WIFI_ID "WIFI"

/*
 *  An API to enable wifi
 */
int board_sdio_wifi_enable(unsigned int param)
{
	int rc;

	// keep S4 regulator in PWM mode when wifi is on
	rc = pmapp_smps_mode_vote(SMPS_WIFI_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_PWM);
	if (rc != 0) {
		printk("pmapp_smps_mode_vote error %d\n", rc);
		return rc;
	}

    printk(KERN_ERR "board_sdio_wifi_enable\n");
    pinmux_config("WIFI_RESET", PINMUX_CONFIG_ACTIVE);
    pinmux_config("WIFI_EN", PINMUX_CONFIG_ACTIVE);
    pinmux_config("WIFI_INT", PINMUX_CONFIG_ACTIVE);

    gpio_set_value(GPIO_WIFI_RESET, 1);
    gpio_set_value(GPIO_WIFI_EN, 1);

    if (wifi_mmc) {
        mmc_detect_change(wifi_mmc, msecs_to_jiffies(250));
    }

    return 0;
}
EXPORT_SYMBOL(board_sdio_wifi_enable);

/*
 *  An API to disable wifi
 */
int board_sdio_wifi_disable(unsigned int param)
{
	int rc;

    printk(KERN_ERR "board_sdio_wifi_disable\n");

    //Put WiFi chip in RESET in order for the detection to fail and remove the card
    gpio_set_value(GPIO_WIFI_RESET, 0);
    gpio_set_value(GPIO_WIFI_EN, 0);

    pinmux_config("WIFI_RESET", PINMUX_CONFIG_SLEEP);
    pinmux_config("WIFI_EN", PINMUX_CONFIG_SLEEP);
    pinmux_config("WIFI_INT", PINMUX_CONFIG_SLEEP);
    if (wifi_mmc) {
        mmc_detect_change(wifi_mmc, msecs_to_jiffies(100));
    }

	// allow S4 to enter PFM mode when Wifi is off.
	rc = pmapp_smps_mode_vote(SMPS_WIFI_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_DONTCARE);
	if (rc != 0) {
		printk("pmapp_smps_mode_vote error %d\n", rc);
		return rc;
	}

    return 0;
}
EXPORT_SYMBOL(board_sdio_wifi_disable);


static unsigned long vreg_sts = 0, gpio_sts = 0;

#define MMC_DEV_ID_MMC  2
#define MMC_DEV_ID_WIFI 3

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{

	if (!(test_bit(dev_id, &gpio_sts)^enable))
                return;

        if (enable)
                set_bit(dev_id, &gpio_sts);
        else
                clear_bit(dev_id, &gpio_sts);

        switch(dev_id)
        {
                // wifi
                case MMC_DEV_ID_WIFI:
                {
                        pinmux_config("WL_SD_CLK", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("WL_SD_CMD", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("WL_SD_DATA3", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("WL_SD_DATA2", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("WL_SD_DATA1", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("WL_SD_DATA0", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        break;
                }
		// mmc
                case MMC_DEV_ID_MMC:
                {
                        pinmux_config("MMC_SD_CLK", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_CMD", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_DATA3", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_DATA2", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_DATA1", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_DATA0", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
                        pinmux_config("MMC_SD_DATA4", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_DATA5", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_DATA6", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
                        pinmux_config("MMC_SD_DATA7", enable ? PINMUX_CONFIG_ACTIVE : PINMUX_CONFIG_SLEEP);
#endif
                        break;
                }
                default:
                        break;
        }
}

#define MMC_DEV_IS_VREG_ON(devid)  (vreg_sts & (1 << devid ))
#define MMC_DEV_IS_VREG_OFF(devid) !MMC_DEV_IS_VREG_ON(devid)

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
        struct platform_device *pdev;
        int dev_id = 0;

        pdev = container_of(dv, struct platform_device, dev);
        dev_id      = pdev->id;

        if (vdd == 0) {

                msm_sdcc_setup_gpio(dev_id, !!vdd);

                if (!vreg_sts)
                        return 0;

                if (dev_id == MMC_DEV_ID_MMC && MMC_DEV_IS_VREG_ON(dev_id)) {
                                pinmux_config("MMC_PWR_EVT2", PINMUX_CONFIG_SLEEP);
                }


                clear_bit(dev_id, &vreg_sts);

                return 0;
        }
	else
        {
                if (dev_id == MMC_DEV_ID_MMC && MMC_DEV_IS_VREG_OFF(dev_id)) {
                                pinmux_config("MMC_PWR_EVT2", PINMUX_CONFIG_ACTIVE);
                }

                set_bit(dev_id, &vreg_sts);

                msm_sdcc_setup_gpio(dev_id, !!vdd);
        }


        return 0;
}

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x30_sdc2_mmc_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x30_sdc3_wifi_data = {
	.ocr_mask	= MMC_VDD_20_21,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
	.board_probe = rib_probe_wifi,
    .board_remove = rib_remove_wifi
};
#endif

static void __init msm7x30_init_mmc(void)
{
	// MMC
	msm_add_sdcc(2, &msm7x30_sdc2_mmc_data);

	// Wifi
	msm_add_sdcc(3, &msm7x30_sdc3_wifi_data);

}

#ifdef CONFIG_CHARGER_SMB339
static struct smb339_platform_data board_smb339_data = {
	.output_current = SMB339_CURRENT_950mA,

};

static struct i2c_board_info smb339_i2c_board_info = {
	I2C_BOARD_INFO(SMB339_I2C_DEVICE, SMB339_I2C_ADDR),
	.platform_data = &board_smb339_data,
	.irq = 0,
};
#endif // CONFIG_CHARGER_SMB339


static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

void msm_board_mux_sleep(void)
{
        pinmux_config("UART3_RX", PINMUX_CONFIG_SLEEP);
        pinmux_config("UART3_TX", PINMUX_CONFIG_SLEEP);
        pinmux_config("UART3_RTS", PINMUX_CONFIG_SLEEP);
        pinmux_config("UART3_CTS", PINMUX_CONFIG_SLEEP);


#ifdef CONFIG_KEYBOARD_GPIO_PE
        pinmux_config("SLIDER_OPEN", PINMUX_CONFIG_SLEEP);
        pinmux_config("SLIDER_CLOSE", PINMUX_CONFIG_SLEEP);
        pinmux_config("RINGER", PINMUX_CONFIG_SLEEP);
        pinmux_config("VOL_UP", PINMUX_CONFIG_SLEEP);
        pinmux_config("VOL_DOWN", PINMUX_CONFIG_SLEEP);
#endif

        // I2C
        pinmux_config("I2C_SCL", PINMUX_CONFIG_SLEEP);
        pinmux_config("I2C_SDA", PINMUX_CONFIG_SLEEP);

		pinmux_config("PROX_INT", PINMUX_CONFIG_SLEEP);
		pinmux_config("MOTION_INT2", PINMUX_CONFIG_SLEEP);

        pinmux_config("MOTION_INT1", PINMUX_CONFIG_SLEEP);

		pinmux_config("CTP_RESET", PINMUX_CONFIG_SLEEP);
		pinmux_config("CTP_RX", PINMUX_CONFIG_SLEEP);
		pinmux_config("CTP_SHDN", PINMUX_CONFIG_SLEEP);
		pinmux_config("CTP_WAKE", PINMUX_CONFIG_SLEEP);
		pinmux_config("CY8CTMA300_SCLK", PINMUX_CONFIG_SLEEP);


        // A6 I2C
        pinmux_config("I2C_SCL_CHG", PINMUX_CONFIG_SLEEP);
        pinmux_config("I2C_SDA_CHG", PINMUX_CONFIG_SLEEP);

        pinmux_config("LDO1", PINMUX_CONFIG_SLEEP);
        pinmux_config("LDO2", PINMUX_CONFIG_SLEEP);
}

void msm_board_mux_wake(void)
{
        pinmux_config("UART3_RX", PINMUX_CONFIG_ACTIVE);
        pinmux_config("UART3_TX", PINMUX_CONFIG_ACTIVE);
        pinmux_config("UART3_RTS", PINMUX_CONFIG_ACTIVE);
        pinmux_config("UART3_CTS", PINMUX_CONFIG_ACTIVE);


#ifdef CONFIG_KEYBOARD_GPIO_PE
        pinmux_config("SLIDER_OPEN", PINMUX_CONFIG_ACTIVE);
        pinmux_config("SLIDER_CLOSE", PINMUX_CONFIG_ACTIVE);
        pinmux_config("RINGER", PINMUX_CONFIG_ACTIVE);
        pinmux_config("VOL_UP", PINMUX_CONFIG_ACTIVE);
        pinmux_config("VOL_DOWN", PINMUX_CONFIG_ACTIVE);
#endif
		// I2C
		pinmux_config("I2C_SCL", PINMUX_CONFIG_ACTIVE);
		pinmux_config("I2C_SDA", PINMUX_CONFIG_ACTIVE);


		pinmux_config("PROX_INT", PINMUX_CONFIG_ACTIVE);
		pinmux_config("MOTION_INT2", PINMUX_CONFIG_ACTIVE);

        pinmux_config("MOTION_INT1", PINMUX_CONFIG_ACTIVE);

		pinmux_config("CTP_RESET", PINMUX_CONFIG_ACTIVE);
		pinmux_config("CTP_SHDN", PINMUX_CONFIG_ACTIVE);
		pinmux_config("CTP_RX", PINMUX_CONFIG_ACTIVE);
		pinmux_config("CTP_WAKE", PINMUX_CONFIG_ACTIVE);
		pinmux_config("CY8CTMA300_SCLK", PINMUX_CONFIG_ACTIVE);

        // A6 I2C
		pinmux_config("I2C_SCL_CHG", PINMUX_CONFIG_ACTIVE);
		pinmux_config("I2C_SDA_CHG", PINMUX_CONFIG_ACTIVE);

        pinmux_config("LDO1", PINMUX_CONFIG_ACTIVE);
        pinmux_config("LDO2", PINMUX_CONFIG_ACTIVE);
}

static void msm_board_mux_init(void)
{
		wlan_reset_pin_control_init();

        pinmux_config("UART1DM_RTS", PINMUX_CONFIG_SLEEP);
        pinmux_config("UART1DM_CTS", PINMUX_CONFIG_SLEEP);
        pinmux_config("UART1DM_RX", PINMUX_CONFIG_SLEEP);
        pinmux_config("UART1DM_TX", PINMUX_CONFIG_SLEEP);

        pinmux_config("UART3_RX", PINMUX_CONFIG_ACTIVE);
        pinmux_config("UART3_TX", PINMUX_CONFIG_ACTIVE);
        pinmux_config("UART3_CTS", PINMUX_CONFIG_ACTIVE);
        pinmux_config("UART3_RTS", PINMUX_CONFIG_ACTIVE);

        pinmux_config("BT_RESET", PINMUX_CONFIG_ACTIVE);                // bluetooth
        pinmux_config("BT_BT_WAKE_HOST", PINMUX_CONFIG_ACTIVE);

        pinmux_config("WIFI_INT", PINMUX_CONFIG_SLEEP);         // wifi sdio
        pinmux_config("WIFI_RESET", PINMUX_CONFIG_SLEEP);
        pinmux_config("WIFI_EN", PINMUX_CONFIG_SLEEP);
        pinmux_config("WL_SD_CLK", PINMUX_CONFIG_SLEEP);
        pinmux_config("WL_SD_CMD", PINMUX_CONFIG_SLEEP);
        pinmux_config("WL_SD_DATA3", PINMUX_CONFIG_SLEEP);
        pinmux_config("WL_SD_DATA2", PINMUX_CONFIG_SLEEP);
        pinmux_config("WL_SD_DATA1", PINMUX_CONFIG_SLEEP);
        pinmux_config("WL_SD_DATA0", PINMUX_CONFIG_SLEEP);
#ifdef CONFIG_KEYBOARD_GPIO_PE
        pinmux_config("SLIDER_OPEN", PINMUX_CONFIG_ACTIVE);
        pinmux_config("SLIDER_CLOSE", PINMUX_CONFIG_ACTIVE);
        pinmux_config("RINGER", PINMUX_CONFIG_ACTIVE);
        pinmux_config("VOL_UP", PINMUX_CONFIG_ACTIVE);
        pinmux_config("VOL_DOWN", PINMUX_CONFIG_ACTIVE);
        pinmux_config("POWER", PINMUX_CONFIG_ACTIVE);
#endif


#ifdef CONFIG_LEDS_LM8502
        pinmux_config("LM8502_EN", PINMUX_CONFIG_ACTIVE);
        pinmux_config("LM8502_INT", PINMUX_CONFIG_ACTIVE);
#endif
		pinmux_config("CTP_RESET", PINMUX_CONFIG_ACTIVE);
		pinmux_config("CTP_RX", PINMUX_CONFIG_ACTIVE);
		pinmux_config("CTP_WAKE", PINMUX_CONFIG_ACTIVE);
		pinmux_config("CY8CTMA300_SCLK", PINMUX_CONFIG_ACTIVE);


        // I2C
        pinmux_config("I2C_SCL", PINMUX_CONFIG_ACTIVE);
        pinmux_config("I2C_SDA", PINMUX_CONFIG_ACTIVE);

	pinmux_config("PROX_INT", PINMUX_CONFIG_ACTIVE);
	pinmux_config("MOTION_INT2", PINMUX_CONFIG_ACTIVE);

 	pinmux_config("MOTION_INT1", PINMUX_CONFIG_ACTIVE);
        pinmux_config("CTP_SHDN", PINMUX_CONFIG_ACTIVE);

        // A6 I2C
        pinmux_config("I2C_SCL_CHG", PINMUX_CONFIG_ACTIVE);
        pinmux_config("I2C_SDA_CHG", PINMUX_CONFIG_ACTIVE);

        pinmux_config("LDO1", PINMUX_CONFIG_ACTIVE);
        pinmux_config("LDO2", PINMUX_CONFIG_ACTIVE);
};



static void __init msm7x30_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);

	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
	msm_spm_init(&msm_spm_data, 1);
	msm_acpu_clock_init(&msm7x30_clock_data);

	board_rib_gpios_init();

	msm_board_mux_init();

#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

#if CONFIG_SENSORS_MSM_ADC
	msm_adc_pdata.dev_names = msm_adc_rib_device_names;
	msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_rib_device_names);
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_USB_EHCI_MSM
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	rmt_storage_add_ramfs();
	msm7x30_init_mmc();

	//msm_qsd_spi_init();

#ifdef CONFIG_MSM_CAMERA
	CDBG("i2c_register_board_info(2)\n");
	i2c_register_board_info(	2,
					msm_camera_boardinfo,
					ARRAY_SIZE(msm_camera_boardinfo));
	CDBG("i2c_register_board_info(2) - done\n");
#endif

	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	buses_init();
#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	msm_snddev_init();
	aux_pcm_gpio_init();
#endif

#ifdef CONFIG_MFD_WM8994
    wm8994_init();
#endif /* CONFIG_MFD_WM8994 */

#ifdef CONFIG_LEDS_LM3528
	i2c_register_board_info(4, &lm3528_i2c_board_info, 1);
#endif

#ifdef CONFIG_LEDS_LM8502
	i2c_register_board_info(4, &lm8502_board_info, 1);
#endif


#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif

#ifdef CONFIG_CHARGER_SMB339
	i2c_register_board_info(0, &smb339_i2c_board_info, 1);
#endif

#ifdef CONFIG_A6
    rib_init_a6();
#endif // CONFIG_A6

#ifdef CONFIG_MSM_CAMERA
	rib_init_camera();
#endif

}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB0_SIZE;
static void __init fb_size_setup(char **p)
{
	fb_size = memparse(*p, p);
}
__early_param("fb_size=", fb_size_setup);

static unsigned pmem_gpu1_size = MSM_PMEM_GPU1_SIZE;
static void __init pmem_gpu1_size_setup(char **p)
{
    pmem_gpu1_size = memparse(*p, p);
}
__early_param("pmem_gpu1_size=", pmem_gpu1_size_setup);

static unsigned gpu_phys_size = MSM_GPU_PHYS_SIZE;
static void __init gpu_phys_size_setup(char **p)
{
	gpu_phys_size = memparse(*p, p);
}
__early_param("gpu_phys_size=", gpu_phys_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static unsigned fluid_pmem_adsp_size = MSM_FLUID_PMEM_ADSP_SIZE;
static void __init fluid_pmem_adsp_size_setup(char **p)
{
	fluid_pmem_adsp_size = memparse(*p, p);
}
__early_param("fluid_pmem_adsp_size=", fluid_pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static void __init pmem_audio_size_setup(char **p)
{
	pmem_audio_size = memparse(*p, p);
}
__early_param("pmem_audio_size=", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;
/*
   Request allocation of Hardware accessible PMEM regions
   at the beginning to make sure they are allocated in EBI-0.
   This will allow 7x30 with two mem banks enter the second
   mem bank into Self-Refresh State during Idle Power Collapse.

    The current HW accessible PMEM regions are
    1. Frame Buffer.
       LCDC HW can access msm_fb_resources during Idle-PC.

    2. Audio
       LPA HW can access android_pmem_audio_pdata during Idle-PC.
*/
	size = MSM_FB0_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = MSM_FB1_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[1].start = __pa(addr);
	msm_fb_resources[1].end = msm_fb_resources[1].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb1\n",
										        size, addr, __pa(addr));

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
	}


#if defined(CONFIG_MSM_KGSL)

#if !defined(CONFIG_MSM_KGSL_MMU)
    size = pmem_gpu1_size;
    if (size) {
        addr = alloc_bootmem(size);
        android_pmem_gpu1_pdata.start = __pa(addr);
        android_pmem_gpu1_pdata.size = size;
        pr_info("allocating %lu bytes at %p (%lx physical) for gpu1 "
            "pmem arena\n", size, addr, __pa(addr));
    }
#endif

    size = gpu_phys_size;
    if (size) {
        addr = alloc_bootmem(size);
        kgsl_resources[1].start = __pa(addr);
        kgsl_resources[1].end = kgsl_resources[1].start + size - 1;
        pr_info("allocating %lu bytes at %p (%lx physical) for "
            "KGSL\n", size, addr, __pa(addr));
    }
#endif


	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}
}

static void __init msm7x30_map_io(void)
{
	//msm_shared_ram_phys = 0x00100000;
	msm_shared_ram_phys = 0x0FF00000;
	msm_map_msm7x30_io();
	msm7x30_allocate_memory_regions();
}

static void	__init rib_fixup(struct machine_desc *m, struct tag *t,
char **c, struct meminfo *mi)
{
	/* XXX total hack to work around problem with old bootloaders that report the first
	 * 2MB of the first bank as non existent.
	 */
	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_MEM) {
			if (t->u.mem.start == 0x00200000) {
				t->u.mem.start = 0;
				t->u.mem.size += 0x00200000;
			}
		}
	}
}

MACHINE_START(RIB, "Rib")
#ifdef CONFIG_MSM_DEBUG_UART
        .phys_io  = MSM_DEBUG_UART_PHYS,
        .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
        .boot_params = 0x00200100,
	.fixup = rib_fixup,
        .map_io = msm7x30_map_io,
        .init_irq = msm7x30_init_irq,
        .init_machine = msm7x30_init,
        .timer = &msm_timer,
MACHINE_END

